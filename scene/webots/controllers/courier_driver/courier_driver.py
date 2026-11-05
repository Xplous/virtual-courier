"""Драйвер робота Courier: мост Webots <-> ROS 2.

Публикует: /clock, /scan, /odom, /tf, /tf_static, /imu,
           /camera/image_raw, /camera/camera_info, /courier/tray_state
Слушает:   /cmd_vel, /courier/tray_cmd
"""

import array
import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, qos_profile_sensor_data
from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu, Image, CameraInfo
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage

from controller import Robot

WHEEL_RADIUS = 0.035
WHEEL_BASE = 0.23
MAX_LIN = 0.7          # м/с
MAX_ANG = 3.0          # рад/с
MAX_LIN_ACC = 1.0      # м/с²
MAX_ANG_ACC = 5.0      # рад/с²
CMD_TIMEOUT = 0.5      # с без /cmd_vel → стоп
TRAY_MAX = 1.05        # рад (~60°)
ENCODER_NOISE = 0.01   # относительный шум шага энкодера

# Периоды должны быть кратны basicTimeStep мира (20 мс).
LIDAR_PERIOD = 100     # мс (10 Гц)
CAMERA_PERIOD = 100    # мс (10 Гц)
IMU_PERIOD = 20        # мс (50 Гц)
ODOM_PERIOD = 20       # мс (50 Гц)

LASER_POSE = (0.09, 0.0, 0.175)
CAMERA_POSE = (0.178, 0.0, 0.07)


def stamp(t):
    msg = Time()
    msg.sec = int(t)
    msg.nanosec = int(round((t - int(t)) * 1e9)) % 1000000000
    return msg


def yaw_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def rpy_quat(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def make_tf(parent, child, xyz, q, t):
    tf = TransformStamped()
    tf.header.stamp = stamp(t)
    tf.header.frame_id = parent
    tf.child_frame_id = child
    tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = xyz
    (tf.transform.rotation.x, tf.transform.rotation.y,
     tf.transform.rotation.z, tf.transform.rotation.w) = q
    return tf


class CourierDriver(Node):
    def __init__(self, robot):
        super().__init__('courier_driver')
        self.robot = robot
        self.dt_ms = int(robot.getBasicTimeStep())

        self.left = robot.getDevice('left_wheel_motor')
        self.right = robot.getDevice('right_wheel_motor')
        for m in (self.left, self.right):
            m.setPosition(float('inf'))
            m.setVelocity(0.0)
        self.left_enc = robot.getDevice('left_wheel_sensor')
        self.right_enc = robot.getDevice('right_wheel_sensor')
        self.left_enc.enable(self.dt_ms)
        self.right_enc.enable(self.dt_ms)

        self.tray = robot.getDevice('tray_motor')
        self.tray.setVelocity(1.0)
        self.tray.setPosition(0.0)
        self.tray_sensor = robot.getDevice('tray_sensor')
        self.tray_sensor.enable(self.dt_ms)

        self.lidar = robot.getDevice('lidar')
        self.lidar.enable(LIDAR_PERIOD)
        self.camera = robot.getDevice('camera')
        self.camera.enable(CAMERA_PERIOD)
        self.imu = robot.getDevice('imu')
        self.imu.enable(IMU_PERIOD)
        self.gyro = robot.getDevice('gyro')
        self.gyro.enable(IMU_PERIOD)
        self.accel = robot.getDevice('accelerometer')
        self.accel.enable(IMU_PERIOD)

        sensor_qos = qos_profile_sensor_data
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', sensor_qos)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 10)
        self.tf_static_pub = self.create_publisher(TFMessage, '/tf_static', latched)
        self.imu_pub = self.create_publisher(Imu, '/imu', sensor_qos)
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', sensor_qos)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', sensor_qos)
        self.tray_pub = self.create_publisher(Float64, '/courier/tray_state', 10)

        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(Float64, '/courier/tray_cmd', self.on_tray_cmd, 10)

        self.cmd = (0.0, 0.0)
        self.cmd_time = -1e9
        self.v = 0.0
        self.w = 0.0
        self.x = self.y = self.th = 0.0
        self.prev_enc = None

        self.publish_static_tf()

    # ---------------------------------------------------------------- входы

    def on_cmd_vel(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)
        self.cmd_time = self.robot.getTime()

    def on_tray_cmd(self, msg):
        angle = max(0.0, min(TRAY_MAX, float(msg.data)))
        self.tray.setPosition(angle)

    # ---------------------------------------------------------------- шаг

    def step(self):
        t = self.robot.getTime()
        ms = int(round(t * 1000))
        dt = self.dt_ms / 1000.0

        clk = Clock()
        clk.clock = stamp(t)
        self.clock_pub.publish(clk)

        self.update_motors(t, dt)
        self.update_odometry(t)

        if ms > 0 and ms % LIDAR_PERIOD == 0:
            self.publish_scan(t)
        if ms > 0 and ms % CAMERA_PERIOD == 0:
            self.publish_camera(t)
        if ms > 0 and ms % IMU_PERIOD == 0:
            self.publish_imu(t)
            self.tray_pub.publish(Float64(data=float(self.tray_sensor.getValue())))

    def update_motors(self, t, dt):
        tv, tw = self.cmd
        if t - self.cmd_time > CMD_TIMEOUT:
            tv, tw = 0.0, 0.0
        tv = max(-MAX_LIN, min(MAX_LIN, tv))
        tw = max(-MAX_ANG, min(MAX_ANG, tw))
        self.v += max(-MAX_LIN_ACC * dt, min(MAX_LIN_ACC * dt, tv - self.v))
        self.w += max(-MAX_ANG_ACC * dt, min(MAX_ANG_ACC * dt, tw - self.w))
        wl = (self.v - self.w * WHEEL_BASE / 2) / WHEEL_RADIUS
        wr = (self.v + self.w * WHEEL_BASE / 2) / WHEEL_RADIUS
        self.left.setVelocity(wl)
        self.right.setVelocity(wr)

    def update_odometry(self, t):
        enc = (self.left_enc.getValue(), self.right_enc.getValue())
        if any(math.isnan(e) for e in enc):
            return
        if self.prev_enc is None:
            self.prev_enc = enc
            return
        dl = (enc[0] - self.prev_enc[0]) * WHEEL_RADIUS
        dr = (enc[1] - self.prev_enc[1]) * WHEEL_RADIUS
        self.prev_enc = enc
        dl *= 1.0 + random.gauss(0.0, ENCODER_NOISE)
        dr *= 1.0 + random.gauss(0.0, ENCODER_NOISE)
        ds = (dl + dr) / 2
        dth = (dr - dl) / WHEEL_BASE
        self.x += ds * math.cos(self.th + dth / 2)
        self.y += ds * math.sin(self.th + dth / 2)
        self.th = math.atan2(math.sin(self.th + dth), math.cos(self.th + dth))

        if int(round(t * 1000)) % ODOM_PERIOD != 0:
            return
        q = yaw_quat(self.th)
        odom = Odometry()
        odom.header.stamp = stamp(t)
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
         odom.pose.pose.orientation.z, odom.pose.pose.orientation.w) = q
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        odom.pose.covariance[0] = odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.02
        odom.twist.covariance[0] = odom.twist.covariance[35] = 0.01
        self.odom_pub.publish(odom)
        self.tf_pub.publish(TFMessage(transforms=[make_tf('odom', 'base_link', (self.x, self.y, 0.0), q, t)]))

    def publish_scan(self, t):
        try:
            ranges = self.lidar.getRangeImage()
        except (ValueError, SystemError):
            return
        if not ranges:
            return
        n = len(ranges)
        msg = LaserScan()
        msg.header.stamp = stamp(t)
        msg.header.frame_id = 'laser'
        # Webots отдаёт лучи по часовой стрелке начиная с -π (сзади);
        # ROS ожидает против часовой — разворачиваем массив.
        msg.angle_min = -math.pi
        msg.angle_increment = 2 * math.pi / n
        msg.angle_max = msg.angle_min + msg.angle_increment * (n - 1)
        msg.time_increment = 0.0
        msg.scan_time = LIDAR_PERIOD / 1000.0
        msg.range_min = self.lidar.getMinRange()
        msg.range_max = self.lidar.getMaxRange()
        msg.ranges = [float(r) for r in reversed(ranges)]
        self.scan_pub.publish(msg)

    def publish_camera(self, t):
        try:
            data = self.camera.getImage()
        except (ValueError, SystemError):
            return
        if not data:
            return
        w, h = self.camera.getWidth(), self.camera.getHeight()
        img = Image()
        img.header.stamp = stamp(t)
        img.header.frame_id = 'camera_optical_frame'
        img.width, img.height = w, h
        img.encoding = 'bgra8'
        img.step = 4 * w
        # array('B') обходит поэлементную проверку типов rclpy (иначе ~50 мс на кадр).
        img.data = array.array('B', data)
        self.image_pub.publish(img)

        fx = (w / 2) / math.tan(self.camera.getFov() / 2)
        info = CameraInfo()
        info.header = img.header
        info.width, info.height = w, h
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [fx, 0.0, w / 2, 0.0, fx, h / 2, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, w / 2, 0.0, 0.0, fx, h / 2, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info)

    def publish_imu(self, t):
        q = self.imu.getQuaternion()
        g = self.gyro.getValues()
        a = self.accel.getValues()
        if any(math.isnan(v) for v in q):
            return
        msg = Imu()
        msg.header.stamp = stamp(t)
        msg.header.frame_id = 'imu_link'
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = g
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = a
        msg.orientation_covariance[0] = msg.orientation_covariance[4] = msg.orientation_covariance[8] = 1e-4
        msg.angular_velocity_covariance[0] = msg.angular_velocity_covariance[4] = \
            msg.angular_velocity_covariance[8] = 2.5e-5
        msg.linear_acceleration_covariance[0] = msg.linear_acceleration_covariance[4] = \
            msg.linear_acceleration_covariance[8] = 1e-4
        self.imu_pub.publish(msg)

    def publish_static_tf(self):
        t = self.robot.getTime()
        tfs = [
            make_tf('base_link', 'laser', LASER_POSE, (0.0, 0.0, 0.0, 1.0), t),
            make_tf('base_link', 'camera_link', CAMERA_POSE, (0.0, 0.0, 0.0, 1.0), t),
            make_tf('camera_link', 'camera_optical_frame', (0.0, 0.0, 0.0),
                    rpy_quat(-math.pi / 2, 0.0, -math.pi / 2), t),
            make_tf('base_link', 'imu_link', (0.0, 0.0, 0.05), (0.0, 0.0, 0.0, 1.0), t),
            make_tf('base_link', 'tray_link', (-0.07, 0.0, 0.092), (0.0, 0.0, 0.0, 1.0), t),
        ]
        self.tf_static_pub.publish(TFMessage(transforms=tfs))


def main():
    robot = Robot()
    rclpy.init()
    node = CourierDriver(robot)
    dt = int(robot.getBasicTimeStep())
    while robot.step(dt) != -1:
        rclpy.spin_once(node, timeout_sec=0)
        node.step()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
