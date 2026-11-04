#!/usr/bin/env python3
"""Узел курьера и доставки «Виртуальный курьер».

Отвечает за:
1. Обнаружение станций и зоны выгрузки по камере (ArUco + классификация состояния ламп).
2. Контроль груза (/courier/cargo_count).
3. Точный подъезд, загрузку и разгрузку (управление лотком /courier/tray_cmd).
"""

import json
import math
import sys
from pathlib import Path

# Подключение локальных модулей
script_dir = Path(__file__).resolve().parent
if str(script_dir) not in sys.path:
    sys.path.insert(0, str(script_dir))

from station_detector import StationDetector, StationState

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
from std_msgs.msg import Float64, String, UInt8

TRAY_DOWN = 0.0
TRAY_UNLOAD = 1.0  # рад — наклон лотка назад для выгрузки


class Courier(Node):
    def __init__(self):
        super().__init__('courier')
        self.declare_parameter('gui', True)
        self.gui = self.get_parameter('gui').value

        # ---- состояние и сенсоры
        self.scan = None
        self.pose = (0.0, 0.0, 0.0)     # x, y, yaw в системе odom
        self.speed = (0.0, 0.0)         # v, w
        self.imu = None
        self.image = None               # BGR numpy-массив
        self.cargo_count = 0
        self.tray_angle = 0.0
        self.bridge = CvBridge()

        # ---- модуль компьютерного зрения
        self.detector = StationDetector()

        # Реестр обнаруженных станций {marker_id: StationDetection}
        self.known_stations = {}

        # ---- подписки
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_camera_info, qos_profile_sensor_data)
        self.create_subscription(UInt8, '/courier/cargo_count', self.on_cargo, 10)
        self.create_subscription(Float64, '/courier/tray_state', self.on_tray, 10)

        # ---- публикаторы
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tray_pub = self.create_publisher(Float64, '/courier/tray_cmd', 10)
        self.stations_status_pub = self.create_publisher(String, '/delivery/stations_status', 10)

        self.state = 'WAIT_SENSORS'
        self.timer = self.create_timer(0.1, self.control_loop)   # 10 Гц
        self.get_logger().info('Courier (модуль доставки) запущен, ожидание сенсоров…')

    # ================================================================ колбэки

    def on_scan(self, msg):
        self.scan = msg

    def on_odom(self, msg):
        p = msg.pose.pose
        yaw = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
        self.pose = (p.position.x, p.position.y, yaw)
        self.speed = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def on_imu(self, msg):
        self.imu = msg

    def on_camera_info(self, msg: CameraInfo):
        self.detector.update_camera_info(list(msg.k))

    def on_image(self, msg: Image):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def on_cargo(self, msg: UInt8):
        if msg.data != self.cargo_count:
            self.get_logger().info(f'Грузов на лотке: {msg.data} (было {self.cargo_count})')
        self.cargo_count = msg.data

    def on_tray(self, msg: Float64):
        self.tray_angle = msg.data

    # ================================================================ управление

    def drive(self, v, w):
        """Задать линейную (м/с) и угловую (рад/с) скорость."""
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

    def stop(self):
        self.drive(0.0, 0.0)

    def set_tray(self, angle):
        """Наклонить лоток: 0 — горизонтально, ~1.0 рад — выгрузка назад."""
        self.tray_pub.publish(Float64(data=float(angle)))

    # ================================================================ лидар

    def sector_min(self, from_deg, to_deg):
        """Минимальная дальность лидара в секторе углов [from, to] (0° — вперёд, + — влево)."""
        s = self.scan
        if s is None:
            return float('inf')
        best = float('inf')
        n = len(s.ranges)
        for deg in range(int(from_deg), int(to_deg) + 1):
            i = int(round((math.radians(deg) - s.angle_min) / s.angle_increment)) % n
            r = s.ranges[i]
            if s.range_min <= r < best:
                best = r
        return best

    # ================================================================ основной цикл

    def control_loop(self):
        if self.state == 'WAIT_SENSORS':
            if self.scan is not None and self.image is not None:
                self.get_logger().info('Датчики готовы — старт работы модуля доставки.')
                self.set_tray(TRAY_DOWN)
                self.state = 'EXPLORE'
            return

        # 1. Компьютерное зрение: детекция станций и ламп
        detections = self.detector.detect(self.image)
        status_dict = {}

        for det in detections:
            # Обновляем реестр, если статус определился или это первое обнаружение
            prev = self.known_stations.get(det.marker_id)
            if prev is None or (det.state != StationState.UNKNOWN and prev.state == StationState.UNKNOWN):
                self.known_stations[det.marker_id] = det
            elif det.state != StationState.UNKNOWN:
                self.known_stations[det.marker_id] = det

            self.get_logger().info(
                f'Маркер {det.marker_id} ({det.name}): {det.state.value} | '
                f'дист: {det.distance:.2f}м, conf: {det.confidence:.2f}',
                throttle_duration_sec=2.0
            )

        # Публикуем текущее знание обо всех обнаруженных станциях
        for mid, det in self.known_stations.items():
            status_dict[det.name] = {
                'id': mid,
                'state': det.state.value,
                'distance': round(float(det.distance), 2),
            }
        if status_dict:
            self.stations_status_pub.publish(String(data=json.dumps(status_dict, ensure_ascii=False)))

        # 2. Движение: временный базовый объезд препятствий (до интеграции с Nav2/explorer)
        if self.state == 'EXPLORE':
            front = self.sector_min(-25, 25)
            left = self.sector_min(30, 90)
            right = self.sector_min(-90, -30)
            if front < 0.45:
                self.drive(0.0, 1.2 if left > right else -1.2)
            else:
                steer = max(-0.8, min(0.8, 0.8 * (min(left, 2.0) - min(right, 2.0))))
                self.drive(0.35, steer)

        # 3. Визуализация
        if self.image is not None:
            view = self.detector.draw_detections(self.image, detections)
            cv2.putText(
                view,
                f'state={self.state} cargo={self.cargo_count}',
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )
            # Сохраняем текущий кадр на диск — его можно открыть прямо в VS Code
            cv2.imwrite('/ros_ws/src/solution/camera_view.jpg', view)

            if self.gui:
                cv2.imshow('courier camera', view)
                cv2.waitKey(1)


def main():
    rclpy.init()
    node = Courier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
