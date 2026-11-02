#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        self.scan = None
        self.scan_time = None
        self.heading = None
        self.last_choice = None
        self.last_control = None
        self.linear_speed = 0.0
        self.trail = []
        self.traveled = 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_subscription = self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Исследователь: лидар + память пути; скорость до 0.35 м/с')

    def on_scan(self, msg):
        self.scan = msg
        self.scan_time = self.get_clock().now()

    def control_loop(self):
        now = self.get_clock().now()
        if self.scan is None or (now - self.scan_time).nanoseconds > 500_000_000:
            self.drive(0.0, 0.0)
            return
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException:
            self.drive(0.0, 0.0)
            return
        p = transform.transform.translation
        q = transform.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if not self.trail or math.hypot(p.x - self.trail[-1][0], p.y - self.trail[-1][1]) >= 0.2:
            if self.trail:
                self.traveled += math.hypot(p.x - self.trail[-1][0], p.y - self.trail[-1][1])
            self.trail.append((p.x, p.y, self.traveled))

        blocked = self.heading is None or self.clearance(self.heading - yaw) < 0.8
        if blocked or self.last_choice is None or (now - self.last_choice).nanoseconds >= 1_000_000_000:
            self.choose_direction(p.x, p.y, yaw, blocked)
            self.last_choice = now
        if self.heading is None:
            self.drive(0.0, 0.0)
            return

        error = math.atan2(math.sin(self.heading - yaw), math.cos(self.heading - yaw))
        front = self.clearance(0.0)
        desired = 0.35 * max(0.0, min(1.0, (front - 0.1) / 0.6)) * max(0.0, math.cos(error))
        dt = min(0.1, (now - self.last_control).nanoseconds / 1e9) if self.last_control else 0.05
        self.last_control = now
        speed = max(self.linear_speed - 0.8 * dt, min(self.linear_speed + 0.4 * dt, desired))
        if front < 0.1 or abs(error) > math.pi / 2:
            speed = 0.0
        self.drive(speed, max(-1.2, min(1.2, 1.8 * error)))

    def choose_direction(self, x, y, yaw, blocked):
        options = []
        for step in range(24):
            heading = step * math.pi / 12
            space = self.clearance(heading - yaw)
            if space >= 1.15:
                options.append((self.direction_score(x, y, yaw, heading, space), heading))
        if not options:
            options = [
                (self.clearance(step * math.pi / 12 - yaw), step * math.pi / 12)
                for step in range(24)
            ]
        score, heading = max(options, key=lambda item: item[0])
        if not blocked:
            current = self.direction_score(x, y, yaw, self.heading, self.clearance(self.heading - yaw))
            if score < current + 0.25:
                return
        changed = self.heading is None or abs(math.atan2(
            math.sin(heading - self.heading), math.cos(heading - self.heading))) > 0.2
        self.heading = heading
        if changed:
            self.get_logger().info(
                f'Направление: {math.degrees(heading):.0f}°; точек в памяти: {len(self.trail)}')

    def direction_score(self, x, y, yaw, heading, space):
        target_x = x + math.cos(heading)
        target_y = y + math.sin(heading)
        novelty = min((
            math.hypot(target_x - px, target_y - py)
            for px, py, distance in self.trail if distance <= self.traveled - 1.5
        ), default=2.0)
        turn = abs(math.atan2(math.sin(heading - yaw), math.cos(heading - yaw)))
        return min(novelty, 2.0) + 0.15 * min(space, 2.0) - 0.4 * turn

    def clearance(self, heading):
        best = self.scan.range_max
        valid = False
        for i, distance in enumerate(self.scan.ranges):
            if math.isnan(distance) or distance < self.scan.range_min:
                continue
            valid = True
            distance = min(distance, self.scan.range_max)
            angle = self.scan.angle_min + i * self.scan.angle_increment - heading
            forward = distance * math.cos(angle)
            side = abs(distance * math.sin(angle))
            # Проверяем полосу движения с запасом относительно радиуса робота 0.175 м.
            if forward > 0 and side < 0.27:
                best = min(best, max(0.0, forward - math.sqrt(0.27 ** 2 - side ** 2)))
        return best if valid else 0.0

    def drive(self, speed, turn):
        msg = Twist()
        msg.linear.x = float(speed)
        msg.angular.z = float(turn)
        self.cmd_pub.publish(msg)
        self.linear_speed = speed


def main():
    rclpy.init()
    node = Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.drive(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
