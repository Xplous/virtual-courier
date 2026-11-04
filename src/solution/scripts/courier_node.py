#!/usr/bin/env python3
"""Узел курьера и доставки «Виртуальный курьер».

Отвечает за:
1. Детекцию станций и зоны выгрузки по камере (StationDetector: ArUco + сигнальные лампы).
2. Прецизионную PBVS-стыковку к станциям и зоне выгрузки (DockingController).
3. Приемку груза с контролем времени непрерывного покоя Dwell Time (CargoLoader).
4. Выгрузку кубиков в зону у Старта с опрокидыванием кузова (UnloadController).
5. Координацию состояний с глобальным навигатором через /delivery/state.
"""

import json
import math
import sys
from pathlib import Path

# Подключение локальных модулей решения
script_dir = Path(__file__).resolve().parent
if str(script_dir) not in sys.path:
    sys.path.insert(0, str(script_dir))

from station_detector import StationDetector, StationState, StationDetection
from docking_controller import DockingConfig, compute_docking_cmd
from cargo_loader import CargoLoader, LoaderConfig, LoadingStatus
from unload_controller import UnloadController

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

        # ---- компоненты подсистемы доставки
        self.detector = StationDetector()
        self.dock_config = DockingConfig(target_dist=0.65, min_safe_laser_dist=0.28)
        self.cargo_loader = CargoLoader(config=LoaderConfig(dwell_target=2.0))
        self.unload_ctrl = UnloadController(target_rear_dist=0.35, min_safe_rear_dist=0.29)

        # Реестр обнаруженных станций {marker_id: StationDetection}
        self.known_stations = {}
        self.target_station_id = None
        self.state = 'WAIT_SENSORS'

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
        self.state_pub = self.create_publisher(String, '/delivery/state', 10)
        self.stations_status_pub = self.create_publisher(String, '/delivery/stations_status', 10)

        self.timer = self.create_timer(0.1, self.control_loop)   # 10 Гц
        self.get_logger().info('Courier (модуль доставки) запущен, ожидание сенсоров…')

    # ================================================================ колбэки сенсоров

    def on_scan(self, msg: LaserScan):
        self.scan = msg

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose
        yaw = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
        self.pose = (p.position.x, p.position.y, yaw)
        self.speed = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def on_imu(self, msg: Imu):
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

    # ================================================================ управление актуаторами

    def drive(self, v: float, w: float):
        """Задать линейную (м/с) и угловую (рад/с) скорость."""
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

    def stop(self):
        self.drive(0.0, 0.0)

    def set_tray(self, angle: float):
        """Наклонить лоток: 0 — горизонтально, ~1.0 рад — выгрузка назад."""
        self.tray_pub.publish(Float64(data=float(angle)))

    def now_sec(self) -> float:
        """Текущее симуляционное время в секундах."""
        return self.get_clock().now().nanoseconds / 1e9

    # ================================================================ лидар

    def sector_min(self, from_deg: float, to_deg: float) -> float:
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

    # ================================================================ конечный автомат (FSM)

    def control_loop(self):
        if self.state == 'WAIT_SENSORS':
            if self.scan is not None and self.image is not None:
                self.get_logger().info('Все датчики готовы — старт миссии доставки!')
                self.set_tray(TRAY_DOWN)
                self.state = 'EXPLORE'
            return

        t = self.now_sec()
        yaw = self.pose[2]
        front_laser = self.sector_min(-15, 15)
        rear_laser = self.sector_min(160, 200)

        # 1. Компьютерное зрение: детекция станций, ламп и зоны выгрузки
        detections = self.detector.detect(self.image)
        status_dict = {}

        for det in detections:
            prev = self.known_stations.get(det.marker_id)
            if prev is None or (det.state != StationState.UNKNOWN and prev.state == StationState.UNKNOWN):
                self.known_stations[det.marker_id] = det
            elif det.state != StationState.UNKNOWN:
                self.known_stations[det.marker_id] = det

        for mid, det in self.known_stations.items():
            status_dict[det.name] = {
                'id': mid,
                'state': det.state.value,
                'distance': round(float(det.distance), 2),
            }
        if status_dict:
            self.stations_status_pub.publish(String(data=json.dumps(status_dict, ensure_ascii=False)))

        active_stations = [d for d in detections if d.marker_id in (1, 2, 3) and d.state == StationState.ACTIVE]
        unload_markers = [d for d in detections if d.marker_id == 0]

        # 2. Логика переходов FSM
        if self.state == 'EXPLORE':
            if self.cargo_count > 0:
                if unload_markers:
                    self.get_logger().info('Обнаружен маркер выгрузки ArUco 0! Переход к DOCK_UNLOAD.')
                    self.state = 'DOCK_UNLOAD'
                else:
                    self.get_logger().info(f'На борту {self.cargo_count} груз(а). Направление в зону выгрузки (NAV_TO_UNLOAD).')
                    self.state = 'NAV_TO_UNLOAD'
            elif active_stations:
                target = min(active_stations, key=lambda d: d.distance)
                self.target_station_id = target.marker_id
                self.get_logger().info(
                    f'Захвачена активная станция {target.name} (ID {target.marker_id}, d={target.distance:.2f}м). Переход к DOCK_STATION.'
                )
                self.state = 'DOCK_STATION'

        elif self.state == 'NAV_TO_UNLOAD':
            if unload_markers:
                target = unload_markers[0]
                self.get_logger().info(f'В поле зрения маркер выгрузки ArUco 0 (d={target.distance:.2f}м)! Переход к DOCK_UNLOAD.')
                self.state = 'DOCK_UNLOAD'

        elif self.state == 'DOCK_STATION':
            det = next((d for d in detections if d.marker_id == self.target_station_id), None)
            if det:
                res = compute_docking_cmd(det.tvec, self.dock_config, front_laser)
                self.drive(res.linear_vel, res.angular_vel)
                if res.is_docked:
                    self.stop()
                    self.cargo_loader.start_loading(t, self.cargo_count)
                    self.state = 'LOAD_CARGO'
                    self.get_logger().info(f'Стыковка со станцией {self.target_station_id} выполнена! Переход к LOAD_CARGO.')
            else:
                self.stop()

        elif self.state == 'LOAD_CARGO':
            self.stop()
            self.set_tray(TRAY_DOWN)
            res = self.cargo_loader.update(
                current_time=t,
                current_cargo_count=self.cargo_count,
                linear_vel=self.speed[0],
                angular_vel=self.speed[1]
            )
            if res.status == LoadingStatus.SUCCESS:
                self.get_logger().info(f'Груз со станции {self.target_station_id} успешно получен! (На лотке: {self.cargo_count}). Переход к NAV_TO_UNLOAD.')
                if self.target_station_id in self.known_stations:
                    self.known_stations[self.target_station_id].state = StationState.ISSUED
                self.state = 'NAV_TO_UNLOAD'
            elif res.status == LoadingStatus.TIMEOUT:
                self.get_logger().warn(f'Таймаут погрузки со станции {self.target_station_id}. Возврат в EXPLORE.')
                self.state = 'EXPLORE'

        elif self.state == 'DOCK_UNLOAD':
            det = next((d for d in detections if d.marker_id == 0), None)
            if det:
                res = compute_docking_cmd(det.tvec, self.dock_config, front_laser)
                self.drive(res.linear_vel, res.angular_vel)
                if res.is_docked or front_laser <= 0.38:
                    self.stop()
                    self.unload_ctrl.start(t, yaw)
                    self.state = 'UNLOAD'
                    self.get_logger().info('Стыковка с зоной выгрузки завершена. Запуск цикла UNLOAD.')
            else:
                if front_laser <= 0.40:
                    self.stop()
                    self.unload_ctrl.start(t, yaw)
                    self.state = 'UNLOAD'
                else:
                    self.drive(0.12, 0.0)

        elif self.state == 'UNLOAD':
            v, w, tray_cmd, is_done, phase_name = self.unload_ctrl.step(
                current_time=t,
                current_yaw=yaw,
                rear_lidar_dist=rear_laser,
                cargo_count=self.cargo_count,
                tray_angle=self.tray_angle
            )
            self.drive(v, w)
            self.set_tray(tray_cmd)
            if is_done:
                self.stop()
                self.set_tray(TRAY_DOWN)
                self.get_logger().info('Выгрузка успешно завершена! Груз доставлен в зону (+25 баллов).')
                remaining_active = [d for d in self.known_stations.values() if d.marker_id in (1, 2, 3) and d.state == StationState.ACTIVE]
                if remaining_active or len(self.known_stations) < 3:
                    self.state = 'EXPLORE'
                else:
                    self.state = 'MISSION_COMPLETE'
                    self.get_logger().info('МИССИЯ ВЫПОЛНЕНА: Все грузы доставлены!')

        elif self.state == 'MISSION_COMPLETE':
            self.stop()
            self.set_tray(TRAY_DOWN)

        # 3. Публикация текущего состояния FSM для координации с навигатором
        self.state_pub.publish(String(data=self.state))

        # 4. Визуализация и оверлей
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
