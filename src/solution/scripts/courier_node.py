#!/usr/bin/env python3
"""Распознавание станций и управление циклом доставки."""

import json
import math
import sys
from pathlib import Path

# Подключение локальных модулей решения
script_dir = Path(__file__).resolve().parent
if str(script_dir) not in sys.path:
    sys.path.insert(0, str(script_dir))

from station_detector import StationDetection, StationDetector, StationState
from docking_controller import DockingConfig, compute_docking_cmd
from cargo_loader import CargoLoader, LoaderConfig, LoadingStatus
from unload_controller import UnloadController

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
from std_msgs.msg import Float64, String, UInt8
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

TRAY_DOWN = 0.0


class Courier(Node):
    def __init__(self):
        super().__init__("courier")
        self.declare_parameter("gui", True)
        self.gui = self.get_parameter("gui").value

        # ---- состояние и сенсоры
        self.scan = None
        self.pose = (0.0, 0.0, 0.0)  # x, y, yaw в системе odom
        self.speed = (0.0, 0.0)  # v, w
        self.imu = None
        self.image = None  # BGR numpy-массив
        self.cargo_count = 0
        self.tray_angle = 0.0
        self.bridge = CvBridge()

        # ---- компоненты подсистемы доставки
        self.detector = StationDetector()
        self.dock_config = DockingConfig(target_dist=0.65, min_safe_laser_dist=0.28)
        self.cargo_loader = CargoLoader(config=LoaderConfig(dwell_target=2.0, timeout=8.0))
        self.station_distances = {}
        self.unload_ctrl = UnloadController(
            target_rear_dist=0.315, min_safe_rear_dist=0.285
        )
        self.unload_count = 0

        # Реестр обнаруженных станций {marker_id: StationDetection}
        self.known_stations = {}
        self.station_cells = {}
        self.collected_stations = set()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_station_id = None
        self.station_target = None
        self.station_nav_started = None
        self.station_retry_after = {}
        self.start_pose = None
        self.stationary_explore_start = None
        self.state = "WAIT_SENSORS"
        self.status_detail = "Waiting for camera and lidar"

        # ---- подписки
        self.create_subscription(
            LaserScan, "/scan", self.on_scan, qos_profile_sensor_data
        )
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(Imu, "/imu", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(
            Image, "/camera/image_raw", self.on_image, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo,
            "/camera/camera_info",
            self.on_camera_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(UInt8, "/courier/cargo_count", self.on_cargo, 10)
        self.create_subscription(Float64, "/courier/tray_state", self.on_tray, 10)

        # ---- публикаторы
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.tray_pub = self.create_publisher(Float64, "/courier/tray_cmd", 10)
        self.state_pub = self.create_publisher(String, "/delivery/state", 10)
        self.status_marker_pub = self.create_publisher(
            Marker, "/delivery/status_marker", 10
        )
        self.station_markers_pub = self.create_publisher(
            MarkerArray, "/delivery/station_markers", 10
        )
        self.station_target_pub = self.create_publisher(
            PointStamped, "/delivery/station_target", 10
        )
        self.create_subscription(
            String, "/delivery/navigation_result", self.on_station_navigation, 10
        )
        self.stations_status_pub = self.create_publisher(
            String, "/delivery/stations_status", 10
        )

        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Гц
        self.get_logger().info("Courier (модуль доставки) запущен, ожидание сенсоров…")

    # ================================================================ колбэки сенсоров

    def on_station_navigation(self, msg):
        if self.state == "NAV_TO_UNLOAD":
            if msg.data == "reached":
                self.stop()
                self.state = "DOCK_UNLOAD"
                self.get_logger().info("Nav2 достиг зоны выгрузки; точная стыковка.")
                self.state_pub.publish(String(data=self.state))
            return
        if self.state != "DOCK_STATION":
            return
        if msg.data == "reached":
            self.stop()
            self.cargo_loader.start_loading(self.now_sec(), self.cargo_count)
            self.state = "LOAD_CARGO"
            self.get_logger().info("Nav2 достиг клетки станции; ожидание груза.")
        else:
            self.station_retry_after[self.target_station_id] = self.now_sec() + 10.0
            self.state = "EXPLORE"
            self.get_logger().warn("Nav2 не достиг станции; возврат к исследованию.")
        self.state_pub.publish(String(data=self.state))

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
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.image_header = msg.header

    def on_cargo(self, msg: UInt8):
        if msg.data != self.cargo_count:
            self.get_logger().info(
                f"Грузов на лотке: {msg.data} (было {self.cargo_count})"
            )
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

    def publish_status_marker(self):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "delivery_status"
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose.position.y = 1.3
        marker.pose.position.z = 0.6
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.18
        marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0
        station = self.target_station_id if self.target_station_id is not None else "-"
        marker.text = (
            f"{self.state}\nStation: {station} | Cargo: {self.cargo_count}\n"
            f"{self.status_detail}"
        )
        self.status_marker_pub.publish(marker)

    def remember_station_cell(self, detection):
        if detection.marker_id not in (1, 2, 3):
            return
        # Фильтр ArUco: принимаем детекции до 3.5 м
        if detection.distance <= 0.15 or detection.distance > 3.5:
            return
        bearing = math.atan2(float(detection.tvec[0]), float(detection.tvec[2]))
        if abs(bearing) > math.radians(50.0):
            return

        prev_dist = self.station_distances.get(detection.marker_id, float('inf'))
        if detection.marker_id in self.station_cells and detection.distance >= prev_dist:
            return

        # Отступ 0.65 м вдоль луча зрения от маркера к роботу
        # Гарантирует попадание строго в ядро свободной клетки коридора перед станцией (без шума PnP rvec)
        RAY_OFFSET = 0.65
        norm_tvec = float(detection.distance)
        if norm_tvec > 1e-3:
            safe_dist = max(0.20, norm_tvec - RAY_OFFSET)
            scale = safe_dist / norm_tvec
            safe_pt = detection.tvec * scale
        else:
            safe_pt = detection.tvec

        point = PointStamped()
        point.header = self.image_header
        point.point.x, point.point.y, point.point.z = map(float, safe_pt)
        try:
            transform = self.tf_buffer.lookup_transform(
                "maze_grid",
                point.header.frame_id,
                rclpy.time.Time(),
            )
        except TransformException:
            return
        position = do_transform_point(point, transform).point
        cell = math.floor(position.x / 2 + 0.5), math.floor(position.y / 2 + 0.5)
        self.station_cells[detection.marker_id] = cell
        self.station_distances[detection.marker_id] = detection.distance
        self.get_logger().info(f"Запомнена станция {detection.name}: клетка {cell} (дистанция {detection.distance:.2f} м).")

    def publish_station_markers(self):
        markers = []
        for station_id, cell in self.station_cells.items():
            marker = Marker()
            marker.header.frame_id = "maze_grid"
            marker.frame_locked = True
            marker.ns = "station_cells"
            marker.id = station_id
            marker.type = Marker.CUBE
            marker.pose.position.x, marker.pose.position.y = float(2 * cell[0]), float(
                2 * cell[1]
            )
            marker.pose.position.z = 0.05
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = 1.8
            marker.scale.z = 0.02
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
                0.75,
                0.2,
                1.0,
                0.5,
            )
            markers.append(marker)
        self.station_markers_pub.publish(MarkerArray(markers=markers))

    # ================================================================ лидар

    def sector_min(self, from_deg: float, to_deg: float) -> float:
        """Минимальная дальность лидара в секторе углов [from, to] (0° — вперёд, + — влево)."""
        s = self.scan
        if s is None:
            return float("inf")
        best = float("inf")
        n = len(s.ranges)
        for deg in range(int(from_deg), int(to_deg) + 1):
            i = int(round((math.radians(deg) - s.angle_min) / s.angle_increment)) % n
            r = s.ranges[i]
            if s.range_min <= r < best:
                best = r
        return best

    # ================================================================ конечный автомат (FSM)

    def control_loop(self):
        if self.state == "WAIT_SENSORS":
            if self.scan is not None and self.image is not None:
                self.get_logger().info("Все датчики готовы — старт миссии доставки!")
                self.set_tray(TRAY_DOWN)
                self.start_pose = (self.pose[0], self.pose[1], self.pose[2])
                self.state = "EXPLORE"
            self.publish_status_marker()
            return

        t = self.now_sec()
        yaw = self.pose[2]
        front_laser = self.sector_min(-15, 15)
        rear_laser = self.sector_min(160, 200)
        self.status_detail = {
            "EXPLORE": "Exploring maze",
            "NAV_TO_UNLOAD": "Nav2: returning to start",
            "DOCK_STATION": "Nav2: approaching station cell",
            "LOAD_CARGO": "Waiting for cargo",
            "MISSION_COMPLETE": "Mission complete",
        }.get(self.state, "Starting docking")

        # 1. Компьютерное зрение: детекция станций, ламп и зоны выгрузки
        detections = self.detector.detect(self.image)
        status_dict = {}

        for det in detections:
            self.remember_station_cell(det)
            prev = self.known_stations.get(det.marker_id)
            if prev is None:
                self.known_stations[det.marker_id] = det
            elif prev.state == StationState.ACTIVE:
                # Защита от шума: однажды подтверждённая ACTIVE станция не может стать INACTIVE/UNKNOWN
                if det.state == StationState.ISSUED:
                    self.known_stations[det.marker_id] = det
                else:
                    # Обновляем позу и углы, сохраняя статус ACTIVE
                    det_updated = StationDetection(
                        marker_id=det.marker_id,
                        name=det.name,
                        state=StationState.ACTIVE,
                        distance=det.distance,
                        tvec=det.tvec,
                        rvec=det.rvec,
                        marker_corners=det.marker_corners,
                        lamp_center_px=det.lamp_center_px,
                        lamp_radius_px=det.lamp_radius_px,
                        confidence=max(prev.confidence, det.confidence),
                    )
                    self.known_stations[det.marker_id] = det_updated
            elif det.state != StationState.UNKNOWN:
                self.known_stations[det.marker_id] = det

        for mid, det in self.known_stations.items():
            status_dict[det.name] = {
                "id": mid,
                "state": det.state.value,
                "distance": round(float(det.distance), 2),
                "cell": self.station_cells.get(mid),
                "collected": mid in self.collected_stations,
            }
        if status_dict:
            self.stations_status_pub.publish(
                String(data=json.dumps(status_dict, ensure_ascii=False))
            )

        active_stations = [
            d
            for d in self.known_stations.values()
            if d.marker_id in self.station_cells
            and d.state == StationState.ACTIVE
            and d.marker_id not in self.collected_stations
            and t >= self.station_retry_after.get(d.marker_id, 0.0)
        ]
        self.publish_station_markers()

        # 2. Логика переходов FSM
        all_stations_accounted = (
            len(self.collected_stations)
            + len([d for d in self.known_stations.values() if d.state == StationState.INACTIVE])
            >= 3
        )

        # Мгновенная погрузка при обнаружении маркера станции вблизи:
        close_station = next(
            (
                d
                for d in detections
                if d.marker_id in (1, 2, 3)
                and d.marker_id not in self.collected_stations
                and d.distance < 0.85
            ),
            None,
        )
        if close_station and self.state in ("EXPLORE", "DOCK_STATION"):
            self.stop()
            self.target_station_id = close_station.marker_id
            self.cargo_loader.start_loading(t, self.cargo_count)
            self.state = "LOAD_CARGO"
            self.get_logger().info(
                f"[INSTANT CARGO LOAD] Маркер станции {close_station.name} прямо перед роботом (d={close_station.distance:.2f}м < 0.85м). Запуск LOAD_CARGO!"
            )

        if self.state == "EXPLORE":
            # Защита от зависания исследования с грузом на борту:
            if self.cargo_count > 0:
                self.get_logger().info(
                    f"На борту {self.cargo_count} груз(а). Направление в зону выгрузки (NAV_TO_UNLOAD)."
                )
                self.state = "NAV_TO_UNLOAD"
            elif active_stations:
                try:
                    position = self.tf_buffer.lookup_transform(
                        "maze_grid", "base_link", rclpy.time.Time()
                    ).transform.translation
                    cur_pos = (position.x, position.y)
                except TransformException:
                    cur_pos = (self.pose[0], self.pose[1])
                target = min(
                    active_stations,
                    key=lambda d: math.dist(
                        cur_pos,
                        tuple(
                            2 * coordinate
                            for coordinate in self.station_cells[d.marker_id]
                        ),
                    ),
                )
                self.target_station_id = target.marker_id
                cell = self.station_cells[target.marker_id]
                cell_pos = (float(2 * cell[0]), float(2 * cell[1]))
                dist_to_st = math.dist(cur_pos, cell_pos)
                if dist_to_st < 0.65:
                    self.stop()
                    self.cargo_loader.start_loading(t, self.cargo_count)
                    self.state = "LOAD_CARGO"
                    self.get_logger().info(
                        f"[INSTANT DOCK] Робот уже в зоне станции {target.name} (d={dist_to_st:.2f}м < 0.65м). Переход к LOAD_CARGO!"
                    )
                else:
                    self.station_target = PointStamped()
                    self.station_target.header.frame_id = "maze_grid"
                    self.station_target.point.x = cell_pos[0]
                    self.station_target.point.y = cell_pos[1]
                    self.station_nav_started = t
                    self.state = "DOCK_STATION"
                    self.get_logger().info(
                        f"Станция {target.name}: Nav2 направляется к её клетке."
                    )

        elif self.state == "DOCK_STATION":
            try:
                position = self.tf_buffer.lookup_transform(
                    "maze_grid", "base_link", rclpy.time.Time()
                ).transform.translation
                cur_pos = (position.x, position.y)
            except TransformException:
                cur_pos = (self.pose[0], self.pose[1])
            cell = self.station_cells.get(self.target_station_id)
            if cell is not None and math.dist(cur_pos, (2 * cell[0], 2 * cell[1])) < 0.65:
                self.stop()
                self.cargo_loader.start_loading(t, self.cargo_count)
                self.state = "LOAD_CARGO"
                self.get_logger().info(
                    f"Робот вошёл в ядро станции {self.target_station_id}. Переход к LOAD_CARGO."
                )
            elif t - self.station_nav_started >= 45.0:
                self.station_retry_after[self.target_station_id] = t + 10.0
                self.state = "EXPLORE"
                self.get_logger().warn(
                    "Таймаут пути к станции (45 с); возврат к исследованию."
                )

        elif self.state == "NAV_TO_UNLOAD":
            det_0 = next((d for d in detections if d.marker_id == 0), None)
            try:
                pos = self.tf_buffer.lookup_transform(
                    "maze_grid", "base_link", rclpy.time.Time()
                ).transform.translation
                in_start_zone = (pos.x <= 1.8 and pos.y <= 1.8)
            except TransformException:
                in_start_zone = False

            if det_0 and (det_0.distance < 1.5 or in_start_zone):
                self.stop()
                self.get_logger().info(
                    f"Маркер выгрузки ArUco 0 обнаружен (d={det_0.distance:.2f}м, в зоне старта={in_start_zone}). Переход к DOCK_UNLOAD."
                )
                self.state = "DOCK_UNLOAD"
            elif in_start_zone and det_0 is None:
                # В зоне старта, но маркер ArUco 0 не в кадре — плавно вращаемся для поиска маркера
                self.drive(0.0, 0.4)
                self.status_detail = "In start zone: scanning for ArUco 0 marker"

        elif self.state == "LOAD_CARGO":
            self.set_tray(TRAY_DOWN)
            res = self.cargo_loader.update(
                current_time=t,
                current_cargo_count=self.cargo_count,
                linear_vel=self.speed[0],
                angular_vel=self.speed[1],
            )
            # В фазе загрузки робот ОБЯЗАН стоять неподвижно (referee требует lin < 0.03, ang < 0.15 в течение 2.0 с).
            # Любое подруливание сбрасывает таймер судьи в ноль — держим 0.0 м/с!
            self.stop()
            self.status_detail = (
                f"Loading: {res.status.value}\nStill for {res.dwell_time:.1f} s"
            )
            self.get_logger().info(
                f"Загрузка: статус={res.status.value}, покой={res.dwell_time:.1f} с, "
                f"ожидание={t - self.cargo_loader.docked_timestamp:.1f} с, "
                f"груз={self.cargo_count}, v={self.speed[0]:.3f}, w={self.speed[1]:.3f}",
                throttle_duration_sec=1.0,
            )
            if res.status == LoadingStatus.SUCCESS:
                self.collected_stations.add(self.target_station_id)
                self.get_logger().info(
                    f"Груз со станции {self.target_station_id} успешно получен! (На лотке: {self.cargo_count}). Переход к NAV_TO_UNLOAD."
                )
                if self.target_station_id in self.known_stations:
                    self.known_stations[self.target_station_id].state = (
                        StationState.ISSUED
                    )
                self.state = "NAV_TO_UNLOAD"
            elif res.status == LoadingStatus.TIMEOUT:
                self.station_retry_after[self.target_station_id] = t + 10.0
                self.get_logger().warn(
                    f"Таймаут погрузки со станции {self.target_station_id}. Возврат в EXPLORE."
                )
                self.state = "EXPLORE"

        elif self.state == "DOCK_UNLOAD":
            det = next((d for d in detections if d.marker_id == 0), None)
            if det:
                offset = (0.0, 0.2, -0.2)[self.unload_count % 3]
                rotation, _ = cv2.Rodrigues(det.rvec)
                target = det.tvec + offset * rotation[:, 0]
                res = compute_docking_cmd(target, self.dock_config, front_laser)
                self.status_detail = (
                    f"Unload position: {offset:+.2f} m along wall\n"
                    f"d={det.distance:.2f} m | lidar={front_laser:.2f} m"
                )
                self.get_logger().info(
                    f"Стыковка выгрузки: d={det.distance:.2f} м, "
                    f"позиция={self.unload_count + 1}, смещение вдоль стены={offset:+.2f} м, "
                    f"угол={math.degrees(res.angle_error):.1f}°, "
                    f"v={res.linear_vel:.2f}, w={res.angular_vel:.2f}, лидар={front_laser:.2f} м",
                    throttle_duration_sec=1.0,
                )
                self.drive(res.linear_vel, res.angular_vel)
                if res.is_docked:
                    self.stop()
                    self.unload_ctrl.start(t, yaw)
                    self.state = "UNLOAD"
                    self.get_logger().info(
                        "Стыковка с зоной выгрузки завершена. Запуск цикла UNLOAD."
                    )
            else:
                self.status_detail = f"Unload marker lost | lidar={front_laser:.2f} m"
                self.get_logger().warn(
                    f"Маркер выгрузки не виден; лидар={front_laser:.2f} м.",
                    throttle_duration_sec=1.0,
                )
                self.stop()

        elif self.state == "UNLOAD":
            previous_phase = self.unload_ctrl.state
            phase_elapsed = t - self.unload_ctrl.phase_start_time
            yaw_error = self.unload_ctrl.target_yaw - yaw
            yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
            phase_ready = {
                "TURN_180": abs(yaw_error) <= self.unload_ctrl.turn_tolerance,
                "BACKUP": rear_laser <= self.unload_ctrl.target_rear_dist,
                "TILT_UP": self.tray_angle >= 0.85 * self.unload_ctrl.tray_unload_angle,
                "WAIT_EMPTY": (
                    self.cargo_count == 0
                    and self.unload_ctrl.settle_start_time is not None
                    and t - self.unload_ctrl.settle_start_time
                    >= self.unload_ctrl.settle_dwell_time
                ),
                "TILT_DOWN": self.tray_angle <= 0.08,
                "CLEAR_FORWARD": phase_elapsed >= self.unload_ctrl.forward_duration,
            }
            v, w, tray_cmd, is_done, phase_name = self.unload_ctrl.step(
                current_time=t,
                current_yaw=yaw,
                rear_lidar_dist=rear_laser,
                cargo_count=self.cargo_count,
                tray_angle=self.tray_angle,
                scan=self.scan,
            )
            if phase_name != previous_phase:
                timed_out = not phase_ready.get(previous_phase, True)
                self.get_logger().info(
                    f"Выгрузка {previous_phase} -> {phase_name}: "
                    f'{"таймаут" if timed_out else "условие выполнено"}, '
                    f"прошло {phase_elapsed:.1f} с, груз={self.cargo_count}, "
                    f"кузов={self.tray_angle:.2f}, задний лидар={rear_laser:.2f} м."
                )
            self.get_logger().info(
                f"Выгрузка: фаза={phase_name}, груз={self.cargo_count}, "
                f"кузов={self.tray_angle:.2f}, задний лидар={rear_laser:.2f} м, "
                f"v={v:.2f}, w={w:.2f}",
                throttle_duration_sec=1.0,
            )
            self.drive(v, w)
            self.set_tray(tray_cmd)
            self.status_detail = f"Unload: {phase_name}\nTray={self.tray_angle:.2f} rad | rear={rear_laser:.2f} m"
            if is_done:
                self.stop()
                self.set_tray(TRAY_DOWN)
                self.unload_count += 1
                self.get_logger().info(
                    f"Манёвр выгрузки {self.unload_count} завершён; "
                    f"на лотке осталось {self.cargo_count}. Всего станций собрано: {len(self.collected_stations)}/3."
                )
                if len(self.collected_stations) >= 3 or all_stations_accounted:
                    self.state = "MISSION_COMPLETE"
                    self.get_logger().info(
                        f"Все активные станции собраны и доставлены (собрано: {len(self.collected_stations)}, учтено 3)! Миссия завершена."
                    )
                else:
                    self.state = "EXPLORE"

        elif self.state == "MISSION_COMPLETE":
            self.stop()
            self.set_tray(TRAY_DOWN)

        # 3. Публикация текущего состояния FSM для координации с навигатором
        self.state_pub.publish(String(data=self.state))
        if self.state == "DOCK_STATION":
            self.station_target_pub.publish(self.station_target)
        self.publish_status_marker()

        # 4. Визуализация и оверлей
        if self.image is not None:
            view = self.detector.draw_detections(self.image, detections)
            cv2.putText(
                view,
                f"state={self.state} cargo={self.cargo_count}",
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )
            cv2.imwrite("/ros_ws/src/solution/camera_view.jpg", view)

            if self.gui:
                cv2.imshow("courier camera", view)
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


if __name__ == "__main__":
    main()
