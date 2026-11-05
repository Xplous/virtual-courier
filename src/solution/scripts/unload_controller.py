#!/usr/bin/env python3
"""
Модуль выгрузки груза в зоне выгрузки у Старта (ArUco 0).

Конечный автомат (FSM) процесса выгрузки:
1. TURN_180: разворот робота на 180° кормой к стене зоны выгрузки.
2. BACKUP: контролируемый подъезд задним ходом по лидару (задний сектор).
3. TILT_UP: подъем лотка до угла ~1.0 рад (~57°).
4. WAIT_EMPTY: фиксация падения грузов (cargo_count == 0) + время успокоения (dwell time).
5. TILT_DOWN: возврат лотка в горизонтальное положение (0.0 рад).
6. CLEAR_FORWARD: отъезд вперед на безопасное расстояние от выгруженных кубиков.
7. DONE: завершение миссии выгрузки.
"""

import math
from typing import Tuple


def normalize_angle(angle: float) -> float:
    """Нормализует угол в диапазон [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def extract_rear_wall_metrics(scan) -> Tuple[float, float]:
    """
    Анализирует точки лидара в декартовом коридоре габарита робота (|y| <= 0.28 м, x in [-1.6, -0.10] м).
    Боковые стены коридора (|y| >= 0.70 м) физически не могут попасть в этот коридор.
    Возвращает:
      - d_perp: перпендикулярное расстояние от лидара до задней стены (м)
      - angle_error: отклонение от перпендикуляра к стене (рад), где 0.0 — строго перпендикулярно.
    """
    if scan is None or not hasattr(scan, "ranges"):
        return None, None
    pts_x = []
    pts_y = []
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r < 0.12 or r > 2.0:
            continue
        angle = scan.angle_min + i * scan.angle_increment
        x = r * math.cos(angle)
        y = r * math.sin(angle)
        if -1.6 <= x <= -0.10 and abs(y) <= 0.28:
            pts_x.append(x)
            pts_y.append(y)

    if len(pts_x) < 5:
        return None, None

    n = len(pts_x)
    mean_x = sum(pts_x) / n
    mean_y = sum(pts_y) / n
    num = sum((pts_y[i] - mean_y) * (pts_x[i] - mean_x) for i in range(n))
    den = sum((pts_y[i] - mean_y) ** 2 for i in range(n))
    if den < 1e-6:
        return abs(mean_x), 0.0

    k = num / den
    angle_error = math.atan(k)
    d_perp = abs(mean_x - k * mean_y)
    return d_perp, angle_error


class UnloadController:
    # Состояния FSM
    STATE_IDLE = 'IDLE'
    STATE_TURN_180 = 'TURN_180'
    STATE_BACKUP = 'BACKUP'
    STATE_TILT_UP = 'TILT_UP'
    STATE_WAIT_EMPTY = 'WAIT_EMPTY'
    STATE_TILT_DOWN = 'TILT_DOWN'
    STATE_CLEAR_FORWARD = 'CLEAR_FORWARD'
    STATE_DONE = 'DONE'

    def __init__(
        self,
        target_rear_dist: float = 0.365,     # Вынос лидара до бампера 0.265 м + зазор 0.10 м.
        min_safe_rear_dist: float = 0.345,   # Аварийный зазор за бампером 0.08 м.
        turn_kp: float = 1.8,                # П-коэффициент угловой скорости при развороте
        max_turn_w: float = 1.2,             # Максимальная угловая скорость разворота (рад/с)
        min_turn_w: float = 0.25,            # Минимальная угловая скорость для преодоления трения покоя (рад/с)
        turn_tolerance: float = 0.06,        # Допустимая угловая ошибка завершения разворота (рад ~ 3.5°)
        reverse_speed: float = -0.09,        # Линейная скорость заднего хода (м/с)
        forward_speed: float = 0.14,         # Линейная скорость отъезда вперед (м/с)
        forward_duration: float = 4.0,       # Длительность отъезда вперед (с) (~0.48 м пути в центр клетки)
        tray_unload_angle: float = 1.0,      # Рабочий угол наклона лотка при выгрузке (рад ~ 57.3°)
        settle_dwell_time: float = 2.0,      # Время успокоения упавших кубиков до v < 0.02 м/с для зачета судьей (с)
        phase_timeout: float = 8.0,          # Сторожевой таймаут на отдельную фазу (с)
    ):
        self.target_rear_dist = float(target_rear_dist)
        self.min_safe_rear_dist = float(min_safe_rear_dist)
        self.turn_kp = float(turn_kp)
        self.max_turn_w = float(max_turn_w)
        self.min_turn_w = float(min_turn_w)
        self.turn_tolerance = float(turn_tolerance)
        self.reverse_speed = float(reverse_speed)
        self.forward_speed = float(forward_speed)
        self.forward_duration = float(forward_duration)
        self.tray_unload_angle = float(tray_unload_angle)
        self.settle_dwell_time = float(settle_dwell_time)
        self.phase_timeout = float(phase_timeout)

        self.state = self.STATE_IDLE
        self.target_yaw = 0.0
        self.phase_start_time = 0.0
        self.settle_start_time = None
        self.is_done = False

    def start(self, current_time: float, current_yaw: float) -> None:
        """Инициализирует и запускает цикл выгрузки с текущей позы робота."""
        self.state = self.STATE_TURN_180
        self.target_yaw = normalize_angle(current_yaw + math.pi)
        self.phase_start_time = float(current_time)
        self.settle_start_time = None
        self.is_done = False

    def reset(self) -> None:
        """Сбрасывает состояние контроллера в исходное."""
        self.state = self.STATE_IDLE
        self.target_yaw = 0.0
        self.phase_start_time = 0.0
        self.settle_start_time = None
        self.is_done = False

    def step(
        self,
        current_time: float,
        current_yaw: float,
        rear_lidar_dist: float,
        cargo_count: int,
        tray_angle: float,
        scan=None,
    ) -> Tuple[float, float, float, bool, str]:
        """
        Выполняет один шаг управления циклом выгрузки.

        Входные параметры:
            current_time: текущее время симуляции (/clock), с
            current_yaw: текущий рыскание робота (из одометрии или IMU), рад
            rear_lidar_dist: минимальная дистанция лидара в заднем секторе [160°, 200°], м
            cargo_count: текущее число грузов на лотке (/courier/cargo_count)
            tray_angle: текущий фактический угол лотка (/courier/tray_state), рад
            scan: полный LaserScan для декартовой фильтрации стены и выравнивания

        Возвращает:
            (linear_vel, angular_vel, tray_cmd, is_done, state_name)
        """
        if self.state == self.STATE_IDLE:
            return 0.0, 0.0, 0.0, False, self.STATE_IDLE

        if self.state == self.STATE_DONE:
            return 0.0, 0.0, 0.0, True, self.STATE_DONE

        elapsed_in_phase = current_time - self.phase_start_time

        # ----------------------------------------------------
        # 1. Фаза разворота на 180 градусов кормой к стене
        # ----------------------------------------------------
        if self.state == self.STATE_TURN_180:
            yaw_error = normalize_angle(self.target_yaw - current_yaw)
            if abs(yaw_error) <= self.turn_tolerance or elapsed_in_phase >= self.phase_timeout:
                self.state = self.STATE_BACKUP
                self.phase_start_time = current_time
                return 0.0, 0.0, 0.0, False, self.STATE_BACKUP

            w = self.turn_kp * yaw_error
            w = max(-self.max_turn_w, min(self.max_turn_w, w))
            if abs(w) < self.min_turn_w:
                w = math.copysign(self.min_turn_w, yaw_error)
            return 0.0, float(w), 0.0, False, self.STATE_TURN_180

        # ----------------------------------------------------
        # 2. Фаза заднего хода к стене зоны выгрузки с выравниванием
        # ----------------------------------------------------
        if self.state == self.STATE_BACKUP:
            d_perp, angle_err = extract_rear_wall_metrics(scan)
            effective_dist = d_perp if d_perp is not None else rear_lidar_dist

            if (effective_dist <= self.target_rear_dist or
                    effective_dist <= self.min_safe_rear_dist or
                    elapsed_in_phase >= self.phase_timeout):
                self.state = self.STATE_TILT_UP
                self.phase_start_time = current_time
                return 0.0, 0.0, self.tray_unload_angle, False, self.STATE_TILT_UP

            # Подруливание по углу наклона стены (|d_theta| < 2 град)
            w = 0.0
            if angle_err is not None and abs(angle_err) > 0.035:
                w = max(-0.30, min(0.30, -1.8 * angle_err))

            # Плавное замедление перед стеной
            dist_to_go = max(0.0, effective_dist - self.target_rear_dist)
            v = max(self.reverse_speed, min(-0.04, -0.6 * dist_to_go - 0.04))
            return v, w, 0.0, False, self.STATE_BACKUP

        # ----------------------------------------------------
        # 3. Фаза подъема кузова (опрокидывание)
        # ----------------------------------------------------
        if self.state == self.STATE_TILT_UP:
            if tray_angle >= 0.85 * self.tray_unload_angle or elapsed_in_phase >= 2.0:
                self.state = self.STATE_WAIT_EMPTY
                self.phase_start_time = current_time
                self.settle_start_time = None
                return 0.0, 0.0, self.tray_unload_angle, False, self.STATE_WAIT_EMPTY

            return 0.0, 0.0, self.tray_unload_angle, False, self.STATE_TILT_UP

        # ----------------------------------------------------
        # 4. Фаза ожидания сброса груза и успокоения кубиков
        # ----------------------------------------------------
        if self.state == self.STATE_WAIT_EMPTY:
            if cargo_count == 0:
                if self.settle_start_time is None:
                    self.settle_start_time = current_time
                elif (current_time - self.settle_start_time) >= self.settle_dwell_time:
                    self.state = self.STATE_CLEAR_FORWARD
                    self.phase_start_time = current_time
                    return 0.05, 0.0, self.tray_unload_angle, False, self.STATE_CLEAR_FORWARD
            else:
                self.settle_start_time = None

            if elapsed_in_phase >= self.phase_timeout:
                self.state = self.STATE_CLEAR_FORWARD
                self.phase_start_time = current_time
                return 0.05, 0.0, self.tray_unload_angle, False, self.STATE_CLEAR_FORWARD

            return 0.0, 0.0, self.tray_unload_angle, False, self.STATE_WAIT_EMPTY

        # ----------------------------------------------------
        # 5. Фаза опускания лотка (сохранена для совместимости)
        # ----------------------------------------------------
        if self.state == self.STATE_TILT_DOWN:
            if tray_angle <= 0.08 or elapsed_in_phase >= 2.0:
                self.state = self.STATE_CLEAR_FORWARD
                self.phase_start_time = current_time
                return 0.05, 0.0, 0.0, False, self.STATE_CLEAR_FORWARD

            return 0.0, 0.0, 0.0, False, self.STATE_TILT_DOWN

        # ----------------------------------------------------
        # 6. Фаза безопасного отъезда вперед (Soft Ramp)
        # ----------------------------------------------------
        if self.state == self.STATE_CLEAR_FORWARD:
            if elapsed_in_phase >= self.forward_duration:
                self.state = self.STATE_DONE
                self.is_done = True
                return 0.0, 0.0, 0.0, True, self.STATE_DONE

            # Soft Ramp:
            # 0.0..0.5 с: v = 0.05 м/с, лоток ПОДНЯТ (tray_cmd = 1.0)
            # 0.5..2.5 с: разгон v: 0.05 -> forward_speed, лоток ПОДНЯТ (tray_cmd = 1.0)
            # 2.5..forward_duration: v = forward_speed, лоток ОПУСКАЕТСЯ (tray_cmd = 0.0)
            if elapsed_in_phase < 0.5:
                v = 0.05
                tray_cmd = self.tray_unload_angle
            elif elapsed_in_phase < 2.5:
                progress = (elapsed_in_phase - 0.5) / 2.0
                v = 0.05 + progress * (self.forward_speed - 0.05)
                tray_cmd = self.tray_unload_angle
            else:
                v = self.forward_speed
                tray_cmd = 0.0

            return v, 0.0, tray_cmd, False, self.STATE_CLEAR_FORWARD

        return 0.0, 0.0, 0.0, False, self.state

