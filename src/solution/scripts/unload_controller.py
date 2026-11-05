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
        target_rear_dist: float = 0.35,      # Целевая дальность по лидару сзади (лидар +0.09м -> бампер в 11 см от стены)
        min_safe_rear_dist: float = 0.28,    # Аварийный порог лидара сзади (бампер в 3.5 см от стены)
        turn_kp: float = 1.8,                # П-коэффициент угловой скорости при развороте
        max_turn_w: float = 1.2,             # Максимальная угловая скорость разворота (рад/с)
        min_turn_w: float = 0.25,            # Минимальная угловая скорость для преодоления трения покоя (рад/с)
        turn_tolerance: float = 0.06,        # Допустимая угловая ошибка завершения разворота (рад ~ 3.5°)
        reverse_speed: float = -0.10,        # Линейная скорость заднего хода (м/с)
        forward_speed: float = 0.12,         # Линейная скорость отъезда вперед (м/с)
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
    ) -> Tuple[float, float, float, bool, str]:
        """
        Выполняет один шаг управления циклом выгрузки.

        Входные параметры:
            current_time: текущее время симуляции (/clock), с
            current_yaw: текущий рыскание робота (из одометрии или IMU), рад
            rear_lidar_dist: минимальная дистанция лидара в заднем секторе [160°, 200°], м
            cargo_count: текущее число грузов на лотке (/courier/cargo_count)
            tray_angle: текущий фактический угол лотка (/courier/tray_state), рад

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
        # 2. Фаза заднего хода к стене зоны выгрузки
        # ----------------------------------------------------
        if self.state == self.STATE_BACKUP:
            if (rear_lidar_dist <= self.target_rear_dist or
                    rear_lidar_dist <= self.min_safe_rear_dist or
                    elapsed_in_phase >= self.phase_timeout):
                self.state = self.STATE_TILT_UP
                self.phase_start_time = current_time
                return 0.0, 0.0, self.tray_unload_angle, False, self.STATE_TILT_UP

            return self.reverse_speed, 0.0, 0.0, False, self.STATE_BACKUP

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
                    self.state = self.STATE_TILT_DOWN
                    self.phase_start_time = current_time
                    return 0.0, 0.0, 0.0, False, self.STATE_TILT_DOWN
            else:
                self.settle_start_time = None

            if elapsed_in_phase >= self.phase_timeout:
                # Таймаут ожидания опустошения — принудительно опускаем лоток
                self.state = self.STATE_TILT_DOWN
                self.phase_start_time = current_time
                return 0.0, 0.0, 0.0, False, self.STATE_TILT_DOWN

            return 0.0, 0.0, self.tray_unload_angle, False, self.STATE_WAIT_EMPTY

        # ----------------------------------------------------
        # 5. Фаза опускания лотка в исходное положение
        # ----------------------------------------------------
        if self.state == self.STATE_TILT_DOWN:
            if tray_angle <= 0.08 or elapsed_in_phase >= 2.0:
                self.state = self.STATE_CLEAR_FORWARD
                self.phase_start_time = current_time
                return self.forward_speed, 0.0, 0.0, False, self.STATE_CLEAR_FORWARD

            return 0.0, 0.0, 0.0, False, self.STATE_TILT_DOWN

        # ----------------------------------------------------
        # 6. Фаза безопасного отъезда вперед от зоны выгрузки
        # ----------------------------------------------------
        if self.state == self.STATE_CLEAR_FORWARD:
            if elapsed_in_phase >= self.forward_duration:
                self.state = self.STATE_DONE
                self.is_done = True
                return 0.0, 0.0, 0.0, True, self.STATE_DONE

            return self.forward_speed, 0.0, 0.0, False, self.STATE_CLEAR_FORWARD

        return 0.0, 0.0, 0.0, False, self.state

