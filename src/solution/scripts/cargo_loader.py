#!/usr/bin/env python3
"""Модуль управления процессом загрузки груза на активной станции."""

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class LoadingStatus(str, Enum):
    IDLE = 'IDLE'               # Ожидание команды на старт загрузки
    WAITING = 'WAITING'         # Робот зафиксирован, идёт отсчёт времени покоя
    SUCCESS = 'SUCCESS'         # Груз успешно получен (cargo_count увеличился)
    TIMEOUT = 'TIMEOUT'         # Истёк защитный таймаут, груз не выдан
    MOVED_RESET = 'MOVED_RESET' # Робот дёрнулся, таймер покоя сброшен


@dataclass
class LoaderConfig:
    """Параметры контроллера загрузки."""
    dwell_target: float = 2.0         # Норматив непрерывной неподвижности судьи (с)
    timeout: float = 4.0              # Максимальный сторожевой интервал ожидания (с)
    still_lin_thresh: float = 0.025   # Порог покоя по линейной скорости (м/с, у судьи 0.03)
    still_ang_thresh: float = 0.12    # Порог покоя по угловой скорости (рад/с, у судьи 0.15)
    tray_level_angle: float = 0.0     # Горизонтальный угол лотка (рад)


@dataclass
class LoaderOutput:
    status: LoadingStatus             # Текущий статус загрузки
    linear_vel: float                 # Рекомендуемая скорость вперед (0.0 м/с)
    angular_vel: float                # Рекомендуемая скорость поворота (0.0 рад/с)
    tray_angle: float                 # Рекомендуемый наклон кузова (0.0 рад)
    dwell_time: float                 # Накопленное непрерывное время покоя (с)


class CargoLoader:
    def __init__(self, config: Optional[LoaderConfig] = None):
        self.config = config or LoaderConfig()
        self.status = LoadingStatus.IDLE

        self.docked_timestamp: Optional[float] = None      # Время начала парковки
        self.dwell_start_timestamp: Optional[float] = None  # Время начала текущего непрерывного покоя
        self.initial_cargo_count: int = 0
        self.accumulated_dwell: float = 0.0

    def start_loading(self, current_time: float, current_cargo_count: int) -> None:
        """Инициализация процесса загрузки после успешной парковки."""
        self.status = LoadingStatus.WAITING
        self.docked_timestamp = current_time
        self.dwell_start_timestamp = current_time
        self.initial_cargo_count = current_cargo_count
        self.accumulated_dwell = 0.0

    def reset(self) -> None:
        """Сброс в исходное состояние IDLE."""
        self.status = LoadingStatus.IDLE
        self.docked_timestamp = None
        self.dwell_start_timestamp = None
        self.initial_cargo_count = 0
        self.accumulated_dwell = 0.0

    def update(
        self,
        current_time: float,
        current_cargo_count: int,
        linear_vel: float = 0.0,
        angular_vel: float = 0.0
    ) -> LoaderOutput:
        """Такт обновления контроллера загрузки (вызывается на каждом тике таймера)."""
        if self.status != LoadingStatus.WAITING:
            return LoaderOutput(
                status=self.status,
                linear_vel=0.0,
                angular_vel=0.0,
                tray_angle=self.config.tray_level_angle,
                dwell_time=self.accumulated_dwell
            )

        # 1. Проверка факта появления груза на датчике
        if current_cargo_count > self.initial_cargo_count:
            self.status = LoadingStatus.SUCCESS
            return LoaderOutput(
                status=self.status,
                linear_vel=0.0,
                angular_vel=0.0,
                tray_angle=self.config.tray_level_angle,
                dwell_time=self.accumulated_dwell
            )

        # 2. Проверка защитного сторожевого таймаута (Watchdog)
        elapsed_total = current_time - self.docked_timestamp
        if elapsed_total >= self.config.timeout:
            self.status = LoadingStatus.TIMEOUT
            return LoaderOutput(
                status=self.status,
                linear_vel=0.0,
                angular_vel=0.0,
                tray_angle=self.config.tray_level_angle,
                dwell_time=self.accumulated_dwell
            )

        # 3. Проверка условия неподвижности
        is_still = (abs(linear_vel) < self.config.still_lin_thresh and
                    abs(angular_vel) < self.config.still_ang_thresh)

        if is_still:
            if self.dwell_start_timestamp is None:
                self.dwell_start_timestamp = current_time
            self.accumulated_dwell = current_time - self.dwell_start_timestamp
            current_status = LoadingStatus.WAITING
        else:
            # Робот сдвинулся — обнуляем непрерывный таймер покоя (как у судьи)
            self.dwell_start_timestamp = None
            self.accumulated_dwell = 0.0
            current_status = LoadingStatus.MOVED_RESET

        return LoaderOutput(
            status=current_status,
            linear_vel=0.0,
            angular_vel=0.0,
            tray_angle=self.config.tray_level_angle,
            dwell_time=self.accumulated_dwell
        )

