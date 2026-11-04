#!/usr/bin/env python3
"""Модуль прецизионной стыковки робота с маркером (Position-Based Visual Servoing)."""

import math
from dataclasses import dataclass
from typing import Optional
import numpy as np


@dataclass
class DockingConfig:
    """Параметры П-регулятора стыковки."""
    target_dist: float = 0.65            # Целевое расстояние от камеры до маркера (м)
    dist_tolerance: float = 0.04         # Допустимая погрешность по дальности (м)
    angle_tolerance: float = 0.05        # Допустимая погрешность по углу (рад, ~2.8°)
    align_angle_thresh: float = 0.35     # Порог чистой ориентации на месте (рад, ~20°)

    kp_dist: float = 0.5                 # Пропорциональный коэффициент по дальности
    kp_angle: float = 1.4                # Пропорциональный коэффициент по углу

    max_lin_vel: float = 0.35            # Ограничение скорости вперёд (м/с)
    min_lin_vel: float = -0.12           # Ограничение скорости назад (м/с)
    max_ang_vel: float = 1.0             # Ограничение угловой скорости (рад/с)

    cam_x_offset: float = 0.178          # Вынос оптической оси камеры от центра вращения base_link (м)
    min_safe_laser_dist: float = 0.28    # Аварийная дистанция по лидару для отсечки движения вперед (м)


@dataclass
class DockingResult:
    linear_vel: float                    # Линейная скорость v (м/с)
    angular_vel: float                   # Угловая скорость w (рад/с)
    is_docked: bool                      # Флаг завершения стыковки
    dist_error: float                    # Ошибка по дальности (м)
    angle_error: float                   # Ошибка по углу (рад)


def compute_docking_cmd(
    tvec: np.ndarray,
    config: Optional[DockingConfig] = None,
    front_laser_dist: float = float('inf')
) -> DockingResult:
    """Вычисление управляющих скоростей (v, w) для стыковки к маркеру.

    Координаты tvec в СК камеры:
        X_c — вправо
        Y_c — вниз
        Z_c — вперед вдоль оптической оси
    """
    if config is None:
        config = DockingConfig()

    xc = float(tvec[0])
    zc = float(tvec[2])

    # 1. Пересчет координат маркера в горизонтальную плоскость СК робота (base_link)
    xr = zc + config.cam_x_offset
    yr = -xc

    # 2. Вычисление ошибки ориентации и дальности
    dist_to_marker = math.hypot(xr, yr)
    target_dist_from_base = config.target_dist + config.cam_x_offset

    dist_error = dist_to_marker - target_dist_from_base
    angle_error = math.atan2(yr, xr)

    # 3. Проверка условия фиксации (Deadband)
    if abs(dist_error) <= config.dist_tolerance and abs(angle_error) <= config.angle_tolerance:
        return DockingResult(
            linear_vel=0.0,
            angular_vel=0.0,
            is_docked=True,
            dist_error=dist_error,
            angle_error=angle_error
        )

    # 4. Фаза 1: Курсовое выравнивание
    if abs(angle_error) > config.align_angle_thresh:
        w_cmd = config.kp_angle * angle_error
        w_cmd = max(-config.max_ang_vel, min(config.max_ang_vel, w_cmd))
        return DockingResult(
            linear_vel=0.0,
            angular_vel=float(w_cmd),
            is_docked=False,
            dist_error=dist_error,
            angle_error=angle_error
        )

    # 5. Фаза 2: Одновременное руление и движение к целевой дистанции
    v_cmd = config.kp_dist * dist_error
    v_cmd = max(config.min_lin_vel, min(config.max_lin_vel, v_cmd))

    # Плавное снижение скорости при угле отклонения
    v_cmd *= max(0.0, math.cos(angle_error))

    w_cmd = config.kp_angle * angle_error
    w_cmd = max(-config.max_ang_vel, min(config.max_ang_vel, w_cmd))

    # 6. Защитный барьер по лидару (Safety Layer)
    if front_laser_dist < config.min_safe_laser_dist and v_cmd > 0.0:
        v_cmd = 0.0

    return DockingResult(
        linear_vel=float(v_cmd),
        angular_vel=float(w_cmd),
        is_docked=False,
        dist_error=dist_error,
        angle_error=angle_error
    )

