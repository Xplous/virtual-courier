#!/usr/bin/env python3
"""Модуль детекции станций и классификации состояния сигнальных ламп.

Геометрия сцены Webots:
- Табличка: 0.24 м; квадрат ArUco без белых полей: 0.18 м, центр на высоте z = 0.20 м.
  ID 0: зона выгрузки (финиш); ID 1: станция A; ID 2: станция B; ID 3: станция C.
- Сигнальная лампа: сфера r = 0.07 м, центр на высоте z = 0.45 м над маркером.
  Смещение относительно центра маркера в локальной СК маркера (X вправо, Y вверх, Z от стены):
  dx = 0.0 м, dy = +0.25 м, dz = +0.053 м.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np


class StationState(str, Enum):
    ACTIVE = 'ACTIVE'       # Зелёная лампа — есть груз
    ISSUED = 'ISSUED'       # Оранжевая лампа — груз уже выдан
    INACTIVE = 'INACTIVE'   # Погасшая лампа — груза нет
    UNKNOWN = 'UNKNOWN'     # Лампа вне кадра, перекрыта, слишком далеко или неоднозначно


@dataclass
class StationDetection:
    marker_id: int
    name: str
    state: StationState
    distance: float                        # Дистанция от оптического центра камеры до маркера, м
    tvec: np.ndarray                       # Вектор смещения [X, Y, Z] в СК камеры
    rvec: np.ndarray                       # Вектор вращения Родрига в СК камеры
    marker_corners: np.ndarray             # 4x2 углы маркера в пикселях
    lamp_center_px: Optional[Tuple[int, int]] = None  # (u, v) центр проекции лампы
    lamp_radius_px: int = 0                # Радиус ROI лампы в пикселях
    confidence: float = 0.0                # Доля пикселей ключевого цвета в ROI


class StationDetector:
    # Физические параметры
    MARKER_SIZE = 0.18  # 384 пикселя маркера на табличке 512 пикселей шириной 0.24 м.
    LAMP_RADIUS_M = 0.07  # м
    # 3D смещение центра лампы в локальных координатах маркера (X: вправо, Y: вверх, Z: из стены)
    LAMP_OFFSET_3D = np.array([[0.0, 0.25, 0.053]], dtype=np.float64)

    # Соответствие ID маркеров
    STATION_NAMES = {
        0: 'FINISH',
        1: 'STATION_A',
        2: 'STATION_B',
        3: 'STATION_C',
    }

    # Пороги HSV для классификации (OpenCV: H 0-179, S 0-255, V 0-255)
    HSV_GREEN_MIN = np.array([35, 90, 90], dtype=np.uint8)
    HSV_GREEN_MAX = np.array([85, 255, 255], dtype=np.uint8)

    HSV_ORANGE_MIN = np.array([5, 110, 110], dtype=np.uint8)
    HSV_ORANGE_MAX = np.array([25, 255, 255], dtype=np.uint8)

    # Пороги дистанции и достоверности
    MAX_VALID_DISTANCE_M = 3.5     # Далее 3.5 м ROI лампы слишком мал
    MIN_COLOR_RATIO = 0.15         # Минимум 15% пикселей ROI должны быть нужного цвета

    def __init__(self, fx: float = 530.35, fy: float = 530.35, cx: float = 320.0, cy: float = 240.0):
        self.camera_matrix = np.array([
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)
        self.fx = fx

        # Углы маркера в локальной 3D СК (Z=0, центр в (0,0,0)):
        # 0: верх-лево, 1: верх-право, 2: низ-право, 3: низ-лево
        hs = self.MARKER_SIZE / 2.0
        self.marker_corners_3d = np.array([
            [-hs,  hs, 0.0],
            [ hs,  hs, 0.0],
            [ hs, -hs, 0.0],
            [-hs, -hs, 0.0]
        ], dtype=np.float64)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        if hasattr(cv2.aruco, 'DetectorParameters'):
            self.aruco_params = cv2.aruco.DetectorParameters()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()

    def update_camera_info(self, k_flat: list):
        """Обновление матрицы K из sensor_msgs/CameraInfo."""
        if len(k_flat) == 9:
            self.camera_matrix = np.array(k_flat, dtype=np.float64).reshape((3, 3))
            self.fx = float(self.camera_matrix[0, 0])

    def detect(self, bgr_image: np.ndarray) -> List[StationDetection]:
        """Обнаружение маркеров и классификация состояния ламп."""
        if bgr_image is None or bgr_image.size == 0:
            return []

        h_img, w_img = bgr_image.shape[:2]
        gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

        results: List[StationDetection] = []
        if ids is None or len(ids) == 0:
            return results

        for corner, marker_id in zip(corners, ids.ravel()):
            mid = int(marker_id)
            if mid not in self.STATION_NAMES:
                continue

            pts_2d = corner.reshape(4, 2).astype(np.float64)

            # PnP: определение 3D-позы маркера относительно камеры
            success, rvec, tvec = cv2.solvePnP(
                self.marker_corners_3d,
                pts_2d,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE if hasattr(cv2, 'SOLVEPNP_IPPE_SQUARE') else cv2.SOLVEPNP_ITERATIVE
            )
            if not success:
                continue

            distance = float(np.linalg.norm(tvec))

            # ID 0 — зона выгрузки (лампы у неё нет по регламенту)
            if mid == 0:
                results.append(StationDetection(
                    marker_id=mid,
                    name=self.STATION_NAMES[mid],
                    state=StationState.UNKNOWN,
                    distance=distance,
                    tvec=tvec.ravel(),
                    rvec=rvec.ravel(),
                    marker_corners=pts_2d,
                ))
                continue

            # Станции 1, 2, 3: репроекция центра лампы
            proj_pts, _ = cv2.projectPoints(
                self.LAMP_OFFSET_3D, rvec, tvec, self.camera_matrix, self.dist_coeffs
            )
            u_lamp, v_lamp = int(round(proj_pts[0, 0, 0])), int(round(proj_pts[0, 0, 1]))

            # Оценка радиуса лампы в пикселях
            radius_px = max(4, int(round((self.fx * self.LAMP_RADIUS_M) / max(0.2, distance))))

            state, conf = self._classify_lamp(bgr_image, u_lamp, v_lamp, radius_px, distance, w_img, h_img)

            results.append(StationDetection(
                marker_id=mid,
                name=self.STATION_NAMES[mid],
                state=state,
                distance=distance,
                tvec=tvec.ravel(),
                rvec=rvec.ravel(),
                marker_corners=pts_2d,
                lamp_center_px=(u_lamp, v_lamp),
                lamp_radius_px=radius_px,
                confidence=conf,
            ))

        return results

    def _classify_lamp(
        self,
        bgr_image: np.ndarray,
        u: int,
        v: int,
        r: int,
        distance: float,
        w_img: int,
        h_img: int
    ) -> Tuple[StationState, float]:
        """Анализ ROI лампы в пространстве HSV."""
        # 1. Проверка дальности: если робот далеко, ROI вырождается в шум
        if distance > self.MAX_VALID_DISTANCE_M:
            return StationState.UNKNOWN, 0.0

        # 2. Проверка выхода ROI за пределы кадра (особенно вверх при близком подъезде)
        if (u - r < 0) or (u + r >= w_img) or (v - r < 0) or (v + r >= h_img):
            return StationState.UNKNOWN, 0.0

        roi_bgr = bgr_image[v - r:v + r + 1, u - r:u + r + 1]
        if roi_bgr.size == 0:
            return StationState.UNKNOWN, 0.0

        # Создаём круглую маску для шара лампы
        roi_h, roi_w = roi_bgr.shape[:2]
        cy, cx = roi_h // 2, roi_w // 2
        y_grid, x_grid = np.ogrid[:roi_h, :roi_w]
        circle_mask = ((x_grid - cx) ** 2 + (y_grid - cy) ** 2) <= (r ** 2)
        total_pixels = int(np.count_nonzero(circle_mask))
        if total_pixels < 8:
            return StationState.UNKNOWN, 0.0

        roi_hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)

        # Маски цветов
        green_mask = cv2.inRange(roi_hsv, self.HSV_GREEN_MIN, self.HSV_GREEN_MAX) & circle_mask
        orange_mask = cv2.inRange(roi_hsv, self.HSV_ORANGE_MIN, self.HSV_ORANGE_MAX) & circle_mask

        green_ratio = np.count_nonzero(green_mask) / total_pixels
        orange_ratio = np.count_nonzero(orange_mask) / total_pixels

        # Средняя яркость и насыщенность в круге
        v_mean = float(np.mean(roi_hsv[circle_mask, 2]))
        s_mean = float(np.mean(roi_hsv[circle_mask, 1]))

        if green_ratio >= self.MIN_COLOR_RATIO and green_ratio > orange_ratio:
            return StationState.ACTIVE, float(green_ratio)

        if orange_ratio >= self.MIN_COLOR_RATIO and orange_ratio > green_ratio:
            return StationState.ISSUED, float(orange_ratio)

        # Если лампа выключена (PBR черная/темно-серая: V низкий и S низкий)
        if v_mean < 85.0 and s_mean < 60.0:
            conf = max(0.0, 1.0 - (v_mean / 85.0))
            return StationState.INACTIVE, float(conf)

        return StationState.UNKNOWN, 0.0

    def draw_detections(self, image: np.ndarray, detections: List[StationDetection]) -> np.ndarray:
        """Отрисовка оверлея детекции на кадре."""
        vis = image.copy()
        for det in detections:
            # Контур маркера
            pts = det.marker_corners.astype(np.int32)
            cv2.polylines(vis, [pts], isClosed=True, color=(0, 255, 255), thickness=2)

            # Текст статуса
            cx, cy = int(pts[:, 0].mean()), int(pts[:, 1].mean())
            state_colors = {
                StationState.ACTIVE: (0, 255, 0),      # Зелёный
                StationState.ISSUED: (0, 140, 255),    # Оранжевый
                StationState.INACTIVE: (100, 100, 100),# Серый
                StationState.UNKNOWN: (0, 0, 255),     # Красный/Неизвестно
            }
            color = state_colors.get(det.state, (255, 255, 255))
            label = f'{det.name}: {det.state.value} ({det.distance:.2f}m)'
            cv2.putText(vis, label, (cx - 60, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # Проекция лампы
            if det.lamp_center_px is not None and det.lamp_radius_px > 0:
                lu, lv = det.lamp_center_px
                lr = det.lamp_radius_px
                # Если в кадре — рисуем круг
                if 0 <= lu < vis.shape[1] and 0 <= lv < vis.shape[0]:
                    cv2.circle(vis, (lu, lv), lr, color, 2)
                    cv2.circle(vis, (lu, lv), 2, color, -1)

        return vis

