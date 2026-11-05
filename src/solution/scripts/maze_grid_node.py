#!/usr/bin/env python3

import math

import cv2
import numpy as np
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


class MazeGrid(Node):
    def __init__(self):
        super().__init__("maze_grid")
        self.angle = 0.0
        self.offset = np.zeros(2)
        self.fitted = False
        self.tf = TransformBroadcaster(self)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.edges_pub = self.create_publisher(MarkerArray, "/maze_edges", qos)
        self.map_subscription = self.create_subscription(
            OccupancyGrid, "/map", self.on_map, qos
        )

    def on_map(self, msg):
        self.resolution = msg.info.resolution
        self.cells = np.asarray(msg.data).reshape(msg.info.height, msg.info.width)
        p = msg.info.origin.position
        q = msg.info.origin.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.map_origin = np.array([p.x, p.y])
        self.map_rotation = self.rotation(yaw)
        self.fit_grid()
        self.grid_rotation = self.rotation(self.angle)
        self.grid_origin = self.offset @ self.grid_rotation.T
        stamp = self.get_clock().now().to_msg()
        transform = TransformStamped()
        transform.header.frame_id = msg.header.frame_id
        transform.header.stamp = stamp
        transform.child_frame_id = "maze_grid"
        transform.transform.translation.x = float(self.grid_origin[0])
        transform.transform.translation.y = float(self.grid_origin[1])
        transform.transform.rotation.z = math.sin(self.angle / 2)
        transform.transform.rotation.w = math.cos(self.angle / 2)
        self.tf.sendTransform(transform)
        self.publish_edges(stamp)

    @staticmethod
    def rotation(angle):
        return np.array(
            [[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]]
        )

    def fit_grid(self):
        occupied = (self.cells >= 65).astype(np.uint8) * 255
        lines = cv2.HoughLinesP(
            occupied,
            1,
            np.pi / 720,
            threshold=max(12, round(0.6 / self.resolution)),
            minLineLength=1.0 / self.resolution,
            maxLineGap=0.15 / self.resolution,
        )
        if lines is None:
            return
        endpoints = (lines[:, 0].reshape(-1, 2, 2) + 0.5) * self.resolution
        endpoints = endpoints @ self.map_rotation.T + self.map_origin
        vectors = endpoints[:, 1] - endpoints[:, 0]
        lengths = np.linalg.norm(vectors, axis=1)
        angles = np.arctan2(vectors[:, 1], vectors[:, 0])
        angle = (
            math.atan2(
                np.sum(lengths * np.sin(4 * angles)),
                np.sum(lengths * np.cos(4 * angles)),
            )
            / 4
        )
        if abs(angle) > math.radians(15):
            return
        errors = (
            np.arctan2(np.sin(4 * (angles - angle)), np.cos(4 * (angles - angle))) / 4
        )
        aligned = np.abs(errors) <= math.radians(3)
        vertical = np.abs(np.sin(angles - angle)) > 0.7
        both_axes = np.any(vertical & aligned) and np.any(~vertical & aligned)
        if lengths[aligned].sum() < 4.0 and not both_axes:
            return
        angle = (
            math.atan2(
                np.sum(lengths[aligned] * np.sin(4 * angles[aligned])),
                np.sum(lengths[aligned] * np.cos(4 * angles[aligned])),
            )
            / 4
        )
        mids = endpoints.mean(axis=1) @ self.rotation(angle)
        offset = self.offset.copy()
        for axis, selected in enumerate((vertical & aligned, ~vertical & aligned)):
            if not np.any(selected):
                continue
            phase = np.pi * (mids[selected, axis] - 1.0)
            weights = lengths[selected]
            offset[axis] = (
                math.atan2(
                    np.sum(weights * np.sin(phase)), np.sum(weights * np.cos(phase))
                )
                / np.pi
            )
        if self.fitted:
            self.angle += 0.2 * (angle - self.angle)
            delta = (offset - self.offset + 1.0) % 2.0 - 1.0
            self.offset += 0.2 * delta
        else:
            self.angle = angle
            self.offset = offset
            self.fitted = True

    def sample(self, grid_points):
        world = grid_points @ self.grid_rotation.T + self.grid_origin
        pixels = np.floor(
            (world - self.map_origin) @ self.map_rotation / self.resolution
        ).astype(int)
        x, y = pixels[..., 0], pixels[..., 1]
        inside = (
            (x >= 0) & (x < self.cells.shape[1]) & (y >= 0) & (y < self.cells.shape[0])
        )
        values = np.full(x.shape, -1, dtype=np.int16)
        values[inside] = self.cells[y[inside], x[inside]]
        return values

    def classify_edge(self, center, vertical):
        along = np.linspace(-0.8, 0.8, max(2, math.ceil(1.6 / self.resolution) + 1))
        normal = np.linspace(-0.4, 0.4, max(3, math.ceil(0.8 / self.resolution) + 1))
        aa, nn = np.meshgrid(along, normal, indexing="ij")
        points = np.stack((nn, aa) if vertical else (aa, nn), axis=-1) + center
        values = self.sample(points)
        wall = (
            np.any(
                values[:, np.abs(normal) <= 0.15 + self.resolution / 2] >= 65, axis=1
            ).mean()
            >= 0.65
        )
        clear = np.all(values == 0, axis=1)
        longest = run = 0
        for free in clear:
            run = run + 1 if free else 0
            longest = max(longest, run)
        opened = max(0, longest - 1) * (along[1] - along[0]) >= 0.5
        if wall and not opened:
            return "wall"
        if opened and not wall:
            return "open"
        return "unknown"

    def publish_edges(self, stamp):
        corners = np.array(
            [
                [0, 0],
                [self.cells.shape[1], 0],
                [0, self.cells.shape[0]],
                [self.cells.shape[1], self.cells.shape[0]],
            ]
        )
        world = corners * self.resolution @ self.map_rotation.T + self.map_origin
        local = (world - self.grid_origin) @ self.grid_rotation
        low = np.floor(local.min(axis=0) / 2).astype(int) - 1
        high = np.ceil(local.max(axis=0) / 2).astype(int) + 1
        colors = {
            "wall": (1.0, 0.15, 0.1, 0.9),
            "open": (0.1, 1.0, 0.2, 0.8),
            "unknown": (0.6, 0.6, 0.6, 0.35),
        }
        markers = {}
        counts = dict.fromkeys(colors, 0)
        self.edges = {}
        for i, (state, color) in enumerate(colors.items()):
            marker = Marker()
            marker.header.frame_id = "maze_grid"
            marker.header.stamp = stamp
            marker.ns = "maze_edges"
            marker.id = i
            marker.type = Marker.LINE_LIST
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.04
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
            marker.lifetime.sec = 3
            markers[state] = marker
        for i in range(low[0], high[0] + 1):
            for j in range(low[1], high[1] + 1):
                for vertical in (True, False):
                    center = np.array(
                        [2 * i + 1, 2 * j] if vertical else [2 * i, 2 * j + 1],
                        dtype=float,
                    )
                    state = self.classify_edge(center, vertical)
                    self.edges[("v" if vertical else "h", i, j)] = state
                    counts[state] += 1
                    delta = np.array([0, 1] if vertical else [1, 0])
                    for point in (center - delta, center + delta):
                        markers[state].points.append(
                            Point(x=float(point[0]), y=float(point[1]), z=0.08)
                        )
        for marker in markers.values():
            marker.action = Marker.ADD if marker.points else Marker.DELETE
        self.edges_pub.publish(MarkerArray(markers=list(markers.values())))
        self.get_logger().info(
            f'Сетка ({"по стенам" if self.fitted else "начальная"}): угол {math.degrees(self.angle):.2f}°, '
            f"смещение ({self.grid_origin[0]:.2f}, {self.grid_origin[1]:.2f}) м; "
            f'стен {counts["wall"]}, проходов {counts["open"]}, неизвестно {counts["unknown"]}',
            throttle_duration_sec=5.0,
        )
