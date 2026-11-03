#!/usr/bin/env python3

from copy import copy

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile

from maze_grid_node import MazeGrid


def suppress_pallet_shadows(cells, resolution, map_origin, map_rotation,
                            grid_origin, grid_rotation):
    occupied = (cells >= 65).astype(np.uint8)
    _, labels, stats, _ = cv2.connectedComponentsWithStats(occupied, connectivity=8)
    pallets = set()
    for label, (x, y, width, height, count) in enumerate(stats[1:], 1):
        if count < 8 or not (0.25 <= width*resolution <= 0.65 and
                             0.25 <= height*resolution <= 0.65):
            continue
        yy, xx = np.nonzero(labels[y:y+height, x:x+width] == label)
        points = (np.column_stack((xx+x, yy+y))+0.5)*resolution
        local = (points @ map_rotation.T+map_origin-grid_origin) @ grid_rotation
        cell = np.floor(local.mean(axis=0)/2+0.5).astype(int)
        size = np.ptp(local, axis=0)+resolution
        if (np.all((size >= 0.25) & (size <= 0.65)) and
                np.max(np.abs(local.mean(axis=0)-2*cell)) <= 0.35 and
                np.max(np.abs(local-2*cell)) <= 0.65):
            pallets.add(tuple(cell))

    result = cells.copy()
    yy, xx = np.nonzero(cells < 0)
    points = (np.column_stack((xx, yy))+0.5)*resolution
    local = (points @ map_rotation.T+map_origin-grid_origin) @ grid_rotation
    for cell in pallets:
        interior = np.max(np.abs(local-2*np.asarray(cell)), axis=1) < 0.65
        # Это запрет целей только для explore_lite; Nav2 читает исходную /map.
        result[yy[interior], xx[interior]] = 100
    return result, pallets


class ExplorationMap(MazeGrid):
    def __init__(self):
        super().__init__()
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.filtered_pub = self.create_publisher(OccupancyGrid, '/exploration_map', qos)

    def on_map(self, msg):
        super().on_map(msg)
        filtered, pallets = suppress_pallet_shadows(
            self.cells, self.resolution, self.map_origin, self.map_rotation,
            self.grid_origin, self.grid_rotation) if self.fitted else (self.cells, set())
        output = copy(msg)
        output.data = filtered.ravel().tolist()
        self.filtered_pub.publish(output)
        hidden = np.count_nonzero((self.cells < 0) & (filtered >= 0))
        self.get_logger().info(
            f'Паллетов: {len(pallets)}; исключено тени: {hidden*self.resolution**2:.2f} м²',
            throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = ExplorationMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
