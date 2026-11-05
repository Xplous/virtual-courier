#!/usr/bin/env python3
"""Преобразование сканов в odom без асинхронного ожидания TF в costmap."""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener


class CostmapCloud(Node):
    def __init__(self):
        super().__init__("costmap_cloud")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.scan = None
        self.publisher = self.create_publisher(PointCloud2, "/costmap/points", 10)
        self.create_subscription(
            LaserScan, "/scan", self.on_scan, qos_profile_sensor_data
        )
        self.create_timer(0.02, self.publish_cloud)

    def on_scan(self, msg):
        self.scan = msg

    def publish_cloud(self):
        scan = self.scan
        if scan is None:
            return
        stamp = rclpy.time.Time.from_msg(scan.header.stamp)
        if (self.get_clock().now() - stamp).nanoseconds > 200_000_000:
            self.scan = None
            return
        try:
            transform = self.buffer.lookup_transform(
                "odom", scan.header.frame_id, stamp
            ).transform
        except TransformException:
            return
        self.scan = None
        ranges = np.asarray(scan.ranges)
        valid = (
            np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges < scan.range_max)
        )
        angles = scan.angle_min + np.flatnonzero(valid) * scan.angle_increment
        ranges = ranges[valid]
        points = np.column_stack(
            (ranges * np.cos(angles), ranges * np.sin(angles), np.zeros_like(ranges))
        )
        q = transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        rotation = np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ]
        )
        t = transform.translation
        points = np.asarray(points @ rotation.T + [t.x, t.y, t.z], dtype="<f4")
        # Точки уже в odom; нулевая метка запрашивает последний доступный TF.
        cloud = PointCloud2(
            header=Header(frame_id="odom"),
            height=1,
            width=len(points),
            fields=[
                PointField(
                    name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1
                )
                for i, name in enumerate(("x", "y", "z"))
            ],
            is_bigendian=False,
            point_step=12,
            row_step=12 * len(points),
            data=points.tobytes(),
            is_dense=True,
        )
        self.publisher.publish(cloud)


def main():
    rclpy.init()
    node = CostmapCloud()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
