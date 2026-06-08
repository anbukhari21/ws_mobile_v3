#!/usr/bin/env python3
"""Short-lived LiDAR obstacle memory for the route + SAC safety layer."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Float32MultiArray


@dataclass
class MemoryPoint:
    x: float
    y: float
    last_seen: float
    observations: int = 1


class ObstacleMemoryNode(Node):
    def __init__(self) -> None:
        super().__init__("obstacle_memory_node")
        self.declare_parameter("memory_timeout_sec", 1.5)
        self.declare_parameter("cluster_radius", 0.10)
        self.declare_parameter("min_valid_range", 0.05)
        self.declare_parameter("max_memory_range", 1.2)
        self.declare_parameter("front_half_angle_deg", 35.0)
        self.declare_parameter("publish_pointcloud", True)

        self.timeout = float(self.get_parameter("memory_timeout_sec").value)
        self.cluster_radius = float(self.get_parameter("cluster_radius").value)
        self.min_valid_range = float(self.get_parameter("min_valid_range").value)
        self.max_memory_range = float(self.get_parameter("max_memory_range").value)
        self.front_half_angle = math.radians(float(self.get_parameter("front_half_angle_deg").value))
        self.publish_pointcloud = bool(self.get_parameter("publish_pointcloud").value)
        self.memory: list[MemoryPoint] = []

        self.create_subscription(LaserScan, "/scan", self._scan_cb, 10)
        self._dist_pub = self.create_publisher(Float32MultiArray, "/safety/min_memory_distances", 10)
        self._cloud_pub = self.create_publisher(PointCloud2, "/safety/obstacle_memory_points", 10)

    def _scan_cb(self, msg: LaserScan) -> None:
        now = time.monotonic()
        angle = float(msg.angle_min)
        live_points: list[tuple[float, float]] = []
        for raw in msg.ranges:
            r = float(raw)
            if math.isfinite(r) and self.min_valid_range <= r <= self.max_memory_range:
                live_points.append((r * math.cos(angle), r * math.sin(angle)))
            angle += float(msg.angle_increment)
        for x, y in live_points:
            self._upsert(x, y, now)
        self.memory = [p for p in self.memory if now - p.last_seen <= self.timeout]
        self._publish_distances(live_points, msg.header.frame_id or "base_footprint")
        if self.publish_pointcloud:
            self._publish_cloud(msg.header.frame_id or "base_footprint")

    def _upsert(self, x: float, y: float, now: float) -> None:
        best: MemoryPoint | None = None
        best_dist = float("inf")
        for point in self.memory:
            dist = math.hypot(point.x - x, point.y - y)
            if dist < best_dist:
                best = point
                best_dist = dist
        if best is not None and best_dist <= self.cluster_radius:
            alpha = 1.0 / float(min(best.observations + 1, 8))
            best.x = (1.0 - alpha) * best.x + alpha * x
            best.y = (1.0 - alpha) * best.y + alpha * y
            best.last_seen = now
            best.observations += 1
            return
        self.memory.append(MemoryPoint(x=x, y=y, last_seen=now))

    def _publish_distances(self, live_points: list[tuple[float, float]], frame_id: str) -> None:
        del frame_id
        points = live_points + [(p.x, p.y) for p in self.memory]
        front = left = right = overall = self.max_memory_range
        for x, y in points:
            dist = math.hypot(x, y)
            angle = math.atan2(y, x)
            overall = min(overall, dist)
            if abs(angle) <= self.front_half_angle:
                front = min(front, dist)
            elif angle > 0.0:
                left = min(left, dist)
            else:
                right = min(right, dist)
        msg = Float32MultiArray()
        msg.data = [front, left, right, overall]
        self._dist_pub.publish(msg)

    def _publish_cloud(self, frame_id: str) -> None:
        cloud = PointCloud2()
        cloud.header.frame_id = frame_id
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.height = 1
        cloud.width = len(self.memory)
        cloud.is_bigendian = False
        cloud.is_dense = True
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        import struct

        cloud.data = b"".join(struct.pack("<fff", float(p.x), float(p.y), 0.0) for p in self.memory)
        self._cloud_pub.publish(cloud)


def main() -> None:
    rclpy.init()
    node = ObstacleMemoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
