#!/usr/bin/env python3
"""Publish a passive RViz circle at the map center."""

import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker


class CenterSuccessMarkerNode(Node):
    def __init__(self) -> None:
        super().__init__("center_success_marker_node")

        self.declare_parameter("marker_topic", "/center_success_radius_marker")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("center_x", 0.0)
        self.declare_parameter("center_y", 0.0)
        self.declare_parameter("center_z", 0.04)
        self.declare_parameter("radius", 0.22)
        self.declare_parameter("line_width", 0.025)
        self.declare_parameter("segments", 96)
        self.declare_parameter("publish_period_sec", 1.0)

        topic = str(self.get_parameter("marker_topic").value)
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(Marker, topic, qos)

        period = max(0.1, float(self.get_parameter("publish_period_sec").value))
        self.timer = self.create_timer(period, self.publish_marker)
        self.publish_marker()

        self.get_logger().info(
            "Publishing 0.22 m center success circle on "
            f"{topic} in frame {self.get_parameter('frame_id').value}"
        )

    def publish_marker(self) -> None:
        radius = max(0.0, float(self.get_parameter("radius").value))
        center_x = float(self.get_parameter("center_x").value)
        center_y = float(self.get_parameter("center_y").value)
        center_z = float(self.get_parameter("center_z").value)
        segments = max(12, int(self.get_parameter("segments").value))

        marker = Marker()
        marker.header.frame_id = str(self.get_parameter("frame_id").value)
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "center_success_radius"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = max(0.001, float(self.get_parameter("line_width").value))
        marker.color.r = 1.0
        marker.color.g = 0.15
        marker.color.b = 0.0
        marker.color.a = 1.0

        for index in range(segments + 1):
            theta = 2.0 * math.pi * float(index) / float(segments)
            point = Point()
            point.x = center_x + radius * math.cos(theta)
            point.y = center_y + radius * math.sin(theta)
            point.z = center_z
            marker.points.append(point)

        self.publisher.publish(marker)


def main() -> None:
    rclpy.init()
    node = CenterSuccessMarkerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
