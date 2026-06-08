#!/usr/bin/env python3
"""Publish SAC lookahead targets and route markers from an autonomous route."""

from __future__ import annotations

import math
from typing import Optional

import rclpy
import rclpy.duration
import rclpy.time
import tf2_ros
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

from mobile_robot.rl.sac_policy_v2.geometry import (
    compute_path_state,
    normalize_path,
    point_at_progress,
    total_path_length,
)

from mobile_robot.sac_navigation.utils import path_to_points, points_to_path, yaw_to_quaternion


class LocalTargetNode(Node):
    def __init__(self) -> None:
        super().__init__("local_target_node")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("lookahead_distance", 0.25)
        self.declare_parameter("local_path_length", 1.0)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("marker_lifetime_sec", 0.5)
        self.declare_parameter("goal_x", 0.0)
        self.declare_parameter("goal_y", 0.0)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.lookahead_distance = float(self.get_parameter("lookahead_distance").value)
        self.local_path_length = float(self.get_parameter("local_path_length").value)
        self.publish_markers = bool(self.get_parameter("publish_markers").value)
        self.marker_lifetime_sec = float(self.get_parameter("marker_lifetime_sec").value)
        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._path_points: list[tuple[float, float]] = []

        self.create_subscription(Path, "/plan", self._path_cb, 10)
        self._lookahead_pub = self.create_publisher(PoseStamped, "/sac/lookahead_pose", 10)
        self._features_pub = self.create_publisher(Float32MultiArray, "/sac/route_features", 10)
        self._local_path_pub = self.create_publisher(Path, "/sac/local_route", 10)
        self._marker_pub = self.create_publisher(MarkerArray, "/sac_navigation/markers", 10)
        self.create_timer(0.1, self._tick)

    def _path_cb(self, msg: Path) -> None:
        try:
            self._path_points = normalize_path(path_to_points(msg))
        except ValueError:
            self.get_logger().warn("Ignoring invalid route with fewer than two distinct points.", throttle_duration_sec=5.0)

    def _robot_pose(self) -> Optional[tuple[float, float, float]]:
        try:
            transform = self._tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.02),
            )
        except Exception as exc:
            self.get_logger().warn(f"Waiting for {self.map_frame}->{self.base_frame} TF: {exc}", throttle_duration_sec=5.0)
            return None
        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return (float(t.x), float(t.y), yaw)

    def _local_path_window(self, progress: float) -> list[tuple[float, float]]:
        if len(self._path_points) < 2:
            return []
        total = total_path_length(self._path_points)
        start = max(0.0, min(float(progress), total))
        end = min(total, start + max(self.lookahead_distance, self.local_path_length))
        samples = max(2, min(20, int(math.ceil((end - start) / 0.10)) + 1))
        return [point_at_progress(self._path_points, start + (end - start) * i / (samples - 1)) for i in range(samples)]

    def _tick(self) -> None:
        if len(self._path_points) < 2:
            return
        pose = self._robot_pose()
        if pose is None:
            return
        x, y, yaw = pose
        state = compute_path_state(
            self._path_points,
            x,
            y,
            yaw,
            lookahead_distance=self.lookahead_distance,
        )
        stamp = self.get_clock().now().to_msg()
        target = PoseStamped()
        target.header.frame_id = self.map_frame
        target.header.stamp = stamp
        target.pose.position.x = float(state.lookahead_point[0])
        target.pose.position.y = float(state.lookahead_point[1])
        target.pose.orientation = yaw_to_quaternion(math.atan2(state.lookahead_point[1] - y, state.lookahead_point[0] - x))
        self._lookahead_pub.publish(target)

        total = max(state.total_length, 1e-9)
        features = Float32MultiArray()
        features.data = [
            float(state.lookahead_distance),
            float(state.lookahead_angle),
            float(state.signed_cross_track_error),
            float(state.abs_cross_track_error),
            float(state.progress_fraction),
            float(state.remaining_distance),
            float(state.path_heading_error),
            float(state.final_distance),
            float(state.segment_index),
            float(total),
        ]
        self._features_pub.publish(features)
        local_points = self._local_path_window(state.progress)
        self._local_path_pub.publish(points_to_path(local_points, frame_id=self.map_frame, stamp=stamp))
        if self.publish_markers:
            self._marker_pub.publish(self._markers(stamp, local_points, state.lookahead_point))

    def _line_marker(self, marker_id: int, ns: str, points: list[tuple[float, float]], color: tuple[float, float, float, float], scale: float) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.lifetime.sec = int(self.marker_lifetime_sec)
        marker.lifetime.nanosec = int((self.marker_lifetime_sec % 1.0) * 1e9)
        for x, y in points:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.03
            marker.points.append(p)
        return marker

    def _sphere_marker(self, marker_id: int, ns: str, x: float, y: float, color: tuple[float, float, float, float], scale: float) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = scale * 0.5
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.lifetime.sec = int(self.marker_lifetime_sec)
        marker.lifetime.nanosec = int((self.marker_lifetime_sec % 1.0) * 1e9)
        return marker

    def _text_marker(self, marker_id: int, ns: str, x: float, y: float, text: str) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = 0.35
        marker.scale.z = 0.16
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = text
        marker.lifetime.sec = int(self.marker_lifetime_sec)
        marker.lifetime.nanosec = int((self.marker_lifetime_sec % 1.0) * 1e9)
        return marker

    def _markers(self, stamp, local_points: list[tuple[float, float]], lookahead: tuple[float, float]) -> MarkerArray:
        del stamp
        arr = MarkerArray()
        arr.markers.append(self._line_marker(1, "global_route", self._path_points, (0.1, 1.0, 0.1, 0.9), 0.025))
        arr.markers.append(self._line_marker(2, "sac_active_route", local_points, (0.0, 0.9, 1.0, 0.95), 0.045))
        arr.markers.append(self._sphere_marker(3, "center_goal", self.goal_x, self.goal_y, (1.0, 0.0, 0.0, 1.0), 0.12))
        arr.markers.append(self._sphere_marker(4, "lookahead", lookahead[0], lookahead[1], (0.0, 0.9, 1.0, 1.0), 0.10))
        arr.markers.append(self._text_marker(5, "center_goal_text", self.goal_x, self.goal_y, "CENTER GOAL"))
        return arr


def main() -> None:
    rclpy.init()
    node = LocalTargetNode()
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
