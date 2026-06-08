#!/usr/bin/env python3
"""SAC runtime that consumes an autonomous route and publishes raw cmd_vel."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
import rclpy.duration
import rclpy.time
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from stable_baselines3 import SAC
from std_msgs.msg import Bool, String

from mobile_robot.rl.sac_policy_v2.geometry import compute_path_state, normalize_path

from mobile_robot.sac_navigation.utils import path_to_points


class SacMotionNode(Node):
    def __init__(self) -> None:
        super().__init__("sac_motion_node")
        self.declare_parameter(
            "model_path",
            str(Path.home() / "ws_mobile_v3" / "rl_models" / "sac_goal_policy.zip"),
        )
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("lookahead_distance", 0.35)
        self.declare_parameter("max_linear", 0.12)
        self.declare_parameter("max_angular", 0.30)
        self.declare_parameter("final_radius", 0.14)
        self.declare_parameter("goal_x", 0.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("lidar_sector_count", 8)
        self.declare_parameter("lidar_max_range", 3.5)
        self.declare_parameter("control_rate_hz", 10.0)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.lookahead_distance = float(self.get_parameter("lookahead_distance").value)
        self.max_linear = float(self.get_parameter("max_linear").value)
        self.max_angular = float(self.get_parameter("max_angular").value)
        self.final_radius = float(self.get_parameter("final_radius").value)
        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)
        self.lidar_sector_count = int(self.get_parameter("lidar_sector_count").value)
        self.lidar_max_range = float(self.get_parameter("lidar_max_range").value)

        model_path = Path(str(self.get_parameter("model_path").value)).expanduser()
        self.get_logger().info(f"Loading SAC navigation model: {model_path}")
        self.model = SAC.load(str(model_path))

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self.path_points: list[tuple[float, float]] = []
        self.scan: Optional[LaserScan] = None
        self.odom: Optional[Odometry] = None
        self.previous_progress = 0.0
        self.previous_cte = 0.0
        self.previous_cmd_linear = 0.0
        self.previous_cmd_angular = 0.0
        self.previous_raw_action = np.zeros(2, dtype=np.float32)
        self._goal_reached = False
        self._motion_enabled = True

        self.create_subscription(NavPath, "/plan", self._path_cb, 10)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, 10)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.create_subscription(Bool, "/sac/enable_motion", self._enable_motion_cb, 10)
        self._cmd_pub = self.create_publisher(Twist, "/sac_cmd_vel_raw", 10)
        self._state_pub = self.create_publisher(String, "/sac/motion_state", 10)

        rate = max(1.0, float(self.get_parameter("control_rate_hz").value))
        self.create_timer(1.0 / rate, self._control_tick)
        self._publish_schema_once()

    def _publish_schema_once(self) -> None:
        schema = [
            "lookahead_distance_norm",
            "lookahead_angle_norm",
            "sin_lookahead_angle",
            "cos_lookahead_angle",
            "signed_cross_track_error_norm",
            "abs_cross_track_error_norm",
            "cross_track_error_delta_norm",
            "path_heading_error_norm",
            "sin_path_heading_error",
            "cos_path_heading_error",
            "path_progress_fraction",
            "remaining_path_distance_norm",
            "distance_to_segment_end_norm",
            "upcoming_turn_angle_norm",
            "upcoming_turn_direction",
            "previous_cmd_linear_norm",
            "previous_cmd_angular_norm",
            "previous_raw_action_linear",
            "previous_raw_action_angular",
            "linear_speed_norm",
            "angular_speed_norm",
            "distance_to_final_waypoint_norm",
            "current_segment_index_norm",
        ] + [f"lidar_sector_{i}_norm" for i in range(self.lidar_sector_count)]
        self.get_logger().info("SAC navigation observation schema:")
        for index, name in enumerate(schema):
            self.get_logger().info(f"  {index:02d}: {name}")

    def _path_cb(self, msg: NavPath) -> None:
        try:
            points = normalize_path(path_to_points(msg))
        except ValueError:
            self.get_logger().warn("Ignoring invalid route with fewer than two distinct points.", throttle_duration_sec=5.0)
            return
        if len(points) >= 2:
            self.path_points = points
            self._goal_reached = False
            self.previous_progress = 0.0
            self.previous_cte = 0.0
            self.previous_cmd_linear = 0.0
            self.previous_cmd_angular = 0.0
            self.previous_raw_action = np.zeros(2, dtype=np.float32)
        else:
            self.path_points = []

    def _scan_cb(self, msg: LaserScan) -> None:
        self.scan = msg

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom = msg

    def _enable_motion_cb(self, msg: Bool) -> None:
        self._motion_enabled = bool(msg.data)
        if not self._motion_enabled:
            self._goal_reached = False
            self.previous_cmd_linear = 0.0
            self.previous_cmd_angular = 0.0
            self._publish_state("MOTION_DISABLED")
            self._publish_cmd(0.0, 0.0)

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

    def _lidar_sectors(self) -> list[float]:
        if self.scan is None or not self.scan.ranges:
            return [1.0] * self.lidar_sector_count
        ranges = np.asarray(self.scan.ranges, dtype=np.float32)
        ranges = np.where(np.isfinite(ranges), ranges, self.lidar_max_range)
        ranges = np.clip(ranges, 0.0, self.lidar_max_range)
        chunks = np.array_split(ranges, self.lidar_sector_count)
        return [float(np.min(chunk) / self.lidar_max_range) if len(chunk) else 1.0 for chunk in chunks]

    def _obs(self, pose: tuple[float, float, float]) -> tuple[np.ndarray, object]:
        x, y, yaw = pose
        state = compute_path_state(
            self.path_points,
            x,
            y,
            yaw,
            lookahead_distance=self.lookahead_distance,
            previous_progress=self.previous_progress,
            previous_cross_track_error=self.previous_cte,
        )
        total = max(float(state.total_length), 1e-9)
        lin_speed = float(self.odom.twist.twist.linear.x) if self.odom is not None else 0.0
        ang_speed = float(self.odom.twist.twist.angular.z) if self.odom is not None else 0.0
        obs = [
            np.clip(state.lookahead_distance / 1.5, -1.0, 1.0),
            np.clip(state.lookahead_angle / math.pi, -1.0, 1.0),
            math.sin(state.lookahead_angle),
            math.cos(state.lookahead_angle),
            np.clip(state.signed_cross_track_error / 0.30, -1.0, 1.0),
            np.clip(state.abs_cross_track_error / 0.30, 0.0, 1.0),
            np.clip(state.cross_track_error_delta / 0.10, -1.0, 1.0),
            np.clip(state.path_heading_error / math.pi, -1.0, 1.0),
            math.sin(state.path_heading_error),
            math.cos(state.path_heading_error),
            np.clip(state.progress_fraction, 0.0, 1.0),
            np.clip(state.remaining_distance / total, 0.0, 1.0),
            np.clip(state.distance_to_segment_end / total, 0.0, 1.0),
            np.clip(state.upcoming_turn_angle / math.pi, -1.0, 1.0),
            state.upcoming_turn_direction,
            np.clip(self.previous_cmd_linear / max(self.max_linear, 1e-9), 0.0, 1.0),
            np.clip(self.previous_cmd_angular / max(self.max_angular, 1e-9), -1.0, 1.0),
            float(self.previous_raw_action[0]),
            float(self.previous_raw_action[1]),
            np.clip(lin_speed / max(self.max_linear, 1e-9), -1.0, 1.0),
            np.clip(ang_speed / max(self.max_angular, 1e-9), -1.0, 1.0),
            np.clip(state.final_distance / total, 0.0, 1.0),
            np.clip(state.segment_index / max(1, len(self.path_points) - 2), 0.0, 1.0),
        ]
        obs.extend(self._lidar_sectors())
        return np.asarray(obs, dtype=np.float32), state

    def _map_action(self, action: np.ndarray) -> tuple[float, float]:
        clipped = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)
        linear = 0.5 * (float(clipped[0]) + 1.0) * self.max_linear
        angular = float(clipped[1]) * self.max_angular
        return linear, angular

    def _publish_cmd(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self._cmd_pub.publish(msg)

    def _publish_state(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._state_pub.publish(msg)

    def _control_tick(self) -> None:
        if not self._motion_enabled:
            self._publish_state("MOTION_DISABLED")
            self._publish_cmd(0.0, 0.0)
            return
        if self._goal_reached:
            self._publish_cmd(0.0, 0.0)
            return
        if len(self.path_points) < 2:
            self._publish_state("WAITING_FOR_PLAN")
            self._publish_cmd(0.0, 0.0)
            return
        pose = self._robot_pose()
        if pose is None or self.scan is None:
            self._publish_state("WAITING_FOR_POSE_OR_SCAN")
            self._publish_cmd(0.0, 0.0)
            return
        obs, state = self._obs(pose)
        center_distance = math.hypot(self.goal_x - pose[0], self.goal_y - pose[1])
        if center_distance <= self.final_radius:
            self._goal_reached = True
            self._publish_state("CENTER_REACHED")
            self._publish_cmd(0.0, 0.0)
            return
        action, _ = self.model.predict(obs, deterministic=True)
        raw_action = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)
        linear, angular = self._map_action(raw_action)
        self.previous_progress = float(state.progress)
        self.previous_cte = float(state.signed_cross_track_error)
        self.previous_raw_action = raw_action
        self.previous_cmd_linear = linear
        self.previous_cmd_angular = angular
        self._publish_state(
            f"TRACKING_ROUTE center_dist={center_distance:.3f} route_final_dist={state.final_distance:.3f} "
            f"cte={state.abs_cross_track_error:.3f} "
            f"lookahead={state.lookahead_angle:.3f}"
        )
        self._publish_cmd(linear, angular)


def main() -> None:
    rclpy.init()
    node = SacMotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._publish_cmd(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
