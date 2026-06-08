#!/usr/bin/env python3
"""Log one CSV row for each velocity command sent to the robot.

The logger is intentionally passive: it subscribes to robot state and sensor
topics and does not publish anything, so it can be used with SAC, teleop,
Gazebo, or the real robot without changing the controller.
"""

import csv
import json
import math
import os
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


CSV_HEADER = [
    "step",
    "time_sec",
    "odom_x",
    "odom_y",
    "yaw",
    "odom_linear_x",
    "odom_angular_z",
    "cmd_linear_x",
    "cmd_angular_z",
    "goal_x",
    "goal_y",
    "distance_to_goal",
    "angle_to_goal",
    "min_lidar_range",
    "mean_lidar_range",
    "collision_flag",
    "reached_goal_flag",
    "control_source",
    "cmd_vel_publishers",
]

SCAN_SNAPSHOT_HEADER = [
    "snapshot",
    "time_sec",
    "angle_min",
    "angle_increment",
    "range_min",
    "range_max",
    "ranges_json",
]


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return yaw in radians from a quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class ActionLoggerNode(Node):
    """CSV logger keyed by control/action decisions on /cmd_vel."""

    def __init__(self) -> None:
        super().__init__("action_logger_node")

        default_log_path = "~/ws_mobile/experiment_logs/robot_action_log.csv"
        self.declare_parameter("log_path", default_log_path)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("goal_distance_topic", "/goal_distance")
        self.declare_parameter("goal_angle_topic", "/goal_angle")
        self.declare_parameter("goal_reached_topic", "/goal_reached")
        self.declare_parameter("collision_status_topic", "/collision_status")
        self.declare_parameter("goal_x", float("nan"))
        self.declare_parameter("goal_y", float("nan"))
        self.declare_parameter("success_radius", 0.20)
        self.declare_parameter("collision_range_threshold", 0.20)
        self.declare_parameter("control_source", "autonomous_sac")
        self.declare_parameter("scan_snapshot_path", "")
        self.declare_parameter("max_scan_snapshots", 12)
        self.declare_parameter("scan_snapshot_interval_sec", 2.0)

        self.log_path = self._expand_path(str(self.get_parameter("log_path").value))
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.goal_distance_topic = str(self.get_parameter("goal_distance_topic").value)
        self.goal_angle_topic = str(self.get_parameter("goal_angle_topic").value)
        self.goal_reached_topic = str(self.get_parameter("goal_reached_topic").value)
        self.collision_status_topic = str(self.get_parameter("collision_status_topic").value)
        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)
        self.success_radius = float(self.get_parameter("success_radius").value)
        self.collision_range_threshold = float(
            self.get_parameter("collision_range_threshold").value
        )
        self.control_source = str(self.get_parameter("control_source").value)
        self.max_scan_snapshots = max(0, int(self.get_parameter("max_scan_snapshots").value))
        self.scan_snapshot_interval_sec = max(
            0.1,
            float(self.get_parameter("scan_snapshot_interval_sec").value),
        )

        self.latest_odom: Optional[Odometry] = None
        self.latest_distance_to_goal: Optional[float] = None
        self.latest_angle_to_goal: Optional[float] = None
        self.latest_min_lidar: Optional[float] = None
        self.latest_mean_lidar: Optional[float] = None
        self.latest_collision: Optional[bool] = None
        self.latest_reached_goal: Optional[bool] = None
        self.step = 0
        self.scan_snapshot_count = 0
        self.last_scan_snapshot_time: Optional[float] = None

        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = self.log_path.open("w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(CSV_HEADER)
        self.csv_file.flush()

        scan_snapshot_param = str(self.get_parameter("scan_snapshot_path").value)
        if scan_snapshot_param:
            self.scan_snapshot_path = self._expand_path(scan_snapshot_param)
        else:
            self.scan_snapshot_path = self.log_path.with_name(
                f"{self.log_path.stem}_scan_snapshots.csv"
            )
        self.scan_snapshot_file = None
        self.scan_snapshot_writer = None
        if self.max_scan_snapshots > 0:
            self.scan_snapshot_path.parent.mkdir(parents=True, exist_ok=True)
            self.scan_snapshot_file = self.scan_snapshot_path.open("w", newline="")
            self.scan_snapshot_writer = csv.writer(self.scan_snapshot_file)
            self.scan_snapshot_writer.writerow(SCAN_SNAPSHOT_HEADER)
            self.scan_snapshot_file.flush()

        self.create_subscription(Odometry, self.odom_topic, self._odom_callback, 20)
        self.create_subscription(Twist, self.cmd_vel_topic, self._cmd_vel_callback, 20)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_callback, 20)
        self.create_subscription(
            Float32,
            self.goal_distance_topic,
            self._goal_distance_callback,
            20,
        )
        self.create_subscription(Float32, self.goal_angle_topic, self._goal_angle_callback, 20)
        self.create_subscription(Bool, self.goal_reached_topic, self._goal_reached_callback, 20)
        self.create_subscription(
            Bool,
            self.collision_status_topic,
            self._collision_status_callback,
            20,
        )

        self.get_logger().info(f"Action logger writing CSV to: {self.log_path}")
        self.get_logger().info(
            f"Listening for odometry on {self.odom_topic}, commands on {self.cmd_vel_topic}, "
            f"and LiDAR on {self.scan_topic}"
        )
        self.get_logger().info(
            f"control_source={self.control_source}; scan snapshots: {self.scan_snapshot_path}"
        )

    @staticmethod
    def _expand_path(path_text: str) -> Path:
        expanded = os.path.expandvars(os.path.expanduser(path_text))
        return Path(expanded).resolve()

    def _odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def _scan_callback(self, msg: LaserScan) -> None:
        finite_ranges = [
            float(value)
            for value in msg.ranges
            if math.isfinite(float(value)) and float(value) > 0.0
        ]
        if finite_ranges:
            self.latest_min_lidar = min(finite_ranges)
            self.latest_mean_lidar = sum(finite_ranges) / len(finite_ranges)
        else:
            self.latest_min_lidar = float("nan")
            self.latest_mean_lidar = float("nan")

        self._maybe_write_scan_snapshot(msg)

    def _goal_distance_callback(self, msg: Float32) -> None:
        self.latest_distance_to_goal = float(msg.data)

    def _goal_angle_callback(self, msg: Float32) -> None:
        self.latest_angle_to_goal = float(msg.data)

    def _goal_reached_callback(self, msg: Bool) -> None:
        self.latest_reached_goal = bool(msg.data)

    def _collision_status_callback(self, msg: Bool) -> None:
        self.latest_collision = bool(msg.data)

    @staticmethod
    def _fmt_float(value: Optional[float], digits: int = 9) -> str:
        if value is None:
            return ""
        try:
            numeric = float(value)
        except (TypeError, ValueError):
            return ""
        if not math.isfinite(numeric):
            return "nan"
        return f"{numeric:.{digits}f}"

    @staticmethod
    def _fmt_bool(value: Optional[bool]) -> str:
        if value is None:
            return ""
        return "1" if bool(value) else "0"

    def _goal_metrics_from_odom(self, odom: Optional[Odometry], yaw: Optional[float]) -> tuple[Optional[float], Optional[float]]:
        if (
            odom is None
            or yaw is None
            or not math.isfinite(self.goal_x)
            or not math.isfinite(self.goal_y)
        ):
            return None, None
        pose = odom.pose.pose
        dx = self.goal_x - float(pose.position.x)
        dy = self.goal_y - float(pose.position.y)
        distance = math.hypot(dx, dy)
        angle = math.atan2(dy, dx) - yaw
        angle = math.atan2(math.sin(angle), math.cos(angle))
        return distance, angle

    def _cmd_vel_publishers(self) -> str:
        try:
            infos = self.get_publishers_info_by_topic(self.cmd_vel_topic)
        except Exception as exc:  # ROS graph lookup should never stop logging.
            self.get_logger().warn(
                f"Could not inspect {self.cmd_vel_topic} publishers: {exc}",
                throttle_duration_sec=10.0,
            )
            return ""

        names = []
        for info in infos:
            namespace = getattr(info, "node_namespace", "") or ""
            name = getattr(info, "node_name", "") or ""
            if not name:
                continue
            full_name = f"{namespace.rstrip('/')}/{name}" if namespace not in ("", "/") else f"/{name}"
            names.append(full_name)
        return ";".join(sorted(set(names)))

    def _maybe_write_scan_snapshot(self, msg: LaserScan) -> None:
        if self.scan_snapshot_writer is None or self.scan_snapshot_file is None:
            return
        if self.scan_snapshot_count >= self.max_scan_snapshots:
            return

        now_sec = self.get_clock().now().nanoseconds / 1_000_000_000.0
        if (
            self.last_scan_snapshot_time is not None
            and now_sec - self.last_scan_snapshot_time < self.scan_snapshot_interval_sec
        ):
            return

        ranges = []
        for value in msg.ranges:
            numeric = float(value)
            ranges.append(numeric if math.isfinite(numeric) else None)

        self.scan_snapshot_count += 1
        self.last_scan_snapshot_time = now_sec
        self.scan_snapshot_writer.writerow(
            [
                self.scan_snapshot_count,
                f"{now_sec:.9f}",
                f"{float(msg.angle_min):.9f}",
                f"{float(msg.angle_increment):.9f}",
                f"{float(msg.range_min):.9f}",
                f"{float(msg.range_max):.9f}",
                json.dumps(ranges, separators=(",", ":")),
            ]
        )
        self.scan_snapshot_file.flush()

    def _cmd_vel_callback(self, msg: Twist) -> None:
        odom = self.latest_odom
        pose = odom.pose.pose if odom is not None else None
        twist = odom.twist.twist if odom is not None else None
        q = pose.orientation if pose is not None else None

        self.step += 1
        time_sec = self.get_clock().now().nanoseconds / 1_000_000_000.0
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w) if q is not None else None

        computed_distance, computed_angle = self._goal_metrics_from_odom(odom, yaw)
        distance_to_goal = (
            self.latest_distance_to_goal
            if self.latest_distance_to_goal is not None
            else computed_distance
        )
        angle_to_goal = (
            self.latest_angle_to_goal if self.latest_angle_to_goal is not None else computed_angle
        )

        collision_flag = self.latest_collision
        if collision_flag is None and self.latest_min_lidar is not None:
            collision_flag = bool(self.latest_min_lidar <= self.collision_range_threshold)

        reached_goal_flag = self.latest_reached_goal
        if reached_goal_flag is None and distance_to_goal is not None:
            reached_goal_flag = bool(distance_to_goal <= self.success_radius)

        self.csv_writer.writerow(
            [
                self.step,
                f"{time_sec:.9f}",
                self._fmt_float(pose.position.x if pose is not None else None),
                self._fmt_float(pose.position.y if pose is not None else None),
                self._fmt_float(yaw),
                self._fmt_float(twist.linear.x if twist is not None else None),
                self._fmt_float(twist.angular.z if twist is not None else None),
                f"{msg.linear.x:.9f}",
                f"{msg.angular.z:.9f}",
                self._fmt_float(self.goal_x),
                self._fmt_float(self.goal_y),
                self._fmt_float(distance_to_goal),
                self._fmt_float(angle_to_goal),
                self._fmt_float(self.latest_min_lidar),
                self._fmt_float(self.latest_mean_lidar),
                self._fmt_bool(collision_flag),
                self._fmt_bool(reached_goal_flag),
                self.control_source,
                self._cmd_vel_publishers(),
            ]
        )
        self.csv_file.flush()

    def destroy_node(self) -> bool:
        if hasattr(self, "csv_file") and not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()
        if (
            hasattr(self, "scan_snapshot_file")
            and self.scan_snapshot_file is not None
            and not self.scan_snapshot_file.closed
        ):
            self.scan_snapshot_file.flush()
            self.scan_snapshot_file.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ActionLoggerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
