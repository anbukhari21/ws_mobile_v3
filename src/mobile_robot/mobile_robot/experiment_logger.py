#!/usr/bin/env python3
"""Unified passive experiment logger for simulation and real robot tests.

The node only subscribes and writes logs. It never publishes velocity commands
or controls the robot.
"""

import csv
import json
import math
import os
import signal
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String


CSV_HEADER = [
    "step",
    "ros_time",
    "elapsed_time",
    "odom_x",
    "odom_y",
    "odom_yaw",
    "odom_linear_x",
    "odom_angular_z",
    "cmd_linear_x",
    "cmd_angular_z",
    "goal_distance",
    "goal_angle",
    "goal_valid",
    "min_lidar_range",
    "collision",
    "success",
    "localization_mode",
    "test_type",
]


def expand_path(path_text: str) -> Path:
    return Path(os.path.expandvars(os.path.expanduser(path_text))).resolve()


def safe_name(text: str) -> str:
    cleaned = "".join(ch if ch.isalnum() or ch in "-_." else "_" for ch in str(text))
    return cleaned.strip("._") or "trial"


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def finite_or_nan(value: Optional[float]) -> float:
    if value is None:
        return float("nan")
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return float("nan")
    return numeric if math.isfinite(numeric) else float("nan")


def fmt(value: Any, digits: int = 6) -> str:
    if value is None:
        return ""
    if isinstance(value, bool):
        return "true" if value else "false"
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return str(value)
    if not math.isfinite(numeric):
        return "nan"
    return f"{numeric:.{digits}f}"


class ExperimentLogger(Node):
    """Write advisor logs, CSV logs, summaries, and plots for one test run."""

    def __init__(self) -> None:
        super().__init__("experiment_logger")

        self.declare_parameter("test_type", "simulation")
        self.declare_parameter("trial_name", "trial")
        self.declare_parameter("output_root", "~/robot_test_logs")
        self.declare_parameter("log_rate_hz", 10.0)
        self.declare_parameter("enable_txt_log", True)
        self.declare_parameter("enable_csv_log", True)
        self.declare_parameter("enable_plots", True)
        self.declare_parameter("arena_radius", 2.0)
        self.declare_parameter("success_distance_threshold", 0.15)
        self.declare_parameter("max_trial_time_sec", 600.0)
        self.declare_parameter("model_path", "")
        self.declare_parameter("world_name", "")
        self.declare_parameter("localization_mode", "")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("goal_distance_topic", "/goal_distance")
        self.declare_parameter("goal_angle_topic", "/goal_angle")
        self.declare_parameter("goal_valid_topic", "/goal_valid")
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("collision_status_topic", "/collision_status")
        self.declare_parameter("episode_status_topic", "/episode_status")
        self.declare_parameter("reward_topic", "/reward")
        self.declare_parameter("target_x", float("nan"))
        self.declare_parameter("target_y", float("nan"))

        self.test_type = str(self.get_parameter("test_type").value)
        if self.test_type not in {"simulation", "real_robot"}:
            raise ValueError("test_type must be 'simulation' or 'real_robot'")

        self.trial_name = safe_name(str(self.get_parameter("trial_name").value))
        self.output_root = expand_path(str(self.get_parameter("output_root").value))
        self.log_rate_hz = max(0.1, float(self.get_parameter("log_rate_hz").value))
        self.enable_txt_log = bool(self.get_parameter("enable_txt_log").value)
        self.enable_csv_log = bool(self.get_parameter("enable_csv_log").value)
        self.enable_plots = bool(self.get_parameter("enable_plots").value)
        self.arena_radius = float(self.get_parameter("arena_radius").value)
        self.success_distance_threshold = float(
            self.get_parameter("success_distance_threshold").value
        )
        self.max_trial_time_sec = float(self.get_parameter("max_trial_time_sec").value)
        self.model_path = str(self.get_parameter("model_path").value)
        self.world_name = str(self.get_parameter("world_name").value)
        self.localization_mode = str(self.get_parameter("localization_mode").value)
        self.target_x = finite_or_nan(self.get_parameter("target_x").value)
        self.target_y = finite_or_nan(self.get_parameter("target_y").value)

        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.log_dir = self.output_root / self.test_type / f"{timestamp}_{self.trial_name}"
        self.log_dir.mkdir(parents=True, exist_ok=True)

        self.start_wall_time = time.time()
        self.start_time_iso = datetime.fromtimestamp(self.start_wall_time).isoformat()
        self.step = 0
        self.finalized = False
        self.stop_requested = False

        self.latest_odom: Optional[Odometry] = None
        self.latest_cmd: Optional[Twist] = None
        self.latest_goal_distance: Optional[float] = None
        self.latest_goal_angle: Optional[float] = None
        self.latest_goal_valid: Optional[bool] = None
        self.latest_min_lidar: Optional[float] = None
        self.latest_collision: Optional[bool] = None
        self.latest_episode_status = ""
        self.latest_reward: Optional[float] = None
        self.latest_amcl_pose: Optional[PoseWithCovarianceStamped] = None

        self.received_topics = {
            "odom": False,
            "cmd_vel": False,
            "scan": False,
            "goal_distance": False,
            "goal_angle": False,
            "goal_valid": False,
            "amcl_pose": False,
            "collision_status": False,
            "episode_status": False,
            "reward": False,
        }
        self.collision_events = 0
        self._last_collision_state = False
        self.total_distance_traveled = 0.0
        self._previous_odom_xy: Optional[tuple[float, float]] = None
        self.minimum_goal_distance: Optional[float] = None
        self.rows: List[Dict[str, Any]] = []

        self.csv_file = None
        self.csv_writer = None
        if self.enable_csv_log:
            self.csv_file = (self.log_dir / "step_log.csv").open(
                "w", newline="", encoding="utf-8"
            )
            self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=CSV_HEADER)
            self.csv_writer.writeheader()
            self.csv_file.flush()

        self.step_txt_file = None
        self.advisor_txt_file = None
        if self.enable_txt_log:
            self.step_txt_file = (self.log_dir / "step_log.txt").open("w", encoding="utf-8")
            self.advisor_txt_file = (self.log_dir / "advisor_step_log.txt").open(
                "w", encoding="utf-8"
            )

        self._write_metadata()
        self._create_subscriptions()
        self.timer = self.create_timer(1.0 / self.log_rate_hz, self._timer_cb)
        self.status_timer = self.create_timer(5.0, self._status_cb)

        self.get_logger().warn(
            "Experiment logger is recording only. It does not control the robot."
        )
        self.get_logger().info(f"Writing experiment logs to: {self.log_dir}")

    def _topic_param(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _create_subscriptions(self) -> None:
        self.create_subscription(Odometry, self._topic_param("odom_topic"), self._odom_cb, 50)
        self.create_subscription(Twist, self._topic_param("cmd_vel_topic"), self._cmd_cb, 50)
        self.create_subscription(LaserScan, self._topic_param("scan_topic"), self._scan_cb, 20)
        self.create_subscription(
            Float32, self._topic_param("goal_distance_topic"), self._goal_distance_cb, 20
        )
        self.create_subscription(
            Float32, self._topic_param("goal_angle_topic"), self._goal_angle_cb, 20
        )
        self.create_subscription(
            Bool, self._topic_param("goal_valid_topic"), self._goal_valid_cb, 20
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self._topic_param("amcl_pose_topic"),
            self._amcl_pose_cb,
            10,
        )
        self.create_subscription(
            Bool, self._topic_param("collision_status_topic"), self._collision_cb, 10
        )
        self.create_subscription(
            String, self._topic_param("episode_status_topic"), self._episode_status_cb, 10
        )
        self.create_subscription(Float32, self._topic_param("reward_topic"), self._reward_cb, 10)

    def _odom_cb(self, msg: Odometry) -> None:
        self.received_topics["odom"] = True
        self.latest_odom = msg
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        if self._previous_odom_xy is not None:
            last_x, last_y = self._previous_odom_xy
            delta = math.hypot(x - last_x, y - last_y)
            if math.isfinite(delta) and delta < 1.0:
                self.total_distance_traveled += delta
        self._previous_odom_xy = (x, y)

    def _cmd_cb(self, msg: Twist) -> None:
        self.received_topics["cmd_vel"] = True
        self.latest_cmd = msg

    def _scan_cb(self, msg: LaserScan) -> None:
        self.received_topics["scan"] = True
        finite_ranges = [float(r) for r in msg.ranges if math.isfinite(float(r)) and float(r) > 0.0]
        self.latest_min_lidar = min(finite_ranges) if finite_ranges else float("nan")

    def _goal_distance_cb(self, msg: Float32) -> None:
        self.received_topics["goal_distance"] = True
        self.latest_goal_distance = float(msg.data)
        if math.isfinite(self.latest_goal_distance):
            if self.minimum_goal_distance is None:
                self.minimum_goal_distance = self.latest_goal_distance
            else:
                self.minimum_goal_distance = min(
                    self.minimum_goal_distance,
                    self.latest_goal_distance,
                )

    def _goal_angle_cb(self, msg: Float32) -> None:
        self.received_topics["goal_angle"] = True
        self.latest_goal_angle = float(msg.data)

    def _goal_valid_cb(self, msg: Bool) -> None:
        self.received_topics["goal_valid"] = True
        self.latest_goal_valid = bool(msg.data)

    def _amcl_pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        self.received_topics["amcl_pose"] = True
        self.latest_amcl_pose = msg

    def _collision_cb(self, msg: Bool) -> None:
        self.received_topics["collision_status"] = True
        collision = bool(msg.data)
        self.latest_collision = collision
        if collision and not self._last_collision_state:
            self.collision_events += 1
        self._last_collision_state = collision

    def _episode_status_cb(self, msg: String) -> None:
        self.received_topics["episode_status"] = True
        self.latest_episode_status = str(msg.data)

    def _reward_cb(self, msg: Float32) -> None:
        self.received_topics["reward"] = True
        self.latest_reward = float(msg.data)

    def _status_cb(self) -> None:
        missing = [name for name, seen in self.received_topics.items() if not seen]
        if not self.received_topics["odom"]:
            self.get_logger().warn("No /odom received yet; logging NaN odometry.", throttle_duration_sec=5.0)
        if not self.received_topics["cmd_vel"]:
            self.get_logger().warn("No /cmd_vel received yet; logging NaN commands.", throttle_duration_sec=5.0)
        if missing:
            self.get_logger().info(
                "Optional topics not seen yet: " + ", ".join(missing),
                throttle_duration_sec=15.0,
            )

    def _timer_cb(self) -> None:
        if self.finalized:
            return
        elapsed = time.time() - self.start_wall_time
        if self.max_trial_time_sec > 0.0 and elapsed > self.max_trial_time_sec:
            self.get_logger().warn("max_trial_time_sec reached; finalizing logger.")
            self.finalize()
            self.stop_requested = True
            return

        self.step += 1
        row = self._make_row(elapsed)
        self.rows.append(row)

        if self.csv_writer is not None and self.csv_file is not None:
            self.csv_writer.writerow(row)
            self.csv_file.flush()

        advisor_line = self._advisor_line(row)
        if self.advisor_txt_file is not None:
            self.advisor_txt_file.write(advisor_line + "\n")
            self.advisor_txt_file.flush()
        if self.step_txt_file is not None:
            self.step_txt_file.write(self._step_line(row) + "\n")
            self.step_txt_file.flush()

    def _make_row(self, elapsed: float) -> Dict[str, Any]:
        odom_x = odom_y = yaw = odom_vx = odom_wz = None
        if self.latest_odom is not None:
            pose = self.latest_odom.pose.pose
            twist = self.latest_odom.twist.twist
            q = pose.orientation
            odom_x = float(pose.position.x)
            odom_y = float(pose.position.y)
            yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
            odom_vx = float(twist.linear.x)
            odom_wz = float(twist.angular.z)

        cmd_vx = cmd_wz = None
        if self.latest_cmd is not None:
            cmd_vx = float(self.latest_cmd.linear.x)
            cmd_wz = float(self.latest_cmd.angular.z)

        success: Optional[bool] = None
        if self.received_topics["goal_distance"] and self.latest_goal_distance is not None:
            success = bool(self.latest_goal_distance <= self.success_distance_threshold)

        ros_time = self.get_clock().now().nanoseconds / 1_000_000_000.0
        return {
            "step": self.step,
            "ros_time": fmt(ros_time, 9),
            "elapsed_time": fmt(elapsed, 3),
            "odom_x": fmt(odom_x),
            "odom_y": fmt(odom_y),
            "odom_yaw": fmt(yaw),
            "odom_linear_x": fmt(odom_vx),
            "odom_angular_z": fmt(odom_wz),
            "cmd_linear_x": fmt(cmd_vx),
            "cmd_angular_z": fmt(cmd_wz),
            "goal_distance": fmt(self.latest_goal_distance),
            "goal_angle": fmt(self.latest_goal_angle),
            "goal_valid": fmt(self.latest_goal_valid),
            "min_lidar_range": fmt(self.latest_min_lidar),
            "collision": fmt(self.latest_collision),
            "success": fmt(success),
            "localization_mode": self.localization_mode,
            "test_type": self.test_type,
        }

    @staticmethod
    def _advisor_line(row: Dict[str, Any]) -> str:
        return (
            f"step={int(row['step']):06d} "
            f"time={row['elapsed_time']} "
            f"odom_x={row['odom_x'] or 'nan'} "
            f"odom_y={row['odom_y'] or 'nan'} "
            f"yaw={row['odom_yaw'] or 'nan'} "
            f"odom_vx={row['odom_linear_x'] or 'nan'} "
            f"odom_wz={row['odom_angular_z'] or 'nan'} "
            f"cmd_vx={row['cmd_linear_x'] or 'nan'} "
            f"cmd_wz={row['cmd_angular_z'] or 'nan'}"
        )

    @staticmethod
    def _step_line(row: Dict[str, Any]) -> str:
        return " ".join(f"{key}={value}" for key, value in row.items())

    def _write_metadata(self) -> None:
        metadata = {
            "test_type": self.test_type,
            "trial_name": self.trial_name,
            "start_time": self.start_time_iso,
            "output_directory": str(self.log_dir),
            "localization_mode": self.localization_mode,
            "world_name": self.world_name,
            "model_path": self.model_path,
            "arena_radius": self.arena_radius,
            "success_distance_threshold": self.success_distance_threshold,
            "max_trial_time_sec": self.max_trial_time_sec,
            "target_x": self.target_x,
            "target_y": self.target_y,
            "topics": {
                "odom": self._topic_param("odom_topic"),
                "cmd_vel": self._topic_param("cmd_vel_topic"),
                "scan": self._topic_param("scan_topic"),
                "goal_distance": self._topic_param("goal_distance_topic"),
                "goal_angle": self._topic_param("goal_angle_topic"),
                "goal_valid": self._topic_param("goal_valid_topic"),
                "amcl_pose": self._topic_param("amcl_pose_topic"),
            },
        }
        with (self.log_dir / "metadata.yaml").open("w", encoding="utf-8") as f:
            self._write_yaml_dict(f, metadata)

    def _write_yaml_dict(self, file_obj, data: Dict[str, Any], indent: int = 0) -> None:
        prefix = " " * indent
        for key, value in data.items():
            if isinstance(value, dict):
                file_obj.write(f"{prefix}{key}:\n")
                self._write_yaml_dict(file_obj, value, indent + 2)
            else:
                if isinstance(value, float) and not math.isfinite(value):
                    value_text = ""
                else:
                    value_text = str(value).replace("\n", " ")
                file_obj.write(f"{prefix}{key}: {value_text}\n")

    def _parse_float_from_row(self, row: Dict[str, Any], key: str) -> Optional[float]:
        value = row.get(key, "")
        if value in ("", "nan", None):
            return None
        try:
            numeric = float(value)
        except (TypeError, ValueError):
            return None
        return numeric if math.isfinite(numeric) else None

    def _summary(self) -> Dict[str, Any]:
        end_wall_time = time.time()
        duration = end_wall_time - self.start_wall_time
        final_row = self.rows[-1] if self.rows else {}
        final_goal_distance = self._parse_float_from_row(final_row, "goal_distance")
        min_goal_distance = self.minimum_goal_distance

        goal_data_available = self.received_topics["goal_distance"]
        success: Optional[bool]
        if goal_data_available:
            success = bool(
                (final_goal_distance is not None and final_goal_distance <= self.success_distance_threshold)
                or (
                    min_goal_distance is not None
                    and min_goal_distance <= self.success_distance_threshold
                )
            )
        else:
            success = None

        linear_values = [
            abs(v)
            for row in self.rows
            for v in [self._parse_float_from_row(row, "odom_linear_x")]
            if v is not None
        ]
        angular_values = [
            abs(v)
            for row in self.rows
            for v in [self._parse_float_from_row(row, "odom_angular_z")]
            if v is not None
        ]
        missing_topics = [name for name, seen in self.received_topics.items() if not seen]

        return {
            "test_type": self.test_type,
            "trial_name": self.trial_name,
            "start_time": self.start_time_iso,
            "end_time": datetime.fromtimestamp(end_wall_time).isoformat(),
            "duration": duration,
            "total_steps": self.step,
            "localization_mode": self.localization_mode,
            "world_name": self.world_name,
            "sac_model_path": self.model_path,
            "final_odom_x": self._parse_float_from_row(final_row, "odom_x"),
            "final_odom_y": self._parse_float_from_row(final_row, "odom_y"),
            "final_yaw": self._parse_float_from_row(final_row, "odom_yaw"),
            "final_goal_distance": final_goal_distance,
            "minimum_goal_distance_reached": min_goal_distance,
            "maximum_linear_velocity": max(linear_values) if linear_values else None,
            "maximum_angular_velocity": max(angular_values) if angular_values else None,
            "average_linear_velocity": (
                sum(linear_values) / len(linear_values) if linear_values else None
            ),
            "average_angular_velocity": (
                sum(angular_values) / len(angular_values) if angular_values else None
            ),
            "total_distance_traveled": self.total_distance_traveled,
            "number_of_collision_events": (
                self.collision_events if self.received_topics["collision_status"] else None
            ),
            "success": success,
            "success_rule_used": (
                f"final or minimum /goal_distance <= {self.success_distance_threshold:.3f} m"
                if goal_data_available
                else "unknown because /goal_distance was not received"
            ),
            "missing_topics": missing_topics,
            "notes": (
                "Experiment logger is passive and did not control the robot. "
                "Missing optional topics are recorded as unavailable."
            ),
        }

    def finalize(self) -> None:
        if self.finalized:
            return
        self.finalized = True

        summary = self._summary()
        with (self.log_dir / "summary.json").open("w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2, sort_keys=True)
        with (self.log_dir / "summary.txt").open("w", encoding="utf-8") as f:
            for key, value in summary.items():
                f.write(f"{key}: {value}\n")

        if self.csv_file is not None and not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()
        if self.step_txt_file is not None and not self.step_txt_file.closed:
            self.step_txt_file.flush()
            self.step_txt_file.close()
        if self.advisor_txt_file is not None and not self.advisor_txt_file.closed:
            self.advisor_txt_file.flush()
            self.advisor_txt_file.close()

        if self.enable_plots:
            try:
                self._save_plots(summary)
                self._write_trial_report(summary)
            except Exception as exc:
                self.get_logger().warn(f"Could not generate plots/report: {exc}")

        self.get_logger().info(f"Final experiment summary saved in: {self.log_dir}")

    def _save_plots(self, summary: Dict[str, Any]) -> None:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        times = [self._parse_float_from_row(r, "elapsed_time") for r in self.rows]
        times = [t if t is not None else 0.0 for t in times]

        xs = [self._parse_float_from_row(r, "odom_x") for r in self.rows]
        ys = [self._parse_float_from_row(r, "odom_y") for r in self.rows]
        xy = [(x, y) for x, y in zip(xs, ys) if x is not None and y is not None]

        plt.figure(figsize=(7, 7))
        if xy:
            plot_x, plot_y = zip(*xy)
            plt.plot(plot_x, plot_y, linewidth=2, label="odom trajectory")
            plt.scatter([plot_x[0]], [plot_y[0]], marker="o", s=60, label="start")
            plt.scatter([plot_x[-1]], [plot_y[-1]], marker="x", s=80, label="end")
        if math.isfinite(self.target_x) and math.isfinite(self.target_y):
            plt.scatter([self.target_x], [self.target_y], marker="*", s=120, label="target")
        arena = plt.Circle((0.0, 0.0), self.arena_radius, fill=False, linestyle="--", label="arena")
        plt.gca().add_patch(arena)
        plt.axis("equal")
        plt.xlabel("odom x (m)")
        plt.ylabel("odom y (m)")
        plt.title("Robot Trajectory")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(self.log_dir / "trajectory.png", dpi=160)
        plt.close()

        odom_v = [finite_or_nan(self._parse_float_from_row(r, "odom_linear_x")) for r in self.rows]
        cmd_v = [finite_or_nan(self._parse_float_from_row(r, "cmd_linear_x")) for r in self.rows]
        odom_w = [finite_or_nan(self._parse_float_from_row(r, "odom_angular_z")) for r in self.rows]
        cmd_w = [finite_or_nan(self._parse_float_from_row(r, "cmd_angular_z")) for r in self.rows]
        plt.figure(figsize=(10, 7))
        ax1 = plt.subplot(2, 1, 1)
        ax1.plot(times, odom_v, label="odom linear x")
        ax1.plot(times, cmd_v, label="cmd linear x")
        ax1.set_ylabel("linear velocity (m/s)")
        ax1.grid(True)
        ax1.legend()
        ax2 = plt.subplot(2, 1, 2, sharex=ax1)
        ax2.plot(times, odom_w, label="odom angular z")
        ax2.plot(times, cmd_w, label="cmd angular z")
        ax2.set_xlabel("elapsed time (s)")
        ax2.set_ylabel("angular velocity (rad/s)")
        ax2.grid(True)
        ax2.legend()
        plt.tight_layout()
        plt.savefig(self.log_dir / "velocity_plot.png", dpi=160)
        plt.close()

        goal_dist = [finite_or_nan(self._parse_float_from_row(r, "goal_distance")) for r in self.rows]
        if any(math.isfinite(v) for v in goal_dist):
            plt.figure(figsize=(10, 4))
            plt.plot(times, goal_dist, label="goal distance")
            plt.axhline(
                self.success_distance_threshold,
                linestyle="--",
                label="success threshold",
            )
            plt.xlabel("elapsed time (s)")
            plt.ylabel("distance (m)")
            plt.title("Goal Distance")
            plt.grid(True)
            plt.legend()
            plt.tight_layout()
            plt.savefig(self.log_dir / "goal_distance_plot.png", dpi=160)
            plt.close()

        lidar_min = [finite_or_nan(self._parse_float_from_row(r, "min_lidar_range")) for r in self.rows]
        if any(math.isfinite(v) for v in lidar_min):
            plt.figure(figsize=(10, 4))
            plt.plot(times, lidar_min, label="minimum LiDAR range")
            plt.xlabel("elapsed time (s)")
            plt.ylabel("range (m)")
            plt.title("Minimum LiDAR Range")
            plt.grid(True)
            plt.legend()
            plt.tight_layout()
            plt.savefig(self.log_dir / "lidar_min_range_plot.png", dpi=160)
            plt.close()

    def _write_trial_report(self, summary: Dict[str, Any]) -> None:
        lines = [
            "# Trial Report",
            "",
            f"- Test type: {summary.get('test_type')}",
            f"- Trial name: {summary.get('trial_name')}",
            f"- Duration: {summary.get('duration'):.3f} s",
            f"- Total steps: {summary.get('total_steps')}",
            f"- Success: {summary.get('success')}",
            f"- Final goal distance: {summary.get('final_goal_distance')}",
            f"- Minimum goal distance: {summary.get('minimum_goal_distance_reached')}",
            f"- Total distance traveled: {summary.get('total_distance_traveled')}",
            f"- Collision events: {summary.get('number_of_collision_events')}",
            f"- Missing topics: {', '.join(summary.get('missing_topics', [])) or 'none'}",
            "",
            "## Generated Files",
            "",
            "- `advisor_step_log.txt`",
            "- `step_log.csv`",
            "- `summary.json`",
            "- `summary.txt`",
            "- `metadata.yaml`",
            "- `trajectory.png`",
            "- `velocity_plot.png`",
            "- `goal_distance_plot.png` if goal data was available",
            "- `lidar_min_range_plot.png` if scan data was available",
        ]
        (self.log_dir / "trial_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")

    def destroy_node(self) -> bool:
        self.finalize()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExperimentLogger()

    def _shutdown_handler(signum, frame) -> None:  # noqa: ARG001
        if rclpy.ok():
            node.get_logger().info(f"Received signal {signum}; saving experiment logs.")
        node.finalize()
        node.stop_requested = True

    signal.signal(signal.SIGINT, _shutdown_handler)
    signal.signal(signal.SIGTERM, _shutdown_handler)

    try:
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
