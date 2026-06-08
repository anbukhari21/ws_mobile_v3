#!/usr/bin/env python3
"""CSV logger for SAC navigation validation."""

from __future__ import annotations

import csv
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String


class NavigationValidationLogger(Node):
    def __init__(self) -> None:
        super().__init__("navigation_validation_logger")
        self.declare_parameter(
            "output_csv",
            "/home/abdulrahman/ws_mobile_v3/rl_results/sac_navigation/sac_motion_validation.csv",
        )
        self.declare_parameter("goal_x", 0.0)
        self.declare_parameter("goal_y", 0.0)
        self.output_csv = Path(str(self.get_parameter("output_csv").value)).expanduser()
        self.output_csv.parent.mkdir(parents=True, exist_ok=True)
        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)

        self.odom: Odometry | None = None
        self.plan: NavPath | None = None
        self.lookahead: PoseStamped | None = None
        self.features: Float32MultiArray | None = None
        self.memory: Float32MultiArray | None = None
        self.raw_cmd: Twist | None = None
        self.final_cmd: Twist | None = None
        self.safety_state = ""
        self.sac_state = ""

        self.create_subscription(Odometry, "/odom", lambda msg: setattr(self, "odom", msg), 10)
        self.create_subscription(NavPath, "/plan", lambda msg: setattr(self, "plan", msg), 10)
        self.create_subscription(PoseStamped, "/sac/lookahead_pose", lambda msg: setattr(self, "lookahead", msg), 10)
        self.create_subscription(Float32MultiArray, "/sac/route_features", lambda msg: setattr(self, "features", msg), 10)
        self.create_subscription(Float32MultiArray, "/safety/min_memory_distances", lambda msg: setattr(self, "memory", msg), 10)
        self.create_subscription(Twist, "/sac_cmd_vel_raw", lambda msg: setattr(self, "raw_cmd", msg), 10)
        self.create_subscription(Twist, "/cmd_vel", lambda msg: setattr(self, "final_cmd", msg), 10)
        self.create_subscription(String, "/safety/state", self._safety_state_cb, 10)
        self.create_subscription(String, "/sac/motion_state", self._sac_state_cb, 10)

        self._fh = self.output_csv.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(
            self._fh,
            fieldnames=[
                "time_sec",
                "robot_x",
                "robot_y",
                "final_distance_to_center",
                "plan_pose_count",
                "lookahead_x",
                "lookahead_y",
                "path_progress_fraction",
                "abs_cross_track_error",
                "memory_front",
                "memory_overall",
                "raw_linear",
                "raw_angular",
                "cmd_linear",
                "cmd_angular",
                "safety_state",
                "sac_state",
            ],
        )
        self._writer.writeheader()
        self.create_timer(0.1, self._tick)
        self.get_logger().info(f"Writing SAC navigation validation CSV: {self.output_csv}")

    def _safety_state_cb(self, msg: String) -> None:
        self.safety_state = msg.data

    def _sac_state_cb(self, msg: String) -> None:
        self.sac_state = msg.data

    def _tick(self) -> None:
        robot_x = robot_y = float("nan")
        final_distance = float("nan")
        if self.odom is not None:
            robot_x = float(self.odom.pose.pose.position.x)
            robot_y = float(self.odom.pose.pose.position.y)
            final_distance = math.hypot(self.goal_x - robot_x, self.goal_y - robot_y)
        lookahead_x = lookahead_y = float("nan")
        if self.lookahead is not None:
            lookahead_x = float(self.lookahead.pose.position.x)
            lookahead_y = float(self.lookahead.pose.position.y)
        progress = cte = float("nan")
        if self.features is not None and len(self.features.data) >= 5:
            cte = float(self.features.data[3])
            progress = float(self.features.data[4])
        memory_front = memory_overall = float("nan")
        if self.memory is not None and len(self.memory.data) >= 4:
            memory_front = float(self.memory.data[0])
            memory_overall = float(self.memory.data[3])
        raw_linear = raw_angular = cmd_linear = cmd_angular = float("nan")
        if self.raw_cmd is not None:
            raw_linear = float(self.raw_cmd.linear.x)
            raw_angular = float(self.raw_cmd.angular.z)
        if self.final_cmd is not None:
            cmd_linear = float(self.final_cmd.linear.x)
            cmd_angular = float(self.final_cmd.angular.z)
        self._writer.writerow(
            {
                "time_sec": self.get_clock().now().nanoseconds * 1e-9,
                "robot_x": robot_x,
                "robot_y": robot_y,
                "final_distance_to_center": final_distance,
                "plan_pose_count": len(self.plan.poses) if self.plan is not None else 0,
                "lookahead_x": lookahead_x,
                "lookahead_y": lookahead_y,
                "path_progress_fraction": progress,
                "abs_cross_track_error": cte,
                "memory_front": memory_front,
                "memory_overall": memory_overall,
                "raw_linear": raw_linear,
                "raw_angular": raw_angular,
                "cmd_linear": cmd_linear,
                "cmd_angular": cmd_angular,
                "safety_state": self.safety_state,
                "sac_state": self.sac_state,
            }
        )
        self._fh.flush()

    def destroy_node(self) -> bool:
        if not self._fh.closed:
            self._fh.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = NavigationValidationLogger()
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
