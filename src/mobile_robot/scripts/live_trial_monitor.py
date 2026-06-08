#!/usr/bin/env python3
"""Terminal live monitor for robot trials.

This script is passive: it subscribes only and never publishes commands.
"""

import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class LiveTrialMonitor(Node):
    def __init__(self) -> None:
        super().__init__("live_trial_monitor")
        self.start_time = time.time()
        self.odom: Optional[Odometry] = None
        self.cmd: Optional[Twist] = None
        self.goal_distance: Optional[float] = None
        self.goal_angle: Optional[float] = None
        self.goal_valid: Optional[bool] = None
        self.min_lidar: Optional[float] = None
        self.success_threshold = 0.15

        self.create_subscription(Odometry, "/odom", self._odom_cb, 20)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 20)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, 10)
        self.create_subscription(Float32, "/goal_distance", self._goal_distance_cb, 10)
        self.create_subscription(Float32, "/goal_angle", self._goal_angle_cb, 10)
        self.create_subscription(Bool, "/goal_valid", self._goal_valid_cb, 10)
        self.create_timer(1.0, self._print_status)
        self.get_logger().warn("Live monitor is passive. It does not control the robot.")

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom = msg

    def _cmd_cb(self, msg: Twist) -> None:
        self.cmd = msg

    def _scan_cb(self, msg: LaserScan) -> None:
        finite = [float(r) for r in msg.ranges if math.isfinite(float(r)) and float(r) > 0.0]
        self.min_lidar = min(finite) if finite else float("nan")

    def _goal_distance_cb(self, msg: Float32) -> None:
        self.goal_distance = float(msg.data)

    def _goal_angle_cb(self, msg: Float32) -> None:
        self.goal_angle = float(msg.data)

    def _goal_valid_cb(self, msg: Bool) -> None:
        self.goal_valid = bool(msg.data)

    def _print_status(self) -> None:
        elapsed = time.time() - self.start_time
        x = y = yaw = float("nan")
        if self.odom is not None:
            pose = self.odom.pose.pose
            q = pose.orientation
            x = float(pose.position.x)
            y = float(pose.position.y)
            yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        cmd_v = cmd_w = float("nan")
        if self.cmd is not None:
            cmd_v = float(self.cmd.linear.x)
            cmd_w = float(self.cmd.angular.z)
        success = (
            self.goal_distance is not None
            and math.isfinite(self.goal_distance)
            and self.goal_distance <= self.success_threshold
        )
        print(
            f"t={elapsed:6.1f}s "
            f"odom=({x:+.3f},{y:+.3f},{yaw:+.3f}) "
            f"cmd=({cmd_v:+.3f},{cmd_w:+.3f}) "
            f"goal=({self.goal_distance if self.goal_distance is not None else float('nan'):.3f},"
            f"{self.goal_angle if self.goal_angle is not None else float('nan'):+.3f}) "
            f"valid={self.goal_valid} "
            f"min_lidar={self.min_lidar if self.min_lidar is not None else float('nan'):.3f} "
            f"success={success}"
        )


def main() -> None:
    rclpy.init()
    node = LiveTrialMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
