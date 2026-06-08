#!/usr/bin/env python3
"""Final emergency-only collision filter from SAC raw commands to /cmd_vel."""

from __future__ import annotations

import copy
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, String


class SacSafetyFilterNode(Node):
    """Pass SAC commands through unless a front obstacle is inside 7 cm.

    This node is intentionally not a navigation or recovery layer. It does not
    slow down, rotate, reverse, freeze angular velocity, or use obstacle memory.
    The route generator and SAC policy remain responsible for navigation.
    """

    def __init__(self) -> None:
        super().__init__("sac_safety_filter_node")
        self.declare_parameter("raw_cmd_topic", "/sac_cmd_vel_raw")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("cmd_timeout_sec", 0.5)
        self.declare_parameter("lidar_min_valid", 0.05)
        self.declare_parameter("emergency_stop_distance", 0.07)
        self.declare_parameter("front_half_angle_deg", 35.0)

        self.cmd_timeout = float(self.get_parameter("cmd_timeout_sec").value)
        self.lidar_min_valid = float(self.get_parameter("lidar_min_valid").value)
        self.emergency_stop_distance = float(self.get_parameter("emergency_stop_distance").value)
        self.front_half_angle = math.radians(float(self.get_parameter("front_half_angle_deg").value))

        self.last_raw_cmd = Twist()
        self.last_raw_time = 0.0
        self.scan: LaserScan | None = None

        self.create_subscription(Twist, str(self.get_parameter("raw_cmd_topic").value), self._raw_cb, 10)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, 10)
        self._cmd_pub = self.create_publisher(Twist, str(self.get_parameter("cmd_vel_topic").value), 10)
        self._state_pub = self.create_publisher(String, "/safety/state", 10)
        self._diag_pub = self.create_publisher(Float32MultiArray, "/safety/diagnostics", 10)
        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            "Emergency-only SAC filter active: if front_min < "
            f"{self.emergency_stop_distance:.3f} m and linear.x > 0, only linear.x is set to 0. "
            "angular.z is preserved exactly."
        )

    def _raw_cb(self, msg: Twist) -> None:
        self.last_raw_cmd = msg
        self.last_raw_time = time.monotonic()

    def _scan_cb(self, msg: LaserScan) -> None:
        self.scan = msg

    def _front_scan_min(self) -> float:
        if self.scan is None or not self.scan.ranges:
            return float("inf")
        ranges = np.asarray(self.scan.ranges, dtype=np.float32)
        angles = float(self.scan.angle_min) + np.arange(len(ranges), dtype=np.float32) * float(self.scan.angle_increment)
        valid = np.isfinite(ranges) & (ranges >= self.lidar_min_valid)
        front_mask = valid & (np.abs(angles) <= self.front_half_angle)
        return float(np.min(ranges[front_mask])) if np.any(front_mask) else float("inf")

    def _publish_state(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._state_pub.publish(msg)

    def _tick(self) -> None:
        raw_age = time.monotonic() - self.last_raw_time
        cmd = Twist()
        state = "NO_RAW_CMD"
        intervention = 0.0

        if raw_age <= self.cmd_timeout:
            cmd = copy.deepcopy(self.last_raw_cmd)
            state = "PASS"

        front_min = self._front_scan_min()
        raw_linear = float(cmd.linear.x)
        raw_angular = float(cmd.angular.z)
        if raw_age <= self.cmd_timeout and raw_linear > 0.0 and front_min < self.emergency_stop_distance:
            cmd.linear.x = 0.0
            cmd.angular.z = raw_angular
            state = "EMERGENCY_LINEAR_STOP"
            intervention = 1.0
            self.get_logger().warn(
                "Emergency filter modified command: "
                f"front_min={front_min:.3f} m threshold={self.emergency_stop_distance:.3f} m "
                f"raw_linear={raw_linear:.3f} final_linear=0.000 angular_preserved={raw_angular:.3f}"
            )

        self._cmd_pub.publish(cmd)
        self._publish_state(state)
        diag = Float32MultiArray()
        diag.data = [
            float(front_min),
            raw_linear,
            raw_angular,
            float(cmd.linear.x),
            float(cmd.angular.z),
            intervention,
        ]
        self._diag_pub.publish(diag)


def main() -> None:
    rclpy.init()
    node = SacSafetyFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node._cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
