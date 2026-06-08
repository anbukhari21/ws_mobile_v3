#!/usr/bin/env python3
"""Live Matplotlib plotter for real-robot SAC trials.

This node is passive. It subscribes to ROS topics only and never publishes
control commands.
"""

import argparse
import math
import time
from collections import deque
from typing import Deque, Optional

import matplotlib.pyplot as plt
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32


def finite_or_nan(value: Optional[float]) -> float:
    if value is None:
        return float("nan")
    value = float(value)
    return value if math.isfinite(value) else float("nan")


class LiveTrialPlotter(Node):
    def __init__(self, window_sec: float, success_radius: float) -> None:
        super().__init__("live_trial_plotter")
        self.start_time = time.time()
        self.window_sec = float(window_sec)
        self.success_radius = float(success_radius)

        self.times: Deque[float] = deque()
        self.goal_distance: Deque[float] = deque()
        self.goal_angle: Deque[float] = deque()
        self.cmd_linear: Deque[float] = deque()
        self.cmd_angular: Deque[float] = deque()
        self.odom_linear: Deque[float] = deque()
        self.odom_angular: Deque[float] = deque()

        self.latest_goal_distance: Optional[float] = None
        self.latest_goal_angle: Optional[float] = None
        self.latest_cmd_linear: Optional[float] = None
        self.latest_cmd_angular: Optional[float] = None
        self.latest_odom_linear: Optional[float] = None
        self.latest_odom_angular: Optional[float] = None

        self.create_subscription(Float32, "/goal_distance", self._goal_distance_cb, 20)
        self.create_subscription(Float32, "/goal_angle", self._goal_angle_cb, 20)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 20)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 20)

        self.get_logger().warn("Live plotter is passive. It does not publish /cmd_vel.")

    def _elapsed(self) -> float:
        return time.time() - self.start_time

    def _goal_distance_cb(self, msg: Float32) -> None:
        self.latest_goal_distance = float(msg.data)

    def _goal_angle_cb(self, msg: Float32) -> None:
        self.latest_goal_angle = float(msg.data)

    def _cmd_cb(self, msg: Twist) -> None:
        self.latest_cmd_linear = float(msg.linear.x)
        self.latest_cmd_angular = float(msg.angular.z)
        self._append_sample()

    def _odom_cb(self, msg: Odometry) -> None:
        self.latest_odom_linear = float(msg.twist.twist.linear.x)
        self.latest_odom_angular = float(msg.twist.twist.angular.z)

    def _append_sample(self) -> None:
        now = self._elapsed()
        self.times.append(now)
        self.goal_distance.append(finite_or_nan(self.latest_goal_distance))
        self.goal_angle.append(finite_or_nan(self.latest_goal_angle))
        self.cmd_linear.append(finite_or_nan(self.latest_cmd_linear))
        self.cmd_angular.append(finite_or_nan(self.latest_cmd_angular))
        self.odom_linear.append(finite_or_nan(self.latest_odom_linear))
        self.odom_angular.append(finite_or_nan(self.latest_odom_angular))
        self._trim_old_samples()

    def _trim_old_samples(self) -> None:
        if self.window_sec <= 0.0:
            return
        cutoff = self._elapsed() - self.window_sec
        while self.times and self.times[0] < cutoff:
            for series in (
                self.times,
                self.goal_distance,
                self.goal_angle,
                self.cmd_linear,
                self.cmd_angular,
                self.odom_linear,
                self.odom_angular,
            ):
                series.popleft()


class PlotWindow:
    def __init__(self, node: LiveTrialPlotter) -> None:
        self.node = node
        self.fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        self.ax_goal, self.ax_linear, self.ax_angular = axes

        (self.line_goal_distance,) = self.ax_goal.plot([], [], label="goal distance")
        (self.line_goal_angle,) = self.ax_goal.plot([], [], label="goal angle")
        self.success_line = self.ax_goal.axhline(
            node.success_radius,
            color="red",
            linestyle="--",
            label=f"success {node.success_radius:.2f} m",
        )

        (self.line_cmd_linear,) = self.ax_linear.plot([], [], label="cmd linear x")
        (self.line_odom_linear,) = self.ax_linear.plot([], [], label="odom linear x")
        (self.line_cmd_angular,) = self.ax_angular.plot([], [], label="cmd angular z")
        (self.line_odom_angular,) = self.ax_angular.plot([], [], label="odom angular z")

        self.ax_goal.set_ylabel("goal")
        self.ax_linear.set_ylabel("linear (m/s)")
        self.ax_angular.set_ylabel("angular (rad/s)")
        self.ax_angular.set_xlabel("time (s)")
        self.fig.suptitle("Live Real-Robot Trial")

        for ax in axes:
            ax.grid(True, alpha=0.3)
            ax.legend(loc="upper right")

    def update(self) -> None:
        x = list(self.node.times)
        if not x:
            plt.pause(0.05)
            return

        self.line_goal_distance.set_data(x, list(self.node.goal_distance))
        self.line_goal_angle.set_data(x, list(self.node.goal_angle))
        self.line_cmd_linear.set_data(x, list(self.node.cmd_linear))
        self.line_odom_linear.set_data(x, list(self.node.odom_linear))
        self.line_cmd_angular.set_data(x, list(self.node.cmd_angular))
        self.line_odom_angular.set_data(x, list(self.node.odom_angular))

        xmin = min(x)
        xmax = max(x) if max(x) > xmin else xmin + 1.0
        for ax in (self.ax_goal, self.ax_linear, self.ax_angular):
            ax.set_xlim(xmin, xmax)
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)

        self.fig.canvas.draw_idle()
        plt.pause(0.05)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Live plot real-robot SAC trial topics.")
    parser.add_argument(
        "--window-sec",
        type=float,
        default=240.0,
        help="Rolling plot window in seconds. Use 0 to keep all samples.",
    )
    parser.add_argument(
        "--success-radius",
        type=float,
        default=0.22,
        help="Success radius line for the goal-distance plot.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = LiveTrialPlotter(window_sec=args.window_sec, success_radius=args.success_radius)
    plotter = PlotWindow(node)
    try:
        while rclpy.ok() and plt.fignum_exists(plotter.fig.number):
            rclpy.spin_once(node, timeout_sec=0.05)
            plotter.update()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
