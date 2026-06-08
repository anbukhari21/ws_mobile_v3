#!/usr/bin/env python3
"""Live endurance timer and battery monitor.

This script is passive. It subscribes only and never publishes motion commands.
"""

import argparse
import csv
import json
import math
import shlex
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Deque, Optional
from collections import deque

import matplotlib.pyplot as plt
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32


BATTERY_STATE_TYPE = "sensor_msgs/msg/BatteryState"
FLOAT32_TYPE = "std_msgs/msg/Float32"
TIMER_GREEN_AFTER_SEC = 60.0 * 60.0
TIMER_NORMAL_COLOR = "black"
TIMER_PASS_COLOR = "green"


def finite_or_nan(value: Optional[float]) -> float:
    if value is None:
        return float("nan")
    value = float(value)
    return value if math.isfinite(value) else float("nan")


def format_hms(seconds: float) -> str:
    total = max(0, int(seconds))
    hours = total // 3600
    minutes = (total % 3600) // 60
    secs = total % 60
    return f"{hours:02d}:{minutes:02d}:{secs:02d}"


class BatteryEnduranceMonitor(Node):
    def __init__(
        self,
        battery_topic: str,
        topic_type: str,
        float_kind: str,
        log_path: str,
        window_sec: float,
        start_offset_sec: float,
        ssh_robot_host: str,
        ssh_workspace: str,
    ) -> None:
        super().__init__("battery_endurance_monitor")
        self.start_time = time.time() - max(0.0, float(start_offset_sec))
        self.window_sec = float(window_sec)
        self.requested_topic = battery_topic
        self.requested_type = topic_type
        self.float_kind = float_kind
        self.active_topic = ""
        self.active_type = ""
        self.subscription = None
        self.ssh_process: Optional[subprocess.Popen] = None
        self.data_lock = threading.Lock()

        self.times: Deque[float] = deque()
        self.voltages: Deque[float] = deque()
        self.percentages: Deque[float] = deque()
        self.currents: Deque[float] = deque()
        self.velocity_times: Deque[float] = deque()
        self.cmd_linear: Deque[float] = deque()
        self.odom_linear: Deque[float] = deque()
        self.cmd_angular: Deque[float] = deque()
        self.odom_angular: Deque[float] = deque()

        self.latest_voltage: Optional[float] = None
        self.latest_percentage: Optional[float] = None
        self.latest_current: Optional[float] = None
        self.latest_status: Optional[int] = None
        self.latest_health: Optional[int] = None
        self.latest_present: Optional[bool] = None
        self.latest_cmd_linear: Optional[float] = None
        self.latest_cmd_angular: Optional[float] = None
        self.latest_odom_linear: Optional[float] = None
        self.latest_odom_angular: Optional[float] = None

        self.csv_file = None
        self.csv_writer = None
        if log_path:
            path = Path(log_path).expanduser().resolve()
            path.parent.mkdir(parents=True, exist_ok=True)
            self.csv_file = path.open("w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(
                [
                    "wall_time",
                    "time_sec",
                    "battery_topic",
                    "battery_type",
                    "voltage",
                    "percentage",
                    "current",
                    "power_supply_status",
                    "power_supply_health",
                    "present",
                ]
            )
            self.csv_file.flush()
            self.get_logger().info(f"Battery endurance CSV: {path}")

        self.detect_timer = self.create_timer(2.0, self._try_configure_subscription)
        if ssh_robot_host:
            self._start_ssh_velocity_stream(ssh_robot_host, ssh_workspace)
        else:
            self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 20)
            self.create_subscription(Odometry, "/odom", self._odom_cb, 20)
        self._try_configure_subscription()
        self.get_logger().warn("Battery endurance monitor is passive. It does not publish /cmd_vel.")

    def destroy_node(self) -> bool:
        if self.ssh_process is not None:
            self.ssh_process.terminate()
            try:
                self.ssh_process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.ssh_process.kill()
            self.ssh_process = None
        if self.csv_file is not None:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None
        return super().destroy_node()

    def _start_ssh_velocity_stream(self, host: str, workspace: str) -> None:
        remote_py = r'''
import json
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class VelocityStream(Node):
    def __init__(self):
        super().__init__("endurance_velocity_stream")
        self.cmd_linear = None
        self.cmd_angular = None
        self.odom_linear = None
        self.odom_angular = None
        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 20)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 20)
        self.create_timer(0.2, self._print_sample)

    def _cmd_cb(self, msg):
        self.cmd_linear = float(msg.linear.x)
        self.cmd_angular = float(msg.angular.z)

    def _odom_cb(self, msg):
        self.odom_linear = float(msg.twist.twist.linear.x)
        self.odom_angular = float(msg.twist.twist.angular.z)

    def _print_sample(self):
        print(json.dumps({
            "cmd_linear": self.cmd_linear,
            "cmd_angular": self.cmd_angular,
            "odom_linear": self.odom_linear,
            "odom_angular": self.odom_angular,
        }), flush=True)


rclpy.init()
node = VelocityStream()
try:
    rclpy.spin(node)
finally:
    node.destroy_node()
    rclpy.shutdown()
'''
        remote_cmd = (
            f"cd {shlex.quote(workspace)} && "
            "source src/mobile_robot/scripts/source_robot_local_v3.sh >/dev/null && "
            f"python3 -u - <<'PY'\n{remote_py}\nPY"
        )
        self.ssh_process = subprocess.Popen(
            ["ssh", host, f"bash -lc {shlex.quote(remote_cmd)}"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        thread = threading.Thread(target=self._read_ssh_velocity_stream, daemon=True)
        thread.start()
        self.get_logger().info(f"Streaming velocity from robot over SSH: {host}")

    def _read_ssh_velocity_stream(self) -> None:
        if self.ssh_process is None or self.ssh_process.stdout is None:
            return
        for line in self.ssh_process.stdout:
            try:
                sample = json.loads(line)
            except json.JSONDecodeError:
                continue
            self._append_velocity_values(
                sample.get("cmd_linear"),
                sample.get("odom_linear"),
                sample.get("cmd_angular"),
                sample.get("odom_angular"),
            )

    def elapsed(self) -> float:
        return time.time() - self.start_time

    def _try_configure_subscription(self) -> None:
        if self.subscription is not None:
            return

        topic, topic_type = self._select_topic()
        if not topic:
            return

        if topic_type == BATTERY_STATE_TYPE:
            self.subscription = self.create_subscription(BatteryState, topic, self._battery_cb, 10)
        elif topic_type == FLOAT32_TYPE:
            self.subscription = self.create_subscription(Float32, topic, self._float_cb, 10)
        else:
            self.get_logger().warn(f"Unsupported battery topic type on {topic}: {topic_type}")
            return

        self.active_topic = topic
        self.active_type = topic_type
        self.get_logger().info(f"Monitoring battery topic {topic} ({topic_type})")

    def _select_topic(self) -> tuple[str, str]:
        topics = self.get_topic_names_and_types()
        type_by_topic = {name: types for name, types in topics}

        if self.requested_topic:
            requested_types = type_by_topic.get(self.requested_topic, [])
            if self.requested_type == "auto":
                if BATTERY_STATE_TYPE in requested_types:
                    return self.requested_topic, BATTERY_STATE_TYPE
                if FLOAT32_TYPE in requested_types:
                    return self.requested_topic, FLOAT32_TYPE
                return "", ""
            expected = BATTERY_STATE_TYPE if self.requested_type == "battery_state" else FLOAT32_TYPE
            if expected in requested_types:
                return self.requested_topic, expected
            return "", ""

        battery_candidates = []
        float_candidates = []
        for name, types in topics:
            lowered = name.lower()
            looks_like_battery = any(
                keyword in lowered for keyword in ("battery", "batt", "volt", "charge", "power")
            )
            if not looks_like_battery:
                continue
            if BATTERY_STATE_TYPE in types:
                battery_candidates.append(name)
            if FLOAT32_TYPE in types:
                float_candidates.append(name)

        if battery_candidates:
            return sorted(battery_candidates)[0], BATTERY_STATE_TYPE
        if float_candidates:
            return sorted(float_candidates)[0], FLOAT32_TYPE
        return "", ""

    def _battery_cb(self, msg: BatteryState) -> None:
        percentage = float(msg.percentage)
        if math.isfinite(percentage) and percentage <= 1.0:
            percentage *= 100.0

        self.latest_voltage = finite_or_nan(float(msg.voltage))
        self.latest_percentage = finite_or_nan(percentage)
        self.latest_current = finite_or_nan(float(msg.current))
        self.latest_status = int(msg.power_supply_status)
        self.latest_health = int(msg.power_supply_health)
        self.latest_present = bool(msg.present)
        self._append_sample()

    def _float_cb(self, msg: Float32) -> None:
        value = finite_or_nan(float(msg.data))
        if self.float_kind == "percentage":
            self.latest_percentage = value
        else:
            self.latest_voltage = value
        self._append_sample()

    def _cmd_cb(self, msg: Twist) -> None:
        self.latest_cmd_linear = float(msg.linear.x)
        self.latest_cmd_angular = float(msg.angular.z)
        self._append_velocity_sample()

    def _odom_cb(self, msg: Odometry) -> None:
        self.latest_odom_linear = float(msg.twist.twist.linear.x)
        self.latest_odom_angular = float(msg.twist.twist.angular.z)

    def _append_sample(self) -> None:
        now = self.elapsed()
        self.times.append(now)
        self.voltages.append(finite_or_nan(self.latest_voltage))
        self.percentages.append(finite_or_nan(self.latest_percentage))
        self.currents.append(finite_or_nan(self.latest_current))
        self._trim_old_samples()
        self._write_csv(now)

    def _append_velocity_sample(self) -> None:
        self._append_velocity_values(
            self.latest_cmd_linear,
            self.latest_odom_linear,
            self.latest_cmd_angular,
            self.latest_odom_angular,
        )

    def _append_velocity_values(
        self,
        cmd_linear: Optional[float],
        odom_linear: Optional[float],
        cmd_angular: Optional[float],
        odom_angular: Optional[float],
    ) -> None:
        now = self.elapsed()
        with self.data_lock:
            self.velocity_times.append(now)
            self.cmd_linear.append(finite_or_nan(cmd_linear))
            self.odom_linear.append(finite_or_nan(odom_linear))
            self.cmd_angular.append(finite_or_nan(cmd_angular))
            self.odom_angular.append(finite_or_nan(odom_angular))
        self._trim_old_samples()

    def velocity_snapshot(self) -> tuple[list[float], list[float], list[float], list[float], list[float]]:
        with self.data_lock:
            return (
                list(self.velocity_times),
                list(self.cmd_linear),
                list(self.odom_linear),
                list(self.cmd_angular),
                list(self.odom_angular),
            )

    def _trim_old_samples(self) -> None:
        if self.window_sec <= 0.0:
            return
        cutoff = self.elapsed() - self.window_sec
        while self.times and self.times[0] < cutoff:
            self.times.popleft()
            self.voltages.popleft()
            self.percentages.popleft()
            self.currents.popleft()
        with self.data_lock:
            while self.velocity_times and self.velocity_times[0] < cutoff:
                self.velocity_times.popleft()
                self.cmd_linear.popleft()
                self.odom_linear.popleft()
                self.cmd_angular.popleft()
                self.odom_angular.popleft()

    def _write_csv(self, elapsed: float) -> None:
        if self.csv_writer is None:
            return
        self.csv_writer.writerow(
            [
                datetime.now().isoformat(timespec="seconds"),
                f"{elapsed:.3f}",
                self.active_topic,
                self.active_type,
                self._fmt(self.latest_voltage),
                self._fmt(self.latest_percentage),
                self._fmt(self.latest_current),
                "" if self.latest_status is None else self.latest_status,
                "" if self.latest_health is None else self.latest_health,
                "" if self.latest_present is None else self.latest_present,
            ]
        )
        self.csv_file.flush()

    @staticmethod
    def _fmt(value: Optional[float]) -> str:
        if value is None or not math.isfinite(float(value)):
            return ""
        return f"{float(value):.6g}"


class MonitorWindow:
    def __init__(self, node: BatteryEnduranceMonitor, timer_only: bool, show_velocity: bool) -> None:
        self.node = node
        self.timer_only = bool(timer_only)
        self.show_velocity = bool(show_velocity)

        self.ax_linear = None
        self.ax_angular = None

        if self.show_velocity:
            self.fig = plt.figure(figsize=(12, 8))
            grid = self.fig.add_gridspec(3, 1, height_ratios=[1, 2, 2])
            self.ax_timer = self.fig.add_subplot(grid[0])
            self.ax_linear = self.fig.add_subplot(grid[1])
            self.ax_angular = self.fig.add_subplot(grid[2], sharex=self.ax_linear)
            self.ax_battery = None
            self.ax_percent = None
            (self.line_cmd_linear,) = self.ax_linear.plot([], [], label="cmd linear x")
            (self.line_odom_linear,) = self.ax_linear.plot([], [], label="odom linear x")
            (self.line_cmd_angular,) = self.ax_angular.plot([], [], label="cmd angular z")
            (self.line_odom_angular,) = self.ax_angular.plot([], [], label="odom angular z")
            self.ax_linear.set_ylabel("linear (m/s)")
            self.ax_angular.set_ylabel("angular (rad/s)")
            self.ax_angular.set_xlabel("time (min)")
            for ax in (self.ax_linear, self.ax_angular):
                ax.grid(True, alpha=0.3)
                ax.legend(loc="upper right")
        elif self.timer_only:
            self.fig, self.ax_timer = plt.subplots(1, 1, figsize=(9, 4))
            self.ax_battery = None
            self.ax_percent = None
        else:
            self.fig = plt.figure(figsize=(12, 7))
            grid = self.fig.add_gridspec(2, 1, height_ratios=[1, 3])
            self.ax_timer = self.fig.add_subplot(grid[0])
            self.ax_battery = self.fig.add_subplot(grid[1])
            self.ax_percent = self.ax_battery.twinx()
            (self.line_voltage,) = self.ax_battery.plot([], [], label="voltage (V)", color="tab:blue")
            (self.line_percentage,) = self.ax_percent.plot(
                [], [], label="percentage (%)", color="tab:orange"
            )
            self.ax_battery.set_xlabel("time (min)")
            self.ax_battery.set_ylabel("voltage (V)")
            self.ax_percent.set_ylabel("percentage (%)")
            self.ax_battery.grid(True, alpha=0.3)

        self.ax_timer.axis("off")
        timer_y = 0.5 if self.timer_only or self.show_velocity else 0.62
        self.timer_text = self.ax_timer.text(
            0.5,
            timer_y,
            "00:00:00",
            ha="center",
            va="center",
            fontsize=44,
            fontweight="bold",
            color=TIMER_NORMAL_COLOR,
            transform=self.ax_timer.transAxes,
        )
        self.status_text = None
        if not self.timer_only and not self.show_velocity:
            self.status_text = self.ax_timer.text(
                0.5,
                0.18,
                "Waiting for battery topic...",
                ha="center",
                va="center",
                fontsize=12,
                transform=self.ax_timer.transAxes,
            )
        self.fig.suptitle("Battery Endurance Test")

    def update(self) -> None:
        elapsed = self.node.elapsed()
        self.timer_text.set_text(format_hms(elapsed))
        self.timer_text.set_color(
            TIMER_PASS_COLOR if elapsed >= TIMER_GREEN_AFTER_SEC else TIMER_NORMAL_COLOR
        )
        if self.status_text is not None:
            self.status_text.set_text(self._status_line())

        if self.show_velocity and self.ax_linear is not None and self.ax_angular is not None:
            x, cmd_linear, odom_linear, cmd_angular, odom_angular = self.node.velocity_snapshot()
            x_min = [value / 60.0 for value in x]
            self.line_cmd_linear.set_data(x_min, cmd_linear)
            self.line_odom_linear.set_data(x_min, odom_linear)
            self.line_cmd_angular.set_data(x_min, cmd_angular)
            self.line_odom_angular.set_data(x_min, odom_angular)
            if x_min:
                xmin = min(x_min)
                xmax = max(x_min) if max(x_min) > xmin else xmin + 1.0 / 60.0
                for ax in (self.ax_linear, self.ax_angular):
                    ax.set_xlim(xmin, xmax)
                    ax.relim()
                    ax.autoscale_view(scalex=False, scaley=True)

        if not self.timer_only and self.ax_battery is not None and self.ax_percent is not None:
            x_min = [value / 60.0 for value in self.node.times]
            self.line_voltage.set_data(x_min, list(self.node.voltages))
            self.line_percentage.set_data(x_min, list(self.node.percentages))

            if x_min:
                xmin = min(x_min)
                xmax = max(x_min) if max(x_min) > xmin else xmin + 1.0 / 60.0
                self.ax_battery.set_xlim(xmin, xmax)
            self.ax_battery.relim()
            self.ax_battery.autoscale_view(scalex=False, scaley=True)
            self.ax_percent.relim()
            self.ax_percent.autoscale_view(scalex=False, scaley=True)

            voltage_handles, voltage_labels = self.ax_battery.get_legend_handles_labels()
            percent_handles, percent_labels = self.ax_percent.get_legend_handles_labels()
            self.ax_battery.legend(
                voltage_handles + percent_handles,
                voltage_labels + percent_labels,
                loc="upper right",
            )

        self.fig.canvas.draw_idle()
        plt.pause(0.05)

    def _status_line(self) -> str:
        topic = self.node.active_topic or "no battery topic detected"
        voltage = self.node.latest_voltage
        percentage = self.node.latest_percentage
        current = self.node.latest_current
        parts = [topic]
        if voltage is not None and math.isfinite(voltage):
            parts.append(f"{voltage:.2f} V")
        if percentage is not None and math.isfinite(percentage):
            parts.append(f"{percentage:.1f}%")
        if current is not None and math.isfinite(current):
            parts.append(f"{current:.2f} A")
        return " | ".join(parts)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Show a battery endurance timer and drain graph.")
    parser.add_argument(
        "--battery-topic",
        default="",
        help="Battery topic to monitor. Empty means auto-detect.",
    )
    parser.add_argument(
        "--topic-type",
        choices=["auto", "battery_state", "float32"],
        default="auto",
        help="Battery topic type. Float32 is treated as voltage unless --float-kind says percentage.",
    )
    parser.add_argument(
        "--float-kind",
        choices=["voltage", "percentage"],
        default="voltage",
        help="Meaning of a Float32 battery topic.",
    )
    parser.add_argument(
        "--log-path",
        default="",
        help="Optional CSV path for endurance battery samples.",
    )
    parser.add_argument(
        "--window-sec",
        type=float,
        default=0.0,
        help="Rolling graph window. Use 0 to keep the full test.",
    )
    parser.add_argument(
        "--timer-only",
        action="store_true",
        help="Show only a large elapsed-time clock.",
    )
    parser.add_argument(
        "--show-velocity",
        action="store_true",
        help="Show commanded and odometry linear/angular velocity below the timer.",
    )
    parser.add_argument(
        "--ssh-robot-host",
        default="",
        help="Optional robot SSH host for streaming /cmd_vel and /odom when laptop ROS discovery is unavailable.",
    )
    parser.add_argument(
        "--ssh-workspace",
        default="/home/abdulrhman/ws_mobile_v3",
        help="Robot workspace path used with --ssh-robot-host.",
    )
    parser.add_argument(
        "--start-offset-sec",
        type=float,
        default=0.0,
        help="Initial elapsed time offset for restarting the timer during a running test.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = BatteryEnduranceMonitor(
        battery_topic=args.battery_topic,
        topic_type=args.topic_type,
        float_kind=args.float_kind,
        log_path=args.log_path,
        window_sec=args.window_sec,
        start_offset_sec=args.start_offset_sec,
        ssh_robot_host=args.ssh_robot_host,
        ssh_workspace=args.ssh_workspace,
    )
    window = MonitorWindow(node, timer_only=args.timer_only, show_velocity=args.show_velocity)
    try:
        while rclpy.ok() and plt.fignum_exists(window.fig.number):
            rclpy.spin_once(node, timeout_sec=0.05)
            window.update()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
