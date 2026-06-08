#!/usr/bin/env python3
"""Call AMCL global localization and rotate slowly for convergence."""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_srvs.srv import Empty


class GlobalAmclAutorotateNode(Node):
    """Startup helper for real-robot AMCL global localization.

    The node publishes /cmd_vel only during the requested autorotation window,
    then publishes a zero Twist and stays idle. It is intended to run before the
    SAC motion stack starts.
    """

    def __init__(self) -> None:
        super().__init__("global_amcl_autorotate_node")

        self.declare_parameter("auto_global_localization", True)
        self.declare_parameter("auto_rotate", True)
        self.declare_parameter("rotate_speed", 0.20)
        self.declare_parameter("rotate_duration", 31.5)
        self.declare_parameter("start_delay", 3.0)
        self.declare_parameter("service_wait_timeout", 120.0)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.client = self.create_client(Empty, "/reinitialize_global_localization")
        self.started = False

        start_delay = max(0.1, float(self.get_parameter("start_delay").value))
        self.timer = self.create_timer(start_delay, self._start_once)
        self.get_logger().info(
            "Global AMCL autorotate helper ready: "
            f"cmd_vel_topic={self.cmd_vel_topic} start_delay={start_delay:.1f}s"
        )

    def _start_once(self) -> None:
        if self.started:
            return
        self.started = True
        self.timer.cancel()

        if not bool(self.get_parameter("auto_global_localization").value):
            self.get_logger().info("auto_global_localization is false; helper is idle.")
            return

        wait_timeout = max(1.0, float(self.get_parameter("service_wait_timeout").value))
        self.get_logger().info(
            "Waiting for /reinitialize_global_localization "
            f"for up to {wait_timeout:.1f}s..."
        )
        deadline = time.monotonic() + wait_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self.client.wait_for_service(timeout_sec=1.0):
                break
        else:
            self.get_logger().error("AMCL global localization service is not available.")
            return

        future = self.client.call_async(Empty.Request())
        future.add_done_callback(self._global_localization_done)

    def _global_localization_done(self, future) -> None:
        try:
            future.result()
            self.get_logger().info("Called /reinitialize_global_localization.")
        except Exception as exc:
            self.get_logger().error(f"Global localization call failed: {exc}")
            return

        if bool(self.get_parameter("auto_rotate").value):
            self._rotate_for_localization()
        else:
            self.get_logger().info("auto_rotate is false; not rotating.")

    def _rotate_for_localization(self) -> None:
        speed = float(self.get_parameter("rotate_speed").value)
        duration = max(0.0, float(self.get_parameter("rotate_duration").value))
        self.get_logger().warn(
            "Auto-rotating slowly for AMCL convergence: "
            f"speed={speed:.2f} rad/s duration={duration:.1f}s"
        )

        cmd = Twist()
        cmd.angular.z = speed
        end_time = time.monotonic() + duration

        while rclpy.ok() and time.monotonic() < end_time:
            self.cmd_pub.publish(cmd)
            time.sleep(0.1)

        self.cmd_pub.publish(Twist())
        self.get_logger().info("Auto-rotation complete; published zero velocity.")

    def destroy_node(self) -> bool:
        try:
            if rclpy.ok():
                self.cmd_pub.publish(Twist())
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GlobalAmclAutorotateNode()
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
