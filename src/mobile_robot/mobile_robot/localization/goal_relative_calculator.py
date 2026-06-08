#!/usr/bin/env python3
"""Publish goal distance and heading error from AMCL/TF for experiment logging."""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from tf2_ros import Buffer, TransformException, TransformListener


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class GoalRelativeCalculator(Node):
    """Publish /goal_distance, /goal_angle, /goal_valid, and /goal_reached."""

    def __init__(self) -> None:
        super().__init__("goal_relative_calculator")

        self.declare_parameter("goal_x", 0.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_theta", 0.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("goal_tolerance_distance", 0.20)
        self.declare_parameter("goal_tolerance_angle", 0.35)
        self.declare_parameter("goal_reached_require_heading", False)
        self.declare_parameter("publish_rate", 10.0)

        self.goal_distance_pub = self.create_publisher(Float32, "/goal_distance", 10)
        self.goal_angle_pub = self.create_publisher(Float32, "/goal_angle", 10)
        self.goal_valid_pub = self.create_publisher(Bool, "/goal_valid", 10)
        self.goal_reached_pub = self.create_publisher(Bool, "/goal_reached", 10)

        self.amcl_pose: Optional[PoseWithCovarianceStamped] = None
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._amcl_pose_cb, 10)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        rate = max(0.1, float(self.get_parameter("publish_rate").value))
        self.create_timer(1.0 / rate, self._timer_cb)
        self.create_timer(3.0, self._log_status)

        self.last_distance = float("inf")
        self.last_angle = 0.0
        self.last_pose_source = "none"
        self.get_logger().info(
            "Goal relative calculator ready: publishes /goal_distance, "
            "/goal_angle, /goal_valid, /goal_reached"
        )

    def _amcl_pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        self.amcl_pose = msg

    def _robot_pose(self) -> Optional[tuple[float, float, float, str]]:
        map_frame = str(self.get_parameter("map_frame").value)
        base_frame = str(self.get_parameter("base_frame").value)

        try:
            transform = self.tf_buffer.lookup_transform(
                map_frame,
                base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
            trans = transform.transform.translation
            rot = transform.transform.rotation
            return float(trans.x), float(trans.y), quaternion_to_yaw(rot), "tf"
        except TransformException:
            pass

        if self.amcl_pose is None:
            return None

        pose = self.amcl_pose.pose.pose
        return (
            float(pose.position.x),
            float(pose.position.y),
            quaternion_to_yaw(pose.orientation),
            "amcl_pose",
        )

    def _timer_cb(self) -> None:
        pose = self._robot_pose()
        if pose is None:
            self.last_distance = float("inf")
            self.last_angle = 0.0
            self.last_pose_source = "none"
            self.goal_distance_pub.publish(Float32(data=float("inf")))
            self.goal_angle_pub.publish(Float32(data=0.0))
            self.goal_valid_pub.publish(Bool(data=False))
            self.goal_reached_pub.publish(Bool(data=False))
            return

        robot_x, robot_y, robot_yaw, source = pose
        goal_x = float(self.get_parameter("goal_x").value)
        goal_y = float(self.get_parameter("goal_y").value)
        goal_theta = float(self.get_parameter("goal_theta").value)
        distance_tol = float(self.get_parameter("goal_tolerance_distance").value)
        angle_tol = float(self.get_parameter("goal_tolerance_angle").value)
        require_heading = bool(self.get_parameter("goal_reached_require_heading").value)

        dx = goal_x - robot_x
        dy = goal_y - robot_y
        distance = math.hypot(dx, dy)
        angle_to_goal = normalize_angle(math.atan2(dy, dx) - robot_yaw)
        heading_error = abs(normalize_angle(goal_theta - robot_yaw))
        reached = distance <= distance_tol and (
            (not require_heading) or heading_error <= angle_tol
        )

        self.last_distance = distance
        self.last_angle = angle_to_goal
        self.last_pose_source = source

        self.goal_distance_pub.publish(Float32(data=float(distance)))
        self.goal_angle_pub.publish(Float32(data=float(angle_to_goal)))
        self.goal_valid_pub.publish(Bool(data=True))
        self.goal_reached_pub.publish(Bool(data=bool(reached)))

    def _log_status(self) -> None:
        self.get_logger().info(
            f"goal_distance={self.last_distance:.3f}m "
            f"goal_angle={self.last_angle:.3f}rad "
            f"pose_source={self.last_pose_source}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalRelativeCalculator()
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
