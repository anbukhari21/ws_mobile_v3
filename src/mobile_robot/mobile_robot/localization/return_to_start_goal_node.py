#!/usr/bin/env python3
"""Publish SAC goal observations for returning to the SLAM start pose.

This node does not command the robot. It only converts the current TF pose in
the SLAM map frame into /goal_distance, /goal_angle, /goal_valid, and
/goal_reached for the SAC controller.
"""

import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from tf2_ros import Buffer, TransformException, TransformListener


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class ReturnToStartGoalNode(Node):
    """Uses SLAM TF to make the map origin the SAC goal."""

    def __init__(self) -> None:
        super().__init__('return_to_start_goal_node')

        self.declare_parameter('pose_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('tf_timeout', 0.05)

        self.goal_distance_pub = self.create_publisher(Float32, '/goal_distance', 10)
        self.goal_angle_pub = self.create_publisher(Float32, '/goal_angle', 10)
        self.goal_valid_pub = self.create_publisher(Bool, '/goal_valid', 10)
        self.goal_reached_pub = self.create_publisher(Bool, '/goal_reached', 10)
        self.goal_debug_pose_pub = self.create_publisher(PoseStamped, '/goal_debug_pose', 10)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_distance = float('inf')
        self.last_angle = 0.0
        self.last_valid = False

        rate = max(0.1, float(self.get_parameter('publish_rate').value))
        self.timer = self.create_timer(1.0 / rate, self._timer_cb)
        self.log_timer = self.create_timer(3.0, self._log_status)

        self.get_logger().info(
            'online_slam_return_start active: target=(0.0, 0.0) in SLAM map frame'
        )

    def _lookup_pose(self) -> Optional[Tuple[float, float, float, PoseStamped]]:
        pose_frame = str(self.get_parameter('pose_frame').value)
        base_frame = str(self.get_parameter('base_frame').value)
        timeout = max(0.0, float(self.get_parameter('tf_timeout').value))

        try:
            transform = self.tf_buffer.lookup_transform(
                pose_frame,
                base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=timeout),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'Waiting for TF {pose_frame} -> {base_frame}: {exc}',
                throttle_duration_sec=2.0,
            )
            return None

        trans = transform.transform.translation
        rot = transform.transform.rotation
        debug_pose = PoseStamped()
        debug_pose.header.stamp = self.get_clock().now().to_msg()
        debug_pose.header.frame_id = pose_frame
        debug_pose.pose.position.x = trans.x
        debug_pose.pose.position.y = trans.y
        debug_pose.pose.position.z = trans.z
        debug_pose.pose.orientation = rot
        return trans.x, trans.y, quaternion_to_yaw(rot), debug_pose

    def _publish_invalid(self) -> None:
        self.last_distance = float('inf')
        self.last_angle = 0.0
        self.last_valid = False
        self.goal_distance_pub.publish(Float32(data=float('inf')))
        self.goal_angle_pub.publish(Float32(data=0.0))
        self.goal_valid_pub.publish(Bool(data=False))
        self.goal_reached_pub.publish(Bool(data=False))

    def _timer_cb(self) -> None:
        pose = self._lookup_pose()
        if pose is None:
            self._publish_invalid()
            return

        robot_x, robot_y, robot_yaw, debug_pose = pose
        target_x = float(self.get_parameter('target_x').value)
        target_y = float(self.get_parameter('target_y').value)
        tolerance = max(0.0, float(self.get_parameter('goal_tolerance').value))

        dx = target_x - robot_x
        dy = target_y - robot_y
        distance = math.hypot(dx, dy)
        angle = normalize_angle(math.atan2(dy, dx) - robot_yaw)

        self.last_distance = distance
        self.last_angle = angle
        self.last_valid = True

        self.goal_distance_pub.publish(Float32(data=float(distance)))
        self.goal_angle_pub.publish(Float32(data=float(angle)))
        self.goal_valid_pub.publish(Bool(data=True))
        self.goal_reached_pub.publish(Bool(data=distance <= tolerance))
        self.goal_debug_pose_pub.publish(debug_pose)

    def _log_status(self) -> None:
        self.get_logger().info(
            f'goal_valid={self.last_valid} '
            f'goal_distance={self.last_distance:.3f}m '
            f'goal_angle={self.last_angle:.3f}rad'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReturnToStartGoalNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
