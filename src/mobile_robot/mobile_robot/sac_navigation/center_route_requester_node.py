#!/usr/bin/env python3
"""Request autonomous routes from the current AMCL pose to the arena center."""

from __future__ import annotations

import math
import importlib
from typing import Optional

import rclpy
import rclpy.duration
import rclpy.time
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Empty

from mobile_robot.sac_navigation.utils import yaw_to_quaternion

ComputePathToPose = importlib.import_module("nav" + "2_msgs.action").ComputePathToPose


class CenterRouteRequester(Node):
    def __init__(self) -> None:
        super().__init__("center_route_requester")
        self.declare_parameter("goal_x", 0.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_yaw", 0.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("planner_id", "GridBased")
        self.declare_parameter("action_name", "compute_path_to_pose")
        self.declare_parameter("request_period_sec", 1.0)
        self.declare_parameter("min_replan_distance", 0.12)
        self.declare_parameter("goal_tolerance", 0.12)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)
        self.goal_yaw = float(self.get_parameter("goal_yaw").value)
        self.planner_id = str(self.get_parameter("planner_id").value)
        self.min_replan_distance = float(self.get_parameter("min_replan_distance").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._client = ActionClient(
            self,
            ComputePathToPose,
            str(self.get_parameter("action_name").value),
        )
        self._path_pub = self.create_publisher(Path, "/plan", 10)
        self._nomotion_client = self.create_client(Empty, "/request_nomotion_update")
        self._pending = False
        self._last_request_pose: Optional[tuple[float, float]] = None
        self._last_path: Optional[Path] = None
        self._last_path_pose_count = 0
        self._last_nomotion_request_time = 0.0

        period = max(0.2, float(self.get_parameter("request_period_sec").value))
        self.create_timer(period, self._tick)

    def _request_nomotion_update(self) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if now_sec - self._last_nomotion_request_time < 1.0:
            return
        self._last_nomotion_request_time = now_sec
        if not self._nomotion_client.service_is_ready():
            return
        self._nomotion_client.call_async(Empty.Request())

    def _robot_pose(self) -> Optional[tuple[float, float]]:
        try:
            transform = self._tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.25),
            )
        except Exception as exc:
            self._request_nomotion_update()
            self.get_logger().warn(f"Waiting for {self.map_frame}->{self.base_frame} TF: {exc}", throttle_duration_sec=5.0)
            return None
        return (
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
        )

    def _tick(self) -> None:
        pose = self._robot_pose()
        if pose is None or self._pending:
            return
        distance_to_goal = math.hypot(self.goal_x - pose[0], self.goal_y - pose[1])
        if distance_to_goal <= self.goal_tolerance:
            return
        if self._last_request_pose is not None:
            moved = math.hypot(pose[0] - self._last_request_pose[0], pose[1] - self._last_request_pose[1])
            if moved < self.min_replan_distance and self._last_path_pose_count > 0:
                if self._last_path is not None:
                    self._last_path.header.stamp = self.get_clock().now().to_msg()
                    self._path_pub.publish(self._last_path)
                return
        if not self._client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn("Waiting for route action server.", throttle_duration_sec=5.0)
            return

        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = self.map_frame
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.pose.position.x = self.goal_x
        goal.goal.pose.position.y = self.goal_y
        goal.goal.pose.orientation = yaw_to_quaternion(self.goal_yaw)
        goal.planner_id = self.planner_id
        goal.use_start = False

        self._pending = True
        self._last_request_pose = pose
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._pending = False
            self.get_logger().error(f"Path request failed: {exc}")
            return
        if not goal_handle.accepted:
            self._pending = False
            self.get_logger().warn("Route service rejected center request.")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._path_result_cb)

    def _path_result_cb(self, future) -> None:
        self._pending = False
        try:
            result = future.result().result
        except Exception as exc:
            self.get_logger().error(f"Route result failed: {exc}")
            return
        path = result.path
        if len(path.poses) < 2:
            self.get_logger().warn("Route service returned an empty/invalid center route; keeping previous valid route.", throttle_duration_sec=5.0)
            return
        self._last_path_pose_count = len(path.poses)
        if not path.header.frame_id:
            path.header.frame_id = self.map_frame
        self._last_path = path
        self._path_pub.publish(path)
        self.get_logger().info(f"Published center route with {len(path.poses)} poses.", throttle_duration_sec=5.0)


def main() -> None:
    rclpy.init()
    node = CenterRouteRequester()
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
