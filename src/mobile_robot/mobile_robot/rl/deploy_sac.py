#!/usr/bin/env python3
"""Standalone deployment node: loads a trained SAC model and drives the robot to a goal.

Usage (direct):
    python3 -m mobile_robot.rl.deploy_sac <model_path> [goal_x] [goal_y]

Usage (via launch file):
    ros2 launch mobile_robot slam_sac_deploy.launch.py \
        model_path:=/abs/path/model.zip goal_x:=1.5 goal_y:=-0.5
"""

import math
import sys

import numpy as np
import rclpy
import rclpy.duration
import rclpy.time
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from stable_baselines3 import SAC

from mobile_robot.rl.stam_policy import STAMFeatureExtractor

_NUM_LIDAR_BEAMS = 36
_HISTORY_LENGTH = 8
_MAX_LINEAR_VEL = 0.25
_MAX_ANGULAR_VEL = 1.20
_GOAL_THRESHOLD = 0.2
_MAX_RANGE = 4.0

# Velocity smoothing — must match robot_nav_env.py step() to stay on-distribution
_LINEAR_ALPHA = 0.45
_ANGULAR_ALPHA = 0.55
_ACTION_GUARD_ENABLED = True
_ACTION_GUARD_GOAL_ANGLE = 0.70
_ACTION_GUARD_FORWARD_ANGLE = 0.70
_ACTION_GUARD_FRONT_CLEARANCE = 0.80
_ACTION_GUARD_MIN_TURN_VEL = 0.50
_ACTION_GUARD_ANGULAR_SCALE = 0.25
_ACTION_GUARD_MIN_FORWARD_VEL = 0.22
_ACTION_GUARD_TURN_TO_GOAL = True
_ACTION_GUARD_TURN_TO_GOAL_ANGLE = 0.90
_ACTION_GUARD_STOP_FRONT = True
_ACTION_GUARD_OBSTACLE_TURN_VEL = 0.55
_ACTION_GUARD_OBSTACLE_FORWARD_VEL = 0.0
_ACTION_GUARD_OBSTACLE_STOP_DISTANCE = 0.65
_COLLISION_DISTANCE = 0.22
_NEAR_OBSTACLE_DISTANCE = 0.55


class DeploySACNode(Node):
    def __init__(self, model_path: str, goal_x: float, goal_y: float) -> None:
        super().__init__('deploy_sac')

        self.goal_x = goal_x
        self.goal_y = goal_y

        self.current_lidar = np.full(_NUM_LIDAR_BEAMS, _MAX_RANGE, dtype=np.float32)
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.pose_ready = False
        self.scan_ready = False

        self.last_linear_cmd = 0.0
        self.last_angular_cmd = 0.0

        self.observation_history = np.zeros(
            (_HISTORY_LENGTH, _NUM_LIDAR_BEAMS + 4), dtype=np.float32
        )

        self.get_logger().info(f'Loading model from {model_path}')
        try:
            self.model = SAC.load(
                model_path,
                custom_objects={"features_extractor_class": STAMFeatureExtractor},
            )
        except Exception as exc:
            print(
                f"ERROR: failed to load model — {exc}\n"
                "If the model was trained with STAMFeatureExtractor, ensure that\n"
                "  from mobile_robot.rl.stam_policy import STAMFeatureExtractor\n"
                "is importable in this environment and that stam_policy.py is on PYTHONPATH."
            )
            raise
        self.get_logger().info(f'Model loaded — goal ({goal_x:.3f}, {goal_y:.3f})')

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self._scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)

        self._print_ticks = 0
        self._done = False
        self.create_timer(0.1, self._control_loop)  # 10 Hz

    # ------------------------------------------------------------------
    # Sensor callbacks
    # ------------------------------------------------------------------

    def _scan_callback(self, msg: LaserScan) -> None:
        ranges = np.array(msg.ranges, dtype=np.float32)
        sensor_max_range = float(msg.range_max) if msg.range_max > 0.0 else _MAX_RANGE
        ranges = np.where(np.isfinite(ranges), ranges, sensor_max_range)
        ranges = np.clip(ranges, 0.0, sensor_max_range)
        obs_ranges = np.clip(ranges, 0.0, _MAX_RANGE)

        if len(obs_ranges) >= _NUM_LIDAR_BEAMS:
            indices = np.linspace(0, len(obs_ranges) - 1, _NUM_LIDAR_BEAMS, dtype=int)
            self.current_lidar = obs_ranges[indices]
        else:
            padded = np.full(_NUM_LIDAR_BEAMS, _MAX_RANGE, dtype=np.float32)
            padded[: len(obs_ranges)] = obs_ranges
            self.current_lidar = padded
        self.scan_ready = True

    def _odom_callback(self, msg: Odometry) -> None:
        self.current_linear_velocity = float(msg.twist.twist.linear.x)
        self.current_angular_velocity = float(msg.twist.twist.angular.z)

    # ------------------------------------------------------------------
    # Pose via tf2 (map -> base_link)
    # ------------------------------------------------------------------

    def _get_slam_pose(self) -> bool:
        for base_frame in ("base_footprint", "base_link"):
            try:
                t = self._tf_buffer.lookup_transform(
                    "map",
                    base_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.0),
                )
                self.robot_x = float(t.transform.translation.x)
                self.robot_y = float(t.transform.translation.y)
                q = t.transform.rotation
                siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
                self.pose_ready = True
                return True
            except Exception:
                continue
        # Keep last known pose on tf failure.
        return False

    # ------------------------------------------------------------------
    # Observation building — must exactly mirror robot_nav_env.py
    # ------------------------------------------------------------------

    def _build_single_obs(self) -> np.ndarray:
        lidar_norm = np.clip(self.current_lidar / _MAX_RANGE, 0.0, 1.0)

        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        distance = math.hypot(dx, dy)
        heading = math.atan2(dy, dx)
        angle = math.atan2(
            math.sin(heading - self.robot_yaw),
            math.cos(heading - self.robot_yaw),
        )

        return np.concatenate([
            lidar_norm.astype(np.float32),
            np.array(
                [
                    float(np.clip(distance / 5.0, 0.0, 1.0)),
                    float(np.clip(angle / math.pi, -1.0, 1.0)),
                    float(np.clip(self.current_linear_velocity / _MAX_LINEAR_VEL, -1.0, 1.0)),
                    float(np.clip(self.current_angular_velocity / _MAX_ANGULAR_VEL, -1.0, 1.0)),
                ],
                dtype=np.float32,
            ),
        ])

    # ------------------------------------------------------------------
    # Control loop (10 Hz timer)
    # ------------------------------------------------------------------

    def _apply_action_guard(
        self,
        linear_cmd: float,
        angular_cmd: float,
        goal_angle: float,
        front_min: float,
    ) -> tuple[float, float]:
        if not _ACTION_GUARD_ENABLED:
            return linear_cmd, angular_cmd

        abs_goal_angle = abs(goal_angle)
        front_clear = front_min >= _ACTION_GUARD_FRONT_CLEARANCE

        if front_clear:
            desired_angular = float(
                np.clip(
                    1.2 * goal_angle,
                    -_MAX_ANGULAR_VEL,
                    _MAX_ANGULAR_VEL,
                )
            )
            if abs_goal_angle > 0.08:
                angular_cmd = desired_angular
            if abs_goal_angle <= _ACTION_GUARD_GOAL_ANGLE:
                turning_away = goal_angle * angular_cmd < 0.0
                if turning_away or abs(angular_cmd) > max(abs(desired_angular), 0.08):
                    angular_cmd = desired_angular
                linear_cmd = max(linear_cmd, _ACTION_GUARD_MIN_FORWARD_VEL)
            elif _ACTION_GUARD_TURN_TO_GOAL and abs_goal_angle <= _ACTION_GUARD_TURN_TO_GOAL_ANGLE:
                linear_cmd = max(
                    linear_cmd,
                    min(_ACTION_GUARD_MIN_FORWARD_VEL * 0.5, _MAX_LINEAR_VEL * 0.25),
                )

        if _ACTION_GUARD_TURN_TO_GOAL and abs_goal_angle > _ACTION_GUARD_TURN_TO_GOAL_ANGLE:
            turn_sign = 1.0 if goal_angle >= 0.0 else -1.0
            if goal_angle * angular_cmd <= 0.0:
                angular_cmd = turn_sign * _ACTION_GUARD_MIN_TURN_VEL
            elif abs(angular_cmd) < _ACTION_GUARD_MIN_TURN_VEL:
                angular_cmd = turn_sign * _ACTION_GUARD_MIN_TURN_VEL
            linear_cmd = 0.0

        if (
            abs_goal_angle <= _ACTION_GUARD_GOAL_ANGLE
            and abs(angular_cmd) >= _ACTION_GUARD_MIN_TURN_VEL
        ):
            aligned_spin = abs_goal_angle <= _ACTION_GUARD_FORWARD_ANGLE
            turning_away = goal_angle * angular_cmd < 0.0
            if aligned_spin or turning_away:
                angular_cmd *= _ACTION_GUARD_ANGULAR_SCALE

        if (
            abs_goal_angle <= _ACTION_GUARD_FORWARD_ANGLE
            and front_clear
            and linear_cmd < _ACTION_GUARD_MIN_FORWARD_VEL
        ):
            linear_cmd = _ACTION_GUARD_MIN_FORWARD_VEL

        if _ACTION_GUARD_STOP_FRONT and front_min < _ACTION_GUARD_FRONT_CLEARANCE:
            if abs(angular_cmd) < _ACTION_GUARD_OBSTACLE_TURN_VEL:
                lidar = np.asarray(self.current_lidar, dtype=np.float32)
                if lidar.size >= 24:
                    right_clear = float(np.min(lidar[9:18]))
                    left_clear = float(np.min(lidar[18:27]))
                    turn_sign = 1.0 if left_clear >= right_clear else -1.0
                else:
                    turn_sign = 1.0 if goal_angle >= 0.0 else -1.0
                angular_cmd = turn_sign * _ACTION_GUARD_OBSTACLE_TURN_VEL

            if (
                _ACTION_GUARD_OBSTACLE_FORWARD_VEL > 0.0
                and front_min >= _ACTION_GUARD_OBSTACLE_STOP_DISTANCE
            ):
                linear_cmd = min(
                    max(linear_cmd, _ACTION_GUARD_OBSTACLE_FORWARD_VEL),
                    _ACTION_GUARD_OBSTACLE_FORWARD_VEL,
                )
            elif linear_cmd > 0.0:
                linear_cmd = 0.0

        global_min = float(np.min(self.current_lidar))
        if global_min < _NEAR_OBSTACLE_DISTANCE:
            lidar = np.asarray(self.current_lidar, dtype=np.float32)
            if lidar.size >= 24:
                right_clear = float(np.min(lidar[9:18]))
                left_clear = float(np.min(lidar[18:27]))
                turn_sign = 1.0 if left_clear >= right_clear else -1.0
            else:
                turn_sign = 1.0 if goal_angle >= 0.0 else -1.0
            if abs(angular_cmd) < _ACTION_GUARD_OBSTACLE_TURN_VEL:
                angular_cmd = turn_sign * _ACTION_GUARD_OBSTACLE_TURN_VEL
            obstacle_is_in_front = front_min < _NEAR_OBSTACLE_DISTANCE
            emergency_close = global_min < _COLLISION_DISTANCE + 0.04
            if (
                (obstacle_is_in_front or emergency_close)
                and linear_cmd > _ACTION_GUARD_OBSTACLE_FORWARD_VEL
            ):
                linear_cmd = _ACTION_GUARD_OBSTACLE_FORWARD_VEL

        return linear_cmd, angular_cmd

    def _publish_stop(self) -> None:
        if rclpy.ok():
            self.cmd_vel_pub.publish(Twist())

    def _control_loop(self) -> None:
        if self._done:
            return

        pose_ok = self._get_slam_pose()

        self._print_ticks += 1
        if not pose_ok:
            if self._print_ticks >= 20:
                self.get_logger().warn(
                    'Waiting for map -> base_footprint/base_link TF. '
                    'Start SLAM or AMCL and set the initial pose before running SAC.'
                )
                self._print_ticks = 0
            return

        if not self.scan_ready:
            if self._print_ticks >= 20:
                self.get_logger().warn('Waiting for /scan before running SAC.')
                self._print_ticks = 0
            return

        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        distance = math.hypot(dx, dy)
        heading = math.atan2(dy, dx)
        goal_angle = math.atan2(
            math.sin(heading - self.robot_yaw),
            math.cos(heading - self.robot_yaw),
        )

        # Print goal distance every ~2 s (20 ticks × 100 ms)
        if self._print_ticks >= 20:
            self.get_logger().info(f'Goal distance: {distance:.3f} m')
            self._print_ticks = 0

        if distance < _GOAL_THRESHOLD:
            self.get_logger().info('Goal reached! Stopping.')
            self._publish_stop()
            self._done = True
            rclpy.shutdown()
            return

        # Build observation and run inference
        single_obs = self._build_single_obs()
        self.observation_history[:-1] = self.observation_history[1:]
        self.observation_history[-1] = single_obs
        obs = self.observation_history.flatten().astype(np.float32)

        action, _ = self.model.predict(obs, deterministic=True)
        action = np.clip(action, -1.0, 1.0)

        # Action scaling — matches robot_nav_env.py step()
        front_min = float(np.min(self.current_lidar[15:21]))
        if action[0] >= 0.0:
            linear_vel = float(action[0] * _MAX_LINEAR_VEL)
        else:
            reverse_scale = 0.14 if front_min < 0.28 else 0.06
            linear_vel = float(action[0] * reverse_scale)
        angular_vel = float(action[1] * _MAX_ANGULAR_VEL)

        # Velocity smoothing — matches robot_nav_env.py step()
        smoothed_linear = _LINEAR_ALPHA * linear_vel + (1.0 - _LINEAR_ALPHA) * self.last_linear_cmd
        smoothed_angular = _ANGULAR_ALPHA * angular_vel + (1.0 - _ANGULAR_ALPHA) * self.last_angular_cmd
        smoothed_linear, smoothed_angular = self._apply_action_guard(
            smoothed_linear,
            smoothed_angular,
            goal_angle,
            front_min,
        )
        self.last_linear_cmd = smoothed_linear
        self.last_angular_cmd = smoothed_angular

        cmd = Twist()
        cmd.linear.x = smoothed_linear
        cmd.angular.z = smoothed_angular
        self.cmd_vel_pub.publish(cmd)


def main() -> None:
    # Strip ROS args (everything from '--ros-args' onward) before parsing positional args
    raw = sys.argv[1:]
    pos_args = []
    for a in raw:
        if a.startswith('--'):
            break
        pos_args.append(a)

    if not pos_args:
        print('Usage: deploy_sac.py <model_path> [goal_x] [goal_y]', file=sys.stderr)
        sys.exit(1)

    model_path = pos_args[0]
    goal_x = float(pos_args[1]) if len(pos_args) > 1 else 0.0
    goal_y = float(pos_args[2]) if len(pos_args) > 2 else 0.0

    rclpy.init()
    node = DeploySACNode(model_path, goal_x, goal_y)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
