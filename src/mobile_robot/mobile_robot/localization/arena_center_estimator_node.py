#!/usr/bin/env python3
"""Estimate circular arena center from LiDAR and publish SAC goal observations.

This node is intentionally independent of AMCL and SLAM. It treats the circular
wall as the geometric reference and rejects nearby boxes/people as outliers.
It does not command the robot.
"""

import math
import time
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Int32


CircleFit = Tuple[float, float, float, float, int]


class ArenaCenterEstimatorNode(Node):
    """Fits the circular wall and publishes the arena center in robot/LiDAR frame."""

    def __init__(self) -> None:
        super().__init__('arena_center_estimator_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('expected_radius', 2.0)
        self.declare_parameter('radius_tolerance', 0.35)
        self.declare_parameter('min_range', 0.15)
        self.declare_parameter('max_range', 4.5)
        self.declare_parameter('min_wall_range', 1.0)
        self.declare_parameter('min_inliers', 25)
        self.declare_parameter('max_fit_error', 0.15)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('use_ransac', True)
        self.declare_parameter('ransac_iterations', 100)
        self.declare_parameter('ransac_threshold', 0.08)
        self.declare_parameter('last_valid_timeout', 1.5)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('max_center_jump', 0.35)
        self.declare_parameter('center_smoothing_alpha', 0.45)

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, 10)

        self.goal_distance_pub = self.create_publisher(Float32, '/goal_distance', 10)
        self.goal_angle_pub = self.create_publisher(Float32, '/goal_angle', 10)
        self.goal_valid_pub = self.create_publisher(Bool, '/goal_valid', 10)
        self.goal_reached_pub = self.create_publisher(Bool, '/goal_reached', 10)
        self.center_pub = self.create_publisher(PointStamped, '/arena_center_point', 10)
        self.radius_pub = self.create_publisher(Float32, '/arena_fit_radius', 10)
        self.error_pub = self.create_publisher(Float32, '/arena_fit_error', 10)
        self.inliers_pub = self.create_publisher(Int32, '/arena_fit_inliers', 10)

        self.latest_scan: Optional[LaserScan] = None
        self.last_fit: Optional[CircleFit] = None
        self.last_valid_time: Optional[float] = None
        self.last_valid = False
        self.rng = np.random.default_rng()

        rate = max(0.1, float(self.get_parameter('publish_rate').value))
        self.timer = self.create_timer(1.0 / rate, self._timer_cb)
        self.log_timer = self.create_timer(3.0, self._log_status)

        self.get_logger().info(
            'arena_center_lidar active: fitting circular arena wall from /scan'
        )

    def _scan_cb(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def _scan_to_points(self, scan: LaserScan) -> np.ndarray:
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        if ranges.size == 0:
            return np.empty((0, 2), dtype=np.float64)

        min_range = max(0.0, float(self.get_parameter('min_range').value))
        max_range = max(min_range, float(self.get_parameter('max_range').value))
        min_wall_range = max(min_range, float(self.get_parameter('min_wall_range').value))

        angles = scan.angle_min + np.arange(ranges.size, dtype=np.float64) * scan.angle_increment
        finite = np.isfinite(ranges)
        wall_candidates = (
            finite
            & (ranges >= min_wall_range)
            & (ranges >= min_range)
            & (ranges <= max_range)
        )
        if not np.any(wall_candidates):
            return np.empty((0, 2), dtype=np.float64)

        candidate_ranges = ranges[wall_candidates]
        candidate_angles = angles[wall_candidates]
        xs = candidate_ranges * np.cos(candidate_angles)
        ys = candidate_ranges * np.sin(candidate_angles)
        return np.column_stack((xs, ys)).astype(np.float64)

    @staticmethod
    def _circle_from_three(points: np.ndarray) -> Optional[Tuple[float, float, float]]:
        (x1, y1), (x2, y2), (x3, y3) = points
        temp = x2 * x2 + y2 * y2
        bc = (x1 * x1 + y1 * y1 - temp) / 2.0
        cd = (temp - x3 * x3 - y3 * y3) / 2.0
        det = (x1 - x2) * (y2 - y3) - (x2 - x3) * (y1 - y2)
        if abs(det) < 1e-6:
            return None
        cx = (bc * (y2 - y3) - cd * (y1 - y2)) / det
        cy = ((x1 - x2) * cd - (x2 - x3) * bc) / det
        radius = math.hypot(cx - x1, cy - y1)
        if not math.isfinite(radius):
            return None
        return cx, cy, radius

    @staticmethod
    def _least_squares_circle(points: np.ndarray) -> Optional[Tuple[float, float, float]]:
        if points.shape[0] < 3:
            return None
        x = points[:, 0]
        y = points[:, 1]
        a = np.column_stack((x, y, np.ones_like(x)))
        b = -(x * x + y * y)
        try:
            coeffs, _, _, _ = np.linalg.lstsq(a, b, rcond=None)
        except np.linalg.LinAlgError:
            return None
        aa, bb, cc = coeffs
        cx = -aa / 2.0
        cy = -bb / 2.0
        radius_sq = cx * cx + cy * cy - cc
        if radius_sq <= 0.0:
            return None
        radius = math.sqrt(radius_sq)
        if not (math.isfinite(cx) and math.isfinite(cy) and math.isfinite(radius)):
            return None
        return float(cx), float(cy), float(radius)

    def _radius_allowed(self, radius: float) -> bool:
        expected = float(self.get_parameter('expected_radius').value)
        tolerance = max(0.0, float(self.get_parameter('radius_tolerance').value))
        return expected - tolerance <= radius <= expected + tolerance

    @staticmethod
    def _score_circle(points: np.ndarray, cx: float, cy: float, radius: float, threshold: float) -> Tuple[np.ndarray, float]:
        residuals = np.abs(np.hypot(points[:, 0] - cx, points[:, 1] - cy) - radius)
        inliers = residuals <= threshold
        if not np.any(inliers):
            return inliers, float('inf')
        return inliers, float(np.mean(residuals[inliers]))

    def _fit_circle(self, points: np.ndarray) -> Optional[CircleFit]:
        min_inliers = int(self.get_parameter('min_inliers').value)
        max_fit_error = float(self.get_parameter('max_fit_error').value)
        threshold = max(0.001, float(self.get_parameter('ransac_threshold').value))
        use_ransac = bool(self.get_parameter('use_ransac').value)

        if points.shape[0] < max(3, min_inliers):
            return None

        best: Optional[Tuple[int, float, float, float, float, np.ndarray]] = None
        if use_ransac:
            iterations = max(1, int(self.get_parameter('ransac_iterations').value))
            for _ in range(iterations):
                sample_indices = self.rng.choice(points.shape[0], size=3, replace=False)
                circle = self._circle_from_three(points[sample_indices])
                if circle is None:
                    continue
                cx, cy, radius = circle
                if not self._radius_allowed(radius):
                    continue
                inliers, error = self._score_circle(points, cx, cy, radius, threshold)
                count = int(np.count_nonzero(inliers))
                if count < min_inliers:
                    continue
                if best is None or count > best[0] or (count == best[0] and error < best[1]):
                    best = (count, error, cx, cy, radius, inliers)
        else:
            circle = self._least_squares_circle(points)
            if circle is not None:
                cx, cy, radius = circle
                inliers, error = self._score_circle(points, cx, cy, radius, threshold)
                best = (int(np.count_nonzero(inliers)), error, cx, cy, radius, inliers)

        if best is None:
            return None

        _, _, cx, cy, radius, inliers = best
        refined = self._least_squares_circle(points[inliers])
        if refined is not None:
            cx, cy, radius = refined
            inliers, error = self._score_circle(points, cx, cy, radius, threshold)
        else:
            inliers, error = self._score_circle(points, cx, cy, radius, threshold)

        inlier_count = int(np.count_nonzero(inliers))
        if inlier_count < min_inliers:
            return None
        if not self._radius_allowed(radius):
            return None
        if error > max_fit_error:
            return None

        return float(cx), float(cy), float(radius), float(error), inlier_count

    def _stabilize_fit(self, fit: CircleFit) -> Optional[CircleFit]:
        if self.last_fit is None:
            return fit

        cx, cy, radius, error, inliers = fit
        last_cx, last_cy, last_radius, last_error, _ = self.last_fit
        center_jump = math.hypot(cx - last_cx, cy - last_cy)
        max_center_jump = max(0.0, float(self.get_parameter('max_center_jump').value))
        if max_center_jump > 0.0 and center_jump > max_center_jump:
            self.get_logger().warn(
                f'Rejecting arena fit jump {center_jump:.3f}m > {max_center_jump:.3f}m',
                throttle_duration_sec=2.0,
            )
            return None

        alpha = float(
            np.clip(float(self.get_parameter('center_smoothing_alpha').value), 0.0, 1.0)
        )
        smooth_cx = alpha * cx + (1.0 - alpha) * last_cx
        smooth_cy = alpha * cy + (1.0 - alpha) * last_cy
        smooth_radius = alpha * radius + (1.0 - alpha) * last_radius
        smooth_error = alpha * error + (1.0 - alpha) * last_error
        return float(smooth_cx), float(smooth_cy), float(smooth_radius), float(smooth_error), int(inliers)

    def _publish_fit(self, scan: LaserScan, fit: CircleFit, valid: bool) -> None:
        cx, cy, radius, error, inliers = fit
        distance = math.hypot(cx, cy)
        angle = math.atan2(cy, cx)
        tolerance = max(0.0, float(self.get_parameter('goal_tolerance').value))

        self.goal_distance_pub.publish(Float32(data=float(distance)))
        self.goal_angle_pub.publish(Float32(data=float(angle)))
        self.goal_valid_pub.publish(Bool(data=bool(valid)))
        self.goal_reached_pub.publish(Bool(data=bool(valid and distance <= tolerance)))
        self.radius_pub.publish(Float32(data=float(radius)))
        self.error_pub.publish(Float32(data=float(error)))
        self.inliers_pub.publish(Int32(data=int(inliers)))

        point = PointStamped()
        point.header.stamp = scan.header.stamp
        point.header.frame_id = scan.header.frame_id
        point.point.x = float(cx)
        point.point.y = float(cy)
        point.point.z = 0.0
        self.center_pub.publish(point)

        self.last_valid = valid

    def _publish_invalid(self) -> None:
        self.goal_distance_pub.publish(Float32(data=float('inf')))
        self.goal_angle_pub.publish(Float32(data=0.0))
        self.goal_valid_pub.publish(Bool(data=False))
        self.goal_reached_pub.publish(Bool(data=False))
        self.radius_pub.publish(Float32(data=0.0))
        self.error_pub.publish(Float32(data=float('inf')))
        self.inliers_pub.publish(Int32(data=0))
        self.last_valid = False

    def _timer_cb(self) -> None:
        scan = self.latest_scan
        if scan is None:
            self.get_logger().warn('Waiting for LaserScan.', throttle_duration_sec=2.0)
            self._publish_invalid()
            return

        points = self._scan_to_points(scan)
        fit = self._fit_circle(points)
        now = time.monotonic()
        if fit is None:
            timeout = max(0.0, float(self.get_parameter('last_valid_timeout').value))
            if (
                self.last_fit is not None
                and self.last_valid_time is not None
                and now - self.last_valid_time <= timeout
            ):
                self._publish_fit(scan, self.last_fit, True)
                return
            self.get_logger().warn(
                f'Arena center estimate invalid. candidates={points.shape[0]}',
                throttle_duration_sec=2.0,
            )
            self._publish_invalid()
            return

        stabilized_fit = self._stabilize_fit(fit)
        if stabilized_fit is None:
            timeout = max(0.0, float(self.get_parameter('last_valid_timeout').value))
            if (
                self.last_fit is not None
                and self.last_valid_time is not None
                and now - self.last_valid_time <= timeout
            ):
                self._publish_fit(scan, self.last_fit, True)
                return
            self._publish_invalid()
            return

        self.last_fit = stabilized_fit
        self.last_valid_time = now
        self._publish_fit(scan, stabilized_fit, True)

    def _log_status(self) -> None:
        if self.last_fit is None:
            self.get_logger().info(f'goal_valid={self.last_valid} no valid arena fit yet')
            return
        cx, cy, radius, error, inliers = self.last_fit
        self.get_logger().info(
            f'goal_valid={self.last_valid} center=({cx:.3f}, {cy:.3f}) '
            f'radius={radius:.3f}m fit_error={error:.3f} inliers={inliers}'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArenaCenterEstimatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
