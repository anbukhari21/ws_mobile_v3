"""Geometry utilities for the clean V2 SAC policy."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence


Point = tuple[float, float]


@dataclass(frozen=True)
class Projection:
    closest_point: Point
    segment_index: int
    segment_t: float
    signed_cross_track_error: float
    abs_cross_track_error: float
    progress: float
    remaining_distance: float
    distance_to_segment_end: float
    path_heading: float


@dataclass(frozen=True)
class PathState:
    robot_x: float
    robot_y: float
    robot_yaw: float
    closest_point: Point
    lookahead_point: Point
    segment_index: int
    signed_cross_track_error: float
    abs_cross_track_error: float
    cross_track_error_delta: float
    progress: float
    progress_delta: float
    progress_fraction: float
    remaining_distance: float
    distance_to_segment_end: float
    lookahead_distance: float
    lookahead_angle: float
    path_heading_error: float
    upcoming_turn_angle: float
    upcoming_turn_direction: float
    total_length: float
    final_distance: float


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def normalize_path(points: Iterable[Sequence[float]]) -> list[Point]:
    """Return a de-duplicated path with at least two points."""
    clean: list[Point] = []
    for point in points:
        if point is None or len(point) < 2:
            continue
        candidate = (float(point[0]), float(point[1]))
        if clean and distance(clean[-1], candidate) < 1e-6:
            continue
        clean.append(candidate)
    if len(clean) < 2:
        raise ValueError("path must contain at least two distinct points")
    return clean


def distance(a: Point, b: Point) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def segment_lengths(path: Sequence[Point]) -> list[float]:
    return [distance(path[i], path[i + 1]) for i in range(len(path) - 1)]


def total_path_length(path: Sequence[Point]) -> float:
    return float(sum(segment_lengths(path)))


def _segment_heading(a: Point, b: Point) -> float:
    return math.atan2(float(b[1]) - float(a[1]), float(b[0]) - float(a[0]))


def _cumulative_lengths(path: Sequence[Point]) -> list[float]:
    lengths = [0.0]
    running = 0.0
    for seg_len in segment_lengths(path):
        running += seg_len
        lengths.append(running)
    return lengths


def project_to_polyline(path: Sequence[Point], x: float, y: float) -> Projection:
    """Project a point onto a polyline and return signed cross-track geometry."""
    path = normalize_path(path)
    px = float(x)
    py = float(y)
    cumulative = _cumulative_lengths(path)
    total = cumulative[-1]
    best: Projection | None = None
    best_dist_sq = float("inf")
    for index, (start, end) in enumerate(zip(path[:-1], path[1:])):
        sx, sy = start
        ex, ey = end
        vx = ex - sx
        vy = ey - sy
        seg_len_sq = vx * vx + vy * vy
        if seg_len_sq <= 1e-12:
            continue
        t = max(0.0, min(1.0, ((px - sx) * vx + (py - sy) * vy) / seg_len_sq))
        cx = sx + t * vx
        cy = sy + t * vy
        dx = px - cx
        dy = py - cy
        dist_sq = dx * dx + dy * dy
        seg_len = math.sqrt(seg_len_sq)
        cross = vx * (py - sy) - vy * (px - sx)
        signed_cte = cross / seg_len
        progress = cumulative[index] + t * seg_len
        remaining = max(0.0, total - progress)
        projection = Projection(
            closest_point=(cx, cy),
            segment_index=index,
            segment_t=t,
            signed_cross_track_error=signed_cte,
            abs_cross_track_error=math.sqrt(dist_sq),
            progress=progress,
            remaining_distance=remaining,
            distance_to_segment_end=max(0.0, (1.0 - t) * seg_len),
            path_heading=math.atan2(vy, vx),
        )
        if dist_sq < best_dist_sq:
            best_dist_sq = dist_sq
            best = projection
    if best is None:
        raise ValueError("path contains no usable segment")
    return best


def point_at_progress(path: Sequence[Point], progress: float) -> Point:
    path = normalize_path(path)
    target = max(0.0, min(float(progress), total_path_length(path)))
    cumulative = _cumulative_lengths(path)
    for index, seg_len in enumerate(segment_lengths(path)):
        start_progress = cumulative[index]
        end_progress = cumulative[index + 1]
        if target <= end_progress or index == len(path) - 2:
            t = 0.0 if seg_len <= 1e-12 else (target - start_progress) / seg_len
            sx, sy = path[index]
            ex, ey = path[index + 1]
            return (sx + t * (ex - sx), sy + t * (ey - sy))
    return path[-1]


def _clamp_point_to_radius(point: Point, radius_limit: float) -> Point:
    x, y = float(point[0]), float(point[1])
    radius = math.hypot(x, y)
    limit = max(0.10, float(radius_limit))
    if radius <= limit or radius <= 1e-9:
        return (x, y)
    scale = limit / radius
    return (x * scale, y * scale)


def _ray_circle_forward_distance(point: Point, heading: float, radius_limit: float) -> float:
    """Return forward ray distance before exiting the allowed circular arena band."""
    x, y = float(point[0]), float(point[1])
    ux, uy = math.cos(float(heading)), math.sin(float(heading))
    radius_sq = max(0.10, float(radius_limit)) ** 2
    c = x * x + y * y - radius_sq
    b = x * ux + y * uy
    disc = max(0.0, b * b - c)
    root = math.sqrt(disc)
    candidates = (-b - root, -b + root)
    positive = [value for value in candidates if value > 1e-6]
    return max(positive, default=0.0)


def _bezier(p0: Point, p1: Point, p2: Point, p3: Point, t: float) -> Point:
    u = 1.0 - float(t)
    b0 = u * u * u
    b1 = 3.0 * u * u * t
    b2 = 3.0 * u * t * t
    b3 = t * t * t
    return (
        b0 * p0[0] + b1 * p1[0] + b2 * p2[0] + b3 * p3[0],
        b0 * p0[1] + b1 * p1[1] + b2 * p2[1] + b3 * p3[1],
    )


def build_heading_aligned_center_path(
    start_x: float,
    start_y: float,
    start_yaw: float,
    *,
    samples: int = 16,
    arena_path_radius_limit: float = 1.85,
    curve_strength: float = 0.28,
    random_selector: float = 0.5,
) -> tuple[list[Point], dict[str, object]]:
    """Build a local curved center path whose first tangent follows robot yaw.

    This is a deterministic local reference path generator, not global planning.
    It does not inspect obstacles, maps, or occupancy grids.
    """
    p0 = (float(start_x), float(start_y))
    p3 = (0.0, 0.0)
    distance_to_center = max(1e-6, math.hypot(p0[0], p0[1]))
    to_center = (-p0[0] / distance_to_center, -p0[1] / distance_to_center)
    center_heading = math.atan2(to_center[1], to_center[0])
    heading_error = wrap_angle(center_heading - float(start_yaw))

    requested_forward = min(0.75, max(0.25, 0.42 * distance_to_center))
    ray_forward = _ray_circle_forward_distance(p0, start_yaw, arena_path_radius_limit)
    forward = min(requested_forward, max(0.0, 0.85 * ray_forward))
    # Near the wall, a robot facing slightly outward can have almost no safe
    # forward ray. Preserve the heading cue minimally, then curve inward.
    if forward < 0.04 and distance_to_center < float(arena_path_radius_limit) - 0.02:
        forward = 0.04
    p1 = (
        p0[0] + math.cos(float(start_yaw)) * forward,
        p0[1] + math.sin(float(start_yaw)) * forward,
    )
    p1 = _clamp_point_to_radius(p1, arena_path_radius_limit)

    side = 1.0 if heading_error >= 0.0 else -1.0
    if abs(heading_error) < math.radians(8.0):
        side = 1.0 if float(random_selector) >= 0.5 else -1.0
    normal = (-to_center[1], to_center[0])
    base_curve = min(0.48, max(0.10, float(curve_strength) * distance_to_center))
    curve = side * base_curve
    final_control = min(0.65, max(0.22, 0.35 * distance_to_center))
    p2 = (
        p3[0] - to_center[0] * final_control + normal[0] * curve,
        p3[1] - to_center[1] * final_control + normal[1] * curve,
    )
    p2 = _clamp_point_to_radius(p2, arena_path_radius_limit)

    sample_count = max(4, int(samples))
    raw_path = [
        _clamp_point_to_radius(
            _bezier(p0, p1, p2, p3, index / (sample_count - 1)),
            arena_path_radius_limit,
        )
        for index in range(sample_count)
    ]
    path = normalize_path(raw_path)
    metadata: dict[str, object] = {
        "style": "heading_curve",
        "samples_requested": int(samples),
        "samples_used": len(path),
        "arena_path_radius_limit": float(arena_path_radius_limit),
        "curve_strength": float(curve_strength),
        "distance_to_center": float(distance_to_center),
        "center_heading_deg": math.degrees(center_heading),
        "start_yaw_deg": math.degrees(float(start_yaw)),
        "heading_error_to_center_deg": math.degrees(heading_error),
        "requested_initial_forward_m": float(requested_forward),
        "used_initial_forward_m": float(forward),
        "curve_side": "left" if side > 0.0 else "right",
        "curve_offset_m": float(curve),
        "control_points": [list(p0), list(p1), list(p2), list(p3)],
        "planner_type": "local_heading_aligned_curve",
        "uses_a_star": False,
        "uses_global_planning": False,
        "uses_occupancy_grid": False,
        "uses_full_replanning": False,
    }
    return path, metadata


def build_center_goal_path(
    start_x: float,
    start_y: float,
    start_yaw: float,
    *,
    style: str = "heading_curve",
    samples: int = 16,
    arena_path_radius_limit: float = 1.85,
    curve_strength: float = 0.28,
    random_selector: float = 0.5,
) -> tuple[list[Point], dict[str, object]]:
    """Build a center-reaching V2 reference path for validation episodes."""
    style_name = str(style).strip().lower()
    if style_name == "straight":
        path = normalize_path([(float(start_x), float(start_y)), (0.0, 0.0)])
        return path, {
            "style": "straight",
            "samples_requested": 2,
            "samples_used": len(path),
            "arena_path_radius_limit": float(arena_path_radius_limit),
            "curve_strength": 0.0,
            "planner_type": "direct_center_line",
            "uses_a_star": False,
            "uses_global_planning": False,
            "uses_occupancy_grid": False,
            "uses_full_replanning": False,
        }
    return build_heading_aligned_center_path(
        start_x,
        start_y,
        start_yaw,
        samples=samples,
        arena_path_radius_limit=arena_path_radius_limit,
        curve_strength=curve_strength,
        random_selector=random_selector,
    )


def upcoming_turn_angle(path: Sequence[Point], segment_index: int) -> float:
    path = normalize_path(path)
    index = int(segment_index)
    if index < 0 or index + 2 >= len(path):
        return 0.0
    current_heading = _segment_heading(path[index], path[index + 1])
    next_heading = _segment_heading(path[index + 1], path[index + 2])
    return wrap_angle(next_heading - current_heading)


def compute_path_state(
    path: Sequence[Point],
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    *,
    lookahead_distance: float,
    previous_progress: float = 0.0,
    previous_cross_track_error: float = 0.0,
) -> PathState:
    path = normalize_path(path)
    projection = project_to_polyline(path, robot_x, robot_y)
    total = max(total_path_length(path), 1e-9)
    lookahead_progress = min(total, projection.progress + max(0.0, float(lookahead_distance)))
    lookahead = point_at_progress(path, lookahead_progress)
    angle_to_lookahead = math.atan2(lookahead[1] - robot_y, lookahead[0] - robot_x)
    lookahead_angle = wrap_angle(angle_to_lookahead - robot_yaw)
    turn = upcoming_turn_angle(path, projection.segment_index)
    final = path[-1]
    return PathState(
        robot_x=float(robot_x),
        robot_y=float(robot_y),
        robot_yaw=wrap_angle(robot_yaw),
        closest_point=projection.closest_point,
        lookahead_point=lookahead,
        segment_index=projection.segment_index,
        signed_cross_track_error=projection.signed_cross_track_error,
        abs_cross_track_error=projection.abs_cross_track_error,
        cross_track_error_delta=projection.abs_cross_track_error
        - float(previous_cross_track_error),
        progress=projection.progress,
        progress_delta=projection.progress - float(previous_progress),
        progress_fraction=max(0.0, min(1.0, projection.progress / total)),
        remaining_distance=projection.remaining_distance,
        distance_to_segment_end=projection.distance_to_segment_end,
        lookahead_distance=distance((robot_x, robot_y), lookahead),
        lookahead_angle=lookahead_angle,
        path_heading_error=wrap_angle(projection.path_heading - robot_yaw),
        upcoming_turn_angle=turn,
        upcoming_turn_direction=0.0 if abs(turn) < 1e-6 else math.copysign(1.0, turn),
        total_length=total,
        final_distance=distance((robot_x, robot_y), final),
    )


def build_fixture_path(shape: str, start_zone: str = "center") -> list[Point]:
    """Return a deterministic no-obstacle path fixture."""
    shape = str(shape).strip().lower()
    zone = str(start_zone).strip().lower()
    offset_y = {"center": 0.0, "middle": 0.18, "near_wall": 0.55}.get(zone, 0.0)
    if shape in {"straight", "straight_short"}:
        return [(-0.75, offset_y), (0.75, offset_y)]
    if shape == "straight_long":
        return [(-1.05, offset_y), (1.05, offset_y)]
    if shape in {"gentle_30", "turn_30"}:
        return [(-0.75, offset_y), (-0.20, offset_y), (0.60, offset_y + 0.32)]
    if shape in {"gentle_45", "turn_45"}:
        return [(-0.75, offset_y), (-0.20, offset_y), (0.50, offset_y + 0.50)]
    if shape in {"gentle_60", "turn_60"}:
        return [(-0.75, offset_y), (-0.20, offset_y), (0.30, offset_y + 0.70)]
    if shape in {"l_shape", "turn_90", "square_corner"}:
        return [(-0.70, offset_y - 0.35), (0.05, offset_y - 0.35), (0.05, offset_y + 0.45)]
    if shape == "square":
        return [
            (-0.55, offset_y - 0.45),
            (0.45, offset_y - 0.45),
            (0.45, offset_y + 0.45),
            (-0.45, offset_y + 0.45),
            (-0.45, offset_y - 0.15),
        ]
    if shape == "triangle":
        return [(-0.65, offset_y - 0.35), (0.45, offset_y - 0.35), (-0.10, offset_y + 0.55), (-0.65, offset_y - 0.35)]
    if shape == "u_shape":
        return [(-0.60, offset_y + 0.45), (-0.60, offset_y - 0.45), (0.55, offset_y - 0.45), (0.55, offset_y + 0.45)]
    if shape == "wide_s":
        return [(-0.80, offset_y - 0.30), (-0.35, offset_y + 0.35), (0.25, offset_y - 0.35), (0.80, offset_y + 0.30)]
    if shape == "zigzag":
        return [(-0.80, offset_y - 0.30), (-0.45, offset_y + 0.30), (-0.10, offset_y - 0.30), (0.25, offset_y + 0.30), (0.70, offset_y - 0.10)]
    raise ValueError(f"unknown path fixture shape: {shape}")


def start_pose_for_path(path: Sequence[Point]) -> tuple[float, float, float]:
    path = normalize_path(path)
    heading = _segment_heading(path[0], path[1])
    return (path[0][0], path[0][1], heading)
