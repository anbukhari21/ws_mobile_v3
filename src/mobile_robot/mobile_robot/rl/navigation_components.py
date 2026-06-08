"""Small navigation components for SAC-based robot control.

These classes keep the learned SAC policy responsible for navigation behavior.
The safety layer only clips physically unsafe commands and records why it
intervened; it never chooses a left/right obstacle-avoidance strategy.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import copy
import math
import os
import time
from typing import Any, Callable, Dict, Iterable, Optional, Tuple

import numpy as np

try:
    import yaml
except Exception:  # pragma: no cover - PyYAML is expected in ROS installs.
    yaml = None


DEFAULT_NAV_CONFIG: Dict[str, Any] = {
    "observation": {
        "enhanced": False,
        "sector_count": 8,
        "include_final_goal": False,
        "include_path_features": False,
    },
    "waypoint_manager": {
        "enabled": False,
        "waypoint_radius": 0.30,
        "final_goal_radius": 0.22,
        "lookahead_distance": 0.65,
        "prune_collected_waypoints": True,
        "passed_waypoint_margin": 0.06,
        "final_goal_prune_radius": 0.40,
        "sparse_path_max_points": 5,
    },
    "waypoint_planner": {
        "risk_profile": "stable",
        "min_gap_width": 0.50,
        "gap_max_lateral_offset": 0.55,
        "gap_entry_offset": 0.24,
        "gap_path_length_slack": 0.45,
        "gap_waypoint_margin": 0.10,
        "gap_segment_margin": 0.08,
        "direct_path_block_margin": 0.28,
        "tight_direct_path_block_margin": 0.23,
        "gap_direct_path_block_margin": 0.22,
    },
    "adaptive_waypoint_replanner": {
        "enabled": True,
        "after_steps": 70,
        "cooldown_steps": 80,
        "window_steps": 70,
        "min_goal_progress": 0.035,
        "min_motion_since_replan": 0.08,
        "stale_waypoint_steps": 130,
        "direct_block_margin": 0.16,
        "waypoint_block_margin": 0.12,
        "replan_on_guard_stop": True,
        "alternate_side_on_stale": True,
        "replacement_min_clearance": 0.10,
        "replacement_clearance_hysteresis": 0.04,
        "soft_replan_min_path_improvement": 0.12,
        "soft_replan_max_extra_length": 0.18,
    },
    "waypoint_smoothing": {
        "enabled": True,
        "method": "chaikin",
        "arc_points": 4,
        "chaikin_iterations": 1,
        "corner_cut": 0.22,
        "min_point_spacing": 0.14,
        "extra_clearance_margin": 0.00,
    },
    "safety_filter": {
        "enabled": True,
        "lidar_min_range": 0.05,
        "critical_clearance": 0.11,
        "caution_clearance": 0.30,
        "prediction_horizon": 0.65,
        "safety_margin": 0.02,
        "blind_zone_stop_distance": 0.07,
        "front_sector_half_angle": 0.52,
        "motion_cone_half_angle": 0.95,
        "max_linear_accel": 0.60,
        "max_angular_accel": 3.00,
        "boundary_safety_margin": 0.35,
        "allow_rotation_when_forward_blocked": True,
    },
    "footprint": {
        "collision_half_length": 0.115,
        "collision_half_width": 0.075,
        "collision_margin": 0.015,
    },
    "gap_corridor_safety": {
        "enabled": False,
        "robot_collision_width": 0.198,
        "robot_collision_half_width": 0.099,
        "sim_side_margin": 0.04,
        "min_gap_width": 0.28,
        "front_center_half_angle": 0.18,
        "side_sector_half_angle": 0.25,
        "heading_alignment_max": 0.35,
        "max_gap_entry_speed": 0.08,
    },
    "reward": {
        "smooth_obstacle_penalty_weight": 0.08,
        "front_obstacle_penalty_weight": 0.45,
        "blind_zone_penalty_weight": 2.50,
        "wall_proximity_penalty_weight": 0.55,
        "angular_velocity_penalty_weight": 0.0005,
        "action_smoothness_penalty_weight": 0.015,
        "no_progress_penalty_weight": 0.004,
        "safety_filter_intervention_penalty": 0.010,
        "safe_progress_clearance_bonus": 0.0005,
    },
    "stuck_detector": {
        "window_steps": 180,
        "min_progress": 0.02,
        "min_displacement": 0.04,
        "low_speed_threshold": 0.015,
        "oscillation_zero_crossings": 8,
        "intervention_ratio": 0.35,
        "collision_risk_ratio": 0.30,
    },
    "recovery_supervisor": {
        "enabled": True,
        "trigger_steps": 55,
        "no_safe_motion_trigger_steps": 8,
        "reverse_seconds": 3.80,
        "escape_arc_seconds": 2.40,
        "hold_seconds": 12.00,
        "reverse_speed": 0.080,
        "reverse_turn_speed": 0.22,
        "escape_forward_speed": 0.085,
        "pivot_angular_velocity": 0.38,
        "rear_clearance": 0.36,
        "escape_front_clearance": 0.38,
        "waypoint_distance": 0.92,
        "waypoint_clearance_margin": 0.34,
        "segment_clearance_margin": 0.08,
        "max_events_per_episode": 3,
    },
    "human_supervisor": {
        "enabled": True,
        "safety_radius": 0.45,
        "path_corridor_width": 0.35,
        "wait_seconds": 3.0,
    },
    "navigation_supervisor": {
        "enabled": True,
        "pre_align_enabled": False,
        "pre_align_run_at_episode_start": True,
        "pre_align_run_after_replan": True,
        "pre_align_run_after_waypoint_switch": False,
        "pre_align_waypoint_switch_min_angle": 0.08,
        "pre_align_waypoint_switch_cooldown_steps": 10,
        "pre_align_heading_threshold": 0.05,
        "pre_align_start_heading_threshold": 0.10,
        "pre_align_timeout_seconds": 6.00,
        "pre_align_timeout_step_cap": 360,
        "pre_align_angular_velocity": 0.95,
        "pre_align_min_angular_velocity": 0.025,
        "pre_align_angular_gain": 1.00,
        "pre_align_release_ramp_steps": 35,
        "pre_align_release_linear_cap": 0.08,
        "pre_align_release_angular_cap": 0.35,
        "pre_align_min_clearance": 0.26,
        "pre_align_front_clearance": 0.30,
        "pre_align_rotation_clearance_margin": 0.12,
        "pre_align_use_local_waypoint": True,
    },
    "metrics": {
        "log_safety_filter_reasons": True,
    },
}

GUARD_RELAXATION_PROFILE_CHOICES = (
    "stable",
    "relaxed_1",
    "relaxed_2",
    "relaxed_3",
    "tight_gap",
)

GUARD_RELAXATION_PROFILES: Dict[str, Dict[str, Dict[str, float]]] = {
    "stable": {
        "safety_filter": {},
        "action_guard": {},
    },
    # Small reductions only. These profiles keep emergency protection active,
    # but reduce ordinary obstacle-side conservatism gradually.
    "relaxed_1": {
        "safety_filter": {
            "critical_clearance": 0.105,
            "caution_clearance": 0.28,
            "prediction_horizon": 0.60,
            "front_sector_half_angle": 0.48,
            "motion_cone_half_angle": 0.90,
        },
        "action_guard": {
            "front_clearance": 0.38,
            "min_turn_velocity": 0.18,
            "turn_to_goal_angle": 0.85,
            "obstacle_stop_distance": 0.28,
        },
    },
    "relaxed_2": {
        "safety_filter": {
            "critical_clearance": 0.100,
            "caution_clearance": 0.26,
            "prediction_horizon": 0.55,
            "front_sector_half_angle": 0.45,
            "motion_cone_half_angle": 0.86,
        },
        "action_guard": {
            "front_clearance": 0.36,
            "min_turn_velocity": 0.16,
            "turn_to_goal_angle": 0.95,
            "obstacle_stop_distance": 0.26,
        },
    },
    "relaxed_3": {
        "safety_filter": {
            "critical_clearance": 0.095,
            "caution_clearance": 0.24,
            "prediction_horizon": 0.50,
            "front_sector_half_angle": 0.42,
            "motion_cone_half_angle": 0.82,
        },
        "action_guard": {
            "front_clearance": 0.34,
            "min_turn_velocity": 0.14,
            "turn_to_goal_angle": 1.05,
            "obstacle_stop_distance": 0.24,
        },
    },
    # Minimum safety-only profile for learning tight-gap navigation from
    # scratch. This keeps blind-zone and imminent-front-collision protection,
    # but avoids treating ordinary side clearance in a 0.35 m gap as a stop.
    "tight_gap": {
        "safety_filter": {
            "safety_margin": 0.005,
            "blind_zone_stop_distance": 0.052,
            "critical_clearance": 0.060,
            "caution_clearance": 0.150,
            "prediction_horizon": 0.25,
            "front_sector_half_angle": 0.26,
            "motion_cone_half_angle": 0.52,
            "boundary_safety_margin": 0.30,
        },
        "action_guard": {
            "front_clearance": 0.18,
            "min_turn_velocity": 0.08,
            "turn_to_goal_angle": 1.10,
            "obstacle_stop_distance": 0.075,
        },
    },
}


def normalize_guard_relaxation_profile(profile: str | None) -> str:
    value = str(profile or "stable").strip().lower().replace("-", "_")
    aliases = {
        "none": "stable",
        "off": "stable",
        "default": "stable",
        "baseline": "stable",
        "mild": "relaxed_1",
        "medium": "relaxed_2",
        "low": "relaxed_3",
        "minimum": "tight_gap",
        "minimal": "tight_gap",
        "tight": "tight_gap",
        "tightgap": "tight_gap",
    }
    value = aliases.get(value, value)
    if value not in GUARD_RELAXATION_PROFILES:
        choices = ", ".join(GUARD_RELAXATION_PROFILE_CHOICES)
        raise ValueError(f"unknown guard relaxation profile '{profile}'. Choose one of: {choices}")
    return value


def apply_guard_relaxation_to_safety_config(
    safety_config: "SafetyFilterConfig",
    profile: str | None,
) -> str:
    normalized = normalize_guard_relaxation_profile(profile)
    overrides = GUARD_RELAXATION_PROFILES[normalized]["safety_filter"]
    for name, value in overrides.items():
        if hasattr(safety_config, name):
            setattr(safety_config, name, float(value))
    return normalized


def guard_relaxation_action_guard_overrides(profile: str | None) -> Dict[str, float]:
    normalized = normalize_guard_relaxation_profile(profile)
    return dict(GUARD_RELAXATION_PROFILES[normalized]["action_guard"])


def _deep_update(base: Dict[str, Any], updates: Dict[str, Any]) -> Dict[str, Any]:
    merged = copy.deepcopy(base)
    for key, value in updates.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_update(merged[key], value)
        else:
            merged[key] = value
    return merged


def default_navigation_config_path() -> Optional[Path]:
    candidates = []
    env_path = os.environ.get("MOBILE_ROBOT_NAV_CONFIG")
    if env_path:
        candidates.append(Path(env_path).expanduser())

    cwd = Path.cwd()
    candidates.append(cwd / "src/mobile_robot/config/sac_navigation_refactor.yaml")
    candidates.append(cwd / "install/mobile_robot/share/mobile_robot/config/sac_navigation_refactor.yaml")

    here = Path(__file__).resolve()
    candidates.append(here.parents[2] / "config/sac_navigation_refactor.yaml")

    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def load_navigation_config(path: Optional[str] = None) -> Dict[str, Any]:
    """Load the navigation refactor config, falling back to safe defaults."""

    config = copy.deepcopy(DEFAULT_NAV_CONFIG)
    resolved: Optional[Path]
    if path is None or str(path).strip().lower() in {"", "auto", "default"}:
        resolved = default_navigation_config_path()
    else:
        resolved = Path(path).expanduser()

    if resolved is None:
        config["_config_path"] = ""
        return config

    if not resolved.exists():
        raise FileNotFoundError(f"navigation config does not exist: {resolved}")

    if yaml is None:
        raise RuntimeError(
            "PyYAML is required to load navigation config files, but it is not available"
        )

    with resolved.open("r", encoding="utf-8") as stream:
        loaded = yaml.safe_load(stream) or {}
    if not isinstance(loaded, dict):
        raise ValueError(f"navigation config must be a mapping: {resolved}")

    config = _deep_update(config, loaded)
    config["_config_path"] = str(resolved)
    return config


def _get_section(config: Dict[str, Any], name: str) -> Dict[str, Any]:
    section = config.get(name, {})
    return section if isinstance(section, dict) else {}


def clean_lidar(lidar: Iterable[float], max_lidar_range: float) -> np.ndarray:
    arr = np.asarray(list(lidar), dtype=np.float32)
    if arr.size == 0:
        return np.asarray([], dtype=np.float32)
    arr = np.nan_to_num(arr, nan=max_lidar_range, posinf=max_lidar_range, neginf=0.0)
    return np.clip(arr, 0.0, max_lidar_range).astype(np.float32)


def lidar_angles(count: int) -> np.ndarray:
    if count <= 1:
        return np.asarray([0.0], dtype=np.float32)
    return np.linspace(-math.pi, math.pi, count, dtype=np.float32)


def angle_diff(a: np.ndarray | float, b: float) -> np.ndarray | float:
    return np.arctan2(np.sin(np.asarray(a) - b), np.cos(np.asarray(a) - b))


def sector_min(
    lidar: np.ndarray,
    angles: np.ndarray,
    target_angle: float,
    half_angle: float,
    fallback: float,
) -> float:
    if lidar.size == 0 or angles.size != lidar.size:
        return float(fallback)
    mask = np.abs(angle_diff(angles, target_angle)) <= half_angle
    if not np.any(mask):
        return float(fallback)
    values = lidar[mask]
    if values.size == 0:
        return float(fallback)
    return float(np.min(values))


def sector_summary(lidar: np.ndarray, angles: np.ndarray, max_lidar_range: float) -> Dict[str, float]:
    """Return sector clearances without turning them into control decisions."""

    sectors = {
        "front": 0.0,
        "front_left": math.pi / 4.0,
        "left": math.pi / 2.0,
        "rear_left": 3.0 * math.pi / 4.0,
        "rear": math.pi,
        "rear_right": -3.0 * math.pi / 4.0,
        "right": -math.pi / 2.0,
        "front_right": -math.pi / 4.0,
    }
    return {
        name: sector_min(lidar, angles, target, math.pi / 8.0, max_lidar_range)
        for name, target in sectors.items()
    }


@dataclass
class WaypointManagerConfig:
    enabled: bool = False
    waypoint_radius: float = 0.30
    final_goal_radius: float = 0.22
    lookahead_distance: float = 0.65
    prune_collected_waypoints: bool = True
    passed_waypoint_margin: float = 0.06
    final_goal_prune_radius: float = 0.40
    sparse_path_max_points: int = 5
    sparse_waypoint_switch_radius: float = 0.24
    sparse_pass_projection: float = 0.93
    sparse_pass_cross_track: float = 0.24

    @classmethod
    def from_config(cls, config: Dict[str, Any]) -> "WaypointManagerConfig":
        section = _get_section(config, "waypoint_manager")
        defaults = cls()
        return cls(
            enabled=bool(section.get("enabled", defaults.enabled)),
            waypoint_radius=max(
                0.01,
                float(section.get("waypoint_radius", defaults.waypoint_radius)),
            ),
            final_goal_radius=max(
                0.01,
                float(section.get("final_goal_radius", defaults.final_goal_radius)),
            ),
            lookahead_distance=max(
                0.0,
                float(section.get("lookahead_distance", defaults.lookahead_distance)),
            ),
            prune_collected_waypoints=bool(
                section.get(
                    "prune_collected_waypoints",
                    defaults.prune_collected_waypoints,
                )
            ),
            passed_waypoint_margin=max(
                0.0,
                float(
                    section.get(
                        "passed_waypoint_margin",
                        defaults.passed_waypoint_margin,
                    )
                ),
            ),
            final_goal_prune_radius=max(
                0.0,
                float(
                    section.get(
                        "final_goal_prune_radius",
                        defaults.final_goal_prune_radius,
                    )
                ),
            ),
            sparse_path_max_points=max(
                2,
                int(section.get("sparse_path_max_points", defaults.sparse_path_max_points)),
            ),
            sparse_waypoint_switch_radius=max(
                0.01,
                float(
                    section.get(
                        "sparse_waypoint_switch_radius",
                        defaults.sparse_waypoint_switch_radius,
                    )
                ),
            ),
            sparse_pass_projection=max(
                0.0,
                float(
                    section.get(
                        "sparse_pass_projection",
                        defaults.sparse_pass_projection,
                    )
                ),
            ),
            sparse_pass_cross_track=max(
                0.0,
                float(
                    section.get(
                        "sparse_pass_cross_track",
                        defaults.sparse_pass_cross_track,
                    )
                ),
            ),
        )


@dataclass
class WaypointState:
    enabled: bool
    x: float
    y: float
    distance: float
    angle: float
    final_distance: float
    final_angle: float
    index: int = 0
    base_index: int = 0
    count: int = 1
    switches: int = 0
    final_goal_reached: bool = False
    reason: str = "final_goal"


class WaypointManager:
    """Selects local target waypoints without producing motion commands."""

    def __init__(self, config: WaypointManagerConfig) -> None:
        self.config = config
        self.path: list[Tuple[float, float]] = []
        self.base_index = 0
        self.current_index = 0
        self.switches = 0

    @staticmethod
    def _distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))

    @staticmethod
    def _target_angle(
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        target_x: float,
        target_y: float,
    ) -> float:
        heading = math.atan2(float(target_y) - float(robot_y), float(target_x) - float(robot_x))
        error = heading - float(robot_yaw)
        return math.atan2(math.sin(error), math.cos(error))

    @staticmethod
    def _clean_path(
        waypoints: Iterable[Tuple[float, float]],
        final_goal: Tuple[float, float],
    ) -> list[Tuple[float, float]]:
        cleaned: list[Tuple[float, float]] = []
        for waypoint in waypoints:
            if waypoint is None:
                continue
            x, y = float(waypoint[0]), float(waypoint[1])
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            if cleaned and math.hypot(cleaned[-1][0] - x, cleaned[-1][1] - y) < 1e-4:
                continue
            cleaned.append((x, y))

        final = (float(final_goal[0]), float(final_goal[1]))
        if not cleaned or math.hypot(cleaned[-1][0] - final[0], cleaned[-1][1] - final[1]) > 1e-4:
            cleaned.append(final)
        return cleaned

    def reset(
        self,
        *,
        final_goal: Tuple[float, float],
        waypoints: Optional[Iterable[Tuple[float, float]]] = None,
    ) -> None:
        self.path = self._clean_path(waypoints or [final_goal], final_goal)
        self.base_index = 0
        self.current_index = 0
        self.switches = 0

    def set_path(
        self,
        waypoints: Iterable[Tuple[float, float]],
        *,
        final_goal: Tuple[float, float],
    ) -> None:
        previous_switches = int(self.switches)
        self.reset(final_goal=final_goal, waypoints=waypoints)
        self.switches = previous_switches

    def _select_lookahead_index(self, robot_x: float, robot_y: float) -> int:
        if not self.path:
            return 0
        if self.config.lookahead_distance <= 1e-6:
            return self.base_index

        robot_pos = (float(robot_x), float(robot_y))
        # Sparse detour paths should be followed one local target at a time, not
        # skipped by lookahead. Otherwise the current-target marker jumps ahead
        # and the SAC policy re-aims mid-arc near obstacle corners. Dense
        # planner paths still use lookahead to avoid point chasing.
        if len(self.path) <= self.config.sparse_path_max_points:
            return self.base_index

        selected = self.base_index
        while selected < len(self.path) - 1:
            if self._distance(robot_pos, self.path[selected]) >= self.config.lookahead_distance:
                break
            selected += 1
        return min(selected, len(self.path) - 1)

    def _advance_base_index(self, robot_pos: Tuple[float, float], final_distance: float) -> None:
        """Retire collected or already-passed waypoints.

        A local waypoint is only a training target. If the robot has passed it
        while following a smooth arc, keeping that old target active can pull the
        policy backward. This pruning changes only the target observation; it
        never commands motion.
        """

        if not self.config.prune_collected_waypoints:
            while self.base_index < len(self.path) - 1:
                if self._distance(robot_pos, self.path[self.base_index]) > self.config.waypoint_radius:
                    break
                self.base_index += 1
                self.switches += 1
            return

        final_prune_radius = max(
            self.config.final_goal_prune_radius,
            self.config.final_goal_radius + 0.08,
            self.config.waypoint_radius + 0.10,
        )
        if final_distance <= final_prune_radius and self.base_index < len(self.path) - 1:
            self.switches += len(self.path) - 1 - self.base_index
            self.base_index = len(self.path) - 1
            return

        pass_margin = float(self.config.passed_waypoint_margin)
        while self.base_index < len(self.path) - 1:
            current = self.path[self.base_index]
            current_distance = self._distance(robot_pos, current)
            sparse_path = len(self.path) <= self.config.sparse_path_max_points
            nxt = self.path[self.base_index + 1]
            next_spacing = self._distance(current, nxt)
            advance_radius = (
                min(self.config.waypoint_radius, self.config.sparse_waypoint_switch_radius)
                if sparse_path
                else min(
                    self.config.waypoint_radius,
                    max(0.055, 0.45 * next_spacing),
                )
            )
            if current_distance <= advance_radius:
                self.base_index += 1
                self.switches += 1
                continue

            vx = float(nxt[0]) - float(current[0])
            vy = float(nxt[1]) - float(current[1])
            seg_len_sq = vx * vx + vy * vy
            if seg_len_sq <= 1e-8:
                self.base_index += 1
                self.switches += 1
                continue

            wx = float(robot_pos[0]) - float(current[0])
            wy = float(robot_pos[1]) - float(current[1])
            projection = (wx * vx + wy * vy) / seg_len_sq
            cross_track = abs(wx * vy - wy * vx) / math.sqrt(seg_len_sq)
            next_distance = self._distance(robot_pos, nxt)
            passed_projection = self.config.sparse_pass_projection if sparse_path else 0.30
            behind_projection = (self.config.sparse_pass_projection + 0.12) if sparse_path else 0.85
            cross_track_ok = (
                True
                if not sparse_path
                else cross_track <= max(self.config.sparse_pass_cross_track, advance_radius)
            )
            passed_waypoint = (
                projection >= passed_projection
                and next_distance + pass_margin < current_distance
                and cross_track_ok
            )
            clearly_behind = (
                projection >= behind_projection
                and current_distance > advance_radius * 0.75
                and cross_track_ok
            )
            if not (passed_waypoint or clearly_behind):
                break
            self.base_index += 1
            self.switches += 1

    def update(
        self,
        *,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        final_goal: Tuple[float, float],
    ) -> WaypointState:
        final_goal = (float(final_goal[0]), float(final_goal[1]))
        if not self.path:
            self.reset(final_goal=final_goal)
        elif self._distance(self.path[-1], final_goal) > 1e-4:
            self.path = self._clean_path(self.path, final_goal)

        final_distance = self._distance((robot_x, robot_y), final_goal)
        final_angle = self._target_angle(robot_x, robot_y, robot_yaw, final_goal[0], final_goal[1])
        final_reached = final_distance <= self.config.final_goal_radius

        if (not self.config.enabled) or final_reached:
            target_x, target_y = final_goal
            distance = final_distance
            angle = final_angle
            reason = "final_goal_reached" if final_reached else "final_goal"
            return WaypointState(
                enabled=bool(self.config.enabled),
                x=target_x,
                y=target_y,
                distance=distance,
                angle=angle,
                final_distance=final_distance,
                final_angle=final_angle,
                index=len(self.path) - 1,
                base_index=self.base_index,
                count=len(self.path),
                switches=self.switches,
                final_goal_reached=final_reached,
                reason=reason,
            )

        robot_pos = (float(robot_x), float(robot_y))
        self._advance_base_index(robot_pos, final_distance)

        selected_index = self._select_lookahead_index(robot_x, robot_y)
        self.current_index = selected_index
        target_x, target_y = self.path[selected_index]
        waypoint_distance = self._distance(robot_pos, (target_x, target_y))
        waypoint_angle = self._target_angle(robot_x, robot_y, robot_yaw, target_x, target_y)
        reason = "lookahead_waypoint" if selected_index < len(self.path) - 1 else "final_goal"

        return WaypointState(
            enabled=True,
            x=target_x,
            y=target_y,
            distance=waypoint_distance,
            angle=waypoint_angle,
            final_distance=final_distance,
            final_angle=final_angle,
            index=selected_index,
            base_index=self.base_index,
            count=len(self.path),
            switches=self.switches,
            final_goal_reached=False,
            reason=reason,
        )


@dataclass
class SafetyFilterConfig:
    enabled: bool = True
    lidar_min_range: float = 0.05
    critical_clearance: float = 0.11
    caution_clearance: float = 0.30
    prediction_horizon: float = 0.65
    safety_margin: float = 0.02
    blind_zone_stop_distance: float = 0.07
    front_sector_half_angle: float = 0.52
    motion_cone_half_angle: float = 0.95
    max_linear_accel: float = 0.60
    max_angular_accel: float = 3.00
    boundary_safety_margin: float = 0.35
    allow_rotation_when_forward_blocked: bool = True
    gap_corridor_safety_enabled: bool = False
    gap_robot_collision_width: float = 0.198
    gap_robot_collision_half_width: float = 0.099
    gap_sim_side_margin: float = 0.04
    gap_min_gap_width: float = 0.28
    gap_front_center_half_angle: float = 0.18
    gap_side_sector_half_angle: float = 0.25
    gap_heading_alignment_max: float = 0.35
    gap_max_gap_entry_speed: float = 0.08

    @classmethod
    def from_config(cls, config: Dict[str, Any]) -> "SafetyFilterConfig":
        section = _get_section(config, "safety_filter")
        defaults = cls()
        values = {
            field_name: section.get(field_name, getattr(defaults, field_name))
            for field_name in cls.__dataclass_fields__
        }
        gap_section = _get_section(config, "gap_corridor_safety")
        gap_aliases = {
            "enabled": "gap_corridor_safety_enabled",
            "robot_collision_width": "gap_robot_collision_width",
            "robot_collision_half_width": "gap_robot_collision_half_width",
            "sim_side_margin": "gap_sim_side_margin",
            "min_gap_width": "gap_min_gap_width",
            "front_center_half_angle": "gap_front_center_half_angle",
            "side_sector_half_angle": "gap_side_sector_half_angle",
            "heading_alignment_max": "gap_heading_alignment_max",
            "max_gap_entry_speed": "gap_max_gap_entry_speed",
        }
        for source_name, target_name in gap_aliases.items():
            if source_name in gap_section and target_name in values:
                values[target_name] = gap_section[source_name]
        return cls(**values)


@dataclass
class SafetyFilterResult:
    linear: float
    angular: float
    intervened: bool
    reason: str = ""
    raw_linear: float = 0.0
    raw_angular: float = 0.0


class SafetyFilter:
    """Minimal safety-only filter for SAC actions."""

    def __init__(self, config: SafetyFilterConfig) -> None:
        self.config = config

    def filter_action(
        self,
        linear_cmd: float,
        angular_cmd: float,
        *,
        lidar: np.ndarray,
        angles: np.ndarray,
        front_clearance: float,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        arena_radius: float,
        max_linear_velocity: float,
        max_reverse_velocity: float,
        max_angular_velocity: float,
        previous_linear_cmd: float,
        previous_angular_cmd: float,
        dt: float,
        human_stop: bool = False,
        zone_sweep_fn: Optional[Callable[[float, float, float, int], Dict[str, Any]]] = None,
        gap_safety_context: Optional[Dict[str, Any]] = None,
    ) -> SafetyFilterResult:
        raw_linear = float(linear_cmd)
        raw_angular = float(angular_cmd)

        linear = float(np.clip(raw_linear, -max_reverse_velocity, max_linear_velocity))
        angular = float(np.clip(raw_angular, -max_angular_velocity, max_angular_velocity))
        reasons: list[str] = []

        if not self.config.enabled:
            return SafetyFilterResult(linear, angular, False, "", raw_linear, raw_angular)

        dt = max(1e-3, float(dt))
        linear_delta = self.config.max_linear_accel * dt
        angular_delta = self.config.max_angular_accel * dt
        linear_accel_limited = float(
            np.clip(
                linear,
                previous_linear_cmd - linear_delta,
                previous_linear_cmd + linear_delta,
            )
        )
        angular_accel_limited = float(
            np.clip(
                angular,
                previous_angular_cmd - angular_delta,
                previous_angular_cmd + angular_delta,
            )
        )
        if abs(linear_accel_limited - linear) > 1e-6:
            reasons.append("accel_linear")
        if abs(angular_accel_limited - angular) > 1e-6:
            reasons.append("accel_angular")
        linear = linear_accel_limited
        angular = angular_accel_limited

        if human_stop:
            return SafetyFilterResult(0.0, 0.0, True, "human_stop", raw_linear, raw_angular)

        if lidar.size:
            min_index = int(np.argmin(lidar))
            min_clearance = float(lidar[min_index])
            min_angle = float(angles[min_index]) if angles.size == lidar.size else 0.0
        else:
            min_clearance = float("inf")
            min_angle = 0.0

        critical = max(
            self.config.lidar_min_range + self.config.safety_margin,
            self.config.critical_clearance,
        )
        horizon = max(1e-3, self.config.prediction_horizon)
        gap_context = gap_safety_context or {}
        gap_corridor_exempt_side_min = self._gap_corridor_exempts_side_minimum(
            linear=linear,
            min_clearance=min_clearance,
            min_angle=min_angle,
            front_clearance=front_clearance,
            critical=critical,
            context=gap_context,
        )

        if (
            min_clearance <= self.config.lidar_min_range + self.config.safety_margin
            and not gap_corridor_exempt_side_min
        ):
            return SafetyFilterResult(0.0, 0.0, True, "blind_zone_emergency", raw_linear, raw_angular)

        if (
            self.config.gap_corridor_safety_enabled
            and bool(gap_context.get("gap_passable", False))
            and linear > self.config.gap_max_gap_entry_speed
        ):
            linear = max(0.0, self.config.gap_max_gap_entry_speed)
            reasons.append("gap_entry_speed_cap")

        if linear > 0.0:
            front_clearance = min(
                float(front_clearance),
                sector_min(
                    lidar,
                    angles,
                    0.0,
                    self.config.front_sector_half_angle,
                    float(front_clearance),
                ),
            )
            if front_clearance <= critical:
                linear = 0.0
                reasons.append("front_critical")
            else:
                safe_linear_cap = max(0.0, (front_clearance - critical) / horizon)
                if linear > safe_linear_cap:
                    linear = min(linear, safe_linear_cap)
                    reasons.append("front_prediction")

        obstacle_direction_pressure = math.cos(min_angle)
        command_moves_toward_min = (
            abs(min_angle) <= self.config.motion_cone_half_angle
            and linear * obstacle_direction_pressure > 0.0
        )
        if (
            command_moves_toward_min
            and min_clearance <= self.config.blind_zone_stop_distance
            and not gap_corridor_exempt_side_min
        ):
            linear = 0.0
            reasons.append("blind_zone_worsening")

        radius = math.hypot(robot_x, robot_y)
        boundary_limit = max(0.10, arena_radius - self.config.boundary_safety_margin)
        if radius > 1e-6:
            outward_component = (
                math.cos(robot_yaw) * robot_x / radius
                + math.sin(robot_yaw) * robot_y / radius
            )
        else:
            outward_component = 0.0
        projected_radius = math.hypot(
            robot_x + linear * math.cos(robot_yaw) * horizon,
            robot_y + linear * math.sin(robot_yaw) * horizon,
        )
        if linear * outward_component > 0.0 and radius >= boundary_limit:
            linear = 0.0
            reasons.append("boundary_stop")
        elif linear * outward_component > 0.0 and projected_radius >= boundary_limit:
            remaining = max(0.0, boundary_limit - radius)
            cap = remaining / max(horizon * abs(outward_component), 1e-6)
            if abs(linear) > cap:
                linear = math.copysign(cap, linear)
                reasons.append("boundary_prediction")

        if zone_sweep_fn is not None and (abs(linear) > 1e-4 or abs(angular) > 1e-3):
            projected = zone_sweep_fn(linear, angular, horizon, 5)
            projected_distance = float(projected.get("distance", float("inf")))
            if bool(projected.get("contact", False)) and projected_distance <= critical:
                if abs(linear) > 1e-4:
                    linear = 0.0
                    reasons.append("footprint_prediction")
                    projected = zone_sweep_fn(0.0, angular, horizon, 5)
                if bool(projected.get("contact", False)):
                    angular = 0.0
                    reasons.append("no_safe_motion")

        if not self.config.allow_rotation_when_forward_blocked and linear == 0.0:
            angular = 0.0

        linear = float(np.clip(linear, -max_reverse_velocity, max_linear_velocity))
        angular = float(np.clip(angular, -max_angular_velocity, max_angular_velocity))
        intervened = bool(
            reasons
            or abs(linear - raw_linear) > 1e-6
            or abs(angular - raw_angular) > 1e-6
        )
        reason = "+".join(dict.fromkeys(reasons))
        return SafetyFilterResult(linear, angular, intervened, reason, raw_linear, raw_angular)

    def _gap_corridor_exempts_side_minimum(
        self,
        *,
        linear: float,
        min_clearance: float,
        min_angle: float,
        front_clearance: float,
        critical: float,
        context: Dict[str, Any],
    ) -> bool:
        if not self.config.gap_corridor_safety_enabled:
            return False
        if linear <= 0.0:
            return False
        if abs(float(min_angle)) <= self.config.gap_front_center_half_angle:
            return False
        try:
            front_center_min = float(context.get("front_center_min", front_clearance))
            gap_width = float(context.get("gap_width_estimate", 0.0))
            side_clearance = float(context.get("gap_side_clearance_min", -1.0))
            heading_error = abs(float(context.get("corridor_heading_error", 0.0)))
            gap_passable = bool(context.get("gap_passable", False))
        except (TypeError, ValueError):
            return False

        if not gap_passable:
            return False
        if gap_width < self.config.gap_min_gap_width:
            return False
        if side_clearance < self.config.gap_sim_side_margin:
            return False
        if front_center_min <= critical + 0.04:
            return False
        if heading_error > self.config.gap_heading_alignment_max:
            return False
        if min_clearance <= max(0.0, self.config.lidar_min_range - 1e-6):
            return False
        return True


@dataclass
class ObservationBuilder:
    num_lidar_beams: int
    max_lidar_range: float
    max_linear_velocity: float
    max_angular_velocity: float
    enhanced: bool = False
    sector_count: int = 8
    include_final_goal: bool = False
    include_path_features: bool = False
    heading_redesign: bool = False

    HEADING_REDESIGN_FEATURE_NAMES = (
        "target_angle",
        "sin_target_angle",
        "cos_target_angle",
        "abs_target_angle",
        "target_angle_sign",
        "previous_target_angle",
        "target_angle_delta",
        "abs_target_angle_delta",
        "target_angle_scaled_0p10",
        "target_angle_scaled_0p03",
        "distance_to_goal",
        "distance_weighted_target_angle",
        "previous_raw_angular_action",
        "previous_executed_angular_cmd",
        "previous_linear_cmd",
        "sign_crossing_flag",
        "wrong_direction_last_step_flag",
        "target_angle_reducing_last_step_flag",
    )

    @classmethod
    def from_config(
        cls,
        config: Dict[str, Any],
        num_lidar_beams: int,
        max_lidar_range: float,
        max_linear_velocity: float,
        max_angular_velocity: float,
    ) -> "ObservationBuilder":
        section = _get_section(config, "observation")
        return cls(
            num_lidar_beams=num_lidar_beams,
            max_lidar_range=max_lidar_range,
            max_linear_velocity=max_linear_velocity,
            max_angular_velocity=max_angular_velocity,
            enhanced=bool(section.get("enhanced", False)),
            sector_count=int(section.get("sector_count", 8)),
            include_final_goal=bool(section.get("include_final_goal", False)),
            include_path_features=bool(section.get("include_path_features", False)),
            heading_redesign=bool(section.get("heading_redesign", False)),
        )

    @property
    def base_feature_size(self) -> int:
        return 6 if self.include_final_goal else 4

    @property
    def single_obs_size(self) -> int:
        base_size = self.num_lidar_beams + self.base_feature_size
        if self.include_path_features:
            base_size += 4
        if not self.enhanced:
            enhanced_size = base_size
        else:
            enhanced_size = base_size + 13
        if self.heading_redesign:
            enhanced_size += len(self.HEADING_REDESIGN_FEATURE_NAMES)
        return enhanced_size

    def single_frame_schema(self) -> list[tuple[int, str]]:
        names = [f"lidar_{i:03d}" for i in range(self.num_lidar_beams)]
        names.extend(["goal_distance", "goal_angle"])
        if self.include_final_goal:
            names.extend(["final_goal_distance", "final_goal_angle"])
        names.extend(["linear_velocity", "angular_velocity"])
        if self.include_path_features:
            names.extend(
                [
                    "path_deviation",
                    "path_heading_error",
                    "path_remaining_distance",
                    "path_progress_fraction",
                ]
            )
        if self.enhanced:
            names.extend(
                [
                    "sector_front",
                    "sector_front_left",
                    "sector_left",
                    "sector_rear_left",
                    "sector_rear",
                    "sector_rear_right",
                    "sector_right",
                    "sector_front_right",
                    "last_action_linear",
                    "last_action_angular",
                    "safety_filter_intervened",
                    "stuck_progress",
                    "displacement_step",
                ]
            )
        if self.heading_redesign:
            names.extend(self.HEADING_REDESIGN_FEATURE_NAMES)
        return list(enumerate(names))

    def schema_lines(self, history_length: int = 1) -> list[str]:
        history = max(1, int(history_length))
        single = self.single_frame_schema()
        lines = [
            f"single_obs_size={self.single_obs_size}",
            f"history_length={history}",
            f"flattened_obs_size={self.single_obs_size * history}",
        ]
        for frame_index in range(history):
            frame_offset = frame_index * self.single_obs_size
            for single_index, name in single:
                lines.append(
                    f"frame={frame_index} single={single_index} flat={frame_offset + single_index} name={name}"
                )
        return lines

    def build(
        self,
        *,
        lidar: np.ndarray,
        goal_distance: float,
        goal_angle: float,
        final_goal_distance: Optional[float] = None,
        final_goal_angle: Optional[float] = None,
        linear_velocity: float,
        angular_velocity: float,
        last_action_linear: float,
        last_action_angular: float,
        safety_filter_intervened: bool,
        stuck_progress: float,
        displacement_step: float,
        path_deviation: float = 0.0,
        path_heading_error: float = 0.0,
        path_remaining_distance: float = 0.0,
        path_progress_fraction: float = 0.0,
        previous_target_angle: float = 0.0,
        previous_raw_angular_action: float = 0.0,
        previous_executed_angular_cmd: float = 0.0,
        previous_linear_cmd: float = 0.0,
        wrong_direction_last_step: bool = False,
        target_angle_reducing_last_step: bool = False,
    ) -> np.ndarray:
        lidar = clean_lidar(lidar, self.max_lidar_range)
        lidar_norm = np.clip(lidar / self.max_lidar_range, 0.0, 1.0)
        goal_distance_norm = np.clip(float(goal_distance) / 5.0, 0.0, 1.0)
        goal_angle_norm = np.clip(float(goal_angle) / math.pi, -1.0, 1.0)
        final_goal_distance_norm = np.clip(
            float(goal_distance if final_goal_distance is None else final_goal_distance) / 5.0,
            0.0,
            1.0,
        )
        final_goal_angle_norm = np.clip(
            float(goal_angle if final_goal_angle is None else final_goal_angle) / math.pi,
            -1.0,
            1.0,
        )
        linear_vel_norm = np.clip(
            float(linear_velocity) / max(self.max_linear_velocity, 1e-6),
            -1.0,
            1.0,
        )
        angular_vel_norm = np.clip(
            float(angular_velocity) / max(self.max_angular_velocity, 1e-6),
            -1.0,
            1.0,
        )
        base_features = [
            goal_distance_norm,
            goal_angle_norm,
            linear_vel_norm,
            angular_vel_norm,
        ]
        if self.include_final_goal:
            base_features = [
                goal_distance_norm,
                goal_angle_norm,
                final_goal_distance_norm,
                final_goal_angle_norm,
                linear_vel_norm,
                angular_vel_norm,
            ]
        parts = [
            lidar_norm.astype(np.float32),
            np.asarray(base_features, dtype=np.float32),
        ]
        if self.include_path_features:
            path_features = np.asarray(
                [
                    np.clip(float(path_deviation) / 1.0, -1.0, 1.0),
                    np.clip(float(path_heading_error) / math.pi, -1.0, 1.0),
                    np.clip(float(path_remaining_distance) / 5.0, 0.0, 1.0),
                    np.clip(float(path_progress_fraction), 0.0, 1.0),
                ],
                dtype=np.float32,
            )
            # Path layout: signed cross-track error, signed path-heading error,
            # remaining path distance, and normalized progress along the path.
            parts.append(path_features)
        if self.enhanced:
            angles = lidar_angles(lidar.size)
            sectors = sector_summary(lidar, angles, self.max_lidar_range)
            sector_values = [
                sectors["front"],
                sectors["front_left"],
                sectors["left"],
                sectors["rear_left"],
                sectors["rear"],
                sectors["rear_right"],
                sectors["right"],
                sectors["front_right"],
            ]
            extra = np.asarray(
                [
                    *[np.clip(v / self.max_lidar_range, 0.0, 1.0) for v in sector_values],
                    np.clip(float(last_action_linear), -1.0, 1.0),
                    np.clip(float(last_action_angular), -1.0, 1.0),
                    1.0 if safety_filter_intervened else 0.0,
                    np.clip(float(stuck_progress) / 0.25, -1.0, 1.0),
                    np.clip(float(displacement_step) / 0.10, 0.0, 1.0),
                ],
                dtype=np.float32,
            )
            # Enhanced layout: 8 sector clearances, previous action (2),
            # safety-filter flag, recent progress, and recent displacement.
            parts.append(extra)
        if self.heading_redesign:
            target_angle = math.atan2(math.sin(float(goal_angle)), math.cos(float(goal_angle)))
            prev_angle = math.atan2(
                math.sin(float(previous_target_angle)),
                math.cos(float(previous_target_angle)),
            )
            angle_delta = math.atan2(
                math.sin(target_angle - prev_angle),
                math.cos(target_angle - prev_angle),
            )
            target_sign = 1.0 if target_angle > 1e-4 else -1.0 if target_angle < -1e-4 else 0.0
            prev_sign = 1.0 if prev_angle > 1e-4 else -1.0 if prev_angle < -1e-4 else 0.0
            sign_crossing = bool(prev_sign != 0.0 and target_sign != 0.0 and prev_sign != target_sign)
            distance_norm = float(np.clip(goal_distance / 5.0, 0.0, 1.0))
            extra_heading = np.asarray(
                [
                    np.clip(target_angle / math.pi, -1.0, 1.0),
                    np.clip(math.sin(target_angle), -1.0, 1.0),
                    np.clip(math.cos(target_angle), -1.0, 1.0),
                    np.clip(abs(target_angle) / math.pi, 0.0, 1.0),
                    target_sign,
                    np.clip(prev_angle / math.pi, -1.0, 1.0),
                    np.clip(angle_delta / 0.10, -1.0, 1.0),
                    np.clip(abs(angle_delta) / 0.10, 0.0, 1.0),
                    np.clip(target_angle / 0.10, -1.0, 1.0),
                    np.clip(target_angle / 0.03, -1.0, 1.0),
                    distance_norm,
                    np.clip(distance_norm * target_angle / 0.10, -1.0, 1.0),
                    np.clip(float(previous_raw_angular_action), -1.0, 1.0),
                    np.clip(
                        float(previous_executed_angular_cmd)
                        / max(self.max_angular_velocity, 1e-6),
                        -1.0,
                        1.0,
                    ),
                    np.clip(
                        float(previous_linear_cmd) / max(self.max_linear_velocity, 1e-6),
                        -1.0,
                        1.0,
                    ),
                    1.0 if sign_crossing else 0.0,
                    1.0 if wrong_direction_last_step else 0.0,
                    1.0 if target_angle_reducing_last_step else 0.0,
                ],
                dtype=np.float32,
            )
            parts.append(extra_heading)
        return np.concatenate(parts).astype(np.float32)


@dataclass
class RewardConfig:
    smooth_obstacle_penalty_weight: float = 0.08
    front_obstacle_penalty_weight: float = 0.45
    blind_zone_penalty_weight: float = 2.50
    wall_proximity_penalty_weight: float = 0.55
    angular_velocity_penalty_weight: float = 0.0005
    action_smoothness_penalty_weight: float = 0.015
    no_progress_penalty_weight: float = 0.004
    safety_filter_intervention_penalty: float = 0.010
    safe_progress_clearance_bonus: float = 0.0005

    @classmethod
    def from_config(cls, config: Dict[str, Any]) -> "RewardConfig":
        section = _get_section(config, "reward")
        defaults = cls()
        return cls(
            **{
                field_name: float(section.get(field_name, getattr(defaults, field_name)))
                for field_name in cls.__dataclass_fields__
            }
        )


class RewardFunction:
    """Smooth navigation-shaping terms that do not select behavior."""

    def __init__(
        self,
        reward_config: RewardConfig,
        safety_config: SafetyFilterConfig,
    ) -> None:
        self.reward_config = reward_config
        self.safety_config = safety_config

    def _smooth_clearance_pressure(self, clearance: float) -> float:
        caution = max(self.safety_config.caution_clearance, self.safety_config.critical_clearance)
        critical = max(self.safety_config.critical_clearance, self.safety_config.lidar_min_range)
        if clearance >= caution:
            return 0.0
        if clearance <= critical:
            return 1.0
        return float((caution - clearance) / max(caution - critical, 1e-6))

    def compute_terms(
        self,
        *,
        min_lidar: float,
        front_min: float,
        wall_clearance: float,
        linear_velocity: float,
        angular_velocity: float,
        previous_linear_cmd: float,
        previous_angular_cmd: float,
        current_linear_cmd: float,
        current_angular_cmd: float,
        distance_change: float,
        safety_filter_intervened: bool,
        collision_risk: bool,
    ) -> Dict[str, float]:
        terms: Dict[str, float] = {}
        global_pressure = self._smooth_clearance_pressure(float(min_lidar))
        front_pressure = self._smooth_clearance_pressure(float(front_min))

        caution = max(self.safety_config.caution_clearance, self.safety_config.critical_clearance)
        critical = max(self.safety_config.critical_clearance, self.safety_config.lidar_min_range)
        near_blind_buffer = critical + max(self.safety_config.safety_margin, 0.04)
        front_path_clear = float(front_min) >= caution
        side_only_pressure = front_path_clear and float(min_lidar) < caution
        if side_only_pressure and float(min_lidar) >= near_blind_buffer:
            # A side obstacle in a physically valid corridor should not teach
            # the policy to abandon a clear route to the target.
            global_pressure = 0.0
        elif side_only_pressure:
            # Still discourage scraping into the blind/critical band.
            global_pressure *= 0.35

        forward_pressure = max(0.0, float(linear_velocity) / 0.25)
        terms["smooth_obstacle_penalty"] = (
            self.reward_config.smooth_obstacle_penalty_weight * global_pressure**2
        )
        terms["front_obstacle_penalty"] = (
            self.reward_config.front_obstacle_penalty_weight
            * front_pressure**2
            * (0.20 + min(1.0, forward_pressure))
        )

        blind_pressure = 0.0
        if min_lidar < self.safety_config.critical_clearance:
            blind_pressure = float(
                np.clip(
                    (self.safety_config.critical_clearance - min_lidar)
                    / max(
                        self.safety_config.critical_clearance
                        - self.safety_config.lidar_min_range,
                        1e-6,
                    ),
                    0.0,
                    1.0,
                )
            )
        terms["blind_zone_penalty"] = (
            self.reward_config.blind_zone_penalty_weight * blind_pressure**2
        )

        wall_pressure = self._smooth_clearance_pressure(float(wall_clearance))
        terms["wall_proximity_penalty"] = (
            self.reward_config.wall_proximity_penalty_weight * wall_pressure**2
        )

        terms["angular_velocity_penalty"] = (
            self.reward_config.angular_velocity_penalty_weight * abs(float(angular_velocity))
        )
        action_delta = (float(current_linear_cmd) - float(previous_linear_cmd)) ** 2 + (
            float(current_angular_cmd) - float(previous_angular_cmd)
        ) ** 2
        terms["action_smoothness_penalty"] = (
            self.reward_config.action_smoothness_penalty_weight * action_delta
        )
        terms["safety_filter_intervention_penalty"] = (
            self.reward_config.safety_filter_intervention_penalty
            if safety_filter_intervened
            else 0.0
        )
        no_progress = distance_change < 0.0005 and abs(linear_velocity) < 0.02
        terms["no_progress_penalty"] = (
            self.reward_config.no_progress_penalty_weight if no_progress else 0.0
        )
        terms["safe_progress_clearance_bonus"] = (
            self.reward_config.safe_progress_clearance_bonus
            if distance_change > 0.001
            and front_min > self.safety_config.caution_clearance
            and min_lidar > near_blind_buffer
            and not collision_risk
            else 0.0
        )
        return terms


@dataclass
class StuckState:
    stuck: bool
    reason: str = ""
    window_progress: float = 0.0
    window_displacement: float = 0.0
    intervention_ratio: float = 0.0
    collision_risk_ratio: float = 0.0
    oscillation_count: int = 0


class StuckDetector:
    """Detector for true no-progress events, not normal slow corridor motion."""

    def __init__(self, config: Dict[str, Any]) -> None:
        section = _get_section(config, "stuck_detector")
        self.window_steps = max(1, int(section.get("window_steps", 180)))
        self.min_progress = max(0.0, float(section.get("min_progress", 0.02)))
        self.min_displacement = max(0.0, float(section.get("min_displacement", 0.04)))
        self.low_speed_threshold = max(0.0, float(section.get("low_speed_threshold", 0.015)))
        self.oscillation_zero_crossings = max(1, int(section.get("oscillation_zero_crossings", 8)))
        self.intervention_ratio_threshold = float(section.get("intervention_ratio", 0.35))
        self.collision_risk_ratio_threshold = float(section.get("collision_risk_ratio", 0.30))
        self.reset()

    def reset(self) -> None:
        self.goal_distances: list[float] = []
        self.positions: list[Tuple[float, float]] = []
        self.angular_signs: list[int] = []
        self.interventions: list[bool] = []
        self.collision_risks: list[bool] = []

    def update(
        self,
        *,
        step: int,
        check_after_steps: int,
        goal_distance: float,
        x: float,
        y: float,
        linear_velocity: float,
        angular_velocity: float,
        safety_intervened: bool,
        collision_risk: bool,
        min_progress_override: Optional[float] = None,
    ) -> StuckState:
        self.goal_distances.append(float(goal_distance))
        self.positions.append((float(x), float(y)))
        self.angular_signs.append(1 if angular_velocity > 0.03 else -1 if angular_velocity < -0.03 else 0)
        self.interventions.append(bool(safety_intervened))
        self.collision_risks.append(bool(collision_risk))
        for values in (
            self.goal_distances,
            self.positions,
            self.angular_signs,
            self.interventions,
            self.collision_risks,
        ):
            if len(values) > self.window_steps:
                values.pop(0)

        if step <= check_after_steps or len(self.goal_distances) < self.window_steps:
            return StuckState(False)

        window_progress = self.goal_distances[0] - self.goal_distances[-1]
        start_x, start_y = self.positions[0]
        end_x, end_y = self.positions[-1]
        displacement = math.hypot(end_x - start_x, end_y - start_y)
        intervention_ratio = sum(self.interventions) / max(len(self.interventions), 1)
        collision_risk_ratio = sum(self.collision_risks) / max(len(self.collision_risks), 1)
        non_zero_signs = [sign for sign in self.angular_signs if sign != 0]
        zero_crossings = sum(
            1
            for prev, cur in zip(non_zero_signs, non_zero_signs[1:])
            if prev != cur
        )
        min_progress = (
            self.min_progress
            if min_progress_override is None
            else max(0.0, float(min_progress_override))
        )
        no_progress = window_progress < min_progress
        no_displacement = displacement < self.min_displacement
        low_speed = abs(float(linear_velocity)) <= self.low_speed_threshold
        external_pressure = (
            intervention_ratio >= self.intervention_ratio_threshold
            or collision_risk_ratio >= self.collision_risk_ratio_threshold
        )
        oscillating = zero_crossings >= self.oscillation_zero_crossings

        stuck = bool(
            no_progress
            and no_displacement
            and (low_speed or external_pressure or oscillating)
        )
        reason = ""
        if stuck:
            reason_parts = ["no_progress", "low_displacement"]
            if low_speed:
                reason_parts.append("low_speed")
            if external_pressure:
                reason_parts.append("safety_pressure")
            if oscillating:
                reason_parts.append("oscillation")
            reason = "+".join(reason_parts)
        return StuckState(
            stuck,
            reason,
            window_progress,
            displacement,
            intervention_ratio,
            collision_risk_ratio,
            zero_crossings,
        )


class HumanInteractionSupervisor:
    """High-level human safety/wait state. It does not steer around humans."""

    NAVIGATE = "NAVIGATE"
    HUMAN_WAIT = "HUMAN_WAIT"
    REPLAN = "REPLAN"
    EMERGENCY_STOP = "EMERGENCY_STOP"

    def __init__(self, config: Dict[str, Any], control_period: float) -> None:
        section = _get_section(config, "human_supervisor")
        self.enabled = bool(section.get("enabled", True))
        self.safety_radius = max(0.0, float(section.get("safety_radius", 0.45)))
        self.path_corridor_width = max(0.0, float(section.get("path_corridor_width", 0.35)))
        wait_seconds = max(0.0, float(section.get("wait_seconds", 3.0)))
        dt = control_period if control_period > 0.0 else 0.10
        self.wait_steps = int(math.ceil(wait_seconds / max(dt, 1e-3)))
        self.reset()

    def reset(self) -> None:
        self.state = self.NAVIGATE
        self.wait_until_step = 0
        self.event = ""
        self.wait_count = 0
        self.replan_count = 0
        self.blocked_no_path_count = 0

    def update(self, *, step: int, human_blocks_path: bool) -> Tuple[bool, str]:
        self.event = ""
        if not self.enabled:
            self.state = self.NAVIGATE
            return False, ""

        if human_blocks_path:
            if self.state != self.HUMAN_WAIT:
                self.wait_until_step = step + self.wait_steps
                self.wait_count += 1
                self.event = "human_wait"
            self.state = self.HUMAN_WAIT
            if step <= self.wait_until_step:
                return True, "human_wait"
            self.state = self.REPLAN
            self.replan_count += 1
            self.event = "human_replan"
            return True, "human_replan"

        if self.state in {self.HUMAN_WAIT, self.REPLAN}:
            self.event = "human_cleared"
        self.state = self.NAVIGATE
        return False, ""


@dataclass
class PreAlignConfig:
    enabled: bool = False
    run_at_episode_start: bool = True
    run_after_replan: bool = True
    run_after_waypoint_switch: bool = False
    waypoint_switch_min_angle: float = 0.08
    waypoint_switch_cooldown_steps: int = 10
    heading_threshold: float = 0.05
    start_heading_threshold: float = 0.10
    timeout_seconds: float = 6.00
    timeout_step_cap: int = 360
    angular_velocity: float = 0.95
    min_angular_velocity: float = 0.025
    angular_gain: float = 1.00
    release_ramp_steps: int = 35
    release_linear_cap: float = 0.08
    release_angular_cap: float = 0.35
    min_clearance: float = 0.38
    front_clearance: float = 0.35
    rotation_clearance_margin: float = 0.12
    use_local_waypoint: bool = True

    @classmethod
    def from_config(cls, config: Dict[str, Any]) -> "PreAlignConfig":
        section = _get_section(config, "navigation_supervisor")
        defaults = cls()
        return cls(
            enabled=bool(section.get("pre_align_enabled", defaults.enabled)),
            run_at_episode_start=bool(
                section.get(
                    "pre_align_run_at_episode_start",
                    defaults.run_at_episode_start,
                )
            ),
            run_after_replan=bool(
                section.get("pre_align_run_after_replan", defaults.run_after_replan)
            ),
            run_after_waypoint_switch=bool(
                section.get(
                    "pre_align_run_after_waypoint_switch",
                    defaults.run_after_waypoint_switch,
                )
            ),
            waypoint_switch_min_angle=max(
                0.0,
                float(
                    section.get(
                        "pre_align_waypoint_switch_min_angle",
                        defaults.waypoint_switch_min_angle,
                    )
                ),
            ),
            waypoint_switch_cooldown_steps=max(
                0,
                int(
                    section.get(
                        "pre_align_waypoint_switch_cooldown_steps",
                        defaults.waypoint_switch_cooldown_steps,
                    )
                ),
            ),
            heading_threshold=max(
                0.0,
                float(
                    section.get(
                        "pre_align_heading_threshold",
                        defaults.heading_threshold,
                    )
                ),
            ),
            start_heading_threshold=max(
                0.0,
                float(
                    section.get(
                        "pre_align_start_heading_threshold",
                        defaults.start_heading_threshold,
                    )
                ),
            ),
            timeout_seconds=max(
                0.0,
                float(
                    section.get(
                        "pre_align_timeout_seconds",
                        defaults.timeout_seconds,
                    )
                ),
            ),
            timeout_step_cap=max(
                1,
                int(
                    section.get(
                        "pre_align_timeout_step_cap",
                        defaults.timeout_step_cap,
                    )
                ),
            ),
            angular_velocity=max(
                0.0,
                float(
                    section.get(
                        "pre_align_angular_velocity",
                        defaults.angular_velocity,
                    )
                ),
            ),
            min_angular_velocity=max(
                0.0,
                float(
                    section.get(
                        "pre_align_min_angular_velocity",
                        defaults.min_angular_velocity,
                    )
                ),
            ),
            angular_gain=max(
                0.0,
                float(section.get("pre_align_angular_gain", defaults.angular_gain)),
            ),
            release_ramp_steps=max(
                0,
                int(
                    section.get(
                        "pre_align_release_ramp_steps",
                        defaults.release_ramp_steps,
                    )
                ),
            ),
            release_linear_cap=max(
                0.0,
                float(
                    section.get(
                        "pre_align_release_linear_cap",
                        defaults.release_linear_cap,
                    )
                ),
            ),
            release_angular_cap=max(
                0.0,
                float(
                    section.get(
                        "pre_align_release_angular_cap",
                        defaults.release_angular_cap,
                    )
                ),
            ),
            min_clearance=max(
                0.0,
                float(
                    section.get(
                        "pre_align_min_clearance",
                        defaults.min_clearance,
                    )
                ),
            ),
            front_clearance=max(
                0.0,
                float(
                    section.get(
                        "pre_align_front_clearance",
                        defaults.front_clearance,
                    )
                ),
            ),
            rotation_clearance_margin=max(
                0.0,
                float(
                    section.get(
                        "pre_align_rotation_clearance_margin",
                        defaults.rotation_clearance_margin,
                    )
                ),
            ),
            use_local_waypoint=bool(
                section.get(
                    "pre_align_use_local_waypoint",
                    defaults.use_local_waypoint,
                )
            ),
        )


@dataclass
class PreAlignDecision:
    active: bool
    angular_velocity: float = 0.0
    reason: str = ""
    event: str = ""
    exit_reason: str = ""
    target_angle: float = 0.0


class PreAlignSupervisor:
    """Short-lived start/replan heading alignment, never normal navigation."""

    def __init__(self, config: PreAlignConfig, control_period: float) -> None:
        self.config = config
        self.control_period = float(control_period)
        dt = control_period if control_period > 0.0 else 0.05
        fallback_timeout_steps = int(math.ceil(self.config.timeout_seconds / max(dt, 1e-3)))
        if self.control_period > 0.0:
            self.timeout_steps = max(1, fallback_timeout_steps)
        else:
            self.timeout_steps = max(
                1,
                int(self.config.timeout_step_cap),
                fallback_timeout_steps,
            )
        self.reset_episode()

    def reset_episode(self) -> None:
        self.active = False
        self.pending = bool(self.config.enabled and self.config.run_at_episode_start)
        self.started_step = 0
        self.started_monotonic = 0.0
        self.trigger_reason = "episode_start" if self.pending else ""
        self.event = ""
        self.exit_reason = ""

    def request_after_replan(self, reason: str = "replan") -> bool:
        if not (self.config.enabled and self.config.run_after_replan):
            return False
        self.active = False
        self.pending = True
        self.started_step = 0
        self.started_monotonic = 0.0
        self.trigger_reason = reason
        self.event = "pre_align_requested"
        self.exit_reason = ""
        return True

    def request_after_waypoint_switch(self, reason: str = "waypoint_switch") -> bool:
        if not (self.config.enabled and self.config.run_after_waypoint_switch):
            return False
        if self.active or self.pending:
            return False
        self.active = False
        self.pending = True
        self.started_step = 0
        self.started_monotonic = 0.0
        self.trigger_reason = reason
        self.event = "pre_align_requested"
        self.exit_reason = ""
        return True

    def _start_threshold_for_trigger(self) -> float:
        route_update_reasons = (
            "waypoint",
            "adaptive_waypoint",
            "static_detour",
            "final_goal_rejoin",
            "stuck_recovery",
            "human_replan",
        )
        reason = str(self.trigger_reason or "")
        if reason == "episode_start":
            # At reset, PRE_ALIGN is deliberately requested so the first SAC
            # action begins from a clean heading. Do not skip visible residual
            # errors just because they are below the looser start threshold.
            return self.config.heading_threshold
        if any(reason.startswith(prefix) for prefix in route_update_reasons):
            return max(
                self.config.heading_threshold,
                self.config.waypoint_switch_min_angle,
            )
        return max(
            self.config.heading_threshold,
            self.config.start_heading_threshold,
        )

    def cancel(self, reason: str) -> PreAlignDecision:
        was_relevant = self.active or self.pending
        self.active = False
        self.pending = False
        self.event = "pre_align_exit" if was_relevant else ""
        self.exit_reason = reason if was_relevant else ""
        return PreAlignDecision(False, reason=self.trigger_reason, event=self.event, exit_reason=self.exit_reason)

    def update(
        self,
        *,
        step: int,
        target_angle: float,
        safe_to_rotate: bool,
        obstacle_too_close: bool,
        human_stop: bool,
    ) -> PreAlignDecision:
        self.event = ""
        self.exit_reason = ""
        target_angle = float(target_angle)
        if not self.config.enabled:
            self.active = False
            self.pending = False
            return PreAlignDecision(False, target_angle=target_angle)

        if human_stop:
            return self.cancel("human_stop")
        if obstacle_too_close:
            return self.cancel("obstacle_or_human_too_close")
        if not safe_to_rotate:
            return self.cancel("rotation_unsafe")

        abs_error = abs(target_angle)
        if abs_error <= self.config.heading_threshold:
            return self.cancel("aligned")

        if not self.active:
            if not self.pending:
                return PreAlignDecision(False, target_angle=target_angle)
            if abs_error < self._start_threshold_for_trigger():
                return self.cancel("below_start_threshold")
            self.active = True
            self.pending = False
            self.started_step = int(step)
            self.started_monotonic = time.monotonic()
            self.event = "pre_align_start"

        elapsed = max(0, int(step) - int(self.started_step))
        elapsed_time = (
            time.monotonic() - float(self.started_monotonic)
            if self.started_monotonic > 0.0
            else elapsed * (self.control_period if self.control_period > 0.0 else 0.05)
        )
        # Use control steps for PRE_ALIGN timeout. Wall-clock time can include
        # Gazebo GUI/service delays where little simulated motion happened,
        # which released alignment before the robot was actually facing target.
        if elapsed >= self.timeout_steps:
            return self.cancel("timeout")

        direction = 1.0 if target_angle >= 0.0 else -1.0
        angular_mag = min(
            self.config.angular_velocity,
            max(
                self.config.min_angular_velocity,
                self.config.angular_gain * abs_error,
            ),
        )
        return PreAlignDecision(
            True,
            angular_velocity=direction * angular_mag,
            reason=self.trigger_reason,
            event=self.event,
            target_angle=target_angle,
        )


class NavigationSupervisor:
    """High-level state label used for logs and evaluation."""

    NAVIGATE = "NAVIGATE"
    PRE_ALIGN = "PRE_ALIGN"
    HUMAN_WAIT = "HUMAN_WAIT"
    REPLAN = "REPLAN"
    RECOVERY = "RECOVERY"
    GOAL_REACHED = "GOAL_REACHED"
    EMERGENCY_STOP = "EMERGENCY_STOP"

    def __init__(self) -> None:
        self.state = self.NAVIGATE

    def update(
        self,
        *,
        goal_reached: bool,
        human_stop: bool,
        human_state: str,
        stuck: bool,
        emergency_stop: bool,
        pre_align_active: bool = False,
    ) -> str:
        if goal_reached:
            self.state = self.GOAL_REACHED
        elif emergency_stop:
            self.state = self.EMERGENCY_STOP
        elif human_stop:
            self.state = self.REPLAN if human_state == HumanInteractionSupervisor.REPLAN else self.HUMAN_WAIT
        elif stuck:
            self.state = self.RECOVERY
        elif pre_align_active:
            self.state = self.PRE_ALIGN
        else:
            self.state = self.NAVIGATE
        return self.state


class MetricsLogger:
    """Per-episode counters for diagnostics."""

    def __init__(self) -> None:
        self.reset_episode()

    def reset_episode(self) -> None:
        self.safety_filter_interventions = 0
        self.safety_filter_reasons: Dict[str, int] = {}
        self.min_clearance = float("inf")
        self.near_obstacle_steps = 0
        self.narrow_passage_steps = 0
        self.oscillation_count = 0
        self.human_wait_events = 0
        self.human_replan_events = 0
        self.recovery_events = 0
        self.pre_align_events = 0
        self.pre_align_steps = 0
        self.pre_align_exit_reasons: Dict[str, int] = {}

    def record_clearance(
        self,
        *,
        min_clearance: float,
        left_clearance: float,
        right_clearance: float,
        caution_clearance: float,
    ) -> None:
        self.min_clearance = min(self.min_clearance, float(min_clearance))
        if min_clearance < caution_clearance:
            self.near_obstacle_steps += 1
        if left_clearance < caution_clearance and right_clearance < caution_clearance:
            self.narrow_passage_steps += 1

    def record_safety_filter(self, result: SafetyFilterResult) -> None:
        if not result.intervened:
            return
        self.safety_filter_interventions += 1
        reason = result.reason or "soft_limited"
        self.safety_filter_reasons[reason] = self.safety_filter_reasons.get(reason, 0) + 1

    def record_human_event(self, event: str) -> None:
        if event == "human_wait":
            self.human_wait_events += 1
        elif event == "human_replan":
            self.human_replan_events += 1

    def record_pre_align(self, active: bool, event: str = "", exit_reason: str = "") -> None:
        if active:
            self.pre_align_steps += 1
        if event == "pre_align_start":
            self.pre_align_events += 1
        if exit_reason:
            self.pre_align_exit_reasons[exit_reason] = (
                self.pre_align_exit_reasons.get(exit_reason, 0) + 1
            )

    def record_recovery(self) -> None:
        self.recovery_events += 1


class TrainingCurriculum:
    """Documentation object for the SAC-only curriculum stages."""

    STAGES = (
        "center_stage0_angular_recovery",
        "center_stage0_empty",
        "center_stage1_one_box",
        "center_stage2_three_boxes",
        "center_stage3_five_boxes",
        "center_stage4_five_boxes_slow_human",
        "center_stage5_dynamic_human_medium",
        "center_stage6_final",
    )


class SACPolicy:
    """Semantic placeholder for the Stable-Baselines3 SAC policy.

    The actual SAC implementation remains Stable-Baselines3. This class exists
    only so the refactored architecture has a clear policy boundary.
    """

    source = "Stable-Baselines3 SAC"
