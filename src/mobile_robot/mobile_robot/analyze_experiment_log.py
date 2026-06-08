#!/usr/bin/env python3
"""Generate advisor-ready plots and validation summaries from action logs."""

import argparse
import csv
import json
import math
import statistics
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


CANONICAL_COLUMNS = {
    "step": ["step"],
    "time_sec": ["time_sec", "elapsed_time", "ros_time"],
    "odom_x": ["odom_x"],
    "odom_y": ["odom_y"],
    "yaw": ["yaw", "odom_yaw"],
    "odom_linear_x": ["odom_linear_x"],
    "odom_angular_z": ["odom_angular_z"],
    "cmd_linear_x": ["cmd_linear_x"],
    "cmd_angular_z": ["cmd_angular_z"],
    "goal_x": ["goal_x", "target_x"],
    "goal_y": ["goal_y", "target_y"],
    "distance_to_goal": ["distance_to_goal", "goal_distance"],
    "angle_to_goal": ["angle_to_goal", "goal_angle"],
    "min_lidar_range": ["min_lidar_range"],
    "mean_lidar_range": ["mean_lidar_range"],
    "collision_flag": ["collision_flag", "collision"],
    "reached_goal_flag": ["reached_goal_flag", "success", "goal_reached"],
    "control_source": ["control_source"],
}

IMPORTANT_COLUMNS = [
    "step",
    "time_sec",
    "odom_x",
    "odom_y",
    "yaw",
    "odom_linear_x",
    "odom_angular_z",
    "cmd_linear_x",
    "cmd_angular_z",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Analyze a passive robot action log and generate validation plots."
    )
    parser.add_argument("--log", required=True, help="CSV log file, for example robot_action_log.csv")
    parser.add_argument("--out", required=True, help="Output folder for PNG plots")
    parser.add_argument("--goal-x", type=float, default=float("nan"))
    parser.add_argument("--goal-y", type=float, default=float("nan"))
    parser.add_argument("--success-radius", type=float, default=0.20)
    parser.add_argument("--collision-threshold", type=float, default=0.20)
    parser.add_argument(
        "--scan-snapshots",
        default="",
        help="Optional scan snapshot CSV written by action_logger_node.",
    )
    parser.add_argument(
        "--summary-dir",
        default="",
        help="Optional directory for experiment_validation_summary.txt/csv.",
    )
    parser.add_argument(
        "--expected-control-source",
        default="autonomous_sac",
        help="Expected value in the control_source column.",
    )
    parser.add_argument(
        "--route-system-expected",
        dest="route_system_expected",
        action="store_true",
        help="Mark autonomous route-generation evidence as expected instead of suspicious.",
    )
    return parser.parse_args()


def expand_path(path_text: str) -> Path:
    return Path(path_text).expanduser().resolve()


def warn(message: str) -> None:
    print(f"WARNING: {message}")


def as_float(value: object) -> Optional[float]:
    if value is None:
        return None
    text = str(value).strip()
    if text == "" or text.lower() in {"nan", "none", "null", "inf", "-inf"}:
        return None
    try:
        number = float(text)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def as_bool(value: object) -> Optional[bool]:
    if value is None:
        return None
    text = str(value).strip().lower()
    if text == "":
        return None
    if text in {"1", "true", "t", "yes", "y"}:
        return True
    if text in {"0", "false", "f", "no", "n"}:
        return False
    number = as_float(text)
    if number is None:
        return None
    return bool(number)


def load_rows(csv_path: Path) -> Tuple[List[Dict[str, str]], List[str]]:
    with csv_path.open("r", encoding="utf-8", newline="") as file_obj:
        reader = csv.DictReader(file_obj)
        rows = list(reader)
        fieldnames = list(reader.fieldnames or [])
    if not rows:
        warn(f"log file has no data rows: {csv_path}")
    return rows, fieldnames


def find_column(fieldnames: Sequence[str], canonical_name: str) -> Optional[str]:
    for candidate in CANONICAL_COLUMNS.get(canonical_name, [canonical_name]):
        if candidate in fieldnames:
            return candidate
    return None


def float_series(
    rows: Sequence[Dict[str, str]],
    fieldnames: Sequence[str],
    canonical_name: str,
    default: Optional[float] = None,
) -> List[float]:
    column = find_column(fieldnames, canonical_name)
    result: List[float] = []
    for row in rows:
        value = as_float(row.get(column)) if column else None
        if value is None and default is not None:
            value = default
        result.append(value if value is not None else float("nan"))
    return result


def bool_series(
    rows: Sequence[Dict[str, str]],
    fieldnames: Sequence[str],
    canonical_name: str,
) -> List[Optional[bool]]:
    column = find_column(fieldnames, canonical_name)
    if not column:
        return [None for _ in rows]
    return [as_bool(row.get(column)) for row in rows]


def string_series(
    rows: Sequence[Dict[str, str]],
    fieldnames: Sequence[str],
    canonical_name: str,
) -> List[str]:
    column = find_column(fieldnames, canonical_name)
    if not column:
        return ["" for _ in rows]
    return [str(row.get(column, "")).strip() for row in rows]


def has_finite(values: Iterable[float]) -> bool:
    return any(math.isfinite(value) for value in values)


def finite_values(values: Iterable[float]) -> List[float]:
    return [value for value in values if math.isfinite(value)]


def mean_or_none(values: Iterable[float]) -> Optional[float]:
    finite = finite_values(values)
    return sum(finite) / len(finite) if finite else None


def min_or_none(values: Iterable[float]) -> Optional[float]:
    finite = finite_values(values)
    return min(finite) if finite else None


def max_abs_or_none(values: Iterable[float]) -> Optional[float]:
    finite = finite_values(values)
    return max((abs(value) for value in finite), default=None)


def fmt(value: object, digits: int = 3) -> str:
    if value is None:
        return "unavailable"
    if isinstance(value, bool):
        return "yes" if value else "no"
    if isinstance(value, float):
        if not math.isfinite(value):
            return "unavailable"
        return f"{value:.{digits}f}"
    return str(value)


def fmt_pair(x_value: Optional[float], y_value: Optional[float]) -> str:
    if x_value is None or y_value is None:
        return "unavailable"
    return f"({x_value:.3f}, {y_value:.3f})"


def normalize_times(raw_times: List[float], row_count: int) -> List[float]:
    finite = finite_values(raw_times)
    if not finite:
        return [float(index) for index in range(row_count)]
    first = finite[0]
    normalized = []
    for index, value in enumerate(raw_times):
        if math.isfinite(value):
            normalized.append(value - first)
        else:
            normalized.append(float(index))
    return normalized


def timestamp_text(raw_time: Optional[float]) -> str:
    if raw_time is None:
        return "unavailable"
    if raw_time > 1_000_000_000:
        try:
            return f"{raw_time:.3f} ({datetime.fromtimestamp(raw_time).isoformat()})"
        except (OSError, OverflowError, ValueError):
            return f"{raw_time:.3f}"
    return f"{raw_time:.3f} sec"


def path_length(xs: Sequence[float], ys: Sequence[float]) -> float:
    total = 0.0
    previous: Optional[Tuple[float, float]] = None
    for x_value, y_value in zip(xs, ys):
        if not math.isfinite(x_value) or not math.isfinite(y_value):
            continue
        current = (x_value, y_value)
        if previous is not None:
            step = math.hypot(current[0] - previous[0], current[1] - previous[1])
            if step < 5.0:
                total += step
        previous = current
    return total


def first_last(values: Sequence[float]) -> Tuple[Optional[float], Optional[float]]:
    finite = finite_values(values)
    if not finite:
        return None, None
    return finite[0], finite[-1]


def fill_goal_series(values: List[float], fallback: float) -> List[float]:
    if math.isfinite(fallback):
        return [value if math.isfinite(value) else fallback for value in values]
    return values


def compute_distance_and_angle(
    xs: Sequence[float],
    ys: Sequence[float],
    yaws: Sequence[float],
    goal_xs: Sequence[float],
    goal_ys: Sequence[float],
    distances: List[float],
    angles: List[float],
) -> Tuple[List[float], List[float]]:
    computed_distances = list(distances)
    computed_angles = list(angles)
    for index, (x_value, y_value, yaw_value, goal_x, goal_y) in enumerate(
        zip(xs, ys, yaws, goal_xs, goal_ys)
    ):
        if not (
            math.isfinite(x_value)
            and math.isfinite(y_value)
            and math.isfinite(goal_x)
            and math.isfinite(goal_y)
        ):
            continue
        dx = goal_x - x_value
        dy = goal_y - y_value
        if not math.isfinite(computed_distances[index]):
            computed_distances[index] = math.hypot(dx, dy)
        if math.isfinite(yaw_value) and not math.isfinite(computed_angles[index]):
            raw_angle = math.atan2(dy, dx) - yaw_value
            computed_angles[index] = math.atan2(math.sin(raw_angle), math.cos(raw_angle))
    return computed_distances, computed_angles


def import_matplotlib():
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    return plt


def save_missing_plot(path: Path, title: str, message: str) -> None:
    plt = import_matplotlib()
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.text(0.5, 0.5, message, ha="center", va="center", wrap=True)
    ax.set_title(title)
    ax.set_axis_off()
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def save_line_plot(
    path: Path,
    title: str,
    xlabel: str,
    ylabel: str,
    x_values: Sequence[float],
    y_values: Sequence[float],
    label: str,
    threshold: Optional[float] = None,
    threshold_label: str = "threshold",
) -> None:
    if not has_finite(y_values):
        warn(f"missing data for {path.name}")
        save_missing_plot(path, title, f"Missing data for {label}.")
        return

    plt = import_matplotlib()
    fig, ax = plt.subplots(figsize=(10, 4.8))
    ax.plot(x_values, y_values, linewidth=1.8, label=label)
    if threshold is not None and math.isfinite(threshold):
        ax.axhline(threshold, color="tab:red", linestyle="--", label=threshold_label)
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.35)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def save_path_plot(
    path: Path,
    xs: Sequence[float],
    ys: Sequence[float],
    goal_x: Optional[float],
    goal_y: Optional[float],
    success_radius: float,
) -> None:
    xy = [
        (x_value, y_value)
        for x_value, y_value in zip(xs, ys)
        if math.isfinite(x_value) and math.isfinite(y_value)
    ]
    if not xy:
        warn("missing odometry x-y data for robot path plot")
        save_missing_plot(path, "Robot Path", "Missing odometry position columns.")
        return

    plt = import_matplotlib()
    fig, ax = plt.subplots(figsize=(7, 7))
    plot_x, plot_y = zip(*xy)
    ax.plot(plot_x, plot_y, linewidth=2.0, label="odometry path")
    ax.scatter([plot_x[0]], [plot_y[0]], marker="o", s=70, label="start")
    ax.scatter([plot_x[-1]], [plot_y[-1]], marker="x", s=90, label="final")
    if goal_x is not None and goal_y is not None:
        ax.scatter([goal_x], [goal_y], marker="*", s=150, label="goal")
        if math.isfinite(success_radius) and success_radius > 0.0:
            circle = plt.Circle(
                (goal_x, goal_y),
                success_radius,
                fill=False,
                linestyle="--",
                color="tab:green",
                label=f"success radius {success_radius:.2f} m",
            )
            ax.add_patch(circle)
    ax.set_title("Robot Path")
    ax.set_xlabel("odom x (m)")
    ax.set_ylabel("odom y (m)")
    ax.axis("equal")
    ax.grid(True, alpha=0.35)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def save_cmd_vs_odom_plot(
    path: Path,
    times: Sequence[float],
    cmd_linear: Sequence[float],
    odom_linear: Sequence[float],
    cmd_angular: Sequence[float],
    odom_angular: Sequence[float],
) -> None:
    if not (has_finite(cmd_linear) or has_finite(odom_linear) or has_finite(cmd_angular)):
        warn("missing command and odometry velocity data")
        save_missing_plot(path, "Commanded vs Odometry Velocity", "Missing velocity columns.")
        return

    plt = import_matplotlib()
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    axes[0].plot(times, cmd_linear, label="cmd linear x")
    axes[0].plot(times, odom_linear, label="odom linear x")
    axes[0].set_ylabel("linear velocity (m/s)")
    axes[0].grid(True, alpha=0.35)
    axes[0].legend()
    axes[1].plot(times, cmd_angular, label="cmd angular z")
    axes[1].plot(times, odom_angular, label="odom angular z")
    axes[1].set_xlabel("time (s)")
    axes[1].set_ylabel("angular velocity (rad/s)")
    axes[1].grid(True, alpha=0.35)
    axes[1].legend()
    fig.suptitle("Commanded Velocity Compared with Odometry Velocity")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def save_collision_plot(
    path: Path,
    times: Sequence[float],
    collision_flags: Sequence[Optional[bool]],
) -> None:
    values = [1.0 if value else 0.0 if value is False else float("nan") for value in collision_flags]
    save_line_plot(
        path,
        "Collision Flag",
        "time (s)",
        "collision flag",
        times,
        values,
        "collision_flag",
    )


def save_timeline_plot(
    path: Path,
    steps: Sequence[float],
    cmd_linear: Sequence[float],
    cmd_angular: Sequence[float],
) -> None:
    if not (has_finite(cmd_linear) or has_finite(cmd_angular)):
        warn("missing command data for command timeline")
        save_missing_plot(path, "Control Command Timeline", "Missing /cmd_vel columns.")
        return

    plt = import_matplotlib()
    fig, ax = plt.subplots(figsize=(10, 4.8))
    ax.plot(steps, cmd_linear, label="cmd linear x")
    ax.plot(steps, cmd_angular, label="cmd angular z")
    ax.set_title("Control Command Timeline")
    ax.set_xlabel("step")
    ax.set_ylabel("command value")
    ax.grid(True, alpha=0.35)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def save_success_plot(
    path: Path,
    times: Sequence[float],
    reached_flags: Sequence[Optional[bool]],
    distances: Sequence[float],
    success_radius: float,
) -> None:
    if any(value is not None for value in reached_flags):
        values = [1.0 if value else 0.0 if value is False else float("nan") for value in reached_flags]
        save_line_plot(
            path,
            "Success Evaluation",
            "time (s)",
            "reached goal flag",
            times,
            values,
            "reached_goal_flag",
        )
        return

    save_line_plot(
        path,
        "Success Evaluation",
        "time (s)",
        "distance to goal (m)",
        times,
        distances,
        "distance_to_goal",
        threshold=success_radius,
        threshold_label=f"success radius {success_radius:.2f} m",
    )


def candidate_snapshot_paths(log_path: Path, explicit_path: str) -> List[Path]:
    paths = []
    if explicit_path:
        paths.append(expand_path(explicit_path))
    paths.extend(
        [
            log_path.with_name(f"{log_path.stem}_scan_snapshots.csv"),
            log_path.parent / "robot_scan_snapshots.csv",
            log_path.parent / "scan_snapshots.csv",
        ]
    )
    unique_paths = []
    seen = set()
    for path in paths:
        if path not in seen:
            unique_paths.append(path)
            seen.add(path)
    return unique_paths


def load_scan_snapshots(snapshot_paths: Sequence[Path]) -> Tuple[List[Dict[str, object]], Optional[Path]]:
    for path in snapshot_paths:
        if not path.exists():
            continue
        snapshots = []
        with path.open("r", encoding="utf-8", newline="") as file_obj:
            for row in csv.DictReader(file_obj):
                ranges_text = row.get("ranges_json", "[]")
                try:
                    ranges_raw = json.loads(ranges_text)
                except json.JSONDecodeError:
                    continue
                ranges = []
                for value in ranges_raw:
                    numeric = as_float(value)
                    ranges.append(numeric if numeric is not None else float("nan"))
                snapshots.append(
                    {
                        "time_sec": as_float(row.get("time_sec")),
                        "angle_min": as_float(row.get("angle_min")),
                        "angle_increment": as_float(row.get("angle_increment")),
                        "range_max": as_float(row.get("range_max")),
                        "ranges": ranges,
                    }
                )
        if snapshots:
            return snapshots, path
    return [], None


def save_lidar_snapshots_plot(
    path: Path,
    snapshots: Sequence[Dict[str, object]],
    times: Sequence[float],
    min_ranges: Sequence[float],
    mean_ranges: Sequence[float],
) -> None:
    if snapshots:
        plt = import_matplotlib()
        indexes = sorted(set([0, len(snapshots) // 2, len(snapshots) - 1]))
        fig, axes = plt.subplots(
            1,
            len(indexes),
            figsize=(5 * len(indexes), 5),
            subplot_kw={"projection": "polar"},
        )
        if len(indexes) == 1:
            axes = [axes]
        for ax, snapshot_index in zip(axes, indexes):
            snapshot = snapshots[snapshot_index]
            ranges = snapshot["ranges"]
            angle_min = snapshot["angle_min"]
            angle_increment = snapshot["angle_increment"]
            if not isinstance(ranges, list) or angle_min is None or angle_increment is None:
                continue
            angles = [angle_min + i * angle_increment for i in range(len(ranges))]
            ax.plot(angles, ranges, linewidth=1.2)
            finite_range_values = finite_values(ranges)
            if finite_range_values:
                ax.set_rmax(min(max(finite_range_values), snapshot.get("range_max") or 8.0))
            snapshot_time = snapshot.get("time_sec")
            title = f"snapshot {snapshot_index + 1}"
            if isinstance(snapshot_time, float) and math.isfinite(snapshot_time):
                title += f"\nt={snapshot_time:.2f}"
            ax.set_title(title)
        fig.suptitle("LiDAR Scan Snapshots")
        fig.tight_layout()
        fig.savefig(path, dpi=160)
        plt.close(fig)
        return

    if has_finite(min_ranges) or has_finite(mean_ranges):
        warn("scan snapshot file not found; plotting LiDAR min/mean summary instead")
        plt = import_matplotlib()
        fig, ax = plt.subplots(figsize=(10, 4.8))
        if has_finite(min_ranges):
            ax.plot(times, min_ranges, label="min_lidar_range")
        if has_finite(mean_ranges):
            ax.plot(times, mean_ranges, label="mean_lidar_range")
        ax.set_title("LiDAR Range Summary")
        ax.set_xlabel("time (s)")
        ax.set_ylabel("range (m)")
        ax.text(
            0.5,
            0.05,
            "Full scan snapshot file was not found; this plot uses CSV summary columns.",
            transform=ax.transAxes,
            ha="center",
        )
        ax.grid(True, alpha=0.35)
        ax.legend()
        fig.tight_layout()
        fig.savefig(path, dpi=160)
        plt.close(fig)
        return

    warn("missing LiDAR data for scan snapshot plot")
    save_missing_plot(path, "LiDAR Scan Snapshots", "Missing /scan data and scan snapshot CSV.")


def save_dashboard(
    path: Path,
    times: Sequence[float],
    xs: Sequence[float],
    ys: Sequence[float],
    goal_x: Optional[float],
    goal_y: Optional[float],
    distances: Sequence[float],
    cmd_linear: Sequence[float],
    cmd_angular: Sequence[float],
    min_lidar: Sequence[float],
    collision_flags: Sequence[Optional[bool]],
    success_radius: float,
    collision_threshold: float,
) -> None:
    plt = import_matplotlib()
    fig, axes = plt.subplots(3, 2, figsize=(13, 11))
    ax_path = axes[0][0]
    xy = [
        (x_value, y_value)
        for x_value, y_value in zip(xs, ys)
        if math.isfinite(x_value) and math.isfinite(y_value)
    ]
    if xy:
        plot_x, plot_y = zip(*xy)
        ax_path.plot(plot_x, plot_y, label="path")
        ax_path.scatter([plot_x[0]], [plot_y[0]], marker="o", label="start")
        ax_path.scatter([plot_x[-1]], [plot_y[-1]], marker="x", label="final")
        if goal_x is not None and goal_y is not None:
            ax_path.scatter([goal_x], [goal_y], marker="*", s=110, label="goal")
        ax_path.axis("equal")
        ax_path.legend()
    else:
        ax_path.text(0.5, 0.5, "missing odometry path", ha="center", va="center")
    ax_path.set_title("Path")
    ax_path.set_xlabel("x (m)")
    ax_path.set_ylabel("y (m)")
    ax_path.grid(True, alpha=0.35)

    axes[0][1].plot(times, distances, label="distance")
    axes[0][1].axhline(success_radius, linestyle="--", color="tab:red", label="success")
    axes[0][1].set_title("Distance to Goal")
    axes[0][1].set_ylabel("m")
    axes[0][1].grid(True, alpha=0.35)
    axes[0][1].legend()

    axes[1][0].plot(times, cmd_linear, label="cmd linear x")
    axes[1][0].set_title("Commanded Linear Velocity")
    axes[1][0].set_ylabel("m/s")
    axes[1][0].grid(True, alpha=0.35)

    axes[1][1].plot(times, cmd_angular, label="cmd angular z", color="tab:orange")
    axes[1][1].set_title("Commanded Angular Velocity")
    axes[1][1].set_ylabel("rad/s")
    axes[1][1].grid(True, alpha=0.35)

    axes[2][0].plot(times, min_lidar, label="min lidar", color="tab:green")
    axes[2][0].axhline(collision_threshold, linestyle="--", color="tab:red", label="danger")
    axes[2][0].set_title("Minimum LiDAR Range")
    axes[2][0].set_xlabel("time (s)")
    axes[2][0].set_ylabel("m")
    axes[2][0].grid(True, alpha=0.35)
    axes[2][0].legend()

    collision_values = [
        1.0 if value else 0.0 if value is False else float("nan") for value in collision_flags
    ]
    axes[2][1].plot(times, collision_values, label="collision flag", color="tab:red")
    axes[2][1].set_title("Collision Flag")
    axes[2][1].set_xlabel("time (s)")
    axes[2][1].set_ylabel("0 or 1")
    axes[2][1].set_ylim(-0.1, 1.1)
    axes[2][1].grid(True, alpha=0.35)

    fig.suptitle("Experiment Dashboard")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def command_nonzero(cmd_linear: Sequence[float], cmd_angular: Sequence[float]) -> bool:
    return any(
        (math.isfinite(vx) and abs(vx) > 1e-4) or (math.isfinite(wz) and abs(wz) > 1e-4)
        for vx, wz in zip(cmd_linear, cmd_angular)
    )


def odom_velocity_nonzero(odom_linear: Sequence[float], odom_angular: Sequence[float]) -> bool:
    return any(
        (math.isfinite(vx) and abs(vx) > 1e-4) or (math.isfinite(wz) and abs(wz) > 1e-4)
        for vx, wz in zip(odom_linear, odom_angular)
    )


def frequency_stats(times: Sequence[float]) -> Dict[str, Optional[float]]:
    dts = [
        times[index] - times[index - 1]
        for index in range(1, len(times))
        if math.isfinite(times[index])
        and math.isfinite(times[index - 1])
        and times[index] > times[index - 1]
    ]
    if not dts:
        return {
            "avg_hz": None,
            "mean_dt": None,
            "median_dt": None,
            "max_dt": None,
            "cv_dt": None,
            "irregular": None,
        }
    mean_dt = sum(dts) / len(dts)
    median_dt = statistics.median(dts)
    stdev_dt = statistics.pstdev(dts) if len(dts) > 1 else 0.0
    cv_dt = stdev_dt / mean_dt if mean_dt > 0.0 else None
    irregular = bool(
        len(dts) > 3
        and (
            (cv_dt is not None and cv_dt > 0.5)
            or (median_dt > 0.0 and max(dts) > max(1.0, 3.0 * median_dt))
        )
    )
    return {
        "avg_hz": 1.0 / mean_dt if mean_dt > 0.0 else None,
        "mean_dt": mean_dt,
        "median_dt": median_dt,
        "max_dt": max(dts),
        "cv_dt": cv_dt,
        "irregular": irregular,
    }


def inspect_ros_graph_snapshot(summary_dir: Path) -> Dict[str, object]:
    snapshot_path = summary_dir / "ros_graph_snapshot.txt"
    result: Dict[str, object] = {
        "path": snapshot_path,
        "exists": snapshot_path.exists(),
        "teleop_keywords": [],
        "route_system_keywords": [],
    }
    if not snapshot_path.exists():
        return result

    text = snapshot_path.read_text(encoding="utf-8", errors="replace").lower()
    teleop_keywords = ["teleop", "keyboard", "joy", "joystick"]
    result["teleop_keywords"] = [keyword for keyword in teleop_keywords if keyword in text]
    result["route_system_keywords"] = []
    return result


def build_summary(
    log_path: Path,
    summary_dir: Path,
    fieldnames: Sequence[str],
    rows: Sequence[Dict[str, str]],
    raw_times: Sequence[float],
    times: Sequence[float],
    steps: Sequence[float],
    xs: Sequence[float],
    ys: Sequence[float],
    distances: Sequence[float],
    cmd_linear: Sequence[float],
    cmd_angular: Sequence[float],
    odom_linear: Sequence[float],
    odom_angular: Sequence[float],
    min_lidar: Sequence[float],
    collision_flags: Sequence[Optional[bool]],
    reached_flags: Sequence[Optional[bool]],
    control_sources: Sequence[str],
    goal_x: Optional[float],
    goal_y: Optional[float],
    success_radius: float,
    collision_threshold: float,
    snapshots_found: bool,
    expected_control_source: str,
    route_system_expected: bool,
) -> Tuple[Dict[str, object], List[str], Dict[str, object]]:
    start_x, final_x = first_last(xs)
    start_y, final_y = first_last(ys)
    final_distance = finite_values(distances)[-1] if has_finite(distances) else None
    minimum_distance = min_or_none(distances)
    total_path_length = path_length(xs, ys)
    total_duration = times[-1] - times[0] if len(times) >= 2 else 0.0
    raw_start = finite_values(raw_times)[0] if has_finite(raw_times) else None
    raw_end = finite_values(raw_times)[-1] if has_finite(raw_times) else None
    freq = frequency_stats(times)

    collision_count = sum(1 for value in collision_flags if value is True)
    cmd_has_nonzero = command_nonzero(cmd_linear, cmd_angular)
    odom_has_nonzero_velocity = odom_velocity_nonzero(odom_linear, odom_angular)
    odom_changed = total_path_length >= 0.03
    lidar_received = has_finite(min_lidar) or snapshots_found

    reached_by_flag = any(value is True for value in reached_flags)
    reached_by_distance = minimum_distance is not None and minimum_distance <= success_radius
    success = bool(reached_by_flag or reached_by_distance)

    clean_sources = sorted({source for source in control_sources if source})
    expected_control_source = expected_control_source.strip()
    control_source_matches_expected = bool(clean_sources) and all(
        source == expected_control_source for source in clean_sources
    )

    missing_columns = [
        canonical
        for canonical in IMPORTANT_COLUMNS
        if find_column(fieldnames, canonical) is None
    ]
    optional_missing = [
        canonical
        for canonical in [
            "goal_x",
            "goal_y",
            "distance_to_goal",
            "angle_to_goal",
            "min_lidar_range",
            "mean_lidar_range",
            "collision_flag",
            "reached_goal_flag",
            "control_source",
        ]
        if find_column(fieldnames, canonical) is None
    ]

    warnings = []
    if missing_columns:
        warnings.append("Important data columns are missing: " + ", ".join(missing_columns))
    if optional_missing:
        warnings.append("Optional validation columns are missing: " + ", ".join(optional_missing))
    if total_path_length < 0.05:
        warnings.append("Robot did not move enough; odometry path length is below 0.05 m.")
    if not cmd_has_nonzero:
        warnings.append("/cmd_vel was all zeros or unavailable.")
    if not odom_changed:
        warnings.append("Odometry position stayed almost constant.")
    if freq["irregular"]:
        warnings.append("Control frequency is irregular; inspect dropped messages or pauses.")
    minimum_lidar = min_or_none(min_lidar)
    if minimum_lidar is not None and minimum_lidar <= collision_threshold:
        warnings.append(
            f"Minimum LiDAR range reached {minimum_lidar:.3f} m, below/equal "
            f"the collision danger threshold {collision_threshold:.3f} m."
        )
    if collision_count > 0:
        warnings.append(f"Collision flag was true for {collision_count} logged steps.")
    if success and total_path_length < 0.05:
        warnings.append("Goal appears reached without meaningful movement; verify start/goal setup.")
    if not lidar_received:
        warnings.append("No LiDAR data was available in the log or scan snapshot file.")
    if clean_sources and not control_source_matches_expected:
        warnings.append(
            f"control_source is not exclusively {expected_control_source}: "
            + ", ".join(clean_sources)
        )
    if not clean_sources:
        warnings.append("control_source column is missing or empty.")

    ros_graph = inspect_ros_graph_snapshot(summary_dir)
    if not ros_graph["exists"]:
        warnings.append("ROS graph snapshot was not found; record it for stronger autonomy evidence.")
    if ros_graph.get("teleop_keywords"):
        warnings.append(
            "ROS graph snapshot contains possible teleop keywords: "
            + ", ".join(ros_graph["teleop_keywords"])
        )
    if ros_graph.get("route_system_keywords") and not route_system_expected:
        warnings.append(
            "ROS graph snapshot contains route-system dependency keywords: "
            + ", ".join(ros_graph["route_system_keywords"])
        )

    nonzero_cmd_and_motion = bool(cmd_has_nonzero and (odom_changed or odom_has_nonzero_velocity))
    summary = {
        "log_file_used": str(log_path),
        "test_start_time": timestamp_text(raw_start),
        "test_end_time": timestamp_text(raw_end),
        "total_duration_sec": total_duration,
        "total_number_of_steps": len(rows),
        "average_control_frequency_hz": freq["avg_hz"],
        "start_position": fmt_pair(start_x, start_y),
        "goal_position": fmt_pair(goal_x, goal_y),
        "final_position": fmt_pair(final_x, final_y),
        "final_distance_to_goal": final_distance,
        "minimum_distance_to_goal": minimum_distance,
        "path_length_from_odometry": total_path_length,
        "average_commanded_linear_velocity": mean_or_none(cmd_linear),
        "average_commanded_angular_velocity_abs": mean_or_none(abs(v) for v in cmd_angular),
        "maximum_commanded_linear_velocity_abs": max_abs_or_none(cmd_linear),
        "maximum_commanded_angular_velocity_abs": max_abs_or_none(cmd_angular),
        "minimum_lidar_range": minimum_lidar,
        "number_of_collision_flags": collision_count,
        "success_or_failure": "SUCCESS" if success else "FAILURE",
        "success_radius": success_radius,
        "cmd_vel_had_nonzero_commands": cmd_has_nonzero,
        "odom_changed_consistently_with_commands": nonzero_cmd_and_motion,
        "lidar_data_was_received": lidar_received,
        "expected_control_source": expected_control_source,
        "control_source_matches_expected": control_source_matches_expected,
        "route_system_expected": route_system_expected,
        "control_sources_seen": ", ".join(clean_sources) if clean_sources else "unavailable",
        "ros_graph_snapshot": str(ros_graph["path"]) if ros_graph["exists"] else "missing",
    }
    return summary, warnings, ros_graph


def write_summary_files(
    summary_dir: Path,
    summary: Dict[str, object],
    warnings: Sequence[str],
    ros_graph: Dict[str, object],
) -> None:
    summary_dir.mkdir(parents=True, exist_ok=True)
    txt_path = summary_dir / "experiment_validation_summary.txt"
    csv_path = summary_dir / "experiment_validation_summary.csv"

    lines = [
        "Experiment Validation Summary",
        "",
        f"Log file used: {summary['log_file_used']}",
        f"Test start time: {summary['test_start_time']}",
        f"Test end time: {summary['test_end_time']}",
        f"Total duration: {fmt(summary['total_duration_sec'])} sec",
        f"Total number of steps: {summary['total_number_of_steps']}",
        f"Average control frequency: {fmt(summary['average_control_frequency_hz'])} Hz",
        f"Start position: {summary['start_position']}",
        f"Goal position: {summary['goal_position']}",
        f"Final position: {summary['final_position']}",
        f"Final distance to goal: {fmt(summary['final_distance_to_goal'])} m",
        f"Minimum distance to goal: {fmt(summary['minimum_distance_to_goal'])} m",
        f"Path length from odometry: {fmt(summary['path_length_from_odometry'])} m",
        (
            "Average commanded linear velocity: "
            f"{fmt(summary['average_commanded_linear_velocity'])} m/s"
        ),
        (
            "Average commanded angular velocity magnitude: "
            f"{fmt(summary['average_commanded_angular_velocity_abs'])} rad/s"
        ),
        (
            "Maximum commanded linear velocity magnitude: "
            f"{fmt(summary['maximum_commanded_linear_velocity_abs'])} m/s"
        ),
        (
            "Maximum commanded angular velocity magnitude: "
            f"{fmt(summary['maximum_commanded_angular_velocity_abs'])} rad/s"
        ),
        f"Minimum LiDAR range during test: {fmt(summary['minimum_lidar_range'])} m",
        f"Number of collision flags: {summary['number_of_collision_flags']}",
        f"Result using success radius: {summary['success_or_failure']}",
        f"/cmd_vel had nonzero commands: {fmt(summary['cmd_vel_had_nonzero_commands'])}",
        (
            "Odometry changed consistently with commands: "
            f"{fmt(summary['odom_changed_consistently_with_commands'])}"
        ),
        f"LiDAR data was received: {fmt(summary['lidar_data_was_received'])}",
        (
            f"control_source is {summary['expected_control_source']}: "
            f"{fmt(summary['control_source_matches_expected'])}"
        ),
        f"Control sources seen: {summary['control_sources_seen']}",
        "",
        "Warnings",
    ]
    if warnings:
        lines.extend(f"- {message}" for message in warnings)
    else:
        lines.append("- none")

    lines.extend(
        [
            "",
            "Autonomy evidence",
            "- The log was generated by subscribing to /cmd_vel, /odom, and /scan.",
            "- Commands were generated during the autonomous test if /cmd_vel nonzero is yes above.",
            (
                f"- The control_source column is {summary['expected_control_source']} "
                "if that check is yes above."
            ),
            "- No teleop evidence is present in this log.",
        ]
    )
    if summary.get("route_system_expected"):
        lines.append(
            "- Autonomous route-generation evidence is expected in this V2 test."
        )
    else:
        lines.append("- No route-generation dependency evidence is present in this log.")
    if ros_graph["exists"]:
        lines.append(f"- ROS graph snapshot was available: {ros_graph['path']}")
        route_system_ok = bool(summary.get("route_system_expected")) or not ros_graph.get("route_system_keywords")
        if not ros_graph.get("teleop_keywords") and route_system_ok:
            if summary.get("route_system_expected") and ros_graph.get("route_system_keywords"):
                lines.append("- Route-generation dependency keywords were found and are expected for this V2 test.")
            else:
                lines.append("- No teleop or route-generation dependency keywords were found in the ROS graph snapshot.")
        else:
            lines.append("- Review ROS graph warnings above before claiming autonomy-only operation.")
    else:
        lines.append("- ROS graph snapshot was not found next to this summary.")

    lines.extend(
        [
            "",
            (
                "This log supports autonomous operation, but it does not mathematically prove "
                "that no other node was active unless the ROS node list and launch "
                "configuration are also recorded."
            ),
        ]
    )
    txt_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    with csv_path.open("w", encoding="utf-8", newline="") as file_obj:
        writer = csv.writer(file_obj)
        writer.writerow(["metric", "value"])
        for key, value in summary.items():
            writer.writerow([key, value])
        writer.writerow(["warnings", " | ".join(warnings) if warnings else "none"])


def generate_plots(
    out_dir: Path,
    times: Sequence[float],
    steps: Sequence[float],
    xs: Sequence[float],
    ys: Sequence[float],
    yaws: Sequence[float],
    odom_linear: Sequence[float],
    odom_angular: Sequence[float],
    cmd_linear: Sequence[float],
    cmd_angular: Sequence[float],
    goal_x: Optional[float],
    goal_y: Optional[float],
    distances: Sequence[float],
    min_lidar: Sequence[float],
    mean_lidar: Sequence[float],
    collision_flags: Sequence[Optional[bool]],
    reached_flags: Sequence[Optional[bool]],
    snapshots: Sequence[Dict[str, object]],
    success_radius: float,
    collision_threshold: float,
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)

    save_path_plot(out_dir / "01_robot_path.png", xs, ys, goal_x, goal_y, success_radius)
    save_line_plot(
        out_dir / "02_distance_to_goal_vs_time.png",
        "Distance to Goal vs Time",
        "time (s)",
        "distance to goal (m)",
        times,
        distances,
        "distance_to_goal",
        threshold=success_radius,
        threshold_label=f"success radius {success_radius:.2f} m",
    )
    save_line_plot(
        out_dir / "03_cmd_linear_velocity_vs_time.png",
        "Commanded Linear Velocity vs Time",
        "time (s)",
        "cmd linear x (m/s)",
        times,
        cmd_linear,
        "cmd_linear_x",
    )
    save_line_plot(
        out_dir / "04_cmd_angular_velocity_vs_time.png",
        "Commanded Angular Velocity vs Time",
        "time (s)",
        "cmd angular z (rad/s)",
        times,
        cmd_angular,
        "cmd_angular_z",
    )
    save_line_plot(
        out_dir / "05_odom_linear_velocity_vs_time.png",
        "Odometry Linear Velocity vs Time",
        "time (s)",
        "odom linear x (m/s)",
        times,
        odom_linear,
        "odom_linear_x",
    )
    save_line_plot(
        out_dir / "06_odom_angular_velocity_vs_time.png",
        "Odometry Angular Velocity vs Time",
        "time (s)",
        "odom angular z (rad/s)",
        times,
        odom_angular,
        "odom_angular_z",
    )
    save_cmd_vs_odom_plot(
        out_dir / "07_cmd_vs_odom_velocity.png",
        times,
        cmd_linear,
        odom_linear,
        cmd_angular,
        odom_angular,
    )
    save_line_plot(
        out_dir / "08_yaw_vs_time.png",
        "Yaw Angle vs Time",
        "time (s)",
        "yaw (rad)",
        times,
        yaws,
        "yaw",
    )
    save_line_plot(
        out_dir / "09_min_lidar_range_vs_time.png",
        "Minimum LiDAR Range vs Time",
        "time (s)",
        "minimum range (m)",
        times,
        min_lidar,
        "min_lidar_range",
        threshold=collision_threshold,
        threshold_label=f"danger threshold {collision_threshold:.2f} m",
    )
    save_lidar_snapshots_plot(
        out_dir / "10_lidar_scan_snapshots.png",
        snapshots,
        times,
        min_lidar,
        mean_lidar,
    )
    save_collision_plot(out_dir / "11_collision_flag_vs_time.png", times, collision_flags)
    save_line_plot(
        out_dir / "12_step_number_vs_time.png",
        "Step Number vs Time",
        "time (s)",
        "step",
        times,
        steps,
        "step",
    )
    save_timeline_plot(out_dir / "13_control_command_timeline.png", steps, cmd_linear, cmd_angular)
    save_success_plot(
        out_dir / "14_success_evaluation.png",
        times,
        reached_flags,
        distances,
        success_radius,
    )
    save_dashboard(
        out_dir / "15_experiment_dashboard.png",
        times,
        xs,
        ys,
        goal_x,
        goal_y,
        distances,
        cmd_linear,
        cmd_angular,
        min_lidar,
        collision_flags,
        success_radius,
        collision_threshold,
    )


def main() -> None:
    args = parse_args()
    log_path = expand_path(args.log)
    out_dir = expand_path(args.out)
    summary_dir = expand_path(args.summary_dir) if args.summary_dir else log_path.parent

    if not log_path.exists():
        raise FileNotFoundError(f"Missing log file: {log_path}")

    rows, fieldnames = load_rows(log_path)
    row_count = len(rows)
    raw_times = float_series(rows, fieldnames, "time_sec")
    times = normalize_times(raw_times, row_count)
    steps = float_series(rows, fieldnames, "step")
    if not has_finite(steps):
        steps = [float(index + 1) for index in range(row_count)]

    xs = float_series(rows, fieldnames, "odom_x")
    ys = float_series(rows, fieldnames, "odom_y")
    yaws = float_series(rows, fieldnames, "yaw")
    odom_linear = float_series(rows, fieldnames, "odom_linear_x")
    odom_angular = float_series(rows, fieldnames, "odom_angular_z")
    cmd_linear = float_series(rows, fieldnames, "cmd_linear_x")
    cmd_angular = float_series(rows, fieldnames, "cmd_angular_z")
    goal_xs = fill_goal_series(float_series(rows, fieldnames, "goal_x"), args.goal_x)
    goal_ys = fill_goal_series(float_series(rows, fieldnames, "goal_y"), args.goal_y)
    distances = float_series(rows, fieldnames, "distance_to_goal")
    angles = float_series(rows, fieldnames, "angle_to_goal")
    distances, angles = compute_distance_and_angle(xs, ys, yaws, goal_xs, goal_ys, distances, angles)
    min_lidar = float_series(rows, fieldnames, "min_lidar_range")
    mean_lidar = float_series(rows, fieldnames, "mean_lidar_range")

    collision_flags = bool_series(rows, fieldnames, "collision_flag")
    if not any(value is not None for value in collision_flags) and has_finite(min_lidar):
        collision_flags = [
            bool(value <= args.collision_threshold) if math.isfinite(value) else None
            for value in min_lidar
        ]

    reached_flags = bool_series(rows, fieldnames, "reached_goal_flag")
    if not any(value is not None for value in reached_flags) and has_finite(distances):
        reached_flags = [
            bool(value <= args.success_radius) if math.isfinite(value) else None
            for value in distances
        ]

    control_sources = string_series(rows, fieldnames, "control_source")
    goal_x = finite_values(goal_xs)[-1] if has_finite(goal_xs) else None
    goal_y = finite_values(goal_ys)[-1] if has_finite(goal_ys) else None

    snapshot_paths = candidate_snapshot_paths(log_path, args.scan_snapshots)
    snapshots, snapshot_path = load_scan_snapshots(snapshot_paths)
    if snapshot_path:
        print(f"Loaded scan snapshots from: {snapshot_path}")

    summary, warnings, ros_graph = build_summary(
        log_path=log_path,
        summary_dir=summary_dir,
        fieldnames=fieldnames,
        rows=rows,
        raw_times=raw_times,
        times=times,
        steps=steps,
        xs=xs,
        ys=ys,
        distances=distances,
        cmd_linear=cmd_linear,
        cmd_angular=cmd_angular,
        odom_linear=odom_linear,
        odom_angular=odom_angular,
        min_lidar=min_lidar,
        collision_flags=collision_flags,
        reached_flags=reached_flags,
        control_sources=control_sources,
        goal_x=goal_x,
        goal_y=goal_y,
        success_radius=args.success_radius,
        collision_threshold=args.collision_threshold,
        snapshots_found=bool(snapshots),
        expected_control_source=args.expected_control_source,
        route_system_expected=bool(args.route_system_expected),
    )
    write_summary_files(summary_dir, summary, warnings, ros_graph)

    generate_plots(
        out_dir=out_dir,
        times=times,
        steps=steps,
        xs=xs,
        ys=ys,
        yaws=yaws,
        odom_linear=odom_linear,
        odom_angular=odom_angular,
        cmd_linear=cmd_linear,
        cmd_angular=cmd_angular,
        goal_x=goal_x,
        goal_y=goal_y,
        distances=distances,
        min_lidar=min_lidar,
        mean_lidar=mean_lidar,
        collision_flags=collision_flags,
        reached_flags=reached_flags,
        snapshots=snapshots,
        success_radius=args.success_radius,
        collision_threshold=args.collision_threshold,
    )

    print(f"Summary written to: {summary_dir / 'experiment_validation_summary.txt'}")
    print(f"Summary CSV written to: {summary_dir / 'experiment_validation_summary.csv'}")
    print(f"Plots written to: {out_dir}")


if __name__ == "__main__":
    main()
