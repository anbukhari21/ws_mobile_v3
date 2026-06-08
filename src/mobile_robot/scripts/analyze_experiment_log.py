#!/usr/bin/env python3
"""Generate plots and a report for one experiment logger trial folder."""

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Dict, List, Optional


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze one robot experiment log directory.")
    parser.add_argument("--log-dir", required=True, help="Trial folder containing step_log.csv")
    parser.add_argument("--arena-radius", type=float, default=2.0)
    parser.add_argument("--success-distance-threshold", type=float, default=0.15)
    parser.add_argument("--target-x", type=float, default=float("nan"))
    parser.add_argument("--target-y", type=float, default=float("nan"))
    return parser.parse_args()


def as_float(value: object) -> Optional[float]:
    if value in (None, "", "nan"):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def load_rows(csv_path: Path) -> List[Dict[str, str]]:
    with csv_path.open("r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def values(rows: List[Dict[str, str]], key: str) -> List[float]:
    result: List[float] = []
    for row in rows:
        value = as_float(row.get(key))
        result.append(value if value is not None else float("nan"))
    return result


def finite_pairs(xs: List[float], ys: List[float]) -> List[tuple[float, float]]:
    return [(x, y) for x, y in zip(xs, ys) if math.isfinite(x) and math.isfinite(y)]


def save_summary_if_needed(log_dir: Path, rows: List[Dict[str, str]], args: argparse.Namespace) -> Dict:
    summary_path = log_dir / "summary.json"
    if summary_path.exists():
        with summary_path.open("r", encoding="utf-8") as f:
            return json.load(f)

    goal_distances = [v for v in values(rows, "goal_distance") if math.isfinite(v)]
    final_goal_distance = goal_distances[-1] if goal_distances else None
    min_goal_distance = min(goal_distances) if goal_distances else None
    success = None
    if goal_distances:
        success = bool(
            final_goal_distance <= args.success_distance_threshold
            or min_goal_distance <= args.success_distance_threshold
        )

    summary = {
        "trial_name": log_dir.name,
        "total_steps": len(rows),
        "duration": as_float(rows[-1].get("elapsed_time")) if rows else 0.0,
        "final_goal_distance": final_goal_distance,
        "minimum_goal_distance_reached": min_goal_distance,
        "success": success,
        "success_rule_used": (
            f"final or minimum /goal_distance <= {args.success_distance_threshold:.3f} m"
            if goal_distances
            else "unknown because /goal_distance was not received"
        ),
    }
    with summary_path.open("w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, sort_keys=True)
    return summary


def make_plots(log_dir: Path, rows: List[Dict[str, str]], summary: Dict, args: argparse.Namespace) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    times = values(rows, "elapsed_time")
    xs = values(rows, "odom_x")
    ys = values(rows, "odom_y")
    xy = finite_pairs(xs, ys)

    plt.figure(figsize=(7, 7))
    if xy:
        plot_x, plot_y = zip(*xy)
        plt.plot(plot_x, plot_y, linewidth=2, label="odom trajectory")
        plt.scatter([plot_x[0]], [plot_y[0]], marker="o", s=60, label="start")
        plt.scatter([plot_x[-1]], [plot_y[-1]], marker="x", s=80, label="end")
    if math.isfinite(args.target_x) and math.isfinite(args.target_y):
        plt.scatter([args.target_x], [args.target_y], marker="*", s=120, label="target")
    arena = plt.Circle((0.0, 0.0), args.arena_radius, fill=False, linestyle="--", label="arena")
    plt.gca().add_patch(arena)
    plt.axis("equal")
    plt.xlabel("odom x (m)")
    plt.ylabel("odom y (m)")
    plt.title("Robot Trajectory")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(log_dir / "trajectory.png", dpi=160)
    plt.close()

    plt.figure(figsize=(10, 7))
    ax1 = plt.subplot(2, 1, 1)
    ax1.plot(times, values(rows, "odom_linear_x"), label="odom linear x")
    ax1.plot(times, values(rows, "cmd_linear_x"), label="cmd linear x")
    ax1.set_ylabel("linear velocity (m/s)")
    ax1.grid(True)
    ax1.legend()
    ax2 = plt.subplot(2, 1, 2, sharex=ax1)
    ax2.plot(times, values(rows, "odom_angular_z"), label="odom angular z")
    ax2.plot(times, values(rows, "cmd_angular_z"), label="cmd angular z")
    ax2.set_xlabel("elapsed time (s)")
    ax2.set_ylabel("angular velocity (rad/s)")
    ax2.grid(True)
    ax2.legend()
    plt.tight_layout()
    plt.savefig(log_dir / "velocity_plot.png", dpi=160)
    plt.close()

    goal_dist = values(rows, "goal_distance")
    if any(math.isfinite(v) for v in goal_dist):
        plt.figure(figsize=(10, 4))
        plt.plot(times, goal_dist, label="goal distance")
        plt.axhline(args.success_distance_threshold, linestyle="--", label="success threshold")
        plt.xlabel("elapsed time (s)")
        plt.ylabel("distance (m)")
        plt.title("Goal Distance")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(log_dir / "goal_distance_plot.png", dpi=160)
        plt.close()

    lidar_min = values(rows, "min_lidar_range")
    if any(math.isfinite(v) for v in lidar_min):
        plt.figure(figsize=(10, 4))
        plt.plot(times, lidar_min, label="minimum LiDAR range")
        plt.xlabel("elapsed time (s)")
        plt.ylabel("range (m)")
        plt.title("Minimum LiDAR Range")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(log_dir / "lidar_min_range_plot.png", dpi=160)
        plt.close()

    report = [
        "# Trial Report",
        "",
        f"- Log directory: `{log_dir}`",
        f"- Total steps: {summary.get('total_steps')}",
        f"- Duration: {summary.get('duration')}",
        f"- Success: {summary.get('success')}",
        f"- Final goal distance: {summary.get('final_goal_distance')}",
        f"- Minimum goal distance: {summary.get('minimum_goal_distance_reached')}",
        f"- Success rule: {summary.get('success_rule_used')}",
        "",
        "## Images",
        "",
        "- `trajectory.png`",
        "- `velocity_plot.png`",
        "- `goal_distance_plot.png` if goal data exists",
        "- `lidar_min_range_plot.png` if scan data exists",
    ]
    (log_dir / "trial_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")


def main() -> None:
    args = parse_args()
    log_dir = Path(args.log_dir).expanduser().resolve()
    csv_path = log_dir / "step_log.csv"
    if not csv_path.exists():
        raise FileNotFoundError(f"Missing step log: {csv_path}")

    rows = load_rows(csv_path)
    summary = save_summary_if_needed(log_dir, rows, args)
    make_plots(log_dir, rows, summary, args)
    print(f"Analysis complete: {log_dir}")


if __name__ == "__main__":
    main()
