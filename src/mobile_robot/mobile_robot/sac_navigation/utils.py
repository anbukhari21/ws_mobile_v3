#!/usr/bin/env python3
"""Shared helpers for the route + SAC bridge nodes."""

from __future__ import annotations

import math
from typing import Iterable

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(0.5 * yaw)
    q.w = math.cos(0.5 * yaw)
    return q


def quaternion_to_yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def path_to_points(path: Path) -> list[tuple[float, float]]:
    return [
        (float(pose.pose.position.x), float(pose.pose.position.y))
        for pose in path.poses
    ]


def points_to_path(
    points: Iterable[tuple[float, float]],
    *,
    frame_id: str,
    stamp,
) -> Path:
    msg = Path()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    previous: tuple[float, float] | None = None
    points_list = list(points)
    for index, (x, y) in enumerate(points_list):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = stamp
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        if index + 1 < len(points_list):
            nx, ny = points_list[index + 1]
            yaw = math.atan2(ny - y, nx - x)
        elif previous is not None:
            yaw = math.atan2(y - previous[1], x - previous[0])
        else:
            yaw = 0.0
        pose.pose.orientation = yaw_to_quaternion(yaw)
        msg.poses.append(pose)
        previous = (x, y)
    return msg

