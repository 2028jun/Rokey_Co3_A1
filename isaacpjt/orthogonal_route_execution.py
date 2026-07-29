"""Convert validated ROS orthogonal route points into Isaac drive stages."""

from __future__ import annotations

import math
from typing import Any


def parse_route_points(raw_points: Any) -> list[tuple[float, float]]:
    """Validate mission points and remove adjacent duplicates."""
    if not isinstance(raw_points, list):
        raise ValueError("route points must be a list")

    points: list[tuple[float, float]] = []
    for index, raw in enumerate(raw_points):
        if not isinstance(raw, dict):
            raise ValueError(f"route point {index} must be an object")
        try:
            x = float(raw["x"])
            y = float(raw["y"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"invalid route point {index}: {raw!r}") from exc
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError(f"non-finite route point {index}: {raw!r}")
        point = (x, y)
        if not points or math.hypot(
            point[0] - points[-1][0], point[1] - points[-1][1]
        ) > 0.02:
            points.append(point)

    if len(points) < 2:
        raise ValueError("route requires at least two distinct points")
    return points


def build_axis_stages(
    points: list[tuple[float, float]],
    *,
    speed_mps: float = 0.50,
    axis_epsilon_m: float = 0.03,
) -> list[dict[str, float | str | bool]]:
    """Build forward-only pivot/translation stages from an axis polyline."""
    if len(points) < 2:
        raise ValueError("route requires at least two points")
    speed = abs(float(speed_mps))
    if not math.isfinite(speed) or speed <= 0.0:
        raise ValueError("route speed must be positive and finite")

    stages: list[dict[str, float | str | bool]] = []
    for index, (start, end) in enumerate(zip(points, points[1:])):
        dx = float(end[0] - start[0])
        dy = float(end[1] - start[1])
        x_motion = abs(dx) > axis_epsilon_m
        y_motion = abs(dy) > axis_epsilon_m
        if x_motion and y_motion:
            raise ValueError(
                f"route segment {index} is diagonal: {start!r} -> {end!r}"
            )
        if not x_motion and not y_motion:
            continue
        if x_motion:
            kind = "axis_x"
            value = float(end[0])
            yaw = 0.0 if dx > 0.0 else math.pi
            cross_axis = "y"
            cross_value = float(end[1])
        else:
            kind = "axis_y"
            value = float(end[1])
            yaw = math.pi / 2.0 if dy > 0.0 else -math.pi / 2.0
            cross_axis = "x"
            cross_value = float(end[0])
        if not stages or abs(float(stages[-1]["yaw"]) - yaw) > 1e-9:
            stages.append({"kind": "pivot", "yaw": yaw})
        stages.append(
            {
                "kind": kind,
                "value": value,
                "speed": speed,
                "yaw": yaw,
                "cross_axis": cross_axis,
                "cross_value": cross_value,
                "planned_route": True,
            }
        )
    if not stages:
        raise ValueError("route contains no executable segment")
    return stages
