"""Route construction for an on-demand return to the kitchen."""

from __future__ import annotations

import math


# Match nav_restaurant_demo SPAWN_POSITION default (1 m left of aisle).
KITCHEN_DOCK = (-1.0, 5.25, -math.pi / 2.0)


def build_kitchen_route(x: float, y: float):
    """Return stages that leave a table bay and finish at the kitchen dock."""
    dock_x, dock_y, dock_yaw = KITCHEN_DOCK
    stages = []
    route_x = float(x)

    if abs(x) > 0.10:
        outward_yaw = 0.0 if x > 0.0 else math.pi
        stages.extend(
            [
                {"kind": "pivot", "yaw": outward_yaw},
                {
                    "kind": "axis_x",
                    "value": 0.0,
                    "speed": -0.22,
                    "yaw": outward_yaw,
                },
            ]
        )
        route_x = 0.0

    if abs(y - dock_y) > 0.03:
        stages.extend(
            [
                {"kind": "pivot", "yaw": math.pi / 2.0},
                {
                    "kind": "axis_y",
                    "value": dock_y,
                    "speed": 0.35,
                    "yaw": math.pi / 2.0,
                },
            ]
        )

    if abs(route_x - dock_x) > 0.03:
        side_yaw = 0.0 if dock_x > route_x else math.pi
        stages.extend(
            [
                {"kind": "pivot", "yaw": side_yaw},
                {
                    "kind": "axis_x",
                    "value": dock_x,
                    "speed": 0.22,
                    "yaw": side_yaw,
                },
            ]
        )

    stages.append({"kind": "pivot", "yaw": dock_yaw})
    return stages
