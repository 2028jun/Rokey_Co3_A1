"""Route construction for an on-demand return to the kitchen."""

from __future__ import annotations

import math


KITCHEN_DOCK = (0.0, 5.25, -math.pi / 2.0)
BACKOUT_SPEED = 0.28
AISLE_SPEED = 0.50


def build_kitchen_route(
    x: float,
    y: float,
    kitchen_dock=KITCHEN_DOCK,
):
    """Return stages that leave a table bay and finish at the kitchen dock."""
    dock_x, dock_y, dock_yaw = kitchen_dock
    stages = []
    if abs(x) > 0.10:
        outward_yaw = 0.0 if x > 0.0 else math.pi
        stages.extend(
            [
                {"kind": "pivot", "yaw": outward_yaw},
                {
                    "kind": "axis_x",
                    "value": 0.0,
                    "speed": -BACKOUT_SPEED,
                    "yaw": outward_yaw,
                },
            ]
        )
    if abs(y - dock_y) > 0.03:
        stages.extend(
            [
                {"kind": "pivot", "yaw": math.pi / 2.0},
                {
                    "kind": "axis_y",
                    "value": dock_y,
                    "speed": AISLE_SPEED,
                    "yaw": math.pi / 2.0,
                },
            ]
        )
    if abs(dock_x) > 0.03:
        slot_yaw = 0.0 if dock_x > 0.0 else math.pi
        stages.extend(
            [
                {"kind": "pivot", "yaw": slot_yaw},
                {
                    "kind": "axis_x",
                    "value": dock_x,
                    "speed": BACKOUT_SPEED,
                    "yaw": slot_yaw,
                },
            ]
        )
    stages.append({"kind": "pivot", "yaw": dock_yaw})
    return stages
