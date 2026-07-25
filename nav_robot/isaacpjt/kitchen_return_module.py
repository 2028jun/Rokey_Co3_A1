"""Route construction for an on-demand return to the kitchen."""

from __future__ import annotations

import math


KITCHEN_DOCK = (0.0, 5.25, -math.pi / 2.0)
BACKOUT_SPEED = 0.28
AISLE_SPEED = 0.50


def build_kitchen_route(x: float, y: float):
    """Return stages that leave a table bay and finish at the kitchen dock."""
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
    if abs(y - KITCHEN_DOCK[1]) > 0.03:
        stages.extend(
            [
                {"kind": "pivot", "yaw": math.pi / 2.0},
                {
                    "kind": "axis_y",
                    "value": KITCHEN_DOCK[1],
                    "speed": AISLE_SPEED,
                    "yaw": math.pi / 2.0,
                },
            ]
        )
    stages.append({"kind": "pivot", "yaw": KITCHEN_DOCK[2]})
    return stages
