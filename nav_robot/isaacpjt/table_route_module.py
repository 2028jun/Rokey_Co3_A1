"""Route construction for on-demand restaurant table moves."""

from __future__ import annotations

import math


TABLE_DOCKS = {
    0: (-1.82, -2.20, math.pi),
    1: (1.82, -2.20, 0.0),
    2: (-1.82, 0.70, math.pi),
    3: (1.82, 0.70, 0.0),
}


def build_table_route(table_id: int, x: float, y: float):
    """Return axis-aligned stages from the current pose to a table dock."""
    goal_x, goal_y, goal_yaw = TABLE_DOCKS[table_id]
    stages = []

    # If a command is issued while docked, back out to the centre aisle first.
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

    aisle_yaw = -math.pi / 2.0 if goal_y < y else math.pi / 2.0
    stages.extend(
        [
            {"kind": "pivot", "yaw": aisle_yaw},
            {
                "kind": "axis_y",
                "value": goal_y,
                "speed": 0.35,
                "yaw": aisle_yaw,
            },
            {"kind": "pivot", "yaw": goal_yaw},
            {
                "kind": "axis_x",
                "value": goal_x,
                "speed": 0.22,
                "yaw": goal_yaw,
            },
        ]
    )
    return stages
