"""Route construction for on-demand restaurant table moves."""

from __future__ import annotations

import math


BACKOUT_SPEED = 0.28
AISLE_SPEED = 0.50
TABLE_APPROACH_SPEED = 0.28


TABLE_DOCKS = {
    0: (-1.82, -2.20, math.pi),
    1: (1.82, -2.20, 0.0),
    2: (-1.82, 0.70, math.pi),
    3: (1.82, 0.70, 0.0),
}


def build_table_route(
    table_id: int,
    x: float,
    y: float,
    table_dock=None,
    aisle_x: float = 0.0,
):
    """Return axis-aligned stages from the current pose to a table dock.

    aisle_x shifts the shared corridor so robot1/robot2 can pass instead of
    meeting head-on on x=0.
    """
    goal_x, goal_y, goal_yaw = table_dock or TABLE_DOCKS[table_id]
    aisle = float(aisle_x)
    stages = []

    # If a command is issued while docked, back out to this robot's aisle lane.
    if abs(x - aisle) > 0.10:
        outward_yaw = 0.0 if x > aisle else math.pi
        stages.extend(
            [
                {"kind": "pivot", "yaw": outward_yaw},
                {
                    "kind": "axis_x",
                    "value": aisle,
                    "speed": -BACKOUT_SPEED,
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
                "speed": AISLE_SPEED,
                "yaw": aisle_yaw,
            },
            {"kind": "pivot", "yaw": goal_yaw},
            {
                "kind": "axis_x",
                "value": goal_x,
                "speed": TABLE_APPROACH_SPEED,
                "yaw": goal_yaw,
            },
        ]
    )
    return stages
