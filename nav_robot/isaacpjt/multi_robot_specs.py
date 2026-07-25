"""Spawn slots for one or two identical nav_robot serving robots.

Uses only nav_robot ``two_wheel_serving_robot_v2.usd``. No nav_robot5 assets.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class RobotSpec:
    robot_id: str
    root: str
    spawn_x: float
    spawn_y: float
    spawn_z: float
    spawn_yaw: float


# Same kitchen row as the widened doorway (|x|<2.0). robot1 keeps the
# previously chosen left wait pose; robot2 mirrors on the right.
ROBOT_SPECS: tuple[RobotSpec, ...] = (
    RobotSpec("robot1", "/World/robot1", -1.0, 5.25, 0.01, -math.pi / 2.0),
    RobotSpec("robot2", "/World/robot2", 1.0, 5.25, 0.01, -math.pi / 2.0),
)
