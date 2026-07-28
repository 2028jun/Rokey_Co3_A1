"""Pure helpers for deterministic peer-robot right-of-way decisions."""

from __future__ import annotations


def is_later_active_order(
    robot_name: str,
    peer_name: str,
    active: dict[str, bool],
    priorities: dict[str, float],
) -> bool:
    """Return whether ``robot_name`` must yield to an earlier active order.

    Fleet priorities are negative monotonic timestamps.  A later timestamp is
    therefore a smaller number.  Missing/inactive intents do not authorize a
    peer-only hard stop; equal values use robot2 as a deterministic tie-break.
    """
    if robot_name not in ("robot1", "robot2"):
        return False
    if peer_name not in ("robot1", "robot2") or peer_name == robot_name:
        return False
    if not active.get(robot_name, False) or not active.get(peer_name, False):
        return False
    my_priority = priorities.get(robot_name)
    peer_priority = priorities.get(peer_name)
    if my_priority is None or peer_priority is None:
        return False
    if abs(my_priority - peer_priority) < 1e-9:
        return robot_name == "robot2"
    return my_priority < peer_priority


def in_narrow_forward_strip(
    local_x: float,
    local_y: float,
    forward_limit_m: float,
    half_width_m: float,
) -> bool:
    """Return whether a lidar hit lies in the narrow peer-yield strip."""
    return 0.0 < local_x <= forward_limit_m and abs(local_y) <= half_width_m
