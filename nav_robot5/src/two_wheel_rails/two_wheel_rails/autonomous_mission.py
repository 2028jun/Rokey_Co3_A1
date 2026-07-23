#!/usr/bin/env python3
"""Free Nav2 path -> simplified straight segments -> rotate/drive mission."""

from __future__ import annotations

import argparse
import sys

import rclpy

from two_wheel_rails.autonomous_navigator import SimplifiedPathNavigator
from two_wheel_rails.nav_bootstrap import (
    make_pose,
    prepare_navigator,
)
from two_wheel_rails.rail_geometry import load_routes


def _goal_from_routes(
    routes: dict,
    table_id: int | None,
    kitchen: bool,
) -> tuple[float, float, float]:
    if kitchen:
        goal = routes.get("kitchen") or routes.get("spawn")
        if goal is None:
            raise RuntimeError("routes config has no kitchen or spawn pose")
    else:
        goal = routes["routes"][f"to_{table_id}"]["dock"]

    return float(goal["x"]), float(goal["y"]), float(goal["yaw"])


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Nav2 자유경로를 단순화해 제자리회전+직진만 수행"
    )
    target_group = parser.add_mutually_exclusive_group(required=True)
    target_group.add_argument(
        "--table-id",
        type=int,
        choices=[0, 1, 2, 3],
        help="Destination table ID (0, 1, 2, 3)",
    )
    target_group.add_argument(
        "--kitchen",
        action="store_true",
        help="Return to kitchen / spawn position",
    )

    args = parser.parse_args()

    routes = load_routes()
    gx, gy, gyaw = _goal_from_routes(routes, args.table_id, args.kitchen)

    rclpy.init(args=sys.argv)
    nav, tf_buffer, tracker = prepare_navigator(node_name="autonomous_navigator")
    nav.waitUntilNav2Active()

    controller = SimplifiedPathNavigator(nav, tf_buffer, tracker)
    goal = make_pose(nav, gx, gy, gyaw)

    label = "kitchen" if args.kitchen else f"table_{args.table_id}"
    print(f"[mission] target={label} goal=({gx:.2f},{gy:.2f},{gyaw:.2f})", flush=True)

    ok = controller.navigate_to(goal, label=label)
    rclpy.shutdown()
    if not ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
