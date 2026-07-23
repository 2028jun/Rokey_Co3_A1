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
    publish_initial_pose,
    sync_spawn,
)
from two_wheel_rails.rail_geometry import load_routes


def _goal_from_routes(routes: dict, table_id: int) -> tuple[float, float, float]:
    dock = routes["routes"][f"to_{table_id}"]["dock"]
    return float(dock["x"]), float(dock["y"]), float(dock["yaw"])


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Nav2 자유경로를 단순화해 제자리회전+직진만 수행"
    )
    parser.add_argument("--table-id", type=int, default=0, choices=[0, 1, 2, 3])
    parser.add_argument("--no-sync-spawn", action="store_true")
    args = parser.parse_args()

    routes = load_routes()
    spawn = routes.get("spawn") or routes.get("kitchen")
    sx, sy, syaw = float(spawn["x"]), float(spawn["y"]), float(spawn["yaw"])
    gx, gy, gyaw = _goal_from_routes(routes, args.table_id)

    rclpy.init(args=sys.argv)
    nav, tf_buffer, tracker = prepare_navigator(node_name="autonomous_navigator")
    nav.initial_pose = make_pose(nav, sx, sy, syaw)
    publish_initial_pose(nav, sx, sy, syaw, repeats=2)
    if tracker.xy is not None:
        nav.initial_pose_received = True
    nav.waitUntilNav2Active()

    if not args.no_sync_spawn and not sync_spawn(nav, tf_buffer, tracker, sx, sy, syaw):
        rclpy.shutdown()
        raise SystemExit(1)

    controller = SimplifiedPathNavigator(nav, tf_buffer, tracker)
    goal = make_pose(nav, gx, gy, gyaw)
    ok = controller.navigate_to(goal, label=f"table_{args.table_id}")
    rclpy.shutdown()
    if not ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
