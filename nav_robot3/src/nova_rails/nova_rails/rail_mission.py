#!/usr/bin/env python3
"""Kitchen spawn → rail to table → optional rail return (Nav2 + cross-track check)."""

from __future__ import annotations

import argparse
import sys

import rclpy

from nova_rails.nav_bootstrap import (
    make_pose,
    prepare_navigator,
    publish_initial_pose,
    sync_spawn,
)
from nova_rails.rail_geometry import load_routes
from nova_rails.rail_navigator import RailNavigator


def main() -> None:
    parser = argparse.ArgumentParser(
        description="nav_robot3: routes.yaml 레일 + Nav2 추종 + 횡오차 검증"
    )
    parser.add_argument("--table-id", type=int, default=0, choices=[0, 1, 2, 3])
    parser.add_argument(
        "--return-kitchen",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument(
        "--list-routes",
        action="store_true",
        help="routes.yaml 키만 출력하고 종료",
    )
    parser.add_argument(
        "--no-sync-spawn",
        action="store_true",
        help="미션 시작 시 텔레포트/AMCL 생략",
    )
    args = parser.parse_args()

    routes = load_routes()
    if args.list_routes:
        for key in sorted((routes.get("routes") or {}).keys()):
            print(key)
        return

    spawn = routes.get("spawn") or routes.get("kitchen")
    sx, sy, syaw = float(spawn["x"]), float(spawn["y"]), float(spawn["yaw"])

    rclpy.init(args=sys.argv)
    nav, tf_buffer, tracker = prepare_navigator()

    nav.initial_pose = make_pose(nav, sx, sy, syaw)
    publish_initial_pose(nav, sx, sy, syaw, repeats=2)
    if tracker.xy is not None:
        nav.initial_pose_received = True
    nav.waitUntilNav2Active()

    if not args.no_sync_spawn:
        if not sync_spawn(nav, tf_buffer, tracker, sx, sy, syaw):
            rclpy.shutdown()
            raise SystemExit(1)

    rail = RailNavigator(nav, tf_buffer, tracker)
    ok = rail.follow_route(f"to_{args.table_id}", routes, label=f"to_{args.table_id}")
    if ok and args.return_kitchen:
        ok = rail.follow_route(
            f"from_{args.table_id}", routes, label=f"from_{args.table_id}"
        )

    rclpy.shutdown()
    if not ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
