#!/usr/bin/env python3
"""Kitchen spawn → rail to table → optional rail return (Nav2 + cross-track check)."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.utilities import remove_ros_args

from two_wheel_rails.nav_bootstrap import (
    make_pose,
    prepare_navigator,
    publish_initial_pose,
    sync_spawn,
    wait_for_existing_localization,
)
from two_wheel_rails.rail_geometry import load_routes
from two_wheel_rails.rail_navigator import RailNavigator


def _routes_path(robot_id: str) -> Path:
    config_dir = Path(get_package_share_directory("two_wheel_rails")) / "config"
    filename = "routes.yaml" if robot_id == "robot1" else f"routes_{robot_id}.yaml"
    return config_dir / filename


def main() -> None:
    parser = argparse.ArgumentParser(
        description="nav_robot5: routes.yaml 레일 + Nav2 추종 + 횡오차 검증"
    )
    parser.add_argument("--table-id", type=int, default=0, choices=[0, 1, 2, 3])
    parser.add_argument(
        "--robot-id",
        default="robot1",
        choices=["robot1", "robot2"],
        help="대상 로봇 (ROS 네임스페이스/routes 파일/TF 프레임 선택; "
        "실행 시 --ros-args -r __ns:=/<robot-id> 를 함께 지정해야 함)",
    )
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
        "--initialize-spawn",
        action="store_true",
        help="Run initial Isaac spawn teleport and AMCL initial pose reset (First run only)",
    )
    # Strip any "--ros-args -r __ns:=/robotN ..." tail (added by t3r) before
    # argparse sees it; rclpy.init() below still receives the full sys.argv.
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    # Frame names are NOT robot-id-prefixed: each robot's TF is already
    # isolated by topic (/robot1/tf vs /robot2/tf), and nav2-internal calls
    # (nav2_util::getCurrentPose) fall back to a hardcoded default frame
    # name rather than reading robot_base_frame, so all robots must share
    # the same plain frame name.
    base_link_frame = "base_link"
    routes = load_routes(_routes_path(args.robot_id))
    if args.list_routes:
        for key in sorted((routes.get("routes") or {}).keys()):
            print(key)
        return

    spawn = routes.get("spawn") or routes.get("kitchen")
    sx, sy, syaw = float(spawn["x"]), float(spawn["y"]), float(spawn["yaw"])

    rclpy.init(args=sys.argv)
    nav, tf_buffer, tracker = prepare_navigator(node_name=f"rail_navigator_{args.robot_id}")

    if args.initialize_spawn:
        print("[sync] Initializing spawn & AMCL pose via --initialize-spawn", flush=True)
        nav.initial_pose = make_pose(nav, sx, sy, syaw)
        publish_initial_pose(nav, sx, sy, syaw, repeats=2)
        if tracker.xy is not None:
            nav.initial_pose_received = True
        nav.waitUntilNav2Active()
        if not sync_spawn(nav, tf_buffer, tracker, sx, sy, syaw, base_link_frame=base_link_frame):
            rclpy.shutdown()
            raise SystemExit(1)
    else:
        # Do not let BasicNavigator publish its default map (0, 0, 0) pose
        # while merely waiting for an already-running Nav2 stack.
        nav.initial_pose_received = True
        nav.waitUntilNav2Active()
        current_pose = wait_for_existing_localization(
            nav, tf_buffer, tracker, timeout_sec=8.0, base_link_frame=base_link_frame
        )
        if current_pose is None:
            print("[mission] current localization unavailable", flush=True)
            rclpy.shutdown()
            raise SystemExit(1)

    rail = RailNavigator(nav, tf_buffer, tracker, base_link_frame=base_link_frame)
    ok = rail.follow_route(f"to_{args.table_id}", routes, label=f"to_{args.table_id}")
    if ok and args.return_kitchen:
        ok = rail.reverse_parking_exit(routes, args.table_id)
        if ok:
            ok = rail.follow_route(
                f"return_{args.table_id}",
                routes,
                label="return_kitchen",
            )

    rclpy.shutdown()
    if not ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
