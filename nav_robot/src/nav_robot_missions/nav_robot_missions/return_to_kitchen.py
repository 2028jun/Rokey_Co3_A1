#!/usr/bin/env python3
"""Table -> kitchen return mission (WIP — separate from go_to_table)."""

from __future__ import annotations

import os
import sys

import rclpy
from rclpy.node import Node

from nav_robot_missions.rail_mission import RailMissionNode, resolve_config


def _load_params(b: Node) -> tuple[int, str, dict]:
    b.declare_parameter("table_id", int(os.environ.get("NAV_ROBOT_TABLE_ID", "0")))
    b.declare_parameter("routes_file", "")
    b.declare_parameter("linear_speed", 0.35)
    b.declare_parameter("angular_speed", 0.55)
    b.declare_parameter("dock_speed", 0.16)
    b.declare_parameter("xy_tolerance", 0.12)
    b.declare_parameter("yaw_tolerance", 0.12)
    b.declare_parameter("lat_tolerance", 0.12)
    b.declare_parameter("segment_timeout", 90.0)
    b.declare_parameter("stall_timeout", 6.0)
    tid = int(b.get_parameter("table_id").value)
    rf = b.get_parameter("routes_file").value
    params = {k: b.get_parameter(k).value for k in (
        "linear_speed", "angular_speed", "dock_speed", "xy_tolerance",
        "yaw_tolerance", "lat_tolerance", "segment_timeout", "stall_timeout",
    )}
    return tid, rf, params


def main(argv=None) -> int:
    rclpy.init(args=None)
    node = None
    try:
        bootstrap = Node("nav_robot_return_to_kitchen_bootstrap")
        table_id, routes_file, params = _load_params(bootstrap)
        bootstrap.destroy_node()
        if table_id < 0 or table_id > 3:
            return 2

        node = RailMissionNode(
            "nav_robot_return_to_kitchen",
            table_id,
            resolve_config("routes.yaml", routes_file or None),
            float(params["linear_speed"]),
            float(params["angular_speed"]),
            float(params["dock_speed"]),
            float(params["xy_tolerance"]),
            float(params["yaw_tolerance"]),
            float(params["lat_tolerance"]),
            float(params["segment_timeout"]),
            float(params["stall_timeout"]),
        )
        node.get_logger().warning(
            "return_to_kitchen is WIP — start at the table dock pose, no teleport."
        )
        if not node._wait_odom():
            node.get_logger().error("no /nav_robot/odom")
            return 1
        p = node.pose()
        if p:
            node.get_logger().info(f"return start odom=({p[0]:.2f},{p[1]:.2f})")
        if node.follow_return(node._route(False)):
            return 0
        node._stop()
        return 2
    finally:
        if node is not None:
            try:
                node._stop()
            except Exception:
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
