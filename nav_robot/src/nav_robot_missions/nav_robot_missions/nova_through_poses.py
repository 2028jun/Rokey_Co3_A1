#!/usr/bin/env python3
"""Nova Carter: kitchen -> aisle -> table via Nav2 goThroughPoses (sample_code style)."""

from __future__ import annotations

import math
import os
import sys
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.parameter import Parameter

KITCHEN_ID = 4  # waypoints.yaml kitchen dock (missions use SPAWN_XY_YAW for AMCL)
# Match Isaac spawn / AMCL initial_pose (not kitchen dock waypoint y=4.90).
SPAWN_XY_YAW = (0.21, 5.25, -math.pi / 2.0)
AISLE_NORTH = (0.0, 4.20)
SPINE_YAW = -math.pi / 2.0


def get_quaternion_from_euler(roll, pitch, yaw):
    qx = (
        math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2)
        - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    )
    qy = (
        math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    )
    qz = (
        math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    )
    qw = (
        math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2)
        + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    )
    return [qx, qy, qz, qw]


def get_euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw


def print_final_pose(pose_msg):
    if not pose_msg:
        return
    pos = pose_msg.pose.position
    ori = pose_msg.pose.orientation
    _, _, yaw_rad = get_euler_from_quaternion(ori.x, ori.y, ori.z, ori.w)
    print("-" * 50)
    print(f"최종 위치: X = {pos.x:.3f} m, Y = {pos.y:.3f} m")
    print(f"최종 방향: {math.degrees(yaw_rad):.1f}°")
    print("-" * 50)


def create_pose(navigator, x, y, yaw_rad):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    q = get_quaternion_from_euler(0, 0, float(yaw_rad))
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose


def resolve_waypoints(explicit: str = "") -> Path:
    if explicit:
        path = Path(explicit)
        if path.is_file():
            return path
        raise FileNotFoundError(path)
    for path in (
        Path(get_package_share_directory("nav_robot_missions")) / "config" / "waypoints.yaml",
        Path(__file__).resolve().parents[2] / "config" / "waypoints.yaml",
    ):
        if path.is_file():
            return path
    raise FileNotFoundError("waypoints.yaml")


def load_destinations(path: Path) -> dict[int, tuple[float, float, float]]:
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    dests = (data or {}).get("destinations") or {}
    return {
        int(key): (float(d["x"]), float(d["y"]), float(d["yaw"]))
        for key, d in dests.items()
    }


def route_kitchen_to_table(table: tuple[float, float, float]) -> list[tuple[float, float, float]]:
    """Spine aisle then branch to table dock (sample goThroughPoses waypoints)."""
    tx, ty, tyaw = table
    branch_x = -1.05 if tx < 0.0 else 1.05
    return [
        (AISLE_NORTH[0], AISLE_NORTH[1], SPINE_YAW),
        (0.0, ty, SPINE_YAW),
        (branch_x, ty, tyaw),
        (tx, ty, tyaw),
    ]


def main(argv=None) -> int:
    os.environ.setdefault("ROS_DOMAIN_ID", "103")
    rclpy.init(args=None)
    nav = BasicNavigator("nova_through_poses")
    nav.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    nav.declare_parameter("table_id", int(os.environ.get("NAV_ROBOT_TABLE_ID", "2")))
    nav.declare_parameter("destinations_file", "")
    table_id = int(nav.get_parameter("table_id").value)
    dest_file = nav.get_parameter("destinations_file").value

    try:
        if table_id < 0 or table_id > 3:
            nav.get_logger().error(f"table_id out of range: {table_id}")
            return 2

        destinations = load_destinations(resolve_waypoints(dest_file or ""))
        if table_id not in destinations:
            nav.get_logger().error(f"table_{table_id} missing from waypoints.yaml")
            return 2

        table = destinations[table_id]

        init_pose = create_pose(nav, *SPAWN_XY_YAW)
        nav.setInitialPose(init_pose)
        nav.waitUntilNav2Active()
        time.sleep(1.0)
        try:
            nav.clearAllCostmaps()
        except Exception:
            pass

        waypoints = [
            create_pose(nav, x, y, yaw)
            for x, y, yaw in route_kitchen_to_table(table)
        ]
        nav.get_logger().info(
            f"nova goThroughPoses table_{table_id} ({len(waypoints)} poses)"
        )
        print("다중 경유지 주행을 시작합니다...")
        nav.goThroughPoses(waypoints)

        last_pose = None
        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            if feedback:
                last_pose = feedback.current_pose
                remaining = getattr(feedback, "number_of_poses_remaining", "?")
                print(
                    f"남은 경유지: {remaining} | "
                    f"남은 거리: {feedback.distance_remaining:.2f} m"
                )
            time.sleep(1.0)

        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"\ntable_{table_id} 경유 도착 완료")
            print_final_pose(last_pose)
            return 0
        if result == TaskResult.CANCELED:
            print("주행이 취소되었습니다.")
            return 1
        print("주행 실패")
        return 1
    finally:
        nav.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
