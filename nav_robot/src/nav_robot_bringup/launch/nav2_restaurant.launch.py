#!/usr/bin/env python3
"""Full Nav2 (planner/controller) for Isaac restaurant — NOT for daily rail.

Do NOT run this alongside go_to_table: both publish /nav_robot/cmd_vel.
Daily table missions use odom-GT rail only; see docs/NAV2_QUICKSTART.md.
"""

import os
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _prepare(context, *args, **kwargs):
    bringup_dir = Path(get_package_share_directory("nav_robot_bringup"))
    params_src = bringup_dir / "config" / "nav2_params.yaml"
    bt_xml = str(bringup_dir / "behavior_trees" / "navigate_w_recovery.xml")

    with params_src.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)

    data.setdefault("bt_navigator", {}).setdefault("ros__parameters", {})
    data["bt_navigator"]["ros__parameters"]["default_nav_to_pose_bt_xml"] = bt_xml

    tmp = Path(tempfile.gettempdir()) / "nav_robot_nav2_params.yaml"
    with tmp.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(data, handle, sort_keys=False)

    os.environ["NAV_ROBOT_GENERATED_PARAMS"] = str(tmp)
    return []


def generate_launch_description():
    bringup_dir = get_package_share_directory("nav_robot_bringup")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    ws_root_guess = Path(bringup_dir).parents[3]
    default_map = ws_root_guess / "maps" / "restaurant" / "map.yaml"
    if not default_map.is_file():
        alt = Path.cwd() / "maps" / "restaurant" / "map.yaml"
        if alt.is_file():
            default_map = alt

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map")
    autostart = LaunchConfiguration("autostart")

    def _launch_nav2(context, *args, **kwargs):
        params_path = os.environ.get(
            "NAV_ROBOT_GENERATED_PARAMS",
            os.path.join(bringup_dir, "config", "nav2_params.yaml"),
        )
        # Isaac bridge already subscribes to both /cmd_vel and /nav_robot/cmd_vel.
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
                ),
                launch_arguments={
                    "map": map_file,
                    "use_sim_time": use_sim_time,
                    "params_file": params_path,
                    "autostart": autostart,
                    "use_composition": "False",
                    "slam": "False",
                }.items(),
            ),
        ]

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                "ROS_DOMAIN_ID", os.environ.get("ROS_DOMAIN_ID", "103")
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("map", default_value=str(default_map)),
            DeclareLaunchArgument("autostart", default_value="true"),
            OpaqueFunction(function=_prepare),
            OpaqueFunction(function=_launch_nav2),
        ]
    )
