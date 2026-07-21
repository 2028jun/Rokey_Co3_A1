#!/usr/bin/env python3
"""AMCL + map_server only (observe / RViz demo).

No Nav2 planner/controller. The rail mission (go_to_table) does NOT read
AMCL — control pose is /nav_robot/odom only. Safe to run beside the mission
for visualization; do NOT use nav2_restaurant.launch.py with the rail.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    bringup_dir = get_package_share_directory("nav_robot_bringup")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    ws_root_guess = Path(bringup_dir).parents[3]
    default_map = ws_root_guess / "maps" / "restaurant" / "map.yaml"
    if not default_map.is_file():
        alt = Path.cwd() / "maps" / "restaurant" / "map.yaml"
        if alt.is_file():
            default_map = alt

    params = os.path.join(bringup_dir, "config", "nav2_params.yaml")

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                "ROS_DOMAIN_ID", os.environ.get("ROS_DOMAIN_ID", "103")
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("map", default_value=str(default_map)),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("params_file", default_value=params),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        nav2_bringup_dir, "launch", "localization_launch.py"
                    )
                ),
                launch_arguments={
                    "map": LaunchConfiguration("map"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "params_file": LaunchConfiguration("params_file"),
                    "autostart": LaunchConfiguration("autostart"),
                }.items(),
            ),
        ]
    )
