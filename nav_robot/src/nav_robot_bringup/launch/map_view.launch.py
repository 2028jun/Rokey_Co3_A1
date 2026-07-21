#!/usr/bin/env python3
"""Map-only RViz support for the Isaac rail-navigation demo.

The Isaac bridge publishes ground-truth, world-aligned odometry.  Running AMCL
on top of it can introduce a false map->odom correction in the symmetric
restaurant, so visualization uses an identity transform instead.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = Path(get_package_share_directory("nav_robot_bringup"))
    workspace = bringup_dir.parents[3]
    default_map = workspace / "nav_robot/maps/restaurant/map.yaml"

    map_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value=str(default_map)),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    {"yaml_filename": map_file, "use_sim_time": use_sim_time}
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map_view",
                output="screen",
                parameters=[
                    {
                        "autostart": True,
                        "node_names": ["map_server"],
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom_identity",
                arguments=[
                    "--x", "0", "--y", "0", "--z", "0",
                    "--roll", "0", "--pitch", "0", "--yaw", "0",
                    "--frame-id", "map", "--child-frame-id", "odom",
                ],
            ),
        ]
    )
