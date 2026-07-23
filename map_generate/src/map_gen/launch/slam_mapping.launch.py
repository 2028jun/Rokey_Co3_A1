#!/usr/bin/env python3
"""SLAM mapping: topic_bridge + robot_state_publisher + async_slam_toolbox (+ optional RViz).

Requires Isaac publishing /two_wheel/scan_raw, /two_wheel/odom_raw, /clock, /cmd_vel.
Drive with: ros2 run map_gen slam_patrol
Save with: bash tools/save_slam_map.sh
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("map_gen")
    slam_params = os.path.join(pkg_share, "config", "slam_toolbox.yaml")
    robot_urdf = os.path.join(pkg_share, "urdf", "two_wheel_robot.urdf")
    default_rviz = os.path.join(pkg_share, "rviz", "slam.rviz")

    with open(robot_urdf, encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Isaac /clock as ROS time",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Launch RViz2",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
                description="RViz config path",
            ),
            Node(
                package="map_gen",
                executable="topic_bridge",
                name="map_gen_topic_bridge",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="map_gen_robot_state_publisher",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"robot_description": robot_description},
                ],
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[slam_params, {"use_sim_time": use_sim_time}],
            ),
            Node(
                condition=IfCondition(rviz),
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
