#!/usr/bin/env python3
"""SLAM mapping mode: reuses nav_restaurant_demo.py's existing /scan, /odom
and TF (via two_wheel_rails' topic_bridge + robot_state_publisher) instead
of duplicating a separate bridge/URDF, then layers async_slam_toolbox_node
on top.

This intentionally does NOT launch AMCL/controller_server/bt_navigator --
there is no map yet.  It also does NOT run nav2_collision_monitor: that
node's stop/slowdown zones are tuned for live serving around people and
false-trigger on doorway frames/walls near pinch points (e.g. the kitchen
doorway spawn), silently withholding all cmd_vel_safe output instead of
publishing.  Mapping is a supervised, person-free patrol
(NAV_CROSSING_PEDESTRIAN=0), so slam_patrol publishes straight to
"cmd_vel_safe" -- the topic NavBridge actually listens to -- itself.

Requires Isaac (nav_restaurant_demo.py) already publishing
/two_wheel/scan_raw, /two_wheel/odom_raw, /clock.
Drive with: ros2 run map_gen slam_patrol
Save with: bash src/map_gen/tools/save_slam_map.sh
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
    two_wheel_rails_share = get_package_share_directory("two_wheel_rails")

    slam_params = os.path.join(pkg_share, "config", "slam_toolbox.yaml")
    default_rviz = os.path.join(pkg_share, "rviz", "slam.rviz")
    robot_urdf = os.path.join(two_wheel_rails_share, "urdf", "two_wheel_robot.urdf")
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
                package="two_wheel_rails",
                executable="topic_bridge",
                name="two_wheel_rails_topic_bridge",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="two_wheel_robot_state_publisher",
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
