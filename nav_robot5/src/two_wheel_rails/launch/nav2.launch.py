#!/usr/bin/env python3
"""Nav2 + AMCL for the two-wheel restaurant robot (occupancy map)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def _workspace_root() -> str:
    return os.environ.get(
        "NAV_ROBOT5_WS",
        os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..")),
    )


def _default_map() -> str:
    root = _workspace_root()
    for path in (
        os.path.join(os.getcwd(), "maps", "restaurant", "map.yaml"),
        os.path.join(root, "maps", "restaurant", "map.yaml"),
    ):
        if os.path.isfile(path):
            return path
    return os.path.join(root, "maps", "restaurant", "map.yaml")


def generate_launch_description():
    pkg_share = get_package_share_directory("two_wheel_rails")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    nav2_bt_dir = get_package_share_directory("nav2_bt_navigator")

    default_map = _default_map()
    default_params = os.path.join(pkg_share, "config", "nav2_params.yaml")
    default_rviz = os.path.join(pkg_share, "rviz", "nav2.rviz")
    robot_urdf = os.path.join(pkg_share, "urdf", "two_wheel_robot.urdf")
    with open(robot_urdf, encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()
    default_bt = os.path.join(
        nav2_bt_dir,
        "behavior_trees",
        "navigate_to_pose_w_replanning_and_recovery.xml",
    )
    default_bt_through = os.path.join(
        nav2_bt_dir,
        "behavior_trees",
        "navigate_through_poses_w_replanning_and_recovery.xml",
    )

    namespace = LaunchConfiguration("namespace")
    map_yaml_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    bt_xml_file = LaunchConfiguration("default_bt_xml_filename")
    bt_through_xml_file = LaunchConfiguration("default_nav_through_poses_bt_xml")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    navigation_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "default_nav_to_pose_bt_xml": bt_xml_file,
            "default_nav_through_poses_bt_xml": bt_through_xml_file,
        },
        convert_types=True,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument(
                "map",
                default_value=default_map,
                description="Occupancy map yaml",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Nav2 parameters",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument(
                "default_bt_xml_filename",
                default_value=default_bt,
            ),
            DeclareLaunchArgument(
                "default_nav_through_poses_bt_xml",
                default_value=default_bt_through,
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, "launch", "localization_launch.py")
                ),
                launch_arguments={
                    "namespace": namespace,
                    "map": map_yaml_file,
                    "params_file": params_file,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
                ),
                launch_arguments={
                    "namespace": namespace,
                    "params_file": navigation_params,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                }.items(),
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
