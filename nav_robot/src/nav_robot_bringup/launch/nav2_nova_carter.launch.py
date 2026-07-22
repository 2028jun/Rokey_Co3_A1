#!/usr/bin/env python3
"""Nav2 stack for Nova Carter in the restaurant scene.

Default map: existing maps/restaurant/map.yaml (Occupancy Map Generator).
RViz: same config as nav2_restaurant.launch.py (nav2_restaurant.rviz).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def _nav_robot_root_from_bringup(bringup_dir: str) -> str:
    return os.path.abspath(os.path.join(bringup_dir, "..", "..", ".."))


def _default_restaurant_map(bringup_dir: str) -> str:
    """Prefer workspace map.yaml (existing Occupancy Map), then install copy."""
    root = _nav_robot_root_from_bringup(bringup_dir)
    for path in (
        os.path.join(os.getcwd(), "maps", "restaurant", "map.yaml"),
        os.path.join(root, "maps", "restaurant", "map.yaml"),
        os.path.join(bringup_dir, "maps", "restaurant", "map.yaml"),
    ):
        if os.path.isfile(path):
            return path
    return os.path.join(root, "maps", "restaurant", "map.yaml")


def _default_restaurant_rviz(bringup_dir: str) -> str:
    """Two-wheel costmap RViz layout; Nova uses /scan instead of /nav_robot/scan."""
    root = _nav_robot_root_from_bringup(bringup_dir)
    for path in (
        os.path.join(root, "src", "nav_robot_bringup", "rviz", "nav2_nova_carter.rviz"),
        os.path.join(bringup_dir, "rviz", "nav2_nova_carter.rviz"),
        os.path.join(root, "src", "nav_robot_bringup", "rviz", "nav2_restaurant.rviz"),
        os.path.join(bringup_dir, "rviz", "nav2_restaurant.rviz"),
    ):
        if os.path.isfile(path):
            return path
    return os.path.join(bringup_dir, "rviz", "nav2_restaurant.rviz")


def generate_launch_description():
    bringup_dir = get_package_share_directory("nav_robot_bringup")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    default_map = _default_restaurant_map(bringup_dir)
    default_rviz = _default_restaurant_rviz(bringup_dir)
    default_params = os.path.join(bringup_dir, "config", "nav2_params_nova_carter.yaml")

    namespace = LaunchConfiguration("namespace")
    map_yaml_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    bt_xml_file = LaunchConfiguration("default_bt_xml_filename")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    navigation_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "default_nav_to_pose_bt_xml": bt_xml_file,
        },
        convert_types=True,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="", description="Top-level namespace"),
            DeclareLaunchArgument(
                "map",
                default_value=default_map,
                description="Existing Occupancy Map (maps/restaurant/map.yaml)",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Nova Carter Nav2 parameters yaml",
            ),
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="Use Isaac /clock as ROS time"
            ),
            DeclareLaunchArgument(
                "autostart", default_value="true", description="Auto-activate lifecycle nodes"
            ),
            DeclareLaunchArgument(
                "default_bt_xml_filename",
                default_value=os.path.join(bringup_dir, "behavior_trees", "navigate_w_recovery.xml"),
                description="NavigateToPose behavior tree (replan + recovery)",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Launch RViz with map / costmaps / plans",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
                description="Same RViz as nav2_restaurant (nav2_restaurant.rviz)",
            ),
            # Nova Carter publishes /chassis/odom + RELIABLE scan; Nav2 needs /odom + BEST_EFFORT /scan.
            Node(
                package="nav_robot_missions",
                executable="nova_carter_topic_bridge",
                name="nova_carter_topic_bridge",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, "launch", "localization.launch.py")
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
