#!/usr/bin/env python3
"""One namespaced Nav2 stack for a serving robot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import RewrittenYaml


def _workspace_root() -> str:
    return os.environ.get(
        "NAV_ROBOT5_WS",
        os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..")),
    )


def _default_map(pkg_share: str) -> str:
    root = _workspace_root()
    for path in (
        os.path.join(pkg_share, "maps", "restaurant", "map.yaml"),
        os.path.join(root, "maps", "restaurant", "map.yaml"),
    ):
        if os.path.isfile(path):
            return path
    return os.path.join(root, "maps", "restaurant", "map.yaml")


def generate_launch_description():
    pkg_share = get_package_share_directory("two_wheel_rails")
    nav2_share = get_package_share_directory("nav2_bringup")
    bt_share = get_package_share_directory("nav2_bt_navigator")
    urdf_path = os.path.join(pkg_share, "urdf", "two_wheel_robot.urdf")
    with open(urdf_path, encoding="utf-8") as stream:
        robot_description = stream.read()

    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    initial_x = LaunchConfiguration("initial_pose_x")
    initial_y = LaunchConfiguration("initial_pose_y")
    initial_yaw = LaunchConfiguration("initial_pose_yaw")

    rewrites = {
        "use_sim_time": use_sim_time,
        "autostart": autostart,
        "x": initial_x,
        "y": initial_y,
        "yaw": initial_yaw,
    }
    navigation_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites=rewrites,
        convert_types=True,
    )
    collision_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=rewrites,
        convert_types=True,
    )
    # The three Nav2 lifecycle managers must not autostart concurrently. Under
    # the two-robot Fast DDS load, simultaneous change_state calls can lose the
    # response after a node has configured successfully, leaving its manager
    # blocked forever. The sequencer below starts the groups one at a time.
    managed_autostart = "false"

    group = GroupAction([
        PushRosNamespace(namespace),
        Node(
            package="two_wheel_rails", executable="topic_bridge",
            name="topic_bridge", output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            name="robot_state_publisher", output="screen",
            parameters=[{"use_sim_time": use_sim_time,
                         "robot_description": robot_description}],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, "launch", "localization_launch.py")
            ),
            launch_arguments={
                "namespace": namespace,
                "map": LaunchConfiguration("map"),
                "params_file": navigation_params,
                "use_sim_time": use_sim_time,
                "autostart": managed_autostart,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, "launch", "navigation_launch.py")
            ),
            launch_arguments={
                "namespace": namespace,
                "params_file": navigation_params,
                "use_sim_time": use_sim_time,
                "autostart": managed_autostart,
            }.items(),
        ),
        Node(
            package="nav2_collision_monitor", executable="collision_monitor",
            name="collision_monitor", output="screen",
            parameters=[collision_params],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_collision_monitor", output="screen",
            # IncludeLaunchDescription arguments are strings, but a directly
            # constructed ROS parameter must retain its boolean type.
            parameters=[{"use_sim_time": use_sim_time}, {"autostart": False},
                        {"node_names": ["collision_monitor"]}],
        ),
        Node(
            package="two_wheel_rails",
            executable="nav2_lifecycle_sequencer",
            name="nav2_lifecycle_sequencer",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                # Service discovery for the second namespaced stack can miss
                # its first Fast DDS discovery window. Never block a whole
                # minute on one stale wait; poll briefly and resume from the
                # lifecycle states that are already available.
                "service_wait_timeout_sec": 2.0,
                "transition_timeout_sec": 5.0,
                "startup_retries": 90,
                "startup_retry_delay_sec": 1.0,
            }],
        ),
    ])

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("map", default_value=_default_map(pkg_share)),
        DeclareLaunchArgument(
            "params_file", default_value=os.path.join(pkg_share, "config", "nav2_params.yaml")
        ),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument(
            "rviz_config", default_value=os.path.join(pkg_share, "rviz", "nav2.rviz")
        ),
        DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_y", default_value="5.25"),
        DeclareLaunchArgument("initial_pose_yaw", default_value="-1.5707963267948966"),
        DeclareLaunchArgument(
            "default_bt_xml_filename",
            default_value=os.path.join(bt_share, "behavior_trees",
                                       "navigate_to_pose_w_replanning_and_recovery.xml"),
        ),
        DeclareLaunchArgument(
            "default_nav_through_poses_bt_xml",
            default_value=os.path.join(bt_share, "behavior_trees",
                                       "navigate_through_poses_w_replanning_and_recovery.xml"),
        ),
        group,
        Node(
            condition=IfCondition(rviz), package="rviz2", executable="rviz2",
            name="rviz2", output="screen", arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
