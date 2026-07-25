#!/usr/bin/env python3
"""Nav2 + AMCL for the two-wheel restaurant robot (occupancy map)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
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

    default_map = _default_map(pkg_share)
    default_params = os.path.join(pkg_share, "config", "nav2_params.yaml")
    default_rviz = os.path.join(pkg_share, "rviz", "nav2.rviz")
    default_urdf = os.path.join(pkg_share, "urdf", "two_wheel_robot_robot1.urdf")
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
    robot_urdf = LaunchConfiguration("robot_urdf")
    # robot_id matches namespace 1:1 in this setup ("robot1"/"robot2"); it
    # picks the routes/TF-frame prefix inside topic_bridge (see robot_id
    # ROS parameter in topic_bridge.py).
    robot_description = ParameterValue(
        Command(["cat ", robot_urdf]), value_type=str
    )

    # base_frame_id/robot_base_frame are NOT overridden per robot: both
    # robots share the plain "ridgeback_base_link" frame name from
    # nav2_params.yaml as-is. TF is isolated per robot by topic
    # (/robot1/tf vs /robot2/tf via PushRosNamespace + the /tf:=tf remap),
    # not by frame name -- and some nav2-internal calls
    # (nav2_util::getCurrentPose) silently fall back to a hardcoded
    # "base_link"-style default rather than reading robot_base_frame, so a
    # per-robot-prefixed frame name breaks bt_navigator's own TF lookups.
    param_rewrites = {
        "use_sim_time": use_sim_time,
        "autostart": autostart,
        "default_nav_to_pose_bt_xml": bt_xml_file,
        "default_nav_through_poses_bt_xml": bt_through_xml_file,
    }

    # navigation_launch.py (nav2_bringup) applies its own root_key=namespace
    # RewrittenYaml pass internally, so this copy must stay UNWRAPPED
    # (root_key="") or its param keys end up nested one level too deep and
    # every node silently falls back to defaults (e.g. DWB instead of our
    # RegulatedPurePursuitController).
    navigation_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites=param_rewrites,
        convert_types=True,
    )
    # collision_monitor is OUR OWN Node() below (not part of an include that
    # wraps it for us), so its params file needs the namespace-nested keys
    # to match its actual fully-qualified name once PushRosNamespace applies.
    collision_monitor_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_rewrites,
        convert_types=True,
    )

    # nav2_bringup's localization_launch.py/navigation_launch.py do NOT push
    # a ROS namespace onto their own nodes (map_server, amcl, controller_server,
    # etc. have no namespace= set in this Humble nav2_bringup) -- they only use
    # `namespace` to root_key-wrap their params file internally. Without an
    # explicit PushRosNamespace here, robot1's and robot2's controller_server
    # (etc.) would both be plain "/controller_server", colliding outright and
    # silently failing to match their (namespace-wrapped) parameters. Wrapping
    # everything below in GroupAction([PushRosNamespace(namespace), ...]) is
    # what actually puts every node -- ours and the included ones -- under
    # /robot1 or /robot2.
    grouped_nodes = GroupAction(
        [
            PushRosNamespace(namespace),
            Node(
                package="two_wheel_rails",
                executable="topic_bridge",
                name="two_wheel_rails_topic_bridge",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time, "robot_id": namespace}],
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
                # robot_state_publisher's C++ tf2_ros broadcaster hardcodes
                # the absolute /tf,/tf_static topics (same issue nav2_bringup
                # works around for its own nodes below); without this remap
                # its static base_link->wheel/lidar transforms would land on
                # the global tree instead of /robot1/tf_static, breaking the
                # whole TF chain AMCL/costmaps need.
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, "launch", "localization_launch.py")
                ),
                launch_arguments={
                    "namespace": namespace,
                    "map": map_yaml_file,
                    "params_file": navigation_params,
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
            # Independent safety layer.  Owns its own lifecycle manager
            # because it is not part of nav2_bringup's navigation_launch.py
            # lifecycle group; see nav2_params.yaml's collision_monitor
            # block for why it sits after velocity_smoother and gates
            # "cmd_vel" -> "cmd_vel_safe".
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="collision_monitor",
                output="screen",
                parameters=[collision_monitor_params],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_collision_monitor",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": ["collision_monitor"]},
                ],
            ),
            Node(
                condition=IfCondition(rviz),
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            ),
        ]
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
            DeclareLaunchArgument(
                "robot_urdf",
                default_value=default_urdf,
                description="Per-robot URDF (frame names prefixed by robot id)",
            ),
            grouped_nodes,
        ]
    )
