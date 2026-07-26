"""Two namespaced navigation workers and one combined RViz.

The Isaac simulator provides two complete robot instances and namespaced
navigation, camera, arm and food endpoints.  Navigation-only mode is the
default for fleet driving checks: robot1 receives the first order and robot2
can receive a later order while robot1 is still active.  Full serving can be
enabled explicitly, with shared payload serialization restored.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def _worker(
    name: str,
    x: str,
    routes_file: str,
    use_sim_time,
    enable_serving,
    navigation_only,
):
    nav_share = get_package_share_directory("two_wheel_rails")
    nav_launch = os.path.join(nav_share, "launch", "nav2.launch.py")
    full_serving_enabled = IfCondition(PythonExpression([
        "'", enable_serving, "'.lower() == 'true' and '",
        navigation_only, "'.lower() == 'false'",
    ]))
    return GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_launch),
            launch_arguments={
                "namespace": name, "rviz": "false", "use_sim_time": use_sim_time,
                "initial_pose_x": x, "initial_pose_y": "5.25",
                "initial_pose_yaw": "-1.5707963267948966",
            }.items(),
        ),
        PushRosNamespace(name),
        Node(
            package="serving_robot_manager", executable="isaac_subsystem_adapter_node",
            name="isaac_subsystem_adapter", output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            condition=full_serving_enabled,
        ),
        Node(
            package="two_wheel_rails", executable="navigation_subsystem",
            name="navigation_subsystem", output="screen",
            parameters=[{"use_sim_time": use_sim_time, "routes_file": routes_file}],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
        Node(
            package="two_wheel_rails", executable="navigation_auto_initializer",
            name="navigation_auto_initializer", output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="serving_robot_manager", executable="manager_node",
            name="manager", output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "navigation_only": ParameterValue(navigation_only, value_type=bool),
                "require_navigation_ready": True,
            }],
            condition=IfCondition(enable_serving),
        ),
        Node(
            package="hand_safety", executable="hand_detector_node",
            name="hand_detector", output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "input_topic": "camera/color/image_raw",
                "roi_intrusion_topic": "hand_safety/intrusion",
                "table_arrived_topic": "serving_robot/table_arrived",
                "confidence": 0.70,
                "image_size": 1280,
                "half": False,
                "confirmation_frames": 5,
                "self_mask_enabled": True,
                "publish_annotated_image": False,
                "show_window": False,
            }],
            condition=full_serving_enabled,
        ),
    ])


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_serving = LaunchConfiguration("enable_serving_workers")
    navigation_only = LaunchConfiguration("navigation_only")
    serialize_shared_payloads = LaunchConfiguration("serialize_shared_payloads")
    nav_share = get_package_share_directory("two_wheel_rails")
    config = os.path.join(nav_share, "config")
    rviz_config = os.path.join(nav_share, "rviz", "multi_robot.rviz")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "enable_serving_workers",
            default_value="true",
            description=(
                "Enable the namespaced workers used by the integrated Isaac simulator"
            ),
        ),
        DeclareLaunchArgument(
            "navigation_only",
            default_value="true",
            description="Skip food spawning and arm serving; drive to the table and return",
        ),
        DeclareLaunchArgument(
            "serialize_shared_payloads",
            default_value="false",
            description="Allow another idle robot to accept an order while one robot is active",
        ),
        _worker(
            "robot1", "-0.90", os.path.join(config, "routes_robot1.yaml"),
            use_sim_time, enable_serving, navigation_only,
        ),
        _worker(
            "robot2", "0.90", os.path.join(config, "routes_robot2.yaml"),
            use_sim_time, enable_serving, navigation_only,
        ),
        Node(
            package="serving_robot_manager", executable="fleet_manager_node",
            name="fleet_manager", output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "serialize_shared_payloads": ParameterValue(
                    serialize_shared_payloads, value_type=bool
                ),
            }],
            condition=IfCondition(enable_serving),
        ),
        Node(
            package="serving_robot_manager",
            executable="path_yield_coordinator_node",
            name="path_yield_coordinator",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(enable_serving),
        ),
        Node(
            package="two_wheel_rails", executable="rviz_tf_aggregator",
            name="rviz_tf_aggregator", output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="rviz2", executable="rviz2", name="multi_robot_rviz",
            output="screen", arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
