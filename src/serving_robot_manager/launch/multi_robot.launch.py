"""Two namespaced integrated serving stacks and one combined RViz.

The Isaac simulator provides two complete robot instances and namespaced
navigation, camera, arm and food endpoints.  Payload prims are still shared,
so FleetManager serializes complete serving orders until those assets become
robot-local.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def _worker(name: str, x: str, routes_file: str, use_sim_time, enable_serving):
    nav_share = get_package_share_directory("two_wheel_rails")
    nav_launch = os.path.join(nav_share, "launch", "nav2.launch.py")
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
            condition=IfCondition(enable_serving),
        ),
        Node(
            package="two_wheel_rails", executable="navigation_subsystem",
            name="navigation_subsystem", output="screen",
            parameters=[{"use_sim_time": use_sim_time, "routes_file": routes_file}],
        ),
        Node(
            package="two_wheel_rails", executable="navigation_auto_initializer",
            name="navigation_auto_initializer", output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="serving_robot_manager", executable="manager_node",
            name="manager", output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
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
            condition=IfCondition(enable_serving),
        ),
    ])


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_serving = LaunchConfiguration("enable_serving_workers")
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
        _worker("robot1", "-0.90", os.path.join(config, "routes_robot1.yaml"), use_sim_time, enable_serving),
        _worker("robot2", "0.90", os.path.join(config, "routes_robot2.yaml"), use_sim_time, enable_serving),
        Node(
            package="serving_robot_manager", executable="fleet_manager_node",
            name="fleet_manager", output="screen",
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
