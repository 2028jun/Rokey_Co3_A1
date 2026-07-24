"""Nav2 Integrated Serving Manager Launch.

Launches:
- Nav2 stack (via two_wheel_rails nav2.launch.py)
- navigation_subsystem node
- manager_node
- hand_detector_node

Keep vision_manager.launch.py intact for original direct navigation testing.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    vision_debug = LaunchConfiguration("vision_debug")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    rviz = LaunchConfiguration("rviz")

    two_wheel_share = get_package_share_directory("two_wheel_rails")
    nav2_launch_path = os.path.join(two_wheel_share, "launch", "nav2.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "vision_debug",
                default_value="false",
                description="Publish /hand_detection/image with boxes and ROI",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Automatically startup Nav2 stack",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Launch RViz2 for navigation visualization",
            ),
            # 1. Nav2 Stack bringing up AMCL, Costmaps, Planner, Controller, Topic Bridge & RViz
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_path),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "rviz": rviz,
                }.items(),
            ),
            # 2. Isaac Subsystem Adapter Node bridging Manager's TaskCommand to Isaac Sim's std_msgs/Int32 triggers/statuses
            Node(
                package="serving_robot_manager",
                executable="isaac_subsystem_adapter_node",
                name="isaac_subsystem_adapter_node",
                output="screen",
            ),
            # 3. Navigation Subsystem Node interfacing Nav2 with Manager Node
            Node(
                package="two_wheel_rails",
                executable="navigation_subsystem",
                name="navigation_subsystem",
                output="screen",
            ),
            # 4. Serving Manager Node handling ordering state machine
            Node(
                package="serving_robot_manager",
                executable="manager_node",
                name="manager_node",
                output="screen",
            ),
            # 5. Hand Safety Detector Node
            Node(
                package="hand_safety",
                executable="hand_detector_node",
                name="hand_detector_node",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/camera/color/image_raw",
                        "roi_intrusion_topic": "/hand_safety/intrusion",
                        "table_arrived_topic": "/serving_robot/table_arrived",
                        "confidence": 0.70,
                        "image_size": 1280,
                        "half": False,
                        "confirmation_frames": 5,
                        "self_mask_enabled": True,
                        "self_mask_value_max": 90,
                        "self_mask_saturation_max": 130,
                        "self_mask_min_box_overlap": 0.20,
                        "publish_annotated_image": ParameterValue(
                            vision_debug, value_type=bool
                        ),
                        "show_window": False,
                    }
                ],
            ),
        ]
    )
