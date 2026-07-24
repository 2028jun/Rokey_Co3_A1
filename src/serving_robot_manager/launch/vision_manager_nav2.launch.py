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
    two_wheel_share = get_package_share_directory("two_wheel_rails")
    nav2_launch_path = os.path.join(two_wheel_share, "launch", "nav2.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "vision_debug",
                default_value="false",
                description="Publish /hand_detection/image with boxes and ROI",
            ),
            # 1. Nav2 Stack bringing up AMCL, Costmaps, Planner, Controller and Topic Bridge
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_path)
            ),
            # 2. Navigation Subsystem Node interfacing Nav2 with Manager Node
            Node(
                package="two_wheel_rails",
                executable="navigation_subsystem",
                name="navigation_subsystem",
                output="screen",
            ),
            # 3. Serving Manager Node handling ordering state machine
            Node(
                package="serving_robot_manager",
                executable="manager_node",
                name="manager_node",
                output="screen",
            ),
            # 4. Hand Safety Detector Node
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
