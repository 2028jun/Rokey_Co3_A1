"""Start the serving manager and its hand-safety detector together."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    vision_debug = LaunchConfiguration("vision_debug")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "vision_debug",
                default_value="false",
                description="Publish /hand_detection/image with boxes and ROI",
            ),
            # ROS Humble Python owns the custom TaskCommand services and
            # forwards plain Int32 triggers to Isaac Sim's Python 3.11 bridge.
            # The Isaac simulation thread still performs the actual
            # stop/pivot/straight axis-route control.
            Node(
                package="nav_robot_missions",
                executable="direct_nav_server_node",
                name="direct_nav_server_node",
                output="screen",
            ),
            Node(
                package="serving_robot_manager",
                executable="manager_node",
                name="manager_node",
                output="screen",
            ),
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
                        "confidence": 0.60,
                        "image_size": 1280,
                        "half": False,
                        "publish_annotated_image": ParameterValue(
                            vision_debug, value_type=bool
                        ),
                        "show_window": False,
                    }
                ],
            ),
        ]
    )
