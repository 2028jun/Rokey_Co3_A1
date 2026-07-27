"""Run both namespaced YOLO hand detectors on a remote ROS 2 computer."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def _detector(robot_name: str, use_sim_time, device, process_rate):
    return GroupAction([
        PushRosNamespace(robot_name),
        Node(
            package="hand_safety",
            executable="hand_detector_node",
            name="hand_detector",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "input_topic": "camera/color/image_raw/compressed",
                "input_transport": "compressed",
                "roi_intrusion_topic": "hand_safety/intrusion",
                "table_arrived_topic": "serving_robot/table_arrived",
                "confidence": 0.70,
                "image_size": 1280,
                # Launch/YAML otherwise coerces the default value "0" to an
                # integer, while hand_detector_node declares this parameter
                # as a string accepted by Ultralytics ("0", "cpu", ...).
                "device": ParameterValue(device, value_type=str),
                "half": False,
                "process_rate": ParameterValue(
                    process_rate, value_type=float
                ),
                "confirmation_frames": 5,
                "self_mask_enabled": True,
                "publish_annotated_image": False,
                "show_window": False,
            }],
        ),
    ])


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    device = LaunchConfiguration("device")
    process_rate = LaunchConfiguration("process_rate")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "device",
            default_value="0",
            description="CUDA device passed to both YOLO detector nodes",
        ),
        DeclareLaunchArgument(
            "process_rate",
            default_value="15.0",
            description="Maximum inference frequency per robot",
        ),
        _detector("robot1", use_sim_time, device, process_rate),
        _detector("robot2", use_sim_time, device, process_rate),
    ])
