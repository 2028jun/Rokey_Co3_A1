from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("ridgeback_m0609_description"))
    xacro_path = share / "urdf" / "ridgeback_m0609.urdf.xacro"
    description = {"robot_description": Command(["xacro ", str(xacro_path)])}

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[description],
                output="screen",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                parameters=[description],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                parameters=[description],
                output="screen",
            ),
        ]
    )
