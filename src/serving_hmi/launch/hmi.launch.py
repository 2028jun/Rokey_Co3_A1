from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='robot1',
            description='Robot namespace shown by the HMI camera and map',
        ),
        DeclareLaunchArgument(
            'camera_stale_timeout_sec',
            default_value='15.0',
            description='Seconds to retain the last camera frame during a stream gap',
        ),
        Node(
            package='serving_hmi',
            executable='hmi_backend_node',
            name='serving_hmi_backend_node',
            output='screen',
            parameters=[{
                'robot_namespace': LaunchConfiguration('robot_namespace'),
                'camera_stale_timeout_sec': LaunchConfiguration(
                    'camera_stale_timeout_sec'
                ),
            }],
        )
    ])
