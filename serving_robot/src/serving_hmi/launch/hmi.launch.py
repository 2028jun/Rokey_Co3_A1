from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serving_hmi',
            executable='hmi_backend_node',
            name='serving_hmi_backend_node',
            output='screen'
        )
    ])
