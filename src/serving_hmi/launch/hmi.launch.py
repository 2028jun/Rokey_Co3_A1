import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    domain_id = os.environ.get('ROS_DOMAIN_ID', '102')
    
    return LaunchDescription([
        Node(
            package='serving_hmi',
            executable='hmi_backend_node',
            name='serving_hmi_backend_node',
            output='screen',
            additional_env={'ROS_DOMAIN_ID': domain_id}
        )
    ])
