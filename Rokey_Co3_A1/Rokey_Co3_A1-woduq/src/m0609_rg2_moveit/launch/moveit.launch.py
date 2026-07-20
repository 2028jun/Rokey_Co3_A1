import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(package_name, relative_path):
    path = os.path.join(get_package_share_directory(package_name), relative_path)
    with open(path, encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def generate_launch_description():
    moveit_share = get_package_share_directory('m0609_rg2_moveit')
    description_share = get_package_share_directory('m0609_isaac_description')
    xacro_path = os.path.join(description_share, 'urdf', 'm0609_moveit.urdf.xacro')
    srdf_path = os.path.join(moveit_share, 'config', 'm0609_rg2.srdf')

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    demo_obstacle = LaunchConfiguration('demo_obstacle')

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_path]), value_type=str
        )
    }
    with open(srdf_path, encoding='utf-8') as stream:
        semantic = {'robot_description_semantic': stream.read()}

    kinematics = {
        'robot_description_kinematics': load_yaml(
            'm0609_rg2_moveit', 'config/kinematics.yaml'
        )
    }
    limits = {
        'robot_description_planning': load_yaml(
            'm0609_rg2_moveit', 'config/joint_limits.yaml'
        )
    }
    ompl = load_yaml('m0609_rg2_moveit', 'config/ompl_planning.yaml')
    planning = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl,
    }
    controllers = load_yaml(
        'm0609_rg2_moveit', 'config/moveit_controllers.yaml'
    )

    common = [
        robot_description,
        semantic,
        kinematics,
        limits,
        planning,
        {'use_sim_time': use_sim_time},
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('demo_obstacle', default_value='true'),
        Node(
            package='m0609_isaac_control',
            executable='trajectory_bridge',
            name='isaac_trajectory_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'state_topic': '/isaac_joint_states',
                'command_topic': '/isaac_joint_commands',
                'action_name': '/isaac_arm_controller/follow_joint_trajectory',
            }],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='m0609_world_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            output='screen',
            parameters=common + [controllers],
        ),
        Node(
            package='m0609_isaac_control',
            executable='demo_obstacle',
            name='moveit_demo_obstacle',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(demo_obstacle),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='moveit_rviz',
            output='screen',
            arguments=['-d', os.path.join(moveit_share, 'launch', 'moveit.rviz')],
            parameters=common,
            condition=IfCondition(rviz),
        ),
    ])
