"""Two namespaced full-serving workers and one combined RViz.

The Isaac simulator provides two complete robot instances and namespaced
navigation, camera, arm and food endpoints. Full serving with isolated payload
roots and remote YOLO is the production default. Navigation-only and local
hand detection remain explicit diagnostic overrides.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def _worker(
    name: str,
    x: str,
    routes_file: str,
    use_sim_time,
    enable_serving,
    navigation_only,
    enable_local_hand_detection,
):
    nav_share = get_package_share_directory("two_wheel_rails")
    nav_launch = os.path.join(nav_share, "launch", "nav2.launch.py")
    full_serving_enabled = IfCondition(PythonExpression([
        "'", enable_serving, "'.lower() == 'true' and '",
        navigation_only, "'.lower() == 'false'",
    ]))
    local_hand_detection_enabled = IfCondition(PythonExpression([
        "'", enable_serving, "'.lower() == 'true' and '",
        navigation_only, "'.lower() == 'false' and '",
        enable_local_hand_detection, "'.lower() == 'true'",
    ]))
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
            condition=full_serving_enabled,
        ),
        Node(
            package="two_wheel_rails", executable="navigation_subsystem",
            name="navigation_subsystem", output="screen",
            parameters=[{"use_sim_time": use_sim_time, "routes_file": routes_file}],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
        Node(
            package="two_wheel_rails", executable="navigation_auto_initializer",
            name="navigation_auto_initializer", output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="serving_robot_manager", executable="manager_node",
            name="manager", output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "navigation_only": ParameterValue(navigation_only, value_type=bool),
                "require_navigation_ready": True,
            }],
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
                "process_rate": 15.0,
                "confirmation_frames": 5,
                "self_mask_enabled": True,
                "publish_annotated_image": False,
                "show_window": False,
            }],
            condition=local_hand_detection_enabled,
        ),
    ])


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_serving = LaunchConfiguration("enable_serving_workers")
    navigation_only = LaunchConfiguration("navigation_only")
    enable_local_hand_detection = LaunchConfiguration(
        "enable_local_hand_detection"
    )
    serialize_shared_payloads = LaunchConfiguration("serialize_shared_payloads")
    nav_share = get_package_share_directory("two_wheel_rails")
    config = os.path.join(nav_share, "config")
    rviz_config = os.path.join(nav_share, "rviz", "multi_robot.rviz")
    robot1_worker = _worker(
        "robot1", "-0.90", os.path.join(config, "routes_robot1.yaml"),
        use_sim_time, enable_serving, navigation_only,
        enable_local_hand_detection,
    )
    robot2_worker = _worker(
        "robot2", "0.90", os.path.join(config, "routes_robot2.yaml"),
        use_sim_time, enable_serving, navigation_only,
        enable_local_hand_detection,
    )
    robot1_gate = Node(
        package="two_wheel_rails",
        executable="navigation_ready_gate",
        name="robot1_navigation_ready_gate",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "detail_topic": "/robot1/navigation/detail",
            "timeout_sec": 180.0,
        }],
    )
    robot2_gate = Node(
        package="two_wheel_rails",
        executable="navigation_ready_gate",
        name="robot2_navigation_ready_gate",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "detail_topic": "/robot2/navigation/detail",
            "timeout_sec": 180.0,
        }],
    )
    fleet_manager = Node(
        package="serving_robot_manager", executable="fleet_manager_node",
        name="fleet_manager", output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "serialize_shared_payloads": ParameterValue(
                serialize_shared_payloads, value_type=bool
            ),
        }],
        condition=IfCondition(enable_serving),
    )
    path_coordinator = Node(
        package="serving_robot_manager",
        executable="path_yield_coordinator_node",
        name="path_yield_coordinator",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(enable_serving),
    )

    def start_robot2(event, _context):
        if event.returncode == 0:
            return [robot2_worker, robot2_gate]
        return [
            LogInfo(msg="ERROR: robot1 initialization failed; robot2 not started"),
            EmitEvent(event=Shutdown(reason="robot1 initialization failed")),
        ]

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "enable_serving_workers",
            default_value="true",
            description=(
                "Enable the namespaced workers used by the integrated Isaac simulator"
            ),
        ),
        DeclareLaunchArgument(
            "navigation_only",
            default_value="false",
            description=(
                "Diagnostic override: skip food spawning and arm serving; "
                "drive to the table and return"
            ),
        ),
        DeclareLaunchArgument(
            "enable_local_hand_detection",
            default_value="false",
            description=(
                "Run both YOLO hand detectors on this computer. The default "
                "is false because remote_hand_detection.launch.py runs on "
                "the separate GPU worker."
            ),
        ),
        DeclareLaunchArgument(
            "serialize_shared_payloads",
            default_value="false",
            description=(
                "Legacy diagnostic serialization switch. Isolated robot "
                "payload roots allow the default false setting."
            ),
        ),
        robot1_worker,
        robot1_gate,
        # Expose /manager/order immediately. FleetManager already filters out
        # workers whose namespaced order service is not ready, so orders can
        # use robot1 while robot2 is still completing Nav2 initialization.
        # HMI keeps the order pending only when no worker is currently ready.
        fleet_manager,
        path_coordinator,
        RegisterEventHandler(
            OnProcessExit(target_action=robot1_gate, on_exit=start_robot2)
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
