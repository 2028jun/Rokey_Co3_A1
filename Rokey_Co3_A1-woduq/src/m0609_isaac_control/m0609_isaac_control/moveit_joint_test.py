"""Send a conservative joint target through MoveIt and execute it in Isaac Sim."""

import argparse
import math
import sys
import time

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes, PlanningOptions
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


JOINTS = [f'joint_{index}' for index in range(1, 7)]
DEFAULT_TARGET_DEG = [20.0, -30.0, 55.0, 0.0, 55.0, 10.0]


class MoveItJointTest(Node):
    def __init__(self) -> None:
        super().__init__('m0609_moveit_joint_test')
        self.client = ActionClient(self, MoveGroup, '/move_action')
        self._have_complete_joint_state = False
        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            qos_profile_sensor_data,
        )

    def _joint_state_callback(self, msg: JointState) -> None:
        if all(name in msg.name for name in JOINTS):
            self._have_complete_joint_state = True

    def _wait_for_joint_state(self, timeout_sec: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while (
            rclpy.ok()
            and not self._have_complete_joint_state
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._have_complete_joint_state

    def run(self, target_deg, plan_only: bool) -> bool:
        self.get_logger().info('waiting for a complete /joint_states message')
        if not self._wait_for_joint_state():
            self.get_logger().error(
                'no complete M0609 /joint_states received; check the Isaac '
                '/isaac_joint_states publisher and that the simulation is playing'
            )
            return False

        self.get_logger().info('waiting for /move_action')
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('/move_action is unavailable')
            return False

        constraints = Constraints()
        for name, degrees in zip(JOINTS, target_deg):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = math.radians(degrees)
            joint.tolerance_above = math.radians(0.5)
            joint.tolerance_below = math.radians(0.5)
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)

        request = MotionPlanRequest()
        request.group_name = 'manipulator'
        request.goal_constraints = [constraints]
        request.allowed_planning_time = 5.0
        request.num_planning_attempts = 5
        request.max_velocity_scaling_factor = 0.15
        request.max_acceleration_scaling_factor = 0.10

        options = PlanningOptions()
        options.plan_only = plan_only
        options.replan = False

        goal = MoveGroup.Goal(request=request, planning_options=options)
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('MoveIt rejected the test goal')
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped = result_future.result()
        result = wrapped.result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f'MoveIt failed: error_code={result.error_code.val}, status={wrapped.status}'
            )
            return False
        mode = 'planning' if plan_only else 'planning and execution'
        self.get_logger().info(f'{mode} test passed: {target_deg} deg')
        return True


def main(args=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--plan-only', action='store_true')
    parser.add_argument('--target-deg', nargs=6, type=float, default=DEFAULT_TARGET_DEG)
    parsed, ros_args = parser.parse_known_args(args=args)
    rclpy.init(args=ros_args)
    node = MoveItJointTest()
    try:
        success = node.run(parsed.target_deg, parsed.plan_only)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if success else 1)
