"""Expose Isaac Sim JointState command transport as FollowJointTrajectory."""

import math
import threading
import time
from typing import Dict, List, Optional

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


ARM_JOINTS = [f'joint_{index}' for index in range(1, 7)]


def duration_seconds(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1.0e-9


class IsaacTrajectoryBridge(Node):
    """Interpolate MoveIt trajectories and publish Isaac joint commands."""

    def __init__(self) -> None:
        super().__init__('isaac_trajectory_bridge')
        self.declare_parameter('state_topic', '/isaac_joint_states')
        self.declare_parameter('command_topic', '/isaac_joint_commands')
        self.declare_parameter(
            'action_name', '/isaac_arm_controller/follow_joint_trajectory'
        )
        self.declare_parameter('control_rate_hz', 100.0)
        self.declare_parameter('state_timeout_sec', 1.0)
        self.declare_parameter('goal_tolerance_rad', 0.05)
        self.declare_parameter('settle_timeout_sec', 2.0)

        state_topic = self.get_parameter('state_topic').value
        command_topic = self.get_parameter('command_topic').value
        action_name = self.get_parameter('action_name').value

        self._lock = threading.Lock()
        self._positions: Dict[str, float] = {}
        self._state_monotonic: Optional[float] = None

        self._command_pub = self.create_publisher(JointState, command_topic, 10)
        self._state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_subscription(
            JointState, state_topic, self._state_callback, qos_profile_sensor_data
        )
        self._server = ActionServer(
            self,
            FollowJointTrajectory,
            action_name,
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        self.get_logger().info(
            f'bridge ready: {state_topic} -> /joint_states, '
            f'{action_name} -> {command_topic}'
        )

    def _state_callback(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        with self._lock:
            for name in ARM_JOINTS:
                if name in positions and math.isfinite(positions[name]):
                    self._positions[name] = float(positions[name])
            if all(name in self._positions for name in ARM_JOINTS):
                self._state_monotonic = time.monotonic()

        filtered = JointState()
        filtered.header = msg.header
        filtered.name = list(ARM_JOINTS)
        filtered.position = [positions[name] for name in ARM_JOINTS if name in positions]
        if len(filtered.position) == len(ARM_JOINTS):
            velocity = dict(zip(msg.name, msg.velocity))
            effort = dict(zip(msg.name, msg.effort))
            if all(name in velocity for name in ARM_JOINTS):
                filtered.velocity = [velocity[name] for name in ARM_JOINTS]
            if all(name in effort for name in ARM_JOINTS):
                filtered.effort = [effort[name] for name in ARM_JOINTS]
            self._state_pub.publish(filtered)

    def _current_positions(self) -> Optional[List[float]]:
        timeout = float(self.get_parameter('state_timeout_sec').value)
        with self._lock:
            if self._state_monotonic is None:
                return None
            if time.monotonic() - self._state_monotonic > timeout:
                return None
            return [self._positions[name] for name in ARM_JOINTS]

    def _goal_callback(self, goal_request) -> GoalResponse:
        trajectory = goal_request.trajectory
        if set(trajectory.joint_names) != set(ARM_JOINTS):
            self.get_logger().error(
                f'reject joint names: {list(trajectory.joint_names)}'
            )
            return GoalResponse.REJECT
        if not trajectory.points or self._current_positions() is None:
            self.get_logger().error('reject: empty trajectory or stale Isaac state')
            return GoalResponse.REJECT

        previous_time = -1.0
        joint_count = len(trajectory.joint_names)
        for point in trajectory.points:
            point_time = duration_seconds(point.time_from_start)
            if (
                len(point.positions) != joint_count
                or not all(math.isfinite(value) for value in point.positions)
                or point_time <= previous_time
            ):
                self.get_logger().error('reject malformed trajectory point')
                return GoalResponse.REJECT
            previous_time = point_time
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_callback(_goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _publish_command(self, positions: List[float]) -> None:
        command = JointState()
        command.header.stamp = self.get_clock().now().to_msg()
        command.name = list(ARM_JOINTS)
        command.position = [float(value) for value in positions]
        self._command_pub.publish(command)

    def _feedback(self, goal_handle, desired: List[float]) -> None:
        actual = self._current_positions()
        if actual is None:
            return
        feedback = FollowJointTrajectory.Feedback()
        feedback.header.stamp = self.get_clock().now().to_msg()
        feedback.joint_names = list(ARM_JOINTS)
        feedback.desired = JointTrajectoryPoint(positions=desired)
        feedback.actual = JointTrajectoryPoint(positions=actual)
        feedback.error = JointTrajectoryPoint(
            positions=[wanted - measured for wanted, measured in zip(desired, actual)]
        )
        goal_handle.publish_feedback(feedback)

    def _execute(self, goal_handle):
        result = FollowJointTrajectory.Result()
        trajectory = goal_handle.request.trajectory
        input_index = {name: index for index, name in enumerate(trajectory.joint_names)}
        points = [
            (
                duration_seconds(point.time_from_start),
                [point.positions[input_index[name]] for name in ARM_JOINTS],
            )
            for point in trajectory.points
        ]
        start = self._current_positions()
        if start is None:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = 'Isaac joint state is unavailable'
            goal_handle.abort()
            return result

        rate = max(float(self.get_parameter('control_rate_hz').value), 1.0)
        start_time = time.monotonic()
        segment_start_time = 0.0
        segment_start = start

        for segment_end_time, segment_end in points:
            duration = segment_end_time - segment_start_time
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = 'trajectory canceled'
                    goal_handle.canceled()
                    return result
                elapsed = time.monotonic() - start_time
                if elapsed >= segment_end_time:
                    break
                alpha = min(max((elapsed - segment_start_time) / duration, 0.0), 1.0)
                desired = [
                    begin + alpha * (end - begin)
                    for begin, end in zip(segment_start, segment_end)
                ]
                self._publish_command(desired)
                self._feedback(goal_handle, desired)
                time.sleep(1.0 / rate)
            self._publish_command(segment_end)
            segment_start_time = segment_end_time
            segment_start = segment_end

        tolerance = float(self.get_parameter('goal_tolerance_rad').value)
        settle_deadline = time.monotonic() + float(
            self.get_parameter('settle_timeout_sec').value
        )
        final = points[-1][1]
        max_error = math.inf
        while rclpy.ok() and time.monotonic() < settle_deadline:
            self._publish_command(final)
            actual = self._current_positions()
            if actual is not None:
                max_error = max(abs(a - b) for a, b in zip(final, actual))
                if max_error <= tolerance:
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    goal_handle.succeed()
                    return result
            time.sleep(1.0 / rate)

        result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
        result.error_string = f'final joint error {max_error:.4f} rad exceeds {tolerance:.4f}'
        goal_handle.abort()
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IsaacTrajectoryBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
