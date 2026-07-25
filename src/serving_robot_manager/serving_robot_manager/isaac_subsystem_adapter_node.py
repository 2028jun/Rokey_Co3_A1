"""Isaac Subsystem Adapter Node for ROS 2 Humble (Python 3.10).

Bridges Manager's TaskCommand services (/arm/command, /food_spawn/command)
to Isaac Sim's std_msgs/Int32 trigger & status topics, overcoming Python 3.10/3.11
custom interface import limitations in Isaac Sim's embedded environment.
"""

from __future__ import annotations

import threading
import time
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32
from serving_robot_interfaces.srv import TaskCommand


class IsaacSubsystemAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("isaac_subsystem_adapter_node")

        # Allow concurrent execution of service requests and status subscriptions
        self._cb_group = ReentrantCallbackGroup()

        status_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        command_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # Commands are events, not state.  TRANSIENT_LOCAL replays stale
        # commands whenever Isaac reconnects and can spawn/serve the same
        # payload repeatedly.  Only status topics should be latched.
        self._arm_trigger_pub = self.create_publisher(Int32, "/arm/trigger", command_qos)
        self._spawn_trigger_pub = self.create_publisher(Int32, "/food_spawn/trigger", command_qos)

        self._arm_status_sub = self.create_subscription(
            Int32, "/arm/status", self._on_arm_status, status_qos, callback_group=self._cb_group
        )
        self._spawn_status_sub = self.create_subscription(
            Int32, "/food_spawn/status", self._on_spawn_status, status_qos, callback_group=self._cb_group
        )

        self._arm_cond = threading.Condition()
        self._arm_last_status = 0
        self._spawn_cond = threading.Condition()
        self._spawn_last_status = 0

        self.create_service(
            TaskCommand, "/arm/command", self._on_arm_command, callback_group=self._cb_group
        )
        self.create_service(
            TaskCommand, "/food_spawn/command", self._on_spawn_command, callback_group=self._cb_group
        )

        self.get_logger().info(
            "Isaac Subsystem Adapter ready: providing /arm/command and /food_spawn/command with ReentrantCallbackGroup"
        )

    def _on_arm_status(self, msg: Int32) -> None:
        self.get_logger().info(f"[ArmStatus RX] value={msg.data}")
        with self._arm_cond:
            self._arm_last_status = msg.data
            self._arm_cond.notify_all()

    def _on_spawn_status(self, msg: Int32) -> None:
        self.get_logger().info(f"[FoodStatus RX] value={msg.data}")
        with self._spawn_cond:
            self._spawn_last_status = msg.data
            self._spawn_cond.notify_all()

    def _on_arm_command(self, request: TaskCommand.Request, response: TaskCommand.Response) -> TaskCommand.Response:
        command = int(request.command)
        if command <= 0:
            self.get_logger().warning(f"Invalid arm command: {command}")
            response.success = False
            return response

        self.get_logger().info(f"Arm command={command} started")

        # Arm service is acceptance-only. Manager separately observes
        # /arm/status WORKING -> COMPLETED/FAILED for actual task completion.
        # Waiting here duplicates that responsibility and caused long integrated
        # serving commands (pizza -> soda -> cutlery) to fail at the adapter's
        # fixed 60 second timeout while Isaac was still operating normally.
        self._arm_trigger_pub.publish(Int32(data=command))
        if command in (98, 99):
            self.get_logger().info(
                f"Forwarded arm safety command={command} to /arm/trigger; "
                "returning acceptance immediately"
            )
        else:
            self.get_logger().info(
                f"Forwarded arm command={command} to /arm/trigger; "
                "returning acceptance immediately"
            )
        response.success = True
        return response

    def _on_spawn_command(self, request: TaskCommand.Request, response: TaskCommand.Response) -> TaskCommand.Response:
        command = int(request.command)
        if command <= 0:
            self.get_logger().warning(f"Invalid food spawn command: {command}")
            response.success = False
            return response

        self.get_logger().info(f"Food command={command} started")
        with self._spawn_cond:
            self._spawn_last_status = 0
            self._spawn_trigger_pub.publish(Int32(data=command))
            self.get_logger().info(f"Forwarded food spawn command={command} to /food_spawn/trigger, waiting for status transition...")

            start_time = time.monotonic()
            timeout_sec = 20.0
            while time.monotonic() - start_time < timeout_sec:
                if self._spawn_last_status == 2:  # 2 = COMPLETED
                    self.get_logger().info(f"Food status transition -> COMPLETED (Food command={command} succeeded)")
                    response.success = True
                    return response
                elif self._spawn_last_status == 3:  # 3 = FAILED
                    self.get_logger().error(f"Food status transition -> FAILED (Food command={command})")
                    response.success = False
                    return response

                # Wait on condition variable for new status notifications
                remaining = timeout_sec - (time.monotonic() - start_time)
                if remaining <= 0 or not self._spawn_cond.wait(timeout=min(1.0, remaining)):
                    pass

        self.get_logger().error(f"Food spawn command={command} timed out after {timeout_sec}s (last_status={self._spawn_last_status})")
        response.success = False
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IsaacSubsystemAdapterNode()
    executor = MultiThreadedExecutor()
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


if __name__ == "__main__":
    main()
