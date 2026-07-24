"""Isaac Subsystem Adapter Node for ROS 2 Humble (Python 3.10).

Bridges Manager's TaskCommand services (/arm/command, /food_spawn/command)
to Isaac Sim's std_msgs/Int32 trigger & status topics, overcoming Python 3.10/3.11
custom interface import limitations in Isaac Sim's embedded environment.
"""

from __future__ import annotations

import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Int32
from serving_robot_interfaces.srv import TaskCommand


class IsaacSubsystemAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("isaac_subsystem_adapter_node")

        self._arm_trigger_pub = self.create_publisher(Int32, "/arm/trigger", 10)
        self._spawn_trigger_pub = self.create_publisher(Int32, "/food_spawn/trigger", 10)

        self._arm_status_sub = self.create_subscription(
            Int32, "/arm/status", self._on_arm_status, 10
        )
        self._spawn_status_sub = self.create_subscription(
            Int32, "/food_spawn/status", self._on_spawn_status, 10
        )

        self._arm_event = threading.Event()
        self._arm_last_status = 0
        self._spawn_event = threading.Event()
        self._spawn_last_status = 0

        self.create_service(TaskCommand, "/arm/command", self._on_arm_command)
        self.create_service(TaskCommand, "/food_spawn/command", self._on_spawn_command)

        self.get_logger().info(
            "Isaac Subsystem Adapter ready: providing /arm/command and /food_spawn/command"
        )

    def _on_arm_status(self, msg: Int32) -> None:
        self._arm_last_status = msg.data
        if msg.data in (2, 3):  # 2 = COMPLETED, 3 = FAILED
            self._arm_event.set()

    def _on_spawn_status(self, msg: Int32) -> None:
        self._spawn_last_status = msg.data
        if msg.data in (2, 3):  # 2 = COMPLETED, 3 = FAILED
            self._spawn_event.set()

    def _on_arm_command(self, request: TaskCommand.Request, response: TaskCommand.Response) -> TaskCommand.Response:
        command = int(request.command)
        if command <= 0:
            self.get_logger().warning(f"Invalid arm command: {command}")
            response.success = False
            return response

        self._arm_last_status = 0
        self._arm_event.clear()
        self._arm_trigger_pub.publish(Int32(data=command))
        self.get_logger().info(f"Forwarded arm command={command} to /arm/trigger, waiting for /arm/status...")

        finished = self._arm_event.wait(timeout=60.0)
        if finished and self._arm_last_status == 2:
            self.get_logger().info(f"Arm command={command} completed successfully in Isaac Sim")
            response.success = True
        else:
            self.get_logger().error(f"Arm command={command} failed or timed out (status={self._arm_last_status})")
            response.success = False
        return response

    def _on_spawn_command(self, request: TaskCommand.Request, response: TaskCommand.Response) -> TaskCommand.Response:
        command = int(request.command)
        if command <= 0:
            self.get_logger().warning(f"Invalid food spawn command: {command}")
            response.success = False
            return response

        self._spawn_last_status = 0
        self._spawn_event.clear()
        self._spawn_trigger_pub.publish(Int32(data=command))
        self.get_logger().info(f"Forwarded food spawn command={command} to /food_spawn/trigger, waiting for /food_spawn/status...")

        finished = self._spawn_event.wait(timeout=15.0)
        if finished and self._spawn_last_status == 2:
            self.get_logger().info(f"Food spawn command={command} completed successfully in Isaac Sim")
            response.success = True
        else:
            self.get_logger().error(f"Food spawn command={command} failed or timed out (status={self._spawn_last_status})")
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
