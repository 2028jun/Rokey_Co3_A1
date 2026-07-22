"""Manager service adapters for Isaac's direct simulation controllers.

This node does not run Nav2, plan paths, or publish velocity commands.  It only
forwards manager-compatible service requests to the simulation thread that
owns the articulation.  Topic forwarding is required because Isaac Sim 5.1's
embedded Python 3.11 cannot always load ROS Humble's Python 3.10 custom-service
type support, while ordinary ``std_msgs/Int32`` topics remain available.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

from serving_robot_interfaces.srv import TaskCommand


class DirectNavServer(Node):
    def __init__(self):
        super().__init__("direct_nav_server_node")
        self._trigger_pub = self.create_publisher(Int32, "/navigation/trigger", 10)
        self._arm_trigger_pub = self.create_publisher(Int32, "/arm/trigger", 10)
        self._spawn_trigger_pub = self.create_publisher(
            Int32, "/food_spawn/trigger", 10
        )
        self.create_service(
            TaskCommand, "/navigation/command", self._on_command
        )
        self.create_service(TaskCommand, "/arm/command", self._on_arm_command)
        self.create_service(
            TaskCommand, "/food_spawn/command", self._on_spawn_command
        )
        self.get_logger().info(
            "Direct Isaac adapters ready: navigation, arm, food spawn"
        )

    def _on_command(self, request, response):
        target = int(request.command)
        if target not in (0, 1, 2, 3, 4, 98, 99):
            self.get_logger().warning(f"unknown navigation command: {target}")
            response.success = False
            return response
        self._trigger_pub.publish(Int32(data=target))
        self.get_logger().info(f"forwarded direct navigation target={target}")
        response.success = True
        return response

    def _on_arm_command(self, request, response):
        command = int(request.command)
        if command <= 0:
            self.get_logger().warning(f"invalid arm command: {command}")
            response.success = False
            return response
        self._arm_trigger_pub.publish(Int32(data=command))
        self.get_logger().info(f"forwarded arm command={command}")
        response.success = True
        return response

    def _on_spawn_command(self, request, response):
        command = int(request.command)
        if command <= 0:
            self.get_logger().warning(f"invalid food spawn command: {command}")
            response.success = False
            return response
        self._spawn_trigger_pub.publish(Int32(data=command))
        self.get_logger().info(f"forwarded food spawn command={command}")
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DirectNavServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
