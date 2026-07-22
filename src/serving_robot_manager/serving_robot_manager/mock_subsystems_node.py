"""Mock Subsystems Node for HMI & Manager Node integration testing.

Simulates responses and status updates for Arm, Navigation, and Food Spawner nodes.
"""

import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
from serving_robot_interfaces.srv import TaskCommand


class MockSubsystemsNode(Node):
    def __init__(self):
        super().__init__('mock_subsystems_node')

        # Publishers
        self.nav_status_pub = self.create_publisher(Int32, '/navigation/status', 10)
        self.nav_location_pub = self.create_publisher(Int32, '/navigation/current_location', 10)
        self.arm_status_pub = self.create_publisher(Int32, '/arm/status', 10)
        self.spawn_status_pub = self.create_publisher(Int32, '/food_spawn/status', 10)
        self.hand_safety_pub = self.create_publisher(Bool, '/hand_safety/intrusion', 10)
        self.spawn_trigger_pub = self.create_publisher(Int32, '/food_spawn/trigger', 10)
        self.arm_trigger_pub = self.create_publisher(Int32, '/arm/trigger', 10)

        # Periodic hand safety publisher (False = no hand intrusion)
        self.create_timer(0.2, self._publish_hand_safety)

        # Services
        self.create_service(TaskCommand, '/navigation/command', self._on_nav_cmd)
        self.create_service(TaskCommand, '/arm/command', self._on_arm_cmd)
        self.create_service(TaskCommand, '/food_spawn/command', self._on_spawn_cmd)

        self.get_logger().info("Mock Subsystems Node started with Hand Safety heartbeat. Services active.")

    def _publish_hand_safety(self):
        self.hand_safety_pub.publish(Bool(data=False))

    def _on_nav_cmd(self, request, response):
        cmd = request.command
        self.get_logger().info(f"[Mock Nav] Received command: {cmd}")
        response.success = True

        # Simulate navigation workflow in background
        def run_nav():
            time.sleep(0.1)
            # Publish MOVING (1)
            self.nav_status_pub.publish(Int32(data=1))
            time.sleep(1.0)
            # Publish ARRIVED (2)
            self.nav_status_pub.publish(Int32(data=2))
            if cmd == 4:  # NAV_CMD_KITCHEN
                self.nav_location_pub.publish(Int32(data=4))
            else:
                self.nav_location_pub.publish(Int32(data=cmd))

        threading.Thread(target=run_nav, daemon=True).start()
        return response

    def _on_arm_cmd(self, request, response):
        cmd = request.command
        self.get_logger().info(f"[Mock Arm] Received command: {cmd} -> Triggering Isaac Sim Arm Serving & Sliding Tray")
        response.success = True

        def run_arm():
            self.arm_trigger_pub.publish(Int32(data=cmd))

        threading.Thread(target=run_arm, daemon=True).start()
        return response

        # Service Bridge Publishers
        self.spawn_trigger_pub = self.create_publisher(Int32, '/food_spawn/trigger', 10)

    def _on_spawn_cmd(self, request, response):
        cmd = request.command
        self.get_logger().info(f"[Mock Spawn] Received command: {cmd} -> Triggering 3D Isaac Sim Food Spawn")
        response.success = True

        def run_spawn():
            self.spawn_trigger_pub.publish(Int32(data=cmd))

        threading.Thread(target=run_spawn, daemon=True).start()
        return response


def main():
    rclpy.init()
    node = MockSubsystemsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
