"""Navigation Service Server Node for Pizza Robot Manager Integration.

Listens for /navigation/command service calls (Table 1, 2, 3 or Kitchen=4),
controls Odom-Rail navigation, and publishes status to /navigation/status and /navigation/current_location.
"""

from __future__ import annotations

import math
import os
import threading
import time
import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int32
from serving_robot_interfaces.srv import TaskCommand

from nav_robot_missions.rail_mission import RailMissionNode, resolve_config

NAV_CMD_KITCHEN = 4
NAV_CMD_RESUME = 98
NAV_CMD_PAUSE = 99

NAV_STATUS_MOVING = 1
NAV_STATUS_ARRIVED = 2
NAV_STATUS_FAILED = 3

NAV_LOCATION_KITCHEN = 4


class NavServerNode(RailMissionNode):
    def __init__(self):
        routes_path = resolve_config("routes.yaml", None)
        super().__init__(
            "nav_server_node",
            table_id=4,
            routes_path=routes_path,
            linear_speed=0.50,
            angular_speed=0.55,
            dock_speed=0.16,
            xy_tolerance=0.12,
            yaw_tolerance=0.12,
            lat_tolerance=0.12,
            segment_timeout=150.0,
            stall_timeout=6.0,
        )

        # Publishers for Manager Node
        self.status_pub = self.create_publisher(Int32, '/navigation/status', 10)
        self.location_pub = self.create_publisher(Int32, '/navigation/current_location', 10)

        # Service for Manager Node
        self.create_service(TaskCommand, '/navigation/command', self._on_nav_command)

        # State management
        self.current_location = NAV_LOCATION_KITCHEN
        self.is_navigating = False
        self.paused = False

        self.get_logger().info("NavServerNode initialized. Active on /navigation/command service.")

        # Spawn/Teleport robot to Kitchen position upon startup
        threading.Thread(target=self._initial_spawn, daemon=True).start()

    def _tick(self, dt: float = 0.05):
        """Override _tick to avoid spin_once conflict with MultiThreadedExecutor."""
        time.sleep(dt)

    def _initial_spawn(self):
        time.sleep(1.5)
        self.get_logger().info("Spawning/positioning robot in front of kitchen entry (x=0.00, y=5.25)...")
        if self._wait_odom(timeout=10.0):
            self.teleport_start(timeout=5.0)
        self.location_pub.publish(Int32(data=NAV_LOCATION_KITCHEN))
        self.status_pub.publish(Int32(data=NAV_STATUS_ARRIVED))

    def _on_nav_command(self, request, response):
        cmd = request.command
        self.get_logger().info(f"[NavServerNode] Received /navigation/command: {cmd}")

        if cmd in (1, 2, 3, 4):
            response.success = True
            threading.Thread(target=self._execute_navigation, args=(cmd,), daemon=True).start()
            return response
        elif cmd == NAV_CMD_PAUSE:
            self.paused = True
            response.success = True
            return response
        elif cmd == NAV_CMD_RESUME:
            self.paused = False
            response.success = True
            return response
        else:
            self.get_logger().warn(f"Unknown navigation command: {cmd}")
            response.success = False
            return response

    def _execute_navigation(self, target_id: int):
        if self.is_navigating:
            self.get_logger().warn("Navigation already in progress, queuing new mission...")
            while self.is_navigating and rclpy.ok():
                time.sleep(0.1)

        self.is_navigating = True
        self.table_id = target_id
        self.status_pub.publish(Int32(data=NAV_STATUS_MOVING))
        self.get_logger().info(f"Started navigation to target_id={target_id}...")

        try:
            if not self._wait_odom(timeout=10.0):
                self.get_logger().error("No /nav_robot/odom received within timeout!")
                self.status_pub.publish(Int32(data=NAV_STATUS_FAILED))
                return

            success = False
            if target_id in (1, 2, 3):
                # Outbound mission from Kitchen to Table target
                route = self._route(outbound=True)
                success = self.follow_outbound(route)
            elif target_id == NAV_CMD_KITCHEN:
                # Inbound mission from Table to Kitchen
                route = self._route(outbound=False)
                success = self.follow_return(route)

            if success:
                self.current_location = target_id
                self.get_logger().info(f"Successfully reached target_id={target_id}!")
                self.location_pub.publish(Int32(data=target_id))
                self.status_pub.publish(Int32(data=NAV_STATUS_ARRIVED))
            else:
                self.get_logger().error(f"Navigation to target_id={target_id} failed!")
                self.status_pub.publish(Int32(data=NAV_STATUS_FAILED))

        except Exception as e:
            self.get_logger().error(f"Navigation exception: {e}")
            self.status_pub.publish(Int32(data=NAV_STATUS_FAILED))
        finally:
            self._stop()
            self.is_navigating = False


def main(args=None):
    rclpy.init(args=args)
    node = NavServerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
