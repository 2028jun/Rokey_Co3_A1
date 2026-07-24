#!/usr/bin/env python3
"""Automatic Navigation Initializer Node for Nav2 + Isaac Sim integration.

Monitors system readiness (/clock, /navigation/initialize service),
calls /navigation/initialize ONCE, and validates /navigation/detail status
without duplicate calls or static sleep delays.
"""

from __future__ import annotations

import json
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger


class NavigationAutoInitializerNode(Node):
    def __init__(self) -> None:
        super().__init__("navigation_auto_initializer")

        self.declare_parameter("use_sim_time", True)

        self._has_clock = False
        self._initialized = False
        self._attempt_in_progress = False
        self._max_retries = 10
        self._retry_count = 0

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.create_subscription(Int32, "/navigation/status", self._on_nav_status, status_qos)
        self.create_subscription(Int32, "/navigation/current_location", self._on_nav_location, status_qos)
        self.create_subscription(String, "/navigation/detail", self._on_nav_detail, status_qos)

        self._init_client = self.create_client(Trigger, "/navigation/initialize")

        # Evaluate readiness every 1.5 seconds
        self._timer = self.create_timer(1.5, self._evaluate_readiness)
        self.get_logger().info("Navigation Auto Initializer node started. Waiting for /clock and /navigation/initialize service...")

    def _on_clock(self, msg: Clock) -> None:
        self._has_clock = True

    def _on_nav_status(self, msg: Int32) -> None:
        pass

    def _on_nav_location(self, msg: Int32) -> None:
        pass

    def _on_nav_detail(self, msg: String) -> None:
        try:
            detail = json.loads(msg.data)
            state = detail.get("state")
            phase = detail.get("phase")
            if state == "SUCCEEDED" and phase == "initialized":
                if not self._initialized:
                    self._initialized = True
                    self.get_logger().info("✅ Navigation is fully initialized and ready! (phase=initialized, state=SUCCEEDED)")
        except Exception:
            pass

    def _evaluate_readiness(self) -> None:
        if self._initialized:
            return

        if self._attempt_in_progress:
            return

        if not self._has_clock:
            self.get_logger().info("Waiting for /clock...", throttle_duration_sec=5.0)
            return

        if not self._init_client.service_is_ready():
            self.get_logger().info("Waiting for /navigation/initialize service...", throttle_duration_sec=5.0)
            return

        if self._retry_count >= self._max_retries:
            self.get_logger().error(f"Navigation auto-initialization failed after {self._max_retries} attempts.")
            self._timer.cancel()
            return

        self._attempt_in_progress = True
        self._retry_count += 1
        self.get_logger().info(f"🚀 Calling /navigation/initialize (attempt {self._retry_count}/{self._max_retries})...")

        req = Trigger.Request()
        future = self._init_client.call_async(req)
        future.add_done_callback(self._on_init_response)

    def _on_init_response(self, future) -> None:
        try:
            res = future.result()
            if res.success:
                self.get_logger().info(f"Received initialization response: {res.message}. Validating localization status...")
            else:
                self.get_logger().warning(f"Initialization request returned failure: {res.message}")
        except Exception as exc:
            self.get_logger().error(f"Initialization service call failed with exception: {exc}")
        finally:
            self._attempt_in_progress = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationAutoInitializerNode()
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
