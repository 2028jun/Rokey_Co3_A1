#!/usr/bin/env python3
"""Automatic Navigation Initializer Node for Nav2 + Isaac Sim integration.

Monitors system readiness (/clock, /navigation/initialize service),
calls /navigation/initialize ONCE, and validates /navigation/detail status
without duplicate calls or static sleep delays.
"""

from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import Trigger


class NavigationAutoInitializerNode(Node):
    def __init__(self) -> None:
        super().__init__("navigation_auto_initializer")

        # use_sim_time comes from launch params; do not redeclare
        # (ParameterAlreadyDeclaredException kills the node at startup).

        self._has_clock = False
        self._nav2_ready = False
        self._initialized = False
        self._attempt_in_progress = False
        self._attempt_deadline = None
        self._max_retries = 10
        self._retry_count = 0
        self._attempt_timeout_sec = float(
            self.declare_parameter("initialization_timeout_sec", 90.0).value
        )

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.create_subscription(
            Bool, "nav2/lifecycle_ready", self._on_nav2_ready, status_qos
        )
        self.create_subscription(Int32, "navigation/status", self._on_nav_status, status_qos)
        self.create_subscription(Int32, "navigation/current_location", self._on_nav_location, status_qos)
        self.create_subscription(String, "navigation/detail", self._on_nav_detail, status_qos)

        self._init_client = self.create_client(Trigger, "navigation/initialize")

        # Evaluate readiness every 1.5 seconds
        self._timer = self.create_timer(1.5, self._evaluate_readiness)
        self.get_logger().info("Navigation Auto Initializer node started. Waiting for /clock and /navigation/initialize service...")

    def _on_clock(self, msg: Clock) -> None:
        self._has_clock = True

    def _on_nav2_ready(self, msg: Bool) -> None:
        if msg.data and not self._nav2_ready:
            self.get_logger().info(
                "Nav2 lifecycle nodes are verified active; initialization enabled"
            )
        self._nav2_ready = bool(msg.data)

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
                    self._attempt_in_progress = False
                    self._attempt_deadline = None
                    self.get_logger().info("✅ Navigation is fully initialized and ready! (phase=initialized, state=SUCCEEDED)")
            elif state == "FAILED" and self._attempt_in_progress:
                self._attempt_in_progress = False
                self._attempt_deadline = None
                reason = detail.get("reason", "unknown failure")
                self.get_logger().warning(
                    f"Navigation initialization failed: {reason}. Retrying..."
                )
        except Exception:
            pass

    def _evaluate_readiness(self) -> None:
        if self._initialized:
            return

        if self._attempt_in_progress:
            if (
                self._attempt_deadline is None
                or time.monotonic() < self._attempt_deadline
            ):
                return
            self.get_logger().warning(
                "Navigation initialization is still running; continuing to "
                "wait instead of submitting a duplicate request"
            )
            self._attempt_deadline = time.monotonic() + self._attempt_timeout_sec
            return

        if not self._has_clock:
            self.get_logger().info("Waiting for /clock...", throttle_duration_sec=5.0)
            return

        if not self._nav2_ready:
            self.get_logger().info(
                "Waiting for verified Nav2 lifecycle activation...",
                throttle_duration_sec=5.0,
            )
            return

        if not self._init_client.service_is_ready():
            self.get_logger().info("Waiting for /navigation/initialize service...", throttle_duration_sec=5.0)
            return

        if self._retry_count >= self._max_retries:
            self.get_logger().error(f"Navigation auto-initialization failed after {self._max_retries} attempts.")
            self._timer.cancel()
            return

        self._attempt_in_progress = True
        self._attempt_deadline = time.monotonic() + self._attempt_timeout_sec
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
                message = str(res.message or "")
                if "busy" in message.lower():
                    # A previously accepted initialization worker still owns
                    # the subsystem. Do not burn retries or submit requests in
                    # a loop; its detail event will complete this attempt.
                    self.get_logger().warning(
                        "Navigation worker is busy; waiting for its status "
                        "instead of counting another retry"
                    )
                    self._attempt_in_progress = True
                    self._attempt_deadline = (
                        time.monotonic() + self._attempt_timeout_sec
                    )
                else:
                    self.get_logger().warning(
                        f"Initialization request returned failure: {message}"
                    )
                    self._attempt_in_progress = False
                    self._attempt_deadline = None
        except Exception as exc:
            self.get_logger().error(f"Initialization service call failed with exception: {exc}")
            self._attempt_in_progress = False
            self._attempt_deadline = None


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
