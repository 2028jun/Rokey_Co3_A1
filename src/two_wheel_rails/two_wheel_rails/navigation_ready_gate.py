#!/usr/bin/env python3
"""Exit successfully after one namespaced navigation worker is initialized."""

from __future__ import annotations

import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class NavigationReadyGate(Node):
    def __init__(self) -> None:
        super().__init__("navigation_ready_gate")
        self.declare_parameter("detail_topic", "/robot1/navigation/detail")
        self.declare_parameter("timeout_sec", 180.0)
        self._ready = False
        self._failed = False
        self._deadline = time.monotonic() + float(
            self.get_parameter("timeout_sec").value
        )
        topic = str(self.get_parameter("detail_topic").value)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, topic, self._on_detail, qos)
        self.create_timer(0.5, self._check_timeout)
        self.get_logger().info(f"waiting for initialized navigation: {topic}")

    @property
    def exit_code(self) -> int:
        return 0 if self._ready else 1

    @property
    def finished(self) -> bool:
        return self._ready or self._failed

    def _on_detail(self, msg: String) -> None:
        try:
            detail = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        state = str(detail.get("state", "")).upper()
        phase = str(detail.get("phase", "")).lower()
        if state == "SUCCEEDED" and phase == "initialized":
            self._ready = True
            self.get_logger().info("navigation initialized; releasing next stack")
        elif state == "FAILED" and phase in ("initialize", "initialized"):
            self._failed = True
            self.get_logger().error(
                f"navigation initialization failed: {detail.get('reason', '')}"
            )

    def _check_timeout(self) -> None:
        if time.monotonic() >= self._deadline:
            self._failed = True
            self.get_logger().error("navigation readiness gate timed out")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationReadyGate()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.5)
        code = node.exit_code
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
