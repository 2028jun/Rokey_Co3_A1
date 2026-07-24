#!/usr/bin/env python3
"""Explicit one-shot client for the navigation spawn initialization service."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("navigation_initialize_client")
    client = node.create_client(Trigger, "/navigation/initialize")
    try:
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("/navigation/initialize service unavailable")
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=15.0)
        if future.result() is None or not future.result().success:
            message = future.result().message if future.result() else "timeout"
            raise RuntimeError(f"initialization rejected: {message}")
        node.get_logger().info(future.result().message)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
