#!/usr/bin/env python3
"""Start one namespaced Nav2 stack in deterministic lifecycle order."""

from __future__ import annotations

import time

import rclpy
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node


class Nav2LifecycleSequencer(Node):
    """Serialize lifecycle startup to avoid concurrent Fast DDS service loss."""

    def __init__(self) -> None:
        super().__init__("nav2_lifecycle_sequencer")
        self.declare_parameter("service_wait_timeout_sec", 60.0)
        self.declare_parameter("startup_timeout_sec", 60.0)
        self._service_wait_timeout = float(
            self.get_parameter("service_wait_timeout_sec").value
        )
        self._startup_timeout = float(
            self.get_parameter("startup_timeout_sec").value
        )
        self._manager_names = (
            "lifecycle_manager_localization",
            "lifecycle_manager_navigation",
            "lifecycle_manager_collision_monitor",
        )
        self._clients = [
            self.create_client(
                ManageLifecycleNodes, f"{name}/manage_nodes"
            )
            for name in self._manager_names
        ]

    def start(self) -> bool:
        namespace = self.get_namespace()
        self.get_logger().info(
            f"sequential Nav2 lifecycle startup begins: namespace={namespace}"
        )
        for name, client in zip(self._manager_names, self._clients):
            self.get_logger().info(f"waiting for {name}/manage_nodes")
            if not client.wait_for_service(timeout_sec=self._service_wait_timeout):
                self.get_logger().error(
                    f"lifecycle service unavailable: {name}/manage_nodes"
                )
                return False

            request = ManageLifecycleNodes.Request()
            request.command = ManageLifecycleNodes.Request.STARTUP
            future = client.call_async(request)
            deadline = time.monotonic() + self._startup_timeout
            while rclpy.ok() and not future.done():
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    self.get_logger().error(
                        f"lifecycle startup timed out: {name}"
                    )
                    return False
                rclpy.spin_once(self, timeout_sec=min(0.2, remaining))

            try:
                response = future.result()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    f"lifecycle startup call failed: {name}: {exc}"
                )
                return False
            if response is None or not response.success:
                self.get_logger().error(
                    f"lifecycle startup rejected: {name}"
                )
                return False
            self.get_logger().info(f"lifecycle group active: {name}")

        self.get_logger().info("all Nav2 lifecycle groups are active")
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Nav2LifecycleSequencer()
    try:
        success = node.start()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
