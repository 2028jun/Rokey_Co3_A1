"""Dispatch global HMI orders to one of two namespaced robot managers."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32
from std_srvs.srv import Trigger

from serving_robot_interfaces.srv import OrderRequest


IDLE, COMPLETED, FAILED = 0, 6, 7


class FleetManager(Node):
    def __init__(self) -> None:
        super().__init__("fleet_manager")
        self.declare_parameter("robot_names", ["robot1", "robot2"])
        self.declare_parameter("serialize_shared_payloads", True)
        self._robots = list(self.get_parameter("robot_names").value)
        self._serialize_shared_payloads = bool(
            self.get_parameter("serialize_shared_payloads").value
        )
        self._states = {name: None for name in self._robots}
        self._reserved = set()
        self._clients = {
            name: self.create_client(OrderRequest, f"/{name}/manager/order")
            for name in self._robots
        }
        self._reset_clients = {
            name: self.create_client(Trigger, f"/{name}/manager/reset_fault")
            for name in self._robots
        }
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        for name in self._robots:
            self.create_subscription(
                Int32,
                f"/{name}/system/status",
                lambda msg, robot=name: self._on_status(robot, msg),
                qos,
            )
        self._status_pub = self.create_publisher(Int32, "/system/status", qos)
        self.create_service(OrderRequest, "/manager/order", self._on_order)
        self.create_service(Trigger, "/manager/reset_fault", self._on_reset)
        self.get_logger().info(f"fleet workers ready: {self._robots}")

    def _on_status(self, robot: str, msg: Int32) -> None:
        self._states[robot] = int(msg.data)
        if msg.data not in (IDLE, COMPLETED):
            self._reserved.discard(robot)
        active = [s for s in self._states.values() if s not in (None, IDLE, COMPLETED)]
        self._status_pub.publish(Int32(data=max(active) if active else IDLE))

    @staticmethod
    def _copy_order(src, dst) -> None:
        for field in (
            "table_id", "pizza1_count", "pizza2_count", "pizza3_count",
            "drink_count", "cutlery_count", "plate_count",
        ):
            setattr(dst, field, getattr(src, field))

    def _on_order(self, request, response):
        fleet_active = any(
            state not in (None, IDLE, COMPLETED)
            for state in self._states.values()
        )
        if self._serialize_shared_payloads and (fleet_active or self._reserved):
            self.get_logger().warning(
                "shared payload workspace busy; order deferred "
                f"states={self._states} reserved={sorted(self._reserved)}"
            )
            response.success = False
            return response
        candidates = [
            name for name in self._robots
            if self._states[name] in (IDLE, COMPLETED)
            and name not in self._reserved
            and self._clients[name].service_is_ready()
        ]
        if not candidates:
            self.get_logger().warning(f"no idle robot: states={self._states}")
            response.success = False
            return response
        robot = candidates[0]
        self._reserved.add(robot)
        child_request = OrderRequest.Request()
        self._copy_order(request, child_request)
        future = self._clients[robot].call_async(child_request)

        def completed(done):
            try:
                accepted = bool(done.result().success)
            except Exception as exc:  # noqa: BLE001
                self._reserved.discard(robot)
                self.get_logger().error(f"{robot} dispatch failed: {exc}")
                return
            if not accepted:
                self._reserved.discard(robot)
            self.get_logger().info(
                f"order table={request.table_id} robot={robot} accepted={accepted}"
            )

        future.add_done_callback(completed)
        # Acceptance here means assignment was made. Worker completion remains
        # observable on /<robot>/system/status.
        response.success = True
        return response

    def _on_reset(self, _request, response):
        targets = [
            name for name in self._robots
            if self._states[name] == FAILED and self._reset_clients[name].service_is_ready()
        ]
        for name in targets:
            self._reset_clients[name].call_async(Trigger.Request())
        response.success = bool(targets)
        response.message = f"reset requested: {targets}" if targets else "no failed robot"
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FleetManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
