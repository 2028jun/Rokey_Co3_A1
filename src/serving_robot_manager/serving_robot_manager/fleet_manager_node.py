"""Dispatch global HMI orders to one of two namespaced robot managers."""

from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger

from serving_robot_interfaces.srv import OrderRequest


IDLE, COMPLETED, FAILED = 0, 6, 7
# Treat unknown (None) as dispatchable so a robot that has not yet latched
# status is not permanently skipped while its peer is busy.
_DISPATCHABLE = (None, IDLE, COMPLETED)
# Guard against double-dispatch before status leaves IDLE. Auto-expires so a
# missed status transition cannot permanently block an idle robot.
_RESERVE_SEC = 3.0
OCCUPYING_PHASES = frozenset(
    {
        "approaching",
        "serving",
        "parking_out",
        "occupying",
        "holding",
        "returning",
    }
)


class FleetManager(Node):
    def __init__(self) -> None:
        super().__init__("fleet_manager")
        self.declare_parameter("robot_names", ["robot1", "robot2"])
        # Legacy switch. Concurrent idle-robot dispatch is always allowed;
        # this flag is kept only for log compatibility and must not block.
        self.declare_parameter("serialize_shared_payloads", False)
        self._robots = list(self.get_parameter("robot_names").value)
        self._serialize_shared_payloads = bool(
            self.get_parameter("serialize_shared_payloads").value
        )
        if self._serialize_shared_payloads:
            self.get_logger().warning(
                "serialize_shared_payloads=true is ignored: idle robots "
                "always accept orders while a peer is busy"
            )
        self._states = {name: None for name in self._robots}
        # robot -> monotonic timestamp of in-flight / just-accepted dispatch
        self._reserved: dict[str, float] = {}
        # robot_id -> {table_id, phase}
        self._table_claims: dict[str, dict] = {}
        # Do not name this `_clients` — that shadows rclpy.Node._clients.
        self._order_clients = {
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
        self.create_subscription(
            String, "/fleet/table_occupancy", self._on_table_occupancy, qos
        )
        self._status_pub = self.create_publisher(Int32, "/system/status", qos)
        self.create_service(OrderRequest, "/manager/order", self._on_order)
        self.create_service(Trigger, "/manager/reset_fault", self._on_reset)
        self.get_logger().info(f"fleet workers ready: {self._robots}")

    def _on_table_occupancy(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"bad table occupancy: {exc}")
            return
        robot = str(payload.get("robot_id", "")).strip()
        if not robot:
            return
        phase = str(payload.get("phase", "clear")).strip().lower()
        raw_table = payload.get("table_id")
        if phase == "clear" or phase not in OCCUPYING_PHASES:
            self._table_claims.pop(robot, None)
            return
        if raw_table is None or robot not in self._robots:
            return
        try:
            table_id = int(raw_table)
        except (TypeError, ValueError):
            return
        self._table_claims[robot] = {"table_id": table_id, "phase": phase}

    def _table_owner(self, table_id: int) -> str | None:
        hard = ("serving", "parking_out", "occupying", "approaching")
        for robot, info in self._table_claims.items():
            if int(info.get("table_id", -1)) != int(table_id):
                continue
            if str(info.get("phase", "")).lower() in hard:
                return robot
        return None

    def _on_status(self, robot: str, msg: Int32) -> None:
        self._states[robot] = int(msg.data)
        # Job started (or failed): reservation no longer needed.
        if msg.data not in (IDLE, COMPLETED):
            self._reserved.pop(robot, None)
        active = [
            s for s in self._states.values() if s not in (None, IDLE, COMPLETED)
        ]
        self._status_pub.publish(Int32(data=max(active) if active else IDLE))

    def _is_reserved(self, robot: str) -> bool:
        started = self._reserved.get(robot)
        if started is None:
            return False
        if time.monotonic() - started > _RESERVE_SEC:
            self._reserved.pop(robot, None)
            return False
        return True

    @staticmethod
    def _copy_order(src, dst) -> None:
        for field in (
            "table_id", "pizza1_count", "pizza2_count", "pizza3_count",
            "drink_count", "cutlery_count", "plate_count", "preferred_robot",
        ):
            if hasattr(src, field):
                setattr(dst, field, getattr(src, field))

    @staticmethod
    def _normalize_preferred(raw: str) -> str:
        name = str(raw or "").strip().lower()
        if name in ("", "auto", "any"):
            return ""
        return name

    def _idle_candidates(self) -> list[str]:
        out = []
        for name in self._robots:
            state = self._states.get(name)
            ready = self._order_clients[name].service_is_ready()
            if state not in _DISPATCHABLE:
                continue
            if self._is_reserved(name):
                continue
            if not ready:
                self.get_logger().warning(
                    f"{name} idle but /manager/order not ready "
                    f"(state={state})"
                )
                continue
            out.append(name)
        return out

    def _on_order(self, request, response):
        response.assigned_robot = ""

        preferred = self._normalize_preferred(
            getattr(request, "preferred_robot", "")
        )
        candidates = self._idle_candidates()
        if preferred:
            if preferred not in self._robots:
                self.get_logger().warning(
                    f"unknown preferred_robot={preferred!r}"
                )
                response.success = False
                return response
            if preferred not in candidates:
                self.get_logger().warning(
                    f"preferred robot unavailable: {preferred} "
                    f"states={self._states} "
                    f"reserved={sorted(self._reserved)} "
                    f"candidates={candidates}"
                )
                response.success = False
                return response
            candidates = [preferred]

        if not candidates:
            self.get_logger().warning(
                f"no idle robot: states={self._states} "
                f"reserved={sorted(self._reserved)}"
            )
            response.success = False
            return response

        robot = candidates[0]
        owner = self._table_owner(int(request.table_id))
        if owner and owner != robot:
            self.get_logger().warning(
                f"table {request.table_id} occupied by {owner}; "
                f"dispatching {robot} (peer will hold at kitchen)"
            )
        self._reserved[robot] = time.monotonic()
        child_request = OrderRequest.Request()
        self._copy_order(request, child_request)
        # Workers ignore preferred_robot; clear to keep request local.
        child_request.preferred_robot = ""
        future = self._order_clients[robot].call_async(child_request)

        def completed(done):
            try:
                accepted = bool(done.result().success)
            except Exception as exc:  # noqa: BLE001
                self._reserved.pop(robot, None)
                self.get_logger().error(f"{robot} dispatch failed: {exc}")
                return
            if not accepted:
                self._reserved.pop(robot, None)
            self.get_logger().info(
                f"order table={request.table_id} robot={robot} "
                f"accepted={accepted}"
            )

        future.add_done_callback(completed)
        response.success = True
        response.assigned_robot = robot
        return response

    def _on_reset(self, _request, response):
        targets = [
            name for name in self._robots
            if self._states[name] == FAILED
            and self._reset_clients[name].service_is_ready()
        ]
        self._reserved.clear()
        for name in targets:
            self._reset_clients[name].call_async(Trigger.Request())
        response.success = bool(targets)
        response.message = (
            f"reset requested: {targets}" if targets else "no failed robot"
        )
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
