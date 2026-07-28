#!/usr/bin/env python3
"""Bring up one namespaced Nav2 stack without fragile manager fan-out calls."""

from __future__ import annotations

import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


LOCALIZATION_NODES = ("map_server", "amcl")
NAVIGATION_NODES = (
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "velocity_smoother",
)
COLLISION_NODES = ("collision_monitor",)


class Nav2LifecycleSequencer(Node):
    """Transition Nav2 nodes directly and verify their actual lifecycle state.

    Nav2's lifecycle manager sends a series of change-state service calls. On
    the loaded two-robot Fast DDS graph, a transition that takes slightly over
    the middleware response lifetime can complete at the server while its
    response is discarded. The manager then waits forever. This sequencer
    treats the node's reported lifecycle state as authoritative, so a lost
    response cannot strand the rest of the stack.
    """

    def __init__(self) -> None:
        super().__init__("nav2_lifecycle_sequencer")
        # A single 60 second wait made robot2 look dead whenever its first
        # Fast DDS service discovery window was missed. Short probes plus the
        # idempotent outer retry continue startup as soon as each service is
        # visible, without reconfiguring nodes that are already active.
        self.declare_parameter("service_wait_timeout_sec", 2.0)
        self.declare_parameter("transition_timeout_sec", 5.0)
        # Costmap activation can legitimately block while the first map->base
        # transform catches up to simulation time. Keep polling the node state
        # long enough for that transition instead of issuing it twice.
        self.declare_parameter("state_wait_timeout_sec", 30.0)
        self.declare_parameter("transition_retries", 3)
        self.declare_parameter("startup_retries", 90)
        self.declare_parameter("startup_retry_delay_sec", 1.0)
        self._service_wait_timeout = float(
            self.get_parameter("service_wait_timeout_sec").value
        )
        self._transition_timeout = float(
            self.get_parameter("transition_timeout_sec").value
        )
        self._state_wait_timeout = float(
            self.get_parameter("state_wait_timeout_sec").value
        )
        self._transition_retries = int(
            self.get_parameter("transition_retries").value
        )
        self._startup_retries = int(
            self.get_parameter("startup_retries").value
        )
        self._startup_retry_delay = float(
            self.get_parameter("startup_retry_delay_sec").value
        )
        if self._service_wait_timeout <= 0.0:
            raise ValueError("service_wait_timeout_sec must be greater than zero")
        if self._startup_retries < 1:
            raise ValueError("startup_retries must be at least 1")
        if self._startup_retry_delay < 0.0:
            raise ValueError("startup_retry_delay_sec must be non-negative")
        self._change_clients: dict[str, object] = {}
        self._state_clients: dict[str, object] = {}

        ready_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._ready_pub = self.create_publisher(
            Bool, "nav2/lifecycle_ready", ready_qos
        )
        self._publish_ready(False)

    def _publish_ready(self, ready: bool) -> None:
        self._ready_pub.publish(Bool(data=ready))

    def _client(self, node_name: str, service_type, suffix: str, cache: dict):
        client = cache.get(node_name)
        if client is None:
            client = self.create_client(service_type, f"{node_name}/{suffix}")
            cache[node_name] = client
        return client

    @staticmethod
    def _forget_request(client, future) -> None:
        try:
            client.remove_pending_request(future)
        except (AttributeError, KeyError, RuntimeError):
            pass

    def _wait_future(self, client, future, timeout_sec: float):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                self._forget_request(client, future)
                return None
            rclpy.spin_once(self, timeout_sec=min(0.1, remaining))
        try:
            return future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"lifecycle service call failed: {exc}")
            return None

    def _get_state(self, node_name: str, timeout_sec: float = 1.0) -> int | None:
        client = self._client(
            node_name, GetState, "get_state", self._state_clients
        )
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return None
        response = self._wait_future(
            client, client.call_async(GetState.Request()), timeout_sec
        )
        if response is None:
            return None
        return int(response.current_state.id)

    def _wait_for_state(
        self, node_name: str, target_state: int, timeout_sec: float | None = None
    ) -> bool:
        deadline = time.monotonic() + (
            self._state_wait_timeout if timeout_sec is None else timeout_sec
        )
        while rclpy.ok() and time.monotonic() < deadline:
            if self._get_state(node_name) == target_state:
                return True
            time.sleep(0.05)
        return False

    def _change_state(
        self, node_name: str, transition_id: int, target_state: int
    ) -> bool:
        client = self._client(
            node_name, ChangeState, "change_state", self._change_clients
        )
        if not client.wait_for_service(timeout_sec=self._service_wait_timeout):
            self.get_logger().error(
                f"lifecycle service unavailable: {node_name}/change_state"
            )
            return False

        for attempt in range(1, self._transition_retries + 1):
            current = self._get_state(node_name)
            if current == target_state:
                return True

            request = ChangeState.Request()
            request.transition.id = transition_id
            response = self._wait_future(
                client,
                client.call_async(request),
                self._transition_timeout,
            )
            if response is None:
                self.get_logger().warning(
                    f"{node_name} transition response lost; checking actual "
                    f"state (attempt {attempt}/{self._transition_retries})"
                )
            elif not response.success:
                self.get_logger().warning(
                    f"{node_name} rejected transition={transition_id}; "
                    "checking actual state"
                )

            if self._wait_for_state(node_name, target_state):
                return True

        actual = self._get_state(node_name)
        self.get_logger().error(
            f"{node_name} failed lifecycle transition={transition_id}: "
            f"target={target_state} actual={actual}"
        )
        return False

    def _bring_up_group(self, label: str, node_names: tuple[str, ...]) -> bool:
        self.get_logger().info(f"configuring Nav2 lifecycle group: {label}")
        for node_name in node_names:
            state = self._get_state(
                node_name, timeout_sec=self._service_wait_timeout
            )
            if state is None:
                self.get_logger().error(
                    f"lifecycle state service unavailable: {node_name}"
                )
                return False
            if state == State.PRIMARY_STATE_UNCONFIGURED and not self._change_state(
                node_name,
                Transition.TRANSITION_CONFIGURE,
                State.PRIMARY_STATE_INACTIVE,
            ):
                return False
            if self._get_state(node_name) not in (
                State.PRIMARY_STATE_INACTIVE,
                State.PRIMARY_STATE_ACTIVE,
            ):
                self.get_logger().error(
                    f"{node_name} is not configurable/active after configure"
                )
                return False

        self.get_logger().info(f"activating Nav2 lifecycle group: {label}")
        for node_name in node_names:
            if not self._change_state(
                node_name,
                Transition.TRANSITION_ACTIVATE,
                State.PRIMARY_STATE_ACTIVE,
            ):
                return False
        self.get_logger().info(f"lifecycle group active: {label}")
        return True

    def start(self) -> bool:
        self.get_logger().info(
            "verified direct Nav2 lifecycle startup begins: "
            f"namespace={self.get_namespace()}"
        )
        groups = (
            ("localization", LOCALIZATION_NODES),
            ("navigation", NAVIGATION_NODES),
            ("collision_monitor", COLLISION_NODES),
        )
        for label, node_names in groups:
            if not self._bring_up_group(label, node_names):
                self._publish_ready(False)
                return False
        self._publish_ready(True)
        self.get_logger().info("all Nav2 lifecycle nodes are verified active")
        return True

    def start_with_retries(self) -> bool:
        """Resume partial lifecycle startup after transient DDS timeouts.

        Every transition in ``start()`` is state-checked and idempotent, so a
        retry safely continues from nodes that already configured or became
        active even when their service response was lost.
        """
        for attempt in range(1, self._startup_retries + 1):
            if self.start():
                return True
            if not rclpy.ok() or attempt >= self._startup_retries:
                break
            self.get_logger().warning(
                "Nav2 lifecycle startup incomplete; retrying from actual "
                f"node states in {self._startup_retry_delay:.1f}s "
                f"(attempt {attempt + 1}/{self._startup_retries})"
            )
            deadline = time.monotonic() + self._startup_retry_delay
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().error(
            "Nav2 lifecycle startup exhausted "
            f"{self._startup_retries} attempts"
        )
        return False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Nav2LifecycleSequencer()
    try:
        success = node.start_with_retries()
        if success:
            # Keep the transient-local readiness publisher alive. The auto
            # initializer may start before or after lifecycle bringup.
            rclpy.spin(node)
    except KeyboardInterrupt:
        success = True
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
