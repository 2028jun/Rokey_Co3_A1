#!/usr/bin/env python3
"""Persistent Manager-facing navigation subsystem for the restaurant robot."""

from __future__ import annotations

import json
import math
import threading
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger

from serving_robot_interfaces.srv import TaskCommand

from two_wheel_rails.autonomous_navigator import SimplifiedPathNavigator
from two_wheel_rails.nav_bootstrap import (
    make_pose,
    prepare_navigator,
    publish_initial_pose,
    sync_spawn,
    wait_for_existing_localization,
)
from two_wheel_rails.rail_geometry import load_routes


TABLE_COMMANDS = {0, 1, 2, 3}
KITCHEN_COMMAND = 4
RESUME_COMMAND = 98
PAUSE_COMMAND = 99

NAV_IDLE = 0
NAV_MOVING = 1
NAV_ARRIVED = 2
NAV_FAILED = 3
NAV_PAUSED = 4

MISSION_COMMAND_TOPIC = "/two_wheel/mission_command"
MISSION_STATUS_TOPIC = "/two_wheel/mission_status"

DOCKING_PHASES = {
    "final_forward_approach",
    "near_goal_creep",
    "near_goal_align",
    "settle_at_table",
    "recovery_backout",
    "recovery_reapproach",
}


class NavigationSubsystemNode(Node):
    """Own one persistent Nav2 client and expose the Manager service contract."""

    def __init__(self) -> None:
        super().__init__("navigation_subsystem")
        self._routes = load_routes()
        self._lock = threading.Lock()
        self._worker: threading.Thread | None = None
        self._worker_nav = None
        self._tf_buffer = None
        self._tracker = None
        self._controller: SimplifiedPathNavigator | None = None
        self._active_command: int | None = None
        self._paused = False
        self._last_phase = "idle"
        self._last_mission_id = ""
        self._failure_reason = ""

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        mission_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._status_pub = self.create_publisher(
            Int32, "/navigation/status", status_qos
        )
        self._location_pub = self.create_publisher(
            Int32, "/navigation/current_location", status_qos
        )
        self._detail_pub = self.create_publisher(
            String, "/navigation/detail", status_qos
        )
        self._mission_command_pub = self.create_publisher(
            String, MISSION_COMMAND_TOPIC, mission_qos
        )
        self.create_subscription(
            String,
            MISSION_STATUS_TOPIC,
            self._on_mission_status,
            mission_qos,
        )
        self.create_service(
            TaskCommand, "/navigation/command", self._on_command
        )
        self.create_service(
            Trigger, "/navigation/initialize", self._on_initialize
        )
        self._publish_status(NAV_IDLE, "IDLE", "idle")
        self.get_logger().info(
            "navigation subsystem ready: commands 0..4, pause=99, resume=98"
        )

    def _on_command(self, request, response):
        command = int(request.command)
        if command == PAUSE_COMMAND:
            response.success = self._pause()
            return response
        if command == RESUME_COMMAND:
            response.success = self._resume()
            return response
        if command not in TABLE_COMMANDS and command != KITCHEN_COMMAND:
            self.get_logger().warning(f"unsupported navigation command={command}")
            response.success = False
            return response

        with self._lock:
            if self._worker is not None and self._worker.is_alive():
                self.get_logger().warning(
                    f"navigation busy; rejected command={command}"
                )
                response.success = False
                return response
            self._active_command = command
            self._paused = False
            self._failure_reason = ""
            self._worker = threading.Thread(
                target=self._run_command,
                args=(command,),
                daemon=True,
                name=f"navigation-command-{command}",
            )
            self._worker.start()
        response.success = True
        return response

    def _on_initialize(self, _request, response):
        with self._lock:
            if self._worker is not None and self._worker.is_alive():
                response.success = False
                response.message = "navigation is busy"
                return response
            self._active_command = None
            self._worker = threading.Thread(
                target=self._run_initialize,
                daemon=True,
                name="navigation-initialize",
            )
            self._worker.start()
        response.success = True
        response.message = "spawn initialization accepted"
        return response

    def _ensure_navigation(self) -> None:
        if self._worker_nav is not None:
            return
        nav, tf_buffer, tracker = prepare_navigator(
            node_name="navigation_subsystem_navigator"
        )
        # Never let BasicNavigator publish its default map (0, 0, 0) pose.
        nav.initial_pose_received = True
        nav.waitUntilNav2Active()
        self._worker_nav = nav
        self._tf_buffer = tf_buffer
        self._tracker = tracker
        self._controller = SimplifiedPathNavigator(nav, tf_buffer, tracker)
        if hasattr(self, "_executor_ref") and self._executor_ref is not None:
            self._executor_ref.add_node(nav)

    def _run_initialize(self) -> None:
        try:
            self._publish_status(
                NAV_MOVING, "VALIDATING_LOCALIZATION", "initialize"
            )
            self._ensure_navigation()
            spawn = self._routes.get("spawn") or self._routes.get("kitchen")
            if spawn is None:
                raise RuntimeError("routes config has no spawn or kitchen pose")
            sx = float(spawn["x"])
            sy = float(spawn["y"])
            syaw = float(spawn["yaw"])
            self._worker_nav.initial_pose = make_pose(
                self._worker_nav, sx, sy, syaw
            )
            publish_initial_pose(
                self._worker_nav, sx, sy, syaw, repeats=2
            )
            if not sync_spawn(
                self._worker_nav,
                self._tf_buffer,
                self._tracker,
                sx,
                sy,
                syaw,
            ):
                raise RuntimeError("spawn teleport/AMCL synchronization failed")
            self._location_pub.publish(Int32(data=KITCHEN_COMMAND))
            self._publish_status(
                NAV_ARRIVED, "SUCCEEDED", "initialized"
            )
        except Exception as exc:  # noqa: BLE001
            self._publish_failure(f"initialization failed: {exc}")
        finally:
            with self._lock:
                self._worker = None

    def _run_command(self, command: int) -> None:
        try:
            self._publish_status(
                NAV_MOVING, "VALIDATING_LOCALIZATION", "localization"
            )
            self._ensure_navigation()
            current = wait_for_existing_localization(
                self._worker_nav,
                self._tf_buffer,
                self._tracker,
                timeout_sec=8.0,
            )
            if current is None:
                raise RuntimeError("current localization unavailable")

            if command == KITCHEN_COMMAND:
                self._park_out_if_needed(current)
                current = wait_for_existing_localization(
                    self._worker_nav,
                    self._tf_buffer,
                    self._tracker,
                    timeout_sec=8.0,
                )
                if current is None:
                    raise RuntimeError(
                        "localization unavailable after park-out"
                    )

            gx, gy, gyaw = self._goal_for_command(command)
            goal = make_pose(self._worker_nav, gx, gy, gyaw)
            label = (
                "kitchen"
                if command == KITCHEN_COMMAND
                else f"table_{command}"
            )
            self._publish_status(NAV_MOVING, "PLANNING", "planning")
            ok = self._controller.navigate_to(
                goal,
                label=label,
                position_then_align=(command == KITCHEN_COMMAND),
            )
            if not ok:
                raise RuntimeError(
                    self._failure_reason or f"{label} mission failed"
                )
            self._location_pub.publish(Int32(data=command))
            self._publish_status(NAV_ARRIVED, "SUCCEEDED", "completed")
        except Exception as exc:  # noqa: BLE001
            self._publish_failure(str(exc))
        finally:
            with self._lock:
                self._active_command = None
                self._paused = False
                self._worker = None

    def _park_out_if_needed(
        self, current_pose: tuple[float, float, float]
    ) -> None:
        px, py, pyaw = current_pose
        near_table_row = min(abs(py + 2.20), abs(py - 0.70)) <= 0.70
        parked_at_table = abs(px) >= 1.15 and near_table_row
        if not parked_at_table:
            return

        expected_yaw = math.pi if px < 0.0 else 0.0
        yaw_error = math.atan2(
            math.sin(expected_yaw - pyaw),
            math.cos(expected_yaw - pyaw),
        )
        if abs(yaw_error) > math.radians(20.0):
            raise RuntimeError(
                f"unsafe park-out heading error="
                f"{math.degrees(yaw_error):.1f}deg"
            )
        self._publish_status(NAV_MOVING, "PARKING_OUT", "park_out")
        if not self._controller.drive_distance(
            0.50, -0.20, label="park_out"
        ):
            raise RuntimeError("0.50m park-out failed")

    def _goal_for_command(self, command: int) -> tuple[float, float, float]:
        if command == KITCHEN_COMMAND:
            goal = self._routes.get("kitchen") or self._routes.get("spawn")
        else:
            goal = self._routes["routes"][f"to_{command}"]["dock"]
        if goal is None:
            raise RuntimeError(f"goal is missing for command={command}")
        return float(goal["x"]), float(goal["y"]), float(goal["yaw"])

    def _pause(self) -> bool:
        with self._lock:
            if self._worker is None or not self._worker.is_alive():
                return False
            if self._paused:
                return True
            self._paused = True
        self._send_control("pause")
        self._publish_status(NAV_PAUSED, "PAUSED", "paused")
        return True

    def _resume(self) -> bool:
        with self._lock:
            if (
                self._worker is None
                or not self._worker.is_alive()
                or not self._paused
            ):
                return False
            self._paused = False
        self._send_control("resume")
        self._publish_status(NAV_MOVING, "EXECUTING", "resumed")
        return True

    def _send_control(self, kind: str) -> None:
        msg = String()
        msg.data = json.dumps({"kind": kind, "mission_id": ""})
        for _ in range(3):
            self._mission_command_pub.publish(msg)

    def _on_mission_status(self, msg: String) -> None:
        try:
            payload: dict[str, Any] = json.loads(msg.data)
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(f"invalid mission status: {exc}")
            return
        self._last_mission_id = str(payload.get("mission_id", ""))
        self._last_phase = str(payload.get("phase", "unknown"))
        state = str(payload.get("state", "unknown"))
        if state in {"failed", "cancelled"}:
            self._failure_reason = (
                f"mission={self._last_mission_id} state={state} "
                f"phase={self._last_phase}"
            )
        if state == "running":
            mapped = (
                "DOCKING"
                if self._last_phase in DOCKING_PHASES
                else ("PAUSED" if self._last_phase == "paused" else "EXECUTING")
            )
            self._publish_detail(mapped, self._last_phase)

    def _publish_status(
        self, status: int, state: str, phase: str, reason: str = ""
    ) -> None:
        self._status_pub.publish(Int32(data=status))
        self._publish_detail(state, phase, reason)

    def _publish_failure(self, reason: str) -> None:
        self.get_logger().error(reason)
        self._failure_reason = reason
        self._publish_status(NAV_FAILED, "FAILED", self._last_phase, reason)

    def _publish_detail(
        self, state: str, phase: str, reason: str = ""
    ) -> None:
        with self._lock:
            command = self._active_command
        msg = String()
        msg.data = json.dumps(
            {
                "command": command,
                "state": state,
                "phase": phase,
                "mission_id": self._last_mission_id,
                "reason": reason,
            },
            ensure_ascii=False,
        )
        self._detail_pub.publish(msg)

    def destroy_node(self):
        if self._worker_nav is not None:
            self._worker_nav.destroy_node()
            self._worker_nav = None
        return super().destroy_node()


def main(args=None) -> None:
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = NavigationSubsystemNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    node._executor_ref = executor
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
