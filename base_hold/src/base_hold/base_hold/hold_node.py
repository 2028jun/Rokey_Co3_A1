#!/usr/bin/env python3
"""Base hold node: gate /cmd_vel while Isaac applies wheel position hold.

Modes
-----
BASE_MOVING (hold=false): pass /cmd_vel_in -> /cmd_vel
ARM_HOLD    (hold=true):  ignore input, republish Twist() at hold_publish_hz

This ROS node alone does NOT resist arm reaction torque — Isaac must also run
``IsaacWheelHold`` (see isaac_wheel_hold.py / examples) which switches wheel
joints to high-stiffness position hold. No world FixedJoint (Isaac-crash prone).
"""

from __future__ import annotations

import threading

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


def _latched_bool() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )


def _volatile(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )


class BaseHoldNode(Node):
    def __init__(self) -> None:
        super().__init__("base_hold_node")
        self.declare_parameter("input_topic", "/cmd_vel_in")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("hold_service", "/base/hold")
        self.declare_parameter("hold_state_topic", "/base/hold_state")
        self.declare_parameter("hold_publish_hz", 20.0)
        self.declare_parameter("start_held", False)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        hold_service = str(self.get_parameter("hold_service").value)
        hold_state_topic = str(self.get_parameter("hold_state_topic").value)
        hz = float(self.get_parameter("hold_publish_hz").value)
        start_held = bool(self.get_parameter("start_held").value)

        self._lock = threading.Lock()
        self._held = start_held

        self._cmd_pub = self.create_publisher(Twist, output_topic, _volatile(20))
        self._state_pub = self.create_publisher(Bool, hold_state_topic, _latched_bool())
        self.create_subscription(Twist, input_topic, self._on_cmd, _volatile(20))
        self.create_service(SetBool, hold_service, self._on_hold_srv)

        period = 1.0 / max(hz, 1.0)
        self.create_timer(period, self._on_timer)

        self._publish_state()
        if self._held:
            self._publish_zero()
        self.get_logger().info(
            f"base_hold ready: {input_topic} -> {output_topic}, "
            f"service={hold_service}, held={self._held} "
            f"(pair with IsaacWheelHold for real lock)"
        )

    def _publish_state(self) -> None:
        msg = Bool()
        with self._lock:
            msg.data = self._held
        self._state_pub.publish(msg)

    def _publish_zero(self) -> None:
        self._cmd_pub.publish(Twist())

    def _on_cmd(self, msg: Twist) -> None:
        with self._lock:
            held = self._held
        if held:
            return
        self._cmd_pub.publish(msg)

    def _on_timer(self) -> None:
        with self._lock:
            held = self._held
        if held:
            self._publish_zero()

    def _on_hold_srv(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        want = bool(request.data)
        with self._lock:
            prev = self._held
            self._held = want
        self._publish_state()
        if want:
            self._publish_zero()
            response.success = True
            response.message = (
                "ARM_HOLD: cmd_vel gated; ensure IsaacWheelHold.engage() is active"
            )
        else:
            response.success = True
            response.message = "BASE_MOVING: cmd_vel pass-through"
        if prev != want:
            self.get_logger().info(response.message)
        return response


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = BaseHoldNode()
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
