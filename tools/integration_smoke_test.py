#!/usr/bin/env python3
"""ROS 2 integration smoke test for the two-wheel serving robot.

Run after nav_robot5 t1 and t2. The default test is read-only. Use
--motion-test only when the robot has free space in front of it.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class IntegrationProbe(Node):
    def __init__(self) -> None:
        super().__init__("serving_robot_integration_probe")
        self.clock: Clock | None = None
        self.scan: LaserScan | None = None
        self.odom: Odometry | None = None
        self.raw_odom: Odometry | None = None
        self.hand_intrusion: Bool | None = None

        self.create_subscription(Clock, "/clock", self._clock_cb, 10)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, SENSOR_QOS)
        self.create_subscription(Odometry, "/odom", self._odom_cb, SENSOR_QOS)
        self.create_subscription(
            Odometry, "/two_wheel/odom_raw", self._raw_odom_cb, 20
        )
        self.create_subscription(
            Bool, "/hand_safety/intrusion", self._hand_cb, 10
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _clock_cb(self, msg: Clock) -> None:
        self.clock = msg

    def _scan_cb(self, msg: LaserScan) -> None:
        self.scan = msg

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom = msg

    def _raw_odom_cb(self, msg: Odometry) -> None:
        self.raw_odom = msg

    def _hand_cb(self, msg: Bool) -> None:
        self.hand_intrusion = msg


def spin_until(node: Node, predicate, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return True
    return False


def lifecycle_active(node: Node, name: str, timeout: float = 5.0) -> bool:
    client = node.create_client(GetState, f"/{name}/get_state")
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            return False
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        response = future.result()
        return response is not None and response.current_state.label == "active"
    finally:
        node.destroy_client(client)


def finite_scan_count(scan: LaserScan | None) -> int:
    if scan is None:
        return 0
    return sum(
        1
        for value in scan.ranges
        if math.isfinite(value) and scan.range_min <= value <= scan.range_max
    )


def raw_xy(msg: Odometry | None) -> tuple[float, float] | None:
    if msg is None:
        return None
    return float(msg.pose.pose.position.x), float(msg.pose.pose.position.y)


def motion_test(node: IntegrationProbe) -> tuple[bool, str]:
    if not spin_until(node, lambda: node.raw_odom is not None, 5.0):
        return False, "raw odom unavailable"
    start = raw_xy(node.raw_odom)
    assert start is not None

    command = Twist()
    command.linear.x = 0.08
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        node.cmd_pub.publish(command)
        rclpy.spin_once(node, timeout_sec=0.04)

    stop = Twist()
    for _ in range(10):
        node.cmd_pub.publish(stop)
        rclpy.spin_once(node, timeout_sec=0.04)

    end = raw_xy(node.raw_odom)
    assert end is not None
    distance = math.hypot(end[0] - start[0], end[1] - start[1])
    return distance >= 0.03, f"travel={distance:.3f} m"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--require-hand-safety", action="store_true")
    parser.add_argument("--motion-test", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = IntegrationProbe()
    results: list[tuple[str, bool, str]] = []
    try:
        spin_until(
            node,
            lambda: all(
                item is not None
                for item in (node.clock, node.scan, node.odom, node.raw_odom)
            ),
            args.timeout,
        )

        results.append(("clock", node.clock is not None, "/clock received"))
        count = finite_scan_count(node.scan)
        results.append(("lidar", count > 0, f"finite ranges={count}"))
        results.append(("odom", node.odom is not None, "/odom received"))
        results.append(
            ("raw_odom", node.raw_odom is not None, "/two_wheel/odom_raw received")
        )

        tf_ok = False
        tf_detail = "map -> ridgeback_base_link unavailable"
        try:
            transform: TransformStamped = node.tf_buffer.lookup_transform(
                "map",
                "ridgeback_base_link",
                rclpy.time.Time(),
                timeout=Duration(seconds=3.0),
            )
            tf_ok = True
            tf_detail = (
                f"x={transform.transform.translation.x:.2f}, "
                f"y={transform.transform.translation.y:.2f}"
            )
        except Exception as exc:
            tf_detail = str(exc)
        results.append(("tf", tf_ok, tf_detail))

        results.append(
            ("amcl", lifecycle_active(node, "amcl"), "lifecycle active")
        )
        results.append(
            (
                "bt_navigator",
                lifecycle_active(node, "bt_navigator"),
                "lifecycle active",
            )
        )

        if args.require_hand_safety:
            hand_ok = spin_until(
                node, lambda: node.hand_intrusion is not None, args.timeout
            )
            detail = (
                f"intrusion={node.hand_intrusion.data}"
                if node.hand_intrusion is not None
                else "topic unavailable"
            )
            results.append(("hand_safety", hand_ok, detail))

        if args.motion_test:
            ok, detail = motion_test(node)
            results.append(("motion", ok, detail))
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print("\n=== Serving Robot Integration Test ===")
    for name, ok, detail in results:
        print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}")
    failed = [name for name, ok, _ in results if not ok]
    if failed:
        print(f"\nFAILED: {', '.join(failed)}")
        return 1
    print("\nALL CHECKS PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
