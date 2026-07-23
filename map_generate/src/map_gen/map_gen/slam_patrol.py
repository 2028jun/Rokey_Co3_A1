#!/usr/bin/env python3
"""One-shot SLAM coverage patrol: Isaac absolute odom + /cmd_vel (in-place spin)."""

from __future__ import annotations

import math
import os
import sys
import threading
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

# Absolute world pose (PATROL waypoints). Bridge /odom is rebased + BEST_EFFORT.
RAW_ODOM_TOPIC = "/two_wheel/odom_raw"
LINEAR_SPEED = 0.40
SPIN_W_MAX = 0.55
YAW_TOL = 0.15
POS_TOL = 0.22
# Face goal when farther than this; final yaw align when close
APPROACH_YAW_TOL = 0.35


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class Segment:
    x: float
    y: float
    yaw: float
    label: str


# Kitchen -> spine -> west/east branches -> kitchen (SLAM coverage)
PATROL = [
    Segment(0.00, 4.20, -math.pi / 2, "spine_4.2"),
    Segment(0.00, 2.50, -math.pi / 2, "spine_2.5"),
    Segment(0.00, 0.70, -math.pi / 2, "spine_0.7"),
    Segment(-1.17, 0.70, math.pi, "branch_west_n"),
    Segment(0.00, 0.70, -math.pi / 2, "spine_mid"),
    Segment(0.00, -2.20, -math.pi / 2, "spine_-2.2"),
    Segment(-1.17, -2.20, math.pi, "branch_west_s"),
    Segment(0.00, -2.20, -math.pi / 2, "spine_s_return"),
    Segment(0.00, 0.70, -math.pi / 2, "spine_n_return"),
    Segment(1.17, 0.70, 0.0, "branch_east_n"),
    Segment(1.17, -2.20, 0.0, "branch_east_s"),
    Segment(0.00, -2.20, -math.pi / 2, "spine_final"),
    Segment(0.00, 4.20, -math.pi / 2, "spine_4.2_up"),
    Segment(0.00, 5.00, -math.pi / 2, "spine_north"),
    Segment(0.21, 5.25, -math.pi / 2, "kitchen"),
]


class SlamPatrol(Node):
    def __init__(self):
        super().__init__("map_gen_slam_patrol")
        self._lock = threading.Lock()
        self._odom: tuple[float, float, float] | None = None

        qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Odometry, RAW_ODOM_TOPIC, self._on_odom, qos)
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", qos)

    def _on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        # Pure-z yaw (more stable near ±90° than the truncated atan2 form)
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        with self._lock:
            self._odom = (
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
                yaw,
            )

    def pose(self):
        with self._lock:
            return self._odom

    def _cmd(self, vx: float, wz: float):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self._cmd_pub.publish(msg)

    def _stop(self):
        for _ in range(5):
            self._cmd(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)

    def _wait_odom(self, timeout: float = 20.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.pose() is not None:
                return True
        return False

    def _spin_yaw(self, target_yaw: float, label: str, deadline: float) -> bool:
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            p = self.pose()
            if p is None:
                continue
            err = _wrap(target_yaw - p[2])
            if abs(err) <= YAW_TOL:
                self._stop()
                return True
            # Proportional spin with floor so we don't stall near tolerance
            mag = max(0.12, min(SPIN_W_MAX, 1.6 * abs(err)))
            wz = math.copysign(mag, err)
            self._cmd(0.0, wz)

        self.get_logger().error(f"yaw timeout ({label})")
        return False

    def _drive_to(self, seg: Segment, deadline: float) -> bool:
        last_log = 0.0
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            p = self.pose()
            if p is None:
                continue

            x, y, yaw = p
            dx, dy = seg.x - x, seg.y - y
            dist = math.hypot(dx, dy)
            yaw_err = _wrap(seg.yaw - yaw)

            if dist <= POS_TOL:
                if abs(yaw_err) <= YAW_TOL:
                    self._stop()
                    self.get_logger().info(
                        f"{seg.label} OK odom=({x:.2f},{y:.2f},{yaw:.2f}) "
                        f"err=({dx:.2f},{dy:.2f})"
                    )
                    return True
                if not self._spin_yaw(seg.yaw, f"{seg.label}_final_yaw", deadline):
                    return False
                continue

            # Far: face bearing-to-goal, then drive (do NOT lock to segment yaw)
            bearing = math.atan2(dy, dx)
            bear_err = _wrap(bearing - yaw)
            if abs(bear_err) > APPROACH_YAW_TOL:
                if not self._spin_yaw(bearing, f"{seg.label}_bear", deadline):
                    return False
                continue

            vx = max(0.08, min(LINEAR_SPEED, 0.9 * dist))
            wz = max(-0.4, min(0.4, 1.2 * bear_err))
            self._cmd(vx, wz)

            now = time.time()
            if now - last_log > 2.0:
                last_log = now
                self.get_logger().info(
                    f"{seg.label} pose=({x:.2f},{y:.2f},{yaw:.2f}) "
                    f"dist={dist:.2f} bear_err={bear_err:.2f} vx={vx:.2f}"
                )

        p = self.pose()
        self.get_logger().error(
            f"timeout ({seg.label}) tgt=({seg.x:.2f},{seg.y:.2f}) "
            f"pose={None if p is None else (round(p[0], 2), round(p[1], 2), round(p[2], 2))}"
        )
        return False

    def run_patrol(self) -> bool:
        p = self.pose()
        if p:
            self.get_logger().info(f"patrol start odom=({p[0]:.2f},{p[1]:.2f},{p[2]:.2f})")
        total = len(PATROL)
        for i, seg in enumerate(PATROL):
            self.get_logger().info(
                f"[{i + 1}/{total}] {seg.label} -> ({seg.x:.2f},{seg.y:.2f}) yaw={seg.yaw:.2f}"
            )
            if not self._drive_to(seg, time.time() + 120.0):
                return False
        self._stop()
        self.get_logger().info(
            "patrol done — run: bash tools/save_slam_map.sh (with slam_mapping.launch still up)"
        )
        return True


def main(argv=None) -> int:
    os.environ["ROS_DOMAIN_ID"] = os.environ.get(
        "MAP_GEN_ROS_DOMAIN_ID", os.environ.get("ROS_DOMAIN_ID", "113")
    )
    rclpy.init(args=None)
    node = None
    try:
        node = SlamPatrol()
        if not node._wait_odom():
            node.get_logger().error(
                f"no {RAW_ODOM_TOPIC} — start t1 (Isaac Play) first"
            )
            return 1
        return 0 if node.run_patrol() else 2
    finally:
        if node is not None:
            try:
                node._stop()
            except Exception:
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
