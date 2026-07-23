#!/usr/bin/env python3
"""Dense SLAM coverage: table backs, door fronts, outer wall lanes.

Uses Isaac absolute pose (/two_wheel/odom_raw) + /cmd_vel.
"""

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

RAW_ODOM_TOPIC = "/two_wheel/odom_raw"
LINEAR_SPEED = 0.26
SPIN_W_MAX = 0.50
YAW_TOL = 0.12
POS_TOL = 0.20
APPROACH_YAW_TOL = 0.30
LOOK_YAWS = (0.0, math.pi / 2, math.pi, -math.pi / 2)
SEG_TIMEOUT_SEC = 150.0

# Geometry from restaurant rails / occupancy docs
DOCK = 1.60          # aisle-side approach (safe of table face ~1.82)
OUTER = 2.55         # behind-table / wall-side lane
FLANK = 0.90         # N/S offset around each table
DOOR_Y = 4.90        # kitchen doorway on spine (keep |x| small — arm hits frame)
KIT_Y = 5.55         # shallow into kitchen only (deeper + side = jam)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class Segment:
    x: float
    y: float
    yaw: float
    label: str
    look: bool = False


def _wp(
    x: float, y: float, yaw: float, label: str, *, look: bool = False
) -> Segment:
    return Segment(x, y, yaw, label, look)


def _table_behind(tid: int, jy: float, side: str) -> list[Segment]:
    """Aisle junction → dock → N/S flanks → outer (behind table) loop → back."""
    west = side == "w"
    sx = -1.0 if west else 1.0
    face = math.pi if west else 0.0
    back = 0.0 if west else math.pi
    dock_x = sx * DOCK
    outer_x = sx * OUTER
    yn, ys = jy + FLANK, jy - FLANK
    tag = f"t{tid}"
    return [
        _wp(0.00, jy, face, f"{tag}_junc"),
        _wp(sx * 0.80, jy, face, f"{tag}_mid"),
        _wp(dock_x, jy, face, f"{tag}_dock", look=True),
        # north of table, then behind
        _wp(dock_x, yn, face, f"{tag}_n_dock"),
        _wp(outer_x, yn, face, f"{tag}_n_outer", look=True),
        _wp(outer_x, jy, back, f"{tag}_behind", look=True),
        _wp(outer_x, ys, back, f"{tag}_s_outer", look=True),
        _wp(dock_x, ys, back, f"{tag}_s_dock"),
        _wp(sx * 0.80, jy, back, f"{tag}_mid_back"),
        _wp(0.00, jy, -math.pi / 2 if jy > 0 else math.pi / 2, f"{tag}_junc_back"),
    ]


def _build_patrol() -> list[Segment]:
    pts: list[Segment] = []

    # --- Door: CENTER LANE ONLY (arm + ridgeback width jam on ±0.85 frame pts) ---
    # No look-around in doorway — spinning swings the arm into the frame.
    pts += [
        _wp(0.00, 5.25, -math.pi / 2, "spawn", look=True),
        _wp(0.00, 5.05, -math.pi / 2, "door_approach"),
        _wp(0.00, DOOR_Y, -math.pi / 2, "door_center"),
        _wp(0.00, 5.20, math.pi / 2, "door_into"),
        _wp(0.21, KIT_Y, math.pi / 2, "kitchen_shallow", look=True),
        _wp(0.00, DOOR_Y, -math.pi / 2, "door_exit"),
        _wp(0.00, 4.50, -math.pi / 2, "door_clear"),
        # Scan door from dining side (safe of frame), not from inside the throat
        _wp(-0.45, 4.40, math.pi / 2, "door_scan_w", look=True),
        _wp(0.45, 4.40, math.pi / 2, "door_scan_e", look=True),
        _wp(0.00, 4.20, -math.pi / 2, "aisle_4.2", look=True),
        _wp(0.00, 2.50, -math.pi / 2, "aisle_2.5"),
        _wp(0.00, 0.70, -math.pi / 2, "aisle_0.7"),
    ]

    # --- North row tables (T2 west, T3 east) ---
    pts += _table_behind(2, 0.70, "w")
    pts += _table_behind(3, 0.70, "e")

    # --- Between table rows ---
    pts += [
        _wp(0.00, -0.75, -math.pi / 2, "mid_gap", look=True),
        _wp(-1.20, -0.75, math.pi, "mid_gap_w", look=True),
        _wp(1.20, -0.75, 0.0, "mid_gap_e", look=True),
        _wp(0.00, -0.75, -math.pi / 2, "mid_gap_c"),
        _wp(0.00, -2.20, -math.pi / 2, "aisle_-2.2"),
    ]

    # --- South row tables (T0 west, T1 east) ---
    pts += _table_behind(0, -2.20, "w")
    pts += _table_behind(1, -2.20, "e")

    # --- Outer wall lanes (stay south of door throat; rejoin spine at y=4.2) ---
    pts += [
        _wp(0.00, -2.20, math.pi, "to_west_wall"),
        _wp(-OUTER, -3.20, -math.pi / 2, "sw_corner", look=True),
        _wp(-OUTER, -2.20, math.pi / 2, "west_lane_s", look=True),
        _wp(-OUTER, -0.75, math.pi / 2, "west_lane_mid", look=True),
        _wp(-OUTER, 0.70, math.pi / 2, "west_lane_n", look=True),
        _wp(-OUTER, 2.20, math.pi / 2, "west_lane_2.2", look=True),
        _wp(-OUTER, 3.20, math.pi / 2, "west_lane_3.2", look=True),
        _wp(-0.50, 4.20, 0.0, "nw_rejoin"),
        _wp(0.00, 4.20, 0.0, "spine_4.2_re"),
        _wp(0.50, 4.20, 0.0, "ne_depart"),
        _wp(OUTER, 3.20, -math.pi / 2, "east_lane_3.2", look=True),
        _wp(OUTER, 2.20, -math.pi / 2, "east_lane_2.2", look=True),
        _wp(OUTER, 0.70, -math.pi / 2, "east_lane_n", look=True),
        _wp(OUTER, -0.75, -math.pi / 2, "east_lane_mid", look=True),
        _wp(OUTER, -2.20, -math.pi / 2, "east_lane_s", look=True),
        _wp(OUTER, -3.20, math.pi, "se_corner", look=True),
        _wp(0.00, -3.20, math.pi / 2, "south_door_front", look=True),
        _wp(-1.00, -3.20, math.pi, "south_door_w", look=True),
        _wp(1.00, -3.20, 0.0, "south_door_e", look=True),
        _wp(0.00, -2.20, math.pi / 2, "aisle_up"),
    ]

    # --- Return on spine; through door CENTER only ---
    pts += [
        _wp(0.00, 0.70, math.pi / 2, "up_0.7"),
        _wp(0.00, 2.50, math.pi / 2, "up_2.5"),
        _wp(0.00, 4.20, math.pi / 2, "up_4.2", look=True),
        _wp(0.00, DOOR_Y, math.pi / 2, "door_return"),
        _wp(0.21, 5.25, -math.pi / 2, "kitchen_home", look=True),
    ]
    return pts


PATROL = _build_patrol()


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

    def _dwell(self, sec: float = 0.7) -> None:
        end = time.time() + sec
        while time.time() < end and rclpy.ok():
            self._cmd(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.05)

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
            mag = max(0.12, min(SPIN_W_MAX, 1.6 * abs(err)))
            self._cmd(0.0, math.copysign(mag, err))

        self.get_logger().error(f"yaw timeout ({label})")
        return False

    def _look_around(self, label: str, deadline: float) -> bool:
        self.get_logger().info(f"{label} look-around")
        for i, yaw in enumerate(LOOK_YAWS):
            if not self._spin_yaw(yaw, f"{label}_look{i}", deadline):
                return False
            self._dwell(0.55)
        return True

    def _drive_to(self, seg: Segment, deadline: float) -> bool:
        last_log = 0.0
        best_dist: float | None = None
        stuck_since: float | None = None
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            p = self.pose()
            if p is None:
                continue

            x, y, yaw = p
            dx, dy = seg.x - x, seg.y - y
            dist = math.hypot(dx, dy)
            yaw_err = _wrap(seg.yaw - yaw)

            # Stuck: no progress for a few seconds (e.g. arm wedged on door)
            if best_dist is None or dist < best_dist - 0.04:
                best_dist = dist
                stuck_since = time.time()
            elif stuck_since is not None and time.time() - stuck_since > 8.0:
                self._stop()
                self.get_logger().warn(
                    f"{seg.label} stuck dist={dist:.2f} (no progress 8s) — skip"
                )
                return False

            if dist <= POS_TOL:
                if abs(yaw_err) <= YAW_TOL:
                    self._stop()
                    self.get_logger().info(
                        f"{seg.label} OK odom=({x:.2f},{y:.2f},{yaw:.2f})"
                    )
                    return True
                if not self._spin_yaw(seg.yaw, f"{seg.label}_final_yaw", deadline):
                    return False
                continue

            bearing = math.atan2(dy, dx)
            bear_err = _wrap(bearing - yaw)
            if abs(bear_err) > APPROACH_YAW_TOL:
                if not self._spin_yaw(bearing, f"{seg.label}_bear", deadline):
                    return False
                continue

            vx = max(0.06, min(LINEAR_SPEED, 0.85 * dist))
            wz = max(-0.35, min(0.35, 1.2 * bear_err))
            self._cmd(vx, wz)

            now = time.time()
            if now - last_log > 2.5:
                last_log = now
                self.get_logger().info(
                    f"{seg.label} pose=({x:.2f},{y:.2f}) dist={dist:.2f}"
                )

        p = self.pose()
        self.get_logger().error(
            f"timeout ({seg.label}) tgt=({seg.x:.2f},{seg.y:.2f}) "
            f"pose={None if p is None else (round(p[0], 2), round(p[1], 2))}"
        )
        return False

    def run_patrol(self) -> bool:
        p = self.pose()
        self.get_logger().info(
            f"patrol start waypoints={len(PATROL)} "
            f"(table-behind + door + outer lanes) "
            f"odom={None if not p else (round(p[0], 2), round(p[1], 2), round(p[2], 2))}"
        )
        total = len(PATROL)
        for i, seg in enumerate(PATROL):
            self.get_logger().info(
                f"[{i + 1}/{total}] {seg.label} -> ({seg.x:.2f},{seg.y:.2f}) "
                f"look={seg.look}"
            )
            deadline = time.time() + SEG_TIMEOUT_SEC
            if not self._drive_to(seg, deadline):
                # Soft-skip unreachable outer points so one collision doesn't abort map
                self.get_logger().warn(
                    f"{seg.label} unreachable — skip and continue coverage"
                )
                self._stop()
                continue
            self._dwell(0.35)
            if seg.look:
                # Extra time budget for look-around
                if not self._look_around(seg.label, time.time() + 60.0):
                    self.get_logger().warn(f"{seg.label} look-around incomplete — continue")
                    self._stop()
        self._stop()
        self.get_logger().info(
            "patrol done — run: save_map (keep t2 / slam_mapping running)"
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
