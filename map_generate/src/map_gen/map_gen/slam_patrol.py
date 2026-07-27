#!/usr/bin/env python3
"""Dense SLAM coverage: table backs, door fronts, outer wall + plant corners.

Uses Isaac absolute pose (/two_wheel/odom_raw) + /cmd_vel.

Plant pots sit near room corners (~±5.1, 3.9) and (~±5.1, -4.0). Patrol
scans from free floor (WALL≈3.10, clear of chairs at |x|≈3.7) with look-around
so LiDAR fills gray unknown cells without wedging into furniture.
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
LINEAR_SPEED = 0.22
SPIN_W_MAX = 0.50
YAW_TOL = 0.12
POS_TOL = 0.22
APPROACH_YAW_TOL = 0.30
LOOK_YAWS = (0.0, math.pi / 2, math.pi, -math.pi / 2)
SEG_TIMEOUT_SEC = 150.0

# Geometry from restaurant rails / occupancy docs
DOCK = 1.60          # aisle-side approach (safe of table face ~1.82)
OUTER = 2.55         # behind-table / wall-side lane
# Chairs sit near |x|≈3.7 and pots at |x|≈5.1 — stay inside free floor.
WALL = 3.10
FLANK = 0.90         # N/S offset around each table
DOOR_Y = 4.90        # kitchen doorway on spine (keep |x| small — arm hits frame)
# Stay on centerline in kitchen — x≈0.21 docks jam the right doorway lip.
KIT_Y = 5.35
# Plant pot centers (from generate_placeholder_map / restaurant USD)
PLANT_N = 3.90
PLANT_S = -4.00
SOUTH_DOOR = -3.40
# Approach points: free floor only (clear of pot AABB ~0.35 m and chairs)
CORNER_N = 3.20
CORNER_S = -3.20
CORNER_TIP = 0.15
# How far toward pots we drive (must stay < chair x≈3.7)
PLANT_APPROACH_X = 3.05
STUCK_REVERSE_M = 0.35
STUCK_REVERSE_SPEED = -0.18
STUCK_NO_PROGRESS_SEC = 6.0


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


def _plant_corner(sx: float, cy: float, tag: str) -> list[Segment]:
    """Scan a plant-pot corner from free floor (do not drive into chairs/pots).

    ``sx`` is -1 (west) or +1 (east). ``cy`` is near PLANT_N or PLANT_S.
    Approach stops at PLANT_APPROACH_X≈3.05 so LiDAR sees pots at ~±5.1.
    """
    face_out = math.pi if sx < 0 else 0.0
    face_in = 0.0 if sx < 0 else math.pi
    ax = sx * PLANT_APPROACH_X
    ox = sx * OUTER
    tip = CORNER_TIP if cy > 0 else -CORNER_TIP
    if abs(cy - PLANT_N) < 1.0:
        approach_yaw = math.atan2(PLANT_N - cy + 0.2, sx * 1.5)
    else:
        approach_yaw = math.atan2(PLANT_S - cy - 0.2, sx * 1.5)
    return [
        _wp(ox, cy, face_out, f"{tag}_from_outer"),
        _wp(ax, cy, approach_yaw, f"{tag}_approach", look=True),
        _wp(ax, cy + tip, face_out, f"{tag}_tip", look=True),
        _wp(ox, cy, face_in, f"{tag}_return_outer"),
    ]


def _build_patrol() -> list[Segment]:
    pts: list[Segment] = []

    # --- Door: CENTER LANE ONLY (arm + ridgeback width jam on ±0.85 frame pts) ---
    # No look-around in doorway — spinning swings the arm into the frame.
    pts += [
        _wp(0.00, 5.25, -math.pi / 2, "spawn", look=True),
        _wp(0.00, 5.05, -math.pi / 2, "door_approach"),
        _wp(0.00, DOOR_Y, -math.pi / 2, "door_center"),
        _wp(0.00, 5.15, math.pi / 2, "door_into"),
        # Centerline only — right offset jammed the kitchen doorway lip.
        _wp(0.00, KIT_Y, math.pi / 2, "kitchen_shallow", look=True),
        _wp(0.00, DOOR_Y, -math.pi / 2, "door_exit"),
        _wp(0.00, 4.50, -math.pi / 2, "door_clear"),
        # Scan door from dining side (safe of frame), not from inside the throat
        _wp(-0.40, 4.35, math.pi / 2, "door_scan_w", look=True),
        _wp(0.40, 4.35, math.pi / 2, "door_scan_e", look=True),
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

    # --- West outer wall (OUTER → WALL), then SW / NW plant corners ---
    pts += [
        _wp(0.00, -2.20, math.pi, "to_west_wall"),
        _wp(-OUTER, CORNER_S, -math.pi / 2, "sw_outer_in"),
    ]
    pts += _plant_corner(-1.0, CORNER_S, "sw_plant")
    pts += [
        _wp(-OUTER, -2.50, math.pi / 2, "sw_pull_in"),
        _wp(-WALL, -2.20, math.pi / 2, "west_wall_s", look=True),
        _wp(-WALL, -0.75, math.pi / 2, "west_wall_mid", look=True),
        _wp(-WALL, 0.70, math.pi / 2, "west_wall_n", look=True),
        _wp(-WALL, 2.20, math.pi / 2, "west_wall_2.2", look=True),
        _wp(-OUTER, CORNER_N, math.pi / 2, "nw_outer_in"),
    ]
    pts += _plant_corner(-1.0, CORNER_N, "nw_plant")
    pts += [
        # Sweep along north dining wall under the pots (y≈3.55)
        _wp(-WALL, CORNER_N, 0.0, "north_wall_w", look=True),
        _wp(-2.00, CORNER_N, 0.0, "north_wall_w2", look=True),
        _wp(-0.80, CORNER_N, 0.0, "north_wall_c", look=True),
        _wp(0.00, 4.20, 0.0, "spine_4.2_re", look=True),
        _wp(0.80, CORNER_N, 0.0, "north_wall_e0"),
        _wp(2.00, CORNER_N, 0.0, "north_wall_e2", look=True),
        _wp(WALL, CORNER_N, 0.0, "north_wall_e", look=True),
    ]
    pts += _plant_corner(1.0, CORNER_N, "ne_plant")

    # --- East outer wall down to SE plant corner ---
    # Pull inward first so NE corner → wall does not wedge against chairs.
    pts += [
        _wp(OUTER, 2.50, -math.pi / 2, "ne_pull_in"),
        _wp(WALL, 2.20, -math.pi / 2, "east_wall_2.2", look=True),
        _wp(WALL, 0.70, -math.pi / 2, "east_wall_n", look=True),
        _wp(WALL, -0.75, -math.pi / 2, "east_wall_mid", look=True),
        _wp(WALL, -2.20, -math.pi / 2, "east_wall_s", look=True),
        _wp(OUTER, CORNER_S, -math.pi / 2, "se_outer_in"),
    ]
    pts += _plant_corner(1.0, CORNER_S, "se_plant")

    # --- South wall / south door front (between SE and SW pots) ---
    pts += [
        _wp(WALL, SOUTH_DOOR, math.pi, "south_wall_e", look=True),
        _wp(2.00, SOUTH_DOOR, math.pi, "south_wall_e2", look=True),
        _wp(0.00, SOUTH_DOOR, math.pi / 2, "south_door_front", look=True),
        _wp(-2.00, SOUTH_DOOR, math.pi, "south_wall_w2", look=True),
        _wp(-WALL, SOUTH_DOOR, math.pi, "south_wall_w", look=True),
        _wp(-OUTER, CORNER_S, math.pi / 2, "sw_rejoin"),
        _wp(0.00, -2.20, math.pi / 2, "aisle_up"),
    ]

    # --- Return on spine; through door CENTER only ---
    pts += [
        _wp(0.00, 0.70, math.pi / 2, "up_0.7"),
        _wp(0.00, 2.50, math.pi / 2, "up_2.5"),
        _wp(0.00, 4.20, math.pi / 2, "up_4.2", look=True),
        _wp(0.00, DOOR_Y, math.pi / 2, "door_return"),
        _wp(0.00, 5.25, -math.pi / 2, "kitchen_home", look=True),
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
            # Longer dwell at plant corners so slam_toolbox integrates rays.
            dwell = 0.85 if "plant" in label or "wall" in label else 0.55
            self._dwell(dwell)
        return True

    def _backoff(self, label: str) -> None:
        """Reverse briefly so a wedged base can clear furniture before skip."""
        self.get_logger().warn(f"{label} backoff reverse {STUCK_REVERSE_M:.2f}m")
        p0 = self.pose()
        if p0 is None:
            self._stop()
            return
        x0, y0, _ = p0
        t_end = time.time() + 2.5
        while time.time() < t_end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            self._cmd(STUCK_REVERSE_SPEED, 0.0)
            p = self.pose()
            if p is None:
                continue
            if math.hypot(p[0] - x0, p[1] - y0) >= STUCK_REVERSE_M:
                break
        self._stop()
        self._dwell(0.25)

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

            # Stuck: no progress (wedged on chair / kitchen doorway lip)
            if best_dist is None or dist < best_dist - 0.04:
                best_dist = dist
                stuck_since = time.time()
            elif (
                stuck_since is not None
                and time.time() - stuck_since > STUCK_NO_PROGRESS_SEC
            ):
                self._stop()
                self.get_logger().warn(
                    f"{seg.label} stuck dist={dist:.2f} "
                    f"(no progress {STUCK_NO_PROGRESS_SEC:.0f}s)"
                )
                self._backoff(seg.label)
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
            f"(table-behind + plant corners + outer walls) "
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
