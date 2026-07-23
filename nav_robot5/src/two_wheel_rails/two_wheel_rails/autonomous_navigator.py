"""Nav2 free-path planning with straight-line simplification and rotate/drive control."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path as NavPath
from nav2_simple_commander.robot_navigator import BasicNavigator

from two_wheel_rails.nav_bootstrap import (
    AmclPoseTracker,
    make_pose,
    normalize_angle,
    resolve_map_xy,
    resolve_map_yaw,
)

Point = tuple[float, float]


@dataclass(frozen=True)
class MotionConfig:
    simplify_tolerance_m: float = 0.18
    min_segment_length_m: float = 0.25
    rotate_done_rad: float = math.radians(4.0)
    rotate_reenter_rad: float = math.radians(10.0)
    final_yaw_tolerance_rad: float = math.radians(4.0)
    waypoint_tolerance_m: float = 0.12
    max_linear_speed_mps: float = 0.22
    min_linear_speed_mps: float = 0.07
    max_angular_speed_rps: float = 0.50
    rotate_gain: float = 1.25
    drive_timeout_base_sec: float = 15.0
    drive_timeout_per_meter_sec: float = 12.0
    replan_cte_m: float = 0.35
    replan_attempts: int = 2
    log_period_sec: float = 1.0


def _point_line_distance(point: Point, start: Point, end: Point) -> float:
    px, py = point
    ax, ay = start
    bx, by = end
    dx, dy = bx - ax, by - ay
    denom = dx * dx + dy * dy
    if denom <= 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom))
    qx, qy = ax + t * dx, ay + t * dy
    return math.hypot(px - qx, py - qy)


def simplify_path(points: Iterable[Point], tolerance_m: float) -> list[Point]:
    """Ramer-Douglas-Peucker simplification preserving collision-planned geometry."""
    pts = list(points)
    if len(pts) <= 2:
        return pts

    start, end = pts[0], pts[-1]
    index = -1
    max_distance = -1.0
    for i in range(1, len(pts) - 1):
        distance = _point_line_distance(pts[i], start, end)
        if distance > max_distance:
            max_distance = distance
            index = i

    if max_distance > tolerance_m and index > 0:
        left = simplify_path(pts[: index + 1], tolerance_m)
        right = simplify_path(pts[index:], tolerance_m)
        return left[:-1] + right
    return [start, end]


def merge_short_segments(points: Iterable[Point], minimum_length_m: float) -> list[Point]:
    """Drop tiny intermediate segments while always preserving the final goal."""
    pts = list(points)
    if len(pts) <= 2:
        return pts
    merged = [pts[0]]
    for point in pts[1:-1]:
        if math.hypot(point[0] - merged[-1][0], point[1] - merged[-1][1]) >= minimum_length_m:
            merged.append(point)
    if math.hypot(pts[-1][0] - merged[-1][0], pts[-1][1] - merged[-1][1]) < minimum_length_m and len(merged) > 1:
        merged[-1] = pts[-1]
    else:
        merged.append(pts[-1])
    return merged


def nav_path_points(path: NavPath) -> list[Point]:
    return [(float(p.pose.position.x), float(p.pose.position.y)) for p in path.poses]


def load_motion_config() -> MotionConfig:
    path = Path(get_package_share_directory("two_wheel_rails")) / "config" / "autonomous_nav.yaml"
    with path.open(encoding="utf-8") as stream:
        raw = yaml.safe_load(stream) or {}
    return MotionConfig(**{k: v for k, v in raw.items() if k in MotionConfig.__dataclass_fields__})


class SimplifiedPathNavigator:
    """Plan with Nav2, then execute only rotate-in-place or straight drive commands."""

    def __init__(self, nav: BasicNavigator, tf_buffer, tracker: AmclPoseTracker) -> None:
        self._nav = nav
        self._tf = tf_buffer
        self._tracker = tracker
        self._cfg = load_motion_config()
        self._cmd_pub = nav.create_publisher(Twist, "/cmd_vel", 10)

    def _stop(self) -> None:
        stop = Twist()
        for _ in range(8):
            self._cmd_pub.publish(stop)
            rclpy.spin_once(self._nav, timeout_sec=0.03)

    def _pose(self) -> tuple[float, float, float] | None:
        xy = resolve_map_xy(self._nav, self._tf, self._tracker)
        yaw = resolve_map_yaw(self._nav, self._tf, self._tracker)
        if xy is None or yaw is None:
            return None
        return xy[0], xy[1], yaw

    def _compute_path(self, goal: PoseStamped) -> list[Point]:
        pose = self._pose()
        if pose is None:
            raise RuntimeError("map pose를 확인할 수 없습니다")
        start = make_pose(self._nav, pose[0], pose[1], pose[2])
        path = self._nav.getPath(start, goal)
        if path is None or len(path.poses) < 2:
            raise RuntimeError("Nav2가 경로를 생성하지 못했습니다")
        raw = nav_path_points(path)
        simplified = simplify_path(raw, self._cfg.simplify_tolerance_m)
        simplified = merge_short_segments(simplified, self._cfg.min_segment_length_m)
        print(
            f"[auto] Nav2 path {len(raw)} poses -> {len(simplified) - 1} straight segments",
            flush=True,
        )
        for i, point in enumerate(simplified):
            print(f"[auto] P{i}=({point[0]:.2f},{point[1]:.2f})", flush=True)
        return simplified

    def _rotate_to(self, target_yaw: float, label: str) -> bool:
        started = time.monotonic()
        last_log = started
        while time.monotonic() - started < 30.0:
            rclpy.spin_once(self._nav, timeout_sec=0.03)
            pose = self._pose()
            if pose is None:
                continue
            error = normalize_angle(target_yaw - pose[2])
            if abs(error) <= self._cfg.rotate_done_rad:
                self._stop()
                return True
            cmd = Twist()
            cmd.angular.z = max(
                -self._cfg.max_angular_speed_rps,
                min(self._cfg.max_angular_speed_rps, self._cfg.rotate_gain * error),
            )
            cmd.linear.x = 0.0
            self._cmd_pub.publish(cmd)
            now = time.monotonic()
            if now - last_log >= self._cfg.log_period_sec:
                print(f"[{label}] rotate yaw_err={math.degrees(error):.1f}deg", flush=True)
                last_log = now
        self._stop()
        return False

    def _drive_segment(self, start: Point, end: Point, label: str) -> bool:
        heading = math.atan2(end[1] - start[1], end[0] - start[0])
        length = math.hypot(end[0] - start[0], end[1] - start[1])
        if not self._rotate_to(heading, f"{label}:align"):
            return False

        started = time.monotonic()
        last_log = started
        timeout = self._cfg.drive_timeout_base_sec + length * self._cfg.drive_timeout_per_meter_sec
        while time.monotonic() - started < timeout:
            rclpy.spin_once(self._nav, timeout_sec=0.03)
            pose = self._pose()
            if pose is None:
                continue
            x, y, yaw = pose
            distance = math.hypot(end[0] - x, end[1] - y)
            if distance <= self._cfg.waypoint_tolerance_m:
                self._stop()
                return True

            heading_error = normalize_angle(heading - yaw)
            if abs(heading_error) >= self._cfg.rotate_reenter_rad:
                self._stop()
                if not self._rotate_to(heading, f"{label}:realign"):
                    return False
                continue

            cte = _point_line_distance((x, y), start, end)
            if cte > self._cfg.replan_cte_m:
                self._stop()
                print(f"[{label}] CTE {cte:.2f}m -> replan requested", flush=True)
                return False

            cmd = Twist()
            cmd.angular.z = 0.0
            cmd.linear.x = max(
                self._cfg.min_linear_speed_mps,
                min(self._cfg.max_linear_speed_mps, 0.55 * distance),
            )
            self._cmd_pub.publish(cmd)

            now = time.monotonic()
            if now - last_log >= self._cfg.log_period_sec:
                print(
                    f"[{label}] drive dist={distance:.2f}m cte={cte:.2f}m "
                    f"yaw_err={math.degrees(heading_error):.1f}deg",
                    flush=True,
                )
                last_log = now
        self._stop()
        return False

    def navigate_to(self, goal: PoseStamped, *, label: str = "goal") -> bool:
        for attempt in range(self._cfg.replan_attempts + 1):
            try:
                points = self._compute_path(goal)
            except RuntimeError as exc:
                print(f"[{label}] planning failed: {exc}", flush=True)
                return False

            failed = False
            for index, (start, end) in enumerate(zip(points, points[1:]), start=1):
                if not self._drive_segment(start, end, f"{label}:seg{index}"):
                    failed = True
                    break
            if not failed:
                q = goal.pose.orientation
                target_yaw = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                )
                return self._rotate_to(target_yaw, f"{label}:dock_yaw")

            if attempt < self._cfg.replan_attempts:
                print(f"[{label}] replanning attempt {attempt + 1}", flush=True)
                self._nav.clearAllCostmaps()
                time.sleep(0.3)
        return False
