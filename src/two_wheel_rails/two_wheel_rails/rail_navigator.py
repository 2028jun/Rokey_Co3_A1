"""Nav2 sequential goals along fixed rails with cross-track validation."""

from __future__ import annotations

import math
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from two_wheel_rails.nav_bootstrap import (
    AmclPoseTracker,
    make_pose,
    normalize_angle,
    resolve_map_xy,
    resolve_map_yaw,
    reverse_open_loop,
)
from two_wheel_rails.rail_geometry import Pose3, RailPolyline, prune_poses, route_poses

_SPINE_X_TOL = 0.15


def _segment_route(poses: list[Pose3]) -> list[tuple[str, list[Pose3]]]:
    """연속 복도(x≈0) 웨이포인트는 spine 구간으로 묶어 goThroughPoses에 사용."""
    out: list[tuple[str, list[Pose3]]] = []
    i = 0
    while i < len(poses):
        if abs(poses[i][0]) < _SPINE_X_TOL:
            start = i
            while i < len(poses) and abs(poses[i][0]) < _SPINE_X_TOL:
                i += 1
            chunk = poses[start:i]
            if len(chunk) >= 2:
                out.append(("spine", chunk))
            else:
                out.append(("leg", chunk))
        else:
            out.append(("leg", [poses[i]]))
            i += 1
    return out


def _load_rail_check() -> dict:
    path = Path(get_package_share_directory("two_wheel_rails")) / "config" / "rail_check.yaml"
    with path.open(encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


class RailNavigator:
    def __init__(
        self,
        nav: BasicNavigator,
        tf_buffer,
        tracker: AmclPoseTracker,
    ) -> None:
        self._nav = nav
        self._tf = tf_buffer
        self._tracker = tracker
        self._cfg = _load_rail_check()
        self._max_lat = float(self._cfg.get("max_lateral_error_m", 0.28))
        self._max_lat_dock = float(self._cfg.get("max_lateral_error_dock_m", 0.35))
        self._max_lat_branch = float(self._cfg.get("max_lateral_error_branch_m", 0.40))
        self._max_lat_entry = float(self._cfg.get("max_lateral_error_entry_m", 0.65))
        self._log_period = float(self._cfg.get("log_period_sec", 2.0))
        self._feedback_sec = float(self._cfg.get("require_nav_feedback_sec", 8.0))
        self._cte_grace = float(self._cfg.get("cte_grace_sec", 1.5))
        self._aisle_margin = float(self._cfg.get("aisle_margin_m", 0.22))
        self._backup_speed = float(self._cfg.get("backup_speed_mps", 0.14))
        self._backup_time = int(self._cfg.get("backup_time_allowance_sec", 45))
        self._parking_dwell = float(self._cfg.get("parking_dwell_sec", 1.0))
        self._park_open_loop = bool(self._cfg.get("park_out_open_loop", True))
        self._aisle_x_max = float(self._cfg.get("aisle_x_max_m", 0.42))
        self._align_heading_enable = bool(self._cfg.get("align_heading_at_dock", True))
        self._align_yaw_tol = float(self._cfg.get("align_heading_yaw_tol_rad", 0.08))
        self._spin_time = int(self._cfg.get("spin_time_allowance_sec", 30))
        self._spin_attempts = int(self._cfg.get("max_spin_attempts", 3))
        self._raw_pose: tuple[float, float, float] | None = None
        self._raw_pose_sub = nav.create_subscription(
            Odometry, "two_wheel/odom_raw", self._on_raw_odom, 20
        )
        self._drive_pub = nav.create_publisher(Twist, "/cmd_vel", 10)

    def _on_raw_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._raw_pose = (float(p.x), float(p.y), yaw)

    def _stop_drive(self) -> None:
        stop = Twist()
        for _ in range(8):
            self._drive_pub.publish(stop)
            rclpy.spin_once(self._nav, timeout_sec=0.03)

    def _drive_waypoint(
        self,
        dest: Pose3,
        leg: RailPolyline,
        label: str,
        max_lat: float,
    ) -> bool:
        """Forward-only differential-drive controller for the fixed rails."""
        self._nav.cancelTask()
        self._nav.result_future = None
        target_x, target_y, _ = dest
        start = time.monotonic()
        last_log = start
        max_cte = 0.0
        timeout_sec = 15.0
        if self._raw_pose is not None:
            timeout_sec += math.hypot(
                target_x - self._raw_pose[0], target_y - self._raw_pose[1]
            ) / 0.10

        while time.monotonic() - start < timeout_sec:
            rclpy.spin_once(self._nav, timeout_sec=0.02)
            pose = self._raw_pose
            if pose is None:
                time.sleep(0.05)
                continue
            x, y, yaw = pose
            dx, dy = target_x - x, target_y - y
            distance = math.hypot(dx, dy)
            if distance <= 0.12:
                self._stop_drive()
                print(f"[{label}] WP OK pose=({x:.2f},{y:.2f})", flush=True)
                return True

            target_heading = math.atan2(dy, dx)
            heading_error = normalize_angle(target_heading - yaw)
            cmd = Twist()
            cmd.angular.z = max(-0.50, min(0.50, 1.25 * heading_error))
            if abs(heading_error) < 0.55:
                speed = max(0.07, min(0.22, 0.55 * distance))
                cmd.linear.x = speed * max(0.25, math.cos(heading_error))
            self._drive_pub.publish(cmd)

            cte = leg.cross_track_m(x, y)
            max_cte = max(max_cte, cte)
            now = time.monotonic()
            if now - last_log >= self._log_period:
                print(
                    f"[{label}] pose=({x:.2f},{y:.2f}) dist={distance:.2f} "
                    f"yaw_err={math.degrees(heading_error):.1f}° cte={cte:.2f}",
                    flush=True,
                )
                last_log = now
            if now - start > self._cte_grace and cte > max_lat + 0.12:
                self._stop_drive()
                print(
                    f"[{label}] 레일 이탈 cte={cte:.2f} > {max_lat + 0.12:.2f}",
                    flush=True,
                )
                return False
            time.sleep(0.03)

        self._stop_drive()
        print(f"[{label}] WP timeout", flush=True)
        return False

    def reverse_parking_exit(self, routes_data: dict, table_id: int) -> bool:
        """전방 주차 후 /cmd_vel 직선 후진으로 복도(x≈0) 진입 (Nav2 유턴 금지)."""
        to_key = f"to_{table_id}"
        dock = route_poses(routes_data, to_key)[-1]
        dx, dy, _ = dock
        backup_m = max(0.35, abs(dx) - self._aisle_margin)
        print(
            f"[park_out] 테이블 T{table_id} — 주차 완료, {self._parking_dwell:.1f}s 후 "
            f"직선 후진 {backup_m:.2f} m (cmd_vel, 복도 진입)",
            flush=True,
        )
        deadline = time.monotonic() + self._parking_dwell
        while time.monotonic() < deadline:
            rclpy.spin_once(self._nav, timeout_sec=0.05)

        self._nav.cancelTask()
        self._nav.result_future = None
        time.sleep(0.2)
        self._nav.clearAllCostmaps()
        time.sleep(0.2)

        traveled = 0.0
        ok = False
        if self._park_open_loop:
            ok, traveled = reverse_open_loop(
                self._nav,
                self._tf,
                self._tracker,
                distance_m=backup_m,
                speed_mps=self._backup_speed,
            )
            print(
                f"[park_out] open-loop 후진 traveled≈{traveled:.2f} m "
                f"(목표 {backup_m:.2f} m, ok={ok})",
                flush=True,
            )
        if not ok and not self._park_open_loop:
            if self._nav.backup(
                backup_dist=backup_m,
                backup_speed=self._backup_speed,
                time_allowance=self._backup_time,
            ):
                ok = self._wait_simple_action("park_out")
        elif not ok:
            print("[park_out] open-loop 부족 — Nav2 BackUp 1회 시도", flush=True)
            if self._nav.backup(
                backup_dist=backup_m * 0.5,
                backup_speed=self._backup_speed,
                time_allowance=self._backup_time,
            ):
                ok = self._wait_simple_action("park_out")

        xy = resolve_map_xy(self._nav, self._tf, self._tracker)
        if xy is None:
            print("[park_out] pose 없음 — 복귀 중단 (유턴 방지)", flush=True)
            return False
        on_aisle = abs(xy[0]) <= self._aisle_x_max
        if not on_aisle:
            print(
                f"[park_out] 복도 미도달 pose=({xy[0]:.2f},{xy[1]:.2f}) "
                f"|x|≤{self._aisle_x_max:.2f} 필요 — return_kitchen 하지 않음 (유턴 방지)",
                flush=True,
            )
            return False
        print(f"[park_out] 후진 완료 pose=({xy[0]:.2f},{xy[1]:.2f})", flush=True)
        return True

    def _wait_simple_action(self, label: str) -> bool:
        if self._nav.result_future is None:
            return False
        t0 = time.monotonic()
        while not self._nav.isTaskComplete():
            if time.monotonic() - t0 > self._backup_time + 5:
                print(f"[{label}] 동작 타임아웃", flush=True)
                self._nav.cancelTask()
                return False
            rclpy.spin_once(self._nav, timeout_sec=0.05)
            time.sleep(0.05)
        ok = self._nav.getResult() == TaskResult.SUCCEEDED
        if not ok:
            print(f"[{label}] 실패 ({self._nav.getResult()})", flush=True)
        return ok

    def follow_route(
        self,
        route_key: str,
        routes_data: dict,
        *,
        label: str | None = None,
    ) -> bool:
        label = label or route_key
        poses = prune_poses(
            route_poses(routes_data, route_key),
            resolve_map_xy(self._nav, self._tf, self._tracker),
        )
        if not poses:
            return False
        print(
            f"[{label}] 레일 {len(poses)} wp, Nav2 추종 "
            f"(복도 spine 일괄) + 구간별 횡오차 검사"
        )
        prev_xy = resolve_map_xy(self._nav, self._tf, self._tracker)
        if prev_xy is None:
            sp = routes_data.get("spawn") or routes_data.get("kitchen") or {}
            prev_xy = (float(sp.get("x", 0.0)), float(sp.get("y", 0.0)))
        n = len(poses)
        idx = 0
        for seg_kind, seg in _segment_route(poses):
            if seg_kind == "spine" and len(seg) >= 2:
                leg_pts = [prev_xy] + [(p[0], p[1]) for p in seg]
                leg = RailPolyline(f"{label}_spine", tuple(leg_pts))
                end_i = idx + len(seg) - 1
                sub = f"{label}_spine" if end_i < n - 1 else label
                max_lat = self._leg_max_lat(leg, at_dock=(end_i == n - 1))
                if not self._go_through_poses(
                    seg, leg, sub, max_lat=max_lat, at_dock=(end_i == n - 1)
                ):
                    return False
                prev_xy = (seg[-1][0], seg[-1][1])
                idx += len(seg)
                continue
            for j, wp in enumerate(seg):
                global_i = idx + j
                sub = label if global_i == n - 1 else f"{label}_wp{global_i + 1}"
                at_dock = global_i == n - 1
                leg = RailPolyline(
                    f"{sub}_leg",
                    (prev_xy, (wp[0], wp[1])),
                )
                if not self._go_pose(wp, leg, sub, at_dock=at_dock):
                    return False
                prev_xy = (wp[0], wp[1])
            idx += len(seg)
        return True

    def _go_through_poses(
        self,
        dests: list[Pose3],
        leg: RailPolyline,
        label: str,
        *,
        max_lat: float,
        at_dock: bool = False,
    ) -> bool:
        ys = ", ".join(f"{d[1]:.2f}" for d in dests)
        print(f"[{label}] 2륜 직접추종 {len(dests)} wp (y: {ys})")
        ok = True
        previous = self._raw_pose[:2] if self._raw_pose else leg.points[0]
        for index, dest in enumerate(dests, 1):
            waypoint_leg = RailPolyline(
                f"{label}_{index}", (previous, (dest[0], dest[1]))
            )
            if not self._drive_waypoint(dest, waypoint_leg, f"{label}_{index}", max_lat):
                ok = False
                break
            previous = (dest[0], dest[1])
        if ok and at_dock:
            ok = self._align_yaw_at_dock(dests[-1][2], label)
        return ok

    def _go_pose(
        self,
        dest: Pose3,
        leg: RailPolyline,
        label: str,
        *,
        at_dock: bool,
    ) -> bool:
        max_lat = self._leg_max_lat(leg, at_dock=at_dock)
        print(f"[{label}] 2륜 직접추종 → ({dest[0]:.2f}, {dest[1]:.2f})")
        ok = self._drive_waypoint(dest, leg, label, max_lat)
        if ok and at_dock:
            ok = self._align_yaw_at_dock(dest[2], label)
        return ok

    def _align_yaw_at_dock(self, target_yaw: float, label: str) -> bool:
        """Align with direct differential wheel commands; never invoke Nav2 recovery."""
        if not self._align_heading_enable:
            return True
        tag = f"{label}_align"
        deadline = time.monotonic() + self._spin_time
        while time.monotonic() < deadline:
            rclpy.spin_once(self._nav, timeout_sec=0.02)
            if self._raw_pose is None:
                time.sleep(0.03)
                continue
            yaw = self._raw_pose[2]
            err = normalize_angle(target_yaw - yaw)
            if abs(err) <= self._align_yaw_tol:
                self._stop_drive()
                print(
                    f"[{tag}] OK yaw={math.degrees(yaw):.1f}° "
                    f"(목표 {math.degrees(target_yaw):.1f}°, err {math.degrees(err):.1f}°)",
                    flush=True,
                )
                return True
            cmd = Twist()
            cmd.angular.z = max(-0.45, min(0.45, 1.2 * err))
            self._drive_pub.publish(cmd)
            time.sleep(0.03)
        self._stop_drive()
        print(f"[{tag}] 방향 정렬 실패", flush=True)
        return False

    def _wait_nav_cte(
        self,
        leg: RailPolyline,
        label: str,
        max_lat: float,
        *,
        dest: Pose3,
    ) -> bool:
        t0 = time.monotonic()
        last_log = t0
        saw_feedback = False
        max_cte = 0.0

        while not self._nav.isTaskComplete():
            now = time.monotonic()
            if now - t0 > self._feedback_sec and not saw_feedback:
                print(f"[{label}] Nav2 피드백 없음 — 실패", flush=True)
                self._nav.cancelTask()
                return False

            xy = resolve_map_xy(self._nav, self._tf, self._tracker)
            if xy is not None and now - t0 >= self._cte_grace:
                cte = leg.cross_track_m(xy[0], xy[1])
                max_cte = max(max_cte, cte)
                if cte > max_lat:
                    print(
                        f"[{label}] 레일 이탈: 횡오차 {cte:.2f} m > {max_lat:.2f} m "
                        f"pose=({xy[0]:.2f},{xy[1]:.2f})",
                        flush=True,
                    )
                    self._nav.cancelTask()
                    return False
                if now - last_log >= self._log_period:
                    print(f"[{label}] 횡오차 {cte:.2f} m (max {max_cte:.2f})", flush=True)
                    last_log = now

            fb = self._nav.getFeedback()
            if fb is not None:
                saw_feedback = True
            rclpy.spin_once(self._nav, timeout_sec=0.05)
            time.sleep(0.1)

        ok = self._nav.getResult() == TaskResult.SUCCEEDED
        xy = resolve_map_xy(self._nav, self._tf, self._tracker)
        if xy:
            err = math.hypot(xy[0] - dest[0], xy[1] - dest[1])
            yaw_now = resolve_map_yaw(self._nav, self._tf, self._tracker)
            yaw_err_s = ""
            if yaw_now is not None:
                yaw_err_s = f" yaw_err={math.degrees(normalize_angle(dest[2] - yaw_now)):.1f}°"
            print(
                f"[{label}] {'OK' if ok else 'FAIL'} "
                f"pose=({xy[0]:.2f},{xy[1]:.2f}) err={err:.2f} m rail_max_cte={max_cte:.2f}{yaw_err_s}",
                flush=True,
            )
        return ok

    def _leg_max_lat(self, leg: RailPolyline, *, at_dock: bool) -> float:
        if at_dock:
            return self._max_lat_dock
        if len(leg.points) >= 2:
            (x0, y0), (x1, y1) = leg.points[0], leg.points[-1]
            if abs(x0) > 0.18 and abs(x1) < 0.15:
                return self._max_lat_entry
            if abs(x1 - x0) > 0.45 and abs(y1 - y0) < 0.35:
                return self._max_lat_branch
        return self._max_lat
