"""Nav2 sequential goals along fixed rails with cross-track validation."""

from __future__ import annotations

import math
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from nova_rails.nav_bootstrap import AmclPoseTracker, make_pose, resolve_map_xy
from nova_rails.rail_geometry import Pose3, RailPolyline, prune_poses, route_poses


def _load_rail_check() -> dict:
    path = Path(get_package_share_directory("nova_rails")) / "config" / "rail_check.yaml"
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
        self._log_period = float(self._cfg.get("log_period_sec", 2.0))
        self._feedback_sec = float(self._cfg.get("require_nav_feedback_sec", 8.0))
        self._cte_grace = float(self._cfg.get("cte_grace_sec", 1.5))

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
        print(f"[{label}] 레일 {len(poses)} wp, Nav2 순차 추종 + 구간별 횡오차 검사")
        prev_xy = resolve_map_xy(self._nav, self._tf, self._tracker)
        if prev_xy is None:
            sp = routes_data.get("spawn") or routes_data.get("kitchen") or {}
            prev_xy = (float(sp.get("x", 0.0)), float(sp.get("y", 0.0)))
        for i, wp in enumerate(poses):
            sub = label if i == len(poses) - 1 else f"{label}_wp{i + 1}"
            at_dock = i == len(poses) - 1
            leg = RailPolyline(
                f"{sub}_leg",
                (prev_xy, (wp[0], wp[1])),
            )
            if not self._go_pose(wp, leg, sub, at_dock=at_dock):
                return False
            prev_xy = (wp[0], wp[1])
        return True

    def _go_pose(
        self,
        dest: Pose3,
        leg: RailPolyline,
        label: str,
        *,
        at_dock: bool,
    ) -> bool:
        max_lat = self._leg_max_lat(leg, at_dock=at_dock)
        self._nav.cancelTask()
        self._nav.result_future = None
        time.sleep(0.25)
        goal = make_pose(self._nav, *dest)
        print(f"[{label}] → ({dest[0]:.2f}, {dest[1]:.2f})")
        if not self._nav.goToPose(goal):
            print(f"[{label}] goal 거부", flush=True)
            return False
        if self._nav.result_future is None:
            print(f"[{label}] result_future 없음", flush=True)
            return False

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
            print(
                f"[{label}] {'OK' if ok else 'FAIL'} "
                f"pose=({xy[0]:.2f},{xy[1]:.2f}) err={err:.2f} m rail_max_cte={max_cte:.2f}",
                flush=True,
            )
        return ok

    def _leg_max_lat(self, leg: RailPolyline, *, at_dock: bool) -> float:
        if at_dock:
            return self._max_lat_dock
        if len(leg.points) >= 2:
            (x0, y0), (x1, y1) = leg.points[0], leg.points[-1]
            if abs(x1 - x0) > 0.45 and abs(y1 - y0) < 0.35:
                return self._max_lat_branch
        return self._max_lat
