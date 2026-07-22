"""Load routes.yaml and compute cross-track error to rail polylines."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory

Pose3 = tuple[float, float, float]


@dataclass(frozen=True)
class RailPolyline:
    name: str
    points: tuple[tuple[float, float], ...]

    def cross_track_m(self, x: float, y: float) -> float:
        if len(self.points) < 2:
            if self.points:
                return math.hypot(x - self.points[0][0], y - self.points[0][1])
            return float("inf")
        best = float("inf")
        for i in range(len(self.points) - 1):
            x0, y0 = self.points[i]
            x1, y1 = self.points[i + 1]
            best = min(best, _point_segment_distance(x, y, x0, y0, x1, y1))
        return best


def _point_segment_distance(
    px: float, py: float, x0: float, y0: float, x1: float, y1: float
) -> float:
    dx, dy = x1 - x0, y1 - y0
    len2 = dx * dx + dy * dy
    if len2 < 1e-12:
        return math.hypot(px - x0, py - y0)
    t = max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / len2))
    qx = x0 + t * dx
    qy = y0 + t * dy
    return math.hypot(px - qx, py - qy)


def _pt(d: dict[str, Any]) -> Pose3:
    return float(d["x"]), float(d["y"]), float(d["yaw"])


def load_routes(path: Path | None = None) -> dict[str, Any]:
    if path is None:
        path = Path(get_package_share_directory("two_wheel_rails")) / "config" / "routes.yaml"
    with path.open(encoding="utf-8") as f:
        return yaml.safe_load(f)


def route_poses(data: dict[str, Any], route_key: str) -> list[Pose3]:
    routes = data.get("routes") or {}
    if route_key not in routes:
        raise KeyError(route_key)
    route = routes[route_key]
    wps: list[Pose3] = []
    if route_key.startswith("from_"):
        for p in route.get("spine") or []:
            wps.append(_pt(p))
    else:
        for p in route.get("spine") or []:
            wps.append(_pt(p))
        if "pre_dock" in route:
            pd = _pt(route["pre_dock"])
            if not wps or math.hypot(wps[-1][0] - pd[0], wps[-1][1] - pd[1]) > 0.12:
                wps.append(pd)
            else:
                # Same point as last spine — keep position, use branch yaw for turn.
                wps[-1] = (wps[-1][0], wps[-1][1], pd[2])
    dock = _pt(route["dock"])
    if not wps or math.hypot(wps[-1][0] - dock[0], wps[-1][1] - dock[1]) > 0.12:
        wps.append(dock)
    else:
        wps[-1] = (dock[0], dock[1], dock[2])
    return wps


def route_polyline(data: dict[str, Any], route_key: str) -> RailPolyline:
    poses = route_poses(data, route_key)
    return RailPolyline(
        route_key,
        tuple((p[0], p[1]) for p in poses),
    )


def prune_poses(
    poses: list[Pose3],
    xy: tuple[float, float] | None,
    near_m: float = 0.55,
) -> list[Pose3]:
    if xy is None or not poses:
        return poses
    px, py = xy
    kept = list(poses)
    while len(kept) > 1:
        wx, wy, _ = kept[0]
        d = math.hypot(px - wx, py - wy)
        if d > near_m:
            break
        # 주방(x≈0.21)에서 복도 정렬(x≈0) 웨이포인트는 가깝더라도 유지
        if abs(wx) < 0.12 and abs(px) > 0.18:
            break
        kept.pop(0)
    return kept
