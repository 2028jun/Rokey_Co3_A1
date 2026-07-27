"""Fleet path conflict observer (advisory only).

Same-table serialization is owned by autonomous_navigator (kitchen hold).
Isaac owns person hard-stops and contact-range peer stops.
This node must NOT call navigation pause/resume — remote 99/98 froze
pivots mid-mission and caused sticky manager FAILED / cancelled orders.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

OCCUPY_PHASES = frozenset({"occupying", "parking_out", "serving", "returning"})
STATIONARY_PHASES = frozenset({"occupying", "serving", "parking_out", "holding"})
APPROACH_PHASES = frozenset({"approaching", "holding", "navigating"})


@dataclass
class RobotIntent:
    robot_id: str
    mission_id: str = ""
    priority: float = 0.0
    pose_xy: tuple[float, float] | None = None
    polyline: list[tuple[float, float]] = field(default_factory=list)
    active: bool = False
    phase: str = "idle"
    table_id: int | None = None
    updated_at: float = 0.0


def _point_seg_distance(
    p: tuple[float, float],
    a: tuple[float, float],
    b: tuple[float, float],
) -> float:
    ax, ay = a
    bx, by = b[0] - a[0], b[1] - a[1]
    denom = bx * bx + by * by
    if denom < 1e-12:
        return math.hypot(p[0] - ax, p[1] - ay)
    t = max(0.0, min(1.0, ((p[0] - ax) * bx + (p[1] - ay) * by) / denom))
    return math.hypot(p[0] - (ax + t * bx), p[1] - (ay + t * by))


def _seg_seg_distance(
    a0: tuple[float, float],
    a1: tuple[float, float],
    b0: tuple[float, float],
    b1: tuple[float, float],
) -> float:
    best = float("inf")
    for i in range(8):
        t = i / 7.0
        p = (a0[0] + t * (a1[0] - a0[0]), a0[1] + t * (a1[1] - a0[1]))
        best = min(best, _point_seg_distance(p, b0, b1))
    for i in range(8):
        t = i / 7.0
        p = (b0[0] + t * (b1[0] - b0[0]), b0[1] + t * (b1[1] - b0[1]))
        best = min(best, _point_seg_distance(p, a0, a1))
    return best


def _trim_horizon(
    pts: list[tuple[float, float]], horizon_m: float
) -> list[tuple[float, float]]:
    if len(pts) < 2:
        return pts
    out = [pts[0]]
    traveled = 0.0
    for i in range(1, len(pts)):
        step = math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
        if traveled + step >= horizon_m:
            remain = horizon_m - traveled
            if step > 1e-6:
                ratio = remain / step
                out.append(
                    (
                        pts[i - 1][0] + ratio * (pts[i][0] - pts[i - 1][0]),
                        pts[i - 1][1] + ratio * (pts[i][1] - pts[i - 1][1]),
                    )
                )
            break
        out.append(pts[i])
        traveled += step
    return out


def _path_clearance(
    poly_a: list[tuple[float, float]],
    poly_b: list[tuple[float, float]],
) -> float:
    if len(poly_a) < 2 or len(poly_b) < 2:
        if poly_a and poly_b:
            return math.hypot(
                poly_a[0][0] - poly_b[0][0], poly_a[0][1] - poly_b[0][1]
            )
        return float("inf")
    best = float("inf")
    for i in range(len(poly_a) - 1):
        for j in range(len(poly_b) - 1):
            best = min(
                best,
                _seg_seg_distance(poly_a[i], poly_a[i + 1], poly_b[j], poly_b[j + 1]),
            )
            if best < 0.05:
                return best
    return best


def _remaining_path(intent: RobotIntent) -> list[tuple[float, float]]:
    pts = list(intent.polyline)
    if intent.pose_xy is not None:
        pts = [intent.pose_xy] + pts
    cleaned: list[tuple[float, float]] = []
    for p in pts:
        if not cleaned or math.hypot(p[0] - cleaned[-1][0], p[1] - cleaned[-1][1]) > 0.05:
            cleaned.append(p)
    return cleaned


class PathYieldCoordinator(Node):
    def __init__(self) -> None:
        super().__init__("path_yield_coordinator")
        self.declare_parameter("robot_names", ["robot1", "robot2"])
        self.declare_parameter("clearance_m", 0.55)
        self.declare_parameter("pose_clearance_m", 0.90)
        self.declare_parameter("horizon_m", 1.6)
        self.declare_parameter("engage_m", 2.0)
        self.declare_parameter("intent_stale_sec", 3.0)
        self._robots = list(self.get_parameter("robot_names").value)
        self._clearance_m = float(self.get_parameter("clearance_m").value)
        self._pose_clearance_m = float(self.get_parameter("pose_clearance_m").value)
        self._horizon_m = float(self.get_parameter("horizon_m").value)
        self._engage_m = float(self.get_parameter("engage_m").value)
        self._intent_stale_sec = float(self.get_parameter("intent_stale_sec").value)

        self._intents: dict[str, RobotIntent] = {
            name: RobotIntent(robot_id=name) for name in self._robots
        }

        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        for name in self._robots:
            self.create_subscription(
                String,
                f"/{name}/fleet/intent",
                lambda msg, robot=name: self._on_intent(robot, msg),
                qos,
            )
        self._status_pub = self.create_publisher(String, "/system/fleet_yield", qos)
        self.create_timer(0.2, self._evaluate)
        self.get_logger().info(
            f"path yield coordinator ready (advisory only) robots={self._robots} "
            f"clearance={self._clearance_m:.2f}m engage={self._engage_m:.1f}m "
            "(no remote pause; same-table hold is in navigator)"
        )

    def _on_intent(self, robot: str, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"bad intent from {robot}: {exc}")
            return
        poly = []
        for pt in payload.get("remaining_polyline") or payload.get("polyline") or []:
            try:
                poly.append((float(pt["x"]), float(pt["y"])))
            except (KeyError, TypeError, ValueError):
                continue
        pose = payload.get("pose_xy") or payload.get("pose")
        pose_xy = None
        if isinstance(pose, dict) and "x" in pose and "y" in pose:
            pose_xy = (float(pose["x"]), float(pose["y"]))
        elif isinstance(pose, (list, tuple)) and len(pose) >= 2:
            pose_xy = (float(pose[0]), float(pose[1]))
        raw_table = payload.get("table_id")
        table_id = None
        if raw_table is not None:
            try:
                table_id = int(raw_table)
            except (TypeError, ValueError):
                table_id = None
        self._intents[robot] = RobotIntent(
            robot_id=robot,
            mission_id=str(payload.get("mission_id", "")),
            priority=float(payload.get("priority", 0.0)),
            pose_xy=pose_xy,
            polyline=poly,
            active=bool(payload.get("active", False)),
            phase=str(payload.get("phase", "idle") or "idle").lower(),
            table_id=table_id,
            updated_at=time.monotonic(),
        )

    def _publish_status(self, yielding: str | None, reason: str) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "yielding": yielding or "",
                "reason": reason,
                "intents": {
                    name: {
                        "active": intent.active,
                        "mission_id": intent.mission_id,
                        "priority": intent.priority,
                        "phase": intent.phase,
                        "table_id": intent.table_id,
                        "points": len(intent.polyline),
                    }
                    for name, intent in self._intents.items()
                },
            }
        )
        self._status_pub.publish(msg)

    def _choose_yielder(self, a: str, b: str) -> str:
        """Occupying robot never yields; else later-order (lower priority) yields."""
        ia, ib = self._intents[a], self._intents[b]
        a_occ = ia.phase in OCCUPY_PHASES
        b_occ = ib.phase in OCCUPY_PHASES
        if a_occ and not b_occ:
            return b
        if b_occ and not a_occ:
            return a
        if ia.priority > ib.priority:
            return a
        if ib.priority > ia.priority:
            return b
        return a if a > b else b

    def _same_table_conflict(self, ia: RobotIntent, ib: RobotIntent) -> bool:
        if ia.table_id is None or ib.table_id is None:
            return False
        if ia.table_id != ib.table_id:
            return False
        a_occ = ia.phase in OCCUPY_PHASES
        b_occ = ib.phase in OCCUPY_PHASES
        a_app = ia.phase in APPROACH_PHASES
        b_app = ib.phase in APPROACH_PHASES
        if (a_occ and b_app) or (b_occ and a_app):
            return True
        if a_app and b_app:
            return True
        return False

    def _conflict(self, a: str, b: str) -> bool:
        ia, ib = self._intents[a], self._intents[b]
        now = time.monotonic()
        if not ia.active or not ib.active:
            return False
        if now - ia.updated_at > self._intent_stale_sec:
            return False
        if now - ib.updated_at > self._intent_stale_sec:
            return False
        if self._same_table_conflict(ia, ib):
            return True

        body_sep = float("inf")
        if ia.pose_xy and ib.pose_xy:
            body_sep = math.hypot(
                ia.pose_xy[0] - ib.pose_xy[0],
                ia.pose_xy[1] - ib.pose_xy[1],
            )
            if (
                "holding" not in (ia.phase, ib.phase)
                and body_sep <= self._pose_clearance_m
            ):
                return True

        if body_sep > self._engage_m:
            return False

        path_a = _remaining_path(ia)
        path_b = _remaining_path(ib)
        if ia.phase in STATIONARY_PHASES:
            path_a = [ia.pose_xy] if ia.pose_xy is not None else []
        if ib.phase in STATIONARY_PHASES:
            path_b = [ib.pose_xy] if ib.pose_xy is not None else []
        if len(path_a) < 2 or len(path_b) < 2:
            return False

        path_a = _trim_horizon(path_a, self._horizon_m)
        path_b = _trim_horizon(path_b, self._horizon_m)
        return _path_clearance(path_a, path_b) <= self._clearance_m

    def _evaluate(self) -> None:
        """Publish conflict status only — never pause robots."""
        active = [
            name for name in self._robots
            if self._intents[name].active
            and time.monotonic() - self._intents[name].updated_at
            <= self._intent_stale_sec
        ]
        conflict_pair = None
        if len(active) >= 2:
            for i, a in enumerate(active):
                for b in active[i + 1:]:
                    if self._conflict(a, b):
                        conflict_pair = (a, b)
                        break
                if conflict_pair:
                    break

        if conflict_pair is None:
            self._publish_status(None, "idle")
            return

        a, b = conflict_pair
        yielder = self._choose_yielder(a, b)
        keeper = b if yielder == a else a
        if self._intents[yielder].phase in OCCUPY_PHASES:
            yielder, keeper = keeper, yielder
        self._publish_status(
            yielder,
            f"advisory_{yielder}_would_yield_to_{keeper}",
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathYieldCoordinator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
