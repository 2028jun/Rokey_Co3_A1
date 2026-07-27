"""Nav2 free-path planning replaced with Orthogonal L-Path, Orthogonal A*, and Mission Command Orchestrator to Isaac Sim Direct Route State Machine."""

from __future__ import annotations

import heapq
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from std_msgs.msg import String as StringMsg
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from two_wheel_rails.nav_bootstrap import (
    AmclPoseTracker,
    make_pose,
    normalize_angle,
    resolve_map_xy,
    resolve_map_yaw,
)

Point = tuple[float, float]
GridCell = tuple[int, int]

# Directions: 0: EAST (+X), 1: NORTH (+Y), 2: WEST (-X), 3: SOUTH (-Y)
DIRECTIONS = [(1, 0), (0, 1), (-1, 0), (0, -1)]


def _point_line_distance(point: Point, start: Point, end: Point) -> float:
    px, py = point
    ax, ay = start
    bx, by = end
    dx, dy = bx - ax, by - ay
    denom = dx * dx + dy * dy
    if denom <= 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(
        0.0,
        min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom),
    )
    qx, qy = ax + t * dx, ay + t * dy
    return math.hypot(px - qx, py - qy)


def simplify_path(
    points: Iterable[Point], tolerance_m: float
) -> list[Point]:
    """Simplify geometry while preserving corners outside the tolerance."""
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


def merge_short_segments(
    points: Iterable[Point], minimum_length_m: float
) -> list[Point]:
    """Remove tiny intermediate segments while preserving the final goal."""
    pts = list(points)
    if len(pts) <= 2:
        return pts
    merged = [pts[0]]
    for point in pts[1:-1]:
        if math.hypot(
            point[0] - merged[-1][0],
            point[1] - merged[-1][1],
        ) >= minimum_length_m:
            merged.append(point)
    if (
        math.hypot(
            pts[-1][0] - merged[-1][0],
            pts[-1][1] - merged[-1][1],
        )
        < minimum_length_m
        and len(merged) > 1
    ):
        merged[-1] = pts[-1]
    else:
        merged.append(pts[-1])
    return merged


@dataclass(frozen=True)
class MotionConfig:
    sample_spacing_m: float = 0.05
    maximum_cost: int = 70
    turn_penalty: float = 8.0
    obstacle_cost_weight: float = 3.0
    corner_replan_attempts: int = 3
    rotation_clearance_radius_m: float = 0.45

    dock_approach_distance_m: float = 0.65
    replan_attempts: int = 2


def load_motion_config() -> MotionConfig:
    path = Path(get_package_share_directory("two_wheel_rails")) / "config" / "autonomous_nav.yaml"
    with path.open(encoding="utf-8") as stream:
        raw = yaml.safe_load(stream) or {}
    return MotionConfig(**{k: v for k, v in raw.items() if k in MotionConfig.__dataclass_fields__})


def remove_near_duplicate_points(points: list[Point], min_dist_m: float = 0.02) -> list[Point]:
    if len(points) <= 1:
        return points
    res = [points[0]]
    for pt in points[1:]:
        if math.hypot(pt[0] - res[-1][0], pt[1] - res[-1][1]) >= min_dist_m:
            res.append(pt)
    return res


def make_l_candidates(start: Point, goal: Point) -> list[list[Point]]:
    sx, sy = start
    gx, gy = goal
    cand1 = remove_near_duplicate_points([(sx, sy), (gx, sy), (gx, gy)])
    cand2 = remove_near_duplicate_points([(sx, sy), (sx, gy), (gx, gy)])
    return [cand1, cand2]


def get_costmap_cost(costmap: OccupancyGrid | None, x: float, y: float) -> int | None:
    if costmap is None:
        return None
    info = costmap.info
    res = info.resolution
    ox = info.origin.position.x
    oy = info.origin.position.y
    col = int(math.floor((x - ox) / res))
    row = int(math.floor((y - oy) / res))
    if col < 0 or col >= info.width or row < 0 or row >= info.height:
        return None
    idx = row * info.width + col
    val = costmap.data[idx]
    if val == -1:
        return 255
    return val


def segment_is_clear(
    start: Point,
    end: Point,
    costmap: OccupancyGrid | None,
    spacing_m: float,
    max_cost: int,
) -> tuple[bool, float, int]:
    if costmap is None:
        return False, float("inf"), 255

    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy)
    steps = max(1, math.ceil(length / spacing_m))

    costs = []
    max_c = 0
    for i in range(steps + 1):
        ratio = i / steps
        x = start[0] + ratio * dx
        y = start[1] + ratio * dy
        c = get_costmap_cost(costmap, x, y)
        if c is None or c < 0 or c > max_cost:
            return False, float("inf"), 255
        costs.append(c)
        if c > max_c:
            max_c = c

    avg_c = sum(costs) / len(costs) if costs else 0.0
    return True, avg_c, max_c


def corner_rotation_is_clear(
    corner: Point,
    costmap: OccupancyGrid | None,
    radius_m: float,
    spacing_m: float,
    max_cost: int,
) -> bool:
    if costmap is None:
        return False

    x0, y0 = corner
    steps = math.ceil((2.0 * radius_m) / spacing_m)
    for ix in range(steps + 1):
        x = x0 - radius_m + ix * spacing_m
        for iy in range(steps + 1):
            y = y0 - radius_m + iy * spacing_m
            if math.hypot(x - x0, y - y0) > radius_m:
                continue
            c = get_costmap_cost(costmap, x, y)
            if c is None or c < 0 or c > max_cost:
                return False
    return True


class SimplifiedPathNavigator:
    """Orthogonal L-path & A* navigator with Mission Command Orchestrator to Isaac Sim Direct Route State Machine."""

    def __init__(self, nav: BasicNavigator, tf_buffer, tracker: AmclPoseTracker) -> None:
        self._nav = nav
        self._tf = tf_buffer
        self._tracker = tracker
        self._cfg = load_motion_config()
        self._control_pose: tuple[float, float, float] | None = None
        self._control_twist: tuple[float, float] | None = None

        # Cache navigation readiness before the first Manager command arrives.
        # The former implementation only checked these inputs inside navigate_to(),
        # which made the first order absorb several seconds of Nav2 startup delay.
        self._inputs_ready = False
        self._readiness_logged = False

        self._costmap: OccupancyGrid | None = None
        self._costmap_sub = nav.create_subscription(
            OccupancyGrid,
            "global_costmap/costmap",
            self._on_costmap,
            1,
        )

        self._raw_odom_sub = nav.create_subscription(
            Odometry,
            "two_wheel/odom_raw",
            self._on_raw_odom,
            20,
        )

        stage_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        occupancy_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._pub_mission_cmd = nav.create_publisher(StringMsg, "two_wheel/mission_command", stage_qos)
        self._sub_mission_status = nav.create_subscription(
            StringMsg,
            "two_wheel/mission_status",
            self._on_mission_status,
            stage_qos,
        )
        # A simultaneous peer may subscribe after our first kitchen claim.
        # Latch the latest intent so the later robot cannot miss it and enter
        # the opposite side of the shared aisle at the same time.
        self._pub_fleet_intent = nav.create_publisher(
            StringMsg, "fleet/intent", occupancy_qos
        )
        self._sub_table_occupancy = nav.create_subscription(
            StringMsg,
            "/fleet/table_occupancy",
            self._on_table_occupancy,
            occupancy_qos,
        )
        ns = (nav.get_namespace() or "").strip("/")
        self._robot_id = ns or "robot"
        self._intent_priority = 0.0
        self._intent_mission_id = ""
        self._intent_polyline: list[dict] = []
        self._intent_phase = "idle"
        self._intent_table_id: int | None = None
        # robot_id -> {table_id, phase}
        self._table_occupancy: dict[str, dict] = {}
        # robot_id -> last fleet intent snapshot (for hold gating)
        self._peer_intents: dict[str, dict] = {}
        self._occupying_timer = None
        self._hold_distance_m = 2.0
        self._hold_wait_timeout_sec = 240.0
        self._corridor_hold_timeout_sec = 120.0
        self._corridor_claim_grace_sec = 2.5

        self._last_status: dict | None = None

        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._pub_l_cand_x = nav.create_publisher(NavPath, "orthogonal_path/l_candidate_x_first", path_qos)
        self._pub_l_cand_y = nav.create_publisher(NavPath, "orthogonal_path/l_candidate_y_first", path_qos)
        self._pub_selected = nav.create_publisher(NavPath, "orthogonal_path/selected", path_qos)
        self._pub_dock_approach = nav.create_publisher(NavPath, "orthogonal_path/dock_approach", path_qos)

        for peer in ("robot1", "robot2"):
            if peer == self._robot_id:
                continue
            nav.create_subscription(
                StringMsg,
                f"/{peer}/fleet/intent",
                lambda msg, name=peer: self._on_peer_intent(name, msg),
                occupancy_qos,
            )

        # Run from the node's existing executor. No extra spin thread is created.
        # Once all inputs are ready, the timer cancels itself.
        self._input_warmup_timer = nav.create_timer(
            0.2, self._warm_navigation_inputs
        )

    def _warm_navigation_inputs(self) -> None:
        if self._inputs_ready:
            return

        if self._costmap is None or self._control_pose is None:
            return

        map_pose = self._map_pose()
        if map_pose is None:
            return

        self._inputs_ready = True
        if not self._readiness_logged:
            self._readiness_logged = True
            print(
                "[auto] navigation inputs prewarmed: "
                "costmap=OK raw_odom=OK map_pose=OK",
                flush=True,
            )

        timer = getattr(self, "_input_warmup_timer", None)
        if timer is not None:
            timer.cancel()

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        self._costmap = msg

    def _on_raw_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._control_pose = (float(p.x), float(p.y), float(yaw))
        self._control_twist = (
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.angular.z),
        )

    def _on_mission_status(self, msg: StringMsg) -> None:
        try:
            payload = json.loads(msg.data)
            self._last_status = payload
        except Exception as exc:
            print(f"[navigator] status parse error: {exc}", flush=True)

    def _send_mission_command(self, payload: dict) -> None:
        msg = StringMsg()
        msg.data = json.dumps(payload)
        self._pub_mission_cmd.publish(msg)

    def _on_peer_intent(self, robot: str, msg: StringMsg) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        raw_table = payload.get("table_id")
        table_id = None
        if raw_table is not None:
            try:
                table_id = int(raw_table)
            except (TypeError, ValueError):
                table_id = None
        self._peer_intents[robot] = {
            "active": bool(payload.get("active", False)),
            "phase": str(payload.get("phase", "idle") or "idle").lower(),
            "table_id": table_id,
            "priority": float(payload.get("priority", 0.0) or 0.0),
            "pose_xy": None,
        }
        pose_xy = payload.get("pose_xy")
        if isinstance(pose_xy, dict):
            try:
                self._peer_intents[robot]["pose_xy"] = (
                    float(pose_xy["x"]),
                    float(pose_xy["y"]),
                )
            except (KeyError, TypeError, ValueError):
                pass

    def _on_table_occupancy(self, msg: StringMsg) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        robot = str(payload.get("robot_id", "")).strip()
        if not robot:
            return
        phase = str(payload.get("phase", "clear")).strip().lower()
        raw_table = payload.get("table_id")
        if phase in ("", "clear"):
            self._table_occupancy.pop(robot, None)
            # Manager already released the dock. Drop stale fleet intent so a
            # hung kitchen wait cannot keep peers frozen at the kitchen in
            # MOVING_TO_TABLE with no motion.
            intent = self._peer_intents.get(robot)
            if intent is not None:
                intent["active"] = False
                intent["phase"] = "idle"
                intent["table_id"] = None
            return
        if raw_table is None:
            return
        try:
            table_id = int(raw_table)
        except (TypeError, ValueError):
            return
        self._table_occupancy[robot] = {
            "table_id": table_id,
            "phase": phase,
        }

    def _table_owner(self, table_id: int | None) -> str | None:
        """Peer that still claims this table until kitchen return."""
        if table_id is None:
            return None
        # Keep later robots waiting through approach, serve, park-out, and
        # the whole return trip. Only 'clear' releases the dock.
        hard = (
            "serving",
            "parking_out",
            "occupying",
            "approaching",
            "returning",
        )
        for robot, info in self._table_occupancy.items():
            if robot == self._robot_id:
                continue
            if int(info.get("table_id", -1)) != int(table_id):
                continue
            if str(info.get("phase", "")).lower() in hard:
                return robot
        return None

    def _peer_blocks_table(self, table_id: int | None) -> str | None:
        """Occupancy claim is the release contract; intent is only a race fill-in.

        Hung kitchen waits used to keep publishing phase=returning after the
        manager already cleared occupancy. Peers then sat in MOVING_TO_TABLE
        at the kitchen and never left.
        """
        owner = self._table_owner(table_id)
        if owner:
            return owner
        if table_id is None:
            return None
        for robot, intent in self._peer_intents.items():
            if robot == self._robot_id:
                continue
            if not intent.get("active"):
                continue
            if intent.get("table_id") is None:
                continue
            if int(intent["table_id"]) != int(table_id):
                continue
            # If manager already cleared this peer, ignore leftover intent.
            if robot not in self._table_occupancy:
                continue
            phase = str(intent.get("phase", "")).lower()
            if phase in (
                "serving",
                "parking_out",
                "occupying",
                "approaching",
            ):
                return robot
        return None

    @staticmethod
    def _table_id_from_label(label: str) -> int | None:
        if not label.startswith("table_"):
            return None
        try:
            return int(label.split("_", 1)[1])
        except (IndexError, ValueError):
            return None

    @staticmethod
    def _table_side(table_id: int | None) -> str | None:
        """Return the shared-aisle side used by a restaurant table."""
        if table_id is None:
            return None
        tid = int(table_id)
        if tid in (0, 2):
            return "west"
        if tid in (1, 3):
            return "east"
        return None

    def _priority_should_yield(self, peer_priority: float) -> bool:
        """Return whether this robot owns the later of two orders."""
        my_priority = float(self._intent_priority or 0.0)
        peer_priority = float(peer_priority or 0.0)
        if abs(my_priority) < 1e-12 and abs(peer_priority) < 1e-12:
            return self._robot_id == "robot2"
        if abs(my_priority) < 1e-12:
            return True
        if abs(peer_priority) < 1e-12:
            return False
        if abs(my_priority - peer_priority) < 1e-9:
            return self._robot_id == "robot2"
        # Priority is -monotonic: a later order is algebraically larger.
        return my_priority > peer_priority

    @staticmethod
    def _peer_cleared_corridor(intent: dict) -> bool:
        """Return whether a peer has cleared the shared central spine."""
        if not intent.get("active"):
            return True
        phase = str(intent.get("phase", "idle") or "idle").lower()
        if phase in ("idle", "occupying", "serving"):
            return True
        pose = intent.get("pose_xy")
        if pose is not None:
            x, y = float(pose[0]), float(pose[1])
            if abs(x) >= 1.20 and phase in (
                "occupying",
                "serving",
                "parking_out",
                "approaching",
            ):
                if phase in ("occupying", "serving") or abs(x) >= 1.60:
                    return True
            if y >= 4.50:
                return phase in ("idle", "returning")
            if 0.30 <= y < 4.70 and abs(x) < 1.20:
                return False
            return phase in ("idle", "returning")
        return phase not in (
            "approaching",
            "navigating",
            "holding",
            "parking_out",
            "returning",
        )

    def _opposite_outbound_blocker(self, my_table: int | None) -> str | None:
        """Return the earlier peer still using the opposite-side aisle."""
        my_side = self._table_side(my_table)
        if my_side is None:
            return None
        peer = "robot2" if self._robot_id == "robot1" else "robot1"
        intent = self._peer_intents.get(peer) or {}
        if not intent.get("active"):
            return None
        peer_side = self._table_side(intent.get("table_id"))
        if not peer_side or peer_side == my_side:
            return None
        phase = str(intent.get("phase", "idle") or "idle").lower()
        if phase not in (
            "approaching",
            "holding",
            "navigating",
            "returning",
            "parking_out",
        ):
            return None
        if self._peer_cleared_corridor(intent):
            return None
        if not self._priority_should_yield(
            float(intent.get("priority", 0.0) or 0.0)
        ):
            return None
        return peer

    def _wait_for_corridor_clear(
        self, my_table: int | None, timeout_sec: float
    ) -> bool:
        """Keep the later robot at the kitchen until the spine is clear."""
        started = time.monotonic()
        last_log = started
        last_claim = started
        while time.monotonic() - started < timeout_sec:
            rclpy.spin_once(self._nav, timeout_sec=0.05)
            now = time.monotonic()
            if now - last_claim >= 1.0:
                pose = self._map_pose()
                polyline = (
                    [{"x": float(pose[0]), "y": float(pose[1])}]
                    if pose is not None
                    else []
                )
                self._publish_fleet_intent(
                    active=True,
                    mission_id=self._intent_mission_id
                    or f"hold_corridor_{my_table}",
                    polyline=polyline,
                    phase="holding",
                    table_id=my_table,
                )
                last_claim = now
            blocker = self._opposite_outbound_blocker(my_table)
            if blocker is None:
                return True
            if now - last_log >= 2.0:
                intent = self._peer_intents.get(blocker) or {}
                print(
                    f"[hold] corridor conflict with {blocker} "
                    f"peer_table={intent.get('table_id')} "
                    f"phase={intent.get('phase')} "
                    f"pose={intent.get('pose_xy')} my_table={my_table}; "
                    f"waiting at kitchen elapsed={now - started:.1f}s",
                    flush=True,
                )
                last_log = now
        print(
            f"[hold] corridor wait timeout after {timeout_sec:.0f}s; "
            "departing with collision monitor protection",
            flush=True,
        )
        return False

    def _wait_opposite_corridor_turn(self, my_table: int | None) -> None:
        """Serialize simultaneous west/east departures by order priority."""
        side = self._table_side(my_table)
        if side is None:
            return
        if self._intent_priority == 0.0:
            self._intent_priority = -float(time.monotonic())
        pose = self._map_pose()
        polyline = (
            [{"x": float(pose[0]), "y": float(pose[1])}]
            if pose is not None
            else []
        )
        self._publish_fleet_intent(
            active=True,
            mission_id=f"claim_corridor_{my_table}_{time.monotonic_ns()}",
            polyline=polyline,
            priority=self._intent_priority,
            phase="holding",
            table_id=my_table,
        )
        deadline = time.monotonic() + self._corridor_claim_grace_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self._nav, timeout_sec=0.05)
            if self._opposite_outbound_blocker(my_table) is not None:
                break
        blocker = self._opposite_outbound_blocker(my_table)
        if blocker is None:
            print(
                f"[hold] opposite corridor clear for table_{my_table} "
                f"(side={side}); departing",
                flush=True,
            )
            return
        print(
            f"[hold] opposite corridor busy for table_{my_table} "
            f"(side={side}, blocker={blocker}); waiting for earlier peer",
            flush=True,
        )
        self._wait_for_corridor_clear(
            my_table, self._corridor_hold_timeout_sec
        )
        print(
            f"[hold] opposite corridor released; starting table_{my_table}",
            flush=True,
        )

    def _lane_x(self) -> float | None:
        if self._robot_id == "robot1":
            return -0.70
        if self._robot_id == "robot2":
            return 0.70
        return None

    def _hold_point_for_dock(
        self, gx: float, gy: float, goal_yaw: float
    ) -> Point:
        hold_dist = max(self._hold_distance_m, self._cfg.dock_approach_distance_m + 0.8)
        hx = gx - hold_dist * math.cos(goal_yaw)
        hy = gy - hold_dist * math.sin(goal_yaw)
        lane_x = self._lane_x()
        # Prefer aisle lane when the hold sits near the corridor spine.
        if lane_x is not None and abs(hx) < 0.55:
            hx = lane_x
        return (hx, hy)

    def _apply_fleet_lane(
        self, points: list[dict] | list[Point]
    ) -> list[dict]:
        """Shift corridor (near x=0) segments onto a per-robot lane.

        robot1 keeps left of center, robot2 keeps right so simultaneous
        kitchen departures do not share the exact same spine.
        """
        lane_x = self._lane_x()

        out: list[dict] = []
        n = len(points)
        for i, pt in enumerate(points):
            if isinstance(pt, dict):
                x, y = float(pt["x"]), float(pt["y"])
            else:
                x, y = float(pt[0]), float(pt[1])
            if lane_x is not None and 0 < i < n - 1 and abs(x) < 0.55:
                x = lane_x
            out.append({"x": round(x, 4), "y": round(y, 4)})
        # Drop near-duplicates after lane snap.
        cleaned: list[dict] = []
        for p in out:
            if (
                not cleaned
                or math.hypot(p["x"] - cleaned[-1]["x"], p["y"] - cleaned[-1]["y"])
                > 0.05
            ):
                cleaned.append(p)
        return cleaned

    def _publish_fleet_intent(
        self,
        *,
        active: bool,
        mission_id: str = "",
        polyline: list[tuple[float, float]] | list[dict] | None = None,
        priority: float | None = None,
        phase: str | None = None,
        table_id: int | None = None,
    ) -> None:
        pose = self._map_pose()
        if polyline is not None:
            points: list[dict] = []
            for pt in polyline:
                if isinstance(pt, dict):
                    points.append({"x": float(pt["x"]), "y": float(pt["y"])})
                else:
                    points.append({"x": float(pt[0]), "y": float(pt[1])})
            self._intent_polyline = points
        if phase is not None:
            self._intent_phase = str(phase)
        if table_id is not None:
            self._intent_table_id = int(table_id)
        points = list(self._intent_polyline)
        # Stationary dock claim: publish pose only. Keeping the old approach
        # polyline makes path_yield think we will redrive the aisle and
        # permanently pauses peers on other tables.
        if active and self._intent_phase in (
            "occupying",
            "parking_out",
            "serving",
        ):
            if pose is not None:
                points = [{"x": float(pose[0]), "y": float(pose[1])}]
                self._intent_polyline = list(points)
            else:
                points = []
                self._intent_polyline = []
        elif active and self._intent_phase == "returning" and pose is not None:
            # Refresh path head with live pose; drop stale duplicates only.
            pass
        payload = {
            "robot_id": self._robot_id,
            "mission_id": mission_id or self._intent_mission_id,
            "priority": float(
                self._intent_priority if priority is None else priority
            ),
            "pose_xy": (
                {"x": float(pose[0]), "y": float(pose[1])}
                if pose is not None
                else None
            ),
            "remaining_polyline": points,
            "active": bool(active),
            "phase": self._intent_phase if active else "idle",
            "table_id": self._intent_table_id,
        }
        msg = StringMsg()
        msg.data = json.dumps(payload)
        self._pub_fleet_intent.publish(msg)
        if not active:
            self._intent_mission_id = ""
            self._intent_polyline = []
            self._intent_phase = "idle"
            self._intent_table_id = None

    def _stop_occupying_heartbeat(self) -> None:
        timer = self._occupying_timer
        if timer is not None:
            timer.cancel()
            self._occupying_timer = None

    def _enter_occupying(self, table_id: int, mission_id: str = "") -> None:
        """Keep fleet intent alive through PnP so peers treat the dock as busy."""
        self._stop_occupying_heartbeat()
        self._intent_table_id = int(table_id)
        self._intent_phase = "occupying"
        if mission_id:
            self._intent_mission_id = mission_id
        elif not self._intent_mission_id:
            self._intent_mission_id = f"occupy_{table_id}_{time.monotonic_ns()}"
        pose = self._map_pose()
        pose_poly = (
            [{"x": float(pose[0]), "y": float(pose[1])}]
            if pose is not None
            else []
        )
        self._publish_fleet_intent(
            active=True,
            mission_id=self._intent_mission_id,
            polyline=pose_poly,
            phase="occupying",
            table_id=table_id,
        )

        def _tick() -> None:
            if self._intent_phase != "occupying":
                self._stop_occupying_heartbeat()
                return
            live = self._map_pose()
            poly = (
                [{"x": float(live[0]), "y": float(live[1])}]
                if live is not None
                else []
            )
            self._publish_fleet_intent(
                active=True,
                mission_id=self._intent_mission_id,
                polyline=poly,
                phase="occupying",
                table_id=self._intent_table_id,
            )

        self._occupying_timer = self._nav.create_timer(0.4, _tick)

    def _clear_occupying(self) -> None:
        self._stop_occupying_heartbeat()
        if self._intent_phase in (
            "occupying",
            "parking_out",
            "holding",
            "approaching",
            "returning",
            "serving",
        ):
            self._publish_fleet_intent(active=False)

    def _wait_for_table_free(
        self, table_id: int, timeout_sec: float
    ) -> bool:
        """Block final dock until peer has fully released the table.

        Release means manager occupancy clear (kitchen arrival), not merely
        park-out. Also honor live peer fleet intents for the same table.
        """
        started = time.monotonic()
        last_log = started
        while time.monotonic() - started < timeout_sec:
            rclpy.spin_once(self._nav, timeout_sec=0.05)
            blocker = self._peer_blocks_table(table_id)
            if blocker is None:
                return True
            now = time.monotonic()
            if now - last_log >= 2.0:
                print(
                    f"[hold] waiting for table_{table_id} free "
                    f"(blocker={blocker}) elapsed={now - started:.1f}s",
                    flush=True,
                )
                last_log = now
                pose = self._map_pose()
                poly = (
                    [{"x": float(pose[0]), "y": float(pose[1])}]
                    if pose is not None
                    else []
                )
                self._publish_fleet_intent(
                    active=True,
                    mission_id=self._intent_mission_id or f"hold_{table_id}",
                    polyline=poly,
                    phase="holding",
                    table_id=table_id,
                )
        print(
            f"[hold] timeout waiting for table_{table_id} after {timeout_sec:.0f}s",
            flush=True,
        )
        return False

    def _run_route_mission(
        self,
        *,
        label: str,
        points: list[Point],
        dock: tuple[float, float, float] | None,
        finish_after_route: bool,
        final_yaw: float,
        phase: str,
        table_id: int | None,
        timeout_sec: float = 180.0,
    ) -> bool:
        for attempt in range(self._cfg.replan_attempts + 1):
            mission_id = f"{label}_{time.monotonic_ns()}_attempt_{attempt}"
            self._last_status = None
            self._spin_sleep(0.1)
            map_p = self._map_pose()
            raw_p = self._motion_pose()
            if map_p is None or raw_p is None:
                print(
                    f"[{label}] planning failed: cannot resolve map/motion pose",
                    flush=True,
                )
                return False

            if attempt > 0:
                start_pt = (map_p[0], map_p[1])
                goal_pt = points[-1] if points else start_pt
                try:
                    points = self._plan_orthogonal_path(start_pt, goal_pt)
                except RuntimeError as exc:
                    print(
                        f"[{label}] replan failed (attempt {attempt}): {exc}",
                        flush=True,
                    )
                    if attempt < self._cfg.replan_attempts:
                        self._spin_sleep(0.5)
                        continue
                    return False

            ctrl_points = [
                {"x": round(p[0], 4), "y": round(p[1], 4)} for p in points
            ]
            lane_points = self._apply_fleet_lane(ctrl_points)
            self._publish_rviz_path(
                self._pub_selected,
                [(p["x"], p["y"]) for p in lane_points],
            )
            if dock is None:
                dock_x, dock_y, dock_yaw = (
                    lane_points[-1]["x"],
                    lane_points[-1]["y"],
                    final_yaw,
                )
            else:
                dock_x, dock_y, dock_yaw = dock

            mission_payload = {
                "mission_id": mission_id,
                "kind": "execute_route",
                "points": lane_points,
                "dock": {
                    "x": round(dock_x, 4),
                    "y": round(dock_y, 4),
                    "yaw": round(dock_yaw, 4),
                },
                "finish_after_route": bool(finish_after_route),
                "final_yaw": round(final_yaw, 4),
            }
            print(
                f"[mission] sent mission={mission_id} points={len(lane_points)} "
                f"dock=({dock_x:.3f},{dock_y:.3f}) phase={phase} "
                f"lane={self._robot_id}",
                flush=True,
            )
            self._intent_mission_id = mission_id
            if self._intent_priority == 0.0 or phase in ("approaching", "holding"):
                # Preserve earlier-order priority across hold -> final approach.
                if self._intent_priority == 0.0:
                    self._intent_priority = -float(time.monotonic())
            self._publish_fleet_intent(
                active=True,
                mission_id=mission_id,
                polyline=lane_points,
                priority=self._intent_priority,
                phase=phase,
                table_id=table_id,
            )
            self._send_mission_command(mission_payload)
            ok, reason = self._wait_for_mission_completion(
                mission_id, timeout_sec, label
            )
            if ok:
                return True
            print(
                f"[{label}] mission failed or timed out ({reason}); "
                f"attempt={attempt}",
                flush=True,
            )
            if attempt < self._cfg.replan_attempts:
                self._spin_sleep(1.0)
                continue
        return False

    def _wait_for_mission_completion(
        self,
        mission_id: str,
        timeout_sec: float,
        label: str,
    ) -> tuple[bool, str]:
        started = time.monotonic()
        last_log = started
        last_intent = started

        while time.monotonic() - started < timeout_sec:
            rclpy.spin_once(self._nav, timeout_sec=0.03)

            # Keep fleet intent fresh so the yield coordinator can use live pose.
            now = time.monotonic()
            if self._intent_mission_id == mission_id and now - last_intent >= 0.4:
                # Manager soft kitchen-arrival clears occupancy while this wait
                # may still be spinning. Stop claiming the table and finish so
                # the peer is not held at the kitchen forever.
                if label == "kitchen" and self._intent_table_id is not None:
                    mine = self._table_occupancy.get(self._robot_id)
                    if mine is None or str(mine.get("phase", "")).lower() in (
                        "",
                        "clear",
                    ):
                        self._publish_fleet_intent(active=False)
                        print(
                            f"[mission] kitchen released by occupancy clear; "
                            f"finishing wait mission={mission_id}",
                            flush=True,
                        )
                        return True, "completed"
                self._publish_fleet_intent(
                    active=True,
                    mission_id=mission_id,
                    priority=self._intent_priority,
                    phase=self._intent_phase,
                    table_id=self._intent_table_id,
                )
                last_intent = now

            status = self._last_status
            if status and status.get("mission_id") == mission_id:
                state = status.get("state")
                phase = str(status.get("phase", "unknown"))
                if state == "completed" or phase in (
                    "park_out_aligned",
                    "completed",
                ):
                    print(
                        f"[mission] completed: mission={mission_id} label={label}",
                        flush=True,
                    )
                    return True, "completed"
                elif state == "accepted":
                    if time.monotonic() - last_log >= 2.0:
                        print(f"[mission] status accepted: mission={mission_id}", flush=True)
                elif state in ("failed", "cancelled"):
                    print(
                        f"[mission] status failed/cancelled: mission={mission_id} label={label} "
                        f"state={state} phase={phase}",
                        flush=True,
                    )
                    return False, state
            # Late / retried completed frames sometimes drop the exact
            # mission_id match after Isaac clears the active id. Still accept
            # a fresh completed kitchen/table status for this label.
            elif status and str(status.get("state", "")).lower() == "completed":
                sid = str(status.get("mission_id", ""))
                if label == "kitchen" and sid.startswith("kitchen_"):
                    print(
                        f"[mission] completed (id-relaxed): mission={sid} "
                        f"waiting={mission_id} label={label}",
                        flush=True,
                    )
                    return True, "completed"
                if label.startswith("table_") and sid.startswith(f"{label}_"):
                    print(
                        f"[mission] completed (id-relaxed): mission={sid} "
                        f"waiting={mission_id} label={label}",
                        flush=True,
                    )
                    return True, "completed"

            now = time.monotonic()
            if now - last_log >= 2.0:
                cur_st = status.get("state") if status else "none"
                cur_ph = status.get("phase") if status else "none"
                print(f"[mission] waiting {label}: mission={mission_id} state={cur_st} phase={cur_ph}", flush=True)
                last_log = now

        print(f"[mission] timeout waiting for {label} (mission={mission_id})", flush=True)

        cancel_payload = {"mission_id": mission_id, "kind": "cancel"}
        self._send_mission_command(cancel_payload)
        return False, "timeout"

    def _spin_sleep(self, duration_sec: float) -> None:
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self._nav, timeout_sec=0.03)

    def drive_distance(
        self,
        distance_m: float,
        speed_mps: float,
        *,
        label: str = "drive_distance",
    ) -> bool:
        self._stop_occupying_heartbeat()
        mission_id = f"{label}_{time.monotonic_ns()}"
        self._last_status = None
        self._intent_mission_id = mission_id
        phase = "parking_out" if label == "park_out" else "drive"
        saved_table = self._intent_table_id
        self._publish_fleet_intent(
            active=True,
            mission_id=mission_id,
            phase=phase,
            table_id=saved_table,
        )
        self._spin_sleep(0.25)
        self._send_mission_command(
            {
                "mission_id": mission_id,
                "kind": "drive_distance",
                "distance": abs(float(distance_m)),
                "speed": float(speed_mps),
            }
        )
        timeout = abs(float(distance_m)) / max(abs(float(speed_mps)), 0.05) + 8.0
        # park-out includes reverse plus opposite-heading alignment in Isaac.
        # The old timeout covered only the straight-line motion and cancelled
        # the mission during the 180-degree pivot.
        if label == "park_out":
            timeout += 30.0
        ok, _reason = self._wait_for_mission_completion(
            mission_id, timeout, label
        )
        if label == "park_out":
            if ok:
                # Keep the table claim through the kitchen return so same-table
                # peers stay on hold until we arrive at the kitchen.
                self._intent_table_id = saved_table
                self._intent_phase = "parking_out"
                pose = self._map_pose()
                poly = (
                    [{"x": float(pose[0]), "y": float(pose[1])}]
                    if pose is not None
                    else []
                )
                self._publish_fleet_intent(
                    active=True,
                    mission_id=mission_id,
                    polyline=poly,
                    phase="parking_out",
                    table_id=saved_table,
                )
            else:
                # Failed park-out must not leave a sticky active claim that
                # permanently pauses the other robot on a different route.
                self._clear_occupying()
        elif not ok:
            self._publish_fleet_intent(active=False, mission_id=mission_id)
        return ok

    def navigate_to(
        self,
        goal: PoseStamped,
        *,
        label: str = "goal",
        position_then_align: bool = False,
    ) -> bool:
        if not self._wait_for_navigation_inputs(timeout_sec=8.0):
            print(f"[{label}] navigation aborted: required inputs are unavailable", flush=True)
            return False

        gx = goal.pose.position.x
        gy = goal.pose.position.y
        q = goal.pose.orientation
        goal_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        table_id = self._table_id_from_label(label)
        returning_table = self._intent_table_id if label == "kitchen" else None

        if label == "kitchen":
            # Keep priority + table claim visible while returning so the later
            # robot stays on hold until manager clears occupancy at kitchen.
            self._stop_occupying_heartbeat()
            if returning_table is not None:
                self._intent_table_id = returning_table
                self._intent_phase = "returning"
        elif position_then_align and table_id is None:
            self._clear_occupying()
            self._intent_priority = 0.0

        app_dist = self._cfg.dock_approach_distance_m
        approach_pt = (
            (gx, gy)
            if position_then_align
            else (
                gx - app_dist * math.cos(goal_yaw),
                gy - app_dist * math.sin(goal_yaw),
            )
        )
        self._publish_rviz_path(self._pub_dock_approach, [approach_pt, (gx, gy)])

        # Same-table serialization: stay in the kitchen seat until the peer
        # has fully returned home. Driving to an aisle hold while the peer is
        # still returning caused head-on pending deadlocks.
        if table_id is not None and not position_then_align:
            for _ in range(10):
                rclpy.spin_once(self._nav, timeout_sec=0.02)
            blocker = self._peer_blocks_table(table_id)
            if not blocker:
                deadline = time.monotonic() + 0.8
                while time.monotonic() < deadline and not blocker:
                    rclpy.spin_once(self._nav, timeout_sec=0.05)
                    blocker = self._peer_blocks_table(table_id)
            if blocker:
                if self._intent_priority == 0.0:
                    self._intent_priority = -float(time.monotonic())
                print(
                    f"[hold] table_{table_id} blocked by {blocker}; "
                    "waiting at kitchen until peer returns home",
                    flush=True,
                )
                pose = self._map_pose()
                poly = (
                    [{"x": float(pose[0]), "y": float(pose[1])}]
                    if pose is not None
                    else []
                )
                self._publish_fleet_intent(
                    active=True,
                    mission_id=f"hold_kitchen_{table_id}_{time.monotonic_ns()}",
                    polyline=poly,
                    priority=self._intent_priority,
                    phase="holding",
                    table_id=table_id,
                )
                if not self._wait_for_table_free(
                    table_id, self._hold_wait_timeout_sec
                ):
                    self._publish_fleet_intent(active=False)
                    return False
                print(
                    f"[hold] table_{table_id} clear (peer home); "
                    "starting table approach",
                    flush=True,
                )

            # Opposite-side routes share the central spine. Publish a latched
            # claim and let only the earlier order leave the kitchen first.
            self._wait_opposite_corridor_turn(table_id)

        map_p = self._map_pose()
        if map_p is None:
            print(f"[{label}] planning failed: cannot resolve map pose", flush=True)
            return False
        start_pt = (map_p[0], map_p[1])
        try:
            points = self._plan_orthogonal_path(start_pt, approach_pt)
        except RuntimeError as exc:
            print(f"[{label}] orthogonal planning failed: {exc}", flush=True)
            return False

        if self._intent_priority == 0.0:
            self._intent_priority = -float(time.monotonic())

        if label == "kitchen":
            phase = "returning"
            mission_table = returning_table
        elif table_id is not None:
            phase = "approaching"
            mission_table = table_id
        else:
            phase = "navigating"
            mission_table = None

        ok = self._run_route_mission(
            label=label,
            points=points,
            dock=(gx, gy, normalize_angle(goal_yaw)),
            finish_after_route=position_then_align,
            final_yaw=normalize_angle(goal_yaw),
            phase=phase,
            table_id=mission_table,
        )
        if ok and table_id is not None and not position_then_align:
            print(
                f"[{label}] docked; keeping occupying intent through serving",
                flush=True,
            )
            self._enter_occupying(table_id, mission_id=self._intent_mission_id)
            return True

        if ok:
            self._publish_fleet_intent(active=False)
            self._intent_priority = 0.0
            print(f"[{label}] entire autonomous mission completed successfully!", flush=True)
            return True

        self._publish_fleet_intent(active=False)
        return False
    def _wait_for_navigation_inputs(self, timeout_sec: float = 8.0) -> bool:
        # Fast path for normal operation: startup readiness was already cached
        # by _warm_navigation_inputs before the order arrived.
        if self._inputs_ready:
            return True

        started = time.monotonic()
        last_log = 0.0

        while time.monotonic() - started < timeout_sec:
            rclpy.spin_once(self._nav, timeout_sec=0.03)

            costmap_ready = self._costmap is not None
            raw_odom_ready = self._control_pose is not None

            if costmap_ready and raw_odom_ready:
                map_pose = self._map_pose()
                if map_pose is not None:
                    self._inputs_ready = True
                    print(
                        "[auto] navigation inputs ready: costmap=OK raw_odom=OK map_pose=OK",
                        flush=True,
                    )
                    timer = getattr(self, "_input_warmup_timer", None)
                    if timer is not None:
                        timer.cancel()
                    return True

            now = time.monotonic()
            if now - last_log >= 1.0:
                print(
                    "[auto] waiting for navigation inputs: "
                    f"costmap={'OK' if costmap_ready else 'WAIT'} "
                    f"raw_odom={'OK' if raw_odom_ready else 'WAIT'}",
                    flush=True,
                )
                last_log = now

        print(
            "[auto] navigation input timeout: "
            f"costmap={self._costmap is not None} "
            f"raw_odom={self._control_pose is not None} "
            f"map_pose={self._map_pose() is not None}",
            flush=True,
        )
        return False

    def _map_pose(self) -> tuple[float, float, float] | None:
        xy = resolve_map_xy(self._nav, self._tf, self._tracker)
        yaw = resolve_map_yaw(self._nav, self._tf, self._tracker)
        if xy is None or yaw is None:
            return None
        return xy[0], xy[1], yaw

    def _motion_pose(self) -> tuple[float, float, float] | None:
        return self._control_pose

    def _publish_rviz_path(self, pub, points: list[Point]) -> None:
        path = NavPath()
        path.header.frame_id = "map"
        path.header.stamp = self._nav.get_clock().now().to_msg()
        for pt in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        pub.publish(path)

    def _evaluate_l_candidate(self, points: list[Point]) -> tuple[bool, float]:
        if self._costmap is None:
            return False, float("inf")

        tot_len = 0.0
        tot_avg_cost = 0.0
        max_cost_all = 0

        for start, end in zip(points, points[1:]):
            tot_len += math.hypot(end[0] - start[0], end[1] - start[1])
            is_clear, avg_c, max_c = segment_is_clear(
                start, end, self._costmap, self._cfg.sample_spacing_m, self._cfg.maximum_cost
            )
            if not is_clear:
                return False, float("inf")
            tot_avg_cost += avg_c
            if max_c > max_cost_all:
                max_cost_all = max_c

        for pt in points[1:-1]:
            if not corner_rotation_is_clear(
                pt,
                self._costmap,
                self._cfg.rotation_clearance_radius_m,
                self._cfg.sample_spacing_m,
                self._cfg.maximum_cost,
            ):
                return False, float("inf")

        score = tot_len + (tot_avg_cost * 0.05) + (max_cost_all * 0.02) + (len(points) * 0.5)
        return True, score

    def _orthogonal_astar(self, start: Point, goal: Point, forbidden_cells: set[GridCell]) -> list[Point] | None:
        if self._costmap is None:
            return None

        info = self._costmap.info
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y

        def world_to_cell(pt: Point) -> GridCell:
            return (
                int(math.floor((pt[0] - ox) / res)),
                int(math.floor((pt[1] - oy) / res)),
            )

        def cell_to_world(cell: GridCell) -> Point:
            return (ox + (cell[0] + 0.5) * res, oy + (cell[1] + 0.5) * res)

        start_cell = world_to_cell(start)
        goal_cell = world_to_cell(goal)

        open_set = []
        for d in range(4):
            heapq.heappush(open_set, (0.0, 0.0, start_cell, d))

        came_from: dict[tuple[GridCell, int], tuple[GridCell, int]] = {}
        g_scores: dict[tuple[GridCell, int], float] = {}

        for d in range(4):
            g_scores[(start_cell, d)] = 0.0

        def heuristic(c: GridCell) -> float:
            return (abs(c[0] - goal_cell[0]) + abs(c[1] - goal_cell[1])) * res

        found_state = None
        while open_set:
            f, g, curr_cell, curr_dir = heapq.heappop(open_set)

            if curr_cell == goal_cell:
                found_state = (curr_cell, curr_dir)
                break

            if g > g_scores.get((curr_cell, curr_dir), float("inf")):
                continue

            for next_dir, (dx, dy) in enumerate(DIRECTIONS):
                nxt_cell = (curr_cell[0] + dx, curr_cell[1] + dy)
                if nxt_cell in forbidden_cells:
                    continue

                nxt_pt = cell_to_world(nxt_cell)
                cost_val = get_costmap_cost(self._costmap, nxt_pt[0], nxt_pt[1])
                if cost_val is None or cost_val < 0 or cost_val > self._cfg.maximum_cost:
                    continue

                move_cost = res
                turn_cost = self._cfg.turn_penalty if next_dir != curr_dir else 0.0
                obs_cost = (cost_val / 100.0) * self._cfg.obstacle_cost_weight

                new_g = g + move_cost + turn_cost + obs_cost
                nxt_state = (nxt_cell, next_dir)

                if new_g < g_scores.get(nxt_state, float("inf")):
                    g_scores[nxt_state] = new_g
                    came_from[nxt_state] = (curr_cell, curr_dir)
                    f_new = new_g + heuristic(nxt_cell)
                    heapq.heappush(open_set, (f_new, new_g, nxt_cell, next_dir))

        if found_state is None:
            return None

        path_cells = []
        curr = found_state
        while curr in came_from:
            path_cells.append(curr[0])
            curr = came_from[curr]
        path_cells.append(start_cell)
        path_cells.reverse()

        first_w = cell_to_world(path_cells[0])
        last_w = cell_to_world(path_cells[-1])

        raw_pts = [
            start,
            (first_w[0], start[1]),
            first_w,
        ]
        for c in path_cells[1:-1]:
            raw_pts.append(cell_to_world(c))

        raw_pts.extend([
            last_w,
            (goal[0], last_w[1]),
            goal,
        ])

        compressed = [raw_pts[0]]
        for i in range(1, len(raw_pts) - 1):
            p_prev, p_curr, p_next = compressed[-1], raw_pts[i], raw_pts[i + 1]
            dir1 = (p_curr[0] - p_prev[0], p_curr[1] - p_prev[1])
            dir2 = (p_next[0] - p_curr[0], p_next[1] - p_curr[1])
            angle_diff = abs(math.atan2(dir1[1], dir1[0]) - math.atan2(dir2[1], dir2[0]))
            if angle_diff > 0.1:
                compressed.append(p_curr)
        compressed.append(goal)
        return remove_near_duplicate_points(compressed)

    def _plan_orthogonal_path(self, start: Point, goal: Point) -> list[Point]:
        if self._costmap is None:
            raise RuntimeError("global costmap (/global_costmap/costmap) has not been received yet")

        l_cands = make_l_candidates(start, goal)
        if len(l_cands) >= 1:
            self._publish_rviz_path(self._pub_l_cand_x, l_cands[0])
        if len(l_cands) >= 2:
            self._publish_rviz_path(self._pub_l_cand_y, l_cands[1])

        best_cand = None
        best_score = float("inf")
        for cand in l_cands:
            is_valid, score = self._evaluate_l_candidate(cand)
            if is_valid and score < best_score:
                best_score = score
                best_cand = cand

        if best_cand is not None:
            print(f"[auto] L-path candidate selected (score={best_score:.2f})", flush=True)
            self._publish_rviz_path(self._pub_selected, best_cand)
            return best_cand

        print("[auto] L-path candidates blocked; running Orthogonal A*...", flush=True)

        forbidden_cells: set[GridCell] = set()
        info = self._costmap.info
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        rad_cells = math.ceil(self._cfg.rotation_clearance_radius_m / res)

        for attempt in range(self._cfg.corner_replan_attempts + 1):
            astar_pts = self._orthogonal_astar(start, goal, forbidden_cells)
            if astar_pts is None:
                raise RuntimeError("Orthogonal A* failed to find path")

            invalid_corner = None
            for corner in astar_pts[1:-1]:
                if not corner_rotation_is_clear(
                    corner,
                    self._costmap,
                    self._cfg.rotation_clearance_radius_m,
                    self._cfg.sample_spacing_m,
                    self._cfg.maximum_cost,
                ):
                    invalid_corner = corner
                    break

            if invalid_corner is None:
                print(f"[auto] Orthogonal A* path selected (attempt {attempt})", flush=True)
                self._publish_rviz_path(self._pub_selected, astar_pts)
                return astar_pts

            center_col = int(math.floor((invalid_corner[0] - ox) / res))
            center_row = int(math.floor((invalid_corner[1] - oy) / res))

            for dr in range(-rad_cells, rad_cells + 1):
                for dc in range(-rad_cells, rad_cells + 1):
                    if math.hypot(dr, dc) <= rad_cells:
                        forbidden_cells.add((center_col + dc, center_row + dr))

            print(f"[auto] Corner at ({invalid_corner[0]:.2f}, {invalid_corner[1]:.2f}) uncleared; expanding forbidden radius and replanning...", flush=True)

        raise RuntimeError("Orthogonal path planning failed all corner clearance checks")
