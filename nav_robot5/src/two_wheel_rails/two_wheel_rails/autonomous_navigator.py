"""Nav2 free-path planning replaced with Orthogonal L-Path, Orthogonal A*, and Mission Command Orchestrator to Isaac Sim Direct Route State Machine."""

from __future__ import annotations

import heapq
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path

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

        self._costmap: OccupancyGrid | None = None
        self._costmap_sub = nav.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self._on_costmap,
            1,
        )

        self._raw_odom_sub = nav.create_subscription(
            Odometry,
            "/two_wheel/odom_raw",
            self._on_raw_odom,
            20,
        )

        stage_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._pub_mission_cmd = nav.create_publisher(StringMsg, "/two_wheel/mission_command", stage_qos)
        self._sub_mission_status = nav.create_subscription(
            StringMsg,
            "/two_wheel/mission_status",
            self._on_mission_status,
            stage_qos,
        )

        self._last_status: dict | None = None

        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._pub_l_cand_x = nav.create_publisher(NavPath, "/orthogonal_path/l_candidate_x_first", path_qos)
        self._pub_l_cand_y = nav.create_publisher(NavPath, "/orthogonal_path/l_candidate_y_first", path_qos)
        self._pub_selected = nav.create_publisher(NavPath, "/orthogonal_path/selected", path_qos)
        self._pub_dock_approach = nav.create_publisher(NavPath, "/orthogonal_path/dock_approach", path_qos)

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

    def _wait_for_mission_completion(
        self,
        mission_id: str,
        timeout_sec: float,
        label: str,
    ) -> tuple[bool, str]:
        started = time.monotonic()
        last_log = started

        while time.monotonic() - started < timeout_sec:
            rclpy.spin_once(self._nav, timeout_sec=0.03)

            status = self._last_status
            if status and status.get("mission_id") == mission_id:
                state = status.get("state")
                phase = status.get("phase", "unknown")
                if state == "completed":
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

    def _wait_for_navigation_inputs(self, timeout_sec: float = 8.0) -> bool:
        started = time.monotonic()
        last_log = 0.0

        while time.monotonic() - started < timeout_sec:
            rclpy.spin_once(self._nav, timeout_sec=0.03)

            costmap_ready = self._costmap is not None
            raw_odom_ready = self._control_pose is not None

            if costmap_ready and raw_odom_ready:
                map_pose = self._map_pose()
                if map_pose is not None:
                    print(
                        "[auto] navigation inputs ready: costmap=OK raw_odom=OK map_pose=OK",
                        flush=True,
                    )
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

    def navigate_to(self, goal: PoseStamped, *, label: str = "goal") -> bool:
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

        app_dist = self._cfg.dock_approach_distance_m
        approach_pt = (
            gx - app_dist * math.cos(goal_yaw),
            gy - app_dist * math.sin(goal_yaw),
        )
        self._publish_rviz_path(self._pub_dock_approach, [approach_pt, (gx, gy)])

        mission_id = f"{label}_{int(time.monotonic())}"

        for attempt in range(self._cfg.replan_attempts + 1):
            map_p = self._map_pose()
            raw_p = self._motion_pose()
            if map_p is None or raw_p is None:
                print(f"[{label}] planning failed: cannot resolve map/motion pose", flush=True)
                return False

            start_pt = (map_p[0], map_p[1])
            try:
                points = self._plan_orthogonal_path(start_pt, approach_pt)
            except RuntimeError as exc:
                print(f"[{label}] orthogonal planning failed (attempt {attempt}): {exc}", flush=True)
                if attempt < self._cfg.replan_attempts:
                    self._spin_sleep(0.5)
                    continue
                return False

            yaw_offset = normalize_angle(raw_p[2] - map_p[2])
            c_rot = math.cos(yaw_offset)
            s_rot = math.sin(yaw_offset)

            # Transform Map Points to Control Frame Points for Isaac Execution
            ctrl_points = []
            for p in points:
                dx_map = p[0] - map_p[0]
                dy_map = p[1] - map_p[1]
                ctrl_x = raw_p[0] + c_rot * dx_map - s_rot * dy_map
                ctrl_y = raw_p[1] + s_rot * dx_map + c_rot * dy_map
                ctrl_points.append({"x": round(ctrl_x, 4), "y": round(ctrl_y, 4)})

            dx_dock = gx - map_p[0]
            dy_dock = gy - map_p[1]
            ctrl_dock_x = raw_p[0] + c_rot * dx_dock - s_rot * dy_dock
            ctrl_dock_y = raw_p[1] + s_rot * dx_dock + c_rot * dy_dock
            ctrl_dock_yaw = normalize_angle(goal_yaw + yaw_offset)

            mission_payload = {
                "mission_id": mission_id,
                "kind": "execute_route",
                "points": ctrl_points,
                "dock": {
                    "x": round(ctrl_dock_x, 4),
                    "y": round(ctrl_dock_y, 4),
                    "yaw": round(ctrl_dock_yaw, 4),
                },
            }

            print(
                f"[mission] sent mission={mission_id} points={len(ctrl_points)} "
                f"dock=({ctrl_dock_x:.3f},{ctrl_dock_y:.3f})",
                flush=True,
            )
            self._send_mission_command(mission_payload)

            ok, reason = self._wait_for_mission_completion(mission_id, 180.0, label)
            if ok:
                print(f"[{label}] entire autonomous mission completed successfully!", flush=True)
                return True

            print(f"[{label}] mission failed or timed out ({reason}); attempt={attempt}", flush=True)
            if attempt < self._cfg.replan_attempts:
                self._spin_sleep(1.0)
                continue

        return False
