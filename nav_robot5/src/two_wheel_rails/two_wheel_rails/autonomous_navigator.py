"""Nav2 free-path planning replaced with Orthogonal L-Path, Orthogonal A*, and Stage Command Orchestrator to Isaac Sim Physics Executor."""

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
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

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


@dataclass
class AxisStage:
    kind: str           # "axis_x" or "axis_y"
    value: float        # Target raw-control axis coordinate
    yaw: float          # Heading for this stage (Control Frame)
    speed: float
    is_last: bool
    axis_sign: float = 1.0


@dataclass(frozen=True)
class MotionConfig:
    sample_spacing_m: float = 0.05
    maximum_cost: int = 70
    turn_penalty: float = 8.0
    obstacle_cost_weight: float = 3.0
    corner_replan_attempts: int = 3
    rotation_clearance_radius_m: float = 0.45

    dock_approach_distance_m: float = 0.60
    dock_step_distance_m: float = 0.08
    dock_max_linear_speed_mps: float = 0.10
    dock_min_linear_speed_mps: float = 0.02
    dock_xy_tolerance_m: float = 0.04
    dock_yaw_tolerance_rad: float = math.radians(2.0)
    dock_realign_threshold_rad: float = math.radians(6.0)

    rotate_done_rad: float = math.radians(2.0)
    rotate_reenter_rad: float = math.radians(5.0)
    final_yaw_tolerance_rad: float = math.radians(2.0)
    axis_tolerance_m: float = 0.03
    final_approach_axis_tolerance_m: float = 0.03
    max_linear_speed_mps: float = 0.22
    max_angular_speed_rps: float = 0.50
    replan_cte_m: float = 0.15
    replan_attempts: int = 2
    log_period_sec: float = 1.0


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
    if val == -1:  # unknown
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
    """Orthogonal L-path & A* navigator with Stage Command Orchestrator to Isaac Sim Physics Executor."""

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

        self._pub_stage_cmd = nav.create_publisher(StringMsg, "/two_wheel/stage_command", stage_qos)
        self._sub_stage_status = nav.create_subscription(
            StringMsg,
            "/two_wheel/stage_status",
            self._on_stage_status,
            stage_qos,
        )

        self._last_status: dict | None = None
        self._sequence_counter = 0

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

    def _on_stage_status(self, msg: StringMsg) -> None:
        try:
            payload = json.loads(msg.data)
            self._last_status = payload
        except Exception as exc:
            print(f"[navigator] status parse error: {exc}", flush=True)

    def _send_stage_command(self, payload: dict) -> None:
        msg = StringMsg()
        msg.data = json.dumps(payload)
        self._pub_stage_cmd.publish(msg)

    def _wait_for_stage_completion(
        self,
        mission_id: str,
        sequence: int,
        timeout_sec: float,
        label: str,
    ) -> tuple[bool, str]:
        started = time.monotonic()
        last_log = started

        while time.monotonic() - started < timeout_sec:
            rclpy.spin_once(self._nav, timeout_sec=0.03)

            status = self._last_status
            if status and status.get("mission_id") == mission_id and status.get("sequence") == sequence:
                state = status.get("state")
                if state == "completed":
                    print(
                        f"[stage] status completed: mission={mission_id} seq={sequence} label={label} "
                        f"error={status.get('error', 0.0)}",
                        flush=True,
                    )
                    return True, "completed"
                elif state == "accepted":
                    if time.monotonic() - last_log >= 2.0:
                        print(f"[stage] status accepted: mission={mission_id} seq={sequence}", flush=True)
                elif state in ("failed", "cancelled"):
                    print(
                        f"[stage] status failed/cancelled: mission={mission_id} seq={sequence} label={label} "
                        f"state={state}",
                        flush=True,
                    )
                    return False, state

            now = time.monotonic()
            if now - last_log >= 2.0:
                cur_st = status.get("state") if status else "none"
                print(f"[stage] waiting {label}: mission={mission_id} seq={sequence} status={cur_st}", flush=True)
                last_log = now

        print(f"[stage] timeout waiting for {label} (mission={mission_id} seq={sequence})", flush=True)

        cancel_payload = {"mission_id": mission_id, "sequence": 99, "kind": "cancel"}
        self._send_stage_command(cancel_payload)
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
            if map_p is None:
                print(f"[{label}] planning failed: cannot resolve map pose", flush=True)
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

            failed = False
            total_segs = len(points) - 1

            for index, (s, e) in enumerate(zip(points, points[1:]), start=1):
                is_last = (index == total_segs)

                # Read fresh pose before generating next stage
                map_p_curr = self._map_pose()
                raw_p_curr = self._motion_pose()
                if map_p_curr is None or raw_p_curr is None:
                    failed = True
                    break

                yaw_offset = normalize_angle(raw_p_curr[2] - map_p_curr[2])
                dx_end = e[0] - map_p_curr[0]
                dy_end = e[1] - map_p_curr[1]

                c_rot = math.cos(yaw_offset)
                s_rot = math.sin(yaw_offset)

                control_start = (raw_p_curr[0], raw_p_curr[1])
                control_end = (
                    raw_p_curr[0] + c_rot * dx_end - s_rot * dy_end,
                    raw_p_curr[1] + s_rot * dx_end + c_rot * dy_end,
                )
                dx = control_end[0] - control_start[0]
                dy = control_end[1] - control_start[1]
                segment_length = math.hypot(dx, dy)

                if segment_length < 0.02:
                    print(f"[{label}:seg{index}] segment skipped: length={segment_length:.3f}m", flush=True)
                    continue

                if abs(dx) >= abs(dy):
                    kind = "axis_x"
                else:
                    kind = "axis_y"

                map_dx = e[0] - s[0]
                map_dy = e[1] - s[1]

                if abs(map_dx) >= abs(map_dy):
                    map_axis_yaw = 0.0 if map_dx >= 0.0 else math.pi
                else:
                    map_axis_yaw = math.pi / 2.0 if map_dy >= 0.0 else -math.pi / 2.0

                stage_yaw = normalize_angle(map_axis_yaw + yaw_offset)
                target_val = control_end[0] if kind == "axis_x" else control_end[1]

                # Step 1: Send Pivot Stage first to align heading
                self._sequence_counter += 1
                seq_pivot = self._sequence_counter
                pivot_payload = {
                    "mission_id": mission_id,
                    "sequence": seq_pivot,
                    "kind": "pivot",
                    "target_value": 0.0,
                    "target_yaw": stage_yaw,
                    "max_speed": self._cfg.max_angular_speed_rps,
                    "position_tolerance": self._cfg.rotate_done_rad,
                }
                print(f"[stage] sent mission={mission_id} seq={seq_pivot} kind=pivot yaw={math.degrees(stage_yaw):.1f}deg", flush=True)
                self._send_stage_command(pivot_payload)

                ok, reason = self._wait_for_stage_completion(mission_id, seq_pivot, 20.0, f"pivot_seg{index}")
                if not ok:
                    print(f"[{label}] pivot stage {seq_pivot} failed ({reason}); replanning...", flush=True)
                    failed = True
                    break

                # Step 2: Send Axis Drive Stage
                self._sequence_counter += 1
                seq_axis = self._sequence_counter
                axis_payload = {
                    "mission_id": mission_id,
                    "sequence": seq_axis,
                    "kind": kind,
                    "target_value": target_val,
                    "target_yaw": stage_yaw,
                    "max_speed": self._cfg.max_linear_speed_mps,
                    "position_tolerance": self._cfg.axis_tolerance_m,
                }
                print(
                    f"[stage] sent mission={mission_id} seq={seq_axis} kind={kind} "
                    f"target_val={target_val:.3f} length={segment_length:.3f}m",
                    flush=True,
                )
                self._send_stage_command(axis_payload)

                timeout_axis = 15.0 + 12.0 * segment_length
                ok, reason = self._wait_for_stage_completion(mission_id, seq_axis, timeout_axis, f"axis_seg{index}")
                if not ok:
                    print(f"[{label}] axis stage {seq_axis} failed ({reason}); replanning...", flush=True)
                    failed = True
                    break

            if failed:
                if attempt < self._cfg.replan_attempts:
                    self._spin_sleep(0.5)
                    continue
                return False

            break

        print(f"[{label}] approach point reached; starting Isaac physics docking stage", flush=True)

        # Final Docking Stage Orchestration
        raw_p = self._motion_pose()
        map_p = self._map_pose()
        if map_p is None or raw_p is None:
            print(f"[{label}] docking failed: missing pose", flush=True)
            return False

        yaw_offset = normalize_angle(raw_p[2] - map_p[2])
        ctrl_goal_yaw = normalize_angle(goal_yaw + yaw_offset)

        dx_map = gx - map_p[0]
        dy_map = gy - map_p[1]
        c_rot = math.cos(yaw_offset)
        s_rot = math.sin(yaw_offset)

        ctrl_dock_x = raw_p[0] + (c_rot * dx_map - s_rot * dy_map)
        ctrl_dock_y = raw_p[1] + (s_rot * dx_map + c_rot * dy_map)

        self._sequence_counter += 1
        seq_dock = self._sequence_counter
        dock_payload = {
            "mission_id": mission_id,
            "sequence": seq_dock,
            "kind": "dock",
            "target_x": ctrl_dock_x,
            "target_y": ctrl_dock_y,
            "target_yaw": ctrl_goal_yaw,
            "max_speed": self._cfg.dock_max_linear_speed_mps,
            "position_tolerance": self._cfg.dock_xy_tolerance_m,
        }
        print(
            f"[stage] sent mission={mission_id} seq={seq_dock} kind=dock "
            f"target_dock=({ctrl_dock_x:.3f},{ctrl_dock_y:.3f})",
            flush=True,
        )
        self._send_stage_command(dock_payload)

        ok, reason = self._wait_for_stage_completion(mission_id, seq_dock, 60.0, "micro_docking")
        if ok:
            print(f"[{label}] entire autonomous mission completed successfully!", flush=True)
            return True

        print(f"[{label}] docking stage failed or timed out ({reason})", flush=True)
        return False
