"""Nav2 free-path planning replaced with Orthogonal L-Path, Orthogonal A*, and Integrated Project Axis Controller & Forward/Lateral Docking Controller."""

from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
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


@dataclass
class AxisStage:
    kind: str           # "axis_x" or "axis_y"
    value: float        # Target raw-control axis coordinate
    yaw: float          # Heading for this stage (Control Frame)
    speed: float
    is_last: bool


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
    dock_xy_tolerance_m: float = 0.03
    dock_yaw_tolerance_rad: float = math.radians(1.0)
    dock_realign_threshold_rad: float = math.radians(6.0)
    dock_yaw_gain: float = 1.0
    dock_max_angular_speed_rps: float = 0.04
    dock_min_angular_speed_rps: float = 0.04
    dock_rotate_min_angular_speed_rps: float = 0.08
    dock_yaw_deadband_rad: float = math.radians(0.2)

    rotate_done_rad: float = math.radians(2.0)
    rotate_reenter_rad: float = math.radians(5.0)
    final_yaw_tolerance_rad: float = math.radians(2.0)
    axis_tolerance_m: float = 0.05
    final_approach_axis_tolerance_m: float = 0.05
    axis_speed_gain: float = 0.8
    axis_min_linear_speed_mps: float = 0.045
    axis_yaw_gain: float = 1.6
    axis_max_yaw_correction_rps: float = 0.28
    max_linear_speed_mps: float = 0.22
    min_linear_speed_mps: float = 0.07
    max_angular_speed_rps: float = 0.50
    rotate_min_angular_speed_rps: float = 0.18
    min_angular_speed_rps: float = 0.06
    rotate_gain: float = 1.25
    straight_yaw_gain: float = 1.2
    straight_max_angular_speed_rps: float = 0.08
    straight_yaw_deadband_rad: float = math.radians(0.3)

    drive_timeout_base_sec: float = 15.0
    drive_timeout_per_meter_sec: float = 12.0
    replan_cte_m: float = 0.15
    replan_attempts: int = 2
    log_period_sec: float = 1.0


def load_motion_config() -> MotionConfig:
    path = Path(get_package_share_directory("two_wheel_rails")) / "config" / "autonomous_nav.yaml"
    with path.open(encoding="utf-8") as stream:
        raw = yaml.safe_load(stream) or {}
    return MotionConfig(**{k: v for k, v in raw.items() if k in MotionConfig.__dataclass_fields__})


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _yaw_feedback(
    error: float,
    gain: float,
    max_angular_speed: float,
    deadband: float,
) -> float:
    if abs(error) <= deadband:
        return 0.0
    return _clamp(gain * error, -max_angular_speed, max_angular_speed)


def _rotation_command(
    error: float,
    gain: float,
    max_speed: float,
    min_speed: float,
) -> float:
    command = max(-max_speed, min(max_speed, gain * error))
    if abs(command) < min_speed:
        command = math.copysign(min_speed, error)
    return command


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
    """Orthogonal L-path & A* navigator with strict costmap, control-frame transformation, Integrated Project Axis Controller & Forward/Lateral Docking Controller."""

    def __init__(self, nav: BasicNavigator, tf_buffer, tracker: AmclPoseTracker) -> None:
        self._nav = nav
        self._tf = tf_buffer
        self._tracker = tracker
        self._cfg = load_motion_config()
        self._cmd_pub = nav.create_publisher(Twist, "/cmd_vel_nav", 10)
        self._control_pose: tuple[float, float, float] | None = None
        self._control_twist: tuple[float, float] | None = None
        self._last_pub_time: float = 0.0

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

    def _publish_cmd(self, cmd: Twist) -> None:
        now = time.monotonic()
        if self._last_pub_time > 0:
            dt = now - self._last_pub_time
            if dt > 0.10:
                print(f"[warning] cmd publish interval delayed: {dt:.3f}s > 0.10s", flush=True)
        self._last_pub_time = now
        self._cmd_pub.publish(cmd)

    def _stop(self) -> None:
        stop = Twist()
        for _ in range(8):
            self._publish_cmd(stop)
            rclpy.spin_once(self._nav, timeout_sec=0.03)

    def _wait_until_stopped(
        self,
        *,
        timeout_sec: float = 2.0,
        linear_threshold_mps: float = 0.01,
        angular_threshold_rps: float = 0.02,
        required_samples: int = 5,
        label: str = "stop",
    ) -> bool:
        started = time.monotonic()
        stable_count = 0
        last_log = started

        while time.monotonic() - started < timeout_sec:
            rclpy.spin_once(self._nav, timeout_sec=0.03)

            twist = self._control_twist
            if twist is None:
                stable_count = 0
                continue

            linear_speed = abs(twist[0])
            angular_speed = abs(twist[1])

            if linear_speed <= linear_threshold_mps and angular_speed <= angular_threshold_rps:
                stable_count += 1
                if stable_count >= required_samples:
                    print(
                        f"[{label}] chassis stopped: vx={linear_speed:.3f}m/s wz={angular_speed:.3f}rad/s",
                        flush=True,
                    )
                    return True
            else:
                stable_count = 0

            now = time.monotonic()
            if now - last_log >= 0.5:
                print(
                    f"[{label}] waiting physical stop: vx={linear_speed:.3f}m/s wz={angular_speed:.3f}rad/s "
                    f"stable={stable_count}/{required_samples}",
                    flush=True,
                )
                last_log = now

        print(f"[{label}] physical stop timeout", flush=True)
        return False

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

        # Step 1 & 2: Evaluate 2 L-shaped candidates
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

        # Step 3 & 4: Orthogonal A* with corner clearance validation
        forbidden_cells: set[GridCell] = set()
        info = self._costmap.info
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
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

    def _rotate_to(self, target_yaw: float, label: str, tolerance_rad: float | None = None) -> bool:
        tolerance = self._cfg.rotate_done_rad if tolerance_rad is None else tolerance_rad
        min_w = self._cfg.dock_rotate_min_angular_speed_rps if (tolerance_rad is not None) else self._cfg.rotate_min_angular_speed_rps
        max_w = self._cfg.max_angular_speed_rps

        started = time.monotonic()
        last_log = started
        stall_check_time = started
        stall_check_yaw = None

        settle_count = 0

        while time.monotonic() - started < 30.0:
            rclpy.spin_once(self._nav, timeout_sec=0.03)
            pose = self._motion_pose()
            if pose is None:
                continue

            if stall_check_yaw is None:
                stall_check_yaw = pose[2]

            error = normalize_angle(target_yaw - pose[2])

            now = time.monotonic()
            if now - stall_check_time >= 1.0:
                yaw_delta = abs(normalize_angle(pose[2] - stall_check_yaw))
                if yaw_delta < math.radians(0.2) and abs(error) > tolerance:
                    print(
                        f"[{label}] rotation stalled: error={math.degrees(error):.1f}deg "
                        f"yaw_delta={math.degrees(yaw_delta):.2f}deg; forcing minimum angular velocity",
                        flush=True,
                    )
                stall_check_yaw = pose[2]
                stall_check_time = now

            if abs(error) <= tolerance:
                stop_cmd = Twist()
                self._publish_cmd(stop_cmd)
                settle_count += 1
                if settle_count >= 5:
                    self._stop()
                    if not self._wait_until_stopped(
                        timeout_sec=2.0,
                        linear_threshold_mps=0.01,
                        angular_threshold_rps=0.02,
                        required_samples=5,
                        label=f"{label}:rotation_stop",
                    ):
                        return False
                    return True
            else:
                settle_count = 0
                cmd = Twist()
                cmd.angular.z = _rotation_command(
                    error=error,
                    gain=self._cfg.rotate_gain,
                    max_speed=max_w,
                    min_speed=min_w,
                )
                cmd.linear.x = 0.0
                self._publish_cmd(cmd)

            if now - last_log >= self._cfg.log_period_sec:
                print(
                    f"[{label}] rotate ctrl_err={math.degrees(error):.1f}deg (tol={math.degrees(tolerance):.1f}deg) "
                    f"(ctrl_yaw={math.degrees(pose[2]):.1f}deg, settle={settle_count})",
                    flush=True,
                )
                last_log = now

        self._stop()
        return False

    def _drive_axis_stage(self, stage: AxisStage, label: str) -> bool:
        if not self._rotate_to(stage.yaw, f"{label}:align"):
            return False

        if not self._wait_until_stopped(timeout_sec=2.0, label=f"{label}:align_stop"):
            return False

        started = time.monotonic()
        last_log = started
        previous_axis_error: float | None = None

        while time.monotonic() - started < 90.0:
            rclpy.spin_once(self._nav, timeout_sec=0.03)
            pose = self._motion_pose()
            if pose is None:
                continue

            x, y, yaw = pose

            axis = x if stage.kind == "axis_x" else y
            axis_error = stage.value - axis
            yaw_error = normalize_angle(stage.yaw - yaw)

            tolerance = (
                self._cfg.final_approach_axis_tolerance_m
                if stage.is_last
                else self._cfg.axis_tolerance_m
            )

            # 1) Reached axis tolerance (5cm)
            if abs(axis_error) <= tolerance:
                self._stop()
                self._spin_sleep(0.10)
                self._wait_until_stopped(timeout_sec=2.0, label=f"{label}:axis_reach_stop")
                print(
                    f"[{label}] axis reached: kind={stage.kind} error={axis_error:.3f}m tol={tolerance:.3f}m",
                    flush=True,
                )
                return True

            # 2) Axis target crossed (sign inversion check)
            crossed = (
                previous_axis_error is not None
                and (axis_error * previous_axis_error <= 0.0)
            )
            if crossed:
                self._stop()
                self._spin_sleep(0.10)
                self._wait_until_stopped(timeout_sec=2.0, label=f"{label}:axis_cross_stop")
                print(
                    f"[{label}] axis target crossed: prev={previous_axis_error:.3f}m current={axis_error:.3f}m",
                    flush=True,
                )
                return True

            previous_axis_error = axis_error

            requested = min(
                abs(stage.speed),
                max(
                    self._cfg.axis_min_linear_speed_mps,
                    abs(axis_error) * self._cfg.axis_speed_gain,
                ),
            )

            cmd = Twist()
            cmd.linear.x = math.copysign(requested, stage.speed)
            cmd.angular.z = max(
                -self._cfg.axis_max_yaw_correction_rps,
                min(
                    self._cfg.axis_max_yaw_correction_rps,
                    self._cfg.axis_yaw_gain * yaw_error,
                ),
            )

            self._publish_cmd(cmd)

            now = time.monotonic()
            if now - last_log >= self._cfg.log_period_sec:
                print(
                    f"[{label}] axis drive kind={stage.kind} axis_error={axis_error:.3f}m "
                    f"yaw_error={math.degrees(yaw_error):.1f}deg vx={cmd.linear.x:.3f} wz={cmd.angular.z:.3f}",
                    flush=True,
                )
                last_log = now

        self._stop()
        print(f"[{label}] axis stage timeout", flush=True)
        return False

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

                map_p_curr = self._map_pose()
                raw_p_curr = self._motion_pose()
                if map_p_curr is None or raw_p_curr is None:
                    failed = True
                    break

                yaw_offset = normalize_angle(raw_p_curr[2] - map_p_curr[2])
                dx_end = e[0] - map_p_curr[0]
                dy_end = e[1] - map_p_curr[0]

                c_rot = math.cos(yaw_offset)
                s_rot = math.sin(yaw_offset)

                control_start = (raw_p_curr[0], raw_p_curr[1])
                control_end = (
                    raw_p_curr[0] + c_rot * dx_end - s_rot * dy_end,
                    raw_p_curr[1] + s_rot * dx_end + c_rot * dy_end,
                )
                dx = control_end[0] - control_start[0]
                dy = control_end[1] - control_start[1]

                if abs(dx) >= abs(dy):
                    stage = AxisStage(
                        kind="axis_x",
                        value=control_end[0],
                        yaw=normalize_angle((0.0 if dx >= 0.0 else math.pi) + yaw_offset),
                        speed=self._cfg.max_linear_speed_mps if dx >= 0.0 else -self._cfg.max_linear_speed_mps,
                        is_last=is_last,
                    )
                else:
                    stage = AxisStage(
                        kind="axis_y",
                        value=control_end[1],
                        yaw=normalize_angle((math.pi / 2.0 if dy >= 0.0 else -math.pi / 2.0) + yaw_offset),
                        speed=self._cfg.max_linear_speed_mps if dy >= 0.0 else -self._cfg.max_linear_speed_mps,
                        is_last=is_last,
                    )

                if not self._drive_axis_stage(stage, f"{label}:seg{index}"):
                    print(f"[{label}] segment {index} failed; replanning (attempt {attempt + 1})...", flush=True)
                    failed = True
                    break

            if failed:
                if attempt < self._cfg.replan_attempts:
                    self._spin_sleep(0.5)
                    continue
                return False

            break

        print(f"[{label}] approach point reached; starting micro-docking stage", flush=True)

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

        if not self._rotate_to(ctrl_goal_yaw, f"{label}:dock_align", self._cfg.dock_yaw_tolerance_rad):
            return False

        if not self._wait_until_stopped(timeout_sec=2.0, label=f"{label}:dock_align_stop"):
            return False

        raw_p_after = self._motion_pose()
        map_p_after = self._map_pose()
        if raw_p_after is not None and map_p_after is not None:
            yaw_offset_after = normalize_angle(raw_p_after[2] - map_p_after[2])
            ctrl_goal_yaw = normalize_angle(goal_yaw + yaw_offset_after)
            dx_map = gx - map_p_after[0]
            dy_map = gy - map_p_after[1]
            c_rot = math.cos(yaw_offset_after)
            s_rot = math.sin(yaw_offset_after)
            ctrl_dock_x = raw_p_after[0] + (c_rot * dx_map - s_rot * dy_map)
            ctrl_dock_y = raw_p_after[1] + (s_rot * dx_map + c_rot * dy_map)

        # Micro-Docking Controller (Integrated Project Forward / Lateral / Yaw Feedback)
        dock_start_time = time.monotonic()
        dock_last_log = dock_start_time
        dock_settle_count = 0

        while time.monotonic() - dock_start_time < 45.0:
            rclpy.spin_once(self._nav, timeout_sec=0.03)
            pose = self._motion_pose()
            if pose is None:
                continue

            x, y, yaw = pose

            dx = ctrl_dock_x - x
            dy = ctrl_dock_y - y

            forward_error = math.cos(ctrl_goal_yaw) * dx + math.sin(ctrl_goal_yaw) * dy
            lateral_error = -math.sin(ctrl_goal_yaw) * dx + math.cos(ctrl_goal_yaw) * dy
            yaw_error = normalize_angle(ctrl_goal_yaw - yaw)
            dist_to_dock = math.hypot(dx, dy)

            position_ok = (dist_to_dock <= 0.04)
            yaw_ok = (abs(yaw_error) <= math.radians(2.0))

            if position_ok and yaw_ok:
                self._stop()
                dock_settle_count += 1
                if dock_settle_count >= 30:
                    print(
                        f"[{label}] docking complete: dist={dist_to_dock:.3f}m "
                        f"forward={forward_error:.3f}m lateral={lateral_error:.3f}m "
                        f"yaw_err={math.degrees(yaw_error):.2f}deg (settled 30 samples)",
                        flush=True,
                    )
                    return True
            else:
                dock_settle_count = 0

                cmd = Twist()
                linear_cmd = max(-0.04, min(0.08, 0.45 * forward_error))
                if abs(linear_cmd) < 0.015 and abs(forward_error) > 0.004:
                    linear_cmd = math.copysign(0.015, forward_error)
                cmd.linear.x = linear_cmd

                cmd.angular.z = max(
                    -0.20,
                    min(
                        0.20,
                        1.8 * yaw_error + 1.4 * lateral_error,
                    ),
                )
                self._publish_cmd(cmd)

            now = time.monotonic()
            if now - dock_last_log >= 0.5:
                print(
                    f"[{label}:dock] remaining={dist_to_dock:.3f}m fwd={forward_error:.3f}m "
                    f"lat={lateral_error:.3f}m yaw_err={math.degrees(yaw_error):.2f}deg "
                    f"vx={cmd.linear.x:.3f}m/s wz={cmd.angular.z:.3f}rad/s settle={dock_settle_count}/30",
                    flush=True,
                )
                dock_last_log = now

        self._stop()
        print(f"[{label}] docking stage timed out", flush=True)
        return False
