#!/usr/bin/env python3
"""Shared odom-rail controller for kitchen ↔ table missions."""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path as FsPath

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

NORTH = math.pi / 2.0
SOUTH = -math.pi / 2.0


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def load_routes(path: FsPath) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not data:
        raise RuntimeError(f"empty yaml: {path}")
    return data


def resolve_config(name: str, explicit: str | None) -> FsPath:
    if explicit:
        path = FsPath(explicit)
        if path.is_file():
            return path
        raise FileNotFoundError(path)
    for path in (
        FsPath(get_package_share_directory("nav_robot_missions")) / "config" / name,
        FsPath(__file__).resolve().parents[3] / "config" / name,
        FsPath.cwd() / "config" / name,
    ):
        if path.is_file():
            return path
    raise FileNotFoundError(name)


class _StallWatch:
    """Fail fast when pose barely changes for stall_timeout seconds."""

    def __init__(self, timeout: float, move_tol: float = 0.04, yaw_tol: float = 0.10):
        self.timeout = timeout
        self.move_tol = move_tol
        self.yaw_tol = yaw_tol
        self._x = self._y = self._yaw = 0.0
        self._t = 0.0
        self._set = False

    def reset(self, p: tuple[float, float, float] | None) -> None:
        if p is None:
            return
        self._x, self._y, self._yaw = p
        self._t = time.time()
        self._set = True

    def tick(self, p: tuple[float, float, float] | None) -> float:
        if p is None or not self._set:
            return 0.0
        moved = math.hypot(p[0] - self._x, p[1] - self._y)
        turned = abs(_wrap(p[2] - self._yaw))
        if moved > self.move_tol or turned > self.yaw_tol:
            self.reset(p)
            return 0.0
        return time.time() - self._t


class RailMissionNode(Node):
    def __init__(
        self,
        node_name: str,
        table_id: int,
        routes_path: FsPath,
        linear_speed: float,
        angular_speed: float,
        dock_speed: float,
        xy_tolerance: float,
        yaw_tolerance: float,
        lat_tolerance: float,
        segment_timeout: float,
        stall_timeout: float,
    ):
        super().__init__(node_name)
        self.table_id = table_id
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.dock_speed = dock_speed
        self.xy_tolerance = xy_tolerance
        self.yaw_tolerance = yaw_tolerance
        self.lat_tolerance = lat_tolerance
        self.segment_timeout = segment_timeout
        self.stall_timeout = stall_timeout

        self.routes_data = load_routes(routes_path)
        self._lock = threading.Lock()
        self._odom = None

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Odometry, "/nav_robot/odom", self._on_odom, qos)
        self._cmd_pub = self.create_publisher(Twist, "/nav_robot/cmd_vel", qos)
        self._teleport_pub = self.create_publisher(
            PoseStamped, "/nav_robot/teleport", qos
        )
        self._path_pub = self.create_publisher(NavPath, "/nav_robot/rail_path", path_qos)

    def _on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))
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
            self._tick(0.02)

    def _tick(self, dt: float = 0.05):
        rclpy.spin_once(self, timeout_sec=0.0)
        time.sleep(dt)

    def _wait_odom(self, timeout: float = 15.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline and rclpy.ok():
            self._tick()
            if self.pose() is not None:
                return True
        return False

    def _kitchen(self):
        k = self.routes_data.get("kitchen") or {}
        return (
            float(k.get("x", 0.21)),
            float(k.get("y", 5.25)),
            float(k.get("yaw", SOUTH)),
        )

    def _route(self, outbound: bool) -> dict:
        key = (
            "to_4"
            if self.table_id == 4
            else (f"to_{self.table_id}" if outbound else f"from_{self.table_id}")
        )
        routes = self.routes_data.get("routes") or {}
        if key not in routes:
            raise KeyError(key)
        return routes[key]

    def _viz(self, route: dict, outbound: bool):
        pts = list(route.get("spine") or [])
        if outbound:
            pts += [route["pre_dock"], route["dock"]]
        else:
            pts = [route["pre_dock"]] + pts + [route["dock"]]
        path = NavPath()
        path.header.frame_id = "odom"
        path.header.stamp = self.get_clock().now().to_msg()
        for wp in pts:
            p = PoseStamped()
            p.header = path.header
            p.pose.position.x = float(wp["x"])
            p.pose.position.y = float(wp["y"])
            yaw = float(wp["yaw"])
            p.pose.orientation.z = math.sin(yaw * 0.5)
            p.pose.orientation.w = math.cos(yaw * 0.5)
            path.poses.append(p)
        self._path_pub.publish(path)

    def teleport_start(self, timeout: float = 10.0) -> bool:
        _kx, ky, kyaw = self._kitchen()
        sx, sy = 0.0, float(ky)
        tele = PoseStamped()
        tele.header.frame_id = "odom"
        tele.pose.position.x = sx
        tele.pose.position.y = sy
        tele.pose.position.z = 0.002
        tele.pose.orientation.z = math.sin(kyaw * 0.5)
        tele.pose.orientation.w = math.cos(kyaw * 0.5)
        self._stop()
        self.get_logger().info(f"teleport aisle-start ({sx:.2f},{sy:.2f})")
        for _ in range(25):
            tele.header.stamp = self.get_clock().now().to_msg()
            self._teleport_pub.publish(tele)
            self._tick()
        deadline = time.time() + timeout
        while time.time() < deadline and rclpy.ok():
            self._tick()
            p = self.pose()
            if p and math.hypot(p[0] - sx, p[1] - sy) < 0.25:
                self.get_logger().info(f"start odom=({p[0]:.2f},{p[1]:.2f})")
                return True
            tele.header.stamp = self.get_clock().now().to_msg()
            self._teleport_pub.publish(tele)
        self.get_logger().error("teleport timeout")
        return False

    def _stall_fail(self, label: str, p, stuck_s: float) -> bool:
        self._stop()
        pose = None if p is None else (round(p[0], 2), round(p[1], 2), round(p[2], 2))
        self.get_logger().error(
            f"STUCK ({label}) {stuck_s:.1f}s at pose={pose} — abort segment"
        )
        return False

    def rotate_to(self, target_yaw: float, label: str) -> bool:
        deadline = time.time() + 30.0
        # Four physical skid-steer wheels need lateral tire slip during an
        # in-place turn.  A lower cap prevents the chassis from being thrown
        # sideways into a chair before the dock segment begins.
        w_max = min(0.25, self.angular_speed)
        stall = _StallWatch(self.stall_timeout, move_tol=0.03, yaw_tol=0.06)
        start_pose = self.pose()
        stall.reset(start_pose)
        while time.time() < deadline and rclpy.ok():
            self._tick()
            p = self.pose()
            if p is None:
                continue
            stuck = stall.tick(p)
            if stuck >= self.stall_timeout:
                return self._stall_fail(label, p, stuck)
            err = _wrap(target_yaw - p[2])
            if abs(err) <= self.yaw_tolerance:
                self._stop()
                end_pose = self.pose()
                if start_pose is not None and end_pose is not None:
                    drift = math.hypot(
                        end_pose[0] - start_pose[0],
                        end_pose[1] - start_pose[1],
                    )
                    self.get_logger().info(
                        f"rotate done ({label}) drift={drift:.3f}m "
                        f"pose=({end_pose[0]:.2f},{end_pose[1]:.2f},"
                        f"{end_pose[2]:.2f})"
                    )
                return True
            self._cmd(0.0, max(-w_max, min(w_max, 1.5 * err)))
        self._stop()
        p = self.pose()
        self.get_logger().error(
            f"rotate timeout ({label}) yaw={None if p is None else round(p[2], 2)}"
        )
        return False

    def drive_rail(
        self,
        tx: float,
        ty: float,
        hold_yaw: float,
        label: str,
        *,
        reverse: bool = False,
        slow: bool = False,
        align_first: bool = True,
    ) -> bool:
        if align_first and not self.rotate_to(hold_yaw, f"{label}_rot"):
            return False

        deadline = time.time() + self.segment_timeout
        max_v = self.dock_speed if (slow or reverse) else self.linear_speed
        travel = _wrap(hold_yaw + math.pi) if reverse else hold_yaw
        ux, uy = math.cos(travel), math.sin(travel)
        k_yaw, k_cross = 1.2, 1.0
        stall = _StallWatch(self.stall_timeout)
        stall.reset(self.pose())

        while time.time() < deadline and rclpy.ok():
            self._tick()
            p = self.pose()
            if p is None:
                continue
            stuck = stall.tick(p)
            if stuck >= self.stall_timeout:
                return self._stall_fail(label, p, stuck)
            x, y, yaw = p
            dx, dy = tx - x, ty - y
            along = dx * ux + dy * uy
            cross = -dx * uy + dy * ux
            yaw_err = _wrap(hold_yaw - yaw)

            if abs(along) <= self.xy_tolerance and abs(cross) <= self.lat_tolerance:
                self._stop()
                return True
            if math.hypot(dx, dy) <= max(self.xy_tolerance, self.lat_tolerance):
                self._stop()
                return True

            wz = k_yaw * yaw_err + (k_cross * cross if not reverse else -k_cross * cross)
            if abs(yaw_err) > 0.20:
                wz = k_yaw * yaw_err
            wz = max(-0.45, min(0.45, wz))

            yaw_gate = 0.50 if reverse else self.yaw_tolerance
            if abs(yaw_err) > yaw_gate:
                self._cmd(0.0, wz)
                continue
            if abs(cross) > 0.20 and abs(yaw_err) > 0.08:
                self._cmd(0.0, wz)
                continue

            speed = min(max_v, max(0.06, 0.7 * max(along, 0.05)))
            if abs(cross) > 0.08:
                speed = min(speed, 0.12)
            if abs(yaw_err) > self.yaw_tolerance:
                speed = min(speed, 0.08)
            self._cmd((-speed if reverse else speed), wz)

        self._stop()
        p = self.pose()
        self.get_logger().error(
            f"timeout ({label}) tgt=({tx:.2f},{ty:.2f}) "
            f"pose={None if p is None else (round(p[0], 2), round(p[1], 2), round(p[2], 2))}"
        )
        return False

    def drive_aisle_north(self, ty: float, label: str, slow: bool = False) -> bool:
        """Creep north on the aisle while turning — avoids pure spin drift."""
        deadline = time.time() + self.segment_timeout
        max_v = self.dock_speed if slow else self.linear_speed
        stall = _StallWatch(self.stall_timeout, move_tol=0.025, yaw_tol=0.04)
        stall.reset(self.pose())

        while time.time() < deadline and rclpy.ok():
            self._tick()
            p = self.pose()
            if p is None:
                continue
            stuck = stall.tick(p)
            if stuck >= self.stall_timeout:
                return self._stall_fail(label, p, stuck)

            x, y, yaw = p
            dy = ty - y
            yaw_err = _wrap(NORTH - yaw)

            if dy <= self.xy_tolerance and abs(x) <= 0.18:
                self._stop()
                return True
            if dy <= self.xy_tolerance:
                self._stop()
                self.get_logger().info(f"{label} y-ok x={x:.2f} (hand off x correction)")
                return True

            wz = 1.2 * yaw_err + 0.35 * x
            wz = max(-0.35, min(0.35, wz))

            creep = min(0.08, max_v * 0.35) if dy > 0.10 else 0.0
            if abs(yaw_err) < 0.35:
                speed = min(max_v, max(creep, 0.55 * dy))
            else:
                speed = creep
            self._cmd(speed, wz)

        self._stop()
        p = self.pose()
        self.get_logger().error(
            f"timeout ({label}) ty={ty:.2f} "
            f"pose={None if p is None else (round(p[0], 2), round(p[1], 2), round(p[2], 2))}"
        )
        return False

    def nudge_x(self, tx: float, hold_yaw: float, label: str) -> bool:
        p = self.pose()
        if p is None:
            return False
        if abs(p[0] - tx) <= 0.06:
            return True

        err_x = p[0] - tx
        if abs(math.cos(hold_yaw)) >= 0.35:
            slide_yaw = hold_yaw
            reverse = (err_x * math.cos(hold_yaw)) > 0.0
        else:
            slide_yaw = 0.0 if err_x < 0.0 else math.pi
            reverse = False

        if abs(_wrap(slide_yaw - p[2])) > self.yaw_tolerance:
            if not self.rotate_to(slide_yaw, f"{label}_face"):
                return False

        deadline = time.time() + 20.0
        stall = _StallWatch(self.stall_timeout, move_tol=0.02, yaw_tol=0.05)
        stall.reset(self.pose())
        while time.time() < deadline and rclpy.ok():
            self._tick()
            p = self.pose()
            if p is None:
                continue
            stuck = stall.tick(p)
            if stuck >= self.stall_timeout:
                return self._stall_fail(label, p, stuck)
            x, _y, yaw = p
            if abs(x - tx) <= 0.06:
                self._stop()
                return True
            yaw_err = _wrap(slide_yaw - yaw)
            wz = max(-0.25, min(0.25, 1.2 * yaw_err))
            if abs(yaw_err) > self.yaw_tolerance:
                self._cmd(0.0, wz)
                continue
            speed = min(self.dock_speed, max(0.05, 0.4 * abs(x - tx)))
            self._cmd(-speed if reverse else speed, 0.4 * yaw_err)
        self._stop()
        p = self.pose()
        self.get_logger().error(
            f"nudge_x timeout ({label}) pose={None if p is None else (round(p[0], 2), round(p[1], 2))}"
        )
        return False

    def _near(self, tx: float, ty: float, tol: float = 0.20) -> bool:
        p = self.pose()
        return p is not None and math.hypot(p[0] - tx, p[1] - ty) < tol

    def follow_outbound(self, route: dict) -> bool:
        self._viz(route, True)
        dock = route["dock"]
        pre = route["pre_dock"]
        spine = list(route.get("spine") or [])
        px, py = float(pre["x"]), float(pre["y"])
        self.get_logger().info(
            f"OUT table={self.table_id} branch=({px:.2f},{py:.2f}) "
            f"dock=({dock['x']:.2f},{dock['y']:.2f})"
        )

        for i, wp in enumerate(spine):
            tx, ty, yaw = float(wp["x"]), float(wp["y"]), float(wp["yaw"])
            if self._near(tx, ty):
                continue
            if not self.drive_rail(tx, ty, yaw, f"spine[{i}]"):
                return False

        if self.table_id not in (0, 1, 2, 3):
            return self.drive_rail(
                float(dock["x"]), float(dock["y"]), float(dock["yaw"]), "kitchen", slow=True
            )

        if not self._near(px, py):
            aisle_yaw = SOUTH if py < (self.pose() or (0, 5, 0))[1] else NORTH
            if not self.drive_rail(px, py, aisle_yaw, "branch", slow=True):
                return False

        p = self.pose()
        if p is None:
            return False
        face = math.atan2(float(dock["y"]) - p[1], float(dock["x"]) - p[0])
        if not self.drive_rail(
            float(dock["x"]), float(dock["y"]), face, "dock", slow=True
        ):
            return False

        p = self.pose()
        if p is None:
            return False
        dx = abs(p[0] - float(dock["x"]))
        dy = abs(p[1] - float(dock["y"]))
        ok = dx < 0.15 and dy < 0.12
        self.get_logger().info(
            f"outbound done ({p[0]:.2f},{p[1]:.2f}) err=({dx:.2f},{dy:.2f}) {'OK' if ok else 'FAIL'}"
        )
        return ok

    def follow_return(self, route: dict) -> bool:
        """WIP: reverse -> branch -> north rail -> kitchen."""
        self._viz(route, False)
        aisle = route["pre_dock"]
        kitchen = route["dock"]
        spine = list(route.get("spine") or [])
        dock_yaw = float(self._route(True)["dock"]["yaw"])
        ax, ay = float(aisle["x"]), float(aisle["y"])
        kx, ky, kyaw = float(kitchen["x"]), float(kitchen["y"]), float(kitchen["yaw"])

        self.get_logger().info(f"RET reverse->branch ({ax:.2f},{ay:.2f})")
        if not self.drive_rail(ax, ay, dock_yaw, "reverse", reverse=True, slow=True):
            p = self.pose()
            if p is None or math.hypot(p[0] - ax, p[1] - ay) > 0.45:
                return False
            self.get_logger().warning(f"reverse soft OK ({p[0]:.2f},{p[1]:.2f})")

        self._stop()
        time.sleep(0.3)
        p = self.pose()
        if p is None:
            return False
        self.get_logger().info(f"at branch ({p[0]:.2f},{p[1]:.2f}) yaw={p[2]:.2f}")

        if abs(p[0] - ax) > 0.06:
            self.get_logger().info("center x on aisle (still facing table)")
            if not self.nudge_x(ax, dock_yaw, "center_x"):
                return False
            p = self.pose()
            if p is None:
                return False
            self.get_logger().info(f"centered x ({p[0]:.2f},{p[1]:.2f})")

        for i, wp in enumerate(spine):
            ty = float(wp["y"])
            if self._near(float(wp["x"]), ty, tol=0.15):
                continue
            self.get_logger().info(f"aisle north -> y={ty:.2f}")
            if not self.drive_aisle_north(ty, f"ret_spine[{i}]", slow=True):
                return False

        if not self._near(kx, ky, tol=0.20):
            self.get_logger().info(f"rail kitchen -> ({kx:.2f},{ky:.2f})")
            if not self.drive_aisle_north(ky, "kitchen_y", slow=True):
                return False
            if not self.drive_rail(kx, ky, kyaw, "kitchen_x", slow=True, align_first=False):
                return False

        p = self.pose()
        if p is None:
            return False
        dx = abs(p[0] - kx)
        dy = abs(p[1] - ky)
        ok = dx < 0.20 and dy < 0.20
        self.get_logger().info(
            f"returned to kitchen ({p[0]:.2f},{p[1]:.2f}) err=({dx:.2f},{dy:.2f}) {'OK' if ok else 'FAIL'}"
        )
        return ok
