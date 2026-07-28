"""Isaac Sim 5.1 restaurant + two-wheel Ridgeback bridge & Direct Route Physics State Machine for nav_robot5.

# Direct route, axis, pivot, docking, and recovery controller
# ported without behavioral simplification from:
# nav_robot/isaacpjt/nav_restaurant_demo.py
"""

from __future__ import annotations

import json
import math
import os
import queue
import sys
import threading
import time
from pathlib import Path

ISAAC_SIM_ROOT = os.environ.get("ISAAC_SIM_ROOT")
if not ISAAC_SIM_ROOT:
    raise RuntimeError(
        "ISAAC_SIM_ROOT가 설정되어 있지 않습니다. "
        "export ISAAC_SIM_ROOT=/path/to/isaac_sim/isaacsim/_build/linux-x86_64/release "
        "실행 후 다시 시도하세요."
    )
_ros_bridge_lib = Path(ISAAC_SIM_ROOT) / "exts/isaacsim.ros2.bridge/humble/lib"
os.environ.setdefault("ROS_DISTRO", "humble")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")

_ld_paths = [path for path in os.environ.get("LD_LIBRARY_PATH", "").split(":") if path]
_python_paths = [
    path
    for path in os.environ.get("PYTHONPATH", "").split(":")
    if path and "python3.10" not in path
]
_needs_ros_env = (
    str(_ros_bridge_lib) not in _ld_paths
    or ":".join(_python_paths) != os.environ.get("PYTHONPATH", "")
)
if _needs_ros_env and os.environ.get("NAV_ROS_REEXEC") != "1":
    _reexec_env = os.environ.copy()
    _reexec_env["LD_LIBRARY_PATH"] = ":".join([str(_ros_bridge_lib), *_ld_paths])
    _reexec_env["PYTHONPATH"] = ":".join(_python_paths)
    _reexec_env["NAV_ROS_REEXEC"] = "1"
    os.execve(sys.executable, [sys.executable, *sys.argv], _reexec_env)

from isaacsim import SimulationApp

HEADLESS = os.environ.get("NAV_ROBOT5_HEADLESS", "0") == "1"
simulation_app = SimulationApp({"headless": HEADLESS})

import numpy as np
import omni.graph.core as og
import omni.kit.app
import omni.kit.commands
import omni.physx
import omni.timeline
import omni.usd
import usdrt.Sdf
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade

_extension_manager = omni.kit.app.get_app().get_extension_manager()
_extension_manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
_extension_manager.set_extension_enabled_immediate("isaacsim.sensors.physx", True)
for _ in range(10):
    simulation_app.update()

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String as StringMsg
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time as RosTime

WORKSPACE = Path(
    os.environ.get("PROJECT_WS", Path(__file__).resolve().parents[1])
).resolve()

RESTAURANT_USD = (
    WORKSPACE / "assets/lightweight_restaurant/lightweight_pizza_restaurant.usda"
)
ROBOT_USD = WORKSPACE / "assets/diagnostics/two_wheel_serving_robot_v2.usd"
ROBOT_ASSET_ROOT = "/two_wheel_ridgeback_serving_robot"

SPAWN_POSITION = Gf.Vec3d(0.00, 5.25, 0.002)
SPAWN_YAW = -math.pi / 2.0

WHEEL_JOINTS = [
    "left_wheel_joint",
    "right_wheel_joint",
]
ARM_JOINTS = [f"joint_{index}" for index in range(1, 7)]
STOW_CONFIGURATION = [0.0, 0.0, 1.57, 0.0, 1.57, 0.0]

# Exact Original Control Constants from nav_restaurant_demo.py
WHEEL_RADIUS = 0.10
DIFFERENTIAL_HALF_TRACK = 0.315
MAX_WHEEL_SPEED = 8.0
LINEAR_ACCEL_LIMIT = 0.80
LINEAR_DECEL_LIMIT = 1.00
ANGULAR_ACCEL_LIMIT = 3.0
ANGULAR_DECEL_LIMIT = 3.5
WHEEL_DRIVE_DAMPING = 140.0
WHEEL_DRIVE_MAX_FORCE = 350.0
TIRE_STATIC_FRICTION = 0.50
TIRE_DYNAMIC_FRICTION = 0.50

# Hysteresis and Settle Tolerances for Docking
DOCK_POSITION_ENTER_TOL = 0.040
DOCK_POSITION_EXIT_TOL = 0.050
DOCK_LATERAL_ENTER_TOL = 0.015
DOCK_LATERAL_EXIT_TOL = 0.025
DOCK_YAW_ENTER_TOL = math.radians(2.0)
DOCK_YAW_EXIT_TOL = math.radians(3.0)
DOCK_SETTLE_TICKS = 20
DOCK_SETTLE_TIMEOUT = 5.0

# Near Goal Docking Tolerances, Hysteresis, and Timeout
DOCK_NEAR_DISTANCE = 0.060
DOCK_NEAR_FORWARD_TOL = 0.060
DOCK_NEAR_YAW_TOL = math.radians(2.0)
DOCK_NEAR_TIMEOUT = 8.0
DOCK_NEAR_ALIGN_ENTER_TOL = math.radians(2.0)
DOCK_NEAR_ALIGN_EXIT_TOL = math.radians(1.5)
DOCK_CREEP_GAIN = 0.80
DOCK_CREEP_MIN_SPEED = 0.040
DOCK_CREEP_MAX_SPEED = 0.060
DOCK_APPROACH_MIN_SPEED = 0.040
DOCK_RECOVERY_BACKOUT_DISTANCE = 0.25
DOCK_RECOVERY_BACKOUT_SPEED = 0.12
DOCK_RECOVERY_REAPPROACH_SPEED = 0.12
PRE_DOCK_POSITION_TOL = 0.030
AXIS_CROSS_TRACK_GAIN = 0.60
AXIS_CROSS_TRACK_MAX_CORRECTION = 0.12

ROBOT_ROOT = "/World/NavRobot"
ARTICULATION_CANDIDATES = [
    f"{ROBOT_ROOT}/Robot/ridgeback_base_link",
    f"{ROBOT_ROOT}/Robot",
]
BASE_LINK_NAME = "ridgeback_base_link"
RAW_ODOM_TOPIC = "/two_wheel/odom_raw"
RAW_SCAN_TOPIC = "/two_wheel/scan_raw"
TELEPORT_TOPIC = "/two_wheel/teleport"
MISSION_CMD_TOPIC = "/two_wheel/mission_command"
MISSION_STATUS_TOPIC = "/two_wheel/mission_status"
CMD_VEL_TOPIC = "/cmd_vel"
DIRECT_CMD_VEL_TOPIC = "/two_wheel/direct_cmd_vel"
EXTERNAL_CMD_TIMEOUT = 0.25

LIDAR_MIN_RANGE = 0.20
LIDAR_MAX_RANGE = 12.0
LIDAR_SAMPLES = 180
LIDAR_PERIOD_SEC = 0.10


def quaternion_to_yaw(orientation) -> float:
    if hasattr(orientation, "GetReal"):
        w = float(orientation.GetReal())
        x, y, z = [float(v) for v in orientation.GetImaginary()]
    else:
        w, x, y, z = [float(v) for v in orientation]
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_to_quat(yaw: float) -> Gf.Quatf:
    return Gf.Quatf(
        float(math.cos(yaw * 0.5)),
        0.0,
        0.0,
        float(math.sin(yaw * 0.5)),
    )


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def open_restaurant_and_robot():
    if not RESTAURANT_USD.is_file():
        raise FileNotFoundError(RESTAURANT_USD)
    if not ROBOT_USD.is_file():
        raise FileNotFoundError(ROBOT_USD)

    context = omni.usd.get_context()
    if not context.open_stage(str(RESTAURANT_USD)):
        raise RuntimeError(f"failed to open {RESTAURANT_USD}")
    for _ in range(30):
        simulation_app.update()

    stage = context.get_stage()
    spawn = UsdGeom.Xform.Define(stage, ROBOT_ROOT)
    spawn.AddTranslateOp().Set(SPAWN_POSITION)
    spawn.AddOrientOp().Set(yaw_to_quat(SPAWN_YAW))
    robot = UsdGeom.Xform.Define(stage, f"{ROBOT_ROOT}/Robot")
    robot.GetPrim().GetReferences().AddReference(
        str(ROBOT_USD), Sdf.Path(ROBOT_ASSET_ROOT)
    )
    return stage


def create_clock_ros_graph():
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": f"{ROBOT_ROOT}/NavSensorsROS2", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ],
        },
    )


def configure_joint_drives(stage):
    for prim in stage.Traverse():
        name = prim.GetName()
        if name in ARM_JOINTS:
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            drive.CreateStiffnessAttr(200000.0)
            drive.CreateDampingAttr(20000.0)
            drive.CreateMaxForceAttr(10000.0)
        elif name in WHEEL_JOINTS:
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            drive.CreateStiffnessAttr(0.0)
            drive.CreateDampingAttr(WHEEL_DRIVE_DAMPING)
            drive.CreateMaxForceAttr(WHEEL_DRIVE_MAX_FORCE)
            drive.CreateTargetVelocityAttr(0.0)


def configure_physics_stability(stage, articulation_path: str):
    scene_prim = stage.GetPrimAtPath("/World/PhysicsScene")
    if not scene_prim.IsValid():
        raise RuntimeError("restaurant PhysicsScene is missing")
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene_prim)
    physx_scene.CreateEnableStabilizationAttr(True)
    physx_scene.CreateEnableGPUDynamicsAttr(False)
    physx_scene.CreateBroadphaseTypeAttr("MBP")
    physx_scene.CreateTimeStepsPerSecondAttr(120)

    articulation_api = PhysxSchema.PhysxArticulationAPI.Apply(
        stage.GetPrimAtPath(articulation_path)
    )
    articulation_api.CreateSolverPositionIterationCountAttr(32)
    articulation_api.CreateSolverVelocityIterationCountAttr(4)
    articulation_api.CreateStabilizationThresholdAttr(0.01)
    articulation_api.CreateSleepThresholdAttr(0.05)


def configure_wheel_contact_material(stage):
    tire = UsdShade.Material.Define(stage, "/World/PhysicsMaterials/Nav5Tire")
    tire_api = UsdPhysics.MaterialAPI.Apply(tire.GetPrim())
    tire_api.CreateStaticFrictionAttr(TIRE_STATIC_FRICTION)
    tire_api.CreateDynamicFrictionAttr(TIRE_DYNAMIC_FRICTION)
    tire_api.CreateRestitutionAttr(0.0)
    tire_physx = PhysxSchema.PhysxMaterialAPI.Apply(tire.GetPrim())
    tire_physx.CreateFrictionCombineModeAttr("average")
    tire_physx.CreateRestitutionCombineModeAttr("average")

    caster = UsdShade.Material.Define(stage, "/World/PhysicsMaterials/Nav5Caster")
    caster_api = UsdPhysics.MaterialAPI.Apply(caster.GetPrim())
    caster_api.CreateStaticFrictionAttr(0.03)
    caster_api.CreateDynamicFrictionAttr(0.03)
    caster_api.CreateRestitutionAttr(0.0)
    PhysxSchema.PhysxMaterialAPI.Apply(
        caster.GetPrim()
    ).CreateFrictionCombineModeAttr("min")

    link_materials = {
        "left_wheel_link": tire,
        "right_wheel_link": tire,
        "front_caster_link": caster,
        "rear_caster_link": caster,
    }
    bound = {name: 0 for name in link_materials}
    for prim in stage.Traverse():
        parent_name = prim.GetParent().GetName()
        if (
            prim.GetName() == "collisions"
            and parent_name in link_materials
            and str(prim.GetPath()).startswith(ROBOT_ROOT)
        ):
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                link_materials[parent_name],
                UsdShade.Tokens.weakerThanDescendants,
                "physics",
            )
            bound[parent_name] += 1
    missing = [name for name, count in bound.items() if count != 1]
    if missing:
        raise RuntimeError(f"wheel/caster collision material binding failed: {bound}")
    print(f"[nav_robot5] contact materials ready: {bound}", flush=True)


def find_articulation_path(stage) -> str:
    for path in ARTICULATION_CANDIDATES:
        if stage.GetPrimAtPath(path).IsValid():
            return path
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if path.startswith(ROBOT_ROOT) and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return path
    raise RuntimeError("could not find robot articulation prim")


def initialize_robot(articulation_path: str):
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(5):
        simulation_app.update()

    articulation = SingleArticulation(
        prim_path=articulation_path, name="nav_ridgeback"
    )
    articulation.initialize()
    if not articulation.handles_initialized:
        for _ in range(20):
            simulation_app.update()
        articulation.initialize()
        if not articulation.handles_initialized:
            raise RuntimeError(f"invalid articulation handle: {articulation_path}")
    articulation.set_enabled_self_collisions(False)

    dof_names = list(articulation.dof_names)
    missing_wheels = set(WHEEL_JOINTS) - set(dof_names)
    if missing_wheels:
        raise RuntimeError(f"missing wheel DOFs: {sorted(missing_wheels)}")

    positions = articulation.get_joint_positions()
    for name in WHEEL_JOINTS:
        positions[dof_names.index(name)] = 0.0
    for name, value in zip(ARM_JOINTS, STOW_CONFIGURATION):
        if name in dof_names:
            positions[dof_names.index(name)] = value
    articulation.set_joint_positions(positions)
    articulation.set_joint_velocities(np.zeros(len(dof_names), dtype=float))

    return articulation, dof_names


# Direct route, axis, pivot, docking, and recovery controller
# ported without behavioral simplification from:
# nav_robot/isaacpjt/nav_restaurant_demo.py

def build_stages_from_points(points: list[dict], current_x: float, current_y: float) -> list[dict]:
    stages = []
    curr_x, curr_y = current_x, current_y

    for p in points:
        target_x = float(p["x"])
        target_y = float(p["y"])
        dx = target_x - curr_x
        dy = target_y - curr_y
        dist = math.hypot(dx, dy)

        if dist < 0.02:
            continue

        stage_yaw = math.atan2(dy, dx)
        stages.append({
            "kind": "pivot",
            "yaw": stage_yaw,
        })
        stages.append({
            "kind": "line",
            "start_x": curr_x,
            "start_y": curr_y,
            "target_x": target_x,
            "target_y": target_y,
            "length": dist,
            "yaw": stage_yaw,
            "speed": 0.22,
        })

        curr_x, curr_y = target_x, target_y

    return stages


class DiffNavBridge(Node):
    def __init__(self, articulation, dof_names):
        super().__init__("nav_robot_isaac_bridge")
        self.articulation = articulation
        self.dof_names = dof_names
        self.wheel_indices = np.asarray(
            [dof_names.index(name) for name in WHEEL_JOINTS], dtype=np.int32
        )

        self._target_vx = 0.0
        self._target_wz = 0.0
        self._cmd_vx = 0.0
        self._cmd_wz = 0.0
        self._last_cmd_time = time.monotonic()
        self._last_cmd_log_time = 0.0
        self._external_cmd_vx = 0.0
        self._external_cmd_wz = 0.0
        self._external_cmd_time = 0.0
        self._direct_cmd_vx = 0.0
        self._direct_cmd_wz = 0.0
        self._direct_cmd_time = 0.0
        self._last_scan_time = -LIDAR_PERIOD_SEC
        self._last_tick_time = time.monotonic()
        self._lock = threading.Lock()
        self._pending_teleport = None

        # Thread-safe Queue for ROS -> Physics Thread messages
        self._cmd_queue = queue.Queue()

        # Thread-safe storage for Status Snapshot (Physics -> ROS Thread)
        self._status_snapshot: dict | None = None

        # Navigation State Machine Variables (Ported from nav_restaurant_demo.py)
        self._direct_nav: dict | None = None
        self._navigation_paused = False
        self._obstacle_stop = False
        self._obstacle_scale = 1.0

        position, orientation = self.articulation.get_world_pose()
        self._odom_origin_x = float(position[0])
        self._odom_origin_y = float(position[1])
        self._odom_origin_yaw = quaternion_to_yaw(orientation)

        stage_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            StringMsg, MISSION_CMD_TOPIC, self._on_mission_command, stage_qos
        )
        self.create_subscription(
            Twist, CMD_VEL_TOPIC, self._on_cmd_vel, stage_qos
        )
        self.create_subscription(
            Twist, DIRECT_CMD_VEL_TOPIC, self._on_direct_cmd_vel, stage_qos
        )

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            PoseStamped, TELEPORT_TOPIC, self._on_teleport, qos
        )

        self.odom_pub = self.create_publisher(Odometry, RAW_ODOM_TOPIC, qos)
        self.scan_pub = self.create_publisher(LaserScan, RAW_SCAN_TOPIC, 10)
        self.clock_pub = self.create_publisher(Clock, "/clock", qos)
        self.mission_status_pub = self.create_publisher(StringMsg, MISSION_STATUS_TOPIC, stage_qos)

    # ROS Callback: Thread-safe Push ONLY (NO articulation calls!)
    def _on_mission_command(self, msg: StringMsg):
        try:
            payload = json.loads(msg.data)
            self._cmd_queue.put(payload)
        except Exception as exc:
            self.get_logger().error(f"failed to parse mission command JSON: {exc}")

    def _on_cmd_vel(self, msg: Twist):
        with self._lock:
            self._external_cmd_vx = float(msg.linear.x)
            self._external_cmd_wz = float(msg.angular.z)
            self._external_cmd_time = time.monotonic()

    def _on_direct_cmd_vel(self, msg: Twist):
        with self._lock:
            self._direct_cmd_vx = float(msg.linear.x)
            self._direct_cmd_wz = float(msg.angular.z)
            self._direct_cmd_time = time.monotonic()

    # Exact Line-by-Line Original Helper: Angle Error from nav_restaurant_demo.py
    @staticmethod
    def _angle_error(target: float, actual: float) -> float:
        return math.atan2(math.sin(target - actual), math.cos(target - actual))

    # Exact Line-by-Line Original Helper: Slew Limit from nav_restaurant_demo.py
    @staticmethod
    def _slew(current: float, target: float, acceleration: float, deceleration: float, dt: float) -> float:
        limit = acceleration if abs(target) > abs(current) else deceleration
        delta = target - current
        max_delta = limit * dt
        if abs(delta) <= max_delta:
            return target
        return current + math.copysign(max_delta, delta)

    # Exact Line-by-Line Original Helper: Differential IK from nav_restaurant_demo.py
    def _differential_ik(self, vx: float, wz: float) -> np.ndarray:
        turn = DIFFERENTIAL_HALF_TRACK * wz
        wheels = np.asarray(
            [
                (vx - turn) / WHEEL_RADIUS,
                (vx + turn) / WHEEL_RADIUS,
            ],
            dtype=float,
        )
        return np.clip(wheels, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)

    # Ported Method: Start Direct Navigation Mission
    def _start_direct_navigation(self, payload: dict, x: float, y: float, yaw: float):
        kind = payload.get("kind", "execute_route")
        mission_id = payload.get("mission_id", "")

        if kind == "cancel":
            self.get_logger().info(f"mission cancel received for mission={mission_id}")
            self._direct_nav = None
            self._cmd_vx = 0.0
            self._cmd_wz = 0.0
            self._update_status_snapshot(mission_id, "cancelled", "cancel", x, y, yaw)
            return

        if kind == "drive_distance":
            speed = float(payload.get("speed", -0.15))
            distance = max(0.05, abs(float(payload.get("distance", 0.0))))
            self._obstacle_stop = False
            self._direct_nav = {
                "mode": "drive_distance",
                "mission_id": mission_id,
                "start_x": x,
                "start_y": y,
                "start_yaw": yaw,
                "speed": speed,
                "distance": distance,
                "stage_start": time.monotonic(),
                "last_log": 0.0,
            }
            self._cmd_vx = 0.0
            self._cmd_wz = 0.0
            self._update_status_snapshot(
                mission_id, "accepted", "drive_distance", x, y, yaw
            )
            self.get_logger().info(
                f"direct distance started mission={mission_id} "
                f"distance={distance:.2f}m speed={speed:.2f}m/s"
            )
            return

        points = payload.get("points", [])
        dock_info = payload.get("dock", {})
        stages = build_stages_from_points(points, x, y)
        finish_after_route = bool(payload.get("finish_after_route", False))
        if finish_after_route:
            stages.append({
                "kind": "pivot",
                "yaw": float(payload.get("final_yaw", dock_info.get("yaw", yaw))),
            })

        self._direct_nav = {
            "mode": "route_and_dock",
            "mission_id": mission_id,
            "stages": stages,
            "index": 0,
            "stage_start": time.monotonic(),
            "last_log": 0.0,
            "finish_after_route": finish_after_route,
            "dock": (
                float(dock_info.get("x", x)),
                float(dock_info.get("y", y)),
                float(dock_info.get("yaw", yaw)),
            ),
        }
        self._obstacle_stop = False
        self._cmd_vx = 0.0
        self._cmd_wz = 0.0
        self._update_status_snapshot(mission_id, "accepted", "start", x, y, yaw)
        self.get_logger().info(
            f"direct route started mission={mission_id} stages={len(stages)} pose=({x:.2f},{y:.2f},{math.degrees(yaw):.1f}deg)"
        )

    # Ported Method: Finish Direct Navigation Mission
    def _finish_direct_navigation(self, success: bool, reason: str = "", x: float = 0.0, y: float = 0.0, yaw: float = 0.0):
        mission = self._direct_nav
        if mission is None:
            return
        mission_id = mission.get("mission_id", "")
        self._direct_nav = None
        self._obstacle_stop = False
        self._cmd_vx = 0.0
        self._cmd_wz = 0.0
        state = "completed" if success else "failed"
        self._update_status_snapshot(mission_id, state, reason, x, y, yaw)
        if success:
            self.get_logger().info(f"direct navigation complete mission={mission_id}")
        else:
            self.get_logger().error(f"direct navigation failed mission={mission_id}: {reason}")

    # Method: _update_direct_navigation with completion check evaluated BEFORE heading mismatch check
    def _update_direct_navigation(self, x: float, y: float, yaw: float) -> tuple[float, float] | None:
        mission = self._direct_nav
        if mission is None:
            return None
        if self._navigation_paused or self._obstacle_stop:
            return 0.0, 0.0

        if mission["mode"] == "drive_distance":
            dx = x - mission["start_x"]
            dy = y - mission["start_y"]
            signed_travel = (
                math.cos(mission["start_yaw"]) * dx
                + math.sin(mission["start_yaw"]) * dy
            )
            direction = 1.0 if mission["speed"] >= 0.0 else -1.0
            traveled = direction * signed_travel
            yaw_error = self._angle_error(mission["start_yaw"], yaw)
            elapsed = time.monotonic() - mission["stage_start"]

            if traveled >= mission["distance"] - 0.03:
                self._finish_direct_navigation(
                    True, "completed", x, y, yaw
                )
                return 0.0, 0.0

            timeout = mission["distance"] / max(abs(mission["speed"]), 0.05) + 5.0
            if elapsed > timeout:
                self._finish_direct_navigation(
                    False,
                    f"drive_distance timeout; traveled={traveled:.3f}m",
                    x,
                    y,
                    yaw,
                )
                return 0.0, 0.0

            if time.monotonic() - mission["last_log"] >= 1.0:
                mission["last_log"] = time.monotonic()
                self.get_logger().info(
                    f"direct distance pose=({x:.2f},{y:.2f},"
                    f"{math.degrees(yaw):.1f}deg) "
                    f"traveled={traveled:.2f}/{mission['distance']:.2f}m "
                    f"yaw_error={math.degrees(yaw_error):.1f}deg"
                )
                self._update_status_snapshot(
                    mission["mission_id"],
                    "running",
                    "drive_distance",
                    x,
                    y,
                    yaw,
                )

            vx = mission["speed"]
            wz = float(np.clip(1.6 * yaw_error, -0.20, 0.20))
            return vx, wz

        if mission["mode"] == "legacy_table":
            return self._update_legacy_table_navigation(mission, x, y, yaw)

        stages = mission["stages"]
        if mission["index"] >= len(stages):
            if mission.get("finish_after_route", False):
                self._finish_direct_navigation(
                    True, "position_reached_and_aligned", x, y, yaw
                )
                return 0.0, 0.0

            # Route stages complete -> Transition to legacy table docking!
            goal = mission["dock"]
            mission_id = mission["mission_id"]
            now = time.monotonic()
            mission.clear()
            mission.update(
                mode="legacy_table",
                mission_id=mission_id,
                goal=goal,
                stage="move_to_pre_dock",
                path_aligned=False,
                stage_start=now,
                last_log=0.0,
                settle_count=0,
                recovery_count=0,
                settling=False,
                settle_start=0.0,
                near_goal_active=False,
                near_goal_start=0.0,
                near_align_active=False,
                near_align_started=False,
                near_align_start_time=0.0,
                near_align_start_yaw=0.0,
            )
            self.get_logger().info(f"route stages complete; starting original pre-dock controller for goal=({goal[0]:.2f},{goal[1]:.2f})")
            return 0.0, 0.0

        stage = stages[mission["index"]]
        elapsed = time.monotonic() - mission["stage_start"]
        kind = stage["kind"]
        vx = wz = 0.0
        done = False

        if kind == "pivot":
            error = self._angle_error(stage["yaw"], yaw)
            done = abs(error) < math.radians(2.5)
            if not done:
                wz = float(np.clip(1.8 * error, -0.65, 0.65))
                if abs(wz) < 0.18:
                    wz = math.copysign(0.18, error)
            timeout = 25.0
            detail = f"yaw_error={math.degrees(error):.1f}deg"
        else:
            heading_x = math.cos(stage["yaw"])
            heading_y = math.sin(stage["yaw"])
            from_start_x = x - stage["start_x"]
            from_start_y = y - stage["start_y"]
            along = heading_x * from_start_x + heading_y * from_start_y
            remaining = stage["length"] - along
            cross_error = (
                -heading_y * from_start_x + heading_x * from_start_y
            )
            target_distance = math.hypot(
                stage["target_x"] - x,
                stage["target_y"] - y,
            )
            done = target_distance <= 0.05 or remaining <= 0.02

            if not done:
                # Steer back toward the actual transformed segment, rather
                # than forcing every segment onto the raw world's X/Y axes.
                heading_offset = float(
                    np.clip(
                        math.atan2(-AXIS_CROSS_TRACK_GAIN * cross_error, 0.8),
                        -AXIS_CROSS_TRACK_MAX_CORRECTION,
                        AXIS_CROSS_TRACK_MAX_CORRECTION,
                    )
                )
                desired_yaw = self._angle_error(
                    stage["yaw"] + heading_offset, 0.0
                )
                yaw_error = self._angle_error(desired_yaw, yaw)
                requested = min(
                    abs(stage["speed"]),
                    max(0.045, max(0.0, remaining) * 0.8),
                )
                vx = requested
                wz = float(np.clip(1.6 * yaw_error, -0.28, 0.28))
            timeout = 90.0
            detail = (
                f"remaining={remaining:.3f}m cross_error={cross_error:.3f}m "
                f"yaw_error={math.degrees(self._angle_error(stage['yaw'], yaw)):.1f}deg"
            )

        now = time.monotonic()
        if now - mission["last_log"] >= 1.0:
            mission["last_log"] = now
            self.get_logger().info(
                f"direct stage={mission['index']} {kind} pose=({x:.2f},{y:.2f},{math.degrees(yaw):.1f}deg) {detail}"
            )
            self._update_status_snapshot(mission["mission_id"], "running", kind, x, y, yaw)

        if elapsed > timeout:
            self._finish_direct_navigation(False, f"{kind} timeout; {detail}", x, y, yaw)
            return 0.0, 0.0

        if done:
            mission["index"] += 1
            mission["stage_start"] = now
            mission["last_log"] = 0.0
            self.get_logger().info(f"direct stage complete; index={mission['index']}")
            return 0.0, 0.0

        return (vx * self._obstacle_scale, wz)

    # Method: _update_legacy_table_navigation with Restored 0.18rad/s Minimum wz & Near Align Hysteresis
    def _update_legacy_table_navigation(self, mission: dict, x: float, y: float, yaw: float) -> tuple[float, float]:
        goal_x, goal_y, goal_yaw = mission["goal"]
        pre_x = goal_x - 0.65 * math.cos(goal_yaw)
        pre_y = goal_y - 0.65 * math.sin(goal_yaw)
        stage = mission["stage"]
        vx = wz = 0.0

        if stage == "move_to_pre_dock":
            dx, dy = pre_x - x, pre_y - y
            distance = math.hypot(dx, dy)
            heading_error = self._angle_error(math.atan2(dy, dx), yaw)
            if mission["path_aligned"]:
                if abs(heading_error) > math.radians(12.0):
                    mission["path_aligned"] = False
            elif abs(heading_error) < math.radians(3.0):
                mission["path_aligned"] = True

            if not mission["path_aligned"]:
                phase = "rotate_to_path"
                wz = float(np.clip(1.8 * heading_error, -0.65, 0.65))
            else:
                phase = "drive_to_pre_dock"
                vx = min(0.35, max(0.08, 0.8 * distance))
                wz = float(np.clip(1.2 * heading_error, -0.65, 0.65))

            if distance <= PRE_DOCK_POSITION_TOL:
                mission["stage"] = "align_at_pre_dock"
                mission["path_aligned"] = False
                mission["stage_start"] = time.monotonic()
                mission["settling"] = False
                mission["settle_count"] = 0
                mission["near_goal_active"] = False
                mission["near_align_active"] = False
                mission["near_align_started"] = False
                phase, vx, wz = "pre_dock_reached", 0.0, 0.0

            detail = f"pre=({pre_x:.2f},{pre_y:.2f}) distance={distance:.3f}m heading_error={math.degrees(heading_error):.1f}deg"
            timeout = 120.0

        elif stage == "align_at_pre_dock":
            distance = math.hypot(pre_x - x, pre_y - y)
            yaw_error = self._angle_error(goal_yaw, yaw)
            phase = "align_at_pre_dock"
            if abs(yaw_error) > math.radians(2.0):
                wz = float(np.clip(1.8 * yaw_error, -0.65, 0.65))
                if abs(wz) < 0.18:
                    wz = math.copysign(0.18, yaw_error)
            else:
                mission["stage"] = "final_approach"
                mission["stage_start"] = time.monotonic()
                mission["settling"] = False
                mission["settle_count"] = 0
                mission["near_goal_active"] = False
                mission["near_align_active"] = False
                mission["near_align_started"] = False
                phase, wz = "start_final_approach", 0.0
            detail = f"distance={distance:.3f}m yaw_error={math.degrees(yaw_error):.1f}deg"
            timeout = 30.0

        elif stage == "recovery_backout":
            dx, dy = goal_x - x, goal_y - y
            forward_error = math.cos(goal_yaw) * dx + math.sin(goal_yaw) * dy
            yaw_error = self._angle_error(goal_yaw, yaw)
            if abs(yaw_error) > math.radians(2.0):
                phase = "recovery_align_before_backout"
                wz = float(np.clip(1.8 * yaw_error, -0.45, 0.45))
                if abs(wz) < 0.18:
                    wz = math.copysign(0.18, yaw_error)
            elif forward_error < DOCK_RECOVERY_BACKOUT_DISTANCE:
                phase = "recovery_backout"
                vx = -DOCK_RECOVERY_BACKOUT_SPEED
                wz = float(np.clip(1.8 * yaw_error, -0.20, 0.20))
            else:
                mission["stage"] = "recovery_align"
                mission["stage_start"] = time.monotonic()
                mission["settling"] = False
                mission["settle_count"] = 0
                mission["near_goal_active"] = False
                mission["near_align_active"] = False
                mission["near_align_started"] = False
                phase, vx, wz = "recovery_backout_complete", 0.0, 0.0
            distance = math.hypot(dx, dy)
            detail = f"backout_forward={forward_error:.3f}m yaw_error={math.degrees(yaw_error):.1f}deg"
            timeout = 15.0

        elif stage == "recovery_align":
            dx, dy = goal_x - x, goal_y - y
            distance = math.hypot(dx, dy)
            approach_yaw = math.atan2(dy, dx)
            heading_error = self._angle_error(approach_yaw, yaw)
            phase = "recovery_align_to_goal"
            if abs(heading_error) > math.radians(2.0):
                wz = float(np.clip(1.8 * heading_error, -0.45, 0.45))
                if abs(wz) < 0.18:
                    wz = math.copysign(0.18, heading_error)
            else:
                mission["recovery_approach_yaw"] = approach_yaw
                mission["stage"] = "recovery_reapproach"
                mission["stage_start"] = time.monotonic()
                mission["settling"] = False
                mission["settle_count"] = 0
                mission["near_goal_active"] = False
                mission["near_align_active"] = False
                mission["near_align_started"] = False
                phase, wz = "recovery_reapproach_start", 0.0
            detail = f"distance={distance:.3f}m approach_yaw={math.degrees(approach_yaw):.1f}deg heading_error={math.degrees(heading_error):.1f}deg"
            timeout = 15.0

        elif stage == "recovery_reapproach":
            dx, dy = goal_x - x, goal_y - y
            distance = math.hypot(dx, dy)
            approach_yaw = mission["recovery_approach_yaw"]
            heading_error = self._angle_error(approach_yaw, yaw)
            phase = "recovery_reapproach"
            if distance > 0.08:
                if abs(heading_error) < math.radians(12.0):
                    vx = min(
                        DOCK_RECOVERY_REAPPROACH_SPEED,
                        max(DOCK_APPROACH_MIN_SPEED, 0.60 * distance),
                    )
                wz = float(np.clip(1.8 * heading_error, -0.25, 0.25))
            else:
                mission["stage"] = "recovery_final_align"
                mission["stage_start"] = time.monotonic()
                mission["settling"] = False
                mission["settle_count"] = 0
                mission["near_goal_active"] = False
                mission["near_align_active"] = False
                mission["near_align_started"] = False
                phase, vx, wz = "recovery_position_reached", 0.0, 0.0
            detail = f"distance={distance:.3f}m heading_error={math.degrees(heading_error):.1f}deg"
            timeout = 25.0

        elif stage == "recovery_final_align":
            dx, dy = goal_x - x, goal_y - y
            distance = math.hypot(dx, dy)
            yaw_error = self._angle_error(goal_yaw, yaw)
            phase = "recovery_final_align"
            if abs(yaw_error) > math.radians(2.0):
                wz = float(np.clip(1.8 * yaw_error, -0.35, 0.35))
                if abs(wz) < 0.18:
                    wz = math.copysign(0.18, yaw_error)
            else:
                mission["stage"] = "final_approach"
                mission["stage_start"] = time.monotonic()
                mission["settling"] = False
                mission["settle_count"] = 0
                mission["near_goal_active"] = False
                mission["near_align_active"] = False
                mission["near_align_started"] = False
                phase, wz = "recovery_final_align_complete", 0.0
            detail = f"distance={distance:.3f}m yaw_error={math.degrees(yaw_error):.1f}deg"
            timeout = 15.0

        else:
            # Final approach (docking) with Near Goal Align Hysteresis & Stalling Boost
            dx, dy = goal_x - x, goal_y - y
            distance = math.hypot(dx, dy)
            yaw_error = self._angle_error(goal_yaw, yaw)
            forward_error = math.cos(goal_yaw) * dx + math.sin(goal_yaw) * dy
            lateral_error = -math.sin(goal_yaw) * dx + math.cos(goal_yaw) * dy

            now = time.monotonic()

            # Priority 1: Lateral Recovery Trigger or Near Goal Timeout Recovery Trigger
            if (
                mission.get("near_goal_active", False)
                and now - mission.get("near_goal_start", now) > DOCK_NEAR_TIMEOUT
                and not (
                    distance <= DOCK_POSITION_ENTER_TOL
                    and abs(lateral_error) <= DOCK_LATERAL_ENTER_TOL
                    and abs(yaw_error) <= DOCK_YAW_ENTER_TOL
                )
            ):
                mission["recovery_count"] = mission.get("recovery_count", 0) + 1
                if mission["recovery_count"] > 3:
                    self._finish_direct_navigation(
                        False,
                        (
                            f"near goal docking exhausted; "
                            f"distance={distance:.3f} "
                            f"lateral={lateral_error:.3f} "
                            f"yaw={math.degrees(yaw_error):.1f}deg"
                        ),
                        x,
                        y,
                        yaw,
                    )
                    return 0.0, 0.0

                mission["stage"] = "recovery_backout"
                mission["stage_start"] = now
                mission["near_goal_active"] = False
                mission["near_align_active"] = False
                mission["near_align_started"] = False
                mission["settling"] = False
                mission["settle_count"] = 0
                phase, vx, wz = "start_near_goal_recovery", 0.0, 0.0
                self.get_logger().warning(
                    f"near goal docking timeout ({DOCK_NEAR_TIMEOUT}s); starting recovery {mission['recovery_count']}/3"
                )

            elif abs(lateral_error) > 0.05 and abs(forward_error) < 0.10:
                mission["recovery_count"] = mission.get("recovery_count", 0) + 1
                if mission["recovery_count"] > 3:
                    self._finish_direct_navigation(False, f"dock recovery exhausted; lateral={lateral_error:.3f}m", x, y, yaw)
                    return 0.0, 0.0
                mission["stage"] = "recovery_backout"
                mission["stage_start"] = now
                mission["near_goal_active"] = False
                mission["near_align_active"] = False
                mission["near_align_started"] = False
                mission["settling"] = False
                mission["settle_count"] = 0
                phase, vx, wz = "start_lateral_recovery", 0.0, 0.0
                self.get_logger().warning(f"dock lateral error={lateral_error:.3f}m; starting re-entry {mission['recovery_count']}/3")

            # Priority 2: Active Settle State Maintenance (Hysteresis Exit Tolerances)
            elif mission.get("settling", False):
                position_exit_ok = distance <= DOCK_POSITION_EXIT_TOL
                lateral_exit_ok = (
                    abs(lateral_error) <= DOCK_LATERAL_EXIT_TOL
                )
                yaw_exit_ok = abs(yaw_error) <= DOCK_YAW_EXIT_TOL
                motion_stopped = (abs(self._cmd_vx) <= 0.005 and abs(self._cmd_wz) <= 0.01)

                if position_exit_ok and lateral_exit_ok and yaw_exit_ok:
                    phase = "settle_at_table"
                    vx = 0.0
                    wz = 0.0
                    if motion_stopped:
                        mission["settle_count"] = mission.get("settle_count", 0) + 1
                    else:
                        mission["settle_count"] = 0

                    if now - mission.get("settle_start", now) > DOCK_SETTLE_TIMEOUT:
                        mission["settling"] = False
                        mission["settle_count"] = 0
                        self.get_logger().warning(f"dock settle timeout after {DOCK_SETTLE_TIMEOUT}s; resuming fine control")
                    elif mission["settle_count"] >= DOCK_SETTLE_TICKS:
                        self.get_logger().info(
                            f"[nav_robot5] docking completed: distance={distance:.3f}m "
                            f"lateral={lateral_error:.3f}m "
                            f"yaw_error={math.degrees(yaw_error):.2f}deg"
                        )
                        self._finish_direct_navigation(True, "completed", x, y, yaw)
                        return 0.0, 0.0
                else:
                    mission["settling"] = False
                    mission["settle_count"] = 0

            # Priority 3: New Settle State Entry (Strict Enter Tolerances)
            elif (
                distance <= DOCK_POSITION_ENTER_TOL
                and abs(lateral_error) <= DOCK_LATERAL_ENTER_TOL
                and abs(yaw_error) <= DOCK_YAW_ENTER_TOL
            ):
                mission["settling"] = True
                mission["settle_count"] = 1
                mission["settle_start"] = now
                mission["near_align_active"] = False
                mission["near_align_started"] = False
                phase = "settle_at_table"
                vx = 0.0
                wz = 0.0
                self.get_logger().info(
                    f"[nav_robot5] dock settle entered: distance={distance:.3f}m "
                    f"lateral={lateral_error:.3f}m "
                    f"yaw_error={math.degrees(yaw_error):.2f}deg"
                )

            # Priority 4: Near Goal Docking (distance <= 6cm)
            elif distance <= DOCK_NEAR_DISTANCE:
                if not mission.get("near_goal_active", False):
                    mission["near_goal_active"] = True
                    mission["near_goal_start"] = now
                    mission["stage_start"] = now

                # Hysteresis for Near Goal Align / Creep
                if mission.get("near_align_active", False):
                    if abs(yaw_error) <= DOCK_NEAR_ALIGN_EXIT_TOL:
                        mission["near_align_active"] = False
                        mission["near_align_started"] = False
                else:
                    if abs(yaw_error) >= DOCK_NEAR_ALIGN_ENTER_TOL:
                        mission["near_align_active"] = True

                if mission.get("near_align_active", False):
                    phase = "near_goal_align"
                    vx = 0.0  # Rotate in place

                    if not mission.get("near_align_started", False):
                        mission["near_align_started"] = True
                        mission["near_align_start_time"] = now
                        mission["near_align_start_yaw"] = yaw

                    wz = float(np.clip(1.8 * yaw_error, -0.35, 0.35))
                    if abs(wz) < 0.18:
                        wz = math.copysign(0.18, yaw_error)

                    # Rotation Stalling Boost
                    if (
                        now - mission.get("near_align_start_time", now) >= 1.5
                        and abs(math.degrees(self._angle_error(yaw, mission.get("near_align_start_yaw", yaw)))) < 0.3
                    ):
                        self.get_logger().warning("near goal rotation stalled; increasing angular command")
                        wz = math.copysign(0.25, yaw_error)
                else:
                    phase = "near_goal_creep"
                    mission["near_align_started"] = False
                    vx = float(
                        np.clip(
                            DOCK_CREEP_GAIN * forward_error,
                            -DOCK_CREEP_MAX_SPEED,
                            DOCK_CREEP_MAX_SPEED,
                        )
                    )
                    # Commands below 0.04 m/s do not reliably overcome the
                    # simulated wheel-drive damping and contact stiction.
                    if (
                        abs(vx) < DOCK_CREEP_MIN_SPEED
                        and abs(forward_error) > 0.004
                    ):
                        vx = math.copysign(DOCK_CREEP_MIN_SPEED, forward_error)
                    # A differential-drive base must briefly steer toward the
                    # goal line to remove lateral error before it can settle.
                    wz = float(
                        np.clip(
                            1.2 * yaw_error + 1.8 * lateral_error,
                            -0.12,
                            0.12,
                        )
                    )

                mission["settling"] = False
                mission["settle_count"] = 0

            # Priority 5: General Final Approach (distance > 6cm)
            else:
                phase = "final_forward_approach"
                if abs(yaw_error) <= math.radians(8.0):
                    vx = float(np.clip(0.45 * forward_error, -0.04, 0.08))
                    if (
                        abs(vx) < DOCK_APPROACH_MIN_SPEED
                        and abs(forward_error) > 0.004
                    ):
                        vx = math.copysign(
                            DOCK_APPROACH_MIN_SPEED, forward_error
                        )
                wz = float(np.clip(1.8 * yaw_error + 1.4 * lateral_error, -0.20, 0.20))
                mission["settling"] = False
                mission["settle_count"] = 0
                mission["near_align_active"] = False
                mission["near_align_started"] = False

            detail = f"goal=({goal_x:.2f},{goal_y:.2f}) distance={distance:.3f}m forward={forward_error:.3f}m lateral={lateral_error:.3f}m yaw_error={math.degrees(yaw_error):.1f}deg"
            timeout = 60.0

        now = time.monotonic()
        if now - mission["last_log"] >= 1.0:
            mission["last_log"] = now
            if mission.get("settling", False):
                self.get_logger().info(
                    f"[nav_robot5] dock settling: count={mission.get('settle_count', 0)}/{DOCK_SETTLE_TICKS} "
                    f"distance={distance:.3f}m yaw_error={math.degrees(yaw_error):.2f}deg cmd=({vx:.3f},{wz:.3f})"
                )
            else:
                self.get_logger().info(f"direct phase={phase} pose=({x:.2f},{y:.2f},{math.degrees(yaw):.1f}deg) {detail}")
            self._update_status_snapshot(mission["mission_id"], "running", phase, x, y, yaw)

        # Do NOT timeout overall mission if currently settling cleanly or active in near_goal_active phase
        if not mission.get("settling", False) and not mission.get("near_goal_active", False) and (now - mission["stage_start"] > timeout):
            self._finish_direct_navigation(False, f"{stage} timeout; {detail}", x, y, yaw)
            return 0.0, 0.0

        return (vx * self._obstacle_scale, wz)

    # Thread-safe Status Snapshot Update
    def _update_status_snapshot(self, mission_id: str, state: str, phase: str, x: float, y: float, yaw: float):
        snapshot = {
            "mission_id": mission_id,
            "state": state,
            "phase": phase,
            "x": round(x, 4),
            "y": round(y, 4),
            "yaw": round(yaw, 4),
        }
        with self._lock:
            self._status_snapshot = snapshot

    # Publish Status Snapshot (called from ROS spin loop safely)
    def publish_status_snapshot(self):
        with self._lock:
            snapshot = self._status_snapshot
        if snapshot is not None:
            msg = StringMsg()
            msg.data = json.dumps(snapshot)
            self.mission_status_pub.publish(msg)

    def _on_teleport(self, msg: PoseStamped):
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        z = float(msg.pose.position.z) if abs(msg.pose.position.z) > 1e-6 else 0.002
        yaw = quaternion_to_yaw(
            (
                float(msg.pose.orientation.w),
                float(msg.pose.orientation.x),
                float(msg.pose.orientation.y),
                float(msg.pose.orientation.z),
            )
        )
        with self._lock:
            self._pending_teleport = (x, y, z, yaw)
            self._cmd_vx = self._cmd_wz = 0.0
            self._target_vx = self._target_wz = 0.0

    def _apply_pending_teleport(self):
        with self._lock:
            pending = self._pending_teleport
            self._pending_teleport = None
        if pending is None:
            return
        x, y, z, yaw = pending
        quat = yaw_to_quat(yaw)
        self.articulation.set_world_pose(
            position=np.array([x, y, z]),
            orientation=np.array([quat.GetReal(), *quat.GetImaginary()]),
        )
        self.articulation.set_joint_velocities(
            np.zeros(len(self.dof_names), dtype=float)
        )
        with self._lock:
            self._odom_origin_x = x
            self._odom_origin_y = y
            self._odom_origin_yaw = yaw

    def _publish_scan(self, stamp: RosTime, sim_time_sec: float, x: float, y: float, yaw: float):
        if sim_time_sec - self._last_scan_time < LIDAR_PERIOD_SEC:
            return
        self._last_scan_time = sim_time_sec

        angle_min = -math.pi
        angle_increment = (2.0 * math.pi) / LIDAR_SAMPLES
        query = omni.physx.get_physx_scene_query_interface()
        sensor_forward = 0.48
        origin = (
            x + sensor_forward * math.cos(yaw),
            y + sensor_forward * math.sin(yaw),
            0.45,
        )
        ranges = []
        for index in range(LIDAR_SAMPLES):
            angle = angle_min + index * angle_increment
            world_angle = yaw + angle
            direction = (math.cos(world_angle), math.sin(world_angle), 0.0)
            hit = query.raycast_closest(origin, direction, LIDAR_MAX_RANGE)
            rigid_body = str(hit.get("rigidBody", "")) if hit["hit"] else ""
            if hit["hit"] and not rigid_body.startswith(ROBOT_ROOT):
                distance = float(hit["distance"])
            else:
                distance = math.inf
            if distance < LIDAR_MIN_RANGE:
                distance = math.inf
            ranges.append(distance)

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = BASE_LINK_NAME
        scan.angle_min = angle_min
        scan.angle_max = angle_min + (LIDAR_SAMPLES - 1) * angle_increment
        scan.angle_increment = angle_increment
        scan.time_increment = 0.0
        scan.scan_time = LIDAR_PERIOD_SEC
        scan.range_min = LIDAR_MIN_RANGE
        scan.range_max = LIDAR_MAX_RANGE
        scan.ranges = ranges
        self.scan_pub.publish(scan)

    # Main Physics Tick Method (Exact Order Matching nav_restaurant_demo.py)
    def tick(self, sim_time_sec: float):
        self._apply_pending_teleport()

        # Read articulation world pose SAFELY inside physics tick thread
        position, orientation = self.articulation.get_world_pose()
        x = float(position[0])
        y = float(position[1])
        yaw = quaternion_to_yaw(orientation)

        # Consume queued ROS command payloads
        try:
            while not self._cmd_queue.empty():
                cmd_payload = self._cmd_queue.get_nowait()
                self._start_direct_navigation(cmd_payload, x, y, yaw)
        except queue.Empty:
            pass

        now = time.monotonic()
        dt = max(1e-4, min(0.05, now - self._last_tick_time))
        self._last_tick_time = now

        # Execute Direct Navigation State Machine
        nav_res = self._update_direct_navigation(x, y, yaw)
        if nav_res is not None:
            target_vx, target_wz = nav_res
        else:
            with self._lock:
                direct_cmd_age = now - self._direct_cmd_time
                external_cmd_age = now - self._external_cmd_time
                if direct_cmd_age <= EXTERNAL_CMD_TIMEOUT:
                    target_vx = self._direct_cmd_vx
                    target_wz = self._direct_cmd_wz
                elif external_cmd_age <= EXTERNAL_CMD_TIMEOUT:
                    target_vx = self._external_cmd_vx
                    target_wz = self._external_cmd_wz
                else:
                    target_vx, target_wz = 0.0, 0.0

        # Exact Slew Sequence from nav_restaurant_demo.py
        self._cmd_vx = self._slew(
            self._cmd_vx, target_vx, LINEAR_ACCEL_LIMIT, LINEAR_DECEL_LIMIT, dt
        )
        self._cmd_wz = self._slew(
            self._cmd_wz, target_wz, ANGULAR_ACCEL_LIMIT, ANGULAR_DECEL_LIMIT, dt
        )
        vx, wz = self._cmd_vx, self._cmd_wz

        if (abs(vx) > 1e-3 or abs(wz) > 1e-3) and now - self._last_cmd_log_time > 1.0:
            self._last_cmd_log_time = now
            print(f"[nav_robot5] cmd vx={vx:.3f} wz={wz:.3f}", flush=True)

        wheel_velocities = self._differential_ik(vx, wz)
        self.articulation.apply_action(
            ArticulationAction(
                joint_velocities=wheel_velocities,
                joint_indices=self.wheel_indices,
            )
        )

        sec = int(sim_time_sec)
        nanosec = int((sim_time_sec - sec) * 1e9)
        stamp = RosTime(sec=sec, nanosec=nanosec)
        clock = Clock()
        clock.clock = stamp
        self.clock_pub.publish(clock)
        self._publish_scan(stamp, sim_time_sec, x, y, yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "world"
        odom.child_frame_id = BASE_LINK_NAME
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = float(position[2])
        odom.pose.pose.orientation.z = math.sin(yaw * 0.5)
        odom.pose.pose.orientation.w = math.cos(yaw * 0.5)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

        # Publish status snapshot safely
        self.publish_status_snapshot()


def main():
    stage = open_restaurant_and_robot()
    configure_joint_drives(stage)
    configure_wheel_contact_material(stage)
    articulation_path = find_articulation_path(stage)
    configure_physics_stability(stage, articulation_path)
    articulation, dof_names = initialize_robot(articulation_path)

    print(f"[nav_robot5] PhysX raycast {RAW_SCAN_TOPIC} ready", flush=True)

    if not rclpy.ok():
        rclpy.init(args=[])
    bridge = DiffNavBridge(articulation, dof_names)
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    spin_thread = threading.Thread(target=executor.spin, name="nav_ros_spin", daemon=True)
    spin_thread.start()

    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        timeline.play()

    print(
        f"[nav_robot5] 2-wheel bridge running; "
        f"Domain={os.environ.get('ROS_DOMAIN_ID', '0')} "
        f"spawn=({SPAWN_POSITION[0]:.2f},{SPAWN_POSITION[1]:.2f})",
        flush=True,
    )

    sim_time = 0.0
    sim_dt = 1.0 / 60.0
    try:
        while simulation_app.is_running():
            simulation_app.update()
            sim_time += sim_dt
            bridge.tick(sim_time)
    finally:
        executor.shutdown()
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()
