"""Isaac Sim 5.1 restaurant + two-wheel Ridgeback bridge & Physics Stage Executor for nav_robot5.

Publishes raw scan/odometry, monotonic /clock, receives /two_wheel/stage_command,
runs physics-tick level axis/pivot/docking/recovery stage execution, and applies wheel actions.
"""

from __future__ import annotations

import json
import math
import os
import sys
import threading
import time
from pathlib import Path

_ros_bridge_lib = Path(
    "/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/"
    "exts/isaacsim.ros2.bridge/humble/lib"
)
os.environ.setdefault("ROS_DISTRO", "humble")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
os.environ["ROS_DOMAIN_ID"] = os.environ.get("ROS_DOMAIN_ID", "102")

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
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time as RosTime

WORKSPACE = Path(
    os.environ.get("NAV_ROBOT5_WS", Path(__file__).resolve().parents[1])
).resolve()

RESTAURANT_USD = (
    WORKSPACE / "assets/lightweight_restaurant/lightweight_pizza_restaurant.usda"
)
if not RESTAURANT_USD.is_file():
    RESTAURANT_USD = WORKSPACE.parent / "nav_robot" / "assets/lightweight_restaurant/lightweight_pizza_restaurant.usda"

ROBOT_USD = WORKSPACE / "assets/diagnostics/two_wheel_serving_robot_v2.usd"
if not ROBOT_USD.is_file():
    ROBOT_USD = (
        WORKSPACE.parent
        / "nav_robot/assets/diagnostics/two_wheel_serving_robot_v2.usd"
    )
ROBOT_ASSET_ROOT = "/two_wheel_ridgeback_serving_robot"

SPAWN_POSITION = Gf.Vec3d(0.00, 5.25, 0.002)
SPAWN_YAW = -math.pi / 2.0

WHEEL_JOINTS = [
    "left_wheel_joint",
    "right_wheel_joint",
]
ARM_JOINTS = [f"joint_{index}" for index in range(1, 7)]
STOW_CONFIGURATION = [0.0, 0.0, 1.57, 0.0, 1.57, 0.0]

WHEEL_RADIUS = 0.10
DIFFERENTIAL_HALF_TRACK = 0.315
MAX_WHEEL_SPEED = 16.0
LINEAR_ACCEL_LIMIT = 0.80
LINEAR_DECEL_LIMIT = 1.00
ANGULAR_ACCEL_LIMIT = 2.0
ANGULAR_DECEL_LIMIT = 2.5
WHEEL_DRIVE_DAMPING = 140.0
WHEEL_DRIVE_MAX_FORCE = 350.0
TIRE_STATIC_FRICTION = 0.50
TIRE_DYNAMIC_FRICTION = 0.50

ROBOT_ROOT = "/World/NavRobot"
ARTICULATION_CANDIDATES = [
    f"{ROBOT_ROOT}/Robot/ridgeback_base_link",
    f"{ROBOT_ROOT}/Robot",
]
BASE_LINK_NAME = "ridgeback_base_link"
RAW_ODOM_TOPIC = "/two_wheel/odom_raw"
RAW_SCAN_TOPIC = "/two_wheel/scan_raw"
TELEPORT_TOPIC = "/two_wheel/teleport"
STAGE_CMD_TOPIC = "/two_wheel/stage_command"
STAGE_STATUS_TOPIC = "/two_wheel/stage_status"
CMD_TIMEOUT_SEC = 0.75

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


class PhysicsStageExecutor:
    """Integrated Isaac Physics Tick Stage Controller for Axis, Pivot, Micro-Docking & Recovery."""

    def __init__(self, node: DiffNavBridge):
        self.node = node
        self.active_stage = None
        self.stage_state = "idle"  # idle, running, completed, failed, cancelled
        self.start_time = 0.0
        self.dock_settle_count = 0
        self.recovery_count = 0
        self.recovery_sub_state = None
        self.recovery_start_pose = None
        self.recovery_start_time = 0.0
        self.zero_ticks_count = 0

    def handle_command(self, payload: dict):
        kind = payload.get("kind")
        mission_id = payload.get("mission_id", "")
        sequence = payload.get("sequence", 0)

        if kind == "cancel":
            print(f"[stage_executor] cancel command received for mission={mission_id}", flush=True)
            self.active_stage = None
            self.stage_state = "cancelled"
            self.node.publish_stage_status(mission_id, sequence, "cancelled", 0.0)
            return

        self.active_stage = {
            "mission_id": mission_id,
            "sequence": sequence,
            "kind": kind,
            "target_value": float(payload.get("target_value", 0.0)),
            "target_yaw": float(payload.get("target_yaw", 0.0)),
            "max_speed": float(payload.get("max_speed", 0.22)),
            "position_tolerance": float(payload.get("position_tolerance", 0.05)),
        }
        self.stage_state = "running"
        self.start_time = time.monotonic()
        self.dock_settle_count = 0
        self.recovery_count = 0
        self.recovery_sub_state = None
        self.zero_ticks_count = 0

        self.node.publish_stage_status(mission_id, sequence, "accepted", 0.0)
        print(
            f"[stage_executor] {kind} accepted: mission={mission_id} seq={sequence} "
            f"target_val={self.active_stage['target_value']:.3f} target_yaw={math.degrees(self.active_stage['target_yaw']):.1f}deg",
            flush=True,
        )

    def tick(self, x: float, y: float, yaw: float) -> tuple[float, float]:
        if self.active_stage is None or self.stage_state != "running":
            return 0.0, 0.0

        stage = self.active_stage
        kind = stage["kind"]
        mission_id = stage["mission_id"]
        sequence = stage["sequence"]
        target_val = stage["target_value"]
        target_yaw = stage["target_yaw"]
        max_speed = stage["max_speed"]
        pos_tol = stage["position_tolerance"]

        now = time.monotonic()
        elapsed = now - self.start_time

        # Timeout Checks
        timeout_limit = 60.0
        if kind in ("axis_x", "axis_y"):
            timeout_limit = 15.0 + 12.0 * abs(target_val - (x if kind == "axis_x" else y))
        elif kind == "pivot":
            timeout_limit = 20.0

        if elapsed > timeout_limit:
            print(f"[stage_executor] {kind} timeout after {elapsed:.1f}s", flush=True)
            self.stage_state = "failed"
            self.node.publish_stage_status(mission_id, sequence, "failed", 0.0, reason="timeout")
            self.active_stage = None
            return 0.0, 0.0

        vx, wz = 0.0, 0.0

        if kind in ("axis_x", "axis_y"):
            axis = x if kind == "axis_x" else y
            error = target_val - axis
            yaw_err = normalize_angle(target_yaw - yaw)

            if abs(error) <= pos_tol:
                if self.zero_ticks_count < 2:
                    self.zero_ticks_count += 1
                    return 0.0, 0.0
                self.stage_state = "completed"
                print(f"[stage_executor] {kind} completed: error={error:.3f}m (tol={pos_tol:.3f}m)", flush=True)
                self.node.publish_stage_status(mission_id, sequence, "completed", abs(error))
                self.active_stage = None
                return 0.0, 0.0

            direction = 1.0 if error >= 0.0 else -1.0
            requested = min(abs(max_speed), max(0.045, abs(error) * 0.8))
            vx = math.copysign(requested, direction)
            wz = max(-0.28, min(0.28, 1.6 * yaw_err))

        elif kind == "pivot":
            yaw_err = normalize_angle(target_yaw - yaw)

            if abs(yaw_err) <= math.radians(2.0):
                if self.zero_ticks_count < 2:
                    self.zero_ticks_count += 1
                    return 0.0, 0.0
                self.stage_state = "completed"
                print(f"[stage_executor] pivot completed: yaw_err={math.degrees(yaw_err):.2f}deg", flush=True)
                self.node.publish_stage_status(mission_id, sequence, "completed", abs(yaw_err))
                self.active_stage = None
                return 0.0, 0.0

            vx = 0.0
            raw_wz = max(-0.65, min(0.65, 1.8 * yaw_err))
            if abs(raw_wz) < 0.18:
                raw_wz = math.copysign(0.18, yaw_err)
            wz = raw_wz

        elif kind == "dock":
            # Docking & Recovery State Machine
            ctrl_dock_x = target_val
            ctrl_dock_y = target_val  # Target encoded
            ctrl_goal_yaw = target_yaw

            dx = ctrl_dock_x - x
            dy = ctrl_dock_y - y

            forward_error = math.cos(ctrl_goal_yaw) * dx + math.sin(ctrl_goal_yaw) * dy
            lateral_error = -math.sin(ctrl_goal_yaw) * dx + math.cos(ctrl_goal_yaw) * dy
            yaw_err = normalize_angle(ctrl_goal_yaw - yaw)
            dist_to_dock = math.hypot(dx, dy)

            # Lateral Recovery Trigger
            if self.recovery_sub_state is None and abs(lateral_error) > 0.05 and abs(forward_error) < 0.10:
                if self.recovery_count < 3:
                    self.recovery_count += 1
                    self.recovery_sub_state = "recovery_backout"
                    self.recovery_start_pose = (x, y, yaw)
                    self.recovery_start_time = now
                    print(f"[stage_executor] docking recovery_backout attempt={self.recovery_count}/3", flush=True)
                else:
                    print(f"[stage_executor] docking recovery limit exceeded", flush=True)

            if self.recovery_sub_state == "recovery_backout":
                vx = -0.06
                wz = 0.0
                if self.recovery_start_pose is not None:
                    moved = math.hypot(x - self.recovery_start_pose[0], y - self.recovery_start_pose[1])
                    if moved >= 0.50 or (now - self.recovery_start_time) > 8.0:
                        self.recovery_sub_state = "recovery_align"
                        self.recovery_start_time = now
                        print(f"[stage_executor] docking recovery_align", flush=True)
                return vx, wz

            elif self.recovery_sub_state == "recovery_align":
                target_entry_yaw = math.atan2(ctrl_dock_y - y, ctrl_dock_x - x)
                e_yaw = normalize_angle(target_entry_yaw - yaw)
                if abs(e_yaw) <= math.radians(2.0) or (now - self.recovery_start_time) > 6.0:
                    self.recovery_sub_state = "recovery_reapproach"
                    self.recovery_start_time = now
                    print(f"[stage_executor] docking recovery_reapproach", flush=True)
                    return 0.0, 0.0
                vx = 0.0
                raw_wz = max(-0.5, min(0.5, 1.8 * e_yaw))
                wz = math.copysign(0.18, e_yaw) if abs(raw_wz) < 0.18 else raw_wz
                return vx, wz

            elif self.recovery_sub_state == "recovery_reapproach":
                rem = math.hypot(ctrl_dock_x - x, ctrl_dock_y - y)
                if rem <= 0.15 or (now - self.recovery_start_time) > 8.0:
                    self.recovery_sub_state = "recovery_final_align"
                    self.recovery_start_time = now
                    print(f"[stage_executor] docking recovery_final_align", flush=True)
                    return 0.0, 0.0
                vx = min(0.08, 0.4 * rem)
                wz = 0.0
                return vx, wz

            elif self.recovery_sub_state == "recovery_final_align":
                e_yaw = normalize_angle(ctrl_goal_yaw - yaw)
                if abs(e_yaw) <= math.radians(2.0) or (now - self.recovery_start_time) > 6.0:
                    self.recovery_sub_state = None
                    print(f"[stage_executor] docking recovery complete; resuming docking loop", flush=True)
                    return 0.0, 0.0
                vx = 0.0
                raw_wz = max(-0.5, min(0.5, 1.8 * e_yaw))
                wz = math.copysign(0.18, e_yaw) if abs(raw_wz) < 0.18 else raw_wz
                return vx, wz

            # Normal Docking Forward/Lateral/Yaw Control
            position_ok = (dist_to_dock <= 0.04)
            yaw_ok = (abs(yaw_err) <= math.radians(2.0))

            if position_ok and yaw_ok:
                self.dock_settle_count += 1
                if self.dock_settle_count >= 30:
                    self.stage_state = "completed"
                    print(f"[stage_executor] docking completed: dist={dist_to_dock:.3f}m yaw_err={math.degrees(yaw_err):.2f}deg", flush=True)
                    self.node.publish_stage_status(mission_id, sequence, "completed", dist_to_dock)
                    self.active_stage = None
                    return 0.0, 0.0
                return 0.0, 0.0
            else:
                self.dock_settle_count = 0

                linear_cmd = max(-0.04, min(0.08, 0.45 * forward_error))
                if abs(linear_cmd) < 0.015 and abs(forward_error) > 0.004:
                    linear_cmd = math.copysign(0.015, forward_error)

                raw_wz = max(-0.20, min(0.20, 1.8 * yaw_err + 1.4 * lateral_error))
                needs_corr = (abs(yaw_err) > math.radians(0.8) or abs(lateral_error) > 0.02)
                if needs_corr and abs(raw_wz) < 0.04:
                    direction_source = raw_wz if abs(raw_wz) > 1e-6 else yaw_err
                    raw_wz = math.copysign(0.04, direction_source)

                vx = linear_cmd
                wz = raw_wz

        return vx, wz


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
        self._last_scan_time = -LIDAR_PERIOD_SEC
        self._lock = threading.Lock()
        self._pending_teleport = None

        position, orientation = self.articulation.get_world_pose()
        self._odom_origin_x = float(position[0])
        self._odom_origin_y = float(position[1])
        self._odom_origin_yaw = quaternion_to_yaw(orientation)

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            PoseStamped, TELEPORT_TOPIC, self._on_teleport, qos
        )
        self.create_subscription(
            StringMsg, STAGE_CMD_TOPIC, self._on_stage_command, qos
        )

        self.odom_pub = self.create_publisher(Odometry, RAW_ODOM_TOPIC, qos)
        self.scan_pub = self.create_publisher(LaserScan, RAW_SCAN_TOPIC, 10)
        self.clock_pub = self.create_publisher(Clock, "/clock", qos)
        self.stage_status_pub = self.create_publisher(StringMsg, STAGE_STATUS_TOPIC, qos)

        self.stage_executor = PhysicsStageExecutor(self)

    def _on_stage_command(self, msg: StringMsg):
        try:
            payload = json.loads(msg.data)
            with self._lock:
                self.stage_executor.handle_command(payload)
        except Exception as exc:
            print(f"[bridge] failed to parse stage command JSON: {exc}", flush=True)

    def publish_stage_status(self, mission_id: str, sequence: int, state: str, error: float, reason: str | None = None):
        position, orientation = self.articulation.get_world_pose()
        x = float(position[0])
        y = float(position[1])
        yaw = quaternion_to_yaw(orientation)

        data = {
            "mission_id": mission_id,
            "sequence": sequence,
            "state": state,
            "error": round(error, 4),
            "x": round(x, 4),
            "y": round(y, 4),
            "yaw": round(yaw, 4),
        }
        if reason:
            data["reason"] = reason

        msg = StringMsg()
        msg.data = json.dumps(data)
        self.stage_status_pub.publish(msg)

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

    def _slew(self, current, target, accel, decel, dt):
        if target >= current:
            limit = accel
        else:
            limit = decel
        step = limit * dt
        if abs(target - current) <= step:
            return target
        return current + math.copysign(step, target - current)

    def _differential_ik(self, vx: float, wz: float) -> np.ndarray:
        turn = DIFFERENTIAL_HALF_TRACK * wz
        left_vel = (vx - turn) / WHEEL_RADIUS
        right_vel = (vx + turn) / WHEEL_RADIUS
        wheels = np.asarray([left_vel, right_vel], dtype=float)
        return np.clip(wheels, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)

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

    def tick(self, sim_time_sec: float):
        self._apply_pending_teleport()
        position, orientation = self.articulation.get_world_pose()
        x = float(position[0])
        y = float(position[1])
        yaw = quaternion_to_yaw(orientation)

        now = time.monotonic()
        dt = 1.0 / 120.0

        # Run Physics Stage Executor directly in Isaac Physics Tick
        with self._lock:
            target_vx, target_wz = self.stage_executor.tick(x, y, yaw)

        self._cmd_vx = self._slew(
            self._cmd_vx, target_vx, LINEAR_ACCEL_LIMIT, LINEAR_DECEL_LIMIT, dt
        )
        self._cmd_wz = self._slew(
            self._cmd_wz, target_wz, ANGULAR_ACCEL_LIMIT, ANGULAR_DECEL_LIMIT, dt
        )
        vx, wz = self._cmd_vx, self._cmd_wz

        if (abs(vx) > 1e-3 or abs(wz) > 1e-3) and now - self._last_cmd_log_time > 1.0:
            self._last_cmd_log_time = now
            print(
                f"[nav_robot5] cmd vx={vx:.3f} wz={wz:.3f}",
                flush=True,
            )

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
        f"Domain={os.environ['ROS_DOMAIN_ID']} "
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
