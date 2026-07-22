"""Isaac Sim 5.1 restaurant scene + Nova Carter (ROS USD) for nav_robot2 Nav2.

Run with Isaac's python:
  export ROS_DOMAIN_ID=103
  export NAV_ROBOT2_WS=$PWD   # nav_robot2 root
  .../python.sh isaacpjt/restaurant_nova_demo.py
"""

from __future__ import annotations

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
os.environ["ROS_DOMAIN_ID"] = os.environ.get(
    "NAV_ROBOT2_ROS_DOMAIN_ID", os.environ.get("ROS_DOMAIN_ID", "103")
)
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

HEADLESS = os.environ.get("NAV_ROBOT2_HEADLESS", "0") == "1"
simulation_app = SimulationApp({"headless": HEADLESS, "renderer": "RaytracedLighting"})

import carb
import numpy as np
import omni.graph.core as og
import omni.timeline
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import is_stage_loading
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.storage.native import get_assets_root_path
from pxr import PhysxSchema

enable_extension("isaacsim.ros2.bridge")
for _ in range(10):
    simulation_app.update()

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

carb.settings.get_settings().set_bool(
    "/exts/isaacsim.ros2.bridge/publish_without_verification", True
)

WORKSPACE = Path(
    os.environ.get("NAV_ROBOT2_WS", Path(__file__).resolve().parents[1])
).resolve()
RESTAURANT_USD = (
    WORKSPACE / "assets/lightweight_restaurant/lightweight_pizza_restaurant.usda"
)
NOVA_CARTER_REL = "/Isaac/Samples/ROS2/Robots/Nova_Carter_ROS.usd"
ROBOT_PRIM = "/World/NovaCarter"
WHEEL_JOINTS = ("joint_wheel_left", "joint_wheel_right")
WHEEL_RADIUS = 0.14
WHEEL_BASE = 0.413
MAX_WHEEL_SPEED = 25.0
CMD_TIMEOUT_S = 0.35

SPAWN_POSITION = np.array(
    [
        float(os.environ.get("NAV_SPAWN_X", "0.21")),
        float(os.environ.get("NAV_SPAWN_Y", "5.25")),
        float(os.environ.get("NAV_SPAWN_Z", "0.0")),
    ],
    dtype=float,
)
SPAWN_YAW = float(os.environ.get("NAV_SPAWN_YAW", str(-math.pi / 2.0)))


def _yaw_quat_wxyz(yaw: float) -> np.ndarray:
    return np.array(
        [math.cos(yaw * 0.5), 0.0, 0.0, math.sin(yaw * 0.5)], dtype=float
    )


def open_restaurant() -> None:
    if not RESTAURANT_USD.is_file():
        raise FileNotFoundError(
            f"{RESTAURANT_USD} missing — run ./tools/sync_restaurant_assets.sh"
        )
    kitchen = WORKSPACE / "assets/Lightwheel_Kitchen/Collected_KitchenRoom/KitchenRoom.usd"
    if not kitchen.is_file():
        print(
            "[warn] Lightwheel Kitchen USD missing — stage may fail to resolve refs",
            flush=True,
        )
    context = omni.usd.get_context()
    if not context.open_stage(str(RESTAURANT_USD)):
        raise RuntimeError(f"failed to open {RESTAURANT_USD}")
    for _ in range(30):
        simulation_app.update()
    while is_stage_loading():
        simulation_app.update()
    stage = context.get_stage()
    scene_prim = stage.GetPrimAtPath("/World/PhysicsScene")
    if scene_prim.IsValid():
        physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene_prim)
        physx_scene.CreateEnableStabilizationAttr(True)
        physx_scene.CreateEnableGPUDynamicsAttr(False)
        physx_scene.CreateTimeStepsPerSecondAttr(120)
    print(f"[nova] restaurant loaded: {RESTAURANT_USD}", flush=True)


def spawn_nova_carter() -> WheeledRobot:
    assets_root = get_assets_root_path()
    if not assets_root:
        raise RuntimeError(
            "Isaac assets root not found (Nova_Carter_ROS.usd via Nucleus)."
        )
    usd_path = assets_root + NOVA_CARTER_REL
    print(f"[nova] loading {usd_path}", flush=True)

    robot = WheeledRobot(
        prim_path=ROBOT_PRIM,
        name="nova_carter",
        wheel_dof_names=list(WHEEL_JOINTS),
        create_robot=True,
        usd_path=usd_path,
        position=SPAWN_POSITION,
        orientation=_yaw_quat_wxyz(SPAWN_YAW),
    )
    for _ in range(20):
        simulation_app.update()

    lidars_2d = [
        f"{ROBOT_PRIM}/ros_lidars/front_2d_lidar_render_product",
        f"{ROBOT_PRIM}/ros_lidars/back_2d_lidar_render_product",
    ]
    for i, path in enumerate(lidars_2d):
        try:
            og.Controller.attribute(f"{path}.inputs:enabled").set(i == 0)
        except Exception as exc:
            print(f"[warn] lidar enable {path}: {exc}", flush=True)
    try:
        og.Controller.attribute(
            f"{ROBOT_PRIM}/ros_lidars/front_3d_lidar_render_product.inputs:enabled"
        ).set(False)
    except Exception:
        pass

    hawk_graphs = ("/front_hawk", "/left_hawk", "/right_hawk", "/back_hawk")
    for hawk in hawk_graphs:
        for side in ("left", "right"):
            attr = f"{ROBOT_PRIM}{hawk}/{side}_camera_render_product.inputs:enabled"
            try:
                og.Controller.attribute(attr).set(False)
            except Exception:
                pass

    print(
        f"[nova] spawned at ({SPAWN_POSITION[0]:.2f},{SPAWN_POSITION[1]:.2f}) "
        f"yaw={SPAWN_YAW:.2f}",
        flush=True,
    )
    return robot


def configure_nova_ros_graphs() -> None:
    twist_topic_attrs = (
        f"{ROBOT_PRIM}/differential_drive/ros2_subscribe_twist.inputs:topicName",
        f"{ROBOT_PRIM}/differential_drive/ROS2SubscribeTwist.inputs:topicName",
    )
    for path in twist_topic_attrs:
        try:
            og.Controller.attribute(path).set("cmd_vel_og_disabled")
        except Exception:
            pass

    accel_attrs = (
        f"{ROBOT_PRIM}/differential_drive/differential_controller_01.inputs:maxAcceleration",
        f"{ROBOT_PRIM}/differential_drive/differential_controller_01.inputs:maxLinearAcceleration",
        f"{ROBOT_PRIM}/differential_drive/differential_controller_01.inputs:maxAngularAcceleration",
        f"{ROBOT_PRIM}/differential_drive/differential_controller_01.inputs:maxDeceleration",
    )
    for path in accel_attrs:
        try:
            og.Controller.attribute(path).set(0.0)
        except Exception:
            pass


def create_clock_ros_graph() -> None:
    keys = og.Controller.Keys
    domain = int(os.environ.get("ROS_DOMAIN_ID", "103"))
    og.Controller.edit(
        {"graph_path": "/World/NovaCarterClockROS2", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.SET_VALUES: [
                ("Context.inputs:domain_id", domain),
                ("Context.inputs:useDomainIDEnvVar", True),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ],
        },
    )
    print(f"[nova] /clock ready (domain={domain})", flush=True)


def _quat_yaw(w: float, x: float, y: float, z: float) -> float:
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


class CmdVelDrive(Node):
    def __init__(self, robot: WheeledRobot):
        super().__init__("nova_carter_cmd_vel_drive")
        self._robot = robot
        self._vx = 0.0
        self._wz = 0.0
        self._last_cmd = time.monotonic()
        self._lock = threading.Lock()
        self._pending_teleport: tuple[float, float, float, float] | None = None
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, qos)
        self.create_subscription(
            PoseStamped, "/nova_carter/teleport", self._on_teleport, qos
        )

    def _on_cmd(self, msg: Twist) -> None:
        with self._lock:
            self._vx = float(msg.linear.x)
            self._wz = float(msg.angular.z)
            self._last_cmd = time.monotonic()

    def _on_teleport(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        o = msg.pose.orientation
        yaw = _quat_yaw(float(o.w), float(o.x), float(o.y), float(o.z))
        z = float(p.z) if abs(float(p.z)) > 1e-6 else float(SPAWN_POSITION[2])
        with self._lock:
            self._pending_teleport = (float(p.x), float(p.y), z, yaw)
            self._vx = 0.0
            self._wz = 0.0

    def apply_teleport(self) -> None:
        with self._lock:
            pending = self._pending_teleport
            self._pending_teleport = None
        if pending is None:
            return
        x, y, z, yaw = pending
        try:
            self._robot.set_world_pose(
                position=np.asarray([x, y, z], dtype=float),
                orientation=_yaw_quat_wxyz(yaw),
            )
            self._robot.apply_wheel_actions(
                ArticulationAction(
                    joint_velocities=np.zeros(2, dtype=float),
                )
            )
        except Exception as exc:
            print(f"[nova] teleport failed: {exc}", flush=True)
            return
        print(
            f"[nova] teleported to ({x:.2f},{y:.2f}) yaw={yaw:.2f} "
            "(topic /nova_carter/teleport)",
            flush=True,
        )

    def apply(self) -> None:
        with self._lock:
            if time.monotonic() - self._last_cmd > CMD_TIMEOUT_S:
                vx, wz = 0.0, 0.0
            else:
                vx, wz = self._vx, self._wz
        half = 0.5 * WHEEL_BASE
        left = (vx - wz * half) / WHEEL_RADIUS
        right = (vx + wz * half) / WHEEL_RADIUS
        left = float(np.clip(left, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED))
        right = float(np.clip(right, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED))
        try:
            self._robot.apply_wheel_actions(
                ArticulationAction(joint_velocities=np.array([left, right], dtype=float))
            )
        except Exception:
            pass


def main() -> int:
    open_restaurant()
    create_clock_ros_graph()

    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0)
    robot = spawn_nova_carter()
    world.scene.add(robot)
    configure_nova_ros_graphs()

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    world.reset()
    for _ in range(30):
        world.step(render=True)
    robot.initialize()

    if not rclpy.ok():
        rclpy.init(args=[])
    drive = CmdVelDrive(robot)
    executor = SingleThreadedExecutor()
    executor.add_node(drive)

    print(
        f"[nova] domain={os.environ['ROS_DOMAIN_ID']} "
        "/clock /cmd_vel /chassis/odom /front_2d_lidar/scan "
        "/nova_carter/teleport",
        flush=True,
    )

    try:
        while simulation_app.is_running():
            executor.spin_once(timeout_sec=0.0)
            drive.apply_teleport()
            drive.apply()
            world.step(render=True)
    finally:
        timeline.stop()
        executor.remove_node(drive)
        drive.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
