"""Isaac Sim 5.1 restaurant navigation bridge for nav_robot.

Loads the lightweight pizza restaurant + Ridgeback USD, publishes
/nav_robot/{scan,odom,depth/points}, /clock, TF odom->ridgeback_base_link,
and applies /nav_robot/cmd_vel (differential vx+yaw) to wheels.

Run with Isaac's python, for example:
  /home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \\
    isaacpjt/nav_restaurant_demo.py
"""

from __future__ import annotations

import math
import os
import sys
import threading
import time
import gc
from pathlib import Path


# Arm serving targets are expressed in the docked robot frame.  The former
# 40 mm / 2 degree completion window was adequate for navigation, but can move
# the plate-rack handle by several centimetres relative to the proven
# standalone pose.  Keep the tighter defaults overridable for field tuning.
DOCK_XY_TOLERANCE_M = float(
    os.environ.get("NAV_DOCK_XY_TOLERANCE_M", "0.025")
)
DOCK_YAW_TOLERANCE_RAD = math.radians(
    float(os.environ.get("NAV_DOCK_YAW_TOLERANCE_DEG", "1.0"))
)

WORKSPACE = Path(
    os.environ.get("NAV_ROBOT_WS", Path(__file__).resolve().parents[1])
).resolve()
SERVING_WORKSPACE = WORKSPACE.parent / "serving_robot"

ISAAC_SIM_ROOT = os.environ.get(
    "ISAAC_SIM_ROOT",
    str(Path.home() / "dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release"),
)
_ros_bridge_lib = Path(ISAAC_SIM_ROOT) / "exts/isaacsim.ros2.bridge/humble/lib"
os.environ.setdefault("ROS_DISTRO", "humble")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
_nav_domain_id = os.environ.get("NAV_ROBOT_ROS_DOMAIN_ID")
if _nav_domain_id:
    os.environ["ROS_DOMAIN_ID"] = _nav_domain_id
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

HEADLESS = os.environ.get("NAV_ROBOT_HEADLESS", "0") == "1"
simulation_app = SimulationApp({"headless": HEADLESS})

import numpy as np
import omni.graph.core as og
import omni.kit.app
import omni.kit.commands
import omni.timeline
import omni.usd
import usdrt.Sdf
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

_extension_manager = omni.kit.app.get_app().get_extension_manager()
_extension_manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
_extension_manager.set_extension_enabled_immediate("isaacsim.sensors.physx", True)
for _ in range(10):
    simulation_app.update()

import json
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time as TimeMsg
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, String
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage

# The colcon workspace builds this custom interface for system Python 3.10,
# while Isaac Sim 5.1 embeds Python 3.11.  Do not abort the entire navigation
# simulator when the generated extension is unavailable to Isaac's rclpy.
try:
    from serving_robot_interfaces.srv import TaskCommand
except (ImportError, ModuleNotFoundError) as _task_command_import_error:
    TaskCommand = None
    print(
        "[warn] serving_robot_interfaces unavailable in Isaac Python; "
        "/food_spawn/command and /arm/command services disabled: "
        f"{_task_command_import_error}",
        flush=True,
    )

sys.path.insert(0, str(SERVING_WORKSPACE / "isaacpjt"))
sys.path.insert(0, str(SERVING_WORKSPACE / "isaacpjt/M0609/rmpflow"))
sys.path.insert(0, str(WORKSPACE / "isaacpjt/M0609/rmpflow"))
# Defaults when serving food modules cannot be imported (nav-only path).
GRIP_CONTACT_STATIC_FRICTION = float(os.environ.get("NAV_GRIP_STATIC_FRICTION", "6.0"))
GRIP_CONTACT_DYNAMIC_FRICTION = float(os.environ.get("NAV_GRIP_DYNAMIC_FRICTION", "5.0"))
GRIPPER_DRIVE_MAX_FORCE = float(os.environ.get("NAV_GRIPPER_DRIVE_MAX_FORCE", "40.0"))
try:
    from drink_serving import spawn_soda_cans
    from cutlery_serving import spawn_cutlery_box
    from pizza_serving import (
        GRIP_CONTACT_DYNAMIC_FRICTION,
        GRIP_CONTACT_STATIC_FRICTION,
        GRIPPER_DRIVE_MAX_FORCE,
        TrayPizzaPickPlace,
    )
    from soda1_delivery import Soda1PickPlace
    from soda2_delivery import Soda2PickPlace
    from cutlery_pick_place import CutleryBoxPickPlace
    try:
        from plate_rack_serving import (
            follow_plate_rack_transport,
            spawn_plate_rack,
        )
        from plate_rack_pick_place import PlateRackPickPlace
    except ImportError as _plate_exc:
        print(f"[warn] plate-rack module import: {_plate_exc}", flush=True)
    print(
        "[food_spawn] loaded pizza, soda1, soda2, cutlery modules",
        flush=True,
    )
except Exception as _food_import_exc:
    print(f"[warn] food spawn module import: {_food_import_exc}", flush=True)

from kitchen_return_module import build_kitchen_route
from table_route_module import build_table_route


_package_roots = [
    WORKSPACE / "install/m0609_isaac_description/share",
    WORKSPACE / "install/ridgeback_m0609_description/share",
    WORKSPACE.parent / "install/m0609_isaac_description/share",
    WORKSPACE.parent / "install/ridgeback_m0609_description/share",
    WORKSPACE.parent / "serving_robot/install/m0609_isaac_description/share",
    WORKSPACE.parent / "serving_robot/install/ridgeback_m0609_description/share",
]
os.environ["ROS_PACKAGE_PATH"] = ":".join(
    [str(path) for path in _package_roots if path.is_dir()]
    + ([os.environ["ROS_PACKAGE_PATH"]] if os.environ.get("ROS_PACKAGE_PATH") else [])
)

# The serving workspace owns the canonical robot model.  nav_robot contains an
# older description copy without the split sliding tray; never use that copy
# to regenerate or overwrite the shared v2 USD.
URDF_PATH = (
    SERVING_WORKSPACE
    / "src/ridgeback_m0609_description/urdf/ridgeback_m0609.urdf"
)
RESTAURANT_USD = (
    WORKSPACE / "assets/lightweight_restaurant/lightweight_pizza_restaurant.usda"
)
ROBOT_USD = (
    WORKSPACE / "assets/diagnostics/two_wheel_serving_robot_v2.usd"
)
ROBOT_ASSET_ROOT = "/two_wheel_ridgeback_serving_robot"
M0609_VISUAL_USD = (
    SERVING_WORKSPACE
    / "isaacpjt/M0609/Collected_m0609_camera2/m0609_gripper.usd"
)
M0609_DARK_SAFETY_MATERIAL_USD = (
    WORKSPACE / "assets/materials/m0609_dark_safety.usda"
)
D455_ASSET_USD = (
    SERVING_WORKSPACE
    / "isaacpjt/M0609/Collected_m0609_camera2/"
    "omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/"
    "Isaac/5.1/Isaac/Sensors/Intel/RealSense/rsd455.usd"
)


def enable_urdf_importer():
    try:
        manager = omni.kit.app.get_app().get_extension_manager()
        manager.set_extension_enabled_immediate("isaacsim.asset.importer.urdf", True)
    except Exception as exc:
        print(f"[warn] enable_urdf_importer: {exc}", flush=True)


def import_robot_usd():
    if not URDF_PATH.is_file():
        print(f"[warn] URDF missing: {URDF_PATH}", flush=True)
        return

    ROBOT_USD.parent.mkdir(parents=True, exist_ok=True)
    if (
        ROBOT_USD.is_file()
        and os.environ.get("NAV_ROBOT_REIMPORT", "0") != "1"
    ):
        print(f"[nav_robot] reuse USD={ROBOT_USD}", flush=True)
        return

    enable_urdf_importer()
    status, config = omni.kit.commands.execute("URDFCreateImportConfig")
    if not status:
        print("[warn] URDFCreateImportConfig failed", flush=True)
        return
    config.merge_fixed_joints = False
    config.convex_decomp = False
    config.import_inertia_tensor = True
    config.fix_base = False
    config.collision_from_visuals = False
    config.distance_scale = 1.0
    config.parse_mimic = True

    status, articulation_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=str(URDF_PATH),
        import_config=config,
        dest_path=str(ROBOT_USD),
        get_articulation_root=True,
    )
    if status and ROBOT_USD.is_file():
        print(
            f"[nav_robot] generated fresh USD={ROBOT_USD} articulation={articulation_path}",
            flush=True,
        )

# Spawn exactly on the restaurant centreline.  With yaw=-90 deg this makes the
# table-row approach an actual straight line to x=0 instead of a diagonal from
# the former x=0.21 offset.
SPAWN_POSITION = Gf.Vec3d(
    float(os.environ.get("NAV_SPAWN_X", "0.00")),
    float(os.environ.get("NAV_SPAWN_Y", "5.25")),
    float(os.environ.get("NAV_SPAWN_Z", "0.01")),
)
SPAWN_YAW = float(os.environ.get("NAV_SPAWN_YAW", str(-math.pi / 2.0)))

WHEEL_JOINTS = ["left_wheel_joint", "right_wheel_joint"]
ARM_JOINTS = [f"joint_{index}" for index in range(1, 7)]
SLIDING_TRAY_JOINTS = [
    "upper_tray_left_slide_joint",
    "upper_tray_right_slide_joint",
]
STOW_CONFIGURATION = list(np.deg2rad([90.0, 0.0, -90.0, 0.0, -60.0, 90.0]))

WHEEL_RADIUS = 0.10
# Geometry of the stable two-drive-wheel/caster base.
DIFFERENTIAL_HALF_TRACK = float(os.environ.get("NAV_HALF_TRACK", "0.315"))
MAX_WHEEL_SPEED = 16.0
DIRECT_CONTROL_HALF_TRACK = DIFFERENTIAL_HALF_TRACK
LINEAR_ACCEL_LIMIT = 0.80
LINEAR_DECEL_LIMIT = 1.00
ANGULAR_ACCEL_LIMIT = 3.0
ANGULAR_DECEL_LIMIT = 3.5
WHEEL_DRIVE_DAMPING = 140.0
WHEEL_DRIVE_MAX_FORCE = 350.0
TIRE_STATIC_FRICTION = float(os.environ.get("NAV_TIRE_STATIC_FRICTION", "0.50"))
TIRE_DYNAMIC_FRICTION = float(os.environ.get("NAV_TIRE_DYNAMIC_FRICTION", "0.50"))
ARM_DRIVE_STIFFNESS = 200000.0
ARM_DRIVE_DAMPING = 20000.0
ARM_DRIVE_MAX_FORCE = 10000.0
ARM_STOW_SPEED = float(os.environ.get("NAV_ARM_STOW_SPEED", "0.45"))
ARM_STOW_TOLERANCE = math.radians(5.0)
TRAY_DRIVE_STIFFNESS = 4000.0
TRAY_DRIVE_DAMPING = 500.0
TRAY_DRIVE_MAX_FORCE = 400.0
TRAY_RETRACT_STEPS = 360

ROBOT_ROOT = "/World/NavRobot"
ARTICULATION_CANDIDATES = [
    f"{ROBOT_ROOT}/Robot/ridgeback_base_link/ridgeback_base_link",
    f"{ROBOT_ROOT}/Robot/ridgeback_base_link",
    f"{ROBOT_ROOT}/Robot",
]
BASE_LINK_NAME = "ridgeback_base_link"
TABLE_CAMERA_PATH = f"{ROBOT_ROOT}/Robot/ridgeback_base_link/ridgeback_base_link/fixed_table_depth_camera/realsense_d455/RSD455/Camera_Pseudo_Depth"
FRONT_LIDAR_TRANSLATION = Gf.Vec3d(0.40, 0.0, 0.33)
FRONT_LIDAR_CONFIG = "RPLIDAR_S2E"
FRONT_LIDAR_FRAME = "base_scan"
FRONT_LIDAR_TOPIC = "/scan"

LIDAR_MIN_RANGE = 0.20
PEER_HARD_STOP_M = 1.10
# Corridor half-separation so simultaneous trips do not share the spine.
FLEET_AISLE_OFFSET_M = 0.55
FLEET_PASS_EXTRA_M = 0.30
LIDAR_MAX_RANGE = 12.0
LIDAR_SAMPLES = 180
LIDAR_PERIOD_SEC = 0.10
LIDAR_SENSOR_FORWARD = 0.48
LIDAR_SENSOR_HEIGHT = 0.45
PERSON_LIDAR_PROXY_PATH = "/World/CrossingPedestrian_person_lidar_collider"
PERSON_LIDAR_PROXY_RADIUS = float(
    os.environ.get("NAV_CROSSING_COLLIDER_RADIUS", "0.30")
)
PERSON_LIDAR_ENABLED_ATTR = "userProperties:lidarEnabled"

# Robot-local safety rectangles for _direct_nav's obstacle check (mission
# driving never touches ROS cmd_vel, so it cannot be protected by
# nav2_collision_monitor -- this mirrors that node's stop/slowdown polygons
# from nav2_params.yaml so the two stay in sync).  Unlike the old
# front_angle=15deg cone, these cover the robot's sides and rear too.
OBSTACLE_STOP_FRONT = 0.75
OBSTACLE_STOP_BACK = -0.55
OBSTACLE_STOP_HALF_WIDTH = 0.60
OBSTACLE_SLOWDOWN_FRONT = 1.35
OBSTACLE_SLOWDOWN_BACK = -0.75
OBSTACLE_SLOWDOWN_HALF_WIDTH = 0.85
OBSTACLE_SLOWDOWN_RATIO = 0.30

_front_lidar_render_product = None
_front_lidar_writer = None
_embedded_lidar_render_product = None
_embedded_lidar_writer = None


def set_robot_context(root: str, spawn_position: Gf.Vec3d) -> None:
    """Select the robot instance used by the legacy setup helpers."""
    global ROBOT_ROOT, SPAWN_POSITION, ARTICULATION_CANDIDATES, TABLE_CAMERA_PATH
    ROBOT_ROOT = root
    SPAWN_POSITION = spawn_position
    ARTICULATION_CANDIDATES = [
        f"{root}/Robot/ridgeback_base_link/ridgeback_base_link",
        f"{root}/Robot/ridgeback_base_link",
        f"{root}/Robot",
    ]
    TABLE_CAMERA_PATH = (
        f"{root}/Robot/ridgeback_base_link/ridgeback_base_link/"
        "fixed_table_depth_camera/realsense_d455/RSD455/Camera_Pseudo_Depth"
    )


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


def attach_m0609_visuals(stage):
    if not M0609_VISUAL_USD.is_file():
        print(f"[warn] M0609 visual USD missing at {M0609_VISUAL_USD}", flush=True)
        return

    wanted = ["base_link", *(f"link_{index}" for index in range(1, 7))]
    attached = []
    for link_name in wanted:
        matches = [
            prim
            for prim in stage.Traverse()
            if prim.GetName() == link_name
            and str(prim.GetPath()).startswith(ROBOT_ROOT)
        ]
        if len(matches) != 1:
            continue
        visual_path = matches[0].GetPath().AppendChild("visuals")
        visual_prim = stage.OverridePrim(visual_path)
        # Each robot needs an independent mutable visual subtree. Otherwise
        # two instances can share the source prototype and render arm meshes
        # at the source asset's local origin near the restaurant center.
        if visual_prim.IsInstanceable():
            visual_prim.SetInstanceable(False)
        visual_prim.GetReferences().ClearReferences()
        visual_prim.GetReferences().SetReferences(
            [
                Sdf.Reference(
                    str(M0609_VISUAL_USD),
                    Sdf.Path(f"/World/m0609/{link_name}/visuals"),
                )
            ]
        )
        attached.append(str(visual_path))
    print(
        f"[nav_robot] attached M0609 visual meshes from {M0609_VISUAL_USD} links={len(attached)}",
        flush=True,
    )

    if os.environ.get("M0609_DARK_SAFETY_VISUALS", "1") != "1":
        return
    if not M0609_DARK_SAFETY_MATERIAL_USD.is_file():
        print(
            f"[warn] dark M0609 safety material missing at "
            f"{M0609_DARK_SAFETY_MATERIAL_USD}",
            flush=True,
        )
        return

    material_path = Sdf.Path("/World/Looks/M0609DarkSafety")
    stage.DefinePrim(material_path.GetParentPath(), "Scope")
    material_prim = stage.OverridePrim(material_path)
    material_prim.GetReferences().SetReferences(
        [
            Sdf.Reference(
                str(M0609_DARK_SAFETY_MATERIAL_USD),
                Sdf.Path("/M0609DarkSafety"),
            )
        ]
    )
    dark_material = UsdShade.Material(material_prim)
    darkened = []
    # Override every M0609 visual link.  Binding at each ``visuals`` prim with
    # stronger-than-descendants also covers the meshes nested below it (for
    # example the wrist/end-effector geometry under the final arm link).
    for link_name in wanted:
        matches = [
            prim
            for prim in stage.Traverse()
            if prim.GetName() == link_name
            and str(prim.GetPath()).startswith(ROBOT_ROOT)
        ]
        if len(matches) != 1:
            continue
        visual_prim = stage.GetPrimAtPath(
            matches[0].GetPath().AppendChild("visuals")
        )
        if not visual_prim.IsValid():
            continue
        UsdShade.MaterialBindingAPI.Apply(visual_prim).Bind(
            dark_material,
            UsdShade.Tokens.strongerThanDescendants,
        )
        darkened.append(str(visual_prim.GetPath()))
    print(
        f"[vision-safety] dark material bound to all M0609 arm visuals: "
        f"{darkened}",
        flush=True,
    )


def sanitize_embedded_rsd455(stage, robot_root: str) -> None:
    """Remove nested rigid-body state from the embedded D455 sensor."""
    rsd_path = (
        f"{robot_root}/Robot/ridgeback_base_link/ridgeback_base_link/"
        "fixed_table_depth_camera/realsense_d455/RSD455"
    )
    rsd_prim = stage.GetPrimAtPath(rsd_path)
    if not rsd_prim.IsValid():
        return

    stripped = 0
    touched = []
    for prim in Usd.PrimRange(rsd_prim):
        changed = False
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
            changed = True
        if prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
            prim.RemoveAPI(PhysxSchema.PhysxRigidBodyAPI)
            changed = True
        if prim.HasAPI(UsdPhysics.MassAPI):
            prim.RemoveAPI(UsdPhysics.MassAPI)
            changed = True
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            prim.RemoveAPI(UsdPhysics.CollisionAPI)
            changed = True
        enabled = prim.GetAttribute("physics:rigidBodyEnabled")
        if enabled and enabled.IsValid():
            enabled.Set(False)
            changed = True
        if changed:
            stripped += 1
            touched.append(str(prim.GetPath()))

    if stripped:
        print(
            f"[nav_robot] stripped rigid-body/collision under {rsd_path} "
            f"n={stripped} roots_touched={touched[:8]}",
            flush=True,
        )


def attach_fixed_table_depth_camera(stage):
    base_path = Sdf.Path(f"{ROBOT_ROOT}/Robot/ridgeback_base_link/ridgeback_base_link")
    if not stage.GetPrimAtPath(base_path).IsValid():
        return
    assembly_path = base_path.AppendChild("fixed_table_depth_camera")
    UsdGeom.Xform.Define(stage, assembly_path)

    mast = UsdGeom.Cylinder.Define(stage, assembly_path.AppendChild("mast"))
    mast.CreateRadiusAttr(0.018)
    mast.CreateHeightAttr(0.935)
    mast.CreateAxisAttr(UsdGeom.Tokens.z)
    mast.AddTranslateOp().Set(Gf.Vec3f(-0.25, 0.285, 1.3225))
    mast.CreateDisplayColorAttr([Gf.Vec3f(0.12, 0.15, 0.18)])
    UsdPhysics.CollisionAPI.Apply(mast.GetPrim())

    boom = UsdGeom.Cylinder.Define(stage, assembly_path.AppendChild("boom"))
    boom.CreateRadiusAttr(0.018)
    boom.CreateHeightAttr(0.215)
    boom.CreateAxisAttr(UsdGeom.Tokens.z)
    boom.AddTranslateOp().Set(Gf.Vec3f(-0.25, 0.3925, 1.79))
    boom.AddRotateXOp().Set(90.0)
    boom.CreateDisplayColorAttr([Gf.Vec3f(0.12, 0.15, 0.18)])
    UsdPhysics.CollisionAPI.Apply(boom.GetPrim())

    if D455_ASSET_USD.is_file():
        camera_position = Gf.Vec3d(-0.25, 0.50, 1.85)
        table_target = Gf.Vec3d(1.00, 0.15, 0.74)
        desired_camera_to_base = Gf.Matrix4d().SetLookAt(
            camera_position, table_target, Gf.Vec3d(0.0, 0.0, 1.0)
        ).GetInverse()

        d455_stage = Usd.Stage.Open(str(D455_ASSET_USD))
        source_color_camera = d455_stage.GetPrimAtPath(
            "/Root/RSD455/Camera_OmniVision_OV9782_Color"
        )
        if source_color_camera.IsValid():
            camera_to_sensor = UsdGeom.Xformable(source_color_camera).GetLocalTransformation()
            source_rsd = d455_stage.GetPrimAtPath("/Root/RSD455")
            rsd_to_sensor = UsdGeom.Xformable(source_rsd).GetLocalTransformation()
            camera_to_outer_mount = camera_to_sensor * rsd_to_sensor
            sensor_to_base = camera_to_outer_mount.GetInverse() * desired_camera_to_base
            sensor_path = assembly_path.AppendChild("realsense_d455")
            sensor_mount = UsdGeom.Xform.Define(stage, sensor_path)
            sensor_mount.MakeMatrixXform().Set(sensor_to_base)

            rsd_path = sensor_path.AppendChild("RSD455")
            rsd_prim = stage.OverridePrim(rsd_path)
            rsd_prim.GetReferences().SetReferences(
                [Sdf.Reference(str(D455_ASSET_USD), Sdf.Path("/Root/RSD455"))]
            )
            print("[nav_robot] attached fixed table depth camera mast & D455 sensor", flush=True)


def attach_front_rplidar_ros2(stage):
    global _front_lidar_render_product, _front_lidar_writer
    try:
        import omni.replicator.core as rep
        base_path = Sdf.Path(f"{ROBOT_ROOT}/Robot/ridgeback_base_link/ridgeback_base_link")
        base_prim = stage.GetPrimAtPath(base_path)
        if not base_prim.IsValid():
            return
        mount_path = base_prim.GetPath().AppendChild(FRONT_LIDAR_FRAME)
        mount = UsdGeom.Xform.Define(stage, mount_path)
        mount.AddTranslateOp().Set(FRONT_LIDAR_TRANSLATION)

        status, lidar_prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path="/RPLIDAR_S2E",
            parent=str(mount_path),
            config=FRONT_LIDAR_CONFIG,
            translation=Gf.Vec3d(0.0, 0.0, 0.0),
            orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
            visibility=True,
        )
        if status and lidar_prim is not None:
            _front_lidar_render_product = rep.create.render_product(
                lidar_prim.GetPath(), [1, 1], name="FrontRPLidarNav"
            )
            _front_lidar_writer = rep.writers.get("RtxLidarROS2PublishLaserScan")
            _front_lidar_writer.initialize(
                topicName=FRONT_LIDAR_TOPIC,
                frameId=FRONT_LIDAR_FRAME,
            )
            _front_lidar_writer.attach([_front_lidar_render_product])
            print("[nav_robot] attached front RPLIDAR S2E on /scan", flush=True)
    except Exception as exc:
        print(f"[warn] RPLIDAR S2E setup: {exc}", flush=True)


def _parking_brake_path(articulation_path):
    articulation_path = str(articulation_path).rstrip("/")
    robot_root = (
        articulation_path.split("/Robot/", 1)[0]
        if "/Robot/" in articulation_path
        else articulation_path.rsplit("/", 1)[0]
    )
    return robot_root, f"{robot_root}/ParkingBrake"


def _require_articulation_rigid_body(stage, articulation_path):
    """Return the articulation base prim only when PhysX can constrain it."""
    articulation_path = str(articulation_path).rstrip("/")
    prim = stage.GetPrimAtPath(articulation_path)
    if not prim.IsValid():
        raise RuntimeError(f"articulation prim is missing: {articulation_path}")
    if not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        raise RuntimeError(
            "articulation prim lacks ArticulationRootAPI: "
            f"{articulation_path}"
        )
    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        raise RuntimeError(
            "parking brake target lacks RigidBodyAPI: "
            f"{articulation_path}"
        )
    return prim


def _set_parking_brake_anchor(joint, articulation_prim):
    """Make the world anchor coincide with the articulation base frame."""
    transform = UsdGeom.XformCache().GetLocalToWorldTransform(
        articulation_prim
    )
    position = transform.ExtractTranslation()
    rotation = transform.ExtractRotationQuat()
    imaginary = rotation.GetImaginary()
    joint.GetLocalPos0Attr().Set(Gf.Vec3f(*map(float, position)))
    joint.GetLocalRot0Attr().Set(
        Gf.Quatf(
            float(rotation.GetReal()),
            Gf.Vec3f(*map(float, imaginary)),
        )
    )
    joint.GetLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.GetLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    return position


def prepare_parking_brake(stage, articulation_path):
    """Pre-author a disabled world-to-base fixed joint before simulation.

    Creating or deleting a joint while PhysX is stepping previously corrupted
    the articulation and produced invalid-transform/broad-phase errors.  The
    joint topology is therefore authored once during scene composition; the
    serving path only updates its anchor and toggles ``jointEnabled``.
    """
    articulation_path = str(articulation_path).rstrip("/")
    articulation_prim = _require_articulation_rigid_body(
        stage, articulation_path
    )
    robot_root, brake_path = _parking_brake_path(articulation_path)
    joint = UsdPhysics.FixedJoint.Define(stage, brake_path)
    joint.CreateBody1Rel().SetTargets([Sdf.Path(articulation_path)])
    _set_parking_brake_anchor(joint, articulation_prim)
    joint.CreateJointEnabledAttr(False).Set(False)
    print(
        f"[parking-brake] prepared robot={robot_root} path={brake_path}",
        flush=True,
    )
    return brake_path


def add_parking_brake(stage, articulation_path):
    """Lock one robot at its current world pose with its pre-authored joint."""
    articulation_path = str(articulation_path).rstrip("/")
    articulation_prim = _require_articulation_rigid_body(
        stage, articulation_path
    )
    robot_root, brake_path = _parking_brake_path(articulation_path)
    joint = UsdPhysics.FixedJoint.Get(stage, brake_path)
    if not joint or not joint.GetPrim().IsValid():
        raise RuntimeError(
            f"parking brake was not prepared before simulation: {brake_path}"
        )

    # Never move the anchor of an active constraint. Re-locking after an
    # interrupted command must first release the old anchor.
    joint.GetJointEnabledAttr().Set(False)

    # Never repair relationship topology while PhysX is stepping.  It must
    # already target the rigid articulation body authored during scene setup.
    body1_targets = joint.GetBody1Rel().GetTargets()
    expected_body1 = [Sdf.Path(articulation_path)]
    if body1_targets != expected_body1:
        raise RuntimeError(
            f"parking brake body1 mismatch: expected={expected_body1} "
            f"actual={body1_targets}"
        )

    position = _set_parking_brake_anchor(joint, articulation_prim)
    joint.GetJointEnabledAttr().Set(True)

    # Keep wheel targets neutral as a second line of defense against stale
    # velocity commands when the fixed joint is released later.
    for prim in stage.Traverse():
        if (
            prim.GetName() in WHEEL_JOINTS
            and str(prim.GetPath()).startswith(f"{robot_root}/")
        ):
            UsdPhysics.DriveAPI.Apply(prim, "angular").CreateTargetVelocityAttr(
                0.0
            ).Set(0.0)
    print(
        f"[parking-brake] locked robot={robot_root} mode=fixed-joint "
        f"pose=({position[0]:.3f},{position[1]:.3f})",
        flush=True,
    )
    return brake_path


def remove_parking_brake(stage, articulation_path=None):
    """Disable pre-authored fixed joints and restore normal wheel drives."""
    robot_roots = None
    if articulation_path is not None:
        articulation_path = str(articulation_path).rstrip("/")
        robot_root, brake_path = _parking_brake_path(articulation_path)
        robot_roots = {robot_root}
        brake_paths = [brake_path]
    else:
        brake_paths = [
            str(prim.GetPath())
            for prim in stage.Traverse()
            if prim.GetName() == "ParkingBrake"
        ]
    for brake_path in brake_paths:
        joint = UsdPhysics.FixedJoint.Get(stage, brake_path)
        if joint and joint.GetPrim().IsValid():
            joint.GetJointEnabledAttr().Set(False)

    wheel_prims = [
        prim
        for prim in stage.Traverse()
        if prim.GetName() in WHEEL_JOINTS
        and (
            robot_roots is None
            or any(
                str(prim.GetPath()).startswith(f"{root}/")
                for root in robot_roots
            )
        )
    ]
    for prim in wheel_prims:
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.CreateStiffnessAttr(0.0).Set(0.0)
        drive.CreateDampingAttr(WHEEL_DRIVE_DAMPING).Set(WHEEL_DRIVE_DAMPING)
        drive.CreateMaxForceAttr(WHEEL_DRIVE_MAX_FORCE).Set(WHEEL_DRIVE_MAX_FORCE)
        drive.CreateTargetVelocityAttr(0.0).Set(0.0)
    if wheel_prims:
        print(
            f"[parking-brake] released robot="
            f"{next(iter(robot_roots)) if robot_roots else 'all'} "
            "mode=fixed-joint "
            f"damping={WHEEL_DRIVE_DAMPING:.0f} "
            f"max_force={WHEEL_DRIVE_MAX_FORCE:.0f}",
            flush=True,
        )


def wait_for_stage_loading(label: str) -> None:
    """Drain asynchronous USD payload/reference loading before PhysX setup."""
    context = omni.usd.get_context()
    timeout_sec = max(
        1.0, float(os.environ.get("NAV_STAGE_LOAD_TIMEOUT_SEC", "120.0"))
    )
    deadline = time.monotonic() + timeout_sec
    while True:
        loading_status = context.get_stage_loading_status()
        pending = int(loading_status[2])
        if pending <= 0:
            break
        if time.monotonic() >= deadline:
            raise TimeoutError(
                f"USD stage loading timed out after {timeout_sec:.1f}s "
                f"during {label}: status={loading_status}"
            )
        simulation_app.update()
    # Process the final ObjectsChanged notices emitted when pending reaches 0.
    for _ in range(2):
        simulation_app.update()
    print(f"[nav_robot] USD loading complete: {label}", flush=True)


def open_restaurant_and_robot(open_stage: bool = True):
    if not RESTAURANT_USD.is_file():
        raise FileNotFoundError(RESTAURANT_USD)

    if not ROBOT_USD.is_file():
        raise FileNotFoundError(ROBOT_USD)

    context = omni.usd.get_context()
    if open_stage:
        if not context.open_stage(str(RESTAURANT_USD)):
            raise RuntimeError(f"failed to open {RESTAURANT_USD}")
        wait_for_stage_loading("restaurant")

    stage = context.get_stage()
    spawn = UsdGeom.Xform.Define(stage, ROBOT_ROOT)
    spawn.AddTranslateOp().Set(SPAWN_POSITION)
    spawn.AddOrientOp().Set(yaw_to_quat(SPAWN_YAW))
    robot = UsdGeom.Xform.Define(stage, f"{ROBOT_ROOT}/Robot")
    robot_prim = robot.GetPrim()
    # Keep each robot's physics and visual overrides out of a shared mutable
    # USD prototype when the same ROBOT_USD is composed more than once.
    robot_prim.SetInstanceable(False)
    robot_prim.GetReferences().AddReference(
        str(ROBOT_USD), Sdf.Path(ROBOT_ASSET_ROOT)
    )
    wait_for_stage_loading(f"{ROBOT_ROOT} robot reference")
    robot_scope = f"{ROBOT_ROOT}/Robot/"
    composed_names = {
        prim.GetName()
        for prim in stage.Traverse()
        if str(prim.GetPath()).startswith(robot_scope)
    }
    missing_tray_prims = set(SLIDING_TRAY_JOINTS) - composed_names
    if missing_tray_prims:
        raise RuntimeError(
            "loaded robot USD is the obsolete fixed/notched tray model; "
            f"missing sliding tray joints={sorted(missing_tray_prims)} "
            f"usd={ROBOT_USD}"
        )
    print(
        f"[nav_robot] verified v2 sliding-tray USD={ROBOT_USD}",
        flush=True,
    )
    # Keep the two-wheel robot's physics/articulation layers, but replace the
    # imported M0609 visual references with the canonical collected visual
    # asset.  Without this composition fix link_2's visual pieces can resolve
    # at the layer origin even though the physical link/joint poses are valid.
    attach_m0609_visuals(stage)
    sanitize_embedded_rsd455(stage, ROBOT_ROOT)
    print("[nav_robot] using embedded D455 and RPLIDAR sensor layer", flush=True)
    return stage


def add_three_by_three_restaurant_tiles(stage):
    """Tile the original dining/decor content over a wall-free 3x3 floor."""
    tiles_root = UsdGeom.Xform.Define(stage, "/World/RestaurantTiles")
    created = 0
    external_tables = 0
    external_plants = 0
    disabled_colliders = 0
    disabled_rigid_bodies = 0
    plant_patterns = {
        (0, 0): (0,),
        (0, 2): (1, 3),
        (1, 0): (2,),
        (1, 1): (0, 3),
        (1, 2): (),
        (2, 0): (1, 2),
        (2, 1): (),
        (2, 2): (3,),
    }
    # Keep the kitchen on the north (+Y) side untouched. The original dining
    # room is the north row; expansion proceeds only sideways and southward.
    for row, offset_y in enumerate((0.0, -10.0, -20.0)):
        for column, offset_x in enumerate((-12.0, 0.0, 12.0)):
            if offset_x == 0.0 and offset_y == 0.0:
                continue
            tile_path = f"{tiles_root.GetPath()}/Tile_{row}_{column}"
            tile = UsdGeom.Xform.Define(stage, tile_path)
            tile.AddTranslateOp().Set(Gf.Vec3d(offset_x, offset_y, 0.0))
            dining_copy = UsdGeom.Xform.Define(stage, f"{tile_path}/Dining")
            dining_copy.GetPrim().GetReferences().AddInternalReference(
                Sdf.Path("/World/Dining")
            )
            for prim in Usd.PrimRange(dining_copy.GetPrim()):
                if prim.GetName().startswith("TableSet_"):
                    external_tables += 1
                if prim.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
                    disabled_colliders += 1
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
                    disabled_rigid_bodies += 1

            decor_copy = UsdGeom.Xform.Define(stage, f"{tile_path}/Decor")
            for plant_index in plant_patterns[(row, column)]:
                for source_name in (
                    f"Plant_{plant_index:02d}",
                    f"PlantCollider_{plant_index:02d}",
                ):
                    plant_copy = stage.DefinePrim(
                        f"{decor_copy.GetPath()}/{source_name}"
                    )
                    plant_copy.GetReferences().AddInternalReference(
                        Sdf.Path(f"/World/Decor/{source_name}")
                    )
                external_plants += 1
            created += 1

    active_colliders = []
    active_rigid_bodies = []
    for prim in Usd.PrimRange(tiles_root.GetPrim()):
        if "/Dining/" not in str(prim.GetPath()):
            continue
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            enabled = UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
            if enabled is not False:
                active_colliders.append(str(prim.GetPath()))
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            enabled = UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Get()
            if enabled is not False:
                active_rigid_bodies.append(str(prim.GetPath()))
    if external_tables != 32 or active_colliders or active_rigid_bodies:
        raise RuntimeError(
            "external table physics verification failed: "
            f"tables={external_tables} active_colliders={active_colliders} "
            f"active_rigid_bodies={active_rigid_bodies}"
        )
    print(
        f"[restaurant] tiles=3x3 added={created} floor=36x30m "
        f"outer_walls=8 kitchen_bump=1 external_tables={external_tables} "
        f"external_plants={external_plants} "
        f"disabled_colliders={disabled_colliders} "
        f"disabled_rigid_bodies={disabled_rigid_bodies}",
        flush=True,
    )


def add_outer_wall_finish(stage):
    """Add wood wainscot and brass trim to the eight outer wall segments."""
    finish_root = UsdGeom.Xform.Define(
        stage, "/World/Architecture/OuterWallFinish"
    )
    segments = {
        "West": ((-18.0, -10.0), (0.18, 30.0)),
        "East": ((18.0, -10.0), (0.18, 30.0)),
        "South": ((0.0, -25.0), (36.0, 0.18)),
        "NorthWest": ((-10.36, 5.0), (15.28, 0.18)),
        "NorthEast": ((10.36, 5.0), (15.28, 0.18)),
        "KitchenWest": ((-2.72, 7.535), (0.18, 5.07)),
        "KitchenEast": ((2.72, 7.535), (0.18, 5.07)),
        "KitchenNorth": ((0.0, 10.07), (5.44, 0.18)),
    }
    for name, (center, footprint) in segments.items():
        wainscot = UsdGeom.Cube.Define(
            stage, f"{finish_root.GetPath()}/{name}_Wainscot"
        )
        wainscot.CreateSizeAttr(1.0)
        wainscot.CreateDisplayColorAttr([(0.27, 0.12, 0.055)])
        wainscot.AddTranslateOp().Set(Gf.Vec3d(center[0], center[1], 0.5))
        wainscot.AddScaleOp().Set(Gf.Vec3f(footprint[0], footprint[1], 1.0))

        trim = UsdGeom.Cube.Define(
            stage, f"{finish_root.GetPath()}/{name}_BrassTrim"
        )
        trim.CreateSizeAttr(1.0)
        trim.CreateDisplayColorAttr([(0.72, 0.50, 0.16)])
        trim.AddTranslateOp().Set(Gf.Vec3d(center[0], center[1], 1.04))
        trim.AddScaleOp().Set(Gf.Vec3f(footprint[0], footprint[1], 0.08))
    print("[restaurant] outer_wall_finish=wood+brass segments=8", flush=True)


def configure_joint_drives(stage):
    for prim in stage.Traverse():
        name = prim.GetName()
        if name in ARM_JOINTS:
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            drive.CreateStiffnessAttr(ARM_DRIVE_STIFFNESS)
            drive.CreateDampingAttr(ARM_DRIVE_DAMPING)
            drive.CreateMaxForceAttr(ARM_DRIVE_MAX_FORCE)
        elif name in SLIDING_TRAY_JOINTS:
            drive = UsdPhysics.DriveAPI.Apply(prim, "linear")
            drive.CreateStiffnessAttr(TRAY_DRIVE_STIFFNESS)
            drive.CreateDampingAttr(TRAY_DRIVE_DAMPING)
            drive.CreateMaxForceAttr(TRAY_DRIVE_MAX_FORCE)
            drive.CreateTargetPositionAttr(0.0)
        elif name in WHEEL_JOINTS:
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            drive.CreateStiffnessAttr(0.0)
            drive.CreateDampingAttr(WHEEL_DRIVE_DAMPING)
            drive.CreateMaxForceAttr(WHEEL_DRIVE_MAX_FORCE)
            drive.CreateTargetVelocityAttr(0.0)


def configure_physics_stability(stage, articulation_path: str):
    """Configure stable 60 Hz PhysX, with GPU dynamics as an opt-in test."""
    scene_prim = stage.GetPrimAtPath("/World/PhysicsScene")
    if not scene_prim.IsValid():
        raise RuntimeError("restaurant PhysicsScene is missing")
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene_prim)
    physx_scene.CreateEnableStabilizationAttr(True)
    use_gpu_physics = os.environ.get("NAV_ROBOT_GPU_PHYSX", "0") == "1"
    if use_gpu_physics:
        # Isaac Sim 5.1 requires the GPU broadphase together with GPU dynamics.
        # CCD is unsupported by that pipeline. Keep this opt-in because this
        # restaurant contains legacy triangle-mesh colliders that have been
        # more stable with CPU PhysX/MBP.
        physx_scene.CreateEnableCCDAttr(False)
        physx_scene.CreateBroadphaseTypeAttr("GPU")
        physx_scene.CreateEnableGPUDynamicsAttr(True)
        physics_label = "GPU/GPU-broadphase"
    else:
        physx_scene.CreateEnableGPUDynamicsAttr(False)
        physx_scene.CreateBroadphaseTypeAttr("MBP")
        physics_label = "CPU/MBP"
    # Two complete mobile manipulators, cameras and LiDARs cannot reliably
    # sustain the former CPU PhysX 120 Hz budget in real time.  The serving
    # controllers already use a 60 Hz control period, so matching PhysX to
    # 60 Hz restores wall-clock speed without changing commanded velocities.
    physx_scene.CreateTimeStepsPerSecondAttr(60)

    articulation_api = PhysxSchema.PhysxArticulationAPI.Apply(
        stage.GetPrimAtPath(articulation_path)
    )
    articulation_api.CreateSolverPositionIterationCountAttr(32)
    articulation_api.CreateSolverVelocityIterationCountAttr(4)
    articulation_api.CreateStabilizationThresholdAttr(0.01)
    articulation_api.CreateSleepThresholdAttr(0.05)
    print(
        f"[nav_robot] physics={physics_label}/60Hz "
        "stabilization=on solver=32/4",
        flush=True,
    )


def configure_wheel_contact_material(stage):
    """Bind moderate tire friction so the two drive wheels grip the floor."""
    material = UsdShade.Material.Define(
        stage, "/World/PhysicsMaterials/RidgebackTire"
    )
    physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_material.CreateStaticFrictionAttr(TIRE_STATIC_FRICTION)
    physics_material.CreateDynamicFrictionAttr(TIRE_DYNAMIC_FRICTION)
    physics_material.CreateRestitutionAttr(0.0)
    physx_material = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
    physx_material.CreateFrictionCombineModeAttr("average")
    physx_material.CreateRestitutionCombineModeAttr("average")

    wheel_links = {"left_wheel_link", "right_wheel_link"}
    bound_colliders = []
    for prim in stage.Traverse():
        if (
            prim.GetName() == "collisions"
            and prim.GetParent().GetName() in wheel_links
            and str(prim.GetPath()).startswith(ROBOT_ROOT)
        ):
            binding_api = UsdShade.MaterialBindingAPI.Apply(prim)
            binding_api.Bind(
                material,
                UsdShade.Tokens.weakerThanDescendants,
                "physics",
            )
            bound_colliders.append(str(prim.GetPath()))

    if len(bound_colliders) != 2:
        wheel_candidates = [
            (
                str(prim.GetPath()),
                prim.GetTypeName(),
                prim.HasAPI(UsdPhysics.CollisionAPI),
            )
            for prim in stage.Traverse()
            if any(link in str(prim.GetPath()) for link in wheel_links)
        ]
        raise RuntimeError(
            "expected two wheel colliders for tire material, got "
            f"{bound_colliders}; candidates={wheel_candidates}"
        )
    print(
        "[nav_robot] tire contact material "
        f"static={TIRE_STATIC_FRICTION:.2f} "
        f"dynamic={TIRE_DYNAMIC_FRICTION:.2f} "
        f"colliders={len(bound_colliders)}",
        flush=True,
    )

    caster_material = UsdShade.Material.Define(
        stage, "/World/PhysicsMaterials/ServingCaster"
    )
    caster_api = UsdPhysics.MaterialAPI.Apply(caster_material.GetPrim())
    caster_api.CreateStaticFrictionAttr(0.03)
    caster_api.CreateDynamicFrictionAttr(0.03)
    caster_api.CreateRestitutionAttr(0.0)
    PhysxSchema.PhysxMaterialAPI.Apply(
        caster_material.GetPrim()
    ).CreateFrictionCombineModeAttr("min")
    caster_links = {"front_caster_link", "rear_caster_link"}
    caster_colliders = []
    for prim in stage.Traverse():
        if (
            prim.GetName() == "collisions"
            and prim.GetParent().GetName() in caster_links
            and str(prim.GetPath()).startswith(ROBOT_ROOT)
        ):
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                caster_material,
                UsdShade.Tokens.weakerThanDescendants,
                "physics",
            )
            caster_colliders.append(str(prim.GetPath()))
    if len(caster_colliders) != 2:
        raise RuntimeError(
            f"expected two caster colliders, got {caster_colliders}"
        )


def configure_gripper_contact_material(stage):
    """Bind the proven high-friction physics material to RG2 moving links."""
    material = UsdShade.Material.Define(
        stage, "/World/PhysicsMaterials/RG2Grip"
    )
    material_api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    material_api.CreateStaticFrictionAttr(GRIP_CONTACT_STATIC_FRICTION)
    material_api.CreateDynamicFrictionAttr(GRIP_CONTACT_DYNAMIC_FRICTION)
    material_api.CreateRestitutionAttr(0.0)
    physx_material = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
    physx_material.CreateFrictionCombineModeAttr("max")
    physx_material.CreateRestitutionCombineModeAttr("min")

    grip_link_names = {
        "rg2_left_inner_knuckle",
        "rg2_right_outer_knuckle",
        "rg2_left_outer_knuckle",
        "rg2_left_inner_finger",
        "rg2_right_inner_finger",
        "rg2_right_inner_knuckle",
    }
    bound_links = []
    for prim in stage.Traverse():
        if not str(prim.GetPath()).startswith(ROBOT_ROOT):
            continue
        if prim.GetName() not in grip_link_names:
            continue
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            material,
            bindingStrength=UsdShade.Tokens.strongerThanDescendants,
            materialPurpose="physics",
        )
        bound_links.append(str(prim.GetPath()))

    if len(bound_links) != len(grip_link_names):
        raise RuntimeError(
            "RG2 grip material setup incomplete: "
            f"expected={len(grip_link_names)} bound={bound_links}"
        )
    print(
        "[nav_robot] RG2 grip material "
        f"static={GRIP_CONTACT_STATIC_FRICTION:.1f} "
        f"dynamic={GRIP_CONTACT_DYNAMIC_FRICTION:.1f} "
        f"links={len(bound_links)} max_force={GRIPPER_DRIVE_MAX_FORCE:.0f}N",
        flush=True,
    )


def find_articulation_path(stage) -> str:
    for path in ARTICULATION_CANDIDATES:
        prim = stage.GetPrimAtPath(path)
        if (
            prim.IsValid()
            and prim.HasAPI(UsdPhysics.ArticulationRootAPI)
            and prim.HasAPI(UsdPhysics.RigidBodyAPI)
        ):
            return path
    # Fallback: first constrainable articulation root under NavRobot.
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if (
            path.startswith(f"{ROBOT_ROOT}/")
            and prim.HasAPI(UsdPhysics.ArticulationRootAPI)
            and prim.HasAPI(UsdPhysics.RigidBodyAPI)
        ):
            return path
    raise RuntimeError(
        f"could not find rigid articulation root under {ROBOT_ROOT}"
    )


def log_arm_chain(stage, label):
    """Print M0609 link world poses to pinpoint the first separated body."""
    cache = UsdGeom.XformCache()
    root = f"{ROBOT_ROOT}/Robot/ridgeback_base_link"
    rows = []
    for name in ("base_link", *(f"link_{index}" for index in range(1, 7))):
        prim = stage.GetPrimAtPath(f"{root}/{name}")
        if not prim.IsValid():
            rows.append(f"{name}=MISSING")
            continue
        position = cache.GetLocalToWorldTransform(prim).ExtractTranslation()
        rows.append(
            f"{name}=({float(position[0]):.3f},"
            f"{float(position[1]):.3f},{float(position[2]):.3f})"
        )
    joint_2 = stage.GetPrimAtPath(f"{root}/joints/joint_2")
    joint_state = "missing"
    if joint_2.IsValid():
        joint = UsdPhysics.Joint(joint_2)
        joint_state = (
            f"enabled={joint.GetJointEnabledAttr().Get()} "
            f"body0={joint.GetBody0Rel().GetTargets()} "
            f"body1={joint.GetBody1Rel().GetTargets()}"
        )
    print(
        f"[arm-chain:{label}] {' '.join(rows)} joint_2[{joint_state}]",
        flush=True,
    )


def log_stray_robot_geometry(stage):
    """List robot geometry rendered near the world origin, away from the robot."""
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
    )
    candidates = []
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not path.startswith(ROBOT_ROOT) or not prim.IsA(UsdGeom.Boundable):
            continue
        try:
            bound = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            center = bound.GetMidpoint()
            size = bound.GetSize()
        except Exception:
            continue
        if (
            abs(float(center[0])) <= 1.5
            and abs(float(center[1])) <= 1.5
            and -0.2 <= float(center[2]) <= 1.5
        ):
            candidates.append(
                f"{path} type={prim.GetTypeName()} "
                f"center=({float(center[0]):.3f},{float(center[1]):.3f},"
                f"{float(center[2]):.3f}) size=({float(size[0]):.3f},"
                f"{float(size[1]):.3f},{float(size[2]):.3f})"
            )
    if candidates:
        for candidate in candidates:
            print(f"[stray-geometry] {candidate}", flush=True)
    else:
        print("[stray-geometry] no USD Boundable under NavRobot near origin", flush=True)


def initialize_robot(articulation_path: str, name: str = "nav_ridgeback"):
    """Bind one fully composed articulation to the running PhysX scene."""
    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        raise RuntimeError(
            f"cannot initialize articulation before physics starts: "
            f"{articulation_path}"
        )

    articulation = SingleArticulation(prim_path=articulation_path, name=name)
    retry_frames = max(
        0, int(os.environ.get("NAV_ARTICULATION_INIT_RETRY_FRAMES", "30"))
    )
    last_error = None
    for attempt in range(retry_frames + 1):
        try:
            articulation.initialize()
            if articulation.handles_initialized:
                break
        except Exception as exc:
            last_error = exc
        if attempt < retry_frames:
            simulation_app.update()
    else:
        detail = f": {last_error}" if last_error is not None else ""
        raise RuntimeError(
            f"PhysX did not create an articulation handle after "
            f"{retry_frames + 1} attempts: {articulation_path}{detail}"
        ) from last_error

    if not articulation.handles_initialized:
        raise RuntimeError(
            f"PhysX did not create an articulation handle: {articulation_path}"
        )
    if last_error is not None:
        print(
            f"[nav_robot] articulation recovered after delayed PhysX "
            f"registration: {articulation_path}",
            flush=True,
        )
    articulation.set_enabled_self_collisions(False)

    dof_names = list(articulation.dof_names)
    missing_wheels = set(WHEEL_JOINTS) - set(dof_names)
    if missing_wheels:
        raise RuntimeError(f"missing wheel DOFs: {sorted(missing_wheels)}")
    missing_trays = set(SLIDING_TRAY_JOINTS) - set(dof_names)
    if missing_trays:
        raise RuntimeError(
            "obsolete robot articulation loaded; missing sliding tray DOFs: "
            f"{sorted(missing_trays)}"
        )

    positions = articulation.get_joint_positions()
    for name in WHEEL_JOINTS:
        positions[dof_names.index(name)] = 0.0
    for name in SLIDING_TRAY_JOINTS:
        positions[dof_names.index(name)] = 0.0
    for name, value in zip(ARM_JOINTS, STOW_CONFIGURATION):
        if name in dof_names:
            positions[dof_names.index(name)] = value
    articulation.set_joint_positions(positions)
    articulation.set_joint_velocities(np.zeros(len(dof_names), dtype=float))

    arm_present = [name for name in ARM_JOINTS if name in dof_names]
    if arm_present:
        arm_indices = np.asarray(
            [dof_names.index(name) for name in arm_present], dtype=np.int32
        )
        articulation.apply_action(
            ArticulationAction(
                joint_positions=np.asarray(
                    STOW_CONFIGURATION[: len(arm_present)], dtype=float
                ),
                joint_indices=arm_indices,
            )
        )
    print(f"[ready] articulation={articulation_path} dofs={dof_names}", flush=True)
    return articulation, dof_names


def create_lidar(stage, parent_path: str) -> str:
    parent = stage.GetPrimAtPath(parent_path)
    if not parent.IsValid():
        # Prefer ridgeback_base_link under Robot
        for prim in stage.Traverse():
            if prim.GetName() == BASE_LINK_NAME and str(prim.GetPath()).startswith(
                ROBOT_ROOT
            ):
                parent_path = str(prim.GetPath())
                break

    result, lidar = omni.kit.commands.execute(
        "RangeSensorCreateLidar",
        path="nav_lidar",
        parent=parent_path,
        min_range=0.2,
        max_range=20.0,
        draw_points=False,
        draw_lines=False,
        horizontal_fov=360.0,
        vertical_fov=30.0,
        horizontal_resolution=0.4,
        vertical_resolution=4.0,
        rotation_rate=0.0,
        high_lod=False,
        yaw_offset=0.0,
        enable_semantics=False,
    )
    if not result:
        raise RuntimeError("RangeSensorCreateLidar failed")
    lidar_path = str(lidar.GetPath())
    lidar.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3d(0.0, 0.0, 0.35))
    print(f"[sensor] lidar={lidar_path}", flush=True)
    return lidar_path


def create_depth_camera(stage, parent_path: str) -> str:
    for prim in stage.Traverse():
        if prim.GetName() == BASE_LINK_NAME and str(prim.GetPath()).startswith(
            ROBOT_ROOT
        ):
            parent_path = str(prim.GetPath())
            break

    cam_path = f"{parent_path}/nav_depth_camera"
    camera = UsdGeom.Camera.Define(stage, cam_path)
    xform = UsdGeom.Xformable(camera.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(0.25, 0.0, 0.55))
    xform.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 15.0, 0.0))
    camera.CreateFocalLengthAttr(18.0)
    camera.CreateHorizontalApertureAttr(20.955)
    camera.CreateVerticalApertureAttr(15.0)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.15, 8.0))
    print(f"[sensor] depth_camera={cam_path}", flush=True)
    return cam_path


def create_sensor_ros_graph(lidar_path: str, camera_path: str):
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/World/NavRobot/NavSensorsROS2", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("LidarBeams", "isaacsim.sensors.physx.IsaacReadLidarBeams"),
                ("LaserScanPub", "isaacsim.ros2.bridge.ROS2PublishLaserScan"),
                ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("DepthPclPub", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ],
            keys.SET_VALUES: [
                ("LidarBeams.inputs:lidarPrim", [usdrt.Sdf.Path(lidar_path)]),
                ("LaserScanPub.inputs:topicName", "nav_robot/scan"),
                ("LaserScanPub.inputs:frameId", "nav_lidar_frame"),
                ("RenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(camera_path)]),
                ("RenderProduct.inputs:width", 640),
                ("RenderProduct.inputs:height", 480),
                ("DepthPclPub.inputs:nodeNamespace", "nav_robot"),
                ("DepthPclPub.inputs:topicName", "depth/points"),
                ("DepthPclPub.inputs:frameId", "nav_depth_optical_frame"),
                ("DepthPclPub.inputs:type", "depth_pcl"),
                ("DepthPclPub.inputs:frameSkipCount", 1),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "LidarBeams.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "RenderProduct.inputs:execIn"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("Context.outputs:context", "LaserScanPub.inputs:context"),
                ("Context.outputs:context", "DepthPclPub.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "LaserScanPub.inputs:timeStamp"),
                ("LidarBeams.outputs:execOut", "LaserScanPub.inputs:execIn"),
                ("LidarBeams.outputs:azimuthRange", "LaserScanPub.inputs:azimuthRange"),
                ("LidarBeams.outputs:depthRange", "LaserScanPub.inputs:depthRange"),
                ("LidarBeams.outputs:horizontalFov", "LaserScanPub.inputs:horizontalFov"),
                (
                    "LidarBeams.outputs:horizontalResolution",
                    "LaserScanPub.inputs:horizontalResolution",
                ),
                (
                    "LidarBeams.outputs:intensitiesData",
                    "LaserScanPub.inputs:intensitiesData",
                ),
                (
                    "LidarBeams.outputs:linearDepthData",
                    "LaserScanPub.inputs:linearDepthData",
                ),
                ("LidarBeams.outputs:numCols", "LaserScanPub.inputs:numCols"),
                ("LidarBeams.outputs:numRows", "LaserScanPub.inputs:numRows"),
                ("LidarBeams.outputs:rotationRate", "LaserScanPub.inputs:rotationRate"),
                ("RenderProduct.outputs:execOut", "DepthPclPub.inputs:execIn"),
                (
                    "RenderProduct.outputs:renderProductPath",
                    "DepthPclPub.inputs:renderProductPath",
                ),
            ],
        },
    )
    print(
        "[ros] /clock /nav_robot/scan /nav_robot/depth/points publishers ready",
        flush=True,
    )


def connect_embedded_sensor_ros(
    stage,
    *,
    robot_root=ROBOT_ROOT,
    robot_name="",
    publish_clock=True,
):
    """Connect the D455/RPLIDAR already contained in the two-wheel USD."""
    global _embedded_lidar_render_product, _embedded_lidar_writer
    camera_width = int(os.environ.get("NAV_CAMERA_WIDTH", "1280"))
    camera_height = int(os.environ.get("NAV_CAMERA_HEIGHT", "960"))
    rgb_frame_skip = int(os.environ.get("NAV_CAMERA_FRAME_SKIP", "3"))
    depth_enabled = os.environ.get("NAV_CAMERA_DEPTH_ENABLED", "0") == "1"
    depth_frame_skip = int(os.environ.get("NAV_CAMERA_DEPTH_FRAME_SKIP", "29"))
    if camera_width <= 0 or camera_height <= 0:
        raise ValueError(
            "NAV_CAMERA_WIDTH and NAV_CAMERA_HEIGHT must be positive"
        )
    if rgb_frame_skip < 0 or depth_frame_skip < 0:
        raise ValueError("camera frame skip values must be non-negative")
    base_path = f"{robot_root}/Robot/ridgeback_base_link/ridgeback_base_link"
    sensor_mount = f"{base_path}/fixed_table_depth_camera/realsense_d455"
    depth_camera = f"{sensor_mount}/RSD455/Camera_Pseudo_Depth"
    lidar_path = f"{base_path}/base_scan/RPLIDAR_S2E"
    for required in (depth_camera, lidar_path):
        if not stage.GetPrimAtPath(required).IsValid():
            raise RuntimeError(f"embedded sensor prim is missing: {required}")

    keys = og.Controller.Keys
    nodes = [
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("Context", "isaacsim.ros2.bridge.ROS2Context"),
        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
        ("ColorPub", "isaacsim.ros2.bridge.ROS2CameraHelper"),
    ]
    connections = [
        ("OnPlaybackTick.outputs:tick", "RenderProduct.inputs:execIn"),
        ("Context.outputs:context", "ColorPub.inputs:context"),
        ("RenderProduct.outputs:execOut", "ColorPub.inputs:execIn"),
        ("RenderProduct.outputs:renderProductPath", "ColorPub.inputs:renderProductPath"),
    ]
    if depth_enabled:
        nodes.append(("DepthPub", "isaacsim.ros2.bridge.ROS2CameraHelper"))
        connections.extend([
            ("Context.outputs:context", "DepthPub.inputs:context"),
            ("RenderProduct.outputs:execOut", "DepthPub.inputs:execIn"),
            ("RenderProduct.outputs:renderProductPath", "DepthPub.inputs:renderProductPath"),
        ])
    if publish_clock:
        nodes.append(("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"))
        connections.extend([
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("Context.outputs:context", "PublishClock.inputs:context"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
        ])
    camera_prefix = f"{robot_name}/" if robot_name else ""
    sensor_values = [
        ("RenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(depth_camera)]),
        ("RenderProduct.inputs:width", camera_width),
        ("RenderProduct.inputs:height", camera_height),
        ("ColorPub.inputs:nodeNamespace", f"{camera_prefix}camera/color"),
        ("ColorPub.inputs:topicName", "image_raw"),
        ("ColorPub.inputs:frameId", "d455_color_optical_frame"),
        ("ColorPub.inputs:type", "rgb"),
        # At 60 Hz, skip=3 publishes the original 1280x960 image at 15 Hz.
        # Resolution and YOLO input quality are unchanged.
        ("ColorPub.inputs:frameSkipCount", rgb_frame_skip),
    ]
    if depth_enabled:
        sensor_values.extend([
            ("DepthPub.inputs:nodeNamespace", f"{camera_prefix}camera/depth"),
            ("DepthPub.inputs:topicName", "image_raw"),
            ("DepthPub.inputs:frameId", "d455_depth_optical_frame"),
            ("DepthPub.inputs:type", "depth"),
            ("DepthPub.inputs:frameSkipCount", depth_frame_skip),
        ])
    og.Controller.edit(
        {"graph_path": f"{robot_root}/EmbeddedSensorsROS2", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: nodes,
            keys.SET_VALUES: sensor_values,
            keys.CONNECT: connections,
        },
    )

    print(
        f"[ros] {robot_name or 'default'} embedded D455 RGB/depth "
        f"{camera_width}x{camera_height} rgb_skip={rgb_frame_skip} "
        f"depth={int(depth_enabled)} clock={int(publish_clock)} connected",
        flush=True,
    )


def create_sensor_static_tf(stage, lidar_path: str, camera_path: str, node: Node, broadcaster):
    # Published every odom tick as well; initial helper keeps frames available.
    pass


class CommandServingSequence:
    """Frame-driven composition of the already tested serving tasks."""

    TRAY_TASK_NAMES = frozenset({"soda1", "soda2", "cutlery", "plate_rack"})
    TRAY_EXTENSION = 0.25
    TRAY_TOLERANCE = 0.005

    def __init__(self, named_tasks):
        self._named_tasks = list(named_tasks)
        if not self._named_tasks:
            raise ValueError("integrated serving sequence contains no tasks")
        names = [name for name, _ in self._named_tasks]
        self._has_pizza = "pizza" in names
        self._has_tray_payload = any(
            name in self.TRAY_TASK_NAMES for name in names
        )
        self._index = 0
        self._tray_indices = None
        self.done = False
        self.failed = False

    def initialize(self, articulation, dof_names):
        self._tray_indices = np.asarray(
            [dof_names.index(name) for name in SLIDING_TRAY_JOINTS],
            dtype=np.int32,
        )
        for name, task in self._named_tasks:
            if name == "pizza":
                if not hasattr(task, "set_parallel_tray_deployment"):
                    raise RuntimeError(
                        "pizza task does not support parallel tray deployment"
                    )
                task.set_parallel_tray_deployment(self._has_tray_payload)
            task.initialize(articulation, dof_names)
        names = " -> ".join(name for name, _ in self._named_tasks)
        mode = (
            "parallel-pizza-tray"
            if self._has_pizza and self._has_tray_payload
            else "pizza-only"
            if self._has_pizza
            else "first-payload-deploys-tray"
        )
        print(
            f"[integrated-serving] order={names} mode={mode} "
            "(all tasks initialized)",
            flush=True,
        )

    def _trays_are_physically_deployed(self, articulation):
        actual = np.asarray(
            articulation.get_joint_positions()[self._tray_indices],
            dtype=float,
        )
        error = float(np.max(np.abs(actual - self.TRAY_EXTENSION)))
        return error <= self.TRAY_TOLERANCE, actual, error

    @property
    def current_name(self):
        if self.done or self.failed or self._index >= len(self._named_tasks):
            return None
        return self._named_tasks[self._index][0]

    def step(self, articulation):
        if self.done or self.failed:
            return
        name, task = self._named_tasks[self._index]
        task.step(articulation)
        if task.failed:
            self.failed = True
            print(f"[integrated-serving] STOPPED: {name} failed", flush=True)
            return
        if not task.done:
            return
        self._index += 1
        if self._index >= len(self._named_tasks):
            self.done = True
            print("[integrated-serving] all requested deliveries complete", flush=True)
            return
        next_name, next_task = self._named_tasks[self._index]
        print(f"[integrated-serving] starting {next_name}", flush=True)
        if hasattr(next_task, "start_with_deployed_trays"):
            deployed, actual, error = self._trays_are_physically_deployed(
                articulation
            )
            if not deployed:
                self.failed = True
                print(
                    "[integrated-serving] STOPPED: refusing to start "
                    f"{next_name}; trays are not physically deployed "
                    f"actual={np.round(actual, 4).tolist()}m "
                    f"error={error:.4f}m",
                    flush=True,
                )
                return
            next_task.start_with_deployed_trays()

    def close(self):
        for _, task in self._named_tasks:
            try:
                task.close()
            except Exception:
                pass


class NavBridge(Node):
    """cmd_vel subscriber + odom/TF publisher + food spawn & arm serving server."""

    def __init__(
        self,
        articulation,
        dof_names,
        stage=None,
        *,
        robot_name="",
        robot_root=ROBOT_ROOT,
    ):
        namespace = f"/{robot_name}" if robot_name else ""
        node_name = (
            f"nav_robot_isaac_bridge_{robot_name}"
            if robot_name
            else "nav_robot_isaac_bridge"
        )
        super().__init__(node_name, namespace=namespace)
        self.articulation = articulation
        self.dof_names = dof_names
        self.stage = stage
        self.robot_name = robot_name
        self.robot_root = robot_root
        self.payload_root = (
            f"/World/RobotPayloads/{robot_name}"
            if robot_name
            else "/World"
        )
        self.delivered_root = (
            f"/World/Delivered/{robot_name}"
            if robot_name
            else "/World/Delivered"
        )
        if self.stage is not None and self.robot_name:
            UsdGeom.Scope.Define(self.stage, "/World/RobotPayloads")
            UsdGeom.Scope.Define(self.stage, self.payload_root)
            UsdGeom.Scope.Define(self.stage, "/World/Delivered")
            UsdGeom.Scope.Define(self.stage, self.delivered_root)
        self.get_logger().info(
            f"payload isolation robot={self.robot_name or 'default'} "
            f"payload_root={self.payload_root} "
            f"delivered_root={self.delivered_root}"
        )
        self.wheel_indices = np.asarray(
            [dof_names.index(name) for name in WHEEL_JOINTS], dtype=np.int32
        )
        self._target_vx = 0.0
        self._target_wz = 0.0
        self._cmd_vx = 0.0
        self._cmd_wz = 0.0
        self._last_cmd_time = time.monotonic()
        self._warned_vy = False
        self._lock = threading.Lock()
        self._last_pose = None
        self._odom_initialized = False
        self._pending_teleport = None  # (x, y, z, yaw) applied on sim thread
        self._pending_food_spawn = None
        self._pending_arm_command = None
        self._active_serving_task = None
        self._serving_paused = False
        self._arm_returning_to_stow = False
        self._arm_stow_started_at = None
        self._arm_stow_settle_count = 0
        self._arm_stow_command = None
        self._arm_stow_last_update = None
        self._arm_stow_last_log = None
        self._tray_returning_home = False
        self._tray_home_started_at = None
        self._tray_home_settle_count = 0
        self._tray_home_step = 0
        self._tray_home_start = None
        self._spawned_serving_tasks = {}
        self._plate_rack_in_transport = False
        self._pending_plate_count = 0
        self._direct_nav = None
        self._direct_nav_request = None
        self._active_two_wheel_mission_id = ""
        self._active_two_wheel_target = None
        self._active_two_wheel_goal = None
        self._completed_mission_latch = None
        self._navigation_paused = False
        self._navigation_pause_started = None
        self._navigation_location = 4
        self._last_navigation_heartbeat = 0.0
        self._active_delivery_table = None
        self._completed_delivery_table = None
        self._delivered_trip_counts = {}
        self._hand_test_controller = None
        self._obstacle_stop = False
        self._obstacle_stop_started = None
        self._obstacle_stop_from_peer = False
        self._peer_hit_this_scan = False
        self._clearance_start = None
        self._obstacle_scale = 1.0
        self._last_scan_time = 0.0
        self._latest_obstacle_position = None
        self._last_decel_event_state = False

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        transient_local_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.scan_pub = self.create_publisher(LaserScan, "scan", sensor_qos)
        self.nav_scan_pub = self.create_publisher(LaserScan, "nav_robot/scan", sensor_qos)
        self.two_wheel_scan_pub = self.create_publisher(LaserScan, "two_wheel/scan_raw", sensor_qos)
        self.create_subscription(Twist, "nav_robot/cmd_vel", self._on_cmd_vel, qos)
        # nav2_collision_monitor sits between the controller/velocity_smoother
        # output and the robot: it re-publishes "cmd_vel" as "cmd_vel_safe"
        # after gating it against the stop/slowdown polygons in
        # nav2_params.yaml.  Listening here (not on raw "cmd_vel") means both
        # Nav2's planned driving and rail_navigator's direct backup commands
        # (which also publish to "cmd_vel") get the same safety gate.
        self.create_subscription(Twist, "cmd_vel_safe", self._on_cmd_vel, qos)
        self.create_subscription(
            PoseStamped, "nav_robot/teleport", self._on_teleport, qos
        )
        self.create_subscription(
            PoseStamped, "two_wheel/teleport", self._on_teleport, qos
        )
        self.odom_pub = self.create_publisher(Odometry, "nav_robot/odom", qos)
        self.two_wheel_odom_pub = self.create_publisher(Odometry, "two_wheel/odom_raw", qos)
        # This is state consumed by the HMI, not a fire-and-forget command.
        # Latch the latest detection so an HMI started/restarted after the
        # person was detected still receives the active map marker.
        self.obstacle_pub = self.create_publisher(
            String,
            "serving_robot/obstacle_event",
            transient_local_qos,
        )
        self.tf_pub = self.create_publisher(TFMessage, "tf", 100)
        static_tf_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.static_tf_pub = self.create_publisher(
            TFMessage, "tf_static", static_tf_qos
        )

        # Subsystems Services & Publishers for Manager Node
        self.spawn_status_pub = self.create_publisher(Int32, "food_spawn/status", transient_local_qos)
        self.arm_status_pub = self.create_publisher(Int32, "arm/status", transient_local_qos)
        # navigation/status Int32 is owned ONLY by navigation_subsystem.
        # Isaac used to also publish it (plus a 1Hz MOVING/ARRIVED heartbeat),
        # which made manager treat mid-attempt failures / idle gaps as terminal
        # ARRIVED/FAILED and cancelled orders. Keep location for TF-less HMI.
        self.navigation_location_pub = self.create_publisher(
            Int32, "navigation/current_location", transient_local_qos
        )

        # Standard Int32 topic subscribers for Isaac Sim food spawning & arm serving
        self.create_subscription(Int32, "food_spawn/trigger", self._on_food_spawn_trigger, qos)
        self.create_subscription(Int32, "arm/trigger", self._on_arm_trigger, qos)
        self.create_subscription(
            Int32, "navigation/trigger", self._on_navigation_trigger, qos
        )

        mission_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.two_wheel_mission_status_pub = self.create_publisher(
            String, "two_wheel/mission_status", mission_qos
        )
        self.create_subscription(
            String,
            "two_wheel/mission_command",
            self._on_two_wheel_mission_command,
            mission_qos,
        )
        # Fleet right-of-way: later order yields when lidar sees the peer robot.
        self._fleet_priorities = {}
        self._fleet_active = {}
        self._fleet_phases = {}
        self._my_fleet_priority = None
        self._peer_swerve_wz = 0.0
        for peer in ("robot1", "robot2"):
            self.create_subscription(
                String,
                f"/{peer}/fleet/intent",
                lambda msg, name=peer: self._on_fleet_intent(name, msg),
                mission_qos,
            )

        self._obstacle_test_controller = None
        self.create_service(
            SetBool,
            "hand_test/set_visible",
            self._on_hand_test_set_visible,
        )
        self.create_service(
            SetBool,
            "obstacle_test/set_visible",
            self._on_obstacle_test_set_visible,
        )

        if TaskCommand is not None:
            self.create_service(
                TaskCommand,
                "food_spawn/command",
                self._on_food_spawn_command,
            )
            self.create_service(
                TaskCommand,
                "arm/command",
                self._on_arm_command,
            )

        # RTX sensor timestamps are based on the ROS /clock graph.  The Python
        # subscriber receives that clock a few render frames later, so the
        # dynamic TF is future-dated slightly below to cover the transport
        # latency seen by RViz/Nav2.
        self._sim_stamp = None
        clock_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Clock, "/clock", self._on_clock, clock_qos)

        self._sensor_offsets = {
            # RTX RPLidar publishes /scan in this frame.
            "base_scan": (
                float(FRONT_LIDAR_TRANSLATION[0]),
                float(FRONT_LIDAR_TRANSLATION[1]),
                float(FRONT_LIDAR_TRANSLATION[2]),
                0.0,
            ),
            "nav_lidar_frame": (0.0, 0.0, 0.35, 0.0),
            "nav_depth_optical_frame": (0.25, 0.0, 0.55, 0.0),
        }
        self._publish_static_sensor_tf()
        self.navigation_location_pub.publish(Int32(data=4))
        subsystem_services = (
            ", /food_spawn/command, /arm/command"
            if TaskCommand is not None
            else " (custom subsystem services unavailable)"
        )
        print(
            f"[ros] active services: /navigation/command{subsystem_services}\n"
            "[ros] status: navigation/status owned by ROS subsystem only; "
            "Isaac publishes location + two_wheel/mission_status",
            flush=True,
        )

    def set_hand_test_controller(self, controller):
        self._hand_test_controller = controller

    def _on_hand_test_set_visible(self, request, response):
        controller = self._hand_test_controller
        if controller is None:
            response.success = False
            response.message = "hand-only test controller is not ready"
            return response
        controller.request_visible(bool(request.data))
        response.success = True
        response.message = (
            "hand spawn queued" if request.data else "hand removal queued"
        )
        return response

    def set_obstacle_test_controller(self, controller):
        self._obstacle_test_controller = controller

    def _on_obstacle_test_set_visible(self, request, response):
        controller = self._obstacle_test_controller
        if controller is None:
            response.success = False
            response.message = "corridor obstacle test controller is not ready"
            return response
        controller.request_visible(bool(request.data))
        response.success = True
        response.message = (
            "corridor person spawn queued" if request.data else "corridor person removal queued"
        )
        return response

    def _archive_delivered_payloads(self, payload_paths):
        """Preserve a visual-only snapshot without moving live physics prims."""
        table_id = self._completed_delivery_table
        if table_id not in (0, 1, 2, 3):
            return False
        trip_number = self._delivered_trip_counts.get(table_id, 0) + 1
        table_root = f"{self.delivered_root}/Table{table_id}"
        archive_root = f"{table_root}/Trip{trip_number}"
        UsdGeom.Scope.Define(self.stage, "/World/Delivered")
        if self.robot_name:
            UsdGeom.Scope.Define(self.stage, self.delivered_root)
        UsdGeom.Scope.Define(self.stage, table_root)
        UsdGeom.Scope.Define(self.stage, archive_root)
        root_layer = self.stage.GetRootLayer()
        for source_path in payload_paths:
            if not self.stage.GetPrimAtPath(source_path).IsValid():
                continue
            target_path = f"{archive_root}/{source_path.rsplit('/', 1)[-1]}"
            Sdf.CopySpec(
                root_layer,
                Sdf.Path(source_path),
                root_layer,
                Sdf.Path(target_path),
            )
        # Strip every copied physics body, collider and joint before the next
        # simulation update.  The archive is only a rendered snapshot.  The
        # original physics prims stay at their stable paths and are reused by
        # the next spawn, so existing tensor views are never invalidated.
        archive_prim = self.stage.GetPrimAtPath(archive_root)
        copied_prims = list(Usd.PrimRange(archive_prim))
        joint_paths = [
            prim.GetPath() for prim in copied_prims if prim.IsA(UsdPhysics.Joint)
        ]
        for joint_path in reversed(joint_paths):
            self.stage.RemovePrim(joint_path)
        for prim in copied_prims:
            if not prim.IsValid():
                continue
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                prim.RemoveAPI(UsdPhysics.CollisionAPI)
            if prim.HasAPI(UsdPhysics.MassAPI):
                prim.RemoveAPI(UsdPhysics.MassAPI)
        self._delivered_trip_counts[table_id] = trip_number
        self._completed_delivery_table = None
        self.get_logger().info(
            f"archived delivered payloads at {archive_root}"
        )
        return True

    def _on_clock(self, msg: Clock):
        self._sim_stamp = msg.clock

    def _on_food_spawn_trigger(self, msg: Int32):
        self.get_logger().info(f"📥 [FoodSpawn Trigger] Received spawn trigger topic: {msg.data}")
        self.spawn_status_pub.publish(Int32(data=1))  # 1 = WORKING
        with self._lock:
            self._pending_food_spawn = int(msg.data)

    def _on_food_spawn_command(self, request, response):
        cmd = request.command
        self.get_logger().info(f"📥 [FoodSpawn] Received /food_spawn/command: {cmd}")
        response.success = True
        # The simulation loop services this callback, then performs the USD
        # mutation from tick().  Do not publish or change bridge state from a
        # detached worker while Isaac's native plugins are updating.
        self.spawn_status_pub.publish(Int32(data=1))  # 1 = WORKING
        with self._lock:
            self._pending_food_spawn = int(cmd)
        return response

    def _on_arm_trigger(self, msg: Int32):
        self.get_logger().info(f"📥 [Arm Trigger] Received arm trigger topic: {msg.data}")
        self._queue_arm_command(int(msg.data))

    def _on_arm_command(self, request, response):
        cmd = request.command
        self.get_logger().info(f"📥 [ArmServer] Received /arm/command: {cmd}")
        response.success = self._queue_arm_command(int(cmd))
        return response

    def _queue_arm_command(self, command):
        with self._lock:
            if command == 99:
                if (
                    self._active_serving_task is None
                    and not self._arm_returning_to_stow
                    and not self._tray_returning_home
                ):
                    return False
                self._serving_paused = True
                self.get_logger().warning("integrated serving paused")
                return True
            if command == 98:
                if (
                    self._active_serving_task is None
                    and not self._arm_returning_to_stow
                    and not self._tray_returning_home
                ):
                    return False
                self._serving_paused = False
                self.get_logger().info("integrated serving resumed")
                return True
            if (
                command <= 0
                or self._active_serving_task is not None
                or self._arm_returning_to_stow
                or self._tray_returning_home
            ):
                return False
            self._pending_arm_command = command
            self._active_delivery_table = (
                self._navigation_location
                if self._navigation_location in (0, 1, 2, 3)
                else None
            )
        self.arm_status_pub.publish(Int32(data=1))
        return True

    def _publish_two_wheel_mission_status(
        self, state, phase, reason="", mission_id=None, **extra
    ):
        active_id = mission_id or self._active_two_wheel_mission_id
        if not active_id:
            return
        payload = {
            "mission_id": active_id,
            "state": str(state),
            "phase": str(phase),
        }
        if reason:
            payload["reason"] = str(reason)
        payload.update(extra)
        self.two_wheel_mission_status_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )

    @staticmethod
    def _two_wheel_target_from_payload(payload):
        mission_id = str(payload.get("mission_id", "")).strip().lower()

        for table_id in range(4):
            if mission_id.startswith(f"table_{table_id}_"):
                return table_id
        if mission_id.startswith("kitchen_"):
            return 4

        for key in (
            "target",
            "command",
            "route_id",
            "table_id",
            "destination",
            "goal_name",
        ):
            if key not in payload:
                continue
            value = payload[key]
            if isinstance(value, bool):
                continue
            if isinstance(value, (int, float)):
                value = int(value)
                if value in (0, 1, 2, 3, 4):
                    return value
                continue

            value = str(value).strip().lower()
            if value in ("kitchen", "home", "주방"):
                return 4
            for table_id in range(4):
                if value in (
                    str(table_id),
                    f"table_{table_id}",
                    f"table{table_id}",
                    f"table {table_id}",
                ):
                    return table_id
        return None

    def _on_two_wheel_mission_command(self, msg: String):
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError("payload must be a JSON object")
        except Exception as exc:
            self.get_logger().error(
                f"[Mission RX] invalid JSON: {exc}; raw={msg.data!r}"
            )
            return

        mission_id = str(payload.get("mission_id", "")).strip()
        kind = str(payload.get("kind", "")).strip().lower()

        if kind == "cancel":
            if mission_id and mission_id == self._active_two_wheel_mission_id:
                with self._lock:
                    self._direct_nav = None
                    self._direct_nav_request = None
                    self._target_vx = 0.0
                    self._target_wz = 0.0
                    self._cmd_vx = 0.0
                    self._cmd_wz = 0.0
                self._publish_two_wheel_mission_status(
                    "cancelled", "cancelled", mission_id=mission_id
                )
                self._active_two_wheel_mission_id = ""
                self._active_two_wheel_target = None
                self._active_two_wheel_goal = None
            return

        if kind in ("pause", "resume"):
            # The ROS navigation subsystem can issue safety control before it
            # has received the first mission-status echo.  In that short
            # window its mission_id is empty; target the currently active
            # Isaac mission instead of rejecting the safety command.
            if not mission_id:
                mission_id = self._active_two_wheel_mission_id
            target = 99 if kind == "pause" else 98
            if not mission_id or not self._queue_navigation(target):
                self.get_logger().warning(
                    f"[Mission RX] ignored {kind}: no active mission"
                )
            return

        if not mission_id:
            self.get_logger().error(
                f"[Mission RX] missing mission_id: {payload}"
            )
            return

        # Relative park-out mission: reverse, then rotate 180 degrees.
        if kind == "drive_distance":
            try:
                distance = abs(float(payload.get("distance", 0.0)))
                speed = abs(float(payload.get("speed", 0.12)))
            except (TypeError, ValueError):
                distance = 0.0
                speed = 0.0

            if distance <= 0.0 or speed <= 0.0:
                self._publish_two_wheel_mission_status(
                    "failed",
                    "parse_error",
                    reason=f"invalid drive_distance payload: {payload}",
                    mission_id=mission_id,
                )
                return

            with self._lock:
                busy = (
                    self._direct_nav is not None
                    or self._direct_nav_request is not None
                )
                if not busy:
                    self._active_two_wheel_mission_id = mission_id
                    self._active_two_wheel_target = None
                    self._completed_mission_latch = None
                    self._navigation_paused = False
                    self._navigation_pause_started = None
                    self._obstacle_stop = False
                    self._obstacle_stop_started = None
                    self._obstacle_stop_from_peer = False
                    self._clearance_start = None
                    self._obstacle_scale = 1.0
                    self._direct_nav = dict(
                        mode="park_out",
                        target=None,
                        phase="init",
                        distance=distance,
                        speed=min(speed, 0.20),
                        stage_start=time.monotonic(),
                        wall_start=time.monotonic(),
                        last_log=0.0,
                    )
                    self._target_vx = 0.0
                    self._target_wz = 0.0
                    self._cmd_vx = 0.0
                    self._cmd_wz = 0.0

            if busy:
                self._publish_two_wheel_mission_status(
                    "failed",
                    "busy",
                    reason="navigation already active",
                    mission_id=mission_id,
                )
                return

            self._publish_two_wheel_mission_status(
                "accepted",
                "park_out_backoff",
                distance=distance,
                align_opposite=True,
                mission_id=mission_id,
            )
            self.get_logger().info(
                f"[Mission RX] park-out id={mission_id} "
                f"reverse={distance:.2f}m then align opposite"
            )
            return

        target = self._two_wheel_target_from_payload(payload)
        if target is None:
            self._publish_two_wheel_mission_status(
                "failed",
                "parse_error",
                reason=f"unable to infer target from payload: {payload}",
                mission_id=mission_id,
            )
            return

        with self._lock:
            busy = (
                self._direct_nav is not None
                or self._direct_nav_request is not None
            )

        if busy:
            self._publish_two_wheel_mission_status(
                "failed",
                "busy",
                reason="navigation already active",
                mission_id=mission_id,
            )
            return

        self._active_two_wheel_mission_id = mission_id
        self._active_two_wheel_target = target
        self._completed_mission_latch = None
        # A fresh mission must not inherit a stale path-yield pause.
        self._navigation_paused = False
        self._navigation_pause_started = None

        points = payload.get("points", [])
        dock = payload.get("dock", payload.get("goal"))
        self._active_two_wheel_goal = None
        if target in (0, 1, 2, 3, 4) and isinstance(dock, dict):
            try:
                self._active_two_wheel_goal = (
                    float(dock["x"]),
                    float(dock["y"]),
                    float(dock.get("yaw", -math.pi / 2.0)),
                )
            except (KeyError, TypeError, ValueError):
                self.get_logger().warning(
                    f"[Mission RX] invalid target dock; using default: {dock}"
                )
        self.get_logger().info(
            f"[Mission RX] id={mission_id} target={target} "
            f"points={len(points) if isinstance(points, list) else '?'} "
            f"dock={dock}"
        )

        self._publish_two_wheel_mission_status(
            "accepted", "accepted", target=target
        )

        if not self._queue_navigation(target):
            self._publish_two_wheel_mission_status(
                "failed",
                "queue_rejected",
                reason=f"direct navigation rejected target={target}",
                mission_id=mission_id,
            )
            self._active_two_wheel_mission_id = ""
            self._active_two_wheel_target = None
            self._active_two_wheel_goal = None

    def _on_navigation_trigger(self, msg: Int32):
        self._queue_navigation(int(msg.data))

    def _queue_navigation(self, target):
        if target == 99:
            with self._lock:
                if self._direct_nav is None and self._direct_nav_request is None:
                    return False
                if not self._navigation_paused:
                    self._navigation_paused = True
                    self._navigation_pause_started = time.monotonic()
                self._target_vx = self._target_wz = 0.0
                self._cmd_vx = self._cmd_wz = 0.0
            self.get_logger().warning("direct navigation paused")
            return True
        if target == 98:
            with self._lock:
                if not self._navigation_paused:
                    return False
                paused_for = time.monotonic() - self._navigation_pause_started
                if self._direct_nav is not None:
                    self._direct_nav["stage_start"] += paused_for
                self._navigation_paused = False
                self._navigation_pause_started = None
            self.get_logger().info("direct navigation resumed")
            return True
        if target not in (0, 1, 2, 3, 4):
            self.get_logger().warning(f"unknown navigation command: {target}")
            return False
        with self._lock:
            if self._direct_nav is not None or self._direct_nav_request is not None:
                if target != 4:
                    self.get_logger().warning("direct navigation is already active")
                    return False
                # Kitchen return is also the escape path from a stalled or
                # imperfect table dock.  Let the simulation thread atomically
                # replace the current controller on its next update.
                self._direct_nav_request = 4
                self._target_vx = 0.0
                self._target_wz = 0.0
                self.get_logger().warning(
                    "preempting active navigation for kitchen return"
                )
                return True
            self._direct_nav_request = target
            self._target_vx = 0.0
            self._target_wz = 0.0
        self.get_logger().info(
            f"direct wheel navigation queued: target_id={target} (Nav2 bypassed)"
        )
        return True

    def _publish_static_sensor_tf(self):
        static_tfs = []
        for frame, (ox, oy, oz, oyaw) in self._sensor_offsets.items():
            st = TransformStamped()
            st.header.stamp = TimeMsg(sec=0, nanosec=0)
            st.header.frame_id = BASE_LINK_NAME
            st.child_frame_id = frame
            st.transform.translation.x = ox
            st.transform.translation.y = oy
            st.transform.translation.z = oz
            st.transform.rotation.z = math.sin(oyaw * 0.5)
            st.transform.rotation.w = math.cos(oyaw * 0.5)
            static_tfs.append(st)
        self.static_tf_pub.publish(TFMessage(transforms=static_tfs))

    def _on_cmd_vel(self, msg: Twist):
        if abs(float(msg.linear.y)) > 1e-3 and not self._warned_vy:
            self.get_logger().warning(
                "linear.y ignored — cylindrical wheel collision needs "
                "differential (vx + yaw) drive like serving_robot"
            )
            self._warned_vy = True
        with self._lock:
            if self._direct_nav is not None or self._direct_nav_request is not None:
                return
            self._target_vx = float(msg.linear.x)
            self._target_wz = float(msg.angular.z)

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
            self._target_vx = 0.0
            self._target_wz = 0.0
            self._cmd_vx = 0.0
            self._cmd_wz = 0.0

    def _publish_physx_scan(self, stamp, x: float, y: float, yaw: float):
        now = time.monotonic()
        if now - self._last_scan_time < LIDAR_PERIOD_SEC:
            return
        self._last_scan_time = now

        angle_min = -math.pi
        angle_increment = 2.0 * math.pi / LIDAR_SAMPLES

        try:
            import omni.physx
            query = omni.physx.get_physx_scene_query_interface()
        except Exception:
            query = None

        if query is None:
            return

        origin = (
            x + LIDAR_SENSOR_FORWARD * math.cos(yaw),
            y + LIDAR_SENSOR_FORWARD * math.sin(yaw),
            LIDAR_SENSOR_HEIGHT,
        )

        robot_prefix = (
            str(self.articulation.prim_path)
            if hasattr(self.articulation, "prim_path")
            else self.robot_root
        )
        # Always exclude the whole robot USD root; articulation path alone can
        # miss tray/sensor prims and false-classify them as a peer robot.
        own_roots = [
            p for p in (self.robot_root, robot_prefix) if p
        ]

        def _is_own_hit(body: str) -> bool:
            return any(body.startswith(root) for root in own_roots)

        # The walking person uses a non-contact capsule.  Read its current
        # world position and intersect it analytically with each planar ray,
        # preserving /scan detection without any PhysX contact response.
        person_proxy = self.stage.GetPrimAtPath(PERSON_LIDAR_PROXY_PATH)
        person_xy = None
        if person_proxy.IsValid():
            enabled_attr = person_proxy.GetAttribute(
                PERSON_LIDAR_ENABLED_ATTR
            )
            enabled = (
                bool(enabled_attr.Get()) if enabled_attr.IsValid() else False
            )
            if enabled:
                matrix = UsdGeom.Xformable(
                    person_proxy
                ).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                translation = matrix.ExtractTranslation()
                person_xy = (float(translation[0]), float(translation[1]))

        ranges = []
        obstacle_ranges = []
        person_hits = []
        self._peer_hit_this_scan = False
        self._peer_near_dist = float("inf")
        for index in range(LIDAR_SAMPLES):
            angle = angle_min + index * angle_increment
            world_angle = yaw + angle
            direction = (
                math.cos(world_angle),
                math.sin(world_angle),
                0.0,
            )

            hit = query.raycast_closest(origin, direction, LIDAR_MAX_RANGE)
            rigid_body = (
                str(hit.get("rigidBody", ""))
                if hit and hit.get("hit")
                else ""
            )
            collision_prim = (
                str(hit.get("collision", ""))
                if hit and hit.get("hit")
                else ""
            )

            own_hit = (
                (bool(rigid_body) and _is_own_hit(rigid_body))
                or (bool(collision_prim) and _is_own_hit(collision_prim))
            )
            if hit and hit.get("hit") and not own_hit:
                distance = float(hit["distance"])
            else:
                distance = math.inf

            person_distance = math.inf
            if person_xy is not None:
                offset_x = origin[0] - person_xy[0]
                offset_y = origin[1] - person_xy[1]
                projection = (
                    offset_x * direction[0] + offset_y * direction[1]
                )
                radius_term = (
                    offset_x * offset_x
                    + offset_y * offset_y
                    - PERSON_LIDAR_PROXY_RADIUS
                    * PERSON_LIDAR_PROXY_RADIUS
                )
                discriminant = projection * projection - radius_term
                if discriminant >= 0.0:
                    root = math.sqrt(discriminant)
                    near = -projection - root
                    far = -projection + root
                    candidate = near if near >= LIDAR_MIN_RANGE else far
                    if LIDAR_MIN_RANGE <= candidate <= LIDAR_MAX_RANGE:
                        person_distance = candidate

            virtual_person_hit = person_distance < distance
            if virtual_person_hit:
                distance = person_distance

            if distance < LIDAR_MIN_RANGE:
                distance = math.inf

            ranges.append(distance)

            # Filter for dynamic obstacles: people always stop us.
            # Peer NavRobot*: only the later-order (lower priority) robot stops.
            # The earlier-order robot ignores the peer so it can keep moving /
            # use its lane while the later robot waits.
            hit_path = (rigid_body + " " + collision_prim).lower()
            is_peer_robot = (
                "navrobot" in hit_path
                and not own_hit
            )
            # Do not match bare "hand" — robot arms / trays false-trigger stops.
            is_person = virtual_person_hit or (
                "/World/CorridorObstacleTestPerson" in (rigid_body + collision_prim)
                or "crossingpedestrian" in hit_path
                or "/person" in hit_path
                or "character" in hit_path
                or "human" in hit_path
            )
            stop_for_peer = False
            if is_peer_robot and math.isfinite(distance):
                # NEVER hard-stop for the peer robot. Mutual peer-stop at
                # ~0.3m caused permanent face-to-face pending. Pass with
                # opposite-lane swerve instead (person obstacles still stop).
                self._peer_near_dist = min(self._peer_near_dist, distance)
                self._peer_hit_this_scan = True
            if is_person and math.isfinite(distance):
                obstacle_ranges.append(distance)
                hit_x = origin[0] + direction[0] * distance
                hit_y = origin[1] + direction[1] * distance
                person_hits.append((distance, hit_x, hit_y))
            else:
                obstacle_ranges.append(math.inf)

        if person_hits:
            nearest_hit = min(person_hits, key=lambda item: item[0])
            self._latest_obstacle_position = {
                "distance": float(nearest_hit[0]),
                "x": float(nearest_hit[1]),
                "y": float(nearest_hit[2]),
            }
        else:
            self._latest_obstacle_position = None

        scan = LaserScan()
        if stamp is not None:
            scan.header.stamp = stamp
        scan.header.frame_id = FRONT_LIDAR_FRAME
        scan.angle_min = angle_min
        scan.angle_max = angle_min + (LIDAR_SAMPLES - 1) * angle_increment
        scan.angle_increment = angle_increment
        scan.time_increment = 0.0
        scan.scan_time = LIDAR_PERIOD_SEC
        scan.range_min = LIDAR_MIN_RANGE
        scan.range_max = LIDAR_MAX_RANGE
        scan.ranges = ranges

        self.scan_pub.publish(scan)
        self.nav_scan_pub.publish(scan)
        self.two_wheel_scan_pub.publish(scan)

        self._process_obstacle_ranges(
            obstacle_ranges,
            angle_min,
            angle_increment,
            LIDAR_MIN_RANGE,
            LIDAR_MAX_RANGE,
        )

    def _publish_obstacle_event(self, detected: bool) -> None:
        event = {
            "detected": bool(detected),
            "active": bool(detected),
            "frame_id": "map",
            "source": "physx_lidar",
            "timestamp": time.time(),
        }

        position = self._latest_obstacle_position

        if detected and position is not None:
            event.update(
                {
                    "x": position["x"],
                    "y": position["y"],
                    "distance": position["distance"],
                }
            )
        else:
            event.update(
                {
                    "x": None,
                    "y": None,
                    "distance": None,
                }
            )

        message = String()
        message.data = json.dumps(
            event,
            ensure_ascii=False,
        )

        self.obstacle_pub.publish(message)

        self.get_logger().info(
            f"obstacle event published: {message.data}"
        )

    def _fleet_aisle_x(self) -> float:
        """Per-robot corridor lane so simultaneous trips do not share x=0.

        When a peer is nearby, widen further so head-on pairs peel apart
        instead of freezing face-to-face.
        """
        if (self.robot_name or "") == "robot1":
            base = -FLEET_AISLE_OFFSET_M
            sign = -1.0
        elif (self.robot_name or "") == "robot2":
            base = FLEET_AISLE_OFFSET_M
            sign = 1.0
        else:
            return 0.0
        peer_near = float(getattr(self, "_peer_near_dist", float("inf")))
        if math.isfinite(peer_near) and peer_near < 2.0:
            extra = FLEET_PASS_EXTRA_M * max(0.0, min(1.0, (2.0 - peer_near) / 1.4))
            return base + sign * extra
        return base

    def _on_fleet_intent(self, robot_name: str, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        active = bool(payload.get("active", False))
        self._fleet_active[robot_name] = active
        phase = str(payload.get("phase", "idle") or "idle").lower()
        if active:
            try:
                self._fleet_priorities[robot_name] = float(payload.get("priority", 0.0))
            except (TypeError, ValueError):
                self._fleet_priorities[robot_name] = 0.0
            self._fleet_phases[robot_name] = phase
        else:
            self._fleet_priorities.pop(robot_name, None)
            self._fleet_phases.pop(robot_name, None)
        if robot_name == (self.robot_name or ""):
            self._my_fleet_priority = self._fleet_priorities.get(robot_name)
        # Drop any legacy peer-stop so we never stay pending on an empty aisle.
        peer = "robot2" if (self.robot_name or "") == "robot1" else "robot1"
        if self._obstacle_stop and self._obstacle_stop_from_peer:
            with self._lock:
                if self._obstacle_stop and self._obstacle_stop_from_peer:
                    self._finish_obstacle_stop(float("inf"))

    def _peer_swerve_bias(self, peer_hit: bool, nearest: float) -> float:
        """Legacy hook: constant-sign yaw bias is unsafe (wrong for northbound).

        Lane separation is handled by heading-aware crosstrack in
        `_update_direct_navigation` toward `_fleet_aisle_x()`.
        """
        return 0.0
    def _clear_obstacle_state(self):
        self._obstacle_stop = False
        self._obstacle_stop_started = None
        self._obstacle_stop_from_peer = False
        self._clearance_start = None
        self._obstacle_scale = 1.0

    def _process_obstacle_ranges(
        self,
        ranges,
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
    ):
        with self._lock:
            mission = self._direct_nav
            if mission is None:
                self._clear_obstacle_state()
                return

            if mission["mode"] == "legacy_table":
                stage = mission.get("stage")
                if stage not in ("move_to_pre_dock", "final_approach"):
                    self._clear_obstacle_state()
                    return
            else:
                stages = mission.get("stages", [])
                index = mission.get("index", 0)
                if index >= len(stages):
                    self._clear_obstacle_state()
                    return
                kind = stages[index].get("kind")
                if kind not in ("axis_x", "axis_y"):
                    # Pivot must stay pure yaw — drop sticky peer swerve from
                    # the previous translate stage so it cannot cancel wz.
                    self._clear_obstacle_state()
                    self._peer_swerve_wz = 0.0
                    return

            # Robot-local (not just forward-cone) safety check: classify each
            # finite hit by its position relative to the robot center, not
            # just its bearing.  OBSTACLE_STOP_*/OBSTACLE_SLOWDOWN_* mirror
            # nav2_params.yaml's collision_monitor stop/slowdown polygons, so
            # a person approaching from the side or rear is caught the same
            # way one approaching head-on is.
            stop_hits = []
            slowdown_hits = []
            for idx, distance in enumerate(ranges):
                if not math.isfinite(distance):
                    continue
                if not (range_min <= distance <= range_max):
                    continue
                angle = angle_min + idx * angle_increment
                lx = LIDAR_SENSOR_FORWARD + distance * math.cos(angle)
                ly = distance * math.sin(angle)
                if (
                    OBSTACLE_SLOWDOWN_BACK <= lx <= OBSTACLE_SLOWDOWN_FRONT
                    and abs(ly) <= OBSTACLE_SLOWDOWN_HALF_WIDTH
                ):
                    slowdown_hits.append(distance)
                    if (
                        OBSTACLE_STOP_BACK <= lx <= OBSTACLE_STOP_FRONT
                        and abs(ly) <= OBSTACLE_STOP_HALF_WIDTH
                    ):
                        stop_hits.append(distance)

            nearest = min(slowdown_hits) if slowdown_hits else float("inf")
            close_points = stop_hits
            peer_hit = bool(getattr(self, "_peer_hit_this_scan", False))
            peer_near = float(getattr(self, "_peer_near_dist", float("inf")))
            # Swerve for nearby peers; never leave a sticky peer hard-stop.
            if self._obstacle_stop and self._obstacle_stop_from_peer:
                self._finish_obstacle_stop(peer_near)
            swerve_range = peer_near if peer_near <= 2.0 else float("inf")
            self._peer_swerve_wz = self._peer_swerve_bias(
                peer_hit or math.isfinite(swerve_range),
                nearest if (peer_hit and math.isfinite(nearest)) else swerve_range,
            )

            now = time.monotonic()

            if not self._obstacle_stop:
                if len(close_points) >= 3:
                    # Person (or non-peer) polygon stop only.
                    self._start_obstacle_stop(nearest, from_peer=False)
                elif slowdown_hits:
                    self._obstacle_scale = OBSTACLE_SLOWDOWN_RATIO
                else:
                    self._obstacle_scale = 1.0
            else:
                self._obstacle_scale = 0.0
                if not slowdown_hits:
                    if self._clearance_start is None:
                        self._clearance_start = now
                    elif now - self._clearance_start >= 0.5:
                        self._finish_obstacle_stop(nearest)
                else:
                    self._clearance_start = None

            is_decel_or_stop = bool(slowdown_hits) or self._obstacle_stop
            if is_decel_or_stop != getattr(self, "_last_decel_event_state", False):
                self._last_decel_event_state = is_decel_or_stop
                self._publish_obstacle_event(is_decel_or_stop)

            if is_decel_or_stop:
                if now - getattr(self, "_last_obstacle_log", 0.0) >= 0.5:
                    self._last_obstacle_log = now
                    self.get_logger().info(
                        f"obstacle distance={nearest:.2f}m scale={self._obstacle_scale:.2f} "
                        f"stop={self._obstacle_stop} peer={self._obstacle_stop_from_peer}"
                    )

    def _start_obstacle_stop(self, distance: float, from_peer: bool = False):
        if self._obstacle_stop:
            return
        self._obstacle_stop = True
        self._obstacle_stop_started = time.monotonic()
        self._obstacle_stop_from_peer = bool(from_peer)
        self._clearance_start = None
        self._obstacle_scale = 0.0
        self._target_vx = 0.0
        self._target_wz = 0.0
        self.get_logger().warning(
            f"주변 장애물 정지영역 진입: distance={distance:.2f}m, 주행 정지"
            + (" (peer yield)" if from_peer else "")
        )

    def _finish_obstacle_stop(self, distance: float):
        if not self._obstacle_stop:
            return
        paused_for = time.monotonic() - (self._obstacle_stop_started or time.monotonic())
        if self._direct_nav is not None and "stage_start" in self._direct_nav:
            self._direct_nav["stage_start"] += paused_for
        was_peer = self._obstacle_stop_from_peer
        self._obstacle_stop = False
        self._obstacle_stop_started = None
        self._obstacle_stop_from_peer = False
        self._clearance_start = None
        self._obstacle_scale = 1.0
        self.get_logger().info(
            f"전방 장애물 해제: distance={distance:.2f}m, 주행 재개"
            + (" (peer clear)" if was_peer else "")
        )

    def _apply_pending_teleport(self):
        with self._lock:
            pending = self._pending_teleport
            self._pending_teleport = None
        if pending is None:
            return
        x, y, z, yaw = pending
        orientation = np.asarray(
            [math.cos(yaw * 0.5), 0.0, 0.0, math.sin(yaw * 0.5)], dtype=float
        )
        self.articulation.set_world_pose(
            position=np.asarray([x, y, z], dtype=float),
            orientation=orientation,
        )
        n_dof = len(self.dof_names)
        self.articulation.set_joint_velocities(np.zeros(n_dof, dtype=float))
        # Zero wheel command so physics does not immediately push away.
        wheels_zero = np.zeros(len(self.wheel_indices), dtype=float)
        self.articulation.apply_action(
            ArticulationAction(
                joint_velocities=wheels_zero,
                joint_indices=self.wheel_indices,
            )
        )
        print(
            f"[nav_robot] teleported to ({x:.2f},{y:.2f},{z:.3f}) yaw={yaw:.2f}",
            flush=True,
        )

    @staticmethod
    def _slew(current, target, acceleration, deceleration, dt):
        limit = acceleration if abs(target) > abs(current) else deceleration
        delta = target - current
        max_delta = limit * dt
        if abs(delta) <= max_delta:
            return target
        return current + math.copysign(max_delta, delta)

    def _differential_ik(self, vx, wz):
        # Left/right differential drive used by the stable two-wheel base.
        turn = DIFFERENTIAL_HALF_TRACK * wz
        wheels = np.asarray(
            [
                (vx - turn) / WHEEL_RADIUS,
                (vx + turn) / WHEEL_RADIUS,
            ],
            dtype=float,
        )
        return np.clip(wheels, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)

    @staticmethod
    def _angle_error(target, actual):
        return math.atan2(math.sin(target - actual), math.cos(target - actual))

    def _direct_wheel_ik(self, vx, wz):
        """Wheel command for the stable two-wheel serving base."""
        turn = DIRECT_CONTROL_HALF_TRACK * wz
        wheels = np.asarray(
            [
                (vx - turn) / WHEEL_RADIUS,
                (vx + turn) / WHEEL_RADIUS,
            ],
            dtype=float,
        )
        # Match the proven controller's limit exactly.
        return np.clip(wheels, -8.0, 8.0)

    def _start_direct_navigation(self, target, x, y, yaw):
        kitchen_dock = self._active_two_wheel_goal or (
            0.0,
            5.25,
            -math.pi / 2.0,
        )
        # The manager deliberately sends target=4 again at the beginning of a
        # new order even after the previous return already reported kitchen.
        # Treat that verification request as an arrival acknowledgement.  A
        # second route here could pivot the base despite no position change.
        if (
            target == 4
            and self._navigation_location == 4
            and math.hypot(x - kitchen_dock[0], y - kitchen_dock[1]) <= 0.10
        ):
            self._direct_nav = None
            self._cmd_vx = self._cmd_wz = 0.0
            self._target_vx = self._target_wz = 0.0
            self.navigation_location_pub.publish(Int32(data=4))
            if self._active_two_wheel_mission_id:
                mission_id = self._active_two_wheel_mission_id
                self._publish_two_wheel_mission_status(
                    "completed", "completed", target=4
                )
                self.get_logger().info(
                    f"[Mission TX] completed id={mission_id} target=4 "
                    "(already at kitchen)"
                )
                self._active_two_wheel_mission_id = ""
                self._active_two_wheel_target = None
                self._active_two_wheel_goal = None
            self.get_logger().info(
                "already at kitchen; acknowledged redundant target=4 "
                "without moving"
            )
            return
        aisle_x = self._fleet_aisle_x()
        stages = (
            build_kitchen_route(
                x, y, kitchen_dock=kitchen_dock, aisle_x=aisle_x
            )
            if target == 4
            else build_table_route(
                target,
                x,
                y,
                table_dock=self._active_two_wheel_goal,
                aisle_x=aisle_x,
            )
        )
        self.get_logger().info(
            f"route aisle_x={aisle_x:.2f} robot={self.robot_name or 'default'} "
            f"target={target}"
        )

        # Normalize only the first kitchen translation after park-out.
        # Park-out already rotates the chassis toward the aisle.  Converting
        # every negative-speed stage changed later, already-proven corner
        # behavior.  Restrict the conversion to the first kitchen axis stage
        # and its immediately preceding pivot; leave every later corner and
        # translation exactly as build_kitchen_route() authored them.
        stages = [dict(stage) for stage in stages]
        if target == 4:
            first_axis_index = next(
                (
                    index
                    for index, stage in enumerate(stages)
                    if stage.get("kind") in ("axis_x", "axis_y")
                ),
                None,
            )
            if first_axis_index is not None:
                first_axis = stages[first_axis_index]
                if float(first_axis.get("speed", 0.0)) < 0.0:
                    forward_yaw = self._angle_error(
                        float(first_axis["yaw"]) + math.pi, 0.0
                    )
                    first_axis["speed"] = abs(float(first_axis["speed"]))
                    first_axis["yaw"] = forward_yaw

                    if (
                        first_axis_index > 0
                        and stages[first_axis_index - 1].get("kind") == "pivot"
                    ):
                        stages[first_axis_index - 1]["yaw"] = forward_yaw

                    self.get_logger().info(
                        "normalized first kitchen segment for forward travel: "
                        f"axis_index={first_axis_index} "
                        f"yaw={math.degrees(forward_yaw):.1f}deg"
                    )

        self._direct_nav = {
            "mode": "axis_route",
            "target": target,
            "stages": stages,
            "index": 0,
            "stage_start": time.monotonic(),
            "last_log": 0.0,
        }
        self._obstacle_stop = False
        self._obstacle_stop_started = None
        self._obstacle_stop_from_peer = False
        self._clearance_start = None
        self._cmd_vx = 0.0
        self._cmd_wz = 0.0
        self.get_logger().info(
            f"direct route started target={target} pose=({x:.2f},{y:.2f},"
            f"{math.degrees(yaw):.1f}deg)"
        )

    def _finish_park_out(self, success, reason=""):
        mission = self._direct_nav
        if mission is None or mission.get("mode") != "park_out":
            return
        mission_id = self._active_two_wheel_mission_id
        self._direct_nav = None
        self._cmd_vx = self._cmd_wz = 0.0
        self._target_vx = self._target_wz = 0.0
        self._obstacle_stop = False
        self._obstacle_stop_started = None
        self._obstacle_stop_from_peer = False
        self._clearance_start = None

        if mission_id:
            if success:
                for _ in range(5):
                    self._publish_two_wheel_mission_status(
                        "completed",
                        "park_out_aligned",
                        mission_id=mission_id,
                    )
                self.get_logger().info(
                    f"[Mission TX] park-out completed id={mission_id}"
                )
            else:
                for _ in range(5):
                    self._publish_two_wheel_mission_status(
                        "failed",
                        "execution_failed",
                        reason=reason,
                        mission_id=mission_id,
                    )
                self.get_logger().error(
                    f"[Mission TX] park-out failed id={mission_id}: {reason}"
                )
        self._active_two_wheel_mission_id = ""
        self._active_two_wheel_target = None
        self._active_two_wheel_goal = None

    def _finish_direct_navigation(self, success, reason=""):
        mission = self._direct_nav
        if mission is None:
            return
        target = mission["target"]
        self._direct_nav = None
        self._obstacle_stop = False
        self._obstacle_stop_started = None
        self._obstacle_stop_from_peer = False
        self._clearance_start = None
        self._cmd_vx = self._cmd_wz = 0.0
        self._target_vx = self._target_wz = 0.0
        if success:
            self._navigation_location = target
            self.navigation_location_pub.publish(Int32(data=target))
            if self._active_two_wheel_mission_id:
                mission_id = self._active_two_wheel_mission_id
                # Keep completed status warm so ROS wait loops cannot miss the
                # first burst and remain stuck on state=accepted.
                self._completed_mission_latch = {
                    "mission_id": mission_id,
                    "target": target,
                    "until": time.monotonic() + 12.0,
                    "last_pub": 0.0,
                }
                for _ in range(5):
                    self._publish_two_wheel_mission_status(
                        "completed", "completed", target=target,
                        mission_id=mission_id,
                    )
                self.get_logger().info(
                    f"[Mission TX] completed id={mission_id} target={target}"
                )
                self._active_two_wheel_mission_id = ""
                self._active_two_wheel_target = None
            self.get_logger().info(f"direct navigation complete target={target}")
        else:
            # String mission_status only — ROS retries attempts. Never publish
            # navigation/status Int32 from Isaac (manager sticky FAILED).
            if self._active_two_wheel_mission_id:
                mission_id = self._active_two_wheel_mission_id
                for _ in range(5):
                    self._publish_two_wheel_mission_status(
                        "failed",
                        "execution_failed",
                        reason=reason,
                        target=target,
                        mission_id=mission_id,
                    )
                self.get_logger().error(
                    f"[Mission TX] failed id={mission_id} target={target}: "
                    f"{reason}"
                )
                self._active_two_wheel_mission_id = ""
                self._active_two_wheel_target = None
            self.get_logger().error(
                f"direct navigation failed target={target}: {reason}"
            )
        self._active_two_wheel_goal = None

    def _update_direct_navigation(self, x, y, yaw):
        mission = self._direct_nav
        if mission is None:
            return None
        if self._navigation_paused or self._obstacle_stop:
            # Legacy peer-stop must never freeze motion - clear and continue.
            if self._obstacle_stop and self._obstacle_stop_from_peer:
                self._finish_obstacle_stop(float("inf"))
            # Table park-out must reverse even if the dock furniture / peer /
            # person polygon trips a stop. Freezing here left phase=
            # park_out_backoff until ROS timed out -> manager sticky FAILED ->
            # HMI cancelled the other robot's order.
            if mission.get("mode") == "park_out":
                if self._obstacle_stop:
                    self._finish_obstacle_stop(float("inf"))
                if self._navigation_paused:
                    self._navigation_paused = False
                    self._navigation_pause_started = None
            elif self._navigation_paused or self._obstacle_stop:
                if (
                    mission.get("mode") not in ("legacy_table",)
                    and not self._obstacle_stop
                ):
                    stages = mission.get("stages") or []
                    index = int(mission.get("index", 0))
                    if index < len(stages) and stages[index].get("kind") == "pivot":
                        error = self._angle_error(stages[index]["yaw"], yaw)
                        if abs(error) < math.radians(8.0):
                            mission["index"] = index + 1
                            mission["stage_start"] = time.monotonic()
                            if mission["index"] >= len(stages):
                                self._finish_direct_navigation(True)
                                return 0.0, 0.0
                            return 0.0, 0.0
                        wz = float(np.clip(1.8 * error, -0.65, 0.65))
                        if abs(wz) < 0.22:
                            wz = math.copysign(0.22, error)
                        return 0.0, wz
                if self._navigation_paused or self._obstacle_stop:
                    return 0.0, 0.0

        if mission["mode"] == "park_out":
            now = time.monotonic()
            phase = mission.get("phase", "init")

            if phase == "init":
                mission["start_x"] = float(x)
                mission["start_y"] = float(y)
                mission["start_yaw"] = float(yaw)
                mission["target_yaw"] = self._angle_error(
                    float(yaw) + math.pi, 0.0
                )
                mission["phase"] = "backoff"
                mission["stage_start"] = now
                mission["last_log"] = 0.0
                self.get_logger().info(
                    "park-out started: reverse "
                    f"{mission['distance']:.2f}m, then align "
                    f"{math.degrees(mission['target_yaw']):.1f}deg"
                )
                return 0.0, 0.0

            if phase == "backoff":
                start_yaw = mission["start_yaw"]
                dx = x - mission["start_x"]
                dy = y - mission["start_y"]
                progress = -(
                    dx * math.cos(start_yaw)
                    + dy * math.sin(start_yaw)
                )
                remaining = mission["distance"] - progress
                yaw_error = self._angle_error(start_yaw, yaw)
                done = remaining <= 0.025
                if not done:
                    if progress >= 0.22 and now - mission["stage_start"] > 5.0:
                        done = True
                    elif progress >= 0.12 and now - mission["stage_start"] > 10.0:
                        done = True
                vx = 0.0 if done else -min(
                    mission["speed"],
                    max(0.06, remaining * 0.8),
                )
                wz = 0.0 if done else float(
                    np.clip(1.6 * yaw_error, -0.25, 0.25)
                )
                detail = (
                    f"progress={progress:.3f}/{mission['distance']:.3f}m "
                    f"yaw_error={math.degrees(yaw_error):.1f}deg"
                )
                timeout = max(
                    14.0,
                    mission["distance"] / max(mission["speed"], 0.05) + 8.0,
                )
                if done:
                    mission["phase"] = "align_opposite"
                    mission["stage_start"] = now
                    mission["last_log"] = 0.0
                    self.get_logger().info(
                        "park-out reverse complete; aligning opposite direction"
                    )
                    return 0.0, 0.0

            else:
                yaw_error = self._angle_error(mission["target_yaw"], yaw)
                done = abs(yaw_error) < math.radians(8.0)
                if not done:
                    if (
                        now - mission["stage_start"] > 3.0
                        and abs(yaw_error) < math.radians(20.0)
                    ):
                        done = True
                    elif (
                        now - mission["stage_start"] > 8.0
                        and abs(yaw_error) < math.radians(35.0)
                    ):
                        done = True
                vx = 0.0
                wz = 0.0
                if not done:
                    wz = float(np.clip(2.4 * yaw_error, -0.90, 0.90))
                    if abs(wz) < 0.30:
                        wz = math.copysign(0.30, yaw_error)
                detail = (
                    f"target_yaw={math.degrees(mission['target_yaw']):.1f}deg "
                    f"yaw_error={math.degrees(yaw_error):.1f}deg"
                )
                timeout = 20.0
                if done:
                    self._finish_park_out(True)
                    return 0.0, 0.0

            if now - mission["last_log"] >= 0.5:
                mission["last_log"] = now
                self.get_logger().info(
                    f"park-out phase={mission['phase']} "
                    f"pose=({x:.2f},{y:.2f},{math.degrees(yaw):.1f}deg) "
                    f"{detail}"
                )

            if now - mission["stage_start"] > timeout:
                if phase == "backoff":
                    start_yaw = mission["start_yaw"]
                    dx = x - mission["start_x"]
                    dy = y - mission["start_y"]
                    progress = -(
                        dx * math.cos(start_yaw)
                        + dy * math.sin(start_yaw)
                    )
                    if progress >= 0.15:
                        mission["phase"] = "align_opposite"
                        mission["stage_start"] = now
                        mission["last_log"] = 0.0
                        self.get_logger().warning(
                            "park-out reverse soft-complete after timeout; "
                            f"{detail}"
                        )
                        return 0.0, 0.0
                if phase == "align_opposite":
                    self._finish_park_out(
                        True, f"align soft-complete; {detail}"
                    )
                    return 0.0, 0.0
                self._finish_park_out(
                    False, f"{mission['phase']} timeout; {detail}"
                )
                return 0.0, 0.0

            return vx, wz

        if mission["mode"] == "legacy_table":
            return self._update_legacy_table_navigation(mission, x, y, yaw)

        stage = mission["stages"][mission["index"]]
        elapsed = time.monotonic() - mission["stage_start"]
        kind = stage["kind"]
        vx = wz = 0.0
        done = False

        if kind == "pivot":
            error = self._angle_error(stage["yaw"], yaw)
            # Skip near-aligned pivots immediately — corridor starts often sit
            # at ~10deg and used to wedge forever under peer lat bias.
            done = abs(error) < math.radians(8.0)
            if not done:
                if elapsed > 2.0 and abs(error) < math.radians(15.0):
                    done = True
                elif elapsed > 5.0 and abs(error) < math.radians(25.0):
                    done = True
            if not done:
                wz = float(np.clip(2.6 * error, -0.95, 0.95))
                if abs(wz) < 0.40:
                    wz = math.copysign(0.40, error)
            timeout = 18.0
            detail = f"yaw_error={math.degrees(error):.1f}deg"
        else:
            axis = x if kind == "axis_x" else y
            # Live retarget corridor x to the peer-aware lane so a head-on
            # pair peels apart even if the stage was planned on a narrow aisle.
            if kind == "axis_x" and abs(float(stage.get("value", 0.0))) <= 0.85:
                stage["value"] = self._fleet_aisle_x()
            error = stage["value"] - axis
            done = abs(error) <= 0.05
            desired_yaw = stage["yaw"]
            yaw_error = self._angle_error(desired_yaw, yaw)
            if not done:
                # If heading is blown out, stop translating and re-align.
                # Otherwise peer lane bias drives the nose into a table bay
                # (kitchen return → east tables) while still rolling forward.
                if abs(yaw_error) > math.radians(30.0):
                    vx = 0.0
                    wz = float(np.clip(2.2 * yaw_error, -0.80, 0.80))
                    if abs(wz) < 0.30:
                        wz = math.copysign(0.30, yaw_error)
                else:
                    requested = min(
                        abs(stage["speed"]), max(0.045, abs(error) * 0.8)
                    )
                    vx = math.copysign(requested, stage["speed"])
                    wz = float(np.clip(1.8 * yaw_error, -0.45, 0.45))
            timeout = 90.0
            detail = f"axis_error={error:.3f}m yaw_error={math.degrees(yaw_error):.1f}deg"

        now = time.monotonic()
        if now - mission["last_log"] >= 1.0:
            mission["last_log"] = now
            self.get_logger().info(
                f"direct stage={mission['index']} {kind} pose=({x:.2f},{y:.2f},"
                f"{math.degrees(yaw):.1f}deg) {detail}"
            )
        if elapsed > timeout:
            self._finish_direct_navigation(False, f"{kind} timeout; {detail}")
            return (0.0, 0.0)
        if done:
            mission["index"] += 1
            if mission["index"] >= len(mission["stages"]):
                if mission["mode"] == "table_transfer":
                    target = mission["target"]
                    goal = mission["goal"]
                    mission.clear()
                    mission.update(
                        mode="legacy_table",
                        target=target,
                        goal=goal,
                        stage="move_to_pre_dock",
                        path_aligned=False,
                        stage_start=now,
                        last_log=0.0,
                        settle_count=0,
                        recovery_count=0,
                    )
                    self.get_logger().info(
                        "table transfer reached centre aisle; starting "
                        "original pre-dock controller"
                    )
                    return (0.0, 0.0)
                self._finish_direct_navigation(True)
                return (0.0, 0.0)
            mission["stage_start"] = now
            mission["last_log"] = 0.0
            next_kind = mission["stages"][mission["index"]]["kind"]
            self.get_logger().info(f"direct stage complete; next={next_kind}")
            return (0.0, 0.0)
        # Lane hold only while translating and roughly aligned. Constant-sign
        # peer swerve + raw (aisle-x) on northbound axis_y turned robot1 CW
        # into +X and parked it in the east table bay (table 4).
        if kind == "axis_y" and abs(vx) > 1e-3:
            desired_yaw = float(stage["yaw"])
            yaw_error = self._angle_error(desired_yaw, yaw)
            if abs(yaw_error) < math.radians(25.0):
                aisle = self._fleet_aisle_x()
                lat_err = aisle - x
                # Heading-aware crosstrack: northbound (+Y) needs
                # wz = -k*(aisle-x); southbound the opposite.
                lane_wz = -1.15 * lat_err * math.sin(desired_yaw)
                peer_near = float(
                    getattr(self, "_peer_near_dist", float("inf"))
                )
                limit = 0.28
                if math.isfinite(peer_near) and peer_near < 2.0:
                    limit = 0.35
                    slow = 0.50 if peer_near < 0.8 else 0.80
                    vx = math.copysign(max(0.12, abs(vx) * slow), vx)
                lane_wz = float(np.clip(lane_wz, -limit, limit))
                wz = float(np.clip(wz + lane_wz, -0.55, 0.55))
        return (vx * self._obstacle_scale, wz)

    def _update_legacy_table_navigation(self, mission, x, y, yaw):
        """Original mobile_manipulator_demo TableNavigationServer controller."""
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
            if distance <= 0.08:
                mission["stage"] = "align_at_pre_dock"
                mission["path_aligned"] = False
                mission["stage_start"] = time.monotonic()
                phase, vx, wz = "pre_dock_reached", 0.0, 0.0
            detail = (
                f"pre=({pre_x:.2f},{pre_y:.2f}) distance={distance:.3f}m "
                f"heading_error={math.degrees(heading_error):.1f}deg"
            )
            timeout = 120.0

        elif stage == "align_at_pre_dock":
            distance = math.hypot(pre_x - x, pre_y - y)
            yaw_error = self._angle_error(goal_yaw, yaw)
            phase = "align_at_pre_dock"
            if abs(yaw_error) > math.radians(2.0):
                wz = float(np.clip(1.8 * yaw_error, -0.65, 0.65))
                # Commands below this cannot overcome the skid-steer tire's
                # static friction, which left the robot parked around 2 deg.
                if abs(wz) < 0.18:
                    wz = math.copysign(0.18, yaw_error)
            else:
                mission["stage"] = "final_approach"
                mission["stage_start"] = time.monotonic()
                phase, wz = "start_final_approach", 0.0
            detail = f"distance={distance:.3f}m yaw_error={math.degrees(yaw_error):.1f}deg"
            timeout = 30.0

        elif stage == "recovery_backout":
            dx, dy = goal_x - x, goal_y - y
            forward_error = math.cos(goal_yaw) * dx + math.sin(goal_yaw) * dy
            yaw_error = self._angle_error(goal_yaw, yaw)
            if abs(yaw_error) > math.radians(2.0):
                # Never reverse while the chassis is still pointing along the
                # diagonal re-entry angle; doing so drove it away sideways.
                phase = "recovery_align_before_backout"
                wz = float(np.clip(1.8 * yaw_error, -0.45, 0.45))
                if abs(wz) < 0.18:
                    wz = math.copysign(0.18, yaw_error)
            elif forward_error < 0.55:
                phase = "recovery_backout"
                vx = -0.08
                wz = float(np.clip(1.8 * yaw_error, -0.20, 0.20))
            else:
                mission["stage"] = "recovery_align"
                mission["stage_start"] = time.monotonic()
                phase, vx, wz = "recovery_backout_complete", 0.0, 0.0
            distance = math.hypot(dx, dy)
            detail = (
                f"backout_forward={forward_error:.3f}m "
                f"yaw_error={math.degrees(yaw_error):.1f}deg"
            )
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
                phase, wz = "recovery_reapproach_start", 0.0
            detail = (
                f"distance={distance:.3f}m approach_yaw="
                f"{math.degrees(approach_yaw):.1f}deg heading_error="
                f"{math.degrees(heading_error):.1f}deg"
            )
            timeout = 15.0

        elif stage == "recovery_reapproach":
            dx, dy = goal_x - x, goal_y - y
            distance = math.hypot(dx, dy)
            # Keep the entry line fixed.  Recomputing atan2 close to the goal
            # made the target heading singular and curled the robot around it.
            approach_yaw = mission["recovery_approach_yaw"]
            heading_error = self._angle_error(approach_yaw, yaw)
            phase = "recovery_reapproach"
            if distance > 0.08:
                if abs(heading_error) < math.radians(12.0):
                    vx = min(0.08, max(0.018, 0.45 * distance))
                wz = float(np.clip(1.8 * heading_error, -0.25, 0.25))
            else:
                mission["stage"] = "recovery_final_align"
                mission["stage_start"] = time.monotonic()
                mission["settle_count"] = 0
                phase, vx, wz = "recovery_position_reached", 0.0, 0.0
            detail = (
                f"distance={distance:.3f}m heading_error="
                f"{math.degrees(heading_error):.1f}deg"
            )
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
                mission["settle_count"] = 0
                phase, wz = "recovery_final_align_complete", 0.0
            detail = (
                f"distance={distance:.3f}m yaw_error="
                f"{math.degrees(yaw_error):.1f}deg"
            )
            timeout = 15.0

        else:
            dx, dy = goal_x - x, goal_y - y
            distance = math.hypot(dx, dy)
            yaw_error = self._angle_error(goal_yaw, yaw)
            forward_error = math.cos(goal_yaw) * dx + math.sin(goal_yaw) * dy
            lateral_error = -math.sin(goal_yaw) * dx + math.cos(goal_yaw) * dy
            position_ok = distance <= DOCK_XY_TOLERANCE_M
            yaw_ok = abs(yaw_error) <= DOCK_YAW_TOLERANCE_RAD
            if (
                abs(lateral_error) > 0.05
                and abs(forward_error) < 0.10
            ):
                mission["recovery_count"] = mission.get("recovery_count", 0) + 1
                if mission["recovery_count"] > 3:
                    self._finish_direct_navigation(
                        False,
                        f"dock recovery exhausted; lateral={lateral_error:.3f}m",
                    )
                    return (0.0, 0.0)
                mission["stage"] = "recovery_backout"
                mission["stage_start"] = time.monotonic()
                mission["settle_count"] = 0
                phase, vx, wz = "start_lateral_recovery", 0.0, 0.0
                self.get_logger().warning(
                    f"dock lateral error={lateral_error:.3f}m; starting "
                    f"re-entry {mission['recovery_count']}/3"
                )
            elif not position_ok:
                phase = "final_forward_approach"
                if abs(yaw_error) <= math.radians(8.0):
                    vx = float(np.clip(0.45 * forward_error, -0.04, 0.08))
                    if abs(vx) < 0.015 and abs(forward_error) > 0.004:
                        vx = math.copysign(0.015, forward_error)
                wz = float(
                    # Positive lateral error is to the goal-frame left and
                    # therefore requires positive yaw.  The copied controller
                    # used the opposite sign for its former frame convention.
                    np.clip(1.8 * yaw_error + 1.4 * lateral_error, -0.20, 0.20)
                )
                mission["settle_count"] = 0
            elif not yaw_ok:
                phase = "fine_align_at_table"
                wz = float(np.clip(1.2 * yaw_error, -0.15, 0.15))
                if abs(wz) < 0.12:
                    wz = math.copysign(0.12, yaw_error)
                mission["settle_count"] = 0
            else:
                phase = "settle_at_table"
                mission["settle_count"] = mission.get("settle_count", 0) + 1
                if mission["settle_count"] >= 30:
                    self.get_logger().info(
                        f"arrived table={mission['target']} pose=({x:.3f},{y:.3f},"
                        f"{math.degrees(yaw):.1f}deg)"
                    )
                    self._finish_direct_navigation(True)
                    return (0.0, 0.0)
            detail = (
                f"goal=({goal_x:.2f},{goal_y:.2f}) distance={distance:.3f}m "
                f"forward={forward_error:.3f}m lateral={lateral_error:.3f}m yaw_error="
                f"{math.degrees(yaw_error):.1f}deg"
            )
            timeout = 60.0

        now = time.monotonic()
        if now - mission["last_log"] >= 1.0:
            mission["last_log"] = now
            self.get_logger().info(
                f"direct phase={phase} pose=({x:.2f},{y:.2f},"
                f"{math.degrees(yaw):.1f}deg) {detail}"
            )
        if now - mission["stage_start"] > timeout:
            self._finish_direct_navigation(False, f"{stage} timeout; {detail}")
            return (0.0, 0.0)
        return (vx, wz)

    def tick(self, _sim_time_sec: float = 0.0):
        self._apply_pending_teleport()

        now_monotonic = time.monotonic()
        if now_monotonic - self._last_navigation_heartbeat >= 1.0:
            self._last_navigation_heartbeat = now_monotonic
            # Location only — never publish navigation/status from Isaac.
            self.navigation_location_pub.publish(
                Int32(data=self._navigation_location)
            )

        position, orientation = self.articulation.get_world_pose()
        x = float(position[0])
        y = float(position[1])
        yaw = quaternion_to_yaw(orientation)

        with self._lock:
            nav_request = self._direct_nav_request
            self._direct_nav_request = None
        if nav_request is not None:
            self._start_direct_navigation(nav_request, x, y, yaw)

        with self._lock:
            pending_spawn = self._pending_food_spawn
            self._pending_food_spawn = None

        if pending_spawn is not None and self.stage is not None:
            self.get_logger().info(
                f"Spawning requested payload command={pending_spawn} in Isaac Sim..."
            )
            try:
                requested_spawn_command = int(pending_spawn)
                # New combined encoding: hundreds digit is plate count and the
                # lower two digits retain pizza/drink/cutlery.  Accept the old
                # plate-only 81..84 values for compatibility with an already
                # running manager during rollout.
                if 81 <= requested_spawn_command <= 84:
                    plate_count = requested_spawn_command - 80
                    spawn_command = 0
                else:
                    plate_count, spawn_command = divmod(
                        requested_spawn_command, 100
                    )
                if plate_count < 0 or plate_count > 4:
                    raise ValueError(
                        "unsupported plate count in food spawn command="
                        f"{requested_spawn_command}"
                    )
                plate_rack_requested = plate_count > 0
                if spawn_command >= 40:
                    cutlery_requested = True
                    remainder = spawn_command - 40
                elif spawn_command >= 20:
                    cutlery_requested = True
                    remainder = spawn_command - 20
                else:
                    cutlery_requested = False
                    remainder = spawn_command

                pizza_type = remainder // 10
                pizza_requested = pizza_type > 0
                drink_count = remainder % 10

                if drink_count < 0 or drink_count > 4:
                    raise ValueError(
                        f"unsupported food spawn command={spawn_command}"
                    )
                # Preserve the completed trip as visual-only USD geometry.
                # Live payload physics remains at stable paths and is reused;
                # deleting or moving it invalidates Isaac's tensor views.
                payload_paths = tuple(
                    f"{self.payload_root}/{name}"
                    for name in (
                        "ServingDish",
                        "PizzaBoardBail",
                        "PizzaBoardBailHinge",
                        "PizzaBoardGripBearing",
                        "PizzaBoardGripBlock",
                        "ServingDrinks",
                        "ServingCutlery",
                        "ServingPlateRack",
                    )
                )
                reused_payload_paths = set()
                if pizza_requested:
                    reused_payload_paths.update(payload_paths[:5])
                if drink_count:
                    reused_payload_paths.add(
                        f"{self.payload_root}/ServingDrinks"
                    )
                if cutlery_requested:
                    reused_payload_paths.add(
                        f"{self.payload_root}/ServingCutlery"
                    )
                if plate_rack_requested:
                    reused_payload_paths.add(
                        f"{self.payload_root}/ServingPlateRack"
                    )
                existing_payloads = [
                    path
                    for path in payload_paths
                    if self.stage.GetPrimAtPath(path).IsValid()
                ]
                archived = self._archive_delivered_payloads(existing_payloads)
                if existing_payloads and not archived:
                    self.get_logger().warning(
                        "previous payload was not archived; reusing its stable prims"
                    )
                # Authoring helpers use AddXformOp.  On a reused prim the old
                # op order must be cleared first, otherwise USD rejects a
                # duplicate xformOp:translate.  This changes no physics prim
                # topology and therefore keeps tensor views valid.
                for payload_path in existing_payloads:
                    if payload_path not in reused_payload_paths:
                        continue
                    payload_prim = self.stage.GetPrimAtPath(payload_path)
                    if not payload_prim.IsValid():
                        continue
                    for prim in Usd.PrimRange(payload_prim):
                        xformable = UsdGeom.Xformable(prim)
                        if xformable:
                            xformable.ClearXformOpOrder()
                self._spawned_serving_tasks = {}
                if drink_count and 'spawn_soda_cans' in globals():
                    spawn_soda_cans(
                        self.stage,
                        count=drink_count,
                        payload_root=self.payload_root,
                        robot_root=self.robot_root,
                    )
                if cutlery_requested and 'spawn_cutlery_box' in globals():
                    spawn_cutlery_box(
                        self.stage,
                        payload_root=self.payload_root,
                        robot_root=self.robot_root,
                    )
                if plate_rack_requested and 'spawn_plate_rack' in globals():
                    spawn_plate_rack(
                        self.stage,
                        plate_count=plate_count,
                        payload_root=self.payload_root,
                        robot_root=self.robot_root,
                    )
                    self._plate_rack_in_transport = True
                    self._pending_plate_count = plate_count
                    rack_prim = self.stage.GetPrimAtPath(
                        f"{self.payload_root}/ServingPlateRack"
                    )
                    visible_plates = [
                        str(prim.GetPath())
                        for prim in Usd.PrimRange(rack_prim)
                        if prim.GetName().startswith("Plate_")
                        and UsdGeom.Imageable(prim).ComputeVisibility()
                        != UsdGeom.Tokens.invisible
                    ]
                    self.get_logger().info(
                        "[FoodSpawn][PlateRack] authored "
                        f"count={plate_count} root_valid={rack_prim.IsValid()} "
                        f"visible_plates={visible_plates}"
                    )
                if pizza_requested and 'TrayPizzaPickPlace' in globals():
                    # Constructor authors the physical dish at the kitchen.
                    # Keep this exact object for delivery: constructing it a
                    # second time would re-author the same USD prim hierarchy.
                    pizza_task = TrayPizzaPickPlace(
                        self.stage,
                        payload_root=self.payload_root,
                        robot_root=self.robot_root,
                    )
                    self._spawned_serving_tasks = {
                        "pizza": pizza_task,
                    }
                    dish_prim = self.stage.GetPrimAtPath(
                        f"{self.payload_root}/ServingDish"
                    )
                    if dish_prim.IsValid():
                        dish_body = UsdPhysics.RigidBodyAPI.Get(self.stage, dish_prim.GetPath())
                        if dish_body:
                            dish_body.GetKinematicEnabledAttr().Set(False)
                            self.get_logger().info(
                                "[FoodSpawn] Enabled dynamic physics for pizza dish"
                            )
                # Verify required prims exist on Stage for requested items
                missing_prims = []
                required_roots = {
                    "pizza": f"{self.payload_root}/ServingDish",
                    "drinks": f"{self.payload_root}/ServingDrinks",
                    "cutlery": f"{self.payload_root}/ServingCutlery",
                    "plates": f"{self.payload_root}/ServingPlateRack",
                }
                requested_roots = (
                    [("pizza", pizza_requested), ("drinks", drink_count > 0),
                     ("cutlery", cutlery_requested),
                     ("plates", plate_rack_requested)]
                )
                for label, requested in requested_roots:
                    path = required_roots[label]
                    if requested and not self.stage.GetPrimAtPath(path).IsValid():
                        missing_prims.append(path)

                if missing_prims:
                    raise RuntimeError(f"Food spawn missing required prims on Stage: {missing_prims}")

                self.get_logger().info(
                    "[FoodSpawn] Successfully spawned and verified food prims "
                    f"for command={requested_spawn_command}"
                )
                self.spawn_status_pub.publish(Int32(data=2))  # 2 = COMPLETED
            except Exception as exc:
                self.get_logger().exception(f"Food spawn execution error for command={pending_spawn}: {exc}")
                self.spawn_status_pub.publish(Int32(data=3))  # 3 = FAILED

        with self._lock:
            arm_command = self._pending_arm_command
            self._pending_arm_command = None

        if arm_command is not None:
            try:
                # Match the successful standalone serving tests exactly:
                # lock the chassis before constructing any task. Constructors
                # cache the arm-base transform and derive table targets from
                # it, so locking only before task.initialize() was still too
                # late for a plate-rack-only delivery after mobile docking.
                add_parking_brake(self.stage, self.articulation.prim_path)
                cutlery_requested = arm_command >= 20
                plate_rack_requested = arm_command >= 40
                remainder = arm_command - (40 if plate_rack_requested else 0)
                cutlery_requested = remainder >= 20
                remainder -= 20 if cutlery_requested else 0
                pizza_requested = remainder >= 10
                drink_count = remainder - (10 if pizza_requested else 0)
                if drink_count > 2:
                    raise ValueError(
                        "only the tested soda1/soda2 pair is supported; "
                        f"requested drinks={drink_count}"
                    )
                named_tasks = []
                if pizza_requested:
                    pizza_task = self._spawned_serving_tasks.get("pizza")
                    if pizza_task is None:
                        raise RuntimeError("pizza task was not prepared by food spawn")
                    named_tasks.append(("pizza", pizza_task))
                if drink_count >= 1:
                    named_tasks.append(
                        ("soda1", Soda1PickPlace(
                            self.stage,
                            wait_for_start=bool(named_tasks),
                            payload_root=self.payload_root,
                            robot_root=self.robot_root,
                        ))
                    )
                if drink_count >= 2:
                    named_tasks.append(
                        ("soda2", Soda2PickPlace(
                            self.stage,
                            wait_for_start=bool(named_tasks),
                            payload_root=self.payload_root,
                            robot_root=self.robot_root,
                        ))
                    )
                if cutlery_requested:
                    named_tasks.append(
                        ("cutlery", CutleryBoxPickPlace(
                            self.stage,
                            wait_for_start=bool(named_tasks),
                            payload_root=self.payload_root,
                            robot_root=self.robot_root,
                        ))
                    )
                if plate_rack_requested:
                    rack_prim = self.stage.GetPrimAtPath(
                        f"{self.payload_root}/ServingPlateRack"
                    )
                    visible_plate_count = 0
                    if rack_prim.IsValid():
                        visible_plate_count = sum(
                            1
                            for prim in Usd.PrimRange(rack_prim)
                            if prim.GetName().startswith("Plate_")
                            and UsdGeom.Imageable(prim).ComputeVisibility()
                            != UsdGeom.Tokens.invisible
                        )
                    if not rack_prim.IsValid() or visible_plate_count == 0:
                        if not (1 <= self._pending_plate_count <= 4):
                            raise RuntimeError(
                                "plate-rack arm command received without a "
                                "successful plate spawn command (expected 81..84)"
                            )
                        self.get_logger().warning(
                            "[PlateRack] payload missing before arm start; "
                            f"respawning count={self._pending_plate_count}"
                        )
                        spawn_plate_rack(
                            self.stage,
                            plate_count=self._pending_plate_count,
                            payload_root=self.payload_root,
                            robot_root=self.robot_root,
                        )
                        if not follow_plate_rack_transport(
                            self.stage,
                            payload_root=self.payload_root,
                            robot_root=self.robot_root,
                        ):
                            raise RuntimeError(
                                "plate rack recovery spawn could not be placed "
                                "on the left tray"
                            )
                        rack_prim = self.stage.GetPrimAtPath(
                            f"{self.payload_root}/ServingPlateRack"
                        )
                    self.get_logger().info(
                        "[PlateRack] arm preflight passed "
                        f"count={self._pending_plate_count} "
                        f"root={rack_prim.GetPath()}"
                    )
                    named_tasks.append(
                        ("plate_rack", PlateRackPickPlace(
                            self.stage,
                            wait_for_start=bool(named_tasks),
                            payload_root=self.payload_root,
                            robot_root=self.robot_root,
                        ))
                    )
                if not named_tasks:
                    raise ValueError(f"arm command contains no delivery: {arm_command}")
                task = CommandServingSequence(named_tasks)
                task.initialize(self.articulation, self.dof_names)
                self._active_serving_task = task
                self._serving_paused = False
                self.get_logger().info(
                    f"[integrated-serving] started arm_command={arm_command}"
                )
            except Exception as exc:
                self.get_logger().error(
                    f"integrated serving initialization failed: {exc}"
                )
                remove_parking_brake(self.stage, self.articulation.prim_path)
                self.arm_status_pub.publish(Int32(data=3))

        if (
            self._plate_rack_in_transport
            and 'follow_plate_rack_transport' in globals()
            and (
                self._active_serving_task is None
                or self._active_serving_task.current_name != "plate_rack"
            )
        ):
            if not follow_plate_rack_transport(
                self.stage,
                payload_root=self.payload_root,
                robot_root=self.robot_root,
            ):
                self.get_logger().error(
                    "[plate-rack] transport follow failed during navigation"
                )
                self._plate_rack_in_transport = False

        if self._active_serving_task is not None:
            self._target_vx = self._target_wz = 0.0
            self._cmd_vx = self._cmd_wz = 0.0
            self.articulation.apply_action(
                ArticulationAction(
                    joint_velocities=np.zeros(len(self.wheel_indices), dtype=float),
                    joint_indices=self.wheel_indices,
                )
            )
            if not self._serving_paused:
                self._active_serving_task.step(self.articulation)
            if self._active_serving_task.current_name == "plate_rack":
                # PlateRackPickPlace now owns tray-follow and switches the rack
                # to dynamic collision after tray deployment.
                self._plate_rack_in_transport = False
            if self._active_serving_task.failed:
                self._active_serving_task.close()
                self._active_serving_task = None
                self._active_delivery_table = None
                remove_parking_brake(self.stage, self.articulation.prim_path)
                self.arm_status_pub.publish(Int32(data=3))
                self.get_logger().error("[integrated-serving] delivery failed")
            elif self._active_serving_task.done:
                self._active_serving_task.close()
                self._active_serving_task = None
                self._spawned_serving_tasks = {}
                self._pending_plate_count = 0
                self._arm_returning_to_stow = True
                self._arm_stow_started_at = time.monotonic()
                self._arm_stow_settle_count = 0
                arm_indices = np.asarray(
                    [self.dof_names.index(name) for name in ARM_JOINTS],
                    dtype=np.int32,
                )
                self._arm_stow_command = self.articulation.get_joint_positions()[
                    arm_indices
                ].copy()
                self._arm_stow_last_update = self._arm_stow_started_at
                self._arm_stow_last_log = self._arm_stow_started_at
                self.get_logger().info(
                    "[integrated-serving] delivery complete; returning arm to stow"
                )

        if self._arm_returning_to_stow:
            self._target_vx = self._target_wz = 0.0
            self._cmd_vx = self._cmd_wz = 0.0
            self.articulation.apply_action(
                ArticulationAction(
                    joint_velocities=np.zeros(len(self.wheel_indices), dtype=float),
                    joint_indices=self.wheel_indices,
                )
            )
            arm_indices = np.asarray(
                [self.dof_names.index(name) for name in ARM_JOINTS],
                dtype=np.int32,
            )
            stow = np.asarray(STOW_CONFIGURATION, dtype=float)
            if not self._serving_paused:
                now = time.monotonic()
                dt = min(max(now - self._arm_stow_last_update, 1.0 / 240.0), 0.05)
                self._arm_stow_last_update = now
                command_error = np.arctan2(
                    np.sin(stow - self._arm_stow_command),
                    np.cos(stow - self._arm_stow_command),
                )
                max_step = ARM_STOW_SPEED * dt
                self._arm_stow_command += np.clip(
                    command_error, -max_step, max_step
                )
                self.articulation.apply_action(
                    ArticulationAction(
                        joint_positions=self._arm_stow_command,
                        joint_indices=arm_indices,
                    )
                )
            current = self.articulation.get_joint_positions()[arm_indices]
            error = np.arctan2(np.sin(stow - current), np.cos(stow - current))
            max_error = float(np.max(np.abs(error)))
            if (
                not self._serving_paused
                and time.monotonic() - self._arm_stow_last_log >= 2.0
            ):
                self._arm_stow_last_log = time.monotonic()
                self.get_logger().info(
                    "[integrated-serving] arm stow "
                    f"actual_deg={np.round(np.rad2deg(current), 1).tolist()} "
                    f"max_error={math.degrees(max_error):.1f}deg"
                )
            if max_error <= ARM_STOW_TOLERANCE:
                self._arm_stow_settle_count += 1
            else:
                self._arm_stow_settle_count = 0

            if self._arm_stow_settle_count >= 15:
                self._arm_returning_to_stow = False
                self._arm_stow_started_at = None
                self._arm_stow_command = None
                self._arm_stow_last_update = None
                self._arm_stow_last_log = None
                self._tray_returning_home = True
                self._tray_home_started_at = time.monotonic()
                self._tray_home_settle_count = 0
                self._tray_home_step = 0
                tray_indices = np.asarray(
                    [self.dof_names.index(name) for name in SLIDING_TRAY_JOINTS],
                    dtype=np.int32,
                )
                self._tray_home_start = self.articulation.get_joint_positions()[
                    tray_indices
                ].copy()
                self.get_logger().info(
                    "[integrated-serving] arm stowed; retracting trays from "
                    f"{np.round(self._tray_home_start, 3).tolist()}m"
                )
            elif (
                not self._serving_paused
                and time.monotonic() - self._arm_stow_started_at > 30.0
            ):
                self._arm_returning_to_stow = False
                self._arm_stow_started_at = None
                self._arm_stow_command = None
                self._arm_stow_last_update = None
                self._arm_stow_last_log = None
                remove_parking_brake(self.stage, self.articulation.prim_path)
                self.arm_status_pub.publish(Int32(data=3))
                self.get_logger().error(
                    "[integrated-serving] arm failed to reach stow within 30s"
                )

        if self._tray_returning_home:
            self._target_vx = self._target_wz = 0.0
            self._cmd_vx = self._cmd_wz = 0.0
            self.articulation.apply_action(
                ArticulationAction(
                    joint_velocities=np.zeros(len(self.wheel_indices), dtype=float),
                    joint_indices=self.wheel_indices,
                )
            )
            tray_indices = np.asarray(
                [self.dof_names.index(name) for name in SLIDING_TRAY_JOINTS],
                dtype=np.int32,
            )
            tray_home = np.zeros(len(tray_indices), dtype=float)
            if not self._serving_paused:
                self._tray_home_step += 1
                raw_amount = min(1.0, self._tray_home_step / TRAY_RETRACT_STEPS)
                amount = raw_amount * raw_amount * (3.0 - 2.0 * raw_amount)
                tray_target = self._tray_home_start * (1.0 - amount)
                self.articulation.apply_action(
                    ArticulationAction(
                        joint_positions=tray_target,
                        joint_indices=tray_indices,
                    )
                )
            current_trays = self.articulation.get_joint_positions()[tray_indices]
            if not self._serving_paused and self._tray_home_step % 60 == 0:
                self.get_logger().info(
                    "[integrated-serving] tray retract "
                    f"target={np.round(tray_target, 3).tolist()}m "
                    f"actual={np.round(current_trays, 3).tolist()}m"
                )
            if (
                self._tray_home_step >= TRAY_RETRACT_STEPS
                and np.max(np.abs(current_trays - tray_home)) <= 0.005
            ):
                self._tray_home_settle_count += 1
            else:
                self._tray_home_settle_count = 0

            if self._tray_home_settle_count >= 15:
                self._tray_returning_home = False
                self._tray_home_started_at = None
                self._tray_home_start = None
                self._tray_home_step = 0
                self._completed_delivery_table = self._active_delivery_table
                self._active_delivery_table = None
                remove_parking_brake(self.stage, self.articulation.prim_path)
                self.arm_status_pub.publish(Int32(data=2))
                self.get_logger().info(
                    "[integrated-serving] trays retracted; delivery complete"
                )
            elif (
                not self._serving_paused
                and time.monotonic() - self._tray_home_started_at > 20.0
            ):
                self._tray_returning_home = False
                self._tray_home_started_at = None
                self._tray_home_start = None
                self._tray_home_step = 0
                self._active_delivery_table = None
                remove_parking_brake(self.stage, self.articulation.prim_path)
                self.arm_status_pub.publish(Int32(data=3))
                self.get_logger().error(
                    "[integrated-serving] trays failed to retract within 20s"
                )

        now = time.monotonic()
        dt = min(max(now - self._last_cmd_time, 1.0 / 240.0), 0.05)
        self._last_cmd_time = now

        latch = self._completed_mission_latch
        if latch is not None:
            if now <= float(latch.get("until", 0.0)):
                if now - float(latch.get("last_pub", 0.0)) >= 0.5:
                    latch["last_pub"] = now
                    self._publish_two_wheel_mission_status(
                        "completed",
                        "completed",
                        target=latch.get("target"),
                        mission_id=latch.get("mission_id"),
                    )
                    if latch.get("target") is not None:
                        self.navigation_location_pub.publish(
                            Int32(data=int(latch["target"]))
                        )
            else:
                self._completed_mission_latch = None

        direct_command = self._update_direct_navigation(x, y, yaw)
        with self._lock:
            target_vx = self._target_vx
            target_wz = self._target_wz
        if direct_command is not None:
            target_vx, target_wz = direct_command

        if direct_command is not None:
            # The original TableNavigationServer applied each closed-loop
            # command directly; retain that behavior instead of adding the
            # Nav2 bridge's velocity ramp on top of it.
            self._cmd_vx = target_vx
            self._cmd_wz = target_wz
        else:
            self._cmd_vx = self._slew(
                self._cmd_vx, target_vx, LINEAR_ACCEL_LIMIT, LINEAR_DECEL_LIMIT, dt
            )
            self._cmd_wz = self._slew(
                self._cmd_wz, target_wz, ANGULAR_ACCEL_LIMIT, ANGULAR_DECEL_LIMIT, dt
            )
        vx, wz = self._cmd_vx, self._cmd_wz

        wheel_velocities = (
            self._direct_wheel_ik(vx, wz)
            if direct_command is not None
            else self._differential_ik(vx, wz)
        )
        self.articulation.apply_action(
            ArticulationAction(
                joint_velocities=wheel_velocities,
                joint_indices=self.wheel_indices,
            )
        )

        position, orientation = self.articulation.get_world_pose()
        x = float(position[0])
        y = float(position[1])
        yaw = quaternion_to_yaw(orientation)

        stamp = self._sim_stamp
        # The parked robot cannot navigate while its arm/trays are active.
        # Avoid 180 PhysX raycasts per robot at 10 Hz during the most
        # contact-sensitive manipulation interval. Odom/TF and arm control
        # continue at the full 60 Hz physics rate.
        manipulation_active = (
            self._active_serving_task is not None
            or self._arm_returning_to_stow
            or self._tray_returning_home
        )
        if not manipulation_active:
            self._publish_physx_scan(stamp, x, y, yaw)
        if stamp is None:
            return

        if not self._odom_initialized:
            self._odom_initialized = True
            self._last_pose = (x, y, yaw)
            return

        # World-aligned odom (same numbers as map/rail waypoints).
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = BASE_LINK_NAME
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = float(position[2])
        odom.pose.pose.orientation.z = math.sin(yaw * 0.5)
        odom.pose.pose.orientation.w = math.cos(yaw * 0.5)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)
        self.two_wheel_odom_pub.publish(odom)

        tf = TransformStamped()
        # /clock reaches this Python node about 0.067 s after the RTX scan has
        # already been stamped.  Publish TF 0.10 s ahead so message filters can
        # interpolate the scan without extrapolation failures.
        tf_nanosec = int(stamp.nanosec) + 100_000_000
        tf_sec = int(stamp.sec) + tf_nanosec // 1_000_000_000
        tf.header.stamp = TimeMsg(
            sec=tf_sec,
            nanosec=tf_nanosec % 1_000_000_000,
        )
        tf.header.frame_id = "odom"
        tf.child_frame_id = BASE_LINK_NAME
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = float(position[2])
        tf.transform.rotation.z = math.sin(yaw * 0.5)
        tf.transform.rotation.w = math.cos(yaw * 0.5)
        self.tf_pub.publish(TFMessage(transforms=[tf]))

        self._last_pose = (x, y, yaw)


def main():
    crossing_pedestrian_module = None
    if (
        os.environ.get("NAV_CROSSING_PEDESTRIAN", "1") == "1"
        or os.environ.get("NAV_TYPING_CUSTOMER", "1") == "1"
    ):
        try:
            import crossing_pedestrian_actor as crossing_pedestrian_module

            crossing_pedestrian_module.enable_extensions()
            for _ in range(30):
                simulation_app.update()
        except Exception as exc:
            crossing_pedestrian_module = None
            print(
                f"[crossing_pedestrian] extension setup warning: {exc}",
                flush=True,
            )

    multi_robot = os.environ.get("NAV_MULTI_ROBOT", "0") == "1"
    robot_count_raw = os.environ.get(
        "NAV_ROBOT_COUNT", "2" if multi_robot else "1"
    )
    try:
        robot_count = int(robot_count_raw)
    except ValueError as exc:
        raise ValueError(
            f"NAV_ROBOT_COUNT must be 1 or 2, got {robot_count_raw!r}"
        ) from exc
    if robot_count not in (1, 2):
        raise ValueError(
            f"NAV_ROBOT_COUNT must be 1 or 2, got {robot_count}"
        )
    multi_robot = multi_robot or robot_count == 2
    robot_configs = (
        [
            {
                "name": "robot1",
                "root": "/World/NavRobot1",
                "spawn": Gf.Vec3d(-0.90, 5.25, 0.01),
            },
            {
                "name": "robot2",
                "root": "/World/NavRobot2",
                "spawn": Gf.Vec3d(0.90, 5.25, 0.01),
            },
        ]
        if multi_robot
        else [
            {
                "name": "",
                "root": "/World/NavRobot",
                "spawn": SPAWN_POSITION,
            }
        ]
    )

    stage = None
    robot_records = []
    for index, config in enumerate(robot_configs):
        set_robot_context(config["root"], config["spawn"])
        stage = open_restaurant_and_robot(open_stage=index == 0)
        configure_wheel_contact_material(stage)
        configure_gripper_contact_material(stage)
        articulation_path = find_articulation_path(stage)
        configure_physics_stability(stage, articulation_path)
        prepare_parking_brake(stage, articulation_path)
        robot_records.append((config, articulation_path))

    # The drive authoring pass may safely cover both complete robot
    # articulations after they have been composed into the stage.
    configure_joint_drives(stage)

    # Finish every character reference and AnimationGraph USD edit while the
    # timeline is still stopped.  Previously these were authored after both
    # PhysX and the ROS camera SDG writers had started.  Isaac Sim 5.1 then
    # delivered their pending ObjectsChanged notices during a physics update,
    # which repeatedly crashed in PXR SdfPath conversion/Python GC.
    timeline = omni.timeline.get_timeline_interface()
    if timeline.is_playing():
        timeline.pause()

    typing_person = None
    crossing_person = None
    crossing_pedestrian_controller = None
    if (
        crossing_pedestrian_module is not None
        and crossing_pedestrian_module.TYPING_ENABLED
    ):
        try:
            typing_person = (
                crossing_pedestrian_module.spawn_typing_customer(stage)
            )
        except Exception as exc:
            print(
                f"[typing_topic] actor setup warning: {exc}",
                flush=True,
            )

    if (
        crossing_pedestrian_module is not None
        and crossing_pedestrian_module.ENABLED
    ):
        try:
            crossing_person = crossing_pedestrian_module.spawn(stage)
            crossing_pedestrian_controller = (
                crossing_pedestrian_module.CrossingPedestrianController(
                    crossing_person
                )
            )
        except Exception as exc:
            print(
                f"[crossing_pedestrian] actor setup warning: {exc}",
                flush=True,
            )

    # Drain character composition and scripting notices before creating the
    # sensor graphs. Physics remains stopped throughout stage authoring.
    for _ in range(30):
        simulation_app.update()
    print(
        "[characters] initialization settled before physics/sensors "
        f"typing={typing_person is not None} "
        f"crossing={crossing_person is not None}",
        flush=True,
    )

    # Author every sensor graph before PhysX starts. Previously robot1 was
    # initialized, its OmniGraph was added while the timeline was running, and
    # only then was robot2 initialized. The resulting stage resync could
    # invalidate or delay robot2's PhysX articulation view intermittently.
    for index, (config, _articulation_path) in enumerate(robot_records):
        set_robot_context(config["root"], config["spawn"])
        connect_embedded_sensor_ros(
            stage,
            robot_root=config["root"],
            robot_name=config["name"],
            publish_clock=index == 0,
        )

    # Let USD/OmniGraph notices settle while stopped, then create the complete
    # two-robot PhysX scene in one timeline transition. A few updates are a
    # deterministic barrier for referenced articulation registration; no USD
    # topology is edited between this point and both initialize() calls.
    for _ in range(10):
        simulation_app.update()
    timeline.play()
    physics_settle_frames = max(
        2, int(os.environ.get("NAV_PHYSICS_INIT_SETTLE_FRAMES", "8"))
    )
    for _ in range(physics_settle_frames):
        simulation_app.update()
    print(
        f"[nav_robot] PhysX scene settled frames={physics_settle_frames} "
        f"articulations={len(robot_records)}",
        flush=True,
    )

    initialized_records = []
    for config, articulation_path in robot_records:
        set_robot_context(config["root"], config["spawn"])
        articulation, dof_names = initialize_robot(
            articulation_path,
            name=config["name"] or "nav_ridgeback",
        )
        initialized_records.append((config, articulation, dof_names))

    # PhysX handle creation can re-process schemas under an articulation.
    # Re-enforce the embedded camera invariant only after both handles exist,
    # so a stage resync cannot race robot2's articulation initialization.
    for config, _articulation, _dof_names in initialized_records:
        sanitize_embedded_rsd455(stage, config["root"])

    if not rclpy.ok():
        rclpy.init(args=[])
    bridges = []
    executor = SingleThreadedExecutor()
    for config, articulation, dof_names in initialized_records:
        bridge = NavBridge(
            articulation,
            dof_names,
            stage,
            robot_name=config["name"],
            robot_root=config["root"],
        )
        bridges.append(bridge)
        executor.add_node(bridge)
    bridge = bridges[0]

    if not timeline.is_playing():
        timeline.play()

    print(
        f"[nav_robot] domain={os.environ.get('ROS_DOMAIN_ID', '0')} "
        f"mode={'multi-integrated' if multi_robot else 'single-integrated'} "
        f"robots={[(item['name'] or 'default', tuple(item['spawn'])) for item in robot_configs]} "
        f"yaw={SPAWN_YAW:.2f}",
        flush=True,
    )
    print(
        "[nav_robot] skid-steer effective half-track="
        f"{DIFFERENTIAL_HALF_TRACK:.3f}m",
        flush=True,
    )

    typing_customer_controller = None
    if typing_person is not None:
        try:
            typing_customer_controller = (
                crossing_pedestrian_module.TypingTopicController(
                    typing_person
                )
            )
        except Exception as exc:
            print(
                f"[typing_topic] actor setup warning: {exc}",
                flush=True,
            )

    if crossing_pedestrian_controller is not None:
        # HMI's legacy "person spawn/remove" test controls now drive the
        # walking CrossingPedestrian instead of the retired corridor actor.
        bridge.set_obstacle_test_controller(crossing_pedestrian_controller)
        print(
            "[crossing_pedestrian] enabled service-controlled visibility: "
            "/obstacle_test/set_visible",
            flush=True,
        )

    # The simple hand-only intrusion actor and the corridor obstacle test
    # actor are retired from the default run path.  HMI's "hand" test button
    # now triggers TypingTopicController via /hand_test/type_keyboard, and
    # the "person" test button drives crossing_pedestrian_controller above.
    # The legacy /hand_test/set_visible and /obstacle_test/set_visible
    # services stay registered on NavBridge for compatibility; without a
    # controller attached, /hand_test/set_visible simply reports "not ready".

    try:
        while simulation_app.is_running():
            simulation_app.update()
            # Serialize ROS callbacks with Isaac/PhysX access.  A background
            # executor could receive the manager's follow-up command while
            # tick() was publishing navigation completion, corrupting native
            # rclpy/Fast DDS state and terminating with "stack smashing".
            executor.spin_once(timeout_sec=0.0)
            if typing_customer_controller is not None:
                try:
                    typing_customer_controller.update()
                except Exception as exc:
                    print(
                        f"[typing_topic] update warning: {exc}",
                        flush=True,
                    )
                    typing_customer_controller.shutdown()
                    typing_customer_controller = None
            if crossing_pedestrian_controller is not None:
                try:
                    crossing_pedestrian_controller.update()
                except Exception as exc:
                    print(
                        f"[crossing_pedestrian] update warning: {exc}",
                        flush=True,
                    )
                    crossing_pedestrian_controller = None
            try:
                sim_time = timeline.get_current_time()
                for config, active_bridge in zip(robot_configs, bridges):
                    set_robot_context(config["root"], config["spawn"])
                    active_bridge.tick(float(sim_time))
            except Exception as exc:
                print(f"[err] tick error: {exc}", flush=True)
    finally:
        if typing_customer_controller is not None:
            typing_customer_controller.shutdown()
        if crossing_pedestrian_controller is not None:
            crossing_pedestrian_controller.shutdown()
        executor.shutdown()
        for active_bridge in bridges:
            active_bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()
