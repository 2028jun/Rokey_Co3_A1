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

WORKSPACE = Path(
    os.environ.get("NAV_ROBOT_WS", Path(__file__).resolve().parents[1])
).resolve()
SERVING_WORKSPACE = WORKSPACE.parent / "serving_robot"

_ros_bridge_lib = Path(
    "/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/"
    "exts/isaacsim.ros2.bridge/humble/lib"
)
os.environ.setdefault("ROS_DISTRO", "humble")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
os.environ["ROS_DOMAIN_ID"] = os.environ.get("NAV_ROBOT_ROS_DOMAIN_ID", "102")
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

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time as TimeMsg
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
from std_srvs.srv import SetBool

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
try:
    from drink_serving import spawn_soda_cans
    from cutlery_serving import spawn_cutlery_box
    from pizza_serving import TrayPizzaPickPlace
    from soda1_delivery import Soda1PickPlace
    from soda2_delivery import Soda2PickPlace
    from cutlery_pick_place import CutleryBoxPickPlace
    print(
        "[food_spawn] loaded pizza, soda1, soda2 and cutlery delivery modules",
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
LIDAR_MAX_RANGE = 12.0
LIDAR_SAMPLES = 180
LIDAR_PERIOD_SEC = 0.10
LIDAR_SENSOR_FORWARD = 0.48
LIDAR_SENSOR_HEIGHT = 0.45

_front_lidar_render_product = None
_front_lidar_writer = None
_embedded_lidar_render_product = None
_embedded_lidar_writer = None


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


def add_parking_brake(stage, articulation_path):
    brake_prim = stage.GetPrimAtPath("/World/NavRobot/ParkingBrake")
    if not brake_prim.IsValid():
        joint = UsdPhysics.FixedJoint.Define(stage, "/World/NavRobot/ParkingBrake")
        joint.CreateBody1Rel().SetTargets([Sdf.Path(articulation_path)])
        base_prim = stage.GetPrimAtPath(articulation_path)
        if base_prim.IsValid():
            transform = UsdGeom.XformCache().GetLocalToWorldTransform(base_prim)
            base_position = transform.ExtractTranslation()
            base_rotation = transform.ExtractRotationQuat()
            imag = base_rotation.GetImaginary()
            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*map(float, base_position)))
            joint.CreateLocalRot0Attr().Set(Gf.Quatf(float(base_rotation.GetReal()), Gf.Vec3f(*map(float, imag))))
            joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            print("[mobile robot] parking brake=on (fixed base via UsdPhysics.FixedJoint)", flush=True)


def remove_parking_brake(stage):
    brake_prim = stage.GetPrimAtPath("/World/NavRobot/ParkingBrake")
    if brake_prim.IsValid():
        stage.RemovePrim("/World/NavRobot/ParkingBrake")
        print("[mobile robot] parking brake=off (unlocked base)", flush=True)


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
    for _ in range(5):
        simulation_app.update()
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
    print("[nav_robot] using embedded D455 and RPLIDAR sensor layer", flush=True)
    return stage


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
    """CPU PhysX + base damping — ported from serving_robot mobile demo."""
    scene_prim = stage.GetPrimAtPath("/World/PhysicsScene")
    if not scene_prim.IsValid():
        raise RuntimeError("restaurant PhysicsScene is missing")
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene_prim)
    physx_scene.CreateEnableStabilizationAttr(True)
    # Kitchen has many legacy triangle-mesh colliders; CPU PhysX is more robust.
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
    print(
        "[nav_robot] physics=CPU/120Hz stabilization=on solver=32/4",
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


def find_articulation_path(stage) -> str:
    for path in ARTICULATION_CANDIDATES:
        if stage.GetPrimAtPath(path).IsValid():
            return path
    # Fallback: first articulation root under NavRobot
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if path.startswith(ROBOT_ROOT) and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return path
    raise RuntimeError("could not find robot articulation prim")


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


def initialize_robot(articulation_path: str):
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(2):
        simulation_app.update()

    articulation = SingleArticulation(
        prim_path=articulation_path, name="nav_ridgeback"
    )
    articulation.initialize()
    if not articulation.handles_initialized:
        raise RuntimeError(f"invalid articulation handle: {articulation_path}")
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


def connect_embedded_sensor_ros(stage):
    """Connect the D455/RPLIDAR already contained in the two-wheel USD."""
    global _embedded_lidar_render_product, _embedded_lidar_writer
    camera_width = int(os.environ.get("NAV_CAMERA_WIDTH", "1280"))
    camera_height = int(os.environ.get("NAV_CAMERA_HEIGHT", "960"))
    if camera_width <= 0 or camera_height <= 0:
        raise ValueError(
            "NAV_CAMERA_WIDTH and NAV_CAMERA_HEIGHT must be positive"
        )
    base_path = (
        f"{ROBOT_ROOT}/Robot/ridgeback_base_link/ridgeback_base_link"
    )
    sensor_mount = f"{base_path}/fixed_table_depth_camera/realsense_d455"
    depth_camera = f"{sensor_mount}/RSD455/Camera_Pseudo_Depth"
    lidar_path = f"{base_path}/base_scan/RPLIDAR_S2E"
    for required in (depth_camera, lidar_path):
        if not stage.GetPrimAtPath(required).IsValid():
            raise RuntimeError(f"embedded sensor prim is missing: {required}")

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": f"{ROBOT_ROOT}/EmbeddedSensorsROS2", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("ColorPub", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("DepthPub", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ],
            keys.SET_VALUES: [
                ("RenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(depth_camera)]),
                ("RenderProduct.inputs:width", camera_width),
                ("RenderProduct.inputs:height", camera_height),
                ("ColorPub.inputs:nodeNamespace", "camera/color"),
                ("ColorPub.inputs:topicName", "image_raw"),
                ("ColorPub.inputs:frameId", "d455_color_optical_frame"),
                ("ColorPub.inputs:type", "rgb"),
                # Publish every rendered frame.  The previous value of 3
                # capped the RGB stream to one image per four sim frames.
                ("ColorPub.inputs:frameSkipCount", 0),
                ("DepthPub.inputs:nodeNamespace", "camera/depth"),
                ("DepthPub.inputs:topicName", "image_raw"),
                ("DepthPub.inputs:frameId", "d455_depth_optical_frame"),
                ("DepthPub.inputs:type", "depth"),
                # Depth is not consumed by hand_safety. Keep it at a low rate
                # so RGB inference gets the shared GPU budget.
                ("DepthPub.inputs:frameSkipCount", 29),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "RenderProduct.inputs:execIn"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("Context.outputs:context", "ColorPub.inputs:context"),
                ("Context.outputs:context", "DepthPub.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("RenderProduct.outputs:execOut", "ColorPub.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", "ColorPub.inputs:renderProductPath"),
                ("RenderProduct.outputs:execOut", "DepthPub.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", "DepthPub.inputs:renderProductPath"),
            ],
        },
    )

    import omni.replicator.core as rep

    _embedded_lidar_render_product = rep.create.render_product(
        lidar_path, [1, 1], name="IntegratedServingRPLidar"
    )
    _embedded_lidar_writer = rep.writers.get("RtxLidarROS2PublishLaserScan")
    _embedded_lidar_writer.initialize(topicName="/scan", frameId="base_scan")
    _embedded_lidar_writer.attach([_embedded_lidar_render_product])
    print(
        f"[ros] embedded D455 RGB/depth {camera_width}x{camera_height}, "
        "RPLIDAR /scan and /clock connected",
        flush=True,
    )


def create_sensor_static_tf(stage, lidar_path: str, camera_path: str, node: Node, broadcaster):
    # Published every odom tick as well; initial helper keeps frames available.
    pass


class CommandServingSequence:
    """Frame-driven composition of the already tested serving tasks."""

    def __init__(self, named_tasks):
        self._named_tasks = list(named_tasks)
        self._index = 0
        self.done = False
        self.failed = False

    def initialize(self, articulation, dof_names):
        for name, task in self._named_tasks:
            task.initialize(articulation, dof_names)
        names = " -> ".join(name for name, _ in self._named_tasks)
        print(f"[integrated-serving] order={names} (all tasks initialized)", flush=True)

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
            next_task.start_with_deployed_trays()

    def close(self):
        for _, task in self._named_tasks:
            try:
                task.close()
            except Exception:
                pass


class NavBridge(Node):
    """cmd_vel subscriber + odom/TF publisher + food spawn & arm serving server."""

    def __init__(self, articulation, dof_names, stage=None):
        super().__init__("nav_robot_isaac_bridge")
        self.articulation = articulation
        self.dof_names = dof_names
        self.stage = stage
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
        self._direct_nav = None
        self._direct_nav_request = None
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
        self._clearance_start = None
        self._obstacle_scale = 1.0
        self._last_scan_time = 0.0

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.scan_pub = self.create_publisher(LaserScan, "/scan", sensor_qos)
        self.nav_scan_pub = self.create_publisher(LaserScan, "/nav_robot/scan", sensor_qos)
        self.create_subscription(Twist, "/nav_robot/cmd_vel", self._on_cmd_vel, qos)
        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, qos)
        self.create_subscription(
            PoseStamped, "/nav_robot/teleport", self._on_teleport, qos
        )
        self.create_subscription(LaserScan, "/scan", self._on_scan, sensor_qos)
        self.create_subscription(LaserScan, "/nav_robot/scan", self._on_scan, sensor_qos)
        self.odom_pub = self.create_publisher(Odometry, "/nav_robot/odom", qos)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Subsystems Services & Publishers for Manager Node
        self.spawn_status_pub = self.create_publisher(Int32, "/food_spawn/status", qos)
        self.arm_status_pub = self.create_publisher(Int32, "/arm/status", qos)
        self.navigation_status_pub = self.create_publisher(
            Int32, "/navigation/status", qos
        )
        self.navigation_location_pub = self.create_publisher(
            Int32, "/navigation/current_location", qos
        )

        # Standard Int32 topic subscribers for Isaac Sim food spawning & arm serving
        self.create_subscription(Int32, "/food_spawn/trigger", self._on_food_spawn_trigger, qos)
        self.create_subscription(Int32, "/arm/trigger", self._on_arm_trigger, qos)
        self.create_subscription(
            Int32, "/navigation/trigger", self._on_navigation_trigger, qos
        )
        self._obstacle_test_controller = None
        self.create_service(
            SetBool,
            "/hand_test/set_visible",
            self._on_hand_test_set_visible,
        )
        self.create_service(
            SetBool,
            "/obstacle_test/set_visible",
            self._on_obstacle_test_set_visible,
        )

        if TaskCommand is not None:
            self.create_service(
                TaskCommand,
                "/food_spawn/command",
                self._on_food_spawn_command,
            )
            self.create_service(
                TaskCommand,
                "/arm/command",
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
        self.navigation_status_pub.publish(Int32(data=2))
        subsystem_services = (
            ", /food_spawn/command, /arm/command"
            if TaskCommand is not None
            else " (custom subsystem services unavailable)"
        )
        print(
            f"[ros] active services: /navigation/command{subsystem_services}\n"
            "[ros] active status topics: /navigation/status, "
            "/food_spawn/status, /arm/status",
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
        archive_root = f"/World/Delivered/Table{table_id}/Trip{trip_number}"
        UsdGeom.Scope.Define(self.stage, "/World/Delivered")
        UsdGeom.Scope.Define(self.stage, f"/World/Delivered/Table{table_id}")
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
                self.navigation_status_pub.publish(Int32(data=1))
                return True
            self._direct_nav_request = target
            self._target_vx = 0.0
            self._target_wz = 0.0
        self.navigation_status_pub.publish(Int32(data=1))
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
        self.static_tf_broadcaster.sendTransform(static_tfs)

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
            else "/World/NavRobot"
        )

        ranges = []
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

            if hit and hit.get("hit") and not rigid_body.startswith(robot_prefix):
                distance = float(hit["distance"])
            else:
                distance = math.inf

            if distance < LIDAR_MIN_RANGE:
                distance = math.inf

            ranges.append(distance)

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

        self._process_obstacle_ranges(
            ranges,
            angle_min,
            angle_increment,
            LIDAR_MIN_RANGE,
            LIDAR_MAX_RANGE,
        )

    def _on_scan(self, msg: LaserScan):
        self._process_obstacle_ranges(
            msg.ranges,
            msg.angle_min,
            msg.angle_increment,
            msg.range_min,
            msg.range_max,
        )

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
                self._obstacle_scale = 1.0
                return

            if mission["mode"] == "legacy_table":
                stage = mission.get("stage")
                if stage not in ("move_to_pre_dock", "final_approach"):
                    self._obstacle_scale = 1.0
                    return
            else:
                stages = mission.get("stages", [])
                index = mission.get("index", 0)
                if index >= len(stages):
                    self._obstacle_scale = 1.0
                    return
                kind = stages[index].get("kind")
                if kind not in ("axis_x", "axis_y"):
                    self._obstacle_scale = 1.0
                    return

            slow_distance = 1.4
            stop_distance = 0.65
            resume_distance = 0.95
            front_angle = math.radians(15.0)

            valid_ranges = []
            for idx, distance in enumerate(ranges):
                angle = angle_min + idx * angle_increment
                norm_angle = math.atan2(math.sin(angle), math.cos(angle))
                if abs(norm_angle) > front_angle:
                    continue
                if not math.isfinite(distance):
                    continue
                if range_min <= distance <= range_max:
                    valid_ranges.append(distance)

            if not valid_ranges:
                nearest = float("inf")
                close_points = []
            else:
                nearest = min(valid_ranges)
                close_points = [d for d in valid_ranges if d <= stop_distance]

            now = time.monotonic()

            if not self._obstacle_stop:
                if len(close_points) >= 3 and nearest <= stop_distance:
                    self._start_obstacle_stop(nearest)
                elif nearest < slow_distance:
                    ratio = (nearest - stop_distance) / (slow_distance - stop_distance)
                    ratio = float(np.clip(ratio, 0.0, 1.0))
                    self._obstacle_scale = max(0.03, ratio * ratio)
                else:
                    self._obstacle_scale = 1.0
            else:
                self._obstacle_scale = 0.0
                if nearest >= resume_distance:
                    if self._clearance_start is None:
                        self._clearance_start = now
                    elif now - self._clearance_start >= 0.5:
                        self._finish_obstacle_stop(nearest)
                else:
                    self._clearance_start = None

            if nearest < slow_distance or self._obstacle_stop:
                if now - getattr(self, "_last_obstacle_log", 0.0) >= 0.5:
                    self._last_obstacle_log = now
                    self.get_logger().info(
                        f"obstacle distance={nearest:.2f}m scale={self._obstacle_scale:.2f} stop={self._obstacle_stop}"
                    )

    def _start_obstacle_stop(self, distance: float):
        if self._obstacle_stop:
            return
        self._obstacle_stop = True
        self._obstacle_stop_started = time.monotonic()
        self._clearance_start = None
        self._obstacle_scale = 0.0
        self._target_vx = 0.0
        self._target_wz = 0.0
        self.get_logger().warning(
            f"전방 장애물(사람) 최근접 감지: distance={distance:.2f}m <= {1.0}m, 주행 정지"
        )

    def _finish_obstacle_stop(self, distance: float):
        if not self._obstacle_stop:
            return
        paused_for = time.monotonic() - (self._obstacle_stop_started or time.monotonic())
        if self._direct_nav is not None and "stage_start" in self._direct_nav:
            self._direct_nav["stage_start"] += paused_for
        self._obstacle_stop = False
        self._obstacle_stop_started = None
        self._clearance_start = None
        self._obstacle_scale = 1.0
        self.get_logger().info(
            f"전방 장애물(사람) 해제: distance={distance:.2f}m >= 1.2m, 0.5s 안전 지연 완료, 주행 재개"
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
        # The manager deliberately sends target=4 again at the beginning of a
        # new order even after the previous return already reported kitchen.
        # Treat that verification request as an arrival acknowledgement.  A
        # second route here could pivot the base despite no position change.
        if (
            target == 4
            and self._navigation_location == 4
            and math.hypot(x - 0.0, y - 5.25) <= 0.10
        ):
            self._direct_nav = None
            self._cmd_vx = self._cmd_wz = 0.0
            self._target_vx = self._target_wz = 0.0
            self.navigation_location_pub.publish(Int32(data=4))
            self.navigation_status_pub.publish(Int32(data=2))
            self.get_logger().info(
                "already at kitchen; acknowledged redundant target=4 "
                "without moving"
            )
            return
        stages = (
            build_kitchen_route(x, y)
            if target == 4
            else build_table_route(target, x, y)
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
        self._clearance_start = None
        self._cmd_vx = 0.0
        self._cmd_wz = 0.0
        self.get_logger().info(
            f"direct route started target={target} pose=({x:.2f},{y:.2f},"
            f"{math.degrees(yaw):.1f}deg)"
        )

    def _finish_direct_navigation(self, success, reason=""):
        mission = self._direct_nav
        if mission is None:
            return
        target = mission["target"]
        self._direct_nav = None
        self._obstacle_stop = False
        self._obstacle_stop_started = None
        self._clearance_start = None
        self._cmd_vx = self._cmd_wz = 0.0
        self._target_vx = self._target_wz = 0.0
        if success:
            self._navigation_location = target
            self.navigation_location_pub.publish(Int32(data=target))
            self.navigation_status_pub.publish(Int32(data=2))
            self.get_logger().info(f"direct navigation complete target={target}")
        else:
            self.navigation_status_pub.publish(Int32(data=3))
            self.get_logger().error(
                f"direct navigation failed target={target}: {reason}"
            )

    def _update_direct_navigation(self, x, y, yaw):
        mission = self._direct_nav
        if mission is None:
            return None
        if self._navigation_paused or self._obstacle_stop:
            return 0.0, 0.0
        if mission["mode"] == "legacy_table":
            return self._update_legacy_table_navigation(mission, x, y, yaw)

        stage = mission["stages"][mission["index"]]
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
            axis = x if kind == "axis_x" else y
            error = stage["value"] - axis
            done = abs(error) <= 0.05
            desired_yaw = stage["yaw"]
            yaw_error = self._angle_error(desired_yaw, yaw)
            if not done:
                requested = min(abs(stage["speed"]), max(0.045, abs(error) * 0.8))
                vx = math.copysign(requested, stage["speed"])
                wz = float(np.clip(1.6 * yaw_error, -0.28, 0.28))
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
            position_ok = distance <= 0.040
            yaw_ok = abs(yaw_error) <= math.radians(2.0)
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
            navigation_active = (
                self._direct_nav is not None
                or self._direct_nav_request is not None
            )
            self.navigation_location_pub.publish(
                Int32(data=self._navigation_location)
            )
            self.navigation_status_pub.publish(
                Int32(data=1 if navigation_active else 2)
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
                spawn_command = int(pending_spawn)
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
                gc.collect()
                payload_paths = (
                    "/World/ServingDish",
                    "/World/PizzaBoardBail",
                    "/World/PizzaBoardBailHinge",
                    "/World/PizzaBoardGripBearing",
                    "/World/PizzaBoardGripBlock",
                    "/World/ServingDrinks",
                    "/World/ServingCutlery",
                )
                reused_payload_paths = set()
                if pizza_requested:
                    reused_payload_paths.update(payload_paths[:5])
                if drink_count:
                    reused_payload_paths.add("/World/ServingDrinks")
                if cutlery_requested:
                    reused_payload_paths.add("/World/ServingCutlery")
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
                    spawn_soda_cans(self.stage, count=drink_count)
                if cutlery_requested and 'spawn_cutlery_box' in globals():
                    spawn_cutlery_box(self.stage)
                if pizza_requested and 'TrayPizzaPickPlace' in globals():
                    # Constructor authors the physical dish at the kitchen.
                    # Keep this exact object for delivery: constructing it a
                    # second time would re-author the same USD prim hierarchy.
                    pizza_task = TrayPizzaPickPlace(self.stage)
                    self._spawned_serving_tasks = {
                        "pizza": pizza_task,
                    }
                    dish_prim = self.stage.GetPrimAtPath("/World/ServingDish")
                    if dish_prim.IsValid():
                        dish_body = UsdPhysics.RigidBodyAPI.Get(self.stage, dish_prim.GetPath())
                        if dish_body:
                            dish_body.GetKinematicEnabledAttr().Set(False)
                            self.get_logger().info(
                                "[FoodSpawn] Enabled dynamic physics for pizza dish"
                            )
            except Exception as exc:
                self.get_logger().error(f"Food spawn execution error: {exc}")
                self.spawn_status_pub.publish(Int32(data=3))
            else:
                self.spawn_status_pub.publish(Int32(data=2))  # 2 = COMPLETED

        with self._lock:
            arm_command = self._pending_arm_command
            self._pending_arm_command = None

        if arm_command is not None:
            try:
                cutlery_requested = arm_command >= 20
                remainder = arm_command - (20 if cutlery_requested else 0)
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
                            self.stage, wait_for_start=bool(named_tasks)
                        ))
                    )
                if drink_count >= 2:
                    named_tasks.append(
                        ("soda2", Soda2PickPlace(
                            self.stage, wait_for_start=bool(named_tasks)
                        ))
                    )
                if cutlery_requested:
                    named_tasks.append(
                        ("cutlery", CutleryBoxPickPlace(
                            self.stage, wait_for_start=bool(named_tasks)
                        ))
                    )
                if not named_tasks:
                    raise ValueError(f"arm command contains no delivery: {arm_command}")
                task = CommandServingSequence(named_tasks)
                add_parking_brake(self.stage, self.articulation.prim_path)
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
                remove_parking_brake(self.stage)
                self.arm_status_pub.publish(Int32(data=3))

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
            if self._active_serving_task.failed:
                self._active_serving_task.close()
                self._active_serving_task = None
                self._active_delivery_table = None
                gc.collect()
                remove_parking_brake(self.stage)
                self.arm_status_pub.publish(Int32(data=3))
                self.get_logger().error("[integrated-serving] delivery failed")
            elif self._active_serving_task.done:
                self._active_serving_task.close()
                self._active_serving_task = None
                self._spawned_serving_tasks = {}
                gc.collect()
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
                remove_parking_brake(self.stage)
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
                remove_parking_brake(self.stage)
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
                remove_parking_brake(self.stage)
                self.arm_status_pub.publish(Int32(data=3))
                self.get_logger().error(
                    "[integrated-serving] trays failed to retract within 20s"
                )

        now = time.monotonic()
        dt = min(max(now - self._last_cmd_time, 1.0 / 240.0), 0.05)
        self._last_cmd_time = now

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
        self.tf_broadcaster.sendTransform(tf)

        self._last_pose = (x, y, yaw)


def main():
    stage = open_restaurant_and_robot()
    configure_joint_drives(stage)
    configure_wheel_contact_material(stage)
    articulation_path = find_articulation_path(stage)
    configure_physics_stability(stage, articulation_path)
    articulation, dof_names = initialize_robot(articulation_path)

    # The robot USD already contains its D455 and RTX RPLIDAR.  Creating the
    # old PhysX nav_lidar here produced a second white lidar body at the world
    # origin, which looked like a detached M0609 link.
    connect_embedded_sensor_ros(stage)

    if not rclpy.ok():
        rclpy.init(args=[])
    bridge = NavBridge(articulation, dof_names, stage)
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)

    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        timeline.play()

    print(
        f"[nav_robot] domain={os.environ['ROS_DOMAIN_ID']} "
        f"spawn=({SPAWN_POSITION[0]:.2f},{SPAWN_POSITION[1]:.2f}) "
        f"yaw={SPAWN_YAW:.2f}",
        flush=True,
    )
    print(
        "[nav_robot] skid-steer effective half-track="
        f"{DIFFERENTIAL_HALF_TRACK:.3f}m",
        flush=True,
    )

    reach_animator = None
    if os.environ.get("MOBILE_DEMO_HAND_TEST", "1") == "1":
        try:
            import hand_intrusion_test_actor as hand_test
            reach_animator = hand_test.HandSpawnAnimator(stage)
            bridge.set_hand_test_controller(reach_animator)
            print(
                "[hand_test] enabled service-controlled hand-only test: "
                "/hand_test/set_visible",
                flush=True,
            )
        except Exception as exc:
            print(f"[hand_test] actor setup warning: {exc}", flush=True)

    obstacle_person_controller = None
    if os.environ.get("MOBILE_DEMO_OBSTACLE_TEST", "1") == "1":
        try:
            import corridor_obstacle_test_actor as obstacle_test
            obstacle_person_controller = obstacle_test.CorridorPersonSpawnController(stage)
            bridge.set_obstacle_test_controller(obstacle_person_controller)
            print(
                "[obstacle_test] enabled service-controlled corridor person test: "
                "/obstacle_test/set_visible",
                flush=True,
            )
        except Exception as exc:
            print(f"[obstacle_test] actor setup warning: {exc}", flush=True)

    try:
        while simulation_app.is_running():
            simulation_app.update()
            # Serialize ROS callbacks with Isaac/PhysX access.  A background
            # executor could receive the manager's follow-up command while
            # tick() was publishing navigation completion, corrupting native
            # rclpy/Fast DDS state and terminating with "stack smashing".
            executor.spin_once(timeout_sec=0.0)
            if reach_animator is not None:
                try:
                    reach_animator.update()
                except Exception:
                    pass
            if obstacle_person_controller is not None:
                try:
                    obstacle_person_controller.update()
                except Exception:
                    pass
            sim_time = timeline.get_current_time()
            bridge.tick(float(sim_time))
    finally:
        executor.shutdown()
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()
