"""Spawn the Ridgeback + serving shelf + M0609 robot in the restaurant.

This script targets Isaac Sim 5.1.0-rc.19.  It is intentionally not launched
by the ROS package build.  Run it only when the Isaac demonstration is needed.
Set MOBILE_DEMO_AUTORUN=1 to enable the short wheel/arm diagnostic sequence.
"""

import math
import os
import sys
import threading
import time
import traceback
from pathlib import Path

# Isaac Sim 5.1 uses Python 3.11, so it must load the bridge-bundled Humble
# libraries rather than the system ROS Python 3.10 extension.  Re-exec once so
# the dynamic loader sees this path before Kit starts.
_ros_bridge_lib = Path(
    "/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/"
    "exts/isaacsim.ros2.bridge/humble/lib"
)
_ros_required = (
    os.environ.get("MOBILE_DEMO_ROS_CAMERA", "1") == "1"
    or os.environ.get("MOBILE_DEMO_TABLE_SERVICE", "1") == "1"
)
if _ros_required:
    os.environ.setdefault("ROS_DISTRO", "humble")
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    _mobile_domain_id = os.environ.get("MOBILE_DEMO_ROS_DOMAIN_ID")
    if _mobile_domain_id:
        os.environ["ROS_DOMAIN_ID"] = _mobile_domain_id
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
    if _needs_ros_env and os.environ.get("MOBILE_ROS_REEXEC") != "1":
        _reexec_env = os.environ.copy()
        _reexec_env["LD_LIBRARY_PATH"] = ":".join([str(_ros_bridge_lib), *_ld_paths])
        _reexec_env["PYTHONPATH"] = ":".join(_python_paths)
        _reexec_env["MOBILE_ROS_REEXEC"] = "1"
        os.execve(sys.executable, [sys.executable, *sys.argv], _reexec_env)

from isaacsim import SimulationApp


HEADLESS = os.environ.get("MOBILE_DEMO_HEADLESS", "0") == "1"
simulation_app = SimulationApp({"headless": HEADLESS})

import numpy as np
import omni.kit.app
import omni.kit.commands
import omni.graph.core as og
import omni.timeline
import omni.usd
import usdrt.Sdf
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.viewports import create_viewport_for_camera
from isaacsim.core.utils.types import ArticulationAction
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

# Enabling the bridge adds Isaac's Python 3.11 Humble rclpy and generated
# standard interface modules to sys.path.  They cannot be imported earlier.
_extension_manager = omni.kit.app.get_app().get_extension_manager()
_extension_manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
for _ in range(3):
    simulation_app.update()

import rclpy
from example_interfaces.srv import AddTwoInts
from rclpy.executors import SingleThreadedExecutor


WORKSPACE = Path(
    os.environ.get("COBOT3_WS", Path(__file__).resolve().parents[1])
).resolve()
CANONICAL_SERVING_WORKSPACE = Path("/home/rokey/cobot3_ws/serving_robot")
# Isaac's URDF importer resolves package:// URLs through ROS_PACKAGE_PATH,
# while a sourced ROS 2/ament workspace does not populate that ROS 1 variable.
_package_roots = [
    WORKSPACE / "install/m0609_isaac_description/share",
    WORKSPACE / "install/ridgeback_m0609_description/share",
]
os.environ["ROS_PACKAGE_PATH"] = ":".join(
    [str(path) for path in _package_roots if path.is_dir()]
    + ([os.environ["ROS_PACKAGE_PATH"]] if os.environ.get("ROS_PACKAGE_PATH") else [])
)
URDF_PATH = (
    WORKSPACE
    / "src/ridgeback_m0609_description/urdf/ridgeback_m0609.urdf"
)
# Always test navigation with the actively maintained robot model.  The
# nested Rokey_Co3_A1 copy has an older set of configuration layers even
# though its top-level USD has the same filename.
ROBOT_USD = (
    CANONICAL_SERVING_WORKSPACE
    / "assets/mobile_manipulator/ridgeback_m0609_v2.usd"
)
M0609_VISUAL_USD = (
    CANONICAL_SERVING_WORKSPACE
    / "isaacpjt/M0609/Collected_m0609_camera2/m0609_gripper.usd"
)
D455_ASSET_USD = (
    CANONICAL_SERVING_WORKSPACE
    / "isaacpjt/M0609/Collected_m0609_camera2/"
    "omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/"
    "Isaac/5.1/Isaac/Sensors/Intel/RealSense/rsd455.usd"
)
RESTAURANT_USD = (
    WORKSPACE
    / "assets/lightweight_restaurant/lightweight_pizza_restaurant.usda"
)

# Dock the robot's -X face 8 cm from TableSet_00's clear right short edge.
SPAWN_POSITION = Gf.Vec3d(-1.82, -2.20, 0.002)
SPAWN_YAW_DEG = 180.0
TABLE_CAMERA_PATH = (
    "/World/ServingRobot/Robot/ridgeback_base_link/ridgeback_base_link/"
    "fixed_table_depth_camera/realsense_d455/RSD455/Camera_Pseudo_Depth"
)
ARM_JOINTS = [f"joint_{index}" for index in range(1, 7)]
WHEEL_JOINTS = [
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_left_wheel_joint",
    "rear_right_wheel_joint",
]
STOW_CONFIGURATION = [0.0, 0.0, 1.57, 0.0, 1.57, 0.0]
ARM_DRIVE_STIFFNESS = float(os.environ.get("MOBILE_ARM_STIFFNESS", "200000"))
ARM_DRIVE_DAMPING = float(os.environ.get("MOBILE_ARM_DAMPING", "20000"))
ARM_DRIVE_MAX_FORCE = float(os.environ.get("MOBILE_ARM_MAX_FORCE", "10000"))
WHEEL_DRIVE_DAMPING = float(os.environ.get("MOBILE_WHEEL_DAMPING", "1500"))
WHEEL_DRIVE_MAX_FORCE = float(os.environ.get("MOBILE_WHEEL_MAX_FORCE", "2000"))
TABLE_SERVICE_ENABLED = os.environ.get("MOBILE_DEMO_TABLE_SERVICE", "1") == "1"
# Table navigation is the default use of this integrated demo.  Stationary arm
# tests can still explicitly engage the fixed parking joint.
PARKED_HOLD = os.environ.get("MOBILE_DEMO_PARKED_HOLD", "0") == "1"

# Current robot convention: local +X, the arm, and the fixed camera all face
# the served table.  Left tables therefore require yaw=pi and right tables
# yaw=0 at the final dock pose.
TABLE_DOCK_POSES = {
    0: (-1.82, -2.20, math.pi),
    1: (1.82, -2.20, 0.0),
    2: (-1.82, 0.70, math.pi),
    3: (1.82, 0.70, 0.0),
}
WHEEL_RADIUS = 0.0759
WHEEL_BASE_SUM = 0.319 + 0.2755
PRE_DOCK_CLEARANCE = 0.65
PRE_DOCK_POSITION_TOLERANCE = 0.15
TABLE_CAMERA_WIDTH = 1280
TABLE_CAMERA_HEIGHT = 960


def enable_urdf_importer():
    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate("isaacsim.asset.importer.urdf", True)


def import_robot_usd():
    """Validate, but never regenerate or overwrite, the canonical v2 USD."""
    if not ROBOT_USD.is_file():
        raise FileNotFoundError(f"canonical robot USD missing: {ROBOT_USD}")
    print(f"[mobile robot] canonical USD={ROBOT_USD}", flush=True)


def open_restaurant_and_reference_robot():
    if not RESTAURANT_USD.is_file():
        raise FileNotFoundError(RESTAURANT_USD)
    context = omni.usd.get_context()
    if not context.open_stage(str(RESTAURANT_USD)):
        raise RuntimeError(f"failed to open {RESTAURANT_USD}")
    for _ in range(30):
        simulation_app.update()

    stage = context.get_stage()
    spawn = UsdGeom.Xform.Define(stage, "/World/ServingRobot")
    spawn.AddTranslateOp().Set(SPAWN_POSITION)
    spawn.AddRotateZOp().Set(SPAWN_YAW_DEG)
    print(
        f"[mobile robot] spawn=({SPAWN_POSITION[0]:.2f}, "
        f"{SPAWN_POSITION[1]:.2f}) yaw={SPAWN_YAW_DEG:.1f}deg",
        flush=True,
    )
    robot = UsdGeom.Xform.Define(stage, "/World/ServingRobot/Robot")
    # Isaac's URDF importer does not author a defaultPrim on this layered USD,
    # so reference its known robot root explicitly.
    robot.GetPrim().GetReferences().AddReference(
        str(ROBOT_USD), Sdf.Path("/ridgeback_m0609")
    )
    for _ in range(5):
        simulation_app.update()
    composed_names = {
        prim.GetName()
        for prim in stage.Traverse()
        if str(prim.GetPath()).startswith("/World/ServingRobot/Robot/")
    }
    required_sliding_trays = {
        "upper_tray_left_slide_joint",
        "upper_tray_right_slide_joint",
    }
    missing = required_sliding_trays - composed_names
    if missing:
        raise RuntimeError(
            "obsolete fixed/notched robot USD loaded; missing sliding tray "
            f"joints={sorted(missing)} usd={ROBOT_USD}"
        )
    print("[mobile robot] verified latest sliding-tray USD", flush=True)
    return stage


def attach_m0609_visuals(stage):
    """Attach the proven M0609 USD visuals to the integrated URDF links.

    Isaac Sim 5.1 creates the DAE-based arm joints from the combined URDF but
    leaves their visual scopes empty.  Referencing only the visual scopes from
    the project's known-good M0609 USD preserves the single mobile-manipulator
    articulation and makes each mesh follow its corresponding joint.
    """
    if not M0609_VISUAL_USD.is_file():
        raise FileNotFoundError(M0609_VISUAL_USD)

    wanted = ["base_link", *(f"link_{index}" for index in range(1, 7))]
    attached = []
    for link_name in wanted:
        matches = [
            prim
            for prim in stage.Traverse()
            if prim.GetName() == link_name
            and str(prim.GetPath()).startswith("/World/ServingRobot")
        ]
        if len(matches) != 1:
            raise RuntimeError(f"expected one integrated {link_name}, got {matches}")
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
        f"[mobile robot] attached M0609 visuals from {M0609_VISUAL_USD} "
        f"links={len(attached)}",
        flush=True,
    )

    # The RealSense is a sibling assembly in the collected USD rather than a
    # child of link_6.  Mount its complete sensor/visual subtree below the
    # integrated RG2 angle bracket so it follows the arm without adding a
    # second M0609 articulation.
    brackets = [
        prim
        for prim in stage.Traverse()
        if prim.GetName() == "rg2_angle_bracket"
        and str(prim.GetPath()).startswith("/World/ServingRobot")
    ]
    if len(brackets) != 1:
        raise RuntimeError(f"expected one RG2 angle bracket, got {brackets}")
    source_stage = Usd.Stage.Open(str(M0609_VISUAL_USD))
    source_mount_path = Sdf.Path(
        "/World/m0609/onrobot_rg2ft/angle_bracket/realsense_d455"
    )
    source_mount = source_stage.GetPrimAtPath(source_mount_path)
    if not source_mount.IsValid():
        raise RuntimeError(f"RealSense source mount missing: {source_mount_path}")

    camera_mount_path = brackets[0].GetPath().AppendChild("realsense_d455")
    camera_mount = UsdGeom.Xform.Define(stage, camera_mount_path)
    camera_mount.MakeMatrixXform().Set(
        UsdGeom.Xformable(source_mount).GetLocalTransformation()
    )
    rsd_path = camera_mount_path.AppendChild("RSD455")
    UsdGeom.Xform.Define(stage, rsd_path)
    source_rsd_path = source_mount_path.AppendChild("RSD455")

    # Reference render geometry and individual cameras, excluding the source
    # rigid-body/tensor configuration whose relationship paths are not valid
    # after re-parenting into this articulation.
    children = [
        "Visual",
        "Camera_Pseudo_Depth",
        "Camera_OmniVision_OV9782_Color",
        "Camera_OmniVision_OV9782_Left",
        "Camera_OmniVision_OV9782_Right",
    ]
    for child_name in children:
        target = stage.OverridePrim(rsd_path.AppendChild(child_name))
        target.GetReferences().SetReferences(
            [
                Sdf.Reference(
                    str(M0609_VISUAL_USD),
                    source_rsd_path.AppendChild(child_name),
                )
            ]
        )
    print(
        "[mobile robot] attached RG2-mounted RealSense D455 "
        "(visual + 4 camera prims)",
        flush=True,
    )


def attach_fixed_table_depth_camera(stage):
    """Add a fixed D455-class overview camera for table/hand detection."""
    # Avoid traversing immediately after composing the external RealSense
    # payload; Isaac Sim 5.1 can terminate while expanding that payload.
    base_path = Sdf.Path(
        "/World/ServingRobot/Robot/ridgeback_base_link/ridgeback_base_link"
    )
    if not stage.GetPrimAtPath(base_path).IsValid():
        raise RuntimeError(f"Ridgeback base is missing: {base_path}")
    assembly_path = base_path.AppendChild("fixed_table_depth_camera")
    UsdGeom.Xform.Define(stage, assembly_path)

    # Match the current robot convention: local +X is forward and the camera
    # mast sits on robot-left (+Y), clear of the arm's forward workspace.
    mast = UsdGeom.Cylinder.Define(stage, assembly_path.AppendChild("mast"))
    mast.CreateRadiusAttr(0.018)
    mast.CreateHeightAttr(0.935)
    mast.CreateAxisAttr(UsdGeom.Tokens.z)
    mast.AddTranslateOp().Set(Gf.Vec3f(-0.25, 0.285, 1.3225))
    mast.CreateDisplayColorAttr([Gf.Vec3f(0.12, 0.15, 0.18)])
    UsdPhysics.CollisionAPI.Apply(mast.GetPrim())

    # A short lateral boom moves the optical axis outside the arm silhouette.
    boom = UsdGeom.Cylinder.Define(stage, assembly_path.AppendChild("boom"))
    boom.CreateRadiusAttr(0.018)
    boom.CreateHeightAttr(0.215)
    boom.CreateAxisAttr(UsdGeom.Tokens.z)
    boom.AddTranslateOp().Set(Gf.Vec3f(-0.25, 0.3925, 1.79))
    boom.AddRotateXOp().Set(90.0)
    boom.CreateDisplayColorAttr([Gf.Vec3f(0.12, 0.15, 0.18)])
    UsdPhysics.CollisionAPI.Apply(boom.GetPrim())

    # Aim along the current robot front (+X) at table height while retaining a
    # useful top-down view of the arm and delivery surface.
    camera_position = Gf.Vec3d(-0.25, 0.50, 1.85)
    table_target = Gf.Vec3d(1.00, 0.15, 0.74)
    desired_camera_to_base = Gf.Matrix4d().SetLookAt(
        camera_position, table_target, Gf.Vec3d(0.0, 0.0, 1.0)
    ).GetInverse()

    # D455 geometry is modeled around a -X optical axis, while a USD Camera
    # looks along -Z.  Preserve the original color-camera transform so the
    # long camera body faces the same direction as its optical image.
    d455_stage = Usd.Stage.Open(str(D455_ASSET_USD))
    source_color_camera = d455_stage.GetPrimAtPath(
        "/Root/RSD455/Camera_OmniVision_OV9782_Color"
    )
    if not source_color_camera.IsValid():
        raise RuntimeError("D455 source color camera is missing")
    camera_to_sensor = UsdGeom.Xformable(
        source_color_camera
    ).GetLocalTransformation()
    source_rsd = d455_stage.GetPrimAtPath("/Root/RSD455")
    rsd_to_sensor = UsdGeom.Xformable(source_rsd).GetLocalTransformation()
    camera_to_outer_mount = camera_to_sensor * rsd_to_sensor
    sensor_to_base = (
        camera_to_outer_mount.GetInverse() * desired_camera_to_base
    )
    sensor_path = assembly_path.AppendChild("realsense_d455")
    sensor_mount = UsdGeom.Xform.Define(stage, sensor_path)
    sensor_mount.MakeMatrixXform().Set(sensor_to_base)

    # Reference the complete D455 scope so Looks and lens materials remain
    # inside the reference namespace.  Physics is disabled because this sensor
    # is rigidly carried by the Ridgeback base articulation.
    rsd_path = sensor_path.AppendChild("RSD455")
    rsd_prim = stage.OverridePrim(rsd_path)
    rsd_prim.GetReferences().SetReferences(
        [Sdf.Reference(str(D455_ASSET_USD), Sdf.Path("/Root/RSD455"))]
    )
    for api_schema in (
        UsdPhysics.RigidBodyAPI,
        UsdPhysics.MassAPI,
        PhysxSchema.PhysxRigidBodyAPI,
    ):
        if rsd_prim.HasAPI(api_schema):
            rsd_prim.RemoveAPI(api_schema)
    for child in Usd.PrimRange(rsd_prim):
        if child.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(child).CreateCollisionEnabledAttr(False)

    # The plate is behind the D455 (+X in its native body frame), leaving the
    # lens face and its complete field of view unobstructed.
    bracket = UsdGeom.Cube.Define(stage, rsd_path.AppendChild("MountPlate"))
    bracket.CreateSizeAttr(1.0)
    bracket.AddTranslateOp().Set(Gf.Vec3f(0.045, 0.0, -0.012))
    bracket.AddScaleOp().Set(Gf.Vec3f(0.012, 0.095, 0.025))
    bracket.CreateDisplayColorAttr([Gf.Vec3f(0.08, 0.09, 0.10)])
    UsdPhysics.CollisionAPI.Apply(bracket.GetPrim())

    depth_camera = UsdGeom.Camera.Get(
        stage, rsd_path.AppendChild("Camera_Pseudo_Depth")
    )
    if not depth_camera.GetPrim().IsValid():
        raise RuntimeError("D455 pseudo-depth camera is missing")
    depth_camera.CreateFocalLengthAttr(1.93)
    depth_camera.CreateHorizontalApertureAttr(3.896)
    # Match the 4:3 render product so fx and fy stay equal in CameraInfo.
    depth_camera.CreateVerticalApertureAttr(2.922)
    depth_camera.CreateClippingRangeAttr(Gf.Vec2f(0.15, 4.0))
    print(
        "[mobile robot] fixed table depth camera height=1.85m "
        "target=(-1.00, -0.15, 0.74) docking=-X",
        flush=True,
    )


def connect_table_camera_ros2(stage):
    """Publish the fixed camera's RGB, depth, and calibration on ROS 2."""
    if not stage.GetPrimAtPath(TABLE_CAMERA_PATH).IsValid():
        raise RuntimeError(f"table camera is missing: {TABLE_CAMERA_PATH}")

    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
    for _ in range(3):
        simulation_app.update()

    keys = og.Controller.Keys
    og.Controller.edit(
        {
            "graph_path": "/World/ServingRobot/TableCameraROS2",
            "evaluator_name": "execution",
        },
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("RGBPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("DepthPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CameraInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.SET_VALUES: [
                (
                    "RenderProduct.inputs:cameraPrim",
                    [usdrt.Sdf.Path(TABLE_CAMERA_PATH)],
                ),
                ("RenderProduct.inputs:width", TABLE_CAMERA_WIDTH),
                ("RenderProduct.inputs:height", TABLE_CAMERA_HEIGHT),
                ("RGBPublish.inputs:nodeNamespace", "serving_robot/table_camera"),
                ("RGBPublish.inputs:topicName", "color/image_raw"),
                ("RGBPublish.inputs:frameId", "table_camera_optical_frame"),
                ("RGBPublish.inputs:frameSkipCount", 1),
                ("RGBPublish.inputs:type", "rgb"),
                ("DepthPublish.inputs:nodeNamespace", "serving_robot/table_camera"),
                ("DepthPublish.inputs:topicName", "depth/image_raw"),
                ("DepthPublish.inputs:frameId", "table_camera_optical_frame"),
                ("DepthPublish.inputs:frameSkipCount", 1),
                ("DepthPublish.inputs:type", "depth"),
                (
                    "CameraInfoPublish.inputs:nodeNamespace",
                    "serving_robot/table_camera",
                ),
                ("CameraInfoPublish.inputs:topicName", "camera_info"),
                (
                    "CameraInfoPublish.inputs:frameId",
                    "table_camera_optical_frame",
                ),
                ("CameraInfoPublish.inputs:frameSkipCount", 1),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "RenderProduct.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "RGBPublish.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "DepthPublish.inputs:execIn"),
                (
                    "RenderProduct.outputs:execOut",
                    "CameraInfoPublish.inputs:execIn",
                ),
                (
                    "RenderProduct.outputs:renderProductPath",
                    "RGBPublish.inputs:renderProductPath",
                ),
                (
                    "RenderProduct.outputs:renderProductPath",
                    "DepthPublish.inputs:renderProductPath",
                ),
                (
                    "RenderProduct.outputs:renderProductPath",
                    "CameraInfoPublish.inputs:renderProductPath",
                ),
            ],
        },
    )
    print(
        "[table camera ROS2] /serving_robot/table_camera/{color/image_raw,"
        f"depth/image_raw,camera_info}} "
        f"{TABLE_CAMERA_WIDTH}x{TABLE_CAMERA_HEIGHT}",
        flush=True,
    )


def open_table_camera_preview():
    if HEADLESS:
        return
    create_viewport_for_camera(
        "Table Camera",
        TABLE_CAMERA_PATH,
        width=TABLE_CAMERA_WIDTH,
        height=TABLE_CAMERA_HEIGHT,
        position_x=760,
        position_y=80,
    )
    print("[table camera] preview viewport opened", flush=True)


def find_articulation_root(stage):
    roots = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
        and str(prim.GetPath()).startswith("/World/ServingRobot")
    ]
    if len(roots) != 1:
        raise RuntimeError(f"expected one serving-robot articulation, got {roots}")
    return roots[0]


def add_parking_brake(stage, articulation_path):
    """Fix the floating base to the world for stationary arm demonstrations.

    Set MOBILE_DEMO_PARKED_HOLD=0 when wheel navigation is being tested.
    """
    if not PARKED_HOLD:
        print("[mobile robot] parking brake=off (navigation mode)", flush=True)
        return
    joint = UsdPhysics.FixedJoint.Define(
        stage, "/World/ServingRobot/ParkingBrake"
    )
    joint.CreateBody1Rel().SetTargets([Sdf.Path(articulation_path)])
    joint.CreateLocalPos0Attr().Set(SPAWN_POSITION)
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    print("[mobile robot] parking brake=on (fixed base)", flush=True)


def configure_physics_stability(stage, articulation_path):
    scene_prim = stage.GetPrimAtPath("/World/PhysicsScene")
    if not scene_prim.IsValid():
        raise RuntimeError("restaurant PhysicsScene is missing")
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene_prim)
    physx_scene.CreateEnableStabilizationAttr(True)
    # The kitchen contains many legacy triangle-mesh colliders.  CPU PhysX is
    # more robust for this mixed scene and leaves the GPU to the RTX viewport.
    physx_scene.CreateEnableGPUDynamicsAttr(False)
    physx_scene.CreateBroadphaseTypeAttr("MBP")
    physx_scene.CreateTimeStepsPerSecondAttr(120)

    articulation_api = PhysxSchema.PhysxArticulationAPI.Apply(
        stage.GetPrimAtPath(articulation_path)
    )
    articulation_api.CreateSolverPositionIterationCountAttr(64)
    articulation_api.CreateSolverVelocityIterationCountAttr(16)
    articulation_api.CreateStabilizationThresholdAttr(0.01)
    articulation_api.CreateSleepThresholdAttr(0.5)

    # A cylindrical approximation of four mecanum wheels can generate tiny,
    # opposing contact impulses on a flat floor.  Damp those impulses at the
    # mobile base instead of masking them with excessively stiff arm drives.
    base_prim = stage.GetPrimAtPath(articulation_path)
    rigid_body_api = PhysxSchema.PhysxRigidBodyAPI.Apply(base_prim)
    rigid_body_api.CreateLinearDampingAttr(5.0)
    rigid_body_api.CreateAngularDampingAttr(10.0)
    rigid_body_api.CreateMaxDepenetrationVelocityAttr(0.2)
    print(
        "[mobile robot] physics=CPU/120Hz stabilization=on solver=64/16",
        flush=True,
    )


def configure_joint_drives(stage):
    configured_arm = []
    configured_wheels = []
    for prim in stage.Traverse():
        name = prim.GetName()
        if name in ARM_JOINTS:
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            drive.CreateStiffnessAttr(ARM_DRIVE_STIFFNESS)
            drive.CreateDampingAttr(ARM_DRIVE_DAMPING)
            drive.CreateMaxForceAttr(ARM_DRIVE_MAX_FORCE)
            configured_arm.append(name)
        elif name in WHEEL_JOINTS:
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            drive.CreateStiffnessAttr(0.0)
            drive.CreateDampingAttr(WHEEL_DRIVE_DAMPING)
            drive.CreateMaxForceAttr(WHEEL_DRIVE_MAX_FORCE)
            drive.CreateTargetVelocityAttr(0.0)
            angle = 45.0 if name in {
                "front_left_wheel_joint",
                "rear_right_wheel_joint",
            } else -45.0
            prim.CreateAttribute(
                "isaacmecanumwheel:radius", Sdf.ValueTypeNames.Float
            ).Set(0.0759)
            prim.CreateAttribute(
                "isaacmecanumwheel:angle", Sdf.ValueTypeNames.Float
            ).Set(angle)
            configured_wheels.append(name)

    if set(configured_arm) != set(ARM_JOINTS):
        raise RuntimeError(f"arm drive setup incomplete: {configured_arm}")
    if set(configured_wheels) != set(WHEEL_JOINTS):
        raise RuntimeError(f"wheel drive setup incomplete: {configured_wheels}")


def initialize_robot(articulation_path):
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    # One frame is enough for PhysX handles.  Waiting longer lets the arm move
    # toward the importer-default zero targets before its stow target is set.
    for _ in range(1):
        simulation_app.update()

    # This demo controls one robot, so use the Isaac Sim 5.1 single-prim
    # wrapper.  The generic Articulation view returns batched (1, N) arrays.
    articulation = SingleArticulation(
        prim_path=articulation_path, name="ridgeback_m0609"
    )
    articulation.initialize()
    if not articulation.handles_initialized:
        raise RuntimeError(f"invalid articulation handle: {articulation_path}")
    # The combined shelf/arm has several fixed collision bodies very close to
    # one another.  Adjacent-link contacts are not useful here and can keep the
    # complete articulation awake indefinitely.
    articulation.set_enabled_self_collisions(False)
    articulation.set_sleep_threshold(0.5)

    dof_names = list(articulation.dof_names)
    expected = set(ARM_JOINTS + WHEEL_JOINTS)
    missing = expected - set(dof_names)
    if missing:
        raise RuntimeError(f"missing articulation DOFs: {sorted(missing)}")

    positions = articulation.get_joint_positions()
    for name, value in zip(ARM_JOINTS, STOW_CONFIGURATION):
        positions[dof_names.index(name)] = value
    for name in WHEEL_JOINTS:
        positions[dof_names.index(name)] = 0.0
    articulation.set_joint_positions(positions)
    articulation.set_joint_velocities(np.zeros(len(dof_names), dtype=float))
    arm_indices = np.asarray(
        [dof_names.index(name) for name in ARM_JOINTS], dtype=np.int32
    )
    articulation.apply_action(
        ArticulationAction(
            joint_positions=np.asarray(STOW_CONFIGURATION, dtype=float),
            joint_indices=arm_indices,
        )
    )
    print(
        f"[ready] serving robot articulation={articulation_path} "
        f"dofs={dof_names}",
        flush=True,
    )
    return articulation, dof_names


def print_arm_visual_bounds(stage):
    """Print composed world bounds to catch missing or misplaced arm meshes."""
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]
    )
    wanted = {"base_link", *(f"link_{index}" for index in range(1, 7))}
    for prim in stage.Traverse():
        if prim.GetName() not in wanted and not prim.GetName().startswith("rg2_") \
                and prim.GetName() != "realsense_d455":
            continue
        bounds = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        print(
            f"[arm bounds] {prim.GetName()} "
            f"min={tuple(round(v, 4) for v in bounds.GetMin())} "
            f"max={tuple(round(v, 4) for v in bounds.GetMax())}",
            flush=True,
        )


def run_optional_diagnostic(articulation, dof_names):
    if os.environ.get("MOBILE_DEMO_AUTORUN", "0") != "1":
        return

    wheel_indices = [dof_names.index(name) for name in WHEEL_JOINTS]
    arm_indices = [dof_names.index(name) for name in ARM_JOINTS]
    print("[diagnostic] wheel rotation and arm stow test", flush=True)
    for frame in range(360):
        velocity = np.zeros(len(dof_names), dtype=float)
        if 120 <= frame < 240:
            velocity[wheel_indices] = [1.2, 1.2, 1.2, 1.2]
        articulation.apply_action(
            ArticulationAction(
                joint_velocities=velocity,
                joint_indices=np.arange(len(dof_names), dtype=np.int32),
            )
        )
        if frame == 300:
            target = articulation.get_joint_positions()
            target[arm_indices] = np.asarray(STOW_CONFIGURATION)
            articulation.set_joint_position_targets(
                target[arm_indices], joint_indices=np.asarray(arm_indices)
            )
        simulation_app.update()


def run_stability_check(articulation, dof_names):
    steps = int(os.environ.get("MOBILE_DEMO_STABILITY_STEPS", "0"))
    if steps <= 0:
        return
    arm_indices = [dof_names.index(name) for name in ARM_JOINTS]
    max_arm_speed = 0.0
    max_base_speed = 0.0
    for frame in range(steps):
        simulation_app.update()
        if frame < steps // 2:
            continue
        joint_velocities = articulation.get_joint_velocities()
        max_arm_speed = max(
            max_arm_speed,
            float(np.max(np.abs(joint_velocities[arm_indices]))),
        )
        max_base_speed = max(
            max_base_speed,
            float(np.linalg.norm(articulation.get_linear_velocity())),
        )
    print(
        f"[stability] sample_frames={steps - steps // 2} "
        f"max_arm_speed={max_arm_speed:.6f}rad/s "
        f"max_base_speed={max_base_speed:.6f}m/s "
        f"final_arm_speed={float(np.max(np.abs(articulation.get_joint_velocities()[arm_indices]))):.6f}rad/s "
        f"final_base_velocity={tuple(round(float(v), 6) for v in articulation.get_linear_velocity())} "
        f"final_base_position={tuple(round(float(v), 6) for v in articulation.get_world_pose()[0])}",
        flush=True,
    )


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(quaternion):
    """Return world Z yaw from Isaac's scalar-first (w, x, y, z) quaternion."""
    w, x, y, z = (float(value) for value in quaternion)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


class TableNavigationServer:
    """ROS service and two-stage pre-dock/final-dock Ridgeback controller."""

    POSITION_TOLERANCE = 0.025
    YAW_TOLERANCE = math.radians(2.0)
    HEADING_ENTER_DRIVE = math.radians(3.0)
    HEADING_LEAVE_DRIVE = math.radians(12.0)
    MAX_LINEAR_SPEED = 0.35
    MAX_DOCK_SPEED = 0.12
    MAX_ANGULAR_SPEED = 0.65
    MAX_WHEEL_SPEED = 8.0

    def __init__(self, articulation, dof_names):
        if not rclpy.ok():
            rclpy.init(args=[])
        self.node = rclpy.create_node("serving_robot_table_navigation")
        self.service = self.node.create_service(
            AddTwoInts,
            "/serving_robot/go_to_table",
            self._handle_request,
        )
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = threading.Thread(
            target=self.executor.spin,
            name="table_navigation_ros_executor",
            daemon=True,
        )
        self.executor_thread.start()
        self.articulation = articulation
        self.wheel_indices = np.asarray(
            [dof_names.index(name) for name in WHEEL_JOINTS], dtype=np.int32
        )
        self.target_table = None
        self.navigation_stage = None
        self._last_phase = None
        self._path_aligned = False
        print(
            "[table navigation] service=/serving_robot/go_to_table "
            "type=example_interfaces/srv/AddTwoInts request.a=table_id(0..3)",
            flush=True,
        )

    def _handle_request(self, request, response):
        table_id = int(request.a)
        if table_id not in TABLE_DOCK_POSES:
            response.sum = -1
            self.node.get_logger().warning(
                f"rejected table_id={table_id}; expected 0, 1, 2, or 3"
            )
            return response
        if PARKED_HOLD:
            response.sum = -2
            self.node.get_logger().error(
                "parking brake is engaged; launch with "
                "MOBILE_DEMO_PARKED_HOLD=0"
            )
            return response

        self.target_table = table_id
        self.navigation_stage = "move_to_pre_dock"
        self._last_phase = None
        self._path_aligned = False
        response.sum = table_id
        goal = TABLE_DOCK_POSES[table_id]
        self.node.get_logger().info(
            f"accepted table={table_id} goal="
            f"({goal[0]:.2f}, {goal[1]:.2f}, {math.degrees(goal[2]):.0f} deg)"
        )
        return response

    def _apply_base_velocity(self, linear_x, angular_z):
        # Ridgeback wheel order: FL, FR, RL, RR.  Only longitudinal motion and
        # yaw are used so the controller remains valid with the conservative
        # cylindrical wheel collision approximation in this scene.
        turn = WHEEL_BASE_SUM * angular_z
        wheel_velocities = np.asarray(
            [
                (linear_x - turn) / WHEEL_RADIUS,
                (linear_x + turn) / WHEEL_RADIUS,
                (linear_x - turn) / WHEEL_RADIUS,
                (linear_x + turn) / WHEEL_RADIUS,
            ],
            dtype=float,
        )
        wheel_velocities = np.clip(
            wheel_velocities, -self.MAX_WHEEL_SPEED, self.MAX_WHEEL_SPEED
        )
        self.articulation.apply_action(
            ArticulationAction(
                joint_velocities=wheel_velocities,
                joint_indices=self.wheel_indices,
            )
        )

    def update(self):
        if self.target_table is None:
            self._apply_base_velocity(0.0, 0.0)
            return

        position, orientation = self.articulation.get_world_pose()
        x, y = float(position[0]), float(position[1])
        yaw = quaternion_to_yaw(orientation)
        goal_x, goal_y, goal_yaw = TABLE_DOCK_POSES[self.target_table]
        # Pre-dock lies behind the final pose along local -X.  The robot turns
        # in the clear aisle, then drives straight forward with its +X camera
        # and arm side facing the table.
        pre_x = goal_x - PRE_DOCK_CLEARANCE * math.cos(goal_yaw)
        pre_y = goal_y - PRE_DOCK_CLEARANCE * math.sin(goal_yaw)

        if self.navigation_stage == "move_to_pre_dock":
            dx, dy = pre_x - x, pre_y - y
            distance = math.hypot(dx, dy)
            desired_heading = math.atan2(dy, dx)
            heading_error = normalize_angle(desired_heading - yaw)
            if self._path_aligned:
                if abs(heading_error) > self.HEADING_LEAVE_DRIVE:
                    self._path_aligned = False
            elif abs(heading_error) < self.HEADING_ENTER_DRIVE:
                self._path_aligned = True

            if not self._path_aligned:
                phase = "rotate_to_path"
                linear_x = 0.0
                angular_z = float(
                    np.clip(1.8 * heading_error, -self.MAX_ANGULAR_SPEED,
                            self.MAX_ANGULAR_SPEED)
                )
            else:
                phase = "drive"
                linear_x = min(self.MAX_LINEAR_SPEED, max(0.08, 0.8 * distance))
                angular_z = float(
                    np.clip(1.2 * heading_error, -self.MAX_ANGULAR_SPEED,
                            self.MAX_ANGULAR_SPEED)
                )
            # The old 2.5 cm threshold was smaller than the stopping distance
            # at the controller's 0.08 m/s minimum speed.  It overshot the
            # point, then circled back repeatedly.  Pre-dock only needs to be
            # inside the clear turning area; final docking remains precise.
            if distance <= PRE_DOCK_POSITION_TOLERANCE:
                self.navigation_stage = "align_at_pre_dock"
                self._path_aligned = False
                phase = "pre_dock_reached"
                linear_x = 0.0
                angular_z = 0.0

        elif self.navigation_stage == "align_at_pre_dock":
            distance = math.hypot(pre_x - x, pre_y - y)
            yaw_error = normalize_angle(goal_yaw - yaw)
            if abs(yaw_error) > self.YAW_TOLERANCE:
                phase = "align_at_pre_dock"
                linear_x = 0.0
                angular_z = float(
                    np.clip(1.8 * yaw_error, -self.MAX_ANGULAR_SPEED,
                            self.MAX_ANGULAR_SPEED)
                )
            else:
                self.navigation_stage = "final_approach"
                phase = "start_final_approach"
                linear_x = 0.0
                angular_z = 0.0

        else:
            dx, dy = goal_x - x, goal_y - y
            distance = math.hypot(dx, dy)
            yaw_error = normalize_angle(goal_yaw - yaw)
            # Express residual position in the desired dock frame.  A small
            # lateral correction is allowed, but large yaw error is corrected
            # while stopped instead of turning beside the table.
            lateral_error = (
                -math.sin(goal_yaw) * dx + math.cos(goal_yaw) * dy
            )
            if distance > self.POSITION_TOLERANCE:
                phase = "final_forward_approach"
                if abs(yaw_error) > self.HEADING_LEAVE_DRIVE:
                    linear_x = 0.0
                else:
                    linear_x = min(
                        self.MAX_DOCK_SPEED, max(0.035, 0.5 * distance)
                    )
                angular_z = float(
                    np.clip(
                        1.8 * yaw_error - 1.2 * lateral_error,
                        -0.30,
                        0.30,
                    )
                )
            elif abs(yaw_error) > self.YAW_TOLERANCE:
                # Normally the pre-dock alignment makes this correction tiny.
                # Keep the rate low to avoid a large collision sweep.
                phase = "fine_align_at_table"
                linear_x = 0.0
                angular_z = float(np.clip(1.2 * yaw_error, -0.15, 0.15))
            else:
                phase = "arrived"
                linear_x = 0.0
                angular_z = 0.0
                self.node.get_logger().info(
                    f"arrived table={self.target_table} pose="
                    f"({x:.3f}, {y:.3f}, {math.degrees(yaw):.1f} deg)"
                )
                self.target_table = None
                self.navigation_stage = None

        if phase != self._last_phase:
            self.node.get_logger().info(
                f"navigation phase={phase} distance={distance:.3f} m"
            )
            self._last_phase = phase
        self._apply_base_velocity(linear_x, angular_z)

    def shutdown(self):
        self._apply_base_velocity(0.0, 0.0)
        self.executor.shutdown()
        self.executor_thread.join(timeout=2.0)
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    enable_urdf_importer()
    import_robot_usd()
    stage = open_restaurant_and_reference_robot()
    attach_m0609_visuals(stage)
    attach_fixed_table_depth_camera(stage)
    connect_table_camera_ros2(stage)
    for _ in range(10):
        simulation_app.update()
    configure_joint_drives(stage)
    articulation_path = find_articulation_root(stage)
    add_parking_brake(stage, articulation_path)
    configure_physics_stability(stage, articulation_path)
    articulation, dof_names = initialize_robot(articulation_path)
    open_table_camera_preview()
    if os.environ.get("MOBILE_DEMO_PRINT_BOUNDS", "0") == "1":
        print_arm_visual_bounds(stage)
    run_optional_diagnostic(articulation, dof_names)
    run_stability_check(articulation, dof_names)
    table_navigation = (
        TableNavigationServer(articulation, dof_names)
        if TABLE_SERVICE_ENABLED
        else None
    )

    if os.environ.get("MOBILE_DEMO_EXIT_AFTER_READY", "0") == "1":
        if table_navigation is not None:
            table_navigation.shutdown()
        simulation_app.close()
        return

    try:
        while simulation_app.is_running():
            if table_navigation is not None:
                table_navigation.update()
            simulation_app.update()
            # Keep the GUI event loop responsive without throttling it to the
            # old uneven 16 ms cadence.
            time.sleep(0.010)
    finally:
        if table_navigation is not None:
            table_navigation.shutdown()


try:
    main()
except BaseException:
    traceback.print_exc()
    raise
finally:
    simulation_app.close()
