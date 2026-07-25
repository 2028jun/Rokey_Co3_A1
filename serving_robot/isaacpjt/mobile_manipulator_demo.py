"""Spawn the Ridgeback + serving shelf + M0609 robot in the restaurant.

This script targets Isaac Sim 5.1.0-rc.19.  It is intentionally not launched
by the ROS package build.  Run it only when the Isaac demonstration is needed.
Set MOBILE_DEMO_AUTORUN=1 to enable the short wheel/arm diagnostic sequence.
"""

import os
import sys
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
if os.environ.get("MOBILE_DEMO_ROS_CAMERA", "1") == "1":
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


WORKSPACE = Path(
    os.environ.get("COBOT3_WS", Path(__file__).resolve().parents[1])
).resolve()
# Isaac's URDF importer resolves package:// URLs through ROS_PACKAGE_PATH,
# while a sourced ROS 2/ament workspace does not populate that ROS 1 variable.
_package_roots = [
    WORKSPACE / "install/m0609_isaac_description/share",
    WORKSPACE / "install/ridgeback_m0609_description/share",
    WORKSPACE.parent / "install/m0609_isaac_description/share",
    WORKSPACE.parent / "install/ridgeback_m0609_description/share",
]
os.environ["ROS_PACKAGE_PATH"] = ":".join(
    [str(path) for path in _package_roots if path.is_dir()]
    + ([os.environ["ROS_PACKAGE_PATH"]] if os.environ.get("ROS_PACKAGE_PATH") else [])
)
URDF_PATH = (
    WORKSPACE
    / "src/ridgeback_m0609_description/urdf/ridgeback_m0609.urdf"
)
ROBOT_USD = WORKSPACE / "assets/mobile_manipulator/ridgeback_m0609_v2.usd"
M0609_VISUAL_USD = (
    WORKSPACE / "isaacpjt/M0609/Collected_m0609_camera2/m0609_gripper.usd"
)
D455_ASSET_USD = (
    WORKSPACE
    / "isaacpjt/M0609/Collected_m0609_camera2/"
    "omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/"
    "Isaac/5.1/Isaac/Sensors/Intel/RealSense/rsd455.usd"
)
RESTAURANT_USD = (
    WORKSPACE
    / "assets/lightweight_restaurant/lightweight_pizza_restaurant.usda"
)

# Keep the robot on the corridor side and rotate it so local +X/Nav2-forward
# and the front-mounted arm face TableSet_00.
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
SLIDING_TRAY_JOINTS = [
    "upper_tray_left_slide_joint",
    "upper_tray_right_slide_joint",
]
STOW_CONFIGURATION = [0.0, 0.0, 1.57, 0.0, 1.57, 0.0]
ARM_DRIVE_STIFFNESS = float(os.environ.get("MOBILE_ARM_STIFFNESS", "200000"))
ARM_DRIVE_DAMPING = float(os.environ.get("MOBILE_ARM_DAMPING", "20000"))
ARM_DRIVE_MAX_FORCE = float(os.environ.get("MOBILE_ARM_MAX_FORCE", "10000"))
WHEEL_DRIVE_DAMPING = float(os.environ.get("MOBILE_WHEEL_DAMPING", "1500"))
WHEEL_DRIVE_MAX_FORCE = float(os.environ.get("MOBILE_WHEEL_MAX_FORCE", "2000"))
TRAY_DRIVE_STIFFNESS = float(os.environ.get("MOBILE_TRAY_STIFFNESS", "4000"))
TRAY_DRIVE_DAMPING = float(os.environ.get("MOBILE_TRAY_DAMPING", "500"))
TRAY_DRIVE_MAX_FORCE = float(os.environ.get("MOBILE_TRAY_MAX_FORCE", "400"))
PARKED_HOLD = os.environ.get("MOBILE_DEMO_PARKED_HOLD", "1") == "1"


def enable_urdf_importer():
    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate("isaacsim.asset.importer.urdf", True)


def import_robot_usd():
    if not URDF_PATH.is_file():
        raise FileNotFoundError(
            f"generated URDF missing: {URDF_PATH}\n"
            "Run xacro after sourcing the workspace."
        )

    ROBOT_USD.parent.mkdir(parents=True, exist_ok=True)
    if ROBOT_USD.is_file() and os.environ.get("MOBILE_DEMO_REIMPORT", "0") != "1":
        print(f"[mobile robot] reuse USD={ROBOT_USD}", flush=True)
        return
    status, config = omni.kit.commands.execute("URDFCreateImportConfig")
    if not status:
        raise RuntimeError("URDFCreateImportConfig failed")
    config.merge_fixed_joints = False
    config.convex_decomp = False
    config.import_inertia_tensor = True
    config.fix_base = False
    config.collision_from_visuals = False
    config.distance_scale = 1.0

    status, articulation_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=str(URDF_PATH),
        import_config=config,
        dest_path=str(ROBOT_USD),
        get_articulation_root=True,
    )
    if not status or not ROBOT_USD.is_file():
        raise RuntimeError("Ridgeback/M0609 URDF import failed")
    print(
        f"[mobile robot] generated USD={ROBOT_USD} "
        f"articulation={articulation_path}",
        flush=True,
    )


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
    robot = UsdGeom.Xform.Define(stage, "/World/ServingRobot/Robot")
    # Isaac's URDF importer does not author a defaultPrim on this layered USD,
    # so reference its known robot root explicitly.
    robot.GetPrim().GetReferences().AddReference(
        str(ROBOT_USD), Sdf.Path("/ridgeback_m0609")
    )
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

    # Tall mast on the -X side, opposite the +X table docking face.  This
    # lets the camera look over the arm instead of through it.
    mast = UsdGeom.Cylinder.Define(stage, assembly_path.AppendChild("mast"))
    mast.CreateRadiusAttr(0.018)
    mast.CreateHeightAttr(0.935)
    mast.CreateAxisAttr(UsdGeom.Tokens.z)
    mast.AddTranslateOp().Set(Gf.Vec3f(-0.25, -0.285, 1.3225))
    mast.CreateDisplayColorAttr([Gf.Vec3f(0.12, 0.15, 0.18)])
    UsdPhysics.CollisionAPI.Apply(mast.GetPrim())

    # A short lateral boom moves the optical axis outside the arm silhouette.
    boom = UsdGeom.Cylinder.Define(stage, assembly_path.AppendChild("boom"))
    boom.CreateRadiusAttr(0.018)
    boom.CreateHeightAttr(0.215)
    boom.CreateAxisAttr(UsdGeom.Tokens.z)
    boom.AddTranslateOp().Set(Gf.Vec3f(-0.25, -0.3925, 1.79))
    boom.AddRotateXOp().Set(90.0)
    boom.CreateDisplayColorAttr([Gf.Vec3f(0.12, 0.15, 0.18)])
    UsdPhysics.CollisionAPI.Apply(boom.GetPrim())

    # The table docks on the arm side (+X).  Aim over the arm at the center of
    # a 0.74 m-high table while retaining a useful top-down view of hands.
    camera_position = Gf.Vec3d(-0.25, -0.50, 1.85)
    table_target = Gf.Vec3d(1.00, -0.15, 0.74)
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
        "target=(1.00, -0.15, 0.74) docking=+X",
        flush=True,
    )


def connect_table_camera_ros2(stage):
    """Publish the fixed camera's RGB, depth, and calibration on ROS 2."""
    if os.environ.get("MOBILE_DEMO_ROS_CAMERA", "1") != "1":
        print("[table camera ROS2] disabled by MOBILE_DEMO_ROS_CAMERA=0", flush=True)
        return
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
                ("RenderProduct.inputs:width", 640),
                ("RenderProduct.inputs:height", 480),
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
        "depth/image_raw,camera_info} 640x480",
        flush=True,
    )


def open_table_camera_preview():
    if HEADLESS:
        return
    create_viewport_for_camera(
        "Table Camera",
        TABLE_CAMERA_PATH,
        width=640,
        height=480,
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
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(0.0, 0.0, 0.0, 1.0))
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
    configured_trays = []
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
        elif name in SLIDING_TRAY_JOINTS:
            drive = UsdPhysics.DriveAPI.Apply(prim, "linear")
            drive.CreateStiffnessAttr(TRAY_DRIVE_STIFFNESS)
            drive.CreateDampingAttr(TRAY_DRIVE_DAMPING)
            drive.CreateMaxForceAttr(TRAY_DRIVE_MAX_FORCE)
            drive.CreateTargetPositionAttr(0.0)
            configured_trays.append(name)

    if set(configured_arm) != set(ARM_JOINTS):
        raise RuntimeError(f"arm drive setup incomplete: {configured_arm}")
    if set(configured_wheels) != set(WHEEL_JOINTS):
        raise RuntimeError(f"wheel drive setup incomplete: {configured_wheels}")
    if set(configured_trays) != set(SLIDING_TRAY_JOINTS):
        raise RuntimeError(f"sliding tray drive setup incomplete: {configured_trays}")


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
    expected = set(ARM_JOINTS + WHEEL_JOINTS + SLIDING_TRAY_JOINTS)
    missing = expected - set(dof_names)
    if missing:
        raise RuntimeError(f"missing articulation DOFs: {sorted(missing)}")

    positions = articulation.get_joint_positions()
    for name, value in zip(ARM_JOINTS, STOW_CONFIGURATION):
        positions[dof_names.index(name)] = value
    for name in WHEEL_JOINTS:
        positions[dof_names.index(name)] = 0.0
    for name in SLIDING_TRAY_JOINTS:
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

    if os.environ.get("MOBILE_DEMO_EXIT_AFTER_READY", "0") == "1":
        simulation_app.close()
        return

    # Initialize ROS 2 Status Publisher for HMI Integration
    import json
    import rclpy
    from std_msgs.msg import String

    if not rclpy.ok():
        rclpy.init()
    status_node = rclpy.create_node('isaac_robot_status_publisher')
    status_pub = status_node.create_publisher(String, '/serving_robot/status', 10)
    last_status_pub_time = 0.0

    print("[HMI Integration] Real-time Isaac Sim pose publisher active on /serving_robot/status", flush=True)

    while simulation_app.is_running():
        simulation_app.update()
        now = time.time()
        if now - last_status_pub_time > 0.1:  # 10 Hz
            last_status_pub_time = now
            try:
                pos, rot = articulation.get_world_pose()
                w, x, y, z = rot
                siny_cosp = 2 * (w * z + x * y)
                cosy_cosp = 1 - 2 * (y * y + z * z)
                yaw = np.arctan2(siny_cosp, cosy_cosp)

                is_parked = os.environ.get("MOBILE_DEMO_PARKED_HOLD", "1") == "1"
                status_payload = {
                    "pose": {"x": float(pos[0]), "y": float(pos[1]), "yaw": float(yaw)},
                    "state": "READY" if is_parked else "NAVIGATING",
                    "parking_brake": is_parked,
                    "battery": 98.5
                }
                msg = String()
                msg.data = json.dumps(status_payload)
                status_pub.publish(msg)
                rclpy.spin_once(status_node, timeout_sec=0)
            except Exception as e:
                pass
        time.sleep(0.010)


try:
    main()
except BaseException:
    traceback.print_exc()
    raise
finally:
    simulation_app.close()
