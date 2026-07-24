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
    os.environ.setdefault("ROS_DOMAIN_ID", "101")
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

import hand_intrusion_test_actor as hand_test
import actor_sdg_test_actor as actor_sdg

# Pass 9 tried to make "actor_sdg" (a real omni.anim.people character
# driving a typing/sit/push_button cycle -- see actor_sdg_test_actor.py)
# the default, per the reviewer's call to discard the hand-authored
# rigid-arm two-bone-IK mechanism from passes 5-8. Pass 9 hit a reproducible
# omni.anim.graph.core registration failure integrating it into this
# restaurant+robot pipeline; pass 10 root-caused and fixed that (a missing
# settle gap between enabling the extensions and opening the stage -- see
# the comment in main() and enable_extensions()'s docstring) and confirmed
# the full cycle now registers, executes, and loops correctly via real
# command-queue logs. What pass 10 could NOT yet confirm is visual pose
# quality (its own ad hoc QA cameras were unreliable -- wall clipping,
# occlusion -- and a real ~90 degree yaw convention mismatch was found
# between the spawned character's orientation and what
# Utils.convert_to_angle() reads back for the loop-closing GoTo). Until
# that's confirmed by eye, "rigid_arm" -- proven end to end -- stays the
# default; "actor_sdg" is a real, working-mechanically opt-in for whoever
# finishes that visual confirmation next. "legacy" is the pre-pass-5
# whole-body-slide fallback.
HAND_TEST_RIG_MODE = os.environ.get("HAND_TEST_RIG_MODE", "rigid_arm")


WORKSPACE = Path(
    os.environ.get("COBOT3_WS", Path(__file__).resolve().parents[1])
).resolve()
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

# Dock the robot's -X face 8 cm from TableSet_00's clear right short edge.
SPAWN_POSITION = Gf.Vec3d(-1.82, -2.20, 0.002)
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

    # Tall mast on the +X/right side, opposite the -X table docking face.  This
    # lets the camera look over the arm instead of through it.
    mast = UsdGeom.Cylinder.Define(stage, assembly_path.AppendChild("mast"))
    mast.CreateRadiusAttr(0.018)
    mast.CreateHeightAttr(0.935)
    mast.CreateAxisAttr(UsdGeom.Tokens.z)
    mast.AddTranslateOp().Set(Gf.Vec3f(0.25, -0.285, 1.3225))
    mast.CreateDisplayColorAttr([Gf.Vec3f(0.12, 0.15, 0.18)])
    UsdPhysics.CollisionAPI.Apply(mast.GetPrim())

    # A short lateral boom moves the optical axis outside the arm silhouette.
    boom = UsdGeom.Cylinder.Define(stage, assembly_path.AppendChild("boom"))
    boom.CreateRadiusAttr(0.018)
    boom.CreateHeightAttr(0.215)
    boom.CreateAxisAttr(UsdGeom.Tokens.z)
    boom.AddTranslateOp().Set(Gf.Vec3f(0.25, -0.3925, 1.79))
    boom.AddRotateXOp().Set(90.0)
    boom.CreateDisplayColorAttr([Gf.Vec3f(0.12, 0.15, 0.18)])
    UsdPhysics.CollisionAPI.Apply(boom.GetPrim())

    # The table docks on the arm side (-X).  Aim over the arm at the center of
    # a 0.74 m-high table while retaining a useful top-down view of hands.
    camera_position = Gf.Vec3d(0.25, -0.50, 1.85)
    table_target = Gf.Vec3d(-1.00, -0.15, 0.74)
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
                # hand_safety crops just the tabletop ROI out of this frame
                # (~35% width x ~45% height) and upscales that crop to its
                # image_size:=1280 YOLO input. At 640x480 the crop was only
                # ~289x262px, a ~4.4x upscale that threw away real hand
                # detail. 1280x960 keeps the same 4:3 aspect and roughly
                # doubles crop resolution (~579x524px, ~2.2x upscale).
                # Raise further if render cost allows.
                ("RenderProduct.inputs:width", 1280),
                ("RenderProduct.inputs:height", 960),
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
        "depth/image_raw,camera_info} 1280x960",
        flush=True,
    )


def _define_lookat_camera(stage, path, eye, target, up):
    """Define a plain UsdGeom.Camera at `path` looking from `eye` at `target`.

    Mirrors the eye/target/up -> SetLookAt(...).GetInverse() recipe already
    used by attach_fixed_table_depth_camera above (that one composes it
    through several mount-frame transforms; this one applies it directly in
    world space since these are free-standing QA cameras, not robot-mounted).
    """
    camera = UsdGeom.Camera.Define(stage, path)
    camera.CreateFocalLengthAttr(24.0)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 20.0))
    camera_to_world = Gf.Matrix4d().SetLookAt(eye, target, up).GetInverse()
    UsdGeom.Xformable(camera.GetPrim()).MakeMatrixXform().Set(camera_to_world)
    return camera


def capture_pose_frames(stage, reach_animator):
    """Render the person's rig from four angles at several reach-cycle
    progress values and save PNGs, so the visual acceptance criteria in
    VISION_TEST_GPU_PROMPT.txt (body pose / joint realism / cone
    transitions / clipping) can actually be inspected frame by frame
    instead of judged from angle numbers alone.

    Gated behind MOBILE_DEMO_CAPTURE_POSES=1; writes into
    MOBILE_DEMO_CAPTURE_DIR (default /tmp/vision_test_pose_frames) and exits
    before the interactive loop, the same way MOBILE_DEMO_EXIT_AFTER_READY
    does.
    """
    from omni.kit.viewport.utility import capture_viewport_to_file

    out_dir = Path(
        os.environ.get("MOBILE_DEMO_CAPTURE_DIR", "/tmp/vision_test_pose_frames")
    )
    out_dir.mkdir(parents=True, exist_ok=True)

    seat = hand_test.SEAT_POSITION
    target = hand_test.TABLE_HAND_TARGET
    forward = Gf.Vec3d(target[0] - seat[0], target[1] - seat[1], 0.0)
    forward = forward / forward.GetLength()
    right = Gf.Vec3d(-forward[1], forward[0], 0.0)
    aim = seat + Gf.Vec3d(0.0, 0.0, 1.2 * hand_test.PERSON_SCALE)

    cameras = {
        "front": (seat + forward * 2.2 + Gf.Vec3d(0, 0, 1.5), aim, Gf.Vec3d(0, 0, 1)),
        "side": (seat + right * 2.2 + Gf.Vec3d(0, 0, 1.5), aim, Gf.Vec3d(0, 0, 1)),
        "overhead": (seat + Gf.Vec3d(0, 0, 3.0), aim, forward),
    }
    viewports = {}
    for name, (eye, look_target, up) in cameras.items():
        cam_path = f"/World/QACamera_{name}"
        _define_lookat_camera(stage, cam_path, eye, look_target, up)
        window = create_viewport_for_camera(f"QA_{name}", cam_path, width=1280, height=960)
        viewports[name] = window.viewport_api
    table_window = create_viewport_for_camera(
        "QA_table", TABLE_CAMERA_PATH, width=1280, height=960
    )
    viewports["table"] = table_window.viewport_api

    if os.environ.get("MOBILE_DEMO_HIDE_CONES", "0") == "1":
        for cone_name in (
            "ShoulderSleeveTransition",
            "ElbowTransition",
            "ShoulderSleeveTransitionL",
            "ElbowTransitionL",
        ):
            for prim in stage.Traverse():
                if prim.GetName() == cone_name and str(prim.GetPath()).startswith(
                    hand_test.PERSON_PRIM_PATH
                ):
                    UsdGeom.Imageable(prim).MakeInvisible()
                    print(f"[debug] hid {prim.GetPath()}", flush=True)

    def _render_settle(frames=15):
        for _ in range(frames):
            simulation_app.update()

    def _capture(progress, views):
        reach_animator._apply_progress(progress)
        _render_settle()
        for view_name in views:
            path = out_dir / f"progress_{progress:.2f}_{view_name}.png"
            capture_viewport_to_file(viewports[view_name], str(path))
            _render_settle(5)
            print(f"[capture] wrote {path}", flush=True)

    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])

    def _print_bboxes(label_prefix):
        for label, path in [
            ("ShoulderSphere", f"{hand_test.ARM_RIG_PATH}/ShoulderSleeveTransition"),
            ("ElbowSphere", f"{hand_test.ELBOW_PATH}/ElbowTransition"),
            ("UpperArm_Skin", f"{hand_test.ARM_RIG_PATH}/UpperArm_Skin"),
            ("UpperArm_Tshirt", f"{hand_test.ARM_RIG_PATH}/UpperArm_Tshirt"),
        ]:
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid():
                print(f"[capture][bbox][{label_prefix}] {label}: prim not found at {path}", flush=True)
                continue
            world_bbox = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
            print(
                f"[capture][bbox][{label_prefix}] {label}: min={tuple(world_bbox.GetMin())} max={tuple(world_bbox.GetMax())}",
                flush=True,
            )

    _capture(0.0, ["front", "side", "overhead", "table"])
    _print_bboxes("rest")
    _capture(0.25, ["table"])
    _capture(0.5, ["table"])
    _capture(0.75, ["table"])
    _capture(1.0, ["front", "side", "overhead", "table"])
    _print_bboxes("full_reach")

    for label, path in [
        ("TableSet_00", "/World/Dining/TableSet_00"),
        ("Chair_00_Visual", "/World/Dining/TableSet_00/Chair_00_Visual"),
        ("Chair_01_Visual", "/World/Dining/TableSet_00/Chair_01_Visual"),
    ]:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            print(f"[capture][bbox] {label}: prim not found at {path}", flush=True)
            continue
        world_bbox = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
        print(f"[capture][bbox] {label}: min={tuple(world_bbox.GetMin())} max={tuple(world_bbox.GetMax())}", flush=True)
    print(f"[capture][target] TABLE_HAND_TARGET={tuple(hand_test.TABLE_HAND_TARGET)}", flush=True)

    # The glove Mesh keeps its full, un-trimmed points array (see the asset
    # build notes at the bottom of hand_intrusion_test_actor.py), so a plain
    # BBoxCache bound is polluted by orphaned points never referenced by any
    # face (e.g. the other hand's vertices). Bound only the points actually
    # referenced by faceVertexIndices to get the true rendered extent.
    glove_prim = stage.GetPrimAtPath(f"{hand_test.ELBOW_PATH}/ForearmHand_Glove")
    glove_mesh = UsdGeom.Mesh(glove_prim)
    points = glove_mesh.GetPointsAttr().Get()
    indices = glove_mesh.GetFaceVertexIndicesAttr().Get()
    used = sorted(set(indices))
    local_min = Gf.Vec3d(*[min(points[i][a] for i in used) for a in range(3)])
    local_max = Gf.Vec3d(*[max(points[i][a] for i in used) for a in range(3)])
    xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    local_to_world = xform_cache.GetLocalToWorldTransform(glove_prim)
    corners = [
        local_to_world.Transform(Gf.Vec3d(x, y, z))
        for x in (local_min[0], local_max[0])
        for y in (local_min[1], local_max[1])
        for z in (local_min[2], local_max[2])
    ]
    world_min = Gf.Vec3d(*[min(c[a] for c in corners) for a in range(3)])
    world_max = Gf.Vec3d(*[max(c[a] for c in corners) for a in range(3)])
    print(
        f"[capture][glove_true_bbox] used_points={len(used)}/{len(points)} "
        f"min={tuple(world_min)} max={tuple(world_max)}",
        flush=True,
    )

    table_cam_prim = stage.GetPrimAtPath(TABLE_CAMERA_PATH)
    table_cam = UsdGeom.Camera(table_cam_prim)
    cam_to_world = xform_cache.GetLocalToWorldTransform(table_cam_prim)
    world_to_cam = cam_to_world.GetInverse()
    focal = table_cam.GetFocalLengthAttr().Get()
    h_ap = table_cam.GetHorizontalApertureAttr().Get()
    v_ap = table_cam.GetVerticalApertureAttr().Get()

    def _project_to_normalized(world_point):
        cam_space = world_to_cam.Transform(world_point)
        ndc_x = (focal / (h_ap / 2.0)) * (cam_space[0] / -cam_space[2])
        ndc_y = (focal / (v_ap / 2.0)) * (cam_space[1] / -cam_space[2])
        return ((ndc_x + 1.0) / 2.0, (1.0 - ndc_y) / 2.0)

    corners = [
        Gf.Vec3d(x, y, z)
        for x in (world_min[0], world_max[0])
        for y in (world_min[1], world_max[1])
        for z in (world_min[2], world_max[2])
    ]
    projected = [_project_to_normalized(c) for c in corners]
    xs = [p[0] for p in projected]
    ys = [p[1] for p in projected]
    print(
        f"[capture][table_cam] glove_true_bbox (at progress=1.0) projects to "
        f"normalized x=[{min(xs):.3f},{max(xs):.3f}] y=[{min(ys):.3f},{max(ys):.3f}]",
        flush=True,
    )

    reach_animator._apply_progress(0.0)
    print(f"[capture] done, frames in {out_dir}", flush=True)


def _find_hand_joint_world_positions(stage, skelroot_prim):
    """Return {joint_name: world_position} for every joint in skelroot_prim's
    skeleton whose name contains "hand" or "wrist" (case-insensitive),
    computed at the current timeline time. Used to measure where the
    actor_sdg character's hand actually is during a behavior state, the
    same kind of true-geometry measurement pass 8 did for the old rig's
    glove mesh -- except here there is no dedicated glove prim, so the
    skeleton joint itself is the anchor.

    Finds the Skeleton prim by walking skelroot_prim's own descendants
    instead of `UsdSkel.BindingAPI(skelroot_prim).GetSkeletonRel()`: on the
    default biped asset (`isaacsim.replicator.agent.core`'s
    `load_character_usd_to_stage()`), that relationship's targets list is
    empty even though a `Skeleton`-typed prim is directly present as a
    child (confirmed pass 11) -- likely bound via the AnimationGraph setup
    rather than an explicit `skel:skeleton` USD relationship. Matching
    "wrist" as well as "hand" matters for the same reason: this skeleton's
    joint names go straight from `L_Wrist`/`R_Wrist` to finger joints, with
    no joint literally named "Hand" anywhere (confirmed pass 11 by dumping
    the full joint list) -- the old rig apparently had one, hence the
    original hand-only filter silently returning nothing for this rig.
    """
    from pxr import UsdSkel

    skeleton_prim = None
    for prim in Usd.PrimRange(skelroot_prim):
        if prim.GetTypeName() == "Skeleton":
            skeleton_prim = prim
            break
    if skeleton_prim is None:
        return {}
    skeleton = UsdSkel.Skeleton(skeleton_prim)
    cache = UsdSkel.Cache()
    skel_query = cache.GetSkelQuery(skeleton)
    if not skel_query:
        return {}
    joint_order = skel_query.GetJointOrder()
    time = Usd.TimeCode(omni.timeline.get_timeline_interface().get_current_time() * stage.GetTimeCodesPerSecond())
    # ComputeJointWorldTransforms() takes a UsdGeomXformCache, not a bare
    # TimeCode (confirmed pass 11 -- the original bare-TimeCode call
    # crashes the whole process with Boost.Python.ArgumentError the
    # moment execution actually reaches it; it never had before, since the
    # skeleton-lookup fix above is what first let this line run at all).
    joint_xform_cache = UsdGeom.XformCache(time)
    world_transforms = skel_query.ComputeJointWorldTransforms(joint_xform_cache)
    if world_transforms is None:
        return {}
    results = {}
    for name, xform in zip(joint_order, world_transforms):
        name_lower = str(name).lower()
        if "hand" in name_lower or "wrist" in name_lower:
            matrix = Gf.Matrix4d(xform)
            results[str(name)] = matrix.ExtractTranslation()
    return results


def capture_actor_sdg_frames(stage, person_prim):
    """Headless visual-QA capture for the actor_sdg_test_actor.py mechanism:
    runs the real timeline (needed for omni.anim.people's BehaviorScript to
    self-drive the character) for one full typing/sit/push_button/sit cycle,
    saving frames from a camera that TRACKS the character's live position
    (fixed cameras proved unreliable once the character starts walking --
    see GPU_RUN_LOG.txt pass 10) plus the real table camera, and printing
    the behavior command queue, the active command's own sub-state (e.g.
    Sit's "walk"/"sit"/"stand" phase), the character's live world
    position/rotation, and any "hand" joint world position (projected
    through the real table camera) at each checkpoint.
    """
    import time as _time

    from omni.kit.viewport.utility import capture_viewport_to_file
    from omni.kit.scripting.scripts.script_manager import ScriptManager

    out_dir = Path(
        os.environ.get("MOBILE_DEMO_CAPTURE_DIR", "/tmp/vision_test_actor_sdg_frames")
    )
    out_dir.mkdir(parents=True, exist_ok=True)

    xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())

    def _character_world_transform():
        matrix = xform_cache.GetLocalToWorldTransform(person_prim)
        return matrix.ExtractTranslation(), matrix.ExtractRotation()

    def _ag_character_world_transform():
        """The outer spawned Xform's own xformOp never moves during
        walk/sit (confirmed by testing) -- Sit/GoTo move the character
        through the ag.Character wrapper's own get_world_transform()/
        set_world_transform(), exactly like Sit.setup()'s own
        `Utils.get_character_pos(self.character)` call does, not a plain
        USD Xform translate. Query that same way for a true reading.
        """
        import carb as _carb
        import omni.anim.graph.core as ag

        skelroot_path = None
        for prim in Usd.PrimRange(person_prim):
            if prim.GetTypeName() == "SkelRoot":
                skelroot_path = str(prim.GetPath())
                break
        if skelroot_path is None:
            return None
        character = ag.get_character(skelroot_path)
        if character is None:
            return None
        pos = _carb.Float3(0, 0, 0)
        rot = _carb.Float4(0, 0, 0, 0)
        character.get_world_transform(pos, rot)
        return tuple(pos), tuple(rot)

    tracking_cam_path = "/World/QACamera_tracking"
    _define_lookat_camera(
        stage, tracking_cam_path,
        Gf.Vec3d(0, 0, 0) + Gf.Vec3d(2.0, -2.0, 1.6),
        Gf.Vec3d(0, 0, 0.9),
        Gf.Vec3d(0, 0, 1),
    )
    tracking_window = create_viewport_for_camera(
        "QA_tracking", tracking_cam_path, width=1280, height=960
    )
    table_window = create_viewport_for_camera("QA_table", TABLE_CAMERA_PATH, width=1280, height=960)
    viewports = {"tracking": tracking_window.viewport_api, "table": table_window.viewport_api}

    tracking_cam_prim = stage.GetPrimAtPath(tracking_cam_path)

    def _update_tracking_camera():
        ag_transform = _ag_character_world_transform()
        char_pos = Gf.Vec3d(*ag_transform[0]) if ag_transform else Gf.Vec3d(*actor_sdg.STAND_XY, 0.0)
        eye = char_pos + Gf.Vec3d(0.1, 0.1, 3.5)
        target = char_pos + Gf.Vec3d(0, 0, 0.3)
        cam_to_world = Gf.Matrix4d().SetLookAt(eye, target, Gf.Vec3d(0, 0, 1)).GetInverse()
        UsdGeom.Xformable(tracking_cam_prim).MakeMatrixXform().Set(cam_to_world)

    table_cam_prim = stage.GetPrimAtPath(TABLE_CAMERA_PATH)
    table_cam = UsdGeom.Camera(table_cam_prim)

    def _project_to_normalized(world_point):
        cam_to_world = xform_cache.GetLocalToWorldTransform(table_cam_prim)
        world_to_cam = cam_to_world.GetInverse()
        cam_space = world_to_cam.Transform(world_point)
        focal = table_cam.GetFocalLengthAttr().Get()
        h_ap = table_cam.GetHorizontalApertureAttr().Get()
        v_ap = table_cam.GetVerticalApertureAttr().Get()
        ndc_x = (focal / (h_ap / 2.0)) * (cam_space[0] / -cam_space[2])
        ndc_y = (focal / (v_ap / 2.0)) * (cam_space[1] / -cam_space[2])
        return ((ndc_x + 1.0) / 2.0, (1.0 - ndc_y) / 2.0)

    def _render_settle(frames=15):
        for _ in range(frames):
            simulation_app.update()

    def _capture(label):
        actor_sdg._patch_sit_command_stand_rotation()
        actor_sdg._patch_timing_template_position_anchor()
        _update_tracking_camera()
        _render_settle(3)
        for view_name in ("tracking", "table"):
            path = out_dir / f"{label}_{view_name}.png"
            capture_viewport_to_file(viewports[view_name], str(path))
        _render_settle(5)

        char_pos, char_rot = _character_world_transform()
        ag_transform = _ag_character_world_transform()
        print(
            f"[capture] wrote {label} | outer Xform world pos={tuple(round(v, 3) for v in char_pos)} "
            f"rot_axis={tuple(round(v, 3) for v in char_rot.GetAxis())} "
            f"rot_angle={round(char_rot.GetAngle(), 2)} | "
            f"ag.Character world pos={tuple(round(v, 3) for v in ag_transform[0]) if ag_transform else None} "
            f"rot={ag_transform[1] if ag_transform else None}",
            flush=True,
        )
        robot_base_prim = stage.GetPrimAtPath(
            "/World/ServingRobot/Robot/ridgeback_base_link/ridgeback_base_link"
        )
        if robot_base_prim.IsValid():
            robot_matrix = xform_cache.GetLocalToWorldTransform(robot_base_prim)
            print(
                f"[capture]   robot base world pos={tuple(round(v, 3) for v in robot_matrix.ExtractTranslation())} "
                f"rot_axis={tuple(round(v, 3) for v in robot_matrix.ExtractRotation().GetAxis())} "
                f"rot_angle={round(robot_matrix.ExtractRotation().GetAngle(), 2)}",
                flush=True,
            )

        sm = ScriptManager.get_instance()
        for scripts in sm._prim_to_scripts.values():
            for _, inst in scripts.items():
                cur_cmd = getattr(inst, "current_command", "NO_ATTR")
                sub_state = getattr(cur_cmd, "current_action", None)
                print(
                    f"[capture]   commands={getattr(inst, 'commands', 'NO_ATTR')} "
                    f"current_command={cur_cmd} sub_state={sub_state!r}",
                    flush=True,
                )
        for prim in Usd.PrimRange(person_prim):
            if prim.GetTypeName() == "SkelRoot":
                hand_positions = _find_hand_joint_world_positions(stage, prim)
                for joint_name, world_pos in hand_positions.items():
                    norm = _project_to_normalized(world_pos)
                    print(
                        f"[capture]   joint '{joint_name}' world={tuple(world_pos)} "
                        f"projects to normalized={tuple(round(v, 3) for v in norm)}",
                        flush=True,
                    )

    chair_prim = stage.GetPrimAtPath(actor_sdg.CHAIR_PRIM_PATH)
    if chair_prim.IsValid():
        matrix = xform_cache.GetLocalToWorldTransform(chair_prim)
        print(
            f"[capture][chair] {actor_sdg.CHAIR_PRIM_PATH} "
            f"world pos={tuple(round(v, 3) for v in matrix.ExtractTranslation())} "
            f"rot_axis={tuple(round(v, 3) for v in matrix.ExtractRotation().GetAxis())} "
            f"rot_angle={round(matrix.ExtractRotation().GetAngle(), 2)}",
            flush=True,
            )

    _capture("start")

    # Watchdog: confirmed by testing (GPU_RUN_LOG.txt pass 11) that the
    # character's world ROTATION drifts off pure-Z-yaw during some (not
    # all) Sit "stand" transitions -- specifically the 2nd sit in a
    # type_keyboard->Sit->push_button->Sit loop, reproduced identically
    # even with two different Sit.update() patch strategies and with 2x
    # the settle time, proving it's the AnimationGraph's own per-frame
    # evaluation overriding rotation, not anything Sit's Python wrapper
    # sets. Since that's vendored/non-repo, correct it here instead:
    # every frame, if rotation has drifted off pure Z, snap it back
    # (preserving current yaw) via the same ag.Character world-transform
    # API Sit itself uses -- this runs after the graph's own evaluation
    # for the frame, so it can override what the graph just computed.
    # NOT a clean fix: confirmed by testing this does hold rotation
    # upright, but the character's Z position sinks noticeably (down to
    # ~-0.19) instead, presumably because stomping rotation every frame
    # fights whatever foot-planting/height logic the graph ties to the
    # same transform. Left here, opt-in via MOBILE_DEMO_UPRIGHT_WATCHDOG=1,
    # for further investigation -- disabled by default.
    import carb as _carb
    import omni.anim.graph.core as _ag

    _watchdog_skelroot_path = None
    for _prim in Usd.PrimRange(person_prim):
        if _prim.GetTypeName() == "SkelRoot":
            _watchdog_skelroot_path = str(_prim.GetPath())
            break

    def _enforce_upright_rotation():
        if _watchdog_skelroot_path is None:
            return
        character = _ag.get_character(_watchdog_skelroot_path)
        if character is None:
            return
        pos = _carb.Float3(0, 0, 0)
        rot = _carb.Float4(0, 0, 0, 0)
        character.get_world_transform(pos, rot)
        if abs(rot[0]) < 1e-4 and abs(rot[1]) < 1e-4:
            return
        z, w = rot[2], rot[3]
        mag = (z * z + w * w) ** 0.5
        if mag < 1e-6:
            return
        upright_rot = _carb.Float4(0.0, 0.0, z / mag, w / mag)
        character.set_world_transform(pos, upright_rot)

    print("[capture] entering play...", flush=True)
    omni.timeline.get_timeline_interface().play()
    total_seconds = float(os.environ.get("MOBILE_DEMO_CAPTURE_SECONDS", "24"))
    interval = float(os.environ.get("MOBILE_DEMO_CAPTURE_INTERVAL", "1.0"))
    next_capture = interval
    elapsed = 0.0
    while elapsed < total_seconds:
        simulation_app.update()
        if os.environ.get("MOBILE_DEMO_UPRIGHT_WATCHDOG", "0") == "1":
            _enforce_upright_rotation()
        _time.sleep(1.0 / 30.0)
        elapsed += 1.0 / 30.0
        if elapsed >= next_capture:
            _capture(f"t{elapsed:05.1f}")
            next_capture += interval

    _capture("end")
    omni.timeline.get_timeline_interface().stop()
    print(f"[capture] done, frames in {out_dir}", flush=True)


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


def main():
    hand_test_enabled = os.environ.get("MOBILE_DEMO_HAND_TEST", "1") == "1"
    if hand_test_enabled and HAND_TEST_RIG_MODE not in ("rigid_arm", "legacy"):
        # Must run BEFORE the restaurant stage is opened below, AND must be
        # followed by several simulation_app.update() calls before that
        # open happens -- confirmed by testing (see actor_sdg_test_actor.
        # enable_extensions()'s docstring): enabling these extensions and
        # opening the stage back-to-back with no settle gap reproducibly
        # segfaults during that open (omni.anim.graph.core's
        # CharacterManager gets torn down mid-startup by the stage
        # transition). Pumping ~30 frames here lets the extensions'
        # startup fully complete first, after which the same stage open is
        # clean and ag.get_character() registers the character normally
        # once it's spawned.
        actor_sdg.enable_extensions()
        for _ in range(30):
            simulation_app.update()
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

    reach_animator = None
    typing_controller = None
    person_prim = None
    if hand_test_enabled and HAND_TEST_RIG_MODE not in ("rigid_arm", "legacy"):
        person_prim = actor_sdg.spawn_and_configure_actor(stage)
        typing_controller = actor_sdg.TypingTopicController(person_prim)

    articulation, dof_names = initialize_robot(articulation_path)

    if hand_test_enabled and HAND_TEST_RIG_MODE in ("rigid_arm", "legacy"):
        person_prim = hand_test.spawn_seated_person(stage)
        reach_animator = hand_test.ReachAnimator(person_prim)
    if os.environ.get("MOBILE_DEMO_PRINT_BOUNDS", "0") == "1":
        print_arm_visual_bounds(stage)
    run_optional_diagnostic(articulation, dof_names)
    run_stability_check(articulation, dof_names)

    if os.environ.get("MOBILE_DEMO_CAPTURE_POSES", "0") == "1":
        if person_prim is None:
            raise RuntimeError("MOBILE_DEMO_CAPTURE_POSES needs MOBILE_DEMO_HAND_TEST=1")
        if HAND_TEST_RIG_MODE in ("rigid_arm", "legacy"):
            capture_pose_frames(stage, reach_animator)
        else:
            capture_actor_sdg_frames(stage, person_prim)
        if typing_controller is not None:
            typing_controller.shutdown()
        simulation_app.close()
        return

    if os.environ.get("MOBILE_DEMO_EXIT_AFTER_READY", "0") == "1":
        if typing_controller is not None:
            typing_controller.shutdown()
        simulation_app.close()
        return

    try:
        while simulation_app.is_running():
            simulation_app.update()
            if reach_animator is not None:
                reach_animator.update()
            if typing_controller is not None:
                typing_controller.update()
            # Keep the GUI event loop responsive without throttling it to the old
            # uneven 16 ms cadence.
            time.sleep(0.010)
    finally:
        if typing_controller is not None:
            typing_controller.shutdown()


try:
    main()
except BaseException:
    traceback.print_exc()
    raise
finally:
    simulation_app.close()
