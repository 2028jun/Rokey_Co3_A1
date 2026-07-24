"""Spawn the Ridgeback + serving shelf + M0609 robot in the restaurant.

This script targets Isaac Sim 5.1.0-rc.19.  It is intentionally not launched
by the ROS package build.  Run it only when the Isaac demonstration is needed.
Set MOBILE_DEMO_AUTORUN=1 to enable the short wheel/arm diagnostic sequence.
"""

import os
import sys
import time
import traceback
import asyncio
import json
import struct
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
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.api.objects import DynamicCylinder, FixedCuboid
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.viewports import create_viewport_for_camera
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.grippers import ParallelGripper
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade


WORKSPACE = Path(
    os.environ.get("COBOT3_WS", Path(__file__).resolve().parents[1])
).resolve()
RMPFLOW_DIR = WORKSPACE / "isaacpjt/M0609/rmpflow"
if str(RMPFLOW_DIR) not in sys.path:
    sys.path.insert(0, str(RMPFLOW_DIR))

from m0609_rmpflow_controller import RMPFlowController

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
# IK-derived safe start for the top-deck pickup.  link_6 begins at roughly
# [0.05, 0.566, 0.092] in the arm frame with the RG2 already horizontal and
# clear of the serving dish, avoiding the large wrist flip from the former
# folded pose.
STOW_CONFIGURATION = [1.4893, 1.1791, 0.9657, -0.1492, 2.5622, -0.1253]
ARM_DRIVE_STIFFNESS = float(os.environ.get("MOBILE_ARM_STIFFNESS", "200000"))
ARM_DRIVE_DAMPING = float(os.environ.get("MOBILE_ARM_DAMPING", "20000"))
ARM_DRIVE_MAX_FORCE = float(os.environ.get("MOBILE_ARM_MAX_FORCE", "10000"))
WHEEL_DRIVE_DAMPING = float(os.environ.get("MOBILE_WHEEL_DAMPING", "1500"))
WHEEL_DRIVE_MAX_FORCE = float(os.environ.get("MOBILE_WHEEL_MAX_FORCE", "2000"))
PARKED_HOLD = os.environ.get("MOBILE_DEMO_PARKED_HOLD", "1") == "1"

# Tray-to-table serving motion, adapted from M0609/4_pick_place.py.
PICK_PLACE_ENABLED = os.environ.get("MOBILE_DEMO_PICK_PLACE", "1") == "1"
VERTICAL_LIFT_TEST = (
    os.environ.get("MOBILE_DEMO_VERTICAL_LIFT_TEST", "1") == "1"
)
EXIT_ON_MOTION_ERROR = (
    os.environ.get("MOBILE_DEMO_EXIT_ON_MOTION_ERROR", "0") == "1"
)
GRIPPER_JOINTS = ["rg2_finger_joint", "rg2_right_inner_knuckle_joint"]
# parse_mimic=True leaves only rg2_finger_joint independently driven.  The
# other five RG2 joints follow it through PhysX mimic constraints, therefore
# ParallelGripper must receive one value (not one value per physical finger).
GRIPPER_OPEN = np.array([0.0])
# Add a small preload beyond the 24 mm centre-grip width.  The command is
# continuously reasserted while carrying the board so arm-controller updates
# cannot relax the RG2 mimic joint after the initial grasp.
GRIPPER_CLOSE = np.array([0.99])
GRIPPER_DELTA = np.array([-0.99])
GRIPPER_DRIVE_STIFFNESS = 30000.0
GRIPPER_DRIVE_DAMPING = 5000.0
GRIPPER_DRIVE_MAX_FORCE = 80.0
GRIP_CONTACT_STATIC_FRICTION = 10.0
GRIP_CONTACT_DYNAMIC_FRICTION = 8.0
M0609_RMPFLOW_URDF = (
    WORKSPACE / "isaacpjt/M0609/doosan-robot2/urdf/m0609_isaac_sim.urdf"
)
M0609_DESCRIPTION = RMPFLOW_DIR / "m0609_description.yaml"
M0609_RMPFLOW_CONFIG = RMPFLOW_DIR / "m0609_rmpflow_common.yaml"
# Coordinates are relative to the M0609 base.  Large pizza serviceware belongs
# on the open top deck, away from the internal tray reserved for cans and small
# personal plates.  An 8 cm stand leaves room below the large plate rim.
TOP_DECK_STAND_LOCAL = np.array([0.38, 0.0, 0.0])
# The 8 cm stand top is at local Z=0.040.  Put the 18 mm board 0.5 mm
# above it so gravity does not move the bail away from the planned grasp.
TOP_DECK_DISH_LOCAL = np.array([0.38, 0.0, 0.0495])
# The notched upper tray consists of a rear panel and a +X side rail.  Values
# are relative to the arm base and match the two boxes authored in the URDF.
UPPER_TRAY_COLLIDERS = (
    (np.array([0.22, -0.145, -0.255]), np.array([0.74, 0.33, 0.025])),
    (np.array([0.43, 0.165, -0.255]), np.array([0.32, 0.29, 0.025])),
)
TABLE_STAND_LOCAL = np.array([-0.65, 0.0, -0.127])
# The table stand top is at -0.087; use the same 0.5 mm contact clearance.
TABLE_DISH_LOCAL = np.array([-0.65, 0.0, -0.0775])
# The 39 cm round deck carries the 32.4 cm pizza.  A separate thin steel bail
# starts flat in the local X-Y plane and rotates +90 degrees about the deck's
# X diameter before it carries the load above the centre of mass.
BOARD_RADIUS = 0.195
BOARD_THICKNESS = 0.018
BAIL_RADIUS = BOARD_RADIUS + 0.010
BAIL_WIRE_RADIUS = 0.004
BAIL_GRIP_RADIUS = 0.012
BAIL_GRIP_LENGTH = 0.090
BAIL_SEGMENTS_PER_SIDE = 14
# The URDF's nominal TCP is at the fingertip.  A shallow 10 mm insertion puts
# the flat grip inside the finger pads without driving the tips into the deck.
BAIL_GRIP_INSERT_DEPTH = 0.010
# Raise the flat hinge plane enough for the 24 mm centre grip to clear the
# board by 1 mm.  The thin wire therefore rests horizontally on low brackets.
BAIL_HINGE_Z = BOARD_THICKNESS * 0.5 + BAIL_GRIP_RADIUS + 0.001
BAIL_GRIP_REACH = np.sqrt(
    BAIL_RADIUS**2 - (BAIL_GRIP_LENGTH * 0.5) ** 2
)
BAIL_UPRIGHT_GRIP_HEIGHT = BAIL_HINGE_Z + BAIL_GRIP_REACH
# The board/pizza assembly is one kilogram as requested.
SERVING_DISH_MASS = 1.0
PIZZA_ASSET_SCALE = 0.72
DISH_RADIUS = BOARD_RADIUS
# DynamicCylinder is only the rigid-body carrier; its collider is disabled.
# The visible round board primitive provides the actual collision.
DISH_HEIGHT = BOARD_THICKNESS
DISH_STAND_SCALE = np.array([0.16, 0.16, 0.08])
# Supreme Pizza mesh bounds are not exactly symmetric about its GLB origin.
# After its authored +90-degree X rotation, the horizontal centre is
# approximately (+0.002577, +0.001766), so cancel that offset here.
PIZZA_CENTER_OFFSET = np.array(
    [
        -0.002577 * PIZZA_ASSET_SCALE,
        -0.001766 * PIZZA_ASSET_SCALE,
        0.038,
    ]
)
PLACE_APPROACH_DISTANCE = 0.08
TOP_DECK_APPROACH_DISTANCE = 0.10
# Twelve centimetres clears the pizza and board during the post-grasp lift.
TOP_DECK_SAFE_CLEARANCE = 0.12
INITIAL_TRANSITION_STEPS = 240
BAIL_RAISE_TRANSITION_STEPS = 360
# Keep the board supported briefly after the folded bail reaches 90 degrees.
# This gives the arm and folded handle time to settle before the one-kilogram
# payload begins its vertical lift.
BAIL_UPRIGHT_SETTLE_STEPS = 90
DISH_LIFT_TRANSITION_STEPS = 240
TRANSFER_CLEARANCE = 0.18
RG2_TCP_LENGTH = 0.231066
# Point the assembled RG2 forward axis (+Z) downward and map its local X
# finger-separation axis to world +Y.  The fingers therefore descend from
# above and close across the bail's X-directed centre grip.  Quaternion order
# is scalar-first.
BAIL_GRASP_EE_ORIENTATION = np.array(
    [0.0, np.sqrt(0.5), np.sqrt(0.5), 0.0], dtype=float
)


def quaternion_slerp(start, end, amount):
    """Shortest-path interpolation for scalar-first unit quaternions."""
    start = np.asarray(start, dtype=float) / np.linalg.norm(start)
    end = np.asarray(end, dtype=float) / np.linalg.norm(end)
    dot = float(np.dot(start, end))
    if dot < 0.0:
        end = -end
        dot = -dot
    if dot > 0.9995:
        result = start + amount * (end - start)
        return result / np.linalg.norm(result)
    angle = np.arccos(np.clip(dot, -1.0, 1.0))
    return (
        np.sin((1.0 - amount) * angle) * start
        + np.sin(amount * angle) * end
    ) / np.sin(angle)

ANTIGRAVITY_FOOD_DIR = Path(
    "/home/rokey/.gemini/antigravity/scratch/assets/food"
)
ANTIGRAVITY_PLATE_GLB = ANTIGRAVITY_FOOD_DIR / "pizza_plate.glb"
SUPREME_PIZZA_GLB = (
    WORKSPACE / "assets/source_food/supreme_pizza_jarlan_perez.glb"
)
CONVERTED_FOOD_DIR = WORKSPACE / "assets/antigravity_food"
PLATE_USD = CONVERTED_FOOD_DIR / "pizza_plate.usd"
PLATE_TEXTURE = CONVERTED_FOOD_DIR / "textures/RecCenter_CafeAssets_tex.png"


def bind_preview_texture(stage, mesh_root, material_path, texture_path):
    """Bind a reliable USD Preview Surface to converted GLB mesh children."""
    if not texture_path.is_file():
        raise FileNotFoundError(texture_path)
    material = UsdShade.Material.Define(stage, material_path)
    surface = UsdShade.Shader.Define(stage, f"{material_path}/Surface")
    surface.CreateIdAttr("UsdPreviewSurface")
    surface.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
    surface.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)

    texture = UsdShade.Shader.Define(stage, f"{material_path}/Texture")
    texture.CreateIdAttr("UsdUVTexture")
    texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath(str(texture_path))
    )
    texture.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")

    primvar = UsdShade.Shader.Define(stage, f"{material_path}/Primvar")
    primvar.CreateIdAttr("UsdPrimvarReader_float2")
    primvar.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        primvar.ConnectableAPI(), "result"
    )
    surface.CreateInput(
        "diffuseColor", Sdf.ValueTypeNames.Color3f
    ).ConnectToSource(texture.ConnectableAPI(), "rgb")
    material.CreateSurfaceOutput().ConnectToSource(
        surface.ConnectableAPI(), "surface"
    )

    bound = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(mesh_root)):
        if prim.IsA(UsdGeom.Mesh):
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)
            bound += 1
    if bound == 0:
        raise RuntimeError(f"no mesh found below converted asset {mesh_root}")


def apply_plate_mesh_collisions(stage, mesh_root, physics_material):
    """Use the visible plate mesh itself as the dynamic collision geometry."""
    colliders = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(mesh_root)):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        collision_api = UsdPhysics.CollisionAPI.Apply(prim)
        collision_api.CreateCollisionEnabledAttr(True)
        mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(prim)
        mesh_collision_api.CreateApproximationAttr().Set("convexDecomposition")
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            physics_material.material,
            bindingStrength=UsdShade.Tokens.strongerThanDescendants,
            materialPurpose="physics",
        )
        colliders += 1
    if colliders == 0:
        raise RuntimeError(f"no plate mesh found below {mesh_root}")
    print(
        f"[serving] enabled convex-decomposition collision on "
        f"{colliders} plate meshes",
        flush=True,
    )


def author_pizza_board_with_bail(
    stage,
    board_root_path,
    bail_root_path,
    physics_material,
):
    """Create a round board and a separate, initially flat wire bail."""

    def author_preview_material(path, color, roughness, metallic):
        material = UsdShade.Material.Define(stage, path)
        shader = UsdShade.Shader.Define(stage, f"{path}/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Float3).Set(
            Gf.Vec3f(*color)
        )
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
        material.CreateSurfaceOutput().ConnectToSource(
            shader.ConnectableAPI(), "surface"
        )
        return material

    board_material = author_preview_material(
        "/World/Looks/WoodenPizzaBoard",
        (0.66, 0.40, 0.18),
        0.72,
        0.0,
    )
    wire_material = author_preview_material(
        "/World/Looks/PizzaBailSteel",
        (0.48, 0.52, 0.56),
        0.28,
        0.85,
    )
    grip_material = author_preview_material(
        "/World/Looks/PizzaBailGrip",
        (0.05, 0.16, 0.23),
        0.58,
        0.05,
    )

    def configure_shape(prim, visual_material, collision_enabled=True):
        binding = UsdShade.MaterialBindingAPI.Apply(prim)
        binding.Bind(
            visual_material,
            bindingStrength=UsdShade.Tokens.strongerThanDescendants,
        )
        binding.Bind(
            physics_material.material,
            bindingStrength=UsdShade.Tokens.strongerThanDescendants,
            materialPurpose="physics",
        )
        if collision_enabled:
            UsdPhysics.CollisionAPI.Apply(
                prim
            ).CreateCollisionEnabledAttr(True)
            physx_collision = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            physx_collision.CreateContactOffsetAttr(0.001)
            physx_collision.CreateRestOffsetAttr(0.0)

    board = UsdGeom.Cylinder.Define(stage, f"{board_root_path}/Board")
    board.CreateAxisAttr(UsdGeom.Tokens.z)
    board.CreateRadiusAttr(BOARD_RADIUS)
    board.CreateHeightAttr(BOARD_THICKNESS)
    board.CreateDisplayColorAttr([Gf.Vec3f(0.66, 0.40, 0.18)])
    configure_shape(board.GetPrim(), board_material)

    bail_root = UsdGeom.Xform.Define(stage, f"{bail_root_path}/Bail")
    # These two ops animate the complete visible bail about its hinge.  The
    # carrier rigid body remains at the board's original centre so PhysX does
    # not overwrite the scripted fold transform.
    bail_root.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
    bail_root.AddOrientOp(UsdGeom.XformOp.PrecisionFloat).Set(
        Gf.Quatf(1.0, 0.0, 0.0, 0.0)
    )

    def author_wire_segment(name, start, end, radius, visual_material):
        start = np.asarray(start, dtype=float)
        end = np.asarray(end, dtype=float)
        direction = end - start
        length = float(np.linalg.norm(direction))
        if length <= 0.0:
            raise ValueError(f"zero-length bail segment: {name}")
        midpoint = 0.5 * (start + end)
        # The authored rest pose lies in local X-Y.  A Z rotation aligns each
        # cylinder's local X axis with its chord; the parent Xform later
        # rotates the complete shape around the X hinge axis.
        rotate_z = float(np.degrees(np.arctan2(direction[1], direction[0])))
        segment = UsdGeom.Cylinder.Define(
            stage, f"{bail_root.GetPath()}/{name}"
        )
        segment.CreateAxisAttr(UsdGeom.Tokens.x)
        segment.CreateRadiusAttr(radius)
        # A slight overlap removes visual gaps between adjacent arc chords.
        segment.CreateHeightAttr(length * 1.03)
        segment.AddTranslateOp().Set(Gf.Vec3d(*map(float, midpoint)))
        segment.AddRotateZOp().Set(rotate_z)
        configure_shape(
            segment.GetPrim(),
            visual_material,
            collision_enabled=False,
        )

    grip_half_length = BAIL_GRIP_LENGTH * 0.5
    grip_boundary = float(np.arccos(grip_half_length / BAIL_RADIUS))

    # Leave the central arc chord out and replace it with a thicker, coloured
    # X-directed grip.  In the rest pose it lies near the +Y rim; after a
    # 90-degree hinge rotation it is directly above the board centre of mass.
    arc_ranges = (
        np.linspace(0.0, grip_boundary, BAIL_SEGMENTS_PER_SIDE + 1),
        np.linspace(
            np.pi - grip_boundary,
            np.pi,
            BAIL_SEGMENTS_PER_SIDE + 1,
        ),
    )
    segment_index = 0
    for angles in arc_ranges:
        points = [
            np.array(
                [
                    BAIL_RADIUS * np.cos(angle),
                    BAIL_RADIUS * np.sin(angle),
                    BAIL_HINGE_Z,
                ]
            )
            for angle in angles
        ]
        for start, end in zip(points[:-1], points[1:]):
            author_wire_segment(
                f"Wire_{segment_index:02d}",
                start,
                end,
                BAIL_WIRE_RADIUS,
                wire_material,
            )
            segment_index += 1

    # Short inward feet join the slightly oversized bail diameter to the
    # circular board instead of leaving the wire floating beyond its rim.
    mount_x = BOARD_RADIUS - BAIL_WIRE_RADIUS
    for side, sign in (("MinusX", -1.0), ("PlusX", 1.0)):
        author_wire_segment(
            f"Mount_{side}",
            np.array([sign * BAIL_RADIUS, 0.0, BAIL_HINGE_Z]),
            np.array([sign * mount_x, 0.0, BAIL_HINGE_Z]),
            BAIL_WIRE_RADIUS,
            wire_material,
        )

    author_wire_segment(
        "CenterGrip",
        np.array([-grip_half_length, BAIL_GRIP_REACH, BAIL_HINGE_Z]),
        np.array([grip_half_length, BAIL_GRIP_REACH, BAIL_HINGE_Z]),
        BAIL_GRIP_RADIUS,
        grip_material,
    )
    print(
        f"[serving] authored round pizza board radius={BOARD_RADIUS:.3f}m "
        f"bail_diameter={2.0 * BAIL_RADIUS:.3f}m "
        f"wire_diameter={2.0 * BAIL_WIRE_RADIUS:.3f}m "
        f"grip={BAIL_GRIP_LENGTH:.3f}x{2.0 * BAIL_GRIP_RADIUS:.3f}m "
        f"rest=flat hinge=+X[0,90]deg "
        f"board_mass={SERVING_DISH_MASS:.3f}kg",
        flush=True,
    )


def author_colored_glb_meshes(stage, root_path, glb_path):
    """Author GLB primitives directly so Isaac's converter cannot drop colors."""
    data = glb_path.read_bytes()
    magic, version, _ = struct.unpack_from("<4sII", data, 0)
    if magic != b"glTF" or version != 2:
        raise RuntimeError(f"unsupported GLB: {glb_path}")
    json_length, json_type = struct.unpack_from("<II", data, 12)
    if json_type != 0x4E4F534A:
        raise RuntimeError(f"GLB JSON chunk missing: {glb_path}")
    document = json.loads(data[20 : 20 + json_length])
    offset = 20 + json_length
    while offset % 4:
        offset += 1
    binary_length, binary_type = struct.unpack_from("<II", data, offset)
    if binary_type != 0x004E4942:
        raise RuntimeError(f"GLB binary chunk missing: {glb_path}")
    binary = data[offset + 8 : offset + 8 + binary_length]

    component_formats = {
        5121: ("B", 1),
        5123: ("H", 2),
        5125: ("I", 4),
        5126: ("f", 4),
    }
    component_counts = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}

    def read_accessor(index):
        accessor = document["accessors"][index]
        view = document["bufferViews"][accessor["bufferView"]]
        code, component_size = component_formats[accessor["componentType"]]
        width = component_counts[accessor["type"]]
        element_size = component_size * width
        stride = view.get("byteStride", element_size)
        start = view.get("byteOffset", 0) + accessor.get("byteOffset", 0)
        fmt = "<" + code * width
        return [
            struct.unpack_from(fmt, binary, start + row * stride)
            for row in range(accessor["count"])
        ]

    mesh_index = document["nodes"][document["scenes"][0]["nodes"][0]]["mesh"]
    primitives = document["meshes"][mesh_index]["primitives"]
    materials = document.get("materials", [])
    for index, primitive in enumerate(primitives):
        if primitive.get("mode", 4) != 4:
            raise RuntimeError("Supreme Pizza contains a non-triangle primitive")
        points = read_accessor(primitive["attributes"]["POSITION"])
        normals = read_accessor(primitive["attributes"]["NORMAL"])
        indices = [value[0] for value in read_accessor(primitive["indices"])]

        mesh = UsdGeom.Mesh.Define(stage, f"{root_path}/Primitive_{index}")
        mesh.CreatePointsAttr([Gf.Vec3f(*point) for point in points])
        mesh.CreateFaceVertexCountsAttr([3] * (len(indices) // 3))
        mesh.CreateFaceVertexIndicesAttr(indices)
        mesh.CreateNormalsAttr([Gf.Vec3f(*normal) for normal in normals])
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        mesh.CreateDoubleSidedAttr(True)

        source_material = materials[primitive.get("material", 0)]
        pbr = source_material.get("pbrMetallicRoughness", {})
        color = pbr.get("baseColorFactor", [0.8, 0.8, 0.8, 1.0])
        # Keep displayColor as a Hydra fallback and also bind an explicit
        # PreviewSurface.  The source GLB uses seven baseColorFactor materials
        # rather than textures.
        mesh.CreateDisplayColorAttr([Gf.Vec3f(*color[:3])])
        mesh.CreateDisplayOpacityAttr([float(color[3])])

        material_path = f"/World/Looks/PizzaPrimitiveMaterial_{index}"
        material = UsdShade.Material.Define(stage, material_path)
        shader = UsdShade.Shader.Define(stage, f"{material_path}/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Float3).Set(
            Gf.Vec3f(*color[:3])
        )
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.65)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        material.CreateSurfaceOutput().ConnectToSource(
            shader.ConnectableAPI(), "surface"
        )
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(
            material,
            bindingStrength=UsdShade.Tokens.strongerThanDescendants,
        )
    print(
        f"[serving] authored {len(primitives)} colored pizza primitives "
        f"from {glb_path.name}",
        flush=True,
    )


def enable_urdf_importer():
    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate("isaacsim.asset.importer.urdf", True)


async def _convert_food_asset(source, destination):
    import omni.kit.asset_converter

    context = omni.kit.asset_converter.AssetConverterContext()
    context.ignore_materials = False
    context.ignore_animation = True
    context.ignore_cameras = True
    context.single_mesh = False
    context.use_meter_as_world_unit = True
    context.create_world_as_default_root_prim = True
    converter = omni.kit.asset_converter.get_instance()
    task = converter.create_converter_task(
        str(source), str(destination), lambda _progress, _total: None, context
    )
    if not await task.wait_until_finished():
        raise RuntimeError(f"asset conversion failed: {source}")


def prepare_antigravity_food_assets():
    """Validate the retained Supreme pizza asset."""
    if not SUPREME_PIZZA_GLB.is_file():
        raise FileNotFoundError(SUPREME_PIZZA_GLB)


def import_robot_usd():
    if not URDF_PATH.is_file():
        raise FileNotFoundError(
            f"generated URDF missing: {URDF_PATH}\n"
            "Run xacro after sourcing the workspace."
        )

    ROBOT_USD.parent.mkdir(parents=True, exist_ok=True)
    usd_is_current = (
        ROBOT_USD.is_file()
        and ROBOT_USD.stat().st_mtime >= URDF_PATH.stat().st_mtime
    )
    if usd_is_current and os.environ.get("MOBILE_DEMO_REIMPORT", "0") != "1":
        print(f"[mobile robot] reuse USD={ROBOT_USD}", flush=True)
        return
    if ROBOT_USD.is_file():
        print("[mobile robot] URDF changed; reimporting robot USD", flush=True)
    status, config = omni.kit.commands.execute("URDFCreateImportConfig")
    if not status:
        raise RuntimeError("URDFCreateImportConfig failed")
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
    preferred = (
        "/World/ServingRobot/Robot/ridgeback_base_link/"
        "ridgeback_base_link"
    )
    roots = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
        and str(prim.GetPath()).startswith("/World/ServingRobot")
    ]
    preferred_prim = stage.GetPrimAtPath(preferred)
    if (
        preferred_prim.IsValid()
        and preferred_prim.HasAPI(UsdPhysics.ArticulationRootAPI)
        and preferred_prim.HasAPI(UsdPhysics.RigidBodyAPI)
    ):
        print(
            f"[mobile robot] selected articulation root={preferred}",
            flush=True,
        )
        return preferred
    rigid_roots = [
        path
        for path in roots
        if stage.GetPrimAtPath(path).HasAPI(UsdPhysics.RigidBodyAPI)
    ]
    if len(rigid_roots) != 1:
        raise RuntimeError(
            f"expected one rigid serving-robot articulation; "
            f"roots={roots} rigid_roots={rigid_roots}"
        )
    print(
        f"[mobile robot] selected fallback articulation root={rigid_roots[0]}",
        flush=True,
    )
    return rigid_roots[0]


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
    base_position, base_orientation, _ = prim_world_pose(
        stage.GetPrimAtPath(articulation_path)
    )
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*map(float, base_position)))
    joint.CreateLocalRot0Attr().Set(
        Gf.Quatf(
            float(base_orientation[0]),
            Gf.Vec3f(*map(float, base_orientation[1:])),
        )
    )
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
    configured_gripper = []
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
        elif name == GRIPPER_JOINTS[0]:
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            drive.CreateStiffnessAttr(GRIPPER_DRIVE_STIFFNESS)
            drive.CreateDampingAttr(GRIPPER_DRIVE_DAMPING)
            drive.CreateMaxForceAttr(GRIPPER_DRIVE_MAX_FORCE)
            configured_gripper.append(name)

    if set(configured_arm) != set(ARM_JOINTS):
        raise RuntimeError(f"arm drive setup incomplete: {configured_arm}")
    if set(configured_wheels) != set(WHEEL_JOINTS):
        raise RuntimeError(f"wheel drive setup incomplete: {configured_wheels}")
    if PICK_PLACE_ENABLED and configured_gripper != [GRIPPER_JOINTS[0]]:
        raise RuntimeError(f"gripper drive setup incomplete: {configured_gripper}")

    # Give the RG2 collision pads a dedicated high-friction physics material;
    # visual material bindings use a separate USD material purpose.
    grip_material = PhysicsMaterial(
        prim_path="/World/Physics_Materials/rg2_grip_material",
        static_friction=GRIP_CONTACT_STATIC_FRICTION,
        dynamic_friction=GRIP_CONTACT_DYNAMIC_FRICTION,
        restitution=0.0,
    )
    grip_link_names = {
        "rg2_left_inner_knuckle",
        "rg2_right_outer_knuckle",
        "rg2_left_outer_knuckle",
        "rg2_left_inner_finger",
        "rg2_right_inner_finger",
        "rg2_right_inner_knuckle",
    }
    grip_links = 0
    for prim in stage.Traverse():
        # Bind to each moving link so any imported or proxy collision geometry
        # inherits the high-friction physics material.
        if not str(prim.GetPath()).startswith("/World/ServingRobot"):
            continue
        if prim.GetName() not in grip_link_names:
            continue
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            grip_material.material,
            bindingStrength=UsdShade.Tokens.strongerThanDescendants,
            materialPurpose="physics",
        )
        grip_links += 1
    print(
        f"[mobile robot] RG2 high-friction links={grip_links}/6 "
        f"drive_max_force={GRIPPER_DRIVE_MAX_FORCE:.0f}N",
        flush=True,
    )


def initialize_robot(articulation_path):
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    # The board's segmented bail adds compound collision cooking.  Allow a few
    # frames for PhysX to finish constructing the full articulation before
    # obtaining its tensor handle; five frames are only ~42 ms at 120 Hz.
    for _ in range(5):
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
        raise RuntimeError(
            f"missing articulation DOFs: {sorted(missing)}; "
            f"selected={articulation_path} actual_dofs={dof_names}"
        )

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


def find_serving_robot_prim(stage, name):
    matches = [
        prim
        for prim in stage.Traverse()
        if prim.GetName() == name
        and str(prim.GetPath()).startswith("/World/ServingRobot")
    ]
    if len(matches) != 1:
        raise RuntimeError(f"expected one serving-robot {name}, got {matches}")
    return matches[0]


def prim_world_pose(prim):
    transform = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
    translation = transform.ExtractTranslation()
    rotation = transform.ExtractRotationQuat()
    imaginary = rotation.GetImaginary()
    return (
        np.array(translation, dtype=float),
        np.array(
            [rotation.GetReal(), imaginary[0], imaginary[1], imaginary[2]],
            dtype=float,
        ),
        transform,
    )


class TrayPizzaPickPlace:
    """Top-down grasp of a folding bail from the deck to a table stand."""

    def __init__(self, stage):
        self._stage = stage
        self._arm_base = find_serving_robot_prim(stage, "base_link")
        self._end_effector = find_serving_robot_prim(stage, "link_6")
        _, _, self._arm_to_world = prim_world_pose(self._arm_base)
        self._up_world = np.array([0.0, 0.0, 1.0])
        # The RG2 approaches downward, so its wrist remains in +Z from the
        # requested finger TCP.  The flat bail initially opens toward +Y.
        self._approach_world = self._up_world.copy()
        self._bail_rest_world = np.array([0.0, 1.0, 0.0])
        self._controller = None
        self._gripper = None
        self._gripper_joint_index = None
        self._phase = 0
        self._phase_steps = 0
        self._settled_steps = 0
        self._targets = []
        self._initial_wrist = None
        self._initial_orientation = None
        self._dish_released_for_lift = True
        self.done = False
        self.failed = False

        stand_color = np.array([0.18, 0.22, 0.25])
        serving_shelf = find_serving_robot_prim(stage, "serving_shelf_link")
        for index, (position, scale) in enumerate(UPPER_TRAY_COLLIDERS):
            proxy = FixedCuboid(
                prim_path=f"/World/UpperTrayCollisionProxy_{index}",
                name=f"upper_tray_collision_proxy_{index}",
                position=self.local_to_world(position),
                scale=scale,
                visible=False,
            )
            # Each proxy overlaps its original shelf collider by design.
            # Filter only that pair; RG2 links still collide with the proxy.
            UsdPhysics.FilteredPairsAPI.Apply(
                stage.GetPrimAtPath(proxy.prim_path)
            ).CreateFilteredPairsRel().AddTarget(serving_shelf.GetPath())
        FixedCuboid(
            prim_path="/World/TopDeckDishStand",
            name="top_deck_dish_stand",
            position=self.local_to_world(TOP_DECK_STAND_LOCAL),
            scale=DISH_STAND_SCALE,
            color=stand_color,
        )
        FixedCuboid(
            prim_path="/World/TableDishStand",
            name="table_dish_stand",
            position=self.local_to_world(TABLE_STAND_LOCAL),
            scale=DISH_STAND_SCALE,
            color=stand_color,
        )

        dish_material = PhysicsMaterial(
            prim_path="/World/Physics_Materials/serving_dish_material",
            static_friction=GRIP_CONTACT_STATIC_FRICTION,
            dynamic_friction=GRIP_CONTACT_DYNAMIC_FRICTION,
            restitution=0.0,
        )
        # Invisible rigid-body carrier; visible geometry comes exclusively
        # from the authored board and pizza children below.
        self._dish = DynamicCylinder(
            prim_path="/World/ServingDish",
            name="serving_dish",
            position=self.local_to_world(TOP_DECK_DISH_LOCAL),
            radius=DISH_RADIUS,
            height=DISH_HEIGHT,
            visible=True,
            mass=SERVING_DISH_MASS,
            physics_material=dish_material,
        )
        dish_body = UsdPhysics.RigidBodyAPI.Get(stage, self._dish.prim_path)
        if not dish_body:
            raise RuntimeError("failed to create /World/ServingDish rigid body")
        self._dish_body = dish_body
        dish_prim = stage.GetPrimAtPath(self._dish.prim_path)
        dish_physx_body = PhysxSchema.PhysxRigidBodyAPI.Apply(dish_prim)
        dish_physx_body.CreateSolverPositionIterationCountAttr(64)
        dish_physx_body.CreateSolverVelocityIterationCountAttr(16)
        dish_physx_body.CreateEnableGyroscopicForcesAttr(True)
        proxy_collision = UsdPhysics.CollisionAPI.Get(
            stage, self._dish.prim_path
        )
        if proxy_collision:
            # DynamicCylinder is retained only as the rigid-body carrier.  Its
            # synthetic cylinder collider is replaced by the real board mesh.
            proxy_collision.GetCollisionEnabledAttr().Set(False)
        # DynamicCylinder creates a 50%-gray PreviewSurface and binds it with
        # strongerThanDescendants.  That inherited binding was overriding all
        # plate and pizza child materials.  The cylinder is collision-only,
        # so replace its visual binding before authoring the visible children.
        UsdShade.MaterialBindingAPI(dish_prim).UnbindAllBindings()
        proxy_material = UsdShade.Material.Define(
            stage, "/World/Looks/ServingDishCollisionInvisible"
        )
        proxy_shader = UsdShade.Shader.Define(
            stage, "/World/Looks/ServingDishCollisionInvisible/Shader"
        )
        proxy_shader.CreateIdAttr("UsdPreviewSurface")
        proxy_shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Float3).Set(
            Gf.Vec3f(0.0, 0.0, 0.0)
        )
        proxy_shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.0)
        proxy_shader.CreateInput(
            "opacityThreshold", Sdf.ValueTypeNames.Float
        ).Set(0.001)
        proxy_material.CreateSurfaceOutput().ConnectToSource(
            proxy_shader.ConnectableAPI(), "surface"
        )
        # The weak binding hides the cylinder itself but allows the explicit
        # plate and pizza materials on descendant meshes to render normally.
        UsdShade.MaterialBindingAPI(dish_prim).Bind(
            proxy_material,
            bindingStrength=UsdShade.Tokens.weakerThanDescendants,
        )
        # The board stays dynamic until pickup support is engaged in phase 4.
        # The timeline is still stopped here, so it cannot fall before setup.
        dish_body.GetKinematicEnabledAttr().Set(False)
        # Do not use visibility=invisible here: USD visibility is inherited
        # and would hide the referenced plate and pizza children as well.
        UsdGeom.Gprim(stage.GetPrimAtPath(self._dish.prim_path)).CreateDisplayOpacityAttr(
            [0.0]
        )

        # Parent the complete handle below the board rigid body.  Its folding
        # transform remains local, while every board translation is inherited
        # automatically so the two can never separate during a lift.
        bail_visual_root = UsdGeom.Xform.Define(
            stage, "/World/ServingDish/PizzaBailVisual"
        )
        author_pizza_board_with_bail(
            stage,
            "/World/ServingDish/WoodenPizzaBoard",
            "/World/ServingDish/PizzaBailVisual",
            dish_material,
        )
        bail_geometry_prim = stage.GetPrimAtPath(
            "/World/ServingDish/PizzaBailVisual/Bail"
        )
        self._bail_geometry_translate = bail_geometry_prim.GetAttribute(
            "xformOp:translate"
        )
        self._bail_geometry_orient = bail_geometry_prim.GetAttribute(
            "xformOp:orient"
        )
        pizza_asset = UsdGeom.Xform.Define(stage, "/World/ServingDish/PizzaAsset")
        # Source diameter is about 45 cm.  At 0.72 scale it becomes 32.4 cm on
        # the 39 cm wooden deck.  Also cancel the source mesh's off-centre
        # pivot and raise its lowest crust geometry above the plate.  This GLB
        # stores the pizza face in X-Z with thickness along Y; rotate +Y onto
        # world +Z so the pizza lies flat instead of standing vertically.
        pizza_asset.AddTranslateOp().Set(Gf.Vec3d(*PIZZA_CENTER_OFFSET))
        pizza_geometry = UsdGeom.Xform.Define(
            stage, "/World/ServingDish/PizzaAsset/Geometry"
        )
        pizza_geometry.AddRotateXOp().Set(90.0)
        pizza_geometry.AddScaleOp().Set(
            Gf.Vec3f(*([PIZZA_ASSET_SCALE] * 3))
        )
        author_colored_glb_meshes(
            stage, pizza_geometry.GetPath(), SUPREME_PIZZA_GLB
        )
        if not dish_prim.IsValid():
            raise RuntimeError("/World/ServingDish was not authored")
        print(
            f"[serving] authored dish prim={dish_prim.GetPath()} "
            f"bail={bail_visual_root.GetPath()} "
            f"spawn={np.round(self.local_to_world(TOP_DECK_DISH_LOCAL), 4)}",
            flush=True,
        )

    def local_to_world(self, position):
        point = self._arm_to_world.Transform(Gf.Vec3d(*map(float, position)))
        return np.array(point, dtype=float)

    def local_vector_to_world(self, vector):
        value = self._arm_to_world.TransformDir(Gf.Vec3d(*map(float, vector)))
        result = np.array(value, dtype=float)
        return result / np.linalg.norm(result)

    def _tcp_to_wrist(self, tcp_position):
        # The assembled RG2 forward axis points opposite approach_world;
        # link_6 therefore stays this far outside the requested finger TCP.
        return tcp_position + self._approach_world * RG2_TCP_LENGTH

    def _command_gripper(self, articulation, target):
        """Drive the one independent RG2 mimic joint to an absolute target."""
        articulation.apply_action(
            ArticulationAction(
                joint_positions=np.asarray([target], dtype=float),
                joint_indices=np.asarray(
                    [self._gripper_joint_index], dtype=np.int32
                ),
            )
        )

    def _set_supported_bail_angle(self, angle, board_position=None):
        """Rotate the bail about its scripted, fixed hinge line."""
        half_angle = 0.5 * angle
        # The rigid-body origin is at the board centre while the hinge is
        # BAIL_HINGE_Z above it.  Compensate the root translation so that an
        # X-axis rotation leaves that hinge line fixed in world space.
        if board_position is None:
            board_position = self._pick_start_position
        local_translation = (
            self._bail_rest_world * (BAIL_HINGE_Z * np.sin(angle))
            + self._up_world * (BAIL_HINGE_Z * (1.0 - np.cos(angle)))
        )
        self._bail_geometry_translate.Set(
            Gf.Vec3d(*map(float, local_translation))
        )
        self._bail_geometry_orient.Set(
            Gf.Quatf(
                float(np.cos(half_angle)),
                float(np.sin(half_angle)),
                0.0,
                0.0,
            )
        )

    def _log_grasp_alignment(self):
        """Report world bounds for the grip and two inner finger links."""
        cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            [UsdGeom.Tokens.default_],
            useExtentsHint=True,
        )
        paths = [
            Sdf.Path(
                "/World/ServingDish/PizzaBailVisual/Bail/CenterGrip"
            ),
            find_serving_robot_prim(
                self._stage, "rg2_left_inner_finger"
            ).GetPath(),
            find_serving_robot_prim(
                self._stage, "rg2_right_inner_finger"
            ).GetPath(),
        ]
        for path in paths:
            prim = self._stage.GetPrimAtPath(path)
            position, orientation, _ = prim_world_pose(prim)
            bounds = cache.ComputeWorldBound(
                prim
            ).ComputeAlignedRange()
            minimum = np.asarray(bounds.GetMin(), dtype=float)
            maximum = np.asarray(bounds.GetMax(), dtype=float)
            print(
                f"[serving-grasp-bounds] prim={path} "
                f"origin={np.round(position, 4)} "
                f"orientation={np.round(orientation, 4)} "
                f"center={np.round(0.5 * (minimum + maximum), 4)} "
                f"size={np.round(maximum - minimum, 4)}",
                flush=True,
            )

    def _enter_phase(self, phase, articulation):
        self._phase = phase
        self._phase_steps = 0
        self._settled_steps = 0
        names = [
            "move above flat bail grip",
            "settle above flat bail grip",
            "descend onto centre grip",
            "close gripper on flat centre grip",
            "rotate bail upright, then lift board",
            "hold after vertical lift",
            "transfer above table",
            "lower to table stand",
            "insert at destination",
            "open gripper",
            "final retreat",
            "done",
        ]
        print(f"[serving-bail] phase={phase} {names[phase]}", flush=True)
        if phase == 3:
            self._command_gripper(articulation, GRIPPER_CLOSE[0])
        elif phase == 4:
            # The board is still resting on its stand while the robot folds
            # the handle upright.  Holding only the board body kinematic here
            # models that support and prevents trajectory-tracking error from
            # prematurely dragging the payload into the air.
            self._dish_body.GetKinematicEnabledAttr().Set(True)
            self._set_supported_bail_angle(0.0)
            self._dish_released_for_lift = False
        elif phase == 5 and VERTICAL_LIFT_TEST:
            board_position = np.asarray(self._dish.get_world_pose()[0])
            board_lift = float(
                np.dot(board_position - self._pick_start_position, self._up_world)
            )
            if board_lift >= 0.08:
                self.done = True
                print(
                    "[serving-bail] vertical lift test complete; "
                    f"board_lift={board_lift:.4f}m, holding pose",
                    flush=True,
                )
            else:
                self.failed = True
                print(
                    "[serving-bail] VERTICAL LIFT FAILED: wrist rose but "
                    f"board_lift={board_lift:.4f}m; simulation remains open "
                    "for inspection",
                    flush=True,
                )
        elif phase == 9:
            self._command_gripper(articulation, GRIPPER_OPEN[0])
        elif phase == 11:
            self.done = True
            print("[serving-bail] pizza delivered to TableSet_00", flush=True)

    def initialize(self, articulation, dof_names):
        missing = [name for name in GRIPPER_JOINTS[:1] if name not in dof_names]
        if missing:
            raise RuntimeError(
                f"missing RG2 drive DOF {missing}; run once with "
                "MOBILE_DEMO_REIMPORT=1"
            )
        self._dish.initialize()
        spawn_position = self.local_to_world(TOP_DECK_DISH_LOCAL)
        self._dish.set_world_pose(position=spawn_position)
        dish_body = UsdPhysics.RigidBodyAPI.Get(self._stage, self._dish.prim_path)
        self._dish_body = dish_body
        dish_body.GetKinematicEnabledAttr().Set(False)
        # Newly initialized rigid bodies already have zero velocity.  Do not
        # write velocity in this frame: PhysX applies state changes on the next
        # update and rejects velocity writes until then.

        self._gripper = ParallelGripper(
            end_effector_prim_path=str(self._end_effector.GetPath()),
            joint_prim_names=GRIPPER_JOINTS,
            joint_opened_positions=GRIPPER_OPEN,
            joint_closed_positions=GRIPPER_CLOSE,
            action_deltas=GRIPPER_DELTA,
            use_mimic_joints=True,
        )
        self._gripper.initialize(
            articulation_apply_action_func=articulation.apply_action,
            get_joint_positions_func=articulation.get_joint_positions,
            set_joint_positions_func=articulation.set_joint_positions,
            dof_names=dof_names,
        )
        self._gripper_joint_index = dof_names.index(GRIPPER_JOINTS[0])
        self._gripper.set_joint_positions(GRIPPER_OPEN)

        arm_position, arm_orientation, _ = prim_world_pose(self._arm_base)
        self._controller = RMPFlowController(
            name="tray_pizza_top_down_rmpflow",
            robot_articulation=articulation,
            urdf_path=str(M0609_RMPFLOW_URDF),
            robot_description_path=str(M0609_DESCRIPTION),
            rmpflow_config_path=str(M0609_RMPFLOW_CONFIG),
            end_effector_frame_name="link_6",
        )
        self._controller.rmp_flow.set_robot_base_pose(
            robot_position=arm_position,
            robot_orientation=arm_orientation,
        )
        self._initial_wrist, self._initial_orientation, _ = prim_world_pose(
            self._end_effector
        )

        pick_center = self._dish.get_world_pose()[0]
        self._pick_start_position = np.asarray(pick_center, dtype=float).copy()
        place_center = self.local_to_world(TABLE_DISH_LOCAL)
        rest_grip_center = (
            pick_center
            + self._bail_rest_world * BAIL_GRIP_REACH
            + self._up_world * BAIL_HINGE_Z
        )
        pick_grasp = (
            rest_grip_center
            - self._approach_world * BAIL_GRIP_INSERT_DEPTH
        )
        upright_grip_center = (
            pick_center + self._up_world * BAIL_UPRIGHT_GRIP_HEIGHT
        )
        pick_upright = (
            upright_grip_center
            - self._approach_world * BAIL_GRIP_INSERT_DEPTH
        )
        pick_vertical = (
            pick_upright + self._up_world * TOP_DECK_SAFE_CLEARANCE
        )
        place_grasp = (
            place_center
            + self._up_world * BAIL_UPRIGHT_GRIP_HEIGHT
            - self._approach_world * BAIL_GRIP_INSERT_DEPTH
        )
        pick_outside = (
            pick_grasp
            + self._approach_world * TOP_DECK_APPROACH_DISTANCE
        )
        pick_safe = pick_vertical.copy()
        # Start directly above the flat grip and descend along world -Z.
        initial_approach = pick_outside.copy()
        place_pre = (
            place_grasp
            + self._approach_world * PLACE_APPROACH_DISTANCE
        )

        self._pick_rest_tcp = pick_grasp.copy()
        self._pick_upright_tcp = pick_upright.copy()
        self._pick_lift_tcp = pick_vertical.copy()

        # Phases 0-2 descend onto the flat grip.  Phase 4 generates its own
        # quarter-circle hinge path followed by a vertical board lift.
        self._targets = [
            initial_approach,
            pick_outside,
            pick_grasp,
            None,
            pick_vertical,
            pick_safe,
            place_pre + self._up_world * TRANSFER_CLEARANCE,
            place_pre,
            place_grasp,
            None,
            place_pre,
        ]
        print(
            "[serving-bail] ready "
            f"board={np.round(pick_center, 4)} "
            f"top_approach={np.round(initial_approach, 4)} "
            f"flat_grip={np.round(rest_grip_center, 4)} "
            f"upright_grip={np.round(upright_grip_center, 4)} "
            f"lift_clearance={np.round(pick_safe, 4)} "
            f"grasp_tcp={np.round(pick_grasp, 4)} "
            f"destination={np.round(place_center, 4)}",
            flush=True,
        )
        self._enter_phase(0, articulation)

    def step(self, articulation):
        if self.done or self.failed:
            return
        self._phase_steps += 1
        if self._phase in (3, 9):
            wait_steps = 120 if self._phase == 3 else 90
            target = GRIPPER_CLOSE[0] if self._phase == 3 else GRIPPER_OPEN[0]
            # Reassert the absolute target throughout the dwell.  This avoids
            # losing a one-shot command when the articulation controller or a
            # mimic constraint updates in the same simulation frame.
            self._command_gripper(articulation, target)
            if self._phase_steps % 30 == 0:
                actual = float(
                    articulation.get_joint_positions()[self._gripper_joint_index]
                )
                print(
                    f"[serving-gripper] phase={self._phase} "
                    f"target={target:.3f} actual={actual:.3f}",
                    flush=True,
                )
            if self._phase_steps >= wait_steps:
                if self._phase == 3:
                    self._log_grasp_alignment()
                self._enter_phase(self._phase + 1, articulation)
            return

        tcp_target = self._targets[self._phase]
        target_orientation = BAIL_GRASP_EE_ORIENTATION
        transition_complete = True
        if self._phase == 4:
            upright_settle_end = (
                BAIL_RAISE_TRANSITION_STEPS + BAIL_UPRIGHT_SETTLE_STEPS
            )
            if self._phase_steps <= BAIL_RAISE_TRANSITION_STEPS:
                raw_amount = min(
                    1.0,
                    self._phase_steps / BAIL_RAISE_TRANSITION_STEPS,
                )
                amount = raw_amount * raw_amount * (3.0 - 2.0 * raw_amount)
                angle = 0.5 * np.pi * amount
                self._set_supported_bail_angle(angle)
                grip_center = (
                    self._pick_start_position
                    + self._bail_rest_world
                    * (BAIL_GRIP_REACH * np.cos(angle))
                    + self._up_world
                    * (
                        BAIL_HINGE_Z
                        + BAIL_GRIP_REACH * np.sin(angle)
                    )
                )
                tcp_target = (
                    grip_center
                    - self._approach_world * BAIL_GRIP_INSERT_DEPTH
                )
                transition_complete = False
            elif self._phase_steps <= upright_settle_end:
                # Hold the top-down grasp at the raised handle's centre while
                # the board remains supported on the top-deck stand.
                self._set_supported_bail_angle(0.5 * np.pi)
                tcp_target = self._pick_upright_tcp
                transition_complete = False
            else:
                if not self._dish_released_for_lift:
                    self._set_supported_bail_angle(0.5 * np.pi)
                    wrist_position, _, _ = prim_world_pose(
                        self._end_effector
                    )
                    actual_tcp = (
                        wrist_position
                        - self._approach_world * RG2_TCP_LENGTH
                    )
                    # Capture a zero-jump rigid translation offset at the
                    # instant lifting starts.  The non-colliding visual bail
                    # and its board then follow the measured gripper motion,
                    # not an ideal target that the arm may lag behind.
                    self._dish_grasp_offset = (
                        self._pick_start_position - actual_tcp
                    )
                    self._dish_released_for_lift = True
                    print(
                        "[serving-bail] upright handle settled; "
                        "locked to gripper; beginning vertical lift",
                        flush=True,
                    )
                lift_step = self._phase_steps - upright_settle_end
                raw_amount = min(
                    1.0, lift_step / DISH_LIFT_TRANSITION_STEPS
                )
                amount = raw_amount * raw_amount * (3.0 - 2.0 * raw_amount)
                tcp_target = (
                    self._pick_upright_tcp
                    + self._up_world * TOP_DECK_SAFE_CLEARANCE * amount
                )
                transition_complete = raw_amount >= 1.0
        wrist_target = self._tcp_to_wrist(tcp_target)
        if self._phase == 0:
            raw_amount = min(1.0, self._phase_steps / INITIAL_TRANSITION_STEPS)
            # Smoothstep has zero slope at both ends, preventing the initial
            # Cartesian target from kicking the arm drives.
            amount = raw_amount * raw_amount * (3.0 - 2.0 * raw_amount)
            wrist_target = (
                self._initial_wrist
                + amount * (wrist_target - self._initial_wrist)
            )
            target_orientation = quaternion_slerp(
                self._initial_orientation,
                BAIL_GRASP_EE_ORIENTATION,
                amount,
            )
            transition_complete = raw_amount >= 1.0
        action = self._controller.forward(
            target_end_effector_position=wrist_target,
            target_end_effector_orientation=target_orientation,
        )
        articulation.apply_action(action)
        # RMPFlow updates the arm every frame.  Reapply the independent RG2
        # target after that update throughout the complete carry sequence;
        # otherwise the one-shot close command can lose preload as the bail is
        # raised and the payload begins to move.
        if 4 <= self._phase <= 8:
            self._command_gripper(articulation, GRIPPER_CLOSE[0])

        ee_position, _, _ = prim_world_pose(self._end_effector)
        if self._phase == 4 and self._dish_released_for_lift:
            actual_tcp = (
                ee_position - self._approach_world * RG2_TCP_LENGTH
            )
            board_position = actual_tcp + self._dish_grasp_offset
            self._dish.set_world_pose(position=board_position)
            self._set_supported_bail_angle(
                0.5 * np.pi,
                board_position=board_position,
            )
        error = float(np.linalg.norm(wrist_target - ee_position))
        if self._phase == 4 and self._phase_steps % 60 == 0:
            board_position = np.asarray(self._dish.get_world_pose()[0])
            grip_position, _, _ = prim_world_pose(
                self._stage.GetPrimAtPath(
                    "/World/ServingDish/PizzaBailVisual/Bail/CenterGrip"
                )
            )
            board_lift = float(
                np.dot(
                    board_position - self._pick_start_position,
                    self._up_world,
                )
            )
            print(
                "[serving-lift-progress] "
                f"step={self._phase_steps} "
                f"wrist={np.round(ee_position, 4)} "
                f"target={np.round(wrist_target, 4)} "
                f"error={error:.4f}m board={np.round(board_position, 4)} "
                f"grip={np.round(grip_position, 4)} "
                f"board_lift={board_lift:.4f}m",
                flush=True,
            )
        position_tolerance = 0.03 if self._phase == 0 else 0.025
        self._settled_steps = (
            self._settled_steps + 1
            if transition_complete and error < position_tolerance
            else 0
        )
        if self._settled_steps >= 15:
            self._enter_phase(self._phase + 1, articulation)
        elif self._phase_steps >= (900 if self._phase == 4 else 600):
            message = (
                f"bail serving phase {self._phase} did not converge; "
                f"position error={error:.4f} m"
            )
            if EXIT_ON_MOTION_ERROR:
                raise RuntimeError(message)
            self.failed = True
            print(
                f"[serving-bail] STOPPED: {message}; "
                "simulation remains open for inspection",
                flush=True,
            )

    def close(self):
        pass


def main():
    enable_urdf_importer()
    prepare_antigravity_food_assets()
    import_robot_usd()
    stage = open_restaurant_and_reference_robot()
    attach_m0609_visuals(stage)
    attach_fixed_table_depth_camera(stage)
    connect_table_camera_ros2(stage)
    pick_place = TrayPizzaPickPlace(stage) if PICK_PLACE_ENABLED else None
    if pick_place is None:
        print(
            "[serving] disabled because MOBILE_DEMO_PICK_PLACE=0; "
            "no dish will be spawned",
            flush=True,
        )
    for _ in range(10):
        simulation_app.update()
    configure_joint_drives(stage)
    articulation_path = find_articulation_root(stage)
    add_parking_brake(stage, articulation_path)
    configure_physics_stability(stage, articulation_path)
    articulation, dof_names = initialize_robot(articulation_path)
    if pick_place is not None:
        pick_place.initialize(articulation, dof_names)
    open_table_camera_preview()
    if os.environ.get("MOBILE_DEMO_PRINT_BOUNDS", "0") == "1":
        print_arm_visual_bounds(stage)
    run_optional_diagnostic(articulation, dof_names)
    run_stability_check(articulation, dof_names)

    if os.environ.get("MOBILE_DEMO_EXIT_AFTER_READY", "0") == "1":
        if pick_place is not None:
            pick_place.close()
        simulation_app.close()
        return

    while simulation_app.is_running():
        simulation_app.update()
        if pick_place is not None:
            pick_place.step(articulation)
        # Keep the GUI event loop responsive without throttling it to the old
        # uneven 16 ms cadence.
        time.sleep(0.010)

    if pick_place is not None:
        pick_place.close()


if __name__ == "__main__":
    try:
        main()
    except BaseException:
        traceback.print_exc()
        raise
    finally:
        simulation_app.close()
