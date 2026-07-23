"""Physical pizza-board serving task for the Ridgeback/M0609/RG2 demo.

The module owns the pizza-board geometry, hinged bail physics, RG2 grasp
configuration, and the complete tray-to-table delivery state machine.
It must be imported only after Isaac Sim's SimulationApp has been created.
"""

import json
import os
import struct
from pathlib import Path

import numpy as np
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.api.objects import DynamicCylinder
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot_motion.motion_generation.lula.kinematics import (
    LulaKinematicsSolver,
)
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade

from isaac_scene_utils import find_serving_robot_prim, prim_world_pose
from m0609_rmpflow_controller import RMPFlowController


WORKSPACE = Path(
    os.environ.get("COBOT3_WS", Path(__file__).resolve().parents[1])
).resolve()
RMPFLOW_DIR = WORKSPACE / "isaacpjt/M0609/rmpflow"
ARM_JOINTS = [f"joint_{index}" for index in range(1, 7)]
SUPREME_PIZZA_GLB = (
    WORKSPACE / "assets/source_food/supreme_pizza_jarlan_perez.glb"
)


# Tray-to-table serving motion, adapted from M0609/4_pick_place.py.
PICK_PLACE_ENABLED = os.environ.get("MOBILE_DEMO_PICK_PLACE", "1") == "1"
VERTICAL_LIFT_TEST = (
    os.environ.get("MOBILE_DEMO_VERTICAL_LIFT_TEST", "1") == "1"
)
# Temporary hinge-only diagnostic: keep the pizza board fixed to the top deck
# so RG2 must rotate the bail relative to the board instead of lifting both.
ANCHOR_BOARD_FOR_BAIL_TEST = (
    os.environ.get("MOBILE_DEMO_ANCHOR_BOARD", "0") == "1"
)
EXIT_ON_MOTION_ERROR = (
    os.environ.get("MOBILE_DEMO_EXIT_ON_MOTION_ERROR", "0") == "1"
)
GRIPPER_JOINTS = ["rg2_finger_joint", "rg2_right_inner_knuckle_joint"]
# parse_mimic=True leaves only rg2_finger_joint independently driven.  The
# other five RG2 joints follow it through PhysX mimic constraints, therefore
# ParallelGripper must receive one value (not one value per physical finger).
GRIPPER_OPEN = np.array([0.0])
# The form-lock grip pad is 26 mm thick.  Command a modest preload and let the
# ±Y rails carry roll torque instead of using excessive normal force.
GRIPPER_CLOSE = np.array([1.06])
GRIPPER_DELTA = np.array([-1.06])
GRIPPER_DRIVE_STIFFNESS = 30000.0
GRIPPER_DRIVE_DAMPING = 5000.0
GRIPPER_DRIVE_MAX_FORCE = 40.0
GRIP_CONTACT_STATIC_FRICTION = 6.0
GRIP_CONTACT_DYNAMIC_FRICTION = 5.0
M0609_RMPFLOW_URDF = (
    WORKSPACE / "isaacpjt/M0609/doosan-robot2/urdf/m0609_isaac_sim.urdf"
)
M0609_DESCRIPTION = RMPFLOW_DIR / "m0609_description.yaml"
M0609_RMPFLOW_CONFIG = RMPFLOW_DIR / "m0609_rmpflow_common.yaml"
# Coordinates are relative to the M0609 base.  Large pizza serviceware belongs
# on the open top deck, away from the internal tray reserved for cans and small
# personal plates.  The board now rests directly on the top deck; the old
# 8 cm support block is removed so it cannot snag during a vertical lift.
TOP_DECK_DISH_LOCAL = np.array([-0.38, 0.0, -0.030])
SLIDING_TRAY_JOINTS = (
    "upper_tray_left_slide_joint",
    "upper_tray_right_slide_joint",
)
SLIDING_TRAY_EXTENSION = 0.25
# Three seconds at the 120 Hz scene rate keeps peak smoothstep velocity below
# the URDF's 0.15 m/s joint limit, which is gentle enough for loaded drinks.
SLIDING_TRAY_DEPLOY_STEPS = 360
# Reachable centreline target on the destination tabletop.  At local X=+0.55
# the 39 cm board remains fully inside the table edge while the arm can first
# translate there at its lifted height, then descend vertically.  The former
# 8 cm support block is gone, so place directly on the table collider.
TABLE_DISH_LOCAL = np.array([0.55, 0.0, -0.14568])
# RG2-specific wooden pizza board.  A rigid U-shaped bail carries the board
# from both ±Y edges so its centre of mass hangs below the grasp point.
BOARD_RADIUS = 0.195
BOARD_THICKNESS = 0.018
BOARD_BAIL_X = 0.0
# Author the folded bail toward robot-local -X (the rear of the chassis).
# The complete board assembly inherits the robot yaw, so this remains correct
# when the restaurant scene changes the initial chassis heading.
BOARD_BAIL_FOLD_X_SIGN = -1.0
BOARD_BAIL_FOLD_ARM_X_SIGN = -1.0
BOARD_BAIL_HALF_SPAN = 0.165
BOARD_BAIL_HEIGHT = 0.165
BOARD_BAIL_MIN_ANGLE_DEG = 10.0
BOARD_BAIL_ROD_RADIUS = 0.006
BOARD_BAIL_MASS = 0.25
BOARD_BAIL_GRIP_SIZE = np.array([0.026, 0.090, 0.040])
BOARD_BAIL_COLLAR_SIZE = np.array([0.034, 0.012, 0.048])
BOARD_BAIL_STOP_SIZE = np.array([0.028, 0.028, 0.027])
BOARD_BAIL_STOP_Y = 0.105
# The declared TCP is near the fingertips.  Descend past the crossbar centre
# so the round sleeve reaches the deeper, parallel section of the RG2 pads.
BOARD_BAIL_GRASP_INSERT_DEPTH = 0.020
BOARD_HANDLE_CROSS_INSERT_X = BOARD_BAIL_X
# Pizza + wooden board combined mass.
BOARD_TEST_MASS = 2.0
MAGNET_MAX_FORCE = float(os.environ.get("MOBILE_MAGNET_MAX_FORCE", "25.0"))
MAGNET_CAPTURE_RANGE = float(
    os.environ.get("MOBILE_MAGNET_CAPTURE_RANGE", "0.003")
)
MAGNET_LATERAL_CAPTURE_RADIUS = 0.060
STEEL_PLATE_RADIUS = 0.025 * 1.15
PIZZA_ASSET_SCALE = 0.72
DISH_RADIUS = BOARD_RADIUS
# DynamicCylinder is only the rigid-body carrier; its collider is disabled.
# The visible board mesh and U-shaped bail provide the actual collisions.
DISH_HEIGHT = BOARD_THICKNESS
DISH_GRASP_HEIGHT_OFFSET = 0.0
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
HORIZONTAL_APPROACH_DISTANCE = 0.08
TOP_DECK_APPROACH_DISTANCE = 0.10
# Lift 20 cm before the J1 half-turn so the 39 cm board clears the top deck.
TOP_DECK_SAFE_CLEARANCE = 0.20
INITIAL_TRANSITION_STEPS = 240
# Two seconds at 120 Hz for each 40-degree bail segment.  Smooth continuous
# arc targets prevent the handle from being impulsively flung past RG2.
BAIL_ARC_STEPS = 240
BAIL_PHASE4_OVERSHOOT_DEG = 60.0
BAIL_UPRIGHT_ACCEPT_DEG = 85.0
VERTICAL_LIFT_STEPS = 360
J1_HALF_TURN_STEPS = 600
# Camera mast now occupies robot-left (+Y), so carry the raised pizza through
# the opposite half-turn direction to keep the board/arm clear of the mast.
J1_DELIVERY_TURN = np.pi
PLACEMENT_DESCENT_STEPS = 360
RG2_TCP_LENGTH = 0.231066
# Keep local X (finger separation) upward and point the assembled RG2 forward
# axis (+Z) toward world +X.  The hand therefore starts on the handle's -X
# side and inserts in world +X as requested.
HORIZONTAL_EE_ORIENTATION = np.array(
    [0.0, np.sqrt(0.5), 0.0, np.sqrt(0.5)], dtype=float
)
# Rotate 180 degrees about world X: the assembled RG2 nose points down while
# its local X finger-separation axis remains aligned with world X, suitable
# for pinching the world-Y bail crossbar from directly above.
# The chassis yaw and mirrored J1 seed cancel each other, reproducing the
# proven world-space downward RG2 pose used before the front-arm conversion.
VERTICAL_EE_ORIENTATION = np.array([0.0, 1.0, 0.0, 0.0], dtype=float)


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


def author_magnetic_steel_plate(stage):
    """Add only the board-side steel target; the magnet is in robot USD."""
    plate = UsdGeom.Cylinder.Define(
        stage, "/World/ServingDish/MagneticSteelPlate"
    )
    plate.CreateAxisAttr(UsdGeom.Tokens.z)
    plate.CreateRadiusAttr(STEEL_PLATE_RADIUS)
    plate.CreateHeightAttr(0.001)
    plate.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.5 * BOARD_THICKNESS))
    plate.CreateDisplayColorAttr([Gf.Vec3f(0.20, 0.22, 0.24)])
    print(
        "[serving-magnet] board steel plate authored; tray magnet comes "
        "from robot USD "
        f"max_force={MAGNET_MAX_FORCE:.1f}N "
        f"capture_range={MAGNET_CAPTURE_RANGE * 1000.0:.1f}mm",
        flush=True,
    )

def author_wooden_pizza_board(stage, mesh_path, physics_material):
    """Create one watertight round board with a two-sided lifting bail."""
    circle_angles = np.linspace(0.0, 2.0 * np.pi, 64, endpoint=False)
    outline = [
        (BOARD_RADIUS * np.cos(angle), BOARD_RADIUS * np.sin(angle))
        for angle in circle_angles
    ]
    half_thickness = BOARD_THICKNESS * 0.5
    points = [Gf.Vec3f(0.0, 0.0, -half_thickness)]
    points.extend(Gf.Vec3f(float(x), float(y), -half_thickness) for x, y in outline)
    top_center = len(points)
    points.append(Gf.Vec3f(0.0, 0.0, half_thickness))
    top_start = len(points)
    points.extend(Gf.Vec3f(float(x), float(y), half_thickness) for x, y in outline)

    face_counts = []
    face_indices = []
    count = len(outline)
    for index in range(count):
        following = (index + 1) % count
        # Bottom, top, then the actual outline side wall.
        face_counts.append(3)
        face_indices.extend([0, 1 + following, 1 + index])
        face_counts.append(3)
        face_indices.extend([top_center, top_start + index, top_start + following])
        face_counts.append(4)
        face_indices.extend(
            [1 + index, 1 + following, top_start + following, top_start + index]
        )

    mesh = UsdGeom.Mesh.Define(stage, mesh_path)
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr(face_counts)
    mesh.CreateFaceVertexIndicesAttr(face_indices)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateDisplayColorAttr([Gf.Vec3f(0.66, 0.40, 0.18)])

    material = UsdShade.Material.Define(stage, "/World/Looks/WoodenPizzaBoard")
    shader = UsdShade.Shader.Define(stage, "/World/Looks/WoodenPizzaBoard/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Float3).Set(
        Gf.Vec3f(0.66, 0.40, 0.18)
    )
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.72)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    material.CreateSurfaceOutput().ConnectToSource(
        shader.ConnectableAPI(), "surface"
    )
    binding = UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim())
    binding.Bind(
        material,
        bindingStrength=UsdShade.Tokens.strongerThanDescendants,
    )
    binding.Bind(
        physics_material.material,
        bindingStrength=UsdShade.Tokens.strongerThanDescendants,
        materialPurpose="physics",
    )
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim()).CreateCollisionEnabledAttr(True)
    UsdPhysics.MeshCollisionAPI.Apply(
        mesh.GetPrim()
    ).CreateApproximationAttr().Set("convexDecomposition")
    # PhysX's automatic contact envelope can be large relative to this 18 mm
    # handle and make an open finger report contact before reaching the visible
    # wood.  Keep collision cooking on the real mesh but reduce that envelope.
    physx_collision = PhysxSchema.PhysxCollisionAPI.Apply(mesh.GetPrim())
    physx_collision.CreateContactOffsetAttr(0.001)
    physx_collision.CreateRestOffsetAttr(0.0)

    # Physical 10-degree rest stops.  The two pads sit below the folded
    # crossbar near the rim, keeping a real gripper-access gap even if the
    # revolute solver starts exactly on its angular limit.
    stop_x = BOARD_BAIL_X + BOARD_BAIL_FOLD_X_SIGN * BOARD_BAIL_HEIGHT * np.cos(
        np.deg2rad(BOARD_BAIL_MIN_ANGLE_DEG)
    )
    stop_z = 0.5 * BOARD_THICKNESS + 0.5 * BOARD_BAIL_STOP_SIZE[2]
    for side, sign in (("MinusY", -1.0), ("PlusY", 1.0)):
        stop_path = f"{mesh_path}_BailStops/{side}"
        stop_root = UsdGeom.Xform.Define(stage, stop_path)
        stop_root.AddTranslateOp().Set(
            Gf.Vec3d(stop_x, sign * BOARD_BAIL_STOP_Y, stop_z)
        )
        stop = UsdGeom.Cube.Define(stage, f"{stop_path}/Geometry")
        stop.CreateSizeAttr(1.0)
        stop.AddScaleOp().Set(Gf.Vec3f(*map(float, BOARD_BAIL_STOP_SIZE)))
        stop.CreateDisplayColorAttr([Gf.Vec3f(0.18, 0.20, 0.22)])
        stop_binding = UsdShade.MaterialBindingAPI.Apply(stop.GetPrim())
        stop_binding.Bind(
            physics_material.material,
            bindingStrength=UsdShade.Tokens.strongerThanDescendants,
            materialPurpose="physics",
        )
        UsdPhysics.CollisionAPI.Apply(
            stop.GetPrim()
        ).CreateCollisionEnabledAttr(True)
        stop_physx = PhysxSchema.PhysxCollisionAPI.Apply(stop.GetPrim())
        stop_physx.CreateContactOffsetAttr(0.001)
        stop_physx.CreateRestOffsetAttr(0.0)

    # One local-Y revolute joint represents the two coaxial physical pivots.
    # The board root carries the robot yaw, so the separate rigid components
    # below receive that same orientation and all geometry stays robot-local.
    dish_prim = stage.GetPrimAtPath("/World/ServingDish")
    _, board_world_orientation, board_to_world = prim_world_pose(dish_prim)
    bail_path = "/World/PizzaBoardBail"
    bail_root = UsdGeom.Xform.Define(stage, bail_path)
    bail_pivot_local = np.array(
        [BOARD_BAIL_X, 0.0, 0.5 * BOARD_THICKNESS + BOARD_BAIL_ROD_RADIUS],
        dtype=float,
    )
    bail_pivot_world = np.asarray(
        board_to_world.Transform(Gf.Vec3d(*map(float, bail_pivot_local))),
        dtype=float,
    )
    bail_root.AddTranslateOp().Set(Gf.Vec3d(*map(float, bail_pivot_world)))
    bail_root.AddOrientOp().Set(
        Gf.Quatf(
            float(board_world_orientation[0]),
            Gf.Vec3f(*map(float, board_world_orientation[1:])),
        )
    )
    UsdPhysics.RigidBodyAPI.Apply(bail_root.GetPrim())
    UsdPhysics.MassAPI.Apply(bail_root.GetPrim()).CreateMassAttr(BOARD_BAIL_MASS)
    bail_physx_body = PhysxSchema.PhysxRigidBodyAPI.Apply(bail_root.GetPrim())
    bail_physx_body.CreateSolverPositionIterationCountAttr(64)
    bail_physx_body.CreateSolverVelocityIterationCountAttr(16)
    # Keep this frame unrotated.  The 10-degree folded geometry is authored
    # from explicit X/Z coordinates below, avoiding all RotateY sign ambiguity.
    bail_geometry_path = f"{bail_path}/GeometryFrame"
    UsdGeom.Xform.Define(stage, bail_geometry_path)
    metal_material = UsdShade.Material.Define(
        stage, "/World/Looks/PizzaBoardBailMetal"
    )
    metal_shader = UsdShade.Shader.Define(
        stage, "/World/Looks/PizzaBoardBailMetal/Shader"
    )
    metal_shader.CreateIdAttr("UsdPreviewSurface")
    metal_shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Float3).Set(
        Gf.Vec3f(0.12, 0.14, 0.16)
    )
    metal_shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.28)
    metal_shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.85)
    metal_material.CreateSurfaceOutput().ConnectToSource(
        metal_shader.ConnectableAPI(), "surface"
    )

    def configure_bail_collider(prim):
        binding = UsdShade.MaterialBindingAPI.Apply(prim)
        binding.Bind(
            metal_material,
            bindingStrength=UsdShade.Tokens.strongerThanDescendants,
        )
        binding.Bind(
            physics_material.material,
            bindingStrength=UsdShade.Tokens.strongerThanDescendants,
            materialPurpose="physics",
        )
        UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
        physx = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        physx.CreateContactOffsetAttr(0.001)
        physx.CreateRestOffsetAttr(0.0)

    def author_bail_rod(name, position, height, axis, radius=None):
        root_path = f"{bail_geometry_path}/{name}"
        root = UsdGeom.Xform.Define(stage, root_path)
        root.AddTranslateOp().Set(Gf.Vec3d(*map(float, position)))
        capsule = UsdGeom.Capsule.Define(stage, f"{root_path}/Geometry")
        capsule.CreateRadiusAttr(
            BOARD_BAIL_ROD_RADIUS if radius is None else float(radius)
        )
        capsule.CreateHeightAttr(float(height))
        capsule.CreateAxisAttr(axis)
        capsule.CreateDisplayColorAttr([Gf.Vec3f(0.12, 0.14, 0.16)])
        configure_bail_collider(capsule.GetPrim())

    def author_bail_box(name, position, size, color):
        root_path = f"{bail_geometry_path}/{name}"
        root = UsdGeom.Xform.Define(stage, root_path)
        root.AddTranslateOp().Set(Gf.Vec3d(*map(float, position)))
        cube = UsdGeom.Cube.Define(stage, f"{root_path}/Geometry")
        cube.CreateSizeAttr(1.0)
        cube.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        configure_bail_collider(cube.GetPrim())

    def author_bail_sphere(name, position):
        root_path = f"{bail_geometry_path}/{name}"
        root = UsdGeom.Xform.Define(stage, root_path)
        root.AddTranslateOp().Set(Gf.Vec3d(*map(float, position)))
        sphere = UsdGeom.Sphere.Define(stage, f"{root_path}/Geometry")
        sphere.CreateRadiusAttr(BOARD_BAIL_ROD_RADIUS)
        sphere.CreateDisplayColorAttr([Gf.Vec3f(0.12, 0.14, 0.16)])
        configure_bail_collider(sphere.GetPrim())

    # Folded pose is explicitly 10 degrees above robot-local -X.
    # Overlapping spheres form each slender arm without an orientation transform.
    folded_angle = np.deg2rad(BOARD_BAIL_MIN_ANGLE_DEG)
    folded_direction = np.array(
        [
            BOARD_BAIL_FOLD_X_SIGN * np.cos(folded_angle),
            0.0,
            np.sin(folded_angle),
        ]
    )
    for side, sign in (("MinusY", -1.0), ("PlusY", 1.0)):
        for index, distance in enumerate(
            np.linspace(0.0, BOARD_BAIL_HEIGHT, 18)
        ):
            position = folded_direction * distance
            position[1] = sign * BOARD_BAIL_HALF_SPAN
            author_bail_sphere(f"Arm_{side}_{index:02d}", position)
    folded_bar_center = folded_direction * BOARD_BAIL_HEIGHT
    author_bail_rod(
        "TopCrossbar",
        folded_bar_center,
        2.0 * BOARD_BAIL_HALF_SPAN,
        UsdGeom.Tokens.y,
    )
    # Small axle collars remain part of the wire bail.  The gripped wooden
    # block authored below is a separate rigid body and cannot slide in Y
    # because its bearing joint removes all translation.
    collar_offset = 0.5 * (
        BOARD_BAIL_GRIP_SIZE[1] + BOARD_BAIL_COLLAR_SIZE[1]
    )
    for side, sign in (("MinusY", -1.0), ("PlusY", 1.0)):
        author_bail_box(
            f"GripCollar_{side}",
            np.array(
                [folded_bar_center[0], sign * collar_offset, folded_bar_center[2]]
            ),
            BOARD_BAIL_COLLAR_SIZE,
            (0.05, 0.05, 0.055),
        )

    grip_block_path = "/World/PizzaBoardGripBlock"
    grip_block_local = bail_pivot_local + folded_bar_center
    grip_block_world = np.asarray(
        board_to_world.Transform(Gf.Vec3d(*map(float, grip_block_local))),
        dtype=float,
    )
    grip_block_root = UsdGeom.Xform.Define(stage, grip_block_path)
    grip_block_root.AddTranslateOp().Set(
        Gf.Vec3d(*map(float, grip_block_world))
    )
    grip_block_root.AddOrientOp().Set(
        Gf.Quatf(
            float(board_world_orientation[0]),
            Gf.Vec3f(*map(float, board_world_orientation[1:])),
        )
    )
    UsdPhysics.RigidBodyAPI.Apply(grip_block_root.GetPrim())
    UsdPhysics.MassAPI.Apply(grip_block_root.GetPrim()).CreateMassAttr(0.08)
    grip_block_physx = PhysxSchema.PhysxRigidBodyAPI.Apply(
        grip_block_root.GetPrim()
    )
    grip_block_physx.CreateSolverPositionIterationCountAttr(64)
    grip_block_physx.CreateSolverVelocityIterationCountAttr(16)
    grip_block = UsdGeom.Cube.Define(stage, f"{grip_block_path}/Geometry")
    grip_block.CreateSizeAttr(1.0)
    grip_block.AddScaleOp().Set(Gf.Vec3f(*map(float, BOARD_BAIL_GRIP_SIZE)))
    grip_block.CreateDisplayColorAttr([Gf.Vec3f(0.55, 0.30, 0.12)])
    grip_binding = UsdShade.MaterialBindingAPI.Apply(grip_block.GetPrim())
    grip_binding.Bind(
        material,
        bindingStrength=UsdShade.Tokens.strongerThanDescendants,
    )
    grip_binding.Bind(
        physics_material.material,
        bindingStrength=UsdShade.Tokens.strongerThanDescendants,
        materialPurpose="physics",
    )
    UsdPhysics.CollisionAPI.Apply(
        grip_block.GetPrim()
    ).CreateCollisionEnabledAttr(True)
    grip_physx = PhysxSchema.PhysxCollisionAPI.Apply(grip_block.GetPrim())
    grip_physx.CreateContactOffsetAttr(0.001)
    grip_physx.CreateRestOffsetAttr(0.0)

    # Bearing between the wooden grip and world-Y crossbar.  RG2 holds the
    # block while the wire bail is free to rotate inside it about the bar axis.
    grip_bearing = UsdPhysics.RevoluteJoint.Define(
        stage, "/World/PizzaBoardGripBearing"
    )
    grip_bearing.CreateBody0Rel().SetTargets([Sdf.Path(bail_path)])
    grip_bearing.CreateBody1Rel().SetTargets([Sdf.Path(grip_block_path)])
    grip_bearing.CreateLocalPos0Attr().Set(
        Gf.Vec3f(*map(float, folded_bar_center))
    )
    grip_bearing.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    grip_bearing.CreateAxisAttr(UsdGeom.Tokens.y)
    grip_bearing.CreateCollisionEnabledAttr(False)

    hinge = UsdPhysics.RevoluteJoint.Define(stage, "/World/PizzaBoardBailHinge")
    hinge.CreateBody0Rel().SetTargets([Sdf.Path("/World/ServingDish")])
    hinge.CreateBody1Rel().SetTargets([Sdf.Path(bail_path)])
    hinge.CreateLocalPos0Attr().Set(
        Gf.Vec3f(
            BOARD_BAIL_X,
            0.0,
            0.5 * BOARD_THICKNESS + BOARD_BAIL_ROD_RADIUS,
        )
    )
    hinge.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    hinge.CreateAxisAttr(UsdGeom.Tokens.y)
    # Permit either PhysX coordinate sign; connected-body collision and the
    # physical stop pads block the downward branch.  This avoids depending on
    # USD joint sign convention while still allowing the upward 80-degree arc.
    hinge.CreateLowerLimitAttr(-(90.0 - BOARD_BAIL_MIN_ANGLE_DEG))
    hinge.CreateUpperLimitAttr(90.0 - BOARD_BAIL_MIN_ANGLE_DEG)
    # Connected-body collision is required for the bail to bear on the two
    # physical rest stops authored on the board.
    hinge.CreateCollisionEnabledAttr(True)
    print(
        f"[serving] authored wooden pizza board radius={BOARD_RADIUS:.3f}m "
        f"hinged_bail_span={2.0 * BOARD_BAIL_HALF_SPAN:.3f}m "
        f"bail_height={BOARD_BAIL_HEIGHT:.3f}m "
        f"free_grip_block={np.round(BOARD_BAIL_GRIP_SIZE, 3)}m "
        f"mesh_collision=convexDecomposition contact_offset=1mm",
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


class TrayPizzaPickPlace:
    """Top-down grasp of a folding bail followed by a vertical board lift."""

    def __init__(self, stage):
        self._stage = stage
        self._arm_base = find_serving_robot_prim(stage, "base_link")
        self._end_effector = find_serving_robot_prim(stage, "link_6")
        _, self._arm_orientation_world, self._arm_to_world = prim_world_pose(
            self._arm_base
        )
        # Retained for scene-coordinate helpers; grasp motion itself is now
        # top-down and the RG2 physical forward axis points toward world -Z.
        self._approach_world = np.array([-1.0, 0.0, 0.0])
        # The board handle itself remains pointed toward world +Y.  Handle
        # placement and gripper approach are intentionally independent.
        self._handle_world = np.array([0.0, 1.0, 0.0])
        self._up_world = np.array([0.0, 0.0, 1.0])
        self._base_forward_world = self.local_vector_to_world(
            np.array([1.0, 0.0, 0.0])
        )
        self._bail_fold_world = self.local_vector_to_world(
            np.array([BOARD_BAIL_FOLD_ARM_X_SIGN, 0.0, 0.0])
        )
        self._controller = None
        self._gripper = None
        self._gripper_joint_index = None
        self._phase = 0
        self._phase_steps = 0
        self._settled_steps = 0
        self._targets = []
        self._initial_wrist = None
        self._initial_orientation = None
        self._delivery_orientation = None
        self._phase5_start_angle_deg = 50.0
        self._lift_j1_hold = None
        self._lift_tcp_start = None
        self._lift_tcp_target = None
        self._lift_orientation = None
        self._tray_joint_indices = None
        self._tray_deploy_steps = 0
        self._trays_deployed = False
        self._magnet_force = 0.0
        self._magnet_gap = 0.0
        self.done = False
        self.failed = False

        dish_material = PhysicsMaterial(
            prim_path="/World/Physics_Materials/serving_dish_material",
            static_friction=GRIP_CONTACT_STATIC_FRICTION,
            dynamic_friction=GRIP_CONTACT_DYNAMIC_FRICTION,
            restitution=0.0,
        )
        # Invisible collision proxy; visible geometry comes exclusively from
        # the Antigravity assets below.
        self._dish = DynamicCylinder(
            prim_path="/World/ServingDish",
            name="serving_dish",
            position=self.local_to_world(TOP_DECK_DISH_LOCAL),
            orientation=self._arm_orientation_world,
            radius=DISH_RADIUS,
            height=DISH_HEIGHT,
            visible=True,
            mass=BOARD_TEST_MASS,
            physics_material=dish_material,
        )
        dish_body = UsdPhysics.RigidBodyAPI.Get(stage, self._dish.prim_path)
        if not dish_body:
            raise RuntimeError("failed to create /World/ServingDish rigid body")
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
        # Keep the dish from falling during the frame in which the robot's
        # articulation and tensor handles are initialized.
        dish_body.GetKinematicEnabledAttr().Set(True)
        # Do not use visibility=invisible here: USD visibility is inherited
        # and would hide the referenced plate and pizza children as well.
        UsdGeom.Gprim(stage.GetPrimAtPath(self._dish.prim_path)).CreateDisplayOpacityAttr(
            [0.0]
        )
        author_wooden_pizza_board(
            stage,
            "/World/ServingDish/WoodenPizzaBoard",
            dish_material,
        )
        author_magnetic_steel_plate(stage)
        self._bail = stage.GetPrimAtPath("/World/PizzaBoardBail")
        if not self._bail.IsValid():
            raise RuntimeError("/World/PizzaBoardBail was not authored")
        self._bail_crossbar = stage.GetPrimAtPath(
            "/World/PizzaBoardBail/GeometryFrame/TopCrossbar"
        )
        if not self._bail_crossbar.IsValid():
            raise RuntimeError("pizza-board bail crossbar was not authored")
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

    def refresh_robot_frame(self):
        """Refresh cached robot-local axes after the mobile base has moved."""
        _, self._arm_orientation_world, self._arm_to_world = prim_world_pose(
            self._arm_base
        )
        self._base_forward_world = self.local_vector_to_world(
            np.array([1.0, 0.0, 0.0])
        )
        self._bail_fold_world = self.local_vector_to_world(
            np.array([BOARD_BAIL_FOLD_ARM_X_SIGN, 0.0, 0.0])
        )

    def _tcp_to_wrist(self, tcp_position):
        # With the downward orientation, the finger TCP is below link_6.
        # Keep the wrist above the requested contact point by the tool length.
        return tcp_position + self._up_world * RG2_TCP_LENGTH

    def _bail_angle_deg(self):
        """Return physical bail elevation: 10 degrees folded, 90 upright."""
        _, _, board_transform = prim_world_pose(
            self._stage.GetPrimAtPath(self._dish.prim_path)
        )
        crossbar_world, _, _ = prim_world_pose(self._bail_crossbar)
        crossbar_board = board_transform.GetInverse().Transform(
            Gf.Vec3d(*map(float, crossbar_world))
        )
        hinge_board = np.array(
            [
                BOARD_BAIL_X,
                0.0,
                0.5 * BOARD_THICKNESS + BOARD_BAIL_ROD_RADIUS,
            ]
        )
        arm_vector = np.asarray(crossbar_board, dtype=float) - hinge_board
        folded_axis_distance = BOARD_BAIL_FOLD_X_SIGN * arm_vector[0]
        return float(np.degrees(np.arctan2(arm_vector[2], folded_axis_distance)))

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

    def _apply_magnetic_retention(self):
        """Apply a weak short-range downward force like a permanent magnet."""
        if ANCHOR_BOARD_FOR_BAIL_TEST:
            self._magnet_force = 0.0
            self._magnet_gap = 0.0
            return
        board_position = np.asarray(self._dish.get_world_pose()[0], dtype=float)
        displacement = board_position - self._pick_start_position
        gap = max(0.0, float(np.dot(displacement, self._up_world)))
        lateral = displacement - self._up_world * float(
            np.dot(displacement, self._up_world)
        )
        lateral_distance = float(np.linalg.norm(lateral))
        force_amount = 0.0
        if (
            gap < MAGNET_CAPTURE_RANGE
            and lateral_distance < MAGNET_LATERAL_CAPTURE_RADIUS
        ):
            normalized_gap = np.clip(gap / MAGNET_CAPTURE_RANGE, 0.0, 1.0)
            # Smoothstep decay has no force discontinuity at the edge of the
            # 3 mm magnetic capture range.
            decay = 1.0 - normalized_gap * normalized_gap * (
                3.0 - 2.0 * normalized_gap
            )
            force_amount = MAGNET_MAX_FORCE * decay
            force = -self._up_world * force_amount
            self._dish._rigid_prim_view.apply_forces(
                np.asarray([force], dtype=np.float32)
            )
        self._magnet_force = force_amount
        self._magnet_gap = gap

    def _enter_phase(self, phase, articulation):
        self._phase = phase
        self._phase_steps = 0
        self._settled_steps = 0
        names = [
            "move to high safe pose above folded bail",
            "descend to pre-grasp above folded bail",
            "descend vertically around free wooden grip block",
            "close gripper",
            "raise folding bail through 50 degrees",
            "raise folding bail upright",
            "lift pizza board vertically",
            "rotate joint_1 by 180 degrees",
            "move above destination table",
            "lower pizza board onto table",
            "open gripper",
            "retreat vertically from table",
            "return arm to initial home pose",
            "pizza delivery complete",
        ]
        print(f"[serving-bail] phase={phase} {names[phase]}", flush=True)
        if phase == 3:
            self._command_gripper(articulation, GRIPPER_CLOSE[0])
        elif phase == 6:
            current = articulation.get_joint_positions()
            self._lift_j1_hold = float(current[self._arm_joint_indices[0]])
            wrist_position, self._lift_orientation, _ = prim_world_pose(
                self._end_effector
            )
            self._lift_tcp_start = (
                wrist_position - self._up_world * RG2_TCP_LENGTH
            )
            self._lift_tcp_target = self._lift_tcp_start.copy()
            self._lift_tcp_target[2] += TOP_DECK_SAFE_CLEARANCE
            print(
                "[serving-bail] vertical-lift J1 branch locked at "
                f"{np.degrees(self._lift_j1_hold):.1f}deg; "
                f"fixed_xy={np.round(self._lift_tcp_start[:2], 4)} "
                f"z={self._lift_tcp_start[2]:.4f}"
                f"->{self._lift_tcp_target[2]:.4f}m",
                flush=True,
            )
            if ANCHOR_BOARD_FOR_BAIL_TEST:
                bail_angle = self._bail_angle_deg()
                self.done = True
                print(
                    "[serving-bail] anchored-board hinge test complete; "
                    f"bail_angle={bail_angle:.1f}deg, holding pose",
                    flush=True,
                )
        elif phase == 7:
            self._lift_j1_hold = None
            board_position = np.asarray(self._dish.get_world_pose()[0])
            board_lift = float(
                np.dot(board_position - self._pick_start_position, self._up_world)
            )
            if board_lift >= 0.08:
                current = articulation.get_joint_positions()
                self._j1_turn_start = float(
                    current[self._arm_joint_indices[0]]
                )
                self._j1_turn_target = (
                    self._j1_turn_start + J1_DELIVERY_TURN
                )
                print(
                    "[serving-bail] lift verified; "
                    f"board_lift={board_lift:.4f}m, starting J1 "
                    f"half-turn direction={np.degrees(J1_DELIVERY_TURN):+.0f}deg",
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
        elif phase == 8:
            # J1 has just turned the complete arm by 180 degrees.  Preserve
            # the resulting downward wrist yaw instead of commanding the
            # pre-turn quaternion and needlessly spinning the gripper back.
            wrist_position, self._delivery_orientation, _ = prim_world_pose(
                self._end_effector
            )
            actual_tcp = wrist_position - self._up_world * RG2_TCP_LENGTH
            # Translate only along base +X toward the table. Keep the measured
            # lateral/Z coordinates; phase 9 performs the vertical descent.
            requested_above = self._targets[8]
            forward_delta = float(
                np.dot(requested_above - actual_tcp, self._base_forward_world)
            )
            delivery_above = (
                actual_tcp + self._base_forward_world * forward_delta
            )
            self._targets[8] = delivery_above
            self._targets[11] = delivery_above.copy()
            print(
                "[serving-bail] delivery wrist orientation locked after J1 turn; "
                "translating only base +X to "
                f"{np.round(delivery_above, 4)}",
                flush=True,
            )
        elif phase == 10:
            self._command_gripper(articulation, GRIPPER_OPEN[0])
        elif phase == 12:
            current = articulation.get_joint_positions()
            self._stow_start_joints = np.asarray(
                current[self._arm_joint_indices], dtype=float
            ).copy()
            print(
                "[serving-bail] pizza placed on table; folding arm joints back to compact ready pose [90, 0, -90, 0, -60, 90]deg",
                flush=True,
            )
        elif phase == 13:
            self.done = True
            print(
                "[serving-bail] pizza delivery complete; arm stowed in home pose",
                flush=True,
            )

    def initialize(self, articulation, dof_names):
        # The dish is authored while the robot is in the kitchen, whereas the
        # manipulation task starts after navigation.  Re-read the base frame
        # so every tray, grasp, and destination coordinate follows the docked
        # robot pose instead of the original spawn pose.
        self.refresh_robot_frame()
        required = (*GRIPPER_JOINTS[:1], *SLIDING_TRAY_JOINTS)
        missing = [name for name in required if name not in dof_names]
        if missing:
            raise RuntimeError(
                f"missing serving drive DOF {missing}; run once with "
                "MOBILE_DEMO_REIMPORT=1"
            )
        self._dish.initialize()
        spawn_position = self.local_to_world(TOP_DECK_DISH_LOCAL)
        self._dish.set_world_pose(position=spawn_position)
        dish_body = UsdPhysics.RigidBodyAPI.Get(self._stage, self._dish.prim_path)
        if ANCHOR_BOARD_FOR_BAIL_TEST:
            dish_body.GetKinematicEnabledAttr().Set(True)
            print(
                "[serving-bail] board anchored to top deck for hinge-only test",
                flush=True,
            )
        else:
            dish_body.GetKinematicEnabledAttr().Set(False)
        # A newly initialized dish already has zero velocity.  Do not write
        # velocity in this frame: PhysX applies the kinematic-state change on
        # the next update and rejects velocity writes until then.

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
        self._arm_joint_indices = np.asarray(
            [dof_names.index(name) for name in ARM_JOINTS], dtype=np.int32
        )
        self._tray_joint_indices = np.asarray(
            [dof_names.index(name) for name in SLIDING_TRAY_JOINTS],
            dtype=np.int32,
        )
        self._gripper.set_joint_positions(GRIPPER_OPEN)

        arm_position, arm_orientation, _ = prim_world_pose(self._arm_base)
        self._controller = RMPFlowController(
            name="tray_pizza_horizontal_rmpflow",
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
        self._lift_ik = LulaKinematicsSolver(
            str(M0609_DESCRIPTION), str(M0609_RMPFLOW_URDF)
        )
        self._lift_ik.set_robot_base_pose(
            robot_position=arm_position,
            robot_orientation=arm_orientation,
        )
        self._initial_wrist, self._initial_orientation, _ = prim_world_pose(
            self._end_effector
        )

        pick_center = self._dish.get_world_pose()[0]
        self._pick_start_position = np.asarray(pick_center, dtype=float).copy()
        world_x = self._bail_fold_world
        hinge = (
            pick_center
            + world_x * BOARD_BAIL_X
            + self._up_world
            * (0.5 * BOARD_THICKNESS + BOARD_BAIL_ROD_RADIUS)
        )
        self._pick_hinge = hinge.copy()
        self._arc_world_x = world_x.copy()
        min_angle = np.deg2rad(BOARD_BAIL_MIN_ANGLE_DEG)
        folded_bar_center = hinge + BOARD_BAIL_HEIGHT * (
            world_x * np.cos(min_angle)
            + self._up_world * np.sin(min_angle)
        )
        folded_grasp = (
            folded_bar_center
            - self._up_world * BOARD_BAIL_GRASP_INSERT_DEPTH
        )
        above_folded = folded_grasp + self._up_world * TOP_DECK_APPROACH_DISTANCE
        safe_above_folded = folded_grasp + self._up_world * 0.30
        middle_angle = np.deg2rad(
            0.5 * (BOARD_BAIL_MIN_ANGLE_DEG + 90.0)
        )
        bail_middle = (
            hinge
            + BOARD_BAIL_HEIGHT
            * (
                world_x * np.cos(middle_angle)
                + self._up_world * np.sin(middle_angle)
            )
            - self._up_world * BOARD_BAIL_GRASP_INSERT_DEPTH
        )
        bail_upright = (
            hinge
            + self._up_world * BOARD_BAIL_HEIGHT
            - self._up_world * BOARD_BAIL_GRASP_INSERT_DEPTH
        )
        board_lifted = bail_upright + self._up_world * TOP_DECK_SAFE_CLEARANCE
        place_center = self.local_to_world(TABLE_DISH_LOCAL)
        place_hinge = place_center + self._up_world * (
            0.5 * BOARD_THICKNESS + BOARD_BAIL_ROD_RADIUS
        )
        place_grasp = (
            place_hinge
            + self._up_world * BOARD_BAIL_HEIGHT
            - self._up_world * BOARD_BAIL_GRASP_INSERT_DEPTH
        )
        # First translate over the destination at the already-safe lift
        # height.  The following phase changes only world Z.
        place_above = place_grasp.copy()
        place_above[2] = board_lifted[2]

        # First descend vertically onto the folded crossbar.  Once closed, two
        # arc waypoints raise the hinged bail without forcing its end through
        # an impossible straight-line path.  The final waypoint lifts the
        # entire board vertically with the bail upright.
        self._targets = [
            safe_above_folded,
            above_folded,
            folded_grasp,
            None,
            bail_middle,
            bail_upright,
            board_lifted,
            None,
            place_above,
            place_grasp,
            None,
            place_above,
        ]
        print(
            "[serving-bail] ready "
            f"board={np.round(pick_center, 4)} "
            f"safe_above={np.round(safe_above_folded, 4)} "
            f"above={np.round(above_folded, 4)} "
            f"folded_grasp={np.round(folded_grasp, 4)} "
            f"arc50={np.round(bail_middle, 4)} "
            f"upright={np.round(bail_upright, 4)} "
            f"board_lift={np.round(board_lifted, 4)} "
            f"table_above={np.round(place_above, 4)} "
            f"table_place={np.round(place_grasp, 4)}",
            flush=True,
        )
        print(
            "[sliding-tray] destination reached; deploying both trays "
            f"to {SLIDING_TRAY_EXTENSION:.2f}m before pick-and-place",
            flush=True,
        )

    def step(self, articulation):
        if self.done or self.failed:
            return
        self._apply_magnetic_retention()
        if not self._trays_deployed:
            self._tray_deploy_steps += 1
            raw_amount = min(
                1.0,
                self._tray_deploy_steps / SLIDING_TRAY_DEPLOY_STEPS,
            )
            amount = raw_amount * raw_amount * (3.0 - 2.0 * raw_amount)
            target = np.full(
                len(SLIDING_TRAY_JOINTS),
                SLIDING_TRAY_EXTENSION * amount,
                dtype=float,
            )
            articulation.apply_action(
                ArticulationAction(
                    joint_positions=target,
                    joint_indices=self._tray_joint_indices,
                )
            )
            actual = articulation.get_joint_positions()[self._tray_joint_indices]
            error = float(np.max(np.abs(target - actual)))
            if self._tray_deploy_steps % 60 == 0:
                print(
                    "[sliding-tray] "
                    f"target={target[0]:.3f}m "
                    f"actual={np.round(actual, 3).tolist()}m",
                    flush=True,
                )
            if raw_amount >= 1.0 and error < 0.005:
                self._trays_deployed = True
                print(
                    "[sliding-tray] deployment complete; "
                    "pick-and-place enabled",
                    flush=True,
                )
                self._enter_phase(0, articulation)
            elif self._tray_deploy_steps >= SLIDING_TRAY_DEPLOY_STEPS + 360:
                self.failed = True
                print(
                    "[sliding-tray] STOPPED: deployment did not converge; "
                    f"joint error={error:.4f}m",
                    flush=True,
                )
            return
        self._phase_steps += 1
        if self._phase in (3, 10):
            wait_steps = 120 if self._phase == 3 else 90
            target = (
                GRIPPER_CLOSE[0]
                if self._phase == 3
                else GRIPPER_OPEN[0]
            )
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
                self._enter_phase(self._phase + 1, articulation)
            return

        if self._phase == 7:
            raw_amount = min(1.0, self._phase_steps / J1_HALF_TURN_STEPS)
            amount = raw_amount * raw_amount * (3.0 - 2.0 * raw_amount)
            target = self._j1_turn_start + amount * (
                self._j1_turn_target - self._j1_turn_start
            )
            articulation.apply_action(
                ArticulationAction(
                    joint_positions=np.asarray([target], dtype=float),
                    joint_indices=np.asarray(
                        [self._arm_joint_indices[0]], dtype=np.int32
                    ),
                )
            )
            actual = float(
                articulation.get_joint_positions()[self._arm_joint_indices[0]]
            )
            error = abs(target - actual)
            if self._phase_steps % 60 == 0:
                print(
                    "[serving-j1-turn] "
                    f"command={np.degrees(target):.1f}deg "
                    f"actual={np.degrees(actual):.1f}deg "
                    f"error={np.degrees(error):.2f}deg",
                    flush=True,
                )
            self._settled_steps = (
                self._settled_steps + 1
                if raw_amount >= 1.0 and error < 0.03
                else 0
            )
            if self._settled_steps >= 15:
                self._enter_phase(8, articulation)
            elif self._phase_steps >= J1_HALF_TURN_STEPS + 300:
                self.failed = True
                print(
                    "[serving-bail] STOPPED: J1 half-turn did not converge; "
                    f"joint error={np.degrees(error):.2f}deg",
                    flush=True,
                )
            return

        if self.done or self.failed or self._phase >= 13:
            return

        tcp_target = (
            self._targets[self._phase]
            if self._phase < len(self._targets)
            else self._initial_wrist
        )
        transition_complete = True
        if self._phase in (4, 5):
            start_angle, end_angle = (
                (BOARD_BAIL_MIN_ANGLE_DEG, 50.0)
                if self._phase == 4
                else (self._phase5_start_angle_deg, 90.0)
            )
            raw_amount = min(1.0, self._phase_steps / BAIL_ARC_STEPS)
            amount = raw_amount * raw_amount * (3.0 - 2.0 * raw_amount)
            commanded_angle = np.deg2rad(
                start_angle + amount * (end_angle - start_angle)
            )
            tcp_target = (
                self._pick_hinge
                + BOARD_BAIL_HEIGHT
                * (
                    self._arc_world_x * np.cos(commanded_angle)
                    + self._up_world * np.sin(commanded_angle)
                )
                - self._up_world * BOARD_BAIL_GRASP_INSERT_DEPTH
            )
            transition_complete = raw_amount >= 1.0
            if self._phase_steps % 60 == 0:
                print(
                    f"[serving-bail-arc] phase={self._phase} "
                    f"command={np.degrees(commanded_angle):.1f}deg",
                    flush=True,
                )
        elif self._phase == 6:
            raw_amount = min(1.0, self._phase_steps / VERTICAL_LIFT_STEPS)
            amount = raw_amount * raw_amount * (3.0 - 2.0 * raw_amount)
            tcp_target = self._lift_tcp_start.copy()
            tcp_target[2] = (
                self._lift_tcp_start[2]
                + amount
                * (self._lift_tcp_target[2] - self._lift_tcp_start[2])
            )
            transition_complete = raw_amount >= 1.0
            wrist_target = self._tcp_to_wrist(tcp_target)
            target_orientation = (
                self._lift_orientation
                if self._lift_orientation is not None
                else VERTICAL_EE_ORIENTATION
            )
            current_arm = np.asarray(
                articulation.get_joint_positions()[self._arm_joint_indices],
                dtype=float,
            )
            lift_joints, ik_ok = self._lift_ik.compute_inverse_kinematics(
                "link_6",
                wrist_target,
                target_orientation,
                current_arm,
                0.003,
                0.03,
            )
            if ik_ok:
                lift_joints = current_arm + (
                    np.asarray(lift_joints, dtype=float)
                    - current_arm
                    + np.pi
                ) % (2.0 * np.pi) - np.pi
                if self._lift_j1_hold is not None:
                    lift_joints[0] = self._lift_j1_hold
                articulation.apply_action(
                    ArticulationAction(
                        joint_positions=np.asarray(lift_joints, dtype=float),
                        joint_indices=self._arm_joint_indices,
                    )
                )
            if self._phase_steps % 60 == 0:
                print(
                    "[serving-lift-direct-ik] vertical-only "
                    f"z={tcp_target[2]:.4f}m "
                    f"target_z={self._lift_tcp_target[2]:.4f}m "
                    f"ik_ok={ik_ok}",
                    flush=True,
                )
            if transition_complete:
                self._enter_phase(7, articulation)
            return
        elif self._phase == 9:
            raw_amount = min(
                1.0, self._phase_steps / PLACEMENT_DESCENT_STEPS
            )
            amount = raw_amount * raw_amount * (3.0 - 2.0 * raw_amount)
            place_above = self._targets[8]
            place_grasp = self._targets[9]
            tcp_target = place_above.copy()
            tcp_target[2] = (
                place_above[2]
                + amount * (place_grasp[2] - place_above[2])
            )
            transition_complete = raw_amount >= 1.0
            if self._phase_steps % 60 == 0:
                print(
                    "[serving-place-descent] "
                    f"z={tcp_target[2]:.4f}m "
                    f"target_z={place_grasp[2]:.4f}m",
                    flush=True,
                )
        wrist_target = self._tcp_to_wrist(tcp_target)
        target_orientation = (
            self._lift_orientation
            if self._phase == 6 and self._lift_orientation is not None
            else (
                self._delivery_orientation
                if 8 <= self._phase <= 11
                and self._delivery_orientation is not None
                else VERTICAL_EE_ORIENTATION
            )
        )
        if self._phase == 12:
            raw_amount = min(1.0, self._phase_steps / INITIAL_TRANSITION_STEPS)
            amount = raw_amount * raw_amount * (3.0 - 2.0 * raw_amount)
            stow_target_joints = np.deg2rad([90.0, 0.0, -90.0, 0.0, -60.0, 90.0])
            target_joints = self._stow_start_joints + amount * (
                stow_target_joints - self._stow_start_joints
            )
            articulation.apply_action(
                ArticulationAction(
                    joint_positions=target_joints,
                    joint_indices=self._arm_joint_indices,
                )
            )
            transition_complete = raw_amount >= 1.0
            if transition_complete:
                self._settled_steps += 1
                if self._settled_steps >= 15:
                    self._enter_phase(13, articulation)
            else:
                self._settled_steps = 0
            return
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
                VERTICAL_EE_ORIENTATION,
                amount,
            )
            transition_complete = raw_amount >= 1.0
        action = self._controller.forward(
            target_end_effector_position=wrist_target,
            target_end_effector_orientation=target_orientation,
        )
        if self._phase == 6 and self._lift_j1_hold is not None:
            # RMPFlow's default posture has J1=0 while this pickup branch is
            # near +pi. Directly override J1 target in the action array before
            # sending a single unified action command to the articulation.
            if action.joint_positions is not None:
                action.joint_positions[0] = self._lift_j1_hold
            if self._phase_steps % 60 == 0:
                actual_j1 = float(
                    articulation.get_joint_positions()[
                        self._arm_joint_indices[0]
                    ]
                )
                print(
                    "[serving-lift] J1 branch hold "
                    f"target={np.degrees(self._lift_j1_hold):.1f}deg "
                    f"actual={np.degrees(actual_j1):.1f}deg",
                    flush=True,
                )
        articulation.apply_action(action)

        ee_position, _, _ = prim_world_pose(self._end_effector)
        error = float(np.linalg.norm(wrist_target - ee_position))
        bail_angle = self._bail_angle_deg()
        if (
            self._phase == 4
            and bail_angle >= BAIL_PHASE4_OVERSHOOT_DEG
        ):
            if bail_angle >= BAIL_UPRIGHT_ACCEPT_DEG:
                print(
                    "[serving-bail] phase=4 overshoot accepted; "
                    f"bail already upright at {bail_angle:.1f}deg, "
                    "skipping phase=5",
                    flush=True,
                )
                self._enter_phase(6, articulation)
            else:
                self._phase5_start_angle_deg = bail_angle
                print(
                    "[serving-bail] phase=4 overshoot accepted; "
                    f"continuing phase=5 from {bail_angle:.1f}deg",
                    flush=True,
                )
                self._enter_phase(5, articulation)
            return
        required_bail_angle = {4: 45.0, 5: 85.0}.get(self._phase)
        bail_angle_reached = (
            required_bail_angle is None
            or bail_angle >= required_bail_angle
        )
        board_position = np.asarray(self._dish.get_world_pose()[0])
        premature_board_lift = float(
            np.dot(
                board_position - self._pick_start_position,
                self._up_world,
            )
        )
        if (
            self._phase in (4, 5)
            and not bail_angle_reached
            and premature_board_lift > 0.03
        ):
            self.failed = True
            print(
                "[serving-bail] STOPPED: board moved before bail reached "
                f"the required angle; board_lift={premature_board_lift:.4f}m "
                f"bail_angle={bail_angle:.1f}deg",
                flush=True,
            )
            return
        if self._phase in (4, 5) and self._phase_steps % 30 == 0:
            print(
                f"[serving-bail-angle] phase={self._phase} "
                f"actual={bail_angle:.1f}deg "
                f"required={required_bail_angle:.1f}deg "
                f"tcp_error={error:.4f}m "
                f"magnet={self._magnet_force:.1f}N "
                f"gap={self._magnet_gap * 1000.0:.2f}mm",
                flush=True,
            )
        if self._phase == 0:
            position_tolerance = 0.03
        elif self._phase == 9:
            position_tolerance = 0.04
        else:
            position_tolerance = 0.025

        if self._phase == 11:
            position_delta = wrist_target - ee_position
            xy_error = float(np.linalg.norm(position_delta[:2]))
            z_error = abs(float(position_delta[2]))
            reached = (
                transition_complete
                and z_error < 0.02
                and xy_error < 0.07
            )
        else:
            reached = (
                transition_complete
                and error < position_tolerance
                and bail_angle_reached
            )

        self._settled_steps = (
            self._settled_steps + 1
            if reached
            else 0
        )
        if self._settled_steps >= 15:
            self._enter_phase(self._phase + 1, articulation)
        elif self._phase_steps >= 600:
            message = (
                f"folding-bail phase {self._phase} did not converge; "
                f"position error={error:.4f} m, "
                f"bail angle={bail_angle:.1f} deg"
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
        # Release tensor-backed wrappers before a later trip removes and
        # recreates /World/ServingDish.  Keeping DynamicCylinder/RMPFlow alive
        # while deleting the prim invalidates the global PhysX simulation view.
        self._controller = None
        self._gripper = None
        self._dish = None
        self._bail = None
        self._bail_crossbar = None
