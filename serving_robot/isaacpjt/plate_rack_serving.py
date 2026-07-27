"""Four-plate rack payload for the serving robot's left-front upper tray."""

import os
from pathlib import Path

import numpy as np
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

from isaac_scene_utils import find_serving_robot_prim, prim_world_pose


WORKSPACE = Path(
    os.environ.get("COBOT3_WS", Path(__file__).resolve().parents[1])
).resolve()

PLATE_RACK_PATH = "/World/ServingPlateRack"
# Shift the rack 25 mm toward the outside of the robot-left tray.  At full
# deployment the old y=0 position put the 200 mm-wide base's inner edge at
# shelf y=0.305 m, overlapping the fixed top deck edge (y=0.310 m) by 5 mm.
# The overlap caught the base during the first vertical lift.  y=+0.025 puts
# the inner edge at y=0.330 m (20 mm deck clearance) while the outer edge stays
# 10 mm inside the tray's outer rim.
PLATE_RACK_TRAY_LOCAL = (0.18, 0.025)
RACK_BASE_SIZE = (0.30, 0.20, 0.018)
PLATE_RACK_ROOT_LOCAL_Z = 0.5 * 0.025 + 0.5 * RACK_BASE_SIZE[2] + 0.001
RACK_MASS = 1.60
# The fixed deck underside is only 162.5 mm above the rack root when the tray
# is closed.  A 150 mm plate plus the 9 mm rack base left just 3.5 mm geometric
# clearance (and previously penetrated by 0.5 mm with the old separator), which
# is not enough once PhysX contact offsets and tray motion are included.  A
# 142 mm service plate leaves 11.5 mm for stable dynamic deployment.
PLATE_RADIUS = 0.071
PLATE_THICKNESS = 0.006
PLATE_X_OFFSETS = (-0.105, -0.070, 0.070, 0.105)
PLATE_LAYOUTS = {
    1: (-0.070,),
    2: (-0.070, 0.070),
    3: (-0.105, -0.070, 0.070),
    4: PLATE_X_OFFSETS,
}
# Match the proven pizza-board grip block at the actual RG2 contact region.
# The previous 50 mm post plus 78 mm top flange already touched the moving RG2
# links at joint angle 0, so the fingers could not generate any closing stroke
# (target=1.06, actual=0.00).  A narrow lower stem supports a 26 x 90 x 40 mm
# block; 26 mm is the tested finger-closing-axis thickness used by pizza.
HANDLE_STEM_SIZE = (0.020, 0.035, 0.080)
HANDLE_GRIP_SIZE = (0.026, 0.090, 0.040)
HANDLE_GRIP_CENTRE_Z = 0.109


def _preview_material(stage, path, color, roughness=0.55, metallic=0.0):
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


def _bind_visual(prim, material):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        material,
        bindingStrength=UsdShade.Tokens.strongerThanDescendants,
    )


def _configure_collider(prim, physics_material):
    UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr().Set(True)
    # Remember which shapes belong to the physical rack.  They are disabled
    # while the kinematic payload follows the extending tray, then restored
    # only after the tray has stopped.  Moving a collision-enabled kinematic
    # body against the robot articulation can inject unbounded contact forces.
    prim.CreateAttribute(
        "plateRack:transportCollider", Sdf.ValueTypeNames.Bool
    ).Set(True)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        physics_material.material,
        bindingStrength=UsdShade.Tokens.strongerThanDescendants,
        materialPurpose="physics",
    )
    physx = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    physx.CreateContactOffsetAttr(0.001)
    physx.CreateRestOffsetAttr(0.0)


def follow_plate_rack_transport(
    stage, *, payload_root="/World", robot_root=None
):
    """Keep the collisionless kinematic rack on the robot-left tray.

    The integrated mission spawns payloads in the kitchen and then drives the
    robot to a table before the arm task starts.  Since the rack deliberately
    has no FixedJoint (to preserve RG2-to-handle collision), its world pose
    must follow the tray throughout that navigation interval.
    """
    payload_root = str(payload_root).rstrip("/") or "/World"
    rack_path = f"{payload_root}/ServingPlateRack"
    rack = stage.GetPrimAtPath(rack_path)
    if not rack.IsValid():
        return False
    tray = find_serving_robot_prim(
        stage, "upper_tray_left_link", robot_root=robot_root
    )
    _, tray_orientation, tray_to_world = prim_world_pose(tray)
    world = np.asarray(
        tray_to_world.Transform(
            Gf.Vec3d(
                float(PLATE_RACK_TRAY_LOCAL[0]),
                float(PLATE_RACK_TRAY_LOCAL[1]),
                float(PLATE_RACK_ROOT_LOCAL_Z),
            )
        ),
        dtype=float,
    )
    tray_orientation = np.asarray(tray_orientation, dtype=float)
    if not (
        np.all(np.isfinite(world))
        and np.all(np.isfinite(tray_orientation))
    ):
        return False
    translate = None
    orient = None
    for op in UsdGeom.Xformable(rack).GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            translate = op
        elif op.GetOpType() == UsdGeom.XformOp.TypeOrient:
            orient = op
    if translate is None or orient is None:
        raise RuntimeError("plate rack transport xform ops are missing")
    translate.Set(Gf.Vec3d(*map(float, world)))
    orient.Set(
        Gf.Quatf(
            float(tray_orientation[0]),
            Gf.Vec3f(*map(float, tray_orientation[1:])),
        )
    )
    return True


def spawn_plate_rack(
    stage, plate_count=4, *, payload_root="/World", robot_root=None
):
    """Spawn a handled rack carrying the requested one to four plates."""
    plate_count = int(plate_count)
    if plate_count not in PLATE_LAYOUTS:
        raise ValueError(f"plate_count must be in 1..4, got {plate_count}")
    payload_root = str(payload_root).rstrip("/") or "/World"
    rack_path = f"{payload_root}/ServingPlateRack"
    tray = find_serving_robot_prim(
        stage, "upper_tray_left_link", robot_root=robot_root
    )
    _, tray_orientation, tray_to_world = prim_world_pose(tray)
    tray_top_z = 0.5 * 0.025
    root_local_z = PLATE_RACK_ROOT_LOCAL_Z
    world = tray_to_world.Transform(
        Gf.Vec3d(
            PLATE_RACK_TRAY_LOCAL[0],
            PLATE_RACK_TRAY_LOCAL[1],
            root_local_z,
        )
    )

    root = UsdGeom.Xform.Define(stage, rack_path)
    root.AddTranslateOp().Set(world)
    root.AddOrientOp().Set(
        Gf.Quatf(
            float(tray_orientation[0]),
            Gf.Vec3f(*map(float, tray_orientation[1:])),
        )
    )
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    # Keep the payload independent from the serving-robot articulation.  A
    # FixedJoint to the tray makes PhysX merge the rack into that articulation;
    # because robot self-collision is disabled, RG2 then passes straight
    # through the rack handle even though the handle owns CollisionAPI.
    # The plate-rack task follows the moving tray kinematically and clears this
    # flag after deployment, preserving external RG2/rack contacts.
    UsdPhysics.RigidBodyAPI(root.GetPrim()).CreateKinematicEnabledAttr().Set(True)
    UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(RACK_MASS)
    rigid = PhysxSchema.PhysxRigidBodyAPI.Apply(root.GetPrim())
    rigid.CreateLinearDampingAttr(0.20)
    rigid.CreateAngularDampingAttr(0.35)
    rigid.CreateSolverPositionIterationCountAttr(48)
    rigid.CreateSolverVelocityIterationCountAttr(12)

    physics_material = PhysicsMaterial(
        prim_path="/World/Physics_Materials/plate_rack_material",
        static_friction=6.0,
        dynamic_friction=5.0,
        restitution=0.0,
    )
    PhysxSchema.PhysxMaterialAPI.Apply(
        physics_material.material.GetPrim()
    ).CreateFrictionCombineModeAttr("max")
    rack_material = _preview_material(
        stage, "/World/Looks/PlateRack", (0.18, 0.20, 0.22), 0.30, 0.75
    )
    plate_material = _preview_material(
        stage, "/World/Looks/ServingPlate", (0.92, 0.93, 0.90), 0.34, 0.0
    )

    base = UsdGeom.Cube.Define(stage, f"{rack_path}/Base")
    base.CreateSizeAttr(1.0)
    base.AddScaleOp().Set(Gf.Vec3f(*RACK_BASE_SIZE))
    _bind_visual(base.GetPrim(), rack_material)
    _configure_collider(base.GetPrim(), physics_material)

    # Handle-free pizza-board disks are reduced to personal-plate size and
    # stored vertically while keeping the centre grip clear. All four stable
    # prim paths are retained across trips; unused plates are non-rendered and
    # non-colliding so changing quantity does not invalidate tensor views.
    # Keep the vertical plates below the fixed top deck.  The previous +4 mm
    # separator put the plate tops at z=0.8255 m while the deck underside is
    # z=0.8250 m: a 0.5 mm penetration that wedged the loaded left tray shut.
    # With no extra separator the top is z=0.8215 m, leaving 3.5 mm clearance.
    plate_z = 0.5 * RACK_BASE_SIZE[2] + PLATE_RADIUS
    selected_offsets = PLATE_LAYOUTS[plate_count]
    for index, default_offset_x in enumerate(PLATE_X_OFFSETS):
        plate = UsdGeom.Cylinder.Define(
            stage, f"{rack_path}/Plate_{index + 1:02d}"
        )
        enabled = index < plate_count
        offset_x = selected_offsets[index] if enabled else default_offset_x
        plate.CreateAxisAttr(UsdGeom.Tokens.x)
        plate.CreateRadiusAttr(PLATE_RADIUS)
        plate.CreateHeightAttr(PLATE_THICKNESS)
        plate.AddTranslateOp().Set(Gf.Vec3d(offset_x, 0.0, plate_z))
        _bind_visual(plate.GetPrim(), plate_material)
        if enabled:
            plate.MakeVisible()
            _configure_collider(plate.GetPrim(), physics_material)
        else:
            plate.MakeInvisible()
            UsdPhysics.CollisionAPI.Apply(
                plate.GetPrim()
            ).CreateCollisionEnabledAttr().Set(False)

    stem = UsdGeom.Cube.Define(stage, f"{rack_path}/HandleStem")
    stem.CreateSizeAttr(1.0)
    stem.AddTranslateOp().Set(
        Gf.Vec3d(
            0.0,
            0.0,
            0.5 * RACK_BASE_SIZE[2] + 0.5 * HANDLE_STEM_SIZE[2],
        )
    )
    stem.AddScaleOp().Set(Gf.Vec3f(*HANDLE_STEM_SIZE))
    _bind_visual(stem.GetPrim(), rack_material)
    _configure_collider(stem.GetPrim(), physics_material)

    handle = UsdGeom.Cube.Define(stage, f"{rack_path}/Handle")
    handle.CreateSizeAttr(1.0)
    handle.AddTranslateOp().Set(
        Gf.Vec3d(0.0, 0.0, HANDLE_GRIP_CENTRE_Z)
    )
    handle.AddScaleOp().Set(Gf.Vec3f(*HANDLE_GRIP_SIZE))
    _bind_visual(handle.GetPrim(), rack_material)
    _configure_collider(handle.GetPrim(), physics_material)

    # Low dividers keep the vertical plates separated while leaving their
    # upper halves visible.  They are part of the same rigid rack body.
    divider_height = 0.035
    for index, offset_x in enumerate((-0.1225, -0.0875, -0.0525, 0.0525, 0.0875, 0.1225)):
        divider = UsdGeom.Cube.Define(
            stage, f"{rack_path}/Divider_{index + 1:02d}"
        )
        divider.CreateSizeAttr(1.0)
        divider.AddTranslateOp().Set(
            Gf.Vec3d(
                offset_x,
                0.0,
                0.5 * RACK_BASE_SIZE[2] + 0.5 * divider_height,
            )
        )
        divider.AddScaleOp().Set(Gf.Vec3f(0.006, 0.17, divider_height))
        _bind_visual(divider.GetPrim(), rack_material)
        _configure_collider(divider.GetPrim(), physics_material)

    # During tray deployment this independent rigid body is repositioned from
    # the tray's measured transform every frame.  It must be non-colliding in
    # that interval; otherwise the kinematic base pushes directly on the tray
    # and can destabilize the complete mobile-manipulator articulation.
    for prim in Usd.PrimRange(root.GetPrim()):
        marker = prim.GetAttribute("plateRack:transportCollider")
        if marker and marker.Get():
            UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(False)

    print(
        f"[plate-rack] spawned {plate_count} vertical plate(s) on left-front upper tray "
        f"path={rack_path} tray_local={PLATE_RACK_TRAY_LOCAL} "
        f"rack={RACK_BASE_SIZE}m plate_diameter={2.0 * PLATE_RADIUS:.3f}m "
        "transport=independent-kinematic",
        flush=True,
    )
    return rack_path
