"""Soda-can assets and tray loading for the serving robot demo.

The source GLB is visual-only and Y-up.  This module authors its colored
triangle primitives as a correctly sized Z-up visual below a simple dynamic
cylinder, which provides stable collision geometry for RG2 pick-and-place.
"""

import json
import os
import struct
from pathlib import Path

from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade

from isaac_scene_utils import find_serving_robot_prim, prim_world_pose


WORKSPACE = Path(
    os.environ.get("COBOT3_WS", Path(__file__).resolve().parents[1])
).resolve()
SODA_CAN_GLB = WORKSPACE / "assets/source_food/soda_can_jeremy.glb"

CAN_RADIUS = 0.033
CAN_HEIGHT = 0.122
CAN_MASS = 0.35
SOURCE_RADIUS = 3.0567619800567627
SOURCE_HEIGHT = 10.058300018310547
SOURCE_MID_Y = SOURCE_HEIGHT * 0.5

# Positions are relative to the centre of upper_tray_right_link.  +X is the
# robot front and -Y is the outer/right side.  The requested 200 mm X and
# 100 mm Y centre spacing leaves 134 mm and 34 mm respectively between the
# 66 mm can outlines while keeping all four cans inside the tray rims.
RIGHT_FRONT_CAN_POSITIONS = (
    (0.08, -0.050),
    (0.08, 0.050),
    (0.28, -0.050),
    (0.28, 0.050),
)


def _read_glb(glb_path):
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
    return document, data[offset + 8 : offset + 8 + binary_length]


def _read_accessor(document, binary, index):
    accessor = document["accessors"][index]
    view = document["bufferViews"][accessor["bufferView"]]
    component_formats = {
        5121: ("B", 1),
        5123: ("H", 2),
        5125: ("I", 4),
        5126: ("f", 4),
    }
    component_counts = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}
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


def _author_can_materials(stage, document):
    authored = []
    for index, source in enumerate(document.get("materials", [])):
        pbr = source.get("pbrMetallicRoughness", {})
        color = pbr.get("baseColorFactor", [0.8, 0.8, 0.8, 1.0])
        path = f"/World/Looks/SodaCanMaterial_{index}"
        material = UsdShade.Material.Define(stage, path)
        shader = UsdShade.Shader.Define(stage, f"{path}/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Float3).Set(
            Gf.Vec3f(*color[:3])
        )
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.35)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(
            0.65 if index == 0 else 0.15
        )
        material.CreateSurfaceOutput().ConnectToSource(
            shader.ConnectableAPI(), "surface"
        )
        authored.append((material, color))
    return authored


def _author_can_visual(stage, root_path, document, binary, materials):
    radial_scale = CAN_RADIUS / SOURCE_RADIUS
    vertical_scale = CAN_HEIGHT / SOURCE_HEIGHT
    node = document["nodes"][document["scenes"][0]["nodes"][0]]
    primitives = document["meshes"][node["mesh"]]["primitives"]
    for index, primitive in enumerate(primitives):
        points = _read_accessor(
            document, binary, primitive["attributes"]["POSITION"]
        )
        normals = _read_accessor(
            document, binary, primitive["attributes"]["NORMAL"]
        )
        indices = [
            value[0]
            for value in _read_accessor(document, binary, primitive["indices"])
        ]
        mesh = UsdGeom.Mesh.Define(stage, f"{root_path}/Visual/Primitive_{index}")
        # glTF is Y-up.  Map its X/Y/Z into USD X/Z/-Y while applying exact
        # physical dimensions and centring the original bottom-origin mesh.
        mesh.CreatePointsAttr(
            [
                Gf.Vec3f(
                    x * radial_scale,
                    -z * radial_scale,
                    (y - SOURCE_MID_Y) * vertical_scale,
                )
                for x, y, z in points
            ]
        )
        mesh.CreateFaceVertexCountsAttr([3] * (len(indices) // 3))
        mesh.CreateFaceVertexIndicesAttr(indices)
        mesh.CreateNormalsAttr([Gf.Vec3f(nx, -nz, ny) for nx, ny, nz in normals])
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        mesh.CreateDoubleSidedAttr(True)
        material_index = primitive.get("material", 0)
        material, color = materials[material_index]
        mesh.CreateDisplayColorAttr([Gf.Vec3f(*color[:3])])
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(
            material,
            bindingStrength=UsdShade.Tokens.strongerThanDescendants,
        )


def _author_dynamic_can(
    stage, index, world_position, world_orientation, document, binary,
    visual_materials, physics_material
):
    root_path = f"/World/ServingDrinks/SodaCan_{index:02d}"
    root = UsdGeom.Xform.Define(stage, root_path)
    root.AddTranslateOp().Set(Gf.Vec3d(*map(float, world_position)))
    root.AddOrientOp().Set(
        Gf.Quatf(
            float(world_orientation[0]),
            Gf.Vec3f(*map(float, world_orientation[1:])),
        )
    )
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(CAN_MASS)
    rigid = PhysxSchema.PhysxRigidBodyAPI.Apply(root.GetPrim())
    rigid.CreateLinearDampingAttr(0.15)
    rigid.CreateAngularDampingAttr(0.25)
    rigid.CreateMaxDepenetrationVelocityAttr(0.5)

    collider = UsdGeom.Cylinder.Define(stage, f"{root_path}/Collision")
    collider.CreateAxisAttr(UsdGeom.Tokens.z)
    collider.CreateRadiusAttr(CAN_RADIUS)
    collider.CreateHeightAttr(CAN_HEIGHT)
    collider.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    UsdPhysics.CollisionAPI.Apply(collider.GetPrim()).CreateCollisionEnabledAttr(
        True
    )
    UsdShade.MaterialBindingAPI.Apply(collider.GetPrim()).Bind(
        physics_material.material,
        bindingStrength=UsdShade.Tokens.strongerThanDescendants,
        materialPurpose="physics",
    )
    physx_collision = PhysxSchema.PhysxCollisionAPI.Apply(collider.GetPrim())
    physx_collision.CreateContactOffsetAttr(0.0015)
    physx_collision.CreateRestOffsetAttr(0.0)
    _author_can_visual(
        stage, root_path, document, binary, visual_materials
    )
    return root_path


def spawn_soda_cans(stage, count=4):
    """Place the requested delivery cans on the right-front upper tray."""
    if os.environ.get("MOBILE_DEMO_SODA_CANS", "1") != "1":
        print("[drinks] soda cans disabled by MOBILE_DEMO_SODA_CANS=0", flush=True)
        return []
    if not SODA_CAN_GLB.is_file():
        raise FileNotFoundError(SODA_CAN_GLB)

    tray = find_serving_robot_prim(stage, "upper_tray_right_link")
    _, tray_orientation, tray_to_world = prim_world_pose(tray)
    document, binary = _read_glb(SODA_CAN_GLB)
    visual_materials = _author_can_materials(stage, document)
    physics_material = PhysicsMaterial(
        prim_path="/World/Physics_Materials/soda_can_material",
        static_friction=0.80,
        dynamic_friction=0.65,
        restitution=0.02,
    )

    # The link origin is at the plate mid-plane.  Raise the can above its
    # 12.5 mm top face with a 1 mm initial separation for stable contact.
    local_z = 0.5 * 0.025 + 0.5 * CAN_HEIGHT + 0.001
    count = int(count)
    if count < 0 or count > 4:
        raise ValueError(f"soda count must be 0..4, got {count}")
    # Delivery tasks address SodaCan_03 first and SodaCan_02 second.
    selected_positions = (
        list(enumerate(RIGHT_FRONT_CAN_POSITIONS))[-count:] if count else []
    )
    paths = []
    for index, (local_x, local_y) in selected_positions:
        world = tray_to_world.Transform(
            Gf.Vec3d(local_x, local_y, local_z)
        )
        paths.append(
            _author_dynamic_can(
                stage,
                index,
                world,
                tray_orientation,
                document,
                binary,
                visual_materials,
                physics_material,
            )
        )
    print(
        f"[drinks] loaded {count} dynamic soda cans on upper right-front tray "
        f"size={2.0 * CAN_RADIUS:.3f}x{CAN_HEIGHT:.3f}m mass={CAN_MASS:.2f}kg",
        flush=True,
    )
    return paths
