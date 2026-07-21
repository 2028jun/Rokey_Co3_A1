"""Physical wooden cutlery box asset for the serving robot tray."""

import json
import math
import os
import struct
from pathlib import Path

from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade

from isaac_scene_utils import find_serving_robot_prim, prim_world_pose


WORKSPACE = Path(
    os.environ.get("COBOT3_WS", Path(__file__).resolve().parents[1])
).resolve()
CUTLERY_BOX_GLB = WORKSPACE / "assets/source_food/cutlery_box.glb"
CUTLERY_WOOD_TEXTURE = WORKSPACE / "assets/source_food/cutlery_box_wood.png"

CUTLERY_BOX_SIZE = (0.060, 0.200, 0.100)
CUTLERY_LID_HEIGHT = 0.004
CUTLERY_BODY_HEIGHT = CUTLERY_BOX_SIZE[2] - CUTLERY_LID_HEIGHT
CUTLERY_BOX_MASS = 0.60
CUTLERY_BOX_PATH = "/World/ServingCutlery/CutleryBox"
# Rear portion of the right tray.  The nearest can outline begins at X=0.047,
# leaving more than 20 cm between it and the box outline at X=-0.160.
CUTLERY_TRAY_LOCAL = (-0.20, 0.0)


def _read_glb(glb_path):
    data = glb_path.read_bytes()
    magic, version, _ = struct.unpack_from("<4sII", data, 0)
    if magic != b"glTF" or version != 2:
        raise RuntimeError(f"unsupported cutlery GLB: {glb_path}")
    json_length, json_type = struct.unpack_from("<II", data, 12)
    if json_type != 0x4E4F534A:
        raise RuntimeError("cutlery GLB JSON chunk missing")
    document = json.loads(data[20 : 20 + json_length])
    offset = (20 + json_length + 3) & ~3
    binary_length, binary_type = struct.unpack_from("<II", data, offset)
    if binary_type != 0x004E4942:
        raise RuntimeError("cutlery GLB binary chunk missing")
    return document, data[offset + 8 : offset + 8 + binary_length]


def _read_accessor(document, binary, index):
    accessor = document["accessors"][index]
    view = document["bufferViews"][accessor["bufferView"]]
    formats = {5121: ("B", 1), 5123: ("H", 2), 5125: ("I", 4), 5126: ("f", 4)}
    widths = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}
    code, component_size = formats[accessor["componentType"]]
    width = widths[accessor["type"]]
    element_size = component_size * width
    stride = view.get("byteStride", element_size)
    start = view.get("byteOffset", 0) + accessor.get("byteOffset", 0)
    fmt = "<" + code * width
    return [
        struct.unpack_from(fmt, binary, start + row * stride)
        for row in range(accessor["count"])
    ]


def _wood_material(stage):
    path = "/World/Looks/CutleryBoxWood"
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    texture = UsdShade.Shader.Define(stage, f"{path}/Texture")
    texture.CreateIdAttr("UsdUVTexture")
    texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath(str(CUTLERY_WOOD_TEXTURE))
    )
    texture.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
    texture.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
    reader = UsdShade.Shader.Define(stage, f"{path}/PrimvarReader")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        reader.ConnectableAPI(), "result"
    )
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        texture.ConnectableAPI(), "rgb"
    )
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.72)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    material.CreateSurfaceOutput().ConnectToSource(
        shader.ConnectableAPI(), "surface"
    )
    return material


def _author_textured_body(stage, document, binary, material):
    primitive = document["meshes"][0]["primitives"][0]
    points = _read_accessor(document, binary, primitive["attributes"]["POSITION"])
    normals = _read_accessor(document, binary, primitive["attributes"]["NORMAL"])
    texcoords = _read_accessor(document, binary, primitive["attributes"]["TEXCOORD_0"])
    indices = [value[0] for value in _read_accessor(document, binary, primitive["indices"])]
    mesh = UsdGeom.Mesh.Define(stage, f"{CUTLERY_BOX_PATH}/VisualBody")
    # The source has a -90-degree X axis correction.  Apply it directly while
    # scaling its unit cube to the exact physical box dimensions.
    mesh.CreatePointsAttr(
        [
            Gf.Vec3f(
                x * CUTLERY_BOX_SIZE[0],
                -z * CUTLERY_BOX_SIZE[1],
                y * CUTLERY_BODY_HEIGHT - 0.5 * CUTLERY_LID_HEIGHT,
            )
            for x, y, z in points
        ]
    )
    mesh.CreateFaceVertexCountsAttr([3] * (len(indices) // 3))
    mesh.CreateFaceVertexIndicesAttr(indices)
    mesh.CreateNormalsAttr([Gf.Vec3f(nx, -nz, ny) for nx, ny, nz in normals])
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    primvars = UsdGeom.PrimvarsAPI(mesh.GetPrim())
    primvars.CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex
    ).Set([Gf.Vec2f(u, v) for u, v in texcoords])
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)


def spawn_cutlery_box(stage):
    """Spawn one closed, graspable cutlery box on the rear right tray."""
    if os.environ.get("MOBILE_DEMO_CUTLERY_BOX", "1") != "1":
        print("[cutlery] disabled by MOBILE_DEMO_CUTLERY_BOX=0", flush=True)
        return None
    if not CUTLERY_BOX_GLB.is_file() or not CUTLERY_WOOD_TEXTURE.is_file():
        raise FileNotFoundError(CUTLERY_BOX_GLB)
    tray = find_serving_robot_prim(stage, "upper_tray_right_link")
    _, tray_orientation, tray_to_world = prim_world_pose(tray)
    local_z = 0.5 * 0.025 + 0.5 * CUTLERY_BOX_SIZE[2] + 0.001
    world = tray_to_world.Transform(
        Gf.Vec3d(CUTLERY_TRAY_LOCAL[0], CUTLERY_TRAY_LOCAL[1], local_z)
    )
    root = UsdGeom.Xform.Define(stage, CUTLERY_BOX_PATH)
    root.AddTranslateOp().Set(world)
    tray_quaternion = Gf.Quatf(
        float(tray_orientation[0]),
        Gf.Vec3f(*map(float, tray_orientation[1:])),
    )
    # Rotate in the tray plane so the 200 mm local-Y edge follows robot/table
    # X while the box remains flat on its 60 x 200 mm base.
    tray_longitudinal_yaw = Gf.Quatf(
        math.cos(0.25 * math.pi),
        Gf.Vec3f(0.0, 0.0, math.sin(0.25 * math.pi)),
    )
    root.AddOrientOp().Set(tray_quaternion * tray_longitudinal_yaw)
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(CUTLERY_BOX_MASS)
    rigid = PhysxSchema.PhysxRigidBodyAPI.Apply(root.GetPrim())
    rigid.CreateLinearDampingAttr(0.18)
    rigid.CreateAngularDampingAttr(0.30)
    rigid.CreateSolverPositionIterationCountAttr(32)
    rigid.CreateSolverVelocityIterationCountAttr(8)

    physics_material = PhysicsMaterial(
        prim_path="/World/Physics_Materials/cutlery_box_material",
        static_friction=0.80,
        dynamic_friction=0.65,
        restitution=0.0,
    )
    collider = UsdGeom.Cube.Define(stage, f"{CUTLERY_BOX_PATH}/Collision")
    collider.CreateSizeAttr(1.0)
    collider.AddScaleOp().Set(Gf.Vec3f(*CUTLERY_BOX_SIZE))
    collider.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    UsdPhysics.CollisionAPI.Apply(collider.GetPrim()).CreateCollisionEnabledAttr(True)
    UsdShade.MaterialBindingAPI.Apply(collider.GetPrim()).Bind(
        physics_material.material,
        bindingStrength=UsdShade.Tokens.strongerThanDescendants,
        materialPurpose="physics",
    )
    physx = PhysxSchema.PhysxCollisionAPI.Apply(collider.GetPrim())
    physx.CreateContactOffsetAttr(0.0015)
    physx.CreateRestOffsetAttr(0.0)

    document, binary = _read_glb(CUTLERY_BOX_GLB)
    wood = _wood_material(stage)
    _author_textured_body(stage, document, binary, wood)
    lid = UsdGeom.Cube.Define(stage, f"{CUTLERY_BOX_PATH}/Lid")
    lid.CreateSizeAttr(1.0)
    lid.AddTranslateOp().Set(
        Gf.Vec3d(0.0, 0.0, 0.5 * CUTLERY_BODY_HEIGHT)
    )
    lid.AddScaleOp().Set(
        Gf.Vec3f(CUTLERY_BOX_SIZE[0], CUTLERY_BOX_SIZE[1], CUTLERY_LID_HEIGHT)
    )
    UsdShade.MaterialBindingAPI.Apply(lid.GetPrim()).Bind(wood)
    seam = UsdGeom.Cube.Define(stage, f"{CUTLERY_BOX_PATH}/LidSeam")
    seam.CreateSizeAttr(1.0)
    seam.AddTranslateOp().Set(
        Gf.Vec3d(0.0, 0.0, 0.5 * CUTLERY_BODY_HEIGHT - 0.001)
    )
    seam.AddScaleOp().Set(
        Gf.Vec3f(
            CUTLERY_BOX_SIZE[0] + 0.001,
            CUTLERY_BOX_SIZE[1] + 0.001,
            0.001,
        )
    )
    seam.CreateDisplayColorAttr([Gf.Vec3f(0.18, 0.08, 0.025)])
    print(
        "[cutlery] spawned closed wooden box "
        f"path={CUTLERY_BOX_PATH} size={CUTLERY_BOX_SIZE}m "
        f"mass={CUTLERY_BOX_MASS:.2f}kg tray_local={CUTLERY_TRAY_LOCAL} "
        "orientation=flat-long-X",
        flush=True,
    )
    return CUTLERY_BOX_PATH
