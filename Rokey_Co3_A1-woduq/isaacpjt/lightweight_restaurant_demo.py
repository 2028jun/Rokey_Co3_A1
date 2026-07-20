"""Build and open a lightweight pizza restaurant for Isaac Sim 5.1.

The scene deliberately uses a small number of low-poly CC0 Kenney assets and
simple PhysX colliders so it remains responsive during ROS/MoveIt/RMPFlow demos.
"""

from isaacsim import SimulationApp


simulation_app = SimulationApp({"headless": False})

import asyncio
import math
import time
import traceback
from pathlib import Path

import omni.kit.asset_converter
import omni.usd
from omni.kit.viewport.utility import get_active_viewport
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics


WORK_DIR = Path("/home/rokey/cobot3_ws/assets/lightweight_restaurant")
SOURCE_DIR = Path(
    "/home/rokey/cobot3_ws/assets/kenney_furniture/extracted/Models/GLTF format"
)
SCENE_PATH = WORK_DIR / "lightweight_pizza_restaurant.usda"
LIGHTWHEEL_KITCHEN_PATH = Path(
    "/home/rokey/.gemini/antigravity/scratch/assets/Lightwheel_Kitchen/"
    "Collected_KitchenRoom/KitchenRoom.usd"
)

ASSETS = {
    "table": (SOURCE_DIR / "tableCrossCloth.glb", WORK_DIR / "tableCrossCloth.usd"),
    "chair": (SOURCE_DIR / "chairRounded.glb", WORK_DIR / "chairRounded.usd"),
    "plant": (SOURCE_DIR / "pottedPlant.glb", WORK_DIR / "pottedPlant.usd"),
    "counter": (SOURCE_DIR / "kitchenBar.glb", WORK_DIR / "kitchenBar.usd"),
}

# Bounds of the converted assets show that their geometry is still Y-up even
# though the USD stage metadata is Z-up.  The X/Z centers below let each model
# rotate upright around its footprint instead of around a corner.
ASSET_XZ_CENTERS = {
    "table": (0.42606, -0.22369),
    "chair": (0.10000, -0.10000),
    "plant": (0.08458, -0.09597),
    "counter": (0.21500, -0.10500),
}


def _pump_future(future):
    while not future.done():
        simulation_app.update()
    return future.result()


def convert_assets():
    WORK_DIR.mkdir(parents=True, exist_ok=True)
    manager = omni.kit.asset_converter.get_instance()

    for name, (source, output) in ASSETS.items():
        if output.is_file():
            continue
        if not source.is_file():
            raise FileNotFoundError(source)

        context = omni.kit.asset_converter.AssetConverterContext()
        context.keep_all_materials = True
        context.export_preview_surface = True
        context.use_meter_as_world_unit = True
        context.convert_stage_up_z = True
        task = manager.create_converter_task(str(source), str(output), None, context)
        future = asyncio.ensure_future(task.wait_until_finished())
        if not _pump_future(future):
            raise RuntimeError(
                f"Asset conversion failed ({name}): {task.get_error_message()}"
            )
        print(f"[asset converted] {name}: {output}", flush=True)


def add_cube(stage, path, position, scale, color, collision=True, visible=True):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(*position))
    cube.AddScaleOp().Set(Gf.Vec3d(*scale))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if collision:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    if not visible:
        cube.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    return cube


def add_reference(
    stage,
    path,
    asset_path,
    position,
    yaw=0.0,
    scale=1.0,
    center_xz=(0.0, 0.0),
):
    """Place a converted Kenney GLB without overriding its root transforms."""
    placement = UsdGeom.Xform.Define(stage, path)
    placement.AddTranslateOp().Set(Gf.Vec3d(*position))
    placement.AddRotateZOp().Set(float(yaw))
    placement.AddScaleOp().Set(Gf.Vec3d(scale, scale, scale))

    # Kenney GLBs are Y-up.  The converter changed the stage metadata but did
    # not rotate this geometry, so explicitly map +Y to +Z here.
    axis = UsdGeom.Xform.Define(stage, f"{path}/AxisCorrection")
    axis.AddRotateXOp().Set(90.0)

    center = UsdGeom.Xform.Define(stage, f"{path}/AxisCorrection/Center")
    center.AddTranslateOp().Set(
        Gf.Vec3d(-center_xz[0], 0.0, -center_xz[1])
    )

    # Keep the reference on a clean child prim.  Referencing on `placement`
    # would override the converter-authored scale/xformOps on the asset root.
    asset = UsdGeom.Xform.Define(
        stage, f"{path}/AxisCorrection/Center/Asset"
    )
    asset.GetPrim().GetReferences().AddReference(str(asset_path))
    return placement


def add_table_set(stage, index, x, y):
    root = f"/World/Dining/TableSet_{index:02d}"
    UsdGeom.Xform.Define(stage, root)
    add_reference(
        stage,
        f"{root}/TableVisual",
        ASSETS["table"][1],
        (x, y, 0.0),
        scale=2.1,
        center_xz=ASSET_XZ_CENTERS["table"],
    )
    # One conservative collider is cheaper and more stable than per-triangle
    # collision for a serving/navigation demonstration.
    add_cube(
        stage,
        f"{root}/TableCollider",
        (x, y, 0.365),
        (1.80, 0.94, 0.73),
        (0.75, 0.20, 0.12),
        visible=False,
    )

    # Seat four customers along the two long edges.  Both short ends remain
    # clear so the mobile manipulator can approach from the central aisle.
    chair_specs = [
        ((x - 0.50, y - 1.00, 0.0), 180.0),
        ((x + 0.50, y - 1.00, 0.0), 180.0),
        ((x - 0.50, y + 1.00, 0.0), 0.0),
        ((x + 0.50, y + 1.00, 0.0), 0.0),
    ]
    for chair_index, (position, yaw) in enumerate(chair_specs):
        chair_path = f"{root}/Chair_{chair_index:02d}"
        add_reference(
            stage,
            f"{chair_path}_Visual",
            ASSETS["chair"][1],
            position,
            yaw,
            scale=1.8,
            center_xz=ASSET_XZ_CENTERS["chair"],
        )
        add_cube(
            stage,
            f"{chair_path}_Collider",
            (position[0], position[1], 0.42),
            (0.42, 0.42, 0.82),
            (0.36, 0.20, 0.12),
            visible=False,
        )


def add_lightwheel_kitchen(stage):
    """Reference a performance-trimmed Lightwheel kitchen behind the hall."""
    if not LIGHTWHEEL_KITCHEN_PATH.is_file():
        raise FileNotFoundError(LIGHTWHEEL_KITCHEN_PATH)

    placement_path = "/World/LightwheelKitchen"
    placement = UsdGeom.Xform.Define(stage, placement_path)
    # Source ground bounds are approximately x=[-2.64, 2.64],
    # y=[-2.44, 2.55].  This puts its open front directly at hall y=5.
    placement.AddTranslateOp().Set(Gf.Vec3d(0.0, 7.442, 0.0005))

    asset_path = f"{placement_path}/Asset"
    asset = UsdGeom.Xform.Define(stage, asset_path)
    asset.GetPrim().GetReferences().AddReference(str(LIGHTWHEEL_KITCHEN_PATH))

    # Preserve architecture, appliances and material scopes, while preventing
    # the heaviest decorative assets and redundant source lights from loading.
    keep_children = {
        "Kitchen_Wall001",
        "Kitchen_Floor",
        "Kitchen_Windows",
        "Kitchen_Ground",
        "Looks",
        "Kitchen_InsularShelf_01",
        "Kitchen_Cabinet001_01",
        "WallStackOven004_01",
        "Kitchen_TopCabinet_01",
        "Sink054_01",
        "Stovetop012_01",
        "CoffeeMachine006",
        "Refrigerator001",
        "Pot057",
        "Kitchen_Cabinet002",
        "RangeHood015",
        "Toaster003",
        "Microwave017",
        "Dishwasher054_01",
        "WallCollider",
        "PhysicsMaterial",
    }

    source_stage = Usd.Stage.Open(str(LIGHTWHEEL_KITCHEN_PATH), load=Usd.Stage.LoadNone)
    source_root = source_stage.GetDefaultPrim()
    if not source_root:
        raise RuntimeError("Lightwheel Kitchen has no default prim")
    for child in source_root.GetChildren():
        if child.GetName() not in keep_children:
            stage.OverridePrim(f"{asset_path}/{child.GetName()}").SetActive(False)

    print(
        f"[scene build] Lightwheel kitchen connected, "
        f"kept={len(keep_children)} top-level prims",
        flush=True,
    )


def build_stage():
    print("[scene build] create stage", flush=True)
    stage = Usd.Stage.CreateNew(str(SCENE_PATH))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/World").GetPrim())

    print("[scene build] physics and architecture", flush=True)
    physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    physics_scene.CreateGravityMagnitudeAttr(9.81)

    # 12 x 10 m hall with a 2 m entrance gap at the front.
    add_cube(stage, "/World/Architecture/Floor", (0, 0, -0.06), (12, 10, 0.12), (0.32, 0.34, 0.38))
    # Rear wall has a 2.4 m opening into the Lightwheel kitchen.
    add_cube(stage, "/World/Architecture/BackWallLeft", (-3.6, 5, 1.4), (4.8, 0.16, 2.8), (0.92, 0.89, 0.82))
    add_cube(stage, "/World/Architecture/BackWallRight", (3.6, 5, 1.4), (4.8, 0.16, 2.8), (0.92, 0.89, 0.82))
    add_cube(stage, "/World/Architecture/LeftWall", (-6, 0, 1.4), (0.16, 10, 2.8), (0.92, 0.89, 0.82))
    add_cube(stage, "/World/Architecture/RightWall", (6, 0, 1.4), (0.16, 10, 2.8), (0.92, 0.89, 0.82))
    add_cube(stage, "/World/Architecture/FrontWallLeft", (-3.5, -5, 1.4), (5, 0.16, 2.8), (0.92, 0.89, 0.82))
    add_cube(stage, "/World/Architecture/FrontWallRight", (3.5, -5, 1.4), (5, 0.16, 2.8), (0.92, 0.89, 0.82))

    print("[scene build] kitchen", flush=True)
    add_lightwheel_kitchen(stage)

    print("[scene build] dining furniture", flush=True)
    # Four customer tables with a clear 1.5+ m mobile-base aisle.
    for index, (x, y) in enumerate(((-3.2, -2.2), (3.2, -2.2), (-3.2, 0.7), (3.2, 0.7))):
        add_table_set(stage, index, x, y)

    for index, (x, y) in enumerate(((-5.1, 3.9), (5.1, 3.9), (-5.1, -4.0), (5.1, -4.0))):
        add_reference(
            stage,
            f"/World/Decor/Plant_{index:02d}",
            ASSETS["plant"][1],
            (x, y, 0.0),
            yaw=index * 35.0,
            scale=2.0,
            center_xz=ASSET_XZ_CENTERS["plant"],
        )
        add_cube(stage, f"/World/Decor/PlantCollider_{index:02d}", (x, y, 0.45), (0.55, 0.55, 0.9), (0.18, 0.30, 0.12), visible=False)

    print("[scene build] lighting and camera", flush=True)
    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/Dome")
    dome.CreateIntensityAttr(650.0)
    dome.CreateColorAttr(Gf.Vec3f(1.0, 0.86, 0.72))

    camera = UsdGeom.Camera.Define(stage, "/World/Camera")
    camera.CreateFocalLengthAttr(26.0)
    camera_matrix = Gf.Matrix4d().SetLookAt(
        Gf.Vec3d(13.0, -17.0, 14.0),
        Gf.Vec3d(0.0, 2.5, 0.8),
        Gf.Vec3d(0.0, 0.0, 1.0),
    ).GetInverse()
    camera.AddTransformOp().Set(camera_matrix)

    stage.GetRootLayer().Save()
    print(f"[scene built] {SCENE_PATH}", flush=True)


def main():
    convert_assets()
    build_stage()

    context = omni.usd.get_context()
    if not context.open_stage(str(SCENE_PATH)):
        raise RuntimeError(f"Failed to open stage: {SCENE_PATH}")

    for _ in range(90):
        simulation_app.update()

    viewport = get_active_viewport()
    if viewport is not None:
        viewport.set_active_camera("/World/Camera")

    stage = context.get_stage()
    mesh_count = sum(1 for prim in stage.Traverse() if prim.IsA(UsdGeom.Mesh))
    collider_count = sum(
        1 for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.CollisionAPI)
    )
    print(
        f"[ready] lightweight restaurant: meshes={mesh_count}, "
        f"colliders={collider_count}",
        flush=True,
    )

    while simulation_app.is_running():
        simulation_app.update()
        time.sleep(0.016)


try:
    main()
except BaseException:
    traceback.print_exc()
    raise
finally:
    simulation_app.close()
