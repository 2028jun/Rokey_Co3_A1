"""Open NVIDIA's Restaurant Demo Pack in the Isaac Sim 5.1 GUI."""

from isaacsim import SimulationApp


simulation_app = SimulationApp({"headless": False})

import time

import omni.usd
from pxr import UsdGeom, UsdPhysics


USD_PATH = (
    "/home/rokey/cobot3_ws/assets/nvidia_restaurant/extracted/"
    "Demos/AEC/TowerDemo/RestaurantDemopack/World_RestaurantDemopack.usd"
)


def main():
    context = omni.usd.get_context()
    print(f"[NVIDIA Restaurant] opening: {USD_PATH}", flush=True)

    if not context.open_stage(USD_PATH):
        raise RuntimeError(f"Failed to open stage: {USD_PATH}")

    # Allow references, MDL materials and textures to populate the viewport.
    for _ in range(180):
        simulation_app.update()

    stage = context.get_stage()
    mesh_count = 0
    collider_count = 0
    rigid_body_count = 0
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            mesh_count += 1
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            collider_count += 1
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid_body_count += 1

    print(f"[NVIDIA Restaurant] loaded: {stage.GetRootLayer().realPath}", flush=True)
    print(
        "[NVIDIA Restaurant] "
        f"meshes={mesh_count}, colliders={collider_count}, "
        f"rigid_bodies={rigid_body_count}",
        flush=True,
    )

    while simulation_app.is_running():
        simulation_app.update()
        time.sleep(0.016)


try:
    main()
finally:
    simulation_app.close()
