"""Print composition and bounds for converted lightweight restaurant assets."""

from isaacsim import SimulationApp


simulation_app = SimulationApp({"headless": True})

import sys

from pxr import Usd, UsdGeom


DEFAULT_PATHS = [
    "/home/rokey/cobot3_ws/assets/lightweight_restaurant/tableCrossCloth.usd",
    "/home/rokey/cobot3_ws/assets/lightweight_restaurant/chairRounded.usd",
    "/home/rokey/cobot3_ws/assets/lightweight_restaurant/kitchenBar.usd",
]

PATHS = sys.argv[1:] or DEFAULT_PATHS

for path in PATHS:
    stage = Usd.Stage.Open(path)
    default_prim = stage.GetDefaultPrim()
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    bounds = cache.ComputeWorldBound(stage.GetPseudoRoot()).ComputeAlignedRange()
    print(
        f"{path}\n"
        f"  default={default_prim.GetPath() if default_prim else None}\n"
        f"  up={UsdGeom.GetStageUpAxis(stage)} "
        f"meters={UsdGeom.GetStageMetersPerUnit(stage)}\n"
        f"  min={bounds.GetMin()} max={bounds.GetMax()} size={bounds.GetSize()}",
        flush=True,
    )
    print(
        "  root_children="
        + ", ".join(str(prim.GetPath()) for prim in stage.GetPseudoRoot().GetChildren()),
        flush=True,
    )
    if default_prim:
        print("  default_children:", flush=True)
        for child in default_prim.GetChildren():
            child_bounds = cache.ComputeWorldBound(child).ComputeAlignedRange()
            print(
                f"    {child.GetPath()} type={child.GetTypeName()} "
                f"min={child_bounds.GetMin()} max={child_bounds.GetMax()}",
                flush=True,
            )

simulation_app.close()
