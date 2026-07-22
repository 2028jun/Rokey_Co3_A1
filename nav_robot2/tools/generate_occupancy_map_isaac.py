#!/usr/bin/env python3
"""Generate occupancy map.pgm/yaml from the restaurant USD via Isaac omap API.

Run with Isaac's python (not system python), for example:

  cd ~/git/Rokey_Co3_A1/nav_robot2
  /path/to/isaacsim/.../python.sh tools/generate_occupancy_map_isaac.py

Outputs:
  maps/restaurant/map.pgm
  maps/restaurant/map.yaml
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

from isaacsim import SimulationApp

HEADLESS = os.environ.get("NAV_OMAP_HEADLESS", "1") == "1"
simulation_app = SimulationApp({"headless": HEADLESS})

import numpy as np
import omni.kit.app
import omni.physx
import omni.timeline
import omni.usd
from isaacsim.core.utils.stage import is_stage_loading
from pxr import PhysxSchema

_em = omni.kit.app.get_app().get_extension_manager()
_em.set_extension_enabled_immediate("isaacsim.asset.gen.omap", True)
for _ in range(15):
    simulation_app.update()

from isaacsim.asset.gen.omap.bindings import _omap

WORKSPACE = Path(__file__).resolve().parents[1]
RESTAURANT_USD = WORKSPACE / "assets/lightweight_restaurant/lightweight_pizza_restaurant.usda"
OUT_DIR = WORKSPACE / "maps" / "restaurant"

CELL_SIZE = 0.05
LOWER = (-6.5, -5.5, 0.1)
UPPER = (6.5, 9.5, 0.62)
ORIGIN_XY = (LOWER[0], LOWER[1])


def write_ros_map(buffer, dims, out_dir: Path) -> None:
    width, height = int(dims[0]), int(dims[1])
    raw = np.asarray(buffer, dtype=np.float32).reshape(height, width)
    flipped = np.flipud(raw)
    pgm = np.full((height, width), 205, dtype=np.uint8)
    pgm[flipped >= 0.99] = 0
    pgm[flipped <= 0.01] = 254

    out_dir.mkdir(parents=True, exist_ok=True)
    pgm_path = out_dir / "map.pgm"
    yaml_path = out_dir / "map.yaml"
    pgm_path.write_bytes(
        f"P5\n{width} {height}\n255\n".encode("ascii") + pgm.tobytes()
    )
    yaml_path.write_text(
        (
            f"image: map.pgm\n"
            f"mode: trinary\n"
            f"resolution: {CELL_SIZE}\n"
            f"origin: [{ORIGIN_XY[0]}, {ORIGIN_XY[1]}, 0.0]\n"
            f"negate: 0\n"
            f"occupied_thresh: 0.65\n"
            f"free_thresh: 0.25\n"
        ),
        encoding="utf-8",
    )
    print(f"wrote {pgm_path} ({width}x{height})")
    print(f"wrote {yaml_path}")
    occ = int((pgm == 0).sum())
    free = int((pgm == 254).sum())
    print(f"cells occupied={occ} free={free} unknown={width * height - occ - free}")


def main() -> int:
    if not RESTAURANT_USD.is_file():
        print(f"missing stage: {RESTAURANT_USD}", file=sys.stderr)
        print("Run: ./tools/sync_restaurant_assets.sh", file=sys.stderr)
        return 1

    context = omni.usd.get_context()
    if not context.open_stage(str(RESTAURANT_USD)):
        print(f"failed to open {RESTAURANT_USD}", file=sys.stderr)
        return 1
    for _ in range(60):
        simulation_app.update()
    while is_stage_loading():
        simulation_app.update()

    stage = context.get_stage()
    scene_prim = stage.GetPrimAtPath("/World/PhysicsScene")
    if scene_prim.IsValid():
        physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene_prim)
        physx_scene.CreateEnableStabilizationAttr(True)
        physx_scene.CreateEnableGPUDynamicsAttr(False)

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(120):
        simulation_app.update()

    physx = omni.physx.get_physx_interface()
    generator = _omap.Generator(physx, context.get_stage_id())
    generator.update_settings(CELL_SIZE, 4, 5, 6)
    mid = (
        0.5 * (LOWER[0] + UPPER[0]),
        0.5 * (LOWER[1] + UPPER[1]),
        0.5 * (LOWER[2] + UPPER[2]),
    )
    generator.set_transform(mid, LOWER, UPPER)
    generator.generate2d()
    for _ in range(5):
        simulation_app.update()

    buffer = generator.get_buffer()
    dims = generator.get_dimensions()
    if not buffer or not dims:
        print("omap returned empty buffer", file=sys.stderr)
        timeline.stop()
        simulation_app.close()
        return 1

    width, height = int(dims[0]), int(dims[1])
    raw = np.asarray(buffer, dtype=np.float32).reshape(height, width)
    occ_frac = float((raw >= 0.99).mean())
    free_frac = float((raw <= 0.01).mean())
    print(
        f"omap buffer: occupied>={occ_frac:.1%} free<={free_frac:.1%} "
        f"min={float(raw.min()):.3f} max={float(raw.max()):.3f}"
    )
    if occ_frac > 0.95 and free_frac < 0.02:
        print(
            "omap degenerate (almost all occupied) — "
            "run: python3 tools/generate_placeholder_map.py",
            file=sys.stderr,
        )
        timeline.stop()
        simulation_app.close()
        return 1

    write_ros_map(buffer, dims, OUT_DIR)
    timeline.stop()
    simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
