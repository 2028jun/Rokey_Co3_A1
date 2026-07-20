"""Open the Lightwheel Kitchen stage in the Isaac Sim GUI."""

from pathlib import Path

from isaacsim import SimulationApp


simulation_app = SimulationApp({"headless": False})

import time

import omni.usd


WORKSPACE = Path(__file__).resolve().parents[1]
USD_PATH = (
    WORKSPACE
    / "assets/Lightwheel_Kitchen/Collected_KitchenRoom/KitchenRoom.usd"
)


def main():
    context = omni.usd.get_context()
    print(f"[Lightwheel Kitchen] opening: {USD_PATH}", flush=True)

    if not USD_PATH.is_file():
        raise FileNotFoundError(
            f"Lightwheel Kitchen asset is missing: {USD_PATH}\n"
            "Extract the runtime archive at the workspace root."
        )

    if not context.open_stage(str(USD_PATH)):
        raise RuntimeError(f"Failed to open stage: {USD_PATH}")

    # Pump frames while USD references, materials, and textures are populated.
    for _ in range(120):
        simulation_app.update()

    stage = context.get_stage()
    print(
        f"[Lightwheel Kitchen] loaded: {stage.GetRootLayer().realPath}",
        flush=True,
    )

    while simulation_app.is_running():
        simulation_app.update()
        time.sleep(0.016)


try:
    main()
finally:
    simulation_app.close()
