from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})     # 1. Application

import numpy as np
import time
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

world = World(stage_units_in_meters=1.0)                # 2. World
stage = omni.usd.get_context().get_stage()              # 3. Stage

cube_prim1 = DynamicCuboid(                              # 4. Prim
    prim_path="/World/RedCube",
    name="RedCube",
    position=np.array([0.0, 0.0, 1]),
    scale=np.array([0.15, 0.15, 0.15]),
    color=np.array([1.0, 0.0, 0.0])
)

cube_prim2 = DynamicCuboid(                              # 4. Prim
    prim_path="/World/RedCube",
    name="RedCube",
    position=np.array([0.0, 1.0, 1]),
    scale=np.array([0.15, 0.15, 0.15]),
    color=np.array([1.0, 0.0, 0.0])
)

world.scene.add_default_ground_plane()                  # 5. Scene
world.scene.add(cube_prim1)
world.scene.add(cube_prim2)
world.reset()

step_count = 0

while simulation_app.is_running():                      # 6. Simulation
    world.step(render=True)
    time.sleep(0.01)
    step_count += 1
    if step_count % 100 == 0:
        print(f"step: {step_count}")
    if step_count == 300:
        cube_prim1.set_world_pose(position=np.array([0.0, 0.0, 1.0]))

    if step_count == 500:
        print("시뮬레이션 종료")
        break

simulation_app.close()
