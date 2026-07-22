from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})     # 1. Application

import numpy as np
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

world = World(stage_units_in_meters=1.0)                # 2. World
stage = omni.usd.get_context().get_stage()              # 3. Stage


cube_prim2 = DynamicCuboid(                              # 4. Prim
    prim_path="/World/RedCube",
    name="red_cube",
    position=np.array([0.0, 0.0, 0.05]),
    scale=np.array([0.1, 0.1, 0.1]),
    color=np.array([1.0, 0.0, 0.0]),
)

world.scene.add_default_ground_plane()                  # 5. Scene
world.scene.add(cube_prim2)

world.reset()

step_count = 0
reset_needed = False
cube_moved = False

while simulation_app.is_running():                      # 6. Simulation
    world.step(render=True)

    if world.is_stopped():
        reset_needed = True

    if world.is_playing():
        if reset_needed:
            world.reset()
            step_count = 0
            cube_moved = False
            print("\n[리셋] 처음부터 다시 시작")
            reset_needed = False

        step_count += 1
        print(f"\rstep: {step_count}", end="", flush=True)

        if step_count == 300 and not cube_moved:
            position, orientation = cube_prim2.get_world_pose()
            position[2] = 1.0
            cube_prim2.set_world_pose(position=position, orientation=orientation)
            cube_moved = True
            print("\n[이동] 300 step - 큐브를 1m 높이로 순간이동")

print(f"\nFinal loop count: {step_count}")
simulation_app.close()
