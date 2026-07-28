"""Fast two-robot full serving test without navigation or ROS nodes.

Robot 1 starts docked at table 0 and robot 2 at table 1.  Each robot owns an
independent payload root and delivers pizza, one soda, cutlery and one plate
rack.  Both sequences are stepped in the same simulation loop to reproduce the
physical load of simultaneous serving without waiting for Nav2, HMI or fleet
routing.

Run with Isaac Sim's Python from the cobot3 workspace root.
"""

from __future__ import annotations

import math
import os
import sys
import time
import traceback
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]

# nav_restaurant_demo owns the proven two-robot composition and physics setup.
# Importing it creates SimulationApp, but its main() (ROS bridges, sensors,
# pedestrians and navigation) is not called by this test.
os.environ.setdefault("PROJECT_WS", str(WORKSPACE))
os.environ.setdefault("NAV_CROSSING_PEDESTRIAN", "0")
os.environ.setdefault("NAV_TYPING_CUSTOMER", "0")

if str(WORKSPACE / "isaacpjt") not in sys.path:
    sys.path.insert(0, str(WORKSPACE / "isaacpjt"))

import nav_restaurant_demo as demo
from pxr import Gf, UsdGeom

from pizza_serving import TrayPizzaPickPlace
from drink_serving import spawn_soda_cans
from cutlery_serving import spawn_cutlery_box
from plate_rack_serving import follow_plate_rack_transport, spawn_plate_rack
from soda1_delivery import Soda1PickPlace
from cutlery_pick_place import CutleryBoxPickPlace
from plate_rack_pick_place import PlateRackPickPlace
from delivery_sequence import CommandServingSequence


ROBOT_CONFIGS = (
    {
        "name": "robot1",
        "root": "/World/NavRobot1",
        "payload_root": "/World/RobotPayloads/robot1",
        "spawn": Gf.Vec3d(-1.72, -2.20, 0.01),
        "yaw": math.pi,
        "table": 0,
    },
    {
        "name": "robot2",
        "root": "/World/NavRobot2",
        "payload_root": "/World/RobotPayloads/robot2",
        "spawn": Gf.Vec3d(1.72, -2.20, 0.01),
        "yaw": 0.0,
        "table": 1,
    },
)


def _spawn_robots():
    """Compose and initialize both docked mobile manipulators."""
    demo.import_robot_usd()

    stage = None
    records = []
    for index, config in enumerate(ROBOT_CONFIGS):
        demo.SPAWN_YAW = config["yaw"]
        demo.set_robot_context(config["root"], config["spawn"])
        stage = demo.open_restaurant_and_robot(open_stage=index == 0)
        demo.configure_wheel_contact_material(stage)
        demo.configure_gripper_contact_material(stage)
        articulation_path = demo.find_articulation_path(stage)
        demo.configure_physics_stability(stage, articulation_path)
        demo.prepare_parking_brake(stage, articulation_path)
        records.append((config, articulation_path))

    demo.configure_joint_drives(stage)

    # Build one complete two-robot PhysX scene before binding either
    # articulation.  Starting physics inside each initialize call allowed a
    # later stage resync to race robot2's articulation-view creation.
    for _ in range(10):
        demo.simulation_app.update()
    timeline = demo.omni.timeline.get_timeline_interface()
    timeline.play()
    physics_settle_frames = max(
        2, int(os.environ.get("NAV_PHYSICS_INIT_SETTLE_FRAMES", "8"))
    )
    for _ in range(physics_settle_frames):
        demo.simulation_app.update()

    initialized = []
    for config, articulation_path in records:
        demo.SPAWN_YAW = config["yaw"]
        demo.set_robot_context(config["root"], config["spawn"])
        articulation, dof_names = demo.initialize_robot(
            articulation_path,
            name=f"dual_serving_{config['name']}",
        )
        demo.add_parking_brake(stage, articulation.prim_path)
        initialized.append((config, articulation, dof_names))

    return stage, initialized


def _create_tasks(stage, initialized):
    """Author one isolated full payload sequence for each robot."""
    UsdGeom.Scope.Define(stage, "/World/RobotPayloads")
    tasks = []
    for config, articulation, dof_names in initialized:
        UsdGeom.Scope.Define(stage, config["payload_root"])
        pizza = TrayPizzaPickPlace(
            stage,
            payload_root=config["payload_root"],
            robot_root=config["root"],
        )
        spawn_soda_cans(
            stage,
            count=1,
            payload_root=config["payload_root"],
            robot_root=config["root"],
        )
        spawn_cutlery_box(
            stage,
            payload_root=config["payload_root"],
            robot_root=config["root"],
        )
        spawn_plate_rack(
            stage,
            plate_count=1,
            payload_root=config["payload_root"],
            robot_root=config["root"],
        )
        if not follow_plate_rack_transport(
            stage,
            payload_root=config["payload_root"],
            robot_root=config["root"],
        ):
            raise RuntimeError(
                f"{config['name']} plate rack could not be placed on its tray"
            )

        named_tasks = [
            ("pizza", pizza),
            (
                "soda1",
                Soda1PickPlace(
                    stage,
                    wait_for_start=True,
                    payload_root=config["payload_root"],
                    robot_root=config["root"],
                ),
            ),
            (
                "cutlery",
                CutleryBoxPickPlace(
                    stage,
                    wait_for_start=True,
                    payload_root=config["payload_root"],
                    robot_root=config["root"],
                ),
            ),
            (
                "plate_rack",
                PlateRackPickPlace(
                    stage,
                    wait_for_start=True,
                    payload_root=config["payload_root"],
                    robot_root=config["root"],
                ),
            ),
        ]
        task = CommandServingSequence(named_tasks)
        task.initialize(articulation, dof_names)
        tasks.append((config, articulation, task))
        print(
            f"[dual-serving] {config['name']} ready at table={config['table']} "
            f"robot_root={config['root']} payload_root={config['payload_root']}",
            flush=True,
        )
    return tasks


def main():
    stage, initialized = _spawn_robots()
    tasks = _create_tasks(stage, initialized)
    status_reported = set()
    auto_exit = os.environ.get("DUAL_SERVING_AUTO_EXIT", "0") == "1"

    print(
        "[dual-serving] simultaneous full delivery started "
        "(robot1=table0, robot2=table1, order=pizza+soda1+cutlery+plate_rack)",
        flush=True,
    )
    while demo.simulation_app.is_running():
        demo.simulation_app.update()
        for config, articulation, task in tasks:
            if not task.done and not task.failed:
                # Match the integrated NavBridge behavior: the rack is a
                # kinematic payload until its own task starts, so it must be
                # reattached to the live left-tray transform every frame while
                # pizza deploys the trays and the preceding payloads run.
                if task.current_name != "plate_rack":
                    if not follow_plate_rack_transport(
                        stage,
                        payload_root=config["payload_root"],
                        robot_root=config["root"],
                    ):
                        raise RuntimeError(
                            f"{config['name']} plate rack transport follow failed"
                        )
                task.step(articulation)
            if (task.done or task.failed) and config["name"] not in status_reported:
                result = "COMPLETED" if task.done else "FAILED"
                print(
                    f"[dual-serving] {config['name']} table={config['table']} "
                    f"result={result}",
                    flush=True,
                )
                status_reported.add(config["name"])

        if len(status_reported) == len(tasks):
            print(
                "[dual-serving] all tasks finished; "
                + ", ".join(
                    f"{config['name']}={'COMPLETED' if task.done else 'FAILED'}"
                    for config, _articulation, task in tasks
                ),
                flush=True,
            )
            if auto_exit:
                break
            # Keep the final physical state available for visual inspection.
            status_reported.add("summary")

        time.sleep(0.005)

    for _config, _articulation, task in tasks:
        task.close()


if __name__ == "__main__":
    try:
        main()
    except BaseException:
        traceback.print_exc()
        raise
    finally:
        demo.simulation_app.close()
