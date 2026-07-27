"""Fast two-robot plate-rack-only serving test.

Robot 1 serves table 0 and robot 2 serves table 1.  This skips pizza, soda,
cutlery, ROS and navigation so plate-rack tray deployment, grasp, transfer and
placement can be iterated quickly.
"""

from __future__ import annotations

import os
import time
import traceback

# Importing dual_pizza_test creates SimulationApp through nav_restaurant_demo.
# Isaac Sim 5.1 exposes pxr only after SimulationApp has been constructed.
import dual_pizza_test as dual
from pxr import UsdGeom

from plate_rack_pick_place import PlateRackPickPlace
from plate_rack_serving import follow_plate_rack_transport, spawn_plate_rack


def _create_plate_tasks(stage, initialized):
    UsdGeom.Scope.Define(stage, "/World/RobotPayloads")
    tasks = []
    for config, articulation, dof_names in initialized:
        UsdGeom.Scope.Define(stage, config["payload_root"])
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

        task = PlateRackPickPlace(
            stage,
            payload_root=config["payload_root"],
            robot_root=config["root"],
        )
        task.initialize(articulation, dof_names)
        tasks.append((config, articulation, task))
        print(
            f"[dual-plate] {config['name']} ready at table={config['table']} "
            f"payload_root={config['payload_root']}",
            flush=True,
        )
    return tasks


def main():
    stage, initialized = dual._spawn_robots()
    tasks = _create_plate_tasks(stage, initialized)
    status_reported = set()
    auto_exit = os.environ.get("DUAL_PLATE_AUTO_EXIT", "0") == "1"

    print(
        "[dual-plate] simultaneous plate-rack-only test started "
        "(robot1=table0, robot2=table1)",
        flush=True,
    )
    while dual.demo.simulation_app.is_running():
        dual.demo.simulation_app.update()
        for config, articulation, task in tasks:
            if not task.done and not task.failed:
                task.step(articulation)
            if (task.done or task.failed) and config["name"] not in status_reported:
                result = "COMPLETED" if task.done else "FAILED"
                print(
                    f"[dual-plate] {config['name']} table={config['table']} "
                    f"result={result}",
                    flush=True,
                )
                status_reported.add(config["name"])

        if len(status_reported) == len(tasks):
            print(
                "[dual-plate] all tasks finished; "
                + ", ".join(
                    f"{config['name']}={'COMPLETED' if task.done else 'FAILED'}"
                    for config, _articulation, task in tasks
                ),
                flush=True,
            )
            if auto_exit:
                break
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
        dual.demo.simulation_app.close()
