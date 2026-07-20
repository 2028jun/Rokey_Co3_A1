"""Standalone RG2 open/close diagnostic for the mobile manipulator.

This intentionally does not create RMPFlow, cameras, food assets, or the
pick-and-place state machine.  It repeatedly drives the one independent RG2
joint so gripper drive and PhysX mimic behavior can be inspected in isolation.
"""

import os
import time
import traceback

# These must be set before importing the shared Isaac Sim setup module.
os.environ["MOBILE_DEMO_PICK_PLACE"] = "0"
os.environ["MOBILE_DEMO_ROS_CAMERA"] = "0"

import numpy as np

import mobile_manipulator_demo_test as demo
from isaacsim.core.utils.types import ArticulationAction


OPEN_POSITION = 0.0
CLOSED_POSITION = 0.95
HOLD_FRAMES = int(os.environ.get("RG2_TEST_HOLD_FRAMES", "180"))
RG2_DOF_PREFIX = "rg2_"


def command_joint(articulation, joint_index, target):
    articulation.apply_action(
        ArticulationAction(
            joint_positions=np.asarray([target], dtype=float),
            joint_indices=np.asarray([joint_index], dtype=np.int32),
        )
    )


def format_rg2_positions(articulation, dof_names, rg2_indices):
    positions = articulation.get_joint_positions()
    return ", ".join(
        f"{dof_names[index]}={float(positions[index]):+.3f}"
        for index in rg2_indices
    )


def main():
    demo.enable_urdf_importer()
    demo.import_robot_usd()
    stage = demo.open_restaurant_and_reference_robot()
    demo.attach_m0609_visuals(stage)
    for _ in range(10):
        demo.simulation_app.update()

    demo.configure_joint_drives(stage)
    articulation_path = demo.find_articulation_root(stage)
    demo.add_parking_brake(stage, articulation_path)
    demo.configure_physics_stability(stage, articulation_path)
    articulation, dof_names = demo.initialize_robot(articulation_path)

    drive_index = dof_names.index(demo.GRIPPER_JOINTS[0])
    rg2_indices = [
        index
        for index, name in enumerate(dof_names)
        if name.startswith(RG2_DOF_PREFIX)
    ]
    print(
        "[rg2-test] direct drive="
        f"{dof_names[drive_index]} index={drive_index}; "
        f"hold_frames={HOLD_FRAMES}",
        flush=True,
    )
    print(
        "[rg2-test] watch the gripper and verify that mimic DOFs move: "
        + format_rg2_positions(articulation, dof_names, rg2_indices),
        flush=True,
    )

    state = 0
    frame = 0
    targets = (("OPEN", OPEN_POSITION), ("CLOSE", CLOSED_POSITION))
    while demo.simulation_app.is_running():
        label, target = targets[state]
        command_joint(articulation, drive_index, target)
        demo.simulation_app.update()
        frame += 1

        if frame % 30 == 0:
            print(
                f"[rg2-test] command={label} target={target:.3f} "
                + format_rg2_positions(articulation, dof_names, rg2_indices),
                flush=True,
            )
        if frame >= HOLD_FRAMES:
            state = 1 - state
            frame = 0
            next_label, _ = targets[state]
            print(f"[rg2-test] switching to {next_label}", flush=True)
        time.sleep(0.010)


if __name__ == "__main__":
    try:
        main()
    except BaseException:
        traceback.print_exc()
        raise
    finally:
        demo.simulation_app.close()
