#!/usr/bin/env python3
"""RMPFlow obstacle-avoidance demo matching the MoveIt comparison scene.

The obstacle pose/size and joint-space goal intentionally match the existing
MoveIt demo.  Run this file with Isaac Sim's python.sh, not system Python.

Target: Isaac Sim 5.1.0-rc.19.
"""

import os

from isaacsim import SimulationApp


HEADLESS = os.environ.get("ISAAC_HEADLESS", "0") == "1"
TEST_STEPS = int(os.environ.get("ISAAC_TEST_STEPS", "0"))
SHOW_COLLISION_SPHERES = os.environ.get("RMPFLOW_SHOW_SPHERES", "0") == "1"

simulation_app = SimulationApp({"headless": HEADLESS})

from pathlib import Path

import numpy as np
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import FixedCuboid, VisualSphere
from isaacsim.core.prims import SingleArticulation
import isaacsim.robot_motion.motion_generation as mg
from pxr import Usd, UsdGeom, UsdPhysics


THIS_DIR = Path(__file__).resolve().parent
USD_PATH = THIS_DIR / "Collected_m0609_camera2" / "m0609_gripper.usd"
URDF_PATH = THIS_DIR / "doosan-robot2" / "urdf" / "m0609_isaac_sim.urdf"
ROBOT_DESCRIPTION_PATH = THIS_DIR / "rmpflow" / "m0609_description.yaml"
RMPFLOW_CONFIG_PATH = THIS_DIR / "rmpflow" / "m0609_rmpflow_common.yaml"

EXPECTED_ROBOT_PRIM_PATH = "/World/m0609"
OBSTACLE_PRIM_PATH = "/World/MoveItDemoObstacle"
GOAL_MARKER_PRIM_PATH = "/World/RMPFlowGoal"

ARM_JOINTS = [f"joint_{index}" for index in range(1, 7)]

# This is the joint state observed at the start of the MoveIt demo.
START_JOINTS_DEG = np.array([0.0, 0.0, 91.4, 0.0, 94.0, 0.0])

# Same goal as moveit_joint_test and the obstacle_demo_goal SRDF state.
GOAL_JOINTS_DEG = np.array([20.0, -30.0, 55.0, 0.0, 55.0, 10.0])


def env_float(name: str, default: float) -> float:
    return float(os.environ.get(name, str(default)))


OBSTACLE_POSITION = np.array(
    [
        env_float("DEMO_OBSTACLE_X", 0.50),
        env_float("DEMO_OBSTACLE_Y", 0.08),
        env_float("DEMO_OBSTACLE_Z", 0.40),
    ]
)
OBSTACLE_SIZE = np.array(
    [
        env_float("DEMO_OBSTACLE_SIZE_X", 0.10),
        env_float("DEMO_OBSTACLE_SIZE_Y", 0.30),
        env_float("DEMO_OBSTACLE_SIZE_Z", 0.24),
    ]
)
START_DELAY_SEC = env_float("RMPFLOW_START_DELAY_SEC", 10.0)
GOAL_TOLERANCE_DEG = env_float("RMPFLOW_GOAL_TOLERANCE_DEG", 2.0)
PHYSICS_DT = 1.0 / 60.0


def load_robot_reference() -> str:
    """Load the collected M0609 USD and return its articulation root path."""
    stage = omni.usd.get_context().get_stage()
    world_prim = stage.GetPrimAtPath("/World")
    if not world_prim.IsValid():
        world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
    world_prim.GetReferences().AddReference(str(USD_PATH))

    for _ in range(20):
        simulation_app.update()

    robot_prim = stage.GetPrimAtPath(EXPECTED_ROBOT_PRIM_PATH)
    if not robot_prim.IsValid():
        raise RuntimeError(
            f"robot prim {EXPECTED_ROBOT_PRIM_PATH} not found after loading {USD_PATH}"
        )

    articulation_paths = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    robot_paths = [
        path
        for path in articulation_paths
        if path == EXPECTED_ROBOT_PRIM_PATH
        or path.startswith(EXPECTED_ROBOT_PRIM_PATH + "/")
        or EXPECTED_ROBOT_PRIM_PATH.startswith(path + "/")
    ]
    if not robot_paths:
        raise RuntimeError(
            "no ArticulationRootAPI found for the M0609; "
            f"articulation roots on stage: {articulation_paths}"
        )

    robot_path = min(
        robot_paths,
        key=lambda path: (
            0 if path == EXPECTED_ROBOT_PRIM_PATH else 1,
            abs(path.count("/") - EXPECTED_ROBOT_PRIM_PATH.count("/")),
        ),
    )
    print(f"[RMPFlow demo] M0609 articulation root: {robot_path}")
    return robot_path


def tune_robot_drives() -> None:
    """Use the same stiff position-drive settings as the Pick & Place demo."""
    stage = omni.usd.get_context().get_stage()
    drive_count = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(EXPECTED_ROBOT_PRIM_PATH)):
        for drive_type in ("angular", "linear"):
            drive = UsdPhysics.DriveAPI.Get(prim, drive_type)
            if drive:
                drive.GetStiffnessAttr().Set(1.0e8)
                drive.GetDampingAttr().Set(1.0e4)
                drive.GetMaxForceAttr().Set(1.0e8)
                drive_count += 1
    print(f"[RMPFlow demo] tuned {drive_count} articulation drives")


def validate_files() -> None:
    for path in (
        USD_PATH,
        URDF_PATH,
        ROBOT_DESCRIPTION_PATH,
        RMPFLOW_CONFIG_PATH,
    ):
        if not path.is_file():
            raise FileNotFoundError(path)


def main() -> None:
    validate_files()

    world = World(
        physics_dt=PHYSICS_DT,
        rendering_dt=PHYSICS_DT,
        stage_units_in_meters=1.0,
    )
    robot_prim_path = load_robot_reference()
    tune_robot_drives()

    robot = world.scene.add(
        SingleArticulation(prim_path=robot_prim_path, name="m0609_robot")
    )

    # FixedCuboid gives the stage a real collider as well as the RMPFlow proxy.
    obstacle = world.scene.add(
        FixedCuboid(
            prim_path=OBSTACLE_PRIM_PATH,
            name="moveit_demo_obstacle",
            position=OBSTACLE_POSITION,
            scale=OBSTACLE_SIZE,
            size=1.0,
            color=np.array([0.9, 0.08, 0.04]),
        )
    )

    world.reset()
    robot.initialize()

    rmp_flow = mg.lula.motion_policies.RmpFlow(
        robot_description_path=str(ROBOT_DESCRIPTION_PATH),
        rmpflow_config_path=str(RMPFLOW_CONFIG_PATH),
        urdf_path=str(URDF_PATH),
        end_effector_frame_name="link_6",
        maximum_substep_size=0.00334,
    )
    articulation_policy = mg.ArticulationMotionPolicy(robot, rmp_flow, PHYSICS_DT)
    active_joints = articulation_policy.get_active_joints_subset()

    if active_joints.joint_names != ARM_JOINTS:
        raise RuntimeError(
            f"RMPFlow active joints are {active_joints.joint_names}, expected {ARM_JOINTS}"
        )

    robot_position, robot_orientation = robot.get_world_pose()
    rmp_flow.set_robot_base_pose(robot_position, robot_orientation)

    start_joints = np.deg2rad(START_JOINTS_DEG)
    goal_joints = np.deg2rad(GOAL_JOINTS_DEG)
    active_joints.set_joint_positions(start_joints)
    active_joints.set_joint_velocities(np.zeros(6))
    active_joints.apply_action(
        joint_positions=start_joints,
        joint_velocities=np.zeros(6),
    )

    # The obstacle has the exact same center and dimensions as the MoveIt demo.
    rmp_flow.add_obstacle(obstacle, static=True)

    # A c-space target gives this demo the same final six-joint state as MoveIt.
    # Removing the EE target makes the c-space attractor the active goal while
    # the collision RMP bends the local motion around the registered box.
    rmp_flow.set_end_effector_target(None)
    rmp_flow.set_cspace_target(start_joints)
    rmp_flow.update_world()

    # Initialize RMPFlow's internal joint state before creating debug spheres.
    # The returned action is intentionally not applied; the filming countdown
    # below holds the exact start state instead.
    articulation_policy.get_next_articulation_action()

    start_position, start_rotation = rmp_flow.get_end_effector_pose(start_joints)
    goal_position, goal_rotation = rmp_flow.get_end_effector_pose(goal_joints)
    gripper_tip_offset = np.array([0.0, 0.0, 0.220])
    start_tip_position = start_position + start_rotation @ gripper_tip_offset
    goal_tip_position = goal_position + goal_rotation @ gripper_tip_offset
    VisualSphere(
        prim_path=GOAL_MARKER_PRIM_PATH,
        name="rmpflow_goal",
        position=goal_position,
        radius=0.025,
        color=np.array([0.0, 1.0, 0.0]),
    )

    if SHOW_COLLISION_SPHERES:
        rmp_flow.visualize_collision_spheres()

    print("[RMPFlow demo] obstacle registered in RMPFlow")
    print(
        f"[RMPFlow demo] obstacle position={OBSTACLE_POSITION.tolist()}, "
        f"size={OBSTACLE_SIZE.tolist()}"
    )
    print(f"[RMPFlow demo] start={START_JOINTS_DEG.tolist()} deg")
    print(f"[RMPFlow demo] goal={GOAL_JOINTS_DEG.tolist()} deg")
    print(
        f"[RMPFlow demo] gripper tip start={start_tip_position.round(4).tolist()}, "
        f"goal={goal_tip_position.round(4).tolist()}"
    )
    print(
        f"[RMPFlow demo] holding start for {START_DELAY_SEC:.1f} s, "
        "then enabling the goal"
    )

    elapsed = 0.0
    goal_enabled = False
    goal_reached = False
    step_count = 0

    try:
        while simulation_app.is_running():
            world.step(render=not HEADLESS)
            if not world.is_playing():
                continue

            elapsed += PHYSICS_DT
            step_count += 1

            if not goal_enabled:
                # Keep the arm completely still during the filming countdown.
                # Running RMPFlow here would blend the start attractor with the
                # nearby obstacle repulsion and make the arm creep or wiggle.
                active_joints.apply_action(
                    joint_positions=start_joints,
                    joint_velocities=np.zeros(6),
                )
                if elapsed < START_DELAY_SEC:
                    continue

                rmp_flow.set_cspace_target(goal_joints)
                goal_enabled = True
                print("[RMPFlow demo] goal enabled: obstacle avoidance running")

            # Static obstacles do not need pose updates, but this keeps the loop
            # identical to the future moving-hand implementation.
            rmp_flow.update_world()
            action = articulation_policy.get_next_articulation_action()
            robot.apply_action(action)

            if goal_enabled and not goal_reached:
                current_joints = active_joints.get_joint_positions()
                max_error_deg = float(
                    np.max(np.abs(np.rad2deg(goal_joints - current_joints)))
                )
                if max_error_deg <= GOAL_TOLERANCE_DEG:
                    goal_reached = True
                    print(
                        "[RMPFlow demo] goal reached: "
                        f"max joint error={max_error_deg:.2f} deg"
                    )

            if TEST_STEPS > 0 and step_count >= TEST_STEPS:
                print(f"[RMPFlow demo] smoke test completed: {step_count} steps")
                break
    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()
