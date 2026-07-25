#!/usr/bin/env python3
"""Run the existing M0609 USD with ROS 2 joint state/command OmniGraph.

Target: Isaac Sim 5.1.0-rc.19 and ROS 2 Humble.
Run this file with Isaac Sim's python.sh, not the system Python.
"""

import os

os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')

from isaacsim import SimulationApp


HEADLESS = os.environ.get('ISAAC_HEADLESS', '0') == '1'
TEST_STEPS = int(os.environ.get('ISAAC_TEST_STEPS', '0'))

simulation_app = SimulationApp({'headless': HEADLESS})

from pathlib import Path

import omni.graph.core as og
import omni.usd
import usdrt.Sdf
from isaacsim.core.api import SimulationContext
from isaacsim.core.utils.extensions import enable_extension
from pxr import Gf, Usd, UsdGeom, UsdPhysics


THIS_DIR = Path(__file__).resolve().parent
USD_PATH = THIS_DIR / 'Collected_m0609_camera2' / 'm0609_gripper.usd'
EXPECTED_ROBOT_PRIM_PATH = '/World/m0609'
GRAPH_PATH = '/MoveItROS2Graph'
OBSTACLE_PRIM_PATH = '/World/MoveItDemoObstacle'
ARM_JOINT_NAMES = {f'joint_{index}' for index in range(1, 7)}


def env_float(name: str, default: float) -> float:
    return float(os.environ.get(name, str(default)))


OBSTACLE_POSITION = (
    env_float('DEMO_OBSTACLE_X', 0.50),
    env_float('DEMO_OBSTACLE_Y', 0.08),
    env_float('DEMO_OBSTACLE_Z', 0.40),
)
OBSTACLE_SIZE = (
    env_float('DEMO_OBSTACLE_SIZE_X', 0.10),
    env_float('DEMO_OBSTACLE_SIZE_Y', 0.30),
    env_float('DEMO_OBSTACLE_SIZE_Z', 0.24),
)
ARM_DRIVE_STIFFNESS = env_float('M0609_DRIVE_STIFFNESS', 1.0e8)
ARM_DRIVE_DAMPING = env_float('M0609_DRIVE_DAMPING', 1.0e4)
ARM_DRIVE_MAX_FORCE = env_float('M0609_DRIVE_MAX_FORCE', 1.0e8)


def load_robot() -> str:
    stage = omni.usd.get_context().get_stage()
    world = stage.GetPrimAtPath('/World')
    if not world.IsValid():
        world = UsdGeom.Xform.Define(stage, '/World').GetPrim()
    world.GetReferences().AddReference(str(USD_PATH))
    for _ in range(20):
        simulation_app.update()
    if not stage.GetPrimAtPath(EXPECTED_ROBOT_PRIM_PATH).IsValid():
        raise RuntimeError(
            f'robot prim {EXPECTED_ROBOT_PRIM_PATH} not found after loading {USD_PATH}'
        )

    articulation_paths = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    robot_paths = [
        path for path in articulation_paths
        if path == EXPECTED_ROBOT_PRIM_PATH
        or path.startswith(EXPECTED_ROBOT_PRIM_PATH + '/')
        or EXPECTED_ROBOT_PRIM_PATH.startswith(path + '/')
    ]
    if not robot_paths:
        raise RuntimeError(
            'no ArticulationRootAPI found below '
            f'{EXPECTED_ROBOT_PRIM_PATH}; found on stage: {articulation_paths}'
        )

    # Prefer the expected robot prim, then its nearest ancestor or child. Some
    # exported USDs put ArticulationRootAPI on World or a child such as
    # base_link rather than on the visible robot reference prim itself.
    robot_path = min(
        robot_paths,
        key=lambda path: (
            0 if path == EXPECTED_ROBOT_PRIM_PATH else 1,
            abs(path.count('/') - EXPECTED_ROBOT_PRIM_PATH.count('/')),
        ),
    )
    print(f'[MoveIt bridge] M0609 articulation root: {robot_path}')
    return robot_path


def create_demo_obstacle() -> None:
    """Create the same static box that is registered in MoveIt."""
    stage = omni.usd.get_context().get_stage()
    cube = UsdGeom.Cube.Define(stage, OBSTACLE_PRIM_PATH)
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr([Gf.Vec3f(0.9, 0.08, 0.04)])
    xform = UsdGeom.XformCommonAPI(cube.GetPrim())
    xform.SetTranslate(Gf.Vec3d(*OBSTACLE_POSITION))
    xform.SetScale(Gf.Vec3f(*OBSTACLE_SIZE))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    print(
        '[MoveIt bridge] demo obstacle: '
        f'position={OBSTACLE_POSITION}, size={OBSTACLE_SIZE}'
    )


def tune_arm_drives() -> None:
    """Make the six M0609 position drives stiff and sufficiently damped."""
    stage = omni.usd.get_context().get_stage()
    robot_prim = stage.GetPrimAtPath(EXPECTED_ROBOT_PRIM_PATH)
    if not robot_prim.IsValid():
        raise RuntimeError(f'robot prim not found: {EXPECTED_ROBOT_PRIM_PATH}')

    tuned_joints = set()
    for prim in Usd.PrimRange(robot_prim):
        joint_name = prim.GetName()
        if joint_name not in ARM_JOINT_NAMES:
            continue

        drive = UsdPhysics.DriveAPI.Get(prim, 'angular')
        if not drive:
            continue

        drive.GetStiffnessAttr().Set(ARM_DRIVE_STIFFNESS)
        drive.GetDampingAttr().Set(ARM_DRIVE_DAMPING)
        drive.GetMaxForceAttr().Set(ARM_DRIVE_MAX_FORCE)
        tuned_joints.add(joint_name)

    missing_joints = sorted(ARM_JOINT_NAMES - tuned_joints)
    if missing_joints:
        raise RuntimeError(
            'angular drive not found for M0609 joints: ' + ', '.join(missing_joints)
        )

    print(
        '[MoveIt bridge] arm drives tuned: '
        f'stiffness={ARM_DRIVE_STIFFNESS:g}, '
        f'damping={ARM_DRIVE_DAMPING:g}, '
        f'max_force={ARM_DRIVE_MAX_FORCE:g}'
    )


def create_ros_graph(robot_prim_path: str) -> None:
    og.Controller.edit(
        {'graph_path': GRAPH_PATH, 'evaluator_name': 'execution'},
        {
            og.Controller.Keys.CREATE_NODES: [
                ('OnPlaybackTick', 'omni.graph.action.OnPlaybackTick'),
                ('ReadSimTime', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
                ('Context', 'isaacsim.ros2.bridge.ROS2Context'),
                ('PublishJointState', 'isaacsim.ros2.bridge.ROS2PublishJointState'),
                ('SubscribeJointState', 'isaacsim.ros2.bridge.ROS2SubscribeJointState'),
                ('ArticulationController', 'isaacsim.core.nodes.IsaacArticulationController'),
                ('PublishClock', 'isaacsim.ros2.bridge.ROS2PublishClock'),
            ],
            og.Controller.Keys.CONNECT: [
                ('OnPlaybackTick.outputs:tick', 'PublishJointState.inputs:execIn'),
                ('OnPlaybackTick.outputs:tick', 'SubscribeJointState.inputs:execIn'),
                ('OnPlaybackTick.outputs:tick', 'ArticulationController.inputs:execIn'),
                ('OnPlaybackTick.outputs:tick', 'PublishClock.inputs:execIn'),
                ('Context.outputs:context', 'PublishJointState.inputs:context'),
                ('Context.outputs:context', 'SubscribeJointState.inputs:context'),
                ('Context.outputs:context', 'PublishClock.inputs:context'),
                ('ReadSimTime.outputs:simulationTime', 'PublishJointState.inputs:timeStamp'),
                ('ReadSimTime.outputs:simulationTime', 'PublishClock.inputs:timeStamp'),
                ('SubscribeJointState.outputs:jointNames', 'ArticulationController.inputs:jointNames'),
                ('SubscribeJointState.outputs:positionCommand', 'ArticulationController.inputs:positionCommand'),
                ('SubscribeJointState.outputs:velocityCommand', 'ArticulationController.inputs:velocityCommand'),
                ('SubscribeJointState.outputs:effortCommand', 'ArticulationController.inputs:effortCommand'),
            ],
            og.Controller.Keys.SET_VALUES: [
                ('ArticulationController.inputs:robotPath', robot_prim_path),
                ('PublishJointState.inputs:topicName', 'isaac_joint_states'),
                ('SubscribeJointState.inputs:topicName', 'isaac_joint_commands'),
                ('PublishJointState.inputs:targetPrim', [usdrt.Sdf.Path(robot_prim_path)]),
            ],
        },
    )


def main() -> None:
    if not USD_PATH.is_file():
        raise FileNotFoundError(USD_PATH)

    if not enable_extension('isaacsim.ros2.bridge'):
        raise RuntimeError('failed to enable isaacsim.ros2.bridge')
    # OGN node types are registered during extension startup/update. A single
    # frame is insufficient on some 5.1 source builds.
    for _ in range(10):
        simulation_app.update()

    simulation = SimulationContext(
        physics_dt=1.0 / 100.0,
        rendering_dt=1.0 / 60.0,
        stage_units_in_meters=1.0,
    )
    robot_prim_path = load_robot()
    tune_arm_drives()
    create_demo_obstacle()
    create_ros_graph(robot_prim_path)
    simulation.initialize_physics()
    simulation.play()

    print('[MoveIt bridge] /isaac_joint_states publishing')
    print('[MoveIt bridge] /isaac_joint_commands subscribed')
    print('[MoveIt bridge] /clock publishing, ROS_DOMAIN_ID=' + os.environ.get('ROS_DOMAIN_ID', '0'))

    try:
        step_count = 0
        while simulation_app.is_running():
            simulation.step(render=True)
            step_count += 1
            if TEST_STEPS > 0 and step_count >= TEST_STEPS:
                print(f'[MoveIt bridge] smoke test completed: {step_count} steps')
                break
    finally:
        simulation.stop()
        simulation_app.close()


if __name__ == '__main__':
    main()
