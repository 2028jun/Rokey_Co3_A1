"""Independent soda-can pick-and-place task for the serving robot demo."""

import os
from pathlib import Path

import numpy as np
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot_motion.motion_generation.lula.kinematics import (
    LulaKinematicsSolver,
)
from pxr import Gf, UsdPhysics

from isaac_scene_utils import find_serving_robot_prim, prim_world_pose
from m0609_rmpflow_controller import RMPFlowController


WORKSPACE = Path(
    os.environ.get("COBOT3_WS", Path(__file__).resolve().parents[1])
).resolve()
RMPFLOW_DIR = WORKSPACE / "isaacpjt/M0609/rmpflow"
M0609_RMPFLOW_URDF = (
    WORKSPACE / "isaacpjt/M0609/doosan-robot2/urdf/m0609_isaac_sim.urdf"
)
M0609_DESCRIPTION = RMPFLOW_DIR / "m0609_description.yaml"
M0609_RMPFLOW_CONFIG = RMPFLOW_DIR / "m0609_rmpflow_common.yaml"

ARM_JOINTS = [f"joint_{index}" for index in range(1, 7)]
GRIPPER_JOINT = "rg2_finger_joint"
GRIPPER_OPEN = 0.0
# The 66 mm can contacts near 0.56 rad.  Use only modest over-travel and a
# soda-specific force cap so the rigid cylinder is held rather than ejected.
GRIPPER_CAN_CLOSE = 0.60
GRIPPER_CAN_MAX_FORCE = 15.0
SLIDING_TRAY_JOINTS = (
    "upper_tray_left_slide_joint",
    "upper_tray_right_slide_joint",
)
SLIDING_TRAY_EXTENSION = 0.25
SLIDING_TRAY_DEPLOY_STEPS = 360

SODA1_PICK_PATH = "/World/ServingDrinks/SodaCan_03"
SODA2_PICK_PATH = "/World/ServingDrinks/SodaCan_02"
CAN_HEIGHT = 0.122
RG2_TCP_LENGTH = 0.231066
CAN_APPROACH_HEIGHT = 0.20
CAN_LIFT_HEIGHT = 0.24
# The upright pizza bail cannot be overflown at the table target: downward
# link_6 IK stops converging above roughly TCP Z=1.02 m there.  Raise the can
# only to a reachable 1.00 m, cross to the delivery side on the robot-side of
# the board, then approach through a corridor outside the bail ends.
SAFE_TRANSIT_TCP_WORLD_Z = 1.00
BOARD_NEAR_GATE_LOCAL_X = 0.28
OUTSIDE_BAIL_LOCAL_Y = 0.28
# At the table target, adding the full 240 mm lift clearance plus the 231 mm
# TCP offset placed link_6 outside the practical downward-tool workspace.  A
# 120 mm TCP clearance still keeps the carried can 120 mm above the tabletop.
TABLE_APPROACH_HEIGHT = 0.12
VERTICAL_STEPS = 300
INITIAL_STEPS = 240
DETOUR_STEPS = 240

# This is the table position formerly derived from the pizza-board centre,
# but it is fully defined here so this task does not instantiate pizza assets.
TABLE_BOARD_CENTRE_LOCAL = np.array([0.55, 0.0, -0.14568])
VERTICAL_EE_ORIENTATION = np.array([0.0, 1.0, 0.0, 0.0], dtype=float)
# Lula IK solution for the can-above target on the high-shoulder branch.  Its
# J2/J3 signs differ from the elbow-down solution selected in the old pizza
# stow branch (approximately J2=-107/J3=+94 degrees).
ELBOW_UP_J2 = np.deg2rad(
    float(os.environ.get("MOBILE_SODA_ELBOW_UP_J2_DEG", "-20.0"))
)
ELBOW_UP_J3 = np.deg2rad(
    float(os.environ.get("MOBILE_SODA_ELBOW_UP_J3_DEG", "-94.0"))
)
ELBOW_UP_J1 = np.deg2rad(98.0)
ELBOW_UP_J5 = np.deg2rad(-66.0)
ELBOW_UP_J6 = np.deg2rad(98.0)


def _quaternion_slerp(start, end, amount):
    start = np.asarray(start, dtype=float) / np.linalg.norm(start)
    end = np.asarray(end, dtype=float) / np.linalg.norm(end)
    dot = float(np.dot(start, end))
    if dot < 0.0:
        end = -end
        dot = -dot
    if dot > 0.9995:
        value = start + amount * (end - start)
        return value / np.linalg.norm(value)
    angle = np.arccos(np.clip(dot, -1.0, 1.0))
    return (
        np.sin((1.0 - amount) * angle) * start
        + np.sin(amount * angle) * end
    ) / np.sin(angle)


class SodaCanPickPlace:
    """Deploy the tray and deliver only its robot-left/front soda can."""

    def __init__(
        self,
        stage,
        wait_for_start=False,
        task_name="soda1",
        pick_path=SODA1_PICK_PATH,
        place_left=True,
    ):
        self._stage = stage
        self._arm_base = find_serving_robot_prim(stage, "base_link")
        self._end_effector = find_serving_robot_prim(stage, "link_6")
        _, _, self._arm_to_world = prim_world_pose(self._arm_base)
        self._up = np.array([0.0, 0.0, 1.0])
        self._task_name = task_name
        self._pick_path = pick_path
        self._place_left = place_left
        self._use_bail_detour = place_left
        self._side_label = "left" if place_left else "right"
        self._gripper_close = GRIPPER_CAN_CLOSE
        self._gripper_max_force = GRIPPER_CAN_MAX_FORCE
        self._object_label = "can"
        self._minimum_verified_lift = 0.08
        self._lock_cspace_after_grasp = False
        self._preserve_grasp_orientation = False
        self._carried_orientation = None
        self._use_seeded_ik_for_lift = False
        # Some payloads have disconnected Cartesian IK branches that RMPFlow
        # may switch between.  Subclasses can keep every Cartesian phase on a
        # continuously seeded Lula branch without changing soda/cutlery.
        self._use_seeded_ik_for_all_motion = False
        self._seeded_ik_solver = None
        self._seeded_ik_max_delta = np.deg2rad(10.0)
        self._continuous_rmp_lift = False
        self._vertical_orientation = VERTICAL_EE_ORIENTATION.copy()
        self._grasp_hold_max_error = None
        self._grasp_object_max_lateral = None
        # Optional tighter convergence requirement for the placement descent
        # (phase 6).  The shared 25 mm tolerance is retained for soda/cutlery;
        # large payloads can require the wrist to get closer before release.
        self._placement_settle_error = None
        # Optional contact check for rigid handled payloads.  If the driven
        # joint reaches the requested close angle within this margin, nothing
        # stopped the fingers and the grasp is empty.  Soda/cutlery retain
        # their existing behavior unless a subclass enables this explicitly.
        self._gripper_contact_margin = None
        # Optional partially closed approach posture.  Most payloads approach
        # fully open; tightly packed payloads can start the final close ramp
        # from a narrower collision-safe finger span.
        self._gripper_pregrasp = GRIPPER_OPEN
        self._gripper_close_ramp_steps = 120
        self._gripper_close_wait_steps = 120
        # Ramp lengths for the Cartesian phases.  Subclasses may shorten them;
        # the tested soda/cutlery timings stay at the module defaults.
        self._vertical_steps = VERTICAL_STEPS
        self._initial_steps = INITIAL_STEPS
        self._transfer_steps = INITIAL_STEPS
        self._phase = 0
        self._phase_steps = 0
        self._settled_steps = 0
        self._deploy_steps = 0
        self._trays_deployed = False
        self._active = not wait_for_start
        self._initialized = False
        self._targets = {}
        self._vertical_starts = {}
        self._can_prim = None
        self._can_pick_start = None
        self._grasp_hold_wrist = None
        self._grasp_hold_orientation = None
        self.done = False
        self.failed = False

    def _local_to_world(self, position):
        point = self._arm_to_world.Transform(Gf.Vec3d(*map(float, position)))
        return np.asarray(point, dtype=float)

    def _tcp_to_wrist(self, tcp):
        return tcp + self._up * RG2_TCP_LENGTH

    def _command_gripper(self, articulation, target):
        articulation.apply_action(
            ArticulationAction(
                joint_positions=np.asarray([target], dtype=float),
                joint_indices=np.asarray([self._gripper_index], dtype=np.int32),
            )
        )

    def _enable_soda_grip_force(self):
        self._gripper_drive.GetMaxForceAttr().Set(self._gripper_max_force)
        print(
            f"[{self._task_name}-gripper] "
            f"max_force={self._gripper_max_force:.1f}N",
            flush=True,
        )

    def initialize(self, articulation, dof_names):
        required = [*ARM_JOINTS, GRIPPER_JOINT, *SLIDING_TRAY_JOINTS]
        missing = [name for name in required if name not in dof_names]
        if missing:
            raise RuntimeError(f"missing soda-task DOFs: {missing}")
        self._gripper_index = dof_names.index(GRIPPER_JOINT)
        gripper_joint_prim = find_serving_robot_prim(self._stage, GRIPPER_JOINT)
        self._gripper_drive = UsdPhysics.DriveAPI.Get(
            gripper_joint_prim, "angular"
        )
        if not self._gripper_drive:
            raise RuntimeError("RG2 angular drive is missing")
        self._arm_indices = np.asarray(
            [dof_names.index(name) for name in ARM_JOINTS], dtype=np.int32
        )
        self._tray_indices = np.asarray(
            [dof_names.index(name) for name in SLIDING_TRAY_JOINTS],
            dtype=np.int32,
        )
        gripper = ParallelGripper(
            end_effector_prim_path=str(self._end_effector.GetPath()),
            joint_prim_names=[GRIPPER_JOINT],
            joint_opened_positions=np.array([GRIPPER_OPEN]),
            joint_closed_positions=np.array([self._gripper_close]),
            action_deltas=np.array([-self._gripper_close]),
            use_mimic_joints=True,
        )
        gripper.initialize(
            articulation_apply_action_func=articulation.apply_action,
            get_joint_positions_func=articulation.get_joint_positions,
            set_joint_positions_func=articulation.set_joint_positions,
            dof_names=dof_names,
        )
        gripper.set_joint_positions(np.array([GRIPPER_OPEN]))
        arm_position, arm_orientation, _ = prim_world_pose(self._arm_base)
        self._controller = RMPFlowController(
            name="soda_can_rmpflow",
            robot_articulation=articulation,
            urdf_path=str(M0609_RMPFLOW_URDF),
            robot_description_path=str(M0609_DESCRIPTION),
            rmpflow_config_path=str(M0609_RMPFLOW_CONFIG),
            end_effector_frame_name="link_6",
        )
        self._controller.rmp_flow.set_robot_base_pose(
            robot_position=arm_position,
            robot_orientation=arm_orientation,
        )
        self._lift_ik = None
        if self._use_seeded_ik_for_lift:
            self._lift_ik = LulaKinematicsSolver(
                str(M0609_DESCRIPTION), str(M0609_RMPFLOW_URDF)
            )
            self._lift_ik.set_robot_base_pose(
                robot_position=arm_position,
                robot_orientation=arm_orientation,
            )
        elbow_up_posture = np.asarray(
            articulation.get_joint_positions()[
                self._arm_indices
            ],
            dtype=float,
        ).copy()
        elbow_up_posture[0] = ELBOW_UP_J1
        elbow_up_posture[1] = ELBOW_UP_J2
        elbow_up_posture[2] = ELBOW_UP_J3
        elbow_up_posture[4] = ELBOW_UP_J5
        elbow_up_posture[5] = ELBOW_UP_J6
        self._controller.rmp_flow.set_cspace_target(elbow_up_posture)
        self._initial_wrist, self._initial_orientation, _ = prim_world_pose(
            self._end_effector
        )
        self._initialized = True
        if self._active:
            self._enable_soda_grip_force()
        print(
            f"[{self._task_name}] ready with elbow-up posture bias "
            f"J2={np.degrees(ELBOW_UP_J2):.1f}deg "
            f"J3={np.degrees(ELBOW_UP_J3):.1f}deg; "
            f"deploying trays before reading live {self._object_label} pose",
            flush=True,
        )
        if not self._active:
            print(
                f"[{self._task_name}] initialized; "
                "waiting for previous delivery task",
                flush=True,
            )

    def refresh_robot_frame(self):
        """Re-read the robot base world transform after navigation."""
        (
            arm_position,
            arm_orientation,
            self._arm_to_world,
        ) = prim_world_pose(self._arm_base)
        up_vec = self._arm_to_world.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0))
        self._up = np.array(up_vec, dtype=float)
        self._up = self._up / np.linalg.norm(self._up)
        if hasattr(self, "_controller") and self._controller is not None:
            self._controller.rmp_flow.set_robot_base_pose(
                robot_position=arm_position,
                robot_orientation=arm_orientation,
            )
        if hasattr(self, "_lift_ik") and self._lift_ik is not None:
            self._lift_ik.set_robot_base_pose(
                robot_position=arm_position,
                robot_orientation=arm_orientation,
            )

    def start_with_deployed_trays(self):
        """Activate after pizza delivery without retracting/redeploying trays."""
        if not self._initialized:
            raise RuntimeError("soda task must be initialized before activation")
        if self._active:
            return
        self._active = True
        self._trays_deployed = True
        # Pizza needs the original 40 N form-lock grip. Lower the force only
        # now, after the pizza task has released its handle.
        self._enable_soda_grip_force()
        # Refresh robot base world pose and axes after mobile navigation
        self.refresh_robot_frame()
        # Phase 0 interpolates from the pose left by pizza delivery, not from
        # the shared ready pose captured before the pizza task began.
        self._initial_wrist, self._initial_orientation, _ = prim_world_pose(
            self._end_effector
        )
        self._prepare_targets()
        self._enter_phase(0)
        print(
            f"[{self._task_name}] activated; reusing deployed trays",
            flush=True,
        )

    def _prepare_targets(self):
        can_prim = self._stage.GetPrimAtPath(self._pick_path)
        if not can_prim.IsValid():
            raise RuntimeError(f"missing soda can: {self._pick_path}")
        pick, _, _ = prim_world_pose(can_prim)
        pick = np.asarray(pick, dtype=float)
        self._can_prim = can_prim
        self._can_pick_start = pick.copy()
        above_pick = pick + self._up * CAN_APPROACH_HEIGHT
        lifted = pick + self._up * CAN_LIFT_HEIGHT
        if self._use_bail_detour:
            lifted[2] = max(lifted[2], SAFE_TRANSIT_TCP_WORLD_Z)
        side_y = OUTSIDE_BAIL_LOCAL_Y if self._place_left else -OUTSIDE_BAIL_LOCAL_Y
        place_local = np.array(
            [
                # Keep the can outside the pizza-board outline without using
                # the unreachable full-extension X=0.75 m target.
                TABLE_BOARD_CENTRE_LOCAL[0] + 0.12,
                TABLE_BOARD_CENTRE_LOCAL[1] + side_y,
                TABLE_BOARD_CENTRE_LOCAL[2]
                - 0.5 * 0.018
                + 0.5 * CAN_HEIGHT
                + 0.001,
            ]
        )
        place = self._local_to_world(place_local)
        above_place = place + self._up * TABLE_APPROACH_HEIGHT
        near_gate = self._local_to_world(
            np.array([BOARD_NEAR_GATE_LOCAL_X, side_y, 0.0])
        )
        near_gate[2] = SAFE_TRANSIT_TCP_WORLD_Z
        transit_target = near_gate if self._use_bail_detour else above_place
        self._targets = {
            0: above_pick,
            1: pick,
            3: lifted,
            4: transit_target,
            5: above_place,
            6: place,
            8: above_place,
        }
        self._vertical_starts = {
            1: above_pick,
            3: pick,
            6: above_place,
            8: place,
        }
        print(
            f"[{self._task_name}] selected={self._pick_path} "
            f"pick={np.round(pick, 4)} "
            + (
                f"near_gate={np.round(near_gate, 4)} "
                if self._use_bail_detour
                else "direct_route=1 "
            )
            + f"place={np.round(place, 4)}",
            flush=True,
        )

    def _enter_phase(self, phase):
        self._phase = phase
        self._phase_steps = 0
        self._settled_steps = 0
        names = [
            f"move above {self._side_label}-front {self._object_label}",
            f"descend vertically around {self._object_label}",
            "close gripper",
            f"lift {self._object_label} vertically",
            (
                "route around upright bail on robot side"
                if self._use_bail_detour
                else "move directly above right table destination"
            ),
            (
                "move through outside-bail corridor above destination"
                if self._use_bail_detour
                else "hold above right table destination"
            ),
            f"lower {self._object_label} vertically",
            "open gripper",
            "retreat vertically",
            "delivery complete",
        ]
        print(f"[{self._task_name}] phase={phase} {names[phase]}", flush=True)
        if phase in (2, 7):
            if phase == 2 and self._continuous_rmp_lift:
                # Preserve the exact final phase-1 command.  Reading the live
                # wrist here and turning it into a new target creates a small
                # policy discontinuity at the rear-tray grasp pose.
                self._grasp_hold_wrist = self._tcp_to_wrist(
                    self._targets[1]
                )
                self._grasp_hold_orientation = (
                    self._vertical_orientation.copy()
                )
            else:
                (
                    self._grasp_hold_wrist,
                    self._grasp_hold_orientation,
                    _,
                ) = prim_world_pose(self._end_effector)
        if phase == 9:
            self.done = True

    def _apply_seeded_ik_action(
        self, articulation, wrist_target, orientation, phase_label
    ):
        """Apply one continuous Lula IK sample from the measured joints."""
        current_arm = np.asarray(
            articulation.get_joint_positions()[self._arm_indices],
            dtype=float,
        )
        joints, ik_ok = self._seeded_ik_solver.compute_inverse_kinematics(
            "link_6",
            wrist_target,
            orientation,
            current_arm,
            0.003,
            0.03,
        )
        if not ik_ok:
            self.failed = True
            print(
                f"[{self._task_name}] STOPPED: seeded IK failed "
                f"phase={phase_label} wrist={np.round(wrist_target, 4)}",
                flush=True,
            )
            return False
        joints = current_arm + (
            np.asarray(joints, dtype=float) - current_arm + np.pi
        ) % (2.0 * np.pi) - np.pi
        joint_step = np.abs(joints - current_arm)
        if float(np.max(joint_step)) > self._seeded_ik_max_delta:
            self.failed = True
            print(
                f"[{self._task_name}] STOPPED: seeded IK branch jump "
                f"phase={phase_label} delta_deg="
                f"{np.round(np.degrees(joint_step), 1).tolist()}",
                flush=True,
            )
            return False
        articulation.apply_action(
            ArticulationAction(
                joint_positions=joints,
                joint_indices=self._arm_indices,
            )
        )
        return True

    def step(self, articulation):
        if not self._active or self.done or self.failed:
            return
        if not self._trays_deployed:
            self._deploy_steps += 1
            raw = min(1.0, self._deploy_steps / SLIDING_TRAY_DEPLOY_STEPS)
            amount = raw * raw * (3.0 - 2.0 * raw)
            target = np.full(2, SLIDING_TRAY_EXTENSION * amount)
            articulation.apply_action(
                ArticulationAction(
                    joint_positions=target, joint_indices=self._tray_indices
                )
            )
            actual = articulation.get_joint_positions()[self._tray_indices]
            error = float(np.max(np.abs(target - actual)))
            if self._deploy_steps % 60 == 0:
                print(
                    f"[{self._task_name}-tray] target={target[0]:.3f}m "
                    f"actual={np.round(actual, 3).tolist()}m",
                    flush=True,
                )
            if raw >= 1.0 and error < 0.005:
                self._trays_deployed = True
                self._prepare_targets()
                self._enter_phase(0)
            elif self._deploy_steps >= SLIDING_TRAY_DEPLOY_STEPS + 360:
                self.failed = True
                print(
                    f"[{self._task_name}] STOPPED: tray deployment failed",
                    flush=True,
                )
            return

        self._phase_steps += 1
        if self._phase in (2, 7):
            # Keep the arm controller active while the fingers move.  Most
            # payloads advance RMPFlow; branch-sensitive payloads hold the
            # same Cartesian pose with continuously seeded Lula IK.
            if self._use_seeded_ik_for_all_motion:
                if not self._apply_seeded_ik_action(
                    articulation,
                    self._grasp_hold_wrist,
                    self._grasp_hold_orientation,
                    self._phase,
                ):
                    return
            else:
                hold_action = self._controller.forward(
                    target_end_effector_position=self._grasp_hold_wrist,
                    target_end_effector_orientation=self._grasp_hold_orientation,
                )
                articulation.apply_action(hold_action)
            if self._phase == 2:
                raw = min(
                    1.0,
                    self._phase_steps / float(self._gripper_close_ramp_steps),
                )
                amount = raw * raw * (3.0 - 2.0 * raw)
                target = self._gripper_pregrasp + (
                    self._gripper_close - self._gripper_pregrasp
                ) * amount
            else:
                target = GRIPPER_OPEN
            self._command_gripper(articulation, target)
            wait = (
                self._gripper_close_wait_steps
                if self._phase == 2
                else 90
            )
            if self._phase_steps % 30 == 0:
                actual = articulation.get_joint_positions()[self._gripper_index]
                contact_status = ""
                if self._phase == 2 and self._grasp_hold_max_error is not None:
                    wrist_now, _, _ = prim_world_pose(self._end_effector)
                    wrist_error = float(
                        np.linalg.norm(wrist_now - self._grasp_hold_wrist)
                    )
                    object_now, _, _ = prim_world_pose(self._can_prim)
                    object_delta = np.asarray(object_now, dtype=float) - self._can_pick_start
                    lateral_delta = float(np.linalg.norm(object_delta[:2]))
                    contact_status = (
                        f" wrist_error={wrist_error:.4f}m "
                        f"object_lateral={lateral_delta:.4f}m"
                    )
                print(
                    f"[{self._task_name}-gripper] target={target:.3f} "
                    f"actual={actual:.3f}{contact_status}",
                    flush=True,
                )
            if self._phase_steps >= wait:
                if (
                    self._phase == 2
                    and self._gripper_contact_margin is not None
                ):
                    actual_gripper = float(
                        articulation.get_joint_positions()[self._gripper_index]
                    )
                    if actual_gripper >= (
                        self._gripper_close - self._gripper_contact_margin
                    ):
                        self.failed = True
                        print(
                            f"[{self._task_name}] STOPPED: empty grasp; "
                            f"gripper reached close target "
                            f"target={self._gripper_close:.3f} "
                            f"actual={actual_gripper:.3f}",
                            flush=True,
                        )
                        return
                if self._phase == 2 and self._grasp_hold_max_error is not None:
                    wrist_now, _, _ = prim_world_pose(self._end_effector)
                    wrist_error = float(
                        np.linalg.norm(wrist_now - self._grasp_hold_wrist)
                    )
                    object_now, _, _ = prim_world_pose(self._can_prim)
                    object_delta = np.asarray(object_now, dtype=float) - self._can_pick_start
                    lateral_delta = float(np.linalg.norm(object_delta[:2]))
                    if (
                        wrist_error > self._grasp_hold_max_error
                        or lateral_delta > self._grasp_object_max_lateral
                    ):
                        self.failed = True
                        print(
                            f"[{self._task_name}] STOPPED: grasp contact "
                            f"disturbed arm/object wrist_error={wrist_error:.4f}m "
                            f"object_lateral={lateral_delta:.4f}m",
                            flush=True,
                        )
                        return
                if self._phase == 2 and self._preserve_grasp_orientation:
                    self._carried_orientation = np.asarray(
                        self._grasp_hold_orientation, dtype=float
                    ).copy()
                    print(
                        f"[{self._task_name}] carrying orientation locked "
                        f"quaternion={np.round(self._carried_orientation, 4)}",
                        flush=True,
                    )
                if self._phase == 2 and self._lock_cspace_after_grasp:
                    grasp_branch = np.asarray(
                        articulation.get_joint_positions()[self._arm_indices],
                        dtype=float,
                    ).copy()
                    self._controller.rmp_flow.set_cspace_target(grasp_branch)
                    print(
                        f"[{self._task_name}] lift branch locked "
                        f"J2={np.degrees(grasp_branch[1]):.1f}deg "
                        f"J3={np.degrees(grasp_branch[2]):.1f}deg",
                        flush=True,
                    )
                self._enter_phase(self._phase + 1)
            return

        tcp_target = self._targets[self._phase].copy()
        transition_complete = True
        if self._phase in self._vertical_starts:
            raw = min(1.0, self._phase_steps / float(self._vertical_steps))
            amount = raw * raw * (3.0 - 2.0 * raw)
            start = self._vertical_starts[self._phase]
            tcp_target[:2] = start[:2]
            tcp_target[2] = start[2] + amount * (tcp_target[2] - start[2])
            transition_complete = raw >= 1.0
        elif self._phase in (4, 5) and self._use_bail_detour:
            raw = min(1.0, self._phase_steps / float(DETOUR_STEPS))
            amount = raw * raw * (3.0 - 2.0 * raw)
            start = self._targets[3] if self._phase == 4 else self._targets[4]
            end = self._targets[self._phase]
            tcp_target = start + amount * (end - start)
            transition_complete = raw >= 1.0
        elif self._phase == 4 and self._use_seeded_ik_for_all_motion:
            # RMPFlow normally turns a distant phase-4 target into a smooth
            # motion internally.  A joint-by-joint seeded IK controller needs
            # that Cartesian interpolation made explicit to preserve branch
            # continuity during the long pickup-to-table transfer.
            raw = min(1.0, self._phase_steps / float(self._transfer_steps))
            amount = raw * raw * (3.0 - 2.0 * raw)
            tcp_target = self._targets[3] + amount * (
                self._targets[4] - self._targets[3]
            )
            transition_complete = raw >= 1.0
        wrist_target = self._tcp_to_wrist(tcp_target)
        orientation = self._vertical_orientation
        if (
            self._preserve_grasp_orientation
            and self._carried_orientation is not None
            and self._phase >= 3
        ):
            # At rear-tray cutlery poses, switching back to the canonical
            # downward quaternion selects a disconnected IK branch.  Keep the
            # actual grasp orientation so phase 2 -> 3 changes translation
            # only and cannot whip J3 across the robot.
            orientation = self._carried_orientation
        if self._phase == 0:
            raw = min(1.0, self._phase_steps / float(self._initial_steps))
            amount = raw * raw * (3.0 - 2.0 * raw)
            wrist_target = self._initial_wrist + amount * (
                wrist_target - self._initial_wrist
            )
            orientation = _quaternion_slerp(
                self._initial_orientation, self._vertical_orientation, amount
            )
            transition_complete = raw >= 1.0
        if self._use_seeded_ik_for_all_motion:
            if not self._apply_seeded_ik_action(
                articulation,
                wrist_target,
                orientation,
                self._phase,
            ):
                return
        elif self._phase == 3 and self._use_seeded_ik_for_lift:
            # RMPFlow can leave the rear-tray IK branch even for a few mm of
            # vertical motion.  Solve each lift sample from the *current*
            # joints instead, making the local branch explicit and continuous.
            current_arm = np.asarray(
                articulation.get_joint_positions()[self._arm_indices],
                dtype=float,
            )
            lift_joints, ik_ok = self._lift_ik.compute_inverse_kinematics(
                "link_6",
                wrist_target,
                orientation,
                current_arm,
                0.003,
                0.03,
            )
            if not ik_ok:
                self.failed = True
                print(
                    f"[{self._task_name}] STOPPED: seeded lift IK failed "
                    f"tcp={np.round(tcp_target, 4)}",
                    flush=True,
                )
                return
            # Lula may express an equivalent revolute solution one full turn
            # away.  Unwrap it around the live joints and reject any remaining
            # discontinuity before it reaches the articulation drives.
            lift_joints = current_arm + (
                np.asarray(lift_joints, dtype=float)
                - current_arm
                + np.pi
            ) % (2.0 * np.pi) - np.pi
            joint_step = np.abs(lift_joints - current_arm)
            if float(np.max(joint_step)) > np.deg2rad(10.0):
                self.failed = True
                print(
                    f"[{self._task_name}] STOPPED: seeded lift IK branch "
                    f"jump rejected delta_deg="
                    f"{np.round(np.degrees(joint_step), 1)}",
                    flush=True,
                )
                return
            articulation.apply_action(
                ArticulationAction(
                    joint_positions=np.asarray(lift_joints, dtype=float),
                    joint_indices=self._arm_indices,
                )
            )
        else:
            action = self._controller.forward(
                target_end_effector_position=wrist_target,
                target_end_effector_orientation=orientation,
            )
            articulation.apply_action(action)
        actual, _, _ = prim_world_pose(self._end_effector)
        error = float(np.linalg.norm(wrist_target - actual))
        settle_error = (
            self._placement_settle_error
            if self._phase == 6 and self._placement_settle_error is not None
            else 0.025
        )
        self._settled_steps = (
            self._settled_steps + 1
            if transition_complete and error < settle_error
            else 0
        )
        if self._phase_steps % 60 == 0:
            arm = articulation.get_joint_positions()[self._arm_indices]
            print(
                f"[{self._task_name}-motion] phase={self._phase} "
                f"tcp={np.round(tcp_target, 4)} error={error:.4f}m "
                f"J2={np.degrees(arm[1]):.1f}deg "
                f"J3={np.degrees(arm[2]):.1f}deg",
                flush=True,
            )
        if self._settled_steps >= 15:
            if self._phase == 3:
                can_position, _, _ = prim_world_pose(self._can_prim)
                can_lift = float(can_position[2] - self._can_pick_start[2])
                if can_lift < self._minimum_verified_lift:
                    self.failed = True
                    print(
                        f"[{self._task_name}] STOPPED: "
                        f"gripper rose without {self._object_label}; "
                        f"can_lift={can_lift:.4f}m",
                        flush=True,
                    )
                    return
                print(
                    f"[{self._task_name}] grasp verified "
                    f"can_lift={can_lift:.4f}m",
                    flush=True,
                )
                if self._use_seeded_ik_for_lift:
                    # Synchronize RMPFlow with the branch reached by local IK
                    # before resuming it for the long table transfer.
                    reached_branch = np.asarray(
                        articulation.get_joint_positions()[self._arm_indices],
                        dtype=float,
                    ).copy()
                    self._controller.reset()
                    self._controller.rmp_flow.set_cspace_target(reached_branch)
                    print(
                        f"[{self._task_name}] seeded vertical lift complete; "
                        "RMPFlow synchronized for transfer",
                        flush=True,
                    )
            self._enter_phase(self._phase + 1)
        elif self._phase_steps >= 900:
            self.failed = True
            print(
                f"[{self._task_name}] STOPPED: phase={self._phase} "
                f"position_error={error:.4f}m",
                flush=True,
            )

    def close(self):
        # RMPFlowController and Lula solvers retain articulation/tensor views.
        # They must be released before the next trip replaces payload prims.
        self._controller = None
        self._lift_ik = None
        self._gripper = None
        self._can_prim = None


class Soda1PickPlace(SodaCanPickPlace):
    """Robot-front/left can to the left side of the delivered pizza."""

    def __init__(self, stage, wait_for_start=False):
        super().__init__(
            stage,
            wait_for_start=wait_for_start,
            task_name="soda1",
            pick_path=SODA1_PICK_PATH,
            place_left=True,
        )


class Soda2PickPlace(SodaCanPickPlace):
    """Robot-front/right can to the right side of the delivered pizza."""

    def __init__(self, stage, wait_for_start=False):
        super().__init__(
            stage,
            wait_for_start=wait_for_start,
            task_name="soda2",
            pick_path=SODA2_PICK_PATH,
            place_left=False,
        )
