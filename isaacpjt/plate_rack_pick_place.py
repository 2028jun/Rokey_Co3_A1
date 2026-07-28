"""Top-down delivery of the plate rack to the table's left side."""

import os

import numpy as np
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot_motion.motion_generation.lula.kinematics import (
    LulaKinematicsSolver,
)
from pxr import UsdPhysics

from isaac_scene_utils import prim_world_pose
from plate_rack_serving import (
    HANDLE_GRIP_CENTRE_Z,
    PLATE_RACK_PATH,
    RACK_BASE_SIZE,
    follow_plate_rack_transport,
)
from soda_pick_place import (
    M0609_DESCRIPTION,
    M0609_RMPFLOW_URDF,
    TABLE_BOARD_CENTRE_LOCAL,
    VERTICAL_EE_ORIENTATION,
    SodaCanPickPlace,
)


PLATE_RACK_APPROACH_HEIGHT = 0.28
PLATE_RACK_LIFT_HEIGHT = 0.22
PLATE_RACK_TABLE_APPROACH_HEIGHT = 0.14
# Use the proven pizza-handle closure.  At 0.55 rad the RG2 drive reached its
# target (actual=0.549) without touching the 50 mm rack post, so the arm lifted
# with visibly open fingers.  The same RG2 already carries the pizza handle at
# 1.06 rad and 40 N; the rack flange provides the corresponding form lock.
PLATE_RACK_GRIPPER_CLOSE = 1.06
PLATE_RACK_GRIPPER_MAX_FORCE = 40.0
# Enter the narrow gap between the vertical plates partially closed.  At 0 rad
# the RG2 fingers extend into both adjacent plates before reaching the handle,
# preventing any closing motion.  The formerly tested 0.55 rad pose clears the
# rack while remaining wider than the 26 mm centre grip block.
PLATE_RACK_GRIPPER_PREGRASP = 0.55
# Keep the 300 x 200 mm rack in the table-left area, but pull it 90 mm toward
# the robot relative to the old X=0.49 target.  The old base reached X=0.64,
# only 30 mm from soda1 at (0.67, +0.28); its 33 mm collider therefore touched
# the rack during descent.  X=0.40 leaves 87 mm between the rack base and can,
# while retaining 45 mm clearance from the pizza board at (0.55, 0.0).
PLATE_RACK_PLACE_LOCAL_X = 0.40
PLATE_RACK_PLACE_LOCAL_Y = 0.34
# Command the base slightly into the theoretical tabletop plane.  Contact
# supports the rack before release instead of allowing the shared controller
# tolerance to open the gripper while the payload is still visibly airborne.
PLATE_RACK_PLACE_SURFACE_BIAS = -0.002
PIZZA_BOARD_THICKNESS = 0.018
# The rack rides the robot-LEFT upper tray, i.e. +Y in the arm base frame.
# Joint 1 must therefore point the tool to +Y.  The shared soda/cutlery ready
# posture uses J1=+90 deg, which aims at the robot-RIGHT tray; seeding IK with
# it made Lula return the far branch (J3/J5 folded by about 170 deg) and the
# pre-approach branch guard rejected every solution.  J1=-84 deg aims straight
# at the deployed left tray and keeps the remaining joints on the same
# elbow-up branch, so only the base rotation is mirrored.
PLATE_RACK_READY_JOINTS = np.deg2rad(
    [
        float(os.environ.get("MOBILE_PLATE_READY_J1_DEG", "-84.0")),
        float(os.environ.get("MOBILE_PLATE_READY_J2_DEG", "-20.0")),
        float(os.environ.get("MOBILE_PLATE_READY_J3_DEG", "-94.0")),
        float(os.environ.get("MOBILE_PLATE_READY_J4_DEG", "0.0")),
        float(os.environ.get("MOBILE_PLATE_READY_J5_DEG", "-66.0")),
        float(os.environ.get("MOBILE_PLATE_READY_J6_DEG", "90.0")),
    ]
)
PLATE_RACK_STOW_JOINTS = np.deg2rad([90.0, 0.0, -90.0, 0.0, -60.0, 90.0])
# Revolute limits of joint_1..joint_6 from ridgeback_m0609.urdf.  Wrapping a
# target to the nearest equivalent angle is not enough on its own: after a
# delivery the arm can sit at J1=-270 deg (the stow controller's shortest-path
# equivalent of +90 deg), and the nearest equivalent of the ready posture from
# there is -444 deg, which no drive can reach.  Targets are therefore chosen
# among in-limit equivalents only.
PLATE_RACK_JOINT_LOWER = np.array(
    [-6.2832, -6.2832, -2.618, -6.2832, -6.2832, -6.2832]
)
PLATE_RACK_JOINT_UPPER = np.array(
    [6.2832, 6.2832, 2.618, 6.2832, 6.2832, 6.2832]
)
PLATE_RACK_READY_TOLERANCE = np.deg2rad(3.0)
PLATE_RACK_READY_SETTLE_STEPS = 15
# Hard command-lead guard used while following planned trajectories.
PLATE_RACK_MAX_JOINT_STEP = np.deg2rad(
    float(os.environ.get("MOBILE_PLATE_MAX_JOINT_STEP_DEG", "6.0"))
)
# Joint-space preparation uses a quintic time law with zero velocity and zero
# acceleration at both ends.  These limits are intentionally below the M0609
# URDF limits (J1/J2: 150 deg/s) so the 174 degree left-tray rotation looks
# like a physical arm motion instead of an instantaneous position step.
PLATE_RACK_READY_MAX_SPEED = np.deg2rad(
    float(os.environ.get("MOBILE_PLATE_READY_MAX_SPEED_DEG_S", "55.0"))
)
PLATE_RACK_READY_MAX_ACCEL = np.deg2rad(
    float(os.environ.get("MOBILE_PLATE_READY_MAX_ACCEL_DEG_S2", "80.0"))
)
PLATE_RACK_CONTROL_HZ = float(
    os.environ.get("MOBILE_PLATE_CONTROL_HZ", "60.0")
)
PLATE_RACK_MAX_IK_DELTA = np.deg2rad(70.0)
PLATE_RACK_PREAPPROACH_STEPS = 180
PLATE_RACK_RELEASE_SETTLE_STEPS = 30


class PlateRackPickPlace(SodaCanPickPlace):
    """Pick the rack's centre post vertically and place it table-left."""

    def __init__(
        self, stage, wait_for_start=False, *, payload_root="/World",
        robot_root=None
    ):
        self._payload_root = str(payload_root).rstrip("/") or "/World"
        self._rack_path = f"{self._payload_root}/ServingPlateRack"
        self._robot_root = robot_root
        super().__init__(
            stage,
            wait_for_start=wait_for_start,
            task_name="plate_rack",
            pick_path=self._rack_path,
            place_left=False,
            robot_root=robot_root,
        )
        self._use_bail_detour = False
        # Preserve the plate-rack tool pose proven on the yaw=pi left-table
        # robot.  SodaCanPickPlace rotates this canonical world quaternion by
        # the current robot yaw, giving the yaw=0 right-table robot the same
        # local wrist pose and IK branch.
        self._canonical_vertical_orientation = np.array(
            [0.0, 0.0, 1.0, 0.0], dtype=float
        )
        self._vertical_orientation = (
            self._canonical_vertical_orientation.copy()
        )
        self._gripper_close = PLATE_RACK_GRIPPER_CLOSE
        self._gripper_max_force = PLATE_RACK_GRIPPER_MAX_FORCE
        self._gripper_pregrasp = PLATE_RACK_GRIPPER_PREGRASP
        self._object_label = "plate rack"
        self._minimum_verified_lift = 0.06
        self._grasp_hold_max_error = 0.035
        self._grasp_object_max_lateral = 0.025
        # Once the rack base bears on the table, the physical contact prevents
        # the commanded wrist pose from reaching the slightly penetrating
        # target.  Integrated runs settle at about 20 mm error, so allow 22 mm
        # here.  This remains tighter than the shared 25 mm threshold, while
        # the -2 mm surface bias ensures release happens with the base supported.
        self._placement_settle_error = 0.022
        # Phase 6 may stop on the table with a small residual wrist error.
        # Verify the rack base itself is at the authored support surface before
        # releasing, rather than timing out or accepting an airborne wrist.
        self._placement_requires_support = True
        self._placement_support_vertical_min = -0.003
        self._placement_support_vertical_max = 0.012
        self._placement_support_lateral = 0.025
        self._placement_support_gap = float("inf")
        self._placement_support_lateral_error = float("inf")
        self._gripper_contact_margin = 0.03
        self._gripper_close_ramp_steps = 120
        self._gripper_close_wait_steps = 165
        # The rack is a rigid handled post on a stable tray, so the long
        # soda-can ramps are not needed.  Halving them keeps the same smooth
        # profile at roughly twice the speed.
        self._vertical_steps = int(
            os.environ.get("MOBILE_PLATE_VERTICAL_STEPS", "150")
        )
        self._initial_steps = int(
            os.environ.get("MOBILE_PLATE_INITIAL_STEPS", "120")
        )
        self._motion_timeout_steps = int(
            os.environ.get("MOBILE_PLATE_MOTION_TIMEOUT_STEPS", "600")
        )
        self._continuous_rmp_lift = True
        self._preserve_grasp_orientation = False
        self._use_seeded_ik_for_lift = False
        self._use_seeded_ik_for_all_motion = True
        self._seeded_ik_max_delta = np.deg2rad(15.0)
        self._transfer_steps = int(
            os.environ.get("MOBILE_PLATE_TRANSFER_STEPS", "180")
        )
        self._lock_cspace_after_grasp = False
        self._ready_pending = True
        self._ready_stage = "stow"
        self._ready_step = 0
        self._ready_settle = 0
        self._ready_plan_start = None
        self._ready_plan_target = None
        self._ready_plan_frames = 0
        self._preapproach_pending = True
        self._preapproach_start = None
        self._preapproach_target = None
        self._preapproach_step = 0
        self._plate_ik = None
        self._transport_released = False
        self._transport_finalized = False
        self._transport_release_steps = 0

    def _follow_transport_tray(self):
        """Place the independent kinematic rack at its tray-local pose."""
        if not follow_plate_rack_transport(
            self._stage,
            payload_root=self._payload_root,
            robot_root=self._robot_root,
        ):
            self.failed = True
            print(
                "[plate-rack] STOPPED: non-finite tray transport pose",
                flush=True,
            )
            return False
        return True

    def _placement_supported(self):
        """Confirm that the rack base, not just the wrist, reached the table."""
        if self._phase != 6 or self._can_prim is None:
            return False
        rack_position, _, _ = prim_world_pose(self._can_prim)
        rack_position = np.asarray(rack_position, dtype=float)
        if not np.all(np.isfinite(rack_position)):
            return False

        # Phase-6 target is the handle centre. Recover the corresponding rack
        # root target and measure along the robot's live up axis. The target is
        # intentionally 2 mm below the ideal contact plane, so a supported base
        # normally reports a small positive gap here.
        rack_target = (
            np.asarray(self._targets[6], dtype=float)
            - self._up * HANDLE_GRIP_CENTRE_Z
        )
        delta = rack_position - rack_target
        vertical_gap = float(np.dot(delta, self._up))
        lateral_delta = delta - self._up * vertical_gap
        lateral_error = float(np.linalg.norm(lateral_delta))
        self._placement_support_gap = vertical_gap
        self._placement_support_lateral_error = lateral_error
        return bool(
            self._placement_support_vertical_min
            <= vertical_gap
            <= self._placement_support_vertical_max
            and lateral_error <= self._placement_support_lateral
        )

    def _release_transport(self):
        """Turn the independently cooked rack from kinematic to dynamic."""
        rack = self._stage.GetPrimAtPath(self._rack_path)
        body = UsdPhysics.RigidBodyAPI.Get(self._stage, rack.GetPath())
        if not body:
            raise RuntimeError("plate rack rigid body API is missing")
        body.CreateKinematicEnabledAttr().Set(False)
        restored_colliders = 0
        for prim in self._stage.Traverse():
            if not str(prim.GetPath()).startswith(f"{self._rack_path}/"):
                continue
            marker = prim.GetAttribute("plateRack:transportCollider")
            if not marker or not marker.Get():
                continue
            collision = UsdPhysics.CollisionAPI.Get(
                self._stage, prim.GetPath()
            )
            if collision:
                collision.GetCollisionEnabledAttr().Set(True)
                restored_colliders += 1
        handle = self._stage.GetPrimAtPath(f"{self._rack_path}/Handle")
        collision = UsdPhysics.CollisionAPI.Get(self._stage, handle.GetPath())
        collision_enabled = bool(
            collision
            and collision.GetCollisionEnabledAttr().Get()
        )
        if not collision_enabled:
            raise RuntimeError("plate rack handle collision is not enabled")
        print(
            "[plate-rack] independent kinematic transport released; "
            f"handle_collision=1 colliders={restored_colliders} "
            "settling before pickup",
            flush=True,
        )

    def initialize(self, articulation, dof_names):
        """Initialize the shared controller and bias it to the safe branch."""
        super().initialize(articulation, dof_names)
        arm_position, arm_orientation, _ = prim_world_pose(self._arm_base)
        self._plate_ik = LulaKinematicsSolver(
            str(M0609_DESCRIPTION), str(M0609_RMPFLOW_URDF)
        )
        self._plate_ik.set_robot_base_pose(
            robot_position=arm_position,
            robot_orientation=arm_orientation,
        )
        self._seeded_ik_solver = self._plate_ik
        self._controller.rmp_flow.set_cspace_target(
            PLATE_RACK_READY_JOINTS.copy()
        )
        print(
            "[plate-rack-ready] configured joint-space preparation "
            f"target_deg={np.round(np.degrees(PLATE_RACK_READY_JOINTS), 1).tolist()}",
            flush=True,
        )

    def refresh_robot_frame(self):
        """Refresh both RMPFlow and the plate-rack-specific IK frame."""
        super().refresh_robot_frame()
        if self._plate_ik is not None:
            arm_position, arm_orientation, _ = prim_world_pose(self._arm_base)
            self._plate_ik.set_robot_base_pose(
                robot_position=arm_position,
                robot_orientation=arm_orientation,
            )

    def _start_ready_plan(self, current, desired):
        """Create a velocity/acceleration-limited quintic joint trajectory."""
        displacement = self._joint_error(current, desired)
        self._ready_plan_start = current.copy()
        self._ready_plan_target = current + displacement
        distance = float(np.max(np.abs(displacement)))
        # For s(u)=10u^3-15u^4+6u^5, max(ds/du)=1.875 and
        # max(abs(d2s/du2))=5.7735.  Choose duration satisfying both limits.
        speed_time = 1.875 * distance / PLATE_RACK_READY_MAX_SPEED
        accel_time = np.sqrt(
            5.7735 * distance / PLATE_RACK_READY_MAX_ACCEL
        )
        duration = max(speed_time, accel_time, 0.25)
        self._ready_plan_frames = max(
            1, int(np.ceil(duration * PLATE_RACK_CONTROL_HZ))
        )
        self._ready_step = 0
        print(
            f"[plate-rack-ready] stage={self._ready_stage} planned "
            f"duration={duration:.2f}s max_speed="
            f"{np.degrees(PLATE_RACK_READY_MAX_SPEED):.1f}deg/s "
            f"max_accel={np.degrees(PLATE_RACK_READY_MAX_ACCEL):.1f}deg/s2",
            flush=True,
        )

    @staticmethod
    def _bounded_joint_target(current, desired):
        """Return the nearest revolute target with a hard per-frame bound."""
        error = PlateRackPickPlace._joint_error(current, desired)
        target = current + np.clip(
            error, -PLATE_RACK_MAX_JOINT_STEP, PLATE_RACK_MAX_JOINT_STEP
        )
        return target, error

    @staticmethod
    def _joint_error(current, desired):
        """Signed error to the closest in-limit equivalent of ``desired``.

        Revolute joints accept ``desired + 2*pi*k``.  The shortest path is
        preferred, but any candidate outside the URDF limits is discarded so
        the arm never chases an unreachable wrap after a previous delivery
        left joint 1 a full turn away.
        """
        current = np.asarray(current, dtype=float)
        desired = np.asarray(desired, dtype=float)
        shortest = current + np.arctan2(
            np.sin(desired - current), np.cos(desired - current)
        )
        turn = 2.0 * np.pi
        best = shortest.copy()
        in_limits = (shortest >= PLATE_RACK_JOINT_LOWER) & (
            shortest <= PLATE_RACK_JOINT_UPPER
        )
        for index in np.flatnonzero(~in_limits):
            candidates = shortest[index] + turn * np.arange(-2, 3)
            allowed = candidates[
                (candidates >= PLATE_RACK_JOINT_LOWER[index])
                & (candidates <= PLATE_RACK_JOINT_UPPER[index])
            ]
            if allowed.size:
                best[index] = allowed[
                    np.argmin(np.abs(allowed - current[index]))
                ]
        return best - current

    def _step_ready_pose(self, articulation):
        """Recover through stow, then enter the left-tray elbow-up posture."""
        current = np.asarray(
            articulation.get_joint_positions()[self._arm_indices],
            dtype=float,
        )
        if not np.all(np.isfinite(current)):
            self.failed = True
            print("[plate-rack-ready] STOPPED: non-finite arm joints", flush=True)
            return

        target_pose = (
            PLATE_RACK_STOW_JOINTS
            if self._ready_stage == "stow"
            else PLATE_RACK_READY_JOINTS
        )
        if self._ready_plan_start is None:
            self._start_ready_plan(current, target_pose)

        self._ready_step += 1
        raw = min(1.0, self._ready_step / float(self._ready_plan_frames))
        # Quintic smoothstep: position, velocity, and acceleration are all
        # continuous, with zero velocity/acceleration at both endpoints.
        amount = raw**3 * (10.0 + raw * (-15.0 + 6.0 * raw))
        planned_target = self._ready_plan_start + amount * (
            self._ready_plan_target - self._ready_plan_start
        )
        # Keep a final measured-pose guard so a stalled physics step can never
        # produce a large articulation-drive target jump.
        target, _ = self._bounded_joint_target(current, planned_target)
        articulation.apply_action(
            ArticulationAction(
                joint_positions=target,
                joint_indices=self._arm_indices,
            )
        )
        self._command_gripper(articulation, self._gripper_pregrasp)

        final_error = self._joint_error(current, self._ready_plan_target)
        max_error = float(np.max(np.abs(final_error)))
        if raw >= 1.0 and max_error <= PLATE_RACK_READY_TOLERANCE:
            self._ready_settle += 1
        else:
            self._ready_settle = 0

        if self._ready_step % 60 == 0:
            print(
                f"[plate-rack-ready] stage={self._ready_stage} "
                f"actual_deg={np.round(np.degrees(current), 1).tolist()} "
                f"progress={100.0 * raw:.0f}% "
                f"max_error={np.degrees(max_error):.1f}deg",
                flush=True,
            )

        if self._ready_settle >= PLATE_RACK_READY_SETTLE_STEPS:
            if self._ready_stage == "stow":
                self._ready_stage = "left_ready"
                self._ready_step = 0
                self._ready_settle = 0
                self._ready_plan_start = None
                self._ready_plan_target = None
                self._ready_plan_frames = 0
                print(
                    "[plate-rack-ready] stow recovered; moving to left-tray ready posture",
                    flush=True,
                )
                return
            self._ready_pending = False
            self._controller.reset()
            self._controller.rmp_flow.set_cspace_target(
                PLATE_RACK_READY_JOINTS.copy()
            )
            self._initial_wrist, self._initial_orientation, _ = prim_world_pose(
                self._end_effector
            )
            # Re-read the live rack pose after tray extension and preparation.
            self._prepare_targets()
            print(
                "[plate-rack-ready] preparation complete; solving seeded pre-approach",
                flush=True,
            )
        elif self._ready_step >= self._ready_plan_frames + 600:
            self.failed = True
            print(
                "[plate-rack-ready] STOPPED: preparation posture did not converge "
                f"max_error={np.degrees(max_error):.1f}deg",
                flush=True,
            )

    def _step_seeded_preapproach(self, articulation):
        """Reach the high pickup waypoint in joint space on one IK branch."""
        current = np.asarray(
            articulation.get_joint_positions()[self._arm_indices], dtype=float
        )
        if not np.all(np.isfinite(current)):
            self.failed = True
            print("[plate-rack-preapproach] STOPPED: non-finite joints", flush=True)
            return

        if self._preapproach_target is None:
            wrist_target = self._tcp_to_wrist(self._targets[0])
            solution, ok = self._plate_ik.compute_inverse_kinematics(
                "link_6",
                wrist_target,
                self._vertical_orientation,
                current,
                0.003,
                0.03,
            )
            if not ok:
                self.failed = True
                print(
                    "[plate-rack-preapproach] STOPPED: seeded IK failed",
                    flush=True,
                )
                return
            solution = np.asarray(solution, dtype=float)
            if solution.shape != current.shape or not np.all(np.isfinite(solution)):
                self.failed = True
                print(
                    "[plate-rack-preapproach] STOPPED: invalid seeded IK solution",
                    flush=True,
                )
                return
            solution = current + self._joint_error(current, solution)
            delta = np.abs(solution - current)
            if float(np.max(delta)) > PLATE_RACK_MAX_IK_DELTA:
                self.failed = True
                print(
                    "[plate-rack-preapproach] STOPPED: IK branch jump rejected "
                    f"delta_deg={np.round(np.degrees(delta), 1).tolist()}",
                    flush=True,
                )
                return
            self._preapproach_start = current.copy()
            self._preapproach_target = solution
            self._preapproach_step = 0
            print(
                "[plate-rack-preapproach] seeded IK accepted "
                f"target_deg={np.round(np.degrees(solution), 1).tolist()}",
                flush=True,
            )

        self._preapproach_step += 1
        raw = min(
            1.0, self._preapproach_step / float(PLATE_RACK_PREAPPROACH_STEPS)
        )
        amount = raw * raw * (3.0 - 2.0 * raw)
        planned_target = self._preapproach_start + amount * (
            self._preapproach_target - self._preapproach_start
        )
        # Follow the smooth plan, but always derive the command from measured
        # joints.  If physics falls behind, the drive target still cannot jump.
        target, _ = self._bounded_joint_target(current, planned_target)
        articulation.apply_action(
            ArticulationAction(
                joint_positions=target, joint_indices=self._arm_indices
            )
        )
        self._command_gripper(articulation, self._gripper_pregrasp)

        error = np.arctan2(
            np.sin(self._preapproach_target - current),
            np.cos(self._preapproach_target - current),
        )
        max_error = float(np.max(np.abs(error)))
        if raw >= 1.0 and max_error <= PLATE_RACK_READY_TOLERANCE:
            self._ready_settle += 1
        else:
            self._ready_settle = 0
        if self._ready_settle >= PLATE_RACK_READY_SETTLE_STEPS:
            self._preapproach_pending = False
            self._controller.reset()
            self._controller.rmp_flow.set_cspace_target(
                self._preapproach_target.copy()
            )
            self._initial_wrist, self._initial_orientation, _ = prim_world_pose(
                self._end_effector
            )
            # Phase zero is already complete; begin only the vertical descent.
            self._enter_phase(1)
            print(
                "[plate-rack-preapproach] complete; starting vertical descent",
                flush=True,
            )
        elif self._preapproach_step >= PLATE_RACK_PREAPPROACH_STEPS + 600:
            self.failed = True
            print(
                "[plate-rack-preapproach] STOPPED: target did not converge",
                flush=True,
            )

    def step(self, articulation):
        if not self._active or self.done or self.failed:
            return
        if not self._trays_deployed:
            # The rack is kinematic and every one of its colliders is disabled
            # during transport, so it can visibly follow the extending tray
            # without injecting contact forces into the robot articulation.
            # The earlier PhysX blow-up came from doing this while rack
            # collision was enabled, not from the visual/kinematic follow.
            if not self._follow_transport_tray():
                return
            super().step(articulation)
            if not self.failed:
                self._follow_transport_tray()
            return
        if not self._transport_finalized:
            if not self._follow_transport_tray():
                return
            self._transport_finalized = True
            print(
                "[plate-rack] collisionless tray-follow complete at final pose",
                flush=True,
            )
            return
        if not self._transport_released:
            self._release_transport()
            self._transport_released = True
            self._transport_release_steps = 0
            return
        if self._transport_release_steps < PLATE_RACK_RELEASE_SETTLE_STEPS:
            self._transport_release_steps += 1
            self._command_gripper(articulation, self._gripper_pregrasp)
            if self._transport_release_steps == PLATE_RACK_RELEASE_SETTLE_STEPS:
                # Always sample the released, settled body again before
                # planning any arm motion. Refresh the fixed robot frame as
                # well: unlike a later sequence task, a plate-only first task
                # does not pass through start_with_deployed_trays().
                self.refresh_robot_frame()
                self._prepare_targets()
                print(
                    "[plate-rack] transport release settled; robot frame and "
                    "live pickup pose refreshed",
                    flush=True,
                )
            return
        if self._ready_pending:
            self._step_ready_pose(articulation)
            return
        if self._preapproach_pending:
            self._step_seeded_preapproach(articulation)
            return
        super().step(articulation)

    def close(self):
        self._seeded_ik_solver = None
        self._plate_ik = None
        super().close()

    def _prepare_targets(self):
        rack_prim = self._stage.GetPrimAtPath(self._pick_path)
        if not rack_prim.IsValid():
            raise RuntimeError(f"missing plate rack: {self._pick_path}")
        rack, _, _ = prim_world_pose(rack_prim)
        rack = np.asarray(rack, dtype=float)
        self._can_prim = rack_prim
        self._can_pick_start = rack.copy()

        grasp = rack + self._up * HANDLE_GRIP_CENTRE_Z
        above_pick = grasp + self._up * PLATE_RACK_APPROACH_HEIGHT
        lifted = grasp + self._up * PLATE_RACK_LIFT_HEIGHT
        table_surface_local_z = (
            TABLE_BOARD_CENTRE_LOCAL[2] - 0.5 * PIZZA_BOARD_THICKNESS
        )
        place_local = np.array(
            [
                PLATE_RACK_PLACE_LOCAL_X,
                PLATE_RACK_PLACE_LOCAL_Y,
                table_surface_local_z
                + 0.5 * RACK_BASE_SIZE[2]
                + PLATE_RACK_PLACE_SURFACE_BIAS,
            ]
        )
        place = self._local_to_world(place_local)
        place_grasp = place + self._up * HANDLE_GRIP_CENTRE_Z
        above_place = place_grasp + self._up * PLATE_RACK_TABLE_APPROACH_HEIGHT

        self._targets = {
            0: above_pick,
            1: grasp,
            3: lifted,
            4: above_place,
            5: above_place,
            6: place_grasp,
            8: above_place,
        }
        self._vertical_starts = {
            1: above_pick,
            3: grasp,
            6: above_place,
            8: place_grasp,
        }
        print(
            f"[plate-rack] selected={self._pick_path} "
            f"rack={np.round(rack, 4)} grasp={np.round(grasp, 4)} "
            f"place={np.round(place, 4)} vertical_handle=1",
            flush=True,
        )
