"""Top-down delivery task for the wooden cutlery box."""

import numpy as np

from isaac_scene_utils import prim_world_pose
from cutlery_serving import CUTLERY_BOX_PATH, CUTLERY_BOX_SIZE
from soda_pick_place import (
    TABLE_BOARD_CENTRE_LOCAL,
    SodaCanPickPlace,
)


CUTLERY_APPROACH_HEIGHT = 0.18
CUTLERY_LIFT_HEIGHT = 0.22
CUTLERY_TABLE_APPROACH_HEIGHT = 0.16
CUTLERY_GRIPPER_CLOSE = 0.63
CUTLERY_GRIPPER_MAX_FORCE = 12.0
CUTLERY_GRASP_Z_OFFSET = 0.010
CUTLERY_VERTICAL_ORIENTATION = np.array(
    [0.0, np.sqrt(0.5), np.sqrt(0.5), 0.0], dtype=float
)

# Robot-right side of the destination table.  The flat box's 200 mm edge runs
# along robot/table X after a 90-degree in-plane rotation.
CUTLERY_PLACE_LOCAL_X = 0.48
CUTLERY_PLACE_LOCAL_Y = -0.38
PIZZA_BOARD_THICKNESS = 0.018


class CutleryBoxPickPlace(SodaCanPickPlace):
    """Move the right-tray cutlery box directly to the table's right side."""

    def __init__(
        self, stage, wait_for_start=False, *, payload_root="/World",
        robot_root=None
    ):
        payload_root = str(payload_root).rstrip("/") or "/World"
        super().__init__(
            stage,
            wait_for_start=wait_for_start,
            task_name="cutlery",
            pick_path=f"{payload_root}/ServingCutlery/CutleryBox",
            place_left=False,
            robot_root=robot_root,
        )
        self._use_bail_detour = False
        self._gripper_close = CUTLERY_GRIPPER_CLOSE
        self._gripper_max_force = CUTLERY_GRIPPER_MAX_FORCE
        self._object_label = "cutlery box"
        self._minimum_verified_lift = 0.06
        self._grasp_hold_max_error = 0.030
        self._grasp_object_max_lateral = 0.030
        self._gripper_close_ramp_steps = 60
        # The 60 mm box contacts near 0.54 rad.  Keep only a short 30-frame
        # dwell after the smooth close; the former 0.75-rad/60-frame overdrive
        # pushed the box 25 cm across the tray.
        self._gripper_close_wait_steps = 75
        # Downward tool pose yawed 90 degrees so the finger closing direction
        # follows the rotated box's narrow 60 mm dimension.
        self._canonical_vertical_orientation = (
            CUTLERY_VERTICAL_ORIENTATION.copy()
        )
        self._vertical_orientation = CUTLERY_VERTICAL_ORIENTATION.copy()
        # Keep one uninterrupted RMPFlow target stream through descent,
        # gripping and vertical lift.  The phase-1 grasp command is held while
        # closing, then phase 3 changes only its Z component.
        self._continuous_rmp_lift = True
        self._preserve_grasp_orientation = False
        self._use_seeded_ik_for_lift = False
        # Do not change the c-space attractor at the phase boundary; that
        # would itself interrupt the RMPFlow policy state we are preserving.
        self._lock_cspace_after_grasp = False

    def _prepare_targets(self):
        box_prim = self._stage.GetPrimAtPath(self._pick_path)
        if not box_prim.IsValid():
            raise RuntimeError(f"missing cutlery box: {self._pick_path}")
        pick, _, _ = prim_world_pose(box_prim)
        pick = np.asarray(pick, dtype=float)
        self._can_prim = box_prim
        self._can_pick_start = pick.copy()

        grasp = pick + self._up * CUTLERY_GRASP_Z_OFFSET
        above_pick = grasp + self._up * CUTLERY_APPROACH_HEIGHT
        lifted = grasp + self._up * CUTLERY_LIFT_HEIGHT
        table_surface_local_z = (
            TABLE_BOARD_CENTRE_LOCAL[2] - 0.5 * PIZZA_BOARD_THICKNESS
        )
        place_local = np.array(
            [
                CUTLERY_PLACE_LOCAL_X,
                CUTLERY_PLACE_LOCAL_Y,
                table_surface_local_z + 0.5 * CUTLERY_BOX_SIZE[2] + 0.001,
            ]
        )
        place = self._local_to_world(place_local)
        above_place = place + self._up * CUTLERY_TABLE_APPROACH_HEIGHT

        # Phases 4 and 5 intentionally share the same target.  Unlike soda1,
        # the right-tray box stays on the right side of the upright pizza bail,
        # so it needs no near-side crossing waypoint.
        self._targets = {
            0: above_pick,
            1: grasp,
            3: lifted,
            4: above_place,
            5: above_place,
            6: place,
            8: above_place,
        }
        self._vertical_starts = {
            1: above_pick,
            3: grasp,
            6: above_place,
            8: place,
        }
        print(
            f"[cutlery] selected={self._pick_path} "
            f"object={np.round(pick, 4)} grasp={np.round(grasp, 4)} "
            f"grasp_z_offset={CUTLERY_GRASP_Z_OFFSET:.3f}m direct_route=1 "
            f"place={np.round(place, 4)} orientation=flat-long-X",
            flush=True,
        )
