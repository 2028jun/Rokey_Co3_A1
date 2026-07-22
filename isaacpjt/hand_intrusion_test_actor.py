"""Spawn a seated person at TableSet_00 who periodically reaches onto the
table so hand_safety's ROI-intrusion detector can be exercised without a
real camera or a real hand.

Hardware pass #1 (commit "Fix hand-safety GPU test scaffold and default it
to run out of the box") confirmed topics/gating/ROI placement work end to
end, but the whole-body-slide reach never puts a recognizable hand in the
ROI: the character's own baked idle animation keeps its hands at its sides
regardless of where the root prim is translated to.

This pass adds real per-joint arm control via UsdSkel: it replaces the
skeleton's animation source with one seeded from its rest pose (so the idle
loop stops fighting us), then rotates just the forearm joint toward the
table over the same 5-10s cycle used before. The forearm joint is found by
fuzzy name matching (see _find_reach_joint_index) since the actual joint
names on F_Business_02's skeleton have never been inspected on real
hardware. Nothing UsdSkel-related below has been run.

`ReachAnimator` still authors the small whole-body lean from pass #1
(distance reduced -- the arm is now expected to do the real work) and now
also drives the forearm joint if skeleton setup succeeds; if it doesn't
(wrong API call for this USD build, no skeleton found, no forearm-like
joint name), it prints why and silently falls back to lean-only, matching
pass #1's behavior exactly.

Usage (opt-in, from mobile_manipulator_demo.py):

    import hand_intrusion_test_actor as hand_test
    ...
    if os.environ.get("MOBILE_DEMO_HAND_TEST", "1") == "1":
        person_prim = hand_test.spawn_seated_person(stage)
        reach_animator = hand_test.ReachAnimator(person_prim, stage)
    ...
    while simulation_app.is_running():
        simulation_app.update()
        if reach_animator is not None:
            reach_animator.update()
        time.sleep(0.010)

Debugging on hardware: if the reach still doesn't reach, run
`hand_test.list_skeleton_joints(stage, hand_test.PERSON_PRIM_PATH)` from
the Isaac Sim script console after spawning, find the real forearm/hand
joint in the printed list, and set:

    export HAND_TEST_REACH_JOINT_NAME="<substring from the printed path>"
    export HAND_TEST_REACH_JOINT_AXIS=X   # or Y / Z -- try each
    export HAND_TEST_REACH_JOINT_ANGLE_DEG=-70  # sign/magnitude, tune visually
"""

from __future__ import annotations

import os
import random
import time

from pxr import Gf, Usd, UsdGeom, UsdSkel


# Verified against the bucket listing (S3 XML ListObjectsV2 on
# omniverse-content-production, prefix
# Assets/Isaac/5.1/Isaac/People/Characters/): the original guessed folder
# name "female_adult_business_02" does not exist (404). The real folder is
# "F_Business_02" (the textures inside it keep the female_adult_business_02
# naming, which is why the original guess looked plausible). Confirmed
# resolvable with `curl -I` -> 200 OK.
PERSON_USD = os.environ.get(
    "HAND_TEST_PERSON_USD",
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/5.1/Isaac/People/Characters/F_Business_02/"
    "F_Business_02.usd",
)

# --- TableSet_00 geometry, read directly from
# assets/lightweight_restaurant/lightweight_pizza_restaurant.usda.
# This is the only table the fixed table_camera ever frames (it is mounted
# on the robot at its TableSet_00 docking pose), so it is "table 1" for
# this workspace.
TABLE_COLLIDER_CENTER = Gf.Vec3d(-3.2, -2.2, 0.365)
TABLE_TOP_Z = 0.73  # collider center z (0.365) + half-height (0.365)
TABLE_HAND_TARGET = Gf.Vec3d(-3.2, -2.2, TABLE_TOP_Z)

# Chair_01_Visual under TableSet_00: translate=(-2.7, -3.2, 0), rotateZ=180
# (facing the table). Verified against a captured table_camera frame:
# Chair_00 (-3.7, -3.2) sits at the far edge of the robot-mounted camera's
# view and is mostly clipped; Chair_01 is well-centered in frame. This
# character asset (F_Business_02) is a standing-pose People asset, not a
# seated one, so SEAT_TORSO_Z is floor height (0.0), not chair-seat height.
SEAT_XY = (
    float(os.environ.get("HAND_TEST_SEAT_X", "-2.7")),
    float(os.environ.get("HAND_TEST_SEAT_Y", "-3.2")),
)
SEAT_YAW_DEGREES = float(os.environ.get("HAND_TEST_SEAT_YAW", "180.0"))
SEAT_TORSO_Z = float(os.environ.get("HAND_TEST_SEAT_Z", "0.0"))
SEAT_POSITION = Gf.Vec3d(SEAT_XY[0], SEAT_XY[1], SEAT_TORSO_Z)

# How far the whole body leans toward the table on top of whatever the arm
# joint contributes. Pass #1 used the full seat-to-table distance (~1m) and
# that alone never got a hand over the table; this is now just a small
# assist, not the primary mechanism.
LEAN_DISTANCE = float(os.environ.get("HAND_TEST_LEAN_DISTANCE", "0.15"))
_seat_to_table = TABLE_HAND_TARGET - SEAT_POSITION
_seat_to_table_length = _seat_to_table.GetLength()
LEAN_TARGET = (
    SEAT_POSITION + _seat_to_table * (LEAN_DISTANCE / _seat_to_table_length)
    if _seat_to_table_length > 1e-6
    else SEAT_POSITION
)

PERSON_PRIM_PATH = "/World/HandSafetyTestActor"

# --- Reach timing. The task asks for a reach every 5-10 seconds.
MIN_PERIOD_SECONDS = float(os.environ.get("HAND_TEST_MIN_PERIOD", "5.0"))
MAX_PERIOD_SECONDS = float(os.environ.get("HAND_TEST_MAX_PERIOD", "10.0"))
REACH_TRAVEL_SECONDS = float(os.environ.get("HAND_TEST_TRAVEL_SECONDS", "0.4"))
REACH_HOLD_SECONDS = float(os.environ.get("HAND_TEST_HOLD_SECONDS", "0.4"))
# hand_safety requires confirmation_frames=3 consecutive detections at
# process_rate=30 Hz (~0.1s) before it reports an intrusion -- the hold
# time above must stay comfortably longer than that or every reach will be
# invisible to the detector.

# --- Forearm joint search/drive. All overridable once the real skeleton
# has been inspected on hardware (see module docstring).
_REACH_JOINT_NAME_OVERRIDE = os.environ.get("HAND_TEST_REACH_JOINT_NAME", "")
_REACH_JOINT_AXIS = os.environ.get("HAND_TEST_REACH_JOINT_AXIS", "X").upper()
REACH_JOINT_ANGLE_DEG = float(
    os.environ.get("HAND_TEST_REACH_JOINT_ANGLE_DEG", "-70.0")
)
_FOREARM_NAME_HINTS = ("forearm", "lowerarm", "lower_arm", "elbow")
_RIGHT_SIDE_HINTS = ("right", "_r_", "r_hand", "r_arm", "rt_")
_AXIS_VECTORS = {
    "X": Gf.Vec3d(1.0, 0.0, 0.0),
    "Y": Gf.Vec3d(0.0, 1.0, 0.0),
    "Z": Gf.Vec3d(0.0, 0.0, 1.0),
}


def _smoothstep(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def spawn_seated_person(stage):
    """Reference a People character at TableSet_00's Chair_01 and return its prim."""
    xform = UsdGeom.Xform.Define(stage, PERSON_PRIM_PATH)
    prim = xform.GetPrim()
    prim.GetReferences().AddReference(PERSON_USD)

    # The referenced character asset's own root prim already authors an
    # `xformOp:rotateXYZ` attribute with double3 precision. Once an
    # attribute name/type is defined in any layer contributing to this
    # prim, USD holds every layer to that same type -- XformCommonAPI
    # always names its rotate op "xformOp:rotateXYZ" and only writes
    # GfVec3f, so it collides no matter what (ClearXformOpOrder only
    # clears the *order* list, not the pre-existing attribute spec).
    # Sidestep by authoring our own double-precision ops directly, using
    # a differently-named rotateZ op (this test only ever needs a
    # yaw-around-Z, so RotateXYZ was never necessary).
    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    translate_op = xformable.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble)
    translate_op.Set(SEAT_POSITION)
    rotate_op = xformable.AddRotateZOp(precision=UsdGeom.XformOp.PrecisionDouble)
    rotate_op.Set(SEAT_YAW_DEGREES)
    print(
        f"[hand_test] spawned {PERSON_USD} at {PERSON_PRIM_PATH}, "
        f"seat={tuple(SEAT_POSITION)} yaw={SEAT_YAW_DEGREES}",
        flush=True,
    )
    return prim


def _find_skeleton(root_prim) -> UsdSkel.Skeleton | None:
    for prim in Usd.PrimRange(root_prim):
        if prim.IsA(UsdSkel.Skeleton):
            return UsdSkel.Skeleton(prim)
    return None


def _find_skel_root(root_prim):
    if root_prim.IsA(UsdSkel.Root):
        return root_prim
    for prim in Usd.PrimRange(root_prim):
        if prim.IsA(UsdSkel.Root):
            return prim
    return None


def list_skeleton_joints(stage, root_path: str = PERSON_PRIM_PATH) -> None:
    """Print every joint path under root_path's skeleton. Run this from the
    Isaac Sim script console after spawn_seated_person() to find the real
    forearm/hand joint name if auto-detection picks the wrong one.
    """
    root_prim = stage.GetPrimAtPath(root_path)
    skeleton = _find_skeleton(root_prim)
    if skeleton is None:
        print(f"[hand_test] no UsdSkel.Skeleton found under {root_path}", flush=True)
        return
    joints = list(skeleton.GetJointsAttr().Get() or [])
    print(f"[hand_test] {len(joints)} joints under {root_path}:", flush=True)
    for index, joint in enumerate(joints):
        print(f"  [{index}] {joint}", flush=True)


def _find_reach_joint_index(joint_strs: list[str]) -> int | None:
    if _REACH_JOINT_NAME_OVERRIDE:
        for index, path_str in enumerate(joint_strs):
            if _REACH_JOINT_NAME_OVERRIDE.lower() in path_str.lower():
                return index
        print(
            f"[hand_test] HAND_TEST_REACH_JOINT_NAME={_REACH_JOINT_NAME_OVERRIDE!r} "
            "matched no joint; falling back to auto-detection",
            flush=True,
        )

    candidates = []
    for index, path_str in enumerate(joint_strs):
        lowered = path_str.lower()
        if any(hint in lowered for hint in _FOREARM_NAME_HINTS):
            side_score = 1 if any(hint in lowered for hint in _RIGHT_SIDE_HINTS) else 0
            candidates.append((side_score, index, path_str))
    if not candidates:
        return None
    candidates.sort(key=lambda item: -item[0])
    return candidates[0][1]


def _setup_skeleton_reach(stage, root_path: str) -> dict | None:
    """Replace the skeleton's animation source with one seeded from its
    rest pose, and return the bits ReachAnimator needs to drive the
    forearm joint's rotation each frame. Returns None (with a printed
    reason) if anything about the asset doesn't match what this function
    assumes -- callers must treat that as "fall back to lean-only", not
    an error.
    """
    root_prim = stage.GetPrimAtPath(root_path)
    skeleton = _find_skeleton(root_prim)
    if skeleton is None:
        print(f"[hand_test] no skeleton under {root_path}; lean-only reach", flush=True)
        return None

    skel_root_prim = _find_skel_root(root_prim)
    if skel_root_prim is None:
        print(f"[hand_test] no UsdSkel.Root under {root_path}; lean-only reach", flush=True)
        return None

    joints_attr = skeleton.GetJointsAttr().Get()
    joints = list(joints_attr or [])
    if not joints:
        print("[hand_test] skeleton has no joints; lean-only reach", flush=True)
        return None
    joint_strs = [str(joint) for joint in joints]

    reach_index = _find_reach_joint_index(joint_strs)
    if reach_index is None:
        print(
            "[hand_test] no forearm-like joint name matched (tried "
            f"{_FOREARM_NAME_HINTS}); run list_skeleton_joints() and set "
            "HAND_TEST_REACH_JOINT_NAME. Lean-only reach for now.",
            flush=True,
        )
        return None

    rest_matrices = skeleton.GetRestTransformsAttr().Get()
    if not rest_matrices or len(rest_matrices) != len(joints):
        print(
            "[hand_test] restTransforms missing or joint-count mismatch; "
            "lean-only reach",
            flush=True,
        )
        return None

    anim_path = root_path + "/HandTestReachAnimation"
    animation = UsdSkel.Animation.Define(stage, anim_path)
    animation.CreateJointsAttr().Set(joints_attr)

    translations, rotations, scales = [], [], []
    for matrix in rest_matrices:
        translation, rotation, scale = UsdSkel.DecomposeTransform(matrix)
        translations.append(translation)
        rotations.append(rotation)
        scales.append(scale)
    animation.CreateTranslationsAttr().Set(translations)
    rotations_attr = animation.CreateRotationsAttr()
    rotations_attr.Set(rotations)
    animation.CreateScalesAttr().Set(scales)

    binding_api = UsdSkel.BindingAPI.Apply(skel_root_prim)
    binding_api.CreateAnimationSourceRel().SetTargets([animation.GetPath()])

    axis_vector = _AXIS_VECTORS.get(_REACH_JOINT_AXIS)
    if axis_vector is None:
        print(
            f"[hand_test] HAND_TEST_REACH_JOINT_AXIS={_REACH_JOINT_AXIS!r} "
            "invalid (use X/Y/Z); defaulting to X",
            flush=True,
        )
        axis_vector = _AXIS_VECTORS["X"]

    print(
        f"[hand_test] driving joint [{reach_index}] {joint_strs[reach_index]} "
        f"around local {_REACH_JOINT_AXIS} up to {REACH_JOINT_ANGLE_DEG} deg "
        f"(seeded {len(joints)}-joint rest-pose animation at {anim_path})",
        flush=True,
    )
    return {
        "rotations_attr": rotations_attr,
        "rest_rotations": rotations,
        "reach_index": reach_index,
        "axis_vector": axis_vector,
    }


class ReachAnimator:
    """Drives PERSON_PRIM_PATH's whole-body lean and (if skeleton setup
    succeeds) its forearm joint's rotation, on a randomized 5-10s period.
    Call update() once per simulation_app frame; timing is wall-clock
    (time.time()), matching this demo's non-headless real-time frame
    pacing.
    """

    def __init__(self, prim, stage=None):
        # Matches the double-precision translateOp spawn_seated_person()
        # authors directly (see its comment on why XformCommonAPI can't be
        # used here). GetOrderedXformOps()[0] is that translate op, since
        # it was the first (and only translate) op added.
        self.translate_op = UsdGeom.Xformable(prim).GetOrderedXformOps()[0]
        self.active = False
        self.event_start: float | None = None
        self.next_event_time = time.time() + random.uniform(
            MIN_PERIOD_SECONDS, MAX_PERIOD_SECONDS
        )

        self.skeleton_reach = None
        if stage is not None:
            try:
                self.skeleton_reach = _setup_skeleton_reach(
                    stage, str(prim.GetPath())
                )
            except Exception as exc:  # noqa: BLE001 -- see module docstring
                print(
                    "[hand_test] skeleton reach setup raised "
                    f"{type(exc).__name__}: {exc}; falling back to "
                    "lean-only reach",
                    flush=True,
                )
                self.skeleton_reach = None

    def _cycle_progress(self, elapsed: float) -> float:
        if elapsed < REACH_TRAVEL_SECONDS:
            return _smoothstep(elapsed / REACH_TRAVEL_SECONDS)
        if elapsed < REACH_TRAVEL_SECONDS + REACH_HOLD_SECONDS:
            return 1.0
        retract_elapsed = elapsed - REACH_TRAVEL_SECONDS - REACH_HOLD_SECONDS
        return 1.0 - _smoothstep(retract_elapsed / REACH_TRAVEL_SECONDS)

    def _apply_progress(self, progress: float) -> None:
        position = SEAT_POSITION + (LEAN_TARGET - SEAT_POSITION) * progress
        self.translate_op.Set(position)

        if self.skeleton_reach is None:
            return
        try:
            angle_deg = REACH_JOINT_ANGLE_DEG * progress
            delta = Gf.Rotation(self.skeleton_reach["axis_vector"], angle_deg)
            delta_quat = Gf.Quatf(delta.GetQuat())
            rest_rotation = self.skeleton_reach["rest_rotations"][
                self.skeleton_reach["reach_index"]
            ]
            rotations = list(self.skeleton_reach["rest_rotations"])
            rotations[self.skeleton_reach["reach_index"]] = rest_rotation * delta_quat
            self.skeleton_reach["rotations_attr"].Set(rotations)
        except Exception as exc:  # noqa: BLE001 -- see module docstring
            print(
                "[hand_test] skeleton reach update raised "
                f"{type(exc).__name__}: {exc}; disabling joint drive for "
                "the rest of this run (lean continues)",
                flush=True,
            )
            self.skeleton_reach = None

    def update(self) -> None:
        now = time.time()
        if not self.active:
            if now < self.next_event_time:
                return
            self.active = True
            self.event_start = now
            print("[hand_test] reach start", flush=True)

        elapsed = now - self.event_start
        total_duration = 2.0 * REACH_TRAVEL_SECONDS + REACH_HOLD_SECONDS
        if elapsed >= total_duration:
            self.active = False
            self._apply_progress(0.0)
            self.next_event_time = now + random.uniform(
                MIN_PERIOD_SECONDS, MAX_PERIOD_SECONDS
            )
            print(
                "[hand_test] reach end, next in "
                f"{self.next_event_time - now:.1f}s",
                flush=True,
            )
            return

        self._apply_progress(self._cycle_progress(elapsed))
