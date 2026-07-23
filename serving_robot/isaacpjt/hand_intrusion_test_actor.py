"""Spawn a seated person at TableSet_00 who periodically reaches onto the
table so hand_safety's ROI-intrusion detector can be exercised without a
real camera or a real hand.

Hardware pass #2 proved that late-bound UsdSkel joint rotations update the
computed skeleton transform but are not rendered by Hydra in Isaac Sim
5.1.0-rc.19. The active path therefore uses the measured F_Business_02
right-hand bind-pose offset and moves the actor root so the hand lands on
TABLE_HAND_TARGET. This root translation was verified on GPU hardware.

Usage (opt-in, from mobile_manipulator_demo.py):

    import hand_intrusion_test_actor as hand_test
    ...
    if os.environ.get("MOBILE_DEMO_HAND_TEST", "1") == "1":
        person_prim = hand_test.spawn_seated_person(stage)
        reach_animator = hand_test.ReachAnimator(person_prim)
    ...
    while simulation_app.is_running():
        simulation_app.update()
        if reach_animator is not None:
            reach_animator.update()
        time.sleep(0.010)

To use only the short fallback lean instead of the measured reach target:

    export HAND_TEST_LEAN_ONLY=1
"""

from __future__ import annotations

import os
import random
import threading
import time
from pathlib import Path

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
# The measured hand offset is centered near the wrist/hand volume. Placing
# that point exactly at TABLE_TOP_Z buries roughly half the rendered hand.
HAND_TABLE_CLEARANCE = float(
    os.environ.get("HAND_TEST_TABLE_CLEARANCE_Z", "0.06")
)
HAND_TARGET_X = float(os.environ.get("HAND_TEST_TARGET_X", "-2.75"))
HAND_TARGET_Y = float(os.environ.get("HAND_TEST_TARGET_Y", "-2.20"))
HAND_ONLY_SCALE = float(os.environ.get("HAND_TEST_SCALE", "1.35"))
TABLE_HAND_TARGET = Gf.Vec3d(
    HAND_TARGET_X, HAND_TARGET_Y, TABLE_TOP_Z + HAND_TABLE_CLEARANCE
)

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
# Turn the fixed test actor sideways so its head/torso stay outside the
# tabletop ROI while the measured right hand remains at TABLE_HAND_TARGET.
SEAT_YAW_DEGREES = float(os.environ.get("HAND_TEST_SEAT_YAW", "90.0"))
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
HAND_ONLY_PRIM_PATH = "/World/HandSafetyTestHand"
HAND_ONLY_USD = (
    Path(__file__).resolve().parents[2]
    / "nav_robot/assets/hand_safety/f_business_02_right_hand.usda"
)

# --- Reach timing. The task asks for a reach every 5-10 seconds.
MIN_PERIOD_SECONDS = float(os.environ.get("HAND_TEST_MIN_PERIOD", "5.0"))
MAX_PERIOD_SECONDS = float(os.environ.get("HAND_TEST_MAX_PERIOD", "10.0"))
REACH_TRAVEL_SECONDS = float(os.environ.get("HAND_TEST_TRAVEL_SECONDS", "0.4"))
REACH_HOLD_SECONDS = float(os.environ.get("HAND_TEST_HOLD_SECONDS", "1.4"))
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


# F_Business_02 right-hand bind-pose offset from the character root. This was
# measured on Isaac Sim 5.1 hardware from the skeleton bind transforms.
_RIGHT_HAND_REST_LOCAL_OFFSET = Gf.Vec3d(
    -0.6220545196533204, 0.050855822563171386, 1.3136680603027344
)


def _compute_hand_reach_target() -> Gf.Vec3d:
    """Return the actor-root target that puts the measured hand on the table."""
    yaw = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), SEAT_YAW_DEGREES)
    world_offset = yaw.TransformDir(_RIGHT_HAND_REST_LOCAL_OFFSET)
    target = TABLE_HAND_TARGET - world_offset
    print(
        "[hand_test] reach target from measured rest-pose hand offset "
        f"{tuple(_RIGHT_HAND_REST_LOCAL_OFFSET)} "
        f"(yawed {tuple(world_offset)}): body root -> {tuple(target)}",
        flush=True,
    )
    return target


class ReachAnimator:
    """Drive the actor root so its measured rest-pose hand reaches the table.

    Isaac Sim 5.1 hardware testing showed that late-bound UsdSkel joint
    rotations update computed transforms but do not reach Hydra skinning.
    The root translate used here is the rendering-safe, verified fallback.
    Call update() once per simulation_app frame; timing is wall-clock
    (time.time()), matching this demo's non-headless real-time frame
    pacing.
    """

    def __init__(self, prim):
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
        # Periodic motion is the active integration-test behavior.  Do not
        # let a stale HAND_TEST_FIXED_REACH=1 exported by an earlier shell
        # silently pin the actor at the table.
        self.fixed_reach = False

        lean_only = os.environ.get("HAND_TEST_LEAN_ONLY", "0") == "1"
        if lean_only:
            print(
                "[hand_test] HAND_TEST_LEAN_ONLY=1; using short fallback lean",
                flush=True,
            )
            self.reach_target = LEAN_TARGET
        else:
            try:
                self.reach_target = _compute_hand_reach_target()
            except Exception as exc:  # noqa: BLE001 -- diagnostic scaffold
                print(
                    "[hand_test] reach target computation raised "
                    f"{type(exc).__name__}: {exc}; using short fallback lean",
                    flush=True,
                )
                self.reach_target = LEAN_TARGET

        if self.fixed_reach:
            self._apply_progress(1.0)
            print(
                "[hand_test] fixed at table hand-detection target; "
                "set HAND_TEST_FIXED_REACH=0 to restore periodic motion",
                flush=True,
            )

    def _cycle_progress(self, elapsed: float) -> float:
        if elapsed < REACH_TRAVEL_SECONDS:
            return _smoothstep(elapsed / REACH_TRAVEL_SECONDS)
        if elapsed < REACH_TRAVEL_SECONDS + REACH_HOLD_SECONDS:
            return 1.0
        retract_elapsed = elapsed - REACH_TRAVEL_SECONDS - REACH_HOLD_SECONDS
        return 1.0 - _smoothstep(retract_elapsed / REACH_TRAVEL_SECONDS)

    def _apply_progress(self, progress: float) -> None:
        position = SEAT_POSITION + (self.reach_target - SEAT_POSITION) * progress
        self.translate_op.Set(position)

    def update(self) -> None:
        if self.fixed_reach:
            # Re-author the target in case another animation layer attempts
            # to modify the actor root while the test is running.
            self._apply_progress(1.0)
            return
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


class HandSpawnAnimator:
    """Create/remove the extracted hand on sim frames from ROS requests."""

    def __init__(self, stage):
        if not HAND_ONLY_USD.is_file():
            raise FileNotFoundError(f"hand-only USD is missing: {HAND_ONLY_USD}")
        self.stage = stage
        self.active = False
        self._request_lock = threading.Lock()
        self._requested_visibility: bool | None = None
        # Remove an actor left in the stage by a prior hot reload.
        self.stage.RemovePrim(PERSON_PRIM_PATH)
        self.stage.RemovePrim(HAND_ONLY_PRIM_PATH)
        print(
            f"[hand_test] service-controlled hand-only test ready: "
            f"asset={HAND_ONLY_USD}",
            flush=True,
        )

    def request_visible(self, visible: bool) -> None:
        """Queue visibility; USD mutation is deferred to the sim thread."""
        with self._request_lock:
            self._requested_visibility = bool(visible)

    def _spawn(self) -> None:
        prim = self.stage.GetPrimAtPath(HAND_ONLY_PRIM_PATH)
        if not prim.IsValid():
            xform = UsdGeom.Xform.Define(self.stage, HAND_ONLY_PRIM_PATH)
            prim = xform.GetPrim()
            prim.GetReferences().AddReference(str(HAND_ONLY_USD))
            xformable = UsdGeom.Xformable(prim)
            xformable.ClearXformOpOrder()
            xformable.AddTranslateOp(
                precision=UsdGeom.XformOp.PrecisionDouble
            ).Set(TABLE_HAND_TARGET)
            xformable.AddRotateZOp(
                precision=UsdGeom.XformOp.PrecisionDouble
            ).Set(SEAT_YAW_DEGREES)
            xformable.AddScaleOp(
                precision=UsdGeom.XformOp.PrecisionDouble
            ).Set(Gf.Vec3d(HAND_ONLY_SCALE, HAND_ONLY_SCALE, HAND_ONLY_SCALE))
        UsdGeom.Imageable(prim).MakeVisible()
        self.active = True
        print(
            f"[hand_test] hand spawned at {tuple(TABLE_HAND_TARGET)}",
            flush=True,
        )

    def _remove(self) -> None:
        prim = self.stage.GetPrimAtPath(HAND_ONLY_PRIM_PATH)
        if prim.IsValid():
            UsdGeom.Imageable(prim).MakeInvisible()
        self.active = False
        print("[hand_test] hand made invisible", flush=True)

    def update(self) -> None:
        with self._request_lock:
            requested_visibility = self._requested_visibility
            self._requested_visibility = None
        if requested_visibility is None:
            return
        if requested_visibility:
            if not self.active:
                self._spawn()
        elif self.active:
            self._remove()
