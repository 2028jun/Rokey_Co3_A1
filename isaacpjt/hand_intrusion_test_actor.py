"""Spawn a seated person at TableSet_00 who periodically reaches onto the
table so hand_safety's ROI-intrusion detector can be exercised without a
real camera or a real hand.

Hardware pass #1 (commit "Fix hand-safety GPU test scaffold and default it
to run out of the box") confirmed topics/gating/ROI placement work end to
end, but the whole-body-slide reach never puts a recognizable hand in the
ROI: the character's own baked idle animation keeps its hands at its sides
regardless of where the root prim is translated to.

Hardware pass #2 tried real per-joint arm control via UsdSkel (rotate the
forearm joint through a freshly-bound, rest-pose-seeded SkelAnimation).
That setup ran without exceptions and `UsdSkel.Cache`/`SkelQuery` confirmed
the joint's *computed* local transform updated correctly -- but the
*rendered* image never showed the arm move, proven with a pixel-diff on
matched before/after renders (isolated repro, dome-lit, camera framed on
the character, explicit `simulation_app.update()` ticks and a single
deterministic `rep.orchestrator.step()` render -- no live-timing race
possible). Writing through the Fabric-native `usdrt.UsdSkel.Animation` API
instead of `pxr.UsdSkel` made no difference either. This matches a
documented NVIDIA caveat (`omni.anim.graph.core` changelog: "usdskel
omnihydra does not handle both fabric + USD change well") -- late-bound
SkelAnimation joint rotations on Isaac Sim 5.1.0-rc.19 do not appear to
reach the Hydra skinning pipeline in this build, regardless of which USD
API authors them. Per-joint rotation is not used here anymore.

This pass instead computes, analytically, the rest-pose *world* offset of
the character's right hand from its own root Xform, rotates that offset by
`SEAT_YAW_DEGREES` to match how the character is placed, and picks the
whole-body translate target so that offset lands exactly on
`TABLE_HAND_TARGET`. This only relies on `Xformable.Set()` on a plain
translate op (confirmed on hardware: a 20s hold with a long, deliberately
un-rushed capture window shows the character's hand landing squarely on
the table -- earlier same-session captures that looked like translate
"wasn't rendering" were simply mistimed, grabbed after a 2-4s reach window
had already ended given this script's own multi-step capture latency, not
a real rendering bug). It is a known visual compromise, not a real arm
reach: since this character's rest pose holds the arm out and up near
shoulder height, the whole body has to translate low enough to bring that
hand down to table height, so during the reach the character visibly
sinks/lies across the table. Acceptable for exercising the detector;
revisit with a seated/lower-rest-pose character if a natural-looking
reach is ever needed.

The hand offset below is hardcoded (measured once via
`skeleton.GetBindTransformsAttr()` in an isolated script) rather than
queried from the live spawned prim each run, purely to keep this module's
runtime path simple -- not because live UsdSkel queries were shown to
cause any problem (they weren't retested after the capture-timing bug
above was found, so that earlier suspicion is unconfirmed). If
`HAND_TEST_PERSON_USD` is overridden to a different character, re-measure
this constant for that asset's own right-hand joint (run
`list_skeleton_joints()` to find the joint, then read its
`bindTransforms` entry) rather than assuming this offset still applies.

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

Debugging on hardware: if the reach target still looks wrong, run
`hand_test.list_skeleton_joints(stage, hand_test.PERSON_PRIM_PATH)` from
the Isaac Sim script console after spawning to see every joint path, and
compare against `_RIGHT_HAND_REST_LOCAL_OFFSET` below (logged at startup
as `[hand_test] reach target from hardcoded rest-pose hand offset ...`).
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

# Fallback lean target (small nudge toward the table) used only if the
# skeleton/hand-joint lookup below fails -- matches pass #1's mechanism.
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
REACH_TRAVEL_SECONDS = float(os.environ.get("HAND_TEST_TRAVEL_SECONDS", "0.8"))
REACH_HOLD_SECONDS = float(os.environ.get("HAND_TEST_HOLD_SECONDS", "0.4"))
# hand_safety requires confirmation_frames=3 consecutive detections at
# process_rate=30 Hz (~0.1s) before it reports an intrusion -- the hold
# time above must stay comfortably longer than that or every reach will be
# invisible to the detector.

_RIGHT_SIDE_HINTS = ("right", "_r_", "r_hand", "r_arm", "rt_")
_HAND_EXCLUDE_HINTS = ("thumb", "index", "mid", "ring", "pinky")


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


def list_skeleton_joints(stage, root_path: str = PERSON_PRIM_PATH) -> None:
    """Print every joint path under root_path's skeleton. Run this from the
    Isaac Sim script console after spawn_seated_person() to sanity-check
    which joint ReachAnimator picked as "the right hand".
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


# Rest-pose world offset of F_Business_02's right hand
# (.../R_Clavicle/R_Upperarm/R_Forearm/R_Hand) from its own character root,
# measured once via `skeleton.GetBindTransformsAttr()` in an isolated
# script rather than queried at runtime (see module docstring for why:
# keeps this module's runtime path simple; re-measure if PERSON_USD is
# overridden to a different character).
_RIGHT_HAND_REST_LOCAL_OFFSET = Gf.Vec3d(
    -0.6220545196533204, 0.050855822563171386, 1.3136680603027344
)


def _compute_hand_reach_target() -> Gf.Vec3d:
    """World-space whole-body translate target that puts the character's
    rest-pose right hand on TABLE_HAND_TARGET, given the character is
    yawed by SEAT_YAW_DEGREES. Pure Gf math against the hardcoded offset
    above.
    """
    yaw = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), SEAT_YAW_DEGREES)
    world_offset = yaw.TransformDir(_RIGHT_HAND_REST_LOCAL_OFFSET)
    target = TABLE_HAND_TARGET - world_offset
    print(
        f"[hand_test] reach target from hardcoded rest-pose hand offset "
        f"{tuple(_RIGHT_HAND_REST_LOCAL_OFFSET)} (yawed {tuple(world_offset)}): "
        f"body root -> {tuple(target)}",
        flush=True,
    )
    return target


class ReachAnimator:
    """Drives PERSON_PRIM_PATH's whole body between SEAT_POSITION and a
    reach target on a randomized 5-10s period, via a plain translate op
    (see module docstring for why per-joint rotation isn't used). Call
    update() once per simulation_app frame; timing is wall-clock
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

        try:
            self.reach_target = _compute_hand_reach_target()
        except Exception as exc:  # noqa: BLE001 -- see module docstring
            print(
                "[hand_test] reach target computation raised "
                f"{type(exc).__name__}: {exc}; falling back to "
                "lean-only reach",
                flush=True,
            )
            self.reach_target = LEAN_TARGET

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
