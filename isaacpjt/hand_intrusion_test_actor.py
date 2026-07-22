"""Spawn a seated person at TableSet_00 who periodically reaches onto the
table so hand_safety's ROI-intrusion detector can be exercised without a
real camera or a real hand.

UNVERIFIED: this module has never been run inside Isaac Sim (this repo was
prepared on a machine with no GPU/Isaac Sim access). Treat every constant
below as a first guess that needs correcting once you can see the scene.
See docs/VISION_TEST_GPU_PROMPT.md for the full task/verification checklist.

Usage (opt-in, from mobile_manipulator_demo.py):

    import hand_intrusion_test_actor as hand_test
    ...
    if os.environ.get("MOBILE_DEMO_HAND_TEST", "0") == "1":
        person_prim = hand_test.spawn_seated_person(stage)
        reach_animator = hand_test.ReachAnimator(person_prim)
    ...
    while simulation_app.is_running():
        simulation_app.update()
        if reach_animator is not None:
            reach_animator.update()
        time.sleep(0.010)
"""

from __future__ import annotations

import math
import os
import random
import time

from pxr import Gf, UsdGeom


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


def _smoothstep(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def spawn_seated_person(stage):
    """Reference a People character at TableSet_00's Chair_00 and return its prim.

    KNOWN SIMPLIFICATION: rather than driving individual skeleton joints
    (this repo has no UsdSkel/keyframe precedent to build on, and the
    actual character's joint names can't be discovered without Isaac Sim
    running), the whole referenced prim is translated as a rigid body
    between a seated rest position and a table-reach position. Visually
    this reads as the person's whole body sliding toward the table rather
    than an isolated arm reach. If a real per-joint arm animation is
    wanted, that requires inspecting the loaded character's skeleton (its
    joint list is visible in the Isaac Sim stage tree / Property window)
    and is out of scope for this pass -- flag it back if you need it.
    """
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


class ReachAnimator:
    """Drives PERSON_PRIM_PATH between SEAT_POSITION and TABLE_HAND_TARGET
    on a randomized 5-10s period. Call update() once per simulation_app
    frame; timing is wall-clock (time.time()), matching this demo's
    non-headless real-time frame pacing.
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

    def _cycle_progress(self, elapsed: float) -> float:
        if elapsed < REACH_TRAVEL_SECONDS:
            return _smoothstep(elapsed / REACH_TRAVEL_SECONDS)
        if elapsed < REACH_TRAVEL_SECONDS + REACH_HOLD_SECONDS:
            return 1.0
        retract_elapsed = elapsed - REACH_TRAVEL_SECONDS - REACH_HOLD_SECONDS
        return 1.0 - _smoothstep(retract_elapsed / REACH_TRAVEL_SECONDS)

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
            self.translate_op.Set(SEAT_POSITION)
            self.next_event_time = now + random.uniform(
                MIN_PERIOD_SECONDS, MAX_PERIOD_SECONDS
            )
            print(
                "[hand_test] reach end, next in "
                f"{self.next_event_time - now:.1f}s",
                flush=True,
            )
            return

        progress = self._cycle_progress(elapsed)
        position = SEAT_POSITION + (TABLE_HAND_TARGET - SEAT_POSITION) * progress
        self.translate_op.Set(position)
