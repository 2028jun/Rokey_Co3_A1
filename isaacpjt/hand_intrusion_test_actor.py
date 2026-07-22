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


# --- VERIFY: no human/character USD asset exists anywhere in this repo or
# in any sibling branch (checked hmi-web, jaehyeon, main, test, woduq,
# younggi). This placeholder path is a guess at NVIDIA's Isaac Sim 5.1
# People asset layout (mirrors the pattern already used for the D455
# sensor asset in mobile_manipulator_demo.py's D455_ASSET_USD). Open the
# Isaac Sim content browser, find an actual seated/standing People
# character under .../Isaac/People/Characters/, and either edit this
# default or export HAND_TEST_PERSON_USD before running.
PERSON_USD = os.environ.get(
    "HAND_TEST_PERSON_USD",
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/5.1/Isaac/People/Characters/female_adult_business_02/"
    "female_adult_business_02.usd",
)

# --- TableSet_00 geometry, read directly from
# assets/lightweight_restaurant/lightweight_pizza_restaurant.usda.
# This is the only table the fixed table_camera ever frames (it is mounted
# on the robot at its TableSet_00 docking pose), so it is "table 1" for
# this workspace.
TABLE_COLLIDER_CENTER = Gf.Vec3d(-3.2, -2.2, 0.365)
TABLE_TOP_Z = 0.73  # collider center z (0.365) + half-height (0.365)
TABLE_HAND_TARGET = Gf.Vec3d(-3.2, -2.2, TABLE_TOP_Z)

# Chair_00_Visual under TableSet_00: translate=(-3.7, -3.2, 0), rotateZ=180
# (facing the table). VERIFY this is the chair actually inside the
# robot-mounted camera's field of view -- pick a different Chair_0N under
# TableSet_00 if Chair_00 turns out to be off-frame or occluded by the
# robot's docking side (robot docks near x=-1.82, i.e. the table's +X/east
# side).
SEAT_XY = (-3.7, -3.2)
SEAT_YAW_DEGREES = 180.0  # VERIFY: faces table if the asset's forward
# axis matches the restaurant's chair convention; Isaac People assets vary,
# rotate in 90-degree steps if the person spawns facing away/sideways.
SEAT_TORSO_Z = 0.55  # approximate seated-hand resting height; VERIFY
# against the actual character's proportions once loaded.
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

    api = UsdGeom.XformCommonAPI(prim)
    api.SetTranslate(SEAT_POSITION)
    api.SetRotate(Gf.Vec3f(0.0, 0.0, SEAT_YAW_DEGREES))
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
        self.api = UsdGeom.XformCommonAPI(prim)
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
            self.api.SetTranslate(SEAT_POSITION)
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
        self.api.SetTranslate(position)
