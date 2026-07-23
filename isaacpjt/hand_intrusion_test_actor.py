"""Spawn a seated person at TableSet_00 who periodically reaches onto the
table so hand_safety's ROI-intrusion detector can be exercised without a
real camera or a real hand.

Hardware pass #1 confirmed topics/gating/ROI placement work end to end, but
the whole-body-slide reach never puts a recognizable hand in the ROI in a
natural way: the character's own baked idle animation keeps its hands at
its sides, so the *entire* body had to translate to bring the hand to
table height, visibly sinking/lying across the table during every reach.

Hardware pass #2 (per-joint UsdSkel rotation) and pass #4 (the officially
supported Animation Graph route) both failed to move the arm on this Isaac
Sim build -- see the original module history for the full writeup. Both
failures are specific to the *skinning* pipeline (Hydra not picking up
late-bound SkelAnimation edits, and omni.anim.graph.core segfaulting
headless). Neither failure applies to a plain, non-skinned Xform hierarchy,
which is exactly how pass #1's whole-body translate itself worked -- a
plain `Xformable` translate/orient op update always rendered correctly.

Pass #5 exploited that: it extracts the character's own right upper-arm and
forearm+hand geometry (skin + sleeve + glove meshes, split by which
skeleton joint dominantly influences each vertex) out of the skinned mesh
entirely and re-parents those pieces under a plain two-bone `Xform` chain
(`RightArmRig -> ElbowPivot`) that this script drives directly with
`xformOp:orient` quaternions -- no UsdSkel, no Animation Graph, so neither
of the previously-hit rendering bugs apply. Verified on real hardware in
pass 5 (shoulder cap + elbow cap spheres added to plug a rotation-swept gap
at the boundary, see `GPU_RUN_LOG.txt`); the two-bone reach mechanism
itself is solid.

Pass #5 still moved the whole body during every reach (a `translateOp`
"lean" toward the target, on top of the arm rotation), because Chair_01's
original seat, ~1.4m from `TABLE_HAND_TARGET` measured from the actual
shoulder position, is far outside this character's ~0.5m two-bone reach --
an arm-only reach was geometrically impossible from that seat. Pass #6
(this one) removes the body motion entirely instead of shrinking it: the
seat itself has been moved close enough to the table that the arm alone
(shoulder + elbow rotation only, `SEAT_POSITION` never animates) covers the
full distance. The character sits still for the entire reach; only
`RightArmRig`'s and `ElbowPivot`'s `xformOp:orient` change.

IMPORTANT geometric limit, worth understanding before moving the seat
again: this character is a *standing*-pose asset, so its shoulder
(`P_UP`) sits 1.44m above its own root regardless of where that root is
placed, while `TABLE_HAND_TARGET` is only 0.83m up -- a 0.61m vertical gap
by itself, before any horizontal distance is even considered. Since a
`rotateZ`-only yaw doesn't change a point's Z, moving the seat's X/Y can
NEVER close that vertical gap; only `SEAT_TORSO_Z` (i.e. sinking the root)
can, and this character's two-bone reach (0.496m max, wrist to shoulder --
see `_ARM_MAX_REACH`) is *less than* that 0.61m vertical gap on its own.
Concretely: with zero root sink, no seat X/Y position whatsoever brings the
target in reach. The shipped defaults below use a modest `SEAT_TORSO_Z`
(-0.20, versus pass 5's worst-case dynamic -0.427) plus moving the seat's
X/Y much closer to the table (from Chair_01's original ~(-2.7,-3.2) to
~(-3.34,-2.22) -- close to the table's own footprint) to land at a natural
~64deg elbow bend with 0.496m*0.85 of the arm's own reach used. This was
computed from `P_UP`/`TABLE_HAND_TARGET`/bone lengths only, offline,
without seeing the actual `TableSet_00`/`Chair_01` geometry or whether a
seat that close visually clips into the table -- **the exact
`HAND_TEST_SEAT_X`/`HAND_TEST_SEAT_Y`/`HAND_TEST_SEAT_Z` values need eyes
on the real scene to confirm they don't clip**, this pass's log entry (or a
follow-up) should record whatever adjustment was actually needed.

The two-bone shoulder/elbow angles are solved analytically once at
`ReachAnimator` construction (`two_bone_ik.solve_two_bone_ik`), not
baked/measured by hand, so moving the seat via the env vars above
re-solves the correct angles automatically -- no code change needed to
retune. `solve_two_bone_ik` clamps its internal distance to the arm's
reachable range, so an unreachable seat position doesn't crash; it just
extends the arm to its max length short of the actual target (watch the
"shoulder->target distance" log line at startup against `_ARM_MAX_REACH`
to tell whether the current seat is actually in reach). The solver, the
extraction pivots, and this seat placement were all unit-verified against
the actual mesh data (see the asset-build notes at the bottom of this
file) with sub-micron/near-float-precision error; what has NOT been
verified is how it actually *renders* -- this pass was written and tested
in a CPU-only environment without Isaac Sim, same as pass 5 prep was.
Confirm the reach visually, and specifically confirm the seat doesn't
visually clip into the table or look out of place, before trusting this
over pass 5's lean-based version (env var `HAND_TEST_RIG_MODE=legacy`
still switches all the way back to pass #1-4's original whole-body-slide
mechanism if this needs A/B debugging).

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
"""

from __future__ import annotations

import os
import random
import time
from pathlib import Path

import omni.kit.app
from pxr import Gf, Usd, UsdGeom

from two_bone_ik import solve_two_bone_ik

# --- Rig asset. Built offline (see build notes at the bottom) by pulling the
# right upper-arm + forearm + hand geometry out of PERSON_USD's skinned
# meshes (by dominant skinning-joint per vertex) and re-parenting it under a
# plain, non-skinned two-bone Xform chain; everything else about the
# character (torso, legs, head, left arm) is an untouched reference to the
# original asset with just the right-arm faces removed so nothing overlaps.
# Override with a local path if you haven't deployed this asset yet -- see
# assets/rigid_arm_asset.usda in this same delivery.
#
# Must be absolute: this prim gets referenced onto a stage whose root layer
# is the restaurant asset (assets/lightweight_restaurant/...), and USD
# resolves a relative reference path against the *referencing layer's*
# location, not this process's CWD -- a bare "assets/rigid_arm_asset.usda"
# silently 404s on hardware as "Could not open asset ... for reference"
# (looks for it under assets/lightweight_restaurant/assets/).
RIG_ASSET_USD = os.environ.get(
    "HAND_TEST_RIG_ASSET",
    str(Path(__file__).resolve().parents[1] / "assets" / "rigid_arm_asset.usda"),
)
RIG_MODE = os.environ.get("HAND_TEST_RIG_MODE", "rigid_arm")  # or "legacy"

# --- TableSet_00 geometry, read directly from
# assets/lightweight_restaurant/lightweight_pizza_restaurant.usda.
# This is the only table the fixed table_camera ever frames (it is mounted
# on the robot at its TableSet_00 docking pose), so it is "table 1" for
# this workspace.
TABLE_COLLIDER_CENTER = Gf.Vec3d(-3.2, -2.2, 0.365)
TABLE_TOP_Z = 0.73  # collider center z (0.365) + half-height (0.365)
HAND_TARGET_Z_OFFSET = float(os.environ.get("HAND_TEST_TARGET_Z_OFFSET", "0.10"))
TABLE_HAND_TARGET = Gf.Vec3d(-3.2, -2.2, TABLE_TOP_Z + HAND_TARGET_Z_OFFSET)

# Chair_01_Visual under TableSet_00 was originally at translate=(-2.7,-3.2,0)
# (pass #1-5), ~1.4m from TABLE_HAND_TARGET measured from the shoulder --
# out of reach for a static arm-only pose (see module docstring). Moved much
# closer to the table for pass #6's static-body reach, computed offline from
# P_UP/TABLE_HAND_TARGET/bone lengths only -- NOT yet confirmed against the
# real TableSet_00/Chair_01 geometry. This is very likely close enough to
# the table's own footprint to need a visual nudge on hardware; treat these
# three env vars as the primary tuning knobs for that, not code changes.
# rotateZ=180 (facing the table) is unchanged from every prior pass.
SEAT_XY = (
    float(os.environ.get("HAND_TEST_SEAT_X", "-3.337")),
    float(os.environ.get("HAND_TEST_SEAT_Y", "-2.216")),
)
SEAT_YAW_DEGREES = float(os.environ.get("HAND_TEST_SEAT_YAW", "180.0"))
SEAT_TORSO_Z = float(os.environ.get("HAND_TEST_SEAT_Z", "-0.20"))
SEAT_POSITION = Gf.Vec3d(SEAT_XY[0], SEAT_XY[1], SEAT_TORSO_Z)

PERSON_PRIM_PATH = "/World/HandSafetyTestActor"
ARM_RIG_PATH = f"{PERSON_PRIM_PATH}/RightArmRig"
ELBOW_PATH = f"{ARM_RIG_PATH}/ElbowPivot"

# --- Reach timing. The task asks for a reach every 5-10 seconds.
MIN_PERIOD_SECONDS = float(os.environ.get("HAND_TEST_MIN_PERIOD", "5.0"))
MAX_PERIOD_SECONDS = float(os.environ.get("HAND_TEST_MAX_PERIOD", "10.0"))
REACH_TRAVEL_SECONDS = float(os.environ.get("HAND_TEST_TRAVEL_SECONDS", "0.8"))
REACH_HOLD_SECONDS = float(os.environ.get("HAND_TEST_HOLD_SECONDS", "0.4"))
# hand_safety requires confirmation_frames=3 consecutive detections at
# process_rate=30 Hz (~0.1s) before it reports an intrusion -- the hold
# time above must stay comfortably longer than that or every reach will be
# invisible to the detector.

# --- Rig geometry, measured once from the rig asset's own skeleton bind
# pose (male_adult_construction_01_new, joints R_Upperarm/R_Forearm/R_Hand)
# via `skeleton.GetBindTransformsAttr()` in an isolated script -- see build
# notes at the bottom. These are positions relative to the character's own
# root Xform (PERSON_PRIM_PATH), matching how RIG_ASSET_USD's RightArmRig
# is authored (its own translate ops already encode P_UP / P_FORE - P_UP).
P_UP = Gf.Vec3d(-0.19073455810546874, 0.06624944686889649, 1.43966064453125)
P_FORE = Gf.Vec3d(-0.4669504547119141, 0.06760619640350342, 1.4339576721191407)
P_HAND = Gf.Vec3d(-0.6862501525878907, 0.07141963958740234, 1.432750244140625)
U1_REST = P_FORE - P_UP  # shoulder -> elbow, upper-arm bone (~0.276m)
U2_REST = P_HAND - P_FORE  # elbow -> wrist, forearm bone (~0.219m)
_ARM_MAX_REACH = U1_REST.GetLength() + U2_REST.GetLength()

# Pole hint biases the elbow to bend downward/forward, matching how a
# seated person's elbow drops when reaching onto a table in front of them
# rather than winging out to the side. Purely a disambiguator for the IK's
# bend plane; doesn't need to be exact.
POLE_HINT_LOCAL_OFFSET = Gf.Vec3d(0.3, 0.0, -1.0)

_seat_yaw_rotation = Gf.Rotation(Gf.Vec3d(0, 0, 1), SEAT_YAW_DEGREES)


def _solve_static_arm_target():
    """Solve the two-bone IK target once against the STATIC `SEAT_POSITION`
    -- no body motion is involved this pass, unlike pass 5's per-frame lean.
    Returns the IK target expressed in the character's own root frame
    (matching P_UP/U1_REST/U2_REST's frame). Logs the shoulder-to-target
    distance against `_ARM_MAX_REACH` so an unreachable `SEAT_POSITION` is
    obvious at startup instead of silently producing a fully-extended arm
    that stops short of the table.
    """
    shoulder_world = _seat_yaw_rotation.TransformDir(P_UP) + SEAT_POSITION
    distance = (TABLE_HAND_TARGET - shoulder_world).GetLength()
    print(
        f"[hand_test] shoulder->target distance={distance:.3f}m "
        f"(arm max reach={_ARM_MAX_REACH:.3f}m, "
        f"{'REACHABLE' if distance <= _ARM_MAX_REACH else 'OUT OF REACH -- move the seat closer via HAND_TEST_SEAT_X/Y/Z'})",
        flush=True,
    )
    return _seat_yaw_rotation.GetInverse().TransformDir(TABLE_HAND_TARGET - SEAT_POSITION)


def spawn_seated_person(stage):
    """Reference the rigid-arm rig asset at TableSet_00's Chair_01 and
    return its prim. Falls back to the old whole-body-slide mechanism
    (PERSON_USD directly, no rig) when HAND_TEST_RIG_MODE=legacy, in case
    the new rig needs debugging on hardware.
    """
    xform = UsdGeom.Xform.Define(stage, PERSON_PRIM_PATH)
    prim = xform.GetPrim()

    if RIG_MODE == "legacy":
        prim.GetReferences().AddReference(_LEGACY_PERSON_USD)
    else:
        prim.GetReferences().AddReference(RIG_ASSET_USD)
        # mobile_manipulator_demo.py constructs ReachAnimator immediately
        # after this call with no simulation_app.update() in between (the
        # legacy character reference tolerated that; this one references
        # rigid_arm_asset.usda, which itself references the S3 character --
        # a nested reference that isn't always composed by the very next
        # frame). Pump the app until RightArmRig actually exists so
        # ReachAnimator._get_orient_op doesn't run against a stage that
        # hasn't finished loading yet -- hit on hardware as
        # "RuntimeError: rig prim missing: .../RightArmRig".
        app = omni.kit.app.get_app()
        arm_rig_path = f"{PERSON_PRIM_PATH}/RightArmRig"
        for _ in range(300):  # ~5s at 60Hz app update; loads are normally <1s
            if stage.GetPrimAtPath(arm_rig_path).IsValid():
                break
            app.update()
        else:
            raise RuntimeError(
                f"timed out waiting for rig asset to load: {RIG_ASSET_USD} "
                f"({arm_rig_path} never appeared)"
            )

    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    translate_op = xformable.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble)
    translate_op.Set(SEAT_POSITION)
    rotate_op = xformable.AddRotateZOp(precision=UsdGeom.XformOp.PrecisionDouble)
    rotate_op.Set(SEAT_YAW_DEGREES)
    print(
        f"[hand_test] spawned rig={RIG_ASSET_USD if RIG_MODE != 'legacy' else _LEGACY_PERSON_USD} "
        f"(mode={RIG_MODE}) at {PERSON_PRIM_PATH}, "
        f"seat={tuple(SEAT_POSITION)} yaw={SEAT_YAW_DEGREES}",
        flush=True,
    )
    return prim


def _get_orient_op(stage, prim_path: str):
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"rig prim missing: {prim_path} (did the rig asset load?)")
    xformable = UsdGeom.Xformable(prim)
    for op in xformable.GetOrderedXformOps():
        if op.GetOpName() == "xformOp:orient":
            return op
    raise RuntimeError(f"{prim_path} has no xformOp:orient (unexpected rig asset)")


def _smoothstep(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


class ReachAnimator:
    """Drives only the two-bone right-arm rig (shoulder + elbow rotation)
    between rest and an IK-solved reach pose, on a randomized 5-10s period.
    The body/seat never moves this pass -- `SEAT_POSITION` is authored once
    in `spawn_seated_person` and never touched again. Call update() once per
    simulation_app frame; timing is wall-clock (time.time()), matching this
    demo's non-headless real-time frame pacing.
    """

    def __init__(self, prim):
        self.stage = prim.GetStage()
        self.active = False
        self.event_start: float | None = None
        self.next_event_time = time.time() + random.uniform(
            MIN_PERIOD_SECONDS, MAX_PERIOD_SECONDS
        )

        if RIG_MODE == "legacy":
            self._init_legacy(prim)
            return

        self.shoulder_op = _get_orient_op(self.stage, ARM_RIG_PATH)
        self.elbow_op = _get_orient_op(self.stage, ELBOW_PATH)

        arm_target_local = _solve_static_arm_target()
        pole_hint_local = P_UP + POLE_HINT_LOCAL_OFFSET
        self.reach_shoulder_rot, self.reach_elbow_rot, elbow_pos = solve_two_bone_ik(
            P_UP, arm_target_local, U1_REST, U2_REST, pole_hint_local
        )
        print(
            f"[hand_test] static seat={tuple(SEAT_POSITION)}, "
            f"shoulder_angle={self.reach_shoulder_rot.GetAngle():.1f}deg, "
            f"elbow_angle={self.reach_elbow_rot.GetAngle():.1f}deg",
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
        # No body/seat motion this pass -- only the two rotate ops move.
        # Interpolating from the identity rotation to a target rotation R by
        # scaling R's own axis-angle by `progress` IS the slerp from
        # identity to R (there's no "shortest path" ambiguity to resolve
        # when one endpoint is identity), so this needs no quaternion
        # slerp helper.
        shoulder_partial = Gf.Rotation(
            self.reach_shoulder_rot.GetAxis(), self.reach_shoulder_rot.GetAngle() * progress
        )
        elbow_partial = Gf.Rotation(
            self.reach_elbow_rot.GetAxis(), self.reach_elbow_rot.GetAngle() * progress
        )
        self.shoulder_op.Set(Gf.Quatd(shoulder_partial.GetQuat()))
        self.elbow_op.Set(Gf.Quatd(elbow_partial.GetQuat()))

    def update(self) -> None:
        if RIG_MODE == "legacy":
            self._update_legacy()
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

    # ------------------------------------------------------------------
    # Legacy whole-body-slide fallback (pass #1-4 mechanism), kept only for
    # A/B debugging against the new rig on hardware.
    # ------------------------------------------------------------------
    def _init_legacy(self, prim):
        self.translate_op = UsdGeom.Xformable(prim).GetOrderedXformOps()[0]
        yaw = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), SEAT_YAW_DEGREES)
        world_offset = yaw.TransformDir(P_HAND)
        full_reach_target = TABLE_HAND_TARGET - world_offset
        _seat_to_table = full_reach_target - SEAT_POSITION
        length = _seat_to_table.GetLength()
        lean_distance = float(os.environ.get("HAND_TEST_LEAN_DISTANCE", "0.15"))
        self.reach_target = (
            SEAT_POSITION + _seat_to_table * (lean_distance / length)
            if length > 1e-6
            else SEAT_POSITION
        )

    def _update_legacy(self):
        now = time.time()
        if not self.active:
            if now < self.next_event_time:
                return
            self.active = True
            self.event_start = now
        elapsed = now - self.event_start
        total_duration = 2.0 * REACH_TRAVEL_SECONDS + REACH_HOLD_SECONDS
        if elapsed >= total_duration:
            self.active = False
            self.translate_op.Set(SEAT_POSITION)
            self.next_event_time = now + random.uniform(MIN_PERIOD_SECONDS, MAX_PERIOD_SECONDS)
            return
        progress = self._cycle_progress(elapsed)
        position = SEAT_POSITION + (self.reach_target - SEAT_POSITION) * progress
        self.translate_op.Set(position)


_LEGACY_PERSON_USD = os.environ.get(
    "HAND_TEST_PERSON_USD",
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/5.1/Isaac/People/Characters/"
    "male_adult_construction_01_new/male_adult_construction_01_new.usd",
)


# ----------------------------------------------------------------------
# Asset build notes (how RIG_ASSET_USD / assets/rigid_arm_asset.usda was
# made -- for reference if it ever needs regenerating, e.g. for a
# different character). Built and verified entirely offline in a CPU-only
# environment using the standalone `usd-core` pip package (no Isaac Sim
# needed for this part -- Isaac Sim is only needed to actually render it):
#
# 1. Downloaded male_adult_construction_01_new.usd and opened it with
#    pxr.Usd.Stage.Open. It's fully self-contained (no external texture/
#    geometry references), one Skeleton (101 joints, Reallusion-style rig)
#    and several skinned Mesh prims (skin, vest, tshirt, gloves, ...).
# 2. Read each mesh's `primvars:skel:jointIndices`/`jointWeights` and, per
#    vertex, took the highest-weight joint as that vertex's "dominant"
#    joint. Bucketed joints into upperarm={R_Upperarm,R_UpperarmTwist01/2}
#    and forearm_hand={R_Forearm,R_ElbowShareBone,R_ForearmTwist01/2,
#    R_Hand,+all finger joints} by checking path *components* (careful:
#    joint paths encode the full ancestor chain, so a naive substring/
#    regex match on leaf names alone false-positives on toe joints like
#    R_PinkyToe1 -- match on whole path segments instead).
#  - skin: 51 upperarm verts / 83 forearm_hand verts / 1542 other (of 1676)
#  - vest: 0 / 0 / 979 (sleeveless, doesn't touch the arm at all)
#  - tshirt: 125 / 0 / 671 (short sleeve covers upper arm, not forearm)
#  - gloves: 0 / 521 / 519 (mesh contains both hands; ~half each)
# 3. For each mesh, split faces into three sets by requiring ALL of a
#    face's vertices share the category (faces straddling a boundary are
#    dropped from every set -- leaves small seams at the shoulder/elbow,
#    acceptable for a test prop). Points/normals arrays are kept at full
#    size (unused entries are harmless) to avoid index remapping entirely.
# 4. Authored a new stage that (a) references the original character file
#    (for materials + the untouched meshes) and adds `over` opinions on
#    skin/tshirt/gloves overriding just faceVertexCounts/faceVertexIndices/
#    primvars:st0:indices to the "other" (non-right-arm) face set -- this
#    removes the original arm from the body without touching anything
#    else (skinning API, materials, other meshes are all inherited as-is,
#    and since the skeleton itself is never animated, its live UsdSkel
#    binding on the remaining meshes is a harmless no-op); and (b) defines
#    new plain (non-skinned) Mesh prims for the upperarm/forearm_hand face
#    sets, with points *recentered* to their own pivot (subtract P_UP for
#    the upperarm piece, P_FORE for the forearm_hand piece) under a new
#    `RightArmRig/ElbowPivot` Xform chain positioned at those same pivots.
# 5. Verified by computing each new prim's ComputeLocalToWorldTransform at
#    rest (identity rotation) and confirming it reconstructs the ORIGINAL
#    mesh's point positions exactly (sub-1e-7 error, float32 rounding
#    only) -- proves the pivot/recenter math has no sign or axis errors.
# 6. The two-bone IK (`two_bone_ik.solve_two_bone_ik`) was unit-tested the
#    same way: built a throwaway in-memory stage with the identical
#    Shoulder->Elbow->hand-marker hierarchy, solved 20 random reachable
#    targets, and confirmed the marker lands on each target to float
#    precision (~1e-16). The one subtlety that failed on the first attempt
#    and is worth knowing if this ever needs re-deriving: the elbow's own
#    xformOp:orient is a LOCAL rotation, so the world-space target
#    direction has to be transformed by the shoulder rotation's *inverse*
#    before solving "rotate the forearm's rest direction onto it" --
#    using the raw world-space direction directly (skipping that inverse)
#    gives a plausible-looking but wrong pose.
# 7. Pass 5 prep's first cut held the body fully fixed at SEAT_POSITION and
#    only rotated the arm -- verification caught that Chair_01 sits ~1.4m
#    from TABLE_HAND_TARGET (measured from the actual shoulder position,
#    not the root) while this character's own two-bone reach is only
#    ~0.5m, so an arm-only reach cannot possibly work here regardless of
#    IK correctness. Pass 5's shipped version instead leaned the body (a
#    `translateOp` animated per reach) to close most of that gap, letting
#    the arm cover only the last ~15%. Verified on hardware in pass 5 (see
#    GPU_RUN_LOG.txt): the two-bone mechanism itself rendered and detected
#    correctly, but the body still visibly moved every reach, which pass 6
#    was asked to eliminate.
# 8. Pass 6: removed the per-reach body lean entirely and moved the SEAT
#    itself (SEAT_XY/SEAT_TORSO_Z) close enough that a static arm-only pose
#    reaches the target -- `_solve_static_arm_target()` solves the IK once
#    against the fixed SEAT_POSITION, `ReachAnimator` never touches a
#    translateOp for `rigid_arm` mode. Re-verified end to end against a
#    synthetic hand marker exactly as pass 5 prep did: lands on
#    TABLE_HAND_TARGET to ~1e-16 error at 113.5 deg shoulder / 64.4 deg
#    elbow, using the shipped SEAT_POSITION default. Found and documented a
#    hard geometric limit while deriving that default (see the module
#    docstring's "IMPORTANT" paragraph): this standing-pose character's
#    shoulder sits 1.44m up regardless of seat X/Y, so the 0.61m vertical
#    gap to the 0.83m target alone exceeds the arm's 0.496m max reach --
#    moving the seat's X/Y can only ever close the *horizontal* component,
#    never that vertical one, so some `SEAT_TORSO_Z` sink is unavoidable
#    for a fully static pose with this character. Chose a modest -0.20m
#    (vs. pass 5's worst-case dynamic -0.427m) plus enough of an X/Y move
#    that the seat now sits close to the table's own footprint -- NOT
#    verified against the actual TableSet_00/Chair_01 mesh (no Isaac Sim
#    access this pass either), flagged prominently for hardware follow-up.
#
# What is NOT verified (needs Isaac Sim / a GPU): whether the moved seat
# position visually clips into the table, chairs, or looks out of place;
# whether the fully-static body (no lean at all now) still looks natural at
# a permanent -0.20m sink versus pass 5's brief dynamic one; and whether
# YOLO's hand-detection confidence/ROI gating (both already confirmed good
# in pass 5 on the rig mechanism itself) are unaffected by the new seat
# position. Set HAND_TEST_RIG_MODE=legacy to compare directly against the
# original whole-body mechanism if this pass's static pose looks wrong.
# ----------------------------------------------------------------------
