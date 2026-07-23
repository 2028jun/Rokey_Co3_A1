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

Pass #5 (this one) exploits that: it extracts the character's own right
upper-arm and forearm+hand geometry (skin + sleeve + glove meshes, split by
which skeleton joint dominantly influences each vertex) out of the skinned
mesh entirely and re-parents those pieces under a plain two-bone `Xform`
chain (`RightArmRig -> ElbowPivot`) that this script drives directly with
`xformOp:orient` quaternions -- no UsdSkel, no Animation Graph, so neither
of the previously-hit rendering bugs apply. The rest of the body (the
original skinned character, with the right arm's faces removed so nothing
overlaps) stays parented under the same seat translate as before.

IMPORTANT: the chair (Chair_01, ~(-2.7,-3.2,0)) sits about 1.4m from
TABLE_HAND_TARGET in a straight line, and this character's own two-bone arm
(upper arm + forearm, measured from its own bind pose) only spans ~0.5m
fully extended. A real arm cannot cover a 1.4m gap by rotating at a fixed
shoulder -- pass #1's whole-body slide existed for exactly this reason, and
that physical constraint doesn't go away just because the arm can now bend.
This pass therefore still leans the whole body most of the way (using the
same translateOp pass #1 already had), but only as far as leaving the last
~85% of the arm's own reach to close -- so the elbow visibly bends into a
natural-looking reach for the final stretch instead of the arm staying
rigidly glued to the side while the whole body does 100% of the work. The
lean is computed from the actual SHOULDER position (seat + yawed P_UP), not
the character root, and still needs to bring the root below its rest Z
(the target sits well below this standing character's shoulder height) --
same "sinks toward the table" characteristic pass #1 already documented,
just less pronounced and now paired with a genuine elbow bend.

The two-bone shoulder/elbow angles and the lean distance are solved
analytically each reach (`two_bone_ik.solve_two_bone_ik`), not
baked/measured by hand, so this generalizes to any target position on the
table without re-tuning. The solver, the extraction pivots, and the full
lean+arm pipeline were unit-verified against the actual mesh data (see the
asset-build notes at the bottom of this file) with sub-micron
reconstruction error; what has NOT been verified is how it actually
*renders* -- this pass was written and tested in a CPU-only environment
without Isaac Sim. Confirm the reach visually before trusting this over the
old whole-body version (env var `HAND_TEST_RIG_MODE=legacy` switches back
to it).

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
RIG_ASSET_USD = os.environ.get(
    "HAND_TEST_RIG_ASSET",
    "assets/rigid_arm_asset.usda",
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

# Chair_01_Visual under TableSet_00: translate=(-2.7, -3.2, 0), rotateZ=180
# (facing the table).
SEAT_XY = (
    float(os.environ.get("HAND_TEST_SEAT_X", "-2.7")),
    float(os.environ.get("HAND_TEST_SEAT_Y", "-3.2")),
)
SEAT_YAW_DEGREES = float(os.environ.get("HAND_TEST_SEAT_YAW", "180.0"))
SEAT_TORSO_Z = float(os.environ.get("HAND_TEST_SEAT_Z", "0.0"))
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

# How much of the arm's own max reach to use for the final stretch (leaves
# some elbow bend rather than locking the arm straight at 100%); the rest
# of the seat-to-target gap is covered by the body lean below. Purely a
# look/feel knob -- lower values lean the body more and bend the arm less.
REACH_EXTENSION_RATIO = float(os.environ.get("HAND_TEST_REACH_EXTENSION_RATIO", "0.85"))

# Pole hint biases the elbow to bend downward/forward, matching how a
# seated person's elbow drops when reaching onto a table in front of them
# rather than winging out to the side. Purely a disambiguator for the IK's
# bend plane; doesn't need to be exact.
POLE_HINT_LOCAL_OFFSET = Gf.Vec3d(0.3, 0.0, -1.0)

_seat_yaw_rotation = Gf.Rotation(Gf.Vec3d(0, 0, 1), SEAT_YAW_DEGREES)


def _compute_lean_and_arm_target():
    """Split the seat-to-target gap between a body lean (this actor's own
    translateOp, same mechanism pass #1 used) and the two-bone arm's own
    reach, so the arm ends up genuinely bent rather than fully extended or
    doing nothing. Returns (lean_target_world, arm_target_local) -- the
    person-root translate value at full reach progress, and the IK target
    for the arm expressed in the character's own root frame (matching
    P_UP/U1_REST/U2_REST's frame).
    """
    shoulder_world_at_rest = _seat_yaw_rotation.TransformDir(P_UP) + SEAT_POSITION
    d_vec = TABLE_HAND_TARGET - shoulder_world_at_rest
    d_total = d_vec.GetLength()
    target_arm_reach = REACH_EXTENSION_RATIO * _ARM_MAX_REACH
    lean_distance = max(0.0, d_total - target_arm_reach)
    lean_dir_world = d_vec.GetNormalized() if d_total > 1e-9 else Gf.Vec3d(0, 0, 0)
    lean_target_world = SEAT_POSITION + lean_dir_world * lean_distance

    arm_target_local = _seat_yaw_rotation.GetInverse().TransformDir(
        TABLE_HAND_TARGET - lean_target_world
    )
    return lean_target_world, arm_target_local


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
    """Drives the body lean (person root translateOp) and the two-bone
    right-arm rig together, between rest and an IK-solved reach pose, on a
    randomized 5-10s period. Call update() once per simulation_app frame;
    timing is wall-clock (time.time()), matching this demo's non-headless
    real-time frame pacing.
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

        self.translate_op = UsdGeom.Xformable(prim).GetOrderedXformOps()[0]
        self.shoulder_op = _get_orient_op(self.stage, ARM_RIG_PATH)
        self.elbow_op = _get_orient_op(self.stage, ELBOW_PATH)

        self.lean_target_world, arm_target_local = _compute_lean_and_arm_target()
        pole_hint_local = P_UP + POLE_HINT_LOCAL_OFFSET
        self.reach_shoulder_rot, self.reach_elbow_rot, elbow_pos = solve_two_bone_ik(
            P_UP, arm_target_local, U1_REST, U2_REST, pole_hint_local
        )
        print(
            f"[hand_test] lean target={tuple(self.lean_target_world)} "
            f"(seat={tuple(SEAT_POSITION)}), "
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
        position = SEAT_POSITION + (self.lean_target_world - SEAT_POSITION) * progress
        self.translate_op.Set(position)

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
# 7. First cut of this pass held the body fully fixed at SEAT_POSITION and
#    only rotated the arm -- verification caught that Chair_01 sits ~1.4m
#    from TABLE_HAND_TARGET (measured from the actual shoulder position,
#    not the root) while this character's own two-bone reach is only
#    ~0.5m, so an arm-only reach cannot possibly work here regardless of
#    IK correctness. `_compute_lean_and_arm_target()` now splits the gap:
#    body leans until only REACH_EXTENSION_RATIO (0.85) of the arm's max
#    reach remains, and the arm covers that remainder. Full pipeline
#    (lean translate + shoulder rotate + elbow rotate, exactly as
#    authored in spawn_seated_person/ReachAnimator) was re-verified to
#    land the hand-marker on TABLE_HAND_TARGET to float precision.
#
# What is NOT verified (needs Isaac Sim / a GPU): whether the extracted
# geometry actually looks right when rendered (seams at the shoulder/elbow
# boundary, whether the removed-face gap in the body is visible, whether
# YOLO's hand-detection confidence on the re-parented glove mesh matches
# the ~0.88 seen on the original skinned version, and how the residual
# body lean/sink -- still ~0.7-1.0m depending on REACH_EXTENSION_RATIO --
# actually looks in motion). Set HAND_TEST_RIG_MODE=legacy to compare
# directly against the old whole-body mechanism if the new rig looks wrong.
# ----------------------------------------------------------------------
