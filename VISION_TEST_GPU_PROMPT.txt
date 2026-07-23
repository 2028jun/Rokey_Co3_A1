# Vision-test GPU execution prompt: verify the rigid-arm reach rig end to end

Paste this document into a coding-agent session running on the GPU computer.
The target environment is Isaac Sim 5.1.0-rc.19, ROS 2 Humble, and an RTX
5080 with 16 GB VRAM. Work on the `vision-test` branch.

Read `GPU_RUN_LOG.txt` before changing anything, especially the "Pass 5 prep"
entry at the bottom -- it explains exactly what was built and why, and what
is and is not already verified. Git history preserves the old Pass 1-4
implementation attempts and this prompt's own prior revisions; this prompt
contains only the current work. Do not copy old pass narratives back into it.

## Repository bootstrap

Before working, run `git status` and preserve any existing uncommitted GPU-side
work; never discard it with reset, checkout, or deletion. Fetch `origin`, switch
to `vision-test`, and update it with `git pull --ff-only origin vision-test`.
If local work prevents a fast-forward, integrate it safely instead of
overwriting it. Then read this prompt and `GPU_RUN_LOG.txt` completely and
execute the work end to end. Do not stop after planning or after one test case.

When finished, append a "Pass 5" entry to `GPU_RUN_LOG.txt` (below the "Pass 5
prep" entry already there -- do not overwrite or renumber it), commit the
tested code and log, push `vision-test`, and report the commit hash, changed
files, measured results, and remaining limitations.

## Objective

A prior CPU-only session (no Isaac Sim, no GPU -- see "Pass 5 prep" in
`GPU_RUN_LOG.txt`) replaced the whole-body-slide reach with a rigid,
non-skinned two-bone arm (`isaacpjt/two_bone_ik.py` +
`assets/rigid_arm_asset.usda`) extracted from the same
`male_adult_construction_01_new` character's own mesh data, driven by
`xformOp:orient` on a plain `Xform` hierarchy (no `UsdSkel`, no Animation
Graph -- both were already proven broken on this build in passes 2 and 4).
The geometry extraction and the IK math were unit-verified offline against
the actual mesh/skeleton data (sub-micron reconstruction error) and a crude
offline wireframe preview looked structurally correct, but **none of it has
been rendered by Isaac Sim or tested against the real YOLO detector yet**.
This pass is that first real hardware verification:

1. Load `assets/rigid_arm_asset.usda` and confirm the geometry, materials,
   and the `RightArmRig`/`ElbowPivot` hierarchy actually render correctly
   (no missing textures, no broken normals, no unexpected geometry).
2. Run the full demo with the new rig (`HAND_TEST_RIG_MODE=rigid_arm`, the
   default) and confirm the reach looks like a plausible arm-driven motion:
   shoulder rotates, elbow visibly bends, hand/glove enters the tabletop
   ROI -- not a repeat of the old whole-torso slide.
3. Confirm `hand_detector_node` still detects the hand on the new rig with
   confidence comparable to the ~0.88-0.89 seen on this same character's
   original (skinned) glove in pass 4, and that `/hand_safety/roi_intrusion`
   still gates correctly (false at rest, true only during a reach hold, RG2
   gripper false positive still excluded).
4. Fix whatever the hardware reveals -- seams at the shoulder/elbow
   boundary, visible gaps in the body where the original arm was removed,
   an unnatural bend direction, or a body lean that looks worse than the
   old whole-body slide -- and record what was wrong and what fixed it.

Keep the current ROS domain configuration and the 1280x960 camera input.
Those are working and out of scope for this pass.

## Known results: do not repeat these dead ends

- Pass 2: late-bound `pxr.UsdSkel`/`usdrt.UsdSkel` joint-rotation edits changed
  `SkelQuery` state but never changed rendered pixels on this exact Isaac Sim
  build. Do not repeat that experiment -- this is exactly why the new rig
  uses a plain non-skinned `Xform` chain instead.
- Pass 4: the officially-documented Animation Graph / Actor Control route
  (`push_button.skelanim.usd` etc.) segfaults headless on this build before
  any Animation-Graph-specific API is even called. Not fixable from a
  standalone script. Do not repeat it.
- Pass 3: the RG2 gripper false positive was fixed by tightening the table
  ROI polygon (`hand_safety/roi_intrusion.py`), not by repainting the
  gripper (tried and failed). Do not revert the tightened ROI.
- Pass 4 picked `male_adult_construction_01_new` (~0.88-0.89 YOLO confidence,
  light work glove) over `F_Business_02` (~0.52) and `F_Medical_01` (~0.82).
  The new rig extracts geometry from this same character for that reason;
  it is not re-comparing characters this pass.
- Pass 5 prep (CPU-only, see log): a first draft of the reach held the
  torso completely fixed and only rotated the arm. Verification caught that
  Chair_01 sits about 1.4 m from `TABLE_HAND_TARGET` (measured from the
  actual shoulder position) while this character's own two-bone arm is only
  about 0.5 m end to end -- an arm-only reach is geometrically impossible
  here regardless of IK correctness. The shipped version leans the body
  (same `translateOp` mechanism pass #1 already used) only as far as
  leaving `HAND_TEST_REACH_EXTENSION_RATIO` (default 0.85) of the arm's own
  reach to close, then bends the elbow for the rest. The residual lean is
  still large (roughly 0.7-1.0 m depending on the ratio) and still pushes
  the character root below its rest Z, same "sinks toward the table"
  characteristic pass #1 documented -- this is a physical consequence of
  the chair/table distance and this character's arm length, not a bug to
  re-fix by changing the IK. If it looks worse than expected on hardware,
  tune `HAND_TEST_REACH_EXTENSION_RATIO` (lower = more arm bend, more lean;
  higher = straighter arm, less lean) rather than re-deriving the math.

## Relevant scene and pipeline facts

- The robot docks at `TableSet_00`; this is the table framed by the fixed
  table camera and treated as table 1 in this project.
- `TableSet_00` tabletop world height is `TABLE_TOP_Z = 0.73 m`; the hand
  target is `TABLE_TOP_Z + HAND_TARGET_Z_OFFSET` (0.83 m by default, set in
  pass 4 and unchanged this pass).
- Hand inference is gated by `/serving_robot/table_arrived` and requires
  three consecutive in-ROI detections by default.
- Main files:
  - `isaacpjt/mobile_manipulator_demo.py` (spawns the actor, drives the
    per-frame `ReachAnimator.update()` loop -- unchanged this pass)
  - `isaacpjt/hand_intrusion_test_actor.py` (rewritten this pass -- see its
    own module docstring and the "Asset build notes" at the bottom for the
    full mechanism and the exact math used)
  - `isaacpjt/two_bone_ik.py` (new this pass -- the two-bone IK solver,
    unit-verified offline; read its use in `hand_intrusion_test_actor.py`
    before touching it)
  - `assets/rigid_arm_asset.usda` (new this pass -- the extracted-arm rig
    asset; references the original `male_adult_construction_01_new.usd`
    from the standard Isaac content S3 bucket for materials and the
    untouched body, so it needs the same network/asset-root access the
    existing character reference already relies on)
  - `hand_safety/hand_safety/hand_detector_node.py`
  - `hand_safety/hand_safety/roi_intrusion.py`
  - `hand_safety/config/hand_safety.yaml`
- `HAND_TEST_RIG_MODE=legacy` switches `hand_intrusion_test_actor.py` back
  to the old whole-body-slide mechanism (pass #1-4) with no other changes,
  for direct A/B comparison if the new rig looks wrong.

## Task 1: confirm the rig asset loads and renders correctly

Before running the full demo, load `assets/rigid_arm_asset.usda` alone (e.g.
reference it onto a throwaway prim in a script-editor session) and inspect
it directly:

1. Confirm the reference to the original character resolves (materials,
   the untouched body meshes with the right arm's faces already removed).
2. Confirm `RightArmRig` and `RightArmRig/ElbowPivot` exist with the
   expected meshes (`UpperArm_Skin`, `UpperArm_Tshirt`, `ForearmHand_Skin`,
   `ForearmHand_Glove`) and that they render with the right materials
   (skin tone, sleeve color, glove color -- not missing/pink/default).
3. Look closely at the shoulder and elbow boundaries for visible gaps or
   z-fighting where the original body mesh was cut and the new piece was
   inserted. The offline build notes in `hand_intrusion_test_actor.py`
   already document that boundary-straddling faces were dropped on both
   sides, so *some* seam is expected -- judge whether it's acceptable at
   the table-camera's actual distance/resolution, not whether it's
   perfectly invisible up close.

## Task 2: run the reach and judge the motion

Run the full demo (`HAND_TEST_RIG_MODE=rigid_arm`, the default) and watch at
least 3-4 reach cycles from more than one camera angle (the fixed table
camera plus a free viewport).

1. Confirm the shoulder visibly rotates and the elbow visibly bends during
   the reach -- this is the entire point of this pass, contrast it
   directly against `HAND_TEST_RIG_MODE=legacy` if it's not obviously
   better.
2. Confirm the hand/glove enters the tabletop ROI without clipping through
   the table (same 0.83 m clearance target as pass 4; do not stack another
   offset -- adjust `HAND_TARGET_Z_OFFSET` only if the target itself is
   wrong, not the rig).
3. If the body lean/sink looks worse than acceptable, try 2-3 values of
   `HAND_TEST_REACH_EXTENSION_RATIO` (e.g. 0.7, 0.85, 0.95) and record which
   looks best, rather than editing the IK/lean formula itself.
4. If the elbow bends the wrong way (e.g. sideways instead of down/forward),
   that is controlled by `POLE_HINT_LOCAL_OFFSET` in
   `hand_intrusion_test_actor.py` (not currently an env-var override --
   add one if it needs iterating on hardware).

## Task 3: confirm hand detection still works on the new rig

With `hand_safety` running against the live table camera:

1. Confirm `/hand_detection/detections` reports a hand on the new rig's
   glove during a reach hold, and record the confidence across a few
   cycles -- compare against pass 4's ~0.88-0.89 on this same character's
   original skinned glove. A meaningfully lower confidence likely means the
   extracted glove geometry/material isn't matching closely enough
   (check UV/material binding first, per Task 1).
2. Confirm `/hand_safety/roi_intrusion` alternates correctly: false at
   rest for at least 20s, true only during reach holds, and that the RG2
   gripper false positive (fixed in pass 3) has not returned.
3. Confirm `table_arrived` gating still suppresses detection when the
   robot hasn't docked.

## Out of scope this pass

- The ~5 FPS RGB/detection performance bottleneck raised in an earlier
  prompt revision was never actually investigated (no entry in
  `GPU_RUN_LOG.txt` mentions it) and remains open, but is unrelated to this
  rig change -- do not fold it into this pass; flag it as still open in the
  Pass 5 log entry if it's still reproducible, but don't spend the pass on
  it.
- Comparing additional People characters -- pass 4 already picked a winner
  and the new rig is built specifically from that character's mesh data.

## Build and run

From the repository root:

```bash
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=101
MOBILE_DEMO_HAND_TEST=1 python isaacpjt/mobile_manipulator_demo.py
```

In another terminal, use the same ROS domain and run hand safety with explicit
parameters. Enable annotated output only for visual validation.

Useful checks:

```bash
ros2 topic echo /serving_robot/table_arrived
ros2 topic echo /hand_safety/roi_intrusion
ros2 topic echo /hand_detection/detections
ros2 topic hz /serving_robot/table_camera/color/image_raw
ros2 run rqt_image_view rqt_image_view /hand_detection/image
```

A/B against the old mechanism:

```bash
HAND_TEST_RIG_MODE=legacy MOBILE_DEMO_HAND_TEST=1 python isaacpjt/mobile_manipulator_demo.py
```

## Acceptance criteria

- `assets/rigid_arm_asset.usda` loads with no missing materials/textures
  and no unexpected geometry.
- The reach is visibly arm-driven (shoulder + elbow rotate) and not a
  repeat of the whole-torso-slide as the primary motion.
- The hand clears the tabletop at the existing 0.83 m target with no
  duplicate offset introduced.
- `/hand_detection/detections` confidence on the new rig is in the same
  ballpark as pass 4's ~0.88-0.89 on the original skinned character (report
  the actual number either way).
- At rest for at least 20 seconds, `/hand_safety/roi_intrusion` remains
  false and the RG2 false positive remains excluded.
- Across multiple reach cycles, intrusion becomes true in sync with the
  rendered hand entering the ROI and returns false after retraction.
- Camera input remains 1280x960 and `table_arrived` gating still works.

## Deliverables

Append one dated "Pass 5" entry to `GPU_RUN_LOG.txt` (after the existing
"Pass 5 prep" entry -- do not edit that one except to fix a factual error)
containing:

- whether the asset loaded cleanly and what, if anything, needed fixing;
- how the reach looked (shoulder/elbow motion, seams, lean/sink severity,
  and the `HAND_TEST_REACH_EXTENSION_RATIO`/pole-hint values settled on);
- detection confidence numbers and intrusion timing, compared explicitly
  against pass 4's baseline;
- any code changes made and why; and
- remaining limitations, including whether `HAND_TEST_RIG_MODE=legacy` is
  still needed as a fallback or the new rig can become the sole mechanism.

Keep implementation changes focused and retain useful environment/config
overrides.
