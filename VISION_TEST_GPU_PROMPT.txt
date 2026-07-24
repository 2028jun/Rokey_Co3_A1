# Vision-test GPU prompt: confirm actor_sdg pose quality, then finish the loop

Paste this entire document into a coding-agent session on the GPU machine.
Target: Isaac Sim 5.1.0-rc.19, ROS 2 Humble, RTX 5080, branch `vision-test`,
`ROS_DOMAIN_ID=102` (matches `AGENTS.md`).

This prompt is self-contained: reading it plus `AGENTS.md`, `GPU_RUN_LOG.txt`,
`ANIM_SPIKE_RESULTS.txt`, and `isaacpjt/actor_sdg_test_actor.py` is enough to
do the whole task and finish it, including pushing the result back to
`vision-test`. Do not wait for further instructions mid-task, and do not stop
after implementing -- run it, look at the rendered frames and real topic
output, and finish per "Finishing" below.

## Bootstrap (do this first, every time)

- `git status`, then `git fetch origin` and `git pull --ff-only origin
  vision-test`. Never discard existing work with reset/checkout/clean.
- Read `AGENTS.md`, `GPU_RUN_LOG.txt` (all passes, especially 9 and 10),
  `ANIM_SPIKE_RESULTS.txt`, `isaacpjt/actor_sdg_test_actor.py`, and
  `isaacpjt/mobile_manipulator_demo.py`'s `main()`, `capture_actor_sdg_
  frames()`, and `enable_extensions()` docstring completely before changing
  anything. Confirm for yourself that `HAND_TEST_RIG_MODE` still defaults
  to `"rigid_arm"` -- pass 10 fixed the registration blocker that made
  `actor_sdg` non-functional, but deliberately did not flip the default
  because visual pose quality still isn't confirmed (see below). If that
  has changed by the time you read this, the direction below still
  applies, just adjust the starting point accordingly.

## Current state (do not re-litigate)

Passes 9-10 replaced `hand_intrusion_test_actor.py`'s rigid-arm two-bone-IK
mechanism with a real `omni.anim.people` character (a
typing -> sit -> push_button -> sit loop at a `TableSet_00` chair), per the
reviewer's call to discard rigid-arm. As of pass 10:

- The mechanism WORKS end to end in the real restaurant+robot pipeline:
  `ag.get_character()` registers the character reliably (confirmed over
  35-42 second runs, zero crashes), and the real `CharacterBehavior`
  command queue executes and loops (`type_keyboard -> Sit -> push_button ->
  Sit -> GoTo`, then repeats via `NUMBER_OF_LOOP=inf`) -- confirmed by
  reading the live instance's own `.commands` list at each checkpoint, not
  assumed.
- The fix was a settle gap: `enable_extensions()` must be called, then
  several `simulation_app.update()` calls must run, THEN the stage opens.
  Getting either half of that wrong breaks it (segfault if the gap is
  missing, or -- pass 9's original mistake -- permanent silent
  registration failure if the extensions are enabled after the stage is
  already open). See `enable_extensions()`'s docstring for the full
  account.
- Visual pose quality is NOT yet confirmed. Pass 10's own ad hoc QA
  cameras (fixed at the character's spawn point) became unreliable once
  the character moved -- wall clipping, chair occlusion, and one
  overhead frame at a loop restart that looked like a flattened/collapsed
  silhouette but could just as easily be a foreshortened top-down view of
  a normal forward-lean typing pose. Not distinguishable from what pass 10
  captured.
- Pass 10 also found a concrete, likely-related lead: the framework's own
  auto-appended loop-closing `GoTo` command's rotation (read back via
  `Utils.convert_to_angle()`) came out to roughly -90 degrees although
  `actor_sdg_test_actor.STAND_YAW_DEGREES` is 0 -- a real yaw-convention
  mismatch, not confirmed as the cause of the pose-quality question above
  but the most likely lead.

## Required work

1. **Build a QA camera that tracks the character**, not a fixed point.
   Query the character prim's live world position/orientation each
   capture (the same way `_find_hand_joint_world_positions()` already
   queries skeleton joints) and aim the camera at that, with enough
   standoff distance to avoid wall/furniture clipping regardless of where
   in the restaurant the character currently is. Verify this camera itself
   works by capturing the character clearly at several different points in
   the cycle before trusting it for anything else.
2. **Investigate the ~90 degree yaw mismatch** pass 10 found. Determine
   whether it's a real bug (fix it -- likely means adjusting
   `STAND_YAW_DEGREES` to compensate for the character asset's actual
   skeleton-forward axis, or finding the correct way to read/set this
   character's orientation) or a red herring, with evidence either way.
3. **Confirm pose quality by eye** with the fixed camera: rest, typing,
   sitting, and push_button should all look like a natural person, not a
   T-pose, not collapsed/clipping into the floor or furniture, through at
   least one full loop (type -> sit -> push -> sit -> repeat).
4. **Only after 1-3 hold**, flip `HAND_TEST_RIG_MODE`'s default from
   `"rigid_arm"` to `"actor_sdg"` in `mobile_manipulator_demo.py`.
5. **Adaptive walk-in (goal 3 from pass 9's original scope).** Project the
   real hand/glove skeleton joint's world position through the table
   camera's intrinsics/extrinsics (`_find_hand_joint_world_positions()` +
   `_project_to_normalized()` already exist for this in
   `mobile_manipulator_demo.py` -- reuse them). For whichever of
   `typing`/`push_button` does NOT bring the hand into
   `TABLE_ROI_NORMALIZED` (`hand_safety/hand_safety/roi_intrusion.py`),
   use `actor_sdg_test_actor.py`'s existing `HAND_TEST_WALK_IN_BEFORE`/
   `HAND_TEST_WALK_IN_X`/`HAND_TEST_WALK_IN_Y` mechanism (already wired,
   never yet exercised with real measurements) to insert a `GoTo` before
   that behavior. Show the before/after projected numbers.
6. **Determine and report which is true, with real evidence (goal 4):**
   either (a) `hand_safety`'s YOLO-based detector correctly reports the
   hand whenever it's genuinely inside the ROI, confirmed by annotated
   frames plus `/hand_safety/roi_intrusion: true` pulses lined up with the
   right behavior states; or (b) the hand is geometrically inside the ROI
   but `hand_safety` does not report it -- say so plainly with the
   frame(s)/topic echo proving the geometric overlap and the absence of a
   detection. Do not assume (a) without checking.

## Iterate until it actually meets the criteria

A single attempt that "runs" is not the deliverable. If a fix doesn't
visually and numerically satisfy the criteria above, keep adjusting and
re-render/re-test until it genuinely does -- not until the code merely
executes without error.

Never paper over a real defect to make it look finished: don't flip
`HAND_TEST_RIG_MODE`'s default without actually confirming pose quality by
eye, don't lower YOLO confidence or move/shrink the ROI without a measured,
camera-projected reason, and don't claim a criterion is met without an
actual rendered frame or real topic echo proving it. Goal 4 explicitly
allows -- requires -- reporting a genuine negative result if that's what
you find; do not convert it into a false pass.

## Finishing (required every time, no exceptions)

1. Append one dated pass entry to `GPU_RUN_LOG.txt` -- what you tried, what
   you observed, what's still open. Do not rewrite prior entries.
2. Rewrite this prompt file (both `VISION_TEST_GPU_PROMPT.txt` and
   `docs/VISION_TEST_GPU_PROMPT.md`, they must stay identical) to describe
   only whatever work genuinely remains. Do not leave finished tasks as
   instructions for the next run.
3. Commit and push the tested code, asset, and log changes to
   `vision-test`. This step is part of the task, not optional cleanup -- a
   run that ends without a pushed commit is not finished.
4. Report the commit hash, changed files, exact run commands, and which of
   the required-work items above are now met versus still open -- including
   whether `HAND_TEST_RIG_MODE`'s default was flipped and why/why not.
