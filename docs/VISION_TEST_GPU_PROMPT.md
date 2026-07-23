# Vision-test GPU prompt: replace hand-built arm rig with Actor SDG animation

Paste this entire document into a coding-agent session on the GPU machine.
Target: Isaac Sim 5.1.0-rc.19, ROS 2 Humble, RTX 5080, branch `vision-test`,
`ROS_DOMAIN_ID=102` (matches `AGENTS.md`).

This prompt is self-contained: reading it plus `AGENTS.md`, `GPU_RUN_LOG.txt`,
and `ANIM_SPIKE_RESULTS.txt` is enough to do the whole task and finish it,
including pushing the result back to `vision-test`. Do not wait for further
instructions mid-task, and do not stop after implementing -- run it, look at
the rendered frames and real topic output, and finish per "Finishing" below.

## Bootstrap (do this first, every time)

- `git status`, then `git fetch origin` and `git pull --ff-only origin
  vision-test`. Never discard existing work with reset/checkout/clean.
- Read `AGENTS.md`, `GPU_RUN_LOG.txt` (passes 1-8), `ANIM_SPIKE_RESULTS.txt`,
  and `isaacpjt/anim_spike_test.py` completely before changing anything. The
  spike already proved the API chain works headless with no crash -- reuse
  it, don't re-derive it.

## Direction change -- read this before touching any code

Passes 5-8 built a hand-authored substitute for a real arm: extracting arm
geometry out of the character's skinned mesh, driving it with a hand-rolled
two-bone IK solver, and patching the resulting seam with primitive sphere
caps. That entire approach is discarded, reviewer's explicit call -- it kept
producing wrong-looking joints/caps every pass and is not worth further
cosmetic iteration.

Replace it with the Actor SDG / omni.anim.people behavior-animation
mechanism `ANIM_SPIKE_RESULTS.txt` already proved works: trigger real,
professionally-authored `push_button`/`type_keyboard` (or similar) skelanim
clips on a normal biped character via `CustomCommand` + the behavior-script
command-file path, exactly as `isaacpjt/anim_spike_test.py` does. Do not
resume tuning `RightArmRig`/`LeftArmRig`/`ShoulderCap`/`ElbowCap` or the
two-bone IK solver in `hand_intrusion_test_actor.py` -- delete/replace that
mechanism rather than patch it further.

## What "done" means this pass

A character, placed and scaled correctly at `TableSet_00`, plays a real
behavior animation (push_button, typing, or another `~/Downloads` clip that
visibly brings a hand into the table's intrusion zone) on the same
5-10s-period reach cycle the old mechanism used, integrated into the actual
`hand_safety` pipeline -- not the spike's empty stage. Non-negotiable
acceptance criteria, judged by eye on rendered frames (front, side,
overhead, fixed table camera) plus real topic output, not by code executing
without error:

1. **Body pose.** Character stands/sits naturally at the table, matching
   whichever placement (standing behind the south-side chairs, or seated)
   reads as natural for the chosen animation. No T-pose, no idle/broken
   pose before or after the behavior plays.
2. **Joint realism.** Since this is now a real authored animation, this
   should hold for free -- confirm it does, don't re-litigate it by hand.
3. **ROI/detection.** The animation's hand motion actually enters
   `hand_safety`'s ROI at the right moment. Measure this the way pass 8 did
   for the old rig -- project the real hand/glove geometry's world bbox
   through the table camera's intrinsics/extrinsics and compare against
   `TABLE_ROI_NORMALIZED` in `hand_safety/hand_safety/roi_intrusion.py` --
   do not assume the old ROI calibration still applies to a differently
   shaped/positioned reach. Confirm on real hardware: at least one real
   annotated hand frame and one `roi_intrusion: true` pulse on
   `/hand_safety/roi_intrusion` per cycle.

## Required work

- Delete or clearly disable the old rigid-arm mechanism in
  `isaacpjt/hand_intrusion_test_actor.py` (`RightArmRig`/`LeftArmRig`
  extraction usage, `solve_two_bone_ik` driving, `ReachAnimator`'s
  progress-based rotation code) and `assets/rigid_arm_asset.usda`'s
  purpose-built arm/cap geometry -- keep them only if still useful as an
  opt-in fallback (e.g. `HAND_TEST_RIG_MODE=legacy`-style), but the shipped
  default must be the Actor SDG path.
- Wire the spike's character-spawn + custom-command + behavior-script
  mechanism into `hand_intrusion_test_actor.py`/`mobile_manipulator_demo.py`
  in place of the old rig: correct placement/scale at `TableSet_00`, correct
  intrusion target, and the same randomized 5-10s reach period
  (`HAND_TEST_MIN_PERIOD`/`MAX_PERIOD` envs) driving when the behavior
  command fires, followed by a return to idle.
- Re-measure and, if needed, recalibrate `TABLE_ROI_NORMALIZED` against this
  new animation's actual hand trajectory -- do not assume pass 8's
  calibration (tuned for the old rig's exact hand position) still applies.
- Run the real four-terminal `hand_safety` workflow
  (`hand_safety/README.md`), watch at least four complete cycles, and
  confirm `roi_intrusion: true` fires on each one.
- Reuse `MOBILE_DEMO_CAPTURE_POSES=1` (or extend it) for fast headless
  visual QA before every live pipeline run, the way pass 8 did.

## Iterate until it actually meets the criteria

A single attempt that "runs" is not the deliverable. If integration doesn't
visually and numerically satisfy the criteria above, keep adjusting
placement/timing/ROI and re-render/re-run until it genuinely does -- not
until the code merely executes without error.

Never paper over a real defect to make it look finished: don't lower YOLO
confidence, move/shrink the ROI without a measured, camera-projected reason,
hide the person or a body part from the camera, disable a check, or claim a
criterion is met without an actual rendered frame or real topic echo proving
it. If something still cannot be fixed this pass, say so plainly in the log
and the final report -- do not claim done, and do not silently leave broken
behavior as the shipped default. An honest "still broken, here's what I
tried and why it didn't work" is the correct outcome if that's the truth; a
fake pass is not.

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
4. Report the commit hash, changed files, exact run commands, and which
   acceptance criteria are now met versus still open.
