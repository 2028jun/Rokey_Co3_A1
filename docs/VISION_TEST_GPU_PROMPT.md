# Vision-test GPU prompt: Actor SDG sit/type/push behavior loop + hand-detection verification

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
- Read `AGENTS.md`, `GPU_RUN_LOG.txt` (all passes), `ANIM_SPIKE_RESULTS.txt`,
  `isaacpjt/anim_spike_test.py`, `isaacpjt/hand_intrusion_test_actor.py`,
  `isaacpjt/mobile_manipulator_demo.py`, and
  `hand_safety/hand_safety/roi_intrusion.py` completely before changing
  anything. Confirm for yourself that `HAND_TEST_RIG_MODE` still defaults to
  `"rigid_arm"` in `hand_intrusion_test_actor.py` -- the prior pass only
  decided to discard that mechanism and spiked the replacement API on an
  empty stage; nobody has integrated it into the real pipeline yet. If that
  has changed by the time you read this, the direction below still applies,
  just adjust the starting point accordingly.

## Direction (unchanged from the previous pass, restated for self-containedness)

Passes 5-8's hand-authored rigid-arm/two-bone-IK mechanism is discarded --
reviewer's explicit call, not worth further cosmetic iteration. Replace it
entirely with the Actor SDG / `omni.anim.people` behavior-animation mechanism
`ANIM_SPIKE_RESULTS.txt` already proved works headless with no crash:
triggering real, professionally-authored skelanim clips via `CustomCommand` +
the behavior-script command-file path, exactly as `isaacpjt/anim_spike_test.py`
does. Do not resume tuning `RightArmRig`/`LeftArmRig`/`ShoulderCap`/
`ElbowCap` or `solve_two_bone_ik` in `hand_intrusion_test_actor.py` -- delete
or clearly gate that mechanism behind an opt-in fallback
(`HAND_TEST_RIG_MODE=legacy`-style) rather than patch it further.

The spike only proved the trigger mechanism on an empty stage with a default
biped standing alone. This pass integrates it into the real `hand_safety`
scene (`TableSet_00`, the fixed table camera, real YOLO detection) and
extends the behavior to a full sit/act/act loop with adaptive repositioning
-- see the four goals below.

## The four goals for this pass

1. **Seat the actor via Actor SDG, not a manual hack.** Place the character
   at one of `TableSet_00`'s chairs (`Chair_00_Visual` or `Chair_01_Visual`
   under `/World/Dining/TableSet_00`) and bring it to a seated pose using
   `omni.anim.people`'s own sit mechanism (a real triggered `Sit`
   command/animation state), not the old manual translate/scale placement
   with no actual sit animation. If `omni.anim.people` has no built-in `Sit`
   command reachable this way, trigger a seated-rest skelanim clip through
   the same `CustomCommand` mechanism the spike used for push_button --
   whichever route is real and inspectable, not just a static pose fudge.
2. **Cyclic behavior loop.** Drive a repeating sequence: **typing -> sit ->
   push_button -> sit -> (repeat)**, each transition a real triggered Actor
   SDG command (same `CustomCommand`/behavior-script path as the spike), not
   a hand-rolled rotation. Keep some idle/hold time in each `sit` step so the
   states are visually distinguishable in rendered frames and topic-timed
   logs, not an instant flicker.
3. **Adaptive walk-in on ROI miss.** For whichever of `typing`/`push_button`
   is measured (project the real hand/glove mesh's world bounding box
   through the table camera's intrinsics/extrinsics, exactly as pass 8 did)
   to NOT bring the hand into `TABLE_ROI_NORMALIZED`
   (`hand_safety/hand_safety/roi_intrusion.py`), insert a short forward walk
   -- a `GoTo`-style navigation command moving the character a small,
   measured distance toward the table -- after the following `sit` step and
   before that behavior fires again. Do not fake this by moving the camera,
   shrinking/relocating the ROI without a projected reason, or scripting a
   plain translate instead of a real navigation command if one exists.
4. **Determine and report which is true, with real evidence:** either (a)
   `hand_safety`'s YOLO-based detector correctly reports the hand whenever
   the animated hand is genuinely inside the ROI, confirmed by annotated
   frames plus `/hand_safety/roi_intrusion: true` pulses lined up with the
   behavior states that reach the ROI; or (b) the hand is geometrically
   inside the ROI (per the projected bbox) during a behavior state but
   `hand_safety` does NOT report it -- in which case say so plainly, with
   the frame(s)/topic echo proving the geometric overlap and the absence of
   a detection, and a best-effort reason if you can identify one (motion
   blur, animation-frame timing versus camera capture, hand pose/occlusion
   outside YOLO's training distribution, etc.). Do not assume (a) without
   checking, and do not silently omit (b) if that's what you find.

## Investigate before implementing -- do not guess these

`AGENTS.md` already says: check the locally installed Isaac Sim 5.1 source
under `/home/rokey/dev_ws/isaac_sim/isaacsim` when an API is uncertain. None
of the following were exercised by the spike or any prior pass, so confirm
them from source/experiment before writing code that assumes an answer:

- Whether `omni.anim.people` ships a built-in `Sit`/`GoTo` command language
  (distinct from the `CustomCommand` TIMING mechanism the spike used for
  push_button/type_keyboard) and its exact command-file syntax --
  check `character_behavior.py` and the command-manager source referenced
  from `ANIM_SPIKE_RESULTS.txt`.
- Whether `Sit` requires a specially tagged/authored "sittable" prim, or
  works against `TableSet_00`'s plain `Chair_00_Visual`/`Chair_01_Visual`
  meshes as-is.
- Whether the `push_button`/`type_keyboard` skelanim clips (from
  `~/Downloads`, as in the spike) assume a standing or seated rest pose, and
  whether they visibly reach a tabletop surface at seated height once the
  character is actually sitting at `TableSet_00` -- confirm by rendering;
  the spike's empty-stage, standing-character result does not automatically
  carry over to a seated context.
- Whether `GoTo`-style navigation needs a baked NavMesh for the restaurant
  floor and whether `assets/lightweight_restaurant/` already has one --
  no prior pass or doc mentions a navmesh, so treat this as unknown, not
  assumed-present.

## What "done" means this pass

Judged by eye on rendered frames (front, side, overhead, fixed table camera)
plus real topic output, not by code executing without error:

1. **Body pose.** Character sits/stands naturally at every stage of the
   cycle (seated idle, typing, push_button, walk-in if triggered). No
   T-pose, no idle/broken pose, no visible clipping into the chair or table.
2. **Cycle correctness.** Rendered frames and behavior-script/command logs
   across at least 4 full cycles show the sequence firing in the intended
   order (typing -> sit -> push_button -> sit, with the walk-in inserted
   exactly when goal 3's measurement says it's needed).
3. **ROI/detection, goal 3.** The adaptive walk-in measurably changes the
   hand's projected position relative to `TABLE_ROI_NORMALIZED` in the
   expected direction (closer to / inside the ROI) -- show the before/after
   projected bbox numbers, not just "it moved."
4. **ROI/detection, goal 4.** A definitive (a)-or-(b) answer per goal 4
   above, backed by at least one real annotated frame and one real
   `/hand_safety/roi_intrusion` topic echo per relevant behavior state.

## Required work

- In `isaacpjt/hand_intrusion_test_actor.py` (or a new module replacing it --
  your call, but keep `mobile_manipulator_demo.py`'s integration point
  similarly named/opt-in-gated): remove or clearly fallback-gate the
  rigid-arm mechanism; wire the spike's character-spawn + custom-command +
  behavior-script mechanism as the shipped default, driving the sit/type/
  push loop and the adaptive walk-in.
- Re-measure and, if needed, recalibrate `TABLE_ROI_NORMALIZED` against the
  new animations' actual hand trajectories -- do not assume pass 8's
  calibration (tuned for the old rig's exact hand position) still applies.
- Reuse or extend `MOBILE_DEMO_CAPTURE_POSES=1`
  (`isaacpjt/mobile_manipulator_demo.py`) for fast headless visual QA of
  every new pose/state before every live pipeline run, the way pass 8 did.
- Run the real four-terminal `hand_safety` workflow (`hand_safety/README.md`)
  and watch at least four complete cycles on real hardware.

## Iterate until it actually meets the criteria

A single attempt that "runs" is not the deliverable. If the loop doesn't
visually and numerically satisfy the criteria above, keep adjusting
placement/timing/ROI/walk-in distance and re-render/re-run until it
genuinely does -- not until the code merely executes without error.

Never paper over a real defect to make it look finished: don't lower YOLO
confidence, move/shrink the ROI without a measured, camera-projected reason,
hide the person or a body part from the camera, disable a check, or claim a
criterion is met without an actual rendered frame or real topic echo proving
it. Goal 4 explicitly allows -- requires -- reporting a genuine negative
result if that's what you find; do not convert it into a false pass.

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
   the four goals and acceptance criteria are now met versus still open --
   including an explicit statement of the goal-4 (a)/(b) finding.
