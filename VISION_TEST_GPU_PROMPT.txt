# Vision-test GPU prompt: joint-cap cosmetics + live multi-cycle confirmation

Paste this entire document into a coding-agent session on the GPU machine.
Target: Isaac Sim 5.1.0-rc.19, ROS 2 Humble, RTX 5080, branch `vision-test`,
`ROS_DOMAIN_ID=102` (matches `AGENTS.md`).

This prompt is self-contained: reading it plus `AGENTS.md` and
`GPU_RUN_LOG.txt` is enough to do the whole task and finish it, including
pushing the result back to `vision-test`. Do not wait for further
instructions mid-task, and do not stop after implementing -- run it, look at
the rendered frames, and finish per "Finishing" below.

## Bootstrap (do this first, every time)

- `git status`, then `git fetch origin` and `git pull --ff-only origin
  vision-test`. Never discard existing work with reset/checkout/clean.
- Read `AGENTS.md`, `GPU_RUN_LOG.txt` (all prior passes, especially pass 8),
  and this file completely before changing anything.
- Do not repeat work already logged as done: pass 6's seat-coordinate search
  and static-arm proof, pass 7's terminal/YAML/observability diagnosis, and
  pass 8's opposite-arm fix, cone-to-sphere revert, ROI recalibration, and
  the `MOBILE_DEMO_CAPTURE_POSES=1` visual-QA capture path it added to
  `isaacpjt/mobile_manipulator_demo.py` (reuse that path -- it's the fast,
  cheap way to check this actor's pose; see its own code for usage).

## Non-negotiable visual acceptance criteria

The reviewer judges this by eye, not by IK math passing silently. All of
the following must be true in rendered frames (front, side, overhead, and
the fixed table camera) before this is considered done:

1. **Body pose.** The person is naturally standing beside/behind the
   south-side chairs (body/legs not clipping the chair or table). Both arms
   rest in a normal relaxed pose when not reaching -- no T-pose, no
   permanently spread/winged arm, on *either* arm. Only the single reaching
   arm moves during an intrusion event; everything else, including the idle
   arm, must look like a person standing still.
2. **Joint realism.** No visible twisting, hyperextension, or bend outside
   normal human range of motion, at rest, mid-reach, and full reach. No
   visible hole, gap, or detachment at the shoulder/elbow transitions at
   any of those stages either.
3. **ROI/detection.** A full reach cycle produces at least one real
   annotated hand frame and at least one `roi_intrusion: true` pulse on
   `/hand_safety/roi_intrusion`, observed on real hardware, not just IK
   math.

## Current state (do not re-litigate)

Pass 8 fixed three real, previously-undiscovered defects, all confirmed on
hardware (see `GPU_RUN_LOG.txt` pass 8 for full detail):

1. The opposite (left) arm now has its own static relaxed rig
   (`LeftArmRig`/`ElbowPivotL`), matching the reaching arm's technique --
   criterion 1 is met for both arms.
2. The shoulder/elbow transition caps are `Sphere` prims again (not
   `Cone`), centered on each rotation pivot, sized to close the seam at
   this pass's more relaxed rest angle (shoulder radius 0.085, elbow 0.06)
   -- no hole or floating spike at rest, mid-reach, or full reach in any of
   the four camera angles.
3. `hand_safety/hand_safety/roi_intrusion.py`'s `TABLE_ROI_NORMALIZED` was
   stale (calibrated for pass 1-5's seat/target position) and never
   overlapped the hand's actual position after pass 6/7 moved the target --
   this silently zeroed out every ROI-confirmed detection for two passes
   even though the reach itself was geometrically correct. Recalibrated by
   projecting the reach arm's true glove mesh bounding box through the real
   camera's own intrinsics/extrinsics; confirmed on hardware: 46 `true`
   samples over a 45 s / 45-cycle-ish window, one confirmed detection
   captured directly (confidence 0.884, bbox [241,339,318,393] px).

Known open defect going into this pass:

1. The shoulder/elbow sphere caps close the seam but still read as a
   visible bulbous ball rather than a seamless joint, especially at the
   elbow (fabric-colored sphere against bare-skin-textured
   forearm/upper-arm). This is a known limitation already flagged back in
   pass 5's log: a primitive sphere has diminishing returns, and a proper
   fix needs non-primitive, UV/normal-matched cap geometry shaped to the
   character's actual joint cross-section. Not attempted in pass 8 --
   the hole/spike defect is fixed, this cosmetic one is not.

## Required work

- Decide whether the pass-8 sphere-cap bulge is acceptable as shipped, or
  build proper non-primitive cap geometry (extract a short cylindrical
  "cuff" segment from the same source mesh the arm pieces came from,
  tapered/blended into both the torso-side and arm-side cut boundaries,
  instead of a primitive sphere) if the reviewer wants it fully seamless.
  Use `isaacpjt/mobile_manipulator_demo.py`'s `MOBILE_DEMO_CAPTURE_POSES=1`
  path to iterate quickly (front/side/overhead/table PNGs at progress
  0/0.25/0.5/0.75/1.0 in ~20s, no ROS/GUI needed) before ever touching the
  live pipeline.
- Run a live four-terminal `hand_safety` session (see `hand_safety/README.md`)
  and specifically watch at least four complete reach cycles end to end,
  confirming: the person's root never translates (compare
  `ComputeLocalToWorldTransform` across cycles, not just a visual
  spot-check, the way pass 6 did for the old rest pose), the reach still
  lands cleanly and produces `roi_intrusion: true` on every cycle (not just
  one, as pass 8 only directly captured), and nothing about repeated cycles
  degrades detection confidence or introduces jitter.
- While that live session is running, also sanity-check
  `/hand_detection/detections` for any spurious non-reach-cycle "hand"
  detections (e.g. from the RG2 gripper or the new sphere caps themselves)
  now that the ROI has moved -- pass 3's gripper false-positive and pass
  5's shoulder-cap false-positive were both real, camera/ROI-position-
  dependent issues, and the ROI just moved to a new part of the frame.

## Iterate until it actually meets the criteria

A single attempt that "runs" is not the deliverable. If a fix doesn't
visually resolve a defect, try a different approach and re-render,
repeating until every criterion above genuinely holds -- not until the code
merely executes without error.

Never paper over a real defect to make it look finished: don't lower YOLO
confidence, move/shrink the ROI without a measured, camera-projected reason
the way pass 8 did, hide the person or a body part from the camera, disable
a check, or claim a criterion is met without an actual rendered frame or
real topic echo proving it. If, after genuinely trying alternatives,
something still cannot be fixed in this pass, say so plainly in the log and
in the final report -- do not claim done, and do not silently leave broken
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
