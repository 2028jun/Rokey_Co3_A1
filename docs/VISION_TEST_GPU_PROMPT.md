# Vision-test GPU prompt: natural human pose + hand-reach visual acceptance

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
- Read `AGENTS.md`, `GPU_RUN_LOG.txt` (all prior passes), and this file
  completely before changing anything.
- Do not repeat work already logged as done: Pass 6's seat-coordinate search
  and static-arm proof, Pass 7's terminal/YAML/observability diagnosis.

## Non-negotiable visual acceptance criteria

The reviewer judges this by eye, not by IK math passing silently. All of the
following must be true in rendered frames (front, side, overhead, and the
fixed table camera) before this is considered done:

1. **Body pose.** The person is either naturally seated in a chair
   (body/legs not clipping the chair or table) or naturally standing
   beside/behind it. Both arms rest in a normal relaxed pose when not
   reaching -- no T-pose, no permanently spread/winged arm, on *either* arm.
   Only the single reaching arm moves during an intrusion event; everything
   else, including the idle arm, must look like a person standing/sitting
   still.
2. **Joint realism.** No visible twisting, hyperextension, or bend outside
   normal human range of motion, at rest, mid-reach, and full reach.
   Concretely, before accepting any solved or authored pose, check it
   against real joint limits:
   - Elbow is a hinge: it flexes roughly 0-145 degrees and must never
     hyperextend backward or bend sideways/twist at the joint itself. If the
     two-bone IK solver or pole hint ever produces an elbow bending the
     wrong direction or looking locked/backward, fix the pole hint or
     target -- not the visual symptom.
   - Shoulder flexion/abduction for a forward table reach should read as
     reaching forward-and-down, not out to the side or through the torso.
   - No joint should visibly rotate through the body or produce a
     twisted/broken-looking limb at any point in the reach cycle, not just
     at the endpoints -- check the transition frames too.
   - If a solved angle sits outside these bounds, the current
     placement/scale/rest-pose choice is geometrically wrong for this
     character. Fix the placement or rest pose; do not force bones into an
     unnatural configuration just to hit the target.

## Current state (do not re-litigate)

Pass 7 moved the person out of the table to a standing customer position,
added 0.85 uniform scaling, moved the intrusion target to the customer-side
edge, replaced spherical shoulder/elbow caps with tapered cone transitions,
and made annotated ROS output (`/hand_detection/image` + `rqt_image_view`)
the default observable workflow. That plumbing works and does not need to be
redone.

Known open defects going into this pass:
1. The untouched opposite arm is still in the source asset's spread pose --
   directly violates acceptance criterion 1 above.
2. The reaching glove's rendered endpoint has not been confirmed against
   `TABLE_HAND_TARGET=(-3.25,-3.33,0.98)` since the exact-rotation fix
   landed.
3. Neither arm's joints have been checked against criterion 2 above at
   mid-reach -- only rest and full-reach angles have been logged as numbers,
   never visually inspected for realism.

## Required work

- Re-run a forced full-reach capture first and confirm/deny the endpoint
  fix. If the glove still doesn't land in the ROI, measure the actual
  rendered wrist world transform and fix the transform/scale composition --
  do not tune YOLO confidence or move the ROI to compensate for a placement
  bug.
- Fix the opposite arm: extract it into its own static relaxed rig (same
  technique as the reaching arm -- see the asset build notes at the bottom
  of `isaacpjt/hand_intrusion_test_actor.py`) or rebuild the body mesh with
  a clean relaxed replacement. Do not hide the person or leave a T-pose.
- Walk both arms through rest -> mid-reach -> full-reach and check every
  stage against the joint-realism criterion above, not just the two
  endpoints.
- Inspect the tapered shoulder/elbow cone transitions at all three stages;
  fix any hole, detachment, or obviously artificial cone silhouette with
  overlapping transition geometry.
- Confirm the person stays outside table/chair geometry and the root stays
  fixed for at least four complete reach cycles.
- Run the four-terminal workflow in `hand_safety/README.md`, manually
  publish `table_arrived=true`, view `/hand_detection/image`, and confirm
  at least one real annotated hand frame and one ROI-intrusion `true`
  pulse.

## Iterate until it actually meets the criteria

A single attempt that "runs" is not the deliverable. If a fix doesn't
visually resolve a defect, try a different approach (different rest pose,
different pole hint, rebuilt geometry, different seat/target placement) and
re-render, repeating until every criterion above genuinely holds -- not
until the code merely executes without error.

Never paper over a real defect to make it look finished: don't lower YOLO
confidence, move/shrink the ROI, hide the person or a body part from the
camera, disable a check, or claim a criterion is met without an actual
rendered frame proving it. If, after genuinely trying alternatives,
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
