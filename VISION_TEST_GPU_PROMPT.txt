# Vision-test GPU execution prompt: make the human/reach visually credible and fix the observable YOLO workflow

Paste this document into a coding-agent session on the GPU computer. The target
is Isaac Sim 5.1.0-rc.19, ROS 2 Humble, RTX 5080 16 GB, branch `vision-test`.
Use `ROS_DOMAIN_ID=102`, matching `AGENTS.md` and the application default.

Read `GPU_RUN_LOG.txt` completely before changing anything. In particular, Pass
6 is already finished and committed. Do not repeat its seat-coordinate search,
static-body proof, or detection measurements. This is the next implementation
pass, driven by the reviewer's direct visual feedback.

## Repository bootstrap and completion contract

Preserve all existing GPU-side work. Run `git status`, fetch `origin`, switch to
`vision-test`, and update with `git pull --ff-only origin vision-test`. Never
discard work with reset, checkout, or deletion. If a fast-forward is blocked,
integrate the local work safely.

Work end to end: inspect, implement, render, run the actual detector, iterate,
and verify the result visually. Do not stop after planning, after reproducing
the defects, or after proving the current geometry impossible.

When finished:

1. Append one dated next-pass entry to `GPU_RUN_LOG.txt`; do not rewrite prior
   pass entries.
2. Update this prompt so it describes only whatever work still remains. Do not
   leave completed tasks as instructions for the next agent.
3. Commit and push the tested code and log to `vision-test`.
4. Report the commit, changed files, exact run commands, visual results, and
   remaining limitations.

## Current reviewer-visible failures

Treat all four as real acceptance failures:

1. The person is embedded in the table and only flicks the arm.
2. The person must instead be naturally seated by a chair or plausibly standing
   beside it.
3. The rigid arm looks grotesque: circular elbow/shoulder caps are visible,
   shoulder geometry is broken, and the arm starts/animates in an unnatural
   spread pose.
4. A reviewer launching the YOLO workflow directly from terminals does not get
   a visible simulation/detection window.

The result must look credible to a human reviewer, not merely satisfy IK math.
If the current arm cannot reach from the chair side, change the design: move
the tabletop intrusion target toward the customer-side edge while keeping it
inside the ROI, uniformly scale the person/arm, use a better character/rest
pose, rebuild the arm geometry, or combine those measures. Do not put the
person back inside the table and do not accept a spread/T-pose arm as “working.”

## Already established — do not redo

- Pass 6 proved the current center target `(-3.2, -2.2, 0.83)`, the standing
  character's 0.496 m arm, a fully static torso, and a person outside the table
  cannot coexist. Exhaustive `HAND_TEST_SEAT_X/Y/Z` tuning cannot solve it.
- The shipped Pass 6 seat `(-3.337, -2.216, -0.20)` is deep inside the real
  table. It is not a usable baseline and must not remain the default.
- Pass 6 proved rigid-arm mode has zero root translation during reach. Preserve
  that property unless the final chosen design needs a small, deliberate,
  natural fixed pose adjustment. Do not restore Pass 5's per-cycle whole-body
  lunge.
- Passes 2 and 4 ruled out late-bound UsdSkel animation and Animation Graph on
  this build (no rendered motion and native segfault respectively). Do not
  repeat them.
- Pass 3 ruled out a PhysX articulation drive on the old People asset because
  it has no articulation/joints. Do not repeat that inspection on the same
  asset. A genuinely different asset may be inspected once if it is a serious
  candidate.
- Pass 5 verified the detached two-bone Xform mechanism, camera at 1280x960,
  detector confidence, ROI gating, and manual `table_arrived` publication.
  This pass is not a confidence/ROI retuning exercise.
- `/serving_robot/table_arrived` intentionally has no real publisher in this
  branch. Publish it manually during this isolated test; do not implement the
  integration publisher here.

## Task 1: redesign placement and reach as one visual system

Start from the real `TableSet_00` and chair transforms, not the current embedded
seat. Choose a customer position that visibly reads as either:

- seated at one of the existing chairs with feet/body clear of table geometry,
  or
- standing naturally next to/behind that chair.

Use free-camera views from the front, side, and above plus the fixed table
camera. The entire body must stay outside the tabletop and its underside
bracing. It must not float, sink through the floor/chair, intersect another
chair, or face away from the table.

The old target is at the tabletop center. It is acceptable and likely necessary
to introduce a separate hand-intrusion test target closer to the customer-side
table edge. It must remain visibly over the tabletop, clear the surface, and
fall inside the existing ROI. Do not move the ROI merely to legitimize a bad
pose. Keep the production table/docking geometry unchanged.

Uniform character scaling is explicitly allowed if it produces credible human
proportions relative to the chair and table. Scale the whole visual system
coherently—body, detached arm, pivots, offsets, bone lengths, and IK—not just
one limb. Reject giant/miniature results that look wrong beside the furniture.

At rest, both arms should read as relaxed beside/on the body or in another
plausible seated/standing pose. During intrusion, only the intended reaching
arm should move. No T-pose, permanently spread arm, backwards elbow, shoulder
rotation through the torso, or hand approaching from underneath the table.
A small fixed torso orientation/lean is allowed if it is natural and does not
animate each cycle; the body must not slide toward the target during reach.

Do not merely report that the old combination is impossible—the reviewer has
authorized a design change. Select and implement the least invasive credible
combination, then iterate against rendered frames.

## Task 2: replace the broken-looking arm construction

Inspect `assets/rigid_arm_asset.usda` and the extraction/build notes in
`isaacpjt/hand_intrusion_test_actor.py`. The current primitive `ShoulderCap` and
`ElbowCap` workaround is not visually acceptable if its spheres can be seen.

Fix the geometry and rest pose rather than hiding the defect with larger
spheres. Acceptable approaches include:

- rebuilding the cut boundaries with overlapping, tapered upper-arm/forearm
  geometry whose rotation seams remain inside the sleeve/joint volume;
- using non-spherical, material/normal/UV-compatible joint transition meshes;
- moving the pivots/cut planes and adding sleeve geometry so the shoulder stays
  filled through the required, now-smaller natural motion range;
- replacing the rigid arm/character asset with a better compatible asset if
  that is demonstrably cleaner on this Isaac Sim build.

The final close-up must have no obvious ball at the elbow or shoulder, no dark
hole/torn shoulder, no detached pieces, and no severe texture/material break.
Test rest, mid-reach, and full-reach from close side/front views. Also check the
operating table-camera distance.

Preserve useful environment overrides and legacy fallback, but the default
path must be the visually acceptable one. Do not solve appearance by disabling
the person or hiding the arm from the relevant camera.

## Task 3: make terminal-launched simulation and YOLO output observable

Reproduce the reviewer's “no screen appears” report using clean terminals.
Diagnose before changing defaults. There are multiple distinct gates:

- Isaac GUI is suppressed by `MOBILE_DEMO_HEADLESS=1`; normal visual testing
  requires `MOBILE_DEMO_HEADLESS=0` and a valid graphical `DISPLAY`.
- `hand_detector_node.py` currently declares `show_window=true` and
  `publish_annotated_image=true`, but `hand_safety/config/hand_safety.yaml`
  sets both false. Determine which parameter source the documented command
  actually uses; make code, YAML, and README unambiguous.
- inference is intentionally disabled until
  `/serving_robot/table_arrived=true`;
- an OpenCV HighGUI window requires a graphical session. Provide
  `rqt_image_view` on `/hand_detection/image` as the reliable ROS-native
  viewing path.

Produce and verify a copy-paste terminal workflow from the repository root,
using `ROS_DOMAIN_ID=102` everywhere:

1. build/source and launch Isaac Sim in non-headless mode;
2. build/source and launch the detector with annotated-image publication
   explicitly enabled (and `show_window` explicitly enabled only when the
   session supports it);
3. manually publish `table_arrived=true`;
4. open `/hand_detection/image` with `rqt_image_view`;
5. verify input/output topics, image rate, detector logs, and at least one
   visible annotated frame.

If direct `cv2.imshow` remains unreliable under the target desktop/Wayland/X11
session, do not claim it works. Make annotated-image + `rqt_image_view` the
documented primary workflow and print a clear startup log explaining where to
view it. Fail or warn clearly when GUI display is requested but unavailable;
do not leave a silently-running terminal that appears hung.

Update `hand_safety/README.md` and any stale quick-start commands. Explicitly
document the difference between the Isaac viewport, OpenCV window, and
`rqt_image_view`, and the need for the manual arrival gate.

## Required verification

Capture or inspect, at minimum:

- rest pose from front, side, overhead, and fixed table camera;
- mid-reach and full-reach close-ups of shoulder and elbow;
- at least 4 full cycles showing no animated root/body slide;
- world/body/table/chair non-intersection in the chosen pose;
- hand visibly crossing into the existing tabletop ROI without passing through
  the tabletop;
- a clean-terminal demonstration in which Isaac GUI is visible and the
  annotated detector stream is visible through the documented command.

Programmatic transform/IK checks are supporting evidence only. Visual captures
are required because the reported defects are visual.

## Acceptance criteria

- The person is credibly seated by a chair or standing beside it, entirely out
  of the table and other furniture.
- Rest and reach poses look human; the arm is not spread at rest and only the
  reaching arm articulates during a cycle.
- The hand enters the tabletop ROI from the customer side and clears the table.
- The shoulder and elbow have no visible spheres, holes, tearing, or detached
  geometry in close-up or the operating camera.
- No per-cycle whole-body translation/lunge is present.
- A reviewer can follow the documented terminal commands and see both the
  simulation and annotated YOLO result, with the arrival gate clearly handled.
- Completed Pass 6 investigations are not repeated, and detection thresholds/
  ROI are not gratuitously retuned.

## Log entry requirements

Record the final person asset/mode, position, yaw, uniform scale, target point,
bone/pivot changes, and whether seated or standing. Explain why the composition
looks natural from each required view. Record the arm-geometry fix, exact
terminal commands, GUI environment, actual resolved ROS parameters, topic
rates, and annotated-view result. List any remaining visual compromise plainly.
