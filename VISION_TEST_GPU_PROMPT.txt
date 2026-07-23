# Vision-test GPU execution prompt: verify the fully-static seated arm-only reach

Paste this document into a coding-agent session running on the GPU computer.
The target environment is Isaac Sim 5.1.0-rc.19, ROS 2 Humble, and an RTX
5080 with 16 GB VRAM. Work on the `vision-test` branch.

Read `GPU_RUN_LOG.txt` before changing anything, especially the "Pass 6 prep"
entry at the bottom -- it explains exactly what changed since pass 5 and why,
and what is and is not already verified. Git history preserves every prior
pass's implementation and this prompt's own prior revisions; this prompt
contains only the current work. Do not copy old pass narratives back into it.

## Repository bootstrap

Before working, run `git status` and preserve any existing uncommitted GPU-side
work; never discard it with reset, checkout, or deletion. Fetch `origin`, switch
to `vision-test`, and update it with `git pull --ff-only origin vision-test`.
If local work prevents a fast-forward, integrate it safely instead of
overwriting it. Then read this prompt and `GPU_RUN_LOG.txt` completely and
execute the work end to end. Do not stop after planning or after one test case.

When finished, append a "Pass 6" entry to `GPU_RUN_LOG.txt` (below the "Pass 6
prep" entry already there -- do not overwrite or renumber it), commit the
tested code and log, push `vision-test`, and report the commit hash, changed
files, measured results, and remaining limitations.

## Objective

Pass 5 replaced the whole-body-slide reach with a two-bone arm rig and
verified it on hardware, but the arm was reached by *leaning the whole body*
toward the target every cycle (see pass 5's log entry) -- the reviewer
watched it in simulation and asked for that removed entirely: **the person
should stay seated and completely still; only the arm joints (shoulder,
elbow) should move.** Pass 6 prep (CPU-only, see the log) did exactly that
in code -- deleted the per-reach body lean, and moved the character's seat
close enough to the table that a fully static arm-only pose can physically
reach `TABLE_HAND_TARGET` (this character's own two-bone reach is short, so
the seat had to move quite a bit closer than before -- see the log's
"Observed" section for the exact geometric reasoning).

This pass has one job, and it is entirely about the *quality of the motion*,
not detection:

1. Confirm the seat's new position (`HAND_TEST_SEAT_X`/`Y`/`Z`, defaults
   ~(-3.337,-2.216,-0.20)) doesn't visually clip into the table, another
   chair, or other scene geometry. This was computed offline without seeing
   the actual `TableSet_00` mesh and is the single most likely thing to be
   wrong -- adjust the three env vars until it looks physically sensible;
   no code change is needed to retune, `_solve_static_arm_target()` re-solves
   the IK automatically for whatever seat position is set.
2. Confirm the body is now genuinely static: watch several reach cycles and
   confirm the torso/head/legs never move even slightly -- only
   `RightArmRig`'s shoulder rotation and `ElbowPivot`'s elbow rotation
   should animate. If ANY body translation is visible, that's a real bug
   (the code shouldn't be touching a translateOp at all in `rigid_arm` mode
   any more -- grep for `self.translate_op` in `ReachAnimator` if this
   somehow still happens and report exactly where it's coming from).
3. Judge whether the reach itself looks natural: does the shoulder/elbow
   articulation read as a person reaching across a table from a seated
   position, or does it look stiff, robotic, or like the arm is bending the
   wrong way? Does the permanent (no longer transient) `-0.20m` root sink
   read as a low/seated posture or as visibly sinking into the chair/floor?
4. **Do not spend this pass judging hand-detection quality or confidence.**
   Pass 5 already confirmed the two-bone rig mechanism itself detects fine
   and gates `roi_intrusion` cleanly (see that log entry for the numbers);
   the reviewer will judge detection by watching the simulation directly
   this time, not from a report. If something about the new seat position
   obviously breaks detection (e.g. the hand no longer enters the ROI at
   all), report that as a motion/placement bug, not a detection-tuning
   task -- do not re-tune YOLO/ROI parameters this pass.

Keep the current ROS domain configuration and the 1280x960 camera input.
`/serving_robot/table_arrived` still has no real publisher in this repo --
that is intentional, it gets connected later during full-system
integration, and is explicitly out of scope for this branch. Continue
publishing it manually for testing exactly as every prior pass did (e.g.
`ros2 topic pub`); do not try to "fix" its absence.

## Known results: do not repeat these dead ends

- Passes 2 and 4: `UsdSkel`/Animation Graph joint animation does not render
  on this Isaac Sim build (silently for UsdSkel, a segfault for Animation
  Graph). This is exactly why the reach uses a plain non-skinned `Xform`
  chain instead -- do not attempt either route again.
- Pass 3: the RG2 gripper false positive was fixed by tightening the table
  ROI polygon, not by repainting the gripper. Do not revert the tightened
  ROI, and do not investigate this again -- it's unrelated to this pass.
- Pass 5: picked `male_adult_construction_01_new` for its glove's YOLO
  confidence, verified the rig's shoulder/elbow-cap geometry plugs the
  rotation-swept gap at the joints (`ShoulderCap`/`ElbowCap` spheres, 0.065
  radius) with an acceptably small residual false-detection rate. None of
  that changed this pass -- do not re-tune cap radius or re-run the
  detection-confidence measurement; that is explicitly out of scope here
  (see Objective item 4).
- Pass 5 also found `/serving_robot/table_arrived` has no publisher
  anywhere in this repo. Confirmed intentional (connected later during
  integration) -- do not treat this as a bug to fix.
- Pass 6 prep found, analytically, that NO seat X/Y position can make this
  standing-pose character's arm reach the table without *some* vertical
  root sink -- its shoulder sits 1.44m up regardless of seat placement,
  the target is 0.83m up (a 0.61m gap `rotateZ` can never close), and the
  arm's own max reach is only 0.496m end to end, less than that vertical
  gap alone. If the shipped `-0.20m` sink still looks wrong on hardware,
  the fix is tuning `HAND_TEST_SEAT_Z` (and re-deriving the matching
  `HAND_TEST_SEAT_X`/`Y` for the horizontal component so the arm still
  reaches -- see the formula in `_solve_static_arm_target()` and the log's
  reasoning), not re-deriving the whole approach. If a genuinely
  zero-sink, fully-natural static reach is required, the real fix is
  switching to a character asset with an actual seated rest pose (not
  attempted so far across any pass) -- flag this explicitly if the
  reviewer decides `-0.20m` isn't acceptable, rather than fighting the
  current character's geometry further.

## Relevant scene and pipeline facts

- The robot docks at `TableSet_00`; this is the table framed by the fixed
  table camera and treated as table 1 in this project.
- `TableSet_00` tabletop world height is `TABLE_TOP_Z = 0.73 m`; the hand
  target is `TABLE_TOP_Z + HAND_TARGET_Z_OFFSET` (0.83 m, unchanged since
  pass 4).
- Main files:
  - `isaacpjt/mobile_manipulator_demo.py` (spawns the actor, drives the
    per-frame `ReachAnimator.update()` loop -- unchanged this pass)
  - `isaacpjt/hand_intrusion_test_actor.py` (changed this pass -- read its
    module docstring's "IMPORTANT" paragraph and the "Asset build notes"
    entries 7-8 at the bottom before touching anything)
  - `isaacpjt/two_bone_ik.py` (unchanged since pass 5)
  - `assets/rigid_arm_asset.usda` (unchanged since pass 5)
- `HAND_TEST_RIG_MODE=legacy` still switches all the way back to the
  original pass #1-4 whole-body-slide mechanism, untouched, for A/B
  comparison if the new static pose needs debugging against something
  known-working.
- New/relevant env vars this pass: `HAND_TEST_SEAT_X`/`HAND_TEST_SEAT_Y`
  (default -3.337/-2.216, was -2.7/-3.2 through pass 5) and
  `HAND_TEST_SEAT_Z` (default -0.20, was 0.0 through pass 5 -- pass 5's
  `-0.427` was a *dynamic* lean value that no longer exists, not a prior
  default). `HAND_TEST_REACH_EXTENSION_RATIO` from pass 5 was deleted
  entirely (no replacement -- the seat position now bakes in the same
  design intent statically).

## Task 1: confirm the seat placement against the real scene

Before watching a reach cycle, just look at the character sitting still at
rest. Check from more than one angle (the fixed table camera plus a free
viewport):

1. Does the seat position clip into the table, another chair, or any other
   prop? This is the thing most likely to be wrong, since it was computed
   without seeing the actual geometry.
2. Does the character still read as "sitting at this table" from a
   reasonable angle, or does the seat move look obviously wrong (e.g. too
   close, facing the wrong way, floating)?
3. If it needs adjusting, tune `HAND_TEST_SEAT_X`/`Y` (and note that
   `HAND_TEST_SEAT_Z` and the seat's distance from the table are coupled --
   see the log's geometric reasoning; if you move the seat further from the
   table, `SEAT_Z` will need to go more negative to keep the target in
   reach, and vice versa). Watch the startup log line
   (`shoulder->target distance=... (arm max reach=..., REACHABLE/OUT OF
   REACH)`) to confirm whatever you land on is still actually reachable.

## Task 2: confirm the body is completely static

Watch at least 3-4 full reach cycles.

1. Confirm the torso, head, and legs do not move at all -- not even
   slightly -- throughout the entire cycle. Only the arm should animate.
2. If the body moves at all, that's a regression from pass 6 prep's intent
   and needs a real code fix (grep for `translate_op`/`translateOp` in
   `ReachAnimator` -- it should not exist for `rigid_arm` mode anymore).

## Task 3: judge the reach motion quality

1. Does the shoulder + elbow articulation look like a plausible seated
   reach across a table, or does it look mechanical/wrong (e.g. elbow
   bending backwards, shoulder rotating through the torso, hand approaching
   from a strange angle)?
2. Does the hand/glove enter the tabletop ROI without clipping through the
   table (same 0.83 m target as every prior pass)?
3. Does the permanent `-0.20m` root sink read as an acceptable low/seated
   posture, or does the character look like it's sinking into the chair or
   floor? This is now a constant, always-on characteristic (not a
   transient lean like pass 5), so judge it as a resting-state property,
   not just during the reach.
4. If the elbow bends the wrong way, `POLE_HINT_LOCAL_OFFSET` in
   `hand_intrusion_test_actor.py` controls that (not currently an
   env-var override -- add one if it needs iterating on hardware).

Do not re-measure YOLO confidence or `roi_intrusion` timing this pass (see
Objective item 4) -- if detection obviously looks broken because of the new
seat position (not because of anything pass 5 already covers), report it
as a placement issue for the reviewer to weigh in on, not something to
re-tune blind.

## Build and run

From the repository root:

```bash
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=101
MOBILE_DEMO_HAND_TEST=1 python isaacpjt/mobile_manipulator_demo.py
```

A/B against the prior (leaning) mechanism, or the original whole-body slide:

```bash
# tune seat placement without touching code
HAND_TEST_SEAT_X=-3.0 HAND_TEST_SEAT_Y=-2.5 HAND_TEST_SEAT_Z=-0.15 \
  MOBILE_DEMO_HAND_TEST=1 python isaacpjt/mobile_manipulator_demo.py

# full fallback to pass #1-4's mechanism
HAND_TEST_RIG_MODE=legacy MOBILE_DEMO_HAND_TEST=1 python isaacpjt/mobile_manipulator_demo.py
```

Useful checks (manually publish `table_arrived` as always):

```bash
ros2 topic pub /serving_robot/table_arrived std_msgs/msg/Bool "data: true" -1
ros2 run rqt_image_view rqt_image_view /hand_detection/image
```

## Acceptance criteria

- The seat visually fits the scene (no clipping into the table/chairs).
- Across multiple reach cycles, the torso/head/legs show zero movement --
  only the arm animates.
- The reach motion reads as a natural seated reach, not stiff/mechanical/
  wrong-direction, and the hand clears the tabletop at the existing 0.83 m
  target.
- The resting `-0.20m` sink (or whatever value was tuned to) is judged
  acceptable as a permanent seated posture, or a specific alternative value
  is recommended if not.
- No detection/confidence re-tuning was done this pass.

## Deliverables

Append one dated "Pass 6" entry to `GPU_RUN_LOG.txt` (after the existing
"Pass 6 prep" entry -- do not edit that one except to fix a factual error)
containing:

- whether the seat position needed adjusting, and the final
  `HAND_TEST_SEAT_X`/`Y`/`Z` values used;
- confirmation (or not) that the body is fully static across multiple
  cycles;
- a qualitative judgment of the reach motion and the resting sink posture;
- any code changes made and why; and
- remaining limitations, including whether a seated-pose character swap
  should be considered if the current sink still looks wrong.

Keep implementation changes focused and retain useful environment/config
overrides.
