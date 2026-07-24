# Vision-test GPU prompt: fix the actor_sdg sit/stand rotation bug and the hand/ROI mismatch

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
- Read `AGENTS.md`, `GPU_RUN_LOG.txt` (all passes, especially 9-12),
  `ANIM_SPIKE_RESULTS.txt`, `isaacpjt/actor_sdg_test_actor.py`, and
  `isaacpjt/mobile_manipulator_demo.py`'s `main()`, `capture_actor_sdg_
  frames()`, `_find_hand_joint_world_positions()`, and `enable_extensions()`
  docstring completely before changing anything. Confirm for yourself that
  `HAND_TEST_RIG_MODE` still defaults to `"rigid_arm"` -- pass 10 fixed the
  registration blocker that made `actor_sdg` non-functional, but passes 11
  and 12 found (not fixed) a rotation-drift bug and a hand/camera/ROI
  geometry mismatch, so the default has still not been flipped. If that has
  changed by the time you read this, the direction below still applies,
  just adjust the starting point accordingly.

## Current state (do not re-litigate)

Passes 9-10 replaced `hand_intrusion_test_actor.py`'s rigid-arm two-bone-IK
mechanism with a real `omni.anim.people` character (a
typing -> sit -> push_button -> sit loop at a `TableSet_00` chair). Pass 11
investigated the user's GUI-testing reports of the character not visibly
sitting and lying on the floor afterward. Pass 12 investigated the user's
third report: confirm whether the hand is actually detected by the real
`hand_safety`/YOLO pipeline during `type_keyboard`.

### Sit/stand rotation drift (pass 11, still open)

- The mechanism WORKS end to end mechanically: `ag.get_character()`
  registers reliably, the command queue executes and loops correctly, and
  the character's real position (read via `ag.Character.get_world_
  transform()`, NOT the outer spawned Xform's own `xformOp:translate`,
  which never changes and is a red herring) walks to and settles exactly
  at the chair's own position with the correct facing rotation. The
  "doesn't look like it sits" complaint is NOT a positional bug.
- **There is a real, confirmed, NOT-yet-fixed rotation-drift bug**: on the
  SECOND `Sit` command in the `type_keyboard -> Sit -> push_button -> Sit
  -> GoTo` loop (not the first), the character's world rotation tips off
  pure-Z-yaw during the "stand" sub-phase (quaternion x growing to roughly
  -0.19), visibly and numerically confirmed, making the character appear
  to collapse/lie on the floor. The first sit-then-stand cycle in the loop
  is clean; only the second is affected.
- **Ruled out** (each confirmed by a real test, not assumed): `Sit.update()`
  re-reading live rotation every frame during "stand" instead of a frozen
  value (fixed via a monkey-patch, verified internally to actually hold
  the freeze -- no change to the external symptom); the frozen rotation
  being captured from a transient mid-turn snapshot instead of the stable
  chair-geometry-derived `self.interact_rot` (tried, no change); and
  insufficient settle time between the two `Sit` commands (doubled
  `SIT_DURATION_SECONDS` from 3.0 to 6.0, no change). All three produced
  byte-for-byte identical drift, proving the drift is NOT controlled by
  `Sit.update()`'s Python-level `set_world_transform()` calls at all --
  something inside `omni.anim.people`'s AnimationGraph itself is
  re-deriving/overriding rotation on the second sit-cycle specifically.
- A monkey-patched `_patch_sit_command_stand_rotation()` now correctly
  patches the REAL `Sit` class Kit actually uses at runtime (previously it
  silently patched the wrong class -- Kit's extension loader imports
  `omni.anim.people.scripts.commands.sit` under a mangled, install-path-
  specific module name, different from what a normal top-level import
  resolves to; the fix scans `sys.modules` for every module ending in
  `commands.sit`). This patch is real, correct, and worth keeping, but by
  itself does not fix the observed drift -- see above.
- A per-frame "snap rotation to pure Z" watchdog
  (`_enforce_upright_rotation()` in `mobile_manipulator_demo.py`, gated
  off by default via `MOBILE_DEMO_UPRIGHT_WATCHDOG=1`) DOES hold rotation
  upright, but introduces a different defect instead: the character's Z
  position sinks to around -0.19 m (into the floor) during the same
  window. Not a working fix as-is.

### Hand detection during typing (pass 12, confirmed negative, still open)

- `_find_hand_joint_world_positions()` had two real, previously-undetected
  bugs (fixed in pass 12): it looked up the character's `Skeleton` prim via
  `UsdSkel.BindingAPI(skelroot_prim).GetSkeletonRel().GetTargets()`, which
  is empty on this asset even though a `Skeleton`-typed prim is a direct
  child (now found by walking descendants instead); and
  `ComputeJointWorldTransforms()` was called with a bare `Usd.TimeCode`
  instead of a required `UsdGeom.XformCache`, which crashed the whole
  process the moment the first bug's fix let execution actually reach it.
  Also broadened its joint-name filter to match "wrist" as well as "hand"
  -- this character's skeleton has `L_Wrist`/`R_Wrist` joints with finger
  children directly, no joint literally named "Hand" anywhere. The
  function now genuinely works for the actor_sdg character for the first
  time.
- **Confirmed negative, with evidence:** the hand is NOT currently detected
  by `hand_safety` during `type_keyboard`. Measured the real wrist joint's
  world position projected through the table camera during typing:
  normalized x roughly [-0.20, 0.14], y roughly [0.49, 0.97]. The
  configured `TABLE_ROI_NORMALIZED` (`hand_safety/hand_safety/
  roi_intrusion.py`) is x=[0.10, 0.32], y=[0.28, 0.52] -- no overlap, and
  that ROI's own comments already say it was measured for the OLD
  rigid-arm rig's reach position, never re-measured for actor_sdg. Ran the
  real `hand_detector_node` against the live camera topic across two runs
  spanning full `type_keyboard` windows: 76 combined samples, every one
  `"count": 0, "roi_intrusion": false`. Saved actual annotated frames
  confirming why: the table camera's fixed position and the character's
  stand position put the character's leg filling most of the frame, with
  its hands/upper body not usefully in view at all, and the ROI box drawn
  over an empty part of the room nowhere near the character.

## Required work

1. **Find and fix the actual root cause of the second-cycle rotation
   drift.** Leads worth checking, in rough priority order: (a) whether
   `InteractableObjectHelper.add_owner()`/`remove_owner()` re-acquiring
   the SAME chair prim a second time in one loop leaves stale ownership
   state that affects the animation graph's blend; (b) whether the
   `omni.anim.people` "Action" variable or some other animation-graph
   variable has residual state from the first sit-stand cycle that
   isn't fully reset before the second `Sit` command starts (e.g. try
   explicitly setting `Action` to some neutral/idle value with a real
   gap, or querying whatever state/variable introspection
   `omni.anim.graph.core`/`omni.anim.people` exposes, right before the
   second `Sit` begins, and compare it against the first cycle's state at
   the equivalent point); (c) if the underlying default biped character
   asset's AnimationGraph genuinely has a defect that can't be worked
   around from Python, consider whether the demo needs two `Sit` calls at
   the same chair per loop at all, or whether one sit (or alternating
   between two different chairs) sidesteps the bug entirely -- if so,
   propose and implement that restructuring, with evidence that it's
   clean.
2. **If pursuing the watchdog approach further**, it must also correct
   the Z-sink side effect (e.g. hold Z at a known-good value using the
   same style of correction used for rotation), not just rotation, before
   it counts as a fix. Don't ship a "upright but sinking into the floor"
   result as done.
3. **Fix the hand/camera/ROI geometry mismatch found in pass 12.** Using
   the now-working `_find_hand_joint_world_positions()` +
   `_project_to_normalized()`, either: (a) use
   `actor_sdg_test_actor.py`'s existing `HAND_TEST_WALK_IN_BEFORE`/
   `HAND_TEST_WALK_IN_X`/`HAND_TEST_WALK_IN_Y` mechanism (already wired,
   never yet exercised with real measurements) to move the character to a
   stand position where the hand actually projects both inside the table
   camera's frame and inside a (possibly also-updated)
   `TABLE_ROI_NORMALIZED`; or (b) if no reachable stand position puts the
   hand usefully in this fixed camera's frame at all (pass 12's frames
   suggest the camera's own placement may be the deeper problem, not just
   the character's stand position), reconsider the table camera's
   placement itself. Iterate with real projected-coordinate measurements
   and real annotated frames until the hand genuinely lands in both the
   frame and the ROI -- don't guess at an offset and call it done.
4. **Re-confirm hand detection** with the real four-terminal `hand_safety`
   workflow (per `hand_safety/README.md`) once 3 is fixed: determine via
   `/hand_detection/image` and `/hand_detection/detections`/`/hand_safety/
   roi_intrusion` whether the hand is now actually detected by YOLO during
   `type_keyboard`, with real topic samples and annotated frames as
   evidence, not assumption.
5. **Confirm pose quality by eye** with the existing character-tracking
   camera (`_update_tracking_camera()`, already built and confirmed
   working in pass 11): rest, typing, sitting, and push_button should all
   look like a natural person, not a T-pose, not collapsed/clipping into
   the floor or furniture, through at least two full sit-then-stand
   cycles (the first alone is not sufficient -- the rotation bug is
   specifically on the second).
6. **Re-confirm the "doesn't look like it sits" complaint** by eye once 1/2
   and 5 above are genuinely fixed -- pass 11's position data says it's
   very likely not a separate bug, just masked by the rotation drift, but
   don't assume that without looking.
7. **Only after 1-6 hold**, flip `HAND_TEST_RIG_MODE`'s default from
   `"rigid_arm"` to `"actor_sdg"` in `mobile_manipulator_demo.py`.

## Iterate until it actually meets the criteria

A single attempt that "runs" is not the deliverable. If a fix doesn't
visually and numerically satisfy the criteria above, keep adjusting and
re-render/re-test until it genuinely does -- not until the code merely
executes without error.

Never paper over a real defect to make it look finished: don't flip
`HAND_TEST_RIG_MODE`'s default without actually confirming pose quality by
eye across at least two sit-cycles AND real hand detection, don't claim the
rotation bug is fixed just because one experiment happened not to trigger
it, and don't claim a criterion is met without an actual rendered frame or
real topic echo proving it. Item 4 explicitly allows -- requires --
reporting a genuine negative result if that's what you find; do not
convert it into a false pass.

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
