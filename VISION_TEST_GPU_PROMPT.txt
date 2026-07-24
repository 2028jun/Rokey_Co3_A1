# Vision-test GPU prompt: root-cause the actor_sdg character-registration blocker

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
- Read `AGENTS.md`, `GPU_RUN_LOG.txt` (all passes, especially pass 9),
  `ANIM_SPIKE_RESULTS.txt`, `isaacpjt/actor_sdg_test_actor.py`, and
  `isaacpjt/mobile_manipulator_demo.py`'s `capture_actor_sdg_frames()` and
  `enable_extensions()` docstring completely before changing anything.
  Confirm for yourself that `HAND_TEST_RIG_MODE` still defaults to
  `"rigid_arm"` -- pass 9 could not get the `actor_sdg` mechanism working in
  the real pipeline (see below) and left `rigid_arm` as the shipped default
  rather than ship something broken. If that has changed by the time you
  read this, the direction below still applies, just adjust the starting
  point accordingly.

## Current state (do not re-litigate)

Pass 9 tried to replace `hand_intrusion_test_actor.py`'s rigid-arm
two-bone-IK mechanism with a real `omni.anim.people` character (a
typing -> sit -> push_button -> sit loop at a real `TableSet_00` chair),
per the reviewer's explicit call to discard rigid-arm entirely. That
integration is BLOCKED by a confirmed, reproducible bug, not a design
question:

- `omni.anim.graph.core`'s character registration (`ag.get_character()`,
  which `CharacterBehavior.init_character()` needs to do anything at all)
  never registers the actor_sdg character when run inside
  `mobile_manipulator_demo.py`'s full restaurant+robot pipeline:
  - Enabling `omni.anim.people`/`isaacsim.replicator.agent.core`/
    `omni.anim.graph.core` BEFORE the restaurant stage's first
    `open_stage()` reproducibly segfaults during that open (see the crash
    signature in `GPU_RUN_LOG.txt` pass 9 -- a `CharacterManager::
    Shutdown()`-without-`Initialize()` warning immediately followed by the
    fatal signal while loading the restaurant's Lightwheel_Kitchen
    sublayer).
  - Enabling them AFTER the stage is open avoids the crash, but
    `ag.get_character()` then never resolves -- confirmed stuck at `None`
    after 60+ real seconds, after toggling the extension off/on, and
    after re-applying `AnimationGraphAPI` post-toggle.
  - The identical `actor_sdg_test_actor.spawn_and_configure_actor()` call,
    run against a BARE restaurant stage with no robot/URDF/physics
    pipeline, resolves `ag.get_character()` one frame after `play()`,
    every time. So the mechanism itself is not broken -- something
    specific to the full pipeline blocks registration.
- Pass 9 ruled out (individually, against the bare stage, without
  reproducing the bug): `enable_urdf_importer()` alone, the robot's USD
  reference alone, and extension-enable-vs-timeline-play ordering alone.
- Two concrete suspects pass 9 did NOT get to: (1) the extra top-level
  imports `mobile_manipulator_demo.py` has that isolated test scripts
  don't (`numpy`, `omni.graph.core`, `isaacsim.core.utils.viewports`,
  etc.) -- bisect by adding them one at a time to a bare-stage isolated
  script until the bug reproduces; (2) the pre-existing "Could not load
  sublayer ... metricsAssembler" warning for the restaurant's
  Lightwheel_Kitchen asset, which appears in every pass's log regardless
  of this change -- try fixing/removing that broken sublayer reference
  directly and see if the crash-on-early-enable disappears, since the
  crash happens immediately after that exact warning.
- Pass 9 also got one piece of positive evidence in isolation: `Sit`
  (a built-in omni.anim.people command, not the CustomCommand mechanism)
  really does walk to and sit at a plain, untagged `Chair_01_Visual` with
  no special authoring, confirmed by a captured frame showing the
  character's feet tucked under the chair. That capture's camera had a
  framing bug (a wall corner clipped most of the view), so overall pose
  quality during sit/typing/push_button was not actually confirmed either
  way -- still open.

## Required work

Two valid ways forward -- pick based on what you find, don't force a
predetermined outcome:

1. **Root-cause and fix the registration blocker**, using the two
   suspects above (or others you find) as a starting point. If you fix
   it, then actually run pass 8's original four goals (still in
   `GPU_RUN_LOG.txt` pass 9's summary of the prompt that started this
   pass, and in the pre-pass-9 git history of this file if you need the
   exact original wording) against the real pipeline: seat the actor via
   `Sit`, drive the typing/sit/push_button loop, measure and add an
   adaptive walk-in if `push_button`/`type_keyboard` doesn't bring the
   hand into `TABLE_ROI_NORMALIZED`, and determine with real evidence
   whether `hand_safety` detects the hand when it's genuinely in the ROI.
   Only then should `HAND_TEST_RIG_MODE`'s default change away from
   `"rigid_arm"`.
2. **If the blocker turns out to be a genuine, unresolvable engine
   limitation** (not just something pass 9 didn't try hard enough),
   say so plainly with your own evidence, and either leave `rigid_arm` as
   the permanent default with `actor_sdg` as a documented, known-blocked
   opt-in, or propose a different integration approach entirely (e.g.
   driving the AnimationGraph's "Action" variable through a lower-level
   OmniGraph node network instead of `CharacterBehavior`/`ag.get_character()`,
   if you can determine that path doesn't depend on the same character
   registration).

Either way, while investigating, first fix the isolated capture's QA
camera framing bug (see `isolate_sit_test.py`-style approach in pass 9's
work, not committed -- rebuild it under `isaacpjt/` if useful) so a
future pass can actually see whether sit/typing/push_button poses look
natural, independent of the registration blocker.

## Iterate until it actually meets the criteria

A single attempt that "runs" is not the deliverable. If a fix doesn't
visually and numerically resolve the blocker, try a different approach and
re-render/re-test until it genuinely does -- not until the code merely
executes without error.

Never paper over a real defect to make it look finished: don't claim
`ag.get_character()` works without printing its actual return value in the
real pipeline, don't lower YOLO confidence or move/shrink the ROI without a
measured reason, and don't claim a criterion is met without an actual
rendered frame or real topic echo proving it. An honest "still broken,
here's what I tried and why it didn't work" is the correct outcome if
that's the truth; a fake pass is not.

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
4. Report the commit hash, changed files, exact run commands, and whether
   the registration blocker was fixed, worked around, or confirmed as a
   genuine engine limitation.
