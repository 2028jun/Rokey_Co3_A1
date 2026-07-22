# Hand-safety intrusion test: GPU-side implementation & verification prompt

Paste this whole document to a Claude Code (or equivalent coding agent) session
running **on the GPU machine that has Isaac Sim 5.1.0-rc.19 and ROS 2 Humble
installed**. Passes 1-3 have already been run and documented on that hardware;
the current assignment is **Pass 4**. Read `GPU_RUN_LOG.txt` and the "Pass 4
tasks" section first. The older pass sections remain below as historical
context and must not be mistaken for still-open work. This Pass-4 plan was
written on a machine with no GPU/Isaac Sim access, so verify every new runtime
assumption against the installed 5.1 source and actual render.

## Pass 2 changes (read this first if you already ran pass 1)

Pass 1's commit (`Fix hand-safety GPU test scaffold and default it to run
out of the box`) confirmed the pipeline wiring works but flagged that the
whole-body-slide reach never gets a hand into frame, and separately a human
reviewer noticed the published camera feed looked very low-resolution. Both
addressed here, neither verified on hardware yet:

1. **Camera resolution raised 640x480 → 1280x960**
   (`isaacpjt/mobile_manipulator_demo.py`'s `connect_table_camera_ros2()`
   `RenderProduct.inputs:width/height`, plus the matching preview viewport
   size). At 640x480, `hand_safety`'s ROI-crop-then-upscale-to-1280
   pipeline was cropping only ~289x262px out of the frame and blowing it up
   ~4.4x before YOLO ever saw it. At 1280x960 the same crop is ~579x524px,
   ~2.2x upscale. Sanity-check with `ros2 topic echo
   /serving_robot/table_camera/camera_info --once` and by eye in
   `rqt_image_view`. Raise further if render cost allows and detail is
   still the bottleneck.

2. **Real per-joint arm reach via UsdSkel**, replacing the whole-body
   slide as the primary mechanism (`isaacpjt/hand_intrusion_test_actor.py`,
   see its module docstring for the full design rationale). In short:
   `_setup_skeleton_reach()` finds `F_Business_02`'s `UsdSkel.Skeleton`,
   seeds a brand-new `UsdSkelAnimation` from its rest pose (so the
   character's baked idle animation stops overriding whatever we set —
   this was the actual cause of pass 1's hands-never-move-forward
   problem), binds it as the animation source, then each frame rotates
   whichever joint fuzzy-matches "forearm" by up to
   `HAND_TEST_REACH_JOINT_ANGLE_DEG` (default -70°) around local
   `HAND_TEST_REACH_JOINT_AXIS` (default X). None of `UsdSkel.Cache`,
   `Skeleton.GetRestTransformsAttr()`, `UsdSkel.DecomposeTransform()`, or
   `UsdSkel.BindingAPI.Apply().CreateAnimationSourceRel()` have been
   exercised against real skeleton data — if any of those calls don't
   match this Isaac Sim build's UsdSkel Python API, `ReachAnimator` catches
   the exception, prints why, and falls back to lean-only (pass 1's
   behavior) rather than crashing. Check the console output for
   `[hand_test] skeleton reach setup raised ...` / `disabling joint drive`
   to know if that happened.

   To debug: after `spawn_seated_person()` runs, call
   `hand_test.list_skeleton_joints(stage)` from the Isaac Sim script
   console to print every joint path, confirm the auto-picked forearm
   joint (logged as `[hand_test] driving joint [N] <path> ...` at startup)
   is actually the right one, and if not, override with:
   ```bash
   export HAND_TEST_REACH_JOINT_NAME="<substring from the printed path>"
   export HAND_TEST_REACH_JOINT_AXIS=X   # try X, Y, Z
   export HAND_TEST_REACH_JOINT_ANGLE_DEG=-70  # sign/magnitude, tune by eye
   ```
   The whole-body lean distance also shrank from ~1m to 15cm
   (`HAND_TEST_LEAN_DISTANCE`) since the arm is now expected to cover most
   of the reach; increase it back if the joint drive turns out not to work
   at all on this asset.

## Pass 3 tasks (read this first if you already ran passes 1-2)

Pass 2's commit (`Fix pass-2 reach: UsdSkel per-joint rotation doesn't
render, whole-body translate does; find a real ROI false-positive from the
robot's gripper`) got a hand landing on the table via a whole-body
translate, but flagged two open problems. Both are unaddressed as of this
writing, plus one already applied blind (not yet hardware-verified):

0. **Reach speed already halved, needs re-checking on hardware.**
   `REACH_TRAVEL_SECONDS` default changed `0.4` -> `0.8`
   (`hand_intrusion_test_actor.py`) per a reviewer's note that the
   character's back-and-forth motion looked too fast. This was a blind
   constant edit (no GPU access here), not re-verified on hardware —
   confirm it actually reads as a natural pace and isn't, e.g., now too
   slow to be believable, and tune `HAND_TEST_TRAVEL_SECONDS` further if
   needed.

1. **The reach still looks unnatural.** Because `F_Business_02`'s rest
   pose holds its arm up near shoulder height, bringing its hand down to
   table height means translating the *entire body*, so the character
   visibly sinks/leans across the table instead of just extending an arm.
   Per-joint `UsdSkel` rotation was already tried and ruled out (see
   `hand_intrusion_test_actor.py`'s module docstring) — late-bound
   `SkelAnimation` joint rotations don't reach the Hydra skinning pipeline
   on this Isaac Sim build (5.1.0-rc.19), confirmed with an isolated
   pixel-diff repro, regardless of whether you author through `pxr.UsdSkel`
   or the Fabric-native `usdrt.UsdSkel` API.

   **Try instead: drive the arm via a PhysX articulation joint (not
   `UsdSkel` animation).** This is a genuinely different subsystem —
   physics joint drives write joint state that PhysX itself resolves each
   step, rather than authoring a `SkelAnimation` sample for Hydra to pick
   up — so it may not hit the same late-bound-Fabric limitation.

   - First check whether `F_Business_02` (or whatever `PERSON_USD` is
     currently set to) actually has a PhysX articulation on its skeleton
     at all. Many Isaac Sim "People" background-crowd characters are
     posed/animated meshes only, with **no** `UsdPhysics.ArticulationRootAPI`
     or joint-drive schemas — if that's the case here, this approach is a
     dead end for this asset and you'll need a physics-ready humanoid
     instead (check Isaac's character/robot library for one with an
     actual articulation, not just a skeleton). Report which case you hit
     either way — a confirmed dead end is still useful information.
   - If an articulation exists: use
     `omni.isaac.core.articulations.Articulation` /
     `ArticulationView` to find the shoulder/elbow joint (fuzzy-match like
     `_find_skeleton()` already does for joint names), and drive it via
     joint position *drive targets* (respecting whatever stiffness/damping
     the asset ships with) rather than snapping the transform directly, so
     it moves like a physically actuated joint rather than teleporting.
   - Keep the existing whole-body analytic reach
     (`_compute_hand_reach_target()` / `ReachAnimator`) as the fallback if
     this doesn't pan out on this asset, matching the try/except-and-fall-
     back pattern already used for the pass-2 attempt.
   - If the articulation drive *also* doesn't pan out (no physics joints
     on any available character, or it renders with the same problem as
     UsdSkel), don't stop at two failed attempts — keep looking for
     another mechanism. One cruder but likely-reliable option: model/cut
     just a small arm+hand prop (doesn't need to be attached to or even
     resemble the seated character's own arm) and animate *that* prop
     sliding in from off-table into the ROI and back out on the reach
     cycle, independent of the body. It only needs to look plausible
     in-frame and register as a hand to YOLO — it sidesteps the
     skeleton/rendering problems entirely, since it's just a plain
     `Xformable` translate on a simple prim, the one motion primitive
     already confirmed to render reliably (pass 2's whole-body reach).
     Treat this as one option among others, not the only fallback — if
     you find a better mechanism, use it and document why in
     `GPU_RUN_LOG.txt`.

2. **The gripper false-positive still blocks a clean acceptance check.**
   `hand_safety`'s YOLO hand detector misdetects the robot's own RG2
   gripper as two "hand" objects, so `/hand_safety/roi_intrusion` reads
   `true` continuously regardless of the test person's reach — this masks
   whether intrusion timing actually correlates with the reach cycle.

   **Try first: repaint the robot matte black.** A team member has
   observed this cut down false positives significantly before (matte
   dark surfaces reduce specular highlights/reflections that appear to
   confuse the hand detector). Bind a matte-black `UsdPreviewSurface`
   (`diffuseColor` ~`(0.02, 0.02, 0.02)`, `roughness` high e.g. `0.9`,
   `metallic` `0`) to the robot's visual meshes — start with the RG2
   gripper specifically, since that's what's triggering the false
   positive, and extend to the rest of the robot if the gripper alone
   isn't enough. This is a material/`UsdShade` change, not a code change
   to `hand_safety` itself, so it's low-risk to try first.

   If repainting doesn't fully clear it, layer on: tightening
   `TABLE_ROI_NORMALIZED` in `hand_safety/hand_safety/roi_intrusion.py` to
   exclude the gripper's resting position in frame, and/or raising the
   YOLO confidence threshold against the gripper hardware. Either way,
   re-run the acceptance check (step 6 below) and confirm `roi_intrusion`
   actually flips in sync with the reach cycle, not just always-true.

## Pass 4 tasks (read this first if you already ran passes 1-3)

Pass 3 (`bb6ea4b`) fixed the RG2 false positive by tightening the table ROI
and confirmed the end-to-end acceptance signal, but the test actor still
moves its entire body to put a hand on the table. Do not repeat pass 2's
late-bound `pxr.UsdSkel`/`usdrt.UsdSkel` attribute-writing experiment or
pass 3's matte-black material experiment: both were already isolated and
ruled out on this exact Isaac Sim build.

The important distinction for this pass is:

- Isaac's People characters do have a `UsdSkel` skeleton and can perform
  per-joint skeletal animations. The official asset library contains
  `Sit.skelanim.usd`, `stand_walk_*.skelanim.usd`,
  `push_button.skelanim.usd`, `type_keyboard.skelanim.usd`, and other
  joint-animation clips.
- Those skinning joints are not PhysX articulation joints. A scan of the
  official Isaac Sim 5.1 `People/Characters` models and representative
  `People/DH_Characters` assets found `Skel`, `ControlRigAPI`, and
  Animation Graph data, but no PhysX articulation/joint/drive schemas.
- Therefore the missing PhysX articulation does **not** mean an arm cannot
  move independently. It means the supported route is the People/Animation
  Graph runtime, not an `ArticulationView` and not late USD attribute edits.
- The character's existing idle animation already renders, while pass 2's
  newly rebound `SkelAnimation` only changed `SkelQuery` results without
  changing pixels. Treat that as evidence that the runtime Animation Graph
  path is worth testing, not as evidence that the character is unrigged.

Official 5.1 asset locations (prepend the GPU machine's Isaac asset root):

```text
/Isaac/People/Characters/Biped_Setup.usd
/Isaac/People/Animations/Sit.skelanim.usd
/Isaac/People/Animations/push_button.skelanim.usd
/Isaac/People/Animations/type_keyboard.skelanim.usd
/Isaac/People/Animations/stand_walk_loop.skelanim.usd
```

The official Isaac Sim 5.1 Actor Control documentation specifically uses
`push_button.skelanim.usd` as the custom-command example and documents
`CustomCommandFilterJoint`. Use the APIs and extension sources actually
installed on this GPU machine (`omni.anim.people` 0.7.9 and the 107.3.x
Animation Graph extensions in Isaac Sim 5.1.0-rc.19); do not copy a current
Isaac Sim 6.x example and assume its names or signatures are compatible.

### 1. Replace the whole-body reach with a supported skeletal-animation path

1. Inspect the installed extension source and sample graph first. Useful
   places are the `omni.anim.people`, `omni.anim.graph.core`, and
   `omni.anim.skelJoint` extension folders plus `Biped_Setup.usd`. Log the
   exact extension versions and the actual character `SkelRoot` path.
2. Enable/setup the People character the same way the 5.1 People system
   does: apply the `Biped_Setup` Animation Graph to the character's
   `SkelRoot` and let the runtime graph own the output pose. Do not bind a
   freshly-created `UsdSkelAnimation` to the already-running character.
3. Try `push_button.skelanim.usd` first. It is the closest built-in motion
   to a forward arm/hand reach and is the official custom-command example.
   Register/play it through `omni.anim.people`/Animation Graph using the
   5.1 mechanism found in the installed source. If the official asset needs
   custom-command attributes, copy or overlay it into a repo-local test
   asset; never edit the remote NVIDIA asset in place.
4. Position and yaw the person so the animated hand travels over
   `TableSet_00`. The torso/root should remain plausibly at the chair while
   the arm supplies the visible reach. Do not declare success just because
   a command state changes: confirm the rendered arm pixels move and YOLO's
   bbox follows the hand.
5. If a seated reach is required, test `Sit` as the lower-body/base pose and
   layer the reach clip on the upper body with the graph's Filter/Blend
   mechanism (or the installed equivalent). If available in this 107.3.x
   build, a Two Bone IK arm chain targeting the tabletop is also a valid
   next experiment. Verify node availability in the installed build first.
6. If `push_button` is unsuitable, try `type_keyboard` and other compatible
   built-in People animations before falling back to the current analytic
   whole-body translation. Record each clip tried and why it passed or
   failed. Keep the existing whole-body implementation available only as a
   diagnostic fallback while the new path is being validated.

### 2. Test multiple human models, not only F_Business_02

Run the same reach experiment with **at least three** visibly different
People assets, including `F_Business_02` and at least two of the following
official 5.1 candidates:

```text
/Isaac/People/Characters/F_Business_02/F_Business_02.usd
/Isaac/People/Characters/F_Medical_01/F_Medical_01.usd
/Isaac/People/Characters/M_Medical_01/M_Medical_01.usd
/Isaac/People/Characters/female_adult_police_01_new/female_adult_police_01_new.usd
/Isaac/People/Characters/female_adult_police_03_new/female_adult_police_03_new.usd
/Isaac/People/Characters/male_adult_construction_01_new/male_adult_construction_01_new.usd
/Isaac/People/Characters/male_adult_construction_05_new/male_adult_construction_05_new.usd
```

Optionally include one `People/DH_Characters/<uuid>/<uuid>.usd` Digital
Human if it loads within the available GPU/memory budget. Do not spend the
whole pass downloading every Digital Human variant.

For every tested model, record:

- whether the URL/path resolves and the model fully loads;
- its `SkelRoot` and whether `Biped_Setup`/retargeting accepts it;
- whether the selected clip visibly moves the arm without translating the
  whole body;
- whether the rendered hand is realistic/large enough for the HaGRIDv2
  model and the observed `/hand_detection/detections` confidence/bbox;
- whether the hand enters the tightened ROI without reintroducing the RG2
  false positive; and
- whether the hand/table geometry visibly intersects.

Do not reuse F_Business_02's hardcoded `_RIGHT_HAND_REST_LOCAL_OFFSET` for
another character. The Animation Graph path should not need that constant;
if any fallback still does, measure and store a per-asset value. Select the
best-performing model as the new default only after comparing the results,
and keep `HAND_TEST_PERSON_USD` override support for the others.

### 3. Give the hand up to 10 cm of table clearance, but only if needed

The table surface is `TABLE_TOP_Z = 0.73`. The current analytic fallback's
`TABLE_HAND_TARGET` places the measured hand-joint origin at exactly that Z,
which can put the lower part of the hand mesh inside the tabletop.

Before changing height, inspect the current code, the actual animation/IK
target, and a side/close camera capture. If the hand is already visibly
clear of the table (or an earlier change already targets about
`TABLE_TOP_Z + 0.10`), **do not add another 10 cm** and do not make a
height-only churn commit.

If the hand intersects or nearly intersects the tabletop, make its final
reach height exactly 10 cm above the tabletop:

```text
desired hand target Z = TABLE_TOP_Z + 0.10 = 0.83 m
```

Apply this to the hand/IK/interaction target, not to the whole chair, table,
or resting character root. For the analytic fallback, prefer one clearly
named, environment-overridable clearance such as
`HAND_TEST_TARGET_Z_OFFSET` and compute the target from `TABLE_TOP_Z`; do
not stack an extra `+0.10` on top of an already-raised target. Capture the
before/after view and report the actual world-space target Z used.

### 4. Pass-4 acceptance and deliverables

Acceptance requires all of the following:

- At least three People models were actually rendered and compared.
- At least one model performs a visibly arm-driven reach; the whole torso
  must not sink/slide onto the table as the primary motion.
- The chosen model's hand clears the tabletop without an excessive gap. If
  a height correction was required, its target is `0.83 m`, not 10 cm on
  top of some other correction.
- With the person held at rest for at least 20 seconds,
  `/hand_safety/roi_intrusion` remains false and the old RG2 false positive
  stays excluded.
- Across several reach cycles, `roi_intrusion` becomes true in sync with
  the rendered hand entering the ROI and returns false after retraction.
- Camera input remains 1280x960 and `table_arrived` gating still works.

Append a detailed **Pass 4** entry to `GPU_RUN_LOG.txt`. Include the exact
asset paths and clip/graph mechanism for every tested person, the table
clearance decision (changed or deliberately left unchanged), observed
detection confidences/bboxes, intrusion timing, failed attempts, and the
final chosen default. Commit the log with the implementation and push it to
`vision-test`; do not leave successful GPU-only changes uncommitted.

Before finishing, append your results to `GPU_RUN_LOG.txt` (repo root) —
see "Keep GPU_RUN_LOG.txt updated" near the end of this doc.

## Goal

At `TableSet_00` (the only table the fixed table camera ever frames — see
"Facts" below), spawn a seated person who periodically extends an arm so a
hand enters the tabletop, on a randomized 5-10 second interval. Use this to
exercise the `hand_safety` package's ROI-intrusion detector end to end,
without a physical camera or a real hand.

## Setup

```bash
git clone https://github.com/2028jun/Rokey_Co3_A1.git
cd Rokey_Co3_A1
git checkout vision-test
```

This branch already contains a scaffold (see "What's already done" below).
It was assembled by diffing several team branches and taking the
most-recently-updated version of each restaurant/robot file, then adding a
new, untested module for the reach animation.

## Facts already confirmed (no need to re-derive)

- `assets/lightweight_restaurant/lightweight_pizza_restaurant.usda` has 4
  table groups: `TableSet_00`, `TableSet_01`, `TableSet_02`, `TableSet_03`,
  each with 4 chairs. There is no explicit "table number" field anywhere —
  numbering is positional by prim name.
- `TableSet_00` is the table used everywhere else in this codebase
  (`isaacpjt/mobile_manipulator_demo.py:93-97`: the robot always docks at
  `SPAWN_POSITION = Gf.Vec3d(-1.82, -2.20, 0.002)`, 8cm from `TableSet_00`'s
  clear edge, and `TABLE_CAMERA_PATH` is mounted on the robot at that dock
  pose). Since the fixed `table_camera` ROS topic is only ever whatever the
  docked robot sees, `TableSet_00` is effectively "table 1" for this
  workspace — treat it as such.
- `TableSet_00/TableCollider`: center `(-3.2, -2.2, 0.365)`, scale
  `(1.8, 0.94, 0.73)` → table top surface ≈ world Z `0.73`.
- `TableSet_00/Chair_00_Visual`: translate `(-3.7, -3.2, 0)`, `rotateZ=180`
  (facing the table). This is the chair the scaffold seats the person in —
  **not verified to be inside the camera's actual field of view**; the
  robot docks from the table's +X/east side (`x=-1.82` vs table `x=-3.2`),
  so a chair on a different side may frame better. `Chair_01`/`Chair_02`/
  `Chair_03` under the same `TableSet_00` block are alternatives if
  `Chair_00` turns out to be off-frame or blocked by the robot.
- `hand_safety/hand_safety/roi_intrusion.py`: `TABLE_ROI_NORMALIZED` is a
  fixed 4-point polygon in 0-1 normalized image space, independent of which
  table — it was calibrated once against this same fixed camera. A comment
  in `hand_safety/config/hand_safety.yaml:9` says *"The rendered table-2
  hand measures about 0.44-0.77 across live frames"*, implying a synthetic
  hand was already tested against this exact ROI previously (no script for
  that survives in the repo). So a hand appearing anywhere on
  `TableSet_00`'s surface, in the robot camera's frame, should already fall
  inside the calibrated ROI without further tuning — assuming it's placed
  and framed similarly to whatever that prior test used.
- `hand_safety` only runs inference while `/serving_robot/table_arrived` is
  `true`, and requires `confirmation_frames=3` consecutive in-ROI detections
  at `process_rate=30Hz` (~`0.1s`) before it reports
  `/hand_safety/roi_intrusion=true`. Keep any reach's "hold" phase
  comfortably longer than that.
- **No human/character USD asset exists anywhere in this repo or in any of
  the team's other branches** (`hmi-web`, `jaehyeon`, `main`, `test`,
  `woduq`, `younggi` were all checked). You will need to source a real
  Isaac Sim People character from the content browser / Nucleus server on
  this machine — nothing here was verified to exist.
- The codebase has **no prior UsdSkel / keyframe-animation code** anywhere.
  All existing prim motion in this repo is done by imperatively calling
  `UsdGeom.XformCommonAPI(prim).SetTranslate(...)` once per
  `simulation_app.update()` tick inside the main loop (see
  `isaacpjt/M0609/moveit_bridge_sim.py:108-110` and
  `isaacpjt/mobile_manipulator_demo.py:167-168` for the pattern already in
  use).

## What's already done (untested scaffold)

- `isaacpjt/hand_intrusion_test_actor.py` (new file): references a person
  USD at `TableSet_00/Chair_00`'s seat position and rigidly translates the
  *whole referenced prim* between a seated rest pose and a pose at the
  table's hand-target position, on a randomized 5-10s period (smoothstep
  ease in/out, configurable hold duration). This is a deliberate
  simplification — it slides the whole body toward the table rather than
  animating an isolated arm/hand joint, because no character skeleton is
  available here to inspect joint names against. If a real arm-only reach
  is wanted, that requires opening the actual character in Isaac Sim,
  reading its skeleton's joint list (Stage tree / Property window), and
  driving the forearm/hand joint's local rotation directly — out of scope
  for what could be built blind.
- `isaacpjt/mobile_manipulator_demo.py`: three small additions, all gated
  behind `MOBILE_DEMO_HAND_TEST=1` so default behavior is unchanged:
  - `import hand_intrusion_test_actor as hand_test` near the top.
  - After `open_table_camera_preview()` in `main()`: spawns the person and
    creates a `ReachAnimator` if the env var is set.
  - In the main `while simulation_app.is_running():` loop: calls
    `reach_animator.update()` each frame.
- `PERSON_USD` in `hand_intrusion_test_actor.py` defaults to a **guessed**
  path (`.../Isaac/5.1/Isaac/People/Characters/female_adult_business_02/
  female_adult_business_02.usd`), overridable via env var
  `HAND_TEST_PERSON_USD`. This guess mirrors the path pattern already used
  in this repo for `D455_ASSET_USD`, but was never confirmed to resolve —
  **treat it as very likely wrong** and replace it with whatever People
  asset actually exists in this machine's content browser.
- Also on this branch (unrelated to the person, brought over from the
  `woduq` branch as the most-recently-updated versions found across the
  team's branches — flagging in case anything looks unfamiliar):
  restaurant asset `lightweight_pizza_restaurant.usda`, and the
  `ridgeback_m0609_description` package's `urdf`/`urdf.xacro`/
  `nav2_footprint.yaml`/`package.xml`/`validate_description.py`. The robot's
  cached USD (`assets/mobile_manipulator/ridgeback_m0609_v2.usd`) was
  renamed to `ridgeback_m0609_v2.usd.stale-bak` so
  `isaacpjt/mobile_manipulator_demo.py`'s importer regenerates it from the
  new urdf on first run instead of silently reusing the stale cache.

## Your task

1. **Build the workspace.** The urdf/xacro changed, so a clean
   `colcon build --symlink-install` (from repo root) is needed before the
   URDF importer's `package://ridgeback_m0609_description/...` mesh paths
   will resolve. `source install/setup.bash` afterward.

2. **Find a real People asset.** Open Isaac Sim's content browser, locate an
   available seated or standing human character (search "People" /
   "Character" in the Nucleus asset tree), and set:
   ```bash
   export HAND_TEST_PERSON_USD="omniverse://<your-actual-path>.usd"
   ```
   If nothing off-the-shelf looks seated, a standing character is fine —
   just adjust `SEAT_TORSO_Z` / `SEAT_XY` in
   `isaacpjt/hand_intrusion_test_actor.py` so it doesn't clip through the
   chair or table.

3. **Run the sim with the test actor enabled:**
   ```bash
   export ROS_DOMAIN_ID=101
   MOBILE_DEMO_HAND_TEST=1 python isaacpjt/mobile_manipulator_demo.py
   ```
   Watch the viewport: confirm the person appears at the chair, faces the
   table, and is inside the robot-mounted table camera's frame. Adjust
   `SEAT_XY` / `SEAT_YAW_DEGREES` / chair choice in
   `hand_intrusion_test_actor.py` if not.

4. **In a second terminal, run hand_safety:**
   ```bash
   source install/setup.bash
   export ROS_DOMAIN_ID=101
   ros2 run hand_safety hand_detector_node --ros-args -p publish_annotated_image:=true
   ```

5. **In a third terminal, verify the pipeline is actually wired up:**
   ```bash
   ros2 topic hz /serving_robot/table_camera/color/image_raw
   ros2 topic echo /serving_robot/table_arrived
   ros2 run rqt_image_view rqt_image_view /hand_detection/image
   ros2 topic echo /hand_safety/roi_intrusion
   ```

6. **Acceptance check:** once `table_arrived` is `true`, confirm
   `/hand_safety/roi_intrusion` flips to `true` roughly once every 5-10
   seconds (matching `hand_intrusion_test_actor.py`'s `MIN_PERIOD_SECONDS`/
   `MAX_PERIOD_SECONDS`) for roughly the reach's hold duration, and stays
   `false` at rest. Use the annotated `/hand_detection/image` view to see
   whether YOLO is actually drawing a hand box on the person's hand, or
   missing it (wrong asset, wrong framing, wrong scale).

7. **Iterate.** This scaffold is a first draft, not a known-good
   implementation — expect to fix the asset path, seat position/orientation,
   and possibly reach target coordinates. If YOLO never fires on the
   person's hand even when it's clearly on the table, the character's
   hand geometry may be too stylized/low-poly for the HaGRIDv2-trained
   model — try a different, more realistic-handed character asset before
   assuming the ROI/wiring is broken.

## Keep GPU_RUN_LOG.txt updated

`GPU_RUN_LOG.txt` (repo root) is a plain-text, human-readable log of every
hardware pass, so anyone following this branch can see what was tried and
what happened without reconstructing it from `git log`. It already has
entries for passes 1-2 — read it for context before you start.

Before you finish (or push any commit), append a new dated entry in the
same format:

```
## Pass N -- YYYY-MM-DD (commit <short-hash-once-committed>)

Tested:
- <what you ran / checked>

Changed:
- <files and the substance of the change, one line each>

Observed:
- <actual results -- topic echo output, timing, screenshots described in
  words, whatever you saw on hardware>

Still broken / follow-up:
- <anything left open for the next pass>
```

Commit `GPU_RUN_LOG.txt` together with (or right after) your code changes
and push to `vision-test` so it's visible on the branch immediately —
don't leave it for a later pass to backfill.

Report back what you changed and the final observed `roi_intrusion` timing
so the scaffold constants can be corrected for next time.
