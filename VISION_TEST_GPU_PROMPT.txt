# Hand-safety intrusion test: GPU-side implementation & verification prompt

Paste this whole document to a Claude Code (or equivalent coding agent) session
running **on the GPU machine that has Isaac Sim 5.1.0-rc.19 and ROS 2 Humble
installed**. It was written on a machine with no GPU/Isaac Sim access, so
nothing in the scaffold below has been run or visually checked. Your job is to
verify it, fix whatever is wrong, and confirm the hand_safety pipeline
actually fires on the simulated reach.

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

Report back what you changed and the final observed `roi_intrusion` timing
so the scaffold constants can be corrected for next time.
