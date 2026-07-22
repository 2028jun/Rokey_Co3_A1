# Vision-test GPU execution prompt: natural hand reach and 5 FPS diagnosis

Paste this document into a coding-agent session running on the GPU computer.
The target environment is Isaac Sim 5.1.0-rc.19, ROS 2 Humble, and an RTX
5080 with 16 GB VRAM. Work on the `vision-test` branch.

Read `GPU_RUN_LOG.txt` before changing anything. It contains the detailed
Pass 1-3 history; Git history preserves the old implementation plans. This
prompt contains only the current work. Do not copy old pass narratives back
into it.

## Repository bootstrap

Before working, run `git status` and preserve any existing uncommitted GPU-side
work; never discard it with reset, checkout, or deletion. Fetch `origin`, switch
to `vision-test`, and update it with `git pull --ff-only origin vision-test`.
If local work prevents a fast-forward, integrate it safely instead of
overwriting it. Then read this prompt and `GPU_RUN_LOG.txt` completely and
execute the work end to end. Do not stop after planning or after one test case.

When finished, update `GPU_RUN_LOG.txt`, commit the tested code and log, push
`vision-test`, and report the commit hash, changed files, measured results,
remaining limitations, and the identified 5 FPS bottleneck.

## Objective

1. Replace the seated test person's whole-body slide/sink with a natural,
   visibly arm-driven reach into `TableSet_00`'s hand-safety ROI.
2. Test the reach with at least three different Isaac People models.
3. Prevent the hand mesh from intersecting the table, adding at most 10 cm of
   clearance and only when the current target is not already high enough.
4. Diagnose why integrated RGB hand detection runs at about 5 FPS despite the
   RTX 5080, then recommend settings based on isolated measurements.

Keep the current ROS domain configuration. It is working and domain-ID
unification is a later cleanup, not part of this task.

## Known results: do not repeat these dead ends

- Pass 3 fixed the RG2 gripper false positive by tightening the table ROI and
  confirmed the end-to-end `/hand_safety/roi_intrusion` signal.
- Direct late-bound `pxr.UsdSkel` and `usdrt.UsdSkel` animation-attribute edits
  changed `SkelQuery` state but did not change rendered pixels in this exact
  Isaac Sim build. Do not repeat that experiment.
- Repainting the gripper matte black was tested and did not solve the false
  positive. Do not repeat it.
- Isaac People characters have `UsdSkel` skinning joints and working skeletal
  animation clips, but the scanned People assets do not expose PhysX
  articulation/joint-drive schemas. Missing PhysX articulation does not mean
  individual limbs cannot move; use the People/Animation Graph runtime.
- The current whole-body analytic translation works as a diagnostic fallback,
  but it is not an acceptable final natural-motion implementation.
- The table-camera source is 1280x960. With ROI inference enabled, the ROI crop
  is still resized/letterboxed to YOLO `image_size:=1280`; cropping alone does
  not make the inference equivalent to 640.

## Relevant scene and pipeline facts

- The robot docks at `TableSet_00`; this is the table framed by the fixed table
  camera and treated as table 1 in this project.
- `TableSet_00` tabletop world height is approximately `TABLE_TOP_Z = 0.73 m`.
- Hand inference is gated by `/serving_robot/table_arrived` and requires three
  consecutive in-ROI detections by default.
- Main files:
  - `isaacpjt/mobile_manipulator_demo.py`
  - `isaacpjt/hand_intrusion_test_actor.py`
  - `hand_safety/hand_safety/hand_detector_node.py`
  - `hand_safety/hand_safety/roi_intrusion.py`
  - `hand_safety/config/hand_safety.yaml`
- Official 5.1 assets to inspect, relative to the installed Isaac asset root:

  ```text
  /Isaac/People/Characters/Biped_Setup.usd
  /Isaac/People/Animations/Sit.skelanim.usd
  /Isaac/People/Animations/push_button.skelanim.usd
  /Isaac/People/Animations/type_keyboard.skelanim.usd
  /Isaac/People/Animations/stand_walk_loop.skelanim.usd
  ```

Verify all APIs, paths, and extension versions against the installed 5.1
source. Do not assume a current Isaac Sim 6.x example is API-compatible.

## Task 1: implement a natural arm-driven reach

1. Inspect the installed `omni.anim.people`, `omni.anim.graph.core`, and
   `omni.anim.skelJoint` sources plus `Biped_Setup.usd`. Record exact extension
   versions, the selected character's `SkelRoot`, graph path, and available
   commands/nodes.
2. Set the character up through the same Animation Graph path used by Isaac
   People. Let the runtime graph own the output pose; do not bind a newly
   created late USD `SkelAnimation` to the running character.
3. Try `push_button.skelanim.usd` first because it is the closest built-in
   forward reach and is used by the official Actor Control custom-command
   example. If customization is needed, copy/overlay the asset locally; never
   edit the NVIDIA asset in place.
4. Position and yaw the person so the animated hand enters the tabletop ROI
   while the torso remains plausibly at the chair. Confirm rendered arm pixels
   move and the YOLO bounding box follows the hand; a command-state change by
   itself is not proof.
5. If a seated reach is required, test `Sit` as the base/lower-body pose and
   layer the reach on the upper body using the installed Filter/Blend
   mechanism. A Two Bone IK arm target is also valid if that node exists in
   this build; verify availability first.
6. If `push_button` is unsuitable, try `type_keyboard` and other installed
   compatible clips. Record every attempted mechanism and why it passed or
   failed.
7. Keep the whole-body translation only as an explicit diagnostic fallback.
   If all Animation Graph routes fail, a separately animated realistic arm/hand
   prop is an acceptable last fallback, but document why it was necessary.

## Task 2: compare multiple People models

Test at least three visibly different models, including `F_Business_02` and at
least two candidates below. Resolve paths against the actual installed asset
root rather than assuming every candidate exists.

```text
/Isaac/People/Characters/F_Business_02/F_Business_02.usd
/Isaac/People/Characters/F_Medical_01/F_Medical_01.usd
/Isaac/People/Characters/M_Medical_01/M_Medical_01.usd
/Isaac/People/Characters/female_adult_police_01_new/female_adult_police_01_new.usd
/Isaac/People/Characters/female_adult_police_03_new/female_adult_police_03_new.usd
/Isaac/People/Characters/male_adult_construction_01_new/male_adult_construction_01_new.usd
/Isaac/People/Characters/male_adult_construction_05_new/male_adult_construction_05_new.usd
```

Optionally test one `People/DH_Characters/<uuid>/<uuid>.usd` Digital Human if
it fits the available memory; do not spend the pass downloading many variants.

For each model, record:

- resolved asset path and successful load;
- `SkelRoot` and Animation Graph/retargeting compatibility;
- whether the arm moves without translating or sinking the torso;
- hand realism, size, YOLO confidence/bbox, and missed detections;
- ROI entry without restoring the RG2 false positive; and
- visible hand/table intersection or clearance.

Do not reuse `F_Business_02`'s hardcoded hand offset for other characters.
Choose a new default only after comparing results, and retain the
`HAND_TEST_PERSON_USD` override.

## Task 3: table clearance, only if needed

Inspect the actual animation/IK target and a close side view before editing the
height. If the hand already clears the tabletop, or the code already targets
approximately `TABLE_TOP_Z + 0.10`, do not add another offset.

If the hand intersects or nearly intersects the table, set the final hand/IK
interaction target to exactly:

```text
TABLE_TOP_Z + 0.10 = 0.83 m
```

Apply the correction to the hand/IK target, not the table, chair, or entire
character root. Prefer one clearly named, environment-overridable offset such
as `HAND_TEST_TARGET_Z_OFFSET`; never stack multiple `+0.10` corrections.
Record before/after evidence and the actual world-space target Z.

## Task 4: isolate the RTX 5080 5 FPS bottleneck

Use the same scene and person count for every case. Change one variable at a
time, warm up the pipeline, and measure for at least 30 seconds.

### A. Confirm actual runtime parameters

A plain `ros2 run hand_safety hand_detector_node` uses Python defaults where
debug image publication and the OpenCV window are `true`. The YAML sets them
to `false` only when it is actually loaded. Record:

```bash
ros2 param get /hand_detector_node image_size
ros2 param get /hand_detector_node process_rate
ros2 param get /hand_detector_node publish_annotated_image
ros2 param get /hand_detector_node show_window
ros2 param get /hand_detector_node roi_inference
ros2 param get /hand_detector_node tiled_inference
```

### B. Separate camera, detector, and GPU measurements

Run concurrently after `table_arrived` is true:

```bash
ros2 topic hz /serving_robot/table_camera/color/image_raw
ros2 topic hz /hand_detection/detections
nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total --format=csv -l 1
```

- RGB near 5 Hz means the upstream Isaac/render/camera publisher is limiting.
- RGB substantially faster than detections means conversion/YOLO/downstream
  work is limiting.
- If needed, add throttled timings around `CvBridge.imgmsg_to_cv2`,
  `model.predict`, debug drawing/publication, and the complete
  `process_latest_frame`. Report milliseconds, not only aggregate FPS.

### C. One-variable comparisons

1. Disable actual depth generation/publication by removing or feature-gating
   `DepthPublish` and its graph connections. Merely not subscribing is not
   sufficient. Confirm the depth topic stops, keep RGB at 1280x960, and
   measure. Prefer a toggle if another workflow needs depth.
2. Disable the extra 1280x960 table-camera preview viewport while keeping the
   ROS RGB RenderProduct, then measure separately from the depth-off case.
3. Verify installed Isaac 5.1 `frameSkipCount` semantics. The helpers currently
   use `1`, which may publish every other render frame. Compare `1` with `0`
   and record simulation/render FPS plus RGB topic FPS. Do not label deliberate
   publisher skipping as YOLO frame drop.
4. Keep camera input at 1280x960 and compare YOLO `image_size` 1280, 768, and
   640 with `roi_inference=true`, `tiled_inference=false`, debug publication
   off, and the OpenCV window off. Record speed and detection quality. Do not
   adopt a faster setting if small/distant-hand detection becomes unreliable.

Use this result table in `GPU_RUN_LOG.txt`:

```text
case | RGB Hz | detection Hz | sim/render FPS | predict ms | GPU % | VRAM | detection quality
```

State whether the limit is camera/render publication, depth/viewport overhead,
frame skipping, ROS/CvBridge CPU work, YOLO inference, or a combination. Keep
source resolution and YOLO inference size as independent controls.

## Build and run

From the repository root:

```bash
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=101
MOBILE_DEMO_HAND_TEST=1 python isaacpjt/mobile_manipulator_demo.py
```

In another terminal, use the same ROS domain and run hand safety with explicit
parameters. Enable annotated output only for visual validation; disable it for
performance measurements.

Useful checks:

```bash
ros2 topic echo /serving_robot/table_arrived
ros2 topic echo /hand_safety/roi_intrusion
ros2 topic hz /serving_robot/table_camera/color/image_raw
ros2 topic hz /hand_detection/detections
ros2 run rqt_image_view rqt_image_view /hand_detection/image
```

## Acceptance criteria

- At least three People models are rendered and compared.
- At least one model performs a visibly arm-driven reach without whole-torso
  sinking/sliding as the primary motion.
- The chosen hand clears the tabletop without an excessive gap; if corrected,
  its target is 0.83 m and no duplicate 10 cm offset exists.
- At rest for at least 20 seconds, `/hand_safety/roi_intrusion` remains false
  and the RG2 false positive remains excluded.
- Across multiple reach cycles, intrusion becomes true in sync with the
  rendered hand entering the ROI and returns false after retraction.
- Camera input remains 1280x960 and `table_arrived` gating still works.
- The performance table explains the approximately 5 FPS result with measured
  before/after data for depth, preview, frame skipping, and YOLO input size.

## Deliverables

Append one dated Pass 4 entry to `GPU_RUN_LOG.txt` containing:

- commands and exact runtime parameters;
- tested asset paths, graph/clip mechanism, and extension versions;
- successes and failed attempts;
- clearance decision and actual target Z;
- detection confidence/bboxes and intrusion timing; and
- the complete performance table and final bottleneck conclusion.

Keep implementation changes focused and retain useful environment/config
overrides.
