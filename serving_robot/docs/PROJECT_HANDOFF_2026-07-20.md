# Pizza Serving Mobile Manipulator — AI Handoff

## 1. Current target and environment

- Workspace: `/home/rokey/cobot3_ws`
- Git remote: `https://github.com/2028jun/Rokey_Co3_A1.git`
- Active branch at handoff: `woduq`
- Isaac Sim: `5.1.0-rc.19` (metadata `5.1.0`)
- ROS: ROS 2 Humble, Ubuntu Jammy
- DDS domain: machine-local `ROS_DOMAIN_ID` from `~/.bashrc`
- Robot: Ridgeback R100 + two food shelves + Doosan M0609 + fixed RG2 model
- Restaurant: lightweight hall joined to the Lightwheel kitchen

The service scenario is kitchen pickup, mobile transport, table docking, hand-aware
placement, and return. MoveIt 2 integration exists, but table-side reactive avoidance
is expected to use RMPFlow.

## 2. Main files

- `isaacpjt/mobile_manipulator_demo.py`: current integrated Isaac application
- `isaacpjt/lightweight_restaurant_demo.py`: builds the lightweight restaurant stage
- `assets/lightweight_restaurant/lightweight_pizza_restaurant.usda`: current world
- `src/ridgeback_m0609_description`: combined Ridgeback/shelf/M0609 description
- `src/m0609_isaac_control`: Isaac trajectory bridge and MoveIt test clients
- `src/m0609_rg2_moveit`: MoveIt configuration
- `isaacpjt/M0609/rmpflow_obstacle_demo.py`: MoveIt/RMPFlow comparison demo
- `isaacpjt/capture_table_camera.py`: saves one ROS RGB camera frame
- `docs/ISAAC_MOVEIT_QUICKSTART.md`: MoveIt startup
- `docs/MOBILE_MANIPULATOR_QUICKSTART.md`: mobile robot startup

## 3. Current physical layout

All positions below use the Ridgeback base frame unless explicitly marked as world.

- Ridgeback convention: `+X` forward, `-X` rear
- Table docking face: robot `-X`, the same side as the arm
- M0609 base: `x=-0.22 m`, base height approximately `0.895 m`
- Shelf levels: `z=0.44 m`, `0.64 m`; top plate `z=0.84 m`
- Customer table: `1.80 x 0.94 x 0.73 m`
- Demonstration table center: world `(-3.20, -2.20)`
- Robot spawn: world `(-1.82, -2.20, 0.002)`
- Robot/table clearance at the short edge: approximately `0.08 m`

The old plan to dock on the arm-opposite `+X` face was rejected. It required roughly
`0.92–1.07 m` of reach and exceeded the useful M0609 range. Docking on `-X` puts a
normal placement point about `0.5–0.65 m` from the arm base.

## 4. Cameras

### Wrist camera

- D455 visual and four source camera prims are mounted below the RG2 bracket.
- It is intended for close manipulation and the remaining fixed-camera blind spot.
- ROS publication for the wrist camera is not connected yet.

### Fixed table camera

- Original Intel RealSense D455 USD hierarchy
- Optical height: `1.85 m`
- Mounted on the `+X`, right side through a short lateral boom
- Camera position: `(0.25, -0.50, 1.85)`
- Look target: `(-1.00, -0.15, 0.74)`
- Render resolution: `640 x 480`
- The fixed view sees most of the tabletop and customer approach area. The M0609
  still masks a small right-edge region; use the wrist camera as the complementary view.

ROS topics must be verified on the machine-local domain:

```text
/serving_robot/table_camera/color/image_raw
/serving_robot/table_camera/depth/image_raw
/serving_robot/table_camera/camera_info
```

The camera frame ID is `table_camera_optical_frame`. A real `rgb8` frame was received
and saved successfully at 640x480.

## 5. Run and verify

```bash
cd /home/rokey/cobot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
  isaacpjt/mobile_manipulator_demo.py --/log/level=error
```

The script automatically removes ROS Python 3.10 paths and re-executes with Isaac's
bundled Humble libraries. This is required because Isaac Sim 5.1 uses Python 3.11.
Do not remove that pre-launch environment block.

Verify from another terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/rokey/cobot3_ws/install/setup.bash
ros2 topic list | grep /serving_robot/table_camera
python3 /home/rokey/cobot3_ws/isaacpjt/capture_table_camera.py
```

The GUI opens an additional `Table Camera` viewport.

## 6. Stability and parking brake

The original robot visibly trembled because of four cylindrical mecanum-wheel contacts,
an initial drive-target mismatch, and excessive arm drive stiffness.

Implemented fixes:

- Arm start pose and drive targets are set together.
- Arm drive stiffness/damping: `200000 / 20000`
- CPU PhysX at 120 Hz; solver iterations `64 / 16`
- Base linear/angular damping and limited depenetration velocity
- Self-collision disabled for the combined fixed shelf/arm structure
- RG2 fixed parts have explicit mass/inertia
- Default stationary `ParkingBrake` fixed joint

Verified stationary values:

```text
max_arm_speed=0.000000 rad/s
max_base_speed=0.000000 m/s
```

For navigation tests, launch with:

```bash
MOBILE_DEMO_PARKED_HOLD=0 .../python.sh isaacpjt/mobile_manipulator_demo.py
```

The final state machine should toggle the parking brake: release while navigating,
engage after docking, then manipulate.

## 7. MoveIt and RMPFlow status

- MoveIt 2 can plan and execute M0609 joint trajectories through the Isaac bridge.
- Planning scene obstacle demos exist in `m0609_isaac_control`.
- RG2 is currently fixed geometry; it has no open/close DOFs.
- RMPFlow is preferred for table-side hand avoidance because it reacts continuously
  to moving obstacles. MoveIt remains useful for global, static collision-free paths.
- RMPFlow collision spheres/config were tuned during the comparison demo.

See `docs/ISAAC_MOVEIT_QUICKSTART.md` for the three-terminal MoveIt workflow.

## 8. Known warnings and limitations

1. The collected D455 asset contains tensor/rigid-body configuration. Since the fixed
   camera is attached to the Ridgeback rigid body, its own rigid-body API is removed.
   Isaac may print `Pattern .../RSD455 did not match any rigid bodies`; publication works.
   A future cleanup should reference only Looks/Visual/Cameras while preserving material
   relationship paths, or remove the collected tensor graph.
2. The Lightwheel kitchen contains legacy dynamic triangle meshes. PhysX falls back to
   convex hulls and prints warnings. CPU PhysX was selected after GPU PhysX produced a
   CUDA error 700 in this mixed scene.
3. CameraInfo currently falls back to `plumb_bob` with zero coefficients because the
   collected camera has no supported physical distortion model.
4. Absolute paths assume `/home/rokey/cobot3_ws`. The Lightwheel source asset must exist
   under `/home/rokey/.gemini/antigravity/scratch/assets/Lightwheel_Kitchen/...` or the
   stage reference must be repathed.
5. Large source packs are intentionally not committed: `assets/nvidia_restaurant`,
   `assets/kenney_furniture`, and unrelated collected block textures.

## 9. Next implementation tasks

1. Publish the wrist D455 RGB/depth/camera_info topics.
2. Add a fixed TF publisher for `base_link -> table_camera_optical_frame` and verify the
   optical convention against ROS REP-103.
3. Detect a hand in RGB, sample registered depth, deproject `(u,v,d)` to camera XYZ,
   and transform it to `base_link`/world using tf2.
4. Feed the moving hand position to RMPFlow as an updated sphere obstacle.
5. Implement RG2 actuation and pizza/plate attach-detach behavior.
6. Add Nav2 holonomic docking and automatic parking-brake switching.
7. Integrate kitchen pickup, transport, docking, hand-aware placement, and return into
   the central state machine.

## 10. Asset portability

The Git commit should include the lightweight restaurant, generated mobile robot USD,
and the selected M0609/RG2/D455 runtime subset. If a clone reports a missing asset,
compare the USD reference with this machine's
`isaacpjt/M0609/Collected_m0609_camera2` and copy only the referenced subtree. Do not
commit the 8.4 GB NVIDIA restaurant download.
