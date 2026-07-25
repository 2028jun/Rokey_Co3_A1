# Rokey_Co3_A1 - Serving Robot & Hand Safety Workspace

## Workspace Structure

```text
Rokey_Co3_A1/
├── nav_robot/          # Isaac Sim standalone bridge (nav_restaurant_demo.py) — /scan, /odom, TF,
│                       #   direct_nav mission driving (park-out, kitchen route, table docking),
│                       #   CrossingPedestrian / TypingCustomer test actors
├── nav_robot5/         # two_wheel_rails ROS 2 package — Nav2 stack, nav2_collision_monitor,
│                       #   topic_bridge, routes.yaml (map-frame dock/spine coordinates), maps/
├── serving_robot/      # HMI web dashboard sources (src/serving_hmi) & run_hmi.sh
├── src/                # serving_robot_manager, serving_robot_interfaces, map_gen, hand_safety (ignored copy)
├── hand_safety/        # Canonical Hand Detection & Safety ROS 2 Package
├── .gitignore
└── README.md
```

`src/hand_safety` is a legacy duplicate and contains `COLCON_IGNORE`.
Build the canonical root-level `hand_safety` package only.

Isaac Sim's install path is configurable via `ISAAC_SIM_ROOT` (defaults to
`~/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release`); set it if your
Isaac Sim lives elsewhere.

Set a unique ROS domain for each machine in `~/.bashrc`. Repository scripts
inherit these values and do not choose a shared domain number:

```bash
export ROS_DOMAIN_ID=<machine-specific-domain>
export NAV_ROBOT_ROS_DOMAIN_ID="$ROS_DOMAIN_ID"
export NAV_ROBOT5_ROS_DOMAIN_ID="$ROS_DOMAIN_ID"
```

Open a new terminal after changing `~/.bashrc`.

## Multi-robot Quick Start (3 terminals)

The integrated multi-robot mode spawns two complete serving robots in Isaac
Sim, runs an independent namespaced Nav2/safety/manager stack for each robot,
and shows both robots in one RViz window. Keep the Isaac and ROS commands in
separate terminals; do not source the ROS setup in the Isaac terminal.

Build once before the first run or after changing ROS packages:

```bash
cd /home/rokey/cobot3_ws
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 ./tools/colcon_safe.py build \
  --packages-up-to serving_robot_manager serving_hmi two_wheel_rails hand_safety \
  --executor sequential \
  --event-handlers console_start_end+ desktop_notification- status- terminal_title-
```

**Terminal 1 — Isaac Sim with two integrated robots**

```bash
cd /home/rokey/cobot3_ws
./tools/run_multi_integrated_isaac.sh
```

This script sets `NAV_MULTI_ROBOT=1`, configures the Isaac ROS 2 bridge
library path, and starts `nav_robot/isaacpjt/nav_restaurant_demo.py`. Wait
until the restaurant and both `/World/NavRobot1` and `/World/NavRobot2`
instances have finished loading.

**Terminal 2 — two Nav2 stacks, fleet manager, safety nodes, and RViz**

```bash
cd /home/rokey/cobot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch serving_robot_manager multi_robot.launch.py
```

The launch starts `/robot1` and `/robot2` workers and opens one RViz window
with both robot models, LiDAR scans, local/global costmaps, footprints, and
stop/slowdown zones.

**Terminal 3 — HMI**

```bash
cd /home/rokey/cobot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch serving_hmi hmi.launch.py
```

Open `http://localhost:8000`. HMI orders are sent to `/manager/order`; the
fleet manager assigns an idle robot. Payload prims are currently shared, so
complete serving orders are serialized even though both navigation and
safety stacks run concurrently.

For a navigation-only smoke test without the HMI:

```bash
ros2 service call /robot1/navigation/command \
  serving_robot_interfaces/srv/TaskCommand "{command: 0}"
ros2 service call /robot2/navigation/command \
  serving_robot_interfaces/srv/TaskCommand "{command: 1}"
```

Commands `0` through `3` select a table; command `4` returns that robot to
the kitchen.

## Quick Start (3-terminal integration test)

**Terminal 1 — Isaac Sim**
```bash
cd /home/rokey/cobot3_ws/nav_robot
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/exts/isaacsim.ros2.bridge/humble/lib"
/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
  isaacpjt/nav_restaurant_demo.py
```
Wait for `[typing_topic] waiting on /hand_test/type_keyboard ...` in the log
before using the HMI test buttons below.

**Terminal 2 — Manager + Nav2 + collision_monitor + hand_safety**
```bash
cd /home/rokey/cobot3_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  serving_robot_interfaces serving_robot_manager hand_safety serving_hmi two_wheel_rails
source install/setup.bash
ros2 launch serving_robot_manager vision_manager_nav2.launch.py \
  vision_debug:=true use_sim_time:=true autostart:=true rviz:=true
```
This single launch starts `direct_nav_server_node`, `manager_node`,
`hand_detector_node`, the Nav2 stack (AMCL/controller/planner/bt_navigator),
and `nav2_collision_monitor` (stop/slowdown safety zones, gates `cmd_vel` →
`cmd_vel_safe`, which is what Isaac's NavBridge actually drives on).

**Terminal 3 — HMI Web Dashboard**
```bash
cd /home/rokey/cobot3_ws
./run_hmi.sh
```
Open browser: `http://localhost:8000`

## HMI test controls

- **사람 스폰/제거** buttons drive `CrossingPedestrian`, a walking test
  actor with a LiDAR-visible collider (contact-filtered so it doesn't push
  the robot).
- **타이핑 시작** button triggers `TypingCustomer`'s one-shot typing
  animation via `/hand_test/type_keyboard`, exercised by `hand_safety`'s
  ROI-intrusion detector.
- The robot map panel draws the live stop/slowdown safety zones (mirrors
  `nav2_collision_monitor`'s polygons in `nav2_params.yaml` and
  `nav_restaurant_demo.py`'s `OBSTACLE_STOP_*`/`OBSTACLE_SLOWDOWN_*` — keep
  all three in sync when retuning).

## SLAM map generation

`src/map_gen` is a one-shot `slam_toolbox` mapping mode that reuses Isaac's
existing `/scan`/`/odom`/TF instead of a separate bridge. Run it standalone
(never alongside the launch above — both drive `cmd_vel`). See
[src/map_gen/README.md](src/map_gen/README.md) for the full workflow and
how to swap a generated map into `nav_robot5/src/two_wheel_rails/maps/`.
