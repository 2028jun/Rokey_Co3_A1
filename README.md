# Rokey_Co3_A1 - Serving Robot & Hand Safety Workspace

## Workspace Structure

```text
Rokey_Co3_A1/
├── serving_robot/     # Serving Robot ROS 2, Isaac Sim & HMI Web Dashboard
│   ├── isaacpjt/     # Isaac Sim 5.1 Standalone Python Scripts
│   ├── src/          # ROS 2 Workspace Packages (serving_hmi, etc.)
│   └── run_hmi.sh    # HMI Web Dashboard Runner
├── hand_safety/       # Canonical Hand Detection & Safety ROS 2 Package
├── .gitignore
└── README.md
```

`src/hand_safety` is a legacy duplicate and contains `COLCON_IGNORE`.
Build the canonical root-level `hand_safety` package only.

## Vision-integrated ROS startup

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=102
colcon build --symlink-install --packages-select \
  serving_robot_interfaces serving_robot_manager hand_safety
source install/setup.bash
ros2 launch serving_robot_manager vision_manager.launch.py
```

The Isaac Sim process publishes `/camera/color/image_raw`. The detector
publishes `/hand_safety/intrusion`, and Manager gates inference with the
latched `/serving_robot/table_arrived` state.
The same launch starts `direct_nav_server_node`, which owns the ROS Humble
custom services and forwards them to Isaac's built-in axis-route controller.

## Quick Start (HMI Web Dashboard)

```bash
# Run HMI Web Dashboard
./run_hmi.sh
# Open Browser: http://localhost:8000
```
