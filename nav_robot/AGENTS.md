# Project environment rules

- Target NVIDIA Isaac Sim version: `5.1.0-rc.19` (application metadata version `5.1.0`).
- Target ROS version: ROS 2 Humble on Ubuntu Jammy.
- Use APIs and import paths compatible with Isaac Sim 5.1 and ROS 2 Humble.
- When an Isaac Sim API signature is uncertain, check the locally installed Isaac Sim 5.1 source under `/home/rokey/dev_ws/isaac_sim/isaacsim` before editing code.
- ROS domain IDs are machine-local settings. Inherit `ROS_DOMAIN_ID` from the shell and never hardcode a shared domain in repository code.
- **Terminal Execution Rules (중요: 두 명령을 동일한 터미널에서 함께 실행금지)**:
  - **아이작심용 터미널**: `export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/exts/isaacsim.ros2.bridge/humble/lib"`
  - **ROS 통신용 터미널**: `source /opt/ros/humble/setup.bash`
- Do not modify sibling workspaces (`serving_robot/`, `Rokey_Co3_A1-woduq/`, `hand_safety/`, `재현/`). Copy assets into `nav_robot/` when needed.
- TF rule for Nav2: publish `map → odom → ridgeback_base_link` only. Never publish a static `world → base_link` (or equivalent) that breaks the Nav2 tree.
