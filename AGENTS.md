# Project environment rules

- Target NVIDIA Isaac Sim version: `5.1.0-rc.19` (application metadata version `5.1.0`).
- Target ROS version: ROS 2 Humble on Ubuntu Jammy (`ros-humble-ros-base` `0.10.0`, `rclpy` `3.3.21`).
- Use APIs and import paths compatible with Isaac Sim 5.1 and ROS 2 Humble. Do not assume signatures or APIs from another Isaac Sim or ROS distribution.
- When an Isaac Sim API signature is uncertain, check the locally installed Isaac Sim 5.1 source or bindings under `$ISAAC_SIM_ROOT` (see README "의존성 설치" for how to set it) before editing code.
- TF rule for Nav2: publish `map → odom → ridgeback_base_link` only. Never publish a static `world → base_link` (or equivalent) that breaks the Nav2 tree.
- ROS domain IDs are machine-local settings. Inherit `ROS_DOMAIN_ID` from the shell and never hardcode a shared domain in repository code.
- **Terminal Execution Rules (중요: 두 명령을 동일한 터미널에서 함께 실행금지)**:
  - **아이작심용 터미널**: `export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/exts/isaacsim.ros2.bridge/humble/lib"`
  - **ROS 통신용 터미널**: `source /opt/ros/humble/setup.bash`
- **ROS build rule**: Do not invoke plain `colcon build` in this workspace. In
  the restricted execution environment, Python's default
  `ThreadedChildWatcher` can delay completed colcon child processes by about
  60 seconds each. Use the workspace wrapper, which selects
  `PidfdChildWatcher` and falls back to `SafeChildWatcher`, and disable user
  site packages to avoid the setuptools `--uninstall` error:
  ```bash
  source /opt/ros/humble/setup.bash
  PYTHONNOUSERSITE=1 ./tools/colcon_safe.py build \
    --packages-up-to <packages...> \
    --executor sequential \
    --event-handlers console_start_end+ desktop_notification- status- terminal_title-
  ```
