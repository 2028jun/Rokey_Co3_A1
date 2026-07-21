# Project environment rules

- Target NVIDIA Isaac Sim version: `5.1.0-rc.19` (application metadata version `5.1.0`).
- Target ROS version: ROS 2 Humble on Ubuntu Jammy.
- Use APIs and import paths compatible with Isaac Sim 5.1 and ROS 2 Humble.
- When an Isaac Sim API signature is uncertain, check the locally installed Isaac Sim 5.1 source under `/home/rokey/dev_ws/isaac_sim/isaacsim` before editing code.
- ROS communication for this workspace uses `ROS_DOMAIN_ID=103` unless the user explicitly requests a different domain.
- Do not modify sibling workspaces (`serving_robot/`, `Rokey_Co3_A1-woduq/`, `hand_safety/`, `재현/`). Copy assets into `nav_robot/` when needed.
- TF rule for Nav2: publish `map → odom → ridgeback_base_link` only. Never publish a static `world → base_link` (or equivalent) that breaks the Nav2 tree.
