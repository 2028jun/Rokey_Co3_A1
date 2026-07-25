# Project environment rules

- Target NVIDIA Isaac Sim version: `5.1.0-rc.19` (application metadata version `5.1.0`).
- Target ROS version: ROS 2 Humble on Ubuntu Jammy (`ros-humble-ros-base` `0.10.0`, `rclpy` `3.3.21`).
- Use APIs and import paths compatible with Isaac Sim 5.1 and ROS 2 Humble. Do not assume signatures or APIs from another Isaac Sim or ROS distribution.
- When an Isaac Sim API signature is uncertain, check the locally installed Isaac Sim 5.1 source or bindings under `/home/rokey/dev_ws/isaac_sim/isaacsim` before editing code.
- ROS domain IDs are machine-local settings. Inherit `ROS_DOMAIN_ID` from the shell and never hardcode a shared domain in repository code.
