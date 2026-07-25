#!/usr/bin/env bash
source /opt/ros/humble/setup.bash 2>/dev/null || true

for pattern in \
  two_wheel_topic_bridge \
  navigation_launch \
  localization_launch \
  controller_server \
  planner_server \
  bt_navigator \
  behavior_server \
  lifecycle_manager \
  map_server \
  amcl \
  velocity_smoother \
  topic_bridge; do
  pkill -f "$pattern" 2>/dev/null || true
done
sleep 1
echo "[kill_nav2] remaining nav-related nodes:"
ros2 node list 2>/dev/null | grep -E 'nav|amcl|map_server|lifecycle|controller|planner|bt_navigator' || echo "(none)"
