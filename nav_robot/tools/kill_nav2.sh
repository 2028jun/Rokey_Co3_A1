#!/usr/bin/env bash
# Stop duplicate Nav2 / SLAM nodes on ROS_DOMAIN_ID (default 103).
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-103}"
source /opt/ros/humble/setup.bash 2>/dev/null || true

for pattern in \
  nav2_restaurant \
  nav2_nova_carter \
  navigation_launch \
  localization.launch \
  slam_mapping \
  async_slam_toolbox \
  map_to_odom_identity \
  controller_server \
  planner_server \
  bt_navigator \
  behavior_server \
  lifecycle_manager \
  map_server \
  amcl \
  velocity_smoother \
  waypoint_follower \
  nova_carter_topic_bridge; do
  pkill -f "$pattern" 2>/dev/null || true
done
sleep 1
echo "[kill_nav2] remaining nav nodes:"
ros2 node list 2>/dev/null | grep -E 'nav|amcl|map_server|slam|lifecycle|controller|planner|bt_navigator' || echo "(none)"
