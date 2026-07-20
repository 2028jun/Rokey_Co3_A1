#!/bin/bash
# Pizza Serving Robot HMI Dashboard Runner
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source /opt/ros/humble/setup.bash
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
elif [ -f "/home/rokey/cobot3_ws/install/setup.bash" ]; then
    source "/home/rokey/cobot3_ws/install/setup.bash"
fi

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-101}

echo "=========================================================="
echo " Starting Pizza Serving Robot HMI Dashboard"
echo " ROS_DOMAIN_ID : $ROS_DOMAIN_ID"
echo " HMI Web Access: http://localhost:$HMI_PORT"
echo "=========================================================="

python3 -m serving_hmi.hmi_backend_node
