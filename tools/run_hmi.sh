#!/bin/bash
# Pizza Serving Robot HMI Dashboard Runner
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$( cd "$SCRIPT_DIR/.." && pwd )"
source /opt/ros/humble/setup.bash || true
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
else
    echo "ERROR: $WORKSPACE_DIR/install/setup.bash not found. Build the ROS workspace first." >&2
    exit 1
fi

export HMI_PORT=${HMI_PORT:-8000}

echo "=========================================================="
echo " Starting Pizza Serving Robot HMI Dashboard"
echo " ROS_DOMAIN_ID : ${ROS_DOMAIN_ID:-0}"
echo " HMI Web Access: http://localhost:$HMI_PORT"
echo "=========================================================="

export PYTHONPATH=$WORKSPACE_DIR/src/serving_hmi:$PYTHONPATH
python3 -m serving_hmi.hmi_backend_node
