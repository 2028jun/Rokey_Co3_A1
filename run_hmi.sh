#!/bin/bash
# Pizza Serving Robot HMI Launcher Wrapper
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
bash "$SCRIPT_DIR/serving_robot/run_hmi.sh" "$@"
