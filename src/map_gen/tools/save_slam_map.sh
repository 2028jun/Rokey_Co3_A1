#!/usr/bin/env bash
# Save /map from slam_toolbox to nav_robot5/src/two_wheel_rails/maps/restaurant/.
# Never overwrites an existing slam_map.* -- uses slam_map_YYYYMMDD_HHMMSS if taken.
# Does NOT touch map.yaml/map.pgm, the map nav2.launch.py actually loads by
# default -- that swap is a deliberate, separate, manual step (see README.md).
set -eo pipefail

WS_ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
OUT_DIR="$WS_ROOT/nav_robot5/src/two_wheel_rails/maps/restaurant"
BASE_NAME="${1:-slam_map}"

mkdir -p "$OUT_DIR"

source /opt/ros/humble/setup.bash
if [[ -f "$WS_ROOT/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "$WS_ROOT/install/setup.bash"
fi

echo "[save] waiting for /map (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)..."
for _ in $(seq 1 30); do
  if ros2 topic list 2>/dev/null | grep -q '^/map$'; then
    break
  fi
  sleep 1
done

if ! ros2 topic list 2>/dev/null | grep -q '^/map$'; then
  echo "[save] /map not found -- keep slam_mapping.launch.py running" >&2
  exit 1
fi

OUT_STEM="$BASE_NAME"
if [[ -e "$OUT_DIR/${OUT_STEM}.pgm" || -e "$OUT_DIR/${OUT_STEM}.yaml" ]]; then
  OUT_STEM="${BASE_NAME}_$(date +%Y%m%d_%H%M%S)"
  echo "[save] existing ${BASE_NAME}.* kept -- writing new file as ${OUT_STEM}"
fi

OUT_BASE="$OUT_DIR/$OUT_STEM"
echo "[save] writing $OUT_BASE"
ros2 run nav2_map_server map_saver_cli -f "$OUT_BASE"

echo "[save] done (map.yaml/map.pgm, the map nav2.launch.py currently loads, untouched):"
ls -la "$OUT_BASE.pgm" "$OUT_BASE.yaml"
cat "$OUT_BASE.yaml"
