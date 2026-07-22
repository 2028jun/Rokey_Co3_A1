#!/usr/bin/env bash
# Copy restaurant stage assets into nav_robot5 (no maps, launch, or nav_robot src).
set -euo pipefail

WS="$(cd "$(dirname "$0")/.." && pwd)"
REPO_ROOT="$(cd "$WS/.." && pwd)"

REST_DST="$WS/assets/lightweight_restaurant"
KITCHEN_DST="$WS/assets/Lightwheel_Kitchen/Collected_KitchenRoom"

mkdir -p "$REST_DST" "$(dirname "$KITCHEN_DST")"

REST_SRC=""
for candidate in \
  "$REPO_ROOT/nav_robot/assets/lightweight_restaurant" \
  "$REPO_ROOT/serving_robot/assets/lightweight_restaurant"; do
  if [[ -f "$candidate/lightweight_pizza_restaurant.usda" ]]; then
    REST_SRC="$candidate"
    break
  fi
done

if [[ -z "$REST_SRC" ]]; then
  echo "[error] lightweight_pizza_restaurant.usda not found under nav_robot or serving_robot" >&2
  exit 1
fi

echo "[sync] restaurant: $REST_SRC -> $REST_DST"
rsync -a "$REST_SRC/" "$REST_DST/"

if [[ -f "$KITCHEN_DST/KitchenRoom.usd" ]]; then
  echo "[sync] kitchen already present: $KITCHEN_DST"
else
  KITCHEN_SRC=""
  for candidate in \
    "$REPO_ROOT/assets/Lightwheel_Kitchen/Collected_KitchenRoom" \
    "$REPO_ROOT/nav_robot/assets/Lightwheel_Kitchen/Collected_KitchenRoom" \
    "$REPO_ROOT/serving_robot/assets/Lightwheel_Kitchen/Collected_KitchenRoom"; do
    if [[ -d "$candidate" ]]; then
      KITCHEN_SRC="$candidate"
      break
    fi
  done
  if [[ -z "$KITCHEN_SRC" ]]; then
    echo "[warn] Lightwheel Collected_KitchenRoom not found — copy manually for full kitchen" >&2
  else
    echo "[sync] kitchen: $KITCHEN_SRC -> $KITCHEN_DST"
    rsync -a "$KITCHEN_SRC/" "$KITCHEN_DST/"
  fi
fi

# Optional README from nav_robot (metadata only)
for readme in \
  "$REPO_ROOT/nav_robot/assets/Lightwheel_Kitchen/README.md" \
  "$REPO_ROOT/serving_robot/assets/Lightwheel_Kitchen/README.md"; do
  if [[ -f "$readme" ]]; then
    cp "$readme" "$WS/assets/Lightwheel_Kitchen/README.md"
    break
  fi
done

echo "[sync] done"
echo "  stage: $REST_DST/lightweight_pizza_restaurant.usda"
if [[ -d "$KITCHEN_DST" ]]; then
  echo "  kitchen: $KITCHEN_DST"
fi
