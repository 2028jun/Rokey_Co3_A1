#!/usr/bin/env bash
# Fetch nav_robot runtime assets from git (map) and tracked USD pack (woduq).
set -euo pipefail

NAV_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
REPO_ROOT="$(cd "$NAV_ROOT/.." && pwd)"
MAP_DIR="$NAV_ROOT/maps/restaurant"
USD_DST="$NAV_ROOT/assets/mobile_manipulator"

echo "[fetch] repo: $REPO_ROOT"
echo "[fetch] nav_robot: $NAV_ROOT"

cd "$REPO_ROOT"
if git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "[fetch] git fetch origin..."
  git fetch origin --quiet || true
  for ref in ("origin/younggi", "origin/woduq", "origin/main", "HEAD"):
    if git cat-file -e "$ref:nav_robot/maps/restaurant/map.pgm" 2>/dev/null; then
      echo "[fetch] map from $ref"
      git checkout "$ref" -- nav_robot/maps/restaurant/map.pgm nav_robot/maps/restaurant/map.yaml
      break
    fi
  done
else
  echo "[warn] not a git repo — skipping map checkout"
fi

mkdir -p "$MAP_DIR" "$USD_DST"

USD_SRC=""
for candidate in \
  "$REPO_ROOT/Rokey_Co3_A1-woduq/assets/mobile_manipulator" \
  "$REPO_ROOT/serving_robot/assets/mobile_manipulator"; do
  if [[ -f "$candidate/ridgeback_m0609_v2.usd" ]]; then
    USD_SRC="$candidate"
    break
  fi
done

if [[ -z "$USD_SRC" ]]; then
  echo "[error] ridgeback USD not found in repo (woduq/serving_robot)" >&2
  exit 1
fi

echo "[fetch] USD copy: $USD_SRC -> $USD_DST"
rm -f "$USD_DST/mobile_manipulator"
rsync -a --delete \
  --exclude 'mobile_manipulator' \
  "$USD_SRC/" "$USD_DST/"

KITCHEN_DST="$NAV_ROOT/assets/Lightwheel_Kitchen/Collected_KitchenRoom"
if [[ ! -e "$KITCHEN_DST" ]]; then
  for candidate in \
    "$REPO_ROOT/serving_robot/assets/Lightwheel_Kitchen/Collected_KitchenRoom" \
    "$REPO_ROOT/nav_robot/assets/Lightwheel_Kitchen/Collected_KitchenRoom"; do
    if [[ -d "$candidate" ]]; then
      echo "[fetch] kitchen symlink -> $candidate"
      ln -sfn "$(realpath "$candidate")" "$KITCHEN_DST"
      break
    fi
  done
fi

echo "[fetch] done"
echo "  map:  $MAP_DIR/map.pgm ($(wc -c < "$MAP_DIR/map.pgm") bytes)"
echo "  usd:  $USD_DST/ridgeback_m0609_v2.usd"
