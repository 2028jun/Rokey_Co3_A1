#!/usr/bin/env bash
# Symlink restaurant / robot / Lightwheel assets (avoid large duplicates).
# Default: sibling nav_robot/assets in Rokey_Co3_multi (override with MAP_GEN_ASSET_SRC).
set -eo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
if [[ -n "${MAP_GEN_ASSET_SRC:-}" ]]; then
  SRC="$MAP_GEN_ASSET_SRC"
elif [[ -d "$ROOT/../nav_robot/assets" ]]; then
  SRC="$(cd "$ROOT/../nav_robot/assets" && pwd)"
elif [[ -d "$ROOT/../nav_robot6/assets" ]]; then
  SRC="$(cd "$ROOT/../nav_robot6/assets" && pwd)"
else
  echo "[sync] no asset source: set MAP_GEN_ASSET_SRC or place nav_robot/assets" >&2
  exit 1
fi
DEST="$ROOT/assets"

mkdir -p "$DEST"

link_rel() {
  local link="$1"
  local target="$2"
  if [[ ! -e "$target" ]]; then
    echo "[sync] missing source: $target" >&2
    return 1
  fi
  local rel
  rel="$(realpath --relative-to="$(dirname "$link")" "$target")"
  ln -sfn "$rel" "$link"
  echo "[sync] $link -> $rel"
}

link_rel "$DEST/lightweight_restaurant" "$SRC/lightweight_restaurant"
link_rel "$DEST/diagnostics" "$SRC/diagnostics"

# Prefer the workspace assets/Lightwheel_Kitchen symlink when present.
if [[ -e "$ROOT/../assets/Lightwheel_Kitchen" ]] \
  && [[ -f "$ROOT/../assets/Lightwheel_Kitchen/Collected_KitchenRoom/KitchenRoom.usd" ]]; then
  ln -sfn ../../assets/Lightwheel_Kitchen "$DEST/Lightwheel_Kitchen"
  echo "[sync] $DEST/Lightwheel_Kitchen -> ../../assets/Lightwheel_Kitchen"
else
  linked_lw=0
  for alt in \
    "$ROOT/../serving_robot/assets/Lightwheel_Kitchen" \
    "$SRC/Lightwheel_Kitchen" \
    "$ROOT/../nav_robot5/assets/Lightwheel_Kitchen"; do
    if [[ -e "$alt" ]] \
      && [[ -f "$alt/Collected_KitchenRoom/KitchenRoom.usd" ]]; then
      link_rel "$DEST/Lightwheel_Kitchen" "$alt"
      linked_lw=1
      break
    fi
  done
  if [[ "$linked_lw" -ne 1 ]]; then
    echo "[sync] Lightwheel_Kitchen with KitchenRoom.usd not found" >&2
    exit 1
  fi
fi

kitchen="$DEST/Lightwheel_Kitchen/Collected_KitchenRoom/KitchenRoom.usd"
if [[ ! -f "$kitchen" ]]; then
  echo "[sync] KitchenRoom.usd not found at $kitchen" >&2
  exit 1
fi
echo "[sync] ok"
