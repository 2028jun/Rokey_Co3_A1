#!/usr/bin/env bash
# Symlink restaurant / robot / Lightwheel assets from nav_robot6 (avoid 1.2G duplicate).
set -eo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SRC="${MAP_GEN_ASSET_SRC:-$(cd "$ROOT/../nav_robot6/assets" && pwd)}"
DEST="$ROOT/assets"

mkdir -p "$DEST"

link_one() {
  local name="$1"
  local target="$SRC/$name"
  local link="$DEST/$name"
  if [[ ! -e "$target" ]]; then
    echo "[sync] missing source: $target" >&2
    return 1
  fi
  ln -sfn "$target" "$link"
  echo "[sync] $link -> $target"
}

link_one lightweight_restaurant
link_one diagnostics
link_one Lightwheel_Kitchen

kitchen="$DEST/Lightwheel_Kitchen/Collected_KitchenRoom/KitchenRoom.usd"
if [[ ! -f "$kitchen" ]]; then
  echo "[sync] KitchenRoom.usd not found at $kitchen" >&2
  exit 1
fi
echo "[sync] ok"
