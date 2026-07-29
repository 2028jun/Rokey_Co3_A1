#!/usr/bin/env bash
set -euo pipefail

REPOSITORY="2028jun/Rokey_Co3_A1"
RELEASE_TAG="kitchen-runtime-v1"
ARCHIVE_NAME="Lightwheel_Kitchen_runtime_v1.tar.zst"
EXPECTED_SHA256="6edbef8b996cd053aa7b0b7570e6aff8825d0610af879deb93dae0ab504c5bd4"

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
asset_parent="$workspace/assets/Lightwheel_Kitchen"
asset_dir="$asset_parent/Collected_KitchenRoom"
archive_override=""
force=0

usage() {
  cat <<'EOF'
Usage: ./tools/install_kitchen_assets.sh [--force] [--archive FILE]

Downloads and installs the Lightwheel Kitchen runtime required by Isaac Sim.

Options:
  --force         Replace an existing Collected_KitchenRoom directory.
  --archive FILE  Install from a local release archive instead of downloading.
  -h, --help      Show this help.
EOF
}

while (($#)); do
  case "$1" in
    --force)
      force=1
      shift
      ;;
    --archive)
      if (($# < 2)); then
        echo "[kitchen] --archive requires a file path." >&2
        exit 2
      fi
      archive_override="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[kitchen] unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -f "$asset_dir/KitchenRoom.usd" && $force -eq 0 ]]; then
  echo "[kitchen] already installed: $asset_dir/KitchenRoom.usd"
  echo "[kitchen] use --force to replace it."
  exit 0
fi

for command_name in sha256sum tar zstd; do
  if ! command -v "$command_name" >/dev/null 2>&1; then
    echo "[kitchen] missing command: $command_name" >&2
    echo "[kitchen] install it first (Ubuntu: sudo apt install zstd)." >&2
    exit 1
  fi
done

tmp_dir="$(mktemp -d "${TMPDIR:-/tmp}/kitchen-assets.XXXXXX")"
cleanup() {
  rm -rf "$tmp_dir"
}
trap cleanup EXIT

archive="$tmp_dir/$ARCHIVE_NAME"
if [[ -n "$archive_override" ]]; then
  if [[ ! -f "$archive_override" ]]; then
    echo "[kitchen] archive not found: $archive_override" >&2
    exit 1
  fi
  cp "$archive_override" "$archive"
else
  if ! command -v curl >/dev/null 2>&1; then
    echo "[kitchen] missing command: curl" >&2
    exit 1
  fi
  url="https://github.com/$REPOSITORY/releases/download/$RELEASE_TAG/$ARCHIVE_NAME"
  echo "[kitchen] downloading $url"
  curl --fail --location --retry 3 --continue-at - \
    --output "$archive" "$url"
fi

actual_sha256="$(sha256sum "$archive" | awk '{print $1}')"
if [[ "$actual_sha256" != "$EXPECTED_SHA256" ]]; then
  echo "[kitchen] checksum mismatch; refusing to install." >&2
  echo "[kitchen] expected: $EXPECTED_SHA256" >&2
  echo "[kitchen] actual:   $actual_sha256" >&2
  exit 1
fi
echo "[kitchen] checksum verified: $actual_sha256"

extract_root="$tmp_dir/extracted"
mkdir -p "$extract_root"
tar --zstd -xf "$archive" -C "$extract_root"

extracted_dir="$extract_root/Collected_KitchenRoom"
if [[ ! -f "$extracted_dir/KitchenRoom.usd" ]]; then
  echo "[kitchen] archive is missing Collected_KitchenRoom/KitchenRoom.usd" >&2
  exit 1
fi

mkdir -p "$asset_parent"
if [[ -e "$asset_dir" ]]; then
  if [[ $force -ne 1 ]]; then
    echo "[kitchen] destination already exists: $asset_dir" >&2
    echo "[kitchen] use --force to replace it." >&2
    exit 1
  fi
  rm -rf "$asset_dir"
fi
mv "$extracted_dir" "$asset_dir"

echo "[kitchen] installed: $asset_dir"
echo "[kitchen] ready: $asset_dir/KitchenRoom.usd"
