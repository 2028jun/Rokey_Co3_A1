#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
asset_root="${workspace}/assets/Lightwheel_Kitchen"
scene="${asset_root}/Collected_KitchenRoom/KitchenRoom.usd"
output="${1:-${workspace}/dist/Lightwheel_Kitchen_runtime_cc-by-nc-4.0.tar.zst}"

if [[ ! -f "${scene}" ]]; then
  echo "Missing Lightwheel Kitchen scene: ${scene}" >&2
  exit 1
fi
if [[ ! -f "${asset_root}/LICENSE.txt" || ! -f "${asset_root}/README.md" ]]; then
  echo "README.md and LICENSE.txt must accompany the CC BY-NC 4.0 asset." >&2
  exit 1
fi

mkdir -p "$(dirname "${output}")"
temporary="${output}.partial"
rm -f "${temporary}"

echo "Packaging Lightwheel Kitchen..."
tar \
  --zstd \
  --exclude='.DS_Store' \
  --exclude='.collect.mapping.json' \
  -cf "${temporary}" \
  -C "${workspace}/assets" \
  Lightwheel_Kitchen
mv "${temporary}" "${output}"
output_dir="$(dirname "${output}")"
output_name="$(basename "${output}")"
(
  cd "${output_dir}"
  sha256sum "${output_name}" > "${output_name}.sha256"
)

echo "Archive: ${output}"
echo "Checksum: ${output}.sha256"
du -h "${output}" "${output}.sha256"
