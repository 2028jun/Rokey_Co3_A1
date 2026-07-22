#!/usr/bin/env bash
# nav_robot2는 더 이상 사용하지 않습니다. nav_robot3 t1/t2/t3로 위임합니다.

echo "[aliases] nav_robot2는 deprecated — nav_robot3 (t1/t2/t3) 사용" >&2
_REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck source=/dev/null
source "$_REPO_ROOT/nav_robot3/tools/aliases.sh"
