#!/usr/bin/env bash
# Nova Carter 레일 주행 (nav_robot3) — 터미널별 t1 / t2 / t3
#
#   source ~/git/Rokey_Co3_A1/tools/aliases.sh
#
#   터미널1: t1  → Isaac Play
#   터미널2: t2  → Nav2
#   터미널3: t3 --table-id 2

_REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=/dev/null
source "$_REPO_ROOT/nav_robot3/tools/aliases.sh"
