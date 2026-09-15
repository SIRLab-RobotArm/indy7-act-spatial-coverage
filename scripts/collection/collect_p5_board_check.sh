#!/usr/bin/env bash
set -eo pipefail

HERE="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
source "$(cd "${HERE}/.." && pwd)/env.sh"

BOARD_CHECK_ID=${GRIP_BOARD_CHECK_ID:-board_shift_check_20260814}
export GRIP_OUTPUT_DIR=${PROJECT_ROOT}/episodes/diagnostic_episodes/${BOARD_CHECK_ID}/p5
export GRIP_DATA_BLOCK=pilot
export GRIP_SESSION_ID=${GRIP_BOARD_CHECK_SESSION_ID:-${BOARD_CHECK_ID}}

exec "${HERE}/collect_position.sh" grid_5 p5
