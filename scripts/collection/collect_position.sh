#!/usr/bin/env bash
set -eo pipefail

if [[ $# -ne 2 ]]; then
  echo "usage: $0 POSITION_ID POSITION_DIRECTORY" >&2
  exit 2
fi

POSITION_ID=$1
POSITION_DIRECTORY=$2

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/env.sh"

OUTPUT_DIRECTORY=${GRIP_OUTPUT_DIR:-${PROJECT_ROOT}/episodes/pre_recollection_episodes/${POSITION_DIRECTORY}}
DATA_BLOCK=${GRIP_DATA_BLOCK:-main}
SESSION_ID=${GRIP_SESSION_ID:-main_session}
OPERATOR_ID=${GRIP_OPERATOR_ID:-operator}

case "${DATA_BLOCK}" in
  dev|pilot|main) ;;
  *)
    echo "GRIP_DATA_BLOCK must be dev, pilot, or main: ${DATA_BLOCK}" >&2
    exit 2
    ;;
esac

cd "${PROJECT_ROOT}"
grip_source_ros
set -u

echo "RGB-only collection: ${POSITION_ID} -> ${OUTPUT_DIRECTORY}"
echo "data_block=${DATA_BLOCK} session=${SESSION_ID} operator=${OPERATOR_ID}"

exec ros2 launch grip_bringup collect_position.launch.py \
  position_id:="${POSITION_ID}" \
  output_dir:="${OUTPUT_DIRECTORY}" \
  data_block:="${DATA_BLOCK}" \
  session_id:="${SESSION_ID}" \
  operator_id:="${OPERATOR_ID}"
