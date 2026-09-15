#!/usr/bin/env bash
set -eo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/env.sh"

cd "${PROJECT_ROOT}"
grip_source_ros

if ! ros2 service list --no-daemon --spin-time 1.0 | grep -Fxq '/indy_srv'; then
  echo 'ERROR: /indy_srv is unavailable. Start the Indy driver first.' >&2
  exit 1
fi

echo 'Moving Indy7 to the fixed collection HOME...'
ros2 service call /indy_srv indy_interfaces/srv/IndyService '{data: 2}'
