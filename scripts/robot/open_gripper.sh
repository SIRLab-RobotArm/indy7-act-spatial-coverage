#!/usr/bin/env bash
set -eo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/env.sh"

cd "${PROJECT_ROOT}"
grip_source_ros

services=$(ros2 service list --no-daemon --spin-time 1.0)
if ! grep -Fxq /gripper/open <<<"${services}"; then
  echo "ERROR: /gripper/open service is unavailable. Wait for the evaluation graph." >&2
  exit 1
fi

output=$(ros2 service call /gripper/open std_srvs/srv/Trigger "{}")
printf '%s\n' "${output}"
if ! grep -Eq 'success[=:][[:space:]]*(True|true)' <<<"${output}"; then
  echo "ERROR: gripper open service returned failure." >&2
  exit 1
fi
