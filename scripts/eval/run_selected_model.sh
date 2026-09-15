#!/usr/bin/env bash
set -eo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/env.sh"

cd "${PROJECT_ROOT}"
grip_activate_conda
grip_source_ros

exec python scripts/eval/run_selected_model.py "$@"
