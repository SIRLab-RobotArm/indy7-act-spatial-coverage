#!/usr/bin/env bash
set -eo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/env.sh"

cd "${PROJECT_ROOT}"
grip_require_display

exec "${GRIP_SYSTEM_PYTHON}" scripts/eval/model_run_ui.py
