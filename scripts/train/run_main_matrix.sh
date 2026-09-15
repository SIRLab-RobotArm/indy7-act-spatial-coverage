#!/usr/bin/env bash

set -eo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/env.sh"

repo_root="${PROJECT_ROOT}"
experiment_id="main_recollection_20260817"
log_dir="${repo_root}/experiment/training_logs"

cd "${repo_root}"
grip_activate_conda

export PYTHONUNBUFFERED=1
mkdir -p "${log_dir}"

for condition in a b c d; do
  dataset_dir="datasets/${experiment_id}_rgb_${condition}"
  dataset_repo_id="sirlab/grip_recollection_20260817_${condition}"

  for seed in 0 1 2; do
    run_name="${experiment_id}_${condition}_s${seed}_100000"
    output_dir="ai/models/${run_name}"
    log_path="${log_dir}/${run_name}.log"

    if [[ -e "${output_dir}" ]]; then
      echo "Refusing to overwrite existing output: ${output_dir}" >&2
      exit 1
    fi

    echo "Starting ${run_name} at $(date --iso-8601=seconds)"
    python -m ai.train.run_train \
      --dataset-dir "${dataset_dir}" \
      --dataset-repo-id "${dataset_repo_id}" \
      --output-dir "${output_dir}" \
      --job-name "${run_name}" \
      --seed "${seed}" \
      --steps 100000 \
      --batch-size 64 \
      --num-workers 14 \
      --save-freq 20000 \
      --log-freq 50 \
      --ram-cache 2>&1 | tee "${log_path}"
    echo "Finished ${run_name} at $(date --iso-8601=seconds)"
  done
done
