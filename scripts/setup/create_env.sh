#!/usr/bin/env bash
#
# Build the Python environment this repository needs, from scratch.
#
#     ./scripts/setup/create_env.sh              # CPU-only, fine for analysis
#     ./scripts/setup/create_env.sh cu130        # CUDA 13.0 build of PyTorch
#     ./scripts/setup/create_env.sh cu121        # CUDA 12.1 build of PyTorch
#
# Run `nvidia-smi` to see which CUDA your driver supports. If you only want to
# re-check the published numbers you do not need a GPU at all: the default
# CPU build is enough for the tests and the analysis.

set -eo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/env.sh"
cd "${PROJECT_ROOT}"

CUDA_TAG="${1:-cpu}"

# The conda hook has to be sourced even when `conda` is already on PATH:
# `conda activate` is a shell function, and a non-interactive shell does not
# have it until the hook runs.
if [[ -f "${GRIP_CONDA_SETUP}" ]]; then
  # shellcheck disable=SC1090
  source "${GRIP_CONDA_SETUP}"
elif command -v conda >/dev/null 2>&1; then
  eval "$(conda shell.bash hook)"
else
  echo "ERROR: conda not found, and ${GRIP_CONDA_SETUP} does not exist." >&2
  echo "       Install Miniconda, then set GRIP_CONDA_SETUP in config/local.env." >&2
  exit 1
fi

echo "==> 1/4 creating the '${GRIP_CONDA_ENV}' environment (Python 3.12)"
if conda env list | awk '{print $1}' | grep -Fxq "${GRIP_CONDA_ENV}"; then
  echo "    already exists, reusing it"
else
  conda env create -f environment.yml -n "${GRIP_CONDA_ENV}"
fi

# shellcheck disable=SC1091
conda activate "${GRIP_CONDA_ENV}"

echo "==> 2/4 installing PyTorch (${CUDA_TAG})"
# The version bounds match what the vendored LeRobot 0.5.1 requires. Without
# them, step 4 would resolve a different PyTorch and quietly replace this one -
# pulling in the CUDA packages even for a CPU install.
pip install "torch>=2.2.1,<2.11.0" "torchvision>=0.21.0,<0.26.0" torchcodec \
  --index-url "https://download.pytorch.org/whl/${CUDA_TAG}"

echo "==> 3/4 installing the remaining Python packages"
pip install -r requirements.txt

echo "==> 4/4 installing the vendored LeRobot 0.5.1 in editable mode"
pip install -e ai/lerobot_source/lerobot

cat <<EOF

Done. Activate it whenever you work on this repository:

    conda activate ${GRIP_CONDA_ENV}

Then check that everything imports:

    python -c "import torch, lerobot; print(torch.__version__, torch.cuda.is_available())"
    pytest
EOF
