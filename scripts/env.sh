# Shared environment bootstrap for every wrapper script in this repository.
#
# Source it, do not run it:
#
#     source "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/env.sh"
#
# It works out where the repository lives from its own location, loads
# config/local.env if you made one, and fills in a default for anything you did
# not set. See config/local.env.example for what each setting means.

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PROJECT_ROOT
export GRIP_REPO_ROOT="${PROJECT_ROOT}"

if [[ -f "${PROJECT_ROOT}/config/local.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "${PROJECT_ROOT}/config/local.env"
  set +a
fi

: "${GRIP_CONDA_SETUP:=${HOME}/miniconda3/etc/profile.d/conda.sh}"
: "${GRIP_CONDA_ENV:=act}"
: "${GRIP_SYSTEM_PYTHON:=/usr/bin/python3}"
: "${GRIP_ROS_SETUP:=/opt/ros/jazzy/setup.bash}"
: "${GRIP_ROS_PYDEPS:=}"
: "${GRIP_INDY_IP:=192.168.0.10}"
: "${GRIP_CAMERA_SERIAL:=_000000000000}"
: "${GRIP_MARK7_PORT:=/dev/ttyACM0}"
: "${GRIP_XBOX_DEVICE:=/dev/input/by-id/usb-Microsoft_Controller-event-joystick}"
: "${GRIP_ACT_ENDPOINT:=tcp://127.0.0.1:5557}"
: "${ROS_DOMAIN_ID:=0}"
: "${ROS_AUTOMATIC_DISCOVERY_RANGE:=LOCALHOST}"

export GRIP_CONDA_SETUP GRIP_CONDA_ENV GRIP_SYSTEM_PYTHON
export GRIP_ROS_SETUP GRIP_ROS_PYDEPS
export GRIP_INDY_IP GRIP_CAMERA_SERIAL GRIP_MARK7_PORT GRIP_XBOX_DEVICE
export GRIP_ACT_ENDPOINT
export ROS_DOMAIN_ID ROS_AUTOMATIC_DISCOVERY_RANGE
export HF_HOME="${HF_HOME:-${PROJECT_ROOT}/ai/.cache/huggingface}"

# Activate the conda environment that holds PyTorch and LeRobot.
grip_activate_conda() {
  if [[ ! -f "${GRIP_CONDA_SETUP}" ]]; then
    echo "ERROR: conda hook not found at ${GRIP_CONDA_SETUP}." >&2
    echo "       Set GRIP_CONDA_SETUP in config/local.env (see config/local.env.example)." >&2
    return 1
  fi
  # shellcheck disable=SC1090
  source "${GRIP_CONDA_SETUP}"
  conda activate "${GRIP_CONDA_ENV}"
}

# Source ROS 2 plus this repository's built colcon workspace.
grip_source_ros() {
  if [[ ! -f "${GRIP_ROS_SETUP}" ]]; then
    echo "ERROR: ROS 2 setup not found at ${GRIP_ROS_SETUP}." >&2
    echo "       Set GRIP_ROS_SETUP in config/local.env (see config/local.env.example)." >&2
    return 1
  fi
  # shellcheck disable=SC1090
  source "${GRIP_ROS_SETUP}"

  local workspace_setup="${PROJECT_ROOT}/ros2_ws/install/setup.bash"
  if [[ ! -f "${workspace_setup}" ]]; then
    echo "ERROR: ${workspace_setup} is missing - build the workspace first:" >&2
    echo "       cd ros2_ws && colcon build --symlink-install" >&2
    return 1
  fi
  # shellcheck disable=SC1090
  source "${workspace_setup}"

  if [[ -n "${GRIP_ROS_PYDEPS}" ]]; then
    export PYTHONPATH="${GRIP_ROS_PYDEPS}${PYTHONPATH:+:${PYTHONPATH}}"
  fi
}

# Fail early with a clear message when a GUI script has no display.
grip_require_display() {
  if [[ -z "${DISPLAY:-}" ]]; then
    echo "ERROR: no graphical DISPLAY. Run this from the desktop session of the" >&2
    echo "       machine that is wired to the robot, not over a plain SSH shell." >&2
    return 1
  fi
}
