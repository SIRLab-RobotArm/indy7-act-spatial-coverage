# Test suite

Regression tests for the RGB-only data contract, the ACT action conversion, the
real-robot safety interlocks, and the evaluation result processing. Nothing here
moves a robot: everything runs against mocks, fixtures and temporary directories.

## What is covered

- HDF5 to LeRobot four-phase conversion, and frozen condition identity
- State/action round-tripping and the RGB-only inference protocol
- Indy driver workspace and watchdog safety rules
- Xbox heartbeat and teach mode
- Recorder pause behaviour and rollout logger metadata
- Executor dry-run and the resident ZMQ model pool
- Evaluation schedule, operator UI state, rollout validator, grasp error, result
  export
- The ACT training baseline configuration

## Running them

```bash
conda activate act
pytest
```

`pyproject.toml` scopes pytest to this directory, so a bare `pytest` from the
repository root does the right thing.

Nine tests need something a plain clone does not have: seven drive the ROS
nodes and need ROS 2 plus this repository's built colcon workspace (including
the Indy7 driver cloned into `ros2_ws/src/indy7_ros2`, see
[`../ros2_ws/src/README.md`](../ros2_ws/src/README.md)), and two check the
trained checkpoints on disk. They skip themselves and say why, so a fresh clone
reports `70 passed, 9 skipped`. To run all 100:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
conda activate act
pytest
```

One test shells out to the system Python that runs ROS, which needs h5py and
pyyaml. If those are not in your system dist-packages, point `GRIP_ROS_PYDEPS` at
a directory that has them in `config/local.env`; `conftest.py` reads that file.

Run a single file the usual way: `pytest tests/test_export_results.py`.

`conftest.py` builds a temporary HDF5 fixture and converts it in a temp
directory. It never touches the project's real episodes or datasets.
