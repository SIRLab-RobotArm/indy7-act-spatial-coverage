import json
import os
from pathlib import Path
import subprocess
import ast

import h5py
import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]

# The two dry-run tests below drive the executor as a subprocess under the
# system Python that runs ROS, so they need ROS 2 and this repository's built
# colcon workspace. The AST checks further down do not, and always run.
_HAS_ROS_PYTHON = subprocess.run(
    ["/usr/bin/python3", "-c", "import rclpy"],
    capture_output=True, check=False).returncode == 0
needs_ros_python = pytest.mark.skipif(
    not _HAS_ROS_PYTHON,
    reason="needs ROS 2 available to /usr/bin/python3: "
           "source /opt/ros/jazzy/setup.bash and ros2_ws/install/setup.bash")


@needs_ros_python
def test_executor_dry_run_matches_recorded_delta_actions(tmp_path):
    episode = tmp_path / "episode.h5"
    frames = 25
    ee = np.zeros((frames, 6), dtype=np.float32)
    ee[:, 0] = np.arange(frames, dtype=np.float32) * 2.0
    ee[:, 1] = np.arange(frames, dtype=np.float32) * -0.5
    ee[:, 2] = 100.0 + np.arange(frames, dtype=np.float32) * 0.25
    grip = np.zeros(frames, dtype=np.uint8)
    grip[8:] = 1
    with h5py.File(episode, "w") as output:
        output.create_dataset("/state/ee_pose", data=ee)
        output.create_dataset("/state/gripper_cmd", data=grip)
    env = os.environ.copy()
    env["PYTHONPATH"] = os.pathsep.join((
        str(ROOT / "ros2_ws/src/grip_eval"),
        str(ROOT),
        env.get("GRIP_ROS_PYDEPS", ""),
        env.get("PYTHONPATH", ""),
    ))
    result = subprocess.run([
        "/usr/bin/python3", "-m", "grip_eval.policy_executor_node",
        "--dry-run-h5", str(episode),
    ], cwd=ROOT, env=env, check=True, capture_output=True, text=True)
    document = json.loads(result.stdout)
    actions = np.asarray(document["recorded_actions"], dtype=np.float32)
    np.testing.assert_array_equal(actions[:, 3], np.array([0, 1, 1, 1, 1, 1]))
    commands = np.asarray(
        [item["pose_delta"][:3] for item in document["executor_commands"]],
        dtype=np.float32)
    gripper = np.asarray(
        [item["gripper_cmd"] for item in document["executor_commands"]], dtype=np.float32)
    np.testing.assert_array_equal(commands, actions[:, :3])
    np.testing.assert_array_equal(gripper, actions[:, 3])
    targets = np.asarray(
        [item["pose_target"][:3] for item in document["executor_commands"]],
        dtype=np.float32)
    np.testing.assert_array_equal(targets, np.cumsum(actions[:, :3], axis=0))


@needs_ros_python
def test_executor_interpolates_each_5hz_endpoint_at_30hz():
    from grip_eval.policy_executor_node import (
        COMMAND_HZ,
        INTERPOLATION_STEPS,
        interpolated_targets,
    )

    start = np.array([3.0, -2.0, 1.0, 0.0, 0.0, 0.0], dtype=np.float32)
    end = np.array([9.0, 4.0, -5.0, 0.0, 0.0, 0.0], dtype=np.float32)
    targets = interpolated_targets(start, end)

    assert COMMAND_HZ == 30.0
    assert INTERPOLATION_STEPS == 6
    assert targets.shape == (6, 6)
    np.testing.assert_allclose(targets[-1], end, rtol=0, atol=0)
    increments = np.diff(np.vstack((start, targets)), axis=0)
    np.testing.assert_allclose(
        increments, np.repeat(((end - start) / 6.0)[None, :], 6, axis=0),
        rtol=0, atol=1e-6)


def test_executor_source_has_no_banned_tuning_knobs():
    source = (ROOT / "ros2_ws/src/grip_eval/grip_eval/policy_executor_node.py").read_text()
    banned = (
        "action_delta_scale", "grasp_delay_steps", "pre_grasp_delta_scale",
        "grasp_confirm_steps", "grasp_max_delta_norm", "grasp_min_elapsed_steps",
        "grasp_min_motion_rad", "trajectory_horizon_sec",
    )
    assert not any(name in source for name in banned)


def test_executor_start_rejects_latched_driver_fault():
    source_path = ROOT / "ros2_ws/src/grip_eval/grip_eval/policy_executor_node.py"
    tree = ast.parse(source_path.read_text())
    start = next(
        node for node in ast.walk(tree)
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        and node.name == "_start"
    )
    start_source = ast.unparse(start)
    assert "fault_latch" in start_source
    assert "recover required" in start_source


def test_executor_has_manual_stop_only_and_no_chunk_boundary_stop():
    source_path = ROOT / "ros2_ws/src/grip_eval/grip_eval/policy_executor_node.py"
    source = source_path.read_text()
    tree = ast.parse(source)
    assert "TRIAL_TIMEOUT_SEC" not in source
    assert "self.deadline" not in source
    chunk_boundary = next(
        node for node in ast.walk(tree)
        if isinstance(node, ast.If)
        and "self.chunk_index >= EXECUTE_STEPS" in ast.unparse(node.test)
    )
    assert "MSG_TELE_STOP" not in ast.unparse(chunk_boundary)
    assert "_predict_chunk" in ast.unparse(chunk_boundary)


def test_executor_tick_streams_interpolated_targets_at_command_rate():
    source_path = ROOT / "ros2_ws/src/grip_eval/grip_eval/policy_executor_node.py"
    source = source_path.read_text()
    assert "self.create_timer(1.0 / COMMAND_HZ, self._tick)" in source
    assert "self.command_targets[self.command_index]" in source
    assert "self.command_index < INTERPOLATION_STEPS" in source
