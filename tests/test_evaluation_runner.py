import json
from pathlib import Path
import subprocess

import pytest

import scripts.eval.run_evaluation as runner
from scripts.eval.trial_commands import trial_spec


def _attempt(root: Path, name: str, metadata: dict) -> Path:
    path = root / name
    (path / "rollout.bag").mkdir(parents=True)
    for relative in ("trace.parquet", "events.json", "rollout.bag/metadata.yaml"):
        (path / relative).write_text("placeholder\n", encoding="utf-8")
    (path / "meta.json").write_text(json.dumps(metadata), encoding="utf-8")
    return path


def test_attempt_needed_preserves_partial_and_uses_retry(tmp_path, monkeypatch):
    scheduled_id = "t001_m01_eval_1_r1"
    assert runner.attempt_needed(tmp_path, scheduled_id) == 1

    (tmp_path / scheduled_id).mkdir()
    assert runner.attempt_needed(tmp_path, scheduled_id) == 2

    excluded = _attempt(tmp_path, f"{scheduled_id}_retry1", {
        "success": False,
        "failure_code": "hardware_fault",
        "object_move": False,
    })
    monkeypatch.setattr(runner, "validate", lambda _path: {"valid": True})
    assert runner._completed_disposition(excluded) == "excluded"
    assert runner.attempt_needed(tmp_path, scheduled_id) == 3


def test_eligible_attempt_completes_scheduled_trial(tmp_path, monkeypatch):
    scheduled_id = "t002_m04_eval_6_r2"
    completed = _attempt(tmp_path, scheduled_id, {
        "success": False,
        "failure_code": "missed_grasp",
        "object_move": False,
    })
    monkeypatch.setattr(runner, "validate", lambda _path: {"valid": True})
    assert runner._completed_disposition(completed) == "eligible"
    assert runner.attempt_needed(tmp_path, scheduled_id) is None


def test_object_move_requires_retry_even_with_result(tmp_path, monkeypatch):
    scheduled_id = "t003_m03_eval_3_r1"
    completed = _attempt(tmp_path, scheduled_id, {
        "success": True,
        "failure_code": None,
        "object_move": True,
    })
    monkeypatch.setattr(runner, "validate", lambda _path: {"valid": True})
    assert runner._completed_disposition(completed) == "excluded"
    assert runner.attempt_needed(tmp_path, scheduled_id) == 2


def test_placed_prompt_shows_progress_condition_and_seed(capsys):
    spec = trial_spec(
        runner.REPO_ROOT /
        "experiment/evaluation/main_recollection_20260817/schedule_full.csv",
        runner.REPO_ROOT / "experiment/positions.yaml",
        4,
    )
    runner._print_placement_progress(spec)
    output = capsys.readouterr().out
    assert "Progress: 3/480 done" in output
    assert "CONDITION D" in output
    assert "SEED 1" in output
    assert "POSITION Q1" in output
    assert "repeat 1/5" in output


@pytest.mark.parametrize("value", ["home", "HOME", "HoMe"])
def test_confirmation_prompts_are_case_insensitive(monkeypatch, value):
    monkeypatch.setattr("builtins.input", lambda _message: value)

    runner._prompt_exact("prompt", "HOME")


@pytest.mark.parametrize(
    ("value", "expected"),
    [
        ("S", "success"),
        ("success", "success"),
        ("success", "success"),
        ("F", "failure"),
        ("FAILURE", "failure"),
        ("failure", "failure"),
    ],
)
def test_binary_outcome_accepts_case_insensitive_aliases(
        monkeypatch, value, expected):
    monkeypatch.setattr("builtins.input", lambda _message: value)

    assert runner._prompt_outcome() == expected


def test_policy_timeout_becomes_binary_failure(capsys):
    assert runner._prompt_outcome(0.0) == "failure"

    output = capsys.readouterr().out
    assert "0s limit" in output
    assert "recording it as a failure" in output


def test_trial_metadata_freezes_sixty_second_policy_horizon():
    spec = trial_spec(
        runner.REPO_ROOT /
        "experiment/evaluation/main_recollection_20260817/schedule_full.csv",
        runner.REPO_ROOT / "experiment/positions.yaml",
        4,
    )

    assert spec.metadata["policy_timeout_s"] == 60.0


def test_post_result_release_confirmation_is_case_insensitive(monkeypatch):
    monkeypatch.setattr("builtins.input", lambda _message: "release")

    runner._prompt_exact("prompt", "RELEASE")


def test_rosbag_stop_uses_recorder_stop_service(monkeypatch):
    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        return subprocess.CompletedProcess(command, 0, "response:\n", "")

    monkeypatch.setattr(runner.subprocess, "run", fake_run)

    runner._stop_rosbag(timeout=12.0)

    assert calls[0][0] == [
        "ros2", "service", "call", "/rosbag2_recorder/stop",
        "rosbag2_interfaces/srv/Stop", "{}",
    ]
    assert calls[0][1]["timeout"] == 12.0


def test_service_discovery_bypasses_ros2_daemon(monkeypatch):
    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        return subprocess.CompletedProcess(
            command, 0, "/grip_eval/start\n/indy_srv\n", "")

    monkeypatch.setattr(runner.subprocess, "run", fake_run)

    assert runner._service_names() == {"/grip_eval/start", "/indy_srv"}
    assert calls[0][0] == [
        "ros2", "service", "list", "--no-daemon", "--spin-time", "1.0",
    ]
    assert calls[0][1]["timeout"] == 10


def test_select_act_model_sends_blind_model_code(monkeypatch):
    class FakeSocket:
        def __init__(self):
            self.options = []
            self.endpoint = None
            self.request = None
            self.closed = False

        def setsockopt(self, name, value):
            self.options.append((name, value))

        def connect(self, endpoint):
            self.endpoint = endpoint

        def send_json(self, request):
            self.request = request

        def recv_json(self):
            return {"ok": True, "model_code": self.request["model_code"]}

        def close(self, linger):
            self.closed = linger == 0

    socket = FakeSocket()

    class FakeContext:
        def socket(self, socket_type):
            assert socket_type == runner.zmq.REQ
            return socket

    monkeypatch.setattr(
        runner.zmq.Context, "instance", lambda: FakeContext())

    runner._select_act_model("M09", timeout_ms=3210)

    assert socket.endpoint == "tcp://127.0.0.1:5557"
    assert socket.request == {
        "protocol": "grip_act_rgb_v1",
        "command": "select",
        "model_code": "M09",
    }
    assert socket.closed


def test_position_summary_prints_reference_actual_and_error(capsys):
    spec = trial_spec(
        runner.REPO_ROOT /
        "experiment/evaluation/main_recollection_20260817/schedule_full.csv",
        runner.REPO_ROOT / "experiment/positions.yaml",
        1,
    )
    result = {
        "grasp_position": {
            "target_xyz_mm": [1.0, 2.0, 3.0],
            "grasp_xyz_mm": [2.0, 4.0, 6.0],
            "delta_xyz_mm": [1.0, 2.0, 3.0],
            "xy_error_mm": 2.236,
            "error_3d_mm": 3.742,
        }
    }

    runner._print_grasp_position(result, spec)

    output = capsys.readouterr().out
    assert "Q interpolation reference" in output
    assert "Measured first-close EEF XYZ" in output
    assert "3-D distance error: 3.742 mm" in output
