#!/usr/bin/env python3
"""Run the frozen real-robot evaluation from one interactive terminal.

The program owns only the ACT and ROS processes that it starts.  Robot motion
always remains behind explicit operator prompts; the program never bypasses
the physical E-stop or decides grasp success automatically.
"""

from __future__ import annotations

import argparse
from contextlib import suppress
from dataclasses import replace
import json
import os
from pathlib import Path
import re
import select
import signal
import socket
import subprocess
import sys
import tempfile
import time
import zmq


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from ai.eval.validate_rollout import validate
from ai.eval.protocol import POLICY_TIMEOUT_S
from scripts.eval.trial_commands import (
    DEFAULT_OUTPUT_ROOT,
    DEFAULT_POSITIONS,
    DEFAULT_SCHEDULE,
    TrialSpec,
    trial_spec,
)


ACT_HOST = "127.0.0.1"
ACT_PORT = 5557
PILOT_OUTPUT_ROOT = REPO_ROOT / "experiment/rollouts/_pilot/smooth30hz_controller"
PILOT_TRIAL = 5  # Condition A/seed 0 at the demonstrated G5 location.
MODEL_KEY = (
    REPO_ROOT / "experiment/evaluation/main_recollection_20260817/model_key.json")
REQUIRED_SERVICES = (
    "/indy_srv",
    "/gripper/open",
    "/grip_eval/log/start",
    "/grip_eval/log/stop",
    "/grip_eval/start",
    "/grip_eval/stop",
    "/grip_eval/result/success",
    "/grip_eval/result/failure/operator_failure",
    "/rosbag2_recorder/stop",
)
POSITION_LABELS = {
    "eval_1": "G5",
    "eval_2": "G1",
    "eval_3": "G6",
    "eval_4": "G8",
    "eval_5": "Q1 (centre of G1/G2/G4/G5)",
    "eval_6": "Q2 (centre of G2/G3/G5/G6)",
    "eval_7": "Q3 (centre of G4/G5/G7/G8)",
    "eval_8": "Q4 (centre of G5/G6/G8/G9)",
}
# Accepted spellings of the operator's verdict. The Korean words are kept so
# that operators who ran the original sessions can keep typing what they know.
OUTCOME_ALIASES = {
    "s": "success",
    "success": "success",
    "성공": "success",
    "f": "failure",
    "failure": "failure",
    "실패": "failure",
}
OUTCOME_SERVICES = {
    "success": "/grip_eval/result/success",
    "failure": "/grip_eval/result/failure/operator_failure",
}


class RunnerError(RuntimeError):
    pass


def _resolve_from_repo(path: Path) -> Path:
    return path.resolve() if path.is_absolute() else (REPO_ROOT / path).resolve()


def _attempt_path(output_root: Path, scheduled_id: str, attempt: int) -> Path:
    suffix = "" if attempt == 1 else f"_retry{attempt - 1}"
    return output_root / f"{scheduled_id}{suffix}"


def _used_attempts(output_root: Path, scheduled_id: str) -> list[tuple[int, Path]]:
    pattern = re.compile(rf"^{re.escape(scheduled_id)}(?:_retry([1-9][0-9]*))?$")
    attempts = []
    if not output_root.exists():
        return attempts
    for path in output_root.iterdir():
        if not path.is_dir():
            continue
        match = pattern.fullmatch(path.name)
        if match:
            number = 1 if match.group(1) is None else int(match.group(1)) + 1
            attempts.append((number, path))
    return sorted(attempts)


def _completed_disposition(path: Path) -> str | None:
    """Return eligible/excluded for a finalized attempt, else None."""
    required = (
        path / "trace.parquet",
        path / "events.json",
        path / "meta.json",
        path / "rollout.bag/metadata.yaml",
    )
    if not all(item.is_file() for item in required):
        return None
    try:
        metadata = json.loads((path / "meta.json").read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    if metadata.get("success") is None:
        return None
    try:
        if not validate(path)["valid"]:
            return None
    except Exception:
        return None
    if metadata.get("object_move") or metadata.get("failure_code") == "hardware_fault":
        return "excluded"
    return "eligible"


def attempt_needed(output_root: Path, scheduled_id: str) -> int | None:
    """Return the next safe attempt, or None if the scheduled trial is done."""
    attempts = _used_attempts(output_root, scheduled_id)
    if any(_completed_disposition(path) == "eligible" for _, path in attempts):
        return None
    return max((number for number, _ in attempts), default=0) + 1


def first_pending_trial(
    schedule: Path, positions: Path, output_root: Path
) -> tuple[int, int] | None:
    first = trial_spec(schedule, positions, 1)
    for trial in range(1, first.trial_count + 1):
        base = trial_spec(schedule, positions, trial)
        attempt = attempt_needed(output_root, base.scheduled_rollout_id)
        if attempt is not None:
            return trial, attempt
    return None


def _port_open(host: str = ACT_HOST, port: int = ACT_PORT) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(0.25)
        return sock.connect_ex((host, port)) == 0


def _tail(path: Path, line_count: int = 30) -> str:
    with suppress(OSError):
        lines = path.read_text(
            encoding="utf-8", errors="replace").splitlines(True)
        return "".join(lines[-line_count:])
    return ""


def _wait_until(predicate, process: subprocess.Popen, timeout: float, description: str,
                log_path: Path) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RunnerError(
                f"The process exited before {description}.\n"
                f"Log: {log_path}\n{_tail(log_path)}")
        if predicate():
            return
        time.sleep(0.5)
    raise RunnerError(
        f"Timed out after {timeout:.0f}s waiting for {description}.\n"
        f"Log: {log_path}\n{_tail(log_path)}")


def _wait_for_act_pool(
        process: subprocess.Popen, timeout: float, log_path: Path) -> None:
    """Wait for all resident models while relaying load progress to the UI."""
    deadline = time.monotonic() + timeout
    offset = 0
    while time.monotonic() < deadline:
        with suppress(OSError):
            content = log_path.read_text(encoding="utf-8", errors="replace")
            if len(content) > offset:
                print(content[offset:], end="", flush=True)
                offset = len(content)
        if process.poll() is not None:
            raise RunnerError(
                "The process exited before the ACT model pool was ready.\n"
                f"Log: {log_path}\n{_tail(log_path)}")
        if _port_open():
            return
        time.sleep(0.5)
    raise RunnerError(
        f"The ACT model pool took longer than {timeout:.0f}s to become ready.\n"
        f"Log: {log_path}\n{_tail(log_path)}")


def _service_names() -> set[str]:
    try:
        result = subprocess.run(
            [
                "ros2", "service", "list", "--no-daemon",
                "--spin-time", "1.0",
            ], cwd=REPO_ROOT,
            text=True, capture_output=True, timeout=10, check=False)
    except subprocess.TimeoutExpired as exc:
        raise RunnerError(
            "Even the daemon-free service query did not finish within 10s."
        ) from exc
    if result.returncode != 0:
        detail = (result.stdout + result.stderr).strip()
        raise RunnerError(f"ROS 2 service query failed: {detail}")
    return set(result.stdout.splitlines())


def _call_trigger(service: str, timeout: float = 180.0) -> str:
    result = subprocess.run(
        ["ros2", "service", "call", service, "std_srvs/srv/Trigger", "{}"],
        cwd=REPO_ROOT, text=True, capture_output=True, timeout=timeout, check=False)
    output = (result.stdout + result.stderr).strip()
    print(output)
    success = re.search(r"success\s*[=:]\s*(?:True|true)", output)
    if result.returncode != 0 or not success:
        raise RunnerError(f"Service call failed: {service}")
    return output


def _stop_rosbag(timeout: float = 180.0) -> str:
    """Finalize the rollout bag while leaving the robot graph available."""
    result = subprocess.run(
        [
            "ros2", "service", "call", "/rosbag2_recorder/stop",
            "rosbag2_interfaces/srv/Stop", "{}",
        ],
        cwd=REPO_ROOT, text=True, capture_output=True, timeout=timeout, check=False)
    output = (result.stdout + result.stderr).strip()
    print(output)
    if result.returncode != 0:
        raise RunnerError("Failed to stop and finalise the rosbag.")
    return output


def _select_act_model(model_code: str, timeout_ms: int = 15_000) -> None:
    """Select one already resident model before starting the ROS graph."""
    context = zmq.Context.instance()
    socket = context.socket(zmq.REQ)
    socket.setsockopt(zmq.RCVTIMEO, int(timeout_ms))
    socket.setsockopt(zmq.SNDTIMEO, int(timeout_ms))
    socket.connect(f"tcp://{ACT_HOST}:{ACT_PORT}")
    try:
        socket.send_json({
            "protocol": "grip_act_rgb_v1",
            "command": "select",
            "model_code": model_code,
        })
        reply = socket.recv_json()
    except zmq.ZMQError as exc:
        raise RunnerError(f"Could not select ACT model {model_code}: {exc}") from exc
    finally:
        socket.close(linger=0)
    if not reply.get("ok") or reply.get("model_code") != model_code:
        raise RunnerError(f"Could not select ACT model {model_code}: {reply}")


def _back_home() -> None:
    result = subprocess.run(
        [str(REPO_ROOT / "scripts/robot/back_home.sh")], cwd=REPO_ROOT,
        text=True, capture_output=True, timeout=180, check=False)
    output = (result.stdout + result.stderr).strip()
    print(output)
    if result.returncode != 0 or not re.search(
            r"success\s*[=:]\s*(?:True|true)", output):
        raise RunnerError("The move-to-HOME command failed.")


def _stop_owned_process(process: subprocess.Popen | None, name: str) -> None:
    if process is None or process.poll() is not None:
        return
    with suppress(ProcessLookupError):
        os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=30)
    except subprocess.TimeoutExpired:
        print(f"WARNING: {name} did not stop on SIGINT; sending SIGTERM.")
        with suppress(ProcessLookupError):
            os.killpg(process.pid, signal.SIGTERM)
        with suppress(subprocess.TimeoutExpired):
            process.wait(timeout=10)


def _prompt_exact(message: str, expected: str) -> None:
    answer = input(message).strip().casefold()
    if answer != expected.casefold():
        raise RunnerError(f"Aborting: '{expected}' was not entered.")


def _timed_stdin(message: str, timeout_s: float) -> str | None:
    """Read one terminal line, returning None when the deadline expires."""
    print(message, end="", flush=True)
    readable, _, _ = select.select([sys.stdin], [], [], max(0.0, timeout_s))
    if not readable:
        return None
    line = sys.stdin.readline()
    if line == "":
        raise RunnerError("The result input terminal was closed.")
    return line.strip()


def _prompt_outcome(timeout_s: float | None = None) -> str:
    deadline = None if timeout_s is None else time.monotonic() + float(timeout_s)
    while True:
        if deadline is None:
            answer = input("Outcome (S=success, F=failure)> ").strip()
        else:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                print(
                    f"\nPolicy run reached its {float(timeout_s):.0f}s limit - "
                    "recording it as a failure and stopping the policy.")
                return "failure"
            answer = _timed_stdin(
                "Outcome (S=success, F=failure)> ", remaining)
            if answer is None:
                print(
                    f"\nPolicy run reached its {float(timeout_s):.0f}s limit - "
                    "recording it as a failure and stopping the policy.")
                return "failure"
        answer = answer.casefold()
        outcome = OUTCOME_ALIASES.get(answer)
        if outcome is not None:
            return outcome
        print("Enter one of: S, F, success, failure.")


def _print_grasp_position(result: dict, spec: TrialSpec) -> None:
    position = result.get("grasp_position") or {}
    target = position.get("target_xyz_mm")
    grasp = position.get("grasp_xyz_mm")
    delta = position.get("delta_xyz_mm")
    print("\nGrasp position:")
    if spec.position_id in {"eval_1", "eval_2", "eval_3", "eval_4"}:
        target_label = "mean first-close pose of the training demonstrations"
    else:
        target_label = "Q interpolation reference, fixed as the mean of the neighbouring grid points"
    if target is not None:
        print(f"- {target_label} XYZ: {[round(value, 3) for value in target]} mm")
    if grasp is None or delta is None:
        print("- No first-close was recorded, so the grasp position error cannot be computed.")
        return
    print(f"- Measured first-close EEF XYZ: {[round(value, 3) for value in grasp]} mm")
    print(f"- Signed error XYZ (measured - reference): {[round(value, 3) for value in delta]} mm")
    print(f"- XY distance error: {position['xy_error_mm']:.3f} mm")
    print(f"- 3-D distance error: {position['error_3d_mm']:.3f} mm")


def _print_trial(spec: TrialSpec) -> None:
    label = POSITION_LABELS.get(spec.position_id, spec.position_id)
    print("\n" + "=" * 72)
    print(
        f"Trial {spec.trial}/{spec.trial_count} | attempt {spec.attempt} | "
        f"session {spec.session} | {spec.model_code} "
        f"(condition {spec.condition}, seed {spec.seed})")
    print(f"Evaluation position: {spec.position_id} = {label} | repeat {spec.repeat}/5")
    print(f"Frozen p_target XYZ: {[round(value, 3) for value in spec.p_target]} mm")
    print(f"Experiment tag: {spec.metadata['experiment_tag']}")
    print(f"Output path: {spec.output_root}")
    print(f"rollout: {spec.rollout_id}")
    print("=" * 72)


def _print_placement_progress(spec: TrialSpec) -> None:
    label = POSITION_LABELS.get(spec.position_id, spec.position_id)
    completed = spec.trial - 1
    percent = 100.0 * completed / spec.trial_count
    print("\n" + "#" * 72)
    print(
        f"Progress: {completed}/{spec.trial_count} done "
        f"({percent:.1f}%) - now running trial {spec.trial}")
    print(
        f"CONDITION {spec.condition} | SEED {spec.seed} | MODEL {spec.model_code} | "
        f"POSITION {label} | repeat {spec.repeat}/5 of this case")
    print(f"Record ID: {spec.rollout_id} (attempt {spec.attempt})")
    print("#" * 72)


def _preflight_pool() -> None:
    if _port_open():
        raise RunnerError(
            f"{ACT_HOST}:{ACT_PORT} is already in use. Shut down the existing ACT server "
            "or evaluation UI safely first.")
    active = _service_names()
    overlap = sorted(active.intersection(REQUIRED_SERVICES))
    if overlap:
        raise RunnerError(
            "An evaluation or robot graph is still running. Shut down the previous "
            f"evaluation UI safely: {overlap}")


def _preflight(spec: TrialSpec) -> None:
    if _attempt_path(spec.output_root, spec.scheduled_rollout_id, spec.attempt).exists():
        raise RunnerError(
            f"Refusing to overwrite an existing rollout: "
            f"{spec.output_root / spec.rollout_id}")
    for path, description in ((spec.model, "model"), (spec.dataset, "dataset")):
        if not path.exists():
            raise RunnerError(f"{description} path does not exist: {path}")
    if not _port_open():
        raise RunnerError("The resident ACT model pool is not responding.")
    active = _service_names()
    overlap = sorted(active.intersection(REQUIRED_SERVICES))
    if overlap:
        raise RunnerError(
            "An evaluation or robot graph is still running. Stop it with Ctrl+C in its "
            f"evaluation UI safely: {overlap}")


def _start_process(command: list[str], log_path: Path) -> tuple[subprocess.Popen, object]:
    stream = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        command, cwd=REPO_ROOT, stdout=stream, stderr=subprocess.STDOUT,
        text=True, start_new_session=True)
    return process, stream


def run_trial(spec: TrialSpec, *, show_camera: bool,
              graph_timeout: float) -> tuple[bool, bool]:
    """Run one trial; return (valid, requires_same_trial_retry)."""
    _print_trial(spec)
    _preflight(spec)
    _prompt_exact(
        "Workspace clear and E-stop within reach? Type READY: ", "READY")

    _select_act_model(spec.model_code)
    print(f"Resident ACT model selected: {spec.model_code}")

    runtime_dir = Path(tempfile.mkdtemp(prefix=f"grip_eval_{spec.rollout_id}_"))
    ros_log = runtime_dir / "ros.log"
    ros_process = None
    ros_stream = None
    logging_started = False
    result_name = None
    object_moved = False
    try:
        metadata_json = json.dumps(spec.metadata, separators=(",", ":"))
        print(f"Starting the robot, camera and evaluation graph... log: {ros_log}")
        ros_process, ros_stream = _start_process([
            "ros2", "launch", "grip_bringup", "eval.launch.py",
            f"rollout_id:={spec.rollout_id}",
            f"output_root:={spec.output_root}",
            f"show_camera:={'true' if show_camera else 'false'}",
            "require_complete_metadata:=true",
            f"meta_json:={metadata_json}",
        ], ros_log)
        _wait_until(
            lambda: set(REQUIRED_SERVICES).issubset(_service_names()),
            ros_process, graph_timeout, "the ROS evaluation services to come up", ros_log)
        print("Evaluation graph ready")

        _prompt_exact("PVC pipe removed? Type HOME (the robot will move): ", "HOME")
        _back_home()
        print("Move-to-HOME command sent")

        _call_trigger("/gripper/open")
        _prompt_exact("Gripper fully open? Confirm visually, then type OPEN: ", "OPEN")

        print("\nPhysical layout")
        print("G1 ─ G2 ─ G3\n│ Q1 │ Q2 │\nG4 ─ G5 ─ G6\n│ Q3 │ Q4 │\nG7 ─ G8 ─ G9")
        _print_placement_progress(spec)
        _prompt_exact(
            "Stand the PVC pipe upright, do not touch the board or camera, then type "
            "PLACED: ",
            "PLACED")

        _call_trigger("/grip_eval/log/start")
        logging_started = True
        print("Recording started. Do not touch the pipe or the board from now on.")
        _prompt_exact(
            "Type START to actually run the policy (keep the E-stop ready): ", "START")
        _call_trigger("/grip_eval/start")
        print(f"\nPolicy running. The time limit is {POLICY_TIMEOUT_S:.0f}s.")
        print(
            "If a collision looks likely, press F immediately - do not wait for contact. "
            "If it is urgent, hit the physical E-stop first.")
        print(
            "A knocked-over pipe, a close at the wrong place, a drop, or a lift or hold "
            "below threshold all count as failure (F).")
        result_name = _prompt_outcome(POLICY_TIMEOUT_S)

        _call_trigger("/grip_eval/stop")
        _call_trigger(OUTCOME_SERVICES[result_name])
        _call_trigger("/grip_eval/log/stop", timeout=300)
        logging_started = False
        _stop_rosbag(timeout=300)
        print(f"Outcome recorded: {result_name}")

        _prompt_exact(
            "Support the pipe by hand, then type RELEASE to open the gripper: ",
            "RELEASE")
        _call_trigger("/gripper/open")
        print(
            "Gripper open command sent. Clear the pipe from the workspace, then "
            "continue to the HOME step of the next trial.")
    except KeyboardInterrupt:
        print("\nInterrupted. Stopping the policy and preserving what has been recorded.")
        if ros_process is not None and ros_process.poll() is None:
            with suppress(Exception):
                _call_trigger("/grip_eval/stop", timeout=15)
        raise RunnerError("Interrupted by the operator.") from None
    finally:
        if logging_started:
            print("WARNING: the logger did not finalise. Keeping the directory for diagnosis.")
        _stop_owned_process(ros_process, "the ROS evaluation graph")
        if ros_stream is not None:
            ros_stream.close()

    rollout_dir = spec.output_root / spec.rollout_id
    result = validate(rollout_dir)
    print("\nValidator result:")
    print(json.dumps(result, indent=2, sort_keys=True))
    _print_grasp_position(result, spec)
    valid = bool(result["valid"])
    requires_retry = bool(object_moved)
    if requires_retry:
        print("This attempt is excluded from the main analysis; the same schedule trial will be rerun as a new attempt.")
    elif not valid:
        print("Validation failed, so we will not advance automatically to the next trial.")
    else:
        print("One valid evaluation trial is complete.")
    print(f"Runtime log: {runtime_dir}")
    return valid, requires_retry


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "One-terminal interactive runner for the frozen 480-rollout "
            "evaluation"))
    parser.add_argument(
        "--start-trial", type=int,
        help="trial to start from; omit to resume at the first incomplete trial")
    parser.add_argument(
        "--attempt", type=int,
        help="explicit retry number; normally omitted so it is chosen automatically")
    parser.add_argument("--schedule", type=Path, default=DEFAULT_SCHEDULE)
    parser.add_argument("--positions", type=Path, default=DEFAULT_POSITIONS)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument(
        "--pilot", action="store_true",
        help="verify the 30 Hz interpolating controller once, separately from the main evaluation")
    parser.add_argument("--no-camera", action="store_true")
    parser.add_argument("--dry-run", action="store_true",
                        help="print the next trial without starting the robot or ACT")
    parser.add_argument(
        "--act-timeout", type=float, default=900.0,
        help="seconds to wait for the 12-model resident ACT pool to become ready")
    parser.add_argument("--graph-timeout", type=float, default=180.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    schedule = _resolve_from_repo(args.schedule)
    positions = _resolve_from_repo(args.positions)
    output_root = (
        PILOT_OUTPUT_ROOT if args.pilot else _resolve_from_repo(args.output_root))

    if args.pilot:
        current_trial = args.start_trial or PILOT_TRIAL
        base = trial_spec(schedule, positions, current_trial)
        automatic_attempt = attempt_needed(output_root, base.scheduled_rollout_id)
        if automatic_attempt is None:
            print("The 30 Hz controller pilot is already complete.")
            return
        attempt = args.attempt or automatic_attempt
    elif args.start_trial is None:
        pending = first_pending_trial(schedule, positions, output_root)
        if pending is None:
            print("All 480 trials in the frozen schedule are complete.")
            return
        current_trial, attempt = pending
    else:
        current_trial = args.start_trial
        base = trial_spec(schedule, positions, current_trial)
        automatic_attempt = attempt_needed(output_root, base.scheduled_rollout_id)
        if args.attempt is not None:
            attempt = args.attempt
        elif automatic_attempt is None:
            raise SystemExit(
                f"ERROR: trial {current_trial} already has an eligible result. "
                "Refusing to run it again.")
        else:
            attempt = automatic_attempt

    def current_spec() -> TrialSpec:
        base = trial_spec(schedule, positions, current_trial, attempt)
        metadata = dict(base.metadata)
        if args.pilot:
            metadata["experiment_tag"] = "smooth30hz_controller_pilot"
        return replace(base, output_root=output_root, metadata=metadata)

    if args.dry_run:
        _print_trial(current_spec())
        print("DRY RUN: no process was started and the robot did not move.")
        return

    _print_trial(current_spec())
    print("Loading the 12 ACT models once, before preparing the trial above.")

    try:
        _preflight_pool()
    except (RunnerError, subprocess.TimeoutExpired) as exc:
        raise SystemExit(f"ERROR: {exc}") from None

    pool_runtime_dir = Path(tempfile.mkdtemp(prefix="grip_eval_act_pool_"))
    pool_log = pool_runtime_dir / "act_pool.log"
    pool_process = None
    pool_stream = None
    try:
        print("Loading the 12 ACT models onto the GPU in order. This wait happens only once.")
        print(f"ACT model pool log: {pool_log}")
        pool_process, pool_stream = _start_process([
            sys.executable, "-m", "ai.serve.zmq_act_pool_server",
            "--model-key", str(MODEL_KEY),
            "--repo-root", str(REPO_ROOT),
            "--device", "cuda",
        ], pool_log)
        _wait_for_act_pool(
            pool_process, args.act_timeout, pool_log)
        print("All 12 ACT models resident - later trials just switch between them.")

        while True:
            spec = current_spec()
            try:
                valid, requires_retry = run_trial(
                    spec, show_camera=not args.no_camera,
                    graph_timeout=args.graph_timeout)
            except (RunnerError, subprocess.TimeoutExpired) as exc:
                raise SystemExit(f"ERROR: {exc}") from None
            if not valid:
                pending = first_pending_trial(schedule, positions, output_root)
                if pending is None:
                    raise RunnerError(
                        "Could not find a schedule row to retry after the validator failed.")
                current_trial, attempt = pending
                answer = input(
                    "The failed attempt is preserved. Enter = retry with the same resident "
                    "model pool, Q = quit safely: ").strip().casefold()
                if answer == "q":
                    print("Stopped. The next run will retry this trial automatically.")
                    return
                continue
            if args.pilot:
                print("One 30 Hz controller pilot run is complete. Check the motion visually.")
                return
            pending = first_pending_trial(schedule, positions, output_root)
            if pending is None:
                print("All 480 evaluation trials are complete.")
                return
            current_trial, attempt = pending
            if requires_retry:
                print(f"Preparing the same schedule trial again as attempt {attempt}.")
            answer = input(
                "Enter = continue to the next trial in this terminal (from HOME and OPEN), "
                "Q = quit safely: ").strip().casefold()
            if answer == "q":
                print("Stopped. The next run will resume from here automatically.")
                return
    except (RunnerError, subprocess.TimeoutExpired) as exc:
        raise SystemExit(f"ERROR: {exc}") from None
    finally:
        _stop_owned_process(pool_process, "the ACT model pool")
        if pool_stream is not None:
            pool_stream.close()
        print(f"ACT model pool runtime log: {pool_runtime_dir}")


if __name__ == "__main__":
    main()
