#!/usr/bin/env python3
"""Run one operator-selected ACT model without recording evaluation data."""

from __future__ import annotations

import argparse
from contextlib import suppress
import json
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import threading
import time


REPO_ROOT = Path(__file__).resolve().parents[2]
MODEL_KEY = REPO_ROOT / "experiment/evaluation/main_recollection_20260817/model_key.json"
ACT_HOST = "127.0.0.1"
ACT_PORT = 5557
POLICY_TIMEOUT_S = 60.0
POSITION_NAMES = tuple(
    [f"G{index}" for index in range(1, 10)]
    + [f"Q{index}" for index in range(1, 5)]
)
REQUIRED_SERVICES = {
    "/indy_srv", "/gripper/open", "/grip_eval/start", "/grip_eval/stop",
}
CONFLICTING_SERVICES = REQUIRED_SERVICES | {
    "/grip_eval/log/start", "/grip_eval/log/stop",
    "/grip_eval/result/success",
}


class DirectRunError(RuntimeError):
    pass


def model_registry(model_key: Path = MODEL_KEY) -> dict[str, dict[str, object]]:
    """Resolve selectable model codes to their single-model sidecar inputs."""
    identities = json.loads(model_key.read_text(encoding="utf-8"))
    registry = {}
    for model_code, identity in sorted(identities.items()):
        condition = str(identity["condition"]).lower()
        seed = int(identity["seed"])
        registry[model_code] = {
            "condition": condition.upper(),
            "seed": seed,
            "model": REPO_ROOT / (
                f"ai/models/main_recollection_20260817_{condition}_s{seed}_100000/"
                "checkpoints/last/pretrained_model"
            ),
            "dataset_root": REPO_ROOT / f"datasets/main_recollection_20260817_rgb_{condition}",
            "dataset_repo_id": f"sirlab/grip_recollection_20260817_{condition}",
        }
    return registry


def _port_open() -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(0.25)
        return sock.connect_ex((ACT_HOST, ACT_PORT)) == 0


def _service_names() -> set[str]:
    try:
        result = subprocess.run(
            ["ros2", "service", "list", "--no-daemon", "--spin-time", "1.0"],
            cwd=REPO_ROOT, text=True, capture_output=True, timeout=10, check=False)
    except subprocess.TimeoutExpired as exc:
        raise DirectRunError("The ROS 2 service query did not finish within 10s.") from exc
    if result.returncode != 0:
        detail = (result.stdout + result.stderr).strip()
        raise DirectRunError(f"ROS 2 service query failed: {detail}")
    return set(result.stdout.splitlines())


def _call_trigger(service: str, timeout: float = 180.0) -> None:
    result = subprocess.run(
        ["ros2", "service", "call", service, "std_srvs/srv/Trigger", "{}"],
        cwd=REPO_ROOT, text=True, capture_output=True, timeout=timeout, check=False)
    output = (result.stdout + result.stderr).strip()
    if output:
        print(output, flush=True)
    if result.returncode != 0 or not re.search(
            r"success\s*[=:]\s*(?:True|true)", output):
        raise DirectRunError(f"Service call failed: {service}")


def _back_home() -> None:
    result = subprocess.run(
        [str(REPO_ROOT / "scripts/robot/back_home.sh")], cwd=REPO_ROOT,
        text=True, capture_output=True, timeout=180, check=False)
    output = (result.stdout + result.stderr).strip()
    if output:
        print(output, flush=True)
    if result.returncode != 0 or not re.search(
            r"success\s*[=:]\s*(?:True|true)", output):
        raise DirectRunError("The move-to-HOME command failed.")


def _relay(stream, prefix: str) -> None:
    try:
        for line in iter(stream.readline, ""):
            print(f"[{prefix}] {line}", end="", flush=True)
    finally:
        stream.close()


def _start_process(command: list[str], name: str) -> subprocess.Popen:
    process = subprocess.Popen(
        command, cwd=REPO_ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1, start_new_session=True)
    assert process.stdout is not None
    threading.Thread(target=_relay, args=(process.stdout, name), daemon=True).start()
    return process


def _stop_process(process: subprocess.Popen | None, name: str) -> None:
    if process is None or process.poll() is not None:
        return
    with suppress(ProcessLookupError):
        os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=30)
    except subprocess.TimeoutExpired:
        print(f"WARNING: {name} is slow to exit; sending SIGTERM.")
        with suppress(ProcessLookupError):
            os.killpg(process.pid, signal.SIGTERM)
        with suppress(subprocess.TimeoutExpired):
            process.wait(timeout=10)


def _wait_until(predicate, process: subprocess.Popen, timeout: float, description: str) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise DirectRunError(f"The process exited before {description}.")
        if predicate():
            return
        time.sleep(0.5)
    raise DirectRunError(f"Timed out after {timeout:.0f}s waiting for {description}.")


def _prompt(name: str, message: str, expected: str) -> None:
    print(f"DIRECT_RUN_PROMPT:{name}", flush=True)
    answer = input(message).strip().casefold()
    if answer != expected.casefold():
        raise DirectRunError(f"Aborting: '{expected}' was not entered.")


def _wait_for_stop(timeout_s: float) -> None:
    print("DIRECT_RUN_PROMPT:stop", flush=True)
    print(
        f"Policy running - type STOP to stop it; automatic stop after {timeout_s:.0f}s: ",
        end="", flush=True)
    deadline = time.monotonic() + timeout_s
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            print("\nReached the 60s limit; stopping the policy automatically.", flush=True)
            return
        import select
        readable, _, _ = select.select([sys.stdin], [], [], remaining)
        if not readable:
            print("\nReached the 60s limit; stopping the policy automatically.", flush=True)
            return
        answer = sys.stdin.readline()
        if answer == "":
            raise DirectRunError("The operator input stream was closed.")
        if answer.strip().casefold() == "stop":
            return
        print("Type STOP: ", end="", flush=True)


def _preflight(entry: dict[str, object]) -> None:
    for key in ("model", "dataset_root"):
        path = Path(entry[key])
        if not path.is_dir():
            raise DirectRunError(f"{key} path does not exist: {path}")
    if _port_open():
        raise DirectRunError(
            f"{ACT_HOST}:{ACT_PORT} is already in use. Stop the existing ACT process first.")
    overlap = sorted(CONFLICTING_SERVICES.intersection(_service_names()))
    if overlap:
        raise DirectRunError(
            "A robot or evaluation graph is already running. Shut it down safely first: "
            + ", ".join(overlap))


def run(model_code: str, position: str, *, show_camera: bool,
        act_timeout: float, graph_timeout: float) -> None:
    registry = model_registry()
    if model_code not in registry:
        raise DirectRunError(f"Unknown model: {model_code}")
    position = position.upper()
    if position not in POSITION_NAMES:
        raise DirectRunError(f"Unknown position: {position}")
    entry = registry[model_code]
    _preflight(entry)

    print("=" * 68)
    print("Direct run (no evaluation result and no rosbag are saved)")
    print(
        f"MODEL {model_code} | CONDITION {entry['condition']} | SEED {entry['seed']} | "
        f"POSITION {position}")
    print("=" * 68)
    _prompt("ready", "Workspace checked and E-stop within reach? Type READY: ", "READY")

    act_process = None
    graph_process = None
    policy_running = False
    try:
        print(f"Loading only the selected model {model_code} onto the GPU.", flush=True)
        act_process = _start_process([
            sys.executable, "-m", "ai.serve.zmq_act_server",
            "--model", str(entry["model"]),
            "--dataset-root", str(entry["dataset_root"]),
            "--dataset-repo-id", str(entry["dataset_repo_id"]),
            "--device", "cuda",
        ], "ACT")
        _wait_until(_port_open, act_process, act_timeout, "the ACT model to be ready")
        print("Selected model ready", flush=True)

        graph_process = _start_process([
            "ros2", "launch", "grip_bringup", "eval.launch.py",
            "record:=false", "rollout_id:=not_recorded",
            f"show_camera:={'true' if show_camera else 'false'}",
        ], "ROS")
        _wait_until(
            lambda: REQUIRED_SERVICES.issubset(_service_names()),
            graph_process, graph_timeout, "the ROS services to come up")
        print("Robot graph ready - the logger and rosbag were not started.", flush=True)

        _prompt("home", "PVC pipe removed? Type HOME (the robot will move): ", "HOME")
        _back_home()
        _call_trigger("/gripper/open")
        _prompt("open", "Gripper fully open? Confirm visually, then type OPEN: ", "OPEN")
        print(f"Stand the PVC pipe upright at {position}.", flush=True)
        _prompt("placed", f"Pipe placed at {position}? Type PLACED: ", "PLACED")
        _prompt("start", "With the E-stop ready, type START to run the policy: ", "START")
        _call_trigger("/grip_eval/start")
        policy_running = True
        _wait_for_stop(POLICY_TIMEOUT_S)
        _call_trigger("/grip_eval/stop")
        policy_running = False
        print("Policy stopped. No evaluation result was saved.", flush=True)
        _prompt("release", "Support the pipe, then type RELEASE to open the gripper: ", "RELEASE")
        _call_trigger("/gripper/open")
        print("DIRECT_RUN_DONE", flush=True)
        print("Run complete - no evaluation result or rosbag was written.", flush=True)
    except KeyboardInterrupt:
        raise DirectRunError("The operator interrupted the run.") from None
    finally:
        if policy_running and graph_process is not None and graph_process.poll() is None:
            with suppress(Exception):
                _call_trigger("/grip_eval/stop", timeout=15)
        _stop_process(graph_process, "ROS graph")
        _stop_process(act_process, "ACT sidecar")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run one selected ACT model and placement without recording")
    parser.add_argument("--model", required=True)
    parser.add_argument("--position", required=True)
    parser.add_argument("--no-camera", action="store_true")
    parser.add_argument("--act-timeout", type=float, default=300.0)
    parser.add_argument("--graph-timeout", type=float, default=180.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    try:
        run(
            args.model.upper(), args.position.upper(),
            show_camera=not args.no_camera,
            act_timeout=args.act_timeout, graph_timeout=args.graph_timeout)
    except KeyboardInterrupt:
        raise SystemExit("ERROR: the operator interrupted the run.") from None
    except (DirectRunError, subprocess.TimeoutExpired) as exc:
        raise SystemExit(f"ERROR: {exc}") from None


if __name__ == "__main__":
    main()
