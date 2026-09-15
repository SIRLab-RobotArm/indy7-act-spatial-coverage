#!/usr/bin/env python3
"""20 Hz raw rollout logger; metrics are deliberately computed offline."""

from __future__ import annotations

import argparse
from contextlib import suppress
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import message_filters
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from indy_interfaces.msg import EefPose


def discover_repo_root() -> Path:
    """Find the repository root without hardcoding anyone's directory layout."""
    override = os.environ.get("GRIP_REPO_ROOT")
    if override:
        return Path(override).resolve()
    for parent in Path(__file__).resolve().parents:
        if (parent / "ai" / "eval").is_dir() and (parent / "experiment").is_dir():
            return parent
    return Path.cwd().resolve()


REPO_ROOT = discover_repo_root()
# Interpreter used to turn the JSONL trace into Parquet. It needs pandas and
# pyarrow, which live in the conda environment rather than in the ROS Python.
# Override with GRIP_PARQUET_PYTHON or --parquet-python.
DEFAULT_PYTHON = os.environ.get("GRIP_PARQUET_PYTHON", sys.executable)
FAILURE_CODES = (
    "operator_failure", "missed_grasp", "slip_drop", "no_lift",
    "collision_stop", "hardware_fault")
REQUIRED_MAIN_METADATA = (
    "condition", "seed", "model_code", "session", "repeat", "attempt",
    "scheduled_rollout_id",
    "position_id", "p_target", "d", "success",
    "object_move", "lift_threshold_mm", "hold_duration_s", "experiment_tag")
SMOOTH_EXECUTION_TAG = "main_recollection_20260817_main_smooth30hz"
SMOOTH_EXECUTION_METADATA = (
    "execution_controller", "policy_action_hz", "robot_command_hz",
    "interpolation_steps")


class RolloutLoggerNode(Node):
    def __init__(self, output_root, rollout_id, metadata, *, mock=False,
                 parquet_python=DEFAULT_PYTHON, require_complete_metadata=False):
        super().__init__("grip_rollout_logger")
        self.output_root = Path(output_root).resolve()
        self.rollout_id = rollout_id
        self.metadata = {
            "condition": None, "seed": None, "position_id": None,
            "p_target": None, "d": None, "success": None,
            "failure_code": None, "object_move": False, "video_path": None,
            "experiment_tag": None, **dict(metadata),
        }
        self.mock = bool(mock)
        self.require_complete_metadata = bool(require_complete_metadata)
        self.parquet_python = parquet_python
        self.directory = self.output_root / rollout_id
        self.directory.mkdir(parents=True, exist_ok=False)
        self.stream = None
        self.events = []
        self.summary = {"t_start": None, "t_close": None, "t_move": None, "t_end": None}
        self.gripper_cmd = 0
        self.autonomy_state = "idle"
        self.last_driver_status = None
        self.start_service = self.create_service(Trigger, "/grip_eval/log/start", self._start)
        self.stop_service = self.create_service(Trigger, "/grip_eval/log/stop", self._stop)
        self.object_move_service = self.create_service(
            Trigger, "/grip_eval/object_move", self._mark_object_move)
        self.success_service = self.create_service(
            Trigger, "/grip_eval/result/success", self._mark_success)
        self.failure_services = [
            self.create_service(
                Trigger, f"/grip_eval/result/failure/{code}",
                self._failure_callback(code))
            for code in FAILURE_CODES
        ]
        self.create_subscription(String, "/grip_eval/event", self._event, 50)
        self.create_subscription(String, "/grip_eval/rollout_meta", self._meta_update, 10)
        self.create_subscription(String, "/indy/teleop_status", self._driver_status, 20)
        if not mock:
            self.ee_sub = message_filters.Subscriber(self, EefPose, "/indy/ee_pose_stamped")
            self.joint_sub = message_filters.Subscriber(self, JointState, "/joint_states")
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [self.ee_sub, self.joint_sub], queue_size=20, slop=0.03)
            self.sync.registerCallback(self._sample)
        self.get_logger().info(f"rollout logger ready: {self.directory} (mock={mock})")

    def _start(self, _request, response):
        if self.stream is not None:
            response.success = False
            response.message = "already logging"
            return response
        self.stream = (self.directory / ".trace.jsonl.partial").open("w", encoding="utf-8")
        now = time.time()
        self.summary["t_start"] = now
        self.events.append({"t": now, "event": "logger_start"})
        if self.mock:
            for index in range(20):
                self._write_sample(
                    now + index / 20.0,
                    [float(index), 0.0, 100.0, 0.0, 0.0, 0.0],
                    [0.0] * 6)
        response.success = True
        response.message = str(self.directory)
        return response

    def _stop(self, _request, response):
        if self.stream is None:
            response.success = False
            response.message = "not logging"
            return response
        if self.require_complete_metadata:
            problems = self._metadata_problems()
            if problems:
                response.success = False
                response.message = "metadata incomplete: " + "; ".join(problems)
                return response
        self._finalize()
        response.success = True
        response.message = str(self.directory / "trace.parquet")
        return response

    def _record_operator_event(self, event, **values):
        now = time.time()
        self.events.append({"t": now, "event": event, **values})
        return now

    def _mark_object_move(self, _request, response):
        if self.stream is None:
            response.success = False
            response.message = "logger is not running"
            return response
        if self.summary["t_move"] is None:
            now = self._record_operator_event("object_move", source="operator")
            self.summary["t_move"] = now
        self.metadata["object_move"] = True
        response.success = True
        response.message = "object_move recorded"
        return response

    def _mark_success(self, _request, response):
        return self._mark_result(True, None, response)

    def _failure_callback(self, code):
        def callback(_request, response):
            return self._mark_result(False, code, response)
        return callback

    def _mark_result(self, success, failure_code, response):
        if self.stream is None:
            response.success = False
            response.message = "logger is not running"
            return response
        self.metadata["success"] = bool(success)
        self.metadata["failure_code"] = failure_code
        self._record_operator_event(
            "operator_result", success=bool(success), failure_code=failure_code)
        response.success = True
        response.message = "success" if success else f"failure: {failure_code}"
        return response

    def _metadata_problems(self):
        problems = []
        for key in REQUIRED_MAIN_METADATA:
            if self.metadata.get(key) is None:
                problems.append(f"{key}=null")
        if self.metadata.get("experiment_tag") == SMOOTH_EXECUTION_TAG:
            for key in SMOOTH_EXECUTION_METADATA:
                if self.metadata.get(key) is None:
                    problems.append(f"{key}=null")
        target = self.metadata.get("p_target")
        if target is not None and (
                not isinstance(target, list) or len(target) != 3):
            problems.append("p_target must contain XYZ")
        distance = self.metadata.get("d")
        if distance is not None:
            try:
                if float(distance) < 0:
                    problems.append("d must be non-negative")
            except (TypeError, ValueError):
                problems.append("d must be numeric")
        success = self.metadata.get("success")
        failure_code = self.metadata.get("failure_code")
        if success is False and failure_code not in FAILURE_CODES:
            problems.append("failed rollout requires a frozen failure_code")
        if success is True and failure_code is not None:
            problems.append("successful rollout cannot have failure_code")
        return problems

    def _sample(self, ee, joint):
        if self.stream is None or len(joint.position) < 6:
            return
        stamp = float(ee.header.stamp.sec) + float(ee.header.stamp.nanosec) * 1e-9
        self._write_sample(stamp, list(ee.pose), list(joint.position[:6]))

    def _write_sample(self, stamp, ee_pose, joint_pos):
        row = {
            "t": float(stamp),
            "ee_pose": [float(x) for x in ee_pose],
            "joint_pos": [float(x) for x in joint_pos],
            "gripper_cmd": int(self.gripper_cmd),
            "autonomy_state": self.autonomy_state,
        }
        self.stream.write(json.dumps(row, separators=(",", ":")) + "\n")
        self.stream.flush()

    def _event(self, message):
        try:
            event = json.loads(message.data)
        except json.JSONDecodeError:
            return
        self.events.append(event)
        name = event.get("event")
        self.autonomy_state = str(name or self.autonomy_state)
        if name == "gripper":
            self.gripper_cmd = int(event.get("command") == "close")
            if self.gripper_cmd and self.summary["t_close"] is None:
                self.summary["t_close"] = event.get("t")
        elif name == "object_move" and self.summary["t_move"] is None:
            self.summary["t_move"] = event.get("t")
            self.metadata["object_move"] = True

    def _driver_status(self, message):
        if self.stream is None:
            return
        try:
            status = json.loads(message.data)
        except json.JSONDecodeError:
            return
        reduced = {
            key: status.get(key) for key in (
                "mode", "op_state", "fault_latch", "fault_reason", "guard_count")
        }
        if reduced != self.last_driver_status:
            self.events.append({
                "t": time.time(), "event": "driver_status", "status": reduced})
            self.last_driver_status = reduced

    def _meta_update(self, message):
        if self.stream is None:
            return
        try:
            update = json.loads(message.data)
        except json.JSONDecodeError:
            return
        allowed = set(self.metadata) - {"rollout_id"}
        self.metadata.update({key: value for key, value in update.items() if key in allowed})
        self.events.append({"t": time.time(), "event": "meta_update", "keys": sorted(update)})

    def _finalize(self):
        now = time.time()
        self.summary["t_end"] = now
        self.events.append({"t": now, "event": "logger_stop"})
        self.stream.close()
        self.stream = None
        partial = self.directory / ".trace.jsonl.partial"
        subprocess.run([
            self.parquet_python, str(REPO_ROOT / "ai/eval/write_trace.py"),
            str(partial), str(self.directory / "trace.parquet")
        ], check=True, cwd=REPO_ROOT)
        partial.unlink()
        event_document = {**self.summary, "events": self.events}
        (self.directory / "events.json").write_text(
            json.dumps(event_document, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        meta = {
            "rollout_id": self.rollout_id,
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "mock": self.mock,
            **self.metadata,
        }
        (self.directory / "meta.json").write_text(
            json.dumps(meta, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        if self.mock:
            bag = self.directory / "rollout.bag"
            bag.mkdir()
            (bag / "MOCK_ONLY.txt").write_text("No ROS transport in mock mode.\n")

    def destroy_node(self):
        if self.stream is not None:
            self._finalize()
        return super().destroy_node()


def _parse_args(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", default=str(REPO_ROOT / "experiment/rollouts"))
    parser.add_argument("--rollout-id", required=True)
    parser.add_argument("--meta-json", default="{}")
    parser.add_argument("--mock", action="store_true")
    parser.add_argument("--mock-once", action="store_true")
    parser.add_argument("--parquet-python", default=DEFAULT_PYTHON)
    parser.add_argument(
        "--require-complete-metadata", choices=("true", "false"), default="false")
    return parser.parse_known_args(argv)


def main(args=None):
    parsed, ros_args = _parse_args(sys.argv[1:] if args is None else args)
    metadata = json.loads(parsed.meta_json)
    rclpy.init(args=ros_args)
    node = RolloutLoggerNode(
        parsed.output_root, parsed.rollout_id, metadata,
        mock=parsed.mock, parquet_python=parsed.parquet_python,
        require_complete_metadata=parsed.require_complete_metadata == "true")
    if parsed.mock_once:
        response = Trigger.Response()
        node._start(None, response)
        node._stop(None, response)
        node.destroy_node()
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        with suppress(KeyboardInterrupt):
            node.destroy_node()
        if rclpy.ok():
            with suppress(KeyboardInterrupt):
                rclpy.shutdown()


if __name__ == "__main__":
    main()
