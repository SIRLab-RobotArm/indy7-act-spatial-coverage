#!/usr/bin/env python3
"""Preregistered stop-and-go ACT executor with a hardware-free replay path."""

from __future__ import annotations

import argparse
from contextlib import suppress
import json
import os
from pathlib import Path
import sys
import time

import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

from indy_interfaces.msg import EefPose
from indy_interfaces.srv import IndyService

from grip_eval.act_client import DryRunActClient, ZmqActClient


def discover_repo_root() -> Path:
    """Find the repository root so `ai.eval` is importable from a ROS node.

    GRIP_REPO_ROOT wins when set (scripts/env.sh exports it); otherwise walk up
    from this file, which resolves into the source tree under --symlink-install.
    """
    override = os.environ.get("GRIP_REPO_ROOT")
    if override:
        return Path(override).resolve()
    for parent in Path(__file__).resolve().parents:
        if (parent / "ai" / "eval").is_dir() and (parent / "experiment").is_dir():
            return parent
    return Path.cwd().resolve()


REPO_ROOT = discover_repo_root()
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
from ai.eval.action_contract import (  # noqa: E402
    GripperLatch, accumulate_pose_target, executor_command,
)


ACTION_HZ = 5.0
COMMAND_HZ = 30.0
INTERPOLATION_STEPS = int(COMMAND_HZ / ACTION_HZ)
CHUNK_SIZE = 40
EXECUTE_STEPS = 10
SETTLE_SEC = 0.3
SYNC_SLOP_SEC = 0.03
OP_IDLE = 5
MSG_TELE_TASK_RLT = 5
MSG_TELE_STOP = 8


def image_to_numpy(msg):
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    image = rows[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    if msg.encoding == "bgr8":
        image = image[..., ::-1]
    elif msg.encoding != "rgb8":
        raise ValueError(f"unsupported RGB encoding {msg.encoding}")
    return np.ascontiguousarray(image)


def dry_run_replay(episode_h5, action_hz=5):
    from grip_eval.replay_node import actions_from_hdf5

    actions = actions_from_hdf5(episode_h5, action_hz)
    latch = GripperLatch(initial=int(actions[0, 3]) if len(actions) else 0)
    commands = []
    pose_target = np.zeros(6, dtype=np.float32)
    for action in actions:
        pose_delta, prediction = executor_command(action)
        pose_target = accumulate_pose_target(pose_target, pose_delta)
        grip, transition = latch.update(prediction)
        commands.append({
            "pose_delta": pose_delta.tolist(),
            "pose_target": pose_target.tolist(), "gripper_cmd": grip,
            "gripper_transition": transition,
        })
    return actions, commands


def interpolated_targets(start, end, steps=INTERPOLATION_STEPS):
    """Return cumulative linear targets ending exactly at one 5 Hz action."""
    start = np.asarray(start, dtype=np.float32).reshape(6)
    end = np.asarray(end, dtype=np.float32).reshape(6)
    steps = int(steps)
    if steps < 1:
        raise ValueError("interpolation steps must be positive")
    fractions = (
        np.arange(1, steps + 1, dtype=np.float32) / np.float32(steps)
    ).reshape(-1, 1)
    targets = start.reshape(1, 6) + fractions * (end - start).reshape(1, 6)
    targets[-1] = end
    return targets


class PolicyExecutorNode(Node):
    def __init__(self, *, mock=False, endpoint="tcp://127.0.0.1:5557",
                 workspace_min=(-250.0, -250.0, 15.0),
                 workspace_max=(250.0, 250.0, 250.0),
                 enforce_workspace=True):
        super().__init__("grip_policy_executor")
        self.mock = bool(mock)
        self.workspace_min = tuple(float(x) for x in workspace_min)
        self.workspace_max = tuple(float(x) for x in workspace_max)
        self.enforce_workspace = bool(enforce_workspace)
        self.pose_pub = self.create_publisher(Float64MultiArray, "/indy/teleop_pose", 10)
        self.event_pub = self.create_publisher(String, "/grip_eval/event", 20)
        self.start_srv = self.create_service(Trigger, "/grip_eval/start", self._start)
        self.stop_srv = self.create_service(Trigger, "/grip_eval/stop", self._stop)
        self.close_client = self.create_client(Trigger, "/gripper/grasp")
        self.open_client = self.create_client(Trigger, "/gripper/open")
        self.indy_client = self.create_client(IndyService, "/indy_srv")
        self.create_subscription(String, "/indy/teleop_status", self._status, 10)
        self.observation = None
        self.driver_status = {"op_state": OP_IDLE, "fault_latch": False} if mock else None
        self.state = "idle"
        self.settle_until = 0.0
        self.service_future = None
        self.chunk = None
        self.chunk_index = 0
        self.pose_target = np.zeros(6, dtype=np.float32)
        self.command_targets = None
        self.command_index = 0
        self.latch = GripperLatch(0)
        if mock:
            self.observation = {
                "state": np.zeros(10, dtype=np.float32),
                "rgb": np.zeros((480, 640, 3), dtype=np.uint8),
                "ee_pose": np.array([0.0, 0.0, 100.0, 0.0, 0.0, 0.0]),
            }
            self.client = DryRunActClient([np.zeros((CHUNK_SIZE, 4), dtype=np.float32)] * 100)
        else:
            self.client = ZmqActClient(endpoint)
            self._setup_sync()
        self.create_timer(1.0 / COMMAND_HZ, self._tick)
        self._emit(
            "executor_ready", mock=self.mock, observation_mode="rgb_only",
            workspace_guard=self.enforce_workspace,
            policy_hz=ACTION_HZ, command_hz=COMMAND_HZ,
            interpolation="linear_cumulative_target",
        )

    def _setup_sync(self):
        self.rgb_sub = message_filters.Subscriber(
            self, Image, "/overhead_camera/camera/color/image_raw")
        self.joint_sub = message_filters.Subscriber(self, JointState, "/joint_states")
        self.ee_sub = message_filters.Subscriber(self, EefPose, "/indy/ee_pose_stamped")
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.joint_sub, self.ee_sub],
            queue_size=20, slop=SYNC_SLOP_SEC)
        self.sync.registerCallback(self._observation)

    def _observation(self, rgb, joint, ee):
        if len(joint.position) < 6:
            return
        pose = np.asarray(ee.pose, dtype=np.float32)
        state = np.concatenate((
            pose[:3], np.asarray(joint.position[:6], dtype=np.float32), [self.latch.state]
        )).astype(np.float32)
        self.observation = {
            "state": state,
            "rgb": image_to_numpy(rgb),
            "ee_pose": pose,
        }

    def _status(self, msg):
        try:
            self.driver_status = json.loads(msg.data)
        except json.JSONDecodeError:
            self._abort("invalid_driver_status")

    def _emit(self, name, **fields):
        msg = String()
        msg.data = json.dumps({"t": time.time(), "event": name, **fields}, sort_keys=True)
        self.event_pub.publish(msg)

    def _call_indy(self, mode):
        if self.mock:
            self.driver_status["op_state"] = OP_IDLE if mode == MSG_TELE_STOP else 17
            self.service_future = None
            return
        request = IndyService.Request()
        request.data = int(mode)
        self.service_future = self.indy_client.call_async(request)

    def _start(self, _request, response):
        if self.state != "idle":
            response.success = False
            response.message = f"executor is {self.state}"
            return response
        if self.observation is None or self.driver_status is None:
            response.success = False
            response.message = "waiting for synchronized observation and driver status"
            return response
        if self.driver_status.get("fault_latch"):
            reason = self.driver_status.get("fault_reason") or "unknown"
            response.success = False
            response.message = f"driver fault latched: {reason}; recover required"
            return response
        self.client.reset()
        self.latch = GripperLatch(int(self.observation["state"][-1]))
        self.pose_target = np.zeros(6, dtype=np.float32)
        self.command_targets = None
        self.command_index = 0
        self._call_indy(MSG_TELE_STOP)
        self.state = "stopping"
        self._emit("trial_start")
        response.success = True
        response.message = "trial started"
        return response

    def _stop(self, _request, response):
        self._abort("operator_stop")
        response.success = True
        response.message = "stop requested"
        return response

    def _abort(self, reason):
        if self.state != "idle":
            self._call_indy(MSG_TELE_STOP)
            self._emit("trial_end", reason=reason)
        self.state = "idle"

    def _service_ok(self):
        if self.service_future is None:
            return True
        if not self.service_future.done():
            return False
        result = self.service_future.result()
        self.service_future = None
        if result is None or not result.success:
            self._abort("indy_service_failed")
            return False
        return True

    def _driver_idle(self):
        return self.driver_status is not None and int(self.driver_status.get("op_state", -1)) == OP_IDLE

    def _predict_chunk(self):
        snapshot = self.observation
        try:
            chunk = self.client.predict_chunk(
                state=snapshot["state"], rgb=snapshot["rgb"])
        except Exception as exc:
            self.get_logger().error(f"ACT inference failed: {exc}")
            self._abort("inference_error")
            return False
        if chunk.shape[0] != CHUNK_SIZE:
            self._abort("invalid_chunk_length")
            return False
        self.chunk = chunk
        self.chunk_index = 0
        return True

    def _publish_pose_target(self):
        msg = Float64MultiArray()
        msg.data = self.pose_target.astype(float).tolist()
        self.pose_pub.publish(msg)

    def _prepare_action_segment(self):
        """Prepare six 30 Hz targets for one unchanged 5 Hz ACT endpoint."""
        action = self.chunk[self.chunk_index]
        if self.enforce_workspace:
            pose, prediction = executor_command(
                action, self.observation["ee_pose"][:3],
                self.workspace_min, self.workspace_max,
            )
        else:
            pose, prediction = executor_command(action)
        end_target = accumulate_pose_target(self.pose_target, pose)
        self.command_targets = interpolated_targets(
            self.pose_target, end_target, INTERPOLATION_STEPS)
        self.command_index = 0
        grip, transition = self.latch.update(prediction)
        self._emit(
            "action", index=self.chunk_index, action=action.tolist(),
            pose_delta=pose.astype(float).tolist(),
            pose_target=end_target.astype(float).tolist(),
            gripper_cmd=grip, command_hz=COMMAND_HZ,
            interpolation_steps=INTERPOLATION_STEPS)
        if transition:
            client = self.close_client if transition == "close" else self.open_client
            if not self.mock:
                client.call_async(Trigger.Request())
            self._emit("gripper", command=transition)
        self.chunk_index += 1

    def _tick(self):
        if self.state == "idle":
            return
        if self.driver_status and self.driver_status.get("fault_latch"):
            self._abort("driver_fault")
            return
        if self.state == "stopping":
            if self._service_ok() and self._driver_idle():
                self.settle_until = time.monotonic() + SETTLE_SEC
                self.state = "settling"
                self._emit("robot_stopped")
            return
        if self.state == "settling":
            if time.monotonic() < self.settle_until:
                return
            if not self._predict_chunk():
                return
            self._call_indy(MSG_TELE_TASK_RLT)
            self.state = "starting"
            self._emit("policy_forward", continuous=True)
            return
        if self.state == "starting":
            if self._service_ok():
                self.state = "executing"
            return
        if self.state == "executing":
            if self.command_targets is None:
                self._prepare_action_segment()
            self.pose_target = self.command_targets[self.command_index].copy()
            self._publish_pose_target()
            self.command_index += 1
            if self.command_index < INTERPOLATION_STEPS:
                return
            self.command_targets = None
            self.command_index = 0
            if self.chunk_index >= EXECUTE_STEPS:
                # Keep task-relative teleop active.  Re-send the current
                # cumulative target before the blocking inference call so the
                # driver sees a fresh heartbeat without an intentional STOP.
                self._publish_pose_target()
                if self._predict_chunk():
                    self._emit("policy_forward", continuous=True)

    def destroy_node(self):
        self.client.close()
        return super().destroy_node()


def _parse_args(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--mock", action="store_true")
    parser.add_argument("--dry-run-h5")
    parser.add_argument("--dry-run-output")
    parser.add_argument("--action-rate", type=int, default=5)
    parser.add_argument("--endpoint", default="tcp://127.0.0.1:5557")
    parser.add_argument("--workspace-min", nargs=3, type=float, default=(-250.0, -250.0, 15.0))
    parser.add_argument("--workspace-max", nargs=3, type=float, default=(250.0, 250.0, 250.0))
    parser.add_argument("--no-workspace", action="store_true")
    return parser.parse_known_args(argv)


def main(args=None):
    parsed, ros_args = _parse_args(sys.argv[1:] if args is None else args)
    if parsed.dry_run_h5:
        actions, commands = dry_run_replay(parsed.dry_run_h5, parsed.action_rate)
        result = {"recorded_actions": actions.tolist(), "executor_commands": commands}
        text = json.dumps(result, indent=2) + "\n"
        if parsed.dry_run_output:
            Path(parsed.dry_run_output).write_text(text, encoding="utf-8")
        else:
            print(text, end="")
        return
    rclpy.init(args=ros_args)
    node = PolicyExecutorNode(
        mock=parsed.mock, endpoint=parsed.endpoint,
        workspace_min=parsed.workspace_min, workspace_max=parsed.workspace_max,
        enforce_workspace=not parsed.no_workspace,
    )
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
