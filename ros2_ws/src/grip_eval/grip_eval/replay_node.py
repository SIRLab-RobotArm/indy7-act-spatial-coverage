#!/usr/bin/env python3
"""Replay raw demonstrations as the exact 5 Hz delta-action contract for G5."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import h5py
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


RECORD_HZ = 20
DEFAULT_ACTION_HZ = 5


def actions_from_hdf5(path, action_hz=DEFAULT_ACTION_HZ, phase=0):
    if RECORD_HZ % int(action_hz):
        raise ValueError("action_hz must divide the 20 Hz recording rate")
    stride = RECORD_HZ // int(action_hz)
    with h5py.File(path, "r") as episode:
        if "action" in episode or "/action" in episode:
            raise ValueError("raw HDF5 must not contain action")
        ee = np.asarray(episode["/state/ee_pose"], dtype=np.float32)
        grip = np.asarray(episode["/state/gripper_cmd"], dtype=np.float32)
    indices = np.arange(int(phase), len(ee) - stride, stride)
    actions = np.empty((len(indices), 4), dtype=np.float32)
    actions[:, :3] = ee[indices + stride, :3] - ee[indices, :3]
    actions[:, 3] = grip[indices + stride]
    return actions


class ReplayNode(Node):
    def __init__(self, actions, mock=False):
        super().__init__("grip_replay")
        self.actions = np.asarray(actions, dtype=np.float32)
        self.index = 0
        self.mock = bool(mock)
        self.publisher = self.create_publisher(Float64MultiArray, "/grip_eval/replay_action", 10)
        self.create_timer(1.0 / DEFAULT_ACTION_HZ, self._tick)

    def _tick(self):
        if self.index >= len(self.actions):
            return
        msg = Float64MultiArray()
        msg.data = self.actions[self.index].astype(float).tolist()
        self.publisher.publish(msg)
        self.index += 1


def _parse_args(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--episode-h5", required=True)
    parser.add_argument("--action-rate", type=int, default=DEFAULT_ACTION_HZ)
    parser.add_argument("--phase", type=int, default=0, choices=range(4))
    parser.add_argument("--mock", action="store_true")
    parser.add_argument("--dump-json")
    return parser.parse_known_args(argv)


def main(args=None):
    parsed, ros_args = _parse_args(sys.argv[1:] if args is None else args)
    actions = actions_from_hdf5(parsed.episode_h5, parsed.action_rate, parsed.phase)
    if parsed.dump_json:
        Path(parsed.dump_json).write_text(json.dumps(actions.tolist(), indent=2) + "\n")
        return
    rclpy.init(args=ros_args)
    node = ReplayNode(actions, mock=parsed.mock)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
