#!/usr/bin/env python3
"""Record session teach-in poses without requiring camera calibration."""

import argparse
import os
from pathlib import Path
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
import yaml


def default_positions_file() -> str:
    """Path to experiment/positions.yaml, resolved from GRIP_REPO_ROOT or layout."""
    override = os.environ.get("GRIP_REPO_ROOT")
    if override:
        return str(Path(override).resolve() / "experiment" / "positions.yaml")
    for parent in Path(__file__).resolve().parents:
        candidate = parent / "experiment" / "positions.yaml"
        if candidate.is_file():
            return str(candidate)
    return "experiment/positions.yaml"


VALID_POSITION_IDS = {
    *(f"grid_{index}" for index in range(1, 10)),
    *(f"eval_{index}" for index in range(1, 9)),
    "pilot_1",
    "pilot_2",
}


class TeachTargetNode(Node):
    def __init__(self, position_id, positions_file, mock=False, mock_pose=None):
        super().__init__('teach_target')
        if position_id not in VALID_POSITION_IDS:
            raise ValueError(f'unknown position_id: {position_id}')
        self.position_id = position_id
        self.positions_file = Path(positions_file).resolve()
        self.latest_pose = list(mock_pose) if mock and mock_pose is not None else None
        self.mock = bool(mock)
        self.create_subscription(
            Float64MultiArray, '/indy/ee_pose', self._pose_callback, 10)
        self.create_service(Trigger, '/grip_eval/teach_target', self._teach_callback)
        self.get_logger().info(
            f'Ready to teach {position_id} into {self.positions_file} (mock={self.mock})')

    def _pose_callback(self, msg):
        if len(msg.data) == 6:
            self.latest_pose = [float(value) for value in msg.data]

    def _teach_callback(self, _request, response):
        if self.latest_pose is None:
            response.success = False
            response.message = 'No /indy/ee_pose sample received'
            return response
        self._write_pose(self.latest_pose)
        response.success = True
        response.message = f'{self.position_id}: {self.latest_pose}'
        return response

    def _write_pose(self, pose):
        with self.positions_file.open('r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream) or {}
        positions = data.setdefault('positions', {})
        positions[self.position_id] = {
            'ee_pose_mm_deg': [float(value) for value in pose],
            'taught': True,
        }
        temporary = self.positions_file.with_suffix('.yaml.tmp')
        with temporary.open('w', encoding='utf-8') as stream:
            yaml.safe_dump(data, stream, sort_keys=False)
        temporary.replace(self.positions_file)


def _parse_args(args):
    parser = argparse.ArgumentParser()
    parser.add_argument('--mock', action='store_true')
    parser.add_argument('--position-id', default='pilot_1', choices=sorted(VALID_POSITION_IDS))
    parser.add_argument('--positions-file', default=default_positions_file())
    parser.add_argument(
        '--mock-pose', nargs=6, type=float,
        default=[0.0, 0.0, 100.0, 0.0, 0.0, 0.0])
    return parser.parse_known_args(args)


def main(args=None):
    parsed, ros_args = _parse_args(sys.argv[1:] if args is None else args)
    rclpy.init(args=[sys.argv[0], *ros_args])
    node = TeachTargetNode(
        parsed.position_id, parsed.positions_file, parsed.mock, parsed.mock_pose)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
