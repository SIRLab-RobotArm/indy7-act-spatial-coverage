import pytest

pytest.importorskip(
    "rclpy",
    reason="needs ROS 2 on the path: source /opt/ros/jazzy/setup.bash and "
           "ros2_ws/install/setup.bash")
pytest.importorskip(
    "indy_interfaces",
    reason="needs this repository's built colcon workspace: "
           "source ros2_ws/install/setup.bash")

from pathlib import Path
import sys


EVAL_SRC = Path(__file__).parents[1] / 'ros2_ws/src/grip_eval'
sys.path.insert(0, str(EVAL_SRC))

from grip_eval.rollout_logger_node import RolloutLoggerNode  # noqa: E402


class _Response:
    success = None
    message = None


def _node():
    node = RolloutLoggerNode.__new__(RolloutLoggerNode)
    node.stream = object()
    node.events = []
    node.summary = {'t_start': 1.0, 't_close': None, 't_move': None, 't_end': None}
    node.metadata = {
        'condition': 'A', 'seed': 0, 'position_id': 'eval_1',
        'model_code': 'M01', 'session': 1, 'repeat': 1, 'attempt': 1,
        'scheduled_rollout_id': 't001_m01_eval_1_r1',
        'p_target': [1.0, 2.0, 3.0], 'd': 0.0,
        'success': None, 'failure_code': None, 'object_move': False,
        'lift_threshold_mm': 50.0, 'hold_duration_s': 3.0,
        'experiment_tag': 'main',
    }
    return node


def test_result_and_object_move_services_complete_metadata():
    node = _node()
    response = _Response()

    node._mark_object_move(None, response)
    assert response.success is True
    assert node.metadata['object_move'] is True
    assert node.summary['t_move'] is not None

    response = _Response()
    node._mark_success(None, response)
    assert response.success is True
    assert node.metadata['success'] is True
    assert node.metadata['failure_code'] is None
    assert node._metadata_problems() == []


def test_failed_rollout_requires_a_frozen_failure_code():
    node = _node()
    node.metadata['success'] = False

    assert 'failed rollout requires a frozen failure_code' in node._metadata_problems()

    response = _Response()
    node._failure_callback('missed_grasp')(None, response)
    assert response.success is True
    assert node.metadata['failure_code'] == 'missed_grasp'


def test_binary_operator_failure_is_a_valid_failure_code():
    node = _node()
    response = _Response()

    node._failure_callback('operator_failure')(None, response)

    assert response.success is True
    assert node.metadata['success'] is False
    assert node.metadata['failure_code'] == 'operator_failure'
    assert node._metadata_problems() == []
