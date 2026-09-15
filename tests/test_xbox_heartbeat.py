import pytest

pytest.importorskip(
    "rclpy",
    reason="needs ROS 2 on the path: source /opt/ros/jazzy/setup.bash and "
           "ros2_ws/install/setup.bash")
pytest.importorskip(
    "indy_interfaces",
    reason="needs this repository's built colcon workspace: "
           "source ros2_ws/install/setup.bash")

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace


TELEOP_DIR = (
    Path(__file__).parents[1]
    / 'ros2_ws/src/indy7_teleop/indy7_teleop'
)
sys.path.insert(0, str(TELEOP_DIR.parent))
SPEC = importlib.util.spec_from_file_location(
    'xbox_servo_node_under_test', TELEOP_DIR / 'xbox_servo_node.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _fake_node(teleop_mode):
    published = []
    node = SimpleNamespace(
        awaiting_label=False,
        teleop_mode=teleop_mode,
        linear_step=0.25,
        relative_pose=[1.0] * 6,
        _button=lambda _index: False,
        _dpad=lambda: (0.0, 0.0),
        _trigger=lambda _index: 0.0,
        _publish_pose=lambda: published.append(list(node.relative_pose)),
    )
    return node, published


def test_idle_input_republishes_cumulative_target_during_task_teleop():
    node, published = _fake_node('task')

    MODULE.XboxServoNode._update_cartesian_from_joystick(node)

    assert published == [[1.0] * 6]


def test_idle_input_does_not_start_teleop():
    node, published = _fake_node(None)

    MODULE.XboxServoNode._update_cartesian_from_joystick(node)

    assert published == []


def test_held_direction_accumulates_relative_target():
    node, published = _fake_node('task')
    node.relative_pose = [0.0] * 6
    node._dpad = lambda: (0.0, 1.0)
    node._ensure_task_teleop = lambda: True

    MODULE.XboxServoNode._update_cartesian_from_joystick(node)
    MODULE.XboxServoNode._update_cartesian_from_joystick(node)

    assert published == [
        [0.25, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.5, 0.0, 0.0, 0.0, 0.0, 0.0],
    ]


def test_ending_recording_keeps_teleop_alive_while_waiting_for_label():
    pause = object()
    node = SimpleNamespace(
        workflow_prompt=None,
        is_recording=True,
        awaiting_label=False,
        recording_task=None,
        collection_task='',
        teleop_mode='task',
        data_pause=pause,
        _call_trigger=lambda client: client is pause,
        _final_gripper_warning=lambda _task: '',
    )

    MODULE.XboxServoNode._handle_record_button(node)

    assert node.teleop_mode == 'task'
    assert node.awaiting_label is True


def test_ending_recording_pauses_frames_before_label_wait():
    calls = []
    pause = object()
    node = SimpleNamespace(
        workflow_prompt=None,
        is_recording=True,
        awaiting_label=False,
        recording_task='',
        collection_task='',
        teleop_mode='task',
        data_pause=pause,
        _call_trigger=lambda client: calls.append(client) or True,
        _final_gripper_warning=lambda _task: '',
    )

    MODULE.XboxServoNode._handle_record_button(node)

    assert calls == [pause]
    assert node.awaiting_label is True


def test_starting_recording_enters_task_teleop_before_data_start():
    calls = []
    node = SimpleNamespace(
        workflow_prompt=None,
        is_recording=False,
        awaiting_label=False,
        recording_task=None,
        collection_task='',
        _ensure_task_teleop=lambda: calls.append('teleop') or True,
        _publish_current_task=lambda repeat=1: calls.append(('context', repeat)),
        _task_label=lambda _task: 'task',
        _call_trigger=lambda _client: calls.append('record') or True,
        data_start=object(),
    )

    MODULE.XboxServoNode._handle_record_button(node)

    assert calls == ['teleop', ('context', 3), 'record']


def test_label_pause_republishes_heartbeat_without_motion():
    node, published = _fake_node('task')
    node.awaiting_label = True

    MODULE.XboxServoNode._update_cartesian_from_joystick(node)

    assert published == [[1.0] * 6]


def test_success_closes_hdf5_before_stopping_task_teleop():
    calls = []
    mark_success = object()
    data_stop = object()
    node = SimpleNamespace(
        awaiting_label=True,
        recording_task='',
        collection_task='',
        data_mark_success=mark_success,
        data_mark_fail=object(),
        data_stop=data_stop,
        _task_label=lambda _task: 'task',
        _final_gripper_warning=lambda _task: '',
        _call_trigger=lambda client, **_kwargs: calls.append(
            'mark' if client is mark_success else 'hdf5_stop') or True,
        _call_indy=lambda code: calls.append(('indy', code)) or True,
        _reset_teleop_targets=lambda: calls.append('reset'),
        _after_recording_finished=lambda _task, _label: calls.append('finished'),
    )

    MODULE.XboxServoNode._finish_recording(node, 'success')

    assert calls == [
        'mark', 'hdf5_stop', ('indy', MODULE.MSG_TELE_STOP), 'reset', 'finished'
    ]
