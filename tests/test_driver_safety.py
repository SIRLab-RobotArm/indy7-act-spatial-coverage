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
import ast
from pathlib import Path
import sys
from types import SimpleNamespace


DRIVER_DIR = (
    Path(__file__).parents[1]
    / 'ros2_ws/src/indy7_ros2/indy_driver/indy_driver'
)
if not (DRIVER_DIR / 'indy_driver.py').is_file():
    pytest.skip(
        "needs the Indy7 driver cloned into ros2_ws/src/indy7_ros2 "
        "(see ros2_ws/src/README.md)",
        allow_module_level=True)
sys.path.insert(0, str(DRIVER_DIR))
SPEC = importlib.util.spec_from_file_location('indy_driver_under_test', DRIVER_DIR / 'indy_driver.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _callback_group_keyword(call):
    for keyword in call.keywords:
        if keyword.arg == 'callback_group':
            return ast.unparse(keyword.value)
    return None


def test_state_timer_is_separate_from_blocking_command_callbacks():
    tree = ast.parse((DRIVER_DIR / 'indy_driver.py').read_text(encoding='utf-8'))
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]
    timer_calls = [
        call for call in calls
        if isinstance(call.func, ast.Attribute) and call.func.attr == 'create_timer'
    ]
    assert any(
        _callback_group_keyword(call) == 'self.state_callback_group'
        for call in timer_calls
    )
    command_calls = [
        call for call in calls
        if isinstance(call.func, ast.Attribute)
        and call.func.attr in {'create_subscription', 'create_service'}
        and _callback_group_keyword(call) == 'self.command_callback_group'
    ]
    assert len(command_calls) >= 4


def test_workspace_and_step_guard():
    validate = MODULE.validate_relative_command
    bounds_min = [-10.0, -10.0, 15.0]
    bounds_max = [10.0, 10.0, 250.0]
    assert validate([0, 0, 100], [10, 0, 0, 0, 0, 0], bounds_min, bounds_max)[0]
    assert validate([0, 0, 100], [26, 0, 0, 0, 0, 0], bounds_min, bounds_max)[1] == 'max_delta'
    assert validate([10, 0, 100], [1, 0, 0, 0, 0, 0], bounds_min, bounds_max)[1] == 'workspace'
    assert validate(
        [10, 0, 100], [1, 0, 0, 0, 0, 0], bounds_min, bounds_max,
        enforce_workspace=False,
    )[0]
    assert validate(
        [0, 0, 100], [26, 0, 0, 0, 0, 0], bounds_min, bounds_max,
        enforce_workspace=False,
    )[1] == 'max_delta'
    assert validate(
        [0, 0, 100], [30, 0, 0, 0, 0, 0], bounds_min, bounds_max,
        enforce_workspace=False, previous_delta_pose=[10, 0, 0, 0, 0, 0],
    )[0]
    assert validate(
        [0, 0, 100], [40, 0, 0, 0, 0, 0], bounds_min, bounds_max,
        enforce_workspace=False, previous_delta_pose=[10, 0, 0, 0, 0, 0],
    )[1] == 'max_delta'
    assert validate([0, 0, 100], [0, 0, 0, 1, 0, 0], bounds_min, bounds_max)[1] == 'rotation_disabled'


def test_mock_indy_applies_relative_motion():
    robot = MODULE.MockIndy()
    robot.start_teleop()
    robot.movetelel_rel([2, -3, 4, 0, 0, 0])
    assert robot.get_control_data()['p'][:3] == [2.0, -3.0, 104.0]
    robot.stop_motion()
    assert robot.stop_count == 1


def test_zero_delta_refreshes_watchdog_without_sending_robot_command():
    robot_commands = []
    robot = SimpleNamespace(
        get_control_data=lambda: {'p': [0.0, 0.0, 100.0]},
        movetelel_rel=lambda **kwargs: robot_commands.append(kwargs),
    )
    node = SimpleNamespace(
        indy_msg_status=MODULE.MSG_TELE_TASK_RLT,
        fault_latch=False,
        enforce_workspace=True,
        workspace_configured=True,
        workspace_min=[-10.0, -10.0, 15.0],
        workspace_max=[10.0, 10.0, 250.0],
        max_delta_mm=25.0,
        guard_count=0,
        indy=robot,
        vel_ratio=0.2,
        acc_ratio=2.0,
        last_teleop_command=None,
        last_command_monotonic=None,
        teleop_origin_pose=[0.0, 0.0, 100.0, 0.0, 0.0, 0.0],
    )

    MODULE.IndyROSConnector.teleop_pose_callback(
        node, SimpleNamespace(data=[0.0] * 6))

    assert robot_commands == []
    assert node.last_teleop_command == [0.0] * 6
    assert node.last_command_monotonic is not None


def test_duplicate_cumulative_target_is_heartbeat_only():
    robot_commands = []
    robot = SimpleNamespace(
        get_control_data=lambda: {'p': [1.0, 0.0, 100.0]},
        movetelel_rel=lambda **kwargs: robot_commands.append(kwargs),
    )
    target = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    node = SimpleNamespace(
        indy_msg_status=MODULE.MSG_TELE_TASK_RLT,
        fault_latch=False,
        enforce_workspace=False,
        workspace_configured=False,
        workspace_min=[-10.0, -10.0, 15.0],
        workspace_max=[10.0, 10.0, 250.0],
        max_delta_mm=1.0,
        guard_count=0,
        indy=robot,
        vel_ratio=0.2,
        acc_ratio=2.0,
        last_teleop_command=list(target),
        last_command_monotonic=None,
        teleop_origin_pose=[0.0, 0.0, 100.0, 0.0, 0.0, 0.0],
    )

    MODULE.IndyROSConnector.teleop_pose_callback(
        node, SimpleNamespace(data=target))

    assert robot_commands == []
    assert node.last_command_monotonic is not None


def test_stop_disarms_watchdog_before_blocking_robot_calls():
    node = SimpleNamespace(
        indy_msg_status=MODULE.MSG_TELE_TASK_RLT,
        last_command_monotonic=123.0,
        teleop_origin_pose=[1.0] * 6,
        last_teleop_command=[2.0] * 6,
    )

    MODULE.IndyROSConnector._disarm_watchdog_for_stop(node)

    assert node.indy_msg_status == MODULE.MSG_TELE_STOP
    assert node.last_command_monotonic is None
    assert node.teleop_origin_pose is None
    assert node.last_teleop_command is None


def test_cumulative_target_guard_limits_only_the_new_increment():
    robot_commands = []
    robot = SimpleNamespace(
        get_control_data=lambda: {'p': [20.0, 0.0, 100.0]},
        movetelel_rel=lambda **kwargs: robot_commands.append(kwargs),
    )
    node = SimpleNamespace(
        indy_msg_status=MODULE.MSG_TELE_TASK_RLT,
        fault_latch=False,
        enforce_workspace=False,
        workspace_configured=False,
        workspace_min=[-10.0, -10.0, 15.0],
        workspace_max=[10.0, 10.0, 250.0],
        max_delta_mm=25.0,
        guard_count=0,
        indy=robot,
        vel_ratio=0.2,
        acc_ratio=2.0,
        last_teleop_command=[10.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        last_command_monotonic=None,
        teleop_origin_pose=[0.0, 0.0, 100.0, 0.0, 0.0, 0.0],
    )

    target = [30.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    MODULE.IndyROSConnector.teleop_pose_callback(
        node, SimpleNamespace(data=target))

    assert robot_commands == [{
        'tpos': target,
        'vel_ratio': 0.2,
        'acc_ratio': 2.0,
    }]
    assert node.last_teleop_command == target


def test_move_home_uses_fixed_collection_joint_pose(monkeypatch):
    monkeypatch.setattr(MODULE.time, 'sleep', lambda _seconds: None)
    robot = MODULE.MockIndy()
    node = SimpleNamespace(
        indy=robot,
        indy_msg_status=MODULE.MSG_TELE_STOP,
        home_joint_deg=list(MODULE.COLLECTION_HOME_JOINT_DEG),
        get_logger=lambda: SimpleNamespace(info=lambda _message: None),
    )
    response = SimpleNamespace(success=False, message='')

    result = MODULE.IndyROSConnector.indy_srv_callback(
        node, SimpleNamespace(data=MODULE.MSG_MOVE_HOME), response)

    assert result.success
    assert robot.q == MODULE.COLLECTION_HOME_JOINT_DEG
    assert node.indy_msg_status == MODULE.MSG_MOVE_HOME
    assert 'collection HOME reached' in result.message
