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
import time
from types import SimpleNamespace


TELEOP_DIR = Path(__file__).parents[1] / 'ros2_ws/src/indy7_teleop/indy7_teleop'
sys.path.insert(0, str(TELEOP_DIR.parent))
SPEC = importlib.util.spec_from_file_location(
    'xbox_teach_node_under_test', TELEOP_DIR / 'xbox_servo_node.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class FakeStore:
    def __init__(self):
        self.saved = []

    def append_sample(self, position_id, pose, joints):
        self.saved.append((position_id, list(pose), list(joints)))
        return {
            'teach': {
                'n_samples': 1,
                'xyz_max_mm': 0.0,
                'spread_warning': False,
            },
        }


def _save_node():
    store = FakeStore()
    counts = {position_id: 0 for position_id in MODULE.TEACH_POSITION_IDS}
    node = SimpleNamespace(
        teach_index=0,
        teach_samples=3,
        teach_counts=counts,
        teach_store=store,
        teach_pose_max_age_s=0.5,
        ee_pose=[100, 200, 300, 10, 20, 30],
        ee_pose_stamp=time.monotonic(),
        joint_positions_deg=[0, 10, 20, 30, 40, 50],
        joint_state_stamp=time.monotonic(),
        teach_home_reached=True,
        sample_needs_regrasp=False,
        last_gripper_action=MODULE.ACTION_CLOSE,
        teach_status='',
        get_logger=lambda: SimpleNamespace(error=lambda _message: None),
    )
    node._current_teach_id = lambda: MODULE.TEACH_POSITION_IDS[node.teach_index]
    node._first_incomplete_teach_index = lambda _start=0: 1
    return node, store


def test_save_records_deep_copies_of_eef_and_joint_values():
    node, store = _save_node()

    MODULE.XboxServoNode._save_teach_sample(node)
    node.ee_pose[0] = -999
    node.joint_positions_deg[0] = -999

    assert store.saved == [(
        'grid_1',
        [100, 200, 300, 10, 20, 30],
        [0, 10, 20, 30, 40, 50],
    )]
    assert node.sample_needs_regrasp is True
    assert node.teach_home_reached is False


def test_save_requires_home_and_a_new_closed_grasp():
    node, store = _save_node()
    node.teach_home_reached = False
    MODULE.XboxServoNode._save_teach_sample(node)
    assert store.saved == []
    assert 'BACK+Y HOME' in node.teach_status

    node.teach_home_reached = True
    node.sample_needs_regrasp = True
    MODULE.XboxServoNode._save_teach_sample(node)
    assert store.saved == []
    assert 'new grasp' in node.teach_status

    node.sample_needs_regrasp = False
    node.last_gripper_action = MODULE.ACTION_OPEN
    MODULE.XboxServoNode._save_teach_sample(node)
    assert store.saved == []
    assert 'close the gripper' in node.teach_status


def test_back_combinations_are_not_consumed_by_teach_buttons():
    node = SimpleNamespace(
        teach_toggle_button=MODULE.BTN_MODE,
        teach_mode=True,
        _button=lambda index: index == MODULE.BTN_BACK,
    )

    handled = MODULE.XboxServoNode._handle_teach_button(node, MODULE.BTN_Y)

    assert handled is False


def test_teach_mode_does_not_publish_collection_context():
    published = []
    node = SimpleNamespace(teach_mode=True, task_pub=SimpleNamespace(
        publish=lambda message: published.append(message)))

    MODULE.XboxServoNode._publish_current_task(node)

    assert published == []
