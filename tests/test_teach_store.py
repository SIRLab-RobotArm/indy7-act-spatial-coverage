from pathlib import Path
import sys

import pytest
import yaml


TELEOP_SRC = Path(__file__).parents[1] / 'ros2_ws/src/indy7_teleop'
sys.path.insert(0, str(TELEOP_SRC))

from indy7_teleop.teach_store import TeachStore, mean_pose  # noqa: E402


def _positions_file(tmp_path):
    path = tmp_path / 'positions.yaml'
    positions = {
        **{f'grid_{index}': {'ee_pose_mm_deg': None, 'taught': False}
           for index in range(1, 10)},
        **{f'eval_{index}': {'ee_pose_mm_deg': None, 'taught': False}
           for index in range(1, 9)},
        'pilot_1': {'ee_pose_mm_deg': None, 'taught': False},
        'pilot_2': {'ee_pose_mm_deg': None, 'taught': False},
    }
    path.write_text(yaml.safe_dump({
        'schema_version': 1,
        'frame': 'indy_base',
        'camera_mount': {'overhead_tilt_deg': None},
        'positions': positions,
    }, sort_keys=False), encoding='utf-8')
    return path


def test_angle_mean_wraps_across_180_boundary():
    average = mean_pose([
        [1, 2, 3, 0, 0, 179.5],
        [1, 2, 3, 0, 0, -179.8],
        [1, 2, 3, 0, 0, 179.9],
    ])

    assert average[5] == pytest.approx(179.8666667)


def test_three_eef_and_joint_samples_are_saved_and_averaged(tmp_path):
    path = _positions_file(tmp_path)
    store = TeachStore(path, samples_target=3, spread_warn_mm=3.0)
    poses = [
        [100, 200, 300, 10, 20, 30],
        [101, 199, 300, 10.2, 20, 29.8],
        [99, 201, 300, 9.8, 20, 30.2],
    ]
    joints = [
        [0, 10, 20, 30, 40, 50],
        [1, 11, 21, 31, 41, 51],
        [-1, 9, 19, 29, 39, 49],
    ]

    for pose, joint in zip(poses, joints):
        entry = store.append_sample('grid_5', pose, joint)

    assert entry['taught'] is True
    assert entry['ee_pose_mm_deg'] == pytest.approx([100, 200, 300, 10, 20, 30])
    assert entry['teach']['samples_mm_deg'] == poses
    assert entry['teach']['joint_deg'] == joints
    assert entry['teach']['joint_mean_deg'] == pytest.approx([0, 10, 20, 30, 40, 50])
    assert entry['teach']['xyz_max_mm'] == pytest.approx(2 ** 0.5)

    saved = yaml.safe_load(path.read_text(encoding='utf-8'))
    mirror = saved['positions']['eval_1']
    assert mirror['ee_pose_mm_deg'] == pytest.approx(
        [100, 200, 300, 10, 20, 30])
    assert mirror['taught'] is True
    assert mirror['mirror_of'] == 'grid_5'
    assert mirror['teach']['joint_deg'] == joints
    assert saved['schema_version'] == 1
    assert saved['camera_mount'] == {'overhead_tilt_deg': None}
    assert not path.with_suffix('.yaml.tmp').exists()


def test_undo_removes_matching_eef_and_joint_sample(tmp_path):
    path = _positions_file(tmp_path)
    store = TeachStore(path)
    store.append_sample('grid_1', [1, 2, 3, 4, 5, 6], [10, 20, 30, 40, 50, 60])
    store.append_sample('grid_1', [2, 3, 4, 5, 6, 7], [11, 21, 31, 41, 51, 61])

    store.pop_sample('grid_1')
    saved = yaml.safe_load(path.read_text(encoding='utf-8'))
    teach = saved['positions']['grid_1']['teach']
    assert teach['n_samples'] == 1
    assert teach['samples_mm_deg'] == [[1, 2, 3, 4, 5, 6]]
    assert teach['joint_deg'] == [[10, 20, 30, 40, 50, 60]]
    assert saved['positions']['eval_2']['mirror_of'] == 'grid_1'
    assert saved['positions']['eval_2']['taught'] is False


def test_large_spread_warns_without_dropping_samples(tmp_path):
    store = TeachStore(_positions_file(tmp_path), spread_warn_mm=3.0)
    entry = None
    for x in (0, 1, 10):
        entry = store.append_sample('eval_5', [x, 0, 0, 0, 0, 0], [0] * 6)

    assert entry['teach']['spread_warning'] is True
    assert entry['teach']['n_samples'] == 3


def test_derived_grid_and_q_targets_are_complete_without_teach_samples(tmp_path):
    path = _positions_file(tmp_path)
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    data['positions']['grid_5'] = {
        'ee_pose_mm_deg': [600, 0, 540, 90, 0, 90],
        'taught': True,
        'target_defined': True,
        'source': {
            'method': 'successful_demonstration_first_close_mean',
            'episode_count': 60,
        },
    }
    data['positions']['eval_5'] = {
        'ee_pose_mm_deg': [675, 75, 540, 90, 0, 90],
        'taught': False,
        'target_defined': True,
        'physically_taught': False,
        'source': {
            'method': 'surrounding_grid_cell_center_mean',
            'q_demonstration_count': 0,
        },
    }
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding='utf-8')

    counts = TeachStore(path, samples_target=3).load_counts()

    assert counts['grid_5'] == 3
    assert counts['eval_5'] == 3
