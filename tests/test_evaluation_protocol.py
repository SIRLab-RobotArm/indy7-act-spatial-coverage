from collections import Counter
import csv

import pytest

from ai.eval.protocol import (
    CONDITION_GRIDS,
    EVAL_IDS,
    GRID_IDS,
    build_balanced_schedule,
    evaluation_distances,
)
from scripts.eval.prepare_targets import first_close_index, interpolated_q_entry
from scripts.eval.trial_commands import commands


def _positions():
    positions = {
        grid_id: {
            'ee_pose_mm_deg': [float(index * 10), 0.0, 500.0, 0.0, 0.0, 0.0],
            'taught': True,
        }
        for index, grid_id in enumerate(GRID_IDS)
    }
    eval_grid_indices = (4, 0, 5, 7)
    for eval_index, grid_index in enumerate(eval_grid_indices, start=1):
        positions[f'eval_{eval_index}'] = {
            'ee_pose_mm_deg': [float(grid_index * 10), 0.0, 500.0, 0.0, 0.0, 0.0],
            'taught': True,
        }
    for eval_index, x in enumerate((15.0, 25.0, 55.0, 65.0), start=5):
        positions[f'eval_{eval_index}'] = {
            'ee_pose_mm_deg': [x, 0.0, 500.0, 0.0, 0.0, 0.0],
            'taught': True,
        }
    return {'positions': positions}


def test_distance_is_planar_and_condition_specific():
    distances = evaluation_distances(_positions())

    assert distances['eval_1']['A'] == pytest.approx(0.0)
    assert distances['eval_2']['A'] == pytest.approx(40.0)
    assert distances['eval_5']['C'] == pytest.approx(5.0)
    assert distances['eval_5']['D'] == pytest.approx(5.0)


def test_480_schedule_is_balanced_and_preserves_192_prefix():
    schedule = build_balanced_schedule(20260824)
    original = build_balanced_schedule(20260824, repeat_count=2)

    assert len(schedule) == 480
    assert schedule[:192] == original
    assert len({row['rollout_id'] for row in schedule}) == 480
    assert all(
        left['model_code'] != right['model_code']
        and left['condition'] != right['condition']
        and left['position_id'] != right['position_id']
        for left, right in zip(schedule, schedule[1:])
    )
    for session in range(1, 21):
        rows = [row for row in schedule if row['session'] == session]
        assert len(rows) == 24
        assert set(Counter(row['model_code'] for row in rows).values()) == {2}
        assert Counter(row['condition'] for row in rows) == {
            condition: 6 for condition in CONDITION_GRIDS
        }
        assert Counter(row['seed'] for row in rows) == {0: 8, 1: 8, 2: 8}
        assert Counter(row['position_id'] for row in rows) == {
            position_id: 3 for position_id in EVAL_IDS
        }
    assert set(Counter(
        (row['model_code'], row['position_id'], row['repeat']) for row in schedule
    ).values()) == {1}
    assert Counter(row['repeat'] for row in schedule) == {
        repeat: 96 for repeat in range(1, 6)
    }


def test_first_close_requires_exactly_one_transition():
    assert first_close_index([0, 0, 1, 1]) == 2
    with pytest.raises(ValueError, match='exactly one'):
        first_close_index([0, 0, 0])
    with pytest.raises(ValueError, match='exactly one'):
        first_close_index([0, 1, 0, 1])


def test_q_target_is_derived_from_four_grids_without_physical_teach_in():
    positions = {
        f'grid_{index}': {'ee_pose_mm_deg': pose}
        for index, pose in enumerate((
            [0, 0, 500, 179, 0, 0],
            [10, 0, 500, -179, 0, 0],
            [0, 10, 500, 178, 0, 0],
            [10, 10, 500, -178, 0, 0],
        ), start=1)
    }

    entry = interpolated_q_entry(
        positions, ('grid_1', 'grid_2', 'grid_3', 'grid_4'))

    assert entry['ee_pose_mm_deg'][:3] == pytest.approx([5, 5, 500])
    assert abs(abs(entry['ee_pose_mm_deg'][3]) - 180) < 1e-9
    assert entry['physically_taught'] is False
    assert entry['taught'] is False
    assert entry['target_defined'] is True
    assert entry['source']['q_demonstration_count'] == 0
    assert entry['source']['weights'] == [0.25] * 4


def test_trial_commands_include_frozen_target_and_complete_metadata(tmp_path):
    schedule = tmp_path / 'schedule.csv'
    with schedule.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=(
            'trial', 'session', 'model_code', 'condition', 'seed',
            'position_id', 'repeat', 'rollout_id'))
        writer.writeheader()
        writer.writerow({
            'trial': 1, 'session': 1, 'model_code': 'M01', 'condition': 'A',
            'seed': 0, 'position_id': 'eval_1', 'repeat': 1,
            'rollout_id': 't001_m01_eval_1_r1',
        })
    positions = _positions()
    positions['evaluation_distances_mm'] = evaluation_distances(positions)
    positions_path = tmp_path / 'positions.yaml'
    import yaml
    positions_path.write_text(yaml.safe_dump(positions), encoding='utf-8')

    result = commands(schedule, positions_path, 1)

    assert 'require_complete_metadata:=true' in result
    assert '"p_target":[40.0,0.0,500.0]' in result
    assert '"attempt":1' in result
    assert '"execution_controller":"linear_interp_30hz_v1"' in result
    assert '"policy_action_hz":5.0' in result
    assert '"robot_command_hz":30.0' in result
    assert '"interpolation_steps":6' in result
    assert '/grip_eval/result/success' in result
    assert '/grip_eval/object_move' in result

    retry = commands(schedule, positions_path, 1, attempt=2)
    assert 'rollout_id:=t001_m01_eval_1_r1_retry1' in retry
    assert '"attempt":2' in retry
