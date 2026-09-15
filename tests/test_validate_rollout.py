import pandas as pd
import pytest

from ai.eval.validate_rollout import grasp_position_metrics, lift_hold_metrics


def _trace(z_values, gripper=None):
    if gripper is None:
        gripper = [1] * len(z_values)
    return pd.DataFrame({
        't': [index * 0.05 for index in range(len(z_values))],
        'ee_pose': [[0.0, 0.0, z, 0.0, 0.0, 0.0] for z in z_values],
        'gripper_cmd': gripper,
    })


def test_lift_and_continuous_hold_are_measured_after_close():
    trace = _trace([500.0] * 10 + [551.0] * 62)

    metrics = lift_hold_metrics(trace, t_close=0.45, threshold_mm=50.0)

    assert metrics['lift_mm'] == pytest.approx(51.0)
    assert metrics['hold_duration_s'] == pytest.approx(3.05)


def test_hold_resets_on_height_drop():
    trace = _trace([500.0] * 5 + [551.0] * 30 + [540.0] + [551.0] * 30)

    metrics = lift_hold_metrics(trace, t_close=0.2, threshold_mm=50.0)

    assert metrics['hold_duration_s'] == pytest.approx(1.45)


def test_grasp_position_metrics_interpolates_and_reports_signed_error():
    trace = pd.DataFrame({
        't': [0.0, 1.0],
        'ee_pose': [
            [1.0, 2.0, 3.0, 0.0, 0.0, 0.0],
            [3.0, 6.0, 9.0, 0.0, 0.0, 0.0],
        ],
    })

    metrics = grasp_position_metrics(trace, 0.5, [1.0, 2.0, 3.0])

    assert metrics['target_xyz_mm'] == [1.0, 2.0, 3.0]
    assert metrics['grasp_xyz_mm'] == [2.0, 4.0, 6.0]
    assert metrics['delta_xyz_mm'] == [1.0, 2.0, 3.0]
    assert metrics['xy_error_mm'] == pytest.approx(5 ** 0.5)
    assert metrics['error_3d_mm'] == pytest.approx(14 ** 0.5)


def test_grasp_position_metrics_without_close_keeps_target_only():
    metrics = grasp_position_metrics(_trace([500.0]), None, [1.0, 2.0, 3.0])

    assert metrics['target_xyz_mm'] == [1.0, 2.0, 3.0]
    assert metrics['grasp_xyz_mm'] is None
    assert metrics['error_3d_mm'] is None
