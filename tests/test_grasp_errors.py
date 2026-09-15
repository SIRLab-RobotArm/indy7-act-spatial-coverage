import numpy as np
import pandas as pd

from ai.eval.analyze import (
    descriptive_success, eligibility, evaluation_group, grasp_errors,
    validate_analysis_set)


def test_grasp_errors_interpolate_first_close_and_cut_before_move():
    trace = pd.DataFrame({
        "t": [0.0, 1.0, 2.0],
        "ee_pose": [
            [10.0, 0.0, 5.0, 0, 0, 0],
            [4.0, 0.0, 3.0, 0, 0, 0],
            [0.0, 0.0, 1.0, 0, 0, 0],
        ],
    })
    e1, e2, e3, clean = grasp_errors(
        trace, {"t_close": 1.5, "t_move": 1.75, "t_end": 2.0}, [0.0, 0.0, 0.0])
    assert e1 == 2.0
    assert e2 == 2.0
    assert e3 == 4.0
    assert clean is True


def test_grasp_errors_no_close_keeps_e3_defined():
    trace = pd.DataFrame({
        "t": [0.0, 1.0],
        "ee_pose": [[3.0, 4.0, 2.0, 0, 0, 0], [0.0, 2.0, 1.0, 0, 0, 0]],
    })
    e1, e2, e3, clean = grasp_errors(
        trace, {"t_close": None, "t_move": None, "t_end": 1.0}, [0.0, 0.0, 0.0])
    assert e1 is None and e2 is None and clean is None
    assert e3 == 2.0


def test_descriptive_success_groups_rgb_models_by_condition():
    data = pd.DataFrame({
        "condition": ["A", "A", "B"],
        "success": [True, False, True],
    })
    rows = descriptive_success(data)
    assert [(row["condition"], row["successes"], row["total"]) for row in rows] == [
        ("A", 1, 2),
        ("B", 1, 1),
    ]


def test_descriptive_success_reports_model_variation_and_eval_strata():
    data = pd.DataFrame({
        "condition": ["A"] * 4,
        "evaluation_group": ["exact_grid", "exact_grid", "q_unseen", "q_unseen"],
        "model_code": ["M01", "M02", "M01", "M02"],
        "success": [True, False, True, True],
    })

    overall = descriptive_success(data)[0]
    strata = descriptive_success(data, ("condition", "evaluation_group"))

    assert overall["model_count"] == 2
    assert overall["model_rate_mean"] == 0.75
    assert [(row["evaluation_group"], row["total"]) for row in strata] == [
        ("exact_grid", 2), ("q_unseen", 2)]


def test_evaluation_group_and_primary_eligibility_are_frozen():
    assert evaluation_group("eval_1") == "exact_grid"
    assert evaluation_group("eval_8") == "q_unseen"
    assert eligibility({"object_move": False, "failure_code": "missed_grasp"}) == (
        True, None)
    assert eligibility({"object_move": True, "failure_code": None}) == (
        False, "object_move")
    assert eligibility({"object_move": False, "failure_code": "hardware_fault"}) == (
        False, "hardware_fault")


def test_analysis_rejects_two_eligible_attempts_for_one_schedule_row():
    data = pd.DataFrame({
        "scheduled_rollout_id": ["t001", "t001"],
        "analysis_eligible": [True, True],
    })
    import pytest
    with pytest.raises(ValueError, match="multiple eligible attempts"):
        validate_analysis_set(data)

    with pytest.raises(ValueError, match="expected 480"):
        validate_analysis_set(data.iloc[[0]], expected_eligible=480)
