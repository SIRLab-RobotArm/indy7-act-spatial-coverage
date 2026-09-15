from scripts.eval.evaluation_ui import EvaluationOutputParser, gripper_open_allowed


def test_parser_extracts_case_position_and_split_prompt():
    parser = EvaluationOutputParser()
    parser.feed(
        "Trial 17/480 | attempt 1 | session 1 | M05 (condition A, seed 2)\n"
        "Evaluation position: eval_6 = Q2 (centre of G2/G3/G5/G6) | repeat 3/5\n")
    parser.feed("Stand the PVC pipe upright, do not touch the board or camera, then type PLA")
    state = parser.feed("CED: ")

    assert state.trial == 17
    assert state.trial_count == 480
    assert state.model == "M05"
    assert state.condition == "A"
    assert state.seed == "2"
    assert state.position_id == "eval_6"
    assert state.position == "Q2"
    assert state.repeat == "3/5"
    assert state.prompt == "placed"


def test_parser_extracts_grasp_metrics_and_clears_answered_prompt():
    parser = EvaluationOutputParser()
    state = parser.feed("Outcome (S=success, F=failure)> ")
    assert state.prompt == "outcome"
    parser.answer_sent()

    state = parser.feed(
        "Grasp position:\n"
        "- mean first-close pose of the training demonstrations XYZ: [640.471, -15.396, 552.202] mm\n"
        "- Measured first-close EEF XYZ: [643.082, -15.744, 552.565] mm\n"
        "- Signed error XYZ (measured - reference): [2.611, -0.348, 0.363] mm\n"
        "- XY distance error: 2.634 mm\n"
        "- 3-D distance error: 2.659 mm\n")

    assert state.prompt is None
    assert state.target_xyz == "[640.471, -15.396, 552.202]"
    assert state.grasp_xyz == "[643.082, -15.744, 552.565]"
    assert state.delta_xyz == "[2.611, -0.348, 0.363]"
    assert state.xy_error == "2.634"
    assert state.error_3d == "2.659"

def test_parser_recognizes_next_trial_prompt():
    parser = EvaluationOutputParser()
    state = parser.feed(
        "Enter = continue to the next trial in this terminal (from HOME and OPEN), "
        "Q = quit safely: ")

    assert state.prompt == "next"


def test_gripper_button_is_scoped_to_safe_pre_start_prompts():
    assert gripper_open_allowed("home")
    assert gripper_open_allowed("open")
    assert gripper_open_allowed("placed")
    assert gripper_open_allowed("start")
    assert gripper_open_allowed("release")
    assert not gripper_open_allowed("outcome")
    assert not gripper_open_allowed("next")


def test_parser_recognizes_post_result_release_prompt():
    parser = EvaluationOutputParser()
    state = parser.feed(
        "Outcome recorded: success\n"
        "Support the pipe by hand, then type RELEASE to open the gripper: ")

    assert state.prompt == "release"


def test_parser_reports_resident_model_pool_loading_progress():
    parser = EvaluationOutputParser()
    state = parser.feed(
        "ACT pool loading 7/12: M07\n"
        "ACT pool loaded 7/12: M07\n")
    assert state.pool_progress == "loaded 7/12 - M07"

    state = parser.feed(
        "ACT model pool ready at tcp://127.0.0.1:5557 (12 models)\n")
    assert state.pool_progress == "ready, 12 models resident"


def test_parser_clears_outcome_prompt_when_policy_timeout_finalizes_result():
    parser = EvaluationOutputParser()
    assert parser.feed("Outcome (S=success, F=failure)> ").prompt == "outcome"

    state = parser.feed(
        "\nPolicy run reached its 60s limit - recording it as a failure and stopping the policy.\n")

    assert state.prompt is None
