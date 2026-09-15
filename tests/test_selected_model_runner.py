from pathlib import Path

import pytest

from scripts.eval.model_run_ui import DirectRunOutputParser
from scripts.eval.run_selected_model import POSITION_NAMES, model_registry


def test_all_models_are_listed_under_their_blind_codes():
    assert list(model_registry()) == [f"M{index:02d}" for index in range(1, 13)]


@pytest.mark.skipif(
    not (Path(__file__).resolve().parents[1] / "ai/models").is_dir(),
    reason="needs the trained checkpoints, which are not distributed with the "
           "repository - see docs/data_and_weights.md")
def test_all_models_resolve_to_existing_single_model_inputs():
    for entry in model_registry().values():
        assert Path(entry["model"]).is_dir()
        assert Path(entry["dataset_root"]).is_dir()


def test_direct_run_offers_grid_and_quadrant_positions():
    assert POSITION_NAMES == (
        "G1", "G2", "G3", "G4", "G5", "G6", "G7", "G8", "G9",
        "Q1", "Q2", "Q3", "Q4",
    )


def test_direct_run_parser_handles_split_prompt_and_done_marker():
    parser = DirectRunOutputParser()

    assert parser.feed("DIRECT_RUN_PRO") is None
    assert parser.feed("MPT:placed\n") == "placed"
    parser.answer_sent()
    assert parser.prompt is None
    assert parser.feed("DIRECT_RUN_PROMPT:stop\n") == "stop"
    assert parser.feed("DIRECT_RUN_DONE\n") is None


def test_eval_launch_can_disable_all_recorders():
    launch_text = (
        Path(__file__).parents[1]
        / "ros2_ws/src/grip_bringup/launch/eval.launch.py"
    ).read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("record", default_value="true")' in launch_text
    assert launch_text.count("record, \"' == 'true'") == 3
