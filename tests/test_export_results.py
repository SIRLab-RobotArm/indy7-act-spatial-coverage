import csv
import json
from pathlib import Path

import pandas as pd

from scripts.eval import export_results


def _write_schedule(path: Path):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=["trial", "rollout_id"])
        writer.writeheader()
        writer.writerow({"trial": 1, "rollout_id": "t001"})
        writer.writerow({"trial": 2, "rollout_id": "t002"})


def test_select_attempts_keeps_one_validated_attempt(monkeypatch, tmp_path):
    schedule = tmp_path / "schedule.csv"
    _write_schedule(schedule)
    rollout_root = tmp_path / "rollouts"
    retry = rollout_root / "t001_retry1"
    valid = rollout_root / "t001"
    retry.mkdir(parents=True)
    valid.mkdir()

    monkeypatch.setattr(
        export_results.runner,
        "_used_attempts",
        lambda _root, rollout_id: (
            [(1, retry), (0, valid)] if rollout_id == "t001" else []
        ),
    )
    monkeypatch.setattr(
        export_results.runner,
        "_completed_disposition",
        lambda path: "eligible" if path == valid else None,
    )

    rows, selected, audit = export_results.select_attempts(
        schedule, rollout_root
    )

    assert len(rows) == 2
    assert selected == [(rows[0], valid)]
    assert audit["disposition"].tolist() == [
        "incomplete_or_invalid", "eligible"
    ]


def test_export_writes_compact_provisional_tables(monkeypatch, tmp_path):
    schedule = tmp_path / "schedule.csv"
    _write_schedule(schedule)
    rollout_root = tmp_path / "rollouts"
    valid = rollout_root / "t001"
    valid.mkdir(parents=True)

    monkeypatch.setattr(
        export_results.runner,
        "_used_attempts",
        lambda _root, rollout_id: [(0, valid)] if rollout_id == "t001" else [],
    )
    monkeypatch.setattr(
        export_results.runner,
        "_completed_disposition",
        lambda _path: "eligible",
    )
    monkeypatch.setattr(
        export_results.analyze,
        "load_rollout",
        lambda _path: {
            "scheduled_rollout_id": "t001",
            "condition": "A",
            "seed": 0,
            "model_code": "M01",
            "position_id": "eval_1",
            "evaluation_group": "exact_grid",
            "success": True,
            "E1": 1.0,
            "E2": -2.0,
            "E3": 0.5,
            "close_error_3d_mm": 2.25,
            "outcome_corrected": False,
            "p_target": [1, 2, 3],
            "outcome_correction": {},
        },
    )

    output = tmp_path / "results"
    summary = export_results.export(schedule, rollout_root, output)

    assert summary["completed_trials"] == 1
    assert summary["remaining_trials"] == 1
    assert summary["next_trial"] == 2
    assert summary["provisional"] is True
    assert summary["success"]["by_condition"][0]["success_rate"] == 1.0
    assert Path(summary["schedule"]) == schedule
    assert len(pd.read_csv(output / "rollouts.csv")) == 1
    assert "p_target" not in pd.read_csv(output / "rollouts.csv").columns
    assert json.loads((output / "summary.json").read_text())["next_trial"] == 2
