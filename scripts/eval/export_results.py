#!/usr/bin/env python3
"""Export compact, Git-trackable results from large local rollout artifacts."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from ai.eval import analyze
from scripts.eval import run_evaluation as runner


DEFAULT_SCHEDULE = (
    REPO_ROOT
    / "experiment/evaluation/main_recollection_20260817/schedule_full.csv"
)
DEFAULT_ROLLOUTS = (
    REPO_ROOT
    / "experiment/rollouts/main_recollection_20260817_main_smooth30hz"
)
DEFAULT_OUTPUT = (
    REPO_ROOT
    / "experiment/evaluation/main_recollection_20260817/results"
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _display_path(path: Path) -> str:
    """Prefer a repository-relative provenance path without requiring one."""
    try:
        return path.relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return str(path)


def select_attempts(schedule: Path, rollout_root: Path):
    """Select exactly one validated eligible attempt per completed schedule row."""
    with schedule.open(newline="", encoding="utf-8") as stream:
        schedule_rows = list(csv.DictReader(stream))

    selected: list[tuple[dict, Path]] = []
    audit: list[dict] = []
    for row in schedule_rows:
        eligible: list[tuple[int, Path]] = []
        for attempt, path in runner._used_attempts(
            rollout_root, row["rollout_id"]
        ):
            disposition = runner._completed_disposition(path)
            audit.append({
                "trial": int(row["trial"]),
                "scheduled_rollout_id": row["rollout_id"],
                "attempt": attempt,
                "attempt_directory": path.name,
                "disposition": disposition or "incomplete_or_invalid",
            })
            if disposition == "eligible":
                eligible.append((attempt, path))
        if len(eligible) > 1:
            names = ", ".join(path.name for _, path in eligible)
            raise ValueError(
                f"multiple eligible attempts for {row['rollout_id']}: {names}"
            )
        if eligible:
            selected.append((row, eligible[0][1]))
    return schedule_rows, selected, pd.DataFrame(audit)


def _group_success(data: pd.DataFrame, columns: list[str]) -> list[dict]:
    rows = []
    for keys, group in data.groupby(columns, dropna=False, sort=True):
        if not isinstance(keys, tuple):
            keys = (keys,)
        successes = int(group["success"].sum())
        total = int(len(group))
        low, high = analyze.wilson_interval(successes, total)
        rows.append({
            **dict(zip(columns, keys)),
            "successes": successes,
            "total": total,
            "success_rate": successes / total,
            "wilson_95_low": low,
            "wilson_95_high": high,
        })
    return rows


def _error_summary(data: pd.DataFrame) -> list[dict]:
    rows = []
    for success, group in data.groupby("success", sort=True):
        for metric in ("E1", "E2", "E3", "close_error_3d_mm"):
            values = group[metric].dropna().astype(float)
            rows.append({
                "success": bool(success),
                "metric": metric,
                "n": int(len(values)),
                "mean": None if values.empty else float(values.mean()),
                "median": None if values.empty else float(values.median()),
                "q25": None if values.empty else float(values.quantile(0.25)),
                "q75": None if values.empty else float(values.quantile(0.75)),
            })
    return rows


def export(schedule: Path, rollout_root: Path, output: Path) -> dict:
    schedule_rows, selected, audit = select_attempts(schedule, rollout_root)
    rows = []
    for schedule_row, path in selected:
        row = analyze.load_rollout(path)
        row["trial"] = int(schedule_row["trial"])
        rows.append(row)
    data = pd.DataFrame(rows).sort_values("trial") if rows else pd.DataFrame()

    output.mkdir(parents=True, exist_ok=True)
    rollout_csv = output / "rollouts.csv"
    attempts_csv = output / "attempts.csv"
    summary_json = output / "summary.json"

    if len(data):
        drop_columns = ["p_target", "outcome_correction"]
        public = data.drop(columns=[
            column for column in drop_columns if column in data.columns
        ])
        public.to_csv(rollout_csv, index=False)
    else:
        rollout_csv.write_text("trial\n", encoding="utf-8")
    audit.to_csv(attempts_csv, index=False)

    completed = len(data)
    summary = {
        "schema_version": 1,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "provisional": completed != len(schedule_rows),
        "schedule": _display_path(schedule),
        "schedule_sha256": _sha256(schedule),
        "scheduled_trials": len(schedule_rows),
        "completed_trials": completed,
        "remaining_trials": len(schedule_rows) - completed,
        "next_trial": None if completed == len(schedule_rows) else int(
            next(
                row["trial"] for row in schedule_rows
                if row["rollout_id"] not in set(data["scheduled_rollout_id"])
            )
        ),
        "attempts": {
            "eligible": int((audit["disposition"] == "eligible").sum()),
            "excluded": int((audit["disposition"] == "excluded").sum()),
            "incomplete_or_invalid": int(
                (audit["disposition"] == "incomplete_or_invalid").sum()
            ),
        } if len(audit) else {
            "eligible": 0, "excluded": 0, "incomplete_or_invalid": 0,
        },
        "outcome_corrections": int(data["outcome_corrected"].sum())
        if len(data) else 0,
        "success": {
            "by_condition": _group_success(data, ["condition"]),
            "by_condition_and_group": _group_success(
                data, ["condition", "evaluation_group"]
            ),
            "by_model": _group_success(
                data, ["condition", "seed", "model_code"]
            ),
            "by_position": _group_success(
                data, ["condition", "position_id"]
            ),
        } if len(data) else {},
        "grasp_error": _error_summary(data) if len(data) else [],
        "artifacts": {
            "rollouts_csv": rollout_csv.name,
            "attempts_csv": attempts_csv.name,
        },
    }
    summary_json.write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--schedule", type=Path, default=DEFAULT_SCHEDULE)
    parser.add_argument("--rollouts", type=Path, default=DEFAULT_ROLLOUTS)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--require-complete", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    summary = export(
        args.schedule.resolve(), args.rollouts.resolve(), args.output.resolve()
    )
    print(json.dumps({
        "completed_trials": summary["completed_trials"],
        "remaining_trials": summary["remaining_trials"],
        "output": str(args.output.resolve()),
    }, indent=2, ensure_ascii=False))
    if args.require_complete and summary["provisional"]:
        raise SystemExit("evaluation is not complete")


if __name__ == "__main__":
    main()
