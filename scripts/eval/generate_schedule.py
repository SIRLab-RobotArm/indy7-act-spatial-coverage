#!/usr/bin/env python3
"""Generate the immutable, session-balanced evaluation order."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from ai.eval.protocol import SCHEDULE_REPEATS, build_balanced_schedule


FULL_FIELDS = (
    "trial", "session", "model_code", "condition", "seed",
    "position_id", "repeat", "rollout_id",
)
BLINDED_FIELDS = (
    "trial", "session", "model_code", "position_id", "repeat", "rollout_id",
)


def _write_csv(path: Path, rows: list[dict], fields: tuple[str, ...]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=fields, extrasaction="ignore", lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def generate(output_dir: Path, seed: int, repeat_count: int = SCHEDULE_REPEATS) -> dict:
    rows = build_balanced_schedule(seed, repeat_count)
    output_dir.mkdir(parents=True, exist_ok=True)
    full_path = output_dir / "schedule_full.csv"
    blinded_path = output_dir / "schedule_blinded.csv"
    key_path = output_dir / "model_key.json"
    manifest_path = output_dir / "manifest.json"
    _write_csv(full_path, rows, FULL_FIELDS)
    _write_csv(blinded_path, rows, BLINDED_FIELDS)
    model_key = {
        row["model_code"]: {
            "condition": row["condition"], "seed": row["seed"]
        }
        for row in rows
    }
    key_path.write_text(
        json.dumps(model_key, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    digest = hashlib.sha256(full_path.read_bytes()).hexdigest()
    manifest = {
        "schema_version": 2,
        "schedule_seed": int(seed),
        "repeat_count": int(repeat_count),
        "trial_count": len(rows),
        "session_count": len({row["session"] for row in rows}),
        "trials_per_session": 24,
        "full_schedule_sha256": digest,
        "balance": (
            "per 24-trial session: each model x2, condition x6, seed x8, "
            "position x3; overall: each model-position-repeat x1"
        ),
        "amendment": (
            "repeat count increased from 2 to 5 after main trials 1-3; "
            "the original 192-trial schedule is preserved as an exact prefix"
        ),
    }
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return manifest


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("experiment/evaluation/main_recollection_20260817"),
    )
    parser.add_argument("--seed", type=int, default=20260824)
    parser.add_argument("--repeat-count", type=int, default=SCHEDULE_REPEATS)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    print(json.dumps(
        generate(args.output_dir, args.seed, args.repeat_count),
        indent=2, sort_keys=True))
