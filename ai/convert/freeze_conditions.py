#!/usr/bin/env python3
"""Freeze deterministic A--D raw-episode membership before model training."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


EXPECTED_COUNTS = {
    "p1": 20,
    "p2": 20,
    "p3": 20,
    "p4": 20,
    "p5": 60,
    "p6": 20,
    "p7": 20,
    "p8": 20,
    "p9": 20,
}
CONDITION_C_COUNTS = {
    "p1": 7,
    "p2": 6,
    "p3": 7,
    "p4": 7,
    "p5": 6,
    "p6": 7,
    "p7": 7,
    "p8": 6,
    "p9": 7,
}


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def position_from_episode(item: dict) -> str:
    position = Path(item["path"]).parent.name.lower()
    if position not in EXPECTED_COUNTS:
        raise ValueError(f"cannot infer P1--P9 position from {item['path']}")
    return position


def deterministic_rank(seed: str, position: str, episode_uuid: str) -> str:
    payload = f"{seed}:{position}:{episode_uuid}".encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def freeze_membership(source_manifest: dict, selection_seed: str) -> dict[str, dict]:
    grouped = {position: [] for position in EXPECTED_COUNTS}
    source_index_by_uuid = {}
    source_path_by_uuid = {}
    for source_index, item in enumerate(source_manifest["episodes"]):
        episode_uuid = str(item["episode_uuid"])
        if episode_uuid in source_index_by_uuid:
            raise ValueError(f"duplicate episode UUID: {episode_uuid}")
        position = position_from_episode(item)
        grouped[position].append(episode_uuid)
        source_index_by_uuid[episode_uuid] = source_index
        source_path_by_uuid[episode_uuid] = str(item["path"])

    actual_counts = {position: len(values) for position, values in grouped.items()}
    if actual_counts != EXPECTED_COUNTS:
        raise ValueError(
            f"source position counts differ from frozen design: {actual_counts}"
        )

    ranked = {
        position: sorted(
            values,
            key=lambda episode_uuid: (
                deterministic_rank(selection_seed, position, episode_uuid),
                episode_uuid,
            ),
        )
        for position, values in grouped.items()
    }

    # D uses every non-P5 episode and a deterministic 20-of-60 P5 selection.
    d_by_position = {
        position: values[:20]
        for position, values in ranked.items()
    }
    # C is deliberately nested inside D so C-vs-D changes count, not membership.
    c_by_position = {
        position: d_by_position[position][:count]
        for position, count in CONDITION_C_COUNTS.items()
    }
    condition_positions = {
        "a": {"p5": ranked["p5"]},
        "b": {position: ranked[position] for position in ("p1", "p6", "p8")},
        "c": c_by_position,
        "d": d_by_position,
    }

    derived_by_uuid: dict[str, list[int]] = {}
    for item in source_manifest["derived_episodes"]:
        derived_by_uuid.setdefault(str(item["episode_uuid"]), []).append(
            int(item["dataset_episode_index"])
        )

    result = {}
    for condition, by_position in condition_positions.items():
        ordered_uuids = [
            episode_uuid
            for position in EXPECTED_COUNTS
            for episode_uuid in by_position.get(position, [])
        ]
        membership_digest = hashlib.sha256(
            ("\n".join(ordered_uuids) + "\n").encode("utf-8")
        ).hexdigest()
        records = [
            {
                "episode_uuid": episode_uuid,
                "position": position_from_episode(
                    source_manifest["episodes"][source_index_by_uuid[episode_uuid]]
                ),
                "source_episode_index": source_index_by_uuid[episode_uuid],
                "source_path": source_path_by_uuid[episode_uuid],
                "selection_rank_sha256": deterministic_rank(
                    selection_seed,
                    position_from_episode(
                        source_manifest["episodes"][source_index_by_uuid[episode_uuid]]
                    ),
                    episode_uuid,
                ),
                "derived_episode_indices": sorted(derived_by_uuid[episode_uuid]),
            }
            for episode_uuid in ordered_uuids
        ]
        result[condition] = {
            "schema_version": 1,
            "condition": condition.upper(),
            "selection_seed": selection_seed,
            "selection_algorithm": "sha256(seed:position:episode_uuid), ascending",
            "nested_membership": "C is a subset of D" if condition in {"c", "d"} else None,
            "counts_by_position": {
                position: len(by_position.get(position, []))
                for position in EXPECTED_COUNTS
            },
            "raw_episode_count": len(ordered_uuids),
            "derived_episode_count": sum(
                len(derived_by_uuid[episode_uuid]) for episode_uuid in ordered_uuids
            ),
            "episode_uuids": ordered_uuids,
            "membership_sha256": membership_digest,
            "episodes": records,
        }
    if not set(result["c"]["episode_uuids"]).issubset(result["d"]["episode_uuids"]):
        raise AssertionError("condition C must be a subset of condition D")
    return result


def write_manifests(args: argparse.Namespace) -> dict[str, Path]:
    source_path = Path(args.source_manifest).resolve()
    source = json.loads(source_path.read_text(encoding="utf-8"))
    frozen = freeze_membership(source, args.selection_seed)
    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    outputs = {}
    for condition, payload in frozen.items():
        output = output_dir / f"{args.prefix}_condition_{condition}_uuids.json"
        if output.exists():
            raise FileExistsError(f"refusing to overwrite frozen manifest: {output}")
        document = {
            "source_manifest": str(source_path),
            "source_manifest_sha256": sha256_file(source_path),
            "source_dataset_directory_sha256": source["dataset_directory_sha256"],
            **payload,
        }
        output.write_text(
            json.dumps(document, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        outputs[condition] = output
    return outputs


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-manifest", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--prefix", required=True)
    parser.add_argument("--selection-seed", required=True)
    return parser.parse_args()


if __name__ == "__main__":
    paths = write_manifests(parse_args())
    print(json.dumps({key: str(value) for key, value in paths.items()}, indent=2))
