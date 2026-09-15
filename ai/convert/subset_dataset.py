#!/usr/bin/env python3
"""Materialize an RGB-only episode subset and recompute LeRobot statistics."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import torch

from lerobot.datasets.lerobot_dataset import LeRobotDataset

from ai.convert.build_dataset import directory_sha256


AUTO_FEATURES = {"timestamp", "frame_index", "episode_index", "index", "task_index"}


def tensor_bytes(value) -> bytes:
    if isinstance(value, torch.Tensor):
        value = value.detach().cpu().numpy()
    return np.ascontiguousarray(value).tobytes()


def update_content_hash(digest, sample, camera_keys) -> None:
    for key in [*camera_keys, "observation.state", "action"]:
        digest.update(key.encode())
        digest.update(tensor_bytes(sample[key]))


def parse_indices(text: str) -> list[int]:
    result = sorted({int(item.strip()) for item in text.split(",") if item.strip()})
    if not result:
        raise ValueError("at least one episode index is required")
    return result


def build_subset(args: argparse.Namespace) -> dict:
    source_root = Path(args.source_dir).resolve()
    output_root = Path(args.output_dir).resolve()
    if output_root.exists():
        raise FileExistsError("subset output directory must not already exist")

    selected_uuids = []
    if args.episodes:
        selected = parse_indices(args.episodes)
    else:
        if not args.source_manifest or not args.episode_uuids:
            raise ValueError("use --episodes or both --source-manifest and --episode-uuids")
        selected_uuids = [item.strip() for item in args.episode_uuids.split(",") if item.strip()]
        source_manifest = json.loads(Path(args.source_manifest).read_text(encoding="utf-8"))
        selected = sorted(
            int(item["dataset_episode_index"])
            for item in source_manifest["derived_episodes"]
            if item["episode_uuid"] in selected_uuids
        )
        if not selected:
            raise ValueError("no derived episodes matched --episode-uuids")

    source = LeRobotDataset(args.source_repo_id, root=source_root)
    if selected[-1] >= source.num_episodes:
        raise IndexError(f"episode {selected[-1]} outside source range 0:{source.num_episodes}")

    features = {key: value for key, value in source.meta.features.items() if key not in AUTO_FEATURES}
    subset = LeRobotDataset.create(
        repo_id=args.output_repo_id,
        fps=source.fps,
        features=features,
        root=output_root,
        robot_type=source.meta.robot_type,
        use_videos=False,
    )
    source_digest = hashlib.sha256()
    subset_digest = hashlib.sha256()
    old_to_new = {old: new for new, old in enumerate(selected)}
    frames_per_episode = {old: 0 for old in selected}
    frame_count = 0
    for old_episode in selected:
        episode_meta = source.meta.episodes[old_episode]
        start = int(episode_meta["dataset_from_index"])
        stop = int(episode_meta["dataset_to_index"])
        for source_index in range(start, stop):
            sample = source[source_index]
            if int(sample["episode_index"]) != old_episode:
                raise AssertionError(
                    f"episode metadata range mismatch at frame {source_index}"
                )
            update_content_hash(source_digest, sample, source.meta.camera_keys)
            frame = {"task": sample["task"]}
            for key in features:
                value = sample[key]
                if key in source.meta.camera_keys:
                    value = (
                        value.detach().cpu().clamp(0, 1).mul(255).round().to(torch.uint8)
                        .permute(1, 2, 0).numpy()
                    )
                elif isinstance(value, torch.Tensor):
                    value = value.detach().cpu().numpy()
                frame[key] = value
            subset.add_frame(frame)
            frames_per_episode[old_episode] += 1
            frame_count += 1
        if frames_per_episode[old_episode] != stop - start:
            raise AssertionError(f"frame count mismatch for source episode {old_episode}")
        subset.save_episode()
    subset.finalize()

    check = LeRobotDataset(args.output_repo_id, root=output_root)
    for sample_index in range(len(check)):
        update_content_hash(subset_digest, check[sample_index], check.meta.camera_keys)
    if source_digest.hexdigest() != subset_digest.hexdigest():
        raise AssertionError("subset RGB/state/action content hash differs from selected source rows")
    manifest = {
        "source_dataset": str(source_root),
        "source_directory_sha256": directory_sha256(source_root),
        "selected_episode_indices": selected,
        "selected_source_episode_uuids": selected_uuids,
        "episode_reindex": old_to_new,
        "frame_count": frame_count,
        "content_sha256": source_digest.hexdigest(),
        "output_directory_sha256": directory_sha256(output_root),
        "stats_recomputed_from_subset": True,
        "storage_note": (
            "LeRobot v3 embeds multiple episodes' PNG bytes in shared parquet files; "
            "selected rows are rewritten losslessly instead of hardlinked."
        ),
    }
    manifest_path = Path(args.manifest)
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return manifest


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-dir", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--episodes", default="", help="comma-separated derived episode indices")
    parser.add_argument("--source-manifest", default="")
    parser.add_argument("--episode-uuids", default="", help="comma-separated raw episode UUIDs")
    parser.add_argument("--source-repo-id", default="sirlab/grip_all")
    parser.add_argument("--output-repo-id", default="sirlab/grip_subset")
    return parser.parse_args()


if __name__ == "__main__":
    print(json.dumps(build_subset(parse_args()), indent=2, sort_keys=True))
