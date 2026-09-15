#!/usr/bin/env python3
"""Convert raw synchronized HDF5 episodes into one RGB LeRobot dataset."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess

import cv2
import h5py
import numpy as np

from lerobot.datasets.lerobot_dataset import LeRobotDataset

CAMERA_KEY = "observation.images.overhead"
STATE_NAMES = ["ee_x_mm", "ee_y_mm", "ee_z_mm"] + [f"joint_{i}_rad" for i in range(6)] + [
    "gripper_cmd"
]
ACTION_NAMES = ["delta_x_mm", "delta_y_mm", "delta_z_mm", "gripper_cmd"]


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git_sha() -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], text=True, stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def directory_sha256(root: Path) -> str:
    digest = hashlib.sha256()
    for path in sorted(item for item in root.rglob("*") if item.is_file()):
        digest.update(path.relative_to(root).as_posix().encode())
        digest.update(bytes.fromhex(sha256_file(path)))
    return digest.hexdigest()


def discover_episodes(root: Path) -> list[Path]:
    return sorted(path for path in root.rglob("episode_*.h5") if ".partial." not in path.name)


def inspect_episode(path: Path, action_rate: float, max_jitter_sec: float) -> dict:
    with h5py.File(path, "r") as episode:
        if "action" in episode:
            raise ValueError(f"raw action must not exist: {path}")
        required = (
            "/obs/rgb", "/state/ee_pose", "/state/joint_pos",
            "/state/gripper_cmd", "/time/stamp_rgb", "/time/stamp_joint",
            "/time/stamp_ee", "/time/stamp_recv",
        )
        missing = [key for key in required if key not in episode]
        if missing:
            raise ValueError(f"missing HDF5 fields in {path}: {missing}")
        count = int(episode["/obs/rgb"].shape[0])
        lengths = {key: int(episode[key].shape[0]) for key in required}
        if any(length != count for length in lengths.values()):
            raise ValueError(f"frame count mismatch in {path}: {lengths}")
        record_hz = float(episode.attrs["record_hz"])
        stride_float = record_hz / action_rate
        stride = int(round(stride_float))
        if stride < 1 or not np.isclose(stride_float, stride, atol=1e-9):
            raise ValueError(f"action_rate={action_rate} must divide record_hz={record_hz}")
        rgb_stamps = np.asarray(episode["/time/stamp_rgb"], dtype=np.float64)
        joint_stamps = np.asarray(episode["/time/stamp_joint"], dtype=np.float64)
        ee_stamps = np.asarray(episode["/time/stamp_ee"], dtype=np.float64)
        expected_period = 1.0 / record_hz
        joint_jitter = np.abs(np.diff(joint_stamps) - expected_period)
        ee_jitter = np.abs(np.diff(ee_stamps) - expected_period)
        max_jitter = float(max(
            joint_jitter.max(initial=0.0),
            ee_jitter.max(initial=0.0),
        ))
        if max_jitter > max_jitter_sec:
            raise ValueError(
                f"robot-state timestamp jitter {max_jitter:.6f}s exceeds "
                f"{max_jitter_sec:.6f}s in {path}")
        max_sync_offset = float(max(
            np.abs(rgb_stamps - joint_stamps).max(initial=0.0),
            np.abs(rgb_stamps - ee_stamps).max(initial=0.0),
        ))
        sync_slop = float(episode.attrs.get("sync_slop_sec", 0.03))
        if max_sync_offset > sync_slop + 1e-6:
            raise ValueError(
                f"RGB/robot sync offset {max_sync_offset:.6f}s exceeds "
                f"sync_slop={sync_slop:.6f}s in {path}")
        rgb_shape = tuple(int(value) for value in episode["/obs/rgb"].shape[1:])
        if len(rgb_shape) != 3 or rgb_shape[-1] != 3:
            raise ValueError(f"expected HWC RGB in {path}, got {rgb_shape}")
        return {
            "path": path,
            "count": count,
            "sample_count": max(0, count - stride),
            "stride": stride,
            "record_hz": record_hz,
            "max_jitter_sec": max_jitter,
            "max_sync_offset_sec": max_sync_offset,
            "image_shape": rgb_shape[:2],
            "episode_uuid": str(episode.attrs["episode_uuid"]),
            "data_block": str(episode.attrs["data_block"]),
        }


def build(args: argparse.Namespace) -> dict:
    episodes_root = Path(args.episodes_dir).resolve()
    output = Path(args.output_dir).resolve()
    paths = discover_episodes(episodes_root)
    if args.max_episodes:
        paths = paths[: args.max_episodes]
    if not paths:
        raise FileNotFoundError(f"no episode_*.h5 under {episodes_root}")
    if output.exists():
        raise FileExistsError("output directory must not already exist")

    # The camera set is fixed before opening the first episode.
    inspected = [inspect_episode(path, args.action_rate, args.max_jitter_sec) for path in paths]
    excluded_short = [item for item in inspected if item["sample_count"] == 0]
    episode_info = [item for item in inspected if item["sample_count"] > 0]
    if not episode_info:
        raise ValueError("no episode is long enough for one action interval")
    image_shapes = {item["image_shape"] for item in episode_info}
    record_rates = {item["record_hz"] for item in episode_info}
    if len(image_shapes) != 1:
        raise ValueError("all episodes must share one RGB image shape")
    if record_rates != {20.0}:
        raise ValueError(f"the preregistered observation rate is 20 Hz, got {record_rates}")
    sample_count = sum(item["sample_count"] for item in episode_info)
    source_hashes = {item["episode_uuid"]: sha256_file(item["path"]) for item in episode_info}

    features = {
        CAMERA_KEY: {
            "dtype": "image",
            "shape": (args.image_height, args.image_width, 3),
            "names": ["height", "width", "channels"],
        },
        "observation.state": {
            "dtype": "float32", "shape": (10,), "names": STATE_NAMES,
        },
        "action": {
            "dtype": "float32", "shape": (4,), "names": ACTION_NAMES,
        },
    }
    if not float(args.action_rate).is_integer():
        raise ValueError("LeRobot metadata requires an integer action rate")
    dataset = LeRobotDataset.create(
        repo_id=args.repo_id,
        fps=int(args.action_rate),
        features=features,
        root=output,
        robot_type="indy7_mark7",
        use_videos=False,
    )
    qa_episodes = []
    derived_episodes = []
    dataset_episode_index = 0
    for source_episode_index, info in enumerate(episode_info):
        with h5py.File(info["path"], "r") as episode:
            for phase in range(info["stride"]):
                source_frames = list(range(phase, info["sample_count"], info["stride"]))
                if not source_frames:
                    continue
                for phase_frame_index, source_frame_index in enumerate(source_frames):
                    target = source_frame_index + info["stride"]
                    rgb = np.asarray(episode["/obs/rgb"][source_frame_index], dtype=np.uint8)
                    rgb = cv2.resize(
                        rgb, (args.image_width, args.image_height), interpolation=cv2.INTER_AREA)
                    ee = np.asarray(
                        episode["/state/ee_pose"][source_frame_index], dtype=np.float32)
                    joint = np.asarray(
                        episode["/state/joint_pos"][source_frame_index], dtype=np.float32)
                    gripper = float(episode["/state/gripper_cmd"][source_frame_index])
                    state = np.concatenate((ee[:3], joint[:6], [gripper])).astype(np.float32)
                    delta = (
                        np.asarray(episode["/state/ee_pose"][target, :3], dtype=np.float32)
                        - ee[:3]
                    )
                    target_gripper = float(episode["/state/gripper_cmd"][target])
                    action = np.concatenate((delta, [target_gripper])).astype(np.float32)
                    dataset.add_frame({
                        "task": args.task,
                        CAMERA_KEY: rgb,
                        "observation.state": state,
                        "action": action,
                    })
                dataset.save_episode()
                derived_episodes.append({
                    "dataset_episode_index": dataset_episode_index,
                    "source_episode_index": source_episode_index,
                    "episode_uuid": info["episode_uuid"],
                    "phase": phase,
                    "frame_count": len(source_frames),
                })
                dataset_episode_index += 1
        qa_episodes.append({
            key: value for key, value in info.items()
            if key not in {"path", "image_shape"}
        } | {"path": str(info["path"]), "sha256": source_hashes[info["episode_uuid"]]})
    dataset.finalize()

    manifest = {
        "schema_version": 1,
        "converter_git_sha": git_sha(),
        "vendored_lerobot_baseline_commit": "59fc197",
        "argv": vars(args),
        "camera_key": CAMERA_KEY,
        "action_semantics": (
            "ee_pose[t+stride,:3]-ee_pose[t,:3], gripper_cmd[t+stride]"
        ),
        "action_rate_hz": float(args.action_rate),
        "observation_rate_hz": 20,
        "phase_split": True,
        "frame_count": sample_count,
        "episodes": qa_episodes,
        "derived_episodes": derived_episodes,
        "excluded_short_episodes": [
            {"path": str(item["path"]), "episode_uuid": item["episode_uuid"]}
            for item in excluded_short
        ],
        "source_h5_sha256": source_hashes,
        "dataset_directory_sha256": directory_sha256(output),
    }
    manifest_path = Path(args.manifest).resolve()
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return manifest


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--episodes-dir", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--repo-id", default="sirlab/grip_all")
    parser.add_argument("--task", default="grasp and lift the cylinder")
    parser.add_argument("--action-rate", type=float, default=5.0)
    parser.add_argument("--max-jitter-sec", type=float, default=0.015)
    parser.add_argument("--image-height", type=int, default=240)
    parser.add_argument("--image-width", type=int, default=320)
    parser.add_argument("--max-episodes", type=int, default=0)
    return parser.parse_args()


if __name__ == "__main__":
    result = build(parse_args())
    print(json.dumps({"frame_count": result["frame_count"]}, sort_keys=True))
