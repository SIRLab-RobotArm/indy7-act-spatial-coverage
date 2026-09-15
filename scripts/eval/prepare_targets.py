#!/usr/bin/env python3
"""Populate evaluation targets without collecting demonstrations at Q1~Q4.

G1~G9 are the mean EEF pose at the first 0->1 gripper transition in every
accepted demonstration at that grid location.  Each Q target is the cell
centre obtained by equal-weight interpolation of its four surrounding grid
targets.  The robot is therefore never manually guided to or demonstrated at
Q1~Q4 before evaluation.
"""

from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

import h5py
import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from ai.eval.protocol import (
    EVAL_TO_GRID,
    GRID_IDS,
    HOLD_DURATION_S,
    LIFT_THRESHOLD_MM,
    evaluation_distances,
)


Q_SURROUNDING_GRIDS = {
    "eval_5": ("grid_1", "grid_2", "grid_4", "grid_5"),
    "eval_6": ("grid_2", "grid_3", "grid_5", "grid_6"),
    "eval_7": ("grid_4", "grid_5", "grid_7", "grid_8"),
    "eval_8": ("grid_5", "grid_6", "grid_8", "grid_9"),
}


def first_close_index(gripper: np.ndarray) -> int:
    values = np.asarray(gripper, dtype=np.float64).reshape(-1)
    transitions = np.flatnonzero((values[:-1] < 0.5) & (values[1:] >= 0.5)) + 1
    if len(transitions) != 1:
        raise ValueError(f"expected exactly one gripper close, got {len(transitions)}")
    return int(transitions[0])


def mean_pose(poses: np.ndarray) -> list[float]:
    result = np.mean(poses[:, :3], axis=0).tolist()
    angles = np.unwrap(np.deg2rad(poses[:, 3:6]), axis=0)
    averaged = np.rad2deg(np.mean(angles, axis=0))
    averaged = (averaged + 180.0) % 360.0 - 180.0
    return [float(value) for value in result + averaged.tolist()]


def interpolated_q_entry(positions: dict, source_grids: tuple[str, ...]) -> dict:
    """Return a cell-centre target derived only from surrounding grid data."""
    poses = np.asarray(
        [positions[grid_id]["ee_pose_mm_deg"] for grid_id in source_grids],
        dtype=np.float64,
    )
    if poses.shape != (4, 6):
        raise ValueError(f"expected four six-value grid poses, got {poses.shape}")
    return {
        "ee_pose_mm_deg": mean_pose(poses),
        "taught": False,
        "target_defined": True,
        "physically_taught": False,
        "source": {
            "method": "surrounding_grid_cell_center_mean",
            "source_grids": list(source_grids),
            "weights": [0.25, 0.25, 0.25, 0.25],
            "q_demonstration_count": 0,
        },
    }


def grid_entry(directory: Path, expected_count: int) -> tuple[dict, dict]:
    paths = sorted(directory.glob("episode_*.h5"))
    if len(paths) != expected_count:
        raise ValueError(
            f"{directory}: expected {expected_count} episodes, found {len(paths)}")
    poses = []
    joints = []
    episode_uuids = []
    for path in paths:
        with h5py.File(path, "r") as episode:
            if str(episode.attrs.get("success_label", "")) != "success":
                raise ValueError(f"non-success episode in main dataset: {path}")
            close_index = first_close_index(episode["/state/gripper_cmd"][:])
            poses.append(np.asarray(episode["/state/ee_pose"][close_index], dtype=np.float64))
            joint = np.asarray(
                episode["/state/joint_pos"][close_index], dtype=np.float64).reshape(-1)
            if len(joint) < 6:
                raise ValueError(f"fewer than six joints in {path}")
            joints.append(joint[:6])
            episode_uuids.append(str(episode.attrs["episode_uuid"]))

    pose_matrix = np.stack(poses)
    xyz = pose_matrix[:, :3]
    average = np.asarray(mean_pose(pose_matrix), dtype=np.float64)
    xyz_offsets = xyz - average[:3]
    xyz_distances = np.linalg.norm(xyz_offsets, axis=1)
    uuid_digest = hashlib.sha256(
        "\n".join(sorted(episode_uuids)).encode("utf-8")
    ).hexdigest()
    entry = {
        "ee_pose_mm_deg": average.tolist(),
        "taught": True,
        "target_defined": True,
        "source": {
            "method": "successful_demonstration_first_close_mean",
            "close_definition": "first gripper_cmd 0->1 transition",
            "episode_count": len(paths),
            "episode_uuid_set_sha256": uuid_digest,
            "xyz_std_mm": np.std(xyz, axis=0).tolist(),
            "xyz_rms_mm": float(np.sqrt(np.mean(xyz_distances ** 2))),
            "xyz_max_dev_mm": float(np.max(xyz_distances)),
            "joint_mean_rad": np.mean(np.stack(joints), axis=0).tolist(),
        },
    }
    summary = {
        "episode_count": len(paths),
        "mean_xyz_mm": average[:3].tolist(),
        "xyz_rms_mm": entry["source"]["xyz_rms_mm"],
        "xyz_max_dev_mm": entry["source"]["xyz_max_dev_mm"],
    }
    return entry, summary


def prepare(episodes_root: Path, positions_path: Path) -> dict:
    document = yaml.safe_load(positions_path.read_text(encoding="utf-8")) or {}
    positions = document.setdefault("positions", {})
    summaries = {}
    for index, grid_id in enumerate(GRID_IDS, start=1):
        expected_count = 60 if index == 5 else 20
        entry, summary = grid_entry(episodes_root / f"p{index}", expected_count)
        positions[grid_id] = entry
        summaries[grid_id] = summary

    for eval_id, grid_id in EVAL_TO_GRID.items():
        positions[eval_id] = {
            "ee_pose_mm_deg": list(positions[grid_id]["ee_pose_mm_deg"]),
            "taught": True,
            "target_defined": True,
            "mirror_of": grid_id,
            "source": deepcopy(positions[grid_id]["source"]),
        }

    for eval_id, source_grids in Q_SURROUNDING_GRIDS.items():
        positions[eval_id] = interpolated_q_entry(positions, source_grids)

    document["schema_version"] = 3
    document["evaluation_protocol"] = {
        "p_target_definition": (
            "exact-grid: mean EEF XYZ at first gripper close; "
            "Q cell-centre: equal-weight mean of four surrounding grid targets"
        ),
        "q_target_has_physical_teach_in": False,
        "distance_definition": "nearest condition training target, XY Euclidean mm",
        "lift_threshold_mm": LIFT_THRESHOLD_MM,
        "hold_duration_s": HOLD_DURATION_S,
        "success_definition": (
            "PVC grasped, EEF rises at least 50 mm after close, "
            "and PVC remains held for at least 3 s"
        ),
        "updated_utc": datetime.now(timezone.utc).isoformat(),
    }
    try:
        document["evaluation_distances_mm"] = evaluation_distances(document)
        distances_ready = True
    except ValueError:
        document.pop("evaluation_distances_mm", None)
        distances_ready = False

    temporary = positions_path.with_suffix(positions_path.suffix + ".tmp")
    temporary.write_text(
        yaml.safe_dump(document, sort_keys=False, allow_unicode=True), encoding="utf-8")
    temporary.replace(positions_path)
    return {"grids": summaries, "evaluation_distances_ready": distances_ready}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--episodes-root",
        default="episodes/main_20260817",
        type=Path,
    )
    parser.add_argument(
        "--positions",
        default="experiment/positions.yaml",
        type=Path,
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    result = prepare(args.episodes_root.resolve(), args.positions.resolve())
    print(json.dumps(result, indent=2, sort_keys=True))
