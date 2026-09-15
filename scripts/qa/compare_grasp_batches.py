#!/usr/bin/env python3
"""Compare first-close EEF XYZ clusters between two raw HDF5 batches."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys
from typing import TextIO

import h5py
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from ai.convert.build_dataset import inspect_episode  # noqa: E402


@dataclass(frozen=True)
class CloseRecord:
    path: Path
    close_frame: int
    xyz_mm: np.ndarray


def complete_h5_files(directory: str | Path) -> list[Path]:
    root = Path(directory).resolve()
    if not root.is_dir():
        raise NotADirectoryError(root)
    return sorted(
        path for path in root.glob("*.h5")
        if path.is_file() and ".partial." not in path.name
    )


def read_first_close(path: str | Path, expected_position: str) -> CloseRecord:
    path = Path(path).resolve()
    with h5py.File(path, "r") as episode:
        actual_position = str(episode.attrs.get("position_id", ""))
        if actual_position != expected_position:
            raise ValueError(
                f"position_id={actual_position!r}, expected {expected_position!r}"
            )
        if "/state/gripper_cmd" not in episode or "/state/ee_pose" not in episode:
            raise ValueError("missing /state/gripper_cmd or /state/ee_pose")

        gripper = np.asarray(episode["/state/gripper_cmd"], dtype=np.float32)
        ee_pose = np.asarray(episode["/state/ee_pose"], dtype=np.float64)
        if gripper.ndim != 1:
            raise ValueError(f"gripper_cmd must be 1-D, got {gripper.shape}")
        if ee_pose.ndim != 2 or ee_pose.shape[1] < 3:
            raise ValueError(f"ee_pose must be [T,>=3], got {ee_pose.shape}")
        if len(gripper) != len(ee_pose):
            raise ValueError(
                f"gripper/ee_pose length mismatch: {len(gripper)} != {len(ee_pose)}"
            )

        close_frames = np.flatnonzero(
            (gripper[:-1] < 0.5) & (gripper[1:] >= 0.5)
        ) + 1
        if len(close_frames) != 1:
            raise ValueError(
                f"expected exactly one 0->1 close transition, got {len(close_frames)}"
            )
        close_frame = int(close_frames[0])
        xyz = np.asarray(ee_pose[close_frame, :3], dtype=np.float64)
        if not np.all(np.isfinite(xyz)):
            raise ValueError(f"non-finite close XYZ: {xyz.tolist()}")
        return CloseRecord(path=path, close_frame=close_frame, xyz_mm=xyz)


def candidate_qa(path: Path) -> tuple[bool, str]:
    try:
        info = inspect_episode(path, action_rate=5.0, max_jitter_sec=0.015)
    except Exception as exc:  # Report per file and continue with cluster analysis.
        return False, f"{type(exc).__name__}: {exc}"
    return True, (
        f"frames={info['count']} max_jitter_sec={info['max_jitter_sec']:.6f} "
        f"max_sync_offset_sec={info['max_sync_offset_sec']:.6f}"
    )


def cluster_stats(records: list[CloseRecord]) -> tuple[np.ndarray, np.ndarray]:
    if not records:
        raise ValueError("no valid close records")
    values = np.stack([record.xyz_mm for record in records])
    return values.mean(axis=0), values.std(axis=0, ddof=0)


def _xyz_text(xyz: np.ndarray) -> str:
    return "(" + ", ".join(f"{value:.3f}" for value in xyz) + ")"


def compare_batches(
    reference_dir: str | Path,
    candidate_dir: str | Path,
    expected_position: str = "grid_5",
    stream: TextIO | None = None,
) -> dict:
    stream = stream or sys.stdout
    reference_paths = complete_h5_files(reference_dir)
    candidate_paths = complete_h5_files(candidate_dir)

    print(
        f"REFERENCE directory={Path(reference_dir).resolve()} files={len(reference_paths)}",
        file=stream,
    )
    reference_records: list[CloseRecord] = []
    for path in reference_paths:
        try:
            record = read_first_close(path, expected_position)
        except Exception as exc:
            print(f"REFERENCE EXCLUDED file={path.name} error={exc}", file=stream)
            continue
        reference_records.append(record)
        print(
            f"REFERENCE OK file={path.name} close_frame={record.close_frame} "
            f"xyz_mm={_xyz_text(record.xyz_mm)}",
            file=stream,
        )

    print(
        f"CANDIDATE directory={Path(candidate_dir).resolve()} files={len(candidate_paths)}",
        file=stream,
    )
    candidate_records: list[CloseRecord] = []
    qa_pass = 0
    qa_fail = 0
    for path in candidate_paths:
        passed, qa_detail = candidate_qa(path)
        qa_pass += int(passed)
        qa_fail += int(not passed)
        qa_label = "PASS" if passed else "FAIL"
        try:
            record = read_first_close(path, expected_position)
        except Exception as exc:
            print(
                f"CANDIDATE EXCLUDED file={path.name} qa={qa_label} "
                f"qa_detail={qa_detail!r} close_error={exc}",
                file=stream,
            )
            continue
        candidate_records.append(record)
        print(
            f"CANDIDATE OK file={path.name} qa={qa_label} "
            f"qa_detail={qa_detail!r} close_frame={record.close_frame} "
            f"xyz_mm={_xyz_text(record.xyz_mm)}",
            file=stream,
        )

    if not reference_records:
        raise ValueError("reference batch has no valid one-close episode")
    if not candidate_records:
        raise ValueError("candidate batch has no valid one-close episode")

    reference_mean, reference_std = cluster_stats(reference_records)
    candidate_mean, candidate_std = cluster_stats(candidate_records)
    delta = candidate_mean - reference_mean
    distance = float(np.linalg.norm(delta))
    print(
        f"REFERENCE SUMMARY valid={len(reference_records)} "
        f"mean_xyz_mm={_xyz_text(reference_mean)} "
        f"std_xyz_mm={_xyz_text(reference_std)}",
        file=stream,
    )
    print(
        f"CANDIDATE SUMMARY valid={len(candidate_records)} qa_pass={qa_pass} "
        f"qa_fail={qa_fail} mean_xyz_mm={_xyz_text(candidate_mean)} "
        f"std_xyz_mm={_xyz_text(candidate_std)}",
        file=stream,
    )
    print(
        f"DELTA candidate_minus_reference_xyz_mm={_xyz_text(delta)} "
        f"l2_mm={distance:.3f}",
        file=stream,
    )
    return {
        "reference_records": reference_records,
        "candidate_records": candidate_records,
        "reference_mean_xyz_mm": reference_mean,
        "reference_std_xyz_mm": reference_std,
        "candidate_mean_xyz_mm": candidate_mean,
        "candidate_std_xyz_mm": candidate_std,
        "candidate_minus_reference_xyz_mm": delta,
        "l2_mm": distance,
        "candidate_qa_pass": qa_pass,
        "candidate_qa_fail": qa_fail,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare first-close EEF XYZ clusters in two HDF5 directories."
    )
    parser.add_argument("--reference-dir", required=True)
    parser.add_argument("--candidate-dir", required=True)
    parser.add_argument("--expected-position", default="grid_5")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        compare_batches(
            args.reference_dir,
            args.candidate_dir,
            expected_position=args.expected_position,
        )
    except (OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
