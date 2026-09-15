#!/usr/bin/env python3
"""Validate one completed main-evaluation rollout before moving on."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd

from ai.eval.protocol import FAILURE_CODES, HOLD_DURATION_S, LIFT_THRESHOLD_MM


REQUIRED_METADATA = (
    "condition", "seed", "model_code", "session", "repeat", "attempt",
    "scheduled_rollout_id",
    "position_id", "p_target", "d", "success",
    "object_move", "lift_threshold_mm", "hold_duration_s", "experiment_tag",
)
SMOOTH_EXECUTION_TAG = "main_recollection_20260817_main_smooth30hz"
SMOOTH_EXECUTION_METADATA = (
    "execution_controller", "policy_action_hz", "robot_command_hz",
    "interpolation_steps")


def lift_hold_metrics(trace: pd.DataFrame, t_close: float | None,
                      threshold_mm: float = LIFT_THRESHOLD_MM) -> dict:
    if t_close is None or trace.empty:
        return {"lift_mm": None, "hold_duration_s": 0.0}
    times = trace["t"].to_numpy(dtype=np.float64)
    poses = np.stack([np.asarray(value, dtype=np.float64) for value in trace["ee_pose"]])
    close_index = int(np.argmin(np.abs(times - float(t_close))))
    close_z = float(poses[close_index, 2])
    post_times = times[close_index:]
    post_z = poses[close_index:, 2]
    gripper = trace["gripper_cmd"].to_numpy(dtype=np.int64)[close_index:]
    lift_mm = float(np.max(post_z) - close_z)
    qualifying = (post_z - close_z >= float(threshold_mm)) & (gripper >= 1)
    longest = 0.0
    start = None
    previous_time = None
    for stamp, active in zip(post_times, qualifying):
        if active:
            if start is None or (
                    previous_time is not None and stamp - previous_time > 0.15):
                start = float(stamp)
            longest = max(longest, float(stamp) - start)
        else:
            start = None
        previous_time = float(stamp)
    return {"lift_mm": lift_mm, "hold_duration_s": float(longest)}


def grasp_position_metrics(trace: pd.DataFrame, t_close: float | None,
                           target_xyz: list[float] | None) -> dict:
    """Compare the first-close EEF XYZ with the frozen position target."""
    target = None if target_xyz is None else np.asarray(
        target_xyz, dtype=np.float64)
    result = {
        "target_xyz_mm": None if target is None else target.tolist(),
        "grasp_xyz_mm": None,
        "delta_xyz_mm": None,
        "xy_error_mm": None,
        "error_3d_mm": None,
    }
    if target is None or target.shape != (3,) or t_close is None or trace.empty:
        return result

    times = trace["t"].to_numpy(dtype=np.float64)
    poses = np.stack([
        np.asarray(value, dtype=np.float64) for value in trace["ee_pose"]
    ])
    stamp = float(t_close)
    if stamp <= times[0]:
        close_xyz = poses[0, :3]
    elif stamp >= times[-1]:
        close_xyz = poses[-1, :3]
    else:
        upper = int(np.searchsorted(times, stamp, side="right"))
        lower = upper - 1
        interval = times[upper] - times[lower]
        weight = 0.0 if interval <= 0 else (stamp - times[lower]) / interval
        close_xyz = (
            poses[lower, :3] * (1.0 - weight) + poses[upper, :3] * weight)
    delta = close_xyz - target
    result.update({
        "grasp_xyz_mm": close_xyz.tolist(),
        "delta_xyz_mm": delta.tolist(),
        "xy_error_mm": float(np.linalg.norm(delta[:2])),
        "error_3d_mm": float(np.linalg.norm(delta)),
    })
    return result


def validate(directory: Path, require_bag: bool = True) -> dict:
    problems = []
    required_files = ("trace.parquet", "events.json", "meta.json")
    for name in required_files:
        if not (directory / name).is_file():
            problems.append(f"missing {name}")
    partials = [path.name for path in directory.rglob("*.partial")]
    partials.extend(path.name for path in directory.rglob(".*.partial"))
    if partials:
        problems.append(f"partial artifacts remain: {sorted(set(partials))}")
    if require_bag and not (directory / "rollout.bag/metadata.yaml").is_file():
        problems.append("rosbag is not finalized (rollout.bag/metadata.yaml missing)")
    if problems and any(problem.startswith("missing") for problem in problems):
        return {"valid": False, "problems": problems}

    meta = json.loads((directory / "meta.json").read_text(encoding="utf-8"))
    events = json.loads((directory / "events.json").read_text(encoding="utf-8"))
    trace = pd.read_parquet(directory / "trace.parquet")
    for key in REQUIRED_METADATA:
        if meta.get(key) is None:
            problems.append(f"metadata {key}=null")
    if meta.get("experiment_tag") == SMOOTH_EXECUTION_TAG:
        for key in SMOOTH_EXECUTION_METADATA:
            if meta.get(key) is None:
                problems.append(f"metadata {key}=null")
    target = meta.get("p_target")
    if target is not None and (not isinstance(target, list) or len(target) != 3):
        problems.append("p_target must contain XYZ")
    if meta.get("success") is False and meta.get("failure_code") not in FAILURE_CODES:
        problems.append("failed rollout has invalid failure_code")
    if meta.get("success") is True and meta.get("failure_code") is not None:
        problems.append("successful rollout has failure_code")
    metrics = lift_hold_metrics(
        trace, events.get("t_close"),
        float(meta.get("lift_threshold_mm", LIFT_THRESHOLD_MM)))
    position_metrics = grasp_position_metrics(
        trace, events.get("t_close"), target)
    if meta.get("success") is True:
        if metrics["lift_mm"] is None:
            problems.append("successful rollout has no gripper close")
        elif metrics["lift_mm"] + 1e-6 < float(meta["lift_threshold_mm"]):
            problems.append("successful rollout does not reach lift threshold")
        if metrics["hold_duration_s"] + 1e-6 < float(
                meta.get("hold_duration_s", HOLD_DURATION_S)):
            problems.append("successful rollout does not reach hold duration")
    return {
        "valid": not problems,
        "problems": problems,
        "metrics": metrics,
        "grasp_position": position_metrics,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("rollout_dir", type=Path)
    parser.add_argument("--no-require-bag", action="store_true")
    args = parser.parse_args()
    result = validate(args.rollout_dir.resolve(), not args.no_require_bag)
    print(json.dumps(result, indent=2, sort_keys=True))
    if not result["valid"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
