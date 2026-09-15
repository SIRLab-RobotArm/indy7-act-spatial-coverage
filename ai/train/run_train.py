#!/usr/bin/env python3
"""Reproducible RGB-only ACT training wrapper."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset

NORMALIZATION = '{"VISUAL":"IDENTITY","STATE":"MEAN_STD","ACTION":"QUANTILE10"}'


def copy_state_baseline(dataset) -> dict:
    # Read only the two numeric Arrow columns. Iterating through ``dataset``
    # would also decode every RGB frame even though the baseline does not use
    # images, which makes startup unnecessarily slow for the full dataset.
    columns = (
        dataset.hf_dataset
        .select_columns(["observation.state", "action"])
        .with_format("numpy")[:]
    )
    target = np.asarray(columns["action"], dtype=np.float32)
    state = np.asarray(columns["observation.state"], dtype=np.float32)
    prediction = np.zeros_like(target)
    prediction[:, -1] = state[:, -1]
    values = np.abs(target - prediction)
    return {
        "definition": "zero XYZ delta plus copied current binary gripper state",
        "l1_mean": float(values.mean()),
        "l1_per_dimension": values.mean(axis=0).tolist(),
        "sample_count": len(values),
    }


def command(args) -> list[str]:
    result = [
        sys.executable, "-m", "ai.train.lerobot_entry",
        "--dataset.repo_id", args.dataset_repo_id,
        "--dataset.root", str(Path(args.dataset_dir).resolve()),
        "--dataset.use_imagenet_stats", "false",
        "--policy.type", "act",
        "--policy.push_to_hub", "false",
        "--policy.device", args.device,
        "--policy.use_amp", str(args.use_amp).lower(),
        "--policy.use_vae", "true",
        "--policy.normalization_mapping", NORMALIZATION,
        "--policy.chunk_size", str(args.chunk_size),
        "--policy.n_action_steps", str(args.n_action_steps),
        "--output_dir", str(Path(args.output_dir).resolve()),
        "--job_name", args.job_name,
        "--seed", str(args.seed),
        "--batch_size", str(args.batch_size),
        "--num_workers", str(args.num_workers),
        "--steps", str(args.steps),
        "--eval_freq", "0",
        "--log_freq", str(args.log_freq),
        "--save_freq", str(args.save_freq),
        "--use_policy_training_preset", "false",
        "--optimizer.type", "adamw",
        "--optimizer.lr", "0.0001",
        "--optimizer.weight_decay", "0.0001",
        "--optimizer.grad_clip_norm", "10.0",
        "--scheduler.type", "cosine_decay_with_warmup",
        "--scheduler.num_warmup_steps", "2000",
        "--scheduler.num_decay_steps", str(args.steps),
        "--scheduler.peak_lr", "0.0001",
        "--scheduler.decay_lr", "0.000001",
    ]
    if args.dev_small:
        result.extend([
            "--policy.pretrained_backbone_weights", "null",
            "--policy.dim_model", "64",
            "--policy.n_heads", "8",
            "--policy.dim_feedforward", "128",
            "--policy.n_encoder_layers", "1",
            "--policy.n_vae_encoder_layers", "1",
        ])
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-dir", required=True)
    parser.add_argument("--dataset-repo-id", default="sirlab/grip_all")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--job-name", default="act_grip")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--steps", type=int, default=100000)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--num-workers", type=int, default=14)
    parser.add_argument("--chunk-size", type=int, default=40)
    parser.add_argument("--n-action-steps", type=int, default=10)
    parser.add_argument("--save-freq", type=int, default=20000)
    parser.add_argument("--log-freq", type=int, default=50)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--use-amp", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--ram-cache", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--dev-small", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    if args.n_action_steps > args.chunk_size:
        parser.error("--n-action-steps cannot exceed --chunk-size")
    if args.steps <= 0:
        parser.error("--steps must be positive")
    return args


def main():
    args = parse_args()
    base = LeRobotDataset(args.dataset_repo_id, root=Path(args.dataset_dir).resolve())
    sample = base[0]
    if next(iter(sample[key] for key in base.meta.camera_keys)).shape[0] != 3:
        raise AssertionError("RGB dataset must provide three image channels")
    baseline = copy_state_baseline(base)
    output = Path(args.output_dir).resolve()
    print(json.dumps({
        "dataset": str(base.root),
        "observation_mode": "rgb_only",
        "normalization": json.loads(NORMALIZATION),
        "use_vae": True,
        "copy_state_baseline": baseline,
    }, indent=2, sort_keys=True))
    cmd = command(args)
    print("COMMAND", " ".join(cmd))
    if args.dry_run:
        return
    output.parent.mkdir(parents=True, exist_ok=True)
    baseline_path = output.parent / f"{output.name}_copy_state_baseline.json"
    baseline_path.write_text(json.dumps(baseline, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    env = os.environ.copy()
    repo_root = str(Path(__file__).resolve().parents[2])
    env["PYTHONPATH"] = repo_root + os.pathsep + env.get("PYTHONPATH", "")
    env["LEROBOT_RAM_CACHE"] = "1" if args.ram_cache else "0"
    env["LEROBOT_RAM_CACHE_WORKERS"] = str(args.num_workers)
    subprocess.run(cmd, check=True, env=env, cwd=repo_root)


if __name__ == "__main__":
    main()
