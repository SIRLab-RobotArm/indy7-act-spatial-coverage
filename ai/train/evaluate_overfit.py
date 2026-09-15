#!/usr/bin/env python3
"""Evaluate a saved ACT checkpoint against the physical-unit copy-state baseline."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import torch
from torch.utils.data import DataLoader

from lerobot.configs.policies import PreTrainedConfig
from lerobot.datasets.dataset_metadata import LeRobotDatasetMetadata
from lerobot.datasets.factory import resolve_delta_timestamps
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.factory import make_policy, make_pre_post_processors
from lerobot.utils.constants import ACTION, OBS_IMAGES

def pretrained_dir(path: str | Path) -> Path:
    root = Path(path).resolve()
    candidates = [root, root / "pretrained_model", root / "checkpoints" / "last" / "pretrained_model"]
    for candidate in candidates:
        if (candidate / "config.json").is_file():
            return candidate
    raise FileNotFoundError(f"no pretrained_model/config.json under {root}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--dataset-dir", required=True)
    parser.add_argument("--dataset-repo-id", default="sirlab/grip_all")
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    model_dir = pretrained_dir(args.checkpoint)
    config = PreTrainedConfig.from_pretrained(model_dir)
    config.pretrained_path = str(model_dir)
    config.device = args.device
    metadata = LeRobotDatasetMetadata(args.dataset_repo_id, root=args.dataset_dir)
    base = LeRobotDataset(
        args.dataset_repo_id,
        root=args.dataset_dir,
        delta_timestamps=resolve_delta_timestamps(config, metadata),
    )
    dataset = base
    policy = make_policy(config, ds_meta=metadata)
    policy.eval()
    preprocessor, _ = make_pre_post_processors(
        policy_cfg=policy.config,
        pretrained_path=model_dir,
        preprocessor_overrides={
            "device_processor": {"device": args.device},
            "rename_observations_processor": {"rename_map": {}},
        },
    )
    q10 = torch.as_tensor(metadata.stats[ACTION]["q10"], device=args.device, dtype=torch.float32)
    q90 = torch.as_tensor(metadata.stats[ACTION]["q90"], device=args.device, dtype=torch.float32)
    denominator = q90 - q10
    denominator = torch.where(denominator == 0, torch.full_like(denominator, 1e-8), denominator)
    model_abs_sum = 0.0
    baseline_abs_sum = 0.0
    element_count = 0
    with torch.no_grad():
        for physical_batch in DataLoader(dataset, batch_size=args.batch_size, shuffle=False):
            targets = physical_batch[ACTION].to(args.device)
            padding = physical_batch["action_is_pad"].to(args.device)
            batch = preprocessor(physical_batch)
            model_batch = dict(batch)
            model_batch[OBS_IMAGES] = [batch[key] for key in policy.config.image_features]
            prediction_normalized = policy.model(model_batch)[0]
            prediction = (prediction_normalized + 1.0) * denominator / 2.0 + q10
            mask = (~padding).unsqueeze(-1).expand_as(targets)
            model_abs_sum += torch.abs(prediction - targets)[mask].sum().item()
            copied_gripper = physical_batch["observation.state"][:, -1].to(args.device)
            baseline = torch.zeros_like(targets)
            baseline[..., 3] = copied_gripper[:, None]
            baseline_abs_sum += torch.abs(baseline - targets)[mask].sum().item()
            element_count += int(mask.sum().item())
    result = {
        "checkpoint": str(model_dir),
        "physical_l1": model_abs_sum / element_count,
        "copy_state_baseline_l1": baseline_abs_sum / element_count,
        "valid_action_elements": element_count,
        "beats_copy_state_baseline": model_abs_sum < baseline_abs_sum,
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
