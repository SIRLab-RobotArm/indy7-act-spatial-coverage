# Copyright 2026 SIRLab - local LeRobot ACT checkpoint
# (heavy imports are deferred to __init__)
from __future__ import annotations

import json
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np

if TYPE_CHECKING:
    import torch

    from lerobot.policies.pretrained import PreTrainedPolicy
    from lerobot.processor import PolicyProcessorPipeline


def resolve_pretrained_model_dir(model_path: str) -> Path:
    """Normalise a checkpoint root (`.../050000`), `.../last` or a
    `pretrained_model` directory to the path that actually holds config.json."""
    p = Path(model_path).expanduser().resolve()
    if not p.exists():
        raise FileNotFoundError(f"model_path does not exist: {p}")
    if (p / "config.json").is_file():
        return p
    sub = p / "pretrained_model"
    if sub.is_dir() and (sub / "config.json").is_file():
        return sub.resolve()
    raise FileNotFoundError(
        f"Expected config.json under {p} or {sub} "
        f"(a LeRobot pretrained_model directory, or checkpoints/XXXXXX|last)."
    )


def read_dataset_root_from_train_config(pretrained_dir: Path) -> str | None:
    tc = pretrained_dir / "train_config.json"
    if not tc.is_file():
        return None
    with open(tc, encoding="utf-8") as f:
        data = json.load(f)
    root = data.get("dataset", {}).get("root")
    return str(root) if root else None


class LeRobotActBackend:
    """Run ACT inference at the same scale as the LeRobotDataset used for training."""

    def __init__(
        self,
        pretrained_model_dir: Path,
        dataset_repo_id: str,
        dataset_root: Path,
        device: str | None = None,
        task: str = "",
    ) -> None:
        import torch
        from lerobot.configs.policies import PreTrainedConfig
        from lerobot.datasets.dataset_metadata import LeRobotDatasetMetadata
        from lerobot.policies.factory import make_policy, make_pre_post_processors
        from lerobot.utils.device_utils import get_safe_torch_device

        self.pretrained_model_dir = Path(pretrained_model_dir).resolve()
        if not self.pretrained_model_dir.is_dir():
            raise NotADirectoryError(str(self.pretrained_model_dir))

        self.device = get_safe_torch_device(device or "cuda", log=True)

        policy_cfg = PreTrainedConfig.from_pretrained(str(self.pretrained_model_dir))
        policy_cfg.pretrained_path = str(self.pretrained_model_dir)
        policy_cfg.device = str(self.device)

        ds_root = Path(dataset_root).resolve()
        if not (ds_root / "meta" / "info.json").is_file():
            raise FileNotFoundError(f"No meta/info.json under the LeRobot dataset root: {ds_root}")

        ds_meta = LeRobotDatasetMetadata(repo_id=dataset_repo_id, root=ds_root)
        self.policy = make_policy(cfg=policy_cfg, ds_meta=ds_meta)
        self.policy.eval()

        preprocessor_overrides: dict[str, Any] = {
            "device_processor": {"device": str(self.policy.config.device)},
            "rename_observations_processor": {"rename_map": {}},
        }
        self.preprocessor, self.postprocessor = make_pre_post_processors(
            policy_cfg=self.policy.config,
            pretrained_path=str(self.pretrained_model_dir),
            preprocessor_overrides=preprocessor_overrides,
        )
        self.policy.reset()
        self.preprocessor.reset()
        self.postprocessor.reset()

        self.task = task
        self.robot_type = ds_meta.robot_type or "pipet"
        self._torch = torch

    def predict(self, observation: dict[str, np.ndarray]) -> np.ndarray:
        from lerobot.utils.control_utils import predict_action

        act = predict_action(
            observation=observation,
            policy=self.policy,
            device=self.device,
            preprocessor=self.preprocessor,
            postprocessor=self.postprocessor,
            use_amp=bool(self.policy.config.use_amp),
            task=self.task or None,
            robot_type=self.robot_type or None,
        )
        if isinstance(act, self._torch.Tensor):
            out = act.detach().float().cpu().numpy().reshape(-1)
        else:
            out = np.asarray(act, dtype=np.float32).reshape(-1)
        return out.astype(np.float32, copy=False)
