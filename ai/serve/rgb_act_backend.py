"""Three-channel RGB adapter for LeRobot ACT chunk inference."""

from __future__ import annotations

from contextlib import nullcontext
from pathlib import Path

import cv2
import numpy as np

from ai.serve.lerobot_act_backend import LeRobotActBackend, resolve_pretrained_model_dir


class RGBActBackend(LeRobotActBackend):
    def __init__(self, pretrained_model_dir, dataset_repo_id, dataset_root,
                 device="cuda", task=""):
        from lerobot.datasets.dataset_metadata import LeRobotDatasetMetadata

        model_dir = resolve_pretrained_model_dir(str(pretrained_model_dir))
        super().__init__(model_dir, dataset_repo_id, dataset_root, device, task)
        meta = LeRobotDatasetMetadata(dataset_repo_id, root=Path(dataset_root).resolve())
        if len(meta.camera_keys) != 1:
            raise ValueError(f"expected exactly one RGB camera key, got {meta.camera_keys}")
        self.camera_key = meta.camera_keys[0]
        feature = meta.features[self.camera_key]
        shape = tuple(feature["shape"])
        names = tuple(feature.get("names") or ())
        channels = int(shape[-1] if names == ("height", "width", "channels") else shape[0])
        if channels != 3:
            raise ValueError(f"RGB-only ACT requires three image channels, got {shape}")
        self.output_hw = (
            (int(shape[0]), int(shape[1]))
            if names == ("height", "width", "channels")
            else (int(shape[-2]), int(shape[-1]))
        )

    def reset(self):
        self.policy.reset()
        self.preprocessor.reset()
        self.postprocessor.reset()

    def predict_chunk(self, *, state, rgb):
        from lerobot.policies.utils import prepare_observation_for_inference

        height, width = self.output_hw
        image = cv2.resize(
            np.asarray(rgb, dtype=np.uint8),
            (width, height),
            interpolation=cv2.INTER_AREA,
        )
        observation = {
            "observation.state": np.asarray(state, dtype=np.float32),
            self.camera_key: image,
        }
        torch = self._torch
        amp = bool(self.policy.config.use_amp) and self.device.type == "cuda"
        with torch.inference_mode(), torch.autocast("cuda") if amp else nullcontext():
            batch = prepare_observation_for_inference(
                observation, self.device, self.task or None, self.robot_type)
            batch = self.preprocessor(batch)
            action = self.policy.predict_action_chunk(batch)
            action = self.postprocessor(action)
        return action.detach().float().cpu().numpy()[0].astype(np.float32, copy=False)
