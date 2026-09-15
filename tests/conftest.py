from argparse import Namespace
import os
from pathlib import Path
import shlex

import h5py
import numpy as np
import pytest

from ai.convert.build_dataset import build


REPO_ROOT = Path(__file__).resolve().parents[1]


def _load_local_env() -> None:
    """Make config/local.env visible to the tests, without overriding the shell.

    A few tests shell out to the system Python that runs ROS, and need
    GRIP_ROS_PYDEPS to find h5py and pyyaml. Reading the same file the wrapper
    scripts read keeps `pytest` working straight after `cp local.env.example`.
    """
    local_env = REPO_ROOT / "config" / "local.env"
    if not local_env.is_file():
        return
    for line in local_env.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        key = key.strip()
        if key and key not in os.environ:
            os.environ[key] = "".join(shlex.split(value.strip()))


_load_local_env()


def write_episode(path: Path, episode_index: int, frame_count: int = 7):
    height, width = 16, 20
    stamp = np.arange(frame_count, dtype=np.float64) / 20.0
    ee = np.zeros((frame_count, 6), dtype=np.float32)
    ee[:, 0] = np.arange(frame_count) * (episode_index + 1)
    ee[:, 1] = np.arange(frame_count) * -0.5
    ee[:, 2] = 100.0 - np.arange(frame_count) * 0.25
    rgb = np.empty((frame_count, height, width, 3), dtype=np.uint8)
    for frame in range(frame_count):
        rgb[frame] = (episode_index * 30 + frame) % 255
    with h5py.File(path, "w") as output:
        output.create_dataset("/obs/rgb", data=rgb, compression="lzf")
        output.create_dataset("/state/ee_pose", data=ee)
        output.create_dataset(
            "/state/joint_pos",
            data=np.arange(frame_count, dtype=np.float32)[:, None].repeat(6, axis=1) / 1000,
        )
        output.create_dataset(
            "/state/gripper_cmd", data=(np.arange(frame_count) >= 4).astype(np.uint8))
        for name, offset in (("rgb", 0.0), ("joint", 0.002), ("ee", 0.002)):
            output.create_dataset(f"/time/stamp_{name}", data=stamp + offset)
        output.create_dataset("/time/stamp_recv", data=stamp + 0.003)
        output.attrs.update({
            "episode_uuid": f"mock-{episode_index}",
            "record_hz": 20.0,
            "data_block": "dev",
        })


@pytest.fixture(scope="session")
def converted_dataset(tmp_path_factory):
    root = tmp_path_factory.mktemp("converted")
    episodes = root / "episodes" / "_dev"
    episodes.mkdir(parents=True)
    for index in range(3):
        write_episode(episodes / f"episode_{index}.h5", index)
    dataset = root / "grip_all"
    manifest = root / "manifest.json"
    build(Namespace(
        episodes_dir=str(episodes), output_dir=str(dataset),
        manifest=str(manifest), repo_id="sirlab/test_grip_all", task="mock grasp",
        action_rate=5.0, max_jitter_sec=0.015, image_height=8, image_width=10,
        max_episodes=0,
    ))
    return {
        "root": root, "episodes": episodes, "dataset": dataset,
        "manifest": manifest, "repo_id": "sirlab/test_grip_all",
    }
