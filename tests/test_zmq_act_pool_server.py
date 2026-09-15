from pathlib import Path

import numpy as np
import pytest

from ai.serve.zmq_act_pool_server import ActModelPool, model_registry


class FakeBackend:
    def __init__(self, value):
        self.value = value
        self.reset_count = 0

    def reset(self):
        self.reset_count += 1

    def predict_chunk(self, *, state, rgb):
        assert np.asarray(state).shape == (10,)
        assert np.asarray(rgb).shape == (2, 3, 3)
        return np.full((40, 4), self.value, dtype=np.float32)


def test_pool_selects_resets_and_routes_one_resident_backend():
    first = FakeBackend(1.0)
    second = FakeBackend(2.0)
    pool = ActModelPool({"M01": first, "M02": second})

    pool.select("M02")
    pool.reset()
    chunk = pool.predict_chunk(
        state=np.zeros(10, dtype=np.float32),
        rgb=np.zeros((2, 3, 3), dtype=np.uint8),
    )

    assert second.reset_count == 2
    assert first.reset_count == 0
    assert chunk.shape == (40, 4)
    assert np.all(chunk == 2.0)


def _frozen_registry():
    repo_root = Path(__file__).resolve().parents[1]
    return model_registry(
        repo_root /
        "experiment/evaluation/main_recollection_20260817/model_key.json",
        repo_root,
    )


def test_frozen_model_key_maps_twelve_blind_codes():
    registry = _frozen_registry()

    assert sorted(registry) == [f"M{index:02d}" for index in range(1, 13)]
    for entry in registry.values():
        assert entry["dataset_repo_id"].startswith("sirlab/grip_recollection_20260817_")


@pytest.mark.skipif(
    not (Path(__file__).resolve().parents[1] / "ai/models").is_dir(),
    reason="needs the trained checkpoints, which are not distributed with the "
           "repository - see docs/data_and_weights.md")
def test_frozen_model_key_resolves_all_twelve_local_resources():
    for entry in _frozen_registry().values():
        assert entry["model"].is_dir()
        assert entry["dataset_root"].is_dir()
