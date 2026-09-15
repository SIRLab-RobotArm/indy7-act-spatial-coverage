#!/usr/bin/env python3
"""One-process GPU-resident pool for all 12 frozen ACT evaluation models."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import zmq

from ai.serve.rgb_act_backend import RGBActBackend
from ai.serve.zmq_act_server import PROTOCOL, decode


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MODEL_KEY = Path(
    "experiment/evaluation/main_recollection_20260817/model_key.json")


def model_registry(model_key: Path, repo_root: Path = REPO_ROOT) -> dict[str, dict]:
    """Resolve the frozen blind model codes to local model and dataset inputs."""
    key = json.loads(model_key.read_text(encoding="utf-8"))
    registry = {}
    for model_code, identity in sorted(key.items()):
        condition = str(identity["condition"]).lower()
        seed = int(identity["seed"])
        registry[model_code] = {
            "model": repo_root / (
                f"ai/models/main_recollection_20260817_{condition}_s{seed}_100000/"
                "checkpoints/last/pretrained_model"),
            "dataset_root": (
                repo_root / f"datasets/main_recollection_20260817_rgb_{condition}"),
            "dataset_repo_id": f"sirlab/grip_recollection_20260817_{condition}",
        }
    return registry


class ActModelPool:
    """Route inference to one selected backend while retaining all on device."""

    def __init__(self, backends: dict[str, object]):
        if not backends:
            raise ValueError("ACT model pool cannot be empty")
        self.backends = dict(backends)
        self.active_model_code: str | None = None

    def select(self, model_code: str) -> None:
        if model_code not in self.backends:
            raise ValueError(f"unknown model_code {model_code!r}")
        self.active_model_code = model_code
        self.backends[model_code].reset()

    def reset(self) -> None:
        if self.active_model_code is None:
            raise RuntimeError("no ACT model has been selected")
        self.backends[self.active_model_code].reset()

    def predict_chunk(self, *, state, rgb):
        if self.active_model_code is None:
            raise RuntimeError("no ACT model has been selected")
        return self.backends[self.active_model_code].predict_chunk(
            state=state, rgb=rgb)


def load_pool(
    registry: dict[str, dict], device: str, *, warmup: bool = True,
    backend_factory=RGBActBackend,
) -> ActModelPool:
    backends = {}
    total = len(registry)
    for index, (model_code, entry) in enumerate(sorted(registry.items()), start=1):
        model = Path(entry["model"])
        dataset_root = Path(entry["dataset_root"])
        if not model.is_dir():
            raise FileNotFoundError(f"model directory missing for {model_code}: {model}")
        if not dataset_root.is_dir():
            raise FileNotFoundError(
                f"dataset directory missing for {model_code}: {dataset_root}")
        print(f"ACT pool loading {index}/{total}: {model_code}", flush=True)
        backend = backend_factory(
            model, entry["dataset_repo_id"], dataset_root, device)
        if warmup:
            backend.predict_chunk(
                state=np.zeros(10, dtype=np.float32),
                rgb=np.zeros((480, 640, 3), dtype=np.uint8),
            )
            backend.reset()
        backends[model_code] = backend
        print(f"ACT pool loaded {index}/{total}: {model_code}", flush=True)
    return ActModelPool(backends)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model-key", type=Path, default=DEFAULT_MODEL_KEY)
    parser.add_argument("--repo-root", type=Path, default=REPO_ROOT)
    parser.add_argument("--endpoint", default="tcp://127.0.0.1:5557")
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--no-warmup", action="store_true")
    args = parser.parse_args()

    repo_root = args.repo_root.resolve()
    model_key = (
        args.model_key.resolve()
        if args.model_key.is_absolute()
        else (repo_root / args.model_key).resolve())
    registry = model_registry(model_key, repo_root)
    pool = load_pool(registry, args.device, warmup=not args.no_warmup)

    context = zmq.Context.instance()
    socket = context.socket(zmq.REP)
    socket.bind(args.endpoint)
    print(
        f"ACT model pool ready at {args.endpoint} ({len(pool.backends)} models)",
        flush=True)
    try:
        while True:
            try:
                header, arrays = decode(socket.recv_multipart())
                command = header.get("command")
                if command == "select":
                    pool.select(str(header.get("model_code")))
                    socket.send_json({
                        "ok": True, "model_code": pool.active_model_code,
                    })
                    continue
                if command == "reset":
                    pool.reset()
                    socket.send_json({
                        "ok": True, "model_code": pool.active_model_code,
                    })
                    continue
                if command != "predict":
                    raise ValueError(f"unsupported command {command!r}")
                state, rgb = arrays
                chunk = pool.predict_chunk(state=state, rgb=rgb)
                socket.send_multipart([
                    json.dumps({
                        "ok": True,
                        "shape": chunk.shape,
                        "model_code": pool.active_model_code,
                    }).encode("utf-8"),
                    np.ascontiguousarray(chunk, dtype="<f4").tobytes(),
                ])
            except Exception as exc:
                socket.send_multipart([
                    json.dumps({"ok": False, "error": repr(exc)}).encode("utf-8"),
                    b"",
                ])
    except KeyboardInterrupt:
        pass
    finally:
        socket.close(linger=0)


if __name__ == "__main__":
    main()
