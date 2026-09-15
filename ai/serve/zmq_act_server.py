#!/usr/bin/env python3
"""Foreground RGB-only ACT inference sidecar for the ROS/PyTorch ABI split."""

from __future__ import annotations

import argparse
import json

import numpy as np
import zmq

from ai.serve.rgb_act_backend import RGBActBackend


PROTOCOL = "grip_act_rgb_v1"


def decode(parts):
    header = json.loads(parts[0])
    if header.get("protocol") != PROTOCOL:
        raise ValueError("protocol mismatch")
    if header.get("command") != "predict":
        return header, None
    if len(parts) != 3:
        raise ValueError("RGB-only predict requires three message parts")
    state = np.frombuffer(parts[1], dtype="<f4").reshape(header["state_shape"]).copy()
    rgb = np.frombuffer(parts[2], dtype=np.uint8).reshape(header["rgb_shape"]).copy()
    return header, (state, rgb)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True)
    parser.add_argument("--dataset-root", required=True)
    parser.add_argument("--dataset-repo-id", default="sirlab/grip_all")
    parser.add_argument("--endpoint", default="tcp://127.0.0.1:5557")
    parser.add_argument("--device", default="cuda")
    args = parser.parse_args()
    backend = RGBActBackend(
        args.model, args.dataset_repo_id, args.dataset_root, args.device)
    context = zmq.Context.instance()
    socket = context.socket(zmq.REP)
    socket.bind(args.endpoint)
    print(f"ACT sidecar ready at {args.endpoint}", flush=True)
    try:
        while True:
            try:
                header, arrays = decode(socket.recv_multipart())
                if header["command"] == "reset":
                    backend.reset()
                    socket.send_json({"ok": True})
                    continue
                state, rgb = arrays
                chunk = backend.predict_chunk(state=state, rgb=rgb)
                socket.send_multipart([
                    json.dumps({"ok": True, "shape": chunk.shape}).encode("utf-8"),
                    np.ascontiguousarray(chunk, dtype="<f4").tobytes(),
                ])
            except Exception as exc:
                socket.send_multipart([
                    json.dumps({"ok": False, "error": repr(exc)}).encode("utf-8"), b""
                ])
    except KeyboardInterrupt:
        pass
    finally:
        socket.close(linger=0)


if __name__ == "__main__":
    main()
