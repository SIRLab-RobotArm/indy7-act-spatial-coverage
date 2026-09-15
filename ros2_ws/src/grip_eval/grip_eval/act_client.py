"""ACT inference clients. ROS uses ZMQ to isolate the NumPy ABI boundary."""

from __future__ import annotations

import json

import numpy as np


PROTOCOL = "grip_act_rgb_v1"


class DryRunActClient:
    def __init__(self, chunks):
        self._chunks = iter(chunks)

    def predict_chunk(self, **_observation):
        chunk = np.asarray(next(self._chunks), dtype=np.float32)
        if chunk.ndim != 2 or chunk.shape[1] != 4:
            raise ValueError(f"dry-run chunk must have shape (K,4), got {chunk.shape}")
        return chunk

    def reset(self):
        return None

    def close(self):
        return None


class ZmqActClient:
    """Small multipart protocol; no Python array object crosses environments."""

    def __init__(self, endpoint="tcp://127.0.0.1:5557", timeout_ms=15000):
        import zmq

        self._zmq = zmq
        self._context = zmq.Context.instance()
        self._socket = self._context.socket(zmq.REQ)
        self._socket.setsockopt(zmq.RCVTIMEO, int(timeout_ms))
        self._socket.setsockopt(zmq.SNDTIMEO, int(timeout_ms))
        self._socket.connect(endpoint)

    def predict_chunk(self, *, state, rgb):
        state = np.ascontiguousarray(state, dtype="<f4")
        rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
        header = {
            "protocol": PROTOCOL,
            "command": "predict",
            "state_shape": state.shape,
            "rgb_shape": rgb.shape,
        }
        self._socket.send_multipart([
            json.dumps(header).encode("utf-8"), state.tobytes(), rgb.tobytes()
        ])
        reply_header, payload = self._socket.recv_multipart()
        reply = json.loads(reply_header)
        if not reply.get("ok"):
            raise RuntimeError(reply.get("error", "ACT sidecar failed"))
        chunk = np.frombuffer(payload, dtype="<f4").reshape(reply["shape"]).copy()
        if chunk.ndim != 2 or chunk.shape[1] != 4:
            raise RuntimeError(f"ACT returned invalid chunk shape {chunk.shape}")
        return chunk

    def reset(self):
        header = {"protocol": PROTOCOL, "command": "reset"}
        self._socket.send_multipart([json.dumps(header).encode("utf-8")])
        reply = json.loads(self._socket.recv())
        if not reply.get("ok"):
            raise RuntimeError(reply.get("error", "ACT reset failed"))

    def close(self):
        self._socket.close(linger=0)
