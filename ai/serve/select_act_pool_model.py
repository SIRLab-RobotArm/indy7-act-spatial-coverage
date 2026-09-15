#!/usr/bin/env python3
"""Select one resident ACT model from an already running evaluation pool."""

from __future__ import annotations

import argparse

import zmq

from ai.serve.zmq_act_server import PROTOCOL


def select_model(model_code: str, endpoint: str, timeout_ms: int = 15_000) -> dict:
    context = zmq.Context.instance()
    socket = context.socket(zmq.REQ)
    socket.setsockopt(zmq.RCVTIMEO, int(timeout_ms))
    socket.setsockopt(zmq.SNDTIMEO, int(timeout_ms))
    socket.connect(endpoint)
    try:
        socket.send_json({
            "protocol": PROTOCOL,
            "command": "select",
            "model_code": model_code,
        })
        reply = socket.recv_json()
    finally:
        socket.close(linger=0)
    if not reply.get("ok") or reply.get("model_code") != model_code:
        raise RuntimeError(f"ACT model selection failed: {reply}")
    return reply


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("model_code")
    parser.add_argument("--endpoint", default="tcp://127.0.0.1:5557")
    parser.add_argument("--timeout-ms", type=int, default=15_000)
    args = parser.parse_args()
    reply = select_model(args.model_code, args.endpoint, args.timeout_ms)
    print(f"ACT resident model selected: {reply['model_code']}")


if __name__ == "__main__":
    main()
