import json
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
GRIP_EVAL_SRC = ROOT / "ros2_ws/src/grip_eval"
sys.path.insert(0, str(GRIP_EVAL_SRC))

from ai.serve.zmq_act_server import PROTOCOL as SERVER_PROTOCOL, decode  # noqa: E402
from grip_eval.act_client import PROTOCOL as CLIENT_PROTOCOL, ZmqActClient  # noqa: E402


def test_rgb_only_server_decodes_three_parts():
    state = np.arange(10, dtype="<f4")
    rgb = np.arange(4 * 5 * 3, dtype=np.uint8).reshape(4, 5, 3)
    header = {
        "protocol": SERVER_PROTOCOL,
        "command": "predict",
        "state_shape": list(state.shape),
        "rgb_shape": list(rgb.shape),
    }

    decoded_header, arrays = decode([
        json.dumps(header).encode("utf-8"), state.tobytes(), rgb.tobytes()
    ])

    assert CLIENT_PROTOCOL == SERVER_PROTOCOL == "grip_act_rgb_v1"
    assert decoded_header == header
    np.testing.assert_array_equal(arrays[0], state)
    np.testing.assert_array_equal(arrays[1], rgb)


def test_rgb_only_client_sends_no_depth_payload():
    sent = []

    class FakeSocket:
        def send_multipart(self, parts):
            sent.append(parts)

        def recv_multipart(self):
            chunk = np.zeros((40, 4), dtype="<f4")
            return json.dumps({"ok": True, "shape": chunk.shape}).encode(), chunk.tobytes()

    client = ZmqActClient.__new__(ZmqActClient)
    client._socket = FakeSocket()
    state = np.zeros(10, dtype=np.float32)
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)

    chunk = client.predict_chunk(state=state, rgb=rgb)

    assert chunk.shape == (40, 4)
    assert len(sent[0]) == 3
    header = json.loads(sent[0][0])
    assert "depth_shape" not in header
    assert "depth_channel" not in header


def test_executor_source_has_no_depth_subscription():
    source = (
        ROOT / "ros2_ws/src/grip_eval/grip_eval/policy_executor_node.py"
    ).read_text(encoding="utf-8")
    assert "aligned_depth_to_color" not in source
    assert "depth_mm" not in source
