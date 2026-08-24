"""litearm-python SDK：send_mit 异步 pub + set_guards/get_guards RPC 对齐。"""
import json
import time

import pytest

from litearm.arm import Arm
from litearm import protocol
from litearm.transport import InProcTransport


def test_send_mit_publishes_mit_frame():
    tp = InProcTransport()
    sub = tp.sub(protocol.command_topic("armA"))
    arm = Arm(transport=tp, arm_id="armA")
    arm.send_mit([15.0] * 7, [2.0] * 7, [0.1] * 7, [0.0] * 7, [0.0] * 7)
    payload = sub.try_recv()
    assert payload is not None
    frame = json.loads(payload.decode("utf-8"))
    assert frame["type"] == "mit"
    assert frame["client_id"]
    assert frame["kp"] == [15.0] * 7
    assert frame["q_ref"] == [0.1] * 7
    assert "tau_ff" in frame and "seq" in frame


def test_set_guards_rpc_roundtrip():
    tp = InProcTransport()
    from litearm import codec
    rpc_topic = protocol.rpc_topic("armA")

    def handler(payload):
        method, kwargs = codec.decode_request(payload)
        assert method == "set_guards"
        assert kwargs["slew_limit"] == 0.5
        return codec.encode_reply(ok=True, result=None)

    tp.declare_queryable(rpc_topic, handler)
    arm = Arm(transport=tp, arm_id="armA")
    arm.set_guards(slew_limit=0.5)


def test_get_guards_rpc_roundtrip():
    tp = InProcTransport()
    from litearm import codec
    rpc_topic = protocol.rpc_topic("armA")

    def handler(payload):
        method, _ = codec.decode_request(payload)
        assert method == "get_guards"
        return codec.encode_reply(ok=True, result={"slew_limit": 0.5,
                                                   "position_bounds": False})

    tp.declare_queryable(rpc_topic, handler)
    arm = Arm(transport=tp, arm_id="armA")
    assert arm.get_guards()["slew_limit"] == 0.5
