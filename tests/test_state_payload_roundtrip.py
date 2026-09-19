"""广播载荷的编解码往返 —— **唯一能拦住"静默空帧"的东西**。

⚠ 强类型 protobuf 对**未知键静默丢弃**：`encode_state(新载荷)` 不会报错，
只会产出一个"成功"的空帧。所以只断言"能编码"是没用的 —— 必须**逐字段**核
往返后每个键都还在。
"""
import pytest

from litearm import codec

PAYLOAD = {
    "mode": 1,
    "mode_name": "MOVE_J",
    "flags": 0x200,                       # bit9 = enabled
    "flag_names": ["WD_TRIPPED"],
    "seq": 7,
    "joint_fault": 0b10,                  # J2 断轴
    "joints": [
        {"q": 0.1, "dq": 0.0, "tau": 0.5, "t_mos": 30.0, "t_coil": 25.0, "err": 0}
        for _ in range(7)
    ],
    "enabled": True,
    "cart_busy": False,
    "faulted": False,
    "fault_axes": [1],
    "hz": 100.0,
    "timestamp": 12345.0,
}


def test_every_key_survives_roundtrip():
    back = codec.decode_state(codec.encode_state(PAYLOAD))
    for k in PAYLOAD:
        assert k in back, f"{k} 被静默丢弃 —— proto 的字段清单漏了它"


@pytest.mark.parametrize("key", sorted(PAYLOAD))
def test_individual_key(key):
    back = codec.decode_state(codec.encode_state(PAYLOAD))
    assert back[key] == PAYLOAD[key], f"{key} 往返后值不对"


def test_old_payload_still_roundtrips():
    """旧（pylitearm 形态）载荷必须仍然能往返 —— 两套字段并存一个版本。"""
    old = {
        "q": [0.0] * 7, "dq": [0.0] * 7, "tau": [0.0] * 7,
        "fault": [(1, 5)], "errs": [0] * 7, "temps": [(30, 25)] * 7,
        "state": "ready", "robot_serial": "SN1", "config_checksum_sha256": "abc",
    }
    back = codec.decode_state(codec.encode_state(old))
    for k in old:
        assert back[k] == old[k], f"旧键 {k} 回退"


def test_missing_new_keys_do_not_break_encoding():
    """只给旧键时不该抛 —— 并存期的兼容性。"""
    codec.encode_state({"q": [0.0] * 7})
