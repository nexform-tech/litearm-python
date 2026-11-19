"""故障详情按需查询（0x51）—— 状态帧不扩长的配套能力。"""
from __future__ import annotations

import struct
from types import SimpleNamespace

import litearm as pa
from litearm import _protocol as P
from litearm import state as ST


def _status(flags_bits=0, n=7):
    body = bytearray(struct.pack("<HH", flags_bits, 1))
    for i in range(n):
        body += struct.pack("<fffff", 0.0, 0.0, 0.0, 30.0, 30.0) + bytes([1])
    body += struct.pack("<H", 0)
    return bytes(body)


def test_fault_detail_available_reads_bit11():
    st = ST.decode_state(_status(1 << P.FLAG_FAULT_DETAIL_BIT))
    assert st.fault_detail_available is True
    st2 = ST.decode_state(_status(0))
    assert st2.fault_detail_available is False


def test_fault_detail_available_does_not_disturb_low_bits():
    # bit11 + FAULT(bit0) + enabled(bit9)
    st = ST.decode_state(_status(0x1 | (1 << 9) | (1 << 11)))
    assert st.flag_names == ["FAULT"]
    assert st.enabled is True
    assert st.fault_detail_available is True


def _fault_payload(tick=12345, cause=0x05, reasons=(0, 0, 0, 1, 0, 0, 0), drop=1):
    """与固件逐字节一致：payload **不含** RSP id。"""
    return struct.pack("<I", tick) + bytes([cause]) + bytes(reasons) + bytes([drop])


def test_get_fault_detail_parses_payload():
    arm = pa.Arm.__new__(pa.Arm)
    arm.n = 7
    arm._write_query = lambda *a, **k: None
    payload = _fault_payload()
    arm._require = lambda: SimpleNamespace(
        expect=lambda *a, **k: (None, payload))

    d = arm.get_fault_detail()
    assert d.fault_tick == 12345
    assert d.fault_cause == 0x05
    assert d.fault_reason == [0, 0, 0, 1, 0, 0, 0]
    assert d.drop_hold_reason == 1
    assert d.fault_reasons == [(3, 1)]


def test_get_fault_detail_short_frame_raises():
    arm = pa.Arm.__new__(pa.Arm)
    arm.n = 7
    arm._write_query = lambda *a, **k: None
    arm._require = lambda: SimpleNamespace(
        expect=lambda *a, **k: (None, bytes([1, 2])))

    try:
        arm.get_fault_detail()
    except pa.TransportError:
        pass
    else:
        raise AssertionError("短帧应抛 TransportError")


def test_fault_code_is_ff_xx():
    t = pa.describe_fault(1)
    assert t.code == "FF-01"
    assert "CAN" in t.title


#: 每个有**专属应答**（不是 ACK/ERR）的命令 -> 那条应答 id。
#: ⚠ `0x40 GET_STATUS` 不在：它走单槽而不是队列；
#: 笛卡尔 `0x3A~0x3E` 不在：`RSP_CART_PLAN(0x4E)` 由 `cart` 模块自管。
_DEDICATED_REPLIES = {
    P.CMD_GET_JOINT_PARAM: P.RSP_JOINT_PARAM,
    P.CMD_GET_FF_VEC: P.RSP_FF_VEC,
    P.CMD_GET_FF_SCALAR: P.RSP_FF_SCALAR,
    P.CMD_LOG_READ: P.RSP_LOG_DATA,
    P.CMD_GET_LICENSE: P.RSP_LICENSE,
    P.CMD_GET_MODEL_PARAM: P.RSP_MODEL_PARAM,
    P.CMD_GET_MODEL_JM: P.RSP_MODEL_JM,
    P.CMD_GET_MODEL_STATUS: P.RSP_MODEL_STATUS,
    P.CMD_GET_GRAVITY: P.RSP_GRAVITY,
    P.CMD_GET_FIRMWARE: P.RSP_FIRMWARE,
    P.CMD_GET_IK: P.RSP_IK,
    P.CMD_GET_TCP: P.RSP_TCP,
    P.CMD_KIN_BENCH: P.RSP_KIN_BENCH,
    P.CMD_GET_FAULT: P.RSP_FAULT,
}


def test_dedicated_replies_are_cleared_before_reuse():
    """每个专属应答都必须在 `RSP_OF_CMD` 里，否则 `drain_for` 清不到它。

    ⚠ 漏项的后果是**假成功方向**：上一条已放弃查询的迟到应答留在队列，
    下一次同命令查询会先取到它（`_wait` 取 `seq` 最小者）。
    """
    assert P.RSP_OF_CMD == _DEDICATED_REPLIES


def test_drain_for_clears_fault_and_license_queues(offline_arm):
    """行为判据：`drain_for` 真的把 0x51 / 0x2F 的专属队列清空。"""
    a = offline_arm._a
    for cmd, rsp in ((P.CMD_GET_FAULT, P.RSP_FAULT),
                     (P.CMD_GET_LICENSE, P.RSP_LICENSE)):
        key = (rsp, None)
        a._queues[key] = [(rsp, b"stale", 0)]
        a.drain_for(cmd)
        assert key not in a._queues, f"drain_for(0x{cmd:02X}) 没清 {key!r}"
