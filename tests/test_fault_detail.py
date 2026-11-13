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
    p = bytes([P.RSP_FAULT]) + struct.pack("<I", tick) + bytes([cause])
    p += bytes(reasons) + bytes([drop])
    return p


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
        expect=lambda *a, **k: (None, bytes([P.RSP_FAULT, 1, 2])))

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
