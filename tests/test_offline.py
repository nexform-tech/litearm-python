#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线单测(无硬件): 帧编解码/G8 状态解析/版本约定/到位判定。"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import struct

from litearm import _protocol as P
from litearm import state as ST
from litearm.arm import Arm
from litearm.errors import FirmwareMismatchError

fails = []


def check(name, cond):
    if not cond:
        fails.append(name)
        print(f"  FAIL: {name}")
    else:
        print(f"  ok:   {name}")


def test_frame_roundtrip():
    for cmd, payload in ((0x01, b""), (0x45, b"\x10"), (0x02, bytes(28))):
        fr = P.pack_frame(cmd, payload)
        assert len(fr) == 3 + len(payload) + 2
        got = P.unpack_frame(fr)
        check(f"frame rt cmd={cmd:02x}", got == (cmd, payload))
    bad = P.unpack_frame(bytes([0xA5, 0x45, 0x01, 0x99, 0, 0]))
    check("frame crc bad -> None", bad is None)
    check("frame nan/short -> None", P.unpack_frame(b"\x01\x02") is None)


def synth_status(n=7, mode=1, flags=0):
    body = bytearray()
    body += struct.pack("<HH", flags | (mode << 6), 5)  # flags2 + seq2
    for i in range(n):
        body += struct.pack("<fffff", i * 0.1, 0.0, 1.0, 30.0, 25.0)
        body.append(0)
    return bytes(body)


def test_state_decode():
    p = synth_status(7, mode=1)
    st = ST.decode_state(p)
    check("G8 n=7", st.n == 7 and st.mode_name == "MOVE_J")
    check("G8 joints/err", abs(st.q[2] - 0.2) < 1e-6 and st.joints[2].err == 0)
    p6 = synth_status(1, mode=6, flags=1)
    st6 = ST.decode_state(p6)
    check("1J/emergency/fault", st6.n == 1 and st6.faulted)
    check("bad -> raise", _raises(ValueError, lambda: ST.decode_state(b"\x01")))


def _raises(exc, fn):
    try:
        fn()
    except exc:
        return True
    return False


def test_fw_version():
    v = P.parse_firmware_version("Litearm1.4.0-7J")
    check("fw parse Litearm", v == (1, 4, 0, "7J"))
    v = P.parse_firmware_version("Litearm1.4.0-1J")
    check("fw parse 1J", v == (1, 4, 0, "1J"))
    check("fw old A -> None", P.parse_firmware_version("A1.3.0-7J-USB") is None)
    check("fw junk -> None", P.parse_firmware_version("hello") is None)


def test_pose_near():
    a = object.__new__(Arm)
    tcp = (0.3002, 0.001, 0.35, 3.1416, 0.0, 0.001)
    goal = (0.30, 0.0, 0.35, 3.1416, 0.0, 0.0)
    check("pose near ok", Arm._pose_near(tcp, goal, 0.006, 0.03))
    tcp2 = (0.35, 0.0, 0.35, 3.14, 0.0, 0.0)
    check("pose far", not Arm._pose_near(tcp2, goal, 0.006, 0.03))


def test_ff_mask_bits():
    check("ff bits", P.FF_ALL == 0x1FF and P.FF_MASTER == 1)


if __name__ == "__main__":
    test_frame_roundtrip()
    test_state_decode()
    test_fw_version()
    test_pose_near()
    test_ff_mask_bits()
    print("offline:", "PASS" if not fails else f"FAIL {len(fails)}: {fails}")
    sys.exit(1 if fails else 0)
