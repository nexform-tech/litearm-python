"""状态帧布局兼容性 —— 固件 1.5.x 的 `6+21N` 与旧版 `4+21N` 都必须能解。

固件侧权威定义: litearm-stm32/User/litearm/hal/usb_cmd.c 的 usb_cmd_report_status()
  0x40 = flags(u16) + seq(u16) + 每轴 [q,dq,tau,t_mos,t_coil f32×5 + err u8] + joint_fault(u16, 仅 1.5.x)
  7J: 6+21*7 = 153B (1.5.x) / 4+21*7 = 151B (<=1.4.x); 1J: 27B / 25B
flags bit9 = enabled (1.5.0 起)
"""
from __future__ import annotations

import struct

import pytest

from litearm import _protocol as P
from litearm import state as ST


def synth_status(n=7, mode=1, flags=0, joint_fault=0, err=0, layout=6):
    """按指定布局造一帧状态 payload (layout=6 → 1.5.x, layout=4 → 旧版)。"""
    body = bytearray(struct.pack("<HH", flags | (mode << 6), 5))
    for i in range(n):
        body += struct.pack("<fffff", i * 0.1, 0.0, 1.0, 30.0, 25.0)
        body += bytes([err])
    if layout == 6:
        body += struct.pack("<H", joint_fault)
    return bytes(body)


# ---------------------------------------------------------------- 解码
def test_decode_1_5_layout_7j():
    """固件 1.5.x: 153B, joint_fault 在尾部。"""
    p = synth_status(7, mode=1, flags=(1 << 9), joint_fault=0b0001010, layout=6)
    assert len(p) == 153
    st = ST.decode_state(p)
    assert st.n == 7
    assert abs(st.q[3] - 0.3) < 1e-6
    assert st.joint_fault == 0b0001010
    assert st.enabled is True


def test_decode_legacy_layout_still_works():
    """旧版 1.4.x: 151B, 无 joint_fault -> 视为 0, 不抛错。"""
    p = synth_status(7, mode=1, flags=(1 << 9), layout=4)
    assert len(p) == 151
    st = ST.decode_state(p)
    assert st.n == 7
    assert st.joint_fault == 0
    assert st.enabled is True


def test_decode_1j_both_layouts():
    for layout, ln in ((6, 27), (4, 25)):
        p = synth_status(1, mode=6, flags=1, layout=layout)
        assert len(p) == ln
        st = ST.decode_state(p)
        assert st.n == 1 and st.faulted


def test_decode_enabled_bit_is_not_flag():
    """bit9=enabled 不能被当成安全 flag 名报出来 (flags 低 6 位才是)。"""
    p = synth_status(7, mode=1, flags=(1 << 9), layout=6)
    st = ST.decode_state(p)
    assert st.flag_names == []
    assert st.enabled is True
    assert not st.faulted


def test_decode_rejects_junk():
    for bad in (b"", b"\x01\x02", synth_status(7, layout=6) + b"\x00"):
        with pytest.raises(ValueError):
            ST.decode_state(bad)


def test_decode_status_tuple_shape():
    """decode_status 返回含 joint_fault (旧调用方按位置解包的形状保持不变)。"""
    flags, seq, mode, names, joints, jf = P.decode_status(synth_status(7, layout=6))
    assert mode == 1 and len(joints) == 7 and jf == 0


def test_err_byte_decoded():
    st = ST.decode_state(synth_status(7, mode=1, err=13, layout=6))
    assert st.joints[0].err == 13


# ---------------------------------------------------------------- 版本门
def test_min_firmware_requires_1_5():
    from litearm.arm import MIN_FW

    assert MIN_FW == (1, 5, 0)


def test_version_gate_rejects_1_4(fake_transport_factory):
    """1.4.0 是旧布局 (4+21N), 本包最低要求 1.5.0。"""
    from litearm import Arm
    from litearm.errors import FirmwareMismatchError

    fake_transport_factory(fw="Litearm1.4.0-7J")
    with pytest.raises(FirmwareMismatchError):
        Arm(port="fake").connect()
