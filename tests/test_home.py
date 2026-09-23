"""`home()` 必须发固件 `CMD_HOME 0x2A`, 不能用 `movej([0]*n)` 冒充。

固件侧 (`usb_cmd_dispatch` 的 `case CMD_HOME`, `usb_cmd.c:279-285`): `CMD_HOME` 空载荷,
`!enabled` 时回 `ERR{0x2A,0x03}`,
否则走 `ctrl_accept_move_j(home_q, 0.10f)` —— **速度写死 0.10**, 且注释明确
「允许从当前越软限/贴端发起」(软限位 clamp 作用于目标, 零位在限内故起点无关)。

旧 SDK 用 `movej([0]*7, speed=0.3)` 冒充, 差异有二: 速度是固件意图的 3 倍;
且只能整臂 7 关节用 (台架 1J 会被 `n != 7` 直接拒掉, 而固件 `CMD_HOME` 台架可用)。
"""
from __future__ import annotations

import pytest

from litearm import _protocol as P
from litearm.errors import UnsupportedByFirmwareError


def _cmds(arm):
    return [c for c, _ in list(arm._tr.tx_log)]


def test_home_sends_cmd_home_not_move_j(offline_arm):
    arm = offline_arm
    arm.enable()
    arm.home()
    cmds = _cmds(arm)
    assert P.CMD_HOME in cmds, "home() 未发 CMD_HOME(0x2A)"
    assert P.CMD_MOVE_J not in cmds, "home() 仍在用 MOVE_J 冒充"


def test_home_sends_empty_payload(offline_arm):
    """固件 CMD_HOME 不检查长度, 约定空载荷。"""
    arm = offline_arm
    arm.enable()
    arm.home()
    payloads = [p for c, p in list(arm._tr.tx_log) if c == P.CMD_HOME]
    assert payloads == [b""]


def test_home_waits_until_arrived(offline_arm):
    """home() 要等到位 (与 movej 同一套到位判定), 不能发完就返回。"""
    arm = offline_arm
    arm.enable()
    st = arm.home()
    assert st is not None
    assert max(abs(v) for v in st.q) < 0.03


def test_home_reports_unsupported_on_old_firmware(offline_arm):
    """老固件 (无 0x2A) 回 ERR{0x2A,0x00} -> 明确报「固件不支持」而不是含糊的超时。"""
    arm = offline_arm
    arm.enable()
    arm._tr.unknown_cmds.add(P.CMD_HOME)
    with pytest.raises(UnsupportedByFirmwareError) as ei:
        arm.home()
    assert ei.value.cmd == P.CMD_HOME


def test_home_no_longer_takes_speed(offline_arm):
    """破坏性变更: 速度由固件写死 0.10, speed 参数已移除 (误用须立刻炸)。"""
    arm = offline_arm
    arm.enable()
    with pytest.raises(TypeError):
        arm.home(speed=0.3)
