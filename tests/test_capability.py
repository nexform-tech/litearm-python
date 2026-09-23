"""能力判定 —— 「这台固件有没有这条命令」必须靠 ERR 语义, 不能靠版本号。

固件 `usb_cmd.c` 全树**只有一处**产生 `ERR{cmd,0x00}`: `default` 分支 (未实现命令)。
已实现命令的错误码一律落在 `0x01..0x05` (长度/参数/未使能/已武装/忙)。于是
`ERR{cmd,0x00}` 就是「固件无此命令」的唯一且稳定的哨兵。

这条判据比版本门可靠: 固件 HEAD 自报 `Litearm1.5.2-7J`, 但 `CMD_HOME`(09-06)、
`CMD_LOG_*`(09-09)、`CMD_ZERO_G`(09-12) 都是 `LITEARM_FW_VERSION` 定为 1.5.2(09-08)
**之后**才合入的 —— 版本号根本区分不出这两批固件。
"""
from __future__ import annotations

import pytest

from litearm import _protocol as P
from litearm.errors import CommandRejectedError, UnsupportedByFirmwareError


def test_unknown_command_maps_to_unsupported(offline_arm):
    """固件回 ERR{cmd,0x00} -> 必须抛 UnsupportedByFirmwareError 并带上 cmd。"""
    arm = offline_arm
    arm._tr.unknown_cmds.add(P.CMD_PARAM_SAVE)
    with pytest.raises(UnsupportedByFirmwareError) as ei:
        arm.save_params()
    assert ei.value.cmd == P.CMD_PARAM_SAVE
    assert ei.value.code == 0x00


def test_unsupported_is_a_command_rejected_subclass(offline_arm):
    """既有捕获 CommandRejectedError 的调用方不得被这次细分破坏。"""
    arm = offline_arm
    arm._tr.unknown_cmds.add(P.CMD_PARAM_SAVE)
    with pytest.raises(CommandRejectedError):
        arm.save_params()
    assert issubclass(UnsupportedByFirmwareError, CommandRejectedError)


def test_other_error_codes_are_not_unsupported(offline_arm):
    """ERR{cmd,0x03}(未使能) 一类是**真拒绝**, 不能误判成「固件不支持」。"""
    arm = offline_arm
    arm._tr.err_override[P.CMD_PARAM_SAVE] = 0x03
    with pytest.raises(CommandRejectedError) as ei:
        arm.save_params()
    assert not isinstance(ei.value, UnsupportedByFirmwareError)
    assert ei.value.code == 0x03


def test_unsupported_during_motion_wait(offline_arm):
    """运动等待路径 (pump/_arrive) 里的 ERR 也要走同一套映射。"""
    arm = offline_arm
    arm._tr.unknown_cmds.add(P.CMD_MOVE_J)
    with pytest.raises(UnsupportedByFirmwareError):
        arm.movej([0.1] * 7)


def test_unsupported_during_status_read(offline_arm):
    """状态读取路径 (_read_status) 里的 ERR 同样要走映射。"""
    arm = offline_arm
    arm._tr.unknown_cmds.add(P.CMD_GET_STATUS)
    arm._tr.err_override.pop(P.CMD_GET_STATUS, None)
    with pytest.raises(UnsupportedByFirmwareError):
        arm.get_status_now(timeout=0.2)
