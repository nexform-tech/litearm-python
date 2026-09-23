"""关节级运行时参数 (固件 0x22/0x23/0x24/0x36) —— 此前 SDK 有常量、零 API。

固件载荷约定 (`usb_cmd_dispatch` 的 `case CMD_SET_JOINT_PARAM`(0x22) /
`CMD_SET_JOINT_LIMITS`(0x23) / `CMD_GET_JOINT_PARAM`(0x24) / `CMD_PARAM_RESET`(0x36),
`usb_cmd.c:611-664` 及 `:677-712`):
  0x22 SET_JOINT_PARAM  : idx(1B) + kp,kd,tau_max(f32×3) = 13B; ERR 0x01=长度 / 0x02=idx越界或值非法
  0x23 SET_JOINT_LIMITS : idx(1B) + q_min,q_max(f32×2)  =  9B; ERR 同上
  0x24 GET_JOINT_PARAM  : idx(1B) -> RSP_JOINT_PARAM(0x49) = idx + kp,kd,tau_max,q_min,q_max (21B)
  0x36 PARAM_RESET      : 空载荷, 恢复出厂 + 失效 flash; **须失能** 否则 ERR{0x36,0x04}
"""
from __future__ import annotations

import struct

import pytest

from litearm import _protocol as P
from litearm.errors import CommandRejectedError, InvalidCommandError


def _payloads(arm, cmd):
    return [p for c, p in list(arm._tr.tx_log) if c == cmd]


def test_set_joint_param_payload_and_ack(offline_arm):
    arm = offline_arm
    arm.params.set_joint_param(2, kp=60.0, kd=3.0, tau_max=12.0)
    ps = _payloads(arm, P.CMD_SET_JOINT_PARAM)
    assert len(ps) == 1 and len(ps[0]) == 13
    idx = ps[0][0]
    kp, kd, tm = struct.unpack_from("<fff", ps[0], 1)
    assert (idx, kp, kd, tm) == (2, 60.0, 3.0, 12.0)


def test_set_joint_param_rejects_bad_index_locally(offline_arm):
    """idx 越界在本地就拦掉 —— 不必往返固件。"""
    arm = offline_arm
    with pytest.raises(InvalidCommandError):
        arm.params.set_joint_param(7, kp=1.0, kd=1.0, tau_max=1.0)
    with pytest.raises(InvalidCommandError):
        arm.params.set_joint_param(-1, kp=1.0, kd=1.0, tau_max=1.0)


def test_set_joint_limits_payload(offline_arm):
    arm = offline_arm
    arm.params.set_joint_limits(1, q_min=-1.5, q_max=1.5)
    ps = _payloads(arm, P.CMD_SET_JOINT_LIMITS)
    assert len(ps) == 1 and len(ps[0]) == 9
    lo, hi = struct.unpack_from("<ff", ps[0], 1)
    assert (ps[0][0], lo, hi) == (1, -1.5, 1.5)


def test_set_joint_limits_rejects_inverted_range(offline_arm):
    """q_min < q_max 是固件侧硬约束, 本地先拦一道以免浪费一次往返。"""
    arm = offline_arm
    with pytest.raises(InvalidCommandError):
        arm.params.set_joint_limits(0, q_min=1.0, q_max=-1.0)


def test_get_joint_param_decodes_21b_reply(offline_arm):
    arm = offline_arm
    arm.params.set_joint_param(3, kp=70.0, kd=4.0, tau_max=15.0)
    arm.params.set_joint_limits(3, q_min=-0.5, q_max=0.5)
    jp = arm.params.get_joint_param(3).value
    assert jp.idx == 3
    assert (jp.kp, jp.kd, jp.tau_max) == (70.0, 4.0, 15.0)
    assert (jp.q_min, jp.q_max) == (-0.5, 0.5)


def test_get_joint_param_propagates_firmware_error(offline_arm):
    arm = offline_arm
    arm._tr.err_override[P.CMD_GET_JOINT_PARAM] = 0x02
    with pytest.raises(CommandRejectedError):
        arm.params.get_joint_param(0)


def test_reset_factory_refused_while_armed(offline_arm):
    """固件对已使能/待使能态回 ERR{0x36,0x04} —— 绝不能静默当成功。"""
    arm = offline_arm
    arm.enable()
    with pytest.raises(CommandRejectedError) as ei:
        arm.params.reset_factory()
    assert ei.value.code == 0x04


def test_reset_factory_ok_when_disarmed(offline_arm):
    arm = offline_arm
    arm.params.set_joint_param(0, kp=99.0, kd=9.0, tau_max=9.0)
    arm.enable()
    arm.disable()
    arm.params.reset_factory()
    jp = arm.params.get_joint_param(0).value
    assert jp.kp != 99.0, "恢复出厂后仍读到写入值"


def test_joint_params_unsupported_on_old_firmware(offline_arm):
    arm = offline_arm
    arm._tr.unknown_cmds.add(P.CMD_GET_JOINT_PARAM)
    from litearm.errors import UnsupportedByFirmwareError
    with pytest.raises(UnsupportedByFirmwareError):
        arm.params.get_joint_param(0)
