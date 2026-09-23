"""零重力 (拖动示教) 保活 —— 固件 0x06 自带 watchdog_kick, **重发即保活**。

固件侧 (control_loop.c ctrl_accept_zero_g) 进入时 `watchdog_kick()`, 而
`watchdog_check()` 的超时是 `watchdog_timeout_s = 0.10s`; `SET_MOTION_MODE` /
`SET_SPEED_PERCENT` 都刻意不 kick, 所以**没有别的命令能维持非 MOVE_J 模式**。
旧 SDK 只单发一次 -> 0.1s 后模式被掐回 fail-soft, 而调用方只看到 ACK (静默失败)。
"""
from __future__ import annotations

import threading
import time

import pytest

from litearm import _protocol as P
from litearm.errors import InvalidCommandError, TransportError

#: 固件看门狗 0.10s; SDK 默认保活周期须留 ≥2 倍余量
_KEEPALIVE_PERIOD = 0.04


def _zg_payloads(arm):
    """桩收到的全部 0x06 载荷序列 (含保活线程的并发写)。"""
    return [p for c, p in list(arm._tr.tx_log) if c == P.CMD_ZERO_G]


def test_zero_g_starts_and_resends_within_keepalive_window(offline_arm):
    """进入后必须周期重发 0x06(on=1), 间隔远小于固件 0.10s 超时。"""
    arm = offline_arm
    with arm.zero_g():
        assert arm.zero_g_active is True
        time.sleep(0.30)
    ons = [p for p in _zg_payloads(arm) if p == b"\x01"]
    assert len(ons) >= 5, (
        f"0.30s 内只发了 {len(ons)} 次 0x06(on=1) —— 保活未生效, "
        f"固件 0.10s 后会把模式掐回 fail-soft")


def test_zero_g_exit_sends_off_once_then_stops(offline_arm):
    """退出: 最后一条 0x06 必须是 on=0, 且之后不再有保活帧、线程已回收。"""
    arm = offline_arm
    with arm.zero_g():
        time.sleep(0.15)
    assert arm.zero_g_active is False
    payloads = _zg_payloads(arm)
    assert payloads[-1] == b"\x00", "退出未发 0x06(on=0)"

    n = len(payloads)
    time.sleep(0.20)
    assert len(_zg_payloads(arm)) == n, "退出后仍在发保活帧"
    assert not [t for t in threading.enumerate() if t.name.startswith("litearm-zero_g")], \
        "保活线程未回收"


def test_other_write_commands_rejected_while_zero_g_active(offline_arm):
    """保活期间拒绝其它**动作类**命令 —— 它们会改写模式/看门狗, 与保活互相打架。"""
    arm = offline_arm
    with arm.zero_g():
        with pytest.raises(InvalidCommandError):
            arm.movej([0.1] * 7)
        with pytest.raises(InvalidCommandError):
            arm.set_speed(50)          # 会改写全局调速器
        with pytest.raises(InvalidCommandError):
            arm.enable()
    arm.set_speed(50)                  # 退出后恢复可用


def test_safety_actions_bypass_zero_g_guard(offline_arm):
    """急停/失能是「降能量」方向的安全动作 —— 保活期间也必须永远可达, 不得被守卫挡住。"""
    arm = offline_arm
    with arm.zero_g():
        arm.emergency_stop()           # 必须发出, 不得抛 InvalidCommandError
        arm.disable()


def test_state_reads_still_work_while_zero_g_active(offline_arm):
    """查询类命令不走守卫 —— 拖动示教期间仍要能读状态。"""
    arm = offline_arm
    with arm.zero_g():
        st = arm.get_state(refresh=True).value
        assert st is not None and st.n == arm.n
        assert arm.get_tcp().value is not None


def test_close_stops_keepalive_and_releases_thread(offline_arm):
    """close() 必须先收干净保活线程 —— 残留线程会在解释器退出时打哑 CDC。"""
    arm = offline_arm
    arm.zero_g()
    time.sleep(0.10)
    arm.close()
    assert arm.zero_g_active is False
    assert not [t for t in threading.enumerate() if t.name.startswith("litearm-zero_g")], \
        "close() 后保活线程仍在运行"


def test_stop_without_start_sends_nothing(offline_arm):
    """幂等退出 ≠ 无中生有: 从未进入过就不该发任何 0x06。"""
    arm = offline_arm
    arm.zero_g_stop()
    arm.zero_g_stop()
    assert _zg_payloads(arm) == [], "未进入零重力却发出了 0x06"


def test_keepalive_write_failure_is_raised_not_silent(offline_arm):
    """保活写失败 (CDC 掉线) 不能在退出时静默吞掉 —— 臂已脱离零重力。"""
    arm = offline_arm
    arm._tr.zg_fail_after = 3
    with pytest.raises(TransportError):
        with arm.zero_g():
            time.sleep(0.40)
    assert arm.zero_g_active is False
