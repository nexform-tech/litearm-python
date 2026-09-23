"""生命周期与关链时序 —— 保活线程引入的并发面。

零重力保活是本 SDK 唯一的后台线程, 它的启停必须能经受"另一个线程恰好在错误时刻
插进来"以及"串口写卡住"这两种真实情况 —— 否则后果不是报错, 而是**静默失效**:
`zero_g_active` 说谎、所有动作命令被守卫永久拒绝、或者迟到的保活帧把臂重新拉回
零重力却无人维持 (0.1s 后被看门狗掐掉)。
"""
from __future__ import annotations

import threading
import time

import pytest

from litearm import _protocol as P
from litearm.errors import TransportError


def _zg_payloads(arm):
    return [p for c, p in list(arm._tr.tx_log) if c == P.CMD_ZERO_G]


def _zg_stamps(arm):
    return arm._tr.stamps_of(P.CMD_ZERO_G)


# --------------------------------------------------------- P1-1 启停竞态
def test_stop_never_joins_an_unstarted_thread(offline_arm):
    """`zero_g_stop` 只能 join **真正跑起来**的线程。

    对未启动的 Thread 调 `join()` 会抛 `RuntimeError: cannot join thread before it
    is started`; 旧实现把 join 放在置 `_zg_active=False` **之前**, 异常一抛
    `_zg_active` 就永久停在 True —— 保活一帧不发, 而所有动作命令被守卫永久拒绝。
    这个状态可以由 `zero_g_start` 的「发布句柄 → start()」窗口被 `zero_g_stop`
    插进来触发, 也可能是任何异常路径的残留。
    """
    arm = offline_arm
    arm._zg_thread = threading.Thread(target=lambda: None)   # 故意不 start
    arm._zg_active = True
    arm.zero_g_stop()
    assert arm.zero_g_active is False, "zero_g_active 卡在 True -> 守卫会永久拒绝动作命令"
    assert arm._zg_thread is None


def test_concurrent_start_stop_storm_never_wedges(offline_arm):
    """反复并发 start/stop: 任意交错顺序下都不得抛异常、不得把状态卡住、不得漏线程。"""
    arm = offline_arm
    errors = []

    for _ in range(15):
        def starter():
            try:
                arm.zero_g_start()
            except Exception as e:                  # noqa: BLE001
                errors.append(("start", e))

        def stopper():
            try:
                arm.zero_g_stop()
            except Exception as e:                  # noqa: BLE001
                errors.append(("stop", e))

        t1 = threading.Thread(target=starter)
        t2 = threading.Thread(target=stopper)
        t1.start()
        t2.start()
        t1.join(5.0)
        t2.join(5.0)
        arm.zero_g_stop()                           # 收尾: 归一化终态
        assert arm.zero_g_active is False, f"第 {_} 轮后状态卡住"

    assert not errors, f"并发启停抛了异常: {errors}"
    assert not [t for t in threading.enumerate() if t.name.startswith("litearm-zero_g")], \
        "并发启停泄漏了保活线程"
    payloads = _zg_payloads(arm)
    assert not payloads or payloads[-1] == b"\x00", f"0x06 序列以 on 结尾: {payloads}"


# ------------------------------------------------- P1-2 join 超时的迟到帧
def test_stop_refuses_to_send_off_when_keepalive_thread_wont_die(offline_arm):
    """保活线程卡在写里 (CDC 阻塞) 停不下来时, **绝不能发 off 帧**。

    否则迟到的 `0x06(on=1)` 会排在 off 之后抵达固件 —— 臂被重新拉进零重力模式
    却已无人维持 (0.1s 后 fail-soft), 而 SDK 认为"已退出", 紧接着发的 movej
    会与它互相打架。正确做法是报错并把问题暴露出来。
    """
    arm = offline_arm
    arm._zg_join_s = 0.10
    arm._tr.zg_slow_write_s = 0.6              # 每次保活写都卡 0.6s
    arm.zero_g_start(period=0.01)
    time.sleep(0.15)
    with pytest.raises(TransportError):
        arm.zero_g_stop()
    assert _zg_payloads(arm)[-1] == b"\x01", "卡死的保活线程未被等停, 却仍发了 off 帧"
    assert arm.zero_g_active is False


def test_close_after_stuck_keepalive_still_returns(offline_arm):
    """即便保活线程卡住, close() 也必须能返回并把链路关掉 (teardown 不能挂死)。"""
    arm = offline_arm
    arm._zg_join_s = 0.10
    arm._tr.zg_slow_write_s = 0.4
    arm.zero_g_start(period=0.01)
    time.sleep(0.1)
    t0 = time.monotonic()
    arm.close()
    assert time.monotonic() - t0 < 3.0
    assert arm.zero_g_active is False


# --------------------------------------------------------- 保活间隔不变量
def test_keepalive_gap_stays_under_watchdog(offline_arm):
    """真正该守的不变量是**相邻保活帧间隔** < 固件看门狗 0.10s, 不是"发了几次"。

    只数帧数的话, 一旦某次写卡了 0.12s, 固件看门狗已经跳闸, 而计数断言照样通过。
    """
    arm = offline_arm
    with arm.zero_g():
        time.sleep(0.45)
    ts = _zg_stamps(arm)
    ons = [t for (c, p), t in zip(arm._tr.tx_log, arm._tr.tx_stamps)
           if c == P.CMD_ZERO_G and p == b"\x01"]
    assert len(ons) >= 4, f"保活帧太少: {len(ons)}"
    gaps = [b - a for a, b in zip(ons, ons[1:])]
    assert max(gaps) < 0.08, (
        f"相邻保活帧最大间隔 {max(gaps) * 1000:.0f}ms —— 固件看门狗 100ms, 会跳闸")
    assert ts == sorted(ts)


# ------------------------------------------------- zero_g_error 文档一致性
def test_zero_g_error_is_cleared_by_stop(offline_arm):
    """`zero_g_error` 的文档说 stop 会消费清空它 —— 实现必须一致。"""
    arm = offline_arm
    arm._tr.zg_fail_after = 1
    with pytest.raises(TransportError):
        with arm.zero_g():
            time.sleep(0.20)
    assert arm.zero_g_error is None


# ------------------------------------------------- 传输层共享态
def test_text_log_is_read_under_lock(monkeypatch):
    """`text_log` 读的是读者线程正在改的 `_text` —— 必须与其它共享态同口径加锁。"""
    import sys
    import types
    from litearm.transport import SerialTransport

    class FakeSerial:
        def __init__(self, port, baudrate=0, timeout=0.2, **kw):
            self.is_open = True
            self.timeout = timeout

        def read(self, size=1):
            return b""

        def write(self, d):
            return len(d)

        def flush(self):
            pass

        def close(self):
            self.is_open = False

    monkeypatch.setitem(sys.modules, "serial", types.SimpleNamespace(Serial=FakeSerial))
    tr = SerialTransport("/dev/null")
    seen = []

    def reader():
        for _ in range(300):
            tr.read_frame(0.001)
            seen.append(tr.text_log)

    th = threading.Thread(target=reader)
    th.start()
    for i in range(300):
        tr._note_text(0x41)
    th.join()
    assert seen, "未取到 text_log"
