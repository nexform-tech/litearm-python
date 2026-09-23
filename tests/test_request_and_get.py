"""同一帧节流 (`Arm._tx_allowed`) —— **取帧模板已随读路径重构删除**。

⚠ 本文件原先叫「Task 5: 取帧收口 —— `_request_and_get` 模板 + 同帧节流」。2026-09-22 的
读路径重构把 `_request_and_get` / `_read_frames` / `_read_one` 整条取帧链**删掉**了
（SDK 现在只有一条读线程 `_Ack._reader_loop`，调用方只等队列），于是：

* timeout 三态 / 每一拍夹紧 / `_MIN_FETCH_S` 那几节 —— **机制不存在了**，删；
  其中仍然成立的契约（`timeout < 0` 抛 `InvalidCommandError`、单帧非阻塞探、
  取帧口只有一个）已搬到 `tests/test_frame_ownership.py`；
* **同帧节流那两节原地保留** —— `_tx_allowed` 挂在唯一的写口 `Arm._raw_write` 上，
  与读路径无关，本次未动。删掉整个文件会连它们一起丢掉。
"""
from __future__ import annotations

import time

import pytest

from litearm import _protocol as P
from litearm.errors import ArmIsInDfuError, InvalidCommandError


# ---------------------------------------------------------------------------
# 3. 同帧节流 (`min_interval`) —— 机制本身
# ---------------------------------------------------------------------------

def test_identical_frames_are_throttled_within_the_interval(offline_arm):
    """同 cmd + **逐字节相同**的载荷在 `min_interval` 内重复 ⇒ 不发, 且**计数**。"""
    arm = offline_arm
    payload = P.pack_f32s([0.0] * arm.n) + b"\x00\x00\x00\x00"
    arm._set_tx_repeat_min_interval(P.CMD_MOVE_JS, 0.5)

    arm._raw_write(P.CMD_MOVE_JS, payload)
    n_after_first = len(arm._tr.stamps_of(P.CMD_MOVE_JS))

    arm._raw_write(P.CMD_MOVE_JS, payload)          # 窗口内重复 ⇒ 丢
    arm._raw_write(P.CMD_MOVE_JS, payload)

    assert len(arm._tr.stamps_of(P.CMD_MOVE_JS)) == n_after_first, "重复帧没被节流"
    assert arm._tx_throttled_frames == 2, (
        "节流丢帧**不静默**: 每丢一帧必须计一次 (否则'指令没出去'完全不可观测)")


def test_a_changed_payload_is_never_throttled(offline_arm):
    """载荷变了就**照发** —— 伺服换目标必须能出去 (节流只吃逐字节相同的重复)。"""
    arm = offline_arm
    arm._set_tx_repeat_min_interval(P.CMD_MOVE_JS, 5.0)

    arm._raw_write(P.CMD_MOVE_JS, P.pack_f32s([0.1] * arm.n) + b"\x00\x00\x00\x00")
    arm._raw_write(P.CMD_MOVE_JS, P.pack_f32s([0.2] * arm.n) + b"\x00\x00\x00\x00")

    assert len(arm._tr.stamps_of(P.CMD_MOVE_JS)) == 2, (
        "换了载荷的新目标被节流丢掉了 —— 伺服会停在旧目标上")
    assert arm._tx_throttled_frames == 0


def test_the_throttle_lets_a_repeat_through_after_the_interval(offline_arm):
    """过了间隔, 同样的帧**照发** (节流是"短时间内不重发", 不是"永不重发")。"""
    arm = offline_arm
    payload = b"\x00" * 8
    arm._set_tx_repeat_min_interval(P.CMD_MOVE_MIT, 0.05)

    arm._raw_write(P.CMD_MOVE_MIT, payload)
    time.sleep(0.06)
    arm._raw_write(P.CMD_MOVE_MIT, payload)

    assert len(arm._tr.stamps_of(P.CMD_MOVE_MIT)) == 2
    assert arm._tx_throttled_frames == 0


def test_the_terminal_state_guard_is_not_short_circuited_by_the_throttle(offline_arm):
    """⚠ 终态守卫 (`_reject_if_in_dfu`) **排在节流之前** —— 终态必须**响亮**, 不许静默。

    `_raw_write` 里两道口的顺序是有射程的: 帧被节流丢掉时返回 `None` (什么都不发生),
    而终态下**任何**写都必须抛 `ArmIsInDfuError` (那是"结构性"的: 终态对象不会再发出
    帧, 而且这件事必须可判定)。若节流排在前面, "终态 + 一条重复载荷"就会静默无声 ——
    调用方看不出自己已经在一个废掉的会话上。故顺序是守卫在前。
    """
    arm = offline_arm
    arm._set_tx_repeat_min_interval(P.CMD_ZERO_G, 5.0)
    arm._raw_write(P.CMD_ZERO_G, b"\x01")           # 先发一条, 让表里有"上一帧"
    arm.enter_dfu()                                 # 终端态 (此后本对象不可用)

    with pytest.raises(ArmIsInDfuError):
        arm._raw_write(P.CMD_ZERO_G, b"\x01")       # 逐字节相同 ⇒ 节流会想丢它
    assert arm._tx_throttled_frames == 0, (
        "终态那条被节流**静默**丢掉了 —— 终态守卫必须排在节流之前")


def test_a_dropped_frame_does_not_clear_the_cart_queue(offline_arm):
    """⚠ 被节流丢掉的帧**不算发出去过** ⇒ 清队钩子**不许**跑。

    `_raw_write` 里那道清队钩子 (`_CART_CLEARS_UPON`) 的理据是"固件**收到**这个命令就会
    `cart_invalidate_before_motion()`" —— 帧压根没上 USB 时固件的状态一点没变, 此时清队
    反而把一条**还有效**的在途请求作废掉。故节流判断必须排在清队**之前**。
    """
    arm = offline_arm
    arm._set_tx_repeat_min_interval(P.CMD_ZERO_G, 5.0)

    arm._raw_write(P.CMD_ZERO_G, b"\x01")           # 第一条: 真发出去 (顺带清队)
    tok = arm._cart.register()

    arm._raw_write(P.CMD_ZERO_G, b"\x01")           # 第二条: 被节流 ⇒ 固件侧什么都没发生

    assert arm._tx_throttled_frames == 1
    assert arm._cart.pending == 1, (
        "被节流丢掉的帧触发了清队 —— 固件那边这条在途请求其实还有效")
    assert tok.lost is None


# ---------------------------------------------------------------------------
# 4. ⚠⚠ 安全 —— 伺服/保活路径**默认不被节流** (本轮最要紧的一格)
# ---------------------------------------------------------------------------

def test_moving_paths_are_not_throttled_by_default(offline_arm):
    """⚠⚠ `move_js` / `send_mit` 这类 ≥10Hz 重发的路径**默认必须不被节流**。

    它们靠**重发**维持固件 0.1s 命令看门狗 (`fail-soft` = 丢重力前馈); 节流会把重发
    丢掉 ⇒ 伺服断流 / 保活断流。判据取**行为** (真发出去的帧数), 不取配置
    (配置可以改对而行为照错): 3 次逐字节相同的 `move_js` / `send_mit` ⇒ 线上必须
    **3** 条。

    ⚠ 本用例在改动前后**都是绿的** (它钉的是"默认关闭"这个**不变量**, 不是新功能) ——
    它的牙由变异证明: 把 `_tx_repeat_min_interval` 改成对 0x03/0x04 预置一个间隔,
    它立刻红在下面那两句计数上。
    """
    arm = offline_arm
    q, dq = [0.1] * arm.n, [0.0] * arm.n

    for _ in range(3):
        arm.move_js(q, dq=dq)                       # 0x03: 伺服
    for _ in range(3):
        arm.send_mit(0, 0.1, 0.0, 50.0, 2.0, 0.0)   # 0x04: 透传

    assert len(arm._tr.stamps_of(P.CMD_MOVE_JS)) == 3, (
        "伺服重发被节流掉了 —— 固件 0.1s 看门狗会 fail-soft")
    assert len(arm._tr.stamps_of(P.CMD_MOVE_MIT)) == 3, (
        "透传重发被节流掉了 —— 同上")
    assert arm._tx_repeat_min_interval == {}, (
        "节流间隔表默认非空 —— 任何一条命令都可能被静默节流")


def test_enabling_the_throttle_on_the_keepalive_drops_it(offline_arm):
    """⚠⚠ 反侧: **一旦**对保活 `0x06` 打开节流, 重发真的会被丢掉。

    这条与上一条合起来才说明"默认关闭"为什么是**安全相关**的开关 (而不是一个
    无害的省流旋钮) —— 同时它也把上一条的牙交代清楚: 机制真的会吃帧。
    """
    arm = offline_arm
    arm._set_tx_repeat_min_interval(P.CMD_ZERO_G, 5.0)

    for _ in range(3):
        arm._raw_write(P.CMD_ZERO_G, b"\x01")

    assert len(arm._tr.stamps_of(P.CMD_ZERO_G)) == 1, (
        "对 0x06 打开节流之后重发没被丢 —— 这条用例失去了意义 (机制没生效)")
    assert arm._tx_throttled_frames == 2


def test_the_throttle_setter_validates_its_arguments(offline_arm):
    """旋钮自己守门: 负间隔 / 越界 cmd 直接拒 (不然它会变成"静默打开"的入口)。"""
    arm = offline_arm
    with pytest.raises(InvalidCommandError):
        arm._set_tx_repeat_min_interval(P.CMD_MOVE_JS, -1.0)
    with pytest.raises(InvalidCommandError):
        arm._set_tx_repeat_min_interval(P.CMD_MOVE_JS, float("nan"))

    arm._set_tx_repeat_min_interval(P.CMD_MOVE_JS, 0.5)
    assert arm._tx_repeat_min_interval == {P.CMD_MOVE_JS: 0.5}
    arm._set_tx_repeat_min_interval(P.CMD_MOVE_JS, 0.0)      # 0 = 关
    assert arm._tx_repeat_min_interval == {}


