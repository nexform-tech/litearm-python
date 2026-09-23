"""Task 5: 笛卡尔到位等待 (`CART_BUSY` bit10) + `wait=True`。

**为什么**: 三条入口从前只等到**规划结果** (`0x4E`) 就返回 —— 而 `0x4E` 只说明"固件收了
这条路点、规划出来了", 不说明**臂动没动、动完没有**。调用方拿到 `CartPlan` 时臂可能正
在半路。本 Task 让入口默认等到**停稳**再返回, 并把收尾事实填进 `CartPlan` 的四个字段。

**本文件钉死的判据**:

1. **主判据**: 可信帧上看到 `bit10` **1→0**, **且** `q` 静止判据成立;
2. **降级分支**: 可信帧上**始终**没见过 `bit10=1` 但 `q` 静止 ⇒ 同样 "停稳" (`settled` 还要过下一关),
   `started_busy=False`, **不抛异常** ("不能判失败");
3. **`q` 静止判据是双判据**: 相邻两帧 Δq < `q_tol` 连续 `arrive_frames` 帧 **且**
   `dq` 的 max-norm < `dq_tol` —— 只抄一半会宽松得多 (`bit10` 落 0 那一刻臂可能还在爬);
4. **故障立刻抛** (`MotorFaultError`) —— 与 `_arrive` 同款, 不能悄悄返回 `settled=False`;
5. **上限是 `move_timeout`** (`MotionTimeoutError`) —— EMERGENCY / 未使能期间固件的 cart
   FSM 完全冻结、`CART_BUSY` **常亮**, 只等 `bit10` 落 0 会耗满超时, 所以上限存在;
   ⚠ **"常亮"只对"安全锁存 + `RUNNING`/`READY`"成立** (见 `cart._wait_settled`):
   `PLANNING` ≤3s 自解, 而**命令**引起的失能/急停反而**会**清掉它 —— 别读成三条路径都亮;
6. **新鲜度闸**: 只信"命令发出之后生成"的状态帧 (见下), 陈旧帧**既不判到位也不判故障**;
7. **`wait=False`** 时四个字段回到"未等待"默认值, 且**一帧都不多读**;
8. **断连唤醒**: `close()` 期间在等的线程必须**带失败原因**立刻退出, 不能挂到 `move_timeout`;
9. ⚠⚠ **"停稳" ≠ "停在了目标上"**: `settled=True` 还要求停稳后回读的实际 TCP 与本次
   请求的目标位姿对得上 —— 否则一条**被别的运动作废**的轨迹 (收尾同样是 `bit10=0` +
   `q` 静止) 会被报成成功 (**假成功**方向);
10. ⚠ **并发读者不许把到位等待饿死**: 判据认的是**帧**, 不是"谁读的帧" —— 只认自己读到的
    帧的实现会被 `get_state()`/`get_tcp()` 抢帧抢到超时 (**假失败**方向)。

⚠ **关于新鲜度闸的一条实测结论 (写在这里免得后人再推一遍)**: 计划给的判据是
"`seq0` = 发命令**之前**已知的最高状态帧序号, 只信 `(seq - seq0) & 0xFFFF ∈ (0, 32768)`"。
在**当前**取帧结构下 **FIFO 链路交付不出**这个形状的帧 —— 命令之前生成的状态帧在流里
一律排在**那条命令的 ACK 之前**, 而 `_request_and_wait` 必读的一条就是 ACK
(`expect(RSP_ACK)` 会一路读到它), 于是它们**不可能**留到到位等待里。换句话说:
"不会被陈旧帧判成到位"这条性质在本结构下**由 ACK 排水顺带保证**, 那道闸平时**不会响**;
它是**不变量守卫** (重放/换取帧口/将来有人把水线取错时才响), 而不是现场可复现的故障形状。
`test_stale_frames_*` 两条因此用**伪造**的陈旧帧 (桩里直接注入 seq 落后于水线的一帧) 来
钉住这个不变量 —— 别把它读成"真链路上会这样"。
"""
from __future__ import annotations

import struct
import threading
import time
from typing import Optional

import pytest

from fake_serial import FakeTransport, _status
from litearm import Arm, _protocol as P
from litearm import state as ST
from litearm.cart import CartPlan, _wait_settled
from litearm.errors import (MotionTimeoutError, MotorFaultError,
                                 NotConnectedError)

#: 桩的初始 TCP (`fake_serial` 的 `self.pose`), `move_c` 的 `pose_start` 必须与它一致。
TCP0 = (0.30, 0.0, 0.35, 0.0, 0.0, 0.0)

#: 桩的 `CART_BUSY` 帧序 (见 `FakeTransport._stamp_status`): 收到 `0x4E` 后**第 2..7 帧**
#: 置位、**第 8 帧**落 0 ⇒ 一条"先见 1 再见 0"的到位等待恰好吃掉 **8** 帧状态。
_BUSY_WINDOW_FRAMES = 8


@pytest.fixture
def cart_arm(monkeypatch):
    """可配置的桩工厂 —— 与 `test_cart_protocol.make_arm` 同款 (在 `connect()` 前装桩)。"""
    import litearm.arm as arm_mod

    def _make(cls=FakeTransport, move_timeout: float = 15.0, **cfg):
        def factory(port="fake", timeout=0.2, **_ignored):
            t = cls(port=port, timeout=timeout)
            for k, v in cfg.items():
                setattr(t, k, v)
            return t
        monkeypatch.setattr(arm_mod, "SerialTransport", factory)
        from litearm import Arm as _Arm
        return _Arm(port="fake", move_timeout=move_timeout).connect()
    return _make


class _ScriptedStatus(FakeTransport):
    """把**交付给 SDK 的**状态帧换成脚本形状的桩 —— 每条判据各配一份脚本。

    * `q_script` —— 逐帧取用的关节角: 列表形式用完后停在最后一组, **可调用**形式
      (`f(i) -> q`) 则每帧现算 (给"永远不停"的形状用); 缺省 = 桩自己的 `q`;
    * `dq_value` —— 每帧的 dq (缺省 0.0);
    * `flags`    —— 每帧的基础 flags (缺省 0; 置 1 = FAULT 位);
    * `no_busy`  —— True 时父类那股 `CART_BUSY` 帧序被压掉 (模拟"`bit10` 没上报");
    * `silent`   —— True 时干脆不交帧 (给"只能等到超时"/"断连唤醒"两条用例)。

    ⚠ 仍然走父类的 `_stamp_status`: `seq` 递增与 `CART_BUSY` 帧序与真桩完全一致 ——
    换了盖章口就等于把被测的取帧语义一起换掉了。
    """

    def __init__(self, *a, **kw) -> None:
        super().__init__(*a, **kw)
        self.q_script: list = []
        self.dq_value = 0.0
        self.extra_flags = 0
        self.no_busy = False
        self.silent = False
        self.frames_served = 0

    def _scripted_q(self):
        i = self.frames_served
        self.frames_served += 1
        if callable(self.q_script):
            return list(self.q_script(i))
        if not self.q_script:
            return list(self.q)
        return list(self.q_script[min(i, len(self.q_script) - 1)])

    def read_frame(self, timeout=None):
        if self._resp:                     # ACK / 0x4E 照旧先走父类
            return super().read_frame(timeout)
        if self.silent:
            time.sleep(0.02)
            return None
        # ⚠⚠ **[2026-09-22] 脚本按「节拍」推进, 不再"读一次推进一格"**。
        # 从前 SDK 的读是**调用方驱动**的 ⇒ "谁读谁推进"成立; 现在 SDK 有**读线程**
        # (`_Ack._reader_loop`), 它会把脚本在微秒内读完, 而被测的到位等待**一帧都看不到**
        # ⇒ 判据全乱 (`started_busy=False` 这种"假到位"就是这么来的)。
        # 真机上一个状态帧**按时间产生、与谁在读无关** —— 桩必须建模这一点:
        # 一个节拍 (`auto_status_period`) 才推进一格, 节拍内重复读返回 `None`
        # (读线程据此退避, 也不空转)。
        now = time.monotonic()
        if now - self._auto_status_last < self.auto_status_period:
            return None
        self._auto_status_last = now
        if self.no_busy:
            self.cart_busy_seq = None
        q = self._scripted_q()
        body = P.unpack_frame(_status(q, dq=[self.dq_value] * self.n,
                                      flags=self.extra_flags, n=self.n))[1]
        return self._stamp_status((P.RSP_STATUS, body))


class _StaleFrames(FakeTransport):
    """在 `0x4E` **之后**先交付几帧"命令之前生成"的状态帧 (伪造 `seq` 落后于水线)。

    ⚠ **伪造是刻意的** (见模块 docstring 最后一段): 本结构下 ACK 排水已经把这形状挡在
    到位等待之外, 所以只能注入。注入点选在 `_resp` **排空之后** —— 否则它们会排在 ACK
    前面被 `expect` 吃掉, 那就什么都没测到。绕过 `_stamp_status` 是必需的: 盖章口会给
    每一帧换新的 `seq`, 一换就不再陈旧。

    `stale_fault` 为真时注入的那几帧带 FAULT 位 (给"陈旧帧**也不判故障**"用)。
    """

    def __init__(self, *a, **kw) -> None:
        super().__init__(*a, **kw)
        self.stale_count = 0
        self.stale_fault = False
        self.stale_pending: list = []
        self._planned = False

    def _push(self, frame: bytes) -> None:
        super()._push(frame)
        # ⚠ 帧头第 0 字节是 SOF, 命令码在第 1 字节 (`_protocol.pack_frame` 的布局)
        if len(frame) > 1 and frame[1] == P.RSP_CART_PLAN:
            self._planned = True

    def read_frame(self, timeout=None):
        if self.stale_pending and not self._resp:
            return self.stale_pending.pop(0)     # "缓冲里躺着的陈旧帧"先出来
        fr = super().read_frame(timeout)
        if self._planned and not self._resp and not self.stale_pending:
            # 规划帧刚被取走 -> 把"缓冲里躺着的陈旧帧"补上 (seq 落后于当前水线)
            self._planned = False
            k = self.status_seq - 5
            for i in range(self.stale_count):
                self.stale_pending.append(self._stale_frame((k + i) & 0xFFFF))
        return fr

    def _stale_frame(self, seq: int):
        flags = (1 if self.stale_fault else 0)
        body = struct.pack("<HH", flags | (1 << 6), seq)
        for _ in range(self.n):
            body += struct.pack("<fffff", 0.0, 0.0, 0.5, 30.0, 25.0) + b"\x00"
        body += struct.pack("<H", 0)
        return (P.RSP_STATUS, body)


# ---------------------------------------------------------------------------
# 1. 三条判据 —— 各一条
# ---------------------------------------------------------------------------

def test_main_criterion_settles_on_bit10_one_then_zero(offline_arm):
    """主判据: **先见 `bit10=1` 再见 `0`**, 且 `q` 静止 ⇒ `settled=True` + `started_busy=True`。

    桩的帧序是写死的 (`第 2..7 帧=1, 第 8 帧=0`) ⇒ 一次"等到停稳"恰好吃掉 **8** 帧状态帧。
    这个数字同时钉住两件事: 判据**没有提前收尾** (第 1 帧 `bit10=0` 且 `q` 静止, 只抄
    "没见过 1 也收"会在这里就返回), 也**没有白等** (第 8 帧一到就返回)。
    """
    arm = offline_arm
    tr = arm._tr
    plan = arm.move_l(TCP0, speed=0.3)
    # ⚠ **判据换了**：读线程一开，"到位等待吃掉几帧"不再是可观察量（帧由读线程消费，
    # 与等谁无关）。"没有提前收尾"改由 `started_busy` 钉 —— 它要求**亲眼见过** `bit10=1`，
    # 而"在第 1 帧（bit10=0 且 q 静止）就收"必定是 `started_busy=False`。
    # （"没有白等"那一半由 `_BUSY_WINDOW_FRAMES` 的桩帧序本身保证：第 8 帧才落 0。）
    assert plan.settled is True and plan.started_busy is True
    assert plan.q_final == list(tr.q), "q_final 不是收尾帧的 q"
    assert plan.settle_err_rad == 0.0, "桩停得纹丝不动, 抖动幅度应为 0"


def test_a_fault_on_a_credible_frame_aborts_immediately(cart_arm):
    """故障**必须**响: 可信帧上 `st.faulted` ⇒ 立刻 `MotorFaultError` (与 `_arrive` 同款)。

    悄悄返回 `settled=False` 不是选项 —— 调用方会以为"只是没停稳"而继续往下走。
    ⚠ 必须**立刻** (远小于 `move_timeout`): 故障位是**锁存**的, 再多等也只是白等。
    """
    arm = cart_arm(cls=_ScriptedStatus, move_timeout=5.0, extra_flags=1)
    t0 = time.monotonic()
    with pytest.raises(MotorFaultError) as ei:
        arm.move_l(TCP0, speed=0.3)
    assert time.monotonic() - t0 < 1.0, "故障位没让到位等待立刻退出"
    assert "FAULT" in str(ei.value)


def test_the_degrade_branch_settles_and_is_not_a_failure(cart_arm):
    """**没见过 1 不能判失败**: 可信帧上始终 `bit10=0` 但 `q` 静止 ⇒ 照样 `settled=True`。

    两个连带的点:

    * `started_busy=False` (如实填) 且**不抛异常** —— 规划短到两帧之间就跑完、或固件
      不报这一位, 都会落在这里;
    * `q` 静止判据比的是**相邻两帧**, 不是"与某个目标" —— 所以下面这组 `q` 取一个
      **离零位很远**的常值 (`[1.5]*7`): 笛卡尔路径在 PC 侧**没有目标 q** 可比, 是不是
      静止只能看它自己动没动。
    """
    arm = cart_arm(cls=_ScriptedStatus, no_busy=True, q_script=[[1.5] * 7])
    plan = arm.move_l(TCP0, speed=0.3)
    assert plan.settled is True and plan.started_busy is False
    assert plan.q_final == [1.5] * 7
    assert plan.settle_err_rad == 0.0


# ---------------------------------------------------------------------------
# 2. `q` 静止判据是**双判据** —— 只抄 `q_tol` 一半会宽松得多
# ---------------------------------------------------------------------------

def test_bit10_falling_to_zero_is_not_enough_without_a_static_q(cart_arm):
    """⚠ `bit10` 落 0 那一刻臂可能**还在爬** —— 主判据必须**同时**要求 `q` 静止。

    这里让 `q` 一直不动而 `dq` 一直超容差 (`0.5 > dq_tol=0.10`): 桩的 `bit10` 帧序照旧
    `1→0`, 但 `dq` 那条判据永远不成立 ⇒ 到位等待只能耗满 `move_timeout`。
    只抄 `q_tol` 一半的实现会在第 8 帧报"到位"。
    """
    arm = cart_arm(cls=_ScriptedStatus, move_timeout=0.4, dq_value=0.5)
    with pytest.raises(MotionTimeoutError):
        arm.move_l(TCP0, speed=0.3)


def test_a_q_that_keeps_moving_never_settles(cart_arm):
    """相邻两帧 Δq 超 `q_tol` 时同样不算静止 (另一半判据 —— `dq` 为 0 也没用)。"""
    # 每帧比上一帧多 0.5 rad: Δq 恒 > q_tol (0.03), 永不停止
    arm = cart_arm(cls=_ScriptedStatus, move_timeout=0.4, no_busy=True,
                   q_script=lambda i: [i * 0.5] * 7)
    with pytest.raises(MotionTimeoutError):
        arm.move_l(TCP0, speed=0.3)


def test_settle_err_rad_is_the_jitter_of_the_settling_frames(cart_arm):
    """⚠ `settle_err_rad` 是**新语义**: "结束时各轴 `q` 与 `q_final` 的差" (最后几帧的
    抖动幅度), **不是**"与终点指令的偏差" (固件原生路径下那个量物理不可得)。

    脚本 `0 → 0 → 0.02`: 第 3 帧时静止窗 (`arrive_frames=3`) 第一次填满, 于是收尾帧的
    `q_final = 0.02`, 而窗内与它的最大差 = `|0 - 0.02| = 0.02` (逐轴). 若实现成
    "与指令的偏差" 或恒 0, 这条会红。
    """
    # ⚠ 脚本改成**交替**（0 / 0.02 交替）而不是"0,0,0.02"：读线程一开，脚本按**时间**推进，
    # "`_wait_settled` 从第几帧开始采样"不再可控。交替的话**任意**连续 3 帧都含 0 与 0.02
    # ⇒ 抖动恒为 0.02，判据与起采样点无关。（|Δq|=0.02 < q_tol 0.03 ⇒ 仍算"静止"。）
    arm = cart_arm(cls=_ScriptedStatus, no_busy=True,
                   q_script=lambda i: [0.0 if i % 2 == 0 else 0.02] * 7)
    plan = arm.move_l(TCP0, speed=0.3)
    assert plan.settled is True
    assert plan.q_final == pytest.approx([0.02] * 7)
    assert plan.settle_err_rad == pytest.approx(0.02, abs=1e-6), (
        "抖动幅度应为窗内 q 与 q_final 的最大差 (0.02)")


# ---------------------------------------------------------------------------
# 3. 上限与 `wait=False`
# ---------------------------------------------------------------------------

def test_the_wait_is_bounded_by_move_timeout(cart_arm):
    """上限沿用既有旋钮 `move_timeout` (不新造字段) —— `bit10` **常亮**时只能耗满它。

    这就是"必须能退出"那条: EMERGENCY / 未使能期间固件的 cart FSM 完全冻结、
    `CART_BUSY` 常亮 (`cart_tick_timeout` 只处理 `RECV`/`PLANNING`), 只等 `bit10` 落 0
    会一直等下去 —— 有上限才退得出来 (另有 `st.faulted` 那条立刻抛)。
    """
    arm = cart_arm(cls=_ScriptedStatus, move_timeout=0.4, extra_flags=1 << 10)
    t0 = time.monotonic()
    with pytest.raises(MotionTimeoutError):
        arm.move_l(TCP0, speed=0.3)
    assert time.monotonic() - t0 < 3.0, "上限不是 move_timeout"


def test_wait_false_stops_at_the_plan_and_leaves_the_four_fields_unwaited(offline_arm):
    """`wait=False` 只等到 `0x4E`: 四个字段是"未等待"默认值, 且**一帧都不多读**。

    "一帧都不多读"由桩的 `cart_busy_seq` 钉住: 那条 `CART_BUSY` 帧序是在**交付第一帧**
    时才往前走一步的, 停在 0 就说明一帧都没读。
    """
    arm = offline_arm
    tr = arm._tr
    plan = arm.move_l(TCP0, speed=0.3, wait=False)
    assert isinstance(plan, CartPlan)
    assert (plan.started_busy, plan.settled, plan.q_final,
            plan.settle_err_rad) == (False, False, [], 0.0)
    # ⚠ **判据换了**：旧写法用 `tr.cart_busy_seq == 0`（"一帧状态都没读"）当代理 —— 读线程
    # 一开，状态帧**总会被读**，那个代理失去意义。改成**直接钉住没进到位等待**：
    # 让 `_wait_settled` 一被调用就失败，`wait=False` 若走到那里就红。
    import litearm.cart as _cart_mod
    _orig = _cart_mod._wait_settled
    try:
        _cart_mod._wait_settled = lambda *a, **k: pytest.fail(
            "`wait=False` 却进了到位等待")
        plan2 = arm.move_l(TCP0, speed=0.3, wait=False)
    finally:
        _cart_mod._wait_settled = _orig
    assert plan2.settled is False


def test_all_three_entries_take_wait(cart_arm):
    """三条入口都收 `wait` —— `wait=True` 填真值, `wait=False` 留默认值。"""
    arm = cart_arm()
    for fn in (lambda w: arm.move_l(TCP0, speed=0.3, wait=w),
               # ⚠ 起点取**实测 TCP**: 上一条命令 (哪怕是 `wait=False`) 一受理就起跑,
               # 桩的 TCP 早已不在 `TCP0` 上了 —— 拿常量当起点会被起点校验拒掉。
               lambda w: arm.move_c(arm.get_tcp().value, (0.30, 0.0, 0.40, 0.0, 0.0, 0.0),
                                    (0.32, 0.0, 0.40, 0.0, 0.0, 0.0),
                                    speed=0.3, wait=w),
               lambda w: arm.move_path([arm.get_tcp().value, (0.31, 0.0, 0.35, 0.0, 0.0, 0.0)],
                                       speed=0.3, wait=w)):
        off = fn(False)
        assert off.settled is False and off.started_busy is False
        on = fn(True)
        assert on.settled is True, "wait=True 没等到停稳"
        assert on.started_busy is True, "wait=True 没见过 CART_BUSY=1"
        assert on.q_final == list(arm._tr.q)


# ---------------------------------------------------------------------------
# 4. 新鲜度闸 —— 陈旧帧既不判到位, **也不判故障**
# ---------------------------------------------------------------------------

def test_stale_frames_cannot_settle_a_motion_that_has_not_stopped(cart_arm):
    """⚠ 陈旧帧**不许**判到位。

    注入 5 帧"命令之前生成"的帧 (伪造 `seq` 落后于水线, `bit10=0`、`q` 静止) ——
    没有这道闸时, 降级分支会在第 3 帧就报 `settled=True` 而 `started_busy=False`
    (即把**运动前的位置**报成"到位"); 有闸时它们被丢弃, 等到真的 `bit10 1→0` 才收尾。
    """
    arm = cart_arm(cls=_StaleFrames, stale_count=5)
    assert arm._tr.stale_count == 5
    plan = arm.move_l(TCP0, speed=0.3)
    assert plan.started_busy is True, (
        "被那几帧陈旧帧判成到位了 (它们 bit10=0 且 q 静止 —— 正是运动前的位置)")
    assert plan.settled is True


def test_stale_frames_cannot_raise_a_fault_either(cart_arm):
    """⚠ 陈旧帧**也不判故障** —— 故障是**锁存**的, 真故障会在可信帧上重现。

    用陈旧帧判故障只会造成**假中止** (一条正在正常跑的运动被一份早就过期的 FAULT 位
    喊停)。注入的这几帧带 FAULT 位: 没有闸时 `move_l` 立刻抛 `MotorFaultError`。
    """
    arm = cart_arm(cls=_StaleFrames, stale_count=5, stale_fault=True)
    plan = arm.move_l(TCP0, speed=0.3)          # 不抛 = 陈旧帧被丢了
    assert plan.settled is True and plan.started_busy is True


# ---------------------------------------------------------------------------
# 5. 断连唤醒 —— 在等的线程必须**带失败原因**退出, 不能挂到超时
# ---------------------------------------------------------------------------

def test_close_wakes_the_waiter_with_a_reason_instead_of_letting_it_time_out(cart_arm):
    """⚠ `Arm.close()` 期间在等到位的人必须**立刻**退出, 且原因可分辨。

    挂到 `move_timeout` 的话, 重连之后调用方拿到的是"**没到位**", 而真相是"**链路断了**"
    —— 归因完全不同 (前者会让人去查臂/调参数, 后者该去查线)。
    判据: 在**远小于** `move_timeout` 的时间内退出, 且异常类型是 `NotConnectedError`
    (`NotConnectedError` 与 `MotionTimeoutError` 是两回事)。
    """
    arm = cart_arm(cls=_ScriptedStatus, move_timeout=30.0, silent=True)
    out = {}

    def run():
        try:
            out["plan"] = arm.move_l(TCP0, speed=0.3)
        except BaseException as e:              # noqa: BLE001 - 结局就是被测的东西
            out["exc"] = e

    th = threading.Thread(target=run, name="cart-waiter", daemon=True)
    th.start()
    time.sleep(0.3)                             # 让它进到位等待 (配对早已完成)
    t0 = time.monotonic()
    arm.close()
    th.join(5.0)
    elapsed = time.monotonic() - t0
    assert not th.is_alive(), "close() 之后等待者还挂着 (会一直挂到 move_timeout=30s)"
    assert elapsed < 5.0, f"等待者花了 {elapsed:.2f}s 才退出 —— 远超应有值"
    assert "exc" in out, f"断连却拿到了 {out.get('plan')!r} —— 该以失败原因退出"
    assert isinstance(out["exc"], NotConnectedError), (
        f"断连时的异常是 {out['exc']!r} —— 必须是 NotConnectedError "
        f"(报成超时会把归因引向'臂没到位', 而真相是链路断了)")


# ---------------------------------------------------------------------------
# 6. ⚠⚠ "停稳" ≠ "停在了目标上" —— `settled` 的**第二条判据** (假成功方向)
# ---------------------------------------------------------------------------

def test_a_motion_aborted_by_something_else_is_not_a_success(cart_arm):
    """⚠⚠ `bit10 1→0` + `q` 静止 **不足以**证明"这条笛卡尔轨迹跑完了"。

    固件 `ctrl_accept_move_j` 的**第一条语句**就是 `cart_invalidate_before_motion()`
    ⇒ 线程 A 的 `move_l(goal, wait=True)` 中途被**别的**运动作废时: 下一帧 `bit10` 就落 0,
    臂转去跑**那条**命令, 它跑完也静止。只看 `bit10` 的实现会在这一刻报 `settled=True`,
    **而 TCP 根本不在目标上**。⚠ 锁只挡住笛卡尔入口, 挡不住 `movej`/`home`/零重力,
    也挡不住**别的进程** (本机 CDC 不独占) —— SDK 侧拦不住。

    桩把"臂停在别处"做成 `pos_override`: 命令的目标仍是 `TCP0`, 而回读的 TCP 是
    0.10/0.50/0.90 (帧序、`q`、`bit10` 全都与"正常跑完"一模一样 —— 这正是这条用例的要点)。
    判据: `ok=True` (固件确实受理并规划了) 但 **`settled=False`**; 收尾三字段照旧填真值
    (`settle_err_rad` 的语义**不变**)。
    """
    arm = cart_arm(pos_override=(0.10, 0.50, 0.90))
    plan = arm.move_l(TCP0, speed=0.3)
    assert plan.ok is True, "固件受理了这条规划 —— `ok` 说的是'受理并规划出来了'"
    assert plan.started_busy is True, "这套帧序里 bit10 确实 1→0 过 (用例前提)"
    assert plan.settled is False, (
        "停稳了但 TCP 不在目标上 —— 报 settled=True 就是**假成功**: 调用方会以为轨迹"
        "跑完了, 而臂停在别处")
    assert plan.settle_err_rad == 0.0, "抖动幅度照旧 (`settle_err_rad` 的语义没变)"


def test_no_tcp_readback_means_not_settled(cart_arm):
    """⚠ `get_tcp()` **取不到**时也必须报"没到位" (保守, 方向安全)。

    连不上/超时/被别人的 `ERR` 串台时, "到底停在哪"是**未知**的 —— 未知不能算成功
    (与"宁可报未知, 绝不报成功"同向)。这里让 `0x43` 回一条 `ERR` 来造这一支。
    """
    arm = cart_arm()
    arm._tr.err_override[P.CMD_GET_TCP] = 0x03
    plan = arm.move_l(TCP0, speed=0.3)
    assert plan.ok is True and plan.started_busy is True
    assert plan.settled is False, "回读 TCP 失败却报 settled=True —— 未知被当成了成功"


# ---------------------------------------------------------------------------
# 7. 并发读者**不许**把到位等待饿死 (假失败方向)
# ---------------------------------------------------------------------------

class _GreedyReader(FakeTransport):
    """并发读者探针 —— 状态帧**只**交给非等待者线程。

    `waiter_name` 指定的线程读状态帧时一律拿到 `None` (不管帧躺在队列里还是来自空闲流),
    模型就是"另一个读者把状态帧全抢走了": `get_state()`/`get_tcp()` **不持**
    `Arm._cart_serial`, 而本包没有读线程 —— "等应答"与"读应答"是同一个取帧口, 谁先读谁拿走。
    受害面是**到位等待**: 只认"自己读到的帧"的实现会被饿到 `move_timeout` (假失败,
    而臂明明已经停稳)。

    ⚠ 只拦**状态帧**(以及空闲流): 等待者自己那条命令的 `ACK`/`0x4E` 与它 `get_tcp` 要的
    `RSP_TCP` 照常交付 —— 那几条是**它在等**的应答, 不是被抢的东西。
    ⚠ `greedy_name` 那个线程 (用例里的主线程) **也不吃非状态帧**: 队首是别人的应答时它改
    拿一帧空闲状态。没有这一条, `get_tcp` 的应答会被它顺手丢掉 (`_read_status` 对非状态帧
    就是丢), 用例就变成测"`get_tcp` 被抢" —— 那是**另一条路径** (spec 规定保守回
    `settled=False`, 已由上面那条用例覆盖), 而不是测"到位等待被饿死"。
    """

    def __init__(self, *a, **kw) -> None:
        super().__init__(*a, **kw)
        self.waiter_name: Optional[str] = None
        self.greedy_name: Optional[str] = None
        self.greedy_reads = 0

    def _idle_status(self):
        """一帧空闲状态 —— **必须**走 `_stamp_status`: `seq` 与 `CART_BUSY` 的盖章口只有
        一个, 绕开它就等于把被测的取帧语义一起换掉了。"""
        return self._stamp_status((P.RSP_STATUS, P.unpack_frame(
            _status(self.q, mode=1, n=self.n))[1]))

    def read_frame(self, timeout=None):
        cur = threading.current_thread().name
        head = P.unpack_frame(self._resp[0])[0] if self._resp else None
        if cur == self.greedy_name:
            self.greedy_reads += 1
            if head is not None and head != P.RSP_STATUS:
                return self._idle_status()      # 别人的应答: 不吃, 改交一帧空闲状态
            return super().read_frame(timeout)
        if cur == self.waiter_name and (head is None or head == P.RSP_STATUS):
            return None                         # 抢走了: 等待者一帧状态都拿不到
        return super().read_frame(timeout)


def test_a_concurrent_status_poller_does_not_disturb_the_arrival_wait(cart_arm):
    """**并发状态读者不再有受害面** —— 判据是"到位等待照常成功"。

    ⚠ 本用例从前叫 `test_a_greedy_concurrent_reader_cannot_starve_the_arrival_wait`，
    靠 `_GreedyReader`（一个按**调用线程名**决定交不交帧的桩）模拟"另一个读者把状态帧
    全抢走"。读线程一开，那个模型**整体作废**：
      · 读 `transport` 的只有读线程一条，桩根本看不到第二个线程来读帧；
      · 状态帧走**单槽 + `status_seq`**（不是队列），所有消费者看的是**同一份事实**，
        "谁读到了"不再影响"谁看到了"。
    故本用例改成测**它现在真正该保证的事**：一边死命轮询 `get_state(refresh=True)`，
    一边 `move_l` 到位 —— 两者都必须正常收场。
    """
    arm = cart_arm()
    tr = arm._tr
    out = {}
    polls = {"n": 0}

    def run():
        try:
            out["plan"] = arm.move_l(TCP0, speed=0.3)
        except BaseException as e:              # noqa: BLE001 - 结局就是被测的东西
            out["exc"] = e

    th = threading.Thread(target=run, name="cart-waiter", daemon=True)
    th.start()
    while th.is_alive():
        arm.get_state(refresh=True)             # 并发读者：死命取状态
        polls["n"] += 1
    th.join(5.0)

    assert polls["n"] > 0, "并发读者一次都没跑 (用例自身失效)"
    assert "exc" not in out, f"到位等待被并发读者搅了: {out['exc']!r}"
    assert out["plan"].settled is True, "并发读者与等待者看到的必须是同一份事实"


# ---------------------------------------------------------------------------
# 11. `_wait_settled` 里两条**从来没被走到**的分支 (变异存活 = 无覆盖)
# ---------------------------------------------------------------------------

def test_the_first_frame_only_establishes_the_watermark(cart_arm):
    """⚠ `_wait_settled` 的"还没有水线"那一支: `seq0 is None` 时**第一帧只当起算点**。

    `seq0` 由 `cart._seq_now(arm)` 取 (`arm._a.state.seq`, 见 `_seq_now` 的 docstring);
    **还没见过任何状态帧**时它是 `None` —— 握手那一帧丢了就是这个形状。

    ⚠ **本条之前没有任何用例走到这里**: `_request_and_wait` 总是拿得到水线 (连 `connect()`
    都见过状态帧)。所以把这一支整段拿掉 (`if seq0 is None:` → `if False:`) 时**加本条之前**
    的全量套件照绿 —— 因为它只在**真进过这一支**之后才会炸 (下一循环就会拿 `None` 去算
    `(st.seq - seq0) & 0xFFFF`)。
    """
    arm = cart_arm()
    arm.move_timeout = 2.0
    arm._a.state = None            # "还没见过任何状态帧" (见 `cart._seq_now`)

    plan = arm.move_l(TCP0, speed=0.3)         # 不应抛

    assert plan.ok, "用例自身失效: 规划没成功"
    assert plan.started_busy is True, "`CART_BUSY` 那一支没被走到 —— 用例自身失效"


def test_the_same_frame_is_not_counted_twice(cart_arm):
    """⚠ **假停稳方向**: 同一帧在两次采样里被数成两帧 ⇒ 静止窗少判一帧。

    `_wait_settled` 的取帧是"自己读不到就采 `arm._require().state` 留下的成果"
    (判据认的是**帧**, 不是"谁读的帧" —— 见 `_GreedyReader` 那条)。并发读者把帧读走之后,
    `arm.state` 在两次采样里可以是**同一帧**; 不去重就等于每拍都往里塞一个采样点,
    `arrive_frames` 拍"静止"可以全由**一帧**凑出来。

    ⚠ 既有用例同样一次都没走到: 桩的每帧都换新的 `seq`。
    形状: 取帧口 (`_read_status`) 恒回 `None` (帧被并发读者抢走), 而 `arm.state` **停在**
    一帧上 ⇒ 去重生效时窗口永远只有 1 帧 ⇒ 必须**超时**; 不去重则 3 拍就凑满窗 ⇒ 假停稳。
    """
    # ⚠ 冻结的**方式**换了：`_Ack.state` 现在归**读线程**所有（它一直在填），
    # 所以"把 state 设成一帧然后不改"做不到 —— 必须让**桩不再产帧**（`silent`）。
    # ⚠ 也删掉了 `monkeypatch.setattr(arm, "_read_status", ...)`：那个方法已经不存在，
    # 而它当年模拟的"帧被并发读者抢走"在本设计下**不可能发生**。
    arm = cart_arm(cls=_ScriptedStatus, silent=True)
    arm.move_timeout = 0.3
    seq0 = 100
    body = P.unpack_frame(_status([0.0] * arm.n, dq=[0.0] * arm.n,
                                  mode=1, seq=seq0 + 1))[1]
    arm._a.state = ST.decode_state(body)       # 水线之后、静止的一帧 (恒为同一帧)

    with pytest.raises(MotionTimeoutError):
        _wait_settled(arm, "去重用例", seq0)
