"""笛卡尔固件原生路径 (`0x3A`/`0x3B`/`0x3C-0x3E` + `RSP_CART_PLAN 0x4E`) 的离线判据。

**为什么单独一个文件**: `test_protocol_sync.py` 管"常量/登记/入口可达"的**静态**契约,
`test_full_coverage.py` 管"每条命令真的被发出去过", `test_cart_queue.py` 管 FIFO 配对。
本文件管**结果语义与守卫顺序** —— 也就是"发出去之后, 这条命令的结局怎么落到调用方身上":

* **守卫顺序** (⚠ 承重): `_CartPending.request()` 只在 `TransportError` 上摘 token,
  其余异常一律 re-raise 且**不动队列** —— 所以"写之前就会抛、却不是 `TransportError`"
  的那两类异常 (`NotConnectedError` / 零重力守卫的 `InvalidCommandError`) **必须**在
  登记 token **之前**跑完, 否则 token 会留在队里, 下一条合法应答就会被配错。
* **能力探测**: 固件受 `#if LITEARM_CART_PLAN` 约束, 且 `0x3A~0x3E` 里**没有只读命令**
  —— 探测只能是"发空载荷撞长度校验", 绝不能发合法载荷 (那等于"connect 之后臂自己动一下")。
* **`err` 映射**: `0x4E` 的 `err` 字节是 `cart_err_t`; 被取代 (CANCELED) 是**正常结果**
  而不是失败, 必须能与"规划失败"分开。
* **`CartPlan` 的四个"未等待"字段**: `wait=False` 时入口只等到**规划结果**, 不到位;
  `wait=True` (默认) 的到位判据在 `tests/test_cart_wait.py`。
"""
from __future__ import annotations

import math
import struct
import sys
import threading
import time
import types

import pytest

from fake_serial import FakeTransport, _status   # tests/ 在 conftest 里进了 sys.path
from litearm import Arm, _protocol as P
from litearm.arm import _orient_angle
from litearm.cart import (CART_START_POS_TOL, CART_START_RPY_TOL, CartPlan,
                               raise_for_plan)
from litearm.errors import (CartesianPlanError, CommandRejectedError,
                                 IKError, InvalidCommandError,
                                 MotionSupersededError, NotConnectedError,
                                 TransportError, UnsupportedByFirmwareError)

#: 桩的初始 TCP (`fake_serial` 的 `self.pose`), `move_c` 的 pose_start 必须与它一致。
TCP0 = (0.30, 0.0, 0.35, 0.0, 0.0, 0.0)


@pytest.fixture
def make_arm(monkeypatch):
    """可配置的桩工厂 —— 在 `connect()` **之前**装配桩 (探测就发生在 connect 里)。"""
    import litearm.arm as arm_mod
    from fake_serial import FakeTransport

    def _make(cfg=None, cls=FakeTransport):
        def factory(port="fake", timeout=0.2, **_ignored):
            t = cls(port=port, timeout=timeout)
            if cfg is not None:
                cfg(t)
            return t
        monkeypatch.setattr(arm_mod, "SerialTransport", factory)
        from litearm import Arm
        return Arm(port="fake")
    return _make


class _SilentProbe:
    """混入: 让桩对 `0x3A` **一片安静** (其余照常) —— 模拟探测应答丢失。"""

    def write_frame(self, cmd, payload=b""):
        if cmd == P.CMD_MOVE_L:
            with self._tx_lock:
                self.tx_log.append((cmd, bytes(payload)))
                self.tx_stamps.append(time.monotonic())
            return
        super().write_frame(cmd, payload)


class _ShortTcpPayload(FakeTransport):
    """`0x43` (GET_TCP) 回一条**短**载荷 (< 24B)。

    `Arm.get_tcp` 对它的契约是**返回 `None`** (`if len(p) >= 24 … return None`) —— 不是抛。
    于是两条守卫同时拿到"取不到 TCP": `move_c` 的起点校验 (发帧**之前**) 与
    `_tcp_reached` (收尾回读)。两条各自都有保守处置, 都要有判据。
    """

    def write_frame(self, cmd, payload=b""):
        if cmd == P.CMD_GET_TCP:
            with self._tx_lock:
                self.tx_log.append((cmd, bytes(payload)))
                self.tx_stamps.append(time.monotonic())
            self._push(P.pack_frame(P.RSP_TCP, b"\x00" * 8))
            return
        super().write_frame(cmd, payload)


def _silent_probe_transport():
    from fake_serial import FakeTransport

    class _T(_SilentProbe, FakeTransport):
        pass
    return _T


def _sent(arm):
    return [c for c, _ in arm._tr.tx_log]


def _payloads(arm, cmd):
    return [p for c, p in arm._tr.tx_log if c == cmd]


def _is_f32(got, want) -> bool:
    """按 f32 精度比较 (载荷是 `struct.pack("<f")` 出来的, 不能拿 f64 全等去比)。"""
    return len(got) == len(want) and all(abs(float(a) - float(b)) < 1e-6
                                         for a, b in zip(got, want))


# ---------------------------------------------------------------------------
# 1. 能力探测 —— 只发空载荷的 0x3A
# ---------------------------------------------------------------------------

def _await_collector(arm, timeout: float = 1.0) -> None:
    """等读线程把已推入桩的帧投递出去 —— 旧"`poll_cart()` 自己读一帧"的等价物。

    ⚠ `poll_cart()` 现在**不碰链路**（读线程是唯一读者），它只是"看一眼收集器的待认领
    队列"。所以用例必须先让帧**被投递**，再调它。
    """
    import time as _t
    end = _t.monotonic() + timeout
    while _t.monotonic() < end and arm._tr._resp:
        _t.sleep(0.002)
    _t.sleep(0.01)


def test_probe_sends_an_empty_move_l_payload(offline_arm):
    """探测帧**必须是空载荷**: 合法载荷的 `0x3A` 会让臂真的动 (那是运动命令, 不是查询)。"""
    arm = offline_arm
    assert arm._cart_supported is True, "普通桩 (有笛卡尔) 的探测结果应为 True"
    assert _payloads(arm, P.CMD_MOVE_L) == [b""], (
        "探测发的 0x3A 不是空载荷 —— 合法载荷就是 connect 之后臂自己动一下")
    assert arm._cart.pending == 0, "探测帧登记了 token (会造出一个永远等不到 0x4E 的悬挂态)"


def test_probe_goes_through_the_query_port_not_the_action_port(offline_arm):
    """探测走 `_write_query` (查询口), **不是** `_write_cmd` (动作口) —— 见 `probe` 的实现。

    ⚠ 两个口在"没在拖动示教"时行为逐字节相同, 所以只有**零重力保活期**这一种形状能把它们
    分开: `_write_cmd` 带零重力守卫, 会把这条**空载荷的探测**当成动作命令拒掉
    (`InvalidCommandError`); 而 `_write_query` 那条路的 docstring 明写"拖动示教期间仍应能
    读状态"。
    ⚠ 探测确实是**查询语义**: 载荷是空的, 固件在**长度校验**那一关就回
    `ERR{cmd,0x01}` —— 排在一切副作用之前, 臂不会动 (见 `probe` 的 docstring 与
    `test_probe_sends_an_empty_move_l_payload`)。故它不该被动作口那道守卫拦。
    """
    from litearm import cart as cart_mod

    arm = offline_arm
    arm._zg_active = True                    # 拖动示教保活期
    try:
        assert cart_mod.probe(arm) is True, "零重力保活期探测被拒了 (走了动作口)"
    finally:
        arm._zg_active = False
    assert (P.CMD_MOVE_L, b"") in arm._tr.tx_log, "探测帧根本没发出去"


def test_probe_is_done_once_in_connect_not_lazily(offline_arm):
    """探测在 `connect()` 里做一次; 之后每次入口调用不再重探。"""
    arm = offline_arm
    n0 = _sent(arm).count(P.CMD_MOVE_L)
    assert n0 == 1, f"connect() 应恰好探测一次, 实测 {n0} 次"
    arm.move_l(TCP0, speed=0.3)
    arm.move_l(TCP0, speed=0.3)
    assert _sent(arm).count(P.CMD_MOVE_L) == 3, "入口调用又探了一次"


def test_no_cart_firmware_is_detected_offline(offline_arm):
    """桩关掉 `cart_supported` = 固件没编进 `LITEARM_CART_PLAN` -> 落 default -> 不支持。"""
    from litearm import cart as cart_mod

    arm = offline_arm
    arm._tr.cart_supported = False
    assert cart_mod.probe(arm) is False
    assert arm._cart.pending == 0, "探测帧进了队列"


def test_unsupported_firmware_raises_and_sends_nothing(make_arm):
    """不支持时三个入口**直接抛 `UnsupportedByFirmwareError` 且一帧都不发**。"""
    arm = make_arm(lambda t: setattr(t, "cart_supported", False)).connect()
    assert arm._cart_supported is False
    for fn in (lambda: arm.move_l(TCP0, speed=0.3),
               lambda: arm.move_c(TCP0, TCP0, TCP0, speed=0.3),
               lambda: arm.move_path([TCP0], speed=0.3)):
        before = list(arm._tr.tx_log)
        with pytest.raises(UnsupportedByFirmwareError):
            fn()
        assert arm._tr.tx_log == before, "不支持时仍然发了帧"
        assert arm._cart.pending == 0


def test_silent_probe_is_treated_as_unsupported_not_as_a_crash(make_arm):
    """探测帧没被回答时**不抛** —— 判成"不支持"。

    依据: 固件的 `default` 分支保证任何固件都会回一条 ERR, 所以"一片安静"只可能是应答
    丢了; 而在固件声明自己实现之前绝不发一条能起规划的命令, 与"这一帧为什么没回来"
    无关。判成"不支持"的代价是归因含糊 (报错里两种可能都写了); 判成"崩溃"的代价是
    整个会话连关节空间都用不了 —— 后者更糟。
    """
    arm = make_arm(cls=_silent_probe_transport()).connect()
    assert arm._cart_supported is False
    with pytest.raises(UnsupportedByFirmwareError):
        arm.move_l(TCP0, speed=0.3)


def _flaky_probe_transport(drop_first: int = 1):
    """桩混入: 把 `0x3A` 的**前 `drop_first` 帧应答丢掉** (其余照常) —— 模拟上行丢帧。

    ⚠ 丢的必须是**应答**而不是下行帧: 下行帧照记 `tx_log` —— "重试有没有真的重发"
    正是靠它判的。
    """
    from fake_serial import FakeTransport

    class _T(FakeTransport):
        _dropped = 0

        def write_frame(self, cmd, payload=b""):
            if cmd == P.CMD_MOVE_L and _T._dropped < drop_first:
                _T._dropped += 1
                with self._tx_lock:
                    self.tx_log.append((cmd, bytes(payload)))
                    self.tx_stamps.append(time.monotonic())
                return
            super().write_frame(cmd, payload)

    return _T


def test_a_lost_probe_reply_is_retried_not_read_as_unsupported(make_arm):
    """⚠ 本组是"安静 ≠ 不支持"的判据: 丢一帧应答**不得**把整个会话判成不支持。

    固件的 `default` 分支保证**任何**固件都会对 `0x3A` 回一条 `ERR{cmd,0x00}`, 所以读超时
    只可能是**这一帧丢了** (本硬件 USB CDC 上行有已知丢帧); 而探测结果在 `connect()`
    里缓存**整个会话**, 一次丢帧就把笛卡尔能力误判成"固件没编进去"。
    """
    arm = make_arm(cls=_flaky_probe_transport(drop_first=1)).connect()
    assert arm._cart_supported is True, "丢了一帧应答就把整个会话的笛卡尔判成不支持"
    assert _payloads(arm, P.CMD_MOVE_L) == [b"", b""], "重试时没有再发一帧探测"


def test_all_silent_probes_leave_the_reason_as_unconfirmed(make_arm):
    """3 次都安静 ⇒ 判"不支持"(fail-closed), 且**报错要能与"固件确报不支持"区分**。

    没有这条区分, 现场只能看到一个笼统的"固件不支持笛卡尔", 无法判断该换固件还是
    该查链路 —— 两种可能的原因完全不同。
    """
    arm = make_arm(cls=_silent_probe_transport()).connect()
    assert arm._cart_supported is False
    assert _payloads(arm, P.CMD_MOVE_L) == [b""] * 3, "安静时的重试次数不是 3"
    with pytest.raises(UnsupportedByFirmwareError) as ei:
        arm.move_l(TCP0, speed=0.3)
    assert "探测未获确认" in str(ei.value), "报错没有写明是「没等到应答」这一支"


def test_a_probe_answered_with_err_00_is_not_retried(make_arm):
    """`ERR{0x3A,0x00}` 是**确定结论** (固件确实没有这条命令) ⇒ 立刻返回, **不重试**。"""
    arm = make_arm(lambda t: setattr(t, "cart_supported", False)).connect()
    assert arm._cart_supported is False
    assert _payloads(arm, P.CMD_MOVE_L) == [b""], "固件已确报不支持却仍在重试"
    with pytest.raises(UnsupportedByFirmwareError) as ei:
        arm.move_l(TCP0, speed=0.3)
    msg = str(ei.value)
    assert "固件确报不支持" in msg, "报错没有写明是「固件确报不支持」这一支"
    assert "探测未获确认" not in msg, "把已确认的结论报成了「未获确认」"


# ---------------------------------------------------------------------------
# 2. 守卫顺序 —— 全部排在 request() (登记 token) 之前
# ---------------------------------------------------------------------------

def test_zero_g_guard_fires_before_the_token_is_registered(offline_arm):
    """⚠ 本组是关键判据: `_zg_active=True` 时抛 `InvalidCommandError` **且 `pending == 0`**。

    `pending == 0` 的意思是"**根本没登记**", 不是"登记之后又摘掉" —— 若守卫被放进
    `request()` 里面, 异常会在写的位置抛出, 而 `_CartPending.request` 只对
    `TransportError` 摘 token, 于是这条 token 留在队里, 下一次 `0x4E` 就配错对象。
    """
    arm = offline_arm
    arm._zg_active = True                          # 只置标志, 不起保活线程
    for label, fn in (("move_l", lambda: arm.move_l(TCP0, speed=0.3)),
                      ("move_c", lambda: arm.move_c(TCP0, TCP0, TCP0, speed=0.3)),
                      ("move_path", lambda: arm.move_path([TCP0], speed=0.3))):
        before = list(arm._tr.tx_log)
        with pytest.raises(InvalidCommandError):
            fn()
        assert arm._cart.pending == 0, f"{label}: 守卫在登记之后才拦 —— token 滞留了"
        assert arm._tr.tx_log == before, f"{label}: 零重力保活期间仍然发了帧"


def test_not_connected_raises_before_the_token_is_registered():
    """未连接 (`NotConnectedError`) 同样必须早于登记 —— 它是另一个"写之前就抛"的异常。"""
    from litearm import Arm

    arm = Arm()
    with pytest.raises(NotConnectedError):
        arm.move_l(TCP0, speed=0.3)
    assert arm._cart.pending == 0


def test_bad_arguments_are_validated_before_the_frame_is_written(offline_arm):
    arm = offline_arm
    for fn in (lambda: arm.move_l(TCP0, speed=1.5),
               lambda: arm.move_l((0.3, 0.0), speed=0.3),
               lambda: arm.move_path([], speed=0.3),
               lambda: arm.move_path([TCP0] * 33, speed=0.3)):
        before = list(arm._tr.tx_log)
        with pytest.raises(InvalidCommandError):
            fn()
        assert arm._tr.tx_log == before, "参数校验失败却已经发了帧"
        assert arm._cart.pending == 0


# ---------------------------------------------------------------------------
# 3. 帧布局 (逐字节, 照固件 `usb_cmd.h`)
# ---------------------------------------------------------------------------

def test_move_l_payload_is_pose6_plus_speed(offline_arm):
    arm = offline_arm
    plan = arm.move_l((0.31, 0.0, 0.36, 0.0, 0.0, 0.0), speed=0.3)
    ps = _payloads(arm, P.CMD_MOVE_L)[-1]
    assert len(ps) == 28, "0x3A 载荷应 28B (pose[6]f32 + sp f32)"
    assert _is_f32(P.unpack_f32s(ps, 0, 6), [0.31, 0.0, 0.36, 0.0, 0.0, 0.0])
    assert P.unpack_f32s(ps, 24, 1)[0] == pytest.approx(0.3, abs=1e-6)
    assert plan.ok and plan.n_wp == arm._tr.cart_n_wp
    assert plan.plan_us == arm._tr.cart_plan_us


def test_move_l_invalid_speed_rejected(offline_arm):
    """`speed` 越界必须在**发帧之前**被拒 —— `0x3A` 一出门, 臂就真的动。

    ⚠ **改靶说明**: 本条从已删的 `tests/test_cartesian_motion.py` 摘出保留 (原名
    `test_movel_invalid_speed_rejected`)。原靶 `Arm.movel` 随 PC 侧规划子包删除, 而
    它测的**从来不是**被删的规划器 —— 是 `speed` 的取值校验本身。新载体是
    `cart._as_speed` (三个固件侧入口 `move_l`/`move_c`/`move_path` 共用同一把尺子),
    这里以 `move_l` 为代表钉住"越界抛 `InvalidCommandError`"。

    ⚠ **判据集不能照搬原用例**: 原用例喂的是 `(0.0, -1.0, 1.5)`, 而 `0.0` 在
    **旧 PC 规划器**那里才是非法的 (它要 `0 < speed` 保证剖面可解)。新载体与新域的
    是 `[0, 1]` **闭区间** —— 与同文件的 `Arm.movej` / `Arm.movej_sync` 逐字一致
    (`arm.py` 两处都写 `0.0 <= float(speed) <= 1.0`)。`0.0` 在这里**合法且安全**:
    固件 `cart_plan.c` 把乘过 speed 的限幅做了下限 clamp (`p->v_lim = fmaxf(v_lim *
    speed, 1e-4f)`), 且那份源码在 clamp 处自带注释"若将来有人想删, 先补一条 speed→0
    的用例" —— 即"speed 极小"是固件显式覆盖过的输入, 不是漏网。照搬 `0.0` 会让本条
    变成常在的假红。
    """
    arm = offline_arm
    for bad in (-1.0, 1.5, 2.0):
        n_before = len(_sent(arm))
        with pytest.raises(InvalidCommandError, match="speed"):
            arm.move_l((0.31, 0.0, 0.36, 0.0, 0.0, 0.0), speed=bad)
        assert _sent(arm)[n_before:] == [], f"speed={bad} 被拒前已经发帧"


def test_move_c_payload_is_via_end_speed(offline_arm):
    arm = offline_arm
    via = (0.30, 0.0, 0.40, 0.0, 0.0, 0.0)
    goal = (0.32, 0.0, 0.40, 0.0, 0.0, 0.0)
    arm.move_c(TCP0, via, goal, speed=0.3)
    ps = _payloads(arm, P.CMD_MOVE_C)[-1]
    assert len(ps) == 52, "0x3B 载荷应 52B (via[6] + end[6] + sp)"
    assert _is_f32(P.unpack_f32s(ps, 0, 6), via)
    assert _is_f32(P.unpack_f32s(ps, 24, 6), goal)
    assert P.unpack_f32s(ps, 48, 1)[0] == pytest.approx(0.3, abs=1e-6)


def test_move_path_is_begin_then_add_then_run(offline_arm):
    """`0x3C`(n+sp) -> `0x3D`×n (idx+pose) -> `0x3E`(空); **token 只登记在 RUN 上**。"""
    arm = offline_arm
    pts = [TCP0, (0.30, 0.0, 0.40, 0.0, 0.0, 0.0), (0.32, 0.0, 0.40, 0.0, 0.0, 0.0)]
    plan = arm.move_path(pts, speed=0.3)
    sent = _sent(arm)
    # ⚠ 切到 **RUN 为止**: `wait=True` 的收尾还会回读一次 `get_tcp` (0x43) 与目标比对,
    # 那一条排在 RUN **之后** —— 用 `sent[-5:]` 会把 0x43 当成第五帧。
    end = sent.index(P.CMD_CART_RUN) + 1
    ids = sent[end - 5:end]
    assert ids == [P.CMD_CART_BEGIN, P.CMD_CART_ADD, P.CMD_CART_ADD, P.CMD_CART_ADD,
                   P.CMD_CART_RUN], ids
    begin = _payloads(arm, P.CMD_CART_BEGIN)[-1]
    assert len(begin) == 5 and begin[0] == 3, "0x3C 载荷应 5B (n u8 + sp f32)"
    adds = _payloads(arm, P.CMD_CART_ADD)
    assert len(adds) == 3
    for i, (a, p) in enumerate(zip(adds, pts)):
        assert len(a) == 25, "0x3D 载荷应 25B (idx u8 + pose[6])"
        assert a[0] == i and _is_f32(P.unpack_f32s(a, 1, 6), p)
    assert _payloads(arm, P.CMD_CART_RUN)[-1] == b"", "0x3E 是空载荷"
    assert plan.ok
    assert arm._cart.pending == 0, "RUN 的 token 没有被 wait 取走"


class _WatermarkSpy(FakeTransport):
    """记下**每次下行之前**的 `status_seq` 与 `tx_log` 长度 —— 给"水位线取在哪"那条用例。

    ⚠ `BEGIN` 的应答前**先交一帧状态**: 真固件的 100Hz 状态流不会因为主机在发 BEGIN 就
    停下来。没有这一帧, "水位线取在 BEGIN 之前"与"取在 BEGIN 之后"在桩上**看不出差别**
    (`status_seq` 只数**交付给 SDK 的**帧, 而 BEGIN 的 ACK 交换本身不交付状态帧),
    那条用例就白写了。
    """

    def __init__(self, *a, **kw) -> None:
        super().__init__(*a, **kw)
        #: (命令, 该次下行**之前**的 status_seq, 该次下行**之前**的 tx_log 长度)
        self.downlink_marks: list = []

    def write_frame(self, cmd, payload=b""):
        self.downlink_marks.append((cmd, self.status_seq, len(self.tx_log)))
        if cmd == P.CMD_CART_BEGIN:
            self._push(_status(self.q, mode=1, n=self.n))
        return super().write_frame(cmd, payload)


def test_move_path_takes_the_watermark_before_begin(monkeypatch):
    """⚠ 水位线 (`seq0`) 必须取在 **BEGIN 之前** —— "命令"是 BEGIN+ADD×n+RUN 这**一整段**。

    取在段中间 (BEGIN 之后) 就把这一段最早的那几帧挪出了"命令之后"的窗口。
    这条事实此前**没有测试**: 把 `seq0 = _seq_now(arm)` 挪进 `with` 里 (BEGIN 之后),
    全量 398 条照旧全绿 —— 而注释是把"取在 BEGIN 之前"当**承重**写的。

    判据 (两条, 同时断言, 各守一头):
    ① **位置**: `_seq_now()` 被调用时 `tx_log` 里**还没有**这一段的第一条帧 (BEGIN ——
       在本桩上它是会话的**第三条**帧: 前两条是 `connect()` 的版本握手与空载荷能力探测)。
       这条就是"取在 BEGIN 之后"的判别式 (那段代码只要挪进 `with` 就必然落在 BEGIN 的帧写
       之后 ⇒ ① 红);
    ② **取值来源**: 它拿到的必须是 BEGIN **下行之前那一刻**的 `status_seq` (桩在 BEGIN
       应答前交了一帧状态, 所以"取在 BEGIN 之后"会拿到一个**更大**的数)。① 过了而 ② 红
       只有一种走法 —— `_seq_now` 被换成读某处**陈旧**水位线 (那时位置对、值不对),
       所以两条都要在。
    """
    import litearm.arm as arm_mod
    import litearm.cart as cart_mod

    def factory(port="fake", timeout=0.2, **_ignored):
        return _WatermarkSpy(port=port, timeout=timeout)

    monkeypatch.setattr(arm_mod, "SerialTransport", factory)
    arm = Arm(port="fake").connect()
    tr = arm._tr

    seen = []
    real_seq_now = cart_mod._seq_now

    def spy(a):
        v = real_seq_now(a)
        seen.append((v, len(a._tr.tx_log)))
        return v

    monkeypatch.setattr(cart_mod, "_seq_now", spy)
    arm.move_path([TCP0, (0.30, 0.0, 0.40, 0.0, 0.0, 0.0)], speed=0.3)

    assert len(seen) == 1, f"水位线取了 {len(seen)} 次 —— 用例自身失效"
    got, n_tx = seen[0]
    marks = [m for m in tr.downlink_marks
             if m[0] in (P.CMD_CART_BEGIN, P.CMD_CART_ADD, P.CMD_CART_RUN)]
    assert marks and marks[0][0] == P.CMD_CART_BEGIN, "这一段的第一条帧不是 BEGIN"
    _, begin_seq, begin_tx = marks[0]
    assert n_tx <= begin_tx, (
        f"水位线是在这一段的帧**写出去之后**取的: 取的时候 `tx_log` 已经有 {n_tx} 条帧, "
        f"而 BEGIN (这一段的第一条) 在下标 {begin_tx} —— 段内最早的几帧被挪出了窗口")
    assert got == begin_seq, (
        f"水位线不是 BEGIN 下行**之前**那一刻的值 (取到 {got}, 应为 {begin_seq})")


def test_move_path_failure_midway_is_raised_and_not_papered_over(offline_arm):
    """BEGIN/ADD 中途失败 -> 抛错, 且**不发任何**"清状态"的补偿命令 (固件 RECV 2s 自愈)。"""
    arm = offline_arm
    arm._tr.err_override[P.CMD_CART_ADD] = 0x04
    with pytest.raises(CommandRejectedError):
        arm.move_path([TCP0, TCP0], speed=0.3)
    assert P.CMD_CART_RUN not in _sent(arm), "ADD 失败后仍然发了 RUN"
    assert arm._cart.pending == 0


# ---------------------------------------------------------------------------
# 4. move_c 的起点校验 —— 复用 `_pose_near` 的**两条**判据
# ---------------------------------------------------------------------------

def test_move_c_rejects_a_start_far_from_the_actual_tcp(offline_arm):
    arm = offline_arm
    before = list(arm._tr.tx_log)
    with pytest.raises(InvalidCommandError) as ei:
        arm.move_c((0.50, 0.0, 0.35, 0.0, 0.0, 0.0), TCP0, TCP0, speed=0.3)
    assert "0.5" in str(ei.value), "报错要给出**实际起点**"
    # ⚠ 只断言"没发 0x3B": 校验要用 `get_tcp` (0x43) 读实测起点, 那一条**该**发。
    assert P.CMD_MOVE_C not in _sent(arm), "校验失败却已经发了帧 (0x3B 受理即起跑)"
    assert len(arm._tr.tx_log) > len(before)
    assert arm._cart.pending == 0


#: 固件/本包两侧**锁带宽度不同**的那条带里的一点 (`_rot.mat_to_rpy` 的 docstring 实测):
#: 本包 `|pitch|` 距 `π/2` 小于 `1e-6` 才走锁分支, 固件是 `|cos(pitch)| <= 1e-4`
#: (`kin.c:362`/`:376`)。取 `5e-5` —— 离两侧边界各约 50 倍, 两条分支各自稳稳成立。
_LOCK_BAND_PITCH = math.pi / 2 - 5e-5


def test_move_c_start_check_reuses_the_rotation_equivalence_branch(offline_arm):
    """⚠ 万向锁附近: **同一个旋转**可以给出差很远的 rpy 分量 ⇒ 只抄逐分量判据会判失败。

    ⚠⚠ **这条判据在 `move_c` 上可达的形状只有一种**, 别按"给一个非规范形的 start"去写
    —— 那种写法**碰不到**这条分支 (实测: 用它时, 把被测实现换成忠实的逐分量版,
    全量套件照**绿**; 也就是"声称守了、实际抓不到"):

    `move_c` 的 `start` 一定过 `cart._as_pose6` ⇒ 一定被 `_rot.mat_to_rpy(rpy_to_mat(…))`
    **归一化** (给出 6 标量或 `(pos, R)` 都一样, 那条路径结尾恒是 `mat_to_rpy`)。所以
    非规范形**只能出在固件回读那一侧** (`get_tcp` 的 `rpy` 走固件的 `kin_rot_to_rpy`)。
    两侧的**锁带宽度不同** (见 `_LOCK_BAND_PITCH`), 于是存在一条带: 固件已经走锁分支
    (强制 `yaw=0`) 而本包还没走 —— 带内对**同一个旋转**给出的分量可以差 O(1) rad。

    本用例就取带内一点: `pitch = π/2 − 5e-5`, `roll = yaw = 0.5`。
      * 固件那条 (`rpy_override`) 是它锁分支的输出 —— `(roll−yaw, pitch, 0)` = `(0, pitch, 0)`;
      * 给 `move_c` 的 `start` 是本包归一化后的 `(0.5, pitch, 0.5)`。
    两者只差测地夹角 ~2.5e-5 (远小于 `CART_START_RPY_TOL`), 逐分量却差 **0.5** rad。
    """
    arm = offline_arm
    pitch = _LOCK_BAND_PITCH
    arm._tr.pos_override = [0.30, 0.0, 0.35]
    arm._tr.rpy_override = [0.0, pitch, 0.0]      # 固件锁分支的输出 (yaw 被强制 0)
    start = (0.30, 0.0, 0.35, 0.5, pitch, 0.5)

    # 用例自身的两道前置判据 (不成立就说明它验的是别的东西, 不是"旋转等价"那一支)
    tcp_rpy = [0.0, pitch, 0.0]
    assert max(abs(start[3 + i] - tcp_rpy[i]) for i in range(3)) >= CART_START_RPY_TOL, (
        "用例没有构造出分量差足够大的姿态 —— 逐分量判据本来就会过, 验不到这一支")
    assert _orient_angle(tcp_rpy, start[3:6]) < CART_START_RPY_TOL, (
        "用例的两个姿态其实**不是**同一个旋转 —— 那它验的是反例那一侧")

    arm.move_c(start, (0.30, 0.0, 0.40, 0.0, 0.0, 0.0),
               (0.32, 0.0, 0.40, 0.0, 0.0, 0.0), speed=0.3)     # 不应抛 (起点校验必须过)

    # 反例: 姿态**真的**不同 (同一个 pitch 下摆 yaw, 相差 0.5 rad) -> 必须拒
    with pytest.raises(InvalidCommandError):
        arm.move_c((0.30, 0.0, 0.35, 0.0, pitch, 0.5),
                   (0.30, 0.0, 0.40, 0.0, 0.0, 0.0),
                   (0.32, 0.0, 0.40, 0.0, 0.0, 0.0), speed=0.3)


# ---------------------------------------------------------------------------
# 5. `0x4E` 的 err -> 异常 (⚠ 与 `RSP_ERR` 的门禁原因码是两套码)
# ---------------------------------------------------------------------------

def test_move_c_refuses_to_send_when_the_current_tcp_cannot_be_read(make_arm):
    """⚠ `move_c` 的起点校验是**发帧之前**的最后一道门 —— 取不到 TCP 就**一帧不发**。

    固件圆弧的起点恒为**实测 TCP** (载荷里没有 start)。发出去就等于让臂从"此刻真实位置"
    起跑, 而我们连"此刻在哪"都没读到 —— 无从校验起点。故这一支必须当场抛
    `TransportError`: `0x3B` **受理即起跑**, 没有"发出去再判"的安全顺序。
    """
    arm = make_arm(cls=_ShortTcpPayload).connect()
    before = len(arm._tr.tx_log)
    with pytest.raises(TransportError) as ei:
        arm.move_c(TCP0, (0.30, 0.0, 0.40, 0.0, 0.0, 0.0),
                   (0.32, 0.0, 0.40, 0.0, 0.0, 0.0), speed=0.3)

    assert "TCP" in str(ei.value), "报的错没说是取不到 TCP"
    assert arm._tr.tx_log[before:] == [(P.CMD_GET_TCP, b"")], (
        "取不到 TCP 却把 0x3B 发出去了 (受理即起跑, 起点无从校验)")
    assert arm._cart.pending == 0, "没发帧却登记了 token"


def test_settled_stays_false_when_the_tcp_cannot_be_read(make_arm):
    """⚠ 「停稳」≠「停在了目标上」: 收尾回读 TCP **取不到**时 `settled` 必须**保持 False**。

    `_tcp_reached` 的保守方向是写死的: 取不到就报"没到位" (超时 / 断连 / 被别的 `ERR`
    串台), 与"宁可报未知"同向。放行成"停在目标上了"就在**假成功**那一侧 —— 而这条正是
    "停稳 ≠ 停在目标上"唯一的一道防线 (轨迹被别的运动作废时收尾同样是 `bit10=0` + `q` 静止)。
    """
    arm = make_arm(cls=_ShortTcpPayload).connect()
    arm.move_timeout = 1.0
    plan = arm.move_l(TCP0, speed=0.3)          # wait=True 默认: 等到停稳 + 回读 TCP

    assert plan.ok, "用例自身失效: 规划没成功"
    assert plan.settled is False, (
        "取不到 TCP 却把这条报成'停在目标上' —— 假成功方向, 且这是唯一一道防线")


@pytest.mark.parametrize("err,exc", [
    (1, IKError),
    (2, CartesianPlanError),          # COLLINEAR (⚠ 固件头文件注释写 0x03, 真值是 2)
    (3, CartesianPlanError),          # TOO_LONG
    (4, CartesianPlanError),          # LIMIT
    (5, MotionSupersededError),       # CANCELED = 预期内的接管
    (6, InvalidCommandError),         # BADARG
    (9, CartesianPlanError),          # 未登记的档: 不得静默吞掉
])
def test_plan_err_maps_to_the_right_exception(offline_arm, err, exc):
    arm = offline_arm
    arm._tr.cart_err_override = err
    with pytest.raises(exc) as ei:
        arm.move_l(TCP0, speed=0.3)
    assert f"err={err}" in str(ei.value), "异常消息里要带原始 err"
    if err == 5:
        assert not isinstance(ei.value, CartesianPlanError), (
            "接管 (CANCELED) 被并进了『规划失败』 —— 调用方会把正常抢占走成故障恢复")
    assert arm._cart.pending == 0


def test_plan_failure_is_not_a_command_rejection(offline_arm):
    """接管/规划失败都不是"固件拒绝执行这条命令" (`RSP_ERR`) —— 不得混进那个类型。"""
    arm = offline_arm
    arm._tr.cart_err_override = 5
    with pytest.raises(MotionSupersededError):
        arm.move_l(TCP0, speed=0.3)
    arm._tr.cart_err_override = 3
    with pytest.raises(CartesianPlanError) as ei:
        arm.move_l(TCP0, speed=0.3)
    assert not isinstance(ei.value, CommandRejectedError)


def test_gate_style_rejection_drops_the_token(offline_arm):
    """固件**显式拒绝** (门禁/长度校验回 `RSP_ERR`) 时那条 token 必须摘掉。

    受理前的每一道校验都在 `cart_req_*` 之前 `break`, 所以"回 ERR ⟹ 永远不会有 0x4E"。
    留着它会占住 FIFO 队首, **下一条**合法命令的应答被配给它 —— 下一条报"结局未知",
    死 token 却被"报成功", 正是本模块最防的那一侧。
    """
    arm = offline_arm
    arm._tr.err_override[P.CMD_MOVE_L] = 0x03        # 固件门禁: 未使能
    with pytest.raises(CommandRejectedError):
        arm.move_l(TCP0, speed=0.3)
    assert arm._cart.pending == 0, "被显式拒绝的 token 留在队里了"
    arm._tr.err_override.pop(P.CMD_MOVE_L)
    plan = arm.move_l(TCP0, speed=0.3)               # 配对没有错位
    assert plan.ok


# ---------------------------------------------------------------------------
# 6. `CartPlan` 的字段语义 (本版只等到"规划结果")
# ---------------------------------------------------------------------------

def test_unwaited_fields_are_the_documented_defaults(offline_arm):
    """⚠ `wait=False` 时到位相关的四个字段恒为"未等待"默认值 (`CartPlan` docstring 的表)。

    这条是给后人的护栏: 谁把 `started_busy` 填成真值, 必须连带把到位判据一起接上,
    不能只改一半 (调用方会以为拿到的是到位结论)。判据落在 **`wait=False` 这一支**上
    —— 不是"入口恒不等待" (`wait=True` 的默认语义见 `tests/test_cart_wait.py`)。
    """
    plan = offline_arm.move_l(TCP0, speed=0.3, wait=False)
    assert (plan.started_busy, plan.settled, plan.q_final,
            plan.settle_err_rad) == (False, False, [], 0.0)
    assert isinstance(plan, CartPlan)


def _plan_payload(ok: int, err: int, n_wp: int = 12, plan_us: int = 4700) -> bytes:
    """`0x4E` 的 8B 载荷: `ok u8 + err u8 + n_wp u16 LE + plan_us u32 LE`。"""
    return (bytes([ok, err]) + struct.pack("<H", n_wp) + struct.pack("<I", plan_us))


@pytest.mark.parametrize("ok,err", [(1, 3), (0, 0)])
def test_from_reply_reads_ok_and_err_from_their_own_bytes(ok, err):
    """`ok` 与 `err` 是**两个独立字段** —— 不许把一个写成另一个的函数。

    ⚠ 既有用例一条都钉不住这一点: 桩造出来的数据里恒有 `err == 0 ⟺ ok == 1`
    (规划成功回 `(1,0)`、失败回 `(0,err≠0)`), 于是 `ok = payload[0] != 0` 与
    `ok = payload[1] == 0` 是**等价变异**、全量套件照绿。
    故这里专门造出**两字段不一致**的两条载荷: `ok=1` 但 `err=3`、`ok=0` 但 `err=0`。
    """
    plan = CartPlan.from_reply(_plan_payload(ok=ok, err=err))

    assert plan.ok is (ok != 0), "`ok` 取的不是载荷第 0 字节"
    assert plan.err == err, "`err` 取的不是载荷第 1 字节"


def test_from_reply_rejects_a_short_frame():
    with pytest.raises(TransportError):
        CartPlan.from_reply(b"\x01\x00\x00")


def test_reply_lost_sentinel_is_not_a_firmware_code():
    assert CartPlan.ERR_REPLY_LOST not in range(0, 7), (
        "ERR_REPLY_LOST 不能落在 cart_err_t 的 0..6 里 —— 那是固件的码空间")


def test_raise_for_plan_is_a_noop_when_ok():
    raise_for_plan(CartPlan(ok=True, err=0, n_wp=3, plan_us=100))


# ---------------------------------------------------------------------------
# 7. `poll_cart` 显式认领
# ---------------------------------------------------------------------------

def _plan_frame(ok: int, err: int, n_wp: int = 0, plan_us: int = 0) -> bytes:
    return bytes([ok, err]) + n_wp.to_bytes(2, "little") + plan_us.to_bytes(4, "little")


def test_poll_cart_claims_a_result_that_nobody_waited_for(offline_arm):
    arm = offline_arm
    arm._cart.register()                    # 手工登记一条在途请求
    arm._tr._resp.clear()                   # 让下一次取帧**一定**拿到下面这一条
    arm._tr.push_frame(P.RSP_CART_PLAN, _plan_frame(1, 0, n_wp=arm._tr.cart_n_wp,
                                                    plan_us=arm._tr.cart_plan_us))
    _await_collector(arm)
    plan = arm.poll_cart()
    assert plan is not None and plan.ok and plan.n_wp == arm._tr.cart_n_wp
    assert arm.poll_cart() is None, "同一条结果被认领了两次"
    assert arm._cart.pending == 0


def test_poll_cart_returns_none_when_there_is_nothing_to_claim(offline_arm):
    assert offline_arm._cart.pending == 0
    assert offline_arm.poll_cart() is None


def test_poll_cart_does_not_steal_a_frame_from_a_running_entry(offline_arm):
    """⚠ `poll_cart()` 会读一帧 ⇒ 它**是**个抢帧者, 必须与在跑的入口互斥。

    实测 (改前): 往内核缓冲里塞一条**在飞那条命令**的 `ACK{0x3A}`, 再调 `poll_cart()` ——
    那条 ACK **被它吃掉** (`_resp` 剩 0 字节), 而它的主人随后耗满窗口报
    `MotionTimeoutError("无应答")`, **命令其实已被固件受理**。这与 F4 修的是**同一类**
    (读帧动作在别处并发发生), 只是 F4 那条是 `move_c` 的 `get_tcp()`, 这条是 `poll_cart`。

    修法是让它也走 `Arm._cart_serial` 的**非阻塞** `acquire`: 拿不到 (有笛卡尔入口在用
    这条链路, 它自己会取帧) 就返回 `None` —— "此刻没有可取的结果"本来就是本方法的 `None`
    语义, 而"别人正在用"比"抢他一帧"好。

    判据: `None` **且**那条 ACK 还在库里 (改前只满足前一半 —— 帧没了却什么都不报)。
    """
    arm = offline_arm
    arm._tr._resp.clear()
    arm._tr.push_frame(P.RSP_ACK, bytes([P.CMD_MOVE_L]))     # 在飞那条命令的 ACK
    assert len(arm._tr._resp) == 1, "用例自身失效: 帧没进库"

    held, release = threading.Event(), threading.Event()

    def _hold():
        arm._cart_serial.acquire()          # 模拟"有笛卡尔入口正持锁在跑"
        held.set()
        release.wait(5.0)
        arm._cart_serial.release()

    t = threading.Thread(target=_hold, daemon=True)
    t.start()
    try:
        assert held.wait(5.0), "用例自身失效: 没拿到锁"
        assert arm.poll_cart() is None, (
            "锁被占时 poll_cart 本该返回 None ('此刻没有可取的结果')")
        assert len(arm._tr._resp) == 1, (
            "poll_cart 把在跑那条命令的 ACK 抢走了 —— 它的主人随后会报'无应答', "
            "而命令其实已被受理 (与 F4 同一类)")
    finally:
        release.set()
        t.join(5.0)


class _KernelBufferSerial:
    """`serial.Serial` 的**保真**替身 —— `read(n)` 从"内核 RX 缓冲"取字节。

    ⚠ 与 `fake_serial.FakeTransport` 的关键差别: 这里"数据在不在库里"与"SDK 有没有去取"
    是**两件分得开**的事 (桩把 `timeout` 参数整个忽略掉, 于是"SDK 一个字节都没读"在桩上
    看不出来)。`timeout=0` 的真实语义 (非阻塞: 有就返回、没有就 `b""`) 由 `read` 体现;
    本用例只走"数据早已在库里"那一支, 所以**不**模拟阻塞等待 (不引入睡眠)。
    """

    def __init__(self, *a, **kw):
        self.timeout = kw.get("timeout", 0.2)
        self.kbuf = bytearray()
        self.is_open = True
        self.written = bytearray()

    def read(self, n=1):
        if not self.kbuf:
            return b""
        out = bytes(self.kbuf[:n])
        del self.kbuf[:n]
        return out

    def write(self, d):
        self.written += d
        return len(d)

    def flush(self):
        pass

    def close(self):
        self.is_open = False


def test_poll_cart_claims_a_result_from_a_real_transport(monkeypatch):
    """⚠ `poll_cart()` 必须在**真** `SerialTransport` 上认领得到结果 —— 桩会骗人。

    `fake_serial.FakeTransport.read_frame` **忽略 `timeout` 参数**, 于是
    `test_poll_cart_claims_a_result_that_nobody_waited_for` 在**桩**上通过 —— 而它断言的
    正是"非阻塞取帧能拿到**已到达**的帧", 真机上不成立 (旧实现 `read_frame(0.0)` 一个字节
    都不读 ⇒ `poll_cart()` 恒 `None`)。本用例把这一层换成"内核 RX 缓冲"模型, 走的仍是
    真正的 `litearm.transport.SerialTransport`。

    ⚠ 与桩用例的**分工**: 桩用例管"认领语义" (claim_resolved / 不重复交付), 本用例只管
    "真读路径确实取得到" —— 前者改桩、后者才咬得住这个缺陷。
    ⚠ 2026-09-22 读路径重构后，"真读路径"从 `poll_cart` 挪到了**读线程**（见下面那行
    `start_reader`）；判据（认领得到、且内核缓冲真的被取空）逐字不变。
    """
    monkeypatch.setitem(sys.modules, "serial",
                        types.SimpleNamespace(Serial=_KernelBufferSerial))
    from litearm.arm import _Ack
    from litearm.transport import SerialTransport

    tr = SerialTransport("faketty")
    kern = tr._ser
    kern.kbuf += P.pack_frame(          # 固件早就发来了, 只是**没人读过**
        P.RSP_CART_PLAN, _plan_frame(1, 0, n_wp=12, plan_us=4700))

    arm = Arm(port="faketty")           # 只测**读路径**: 手工接线, 不做握手/探测
    arm._tr = tr
    arm._a = _Ack(arm)
    arm._cart.register()                # 一条"登记了、但没人 wait"的在途请求
    # ⚠ **[2026-09-22] 主体改了**：`poll_cart()` 不再自己读链路（读线程是唯一读者），
    # 所以"真 transport 上取得到已到达的帧"这件事现在由**读线程**来负责验 —— 不启动
    # reader 的话，帧就永远躺在内核缓冲里。判据里那个"内核缓冲真的被取空"的检查不变。
    arm._a.start_reader(tr)
    import time as _t
    plan = None
    for _ in range(500):
        plan = arm.poll_cart()
        if plan is not None:
            break
        _t.sleep(0.002)
    assert plan is not None and (plan.ok, plan.n_wp) == (True, 12), (
        "真 transport 上认领不到**已到达**的结果 —— 非阻塞取帧没有真的去读 "
        f"(拿到 {plan!r})")
    assert kern.kbuf == bytearray(), "帧还留在'内核缓冲'里 —— 一个字节都没取走"
    assert arm._cart.pending == 0 and arm.poll_cart() is None

    #: ⚠ 收尾**必须显式 close()**: `tr` 挂在一个 `Arm` 上, 而 `Arm` **有引用环**
    #: (`_CartPending(arm=self)`) ⇒ 用例返回后它不会被引用计数回收, 要等一次全量 gc;
    #: 在那之前端口名 `"faketty"` 一直被登记表认作占用。靠"对象迟早被回收"来释放端口
    #: 就是把释放时机交给 gc —— `close()` 落在那句 `_release_port` 上, 与 gc 何时跑无关。
    tr.close()


def test_poll_cart_maps_failures_like_the_entries(offline_arm):
    arm = offline_arm
    arm._cart.register()
    arm._tr._resp.clear()
    arm._tr.push_frame(P.RSP_CART_PLAN, _plan_frame(0, 5))            # CANCELED
    _await_collector(arm)
    with pytest.raises(MotionSupersededError):
        arm.poll_cart()
    assert arm._cart.pending == 0, "结局已取走的 token 必须摘掉 (否则会错配下一条)"


# ---------------------------------------------------------------------------
# 8. 状态帧的 `CART_BUSY` (bit10)
# ---------------------------------------------------------------------------

def test_cart_busy_reads_bit10_and_is_not_a_fault_flag():
    from litearm.state import RobotState

    st = RobotState(flags=(1 << 10))
    assert st.cart_busy is True and st.flag_names == [] and st.fault_detail == "无故障位"
    assert RobotState(flags=0).cart_busy is False


def test_cart_busy_must_not_enter_flag_names():
    """⚠ `FLAG_NAMES` 只放**安全故障位** (bit0..bit5) —— `CART_BUSY`(10) 不属此列。

    真正的护栏是 `decode_status` 里那个**隐式的 `range(6)`** (`flag_names` 只看 bit0..5),
    **不是**"并进去会被打印成故障位" (实测: 往 `FLAG_NAMES` 加 `10: "CART_BUSY"` 什么都不会
    发生, 加了照样全绿)。但"现在无事发生"= 这条防线写在**别人的隐式常量**里: 谁把
    `range(6)` 改成"遍历整张表", bit6..bit8 (mode) / bit9 (enabled) / bit10 (cart_busy)
    会**一起**变成"故障位" (故障消息里冒出 `flags=CART_BUSY`)。

    故这里把**意图**钉在表面上 (键集合恰是 bit0..bit5) —— 判据能失败: 加一条
    `FLAG_NAMES[10] = "CART_BUSY"` 即红。
    """
    assert set(P.FLAG_NAMES) == set(range(6)), (
        f"`FLAG_NAMES` 的键集合变了: {sorted(P.FLAG_NAMES)} —— 它只装安全故障位 "
        f"(bit0..bit5); mode(6..8)/enabled(9)/CART_BUSY(10) 都不是故障, 不属此列")
    assert 10 not in P.FLAG_NAMES, "CART_BUSY 被当成故障位了"


class _StalledCartWrite(FakeTransport):
    """`0x3A` 的**合法载荷**写会**卡住** (等 `release`) 且**永不回**应答。

    给"持锁者长时间不放手"那条用例用 —— 那条请求会一直握着 `_cart_serial`。
    ⚠ 其它命令照常应答 (急停/失能/取 TCP 都要能在这期间走过去)。
    """

    def __init__(self, *a, **kw) -> None:
        super().__init__(*a, **kw)
        self.write_entered = threading.Event()
        self.release = threading.Event()

    def write_frame(self, cmd, payload=b""):
        if cmd == P.CMD_MOVE_L and payload:
            with self._tx_lock:
                self.tx_log.append((cmd, bytes(payload)))
                self.tx_stamps.append(time.monotonic())
            self.write_entered.set()
            self.release.wait(5.0)              # 卡住: 持锁者不放手的写照
            return None
        super().write_frame(cmd, payload)


def test_lower_energy_commands_stay_reachable_while_the_cart_lock_is_held(monkeypatch):
    """⚠ 串行锁**不许**把"降能量方向"的动作与只读查询挡在外面。

    持锁者可能阻塞到 `move_timeout` (默认 15s), 而急停/失能 (`guarded=False`) 与取状态
    必须**永远可达** —— 把它们拉进 `_cart_serial` 就等于"一条卡住的笛卡尔命令能把急停
    关在门外"。本条的桩把那条 `0x3A` 的写卡住 (于是它持锁不放), 然后断言
    `get_tcp()`/`disable()`/`emergency_stop()` 仍能在 **1s** 内跑完。

    判据能失败: 让 `emergency_stop` (或它经过的 `_write_cmd`) 去拿这把锁 ——
    这里会等到超时并报"被挡在门外"。
    """
    import litearm.arm as arm_mod

    def factory(port="fake", timeout=0.2, **_ignored):
        return _StalledCartWrite(port=port, timeout=timeout)

    monkeypatch.setattr(arm_mod, "SerialTransport", factory)
    arm = Arm(port="fake").connect()
    tr = arm._tr

    def run_cart() -> None:
        try:
            arm.move_l(TCP0, speed=0.3)
        except BaseException:                # noqa: BLE001 - 这条请求的结局不是被测对象
            pass

    holder = threading.Thread(target=run_cart, name="cart-holder", daemon=True)
    holder.start()
    assert tr.write_entered.wait(2.0), "那条 0x3A 没进到写里 (用例自身失效)"
    try:
        done = threading.Event()

        def low_energy() -> None:
            arm.get_tcp()                    # 只读
            arm.disable()                    # 降能量
            arm.emergency_stop()             # 降能量
            done.set()

        threading.Thread(target=low_energy, name="estop", daemon=True).start()
        assert done.wait(1.0), (
            "急停/失能/取 TCP 被 `_cart_serial` 挡在门外 —— 持锁者可能阻塞到 "
            "move_timeout(15s), 这条路径必须永远可达")
    finally:
        tr.release.set()                     # 放手, 别让卡住的写拖住收尾
        tr.push_frame(P.RSP_ACK, bytes([P.CMD_MOVE_L]))
        holder.join(2.0)


def test_stub_emits_the_written_busy_frame_order(offline_arm):
    """桩的 bit10 帧序**写死**: 收到 `0x4E` 后第 2 帧置 1、第 8 帧落 0。

    这条把桩的契约钉住 —— "先见 1 再见 0"的到位判据正是靠它才有主判据可测。

    ⚠ **[2026-09-22] 观测方式换了**：从前是"自己取 8 帧、逐帧看 bit10" —— 读线程一开，
    帧由读线程消费，调用方**取不到帧**了。现在看 `_Ack.state` 单槽的 `cart_busy` **轨迹**
    （每次 `seq` 变化采一个点），判据是"出现一段**恰好 6 个连续 True**，且它前后都是 False"。
    ⚠ 不钉"第 2..7 帧"这个索引（采样点由读线程的节奏决定，索引不再可观察）。
    """
    import time as _t
    arm = offline_arm
    arm.move_l(TCP0, speed=0.3, wait=False)

    frames = []
    last_seq = None
    deadline = _t.monotonic() + 2.0
    while len(frames) < 12 and _t.monotonic() < deadline:
        st = arm._a.state
        if st is not None and st.seq != last_seq:
            last_seq = st.seq
            frames.append(bool(st.cart_busy))
        _t.sleep(0.0005)

    assert True in frames and False in frames, f"没观察到 bit10 的跳变: {frames}"
    best = cur = 0
    for b in frames:                      # 最长的一段连续 True = CART_BUSY 窗口
        cur = cur + 1 if b else 0
        best = max(best, cur)
    assert best == 6, f"CART_BUSY 窗口不是 6 帧宽: {frames} (最长连续 True={best})"
    assert frames[-1] is False, (
        f"观察窗结束时 bit10 还亮着 —— 窗口没有落回 0: {frames}")


# ---------------------------------------------------------------------------
# 8b. 串行是强制的 —— ≥2 条在途会**交叉交付** (假成功)
# ---------------------------------------------------------------------------

#: 桩等"第二个写者"的窗口 (秒)。串行用法下那个写者**不会**来 (它在 `Arm._cart_serial`
#: 上排队), 于是每条用例白等这么久 —— 取 0.4s: 足够让一个已在跑的线程把自己的帧写进来。
_OVERLAP_WINDOW = 0.4

#: 线程名 -> 用例里的短标签 (桩记的是 `threading.current_thread().name`)。
_THREAD_LABELS = {"cart-A": "A", "cart-B": "B"}


class _CrossDeliverTransport(FakeTransport):
    """把"第一条"的应答推迟到"第二条"的 `ERR` **之后**的桩 —— 复现真机的帧顺序。

    真机顺序: 后受理那条被门禁拒 ⇒ `ERR` **立刻**回; 先受理那条要**等规划**才回 `0x4E`
    (10ms~1s) ⇒ `ERR` 排在先受理者的 `ACK`/`0x4E` **之前**。

    ⚠ 只对**合法载荷**的 `0x3A` 生效 (空载荷是 connect 里的能力探测, 照常走长度校验)。
    ⚠ 第二条的写要**等到那条 `ERR` 被读走**才返回 —— 否则"谁先读"变成抢跑, 用例就不是
    确定性的 (也就测不出加锁前后的差别)。
    """

    def __init__(self, *a, **kw) -> None:
        super().__init__(*a, **kw)
        self._gate = threading.Lock()
        #: 谁的 `0x3A` 真被固件受理了 —— 只有它该拿到 `CartPlan`。
        self.accepted_thread = None
        self._first_in_write = False
        self.second_write_entered = threading.Event()
        self.err_pushed = threading.Event()
        self.err_consumed = threading.Event()

    def read_frame(self, timeout=None):
        fr = super().read_frame(timeout)
        if fr is not None and fr[0] == P.RSP_ERR:
            self.err_consumed.set()
        return fr

    def write_frame(self, cmd, payload=b""):
        if cmd != P.CMD_MOVE_L or len(payload) != 28:
            return super().write_frame(cmd, payload)
        with self._gate:
            first = self.accepted_thread is None
            if first:
                self.accepted_thread = threading.current_thread().name
                self._first_in_write = True
        if first:
            try:
                if self.second_write_entered.wait(_OVERLAP_WINDOW):
                    self.err_pushed.wait(_OVERLAP_WINDOW)   # 等它的 ERR 落进队列
                return super().write_frame(cmd, payload)    # 现在才推 ACK + 0x4E
            finally:
                self._first_in_write = False
        self.second_write_entered.set()
        self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x03])))   # 固件门禁: 未使能
        overlapped = self._first_in_write
        self.err_pushed.set()
        if overlapped:
            self.err_consumed.wait(_OVERLAP_WINDOW)
        return None


def test_two_concurrent_requests_cannot_cross_deliver(monkeypatch):
    """⚠ **两条在途 = 报告与物理事实相反** —— 串行锁 (`Arm._cart_serial`) 必须把两条排开。

    复现场景 (真机形状): A、B 同时发 `0x3A`, B 被固件门禁拒 ⇒ `ERR{0x3A,0x03}`,
    而 A 那条**已受理**的 `0x4E` 因为要等规划排在 `ERR` **之后**。无锁时两条线程的
    `expect` 会**互相吃掉对方的帧** (回显只有命令码, `0x3A/0x3B/0x3E` 共用码空间,
    分辨不出来) ⇒ **被拒那条报的不是它自己那条 `ERR`**。

    判据必须**逐线程** ("结局集合里有一个成功、一个失败"在错配下**也成立** —— 它恰好
    就是无锁时的观测): 被受理那条 (由桩记下是哪个线程) 必须拿到 `CartPlan`; 另一条
    **绝不许**是 `CartPlan`, 必须是它自己那条 `CommandRejectedError`。

    ⚠ **咬住这条用例的是最后那条"逐线程归因"** —— 实测去掉锁之后, 前两条 (被受理者拿到
    `CartPlan` / 另一条不是 `CartPlan`) **照样通过**, 只有归因那条会红 (被拒那条报的是
    `MotionTimeoutError`)。原因: `_Ack.expect(..., err_waits_for_ack=True)` 让一条按命令码
    匹配的 `ERR` 不再当场作数 (见 `_request_and_wait`), 于是"错拿别人的 ERR ⇒ 摘掉自己的
    token ⇒ 下一条假成功"这条**旧**形态不再发生。锁仍然不可省, 但理由换成了**顺序**:
    `_request_and_wait` 先登记后写, 无锁时登记顺序与固件受理顺序可以倒置, 而 `0x4E` 里
    没有命令 id ⇒ 应答配错 token (失败那条被报成成功)。
    """
    import litearm.arm as arm_mod

    def factory(port="fake", timeout=0.2, **_ignored):
        return _CrossDeliverTransport(port=port, timeout=timeout)

    monkeypatch.setattr(arm_mod, "SerialTransport", factory)
    arm = Arm(port="fake").connect()
    tr = arm._tr
    # ⚠ `connect()` 里的能力探测**已经读过一条** `ERR` (空载荷撞长度校验那条), 于是
    # `err_consumed` 在起跑前就是置位的 —— 不清掉的话"第二条写"会直接冲进自己的 `expect`,
    # 抢先把 `ERR` 读走, 这条用例就**测不出**加锁前后的差别 (跑出一个"碰巧正确"的结局)。
    tr.err_consumed.clear()
    out = {}
    start = threading.Barrier(2)

    def run(name: str) -> None:
        start.wait()
        try:
            out[name] = arm.move_l(TCP0, speed=0.3)
        except BaseException as e:          # noqa: BLE001 - 结局本身就是被测的东西
            out[name] = e

    ta = threading.Thread(target=run, args=("A",), name="cart-A")
    tb = threading.Thread(target=run, args=("B",), name="cart-B")
    ta.start()
    tb.start()
    # ⚠ `join` 的上限取 **5s** (不是 30s): 它只在"判据失效"那一支上花时间 ——
    # `RLock`→`Lock` 的变异会让同线程二次获取**自己锁死**, 那时两条线程都回不来,
    # 两个 `join` 的下限就是这一支的**实测时长** (30s 会把整轮拖住 60s)。
    # 合法路径的时长由 `_OVERLAP_WINDOW` (0.4s) 决定, 5s 有 10 倍余量。
    ta.join(5)
    tb.join(5)

    assert set(out) == {"A", "B"}, f"线程没回来: {sorted(out)}"
    accepted = _THREAD_LABELS.get(tr.accepted_thread)
    assert accepted in ("A", "B"), (
        f"桩没记下哪条被受理 (拿到 {tr.accepted_thread!r}) —— 用例自身失效")
    other = "B" if accepted == "A" else "A"

    assert isinstance(out[accepted], CartPlan), (
        f"{accepted} 那条**被固件受理**了, 却报 {out[accepted]!r} —— "
        f"它的 expect 认下了另一条命令的 ERR (按命令码判, 0x3A 共用码空间)")
    assert not isinstance(out[other], CartPlan), (
        f"{other} 那条**被固件拒绝**了, 却拿到一个 CartPlan ({out[other]}) —— "
        f"假成功: 它把被受理那条的 0x4E 配给了自己")
    assert isinstance(out[other], CommandRejectedError), (
        f"{other} 的结局是 {out[other]!r}, 应为固件那条 ERR 对应的 CommandRejectedError")


def _safe(fn):
    """跑一下, 把异常当结局返回 (逐线程判据用)。"""
    try:
        return fn()
    except BaseException as e:                              # noqa: BLE001
        return e


class _GatedPlanTransport(FakeTransport):
    """第一条**合法载荷**的 `0x3A` 受理后**扣住** `0x4E`, 直到 `release` 被置位。

    给"持锁者还在跑"造一个确定的窗口 (它靠 `wait=True` 一直持着 `Arm._cart_serial`),
    于是"别人能不能在这期间**读帧**"变成可判的。判据落在**下行时序**上 (不依赖"谁先读到"
    这种抢跑): `GET_TCP (0x43)` 是 `move_c` 起点校验那次读, 它若在 `release` 之前就写出去,
    就是在**锁外**读的。
    """

    def __init__(self, *a, **kw) -> None:
        super().__init__(*a, **kw)
        self.release = threading.Event()
        self.gate_open = False
        #: 下行时序 (只看这两条): "plan" = 在飞那条的 `0x4E` 被推出; "tcp" = GET_TCP 被写
        self.order: list = []
        self.early_tcp_read = False

    def _cart_cmd(self, cmd: int, payload: bytes) -> None:
        if cmd == P.CMD_MOVE_L and payload and not self.gate_open:
            self.gate_open = True
            self._push(P.pack_frame(P.RSP_ACK, bytes([cmd])))
            if not self.release.wait(5.0):       # 扣住 0x4E
                raise AssertionError("用例自身失效: release 一直没来")
            self.order.append("plan")
            # ⚠ `_push` 收的是**整帧**, `push_frame` 收的是 (id, 载荷) —— 别把载荷喂给
            # `_push` (它会当成帧塞进队列, 再被 `unpack_frame` 判成坏帧**静默丢掉**)。
            self.push_frame(P.RSP_CART_PLAN,
                            _plan_frame(1, 0, n_wp=self.cart_n_wp,
                                        plan_us=self.cart_plan_us))
            return
        super()._cart_cmd(cmd, payload)

    def write_frame(self, cmd: int, payload: bytes = b"") -> None:
        if cmd == P.CMD_GET_TCP:
            self.order.append("tcp")
            if self.gate_open and not self.release.is_set():
                self.early_tcp_read = True       # ← 在跑那条还没收尾就写了 = 锁外读
        super().write_frame(cmd, payload)


def test_move_c_reads_the_tcp_inside_the_cart_lock(monkeypatch):
    """⚠ `move_c` 起点校验那次 `get_tcp()` 必须在 `Arm._cart_serial` **之内** —— 它读的是
    "此刻 TCP", 与正在跑的那条运动本来就是同一个临界区。

    锁外读的代价 (E4b 实测 8/8): 那条 `get_tcp` 在**别的线程**正跑着一条笛卡尔轨迹时把
    它的 `ACK{0x3A}` 抢走 —— `expect` 只认自己读到的那条, 被抢走就**再也找不回来** ⇒
    **受害者报 `MotionTimeoutError`"无应答", 而它的命令早已写进固件并被受理**: 报告与
    物理事实相反。

    判据 (**下行时序**, 与"谁先读到帧"无关): A 那条 `0x4E` 被推出**之前**, `GET_TCP`
    **一个都不许**写出去。⚠ 造窗口靠 `release` (0.3s 垫底), 所以本用例**不会**误报失败,
    极端负载下可能误报**通过** —— 判据仍能咬住实现 (去掉锁会立刻变成 `early_tcp_read`)。

    ⚠ `move_timeout` 取小值 (0.5) 只为让**修复前**那支快点收场: 那里 A 会先耗满 1.0s 的
    ACK 窗口, 而它留在队里的那条陈旧 token 又会吃掉 B 的 `0x4E` ⇒ B 白等一个
    `move_timeout`。修复后两条都按正常路径走完 (实测 ~0.4s)。
    """
    def factory(port="fake", timeout=0.2, **_ignored):
        return _GatedPlanTransport(port=port, timeout=timeout)

    import litearm.arm as _arm_mod

    monkeypatch.setattr(_arm_mod, "SerialTransport", factory)
    arm = Arm(port="fake", move_timeout=0.5).connect()
    tr = arm._tr
    out = {}

    def run_l():
        try:
            out["l"] = arm.move_l(TCP0, speed=0.3)         # wait=True: 持锁到停稳
        except BaseException as e:                          # noqa: BLE001
            out["l"] = e

    ta = threading.Thread(target=run_l, name="cart-A")
    ta.start()
    deadline = time.monotonic() + 2.0
    while not tr.gate_open and time.monotonic() < deadline:
        time.sleep(0.005)
    assert tr.gate_open, "用例自身失效: 第一条 0x3A 没被受理 (拿不到'在跑'这个窗口)"

    tb = threading.Thread(target=lambda: out.update(
        c=_safe(lambda: arm.move_c(TCP0, (0.35, 0.0, 0.32, 0.0, 0.0, 0.0), TCP0,
                                   speed=0.3, wait=False))), name="cart-B")
    tb.start()
    time.sleep(0.3)                       # 给 B 充足的时间去"抢读" (修复后它只能排队)
    tr.release.set()
    ta.join(5)
    tb.join(5)

    assert isinstance(out.get("l"), CartPlan), f"A 那条没跑通: {out.get('l')!r}"
    assert isinstance(out.get("c"), CartPlan), f"B 那条没跑通: {out.get('c')!r}"
    assert not tr.early_tcp_read, (
        "`move_c` 在**锁外**读了 TCP —— 它是在飞那条还没收尾时把 GET_TCP 写出去的, "
        "那一次读会抢走在跑那条的 ACK = 受害者报 '无应答' 而命令其实已生效")
    assert tr.order[:1] == ["plan"], (
        f"下行时序是 {tr.order} —— 那条在飞请求的 0x4E 必须先出; 它后面的 'tcp' 里既有 "
        f"A 自己的到位回读 (`_tcp_reached`), 也有 B 那次起点校验")


class _BeginRendezvousTransport(FakeTransport):
    """给"`move_path` 的 BEGIN/ADD 也在不在串行锁里"造窗口的桩。

    第一条 `0x3C` (BEGIN) **先落进 `tx_log`, 再卡住** `_OVERLAP_WINDOW` 等第二个写者 ——
    于是无锁时第二个线程能在它卡住期间把自己的 BEGIN/ADD/RUN **整段**写完 (两条请求的
    帧在 `tx_log` 里交错); 有锁时第二个线程进不来 (它在 `Arm._cart_serial` 上排队),
    那个写者永远不来 ⇒ 每条用例白等 `_OVERLAP_WINDOW` (与 `_CrossDeliverTransport` 同款,
    是知情的代价)。

    ⚠ "先落 log 再卡住"是**承重**的: 反过来 (卡住后再落 log) 会把第一条的整段帧推迟到
    第二条之后, 于是**无锁时也看着是连续的** —— 那条用例就测不出加锁前后的差别。

    ⚠ **建模缺口**: 本桩对**第二条** `0x3C` 也回 `RSP_ACK`, 而真固件会拒
    (`cart_req_begin` 只在 `IDLE` 受理, `cart_exec.c:304` ⇒ `ERR{0x3C,0x04}`)。
    故本桩只建模"锁把并发变成排队"这一面, 不建模"真固件下第二条根本发不出 ADD"。
    """

    def __init__(self, *a, **kw) -> None:
        super().__init__(*a, **kw)
        self._begin_writers = 0
        self.first_begin_logged = threading.Event()
        self.second_writer_entered = threading.Event()
        self._begin_lock = threading.Lock()

    def write_frame(self, cmd, payload=b""):
        if cmd != P.CMD_CART_BEGIN:
            return super().write_frame(cmd, payload)
        with self._begin_lock:
            first = self._begin_writers == 0
            self._begin_writers += 1
        with self._tx_lock:                       # 与 FakeTransport 同款记账 (先落 log)
            self.tx_log.append((cmd, bytes(payload)))
            self.tx_stamps.append(time.monotonic())
        if first:
            self.first_begin_logged.set()
            self.second_writer_entered.wait(_OVERLAP_WINDOW)
        else:
            self.second_writer_entered.set()
        self._push(P.pack_frame(P.RSP_ACK, bytes([cmd])))


#: `move_path` 一次调用在 `tx_log` 里留下的**笛卡尔帧序** (过滤掉其它命令后):
#: BEGIN + ADD(0) + ADD(1) + RUN —— 两条请求各一份, **两次调用不许交错**。
_MOVE_PATH_FRAME_ORDER = [(P.CMD_CART_BEGIN, None), (P.CMD_CART_ADD, 0),
                          (P.CMD_CART_ADD, 1), (P.CMD_CART_RUN, None)]


def test_move_path_begin_and_add_are_inside_the_serial_lock(monkeypatch):
    """⚠ `move_path` 的 **BEGIN + ADD×n + RUN 整段**必须在 `Arm._cart_serial` 里。

    只圈 RUN (经 `_request_and_wait`) 是不够的: 固件的 `RECV` 收集态**只从 `IDLE` 开**
    (`cart_exec.c:304`: `state != CART_ST_IDLE` ⇒ `CART_REQ_STATE`), 两条并发 `move_path`
    里**后到的那条会在 BEGIN 就被拒** (`ERR{0x3C,0x04}`), 于是它连自己的 ADD 都发不出去
    —— 那是**响亮失败**, 不是静默串路。持锁把它**变成排队** (两条都跑完整段), 并把
    `BEGIN→RUN` 整段与其它笛卡尔入口互斥。

    ⚠ **桩的建模缺口 —— 别按错模型去改这个用例**: 本桩
    (`_BeginRendezvousTransport`) 对**第二条** BEGIN 也回 `RSP_ACK`, 所以**不持锁时**
    第二条会照常把 ADD/RUN 写完、`tx_log` 里真的交错 (用例**红**)。真固件不会这样: 它的
    第二条 BEGIN 会被 `0x04` 拒。⇒ 本用例绿的原因是"**锁让它们排队**", **不是**"不锁就会
    拼出同一条路径"。
    ⚠ 本段原写"固件的 `RECV` 缓冲按**到达顺序**收路点 ⇒ 两条请求的路点会拼进**同一条**
    收集态" —— **归因错**: 缓冲按帧里的 `idx` 索引而非到达顺序 (`cart_exec.c:321`), 且
    重复 idx 被去重 (`cart_exec.c:320`), 而同一时刻只可能存在**一条**收集会话 (上面那条
    `IDLE` 判据)。

    判据: `tx_log` 里过滤出 `0x3C/0x3D/0x3E` 后的帧序必须**恰好**是两份连续的路点块。
    红时的实测帧序是 `3C, 3C, 3D(0), 3D(1), 3E, 3D(0), 3D(1), 3E`
    —— 第二条的 BEGIN 插在第一条的 BEGIN 与它的 ADD 之间。
    """
    import litearm.arm as arm_mod

    def factory(port="fake", timeout=0.2, **_ignored):
        return _BeginRendezvousTransport(port=port, timeout=timeout)

    monkeypatch.setattr(arm_mod, "SerialTransport", factory)
    arm = Arm(port="fake").connect()
    tr = arm._tr
    out = {}

    def run(name, poses):
        try:
            out[name] = arm.move_path(poses, speed=0.3)
        except BaseException as e:          # noqa: BLE001 - 结局不是本条的被测对象
            out[name] = e

    pa = [(0.30, 0.0, 0.35, 0.0, 0.0, 0.0), (0.31, 0.0, 0.35, 0.0, 0.0, 0.0)]
    pb = [(0.32, 0.0, 0.35, 0.0, 0.0, 0.0), (0.33, 0.0, 0.35, 0.0, 0.0, 0.0)]
    ta = threading.Thread(target=run, args=("A", pa), name="cart-A", daemon=True)
    ta.start()
    assert tr.first_begin_logged.wait(2.0), "第一条 BEGIN 没写进来 (用例自身失效)"
    tb = threading.Thread(target=run, args=("B", pb), name="cart-B", daemon=True)
    tb.start()
    ta.join(5)                              # 上限同前: 只给"判据失效"那一支用 (0.4s 的余量)
    tb.join(5)
    assert set(out) == {"A", "B"}, f"线程没回来: {sorted(out)}"

    got = [(c, p[0] if c == P.CMD_CART_ADD else None)
           for c, p in tr.tx_log if c in (P.CMD_CART_BEGIN, P.CMD_CART_ADD, P.CMD_CART_RUN)]
    assert got == _MOVE_PATH_FRAME_ORDER * 2, (
        f"两条 `move_path` 的 BEGIN/ADD 交错了下行帧: {got} —— "
        f"锁没能把第二条拦在 BEGIN 之前 (真固件会在它的 BEGIN 回 0x3C/0x04, "
        f"见用例 docstring 的'桩的建模缺口')")


# ---------------------------------------------------------------------------
# 9. 位姿入参的形态 —— 三条入口要收下 `movel/movec` 收过的**全部**写法
# ---------------------------------------------------------------------------

def _R_from_rpy(r: float, p: float, y: float):
    """照固件 `kin_rpy_to_rot` 的定义**重写一遍** (ZYX 内旋)。

    刻意不用被测代码 (`rpy_to_mat`) 造输入: 拿被测函数构造期望值, 两边一起错也测不出来。
    """
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp,     cp * sr,                cp * cr]]


def test_a_pose_pair_produces_the_same_frame_as_the_six_scalars(offline_arm):
    """⚠ `(pos[3], R[3x3])` 与等价的 6 标量必须产生**逐字节相同**的下行载荷。

    这三条入口替代的 `movel/movec` 收两种写法 ("固件的 `[x,y,z,r,p,y]` **或**
    pylitearm 的 `(position[3], rotation[3x3])`"), 只收 6 标量就是**公开 API 的能力
    倒退**。判据取"逐字节相同"而不是"都能跑": 两条路径必须殊途同归到**同一帧**。

    ⚠ **它守的是"两条路径互相一致", 不是绝对值**: 两边**一起错** (例如 rpy 约定整体
    反了、`mat_to_rpy` 与固件的 `kin_rot_to_rpy` 一起偏) 时它照样绿 —— 因为输入里的
    `R` 正是用同一套约定造出来的。真正钉住绝对值的是同文件里**用独立重写的
    `_R_from_rpy` 造输入、再断言载荷逐字节等于原 rpy** 的那几条
    (`test_a_homogeneous_4x4_pose_is_accepted_too` /
    `test_move_c_pose_start_also_accepts_a_pose_pair`) 以及上面那两条直接断言
    6 标量载荷的用例。别以为本条守住了绝对值。
    """
    arm = offline_arm
    pos, rpy = (0.31, 0.0, 0.36), (0.2, 0.3, 0.4)
    arm.move_l(list(pos) + list(rpy), speed=0.3)
    from_scalars = _payloads(arm, P.CMD_MOVE_L)[-1]
    arm.move_l((pos, _R_from_rpy(*rpy)), speed=0.3)
    from_pair = _payloads(arm, P.CMD_MOVE_L)[-1]
    assert from_scalars == from_pair, "两种写法下发的载荷不同"


def test_a_homogeneous_4x4_pose_is_accepted_too(offline_arm):
    """4x4 齐次矩阵也收 (`as_pose` 的第三种形态)。"""
    arm = offline_arm
    pos, rpy = (0.31, 0.0, 0.36), (0.2, 0.3, 0.4)
    R = _R_from_rpy(*rpy)
    M = [R[0] + [pos[0]], R[1] + [pos[1]], R[2] + [pos[2]], [0.0, 0.0, 0.0, 1.0]]
    arm.move_l(M, speed=0.3)
    ps = _payloads(arm, P.CMD_MOVE_L)[-1]
    assert _is_f32(P.unpack_f32s(ps, 0, 6), list(pos) + list(rpy))


def test_move_c_pose_start_also_accepts_a_pose_pair(offline_arm):
    """`pose_start` (要发 `get_tcp` 比对的起点) 同样收两种形态 —— 旧 `movec` 也收。

    ⚠ 三个位姿都必须是**完整位姿** (6 标量或位姿对): `[x,y,z]` 这种 3 元素写法
    从来就不被收 (`as_pose` 与旧的 `_as_pose6` 一致) —— `pose_via` 的**姿态被固件忽略**
    是固件侧的事, 载荷里那一格仍然存在。
    """
    arm = offline_arm
    arm.move_c(((TCP0[0], TCP0[1], TCP0[2]), _R_from_rpy(0.0, 0.0, 0.0)),
               ((0.30, 0.0, 0.40), _R_from_rpy(0.0, 0.0, 0.0)),
               (0.32, 0.0, 0.40, 0.0, 0.0, 0.0), speed=0.3)
    assert P.CMD_MOVE_C in _sent(arm), "位姿对形态的位姿被拒了"
    ps = _payloads(arm, P.CMD_MOVE_C)[-1]
    assert _is_f32(P.unpack_f32s(ps, 0, 6), [0.30, 0.0, 0.40, 0.0, 0.0, 0.0])
    assert _is_f32(P.unpack_f32s(ps, 24, 6), [0.32, 0.0, 0.40, 0.0, 0.0, 0.0])


def test_move_path_waypoints_also_accept_a_pose_pair(offline_arm):
    """`move_path` 的**每一个**路点同样收两种形态 —— 逐点 `_as_pose6`, 不是只认 6 标量。

    与上面两条同族 (同一形态在三条入口上各有一条用例)。⚠ 判据必须**逐点**比、且**混着传**:
    `move_path` 的实现是 `pts = [_as_pose6(p, "move_path") for p in poses]` —— 一个路点一个
    路点各转各的, 所以"第一个点收了"证明不了第二个点也收 (整段只转一次的实现照样过)。

    ⚠ 它守的**不是**"位姿对语法转得对" (那由 `test_a_pose_pair_produces_the_same_frame_as_
    the_six_scalars` 的**逐字节**比对守): 这里只钉"`move_path` 的路点走的是同一条转换",
    防的是将来有人把这条入口改成只收 6 标量。
    """
    arm = offline_arm
    pair = ((0.30, 0.0, 0.40), _R_from_rpy(0.0, 0.0, 0.0))
    arm.move_path([TCP0, pair], speed=0.3)
    adds = _payloads(arm, P.CMD_CART_ADD)
    assert len(adds) == 2, "路点没全发出去 (位姿对形态的路点被拒了)"
    assert adds[0][0] == 0 and adds[1][0] == 1, "idx 不连续"
    assert _is_f32(P.unpack_f32s(adds[0], 1, 6), TCP0)
    assert _is_f32(P.unpack_f32s(adds[1], 1, 6), [0.30, 0.0, 0.40, 0.0, 0.0, 0.0]), (
        "位姿对形态的路点没有走 `_as_pose6` 那条转换")


@pytest.mark.parametrize("bad,shape", [
    ([0.1] * 9, "len=9"),          # 扁平 9 元素旋转矩阵 (位置不可省)
    ((0.1, 0.2), "len=2"),
    (3.14, "float"),
    ("0.3,0,0.35,0,0,0", "str"),
])
def test_an_illegal_pose_reports_the_shape_that_was_actually_received(
        offline_arm, bad, shape):
    """非法形态抛 `InvalidCommandError`, 且**消息里含实际收到的形状**。

    只说 "pose 非法" 会让调用方在 6 向量与位姿对之间反复猜 —— 这正是 `as_pose` 那条
    文案存在的理由, 别在外面包一层把它盖掉。
    """
    arm = offline_arm
    before = list(arm._tr.tx_log)
    with pytest.raises(InvalidCommandError) as ei:
        arm.move_l(bad, speed=0.3)
    assert shape in str(ei.value), f"报错没有给出实际形状 ({shape})"
    assert arm._tr.tx_log == before, "形态非法却已经发了帧"
    assert arm._cart.pending == 0


@pytest.mark.parametrize("bad,reason", [
    ([0.1, 0.2, 0.3, 0.4, 0.5, "x"], "float"),              # 6 标量里混了字符串
    (((0.1, 0.2, 0.3), [1, 2, 3]), "not iterable"),         # 位姿对的 R 不是"行的序列"
])
def test_malformed_pose_values_stay_inside_the_lite_arm_error_family(
        offline_arm, bad, reason):
    """⚠ 畸形**数值** (`ValueError`/`TypeError`) 也必须收进 `LiteArmError` 体系。

    这两类发生在 `as_pose` 的**数值转换**里 (`float(v)` / `list(row)`), 与上面那条
    "形态不合法" 是**同一种处境**、同一种归因 —— 而且对老版本**正是** `InvalidCommandError`:
    `7e8c2f4` 的 `_as_pose6` 写着 `except (TypeError, ValueError): raise InvalidCommandError`。
    把形态判定整个委托给 `_rot.as_pose` 之后漏了这一步 ⇒ **回归**: 调用方按 `LiteArmError`
    做分支 (重连/改参数/放弃) 的代码对这几种输入会全部失效 (帧未发出, 方向安全,
    但异常类型契约破了)。

    判据: 类型是 `InvalidCommandError` (**不是** `builtins.ValueError`/`TypeError`),
    文案里带上**哪条入口**, 且**原文不丢** (它含实际收到的值/形状), 帧也没发出去。
    """
    arm = offline_arm
    before = list(arm._tr.tx_log)
    with pytest.raises(InvalidCommandError) as ei:
        arm.move_l(bad, speed=0.3)
    assert "move_l" in str(ei.value), "报错没说是哪条入口的位姿"
    assert reason in str(ei.value), "`as_pose` 那句原文被盖掉了"
    assert arm._tr.tx_log == before, "入参畸形却已经发了帧"
    assert arm._cart.pending == 0


def test_cart_start_tolerances_track_move_p_defaults():
    """⚠ `move_c` 的起点容差必须**跟随** `Arm.move_p` 的默认值 (同源的, 不是巧合)。

    `move_c` 判的就是"实际 TCP 是否在 `pose_start` 附近", 与 `move_p` 的到位判据必须是
    **同一把尺子** —— 否则 `move_p` 刚判定"到位"的位置会被 `move_c` 判成"起点不一致"。
    这两个常量是从 `move_p` 的默认值**复制**来的, 没有任何机制保证它们跟随; 判据能失败:
    把 `Arm.move_p` 的 `pos_tol`/`rpy_tol` 默认值改一个数, 这条即红。
    """
    import inspect

    params = inspect.signature(Arm.move_p).parameters
    assert CART_START_POS_TOL == params["pos_tol"].default, (
        f"move_c 的起点位置容差 {CART_START_POS_TOL} 与 move_p 的默认 "
        f"{params['pos_tol'].default} 不一致 —— 两把尺子分叉了")
    assert CART_START_RPY_TOL == params["rpy_tol"].default, (
        f"move_c 的起点姿态容差 {CART_START_RPY_TOL} 与 move_p 的默认 "
        f"{params['rpy_tol'].default} 不一致 —— 两把尺子分叉了")
