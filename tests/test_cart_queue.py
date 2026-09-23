"""Task 3: `_CartPending` 队列 + `0x4E` 收集器 + 清队钩子。

**为什么**: 固件原生笛卡尔 (`0x3A/0x3B/0x3E` + `RSP_CART_PLAN 0x4E`) 的应答载荷里
**没有命令 id**, 只能按**受理顺序** FIFO 配对。而"作废在途规划"在固件侧有两条
**语义完全相反**的路径:

* 走 `cart_invalidate_before_motion()` 的 opcode → **静默作废** (一条 `0x4E` 都不发)
  → 主机必须**在发出前**把队清掉, 否则陈旧 token 会被下一个 `0x4E` 错配;
* `0x3A/0x3B/0x3E` → **绝不清队** (被取代的那条必有 CANCELED), 清队会让
  "报告"与"物理事实"相反。

**本 Task 钉死的三条**:

1. 清队集合 = **12** 个 opcode (固件 `cart_invalidate_before_motion()` 的调用点归约),
   `0x3A/0x3B/0x3E` **不在**里面;
2. 清队**只挂在最底层写口** `Arm._raw_write` 上 —— 零重力保活线程刻意绕过
   `_write_cmd` (避开零重力守卫), 但它**不该**绕过清队: 它发的 `0x06` 正是会静默
   作废在途规划的那一类;
3. `0x4E` 一律由 `Arm._read_one` 交给收集器 (**不返回给调用方**、也不计入未识别帧);
   队列空却收到应答 = "多了一条" → **计数 + 报错**, 不静默丢弃。

⚠ 清队**销毁在途记账**, 于是后来那条 `0x4E` 归属不明 (载荷里没有命令 id): 它可能是
"清队后固件根本没发" 那支 (无应答), 也可能是 "清队那一刻固件**已经跑完**、应答早躺在
主机 RX 缓冲里" 那支。后者属于刚被清掉的那条请求 —— 按"多了一条"无条件报错会让
`LiteArmError` 从毫不相干的读路径里炸出来, 并把一条**已经成功**的运动报成"结局未知"。
故清队**记吸收额度**, 见本文件第 5 节 (Task 3.5)。

⚠ 本 Task (历史坐标) **只**做队列/收集器/清队 —— 当时把 `CartPlan`、入口方法、能力探测
**留给后续 Task**; 三者今天**都已交付并有测试** (别把这句读成"今天还没有"):
`CartPlan.from_reply` 见 `cart.py` / 用例 `test_cart_protocol.py::test_from_reply_reads_ok_and_err_from_their_own_bytes`;
入口方法与探测在同文件 (`test_probe_*` / `test_move_l_*` / `test_move_c_*` / `test_move_path_*`)
与 `tests/test_cart_wait.py`。
"""
from __future__ import annotations

import pathlib
import sys
import threading
import time
import types

import pytest

import litearm.arm as arm_mod
import litearm.cart as cart_mod
from fake_serial import FakeTransport
from litearm import _protocol as P
from litearm.arm import _ZG_THREAD_NAME
from litearm.cart import _CART_CLEARS_UPON, _UNCLAIMED_MAX, _CartPending
from litearm.errors import (CartReplyLostError, CommandRejectedError,
                                 LiteArmError, MotionTimeoutError,
                                 TransportError)

#: 固件 `cart_invalidate_before_motion()` 的调用点归约出的 opcode —— **12 个**
#: (实测 `control_loop.c` 11 处 + `kin_runner.c:176` 1 处; 其中 `0x2A` home 复用
#: `ctrl_accept_move_j` 那一处, 故"调用点数"= "opcode 数" = 12)。
#:
#: ⚠ 提示语里写的"13 个 opcode"与规格表 (12 行) 和固件实测都不符 —— 以表/固件为准。
EXPECTED_CLEARS = frozenset({
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,     # move_j/P/js/mit/mit_all/zero_g/j_sync
    0x11, 0x12, 0x13, 0x14, 0x2A,                 # disable/estop/clear_faults/reset/home
})


def _push(arm, fid: int, payload: bytes = b"") -> None:
    """往桩的响应队列里塞一帧 (`FakeTransport._push(frame)`)。"""
    arm._tr._push(P.pack_frame(fid, payload))


def _plan_payload(ok: int = 1, err: int = 0, n_wp: int = 12,
                  plan_us: int = 4700) -> bytes:
    """`RSP_CART_PLAN (0x4E)` 载荷 (8B): `ok u8 + err u8 + n_wp u16 LE + plan_us u32 LE`。"""
    return bytes([ok, err]) + n_wp.to_bytes(2, "little") + plan_us.to_bytes(4, "little")


# ---------------------------------------------------------------------------
# 1. 清队集合 —— 边界的两侧各钉一遍
# ---------------------------------------------------------------------------

def _settle(arm, timeout: float = 1.0) -> None:
    """等读线程把**已经推入桩的**帧投递完 —— 旧 `arm._read_one(0.2)` 的等价物。

    ⚠ 旧写法是"调用方驱动取一帧"（那时 SDK 没有读线程）；现在是"**等**那一帧被读线程
    取走并投递"。判据 = 桩的接收队列排空 + 一小段静默（投递异步，见 `_Ack._reader_loop`）。
    ⚠ 它**不做任何归属判定** —— 那正是重构要消灭的动作。
    """
    import time as _t
    end = _t.monotonic() + timeout
    while _t.monotonic() < end and arm._tr._resp:
        _t.sleep(0.002)
    _t.sleep(0.01)


def test_clear_set_is_exactly_the_twelve_firmware_opcodes():
    """集合**恰好**是固件那 12 个 (多一个=误伤笛卡尔应答, 少一个=泄漏 token)。

    逐 opcode 的固件依据:
      `0x01` ctrl_accept_move_j / `0x02` kin_runner_request_move_p /
      `0x03` ctrl_accept_move_js / `0x04` ctrl_accept_move_mit /
      `0x05` ctrl_accept_move_mit_all / `0x06` ctrl_accept_zero_g /
      `0x07` ctrl_accept_move_j_sync / `0x11` ctrl_disable /
      `0x12` ctrl_emergency_stop / `0x13` ctrl_clear_faults /
      `0x14` ctrl_reset / `0x2A` home (复用 ctrl_accept_move_j)。
    """
    assert _CART_CLEARS_UPON == EXPECTED_CLEARS, (
        f"清队集合与固件对不上: 多={sorted(_CART_CLEARS_UPON - EXPECTED_CLEARS)} "
        f"少={sorted(EXPECTED_CLEARS - _CART_CLEARS_UPON)}")


def test_clear_set_never_includes_the_cart_commands():
    """⚠ 0x3A/0x3B/0x3E 绝不清队 —— 它们欠的是**两条已知应答** (A 的 CANCELED + B 的结果)。

    危害由 `test_the_clear_set_must_not_swallow_the_two_known_replies` 实地摆出来;
    一句话: 清队把**已知**的结局 (接管 / 成功) 报成**未知**, 并把 B 的结果丢掉。
    ⚠ 别写成"B 会收到 `MotionSupersededError`、于是不去纠正" —— 那是后果链说反
    (`MotionSupersededError` 只在**读到** `err=5` 时产生, 而这里两条应答都被吸收了)。
    """
    for cmd in (0x3A, 0x3B, 0x3E):
        assert cmd not in _CART_CLEARS_UPON, f"0x{cmd:02X} 欠两条 0x4E, 清队会错配"


#: 手工走 `_CartPending` 那几步的用例用的**定值** TTL —— 见
#: `test_the_clear_set_must_not_swallow_the_two_known_replies` 的 docstring:
#: 这几条验的是**配对/吸收语义**, 不是"额度能活多久", 故不许跟 `_ABSORB_TTL_FLOOR` 耦合
#: (耦合之后把那个常量改成 0.0 会让它们红在错的地方 —— 判据一句都没验到)。
_FIXED_TTL = 15.0


def test_the_clear_set_must_not_swallow_the_two_known_replies():
    """⚠ "清队集合为什么**恰好**是那 12 个"的另一半: 把假设摆出来, 看清代价落在谁头上。

    不改清队集合, 只手工走一遍 `_CartPending` 上那几步 —— 顺序按真实代码:
    `request()` 是**先登记、后写**, 清队钩子挂在**写口** (`_raw_write`) 上。

    1. A 已在飞 (PLANNING);
    2. B 登记 —— 它的 token **此刻已在队里**, 而那一次写会触发清队;
    3. 清队之后, 固件那两条应答才到: A 的 CANCELED (取代它的正是 B) 与 B 自己的结果。

    判据 (每一条都是"报告与物理事实相反"的一种):
    ① 清队摘掉的是 **2** 条 (A 与 B 自己的) ⇒ 两位调用方**都**被判"结局未知",
       而真相是"A 被**预期内**地接管 + B 马上就要跑";
    ② 那两条应答**全数被吸收** (`absorbed_replies == 2`) —— B 那条本该交付的规划结果
       被**丢掉**, 谁也没拿到;
    ③ 谁也没读到 `err=5` (所以**不**是 `MotionSupersededError`) —— "接管"与"结局未知"
       在报告上分不开, 而前者按约定是**不该纠正**的。

    ⚠ **TTL 刻意用一个与常量解耦的定值** (`_FIXED_TTL`), 不用 `_ABSORB_TTL_FLOOR`:
    后者被改成 `0.0` 时, 这里现构造的 `_CartPending` 的额度当场过期 ⇒ `on_reply` 走
    "队列空 + 无额度"那条路抛 `LiteArmError`, 用例红在 `on_reply` **内部**,
    上面三条判据**一个字都没验到** (归因错)。"下限本身够不够"由
    `test_the_absorb_ttl_floor_is_the_firmware_planning_bound_plus_margin` 单独守。
    """
    q = _CartPending(absorb_ttl=_FIXED_TTL)
    a = q.register()                    # A 已在飞 (PLANNING)
    b = q.register()                    # B: 登记**先于**写 (见 `_CartPending.request`)
    assert q.clear_pending("假设 0x3A 在清队集合里") == 2, (
        "清队只摘掉了在飞那条 —— 而 `request()` 是先登记后写, B 那条此时已经在队里了")

    assert "结局未知" in (a.lost or "") and "结局未知" in (b.lost or ""), (
        "清队没把两条都判成结局未知 —— 与上面那句 `== 2` 不符")
    q.on_reply(_plan_payload(ok=0, err=5))             # 固件为 A 发的 CANCELED
    q.on_reply(_plan_payload(ok=1, err=0, n_wp=12))    # B 自己的结果
    assert (q.absorbed_replies, q.extra_replies) == (2, 0), (
        "两条已知应答应当**双双被吸收** (2 格额度正对 2 条应答): 既没被交付, "
        "也没被当成脱同步报出来")
    assert a.reply is None and b.reply is None, (
        "有谁拿到了应答 —— 与'两条都被吸收'不符 (如果 B 拿到了, 那才是假成功)")


def test_clear_set_excludes_the_other_cartesian_and_readonly_opcodes():
    """另外几个**看着像**却不清队的: BEGIN/ADD 与 GET_IK。

    * `0x3C` BEGIN / `0x3D` ADD: 受理态不同 (`IDLE` 与 `RECV`), 但**都不涉及**
      丢弃在途规划 —— 不满足受理条件一律回 `0x04`。
    * `0x42` GET_IK: 固件走 `kin_runner_request_ik`, 与唯一带 `cart_invalidate` 的
      `kin_runner_request_move_p` 是**两个函数** (纯读, 不会间接走到 ctrl_accept_move_j)。
    * `0x40` GET_STATUS / `0x43` GET_TCP / `0x2B`~`0x2E` 读回: 纯查询。
    """
    for cmd in (0x3C, 0x3D, 0x42, 0x40, 0x43, 0x2B, 0x2C, 0x2D, 0x2E):
        assert cmd not in _CART_CLEARS_UPON, f"0x{cmd:02X} 不作废在途规划, 不该清队"


def test_clear_set_includes_home_and_stop_commands():
    """0x2A home 经 ctrl_accept_move_j 静默丢弃; 0x06/0x11/0x12/0x13/0x14 同理。"""
    assert P.CMD_HOME == 0x2A                      # 钉住码值, 防语义漂移
    assert 0x2A in _CART_CLEARS_UPON, "0x2A home 最易漏 —— 它复用 ctrl_accept_move_j"
    for cmd in (P.CMD_ZERO_G, P.CMD_DISABLE, P.CMD_EMERGENCY_STOP,
                P.CMD_CLEAR_FAULTS, P.CMD_RESET):
        assert cmd in _CART_CLEARS_UPON, f"0x{cmd:02X} 会静默作废在途规划"


# ---------------------------------------------------------------------------
# 2. 清队挂在**最底层写口**上 —— 三处 `write_frame` 逐处验
# ---------------------------------------------------------------------------

def test_write_frame_has_single_call_site():
    """结构断言: `arm.py` 里只剩 `_raw_write` 一处 `write_frame`。

    没有这条, "三处改走 `_raw_write`"只能靠人肉数 —— 漏掉保活线程那一处时
    (它是唯一绕过 `_write_cmd` 的写者) 离线用例全绿, 真机上它每 40ms 静默作废
    一次在途规划而 SDK 毫无察觉。
    """
    import litearm.arm as arm_mod

    src = pathlib.Path(arm_mod.__file__).read_text(encoding="utf-8")

    assert src.count(".write_frame(") == 1, (
        "还有直接写 `_tr.write_frame` 的地方 —— 清队钩子拦不到它")


def test_connect_handshake_goes_through_raw_write(monkeypatch, fake_transport_factory):
    """写口 ①: `connect()` 握手 (`CMD_GET_FIRMWARE`)。"""
    import litearm.arm as arm_mod

    seen = []
    orig = arm_mod.Arm._raw_write

    def spy(self, cmd, payload=b""):
        seen.append(cmd)
        return orig(self, cmd, payload)

    monkeypatch.setattr(arm_mod.Arm, "_raw_write", spy)
    fake_transport_factory()

    arm_mod.Arm(port="fake").connect()

    assert P.CMD_GET_FIRMWARE in seen, "握手没走 _raw_write"


def test_write_query_goes_through_raw_write(monkeypatch, offline_arm):
    """写口 ②: `_write_query` —— **所有**命令的出口。"""
    arm = offline_arm
    seen = []
    orig = type(arm)._raw_write

    def spy(self, cmd, payload=b""):
        seen.append(cmd)
        return orig(self, cmd, payload)

    monkeypatch.setattr(type(arm), "_raw_write", spy)
    arm._write_query(P.CMD_CLEAR_FAULTS)

    assert seen == [P.CMD_CLEAR_FAULTS]


def test_keepalive_thread_also_clears(offline_arm, monkeypatch):
    """写口 ③: **零重力保活线程** —— 保活线程走 `_raw_write`, 所以它的 0x06 也会清队。

    这一条必须单独钉: 保活线程**刻意**绕过 `_write_cmd` (避开零重力守卫), 清队钩子
    若挂在 `_write_cmd`/`_write_query` 上就拦不到它 —— 而 `0x06` 恰恰是清队集合成员。

    判据用"**清队发生在哪个线程**"来钉 (线程名), 不用时序推断: 时序推断在
    "注册后马上断言队列非空"那一步就有竞态 (线程可能恰好抢先清掉)。
    """
    arm = offline_arm
    threads = []
    orig_clear = _CartPending.clear_pending

    def spy(self, reason=""):
        threads.append(threading.current_thread().name)
        return orig_clear(self, reason)

    monkeypatch.setattr(_CartPending, "clear_pending", spy)

    arm.zero_g_start(period=0.01)
    try:
        threads.clear()                          # 丢掉 zero_g_start 自己在主线程发的那条
        arm._cart.register()
        assert arm._cart.pending == 1

        deadline = time.monotonic() + 2.0
        while not threads and time.monotonic() < deadline:
            time.sleep(0.005)

        assert threads, "保活线程一拍都没发 —— 用例没测到要测的东西"
        assert threads[0] == _ZG_THREAD_NAME, (
            f"清队发生在 '{threads[0]}' 线程 —— 保活线程的 0x06 绕过了清队")
        assert arm._cart.pending == 0, "保活线程写的 0x06 没清队"
    finally:
        arm.zero_g_stop()


def test_clearing_happens_before_the_write(offline_arm, monkeypatch):
    """**发出前清, 不看应答** —— 写这一瞬间队列必须已经空了。

    固件的 `cart_invalidate_before_motion()` 是 `ctrl_accept_move_j` 的**第一条语句**,
    位于**全部门禁之前**: 被 `ERR 0x03/0x04/0x06` 拒掉的命令**同样已经**作废了在途
    规划。写成"收到 ACK 才清队"就会在这条被拒路径上留下陈旧 token。
    """
    arm = offline_arm
    at_write = []
    tr = arm._tr
    orig = tr.write_frame

    def spy(cmd, payload=b""):
        at_write.append(arm._cart.pending)
        return orig(cmd, payload)

    monkeypatch.setattr(tr, "write_frame", spy)
    arm._cart.register()

    arm._raw_write(P.CMD_EMERGENCY_STOP, b"")

    assert at_write == [0], "清队必须发生在发出**之前**"


def test_rejected_command_still_clears(offline_arm):
    """被固件 ERR 拒掉的清队命令**照样**已经作废在途规划 —— 判据不是应答。"""
    arm = offline_arm
    arm._tr.err_override[P.CMD_CLEAR_FAULTS] = 0x04
    tok = arm._cart.register()

    with pytest.raises(CommandRejectedError):
        arm._cmd(P.CMD_CLEAR_FAULTS, b"", "clear_faults")

    assert arm._cart.pending == 0
    assert tok.lost is not None, "token 必须被标成『结局未知』, 不能被静默留着"


def test_non_clearing_command_leaves_the_queue_alone(offline_arm):
    """不在表里的命令**一条都不许清** —— 否则正常的查询也会作废在途规划。"""
    arm = offline_arm
    tok = arm._cart.register()

    arm._raw_write(0x3A, b"\x00" * 7)             # move_l: 固件侧欠一条 0x4E
    arm._raw_write(P.CMD_GET_STATUS)

    assert arm._cart.pending == 1
    assert tok.lost is None


# ---------------------------------------------------------------------------
# 3. 队列 + 收集器 (登记 / FIFO 配对 / 两条守卫)
# ---------------------------------------------------------------------------

def test_request_registers_before_the_write(offline_arm, monkeypatch):
    """登记与写在**同一个 try 内**, 且登记在前 —— 否则应答会落在空队列上。"""
    arm = offline_arm
    at_write = []
    tr = arm._tr
    orig = tr.write_frame

    def spy(cmd, payload=b""):
        at_write.append(arm._cart.pending)
        return orig(cmd, payload)

    monkeypatch.setattr(tr, "write_frame", spy)

    tok = arm._cart.request(lambda: arm._raw_write(0x3A, b"\x00" * 7))

    assert at_write == [1], "写的时候 token 必须已经在队里"
    assert arm._cart.pending == 1
    assert tok.lost is None


def test_write_failure_drops_its_own_token(offline_arm, monkeypatch):
    """写失败 (**仅** `TransportError`) **立即摘除自己那个 token** 并原样抛出。

    ⚠ 口径**只在 `TransportError` 上**: 其余异常证明不了帧没送达, 摘了就是假成功 ——
    见 `test_interrupt_after_the_frame_is_written_leaves_its_token_in_the_queue`。

    ⚠ 这一步的正当性来自**载重不变量"抛 ⟹ 整帧未送达"**(见
    `test_flush_failure_is_not_reported_as_a_write_failure` 与
    `_CartPending.request` 的 docstring): 固件从没收到这条请求 ⟹ 永远不欠它应答 ⟹
    摘掉它才恰好让在途条数与应答条数相等。
    **不是**"滞留比报错更危险" —— 正相反: 滞留只会让末尾那条活 token 报"结局未知",
    而**丢弃一条其实已送达的** token 才会把固件的应答错配给下一条 = 假成功。
    """
    arm = offline_arm

    def boom(cmd, payload=b""):
        raise TransportError("桩: 写失败")

    monkeypatch.setattr(arm._tr, "write_frame", boom)

    with pytest.raises(TransportError):
        arm._cart.request(lambda: arm._raw_write(0x3A, b"\x00" * 7))

    assert arm._cart.pending == 0


def _stub_serial(monkeypatch):
    """注入只服务 `write_frame` 的桩串口; 返回 `holder["ser"]` = 建出来的那一个。"""
    holder = {}

    class _WStubSerial:
        """`write` / `flush` 各自可注入异常 —— 这两半的**失败语义完全不同**。"""

        def __init__(self, port, baudrate=0, timeout=0.2, **kw):
            self.is_open = True
            self.written = bytearray()
            self.write_exc = None
            self.flush_exc = None
            holder["ser"] = self

        def write(self, d):
            if self.write_exc is not None:
                raise self.write_exc
            self.written += d
            return len(d)

        def flush(self):
            if self.flush_exc is not None:
                raise self.flush_exc

        def close(self):
            self.is_open = False

    monkeypatch.setitem(sys.modules, "serial", types.SimpleNamespace(Serial=_WStubSerial))
    return holder


def test_flush_failure_is_not_reported_as_a_write_failure(monkeypatch):
    """⚠ 配对器的**载重不变量**: "抛 `TransportError` ⟹ 整帧未送达"。

    `_CartPending.request()` 靠"写失败 (**仅** `TransportError`) 就摘掉自己那个 token"
    成立, 而这条**只在 "抛错 ⟹ 帧没出去" 成立时才对**: 帧其实送到了却把 token 摘掉,
    固件那条 `0x4E` 就会配给队列里**后面那条活 token** —— 报的是"成功"而实际没跑,
    即**假成功** (本项目最忌讳的方向)。

    两半的物理事实不同, 故**必须拆开**:

    * `write()` 抛 → 帧没送达 (残缺帧被固件按 CRC 丢) ⇒ 照旧 `TransportError`;
    * `flush()` (tcdrain) 抛 → **整帧已经在驱动里, 会被送达**, 失败也收不回来
      ⇒ **不报成发送失败**, 只把这次失败记进可观测计数 `flush_failures`
      (静默是另一条底线)。真链路故障会在**下一次写**上暴露。
    """
    holder = _stub_serial(monkeypatch)
    from litearm.transport import SerialTransport

    tr = SerialTransport("/dev/null")
    ser = holder["ser"]

    ser.flush_exc = OSError("桩: tcdrain 失败 (帧已在驱动缓冲里)")
    tr.write_frame(0x3A, b"\x00" * 7)              # ← 不许抛

    assert tr.flush_failures == 1, "flush 失败必须留下可观测计数, 不能静默"
    assert P.unpack_frame(bytes(ser.written)) == (0x3A, b"\x00" * 7), (
        "这一半的前提是'整帧已交给驱动' —— 帧没出去的话另一支才成立")

    ser.write_exc = OSError("桩: 写失败")
    with pytest.raises(TransportError):
        tr.write_frame(0x3A, b"\x00" * 7)          # ← 这半不能退化
    assert tr.flush_failures == 1, "write 失败不该计进 flush_failures"

    #: ⚠ 收尾**必须显式 close()**, 不能指望"函数返回就释放了": 上面那条 `pytest.raises`
    #: 造出 `帧 -> traceback -> 帧` 的环, 把 `tr` 钉到一次全量 gc 之后 (与 `test_transport.py`
    #: "端点独占"一节的注释同源)。而 `tr` 到此**从没关过** ⇒ 端口 `/dev/null` 会**按 gc 时机**
    #: 继续被登记表认作占用 —— 本文件下一条用例正好也用 `/dev/null`, 于是"红不红"成了看时机的事。
    #: `close()` 落在那句 `_release_port` 上, 释放与 gc 何时跑无关 (判据见 `transport._claim_port`)。
    tr.close()


def test_interrupt_after_the_frame_is_written_leaves_its_token_in_the_queue(monkeypatch):
    """⚠ **摘 token 只能发生在 `TransportError` 上** —— 其余异常**证明不了**帧没送达。

    最现实的那一支 (本用例复现的): `KeyboardInterrupt` 落在 `flush()` 里 —— 那一刻
    `write()` **已经返回**, 整帧**已交进驱动、会被送达** (与"`write()` 抛"那支的物理
    事实**相反**, 见上面两条用例)。固件于是真会受理并回一条 `0x4E`; 此时若按"没送达"
    把自己那个 token 摘掉, 那条应答就配给队列里**后面那条活 token** ⇒ 报"成功"而实际
    没跑 = **假成功** (本项目最忌讳的方向)。

    故除 `TransportError` 外一律 re-raise 且**不动队列**。代价是可能滞留一条其实没送达
    的 token, 而那只会让队列**末尾**那条活 token 报"结局未知" (`CartReplyLostError`)
    —— **安全方向**, 这才是两个方向里该选的那一个。
    """
    holder = _stub_serial(monkeypatch)
    from litearm.transport import SerialTransport

    tr = SerialTransport("/dev/null")
    ser = holder["ser"]
    ser.flush_exc = KeyboardInterrupt()            # ← 落在 write() 返回之后
    q = _CartPending(absorb_ttl=1.0)

    with pytest.raises(KeyboardInterrupt):         # ← 异常照常传播
        q.request(lambda: tr.write_frame(0x3A, b"\x00" * 7))

    assert P.unpack_frame(bytes(ser.written)) == (0x3A, b"\x00" * 7), (
        "这一支的前提正是'整帧已交给驱动' —— 帧没出去的话摘 token 才成立")
    assert q.pending == 1, (
        "其余异常下**不许动队列**: 摘掉它会让固件随后的 0x4E 配给后面的活 token = 假成功")

    #: 同 `test_flush_failure_is_not_reported_as_a_write_failure` 的收尾: 本用例也用
    #: `/dev/null`, 开着就走会把端口名留给下一条同端口的用例 (释放时机取决于 gc)。
    tr.close()


def test_drop_removes_its_own_token_not_the_tail(offline_arm):
    """摘除必须按**对象身份**, 不能 pop 末位 —— 并发下末位是别人的 token。

    行为证明: 三条 token 里摘掉中间那条, 两条应答必须归 a 与 c (b 一条都不吃)。
    """
    arm = offline_arm
    a = arm._cart.register()
    b = arm._cart.register()
    c = arm._cart.register()

    arm._cart.drop(b)
    _push(arm, 0x4E, b"first")
    _push(arm, 0x4E, b"second")

    _settle(arm)
    _settle(arm)

    assert a.reply == b"first"
    assert c.reply == b"second"
    assert b.reply is None, "被摘掉的 token 不该吃到任何一条应答"
    assert arm._cart.pending == 0
    assert arm._cart.extra_replies == 0


def test_0x4e_reply_is_claimed_by_the_collector(offline_arm):
    """`0x4E` 由读线程交给收集器: **不返回给调用方**、也不进队列（更不会被计成丢弃）。"""
    arm = offline_arm
    tok = arm._cart.register()
    _push(arm, 0x4E, b"\x01\x00\x00\x00")

    _settle(arm)
    assert tok.reply == b"\x01\x00\x00\x00"
    assert arm._cart.pending == 0
    assert arm._a.dropped == 0, "收集器认领的帧不该被计成丢弃"


def test_wait_returns_the_reply(offline_arm):
    """配对成功: `wait()` 交回原始载荷, 队列复原。"""
    arm = offline_arm
    tok = arm._cart.register()
    _push(arm, 0x4E, b"ok")

    _settle(arm)

    assert arm._cart.wait(tok, 0.05) == b"ok"
    assert arm._cart.pending == 0


def test_wait_can_drive_the_reads_itself(offline_arm):
    """`pump` 给出时由 `wait()` 自己驱动取帧 (本包**没有读线程**, 决策 12)。"""
    arm = offline_arm
    tok = arm._cart.request(lambda: arm._raw_write(0x3A, b"\x00" * 7))
    _push(arm, 0x4E, b"driven")

    assert arm._cart.wait(tok, 0.5) == b"driven"


def test_a_result_delivered_by_wait_is_not_delivered_again_by_poll_cart(offline_arm):
    """`wait` 成功交付之后, 那条结果**不许**再被 `poll_cart` 认领一次。

    两条路都指向同一句 "结果已被取走": `wait` 结尾的 `self._unclaim(token)` 与
    `poll_cart` 的 `claim_resolved`。少了前者, 同一条结果被交付两次 —— 调用方会把它
    当成**两条不同的规划** (而它其实只有一条)。

    判据能失败: 删掉 `wait` 末尾那句 `self._unclaim(token)`, 下面 `poll_cart` 会返回
    一个 `CartPlan` (而不是 `None`)。
    """
    arm = offline_arm
    tok = arm._cart.register()
    arm._tr._resp.clear()                       # 让 pump 一定拿到下面这条 0x4E
    arm._tr.push_frame(P.RSP_CART_PLAN, _plan_payload())

    assert arm._cart.wait(tok, 0.5) == _plan_payload()
    assert arm._cart.pending == 0
    assert arm.poll_cart() is None, (
        "同一条结果先被 wait 交付、又被 poll_cart 认领了一次 (重复交付)")


def test_the_unclaimed_column_is_filled_before_the_waiter_is_woken(offline_arm,
                                                                  monkeypatch):
    """⚠ 钉住 `on_reply` 里"**先 append 再 resolve**"这个顺序 (反序必须变红)。

    反序不是风格问题: `resolve()` 一置位, 另一个线程里正 `wait` 的等待者就会醒来, 而它
    醒来后做的第一件事就是 `_unclaim` —— 反序时那次摘除**扑空**, 随后 append 才发生,
    于是同一条结果永久留在那一列, `poll_cart` 会**重复交付**它 (`wait` 的 `_unclaim` 是
    在 `resolve` **之后**跑的, 补不了这个洞)。

    判据 (确定性, 不靠抢跑): 把"等待者醒来后做的第一件事" (`_unclaim`) 与"那一刻 token
    在不在列里"一起挂在 `resolve` 上 —— 这正是坏交错里等待者所处的位置与时机。
    先 append 的实现: 交付那一刻它在列里, 摘得掉 (列空) ⇒ 两条断言都绿;
    反序的实现: 摘除扑空、随后 append 把它留下 ⇒ **第一条断言即红**。
    """
    arm = offline_arm
    tok = arm._cart.register()
    seen = []
    real_resolve = cart_mod._CartToken.resolve

    def resolve_and_let_the_waiter_wake(self, payload):
        if self is tok:
            # 置位**之前**: 等待者被唤醒后要摘的正是这一列
            seen.append(any(t is tok for t in arm._cart._unclaimed))
        real_resolve(self, payload)
        if self is tok:
            arm._cart._unclaim(self)      # 等待者醒来后的第一件事 (= 那边 `wait` 做的)

    monkeypatch.setattr(cart_mod._CartToken, "resolve",
                        resolve_and_let_the_waiter_wake)
    _push(arm, 0x4E, _plan_payload())
    _settle(arm)

    assert seen == [True], (
        "resolve 置位的那一刻 token **还不在** `_unclaimed` 里 —— 等待者醒来后那次摘除会"
        "扑空, 随后 append 把它留成一条永远不会被取走的结果 (poll_cart 会重复交付)")
    assert arm.poll_cart() is None, (
        "结果已被等待者取走, poll_cart 却又认领到一次 (同上, 顺序反了)")
    assert arm._cart.wait(tok, 0.05) == _plan_payload()   # wait 自身仍能交付


def test_claim_resolved_ignores_a_token_that_has_not_settled_yet():
    """⚠ `claim_resolved` 的判据必须含 `_done.is_set()`, 不能只看"那一列非空"。

    `on_reply` 是**先登记进那一列、后 `resolve`** (顺序承重, 见
    `test_the_unclaimed_column_is_filled_before_the_waiter_is_woken`), 两条语句之间有一条
    缝 —— 并发读者恰好落进去, 拿到的是一个**还没收尾**的 token, 于是
    `CartPlan.from_reply(None)` 当场 `TypeError` (单线程不可达, 故此前没暴露)。
    缝里返回 `None` 才是对的语义 ("此刻没有可取的结果"), `poll_cart` 本来就以 `None`
    表示它。

    判据: 手工造出"已登记、未收尾"那一态 (就是缝里的观测面), 断言认领不到; `resolve`
    之后才认得动。
    """
    cp = _CartPending(absorb_ttl=15.0)
    tok = cp.register()
    with cp._lock:
        cp._unclaimed.append(tok)          # 缝: `on_reply` 已登记, 尚未 resolve

    assert cp.claim_resolved() is None, (
        "认领到了一个还没 resolve 的 token —— `CartPlan.from_reply(None)` 会 TypeError")

    tok.resolve(_plan_payload())
    assert cp.claim_resolved() is tok, "收尾之后反而认领不到了"


def test_drop_removes_the_token_from_both_columns():
    """`drop` 要摘**两列** (`_q` 与 `_unclaimed`) —— 见它的 docstring:
    "一条被判死的请求不该还能被 `poll_cart` 认领出结果"。

    ⚠ 既有用例**一次都没走到 `_unclaimed` 那一半**: 唯一的调用点
    (`_request_and_wait` 的 ACK 超时支) 一定发生在应答到达**之前**, 那时 token 只在 `_q`
    里。把 `_unclaim_locked` 里 `self._unclaimed.remove(token)` 换成 `pass` 全量套件照绿。
    这里直接把"应答已到、还没人认领"那一态造出来再摘。
    """
    cp = _CartPending(absorb_ttl=_FIXED_TTL)
    tok = cp.register()
    cp.on_reply(_plan_payload())          # 应答到了 ⇒ token 进 `_unclaimed` 并 resolve

    assert tok in cp._unclaimed, "用例自身失效: token 没进 `_unclaimed`"
    cp.drop(tok)

    assert tok not in cp._unclaimed, (
        "`drop` 只摘了 `_q` 那一列 —— 被判死的请求还能被 `poll_cart` 认领出结果 (重复交付)")
    assert cp.claim_resolved() is None, "被判死的 token 仍被 `claim_resolved` 交付了"


def test_token_timeout_raises_reply_lost_and_drops_token(offline_arm):
    """**少了一条** —— 超时 (用 `Arm.move_timeout`, 不新造字段) → 摘除 token 并抛
    `CartReplyLostError`。缺了这条, 后续的 `0x4E` 会被错配给一条早已被吞掉的请求
    (固件单槽 pending: 同一 main 排空窗口内登记 ≥3 条时中段应答会被吞掉, N≥3 只存活 2 条)。"""
    arm = offline_arm
    tok = arm._cart.register()

    with pytest.raises(CartReplyLostError) as ei:
        arm._cart.wait(tok, 0.05)

    assert arm._cart.pending == 0, "超时的 token 必须摘除, 否则下一个 0x4E 会错配"
    assert "超时" in str(ei.value)


def test_queue_empty_reply_is_counted_and_raises(offline_arm):
    """**多了一条** —— 队列空却收到 `0x4E`: 计数 + 报错, **不静默丢弃**。

    内部一致性错误 (应答比请求多 ⟹ 固件与主机已经错位), 故**不是**
    `CartReplyLostError` (那是"少了一条"的未知结局)。

    ⚠ 判据: "**少一条**"是调用方面对的**处境** (结局未知, 要能与失败/接管区分)
    ⇒ 独立类型、可被 `except`; "**多一条**"是 SDK 配对模型与固件**脱同步**的
    内部不变量破坏 ⇒ 抛基类 `LiteArmError`, **不该**被专门 catch。本条钉住这一点
    (断言不是 `CartReplyLostError`) —— 别顺手给它补一个专用子类。
    """
    arm = offline_arm
    _push(arm, 0x4E, b"\x01\x00\x00\x00")
    _settle(arm)                     # 读线程先把它交给收集器 (收集器在这一刻抛)

    # ⚠ 读线程**不当场抛、也不死**（`_Ack._reader_loop`）：它把异常存进 `_errors`，
    # 由**下一个等待者**抛出。所以这里要真的等一次（用驱动型读者，它现在也看 `_errors`）。
    with pytest.raises(LiteArmError) as ei:
        arm.get_status_now(timeout=0.5)

    assert not isinstance(ei.value, CartReplyLostError)
    assert "0x4E" in str(ei.value)
    assert arm._cart.extra_replies == 1


def test_clear_pending_fails_tokens_as_unknown_outcome(offline_arm):
    """清队 = 那几条请求**永远等不到应答**了, 必须标成"未知结局"并唤醒等待者 ——
    否则调用方会白等到超时, 而臂可能已经朝着旧目标动了。"""
    arm = offline_arm
    tok = arm._cart.register()

    assert arm._cart.clear_pending("测试清队") == 1
    assert arm._cart.pending == 0
    assert tok.lost is not None

    with pytest.raises(CartReplyLostError) as ei:
        arm._cart.wait(tok, 0.05)

    assert "测试清队" in str(ei.value)


def test_clear_pending_on_empty_queue_is_a_noop(offline_arm):
    """空队清队不该报错也不该计数 (保活线程每 40ms 清一次, 大多数时候队是空的)。"""
    arm = offline_arm

    assert arm._cart.clear_pending() == 0
    assert arm._cart.extra_replies == 0


def test_extra_replies_counter_is_independent_from_the_dropped_counter(offline_arm):
    """两个计数**各记各的**: `0x4E` 由收集器认领 (不计丢弃), 没人认领的 id 只进队列。

    ⚠ 重构后 `unexpected_frames` / `foreign_frames` 合并成 `_Ack.dropped`（只在**队列封顶**
    时递增）；一条没人认领的未知帧**进队列等着**，并不立刻计丢弃 —— 那是设计（"不丢"）。
    """
    arm = offline_arm
    _push(arm, 0x7F)                               # 没人认领的上行 id

    arm.get_status_now(timeout=0.5)

    assert arm._a._queues.get((0x7F, None)), "未知 id 应当进队列等着, 而不是被丢掉"
    assert arm._a.dropped == 0
    assert arm._cart.extra_replies == 0


# ---------------------------------------------------------------------------
# 4. 生命周期 —— 重连 = 新会话 = 新配对状态
# ---------------------------------------------------------------------------

def test_reconnect_drops_stale_tokens(offline_arm):
    """`close()` → `connect()` 后**陈旧 token 必须消失** —— 否则 off-by-one 传递。

    配对状态与链路同寿: 陈旧 token 留着, 下一条**合法** `0x4E` 会配给那条早就没人
    认领的死 token, 于是活 token 一条应答都收不到、一路走到超时
    —— 这正是 FIFO 配对存在的意义被反噬的形态。

    ⚠ 判据必须能真的失败: 只断言"队列空"的话, 实现若改成"重连时把旧 token 一个个
    `drop` 掉"也会绿, 而那并不能证明陈旧者不再吃应答。故这里**同时**断言"陈旧 token
    一条都吃不到"。

    ⚠⚠ **重连时会继承一格吸收额度** (见
    `test_reconnect_inherits_the_in_flight_count_as_absorb_credit`), 于是**第一条**到
    的 `0x4E` 是"旧会话那条"还是"新 token 自己那条"**在原理上无法区分** (载荷里没有
    命令 id) —— 本用例把两只都摆出来: 第一条被**吸收**, 第二条 (新会话自己那条) 才
    配上 `fresh`。取舍是知情的: 反过来 (不继承额度) 会让旧会话那条迟到应答配给
    **新** token = **假成功**, 而本模块的方向一贯是"宁可报结局未知"。
    """
    arm = offline_arm
    stale = arm._cart.register()

    arm.close()
    arm.connect()                                  # 新会话

    assert arm._cart.pending == 0, "重连后陈旧 token 还留在队里"

    fresh = arm._cart.register()
    _push(arm, 0x4E, b"stale")                     # 旧会话那条迟到应答

    _settle(arm)
    assert arm._cart.absorbed_replies == 1, (
        "旧会话那条迟到应答既没被吸收、也没配对 —— 它去哪了?")
    assert fresh.reply is None and stale.reply is None, (
        "陈旧 token 吃到了应答 (off-by-one 错配)")

    _push(arm, 0x4E, b"fresh")                     # 新会话**自己**那条
    _settle(arm)
    assert fresh.reply == b"fresh", "新 token 没配到它自己那条应答"
    assert stale.reply is None, "陈旧 token 吃到了新会话的应答 (off-by-one 错配)"
    assert arm._cart.extra_replies == 0


# ---------------------------------------------------------------------------
# 5. 清队吸收额度 (Task 3.5) —— 清队销毁了归属信息, 只能按"判错得安全"那一支收场
# ---------------------------------------------------------------------------

class _FakeClock:
    """确定性时间源 —— 由用例 `monkeypatch.setattr(cart_mod, "time", clock)` 装上去。

    `cart.py` 只从 `time` 用 `monotonic()` 一个方法, 所以这个假对象就够。
    ⚠ **不用真实 `sleep` 卡过期边界**: 那种断言在负载下必假 (边界附近的 flaky
    用例比没有更坏), 而把 `time.monotonic` 打补丁会连带影响 `threading`。
    """

    def __init__(self, t: float = 1000.0) -> None:
        self.t = t

    def monotonic(self) -> float:
        return self.t


def test_late_reply_for_a_cleared_request_is_absorbed_not_raised(offline_arm):
    """清队后迟到的那条 `0x4E` (固件**已经跑完**、应答早躺在 RX 缓冲里): 进
    `absorbed_replies`, **不报错**, `extra_replies` 仍为 0。

    按"多了一条"无条件报错的话, `LiteArmError` 会从毫不相干的读路径里炸出来, 并把一条
    **已经成功**的运动报成"结局未知" —— 触发条件只是"清队 opcode 紧跟在笛卡尔命令之后、
    中间没有读帧", 而 abort / 急停 / 收尾路径天生就是这个形状。
    """
    arm = offline_arm
    tok = arm._cart.register()
    arm._cart.clear_pending("测试清队")

    assert tok.lost is not None
    assert arm._cart.absorbed_replies == 0

    _push(arm, 0x4E, b"\x01\x00\x00\x00")          # 迟到的、不可归属的那条应答

    _settle(arm)     # "不返回给调用方" 现在是**结构事实**: 没有调用方取帧这一说了
    assert arm._cart.absorbed_replies == 1
    assert arm._cart.extra_replies == 0, (
        "清队后的不可归属应答被算成了真·脱同步 —— 正常收尾被炸成硬异常")
    assert arm._cart.pending == 0


def test_an_absorb_during_the_wait_window_suppresses_the_fresh_credit():
    """⚠ `drop_and_absorb` 的**例外**: 本条请求的窗口里吸收过一条 ⇒ **不补**额度。

    理据 (见那个函数的 docstring): 吸收**先于**配对 ⇒ 一条在途 token 的应答到了、而额度
    还有存量时, 被吃掉的就是**它自己那条** —— 固件不再欠它什么。此时若照旧补一格, 补出来
    的那格会去吃**下一条**命令的应答, 那条又超时、又补一格 … **自持**
    (入口: `connect()` 继承来的那一格; 端到端用例见
    `test_an_inherited_credit_cannot_perpetuate_itself_across_a_reconnect`)。

    判据分两半, **缺一不可** (只验前半会让"干脆永远不补"也绿 —— 那会退化 E3 的守卫):
      · 窗口里吸收过 ⇒ `_absorb` 仍是 0;
      · 没吸收过 ⇒ 照旧 1 (既有行为**不变**)。

    ⚠ 两条 `drop_and_absorb` 都传 `timed_out=True` (= **超时支**) —— 例外**只**对这一支
    生效; pump (读链路炸) 支走例外会翻成**假成功**, 由
    `test_a_pump_failure_must_not_suppress_the_credit` 单独钉住。
    """
    cp = _CartPending(absorb_ttl=15.0)

    absorbed = cp.register()
    assert cp.clear_pending("清队") == 1        # 时长队: 那份额度是**挣来的**(与继承同形)
    cp.on_reply(b"late")                        # 窗口里被消费掉 ⇒ "已经交出来了"
    assert cp.absorbed_replies == 1
    cp.drop_and_absorb(absorbed, timed_out=True)
    assert cp._absorb == 0, (
        "窗口里已经吸收过还补一格 —— 它就是自持入口: 下一条命令的应答会被这格吞掉")

    plain = cp.register()
    cp.drop_and_absorb(plain, timed_out=True)
    assert cp._absorb == 1, (
        "没吸收过也不补额度 —— 固件真欠的那条应答会撞空队列, E3 守卫退化")


def test_a_dead_reader_must_not_suppress_the_credit(offline_arm):
    """⚠⚠ **"读线程死"那一支照旧补额度** —— 例外只许挂在**超时**那一支上。

    (`pump` 参数已随读路径重构删除；旧"pump 支"= 调用方驱动取帧时炸，它的等价物是
     **读线程退出** `_Ack._reader_error` —— 两者都是"链路没了、这条请求没人再来收"。)

    `drop_and_absorb` 有**两条**调用点 (都在 `wait()` 里): 链路没那条与超时那条。
    例外段那个判据 ("窗口里吸收过 ⇒ 不补") 的理据**只对超时那一支成立** —— 它靠的是
    "窗口够长 ⟹ 固件早该把该发的都发了"; 而 pump 那一支是**读链路炸了**, 根本没等过窗口,
    于是"被吸收的那条是本 token 自己的"这个推断在那一支上**没有依据**。

    ⚠ 判错的方向是**假成功**, 不是"宁可让一条报结局未知": 被放弃的那条请求**固件真还欠
    它一条应答**, 而例外把该补的那格吞了 ⇒ 那条应答到达时队列非空 ⇒ `on_reply` 直接配给
    **下一条** token ⇒ 调用方拿到**别人那条**的规划结果并**报成功** (`0x4E` 载荷里没有
    命令 id, 它无从分辨)。

    场景 (照 `test_an_absorb_during_the_wait_window_suppresses_the_fresh_credit` 摆,
    只把"放弃"那一步换成 pump 支):
      清队(额度 1) → 登记本 token(**先于**那条旧应答到达) → 旧应答被吸收 → pump 炸 ⇒
      本 token 被放弃 → 它的**自己那条**应答到达。

    判据: 放弃之后 `_absorb == 1`, 且那条应答**不许**配给下一个 token。
    """
    # ⚠ 必须带 `arm`：旧 `pump` 支的等价物是"读线程死掉"，而那一支要读 `arm._a._reader_error`。
    cp = _CartPending(absorb_ttl=15.0, arm=offline_arm)

    stale = cp.register()                       # 清队前的那条在途记账
    assert cp.clear_pending("清队") == 1         # ⇒ 额度 1 (固件可能还欠 `stale` 一条)
    assert stale.lost is not None

    tok = cp.register()                         # 本 token: `absorbed_at` 快照 = 0
    cp.on_reply(b"stale")                       # **旧的**那条先到 ⇒ 被吸收 (不配给 `tok`)
    assert cp.absorbed_replies == 1

    # ⚠ `pump` 参数已删。旧"pump 支"（调用方驱动取帧时炸）的等价物是**读线程死掉**
    # —— 两者都是"链路没了、这条请求没人再来收"，`wait` 必须同样放弃并**原样抛出病因**。
    cp._arm._a._reader_error = TransportError("桩: 读链路炸了 (读线程支)")

    with pytest.raises(TransportError):
        cp.wait(tok, 5.0)                       # 这一支放弃 —— **没等过窗口**

    credit = cp._absorb                         # ⚠ 必须在下面那条 `on_reply` **之前**读:
                                                # 应答一到就会把这格额度消费掉

    nxt = cp.register()
    cp.on_reply(b"tok-own")                     # `tok` **自己**那条 (固件确实会发)
    mismatched = nxt.reply is not None

    assert not mismatched, (
        "`tok` 自己那条应答配给了下一个 token = **报成功而实际没跑完** (假成功) —— "
        "pump 失败支错走了例外, 该补的那格额度没补")
    assert credit == 1, (
        "pump 失败支也没补额度 —— 例外只许挂在超时那一支上 (那一支才有"
        "'窗口够长'的理据); 补不上就没有东西替这条迟到的应答兜底")


def test_absorb_happens_before_pairing_even_with_a_live_token_queued(offline_arm):
    """⚠ 本组最关键的一条 —— 防**假成功**。

    清队(T1) → 立刻登记(T2, 队列**非空**) → T1 的迟到应答到达。吸收若只挂在"队列空"
    那个分支上, 这条应答会被配给 **T2** —— 即**报成功而实际没跑完**。

    判据: `absorbed_replies == 1` **且 T2 仍未收尾** (还留在队里等它自己那条)。
    """
    arm = offline_arm
    t1 = arm._cart.register()
    arm._cart.clear_pending("T1 清队")
    t2 = arm._cart.register()                       # 清队后立刻登记 → 队列非空

    assert arm._cart.pending == 1
    assert t1.lost is not None

    _push(arm, 0x4E, b"late-T1")
    _settle(arm)

    assert arm._cart.absorbed_replies == 1, "队列非空时吸收没生效"
    assert t2.reply is None, (
        "T2 配到了 T1 的迟到应答 = 假成功 (T2 实际还没跑完)")
    assert t2.lost is None, "T2 既没配对也没被作废 —— 它应当还在等自己的应答"
    assert arm._cart.pending == 1
    assert arm._cart.extra_replies == 0


def test_absorbance_expires_after_move_timeout(monkeypatch):
    """额度过期后, 队列空再收应答仍**计数并报错** —— 真·脱同步不能被吞。

    "多了一条"这条守卫的语义没有变, 只是被吸收额度**让路**了一段有限的时间。
    """
    clock = _FakeClock()
    monkeypatch.setattr(cart_mod, "time", clock)

    fresh = _CartPending(absorb_ttl=15.0)
    fresh.register()
    assert fresh.clear_pending("测试清队") == 1     # 额度 1, 截止 = 清队时刻(1000) + 15
    clock.t = 1014.5
    fresh.on_reply(b"within-ttl")
    assert fresh.absorbed_replies == 1, "没过期的额度居然不吸收"

    stale = _CartPending(absorb_ttl=15.0)
    stale.register()
    assert stale.clear_pending("测试清队") == 1     # 额度 1, 截止 = 清队时刻(1014.5) + 15
    clock.t = 1029.501                             # 越过截止 (`now > 截止` 才过期)
    with pytest.raises(LiteArmError):
        stale.on_reply(b"after-ttl")
    assert stale.extra_replies == 1, "过期的额度还在吞应答 —— 硬错误降级成静默"
    assert stale.absorbed_replies == 0

    # ⚠ 边界: "now **恰好** == 截止"算**未**过期 (实现取 `now > 截止`) —— 规格没写这
    # 一侧, 是实现的刻意选择, 故在这里钉住: 改成 `>=` 本断言必须变红。
    edge = _CartPending(absorb_ttl=15.0)
    clock.t = 2000.0
    edge.register()
    assert edge.clear_pending("边界清队") == 1       # 额度 1, 截止 = 2000 + 15 = 2015
    clock.t = 2015.0                                # **恰好**落在截止上
    edge.on_reply(b"exactly-at-deadline")
    assert edge.absorbed_replies == 1, (
        "\"now == 截止\"被判成了过期 —— 实现取 `now > 截止` (相等仍未过期)")
    assert edge.extra_replies == 0


def test_allowance_accumulates_across_clears():
    """清两次 (各清掉一条) ⇒ 能吸收**两条**。

    写成"重置为本次条数"的话, 前一次那条待吸收的迟到应答会退化成硬错误 —— 而清队与
    迟到应答的到达顺序本来就无法保证 (两条清队之间那条应答可能还没被读到)。
    """
    cp = _CartPending(absorb_ttl=15.0)
    cp.register()
    assert cp.clear_pending("第一次") == 1
    cp.register()
    assert cp.clear_pending("第二次") == 1          # 累加, 不是重置

    cp.on_reply(b"late-first")
    cp.on_reply(b"late-second")

    assert cp.absorbed_replies == 2
    assert cp.extra_replies == 0


def test_clear_of_an_empty_queue_does_not_refresh_the_deadline(monkeypatch):
    """⚠ 空清队**不许**刷新截止时间 —— 否则额度永不过期。

    零重力保活线程每 40ms 发一条 `0x06` (它在 `_CART_CLEARS_UPON` 里), 那几乎永远是
    "空清队"。若 `n == 0` 也把截止推到 `now + ttl`, 额度就永远活着 —— 此后一条**真**
    脱同步的应答会被永久静默吞掉, 硬错误降级成静默。
    """
    clock = _FakeClock()
    monkeypatch.setattr(cart_mod, "time", clock)

    cp = _CartPending(absorb_ttl=15.0)
    cp.register()
    assert cp.clear_pending("T1 清队") == 1         # 额度 1, 截止 = 1000 + 15 = 1015

    clock.t = 1010.0
    for _ in range(5):                              # 保活线程式的空清队 (队里没有 token)
        assert cp.clear_pending("保活 0x06") == 0
        clock.t += 1.0                              # 1010 → 1015

    clock.t = 1016.0                                # 越过最初那条截止 (1015)
    with pytest.raises(LiteArmError):
        cp.on_reply(b"desync")
    assert cp.extra_replies == 1
    assert cp.absorbed_replies == 0, (
        "空清队刷新了截止 → 额度永不过期, 真脱同步被永久静默")


def test_both_cart_pending_construction_sites_take_move_timeout(monkeypatch,
                                                               fake_transport_factory):
    """两个构造点 (`__init__` 与 `connect()`) **都**得按 `move_timeout` 现算 TTL,
    且**都不许低于** `_ABSORB_TTL_FLOOR`。

    ⚠ 只验一个点会漏掉另一个: `connect()` 会**重建** `_CartPending`, 那一处若写死默认
    值, 真机 (总是先 `connect()`) 走的就是**没接线**的那个 —— 于是要么 `move_timeout`
    调小不生效, 要么额度永不过期。

    ⚠ 期望值是 `max(move_timeout, _ABSORB_TTL_FLOOR)` **不是** `move_timeout`:
    额度过期而那条应答还在路上时, 它会被 `on_reply` 配给**新登记**的那条 token ⇒
    调用方拿到**别人那条**的规划结果并**报成功** (见
    `test_a_reply_that_outlives_move_timeout_is_not_paired_to_the_next_request`)。
    故 TTL 有一个**固件依据的下限** (三段推导见 `_ABSORB_TTL_FLOOR` 的定义 —— 别在这里
    复述那个算式: 复述出来的第二份就是会漂移的那一份);
    这半条判据由"取一个**远小于**下限的 `move_timeout`, 断言 TTL **没有**跟着变小"
    钉住 —— 只断言"等于 `max(...)"的话, 实现若把下限整个删掉、改成恒等于 `move_timeout`
    照样绿。

    ⚠ 两半的边界都要**相对清队时刻**取: 写成"绝对时刻 + 硬编码边界"的话, 把 TTL 改成
    0.5 也照样绿 —— 只验得出"太大"、验不出"太小" (那正是漏接线的形态)。
    """
    clock = _FakeClock()
    monkeypatch.setattr(cart_mod, "time", clock)
    fake_transport_factory()
    from litearm import Arm

    MV_TIMEOUT = 2.0                     # < 下限 ⇒ 走"下限兜底"那一支
    MV_TIMEOUT_BIG = 20.0                # > 下限 ⇒ 必须整段跟随 `move_timeout`
    FLOOR = cart_mod._ABSORB_TTL_FLOOR

    def _probe(cart, expected: float) -> None:
        """以当前时刻清队, 再验"TTL 内吸收 / 越过 TTL 报错" —— 两半都相对清队时刻取边界。"""
        ttl = cart._absorb_ttl                      # 接线成果本身, 不是期望值
        assert ttl == expected, (
            f"TTL 拿到 {ttl}, 应为 {expected} "
            f"(写死/漏传/漏掉下限都会在这里露出来)")

        base = clock.t
        cart.register()
        assert cart.clear_pending("清队") == 1       # 截止 = base + ttl

        clock.t = base + ttl - 1e-3                 # 差一点才到截止 —— 必须吸收
        cart.on_reply(b"within-ttl")
        assert cart.absorbed_replies == 1, "没接上额度 (额度根本没生效)"

        base = clock.t
        cart.register()
        assert cart.clear_pending("清队") == 1       # 截止 = base + ttl
        clock.t = base + ttl + 1e-3                 # 刚过截止
        with pytest.raises(LiteArmError):
            cart.on_reply(b"over-ttl")
        assert cart.extra_replies == 1, "TTL 太长 (真脱同步被吞)"

    arm = Arm(port="fake", move_timeout=MV_TIMEOUT)  # 构造点 ①: `__init__`
    _probe(arm._cart, max(MV_TIMEOUT, FLOOR))
    assert arm._cart._absorb_ttl > MV_TIMEOUT, (
        "TTL 跟着小 move_timeout 一起变小了 —— 额度过期早于固件那条应答的到达时刻 "
        "= 陈旧 0x4E 会配给新 token (假成功)")

    cart_before = arm._cart
    # ⚠ 走 `reconnect()` 而不是 `connect()`: `connect()` 现在是**幂等**的 (已连着同一
    # 目标时是 no-op, 不重建会话), 而本行要的正是"会话重建"这个动作本身。
    arm.reconnect()                                 # 构造点 ②: `connect()` 重建
    assert arm._cart is not cart_before, (
        "会话重建没有重建 `_CartPending` —— 旧对象的累积计数会被下半个 probe 继承, "
        "于是失败会表现成'额度没生效'(断言 `absorbed_replies == 1` 上) 而非本行; "
        "先钉住对象身份, 失败消息才指向真实死因")
    _probe(arm._cart, max(MV_TIMEOUT, FLOOR))

    # ⚠ `max` 的另一半: 大 `move_timeout` 那支**不许**被下限砍短 (方向是"只增不减")
    arm.move_timeout = MV_TIMEOUT_BIG
    arm.reconnect()
    _probe(arm._cart, MV_TIMEOUT_BIG)


def test_the_absorb_ttl_floor_is_the_firmware_planning_bound_plus_margin():
    """吸收额度的存活下限**不是拍脑袋的常数** —— 它有固件依据, 写在常量的定义里。

    ⚠ 判据要覆盖的是**到达**时刻, 三段相加 (逐段的 `文件:行号` 见 `_ABSORB_TTL_FLOOR`):

    ① **生成** ≤ 写帧 + **3 s**: `RSP_CART_PLAN` 的唯一发射点是 `cart_report_if_done()`
       (`usb_cmd.c:1246`, `usb_cmd_reply(RSP_CART_PLAN,…)` 在 **`:1261`**), 由 main 循环
       每圈调 (`Core/Src/main.c:187`) —— **不在** `cart_publish()` 里。规划期硬上界
       `CART_PLAN_MAX_TICKS` = `LITEARM_CTRL_HZ * 3u` = 3 s (`cart_exec.h:42`), 到点
       `cart_tick_timeout` 会 `cart_abort()` (`.c:890`) 并补发终态应答 (`.c:917`)。
    ② **发射时延** ≤ **~8 s**: main 循环里排在 `cart_report_if_done` **前面**的
       `usb_cmd_report()` (`main.c:179`) 可能卡在参数保存的阻塞擦写上 —— 固件自述
       "擦写 ~1s CPU 全停" (`usb_cmd.c:1272`), 另两处按 ~1.86s 记 (`usb_cmd.c:690` /
       `usb_cmd.h:94`)。⚠ **这两个只是典型值**, 唯一的上界是同一个现象在**看门狗**那个
       量纲上的自述: `hw_watchdog.h:28` "擦除期间 CPU 取指停顿 … 擦写前放宽到 ~8 s"
       (实现在 `hw_watchdog.c:63-66`, 由 `flash_store.c:332/:351` 夹住整次擦写)。取它。
    ③ **链路 + 主机侧取帧余量** ~1 s。

    ⇒ 下限 ≈ 3 + 8 + 1 = **12 s**。⚠ 只按 ① 取 (旧写法: 3s + 1s = 4s) **短一截** ——
    那只覆盖了"结果被生成", 没覆盖"它还要等 main 循环把它送出去"。
    ⚠⚠ 下限**两个方向都错**, 别只记一侧: 取大 ⇒ 真脱同步被**静默**吸收; 取小 ⇒ 迟到的
    陈旧应答配给新 token ⇒ **假成功**。故宁可取大 (这是一条**正确性条件**, 不是余量)。
    """
    assert cart_mod._ABSORB_TTL_FLOOR >= 12.0, (
        "下限没有覆盖'生成 3s + 发射时延 ~8s (IWDG park 窗口, hw_watchdog.h:28)' —— "
        "额度会先过期, 陈旧应答就会配给新 token ⇒ 假成功")
    assert cart_mod._ABSORB_TTL_FLOOR <= 15.0, (
        "余量过大: 它只让'真脱同步被静默吸收'的时间窗变长, 对正确性没有贡献")
    assert cart_mod.cart_absorb_ttl(0.1) == cart_mod._ABSORB_TTL_FLOOR
    assert cart_mod.cart_absorb_ttl(20.0) == 20.0


class _SwallowPlanOnce(FakeTransport):
    """第一条**合法载荷**的 `0x3A` 只回 ACK、**不推** `0x4E` —— 模拟"应答比
    `move_timeout` 还慢" (规划慢了 / 上行挤满了 / 我们读得太迟)。

    ⚠ 空载荷 (`probe` 的探测帧) 不走这一支: 它照常撞长度校验回 `ERR{0x3A,0x01}`。
    """

    swallow = True

    def _cart_cmd(self, cmd: int, payload: bytes) -> None:
        if cmd == P.CMD_MOVE_L and payload and self.swallow:
            self.swallow = False
            self._push(P.pack_frame(P.RSP_ACK, bytes([cmd])))
            return
        super()._cart_cmd(cmd, payload)


def test_a_reply_that_outlives_move_timeout_is_not_paired_to_the_next_request(monkeypatch):
    """⚠⚠ 额度过期 + 队列非空 ⇒ 清队前的**陈旧** `0x4E` 被悄悄配给**新** token:
    调用方**报成功**, 却拿到**别人那条**的规划结果 (本项目最忌讳的方向, E3 实测)。

    场景 (真机形状): 固件那条应答比 `move_timeout` 还慢 ⇒ 主机先超时放弃
    (`drop_and_absorb` 留一格额度), 而应答随后才到。**额度还活着**时它被吸收, 下一条
    命令拿到**自己**那条; **额度已经过期**时 `on_reply` 就直接把它配给队列里**新登记**
    的那条 token —— `0x4E` 载荷里**没有命令 id**, 它无从分辨。

    ⇒ 额度的存活时间必须覆盖"固件还可能发出那条应答"的**最长时间** —— 那是一个三段
    推导的**到达**上界 (见 `_ABSORB_TTL_FLOOR`: 生成 3s + 发射时延 ~8s + 链路余量)。
    光在 docstring 里写"过期后不可归属"**不改可达性** —— 本用例就是那条可达性。

    判据: 第二条拿到的必须是**它自己**那条的 `n_wp`/`plan_us`。
    """
    from litearm.arm import Arm

    def factory(port="fake", timeout=0.2, **_ignored):
        return _SwallowPlanOnce(port=port, timeout=timeout)

    monkeypatch.setattr(arm_mod, "SerialTransport", factory)
    arm = Arm(port="fake", move_timeout=0.15).connect()
    tr = arm._tr
    tr.swallow = True

    with pytest.raises(CartReplyLostError):
        arm.move_l(_L_POSE, speed=0.3, wait=False)      # 固件**其实已受理**

    time.sleep(0.20)        # ← 比 `move_timeout` (0.15) 还慢 —— 额度若只有那么长就已过期
    tr.push_frame(P.RSP_CART_PLAN, _plan_payload(n_wp=99, plan_us=1111))   # 第一条那条

    plan = arm.move_l(_L_POSE, speed=0.3, wait=False)
    assert (plan.n_wp, plan.plan_us) == (tr.cart_n_wp, tr.cart_plan_us), (
        f"第二条拿到了 (n_wp={plan.n_wp}, plan_us={plan.plan_us}) —— 那是**第一条**那条"
        f"陈旧应答的结果 (自己那条是 n_wp={tr.cart_n_wp}, plan_us={tr.cart_plan_us}) "
        f"= 假成功; 额度到期得太早 (`move_timeout`=0.15 不足以覆盖它)")
    assert arm._cart.absorbed_replies == 1, "那条陈旧应答不是被吸收的 —— 那就是被错配了"


# ---------------------------------------------------------------------------
# 6. 放弃路径: `wait` 超时后的迟到应答 + "待认领"列的界 (Task 4.6)
# ---------------------------------------------------------------------------

class _SilentCartAck:
    """混入: **合法载荷**的 `0x3A` 只记不发 (应答不回) —— 模拟"ACK 丢在链路上"。

    这正是 `_request_and_wait` 那条决定的现场: 命令**已送达** (固件会受理并回 `0x4E`),
    但主机读不到 ACK ⇒ 超时放弃。空载荷的探测帧不参与 (照常走 `default`/长度校验)。
    """

    def write_frame(self, cmd, payload=b""):
        if cmd == P.CMD_MOVE_L and payload:
            with self._tx_lock:
                self.tx_log.append((cmd, bytes(payload)))
                self.tx_stamps.append(time.monotonic())
            return
        super().write_frame(cmd, payload)


class _TickingClock:
    """**每次读时都前进**的假时钟 —— 让 `_Ack.expect` 的 1.0s 窗口几乎立刻到期。

    ⚠ 与 `_FakeClock` 的区别: 那个是**冻结**的 (给"额度过期边界"用)。冻结的时钟在这里
    不行 —— `expect` 的循环形如 `while monotonic() < end`, 时钟不动就是**死循环**
    (每次读到的状态帧都不满足 `want`, 继续等)。
    本类不动 `threading` (只换 `arm` 模块里的 `time` 引用), 故不影响别的线程。
    """

    def __init__(self, t: float = 1000.0, step: float = 0.25) -> None:
        self.t = t
        self.step = step

    def monotonic(self) -> float:
        self.t += self.step
        return self.t


def test_a_late_reply_after_a_token_timeout_is_absorbed_not_raised(offline_arm):
    """⚠ `wait` 超时摘 token 之后, 固件**真欠**的那条 `0x4E` 会炸**无关读路径** ——
    摘 token 时必须**同时**留一格吸收额度 (`drop_and_absorb`, 与 `clear_pending` 同构)。

    场景: 固件已受理这条命令 (应答在路上), 而主机等不到就先超时放弃了 —— `wait` 摘掉
    token 并抛 `CartReplyLostError` (**结局未知**, 这个判断是对的)。随后那条应答到达,
    若不留额度, 它撞上"队列空"判据 ⇒ `LiteArmError` 从 `get_state()` 里炸出来, 实测形状
    与一次**真**脱同步**一模一样** ("收到 0x4E ... 队列为空"), 现场无法归因。

    判据: `get_state()` 不抛, 且 `extra_replies == 0` —— 那一条被**吸收**
    (`absorbed_replies == 1`), 不是被当成脱同步。方向与"多了一条"守卫一致:
    宁可多吸收一条, 绝不把"我们自己放弃了一条"报成"固件与主机已错配"。
    """
    arm = offline_arm
    tok = arm._cart.register()

    with pytest.raises(CartReplyLostError):
        arm._cart.wait(tok, 0.05)
    assert arm._cart.absorbed_replies == 0, "还没给额度就先吸收了一条?"

    arm._tr._resp.clear()
    _push(arm, 0x4E, _plan_payload())               # 固件那条迟到/在路上的应答

    st = arm.get_state(refresh=True, timeout=0.2).value   # ⚠ 无关读路径: 不许抛
    assert st is not None
    assert arm._cart.absorbed_replies == 1
    assert arm._cart.extra_replies == 0, (
        "超时放弃后那条应答被算成了真·脱同步 —— `LiteArmError` 从读路径里炸出来了")


def test_the_unclaimed_column_is_bounded_when_the_caller_gives_up(monkeypatch,
                                                                 fake_transport_factory):
    """⚠ "待认领"列 (`_unclaimed`) 在**调用方放弃 token** 的路径上会无界增长 —— 必须封顶。

    漏点只有一个: `_request_and_wait` 的 ACK 超时 (`MotionTimeoutError`) **刻意不摘**
    token (摘了会让"固件其实已受理"那条应答配给后面的活 token = 假成功 —— 那个决定是
    对的)。于是那条请求的 `0x4E` 一到就落进 `_unclaimed`, 而它的调用方**早已带着异常
    返回**、再也不会来 `wait`; 只有 `poll_cart` 能摘 ⇒ 无界就是内存增长。

    判据 (与轮数无关的"有界"): 跑 `_UNCLAIMED_MAX + 5` 轮 (每轮 = 一次 ACK 超时 + 那条
    迟到的 `0x4E`), 断言列长**恰好**等于上界 (不再随轮数增长), 且越界的被**记进**
    `evicted_unclaimed` —— 丢弃不报错 (报错会从无关读路径炸出来), 但不能是静默的。

    ⚠ 耗时: `expect` 的 1.0s 窗口由 `_TickingClock` 假时钟**快速**推到期, 而 ACK 由
    `_SilentCartAck` 桩**不回** (两个都是这条用例的必要条件: 一个真等 1s × N 轮太慢,
    另一个若不静默则 `expect` 会读到 ACK 而根本不超时)。时钟只在 `connect()` **之后**
    打补丁 —— 握手/探测那几步要用真时钟驱动读循环。
    """
    from fake_serial import FakeTransport

    class _T(_SilentCartAck, FakeTransport):
        pass

    def factory(port="fake", timeout=0.2, **_ignored):
        return _T(port=port, timeout=timeout)

    monkeypatch.setattr(arm_mod, "SerialTransport", factory)
    from litearm import Arm

    arm = Arm(port="fake").connect()
    clock = _TickingClock()                 # 每次读时间戳都前进 -> 窗口几乎立刻到期
    monkeypatch.setattr(arm_mod, "time", clock)

    rounds = _UNCLAIMED_MAX + 5
    for i in range(rounds):
        with pytest.raises(MotionTimeoutError):
            arm.move_l((0.30, 0.0, 0.35, 0.0, 0.0, 0.0), speed=0.3)
        # 调用方带着异常走了, 而固件**其实已受理** —— 那条 0x4E 现在才到
        # (`n_wp` 当序号, 便于分辨留下的是哪几条)
        _push(arm, 0x4E, _plan_payload(n_wp=i))
        _settle(arm)

    assert arm._cart.pending == 0
    assert len(arm._cart._unclaimed) == _UNCLAIMED_MAX, (
        f"放弃路径让'待认领'列涨到 {len(arm._cart._unclaimed)} 条 (上界 {_UNCLAIMED_MAX})"
        f" —— 无界增长")
    assert arm._cart.evicted_unclaimed == rounds - _UNCLAIMED_MAX, (
        "越界丢弃是静默的 (没有计数) —— 本模块其余丢弃点都留了可观测面")

    # ⚠ 界必须保住 `poll_cart` 的语义: 剩下的仍是**可交付的结果**, 且丢的是**最旧**的
    # (丢最新会让调用方先拿到早已过期的那些 —— 同一句 "丢最旧" 必须真的落在 pop(0) 上)。
    assert [arm.poll_cart().n_wp for _ in range(_UNCLAIMED_MAX)] == \
        list(range(rounds - _UNCLAIMED_MAX, rounds)), (
        "留下的不是**最新**那 8 条 (丢错了一端), 或 `poll_cart` 已经交付不出结果")
    assert arm.poll_cart() is None
    assert len(arm._cart._unclaimed) == 0


# ---------------------------------------------------------------------------
# 7. 一条**外来**的 `ERR{cmd}` 不许把本命令判成"被拒" (假成功的高发点)
# ---------------------------------------------------------------------------

#: 随便一条合法 `move_l` 位姿 (这条命令不做起点校验, 只要长度够 28B)。
_L_POSE = (0.30, 0.0, 0.35, 0.0, 0.0, 0.0)


class _ForeignCartErrOnce(FakeTransport):
    """在**受理之前**先塞一条**不属于本次请求**的 `ERR{0x3A,0x01}`, 然后照常受理。

    这条外来帧的来源是真实可达的 (不是编出来的): `cart.probe()` 的 docstring 自己写明
    "迟到的探测 ERR 会从无关读路径里冒出来", 而 `probe` 会**重试** (第一次超时、第二次
    读到第一次那条) ⇒ `connect()` 返回时, 下游还欠着一条 `ERR{0x3A}` 没人读。它一到,
    就会被**下一条** `move_l` 的 ACK 等待撞上。

    之后走 `super()._cart_cmd` —— 也就是**固件其实受理了**这条命令 (ACK + `0x4E` 都会发)。
    """

    stray_pending = True
    reject_next = False

    def _cart_cmd(self, cmd: int, payload: bytes) -> None:
        # `payload` 非空 = 真的是一条命令; 空载荷是 `probe()` 的探测帧, 不掺和
        if cmd == P.CMD_MOVE_L and payload and self.stray_pending:
            self.stray_pending = False
            self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x01])))    # 外来 ERR
        if cmd == P.CMD_MOVE_L and payload and self.reject_next:
            self.reject_next = False
            self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x03])))    # 本条被门禁拒
            return
        super()._cart_cmd(cmd, payload)


def test_a_foreign_err_cannot_be_taken_for_this_commands_own_rejection(monkeypatch):
    """⚠ `expect(echo_cmd=...)` **只按命令码**比对, 而 `0x3A/0x3B/0x3E` **共用码空间**
    ⇒ 流里一条**别人的** `ERR{0x3A}` 会被本命令认成"我被拒了"。认错的代价是**双向**的:

    * 报错的那条命令**其实已被受理** ⇒ 调用方拿到了一个假的失败 (臂真的会动);
    * 由此摘掉的那条活 token 让"在途 ⟹ 应答"少一格 ⇒ 固件为它发的那条 `0x4E` 配给
      **下一条**命令 ⇒ **下一条报成功, 而它的命令被固件拒了、一步没跑** (假成功 ——
      本项目最忌讳的方向)。

    判据 (逐条命令, 不能只看"一成功一失败" —— 那恰好是无修复时的观测):
    ① 被受理那条必须拿到 `CartPlan` (它真的跑了);
    ② 被拒那条必须是它自己那条 `CommandRejectedError`, **绝不许**是 `CartPlan`;
    ③ 被拒之后账必须重新平 (`pending == 0` 且没有多出来的吸收额度) —— 否则下一条命令
       的应答会被这笔无主的额度吞掉, 变成此后**每一条**都报"结局未知"。

    ⚠ 判据 ② 的 `ERR` 是**门禁**那条 (`0x03`); 于是"这两个 ERR 是不是同一条命令的"
    在**码**上不可分辨 —— 唯一可分辨的信号是固件**受理必回 `ACK{cmd}`、拒绝绝不回**。
    """
    from litearm.cart import CartPlan

    def factory(port="fake", timeout=0.2, **_ignored):
        return _ForeignCartErrOnce(port=port, timeout=timeout)

    monkeypatch.setattr(arm_mod, "SerialTransport", factory)
    from litearm import Arm

    arm = Arm(port="fake", move_timeout=0.3).connect()
    tr = arm._tr
    assert tr.stray_pending, "用例自身失效: 外来 ERR 在 connect() 期间就被用掉了"

    plan = arm.move_l(_L_POSE, speed=0.3, wait=False)      # ← 固件其实**受理了**这条
    assert isinstance(plan, CartPlan), (
        f"第①条被固件受理了, 却报 {plan!r} —— 它认下了 probe 遗留的那条**外来** ERR")
    assert plan.ok

    tr.reject_next = True
    with pytest.raises(CommandRejectedError):
        arm.move_l((0.40, 0.0, 0.40, 0.0, 0.0, 0.0), speed=0.3, wait=False)

    assert arm._cart.pending == 0
    assert arm._cart.absorbed_replies == 0, (
        "留下了一笔无主的吸收额度 —— 下一条命令的应答会被它吞掉 (那条报'结局未知')")
    tr.reject_next = False
    plan = arm.move_l(_L_POSE, speed=0.3, wait=False)      # 配对没有错位
    assert plan.ok and arm._cart.absorbed_replies == 0


# ---------------------------------------------------------------------------
# 8. 会话重建 = 第二处"销毁记账" (`Arm.connect()` 重建 `_CartPending`)
# ---------------------------------------------------------------------------

def test_reconnect_wakes_the_old_sessions_waiters(fake_transport_factory):
    """⚠ `connect()` 重建 `_CartPending` 时, **旧对象**里在途的 token 必须被**唤醒**。

    不唤醒的后果是"两个窗口分叉": 旧会话那条请求的调用方(可能正阻塞在 `wait` 上)会一直
    挂到 `move_timeout`, 而新会话早已开始 —— 它等的那条应答**永远不会**再配上它
    (新对象是另一份配对状态)。
    """
    fake_transport_factory()
    from litearm import Arm

    arm = Arm(port="fake").connect()
    stale = arm._cart.register()            # 旧会话里在途的一条

    # ⚠ `reconnect()` 而不是 `connect()`: 后者幂等 (已连着即 no-op, 不再重建会话)。
    arm.reconnect()                         # 新会话 = 新 `_CartPending`

    assert stale.lost is not None, (
        "旧会话那条在途 token 没被唤醒 —— 等它的线程会一直挂到 move_timeout, "
        "而重连之后它已经不可能再收到应答了")


def test_reconnect_inherits_the_in_flight_count_as_absorb_credit(fake_transport_factory):
    """⚠ 会话重建是**第二处"销毁记账"**: 旧的账被销毁了, 但那份"可能还欠应答"的额度
    **必须跟着搬过去** —— 否则旧会话那条迟到 `0x4E` 会撞上新对象的"队列为空"判据,
    从一个**毫不相干的读路径** (`get_state()`) 炸出 `LiteArmError`, 归因指向
    "固件与主机已错配", 而真相是"我们自己重建了会话"。

    (`clear_pending`/超时那两处都留了额度; `connect()` 这处从前一格都不给。)
    """
    fake_transport_factory()
    from litearm import Arm

    arm = Arm(port="fake").connect()
    arm._cart.register()                    # 旧会话留下一条在途记账
    arm.reconnect()                         # ⚠ 不是 connect(): 后者幂等, 不重建会话

    _push(arm, 0x4E, _plan_payload(n_wp=7))  # 旧会话那条应答现在才到

    st = arm.get_state(refresh=True, timeout=0.2).value      # 无关读路径: 不许抛
    assert st is not None
    assert arm._cart.extra_replies == 0, (
        "旧会话的迟到应答被算成'真·脱同步' —— `LiteArmError` 从无关读路径里炸出来了")
    assert arm._cart.absorbed_replies == 1, "那条应答既没被吸收、也没报错?"


def test_an_inherited_credit_cannot_perpetuate_itself_across_a_reconnect(
        fake_transport_factory):
    """⚠⚠ 继承额度**不许被超时路径续成永动机** —— 那是 F5 修出来的**新入口** (实测)。

    继承来的那一格额度与"挣来的"额度在 `on_reply` 里**先于配对**被消费 (顺序是载重的,
    见 `on_reply`), 所以它吃掉的是**新会话自己那条**应答: 那条命令报"结局未知"
    (`CartReplyLostError`) —— 这是 F5 的**知情取舍** (两只在原理上无法区分: 载荷里没有
    命令 id, 旧会话那条迟到应答与新 token 自己那条形状完全一样)。

    ⚠ **回不来的那一支才是缺陷**: 那条命令随后超时放弃, 而**超时路径**的
    `drop_and_absorb` 会按"固件还欠它一条"补一格额度 —— 若那格的截止又被顺手续上, 额度
    就**永远 ≥ 1**: 此后**每一条**命令的应答都被吞, 每条都报"结局未知", 且每轮再补一格,
    **自持**。实测 (改前): `#1..#6` 全红, `absorb=1 absorbed=1..6`。

    判据: 第 #1 条照旧报结局未知 (取舍, 必须留着 —— 去掉它就是允许旧会话那条迟到应答
    配给新 token = **假成功**), 但 **#2 起必须恢复正常**。机理不是"少补一格"而是
    "**不许为一条已经被吸收过的请求再记一笔额度**" (见 `drop_and_absorb`: 吸收额度在
    这条 token 的窗口里被消费过 ⟹ 那条被吃掉的应答**就是它的** ⟹ 固件不再欠)。
    """
    fake_transport_factory()
    from litearm import Arm

    arm = Arm(port="fake", move_timeout=0.15).connect()
    arm._cart.register()            # 旧会话留下一条在途记账 ⇒ 新会话继承额度 1
    arm.reconnect()                 # ⚠ 不是 connect(): 后者幂等, 不重建会话
    assert arm._cart._absorb == 1, "用例自身失效: 额度没被继承 (那测的就不是这条路径)"

    got = []
    for _ in range(6):
        try:
            arm.move_l(_L_POSE, speed=0.3, wait=False)
            got.append("ok")
        except CartReplyLostError:
            got.append("lost")

    assert got[0] == "lost", (
        "第 #1 条不该正常 —— 继承那一格**应当**吃掉它的应答 (那是 F5 的取舍; 两只无法区分)")
    assert got[1:] == ["ok"] * 5, (
        f"继承额度自持了: #2 起仍报结局未知 ({got}) —— 超时路径每轮给它续一格, "
        f"此后**每一条**都会报'结局未知'。恢复只该发生在额度过期/被消费光/重连")
    assert arm._cart.absorbed_replies == 1, (
        "吸收计数与'只吃掉一条'不符 —— 自持的痕迹")
