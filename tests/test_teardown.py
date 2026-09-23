"""会话收尾 (teardown) —— 幂等 `connect()`/`disconnect()`/`close()` + 上下文管理器 + 兜底收尾。

分层:
  - **幂等**: 重复调用是 no-op (不抛、也不重建会话);
  - **收口**: `disconnect()` 与 `close()` 是同一个操作; 收尾 = 收保活线程 → 关链路 → 清状态;
  - **兜底**: 用户**没**显式 `close()` 时也必须收干净 —— 且兜底**只能**是 `close()` 自己
    (第二套清理逻辑会漂), 且不许挂死/漏异常。

⚠ 兜底的**机制**与规格 §6.6 写的不一样 (规格写 `weakref.finalize`), 依据是实测:

    `weakref.finalize(obj, func, *args)` 的 `func` 与 `args` **一个都不许引用 obj**
    (CPython 文档原话: 否则 obj 永远不会被回收 ⇒ 兜底永不触发); 而回调真正跑起来时,
    obj **已经**被回收 —— 连 `weakref.ref(obj)` 放进 args 也拿不到东西:

        weakref.finalize(self, _cb, weakref.ref(self))
        # _cb 里 ref() 返回 **None**  (实测: CPython 3.13.5)

    ⇒ "兜底 ⟹ 委托给 `Arm.close()`" 这条要求在 `weakref.finalize` 下**物理上做不到**,
    除非把整个会话状态搬进一个独立对象 (那是重写, 不属于本 Task)。故用 `Arm.__del__`
    —— 它拿到的 `self` 是**活的**, 于是能原样委托给 `close()`。
    ⚠ 代价 (如实记, 不是缺陷掩盖): 保活线程的 target 是 `self._zg_keepalive` (绑定方法),
    线程活着就**钉住** Arm ⇒ **保活进行中**被丢弃的 Arm 收不到兜底 (见下面
    `test_a_running_keepalive_thread_pins_the_arm` —— 它把这条限制钉住, 免得被误读成保证)。
"""
from __future__ import annotations

import gc
import sys
import threading
import time

import pytest

import litearm as pa
from litearm import _protocol as P
from litearm.errors import TransportError


def _no_unraisable(monkeypatch):
    """捕获 `sys.unraisablehook` 的报告 —— 兜底里的异常绝不该从这里漏出去。

    为什么用这个口而不是 `pytest.raises`: 兜底跑在 **gc 时刻**, 异常不会被任何
    `try` 接住 —— 它要么被我们自己吞掉, 要么以 "Exception ignored in ..." 的形式
    走 `sys.unraisablehook` (实测: 不吞的 `__del__` 会走这里, 吞掉的不会)。
    """
    seen = []
    monkeypatch.setattr(sys, "unraisablehook", lambda a: seen.append(str(a.exc_value)))
    return seen


# =========================== 幂等: connect ===========================

def test_connect_three_times_does_not_rebuild_the_session(offline_arm):
    """重复 `connect()` **不抛**, 而且必须是 **no-op** —— 不是"关掉再连一遍"。

    只断言"不抛"是不够的: 旧实现每次 `connect()` 都 `close()` 后重建 (桩上照样不抛),
    于是**在途的笛卡尔记账被清空、保活被中断、串口被关掉又打开** —— 一个"重复调用"
    白白打断正在跑的会话。判据取 **transport 身份不变** (no-op 的硬证据)。
    """
    arm = offline_arm
    tr = arm._tr
    for _ in range(3):
        assert arm.connect() is arm
    assert arm._tr is tr, "重复 connect() 重建了会话 —— 它是 no-op, 不是重连"


def test_connect_with_a_different_port_does_retarget(offline_arm):
    """⚠ **幂等不是"永远不做事"**: 显式给了**另一个端口**时必须真的改靶。

    否则 `connect("/dev/ttyACM2")` 会被静默吞掉, 用户以为连的是 ACM2 而实际还在 ACM1
    —— 本模块一贯不接受这种"静默无效"。要"强制重建同一个目标"用 `reconnect()`。
    """
    arm = offline_arm
    tr = arm._tr
    arm.connect("/dev/other")
    assert arm._tr is not tr, "换端口被静默吞掉了"
    assert arm._tr.port == "/dev/other"


def test_retargeting_closes_the_old_transport_before_opening_the_new_one(offline_arm):
    """⚠ 改靶必须**先收干净旧会话**再开新的 —— "换个引用"不是收尾。

    旧链路不关 = 串口句柄还开着 (fd/驱动侧资源泄漏), 而且旧传输**仍在册**
    (`transport._PORT_OWNERS`) ⇒ 同一个口想再连回来会被自己的僵尸挡住。既有那条改靶
    用例只断言"换了新 `_tr` 且 `.port` 对", 对"旧链路关没关"**恒不敏感**
    (实测: 把那一处 `self.close()` 换成 `pass`, 全量 531 passed 全绿)。
    """
    arm = offline_arm
    old = arm._tr
    arm.connect("/dev/other")

    assert arm._tr is not old, "换端口被静默吞掉了"
    assert arm._tr.port == "/dev/other"
    assert not old.is_open, (
        "改靶没有关掉旧链路 —— 旧串口还开着 (既漏句柄, 又让旧端口名在册占位)")


def test_reconnect_forces_a_fresh_session(offline_arm):
    """`reconnect()` 与 `connect()` 的分工: 前者**无条件**重建, 后者已连就 no-op。

    (这条是为了让 `reconnect()` 名副其实 —— 旧实现它是 `return self.connect(port)`,
    一旦 `connect()` 变成 no-op, 它就会跟着变成一个什么都不做的别名。)
    """
    arm = offline_arm
    tr = arm._tr
    assert arm.reconnect() is arm
    assert arm._tr is not tr, "reconnect() 没重建会话"


# =========================== 幂等: transport.close ===========================

def test_serial_transport_close_is_idempotent(monkeypatch):
    """`SerialTransport.close()` 幂等 —— 判据不止"不抛", 还有"**第二次不再碰串口**"。

    上层有**三条**收尾路径 (`close()` / `disconnect()` / `__del__` 的兜底) 全落在这一个
    方法上, 幂等是它们能互相叠加的前提。⚠ 只断言"不抛"是不够的: pyserial 的 `close()`
    自己带 `if self.is_open:` 守卫, 于是"靠它兜"的实现也能不抛 —— 但那是**别人的**
    实现细节 (它 `os.close()` 中途失败时 `is_open` 会停在 True, 下一次就重试一串已经
    关掉的 fd), 契约得由我们这一层说了算。判据取**实际调用次数**。
    """
    import types

    from litearm.transport import SerialTransport

    calls = []

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
            calls.append(1)
            self.is_open = False

    monkeypatch.setitem(sys.modules, "serial", types.SimpleNamespace(Serial=FakeSerial))
    tr = SerialTransport("/dev/null")
    assert tr.is_open is True
    tr.close()
    tr.close()
    tr.close()
    assert len(calls) == 1, f"重复 close() 又去关串口了 (实际调了 {len(calls)} 次)"
    assert tr.is_open is False


# =========================== 幂等: disconnect / close ===========================

def test_disconnect_three_times_does_not_raise(offline_arm):
    """`disconnect()` 是幂等的 —— 连续 3 次是 no-op, 不是错误。"""
    arm = offline_arm
    tr = arm._tr
    arm.disconnect()
    assert tr.closed is True, "disconnect() 没关链路"
    for _ in range(3):
        arm.disconnect()


def test_disconnect_on_a_never_connected_arm_is_silent():
    """从未 `connect()` 过的对象上 `disconnect()` 也必须不抛 (teardown 永远可达)。"""
    pa.Arm(port="/dev/never").disconnect()


def test_close_three_times_does_not_raise(offline_arm):
    """`close()` 幂等 —— 它是所有收尾路径的**唯一**实现, 不许因为"已经关过"就抛。"""
    arm = offline_arm
    arm.close()
    for _ in range(3):
        arm.close()


def test_disconnect_and_close_are_the_same_operation(fake_transport_factory):
    """两个名字**一个操作** —— `disconnect()` 只是松灵 API 的肌肉记忆名。

    判据取"可观测结局逐条相同": 链路关、`_tr`/`_a` 置空、保活停、会话状态清零。
    """
    fake_transport_factory()
    a, b = pa.Arm(port="fake").connect(), pa.Arm(port="fake").connect()
    a._zg_active = True                       # 让"收保活"这一步有活干
    b._zg_active = True
    a.close()
    b.disconnect()
    for arm in (a, b):
        assert arm._tr is None and arm._a is None
        assert arm._zg_active is False
        assert (arm.firmware, arm.fw_version, arm.n) == ("", None, 0)


def test_close_clears_the_session_state(offline_arm):
    """收尾的第三件事是**清状态** —— 关掉之后不许再报上一个会话的事实。

    `firmware`/`n`/`_cart_supported` 都是**握手与探测**的产物: 链路一关, 它们说的就是
    一个已经不存在的会话 (下一个对象可能是另一台臂、另一个固件版本)。留着它们必然
    导致"读到一个没人再维护的旧值"。
    """
    arm = offline_arm
    assert arm.firmware.startswith("Litearm") and arm.n > 0, "前置: 握手已完成"
    assert arm._cart_supported is not None, "前置: 笛卡尔能力已探过"
    arm.close()
    assert arm.firmware == "", "close() 后仍在报旧会话的固件版本串"
    assert arm.fw_version is None
    assert arm.n == 0, "close() 后仍在报旧会话的关节数"
    assert arm._cart_supported is None, "close() 后仍缓存着旧会话的能力探测结果"


def test_close_does_not_clear_what_is_not_session_state(offline_arm):
    """⚠ 反向判据: **别把旋钮和累计计数一起清掉**。

    - `_tx_repeat_min_interval` 是**用户配置**, 清掉 = 静默关掉同帧节流;
    - `_tx_throttled_frames` 是**累计可观测计数** (与 `flush_failures` 同类), 它记的是
      "这个进程一共丢过几帧", 不随会话归零;
    - `_dfu_entered` 是会话**终态** (见 `ArmIsInDfuError`) —— 关闭**不**能给它解锁;
    - `_cart` 的在途记账**刻意**不在这里清 (见 `Arm.connect()` 里那段: 断连那一刻在途
      token 的结局属 `wait()` 的决定)。
    """
    arm = offline_arm
    arm._set_tx_repeat_min_interval(P.CMD_GET_FIRMWARE, 0.05)
    arm._tx_throttled_frames = 7
    cart = arm._cart
    arm._dfu_entered = True                   # 直接置位: 只验 close() 不动它
    arm.close()
    assert P.CMD_GET_FIRMWARE in arm._tx_repeat_min_interval, "用户配置被 close() 清掉了"
    assert arm._tx_throttled_frames == 7, "累计计数被 close() 归零了"
    assert arm._dfu_entered is True, "close() 给终态解锁了"
    assert arm._cart is cart, "close() 重建/清空了在途记账 (那是 connect()/wait() 的事)"


# =========================== 上下文管理器 ===========================

def test_context_manager_closes_the_link_on_exit(fake_transport_factory):
    """`with Arm(...) as arm:` 退出即关 —— 不管块里有没有异常。"""
    fake_transport_factory()
    with pa.Arm(port="fake") as arm:
        tr = arm._tr
        assert tr is not None and arm.firmware.startswith("Litearm")
    assert tr.closed is True, "with 块退出后链路还开着"
    assert arm._tr is None


def test_context_manager_closes_even_on_exception(fake_transport_factory):
    """块里抛异常也要关 (且不吞掉那个异常 —— `__exit__` 返回 False)。"""
    fake_transport_factory()
    arm = pa.Arm(port="fake")
    with pytest.raises(RuntimeError, match="块内故障"):
        with arm:
            tr = arm._tr
            raise RuntimeError("块内故障")
    assert tr.closed is True, "with 块异常退出后链路没关"
    assert arm._tr is None


def test_context_manager_reuses_an_already_connected_arm(offline_arm):
    """已经连上的对象进 `with` 不该重连, 但**退出时必须关**。"""
    arm = offline_arm
    tr = arm._tr
    with arm as got:
        assert got is arm
        assert arm._tr is tr, "进 with 时重连了"
    assert tr.closed is True, "退出 with 时没关"


# =========================== 兜底收尾 (没有显式 close) ===========================

def test_dropping_the_arm_closes_the_link(fake_transport_factory):
    """**核心判据**: 构造 → 丢弃引用 → 触发 GC ⇒ 链路已关 (句柄不泄漏)。

    必须 `gc.collect()`: `Arm` 与 `_Ack`/`_CartPending` 之间是**引用环**
    (`_CartPending(arm=self)`), 光靠引用计数收不掉。
    """
    fake_transport_factory()
    arm = pa.Arm(port="fake").connect()
    tr = arm._tr
    assert tr.closed is False
    del arm
    gc.collect()
    assert tr.closed is True, "丢弃 Arm 后串口句柄仍开着 (兜底没生效)"


def test_the_backstop_is_close_itself(fake_transport_factory):
    """⚠⚠ 兜底**不许**是第二套清理逻辑 —— 判据: 它走的**就是** `Arm.close()`。

    用一个把 `close()` 记账 (并 `super().close()`) 的子类: 若兜底自己另外实现一套,
    这里的计数就是 0。两套清理逻辑必然漂 (改了一处忘了另一处)。
    """
    calls = []

    class Spy(pa.Arm):
        def close(self):
            calls.append(1)
            super().close()

    fake_transport_factory()
    arm = Spy(port="fake").connect()
    tr = arm._tr
    del arm
    gc.collect()
    assert calls == [1], "兜底没走 close() —— 那就是第二套清理逻辑"
    assert tr.closed is True, "兜底走了 close(), 但链路没关"


def test_backstop_still_armed_after_close_then_connect(fake_transport_factory):
    """⚠ `close()` **不许**把兜底拆掉 (detach): 关掉之后又连上的会话, 照样要有人收尾。

    否则 `arm.close(); arm.connect()` 之后丢弃对象 ⇒ **新**会话的句柄泄漏 —— 而这条
    路径恰恰是"重连"最常见的写法。
    """
    fake_transport_factory()
    arm = pa.Arm(port="fake").connect()
    arm.close()
    arm.connect()
    tr2 = arm._tr
    del arm
    gc.collect()
    assert tr2.closed is True, "close() 之后重连的会话没被兜底收掉"


def test_backstop_closes_the_link_even_when_teardown_raises(fake_transport_factory, monkeypatch):
    """兜底里收保活失败 (抛) 时: **吞掉异常后照常关链路**, 且不许漏成 unraisable。

    这就是段二 `close()` 上那条注释记的坑的另一半 —— 解释器退出/GC 时刻把异常抛出去,
    轻则 "Exception ignored in" 刷屏, 重则退出流程挂死。
    """
    seen = _no_unraisable(monkeypatch)
    fake_transport_factory()
    arm = pa.Arm(port="fake").connect()
    tr = arm._tr

    def boom():
        raise TransportError("模拟收保活失败 (串口写卡住)")

    arm.zero_g_stop = boom
    arm._zg_active = True                     # 让 close() 真的走"收保活"这一步
    del arm
    gc.collect()
    assert tr.closed is True, "收尾抛异常后链路没关 —— teardown 必须照常走完"
    assert not seen, f"兜底把异常漏成了 unraisable: {seen}"


def test_backstop_does_not_hang_on_a_slow_teardown_write(fake_transport_factory):
    """兜底期间那次退出帧 (0x06 off) 写得很慢时: 仍然**返回**并把链路关掉。

    "不能打哑 CDC" 的可测形态就是这条 —— 收尾路径不许变成一个不返回的调用。
    """
    fake_transport_factory()
    arm = pa.Arm(port="fake").connect()
    tr = arm._tr
    real_write = tr.write_frame

    def slow_write(cmd, payload=b""):
        if cmd == P.CMD_ZERO_G:
            time.sleep(0.4)
        return real_write(cmd, payload)

    tr.write_frame = slow_write
    arm._zg_active = True
    del arm
    t0 = time.monotonic()
    gc.collect()
    elapsed = time.monotonic() - t0
    assert tr.closed is True, "慢写之后链路没关"
    assert elapsed < 5.0, f"兜底花了 {elapsed:.2f}s —— 收尾不许无界等待"


def test_a_running_keepalive_thread_pins_the_arm(fake_transport_factory):
    """⚠ **如实钉住一条限制** (不是保证): 保活线程活着时, 丢弃 Arm 收不到兜底。

    机制: 线程的 target 是 `self._zg_keepalive` (绑定方法) ⇒ 线程持有一个对 Arm 的
    **强引用**, 且运行中的线程本身被 `threading._active` 强引用 ⇒ Arm 根本不可回收,
    兜底自然没机会跑。**这不是本 Task 引入的**, 而是"保活线程"这个设计的固有性质;
    写在这里是为了不让 `__del__` 被读成"忘了关也一定没事" —— 保活期该走
    `with arm.zero_g():` (它退出时会停保活), 而不是指望兜底。

    ⚠ 本条**不能**用 `offline_arm` 夹具: pytest 会把夹具结果缓存到用例结束, 那时候
    `del arm` 掉的只是本帧的引用, 对象照样活着 (那样这条判据会变成永真的假绿)。
    """
    fake_transport_factory()
    arm = pa.Arm(port="fake").connect()
    tr = arm._tr
    arm._zg_active = True
    arm._zg_stop.clear()
    # 真的保活线程 (period 5s: 用例结束前不会真写帧), target 是绑定方法
    th = threading.Thread(target=arm._zg_keepalive, args=(5.0,), daemon=True)
    th.start()
    arm._zg_thread = th
    stop_ev = arm._zg_stop
    del arm
    gc.collect()
    assert tr.closed is False, (
        "这条钉的是**限制**: 线程持引用时兜底跑不了 —— 如果这里变成 True, 说明线程的"
        "持引用方式变了, 请同步更新本模块 docstring 与 `Arm.__del__` 的说明")
    stop_ev.set()
    th.join(2.0)
    assert not th.is_alive()
    gc.collect()
    assert tr.closed is True, "线程退出后兜底应当补上"
