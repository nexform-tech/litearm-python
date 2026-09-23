"""pytest fixtures —— 桩 transport 连接 (离线) / 真机 live 开关。

用法:
  pytest                          # 只跑离线 (桩 transport), 不碰真机
  PYLITEARM_LIVE=1 pytest         # 额外跑真机 live (需接 Litearm1.5.0+ 整臂/台架)
"""
from __future__ import annotations

import os
import sys
import weakref

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)                       # tests/ (fake_serial)
sys.path.insert(0, os.path.join(_HERE, "..", "src"))  # litearm (免安装)

from fake_serial import FakeTransport  # noqa: E402

LIVE = bool(os.environ.get("PYLITEARM_LIVE"))


@pytest.fixture(autouse=True)
def _no_leaked_port_registration():
    """**本用例**收尾时不许留下 open 着的 `SerialTransport` (登记表里那条要还在)。

    为什么需要这道守卫: 端口登记表按**端口字符串**比 (`transport._PORT_OWNERS`)。一个用例
    把传输 open 着留在册上, 下一个用**同一端口名**的用例就会在 `_claim_port` 上炸 —— 而
    "它还留着吗"取决于该传输**何时被回收**: 无环的传输随引用计数当场消失 (没事), 被引用环
    或回溯钉住的传输要等一次全量 gc (于是失败**按用例顺序传染**, 且 gc 何时跑看不出来)。
    `tests/test_transport.py` 的"端点独占"一节靠**各用例独享端口名**绕开这条耦合; 本守卫把它
    变成**泄漏当场、就地报出来**的失败, 与跑的顺序、gc 开不开都无关。

    ⚠ **按增量判**: 只认"本用例新登记进来、且收尾时还开着"的那几条 —— 否则一个没修的泄漏
    会让**后面每一个**用例都报错 (归因就指错了人)。判据比的是 `(端口, 持有者 id)`, 所以
    前一个用例留下的那条不会被算到本用例头上。

    ⚠ 判据只取"**在册 且 `is_open`**": 在册但**已 `close()`** 的传输不占端口 (fd 已释放),
    不算泄漏 —— 那正是 `close()` 该有的收尾 (登记随 `_release_port` 当场消失)。本守卫因此
    不要求"对象被回收", 只要求"口被关上"。
    """
    from litearm.transport import _PORT_OWNERS

    def _snapshot():
        out = {}
        for port, ref in list(_PORT_OWNERS.items()):
            owner = ref()
            if owner is not None:
                out[(port, id(owner))] = owner
        return out

    before = set(_snapshot())

    yield

    leaked = sorted({
        port for (port, _oid), owner in _snapshot().items()
        if (port, _oid) not in before and owner.is_open
    })
    assert not leaked, (
        f"本用例结束时留下了 open 着的传输, 占着端口 {leaked!r} —— 它会挡住后面任何一个用"
        f"同一端口名的用例, 而挡不挡得住还取决于 gc 何时跑 (引用环/回溯钉住的对象要等全量"
        f" gc)。请在用例里 close() 它, 或改用本用例独享的端口名。")


def _live_reader_threads() -> set:
    """当前活着的读线程（按名字）—— `Arm` 的读线程固定叫 `litearm-reader`。"""
    import threading
    return {t for t in threading.enumerate()
            if t.is_alive() and t.name == "litearm-reader"}


@pytest.fixture(autouse=True)
def _no_leaked_reader_threads():
    """⚠ 每条用例结束后**不许**留下读线程 —— 离线侧的**唯一**泄漏哨兵。

    **为什么需要它**：`Arm` 现在带一条读线程，而线程的 target 是绑定方法 ⇒ 线程强引用
    `_Ack`→`Arm`。**忘了 `close()` 的 `Arm` 会带着它的线程一起活着**，而
    `tests/conftest.py` 另一条"端口泄漏"哨兵对离线用例**是空转的** ——
    `FakeTransport` 不登记 `_PORT_OWNERS`，它什么都抓不住（实测）。

    **两条职责分开**（别把第一条读成"哨兵在放水"）：

    1. **自动收尾**：本用例里构造过、又没 `close()` 的 `Arm`，由这里统一收掉 ——
       跑一次测试不该给下一条用例留几十条活线程（那会把整套拖垮）。
       ⚠ 这是**兜底**，不是"可以不 close"：生产代码里 `close()` 仍然必须显式调。
    2. **断言**：收完之后**仍须一条不剩** —— 它验的是「**`close()` 真的停得掉读线程**」
       这个性质。`close()` 少了 `stop_reader()`、或 join 超时不够，这里就会红。
    """
    from litearm import arm as _arm_mod

    #: ⚠ **弱引用**，不是强引用 —— 强引用会把这些 `Arm` 钉到用例结束，于是
    #: `test_teardown.py` 那几条"松开 `Arm` 就该回收 + 跑兜底收尾"的用例全部失效
    #: （它们靠 `del arm` + `gc.collect()` 让 `Arm.__del__` 触发）。**实测踩过**。
    made: list = []
    _orig_init = _arm_mod.Arm.__init__

    def _init(self, *args, **kwargs):
        _orig_init(self, *args, **kwargs)
        made.append(weakref.ref(self))

    _arm_mod.Arm.__init__ = _init
    before = _live_reader_threads()
    try:
        yield
    finally:
        _arm_mod.Arm.__init__ = _orig_init
        for _ref in made:                   # ① 兜底收尾
            arm = _ref()
            if arm is None:
                continue                    # 已被回收(兜底收尾已跑过)
            try:
                arm.close()
            except Exception:               # noqa: BLE001 - 收尾失败不该掩盖断言
                pass
    leaked = _live_reader_threads() - before    # ② 断言
    assert not leaked, (
        f"本用例留下了 {len(leaked)} 条读线程 —— **连 `close()` 都停不掉它**。"
        f"检查 `_Ack.stop_reader()` 是否被 `Arm.close()` 调到、join 超时够不够。")


@pytest.fixture
def fake_transport_factory(monkeypatch):
    """把 arm.SerialTransport 换成 FakeTransport, 返回工厂 (fw=..., port=...)。"""
    import litearm.arm as arm_mod

    def _factory(fw: str = "Litearm1.7.0-7J", **kw):
        def make(port="fake", timeout=0.2, **_ignored):
            return FakeTransport(port=port, timeout=timeout, fw=fw, **kw)
        monkeypatch.setattr(arm_mod, "SerialTransport", make)
        return make
    return _factory


@pytest.fixture
def offline_arm(fake_transport_factory):
    """连上桩固件的 Arm (默认 Litearm1.7.0-7J)。

    ⚠⚠ **必须 `yield` + `close()`** (2026-09-22 起): `Arm` 现在带一条**读线程**,
    而线程的 target 是绑定方法 ⇒ 线程强引用 `_Ack`→`Arm`, 活线程又被 `threading._active`
    强引用 ⇒ **不 `close()` 就回收不掉 `Arm`**, 且线程会一直活着。
    ⚠ 离线侧**没有哨兵兜底**: `FakeTransport` 不登记 `_PORT_OWNERS`, 那条"端口泄漏"
    哨兵对离线用例是空转的 —— 漏了这里就是一堆静默的活线程。
    """
    fake_transport_factory()          # 应用桩 (monkeypatch)
    from litearm import Arm
    arm = Arm(port="fake").connect()
    try:
        yield arm
    finally:
        arm.close()


@pytest.fixture
def offline_arm_1j(fake_transport_factory):
    """连上**台架单电机**桩固件 (Litearm1.7.0-1J, N=1)。

    固件侧台架版: `LITEARM_NUM_JOINTS=1` 且 `LITEARM_BENCH_MODEL_AXIS=5`
    (joint_cfg.h), 即台架那台电机对应整臂运动学模型的第 6 轴。

    ⚠ 同 `offline_arm`: **必须 `yield` + `close()`** (读线程的生命周期)。
    """
    fake_transport_factory(fw="Litearm1.7.0-1J", n=1)
    from litearm import Arm
    arm = Arm(port="fake").connect()
    try:
        yield arm
    finally:
        arm.close()


@pytest.fixture
def live_only():
    """真机用例: 未设 PYLITEARM_LIVE=1 时跳过。"""
    if not LIVE:
        pytest.skip("真机用例需 PYLITEARM_LIVE=1 (并接好 Litearm1.5.0+ 固件)")
