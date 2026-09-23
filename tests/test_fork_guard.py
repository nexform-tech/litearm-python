"""fork 守卫 —— 见 `litearm.errors.ForkedSessionError`。

**为什么判据必须在子进程里跑**：父进程里"守卫不触发"只是基线；要被证明的性质是
"**fork 出来的子进程**用继承来的会话会被拒、且**一个字节都不下发**"。这两件事在父进程
里都观察不到 —— 只能在 `os.fork()` 之后把结论带回父进程断言。

桩与真机在这个性质上的差别只有一处：真机上那条命令**确实**到了 CDC（实测用 pty 当串口：
父进程的排水线程读到了那条 MOVE_J 帧），桩上它只到 `write_frame`。故本文件把"零下发"
钉在 `FakeTransport.tx_log` 的**增量**上 —— 它对应真机的"字节出没出 USB"。
"""
from __future__ import annotations

import os
import pickle
import threading

import pytest

from litearm import ForkedSessionError, NotConnectedError

# Python 3.12+ 对多线程进程里的 fork 会打 DeprecationWarning —— 本文件**就是要**测那个
# 场景，警告不是问题信号。
pytestmark = pytest.mark.filterwarnings("ignore::DeprecationWarning")


def _in_child(fn):
    """在 fork 出的子进程里跑 `fn()`，把 `(status, payload)` 带回父进程。

    ⚠ 子进程**必须 `os._exit`**：让 pytest 的收尾在子进程里再跑一遍会重复拆夹具、重复报
    结果（而且那些夹具在子进程里未必成立）。
    """
    r, w = os.pipe()
    pid = os.fork()
    if pid == 0:                                     # ---- 子进程 ----
        os.close(r)
        try:
            result = ("ok", fn())
        except BaseException as e:                   # noqa: BLE001 - 子进程里不许逃逸
            result = ("err", f"{type(e).__name__}: {e}")
        try:
            blob = pickle.dumps(result) + b"\x00"    # 哨兵: 见父进程那边的截断
        except Exception:                            # noqa: BLE001
            blob = pickle.dumps(("err", "unpicklable")) + b"\x00"
        while blob:
            blob = blob[os.write(w, blob):]
        os._exit(0)                                  # ⚠ 不是 sys.exit
    # ---- 父进程 ----
    os.close(w)
    chunks = []
    while True:
        b = os.read(r, 65536)
        if not b:
            break
        chunks.append(b)
    os.close(r)
    os.waitpid(pid, 0)
    return pickle.loads(b"".join(chunks)[:-1])       # 去掉哨兵


def _try(fn):
    """调 `fn()`，把"抛了什么"变成数据（异常对象本身不可靠地跨进程 pickle）。"""
    try:
        fn()
        return {"exc": None, "msg": ""}
    except BaseException as e:                       # noqa: BLE001
        return {"exc": type(e).__name__, "msg": str(e)}


def test_the_exception_is_catchable_as_not_connected():
    """契约: 子进程里"这个会话不可用"与"没连接"是**同一类**，故继承 `NotConnectedError`。"""
    assert issubclass(ForkedSessionError, NotConnectedError)


def test_reading_state_in_the_child_raises_instead_of_returning_stale(offline_arm):
    """读路径必须**响亮** —— 这正是 fork 最隐蔽的一处。

    没有守卫时的行为是**不报错**的：`get_state()` 等不到新状态帧，烧满超时后回**继承来的
    那个陈旧 `state`** ⇒ 调用方完全看不出自己在读一个没有读者的会话。
    """
    owner_pid = os.getpid()
    status, value = _in_child(lambda: _try(lambda: offline_arm.get_state(refresh=True)))
    assert status == "ok", value
    assert value["exc"] == "ForkedSessionError", value
    # 报错要点名**会话属于哪个 PID** —— 否则用户不知道该去哪儿找那个 owner。
    assert str(owner_pid) in value["msg"], value


def test_the_child_sends_zero_bytes(offline_arm):
    """**本文件最重要的一条**: 子进程里命令被拒, 而且**一个字节都没下发**。

    对应真机上的安全性质: 子进程报"无应答"的同时, 那条命令**不该**已经在 CDC 上跑。
    """
    def probe():
        tr = offline_arm._tr
        before = len(tr.tx_log)
        out = _try(lambda: offline_arm.movej([0.1, 0, 0, 0, 0, 0, 0], speed=0.3))
        out["written"] = len(tr.tx_log) - before
        return out

    status, value = _in_child(probe)
    assert status == "ok", value
    assert value["exc"] == "ForkedSessionError", value
    assert value["written"] == 0, (
        f"子进程往下行写了 {value['written']} 条帧 —— fork 守卫没拦住唯一写口")


def test_guard_covers_a_frame_waiting_entry(offline_arm):
    """取帧型入口（`get_tcp` 走 `_Ack._wait`）同样被拦 —— 收口不能只盖住 `_cmd`。"""
    status, value = _in_child(lambda: _try(offline_arm.get_tcp))
    assert status == "ok", value
    assert value["exc"] == "ForkedSessionError", value


def test_the_sole_write_port_refuses_the_child_directly(offline_arm):
    """**唯一写口自己**必须拦得住 —— 这是"子进程零下发"的**结构性**依据。

    ⚠ 为什么这条非有不可：上面那些入口（`movej` / `get_tcp`）在**上游**（`_require()` /
    `_Ack._wait()`）就已经被拦，所以它们红不红**证明不了**写口这道守卫 —— 实测把
    `_raw_write` 里那句守卫单独摘掉，本文件其余用例**全绿**（判据没有覆盖它）。
    """
    def probe():
        tr = offline_arm._tr
        before = len(tr.tx_log)
        out = _try(lambda: offline_arm._raw_write(0x06, b"\x01"))
        out["written"] = len(tr.tx_log) - before
        return out

    status, value = _in_child(probe)
    assert status == "ok", value
    assert value["exc"] == "ForkedSessionError", value
    assert value["written"] == 0, value


def test_the_keepalive_writer_cannot_reach_the_wire(offline_arm):
    """**绕过 `_require` 的那个写者**（`zero_g` 保活线程）也到不了线上。

    这是**唯一**能隔离写口守卫的判据：`_zg_keepalive()` 刻意走 `_raw_write` 而不走
    `_write_cmd`（见它的 docstring），所以它不经过 `_require()`。摘掉写口那句守卫 ⇒
    这条**必红**（0x06 会真写出去、`_zg_error` 保持 None）。

    线程体把写失败收进 `_zg_error`（不是静默吞掉），故判据读那一格 —— 与
    `zero_g_stop(raise_on_lost=True)` 消费它的口径一致。

    ⚠ **必须有界**：`_zg_keepalive` 是个 `while not _zg_stop.wait(period)` 循环，它靠
    "写失败 ⇒ 置 `_zg_stop`" 退出。守卫一旦缺失，这个循环**永远不退出** ⇒ 直接在子进程里
    顺序调用会让**父进程卡在管道读上**（实测：变异后整轮挂死到被 SIGTERM，而不是给出红
    结论）。故放进子线程 + 到时收工：把"挂死"变成"这条红了，且带着帧数"。
    """
    def keepalive_bounded(window_s: float):
        tr = offline_arm._tr
        before = len(tr.tx_log)
        done = threading.Event()

        def body():
            try:
                offline_arm._zg_keepalive(0.005)
            finally:
                done.set()

        threading.Thread(target=body, daemon=True).start()
        finished = done.wait(window_s)
        offline_arm._zg_stop.set()                # 兜底收工（正常路径下它已被自己置上）
        done.wait(1.0)
        return {
            "exited_on_its_own": finished,
            "written": len(tr.tx_log) - before,
            "zg_error": type(offline_arm._zg_error).__name__,
        }

    status, value = _in_child(lambda: keepalive_bounded(0.5))
    assert status == "ok", value
    assert value["exited_on_its_own"], (
        "保活线程没有自己退出 —— 说明它的写没有被拒（守卫缺失时的形态）")
    assert value["written"] == 0, (
        f"保活线程在子进程里真写出了 {value['written']} 条 0x06 —— 写口守卫没拦住")
    assert value["zg_error"] == "ForkedSessionError", value


def test_close_is_still_allowed_in_the_child(offline_arm):
    """收尾**必须放行**（与 `emergency_stop` 降能量方向可达同理），否则子进程连清理都做不了。

    ⚠ 关完之后出口是"没会话"（`NotConnectedError`），不是"守卫又拦了一次" —— 两条路
    的异常类型不同，判据要把这点钉住。
    """
    def close_then_report():
        offline_arm.close()
        return _try(offline_arm.get_state)

    status, value = _in_child(close_then_report)
    assert status == "ok", value
    assert value["exc"] == "NotConnectedError", value


def test_close_in_the_child_does_not_touch_the_transport(offline_arm):
    """子进程里 `close()` **不许碰传输层** —— 真机上那一步会**永久挂死**。

    ⚠⚠ 这条是**真机抓出来的**（2026-09-22）。为什么离线测不出：
    上面那条"close() 能返回"在离线**恒为真** —— `FakeTransport.close()` 就是一句赋值、
    没有锁, 所以判据没有判别力。真机上 `SerialTransport.close()` 要取 `self._rlock`
    （`transport.py:218`），而**父进程的读线程几乎一直持着它**（`read_frame` 阻塞满
    `_READ_SLICE_S` 期间都持锁）⇒ fork 出的子进程继承到一把**已加锁的互斥量**，能解锁的
    那个线程又不在子进程里 ⇒ `with self._rlock` 永远等不到 ⇒ `Arm.close()` 卡死。

    故判据换成**结构性**的：子进程里 `close()` 之后，传输**从未被 close 过**。
    修复前（`close()` 会走到 `self._tr.close()`）这条**必红**。
    """
    tr = offline_arm._tr

    def probe():
        offline_arm.close()
        return {"transport_closed": tr.closed, "session_cleared": offline_arm._tr is None}

    status, value = _in_child(probe)
    assert status == "ok", value
    assert value["transport_closed"] is False, (
        "子进程里 close() 调用了传输层的 close() —— 真机上那一步会取 _rlock 并永久挂死")
    assert value["session_cleared"] is True, (
        "子进程里 close() 仍应清掉会话引用（_a/_tr 置 None），否则守卫会一直放行")


def test_a_fresh_arm_works_in_the_child(offline_arm):
    """**对照组**: 守卫不是"子进程一律禁用"。

    子进程**自己新建**的 `Arm` 必须完全可用 —— 否则这条守卫就从"防误用"变成了"禁止多进程
    用臂"，那是另一回事（也是用户真正会踩的用法）。
    """
    def fresh_arm_moves():
        from litearm import Arm
        arm = Arm(port="fake").connect()
        try:
            arm.movej([0.1, 0, 0, 0, 0, 0, 0], speed=0.3)
            return "moved"
        finally:
            arm.close()

    status, value = _in_child(fresh_arm_moves)
    assert status == "ok", value
    assert value == "moved", value


def test_forking_does_not_poison_the_parent(offline_arm):
    """反向判据: 父进程的会话在 fork 之后**照常可用**（否则就是过度拦截）。"""
    status, _ = _in_child(lambda: None)
    assert status == "ok"
    offline_arm.movej([0.1, 0, 0, 0, 0, 0, 0], speed=0.3)
    assert offline_arm.get_state(refresh=True).value is not None
