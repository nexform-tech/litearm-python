"""帧归属 —— 读线程模型的**验收闸门**（2026-09-22 读路径重构）。

**这条不变量一句话**：

> 推进链路的任一帧，要么被某个队列的等待者取走，要么在**队列封顶**时被计数。
> **不存在"静默销毁"这条路。**

它靠一个机制成立：**SDK 只有一条读线程**（`_Ack._reader_loop`，唯一碰
`transport.read_frame` 的地方），它把每帧按 `(上行 id, 回显码)` 投进队列，
等待者只从自己的队列里取。于是"谁读到归谁"这个问题**不存在**了。

⚠ 旧实现在这里放的是 11 条 `xfail`（那时断言的是 `_take_pending`）。那套机制
（暂存盒 + `want` + `since` + `unexpected_frames` / `foreign_frames` 八处计数）
已整体删除，本文件随之改写 —— **契约不变，断言的机制换了**。
"""
from __future__ import annotations

import ast
import pathlib
import threading

import pytest

from litearm import _protocol as P
from litearm.arm import _QUEUE_MAX, _wait_keys
from litearm.errors import InvalidCommandError, MotionTimeoutError, NotConnectedError

_ARM_PY = pathlib.Path(__file__).resolve().parents[1] / "src" / "litearm" / "arm.py"


def _push(arm, fid: int, payload: bytes = b"") -> None:
    """往桩的响应队列里塞一帧（桩上没有 `feed`，实测写法是 `_push(pack_frame(...))`）。"""
    arm._tr._push(P.pack_frame(fid, payload))


# ---------------------------------------------------------------------------
# 1. 结构性判据 —— 这两条一红，整套机制的地基就没了
# ---------------------------------------------------------------------------

def test_read_frame_has_a_single_call_site_and_it_is_the_reader_loop():
    """全包只有**一处** `transport.read_frame` 调用点，且它在读线程里。

    ⚠ 这是本设计的**承重结构**：只要还有第二处，那份代码就又在"自己决定帧归谁"。
    判据走 AST（不是 grep）：散文里提到 `read_frame` 不算，注释掉的也不算。
    """
    tree = ast.parse(_ARM_PY.read_text(encoding="utf-8"))
    sites = []
    for fn in ast.walk(tree):
        if not isinstance(fn, ast.FunctionDef):
            continue
        for node in ast.walk(fn):
            if (isinstance(node, ast.Call)
                    and getattr(node.func, "attr", None) == "read_frame"):
                sites.append((fn.name, node.lineno))
    assert len(sites) == 1, f"`read_frame` 调用点应当恰好一处, 实际: {sites}"
    assert sites[0][0] == "_reader_loop", (
        f"`read_frame` 的唯一调用点必须是 `_reader_loop`, 实际在 `{sites[0][0]}`")


def test_no_wildcard_fetch_point_remains():
    """`src/` 里不存在"什么都不要、但读到什么就吃掉什么"的取帧点。

    旧实现里 `want=None` 的通配取帧会让整条归属机制失效（`poll_cart` 就走过那条路）。
    """
    src = _ARM_PY.parent
    offenders = []
    for p in src.rglob("*.py"):
        for node in ast.walk(ast.parse(p.read_text(encoding="utf-8"))):
            if (isinstance(node, ast.Call)
                    and getattr(node.func, "attr", None) in ("_read_one", "_read_frames")):
                offenders.append(f"{p.name}:{node.lineno}")
    assert offenders == [], f"仍存在取帧口 (应为队列等待): {offenders}"


# ---------------------------------------------------------------------------
# 2. 归属判据 —— 同 id 不同命令**永不互通**
# ---------------------------------------------------------------------------

def test_an_ack_waiter_must_declare_which_command_it_waits_for():
    """等 `RSP_ACK`/`RSP_ERR` 时 `echo_cmd` **必填** —— 它就是归属判据。

    不给就等于"随便哪条 ACK 都算我的"，而那正是并发下两条命令互吃应答的成因
    （实测：到达序与等待序不一致时 **60/60 轮必有一方丢**）。
    ⚠ 库内 11 处调用点**全都给了**（`ast` 实测）⇒ 这里直接拒绝而不是退化成通配。
    """
    with pytest.raises(InvalidCommandError) as ei:
        _wait_keys(P.RSP_ACK, None)
    assert "echo_cmd" in str(ei.value)


def test_a_reply_goes_to_the_waiter_that_owns_it_not_the_first_to_read(offline_arm):
    """`ACK{0x11}` 落在 `(0x11)` 那条队列里，等 `ACK{0x10}` 的人**碰不到它**。

    形状：先塞一条别人命令的 ACK，再让"别人的读者"来取 —— 它取不到，而主人取得到。
    """
    arm = offline_arm
    _push(arm, P.RSP_ACK, bytes([P.CMD_DISABLE]))
    # 等 `enable` 的一方来取（"别人的读者"）—— 它不该拿到这条
    assert arm._a._wait(((P.RSP_ACK, P.CMD_ENABLE),), 0.05) is None
    # 主人拿得到（队列里那条还在）
    arm._a.expect(P.RSP_ACK, 0.3, "disable", echo_cmd=P.CMD_DISABLE)


def test_two_concurrent_commands_do_not_eat_each_others_replies(offline_arm):
    """**真两线程**：到达序与等待序相反时，两个等待者**都必须成功**。

    ⚠ 这是旧实现最要命的一条（实测 100% 有一方丢应答）。判据是"两条命令都被受理过、
    两条 ACK 也都到达过 ⇒ 两个等待者都该成功" —— 与谁先取无关。
    """
    arm = offline_arm
    _push(arm, P.RSP_ACK, bytes([P.CMD_DISABLE]))     # 到达序与等待序**相反**
    _push(arm, P.RSP_ACK, bytes([P.CMD_ENABLE]))

    got: dict = {}

    def waiter(echo: int, label: str) -> None:
        try:
            arm._a.expect(P.RSP_ACK, 0.5, label, echo_cmd=echo)
            got[label] = True
        except MotionTimeoutError:
            got[label] = False

    t1 = threading.Thread(target=waiter, args=(P.CMD_ENABLE, "enable"))
    t2 = threading.Thread(target=waiter, args=(P.CMD_DISABLE, "disable"))
    t1.start(); t2.start(); t1.join(); t2.join()
    assert got == {"enable": True, "disable": True}, f"有人丢了应答: {got}"


#: 一个**不存在**的命令码 —— 用它当"别人的读者"，保证与任何被测命令都不是同一条队列。
_OTHER_CMD = 0x7F


@pytest.mark.parametrize("cmd", [P.CMD_ENABLE, P.CMD_DISABLE, P.CMD_EMERGENCY_STOP,
                                 P.CMD_RESET, P.CMD_CLEAR_FAULTS])
def test_safety_command_replies_are_never_stolen(offline_arm, cmd):
    """安全命令（使能/失能/急停/复位/清故障）的应答是**最不能丢**的一族 —— 各钉一条。"""
    arm = offline_arm
    _push(arm, P.RSP_ACK, bytes([cmd]))
    # 一个"别人的"读者先来取（⚠ 键必须与被测命令不同，否则它就是在取自己那条）
    assert arm._a._wait(((P.RSP_ACK, _OTHER_CMD),), 0.05) is None
    arm._a.expect(P.RSP_ACK, 0.3, f"cmd {cmd:#x}", echo_cmd=cmd)


def test_a_rejection_is_still_visible_to_a_non_ack_waiter(offline_arm):
    """非 `ACK` 型等待（`want` 是某条 `RSP_*`）也必须看到**本命令的 ERR**。

    ⚠ 实测过的回归：不给这类等待带上 `(RSP_ERR, echo_cmd)`，一次**被拒**会退化成
    "无应答"超时（`model.probe()` 撞的就是这条）。判据用 `model.get_body` 的真路径。
    """
    arm = offline_arm
    arm._tr.err_override[P.CMD_GET_MODEL_PARAM] = 0x01
    with pytest.raises(Exception) as ei:
        arm.model.get_body(0)
    assert "被固件拒绝" in str(ei.value), (
        f"被拒必须报'被拒', 不能退化成超时: {ei.value!r}")


# ---------------------------------------------------------------------------
# 3. 陈旧帧 —— 不许冒充新应答（"假成功"是本包最忌讳的方向）
# ---------------------------------------------------------------------------

def _await_queued(arm, key, timeout: float = 1.0) -> bool:
    """等读线程把某条队列填上（用例要控制"帧已经躺在队列里"这个前提）。"""
    import time as _t
    end = _t.monotonic() + timeout
    while _t.monotonic() < end:
        if arm._a._queues.get(key):
            return True
        _t.sleep(0.002)
    return bool(arm._a._queues.get(key))


def test_a_stale_reply_already_in_the_queue_is_dropped_before_the_next_command(offline_arm):
    """发帧**之前**清本命令的应答队列（`_Ack.drain_for`，由 `_raw_write` 调）。

    形状：上一条 `reset` 已放弃，它的 ACK 已经**躺在队列里**；此时再发一条 `reset`
    —— 那条陈旧 ACK **必须被清掉**，本条只能报超时，**绝不能凭它报成功**。
    ⚠ 方向是承重的：清晚了会吃掉自己的应答（假超时，安全侧）；不清就是**假成功**（危险侧）。

    ⚠⚠ **残余窗口（知情的、不可再约）**：清队只清**队列**。一条还在**传输缓冲/线上**的
    陈旧应答，会在我们写完之后才被读线程取到 ⇒ 它躲过清队、被当成本次的应答。
    根因是线上**没有请求 id**（松灵也一样）。⇒ 用例必须先把帧**送进队列**再发命令，
    否则测的是那个残余窗口，不是清队本身。
    """
    arm = offline_arm
    _push(arm, P.RSP_ACK, bytes([P.CMD_RESET]))       # 陈旧 ACK 先到
    assert _await_queued(arm, (P.RSP_ACK, P.CMD_RESET)), "用例自身失效: 帧没进队列"
    # ⚠ 必须让桩**不再回答** `0x14`：否则这次成功可能来自固件那条**真** ACK，
    # 用例就证明不了"清掉了陈旧的那条"（判据失去判别力）。
    _orig = arm._tr.write_frame
    arm._tr.write_frame = lambda cmd, payload=b"": (
        None if cmd == P.CMD_RESET else _orig(cmd, payload))
    with pytest.raises(MotionTimeoutError):
        arm.reset()                                    # 不许凭陈旧帧"成功"


def test_the_drain_does_not_swallow_the_commands_own_reply(offline_arm):
    """清队排在**写之前** ⇒ 不可能吃掉自己的应答（写之前它还不存在）。

    ⚠ 这条是被实测逼出来的：早先的写法（在 `expect` 入口清队）会 **100%** 吃掉自己的
    应答 —— 桩上直接让 `connect()` 的固件握手超时。
    """
    arm = offline_arm
    arm.emergency_stop()          # 活着回来即证明没有吃掉自己的 ACK


# ---------------------------------------------------------------------------
# 4. 有界与计数 —— 丢必须看得见
# ---------------------------------------------------------------------------

def test_the_queue_is_bounded_and_counts_what_it_evicts(offline_arm):
    """每条队列封顶 `_QUEUE_MAX`；挤掉的帧必须计进 `_a.dropped`（**不静默**）。"""
    arm = offline_arm
    assert arm._a.dropped == 0
    for _ in range(_QUEUE_MAX + 5):
        _push(arm, P.RSP_TCP, b"\x00" * 8)     # 没人等 `(RSP_TCP, None)`
        arm._a._wait(((P.RSP_ACK, P.CMD_ENABLE),), 0.002)   # 驱动读线程把它投递掉
    assert arm._a.dropped >= 5, f"封顶丢帧没有计数 (dropped={arm._a.dropped})"
    assert len(arm._a._queues.get((P.RSP_TCP, None), [])) <= _QUEUE_MAX


def test_the_queues_are_cleared_with_the_session(offline_arm):
    """队列与 `_Ack` 同寿：`connect()` 重建 ⇒ 新会话看不到旧会话的残留。"""
    arm = offline_arm
    _push(arm, P.RSP_ACK, bytes([P.CMD_DISABLE]))
    arm._a._wait(((P.RSP_ACK, P.CMD_ENABLE),), 0.002)
    assert arm._a._queues.get((P.RSP_ACK, P.CMD_DISABLE))
    arm.reconnect()
    assert arm._a._queues.get((P.RSP_ACK, P.CMD_DISABLE)) is None, "旧会话的帧漏进新会话"


# ---------------------------------------------------------------------------
# 5. 守卫 —— 从前挂在 `_read_one`/`_request_and_get` 上，必须跟着搬家
# ---------------------------------------------------------------------------

def test_negative_timeout_is_rejected_not_treated_as_wait_forever(offline_arm):
    """`timeout < 0` 抛 `InvalidCommandError`。

    ⚠ 这是守卫搬家清单里最危险的一条：漏了它，**负 timeout 会变成"等到永远"**。
    """
    arm = offline_arm
    with pytest.raises(InvalidCommandError):
        arm._a._wait(((P.RSP_ACK, P.CMD_ENABLE),), -1.0)


def test_a_held_ack_reference_after_close_reports_not_connected(offline_arm):
    """`close()` 之后经**持有的 `_Ack` 引用**调 `expect` ⇒ `NotConnectedError`。

    ⚠ 不能退化成"等满超时"：那会把"链路已关"报成"固件不理我"。
    """
    arm = offline_arm
    a = arm._a
    arm.close()
    with pytest.raises(NotConnectedError):
        a.expect(P.RSP_ACK, 0.1, "after close", echo_cmd=P.CMD_ENABLE)


@pytest.mark.parametrize("entry", ["get_state", "get_status_now", "get_tcp"])
def test_arm_entries_still_raise_not_connected_after_close(offline_arm, entry):
    """`Arm` 级入口照旧**立刻**抛 `NotConnectedError`（它们先过 `_require()`）。"""
    arm = offline_arm
    arm.close()
    with pytest.raises(NotConnectedError):
        getattr(arm, entry)()
