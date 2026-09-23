"""反向守卫: **笛卡尔在途时拒绝进入零重力** (`zero_g()` / `zero_g_start()`)。

规格 §5.2(4) 的两条理由 (缺一不可):

1. 固件 `0x06` 丢掉位置环、只剩重力前馈 —— 轨迹中途进场会让臂**靠摩擦滑停
   (coast)**, 比 `movej()` 的受控接管 / `emergency_stop()` 差得多。拦下来并把
   替代动作写进消息 = 把操作员推向**更好的停机动作**。
2. 它堵掉吸收额度的残余: 保活线程每 40ms 一条 `0x06` 在清队集合里, "笛卡尔在途 +
   零重力保活"会让清队拿到 `n=1` 的额度并存活 `move_timeout`, 那条额度可能吞掉
   下一条**合法**应答。拦住这个交错 ⇒ 该残余在生产路径上不再可达。

⚠ **方向相反的另一侧不拦**: `movej`/`movej_sync` 是**受控接管** (新 S 曲线从当前
状态收口), 拦它等于把人推去用更重的 `emergency_stop`。只做 `zero_g` 这一侧。
⚠ **`zero_g_stop()` 不拦** (退出动作, 降能量方向必须永远可达), 同理
`emergency_stop()` / `disable()` / `get_tcp()`。

判据用**两条信号** (见 `Arm._reject_if_cart_in_flight`): ① `_cart_serial` 被持有;
② **现取**的状态帧 bit10。只用①漏 `wait=False`, 只用②漏"刚登记还没起规划"。
"""
from __future__ import annotations

import struct
import threading

import pytest

from litearm import _protocol as P
from litearm.errors import InvalidCommandError, LiteArmError, MotionTimeoutError

#: 与 `test_cart_protocol.py` 同款的目标位姿 (桩的初始 TCP 就是它)
TCP0 = (0.30, 0.0, 0.35, 0.0, 0.0, 0.0)


def _zg_writes(arm):
    """桩收到的 0x06 载荷序列 (守卫失败时必须是空集 —— 一帧都不该发)。"""
    return [p for c, p in list(arm._tr.tx_log) if c == P.CMD_ZERO_G]


def _enter_busy_window(arm):
    """把桩摆到「0x4E 已回、规划仍在跑」那一刻。

    桩的 `CART_BUSY` 窗口写死为 `0x4E` 之后的**第 2..7 帧** (`fake_serial._stamp_status`),
    而 `cart_busy_seq` 每次受理时归 0 —— 于是紧接着的第一帧恰好不带 bit10。
    置 1 即"下一帧就落在窗口内", 也就是 `wait=False` 返回之后臂仍在跑的真实时刻。
    """
    arm._tr.cart_busy_seq = 1


# --------------------------------------------------------------- 判据 ①
def test_zero_g_rejected_while_a_cartesian_call_holds_the_serial_lock(offline_arm):
    """① 串行锁被持有 ⇒ 有一次笛卡尔调用正在途 (三条入口全程持它)。"""
    arm = offline_arm
    holding = threading.Event()
    release = threading.Event()

    def holder():
        with arm._cart_serial:            # 模拟一条正在途的 move_l/move_c/move_path
            holding.set()
            release.wait(5.0)

    th = threading.Thread(target=holder, name="cart-in-flight", daemon=True)
    th.start()
    try:
        assert holding.wait(5.0), "辅助线程没拿到锁 —— 用例没测到要测的东西"
        before = list(arm._tr.tx_log)
        with pytest.raises(InvalidCommandError) as ei:
            arm.zero_g()
        msg = str(ei.value)
        assert "零重力" in msg
        assert "movej" in msg or "emergency_stop" in msg, \
            f"只拒绝而不给出路会把操作员卡住: {msg}"
        assert arm._tr.tx_log == before, "守卫拦下之前就写了帧 (0x06 必须在写之前拦)"
        assert arm.zero_g_active is False, "被拒的 zero_g 不该留下 active 状态"
    finally:
        release.set()
        th.join(5.0)


# --------------------------------------------------------------- 判据 ②
def test_zero_g_rejected_after_wait_false_returned_with_the_arm_still_running(offline_arm):
    """② `wait=False` 已返回 (锁已释放) 但臂仍在跑 ⇒ 只看锁会放行, 必须靠现取的 bit10。

    这正是计划点名的那条坑, 也是真机 V3 唯一走得通的形状 (`wait=True` 时调用方
    阻塞在到位等待里, 根本调不到 `zero_g()`)。
    """
    arm = offline_arm
    arm.move_l(TCP0, speed=0.3, wait=False)
    _enter_busy_window(arm)

    # 锁**确实**空闲 —— 否则本用例就退化成判据 ①, 测不到 bit10 这一条
    assert arm._cart_serial.acquire(blocking=False) is True, "锁被持有, 用例没测到 bit10"
    arm._cart_serial.release()
    # ⚠ 此刻 `_a.state` 还是**旧**帧 (bit10=0): 拿它当判据就会假放行 —— 守卫必须现取。
    #   这一行同时是"新鲜采样"那一条的判据 (用陈旧帧的实现会在这里放行)。
    assert arm._a.state is not None and arm._a.state.cart_busy is False, \
        "用例前提不成立: 陈旧帧没有停在 bit10=0, 分辨不出'现取'与'用旧帧'"

    with pytest.raises(InvalidCommandError) as ei:
        arm.zero_g()
    assert "零重力" in str(ei.value)
    # ⚠ 判据只钉 `0x06`: 守卫自己会发一条 `0x40` (现取状态帧), 那是它的手段不是副作用
    assert _zg_writes(arm) == [], "守卫拦下之前就把 0x06 发了出去"
    # 事后复核: 那一刻固件确实报忙 (否则本用例是空跑)
    assert arm.get_status_now().value.cart_busy is True, "桩此刻没报忙, 用例没测到 bit10"

    # 反向对照: 桩报停稳时同一条调用必须放行 (否则守卫会把正常示教拦死)
    arm._tr.cart_busy_seq = None
    with arm.zero_g():
        assert arm.zero_g_active is True


# --------------------------------------------------------------- 判据 ③
def test_exit_and_energy_down_actions_are_never_blocked(offline_arm):
    """③ 退出/降能量/只读动作在**同样条件下**一律不被拦。

    守卫若被挂到 `_write_cmd`/`_raw_write` 上 (而不是 `zero_g_start` 里), 这四条会
    一起被挡 —— 那是把安全动作变成不可达, 方向错得比漏拦更严重。
    """
    arm = offline_arm
    arm.move_l(TCP0, speed=0.3, wait=False)
    _enter_busy_window(arm)
    holding = threading.Event()
    release = threading.Event()

    def holder():
        with arm._cart_serial:
            holding.set()
            release.wait(5.0)

    th = threading.Thread(target=holder, name="cart-in-flight", daemon=True)
    th.start()
    try:
        assert holding.wait(5.0)
        arm.zero_g_stop()                       # 退出动作 (且幂等)
        arm.zero_g_stop()
        arm.emergency_stop()                    # 降能量方向
        arm.disable()
        assert arm.get_tcp().value is not None        # 只读查询
        assert _zg_writes(arm) == [], "从未进入零重力却发出了 0x06"
    finally:
        release.set()
        th.join(5.0)


# ------------------------------------------------- 取不到状态 ⇒ 保守拒绝
def test_status_that_cannot_be_confirmed_is_a_conservative_refusal(offline_arm, monkeypatch):
    """取不到状态帧 ⇒ **保守拒绝** —— "未确认就不拦"漏过去的正是 coast 那一侧。"""
    arm = offline_arm
    # (a) 等状态帧时**读到一条 ERR** —— 用桩的 `unknown_cmds` 造出来 (它照 `fake_serial.py`
    #     建模固件 `usb_cmd.c` 的 `default` 分支: 回 `ERR{cmd,0x00}`)。
    # ⚠ 别把这句读成"固件对 0x40 回 ERR": 真固件的 `CMD_GET_STATUS`
    #   (`usb_cmd.c:566-568`) 只把 RAM 里的 `g_arm` 打包 (`usb_cmd_report_status()`),
    #   **没有失败路径、没有门禁/长度校验** —— 它**永远回 `RSP_STATUS`、永不回 `ERR`**。
    #   真能产生 `ERR{0x40,·}` 的只有一根: 固件的 `default` 分支 (那版固件里根本没有这条
    #   命令) —— 也就是上面桩建模的那一支。另一支能走到这里的是**别的命令的迟到 `ERR`
    #   串台**, 但那种 ERR 带的是**别的**命令码, 不是 0x40。两支在这儿殊途同归:
    #   `get_status_now` 对**任意** `RSP_ERR` 都抛 (它自己那一句 `raise _err_from(...)`,
    #   `arm.py:655`; `_err_from` 的**定义**在 `arm.py:134` —— 别引成 `_read_status` 里
    #   那处 `:627`。这条路径**没有** `expect` 那层 `echo_cmd` 过滤), 于是同样落到
    #   "保守拒绝"。
    #   本用例测的是**那条拒绝路径**, 不是固件真会这么回。
    arm._tr.unknown_cmds.add(P.CMD_GET_STATUS)
    with pytest.raises(InvalidCommandError) as ei:
        arm.zero_g()
    assert "保守拒绝" in str(ei.value), str(ei.value)
    assert _zg_writes(arm) == [], "状态未确认却仍然进了零重力"
    arm._tr.unknown_cmds.discard(P.CMD_GET_STATUS)

    # (b) 超时: 连状态帧都收不到
    def _timeout(self, timeout: float = 0.5):
        raise MotionTimeoutError("桩: 故意超时")

    monkeypatch.setattr(type(arm), "get_status_now", _timeout)
    with pytest.raises(InvalidCommandError) as ei2:
        arm.zero_g()
    assert "保守拒绝" in str(ei2.value), str(ei2.value)
    assert _zg_writes(arm) == [], "状态未确认却仍然进了零重力"


def test_extra_0x4e_desync_is_not_masked_as_unconfirmed(offline_arm):
    """⚠ 例外里**不能**兜整个 `LiteArmError` —— "多了一条 `0x4E`" 抛的是基类。

    那条的含义是"固件与主机**已经**错配", `cart.py` 刻意让它响亮 (连专用异常类型都
    刻意不给, 免得后人 `except` 掉)。守卫把它改写成"取不到状态、保守拒绝"就等于
    把协议失步藏进一句常规拒绝里 —— 方向与那处刻意相反。
    """
    arm = offline_arm
    arm._tr.push_frame(P.RSP_CART_PLAN, bytes([1, 0]) + struct.pack("<HI", 3, 100))
    with pytest.raises(LiteArmError) as ei:
        arm.zero_g()
    assert not isinstance(ei.value, InvalidCommandError), \
        f"协议失步被伪装成了常规拒绝: {ei.value}"
    assert "错配" in str(ei.value), str(ei.value)

