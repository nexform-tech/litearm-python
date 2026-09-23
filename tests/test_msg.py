"""段三 Task 8: 返回信封 `Msg(value, hz, timestamp)` —— **破坏性变更**的判据。

11 个"读一帧"型 getter 的返回值从 `T` 变成 `Msg[T]`: `get_state` / `get_status_now` /
`get_tcp` / `get_ff_vec` / `get_ff_scalar` / `params.get_joint_param` / `model.get_body` /
`model.get_jm` / `model.status` / `model.get_gravity` / `diag.kin_bench`。

本文件钉四件事:

1. `value` 与**旧返回值逐字相同** (含 `None`: "取不到帧"从"返回 `None`"变成
   "`Msg.value is None`"), 且**只有**这 11 个被包;
2. `timestamp` 单调不减、与 `time.monotonic()` 同量纲;
3. `hz` 的**分母是实际到达的间隔** (`(count-1)/(last-first)`), 样本不足 2 条时写死 `0.0`
   —— 被动连续流 (RSP_STATUS) 与单发请求/应答式那 5 个共用这一条规则;
4. `hz` 的载体是**新造的 per-type 接收表**, **不是** `_Ack.unexpected_frames`
   (spec §6.6 第 6 轮订正: 两份不同的东西) —— 所以下面既有"状态帧给出 hz 而不涨
   未识别计数"那半, 也有"未识别帧涨计数但不是任何 getter 的 hz"那半。

⚠ 两个**派生** getter **刻意不包** (`get_ff_mask` 仍是裸 `int`、`all_joint_params` 仍是
`list[JointParam]`) —— 判据在第 5 节。
"""
from __future__ import annotations

import dataclasses
import time

import pytest

import litearm as pa
from litearm import _protocol as P
from litearm.model import ModelStatus
from litearm.params import JointParam

#: 一个**没人认领**的上行 id (与 `test_read_one.py` 同款) —— 只该进未识别计数。
UNKNOWN_ID = 0x7F


# ---------------------------------------------------------------------------
# 1. 信封形状 + 被包的恰好这 11 个
# ---------------------------------------------------------------------------

def test_msg_is_a_frozen_generic_dataclass_and_is_exported():
    assert "Msg" in pa.__all__, "新公开类型没进 __all__ ⇒ 使用者只能去摸私有模块"
    assert dataclasses.is_dataclass(pa.Msg) and pa.Msg.__dataclass_params__.frozen, (
        "Msg 必须是 frozen 的 (一帧的到达统计不该被调用方改写)")
    fields = [f.name for f in dataclasses.fields(pa.Msg)]
    assert fields == ["value", "hz", "timestamp"], f"字段名/顺序变了: {fields}"
    m = pa.Msg(value=1, hz=2.0, timestamp=3.0)
    with pytest.raises(dataclasses.FrozenInstanceError):
        m.hz = 9.0
    # 泛型参数要能用 (`Msg[RobotState]` 这种标注不该炸)
    assert pa.Msg[int] is not None


#: (名字, 调用, 期望值) —— 期望值**独立于被测代码** (取自桩自己的状态 / 解码类型),
#: 所以它真的是"`value` 与旧返回值逐字相同"的判据, 而不是"包了一层"的自证。
_WRAPPED = [
    ("get_state", lambda a: a.get_state(refresh=True),
     lambda a, m: list(m.value.q) == list(a._tr.q)),
    ("get_status_now", lambda a: a.get_status_now(),
     lambda a, m: list(m.value.q) == list(a._tr.q)),
    ("get_tcp", lambda a: a.get_tcp(),
     lambda a, m: m.value == pytest.approx(tuple(a._tr.pose))),   # 桩存的 pose 是 f32 回读
    ("get_ff_vec", lambda a: a.get_ff_vec(7),
     lambda a, m: len(m.value) == 7 and all(isinstance(v, float) for v in m.value)),
    ("get_ff_scalar", lambda a: a.get_ff_scalar(2, 0),
     lambda a, m: isinstance(m.value, float)),
    ("params.get_joint_param", lambda a: a.params.get_joint_param(0),
     lambda a, m: isinstance(m.value, JointParam) and m.value.idx == 0),
    ("model.get_body", lambda a: a.model.get_body(0),
     lambda a, m: len(m.value) == 10 and all(isinstance(v, float) for v in m.value)),
    ("model.get_jm", lambda a: a.model.get_jm(),
     lambda a, m: m.value == list(a._tr.model_jm)),
    ("model.status", lambda a: a.model.status(),
     lambda a, m: isinstance(m.value, ModelStatus)),
    ("model.get_gravity", lambda a: a.model.get_gravity([0.0] * 7),
     lambda a, m: len(m.value) == 7 and all(isinstance(v, float) for v in m.value)),
    ("diag.kin_bench", lambda a: a.diag.kin_bench(),
     lambda a, m: isinstance(m.value, pa.KinBenchResult)
     and m.value.raw == a._tr.kin_bench_text),
]


@pytest.mark.parametrize("name,call,same", _WRAPPED, ids=[c[0] for c in _WRAPPED])
def test_wrapped_getter_returns_a_msg_carrying_the_old_value(offline_arm, name, call, same):
    m = call(offline_arm)

    assert isinstance(m, pa.Msg), f"{name} 没返回 Msg (返回值类型变了两次?)"
    assert same(offline_arm, m), f"{name} 的 Msg.value 与旧返回值不一致"


def test_the_unwrapped_set_never_returns_a_msg(offline_arm):
    """⚠ **反向**判据: **不是**这 11 个的读口**不许**被包 (别顺手全包)。

    `movej`/`home` 返回的是"动作结果"(`RobotState`), 纯本地量 (`n`/`firmware`/
    `last_reset_reason`) 根本没有帧 —— 给它们包信封是**语义错位**。

    ⚠ **本用例只有这一半** —— "恰好这 11 个"的**正向**那一半在别处: 11 个 getter
    逐个返回 `Msg` 由上面 `_WRAPPED` 参数化的那两条钉住, 而**花名册**恰好是这 11 个由
    `test_the_wrapped_roster_pins_exactly_the_eleven_names` 钉住。
    ⚠ 本用例对"11 个里有谁被**解包**"**恒不敏感** (`assert not isinstance(raw, Msg)`
    对裸值恒真) —— 那件事由参数化那两条接住 (实测: 把 `get_tcp` 退回裸值 ⇒ 红在
    `test_wrapped_getter_returns_a_msg_carrying_the_old_value[get_tcp]`, 本用例照样绿)。
    ⚠ 名字也**别**再改回 "exactly the eleven": 本用例只断言"这 4 个没被包", 给它挂
    "恰好"三个字是没有判据支撑的 over-claim。
    """
    arm = offline_arm
    arm.enable()
    assert not isinstance(arm.movej([0.0] * arm.n), pa.Msg), "movej 是动作结果, 不该包"
    for local in ("n", "firmware", "last_reset_reason"):
        assert not isinstance(getattr(arm, local), pa.Msg), f"{local} 不该被包"


#: 被包的 getter **恰好这 11 个** (字面量 —— 与上面 `_WRAPPED` 各自独立地写一遍)。
_THE_ELEVEN = [
    "get_state", "get_status_now", "get_tcp", "get_ff_vec", "get_ff_scalar",
    "params.get_joint_param", "model.get_body", "model.get_jm", "model.status",
    "model.get_gravity", "diag.kin_bench",
]


def test_the_wrapped_roster_pins_exactly_the_eleven_names():
    """⚠ **花名册**判据: `_WRAPPED` 的名字**逐字**就是那 11 个 (顺序也钉住)。

    为什么单设一条: `_WRAPPED` 是上面两条 `parametrize` 用例**唯一**的驱动 —— 从它里面
    悄悄**删掉**一条, 两条用例就一起少跑一格 (参数化 id 也随之消失), 而**没有任何用例
    报错**: 实测删掉 `get_tcp` 那一项 ⇒ **全量 531 → 529 passed, 2 skipped 全绿**。
    `test_the_unwrapped_set_never_returns_a_msg` 只查**反向** (那 4 个没被包), 对花名册
    缩水**恒不敏感** ⇒ "恰好这 11 个"那半边此前**没有任何判据** (只有"正向逐个"那半边有)。

    本用例把花名册钉成字面量, 于是"被包的恰好是这 11 个" = 本条 (名字集合) ∧
    `test_wrapped_getter_returns_a_msg_carrying_the_old_value` (逐个 `isinstance Msg`)
    ∧ `test_the_unwrapped_set_never_returns_a_msg` (这 4 个不是) 三条合起来成立。
    """
    assert [c[0] for c in _WRAPPED] == _THE_ELEVEN, (
        "被包的 getter 花名册变了 —— 参数化那两条用例会跟着一起少跑/多跑, 而它们的 id "
        "变了也不会报错 (本用例是唯一的兜底)")


def test_home_still_returns_the_action_result(offline_arm):
    """`home()` 不在 11 个之内 (它等的是**动作结果**) —— 别顺手包。"""
    offline_arm.enable()                    # 固件 `!enabled` 回 ERR{0x2A,0x03}
    assert not isinstance(offline_arm.home(), pa.Msg)


def test_cached_state_path_also_returns_a_msg(offline_arm):
    """`get_state()` 的**缓存**分支照样回信封 (它不取帧, 但统计是 per-type 的)。"""
    arm = offline_arm
    arm.get_state(refresh=True)
    m = arm.get_state()                     # 这一次不取帧
    assert isinstance(m, pa.Msg) and m.value is not None


# ---------------------------------------------------------------------------
# 2. timestamp —— 单调不减
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("name,call,same", _WRAPPED, ids=[c[0] for c in _WRAPPED])
def test_timestamp_is_monotone_non_decreasing(offline_arm, name, call, same):
    """连调两次: 第二次的 `timestamp` **不许变小**。

    `>=` 而不是 `>`: `get_state()` 的缓存分支**不取帧** ⇒ 两次回的是同一帧的时刻。
    """
    a = call(offline_arm)
    time.sleep(0.002)
    b = call(offline_arm)

    assert b.timestamp >= a.timestamp, f"{name}: timestamp 回退了"
    assert b.timestamp > 0.0, f"{name}: timestamp 没被填 (哨兵 0.0 只表示'从没收到过')"


def test_timestamp_is_a_local_monotonic_reading(offline_arm):
    arm = offline_arm
    before = time.monotonic()
    m = arm.get_state(refresh=True)
    after = time.monotonic()
    assert before <= m.timestamp <= after, (
        "timestamp 必须是本机 time.monotonic() (段一到三全库用的都是它)")


# ---------------------------------------------------------------------------
# 3. hz —— 分母是"实际到达的间隔"
# ---------------------------------------------------------------------------

def test_recv_stats_formula_is_exact_on_a_synthetic_clock(offline_arm):
    """**公式的**判据 (确定性, 不靠真实时钟): `(count-1)/(last-first)`。

    直接喂 `_note_recv` —— 这是唯一能把"分母是什么"钉死的测法: 真实时钟下只能断言一个
    带宽度的区间。
    """
    a = offline_arm._a
    for t in (10.0, 10.5, 11.0, 11.5):       # 4 帧, 间隔 0.5s
        a._note_recv(P.RSP_FF_VEC, t)

    hz, ts = a.recv_stats(P.RSP_FF_VEC)
    assert hz == pytest.approx(3 / 1.5), "分母不是 (last-first) —— 公式变了"
    assert ts == 11.5, "timestamp 不是**最近一帧**的时刻"


def test_recv_stats_is_zero_below_two_samples_and_for_unseen_ids(offline_arm):
    a = offline_arm._a

    assert a.recv_stats(P.RSP_TCP) == (0.0, 0.0), "没收到过该类帧时该是 (0.0, 0.0) 哨兵"
    a._note_recv(P.RSP_TCP, 5.0)
    hz, ts = a.recv_stats(P.RSP_TCP)
    assert (hz, ts) == (0.0, 5.0), "一个样本定不出频率 ⇒ hz 写死 0.0, 但 timestamp 照给"


def test_hz_of_a_single_frame_getter_needs_a_second_call(offline_arm):
    """单发请求/应答式那几个: 第一次 `hz == 0.0`, 第二次起非 0。

    它们的"到达率"**就是调用方自己的轮询频率** (一次调用只到一个帧) —— 所以判据只能
    这么写, `hz > 0` 那条通不过第一次调用。

    ⚠ **`diag.kin_bench` 从前在名单里, 现在不在** —— 它的回执是**连续两帧**
    (耗时帧 + LINK 帧, 见 `diagnostics._KIN_BENCH_FRAMES`), 一次调用到达 2 帧。
    于是 `hz` 第一次就非 0, 且那个数**没有意义**: 分子被拆帧放大 (2 而不是 1),
    分母仍是调用间隔。**它对 `kin_bench` 从来就不是"到达率"** —— 收窄成单帧那会儿
    它恰好等于轮询频率, 那是巧合, 不是语义。
    """
    arm = offline_arm
    for name, call in [
            ("params.get_joint_param", lambda: arm.params.get_joint_param(0)),
            ("model.get_body", lambda: arm.model.get_body(0)),
            ("model.get_jm", lambda: arm.model.get_jm()),
            ("model.get_gravity", lambda: arm.model.get_gravity([0.0] * 7)),
    ]:
        assert call().hz == 0.0, f"{name}: 第一次调用 (只有一个样本) 的 hz 该是 0.0"
        time.sleep(0.002)
        assert call().hz > 0.0, f"{name}: 第二次调用之后 hz 仍是 0 —— 接收表没在记"


def test_hz_of_the_passive_status_stream_tracks_the_arrival_cadence(offline_arm):
    """被动连续流那一支: **到达节奏 20ms** ⇒ `hz` ≈ 50。

    ⚠ **[2026-09-22] 节奏的**来源**变了**：从前"20ms 一帧"靠**调用方**每 20ms 读一次
    造出来（那正是"读驱动时间"）；现在帧由**读线程**按桩的节拍收进来 ⇒ 节奏由
    `FakeTransport.auto_status_period` 决定。所以这里显式把它设成 20ms，判据（hz≈50）
    与区间含义都不变，变的是**谁决定节奏**。
    ⚠ 判据仍是**区间**而不是定值 (真实时钟下做不到确定): 上界 100 排除"写死一个 100Hz"
    那种假实现, 下界 10 容忍宿主机卡顿。
    """
    arm = offline_arm
    arm._tr.auto_status_period = 0.02          # ⇒ 50Hz 的被动流
    time.sleep(0.12)                            # 攒够样本

    hz = arm.get_state(refresh=True).hz
    assert 10.0 < hz < 100.0, f"状态帧的 hz={hz} 不在按 20ms 采样应有的区间里"


def test_hz_uses_only_arrivals_of_that_type(offline_arm):
    """`hz` 是 **per-type** 的: 别的帧到得再多, 也不该改这一类帧的统计。

    ⚠ 判据用 `recv_stats` 而不是"再调一次 getter 看 hz": 后者会把**自己**那一类的样本
    从 1 条加到 2 条, 于是无论 per-type 有没有失效都变成"hz > 0" —— 那是条假绿。
    """
    arm = offline_arm
    for _ in range(3):                       # 灌一堆 RSP_TCP 与状态帧
        arm.get_tcp()
    arm.get_status_now()

    assert arm._a.recv_stats(P.RSP_MODEL_JM) == (0.0, 0.0), (
        "别的帧类型污染了本类帧的统计 (per-type 失效)")
    assert arm._a.recv_stats(P.RSP_TCP)[0] > 0.0, "RSP_TCP 自己反倒没统计上"

    arm.model.get_jm()                       # 真调一次: 现在只有**这一类**动了
    time.sleep(0.002)
    assert arm.model.get_jm().hz > 0.0


# ---------------------------------------------------------------------------
# 4. hz 的载体**不是** `unexpected_frames` (§6.6 第 6 轮订正)
# ---------------------------------------------------------------------------

def test_status_frames_produce_hz_while_the_dropped_counter_stays_zero(offline_arm):
    """状态帧给出 `hz`, 却**永远不会**让 `dropped` +1 —— 它走单槽、不进队列。"""
    arm = offline_arm
    arm.get_state(refresh=True)
    time.sleep(0.002)
    m = arm.get_state(refresh=True)

    assert m.hz > 0.0, "状态帧的 hz 是 0 ⇒ 统计没发生"
    # ⚠ 状态帧走**单槽**（不进队列）⇒ 它永远不会被封顶挤掉。
    assert arm._a.dropped == 0, "状态帧被计成了丢弃"


def test_unrecognized_frames_are_queued_not_dropped_and_get_their_own_hz(offline_arm):
    """未识别帧进**它自己那条队列**等着（不丢), 在接收表里也只记在**它自己**那个 id 下。

    ⚠ 重构把 `unexpected_frames` / `foreign_frames` 合并成 `dropped`，且**只在队列封顶时**
    递增 —— 一条没人认领的未知帧**进队列等着**，不计丢弃。「不丢」才是设计。
    """
    arm = offline_arm
    for _ in range(3):
        arm._tr._push(P.pack_frame(UNKNOWN_ID))

    for _ in range(200):                         # 等读线程把它们收进来
        if len(arm._a._queues.get((UNKNOWN_ID, None), [])) >= 3:
            break
        time.sleep(0.002)

    assert len(arm._a._queues.get((UNKNOWN_ID, None), [])) == 3, "未识别帧没进队列"
    assert arm._a.dropped == 0, "没人认领 ≠ 丢弃 (队列远没到封顶)"
    assert arm._a.recv_stats(UNKNOWN_ID)[1] > 0.0, "接收表没记它 (两份统计应当各记各的)"
    assert arm._a.recv_stats(P.RSP_MODEL_JM) == (0.0, 0.0), (
        "未识别帧漏进了某一类真帧的 hz")


def test_normal_replies_of_ids_outside_known_rsp_ids_are_not_counted(offline_arm):
    """`RSP_GRAVITY(0x57)` 的**正常应答**进它自己那条队列、被主人取走 ——
    既不计丢弃, 也照记进接收表 (`hz` 非 0)。"""
    arm = offline_arm
    arm.model.get_gravity([0.0] * 7)
    time.sleep(0.002)
    m = arm.model.get_gravity([0.0] * 7)

    assert m.hz > 0.0, "RSP_GRAVITY 没进接收表"
    assert arm._a.dropped == 0, "正常应答被计成了丢弃"


def test_the_reception_table_records_every_arriving_frame_including_0x4e(offline_arm):
    """`0x4E` 被收集器认领**之前**就已经进接收表 —— 统计的是"到达", 与归谁认领无关。"""
    arm = offline_arm
    assert P.RSP_CART_PLAN not in arm._a._recv
    arm._cart.register()
    arm._tr._resp.clear()
    arm._tr.push_frame(P.RSP_CART_PLAN, bytes([1, 0]) + (12).to_bytes(2, "little")
                       + (4700).to_bytes(4, "little"))

    # ⚠ `poll_cart()` 不再驱动取帧（读线程是唯一读者）⇒ 必须先等读线程把它投递出去，
    # 再让 `poll_cart` 去收集器认领。少了这一步，测的是"谁先跑"而不是接收表。
    for _ in range(200):
        if P.RSP_CART_PLAN in arm._a._recv:
            break
        time.sleep(0.002)

    assert arm.poll_cart() is not None, "桩这条规划应答没被认领 (用例前提不成立)"
    assert P.RSP_CART_PLAN in arm._a._recv, (
        "被收集器吃掉的帧没进接收表 —— 说明记录排在收集器分支之后了")


def test_the_reception_table_resets_with_the_session(offline_arm):
    """接收表与 `_Ack` 同寿 ⇒ `reconnect()` 之后统计从零开始 (不是跨会话累计)。

    ⚠ 判据不能写成"重连后第一次读的 `hz == 0.0`": `connect()` 的握手**自己就会**读进
    一帧状态, 而两帧挨得极近时 `hz` 是个几千的数 (公式没错, 是样本太少) —— 那种写法
    会把一个正常行为判成缺陷。
    """
    arm = offline_arm
    for _ in range(4):
        arm.get_state(refresh=True)
        time.sleep(0.002)
    assert arm._a.recv_stats(P.RSP_STATUS)[0] > 0.0, "前提: 旧会话攒到了样本"
    old_table = arm._a._recv

    arm.reconnect()

    assert arm._a._recv is not old_table, "接收表跨会话复用了 (统计没跟着会话走)"
    # ⚠ 判据从「`hz == 0.0`」改成「**计数从 1 重新起**」：读线程在 `connect()` 里就起来了，
    # 它会在我们检查之前就投进若干状态帧 ⇒ `hz == 0.0` 与读线程**竞态**（实测红）。
    # "统计从零开始"的确定性表述是：新表的 count **小于**旧表攒到的数。
    assert arm._a._recv[P.RSP_STATUS][0] < old_table[P.RSP_STATUS][0], (
        "换了一条会话, 计数该从 1 重新起 (不是跨会话累计)")


# ---------------------------------------------------------------------------
# 5. 两个派生 getter 仍是裸值 (`get_ff_mask` / `all_joint_params`)
# ---------------------------------------------------------------------------

def test_get_ff_mask_stays_a_bare_int(offline_arm):
    """`get_ff_mask` 是 `get_ff_scalar(9, 0)` 的**标量投影** —— 包两层会把同一帧的同一
    份统计说两遍, 而且它的定义 `int(round(...))` 本身就只能消费裸值。"""
    got = offline_arm.get_ff_mask()
    assert isinstance(got, int) and not isinstance(got, pa.Msg)


def test_all_joint_params_stays_a_list_of_jointparam(offline_arm):
    """N 次往返的**聚合** —— 一个 `hz`/`timestamp` 描述不了它 (N 帧 N 个时刻),
    且逐项读字段的既有契约 (`for jp in ...: jp.q_min`) 不许破。"""
    arm = offline_arm
    got = arm.params.all_joint_params()
    assert len(got) == arm.n
    assert all(isinstance(jp, JointParam) for jp in got), f"returned list[Msg]? {got!r}"


def test_the_scalar_projection_still_shares_the_underlying_frame_stats(offline_arm):
    """要那一帧的信封就直接调 `get_ff_scalar(9, 0)` —— 两条路的**值**必须一致。"""
    arm = offline_arm
    assert arm.get_ff_mask() == int(round(arm.get_ff_scalar(9, 0).value))
