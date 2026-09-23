"""笛卡尔固件规划的**应答配对** —— `0x4E` 与在途请求的 FIFO 配对。

固件原生笛卡尔 (`0x3A/0x3B/0x3E`) 的应答载荷里**没有命令 id**, 只能按**受理顺序**
FIFO 配对; 而固件那份应答只有**一个槽位** (`plan_pending` 单 bool + 单份载荷):
同一个 main 排空窗口内登记 ≥3 条时, **中段请求零应答** —— N 条只存活
「第一条被取代者的 CANCELED」+「最后一条的结果」= 2 条。于是两条守卫**缺一不可**:

* **多了一条** 分两支。清队**销毁在途记账**, 于是后来那条 `0x4E` 归属不明 —— 固件的应答
  是在**规划完成时**就推上 USB 的, 而主机清队发生在**我们写那条清队 opcode 的那一刻**;
  两者之间隔着 USB 延迟。所以"清队那一刻固件那条规划**已经跑完**、应答早躺在主机 RX
  缓冲里"是**可达**的一支, 它属于**刚被我们清掉的那条请求**。而 `0x4E` 载荷里**没有命令
  id**, "这条应答属于被作废那条还是新发那条"**在原理上无法区分** ⇒ 任何 FIFO 方案必有
  一支判错, **只能选判错得安全的那一支: 宁可报"结局未知", 绝不报"成功"**:
    - **清队后的不可归属应答** (吸收额度 > 0) → 计数 + **吸收** (`absorbed_replies`),
      不报错 (报错会让 `LiteArmError` 从毫不相干的读路径里炸出来, 并把一条**已经成功**
      的运动报成"结局未知" —— abort/急停路径天生是"清队紧跟笛卡尔命令"的形状);
    - **真·脱同步** (额度耗尽且队列空) → 计数 + **报错**, **不静默丢弃**。
* **少了一条** (token 超时没收尾) → 摘除该 token 并抛 :class:`CartReplyLostError`
  —— 缺了这条, 后续的 `0x4E` 会被错配给一条早已被吞掉的请求。

⚠ **串行是强制的 (不是建议)**, 由 `Arm._cart_serial` 那把锁实现 —— 上面两条守卫
**都只在串行下成立**, 同时 ≥2 条在途时它们会一起判错:

* **顺序必须有唯一来源** (最根本的一条, 也是锁不可省的理由): `_request_and_wait` 是
  **先登记、后写**, 而配对按**受理顺序** FIFO ⇒ 没有锁时"两条线程的登记顺序"与"固件
  的受理顺序"可以**倒置** (线程在登记之后、写之前被抢占) ⇒ 两条应答各自配给**对方**
  那条 token; `0x4E` 载荷里没有命令 id, 原理上不可区分。一侧失败、另一侧被报成成功
  (即**假成功**), 或两条都报成功而规划结果互换。**这一条 `err_waits_for_ack` 挡不住**
  (它挡的是"认错帧", 不是"顺序倒置")。
* **两条 `expect` 会互相吃帧**: `RSP_ERR`/`RSP_ACK` 的回显只有**命令码**
  (`expect(echo_cmd=...)` 只能按码比), 而 `0x3A/0x3B/0x3E` **共用码空间** ⇒ 后受理那条
  被门禁拒的 `ERR` 可能被**先受理那条**的 `expect` 读走, 而先受理者的 `ACK` 也被后受理者
  读走 ⇒ **被拒那条报的不是它自己那条 `ERR`**, 而是耗满窗口的 `MotionTimeoutError` /
  `CartReplyLostError` (报告与物理事实相反; 实测见
  `test_cart_protocol.py::test_two_concurrent_requests_cannot_cross_deliver`, 那条用例
  现在**靠"逐线程归因"那一条判据咬住** —— 前两条在 `err_waits_for_ack` 之后已不再区分)。
  ⚠ 这一支的**旧**形态 ("先受理者摘掉自己的 token ⇒ 后受理者拿到它那条 `0x4E` 而报成功")
  被 `err_waits_for_ack` 消掉了, 别再照旧文档复述。

单槽 pending 决定了"流水线发多条"还会额外踩上丢应答路径。三条入口的
`_request_and_wait` **全程持锁** (登记 → 等 ACK → 等 `0x4E` → (`wait=True` 时) 等停稳),
代价是持锁期间可能阻塞到 `move_timeout` —— `move_path` 另把 BEGIN/ADD 也圈进同一把锁
(见 `move_path`)。

⚠ **线程安全**: 零重力保活线程 (`Arm._zg_keepalive`) 会并发调用
`clear_pending()` —— 本模块的全部状态变更都在自己的锁内。

本模块的两半:

* **配对** (`_CartPending` / `_CartToken`) —— 只持有"谁在等哪条应答"这一个事实;
* **结果语义** (`CartPlan` / `cart_err_t` 映射 / 三条入口 / 能力探测) —— 见文件末。
  三条入口 (`move_l` / `move_c` / `move_path`) 的**全部守卫都排在 `request()` 之外**
  (理由见 `_CartPending.request` 的 docstring: `request()` 只在 `TransportError` 上
  摘 token, 其余异常一律原样 re-raise 且**不动队列** —— 写之前抛的非 `TransportError`
  异常会把 token 留在队里)。
"""
from __future__ import annotations

import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional

from litearm import _protocol as P
from litearm._rot import as_pose, mat_to_rpy
from litearm.errors import (CartReplyLostError, CartesianPlanError,
                                 CommandRejectedError, IKError,
                                 InvalidCommandError, LiteArmError,
                                 MotionSupersededError, MotionTimeoutError,
                                 MotorFaultError, TransportError,
                                 UnsupportedByFirmwareError)

__all__ = ["CartPlan"]

#: 会**静默作废**在途笛卡尔规划的 opcode (固件侧走 `cart_invalidate_before_motion()`
#: → `cart_abort()`, **一条 `0x4E` 都不发**)。
#:
#: 逐条依据 (固件运行路径的调用点): `ctrl_accept_move_j` (0x01, 且 0x2A home 复用同一处)
#: / `kin_runner_request_move_p` (0x02) / `ctrl_accept_move_js` (0x03) /
#: `ctrl_accept_move_mit` (0x04) / `ctrl_accept_move_mit_all` (0x05) /
#: `ctrl_accept_zero_g` (0x06) / `ctrl_accept_move_j_sync` (0x07) / `ctrl_disable` (0x11) /
#: `ctrl_emergency_stop` (0x12) / `ctrl_clear_faults` (0x13) / `ctrl_reset` (0x14)。
#:
#: ⚠ `0x3A/0x3B/0x3E` **不在**此列 —— 它们欠应答, 而且**欠两条**: 被取代的那条在
#: PLANNING 期由 `cart_begin_request` 补一条 CANCELED, 新受理那条随后还有自己的结果。
#: 清队会让报告与物理事实**相反**, 且**两个方向都反** (机制由
#: `tests/test_cart_queue.py::test_the_clear_set_must_not_swallow_the_two_known_replies`
#: 实地摆出来):
#:
#: ① **被摘掉的不止在飞那条**: `_request_and_wait` 是**先登记、后写**, 而清队钩子挂在
#:    写口 (`_raw_write`) 上 ⇒ 新受理那条的 token 刚登记、还没写出去就被**一起摘掉**
#:    (实测 `clear_pending` 返回 2)。于是两条**都**被判"结局未知"
#:    (`CartReplyLostError`) —— 而真相是"A 被 B **预期内**地接管 (不是失败)"与
#:    "B 马上就要跑";
#: ② **随后那两条应答全被吸收**: 2 格额度正对 2 条应答 (A 的 CANCELED + B 的结果)
#:    ⇒ 它们被当成"清队前就已发出的迟到应答"吞掉, **B 那条本该交付的规划结果被丢弃**。
#:
#: 一句话: 这 3 条 opcode 的应答是**已知**的, 清队把已知的东西报成了未知。
#:
#: ⚠ 上面这 ①② **不是**"调用方会收到 `MotionSupersededError`" —— 那个类型唯一的产出
#: 点是**读到** `err=5` 的 `0x4E` (见 :func:`raise_for_plan`), 而这里两条应答都被吸收,
#: 谁也没读到它。写"B 会收到接管错误、于是不去纠正"是**后果链说反** (旧版本文档就是
#: 这么写的); 真实代价是"**两条已知结局都退化成未知** + B 的结果被丢"。
#:
#: ⚠ 这张表只解释"**少发**"的一半: 突发下 N≥3 的吞应答**不经过**
#: `cart_invalidate_before_motion()` (是 `cart_begin_request` 的 `unreported` 分支吞的),
#: 那半边由 :meth:`_CartPending.wait` 的"少了一条"守卫兜住 ——
#: **别以为"没在这张表里 ⟹ 必有应答"**。
#:
#: ⚠ `0x2A` home 最易漏 (它经 `ctrl_accept_move_j` 受理, 与 `0x01` 同一处调用点)。
#:
#: ⚠ **为什么是 12 而不是 13** (2026-09-19 复核): 按**代码位置**数 `cart_invalidate_before_motion()`
#: 的调用点, `control_loop.c` 有 11 处 + `kin_runner.c:176` 共 12 处, 可其中
#: `control_loop.c:655` 那处在 `ctrl_accept_move_p` (`:654`) 里 —— 该函数**在固件运行
#: 路径上无人调用** (0x02 走的是 `kin_runner_request_move_p`, `usb_cmd.c:313`; 该函数的
#: 全部引用只有声明 `control_api.h:38`、定义本身、以及宿主自检工具
#: `tools/ctl_loop_check_host.c:1750` 的直调 —— 那是测试夹具, 不是运行路径), 即**不对应
#: 任何 opcode**; 而 `0x2A` 复用 `0x01` 的调用点、**没有自己的调用点**。
#: 两个 off-by-one 相抵 ⇒ 12 处调用点 ↔ **12** 个 opcode (不是 12+1=13, 也不是 12-1=11)。
#: 两条易错梗都记在这里, 免得后来者再数一遍得出 13。
_CART_CLEARS_UPON = frozenset({
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,          # move_j/P/js/mit/mit_all/zero_g/j_sync
    0x11, 0x12, 0x13, 0x14, 0x2A,                      # disable/estop/clear_faults/reset/home
})


#: "已收到应答但没人认领"那一列 (`_CartPending._unclaimed`) 的上界 —— 超出即丢**最旧**的。
#:
#: **为什么必须有界**: 调用方放弃 token 的那条路径 (ACK 超时, 见
#: `_request_and_wait`) 每放弃一条就在那一列永久留下一条 —— 固件其实已受理, 迟到的那条
#: `0x4E` 会把它配进去, 而它的调用方再也不会来 `wait`。
#:
#: **为什么这个界是安全的** (三条, 缺一不可):
#:   · 那一列里的结果**只**能由 `Arm.poll_cart()` 认领, 而它一次只认领一条 —— 积压超过
#:     几条本身就说明没有人在取 (三条入口成功时各 `wait` 一次, 正常用法下那一列恒为空);
#:   · 被丢的只是那些请求的**规划结果** (`CartPlan`), 它们对应的调用方**早已**拿到异常
#:     放弃了; 真要知道臂现在什么样, 该回读 `get_state()`/`get_tcp()`, 而不是拿一条陈旧
#:     规划去推断;
#:   · 丢弃**不碰** `wait` 的路径 (它走 token 自身的应答, 与这一列无关) ⇒ **不制造假成功**。
#: 越界条数记进 `_CartPending.evicted_unclaimed` (丢弃本身不报错, 见 `on_reply` 的理由)。
_UNCLAIMED_MAX = 8

#: 吸收额度的**存活下限** (秒) —— `Arm` 传进来的 `absorb_ttl` 取
#: `max(move_timeout, 本值)`, 而不是直接拿 `move_timeout`。
#:
#: **为什么必须有下限**: 额度是"清队/放弃那一刻, 那条请求**可能**还欠一条应答"的通行证。
#: 额度**过期**而那条应答**还在路上**时, 它一到就撞上 `on_reply`: 队列若**非空** (清队后
#: 又登记了新请求), 它被**配给那条新 token** ⇒ 调用方拿到**别人那条**的规划结果并**报
#: 成功** (E3 实测, 见 `test_a_reply_that_outlives_move_timeout_is_not_paired_to_the_next_request`);
#: 队列若空, 报的是 `LiteArmError` "固件与主机已错配" (**归因误导** —— 真相只是"我们自己
#: 放弃了一条请求, 而额度先过期了")。故额度的存活时间必须覆盖
#: "**固件还可能发出那条应答**"的最长时间。
#:
#: **固件依据** (不是拍脑袋的常数) —— 判据要覆盖的是**应答到达**的时刻, 所以是三段相加,
#: 每段各自有出处。⚠ 只算第 ① 段是不够的 (这条注释**曾经**就只算第 ① 段):
#:
#:   ① **生成** (`≤ 写帧 + 3 s`): `RSP_CART_PLAN` 的**唯一发射点**是 `cart_report_if_done()`
#:      (`usb_cmd.c:1246`; `usb_cmd_reply(RSP_CART_PLAN, out, 8)` 在 **`:1261`**), 由
#:      **main 循环**每圈调一次 (`Core/Src/main.c:187`) —— **不在** `cart_publish()` 里
#:      (`cart_exec.c` 不引用 `usb_cmd`, 它只置 `plan_pending`)。
#:      规划期的硬上界是 `CART_PLAN_MAX_TICKS` = `LITEARM_CTRL_HZ * 3u` = **3 s**
#:      (`cart_exec.h:42` —— 固件刻意把"心跳豁免窗口"与"强制 abort 点"收口在这**一处**
#:      定义), 到点 `cart_tick_timeout` 会 `cart_abort()` (`cart_exec.c:890`) 并补发一条
#:      终态应答 (**`:917`** 那句 `plan_pending = true`, 整段见 `:889-917`) ⇒ 此后不会
#:      再有应答。请求的受理时刻 ≈ 我们写下那一帧的时刻 ⇒ 结果**生成**于 `写帧 + 3s` 内。
#:      ⚠ `plan_pending` 一共**三处**置位, 别只记 `cart_publish`: `cart_begin_request`
#:      (`cart_exec.c:256`, 为被取代者补一条 CANCELED)、`cart_publish` (`:414`)、
#:      `cart_tick_timeout` (`:917`)。
#:   ② **发射时延** (`≤ ~8 s`): "结果生成" ≠ "已上 USB" —— 它要等 main 循环轮到
#:      那一句, 而**同一圈里排在它前面**的 `usb_cmd_report()` (`Core/Src/main.c:179`) 可能
#:      正卡在参数保存的阻塞擦写上: 固件自述"擦写 **~1s** CPU 全停"(`usb_cmd.c:1272`,
#:      并专门给它开了心跳监督豁免 `g_param_save_active` (`usb_cmd.c:1277`)),
#:      同一件事在另两处按 **~1.86s** 记 (`usb_cmd.c:690` / `usb_cmd.h:94`)。
#:      ⚠⚠ 这两个都只是**典型值, 不是上界** —— 拿典型值当上界, 余量就留在了**假成功**
#:      那一侧 (见下面"两个方向")。同一个现象 (整扇区擦除期间 CPU 取指停顿 ⇒ 排在它后面的
#:      那一句发不出去) 在固件里还有**第三个量纲的自述, 而且它是唯一的上界**:
#:      `hw_watchdog.h:28` —— "单 bank H723 整扇区擦除期间 CPU 取指停顿, TIM3 无法喂狗;
#:      擦写前放宽到 **~8 s**, 完成后恢复" (实现: `hw_watchdog.c:63-66` 把 IWDG 重装成
#:      `IWDG_RELOAD_PARK_8S` = 2000 拍 @250Hz = **~8 s**, 见 `hw_watchdog.c:18`;
#:      由 `flash_store.c:332` 在**擦除前**调、`:351` 在**编程后**恢复
#:      ⇒ 它覆盖的正是一整次阻塞擦写)。
#:      ⚠ 取舍 (为什么敢把 IWDG 窗口当**时长上界**用): 它确实**不是**擦写时长, 而是"允许
#:      多长的停顿才不复位"的窗口。但它对**本条判据要覆盖的那一支**恰好成立: 请求的应答
#:      "还会到达" ⟹ 那个停顿必然短于它 (超过就 IWDG 复位, 应答**永远不会来** —— 那是
#:      另一支, 本来就不需要额度兜)。而 1s/1.86s 只是典型值, 不是上界。
#:      ⇒ 取证方向取**保守侧**: 取大只赔"静默", 取小赔"假成功" (见下), 故站 8s 这一侧。
#:   ③ **链路 + 主机侧取帧余量**: ~1 s 量级 (CDC 上行间歇低吞吐是已知现象)。
#:
#: ⇒ 下限 = **12.0 s** ≈ 3 + 8 + 1, 写成整秒。
#:
#: ⚠⚠ **下限的两个方向都会出错, 别只记一侧** (本注释曾经只写了后一半):
#:   · **取大** ⇒ "真·脱同步"的那条应答被静默吸收掉, 硬错误被降级成**静默** (只是窗口
#:     被拉长, 见 `_CartPending.__init__` 里那段"额度有效期偏长的后果");
#:   · **取小 / 过期** ⇒ 一条**还在路上**的应答撞上"队列**非空**"而被配给**新登记**的
#:     token ⇒ 调用方拿到**别人那条**的规划结果并**报成功** = **假成功** (E3 实测, 见
#:     `test_a_reply_that_outlives_move_timeout_is_not_paired_to_the_next_request`);
#:     撞上"队列空"则报 `LiteArmError` (归因误导)。这半写在上面"为什么必须有下限"那段。
#: ⇒ 两害相权**必须偏向取大**: "静默"是少报一次错, "假成功"是报了个**错的结果**。
#: (G1 实测的另一半仍成立: 额度本身不会把"被拒"变成"成功"。)
#:
#: ⚠ **为什么是"取较大者"而不是直接钉死在下限**: 就本判据而言下限已经够; 保留
#: `move_timeout` 那一支是为了**不缩小**既有行为 —— `move_timeout` 更大时今天本来就有
#: 更长的额度, 不该被一条**新加**的兜底砍短 (那会把本来能吸收的迟到应答变成硬错误)。
#: 方向是"额度**只增不减**"。
_ABSORB_TTL_FLOOR = 12.0


def cart_absorb_ttl(move_timeout: float) -> float:
    """`Arm` 要传给 `_CartPending` 的 `absorb_ttl` —— **两个构造点共用这一处**
    (`Arm.__init__` ① 与 `Arm.connect()` ②), 免得两处各写一遍 `max` 而漏掉一处。

    ⚠ 它**不是**"`move_timeout` 换个名字": 传 `move_timeout` 进去只是不想把大
    `move_timeout` 那支的额度砍短, 下限才是这条判据的**正确性条件** (见 `_ABSORB_TTL_FLOOR`)。
    """
    return max(move_timeout, _ABSORB_TTL_FLOOR)


class _CartToken:
    """一条**在途**笛卡尔请求的票据 (SDK 自造, 固件不知道它的存在)。

    `reply` 是原始 `0x4E` 载荷 —— 解成规划结果**不由本类做**: `CartPlan` 与它的
    `from_reply` 都在同文件里 (`CartPlan.from_reply` 已交付并被覆盖), 本类只负责
    "这条请求收尾了没有、怎么收的"。
    """

    __slots__ = ("_done", "reply", "lost", "absorbed_at")

    def __init__(self) -> None:
        self._done = threading.Event()
        self.reply: Optional[bytes] = None
        #: 非 None = **结局未知** (被清队 / 超时无应答), 值即给调用方看的原因
        self.lost: Optional[str] = None
        #: 登记那一刻的 `_CartPending.absorbed_replies` 水位线 —— 用来判"本条请求的等待
        #: 窗口里有没有吸收额度被消费掉"(见 `drop_and_absorb`: 消费过就等于固件已经把它
        #: 那条应答交出来了, 于是**不许**再补额度, 否则额度永 ≥1 = 自持)。
        self.absorbed_at = 0

    def wait(self, timeout: float) -> bool:
        """等收尾; `timeout=0` = 只探一下。返回值只表示"有没有收尾", 不表示收得好。"""
        return self._done.wait(timeout)

    def resolve(self, payload: bytes) -> None:
        self.reply = payload
        self._done.set()

    def fail(self, reason: str) -> None:
        """结局未知 —— 唤醒等待者 (不设 `reply`)。"""
        self.lost = reason
        self._done.set()


class _CartPending:
    """在途笛卡尔请求的 FIFO 队列 + `0x4E` 收集器。

    三种角色落在同一个对象上 (它们必须共享同一份队列状态):

    * :meth:`register` / :meth:`request` / :meth:`drop` —— **记账** (发出命令的入口用);
    * :meth:`on_reply` —— **收集器**: `Arm._read_one` 认领到 `0x4E` 就交给它;
    * :meth:`clear_pending` —— **清队**: 被 `_CART_CLEARS_UPON` 里的命令作废时调用,
      并把被清掉的条数记成**吸收额度** (见模块 docstring "多了一条" 那一支)。
    """

    def __init__(self, *, absorb_ttl: float, absorb: int = 0,
                 arm: Optional["Arm"] = None) -> None:
        """
        `absorb_ttl` 是**吸收额度的存活时间**, 由 `Arm` 传
        `max(Arm.move_timeout, _ABSORB_TTL_FLOOR)` —— 沿用既有旋钮, 外加一个**固件依据的
        下限** (为什么必须有下限、为什么是"取较大者", 见那个常量的定义)。

        `arm` 是**反向引用**, 不是图省事 (与 `_Ack.__init__` 里那处同源): `wait` 里那个
        pump 循环是本包**第六处**读循环, 它丢掉的"别人的帧"必须计进 `_Ack.foreign_frames`
        (`_note_foreign`), 而计数按设计留在 `_Ack` 上 ⇒ 两边必须互相够得着。
        ⚠ `arm=None` 只给**直接构造本类**的单元用例用 (它们验的是配对/吸收语义, 不经
        `Arm`) —— 那种情形下 pump 循环**不计数**, 与它压根没有 `_Ack` 是一致的。
        ⚠ `Arm.__init__` 建本对象时 `_a` 还不存在 (它在 `connect()` 里建), 所以这里存的是
        **`Arm` 而不是 `_Ack`**, 取用点在调用时刻 (`_note_foreign` 内部现读 `arm._a`)。

        额度**两个方向**都会出错, 别只记一侧:

        * **永不过期** ⇒ 它会把此后一条**真**脱同步的应答也吞掉 (硬错误降级成**静默**);
        * **到期太早** ⇒ 一条**还在路上**的应答会撞上"队列非空"而被配给**新登记**的
          token ⇒ 调用方拿到别人那条的结果并**报成功** (**假成功**)。

        ⚠ **它是构造时的一次性快照, 不跟随 `Arm.move_timeout` 的后续改动**: `wait` 每次
        现读 `arm.move_timeout` 当等待窗口, 而额度有效期固定为**本对象建成那一刻**的值
        (建成点是 `Arm.__init__` 与 `Arm.connect()`, 后者会重建本对象 —— 所以 connect
        之后才改 `move_timeout` 属于"两个窗口分叉": 等待窗口跟着变, 额度有效期不变)。
        额度有效期**偏长**的后果是"真·脱同步被静默吸收掉的时间窗变长" (那条本应报
        `LiteArmError` 的应答被当成清队的迟到应答吞掉), 方向是**静默**而不是假成功;
        `Arm.move_timeout` 的 docstring 里记了这条耦合 —— 要改就 `reconnect()`。
        """
        self._lock = threading.Lock()
        self._arm = arm
        self._q: List[_CartToken] = []
        #: 已收到 `0x4E`、但**还没有人 `wait` 过**的 token (按到达顺序)。
        #: `on_reply` 配对成功后在这里留一份: 结果已经到手, 若调用方没阻塞取走, 就只能靠
        #: `Arm.poll_cart()` 显式认领。`wait` 取走结果时把它摘掉 (`_unclaim`)。
        #:
        #: ⚠ **它不恒为空**: 三条入口**正常成功**时确实各 `wait` 一次 (于是留不下东西),
        #: 但有一条**放弃路径**会永久滞留 —— `_request_and_wait` 的 ACK 超时
        #: (`MotionTimeoutError`) 刻意**不**摘 token (摘了会让"固件其实已受理"那条应答配给
        #: 后面的活 token = 假成功), 而它的调用方已带着异常返回、再也不会来 `wait`;
        #: 固件那条 `0x4E` 一到就落进这里, 且**只有 `poll_cart`** 能摘。故本列必须有界
        #: (见 `_UNCLAIMED_MAX` 与 `on_reply`)。
        self._unclaimed: List[_CartToken] = []
        #: 因越界被丢弃的"待认领"条数 (**累计**) —— 丢弃不报错 (理由同 `extra_replies` 那
        #: 段的"不静默"), 故至少要看得见。与 `absorbed_replies`/`extra_replies` 并列。
        self.evicted_unclaimed = 0
        #: 队列空却收到 `0x4E` 的**累计**条数 —— "多了一条"的可观测面 (同
        #: `_Ack.unexpected_frames`: 计数不替代报错, 报错也不替代计数)。
        self.extra_replies = 0
        #: 被**吸收**掉的不可归属应答的**累计**条数 —— 与 `extra_replies` 并列的另一个
        #: 可观测面: 吸收是"清队后的副作用", 静默与否要能看见。**不报错**。
        self.absorbed_replies = 0
        self._absorb_ttl = absorb_ttl
        #: 剩余吸收额度 (仅在 `_absorb_deadline` 之前有效)。
        #: `absorb` 是**建对象时就带进来的**额度 —— 只给 `Arm.connect()` 那一处用:
        #: 会话重建销毁了旧对象的账, 而旧会话在途那几条**仍可能欠着应答** (`0x4E` 已在
        #: USB 上, 或至多 `CART_PLAN_MAX_TICKS` 之后才发出) ⇒ 新对象必须把那份额度**继承**
        #: 过来, 否则那条迟到应答一到就撞上"队列为空"判据, 从毫不相干的读路径里炸出
        #: `LiteArmError` (归因指向"固件与主机已错配", 真相是"我们自己重建了会话")。
        #: ⚠ 只搬**条数**: 旧对象里那几条的调用方由 `clear_pending` 唤醒, 不在这里管。
        #: ⚠⚠ 继承来的额度**至多影响新会话的一条命令, 且不被超时路径续期**: 它若先于配对
        #: 吃掉新会话**自己那条**应答 (两只在原理上无法区分), 那条命令报"结局未知", 而
        #: 它的超时路径**不再补额度** ⇒ 下一条起恢复正常 (见 `drop_and_absorb` 的例外段与
        #: `test_an_inherited_credit_cannot_perpetuate_itself_across_a_reconnect`)。
        self._absorb = absorb
        #: 额度截止 (monotonic 秒); `_absorb == 0` 时无意义。
        #: 带进来的额度**从建对象这一刻起算** `absorb_ttl` (与 `clear_pending`/
        #: `drop_and_absorb` 给额度时的口径一致: 额度总是"从现在起再活 `absorb_ttl`")。
        self._absorb_deadline = (time.monotonic() + absorb_ttl) if absorb else 0.0

    # ---- 记账 ----

    def register(self) -> _CartToken:
        """登记一条在途请求。**必须在写之前调** —— 否则窗口内到来的 `0x4E` 会落空。"""
        tok = _CartToken()
        with self._lock:
            tok.absorbed_at = self.absorbed_replies    # 水位线: 见 `drop_and_absorb`
            self._q.append(tok)
        return tok

    def request(self, write: Callable[[], None]) -> _CartToken:
        """登记 token 并**立刻**发帧 —— 登记与写在同一个 try 内。

        `write` 是零参可调用 (通常是 ``lambda: arm._raw_write(cmd, payload)``)。

        ⚠ **`write` 只许包含"写"** —— 校验必须由调用方放在 `request()` **之外**:
        校验失败根本**不该**登记 token (登记了又要摘, 就多出一段"这个异常到底发生在写之前
        还是写之后"的推断, 而这段推断正是下面那条区分要消灭的)。三条入口 (Task 4) 按此约定
        调用 —— 先校验, 过了再进这里。具体要挡在门外的两个 (它们都在写之前就会抛, 却**不是**
        `TransportError`, 从 `request()` 里看不出"没写"): 零重力守卫的 `InvalidCommandError`
        (`arm._write_cmd`) 与未连接的 `NotConnectedError` (`arm._require`)。

        ⚠ **写失败 (`TransportError`) 才摘除自己那个 token** 并原样抛出。摘除**本身不是**
        "比滞留更安全", 它是有前提的: `TransportError` ⟹ **整帧未送达** (载重不变量, 见
        `transport.write_frame` —— 它把"`write()` 抛"与"`flush()` 抛"**分开报**, 后者
        **不抛**)。固件从没收到这条请求, 也就**永远不欠**它的应答, 于是摘掉 token 恰好
        让"在途条数"与"应答条数"重新相等。

        ⚠ 两个方向**别记反** (本例正是"假成功"的高发区):

        * **滞留**一条未送达的 token ⇒ 在途比应答**多**一条 ⇒ 应答顺次前移一格, 队列
          **末尾**那条活 token 拿不到应答 ⇒ 报"结局未知" (`CartReplyLostError`), 安全;
        * **丢弃**一条其实**已送达**的 token ⇒ 在途比应答**少**一条 ⇒ 固件那条应答配给
          接在其后的活 token ⇒ 报"成功"而实际没跑 = **假成功**, 危险。

        (上面这组对照按本文档要求的**串行用法**成立 —— 同时 ≥2 条在途时两支都会错配,
        所以"串行"与这条不变量**两条都要守**。)

        ⚠ **其余任何异常 (`KeyboardInterrupt` 及 `write` 体内别的东西) 一律 re-raise 且
        不动队列** —— 它们**证明不了**帧没送达。最现实的一支是 `KeyboardInterrupt` 落在
        `flush()` 里: 那时 `write()` 早已返回、整帧**已在驱动里会被送达**, 固件真会回一条
        `0x4E`; 此时摘 token 就把它配给后面的活 token = **假成功**。反过来滞留最坏也只是
        报"结局未知"(安全方向)。
        **别为"简洁"把下面两个 except 合成一个 `BaseException`** —— 那会静默重造假成功。

        ⚠ 只给 `0x3A/0x3B/0x3E` 用: 清队 opcode 绝不该走这里 (登记完就立刻被
        `Arm._raw_write` 自己的清队钩子摘掉)。
        """
        tok = self.register()
        try:
            write()
        except TransportError:
            # 契约: write_frame 抛 TransportError ⟹ 整帧未送达 (残缺帧被固件按 CRC 丢)
            # ⇒ 摘下自己那个 token 才是干净的 (在途条数 ⟷ 应答条数重新相等)。
            self.drop(tok)
            raise
        except BaseException:
            # ⚠ 其余异常**证明不了**帧没送达 (最现实的是 KeyboardInterrupt 落在 flush 之后,
            # 那时整帧已在驱动里) ⇒ **留在队里**: 最坏是后续请求报"结局未知" (安全方向),
            # 而摘掉它会让那条应答配给后面的活 token = 假成功 (危险方向)。
            raise
        return tok

    def drop(self, token: _CartToken) -> None:
        """摘除**指定的**那条 token (按对象身份, 不是队尾 —— 并发下队尾是别人的)。

        两个地方都要摘: 在途队列 (`_q`) 与"已收到应答但没人认领"那一列 (`_unclaimed`)
        —— 一条被判死的请求不该还能被 `poll_cart` 认领出结果。
        """
        with self._lock:
            self._unclaim_locked(token)

    def drop_and_absorb(self, token: _CartToken, *, timed_out: bool) -> None:
        """摘除 token, **并留一格吸收额度** —— 摘 token 与给额度必须在同一把锁内。

        `timed_out` 说明**为什么要放弃**这条请求 (两条调用点各传一个值, 别统一):
        超时支传 `True`、读链路炸 (pump) 支传 `False`。它**只**决定下面那个"例外段"
        生不生效, **不影响**"摘 token"这一半。

        与 `clear_pending` **同构** (理由逐字相同, 见那里): 摘掉 token 的那一刻, 固件那条
        应答可能**已经在路上** (它受理了这条命令, 只是我们等不到/读不到)。不留额度的话,
        它一到就撞上"队列空"那条判据 ⇒ `LiteArmError` 从**毫不相干的读路径**
        (`get_state()`/`get_tcp()`) 炸出来, 把一次本可解释的迟到报成"固件与主机已脱同步"。

        额度 = 1 (这条请求**至多**欠一条应答)。两支:

        * 固件其实**会**发 ⇒ 那条应答被吸收 (计数 `absorbed_replies`), 不报错;
        * 固件其实**不会**发 ⇒ 额度白给一格 —— 最坏情形是它吞掉此后的**一条**真脱同步
          应答 (报"结局未知"而不是硬错), 且额度受 `absorb_ttl` 过期约束。

        ⚠⚠ **给额度只把误炸挡在"额度还活着"那一段时间里, 不是"这类误炸不会再发生"** ——
        额度一过期, 那条迟到应答就回到原样: 撞上"队列非空"被配给**新** token (**假成功**,
        见 `_ABSORB_TTL_FLOOR` 与本文件顶部那段), 撞上"队列空"照样炸 `LiteArmError`。
        所以那个下限是**正确性条件**, 不是余量; 而"窗口外"那一支的归因仍然是错的: 报出来
        的是"固件与主机已错配", 真相只是"**我们自己的**额度到期了" (E3 实测)。

        方向与"多了一条"那条守卫一致: **宁可多吸收一条, 绝不假成功**。反过来 (不给额度)
        是**归因误导**: 用户看到的是"配对已错位", 真相是"我们自己放弃了一条请求"。

        ⚠⚠ **例外 (唯一一条, 必须先于"给一格"判断, 且**只对超时支生效**):
        `timed_out` 且 本条请求的窗口里已经消费过额度 ⟹ 不补。** 吸收**先于**配对
        (`on_reply`), 所以一条在途 token 的应答到了、而额度还有存量时, 被吃掉的就是
        **它自己那条** —— 那条应答**已经交出来了**, 固件不再欠它什么。此时再补一格,
        补出来的那格会去吃**下一条**命令的应答, 那条又超时、又补一格 … **自持**:
        实测 (改前) `connect()` 继承来的一格额度足以让新会话**此后每一条**笛卡尔命令都
        报"结局未知" (`#1..#6` 连报, `absorbed_replies` 一路涨)。
        F5 之前同一场景分两支 (两支都实测过):
          · 旧会话那条应答**始终没到** ⇒ **全绿** —— 即 F5 把一条本来健康的会话变成了永久
            错位 (那格额度当时不存在, 链条根本没起点);
          · 它**确实会到** ⇒ "炸一次 `LiteArmError` (无关读路径) 然后恢复正常"。

        ⚠⚠ **为什么例外只许挂在 `timed_out` 上, 不许两条放弃路径共用**: 上面那句推断
        ("被吸收的那条就是本 token 自己的") 靠的是**等满了整个等待窗口** —— 窗口够长时,
        固件那边该发的早该发完了, 于是"还有**别人的**应答在路上"不再可达。
        **pump 那一支根本没等过窗口** (读链路当场就炸了), 这条前提在它身上**没有依据**
        ⇒ 它照旧补一格。⚠ 别再把这一支的判据写成"只在 `move_timeout < 3s` 时才可达":
        pump 支**完全不经过超时**, 默认 `move_timeout = 15.0` 下照样可达。

        ⚠⚠ **例外判错的方向是假成功, 不是"宁可让一条报结局未知"** (别只记后者): 若该补的
        那格没补, 而固件**真还欠**本 token 一条应答, 它到达时队列非空 ⇒ `on_reply` 直接
        配给**下一条** token ⇒ 调用方拿到**别人那条**的规划结果并**报成功**; 撞上队列空则
        炸 `LiteArmError` (归因仍是错的)。`0x4E` 载荷里没有命令 id, 它无从分辨。
        ⚠ pump 支走例外时正是这一侧: 被吸收的可能是**清队前遗留的**那条, 而本 token 自己
        那条还在路上 —— `test_a_pump_failure_must_not_suppress_the_credit` 钉的就是它。
        (超时支若同样判错, 代价也是假成功; 但那一支至少还有"窗口够长"可依, 且**没有它
        自持就回不来** —— 实测 `#1..#6` 全红, 那是本例外唯一的立论。)

        判"已经消费过"用的是 `_CartToken.absorbed_at` 水位线 (登记时快照)。⚠ 这只用
        "**有**消费"这一个事实, 不去猜消费掉的是谁那条 —— 判错的那一侧 (被吃掉的其实是
        旧会话/别的进程遗留的一条, 而固件真还欠本 token 一条) 的代价是: 那条迟到应答
        落回"额度过期"那一支 (撞队列非空 ⇒ 错配 ⇒ **假成功** / 撞队列空 ⇒ `LiteArmError`)。

        小结 (两个方向都在, 别只记一侧):
          · **补**额度 ⇒ 可能**静默** (吞掉一条真脱同步的应答, 降级成"结局未知");
          · **不补** ⇒ 可能**假成功** (错配)。
        故只在有"窗口够长"依据的那一支 (超时支) 才敢不补。
        """
        with self._lock:
            self._unclaim_locked(token)
            if timed_out and self.absorbed_replies != token.absorbed_at:
                # 超时支 + 窗口里已经吸收过一条 —— 那条就是本条请求的应答 (见 docstring
                # 的例外段): 固件不再欠, 所以**不补额度**(补了就是自持)。两个字段都不动。
                # ⚠ pump 支**不走这里**: 它没等过窗口, "被吃掉的是自己那条"没有依据。
                return
            now = time.monotonic()
            self._absorb = self._absorbance_locked(now) + 1
            self._absorb_deadline = now + self._absorb_ttl

    def _unclaim_locked(self, token: _CartToken) -> None:
        """**调用方必须持 `self._lock`** —— 两列里各摘一次 (不在的忽略)。"""
        try:
            self._q.remove(token)
        except ValueError:                        # 已被清队摘走/已配对消费
            pass
        try:
            self._unclaimed.remove(token)
        except ValueError:
            pass

    def claim_resolved(self) -> Optional[_CartToken]:
        """**非阻塞**认领: 队首 token 若已收尾就摘掉并返回它, 否则 `None`。

        FIFO 配对下队首就是"最早那条" —— 它被解答之前, 后面的 token 不可能被解答
        (应答按受理顺序发出), 所以只看队首是对的。

        用途只有一个: `Arm.poll_cart()` 显式认领一条**没有被 `wait` 取走**的结果。

        ⚠ 判据必须含 **`_done.is_set()`**, 不能只看"那一列非空": `on_reply` 是**先登记
        进那一列、后 `resolve`** (顺序是承重的, 见那里), 两者之间有一条缝 —— 并发读者
        恰好落进这条缝就会拿到一个**还没收尾**的 token, `CartPlan.from_reply(None)`
        当场 `TypeError` (单线程不可达, 故此前没暴露)。缝里返回 `None` 是对的语义:
        "此刻还没有可取的结果", `poll_cart` 本来就以 `None` 表示这个。
        """
        with self._lock:
            if self._unclaimed and self._unclaimed[0]._done.is_set():
                return self._unclaimed.pop(0)
        return None

    def _unclaim(self, token: _CartToken) -> None:
        """`wait` 把结果取走之后, 从"待认领"那一列移除 —— 否则 `poll_cart` 会**重复交付**。"""
        with self._lock:
            try:
                self._unclaimed.remove(token)
            except ValueError:
                pass

    # ---- 收集器 ----

    def on_reply(self, payload: bytes) -> None:
        """收集器 —— `Arm._read_one` 见到 `0x4E` (`P.RSP_CART_PLAN`, 见那里的常量分支)
        时调它。

        ⚠ **先消费吸收额度, 再配对** —— 顺序是决定性的: 清队后立刻登记的新 token 会让队列
        **非空**, 而那条清队前就已到达的迟到应答若被拿去配对, 就**配错了** (报成功而实际
        没跑完, 即**假成功**)。故额度那一步必须排在"队列空/非空"判断之**前**。

        ⚠ 队列空**且额度为 0** 才是真 "多了一条": 计数**并报错**, 不静默丢弃。这里比
        "未识别帧计数"更硬气是有理由的 —— 应答比请求多说明固件与主机**已经**错位,
        吞掉它就把错位藏了起来。

        ⚠ **为什么"多了一条"抛基类 `LiteArmError` 而不给它一个专门的异常类型**
        (与"少了一条"配 `CartReplyLostError` 形成对照, 别顺手补一个子类):
        两者的**性质不同** —— "少了一条"是**调用方面对的处境** (这条运动的结局未知,
        必须能与"规划失败"/"被接管"分开, 调用方要据此决定回读还是报警), 所以它得是
        独立成型、可被 `except` 的类型; "多了一条"是 **SDK 的配对模型与固件脱同步**
        —— 内部不变量被破坏, 没有任何调用方能据它做出正确决定, 也**不该被专门 catch**
        (catch 了就等于把"配对已错位"当成可恢复的常规分支)。给它一个专用类型反而在
        邀请后续代码去 `except` 它。

        ⚠ 配对成功的那条进"待认领"列时**封顶** (超出丢最旧, 见 `_UNCLAIMED_MAX`) ——
        那一列在"调用方放弃 token"的路径上无界增长。封顶**只丢**"没人来取的陈旧结果",
        不改变本方法的任何分支, 也不影响 `wait`。
        """
        with self._lock:
            if self._absorbance_locked(time.monotonic()) > 0:
                # 清队后的不可归属应答 —— 属于**刚被清掉的那条请求**, 与"真·脱同步"是
                # 两回事: 消费一格额度, 计数, 直接返回 (不配对、不报错、队列不动)。
                self._absorb -= 1
                self.absorbed_replies += 1
                return
            if not self._q:
                self.extra_replies += 1
                raise LiteArmError(
                    f"收到 0x4E (笛卡尔规划应答) 但队列为空 —— 应答比请求多, "
                    f"固件与主机已错配 (已计数 extra_replies={self.extra_replies}); "
                    f"这不是可以忽略的噪声")
            tok = self._q.pop(0)
            # 先登记"待认领"再唤醒等待者: `wait` 醒来后会把它从这一列移走, 顺序反过来的话
            # 那次移除会扑空, 于是同一条结果被 `poll_cart` 再交付一次。
            self._unclaimed.append(tok)
            # 给那一列封顶 (见 `_UNCLAIMED_MAX`): 无界的唯一现实来源是"调用方放弃 token"
            # (ACK 超时路径), 每放弃一条就永久留下一条。丢**最旧**的, 并计数 —— 静默丢弃
            # 正是本模块一直在防的东西。
            while len(self._unclaimed) > _UNCLAIMED_MAX:
                self._unclaimed.pop(0)
                self.evicted_unclaimed += 1
        tok.resolve(payload)

    # ---- 清队 ----

    def clear_pending(self, reason: str = "") -> int:
        """清队 —— 由 `Arm._raw_write` 在**发出清队 opcode 之前**调用。返回摘除条数。

        这几条请求在固件侧已经**静默作废**, 永远等不到 `0x4E` 了: 它们被标成
        "结局未知"并唤醒等待者 —— 不能留着让调用方白等到超时, 更不能让下一个
        `0x4E` 配给它们。

        ⚠ **记账被销毁了, 但被清掉的条数留下一份额度**: 清队那一刻固件那条规划可能
        **已经跑完、应答早躺在主机 RX 缓冲里** (两支在原理上无法区分, 见模块 docstring)。
        故 `n = 本次清掉的条数`, **仅当 `n > 0`** 时 `额度 = 当前有效额度 + n` 且
        `截止 = now + absorb_ttl`; **`n == 0` 时两个字段都不动** —— 这不是优化, 是正确性:
        零重力保活线程每 40ms 发一条清队 opcode (`0x06`, 而它几乎永远是空清队), 若空清队
        也刷新截止, 额度就**永远不过期**, 此后一条**真**脱同步的应答会被永久静默吞掉。

        额度**跨多次清队累加**: 写成"重置为本次条数"的话, 上一次那条待吸收的迟到应答
        会退化成硬错误。
        """
        with self._lock:
            tokens, self._q = self._q, []
            n = len(tokens)
            if n:
                now = time.monotonic()
                self._absorb = self._absorbance_locked(now) + n
                self._absorb_deadline = now + self._absorb_ttl
        why = (reason or "在途规划被其它命令作废") + \
            " —— 固件侧不会再发 0x4E, 结局未知 (只能回读状态判定)"
        for t in tokens:
            t.fail(why)
        return n

    def _absorbance_locked(self, now: float) -> int:
        """当前**有效**吸收额度 —— 惰性过期: `now > 截止` 即视为 0。

        **调用方必须持 `self._lock`**。惰性 (而不是起个定时器) 是有意的: 额度本来就只有
        收集器一带在查, 没有别的读者需要被"及时通知"。

        ⚠ 判据是**严格大于**: `now == 截止` 算**未**过期 (刻意的 —— 规格只写了
        "`move_timeout` 后惰性归零", 没定这一侧; 本实现取"到期时刻仍有效")。
        """
        if self._absorb and now > self._absorb_deadline:
            self._absorb = 0
        return self._absorb

    # ---- 等待 ----

    def wait(self, token: _CartToken, timeout: float) -> bytes:
        """等这条 token 收尾; **超时按"少了一条"处理**。返回原始 `0x4E` 载荷。

        `timeout` 由调用方给 (惯例是 `Arm.move_timeout` —— 沿用既有旋钮, 不新造字段)。

        ⚠⚠ **[2026-09-22] `pump` 参数已删除。** 它存在的唯一理由是「本包**没有读线程**,
        所以'等应答'与'读应答'必须是同一件事」—— 现在 `litearm-python` **有**读线程
        (`_Ack._reader_loop`, 唯一读者), `0x4E` 由它在 `_Ack._deliver` 里直接交给收集器
        (`on_reply`) ⇒ 本方法**只在事件上阻塞**即可。连带删掉的还有 `_count_pumped`
        (逐条计 `foreign_frames`): "读到了却不认领"这个动作不存在了。

        收尾不成功一律抛 :class:`CartReplyLostError` (**不是** `MotionTimeoutError`:
        这里要表达的是"未知结局", 混进通用超时会让调用方按"没生效"去重发)。

        ⚠ 两条**放弃**路径 (超时 / 读线程死) 摘 token 时都留**一格吸收额度**
        (`drop_and_absorb`) —— 与 `clear_pending` 同构: 固件**真欠**的那条 `0x4E` 可能已经
        上路了, 不留额度它就会撞上"队列空"那条判据, 把 `LiteArmError` 从毫不相干的读路径
        (`get_state()`) 里炸出来。详见 `drop_and_absorb` 的两支分析。
        ⚠ 但两条路径传给 `drop_and_absorb` 的 `timed_out` **不同** (超时 `True` / 读线程死
        `False`): 那个函数的"例外段" (窗口里吸收过 ⟹ 不补额度) **只对超时支成立** ——
        线程死那一支没等过窗口, 走了例外就会把固件真欠的那条应答变成下一条命令的假成功。

        ⚠ 读线程死了要**当场响亮**: 等满 `timeout` 才报"结局未知"会把"链路断了"说成
        "固件没回"。所以每一拍醒来看一眼 `_Ack._reader_error`, 置了就立刻收场。
        """
        end = time.monotonic() + timeout
        link_dead = False
        while token.reply is None and token.lost is None:
            left = end - time.monotonic()
            if left <= 0.0:
                break
            token.wait(left)
            a_ = self._arm._a if self._arm is not None else None
            if a_ is not None and a_._reader_error is not None:
                # 链路没了 (读线程已退出): 这条请求的结局已不可知, 而且**没人再来收它** ——
                # 留着 token 会让下一次会话的 0x4E 配给一条早就无人认领的请求。
                link_dead = True
                break
        if token.lost is not None:
            raise CartReplyLostError(token.lost)
        if link_dead:
            # ⚠ **把病因原样抛出去**（与旧 `pump` 支一致）：报 `CartReplyLostError`
            # ("结局未知") 会把"链路断了"说成"固件没回"，排查方向直接反。
            # ⚠ `timed_out=False`：这一支**没等过窗口** ⇒ 照旧补额度。走成 `True` 会让
            # 固件真欠的那条应答被"例外段"吞掉 ⇒ **下一条命令假成功**。
            self.drop_and_absorb(token, timed_out=False)
            token.fail("读线程已退出 (链路没了) —— 结局未知")
            a_ = self._arm._a if self._arm is not None else None
            raise TransportError(
                f"等待 0x4E 期间读线程已退出: {getattr(a_, '_reader_error', None)}")
        if token.reply is None:                       # 超时 —— "少了一条"
            self.drop_and_absorb(token, timed_out=True)
            token.fail(f"超时 {timeout:.1f}s 没等到 0x4E 应答 (固件单槽 pending 在突发下"
                       f"会吞掉中段应答) —— 结局未知 (只能回读状态判定)")
            raise CartReplyLostError(token.lost)
        self._unclaim(token)                       # 结果已交付, 不许再被 poll_cart 认领
        return token.reply

    # ---- 只读 ----

    @property
    def pending(self) -> int:
        """在途条数。"""
        with self._lock:
            return len(self._q)


# ===========================================================================
# 规划结果的语义 —— `RSP_CART_PLAN (0x4E)` 载荷 / 异常映射 / 能力探测 / 三条入口
# ===========================================================================

#: 固件 `cart_err_t` (`cart_plan.h`) 的取值 —— `0x4E` 载荷第 2 字节。
#: ⚠ 这是**规划层**的码, 与 `RSP_ERR` 第二字节的**门禁原因码** (未使能 0x03 /
#: 零重力 0x04 / 掉线锁存 0x06) **共用 1~6 的数值区间却语义完全不同**, 别互推。
#: 整套枚举 (含 0) 都镜像在这里, 免得读的人为了确认"2 到底是哪个"去翻 cart_plan.h;
#: `CART_ERR_OK` 没有别的引用点 —— "成功"在代码里由 `CartPlan.ok` 表达。
CART_ERR_OK = 0
CART_ERR_IK = 1              # 某路点 IK 无解, 或相邻解跳变超阈值
CART_ERR_COLLINEAR = 2       # move_c 三点共线, 定不出圆
CART_ERR_TOO_LONG = 3        # 路点超容量 —— **整条拒绝, 臂一步没动**
CART_ERR_LIMIT = 4           # 路点本身或段中插值越关节限位
CART_ERR_CANCELED = 5        # 被新请求取代 (预期内的接管, 不是故障)
CART_ERR_BADARG = 6          # 入参非法

#: move_c 的起点校验容差 —— 与 `Arm.move_p` 的默认值同源 (6mm / 0.03rad), 不是**抄**来的
#: 巧合: `move_c` 校验的就是"实际 TCP 是否在 `pose_start` 附近", 判据必须与 `move_p` 的
#: 到位判据一致 (同一把尺子), 否则 move_p 收工的位置会被 move_c 判成"起点不一致"。
#: ⚠ 同源关系由 `test_cart_protocol.py::test_cart_start_tolerances_track_move_p_defaults`
#: 钉住 —— 只靠这两个数字与 `Arm.move_p` 的默认值**字形相同**是守不住的: 改了 move_p 默认
#: 而忘了这里, `move_c` 的起点判据会**静默不跟随**。
CART_START_POS_TOL = 0.006
CART_START_RPY_TOL = 0.03


@dataclass
class CartPlan:
    """一条笛卡尔命令的**规划结果** (固件 `RSP_CART_PLAN 0x4E` 的载荷)。

    三个入口 (:meth:`Arm.move_l` / :meth:`Arm.move_c` / :meth:`Arm.move_path`)
    都返回它。`ok=False` 时入口**不返回**而是抛对应异常 (见 :func:`raise_for_plan`)。

    ⚠ **`started_busy` / `settled` / `q_final` / `settle_err_rad` 四个字段由 `wait`
    决定** (三条入口的 `wait` 参数, 默认 `True`):

    | `wait` | 这四个字段 |
    |---|---|
    | `True` | 填**真值** —— 入口等到状态帧的 `CART_BUSY` (bit10) 落 0 且 `q` 静止, **再回读一次 TCP 与目标比对**才返回 |
    | `False` | 一律"未等待"默认值 (`False` / `False` / `[]` / `0.0`), 只等到**规划结果** |

    `wait=False` 时这四个字段**不是**"没到位", 而是"**没等**" —— 想知道臂现在什么样,
    该 `get_state()` 回读, 而不是拿这份陈旧的规划去推断。

    `started_busy` = 是否在**可信**状态帧上见过 `bit10=1`; `q_final` = **收尾帧**的 `st.q`。

    ⚠⚠ `settled=True` 要**两条一起**成立 (缺一不可, 见 `_tcp_reached`):
    ① 到位判据 (先见 `bit10=1` 再见 `0` 且 `q` 静止) 收的尾; **且** ② 到位判据满足**之后**
    回读的**实际 TCP** 与**本次请求的目标位姿**对得上 (容差 6mm / 0.03rad)。

    ⚠⚠ **`ok=True` 而 `settled=False` 是一个必须存在的结局** —— 它的含义是"这条笛卡尔
    轨迹被**别的运动作废**了" (锁外/进程外的 `movej`/`home`/`zero_g`: 固件一收到就
    `cart_invalidate_before_motion()` 把这条轨迹作废, 而收尾照样是 `bit10=0` + `q` 静止),
    或者 `get_tcp()` 取不到 (超时/断连 —— 保守回"没到位")。
    **`ok` 只说明"固件受理并规划出来了"**, 不说明臂停在目标上。
    ⇒ 现场**必须回读 `get_tcp()` 看真实落点**, 不要把它读成"只是没停稳"。
    (`wait=True` 的**超时/故障**不是这个形状 —— 那两条会抛 `MotionTimeoutError` /
    `MotorFaultError`, `settled` 保持默认 `False` 且调用方拿不到 `CartPlan`。)

    ⚠ **降级分支**: 可信帧上**始终**没见过 `bit10=1` 但 `q` 静止判据成立时, 同样
    `settled=True` 而 `started_busy=False`, **不抛异常** ("不能判失败" —— 规划短到
    两帧之间就跑完、或 `bit10` 这一位没上报, 都会落在这里)。

    ⚠ `settle_err_rad` 的名字沿用旧 PC 侧规划器的字段, 但**语义已经变了**: 从前是
    "收尾后与**终点指令**的最大关节偏差", 而在固件原生路径下**该量物理不可得**
    (PC 侧没有目标关节向量, 也不许 IK)。现在它是"**结束时各轴 `q` 与 `q_final` 的差**"
    —— 即判到位那 `arrive_frames` 帧的抖动幅度 (与**终点指令**无关)。
    """

    ok: bool = False
    err: int = 0
    n_wp: int = 0
    plan_us: int = 0
    started_busy: bool = False
    settled: bool = False
    q_final: List[float] = field(default_factory=list)
    settle_err_rad: float = 0.0

    #: SDK 自造的 `err` 档 (**不是**固件 `cart_err_t` 的取值): 这条请求的**结局未知**。
    #: 与 :class:`CartReplyLostError` 说的是同一件事, 只是从 `CartPlan` 的角度表述。
    #: ⚠ 当前**没有产出者** —— 三条入口在这个情形下直接抛 `CartReplyLostError`
    #: (见 `_CartPending.wait` 的"少了一条"守卫), 于是不会构造出带本档的 `CartPlan`。
    ERR_REPLY_LOST = -1

    @classmethod
    def from_reply(cls, payload: bytes) -> "CartPlan":
        """`0x4E` 载荷 -> `CartPlan`。

        载荷 = `ok u8 + err u8 + n_wp u16 LE + plan_us u32 LE` (8B)。
        """
        if len(payload) < 8:
            raise TransportError(
                f"RSP_CART_PLAN 帧短: {len(payload)}B (应 8B) —— 帧布局漂移?")
        return cls(ok=payload[0] != 0, err=payload[1],
                   n_wp=struct.unpack_from("<H", payload, 2)[0],
                   plan_us=struct.unpack_from("<I", payload, 4)[0])


def raise_for_plan(plan: CartPlan) -> None:
    """`ok=False` 时按 `err` 抛对应异常 (`ok=True` 时什么都不做)。

    | err | 抛 |
    |---|---|
    | 1 `IK` | `IKError` |
    | 2 `COLLINEAR` / 3 `TOO_LONG` / 4 `LIMIT` | `CartesianPlanError` |
    | 5 `CANCELED` | `MotionSupersededError` (**独立成型, 不是失败**) |
    | 6 `BADARG` | `InvalidCommandError` |

    ⚠ `CANCELED` 单独成型是刻意的: 接管是正常用法 (新目标来了就接管旧的), 混进
    "规划失败"会让调用方走故障恢复。反过来 `CartesianPlanError` 的三档都是
    **"这条轨迹本身不成立, 臂一步没动"**。

    ⚠ 未登记的 `err` 值也归到 `CartesianPlanError` (ok=0 就是"规划没成"), 但异常消息里
    带上原始码 —— 固件新增一档时不该被静默吞掉。
    """
    if plan.ok:
        return
    err = plan.err
    if err == CART_ERR_IK:
        raise IKError(f"固件笛卡尔规划失败: 路点 IK 无解或解跳变 (err={err})")
    if err == CART_ERR_CANCELED:
        raise MotionSupersededError(
            "这条笛卡尔请求被固件的另一条请求取代 (err=5 CANCELED) —— "
            "预期内的接管, 不是失败")
    if err == CART_ERR_BADARG:
        raise InvalidCommandError(f"固件拒绝这条笛卡尔命令: 入参非法 (err={err})")
    detail = {CART_ERR_COLLINEAR: "三点共线, 定不出圆 (move_c)",
              CART_ERR_TOO_LONG: "路点超容量 —— 整条拒绝, 臂一步没动",
              CART_ERR_LIMIT: "路点或段中插值越关节限位"}.get(err, f"未登记的 err={err}")
    raise CartesianPlanError(f"固件笛卡尔规划失败: {detail} (err={err})")


# ---------------------------------------------------------------------------
# 能力探测
# ---------------------------------------------------------------------------

#: 探测的**总**尝试次数 (首次 + 重试)。见 `probe` 的 docstring: 读超时只可能是丢帧。
_CART_PROBE_ATTEMPTS = 3


def probe(arm) -> bool:
    """固件是否支持笛卡尔 (受 `#if LITEARM_CART_PLAN` 编译开关约束)。

    ⚠ **只发空载荷的 `0x3A`**: `0x3A~0x3E` 里**没有只读命令** —— 发一条合法载荷就是
    "connect 之后臂自己动一下"。空载荷撞的是固件的**长度校验** (`usb_cmd.c` 的
    `len < 28`), 它排在 `cart_gate_ok` 与一切副作用**之前**, 于是:

    | 固件 | 空载荷 0x3A 的回码 | 判读 |
    |---|---|---|
    | 有笛卡尔 (开关 ON) | `ERR{0x3A, 0x01}` (长度不足) | **能力在** |
    | 无笛卡尔 (开关 OFF) | `ERR{0x3A, 0x00}` (`default` 分支) | **能力不在** |

    (`0x3B`/`0x3C`/`0x3D` 的长度校验同样排在一切副作用之前, 所以它们也能当探针;
    选 `0x3A` 只是因为它是笛卡尔里语义最直白的那条。)

    ⚠ **探测帧不进 `_CartPending` 队列** (走 `_write_query`, 不登记 token): 登记了就会
    造出一个永远等不到 `0x4E` 的悬挂态 —— `0x4E` 只在**受理了**一条规划之后才发。

    ⚠⚠ **"一片安静" ≠ "不支持" —— 故本探测有界重试** (`_CART_PROBE_ATTEMPTS` 次):
    固件的 `default` 分支保证**任何**固件都会对 `0x3A` 回一条 `ERR{cmd,0x00}`, 所以
    读超时只可能是**这一帧丢了** (本硬件的 USB CDC 上行有已知丢帧), 而不是"固件没有
    这条命令"; 而本函数的结果在 `connect()` 里缓存**整个会话**, 一次丢帧就会让这个
    会话的笛卡尔能力被误判成"固件没编进去"。两种回应的处置**不同**:

    * **`ERR{0x3A, 0x00}` = 确定结论** (固件确实没有这条命令) ⇒ **立刻返回 `False`,
      不重试**;
    * **读超时 (一片安静)** ⇒ 重试, 至多 `_CART_PROBE_ATTEMPTS` 次; 仍安静才返回
      `False` (**fail-closed** —— 未确认支持就不发能起规划的命令), 并把这一情形
      记进 `arm._cart_probe_silent`, 让 `_require_cart_support` 的报错能与"固件确报
      不支持"**分别措辞** (否则现场无法归因: 该换固件还是该查链路)。

    ⚠ **迟到的探测 ERR 会从无关读路径里冒出来** (已知的归因误导, 不是新缺陷):
    `expect(..., raise_on_err=False)` 只认**读到**的那一条; 若三次都判成"一片安静"
    (本函数已 fail-closed 返回), 而固件那条 `ERR{0x3A,0x01}` 其实只是**晚到** —— 它就会在
    之后某次读里被别的读循环认下, 而 `_Ack.expect`/`Arm._read_status` 匹配 `RSP_ERR`
    时**只看命令码** ⇒ `get_state()`/`get_tcp()` 会抛 `CommandRejectedError{0x3A,0x01}`。
    方向是安全的 (fail-closed: 一条能起规划的命令都没发出去, 臂不会因它而动), 但现场读到
    的是"笛卡尔命令被拒", 与真相 ("探测的应答迟到") 不符。
    ⚠ 重试把这种机会从 1 次变成 `_CART_PROBE_ATTEMPTS` 次 —— 换来的好处见上 (一次丢帧
    不该把整个会话的笛卡尔能力判成"固件没编进去"), 这个代价是知情的。
    """
    arm._cart_probe_silent = False
    for _ in range(_CART_PROBE_ATTEMPTS):
        arm._write_query(P.CMD_MOVE_L, b"")
        try:
            _, payload = arm._require().expect(
                P.RSP_ERR, 1.0, "笛卡尔能力探测 (0x3A 空载荷)",
                echo_cmd=P.CMD_MOVE_L, raise_on_err=False)
        except MotionTimeoutError:
            continue                       # 安静 = 这一帧丢了, 再试一次
        # 判据就是能力判定那一条: **0x00 恒等于固件的 `default` 分支** (未实现该命令),
        # 其余任何非 0 的错误码都说明这条命令在固件里存在 (`test_capability.py` 与
        # `test_protocol_sync.py::test_err_code_zero_only_comes_from_default_branch` 守它)。
        return (payload[1] if len(payload) > 1 else 0) != 0x00
    arm._cart_probe_silent = True          # 3 次均无应答 —— 探测未获确认 (不是"确报不支持")
    return False


# ---------------------------------------------------------------------------
# 三条入口 —— ⚠ 守卫顺序是承重的: 见本文件顶部 docstring 与下面 `_request_and_wait`
# ---------------------------------------------------------------------------

def _require_cart_support(arm) -> None:
    """② 能力检查 —— 排在 `_require()` 之后、零重力守卫之前。

    ⚠ 缓存在 `Arm._cart_supported` (私有名)。写成公开的 `arm.cart_supported` 或加一层
    `arm.cart` 门面会让 `tests/test_full_coverage.py::test_public_api_surface_is_all_exercised`
    变红 —— 它会要求每个公开成员都被练习过。

    ⚠ 报错**分两支措辞** —— 探测那一步把两种"不支持"分开了 (`arm._cart_probe_silent`),
    在这里混成一句话会让现场无法归因: 一支该换固件, 另一支该查链路。
    两支的处置方向相同 (都不发帧, fail-closed), 分开的只是**说法**。
    """
    if arm._cart_supported:
        return
    if arm._cart_probe_silent:
        why = (f"探测未获确认: connect() 时的空载荷 0x3A 探测 {_CART_PROBE_ATTEMPTS} 次"
               f"都没等到应答 —— 固件的 default 分支保证任何固件都会回一条 ERR, "
               f"所以这只能是上行丢帧 (本硬件 USB CDC 有已知丢帧), 不是'固件没有这条命令'")
    else:
        why = ("固件确报不支持: connect() 时的空载荷 0x3A 探测回了 ERR{0x3A,0x00} "
               "(default 分支 ⇒ 固件没编进 LITEARM_CART_PLAN)")
    raise UnsupportedByFirmwareError(
        f"{why} —— 故不发帧 (未确认支持就不发能起规划的命令)。"
        f"当前固件: {arm.firmware or '?'}", cmd=P.CMD_MOVE_L, code=0x00)


def _reject_in_zero_g(arm) -> None:
    """③ 零重力守卫 —— **必须在 `request()` 之前**跑完。

    `Arm._write_cmd` 里有一道判据完全相同的守卫 (`if guarded and self._zg_active`),
    它在这里**重复**出现, 是为了让"零重力保活期间不许发动作命令"这条判定发生在
    **登记 token 之前** —— 否则 `InvalidCommandError` 会在写失败的位置抛出, 而
    `_CartPending.request` 只对 `TransportError` 摘 token, 于是 token 留在队里。

    保留 `_write_cmd` 里那道不是冗余: 它是**兜底**, 覆盖"检查与写之间 `_zg_active`
    翻转"的极端竞态 (那时 token 滞留 —— 方向安全, 后续报"结局未知"而不是假成功)。

    文案取自 `arm.ZERO_G_GUARD_MESSAGE` (与 `_write_cmd` 那道**同一个常量** —— 从前是各抄
    一份逐字节相同的字符串, 改一侧就静默漂移; 调用方按文案归因, 两句不一样会被读成两种
    不同的拒绝)。⚠ 该常量只能在**函数内**导入: `arm` 在模块级导入 `cart`, 反过来在模块级
    导入会让两边都在半初始化状态 (同 `_rot.as_pose` 里那处 `errors` 的延迟导入)。
    """
    if arm._zg_active:
        from litearm.arm import ZERO_G_GUARD_MESSAGE
        raise InvalidCommandError(ZERO_G_GUARD_MESSAGE)


def _as_pose6(pose, label: str) -> List[float]:
    """位姿入参 -> `[x,y,z,r,p,y]` (固件 `0x4E` 家族的载荷形态)。

    形态判定整个委托给 :func:`litearm._rot.as_pose` —— 它收
    `(pos[3], R[3x3])` / `[x,y,z,roll,pitch,yaw]` / 4x4 齐次矩阵三种写法。
    这**不是**放宽: 三条入口替代的旧 `movel/movec` 本来就收两种写法
    ("固件的 `[x,y,z,r,p,y]` **或** pylitearm 的 `(position[3], rotation[3x3])`"),
    只收 6 标量是**公开 API 的能力倒退**。

    ⚠ 不合法时 `as_pose` 抛的 `InvalidCommandError` 文案里带**实际收到的形状** ——
    这里只把 `label` (哪条入口的哪个位姿) 前缀上去, 原文**原样保留**, 不重写、
    不换成笼统的"pose 非法": 只说非法会让调用方在 6 向量与位姿对之间反复猜。

    ⚠ **`ValueError`/`TypeError` 也必须收进 `LiteArmError` 体系** —— 它们是**入参畸形**
    (`move_l([0.1,...,"x"])` 的 `float("x")`、`move_l((pos, [1,2,3]))` 的容器类型不对),
    与上一条是同一种处境, 只是发生在 `as_pose` 的**数值转换**里而不是形态判定里。
    从前 (`_as_pose6` 的老版本) 这两类**正是** `InvalidCommandError`; 把形态判定整个委托
    给 `as_pose` 之后漏了这一步 ⇒ **回归**: 调用方按 `LiteArmError` 做分支的代码对 5 种
    畸形入参全部失效 (帧未发出, 方向安全, 但异常类型契约破了)。
    故这里原样转包, 文案仍保留 `as_pose` 那句的原文 (它带实际收到的形状/值)。
    """
    try:
        pos, R = as_pose(pose)
    except InvalidCommandError as e:
        raise InvalidCommandError(f"{label}: {e}") from None
    except (TypeError, ValueError) as e:
        raise InvalidCommandError(f"{label}: {e}") from None
    return list(pos) + mat_to_rpy(R)


def _as_speed(speed, label: str) -> float:
    sp = float(speed)
    if not 0.0 <= sp <= 1.0:
        raise InvalidCommandError(f"{label}: speed 需 0..1 (给的是 {speed})")
    return sp


def _seq_now(arm) -> Optional[int]:
    """此刻**已知**的最高状态帧序号 (`RobotState.seq`, `_on_status` 每帧填)。

    它是到位判据的**新鲜度闸**水位线: 见 :func:`_wait_settled`。还没见过任何状态帧时
    返回 `None` (那次等待会先拿第一帧当起算点)。

    ⚠ "已知的最高" = **已解码的最后一帧**, 不是"固件此刻的计数器" —— 主机没有后者的
    读法 (状态帧里那个 `seq` 就是全部信息)。
    """
    st = arm._require().state
    return None if st is None else st.seq


def _tcp_reached(arm, goal) -> bool:
    """到位判据满足**之后**, 回读一次**实际 TCP** 与本次请求的目标位姿比对。

    ⚠⚠ **这是"假成功"方向唯一的一道防线**: `bit10 1→0` + `q` 静止 这个观察, **同样**
    由"这条轨迹被**别的运动**作废之后的收尾"产生 —— 固件 `ctrl_accept_move_j` 的**第一条
    语句**就是 `cart_invalidate_before_motion()` ⇒ `cart_abort()`, 于是下一帧 `bit10=0`,
    臂转去执行**那条**命令; 它跑完也静止 ⇒ 只看 `bit10` 会把"中途被扯断、TCP 根本不在
    目标上"报成到位。跨进程同样成立 (本机 CDC 口不独占, 别的进程发一条 `0x01` 就能作废
    我们这条规划), **SDK 侧拦不住**。
    ⚠ 处置刻意**不是**去拦 `movej`: 它是**受控接管** (新 S 曲线从当前状态收口), 拦它等于
    把人推去用更重的 `estop`; 而 `zero_g` 丢掉位置环、靠摩擦滑停, 拦它才是推向更好的动作
    —— 两根方向相反的判断, 别抄成一条 (见 §5.3)。

    容差复用 `CART_START_POS_TOL`/`CART_START_RPY_TOL`: 与 `Arm.move_p` 的到位判据、
    `move_c` 的起点校验是**同一把尺子** (这三处的同源关系由
    `test_cart_protocol.py::test_cart_start_tolerances_track_move_p_defaults` 钉住)。

    ⚠ **取不到就报"没到位"** (超时 / 断连 / 被别的 `ERR` 串台): 保守方向, 与"宁可报未知"
    同向。故障不归这里管 —— 它由到位等待里的 `st.faulted` 立刻抛 `MotorFaultError`。
    """
    try:
        tcp = arm.get_tcp().value
    except LiteArmError:
        return False
    if tcp is None:
        return False
    return arm._pose_near(tcp, goal, CART_START_POS_TOL, CART_START_RPY_TOL)


def _wait_settled(arm, label: str, seq0: Optional[int]):
    """等"臂真的停下来" —— 返回 `(started_busy, q_final, settle_err_rad)`。

    ⚠ **判据是新写的, 不能复用 `_Ack.pump` / `Arm._arrive`**: 两者的 `done` 判据都要
    **目标关节向量**, 而笛卡尔路径的目标 `q` 在 PC 侧**不存在** (规划在固件里, PC 不发
    也不做 IK) ⇒ 这里的 `q_tol` 语义随之从"与**目标**的差"改成"与**上一帧**的差"
    (数值沿用 `0.03`)。取帧走 `Arm._read_status` (它内部就是 `_read_one`), 上限沿用
    `Arm.move_timeout` (不新造旋钮)。

    ⚠ **双判据缺一不可** (与 `Arm._arrive` 的 `done()` 同款, 两处都必须是这两条):
    相邻两帧的 q 逐轴差 < `q_tol` 连续 `arrive_frames` 帧 **且** `dq` 的 max-norm <
    `dq_tol`。只抄 `q_tol` 一半会宽松得多 —— 静态保持下 `dq` 的抖动足以把"还在爬行"
    判成"停稳"。

    ⚠⚠ **新鲜度闸** (`seq0` + `(st.seq - seq0) & 0xFFFF ∈ (0, 32768)`, u16 回绕安全):
    只信"命令发出之后生成"的状态帧。**陈旧帧直接丢弃: 既不判到位, 也不判故障** ——
    故障是锁存的, 真故障会在可信帧上重现, 用陈旧帧判故障只会造成假中止。

    ⚠ **这道闸的射程要说清** (别把它读成"运动前的帧都是它挡住的"): 命令之前生成的状态帧
    在流里一律排在**那条命令的 ACK 之前**, 而 `_request_and_wait` 必读 ACK
    (`expect(RSP_ACK)` 会一路读到它) ⇒ 它们**到不了本循环**。真正挡住"把运动前的位置报成
    到位"的是**那次 ACK 排水**; 本闸守的是另一支: 万一有帧的 `seq` 落在水线之前
    (重放 / 换取帧口 / 水线取错), 它必须被丢掉而不是拿去判到位或判故障。

    ⚠ **必须能退出**: 一旦臂在**安全包络触发**下锁存 (`control_loop.c:1713-1719` 置
    `mode = ARM_MODE_EMERGENCY` + `enabled = false`, **不动** cart FSM —— `safety_check.c`
    里一个 `cart` 字样都没有), `cart_advance()` 又排在控制拍那两个早退分支**之后**
    (`control_loop.c:1743` EMERGENCY / `:1770` 未使能 vs `:1890`), 于是 **RUNNING/READY**
    期的状态机推不动、`CART_BUSY` **常亮** (`cart_tick_timeout` 不判这两个态 ——
    `cart_exec.c:881` 只判 PLANNING)。只等 `bit10` 落 0 会一直等到超时 —— 这就是上限存在
    的理由 (另外 `st.faulted` 那条会立刻抛: EMERGENCY 置 mode=6)。
    ⚠ **但"常亮"别读成三条路径都成立**:
      · **`PLANNING` 是例外, ≤3s 自解** —— `cart_tick_timeout` 排在两个早退分支**之前**
        (`control_loop.c:1666` vs `:1743`/`:1770`), 它的 `plan_left` 到点走 `cart_abort()`
        (`cart_exec.c:889-890`) 回 `IDLE` ⇒ `bit10` 最多亮 `CART_PLAN_MAX_TICKS` = 3s
        (`cart_exec.h:42`);
      · **命令**引起的失能/急停反而**会**清掉它 —— 全固件只有四处写 `enabled = false`,
        其中三处是命令路径 (`ctrl_disable()` `control_loop.c:1207` / `ctrl_emergency_stop()`
        `:1225` / `ctrl_reset()` `:1302`), 三处都**先**调 `cart_invalidate_before_motion()`
        (`:1208` / `:1226` / `:1303`) → `cart_abort()`; 第四处就是上面那条安全锁存
        (`:1718`)。所以"常亮"只对 **安全锁存 + RUNNING/READY** 成立。

    ⚠ 断连 (`Arm.close()`) 时本循环在**下一次取帧**就拿到 `NotConnectedError` 退出
    (远小于 `move_timeout`), 不会挂到超时 —— 归因必须是"链路断了"而不是"没到位"。
    故取帧**必须**经 `arm._read_status` / `arm._require()`: 直接摸 `arm._a` 在 `close()`
    之后是 `None`, 抛出来的会是 `AttributeError` 而不是 `NotConnectedError`。

    ⚠ **帧被别的读者抢走不算异常**: `get_state()` / `get_tcp()` 不持 `Arm._cart_serial`,
    并发调用会把状态帧从本循环的取帧口抢走。所以本循环**不假设帧是自己读到的** —— 只要
    `arm._require().state` 的 `seq` 变了就重新评估判据 (同一帧用 `last_seq` 去重, 免得
    被数成 `arrive_frames` 帧)。只认"自己读到的帧"的实现会被并发读者**饿到超时**
    (假失败)。
    ⚠ 状态帧被抢走无所谓 (上面这条兜住了), **`ACK`/`0x4E` 被抢走则不可恢复** —— 那条
    请求的主人再也拿不到自己那条应答。这类"认识、但不是本次要的"帧记在
    `_Ack.foreign_frames` 上 (**八处逐处**计: 七处读循环 `_Ack.expect` / `_Ack.pump` /
    `_read_status` / `get_status_now` / `_arrive` / 本地那处 pump / `Arm._wait_until_link_lost`
    的 DFU 消失观察窗口, 外加 `Arm.poll_cart()`), 从前**连计数都没有**。
    ⚠ `Arm.poll_cart()` 也在这张计数清单里 (它同样是抢帧者, 见下面那份清单) —— 它**不是读
    循环** (只取一帧就返回), 但被它吃掉的帧同样计: "是不是循环"与"计不计"无关, 而那条帧的
    主人照样只会看到"无应答"。

    ⚠ **别的命令被拒会从本循环里炸出来**: 取帧走 `Arm._read_status`, 而它对**任意**
    `RSP_ERR` 都是直接抛 (`_err_from` 只按命令码归因, 不判"这条 ERR 是不是我这道命令的")
    —— `wait=True` 期间别人的命令被拒 (例如保活 `0x06`) 会抛成
    `CommandRejectedError` 而不是"没到位"。这是 `Arm._read_status` 的既有行为
    (`Arm._arrive` 同款), **不是本步引入的**: 串行锁只挡住笛卡尔入口, 挡不住所有下行。
    """
    budget = arm.move_timeout
    end = time.monotonic() + budget
    started_busy = False
    prev_q: Optional[List[float]] = None
    #: 判到位的那几帧的 q (`arrive_frames` 帧为窗, 静止判据一破就清空重来) ——
    #: 收尾时用它算 `settle_err_rad` (最后几帧的抖动幅度)。
    window: List[List[float]] = []
    #: 上一帧的 `seq` —— 去重: 采样是靠"等 `status_seq` 前进"驱动的, 但**等超时**时
    #: `state` 可能还是同一帧; 数两次就等于窗里少判了一帧。
    last_seq: Optional[int] = None
    while time.monotonic() < end:
        # ⚠ **[2026-09-22] 采样方式改了**: 从前是"自己取帧 + 容忍并发读者抢走", 现在
        # 读线程一直在填 `_Ack.state`, 这里只负责**等下一拍**再采它 —— 原来那段
        # "被别人读走了就采它留下的成果"整块消失(帧不会被抢)。判据仍认"帧"。
        a_ = arm._require()
        _cur = a_.status_seq
        left = end - time.monotonic()
        arm._pump_until(min(0.05, max(left, 0.0)),
                        lambda a, c=_cur: a.status_seq != c, label)
        st = arm._require().state
        if st is None:
            continue
        if st.seq == last_seq:
            continue                           # 同一帧: 不重复计数
        last_seq = st.seq
        if seq0 is None:                       # 还没有水位线: 第一帧只当起算点
            seq0 = st.seq
            continue
        if not 0 < ((st.seq - seq0) & 0xFFFF) < 32768:
            continue                           # 陈旧帧: 不判到位, **也不判故障**
        if st.faulted:
            raise MotorFaultError(f"{label}: 未到位即故障 FAULT {st.fault_detail}")
        if st.cart_busy:
            started_busy = True
        q = list(st.q)
        # ⚠ 无前一帧时**不能**短路掉 `dq` 那半边 (起点那帧一样要过 `dq_tol`): 让一个
        # `dq` 超容差的帧当上静止窗的起点, 就等于窗里少判了一帧。
        still = (all(abs(d) < arm.dq_tol for d in st.dq)
                 and (prev_q is None or (len(prev_q) == len(q)
                      and all(abs(q[i] - prev_q[i]) < arm.q_tol
                              for i in range(len(q))))))
        # 窗 = 本段**连续静止**的帧 (起点那帧没有"上一帧"可差, 但它不破坏连续性: 与
        # `_arrive` 的 `n_ok` 同款 —— 第一帧就算数); 一动就清空重来。
        window = (window + [q])[-arm.arrive_frames:] if still else []
        prev_q = q
        # 主判据: 见过 bit10=1 ⇒ 要的正是 1→0 那一刻; 降级分支: 始终没见过 1 也收
        if len(window) >= arm.arrive_frames and not st.cart_busy:
            q_final = window[-1]
            err = max((abs(v - f) for w in window for v, f in zip(w, q_final)),
                      default=0.0)
            return started_busy, q_final, err
    raise MotionTimeoutError(f"{label} 未到位, 超时 {budget}s")


def _request_and_wait(arm, cmd: int, payload: bytes, label: str, goal,
                      seq0: Optional[int] = None, wait: bool = True) -> CartPlan:
    """⑤ 登记 token + 写帧, 然后等 `0x4E` 并映射结果。**全程持 `arm._cart_serial`。**

    `request()` 的 callable **只包含那一次帧写** —— `_write_cmd` (而不是 `_raw_write`):
    它带零重力守卫, 是既有约定; 绕开它会让"零重力中发笛卡尔"变成只能在固件侧被拒。

    ⚠ **串行是强制的** (见模块 docstring): 持锁范围是**登记 → 等 ACK/ERR → 等 `0x4E`**
    —— 少一段都不行, 因为假成功正是"两条在途交错"造成的 (`ERR` 只能按命令码回显,
    `0x3A/0x3B/0x3E` 共用码空间: 先受理者的 `expect` 会认下后受理者那条被拒的 `ERR`,
    随后先受理者那条真 `0x4E` 配给后受理者的活 token ⇒ **后受理者报成功而一步没跑**)。
    单槽 pending 也决定了流水线发多条必然丢应答。

    ⚠ **持锁期间可能阻塞**, 上限**约** `move_timeout` (默认 15s; `move_path` 另含每帧
    BEGIN/ADD 的 1.2s ACK 窗口 —— 32 路点时最坏 ≈ `move_timeout + 33×1.2s`, 因为那一段
    也在同一把锁里)。这是"串行"的应有语义, 不是缺陷: 并发调用笛卡尔入口的第二个线程
    **排队**等待, 而不是与第一条交错。锁序
    (`Arm._cart_serial` → `_CartPending._lock` → `transport._wlock`) 见
    `Arm.__init__` 里 `_cart_serial` 的注释 —— 反向无环。
    ⚠ `seq0` 由调用方给 (见 :func:`_seq_now`): 它必须是**命令发出之前**的水位线,
    所以只能由"知道那一次帧写在哪"的调用方取 —— 在本函数里取就已经晚了
    (登记/写帧已经在跑)。`None` = 调用方没取到 (此前一帧状态帧都没见过),
    那就交给到位等待按"先等第一帧再起算"处理 (见 `_wait_settled`)。

    ⚠ **等 ACK/ERR 不是可选的**: 固件受理后立刻回一条 `RSP_ACK{cmd}`, 被门禁或长度校验
    拒掉时回的是 `RSP_ERR{cmd,code}` 且**不会有 `0x4E`**。不读这一条的话, 一次
    "臂未使能被拒" 会表现成耗满 `move_timeout` 之后的 `CartReplyLostError` (报"结局未知"),
    把用户推向"臂可能正在动"的错误结论。

    ⚠⚠ **这一等必须按"受理必回 ACK、拒绝绝不回 ACK"的形状读** (`err_waits_for_ack=True`,
    理由见 `_Ack.expect`): 一条**匹配命令码**的 `ERR` **证明不了**"我这条被拒了" ——
    `0x3A/0x3B/0x3E` **共用码空间**, 而流里可以有**别人的** `ERR` (最现实的一条: `probe()`
    的迟到探测应答, 见它自己的 docstring; 另一条: 上一会话/别的进程遗留)。认错的代价是
    **双向**的, 两侧都实测过:

    * 我这条**其实已被受理** ⇒ 调用方拿到假的"被拒"(臂真的会动);
    * 因此摘掉的那条活 token 让"在途 ⟹ 应答"**少一格** ⇒ 固件为它发的那条 `0x4E` 配给
      **下一条**命令 ⇒ 下一条**报成功, 而它的命令被拒、一步没跑** = **假成功**。

    故本入口**只在窗口里出现过 `ACK{cmd}` 时**才把它当"已受理"收场, 窗口耗尽仍只有 ERR
    才判"被拒"。代价: **真被拒时本入口要等满那个 ACK 窗口 (1.0s) 才返回** —— 知情的,
    换来的是"回 ERR ⟹ 本命令确实没被受理"这句话**在生成侧**成立, ⚠ **但到达侧不是**:
    受理那条 `ACK{cmd}` **自己也会丢** (固件应答 FIFO 满时丢**最新** —— 散文在
    `usb_cmd.c:63-64`, 计数点是 `:1126` 的 `rf_dropped++`; 别的读循环也能把它抢走,
    见下面的残余风险段)。那时
    窗口里只剩一条**别人的** `ERR` ⇒ 本入口判"被拒", 而本命令其实**已被受理**: 调用方
    拿到假的"被拒" (臂真的会动), 且那条真 `0x4E` 稍后到达时 —— 队列空 ⇒ `LiteArmError`
    从无关读路径炸出; 队列非空 ⇒ 配给**下一条**命令 = **假成功**。
    ⚠ 这一支**两树同形、非本次回归**: 只要"读帧在别处可以并发发生"这条前提不变就消不掉。

    ⚠ **固件显式拒绝 (`CommandRejectedError`) 时摘掉自己那条 token**: 受理前的每一道
    校验/门禁都在 `cart_req_*` 之前 `break`, 所以"**本命令的** ERR ⟹ 永远不会有 `0x4E`"
    (上面那条闸把它的成立范围缩到"**且**那条 ACK 没丢/没被抢", 见上一段)。
    留着它会占住 FIFO 队首, 让**下一条**合法笛卡尔命令的应答配给这条死 token
    (下一条报"结局未知", 死 token 却被"报成功" —— 正是本模块最防的那一侧)。
    ⚠ **这里只 `drop`, 不给吸收额度** (`clear_pending`/超时那两处才给): 本命令**确实没被
    受理** ⇒ 固件**一格应答都不欠** ⇒ 留额度只会去吞**下一条**命令自己的 `0x4E`
    (它一到就撞上额度 ⇒ 那条报"结局未知")。判"被拒"与"给额度"这两件事**不能同时做**。
    ⚠ 其它异常 (ACK 超时 / `TransportError`) **不摘**: 它们证明不了"固件不会回 `0x4E`",
    滞留在此是安全方向; 代价是超时路径每轮 `drop_and_absorb` 补一格额度, 而额度**先于**
    配对吃掉此后到达的应答 ⇒ 积压几格就有**几条**后续命令被误报"结局未知" (E5 实测)。
    ⚠ **恢复路径有三条, 别写成"只有 `reconnect()`"**:
    ① 积压的额度**被消费光** —— 每吃掉一格就不再补 (见 `drop_and_absorb` 的例外段);
    ② 一次**长过 `absorb_ttl` 的空闲** —— 额度惰性过期 (见 `_absorbance_locked`);
    ③ `reconnect()` —— 重建 `_CartPending`, 账目归零。
    ⚠ 但"不摘"的那个 token 会**永久**留在 `_unclaimed` 里 (调用方已带着异常返回) ——
    界由 `_UNCLAIMED_MAX` 兜住, 见那里。

    ⚠ `wait=True` 时**到位等待也在 `with arm._cart_serial` 之内** (不是本函数返回之后
    再跑): 一条笛卡尔命令的"整条运动"才是串行的单位 —— 释放锁再等的话, 另一个线程可以
    在持锁者还没停稳时就发下一条 (固件那条会**接管**它), 于是持锁者的到位判据观测到的
    是**别人那条**运动的收尾 = 假到位。代价是持锁期从"到 `0x4E`"拉长到"到停稳",
    上限仍是 `move_timeout` (见 `_wait_settled`)。
    ⚠ 但那把锁**挡不住锁外/进程外的行为者** (`movej`/`home`/`zero_g` 与别的进程都在
    `_cart_serial` 之外): 固件收到它们时会 `cart_invalidate_before_motion()` 把这条轨迹
    作废 —— 于是"停稳"也可能是**别人那条**命令的收尾。故 `settled` 不能只看 `bit10`,
    还要 `goal` 对得上 (见 :func:`_tcp_reached`); `goal` 由调用方给 (它知道本次请求的
    终点: `move_l`/`move_c` 用终点, `move_path` 用**最后一个**路点)。

    ⚠ **目标位姿 (`goal`) 是必填的**: 没有它就只剩"看 `bit10`"这一条判据 —— 那是本模块
    最防的**假成功**方向 (一条被作废的轨迹, 收尾照样是 `bit10=0` + `q` 静止)。留个
    `None` 默认值等于给后人留一个静默关掉校验的开关。

    ⚠⚠ **残余风险 (锁只管笛卡尔入口之间, 不是"读帧是安全的")**: 本锁只在**同一进程里
    并发的三个笛卡尔入口**之间强制串行。**读帧**这个动作本身在别处仍然可以并发发生 ——
    其它线程的 `get_state()`/`get_tcp()`、`movej` 的到位等待、**别的进程**(CDC 口不独占),
    以及 `clear_pending` 这类写口钩子, 都能把在跑那条的 `ACK{cmd}`/`0x4E` 抢走。本包**没有
    读线程** (决策 12) ⇒ 帧被谁读走就是谁的, 缓存不了、也退不回来; 症状是"受害者报
    `MotionTimeoutError` / `CartReplyLostError`, 而它的命令其实已生效"。
    ⚠ **这份清单里也包括 `Arm.poll_cart()`** (实测: 把一条在飞的 `ACK{0x3A}` 放进内核
    缓冲再调它, 那条 ACK **被它吃掉** —— 而它的主人随后报"无应答", 与 F4 修的同类)。
    它**已经**改成与笛卡尔入口互斥 (非阻塞 `acquire(blocking=False)`, 拿不到就返回 `None`),
    所以抢不走**入口**的帧; 但清单里其余那些读者都不持那把锁 ⇒ `poll_cart` 仍可能把
    **它们**的帧吃掉。这一条是**实现里检查过的**: `test_poll_cart_does_not_steal_a_frame_from_a_running_entry`。
    ⚠ 故**别**把这里读成"串行是强制的"这种无条件说法: 强制只覆盖"**入口与入口**之间"。
    ⚠ 段三又添了一个抢帧者: `Arm.enter_dfu()` 的**消失观察窗口**
    (`Arm._wait_until_link_lost`, 在 ACK 之后读满 0.3s 等设备离开) —— 它同样不持本锁,
    窗口里撞上在飞那条的 `ACK{cmd}` 就会把它吃掉 (那条请求的 `0x4E` 不受影响:
    `_read_one` 会把它交给收集器)。与 `poll_cart` 不同的是它**不能**与入口互斥:
    它要观察的是"链路还在不在", 拿不到锁就不该继续。
    (可观测性: 被别的读者丢掉的这类帧计在 `_Ack.foreign_frames` 上 —— **八处逐处**计,
    含本文件 `wait` 里那处 pump 与上面点名的 `poll_cart`; 从前**连计数都没有**。
    ⚠ `poll_cart` 不是读循环, 但被它吃掉的帧同样计 —— "是不是循环"与"计不计"无关。)
    """
    with arm._cart_serial:
        tok = arm._cart.request(lambda: arm._write_cmd(cmd, payload))
        try:
            # ⚠ `err_waits_for_ack=True` 是**载重**的 (理由见 docstring): 只按命令码比对的
            # `ERR` 会把别人的拒绝认成本条的, 而认错的代价是"假失败 + 下一条假成功"。
            arm._require().expect(P.RSP_ACK, 1.0, label, echo_cmd=cmd,
                                  err_waits_for_ack=True)
        except CommandRejectedError:
            arm._cart.drop(tok)
            raise
        # ⚠ **[2026-09-22] 这里不再传 `pump`**：`0x4E` 由读线程在 `_Ack._deliver` 里
        # 直接交给收集器, 本方法只在 token 上阻塞 (理由见 `_CartPending.wait`)。
        reply = arm._cart.wait(tok, arm.move_timeout)
        plan = CartPlan.from_reply(reply)
        raise_for_plan(plan)
        if wait:
            plan.started_busy, plan.q_final, plan.settle_err_rad = \
                _wait_settled(arm, label, seq0)
            # 到位判据满足**之后**才回读 TCP: "停稳"不等于"停在了目标上" (见 `_tcp_reached`)
            plan.settled = _tcp_reached(arm, goal)
    return plan


def move_l(arm, pose, speed: float = 1.0, wait: bool = True) -> CartPlan:
    """`Arm.move_l` 的实现 —— 笛卡尔直线 (`0x3A`)。守卫顺序见 `_request_and_wait`。

    `wait=True` (默认) 时会等到臂**停稳**才返回 (见 `_wait_settled`); 水位线 `seq0`
    取在**那一次 `0x3A` 帧写之前** —— 按"只信命令发出之后生成的状态帧"取
    (这道闸的**射程**见 `_wait_settled`, 别当成"运动前的帧全靠它挡")。
    """
    arm._require()                          # ① 未连接 -> NotConnectedError (早于登记)
    _require_cart_support(arm)              # ②
    _reject_in_zero_g(arm)                  # ③
    vec = _as_pose6(pose, "move_l")         # ④ (无副作用的参数校验)
    sp = _as_speed(speed, "move_l")
    return _request_and_wait(
        arm, P.CMD_MOVE_L, P.pack_f32s(vec) + struct.pack("<f", sp), "move_l",
        vec, seq0=_seq_now(arm), wait=wait)


def move_c(arm, pose_start, pose_via, pose_goal, speed: float = 1.0,
           wait: bool = True) -> CartPlan:
    """`Arm.move_c` 的实现 —— 笛卡尔圆弧 (`0x3B`)。

    ⚠ **先校验后发帧** (`0x3B` 受理即起跑, 没有"发出去再判"的安全顺序): 校验的是
    "**实际 TCP** 是否在 `pose_start` 附近", 容差复用 `Arm._pose_near`
    (**连"旋转等价"那条判据一起** —— 只抄逐分量会在 `|pitch|≈π/2` 万向锁时永远判失败)。

    ⚠ 水位线 `seq0` 取在 `get_tcp()` **之后**: 那次读已经把 TCP 校验要用的状态帧
    吃进来了, 取在它之前只会把水位线白白放低 (闸更松), 不会有别的效果。

    ⚠⚠ **起点校验那次 `get_tcp()` 在 `_cart_serial` 之内** (不是"取锁之前"): 它读的就是
    "此刻 TCP", 与正在跑的那条运动**本来就是同一个临界区**。放在锁外实测是这样的 (8/8):
    另一个线程正跑着一条笛卡尔轨迹时, 这一次读会把它的 `ACK{0x3A}` 抢走
    ⇒ **受害者报 `MotionTimeoutError`"无应答", 而它的命令早已写进固件并被受理** ——
    报告与物理事实相反; 而它留在队里的 token 随后还会吃掉**下一条**命令的 `0x4E`。
    ⚠ [2026-09-21] 括号里那句"(被抢走就**再也找不回来**)"**已经过时**: `_read_one` 现在
    会把本读口不要的已知帧**存进 `_Ack._pending`**，`expect` 下一次取帧就能认领回来
    （实测经 server 的 `reset`/`disable`/`emergency_stop` 由 8/12、3/8、9/12 变 12/12）。
    但**本段结论不变、这条锁也不能撤**：暂存区只解决"帧被同进程另一个读者抢走"，
    而这里要保的是**顺序**（登记 vs 固件受理）—— 那是 FIFO 配对的唯一来源，
    与帧丢不丢无关。
    ⚠ 参数校验 (①②③④) 留在锁外: 它们不读帧、不写帧, 进锁只会平白拉长持锁时间。
    """
    arm._require()                          # ①
    _require_cart_support(arm)              # ②
    _reject_in_zero_g(arm)                  # ③
    start = _as_pose6(pose_start, "move_c pose_start")   # ④
    via = _as_pose6(pose_via, "move_c pose_via")
    goal = _as_pose6(pose_goal, "move_c pose_goal")
    sp = _as_speed(speed, "move_c")
    with arm._cart_serial:                  # ⑤ 起点校验 + 发帧 + 等应答 = 一个临界区
        tcp = arm.get_tcp().value
        if tcp is None:
            raise TransportError("move_c 需要校验起点, 但取不到当前 TCP (get_tcp 无应答)")
        if not arm._pose_near(tcp, start, CART_START_POS_TOL, CART_START_RPY_TOL):
            raise InvalidCommandError(
                f"move_c: pose_start 与当前 TCP 不一致 —— 固件圆弧的起点恒为**实测 TCP**, "
                f"给的 start={['%.4f' % v for v in start]}, "
                f"实际 tcp={['%.4f' % v for v in tcp]}")
        return _request_and_wait(
            arm, P.CMD_MOVE_C,
            P.pack_f32s(via) + P.pack_f32s(goal) + struct.pack("<f", sp),
            "move_c", goal, seq0=_seq_now(arm), wait=wait)


def move_path(arm, poses, speed: float = 1.0, wait: bool = True) -> CartPlan:
    """`Arm.move_path` 的实现 —— 多路点 (`0x3C` BEGIN / `0x3D` ADD×n / `0x3E` RUN)。

    ⚠ **token 只登记在 `0x3E` (RUN) 上**: 只有 RUN 会起规划, 也只有它会产出 `0x4E`。
    `0x3C`/`0x3D` 既不在清队集合里, 也不会产生 `0x4E`。

    ⚠ **BEGIN + ADD×n + RUN 整段持 `arm._cart_serial`** —— 只圈 RUN 是不够的: 固件的
    `RECV` 收集态**只从 `IDLE` 开** (`cart_exec.c:304`: `state != CART_ST_IDLE` ⇒
    `CART_REQ_STATE`)。两条并发 `move_path` 里**后到的那条会在 BEGIN 就被拒**
    (`ERR{0x3C,0x04}`, 注释见 `cart_exec.c:299-303`), 于是它的 `_cmd` 在
    `expect(RSP_ACK, echo_cmd=0x3C)` 上抛出, **根本发不出自己的 ADD** —— 这是一次
    **响亮失败**, 不是"静默拼成一条谁也不是的路径"。持锁的作用是**把它变成排队**
    (两条都跑完, 而不是一条被拒), 并把 `BEGIN→RUN` 整段与其它笛卡尔入口互斥。
    ⚠ 本段原写"`RECV` 缓冲按**到达顺序**收路点 ⇒ 并发会拼进同一条收集态" —— **归因错**:
    缓冲按帧里的 `idx` 索引而非到达顺序 (`cart_exec.c:321`, 且重复 idx 被去重,
    `cart_exec.c:320`), 而同一时刻只可能存在**一条**收集会话 (上面那条 `IDLE` 判据)。
    ⚠ 这一段里的 `_request_and_wait` 会**再次**获取同一把锁 ⇒ `Arm._cart_serial` 是
    `RLock` (见 `Arm.__init__` 里那处注释)。

    ⚠ **BEGIN/ADD 中途失败时不做任何"清状态"的收尾**: 抛错即可 —— 固件侧 `RECV` 态带
    **2s 倒计时** (无 `RUN` 就回 `IDLE` 并丢弃已收的路点, `cart_exec.c:873-877`), 会自行
    收敛; 补发一条清状态的命令是多余的副作用。
    ⚠ 但**别拿"不占用臂"当这个结论的论据** —— `RECV` **占用臂**: BEGIN/ADD 在
    `cart_req_begin`/`cart_req_add` **之前**无条件调 `ctrl_cart_hold_begin()`
    (`usb_cmd.c:410` / `usb_cmd.c:436`), 后者 `scurve_abort_all()` + 把
    `target_q/target_dq` 锚到当前 `q_ref` + 清 `park_requested` + 置 `mode=ARM_MODE_MOVE_J`
    + `watchdog_kick()` (`control_loop.c:1547-1625`, 关键语句在 `:1606-1611` 与 `:1622-1624`)
    ⇒ 一次 `move_path` **哪怕随后被 `0x04` 拒、或中途抛错**, 也已经**当场作废了在途的逐
    关节轨迹并把参考锚住** (一次进行中的 `movej` 会被它停掉)。不占用的是 `CART_BUSY`
    (状态帧 bit10) **那一位** —— 固件只在 PLANNING/READY/RUNNING 置它, `RECV` 不置
    (`usb_cmd.c:1204-1210`)。⚠ 固件侧口径**并非全树统一**: `usb_cmd.h:156-159` 已明写这条
    "RECV 不占臂"的豁免**被撤销**, 而 `usb_cmd.c:1204` 仍留着旧口径「RECV 不占臂（只是收
    缓冲）, 不该报忙」—— 那是**固件侧的陈旧注释**(它的 bit10 代码本身是对的: 只置
    PLANNING/READY/RUNNING)。`usb_cmd.h` 才是现行口径。
    ⚠ **重试窗口是 2s**: `cart_req_begin` 只在 `IDLE` 受理 (`cart_exec.c:304`), 所以一次
    BEGIN 受理之后 (哪怕随后失败), **2s 内任何新的 BEGIN (含本 SDK 的重试) 都会拿到
    `ERR{0x3C,0x04}`** —— "会自行收敛"成立, 但要等满那个 2s 倒计时。
    """
    arm._require()                          # ①
    _require_cart_support(arm)              # ②
    _reject_in_zero_g(arm)                  # ③
    pts = [_as_pose6(p, "move_path") for p in poses]     # ④
    if not pts:
        raise InvalidCommandError("move_path: 路径为空")
    if len(pts) > 32:                       # 固件 CART_MAX_GOAL = 32
        raise InvalidCommandError(
            f"move_path: 路点数 {len(pts)} 超固件上限 32 (CART_MAX_GOAL)")
    sp = _as_speed(speed, "move_path")
    # 水位线取在 **BEGIN 之前** —— 理由是**窗口完整性**: "命令"是这一整段
    # (BEGIN+ADD×n+RUN), 而闸只信 `seq > seq0` 的帧 ⇒ 水位线取得**越晚、被排除的帧越多**,
    # 取在这一段之前才让整段的帧都落在可信窗口内。
    # ⚠ 别把方向写反 (本注释原文写的是"取在中间就会把前面几帧**算成可信帧**" —— 那是反的,
    #   闸是"只信更大的 seq")。
    # ⚠ 也不要把它说成"否则会误判到位": 真正挡住"运动前位置"的是 RUN 那次 ACK 的**排水**
    #   (`expect` 一路读到 ACK), 见 `_wait_settled`; 这里的水位线是**不变量守卫**。
    seq0 = _seq_now(arm)
    with arm._cart_serial:
        arm._cmd(P.CMD_CART_BEGIN, bytes([len(pts)]) + struct.pack("<f", sp),
                 "move_path BEGIN")
        for i, p in enumerate(pts):
            arm._cmd(P.CMD_CART_ADD, bytes([i]) + P.pack_f32s(p), f"move_path ADD {i}")
        return _request_and_wait(arm, P.CMD_CART_RUN, b"", "move_path RUN",
                                 pts[-1], seq0=seq0, wait=wait)
