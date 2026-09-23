"""Arm —— 高层子集: 直连 STM32 固件, 轨迹/IK/动力学/控制律全由固件内置承担。

设计定位 (阶段 C「pylitearm STM32 直连后端」):
  - 单发语义: movej/move_p 一次下发, 固件 S 曲线自完成 + 到位受控静止保持(B);
  - 不做 PC 侧轨迹/运动学; fk 仅当前位姿(get_tcp), ik 走固件 get_ik;
  - 笛卡尔运动只有一条路: **固件规划** move_l/move_c/move_path (PC 只发点, 收 0x4E
    结果); 阻抗控制不在本包 (高级走 pylitearm+server);
  - 连续伺服(move_js/send_mit)需调用方按 ≥10Hz 重发, 否则 0.1s 看门狗 fail-soft。

固件版本约定: get_firmware 应回 'Litearm<主.次.修>-{7J|1J}';
<1.5.0 拒绝(依赖 6+21N 状态帧/joint_fault/enabled 位 + 静止保持语义)。
"""
from __future__ import annotations

import math
import os
import struct
import threading
import time
import weakref
from dataclasses import dataclass
from typing import Any, Generic, List, Optional, Sequence, Tuple, TypeVar

from litearm import _protocol as P
from litearm import state as ST
from litearm.errors import (
    ArmIsInDfuError,
    CommandRejectedError,
    ENABLE_RETRYABLE_CODES,
    FirmwareMismatchError,
    ForkedSessionError,
    IKError,
    InvalidCommandError,
    LiteArmError,
    MotionTimeoutError,
    MotorFaultError,
    NotConnectedError,
    TransportError,
    UnsupportedByFirmwareError,
    err_reason,
)
from litearm import cart as cart_mod
from litearm.cart import _CART_CLEARS_UPON, _CartPending, CartPlan
from litearm.diagnostics import Diagnostics
from litearm.log import ArmLog
from litearm.model import ModelParams
from litearm.params import JointParams
from litearm.transport import SerialTransport, find_cdc_port

__all__ = ["Arm", "Msg", "main", "MIN_FW", "FIRMWARE_PREFIX"]

#: 后端要求的最低固件 (B1+B4 + move_j 受控静止保持语义 + 6+21N 状态帧/joint_fault/enabled 位)
MIN_FW = (1, 5, 0)
FIRMWARE_PREFIX = "Litearm"

#: 零重力保活周期。固件 `watchdog_timeout_s = 0.10s`, 且 0x06 自带 watchdog_kick
#: (「重发即保活」)—— 取 0.04s 留 2.5 倍余量。
ZG_KEEPALIVE_S = 0.04
#: 等保活线程停下来的上限 (超过说明串口写阻塞)
ZG_JOIN_S = 1.0
_ZG_THREAD_NAME = "litearm-zero_g-keepalive"

#: "零重力保活期间拒绝其它下行命令"这条守卫的**唯一**文案 —— 两个抛出点共用:
#: :meth:`Arm._write_cmd` (兜底, 覆盖"检查与写之间 `_zg_active` 翻转"的竞态) 与
#: `cart._reject_in_zero_g` (让判定发生在**登记 token 之前**)。
#:
#: 为什么必须是**同一个常量**: 两处的判据必须一致 —— 从前是各抄一份逐字节相同的字符串,
#: 改一侧就静默漂移 (调用方按文案归因, 两句不一样会被读成两种不同的拒绝)。文案本身仍是
#: 用户可见的**归因入口**: 它要说清"为什么拒"与"怎么退出"。
ZERO_G_GUARD_MESSAGE = (
    "零重力保活正在进行, 拒绝其它下行命令 (会改写模式/看门狗, 与保活互相打架); "
    "先 arm.zero_g_stop() 退出")

#: "笛卡尔在途时拒绝进入零重力"这条**反向**守卫的文案 (见 `Arm._reject_if_cart_in_flight`)。
#:
#: 只拒绝而不给出路会把操作员卡住 —— 文案必须把**更好的停机动作**写在里面:
#: `arm.movej()` 是受控接管 (新 S 曲线从当前状态收口), 比 `0x06` 的"丢位置环、靠摩擦
#: 滑停 (coast)"好得多; `arm.emergency_stop()` 是更重的兜底; 或者干脆等它结束。
CART_IN_FLIGHT_GUARD_MESSAGE = (
    "笛卡尔运动在途, 拒绝进入零重力 (中途进场会丢掉位置环、只剩重力前馈, "
    "臂会靠摩擦滑停): 先 arm.movej() 受控接管收口, 或 arm.emergency_stop() 急停, "
    "或等它结束再进入")

#: 上面那条的"**没能确认**是否在途"变体 —— 保守拒绝时必须说清是哪一种拒绝。
CART_IN_FLIGHT_UNCONFIRMED_MESSAGE = (
    "取不到状态帧, 无法确认笛卡尔是否在途 —— 保守拒绝进入零重力 (未确认就放行, "
    "漏的正是'轨迹中途进场靠摩擦滑停'那一侧): 先 arm.get_state(refresh=True) "
    "确认臂已停稳, 再重试")

#: `Arm.enter_dfu()` 的本地**使能态**预检文案 (状态帧 bit9 = 固件 `g_arm.enabled`)。
#:
#: 为什么使能中不许跳 (固件 `control_loop.c:1088-1090` 的原注释): 跳转停 TIM3 ⇒
#: 不再发 MIT 帧 ⇒ 电机侧 `RID_TIMEOUT`(100ms) 松开 ⇒ **有重力负载则臂下垂**。
#: ⚠ 本地预检的口径**只有** `enabled` —— 固件那道门禁是 `ctrl_is_armed() =
#: enabled || enable_pending` (定义在 `control_loop.c:1334-1336`; `:1091` 是
#: `ctrl_request_enter_dfu` 里把**同一谓词内联**写的那一行), SDK 从状态帧**看不到**
#: `enable_pending` ⇒ 它在途时会放行到固件, 由固件回 `ERR{0x15,0x03}`
#: (那一条的处置见 `errors.ERR_TEXT[(0x15, 0x03)]`: 照实透出, **不是**"登记被撤销")。
DFU_ENABLED_MESSAGE = (
    "使能中拒绝进入 DFU —— 跳转停 TIM3 后电机 100ms 就松开, 有重力负载会下垂; "
    "先 arm.disable() 再调 enter_dfu()")

#: `Arm.enter_dfu()` 的"**登记被撤销/未执行**"文案 —— 收到 `ACK{0x15}` 之后设备
#: 在等待窗口内**没有**从 CDC 上消失。
#:
#: `ACK{0x15}` 只表示"已登记" (`control_loop.c:1096-1097`), 而 `dfu_pending = false`
#: 的**静默撤销**点有**三个** (都不回报, 且都发生在 main/控制拍里): 并发 ENABLE
#: (`:1106`)、复读向量表失败 (`:1142`) 与 1s 总超时兜底 (`:1112`)。⚠ 第三条 (1s 兜底)
#: **实际不可达**: 10ms/100ms/100ms 三道门控 (最短 `:1118`、两道 `:1131`/`:1137`) 全在
#: `age <= 100ms` 处放行, 而 1s 判据要 `age > 300` 拍 (`:1111`) ⇒ 到点必然先在
#: `:1142`/`:1156` 落定, 走不到 `:1112`。所以"ACK 到了但设备没走"是**可达**的正常
#: 结局 —— 这条异常就是它的**唯一出口**: 说清"什么都没发生", 并让 `Arm` 保持可用
#: (那条消息里必须能读出"没跳", 否则现场会误判成跳失败的硬件问题)。
DFU_REVOKED_MESSAGE = (
    "DFU 登记被撤销/未执行 —— 固件回了 ACK 但设备在等待窗口内没有离开 CDC "
    "(并发使能或交棒前向量表复读失败都会静默清掉登记, 都不回报); "
    "臂与控制链路原样可用, 确认已失能后重试即可")

#: `Arm.enter_dfu()` 等 `RSP_ACK{0x15}` 的窗口 (与 `_cmd` 的 1.2s 同档)。
DFU_ACK_TIMEOUT_S = 1.2

#: `Arm.enter_dfu()` 观察"设备真的消失"的窗口 —— 取 **0.3s 量级** (spec §6.1)。
#: 固件侧上界是 **100ms**: 三道前置门控 (最短 10ms / 等 ACK 出 USB 100ms / 等 CAN 空闲
#: 100ms) **共用同一个 `age`** (都自登记那刻起算, `control_loop.c:1110-1137`) ⇒ 到点
#: 必然交棒或清 pending。⚠ **不是 10+100+100=210ms** (规格初稿这样加过, 已订正) ——
#: 所以本窗口留的是 **USB 重枚举的余量**, 不是"等固件那个上界"。
DFU_VANISH_TIMEOUT_S = 0.3


def _rpy_to_R(rpy: Sequence[float]):
    """rpy(roll, pitch, yaw) -> 3x3 行主序旋转矩阵 (扁平 9 元组)。

    **逐字镜像固件 `kin_rpy_to_rot`** (`kin.c:343-358`, `R = Rz(yaw)·Ry(pitch)·Rx(roll)`)。
    判到位必须与固件同一约定, 否则比的是两种不同的姿态表示。

    实现委托给 `_rot.rpy_to_mat` —— 同一约定**只留一份代码**。先前这里与
    笛卡尔子包各写一遍, 是典型的漂移源: 两处都"当下一致", 改一处不漏另一处就静默
    变成两种姿态表示。转置/翻转矩阵仍走 PID 标定, 与固件 `tools/*.py` 同表。
    """
    from litearm import _rot
    m = _rot.rpy_to_mat(rpy)
    return (m[0][0], m[0][1], m[0][2], m[1][0], m[1][1], m[1][2],
            m[2][0], m[2][1], m[2][2])


def _is_single_pose(x) -> bool:
    """这是**单个位姿**还是**位姿序列**? —— `move_p` 的唯一调用点 (它只收单个)。

    保留理由 (原本它是"单点/序列"的分派判据, 序列那条实现已随 PC 侧规划一起退役):
    它现在判的是"该不该抛那句指名 `move_path` 的错", 而**形状歧义仍然存在** ——
    ``[pose1, pose2]`` 与 ``(pos[3], R[3x3])`` 都是"长度为 2 且两个元素都是序列",
    只看长度会把合法的位姿对判成序列 (反过来说错话)。这套逐层验形状的逻辑留着,
    比在 `move_p` 里重写一份短判据安全 —— 那正是两处判据各自漂移的老路。

    * ``[x,y,z,r,p,y]``   6 个标量
    * ``(pos[3], R[3x3])`` pylitearm 形式
    * 4x4 齐次矩阵
    """
    if not isinstance(x, (list, tuple)) or isinstance(x, (str, bytes)):
        return False
    n = len(x)
    if n == 4 and all(isinstance(r, (list, tuple)) and len(r) == 4 for r in x):
        return True
    if n == 6 and not any(isinstance(v, (list, tuple)) for v in x):
        return True
    if n == 2 and all(isinstance(v, (list, tuple)) for v in x):
        p, R = list(x[0]), list(x[1])
        return (len(p) == 3 and len(R) == 3
                and all(isinstance(r, (list, tuple)) and len(r) == 3 for r in R))
    return False


def _orient_angle(a: Sequence[float], b: Sequence[float]) -> float:
    """两组 rpy 所代表旋转之间的测地夹角 (rad)。`trace(Raᵀ·Rb)` 求夹角。"""
    Ra, Rb = _rpy_to_R(a), _rpy_to_R(b)
    c = sum(Ra[i] * Rb[i] for i in range(9))          # trace(Raᵀ Rb)
    return math.acos(max(-1.0, min(1.0, (c - 1.0) / 2.0)))


def _err_from(payload: bytes, prefix: str) -> CommandRejectedError:
    """把固件 ERR 应答映射成异常 (可读文本查 `errors.ERR_TEXT`, 见 spec §6.3)。

    `code == 0x00` ⟺ 固件 `usb_cmd.c` 的 `default` 分支 —— 即**未实现该命令**,
    且固件侧无任何副作用 (唯一真源, 由 `tests/test_protocol_sync.py` 钉住)。
    已实现命令的错误码一律落在 `0x01..0x07`。

    ⚠ 消息里的语义文本**不在这里写死**: 走 :func:`litearm.errors.err_reason`,
    它的第 2/3 档会**带上原始码** —— 固件新增一档时这里是唯一会如实说出
    "我没见过这个码"的地方 (折叠成通用文本 = 把它藏起来)。
    `.cmd`/`.code` 两个字段照旧供调用方编程判定 (它们是本异常存在的理由)。
    """
    cmd = payload[0] if len(payload) > 0 else 0
    code = payload[1] if len(payload) > 1 else 0
    base = f"{prefix}ERR [{cmd:02X},{code}]"
    if code == 0x00:
        return UnsupportedByFirmwareError(
            f"{base} —— 固件没有实现这条命令 (需烧入引入该命令的固件版本)",
            cmd=cmd, code=code)
    return CommandRejectedError(f"{base} —— {err_reason(cmd, code)}",
                                cmd=cmd, code=code)


class _ZeroGSession:
    """`Arm.zero_g()` 的返回值: 既已启动保活, 又可用作上下文管理器。"""

    def __init__(self, arm: "Arm"):
        self._arm = arm

    def __enter__(self) -> "Arm":
        return self._arm

    def __exit__(self, exc_type, exc, tb) -> bool:
        # 保活若已中断则在此抛出; 但不得掩盖 with 体内原本的异常
        self._arm.zero_g_stop(raise_on_lost=exc_type is None)
        return False


#: **以回显命令码为归属判据**的上行 id。只有这两类帧的 `payload[0]` 是"原命令码"，
#: 因此只有它们需要第二维来区分同 id 的不同命令（`ACK{0x10}` vs `ACK{0x11}`）。
#: ⚠ 别的帧的 `payload[0]` 是**数据**（关节号 / item 索引 / 文本首字节），拿它当归属键是错的。
_ECHOED = frozenset({P.RSP_ACK, P.RSP_ERR})

#: 每条应答队列的深度上限。真堆到这个数说明主人再也不来取了（孤儿），丢**最旧**的并计
#: `_Ack.dropped` —— 与全包对"丢"的一贯口径一致（不静默）。
_QUEUE_MAX = 64

#: 读线程每一拍允许阻塞的时长（秒）。
_READ_SLICE_S = 0.1

#: 读线程读到"没有帧"时的**退避**（秒）—— 见 `_Ack._reader_loop`。
#: ⚠ **不能依赖"传输会阻塞"**：那是 `SerialTransport` 的实现性质，不是传输接口的契约 ——
#: 测试桩就完全忽略 `timeout`。实测（`FakeTransport`）不退避的空循环跑出
#: **7 992 652 次/秒**（100% CPU）。真机上这 1ms 只发生在本来就空闲的那一拍。
_IDLE_SLEEP_S = 0.001


def _read_key(c: int, payload: bytes) -> tuple:
    """一条帧该进哪条队列 —— **(上行 id, 回显码或 None)**。

    帧的**归属 = 它落在哪条队列**。这是本设计的全部秘密：`ACK{0x10}` 与 `ACK{0x11}`
    进**两条**队列，因此永远配不错。
    """
    if c in _ECHOED and payload:
        return (c, payload[0])
    return (c, None)


def _wait_keys(want: int, echo_cmd: Optional[int]) -> tuple:
    """等一条 `want` 应答时该看哪几条队列。

    ⚠ `want` 是 `RSP_ACK`/`RSP_ERR` 时 **`echo_cmd` 必填**：归属判据就是那个回显码，
    不给就等于"随便哪条 ACK 都算我的" —— 那正是并发下两条命令互吃应答的成因
    （真机实测：到达序与等待序不一致时 100% 有一方丢）。库内 11 处调用点**全都给了**
    （`ast` 实测），故这里**直接拒绝**而不是退化成通配。
    """
    if want not in _ECHOED:
        # ⚠ 给了 `echo_cmd` 就**还要**等"回显了本命令码的 ERR" —— 那是本命令被拒的唯一
        # 载体（`expect` 的 `raise_on_err` 靠它）。少了这一条，一次**被拒**会退化成
        # "无应答"超时（实测：`model.probe()` 里 `0x34` 的 `ERR{0x34,0x00}` 丢失）。
        if echo_cmd is None:
            return ((want, None),)
        return ((want, None), (P.RSP_ERR, echo_cmd))
    if echo_cmd is None:
        raise InvalidCommandError(
            f"等 {want:#04x} 必须给 echo_cmd —— 它的归属判据是**回显的命令码**; "
            f"不给就分不清'我的 ACK'与'别人的 ACK' (并发下会互吃)")
    if want == P.RSP_ACK:
        # 受理回 `ACK{cmd}`、拒绝回 `ERR{cmd,code}` (**绝不补 ACK**) ⇒ 两条队列都得等
        # (`err_waits_for_ack` 与 `raise_on_err` 都靠这一条)。
        return ((P.RSP_ACK, echo_cmd), (P.RSP_ERR, echo_cmd))
    return ((want, echo_cmd),)


_T = TypeVar("_T")


@dataclass(frozen=True)
class Msg(Generic[_T]):
    """**一帧的值 + 它到达的统计** —— 11 个"读一帧"型 getter 的返回信封。

    形态借自松灵的同名结构 (`{msg, hz, timestamp}`), 语义由本包定:

    * `value` —— 原返回值, **逐字相同**。取不到帧的入口 (`get_state`/`get_tcp`)
      `value=None`, 即"没取到"这件事从"返回 `None`"变成"`Msg.value is None`";
    * `hz` —— 该类帧在本会话里的**平均到达频率**, 见 `_Ack.recv_stats`;
    * `timestamp` —— 该类帧**最近一帧**的本地 `time.monotonic()`
      (本会话从没收到过该类帧时为 `0.0`)。

    ⚠ **这是破坏性变更** (2.0.0): 这 11 个入口的返回值从 `T` 变成 `Msg[T]`,
    既有调用方要读 `.value`。被包的 11 个: `get_state` / `get_status_now` / `get_tcp` /
    `get_ff_vec` / `get_ff_scalar` / `params.get_joint_param` / `model.get_body` /
    `model.get_jm` / `model.status` / `model.get_gravity` / `diag.kin_bench`。

    ⚠ **刻意不包的**这些 (别顺手全包):
      · `move_*` (`movej`/`move_j*`/`move_p`/`move_l`/`move_c`/`move_path`) 与 `home()`
        —— 它们返回 `RobotState`/`CartPlan`, 语义是"**动作结果**"不是"读一帧";
      · 纯本地量 (`n` / `firmware` / `last_reset_reason`) —— 根本没有帧;
      · `license()` —— `RSP_LICENSE` 是请求/应答式的**设备身份记录**（激活后不可变）,
        **没有固件发起的流量**, 故 `hz`/`timestamp` 只会度量"调用方自己轮询的频率"。
        理由见那个方法;
      · 另有两个**派生** getter 也保持裸值, 理由见 `get_ff_mask` / `all_joint_params`。
    """

    value: _T
    hz: float
    timestamp: float


#: `RSP_LICENSE` 的 `state` 字节 -> 可读名 (逐值取自固件 `params/license.h:138-139`)。
LICENSE_STATES = {0: "not_activated", 1: "activated", 2: "activated_factory"}


@dataclass(frozen=True)
class LicenseInfo:
    """设备授权记录 —— `CMD_GET_LICENSE(0x2F)` 的应答 (`RSP_LICENSE 0x4F`, 26B)。

    ⚠ **未激活时也回 UID, 而 `cust_id`/`issued`/`flags` 全 0** —— 靠"调用方预置零 +
    `license_get_info()` 在未激活时不写输出指针"两条同时成立 (`usb_cmd.c:1019-1020`)。
    签发工具**必须**从本记录取 `uid`（`uid_hex` 就是它要的形态）—— **不得**改用 USB
    序列号字符串, 两者不是一个东西。

    ⚠ 本包**不解释** `uid` 那 12 字节的语义 (固件是 `HAL_GetUIDw0/1/2` 各 4B 直接
    memcpy, 原始寄存器内存序), 只把设备给的原样转交 —— 签发器读的也是同一份字节,
    故二者天然自洽。
    """

    state: int           #: 0=未激活 / 1=已激活 / 2=已激活且产线模式
    ver: int             #: 记录版本 (`LICENSE_REC_VER`, 当前 1)
    uid: bytes           #: 12B, 本机 UID (原始寄存器内存序)
    cust_id: int         #: 客户号 (未激活恒 0)
    issued: int          #: 签发日 YYYYMMDD (未激活恒 0)
    flags: int           #: bit0 = 产线码 (未激活恒 0)

    @property
    def activated(self) -> bool:
        """是否已激活 —— 判据只是 `state != 0` (未激活时其余字段无意义)。"""
        return self.state != 0

    @property
    def factory_mode(self) -> bool:
        """产线码 (`flags` bit0) —— **不表示"激活与否"**, 故别拿它替代 `activated`。"""
        return bool(self.flags & 0x1)

    @property
    def state_name(self) -> str:
        """`state` 的可读名; 认不出的码**带上原值**回 (不静默)。"""
        return LICENSE_STATES.get(self.state, f"unknown_state_{self.state}")

    @property
    def uid_hex(self) -> str:
        """UID 的 24 位小写 hex —— **签发器要的就是这个形态**。"""
        return self.uid.hex()

    @classmethod
    def decode(cls, payload: bytes) -> "LicenseInfo":
        # ⚠⚠ **本帧载荷的首字节是 `state`, 不是冗余帧 id** —— 与 `RSP_MODEL_STATUS` /
        #    `RSP_FF_VEC` / `RSP_FF_SCALAR` 那一类（固件把本 id 冗余回填进 `payload[0]`）
        #    **不同**。别照抄它们的 decode 去比 `payload[0] == P.RSP_LICENSE`。
        #    两条独立旁证: ① 固件 `usb_cmd.c:1017-1028` 的 `b[0] = license_state()`;
        #    ② 而 `usb_cmd_reply()` 的 `memcpy(&b[3], payload, len)` 说明**帧头不额外预置 id**
        #    (`usb_cmd.c:1210-1217`) ⇒ 主机收到的 `payload` 就是那 26 字节。
        #    厂商侧工具为此专门留了一句警告 (`tools/litearm_license/_proto.py:227-228`):
        #    按前缀规则匹配 `RSP_LICENSE` 会**永远匹配不上**。
        if len(payload) != 26:
            raise TransportError(f"RSP_LICENSE 帧长 {len(payload)}B, 期望 26B")
        return cls(
            state=int(payload[0]),
            ver=int(payload[1]),
            uid=bytes(payload[2:14]),
            cust_id=struct.unpack_from("<I", payload, 14)[0],
            issued=struct.unpack_from("<I", payload, 18)[0],
            flags=struct.unpack_from("<I", payload, 22)[0],
        )


class _Ack:
    """send→期望应答的小型等待器 (状态帧入缓存, ACK/ERR/目标应答命中)。

    同时也是**帧语义的持有者**: 两个"被丢弃的帧"的计数都记在这里 (累积计数, 不是
    "见过就置位"的布尔):

    * `unexpected_frames` —— **未识别 id** (由 `Arm._read_one` 递增);
    * `foreign_frames` —— **认识、但不是本命令要的那条** (`ACK`/`ERR` 回显了别人的
      命令码), 由**丢弃它的那一处**在丢弃的那一刻递增 (见 `_foreign`)。
      ⚠ 这一条必须**逐处**做: `unexpected_frames` 由 `_read_one` 一个口统一计, 而"这不是
      我要的帧"只有**取帧的那一处自己**知道 ⇒ **八处**各计各的 —— **七处读循环**:
      `_Ack.expect` / `_Ack.pump` / `_read_status` / `get_status_now` / `_arrive`
      (这五处在 `arm.py`) + `cart._CartPending.wait` 的 pump 循环 (在 `cart.py`, 经
      `Arm._note_foreign` 转发) + `Arm._wait_until_link_lost()` 的**消失观察窗口**
      (DFU, 在 `arm.py`); 外加 **`Arm.poll_cart()`** 的单帧非阻塞取帧
      (`_read_one(0.0)`) —— 它**不是读循环** (取一帧就返回), 但"帧被吃掉就再也找不回来"
      与"是不是循环"无关。
      ⚠⚠ [2026-09-21] **这份"逐处计数"的账已经大半年不是全貌了**: `_read_one` 现在会把
      "本循环不要的**已知**帧"（`RSP_STATUS`/`RSP_ERR` 除外）**存进 `_Ack._pending`**
      而不是丢掉 ⇒ 那几处**不再计 `foreign_frames`**、帧也不再丢。今天**仍然会丢帧**的只剩:
      `want is None` 的通配读口（`poll_cart`、`cart` 的 pump **在它传 `want` 之前**）与
      `expect` 里"回显了别人命令码"的那一支（它必须**亲眼看到**那条 ACK 才能比回显）。
      下面这份清单留着是因为它记录了**每个站点在丢弃发生时的计数职责** —— 那种情况
      今天仍可能出现，别把它读成"这些站点今天都在丢帧"。
      ⚠ 从前只有两处做 (`expect` 与 `_read_status`), 而 `Arm._read_one` 的 docstring 点名的
      `_arrive` 恰好是没做的那一个 (实测: 逐处各塞一条外来 `ACK`, 只有那两处涨计数);
      `cart` 那处是后来补齐的一处 (而它偏偏是持锁最长的一段), `poll_cart` 是紧随其后的
      一处 (它从前在本 docstring 里被**单列**成"已知缺口", 现已补上), `_wait_until_link_lost`
      是**段三最后**补的一处 (见 `tests/test_read_one.py` 里那几条同形用例, 以及
      `tests/test_dfu.py` 里专给它的一条)。
    """

    def __init__(self, arm: "Arm"):
        #: ⚠ **弱引用**：读线程不能钉住 `Arm`。线程的 target 是绑定方法 ⇒ 线程持有一个
        #: 对 `_Ack` 的强引用，若 `_Ack` 再强引 `Arm`，那么**忘了 `close()` 的 `Arm` 再也
        #: 不会被回收** ⇒ `Arm.__del__` 的兜底收尾永不触发 ⇒ 端口不释放（实测：
        #: `test_teardown.py` 的六条兜底用例全红）。弱引用把这条边断掉，兜底就能回来。
        self._arm = weakref.ref(arm)
        # ---- 读线程 (起停见 `Arm.connect()` / `Arm.close()`) ----
        self._reader: Optional[threading.Thread] = None
        #: 退出信号 —— 读线程在**退避时**也等它（见 `_reader_loop`）。
        self._stop_evt = threading.Event()
        #: 读线程的**死因**（传输层异常）。非 None = 链路没了。`_wait` 会抛它,
        #: `Arm._wait_until_link_lost` 会**把它当成"设备已消失"的判据** ——
        #: 故"正常关闭"绝不许写这一格（顺序见 `stop_reader`）。
        self._reader_error: Optional[BaseException] = None
        #: 投递时**协议层**抛出的异常（典型: `cart.on_reply` 的"多了一条"）。
        #: 读线程**不当场抛、也不死**（见 `_reader_loop`），存这里由**下一个等待者**抛出，
        #: 抛一次就出队 ⇒ 不会 50Hz 刷屏。
        self._errors: list = []
        # ---- 共享状态: 全部由 `_cond` 保护 ----
        #: 全会话**唯一**的锁 + 条件变量。投递与等待都走它，投递后 `notify_all`。
        #: ⚠ **叶子锁的加强版**：只在 `_deliver`/`_wait` 内部持有，且 `_deliver` 里
        #: **不调用任何子系统**（`cart.on_reply` 刻意留在锁外，见那里）。
        self._cond = threading.Condition()
        #: **应答队列表** —— `(上行 id, 回显码或 None) -> [(id, payload, seq), ...]`。
        #: 按**到达序**。帧的归属就是它落在哪条队列（见 `_read_key`）。
        self._queues: dict = {}
        #: 单调递增的**到达序号**，逐帧盖章。等待者拿它当水位：只认自己"发出之后"
        #: 到的那条（见 `_wait` 的 `since`）。
        self._seq = 0
        #: 因**队列封顶**被挤掉的帧累计条数。它合并了从前的 `unexpected_frames` 与
        #: `foreign_frames` —— 两者含义本就相同：「没等到主人的帧」。
        #: ⚠ 只做可观测性，不改变任何行为。
        self.dropped = 0
        #: **per-type 接收表** —— `上行 id -> [count, first_monotonic, last_monotonic]`。
        #: 返回信封 (`Msg`) 的 `hz`/`timestamp` 唯一来源，由读线程在 `_deliver` 里记。
        #: ⚠ 表里多存一个 `first`：`hz` 的分母必须是"**实际到达的间隔**" —— 只有
        #: `count` 与 `last` 是**没有分母**的（要么退化成"自 connect 起"而随链路空闲
        #: 一起衰减，要么只能上 EWMA 那样一个拍脑袋的时间常数）。
        #: 与 `_Ack` 同寿 ⇒ 本会话的统计，`close()`/`connect()` 天然清零。
        self._recv: dict = {}
        #: **状态单槽** + 它的到达计数。100Hz 连续流**不进队列**：进队列会在 0.6s 内
        #: 撑满 `_QUEUE_MAX` 并制造恒定的淘汰噪声；而且它是**广播**帧（每个消费者都要），
        #: 队列"取走就没"的语义天生不适合它。等待者靠 `status_seq` 判"有没有新的"。
        self.state: Optional[ST.RobotState] = None
        self.status_seq = 0

    # ---- 读线程侧: 只投递, 不判定 ----

    def start_reader(self, tr) -> None:
        """起读线程。**唯一调用点**是 `Arm.connect()`（时序见那里）。"""
        self._stop_evt.clear()
        self._reader = threading.Thread(
            target=self._reader_loop, args=(tr,), daemon=True, name="litearm-reader")
        self._reader.start()

    def stop_reader(self) -> None:
        """停读线程。**必须排在 `transport.close()` 之前**。

        否则读线程会从**已经关掉的**传输上抛 `TransportError` ⇒ `_reader_error` 非空
        ⇒ 一次**正常关闭**被记成"链路丢失"，而 `Arm._wait_until_link_lost` 正是拿
        `_reader_error` 当"设备没了"的判据。

        ⚠ 也必须 `join`：线程的 target 是绑定方法 ⇒ 线程强引用本 `_Ack`→`Arm`，
        而活着的线程又被 `threading._active` 强引用 ⇒ **不 join 就回收不掉 `Arm`**
        （与既有的 `zero_g` 保活线程同款，见 `Arm.__del__` 里那段）。
        """
        self._stop_evt.set()              # 立刻唤醒正在退避的读线程
        th = self._reader
        if th is not None and th.is_alive():
            th.join(1.0)

    def _reader_loop(self, tr) -> None:
        """**唯一的读者** —— 全包只有这一个地方碰 `transport.read_frame`。

        本设计的全部代价就是下面两条纪律：

        1. **只投递，不判定** —— "这帧是谁的"由队列决定，**不由线程决定**；
        2. **死了要响亮** —— 死因进 `_reader_error` 并唤醒所有等待者，绝不静默退出。
        """
        while not self._stop_evt.is_set():
            try:
                fr = tr.read_frame(_READ_SLICE_S)
            except Exception as e:                      # noqa: BLE001
                self._die(e)
                return
            if fr is None:
                # ⚠ **退避**：`SerialTransport` 空闲时会阻塞满 `_READ_SLICE_S`，但
                # **传输接口没有这个契约** —— 测试桩就完全忽略 `timeout`。
                # ⚠⚠ **用 `Event.wait` 而不是 `time.sleep`**：测试拿 monkeypatch 过的
                # `time.sleep` 当"等了多久"的探针（`enable` 的重试间隔），读线程若也走它，
                # 那些探针会抓到成千上万次 (实测 76834 次)。`Event.wait` 既避开这点，
                # 又让 `stop_reader` 不必等满这一拍。
                self._stop_evt.wait(_IDLE_SLEEP_S)
                continue
            try:
                self._deliver(fr[0], fr[1], time.monotonic())
            except Exception as e:                      # noqa: BLE001
                # 协议层异常（`cart.on_reply` 的"多了一条"）：**不当场抛、也不死**，
                # 存起来交给下一个等待者（见 `_errors`）。
                with self._cond:
                    self._errors.append(e)
                    self._cond.notify_all()

    def _die(self, e: BaseException) -> None:
        """链路没了 —— 记死因并唤醒所有等待者（**绝不静默退出**）。"""
        with self._cond:
            self._reader_error = e
            self._cond.notify_all()

    def _deliver(self, c: int, payload: bytes, now: float) -> None:
        """把一帧投递到它该去的地方 —— 本包**唯一**的分发点。"""
        cart = None
        with self._cond:
            self._note_recv(c, now)
            if c == P.RSP_STATUS:
                self._on_status(payload)
                self.status_seq += 1
            elif c == P.RSP_CART_PLAN:
                owner = self._arm()
                cart = None if owner is None else owner._cart   # 收集器在锁外调（见下）
            else:
                k = _read_key(c, payload)
                q = self._queues.get(k)
                if q is None:
                    q = self._queues[k] = []
                elif len(q) >= _QUEUE_MAX:
                    q.pop(0)                     # 孤儿: 丢最旧, 不静默
                    self.dropped += 1
                q.append((c, payload, self._seq))
                self._seq += 1
            self._cond.notify_all()
        # ⚠ `on_reply` 必须在 `_cond` **之外**：它要去取 `_CartPending._lock`，
        # 在全局锁里调另一个子系统的锁，那边一慢就会把**所有**等待者一起卡住。
        if cart is not None:
            cart.on_reply(payload)

    # ---- 等待者侧 ----

    def drain_for(self, cmd: int) -> None:
        """清掉 `cmd` 的应答队列 —— **由 `Arm._raw_write` 在发帧之前调**。

        这是"陈旧帧冒充新应答 ⇒ 假成功"的**唯一**一道闸，位置是**载重**的：

        * **必须在写之前** —— 我们的应答只可能在写之后到达，故清队**不可能**吃掉
          自己的应答；反过来（写在清队之前）会 100% 吃掉，实测在桩上直接让
          `connect()` 的固件握手超时。
        * **必须与写同一个线程、紧挨着** —— 清队与写之间只隔几微秒，那几微秒里到达的
          陈旧帧是**残余风险**（知情的、不可再约：线上没有请求 id）。

        ⚠ 因此 `Arm._raw_write` 是它**唯一**的调用点（全包唯一的写口）。
        ⚠ 残余风险的**方向对比**：清晚了会吃自己的应答（**假超时**，安全侧）；
        不清则上一条已放弃命令的迟到应答会被当成本次的（**假成功**，危险侧）。
        """
        keys = [(P.RSP_ACK, cmd), (P.RSP_ERR, cmd)]
        rsp = P.RSP_OF_CMD.get(cmd)
        if rsp is not None:
            keys.append((rsp, None))
        with self._cond:
            for k in keys:
                q = self._queues.pop(k, None)
                if q:
                    self.dropped += len(q)

    def _wait(self, keys, timeout: float) -> Optional[tuple]:
        """在 `keys` 这几条队列上等**最早到达**的那条；超时返回 `None`。

        ⚠ 队列**在发帧之前刚被 `drain_for` 清过**（见那里）⇒ 这里**不需要水位**：
        队列里出现的东西一定是写之后到达的。这是本设计比"盒子 + `since` 时间戳"
        更简的一处 —— 少一个概念、少一个方向判错的余地。

        ⚠ 多条 key 都有货时取 `seq` **最小**的那条（`ACK` 与 `ERR` 可能同时在队，
        而 `err_waits_for_ack` 要求按到达序先看到 `ERR`）。

        ⚠ 三个守卫排最前，一个都不能少 —— 它们从前挂在 `_read_one` / `_request_and_get`
        上，取帧口搬走了就得跟着搬，漏哪条都是从"响亮地报错"静默退化成"等满超时"：
        ① 终态 → ② 未连接 → ③ 负 timeout。
        """
        owner = self._arm()
        if owner is None:
            # 会话对象已被回收（`close()` 之后没人再持有它）—— 与"链路已关"同一类。
            raise NotConnectedError(
                "链路已关闭 (Arm.close() 之后) —— 请先 connect()")
        owner._reject_if_in_dfu()                   # ① ⚠ 必须排在"已关"之前
        owner._reject_if_wrong_process()            # ⓪ fork 守卫: 唯一取帧口
        if owner._tr is None:                       # ② 链路已关
            raise NotConnectedError(
                "链路已关闭 (Arm.close() 之后) —— 请先 connect()")
        if timeout < 0.0:                           # ③ 负值没有等待语义
            raise InvalidCommandError(
                f"timeout 不能为负 (给的是 {timeout!r}) —— 负值没有等待语义; "
                f"要'不等、只探一次'请传 0.0")
        end = time.monotonic() + timeout
        with self._cond:
            while True:
                best = None
                best_key = None
                for k in keys:
                    q = self._queues.get(k)
                    if q and (best is None or q[0][2] < best[2]):
                        best, best_key = q[0], k
                if best is not None:
                    self._queues[best_key].pop(0)
                    return (best[0], best[1])
                if self._errors:
                    raise self._errors.pop(0)
                if self._reader_error is not None:
                    raise TransportError(f"读线程已退出: {self._reader_error}")
                left = end - time.monotonic()
                if left <= 0.0:
                    return None
                self._cond.wait(left)

    def _note_recv(self, c: int, now: float) -> None:
        """记一帧到达 —— 本表**唯一**的写口 (`Arm._read_one` 调用, 与那里同一个理由)。"""
        ent = self._recv.get(c)
        if ent is None:
            self._recv[c] = [1, now, now]
        else:
            ent[0] += 1
            ent[2] = now

    def recv_stats(self, c: int) -> Tuple[float, float]:
        """本会话里 id `c` 那类帧的 `(hz, 最近一帧的本地时间)`。

        `hz` = **自本会话首次收到该类帧起的平均到达频率** `(count - 1) / (last - first)`。
        分母是**实际到达的间隔**, 故它不随链路空闲而衰减 (与"自 connect 起算"的写法不同),
        也不受别的帧类型影响。

        样本不足 2 条 (`count < 2`) 时 `hz` 写死 `0.0` —— 一个样本定不出频率, 它同时把
        两类入口的形状说清楚了:

        * **单发请求/应答式** (`get_joint_param` / `get_body` / `get_jm` / `get_gravity`):
          一次调用只到达一帧 ⇒ 第一次调用必然 `hz == 0.0`; 第二次起它等于
          **调用方自己的轮询频率** (请求/应答式帧的到达率就是请求率), 不是固件的什么周期。
          ⚠ **`diag.kin_bench` 不在这一组**: 它的回执是**连续两帧** (耗时帧 + LINK 帧,
          见 `diagnostics._KIN_BENCH_FRAMES`), 一次调用到达 2 帧 ⇒ 第一次调用 `hz` 就非 0,
          且那个数**没有意义** (分子是被拆帧放大的, 分母还是调用间隔)。
        * **被动连续流** (`RSP_STATUS`, 固件 100Hz): 两帧之后收敛到 ~100。

        `timestamp` 是本会话**最近一帧**的到达时刻 (样本只有 1 条时照样是它);
        本会话从没收到过该类帧时回 `(0.0, 0.0)` —— `0.0` 是哨兵 (`time.monotonic()`
        恒 > 0, 不会与真时刻撞车)。
        """
        ent = self._recv.get(c)
        if ent is None:
            return (0.0, 0.0)
        span = ent[2] - ent[1]
        if ent[0] < 2 or span <= 0.0:
            # `span <= 0` 是**防御性**的: `time.monotonic()` 在 Linux 上分辨率 1ns,
            # 两帧撞进同一个读数到不了 —— 留着是为了让本函数对任何输入都有定义 (别拿
            # 它当"某条路径真的会这样"的证据)。
            return (0.0, ent[2])
        return ((ent[0] - 1) / span, ent[2])

    def _on_status(self, payload: bytes) -> None:
        try:
            self.state = ST.decode_state(payload)
        except ValueError:
            pass

    def expect(self, want: int, timeout: float, cmdlabel: str,
               raise_on_err: bool = True, echo_cmd: Optional[int] = None,
               err_waits_for_ack: bool = False):
        """等 `want` 那条应答 —— 从**属于本命令**的那条队列里等。

        **归属判据 `(上行 id, 回显码)`**：`want` 是 `RSP_ACK`/`RSP_ERR` 时 `echo_cmd`
        **必填**（见 `_wait_keys`），于是 `ACK{0x10}` 与 `ACK{0x11}` 天然落在两条队列里，
        **并发命令互吃应答这件事在结构上不可能发生**。

        `err_waits_for_ack=True` 时，一条匹配 `echo_cmd` 的 `ERR` **不是终局**：记下它，
        继续等 `ACK{echo_cmd}` —— 找到 ⇒ 那条 ERR 是**别人的**，本命令其实**已被受理**
        （按正常应答返回，**不抛**）；窗口耗尽仍只有 ERR ⇒ 那条 ERR 就是本命令的，抛它。

        判据是固件对"受理/拒绝"的**应答形状**不同：受理**必生成**一条 `RSP_ACK{cmd}`
        （含 `cart_reply`），拒绝**只**生成 `RSP_ERR{cmd,code}`（**绝不**补 ACK）。
        ⚠ **这条形状只在"生成侧"成立，到达侧不是**（别读成"回 ERR ⟹ 必被拒"）：受理那条
        `ACK{cmd}` **自己也会丢** —— 固件应答 FIFO 满时丢**最新**（`usb_cmd.c:63-64`；
        计数点是 `:1123` 的 `rf_dropped++`）。那时窗口里只剩一条 ERR ⇒ 本方法判"被拒"而
        本命令**其实已被受理**。

        ⚠ 代价（知情的）：真被拒时要**等满整个窗口**才返回 —— ERR 到得再早也不算数。
        所以**只有笛卡尔三条入口**开这个开关（见 `cart._request_and_wait`）。
        """
        keys = _wait_keys(want, echo_cmd)
        #: 见过、但还**不能判定属于本命令**的那条 ERR 的载荷（见 `err_waits_for_ack`）。
        err_seen = None
        # ⚠ 这里**不清队**：清队由 `Arm._raw_write` 在**发帧之前**做（`drain_for`）——
        # 放在这里会 100% 吃掉自己的应答（实测在桩上直接让 `connect()` 握手超时）。
        end = time.monotonic() + timeout
        while True:
            left = end - time.monotonic()
            if left <= 0.0:
                break
            fr = self._wait(keys, left)
            if fr is None:
                break
            c, pl = fr
            if c == P.RSP_ERR:
                if err_waits_for_ack and err_seen is None:
                    # 先记下, 等窗口给结论 (见 docstring)。⚠ 第二条起的 ERR 不再记 ——
                    # 一条命令至多被拒一次; 但**也不计数**: 它是本命令队列里的重复帧,
                    # 被自己消费掉的不算"没等到主人的帧"（与 `dropped` 的口径不同）。
                    err_seen = pl
                    continue
                if raise_on_err:
                    raise _err_from(pl, f"{cmdlabel} 被固件拒绝: ")
                return ("ERR", pl)
            return ("ACK", pl) if want == P.RSP_ACK else ("RSP", pl)
        if err_seen is not None:
            # 窗口里只见过 ERR、**没**见过 `ACK{echo_cmd}` ⇒ 它就是本命令的拒绝。
            if raise_on_err:
                raise _err_from(err_seen, f"{cmdlabel} 被固件拒绝: ")
            # ⚠ **本行今天没有调用方走到**（`err_waits_for_ack=True` **且**
            # `raise_on_err=False` 才可达，而库里两处调用各只开一个）。保留它是**刻意**的：
            # 它两个兄弟分支各覆盖一种开关组合，三者共同定义 `expect` 的返回契约 ——
            # 删掉它，那条"我不想抛、但请把拒绝告诉我的"组合会掉到末尾的
            # `MotionTimeoutError`, 而**那时明明收到了应答**（一条 ERR），归因直接反了。
            return ("ERR", err_seen)
        raise MotionTimeoutError(f"{cmdlabel} 无应答(超时 {timeout:.1f}s)")


class Arm:
    """直连后端子集。用法::

        import litearm as pa
        arm = pa.Arm().connect()
        arm.enable()
        arm.movej([0, 0, 0, 0, 0, 0, 0], speed=0.3)
        tcp = arm.get_tcp()           # -> Msg: 读值要 .value (2.0 起的返回信封)
        print(tcp.value, tcp.hz)
        arm.move_p((0.30, 0, 0.35, 3.14, 0, 0))
        arm.close()
    """

    def __init__(self, port: Optional[str] = None, *,
                 transport_factory: Optional[Any] = None,
                 min_firmware: tuple = MIN_FW, q_tol: float = 0.03,
                 dq_tol: float = 0.10, arrive_frames: int = 3,
                 move_timeout: float = 15.0):
        self._port = port
        #: 传输工厂（缺省 `SerialTransport`）。给它是为了接假传输做离线运行
        #: (`litearm.testing.FakeTransport`) —— 不需要真硬件就能起一个完整会话。
        #: ⚠ **注入点排在 `connect()` 的 `find_cdc_port()` 空值检查之后**（见那里），
        #: 所以调用方**必须同时给一个占位 port**（如 `"fake"`）：否则无硬件时会在走到
        #: 本工厂**之前**就抛 `TransportError("未找到 STM32 CDC")`。
        #: ⚠ 带前导下划线是**刻意**的 —— `tests/test_full_coverage.py` 的公开成员哨兵
        #: (`{n for n in dir(arm) if not n.startswith("_")}`) 看不见它，因此新增本参数
        #: **不需要**放宽那条哨兵（白放宽一条哨兵是本仓文化里的危险动作）。
        self._transport_factory = transport_factory
        self._tr: Optional[SerialTransport] = None
        self._a: Optional[_Ack] = None
        self.min_firmware = tuple(min_firmware)
        self.q_tol = q_tol
        self.dq_tol = dq_tol
        self.arrive_frames = arrive_frames
        #: 动作等待窗口 (`move_p` 到位 / 笛卡尔等 `0x4E` 都用它)。
        #: ⚠ **`_CartPending` 的吸收额度只在本属性被读到的那一刻取一次快照** (两个构造点:
        #: 下面 ① 与 `connect()` 里那处重建), 且**有下限** —— 传下去的是
        #: `cart_mod.cart_absorb_ttl(self.move_timeout)` = `max(move_timeout,
        #: _ABSORB_TTL_FLOOR)`, 不是裸的本属性 (下限的固件依据与两侧的代价见那个常量)。
        #: 于是 connect **之后**再改本属性会让"等待窗口"与"额度有效期"分叉, 且**两个方向
        #: 都不安全**:
        #:   · 调**小**它 ⇒ 额度有效窗相对变**长** ⇒ 一条真·脱同步的应答被静默吸收的时间
        #:     窗被拉长 (本该报 `LiteArmError` 的那条被当成清队的迟到应答吞掉);
        #:   · 调**大**它 ⇒ 额度有效窗相对变**短** ⇒ 固件那条已在路上的迟到应答会撞上
        #:     "队列空"判据, 把 `LiteArmError` 从无关读路径里炸出来。
        #: 要改就 `reconnect()` (它按新值重建 `_CartPending`)。详见 `_CartPending.__init__`。
        self.move_timeout = move_timeout
        self.firmware: str = ""
        self.fw_version: Optional[tuple] = None
        self.n: int = 0
        #: 台架电机对应的模型轴 (仅供非整臂 IK 种子映射; 整臂不受影响)。
        #: 默认镜像固件 joint_cfg.h 的 `LITEARM_BENCH_MODEL_AXIS`。
        self.bench_model_axis: Optional[int] = P.BENCH_MODEL_AXIS
        # --- 零重力保活会话 (见 zero_g_start/stop) ---
        self._zg_active = False
        self._zg_thread: Optional[threading.Thread] = None
        self._zg_stop = threading.Event()
        self._zg_error: Optional[BaseException] = None
        #: 启停串行化 —— 否则 stop 会插进 start 的「起线程 / 发布句柄」之间
        self._zg_lock = threading.RLock()
        #: 等保活线程停下来的上限; 超时说明串口写卡住
        self._zg_join_s = ZG_JOIN_S
        self._params: Optional[JointParams] = None
        self._model: Optional["ModelParams"] = None      # [2026-09-14] 模型在线导入
        self._log: Optional[ArmLog] = None
        self._diag: Optional[Diagnostics] = None
        #: 在途笛卡尔请求的 FIFO 配对队列 + `0x4E` 收集器 (见 `cart.py`)。
        #: 与写口同寿: `_raw_write` 里挂着它的清队钩子, 故**必须早于任何写**存在。
        #: 吸收额度的存活时间 = `cart_absorb_ttl(move_timeout)` (沿用既有旋钮 + 一个有
        #: 固件依据的**下限**, 见那个函数与 `_ABSORB_TTL_FLOOR`)。
        #: ⚠ 这是**构造点 ①** (`connect()` 里那处是 ②, 会重建本对象) —— `move_timeout`
        #: 的 docstring 里记了"改这个旋钮会让两个窗口分叉"的耦合。
        self._cart = _CartPending(absorb_ttl=cart_mod.cart_absorb_ttl(self.move_timeout),
                                  arm=self)
        #: 笛卡尔入口的**串行锁** —— `cart._request_and_wait` 从登记 token 一直持到
        #: `0x4E` 配对完成; `wait=True` 时**再持到停稳** (`_wait_settled` + 终末那次
        #: `get_tcp` 回读), 因为"一条笛卡尔命令的整条运动"才是串行的单位。
        #: `cart.move_path` 还从 **BEGIN** 起持 (BEGIN+ADD×n+RUN 整段)。
        #: (理由见 `cart.py` 模块 docstring 的"串行是强制的"那段:
        #: `0x3A/0x3B/0x3E` 共用码空间, `ERR` 回显只能按**码**判, ≥2 条在途时
        #: 会互相认下对方的 `ERR`/`0x4E` ⇒ **假成功**)。
        #:
        #: ⚠ **可重入 (`RLock`), 不是 `Lock`** —— 嵌套点只有一个:
        #: `cart.move_path` 把 BEGIN + ADD×n + RUN **整段**圈进本锁 (两个并发 `move_path`
        #: 的 RECV 收集态会交错成一条谁也不是的路径), 而 RUN 那一段本身就走
        #: `cart._request_and_wait`, 它**也要**拿这把锁。换成 `Lock` 会在那个嵌套点
        #: 把调用者自己锁死 (同线程二次获取), 所以形状只能是 `RLock`。
        #:
        #: ⚠ **锁序**: `_cart_serial` → `_CartPending._lock` → `transport._wlock`
        #: (另一条路是 `_zg_lock` → `_CartPending._lock` → `transport._wlock`)。
        #: **反向无环**, 判据是"下层都不知道上层": `_CartPending` 与 `transport` 都不引用
        #: `_cart_serial` / `_zg_lock`; 而 `_CartPending.wait` 是从 `_CartPending._lock`
        #: **之外**驱动 `pump` (取帧) 的, 所以取帧路径上的清队/配对只在锁内一闪而过。
        #:
        #: ⚠ **`emergency_stop`/`disable`/`zero_g*`/`get_tcp` 自己不许获取本锁** ——
        #: 降能量方向的动作与只读查询必须永远可达 (持锁者可能阻塞到 `move_timeout`)。
        #: ⚠ 但**入口在自己的临界区里调用 `get_tcp()` 是另一回事**: `cart.move_c` 的
        #: 起点校验就是那样 (它读的就是"此刻 TCP", 与正在跑的那条运动本来就是同一个临界
        #: 区; 放在锁外实测会把在跑那条的 `ACK{0x3A}` 抢走 ⇒ 受害者报"无应答"而命令
        #: 其实已生效)。两句话不矛盾: "不许在 `get_tcp` **里面**拿锁" ≠ "入口不许在
        #: 持锁时调用它"。
        #: ⚠ `poll_cart` **也拿这把锁, 且用非阻塞 `acquire`** (拿不到就 `None`): 它读一帧
        #: 来认领结果, 而"读一帧"正是抢帧动作 (实测它会吃掉在飞那条的 `ACK{0x3A}`) ——
        #: 拿不到锁说明有笛卡尔入口正在用链路、且它自己会取帧, 那时返回 `None` 才对。
        #:
        #: ⚠ 只在 `__init__` 建一次、**不**随 `connect()` 重建 (虽说 `_cart` 会重建):
        #: 锁不是会话状态, 重建会让"旧会话的持锁者"与"新会话的持锁者"同时进临界区。
        self._cart_serial = threading.RLock()
        #: 固件是否支持笛卡尔 (受 `#if LITEARM_CART_PLAN` 约束) —— `connect()` 里探一次。
        #: ⚠ 名字**必须**带前导下划线: `test_full_coverage.py` 的
        #: `test_public_api_surface_is_all_exercised` 要求每个公开成员都被练习过,
        #: 公开一个没人读的探测缓存会让那条哨兵变红。
        self._cart_supported: Optional[bool] = None
        #: 探测**没获确认** (3 次全安静 = 上行丢帧) 而不是"固件确报不支持" —— 只用来给
        #: `_require_cart_support` 分两支措辞 (归因用), 不参与任何判据。
        self._cart_probe_silent: bool = False
        #: 会话**终态**: `enter_dfu()` 已确认设备从 CDC 上消失 (见那个方法)。
        #: 置位后**所有**入口抛 `ArmIsInDfuError` —— 主体由 `_reject_if_in_dfu()` 在三处
        #: **结构性的**收口上统一拦 (`_require()` / `_read_one()` / `_raw_write()`),
        #: 而不是逐入口枚举 (枚举漏一个就是"终端态能继续用")。
        #: ⚠ 但那三处收口**盖不住**"本地预检排在收口之前"的入口: `move_js` / `send_mit` /
        #: `send_mit_all` 拿 `self.n` 做 arity/idx 预检, 而终态下 `close()` 已把 `self.n`
        #: 清成 `0` ⇒ 预检先炸, 报出来的是**误导性**的 "q 需 N 个" (N=0, 把人引向一个不
        #: 存在的 arity bug)。故这三条各自在**最前**补一句 `_reject_if_in_dfu()` ——
        #: `_dfu_entered` 为假时它是纯空操作, 非终态行为一个字节都不变。
        #: ⚠ 它**不**随 `connect()` 复活, 也不随 `close()` 清除 —— 见 `ArmIsInDfuError`。
        self._dfu_entered: bool = False
        #: 建立**本会话的那个进程** (`connect()` 里落章) —— fork 守卫的判据, 见
        #: `ForkedSessionError` / `_reject_if_wrong_process()`。
        #: `None` = 还没有会话 (未 connect, 或 `close()` 已清) ⇒ 守卫放行, 由既有的
        #: `NotConnectedError` 去报"没连"。
        #: ⚠ 只在 `connect()` 落章、在 `_clear_session_state()` 清 —— 与会话同寿。
        #: ⚠ 子进程里 `close()` **放行但走轻路径** (只清会话状态、不碰传输层 —— 会挂死,
        #: 见 `close()` 的 docstring); 它会把 `_pid` 清成 None ⇒ 此后守卫不再拦, 但那时
        #: `_a`/`_tr` 也是 None, `_require()` 照样抛 `NotConnectedError` —— 两条路都指向
        #: "没会话可用", 不构成漏洞。
        self._pid: Optional[int] = None
        #: 同帧节流 (`min_interval`) 的三张表 —— cmd id -> 最短重发间隔 / 上次发出去的载荷 /
        #: 那次发出的时刻 (monotonic)。**默认全空 = 一处都不节流**。
        #: ⚠⚠ 默认必须是空的: 对 `move_js`/`send_mit` 这类 ≥10Hz 重发的伺服/透传路径 (以及
        #: 保活 `0x06`) 打开它 ⇒ 重发被丢 ⇒ 0.1s 命令看门狗 fail-soft / 伺服断流。开关与
        #: 两条后果见 `_set_tx_repeat_min_interval`。
        #: ⚠ 三张表在 `_raw_write` 里被读写, 而保活线程**也会**走那个口 ⇒ 必须持
        #: `_tx_repeat_lock`。锁序: `_cart_serial` / `_zg_lock` → `_tx_repeat_lock` →
        #: `transport._wlock` —— 本锁是**叶子** (它内部不获取任何别的锁, 也不回调出去),
        #: 故不引入环。
        self._tx_repeat_min_interval: dict = {}
        self._tx_last_payload: dict = {}
        self._tx_last_stamp: dict = {}
        self._tx_repeat_lock = threading.Lock()
        #: 被同帧节流**丢掉**的累计帧数 —— 只做可观测性 (丢帧不许静默), 不改变任何行为。
        self._tx_throttled_frames = 0

    # ---------- 连接 ----------
    def connect(self, port: Optional[str] = None) -> "Arm":
        """连上 CDC 并握手 (**幂等**)。

        已经连着**同一个目标**时是 no-op (返回 `self`, 不关链路、不重建会话) ——
        重复调用最不该打断的正是**正在跑的那个会话** (在途笛卡尔记账、零重力保活、
        串口句柄), 而"再连一次"在旧实现里恰恰把这三样全推倒重来。

        ⚠ 幂等**不是**"永远不做事": 显式给了**另一个端口**时**真的改靶**。否则
        `connect("/dev/ttyACM2")` 会被静默吞掉 —— 用户以为连的是 ACM2, 实际还在
        ACM1, 而本模块一贯不接受"静默无效"。想**强制**重建同一个目标用 `reconnect()`。
        """
        # ⚠ 终态**不因 connect() 复活** (唯一的出路是新建一个 `Arm`) —— 见
        # `_reject_if_in_dfu` 与 `ArmIsInDfuError`。放在**最前**: 终态下连收尾动作都不做
        # (fail-fast、无副作用; `close()` 自己是幂等空操作, 但"先关一遍再报错"没有意义)。
        self._reject_if_in_dfu()
        # ⚠ fork 守卫也放在**最前**: 否则下面那句"幂等早退"会在子进程里**成功返回 self**
        # —— 用户以为连上了, 拿到的是一个没有读线程的会话, 后果见 `ForkedSessionError`。
        self._reject_if_wrong_process()
        # 幂等早退。比较的是**当前链路的 port** (不是 `self._port`): 后者只是构造函数的
        # 入参, 省略端口时它是 `None` 而实际连上的端口是 `find_cdc_port()` 找到的那个。
        # ⚠ `getattr` 兜底**射程比它读起来窄**: 第一个析取项 `port is None` 先命中, 所以
        # **无参 `connect()` 在 `_tr is not None` 时恒早退** (连关都不关) —— 对**没有
        # `.port`** 的传输 (自定义实现), "取不到 port 就当不同目标、宁可多重建一次"这条
        # 规则**只在显式传了 port 时**才成立 (实测: 无参 → 早退、close 0 次; 传
        # `"/dev/…"` → 旧传输被 close 后换新)。无参时它被当成**同一目标**, 于是那种传输
        # 一旦坏了 `connect()` 永远救不回来 (只有 `reconnect()` 能) —— 与"宁可多重建一次"
        # **正相反**。
        # ⚠ 对真传输这段兜底是**死代码**: `SerialTransport` 恒有 `.port`
        # (`transport.py:136` 的 `self.port = port`), 故 `getattr` 永远取得到值, 这里
        # 没有第三种情况。
        if self._tr is not None and (port is None
                                     or port == getattr(self._tr, "port", None)):
            return self
        if self._tr is not None:
            # 只有"改靶"才走到这里 (同目标已在上面早退) —— 先收干净旧会话再开新的。
            self.close()
        p = port or self._port or find_cdc_port()
        if not p:
            raise TransportError("未找到 STM32 CDC (VID:PID 1d50:606f), 请用 --port 指定")
        # 唯一的传输构造点（`reconnect()` = `close()` + 本函数，故也走这里）。
        # 注入了工厂就用工厂 —— 离线 (`testing.FakeTransport`) 与真机走同一条会话装配路径。
        self._tr = (self._transport_factory or SerialTransport)(p)
        self._a = _Ack(self)
        #: fork 守卫的落章点 —— 本会话属于**这个**进程 (见 `ForkedSessionError`)。
        #: 位置: 会话对象一建好就落, 早于下面那句握手写帧, 也早于读线程起。
        self._pid = os.getpid()
        # ⚠⚠ **读线程的起点只有这一处合法位置** —— `_Ack` 已就绪、而下面那句握手的
        # `expect` 必须已经有人读。起早了没有 `_Ack` 可投递; 起晚了握手那条
        # `expect` 永远等不到。失败路径 (`close()`) 能停掉一条已起、但握手失败的线程。
        self._a.start_reader(self._tr)
        # 新会话 = 新配对状态, 与 `_a` 同一处对称重建: 旧会话的 token 永远等不到它那条
        # `0x4E` 了, 留着会让新会话的**第一条**合法应答配给死 token, 活 token 反而走到
        # 超时 —— FIFO 配对最怕的 off-by-one 传递。
        # ⚠ 刻意**不**在 `close()` 里一并清: 断连那一刻在途 token 该怎么唤醒 (标成未知
        # 结局还是留着等 `wait()` 自己超时) 属 `wait()` 的决定; `connect()` 这一处收尾
        # (下面那两半), 合起来保证"重连之后一定是干净的"。
        # ⚠⚠ **重建是第二处"销毁记账"** (第一处是 `clear_pending` 的各个调用点), 而
        # "销毁"这件事在这一处必须**做两半**, 缺一半都会出问题:
        #   ① 旧对象走一次 `clear_pending` —— 把在途 token 标成"结局未知"并**唤醒**
        #      仍在等它的线程 (否则那些线程会挂到 `move_timeout`, 而新会话的配对状态
        #      已经换了, 它们等的那条应答永远不会配给它);
        #   ② 把旧的**在途条数**作为吸收额度**继承给新对象** —— 只清旧对象没用: 旧会话
        #      那几条**仍可能欠着应答** (`0x4E` 已在 USB 上, 或至多 `CART_PLAN_MAX_TICKS`
        #      之后发出), 而新对象一格额度都没有 ⇒ 那条迟到应答一到就撞上"队列为空"
        #      判据, `LiteArmError` 从**毫不相干的读路径**里炸出来, 归因指向
        #      "固件与主机已错配", 真相是"我们自己重建了会话"。
        # ⚠⚠ 继承那一格的**取舍与射程**都要写清楚 (它是"两害相权", 不是纯增益):
        #    · 额度**先于配对**被消费 (`_CartPending.on_reply` 的顺序是载重的), 所以旧会话
        #      那条迟到应答与**新会话自己那条**在原理上无法区分 (载荷里没有命令 id) ——
        #      继承额度可能吃掉**新会话第一条**命令的应答, 那条报 `CartReplyLostError`
        #      ("结局未知")。这是**知情的**: 反过来 (不继承) 就是让旧会话那条迟到应答配给
        #      新 token = **假成功**, 而本模块一贯宁可报"结局未知"。
        #    · 但它的射程**只有一条命令**: 那条被吃掉的请求随后超时放弃, 而超时路径
        #      **不再给它补额度** (`drop_and_absorb` 的例外段: 窗口里已经吸收过 ⟹ 固件
        #      不再欠), 于是**第二条起恢复正常**。⚠ 没有这条, 额度会被超时路径每轮续上
        #      ⇒ 新会话**此后每一条**笛卡尔命令都报"结局未知" (实测 F5 引入的入口)。
        # ⚠ 这一处**也要**按 `move_timeout` 现算 TTL (同一个 `cart_absorb_ttl`) ——
        # 对象是新建的, 丢弃那个调用方设过 TTL 的旧对象。
        inflight = self._cart.pending
        self._cart.clear_pending("会话重建 (connect) —— 固件侧不会再发 0x4E, 结局未知")
        self._cart = _CartPending(absorb_ttl=cart_mod.cart_absorb_ttl(self.move_timeout),
                                  absorb=inflight, arm=self)
        # 握手: 固件版本
        self._raw_write(P.CMD_GET_FIRMWARE)
        try:
            _, verp = self._a.expect(P.RSP_FIRMWARE, 1.5, "get_firmware",
                                     echo_cmd=P.CMD_GET_FIRMWARE)
            self.firmware = verp.decode(errors="replace").strip()
        except Exception:
            self.close()
            raise
        ver = P.parse_firmware_version(self.firmware)
        if ver is None or not self.firmware.startswith(FIRMWARE_PREFIX):
            self.close()
            raise FirmwareMismatchError(
                f"固件版本不符合约定: '{self.firmware}' —— 应为 {FIRMWARE_PREFIX}<主.次.修>-{{7J|1J}}; "
                f"请烧录 ≥{FIRMWARE_PREFIX}{self.min_firmware[0]}.{self.min_firmware[1]}.{self.min_firmware[2]}")
        if ver[:3] < self.min_firmware:
            self.close()
            raise FirmwareMismatchError(
                f"固件 {self.firmware} 过旧: 需 ≥{FIRMWARE_PREFIX}"
                f"{self.min_firmware[0]}.{self.min_firmware[1]}.{self.min_firmware[2]}"
                f"(move_j 受控静止保持语义)")
        self.fw_version = ver[:3]
        # 等待一帧状态定关节数
        st = self._read_status(timeout=1.0)
        if st is None:
            self.close()
            raise TransportError("连上但收不到状态帧")
        self.n = st.n
        # 笛卡尔能力探测 —— **连接时一次**, 不是首次调用时才探: 后者会让第一条 `move_l`
        # 的语义变成"顺便探测", 且探测帧与 `_CartPending` 队列纠缠在一起。
        # ⚠ 探测本身**不抛**"固件不支持" (它只是返回 False); 能从上面逃出来的是**写失败**
        # (链路问题) —— 与版本握手失败同级: 关链路再抛, 不留半开的会话。
        try:
            self._cart_supported = cart_mod.probe(self)
        except Exception:
            self.close()
            raise
        return self

    def close(self) -> None:
        """收尾 (**幂等**) —— 收保活线程 → 关链路 → 清会话状态。

        这是**全部**收尾路径的唯一实现: `disconnect()` 与 `__del__` (兜底) 都委托到这里,
        不另起一套 (两套清理逻辑必然漂: 改了一处忘另一处)。

        ⚠ 第一步收保活线程必须是**第一步** (它持有一个写者; 残留会让解释器退出时打哑
        CDC)。收不干净 (串口写卡住) 时 `zero_g_stop()` 会抛错 —— teardown 不能因此挂死
        或半途而废, 这里吞掉异常后照常关链路; 卡住的线程下一轮写会拿到 `_tr=None` 而
        自行退出。

        ⚠⚠ **[2026-09-22 真机实测] fork 出来的子进程里, 本方法**不做**传输层收尾** ——
        因为那会**永久挂死**。机制: `SerialTransport.close()` 要取 `self._rlock`
        (见 `transport.py:218`), 而父进程的读线程**几乎一直持着它** (`read_frame` 阻塞
        满 `_READ_SLICE_S` 期间都持锁) ⇒ fork 出的子进程继承到一把**已加锁的互斥量**,
        而能解锁的那个线程**在子进程里不存在** (线程不被 fork 复制) ⇒ `with self._rlock`
        永远等不到。**真机症状是 `Arm.close()` 卡死**(离线 `FakeTransport` 没有这把锁,
        所以离线判据测不出来 —— 这条是拿真机 pty/真板子才暴露的)。
        ⚠ 于是子进程只做**会话状态清理**: 置 `_a`/`_tr` 为 `None`、清派生状态。子进程
        持有的那个**继承来的 fd 留给进程退出时由内核关闭** —— 它既不读也不写 (守卫把
        命令全拒了), 父进程也另有自己的 fd, 故留着是无害的; 比"为了体面地关一个 fd 而
        挂死"划算得多。
        ⚠ 判据: `tests/test_fork_guard.py` 里那条"`close()` 不碰传输"的用例 —— 它在
        修复前**必红** (修复前会走到 `self._tr.close()`)。
        """
        if self._pid is not None and os.getpid() != self._pid:
            # fork 守卫: 见 docstring —— 绝不能走到下面取传输层锁的那几行。
            self._a = None
            self._tr = None
            self._clear_session_state()
            return
        if self._zg_thread is not None or self._zg_active:
            try:
                self.zero_g_stop()
            except Exception:  # noqa: BLE001 - 关链路时保活故障不该阻断关闭
                pass
        # ⚠ **停读线程必须排在 `_tr.close()` 之前**：否则它会从已经关掉的传输上抛
        # `TransportError` ⇒ `_reader_error` 非空 ⇒ 一次**正常关闭**被记成"链路丢失"，
        # 而 `_wait_until_link_lost` 正是拿那一格当"设备没了"的判据。
        # ⚠ 也**必须排在 `_a = None` 之前**（线程的对象引用在 `_a` 上）。
        if self._a is not None:
            self._a.stop_reader()
        if self._tr is not None:
            self._tr.close()
            self._tr = None
            self._a = None
        self._clear_session_state()

    def _clear_session_state(self) -> None:
        """清**会话派生**的状态 —— 关掉之后不许再报上一个会话的事实。

        留下的比清掉的多, 每一条都是判据:

        * `_port` —— **不是**会话状态: 它是构造函数的入参, 也正是"重连到同一台"的依据;
        * `_cart` (在途笛卡尔记账) —— 刻意**不**在这里清: 断连那一刻在途 token 的结局
          (标成未知结局还是留给 `wait()` 自己超时) 是 `wait()` 的决定, 见 `connect()`
          里那段 (connect 重建它是另一处独立的收尾);
        * `_dfu_entered` —— 会话**终态**, `close()` **不许**给它解锁 (见 `ArmIsInDfuError`:
          终态的唯一出路是新建一个 `Arm`);
        * `_tx_repeat_min_interval` —— **用户配置** (同帧节流旋钮), 清掉 = 静默关掉节流;
        * `_tx_throttled_frames` —— **累计**可观测计数 (与 transport 的 `flush_failures`
          同类), 记的是"这个进程一共丢过几帧", 不随会话归零;
        * `_pid` (fork 守卫的落章) —— **清**: 它是"本会话属于哪个进程"的记账, 会话没了
          就该没。清掉之后守卫放行, 但 `_a`/`_tr` 同样已清 ⇒ `NotConnectedError` 兜住;
        * `_zg_error` —— 由 `zero_g_stop()` 消费清空 (它的文档就是这么写的), 不重复清。
        """
        self.firmware = ""
        self.fw_version = None
        self.n = 0
        self._cart_supported = None
        self._cart_probe_silent = False
        # fork 守卫的落章与会话同寿 —— 清掉之后守卫放行, 但 `_a`/`_tr` 也在同一次
        # `close()` 里清成 None ⇒ `_require()` 抛 `NotConnectedError`, 两条路等价。
        self._pid = None
        # "上次发过什么/什么时候"是**会话内**的配对依据: 留着它会让新会话的第一帧撞上
        # 上一个会话留下的时刻戳而被**静默节流** (同帧节流的代价就是丢帧, 不报错)。
        with self._tx_repeat_lock:
            self._tx_last_payload.clear()
            self._tx_last_stamp.clear()

    def disconnect(self) -> None:
        """`close()` 的**别名** —— 同一个操作的两个名字, 判据也完全一样。

        为什么留两个名字: `disconnect()` 是松灵 API 的肌肉记忆名 (那边
        `connect()`/`disconnect()` 成对, 而 `disconnect()` 是唯一的收尾口), 而 `close()`
        是本包先行交付、README 与工具仓都在用的名字。二者**没有**语义差别 —— 都不置终态,
        之后都还能 `connect()`。所以这里只有一行转发, 不各自实现 (两套必然漂)。
        """
        self.close()

    def __enter__(self) -> "Arm":
        """`with Arm(...) as arm:` —— 进块时连上 (未连则连, 已连则 no-op)。"""
        if self._tr is None:
            self.connect()
        return self

    def __exit__(self, *exc) -> bool:
        """退出即关。返回 `False` = **不吞**块内的异常 (收尾不该改变异常语义)。"""
        self.close()
        return False

    def __del__(self) -> None:
        """**兜底**收尾 (GC 时刻) —— 用户忘了 `close()` 时也把链路收干净。

        ⚠ 它**只**是兜底, 逻辑**全部**委托给 `close()`, 自己一行业务都不写 ——
        第二套清理逻辑必然漂 (改了一处忘另一处)。

        ⚠ 为什么不按 §6.6 用 `weakref.finalize`: 那条路在这里**物理上走不通** ——
          · `finalize(obj, func, *args)` 的 `func`/`args` **一个都不许引用 obj**
            (CPython 文档原话: 否则 obj 永远收不掉, 兜底**永不触发**), 而"委托给
            `close()`"恰恰要求拿得到 obj;
          · 而回调真正跑起来时 obj **已经被回收**: 连把 `weakref.ref(obj)` 塞进 args
            也拿不到东西 —— 实测 (CPython 3.13.5) 那里 `ref()` 返回 `None`。
        `__del__` 拿到的 `self` 是**活的**, 于是能原样委托 (对**引用环**也成立: Arm 与
        `_CartPending(arm=self)` 之间就是环, 靠 gc 收 —— 所以兜底的判据必须
        `gc.collect()`, 见 tests/test_teardown.py)。

        ⚠ 异常必须**吞掉**: 这一刻 (GC 或解释器退出) 抛出去没人接, 会以
        "Exception ignored in ..." 走 `sys.unraisablehook`; 解释器退出时更可能挂死。

        ⚠ **已知限制, 不掩盖**: 保活线程活着时本方法**不会被调用** —— 线程的 target 是
        `self._zg_keepalive` (绑定方法) ⇒ 线程持有一个对 Arm 的强引用, 而运行中的线程
        自己又被 `threading._active` 强引用 ⇒ Arm 根本不可回收。这是"保活线程"这个设计的
        固有性质 (不是本兜底引入的): 保活期该走 `with arm.zero_g():` (退出时会停保活),
        而不是指望兜底。tests/test_teardown.py 有一条用例**钉住**这个限制。
        """
        try:
            self.close()
        except Exception:  # noqa: BLE001 - GC/解释器退出时刻不许再抛
            pass

    def reconnect(self, port: Optional[str] = None) -> "Arm":
        """**强制**重建会话 —— 与 `connect()` 的分工: 前者无条件重建, 后者已连即 no-op。

        ⚠ 不是 `return self.connect(port)`: `connect()` 幂等之后那样就成了"什么都不做"
        的别名 (已连时它直接早退)。这里**先关**再连 —— 关是幂等的, 未连接时是空操作。
        ⚠ 终态下仍会抛 `ArmIsInDfuError` (来自 `connect()` 顶部那句守卫), 终态**不因
        重连复活。
        """
        self.close()
        return self.connect(port)

    def _reject_if_in_dfu(self) -> None:
        """会话**终态**的统一守卫 —— 见 `Arm._dfu_entered` 与 `ArmIsInDfuError`。

        挂点是**结构性**的三个收口, 不是逐入口枚举 (枚举今天漏一个, 明天新入口再漏一个,
        症状是"终态的对象还能接着用"):
          · `_require()` —— 状态读取与所有经 `_write_query` 的下行都从这里过;
          · `_read_one()` —— **唯一**取帧口 (直接持 `_Ack` 的路径也拦得住);
          · `_raw_write()` —— **唯一**写口 (含 `connect()` 的握手与保活线程绕过
            `_write_cmd` 的那条);
        外加 `connect()` 顶部一句 (它不经上面任一处就先去开串口了)。

        ⚠ 那三处收口**盖不住本地预检**: `move_js` / `send_mit` / `send_mit_all` 把
        arity/idx 预检排在 `_cmd()` **之前**, 而终态下 `self.n` 是 `0` (`close()` 清的)
        ⇒ 预检先炸, 报的是 "q 需 N 个" 这种**误导性**消息 (指针指向一个不存在的 arity
        bug)、且文档给的"捕获 `ArmIsInDfuError` 再新建 `Arm`"那个迁移姿势会**漏掉**它们。
        故这三条各自在**最前**补一句本守卫 —— 判据在 `tests/test_dfu.py` 的入口清单里
        (那三条与其余入口**同表断言** `ArmIsInDfuError`)。
        ⚠ 补这一句**不动非终态**: `_dfu_entered` 为假时它只是个布尔读; 那三条的
        arity/idx 预检与"预检先于连接"的既有归因**逐字未动** (那是别处刻意定的, 见
        `movej` 顶部那句注释的另一取向 —— 刻意**不**在本轮统一)。

        ⚠⚠ **射程要说准, 别读成"任何入口都抛本异常"**: 本守卫只保证"**走到了取帧 / 写帧
        收口**"的那条路径上先抛 `ArmIsInDfuError`"。收口**之前**还有各入口自己的**本地
        参数预检**, 那些预检**照样先炸**(它们排在 `_require()`/`_cmd()` 之前) —— 实测(终态
        对象, 故意给非法参数): `set_ff_mask(0x1000)` / `ff_preset(5)` / `set_speed(200)` /
        `set_gravity_scale(4 值)` / `set_inertia_scale(4 值)` / `set_ff_vec(0,…)` /
        `get_ff_vec(0)` / `zero_g(1.0)` / `ik([1,2])` 这 **9 个入口**抛的都是
        `InvalidCommandError`, 不是本异常。
        ⚠ 这**不是**"终端态还能接着用": 那些参数在**任何**状态下都会先被本地预检拒掉, 与
        终态无关 —— 想让每条入口都在最前插一句本守卫就**要为 50 个入口各改一处**, 换来的
        只是把一条"与状态无关的参数错"改报成"状态错", 解决不了新问题, 故**刻意不做**。
        反过来说: 用**合法参数**调用时, 只有 `last_reset_reason`/`zero_g_active`/
        `zero_g_error` 这三个**纯本地访问器**不抛 (它们本来就不碰链路, 那是**对的**)。

        ⚠ `close()` **故意不拦**: teardown 在任何状态下都必须可用 (与 `emergency_stop`
        "降能量方向的动作永远可达"同一个理由); 终态下它本来就是幂等的空操作
        (`_tr` 已在 `enter_dfu()` 里置 `None`)。
        """
        if self._dfu_entered:
            raise ArmIsInDfuError(
                "本 Arm 已交棒进 DFU (会话终态): 设备当时从 CDC 上消失, 此后所有会走到"
                "取帧/写帧收口的入口都抛本异常 (含 `move_js`/`send_mit`/`send_mit_all` "
                "这类带本地 arity/idx 预检的)。⚠ 参数本身已非法时, 入口自己的本地预检"
                "会先抛 `InvalidCommandError` —— 那是另一件事 (与终态无关), **不是**说"
                "终端态还能接着用。烧完固件请**新建一个 Arm** 连新固件 (本对象不会复活)")

    def _reject_if_wrong_process(self) -> None:
        """**fork 守卫**的收口 —— 见 `ForkedSessionError`。

        判据一条: 建立会话的进程 ≠ 当前进程。挂点与 `_reject_if_in_dfu()` 一样取
        **结构性的收口**, 不逐入口枚举:
          · `_require()` —— 状态读取与所有经 `_write_query` 的下行;
          · `_Ack._wait()` —— **唯一**取帧口 (直接持 `_Ack` 的路径也拦得住);
          · `_raw_write()` —— **唯一**写口 (含 `connect()` 的握手与保活线程那条);
        外加 `connect()` 顶部一句 (它在早退路径上就不经上面任一处)。

        ⚠ **顺序**: 排在 `_reject_if_in_dfu()` **之后** —— 终态是本对象自己的事实
        (更具体), 而"在子进程里"是环境的事实; 两者给出的出路是同一条 (新建 `Arm`),
        故先报终态不损失信息。

        ⚠ **为什么不用 `os.register_at_fork`**: 回调跑在**子进程**里, 而那一刻它可能
        正持有从父进程继承来、**永远不会被释放**的锁 (回调里一碰 `_cond` 就死锁)。
        判据放在命令路径上**按 PID 现算**, 既不碰锁, 也不依赖"回调有没有跑成"。
        ⚠ 代价: 每条命令多一次 `os.getpid()` (命令本身是 ms 量级, 这一项百纳秒量级)。

        ⚠ **不做的事**: 不自动重连。子进程要真用臂, 得由用户**显式**新建 `Arm` ——
        自动重连会在用户不知情时让两个进程同时操作同一条 CDC。
        """
        pid = self._pid
        if pid is not None and os.getpid() != pid:
            raise ForkedSessionError(
                f"本会话是在 PID {pid} 里建立的, 当前进程是 {os.getpid()} —— fork 出的"
                f"子进程不继承线程 (读线程不存在), 命令会**真的写出去**却永远等不到应答,"
                f"调用方只会看到'无应答'超时 (读状态更隐蔽: 静默回继承来的陈旧值)。"
                f"故本包在这里 fail-closed: 一个字节都不下发。"
                f"子进程要用臂请**新建一个 Arm**; `close()` 仍可调, 收尾不受影响")

    def _require(self) -> _Ack:
        self._reject_if_in_dfu()
        self._reject_if_wrong_process()
        if self._a is None:
            raise NotConnectedError("未 connect()")
        return self._a

    def _pump_until(self, timeout: float, done, label: str):
        """**驱动型读者**的收口：等到 `done(a)` 为真、或链路没了、或窗口用尽。

        它**一条帧都不认领** —— 状态帧由读线程在 `_Ack._deliver` 里填进单槽，
        本方法只是「等到单槽前进到满足条件」。

        ⚠ 三处用它：`_read_status` / `get_status_now` / `_arrive`。它们今天各自是一个
        读循环，且各自抄了一遍 `a._on_status(p)` 与"把别人的帧计一次" —— 那些整段消失。

        ⚠ 「链路没了」由 `_Ack._reader_error` 判（读线程把传输层异常存在那里），
        与 `Arm._wait_until_link_lost` 用的是同一个信号。
        """
        a = self._require()
        end = time.monotonic() + timeout
        with a._cond:
            while True:
                if a._reader_error is not None:
                    raise TransportError(f"读线程已退出: {a._reader_error}")
                if a._errors:
                    # 协议层异常（典型: `cart.on_reply` 的"多了一条"）—— 读线程把它存起来
                    # 交给等待者。⚠ **驱动型读者也要看它**：否则一次"配对已错位"只对
                    # 队列等待者可见，而纯状态等待（广播那种）永远看不到。
                    raise a._errors.pop(0)
                if done(a):
                    return a
                left = end - time.monotonic()
                if left <= 0.0:
                    return None
                a._cond.wait(left)


    def _msg(self, value, ch: int) -> Msg:
        """把"刚读到的那一帧的值"包成 :class:`Msg` —— 本包被包的 11 个读口共用一个形状。

        `ch` 是**上行帧 id** (接收表的分类键), 不是命令码。
        """
        a = self._a
        if a is None:
            # 11 个入口都在 `_require()` 之后调本方法, 正常到不了这里; 到得了的只有
            # "同一对象上 close() 与读并发"那类既有竞态 ⇒ 回零统计, 不额外抛第二类异常。
            return Msg(value=value, hz=0.0, timestamp=0.0)
        hz, ts = a.recv_stats(ch)
        return Msg(value=value, hz=hz, timestamp=ts)

    # ---------- 状态读取 ----------
    def get_state(self, refresh: bool = False,
                  timeout: float = 0.5) -> "Msg[Optional[ST.RobotState]]":
        """当前状态 (固件 100Hz 被动状态流的最近一帧)。

        `refresh=False` 且本会话已有缓存时**不取帧**, 直接回缓存 —— 那种情形下
        `Msg` 的 `hz`/`timestamp` 描述的仍是**缓存那一帧所属那类帧**在本会话的到达统计
        (接收表是 per-type 的, 与"这一次调用取没取帧"无关)。

        返回 :class:`Msg` 信封 (帧 id `RSP_STATUS`); 取不到帧时 `value=None`。
        """
        a = self._require()
        if a.state is None or refresh:
            return self._msg(self._read_status(timeout=timeout), P.RSP_STATUS)
        return self._msg(a.state, P.RSP_STATUS)

    def _read_status(self, timeout: float = 0.5) -> Optional[ST.RobotState]:
        """等到一条**新的**状态帧（`status_seq` 前进），返回它解码出的 state。

        ⚠ 与今天逐字同语义（"从链路上取一条新状态"），只是驱动方式从"自己取帧"变成
        "等读线程填槽"。取不到返回 `None`（调用方决定是抛还是回 `Msg(value=None)`）。
        """
        a = self._require()
        seq0 = a.status_seq
        self._pump_until(timeout, lambda a: a.status_seq != seq0, "状态帧")
        return a.state

    def get_status_now(self, timeout: float = 0.5) -> "Msg[ST.RobotState]":
        """主动发 `GET_STATUS 0x40` 取一帧状态。

        与 `get_state()` 的区别: 后者只消费固件 100Hz 的被动流, 链路静默时只能靠
        超时发现; 本方法主动要一帧, 用于确认链路活性/拿到"此刻"的状态。

        ⚠ `timeout=0.0` 在新机制下**没有"非阻塞探一帧"这个动作了**（没有"取帧"）——
        它的语义是「**立刻返回当前缓存**」，见下面那支。
        """
        a = self._require()
        # `0x40` 是查询类: 不受零重力守卫限制 —— 由 `_write_query` 保证。
        self._write_query(P.CMD_GET_STATUS)
        if timeout <= 0.0:
            # 不等待: 读线程一直在填, "现在这一刻的状态"就是缓存里那个。
            if a.state is None:
                raise MotionTimeoutError("get_status_now: 还没有任何状态帧 (链路刚起?)")
            return self._msg(a.state, P.RSP_STATUS)
        seq0 = a.status_seq
        err_key = (P.RSP_ERR, P.CMD_GET_STATUS)

        def _done(a) -> bool:
            # ⚠ **本命令被拒也算"有结论"**：`0x40` 被拒时固件回 `ERR{0x40,code}`
            # （真固件的 `GET_STATUS` 没有失败路径，这一支只可能来自"那版固件里没有
            # 这条命令"的 `default` 分支 —— 但契约必须成立）。
            # ⚠ 只看**回显本命令码**的那条：别人的 ERR 落在**别的**队列里，不是我们的
            # 结论（这正是重构要买的东西 —— 从前"对任意 ERR 都抛"会把别人的拒绝记到
            # 自己头上，见 `tests/test_zero_g_cart_guard.py`）。
            return bool(a._queues.get(err_key)) or a.status_seq != seq0

        self._pump_until(timeout, _done, "get_status_now")
        errq = a._queues.get(err_key)
        if errq:
            _c, p, _sq = errq.pop(0)
            raise _err_from(p, "get_status_now: ")
        st = a.state
        if st is None:
            raise MotionTimeoutError(
                f"get_status_now 无状态帧应答 (超时 {timeout:.1f}s)")
        return self._msg(st, P.RSP_STATUS)

    # ---------- 命令 ----------
    def _raw_write(self, cmd: int, payload: bytes = b"") -> None:
        """最底层写口 —— **所有**下行帧的唯一出口 (含零重力保活线程)。

        ⚠ 保活线程刻意绕过 `_write_cmd` (避开零重力守卫), 但**不该绕过清队**:
        它发的 `0x06` 正是会**静默作废**在途笛卡尔规划的那一类。所以清队钩子挂在
        这一层、而**不是** `_write_cmd`/`_write_query`: 必须比它俩更低, 才拦得住
        绕过零重力守卫的写者。

        **发出前清, 不看应答**: 固件的 `cart_invalidate_before_motion()` 是
        `ctrl_accept_move_j` 的**第一条语句**, 位于**全部门禁之前** —— 所以被
        `ERR 0x03/0x04/0x06` 拒掉的命令**同样已经**作废了在途规划。写成"收到 ACK
        才清队"就会在这条被拒路径上留下陈旧 token, 下一个 `0x4E` 一到就错配。

        不做 `_require()`: 三处调用点各自已有自己的守卫 (`_write_query` 先
        `_require()`, `connect()` 刚建好链路, 保活线程自会被 `except` 收口) ——
        这里再加一道只会改变 `connect()` 的既有语义。
        ⚠ 唯一例外是**终态守卫**: 它**必须**在这里 (而不是只在 `_require()` 里) 再拦一道,
        因为这是"唯一写口" —— 挂在收口上, "终态的 Arm 不会再发出任何帧"才是**结构性**的
        事实, 而不是靠逐个入口枚举 (枚举漏一个就是终态还能写)。
        ⚠ 同帧节流也在这层 (判据与理由见 `_tx_allowed`): 挂"唯一写口"而不是挂在某个命令的
        入口上, 才拦得住保活线程这类绕过 `_write_cmd` 的写者。
        """
        self._reject_if_in_dfu()
        self._reject_if_wrong_process()     # fork 守卫: 唯一写口, 结构性地保证"子进程零下发"
        # ⚠ 同帧节流**必须**排在下面那道清队钩子**之前**: 钩子的理据是"固件**收到**这条
        # 命令就会 `cart_invalidate_before_motion()`", 而帧压根没上 USB 时固件的状态一点
        # 没变 —— 此时清队会把一条**还有效**的在途请求作废掉 (见
        # `test_a_dropped_frame_does_not_clear_the_cart_queue`)。
        if not self._tx_allowed(cmd, payload):
            return
        if cmd in _CART_CLEARS_UPON:
            self._cart.clear_pending(
                f"被 0x{cmd:02X} 作废 (固件侧 cart_invalidate_before_motion)")
        # ⚠⚠ **清本命令的应答队列必须排在写之前**（`_Ack.drain_for`）：我们的应答只可能
        # 在写**之后**到达，故此刻清队不可能吃掉自己的应答。反过来（先写后清）会 100% 吃掉
        # —— 实测在桩上直接让 `connect()` 的固件握手超时。位置的口径见 `drain_for`。
        if self._a is not None:
            self._a.drain_for(cmd)
        self._tr.write_frame(cmd, payload)

    def _tx_allowed(self, cmd: int, payload: bytes) -> bool:
        """同帧节流判据 —— 返回 True 表示这一帧该发 (False = 丢, 且**只计数不改别的**)。

        两条**同时**成立才丢:

        * 该 `cmd` 在 `_tx_repeat_min_interval` 里有一个 >0 的间隔 (**表默认空 ⇒ 全关**);
        * 上一次**真发出去**的那一帧与本帧 `cmd` 相同、载荷**逐字节相同**, 且距今 < 间隔。

        ⚠ 载荷一变就照发 (伺服换目标必须能出去; 见
        `test_a_changed_payload_is_never_throttled`)。
        ⚠ 被丢的帧**不更新**时间戳 ⇒ 持续重发同一条帧是"每 `min_interval` 出去一帧",
        不是"每帧都延期"。
        ⚠ 丢帧**不静默**: `_tx_throttled_frames` +1。
        ⚠ 这里**不做** `_require()`/终态守卫 —— 那是调用方 `_raw_write` 的事, 且本方法在
        `_tr` 已是 `None` 时也不会被走到 (`_raw_write` 先拦)。
        """
        min_interval = self._tx_repeat_min_interval.get(cmd, 0.0)
        if min_interval <= 0.0:
            return True
        now = time.monotonic()
        with self._tx_repeat_lock:
            if (self._tx_last_payload.get(cmd) == payload
                    and now - self._tx_last_stamp.get(cmd, 0.0) < min_interval):
                self._tx_throttled_frames += 1
                return False
            self._tx_last_payload[cmd] = payload
            self._tx_last_stamp[cmd] = now
            return True

    def _set_tx_repeat_min_interval(self, cmd: int, interval: float) -> None:
        """同帧节流旋钮: `cmd` 的最短重发间隔 (秒), `0` = 关闭该命令的节流。

        ⚠ 是**私有**的: `test_full_coverage.py::test_public_api_surface_is_all_exercised`
        要求每个公开成员都进它的名单 ⇒ 公开它就要改那条既有用例 (本轮的判据是"既有用例
        一行不改")。要把它做成产品面, 那是另一步 —— §6.6 只要求这条约定**存在且默认关闭**。

        ⚠⚠ 打开之前必须知道**两条**后果 (两条都有用例钉住):

        * **伺服/保活会断流**: `move_js`/`send_mit` 靠 ≥10Hz 重发维持固件 0.1s 命令看门狗
          (`fail-soft` = 丢重力前馈), 保活 `0x06` 靠重发维持零重力 —— 重发被节流掉就是
          断流 (`test_moving_paths_are_not_throttled_by_default` /
          `test_enabling_the_throttle_on_the_keepalive_drops_it`);
        * **被丢的请求不会有应答**: 一条被节流的请求, 它的调用方会**等到窗口耗尽**再报
          "无应答" (命令其实**没发**) —— 这是与上游那个"请求被节流仍能从解析缓存取值"的
          设计的**根本差别**。⚠ [2026-09-21] 论据换了一个, 结论没变: 从前写的是"本包
          **没有读线程**", 而现在本包有一个**待取帧暂存区** (`_Ack._pending`) ⇒ 那句已经
          不成立。但结论照样成立 —— 暂存区只能分派**到达的**帧, 一条被节流、**压根没写
          出去**的请求, 固件永远不会应答, 也就永远不会有帧可暂存。
          ⇒ 与上游的差别从"没有读线程"变成"**暂存区只装到达过的帧**"。
          故本包默认**一处都不开**; 真要开, 请只用在"丢了也无所谓"的
          **重复**帧上 (载荷逐字节相同 = 与上一条完全同一个请求)。
        """
        if isinstance(interval, bool) or not isinstance(interval, (int, float)):
            raise InvalidCommandError(f"min_interval 需数字 (给的是 {interval!r})")
        interval = float(interval)
        if not interval == interval:                      # NaN
            raise InvalidCommandError("min_interval 不能是 NaN")
        if interval < 0.0:
            raise InvalidCommandError(f"min_interval 不能为负 (给的是 {interval!r})")
        with self._tx_repeat_lock:
            if interval == 0.0:
                self._tx_repeat_min_interval.pop(cmd, None)
            else:
                self._tx_repeat_min_interval[cmd] = interval

    def _write_query(self, cmd: int, payload: bytes = b"") -> None:
        """**查询类**下行命令的唯一出口 (get_tcp/get_ik/参数读回/采集/自检)。

        与 `_write_cmd` 的差别: 不做零重力守卫 —— 拖动示教期间仍应能读状态。
        两者都先 `_require()` 再写: 未连接时必须是 `NotConnectedError`,
        不能因 `self._tr is None` 退化成 `AttributeError`。
        """
        self._require()
        self._raw_write(cmd, payload)

    def _write_cmd(self, cmd: int, payload: bytes = b"", *,
                   guarded: bool = True) -> None:
        """**动作类**下行命令的唯一出口 —— 零重力保活期在这里统一拒绝。

        `guarded=False` 只给「降能量」方向的安全动作 (急停/失能) —— 它们必须永远可达。
        """
        if guarded and self._zg_active:
            raise InvalidCommandError(ZERO_G_GUARD_MESSAGE)
        self._write_query(cmd, payload)

    def _cmd(self, cmd: int, payload: bytes, label: str,
             timeout: float = 1.2) -> None:
        a = self._require()
        self._write_cmd(cmd, payload)
        a.expect(P.RSP_ACK, timeout, label, echo_cmd=cmd)

    def enable(self, attempts: int = 12) -> None:
        """使能全关节 (`CMD_ENABLE 0x10`), 必要时重试。

        ⚠ **只有固件明说"可重试"的码才重试** —— 判据是白名单
        :data:`litearm.errors.ENABLE_RETRYABLE_CODES` (= `{0x03}`), 不是
        "除锁存外都重试"。固件对 `0x10` 的返回码是**并列**的四个, 其中三个都明确
        "重发无用" (`litearm.errors.ENABLE_RETRYABLE_CODES`
        的表列了原注释与出处):

        ⚠ **`ENABLE_RETRYABLE_CODES` / `ERR_TEXT` / `err_reason` 都【不在】包级公开面**
        —— 它们不在 `litearm.__all__` 里, `from litearm import
        ENABLE_RETRYABLE_CODES` 会 `ImportError`。要读判据请写全限定:

        ```python
        from litearm.errors import ENABLE_RETRYABLE_CODES, ERR_TEXT, err_reason
        ```

        这里只把它们当**散文引用** (让读者知道判据在哪), 不是承诺的公开 API ——
        spec/plan 都没点名这三个名字, 所以本轮**刻意不扩 `__all__`**。

        | code | 含义 | 处置 |
        |---|---|---|
        | `0x03` | 反馈未齐 / CMODE 首写 ("重发即可") | **重试** (真机首使能必走这条) |
        | `0x06` | 锁存, 须先 `reset()` (EMERGENCY / `joint_fault`) | **立刻抛** |
        | `0x07` | CMODE 补写预算耗尽 ("重发无用, 须现场排查") | **立刻抛** |
        | `0x00` | 固件没有这条命令 | **立刻抛** (`UnsupportedByFirmwareError`) |

        为什么值得区分: 每次重试之间 `sleep(0.3)`, 但 `sleep` 只在**重试分支**里执行
        (`for i in range(attempts)` 里 `i >= attempts - 1` 直接 `raise`) ⇒ 12 次尝试
        只有 **11 次** sleep ≈ **3.3 秒**。另有 12 次 `expect(P.RSP_ACK, 1.0, …)`,
        但那是**等应答**、不是固定等待 —— `1.0` 只是"等不到时"的超时上界, 应答一到
        就返回 ⇒ **可预期的挂起量级**就是那 3.3 秒 (超时全踩满才是 3.3+12×1.0s,
        那是链路坏掉的档, 不是这里讨论的常态)。
        对锁存故障 (EMERGENCY / 单关节故障) 那 3.3 秒是纯浪费 —— 固件在**第一次**答复里就已经
        说清了"须先 RESET", 而重试**不可能**让它变绿 (`ctrl_enable` 的这两个守卫
        在重试期间没有任何东西能解除: 只有 `reset()`/`clear_faults()` 能清)。

        抛出的仍是 :class:`CommandRejectedError` (层级不变), `.code` 供调用方判定
        "该去 `reset()` 还是该现场排查"。
        """
        a = self._require()
        for i in range(attempts):
            self._write_cmd(P.CMD_ENABLE)
            try:
                a.expect(P.RSP_ACK, 1.0, f"enable({i + 1})", echo_cmd=P.CMD_ENABLE)
                return
            except CommandRejectedError as e:
                # `echo_cmd=P.CMD_ENABLE` 保证到达这里的 ERR 回显的就是 0x10
                # (别人的 ERR 在 `_Ack.expect` 里被计成 foreign 并跳过), 故 `e.code`
                # 就是上表那一列。
                if i >= attempts - 1 or e.code not in ENABLE_RETRYABLE_CODES:
                    raise
                time.sleep(0.3)
        raise LiteArmError("enable 超时")

    def disable(self) -> None:
        """失能 —— 同样绕过零重力守卫 (降能量方向)。"""
        self._write_cmd(P.CMD_DISABLE, guarded=False)
        self._require().expect(P.RSP_ACK, 1.2, "disable", echo_cmd=P.CMD_DISABLE)

    def emergency_stop(self) -> None:
        """急停 —— 绕过零重力守卫 (降能量方向的安全动作必须永远可达)。"""
        self._write_cmd(P.CMD_EMERGENCY_STOP, guarded=False)
        self._require().expect(P.RSP_ACK, 1.2, "emergency_stop",
                               echo_cmd=P.CMD_EMERGENCY_STOP)

    def reset(self) -> None:
        self._cmd(P.CMD_RESET, b"", "reset")

    def clear_faults(self) -> None:
        self._cmd(P.CMD_CLEAR_FAULTS, b"", "clear_faults")

    def set_motion_mode(self, mode: int) -> None:
        """发 `SET_MOTION_MODE 0x20`。

        ⚠ **本固件只识别 `mode=0`**: 固件 `ctrl_set_motion_mode()` 的实现就是一行
        `park_requested = (mode == 0u)` (`control_loop.c:1338-1342`) —— mode **不写进**
        `g_arm.mode`, 其它值只是"清除 park 声明", 回 ACK 而模式不变 (阶段 A 的
        运动学模式未内建)。要声明 park 请用 `park()`。

        非 0 值**响亮失败**: 从前只发一条 `RuntimeWarning` 就照发命令 —— 那条告警
        淹没在日志里, 调用方拿到的是一个"成功"的返回, 臂其实还在旧模式。
        """
        m = int(mode)
        if not 0 <= m <= 255:
            raise InvalidCommandError(f"mode 需 0..255 (给的是 {mode})")
        if m != 0:
            raise InvalidCommandError(
                f"set_motion_mode({m}): 本固件只识别 0 (park 声明) —— 其它值固件回 ACK "
                f"但模式不变 (ctrl_set_motion_mode 只写 park_requested, 不写 g_arm.mode); "
                f"要声明 park 请用 arm.park()")
        self._cmd(P.CMD_SET_MOTION_MODE, bytes([m]), "set_motion_mode")

    def park(self) -> None:
        """SET_MOTION_MODE J=0: 声明 park, 静止保持用全刚度 (G9)。"""
        self.set_motion_mode(0)

    # ---------- 零重力 (拖动示教) ----------
    def _reject_if_cart_in_flight(self) -> None:
        """**反向守卫** —— 笛卡尔在途时拒绝进入零重力 (文案见 `CART_IN_FLIGHT_GUARD_MESSAGE`)。

        为什么拦: 固件 `0x06` 丢掉位置环、只剩重力前馈 ⇒ **轨迹中途进场会让臂靠摩擦
        滑停 (coast)**, 比 `movej()` 的受控接管 / `emergency_stop()` 差 ⇒ 拦住它并把
        替代动作写进消息 = 把操作员推向更好的停机动作。(它同时堵掉"笛卡尔在途 +
        零重力保活"那条交错 —— 见 `cart.py` 里的吸收额度。)

        ⚠ **只拦这一侧**: `movej`/`movej_sync` 是**受控接管**, 拦它等于把人推去用更重的
        `emergency_stop`; `zero_g_stop()`/`emergency_stop()`/`disable()` 是降能量方向,
        永远可达, 本方法也不碰它们。

        两条信号, **缺一不可**:

        ① `_cart_serial` 被持有 —— 三条笛卡尔入口 (`move_l`/`move_c`/`move_path`)
           全程持它, 所以"被持有"= 有一次调用正在途;
        ② **现取**一帧状态的 `bit10` (`cart_busy`) 为 1 —— 臂仍在跑。

        ⚠ 只用 ① 会漏掉 `wait=False`: 那条调用已经返回、**锁已释放**, 而臂仍在跑
        (`wait=True` 时调用方阻塞在到位等待里, 根本调不到这里 —— 所以这条路径只能靠
        `wait=False` 走通, 正是 ① 漏掉的那条)。
        ⚠ 只用 ② 会漏掉"锁已被持有、规划还没起"的那一小段 (那一刻 `bit10` 仍是 0)。
        ⚠ ② 必须**现取** (`get_status_now` 主动发 `0x40`), **不能**用 `self._a.state`:
        那可能是很久以前的帧 —— 臂已停稳却仍报忙 = **假拒绝**, 把操作员推去用更重的
        `emergency_stop`, 方向是错的。
        ⚠ 状态取不到 (超时/链路断/固件回 ERR) ⇒ **保守拒绝**: "未确认就不拦"漏过去的
        正是 coast 那一侧。
        ⚠ 但**只兜取不到的那三类** (`MotionTimeoutError`/`TransportError`/
        `CommandRejectedError`), 不兜整个 `LiteArmError`:
          · `NotConnectedError` 照常抛 —— 链路本来就断, 那是调用方该看到的错误;
          · `_CartPending.on_reply` 的"多了一条 `0x4E`"抛的是**基类** `LiteArmError`,
            含义是**两边已经错配** —— `cart.py` 刻意让它响亮, 在这里改写成"保守拒绝"
            等于把协议失步藏进一句"取不到状态", 方向与那处刻意相反。
        其余异常从本方法逃出去**同样是拒绝** (没进零重力), 方向已经安全。
        """
        if not self._cart_serial.acquire(blocking=False):
            raise InvalidCommandError(CART_IN_FLIGHT_GUARD_MESSAGE)
        self._cart_serial.release()
        try:
            busy = self.get_status_now().value.cart_busy
        except (MotionTimeoutError, TransportError, CommandRejectedError):
            raise InvalidCommandError(CART_IN_FLIGHT_UNCONFIRMED_MESSAGE) from None
        if busy:
            raise InvalidCommandError(CART_IN_FLIGHT_GUARD_MESSAGE)

    def zero_g(self, period: float = ZG_KEEPALIVE_S) -> _ZeroGSession:
        """[段②] 进入零重力拖动示教, 并**启动保活**。用法::

            with arm.zero_g():          # 进入 + 保活, 退出块时自动收尾
                ...                     # 可 arm.get_state() 读状态, 不可发其它动作命令
            # —— 或显式配对 ——
            arm.zero_g(); ...; arm.zero_g_stop()

        固件 `CMD_ZERO_G` 自带 `watchdog_kick`(「重发即保活」), 且
        `SET_MOTION_MODE`/`SET_SPEED_PERCENT` 都刻意不 kick —— 固件里**没有别的
        命令**能维持一个非 MOVE_J 模式不被 0.10s 看门狗掐死。故保活必须由本方法
        的后台线程按 `period`(默认 0.04s) 周期重发; 只发一次会在 0.1s 后掉回
        fail-soft 持位, 而调用方只看到 ACK (静默失败)。

        ⚠ 退出前先把关节推回软限位内: 零重力模式会跳过位置越限锁存(否则拖动到
        限位附近会永久锁死), 但一旦退出, 越限锁存立刻生效且重启会重锁。

        ⚠ **笛卡尔在途时会被拒绝** (`InvalidCommandError`, 见
        `_reject_if_cart_in_flight`) —— 中途进场会丢掉位置环、靠摩擦滑停。
        """
        self.zero_g_start(period=period)
        return _ZeroGSession(self)

    def zero_g_start(self, period: float = ZG_KEEPALIVE_S) -> None:
        """进入零重力并启动保活线程 (幂等: 已激活时直接返回)。

        ⚠ 反向守卫 (`_reject_if_cart_in_flight`) 排在 `_zg_lock` **之外**: 它要读一帧
        状态 (上限 `get_status_now` 的 timeout), 圈进锁里就会让 `zero_g_stop()` /
        `close()` 陪着等 —— 降能量方向的退出动作不该被一次查询拖住。位置仍在
        **写 `0x06` 之前**、**置 `_zg_active` 之前**, 两条都满足。
        """
        if not 0.005 <= period < 0.10:
            raise InvalidCommandError(
                f"保活周期需 ∈[0.005, 0.10) —— 固件看门狗超时是 0.10s (给的是 {period})")
        if not self._zg_active:      # 已激活 = 幂等 no-op: 根本不写 0x06, 无 coast 风险
            self._reject_if_cart_in_flight()
        with self._zg_lock:
            if self._zg_active:
                return
            self._cmd(P.CMD_ZERO_G, b"\x01", "zero_g(on)")  # 先确认进入再开线程
            self._zg_error = None
            self._zg_stop.clear()
            # ⚠ 顺序要紧: 先置 active (守卫立刻生效) -> 起线程 -> 最后才发布句柄。
            # 反过来 (发布句柄再 start) 时, 插进来的 zero_g_stop 会 join 一个**尚未
            # 启动**的 Thread -> RuntimeError, 且异常在置 active=False 之前抛出,
            # 于是 active 永久卡 True: 保活一帧不发、动作命令被守卫永久拒绝。
            self._zg_active = True
            th = threading.Thread(target=self._zg_keepalive, args=(period,),
                                  name=_ZG_THREAD_NAME, daemon=True)
            th.start()
            self._zg_thread = th

    def _zg_keepalive(self, period: float) -> None:
        """保活线程: **只写不读**。

        不读是刻意的 —— 主线程正在 `_Ack.expect()` 里等某条命令的 ACK, 保活线程
        若去读串口就会把那条 ACK 抢走, 制造难查的偶发超时。

        写走 `_raw_write` (而不是直写 `self._tr`): 它绕开的是**零重力守卫**, 不该
        顺带绕开**清队** —— 保活每 40ms 一条 `0x06` 正是会静默作废在途笛卡尔规划
        的那一类, 绕过清队会让规划无声作废而 SDK 毫无察觉。
        """
        while not self._zg_stop.wait(period):
            try:
                self._raw_write(P.CMD_ZERO_G, b"\x01")
            except Exception as e:  # noqa: BLE001 - 写失败=保活已断, 必须让调用方知道
                self._zg_error = e
                self._zg_active = False
                self._zg_stop.set()
                return

    def zero_g_stop(self, raise_on_lost: bool = False) -> None:
        """停保活并退出零重力 (幂等)。退出的 on=0 固件侧也幂等。

        raise_on_lost=True 时, 若保活曾因写失败中断, 在此抛出原异常 —— 臂已脱离
        零重力, 不能静默。
        """
        with self._zg_lock:
            was_session = self._zg_thread is not None or self._zg_active
            th, self._zg_thread = self._zg_thread, None
            self._zg_stop.set()
            self._zg_active = False
            # 只 join 真跑起来的线程: 对未启动的 Thread 调 join 会抛 RuntimeError
            if th is not None and th.is_alive():
                th.join(timeout=self._zg_join_s)
                if th.is_alive():
                    # 线程卡在写里 (CDC 阻塞) 停不下来。此刻**绝不能发 off 帧** ——
                    # 它随后重发的 on 帧会排在 off 之后抵达固件, 臂被重新拉回零重力
                    # 却已无人维持, 与紧接着的 movej 互相打架。宁可报错。
                    self._zg_thread = th          # 归还句柄以便重试/诊断
                    raise TransportError(
                        f"零重力保活线程未在 {self._zg_join_s}s 内停止 (串口写阻塞?) —— "
                        f"未发送退出帧 (否则会被迟到的保活帧覆盖); 检查链路后重试 "
                        f"zero_g_stop()")
            exit_err = None
            # 从未进入过就不发任何 0x06 —— 幂等退出不等于无中生有
            if was_session and self._tr is not None:
                try:
                    self._write_cmd(P.CMD_ZERO_G, b"\x00")  # 此刻 inactive, 守卫放行
                except Exception as e:  # noqa: BLE001
                    exit_err = e
            err, self._zg_error = self._zg_error, None    # stop 消费并清空 (与文档一致)
            if raise_on_lost:
                # 保活中断优先于退出帧失败上报 (前者才是臂为什么脱离零重力的原因)
                if err is not None:
                    raise err
                if exit_err is not None:
                    raise exit_err

    @property
    def last_reset_reason(self) -> Optional[str]:
        """上次启动原因: `"normal"` | `"iwdg-rst"` | `None`(未收到开机签名)。

        解析固件开机签名 (banner)。`iwdg-rst` 表示固件侧独立看门狗复位过 —— 这是
        固件唯一会外显该信息的地方, 也是排查"MCU 偶发重启"的关键线索。
        """
        if self._tr is None:
            return None
        return P.parse_boot_banner(getattr(self._tr, "text_log", ""))

    @property
    def zero_g_active(self) -> bool:
        """保活是否仍在维持 (保活期写失败会立刻变 False)。"""
        return self._zg_active

    @property
    def zero_g_error(self) -> Optional[BaseException]:
        """保活中断的原因 (无则 None)。`zero_g_stop()` 会消费并清空它。"""
        return self._zg_error

    # ---------- 关节级参数 (固件 0x22/0x23/0x24/0x36) ----------
    @property
    def params(self) -> "JointParams":
        """关节级参数读写入口 (见 `litearm.params.JointParams`)。"""
        if self._params is None:
            self._params = JointParams(self)
        return self._params

    # ---------- 动力学模型在线导入 (固件 0x30/0x32/0x33/0x34/0x35/0x37/0x38/0x39) ----------
    @property
    def model(self) -> "ModelParams":
        """动力学模型读写入口 (见 `litearm.model.ModelParams`)。

        用途: 产线把 HYY 控制器辨识出的逐台动力学参数写进固件, 无需重新编译。
        典型序列: `set_body(1..7)` + `set_jm()` + `commit(MODEL_MASK_WRITTEN)` (须失能)。
        """
        if self._model is None:
            self._model = ModelParams(self)
        return self._model

    # ---------- 300Hz 采集 (固件 0x2D/0x2E) ----------
    @property
    def log(self) -> "ArmLog":
        """300Hz 控制拍采集入口 (见 `litearm.log.ArmLog`)。"""
        if self._log is None:
            self._log = ArmLog(self)
        return self._log

    # ---------- 固件自检 (固件 0x49) ----------
    @property
    def diag(self) -> "Diagnostics":
        """固件自检入口 (见 `litearm.diagnostics.Diagnostics`)。"""
        if self._diag is None:
            self._diag = Diagnostics(self)
        return self._diag

    # ---------- 笛卡尔运动 (固件原生规划: 0x3A/0x3B/0x3C-0x3E + RSP_CART_PLAN) ----------
    # 规划 (采样/逐点 IK/Hermite 播放) 全在固件里, PC 侧只发点、收 `0x4E` 结果帧。
    # 能力 (受固件 `#if LITEARM_CART_PLAN` 约束) 在 `connect()` 时探一次, 缓存在
    # `self._cart_supported`; 不支持时下面三个入口抛 `UnsupportedByFirmwareError` 且**不发帧**。
    # ⚠ 三条命令**串行**使用 (一条受理前不要发下一条): 固件的规划结果只有**一个槽位**,
    #   连续流水线发多条会吞掉中段应答 (那时报 `CartReplyLostError` = 结局未知)。
    # ⚠ 被非笛卡尔命令 (movej/estop/disable/home/… 共 12 条 opcode) 作废时, 固件
    #   **一条 `0x4E` 都不发** —— 那些命令在发出前就会清掉本 SDK 的待配队列。
    # ⚠ `move_p` 在途时不要发笛卡尔请求: 固件后台 IK 算完后会再清一次队, 那条笛卡尔
    #   会被静默作废 (且臂会朝上一条 `move_p` 的目标真的走掉)。
    def move_l(self, pose, speed: float = 1.0, wait: bool = True) -> CartPlan:
        """笛卡尔**直线** (`CMD_MOVE_L 0x3A`): 末端沿起点→目标直线走, 姿态 slerp。

        `pose` 是**实测 TCP 之外**的目标位姿 —— 收 `[x,y,z,roll,pitch,yaw]`, 也收
        `(pos[3], R[3x3])` 与 4x4 齐次矩阵 (三种都由 `_rot.as_pose` 归一化); 起点由固件
        取当前实测 TCP (不是参数)。规划结果由 `0x4E` 返回, 失败按 `err` 抛
        (`IKError` / `CartesianPlanError` / `MotionSupersededError` / `InvalidCommandError`)。

        `wait=True` (默认) 时**等到臂停稳**才返回: `CartPlan` 的四个收尾字段
        (`started_busy`/`settled`/`q_final`/`settle_err_rad`) 填真值, 超时抛
        `MotionTimeoutError`、故障抛 `MotorFaultError`。`wait=False` 只等到**规划结果**
        (四个字段为"未等待"默认值) —— 适合"发出去就不管"或自己轮询的用法。

        ⚠ **`settled=True` 还要求"停在了目标上"**: 停稳后会回读一次实际 TCP 与 `pose`
        比对 (6mm / 0.03rad)。对不上 (轨迹被 `movej`/`home`/别的进程等**锁外**的行为者
        作废) ⇒ `settled=False` 而**不抛异常** —— `ok=True` 只说明固件受理了,
        **不是**"臂停在了目标上", 现场要回读 `get_tcp()` 看真实落点
        (见 :class:`~litearm.cart.CartPlan`)。

        两条路径都在 `Arm._cart_serial` 里跑 (一条笛卡尔命令的**整条运动**是串行单位,
        见 `cart._request_and_wait`): 并发调用会**排队**, 而不是与在跑的那条交错。
        """
        return cart_mod.move_l(self, pose, speed, wait)

    def move_c(self, pose_start, pose_via, pose_goal, speed: float = 1.0,
               wait: bool = True) -> CartPlan:
        """笛卡尔**圆弧** (`CMD_MOVE_C 0x3B`): 起点 + `pose_via` + `pose_goal` 三点定圆。

        三个位姿都收 `[x,y,z,roll,pitch,yaw]` / `(pos[3], R[3x3])` / 4x4 齐次矩阵
        (见 `move_l`)。`pose_via` 的**姿态被忽略** (固件只取它的位置三分量), 姿态从起点
        slerp 到终点。

        ⚠ `pose_start` **必须与调用时的实测 TCP 一致** (容差 6mm / 0.03rad, 含旋转等价
        判定) —— 固件圆弧的起点恒为实测 TCP, 超差抛 `InvalidCommandError` 并给出实际值。
        ⚠ 选弧规则由固件定 (有 360° 上界, 近距离三点可能扫出很大的弧) —— 事后只能从
        `n_wp` 判读。
        `wait` 的含义同 `move_l`。
        """
        return cart_mod.move_c(self, pose_start, pose_via, pose_goal, speed, wait)

    def move_path(self, poses, speed: float = 1.0, wait: bool = True) -> CartPlan:
        """多路点 (`0x3C` BEGIN → `0x3D` ADD×n → `0x3E` RUN): 依次经过 `poses`, **尖角**。

        `poses` 是最多 32 个位姿, 每个收 `[x,y,z,roll,pitch,yaw]` / `(pos[3], R[3x3])` /
        4x4 齐次矩阵 (见 `move_l`)。协议里没有倒角字段, 所以拐角是**尖的**
        (固件 planner 支持 blend, 但协议没给它开口)。

        ⚠ BEGIN/ADD 中途失败时本方法**不补发任何"清状态"命令**: 固件侧 `RECV` 态自带
        **2s 倒计时**, 到点自行回 `IDLE` 并丢弃已收的路点。
        `wait` 的含义同 `move_l`, 只是"到达位置对得上"那条判据比的是**最后一个路点**
        (`settled=True` 不保证中间路点都被精确经过 —— 固件路径本来就是折线)。
        ⚠ 整段 (BEGIN/ADD/RUN) 持 `Arm._cart_serial`, 故并发调用时的排队时间上界**高于**
        单条 `move_l`/`move_c` (每帧 BEGIN/ADD 各带一个 1.2s ACK 窗口)。
        """
        return cart_mod.move_path(self, poses, speed, wait)

    def poll_cart(self) -> Optional[CartPlan]:
        """认领一条**还没有被取走**的笛卡尔规划结果。

        ⚠ **本方法不再碰链路。** `0x4E` 由读线程在 `_Ack._deliver` 里直接交给收集器
        （`cart.on_reply`），所以"认领"就只是**看一眼收集器的待认领队列**。

        ⚠ 从前它要 `_cart_serial` 非阻塞抢锁、还要"驱动一次取帧"，两件事都是为了不抢帧；
        现在帧根本不会被抢（读线程是**唯一**读者）⇒ 那两件都删了。`None` 的语义不变
        （"此刻可取的结果是空的"），只是不再被"别人正在用链路"影响。
        """
        self._require()
        tok = self._cart.claim_resolved()
        if tok is None:
            return None
        plan = CartPlan.from_reply(tok.reply)
        cart_mod.raise_for_plan(plan)
        return plan

    def set_speed(self, percent: int) -> None:
        """设置**全局调速器**百分比 0..100 (固件 0x21)。

        ⚠ 这是整数百分比, 且固件侧是**全局且持续**的 (`gov_ratio = percent/100`,
        `control_loop.c:1344-1345`), 与 `movej(speed=0..1)` 的**单条轨迹倍率**不是一回事:
        `set_speed(1)` 是 **1% 速度**(全局生效直到 reset/reset 语义的调用),
        按 0..1 思维调用会得到"臂爬行"。
        """
        if isinstance(percent, bool) or not isinstance(percent, int):
            raise InvalidCommandError(
                f"percent 需整数百分比 0..100 (给的是 {percent!r}); "
                f"若手里是 0..1 的倍率请乘 100")
        if not 0 <= percent <= 100:
            raise InvalidCommandError("percent 需 0..100")
        self._cmd(P.CMD_SET_SPEED_PERCENT, bytes([percent]), "set_speed")

    def _arrive(self, target: Sequence[float],
                timeout: Optional[float] = None) -> ST.RobotState:
        """等到连续 `arrive_frames` 拍都判到位、或超时、或故障。

        ⚠ **从"读每一帧"改成"采样最新 state"**：读线程按 100Hz 一直填 `_Ack.state`，
        本方法每见 `status_seq` 前进就重判一次。两个 `_arrive` 同时跑**不再互相饿**
        （`cart.py` 那句"只认'自己读到的帧'的实现会被并发读者饿到超时"随之作废）。

        ⚠ **理论上**多了一个"两拍之间的瞬时故障"的口子（今天逐帧判、新机制采样）。
        固件的故障主要是**锁存**的（`reset` 清不掉的那一类）⇒ 实际不构成风险；
        **但"哪些故障可能瞬时"没有核过**（见 `docs/reports/READ_DESIGN.md` §七）。
        """
        a = self._require()
        tgt = [float(v) for v in target]
        budget = self.move_timeout if timeout is None else float(timeout)

        def done(st: ST.RobotState) -> bool:
            if len(st.joints) != len(tgt):
                return False
            qs = st.q
            return (all(abs(qs[i] - tgt[i]) < self.q_tol for i in range(len(tgt)))
                    and all(abs(d) < self.dq_tol for d in st.dq))

        n_ok = 0                              # 连续到位拍数（防到位瞬间误判/抖动）
        seq0 = a.status_seq
        end = time.monotonic() + budget
        with a._cond:
            while True:
                if a._reader_error is not None:
                    raise TransportError(f"读线程已退出: {a._reader_error}")
                st = a.state
                if st is not None and a.status_seq != seq0:
                    seq0 = a.status_seq
                    if st.faulted:
                        raise MotorFaultError(f"未到位即故障: FAULT {st.fault_detail}")
                    n_ok = n_ok + 1 if done(st) else 0
                    if n_ok >= self.arrive_frames:
                        return st
                left = end - time.monotonic()
                if left <= 0.0:
                    break
                a._cond.wait(left)
        raise MotionTimeoutError(f"未到位, 超时 {budget}s")

    def movej(self, q: Sequence[float], speed: float = 1.0) -> ST.RobotState:
        # 先判连接: 未连接时 self.n 还是 0, 拿它做 arity 校验只会给出误导性的
        # "需要 0 个关节角"。
        a = self._require()
        if not 0.0 <= float(speed) <= 1.0:
            raise InvalidCommandError("speed 需 0..1")
        if len(q) != self.n:
            raise InvalidCommandError(f"movej 需要 {self.n} 个关节角 (N={self.n})")
        payload = P.pack_f32s(q) + struct.pack("<f", float(speed))
        a = self._require()
        self._write_cmd(P.CMD_MOVE_J, payload)
        a.expect(P.RSP_ACK, 1.0, "movej", echo_cmd=P.CMD_MOVE_J)  # ACK 后进入等待到位
        return self._arrive(list(q))

    def movej_sync(self, q: Sequence[float], speed: float = 1.0) -> ST.RobotState:
        """**同步 PTP** movej: 全关节**同时到达**, 末端沿**关节空间直线** q(s) = q0 + s·Δq。

        与 :meth:`movej` 的唯一差别是**轨迹形状**:

        * ``movej``      —— 每轴一条独立的 S 曲线, 各轴**先到先停**, 末端中间轨迹
          **不可预测**(既不是直线也不是任何确定几何) -> 无法做路径校验/避障;
        * ``movej_sync`` —— **一条**路径标量 ``s: 0->1`` 的 S 曲线同时驱动全轴,
          各轴限幅由 ``min(speed_limit_i/|Δq_i|)`` 归约保证, 故所有关节**同一拍到达**,
          且末端落在关节空间线段上(可预测、可校验)。

        ⚠ **代价**: 同步被**最慢的轴**拖住, 整体比 ``movej`` 慢 —— 这是**可选模式**而非
        替换。要快用 :meth:`movej`, 要末端轨迹可预测/可校验用本方法。

        ⚠ 固件状态帧仍报 ``mode=MOVE_J``, **无法**从状态区分同步/异步(但你自己知道发了哪条)。
        ⚠ 保证是**参考级**同步(参考同拍到达); 物理跟踪差异仍在, :meth:`_arrive` 等的仍是
        实测到位。
        """
        a = self._require()          # 先判连接: 未连接时 self.n 还是 0, arity 报错会误导
        if not 0.0 <= float(speed) <= 1.0:
            raise InvalidCommandError("speed 需 0..1")
        if len(q) != self.n:
            raise InvalidCommandError(f"movej_sync 需要 {self.n} 个关节角 (N={self.n})")
        payload = P.pack_f32s(q) + struct.pack("<f", float(speed))
        self._write_cmd(P.CMD_MOVE_J_SYNC, payload)
        a.expect(P.RSP_ACK, 1.0, "movej_sync", echo_cmd=P.CMD_MOVE_J_SYNC)
        return self._arrive(list(q))

    def move_p(self, pose, speed: float = 1.0,
               pos_tol: float = 0.006, rpy_tol: float = 0.03) -> ST.RobotState:
        """点到点运动 (`CMD_MOVE_P 0x02`): 固件后台自己 IK + 走 S 曲线, 本方法轮询
        `get_tcp` 判到位, 返回 `RobotState`。

        这是**关节空间插值, 不是直线** —— 实测 30 mm 的目标, 末端横向摆出 17.6 mm、
        实际走了 51.7 mm(1.72×)。要末端走直线/圆弧/多路点见 :meth:`move_l` /
        :meth:`move_c` / :meth:`move_path` (规划在固件里, 返回 `CartPlan`)。

        ⚠ **本方法只收单个位姿** (固件那条命令的载荷就是一个目标位姿), 且**位姿序列
        不是 `move_path` 的等价物** —— 两者不是同一个东西:

        * `move_p` 是**关节空间**点到点: 中间走成什么样由固件每轴 S 曲线决定, 末端轨迹
          无从保证 (见上面的实测)。
        * :meth:`move_path` 是**笛卡尔**多路点: 固件规划末端依次经过给定位姿。

        从前这里有个"收序列就走 PC 侧规划"的重载, 已移除 (那套 PC 侧规划随之退役);
        现在传序列会抛 `InvalidCommandError` 并指名 `move_path`, 不会静默换语义。
        """
        a = self._require()          # 先判连接: 未连接时的形状报错会掩盖第一因
        if not _is_single_pose(pose):
            raise InvalidCommandError(
                "move_p 只收单个位姿；多个位姿请用 move_path（注意语义不同："
                "move_p 是关节空间点到点，move_path 是笛卡尔多路点）")
        # 固件的 `CMD_MOVE_P` 只吃 [x,y,z,r,p,y], 但 `(pos[3], R[3x3])` 也得收 ——
        # 两种写法在 pylitearm 与固件各自是"本家"形式, 让调用方自己挑哪种是负担。
        if len(pose) == 6 and not any(isinstance(v, (list, tuple)) for v in pose):
            vec = [float(v) for v in pose]
        else:
            from litearm._rot import as_pose, mat_to_rpy
            p, R = as_pose(pose)
            vec = list(p) + list(mat_to_rpy(R))
        pose = vec
        payload = P.pack_f32s(pose) + struct.pack("<f", float(speed))
        self._write_cmd(P.CMD_MOVE_P, payload)
        a.expect(P.RSP_ACK, 1.0, "move_p", echo_cmd=P.CMD_MOVE_P)
        end = time.monotonic() + self.move_timeout
        while time.monotonic() < end:
            # 先看状态: 故障即抛
            st = self.get_state(refresh=True, timeout=0.1).value
            if st is not None and st.faulted:
                raise MotorFaultError(f"move_p: FAULT {st.fault_detail}")
            tcp = self.get_tcp().value
            if st is not None and tcp is not None and self._pose_near(tcp, pose, pos_tol, rpy_tol):
                return st
            time.sleep(0.02)
        raise MotionTimeoutError(f"move_p 未到目标位姿, 超时 {self.move_timeout}s")

    @staticmethod
    def _pose_near(tcp, goal, pos_tol, rpy_tol) -> bool:
        """位置逐分量 + 朝向的**逐分量或旋转等价**判定。

        朝向必须补一条旋转等价判定: 固件 `kin_rot_to_rpy` 在 |pitch|≈π/2(万向锁)时
        **强制 yaw=0**(锁分支 `kin.c:380-384`, 关键句 `:381` `rpy[2] = 0.0f`),
        于是同一个旋转的 rpy 分量可以差很远,
        逐分量比较会**永远判不到位** —— 表现为每次 move_p 都耗满 move_timeout。
        旋转判据是逐分量判据的**超集**(先试逐分量, 不满足再看夹角), 不会放松原语义。
        """
        dp = max(abs(tcp[i] - goal[i]) for i in range(3))
        if dp >= pos_tol:
            return False
        if max(abs(tcp[i] - goal[i]) for i in range(3, 6)) < rpy_tol:
            return True
        return _orient_angle(tcp[3:6], goal[3:6]) < rpy_tol

    def get_tcp(self, timeout: float = 0.6) -> "Msg[Optional[tuple]]":
        """当前末端位姿 pos[3]+rpy[3] (固件当前反馈 FK)。

        返回 :class:`Msg` 信封 (帧 id `RSP_TCP`); 帧短/取不到时 `value=None`。
        """
        a = self._require()
        self._write_query(P.CMD_GET_TCP)
        kind, p = a.expect(P.RSP_TCP, timeout, "get_tcp",
                               echo_cmd=P.CMD_GET_TCP)
        if len(p) >= 24:
            return self._msg(tuple(P.unpack_f32s(p, 0, 6)), P.RSP_TCP)
        return self._msg(None, P.RSP_TCP)

    # ---------- 授权/激活 (固件 1.8.0+; 常量与线格式见 `_protocol` 的 license 段) ----------
    def license(self, timeout: float = 1.0) -> LicenseInfo:
        """读设备授权记录 (`CMD_GET_LICENSE 0x2F` → `RSP_LICENSE 0x4F`, 26B)。

        ⚠ **未激活的臂除 `ENABLE` 外一切照常**（售后/产线要能诊断）⇒ 本方法在未激活时
        **正常返回**（`LicenseInfo.activated is False`），**不抛异常** ——
        "未激活"是一种**状态**, 不是错误。

        ⚠ 刻意**不**返回 :class:`Msg`: `RSP_LICENSE` 没有固件发起的流量 —— 它是
        请求/应答式的**设备身份记录**（激活后不可变）, `Msg` 的 per-type `hz`/`timestamp`
        只会变成"调用方自己轮询的频率", 对设备一无所指（同类: `get_ff_mask` 等派生量）。
        """
        arm = self._require()
        self._write_query(P.CMD_GET_LICENSE)
        _, p = arm.expect(P.RSP_LICENSE, timeout, "get_license")
        return LicenseInfo.decode(p)

    def activate(self, *, cust_id: int, issued: int, flags: int = 0,
                 mac: bytes, timeout: float = 2.0) -> None:
        """提交厂商签发的授权凭据 (`CMD_ACTIVATE 0x3F`)。**须先 `disable()`**。

        `mac` = 16 字节（两个 SipHash-2-4 标签），由**厂商侧**签发工具产出。
        **本包不产生也不需要密钥** —— 规格的硬要求（`litearm-stm32` 的 activation
        设计 §3.3）: 客户侧只要有一份能算 MAC 的代码, 这套机制就归零。

        ⚠ **已武装（使能中或使能在途）会被拒 `ERR{0x3F,0x04}`** —— 与 `save_params`
        同语义: 写 flash 期间电机不得在无监督下保持使能。

        ⚠⚠ **`ERR{0x3F,0x02}` 是聚合档, 光看码会把"已经激活过"误判成失败** ——
        固件把「flags 保留位非 0 / 已存在 / MAC 不符 / 密钥非法 / 写或读回失败」**全折成
        0x02**（`usb_cmd.c:1067-1079`）, 于是上一条 ACK 被丢掉后重发就会拿到 0x02,
        而机器**其实已经解锁**。故本方法在这一档**回读一次 `license()`**: 只有设备确实
        `state == 0` 才抛 —— 这正是厂商侧工具的口径
        (`tools/litearm_license/gui.py:420-438`)。其余码不理睬、直接抛。

        ⚠ 判据是固件那句 `len < 28`（**不是 `== 28`**）: 更长的载荷会被**接受**、
        尾部静默忽略 —— 本方法自己 pack, 故不会撞上; 但手搓帧的人要知道。

        核查用 `license()`（读回才是"落位"的证据; ACK 只说明固件答应了）。
        """
        if len(mac) != 16:
            raise InvalidCommandError(
                f"mac 需 16 字节 (两个 SipHash-2-4 标签), 给的是 {len(mac)}")
        if int(flags) & ~0x1:
            # 保留位必须为 0 (固件会拒, 但那边折进 0x02 的聚合档 ⇒ 本地先说清楚)
            raise InvalidCommandError(
                f"flags 只允许 bit0 (产线码), 给的是 0x{int(flags):X}")
        payload = (struct.pack("<III", int(cust_id), int(issued), int(flags))
                   + bytes(mac))
        # ⚠ 这里**不需要**再写一句 `self._require()`: `_cmd` 自己就调它
        #   (守卫/未连接/终态/fork 四道都在 `_cmd` → `_write_cmd` → `_write_query` → `_require`
        #    这条路上), 多写一句就是一句死代码。
        try:
            self._cmd(P.CMD_ACTIVATE, payload, "activate", timeout=timeout)
        except CommandRejectedError as e:
            if getattr(e, "code", 0) != 0x02:
                raise
            # 0x02 聚合档 ⇒ 只有回读能定性 (见 docstring)
            if self.license().activated:
                return
            raise

    def ik(self, pose: Sequence[float], q_seed: Optional[Sequence[float]] = None,
           timeout: float = 3.0) -> List[float]:
        """固件后台 IK: `pose[6]` + `seed[7]` -> `q[7]`。失败抛 `IKError`。

        ⚠ seed 恒为**模型 7 轴** (`KIN_N`), 与关节数解耦 (固件 `CMD_GET_IK` 约定)。
        台架 1J 上当前反馈只有 1 个值, 故未显式给 `q_seed` 时按
        `BENCH_MODEL_AXIS`(=5) 把台架电机的实测值填进 7 轴种子的对应位, 其余为 0。
        """
        if len(pose) != 6:
            raise InvalidCommandError("ik 需要 pose[6]")
        a = self._require()
        if q_seed is None:
            st = self.get_state().value
            if st is None:
                raise TransportError("ik 需要 q_seed, 但当前取不到状态帧")
            if st.n == P.KIN_N:
                seed = list(st.q)
            else:
                # 台架/非整臂: 关节数 != 模型轴数, 按模型轴映射构造 7 轴种子
                seed = [0.0] * P.KIN_N
                ax = self.bench_model_axis
                if ax is not None and 0 <= ax < P.KIN_N and st.n > 0:
                    seed[ax] = float(st.q[0])
        else:
            seed = list(q_seed)
        if len(seed) != P.KIN_N:
            raise InvalidCommandError(f"q_seed 需 {P.KIN_N} 个 (固件 IK seed 恒为模型 7 轴)")
        self._write_query(P.CMD_GET_IK, P.pack_f32s(list(pose) + seed))
        _, p = a.expect(P.RSP_IK, timeout, "ik", echo_cmd=P.CMD_GET_IK)
        if len(p) < 7 * 4 + 1:
            raise TransportError("ik 应答帧短")
        qs = P.unpack_f32s(p, 0, 7)
        ok = p[7 * 4]
        if not ok:
            raise IKError("目标不可达/IK 失败")
        return qs

    def home(self, *, timeout: Optional[float] = None) -> ST.RobotState:
        """固件 `CMD_HOME 0x2A`: 各轴回 URDF 零位舒展姿。

        `timeout` 仅限关键字 —— 旧签名是 `home(speed)`, 改成位置参数会把速度值
        静默当超时用 (比直接报错更难查)。

        固件侧速度写死 `0.10`(低安全速度), 故本方法**不接受 speed**。与
        `movej([0]*n)` 的差异: 固件注释明确该命令「允许从当前越软限/贴端发起」
        —— 软限位 clamp 作用于**目标**(零位在限内), 起点不影响; 且台架 1J 同样可用。
        仍受 tau 上限 / 看门狗 / 可急停约束, 须先 `enable()`。
        """
        self._write_cmd(P.CMD_HOME, b"")
        a = self._require()
        a.expect(P.RSP_ACK, 1.5, "home", echo_cmd=P.CMD_HOME)
        return self._arrive([0.0] * self.n, timeout=timeout)

    # ---------- 连续伺服 / 透传 ----------
    def move_js(self, q: Sequence[float], dq: Optional[Sequence[float]] = None,
                tau_ff: Optional[Sequence[float]] = None) -> None:
        """单发 move_js。连续伺服需调用方按 ≥10Hz 重发, 否则 0.1s 看门狗 fail-soft。"""
        # ⚠ 终态守卫排在**本地 arity 预检之前**: 终态下 `self.n` 已是 0, 让预检先跑只会
        #   报 "q 需 N 个" (N=0) 这种误导性消息 —— 见 `_reject_if_in_dfu` (非终态空操作)。
        self._reject_if_in_dfu()
        if len(q) != self.n:
            raise InvalidCommandError("move_js q 需 N 个")
        dq = list(dq) if dq is not None else [0.0] * self.n
        if len(dq) != self.n:
            raise InvalidCommandError("move_js dq 需 N 个")
        payload = P.pack_f32s(list(q) + dq)
        if tau_ff is not None:
            if len(tau_ff) != self.n:
                raise InvalidCommandError("move_js tau_ff 需 N 个")
            payload += P.pack_f32s(tau_ff)
        self._cmd(P.CMD_MOVE_JS, payload, "move_js")

    def send_mit(self, idx: int, q: float, dq: float,
                 kp: float, kd: float, tau: float) -> None:
        self._reject_if_in_dfu()     # 排在 idx 预检之前 (终态下 self.n=0 ⇒ 否则报"idx 越界")
        if not 0 <= int(idx) < self.n:
            raise InvalidCommandError("idx 越界")
        payload = bytes([int(idx)]) + P.pack_f32s([q, dq, kp, kd, tau])
        self._cmd(P.CMD_MOVE_MIT, payload, "send_mit")

    def send_mit_all(self, q, dq, kp, kd, tau) -> None:
        self._reject_if_in_dfu()     # 排在 arity 预检之前 (终态下 self.n=0 ⇒ 否则报"需 N 个")
        for arr, label in ((q, "q"), (dq, "dq"), (kp, "kp"), (kd, "kd"), (tau, "tau")):
            if len(arr) != self.n:
                raise InvalidCommandError(f"send_mit_all {label} 需 N 个")
        payload = P.pack_f32s(list(q) + list(dq) + list(kp) + list(kd) + list(tau))
        self._cmd(P.CMD_MOVE_MIT_ALL, payload, "send_mit_all")

    # ---------- FF/动力学 调参 (固件 0x26-0x28/0x31) ----------
    def set_ff_mask(self, mask: int) -> None:
        """写 `ff_mask` (固件 0x27)。

        ⚠ 只接受 `FF_ALL` 范围内的位。固件侧 (与旧 SDK) 会做 `mask & FF_ALL_MASK`
        (`params/params.c:237`), 于是 `0x1000` 这类误用被**静默折成 0 = 前馈全关**
        (重力补偿被关掉, 臂会垂下来), `-1` 折成 `0x1FF = 全开`。宁可在这里报错,
        也不让一个危险值悄悄变成另一个"合法"值。
        """
        m = int(mask)
        if m < 0 or (m & ~P.FF_ALL):
            raise InvalidCommandError(
                f"ff_mask 0x{m & 0xFFFFFFFF:X} 超出范围 —— 有效位只有 "
                f"FF_ALL=0x{P.FF_ALL:03X} (固件侧会静默掩码, 误用高位会被折成 0=前馈全关)")
        self._cmd(P.CMD_SET_FF_FLAGS, struct.pack("<I", m), "set_ff_mask")

    def ff_preset(self, preset: int) -> None:
        if preset not in (0, 1, 2):
            raise InvalidCommandError("preset 需 0(全关)/1(出厂)/2(全开)")
        self._cmd(P.CMD_FF_PRESET, bytes([preset]), "ff_preset")

    #: 0x26 item -> 含义 (与固件 params.c params_ff_vec 的守卫/case 一一对应)
    FF_VEC_ITEMS = {1: "friction", 2: "ki", 3: "i_max", 4: "wall_stiff", 5: "wall_damp",
                    6: "wall_tau_max", 7: "gravity_scale", 8: "inertia_scale",
                    9: "friction_v", 10: "friction_fc0", 11: "friction_fc1",
                    12: "zg_kp", 13: "zg_kd", 14: "zg_damping", 15: "kd_extra"}
    #: 0x28 item -> 含义 (item 9 保留: 写口拒绝, 只给 0x2C 读 ff_mask)
    FF_SCALAR_ITEMS = {1: "fric_db", 2: "wall_margin", 3: "friction_slew",
                       4: "payload_mass", 5: "payload_com", 6: "gravity",
                       7: "friction_model", 8: "fric_v2_eps",
                       10: "drag_gain", 11: "drag_db", 12: "drag_kd_margin",
                       13: "zg_vel_thr", 14: "zg_engage_sec", 15: "zg_engage_kp",
                       16: "zg_engage_kd", 17: "wall_fw_kd", 18: "hold_kp_gain"}
    #: 0x2C 只读 item (写口不接受的)
    FF_SCALAR_RO_ITEMS = {9: "ff_mask"}

    def set_ff_vec(self, item: int, values: Sequence[float]) -> None:
        """写 0x26 向量。item 见 `FF_VEC_ITEMS` (1..15), values 恒 7 个。"""
        if item not in self.FF_VEC_ITEMS or len(values) != 7:
            raise InvalidCommandError("set_ff_vec item∈1..15 且需 7 值")
        self._cmd(P.CMD_SET_FF_VEC, bytes([item]) + P.pack_f32s(values),
                  f"set_ff_vec item{item}")

    def set_ff_scalar(self, item: int, sub: int, value: float) -> None:
        """写 0x28 标量。item 见 `FF_SCALAR_ITEMS`; sub 仅 item 5/6 有意义 (0..2)。"""
        if item not in self.FF_SCALAR_ITEMS or not 0 <= sub <= 2:
            raise InvalidCommandError("set_ff_scalar item∈{1..8,10..18}, sub∈0..2")
        self._cmd(P.CMD_SET_FF_SCALAR,
                  bytes([item, sub]) + struct.pack("<f", float(value)),
                  f"set_ff_scalar{item}")

    # ---------- 参数读回 (0x2B/0x2C; 需固件 1.5.x readback) ----------
    def get_ff_vec(self, item: int, timeout: float = 1.0) -> "Msg[List[float]]":
        """读回 0x26 向量 (7 值)。与 `set_ff_vec` 同 item 编号。

        返回 :class:`Msg` 信封 (帧 id `RSP_FF_VEC`)。
        """
        if item not in self.FF_VEC_ITEMS:
            raise InvalidCommandError("get_ff_vec item∈1..15")
        a = self._require()
        self._write_query(P.CMD_GET_FF_VEC, bytes([item]))
        _, p = a.expect(P.RSP_FF_VEC, timeout, f"get_ff_vec item{item}",
                            echo_cmd=P.CMD_GET_FF_VEC)
        if len(p) < 2 + 7 * 4:
            raise TransportError(f"RSP_FF_VEC 帧短 ({len(p)}B)")
        # [0]=RSP id, [1]=item, 之后 7×f32
        return self._msg(P.unpack_f32s(p, 2, 7), P.RSP_FF_VEC)

    def get_ff_scalar(self, item: int, sub: int = 0, timeout: float = 1.0) -> "Msg[float]":
        """读回 0x28 标量。item 9 为只读扩展 (ff_mask), 见 `get_ff_mask`。

        返回 :class:`Msg` 信封 (帧 id `RSP_FF_SCALAR`)。单发请求/应答式 ⇒ 第一次调用
        `hz == 0.0`, 第二次起等于本调用方自己的轮询频率 (见 `Msg`)。
        """
        if not (item in self.FF_SCALAR_ITEMS or item in self.FF_SCALAR_RO_ITEMS):
            raise InvalidCommandError("get_ff_scalar item∈{1..18}")
        if not 0 <= sub <= 2:
            raise InvalidCommandError("get_ff_scalar sub∈0..2")
        a = self._require()
        self._write_query(P.CMD_GET_FF_SCALAR, bytes([item, sub]))
        _, p = a.expect(P.RSP_FF_SCALAR, timeout, f"get_ff_scalar item{item}",
                            echo_cmd=P.CMD_GET_FF_SCALAR)
        if len(p) < 3 + 4:
            raise TransportError(f"RSP_FF_SCALAR 帧短 ({len(p)}B)")
        # [0]=RSP id, [1]=item, [2]=sub, 之后 f32
        return self._msg(struct.unpack_from("<f", p, 3)[0], P.RSP_FF_SCALAR)

    def get_ff_mask(self, timeout: float = 1.0) -> int:
        """读回 ff_mask (走 0x2C item 9; 值 ≤0x1FF, f32 精确)。

        ⚠ **刻意保持裸 `int`** (不返回 `Msg`), 两个理由:
          · 它是 `get_ff_scalar(9, 0)` 的**标量投影** —— 要那一帧的信封直接调
            `get_ff_scalar(9, 0)` (同一帧的同一份统计), 包两层只会把同一件事说两遍;
          · `int(round(...))` 是本方法的**定义**: `round(Msg)` 没有意义, 它只能消费裸值。
        同理 (聚合而非单帧): `JointParams.all_joint_params` 仍是 `list[JointParam]`。
        """
        return int(round(self.get_ff_scalar(9, 0, timeout=timeout).value))

    def set_gravity_scale(self, gs: Sequence[float]) -> None:
        if len(gs) != 7:
            raise InvalidCommandError("gravity_scale 需 7 值")
        self.set_ff_vec(7, gs)

    def set_inertia_scale(self, isc: Sequence[float]) -> None:
        if len(isc) != 7:
            raise InvalidCommandError("inertia_scale 需 7 值")
        self.set_ff_vec(8, isc)

    def set_payload(self, mass: float, com=(0.0, 0.0, 0.0)) -> None:
        """设末端载荷质量 (item 4) 与质心 (item 5, sub 0..2)。

        ⚠ **`mass` 为负 (或 >20) 不会被拒** —— 固件 `params.c:184` 走
        `ff_clamp(v, 0.0f, 20.0f)` **静默钳制**后回 ACK (`ERR{0x28,0x02}` 的触发条件
        里**没有**质量这一项, 见 `errors.ERR_TEXT[(0x28, 0x02)]`)。所以
        `set_payload(-5)` **会成功**, 而质量被钳成 **0** ⇒ 重力前馈随之改变。
        别靠"传负值探边界"来判断参数有没有写进去: 写进去的是**钳后的值**, 要确认
        请用 :meth:`get_ff_scalar` 读回 (读回的是固件里钳后的真值)。
        `com` 同理逐轴钳 [-1, 1] (`params.c:189`)。
        """
        self.set_ff_scalar(4, 0, float(mass))
        for k, v in enumerate(com):
            self.set_ff_scalar(5, k, float(v))

    def set_gravity_vector(self, g) -> None:
        if len(g) != 3:
            raise InvalidCommandError("gravity 需 3 值")
        for k, v in enumerate(g):
            self.set_ff_scalar(6, k, float(v))

    def save_params(self) -> None:
        """持久化当前运行时参数到 Flash (固件异步, ~0.5-1s)。"""
        self._cmd(P.CMD_PARAM_SAVE, b"", "save_params", timeout=2.5)

    # ---------- DFU (SDK 里唯一的终端态操作) ----------
    def _wait_until_link_lost(self, timeout: float) -> bool:
        """在 `timeout` 窗口内观察链路是否消失; 消失返回 True。

        **判据只有一条：`_Ack._reader_error` 被置** —— 读线程从传输层收到异常就把它
        记在那里（`_die`）并唤醒所有等待者。设备被摘掉 / 跳进 ROM bootloader 时
        `serialposix.py` 抛 `SerialException`（那里注释的原话就是"Linux 上断开的设备
        就是这样"），被 `transport._read_chunk` 包成 `TransportError`。

        ⚠ **比从前更准**：旧实现要**自己**读满窗口去撞那个异常，而它是不持 `_cart_serial`
        的抢帧者（`enter_dfu` 期间撞上在飞命令的 `ACK` 就会吃掉）。现在它只是**看一格
        状态**，一个字节都不碰链路。

        ⚠ **正常关闭绝不能写成这一格**（`close()` 必须先停读线程再关传输）—— 否则
        "我们主动收尾"会被读成"设备没了"。`_Ack.stop_reader()` 那条纪律就是为这个。
        """
        a = self._require()
        end = time.monotonic() + timeout
        with a._cond:
            while a._reader_error is None:
                left = end - time.monotonic()
                if left <= 0.0:
                    return False
                a._cond.wait(left)
            return True

    def enter_dfu(self, timeout: float = DFU_VANISH_TIMEOUT_S) -> None:
        """进 ROM 系统 bootloader (`CMD_ENTER_DFU 0x15`) —— **两段式**, 会话**终态**。

        设备随后重新枚举成 `0483:DF11` (与 CDC 端口无关), 本对象**不再可用**:
        成功返回后任何走 `_require()` / 读口 / 写口的调用 (含 `connect()`/`reconnect()`)
        都抛 :class:`~litearm.errors.ArmIsInDfuError`; 烧完固件请**新建一个 `Arm`**。
        `close()` 照旧可用 (幂等空操作) —— teardown 在任何状态下都不该抛。

        两段式的理由 (固件契约, 逐条核过):

        1. **使能中不跳**: 本地预检 `st.enabled` (状态帧 bit9) 为真就抛
           `InvalidCommandError` 并**一个字节都不发**。固件侧那道门禁是
           `ctrl_is_armed() = enabled || enable_pending` (定义在 `control_loop.c:1334-1336`;
           `:1091` 是 `ctrl_request_enter_dfu` 里把同一谓词**内联**写的那一行), 而
           `enable_pending` 在状态帧里**没有位** ⇒ 它在途时预检放行, 由固件回
           `ERR{0x15,0x03}` (照实透出成 `CommandRejectedError.code == 0x03`,
           **不是**"登记被撤销" —— 那一刻固件还没登记);
        2. **发空载荷** `0x15` (**恒**空: 非空载荷固件回 `ERR{0x15,0x01}`,
           `usb_cmd.c:547`) 等 `RSP_ACK{0x15}`;
        3. `ACK{0x15}` **只表示"已登记"** (`control_loop.c:1096-1097` 只置
           `dfu_pending`) —— 不表示"会跳"、更不表示"已经跳"。登记的**静默撤销**点有
           **三个** (都不回报而 ACK 是成功的): 并发 ENABLE (`:1106`)、交棒前向量表复读
           失败 (`:1142`) 与 1s 总超时兜底 (`:1112`) —— ⚠ 第三条**实际不可达**, 理由与
           三道门控的算术见模块顶部 `DFU_REVOKED_MESSAGE` 那段注释。
           所以还要**等设备真的消失** (`_wait_until_link_lost`)。
        4. 消失 ⇒ 置终态 (关 transport); `timeout`(默认 0.3s) 内**没**消失 ⇒ 抛
           "登记被撤销/未执行" 且 `Arm` **保持可用** —— 那是静默撤销路径的**唯一出口**
           (缺了它就是"ACK 成功了但什么都没发生")。

        ⚠ 固件侧从登记到交棒的上界是 **100ms** (三道门控共用同一个 `age`,
        `control_loop.c:1110-1137`), 本方法的 `timeout` 留的是 **USB 重枚举**的余量。
        ⚠ **已知边界** (不处理, 留底): 固件那条 `ACK{0x15}` 若在应答 FIFO 满时被丢
        (`usb_cmd.c:1126` "满: 丢**最新**"), 而设备随后照样跳了 —— 本方法会从 ACK 等待里
        抛 `TransportError` (读路径先炸), **不**置终态: 那一刻没有任何信号能把"跳了但
        ACK 丢了"与"单纯掉线"分开, 猜错方向比报"链路出错"更坏。此后本对象照旧报链路错。
        ⚠ 本方法**不是**线程安全的串行化点 (与 `disable()`/`emergency_stop()` 一样不取
        `_cart_serial`): 调用期间不应有别的线程在读同一条链路 —— 消失观察窗口读到的帧
        按第七处读循环的纪律计数, 但一条**在飞**的命令的 `ACK` 仍可能被它读走。
        """
        a = self._require()                 # 未连接 ⇒ NotConnectedError (与别的入口一致)
        # ⚠ 预检**读一帧现取的状态** (refresh=True), 不用缓存: 拿一个十几秒前的 `enabled`
        # 去判"能不能跳"两个方向都错 (拒绝合法的升级 / 漏掉真实的使能中)。读不到状态帧
        # (`None`) 时**不拦** —— 门禁的权威在固件 (`enabled || enable_pending` 一律回
        # `0x03`), 本地预检只为可读性; 拿"读不到状态"当拒绝理由会把一次合法升级卡死在
        # 一条与它无关的故障上。
        st = self.get_state(refresh=True).value
        if st is not None and st.enabled:
            raise InvalidCommandError(DFU_ENABLED_MESSAGE)
        self._write_cmd(P.CMD_ENTER_DFU)    # 空载荷; guarded 保持默认 (零重力期同样拒)
        a.expect(P.RSP_ACK, DFU_ACK_TIMEOUT_S, "enter_dfu", echo_cmd=P.CMD_ENTER_DFU)
        if not self._wait_until_link_lost(timeout):
            raise LiteArmError(DFU_REVOKED_MESSAGE)
        try:
            self.close()                    # 与 Arm.close() 同一套收尾 (含收保活线程)
        finally:
            # ⚠ 置位放 `finally`: 设备**确实**没了, 无论收尾是否出岔子, 本对象都不该再
            # 被当成一个活着的会话 (否则后续调用会从读路径抛 TransportError —— 归因
            # 指向"链路故障", 而真相是"我们把它交出去了")。
            self._dfu_entered = True


# ---------------------------------------------------------------------------
# 简易 CLI (巡检/冒烟)
# ---------------------------------------------------------------------------
def _print_state(m: Msg) -> None:
    """打印一帧状态 —— 收 :class:`Msg` 信封 (帧统计与内容是两份信息, 都打出来)。"""
    st = m.value
    print(f"mode={st.mode_name} seq={st.seq} flags={st.flag_names or '-'} "
          f"hz={m.hz:.1f} age={time.monotonic() - m.timestamp:.2f}s")
    for i, j in enumerate(st.joints):
        print(f"  J{i + 1}: q={j.q: .3f} dq={j.dq: .2f} tau={j.tau: .2f} "
              f"T={j.t_mos: .0f}/{j.t_coil: .0f}C err={j.err:02X}")


def main(argv=None) -> int:  # pragma: no cover - CLI
    import argparse
    import sys
    ap = argparse.ArgumentParser(prog="litearm-python",
                                 description="LiteArm STM32 直连控制台")
    ap.add_argument("--port", default=None)
    ap.add_argument("action", nargs="?", default="status",
                    choices=["status", "fw", "enable", "disable", "reset",
                             "emergency", "movej", "home", "tcp"])
    ap.add_argument("targets", nargs="*", type=float)
    ap.add_argument("--speed", type=float, default=0.3)
    args = ap.parse_args(argv)

    arm = Arm(port=args.port).connect()
    try:
        if args.action == "fw":
            print("firmware:", arm.firmware, "| n =", arm.n)
        elif args.action == "status":
            msg = arm.get_state()
            if msg.value is None:
                sys.exit("取不到状态帧 (链路无上行)")
            _print_state(msg)
        elif args.action == "enable":
            arm.enable(); print("enabled")
        elif args.action == "disable":
            arm.disable(); print("disabled")
        elif args.action == "reset":
            arm.reset(); print("reset")
        elif args.action == "emergency":
            arm.emergency_stop(); print("emergency")
        elif args.action == "home":
            # 速度由固件写死 0.10, CLI 的 --speed 对 home 无意义 (只作用于 movej)
            arm.home(); print("home done")
        elif args.action == "movej":
            if len(args.targets) != arm.n:
                sys.exit(f"movej 需 {arm.n} 个关节角")
            arm.movej(args.targets, args.speed)
            print("movej done")
        elif args.action == "tcp":
            msg = arm.get_tcp()
            print(f"tcp: {msg.value} (hz={msg.hz:.1f})")
    finally:
        arm.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
