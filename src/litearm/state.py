"""运行状态 dataclass —— 状态帧(G8)的面向对象视图。"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List

from litearm import _protocol as P


@dataclass
class JointState:
    q: float = 0.0
    dq: float = 0.0
    tau: float = 0.0
    t_mos: float = 0.0
    t_coil: float = 0.0
    err: int = 0


@dataclass
class FaultDetail:
    """`CMD_GET_FAULT(0x51)` 的应答 —— 按轴故障原因。

    由 `Arm.get_fault_detail()` 在 `fault_detail_available` 为真时按需读取；
    正常运行时没有这次往返。文案见 `errors.describe_fault()`。
    """
    fault_tick: int = 0
    fault_cause: int = 0
    fault_reason: List[int] = field(default_factory=list)
    drop_hold_reason: int = 0

    @property
    def fault_reasons(self) -> List[tuple]:
        return [(i, r) for i, r in enumerate(self.fault_reason) if r]


@dataclass
class RobotState:
    mode: int = 0
    mode_name: str = "INIT"
    flags: int = 0
    flag_names: List[str] = field(default_factory=list)
    seq: int = 0
    joints: List[JointState] = field(default_factory=list)
    joint_fault: int = 0        # 固件 G7 断轴位图 (1.5.0 起上报; 旧布局恒 0)
    #: 每轴首次锁存原因 (FAULT_R_*, 纯码; 0=未锁存). **只有 1.12.2 的 161B 帧**
    #: 携带; 1.12.3 起状态帧回到 153B, 原因改走 `Arm.get_fault_detail()` (0x51) ——
    #: 1.12.3 上本字段恒为空列表, **不表示"未锁存"**. 判据: `fault_detail_available`.
    #: (填充时的不变量: fault_reason[i] != 0 <=> joint_fault 第 i 位为 1.)
    fault_reason: List[int] = field(default_factory=list)
    #: 整臂掉线刚性持位的首次原因 (DROP_R_*), 旧固件恒 0.
    drop_hold_reason: int = 0

    @property
    def n(self) -> int:
        return len(self.joints)

    @property
    def enabled(self) -> bool:
        """flags bit9 (固件 1.5.0 起上报; 旧固件恒 False, 不可据此判失能)。"""
        return bool(self.flags & (1 << P.FLAG_ENABLED_BIT))

    @property
    def fault_detail_available(self) -> bool:
        """flags bit11：固件 1.12.3 起表示"可用 `get_fault_detail()` 查原因"。

        旧固件恒 False；旧 SDK 不读这一位，因此状态帧布局保持不变。
        """
        return bool(self.flags & (1 << P.FLAG_FAULT_DETAIL_BIT))

    @property
    def cart_busy(self) -> bool:
        """固件 flags bit10 = 笛卡尔规划/播放进行中。

        ⚠ **刻意不放进 `FLAG_NAMES`** (本属性就是它唯一的读法)。真正的护栏是
        `_protocol.decode_status` 里那个**隐式的 `range(6)`** ——

            names = [FLAG_NAMES[k] for k in range(6) if flags & (1 << k)]

        —— `flag_names` 只会去看 bit0..bit5, 于是"往 `FLAG_NAMES` 加 `10: "CART_BUSY"`
        什么都不会发生" (加了也读不到)。**这不是"会被打印成故障位"** (实测: 加了当时
        全套用例照样全绿; 那些用例现已移除, 别依赖"测试会红"来防这件事),
        别照着那句假后果去推理: `fault_detail()` 只拼 `flag_names` 这个**列表**,
        而列表本身由 `range(6)` 决定。
        ⚠ 但"现在无事发生"= 一条防线写在**别人的隐式常量**里 —— 一旦有人把 `range(6)`
        改成 `range(11)` (或改成遍历整张表), bit6..bit10 (mode 三位 + enabled + cart_busy)
        会**一起**变成"故障位"。改 `range(6)` 或 `FLAG_NAMES` 前先检查这一点。
        """
        return bool(self.flags & (1 << 10))

    @property
    def fault_axes(self) -> List[int]:
        """固件已断轴(失能)的关节号, 0 基 (如 [1, 3] = J2/J4)。"""
        return [i for i in range(P.MAX_JOINTS) if self.joint_fault & (1 << i)]

    @property
    def fault_reasons(self) -> List[tuple]:
        """已锁存轴的 `(关节号, 原因码)`, 0 基。**只有 1.12.2 的 161B 帧会填**;
        1.12.3 起恒为空 —— 原因请用 `Arm.get_fault_detail()`。

        原因码是固件原始枚举（`FAULT_R_*`），不含文案 —— 与 `fault_axes` 的区别
        只是多带"为什么"。`fault_axes` 仍保留，便于既有调用方不改。
        """
        return [(i, r) for i, r in enumerate(self.fault_reason) if r]

    @property
    def q(self) -> List[float]:
        return [j.q for j in self.joints]

    @property
    def dq(self) -> List[float]:
        return [j.dq for j in self.joints]

    @property
    def tau(self) -> List[float]:
        return [j.tau for j in self.joints]

    @property
    def drop_hold_inferred(self) -> bool:
        """⚠ **旧固件上的推断值** —— 掉线刚性持位锁存 (固件 `g_arm.drop_hold`)
        是否**可能**在生效。属 S3 的具名暴露 (spec §6.3)。

        1.12.3 起**不需要推断**：状态帧 flags bit11 = 有故障详情可查
        (`fault_detail_available`)，随后 `Arm.get_fault_detail()` 的
        `drop_hold_reason` 直接给出 0/1/2。本属性保留给**旧固件**
        (bit11 恒 0、没有 0x51 查询)。

        旧固件**不上报**这个量：
        ① 状态帧 (6+21N) 里没有它的位；② 它没有自己的 `ARM_FLAG_*`；
        ③ `usb_cmd.c` 里 `drop_hold` 只出现在门禁处，没有上报路径。
        SDK 侧能拿到的是固件自己写下的一条**单向**蕴含
        (固件 `litearm.h` 的 `drop_hold` 声明处原注释):

            "ENABLE 门禁无需另设 —— drop_hold 为真时 joint_fault 必非 0"

        ⇒ 逆否: `joint_fault == 0` ⟹ `drop_hold == False`。**反向不成立**。所以:

        * 本属性为 ``False`` ⇒ **确定没有**该锁存；
        * 本属性为 ``True``  ⇒ **可能**在锁存，也可能只是一次"只降级单轴"的故障 ——
          温度 / 跟随误差 / 限位 / 超速那几类按用户裁决 (2026-09-17) **只锁该轴、
          不置 drop_hold**。

        **旧固件要确证** drop_hold，用会带该门禁的命令抛出的
        `CommandRejectedError.code == 0x06` (`0x01`/`0x03`/`0x04`/`0x05`/`0x07`/
        `0x2A` 与笛卡尔 `0x3A~0x3E`)。1.12.3 起直接用
        `get_fault_detail().drop_hold_reason`。

        ⚠ `0x02` MOVE_P 与 `0x10` ENABLE 的 `0x06` **不含** drop_hold 判据，
        别按"运动类都判"去推。

        ⚠ 名字带 `_inferred` 是**刻意**的：一个叫 `drop_hold` 的布尔会让人以为它
        来自状态帧，从而写出"它一变就能自恢复"的代码 —— 而固件语义是
        "**必须锁存，不能随标志位自恢复**"（掉线是持续状态，未修复时自动恢复等于
        让臂带着一条瘫软的轴继续跑）。清它的唯一途径是
        `reset()` / `clear_faults()`。

        等价写法是 `bool(self.joint_fault)`。
        """
        return bool(self.joint_fault)

    @property
    def faulted(self) -> bool:
        """FAULT 位 / EMERGENCY / **任一轴被固件断轴(G7)**。

        固件单轴锁存（断轴/越限/超速/温度）**不一定**置全局 FAULT 位，只置
        对应的 flags 位并把该轴记进 joint_fault —— 若不看 joint_fault, movej 不会
        早失败, 只会耗满超时才报"未到位"。
        """
        return bool(self.flags & 1) or self.mode == 6 or self.joint_fault != 0

    @property
    def fault_detail(self) -> str:
        """人类可读的故障描述 (安全 flags + 断轴号)。"""
        parts = []
        if self.flag_names:
            parts.append("flags=" + ",".join(self.flag_names))
        if self.joint_fault:
            parts.append("断轴=" + ",".join(f"J{a + 1}" for a in self.fault_axes))
        return " ".join(parts) if parts else "无故障位"


def decode_state(payload: bytes) -> "RobotState":
    dec = P.decode_status(payload)
    if dec is None:
        raise ValueError("非法状态帧")
    (flags, seq, mode, names, joints, joint_fault,
     fault_reason, drop_hold_reason) = dec
    st = RobotState(mode=mode, mode_name=P.MODE_NAMES.get(mode, str(mode)),
                    flags=flags, flag_names=names, seq=seq,
                    joint_fault=joint_fault,
                    fault_reason=list(fault_reason),
                    drop_hold_reason=int(drop_hold_reason))
    st.joints = [JointState(*j) for j in joints]
    return st
