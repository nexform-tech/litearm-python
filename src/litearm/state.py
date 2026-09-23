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
class RobotState:
    mode: int = 0
    mode_name: str = "INIT"
    flags: int = 0
    flag_names: List[str] = field(default_factory=list)
    seq: int = 0
    joints: List[JointState] = field(default_factory=list)
    joint_fault: int = 0        # 固件 G7 断轴位图 (1.5.0 起上报; 旧布局恒 0)

    @property
    def n(self) -> int:
        return len(self.joints)

    @property
    def enabled(self) -> bool:
        """flags bit9 (固件 1.5.0 起上报; 旧固件恒 False, 不可据此判失能)。"""
        return bool(self.flags & (1 << P.FLAG_ENABLED_BIT))

    @property
    def cart_busy(self) -> bool:
        """固件 flags bit10 = 笛卡尔规划/播放进行中。

        ⚠ **刻意不放进 `FLAG_NAMES`** (本属性就是它唯一的读法)。真正的护栏是
        `_protocol.decode_status` 里那个**隐式的 `range(6)`** ——

            names = [FLAG_NAMES[k] for k in range(6) if flags & (1 << k)]

        —— `flag_names` 只会去看 bit0..bit5, 于是"往 `FLAG_NAMES` 加 `10: "CART_BUSY"`
        什么都不会发生" (加了也读不到)。**这不是"会被打印成故障位"** (实测: 加了照样全绿
        —— 当时套件 **374** 条, 那是**当时的规模**、不是现值, 别拿它核现在的全量输出),
        别照着那句假后果去推理: `fault_detail()` 只拼 `flag_names` 这个**列表**,
        而列表本身由 `range(6)` 决定。
        ⚠ 但"现在无事发生"= 一条防线写在**别人的隐式常量**里 —— 一旦有人把 `range(6)`
        改成 `range(11)` (或改成遍历整张表), bit6..bit10 (mode 三位 + enabled + cart_busy)
        会**一起**变成"故障位"。`test_cart_protocol.py::test_cart_busy_must_not_enter_flag_names`
        就是为这条可能的改动守的。
        """
        return bool(self.flags & (1 << 10))

    @property
    def fault_axes(self) -> List[int]:
        """固件已断轴(失能)的关节号, 0 基 (如 [1, 3] = J2/J4)。"""
        return [i for i in range(P.MAX_JOINTS) if self.joint_fault & (1 << i)]

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
        """⚠ **推断值, 不是读到的位** —— 掉线刚性持位锁存 (固件 `g_arm.drop_hold`)
        是否**可能**在生效。属 S3 的具名暴露 (spec §6.3)。

        先说清**为什么只能是推断**: 固件**不上报**这个量 ——
        ① 状态帧 (6+21N) 里没有它的位; ② 它也没有自己的 `ARM_FLAG_*`
        (`litearm.h` 只定义 bit0..bit5, 而 bit6..bit8 是 mode 三位 / bit9 enabled /
        bit10 cart_busy; **bit11 目前未分配** —— 固件 spec 明文不分配它
        (`litearm-stm32/docs/superpowers/specs/2026-09-13-firmware-cartesian-planning-design.md`:
        "本轮只定义 bit10 一个位, bit11 不分配任何语义"), 构造器也确实从不写它
        (写位的只有 `usb_cmd.c:1196-1211` 那四处); 授权若落地会占它, **SDK 不得先行解读**);
        ③ 固件 `hal/usb_cmd.c` 里 `drop_hold` 只出现在**门禁**处
        (**3 处** —— 该文件里 `drop_hold` 共出现 4 次, 第 4 次是解释档序的注释),
        没有任何上报路径。SDK 侧**唯一**能拿到的是固件自己写下的一条
        **单向蕴含** (固件 `litearm.h` 的 `drop_hold` 声明处原注释):

            "ENABLE 门禁无需另设 —— drop_hold 为真时 joint_fault 必非 0"

        ⇒ 逆否: `joint_fault == 0` ⟹ `drop_hold == False`。**反向不成立**。所以:

        * 本属性为 ``False`` ⇒ **确定没有**该锁存 (上面那条逆否);
        * 本属性为 ``True``  ⇒ **可能**在锁存, 也可能只是一次"只降级单轴"的故障 ——
          温度 / 跟随误差 / 限位 / 超速那几类按用户裁决 (2026-09-17) **只锁该轴、
          不置 drop_hold** (`safety_check.c` 的原注释明说刻意不纳入); 反向的实测记录
          固件自己也留着: `safety_check.c` 的 `REG_REPLY_WINDOW_TICKS` 那段写着
          "宿主 t13 可复现: **joint_fault=0x0008 而 drop_hold=0**" (先拔线再使能)。

        **要确证** drop_hold, 用 `0x06` 错误码 —— 会带 drop_hold 门禁的那些命令
        (`0x01`/`0x03`/`0x04`/`0x05`/`0x07`/`0x2A` 与笛卡尔 5 条 `0x3A~0x3E`) 抛出的
        `CommandRejectedError.code == 0x06` **就是**当拍 `drop_hold` 为真: 那几个收口
        函数与 `cart_gate_ok()` 里 `0x06` **只有**这一个来源。

        ⚠ **上面这个配方在两边都成立（2026-09-22 起）** —— `cmd`/`code` 现在**会过线**，
        承载在 `Error.details`（proto 字段 3，见 `litearm-python/src/litearm/codec.py`
        的 `encode_reply` / `decode_reply`）。
        改动前的实测是：固件回 `ERR [01,2]`，客户端拿到 `CommandRejectedError` 而
        **`.code == 0`、`.cmd == 0`** —— 而 `0x00` 按设计恒等于「固件没有这条命令」，
        两者含义**相反** ⇒ 照上面那句写会**判反**。所以这条不是"少个字段"，是判据错。

        ⚠ **一个仍然要小心的版本窗口**：`cmd`/`code` 是**新填**的键 ⇒ 连到**旧版
        `litearm-server`**（它不填）时客户端读到的仍是 `0`。若你的部署可能新旧混跑，
        判 `drop_hold` 之前先确认服务端版本；或退回"读 `str(e)` 里的 `ERR [cmd,code]`"
        这个笨办法（它在两版上都成立）。
        `0x00` 那条哨兵**不靠这个数值**，它由类名 `UnsupportedByFirmwareError` 承载
        （`decode_reply` 里两条路互补），所以不受版本窗口影响。
        ⚠ **`0x02` MOVE_P 不在其中** —— 它的 dispatch 直接问 `kin_runner_request_move_p`
        (后台 IK), 三档只有 `!enabled`/零重力/忙, **没有** drop_hold 门禁, 永远不会回
        `0x06` (`errors.ERR_TEXT` 里也没有 `(0x02, 0x06)` 这一条)。它是运动类命令里
        唯一的例外 —— 别按"运动类都判"去推。
        ⚠ 另外 `0x10` ENABLE 的 `0x06` **不含** drop_hold 判据 (它判的是 EMERGENCY /
        `joint_fault`, 见 `errors.ERR_TEXT[(0x10, 0x06)]`), 别混。

        ⚠ 名字带 `_inferred` 是**刻意**的 (spec: "不能假装能直接读到"): 一个叫
        `drop_hold` 的布尔会让人以为它来自状态帧, 从而写出"它一变就能自恢复"的代码
        —— 而固件那边的真实语义恰恰是"**必须锁存, 不能随标志位自恢复**"
        (掉线是持续状态, 未修复时自动恢复等于让臂带着一条瘫软的轴继续跑)。
        清它的唯一途径是 `reset()` / `clear_faults()`。

        等价写法是 `bool(self.joint_fault)` —— 具名暴露的意义是让调用方**不必**知道
        上面那条蕴含及其方向。
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
    flags, seq, mode, names, joints, joint_fault = dec
    st = RobotState(mode=mode, mode_name=P.MODE_NAMES.get(mode, str(mode)),
                    flags=flags, flag_names=names, seq=seq,
                    joint_fault=joint_fault)
    st.joints = [JointState(*j) for j in joints]
    return st
