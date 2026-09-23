"""错误层级 —— 镜像 pylitearm 常用错误名, 本包独立定义(不依赖 pylitearm 源码)。"""


class LiteArmError(Exception):
    """本包所有错误基类。"""


class NotConnectedError(LiteArmError):
    """未连接/串口未开时调用。"""


class ForkedSessionError(NotConnectedError):
    """在 `fork()` 出的**子进程**里使用**父进程**建立的会话 —— 本包 fail-closed 拒绝。

    为什么必须拒绝（不是保守，是唯一正确的选择）：

    * 子进程**继承 fd**（`os.write` 照样把字节写到 CDC 上）但**不继承线程** ——
      线程不被 `fork` 复制 ⇒ 读线程在子进程里不存在;
    * 于是命令会**真的发出去**，而应答永远没人读 ⇒ 子进程**等满超时**报
      "无应答"，用户以为命令失败、重试 ⇒ **重复下发**。
      实测（真 pty 当串口，不是桩）：子进程 `movej` 抛 `MotionTimeoutError` 的同时，
      父进程的排水线程**读到了那条 MOVE_J 帧**;
    * `get_state()` 那种读路径更隐蔽：它**不报错**，只是静默返回继承来的**陈旧** `state`。

    所以口径是：**子进程里一个字节都不下发、也不认任何缓存**，立刻抛本异常。
    子进程要用臂 ⇒ **新建一个 `Arm`**。

    ⚠ `close()` 在子进程里**仍可调**（收尾不能被拦死），但它**只做会话状态清理**，
    **不碰传输层** —— 那会取一把从父进程继承来、永远不会释放的锁（见 `Arm.close()`）。
    ⚠ 在**父进程里** `fork` 之前先 `close()`、或者在 `fork` **之后**才 `connect()`，
    是子进程要用臂的两种正路。

    ⚠ 继承自 `NotConnectedError`：`except NotConnectedError` 与 `except LiteArmError`
    都抓得到它（这个会话在本进程里**确实**不可用）。
    """


class TransportError(LiteArmError):
    """串口读写/帧 CRC 校验失败。"""


class FirmwareMismatchError(LiteArmError):
    """固件版本不符合约定 (应 Litearm<主.次.修>-{7J|1J}) 或不满足最低版本。"""


class InvalidCommandError(LiteArmError):
    """参数/命令非法 (长度、取值越界等)。"""


class MotorFaultError(LiteArmError):
    """状态帧 flags FAULT 或进入 EMERGENCY (含 G7 单关节故障降级)。"""


class MotionTimeoutError(LiteArmError):
    """move 超时未到位。"""


class IKError(LiteArmError):
    """IK 求解失败 / 目标不可达。"""


class CommandRejectedError(LiteArmError):
    """固件显式 ERR (payload 含 `[cmd, code]`)。

    两个字段都是**可编程判定**用的, 消息里另有可读文本 (由 :func:`err_reason` 拼出):

    * ``.cmd``  —— 固件回显的命令码。**以它为准**, 不要拿"我刚发了哪条"去推断:
      异步路径 (``0x02`` MOVE_P 的 IK 收尾 / ``0x25`` 参数保存) 的 ERR 与它对应的
      那条命令**不在同一次收发里** (:attr:`UnsupportedByFirmwareError` 的判定同理)。
    * ``.code`` —— 错误码。语义**逐命令**定义 (同一个 `0x03` 在 `0x01` 上是"未使能"、
      在 `0x10` 上是"可重试"、在 `0x23` 上是"新限位关不住在途目标"), 查
      :data:`ERR_TEXT` / :func:`err_reason`; ``0x00`` 恒为"固件没有这条命令"
      (见 :class:`UnsupportedByFirmwareError`)。

    ⚠ 带 drop_hold 门禁的那些运动命令 (`0x01`/`0x03`/`0x04`/`0x05`/`0x07`/`0x2A` 与
    笛卡尔 `0x3A~0x3E`) 的 ``0x06`` 恒等于「掉线刚性持位锁存 (``drop_hold``)」——
    那是 SDK 唯一能**确证**该锁存存在的信号; 状态帧里读不到它 (见
    :attr:`litearm.state.RobotState.drop_hold_inferred`)。
    ⚠ 两个例外: `0x02` MOVE_P **没有**该门禁 (永不回 `0x06`); `0x10` ENABLE 的
    `0x06` 判的是 EMERGENCY / `joint_fault`, **不含** drop_hold —— 见
    :data:`ERR_TEXT` 里这两条的文本。
    """

    def __init__(self, message: str, cmd: int = 0, code: int = 0):
        super().__init__(message)
        self.cmd = cmd
        self.code = code


class UnsupportedByFirmwareError(CommandRejectedError):
    """固件**没有实现**这条命令 (ERR 码 == 0x00)。

    固件 `usb_cmd.c` 的 `default` 分支对未实现命令只回 `ERR{cmd,0x00}` 且**无任何
    副作用**; 已实现命令的错误码一律落在 `0x01..0x07` (0x07 = 模型掩码不符, 2026-09-14 增)。
    故 `0x00` 是「固件无此命令」
    的唯一稳定哨兵 —— 比版本号可靠 (固件版本号不随命令增删而变)。

    是 `CommandRejectedError` 的子类, 既有捕获 `CommandRejectedError` 的调用方不受影响。
    """


class CartesianPlanError(LiteArmError):
    """固件规划被拒 (IK 无解 / 三点共线 / 超容量 / 越限位)。

    对应固件 `RSP_CART_PLAN` 的 `err` 字段 (`cart_err_t`) 里
    2=COLLINEAR / 3=TOO_LONG / 4=LIMIT 三种 —— 都是**这条轨迹本身不成立**,
    臂一步没动。

    刻意**不继承** `CommandRejectedError`: 固件这里回的是规划结果, 不是
    `ERR{cmd, code}` 形态的「拒绝执行这条命令」, 硬套会凭空多出语义错误的
    `cmd`/`code` 字段 (尤其 `code=0x00` 在既有约定里意为「固件无此命令」)。
    """


class MotionSupersededError(LiteArmError):
    """这条笛卡尔请求被新请求取代 —— **预期内的接管，不是失败**。

    对应 `err=5=CANCELED`。抢占是正常用法 (新目标来了就接管旧的), 所以
    调用方必须能把它与「规划失败」分开: 混进通用异常会让正常抢占走成故障
    分支, 触发本不该有的错误恢复。**不得继承 `CartesianPlanError`**。
    """


class CartReplyLostError(LiteArmError):
    """`0x4E` 应答丢失（固件单槽 pending 在突发下会吞应答）—— **未知结局**。

    固件侧那份应答只有**一个槽位**: 同一 main 排空窗口内登记 ≥3 条笛卡尔
    请求时会吞掉中段应答。这是本 SDK 自造的语义, **不是固件回码** ——
    臂可能已经动了, 也可能没有, 只能靠回读实际状态判定, 不可当失败重发。
    """


class ArmIsInDfuError(LiteArmError):
    """本 `Arm` 已把设备交棒进 ROM bootloader —— 会话的**终态**, 不可再用。

    由 :meth:`litearm.Arm.enter_dfu` 置位, 且**只在确认设备真的从 CDC 上消失之后**
    (判据 = 读路径抛 :class:`TransportError`, 见那个方法): 那一刻本对象关掉 transport,
    此后任何走 `_require()` / 读口 / 写口的调用 —— **含 `connect()` / `reconnect()`** ——
    都抛本异常。设备随后重新枚举成 `0483:DF11` (ROM bootloader, 与 CDC 端口无关);
    烧完固件要接着用臂, 请**新建一个 `Arm`** (本对象刻意不提供"复活"入口: 隐式清掉终态
    会让"这台臂已经交出去了"这件事在代码里失去唯一可判定的信号)。

    刻意**不继承** `NotConnectedError`: 那个的语义是"还没连 / 链路断了, 连上即可", 而这里
    不是一次可恢复的掉线 —— 是本会话按调用方要求**主动**把设备交了出去。混在一起会让
    "断连就重连"的恢复逻辑把 DFU 当成普通掉线, 一路重试到超时。

    ⚠ 与"登记被撤销"不是一回事: 后者 (`enter_dfu` 收到 ACK 但设备没消失) 抛
    `LiteArmError` 且**对象照旧可用** —— 那时设备还在 CDC 上, 什么都没发生。
    """


# ===========================================================================
# (cmd, code) 语义表 —— 固件 `ERR` 应答的**唯一**解读处 (spec §6.3)
#
# 真源 = 固件仓（`LITEARM_FW_DIR` 指向的那棵树; 2026-09-19 逐条核过, 见下"穷举口径"）:
#   · 发射点 `User/litearm/hal/usb_cmd.c` 的 `usb_cmd_dispatch()` —— **94 处**
#     `usb_cmd_reply(RSP_ERR, ...)`; 另 **2 处**在 `User/litearm/kinematics/kin_runner.c`
#     （后台 IK 的收尾, 不在 usb_cmd.c 里 —— 只扫 usb_cmd.c 会漏掉它们）。
#   · **13 处**的载荷**无法字面解析**（`{cmd, rc}` / `{0x10, e}` …）—— 其中 **9 处**
#     的**错码**是变量, 另 **4 处**的**命令码**是变量（那 4 处的错码是字面量; 有 1 处
#     两者都是变量, 已归入前 9 处）: 它们的取值集合由 `control_loop.c` / `cart_exec.h`
#     的收口函数定义, 字面上无从解析, 逐条如下 ——
#       `ctrl_accept_move_j/_sync/_js/_mit/_mit_all` -> {0x03, 0x04, 0x06}
#       `ctrl_enable`                              -> {0x03, 0x06, 0x07}
#       `ctrl_request_enter_dfu`                   -> {0x02, 0x03}
#       `cart_req_*`（cart_exec.h:61-63）          -> {0x02, 0x04}
#
# ⚠ **不要照抄 `litearm-stm32/tools/arm_blackbox_test.py:100-128` 的 `ERR_TEXT`** ——
#   那是**旧固件**时代的表, 缺: 笛卡尔 `0x3A~0x3E` 的全部码、S4 fix 新增的 `0x06`
#   （掉线刚性持位）、`{0x25,0x03}`（异步保存失败）、`0x15`(DFU) 与 HYY 模型导入那些码。
#   本表是**重新穷举**的结果, 不是它的修订版。
#
# ⚠ 表里有两处**刻意保留位置、但当前 master 固件不会发出**的条目
#   （`0x26 item7` / `0x28 item6` 的武装门禁, spec R5）—— 它们的出处是
#   `origin/feat/hyy-model-import`（1.5.3 `cdb744a` 不是 master 祖先）。留着是为了
#   产线若烧那条分支时 SDK 的解读不落回通用档; 表里的注释已标出分支归属。
# ===========================================================================

#: 具体档: `(cmd, code) -> 语义`。**逐条取自固件原注释, 不自行起名。**
#: ⚠ `code == 0x00` 一律不在此表 —— 它**恒**等于"固件没有这条命令"（唯一真源是
#: `usb_cmd.c` 的 `default` 分支, 由 `tests/test_protocol_sync.py` 的
#: `test_err_code_zero_only_comes_from_default_branch` 钉住）, 由通用档承担。
ERR_TEXT = {
    # ---- 0x01 MOVE_J（每轴一条独立 S 曲线; 0x07 是同步版, 同码同语义）----
    (0x01, 0x01): "载荷长度不足 (需 4×N+4 字节)",
    (0x01, 0x02): "含非有限值, 或 sp 越界 —— sp 是 0..1 的**小数倍率** (传 30 想表达 "
                  "30% 属单位陷阱, 固件显式拒而不静默 clamp 成 100%)",
    (0x01, 0x03): "未使能, 或 EMERGENCY 锁存 (ctrl_accept_move_j)",
    (0x01, 0x04): "零重力(拖动示教)进行中 —— 须显式 zero_g off 退出, 固件不隐式退出",
    (0x01, 0x06): "掉线刚性持位锁存 (drop_hold) 中 —— 须先 reset/clear_faults",

    # ---- 0x07 MOVE_J_SYNC（载荷与 0x01 逐字节相同, 只换轨迹形状）----
    (0x07, 0x01): "载荷长度不足 (需 4×N+4 字节)",
    (0x07, 0x02): "含非有限值, 或 sp 越界 (同 0x01 的 0..1 小数倍率约定)",
    (0x07, 0x03): "未使能, 或 EMERGENCY 锁存 (ctrl_accept_move_j_sync)",
    (0x07, 0x04): "零重力(拖动示教)进行中",
    (0x07, 0x06): "掉线刚性持位锁存 (drop_hold) 中 —— 须先 reset/clear_faults",

    # ---- 0x02 MOVE_P（固件后台 IK + 走 S 曲线）----
    (0x02, 0x01): "载荷长度不足 (需 28 字节: pose[6] + sp)",
    (0x02, 0x02): "位姿含非有限值, 或 sp 越界 (0..1 小数倍率)",
    (0x02, 0x03): "⚠ **二义码, 两处发射点语义不同**: ① dispatch 的「未使能 / "
                  "EMERGENCY 锁存」(usb_cmd.c, 有 enabled 前置) ② **后台 IK 失败** —— "
                  "原注释「不可达 / 解非法」(kin_runner.c, 该处**没有** enabled 前置, "
                  "到达它时臂本来是使能的)。上位机**无法**从本码区分两者, "
                  "只能按「这条 move_p 没成立」处置 (不要据此判失能)",
    (0x02, 0x04): "零重力(拖动示教)进行中",
    (0x02, 0x05): "IK 后台通道忙 (已有一次后台求解在途)",

    # ---- 0x3A / 0x3B MOVE_L / MOVE_C（固件原生笛卡尔, #if LITEARM_CART_PLAN）----
    # 门禁三档由 cart_gate_ok() 统一发: !enabled(0x03) -> drop_hold(0x06) -> 零重力(0x04);
    # 收口码由 cart_reply() 透传 cart_req_* 的 rc: {0x02 = BADARG, 0x04 = STATE}。
    (0x3A, 0x01): "载荷长度不足 (需 28 字节)",
    (0x3A, 0x02): "位姿含非有限值, 或 sp 越界 (0..1 小数倍率)",
    (0x3A, 0x03): "未使能 (笛卡尔门禁 cart_gate_ok)",
    (0x3A, 0x04): "零重力中 (门禁) 或 状态机不允许 —— RECV 会话未收尾时不许插队 "
                  "(cart_req_movel)",
    (0x3A, 0x06): "掉线刚性持位锁存 (drop_hold) 中 —— 须先 reset/clear_faults",
    (0x3B, 0x01): "载荷长度不足 (需 52 字节: 途经点 + 终点 + sp)",
    (0x3B, 0x02): "位姿含非有限值, 或 sp 越界 (0..1 小数倍率)",
    (0x3B, 0x03): "未使能 (笛卡尔门禁)",
    (0x3B, 0x04): "零重力中 (门禁) 或 状态机不允许 —— RECV 会话未收尾时不许插队 "
                  "(cart_req_movec)",
    (0x3B, 0x06): "掉线刚性持位锁存 (drop_hold) 中",

    # ---- 0x3C / 0x3D / 0x3E CART_BEGIN / ADD / RUN（多路点三段式）----
    # ⚠ BEGIN/ADD 刻意**不走** cart_gate_ok（RECV 态不占用臂）, 但三档判据与档序
    #   逐档一致, 由各自的 case 直接发; RUN 走完整门禁。
    (0x3C, 0x01): "载荷长度不足 (需 5 字节: 点数 n + sp)",
    (0x3C, 0x02): "sp 非有限/越界, 或点数 n 越界 (须 1..CART_MAX_GOAL=32)",
    (0x3C, 0x03): "未使能",
    (0x3C, 0x04): "零重力中 或 状态机不允许 —— 只有 IDLE 能开新会话 (含 RECV 重入)",
    (0x3C, 0x06): "掉线刚性持位锁存 (drop_hold) 中",
    (0x3D, 0x01): "载荷长度不足 (需 25 字节: idx + pose[6])",
    (0x3D, 0x02): "位姿含非有限值 (dispatch), 或 idx 越界 (cart_req_add)",
    (0x3D, 0x03): "未使能",
    (0x3D, 0x04): "零重力中, 或 非 RECV 态, 或该 idx 已 ADD 过 (去重)",
    (0x3D, 0x06): "掉线刚性持位锁存 (drop_hold) 中",
    (0x3E, 0x03): "未使能 (笛卡尔门禁)",
    (0x3E, 0x04): "零重力中 (门禁) 或 状态机不允许 —— 非 RECV 态, 或点名不满 "
                  "(会话不完整, cart_req_run)",
    (0x3E, 0x06): "掉线刚性持位锁存 (drop_hold) 中",

    # ---- 0x03 MOVE_JS（流式关节透传）----
    (0x03, 0x01): "载荷长度不足 (需 8×N 字节; 带 tau_ff 时 12×N)",
    (0x03, 0x02): "q/dq(/tau_ff) 含非有限值",
    (0x03, 0x03): "未使能, 或 EMERGENCY 锁存 (ctrl_accept_move_js)",
    (0x03, 0x04): "零重力(拖动示教)进行中",
    (0x03, 0x06): "掉线刚性持位锁存 (drop_hold) 中",

    # ---- 0x04 MOVE_MIT（单关节透传）----
    (0x04, 0x01): "载荷长度不足 (需 21 字节)",
    (0x04, 0x02): "idx 越界, 或 q/dq/kp/kd/tau 含非有限值",
    (0x04, 0x03): "未使能, 或 EMERGENCY 锁存 (ctrl_accept_move_mit)",
    (0x04, 0x04): "零重力(拖动示教)进行中",
    (0x04, 0x06): "掉线刚性持位锁存 (drop_hold) 中",

    # ---- 0x05 MOVE_MIT_ALL（全臂单帧透传）----
    (0x05, 0x01): "载荷长度不足 (需 20×N 字节)",
    (0x05, 0x02): "q/dq/kp/kd/tau 含非有限值",
    (0x05, 0x03): "未使能, 或 EMERGENCY 锁存 (ctrl_accept_move_mit_all)",
    (0x05, 0x04): "零重力(拖动示教)进行中",
    (0x05, 0x06): "掉线刚性持位锁存 (drop_hold) 中",

    # ---- 0x06 ZERO_G（进入/退出拖动示教; 重发即保活）----
    (0x06, 0x01): "载荷长度不足 (需 1 字节: on)",
    (0x06, 0x02): "拒绝: EMERGENCY 锁存（进入与退出都拒）, 或**进入**时未使能 "
                  "(退出 on=0 幂等, 未使能也收)",

    # ---- 0x2A HOME（回零位舒展姿; 内部复用 ctrl_accept_move_j）----
    (0x2A, 0x03): "未使能 (dispatch 前置) 或 EMERGENCY 锁存 (ctrl_accept_move_j)",
    (0x2A, 0x04): "零重力(拖动示教)进行中",
    (0x2A, 0x06): "掉线刚性持位锁存 (drop_hold) 中",

    # ---- 0x10 ENABLE（使能全关节; 码位决定**是否值得重试**, 见 ENABLE_RETRYABLE_CODES）----
    (0x10, 0x08): "**未激活（未授权）—— 重发无用, 无旁路**。固件 `ctrl_enable()` 的**第一条**"
                  "判据 (`control_loop.c:944`) 就是 `!license_is_activated()`, 故本码说的是"
                  "「这台臂有没有被授权」, 而不是「现在能不能使能」—— 后者优先于其余所有码。"
                  "补救: 用 `Arm.license()` 查 `state`, 再拿厂商签发的凭据走 `Arm.activate()`;"
                  "⚠ 规格**明令不得**为这道门禁加任何旁路 (编译宏/调试开关/隐藏命令/参数位)。",
    (0x10, 0x03): "可重试 —— 电机反馈未齐 / CMODE 首写 (含 CMODE 回读不良已补写, "
                  "重发即可)",
    (0x10, 0x06): "**锁存, 须先 RESET** —— EMERGENCY 或 joint_fault 非 0 "
                  "(重发无用)。⚠ 码值虽与运动命令的 0x06 相同, 但**判据不同** —— "
                  "本档只看 EMERGENCY / joint_fault, 不判掉线刚性持位锁存",
    (0x10, 0x07): "部分轴 CMODE 补写预算耗尽仍未进 MIT —— **重发无用**, 须现场排查 "
                  "(该状态下不使能: 位置/速度模式的轴收到 MIT 帧不执行, "
                  "使能等于让它失力下垂)",

    # ---- 0x15 ENTER_DFU（SDK 里唯一的终端态操作: 进 ROM bootloader）----
    (0x15, 0x01): "载荷必须为空 (本命令不吃参数)",
    (0x15, 0x02): "ROM 系统 bootloader 向量表无效 —— 跳过去也起不来, 故当场拒 "
                  "(不是「答应了却没做」)",
    (0x15, 0x03): "使能中, 或使能在途 (enable_pending) —— 跳转会停 TIM3, "
                  "电机 100ms 后松开, 有重力负载则下垂",

    # ---- 0x3F ACTIVATE（提交授权凭据; 固件 1.8.0+）----
    # ⚠ 线上码除**长度**沿用 0x01 外, 其余**一律折成 0x02**（固件 `usb_cmd.c:1067-1079`）——
    #   故 0x02 **不等于"你的码不对"**: "已经激活过了"与真失败混在同一档。
    #   ⇒ 调用方（含本包的 `Arm.activate()`）必须**回查 `0x2F` 看 state** 才能定性。
    (0x3F, 0x01): "载荷长度不足 (需 ≥28 字节) —— 注意判据是 `< 28`, 更长的载荷会被接受并"
                  "**静默忽略尾部**",
    (0x3F, 0x02): "**聚合档, 必须回查 `0x2F` 才能定性** —— flags 保留位非 0 / 已经激活过 / "
                  "MAC 不符 / 固件编译进来的密钥非法 / 写或读回失败, 全折叠成本码。"
                  "⚠ 其中「已经激活过」意味着设备**可能其实已经解锁**（例如上一条 ACK 被"
                  "丢掉后重发）—— 把它当失败会让产线重复返工, 故**不要**据本码判机器没激活",
    (0x3F, 0x04): "已武装 (使能中或使能在途) 须先失能 —— 与 0x25 同语义: 写 flash 期间"
                  "电机不得在无监督下保持使能",
    (0x3F, 0x05): "flash 忙 (main 正在整扇区保存参数) —— 稍后重试即可",

    # ---- 0x20 / 0x21 运行期旋钮（都只判长度）----
    (0x20, 0x01): "载荷长度不足 (需 1 字节: mode)",
    (0x21, 0x01): "载荷长度不足 (需 1 字节: 百分比)",

    # ---- 0x42 GET_IK / 0x49 KIN_BENCH（后台求解通道）----
    (0x42, 0x01): "载荷长度不足 (需 52 字节: pose[6] + seed[7])",
    (0x42, 0x02): "pose 或 seed 含非有限值",
    (0x42, 0x05): "忙 —— 已有一次后台 IK 在途 (**成功登记时不即时应答**, 等 RSP_IK)",
    (0x49, 0x05): "忙 —— 已有一次 kin_bench 在途",

    # ---- 0x22 / 0x23 / 0x24 关节参数 ----
    (0x22, 0x01): "载荷长度不足 (需 13 字节: idx + kp + kd + tau_max)",
    (0x22, 0x02): "idx 越界, 或 kp/kd/tau_max 非法 (NaN 或 kp<0 / kd<0 / tau_max<=0)",
    (0x23, 0x01): "载荷长度不足 (需 9 字节: idx + q_min + q_max)",
    (0x23, 0x02): "数值非法 (NaN / q_min>=q_max), 或 idx 越界, 或**放宽**请求"
                  "(固件只许收窄, 照编译期默认值取交集 —— 原实现先写再判, "
                  "留下半写状态)",
    (0x23, 0x03): "新限位关不住在途目标 —— 武装中且该轴的当前参考/在途轨迹目标落在"
                  "新区间之外; **笛卡尔在途 (state != IDLE) 或同步轨迹在途时一律回本码** "
                  "(路径的未来点不在判据视野里)",
    (0x24, 0x01): "载荷长度不足 (需 1 字节: idx) —— 全局约定, 原与 0x02 共用后者",
    (0x24, 0x02): "idx 越界",

    # ---- 0x25 / 0x36 参数固化与恢复出厂 ----
    (0x25, 0x04): "武装中 (enabled 或 enable_pending 在途) —— 须先失能。"
                  "**两处都会发本码**: 受理时, 以及异步擦写执行前对武装态的复查",
    (0x25, 0x03): "**异步保存失败** —— flash_store_save() 返回 false (擦写未成功)。"
                  "⚠ 与 0x04 不同, 它**在受理之后**才到达 (main 循环里执行)",

    (0x36, 0x04): "武装中 (enabled 或 enable_pending 在途) —— 须先失能",
    (0x36, 0x05): "flash 参数保存进行中 —— **忙, 稍后重试** (两处判据: "
                  "g_param_save_active / flash_store_is_saving; 交错会毁参数)",
    (0x36, 0x03): "flash 失效化失败 —— 恢复出厂未生效 (RAM 与 flash 都保持旧值, "
                  "回本码才是真话)",

    # ---- 0x26 / 0x27 / 0x28 / 0x2B / 0x2C / 0x31 前馈与动力学旋钮 ----
    (0x26, 0x01): "载荷长度不足 (需 1 + 4×DOF 字节)",
    (0x26, 0x02): "item 越界 (1..15), 或值非法 (NaN / item>=9 的符号契约不符: "
                  "fv>=0, fc0>=0, fc1<=0)",
    # ⚠ 下面两条**只存在于 `origin/feat/hyy-model-import`**（1.5.3 `cdb744a` 不是
    #   master 祖先）: master 上这两个 case 没有武装门禁, 故永不发 0x04。
    #   刻意保留位置 (spec R5), 免得产线烧那条分支时解读落回通用档。
    (0x26, 0x04): "[仅 `feat/hyy-model-import` 分支] item 7 (gravity_scale) 武装态拒 "
                  "—— 须先失能 (写它会当拍阶跃重力项)",
    (0x27, 0x01): "载荷长度不足 (需 4 字节: ff_mask)",
    (0x28, 0x01): "载荷长度不足 (需 6 字节: item + sub + f32)",
    # ⚠ 本条曾写成 "值非法 (… / payload_mass<=0)" —— 那个 "payload_mass<=0" **不是**固件的
    #   拒绝条件, 已删。全树没有任何路径因质量为负而回本码: `params.c:184` 对 item 4 走
    #   `ff_clamp(v, 0.0f, 20.0f)` **静默钳制**, 随后 `return true` ⇒ 回的是 ACK。
    #   会回本码的**只有四类**, 全在 `params_ff_scalar()` (`params.c:170-232`):
    #     NaN (`:172`) / item 5/6 的 `sub > 2` (`:188` / `:193`) /
    #     item 7 的 `v ∉ {0,1,2}` (`:199`) / 其余 item 落 `default` (`:219-220`)
    #   ⚠ "幅值静默钳制"那句的**射程**只覆盖"**有 clamp 的** item" —— item 7 与 item 5/6
    #   的 `sub` 是**值域拒绝** (在**任何** `ff_clamp()` 之前就 `return false`), 不是
    #   幅值越界。判据 (地面真值现解 `params.c`):
    #   `test_error_codes.py::test_0x28_02_text_scopes_the_clamp_claim_to_items_that_actually_clamp`
    (0x28, 0x02): "item 越界 (合法区间是 1..8 与 10..18; **item 9 已保留**给 0x2C 读 "
                  "ff_mask, 写它走 `default` 同样回本码), 或值非法 —— 只有三类: **NaN**、"
                  "item 5/6 的 `sub>2`、item 7 的 `v∉{0,1,2}`。"
                  "⚠ **幅值一律静默钳制**只对**有 `clamp` 的那些 item** 成立: 它们过 "
                  "`ff_clamp()` 后照样 `return true` 回 ACK (`params.c:232`), 固件"
                  "**没有**「幅值越界 ⇒ ERR」这条路径。"
                  "⚠ 但 **item 7 与 item 5/6 的 `sub`** 是**值域拒绝** (回本码), **不是**"
                  "幅值越界 —— 它们在**任何** `ff_clamp()` 之前就 `return false` "
                  "(`params.c:199` / `:188` / `:193`; 固件自查头也写「其余拒绝」 "
                  "`usb_cmd.h:83`), 正是上面「只有三类」里的后两类, **不是**第四类。"
                  "例: `payload_mass` 钳 [0,20] (`params.c:184`) ⇒ `set_payload(-5)` "
                  "**回 ACK 并把质量静默钳成 0** (重力前馈随之改变), **不是**被拒",
    (0x28, 0x04): "[仅 `feat/hyy-model-import` 分支] item 6 (gravity 向量) 武装态拒 "
                  "—— 须先失能 (正装↔侧装 |Δg| = 13.87 m/s² = 1.41g)",
    (0x2B, 0x01): "载荷长度不足 (需 1 字节: item)",
    (0x2B, 0x02): "item 越界 (须 1..15)",
    (0x2C, 0x01): "载荷长度不足 (需 2 字节: item + sub)",
    (0x2C, 0x02): "item 越界 (须 1..18)",
    (0x31, 0x02): "preset 非法 (须 0..2), **或**载荷长度不足 (两者共用本码)",

    # ---- 0x30 / 0x32 / 0x33 / 0x34 / 0x37 / 0x39 HYY 动力学模型在线导入 ----
    (0x30, 0x01): "载荷长度不足 (需 1 + 4×BODY_PARAMS 字节)",
    (0x30, 0x02): "body_idx 越界, 或 staging 失败 (数值非法)",
    (0x32, 0x01): "载荷长度不足 (需 2 字节: expected_mask)",
    (0x32, 0x04): "武装中 —— 须先失能 (门控**排在**掩码校验之前)",
    (0x32, 0x07): "**模型掩码不符** —— 产线最常见故障, 与「数值非法」完全不同",
    (0x32, 0x02): "dyn_model_commit 失败",
    (0x33, 0x01): "载荷长度不足 (需 4×JM_N 字节)",
    (0x33, 0x02): "staging 失败 (数值非法)",
    (0x34, 0x01): "载荷长度不足 (需 1 字节: body_idx)",
    (0x34, 0x02): "body_idx 越界",
    (0x37, 0x04): "武装中 —— 须先失能",
    (0x39, 0x01): "载荷长度不足 (需 4×DOF 字节: q[7])",

    # ---- 0x2D / 0x2E 300Hz 控制拍采集 ----
    (0x2D, 0x01): "载荷长度不足 (需 4 字节: n_ticks)",
    (0x2D, 0x02): "超容量 —— 请求的拍数超过 LOG_MAX_SAMPLES",
    (0x2E, 0x01): "载荷长度不足 (需 4 字节: offset)",
}

#: 通用档: `code -> 语义`（跨命令）。具体档未命中时用。
#: ⚠ 通用档**不是**"可以随便猜"的意思 —— 每个 code 在固件里都有稳定的骨架含义
#: （长度 / 非法 / 未使能 / 门禁 / 忙 / 锁存 / 掩码), 只是**具体是哪一种被拒**要看命令。
ERR_CODE_TEXT = {
    0x00: "固件没有实现这条命令 (default 分支, 无任何副作用)",
    0x01: "载荷长度不足",
    0x02: "字段非法 (非有限值 / 越界 / idx 或 item 非法)",
    0x03: "被拒 —— 逐命令而异 (常见: 未使能 / EMERGENCY 锁存 / 新限位关不住在途目标)",
    0x04: "被拒 —— 逐命令而异 (常见: 须先失能 / 零重力中 / 状态机不允许)",
    0x05: "忙 —— 通道被占或参数保存中, 稍后重试",
    0x06: "锁存 —— 须先 reset / clear_faults",
    0x07: "掩码不符, 或 CMODE 补写预算耗尽 (重发无用)",
}


def err_reason(cmd: int, code: int) -> str:
    """`(cmd, code)` -> 可读语义。**未登记的码不静默**。

    三级查找 (照 :func:`litearm.cart.raise_for_plan` 的口径: 认不出也要把原始码
    带出来, 不许吞):

    1. 具体档 :data:`ERR_TEXT` 命中 -> 用它;
    2. 否则通用档 :data:`ERR_CODE_TEXT` 命中 -> 用它, **并附上原始码**;
    3. 都没有 -> 明说"未登记", **带上原始 `cmd`/`code`**。

    第 2/3 档之所以必须带原始码: 固件新增一档错误码时, 旧 SDK 的这条路径就是**唯一**
    会让上位机看见"这是我没见过的码"的地方 —— 折叠成一句通用文本等于把它藏起来。

    ⚠ **本函数与 :data:`ERR_TEXT` / :data:`ERR_CODE_TEXT` / :data:`ENABLE_RETRYABLE_CODES`
    都【不在】包级公开面** —— 它们不在 `litearm.__all__` 里,
    `from litearm import err_reason` 会 `ImportError`。要它们请写全限定::

        from litearm.errors import err_reason, ERR_TEXT, ENABLE_RETRYABLE_CODES

    (spec/plan 都没点名这三个名字 ⇒ 刻意**不扩 `__all__`**; 扩了要连带 `__all__`
    哨兵与文档一起改。)
    """
    hit = ERR_TEXT.get((int(cmd), int(code)))
    if hit is not None:
        return hit
    gen = ERR_CODE_TEXT.get(int(code))
    if gen is not None:
        return f"{gen} [本命令未登记此码: cmd=0x{int(cmd):02X}, code=0x{int(code):02X}]"
    return (f"**未登记的固件错误码** cmd=0x{int(cmd):02X}, code=0x{int(code):02X} "
            f"—— 子版本可能比本 SDK 新, 请对照固件 usb_cmd.c 的 usb_cmd_reply(RSP_ERR…)")


#: `CMD_ENABLE (0x10)` 的返回码里**值得重试**的那些 —— `Arm.enable()` 的重试白名单。
#:
#: ⚠ 判据是**白名单**（"只有它说可重试才重试"）而不是"黑名单"（"除了锁存都重试"）,
#:   理由是固件对这几个码的注释是**并列**的, 而 `0x07` 与 `0x00` 也都明确是"重发无用":
#:
#:   | code | 固件原注释 | 出处 | 该重试吗 |
#:   |---|---|---|---|
#:   | `0x03` | "可重试 (反馈未齐 / CMODE 首写…重发即可)" | `control_api.h:123-128` | **是** |
#:   | `0x06` | "锁存须先 reset (EMERGENCY/joint_fault)" | 同上 | 否 |
#:   | `0x07` | "部分轴 CMODE 补写预算耗尽仍未进 MIT —— **重发无用**, 须现场排查" | 同上 | 否 |
#:   | `0x00` | 固件没有这条命令 (`default` 分支) | `usb_cmd.c` | 否 |
#:
#:   黑名单写法 ("除 `0x06` 外都重试") 会让 `0x07` 与 `0x00` 各自白耗 **3.3 秒** ——
#:   与 `0x06` 是同一类浪费, 且 `0x00` 那一档白白把
#:   :class:`UnsupportedByFirmwareError` ("固件太旧") 的判定推迟 **3.3 秒**。
#:   白名单对**将来新增的码**也 fail-fast 且不静默 (异常里带着原始码)。
ENABLE_RETRYABLE_CODES = frozenset({0x03})


# ── 跨线错误（原住在 litearm-server）───────────────────────────────────────
# ⚠ 这四个原先都不是 LiteArmError 子类 ⇒ 客户端 `except LiteArmError` 抓不到
#   "已被遥操占用"。收进本模块后全局只有一份定义。

class NotRemoteable(LiteArmError):
    """该入口不可跨线：未注册 / 返回活对象 / 返回不可序列化类型。

    ⚠ 与"固件拒绝"刻意分开：这不是设备出错，**不要重试**。
    """

class NotSupportedOnThisBackend(LiteArmError):
    """该能力在当前后端上没有实现（不是因为调用方式不对）。"""

class TeleopLockedError(LiteArmError):
    """遥操态下拒绝手动控制。"""

class TeleopBusyError(LiteArmError):
    """遥操正在切换中（enter/exit 未完成）。"""
