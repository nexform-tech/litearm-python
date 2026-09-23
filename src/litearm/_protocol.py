"""帧协议层 —— 与 litearm-stm32 固件 / tools/arm_console.py 逐字节一致。

帧: SOF(0xA5) CMD(1B) LEN(1B) PAYLOAD(0..255) CRC16_LO CRC16_HI
CRC16-CCITT-FALSE, 覆盖 [SOF..PAYLOAD]。
上行 100Hz 状态帧: 6+21N (固件 >=1.5.0, 尾部 joint_fault u16) 或 4+21N (<=1.4.x, 仅兼容解析);
应答/异步返回穿插。
"""
from __future__ import annotations

import binascii
import re
import struct
from typing import List, Optional, Tuple

SOF = 0xA5

# 下行 CMD (固件 hal/usb_cmd.h)
CMD_MOVE_J = 0x01
CMD_MOVE_P = 0x02
CMD_MOVE_JS = 0x03
CMD_MOVE_MIT = 0x04
CMD_MOVE_MIT_ALL = 0x05
# [段②] 零重力(拖动示教) 进入/退出: payload [u8] 1=进 0=退 -> ACK
# ⚠ 固件侧自带 watchdog_kick, 重发即保活 —— 上位机需周期重发(建议 <=50ms),
#    否则 0.1s 后看门狗把模式掐回 fail-soft 持位
CMD_ZERO_G = 0x06
#: [SYNC] 同步 PTP movej: 全关节**同时到达**、末端沿**关节空间直线** q(s) = q0 + s·Δq。
#: 载荷与 `CMD_MOVE_J` **逐字节相同**; 语义差异只在轨迹形状 —— 0x01 每轴一条独立 S 曲线
#: (先到先停、末端轨迹不可预测), 本命令用**一条**路径标量 s: 0->1 的 S 曲线驱动全轴。
#: ⚠ 代价: 被最慢的轴拖住, 整体比 0x01 慢 —— 故是**可选模式**, 不是替换。
#: ⚠ 固件状态帧仍报 mode=MOVE_J, **无法**从状态区分同步/异步。
CMD_MOVE_J_SYNC = 0x07
CMD_ENABLE = 0x10
CMD_DISABLE = 0x11
CMD_EMERGENCY_STOP = 0x12
CMD_CLEAR_FAULTS = 0x13
CMD_RESET = 0x14
# [DFU] 免探针进 ROM 系统 bootloader (**终端态**操作): 空载荷 -> ACK 表示"已登记",
# **不表示**"会跳"/"已经跳" (真跳转在 main 线程, 上界 100ms)。
# 非空载荷 -> ERR{0x15,0x01}; enabled||enable_pending -> ERR{0x15,0x03};
# ROM 向量表无效 -> ERR{0x15,0x02}。入口 = `Arm.enter_dfu()` (两段式, 见那里)。
CMD_ENTER_DFU = 0x15
CMD_SET_MOTION_MODE = 0x20
CMD_SET_SPEED_PERCENT = 0x21
CMD_SET_JOINT_PARAM = 0x22
CMD_SET_JOINT_LIMITS = 0x23
CMD_GET_JOINT_PARAM = 0x24
CMD_PARAM_SAVE = 0x25
# [P1-C] 恢复出厂默认 + 失效 flash (空载荷; 须失能, 否则 ERR{0x36,0x04})
CMD_PARAM_RESET = 0x36
# [HOME] 回零位舒展姿 (空载荷): 目标=URDF 零位 0, 固件侧速度写死 0.10;
# 注释明确「不受当前越软限/贴端限制」(软限位 clamp 作用于目标, 零位在限内)。
# 未使能 -> ERR{0x2A,0x03}
CMD_HOME = 0x2A
CMD_SET_FF_VEC = 0x26        # item(1B)+f32[7]: 1 friction/2 ki/3 i_max/4-6 wall/7 gs/8 is
                             #   9-11 摩擦 v2 (fv/fc0/fc1) / 12-14 零重力 (zg_kp/zg_kd/zg_damping)
                             #   15 kd_extra
                             #
                             # [item 12~15 语义 · 固件 段②/段③] 真源 = 固件 litearm.h:189,217-219
                             #   12 zg_kp      松手保持的虚拟弹簧刚度 Nm/rad       钳 [0, 500]
                             #   13 zg_kd      MIT 速度阻尼 (固件会抬到下限以上)  钳 [0, 5]
                             #   14 zg_damping 附加速度阻尼 Nm·s/rad, 出厂 0      钳 [0, 50]
                             #   15 kd_extra   τ 域软件微分阻尼: τ += kd_extra·(dq_d − dq_meas),
                             #                 治 movej 起步振铃; 出厂 [6,6,6,6,0,0,0] ——
                             #                 **腕部 J5-J7 必须留 0** (J7≈2e-4 kg·m²,
                             #                 kd_extra/J ≈ 4e4 /s 远快于 8ms 环, 延迟阻尼
                             #                 会反向激发)。            钳 [0, 50], 须 >= 0
                             # 12~14 属**零重力拖动示教**参数 (`Arm.zero_g` 会话);
                             # 共通: 任一分量 NaN 整组拒绝 (ERR{0x26,0x02}), 幅值超限静默钳制。
                             # ⚠ SDK 现状: `arm.FF_VEC_ITEMS` 一直按 **1..15** 收, 故 12~15 用
                             #   通用 `set_ff_vec(item, vals)` **能写**、`get_ff_vec(item)` **能读回**;
                             #   **没有具名入口** (不像 item 7/8 有 `set_gravity_scale()`/
                             #   `set_inertia_scale()`) —— 是否给具名方法留待后续。
                             # ⚠ 固件头 `usb_cmd.h:73-78` 的 0x26 注释**只写到 11**, 是固件侧
                             #   注释未随 [段②/段③] 更新 (实现见 params.c:112 守卫 + :147 item15
                             #   + :159-161 item12-14): 别拿它当能力边界, 也**不要**据此把 SDK
                             #   收窄成 1..11。
                             # ⚠ `0x28` 的 `item 18 hold_kp_gain` (到位增刚倍数) 与 `kd_extra`
                             #   是**两回事**, 别混 (旧注释把两者写在一块, 已拆开)。
CMD_SET_FF_FLAGS = 0x27      # u32 ff_mask LE
CMD_SET_FF_SCALAR = 0x28     # item+sub+f32: 1 fric_db/2 margin/3 slew/4 payload_mass/5 payload_com/6 gravity
                             #   7 friction_model (0/1/2) / 8 fric_v2_eps
                             #   10-17 零重力 (drag_gain/drag_db/drag_kd_margin/vel_thr/
                             #            engage_sec/engage_kp/engage_kd/wall_fw_kd)
                             #   18 hold_kp_gain (§6.2). item 9 保留给 0x2C 的 ff_mask 读回
CMD_FF_PRESET = 0x31         # u8: 0 全关 / 1 出厂(G+惯量+科氏+摩擦+积分) / 2 全开
# [readback 0x2B/0x2C] 参数读回 (镜像写口, 纯读)
CMD_GET_FF_VEC = 0x2B        # item(1B) -> RSP_FF_VEC: [0x4B, item, 7×f32]; item 1..15
CMD_GET_FF_SCALAR = 0x2C     # item(1B)+sub(1B) -> RSP_FF_SCALAR: [0x4C, item, sub, f32]
                             #   item 1..8 + 10..18, **item 9 = ff_mask (只读)**
# [P1 傅里叶辨识采集] 300Hz 控制拍记录/读回 (固件 hal/log_capture.*)
CMD_LOG_CTRL = 0x2D          # u32 n_ticks LE (0=停/清; 记满自停) -> ACK
CMD_SET_MODEL_PARAM = 0x30   # body_idx(1B)+f32[10] -> staging (不生效)
CMD_MODEL_COMMIT = 0x32      # u16 expected_mask LE -> staging 整组原子生效 (须失能)
CMD_SET_MODEL_JM = 0x33      # f32[7] -> staging 的 jm (原名 CMD_SET_MODEL_PARAM2, 已语义化改名)
CMD_GET_MODEL_PARAM = 0x34   # body_idx(1B) -> RSP_MODEL_PARAM (读 bank; 也是能力探测口)
CMD_GET_MODEL_JM = 0x35      # (空) -> RSP_MODEL_JM
CMD_REVERT_MODEL = 0x37      # (空) -> ACK: 只回退模型, 不动 flash (须失能)
CMD_GET_MODEL_STATUS = 0x38  # (空) -> RSP_MODEL_STATUS
CMD_GET_GRAVITY = 0x39       # q[7]f32 -> RSP_GRAVITY: G(q)[7] (纯读)
CMD_LOG_READ = 0x2E          # u32 offset_byte LE -> RSP_LOG_DATA(0x4D)
# ⚠ 双 ID: 下行 CMD_KIN_BENCH 与上行 RSP_JOINT_PARAM 同为 0x49 (固件 usb_cmd.h)。
#   收发方向不同, 故靠方向区分, 不可用单张 ID 表查。
CMD_KIN_BENCH = 0x49         # 运动学开销自测 (DWT cycles) -> RSP_KIN_BENCH(0x4A) 文本
CMD_GET_STATUS = 0x40
CMD_GET_FIRMWARE = 0x41
CMD_GET_IK = 0x42
CMD_GET_TCP = 0x43

# ---- [授权/激活] 固件 `license.c` + `usb_cmd.c` 的 license 分支 (固件 1.8.0+) ----
# ⚠ **本包只做"查询 + 提交凭据", 不含任何算 MAC 的代码** —— 这是规格的硬要求
#   (`litearm-stm32` 的 activation 设计 §3.3): 持密钥的签发件绝不得进客户侧交付物,
#   客户侧只要有一份能算 MAC 的代码, 这套机制就归零。故本文件里**没有** SipHash。
#   签发在厂商侧工具 (`litearm-stm32/tools/litearm_license/sign.py`), 不进本包。
CMD_GET_LICENSE = 0x2F       # 无载荷 -> RSP_LICENSE(0x4F) 26B
# ⚠ 请求**不校验长度** (固件 `usb_cmd.c:1016` 一行都没碰 `len`) —— 这是**规格层面的有意
#   放宽**, 不是漏判: 给一条纯只读命令加 `ERR{0x2F,0x01}` 只会多一种没有安全收益的失败模式。
CMD_ACTIVATE = 0x3F          # 28B -> ACK{0x3F} / ERR{0x3F,code}
# ⚠ 28B = cust_id u32 LE + issued u32 LE + flags u32 LE + mac[16] (两个 SipHash-2-4 标签)。
#   长度判据是 `len < 28` ⇒ **更长的载荷会被接受**, 尾部多余字节不进 MAC、不进记录、
#   不回显 (固件 `usb_cmd.h:229-232` 记为已记录的取舍)。

# ---- [笛卡尔运控] 固件原生规划 (受固件 `#if LITEARM_CART_PLAN` 编译开关约束) ----
# 关掉开关时这 5 条**整段不在固件里**, 会落到 `usb_cmd.c` 的 `default` 分支 ->
# `ERR{cmd,0x00}`。故 SDK 必须**探测**而不是假定 (见 `cart.probe`)。
# ⚠ `0x3A/0x3B/0x3C/0x3D` 的**长度校验排在门禁与一切副作用之前**
#   (`usb_cmd.c:323` / `:336` / `:352` / `:415`; `cart_gate_ok` 在 `:331` / `:347`,
#    两条不走门禁的 `ctrl_cart_hold_begin()` 在 `:410` / `:436`), 所以"发空载荷"不会让臂动
#    —— 这**四条**可以当能力探针 (见 `cart.probe`)。
# ⚠ **`0x3E (CART_RUN)` 是例外**: 它**没有长度校验、空载荷就是它的合法载荷**
#   (`usb_cmd.c:440-445` 直接 `cart_gate_ok(CMD_CART_RUN)` -> `cart_req_run()`)。
#   而 `cart_gate_ok` 的门禁有**实打实的副作用** (`cart_limits_set` / `cart_q_start_fill` /
#   `ctrl_cart_hold_begin()`, 见 `usb_cmd.c:192-215`), 所以发一条空载荷 `0x3E` 会在
#   `cart_req_run()` 之前**先把副作用执行掉** (作废在途轨迹、置 mode、kick 看门狗)。
#   此后分两种情形: **臂正处在 `RECV` 且点名已满** -> `cart_req_run()` 返回 OK, 这一条
#   **真的跑起来** (空载荷本就是它的合法载荷); 否则 (不在 `RECV` / 点名不满) 才落
#   `ERR{0x3E,0x04}`。两种情形副作用都已发生 —— 别拿它当探针。
CMD_MOVE_L = 0x3A            # pose[6]f32 + sp f32 (28B): 笛卡尔直线
CMD_MOVE_C = 0x3B            # via[6] + end[6] + sp (52B): 圆弧 (via 只取位置三分量)
CMD_CART_BEGIN = 0x3C        # n u8 + sp f32 (5B, n ∈ [1,32]): 进 RECV 收集态
CMD_CART_ADD = 0x3D          # idx u8 + pose[6] (25B): 逐点追加 (重复 idx 拒 0x04)
CMD_CART_RUN = 0x3E          # 空载荷: 提交并开始规划 (点名不齐 -> 0x04)

# 上行
RSP_STATUS = 0x40
# ⚠ 死登记 (本轮只加注释,**不删常量**)。两条事实:
#   ① 固件**至今仍定义** `RSP_DETAIL 0x41` (`litearm-stm32/User/litearm/hal/usb_cmd.h:215`,
#      注释还写着"温度/错误详情 10Hz"), 但**发送方已删** —— `grep -c RSP_DETAIL usb_cmd.c`
#      = 0, 整个 `User/` 只剩那一行 `#define`。故 SDK 侧这个常量**永远不会被收到**。
#   ② 与下行 `CMD_GET_FIRMWARE 0x41` **撞号** —— 同 `0x49` (CMD_KIN_BENCH / RSP_JOINT_PARAM)
#      那类"双 ID": 收发方向不同, 靠方向区分, 不可用单张 ID 表查。
#
# ⚠ **为什么保留而不删**: `test_protocol_sync.py::test_every_firmware_rsp_has_sdk_constant`
#   按**名字**遍历固件头的 `RSP_*`, 固件头里那一行还在 ⇒ **删 SDK 常量会让段三验收立刻红**;
#   而且 0x41 属**固件仓**的决定, 本轮是纯 SDK/文档侧 (spec §8)。
# ⚠ **别说成"没有别的豁免路径"**: 机制上当然可以再往 `FIRMWARE_ONLY_RSPS` 添一项来豁免它;
#   **不这么做是设计选择** —— 那张表的语义是"用户裁决 SDK **有意不暴露**" ("不做", 不是
#   "还没做"), 把一条**没人发**的应答塞进去语义不符 (同理见 `PREEXISTING_GAPS` 的注释)。
# ⚠ **真要删这个死 ID**: 得先在**固件侧**删掉 `usb_cmd.h:215` 那行 `#define` —— 属另一笔。
RSP_DETAIL = 0x41
RSP_FIRMWARE = 0x44
RSP_ACK = 0x45
RSP_ERR = 0x46
RSP_IK = 0x47
RSP_TCP = 0x48
RSP_JOINT_PARAM = 0x49
RSP_KIN_BENCH = 0x4A         # CMD_KIN_BENCH(0x49) 应答: 性能自检文本 (含诊断计数)
RSP_FF_VEC = 0x4B            # 应答 CMD_GET_FF_VEC, payload 首字节冗余回填本 id
RSP_FF_SCALAR = 0x4C         # 应答 CMD_GET_FF_SCALAR, 同上
RSP_LOG_DATA = 0x4D          # 应答 CMD_LOG_READ(0x2E): [u32 total][u32 next][u8 n][n bytes]
#: [笛卡尔] 规划结果: `ok u8 + err u8 + n_wp u16 LE + plan_us u32 LE` = 8B。
#: ⚠ 载荷里**没有命令 id** —— 只能按**受理顺序** FIFO 配对 (见 `cart.py`)。
#: ⚠ 规划**失败也必须发这一条** (只回 ACK 的话上位机会以为臂在动, 其实一步没动)。
RSP_CART_PLAN = 0x4E

#: [授权] `CMD_GET_LICENSE(0x2F)` 的应答 —— **26B**:
#: `[state u8][ver u8][uid 12B 原始寄存器内存序][cust_id u32 LE][issued u32 LE][flags u32 LE]`。
#: ⚠ **载荷首字节是 `state`(0/1/2), 不是命令号** ⇒ 它**不在** `_read_key` 的回显归属那一类
#:   (那一类只有 `RSP_ACK`/`RSP_ERR`), 队列键是 `(0x4F, None)`, 与 `0x2F` 一一对应。
#: ⚠ **未激活时 `cust_id`/`issued`/`flags` 全 0**, 但 **UID 照回** —— 签发器**必须**从本应答
#:   取那 12 字节, 不得经 USB 序列号字符串 (两者不是一个东西)。
RSP_LICENSE = 0x4F
#: [授权] **本版无生产者** —— 激活成功回的是 `ACK{0x3F}`(见 `CMD_ACTIVATE`),
#: 故固件里**没有任何代码会发出** 0x50; 该号只被占住。
#: ⚠ 保留定义是为了让 `test_protocol_sync` 的按名比对自然通过 —— **不要**给它造生产者,
#:   也不要塞进 `FIRMWARE_ONLY_RSPS` (那张表的语义是"SDK 有意不暴露", 与"固件没有生产者"不同)。
RSP_ACTIVATE = 0x50

RSP_MODEL_PARAM = 0x54       # [0x54, body_idx, f32[10]] = 42B
RSP_MODEL_JM = 0x55          # [0x55, f32[7]] = 29B
RSP_MODEL_STATUS = 0x56      # [0x56, override u8, staged_mask u16 LE, dirty u8] = 5B
RSP_GRAVITY = 0x57           # [0x57, G(q)[7]] = 29B

# ---- [2026-09-14] 动力学模型在线导入 (0x30/0x32/0x33/0x34/0x35/0x37/0x38/0x39) ----
# 固件侧三层语义 (bank/staging/生效层) 与完整注释见 litearm-stm32 的
# User/litearm/hal/usb_cmd.h; SDK 封装见 litearm/model.py。
# ⚠ 常量名必须与固件头**逐字相同** —— test_protocol_sync 按名字双向比对。

# ff_mask 位 (固件 litearm.h)
FF_MASTER = 0x0001
FF_G = 0x0002
FF_INERTIA = 0x0004
FF_CORIOLIS = 0x0008
FF_FRICTION = 0x0010
FF_INTEGRAL = 0x0020
FF_WALL = 0x0040
FF_QUANT = 0x0080
FF_VELREF = 0x0100
FF_ALL = 0x01FF

#: 固件 IK 的 seed 恒为**模型 7 轴** (`KIN_N`), 与 `LITEARM_NUM_JOINTS` 解耦
#: (usb_cmd.c CMD_GET_IK: "seed 恒为模型 7 轴 (KIN_N), 与 LITEARM_NUM_JOINTS 解耦")。
#: 台架 1J 版把台架那台电机映射到模型第 `BENCH_MODEL_AXIS` 轴。
KIN_N = 7
#: 台架电机对应的整臂模型轴 (`joint_cfg.h` 的 `LITEARM_BENCH_MODEL_AXIS`, 台架=5)。
#: ⚠ 跨仓常量 —— 由 tests/test_protocol_sync.py 直接解析固件 joint_cfg.h 断言其一致。
BENCH_MODEL_AXIS = 5

MODE_ZERO_G = 7
MODE_NAMES = {0: "INIT", 1: "MOVE_J", 2: "MOVE_P", 3: "MOVE_JS",
              4: "MOVE_MIT", 5: "MIT_ALL", 6: "EMERGENCY", 7: "ZERO_G"}
FLAG_NAMES = {0: "FAULT", 1: "WD_TRIPPED", 2: "FB_STALE",
              3: "TEMP_WARN", 4: "POS_VIOL", 5: "OVERSPEED"}
#: flags bit9 = 使能位 (固件 1.5.0 起; 低 6 位才是安全 flag, 6..8 是 mode)
FLAG_ENABLED_BIT = 9
#: joint_fault 位图 (u16) 能表达的关节数上限
MAX_JOINTS = 16


#: 开机签名 (固件 usb_cmd.c 的 sig_ok/sig_iwdg), IWDG 复位多带 ", iwdg-rst"
_BANNER_RE = re.compile(r"\[litearm-usbcdc\]\s+ready\s*\(([^)]*)\)")


def parse_boot_banner(text: str) -> Optional[str]:
    """从串口噪声文本里解析开机签名 -> "normal" | "iwdg-rst"; 没有则 None。

    固件唯一能区分「独立看门狗复位过」的地方 (banner 带 `, iwdg-rst`)。重连/多次
    枚举会重复出现, 取最后一次。
    """
    hits = _BANNER_RE.findall(text or "")
    if not hits:
        return None
    return "iwdg-rst" if "iwdg-rst" in hits[-1] else "normal"


#: 「协议无死角」契约: 固件每条**已实现**的下行命令 -> SDK 上的一等方法入口。
#: 由 `tests/test_protocol_sync.py` 强制: 直接解析固件 `hal/usb_cmd.h`, 双向比对
#: 命令集合与 ID, 并逐条断言这里的入口在 `Arm` 上真的可解析到。少一条命令就红。
#: **请求 → 它的那条 `RSP_*` 应答**（只列"应答不是 `ACK`/`ERR`"的命令）。
#: ⚠ **名字刻意不叫 `CMD_*`**：`tests/test_protocol_sync.py` 把所有 `CMD_` 前缀的
#: 模块级名字当成**命令码常量**去和固件头对表，叫 `CMD_TO_RSP` 会被判成
#: "SDK 声明了固件没有的命令"（实测踩过）。
#:
#: 唯一的用途是 `Arm._raw_write` 在**发出前**清掉本命令的应答队列（见那里）——
#: `ACK`/`ERR` 两条由命令码自己就能推出来，故**不在这张表里**。
#: ⚠ `0x40` (`GET_STATUS`) 也不在：它的应答是 `RSP_STATUS`，走的是**单槽**不是队列。
#: ⚠ 与固件的对应关系逐条抄自本文件上面各 `CMD_*` 的注释；改固件时这里要跟着改
#: （`tests/test_protocol_sync.py` 会盯 `CMD_*`/`RSP_*` 常量本身，这张表**没有**自动判据）。
RSP_OF_CMD = {
    0x24: 0x49,      # GET_JOINT_PARAM -> RSP_JOINT_PARAM
    0x2B: 0x4B,      # GET_FF_VEC      -> RSP_FF_VEC
    0x2C: 0x4C,      # GET_FF_SCALAR   -> RSP_FF_SCALAR
    0x2E: 0x4D,      # LOG_READ        -> RSP_LOG_DATA
    0x34: 0x54,      # GET_MODEL_PARAM -> RSP_MODEL_PARAM
    0x35: 0x55,      # GET_MODEL_JM    -> RSP_MODEL_JM
    0x38: 0x56,      # GET_MODEL_STATUS-> RSP_MODEL_STATUS
    0x39: 0x57,      # GET_GRAVITY     -> RSP_GRAVITY
    0x41: 0x44,      # GET_FIRMWARE    -> RSP_FIRMWARE
    0x42: 0x47,      # GET_IK          -> RSP_IK
    0x43: 0x48,      # GET_TCP         -> RSP_TCP
    0x49: 0x4A,      # KIN_BENCH       -> RSP_KIN_BENCH
}


COMMAND_COVERAGE = {
    0x01: "Arm.movej",
    0x07: "Arm.movej_sync",
    0x02: "Arm.move_p",
    0x03: "Arm.move_js",
    0x04: "Arm.send_mit",
    0x05: "Arm.send_mit_all",
    0x06: "Arm.zero_g",
    0x10: "Arm.enable",
    0x11: "Arm.disable",
    0x12: "Arm.emergency_stop",
    0x13: "Arm.clear_faults",
    0x14: "Arm.reset",
    0x15: "Arm.enter_dfu",
    0x20: "Arm.set_motion_mode",
    0x21: "Arm.set_speed",
    0x22: "Arm.params.set_joint_param",
    0x23: "Arm.params.set_joint_limits",
    0x24: "Arm.params.get_joint_param",
    0x25: "Arm.save_params",
    0x26: "Arm.set_ff_vec",
    0x27: "Arm.set_ff_mask",
    0x28: "Arm.set_ff_scalar",
    0x2A: "Arm.home",
    0x2B: "Arm.get_ff_vec",
    0x2F: "Arm.license",
    0x3F: "Arm.activate",
    0x2C: "Arm.get_ff_scalar",
    0x2D: "Arm.log.start",
    0x2E: "Arm.log.reader",
    0x30: "Arm.model.set_body",
    0x31: "Arm.ff_preset",
    0x32: "Arm.model.commit",
    0x33: "Arm.model.set_jm",
    0x34: "Arm.model.get_body",
    0x35: "Arm.model.get_jm",
    0x37: "Arm.model.revert",
    0x38: "Arm.model.status",
    0x39: "Arm.model.get_gravity",
    0x36: "Arm.params.reset_factory",
    0x40: "Arm.get_status_now",
    0x41: "Arm.connect",
    0x42: "Arm.ik",
    0x43: "Arm.get_tcp",
    0x49: "Arm.diag.kin_bench",
    # [笛卡尔] 0x3C/0x3D/0x3E 是同一条入口的三个阶段 (BEGIN -> ADD×n -> RUN):
    # 三者的帧都由 `Arm.move_path` 发出, 故三条都指向它。
    0x3A: "Arm.move_l",
    0x3B: "Arm.move_c",
    0x3C: "Arm.move_path",
    0x3D: "Arm.move_path",
    0x3E: "Arm.move_path",
}

#: 固件注释即「未实现」的命令 —— SDK 有意不提供入口。
#: ⚠ 不是永久豁免: 同步测试会回头核对固件头文件里这些命令**仍然**标注未实现,
#: 一旦固件实现它们, 测试立刻失败提醒补入口。
#:
#: [2026-09-14] 0x30 / 0x33 已由「动力学模型在线导入」实现 (见 model.py),
#: 故本表**清空**。留空是正常的 —— 它现在是"当前没有未实现命令"的断言载体。
UNIMPLEMENTED_CMDS: dict = {}


# ---------------------------------------------------------------------------
# [笛卡尔运控] 固件单边命令 —— 显式豁免表（**现已清空**）
#
# 历史: 用户 2026-09-13 一度裁决 SDK **不暴露**笛卡尔运控入口（spec §7.1 选项 A），
#   故 `0x3A~0x3E` / `0x4E` 这 6 条曾是"固件有、SDK 无"，在这里显式豁免，
#   免得 test_protocol_sync.py 因双向比对而长期红着（哨兵红久了就失去意义）。
#
# [段二] 该裁决已被"固件原生笛卡尔"取代: 5 条命令与那条应答现在都有 SDK 入口与常量
#   (`COMMAND_COVERAGE` 的 0x3A/0x3B/0x3C/0x3D/0x3E / 本节上方的 `RSP_CART_PLAN`),
#   于是两张豁免表**同时清空**。留空是有意的 —— 与 `UNIMPLEMENTED_CMDS` 同一个用法:
#   它现在是"当前没有任何固件单边命令"这句话的断言载体, 而不是可以顺手塞东西的地方。
#
# ⚠ 与 `test_protocol_sync.py` 的耦合: `test_firmware_only_exemption_is_still_valid`
#   逐个断言"豁免表里的名字在固件头文件里仍然存在、且 SDK **还没有**同名常量"。
#   往这里加条目等于宣布"SDK 有意不做这条命令", 加之前先想清楚是不是真的不做。
# ⚠ **不要**把"已知缺口"塞进来: 那是 `PREEXISTING_GAPS` 的语义（本应同步而未同步）,
#   两者的理由不同, 混在一起会让读的人以为缺口是有意为之。
# ---------------------------------------------------------------------------

#: 固件单边下行命令: 码值 -> 名字（SDK 不提供入口）。当前为空。
FIRMWARE_ONLY_CMDS: dict = {}

#: 固件单边上行应答: 码值 -> 名字。当前为空。
FIRMWARE_ONLY_RSPS: dict = {}

# ---------------------------------------------------------------------------
# 【既有的固件/SDK 不同步 —— **只豁免, 不修复**】（**现已清空**）
#
# 历史: 固件自 `b6cbc33`（DFU 免探针烧录）起有 `CMD_ENTER_DFU = 0x15`, 而 SDK 侧
#   `grep -ri dfu src/ tests/` **命中 0**。当时**豁免而非修复**的理由三条:
#   ① 它不是那一笔的产物（修它等于把另一个特性的决定塞进去）;
#   ② 长期红着会让**真正的**回归看不见（哨兵失效, 与 `UNIMPLEMENTED_CMDS` 同一个理由）;
#   ③ DFU 是固件**维护**命令（进 ROM bootloader）, 不是运动 API, 豁免语义成立。
#
# [段三] 该缺口已被**补上**: `0x15` 现在有常量、有入口 (`Arm.enter_dfu`, 两段式:
#   ACK 之后还要**等设备真的消失**), 并登记进 `COMMAND_COVERAGE` —— 于是本表**清空**,
#   覆盖契约 = **47/47**。留空是有意的, 与 `UNIMPLEMENTED_CMDS` / `FIRMWARE_ONLY_CMDS`
#   同一个用法: 它现在是"**当前没有任何既有的固件/SDK 不同步**"这句话的断言载体,
#   而不是可以顺手塞东西的地方（哨兵 `test_firmware_only_exemption_is_still_valid`
#   对它逐条断言"固件里仍有这条命令、且 SDK **还没有**同名常量" ⇒ 往空表里塞东西前,
#   先确认那件事真的没做）。
# ⚠ **两张表的语义不要混**: `FIRMWARE_ONLY_CMDS`/`FIRMWARE_ONLY_RSPS` 的语义是
#   "用户裁决 SDK **有意不暴露**"（"不做", 不是"还没做"; 笛卡尔 0x3A~0x3E / 0x4E 曾是
#   那里的例子, 段二已暴露并补上入口, 见本节上方那段历史说明 —— ⚠ 别按行号找, 它会漂）;
#   而本表的语义自始至终是"**已知缺口**、本应同步而未同步" —— 混进"有意不做"的条目
#   会让人以为那缺口是有意留的。两张表现在**都是空的**。
# ---------------------------------------------------------------------------

#: 既有的固件/SDK 不同步（非笛卡尔）: 码值 -> 名字。当前为空 (DFU 已在段三补上)。
PREEXISTING_GAPS: dict = {}


def crc16_ccitt_false(data: bytes) -> int:
    """**CRC-16/CCITT-FALSE**（poly `0x1021` / init `0xFFFF` / 无末异或）。

    覆盖 `[SOF..PAYLOAD]`，两字节**低字节在前**（见 `pack_frame`）。
    `binascii.crc_hqx(data, 0xFFFF)` **就是同一个算法**（同 poly、同 init、同无末异或、
    同 MSB-first），它由 C 实现。

    ⚠ **为什么从纯 Python 位循环换成它**（2026-09-22 实测，`.149`）：

    * 纯 Python 那个双重循环（每字节 8 次位运算）**每帧 158B 要 0.595 ms**，
      而它**每一帧都要算**（`pack_frame` 下行 + `unpack_frame` 上行各一次）；
    * 那 0.595 ms 占读线程**每帧总成本（1.774 ms）的 34%**；读线程又按固件推的
      **100 Hz** 跑 ⇒ 约 6% 的一个核，是**裸 SDK 空闲 21.4% 里的约 28%**；
    * `crc_hqx` 实测 **232×** 快（68.1 µs → 0.29 µs，同一台机器同一载荷）。

    ⚠ **等价性不是"看算法像"认定的，是测出来的**：3260 个载荷（长度 0..259 全覆盖 +
    随机）与真帧逐一比对 **0 不一致**；并由 `tests/test_protocol_crc.py` **长期钉住** ——
    那里有一份**独立的纯 Python 参考实现当判据**，另有 CRC 目录的**标准检查值**
    （`"123456789"` → `0x29B1`）。**别把那条用例改成拿本函数跟 `crc_hqx` 对拍** ——
    那是自己验自己，判据会失去判别力。
    """
    return binascii.crc_hqx(data, 0xFFFF)


def pack_frame(cmd: int, payload: bytes = b"") -> bytes:
    if len(payload) > 255:
        raise ValueError("payload >255B")
    body = bytes([SOF, cmd, len(payload)]) + payload
    crc = crc16_ccitt_false(body)
    return body + bytes([crc & 0xFF, crc >> 8])


def unpack_frame(frame: bytes) -> Optional[Tuple[int, bytes]]:
    """校验完整帧; 合法返回 (cmd,payload), 否则 None。"""
    if len(frame) < 5 or frame[0] != SOF:
        return None
    ln = frame[2]
    if len(frame) != ln + 5:
        return None
    body = frame[:3 + ln]
    crc = frame[3 + ln] | (frame[4 + ln] << 8)
    if crc16_ccitt_false(body) != crc:
        return None
    return frame[1], frame[3:3 + ln]


def pack_f32s(values) -> bytes:
    return b"".join(struct.pack("<f", float(v)) for v in values)


def unpack_f32s(b: bytes, off: int, count: int) -> List[float]:
    return list(struct.unpack_from("<" + "f" * count, b, off))


def parse_firmware_version(ver: str) -> Optional[Tuple[int, int, int, str]]:
    """'Litearm1.5.2-7J' -> (1,5,2,'7J'); 旧 'A1.x-...-USB' 返回 None(不符合新约定)。
    """
    s = ver.strip()
    if s.startswith("Litearm"):
        s = s[len("Litearm"):]
    elif s.startswith("A"):
        return None
    else:
        return None
    ver_part, _, variant = s.partition("-")
    variant = variant or ""
    parts = ver_part.split(".")
    if len(parts) != 3 or not all(p.isdigit() for p in parts):
        return None
    return int(parts[0]), int(parts[1]), int(parts[2]), variant


def decode_status(payload: bytes):
    """状态帧 -> (flags:int, seq:int, mode:int, flag_names:list,
    joints:list[(q,dq,tau,tmos,tcoil,err)], joint_fault:int); 非法返回 None。

    兼容两种布局 (尾部 u16 joint_fault 为固件 1.5.0 起新增):
      4+21N  固件 <=1.4.x   (无 joint_fault, 视为 0)
      6+21N  固件 >=1.5.0   (7J=153B / 1J=27B)
    """
    if len(payload) < 4:
        return None
    flags = struct.unpack_from("<H", payload, 0)[0]
    seq = struct.unpack_from("<H", payload, 2)[0]
    n = (len(payload) - 4) // 21
    if n not in (1, 7):
        return None
    tail = len(payload) - 4 - n * 21
    if tail not in (0, 2):        # 0 = 旧布局, 2 = joint_fault u16
        return None
    joint_fault = (struct.unpack_from("<H", payload, 4 + n * 21)[0]
                   if tail == 2 else 0)
    joints = []
    for i in range(n):
        q, dq, tau, tmos, tcoil = struct.unpack_from("<fffff", payload, 4 + i * 21)
        err = payload[4 + i * 21 + 20]
        joints.append((q, dq, tau, tmos, tcoil, err))
    mode = (flags >> 6) & 0x7
    names = [FLAG_NAMES[k] for k in range(6) if flags & (1 << k)]
    return flags, seq, mode, names, joints, joint_fault
