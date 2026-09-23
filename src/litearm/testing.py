"""脚本化应答的假传输 —— 让 Arm 全流程可离线测试 (镜像 pylitearm 桩硬件风格)。

按收到的下行命令推送预设帧 (ACK/状态/RSP_TCP/RSP_IK), 维持 Arm 接口
(write_frame/read_frame/close/is_open), 供 pytest monkeypatch SerialTransport 用。
"""
from __future__ import annotations

import struct
import threading
import time
from typing import Optional, Tuple

from litearm import _protocol as P
from litearm.errors import TransportError

_N = 7


def _status(q, dq=None, mode=1, flags=0, seq=0, joint_fault=0, n=_N) -> bytes:
    """按**固件 1.5.x 真实布局** (6+21N) 造状态帧 —— 尾部带 joint_fault u16。

    旧桩只造 4+21N, 导致"SDK 解不了真固件状态帧"这一 P0 在离线测试里全绿被掩盖。
    """
    body = bytearray(struct.pack("<HH", flags | (mode << 6), seq))
    for i in range(n):
        body += struct.pack("<fffff", q[i] if i < len(q) else 0.0,
                            (dq[i] if dq is not None else 0.0), 0.5, 30.0, 25.0)
        body.append(0)
    body += struct.pack("<H", joint_fault)
    return P.pack_frame(P.RSP_STATUS, bytes(body))


def _ack(cmd: int) -> bytes:
    return P.pack_frame(P.RSP_ACK, bytes([cmd]))


#: [笛卡尔] 5 条命令各自的最小载荷长度 —— **逐字照固件 `usb_cmd.c` 的长度校验**
#: (`CMD_MOVE_L` 28 / `CMD_MOVE_C` 52 / `CART_BEGIN` 5 / `CART_ADD` 25 / `CART_RUN` 0)。
#: 固件把长度校验排在门禁与一切副作用**之前**, 桩照同一顺序, 于是空载荷只会得到
#: `ERR{cmd,0x01}` 而不会让臂动 —— SDK 的能力探测正是靠这一点, 别把这层改成"先受理"。
_CART_MIN_LEN = {0x3A: 28, 0x3B: 52, 0x3C: 5, 0x3D: 25, 0x3E: 0}

#: 会**起规划**、因而会产出一条 `0x4E` 的三条; `0x3C`/`0x3D` (收集态) 不产出。
_CART_PLANNING_CMDS = frozenset({0x3A, 0x3B, 0x3E})


#: KIN_BENCH 回文本 —— **逐字节镜像固件 kin_runner.c 的组装结果**, 不是"看着像"。
#: 关键在 `u32_cat()` 的语义: 数字**先写digits再补一个尾随空格**, 而 `txt()` 原样拼接
#: (不加分隔符)。于是:
#:   txt("FK") + u32_cat(200) + u32_cat(108) + u32_cat(240) -> `FK200 108 240 `
#:   txt("LOOP1") + u32_cat(4000)                            -> `LOOP14000 `   ← 名字与数字相连!
#:   ...u32_cat(12345) + txt("/") + u32_cat(6) + txt("st=")  -> `flt=12345 /6 st=1 `  ← 数字与 '/' 间有空格
#: 最后一行 LINK 是拿到 CRC 坏帧 / 应答 FIFO 丢弃 / CAN TX 失败分类桶等诊断计数的唯一通道。
KIN_BENCH_TEXT = (
    "FK200 108 240 \n"
    "JAC200 220 610 \n"
    "IK100 2600 5200 \n"
    "LOOP14000 \n"
    "SC200 60 120 \n"
    "G200 300 700 \n"
    "RNEA200 800 1500 \n"
    "M200 900 2200 \n"
    "LAW200 50 90 \n"
    # ⚠ LINK 行的键**按固件 `kin_runner.c:513-560` 的实际发射顺序与命名**抄全 ——
    #   桩比真机短是这一类缺陷反复漏网的原因 (`rxl0`/`rxl1`/`rbd`/`cmode`… 从前就没有,
    #   于是"解析器漏掉它们"在离线永远测不出来)。
    #   ⚠ `rxl0`/`rxl1` **名字里带数字** —— 解析器必须吃得下 (见 `test_..._digit_keys`)。
    "LINK crc=3 ovf=11 rfd=1 txf=7 rxlen=5 rxl0=13 rxl1=17 rbd=19 "
    "cmode=127 cmb=23 tfe=1 tfd=2 tfc=3 tfs=0 tfm=1 tec=29 rec=31 psr=8 "
    "ebo=0 eep=0 eew=0 epa=0 epd=0 "
    " loop_k=2 ovr=4  flt=12345 /6 st=1 \n"
)


def _split_kin_bench(text, one_frame=False):
    """把 KIN_BENCH 全文按**真机那样**拆帧 (第 1 帧耗时 / 第 2 帧 LINK 行)。

    固件侧 `[H-9 fix 2026-09-13]` 的原话: "把 bench 各项耗时作为第 1 帧先发,
    再把 LINK 诊断行重置缓冲后作第 2 帧单独发。`usb_cmd_reply` 走 FIFO 保序,
    上位机按连续两帧组装即可"。

    ⚠ 拆点取 **`LINK` 行首** —— 固件是"重置缓冲后从 `LINK ` 重新写", 故第 2 帧
      以 `LINK` 开头、不含前面的换行。
    ⚠ 不含 `LINK` 的文本仍回**一帧** —— 空文本要走 `parse_kin_bench` 的
      "空回执 = 异常" 判据, 拆成零帧会让那条判据变成"收不到帧"。
    """
    if "LINK" not in text:
        return [text]
    head, _, tail = text.partition("LINK")
    parts = [head, "LINK" + tail]
    return parts[:1] if one_frame else parts


def _mk_log(n_ticks: int, n: int) -> bytes:
    """固件 log_sample_t 的字节流: u32 tick + q_ref[N] + dq[N] + tau[N] (4+12N B)。

    内容取确定值, 便于断言端到端字节一致 (不模拟轨迹)。
    """
    out = bytearray()
    for i in range(n_ticks):
        out += struct.pack("<I", i)
        out += P.pack_f32s([float(10 * j + 1) for j in range(n)])   # q_ref
        out += P.pack_f32s([0.5] * n)                               # dq
        out += P.pack_f32s([-0.25] * n)                             # tau
    return bytes(out)


class FakeTransport:
    """真实串口传输的替换物: 不收字节, 直接对 write_frame 的命令做脚本应答。"""

    def __init__(self, port: str = "fake", timeout: float = 0.2,
                 fw: str = "Litearm1.7.0-7J", n: int = _N):
        self.port = port
        self.timeout = timeout
        self.fw = fw
        self.n = int(n)
        self._resp: list[bytes] = []
        self.ff_vec: dict = {}           # item -> 7 值 (供 0x2B 读回)
        self.ff_scalar: dict = {}        # (item, sub) -> 值 (供 0x2C 读回)
        self.ff_mask = 0                 # item 9 只读回填
        self.q = [0.0] * self.n          # 当前(假)关节位形
        self.pose = [0.300, 0.0, 0.350, 0.0, 0.0, 0.0]
        self.auto_status = False         # 缓冲空时是否补一拍 idle 状态 (模拟 100Hz)
        #: 合成状态帧的**最小间隔**（秒）—— 见 `read_frame`。
        #: ⚠⚠ 没有它，`read_frame` 会**无限快**地造状态帧。从前没人在意（读是**调用方
        #: 驱动**的：调用方不读就没有帧）；2026-09-22 起 SDK 有**读线程**，它会以
        #: 桩能供上的最高速率空转 ⇒ 整套用例被 GIL 拖垮（实测单文件从秒级变到十几秒）。
        #: 取 1ms 而不是真实的 10ms（100Hz）：桩**可以**比真机快，但不能无界。
        self.auto_status_period = 0.001
        self._auto_status_last = 0.0
        self.closed = False
        #: 收到的全部下行命令 —— 保活线程会并发写, 故用锁保护。
        #: `tx_stamps` 与 `tx_log` 同下标, 供"相邻帧间隔"类不变量断言用。
        self.tx_log: list = []
        self.tx_stamps: list = []
        self._tx_lock = threading.Lock()
        #: 注入写失败: 第 N 次 CMD_ZERO_G 之后抛 TransportError (模拟 CDC 掉线)
        self.zg_fail_after: Optional[int] = None
        self.zg_writes = 0
        #: >0 时每次保活写 (0x06 on=1) 卡这么久 —— 模拟 CDC 写阻塞
        self.zg_slow_write_s = 0.0
        #: 固件"没有实现"的命令 —— 按固件 default 分支回 ERR{cmd,0x00}
        self.unknown_cmds: set = set()
        #: 强制某命令回指定错误码 (模拟 ERR{cmd,code})
        self.err_override: dict = {}
        #: [2026-09-14] 动力学模型在线导入 (0x30/0x32/0x33/0x34/0x35/0x37/0x38/0x39)。
        #: 模型恒 9 刚体 —— **与 self.n (关节数) 无关**, 台架 1J 下也是 9。
        #: body8 (ee) 质量必须恒 0; 初始 = "编译期常量" (用全 1 质量 / 0 com / 0 I 代表)。
        self.model_nbody = 9
        self.model_body = [[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                           for _ in range(self.model_nbody)]
        self.model_body[self.model_nbody - 1][0] = 0.0     # body8 质量恒 0
        self.model_jm = [0.0] * 7
        #: staging (0x30/0x33 写这里, 不生效)
        self.model_stage_body = {}
        self.model_stage_jm = None
        self.model_override = False
        self.model_dirty = False
        #: 关节级参数表 (0x22/0x23 写, 0x24 读, 0x36 回出厂默认)
        self.enabled = False
        # ---- [授权] 见 `_protocol` 的 license 段; 判据在 tests/test_license.py ----
        #: 是否已激活。**默认 True** —— 否则每一条既有用例的 `enable()` 都会撞
        #: `ERR{0x10,0x08}`（固件 `ctrl_enable()` 的第一条判据）, 而那些用例与授权无关。
        #: 授权用例把它置 False 来复现"开机即锁"。
        self.activated = True
        #: 激活记录 —— `0x2F` 回报的就是这几格 (未激活时后三格按固件约定**恒 0**)。
        self.license_ver = 1
        self.license_uid = bytes(range(0x10, 0x1C))     # 12B, 原始寄存器内存序
        self.license_cust_id = 0
        self.license_issued = 0
        self.license_flags = 0
        #: 注入: 下一次 `0x3F` 回 `ERR{0x3F,0x02}`（模拟 MAC 不符 / 密钥非法 / 写失败,
        #: 它们**全折成这一档**）。
        #: ⚠ 桩**不可能**真的验 MAC —— 它**没有也不需要**密钥（规格硬要求: 客户侧不许有
        #: 算 MAC 的代码, 否则机制归零）。真验签是固件的活; SDK 侧要测的只是
        #: 「0x02 是聚合档 ⇒ 必须回读 `0x2F` 才能定性」那一段。
        self.license_fail_next = False
        self._jp_default = dict(kp=50.0, kd=2.0, tau_max=10.0, q_min=-3.0, q_max=3.0)
        self.joint_params = [dict(self._jp_default) for _ in range(self.n)]
        #: 300Hz 采集: log_bytes 为固件侧缓冲的原始字节流 (LOG_MAX_SAMPLES=2400)
        self.log_max = 2400
        self.log_bytes = b""
        self.log_active = False
        #: 模拟 USB 掉帧: 下一次 CMD_LOG_READ 不回 (读者须按游标续读/重试)
        self.log_read_fail_once = False
        #: >0 时模拟固件的**逐拍记录** (300Hz 语义): 启动后按"每拍 1/log_hz 秒"
        #: 推进, **与读回次数无关** —— 这是必须的真实模型: start() 之后立刻读回
        #: 只能拿到一个前缀, 等待必须靠轮询已记录量。(若让"记录"随读回次数推进,
        #: read_all() 会自己"等到"录满, 测试就变成假通过。)
        #: 0 = 立刻录满 (只有数据内容需求的用例用这个)。
        self.log_hz = 0.0
        self.log_t0 = None
        self.log_target = 0
        self.log_recorded_ticks = -1     # 上次重建 log_bytes 时的拍数 (避免每读回一次重建)
        #: 模拟固件回一个**不前进**的游标 (next_byte == offset) —— 读者必须自己
        #: 判定"无进展"并报错, 否则会死循环。上限次后抛错以免测试挂死。
        self.log_cursor_stuck = False
        self._log_read_calls = 0
        #: KIN_BENCH 回文本 (可改写成空/畸形以测异常路径)
        self.kin_bench_text = KIN_BENCH_TEXT
        #: 模拟**旧固件**: 只发第 1 帧 (耗时帧), 不发 LINK 诊断帧。
        #: 用于钉住"收第 2 帧超时不是错误"这条兼容行为。
        self.kin_bench_one_frame = False
        #: 覆盖 GET_TCP 回读的位置/朝向 (模拟固件的 rpy 规范化 —— 含万向锁强制 yaw=0,
        #: 见 `kin_rot_to_rpy` `kin.c:360-385`, 锁分支在 `:380-384`)
        self.pos_override = None
        self.rpy_override = None
        # ---- [笛卡尔] 0x3A~0x3E + RSP_CART_PLAN ----
        #: 固件的 `#if LITEARM_CART_PLAN` 编译开关。False = 这 5 条整段不在固件里,
        #: 于是落到 `default` 分支回 `ERR{cmd,0x00}` (SDK 的 `UnsupportedByFirmwareError`)。
        self.cart_supported = True
        #: `ok=1` 的 `0x4E` 载荷里的 n_wp / plan_us (确定性常量, 便于断言)。
        self.cart_n_wp = 12
        self.cart_plan_us = 4700
        #: 非 None 时 0x4E 回 `ok=0 + err=该值` —— 造规划失败 (`cart_err_t` 1..6)。
        #: ⚠ 与 `err_override` 不同: 后者回的是 `RSP_ERR` (命令级拒绝), 前者是**规划结果**。
        self.cart_err_override: Optional[int] = None
        #: 交付给 SDK 的状态帧计数 (自最后一条 `0x4E` 起) —— 驱动 CART_BUSY (bit10)。
        #: None = 当前没有在跑的计划, 状态帧照原样发。
        self.cart_busy_seq: Optional[int] = None
        #: `move_path` 最近一条 `0x3D` (ADD) 的位姿 —— `0x3E` (RUN) 的目标就是它。
        #: 与 `pose` 一样是"桩不建模运动过程, 只保证收尾时 TCP 在目标上"的一部分。
        self._path_last_pose = list(self.pose)
        #: **交付给 SDK 的状态帧序号** (`g_arm.seq`) —— 每交付一帧 +1 (u16 回绕)。
        #: 固件那个计数器就是"这一帧什么时候生成的"的唯一线索, 到位判据的**新鲜度闸**
        #: 靠它判"这一帧是不是命令之后生成的" ⇒ 桩必须逐帧递增, 否则每一帧都会被判成
        #: 陈旧帧 (闸恒关)。⚠ 在**交付点**递增 (与 `cart_busy_seq` 同处): 数的是 SDK
        #: 真正看到的帧, 与它来自命令应答还是空闲流无关。
        self.status_seq = 0
        # ---- [DFU] 0x15: 只登记, 不在命令处理里跳 ----
        #: `ctrl_request_enter_dfu` 的**另一半**门禁: 固件判的是
        #: `ctrl_is_armed() = enabled || enable_pending` (定义在 `control_loop.c:1334-1336`;
        #: `:1091` 是 `ctrl_request_enter_dfu` 里把同一谓词**内联**写的那一行), 而
        #: `enable_pending` 在状态帧里**没有位** (SDK 只能看到 `self.enabled`/bit9)。
        #: 置 True = 造"使能在途"那个形状: SDK 预检放行、固件回 `ERR{0x15,0x03}`。
        self.dfu_armed_pending = False
        #: 模拟 `rom_bl_table_valid()` 判失败 ⇒ `ERR{0x15,0x02}` (`control_loop.c:1095`)。
        self.dfu_rom_table_invalid = False
        #: 登记成功后设备**是否真的离开** CDC。
        #: True (默认) = 真跳转: ACK 之后**读路径抛 `TransportError`**, 与真机在
        #: Linux 上的行为同形 (`select` 报就绪而 `read()` 返回空 ⇒ pyserial 抛
        #: `SerialException` ⇒ `transport._read_chunk` 包成 `TransportError`)。
        #: False = 两条**静默撤销**路径的形状 (`control_loop.c:1106` 并发 ENABLE /
        #: `:1141-1144` 交棒前复读失败): 设备还在 CDC 上, 而 ACK 是成功的。
        self.dfu_vanishes = True
        #: 状态: 设备已经离开 (读/写都抛 `TransportError`)。由成功登记 + `dfu_vanishes` 置。
        self.dfu_gone = False
        #: 登记 ACK **之后**、设备离开**之前**还会继续投递的帧 (现实里是 100Hz 状态流与
        #: 别的命令的迟到应答)。给"消失观察窗口不许静默吞帧"那条纪律造形状用。
        self.dfu_post_ack_frames: list = []

    def stamps_of(self, cmd: int) -> list:
        """某命令各次下行的时刻 (与 `tx_log` 同下标)。"""
        return [t for (c, _p), t in zip(self.tx_log, self.tx_stamps) if c == cmd]

    @property
    def armed(self) -> bool:
        """固件 `ctrl_is_armed()` (`control_loop.c:1334-1336`) 的桩侧等价物。

        固件判的是 `g_arm.enabled || enable_pending`, 而 `enable_pending` 在状态帧里
        **没有位** (SDK 只看得到 `self.enabled` / bit9) ⇒ 桩里由 `dfu_armed_pending`
        造那个形状。**五处门禁 (0x25/0x36/0x32/0x37/0x23) 与 0x15 共用这一个判据** ——
        逐条对得上固件这六处 `usb_cmd.c` 的用法 (`:670` / `:684` / `:877` / `:923`
        / `:633` / `:543-555`)。判据只有这一份, 别在任何一处退回 `self.enabled`
        (`test_full_coverage.py::test_all_gates_share_the_one_armed_predicate` 逐处钉住)。
        """
        return self.enabled or self.dfu_armed_pending

    def _log_recorded(self) -> int:
        """固件此刻已记录多少拍 (按时间推进)。"""
        if self.log_hz <= 0 or self.log_t0 is None:
            return self.log_target
        return min(self.log_target, int((time.monotonic() - self.log_t0) * self.log_hz))

    def _axis_outside(self, idx: int, qmin: float, qmax: float) -> bool:
        """`0x23` 门禁的核心判据 —— 逐条镜像固件 `ctrl_axis_outside`
        (`litearm-stm32/User/litearm/control/control_loop.c:610-652`)。

        固件那六条 (命中任意一条即视为"新限位关不住执行器接下来会走到的地方"):
          ① `idx` 越界   ② `!(qmin < qmax)` (空区间/NaN)
          ③ `cart_state() != IDLE`   ④ `sync_playing()`
          ⑤ `g_arm.cmd[idx].q_ref` 越界   ⑥ `g_sc[idx].active && g_sc[idx].qf` 越界

        **本桩能复现 ①②⑤ 三条**; 其余三条 (笛卡尔在途 / 同步轨迹在途 / 在途 S 曲线的
        终点 `qf`) **未建模** —— 桩不建模"运动过程", 没有在途状态可查。要在离线用例里
        造"固件拒绝"的形状, 用既有的 `err_override[cmd] = 0x03` 注入, 效果等价。

        ⚠ **⑤ 的载体是代理关系, 不是近似 —— 且只在两个命令上精确**: 固件查的是
        `g_arm.cmd[idx].q_ref` (它在命令受理时就更新成新目标), 而桩里没有对应**字段**
        (命令缓存与运动过程都不建模, 只有 `self.q`) —— 这里规定以 `self.q[idx]` 代理它。
        逐条核对本桩哪些分支**写** `self.q` (实测: 用裸 `write_frame` 打一遍, 看 `q[0]`
        是否动):

          精确: `0x01 MOVE_J` / `0x2A HOME` —— 这两个把 `self.q` 直接置成目标。
          停在旧值: `0x02 MOVE_P` (只写 `self.pose`) / `0x03 MOVE_JS` / `0x04 MOVE_MIT`
            / `0x05 MOVE_MIT_ALL` (三条只回 ACK) / `0x07 MOVE_J_SYNC` (桩里**没有分支**,
            落到 `else: _ack`)。

        ⚠ 于是上面五种情况里代理**失真** (固件 `q_ref` 已到新目标, 桩 `self.q` 还在旧值)。
        这属**潜伏**, 目前无任何用例能踩到: `0x07` 是最像的同族, 而 `Arm.movej_sync` 到
        不了位就抛 `MotionTimeoutError` (异目标实测), 控制回不到用例体; 唯一调用它的
        用例 (`test_full_coverage.py:36`) 给的目标与当前 `q` 相同, 于是立刻判到位、代理
        仍然准确。其余四条之后现有用例发的 `0x23` 要么根本不在武装态 (门禁整条跳过),
        要么区间容得下旧值 (`test_full_coverage.py:54` 的 `±3.0`)。
        """
        if not 0 <= idx < self.n:
            return True                    # ① (固件收 uint8_t, 只有上界那一半会命中)
        if not qmin < qmax:
            return True                    # ② 空区间/NaN —— 固件注释: 交给 params 层判
        return self.q[idx] < qmin or self.q[idx] > qmax            # ⑤ (q_ref 的代理)

    # ---- SerialTransport 接口 ----
    def close(self) -> None:
        self.closed = True

    @property
    def is_open(self) -> bool:
        return not self.closed

    def write_frame(self, cmd: int, payload: bytes = b"") -> None:
        if self.dfu_gone:
            # 设备已进 ROM bootloader: 帧发不出去 (**不记账** —— 现实里它根本没进 CDC)
            raise TransportError("桩: 设备已进 ROM bootloader, CDC 上的写路径失败")
        with self._tx_lock:
            self.tx_log.append((cmd, bytes(payload)))
            self.tx_stamps.append(time.monotonic())
        if cmd in self.unknown_cmds:
            # 固件 usb_cmd.c 的 default 分支: 未实现命令只回 ERR{cmd,0x00}, 无任何副作用
            self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x00])))
            return
        if cmd in self.err_override:
            self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, self.err_override[cmd]])))
            return
        if cmd == P.CMD_ZERO_G:
            self.zg_writes += 1
            if self.zg_slow_write_s and payload == b"\x01" and self.zg_writes > 1:
                time.sleep(self.zg_slow_write_s)
            if self.zg_fail_after is not None and self.zg_writes > self.zg_fail_after:
                raise TransportError("桩: CDC 写失败 (注入)")
            self._push(_ack(cmd))
        elif cmd == P.CMD_GET_FIRMWARE:                 # 握手
            self._push(P.pack_frame(P.RSP_FIRMWARE, self.fw.encode()))
            self.auto_status = True
            self._push(_status(self.q, mode=0, n=self.n))
        elif cmd == P.CMD_GET_STATUS:
            self._push(_status(self.q, mode=1, n=self.n))
        elif cmd == P.CMD_SET_FF_VEC:                 # 存下来供 0x2B 读回
            self.ff_vec[payload[0]] = list(P.unpack_f32s(payload, 1, self.n))
            self._push(_ack(cmd))
        elif cmd == P.CMD_SET_FF_SCALAR:              # 存下来供 0x2C 读回
            self.ff_scalar[(payload[0], payload[1])] = struct.unpack_from("<f", payload, 2)[0]
            self._push(_ack(cmd))
        elif cmd == P.CMD_SET_FF_FLAGS:
            self.ff_mask = struct.unpack_from("<I", payload, 0)[0]
            self._push(_ack(cmd))
        elif cmd == P.CMD_GET_FF_VEC:                 # [readback] 载荷首字节 = RSP id
            item = payload[0]
            vals = self.ff_vec.get(item, [0.0] * self.n)
            self._push(P.pack_frame(P.RSP_FF_VEC,
                                    bytes([P.RSP_FF_VEC, item]) + P.pack_f32s(vals)))
        elif cmd == P.CMD_GET_FF_SCALAR:              # item 9 = ff_mask (只读)
            item, sub = payload[0], payload[1]
            val = float(self.ff_mask) if item == 9 else self.ff_scalar.get((item, sub), 0.0)
            self._push(P.pack_frame(P.RSP_FF_SCALAR,
                                    bytes([P.RSP_FF_SCALAR, item, sub])
                                    + struct.pack("<f", val)))
        elif cmd == P.CMD_ENABLE:                     # 跟踪使能态 (0x36 门禁用)
            if not self.activated:
                # 照 `control_loop.c:938-946`: 未激活是 `ctrl_enable()` 的**第一条**判据
                # (优先于 EMERGENCY / joint_fault / CMODE 那些), 重发无用、无旁路。
                self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x08])))
            else:
                self.enabled = True
                self._push(_ack(cmd))
        elif cmd == P.CMD_GET_LICENSE:                # 0x2F -> RSP_LICENSE(0x4F) 26B
            # 逐条照 `usb_cmd.c:1016-1030`:
            #   · 请求**不校验长度** (有意放宽, 不是为了省事);
            #   · **未激活也回 UID**, 而 cust_id/issued/flags 保持 0;
            #   · 载荷首字节是 `state`, **不是**冗余帧 id (与 RSP_MODEL_STATUS 那类不同)。
            st = 2 if (self.activated and self.license_flags & 0x1) else (
                1 if self.activated else 0)
            body = bytearray([st, self.license_ver]) + self.license_uid
            if self.activated:
                body += struct.pack("<III", self.license_cust_id,
                                    self.license_issued, self.license_flags)
            else:
                body += struct.pack("<III", 0, 0, 0)
            self._push(P.pack_frame(P.RSP_LICENSE, bytes(body)))
        elif cmd == P.CMD_ACTIVATE:                   # 0x3F: 28B -> ACK{0x3F} / ERR
            # 逐条照 `usb_cmd.c:1031-1081` 的顺序: 长度 -> 武装 -> flash 忙 -> 写。
            if len(payload) < 28:
                self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x01])))
            elif self.armed:
                # ⚠ 用统一的 `armed` 判据 (固件 `ctrl_is_armed()`), 不是裸 `self.enabled`
                # —— 由 `test_full_coverage.py::test_all_gates_share_the_one_armed_predicate` 钉住。
                self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x04])))
            elif self.license_fail_next or self.activated:
                # 0x02 是**聚合档** —— "已经激活过"与"MAC 不符/写失败"混在一起。
                self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x02])))
            else:
                (self.license_cust_id, self.license_issued,
                 self.license_flags) = struct.unpack_from("<III", payload, 0)
                self.activated = True
                self._push(_ack(cmd))
        elif cmd == P.CMD_ENTER_DFU:
            # [DFU] 0x15 —— 逐条照 `usb_cmd.c:543-555` + `control_loop.c:1087-1099`:
            # 长度优先, 再两道门禁 (使能 / ROM 向量表), 最后**只登记**。
            # ⚠ 桩**不模拟跳转本身** (真固件交棒给 main 线程, 上界 100ms); 它模拟的是
            # 主机**看得见**的那一半: ACK 之后设备是否还在 CDC 上 (`dfu_vanishes`)。
            if payload:
                self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x01])))
            elif self.armed:
                self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x03])))
            elif self.dfu_rom_table_invalid:
                self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x02])))
            else:
                self._push(_ack(cmd))                 # ACK{0x15} = "已登记"
                for extra_cmd, extra_payload in self.dfu_post_ack_frames:
                    self._push(P.pack_frame(extra_cmd, extra_payload))
                if self.dfu_vanishes:
                    self.dfu_gone = True
        elif cmd == P.CMD_DISABLE:
            self.enabled = False
            self._push(_ack(cmd))
        elif cmd in (P.CMD_EMERGENCY_STOP,
                     P.CMD_CLEAR_FAULTS, P.CMD_RESET, P.CMD_SET_SPEED_PERCENT,
                     P.CMD_SET_MOTION_MODE, P.CMD_MOVE_JS, P.CMD_MOVE_MIT,
                     P.CMD_MOVE_MIT_ALL, P.CMD_FF_PRESET):
            self._push(_ack(cmd))
        elif cmd == P.CMD_SET_JOINT_PARAM:            # 0x22: idx + kp,kd,tau_max
            if len(payload) < 13:
                self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x01])))
            elif payload[0] >= self.n:
                self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x02])))
            else:
                kp, kd, tm = struct.unpack_from("<fff", payload, 1)
                self.joint_params[payload[0]].update(kp=kp, kd=kd, tau_max=tm)
                self._push(_ack(cmd))
        elif cmd == P.CMD_SET_JOINT_LIMITS:           # 0x23: idx + q_min,q_max
            # ⚠ 分支顺序**逐条照固件** `usb_cmd.c:625-641` 的 case: 长度(0x01) → **门禁
            # (0x03)** → idx/空区间(0x02)。门禁排在 idx 校验**之前**不是可选风格: 固件对
            # "武装 + idx 越界" 回 `0x03` (`ctrl_axis_outside` 第①条命中), 桩把 idx
            # 提前就回 `0x02` —— 这一格与真机**相反**, 而所有走 SDK 入口的用例都看不见
            # (SDK 在本地 `_check_idx` 就拦了)。
            #
            # ⚠ **`0x02` 这一层只建模了 `idx` / 空区间两格, 不是"值校验完整"**。固件那条
            # `0x02` 来自 `params_set_joint_limits` (`params.c:59-92`), 它还有两个桩**没有**
            # 的层:
            #   ① **钳位层** (`[N-6 fix]`): 两个值先各自钳到本关节**编译期**软限位
            #      (`joint_limit_macros.h` 的 JL_QMIN/MAX), 钳后 `nmin >= nmax` 才回 `0x02`。
            #      桩直接存原值 ⇒ 两格假绿:
            #        `set_joint_limits(0, 5.0, 6.0)` (失能): 固件 clamp(5.0)=clamp(6.0)
            #        =2.809547 ⇒ **ERR{0x23,0x02}**; 桩 **ACK 并把 (5.0,6.0) 存进参数表**。
            #        `set_joint_limits(0, -15.0, 15.0)`: 固件 ACK 但**静默收窄**到 ±2.809547;
            #        桩**原样存 ±15** (放宽 = 等价关掉 safety_check 的位置包络)。
            #   ② **`[P2-12]` 层**: `fresh` 且有反馈时, 把**当前实测位置**关在新区间外的
            #      请求也回 `0x02` (该锁存不受 `enabled` 门控 ⇒ "设成功了但臂锁死")。
            # 本段 (段三 §6.4) 只要求门禁一层, 故**不实现**这两层 —— 残余已记在
            # `docs/superpowers/plans/2026-09-19-litearm-python-migration-stage3.md`
            # 的"本段明确不做"表 (含上面两条可失败证据), 别让它悄悄消失。
            if len(payload) < 9:
                self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x01])))
            else:
                idx = payload[0]
                lo, hi = struct.unpack_from("<ff", payload, 1)
                if self.armed and self._axis_outside(idx, lo, hi):
                    # [B1 fix] 武装中该轴的当前参考必须落在新区间内, 否则 ACK 之后在途
                    # 轨迹仍奔向收窄前的旧目标 ⇒ 轴被开出新软限位 ⇒ 位置包络锁存。
                    self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x03])))
                elif idx >= self.n or not lo < hi:
                    self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x02])))
                else:
                    self.joint_params[idx].update(q_min=lo, q_max=hi)
                    self._push(_ack(cmd))
        elif cmd == P.CMD_GET_JOINT_PARAM:            # 0x24 -> RSP_JOINT_PARAM(0x49)
            if len(payload) < 1:
                self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x01])))
            elif payload[0] >= self.n:
                self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x02])))
            else:
                jp = self.joint_params[payload[0]]
                body = bytes([payload[0]]) + struct.pack(
                    "<fffff", jp["kp"], jp["kd"], jp["tau_max"], jp["q_min"], jp["q_max"])
                self._push(P.pack_frame(P.RSP_JOINT_PARAM, body))
        elif cmd == P.CMD_PARAM_SAVE:                 # 0x25: 须失能 (固件 ctrl_is_armed)
            # [M7] 使能运动中禁止整扇区擦写 (H723 单 bank 无 RWW, 擦写窗口 CPU 停顿,
            # 电机无监督运行): 先 DISABLE 再发 PARAM_SAVE。
            # 门禁判据 = `self.armed` (= 固件 `ctrl_is_armed()`), **含 `enable_pending`
            # 那一半**: 固件为此专门改过一次 (`usb_cmd.c:668-670` [M2 fix] 的理由就是
            # "登记→执行间隙 pending 自动完成也会武装电机"), 且执行侧 (`usb_cmd.c:1274`)
            # 还有一道复查。ACK 只表示"已登记", 擦写在 main 循环里做, 失败另回 0x25/0x03。
            if self.armed:
                self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x04])))
            else:
                self._push(_ack(cmd))
        elif cmd == P.CMD_PARAM_RESET:                # 0x36: 须失能
            if self.armed:
                self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x04])))
            else:
                self.joint_params = [dict(self._jp_default) for _ in range(self.n)]
                self._push(_ack(cmd))
        # ---- [2026-09-14] 动力学模型在线导入 ----
        elif cmd == P.CMD_SET_MODEL_PARAM:            # 0x30: body_idx + f32[10] -> staging
            if len(payload) < 1 + 4 * 10:
                self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x01])))
            elif payload[0] >= self.model_nbody:
                self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x02])))
            else:
                vals = list(struct.unpack_from("<" + "f" * 10, payload, 1))
                if any(v != v or abs(v) > 1e6 for v in vals):
                    self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x02])))
                elif payload[0] == self.model_nbody - 1 and vals[0] != 0.0:
                    self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x02])))
                else:
                    self.model_stage_body[payload[0]] = vals
                    self.model_dirty = True
                    self._push(_ack(cmd))
        elif cmd == P.CMD_SET_MODEL_JM:               # 0x33: f32[7] -> staging
            if len(payload) < 4 * 7:
                self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x01])))
            else:
                vals = list(struct.unpack_from("<" + "f" * 7, payload, 0))
                if any(v != v or abs(v) > 1e6 for v in vals):
                    self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x02])))
                else:
                    self.model_stage_jm = vals
                    self.model_dirty = True
                    self._push(_ack(cmd))
        elif cmd == P.CMD_MODEL_COMMIT:               # 0x32: u16 mask LE
            if len(payload) < 2:
                self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x01])))
            elif self.armed:
                # ⚠ 与固件同序: 门控优先于掩码检查
                self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x04])))
            else:
                mask = struct.unpack_from("<H", payload, 0)[0]
                staged = 0
                for i in self.model_stage_body:
                    staged |= (1 << i)
                if self.model_stage_jm is not None:
                    staged |= (1 << 9)
                if mask != staged:
                    # 掩码不符 -> 0x07 (与"数值非法"分开的码)
                    self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x07])))
                elif mask & ~0x3FF:
                    self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x02])))
                else:
                    for i, v in self.model_stage_body.items():
                        self.model_body[i] = v
                    if self.model_stage_jm is not None:
                        self.model_jm = self.model_stage_jm
                    self.model_override = True
                    self.model_stage_body = {}
                    self.model_stage_jm = None
                    self._push(_ack(cmd))
        elif cmd == P.CMD_GET_MODEL_PARAM:            # 0x34 -> RSP_MODEL_PARAM(0x54)
            if len(payload) < 1:
                self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x01])))
            elif payload[0] >= self.model_nbody:
                self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x02])))
            else:
                idx = payload[0]
                body = bytes([P.RSP_MODEL_PARAM, idx]) + struct.pack(
                    "<" + "f" * 10, *self.model_body[idx])
                self._push(P.pack_frame(P.RSP_MODEL_PARAM, body))
        elif cmd == P.CMD_GET_MODEL_JM:               # 0x35 -> RSP_MODEL_JM(0x55)
            body = bytes([P.RSP_MODEL_JM]) + struct.pack("<" + "f" * 7, *self.model_jm)
            self._push(P.pack_frame(P.RSP_MODEL_JM, body))
        elif cmd == P.CMD_REVERT_MODEL:               # 0x37: 只回退模型
            if self.armed:
                self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x04])))
            else:
                self.model_override = False
                self.model_stage_body = {}
                self.model_stage_jm = None
                self.model_dirty = True
                self._push(_ack(cmd))
        elif cmd == P.CMD_GET_MODEL_STATUS:           # 0x38 -> RSP_MODEL_STATUS(0x56)
            staged = 0
            for i in self.model_stage_body:
                staged |= (1 << i)
            if self.model_stage_jm is not None:
                staged |= (1 << 9)
            body = bytes([P.RSP_MODEL_STATUS, 1 if self.model_override else 0]) + \
                struct.pack("<H", staged) + bytes([1 if self.model_dirty else 0])
            self._push(P.pack_frame(P.RSP_MODEL_STATUS, body))
        elif cmd == P.CMD_GET_GRAVITY:                # 0x39 -> RSP_GRAVITY(0x57)
            if len(payload) < 4 * 7:
                self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x01])))
            else:
                # 假固件: 回一个确定性可预测的 G (sum(m)*g 在 J1 上 + 常量梯度),
                # 只为断言"帧格式对、值能被 SDK 解出来", 不做真实 RNEA。
                q = struct.unpack_from("<" + "f" * 7, payload, 0)
                msum = sum(b[0] for b in self.model_body[1:])
                g = [msum * 9.81 * 0.001 * (i + 1) for i in range(7)]
                g[0] = msum * 9.81 * q[1] * 0.1
                body = bytes([P.RSP_GRAVITY]) + struct.pack("<" + "f" * 7, *g)
                self._push(P.pack_frame(P.RSP_GRAVITY, body))
        elif cmd == P.CMD_KIN_BENCH:                  # 0x49 -> RSP_KIN_BENCH(0x4A) 文本
            # ⚠ **真机是连续两帧**, 不是一帧 (固件 `[H-9 fix 2026-09-13]`): 单帧载荷
            #   上限 255B 而全文 ~306B, 原单帧会在 253B 处截断, 截断点恰好落在 LINK 行
            #   的 `loop_k=` 之后 ⇒ 链路计数全丢。
            #   桩从前把全文塞一帧, 于是"只收一帧"的 SDK **离线全绿、真机上丢掉全部
            #   链路计数** —— 而丢得静默 (`link.get(k, 0)` 报 0), 与"没出错"分不开。
            for part in _split_kin_bench(self.kin_bench_text,
                                         one_frame=self.kin_bench_one_frame):
                self._push(P.pack_frame(P.RSP_KIN_BENCH, part.encode()))
        elif cmd == P.CMD_LOG_CTRL:                   # 0x2D: u32 n_ticks (0=停/清)
            if len(payload) < 4:
                self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x01])))
            else:
                n = struct.unpack_from("<I", payload, 0)[0]
                if n > self.log_max:
                    self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x02])))
                else:
                    self.log_target = int(n)
                    self.log_active = bool(n)
                    self.log_target = int(n)
                    if self.log_hz > 0 and n:
                        self.log_t0 = time.monotonic()   # 逐拍记录: 此刻还没有数据
                        self.log_bytes = b""
                        self.log_recorded_ticks = 0
                    else:
                        self.log_t0 = None
                        self.log_bytes = _mk_log(int(n), self.n) if n else b""
                        self.log_recorded_ticks = int(n)
                    self._push(_ack(cmd))
        elif cmd == P.CMD_LOG_READ:                   # 0x2E: u32 offset -> RSP_LOG_DATA
            if len(payload) < 4:
                self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x01])))
            elif self.log_read_fail_once:
                self.log_read_fail_once = False       # 掉帧: 什么都不回
            else:
                self._log_read_calls += 1
                if self.log_cursor_stuck:
                    # 第一次正常前进到 245, 此后永远回同一个游标 (next == off != 0)。
                    # 读者若无"无进展"保护就会死循环 —— 这里用次数上限把死循环
                    # 变成测试失败而不是挂死。
                    if self._log_read_calls > 50:
                        raise AssertionError(
                            "LOG_READ 游标不前进却一直被读 —— 读者缺无进展保护(会死循环)")
                    off0 = struct.unpack_from("<I", payload, 0)[0]
                    nxt = 245 if off0 == 0 else off0
                    n = 245 if off0 == 0 else 0
                    body = struct.pack("<IIB", 999999, nxt, n) + b"\x00" * n
                    self._push(P.pack_frame(P.RSP_LOG_DATA, body))
                    return
                if self.log_hz > 0 and self.log_t0 is not None:
                    # 逐拍记录: 按**经过的时间**推进 (与本次读回次数无关)。
                    # 只在拍数真的变了才重建缓冲 —— 否则每次读回都 O(全缓冲) 重建,
                    # 几百块读回会白白拖慢几秒 (固件侧只是 memcpy 245B)。
                    cur = self._log_recorded()
                    if cur != self.log_recorded_ticks:
                        self.log_recorded_ticks = cur
                        self.log_bytes = _mk_log(cur, self.n)
                off = struct.unpack_from("<I", payload, 0)[0]
                total = len(self.log_bytes)
                n = min(total - off, 245) if off < total else 0
                nxt = (off + n) if (off + n) < total else 0
                body = struct.pack("<IIB", total, nxt, n) + self.log_bytes[off:off + n]
                self._push(P.pack_frame(P.RSP_LOG_DATA, body))
        elif cmd == P.CMD_HOME:                       # 固件: 各轴低速回 URDF 零位
            self._push(_ack(cmd))
            self.q = [0.0] * self.n
            for _ in range(3):
                self._push(_status(self.q, dq=[1.0] * self.n, n=self.n))
            for _ in range(6):
                self._push(_status(self.q, dq=[0.0] * self.n, n=self.n))
        elif cmd == P.CMD_MOVE_J:
            self._push(_ack(cmd))
            q = P.unpack_f32s(payload, 0, self.n)     # payload: q×N + sp
            self.q = [float(v) for v in q]
            # 先 3 拍"运动中"(dq=1) 再 6 拍到位(dq=0) —— 检验 Arm 到位等待
            for _ in range(3):
                self._push(_status(self.q, dq=[1.0] * self.n, n=self.n))
            for _ in range(6):
                self._push(_status(self.q, dq=[0.0] * self.n, n=self.n))
        elif cmd == P.CMD_MOVE_P:
            self._push(_ack(cmd))
            self.pose = [float(v) for v in P.unpack_f32s(payload, 0, 6)]
            self._push(_status(self.q, dq=[0.0] * self.n, n=self.n))  # get_state refresh 用
        elif cmd == P.CMD_GET_TCP:
            pose = list(self.pose)
            if self.pos_override is not None:
                pose[:3] = list(self.pos_override)
            if self.rpy_override is not None:
                pose[3:] = list(self.rpy_override)
            self._push(P.pack_frame(P.RSP_TCP, P.pack_f32s(pose)))
        elif cmd == P.CMD_GET_IK:
            qs = [0.1, -0.2, 0.3, -0.1, 0.2, -0.05, 0.15]
            self._push(P.pack_frame(P.RSP_IK, P.pack_f32s(qs) + b"\x01"))
        elif cmd in _CART_MIN_LEN:                     # [笛卡尔] 0x3A~0x3E
            self._cart_cmd(cmd, payload)
        else:
            self._push(_ack(cmd))

    def _cart_cmd(self, cmd: int, payload: bytes) -> None:
        """[笛卡尔] 0x3A~0x3E 的桩行为 —— 长度校验 -> 受理(ACK) -> 规划结果(0x4E)。

        ⚠ 桩**不**建模使能/零重力/掉线锁存三道门禁 (那是另一件事): 要在离线用例里造
        "被门禁拒", 用既有的 `err_override[cmd] = 0x03` 注入 `RSP_ERR`, 效果等价。

        ⚠ **规划成功时桩把 `pose` 直接置成该命令的目标**: 到位之后回读的 TCP 必须与目标
        对得上 (`Arm._pose_near` 那条判据), 否则每条 `wait=True` 的用例都会拿到
        `settled=False`。桩不建模运动过程 (没有中间位姿), 只保证"收尾时 TCP 在目标上"
        —— 这也是固件的真实行为。要造"**没**走到目标"的形状 (轨迹被别的运动作废), 用
        `pos_override` 覆盖回读值。
        """
        if not self.cart_supported:
            # 固件关掉 LITEARM_CART_PLAN 时这 5 条整段不存在 -> default 分支
            self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x00])))
            return
        if len(payload) < _CART_MIN_LEN[cmd]:
            self._push(P.pack_frame(P.RSP_ERR, bytes([cmd, 0x01])))
            return
        if cmd == P.CMD_CART_ADD:                      # 记住最后一个路点 (RUN 的目标)
            self._path_last_pose = list(P.unpack_f32s(payload, 1, 6))
        self._push(_ack(cmd))                          # 受理即回 ACK (固件 cart_reply)
        if cmd in _CART_PLANNING_CMDS:
            if self.cart_err_override is None:         # 规划成功才会真的动
                self.pose = self._cart_goal(cmd, payload)
            # 起规划 -> 稍后一条 0x4E。桩立刻发 (固件要等 10ms~1s, 但配对语义相同)。
            self.cart_busy_seq = 0                     # 状态帧开始按写死的帧序带 bit10
            self._push(self._cart_plan_frame())

    def _cart_goal(self, cmd: int, payload: bytes) -> list:
        """该命令把 TCP 开到哪儿 (载荷里就是目标位置: `move_l` 前 6 f32 / `move_c` 的后
        6 f32 / `RUN` 取**最后一个 ADD**)。姿态部分桩不建模 rpy 规范化, 原样留着。"""
        if cmd == P.CMD_MOVE_L:
            return list(P.unpack_f32s(payload, 0, 6))
        if cmd == P.CMD_MOVE_C:
            return list(P.unpack_f32s(payload, 24, 6))
        return list(self._path_last_pose)              # 0x3E RUN

    def _cart_plan_frame(self) -> bytes:
        """`RSP_CART_PLAN (0x4E)` 响应帧: `ok u8 + err u8 + n_wp u16 LE + plan_us u32 LE`。

        规划失败**也要发**这一条 (固件契约) —— 只回 ACK 的话上位机会以为臂在动。
        """
        if self.cart_err_override is not None:
            # ok=0 时 n_wp / plan_us 一律 0 (固件同款: 失败时那两个数没有意义)
            return P.pack_frame(P.RSP_CART_PLAN,
                                bytes([0, self.cart_err_override]) + struct.pack("<HI", 0, 0))
        return P.pack_frame(P.RSP_CART_PLAN, bytes([1, 0]) + struct.pack(
            "<HI", self.cart_n_wp, self.cart_plan_us))

    def push_frame(self, cmd: int, payload: bytes = b"") -> None:
        """**注帧口**: 往响应队列里塞一帧 (不需要先有一条下行命令)。

        给"手工造一条上行帧"的用例用 (例: 往队列里放一条 `RSP_CART_PLAN` 再让读循环认领)。
        """
        self._push(P.pack_frame(cmd, payload))

    def read_frame(self, timeout: Optional[float] = None) -> Optional[Tuple[int, bytes]]:
        if self.dfu_gone and not self._resp:
            # ⚠ 队列**先投完**再抛: 现实里登记 ACK 必须在跳转之前出 USB
            # (固件为此专门有 `DFU_MIN_DELAY_TICKS`/`usb_cmd_reply_idle()` 两道门控),
            # 桩若先把 ACK 吞掉, SDK 那条 ACK 等待会超时 —— 形状就错了。
            raise TransportError("桩: 设备已进 ROM bootloader (CDC 上的读路径抛错)")
        if self._resp:
            fr = P.unpack_frame(self._resp.pop(0))
        elif self.auto_status:
            # 模拟 100Hz 主动状态流 (空闲帧) —— ⚠ 按 `auto_status_period` 节流：
            # 不节流的话读线程会以桩的最高速率空转 (理由见该字段的声明)。
            now = time.monotonic()
            if now - self._auto_status_last < self.auto_status_period:
                return None
            self._auto_status_last = now
            fr = P.unpack_frame(_status(self.q, mode=1, n=self.n))
        else:
            fr = None
        return self._stamp_status(fr)

    def _stamp_status(self, fr):
        """给**交付给 SDK 的**状态帧盖三样固件属性: 帧序号 `seq`、`enabled` (bit9)、
        `CART_BUSY` (bit10)。

        * `seq` (`g_arm.seq`, 头部偏移 2..4) **每交付一帧 +1** —— 到位判据的新鲜度闸
          判的就是它 ("这一帧是不是命令之后生成的")。不递增的话每帧都会被判成陈旧帧。
        * `CART_BUSY` 帧序**写死**: 收到 `0x4E` 之后的第 2 帧置 1、第 8 帧落 0
          (即第 2..7 帧为 1)。不写死会有两种坑: 常亮 -> 到位等待耗满 `move_timeout`
          才失败 (全量红); 从不置位 -> 只能走"没见过 1"的降级分支, 覆盖不到"先见 1"
          那条主判据。
        * `enabled` (bit9) —— 固件那一位报的就是 `g_arm.enabled`, 桩必须跟着自己的
          `self.enabled` 走。⚠ 这一位**不是**可选的: `Arm.enter_dfu()` 的本地使能态预检
          读的就是它 —— 桩不报的话预检恒不触发, 于是"使能中不许进 DFU"那条判据在离线
          用例里**测不到** (帧会一路发到桩的门禁, 变成 `ERR{0x15,0x03}`, 形状就错了)。

        三样都在**交付点** (而不是 `_push`) 盖: 这里盖的才是 SDK 真正看到的那一帧,
        无论它来自命令应答还是 `auto_status` 的空闲流。
        """
        if fr is None or fr[0] != P.RSP_STATUS:
            return fr
        self.status_seq = (self.status_seq + 1) & 0xFFFF
        c, payload = fr
        payload = payload[:2] + struct.pack("<H", self.status_seq) + payload[4:]
        flags = struct.unpack_from("<H", payload, 0)[0]
        if self.enabled:
            flags |= (1 << P.FLAG_ENABLED_BIT)
        else:
            flags &= ~(1 << P.FLAG_ENABLED_BIT)
        if self.cart_busy_seq is not None:
            self.cart_busy_seq += 1
            n = self.cart_busy_seq
            if n > 8:                  # 第 8 帧已落 0 -> 这次规划结束, 停止计数
                self.cart_busy_seq = None
            elif 2 <= n <= 7:
                flags |= (1 << 10)
        return (c, struct.pack("<H", flags) + payload[2:])

    def _push(self, frame: bytes) -> None:
        self._resp.append(frame)
