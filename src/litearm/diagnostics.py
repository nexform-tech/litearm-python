"""固件自检 —— `CMD_KIN_BENCH(0x49)` -> `RSP_KIN_BENCH(0x4A)` 文本回执的解析。

固件侧 (`kin_runner.c` 的 `kin_bench_run`, `:335-564`; LINK 行在 `:514`) 在后台用 DWT
cycle 测各热点单拍开销 (禁中断测量,
期间 `g_kin_bench_active` 置位以豁免主循环心跳监督; 最长窗口 5s), 结果拼成多行文本
经可靠应答通道回。**最有价值的产出是最后一行 LINK** —— 它承载了固件里唯一的链路
诊断计数 (CRC 坏帧 / RX 环溢出 / 应答 FIFO 丢弃 / CAN TX 失败分类桶 / 控制拍峰值与
超预算拍数 / 锁存拍与原因), 这些在固件里原本全部静默 (L3/M1 fix)。

⚠ `0x49` 是双 ID: 下行 `CMD_KIN_BENCH` 与上行 `RSP_JOINT_PARAM` 同值, 靠收发方向区分。
⚠ 该命令**不检查 payload 长度**, 发出去必产生一次真实测量 —— 故仅用于显式调用,
不做连接期探测。
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Dict, Tuple

from litearm import _protocol as P
from litearm.errors import MotionTimeoutError, TransportError

if TYPE_CHECKING:                       # 避免与 arm 循环导入
    from litearm.arm import Arm, Msg

__all__ = ["KinBenchResult", "Diagnostics"]

#: 固件 bench 最长窗口 KIN_BENCH_MAX_TICKS = 5s (`control_loop.c`), 留 1.6 倍余量
KIN_BENCH_TIMEOUT = 8.0

#: 固件把 KIN_BENCH 回执拆成**连续两帧**发 (`[H-9 fix 2026-09-13]`) ——
#: 第 1 帧 = 各项耗时, 第 2 帧 = **LINK 诊断行**。
#:
#: 拆帧的理由是单帧载荷上限 255B (`USB_MAX_PAYLOAD`), 而全文实测 ~306B: 原单帧发出会
#: 在 253B 处硬截断, **截断点恰好落在 LINK 行的 `loop_k=` 之后** ⇒ `loop_k`/`ovr`/
#: `flt`/`cause`/`st` 一个都读不到 (`kin_runner.c:497-505` 原文)。
#:
#: ⚠⚠ **只收一帧必然丢掉全部链路计数, 而丢了它不会报错** —— `KinBenchResult` 的五个
#: 便捷出口 (`crc_errors` / `reply_dropped` / `can_tx_fail` / `loop_max_kcycle` /
#: `loop_overruns`) 全是 `link.get(key, 0)`, 拿不到就是**静默的 0**, 与"没有出错"分不开。
#: 本包迁移时漏了这个双帧契约, 于是把固件刚修好的 H-9 盲区又引入了一次
#: (真机实证: 回执里没有 LINK 行, `link == {}`, 五个出口全报 0)。
_KIN_BENCH_FRAMES = 2

#: 第 2 帧的等待窗口。**新固件它紧跟第 1 帧到; 旧固件根本不发** ⇒ 靠超时退出,
#: 与收窄前的行为兼容 (旧工具 `tools/kin_bench.py` 同款口径: "收满两帧再拼接;
#: 旧固件只会来一帧, 循环靠超时退出")。
#: ⚠ 代价: 在**只发一帧的旧固件**上, 每次 `kin_bench()` 会多等这半个秒。
#:   换来的是"新固件的链路计数真的读得到" —— 那五个数正是本命令最实用的产出。
_KIN_BENCH_MORE_S = 0.5

#: 固件 `kin_bench_run` 产出的计时段名 (长名在前, 避免前缀误配)。
#: ⚠ 固件用 `txt(名字)` + `u32_cat(n)` 拼接, 而 `u32_cat` 只在**数字之后**补空格 ——
#: 所以名字与第一个数字是**相连**的 (`FK200 108 240 ` / `LOOP14000 `), 不能用
#: 「名字 + 空白 + 数字」式正则去切。
_BLOCK_NAMES = ("LOOP1", "RNEA", "JAC", "IK", "FK", "SC", "G", "M", "LAW")
_TIMING_RE = re.compile(r"^(%s)([\d\s]+)$" % "|".join(_BLOCK_NAMES))
#: `flt=<锁存拍> /<原因码>` —— 数字与 '/' 之间有一个尾随空格 (u32_cat 语义所致)
_FLT_RE = re.compile(r"flt=(\d+)\s*/\s*(\d+)")
#: `key=value` 对。
#: ⚠⚠ 键名**可以带数字** —— `rxl0` / `rxl1` 就是 (它们正是"RX FIFO 溢出 = 静默丢反馈
#: = 可能整臂 EMERGENCY"的那两个计数器, 见固件 `kin_runner.c:520-522`)。
#: 从前写的是 `[a-z_]+`, **匹配不到带数字的键**, 于是那两个静默丢失。
#: ⚠ 这是**同一个文件里第二次**栽在"名字里带数字"上 —— `_TIMING_RE` 上面那段注释
#: 早就写过 `LOOP1` 行是 `LOOP14000`, "名字里本身带数字, 故…不能用通用正则";
#: 而同格式的 `_KV_RE` 还是写窄了。
_KV_RE = re.compile(r"([a-z_][a-z0-9_]*)=(\d+)")


@dataclass(frozen=True)
class KinBenchResult:
    """KIN_BENCH 回执。`raw` 是固件原文 (逐行), `timings`/`link` 是解析结果。

    `timings`: 块名 -> 数字元组。`FK/JAC/SC/G/RNEA/M/LAW` 是 `(n, avg_cyc, max_cyc)`,
    `IK` 是 `(n_try, min_cyc, max_cyc)`, `LOOP1` 是 `(cycles,)`。
    `link`: 链路诊断计数 (见模块 docstring)。
    """

    raw: str
    timings: Dict[str, Tuple[int, ...]] = field(default_factory=dict)
    link: Dict[str, int] = field(default_factory=dict)

    def __str__(self) -> str:
        return self.raw

    # ---- 常用诊断的便捷出口 ----
    @property
    def crc_errors(self) -> int:
        """固件累计 CRC 坏帧 (上位机发出的帧被固件丢弃)。"""
        return self.link.get("crc", 0)

    @property
    def reply_dropped(self) -> int:
        """固件应答 FIFO 队满丢弃数 (上位机可能等不到某条应答)。"""
        return self.link.get("rfd", 0)

    @property
    def can_tx_fail(self) -> int:
        """CAN 发送失败累计。"""
        return self.link.get("txf", 0)

    @property
    def loop_max_kcycle(self) -> int:
        """控制拍峰值耗时 (千 cycle)。"""
        return self.link.get("loop_k", 0)

    @property
    def loop_overruns(self) -> int:
        """超出周期预算的控制拍数。"""
        return self.link.get("ovr", 0)

    # ---- 下面三个是固件自己点名的"静默丢反馈"项 (`kin_runner.c:520-524`) ----
    # `[N-12 fix] RX FIFO 溢出 (原为零遥测): 溢出 = 静默丢反馈 = 可能整臂 EMERGENCY`
    # ⚠⚠ 它们的键名**带数字** (`rxl0`/`rxl1`) —— 解析器从前吃不下, 于是这三个出口
    #    恒返回 0。**"读不到"与"没丢过"在本类里长得一模一样**, 见模块里 `_KV_RE` 的注释。

    @property
    def rx_fifo_lost_motor(self) -> int:
        """**电机域** CAN FIFO0 溢出计数 (丢的是电机反馈)。"""
        return self.link.get("rxl0", 0)

    @property
    def rx_fifo_lost_bridge(self) -> int:
        """**gs_usb 桥** CAN FIFO1 溢出计数。"""
        return self.link.get("rxl1", 0)

    @property
    def gsusb_ring_drops(self) -> int:
        """gs_usb 桥环满丢帧数。"""
        return self.link.get("rbd", 0)


def parse_kin_bench(text: str) -> KinBenchResult:
    """解析固件 KIN_BENCH 文本; 空文本视为异常回执。"""
    if not text.strip():
        raise TransportError("KIN_BENCH 回执为空 —— 非正常响应, 拒绝当成全 0 返回")
    timings: Dict[str, Tuple[int, ...]] = {}
    link: Dict[str, int] = {}
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        if line.startswith("LINK"):
            body = line[len("LINK"):]
            m = _FLT_RE.search(body)
            if m:
                link["fault_tick"] = int(m.group(1))
                link["fault_cause"] = int(m.group(2))
                body = body[:m.start()] + body[m.end():]
            for k, v in _KV_RE.findall(body):
                if k == "st":
                    link["store_from_flash"] = bool(int(v))
                else:
                    link[k] = int(v)
            continue
        m = _TIMING_RE.match(line)
        if m:
            timings[m.group(1)] = tuple(int(x) for x in m.group(2).split())
    return KinBenchResult(raw=text, timings=timings, link=link)


class Diagnostics:
    """`arm.diag` —— 固件自检入口。"""

    def __init__(self, arm: "Arm"):
        self._arm = arm

    def kin_bench(self, timeout: float = KIN_BENCH_TIMEOUT) -> "Msg[KinBenchResult]":
        """触发固件运动学/动力学单拍开销自测, 返回解析后的回执。

        固件侧最长窗口 5s (期间控制循环心跳监督豁免), 默认超时 8s。
        ⚠ 该命令不检查载荷长度, 发出即产生一次真实测量。

        ⚠⚠ **回执是连续两帧, 本方法两帧都收** —— 见 `_KIN_BENCH_FRAMES` 的说明。
        只收第 1 帧会**静默**丢掉全部链路计数 (那五个 `link.get(k, 0)` 出口会全报 0,
        与"没有出错"分不开)。旧固件只发一帧, 靠短窗口超时退出, 行为兼容。

        返回 :class:`Msg` 信封 (帧 id `RSP_KIN_BENCH`); 单发请求/应答式, 见 `Msg`。
        """
        arm = self._arm
        arm._write_query(P.CMD_KIN_BENCH)               # 查询/自检类: 不受零重力守卫
        # 第 1 帧 (各项耗时) —— 等满 `timeout`
        _, p = arm._require().expect(P.RSP_KIN_BENCH, timeout, "kin_bench",
                                         echo_cmd=P.CMD_KIN_BENCH)
        parts = [p.decode(errors="replace")]
        # 第 2 帧 (LINK 诊断行) —— 只等一个短窗口: 新固件它紧跟第 1 帧到, 旧固件不发。
        # ⚠ 超时**不是错误** (旧固件只有一帧, 那是兼容情形); 但也**不能**因为
        #   "拿不到就算了"而把窗口设成 0 —— 那等于没改, 又回到静默 0。
        while len(parts) < _KIN_BENCH_FRAMES:
            try:
                _, p = arm._require().expect(P.RSP_KIN_BENCH, _KIN_BENCH_MORE_S,
                                             "kin_bench(LINK 帧)", raise_on_err=False,
                                             echo_cmd=P.CMD_KIN_BENCH)
            except (MotionTimeoutError, TransportError):
                break
            parts.append(p.decode(errors="replace"))
        return arm._msg(parse_kin_bench("".join(parts)), P.RSP_KIN_BENCH)
