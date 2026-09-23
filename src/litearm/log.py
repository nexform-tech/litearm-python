"""300Hz 控制拍采集 —— 固件 `CMD_LOG_CTRL(0x2D)` / `CMD_LOG_READ(0x2E)` 的封装。

固件侧 (`hal/log_capture.h`): 跑激励轨迹时把每控制拍 (tick / 执行参考 q_ref /
执行速度 dq / 电机实测力矩 tau) 追加进 RAM 缓冲, `LOG_MAX_SAMPLES=2400` (≈8s@300Hz)
记满自停; 上位机用 `LOG_READ` 按字节游标分块读回 —— 这条"绕开连续 USB 流、按游标
续读"的设计正是为规避 CDC 间歇掉帧而做的, 所以本模块的重试/续读语义是**必须**的,
不是可选优化。

样本布局 (固件 `log_sample_t`): `u32 tick + q_ref[N] + dq[N] + tau[N]`, 即 `4 + 12N` 字节。
"""
from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Iterator, List, Optional

from litearm import _protocol as P
from litearm.errors import (
    InvalidCommandError,
    MotionTimeoutError,
    TransportError,
)

if TYPE_CHECKING:                       # 避免与 arm 循环导入
    from litearm.arm import Arm

__all__ = ["LogSample", "LogReader", "ArmLog", "sample_size"]

#: 固件 `LOG_MAX_SAMPLES` (hal/log_capture.h) —— 超过会回 ERR{0x2D,0x02}
LOG_MAX_SAMPLES = 2400
#: 固件单块读回上限 `LOG_READ_CHUNK` (usb_cmd.c) —— 9B 头 + n ≤ 245
LOG_READ_CHUNK = 245
#: 单块 RSP_LOG_DATA 的头长度: u32 total + u32 next + u8 n
_LOG_HDR = 9
#: 固件控制循环频率 (`litearm.h` 的 `LITEARM_CTRL_HZ`) —— 采集是**逐拍**记录的,
#: 故 n 拍需要 n/HZ 秒才录满。用于推算默认等待预算。
CTRL_HZ = 300
#: 轮询已记录量的间隔
_POLL_S = 0.05


def sample_size(n_joints: int) -> int:
    """单拍字节数 = `sizeof(u32) + 3 * N * sizeof(f32)`。"""
    return 4 + 12 * int(n_joints)


@dataclass(frozen=True)
class LogSample:
    """一拍采样 (固件 `log_sample_t`)。`q_ref`/`dq`/`tau` 为 N 元组。"""

    tick: int
    q_ref: tuple
    dq: tuple
    tau: tuple


def parse_samples(blob: bytes, n_joints: int) -> List[LogSample]:
    """把原始字节流解析成样本列表; 不是样本整数倍则报错。"""
    sz = sample_size(n_joints)
    if sz <= 0:
        raise InvalidCommandError(f"关节数非法: {n_joints}")
    if len(blob) % sz:
        raise TransportError(
            f"采集字节数 {len(blob)} 不是单拍 {sz}B 的整数倍 —— 数据截断/错位, "
            f"拒绝解析出半截样本")
    out = []
    for off in range(0, len(blob), sz):
        tick = struct.unpack_from("<I", blob, off)[0]
        q_ref = tuple(P.unpack_f32s(blob, off + 4, n_joints))
        dq = tuple(P.unpack_f32s(blob, off + 4 + 4 * n_joints, n_joints))
        tau = tuple(P.unpack_f32s(blob, off + 4 + 8 * n_joints, n_joints))
        out.append(LogSample(tick, q_ref, dq, tau))
    return out


class LogReader:
    """按固件游标 (`next_byte`) 分块读回采集缓冲。

    `next_byte == 0` 表示读完; 掉帧 (应答整块丢失) 时按当前游标重试, 不会丢一段数据。
    """

    def __init__(self, arm: "Arm", timeout: float = 1.0, retries: int = 3):
        self._arm = arm
        self.timeout = float(timeout)
        self.retries = int(retries)
        #: 最近一次读回的固件总字节数 (读完后可查)
        self.total_bytes = 0
        self._next = 0

    def _read_chunk(self, offset: int) -> bytes:
        last: Optional[Exception] = None
        for _ in range(self.retries + 1):
            try:
                self._arm._write_query(P.CMD_LOG_READ, struct.pack("<I", offset))
                _, p = self._arm._require().expect(
                    P.RSP_LOG_DATA, self.timeout, f"log_read@{offset}",
                    echo_cmd=P.CMD_LOG_READ)
            except MotionTimeoutError as e:         # 掉帧: 同一游标重试
                last = e
                continue
            if len(p) < _LOG_HDR:
                raise TransportError(f"RSP_LOG_DATA 帧短 ({len(p)}B)")
            total, nxt, n = struct.unpack_from("<IIB", p, 0)
            if n > LOG_READ_CHUNK or len(p) < _LOG_HDR + n:
                raise TransportError(f"RSP_LOG_DATA 载荷异常: n={n}, 实到 {len(p) - _LOG_HDR}B")
            self.total_bytes = int(total)
            self._next = int(nxt)
            return bytes(p[_LOG_HDR:_LOG_HDR + n])
        raise last if last is not None else TransportError("log_read 失败")

    def total(self) -> int:
        """查询固件当前**已记录**的字节数 (一次 `LOG_READ` 往返, 不保存数据)。

        采集是 300Hz 逐拍进行的, `start(n)` 返回时缓冲里还没有数据 —— 等待录满
        必须靠轮询本方法, 不能靠 sleep 猜时间。
        """
        self._read_chunk(0)
        return self.total_bytes

    def wait_for(self, n_ticks: int, timeout: Optional[float] = None,
                 poll: float = _POLL_S) -> int:
        """等固件记满 `n_ticks` 拍; 返回实际总字节数。

        `timeout=None` 时按 `n_ticks / CTRL_HZ * 1.5 + 3s` 推算 (300Hz 逐拍记录)。
        超时抛 `MotionTimeoutError` 并报出实际录到多少拍 —— 绝不返回半截数据冒充成功。
        """
        n = int(n_ticks)
        sz = sample_size(self._arm.n)
        want = n * sz
        if want <= 0:
            return 0
        if timeout is None:
            timeout = n / float(CTRL_HZ) * 1.5 + 3.0
        end = time.monotonic() + float(timeout)
        while True:
            total = self.total()
            if total >= want:
                return total
            if time.monotonic() >= end:
                raise MotionTimeoutError(
                    f"采集未在 {float(timeout):.1f}s 内录满 {n} 拍 "
                    f"(只录到 {total // sz} 拍 / {total}B) —— 可调大 record_timeout")
            time.sleep(poll)

    def iter_chunks(self) -> Iterator[bytes]:
        """逐块产出原始字节; 读到 `next_byte == 0` 结束。

        ⚠ 必须自检**游标是否前进**: 固件游标若因异常/干扰回了一个不大于当前 offset
        的非零值, 天真的实现会在同一位置无限次重读 (死循环)。这里直接报错。
        """
        off = 0
        while True:
            chunk = self._read_chunk(off)
            if chunk:
                yield chunk
            if self._next == 0:
                return
            if self._next <= off:
                raise TransportError(
                    f"LOG_READ 游标未前进 (offset={off} -> next_byte={self._next}), "
                    f"拒绝在同一游标上重复读回 (死循环保护)")
            off = self._next

    def read_all(self) -> bytes:
        return b"".join(self.iter_chunks())

    def samples(self) -> List[LogSample]:
        """读回并解析为样本列表 (便捷; 大缓冲建议先 `dump()` 落盘)。"""
        return parse_samples(self.read_all(), self._arm.n)


class ArmLog:
    """`arm.log` —— 采集启停 + 读回入口。"""

    def __init__(self, arm: "Arm"):
        self._arm = arm
        #: 最近一次 `start(n)` 的 n —— `dump()` 靠它知道该等录满多少拍
        self._last_target = 0

    def start(self, n_ticks: int) -> None:
        """开始记录 `n_ticks` 拍后自停 (n=0 等效 `stop()`)。

        ⚠ 固件是**逐拍**记录的: 本调用返回时缓冲里还没有数据 (300Hz, n 拍要
        n/300 秒)。要拿完整数据须用 `capture()` (自动等) 或 `dump()` (默认等)。

        固件侧 `n > LOG_MAX_SAMPLES(2400)` 回 `ERR{0x2D,0x02}`。
        """
        n = int(n_ticks)
        if n < 0:
            raise InvalidCommandError(f"n_ticks 需 >=0 (给的是 {n_ticks})")
        self._arm._cmd(P.CMD_LOG_CTRL, struct.pack("<I", n), "log_start")
        self._last_target = n

    def stop(self) -> None:
        """停止/清空记录 (停止后仍可用 `reader()` 读回已记录部分)。"""
        self._arm._cmd(P.CMD_LOG_CTRL, struct.pack("<I", 0), "log_stop")
        self._last_target = 0

    def reader(self, timeout: float = 1.0, retries: int = 3) -> LogReader:
        return LogReader(self._arm, timeout=timeout, retries=retries)

    def capture(self, n_ticks: int, timeout: float = 1.0, retries: int = 3,
                record_timeout: Optional[float] = None) -> List[LogSample]:
        """便捷: 采 `n_ticks` 拍 -> **等录满** -> 读回 -> 解析成样本列表。

        ⚠ 固件是 300Hz **逐拍**记录的 —— `start(n)` 返回时缓冲里还没有数据,
        `n_ticks=600` 要约 2 秒才录满, 故本方法会轮询等待 (`record_timeout=None`
        时按 `n/HZ*1.5+3s` 推算)。等不满则抛错, 不返回半截数据。

        ⚠ 读回是 ~`n_ticks*(4+12N)/245` 次往返 (7J 满量程 2400 拍 ≈ 863 次),
        大缓冲建议用 `dump()` 落盘后离线解析。`timeout` 是**单块**超时。
        """
        self.start(n_ticks)
        reader = self.reader(timeout=timeout, retries=retries)
        reader.wait_for(n_ticks, timeout=record_timeout)
        return reader.samples()

    def dump(self, path: str, wait: bool = True, timeout: float = 1.0,
             retries: int = 3, record_timeout: Optional[float] = None) -> int:
        """读回原始字节流落盘 (解析交给调用方 / `parse_samples`)。返回字节数。

        `wait=True`(默认) 会先等最近一次 `start(n)` 的 n 拍录满 —— 否则 `start()`
        之后立刻 dump 会落盘一个空/半截文件。要读"此刻已录到多少"就传
        `wait=False`。
        """
        reader = self.reader(timeout=timeout, retries=retries)
        if wait and self._last_target > 0:
            reader.wait_for(self._last_target, timeout=record_timeout)
        blob = reader.read_all()
        with open(path, "wb") as f:
            f.write(blob)
        return len(blob)
