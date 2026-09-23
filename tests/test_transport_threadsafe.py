"""SerialTransport 的并发安全 —— 加保活线程之前必须先钉住的两个契约。

背景: `Arm.zero_g()` 的保活线程会是本 SDK **第一个并发写入者**。原 transport
类 docstring 写着「读写同一锁」, 但文件里没有任何 Lock 对象。并发上线后会出现:
  - 保活线程的写与主线程的写交错 -> 帧体互相插入 -> 上位机侧 CRC 全坏;
  - 若用「单一 RLock 罩住读写」图省事 -> 主线程一次最长 80ms 的阻塞读会把
    50ms 周期的保活写拖过期 -> 零重力被看门狗掐掉。
故设计取**读写各一把锁**: 读者之间互斥 (保护 _buf/_partial_since/_ser.timeout),
写者之间互斥 (保护帧完整性), 读写之间**不互斥** (pyserial 的 os.read/os.write
并发本就安全, 且 _ser.timeout 只被读者改)。
"""
from __future__ import annotations

import struct
import sys
import threading
import time
import types

from litearm import _protocol as P
from litearm.transport import SerialTransport


class _RaceSerial:
    """模拟真实串口: write 非原子 (分两段 + 中间释放 GIL), read 可被拖慢。"""

    def __init__(self, port, baudrate=0, timeout=0.2, **kw):
        self.is_open = True
        self.timeout = timeout
        self.sink = bytearray()          # 上位机"发出去"的字节
        self.rx = bytearray()            # 待上位机读的字节
        self.read_delay = 0.0

    def write(self, d):
        h = len(d) // 2
        self.sink += d[:h]
        time.sleep(0.0005)               # 抢占点: 无锁时另一线程的字节会插进来
        self.sink += d[h:]
        return len(d)

    def read(self, size=1):
        if self.read_delay:
            time.sleep(self.read_delay)
        if not self.rx:
            return b""
        out = bytes(self.rx[:size])
        del self.rx[:size]
        return out

    def flush(self):
        pass

    def close(self):
        self.is_open = False


def _install(monkeypatch, read_delay=0.0):
    holder = {}

    class _F(_RaceSerial):
        def __init__(self, *a, **kw):
            super().__init__(*a, **kw)
            self.read_delay = read_delay
            holder["ser"] = self

    monkeypatch.setitem(sys.modules, "serial", types.SimpleNamespace(Serial=_F))
    return holder


def _scan(buf: bytes):
    """按 SOF 扫描出全部**完整且 CRC 正确**的帧; 坏帧被丢弃后继续重扫。"""
    out, i = [], 0
    while i + 5 <= len(buf):
        if buf[i] != P.SOF:
            i += 1
            continue
        ln = buf[i + 2]
        total = 3 + ln + 2
        if i + total > len(buf):
            break
        got = P.unpack_frame(buf[i:i + total])
        if got is not None:
            out.append(got)
        i += 1
    return out


def test_concurrent_writers_never_interleave_a_frame(monkeypatch):
    """四个线程各写 40 帧 -> 静置区必须恰好 160 个完整帧, 无一帧被拆散。

    无锁时 write 的抢占点会让两帧字节交错, 扫描出的完整帧数远少于 160
    (交错帧 CRC 全坏, 只能被丢弃)。
    """
    holder = _install(monkeypatch)
    tr = SerialTransport("/dev/null")
    n_threads, n_each = 4, 40

    def worker(tid: int):
        for i in range(n_each):
            tr.write_frame(P.CMD_MOVE_MIT, struct.pack("<ff", float(tid), float(i)))

    ths = [threading.Thread(target=worker, args=(t,)) for t in range(n_threads)]
    for t in ths:
        t.start()
    for t in ths:
        t.join()

    frames = _scan(bytes(holder["ser"].sink))
    assert len(frames) == n_threads * n_each, \
        f"只解出 {len(frames)} 个完整帧 (期望 {n_threads * n_each}) —— 写帧被交错拆散了"
    for cmd, payload in frames:
        assert cmd == P.CMD_MOVE_MIT
        tid, i = struct.unpack("<ff", payload)
        assert 0.0 <= tid < n_threads and 0.0 <= i < n_each


def test_in_progress_read_does_not_block_write(monkeypatch):
    """在途的阻塞读不得拖住写 —— 保活写必须能按 50ms 周期插进去。

    ⚠ 这是**设计约束守卫**, 不是缺陷复现: 基线(无锁)本就通过。它的价值在于
    挡住「用一把 RLock 罩住读写」的省事改法 —— 那样写要等读的 0.30s 锁释放,
    保活会稳定错过 0.1s 看门狗窗口。
    """
    holder = _install(monkeypatch, read_delay=0.30)
    tr = SerialTransport("/dev/null")
    holder["ser"].rx += b""            # 无数据: 读会一直阻塞到超时

    started = threading.Event()

    def slow_read():
        started.set()
        tr.read_frame(0.30)

    th = threading.Thread(target=slow_read)
    th.start()
    started.wait(0.5)
    time.sleep(0.05)                   # 让读真正进入阻塞

    t0 = time.monotonic()
    tr.write_frame(P.CMD_ZERO_G, b"\x01")
    dt = time.monotonic() - t0
    th.join()

    assert dt < 0.15, (
        f"一次写花了 {dt:.3f}s —— 被在途阻塞读挡住了; "
        f"保活写会被拖过 50ms 周期, 零重力将被 0.1s 看门狗掐掉")
