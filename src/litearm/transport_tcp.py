"""TCP 传输 —— 与串口**同一套帧**, 只是载体从 USB CDC 换成 socket。

用途只有一个: 把 `litearm-python` 接到一个"固件替身"上 (仿真的 MuJoCo 后端, 或任何
在别处实现了本协议的东西), 命令行与真机逐字节相同 —— 于是同一份采集/策略脚本
不用改一行, 只是把端口从 `/dev/ttyACM0` 换成 `tcp://127.0.0.1:5601`。

**它不是"另一个后端"**: 帧的语义、时序假设、错误分类全部与 :mod:`litearm.transport`
一致。下面每一条都是 :class:`~litearm.transport.SerialTransport` 的契约照抄, 因为它们
是 `Arm` 层的承重假设, 不是串口特有的实现细节:

* :attr:`port` **原样保存 URL 字符串** —— `Arm.connect()` 的幂等比较读的就是它
  (`arm.py` 里 `port == getattr(self._tr, "port", None)`)。存成 `(host, port)` 元组会让
  重复 `connect()` 每次都被判成"换了目标", 把在途笛卡尔记账与零重力保活推倒重来。
* :meth:`read_frame` 超时返回 ``None``, **半帧留在缓冲里**不丢; 且
  ``timeout=0`` 是"有就给我", 不是"一个字节都不碰" (否则 `Arm.poll_cart()` 恒 ``None``)。
* :meth:`write_frame` 的**写失败必须抛** :class:`~litearm.errors.TransportError` ——
  `_CartPending` 靠"抛了 ⟹ 整帧未送达"这条不变量摘 token, 破了就是**假成功**。
* :meth:`close` 幂等; 对端关闭 (recv 得到 ``b""``) 要**报链路断**, 不能静默返回 ``None``
  —— 否则上层只能靠超时发现, 归因会指向"固件不理我"。

⚠ 零依赖: 只用标准库 ``socket``/``select``。本包的唯一依赖仍是 pyserial。
"""
from __future__ import annotations

import select
import socket
import threading
import time
from typing import Optional, Tuple

from litearm import _protocol as P
from litearm.errors import TransportError

#: 残缺帧 (SOF 起算但未收齐) 最多等多久。与 `transport._PARTIAL_MAX_S` 同值同理由:
#: 噪声里凑出的假帧头永远收不齐, 不设上限就会把后面的真帧永久挡住。
_PARTIAL_MAX_S = 0.25

#: 读超时之后那段"非阻塞补读"的时间上限 (秒)。照抄 `transport._NONBLOCK_EXTRA_S`:
#: `timeout=0` 时要把**已经到达**的字节读进来, 但数据持续到达时不能变成无界循环。
_NONBLOCK_EXTRA_S = 0.02

#: 一次 recv 的字节数。串口那边按 `in_waiting` 要, socket 这边一次给多少都行 ——
#: 装配循环本来就按"来多少字节都可能"写的。
_RECV_MAX = 4096

#: 连接超时。真机上是 `serial.Serial()` 打开设备, 这边是 TCP 三次握手。
_CONNECT_TIMEOUT_S = 2.0

#: 写超时 —— **必须有界**。`emergency_stop()` 必须永远可达 (见 SerialTransport 的同名注释)。
_WRITE_TIMEOUT_S = 0.5

_SCHEMES = ("tcp://", "sim://")


def is_tcp_url(port: object) -> bool:
    """`port` 是不是一个"要走 TCP 的目标"。

    认三种写法: `tcp://host:port`、`sim://host:port`、以及裸的 `host:port`
    (只含一个冒号、冒号后是纯数字、不含路径分隔符 —— 这样 `/dev/ttyACM0`
    与 `COM3` 都不会被误判)。
    """
    if not isinstance(port, str) or not port:
        return False
    if port.startswith(_SCHEMES):
        return True
    if "/" in port or "\\" in port or port.count(":") != 1:
        return False
    host, _, portnum = port.partition(":")
    if not host or not portnum.isdigit():
        return False
    # ⚠ 必须查范围: 光看 isdigit 会把 "host:70000" 判成 TCP 目标, 然后在
    # connect 时以一个和"端口写错"毫不相干的错误炸掉。
    return 0 < int(portnum) < 65536


def parse_tcp_url(port: str) -> Tuple[str, int]:
    """`tcp://host:port` / `host:port` -> `(host, port)`。非法时抛 ValueError。"""
    s = port
    for scheme in _SCHEMES:
        if s.startswith(scheme):
            s = s[len(scheme):]
            break
    host, sep, portnum = s.rpartition(":")
    if not sep or not host:
        raise ValueError(f"TCP 目标应形如 tcp://host:port (收到 {port!r})")
    try:
        n = int(portnum)
    except ValueError:
        raise ValueError(f"端口号不是整数: {portnum!r} (来自 {port!r})") from None
    if not 0 < n < 65536:
        raise ValueError(f"端口号越界: {n} (来自 {port!r})")
    return host, n


class TcpTransport:
    """帧读写传输 (TCP 版)。接口与行为契约见模块 docstring。

    **读写各一把锁, 读写之间不互斥** —— 与 :class:`~litearm.transport.SerialTransport`
    同结构: 读者互斥 `_buf`/`_partial_since`, 写者互斥"一帧的字节原子交给 socket";
    读写之间并发本来就安全, 而用单把锁罩住会让一次阻塞读把保活写拖过 0.1s 看门狗窗口。
    """

    TEXT_LOG_MAX = 2048

    def __init__(self, port: str, timeout: float = 0.2,
                 connect_timeout: float = _CONNECT_TIMEOUT_S):
        self.port = port                      # ⚠ 原样保存, 见模块 docstring
        self.timeout = timeout
        host, portnum = parse_tcp_url(port) if isinstance(port, str) else (None, None)
        if host is None:
            raise TransportError(f"不是合法的 TCP 目标: {port!r}")
        try:
            self._sock = socket.create_connection((host, portnum),
                                                  timeout=connect_timeout)
        except OSError as e:
            raise TransportError(f"连接 {port} 失败: {e}") from e
        try:
            self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass                              # 无 Nagle 更好; 设不上不致命
        self._sock.settimeout(None)           # 一律用 select 控阻塞, 不用 socket 超时
        self._buf = bytearray()
        self._partial_since: Optional[float] = None
        self._text = bytearray()
        self._rlock = threading.RLock()
        self._wlock = threading.RLock()
        self._closed = False
        self._peer_closed = False
        self.flush_failures = 0               # 与串口同名同义 (socket 无 flush, 恒 0)

    # -- 状态 ---------------------------------------------------------------
    @property
    def text_log(self) -> str:
        """最近被跳过的非帧字节 —— 与串口同语义 (开机签名在那边靠它解析)。"""
        with self._rlock:
            return bytes(self._text).decode("utf-8", errors="replace")

    @property
    def is_open(self) -> bool:
        if self._closed:
            return False
        try:
            return not self._peer_closed
        except Exception:                     # noqa: BLE001
            return False

    def close(self) -> None:
        """幂等关闭。**先 shutdown 再 close** —— 只 close 会让对端读到一个
        半关闭的连接而继续写, 从这里看就是"关了还在收"。"""
        with self._rlock, self._wlock:
            if self._closed:
                return
            self._closed = True
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                self._sock.close()
            except OSError:
                pass

    # -- 写 -----------------------------------------------------------------
    def write_frame(self, cmd: int, payload: bytes = b"") -> None:
        """写一帧。失败**必须抛** `TransportError` (整帧未送达的判据, 见模块 docstring)。"""
        frame = P.pack_frame(cmd, payload)
        with self._wlock:
            if self._closed or self._peer_closed:
                raise TransportError(f"写失败: {self.port} 已关闭")
            try:
                self._sock.settimeout(_WRITE_TIMEOUT_S)
                self._sock.sendall(frame)
            except OSError as e:
                raise TransportError(f"写失败: {e}") from e
            finally:
                try:
                    self._sock.settimeout(None)
                except OSError:
                    pass

    # -- 读 -----------------------------------------------------------------
    def read_frame(self, timeout: Optional[float] = None) -> Optional[Tuple[int, bytes]]:
        """读一帧; 超时返回 None。跳过噪声字节, 丢弃 CRC 坏帧 (与串口逐字同义)。"""
        end = time.monotonic() + (self.timeout if timeout is None else timeout)
        with self._rlock:
            return self._read_frame_locked(end)

    def _read_frame_locked(self, end: float) -> Optional[Tuple[int, bytes]]:
        nb_end: Optional[float] = None
        while True:
            while self._buf:
                if self._buf[0] != P.SOF:
                    self._note_text(self._buf[0])
                    del self._buf[0]
                    continue
                if len(self._buf) < 3:
                    break
                ln = self._buf[2]
                total = 3 + ln + 2
                if len(self._buf) < total:
                    break
                got = P.unpack_frame(bytes(self._buf[:total]))
                if got is not None:
                    del self._buf[:total]
                    self._partial_since = None
                    return got
                del self._buf[0]
            now = time.monotonic()
            if self._buf and self._buf[0] == P.SOF:
                if self._partial_since is None:
                    self._partial_since = now
                elif now - self._partial_since > _PARTIAL_MAX_S:
                    del self._buf[0]
                    self._partial_since = None
                    continue
            else:
                self._partial_since = None
            if now >= end:
                # 窗口用尽, 但仍要把**已经到达**的字节读进来 (timeout=0 的语义)
                if nb_end is None:
                    nb_end = now + _NONBLOCK_EXTRA_S
                elif now > nb_end:
                    return None
                chunk = self._recv(0.0)
                if chunk:
                    self._buf += chunk
                    continue
                return None
            chunk = self._recv(end - now)
            if chunk:
                self._buf += chunk

    def _recv(self, wait: float) -> Optional[bytes]:
        """等至多 `wait` 秒取一块。对端关闭时抛 `TransportError` (链路断, 不静默)。"""
        if self._closed:
            raise TransportError("读失败: 链路已关闭")
        try:
            ready, _, _ = select.select([self._sock], [], [], max(wait, 0.0))
        except (OSError, ValueError) as e:
            raise TransportError(f"读失败: {e}") from e
        if not ready:
            return None
        try:
            data = self._sock.recv(_RECV_MAX)
        except OSError as e:
            raise TransportError(f"读失败: {e}") from e
        if not data:
            self._peer_closed = True
            raise TransportError(f"读失败: {self.port} 对端关闭了连接")
        return data

    def _note_text(self, byte: int) -> None:
        self._text.append(byte)
        if len(self._text) > self.TEXT_LOG_MAX:
            del self._text[:len(self._text) - self.TEXT_LOG_MAX]
