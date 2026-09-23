"""串口传输 —— USB CDC 帧读写 (含自动发现/自愈)。

**端点独占是两层的, 缺一层都不够** —— 实测 (E5): Linux 下 CDC 口**不**独占, 第二个进程
`open` **成功**且**分吃同一字节流** (双方拿残帧、静默失败); Windows 反而由 OS 强制独占
⇒ 独占性因平台而异, 不能当常量依赖。固件那侧无从提供 (`CDC_SET_CONTROL_LINE_STATE`
是空 break), 只能主机侧收口:

* **跨进程**: `serial.Serial(..., exclusive=True)` —— posix 侧是内核的
  `flock(LOCK_EX|LOCK_NB)` (它比的是 open file description, 故**同进程内两次 open 也会
  互斥**), 失败抛 `SerialException`, 由 :meth:`SerialTransport.__init__` 翻成
  `TransportError`; serialwin32 侧本来就只支持独占 (传 `exclusive=False` 才 `ValueError`)。
* **进程内**: :func:`_claim_port` 的 `port -> 持有者` 登记表。它与 flock **不是重复的**:
  它排在 `serial.Serial()` **之前** (无副作用、fail-fast), 且那一刻**分得清**是"自己人
  重复开"还是"别的进程占着" —— 等 pyserial 的 flock 失败时只剩一句
  "Resource temporarily unavailable", 两者无从分辨。

⚠ 两层挡的都是**端口**级的互斥, 与上层开了几个 `Arm` 无关: 两个 `Arm` 各指一个口是合法的
(照样全开), 要挡的是**同一个口**被两条链路同时开 —— 不论那两条来自两个 `Arm`、同一个
`Arm` 的两次构造, 还是绕过 `Arm` 直接 new 出来的传输。故这道门挂在**传输**这一层。
"""
from __future__ import annotations

import threading
import weakref
from typing import Optional, Tuple

from litearm import _protocol as P
from litearm.errors import TransportError

_CDC_VID = 0x1D50
_CDC_PID = 0x606F

#: 以 SOF 起算但未收齐的"残缺帧"最多允许等待的时间。
#: 真帧在 921600 baud 下 260B 上限约 2.8ms 即收齐; 噪声里凑出的**假帧头**
#: (0xA5 + 声称长度大于实际数据) 则永远收不齐 —— 超过本窗口即丢弃该 SOF 重扫,
#: 否则它会把它**后面**的真帧永久挡住 (源不再来数据 = 永不返回)。
_PARTIAL_MAX_S = 0.25

#: 读超时**之后**那段"非阻塞补读"的时间上限 (秒)。见 `SerialTransport._read_frame_locked`:
#: 超时已到仍要把**已经到达**的字节读进来 (否则 `timeout=0` 一个字节都不碰), 但数据
#: **持续**到达时不能因此变成没有界的循环 —— 取 20ms: 库里已有的字节全是 `read(1)`
#: 同步返回, 20ms 读到**数千~一万字节量级** (本站 pty @921600 实测跑满 `_read_frame_locked`
#: 那条路: 20.0ms / 3 952~4 106 B ≈ **5.0 µs 每字节**; 评审另一次实测 20.0ms / 9 210 B
#: ≈ 2.2 µs 每字节 —— 同一量级, 随机器与负载浮动), 相对一帧上限 260B 仍有 **~15× 以上**
#: 余量; 而噪声灌满的线上最多多耗 20ms。
#: ⚠ **别把数量级往上写**: 这里是"**已经到达的字节**要读完花多久", 不是吞吐上限 ——
#: 旧措辞写"20ms 够读几万个字节", 比实测**高约一个量级**, 会把余量感夸大。
#: ⚠ 它与 `read_frame` 的 `timeout` **无关**: 后者决定"等多久才有新数据", 这个是
#: "已经到达的字节最多花多久读完"。
_NONBLOCK_EXTRA_S = 0.02

#: 写超时 —— **必须有界**。pyserial 默认 `write_timeout=None` 即无限阻塞, 一次卡住的
#: 保活写会长时间占着写锁, 把 `emergency_stop()`/`close()` 一起拖住; 而"急停必须
#: 永远可达"是本包的安全底线。921600 baud 下 260B 帧约 2.8ms, 0.5s 余量足够。
_WRITE_TIMEOUT_S = 0.5

#: 一次最多向串口索取的字节数 (见 `_read_chunk` / `_want_bytes`)。
#:
#: **2026-09-22 真机实测的由来** —— 从前这里是"每次 `read(1)`", 于是 CPU 代价与
#: **字节数**成正比: 每字节一次 `select()` + 一次 `os.read()`。这块板子**空闲时**
#: 状态流就有 ~14 kB/s (100Hz × 79B) ⇒ ~1.4 万次 syscall/秒 ⇒ 在 2 核机上
#: **一整核** (实测 server 无客户端时也占 100.0% CPU)。
#: 同字节量、同时长的微基准: 逐字节 **98.6%** vs 按到达量取走 **3.0%** (33×)。
#:
#: ⚠ **为什么不干脆 `read(4096)`**: pyserial 3.5 的 `read(size)` 是
#: `while len(read) < size` —— 它会**等到凑满 size 或超时**才返回。所以"一次要很多"
#: 本身**不省延迟、反而给每帧加最多一个 timeout 窗口**。必须**按"已经到达的字节数"**
#: (`in_waiting`) 要, 第一轮 `os.read` 就满足 `size` ⇒ 立即返回、不等待。
_READ_CHUNK_MAX = 4096


def find_cdc_port() -> Optional[str]:
    try:
        from serial.tools import list_ports
    except Exception:
        return None
    for p in list_ports.comports():
        if p.vid == _CDC_VID and p.pid == _CDC_PID:
            return p.device
    return None


#: **进程内**端口登记表: `port -> 持有者的弱引用`。
#:
#: 判据是持有者"**还活着**"而不是"登记过": 值持**弱引用**, 于是持有者一被回收这条登记
#: 自动消失。这不是图省事 —— 持有者一死, 它那个 pyserial 句柄也被一并回收 (pyserial 在
#: `__del__` 里关 fd), 端口**真的**空了: 弱引用消失与 fd 消失是同一件事。用**强**引用则
#: 相反 —— 一个忘了 `close()` 的旧对象会**永久**占着这个端口名, 把此后每一次连接都挡掉。
#:
#: ⚠ 它**不摆脱 gc 依赖**: 那条 `weakref` 要等**持有者被回收**才失效, 而 `SerialTransport`
#: 在本仓常被挂进 `Arm` (`arm._tr = tr`), 而 `Arm` **有环** (`_CartPending(arm=self)`)
#: ⇒ 挂**哪一层**都一样, 同样要等**全量 gc** —— 关键是**持有者要 `close()`**
#: (`_release_port` 当场生效, 与 gc 无关)。选**传输**这一层 (而非 `Arm`) 的**真实**理由是:
#: `Arm.close()`/`disconnect()`/`__del__` 三条收尾都落到 `transport.close()`, 且顺带盖住绕过 `Arm` 直接 new 传输的调用方。
#:
#: ⚠ 它是按**端口字符串**比的 —— 同一台设备的两种写法 (`/dev/ttyACM1` 与某个软链) 撞不上;
#: 那种情况由上面那层 flock 兜住 (内核比的是 open file description, 与字符串无关)。
_PORT_OWNERS: dict = {}
_PORT_OWNERS_LOCK = threading.Lock()


def _claim_port(port: str, owner: object) -> None:
    """把 `port` 记到 `owner` 名下; 已被**另一个活着的**持有者占着时**响亮失败**。

    ⚠ 排在 `serial.Serial()` **之前** (见模块顶部): 那一刻还没有任何副作用, 且错里能直接
    写清"是进程内自己人占用" —— 等 pyserial 的 flock 失败时就只剩一句 "Resource
    temporarily unavailable", 进程内/进程外再也分不出来。
    """
    with _PORT_OWNERS_LOCK:
        cur = _PORT_OWNERS.get(port)
        holder = cur() if cur is not None else None
        if holder is not None and holder is not owner:
            raise TransportError(
                f"端口 {port} 已被本进程内另一个传输占用 —— 同一端口同时只能有一条链路 "
                f"(跨进程那半由打开时的 flock 保证)。先把它关掉 "
                f"(持有者是 Arm 的话是 arm.close() / arm.disconnect()), 或改用另一个端口。")
        _PORT_OWNERS[port] = weakref.ref(owner)


def _release_port(port: str, owner: object) -> None:
    """撤掉 `port` 上**属于 `owner`** 的那条登记 (不是无条件 `del`)。

    ⚠ `is owner` 这个判据今天是**防御性**的, 到不了"该登记已经属于别人"那一步: 登记只在
    持有者**死后**才被别人替换, 而死者不会再来撤登记 (活着的持有者撤完自己那条之后,
    `close()` 的幂等早退也排在前面)。变异实测: 把它改成无条件 `del`, 全套 **483** 条
    **照样全绿** ⇒ 别把它当成在挡某个今天跑得出来的场景。⚠ "483" 是**当时的规模**、
    不是现值 (现值看 `pytest --collect-only`), 别拿它核现在的全量输出 —— 与
    `state.py:50` 的 "374" 同一种标注口径。留着是因为**本函数的契约应当是自明
    的** ("只撤自己的"), 而不是靠上面那句推理**永远**成立。
    """
    with _PORT_OWNERS_LOCK:
        cur = _PORT_OWNERS.get(port)
        if cur is not None and cur() is owner:
            del _PORT_OWNERS[port]


class SerialTransport:
    """帧读写传输。**读写各一把锁**, 读写之间不互斥。

    读者之间互斥: `_buf` / `_partial_since` / `_ser.timeout` 都是非线程安全的共享态。
    写者之间互斥: 一帧的字节必须原子地交给串口, 否则两帧交错 -> 上位机侧 CRC 全坏。
    读写之间**刻意不互斥**: pyserial 的 `os.read`/`os.write` 并发本就安全, 且
    `_ser.timeout` 只被读者改写 —— 若图省事用单把 RLock 罩住读写, 主线程一次最长
    几十 ms 的阻塞读会把 50ms 周期的保活写(见 `Arm.zero_g`)拖过 0.1s 看门狗窗口。
    """

    def __init__(self, port: str, timeout: float = 0.2):
        import serial
        #: 这两个先落 —— 失败路径 (`_release_port`) 与 `close()` 都要用 `port`。
        self.port = port
        self.timeout = timeout
        #: ⚠ 进程内那道门 (见模块顶部) —— 排在 `serial.Serial()` **之前**。
        _claim_port(port, self)
        try:
            self._ser = serial.Serial(port, 921600, timeout=timeout,
                                      write_timeout=_WRITE_TIMEOUT_S,
                                      exclusive=True)
        except Exception as e:  # noqa: BLE001
            #: ⚠ **构造处是 `exclusive` 真正抛错的地方** (posix 的 flock 失败 →
            #: `SerialException`), 而今天只有 `_read_chunk`/`write_frame` 两处包了
            #: `TransportError` ⇒ 不补这一处的话, 裸 `serial` 异常会从 `Arm.connect()`
            #: 一路逃到调用方 (`except LiteArmError` 那类分支全接不住, 它连本包定义的
            #: 异常都不是)。`from e` + 原文进 message: 出错原因不许在这里丢掉。
            #: 那一格进程内登记也要撤 —— 否则这次失败会把端口**占死**。
            _release_port(port, self)
            raise TransportError(f"打开串口 {port} 失败: {e}") from e
        self._buf = bytearray()
        self._partial_since = None      # 残缺帧计时起点 (见 read_frame)
        self._rlock = threading.RLock()  # 读者互斥
        self._wlock = threading.RLock()  # 写者互斥
        self._text = bytearray()        # 被丢弃的噪声字节留痕 (含开机签名), 有上限
        #: 本传输是否已关 —— `close()` 的幂等**靠这一格**, 不靠 pyserial 的内部守卫
        #: (它的 `close()` 确实有 `if self.is_open:` 兜底, 但那是它的实现细节:
        #: `os.close()` 中途抛 (fd 已被别人关掉) 时它的 `is_open` 会停在 True, 于是
        #: "再关一次"会去重试一串已经关掉的 fd)。契约必须由**我们这一层**说了算。
        self._closed = False
        #: `flush()` (tcdrain) 失败的**累计**次数 —— 只做可观测性, 不改变任何行为:
        #: 那一下失败**不算**发送失败 (帧已经交进驱动了, 收不回来), 见 `write_frame`。
        self.flush_failures = 0

    #: 噪声留痕上限 —— 固件开机签名 (banner) 不是帧, 会被当噪声丢弃; 留痕是为了
    #: 解析出「本次是否 IWDG 复位」。噪声可能无限多, 故必须封顶。
    TEXT_LOG_MAX = 2048

    @property
    def text_log(self) -> str:
        """最近被跳过的非帧字节 (解码后), 用于解析固件开机签名。

        `_text` 与 `_buf`/`_partial_since`/`_ser.timeout` 同属读者线程的共享态,
        故读它也要持读锁 (free-threaded 构建下不加锁是真数据竞争)。
        """
        with self._rlock:
            return bytes(self._text).decode("utf-8", errors="replace")

    @property
    def is_open(self) -> bool:
        if self._closed:
            return False
        try:
            return self._ser.is_open
        except Exception:
            return False

    def close(self) -> None:
        """关链路 (**幂等**: 重复调用是 no-op, 不抛)。与在途读/写互斥 —— 否则
        `os.close(fd)` 与 pyserial 置 `fd=None` 之间 `is_open` 仍为真, 落在这个窗口
        的读者会对**已被释放的 fd 号**做 termios 配置 (fd 复用后即操作到无关对象)。

        ⚠ 幂等的判据是 `_closed` 那一格, **不是**"关两次恰好也没事": 上层有**多条**
        收尾路径 (`Arm.close()` / `Arm.disconnect()` / `Arm.__del__` 的兜底), 它们
        全落在这里 —— 幂等是它们能互相叠加的前提。

        ⚠ **进程内登记必须在这里撤** (在幂等早退**之后**): 不撤的话, `close()`/
        `disconnect()` 之后连同一个口会被自己的登记挡住 —— `Arm.reconnect()` 走的正是
        "先关再连"这条路。上面那三条收尾路径因此全都自带释放 (它们都落在这一个方法上),
        这正是"收口一处"要买的东西。
        """
        with self._rlock, self._wlock:
            if self._closed:
                return
            self._closed = True
            try:
                self._ser.close()
            except Exception:
                pass
        # 放在实例锁**外**: 它取的是模块级那把锁, 而全仓没有第二处会**持着模块锁再去取
        # 实例锁** ⇒ 两把锁不嵌套, 就不必去论证"没有谁按相反顺序取它们"。
        _release_port(self.port, self)

    def write_frame(self, cmd: int, payload: bytes = b"") -> None:
        """写一帧。**`write()` 与 `flush()` 的失败必须分开报** —— 别顺手合并两个 try。

        ⚠ **"抛 `TransportError` ⟹ 整帧未送达"是一条载重不变量**:
        :meth:`_CartPending.request` 靠"写失败就摘掉自己那个 token"成立, 而那条只在
        "帧没出去"时才安全 —— 帧其实送到了却摘掉 token, 固件随后那条 `0x4E` 会配给
        队列里**后面那条活 token**, 于是报"成功"而实际没跑 (**假成功**)。

        而两半的物理事实不同:

        * `write()` 抛 → 帧没送达 (残缺帧被固件按 CRC 丢) ⇒ 抛 `TransportError`;
        * `flush()` (tcdrain) 抛 → **整帧已经交进驱动、会被送达**, 失败收不回来
          ⇒ **不算发送失败**, 只计进 `flush_failures` (静默是另一条底线)。

        刻意不为此新增异常类型: 真链路故障会在**下一次写**上由 `write()` 抛出, 藏不住。
        """
        with self._wlock:
            try:
                self._ser.write(P.pack_frame(cmd, payload))
            except Exception as e:  # noqa: BLE001
                raise TransportError(f"写失败: {e}") from e
            try:
                self._ser.flush()
            except Exception:  # noqa: BLE001
                self.flush_failures += 1

    def read_frame(self, timeout: Optional[float] = None) -> Optional[Tuple[int, bytes]]:
        """读一帧; 超时返回 None。跳过噪声字节, 丢弃 CRC 坏帧。

        缓冲自持完整流 (含候选帧头 SOF): 读超时时**已到达的半帧留在缓冲里**,
        下次调用续读即可 —— 旧实现在超时时已把 SOF 丢掉, 余下字节会被当新流扫描,
        导致整帧丢失(半帧超时 = 丢一帧)。

        ⚠ **`timeout=0` 是"有就给我", 不是"一个字节都不准碰"**: 超时已到时仍要把
        **已经到达**的字节**非阻塞**地读进来, 否则"数据早已躺在驱动缓冲里"这一支永远
        取不到 —— 真机上
        `Arm.poll_cart()` (它走的就是本方法的 `timeout=0`) 会**恒**返回 `None`, 而离线用例
        全绿 (桩把 `timeout` 参数整个忽略掉了)。见 `_read_frame_locked` 里那条分支。
        """
        end = _now() + (timeout if timeout is not None else self.timeout)
        with self._rlock:
            return self._read_frame_locked(end)

    def _note_text(self, byte: int) -> None:
        """留存一个被当作噪声丢掉的字节 (滚动窗口, 见 `text_log`)。"""
        self._text.append(byte)
        if len(self._text) > self.TEXT_LOG_MAX:
            del self._text[:len(self._text) - self.TEXT_LOG_MAX]

    def _read_frame_locked(self, end: float) -> Optional[Tuple[int, bytes]]:
        #: 超时之后那段**非阻塞**取字节的截止 (见下面那条分支); `None` = 还没进那一支。
        nb_end: Optional[float] = None
        while True:
            while self._buf:
                if self._buf[0] != P.SOF:
                    self._note_text(self._buf[0])
                    del self._buf[0]              # 不是帧头 -> 丢一个字节继续找
                    continue
                if len(self._buf) < 3:
                    break                         # CMD/LEN 未齐, 等更多字节
                ln = self._buf[2]
                total = 3 + ln + 2                # SOF+CMD+LEN + payload + crc2
                if len(self._buf) < total:
                    break                         # 帧体未齐, 保留(含 SOF)等下一轮
                got = P.unpack_frame(bytes(self._buf[:total]))
                if got is not None:
                    del self._buf[:total]
                    self._partial_since = None
                    return got
                del self._buf[0]                  # CRC/长度坏 -> 丢这个 SOF 重找
            now = _now()
            if self._buf and self._buf[0] == P.SOF:
                if self._partial_since is None:
                    self._partial_since = now     # 残缺帧开始计时
                elif now - self._partial_since > _PARTIAL_MAX_S:
                    del self._buf[0]              # 收不齐 -> 假帧头, 丢掉重扫
                    self._partial_since = None
                    continue
            else:
                self._partial_since = None
            if now >= end:
                # 窗口已经用尽 —— 但**仍要**把**已经到达**的字节读进来: `timeout=0` 的
                # 语义是"有就给我", 旧实现直接 `return None` 是"一个字节都不碰", 于是
                # "数据早已在驱动缓冲里"这一支永远取不到 (`Arm.poll_cart()` 在真机上恒
                # `None`)。这一步**不会等新数据**: 传进去的 `end` 就是此刻, `_read_chunk`
                # 会把它夹到 0 (pyserial 的非阻塞档) ⇒ 没数据立刻 `b""`。
                # ⚠ **必须有时间上限** (`_NONBLOCK_EXTRA_S`): 数据**持续**到达时, 光靠
                # "读到凑出一帧"是**没有界**的 (阻塞档有 `end` 兜底, 这一支没有) ——
                # 噪声灌满的线上不能把本调用变成不返回的循环。
                if nb_end is None:
                    nb_end = now + _NONBLOCK_EXTRA_S
                elif now > nb_end:
                    return None
                chunk = self._read_chunk(now)
                if chunk:
                    self._buf += chunk
                    continue                     # 有新字节 -> 回上面重试装配
                return None
            chunk = self._read_chunk(end)
            if chunk:
                self._buf += chunk

    def _want_bytes(self) -> int:
        """这次 `read` 该要几个字节 = **已经到达的字节数**（封顶 `_READ_CHUNK_MAX`）。

        * `in_waiting > 1` ⇒ 按它要 —— 一次把这批取走, 这是本改动的全部收益所在；
        * 否则（`0`/`1`/拿不到）⇒ 要 `1`：**阻塞等第一个字节**, 与从前逐字节读
          的**行为完全一致**。

        ⚠ **拿不到就必须退化成 1, 不能抛**: 测试桩没有 `in_waiting`, 某些平台也可能
        不支持。退化成 1 **只慢不错** —— 装配循环本来就按"来多少字节都可能"写的。
        ⚠ 反过来 `in_waiting` 报多了也不会丢字节: pyserial 在超时到点时把**已经读到的**
        部分返回 (`read` 的 `while len(read) < size` 以 `timeout.expired()` 收尾)。
        """
        try:
            n = int(self._ser.in_waiting)
        except Exception:                    # noqa: BLE001 - 桩/平台不支持
            return 1
        return min(n, _READ_CHUNK_MAX) if n > 1 else 1

    def _read_chunk(self, end: float):
        """读一块。`end <= now` 时是**非阻塞**读 (pyserial `timeout=0`)。

        ⚠ 旧实现在 `remain <= 0` 时**直接返回 `None`** (一个字节都不碰) —— 那是
        "非阻塞读"被实现成了"不读"。现在负值夹到 `0`: pyserial 在 `timeout=0` 下
        **不等待、但照取样**, 这才是 `read_frame(0)` 要的语义 (见那里)。

        **一次读多少** 由 `_want_bytes()` 决定 —— 见 `_READ_CHUNK_MAX` 那条。
        """
        import time
        remain = end - time.monotonic()
        if remain < 0:
            remain = 0.0
        try:
            self._ser.timeout = min(remain, 0.05)
            got = self._ser.read(self._want_bytes())
            if not got:
                return None
            return got
        except Exception as e:  # noqa: BLE001
            raise TransportError(f"读失败: {e}") from e


def _now() -> float:
    import time
    return time.monotonic()
