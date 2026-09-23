"""真实 SerialTransport 的字节流解析 —— 此前零覆盖 (桩测试整替换掉了 transport)。

覆盖: 连续多帧、逐字节到达、噪声字节、CRC 坏帧、**半帧读超时不丢帧**。
用 fake `serial.Serial` 注入 (read(1) 每次最多 1 字节, 与真实用法一致)。
"""
from __future__ import annotations

import os
import struct
import sys
import time
import types
import weakref

import pytest

from litearm import _protocol as P
from litearm.errors import TransportError
from litearm.transport import SerialTransport


def mk_status(seq, n=7):
    body = bytearray(struct.pack("<HH", (1 << 6), seq))
    for i in range(n):
        body += struct.pack("<fffff", i * 0.1, 0.0, 0.5, 30.0, 25.0) + b"\x00"
    body += struct.pack("<H", 0)
    return P.pack_frame(P.RSP_STATUS, bytes(body))


def _install(monkeypatch, data, delay_at=None, delay_s=0.06):
    """注入 fake Serial。delay_at = 读到第 N 个字节前先睡 delay_s (模拟上位机读超时)。"""
    st = {"buf": bytearray(data), "read_calls": 0}

    class FakeSerial:
        def __init__(self, port, baudrate=0, timeout=0.2, **kw):
            self.is_open = True
            self.timeout = timeout

        def read(self, size=1):
            if delay_at is not None and st["read_calls"] == delay_at:
                time.sleep(delay_s)
            if not st["buf"]:
                return b""
            out = bytes(st["buf"][:size])
            del st["buf"][:size]
            st["read_calls"] += 1
            return out

        def write(self, d):
            return len(d)

        def flush(self):
            pass

        def close(self):
            self.is_open = False

    monkeypatch.setitem(sys.modules, "serial", types.SimpleNamespace(Serial=FakeSerial))
    return st


def _drain(tr, limit=20, timeout=0.05):
    """反复读直到没数据 (与上层调用模式一致)。"""
    out, idle = [], 0
    while len(out) < limit and idle < 3:
        fr = tr.read_frame(timeout)
        if fr is None:
            idle += 1
        else:
            idle = 0
            out.append(fr)
    return out


def test_parses_consecutive_frames(monkeypatch):
    frames = [mk_status(s) for s in range(1, 6)]
    _install(monkeypatch, b"".join(frames))
    tr = SerialTransport("/dev/null")
    got = _drain(tr)
    assert [struct.unpack_from("<H", p, 2)[0] for _, p in got] == [1, 2, 3, 4, 5]


def test_skips_noise_bytes(monkeypatch):
    frames = [mk_status(1), mk_status(2)]
    _install(monkeypatch, b"\x11\x22\x00" + b"".join(frames) + b"\x7f")
    tr = SerialTransport("/dev/null")
    got = _drain(tr)
    assert [struct.unpack_from("<H", p, 2)[0] for _, p in got] == [1, 2]


def test_drops_crc_bad_frame_keeps_next(monkeypatch):
    good = mk_status(7)
    bad = bytearray(mk_status(8))
    bad[-1] ^= 0xFF                       # 破坏 CRC
    _install(monkeypatch, bytes(bad) + good)
    tr = SerialTransport("/dev/null")
    got = _drain(tr)
    assert [struct.unpack_from("<H", p, 2)[0] for _, p in got] == [7]


def test_midframe_read_timeout_does_not_lose_frame(monkeypatch):
    """半帧处读超时 -> 该帧必须能续读出来, 不能被丢掉。

    旧实现在超时时把已消费的 SOF 丢掉、把余下字节当新流扫描, 导致整帧丢失
    (注入 5 帧只收到 seq [1,3,4,5])。
    """
    frames = [mk_status(s) for s in range(1, 6)]
    _install(monkeypatch, b"".join(frames), delay_at=len(frames[0]) // 2)
    tr = SerialTransport("/dev/null")
    got = _drain(tr)
    assert [struct.unpack_from("<H", p, 2)[0] for _, p in got] == [1, 2, 3, 4, 5]


def test_no_data_returns_none(monkeypatch):
    _install(monkeypatch, b"")
    tr = SerialTransport("/dev/null")
    assert tr.read_frame(0.02) is None


def test_partial_header_preserved(monkeypatch):
    """只到了 SOF+CMD 就超时 -> 后续补上 LEN/PAYLOAD 仍能解出。"""
    fr = mk_status(42)
    _install(monkeypatch, fr, delay_at=2, delay_s=0.06)
    tr = SerialTransport("/dev/null")
    got = _drain(tr)
    assert [struct.unpack_from("<H", p, 2)[0] for _, p in got] == [42]


def test_false_sof_with_oversized_len_does_not_block_next_frame(monkeypatch):
    """噪声字节恰好构成"假帧头": 0xA5 + 声称长度超过实际剩余数据。

    解析器若一直等这个假帧补齐, 就会把它**后面**的真帧永久挡住 (源已耗尽 = 永不
    返回)。必须在一个有限的装配窗口后放弃假帧头并重扫。
    """
    good = mk_status(9)
    bogus = bytes([P.SOF, 0x56, 0xED])      # SOF + 声称 237B payload
    _install(monkeypatch, bogus + good)
    tr = SerialTransport("/dev/null")
    got = []
    end = time.monotonic() + 2.0
    while time.monotonic() < end and not got:
        fr = tr.read_frame(0.05)
        if fr is not None:
            got.append(fr)
    assert [struct.unpack_from("<H", p, 2)[0] for _, p in got] == [9], \
        "假帧头挡住了后面的真帧"


def test_partial_real_frame_still_preserved_within_window(monkeypatch):
    """装配窗口内, 真帧的半帧仍必须保留 (不能因为上面的放弃逻辑把正常续读也砍掉)。"""
    fr = mk_status(42)
    _install(monkeypatch, fr, delay_at=len(fr) // 2, delay_s=0.08)
    tr = SerialTransport("/dev/null")
    got = _drain(tr)
    assert [struct.unpack_from("<H", p, 2)[0] for _, p in got] == [42]


def test_a_decode_resets_the_partial_timer_so_the_next_tail_still_resumes(monkeypatch):
    """⚠ **一次成功解码之后**, 残缺帧计时器必须**归零** (`_partial_since = None`)。

    少了那一句, 同一次调用里保留下来的尾巴会沿用**上一次**半帧的时刻, 过了
    `_PARTIAL_MAX_S` (0.25s) 就被判成假帧头**丢一个字节再逐字节吃光** —— 一条本来能
    续读的半帧就这么没了。残缺帧计时器是**每实例一个**的跨调用状态, 所以这条只在
    "解码之后 `_buf` 里还留着以 SOF 开头的尾巴"时才可观测。

    ⚠ **这个形状今天只有一条可达路径** (`_read_chunk` 每次只 `read(1)`, `_buf` 是**一
    字节一字节**长到整帧的 ⇒ 正常流里解完一帧 `_buf` 恰好被取空, 尾巴长度是 0 ——
    实测全量套件共 22 次真实解码, **除本用例自己造的那一条 (尾巴 11B) 之外, 其余 21 次
    的尾巴长度全是 0**): **坏 CRC 的外层帧**被丢掉一个 SOF 之后**在同一次内层扫描里**
    继续找 SOF, 于是它载荷里那条**完好的内层帧**被解出来, 后面的字节全成了尾巴。
    本用例就照这个形状造 (外层 20B 坏帧: 头 3B + 内层好帧 6B + 尾巴 9B + 2B 坏 CRC)。

    判据是**尾巴仍可续读**: 补上尾帧剩下的字节之后必须解出**那条尾帧**。
    """
    tail_frame = P.pack_frame(0x01, bytes(200))            # 205B, 尾巴就是它的前缀
    inner = P.pack_frame(0x01, b"\x00")                    # 6B: 载荷里那条**完好**的帧
    outer = bytes([P.SOF, 0x56, 15]) + inner + tail_frame[:9] + tail_frame[9:11]
    assert len(outer) == 20 and outer[3:9] == inner and outer[9] == P.SOF
    assert P.unpack_frame(inner) == (0x01, b"\x00"), "前提: 内层帧必须是好的"
    assert P.unpack_frame(outer) is None, "前提: 外层帧必须是坏的 (CRC 坏)"

    st = _install(monkeypatch, outer)
    # 端口名**本用例独享** (conftest 那条泄漏守卫的建议): 本用例失败时回溯会钉住 `tr`
    # ⇒ 不 close, 若用 `/dev/null` 这个名字会连带把**后面**用同一名字的用例一起弄红,
    # 归因就指错了人。
    tr = SerialTransport("/dev/null-partial-timer")

    assert tr.read_frame(0.05) == (0x01, b"\x00"), "畸形扫描没解出内层那条好帧"
    assert len(tr._buf) == 11 and tr._buf[0] == P.SOF, (
        f"解出内层帧之后 `_buf` 里没有留下以 SOF 开头的尾巴: {bytes(tr._buf)!r}"
        f" —— 用例前提不成立")

    time.sleep(0.3)                       # 越过 `_PARTIAL_MAX_S` (0.25s)

    assert tr.read_frame(0.05) is None
    st["buf"] += tail_frame[11:]          # 补上尾帧剩下的 194B
    assert tr.read_frame(0.2) == (0x01, bytes(200)), (
        "那条尾巴没能续读出来 —— 解码时没有把残缺帧计时器归零 (`_partial_since`), "
        "它沿用了上一次半帧的时刻, 尾巴被判成假帧头丢掉了")


def test_a_zero_timeout_read_still_takes_bytes_that_already_arrived(monkeypatch):
    """⚠ `read_frame(0)` 的语义是"**有就给我**", 不是"一个字节都不准碰"。

    旧实现在进入循环后**先**判 `now >= end` 就 `return None` —— 一个字节都不读, 于是
    "数据早已躺在驱动缓冲里"这一支**永远取不到**。真机上 `Arm.poll_cart()` 因此在**任何**
    情况下都返回 `None` (它走的就是 `read_frame(0.0)`), 而离线用例全绿 —— 因为桩
    (`fake_serial.FakeTransport.read_frame`) 把 `timeout` 参数整个**忽略**掉了
    (它只管 `if self._resp: pop(0)`)。
    """
    st = _install(monkeypatch, P.pack_frame(P.RSP_STATUS, b"\x01\x02"))
    tr = SerialTransport("/dev/null")

    fr = tr.read_frame(0.0)
    assert fr is not None and fr[0] == P.RSP_STATUS, (
        "库里就有一条完整帧, 非阻塞读却返回 None —— 它没有真的去取")
    assert st["buf"] == bytearray(), "帧还留在'内核缓冲'里 —— 一个字节都没取走"


def test_a_zero_timeout_read_does_not_wait_for_a_partial_frame(monkeypatch):
    """非阻塞读**不许**为了凑齐半帧而等 —— 库里只有半帧时立刻返回 `None` 并**保留**它。

    (这条与上一条是一对: 上一条钉"要真的读", 这一条钉"别读起来没完"。)
    """
    full = P.pack_frame(P.RSP_STATUS, b"\x01\x02")
    st = _install(monkeypatch, full[:6])
    tr = SerialTransport("/dev/null")

    t0 = time.monotonic()
    assert tr.read_frame(0.0) is None
    assert time.monotonic() - t0 < 0.05, "非阻塞读睡下去了"

    assert st["buf"] == bytearray() and len(tr._buf) == 6, "半帧没被保留下来"
    st["buf"] += full[6:]
    assert tr.read_frame(0.05) is not None, "半帧之后的续读没能凑齐"


def _install_endless_noise(monkeypatch, cut_after_s: float = 3.0):
    """注入 fake Serial: `read()` **永远**返回噪声字节, **永不**返回 `b""`。

    ⚠ 这是 `_NONBLOCK_EXTRA_S` 存在理由所对应的**唯一**形状: 数据**持续**到达时, 超时后
    那段"非阻塞补读"只能靠**时间**收口 —— 既有用例的桩缓冲要么空 (`b""`)、要么有限, 循环
    都必然自己结束, 于是**没有任何用例**碰到那条时间上限 (把 `_NONBLOCK_EXTRA_S` 改成
    `float('inf')`, 全量套件照绿)。

    `cut_after_s` 是**测试自保**: 到点抛异常, 把"没有时间上限"的实现掐断。挂死会拖垮整个
    套件, 而"抛出来"同样是一条红。
    """
    st = {"reads": 0, "t0": None}

    class EndlessNoise:
        def __init__(self, port, baudrate=0, timeout=0.2, **kw):
            self.is_open = True
            self.timeout = timeout

        def read(self, size=1):
            st["reads"] += 1
            if st["t0"] is None:
                st["t0"] = time.monotonic()
            elif time.monotonic() - st["t0"] > cut_after_s:
                raise RuntimeError(
                    "测试自保: 噪声已灌满 cut_after_s, 被测实现没有时间上限")
            return b"\x00" * size       # 永远有数据

        def write(self, d):
            return len(d)

        def flush(self):
            pass

        def close(self):
            self.is_open = False

    monkeypatch.setitem(sys.modules, "serial",
                        types.SimpleNamespace(Serial=EndlessNoise))
    return st


def test_a_zero_timeout_read_is_bounded_even_when_data_keeps_arriving(monkeypatch):
    """⚠ `_NONBLOCK_EXTRA_S` 说的是"**已经到达**的字节最多花多久读完" —— 必须**有上限**。

    那一支 (`_read_frame_locked` 里 `now >= end` 之后) **没有** `end` 兜底: 只要数据持续
    到达, 光靠"读到凑出一帧"就没有界。噪声灌满的线上不能把 `read_frame` 变成不返回的循环
    —— 它正是 `Arm.poll_cart()` 走的那条非阻塞口 (`_read_one(0.0)`)。

    判据: `read()` 永不返回 `b""` 时, `read_frame(0.0)` 仍必须在 `_NONBLOCK_EXTRA_S`
    量级内返回 `None`。
    (改前实测: 上限取 `inf` ⇒ 这里不返回, 由 `cut_after_s` 抛出 `TransportError` 掐断 ⇒ 红。)
    """
    _install_endless_noise(monkeypatch)
    tr = SerialTransport("/dev/null")

    t0 = time.monotonic()
    fr = tr.read_frame(0.0)
    dt = time.monotonic() - t0

    assert fr is None, "噪声里不该凑出一帧"
    assert dt < 0.5, (
        f"read_frame(0.0) 花了 {dt:.3f}s —— `_NONBLOCK_EXTRA_S` 的时间上限没生效 "
        f"(数据持续到达时本调用必须有界; 它是 `poll_cart` 的口)")



# ========================== 端点独占 (§6.6 ① ② ③) ==========================
# 背景实测 (E5): **Linux 下 CDC 口不独占** —— 第二个进程 `open` **成功**且分吃同一字节
# 流 (双方拿残帧、**静默失败**); Windows 反而由 OS 强制独占 ⇒ 独占性因平台而异, 不能
# 当常量依赖。三处收口都在 `transport.py`, 本节**逐层**咬:
#   · 跨进程那层 —— 真 pyserial + 真 flock (`test_an_externally_locked_port_...`);
#   · 构造处翻译 —— `exclusive` 抛的就是 `SerialException`;
#   · 进程内那层 —— 端口登记表 (用 fake `serial` 把 flock 那层**摘掉**, 只剩登记表)。
#
# ⚠ 本节的用例**各用一个独享的端口名**: 登记表按端口字符串比, 若都挤在 "/dev/null" 上,
# 前一个用例里那个"还活着"的传输就会挡到后一个 —— 而"它还活着"取决于**帧有没有被引用环
# 钉住** (pytest 的 `raises(...) as ei` 会造出 `帧 -> ei -> traceback -> 帧` 这个环 ⇒ 该帧
# 与其中的传输要等 gc 才消失), 于是失败会**按用例顺序传染**。独享端口名把这条耦合去掉。
# (这不是实现缺陷: 被环钉住的传输**真的**还开着那个 fd —— 登记表报"占用"是真话。)

def _install_serial(monkeypatch, ser_cls, exc_cls=None):
    """注入 fake `serial` 模块 (可带 `SerialException`)。"""
    ns = {"Serial": ser_cls}
    if exc_cls is not None:
        ns["SerialException"] = exc_cls
    monkeypatch.setitem(sys.modules, "serial", types.SimpleNamespace(**ns))


_ORIGINAL_TEXT = "桩: 原文保留判据"


def _broken_serial(exc_cls, at: str):
    """造一个"在 `at` 处抛 `exc_cls(原文)`"的 fake `serial.Serial`。

    `at="open"` = 构造处抛 (`exclusive` 被拒就是这一支); `"read"`/`"write"` = 开成功,
    句柄在这两处抛。
    """
    def _boom():
        raise exc_cls(_ORIGINAL_TEXT)

    class BrokenSerial:
        def __init__(self, port, baudrate=0, timeout=0.2, **kw):
            if at == "open":
                _boom()
            self.is_open = True
            self.timeout = timeout

        def read(self, size=1):
            if at == "read":
                _boom()
            return b""

        def write(self, d):
            if at == "write":
                _boom()
            return len(d)

        def flush(self):
            pass

        def close(self):
            self.is_open = False

    return BrokenSerial


@pytest.fixture
def pty_port():
    """一对**真** pty 的从端路径 —— 走真 pyserial (本文件的 `_install*` 只在本用例内
    注入 fake `serial`)。pty 的两个 fd 全程开着, 用完关掉。"""
    master, slave = os.openpty()
    try:
        yield os.ttyname(slave)
    finally:
        for fd in (slave, master):
            try:
                os.close(fd)
            except OSError:
                pass


def test_an_externally_locked_port_cannot_be_opened(pty_port):
    """① **载重判据**: 该口已被另一条 `open file description` 独占时, `SerialTransport`
    必须**开不起来**。

    那条句柄走的是**真的** pyserial (`exclusive=True`), 与"另一个进程打开"是同一条内核
    判据 —— 测试里起不了第二个进程, 真机的跨进程那半见 Task 9 V5。

    变异 (删掉 `exclusive=True`) ⇒ 本用例红: 没有 flock 时第二次 `open` 成功, 那正是
    Linux 上"两个进程静默分吃同一字节流"的形状 (实测本机: 不带 `exclusive` 的第二次
    `open` 确实成功)。
    """
    import serial as real_serial

    holder = real_serial.Serial(pty_port, 921600, timeout=0.2, exclusive=True)
    try:
        with pytest.raises(TransportError) as ei:
            SerialTransport(pty_port)
        assert pty_port in str(ei.value), (
            "翻译后的错必须点名是哪个口 —— 只留 pyserial 那句 'Resource temporarily "
            f"unavailable' 无从照做 (实际 {str(ei.value)!r})")
    finally:
        holder.close()

    # 让开之后同一个口必须能开 —— 否则上面那条红可能只是"这个口本身坏了"
    tr = SerialTransport(pty_port)
    assert tr.is_open
    tr.close()


def test_arm_connect_on_a_taken_port_raises_a_lite_arm_error(pty_port):
    """② 的用户面: 口被占时 `Arm.connect()` 抛的必须是**本包的错误**。

    裸 `serial.SerialException` 从 `connect()` 逃出去的话, 上游 `except LiteArmError`
    那类分支全接不住 (它连本包的基类都不是)。
    """
    import serial as real_serial

    from litearm import Arm

    holder = real_serial.Serial(pty_port, 921600, timeout=0.2, exclusive=True)
    try:
        with pytest.raises(TransportError):
            Arm(port=pty_port).connect()
    finally:
        holder.close()


def test_a_second_transport_on_the_same_port_fails_loudly(monkeypatch):
    """③ 同进程内两个传输指同一个口 ⇒ 第二个**响亮失败**, 不是静默分吃。

    用 fake `serial` (每次构造给一个独立句柄) 把 flock 那层**摘掉** —— 于是本用例咬的
    **只有**登记表: 抽掉登记表, 第二个构造会成功, 那正是"静默分吃"的形状。
    """
    port = "/dev/fake-excl-second"
    _install(monkeypatch, b"")
    tr1 = SerialTransport(port)

    with pytest.raises(TransportError) as ei:
        SerialTransport(port)

    assert port in str(ei.value), f"错里没点名端口: {str(ei.value)!r}"
    assert tr1.is_open, "第一个传输不该被这一下动到"
    tr1.close()


def test_closing_releases_the_port_registration(monkeypatch):
    """③ 释放登记 —— 否则**重连会被自己挡住**。

    `close()`/`disconnect()` 之后同一个口必须能再开: `Arm.reconnect()` 走的就是
    "先关再连"这条路, 而 `disconnect()` 是 `close()` 的别名 ⇒ 两条都落在这一格上。
    """
    port = "/dev/fake-excl-close"
    _install(monkeypatch, b"")
    tr1 = SerialTransport(port)
    tr1.close()

    tr2 = SerialTransport(port)                 # ← 不许被自己上一个传输挡住
    assert tr2.is_open
    tr2.close()


def test_dropping_a_transport_releases_the_port_registration(monkeypatch):
    """③ 登记判的是"持有者**还活着**", 不是"登记过"。

    对象一被回收, 它那个 pyserial 句柄也被一并回收 (pyserial 在 `__del__` 里关 fd) ⇒
    口**真的**空了。若登记表持**强**引用, 一个忘了 `close()` 的旧对象会永久占住端口名,
    把此后每一次连接都挡掉 (而且没有解开的入口)。

    前提 (`weakref` 立刻为 `None`) 是本用例的一部分: 登记表依赖"传输对象**无环**"。
    """
    port = "/dev/fake-excl-drop"
    _install(monkeypatch, b"")
    tr1 = SerialTransport(port)
    ref = weakref.ref(tr1)
    del tr1

    assert ref() is None, (
        "前提破了: `SerialTransport` 变得需要 gc 才回收 —— 登记表的弱引用会滞留, "
        "于是'死掉的'传输还会挡住重连")

    tr2 = SerialTransport(port)                 # ← 不许被一个已死的对象挡住
    assert tr2.is_open
    tr2.close()


class _Pinned:
    """一个**自环**持有者 —— 与 `Arm` 的 `_CartPending(arm=self)` 同型。

    `self.self = self` 造出引用环: 外层引用一丢, 引用计数收不掉它, 只有全量 gc 能收。
    真实事故的形状就是这样 —— 用例函数返回、局部名消失之后传输**还活着**, 端口登记里那条
    弱引用**不会被自动清掉** (要等一次全量 gc), 于是"释放"变成了看时机的事。

    ⚠ 用例里仍把它放在**局部名**里。那是为了让"持有者此刻一定还活着"成为**确定**的前置
    (不必去赌 gc 跑没跑), 判据才确定; 环本身不参与任何断言。
    """

    def __init__(self):
        self.self = self
        self.tr = None


def test_a_closed_transport_does_not_block_the_port_even_when_it_is_pinned(monkeypatch):
    """③ `close()` 之后同一个口**立刻**能再开 —— 释放**不能**取决于 gc 何时跑。

    ⚠ 与上一条 (`test_dropping_a_transport_releases_the_port_registration`) 的**分工**:
    上一条咬的是"对象被回收 ⇒ 登记自动消失"那条路 (弱引用), 那一条**本来就**看 gc 何时跑;
    本用例咬的是**另一条路** —— `close()` 里那句 `_release_port`, 它必须**当场**撤登记。
    两条路都要在, 因为"重连会不会被自己上一个传输挡住"这件事不许由 gc 的调度决定。

    持有者用**自环** (`_Pinned`, 与 `Arm` 的 `_CartPending(arm=self)` 同型) 并留在局部名里:
    它此刻**一定**还活着, 所以下面那次构造成功就**只**能来自 `_release_port`, 排除了"其实
    是弱引用刚好断了"这种解释 —— 断言因此可以下在登记表本身 (**立刻**消失) 上, 而不是
    "迟早会消失"。
    """
    from litearm.transport import _PORT_OWNERS

    port = "/dev/fake-excl-pinned-closed"
    _install(monkeypatch, b"")
    pinned = _Pinned()
    tr1 = SerialTransport(port)
    pinned.tr = tr1                               # ← 挂在环上 (函数返回后它也还在)
    tr1.close()                                   # ← fd 已释放

    assert port not in _PORT_OWNERS, (
        "close() 之后登记必须**立刻**消失 —— 要靠 gc 才消失的话, 重连会不会被自己上一个"
        "传输挡住就成了看时机的事")

    tr2 = SerialTransport(port)                   # ← 已 close 的口不许被挡住
    assert tr2.is_open, "close() 之后的传输仍然占着端口名 —— 后来者被一个僵尸挡住了"
    tr2.close()


def test_an_open_transport_blocks_the_port_even_when_it_is_pinned(monkeypatch):
    """③ **反向**: 被环钉住**且还开着**的传输, 同一个口的第二者必须响亮失败。

    与上一条**只差一个 `close()`** —— 两条合起来把"拦住的是**口还开着**这件事"钉死:
    既不是"登记过", 也不是"对象还在"。少了本用例, 上一条的断言只看"能再开", 无法区分
    "因为 `close()` 撤了登记"与"因为压根没人拦" —— 真·静默分吃会从那个缺口回来。
    """
    port = "/dev/fake-excl-pinned-open"
    _install(monkeypatch, b"")
    pinned = _Pinned()
    pinned.tr = SerialTransport(port)

    assert pinned.tr.is_open
    with pytest.raises(TransportError) as ei:
        SerialTransport(port)

    assert port in str(ei.value), f"错里没点名端口: {str(ei.value)!r}"
    pinned.tr.close()


def test_open_failure_is_a_transport_error_that_keeps_the_original_text(monkeypatch):
    """② **构造处** —— `exclusive` 被拒时抛的**就是** `SerialException`, 而 `__init__`
    里那句 `serial.Serial(...)` 此前**没有**任何包装 ⇒ 裸异常会从 `Arm.connect()` 逃出去。
    """
    class SerialException(OSError):
        pass

    _install_serial(monkeypatch, _broken_serial(SerialException, "open"), SerialException)

    with pytest.raises(TransportError) as ei:
        SerialTransport("/dev/fake-excl-open")

    assert _ORIGINAL_TEXT in str(ei.value), (
        f"翻成 TransportError 时把原文丢了: {str(ei.value)!r}")


def test_a_failed_open_does_not_leave_the_port_claimed(monkeypatch):
    """② 的收尾: 开失败那次**不能**把端口占死 —— 否则修好原因之后重连会被自己挡住
    (那个半成品对象连 `close()` 都不会被调到, 登记没别的释放点)。

    ⚠ `as ei` 是**载重**的, 不是顺手写的: 它复现的是"异常被上层接住并留着"的真实形状
    (记日志/包一层再抛), 而那一刻 `帧 -> ei -> traceback -> 帧` 成环 ⇒ `__init__` 的帧
    (连其中的 `self`) 要等 gc 才消失。只靠弱引用自动释放的话, 这段时间里端口是**被占着**
    的 —— 显式撤登记才让它与 gc 何时跑无关。
    (变异实测: 把 `__init__` except 里那句 `_release_port` 删掉 ⇒ 本用例红。)
    """
    class SerialException(OSError):
        pass

    port = "/dev/fake-excl-open2"
    _install_serial(monkeypatch, _broken_serial(SerialException, "open"), SerialException)

    with pytest.raises(TransportError) as ei:
        SerialTransport(port)
    assert _ORIGINAL_TEXT in str(ei.value)      # ← `ei` 被留到用例结束 (见 docstring)

    _install(monkeypatch, b"")                  # 换成能开成功的桩
    tr = SerialTransport(port)
    assert tr.is_open, "开失败那次把端口占死了 —— 修好原因之后重连会被自己挡住"
    tr.close()


def test_arm_close_and_disconnect_release_the_port_registration(monkeypatch):
    """③ 的**用户面**收尾: `Arm.close()` / `Arm.disconnect()` 之后同一个口必须能重连
    (那是 `Arm.reconnect()` 走的"先关再连"那条路)。

    手工接线 (只测收尾那一格, 不做握手/探测 —— 与本仓既有的手工接线用法同形)。
    `disconnect()` 是 `close()` 的别名, 两条**都咬**: 将来谁把它改成第二套实现 (于是
    少放一次登记) 就得红。两个名字各用**自己的**端口, 免得第一个 `Arm` 的引用环还没被
    gc 收掉就干扰第二个。
    """
    from litearm.arm import Arm

    _install(monkeypatch, b"")
    for closer in ("close", "disconnect"):
        port = f"/dev/fake-excl-arm-{closer}"
        tr = SerialTransport(port)
        arm = Arm(port=port)
        arm._tr = tr

        getattr(arm, closer)()

        assert not tr.is_open, f"Arm.{closer}() 没把链路关掉"
        tr2 = SerialTransport(port)             # ← 登记必须已经放开
        assert tr2.is_open, f"Arm.{closer}() 之后同一个口连不回来 (被自己的登记挡住)"
        tr2.close()


def test_read_failure_is_a_transport_error_that_keeps_the_original_text(monkeypatch):
    """② **读处**: `serial.SerialException` 不许从 `read_frame` 逃出去。"""
    class SerialException(OSError):
        pass

    _install_serial(monkeypatch, _broken_serial(SerialException, "read"), SerialException)
    tr = SerialTransport("/dev/fake-excl-read")
    try:
        with pytest.raises(TransportError) as ei:
            tr.read_frame(0.05)
        assert _ORIGINAL_TEXT in str(ei.value)
    finally:
        tr.close()


def test_write_failure_is_a_transport_error_that_keeps_the_original_text(monkeypatch):
    """② **写处**: 同上 (⚠ 这一处是**载重**的 —— `_CartPending.request()` 靠
    "`TransportError` ⟹ 整帧未送达" 摘 token, 类型不对会静默变成假成功)。"""
    class SerialException(OSError):
        pass

    _install_serial(monkeypatch, _broken_serial(SerialException, "write"), SerialException)
    tr = SerialTransport("/dev/fake-excl-write")
    try:
        with pytest.raises(TransportError) as ei:
            tr.write_frame(P.CMD_GET_STATUS)
        assert _ORIGINAL_TEXT in str(ei.value)
    finally:
        tr.close()


# ---------------------------------------------------------------------------
# 一次读多少字节 —— `_want_bytes` / `_READ_CHUNK_MAX`（2026-09-22 的 33× 修复）
# ---------------------------------------------------------------------------
#
# ⚠ 上面那批用例用的 `_install` fake **没有** `in_waiting` ⇒ 它们跑的是**逐字节**那条
# 路（语义与改动前逐字相同，所以一条都不用改）。**整块读那条路必须有它自己的用例** ——
# 否则"新路径从没被跑过"就是本仓最忌讳的那种绿。

def _close_quietly(*trs):
    """用例失败时也要放掉端口注册。

    ⚠ 不是洁癖：`SerialTransport` 打开时会把端口名**注册**进全局表（同文件后面
    `test_an_open_transport_blocks_the_port_*` 钉的就是它）。一条**该红**的用例若在
    `close()` 之前就断言失败，会把这个注册留在全局 —— 于是它自己多报一条与病因无关的
    `ERROR`，还可能连累后面的用例（本文件某条注释专门讲过这个坑）。
    """
    for t in trs:
        try:
            t.close()
        except Exception:                    # noqa: BLE001 - 收尾不许掩盖原异常
            pass


def _install_with_in_waiting(monkeypatch, data):
    """带 `in_waiting` 的 fake Serial —— **真串口就是这样**（有数据就报数量）。

    与 `_install` 的差别只有两点：多一个 `in_waiting`、`read` 记下**被要了几个字节**。
    """
    st = {"buf": bytearray(data), "read_sizes": []}

    class FakeSerial:
        def __init__(self, port, baudrate=0, timeout=0.2, **kw):
            self.is_open = True
            self.timeout = timeout

        @property
        def in_waiting(self):
            return len(st["buf"])

        def read(self, size=1):
            st["read_sizes"].append(size)
            if not st["buf"]:
                return b""
            out = bytes(st["buf"][:size])
            del st["buf"][:size]
            return out

        def write(self, d):
            return len(d)

        def flush(self):
            pass

        def close(self):
            self.is_open = False

    monkeypatch.setitem(sys.modules, "serial",
                        types.SimpleNamespace(Serial=FakeSerial))
    return st


def test_a_port_reporting_in_waiting_is_drained_in_chunks_not_byte_by_byte(monkeypatch):
    """⭐ 本修复的**判别力所在**：报得出到达量时，一次多取，不许退回逐字节。

    真机实测：这块板子**空闲**时状态流就 ~14 kB/s ⇒ 逐字节读 = 每字节一次
    `select()`+`os.read()` ⇒ 在 2 核机上烧满**一整核**（server 无客户端时也 100.0%）。
    判据取"一帧只该花常数次读"：旧行为对这一帧是 **158 次**。
    """
    frame = mk_status(1)
    st = _install_with_in_waiting(monkeypatch, frame)
    tr = SerialTransport("/dev/null-chunked")

    try:
        fr = tr.read_frame(0.05)
        assert fr is not None and fr[0] == P.RSP_STATUS, "整块读那条路没解出帧"
        assert len(st["read_sizes"]) <= 3, (
            f"一帧读了 {len(st['read_sizes'])} 次 —— 退回逐字节了: {st['read_sizes']}")
        assert max(st["read_sizes"]) > 1, (
            f"从没按到达量取过（一直要 1）: {st['read_sizes']}")
    finally:
        _close_quietly(tr)


def test_the_same_stream_parses_identically_byte_by_byte_and_all_at_once(monkeypatch):
    """⚠ **粒度不许改变解析结果** —— 这条是"整块读"敢上真机的依据。

    同一串字节，分别用【逐字节】与【一次到齐】两种到达方式喂进去，
    解出来的帧序列必须**逐帧相同**（含跨帧的半帧保留、噪声字节跳过）。
    """
    stream = mk_status(1) + mk_status(2) + P.pack_frame(0x34, b"abc") + mk_status(3)

    _install(monkeypatch, stream)                       # 无 in_waiting ⇒ 逐字节
    tr_a = SerialTransport("/dev/null-gran-a")
    byte_by_byte = _drain(tr_a)

    _install_with_in_waiting(monkeypatch, stream)       # 有 ⇒ 一次到齐
    tr_b = SerialTransport("/dev/null-gran-b")
    all_at_once = _drain(tr_b)
    trs = (tr_a, tr_b)

    try:
        assert len(byte_by_byte) == 4, f"逐字节那条没解出 4 帧: {byte_by_byte}"
        assert byte_by_byte == all_at_once, (
            "到达粒度改变了解析结果 —— 整块读不能上真机")
    finally:
        _close_quietly(*trs)


def test_a_port_without_in_waiting_degrades_to_one_byte_reads(monkeypatch):
    """拿不到 `in_waiting` ⇒ **退化成 1**（桩 / 平台不支持时不许抛）。

    这条把"既有桩跑的就是旧的逐字节语义"从**巧合**变成**契约**：
    上面那批老用例全绿，靠的就是这条退化路径。
    """
    _install(monkeypatch, b"")
    tr = SerialTransport("/dev/null-no-inwaiting")
    try:
        assert tr._want_bytes() == 1, "没有 `in_waiting` 时没有退化成 1"
    finally:
        _close_quietly(tr)


def test_a_port_reporting_zero_in_waiting_still_reads(monkeypatch):
    """`in_waiting == 0` 是"**还没到**"，要 **1**（阻塞等），**不是"不读"**。

    ⚠ 方向是承重的：把 0 读成"没数据 ⇒ 直接返回 b''"会让**每一帧的起始字节都得等
    下一次调用**，而 `read_frame(0)` 会因此恒返回 None —— 那正是旧实现踩过的坑
    （`poll_cart()` 在真机上恒 None，而离线全绿）。
    """
    frame = mk_status(1)
    st = {"buf": bytearray(frame), "read_sizes": []}

    class LyingSerial:
        """`in_waiting` **恒报 0**（谎报），但 `read` 照给数据。"""

        def __init__(self, port, baudrate=0, timeout=0.2, **kw):
            self.is_open = True
            self.timeout = timeout

        @property
        def in_waiting(self):
            return 0

        def read(self, size=1):
            st["read_sizes"].append(size)
            if not st["buf"]:
                return b""
            out = bytes(st["buf"][:size])
            del st["buf"][:size]
            return out

        def write(self, d):
            return len(d)

        def close(self):
            self.is_open = False

    monkeypatch.setitem(sys.modules, "serial",
                        types.SimpleNamespace(Serial=LyingSerial))
    tr = SerialTransport("/dev/null-lying")

    try:
        assert tr.read_frame(0.2) is not None, (
            "报 0 时一个字节都不读 —— 帧永远取不到")
        assert set(st["read_sizes"]) == {1}, (
            f"报 0 时不该要多个: {sorted(set(st['read_sizes']))}")
    finally:
        _close_quietly(tr)


def test_the_chunk_size_is_capped_and_odd_reports_are_safe(monkeypatch):
    """封顶 `_READ_CHUNK_MAX`；负数/异常报告一律当 1。"""
    from litearm.transport import _READ_CHUNK_MAX

    st = _install_with_in_waiting(monkeypatch, b"")
    tr = SerialTransport("/dev/null-cap")

    try:
        st["buf"] = bytearray(100_000)
        assert tr._want_bytes() == _READ_CHUNK_MAX, "没有封顶"
        st["buf"] = bytearray(2)
        assert tr._want_bytes() == 2, "到达量 >1 时没有按到达量要"
        st["buf"] = bytearray(1)
        assert tr._want_bytes() == 1, "到达量为 1 时应与从前一致（要 1）"
    finally:
        _close_quietly(tr)
