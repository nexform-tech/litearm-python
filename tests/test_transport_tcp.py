"""TCP 传输 —— 契约与串口逐条对齐 (见 `litearm/transport_tcp.py` 模块 docstring)。

这些用例**不依赖**任何仿真后端: 一个本地 socket 假装成固件, 只验传输层本身。
"""
from __future__ import annotations

import contextlib
import socket
import threading
import time

import pytest

from litearm import _protocol as P
from litearm import transport as tr_mod
from litearm.errors import TransportError
from litearm.transport import is_tcp_target
from litearm.transport_tcp import TcpTransport, is_tcp_url, parse_tcp_url


@contextlib.contextmanager
def fake_firmware():
    """起一个只监听 127.0.0.1 的"固件", 返回 (url, 服务端 socket)。"""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    port = srv.getsockname()[1]
    accepted = {}

    def _accept():
        conn, _ = srv.accept()
        accepted["conn"] = conn

    th = threading.Thread(target=_accept, daemon=True)
    th.start()
    try:
        yield f"tcp://127.0.0.1:{port}", accepted
    finally:
        th.join(timeout=2.0)
        for s in (accepted.get("conn"), srv):
            if s is not None:
                try:
                    s.close()
                except OSError:
                    pass


def _conn(box, timeout=2.0):
    """等服务端 accept 落地, 返回那条连接 (构造 TcpTransport 是异步完成的)。"""
    end = time.time() + timeout
    while "conn" not in box and time.time() < end:
        time.sleep(0.005)
    assert "conn" in box, "服务端没接受连接"
    return box["conn"]


# --------------------------------------------------------------------------- #
# 端口字符串的判定
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("port", ["tcp://127.0.0.1:5601", "sim://localhost:9000",
                                  "127.0.0.1:5601", "host:1"])
def test_tcp_urls_are_recognized(port):
    assert is_tcp_url(port) is True


@pytest.mark.parametrize("port", ["/dev/ttyACM0", "COM3", "fake", "", None, 42,
                                  "C:\\Users\\x", "host:abc", "host:70000", ":5601"])
def test_non_tcp_ports_are_not_misdetected(port):
    assert is_tcp_url(port) is False, f"{port!r} 不应被判成 TCP"


def test_parse_rejects_bad_urls():
    for bad in ("tcp://noport", "tcp://:123", "tcp://h:notanumber", "tcp://h:99999"):
        with pytest.raises(ValueError):
            parse_tcp_url(bad)


def test_parse_accepts_scheme_less_form():
    assert parse_tcp_url("127.0.0.1:5601") == ("127.0.0.1", 5601)
    assert parse_tcp_url("sim://localhost:7") == ("localhost", 7)


# --------------------------------------------------------------------------- #
# 收发契约
# --------------------------------------------------------------------------- #
def test_roundtrip_write_then_read():
    with fake_firmware() as (url, accepted):
        tr = TcpTransport(url, timeout=0.5)
        try:
            tr.write_frame(P.CMD_GET_STATUS)
            _conn(accepted).settimeout(2.0)
            data = _conn(accepted).recv(64)
            assert P.unpack_frame(data) == (P.CMD_GET_STATUS, b"")
            _conn(accepted).sendall(P.pack_frame(P.RSP_FIRMWARE, b"Litearm1.10.0-7J"))
            got = tr.read_frame(timeout=2.0)
            assert got == (P.RSP_FIRMWARE, b"Litearm1.10.0-7J")
        finally:
            tr.close()


def test_port_is_kept_verbatim_for_connect_idempotency():
    """`Arm.connect()` 的幂等判据读的是 `transport.port`, 必须是原样的 URL。"""
    with fake_firmware() as (url, accepted):
        tr = TcpTransport(url)
        try:
            assert tr.port == url
            assert isinstance(tr.port, str)
        finally:
            tr.close()


def test_half_frame_survives_a_timeout():
    """半帧超时**不丢**: 下一次读要能接着把它装配出来 (串口的同名契约)。"""
    with fake_firmware() as (url, accepted):
        tr = TcpTransport(url, timeout=0.05)
        try:
            frame = P.pack_frame(P.RSP_TCP, P.pack_f32s([0.1, 0.2, 0.3, 0.0, 0.0, 0.0]))
            conn = _conn(accepted)
            conn.sendall(frame[:6])
            assert tr.read_frame(timeout=0.15) is None, "半帧不该装配成功"
            conn.sendall(frame[6:])
            got = tr.read_frame(timeout=1.0)
            assert got is not None and got[0] == P.RSP_TCP
        finally:
            tr.close()


def test_timeout_zero_returns_what_already_arrived():
    """`timeout=0` 是"有就给我" —— `Arm.poll_cart()` 靠它。"""
    with fake_firmware() as (url, accepted):
        tr = TcpTransport(url, timeout=0.2)
        try:
            _conn(accepted).sendall(P.pack_frame(P.RSP_ACK, bytes([P.CMD_ENABLE])))
            time.sleep(0.05)                       # 让字节真的到达内核缓冲
            got = tr.read_frame(0)
            assert got == (P.RSP_ACK, bytes([P.CMD_ENABLE]))
        finally:
            tr.close()


def test_noise_bytes_go_to_text_log():
    with fake_firmware() as (url, accepted):
        tr = TcpTransport(url, timeout=0.2)
        try:
            _conn(accepted).sendall(b"[litearm-usbcdc] ready (normal)")
            assert tr.read_frame(timeout=0.2) is None
            assert "ready" in tr.text_log
        finally:
            tr.close()


def test_peer_close_raises_instead_of_silently_timing_out():
    """对端关闭必须**报链路断**: 静默超时会把归因指向"固件不理我"。"""
    with fake_firmware() as (url, accepted):
        tr = TcpTransport(url, timeout=0.5)
        try:
            _conn(accepted).close()
            with pytest.raises(TransportError):
                for _ in range(50):
                    tr.read_frame(timeout=0.05)
        finally:
            tr.close()


def test_write_after_close_raises_transport_error():
    with fake_firmware() as (url, accepted):
        tr = TcpTransport(url)
        tr.close()
        with pytest.raises(TransportError):
            tr.write_frame(P.CMD_GET_STATUS)
        assert tr.is_open is False
        tr.close()                                  # 幂等


def test_connect_failure_is_a_transport_error():
    """端口没人监听时, 必须是本包的 TransportError (调用方按它做分支)。"""
    with pytest.raises(TransportError):
        TcpTransport("tcp://127.0.0.1:1", connect_timeout=0.5)


# --------------------------------------------------------------------------- #
# 分发 (Arm.connect 的分支)
# --------------------------------------------------------------------------- #
def test_is_tcp_target_delegates_to_the_same_rules():
    assert is_tcp_target("tcp://127.0.0.1:5601") is True
    assert is_tcp_target("127.0.0.1:5601") is True
    assert is_tcp_target("/dev/ttyACM0") is False
    assert is_tcp_target("fake") is False


def test_arm_keeps_using_serial_transport_for_non_tcp_ports(monkeypatch):
    """串口分支必须**仍然直接引用 `arm.SerialTransport`** —— 那是离线测试与
    `litearm-server --dry-run` 的注入点 (tests/conftest.py 靠替换它注桩)。
    """
    from litearm import arm as arm_mod
    from litearm.testing import FakeTransport

    made = []

    def factory(port):
        made.append(port)
        return FakeTransport()

    monkeypatch.setattr(arm_mod, "SerialTransport", factory)
    arm_mod.Arm(port="fake").connect().close()
    assert made == ["fake"], "非 TCP 端口没有走 SerialTransport 注入点"


def test_arm_routes_a_tcp_url_to_the_tcp_transport(monkeypatch):
    from litearm import arm as arm_mod

    monkeypatch.setattr(arm_mod, "SerialTransport",
                        lambda *a, **k: pytest.fail("TCP 目标不该去开串口"))
    with pytest.raises(TransportError):
        # 端口没人监听 -> 传输层报 TransportError, 但**不是**串口路径
        arm_mod.Arm(port="tcp://127.0.0.1:1").connect()
