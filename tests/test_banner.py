"""开机签名 (banner) —— 固件在 USB 枚举完成后发一次, 是**唯一能区分 IWDG 复位**的信息。

固件 `usb_cmd_report` (枚举后单发, `usb_cmd.c:1297-1303`):
    sig_ok   = "\\r\\n[litearm-usbcdc] ready (" LITEARM_FW_VERSION ")\\r\\n"
    sig_iwdg = "\\r\\n[litearm-usbcdc] ready (" LITEARM_FW_VERSION ", iwdg-rst)\\r\\n"
即独立看门狗复位过的那次启动会带 `, iwdg-rst` 标注 (原实现在 init 里三连发, 未枚举时
必丢, 该标注从来到不了上位机; L9 fix 移到枚举后单发)。

这段文本不是帧 (不以 SOF 0xA5 开头), 旧实现把它当噪声逐字节丢弃。现改为在丢弃路径
上留痕, 供上层解析。
"""
from __future__ import annotations

import struct
import sys
import time
import types

from litearm import _protocol as P
from litearm.transport import SerialTransport

BANNER_OK = "\r\n[litearm-usbcdc] ready (Litearm1.7.0-7J)\r\n"
BANNER_IWDG = "\r\n[litearm-usbcdc] ready (Litearm1.7.0-7J, iwdg-rst)\r\n"


# ---------------------------------------------------------------- 纯解析
def test_parse_banner_normal_boot():
    assert P.parse_boot_banner(BANNER_OK) == "normal"


def test_parse_banner_iwdg_reset():
    assert P.parse_boot_banner(BANNER_IWDG) == "iwdg-rst"


def test_parse_banner_absent_or_noise():
    assert P.parse_boot_banner("") is None
    assert P.parse_boot_banner("\x11\x22random noise\x00") is None


def test_parse_banner_picks_latest_when_repeated():
    """重连/多次枚举会重复出现; 取最后一次。"""
    assert P.parse_boot_banner(BANNER_IWDG + "junk" + BANNER_OK) == "normal"


# ---------------------------------------------------------------- 传输层留痕
def mk_status(seq):
    body = bytearray(struct.pack("<HH", (1 << 6), seq))
    for i in range(7):
        body += struct.pack("<fffff", 0.0, 0.0, 0.0, 30.0, 25.0) + b"\x00"
    body += struct.pack("<H", 0)
    return P.pack_frame(P.RSP_STATUS, bytes(body))


def _install(monkeypatch, data):
    st = {"buf": bytearray(data)}

    class FakeSerial:
        def __init__(self, port, baudrate=0, timeout=0.2, **kw):
            self.is_open = True
            self.timeout = timeout

        def read(self, size=1):
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

    monkeypatch.setitem(sys.modules, "serial", types.SimpleNamespace(Serial=FakeSerial))


def test_transport_keeps_banner_text_while_skipping_noise(monkeypatch):
    """banner 不是帧; 传输层必须在丢噪声的同时把它留下来 (否则信息永久丢失)。"""
    _install(monkeypatch, BANNER_IWDG.encode() + mk_status(1) + b"\x7f\x7f")
    tr = SerialTransport("/dev/null")
    got = None
    end = time.monotonic() + 1.0
    while got is None and time.monotonic() < end:
        got = tr.read_frame(0.05)
    assert got is not None, "错过正常状态帧"
    assert "litearm-usbcdc" in tr.text_log
    assert "iwdg-rst" in tr.text_log


def test_transport_text_log_is_bounded(monkeypatch):
    """噪声可能无限多, 留痕必须有上限 (不能吃内存)。"""
    _install(monkeypatch, b"x" * 20000)
    tr = SerialTransport("/dev/null")
    tr.read_frame(0.05)
    assert len(tr.text_log) <= 2048


def test_arm_exposes_last_reset_reason(offline_arm):
    arm = offline_arm
    arm._tr.text_log = BANNER_IWDG
    assert arm.last_reset_reason == "iwdg-rst"
    arm._tr.text_log = BANNER_OK
    assert arm.last_reset_reason == "normal"
    arm._tr.text_log = ""
    assert arm.last_reset_reason is None
