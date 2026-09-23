"""ACK 必须回显原命令 —— 迟到的旧 ACK 不能被当成新命令的成功。

固件侧 ERR 载荷是 [回显cmd, code], ACK 载荷首字节同样是原命令
(litearm-stm32/User/litearm/hal/usb_cmd.c)。若上位机不比对, 上一条命令超时后
迟到的 ACK 会让下一条命令"假成功"(对 disable/emergency_stop 这类安全命令尤其危险)。
"""
from __future__ import annotations

import struct

import pytest

from litearm import _protocol as P
from litearm.errors import MotionTimeoutError


def _status_frame(n=7):
    body = bytearray(struct.pack("<HH", (1 << 6), 1))       # mode=MOVE_J
    for i in range(n):
        body += struct.pack("<fffff", 0.0, 0.0, 0.5, 30.0, 25.0) + b"\x00"
    body += struct.pack("<H", 0)                            # 1.5.x 布局: joint_fault
    return P.pack_frame(P.RSP_STATUS, bytes(body))


class _Stub:
    """握手正常; 对指定命令按脚本回帧 (可回别的命令的 ACK 以模拟"迟到旧 ACK")。"""

    def __init__(self, replies, port="stub", timeout=0.2):
        # ⚠ 固件应答**不许预置**：真机上应答只可能在写**之后**到达，预置等于让桩
        # 违反线序。而 `Arm._raw_write` 会在发帧前清掉本命令的应答队列（那是防
        # "陈旧帧冒充新应答"的闸）⇒ 预置的固件帧会被正确地当成陈旧帧丢掉，握手超时。
        # 状态帧可以预置：它走**单槽**不走队列，不受清队影响。
        self._q = [_status_frame()]
        self._replies = replies          # dict[cmd] = list[bytes]
        self.closed = False

    @property
    def is_open(self):
        return not self.closed

    def close(self):
        self.closed = True

    def write_frame(self, cmd, payload=b""):
        if cmd == P.CMD_GET_FIRMWARE:
            # 应答在**写之后**产生（线序），见 `__init__` 那段。
            self._q.append(P.pack_frame(P.RSP_FIRMWARE, b"Litearm1.5.2-7J"))
            return
        for fr in self._replies.get(cmd, []):
            self._q.append(fr)

    def read_frame(self, timeout=None):
        if self._q:
            return P.unpack_frame(self._q.pop(0))
        return None


def _make_arm(monkeypatch, replies):
    import litearm.arm as arm_mod

    monkeypatch.setattr(arm_mod, "SerialTransport",
                        lambda port=None, timeout=0.2: _Stub(replies, port))
    from litearm import Arm
    return Arm(port="stub").connect()


def test_foreign_ack_does_not_satisfy_command(monkeypatch):
    """只回了别的命令的 ACK -> 必须超时失败, 不能算成功。"""
    arm = _make_arm(monkeypatch, {P.CMD_DISABLE: [P.pack_frame(P.RSP_ACK, bytes([P.CMD_MOVE_J]))]})
    with pytest.raises(MotionTimeoutError):
        arm.disable()


def test_matching_ack_accepted_after_foreign(monkeypatch):
    """先来一个别的命令的 ACK, 再来正确的 -> 必须成功 (不能过度严格)。"""
    arm = _make_arm(monkeypatch, {P.CMD_DISABLE: [
        P.pack_frame(P.RSP_ACK, bytes([P.CMD_MOVE_J])),
        P.pack_frame(P.RSP_ACK, bytes([P.CMD_DISABLE])),
    ]})
    arm.disable()          # 不抛即成功


def test_emergency_stop_requires_own_echo(monkeypatch):
    """急停同样不能被别的命令的 ACK 顶替。"""
    arm = _make_arm(monkeypatch, {P.CMD_EMERGENCY_STOP: [
        P.pack_frame(P.RSP_ACK, bytes([P.CMD_ENABLE])),
    ]})
    with pytest.raises(MotionTimeoutError):
        arm.emergency_stop()


def test_foreign_err_does_not_fail_current_command(monkeypatch):
    """迟到的**别的命令**的 ERR 不能算到当前命令头上 (ACK 已过滤, ERR 也要过滤)。

    否则 enable() 会对无关 ERR 触发无意义重试, 连续伺服会打断一条本来有效的命令。
    """
    arm = _make_arm(monkeypatch, {P.CMD_DISABLE: [
        P.pack_frame(P.RSP_ERR, bytes([P.CMD_MOVE_J, 0x05])),   # 别人的 ERR
        P.pack_frame(P.RSP_ACK, bytes([P.CMD_DISABLE])),
    ]})
    arm.disable()                       # 不应被别人的 ERR 打断


def test_own_err_still_raises(monkeypatch):
    from litearm.errors import CommandRejectedError
    arm = _make_arm(monkeypatch, {P.CMD_DISABLE: [
        P.pack_frame(P.RSP_ERR, bytes([P.CMD_DISABLE, 0x03])),
    ]})
    with pytest.raises(CommandRejectedError):
        arm.disable()
