"""300Hz 控制拍采集 (固件 0x2D/0x2E + RSP_LOG_DATA 0x4D) —— 此前 SDK 无任何入口。

固件侧 (`hal/log_capture.h` + `usb_cmd_dispatch` 的 `case CMD_LOG_CTRL`/`CMD_LOG_READ`,
`usb_cmd.c:960-984`):
  0x2D LOG_CTRL  : u32 n_ticks(f32? 否 —— u32 LE); 0=停/清; n>LOG_MAX_SAMPLES(2400) -> ERR{0x2D,0x02}
  0x2E LOG_READ  : u32 offset_byte LE -> RSP_LOG_DATA(0x4D)
                   data = [u32 total_bytes][u32 next_byte][u8 n][bytes n], n<=245;
                   **next_byte == 0 表示读完** (掉帧时按 next_byte 续读)
  样本布局       : u32 tick + q_ref[N]f32 + dq[N]f32 + tau[N]f32 = 4 + 12N B (7J = 88B)
"""
from __future__ import annotations

import struct

import pytest

from litearm import _protocol as P
from litearm.errors import (
    CommandRejectedError,
    MotionTimeoutError,
    TransportError,
    UnsupportedByFirmwareError,
)


def _payloads(arm, cmd):
    return [p for c, p in list(arm._tr.tx_log) if c == cmd]


def test_start_sends_n_ticks_as_u32_le(offline_arm):
    arm = offline_arm
    arm.log.start(600)
    ps = _payloads(arm, P.CMD_LOG_CTRL)
    assert len(ps) == 1 and len(ps[0]) == 4
    assert struct.unpack_from("<I", ps[0], 0)[0] == 600


def test_stop_sends_zero(offline_arm):
    arm = offline_arm
    arm.log.start(600)
    arm.log.stop()
    assert struct.unpack_from("<I", _payloads(arm, P.CMD_LOG_CTRL)[-1], 0)[0] == 0


def test_start_rejects_over_capacity(offline_arm):
    """超容量固件回 ERR{0x2D,0x02}; 不得静默。"""
    arm = offline_arm
    with pytest.raises(CommandRejectedError) as ei:
        arm.log.start(99999)
    assert ei.value.code == 0x02


def test_capture_round_trips_all_bytes(offline_arm):
    """读回的字节流必须与固件缓冲逐字节一致 (分块拼装无丢失/错位)。"""
    arm = offline_arm
    arm.log.start(600)
    blob = arm.log.reader().read_all()
    assert blob == arm._tr.log_bytes
    assert len(blob) == 600 * (4 + 12 * arm.n)


def test_capture_parses_samples(offline_arm):
    arm = offline_arm
    samples = arm.log.capture(50)
    assert len(samples) == 50
    assert [s.tick for s in samples] == list(range(50))
    assert samples[0].q_ref == tuple(float(10 * j + 1) for j in range(arm.n))
    assert samples[0].dq == tuple([0.5] * arm.n)
    assert samples[0].tau == tuple([-0.25] * arm.n)


def test_capture_survives_dropped_chunk(offline_arm):
    """USB 掉帧 (整块读回应答丢失) -> 必须按游标重试, 而不是丢一段数据。"""
    arm = offline_arm
    arm.log.start(600)
    arm._tr.log_read_fail_once = True
    r = arm.log.reader(timeout=0.05, retries=3)
    assert r.read_all() == arm._tr.log_bytes


def test_reader_on_empty_log_returns_nothing(offline_arm):
    arm = offline_arm
    arm.log.stop()                      # n=0: 停/清
    assert arm.log.reader().read_all() == b""


def test_truncated_blob_is_rejected(offline_arm):
    """字节数不是样本整数倍 -> 明确报错, 不能悄悄解析出半截样本。"""
    arm = offline_arm
    arm.log.start(10)
    arm._tr.log_bytes = arm._tr.log_bytes[:-3]      # 截断到非整数倍
    with pytest.raises(TransportError):
        arm.log.reader().samples()


def test_dump_writes_raw_bytes(offline_arm, tmp_path):
    arm = offline_arm
    arm.log.start(20)
    out = tmp_path / "cap.bin"
    arm.log.dump(str(out))
    assert out.read_bytes() == arm._tr.log_bytes


def test_capture_waits_until_recording_completes(offline_arm):
    """固件是**逐拍**记录的 (300Hz, `log_capture_tick` 一拍一条):

        log_capture_start(600) 之后缓冲里是空的, 要 2 秒才录满。

    `capture(n)` 若 start() 后立刻读回, 只能拿到 0 或一个前缀 —— 必须等到录满。
    """
    arm = offline_arm
    arm._tr.log_hz = 2000.0                   # 2000 拍/秒 -> 600 拍约 0.3s 录满
    samples = arm.log.capture(600, record_timeout=5.0)
    assert len(samples) == 600, (
        f"只拿到 {len(samples)}/600 拍 —— capture() 没等固件录满就读回了")
    assert [s.tick for s in samples] == list(range(600))


def test_dump_waits_for_the_started_capture(offline_arm, tmp_path):
    """`start(200)` 后立刻 dump -> 不能落盘一个空/半截文件 (同一类逐拍记录陷阱)。"""
    arm = offline_arm
    arm._tr.log_hz = 2000.0                   # 200 拍约 0.1s 录满
    arm.log.start(200)
    out = tmp_path / "c.bin"
    arm.log.dump(str(out))
    assert len(out.read_bytes()) == 200 * (4 + 12 * arm.n)


def test_dump_can_skip_waiting_explicitly(offline_arm, tmp_path):
    """要读"此刻已录到多少"时, 显式关掉等待。"""
    arm = offline_arm
    arm._tr.log_hz = 20.0                     # 200 拍要 10s; 不等 -> 立刻读只能拿到前缀
    arm.log.start(200)
    out = tmp_path / "c.bin"
    arm.log.dump(str(out), wait=False)
    assert len(out.read_bytes()) < 200 * (4 + 12 * arm.n), "不该已经录满"""


def test_capture_raises_when_recording_never_finishes(offline_arm):
    """录不满必须报错, 不能返回半截数据冒充成功。"""
    arm = offline_arm
    arm._tr.log_hz = 5.0                      # 5 拍/秒 -> 600 拍要 120s, 必然等不满
    with pytest.raises(MotionTimeoutError) as ei:
        arm.log.capture(600, record_timeout=0.3)
    assert "600" in str(ei.value)


def test_reader_rejects_non_advancing_cursor(offline_arm):
    """固件回 `next_byte == offset` (无进展) 时必须报错 —— 否则读者死循环。"""
    arm = offline_arm
    arm.log.start(10)
    arm._tr.log_cursor_stuck = True
    with pytest.raises(TransportError):
        arm.log.reader().read_all()


def test_log_unsupported_on_old_firmware(offline_arm):
    arm = offline_arm
    arm._tr.unknown_cmds.add(P.CMD_LOG_CTRL)
    with pytest.raises(UnsupportedByFirmwareError):
        arm.log.start(10)
