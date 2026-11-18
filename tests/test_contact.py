"""Contact protection command surface (firmware 1.13.0), offline."""
from __future__ import annotations

import struct

try:
    import pytest
except ImportError:
    import unittest
    raise unittest.SkipTest("pytest is not installed")

from litearm import _protocol as P
from litearm.errors import (
    ERR_TEXT,
    CommandRejectedError,
    InvalidCommandError,
    err_reason,
)


def _payloads(arm, cmd):
    return [payload for sent, payload in arm._tr.tx_log if sent == cmd]


def _last(arm, cmd):
    hits = _payloads(arm, cmd)
    assert hits, f"0x{cmd:02X} 没有发出"
    return hits[-1]


def test_set_contact_config_payload_layout(offline_arm):
    arm = offline_arm
    threshold = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]
    hard = [1.5, 3.0, 4.5, 6.0, 7.5, 9.0, 10.5]
    arm.set_contact_config(
        True, min_joints=2, debounce=4, hard_debounce=3, leak=2.5,
        ko=[31.0] * 7, deadband=[0.1] * 7,
        threshold=threshold, hard_threshold=hard)

    payload = _last(arm, P.CMD_SET_CONTACT_CONFIG)
    assert len(payload) == 120
    assert payload[0:4] == bytes([1, 2, 4, 3])
    assert struct.unpack_from("<f", payload, 4)[0] == pytest.approx(2.5)
    assert list(struct.unpack_from("<7f", payload, 8)) == [31.0] * 7
    assert list(struct.unpack_from("<7f", payload, 36)) == pytest.approx([0.1] * 7)
    assert list(struct.unpack_from("<7f", payload, 64)) == threshold
    assert list(struct.unpack_from("<7f", payload, 92)) == hard


def test_contact_on_requires_calibrated_thresholds(offline_arm):
    """The generic table must never be armed silently: it is uncalibrated and
    the earlier table false-triggered on a real slow MoveJ (J4 1.016 vs 0.76)."""
    arm = offline_arm
    with pytest.raises(InvalidCommandError):
        arm.contact_on()
    assert not _payloads(arm, P.CMD_SET_CONTACT_CONFIG)

    arm.contact_on(allow_default_thresholds=True)
    assert len(_last(arm, P.CMD_SET_CONTACT_CONFIG)) == 120


def test_contact_config_validates_vectors_and_bounds(offline_arm):
    arm = offline_arm
    with pytest.raises(InvalidCommandError):
        arm.set_contact_config(threshold=[1.0] * 6)
    with pytest.raises(InvalidCommandError):
        arm.set_contact_config(threshold=[1.0] * 7, hard_threshold=[0.5] * 7)
    with pytest.raises(InvalidCommandError):
        arm.set_contact_config(min_joints=0)
    with pytest.raises(InvalidCommandError):
        arm.set_contact_config(debounce=0)
    assert not _payloads(arm, P.CMD_SET_CONTACT_CONFIG)


def test_contact_off_writes_disabled_config(offline_arm):
    offline_arm.contact_off()
    assert _last(offline_arm, P.CMD_SET_CONTACT_CONFIG)[0] == 0


def test_get_contact_state_decodes_64_bytes(offline_arm):
    arm = offline_arm
    tr = arm._tr
    tr.contact_phase = 4
    tr.contact_enabled = True
    tr.contact_latched = True
    tr.contact_hard = False
    tr.contact_mask = 0x0005
    tr.contact_samples = 200
    tr.contact_residual = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7]
    tr.contact_peak = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]

    msg = arm.get_contact_state()
    value = msg.value
    assert value["phase"] == 4
    assert value["phase_name"] == "LATCHED_HOLD"
    assert value["enabled"] is True
    assert value["latched"] is True
    assert value["hard"] is False
    assert value["joint_mask"] == 0x0005
    assert value["calibration_samples"] == 200
    assert value["residual"] == pytest.approx(tr.contact_residual)
    assert value["peak"] == pytest.approx(tr.contact_peak)


def test_set_contact_mode_payload_and_validation(offline_arm):
    arm = offline_arm
    arm.set_contact_mode(1, 7, static_detect=False, auto_clear=True,
                         auto_clear_ms=1500)
    assert _last(arm, P.CMD_SET_CONTACT_MODE) == (
        bytes([1, 7, 0x02]) + struct.pack("<H", 1500))

    with pytest.raises(InvalidCommandError):
        arm.set_contact_mode(3, 6)
    with pytest.raises(InvalidCommandError):
        arm.set_contact_mode(0, 9)
    with pytest.raises(InvalidCommandError):
        arm.set_contact_mode(0, 6, auto_clear=True, auto_clear_ms=50)

    # The documented names are accepted alongside the raw protocol values.
    arm.set_contact_mode("warn", 6)
    assert _last(arm, P.CMD_SET_CONTACT_MODE)[0] == 1
    arm.set_contact_mode("hard-only", 6)
    assert _last(arm, P.CMD_SET_CONTACT_MODE)[0] == 2
    with pytest.raises(InvalidCommandError):
        arm.set_contact_mode("nope", 6)


def test_get_contact_mode_decodes_effective_thresholds(offline_arm):
    arm = offline_arm
    tr = arm._tr
    tr.contact_response = 1
    tr.contact_level = 7
    tr.contact_static = False
    tr.contact_auto_clear = True
    tr.contact_auto_ms = 1200
    tr.contact_warn_mask = 0x0003
    tr.contact_resume_available = True
    tr.contact_threshold = [5.0] * 7
    tr.contact_hard_threshold = [12.5] * 7

    value = arm.get_contact_mode().value
    assert value["response"] == 1
    assert value["response_name"] == "warn"
    assert value["level"] == 7
    assert value["static_detect"] is False
    assert value["auto_clear"] is True
    assert value["auto_clear_ms"] == 1200
    assert value["warn_mask"] == 0x0003
    assert value["resume_available"] is True
    assert value["threshold"] == [5.0] * 7
    assert value["hard_threshold"] == [12.5] * 7


def test_calibrate_clear_and_resume_are_sent(offline_arm):
    arm = offline_arm
    arm.enable()
    arm.contact_on(allow_default_thresholds=True)
    arm.contact_calibrate()
    arm.contact_clear()
    arm.contact_resume()
    for cmd in (P.CMD_CONTACT_CALIBRATE, P.CMD_CLEAR_CONTACT,
                P.CMD_CONTACT_RESUME):
        assert _payloads(arm, cmd)[-1] == b""


def test_calibrate_before_enable_reports_firmware_code(offline_arm):
    arm = offline_arm
    with pytest.raises(CommandRejectedError) as excinfo:
        arm.contact_calibrate()
    assert excinfo.value.cmd == P.CMD_CONTACT_CALIBRATE
    assert excinfo.value.code == 0x03


def test_contact_commands_are_allowed_in_zero_g(offline_arm):
    """0x16/0x1A only store configuration while zero-g is active; the firmware
    defers calibration. The SDK must not reject them host-side, while motion
    commands stay guarded."""
    arm = offline_arm
    arm.enable()
    arm.zero_g_start()
    try:
        arm.set_contact_config(False)
        arm.set_contact_mode(0, 0)
        assert _payloads(arm, P.CMD_SET_CONTACT_CONFIG)
        assert _payloads(arm, P.CMD_SET_CONTACT_MODE)
        with pytest.raises(InvalidCommandError):
            arm.movej([0.0] * 7)
    finally:
        arm.zero_g_stop()


def test_contact_error_codes_have_specific_text():
    for cmd in range(0x16, 0x1D):
        hits = [code for (c, code) in ERR_TEXT if c == cmd]
        assert hits, f"0x{cmd:02X} 没有任何具体错误码文案"
    assert "锁存" in err_reason(0x16, 0x06)
    assert "0x16" in err_reason(0x17, 0x07)
    assert "0.05" in err_reason(0x1C, 0x07)
