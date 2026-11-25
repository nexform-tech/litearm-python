"""Minimal offline smoke tests for the public API."""

import math
import os
import re
import threading
import time
import unittest

import litearm
from litearm import (
    Arm,
    CartesianPlanError,
    CommandRejectedError,
    ForkedSessionError,
    LiteArmError,
    NotConnectedError,
    UnsupportedByFirmwareError,
    mat_to_rpy,
    rpy_to_mat,
)
from litearm.testing import FakeTransport
from litearm import _protocol as P
from litearm import errors as E


class PublicApiTest(unittest.TestCase):
    def test_version(self):
        self.assertEqual(litearm.__version__, "2.4.1")

    def test_error_hierarchy(self):
        self.assertTrue(issubclass(UnsupportedByFirmwareError, CommandRejectedError))
        self.assertTrue(issubclass(ForkedSessionError, NotConnectedError))
        self.assertTrue(issubclass(CartesianPlanError, LiteArmError))
        self.assertFalse(issubclass(CartesianPlanError, CommandRejectedError))

    def test_pose_roundtrip(self):
        rpy = (0.2, -0.3, 1.1)
        rot = rpy_to_mat(rpy)
        back = mat_to_rpy(rot)
        for got, want in zip(back, rpy):
            self.assertTrue(math.isclose(got, want, abs_tol=1e-9))

        # A rotation matrix is orthonormal with determinant +1.
        for i in range(3):
            for j in range(3):
                dot = sum(rot[k][i] * rot[k][j] for k in range(3))
                self.assertTrue(math.isclose(dot, 1.0 if i == j else 0.0, abs_tol=1e-9))

    def test_fake_transport_session(self):
        arm = Arm(transport_factory=lambda port: FakeTransport(), move_timeout=2.0)
        arm.connect("fake")
        try:
            state = arm.get_state()
            self.assertIsNotNone(state.value)
            self.assertEqual(len(state.value.q), 7)
            self.assertGreaterEqual(state.hz, 0.0)
        finally:
            arm.close()

    def test_all_positive_motion_entry_points_share_the_serial_lock(self):
        q = [0.0] * 7
        cases = (
            ("home", P.CMD_HOME, lambda arm: arm.home(timeout=1.0)),
            ("move_js", P.CMD_MOVE_JS, lambda arm: arm.move_js(q)),
            ("send_mit", P.CMD_MOVE_MIT,
             lambda arm: arm.send_mit(0, 0.0, 0.0, 1.0, 0.1, 0.0)),
            ("send_mit_all", P.CMD_MOVE_MIT_ALL,
             lambda arm: arm.send_mit_all(q, q, q, q, q)),
            ("joint_follow", P.CMD_JOINT_FOLLOW,
             lambda arm: arm.joint_follow(q, q, q, q)),
        )
        for name, command, invoke in cases:
            with self.subTest(name=name):
                transport = FakeTransport()
                arm = Arm(transport_factory=lambda _port: transport,
                          move_timeout=1.0).connect("fake")
                entered = threading.Event()
                finished = threading.Event()
                failures = []

                def run():
                    entered.set()
                    try:
                        invoke(arm)
                    except BaseException as exc:
                        failures.append(exc)
                    finally:
                        finished.set()

                try:
                    with arm._motion_serial:
                        start = len(transport.tx_log)
                        worker = threading.Thread(target=run, daemon=True)
                        worker.start()
                        self.assertTrue(entered.wait(1.0))
                        time.sleep(0.03)
                        self.assertFalse(finished.is_set())
                        self.assertFalse(any(cmd == command
                                             for cmd, _ in transport.tx_log[start:]))
                    worker.join(2.0)
                    self.assertTrue(finished.is_set())
                    self.assertFalse(failures)
                    self.assertTrue(any(cmd == command
                                        for cmd, _ in transport.tx_log[start:]))
                finally:
                    arm.close()

    def test_estop_and_disable_bypass_the_motion_lock(self):
        transport = FakeTransport()
        arm = Arm(transport_factory=lambda _port: transport).connect("fake")
        try:
            with arm._motion_serial:
                start = len(transport.tx_log)
                arm.emergency_stop()
                arm.disable()
            commands = [cmd for cmd, _ in transport.tx_log[start:]]
            self.assertEqual(commands, [P.CMD_EMERGENCY_STOP, P.CMD_DISABLE])
        finally:
            arm.close()


#: Every command with a dedicated (non-ACK/ERR) reply -> that reply id.
#: `0x40 GET_STATUS` is absent: it uses a single slot, not a queue.
#: The cartesian commands `0x3A~0x3E` are absent: `RSP_CART_PLAN(0x4E)` is owned
#: by the cart module. A missing entry here means a late reply can be mistaken
#: for the result of the next query.
_DEDICATED_REPLIES = {
    P.CMD_GET_JOINT_PARAM: P.RSP_JOINT_PARAM,
    P.CMD_GET_FF_VEC: P.RSP_FF_VEC,
    P.CMD_GET_FF_SCALAR: P.RSP_FF_SCALAR,
    P.CMD_LOG_READ: P.RSP_LOG_DATA,
    P.CMD_GET_LICENSE: P.RSP_LICENSE,
    P.CMD_GET_MODEL_PARAM: P.RSP_MODEL_PARAM,
    P.CMD_GET_MODEL_JM: P.RSP_MODEL_JM,
    P.CMD_GET_MODEL_STATUS: P.RSP_MODEL_STATUS,
    P.CMD_GET_GRAVITY: P.RSP_GRAVITY,
    P.CMD_GET_FIRMWARE: P.RSP_FIRMWARE,
    P.CMD_GET_IK: P.RSP_IK,
    P.CMD_GET_TCP: P.RSP_TCP,
    P.CMD_KIN_BENCH: P.RSP_KIN_BENCH,
    P.CMD_GET_FAULT: P.RSP_FAULT,
}


class DedicatedReplyTest(unittest.TestCase):
    def test_dedicated_replies_are_cleared_before_reuse(self):
        self.assertEqual(P.RSP_OF_CMD, _DEDICATED_REPLIES)

    def test_drain_for_clears_fault_and_license_queues(self):
        arm = Arm(transport_factory=lambda _port: FakeTransport(),
                  move_timeout=2.0).connect("fake")
        try:
            agent = arm._a
            for cmd, rsp in ((P.CMD_GET_FAULT, P.RSP_FAULT),
                             (P.CMD_GET_LICENSE, P.RSP_LICENSE)):
                key = (rsp, None)
                agent._queues[key] = [(rsp, b"stale", 0)]
                agent.drain_for(cmd)
                self.assertNotIn(key, agent._queues,
                                 f"drain_for(0x{cmd:02X}) left {key!r}")
        finally:
            arm.close()


# --------------------------------------------------------------------------
# Item-set drift guard: the "valid set" spelled out in ERR_TEXT must match
# what the firmware source actually accepts. The firmware is an input here,
# so these run only when LITEARM_FW_DIR points at a checkout.
# --------------------------------------------------------------------------
_FW_DIR = os.environ.get("LITEARM_FW_DIR") or os.path.expanduser("~/litearm-stm32")
_FW_SUBDIR = os.path.join("User", "litearm")


def _fw_path(*parts):
    return os.path.join(_FW_DIR, _FW_SUBDIR, *parts)


def _read_fw(*parts):
    with open(_fw_path(*parts), encoding="utf-8", errors="replace") as f:
        return f.read()


def _fmt_set(items):
    """{1..11,15,16} -> '1..11、15、16' (the exact spelling used by ERR_TEXT)."""
    xs = sorted(items)
    out, i = [], 0
    while i < len(xs):
        j = i
        while j + 1 < len(xs) and xs[j + 1] == xs[j] + 1:
            j += 1
        if j - i >= 2:
            out.append(f"{xs[i]}..{xs[j]}")
        else:
            out.append("、".join(map(str, xs[i:j + 1])))
        i = j + 1
    return "、".join(out)


def _vec_item_set_26():
    """0x26 valid items = joint branch {1..6,9..11,15,16} union dyn {7,8}."""
    src = _read_fw("params", "params.c")
    m = re.search(r"if\s*\(\s*item\s*<\s*1\s*\|\|\s*item\s*>\s*(\d+)\s*\)", src)
    assert m, "0x26 range guard not found in firmware params.c"
    hi = int(m.group(1))
    m = re.search(r"if\s*\(\(item\s*>=\s*1.*?item\s*==\s*16\)", src, re.S)
    assert m, "0x26 joint-branch guard not found in firmware params.c"
    valid = set()
    for lo, hi2 in re.findall(
            r"item\s*>=\s*(\d+)\s*&&\s*item\s*<=\s*(\d+)", m.group(0)):
        valid |= set(range(int(lo), int(hi2) + 1))
    for eq in re.findall(r"item\s*==\s*(\d+)", m.group(0)):
        valid.add(int(eq))
    assert "case 7:" in src and "gravity_scale" in src, "dyn item7 missing"
    assert "case 8:" in src and "inertia_scale" in src, "dyn item8 missing"
    return set(range(1, hi + 1)) & (valid | {7, 8})


def _scalar_item_set_28():
    """0x28 valid items = case labels of params_ff_scalar (default rejects)."""
    src = _read_fw("params", "params.c")
    m = re.search(r"bool\s+params_ff_scalar\s*\([^)]*\)\s*\{(.*?)\n\}", src, re.S)
    assert m, "params_ff_scalar not found in firmware params.c"
    return set(map(int, re.findall(r"case\s+(\d+)\s*:", m.group(1))))


def _usb_item_set(cmd_symbol, end_symbol):
    """0x2B/0x2C valid items = range upper bound minus explicit rejects."""
    src = _read_fw("hal", "usb_cmd.c")
    block = src[src.index(f"case {cmd_symbol}:"):src.index(f"case {end_symbol}:")]
    m = re.search(r"item\s*<\s*1\s*\|\|\s*item\s*>\s*(\d+)", block)
    assert m, f"item range guard not found in firmware {cmd_symbol} branch"
    hi = int(m.group(1))
    tail = re.split(r"switch\s*\(|uint8_t\s+out|float\s+val", block[m.end():])[0]
    rej = set()
    for lo, hi2 in re.findall(
            r"item\s*>=\s*(\d+)\s*&&\s*item\s*<=\s*(\d+)", tail):
        rej |= set(range(int(lo), int(hi2) + 1))
    for eq in re.findall(r"item\s*==\s*(\d+)", tail):
        rej.add(int(eq))
    return set(range(1, hi + 1)) - rej


@unittest.skipUnless(os.path.isdir(_FW_DIR),
                     "firmware source not present; set LITEARM_FW_DIR")
class FirmwareItemSetTest(unittest.TestCase):
    def _check(self, cmd, derive):
        items = derive()
        self.assertIn(
            _fmt_set(items), E.ERR_TEXT[(cmd, 0x02)],
            f"{{0x{cmd:02X},0x02}} item set is stale: firmware={_fmt_set(items)}; "
            f"text={E.ERR_TEXT[(cmd, 0x02)]!r}")

    def test_0x26_item_set(self):
        self._check(0x26, _vec_item_set_26)

    def test_0x28_item_set(self):
        self._check(0x28, _scalar_item_set_28)

    def test_0x2b_item_set(self):
        self._check(0x2B, lambda: _usb_item_set("CMD_GET_FF_VEC", "CMD_GET_FF_SCALAR"))

    def test_0x2c_item_set(self):
        self._check(0x2C, lambda: _usb_item_set("CMD_GET_FF_SCALAR", "CMD_LOG_CTRL"))


if __name__ == "__main__":
    unittest.main()
