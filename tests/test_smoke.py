"""Minimal offline smoke tests for the public API."""

import math
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


class PublicApiTest(unittest.TestCase):
    def test_version(self):
        self.assertEqual(litearm.__version__, "2.2.0")

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


if __name__ == "__main__":
    unittest.main()
