"""Minimal offline smoke tests for the public API."""

import math
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


class PublicApiTest(unittest.TestCase):
    def test_version(self):
        self.assertEqual(litearm.__version__, "2.1.0")

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


if __name__ == "__main__":
    unittest.main()
