"""Small pytest support for the optional local-kinematics regression tests."""
from __future__ import annotations

import pytest

from litearm import Arm
from litearm.testing import FakeTransport


@pytest.fixture
def offline_arm():
    transport = FakeTransport()
    arm = Arm(transport_factory=lambda _port: transport, move_timeout=2.0)
    arm.connect("fake")
    try:
        yield arm
    finally:
        arm.close()
