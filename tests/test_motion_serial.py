"""Positive motion is serialized, while energy-reducing commands stay reachable."""
from __future__ import annotations

import threading
import time

import pytest

from litearm import _protocol as P


def test_positive_motion_waits_but_estop_bypasses_motion_lock(offline_arm):
    arm = offline_arm
    arm.enable()
    target = [0.05, -0.10, 0.05, -0.20, 0.0, 0.10, 0.0]
    entered = threading.Event()
    finished = threading.Event()
    failures = []

    def move():
        entered.set()
        try:
            arm.movej(target, speed=0.2)
        except BaseException as exc:  # pragma: no cover - included in assertion below
            failures.append(exc)
        finally:
            finished.set()

    with arm._motion_serial:
        start = len(arm._tr.tx_log)
        worker = threading.Thread(target=move, name="serialized-move", daemon=True)
        worker.start()
        assert entered.wait(1.0)
        time.sleep(0.05)
        assert not finished.is_set()
        assert not any(cmd == P.CMD_MOVE_J for cmd, _ in arm._tr.tx_log[start:])

        arm.emergency_stop()
        assert any(cmd == P.CMD_EMERGENCY_STOP for cmd, _ in arm._tr.tx_log[start:])
        assert not finished.is_set()

    worker.join(2.0)
    assert finished.is_set()
    assert not failures
    sent = [cmd for cmd, _ in arm._tr.tx_log[start:]
            if cmd in (P.CMD_EMERGENCY_STOP, P.CMD_MOVE_J)]
    assert sent == [P.CMD_EMERGENCY_STOP, P.CMD_MOVE_J]


def test_disable_bypasses_motion_lock(offline_arm):
    arm = offline_arm
    arm.enable()
    with arm._motion_serial:
        start = len(arm._tr.tx_log)
        arm.disable()
    assert [cmd for cmd, _ in arm._tr.tx_log[start:]] == [P.CMD_DISABLE]


@pytest.mark.parametrize(("expected", "invoke"), [
    (P.CMD_HOME, lambda arm: arm.home(timeout=1.0)),
    (P.CMD_MOVE_JS, lambda arm: arm.move_js([0.0] * arm.n)),
    (P.CMD_MOVE_MIT,
     lambda arm: arm.send_mit(0, 0.0, 0.0, 1.0, 0.1, 0.0)),
    (P.CMD_MOVE_MIT_ALL,
     lambda arm: arm.send_mit_all(*([[0.0] * arm.n] * 5))),
    (P.CMD_JOINT_FOLLOW,
     lambda arm: arm.joint_follow(*([[0.0] * arm.n] * 4))),
])
def test_remaining_positive_motion_entry_points_are_serialized(
        offline_arm, expected, invoke):
    arm = offline_arm
    entered = threading.Event()
    finished = threading.Event()
    failures = []

    def send():
        entered.set()
        try:
            invoke(arm)
        except BaseException as exc:  # pragma: no cover - asserted below
            failures.append(exc)
        finally:
            finished.set()

    with arm._motion_serial:
        start = len(arm._tr.tx_log)
        worker = threading.Thread(target=send, daemon=True)
        worker.start()
        assert entered.wait(1.0)
        time.sleep(0.03)
        assert not finished.is_set()
        assert not any(cmd == expected for cmd, _ in arm._tr.tx_log[start:])

    worker.join(2.0)
    assert finished.is_set()
    assert not failures
    assert any(cmd == expected for cmd, _ in arm._tr.tx_log[start:])
