"""Positive motion is serialized, while energy-reducing commands stay reachable."""
from __future__ import annotations

import threading
import time

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
