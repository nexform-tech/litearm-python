"""小位移到位判据的离线回归 (FakeTransport, 不需要硬件)。"""
from __future__ import annotations

import threading
import time
import unittest
import warnings

from litearm import _protocol as P
from litearm import Arm
from litearm.errors import MotionTimeoutError
from litearm.testing import FakeTransport


def _run_plateau_step(delta, residual, move_timeout=2.0, q_tol=0.03):
    """推 10 拍"停在起点", 再持续推"停在离目标 residual 处"的平台帧。

    返回 `(state, elapsed_s, caught_warnings, target)`。
    """
    transport = FakeTransport()
    arm = Arm(transport_factory=lambda _port: transport,
              move_timeout=move_timeout, q_tol=q_tol)
    arm.connect("fake")
    feeder = None
    real_write = transport.write_frame
    try:
        arm.enable()
        start = [0.0] * transport.n
        target = [delta] * transport.n
        plateau = [delta - residual] * transport.n

        def write(cmd, payload=b""):
            nonlocal feeder
            if cmd != P.CMD_MOVE_J:
                return real_write(cmd, payload)
            transport.tx_log.append((cmd, bytes(payload)))
            transport.tx_stamps.append(time.monotonic())
            transport._push(P.pack_frame(P.RSP_ACK, bytes([cmd])))

            def feed():
                for _ in range(10):
                    transport._push(
                        transport._status_frame(start, dq=[0.0] * transport.n))
                    time.sleep(0.01)
                transport.q = list(plateau)
                for _ in range(100):          # 1 s, 足够越过 200 ms 稳定窗
                    transport._push(
                        transport._status_frame(plateau, dq=[0.0] * transport.n))
                    time.sleep(0.01)

            feeder = threading.Thread(target=feed, daemon=True)
            feeder.start()

        transport.write_frame = write
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            t0 = time.monotonic()
            state = arm.movej(target, speed=0.3)
            elapsed = time.monotonic() - t0
        return state, elapsed, list(caught), target
    finally:
        transport.write_frame = real_write
        if feeder is not None:
            feeder.join(2.0)
        arm.close()


class SmallMoveArrivalTest(unittest.TestCase):
    def test_small_move_does_not_arrive_before_the_arm_moves(self):
        """3 mm 级微动 (≈0.008 rad) 不能被名义 `q_tol=0.03` 判成"出发即到位"。

        真机实测 (2026-11-07, `Litearm1.11.1-7J`): 3 mm `move_pose` 时臂还停在
        起点 (偏差 0.008 rad, 名义容差内), 连续 3 拍静止 (30 ms) 就判到位, 返回于
        0.154 s, 而物理运动 0.18 s 才起步、0.94 s 才停稳。`_arrive` 现在要求小位移
        已走过一半且再静 `SMALL_MOVE_SETTLE_S`。

        本用例让桩先推 10 拍"停在起点", 再推到位帧: 旧实现会拿着起点的 q 返回,
        新实现必须等到目标。
        """
        transport = FakeTransport()
        arm = Arm(transport_factory=lambda _port: transport, move_timeout=2.0)
        arm.connect("fake")
        try:
            arm.enable()
            start = [0.0] * transport.n
            target = [0.008] * transport.n
            real_write = transport.write_frame
            feeder = None

            def write(cmd, payload=b""):
                nonlocal feeder
                if cmd != P.CMD_MOVE_J:
                    return real_write(cmd, payload)
                transport.tx_log.append((cmd, bytes(payload)))
                transport.tx_stamps.append(time.monotonic())
                transport._push(P.pack_frame(P.RSP_ACK, bytes([cmd])))

                def feed():
                    # 按真机 100Hz 的节奏推帧: 先 10 拍"还停在起点", 再到位.
                    # 旧实现会在这 10 拍里凑满 3 拍静止, 拿着起点的 q 返回.
                    for _ in range(10):
                        transport._push(
                            transport._status_frame(start, dq=[0.0] * transport.n))
                        time.sleep(0.01)
                    transport.q = list(target)
                    for _ in range(6):
                        transport._push(
                            transport._status_frame(target, dq=[0.0] * transport.n))
                        time.sleep(0.01)

                feeder = threading.Thread(target=feed, daemon=True)
                feeder.start()

            transport.write_frame = write
            try:
                with warnings.catch_warnings(record=True) as caught:
                    warnings.simplefilter("always")
                    state = arm.movej(target, speed=0.3)
            finally:
                transport.write_frame = real_write
                if feeder is not None:
                    feeder.join(2.0)
            for got, want in zip(state.q, target):
                self.assertAlmostEqual(got, want, places=6)
            self.assertEqual(state.faulted, False)
            self.assertFalse(
                any("平台态" in str(w.message) for w in caught),
                "正常到位不应触发平台态告警")
        finally:
            arm.close()

    def test_platform_residual_returns_with_warning_instead_of_timeout(self):
        """修正型小位移: 物理停稳在离目标 ~0.02 rad 处应判到位并告警, 不抛超时。

        真机回归 (2026-11-24, `Litearm1.12.3-7J`): `reset`→`enable`→`movej(READY)`
        (delta 0.0444, 稳态残差 0.0187) 与 `send_mit` 退出→`movej(READY)`
        (delta 0.0233, 稳态残差 0.0210) 在收紧容差下都会耗满 `move_timeout`。
        """
        for delta, residual in ((0.0444, 0.0187), (0.0233, 0.0210)):
            with self.subTest(delta=delta, residual=residual):
                state, elapsed, caught, target = _run_plateau_step(delta, residual)
                got = max(abs(state.q[i] - target[i]) for i in range(len(target)))
                self.assertAlmostEqual(got, residual, places=3)
                self.assertGreaterEqual(
                    elapsed, 0.25, f"delta={delta}: 没等满 200 ms 稳定窗就返回")
                self.assertLess(elapsed, 1.5, f"delta={delta}: 回落成了超时等待")
                self.assertTrue(
                    any("平台态" in str(w.message) for w in caught),
                    f"delta={delta}: 平台态放行缺少 RuntimeWarning")

    def test_plateau_respects_explicitly_tightened_q_tol(self):
        """平台态上界是调用方的 `q_tol`: 显式收紧到 0.01 时 0.021 的残差必须超时。"""
        with self.assertRaises(MotionTimeoutError):
            _run_plateau_step(0.0233, 0.0210, move_timeout=0.6, q_tol=0.01)

    def test_platform_residual_at_start_still_times_out(self):
        """臂没离开起点 (残差也在 q_tol 内) 不能靠平台态提前放行。

        半位移门必须继续挡住原始 bug: 平台态只对"已走过一半且停稳"的帧生效。
        """
        delta = 0.0233
        transport = FakeTransport()
        arm = Arm(transport_factory=lambda _port: transport, move_timeout=0.4)
        arm.connect("fake")
        feeder = None
        real_write = transport.write_frame
        try:
            arm.enable()
            start = [0.0] * transport.n

            def write(cmd, payload=b""):
                nonlocal feeder
                if cmd != P.CMD_MOVE_J:
                    return real_write(cmd, payload)
                transport._push(P.pack_frame(P.RSP_ACK, bytes([cmd])))

                def feed():
                    for _ in range(100):      # 一整段都停在起点
                        transport._push(
                            transport._status_frame(start, dq=[0.0] * transport.n))
                        time.sleep(0.01)

                feeder = threading.Thread(target=feed, daemon=True)
                feeder.start()

            transport.write_frame = write
            with self.assertRaises(MotionTimeoutError):
                arm.movej([delta] * transport.n, speed=0.3)
        finally:
            transport.write_frame = real_write
            if feeder is not None:
                feeder.join(2.0)
            arm.close()


if __name__ == "__main__":
    unittest.main()
