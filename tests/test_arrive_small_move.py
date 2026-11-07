"""小位移到位判据的离线回归 (FakeTransport, 不需要硬件)。"""
from __future__ import annotations

import threading
import time
import unittest

from litearm import _protocol as P
from litearm import Arm
from litearm.testing import FakeTransport


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
                state = arm.movej(target, speed=0.3)
            finally:
                transport.write_frame = real_write
                if feeder is not None:
                    feeder.join(2.0)
            for got, want in zip(state.q, target):
                self.assertAlmostEqual(got, want, places=6)
            self.assertEqual(state.faulted, False)
        finally:
            arm.close()


if __name__ == "__main__":
    unittest.main()
