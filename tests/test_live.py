"""真机 live 用例 —— 需 LITEARM_LIVE=1 + 接好 Litearm1.5.0+ 整臂(或台架)。

风险自担: 会 enable 并小幅 move_j (当前姿态 +小量, 不影响他人/障碍时跑)。
"""
from __future__ import annotations

import pytest

from litearm import Arm


def test_live_connect_read(live_only):
    arm = Arm().connect()
    try:
        assert arm.firmware.startswith("Litearm")
        assert arm.n in (1, 7)
        assert arm.get_state().value is not None
        assert arm.get_tcp().value is not None
    finally:
        arm.close()


def test_live_movej_tcp_ik(live_only):
    arm = Arm().connect()
    try:
        arm.enable()
        st = arm.get_state().value
        tgt = list(st.q)
        idx = 5 if arm.n == 7 else 0
        tgt[idx] += 0.04 if idx == 5 else 0.10
        arm.movej(tgt, speed=0.2)
        st2 = arm.get_state().value
        assert all(abs(st2.q[i] - tgt[i]) < 0.06 for i in range(arm.n))
        tcp = arm.get_tcp().value
        assert tcp is not None
        arm.disable()
    finally:
        try:
            arm.disable()
        finally:
            arm.close()
