"""台架 1J 上的 `ik()` —— 固件 IK 的 seed 恒为 7 轴, 与 `LITEARM_NUM_JOINTS` 解耦。

固件 `usb_cmd_dispatch` 的 `case CMD_GET_IK` (`usb_cmd.c:574-575`):
    /* pose[6] + q_seed[7] = 52B ... seed 恒为模型 7 轴 (KIN_N), 与
     * LITEARM_NUM_JOINTS 解耦 */

而旧 SDK 在 `q_seed=None` 时直接拿 `st.q` 当种子 —— 台架上 `st.q` 只有 1 个元素,
随即在长度校验处抛 `InvalidCommandError("q_seed 需 7 个")`, **台架 IK 完全不可用**。

台架那台电机对应模型第 `LITEARM_BENCH_MODEL_AXIS` 轴 (joint_cfg.h: 台架=5, 整臂=0),
故种子应构造为: 7 个 0, 其中台架轴填当前实测值。
"""
from __future__ import annotations

import pytest

from litearm import _protocol as P
from litearm.errors import InvalidCommandError


def _ik_payload(arm):
    ps = [p for c, p in list(arm._tr.tx_log) if c == P.CMD_GET_IK]
    assert ps, "未发出 GET_IK"
    return ps[-1]


def test_ik_on_bench_sends_seven_axis_seed(offline_arm_1j):
    """台架上 ik() 必须能发出 52B 载荷 (pose[6] + seed[7]), 而不是本地就炸。"""
    arm = offline_arm_1j
    assert arm.n == 1
    q = arm.ik((0.30, 0.0, 0.35, 0.0, 0.0, 0.0))
    assert len(q) == 7

    p = _ik_payload(arm)
    assert len(p) == 52, f"GET_IK 载荷应为 52B, 实为 {len(p)}B"
    seed = P.unpack_f32s(p, 24, 7)
    assert len(seed) == 7


def test_ik_on_bench_seeds_the_bench_axis_from_feedback(offline_arm_1j):
    """台架轴 (LITEARM_BENCH_MODEL_AXIS=5) 的种子应来自实测, 其余轴为 0。"""
    arm = offline_arm_1j
    arm._tr.q = [0.42]                      # 台架电机当前位形
    arm.get_state(refresh=True)             # 让 SDK 拿到该反馈
    arm.ik((0.30, 0.0, 0.35, 0.0, 0.0, 0.0))

    seed = P.unpack_f32s(_ik_payload(arm), 24, 7)
    assert seed[P.BENCH_MODEL_AXIS] == pytest.approx(0.42, abs=1e-6), \
        "台架轴种子未取自实测反馈"
    assert [v for i, v in enumerate(seed) if i != P.BENCH_MODEL_AXIS] == [0.0] * 6


def test_ik_on_full_arm_still_seeds_from_current_q(offline_arm):
    """整臂行为不变: 7 关节直接用当前 q 当种子。"""
    arm = offline_arm
    arm._tr.q = [0.1 * (i + 1) for i in range(7)]
    arm.get_state(refresh=True)
    arm.ik((0.30, 0.0, 0.35, 0.0, 0.0, 0.0))
    seed = P.unpack_f32s(_ik_payload(arm), 24, 7)
    assert seed == pytest.approx([0.1 * (i + 1) for i in range(7)], abs=1e-6)


def test_explicit_seed_still_honoured(offline_arm_1j):
    arm = offline_arm_1j
    my_seed = [0.0, 0.0, 0.0, 0.0, 0.0, 0.9, 0.0]
    arm.ik((0.30, 0.0, 0.35, 0.0, 0.0, 0.0), q_seed=my_seed)
    assert P.unpack_f32s(_ik_payload(arm), 24, 7) == pytest.approx(my_seed)


def test_explicit_seed_must_be_seven(offline_arm_1j):
    arm = offline_arm_1j
    with pytest.raises(InvalidCommandError):
        arm.ik((0.30, 0.0, 0.35, 0.0, 0.0, 0.0), q_seed=[0.1])
