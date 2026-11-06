"""SDK-local FK/IK/Jacobian and fail-closed PTP motion."""
from __future__ import annotations

import importlib.util
import math
import copy

import pytest

import litearm as pa
from litearm import _protocol as P


pytestmark = pytest.mark.skipif(
    importlib.util.find_spec("pinocchio") is None,
    reason="optional robotics dependency is not installed")


SAFE_Q = [0.0, -0.5, 0.0, -1.0, 0.0, 0.5, 0.0]
TARGET_Q = [0.15, -0.55, 0.10, -1.10, 0.05, 0.55, -0.05]


def test_fk_zero_matches_firmware_model_golden():
    arm = pa.Arm()
    pose = arm.kinematics.fk([0.0] * 7, frame="flange")
    assert pose[:3] == pytest.approx(
        [-9.21449129e-7, 2.52723928e-6, 0.782999783], abs=2e-7)
    assert pose[3:] == pytest.approx(
        [-3.463023e-6, 8.229795e-6, 1.570792654], abs=2e-6)


def test_local_jacobian_matches_fk_finite_difference():
    import numpy as np

    arm = pa.Arm()
    kin = arm.kinematics
    J = np.asarray(kin.jacobian(SAFE_Q, frame="flange").matrix)
    eps = 1e-6
    p0 = np.asarray(kin.fk(SAFE_Q, frame="flange")[:3])
    for joint in range(7):
        q = list(SAFE_Q)
        q[joint] += eps
        dp = (np.asarray(kin.fk(q, frame="flange")[:3]) - p0) / eps
        np.testing.assert_allclose(dp, J[:3, joint], atol=2e-5)
    assert J.shape == (6, 7)
    assert kin.jacobian(SAFE_Q, frame="flange").sigma_min > 0.05


def test_local_ik_round_trip_stays_on_seeded_branch():
    arm = pa.Arm()
    target = arm.kinematics.fk(TARGET_Q, frame="flange")
    solution = arm.kinematics.ik(target, q_seed=SAFE_Q, frame="flange")
    reached = arm.kinematics.fk(solution.q, frame="flange")
    assert reached[:3] == pytest.approx(target[:3], abs=5e-4)
    assert solution.position_error < 5e-4
    assert solution.orientation_error < 5e-3
    assert solution.max_joint_delta < 0.35
    assert solution.sigma_min >= 0.05


def test_zero_configuration_is_rejected_as_singular():
    arm = pa.Arm()
    pose = arm.kinematics.fk([0.0] * 7, frame="flange")
    with pytest.raises(pa.LocalKinematicsError) as caught:
        arm.kinematics.ik(pose, q_seed=[0.0] * 7, frame="flange")
    assert caught.value.reason == "near_singular"
    assert caught.value.sigma_min < 0.05


def test_small_cartesian_step_from_zero_cannot_trigger_large_rebranch():
    arm = pa.Arm()
    target = list(arm.kinematics.fk([0.0] * 7, frame="flange"))
    target[0] += 0.005
    with pytest.raises(pa.LocalKinematicsError) as caught:
        arm.kinematics.ik(target, q_seed=[0.0] * 7, frame="flange")
    assert caught.value.reason in ("no_solution", "branch_jump", "near_singular")


def test_tool_frame_fk_and_ik_share_tcp_offset():
    arm = pa.Arm()
    arm.set_tcp_offset((0.0, 0.0, 0.10))
    flange = arm.kinematics.fk(SAFE_Q, frame="flange")
    tool = arm.kinematics.fk(SAFE_Q, frame="tool")
    assert math.dist(flange[:3], tool[:3]) == pytest.approx(0.10, abs=1e-7)
    solution = arm.kinematics.ik(tool, q_seed=SAFE_Q, frame="tool")
    assert solution.max_joint_delta < 1e-8


def test_move_pose_sends_only_one_synchronized_joint_target(offline_arm):
    arm = offline_arm
    arm._tr.q = list(SAFE_Q)
    arm.enable()
    arm.get_state(refresh=True)
    target = arm.kinematics.fk(TARGET_Q, frame="flange")
    start = len(arm._tr.tx_log)
    state = arm.kinematics.move_pose(target, speed=0.2, frame="flange")
    sent = arm._tr.tx_log[start:]
    motion = [(cmd, payload) for cmd, payload in sent
              if cmd in (P.CMD_MOVE_J, P.CMD_MOVE_J_SYNC, P.CMD_GET_IK)]
    assert [cmd for cmd, _ in motion] == [P.CMD_MOVE_J_SYNC]
    assert state.q == pytest.approx(P.unpack_f32s(motion[0][1], 0, 7), abs=1e-6)


def test_move_pose_failure_sends_no_motion(offline_arm, monkeypatch):
    arm = offline_arm
    arm._tr.q = list(SAFE_Q)
    arm.enable()
    arm.get_state(refresh=True)
    target = arm.kinematics.fk(TARGET_Q, frame="flange")

    def reject(*_args, **_kwargs):
        raise pa.LocalKinematicsError("注入碰撞", reason="collision")

    monkeypatch.setattr(arm.kinematics, "_check_path", reject)
    start = len(arm._tr.tx_log)
    with pytest.raises(pa.LocalKinematicsError, match="注入碰撞"):
        arm.kinematics.move_pose(target, frame="flange")
    assert not any(cmd in (P.CMD_MOVE_J, P.CMD_MOVE_J_SYNC, P.CMD_MOVE_P,
                           P.CMD_MOVE_L, P.CMD_MOVE_C, P.CMD_GET_IK)
                   for cmd, _ in arm._tr.tx_log[start:])


def test_move_pose_rejects_nonfinite_target_without_motion(offline_arm):
    arm = offline_arm
    arm._tr.q = list(SAFE_Q)
    arm.enable()
    start = len(arm._tr.tx_log)
    with pytest.raises(pa.InvalidCommandError, match="非有限"):
        arm.kinematics.move_pose((math.nan, 0, 0, 0, 0, 0), frame="flange")
    assert not any(cmd in (P.CMD_MOVE_J, P.CMD_MOVE_J_SYNC, P.CMD_GET_IK)
                   for cmd, _ in arm._tr.tx_log[start:])


def test_move_pose_missing_fresh_state_sends_no_motion(offline_arm, monkeypatch):
    arm = offline_arm
    monkeypatch.setattr(
        arm, "get_state", lambda **_kwargs: pa.Msg(None, 0.0, 0.0))
    start = len(arm._tr.tx_log)
    with pytest.raises(pa.TransportError, match="取不到最新状态帧"):
        arm.kinematics.move_pose((0, 0, 0, 0, 0, 0), frame="flange")
    assert not any(cmd in (P.CMD_MOVE_J, P.CMD_MOVE_J_SYNC, P.CMD_GET_IK)
                   for cmd, _ in arm._tr.tx_log[start:])


def test_move_pose_aborts_after_start_moves_twice(offline_arm, monkeypatch):
    arm = offline_arm
    arm._tr.q = list(SAFE_Q)
    arm.enable()
    base = arm.get_state(refresh=True).value

    def state_at(q0):
        state = copy.deepcopy(base)
        for joint, value in zip(state.joints, q0):
            joint.q = value
        return pa.Msg(state, 0.0, 0.0)

    moved1 = list(SAFE_Q)
    moved1[0] += 0.04
    moved2 = list(moved1)
    moved2[0] += 0.04
    replies = iter((state_at(SAFE_Q).value, state_at(moved1).value,
                    state_at(moved1).value, state_at(moved2).value))
    monkeypatch.setattr(arm.kinematics, "_fresh_state", lambda: next(replies))
    solution = pa.IKSolution(tuple(TARGET_Q), 0.0, 0.0, 0.1, 0.1, 1)
    solves = []
    monkeypatch.setattr(arm.kinematics, "_solve",
                        lambda *_args, **_kwargs: solves.append(1) or solution)
    monkeypatch.setattr(arm.kinematics, "_check_path", lambda *_args, **_kwargs: None)

    start = len(arm._tr.tx_log)
    with pytest.raises(pa.LocalKinematicsError) as caught:
        arm.kinematics.move_pose((0, 0, 0, 0, 0, 0), frame="flange")
    assert caught.value.reason == "start_moved"
    assert len(solves) == 2
    assert not any(cmd in (P.CMD_MOVE_J, P.CMD_MOVE_J_SYNC, P.CMD_GET_IK)
                   for cmd, _ in arm._tr.tx_log[start:])


def test_collision_checker_exposes_clearance():
    checker = pa.CollisionModel()
    assert checker.minimum_distance(SAFE_Q) > 0.005
    folded = [-0.753, 1.4236, 0.1053, -2.6041, -0.0948, -0.7629, 0.4072]
    assert checker.minimum_distance(folded) <= 0.0
