"""Tests for litearm.arm using InProcTransport mock server.

Simulates a litearm-server by registering a queryable handler that
processes RPC requests and returns canned responses.
"""
import pytest
import msgpack

from litearm.arm import Arm
from litearm import codec, protocol
from litearm.exceptions import (
    LiteArmError,
    SafetyViolationError,
    MotionCancelled,
)
from litearm.transport import InProcTransport
from litearm.types import ArmState, JointTrajectory, TrajectoryFrame


# ── Mock server helper ───────────────────────────────────────────────────────

def _make_mock_arm(arm_id="armA"):
    """Create an Arm connected via InProcTransport with a mock server.

    Returns (arm, transport) so tests can publish state broadcasts.
    """
    tp = InProcTransport()
    rpc_topic = protocol.rpc_topic(arm_id)

    def mock_rpc_handler(payload: bytes) -> bytes:
        method, kwargs = codec.decode_request(payload)

        # Dispatch to mock implementations
        if method == "fk":
            q = kwargs["q"]
            pos = [0.5, 0.0, 0.3]
            R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
            return codec.encode_reply(ok=True, result=[pos, R])

        elif method == "ik":
            pos_d = kwargs["pos_d"]
            q_sol = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
            return codec.encode_reply(ok=True, result=[q_sol, True])

        elif method == "plan_movel":
            path = [[0.0] * 7, [0.1] * 7, [0.2] * 7]
            return codec.encode_reply(ok=True, result=path)

        elif method == "movej":
            return codec.encode_reply(ok=True, result=True)

        elif method == "movel":
            return codec.encode_reply(ok=True, result=True)

        elif method == "movec":
            return codec.encode_reply(ok=True, result=True)

        elif method == "movep":
            return codec.encode_reply(ok=True, result=True)

        elif method == "hold":
            return codec.encode_reply(ok=True, result=True)

        elif method == "zero_gravity":
            return codec.encode_reply(ok=True, result=True)

        elif method == "joint_impedance":
            return codec.encode_reply(ok=True, result=True)

        elif method == "cartesian_impedance":
            return codec.encode_reply(ok=True, result=True)

        elif method == "joint_follow":
            return codec.encode_reply(ok=True, result=True)

        elif method == "replay_joint_path":
            return codec.encode_reply(ok=True, result=True)

        elif method == "replay_trajectory":
            return codec.encode_reply(ok=True, result=True)

        elif method == "record_trajectory":
            # Return a minimal trajectory dict
            traj = JointTrajectory(
                frames=[
                    TrajectoryFrame(t=0.0, q=[0.0] * 7),
                    TrajectoryFrame(t=0.1, q=[0.01] * 7),
                    TrajectoryFrame(t=0.2, q=[0.02] * 7),
                ],
                name="test_recording",
                sample_rate_hz=100.0,
                filter_alpha=0.15,
            )
            return codec.encode_reply(ok=True, result=traj.to_dict())

        elif method == "get_tcp_pose":
            return codec.encode_reply(ok=True, result=[[0.5, 0.0, 0.3], [[1, 0, 0], [0, 1, 0], [0, 0, 1]]])

        elif method == "clear_stop":
            return codec.encode_reply(ok=True, result=None)

        elif method == "set_gains":
            return codec.encode_reply(ok=True, result={"kp": kwargs.get("kp"), "kd": kwargs.get("kd")})

        elif method == "get_gains":
            return codec.encode_reply(ok=True, result={"kp": [100.0] * 7, "kd": [5.0] * 7})

        elif method == "clear_faults":
            return codec.encode_reply(ok=True, result=[])

        elif method == "set_payload":
            return codec.encode_reply(ok=True, result={"mass": kwargs["mass"], "com": kwargs["com"]})

        elif method == "get_payload":
            return codec.encode_reply(ok=True, result={"mass": 0.0, "com": [0.0, 0.0, 0.0]})

        elif method == "set_installation":
            return codec.encode_reply(ok=True, result={"base_rpy": kwargs.get("base_rpy"), "gravity": kwargs.get("gravity")})

        elif method == "get_installation":
            return codec.encode_reply(ok=True, result={"base_rpy": [0, 0, 0], "gravity": [0, 0, -9.81]})

        else:
            return codec.encode_reply(ok=False, error=LiteArmError(f"Unknown method: {method}"))

    tp.declare_queryable(rpc_topic, mock_rpc_handler)
    arm = Arm(arm_id=arm_id, transport=tp)
    return arm, tp


# ── Tests ────────────────────────────────────────────────────────────────────

class TestFkIk:
    def test_fk(self):
        arm, tp = _make_mock_arm()
        pos, R = arm.fk([0.0] * 7)
        assert pos == [0.5, 0.0, 0.3]
        assert R == [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

    def test_ik(self):
        arm, tp = _make_mock_arm()
        q, success = arm.ik([0.5, 0.0, 0.3], [[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        assert success is True
        assert len(q) == 7

    def test_plan_movel(self):
        arm, tp = _make_mock_arm()
        path = arm.plan_movel([0.0] * 7, {"pos": [0.5, 0.0, 0.3]})
        assert len(path) == 3
        assert path[0] == [0.0] * 7


class TestMotionExecution:
    def test_movej(self):
        arm, tp = _make_mock_arm()
        result = arm.movej([0.0] * 7, speed=0.5)
        assert result is True

    def test_movel(self):
        arm, tp = _make_mock_arm()
        result = arm.movel({"pos": [0.5, 0.0, 0.3]}, speed=0.5)
        assert result is True

    def test_movec(self):
        arm, tp = _make_mock_arm()
        result = arm.movec(
            {"pos": [0.4, 0.1, 0.3]},
            {"pos": [0.5, 0.0, 0.3]},
            speed=0.5,
        )
        assert result is True

    def test_movep(self):
        arm, tp = _make_mock_arm()
        poses = [{"pos": [0.4, 0.0, 0.3]}, {"pos": [0.5, 0.0, 0.3]}]
        result = arm.movep(poses, speed=0.5)
        assert result is True

    def test_hold(self):
        arm, tp = _make_mock_arm()
        assert arm.hold(kp_scale=3.0) is True

    def test_zero_gravity(self):
        arm, tp = _make_mock_arm()
        assert arm.zero_gravity() is True

    def test_joint_impedance(self):
        arm, tp = _make_mock_arm()
        result = arm.joint_impedance(
            q_des=[0.0] * 7, K=[100.0] * 7, B=[10.0] * 7
        )
        assert result is True

    def test_cartesian_impedance(self):
        arm, tp = _make_mock_arm()
        result = arm.cartesian_impedance(
            q_des=[0.0] * 7,
            K_cart=[500.0] * 6,
            B_cart=[50.0] * 6,
        )
        assert result is True

    def test_joint_follow(self):
        arm, tp = _make_mock_arm()
        result = arm.joint_follow(duration_s=5.0)
        assert result is True

    def test_replay_joint_path(self):
        arm, tp = _make_mock_arm()
        q_path = [[0.0] * 7, [0.1] * 7, [0.2] * 7]
        result = arm.replay_joint_path(q_path, speed=0.5)
        assert result is True

    def test_replay_trajectory(self):
        arm, tp = _make_mock_arm()
        traj = JointTrajectory(
            frames=[
                TrajectoryFrame(t=0.0, q=[0.0] * 7),
                TrajectoryFrame(t=0.1, q=[0.01] * 7),
                TrajectoryFrame(t=0.2, q=[0.02] * 7),
            ],
        )
        result = arm.replay_trajectory(traj, speed=0.5)
        assert result is True


class TestRecordTrajectory:
    def test_record_trajectory(self):
        arm, tp = _make_mock_arm()
        traj = arm.record_trajectory(
            output="/tmp/test_trajs",
            duration_s=2.0,
            sample_rate_hz=100.0,
            filter_alpha=0.15,
            name="test_drag",
        )
        assert isinstance(traj, JointTrajectory)
        assert traj.name == "test_recording"
        assert len(traj.frames) == 3
        assert traj.sample_rate_hz == 100.0


class TestRequestStop:
    def test_request_stop_publishes_estop(self):
        arm, tp = _make_mock_arm()
        # Subscribe to estop topic to verify the message
        estop_sub = tp.sub(protocol.estop_topic("armA"))

        arm.request_stop()

        raw = estop_sub.try_recv()
        assert raw is not None
        assert codec.decode_estop(raw) is True

    def test_clear_stop(self):
        arm, tp = _make_mock_arm()
        # clear_stop should succeed without error
        arm.clear_stop()


class TestGetState:
    def test_get_state_no_broadcast(self):
        arm, tp = _make_mock_arm()
        # No state broadcast yet, should return None
        state = arm.get_state()
        assert state is None

    def test_get_state_with_broadcast(self):
        arm, tp = _make_mock_arm()
        state_topic = protocol.state_topic("armA")

        # Simulate server broadcasting state
        mock_state = {
            "q": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
            "dq": [0.0] * 7,
            "tau": [0.0] * 7,
            "fault": [],
            "errs": [0] * 7,
            "temps": [(i, 25) for i in range(1, 8)],
            "state": "ready",
            "feedback": {
                "max_age_s": 0.01,
                "joints": [],
                "stale_joints": [],
            },
            "watchdog": {
                "enabled": True,
                "timeout_s": 0.5,
                "mode": "stop",
                "tripped": False,
                "last_kick_age_s": 0.001,
            },
            "robot_serial": "LITEARM-001",
            "config_checksum_sha256": "abc123",
        }
        tp.pub(state_topic, codec.encode_state(mock_state))

        state = arm.get_state()
        assert state is not None
        assert state["q"] == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
        assert state["state"] == "ready"
        assert state["robot_serial"] == "LITEARM-001"

    def test_get_state_returns_latest(self):
        arm, tp = _make_mock_arm()
        state_topic = protocol.state_topic("armA")

        # Broadcast multiple states
        for i in range(5):
            state = {
                "q": [float(i)] * 7,
                "dq": [0.0] * 7,
                "tau": [0.0] * 7,
                "fault": [],
                "errs": [0] * 7,
                "temps": [],
                "state": "ready",
                "feedback": {"max_age_s": 0.01, "joints": [], "stale_joints": []},
                "watchdog": {"enabled": False, "timeout_s": 0.5, "mode": "stop", "tripped": False, "last_kick_age_s": 0.0},
                "robot_serial": "TEST",
                "config_checksum_sha256": "",
            }
            tp.pub(state_topic, codec.encode_state(state))

        result = arm.get_state()
        # Should get the latest (i=4)
        assert result["q"][0] == 4.0


class TestParameterTuning:
    def test_get_gains(self):
        arm, tp = _make_mock_arm()
        gains = arm.get_gains()
        assert "kp" in gains
        assert "kd" in gains
        assert len(gains["kp"]) == 7

    def test_set_gains(self):
        arm, tp = _make_mock_arm()
        result = arm.set_gains(kp=[200.0] * 7, kd=[10.0] * 7)
        assert result["kp"] == [200.0] * 7

    def test_clear_faults(self):
        arm, tp = _make_mock_arm()
        result = arm.clear_faults()
        assert result == []

    def test_set_payload(self):
        arm, tp = _make_mock_arm()
        result = arm.set_payload(mass=1.5, com=(0.01, 0.0, 0.05))
        assert result["mass"] == 1.5

    def test_get_payload(self):
        arm, tp = _make_mock_arm()
        result = arm.get_payload()
        assert result["mass"] == 0.0

    def test_set_installation(self):
        arm, tp = _make_mock_arm()
        result = arm.set_installation(base_rpy=[0.0, 0.1, 0.0])
        assert result["base_rpy"] == [0.0, 0.1, 0.0]

    def test_get_installation(self):
        arm, tp = _make_mock_arm()
        result = arm.get_installation()
        assert result["gravity"] == [0, 0, -9.81]

    def test_get_tcp_pose(self):
        arm, tp = _make_mock_arm()
        pos, R = arm.get_tcp_pose()
        assert pos == [0.5, 0.0, 0.3]
        assert R == [[1, 0, 0], [0, 1, 0], [0, 0, 1]]


class TestErrorHandling:
    def test_server_returns_error(self):
        tp = InProcTransport()
        rpc_topic = protocol.rpc_topic("armA")

        def error_handler(payload: bytes) -> bytes:
            method, kwargs = codec.decode_request(payload)
            return codec.encode_reply(ok=False, error=SafetyViolationError(
                "following error exceeded", details={"joint": 2, "error_rad": 0.15}
            ))

        tp.declare_queryable(rpc_topic, error_handler)
        arm = Arm(arm_id="armA", transport=tp)

        with pytest.raises(SafetyViolationError, match="following error"):
            arm.movej([0.0] * 7)


class TestArmContextManager:
    def test_context_manager(self):
        arm, tp = _make_mock_arm()
        with arm as a:
            result = a.fk([0.0] * 7)
            assert result is not None
        # After __exit__, transport is closed
        # Calling methods should raise

    def test_repr(self):
        arm, tp = _make_mock_arm()
        assert "armA" in repr(arm)
