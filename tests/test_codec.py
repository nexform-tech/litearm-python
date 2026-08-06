"""Tests for litearm.codec — protobuf serialization with exception registry."""
import pytest

from litearm.codec import (
    decode_estop,
    decode_reply,
    decode_request,
    decode_state,
    encode_estop,
    encode_reply,
    encode_request,
    encode_state,
)
from litearm.exceptions import (
    LiteArmError,
    SafetyViolationError,
    FeedbackTimeoutError,
    MotionCancelled,
    ConfigurationError,
    TransportError,
    CartesianPlanError,
)


class TestEncodeDecodeRequest:
    def test_roundtrip(self):
        payload = encode_request("movej", {"q_target": [0.0] * 7, "speed": 0.5})
        method, kwargs = decode_request(payload)
        assert method == "movej"
        assert kwargs["q_target"] == [0.0] * 7
        assert kwargs["speed"] == 0.5

    def test_empty_kwargs(self):
        payload = encode_request("clear_faults", {})
        method, kwargs = decode_request(payload)
        assert method == "clear_faults"
        assert kwargs == {}

    def test_nested_kwargs(self):
        payload = encode_request("set_payload", {"mass": 1.5, "com": [0.01, 0.0, 0.05]})
        method, kwargs = decode_request(payload)
        assert method == "set_payload"
        assert kwargs["mass"] == 1.5
        assert kwargs["com"] == [0.01, 0.0, 0.05]


class TestEncodeDecodeReplyOk:
    def test_none_result(self):
        payload = encode_reply(ok=True, result=None)
        result = decode_reply(payload)
        assert result is None

    def test_list_result(self):
        data = [1.0, 2.0, 3.0]
        payload = encode_reply(ok=True, result=data)
        assert decode_reply(payload) == data

    def test_dict_result(self):
        data = {"kp": [100.0] * 7, "kd": [5.0] * 7}
        payload = encode_reply(ok=True, result=data)
        assert decode_reply(payload) == data

    def test_bool_result(self):
        payload = encode_reply(ok=True, result=True)
        assert decode_reply(payload) is True

    def test_tuple_result(self):
        # Tuples become lists in protobuf
        data = ([0.1, 0.2, 0.3], [[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        payload = encode_reply(ok=True, result=data)
        result = decode_reply(payload)
        assert result[0] == [0.1, 0.2, 0.3]
        assert result[1] == [[1, 0, 0], [0, 1, 0], [0, 0, 1]]


class TestEncodeDecodeReplyError:
    def test_litearm_error(self):
        err = LiteArmError("something failed")
        payload = encode_reply(ok=False, error=err)
        with pytest.raises(LiteArmError, match="something failed"):
            decode_reply(payload)

    def test_safety_violation_with_details(self):
        err = SafetyViolationError("overtemp", details={"motor": 3, "temp": 85.2})
        payload = encode_reply(ok=False, error=err)
        with pytest.raises(SafetyViolationError, match="overtemp") as exc_info:
            decode_reply(payload)
        assert exc_info.value.details == {"motor": 3, "temp": 85.2}

    def test_feedback_timeout(self):
        err = FeedbackTimeoutError("joint 2 stale", details={"joint": 2})
        payload = encode_reply(ok=False, error=err)
        with pytest.raises(FeedbackTimeoutError):
            decode_reply(payload)

    def test_motion_cancelled(self):
        err = MotionCancelled("user requested stop")
        payload = encode_reply(ok=False, error=err)
        with pytest.raises(MotionCancelled, match="user requested stop"):
            decode_reply(payload)

    def test_configuration_error(self):
        err = ConfigurationError("missing URDF")
        payload = encode_reply(ok=False, error=err)
        with pytest.raises(ConfigurationError, match="missing URDF"):
            decode_reply(payload)

    def test_transport_error(self):
        err = TransportError("CAN bus timeout")
        payload = encode_reply(ok=False, error=err)
        with pytest.raises(TransportError, match="CAN bus timeout"):
            decode_reply(payload)

    def test_unknown_error_falls_back_to_base(self):
        """If the error_type is not in the registry, fall back to LiteArmError."""
        # Create a reply with an unknown error type
        from litearm import litearm_pb2
        reply = litearm_pb2.RpcReply(ok=False)
        reply.error.type = "NonExistentErrorType"
        reply.error.message = "mystery"
        payload = reply.SerializeToString()

        with pytest.raises(LiteArmError, match="mystery"):
            decode_reply(payload)

    def test_string_error(self):
        payload = encode_reply(ok=False, error="raw string error")
        with pytest.raises(LiteArmError, match="raw string error"):
            decode_reply(payload)


class TestEncodeDecodeState:
    def test_full_state_roundtrip(self):
        state = {
            "q": [0.0] * 7,
            "dq": [0.0] * 7,
            "tau": [0.0] * 7,
            "fault": [],
            "errs": [0] * 7,
            "temps": [(1, 25), (2, 26), (3, 27), (4, 28), (5, 29), (6, 30), (7, 31)],
            "state": "ready",
            "feedback": {
                "max_age_s": 0.01,
                "joints": [
                    {"joint": i, "received": 1000, "age_s": 0.001, "fresh": True}
                    for i in range(7)
                ],
                "stale_joints": [],
            },
            "watchdog": {
                "enabled": True,
                "timeout_s": 0.5,
                "mode": "stop",
                "tripped": False,
                "last_kick_age_s": 0.002,
            },
            "robot_serial": "LITEARM-001",
            "config_checksum_sha256": "abc123",
        }
        payload = encode_state(state)
        decoded = decode_state(payload)
        assert decoded["q"] == [0.0] * 7
        assert decoded["state"] == "ready"
        assert decoded["robot_serial"] == "LITEARM-001"
        assert decoded["config_checksum_sha256"] == "abc123"
        assert decoded["watchdog"]["enabled"] is True
        assert decoded["watchdog"]["timeout_s"] == 0.5
        assert len(decoded["feedback"]["joints"]) == 7
        assert decoded["feedback"]["joints"][0]["fresh"] is True
        assert len(decoded["temps"]) == 7


class TestEncodeDecodeEstop:
    def test_roundtrip(self):
        payload = encode_estop()
        assert decode_estop(payload) is True

    def test_missing_field_defaults_false(self):
        # Protobuf defaults bool fields to False when not set
        from litearm import litearm_pb2
        estop = litearm_pb2.Estop()  # trigger field not set
        payload = estop.SerializeToString()
        assert decode_estop(payload) is False
