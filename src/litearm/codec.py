"""Protobuf serialization for litearm v4 wire protocol.

All messages use protobuf for type-safe, multi-language serialization.
Exception types are registered so that remote errors can be reconstructed
on the client side.
"""
from __future__ import annotations

from typing import Any, Dict, Optional, Tuple

from . import litearm_pb2
from .exceptions import EXCEPTION_REGISTRY, LiteArmError, SafetyViolationError

PROTOCOL_VERSION = 1


# ── Python ↔ Protobuf Value conversion ───────────────────────────────────────

def _python_to_value(obj: Any) -> litearm_pb2.Value:
    """Convert a Python object to protobuf Value."""
    val = litearm_pb2.Value()
    if obj is None:
        val.none_val.CopyFrom(litearm_pb2.NoneValue())
    elif isinstance(obj, bool):
        val.bool_val = obj
    elif isinstance(obj, int):
        val.int_val = obj
    elif isinstance(obj, float):
        val.double_val = obj
    elif isinstance(obj, str):
        val.string_val = obj
    elif isinstance(obj, bytes):
        val.bytes_val = obj
    elif isinstance(obj, (list, tuple)):
        list_val = litearm_pb2.ListValue()
        for item in obj:
            list_val.values.append(_python_to_value(item))
        val.list_val.CopyFrom(list_val)
    elif isinstance(obj, dict):
        map_val = litearm_pb2.MapValue()
        for k, v in obj.items():
            map_val.values[k].CopyFrom(_python_to_value(v))
        val.map_val.CopyFrom(map_val)
    else:
        # Fallback: convert to string
        val.string_val = str(obj)
    return val


def _value_to_python(val: litearm_pb2.Value) -> Any:
    """Convert a protobuf Value to Python object."""
    if val.HasField("none_val"):
        return None
    elif val.HasField("bool_val"):
        return val.bool_val
    elif val.HasField("int_val"):
        return val.int_val
    elif val.HasField("double_val"):
        return val.double_val
    elif val.HasField("string_val"):
        return val.string_val
    elif val.HasField("bytes_val"):
        return val.bytes_val
    elif val.HasField("list_val"):
        return [_value_to_python(v) for v in val.list_val.values]
    elif val.HasField("map_val"):
        return {k: _value_to_python(v) for k, v in val.map_val.values.items()}
    else:
        return None


# ── Request encoding/decoding ────────────────────────────────────────────────

def encode_request(method: str, kwargs: dict) -> bytes:
    """Encode an RPC request to protobuf bytes."""
    req = litearm_pb2.RpcRequest(method=method)
    for k, v in kwargs.items():
        req.kwargs[k].CopyFrom(_python_to_value(v))
    return req.SerializeToString()


def decode_request(payload: bytes) -> Tuple[str, dict]:
    """Decode an RPC request. Returns (method, kwargs)."""
    req = litearm_pb2.RpcRequest()
    req.ParseFromString(payload)
    kwargs = {k: _value_to_python(v) for k, v in req.kwargs.items()}
    return req.method, kwargs


# ── Reply encoding/decoding ──────────────────────────────────────────────────

def encode_reply(ok: bool, result: Any = None, error: Any = None) -> bytes:
    """Encode an RPC reply (success or error) to protobuf bytes."""
    reply = litearm_pb2.RpcReply(ok=ok)

    if ok:
        reply.result.CopyFrom(_python_to_value(result))
    else:
        err = litearm_pb2.Error()
        if isinstance(error, BaseException):
            err.type = type(error).__name__
            err.message = str(error)
            # Preserve SafetyViolationError details
            details = getattr(error, "details", None)
            if details:
                for k, v in details.items():
                    err.details[k].CopyFrom(_python_to_value(v))
        else:
            err.type = "LiteArmError"
            err.message = str(error) if error else "Unknown error"
        reply.error.CopyFrom(err)

    return reply.SerializeToString()


def decode_reply(payload: bytes) -> Any:
    """Decode an RPC reply. Returns result on success, raises on error."""
    reply = litearm_pb2.RpcReply()
    reply.ParseFromString(payload)

    if reply.ok:
        return _value_to_python(reply.result)

    # Reconstruct exception from registry
    error_type = reply.error.type
    error_msg = reply.error.message
    details = {k: _value_to_python(v) for k, v in reply.error.details.items()}

    exc_class = EXCEPTION_REGISTRY.get(error_type, LiteArmError)

    # SafetyViolationError and subclasses take (message, details)
    if issubclass(exc_class, SafetyViolationError):
        raise exc_class(error_msg, details=details or {})

    # CartesianPlanError takes (message, partial, index) — we only have msg
    if error_type == "CartesianPlanError":
        raise exc_class(error_msg)

    raise exc_class(error_msg)


# ── State encoding/decoding ──────────────────────────────────────────────────

def _dict_to_robot_state(state_dict: dict) -> litearm_pb2.RobotState:
    """Convert a Python dict to protobuf RobotState."""
    state = litearm_pb2.RobotState()

    # Simple fields
    state.q.extend(state_dict.get("q", []))
    state.dq.extend(state_dict.get("dq", []))
    state.tau.extend(state_dict.get("tau", []))
    state.errs.extend(state_dict.get("errs", []))
    state.state = state_dict.get("state", "")
    state.robot_serial = state_dict.get("robot_serial", "")
    state.config_checksum_sha256 = state_dict.get("config_checksum_sha256", "")

    # Fault list
    for fault in state_dict.get("fault", []):
        f = state.fault.add()
        f.joint = fault[0]
        f.err_code = fault[1]

    # Temperature list
    for temp in state_dict.get("temps", []):
        t = state.temps.add()
        t.mos_temp = temp[0]
        t.coil_temp = temp[1]

    # Feedback state
    if "feedback" in state_dict:
        fb_dict = state_dict["feedback"]
        fb = state.feedback
        fb.max_age_s = fb_dict.get("max_age_s", 0.0)
        fb.stale_joints.extend(fb_dict.get("stale_joints", []))
        for jfb_dict in fb_dict.get("joints", []):
            jfb = fb.joints.add()
            jfb.joint = jfb_dict.get("joint", 0)
            jfb.received = jfb_dict.get("received", 0)
            jfb.age_s = jfb_dict.get("age_s", 0.0)
            jfb.fresh = jfb_dict.get("fresh", False)

    # Watchdog state
    if "watchdog" in state_dict:
        wd_dict = state_dict["watchdog"]
        wd = state.watchdog
        wd.enabled = wd_dict.get("enabled", False)
        wd.timeout_s = wd_dict.get("timeout_s", 0.0)
        wd.mode = wd_dict.get("mode", "")
        wd.tripped = wd_dict.get("tripped", False)
        wd.last_kick_age_s = wd_dict.get("last_kick_age_s", 0.0)

    return state


def _robot_state_to_dict(state: litearm_pb2.RobotState) -> dict:
    """Convert a protobuf RobotState to Python dict."""
    state_dict = {
        "q": list(state.q),
        "dq": list(state.dq),
        "tau": list(state.tau),
        "fault": [(f.joint, f.err_code) for f in state.fault],
        "errs": list(state.errs),
        "temps": [(t.mos_temp, t.coil_temp) for t in state.temps],
        "state": state.state,
        "robot_serial": state.robot_serial,
        "config_checksum_sha256": state.config_checksum_sha256,
    }

    # Feedback state
    if state.HasField("feedback"):
        fb = state.feedback
        state_dict["feedback"] = {
            "max_age_s": fb.max_age_s,
            "stale_joints": list(fb.stale_joints),
            "joints": [
                {
                    "joint": jfb.joint,
                    "received": jfb.received,
                    "age_s": jfb.age_s,
                    "fresh": jfb.fresh,
                }
                for jfb in fb.joints
            ],
        }

    # Watchdog state
    if state.HasField("watchdog"):
        wd = state.watchdog
        state_dict["watchdog"] = {
            "enabled": wd.enabled,
            "timeout_s": wd.timeout_s,
            "mode": wd.mode,
            "tripped": wd.tripped,
            "last_kick_age_s": wd.last_kick_age_s,
        }

    return state_dict


def encode_state(state: dict) -> bytes:
    """Encode a RobotState broadcast to protobuf bytes."""
    robot_state = _dict_to_robot_state(state)
    return robot_state.SerializeToString()


def decode_state(payload: bytes) -> dict:
    """Decode a RobotState broadcast. Returns the state dict."""
    robot_state = litearm_pb2.RobotState()
    robot_state.ParseFromString(payload)
    return _robot_state_to_dict(robot_state)


# ── Estop encoding/decoding ──────────────────────────────────────────────────

def encode_estop() -> bytes:
    """Encode an emergency stop signal."""
    estop = litearm_pb2.Estop(trigger=True)
    return estop.SerializeToString()


def decode_estop(payload: bytes) -> bool:
    """Decode an emergency stop signal. Returns True if estop is active."""
    estop = litearm_pb2.Estop()
    estop.ParseFromString(payload)
    return estop.trigger
