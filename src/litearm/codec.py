"""Msgpack serialization for litearm v4 wire protocol.

All messages carry a protocol version number. Exception types are registered
so that remote errors can be reconstructed on the client side.
"""
from __future__ import annotations

from typing import Any, Dict, Optional, Tuple

import msgpack

from .exceptions import EXCEPTION_REGISTRY, LiteArmError, SafetyViolationError

PROTOCOL_VERSION = 1


# ── Request encoding/decoding ────────────────────────────────────────────────

def encode_request(method: str, kwargs: dict) -> bytes:
    """Encode an RPC request to msgpack bytes."""
    return msgpack.packb({
        "v": PROTOCOL_VERSION,
        "method": method,
        "kwargs": kwargs,
    })


def decode_request(payload: bytes) -> Tuple[str, dict]:
    """Decode an RPC request. Returns (method, kwargs)."""
    msg = msgpack.unpackb(payload, raw=False)
    if msg["v"] != PROTOCOL_VERSION:
        raise ValueError(
            f"Protocol version mismatch: got {msg['v']}, expected {PROTOCOL_VERSION}"
        )
    return msg["method"], msg["kwargs"]


# ── Reply encoding/decoding ──────────────────────────────────────────────────

def encode_reply(ok: bool, result: Any = None, error: Any = None) -> bytes:
    """Encode an RPC reply (success or error) to msgpack bytes."""
    if ok:
        return msgpack.packb({
            "v": PROTOCOL_VERSION,
            "ok": True,
            "result": result,
        })
    else:
        if isinstance(error, BaseException):
            error_type = type(error).__name__
            error_msg = str(error)
            # Preserve SafetyViolationError details
            details = getattr(error, "details", None)
        else:
            error_type = "LiteArmError"
            error_msg = str(error) if error else "Unknown error"
            details = None
        payload: Dict[str, Any] = {
            "v": PROTOCOL_VERSION,
            "ok": False,
            "error_type": error_type,
            "error_msg": error_msg,
        }
        if details is not None:
            payload["details"] = details
        return msgpack.packb(payload)


def decode_reply(payload: bytes) -> Any:
    """Decode an RPC reply. Returns result on success, raises on error."""
    msg = msgpack.unpackb(payload, raw=False)
    if msg["v"] != PROTOCOL_VERSION:
        raise ValueError(
            f"Protocol version mismatch: got {msg['v']}, expected {PROTOCOL_VERSION}"
        )
    if msg["ok"]:
        return msg["result"]

    # Reconstruct exception from registry
    error_type = msg["error_type"]
    error_msg = msg["error_msg"]
    details = msg.get("details")

    exc_class = EXCEPTION_REGISTRY.get(error_type, LiteArmError)

    # SafetyViolationError and subclasses take (message, details)
    if issubclass(exc_class, SafetyViolationError):
        raise exc_class(error_msg, details=details or {})

    # CartesianPlanError takes (message, partial, index) — we only have msg
    if error_type == "CartesianPlanError":
        raise exc_class(error_msg)

    raise exc_class(error_msg)


# ── State encoding/decoding ──────────────────────────────────────────────────

def encode_state(state: dict) -> bytes:
    """Encode a RobotState broadcast to msgpack bytes."""
    return msgpack.packb({
        "v": PROTOCOL_VERSION,
        "state": state,
    })


def decode_state(payload: bytes) -> dict:
    """Decode a RobotState broadcast. Returns the state dict."""
    msg = msgpack.unpackb(payload, raw=False)
    if msg["v"] != PROTOCOL_VERSION:
        raise ValueError(
            f"Protocol version mismatch: got {msg['v']}, expected {PROTOCOL_VERSION}"
        )
    return msg["state"]


# ── Estop encoding/decoding ──────────────────────────────────────────────────

def encode_estop() -> bytes:
    """Encode an emergency stop signal."""
    return msgpack.packb({
        "v": PROTOCOL_VERSION,
        "estop": True,
    })


def decode_estop(payload: bytes) -> bool:
    """Decode an emergency stop signal. Returns True if estop is active."""
    msg = msgpack.unpackb(payload, raw=False)
    return msg.get("estop", False)
