"""Zenoh topic naming conventions for litearm v4 protocol."""

PROTOCOL_VERSION = 1


def rpc_topic(arm_id: str) -> str:
    """RPC request/reply topic for the given arm."""
    return f"litearm/v4/{arm_id}/rpc"


def state_topic(arm_id: str) -> str:
    """State broadcast topic for the given arm."""
    return f"litearm/v4/{arm_id}/state"


def command_topic(arm_id: str) -> str:
    """Command channel topic for the given arm (e.g. servo targets)."""
    return f"litearm/v4/{arm_id}/command"


def estop_topic(arm_id: str) -> str:
    """High-priority emergency stop topic for the given arm."""
    return f"litearm/v4/{arm_id}/estop"
