"""Structured exceptions for the litearm client SDK.

Mirrors pylitearm.errors but is fully independent — no cross-imports.
"""


class LiteArmError(RuntimeError):
    """Base class for all runtime SDK failures."""


class ConfigurationError(LiteArmError):
    """Robot configuration is missing, inconsistent, or fails integrity checks."""


class StateTransitionError(LiteArmError):
    """An API operation is not valid in the current arm state."""


class NotConnectedError(StateTransitionError):
    """The requested operation requires a live hardware connection."""


class InvalidCommandError(LiteArmError, ValueError):
    """A target or control parameter is malformed or outside its safe range."""


class CartesianPlanError(InvalidCommandError):
    """Cartesian planning failed (e.g. IK singularity or path infeasible)."""

    def __init__(self, message: str, partial=None, index: int = -1) -> None:
        super().__init__(message)
        self.partial = partial  # partial path computed before failure (list of q)
        self.index = index      # index of the failing waypoint


class SafetyViolationError(LiteArmError):
    """A planned or executing motion violated a configured safety rule."""

    def __init__(self, message: str, details: dict = None) -> None:
        super().__init__(message)
        self.details = details or {}


class FeedbackTimeoutError(SafetyViolationError):
    """One or more joints did not provide sufficiently fresh feedback."""


class FollowingError(SafetyViolationError):
    """Measured joint position departed too far from the commanded trajectory."""


class MotionTimeoutError(SafetyViolationError):
    """The robot did not physically settle at the requested target in time."""


class MotorFaultError(SafetyViolationError):
    """One or more drives reported a motor fault."""


class ArmFault(MotorFaultError):
    """A specific arm-level fault condition (e.g. undervoltage, overtemp)."""


class WatchdogError(SafetyViolationError):
    """The host-side command watchdog had to take over control."""


class TransportError(LiteArmError):
    """CAN/serial transport failed or could not accept a command."""


class CanModeMismatch(TransportError):
    """CAN bus mode (classic/FD, baud rate) does not match expected configuration."""


class CanMotorNotRegistered(TransportError):
    """A motor ID is not registered on the CAN bus."""


class CanWriteTimeout(TransportError):
    """CAN frame write timed out (bus full or device not responding)."""


class MotionCancelled(LiteArmError):
    """Motion was cancelled by a cancellation token or stop request."""


# ── Exception registry for codec deserialization ──────────────────────────────

EXCEPTION_REGISTRY = {
    "LiteArmError": LiteArmError,
    "ConfigurationError": ConfigurationError,
    "StateTransitionError": StateTransitionError,
    "NotConnectedError": NotConnectedError,
    "InvalidCommandError": InvalidCommandError,
    "CartesianPlanError": CartesianPlanError,
    "SafetyViolationError": SafetyViolationError,
    "FeedbackTimeoutError": FeedbackTimeoutError,
    "FollowingError": FollowingError,
    "MotionTimeoutError": MotionTimeoutError,
    "MotorFaultError": MotorFaultError,
    "ArmFault": ArmFault,
    "WatchdogError": WatchdogError,
    "TransportError": TransportError,
    "CanModeMismatch": CanModeMismatch,
    "CanMotorNotRegistered": CanMotorNotRegistered,
    "CanWriteTimeout": CanWriteTimeout,
    "MotionCancelled": MotionCancelled,
}
