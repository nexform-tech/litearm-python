"""litearm-python: LiteArm 机械臂 Python 客户端 SDK（远程调用 litearm-server）。"""

__version__ = "0.1.0"

from .arm import Arm
from .exceptions import (
    ArmFault,
    CanModeMismatch,
    CanMotorNotRegistered,
    CanWriteTimeout,
    CartesianPlanError,
    ConfigurationError,
    FeedbackTimeoutError,
    FollowingError,
    InvalidCommandError,
    LiteArmError,
    MotionCancelled,
    MotionTimeoutError,
    MotorFaultError,
    NotConnectedError,
    SafetyViolationError,
    StateTransitionError,
    TransportError,
    WatchdogError,
)
from .types import (
    ArmState,
    CancellationToken,
    FeedbackState,
    JointFeedbackState,
    JointTrajectory,
    RobotState,
    TrajectoryFrame,
    WatchdogState,
)

__all__ = [
    # Core
    "Arm",
    "__version__",
    # Types
    "ArmState",
    "CancellationToken",
    "FeedbackState",
    "JointFeedbackState",
    "JointTrajectory",
    "RobotState",
    "TrajectoryFrame",
    "WatchdogState",
    # Exceptions
    "ArmFault",
    "CanModeMismatch",
    "CanMotorNotRegistered",
    "CanWriteTimeout",
    "CartesianPlanError",
    "ConfigurationError",
    "FeedbackTimeoutError",
    "FollowingError",
    "InvalidCommandError",
    "LiteArmError",
    "MotionCancelled",
    "MotionTimeoutError",
    "MotorFaultError",
    "NotConnectedError",
    "SafetyViolationError",
    "StateTransitionError",
    "TransportError",
    "WatchdogError",
]
