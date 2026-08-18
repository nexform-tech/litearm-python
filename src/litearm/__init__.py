"""litearm-python: LiteArm 机械臂 Python 客户端 SDK（远程调用 litearm-server）。

外设支持：
- arm.device("hand_0") → 灵巧手远程接口
- arm.device("gripper_0") → 夹爪远程接口
- arm.device("teach_0") → 示教板远程接口
- from litearm.can_bridge import RemoteCAN → CAN 隧道（直接使用厂商 SDK）
"""

__version__ = "0.1.0"

from .arm import Arm
from .device import DeviceManager, RemoteDevice
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
from .hand import RemoteHand
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
    "DeviceManager",
    "RemoteDevice",
    "RemoteHand",
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
