"""litearm-python —— LiteArm STM32 直连后端 (薄协议 SDK)。

定位: 固件(litearm-stm32, 版本约定 ``Litearm1.5.0``+)已内置 S 曲线/IK/动力学/
控制律, 本包直连 USB CDC 用高层子集镜像 pylitearm 常用用法, **不改 pylitearm 源码**、
PC 端不做轨迹/运动学。

典型用法::

    import litearm as pa
    arm = pa.Arm().connect()          # 校验固件版本约定
    arm.enable()
    arm.movej([0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], speed=0.3)
    tcp = arm.get_tcp().value         # ⚠ 2.0 起读一帧的 getter 返回 Msg 信封, 读值要 .value
    q = arm.ik((0.30, 0.0, 0.35, 3.14, 0.0, 0.0))
    arm.move_l((0.30, 0.0, 0.40, 3.14, 0.0, 0.0), speed=0.5)  # 笛卡尔直线 (固件规划)
    arm.move_c(tcp, via_pose, goal_pose)                      # 笛卡尔圆弧 (起点须为实测 TCP)
    arm.move_path([p1, p2, p3], speed=0.5)                    # 笛卡尔多路点 (尖角)
    arm.disable()
    arm.close()

**返回信封** `Msg(value, hz, timestamp)`: 11 个"读一帧"型 getter (`get_state` /
`get_status_now` / `get_tcp` / `get_ff_vec` / `get_ff_scalar` / `params.get_joint_param` /
`model.get_body` / `model.get_jm` / `model.status` / `model.get_gravity` /
`diag.kin_bench`) 返回它 —— `value` 是原返回值, `hz` 是该类帧在本会话里的平均到达频率,
`timestamp` 是最近一帧的本地时刻。`move_*`/`home` (动作结果) 与纯本地量不包。

**笛卡尔运动** 只有固件规划一条路: `move_l` / `move_c` / `move_path` —— 采样/逐点 IK/播放
全在固件里, PC 只发点、收 `0x4E` 结果帧, 返回 `CartPlan`。

⚠ 2.0 起**移除了 PC 侧规划**: 原子包 `cartesian/` 与 `Cartesian` / `CartesianPlanner` /
`MotionResult` / `arm.cartesian` / `movel` / `movec` 都不再存在, 替代是上面三个入口
(`move_l` 取代 `movel`、`move_c` 取代 `movec`、`move_path` 收多路点)。

⚠ `move_p` (`0x02`) 是**关节空间**点到点, 与上面这些**笛卡尔**路径不是一回事: 它只收
单个位姿 (多路点是 `move_path`), 末端轨迹不保证 (实测 30mm 的目标末端横摆 17.6mm)。

能力边界(刻意不做): 任意关节角 fk(q) —— 固件 `CMD_GET_TCP` 只能算**当前反馈 q** 的
位姿, 没有"给 q 求位姿"的下行命令, 所以拿不到任意构型的 FK (连带影响: 起点奇异预检
σmin 算不了); 阻抗控制 —— 高级场景走 pylitearm+server。

依赖边界(刻意保持): 只有 `pyserial`。PC 侧不做规划, 只用**纯 Python** 数学(`math` 模块,
见 `_rot.py`)归一化位姿形态与判到位姿态, 不引入 numpy/pinocchio —— 重型计算本来就在
固件的 IK 里。

协议覆盖: 固件 `hal/usb_cmd.h` 里每条**已实现**的下行命令都有对应入口
(`arm.params.*` 关节级参数 / `arm.log.*` 300Hz 采集 / `arm.diag.kin_bench` 自检);
覆盖契约见 `_protocol.COMMAND_COVERAGE`, 由 `tests/test_protocol_sync.py` 直接解析
固件头文件强制。命令不被本机固件支持时抛 `UnsupportedByFirmwareError`。

零重力拖动示教见 `Arm.zero_g()` —— 返回上下文管理器, 保活由后台线程自动维持。
"""
from litearm.arm import (Arm, FIRMWARE_PREFIX, LicenseInfo, main, MIN_FW, Msg)
from litearm._rot import as_pose, mat_to_rpy, rpy_to_mat
from litearm.cart import CartPlan
from litearm.diagnostics import KinBenchResult, parse_kin_bench
from litearm.errors import (
    ArmIsInDfuError,
    CartReplyLostError,
    CartesianPlanError,
    CommandRejectedError,
    FirmwareMismatchError,
    ForkedSessionError,
    IKError,
    InvalidCommandError,
    LiteArmError,
    MotionSupersededError,
    MotionTimeoutError,
    MotorFaultError,
    NotConnectedError,
    NotRemoteable,
    NotSupportedOnThisBackend,
    TeleopBusyError,
    TeleopLockedError,
    TransportError,
    UnsupportedByFirmwareError,
)
from litearm.log import LogReader, LogSample, parse_samples
from litearm.model import (MODEL_MASK_WRITTEN, ModelParams, ModelStatus)
from litearm.params import JointParam, JointParams
from litearm.state import JointState, RobotState
from litearm.transport import find_cdc_port

__version__ = "2.1.0"

__all__ = [
    "Arm", "Msg", "LicenseInfo", "RobotState", "JointState", "find_cdc_port", "main",
    "MIN_FW", "FIRMWARE_PREFIX",
    "LiteArmError", "NotConnectedError", "ForkedSessionError", "TransportError",
    "FirmwareMismatchError", "InvalidCommandError", "MotorFaultError",
    "MotionTimeoutError", "IKError", "CommandRejectedError",
    "UnsupportedByFirmwareError",
    "MotionSupersededError", "CartReplyLostError", "ArmIsInDfuError",
    "NotRemoteable", "NotSupportedOnThisBackend",
    "TeleopLockedError", "TeleopBusyError",
    "CartPlan",
    "JointParam", "JointParams", "ModelParams", "ModelStatus", "MODEL_MASK_WRITTEN",
    "LogSample", "LogReader", "parse_samples",
    "KinBenchResult", "parse_kin_bench",
    "CartesianPlanError",
    "as_pose", "rpy_to_mat", "mat_to_rpy",
    "__version__",
]
