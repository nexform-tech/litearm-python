"""litearm-python —— LiteArm STM32 直连 SDK。

定位: 固件(litearm-stm32, 版本约定 ``Litearm1.5.0``+)内置 S 曲线/IK/动力学/控制律；
本包直连 USB CDC，并可选提供基于随包 URDF 的本地 FK/IK/Jacobian 与安全 PTP 预检。
固件仍负责最终的速度、加速度、jerk 约束和同步 S 曲线执行。

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

**笛卡尔路径** `move_l` / `move_c` / `move_path` 仍由固件规划。另一条互不冲突的入口是
`arm.kinematics.move_pose()`：SDK 本地求 IK 并检查整条关节直线，然后只发送一条
`CMD_MOVE_J_SYNC`，由固件执行同步 PTP；它不保证末端走笛卡尔直线。

⚠ 2.0 起**移除了 PC 侧规划**: 原子包 `cartesian/` 与 `Cartesian` / `CartesianPlanner` /
`MotionResult` / `arm.cartesian` / `movel` / `movec` 都不再存在, 替代是上面三个入口
(`move_l` 取代 `movel`、`move_c` 取代 `movec`、`move_path` 收多路点)。

⚠ `move_p` (`0x02`) 是**关节空间**点到点, 与上面这些**笛卡尔**路径不是一回事: 它只收
单个位姿 (多路点是 `move_path`), 末端轨迹不保证 (实测 30mm 的目标末端横摆 17.6mm)。

接口边界: `Arm.ik()` / `Arm.get_tcp()` 保持为固件远程计算；任意构型的本地
FK/IK/Jacobian 放在 `arm.kinematics`，微分速度 IK 放在 `litearm.exact_ik`。阻抗控制仍不
在本包范围内。安全 PTP 只检查自碰，不检测工作台、人员或外部障碍物。

工具点 (TCP) 偏移是**宿主侧**的一层换算: 固件没有工具坐标系命令, `get_tcp()` 恒为法兰,
所以 `set_tcp_offset(xyz, rpy)` 只在 SDK 里把 `move_p`/`move_l`/`move_c`/`move_path`/`ik`
的位姿入参换算成法兰再下发, 并用 `get_tool_tcp()` 提供读回 (见 :mod:`litearm.tool`)。
换工具时的重量要另设 `set_payload()` —— 两者不是一回事。

末端负载辨识 (见 :mod:`litearm.payload`): 夹上负载后用它自己的重力矩反推质量与质心 ——
几个静态构型上持位采样, 与一份**空载基线 JSON** 逐构型相减, 回归矩阵 `Y_g(q)` 由固件探针
(`0x39` 收任意 `q`, 配合临时 `set_payload`) 现算 ⇒ 仍然**不需要** PC 侧任何动力学模型。
产物直接喂 `set_payload()`。

依赖边界: 默认安装仍只有 `pyserial`。`arm.kinematics` 与碰撞检测属于可选的
`robotics` 依赖组（NumPy、SciPy、Pinocchio/hpp-fcl）；没有安装时只在调用这些入口时给出
明确提示，不影响轻量串口功能。

协议覆盖: 固件 `hal/usb_cmd.h` 里每条**已实现**的下行命令都有对应入口
(`arm.params.*` 关节级参数 / `arm.log.*` 300Hz 采集 / `arm.diag.kin_bench` 自检);
覆盖契约见 `_protocol.COMMAND_COVERAGE`。命令不被本机固件支持时抛
`UnsupportedByFirmwareError`。

零重力拖动示教见 `Arm.zero_g()` —— 返回上下文管理器, 保活由后台线程自动维持。
"""
from litearm.arm import (Arm, FIRMWARE_PREFIX, LicenseInfo, main, MIN_FW, Msg)
from litearm._rot import as_pose, mat_to_rpy, rpy_to_mat
from litearm.cart import CartPlan
from litearm.collision import CollisionModel, collision_urdf_path
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
from litearm.kinematics import (IKSolution, JacobianResult, Kinematics,
                                LocalKinematicsError)
from litearm.model import (MODEL_MASK_WRITTEN, ModelParams, ModelStatus)
from litearm.params import JointParam, JointParams
from litearm.payload import Baseline, HoldSample, PayloadResult
from litearm.state import JointState, RobotState
from litearm.tool import TcpOffset
from litearm.transport import find_cdc_port

__version__ = "2.2.0"

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
    "CollisionModel", "collision_urdf_path",
    "Kinematics", "JacobianResult", "IKSolution", "LocalKinematicsError",
    "JointParam", "JointParams", "ModelParams", "ModelStatus", "MODEL_MASK_WRITTEN",
    "LogSample", "LogReader", "parse_samples",
    "KinBenchResult", "parse_kin_bench",
    "CartesianPlanError",
    "TcpOffset",
    "Baseline", "HoldSample", "PayloadResult",
    "as_pose", "rpy_to_mat", "mat_to_rpy",
    "__version__",
]
