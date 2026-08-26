# litearm-python 开发指南与接口说明

`litearm-python` 是 LiteArm 机械臂的 Python 客户端库（`__version__ = "0.1.0"`）。
通过网络连接机械臂控制服务，即可在任意普通电脑上控制机械臂。
**客户端无硬件依赖、无 numpy**，运行在机械臂控制器之外的机器上即可。

```text
你的程序 ──→ 机械臂控制服务 ──→ 机械臂 / CAN
```

---

## 目录

- [1. 环境要求与安装](#1-环境要求与安装)
- [2. 快速开始](#2-快速开始)
- [3. 连接管理](#3-连接管理)
- [4. 接口说明](#4-接口说明)
  - [4.1 计算（不驱动电机）](#41-计算不驱动电机)
  - [4.2 运动控制](#42-运动控制)
  - [4.3 状态读取](#43-状态读取)
  - [4.4 急停 / 使能](#44-急停--使能)
  - [4.5 DIRECT 模式 —— 逐帧 MIT 直接控制](#45-direct-模式--逐帧-mit-直接控制)
  - [4.6 参数调节](#46-参数调节)
  - [4.7 外设设备](#47-外设设备)
  - [4.8 系统 / 设置](#48-系统--设置)
  - [4.9 轨迹管理（服务端录制与管理）](#49-轨迹管理服务端录制与管理)
  - [4.10 末端设备管理](#410-末端设备管理)
  - [4.11 遥操（主从机械臂）](#411-遥操主从机械臂)
  - [4.12 CAN 隧道（高级）](#412-can-隧道高级)
- [5. 异常处理](#5-异常处理)
- [6. 安全提示](#6-安全提示)
- [7. 常见问题](#7-常见问题)

## 1. 环境要求与安装

| 项目 | 要求 |
|---|---|
| Python | 3.10+ |
| 依赖 | 安装时自动处理 |

```bash
pip install litearm-python          # 发布安装
pip install -e .                    # 开发安装（源码目录内）
```

## 2. 快速开始

```python
import litearm

with litearm.Arm(endpoint="tcp/192.168.1.100:7447") as arm:
    state = arm.get_state()          # 读取当前状态，q/dq/tau/fault/...
    arm.movej([0.0] * 7, speed=0.5)  # 关节运动
    arm.home(speed=0.3)               # 回零：所有关节归零，绕开限位检查

    hand = arm.device("hand_0")      # 末端外设：灵巧手
    hand.open()
    hand.set_gesture("pinch")
```

## 3. 连接管理

```python
arm = litearm.Arm(endpoint="tcp/127.0.0.1:7447", arm_id="armA")
# endpoint     : 机械臂控制服务地址，如 "tcp/192.168.1.100:7447"
# arm_id       : 机械臂标识，默认 "armA"，需与服务端设置一致
# query_timeout: 单次调用超时秒数（默认约 11.5 天，按需调小）
arm.close()                          # 关闭连接
```

- 支持上下文管理器：`with litearm.Arm(...) as arm:`，退出自动 `close()`。
- `get_state()` 同步读取服务端推送的最新状态缓存，未收到时返回 `None`。

## 4. 接口说明

> 通用约定：位姿用纯 list。`pose = [position, rotation]`，
> `position = [px, py, pz]`，`rotation = 3×3 行主序旋转矩阵`。
> 运动类方法（`movej`/`movel`/...）返回 `bool`；纯计算返回数据。

### 4.1 计算（不驱动电机）

| 方法 | 说明 |
|---|---|
| `fk(q)` | 正运动学 → `(位置, 旋转矩阵)` |
| `ik(pos_d, R_d, q_seed=None)` | 逆运动学 → `(q, 是否成功)` |
| `plan_movel(q_start, pose_goal)` | 直线笛卡尔路径规划 → 关节路径 |
| `plan_movec(q_start, pose_via, pose_goal)` | 圆弧路径规划（过中间点） |
| `plan_movep(q_start, poses_goal)` | 多航点路径规划 |

### 4.2 运动控制

| 方法 | 说明 |
|---|---|
| `movej(q_target, speed=1.0, settle_s=1.0, max_cycles=None, allow_start_collision_recovery=False)` | 关节空间点到点 |
| `home(speed=0.3, settle_s=0.5, max_cycles=None)` | 回零：所有关节归零，绕开限位和自碰路径检查 |
| `recover_joint_limits(speed=0.05, settle_s=0.5, max_cycles=None, inset_rad=0.0)` | 越限关节缓慢回安全边界（需服务端 `allow_limit_recovery=True`） |
| `movel(pose_goal, speed=1.0, settle_s=0.8, max_cycles=None)` | 笛卡尔直线 |
| `movec(pose_via, pose_goal, speed=1.0, settle_s=0.8, max_cycles=None)` | 笛卡尔圆弧 |
| `movep(poses_goal, speed=1.0, settle_s=0.8, max_cycles=None)` | 多航点带拐角平滑 |
| `replay_joint_path(q_path, speed=1.0, settle_s=0.5, goto_start=True, goto_speed=0.3, max_cycles=None)` | 回放关节序列 |
| `replay_trajectory(traj_q, speed=1.0, goto_start=True, goto_speed=0.3, max_cycles=None, check_singularity=True)` | 回放已录轨迹（JointTrajectory 或 dict） |
| `replay_timed_trajectory(traj_q, traj_t, speed=1.0, goto_start=True, goto_speed=0.3, simplify_tolerance_rad=0.01, max_cycles=None)` | 按原始时间轴回放（自动拉伸保安全） |
| `play_trajectory(trajectory, speed=1.0, goto_start=True, goto_speed=0.3, verify_robot=True, simplify_tolerance_rad=0.01, max_cycles=None)` | 回放已保存轨迹（对象或服务端路径字符串） |
| `record_trajectory(output="trajectories", duration_s=None, sample_rate_hz=100.0, filter_alpha=0.15, name=None)` | 拖动录轨迹 → `JointTrajectory` |
| `hold(kp_scale=3.0, max_cycles=None)` | 提高刚度持位 |
| `zero_gravity(max_cycles=None, duration_s=None, measured_overspeed_factor=None, vel_max=None)` | 零重力（自由拖动）模式 |
| `joint_impedance(q_des, K, B, tau_max=None, engage_sec=0.3, max_cycles=None)` | 关节空间阻抗控制 |
| `cartesian_impedance(q_des, K_cart, B_cart, v_des=None, tau_max=None, engage_sec=0.3, max_cycles=None, sigma_min_thresh=None, max_ori_err=None, measured_overspeed_factor=None, vel_max=None)` | 笛卡尔空间阻抗控制 |
| `joint_follow(K=None, B=None, speed_limit=None, accel_limit=None, engage_sec=0.3, max_cycles=None, duration_s=None)` | 跟随外部目标 |

### 4.3 状态读取

| 方法 | 说明 |
|---|---|
| `get_state(refresh=False)` | 状态缓存最近值（同步）；`refresh=True` 强制拉取 |
| `get_tcp_pose()` | 当前 TCP 位姿 → `(位置, 旋转矩阵)` |

### 4.4 急停 / 使能

| 方法 | 说明 |
|---|---|
| `request_stop()` | 高优先级急停（独立急停通道） |
| `clear_stop()` | 清除停止状态回到就绪 |
| `enable()` | 使能全部电机并锁住当前姿态 |
| `disable()` | ⚠️ 失能全部电机（机械臂会掉臂！），CAN 保持连接 |
| `clear_faults()` | 清除电机故障 → `[(motor_id, fault_code), ...]` |

### 4.5 DIRECT 模式 —— 逐帧 MIT 直接控制

> DIRECT 模式是 LiteArm 的逐帧 MIT 直接控制通道。通过 `send_mit` 以 250Hz 典型频率
> 发送五参数 (kp/kd/q_ref/dq_ref/tau_ff) 实时控制关节电机，内置 4 条永不关闭的核心安全护栏。

**与普通运动控制的区别：**

| 特性 | 普通运动控制 (`movej` 等) | DIRECT 模式 (`send_mit`) |
| --- | --- | --- |
| 控制方式 | 目标位置 + 速度，自动规划 | 逐帧五参数 MIT 命令 |
| 帧率 | 一次调用，自动执行 | 用户循环控制（典型 250Hz） |
| 阻塞 | 阻塞，等运动完成 | 非阻塞，立即返回 |
| 轨迹 | 自动规划 + 插值 | 用户自行生成 |
| 护栏 | 内置限位/限速 | 4 核心护栏 + 3 可选防护 |

**进入与退出：**

- **进入**：首次调用 `send_mit` 时自动进入 `ArmState.DIRECT`
- **退出**：`request_stop()` 主动退出 / 看门狗超时自动回 hold / 电机故障自动退出

#### send_mit —— 发送 MIT 控制帧

**Description:** 异步 pub 五参数 MIT 控制帧到机械臂命令通道。非阻塞，立即返回。首次调用自动进入 DIRECT 模式。帧率由用户循环控制。

**Function Definition:**

```python
def send_mit(
    self,
    kp: list[float],       # 长度 7，位置刚度
    kd: list[float],       # 长度 7，速度阻尼
    q_ref: list[float],    # 长度 7，目标关节角度 (rad)
    dq_ref: list[float],   # 长度 7，目标角速度 (rad/s)
    tau_ff: list[float],   # 长度 7，前馈力矩 (N·m)
) -> None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `kp` | `list[float]` | 位置刚度，长度 7，范围 `[0, 500]`。越大位置跟踪越紧，过大会振荡。典型值 15–200 |
| `kd` | `list[float]` | 速度阻尼，长度 7，范围 `[0, 5]`。抑制振荡，典型值 0.5–3.0 |
| `q_ref` | `list[float]` | 目标关节角度（rad），长度 7。相邻帧跳变受斜率限制。首帧以当前实测位置为锚点 |
| `dq_ref` | `list[float]` | 目标角速度（rad/s），长度 7。被 clamp 到 `±DQ_MAX`（协议上限） |
| `tau_ff` | `list[float]` | 前馈力矩（N·m），长度 7。被 clamp 到 `±min(guards_tau_max, TAU_MAX)` |

**Return Value:** `None` — 异步发送，不等待回执。

**Usage Example:**

```python
import time
import litearm

arm = litearm.Arm(endpoint="tcp/192.168.1.100:7447")

# 发送单帧（自动进入 DIRECT 模式）
arm.send_mit(
    kp=[50.0] * 7,
    kd=[1.5] * 7,
    q_ref=[0.0] * 7,
    dq_ref=[0.0] * 7,
    tau_ff=[0.0] * 7,
)
```

#### set_guards —— 配置全局护栏

**Description:** 全局一次性配置护栏参数。所有参数为关键字参数，`None` 表示不改变。数值参数在物理极限内自动 clamp。**全局持久**：退出 DIRECT 后不重置，再次进入时仍按同一配置执行。

**Function Definition:**

```python
def set_guards(
    self,
    *,
    slew_limit: float | None = None,
    tau_max: float | None = None,
    watchdog_timeout: float | None = None,
    position_bounds: bool | None = None,
    velocity_bounds: bool | None = None,
    jerk_limit: bool | None = None,
) -> Any
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `slew_limit` | `float` or `None` | 全局斜率限制（rad/s）。`None` = 不改变。clamp 到 `(0, 最低额定速度]` |
| `tau_max` | `float` or `None` | 全局力矩上限（N·m）。`None` = 不改变。clamp 到 `(0, 最低物理 TAU_MAX]` |
| `watchdog_timeout` | `float` or `None` | 看门狗超时（秒），范围 `[0.05, 2.0]`。`None` = 不改变 |
| `position_bounds` | `bool` or `None` | 是否开启位置软限位 clamp。`None` = 不改变。默认 `False` |
| `velocity_bounds` | `bool` or `None` | 是否开启速度软限位 clamp。`None` = 不改变。默认 `False` |
| `jerk_limit` | `bool` or `None` | 是否开启加加速度限制。`None` = 不改变。默认 `False` |

**Return Value:** RPC 返回的回复对象（通常忽略）。

**Usage Example:**

```python
# 收紧斜率限制（保守模式，单帧最大跳变 ≈ 0.05 rad）
arm.set_guards(slew_limit=0.5)

# 收紧看门狗（高频控制必须，保证 ≥20Hz 帧率）
arm.set_guards(watchdog_timeout=0.05)

# 开启位置软限位（防止越限）
arm.set_guards(position_bounds=True)

# 组合配置
arm.set_guards(
    slew_limit=1.0,
    tau_max=10.0,
    watchdog_timeout=0.10,
    position_bounds=True,
)
```

#### get_guards —— 读取当前护栏配置

**Description:** 读取当前生效的护栏配置（RPC 同步）。

**Function Definition:**

```python
def get_guards(self) -> dict[str, Any]
```

**Return Value:**

```json
{
    "slew_limit": 1.0,
    "tau_max": 10.0,
    "watchdog_timeout": 0.10,
    "position_bounds": true,
    "velocity_bounds": false,
    "jerk_limit": false
}
```

**Usage Example:**

```python
guards = arm.get_guards()
print(f"slew_limit = {guards['slew_limit']} rad/s")
```

#### 完整控制循环示例

```python
"""DIRECT 模式 250Hz 控制循环 —— 正弦波扫关节1。"""
import math
import time
import litearm

arm = litearm.Arm(endpoint="tcp/192.168.1.100:7447")

# 配置护栏（一次性）
arm.set_guards(slew_limit=2.0, tau_max=20.0, watchdog_timeout=0.10, position_bounds=True)

DT = 0.004      # 4ms → 250Hz
FREQ = 0.5      # 正弦波频率 (Hz)
AMP = 0.5       # 幅度 (rad)

t = 0.0
try:
    while True:
        loop_start = time.perf_counter()

        # 仅关节1正弦运动，其余关节保持零点
        q_ref = [AMP * math.sin(2 * math.pi * FREQ * t)] + [0.0] * 6

        arm.send_mit(
            kp=[50.0] * 7, kd=[1.5] * 7,
            q_ref=q_ref, dq_ref=[0.0] * 7, tau_ff=[0.0] * 7,
        )

        t += DT
        elapsed = time.perf_counter() - loop_start
        if elapsed < DT:
            time.sleep(DT - elapsed)

except KeyboardInterrupt:
    arm.request_stop()
    print("已退出 DIRECT 模式")
```

#### 安全护栏说明

| 护栏 | 说明 | 可关闭？ |
| --- | --- | --- |
| **护栏1：协议参数 clamp** | kp≤500、kd≤5、dq_ref≤DQ_MAX、tau_ff≤TAU_MAX | 不可关闭 |
| **护栏2：命令通道斜率限制** | 相邻帧 q_ref 跳变 ≤ `min(slew_limit, 额定速度) × dt`，dt 有上界 0.10s | 不可关闭，`slew_limit` 可收紧 |
| **护栏3：看门狗 fail-soft** | 命令中断超时自动回 hold（低刚度 PD 托住当前位置） | 不可关闭，`watchdog_timeout` 可调 |
| **护栏4：单一所有权** | DIRECT 激活时拒绝运动命令和遥操；server 侧同一时刻只允许一个 client | 不可关闭 |
| **位置限制（附加）** | q_ref 逐帧 clamp 到关节软限位 `[q_min, q_max]` | 默认关，`position_bounds=True` 开启 |
| **速度限制（附加）** | dq_ref 逐帧 clamp 到 `±DQ_MAX` | 默认关，`velocity_bounds=True` 开启 |
| **加加速度限制（附加）** | dq_ref 变化率受限 | 默认关，`jerk_limit=True` 开启 |

> **安全底线：机械臂永远不允许乱飞。** 命令通道斜率限制 + 单一收口 + 看门狗 fail-soft + 固件兜底
> 是所有安全改动的验收前提。

### 4.6 参数调节

| 方法 | 说明 |
|---|---|
| `set_gains(kp=None, kd=None)` / `get_gains()` | PD 增益设置/读取 |
| `set_payload(mass, com=(0,0,0))` / `get_payload()` | 末端负载（质量 + 质心） |
| `set_installation(base_rpy=None, gravity=None)` / `get_installation()` | 安装姿态（基座 RPY 或重力向量） |
| `get_joint_limits()` / `set_joint_limits(limits)` | 关节限位 |
| `get_zero_offsets()` / `set_zero_offsets(offsets)` | 零位偏移 |
| `get_end_effector()` / `set_end_effector(config)` | 末端执行器配置 |
| `get_cartesian_limits()` / `set_cartesian_limits(limits)` | 笛卡尔限位 |
| `get_collision_config()` / `set_collision_config(config)` | 碰撞配置 |

### 4.7 外设设备

统一入口 `arm.device(device_id)`，方法路由到对应设备接口 `device.{device_id}.{method}`。

```python
hand = arm.device("hand_0")
hand.open(); hand.close()                    # 开/合
hand.set_force(force)                        # 抓取力
hand.get_state(); hand.list_gestures()       # 状态 / 支持手势
hand.set_gesture("pinch")                    # 手势
hand.finger_move(pose); hand.set_speed(speed); hand.set_torque(torque)  # 逐指

gripper = arm.device("gripper_0")
gripper.set_width(0.5); w = gripper.get_width()

teach = arm.device("teach_0")
teach.get_joints(); teach.get_buttons()

# 通用：get_status / get_info / connect / disconnect / clear_faults
```

- 设备管理器：`arm.devices["hand_0"]` 等价于 `arm.device("hand_0")`（延迟创建）。
- 向后兼容便捷属性：`arm.hand.open()` 等价于 `arm.device("hand_0").open()`。

### 4.8 系统 / 设置

| 方法 | 说明 |
|---|---|
| `get_system_stats()` | CPU / 内存 / 板温 / 运行时长 |
| `get_logs(page=1, size=50, search="")` | 分页日志 |
| `restart_service()` | 重启 arm 服务 |
| `reconnect()` | 硬件重连（从任意状态重新初始化电机，用于机械臂热重启后恢复） |

### 4.9 轨迹管理（服务端录制与管理）

```python
arm.start_recording();  arm.get_recording_state();  arm.stop_recording();  arm.discard_recording()
arm.list_trajectories()
arm.save_trajectory("t1", "demo", points, duration=None)
arm.delete_trajectory("t1")
arm.get_playback_state()
```

### 4.10 末端设备管理

```python
arm.list_device_types()
arm.connect_device(category="hand", subtype="lite6_hand", device_id="end_0", can_iface="", config=None)
arm.get_active_device(device_id="end_0")
arm.disconnect_device(device_id="end_0")
```

### 4.11 遥操（主从机械臂）

```python
arm.enter_teleop("master")                                    # 本臂作为主臂
arm.enter_teleop("slave", peer="tcp/10.0.0.2:7447")           # 跟随主臂
arm.get_teleop_status()
arm.exit_teleop()
```

> 遥操态下服务端拒绝一切手动控制指令，只放行只读 / 急停 / `exit_teleop`。

### 4.12 CAN 隧道（高级）

需要在本地直接使用厂商 CAN 协议时可用：

```python
from litearm.can_bridge import RemoteCAN

can = RemoteCAN("tcp/127.0.0.1:7447", vcan_iface="vcan0")
can.start()        # 把控制器 can0 桥接到本地 vcan0
# ... 用厂商 SDK 在本地 vcan0 收发 ...
can.stop()
```

## 5. 异常处理

所有异常继承 `LiteArmError`（`RuntimeError` 子类），服务端抛出的异常会在客户端以相同类型抛出。

```python
from litearm import LiteArmError, SafetyViolationError

try:
    arm.movej([0.0] * 7)
except SafetyViolationError as e:      # 含安全违规（超时/跟随/故障/看门狗）
    print(e.details)
except LiteArmError as e:              # 兜底
    print(e)
```

常用类型（节选）：`NotConnectedError`、`ConfigurationError`、`InvalidCommandError`、
`CartesianPlanError`、`MotionTimeoutError`、`MotorFaultError`、`ArmFault`、`WatchdogError`、`MotionCancelled`。

## 6. 安全提示

- ⚠️ `disable()` 会使机械臂在重力作用下坠落，务必确认安全。
- `request_stop()` 为高优先级急停，应绑定到独立物理急停通道。
- 遥操态下不会执行手动控制指令。
- `recover_joint_limits` 仅在服务端以 `allow_limit_recovery=True` 启动时可用。

## 7. 常见问题

| 问题 | 处理 |
|---|---|
| `get_state()` 返回 `None` | 未收到状态：确认服务已启动、endpoint/arm_id 正确 |
| `NotConnectedError` | 需要硬件连接的操作，先确认服务在线 |
| 调用长时间无响应 | 调小 `query_timeout` 或检查网络 / 服务状态 |
| 配置校验失败 | 检查机械臂控制服务的配置是否正确 |

## License

Proprietary
