# litearm-python 开发者指南

LiteArm 机械臂的 Python SDK，USB 串口直连固件。

规划、运动学、动力学都在固件里，PC 侧只编解码帧、下发命令、判到位。
位姿是 6 个数（list 或 tuple），只依赖 `pyserial`。

## 目录

1. [环境要求与安装](#1-环境要求与安装)
2. [快速开始](#2-快速开始)
3. [连接管理](#3-连接管理)
4. [读一帧的返回值 `Msg`](#4-读一帧的返回值-msg)
5. [API 参考](#5-api-参考)
6. [异常](#6-异常)
7. [注意事项](#7-注意事项)
8. [架构](#8-架构)
9. [命令行](#9-命令行)
10. [测试](#10-测试)

---

## 1. 环境要求与安装

- Python **3.9 及以上**
- `pyserial >= 3.4`（唯一依赖）
- 固件 **`Litearm1.5.0` 及以上**，版本串为 `Litearm<主.次.修>-{7J|1J}`
- Linux 需串口权限：`sudo usermod -aG dialout $USER`（重新登录生效）

```bash
pip install -e .            # 安装
pip install -e ".[dev]"     # 开发（含 pytest）
```

`pip` 与 `python` 指向不同解释器时用 `python -m pip`。

---

## 2. 快速开始

```python
import litearm as pa

arm = pa.Arm().connect()          # 找串口 + 校验固件版本
arm.enable()                      # 运动前必须先使能
arm.movej([0.1, 0, 0, 0, 0, 0, 0], speed=0.3)
print(arm.get_tcp().value)        # 读值要走 .value，见 §4
arm.close()
```

`connect()` 返回时握手已完成，`arm.n` / `arm.firmware` 一定可用。

每个会话有后台读线程，**用完必须 `close()`**，`with` 会自动关：

```python
with pa.Arm().connect() as arm:
    print(arm.get_state().value.q)
```

下面各节出现的 `arm` 都指这个已连好的会话对象；片段只写该节要讲的那几步。

子进程不能用继承来的会话，见 [README](../README.zh-CN.md#多进程fork-之后子进程不能用继承来的-arm)。

---

## 3. 连接管理

```python
Arm(port=None, *, transport_factory=None, min_firmware=MIN_FW,
    q_tol=0.03, dq_tol=0.10, arrive_frames=3, move_timeout=15.0)

connect(port=None) -> Arm
close() -> None
disconnect() -> None                 # close() 的别名
reconnect(port=None) -> Arm          # 等于 close() 再 connect()
__enter__()                          # with 用法
__exit__(*exc)                       # 返回 False，不吞异常
__del__()                            # 回收兜底
```

### 构造参数

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `port` | `None` | 串口路径。`None` 表示自动发现（VID:PID `1d50:606f`）。SDK **不读环境变量**；`LITEARM_PORT` 只被 `examples/_common.py` 使用 |
| `transport_factory` | `None` | 注入传输层（测试用），需同时传占位 `port` |
| `min_firmware` | `(1, 5, 0)` | 版本门下限 |
| `q_tol` | `0.03` | 到位判据：关节角容差（rad） |
| `dq_tol` | `0.10` | 到位判据：关节速度容差 |
| `arrive_frames` | `3` | 到位判据：连续满足的帧数 |
| `move_timeout` | `15.0` | 运动超时（s） |

### 会话方法

| 方法 | 注意 |
| --- | --- |
| `connect()` | 幂等。任何一步失败都**先关链路再抛**，不会留下半开的会话 |
| `close()` | 幂等。停读线程 → 关传输。之后所有入口抛 `NotConnectedError` |
| `reconnect()` | 换会话：读线程重起，`Msg.hz` 统计归零 |

模块常量：`litearm.MIN_FW`、`litearm.FIRMWARE_PREFIX`。

### 固件版本约定

`firmware` 返回 `Litearm<主.次.修>-{7J|1J}`（如 `Litearm1.8.0-7J`）。

| 固件 | 结果 |
| --- | --- |
| `Litearm1.5.x-*` 及以上 | 接受 |
| `Litearm1.4.x-*` 或更早 | `FirmwareMismatchError` |
| 其它命名 | `FirmwareMismatchError` |

---

## 4. 读一帧的返回值 `Msg`

下面 11 个「读一帧」接口返回 `Msg[T]`：

| # | 入口 | 帧 |
| --- | --- | --- |
| 1 | `get_state()` | `RSP_STATUS` |
| 2 | `get_status_now()` | `RSP_STATUS` |
| 3 | `get_tcp()` | `RSP_TCP` |
| 4 | `get_ff_vec(item)` | `RSP_FF_VEC` |
| 5 | `get_ff_scalar(item, sub)` | `RSP_FF_SCALAR` |
| 6 | `params.get_joint_param(idx)` | `RSP_JOINT_PARAM` |
| 7 | `model.get_body(idx)` | `RSP_MODEL_PARAM` |
| 8 | `model.get_jm()` | `RSP_MODEL_JM` |
| 9 | `model.status()` | `RSP_MODEL_STATUS` |
| 10 | `model.get_gravity(q)` | `RSP_GRAVITY` |
| 11 | `diag.kin_bench()` | `RSP_KIN_BENCH` |

```python
@dataclass(frozen=True)
class Msg(Generic[T]):
    value: T          # 原始返回值（取不到帧时是 None）
    hz: float         # 这类帧在本会话的平均到达频率
    timestamp: float  # 最近一帧的本地 time.monotonic()（从未收到为 0.0）
```

`hz` = 该类帧自本会话首次到达起的平均频率，**样本不足 2 条时为 `0.0`**。

- 被动连续流（`RSP_STATUS`，100 Hz）：两三帧后收敛到约 100，链路空闲不会衰减。
- 单发请求/应答式的 4 个（`params.get_joint_param` / `model.get_body` / `model.get_jm` /
  `model.get_gravity`）：一次调用只到一帧，**第一次调用必然 `hz == 0.0`**，第二次起等于
  **你自己的轮询频率**。
- ⚠ `diag.kin_bench()` **不在**上面那一组：它的回执是**连续两帧**（耗时帧 + LINK 帧），
  一次调用到 2 帧，所以第一次调用 `hz` 就非 0，而且那个数**没有意义**（分子被拆帧放大、
  分母还是调用间隔）。判"有没有读到"要看 `timestamp`。
- `reconnect()` 后归零。

**`hz == 0.0` 且 `timestamp == 0.0` 表示这类帧从没到过**，不是链路慢。

### 不返回 `Msg` 的入口

- `move_*` 与 `home()` —— 返回动作结果（`RobotState` / `CartPlan`）
- `n` / `firmware` / `last_reset_reason` / `zero_g_active` —— 没有帧
- `ik()` —— 计算请求
- `get_ff_mask()` 返回裸 `int`；`params.all_joint_params()` 返回 `list[JointParam]`

### `RobotState`

```python
get_state(refresh=False, timeout=0.5) -> Msg[Optional[RobotState]]
get_status_now(timeout=0.5)           -> Msg[RobotState]
```

| 字段 | 说明 |
| --- | --- |
| `mode` / `mode_name` | 当前模式 |
| `flags` / `flag_names` | 标志位与名字 |
| `seq` | 状态帧到达序号，用来判断数据流是否还在动 |
| `joints` | `list[JointState]` |
| `joint_fault` | 逐轴掉线位图 |

派生属性：`n`、`enabled`、`cart_busy`、`q`、`dq`、`tau`、`fault_axes`、`faulted`、
`fault_detail`、`drop_hold_inferred`。

`JointState`：`q`、`dq`、`tau`、`t_mos`、`t_coil`、`err`。

`get_status_now()` 会**主动发一条 `GET_STATUS`**，与只消费被动流的 `get_state()` 不同，
可用来确认链路活性。

⚠ `get_status_now(timeout=0.0)` **不是**"非阻塞探一帧"——它的意思是**立刻返回当前缓存**。
本会话还**一帧都没收到过**时它抛 `MotionTimeoutError`。

---

## 5. API 参考

位姿是 6 个数：位置 3（m）+ 姿态 3（rad，RPY）。list 和 tuple 都收。

```python
pose = [0.30, 0.0, 0.35, 3.1416, 0, 0]

arm.move_p(pose)
```

也接受「位置 3 个数 + 3×3 旋转矩阵」这种写法：

```python
arm.move_p(([0.30, 0.0, 0.35], [[1, 0, 0], [0, 1, 0], [0, 0, 1]]))
```

返回类型不统一，每个入口的 docstring 都写明了自己是哪一种：

| 返回类型 | 入口 |
| --- | --- |
| `Msg[...]` | 上面 11 个「读一帧」接口 |
| `RobotState` | `movej` `movej_sync` `move_p` `home` |
| `CartPlan` | `move_l` `move_c` `move_path` |
| `list[float]` | `ik` |

### 5.1 生命 / 安全

```python
enable(attempts=12)
disable()
emergency_stop()
reset()
clear_faults()
set_motion_mode(mode)
park()
```

| 方法 | 注意 |
| --- | --- |
| `enable(attempts=12)` | 使能全部关节。重试是白名单，**只有 `(0x10, 0x03)` 会重试** |
| `disable()` | 切断位置环。此后机械臂不再被托住 |
| `emergency_stop()` | 单帧、单向、不读状态，唯一没有前置条件的入口 |
| `reset()` | 软件状态复位，**不是 MCU 重启**，同一对象仍可用 |
| `clear_faults()` | 只清 RAM 故障位，不写 flash |
| `set_motion_mode(mode)` | 固件只认 `0`，其它值本地抛 `InvalidCommandError` |
| `park()` | 等价于 `set_motion_mode(0)` |

### 5.2 关节运动

```python
movej(q, speed=1.0) -> RobotState
movej_sync(q, speed=1.0) -> RobotState
move_js(q, dq=None, tau_ff=None) -> None
home(*, timeout=None) -> RobotState
```

| 方法 | 注意 |
| --- | --- |
| `movej(q, speed=1.0)` | 单发：固件规划 S 曲线并走完，到位后静止保持。`speed` ∈ `0..1`。**不校验关节限位**，越限目标会被截断后走满行程 |
| `movej_sync(q, speed=1.0)` | 同步点到点，各轴一起到位 |
| `move_js(q, dq=None, tau_ff=None)` | 低层关节流，绕过规划。`dq` 是速度参考不是限位；需调用方 ≥10 Hz 重发 |
| `home(*, timeout=None)` | 回零。**须先 `enable()`**，否则 `ERR{0x2A,0x03}`。`timeout` 关键字专用；固件把速度写死 0.10，不接受 `speed`。固件**允许**从越软限 / 贴端位姿发起 |

`home(speed=0.3)` 会抛 `TypeError`，写 `arm.home()` 或 `arm.home(timeout=30.0)`。

### 5.3 笛卡尔运动

```python
move_p(pose, speed=1.0, pos_tol=0.006, rpy_tol=0.03) -> RobotState
move_l(pose, speed=1.0, wait=True) -> CartPlan
move_c(pose_start, pose_via, pose_goal, speed=1.0, wait=True) -> CartPlan
move_path(poses, speed=1.0, wait=True) -> CartPlan
poll_cart() -> Optional[CartPlan]
set_speed(percent)
```

规划全在固件里，PC 只发点、收 `0x4E` 结果帧。

| 方法 | 末端走什么 |
| --- | --- |
| `move_p(pose)` | 关节空间插值，点到点，**不是直线** |
| `move_l(pose)` | 直线（位置线性 + 姿态球面插值） |
| `move_c(start, via, goal)` | 圆弧（三点定圆，`via` 的姿态被忽略） |
| `move_path(poses)` | 依次经过多个路点，**尖角** |

| 方法 | 注意 |
| --- | --- |
| `move_p` | 只收单个位姿，传序列抛 `InvalidCommandError`。到位判据是 TCP 容差 |
| `move_l` / `move_c` / `move_path` | 返回 `CartPlan`。`wait=False` 时不阻塞，用 `poll_cart()` 查进度 |
| `move_c` | `start` 必须与调用时的实测 TCP 一致（容差 6 mm / 0.03 rad），写成 `arm.get_tcp().value` |
| `poll_cart()` | 只读收集器的待认领队列，不碰链路 |
| `set_speed(percent)` | 全局、**持续**的调速器。`percent` 是 **0..100 的整数百分比**——`set_speed(1)` 就是 **1% 速度**，不是"满速"；它和 `movej(speed=0..1)` 的单条轨迹倍率不是一回事 |

能力边界：无拐角倒角，无下发前预览，速度预检只在固件里。

失败都抛 `CartesianPlanError`，整条拒绝、臂一步没动：
`err=1` 逆解无解 / `err=2` 三点共线 / `err=3` 超容量、不可达。

### 5.4 位姿 / 运动学

```python
get_tcp(timeout=0.6) -> Msg[Optional[tuple]]
ik(pose, q_seed=None, timeout=3.0) -> list[float]
```

| 方法 | 注意 |
| --- | --- |
| `get_tcp()` | 当前末端位姿（固件正运动学），6 个数 |
| `ik(pose, q_seed=None)` | 反解。可能返回另一个同样有效的分支，与 `q_seed` 差得远未必是错误 |

没有 `fk(q)`。PC 侧不带运动学模型，正运动学只有 `get_tcp()` 这一条路。

### 5.5 前馈 / 动力学调参

```python
set_ff_mask(mask)
ff_preset(preset)                        # 0 / 1 / 2
set_ff_vec(item, values)                 # values 长度必须 = n
set_ff_scalar(item, sub, value)
get_ff_vec(item, timeout=1.0) -> Msg[list[float]]
get_ff_scalar(item, sub=0, timeout=1.0) -> Msg[float]
get_ff_mask(timeout=1.0) -> int          # 裸 int，不是 Msg
set_gravity_scale(gs)                    # 长度 = n
set_inertia_scale(isc)                   # 长度 = n
set_payload(mass, com=(0.0, 0.0, 0.0))
set_gravity_vector(g)                    # 长度 3
```

`get_ff_mask()` 返回裸 `int`，要信封就调 `get_ff_scalar`。

`set_ff_vec` 的 item 12~15 只有通用入口：`12 zg_kp` / `13 zg_kd` / `14 zg_damping`
（拖动示教用）、`15 kd_extra`（软件微分阻尼，可治 `movej` 起步振铃）。
**`kd_extra` 的腕部 J5–J7 必须留 0**（出厂 `[6,6,6,6,0,0,0]`）。

item 名表是 `Arm` 的类属性，也是入参校验白名单：`FF_VEC_ITEMS`（1..15）、
`FF_SCALAR_ITEMS`（1..18，缺 9）、`FF_SCALAR_RO_ITEMS`（只有 9）。

写的是 RAM，要持久化得调 `save_params()`。

`set_ff_vec` 写入时，**任一分量是 `NaN` 会被固件整组拒绝**（`ERR{0x26,0x02}`）；
幅值超限则**静默钳制**到该 item 的合法区间，不报错。

### 5.6 拖动示教

```python
zero_g(period=0.04)              # 上下文管理器
zero_g_start(period=0.04)
zero_g_stop(raise_on_lost=False)
```

`zero_g()` 是客户端组合（`zero_g_start` + `zero_g_stop`），不是单条命令。

```python
with arm.zero_g():
    input("拖动机械臂，然后回车")
```

- 保活由 SDK 后台线程自动重发，`period` 必须在 `[0.005, 0.10)` 内。
- 保活期拒绝其它动作命令，查询类不受限，急停 / 失能例外。
- 退出是异步的，`zero_g_stop()` 返回后固件侧还要一点时间收尾。
- 保活因写失败中断时，退出抛异常而不是静默。
- 只读属性 `zero_g_active` / `zero_g_error` 可查状态。

### 5.7 透传 / 伺服

```python
send_mit(idx, q, dq, kp, kd, tau)
send_mit_all(q, dq, kp, kd, tau)
```

绕过运动规划，**调用方必须自己保活**：需 ≥10 Hz 重发，否则 0.1 s 看图门狗进入 fail-soft
（降刚度 + τ=0），机械臂在重力下缓慢塌下去。

`send_mit_all` 的五个数组长度必须都等于 `n`（本地校验），且**必须是有限数** ——
`NaN` / `Inf` 会被**固件**整帧拒收，回 `ERR{cmd,0x02}`。`send_mit` / `move_js` 同样校验有限性。

本组入口在真机上未经完整验证，见[排障指南 §16](../TROUBLESHOOTING.zh-CN.md#16-尚未验证的部分)。

### 5.8 子对象

#### `arm.params.*` —— 关节级参数

```python
set_joint_param(idx, kp, kd, tau_max)
set_joint_limits(idx, q_min, q_max)
get_joint_param(idx, timeout=1.0) -> Msg[JointParam]
all_joint_params() -> list[JointParam]
reset_factory()
```

`JointParam`：`idx`、`kp`、`kd`、`tau_max`、`q_min`、`q_max`。

`set_joint_limits()` **只许收窄**，写回当前值会被判成放宽请求并拒（`ERR[23,2]`），
所以它不幂等，别拿它做读写回环。`reset_factory()` 要求失能态，且不可逆。

#### `arm.model.*` —— 动力学模型在线导入

```python
probe() -> bool
get_body(idx, timeout=1.0) -> Msg[list[float]]
set_body(idx, vals)              # 需 10 个值
get_jm(timeout=1.0) -> Msg[list[float]]
set_jm(vals)                     # 需 7 个值
status(timeout=1.0) -> Msg[ModelStatus]
get_gravity(q, timeout=1.0) -> Msg[list[float]]
commit(expected_mask)
revert()
```

`ModelStatus`：`override`、`staged_mask`、`dirty`。

写入进 staging 层，不立即生效，判据看 `staged_mask`。`commit(expected_mask)` 才应用，
`revert()` 丢弃。**两者都要求失能态**，已使能时分别回 `ERR{0x32,0x04}` / `ERR{0x37,0x04}`。

⚠ `revert()` **只回退 RAM，不动 flash**，三条后果必须知道：

1. flash 里那份导入模型还在，**重新上电会复活**；
2. 此后任何一次 `save_params()` 会把 flash 里那份**一并抹掉**（整扇区擦除，不可恢复）；
3. 回退后 `status().dirty == 1`（RAM ≠ flash），但**别**据此提示"补固化"。

**`set_jm()` 建议永不调用**：它改关节映射（含符号），改错有乱飞风险，
而本机唯一的恢复手段本身也不可逆。

#### `arm.log.*` —— 300 Hz 控制拍采集

```python
start(n_ticks)
stop()
reader(timeout=1.0, retries=3) -> LogReader
capture(n_ticks, timeout=1.0, retries=3, record_timeout=None) -> list[LogSample]
dump(path, wait=True, timeout=1.0, retries=3, record_timeout=None) -> int
```

`r = arm.log.reader()` 拿到的 `LogReader` 提供：

```python
r.total()                              # 已落盘的拍数
r.wait_for(n_ticks, timeout=None, poll=0.05) -> int
r.read_all() -> bytes
r.samples() -> list[LogSample]
r.iter_chunks() -> Iterator[bytes]
r.total_bytes                          # 属性
```

`LogSample`：`tick`、`q_ref`、`dq`、`tau`。
常量：`LOG_MAX_SAMPLES = 2400`（约 8 s @300 Hz，记满自停）、`CTRL_HZ = 300`。

`reader()` 按固件游标分块读回，掉帧自动重试。`dump()` 落盘原始字节流。

#### `arm.diag.*` —— 固件自检

```python
kin_bench(timeout=8.0) -> Msg[KinBenchResult]
```

`KinBenchResult`：`raw`、`timings`、`link`，以及 `crc_errors`、`reply_dropped`、
`can_tx_fail`、`loop_max_kcycle`、`loop_overruns`、`rx_fifo_lost_motor`、
`rx_fifo_lost_bridge`、`gsusb_ring_drops`。

它是回链路诊断计数的唯一来源，但**计数器全 0 可能是根本没读到**，
见[排障 §11](../TROUBLESHOOTING.zh-CN.md#11-kin_bench-的计数器全是-0)。

### 5.9 固件升级（DFU）

```python
enter_dfu(timeout=0.3) -> None
```

唯一的终端态操作，免探针进 ROM bootloader。

- 两段式：`ACK{0x15}` 只表示已登记，还要等设备真的从 CDC 上消失。
- 使能中本地拒绝（跳转会停 TIM3，电机 100 ms 松开）。
- 成功后本 `Arm` 不可再用（所有入口抛 `ArmIsInDfuError`，`close()` 例外），
  设备重新枚举成 `0483:DF11`，烧完固件新建一个 `Arm`。
- 超时未消失则抛异常，对象照旧可用。

进 DFU 后不能立刻刷，要等 USB 重新枚举。

### 5.10 参数持久化

```python
save_params() -> None
```

写 flash，不可逆。

### 5.11 只读属性

```python
params / model / log / diag      # 子对象
last_reset_reason                # "normal" / "iwdg-rst" / None
zero_g_active / zero_g_error

n                                # 关节数
firmware / fw_version            # 版本串 / 元组
min_firmware / q_tol / dq_tol / arrive_frames / move_timeout
bench_model_axis                 # 台架标定轴
```

`last_reset_reason` 常态是 `None`，那是正确行为：开机签名只在真 MCU 复位后发一次，
`reset()` 不会让它重发。

---

## 6. 异常

全部继承 `LiteArmError`。

```python
from litearm import (
    LiteArmError, NotConnectedError, ForkedSessionError, TransportError,
    FirmwareMismatchError, InvalidCommandError, MotorFaultError,
    MotionTimeoutError, IKError, CommandRejectedError, UnsupportedByFirmwareError,
    CartesianPlanError, MotionSupersededError, CartReplyLostError,
    ArmIsInDfuError,
)
```

| 异常 | 何时抛 |
| --- | --- |
| `NotConnectedError` | 未连接时调用，或 `close()` 之后调用 |
| `ForkedSessionError` | 子进程里使用继承来的会话（`NotConnectedError` 子类），零下发 |
| `TransportError` | 串口读写失败，或帧 CRC 校验不过 |
| `FirmwareMismatchError` | 固件不符合命名约定，或低于下限 |
| `InvalidCommandError` | 参数非法（长度、越界、类型），大多在本地就拦下 |
| `MotorFaultError` | 状态帧出现 FAULT 标志或 EMERGENCY |
| `MotionTimeoutError` | 运动在 `move_timeout` 内未到位 |
| `IKError` | 反解失败 / 目标不可达 |
| `CommandRejectedError` | 固件明确回 `ERR`，带 `.cmd` / `.code` |
| `UnsupportedByFirmwareError` | 固件没实现这条命令（`code == 0x00`），是 `CommandRejectedError` 的子类 |
| `CartesianPlanError` | 固件规划被拒。直挂 `LiteArmError`，**不**继承 `InvalidCommandError` |
| `MotionSupersededError` | 笛卡尔请求被新请求取代，是预期内的接管，不是失败 |
| `CartReplyLostError` | `0x4E` 应答丢失，结局未知 |
| `ArmIsInDfuError` | 本 `Arm` 已把设备交给 ROM bootloader，终端态 |

`except InvalidCommandError` 不覆盖 `CartesianPlanError`，固件回的是规划结果，
不是命令被拒绝。

错误码 `ERR{cmd, code}` 里 `code == 0x00` 恒表示固件没有这条命令。

---

## 7. 注意事项

1. **臂绝不能飞出去。** 这是硬线，也是每次安全改动的验收判据。
2. **子进程不能用继承来的会话**，fail-closed，零下发。父进程必须先释放端口。
3. **`movej` 返回 ≠ 停稳**，残差约 0.012 rad，在 `q_tol` 内。
4. **`movej` / `movej_sync` 不校验关节限位**，发之前自己比对。
5. **`disable()` 之后臂不再被托住。**
6. **`save_params()` 没有撤销。**

### 不可逆命令：不要在标定过的臂上执行

下面四条会覆盖或抹掉该台设备逐台辨识的动力学模型，唯一解锁方式是在一台没有标定价值的板子上做：

| 命令 | 入口 |
| --- | --- |
| `0x25` | `save_params()` |
| `0x32` | `arm.model.commit()` |
| `0x36` | `arm.params.reset_factory()` |
| `0x37` | `arm.model.revert()` |

`model.set_jm()` 建议永不调用，改错关节映射有乱飞风险，而没有可依赖的退路。

压测 CAN 链路只开 `candump`（只读），绝不 `cangen`：`can0` 就是电机总线。

尚未验证的部分见[排障指南 §16](../TROUBLESHOOTING.zh-CN.md#16-尚未验证的部分)。

---

## 8. 架构

每个会话在 `connect()` 时起一条后台读线程（`litearm-reader`，daemon）。
它是全包唯一碰传输层读口的地方，只做两件事：**把帧投递到该去的队列、死了要响亮。**

```text
一条读线程：  帧来了 → 按 (帧号, 回显码) 放进对应的队列
其余所有人：  发命令 → 在自己那几条队列上等
```

帧的归属由它落在哪条队列决定，不由线程决定。`ACK{0x10}` 与 `ACK{0x11}` 天然落在两条队列，
所以并发命令不会互吃应答。`RSP_STATUS`（100 Hz 连续流）不进队列，它进单槽 + 到达序号。

两条规矩：**读线程只投递不判定**；**读线程死了要响亮**，异常存进 `_reader_error`
唤醒所有等待者，绝不静默退出。

代价是每个会话多一条线程，这也是子进程不能继承会话的原因。

不解决的：线上没有请求 id，两条完全相同的命令并发时无法配对；固件只装一条待执行的
笛卡尔规划，所以笛卡尔并发深度是 1。

### 文件

```text
src/litearm/
  _protocol.py     帧编解码 + 命令/应答常量 + 状态帧解析 + 命令覆盖契约
  transport.py     pyserial CDC 读写 / 自动发现
  state.py         RobotState / JointState
  errors.py        异常体系 + 错误码文案
  arm.py           Arm 核心 + 命令行 + 读线程 + fork 守卫
  cart.py          笛卡尔：结果帧配对 + 到位判据
  params.py        arm.params.*
  model.py         arm.model.*
  log.py           arm.log.*
  diagnostics.py   arm.diag.*
  _rot.py          旋转 / 位姿纯数学
  testing.py       离线桩（FakeTransport）
```

固件里每条已实现的下行命令都有对应入口，契约落在 `_protocol.COMMAND_COVERAGE`，
由 `tests/test_protocol_sync.py` 直接解析固件头文件双向强制。

---

## 9. 命令行

```bash
litearm-python [--port PORT] [ACTION] [TARGETS...] [--speed SPEED]
python -m litearm ...                 # 等价
```

`ACTION` ∈ `status`（默认）/ `fw` / `enable` / `disable` / `reset` / `emergency` /
`movej` / `home` / `tcp`。

```bash
litearm-python status                          # 只读
litearm-python fw                              # 版本串 + 轴数
litearm-python tcp                             # 当前位姿 + 帧率
litearm-python movej -0.1 0 0 0 0 0 0 --speed 0.3
```

`movej` 要求 `TARGETS` 个数正好等于 `arm.n`；`home` 的 `--speed` 被忽略（固件写死 0.10）。

---

## 10. 测试

```bash
pytest                         # 离线全流程，不碰真机
LITEARM_LIVE=1 pytest          # 额外跑真机用例
python tests/test_offline.py   # 不装 pytest 也能跑
```

跑测试不需要装包，`tests/conftest.py` 会把 `src/` 与 `tests/` 塞进 `sys.path`。
无人在场时不要设 `LITEARM_LIVE`，也绝不调用 `enter_dfu()` / `reset_factory()`。

| 测试 | 守什么 |
| --- | --- |
| `test_protocol_sync.py` | 协议漂移防护，直接解析固件头文件比对命令集合与 ID。固件仓库位置由环境变量 `LITEARM_FW_DIR` 指定（默认 `~/litearm-stm32`），**找不到时 skip 而不是 pass** |
| `test_protocol_crc.py` | CRC16，用外部权威检查值 + 独立参考实现 |
| `test_frame_ownership.py` | 帧没被静默销毁，它的主人拿到了它 |
| `test_transport.py` | 真实字节流解析（连续帧 / 噪声 / 坏帧 / 半帧） |
| `test_ack_echo.py` | 应答必须回显原命令，迟到的旧应答不得让新命令假成功 |
| `test_capability.py` | 固件没有这条命令 → `UnsupportedByFirmwareError` |
| `test_fork_guard.py` | fork 守卫：子进程零下发 |
| `test_zero_g.py` | 拖动示教保活周期 / 退出收尾 / 线程回收 / 命令门禁 |
| `test_live.py` | 真机冒烟，`LITEARM_LIVE=1` 才跑 |

`tests/fake_serial.py` 的桩必须与固件真实布局一致，否则离线假绿而真机必挂。
