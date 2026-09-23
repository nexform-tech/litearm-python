# litearm-python 开发者指南 · API 参考

LiteArm 机械臂的 Python SDK —— **直连 `litearm-stm32` 固件**（USB CDC 串口）。

本 SDK 是一份**薄协议绑定**：PC 侧只编解码帧、下发命令、判定到位。
**轨迹规划、运动学、动力学全在固件里**（B2 S 曲线 + B3 运动学 + B4 动力学 + B1 控制律），
PC 侧**不做**这些 —— 不然同一个约定写两遍，改一处漏另一处就静默变成两套语义。

除 `pyserial` 外**零依赖**。位姿是**纯 Python list**，不需要 numpy。

## 目录

1. [要求与安装](#1-要求与安装)
2. [快速开始](#2-快速开始)
3. [连接管理](#3-连接管理)
4. [读状态 —— 返回信封 `Msg`](#4-读状态--返回信封-msg)
5. [API 参考](#5-api-参考)
6. [异常](#6-异常)
7. [架构 —— 一条读线程](#7-架构--一条读线程)
8. [命令行](#8-命令行)
9. [测试](#9-测试)
10. [安全须知](#10-安全须知)

---

## 1. 要求与安装

- Python **>= 3.9**
- `pyserial >= 3.4`（唯一依赖）
- 固件 **`Litearm1.5.0+`**，约定为 `Litearm<主.次.修>-{7J|1J}`

```bash
pip install -e .

# 开发（含 pytest）
pip install -e ".[dev]"
```

> ⚠ `pip` 与 `python` 指向不同解释器时，一律用 `python -m pip`，
> 让包装进你真正运行的那个解释器。

---

## 2. 快速开始

```python
import litearm as pa

arm = pa.Arm().connect()          # 自动找 CDC + 校验固件版本约定
arm.enable()                      # 运动前必须先使能
arm.movej([0.1, 0, 0, 0, 0, 0, 0], speed=0.3)
print(arm.get_tcp().value)        # 读值走 .value（2.0 起的返回信封，见 §4）
arm.close()
```

`Arm().connect()` 是**唯一入口**。`connect()` 返回时握手已完成，`arm.n` / `arm.firmware`
一定可用。

每个会话带**一条后台读线程**，所以每个 `Arm` 都要 `close()` —— 用 `with` 可以省掉：

```python
with pa.Arm().connect() as arm:
    print(arm.get_state().value.q)
# 退出 with 即 close()
```

> ⚠ **`fork` 之后子进程不能继承这个会话**，详见 [README](../README.zh-CN.md#1-多进程--fork子进程不能用继承来的-arm)。

---

## 3. 连接管理

```python
Arm(port=None, *, transport_factory=None, min_firmware=MIN_FW,
    q_tol=0.03, dq_tol=0.10, arrive_frames=3, move_timeout=15.0)

connect(port=None) -> Arm
close() -> None
disconnect() -> None                 # close() 的别名
reconnect(port=None) -> Arm          # 等于 close() 再 connect()
__enter__() / __exit__(*exc)         # with 用法；__exit__ 返回 False（不吞异常）
__del__()                            # GC 兜底，等价于 close()
```

| 参数 | 默认 | 含义 |
|---|---|---|
| `port` | `None` | 串口路径；`None` ⇒ 自动发现 `1d50:606f`。优先级：`port` 参数 > `LITEARM_PORT` > 自动发现 |
| `transport_factory` | `None` | 注入传输（测试用）。⚠ 还要传一个占位 `port`，因为 `connect()` 会先走 `find_cdc_port()` |
| `min_firmware` | `(1, 5, 0)` | 版本门下限 |
| `q_tol` | `0.03` | 到位判据：关节角容差（rad） |
| `dq_tol` | `0.10` | 到位判据：关节速度容差 |
| `arrive_frames` | `3` | 到位判据：连续满足的帧数 |
| `move_timeout` | `15.0` | 运动超时（s） |

| 方法 | 注意 |
|---|---|
| `connect()` | **幂等**（同目标重复调直接返回 `self`）。⚠ 握手**写**失败时可能留下半开会话，重连会**静默报成功** |
| `close()` | 幂等。停读线程 → 关传输。之后所有入口抛 `NotConnectedError`（`close()` 自己例外） |
| `reconnect()` | 会**换会话**：读线程重起、`Msg.hz` 统计归零 |

模块常量：`litearm.MIN_FW`、`litearm.FIRMWARE_PREFIX`。

### 固件版本约定

`firmware` 返回 `Litearm<主.次.修>-{7J|1J}`（如 `Litearm1.8.0-7J`）。

| 固件 | 结果 |
| --- | --- |
| `Litearm1.5.x-*` 及以上 | ✅ 接受 |
| `Litearm1.4.x-*` 或更早 | ❌ `FirmwareMismatchError` |
| `A1.x-*-USB`（旧命名） | ❌ 不符合约定 |

> 状态帧解析**同时兼容** `4+21N`（≤1.4.x）与 `6+21N`（≥1.5.0）两种布局 ——
> 那段兼容分支只用于离线/历史帧解析（例如分析抓包），`connect()` 走不到。

---

## 4. 读状态 —— 返回信封 `Msg`

### 哪些入口返回 `Msg`

**11 个「读一帧」型 getter** 返回 `Msg[T]`（2.0 起的破坏性变更）：

| # | 入口 | 帧 |
|---|---|---|
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
    value: T          # 原返回值（取不到帧的入口这里是 None）
    hz: float         # 该类帧在本会话的平均到达频率
    timestamp: float  # 最近一帧的本地 time.monotonic()（从没收到过则为 0.0）
```

**`hz` 的口径（写死，不是估的）**：**该类帧自本会话首次到达起的平均频率**
`(到达条数 − 1) / (最近一帧时刻 − 首帧时刻)`，**样本不足 2 条时是 `0.0`**。

- 被动连续流（`RSP_STATUS`，100 Hz）：两三帧后收敛到 ~100。链路空闲**不会**让它衰减。
- 单发请求/应答式那 5 个（`get_joint_param` / `get_body` / `get_jm` / `get_gravity` / `kin_bench`）：
  一次调用只到一个帧 ⇒ **第一次调用必然 `hz == 0.0`**，第二次起它等于**你自己的轮询频率**。
- `reconnect()` 后统计归零。

⇒ **`hz == 0.0` 且 `timestamp == 0.0` 是"这一类帧从没到过"的判据**，不是"链路慢"。

### 哪些入口**不**返回 `Msg`

- `move_*` 与 `home()` —— 返回的是「动作结果」（`RobotState` / `CartPlan`），不是「读一帧」
- `n` / `firmware` / `last_reset_reason` / `zero_g_active` —— 根本没有帧
- `ik()` —— 一次**计算**请求
- `license()` —— **请求/应答式的设备身份记录**（没有固件发起的流量，`hz` 只会度量你自己轮询的频率）
- 两个**派生** getter：`get_ff_mask()` 仍是裸 `int`（它是 `get_ff_scalar(9,0)` 的标量投影）；
  `params.all_joint_params()` 仍是 `list[JointParam]`（它是 N 次往返的聚合，一个 `hz` 描述不了 N 帧）

### `RobotState`

```python
get_state(refresh=False, timeout=0.5) -> Msg[Optional[RobotState]]
get_status_now(timeout=0.5)           -> Msg[RobotState]
```

| 字段 | 说明 |
|---|---|
| `mode` / `mode_name` | 当前模式 |
| `flags` / `flag_names` | 原始标志位与名字 |
| `seq` | 状态帧到达序号 —— **判别"流是否还在动"** |
| `joints` | `list[JointState]` |
| `joint_fault` | 固件 G7 逐轴掉线位图（1.5.0 起；旧布局恒 0） |

派生属性：`n`、`enabled`（flags bit9）、`cart_busy`（flags bit10）、`q`、`dq`、`tau`、
`fault_axes`、`faulted`、`fault_detail`、`drop_hold_inferred`。

`JointState`：`q`、`dq`、`tau`、`t_mos`、`t_coil`、`err`。

> ⚠ `get_status_now(timeout=0.0)` **不是**"非阻塞探一帧"，它的意思是
> **"立刻返回当前缓存"**。

---

## 5. API 参考

位姿 = **6 个数**的纯 Python list：位置 3 + RPY 3。

```python
pose = [px, py, pz, rx, ry, rz]

# 四种写法都会被 as_pose() 归一化接受
arm.move_p([0.30, 0.0, 0.35, 3.1416, 0, 0])
```

> ⚠ **返回形状不统一**：`get_state()` / `get_status_now()` / `get_tcp()` 给 `Msg` 信封；
> `movej` / `movej_sync` / `move_p` / `home` 给 `RobotState`；
> `move_l` / `move_c` / `move_path` 给 `CartPlan`；`ik()` 给 `list[float]`。
> **每个入口的 docstring 都写明了自己的形状。**

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
|---|---|
| `enable(attempts=12)` | 使能全部关节。`attempts` 是重试次数，**重试是白名单**：只有 `(0x10, 0x03)` 会重试，其余码重发无用 |
| `disable()` | 切断位置环。⚠ 使能一断，臂不再被托住 |
| `emergency_stop()` | 单帧、单向、不读状态 —— **唯一没有前置条件的入口** |
| `reset()` | 清故障 + 重锚控制环。⚠ 它是**软件状态复位，不是 MCU 重启**（无 USB 重枚举，同一对象仍可用） |
| `clear_faults()` | 只清 RAM 故障位，**不写 flash** |
| `set_motion_mode(mode)` | **固件只认 `0`**，其它值本地抛 `InvalidCommandError`（fail-closed） |
| `park()` | 等价于 `set_motion_mode(0)` |

### 5.2 关节运动

```python
movej(q, speed=1.0) -> RobotState
movej_sync(q, speed=1.0) -> RobotState
move_js(q, dq=None, tau_ff=None) -> None
home(*, timeout=None) -> RobotState
```

| 方法 | 注意 |
|---|---|
| `movej(q, speed=1.0)` | **单发**：固件规划 S 曲线、自完成、到位后静止保持（无需 PC 逐帧保活）。`speed` ∈ `0..1`。⚠ **不校验关节限位** —— 越限目标会被固件 `clampf` 后**照走满行程**。发之前自己比对限位 |
| `movej_sync(q, speed=1.0)` | 同步 PTP，各轴一起到位 |
| `move_js(q, dq=None, tau_ff=None)` | 低层关节流，**绕过规划**。⚠ `dq` 是**速度参考，不是限位**；**需调用方 ≥10 Hz 重发**，否则 0.1 s 看门狗 fail-soft |
| `home(*, timeout=None)` | 固件 `CMD_HOME 0x2A`。⚠ `timeout` 是**关键字专用**；固件把速度**写死 0.10，不接受 speed**。与 `movej` 不同，固件**明确允许**从越软限/贴端位姿发起 `home` |

> ⚠ `home(speed=0.3)` 会抛 `TypeError`。写 `arm.home()` 或 `arm.home(timeout=30.0)`。

### 5.3 笛卡尔（**固件规划**）

```python
move_p(pose, speed=1.0, pos_tol=0.006, rpy_tol=0.03) -> RobotState
move_l(pose, speed=1.0, wait=True) -> CartPlan
move_c(pose_start, pose_via, pose_goal, speed=1.0, wait=True) -> CartPlan
move_path(poses, speed=1.0, wait=True) -> CartPlan
poll_cart() -> Optional[CartPlan]
set_speed(percent)
```

**规划全在固件里**：PC 只发点、收 `0x4E` 结果帧。三条路径入口的分工：

| 方法 | 末端走什么 |
|---|---|
| `move_p(pose)` | **关节空间**插值（点到点，**不是**直线） |
| `move_l(pose)` | **直线**（位置线性 + 姿态 slerp） |
| `move_c(start, via, goal)` | **圆弧**（三点定圆；`via` 的姿态被忽略） |
| `move_path(poses)` | 依次经过多路点（**尖角**，协议无倒角字段） |

| 方法 | 注意 |
|---|---|
| `move_p` | ⚠ **只收单个位姿**，传序列抛 `InvalidCommandError`。到位判据是 TCP 容差 |
| `move_l` / `move_c` / `move_path` | 返回 `CartPlan`。`wait=False` 时不阻塞，用 `poll_cart()` 查进度 |
| `move_c` | ⚠ `start` **必须与调用时的实测 TCP 一致**（容差 6 mm / 0.03 rad）—— 它是校验收到的，不是自由参数 |
| `poll_cart()` | 只读收集器的待认领队列，**不碰链路** |
| `set_speed(percent)` | 全局调速。⚠ **非线性**（100→50 只慢 1.48×），且入参必须是 `0..100` 的 **`int`** |

**已知的降级（相对 PC 侧规划，2.0 起有意为之）**：**无拐角倒角**、
**无下发前预览**（固件没有 dry-run，`0x4E` 要发出去才回）、
**PC 侧速度预检取消**（判据只在固件手里一份）。

⚠ 失败的三种形态（都抛 `CartesianPlanError`，**整条拒绝、臂一步没动**）：
`err=1` IK 无解 / `err=2` 三点共线 / `err=3` 超容量、不可达。

⚠ **`movel` / `movec` / `movep` 已更名**为 `move_l` / `move_c` / `move_p`，旧名不存在。

### 5.4 位姿 / 运动学

```python
get_tcp(timeout=0.6) -> Msg[Optional[tuple]]
ik(pose, q_seed=None, timeout=3.0) -> list[float]
```

| 方法 | 注意 |
|---|---|
| `get_tcp()` | 当前末端位姿（固件 FK），**6 个数**（不是旋转矩阵） |
| `ik(pose, q_seed=None)` | 反解。⚠ 可能返回**另一个同样有效的分支** ⇒ 与 seed 差得远**未必**是错误 |

> ⚠ **没有 `fk(q)`** —— PC 不带运动学模型，FK 只有"当前反馈"这一条路（`get_tcp()`）。

### 5.5 前馈 / 动力学调参

```python
set_ff_mask(mask)
ff_preset(preset)                        # 0 / 1 / 2
set_ff_vec(item, values)                 # values 长度必须 = n
set_ff_scalar(item, sub, value)
get_ff_vec(item, timeout=1.0) -> Msg[list[float]]
get_ff_scalar(item, sub=0, timeout=1.0) -> Msg[float]
get_ff_mask(timeout=1.0) -> int          # ⚠ 裸 int，不是 Msg
set_gravity_scale(gs)                    # 长度 = n
set_inertia_scale(isc)                   # 长度 = n
set_payload(mass, com=(0.0, 0.0, 0.0))
set_gravity_vector(g)                    # 长度 3
```

⚠ **`get_ff_mask()` 是裸 `int`**（它是 `get_ff_scalar(9, 0)` 的标量投影；
要那一帧的信封就直接调 `get_ff_scalar`）。

⚠ `set_ff_vec` 的 **item 12~15** 只有通用入口，没有具名方法：
`12 zg_kp` / `13 zg_kd` / `14 zg_damping`（零重力拖动示教用）、
`15 kd_extra`（τ 域软件微分阻尼，治 `movej` 起步振铃）——
**`kd_extra` 的腕部 J5-J7 必须留 0**（出厂 `[6,6,6,6,0,0,0]`）。

⚠ item 名表是 `Arm` 上的类属性（也是入参校验用的白名单）：
`FF_VEC_ITEMS`（1..15）、`FF_SCALAR_ITEMS`（1..18，缺 9）、`FF_SCALAR_RO_ITEMS`（只有 9 = `ff_mask`）。

⚠ 写的是 **RAM**，要持久化得调 `save_params()`。

### 5.6 零重力拖动示教

```python
zero_g(period=0.04)              # 上下文管理器
zero_g_start(period=0.04)
zero_g_stop(raise_on_lost=False)
```

`zero_g()` 是**客户端组合**（`zero_g_start` + `zero_g_stop`），**不是 RPC**。

```python
with arm.zero_g():
    input("拖动机械臂，然后回车")
```

- **保活由 SDK 后台线程自动重发**（默认 `period=0.04 s`）—— 固件 `0x06` 自带
  `watchdog_kick`，**0.10 s 不重发即掉出 fail-soft**。`period` 必须 ∈ `[0.005, 0.10)`。
- **保活期拒绝其它动作命令**；**查询类不受限**，**急停/失能例外**。
- **退出是异步的**：`zero_g_stop()` 返回后固件侧还要一点时间收尾。
- 保活若因**写失败**中断，退出时**抛异常**而非静默。
- 只读属性 `zero_g_active` / `zero_g_error` 可查状态。

### 5.7 透传 / 伺服（**第二条通路**）

```python
move_js(q, dq=None, tau_ff=None)
send_mit(idx, q, dq, kp, kd, tau)
send_mit_all(q, dq, kp, kd, tau)
```

⚠ **这三个绕过运动规划，且调用方必须自己保活**：**需 ≥10 Hz 重发**，
否则 0.1 s 命令看门狗 fail-soft（降刚度 + τ=0），臂在重力下缓慢塌下去。

⚠ `send_mit_all` 的五个数组长度必须 = `n`，且必须是**有限数**。

⚠ **本组入口在真机上未经完整验证**（见[TROUBLESHOOTING](../TROUBLESHOOTING.zh-CN.md#明确未验证的部分)）。

### 5.8 子对象

#### `arm.params.*` —— 关节级参数（4）

```python
set_joint_param(idx, kp, kd, tau_max)
set_joint_limits(idx, q_min, q_max)
get_joint_param(idx, timeout=1.0) -> Msg[JointParam]
all_joint_params() -> list[JointParam]
reset_factory()
```

`JointParam`：`idx`、`kp`、`kd`、`tau_max`、`q_min`、`q_max`。

- ⚠ `set_joint_limits()` **只许收窄**：写回**当前值**会被判成"放宽请求"并拒
  （`ERR[23,2]`）⇒ **该入口不幂等**，别用它做读写回环校验。
- ⚠ `reset_factory()` 要求**失能态**，已使能回 `ERR{0x36,0x04}`。**不可逆**。

#### `arm.model.*` —— 动力学模型在线导入（9）

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

- 写入进 **staging 层**，**不生效** —— 判据应是 `staged_mask`，不是"生效层读回"。
- `commit(expected_mask)` 才应用，`revert()` 丢弃。
- ⚠ **`set_jm()` 建议永不调用**：它改关节映射（含符号），改错有**乱飞**风险，
  而本机唯一的恢复手段（`revert` / `save_params`）**本身也不可逆** ⇒ 没有可依赖的退路。
- ⚠ `commit()` / `revert()` 都会**覆盖本机逐台辨识的动力学模型**，见 §10。

#### `arm.log.*` —— 300 Hz 控制拍采集（4 + `LogReader`）

```python
start(n_ticks)
stop()
reader(timeout=1.0, retries=3) -> LogReader
capture(n_ticks, timeout=1.0, retries=3, record_timeout=None) -> list[LogSample]
dump(path, wait=True, timeout=1.0, retries=3, record_timeout=None) -> int
```

```python
r = arm.log.reader()
r.total()                              # 本会话已落盘的拍数
r.wait_for(n_ticks, timeout=None, poll=0.05) -> int
r.read_all() -> bytes                  # 读回全部缓冲
r.samples() -> list[LogSample]
r.iter_chunks() -> Iterator[bytes]
r.total_bytes                          # 属性
```

`LogSample`：`tick`、`q_ref`、`dq`、`tau`。模块常量：`LOG_MAX_SAMPLES = 2400`（≈8 s@300 Hz，**记满自停**）、`CTRL_HZ = 300`。

- `reader()` 是**工厂**：按固件游标 `next_byte` 分块读回，**掉帧自动按游标重试**。
- `dump(path)` 落盘原始字节流（满量程 ≈ 863 次往返，大缓冲建议先落盘再解析）。
- ⚠ **失能态下 `capture()` 恒录到 0 拍** —— 这是**固件行为**，不是缺陷。

#### `arm.diag.*` —— 固件自检（1）

```python
kin_bench(timeout=8.0) -> Msg[KinBenchResult]
```

`KinBenchResult` 的方法/属性：`raw`、`timings`、`link`，以及
`crc_errors`、`reply_dropped`、`can_tx_fail`、`loop_max_kcycle`、`loop_overruns`、
`rx_fifo_lost_motor`、`rx_fifo_lost_bridge`、`gsusb_ring_drops`。

⚠ **它是回链路的唯一诊断计数来源**（`crc` / `reply_dropped` / `can_tx_fail` /
`loop_max_kcycle` / `loop_overruns`）。
⚠ 但**五个计数器全 0 可能是"静默 0"**，见[TROUBLESHOOTING §11](../TROUBLESHOOTING.zh-CN.md#11-kin_bench-的五个计数器全-0--静默-0)。

### 5.9 授权 / 激活（固件 1.8.0+）

```python
license(timeout=1.0) -> LicenseInfo
activate(*, cust_id, issued, flags=0, mac, timeout=2.0) -> None
```

`LicenseInfo`：`state`、`ver`、`uid`、`cust_id`、`issued`、`flags`
＋ 派生 `activated`、`factory_mode`、`state_name`、`uid_hex`。

- 固件在**独立 flash 扇区**（sector 6）存一条记录，**写一次永不擦**；
  未激活时**只锁 `ENABLE`**（`ERR{0x10,0x08}`），其余命令一切照常。
- `license()` **未激活时不抛异常**（它是一种**状态**），且**未激活也回 UID** ——
  那是签发器的唯一来源，别改用 USB 序列号字符串。
- `activate()` **全部参数关键字专用**，`mac` 无默认值。**须先失能**，否则 `ERR{0x3F,0x04}`。
- ⚠ **本包不含密钥，也不含任何算 MAC 的代码** —— 签发在厂商侧工具里。
- ⚠⚠ `ERR{0x3F,0x02}` 是**聚合档**（已激活/MAC 不符/密钥非法/写失败同码）。
  本包在这一档**自动回读 `0x2F`**：设备确实 `state != 0` 就当成功返回，否则才抛。
- 擦除授权记录**只能走 SWD**（`pyocd erase -s 0x080C0000`）—— 固件**没有**擦除命令。

### 5.10 烧录 DFU

```python
enter_dfu(timeout=0.3) -> None
```

**唯一的终端态操作。** 免探针进 ROM 系统 bootloader（`CMD_ENTER_DFU 0x15`）。

- **两段式**：`ACK{0x15}` 只表示"已登记"，还要等设备真的从 CDC 上消失。
- 使能中**本地拒绝**（跳转会停 TIM3 ⇒ 电机 100 ms 松开、有负载则下垂）。
- 成功返回后**本 `Arm` 不可再用**（所有入口抛 `ArmIsInDfuError`，`close()` 例外），
  设备重枚举成 `0483:DF11`，烧完固件**新建一个 `Arm`**。
- 超时未消失则抛"登记被撤销/未执行"，且对象照旧可用。

> ⚠ **进 DFU 后不能立刻刷** —— 要等 USB 重枚举。**先证明能救，再故意弄坏。**

### 5.11 持久化

```python
save_params() -> None
```

写 flash（`0x25`）。⚠ **不可逆**，见 §10。

### 5.12 只读属性

```python
params / model / log / diag      # 子对象
last_reset_reason                # "normal" / "iwdg-rst" / None
zero_g_active                    # bool
zero_g_error                     # Optional[BaseException]

n                                # 关节数（握手后可用）
firmware                         # 版本串，如 "Litearm1.8.0-7J"
fw_version                       # 元组，如 (1, 8, 0)
min_firmware                     # 本包要求的下限
q_tol / dq_tol / arrive_frames   # 到位判据
move_timeout                     # 运动超时
bench_model_axis                 # 台架标定轴
```

> ⚠ **`last_reset_reason` 常态是 `None`，那是正确行为** ——
> 开机 banner **只在真 MCU 复位后发一次**，`reset()` 不会让它重发。
> 见[TROUBLESHOOTING §10](../TROUBLESHOOTING.zh-CN.md#10-last_reset_reason-是-none多数时候正确)。

---

## 6. 异常

全部继承 `LiteArmError`。

```python
from litearm import (
    LiteArmError, NotConnectedError, ForkedSessionError, TransportError,
    FirmwareMismatchError, InvalidCommandError, MotorFaultError,
    MotionTimeoutError, IKError, CommandRejectedError, UnsupportedByFirmwareError,
    CartesianPlanError, MotionSupersededError, CartReplyLostError,
    ArmIsInDfuError, NotRemoteable, NotSupportedOnThisBackend,
    TeleopLockedError, TeleopBusyError,
)
```

| 异常 | 何时抛 |
|---|---|
| `NotConnectedError` | 未连接时调用；`close()` 之后调用 |
| `ForkedSessionError` | **`fork` 出来的子进程**里使用继承来的会话（`NotConnectedError` 的子类）。fail-closed：**一个字节都不下发** |
| `TransportError` | 串口读写失败，或帧 CRC 校验不过 |
| `FirmwareMismatchError` | 固件不符合 `Litearm<主.次.修>-{7J\|1J}` 约定，或低于下限 |
| `InvalidCommandError` | 参数非法（长度、越界、类型）—— 大多在**本地**就拦下 |
| `MotorFaultError` | 状态帧 FAULT 标志或 EMERGENCY（含 G7 单轴故障降级） |
| `MotionTimeoutError` | 运动在 `move_timeout` 内未到位 |
| `IKError` | 反解失败 / 目标不可达 |
| `CommandRejectedError` | 固件明确回 `ERR`。带 `.cmd` / `.code`（**`.cmd` 是固件回显的，比我刚发出去的可信**） |
| `UnsupportedByFirmwareError` | 固件**没有实现**这条命令（`code == 0x00`）。**子类是 `CommandRejectedError`** |
| `CartesianPlanError` | 固件规划被拒（IK 无解 / 三点共线 / 超容量 / 越限）。⚠ 它**直挂 `LiteArmError`**，**不**继承 `InvalidCommandError` |
| `MotionSupersededError` | 笛卡尔请求被新请求取代 —— **预期内的接管，不是失败**。**刻意不**继承 `CartesianPlanError` |
| `CartReplyLostError` | `0x4E` 应答丢失 ⇒ **结局未知**（SDK 自造语义，不是固件码） |
| `ArmIsInDfuError` | 本 `Arm` 已把设备交给 ROM bootloader —— **终端态**，不可复活。**刻意不**是 `NotConnectedError` |
| `NotRemoteable` | 该入口不可远程化 |
| `NotSupportedOnThisBackend` | 当前后端未实现该能力 |
| `TeleopLockedError` / `TeleopBusyError` | 遥操状态拒绝该命令 |

> ⚠ **捕获面的一个有意的变化**：`CartesianPlanError` 2.0 起直挂 `LiteArmError`，
> 所以 `except InvalidCommandError` **不再覆盖它** —— 固件回的是**规划结果**，
> 不是"这条命令被拒绝"。

**错误码**：`ERR{cmd, code}` 里 `code == 0x00` **恒表示"固件没有这条命令"**，
是稳定且唯一的能力探测哨兵。

---

## 7. 架构 —— 一条读线程

### 形状

每个会话在 `connect()` 时起**一条**后台读线程（`litearm-reader`，daemon）。
它是全包**唯一**碰 `transport.read_frame` 的地方，只做两件事：
**把帧投递到它该去的队列、死了要响亮。**

```text
一条读线程：  帧来了 → 按 (帧号, 回显码) 放进对应的队列
其余所有人：  发命令 → 在自己那几条队列上等
```

**帧的归属 = 它落在哪条队列**，**不由线程决定**。
`ACK{0x10}` 与 `ACK{0x11}` 天然落在**两条**队列 ⇒ 并发命令不会互吃应答。

- `RSP_STATUS`（100 Hz 连续流）**不进队列**：它进**单槽** + 到达序号，
  等待方等的是"序号前进"。
- 陈旧应答靠**发帧前清队**挡（在**唯一写口** `_raw_write` 里）；
  `echo_cmd` 对 `ACK`/`ERR` 是**必填** ⇒ 同 id 互吃在**结构上不可能**。

### 只有两条规矩

1. **读线程只投递，不判定。**
2. **读线程死了要响亮。** 传输层异常存进 `_reader_error` → 唤醒所有等待者 →
   由它们抛出。**绝不静默退出。**

### 三条实现约束（都是错不起的）

- **`close()` 必须先停线程，再关传输。** 否则读线程会从已关掉的传输上抛 `TransportError`，
  一次**正常关闭**会被记成"链路丢失"。顺序：`zero_g_stop()` → 停读线程 + `join(1.0)` → 关传输。
- **读线程绑死"起线程那一刻的 `_Ack`"**，不在循环里再读 `self._a` ——
  否则 `reconnect()` 的窗口期会把帧投递到新旧混杂的对象上。
- **`_Ack` 持有 `Arm` 的弱引用。** 线程的 target 是绑定方法 ⇒ 线程强引用 `_Ack`；
  `_Ack` 再强引 `Arm` 的话，**忘了 `close()` 的 `Arm` 再也回收不掉**，
  `__del__` 的兜底收尾永不触发 ⇒ 端口不释放。

### 代价

⚠ 每个会话**多一条线程**。读线程的退避用 `Event.wait` 而非 `time.sleep`
（后者会被测试里"等了多久"的探针抓走成千上万次）。

⚠ **`fork` 之后子进程不可用** —— 这是这套架构**唯一**的系统级代价，
对应 README「两条必读」第 1 条。

### 本机制**不解决**的（别指望）

| 不解决 | 根因 | 在哪一层 |
|---|---|---|
| 两条**完全相同**的命令并发时的配对 | 线上**没有请求 id** | 线协议 |
| 固件 `plan_pending` 单槽 ⇒ 笛卡尔并发深度 1 | 设备侧容量 | 固件 |
| 一条还在传输缓冲/线上的陈旧应答躲过清队 | 同上（无请求 id）——**知情的、不可再约的残余窗口** | 线协议 |

> 📌 实测：长时并发下队列深度**峰值 = 1**（封顶 64 从未接近），
> 固件侧 `crc` / `rxfifo` 计数增量全 0 —— **结构化不丢帧**。

### 传输层的两条性能事实

- **读是"有多少读多少"**（按 `in_waiting` 要、封顶 4096 字节），**不是逐字节读**。
  逐字节读的代价与字节数成正比，而这块板子空闲时状态流就有 ~14 kB/s
  ⇒ **常驻一整核**。
- **CRC16 走 `binascii.crc_hqx`**（poly `0x1021` / init `0xFFFF`），不是纯 Python 双循环 —— **232×** 快。

实测收益（裸 SDK 会话空闲 CPU）：逐字节 + 纯 Python CRC **98.8%**
→ 有多少读多少 **21.4%** → CRC 换 C 实现 **6.8%**。

### 文件

```text
src/litearm/
  _protocol.py     帧编解码 (0xA5 CMD LEN PAYLOAD CRC16-CCITT-FALSE) + 命令/应答常量
                   + 状态帧解析 + 开机签名解析 + COMMAND_COVERAGE 覆盖契约
                   ⚠ 授权那两条也在这里 (0x2F/0x3F/0x4F/0x50); 没有算 MAC 的代码
  transport.py     pyserial CDC 读写/自动发现 (VID:PID 1d50:606f);
                   读写各一把锁 (zero_g 保活线程是第一个并发写入者)
  state.py         RobotState / JointState
  errors.py        错误层级 + ERR_TEXT 码表
  arm.py           Arm 核心 + CLI + 读线程 `_Ack` + fork 守卫
  cart.py          笛卡尔: 0x4E 配对 + 到位判据 (bit10) + 能力探测
                   ⚠ PC 侧不做规划 —— 规划在固件里
  params.py        arm.params.*  关节级参数
  model.py         arm.model.*   动力学模型在线导入
  log.py           arm.log.*     300Hz 采集 + LogReader 游标读回
  diagnostics.py   arm.diag.*    KIN_BENCH 自检
  _rot.py          旋转/位姿纯数学 (rpy⇄矩阵, as_pose 归一化)
  testing.py       公开的离线桩 (FakeTransport) —— 按固件真实布局造帧与应答
```

### 覆盖契约

固件 `hal/usb_cmd.h` 里每条**已实现**的下行命令都有对应入口。
契约落在 `_protocol.COMMAND_COVERAGE`（命令 id → SDK 入口），
由 `tests/test_protocol_sync.py` **直接解析固件头文件**双向强制 ——
固件加了命令而 SDK 没跟上、或状态帧布局被改动，测试立刻失败。

---

## 8. 命令行

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

⚠ `movej` 要求 `TARGETS` 个数**正好等于 `arm.n`**；`home` 的 `--speed` **被忽略**
（固件写死 0.10）。

---

## 9. 测试

```bash
pytest                         # 离线全流程（桩 transport，不碰真机）
PYLITEARM_LIVE=1 pytest        # + 真机 live（需接 Litearm1.5.0+ 整臂/台架，会小幅运动）
python tests/test_offline.py   # 无 pytest 也可（脚本式断言）
```

**跑测试不需要装包** —— `tests/conftest.py` 会把 `src/` 与 `tests/` 自己塞进 `sys.path`。

⚠ **无人在场时不要设 `PYLITEARM_LIVE`**，也绝不调用 `enter_dfu()` / `reset_factory()`。

关键的几组：

| 测试 | 守什么 |
|---|---|
| `test_protocol_sync.py` | **协议漂移防护** —— 直接解析固件仓库的 `usb_cmd.h` / `usb_cmd.c` / `joint_cfg.h`，双向比对命令集合与 ID、钉死状态帧布局。固件仓库位置由 `LITEARM_FW_DIR` 指定；**找不到时 skip 而不是 pass** |
| `test_protocol_crc.py` | CRC16 —— 用**外部权威检查值**（`"123456789"` → `0x29B1`）+ 文件内独立参考实现，刻意不依赖被测实现 |
| `test_frame_ownership.py` | 帧归属契约：**帧没被静默销毁，它的主人拿到了它** |
| `test_transport.py` | 真实字节流解析（连续帧/噪声/坏帧/半帧/两种到达方式解出同样的帧） |
| `test_ack_echo.py` | ACK 必须回显原命令（迟到旧 ACK 不得让新命令"假成功"） |
| `test_capability.py` | `ERR{cmd,0x00}` → `UnsupportedByFirmwareError`（"真拒绝"不得误判） |
| `test_fork_guard.py` | fork 守卫：子进程零下发 |
| `test_zero_g.py` | 零重力保活周期/退出收尾/线程回收/命令门禁 |
| `test_status_layout.py` | `6+21N` 与 `4+21N` 两种状态帧布局 |
| `test_live.py` | 真机冒烟（`PYLITEARM_LIVE=1` 才跑） |

⚠ **`tests/fake_serial.py` 的桩必须与固件真实布局一致** ——
桩若与真固件不一致，离线会"假绿"而真机必挂。这个教训有专门测试守着。

---

## 10. 安全须知

1. **臂绝不能飞出去。** 这是产品的硬线，也是每一次安全改动的验收判据。
2. **`fork` 之后子进程不能用继承来的会话** —— fail-closed，零下发。父进程**必须先释放端口**。
3. **`movej` 返回 ≠ 停稳**（残差 ~0.012 rad，在 `q_tol` 内）。
4. **`movej` / `movej_sync` 目前不校验关节限位** —— 越限目标会被走满行程。发之前自己比对限位。
5. **`disable()` 之后臂不再被托住** —— 位置环一断，它会动。
6. **`save_params()` 没有撤销。**

### ⚠ 不可逆命令 —— 不要在标定过的臂上执行

这四条会**覆盖/抹掉该台设备逐台辨识的动力学模型**（整扇区擦除 + 写当前 RAM）：

| 命令 | 入口 |
|---|---|
| `0x25` | `save_params()` |
| `0x32` | `arm.model.commit()` |
| `0x36` | `arm.params.reset_factory()` |
| `0x37` | `arm.model.revert()` |

**唯一解锁方式：在一台没有标定价值的板子上做。**

⚠ **`model.set_jm()` 建议永不调用** —— 改错关节映射有**乱飞**风险，
而没有可依赖的退路。

⚠ 压测 CAN 链路**只开 `candump`（只读），绝不 `cangen`** —— `can0` 就是电机总线。

**明确未验证的部分**见 [TROUBLESHOOTING](../TROUBLESHOOTING.zh-CN.md#明确未验证的部分)。

## License

MIT
