# litearm-python

LiteArm 机械臂的 Python SDK。通过 USB 串口直连机械臂固件，即可从任意机器控制机械臂 ——
不需要服务端或中间件。轨迹规划、运动学、动力学全部由固件承担。

> 📖 完整接口说明见 [docs/DEVELOPER_GUIDE.zh-CN.md](docs/DEVELOPER_GUIDE.zh-CN.md)。

## 特性

- 🐍 **纯 Python**：Python 3.9+，位姿就是普通的 list / tuple，不需要 numpy
- 📦 **单一依赖**：只有 `pyserial`
- 🔌 **USB 直连**：一根线接到固件，不经过任何服务端
- ⚙️ **重活交给固件**：规划、运动学、动力学都在固件里，PC 侧只发点、判到位
- 🦾 **完整运动接口**：关节运动、笛卡尔直线 / 圆弧 / 多路点、拖动示教
- 🛡️ **安全兜底**：子进程 fail-closed，急停独立入口，不可逆命令逐个标注

## 安装

| 项目 | 要求 |
| --- | --- |
| Python | 3.9 及以上 |
| 依赖 | `pyserial >= 3.4`（唯一依赖） |
| 固件 | `Litearm1.5.0` 及以上 |
| 连接 | USB CDC 串口，VID:PID `1d50:606f` |

```bash
pip install -e .
python3 -c "import litearm; print(litearm.__version__)"    # 验证
```

Linux 需要串口权限：

```bash
sudo usermod -aG dialout $USER      # 重新登录后生效
```

## 快速开始

```python
import litearm as pa

# 连接：自动找串口 + 校验固件版本
with pa.Arm().connect() as arm:
    print(arm.firmware, arm.n)                  # 版本串、关节数

    # 使能（运动之前必须）
    arm.enable()

    # 关节运动：固件规划 S 曲线并自己走完
    arm.movej([0.1, 0, 0, 0, 0, 0, 0], speed=0.3)

    # 笛卡尔直线运动：位姿 = 位置 3 个数（m）+ 姿态 3 个数（rad）
    arm.move_l((0.30, 0.0, 0.40, 3.1416, 0.0, 0.0), speed=0.5)

    # 读状态
    print(arm.get_state().value.q)              # 当前关节角
    print(arm.get_tcp().value)                  # 当前末端位姿

    # 逆解：位姿 → 关节角
    print(arm.ik((0.30, 0.0, 0.35, 3.1416, 0.0, 0.0)))

    # 回零
    arm.home()
# 退出 with 即断开
```

`Arm().connect()` 是唯一入口。每个会话背后有一条读线程，**用完必须 `close()`**，`with` 会自动关。
串口由 `port` 参数指定，不传则自动发现（VID:PID `1d50:606f`）。SDK **不读任何环境变量**。

下面各节的 `arm` 都指这个已连好的会话对象，片段只写该节要讲的那几步。

## API 参考

### 连接管理

```python
arm = pa.Arm(port="/dev/ttyACM0").connect()     # 不传 port 则自动发现
print(arm.firmware)                             # 版本串，如 "Litearm1.8.0-7J"
print(arm.n)                                    # 关节数
arm.close()                                     # 断开（用 with 会自动调）
```

### 关节运动

三个入口都要先 `enable()`：`movej` 单发（固件规划 S 曲线、自己走完），
`movej_sync` 同步点到点（各轴一起到位），`home` 回零。

```python
arm.enable()                                                        # 运动前必须先使能
arm.movej([0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], speed=0.3)          # 单发
arm.home()                                                          # 回零
```

### 笛卡尔运动

```python
P1 = (0.30, 0.0, 0.40, 3.1416, 0.0, 0.0)
P2 = (0.32, 0.0, 0.42, 3.1416, 0.0, 0.0)
P3 = (0.34, 0.0, 0.44, 3.1416, 0.0, 0.0)

arm.move_l(P2, speed=0.5)                       # 直线
arm.move_p(P2)                                  # 关节空间点到点 —— 末端不走直线
arm.move_path([P1, P2, P3], speed=0.5)          # 依次经过多个路点，转角是尖角
arm.move_c(arm.get_tcp().value, P2, P3)         # 圆弧 —— 起点必须是实测 TCP

plan = arm.move_l(P2, wait=False)               # 不阻塞，稍后查进度
print(arm.poll_cart())
arm.set_speed(50)                               # 全局调速：整数百分比 0..100
```

### 状态 / 运动学

```python
state = arm.get_state().value
print(state.q, state.dq, state.tau)             # 关节角 / 速度 / 力矩
print(state.enabled, state.faulted)             # 使能中？有故障？
print(arm.get_tcp().value)                      # 末端位姿（固件正运动学）
print(arm.ik((0.30, 0.0, 0.35, 3.1416, 0.0, 0.0)))   # 位姿 → 关节角
```

### 生命 / 安全

```python
arm.emergency_stop()                            # 急停：唯一没有前置条件的入口
arm.reset()                                     # 清故障 + 重锚控制环（不是 MCU 重启）
arm.clear_faults()                              # 只清 RAM 故障位
arm.disable()                                   # ⚠ 失能后不再被位置环托住
```

### 前馈 / 动力学调参

```python
arm.set_payload(1.0)                            # 换负载必须调，否则有恒定偏移
arm.set_gravity_scale([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
arm.ff_preset(1)                                # 0 全关 / 1 出厂 / 2 全开
print(arm.get_ff_scalar(4).value)               # 读回 payload_mass
```

### 拖动示教

```python
with arm.zero_g():
    input("拖动机械臂，然后回车")
```

### 透传 / 伺服

```python
arm.send_mit(0, 0.0, 0.0, 30.0, 1.0, 0.0)       # ⚠ 绕过规划，需 ≥10 Hz 自己保活
```

### 参数（`arm.params.*`）

```python
p = arm.params.get_joint_param(0).value         # J1 的 kp / kd / tau_max / 软限位
arm.params.set_joint_param(0, 30.0, 1.0, 20.0)  # idx, kp, kd, tau_max
for jp in arm.params.all_joint_params():
    print(jp.idx, jp.kp, jp.q_min, jp.q_max)
```

### 动力学模型（`arm.model.*`）

```python
print(arm.model.probe())                        # 固件是否支持
print(arm.model.status().value)                 # override / staged_mask / dirty
print(arm.model.get_gravity([0.0] * 7).value)   # G(q)
```

### 数据采集（`arm.log.*`）

```python
arm.log.start(300)                              # 开始录 300 拍后自停
r = arm.log.reader()
print(r.total())                                # 已落盘的拍数
for s in r.samples()[:3]:
    print(s.tick, s.q_ref, s.dq, s.tau)
```

### 固件自检（`arm.diag.*`）

```python
print(arm.diag.kin_bench().value)               # CAN 链路诊断计数
```

### 参数持久化

```python
arm.save_params()                               # ⚠ 写入 flash，不可逆
```

### 只读属性

```python
print(arm.n, arm.firmware, arm.fw_version)      # 关节数 / 版本串 / 版本元组
print(arm.q_tol, arm.dq_tol, arm.move_timeout)  # 到位判据与运动超时
print(arm.zero_g_active, arm.last_reset_reason)
```

## 命令行

```bash
litearm-python status                          # 只读
litearm-python fw                              # 版本串 + 轴数
litearm-python tcp                             # 当前位姿 + 帧率
litearm-python movej -0.1 0 0 0 0 0 0 --speed 0.3
litearm-python home
```

`status` / `fw` / `tcp` 只读，其余会真的驱动机械臂。等价写法是 `python -m litearm`。

## 注意事项

### 读值要走 `.value`

11 个"读一帧"接口返回的是信封 `Msg(value, hz, timestamp)` —— `.value` 才是数据本身，
`hz` / `timestamp` 是这类帧的到达频率与最近到达时刻。两者**同时为 0** 表示这类帧从没到过。

```python
print(arm.get_state())              # Msg(value=RobotState(...), hz=100.2, timestamp=...)
print(arm.get_state().value.q)      # [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

### 多进程：`fork` 之后子进程不能用继承来的 `Arm`

命令会**真的发出去**，但应答被父进程的读线程吃掉 —— 你只看到"无应答"超时，
一重试就是**重复下发**。读状态更隐蔽：它不报错，只是**永远返回陈旧值**。

所以本库 **fail-closed**：子进程里任何命令立刻抛 `ForkedSessionError`，一个字节都不下发。

子进程要用机械臂，**父进程必须先 `close()` 释放串口**，再 `fork`，然后在子进程里新建 `Arm`：

```python
import multiprocessing

def worker():
    a = pa.Arm().connect()        # 在子进程里新建

a = pa.Arm().connect()
a.close()                          # 不释放，子进程连不上
p = multiprocessing.Process(target=worker)
p.start()
```

子进程里 `close()` 仍可调（它只清会话状态、不碰传输层，不会挂死），但别指望靠它释放串口。

### 安全红线

1. **`disable()` 之后机械臂不再被位置环托住** —— 有负载就会往下垂。
2. **`movej` 不校验关节限位** —— 越限目标会被固件截断后**照走满行程**。
3. **`move_js` / `send_mit` 需 ≥10 Hz 自己保活** —— 否则 0.1 s 看门狗降刚度，臂缓慢塌下去。
4. **`enter_dfu()` 是终端态操作** —— 执行后所有入口失效，烧完固件要新建一个 `Arm`。

### 不可逆命令：不要在标定过的机械臂上执行

下列入口会覆盖或抹掉该台设备逐台辨识的动力学模型，**没有撤销**：

| 入口 | 作用 |
| --- | --- |
| `save_params()` | 当前 RAM 写入 flash |
| `arm.model.commit()` | 应用暂存的动力学模型改动 |
| `arm.model.revert()` | 回滚动力学模型（不动 flash，重新上电会复活） |
| `arm.params.reset_factory()` | 恢复出厂 |

只在一台没有标定价值的板子上做。**`arm.model.set_jm()` 建议永不调用** ——
改错关节映射有乱飞风险，而没有可依赖的退路。

压测 CAN 链路时只开 `candump`（只读），绝不 `cangen` —— `can0` 就是电机总线。

## 示例

见 [examples/README.zh-CN.md](examples/README.zh-CN.md)：

- `01_hello.py` — 连接握手 + 固件版本 + 读状态
- `02_movej.py` — 关节运动
- `03_move_p.py` — 笛卡尔点到点
- `04_ik_tcp.py` — 逆解与当前位姿
- `05_ff_tune.py` — 动力学 / 控制律调参
- `06_cartesian.py` — 笛卡尔直线 / 圆弧 / 多路点
- `07_vel_jitter_trace.py` — 300 Hz 逐拍采集

样例**默认只读**，会运动的必须加 `--go`：

```bash
source env.sh                       # 导出 PYTHONPATH / PYTHON_BIN / LITEARM_PORT
python3 examples/01_hello.py
./run_example.sh 02_movej.py --go
```

## 开发

```bash
pip install -e ".[dev]"
pytest                         # 离线全流程，不碰真机
LITEARM_LIVE=1 pytest          # 额外跑真机用例，会小幅运动
```

跑测试不需要装包，`tests/conftest.py` 会自己设好 `sys.path`。
无人在场时不要设 `LITEARM_LIVE`，也不要调用 `enter_dfu()` / `reset_factory()`。

Windows 用 `env.ps1` / `env.cmd` 与 `run_example.ps1` / `run_example.cmd`。

## License

MIT
