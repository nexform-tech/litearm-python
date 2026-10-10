# litearm-python

LiteArm 机械臂的 Python SDK：从 PC 经一根 USB 串口线直连固件驱动机械臂 ——
不需要服务端或中间件。轨迹规划、运动学、动力学全部由固件承担。

> 完整接口说明见 [docs/DEVELOPER_GUIDE.zh-CN.md](docs/DEVELOPER_GUIDE.zh-CN.md)。

## 特性

- **纯 Python**：Python 3.9+，位姿就是普通的 list / tuple，不需要 numpy
- **单一依赖**：只有 `pyserial`
- **USB 直连**：一根线接到固件，不经过任何服务端
- **规划在固件里**：运动学、动力学都在固件里，PC 侧只发点、判到位
- **完整运动接口**：关节运动、笛卡尔直线 / 圆弧 / 多路点、拖动示教
- **安全兜底**：子进程 fail-closed，急停独立入口，不可逆命令逐个标注

## 安装

| 项目 | 要求 |
| --- | --- |
| Python | 3.9 及以上 |
| 依赖 | `pyserial >= 3.4`（唯一依赖） |
| 固件 | `Litearm1.5.0` 及以上 |
| 连接 | USB CDC 串口，VID:PID `1d50:606f` |

在仓库根目录执行：

```bash
pip install -e .
python3 -c "import litearm; print(litearm.__version__)"    # 验证
```

Linux 需要串口权限 —— 先看设备节点本身怎么说：

```bash
ls -l /dev/ttyACM0
```

是 `crw-rw-rw-` 就不必再加组，直接能开。是 `crw-rw----` 才需要把自己加进 `dialout`：

```bash
sudo usermod -aG dialout $USER      # 重新登录后生效
```

`crw-rw-rw-` 一般来自一条 udev 规则，而不是 `dialout` 组：

```
# /etc/udev/rules.d/99-litearm.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", MODE="0666"
```

这条规则**不属于任何软件包**，是装机时放进去的。`MODE="0666"` 意味着
**这台机器上任何本地用户都能驱动机械臂** —— 这是刻意的选择，不是疏漏。

## 快速开始

```python
import litearm as pa

# 连接：自动找串口 + 校验固件版本
with pa.Arm().connect() as arm:
    print(arm.firmware, arm.n)                  # 版本串、关节数

    # 上电后第一次必须先 reset()：臂上电就停在 EMERGENCY 锁存，
    # 不解开它，enable() 一律被拒 ERR{0x10,0x06}
    arm.reset()

    # 使能（运动之前必须）
    arm.enable()

    # 关节运动：固件规划 S 曲线并自己走完
    # 起手摆到一个各关节都远离限位的位形：全伸直（q ≈ 0）是奇异位形，
    # 从那里出发的笛卡尔运动一条都规划不出来
    arm.movej([0.0, 0.4, -0.8, -1.2, 0.0, 0.6, 0.0], speed=0.3)

    # 笛卡尔直线运动：位姿 = 位置 3 个数（m）+ 姿态 3 个数（rad）
    # 拿刚读到的实测位姿当起点，只改位置、姿态照抄 —— 省得自己配一套姿态
    tcp = arm.get_tcp().value
    arm.move_l((tcp[0], tcp[1], tcp[2] - 0.05, *tcp[3:]), speed=0.5)

    # 读状态
    print(arm.get_state().value.q)              # 当前关节角
    print(arm.get_tcp().value)                  # 当前末端位姿

    # 逆解：位姿 → 关节角
    print(arm.ik((tcp[0], tcp[1], tcp[2] - 0.07, *tcp[3:])))

    # 回零
    arm.home()
# 退出 with 即断开
```

`reset()` 是上电后的前置步骤，不是可选项：臂一上电就处于 `EMERGENCY` 锁存
（`mode_name == "EMERGENCY"`、`flags` 含 `FAULT`、`joint_fault` 非 0），
不先解开它，`enable()` 会**立刻**抛 `ERR{0x10,0x06}`，一次重试都不发。
`reset()` 之后 `mode` 回到 `INIT`、故障位清零，整段快速开始即可跑通。

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

⚠ **别在 `enable()` 的下一行读 `get_state().enabled`。** 它在状态帧反映之前就返回——
实测返回耗时 2.0~2.7 ms，而那个标志变真要到 7.0~7.8 ms——所以读到的是 `False`，
那个值没有任何意义。判定前先等一拍。

### 笛卡尔运动

```python
# 在**实测**位姿上做偏移。可达性由臂的几何决定，不由请求决定，
# 写死的绝对位姿可能根本没有逆解。
p = arm.get_tcp().value
P1 = (p[0] + 0.02, p[1], p[2], p[3], p[4], p[5])
P2 = (p[0] + 0.04, p[1], p[2], p[3], p[4], p[5])
P3 = (p[0] + 0.06, p[1], p[2], p[3], p[4], p[5])

arm.move_l(P1, speed=0.3)                       # 直线
arm.move_p(P1)                                  # 关节空间点到点 —— 末端不走直线
arm.move_path([P1, P2, P3], speed=0.3)          # 依次经过多个路点，转角是尖角
arm.move_c(arm.get_tcp().value, P1, P2)         # 圆弧 —— 起点必须是实测 TCP

plan = arm.move_l(P2, wait=False)               # 仍会阻塞到规划应答到达
print(plan.settled)                             # False —— 我们没等臂停稳
arm.set_speed(50)                               # 全局调速：整数百分比 0..100
```

⚠ **机械臂开机停在零位，那是完全伸展的奇异位形。** 从那里发笛卡尔命令没有可用逆解，
会被拒成 `IKError`（`err=1`）。先弯一个关节
（`arm.movej([0.0, -0.5, 0.0, -1.0, 0.0, 0.0, 0.0], speed=0.3)`）—— 见「快速开始」。

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
arm.clear_faults()                              # 只清 FAULT 一位，清不掉锁存态
arm.disable()                                   # 失能后不再被位置环托住
```

`clear_faults()` **清不掉锁存态**：`FAULT` 位会被清掉，`EMERGENCY` 模式与
`FB_STALE` 立刻回来，臂仍然使能不了。这两种情况只有 `reset()` 能恢复 ——
`clear_faults()` 只清 `ARM_FLAG_FAULT` 这一位，使能门禁看的是 `reset()` 才清的
锁存判据。

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
arm.send_mit(0, 0.0, 0.0, 30.0, 1.0, 0.0)       # 绕过规划，需 ≥10 Hz 自己保活
arm.joint_follow([0.0]*7, [0.0]*7, [30.0]*7, [1.0]*7)   # 帧里没有 tau，前馈由固件算
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

### 授权与激活

未激活的臂 `enable()` 回 `ERR{0x10,0x08}`，其余命令一律照常，所以现场仍可诊断。
`arm.license()` 读授权记录（裸 `LicenseInfo`，不是 `Msg`）；`arm.activate()` 提交厂商签发的凭据，
须先 `disable()`。见[开发者指南 §5.15](docs/DEVELOPER_GUIDE.zh-CN.md#515-授权与激活)。

### 参数持久化

```python
arm.disable()                                   # 须先失能，使能中调用会被固件拒
arm.save_params()                               # 写入 flash，不可逆
arm.enable()
```

`save_params()` **要求先失能**：使能中（或 `enable` 在途）调用回 `ERR{0x25,0x04}`。
而 `disable()` 的那一刻臂不再被位置环托住 —— **侧装或带负载时先托住臂，或先走到低位**，
否则它会因自重落下。

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

`status` / `fw` / `tcp` 只读，其余会真的驱动机械臂。

⚠ **别指望照抄的 `litearm-python` 这个名字能解析得到**：控制台脚本装在
`~/.local/bin`，那个目录默认**不在 `PATH` 上**（见「安装」），照抄会报"未找到命令"。
用 `python3 -m litearm <子命令>`（等价写法），或把 `~/.local/bin` 加进 `PATH`。

## 注意事项

先看一份[现场排障](TROUBLESHOOTING.zh-CN.md)：串口被占用怎么定位占用方、
`enable()` 被拒的每个码怎么处置、笛卡尔报错怎么分辨，那里都有现成命令。

### 读值要走 `.value`

11 个"读一帧"接口返回的是信封 `Msg(value, hz, timestamp)` —— `.value` 才是数据本身，
`hz` / `timestamp` 是这类帧的到达频率与最近到达时刻。两者**同时为 0** 表示这类帧从没到过。

```python
print(arm.get_state())              # Msg(value=RobotState(...), hz=100.2, timestamp=...)
print(arm.get_state().value.q)      # [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

**"同时为 0"这条判据有例外**：进程内读到的**第一帧**，`hz` 按 0 算 ——
刚连上就 `print(arm.get_state())` 会看到 `hz=0.0`，但那帧是真的，几秒后同一个会话再读就是
`hz≈98.7`。要判断"这类帧有没有到过"，看 `timestamp`（或者隔一会儿再读一次），别看单次 `hz`。

### `mode` / `flags` 怎么读

`RobotState.mode_name` 是固件的运动模式，取值只有这 8 个：

| `mode` | `mode_name` | 含义 |
| --- | --- | --- |
| 0 | `INIT` | 启动 / 未使能。`reset()` 之后没有运动时就是它 |
| 1 | `MOVE_J` | 关节轨迹在途（`movej` / `home`） |
| 2 | `MOVE_P` | 笛卡尔点到点 |
| 3 | `MOVE_JS` | 关节透传 / 伺服在途 |
| 4 | `MOVE_MIT` | `send_mit` 在途 |
| 5 | `MIT_ALL` | `send_mit_all` 在途 |
| 6 | `EMERGENCY` | 急停锁存。**臂上电就停在这里**，见[快速开始](#快速开始) |
| 7 | `ZERO_G` | 零重力拖动示教中 |

表外的值直接显示成数字。真机上常见的是 `INIT` / `MOVE_J` / `EMERGENCY` / `ZERO_G`；
`MOVE_P` 在固件里标注为"B3 前由 PC 算 IK 转 `move_j`"，所以通常看不到它。

`flags` / `flag_names` 是安全位，取值 `FAULT` / `WD_TRIPPED` / `FB_STALE` /
`TEMP_WARN` / `POS_VIOL` / `OVERSPEED`。其中两个别读错：

- **`WD_TRIPPED` 是常态，不是故障。** 臂使能但空闲（`mode != MOVE_J`）时它一直亮着 ——
  那是固件的命令看门狗（0.1 s）在"没有运动命令续命"时的正常状态。
- **`FB_STALE` 是活条件**，只清故障位清不掉，见[生命 / 安全](#生命--安全)。

`mode == ZERO_G` 说的是**臂**在零重力里，由固件持有；`arm.zero_g_active` 说的是
**本进程**还在不在保活。两者可以不一致：进程被硬杀之后臂仍在 `ZERO_G`，
而新进程读到的 `zero_g_active` 是 `False`。

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

样例**默认只读**，会运动的必须加 `--go`。都在仓库根目录执行：

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

**本机装了 ROS 2 时 `pytest` 会在收集阶段崩掉，与本仓库无关**：pytest 按 entry point
自动加载 `/opt/ros/...` 里的插件，其中 `launch` 需要未安装的 `lark`：

```
ModuleNotFoundError: No module named 'lark'
```

venv 拦不住它 —— `PYTHONPATH` 的优先级高于 venv 的 `site-packages`。加下面任一个即可：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest
env -u PYTHONPATH pytest
```

`-p no:launch_testing` **不够**：`launch_ros` 那个入口照样会被加载。

Windows 用 `env.ps1` / `env.cmd` 与 `run_example.ps1` / `run_example.cmd`。

## License

MIT
