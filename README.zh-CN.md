# litearm-python —— LiteArm STM32 直连后端

LiteArm 机械臂的 Python SDK：**直连 `litearm-stm32` 固件**（USB CDC 串口），
用高层子集镜像 `pylitearm` 的常用用法。

这是一份**薄协议绑定** —— **PC 侧不做轨迹规划、不做运动学、不做动力学**，
这些全部由固件内置承担（B2 S 曲线 + B3 运动学 + B4 动力学 + B1 控制律）。
PC 侧只干三件事：编解码帧、下发命令、判定到位。

**不修改 `pylitearm` 源码。** 除 `pyserial` 外**零依赖**（不引 numpy / pinocchio）。

## 安装

```bash
pip install -e .            # 依赖仅 pyserial
```

## 快速开始

```python
import litearm as pa

arm = pa.Arm().connect()          # 自动找 CDC，并校验固件版本约定
arm.enable()                      # 运动前必须先使能
arm.movej([0.1, 0, 0, 0, 0, 0, 0], speed=0.3)   # 单发：固件 S 曲线自完成
print(arm.get_tcp().value)        # 当前末端 pos[3] + rpy[3]（2.0 起读值走 .value）
q = arm.ik((0.30, 0.0, 0.35, 3.1416, 0, 0))     # pose → 关节角（固件异步 IK）
arm.close()
```

`Arm().connect()` 是**唯一入口**。会话带一条后台读线程，所以每个 `Arm` 都要
`close()` —— 用 `with` 块可以省掉这件事：

```python
with pa.Arm().connect() as arm:
    print(arm.get_state().value.q)
```

## CLI 巡检

```bash
litearm-python status                          # 或 python -m litearm ...
litearm-python movej -0.1 0 0 0 0 0 0 --speed 0.3
litearm-python home
```

`status` / `fw` / `tcp` 只读；`enable` / `disable` / `reset` / `emergency` /
`movej` / `home` 会真的动。

## ⚠ 两条必读（会咬人的）

### 1. 多进程 / `fork()`：子进程**不能**用继承来的 `Arm`

本包每个会话带**一条后台读线程**（见[架构](docs/DEVELOPER_GUIDE.zh-CN.md#7-架构--一条读线程)）。线程**不被 `fork` 复制**，
而文件描述符会 —— 于是子进程里：命令**真的写出去**，应答却永远没人读，调用方只看到
"无应答"超时，重试就是**重复下发**；读状态更隐蔽，它**不报错**，只是静默回继承来的**陈旧**值。

故本包 **fail-closed**：子进程里任何命令立刻抛 `ForkedSessionError`
（`NotConnectedError` 的子类），**一个字节都不下发**。

**⚠ 子进程要用臂，前提是父进程先释放端口。** 串口是独占的，父进程还持有它时
子进程**连不上** —— 两端都会拦：进程内的端口登记表（随 `fork` 复制且仍指向父进程那个活传输），
以及 pyserial `exclusive=True` 在**继承来的 fd** 上的 `flock`。
⇒ 正路是**先 `close()` 父进程的会话，再 `fork`**，然后子进程里**新建**一个 `Arm`。
（"父进程不动，只在子进程里 `connect()`" 这条在真机上**走不通**。）

```python
# Linux 上 multiprocessing 默认就是 fork ⇒ 这条不是罕见路径
def worker():
    a = pa.Arm().connect()        # ✅ 在子进程里新建
    ...

a = pa.Arm().connect()
a.close()                          # ✅ 先释放端口 —— 漏了这步子进程连不上
p = multiprocessing.Process(target=worker)
p.start()                          # 别把 a 传进子进程用
```

⚠ 子进程里**不该调 `close()`**：那里要取一把从父进程继承来、永远不会释放的锁，碰了就
**永久挂死**。而且它在子进程里**并不保证安全**：轻路径自己也会取 `_tx_repeat_lock`。让那个继承来的 fd 随进程退出由内核关闭 —— 它既不读也不写，无害。

### 2. 未激活的板子：`enable()` 会被拒（`ERR{0x10,0x08}`）

固件 1.8.0 起**开机即锁**：未激活时 `ctrl_enable()` 的**第一条**判据就是"有没有被授权"，
**重发无用、无旁路**。**其余命令一切照常**（售后／产线要能诊断），`license()` 也正常返回
—— 见[授权／激活](#授权激活固件-180)。

## 固件版本约定

`firmware` 返回 `Litearm<主.次.修>-{7J|1J}`（如 `Litearm1.8.0-7J`）。连接时校验：

| 固件 | 结果 |
| --- | --- |
| `Litearm1.5.x-7J` / `Litearm1.5.x-1J` 及以上 | ✅ 接受（最低 **1.5.0**） |
| `Litearm1.4.x-*` 或更早 | ❌ `FirmwareMismatchError` |
| `A1.x-*-USB`（旧命名） | ❌ 不符合约定（提示烧 `Litearm1.5.0+`） |

> 版本门卡在 1.5.0，但状态帧解析**同时兼容** `4+21N`（≤1.4.x）与 `6+21N`（≥1.5.0）
> 两种布局 —— 那段兼容分支只用于离线／历史帧解析（例如分析抓包），不会被 `connect()` 走到。

## 授权／激活（固件 1.8.0+）

固件在**独立 flash 扇区**（sector 6）存一条授权记录，**写一次永不擦**；
未激活时**只锁 `ENABLE`**，其余命令一切照常。

```python
lic = arm.license()                       # 0x2F → LicenseInfo
if not lic.activated:
    print(lic.state_name, lic.uid_hex)    # uid_hex 就是签发器要的那 24 位 hex
    arm.disable()                         # activate 要求先失能
    arm.activate(cust_id=<customer-id>, issued=<YYYYMMDD>,
                 mac=<厂商签发的 16 字节>)   # 0x3F
```

- `license()` → `LicenseInfo`，**未激活时不抛异常**（它是一种**状态**），
  而且**未激活也回 UID** —— 那是签发器的唯一来源，别改用 USB 序列号字符串。
- `activate(*, cust_id, issued, flags=0, mac)` —— `mac` 由厂商侧签发。
  **须先失能**，否则 `ERR{0x3F,0x04}`；本地预检 `mac` 长度与 `flags` 保留位，
  不合规的帧**不下发**。
- ⚠ **本包不含密钥，也不含任何算 MAC 的代码** —— 签发在厂商侧工具里。这是规格硬要求：
  客户侧只要有一份能算 MAC 的代码，这套机制就归零。
- ⚠⚠ `ERR{0x3F,0x02}` 是**聚合档**：固件把「已经激活过／MAC 不符／密钥非法／写失败」
  全折成同一个码 ⇒ **光看码会把一台其实已经解锁的机器报成失败**。
  本包在这一档**自动回读 `0x2F`**：设备确实 `state != 0` 就当成功返回，否则才抛。
- 擦除授权记录**只能走 SWD**（`pyocd erase -s 0x080C0000`）—— 固件**没有**擦除命令。

## API 一览

| 分组 | 入口 |
| --- | --- |
| 会话 | `connect` `close` `disconnect` `reconnect` `__enter__` |
| 生命／安全 | `enable` `disable` `emergency_stop` `reset` `clear_faults` `set_motion_mode` `park` |
| 关节运动 | `movej` `movej_sync` `move_js` `home` |
| 笛卡尔 | `move_p` `move_l` `move_c` `move_path` `poll_cart` `set_speed` |
| 状态／运动学 | `get_state` `get_status_now` `get_tcp` `ik` |
| 前馈／动力学 | `set_ff_mask` `ff_preset` `set_ff_vec` `set_ff_scalar` `get_ff_vec` `get_ff_scalar` `get_ff_mask` `set_gravity_scale` `set_inertia_scale` `set_payload` `set_gravity_vector` |
| 透传／伺服 | `send_mit` `send_mit_all` |
| 零重力拖动示教 | `zero_g`（上下文管理器） `zero_g_start` `zero_g_stop` |
| 授权 | `license` `activate` |
| 烧录 | `enter_dfu` |
| 持久化 | `save_params` |
| 子对象 | `arm.params.*`（4） · `arm.model.*`（9） · `arm.log.*`（4 + `LogReader`） · `arm.diag.kin_bench` |
| 只读属性 | `n` `firmware` `fw_version` `min_firmware` `q_tol` `dq_tol` `arrive_frames` `move_timeout` `bench_model_axis` `last_reset_reason` `zero_g_active` `zero_g_error` |

完整签名、返回类型与逐条注意事项见 [docs/DEVELOPER_GUIDE.zh-CN.md](docs/DEVELOPER_GUIDE.zh-CN.md)。

### ⚠ 危险入口

先读警告，再看签名。

**`save_params()` —— 持久化到 flash（`0x25`）。**
写的是**当前 RAM**，没有撤销。

**`reset_factory()` —— 恢复出厂并失效 flash（`0x36`）。**
**固件要求失能态**，已使能时回 `ERR{0x36,0x04}`。

**`enter_dfu()` —— 唯一的终端态操作。**
两段式（`ACK{0x15}` 只表示"已登记"，还要等设备真的从 CDC 上消失）；使能中本地拒绝
（跳转会停 TIM3 ⇒ 电机 100 ms 松开、有负载则下垂）。成功返回后**本 `Arm` 不可再用**
（所有入口抛 `ArmIsInDfuError`，`close()` 例外），设备重枚举成 `0483:DF11`，
烧完固件**新建一个 `Arm`**。

**`send_mit` / `move_js` —— 绕过规划，且需调用方自己保活。**
**需 ≥10 Hz 重发**，否则 0.1 s 命令看门狗 fail-soft。

**`disable()` —— 使能一断，臂不再被位置环托住。**

## 文档

- [docs/DEVELOPER_GUIDE.zh-CN.md](docs/DEVELOPER_GUIDE.zh-CN.md) —— 完整 API 参考、返回信封、架构
- [TROUBLESHOOTING.zh-CN.md](TROUBLESHOOTING.zh-CN.md) —— 现场笔记：容易误诊的失效模式
- [examples/README.zh-CN.md](examples/README.zh-CN.md) —— 可运行样例（默认只读，运动需 `--go`）

## 开发

```bash
pip install -e ".[dev]"
pytest                         # 离线全流程（桩 transport，不碰真机）
PYLITEARM_LIVE=1 pytest        # + 真机 live（需接 Litearm1.5.0+ 整臂/台架，会小幅运动）
```

⚠ **无人在场时不要设 `PYLITEARM_LIVE`**，也绝不调用 `enter_dfu()` / `reset_factory()`。

样例先加载环境：

```bash
source env.sh                       # 导出 PYTHONPATH/PYTHON_BIN/LITEARM_PORT
python3 examples/01_hello.py
./run_example.sh 02_movej.py --go   # 或包装脚本一键跑
```

Windows 用 `env.ps1` / `env.cmd` 与 `run_example.ps1` / `run_example.cmd`，
`LITEARM_PORT` 可锁 `COM5` 等。

## License

MIT
