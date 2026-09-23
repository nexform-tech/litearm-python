# litearm-python 样例

每个脚本可独立运行，**先读懂再上真机**。

**默认只读** —— 会 `enable` / 运动 / 改参的样例必须显式加 `--go`，防误动。

## 前提

1. 已装本包（或 `PYTHONPATH=src`）。
2. 机械臂已连接，固件 `Litearm1.5.0+`。
3. 串口可被独占 —— 关掉其它占着 `/dev/ttyACM*` 的进程。

端口优先级：`--port` > 环境变量 `LITEARM_PORT` > 自动发现（`1d50:606f`）。

```bash
source env.sh                       # 导出 PYTHONPATH/PYTHON_BIN/LITEARM_PORT
```

Windows 用 `env.ps1` / `env.cmd`；`LITEARM_PORT` 可锁 `COM5` 等。

## 运行

```bash
python3 examples/01_hello.py                    # 只读，不需要 --go
python3 examples/02_movej.py --go               # 会运动
./run_example.sh 02_movej.py --go               # 或包装脚本一键跑
LITEARM_PORT=/dev/ttyACM0 ./run_example.sh 03_move_p.py --go
```

```powershell
# PowerShell
. .\env.ps1
python examples\01_hello.py
.\run_example.ps1 02_movej.py --go
```

```bat
rem cmd.exe
call env.cmd
python examples\01_hello.py
```

## 样例列表

| 样例 | 演示 | 需 `--go` |
|---|---|---|
| [01_hello.py](01_hello.py) | 连接握手 + 固件版本约定 + 状态/末端位姿 | 否 |
| [02_movej.py](02_movej.py) | `movej` 单发 → 固件 S 曲线自完成 + 静止保持 | 是 |
| [03_move_p.py](03_move_p.py) | `move_p` 单 pose → 固件内置 IK + S 曲线（TCP 到位判定） | 是 |
| [04_ik_tcp.py](04_ik_tcp.py) | `ik(pose)` 反解 + `get_tcp()` 当前位姿（自洽校验） | 否 |
| [05_ff_tune.py](05_ff_tune.py) | 内置动力学/控制律调参（`ff_preset` / 重力 / 惯量 / payload / save） | 是 |
| [06_cartesian.py](06_cartesian.py) | 笛卡尔路径 `move_l` / `move_c` / `move_path`（固件规划 + `CartPlan`） | 是 |
| [07_vel_jitter_trace.py](07_vel_jitter_trace.py) | 慢速 `movej` 的逐拍采集（300 Hz 固件日志 + 100 Hz 实测流双通道） | 是 |

每个脚本的 docstring 含运行与安全说明。

## ⚠ 安全提示

会运动的样例（02 / 03 / 05 / 06 / 07）**真实驱动机械臂**：

- 首次运行 `speed` 保持 0.1~0.3
- 人站在急停旁，确保周围无人无障碍
- 跑之前先读 [TROUBLESHOOTING.zh-CN.md](../TROUBLESHOOTING.zh-CN.md)
- ⚠ **`movej` 目前不校验关节限位** —— 越限目标会被走满行程

## 关于 06_cartesian.py

它是**笛卡尔路径**入口：`move_l` 走直线、`move_c` 走圆弧、`move_path` 依次经过多路点
（**尖角**，协议无倒角字段）。规划（采样 / 逐点 IK / 播放）全在**固件**里 ——
PC 只发点、收 `0x4E` 结果帧，返回 `CartPlan`。

已知降级（无倒角 / 无下发前预览 / 速度预检改由固件做）见脚本 docstring 与
[开发者指南](../docs/DEVELOPER_GUIDE.zh-CN.md#53-笛卡尔固件规划)。
运动学 / 阻抗辨识等其余高级场景仍参考 `pylitearm` 本体 `examples/`。

## 位姿格式

位姿是 **6 个数**的纯 Python list：位置 3 + RPY 3。不需要 numpy。

```python
pose = [px, py, pz, rx, ry, rz]

m = arm.get_tcp()        # Msg 信封
p = m.value              # 6 个数（或 None）
p[:3]                    # 位置，m
p[3:6]                   # RPY，rad
```
