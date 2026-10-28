# litearm-python 样例

litearm-python SDK 的可运行样例，一个脚本一项功能。每个都能独立运行，**先读懂再上真机**。

**默认只读**——会 `enable` / 运动 / 改参的样例必须显式加 `--go`，防误动。

## 前提

1. 已装本包（或让 `PYTHONPATH` 指向 `src`）；
2. 机械臂已连接，固件 `Litearm1.5.0` 及以上；
3. 串口可被独占——关掉其它占着 `/dev/ttyACM*` 的进程。

端口优先级：`--port` > 环境变量 `LITEARM_PORT` > 自动发现（`1d50:606f`）。

## 运行

```bash
source env.sh                       # 导出 PYTHONPATH / PYTHON_BIN / LITEARM_PORT

python3 examples/01_hello.py                    # 只读，不需要 --go
python3 examples/02_movej.py --go               # 会运动
./run_example.sh 02_movej.py --go               # 或用包装脚本一键跑
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

Windows 上 `LITEARM_PORT` 可以锁定 `COM5` 这类端口。

## 样例列表

| 样例 | 演示什么 | 需要 `--go` |
| --- | --- | --- |
| [01_hello.py](01_hello.py) | 连接握手 + 固件版本 + 读状态与末端位姿 | 否 |
| [02_movej.py](02_movej.py) | `movej` 单发：固件规划并走完，到位后静止保持 | 是 |
| [03_move_p.py](03_move_p.py) | `move_p` 单个位姿：固件内置逆解 + S 曲线，按 TCP 判到位 | 是 |
| [04_ik_tcp.py](04_ik_tcp.py) | `ik(pose)` 反解 + `get_tcp()` 读当前位姿（自洽校验） | 否 |
| [06_cartesian.py](06_cartesian.py) | 笛卡尔路径：`move_l` / `move_c` / `move_path` | 是 |
| [07_vel_jitter_trace.py](07_vel_jitter_trace.py) | 慢速 `movej` 的逐拍采集（300 Hz 固件日志 + 100 Hz 实测流） | 是 |
| [08_tcp_offset.py](08_tcp_offset.py) | 多点法最小二乘量工具 TCP 偏移（纯 `math`），量完交给 `set_tcp_offset` | 否（不发运动；`--go` 开始采点） |
| [09_payload_identify.py](09_payload_identify.py) | 对着空载基线辨识末端负载的质量与质心，然后 `set_payload` | 是（且需要一份基线 JSON） |

每个脚本的 docstring 都写明了运行方式与安全注意事项。

## 安全提示

会动臂或改参数的样例（02 / 03 / 06 / 07 / 09）**会真实改变机械臂状态**：

- 首次运行把 `--speed` 保持在 0.1~0.3；
- 人站在急停旁，确保周围无人无障碍；
- 跑之前先读 [排障指南](../TROUBLESHOOTING.zh-CN.md)；
- **`movej` 不校验关节限位**——越限目标会被走满行程。

会动的样例（02 / 03 / 06 / 07 / 09）收尾**刻意不 disable**：结束时保持使能持位，
需要手动失能时先托住臂或走到低位。只读预览（不带 `--go`）不写任何参数，也不改电机状态。

[08_tcp_offset.py](08_tcp_offset.py) 是例外：它不发运动、不上力，需要臂**能被手拖动**
（已上力且保持中就先 `arm.disable()`）。

[09_payload_identify.py](09_payload_identify.py) 既会移动机械臂、**也会写固件参数**。第一次
跑之前先看清基线里有几个构型：10 个构型就是 10 段 `movej`。

## 位姿格式

位姿是 **6 个数**：位置 3（m）+ 姿态 3（rad，RPY）。list 和 tuple 都收，不需要 numpy。

```python
m = arm.get_tcp()        # Msg 信封
p = m.value              # 6 个数，取不到帧时为 None

p[:3]                    # 位置，单位 m
p[3:6]                   # 姿态 RPY，单位 rad
```
