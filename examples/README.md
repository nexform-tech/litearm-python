# litearm-python examples

Each script runs on its own. **Read it before you run it on real hardware.**

**Read-only by default** — any example that enables, moves or retunes parameters requires an
explicit `--go`, so nothing moves by accident.

## Prerequisites

1. This package installed (or `PYTHONPATH` pointing at `src`);
2. The arm connected, firmware `Litearm1.5.0` or later;
3. The serial port available for exclusive use — shut down anything else holding
   `/dev/ttyACM*`.

Port priority: `--port` > the `LITEARM_PORT` environment variable > auto-discovery
(`1d50:606f`).

## Running

```bash
source env.sh                       # exports PYTHONPATH / PYTHON_BIN / LITEARM_PORT

python3 examples/01_hello.py                    # read-only, no --go needed
python3 examples/02_movej.py --go               # moves the arm
./run_example.sh 02_movej.py --go               # or one-shot via the wrapper
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

On Windows, `LITEARM_PORT` pins a port such as `COM5`.

## The examples

| Example | What it shows | Needs `--go` |
| --- | --- | --- |
| [01_hello.py](01_hello.py) | Handshake + firmware version + reading state and tool pose | No |
| [02_movej.py](02_movej.py) | `movej` as a single shot: firmware plans and completes it, then holds position | Yes |
| [03_move_p.py](03_move_p.py) | `move_p` with one pose: firmware IK + S-curve, arrival judged by TCP | Yes |
| [04_ik_tcp.py](04_ik_tcp.py) | `ik(pose)` plus `get_tcp()` for a self-consistency check | No |
| [05_ff_tune.py](05_ff_tune.py) | Dynamics / control-law tuning (`ff_preset` / gravity / inertia / payload / persist) | Yes |
| [06_cartesian.py](06_cartesian.py) | Cartesian paths: `move_l` / `move_c` / `move_path` | Yes |
| [07_vel_jitter_trace.py](07_vel_jitter_trace.py) | Per-tick capture of a slow `movej` (300 Hz firmware log + 100 Hz live stream) | Yes |

Every script's docstring states how to run it and what to watch out for.

## Safety notes

The examples that move the arm (02 / 03 / 05 / 06 / 07) **really drive it**:

- keep `--speed` at 0.1–0.3 the first time;
- stand by the emergency stop, and make sure nobody and nothing is in the workspace;
- read the [troubleshooting guide](../TROUBLESHOOTING.md) first;
- ⚠ **`movej` does not check joint limits** — an out-of-range target travels the full stroke.

## Pose format

A pose is **6 numbers**: 3 of position (m) plus 3 of orientation (rad, RPY). Lists and tuples
both work, and no numpy is needed.

```python
m = arm.get_tcp()        # Msg envelope
p = m.value              # 6 numbers, or None if no frame could be obtained

p[:3]                    # position, in metres
p[3:6]                   # orientation RPY, in radians
```
