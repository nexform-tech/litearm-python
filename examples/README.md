# litearm-python examples

Each script runs standalone. **Read it before you point it at real hardware.**

**Read-only by default** — anything that enables, moves, or writes parameters
requires an explicit `--go`, so a stray run cannot move the arm.

## Prerequisites

1. This package is installed (or `PYTHONPATH=src`).
2. The arm is connected, firmware `Litearm1.5.0+`.
3. The serial port is free — close anything else holding `/dev/ttyACM*`.

Port priority: `--port` > `LITEARM_PORT` env var > auto-discovery (`1d50:606f`).

```bash
source env.sh                       # exports PYTHONPATH/PYTHON_BIN/LITEARM_PORT
```

On Windows use `env.ps1` / `env.cmd`; `LITEARM_PORT` can pin e.g. `COM5`.

## Running

```bash
python3 examples/01_hello.py                    # read-only, no --go needed
python3 examples/02_movej.py --go               # moves the arm
./run_example.sh 02_movej.py --go               # or one-shot wrapper
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

## The examples

| Example | Shows | Needs `--go` |
|---|---|---|
| [01_hello.py](01_hello.py) | connect handshake + firmware convention + state / TCP pose | no |
| [02_movej.py](02_movej.py) | `movej` single-shot → firmware S-curve + hold at target | yes |
| [03_move_p.py](03_move_p.py) | `move_p` single pose → firmware IK + S-curve (TCP arrival) | yes |
| [04_ik_tcp.py](04_ik_tcp.py) | `ik(pose)` solve + `get_tcp()` current pose (self-consistency) | no |
| [05_ff_tune.py](05_ff_tune.py) | dynamics / control-law tuning (`ff_preset`, gravity, inertia, payload, save) | yes |
| [06_cartesian.py](06_cartesian.py) | Cartesian paths `move_l` / `move_c` / `move_path` (firmware-planned, `CartPlan`) | yes |
| [07_vel_jitter_trace.py](07_vel_jitter_trace.py) | per-tick capture of a slow `movej` (300 Hz firmware log + 100 Hz live stream) | yes |

Each script's docstring carries its own run and safety notes.

## ⚠ Safety

The moving examples (02 / 03 / 05 / 06 / 07) **drive the real arm**:

- keep `speed` at 0.1~0.3 the first time
- stand at the e-stop, keep the workspace clear
- read [TROUBLESHOOTING.md](../TROUBLESHOOTING.md) first
- ⚠ **`movej` does not currently check joint limits** — an out-of-range target
  is driven the full way

## About 06_cartesian.py

This is the **Cartesian path** entry point: `move_l` goes straight, `move_c` goes
around an arc, `move_path` visits waypoints in turn (**sharp corners** — the
protocol has no blend field). Planning (sampling / per-point IK / playback) all
happens **in the firmware**: the PC sends points and collects the `0x4E` result
frame, returned as a `CartPlan`.

Known degradations (no corner blending / no pre-send preview / speed pre-check
delegated to firmware) are in the script docstring and in the
[Developer Guide](../docs/DEVELOPER_GUIDE.md#53-cartesian-firmware-planned).
For kinematics and impedance identification, see `pylitearm`'s own `examples/`.

## Pose format

A pose is a plain Python list of **6 numbers**: position 3 + RPY 3. No numpy.

```python
pose = [px, py, pz, rx, ry, rz]

m = arm.get_tcp()        # Msg envelope
p = m.value              # 6 numbers (or None)
p[:3]                    # position, m
p[3:6]                   # RPY, rad
```
