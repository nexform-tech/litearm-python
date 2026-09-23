# litearm-python

Python SDK for the LiteArm robotic arm. Connect over USB serial straight to the arm's firmware
and control it from any machine — no server or middleware in between. Trajectory planning,
kinematics and dynamics are all carried by the firmware.

> 📖 Full interface reference: [docs/DEVELOPER_GUIDE.md](docs/DEVELOPER_GUIDE.md).

## Features

- 🐍 **Pure Python**: Python 3.9+, poses are plain lists / tuples, no numpy needed
- 📦 **A single dependency**: `pyserial`, nothing else
- 🔌 **Direct over USB**: one cable to the firmware, no server in between
- ⚙️ **The firmware does the heavy lifting**: planning, kinematics and dynamics live there; the
  PC side just sends points and decides arrival
- 🦾 **Full motion API**: joint moves, Cartesian lines / arcs / waypoints, hand-guiding
- 🛡️ **Safety built in**: fail-closed in child processes, a dedicated emergency stop, and every
  irreversible command flagged

## Install

| Item | Requirement |
| --- | --- |
| Python | 3.9 or later |
| Dependencies | `pyserial >= 3.4` (the only one) |
| Firmware | `Litearm1.5.0` or later |
| Connection | USB CDC serial, VID:PID `1d50:606f` |

```bash
pip install -e .
python3 -c "import litearm; print(litearm.__version__)"    # verify
```

Linux needs serial permissions:

```bash
sudo usermod -aG dialout $USER      # log in again for it to take effect
```

## Quick start

```python
import litearm as pa

# connect: find the port and check the firmware version
with pa.Arm().connect() as arm:
    print(arm.firmware, arm.n)                  # version string, joint count

    # enable (required before any motion)
    arm.enable()

    # joint move: the firmware plans the S-curve and completes it
    arm.movej([0.1, 0, 0, 0, 0, 0, 0], speed=0.3)

    # Cartesian line: a pose is 3 position values (m) + 3 orientation values (rad)
    arm.move_l((0.30, 0.0, 0.40, 3.1416, 0.0, 0.0), speed=0.5)

    # read state
    print(arm.get_state().value.q)              # current joint angles
    print(arm.get_tcp().value)                  # current tool pose

    # inverse kinematics: pose → joint angles
    print(arm.ik((0.30, 0.0, 0.35, 3.1416, 0.0, 0.0)))

    # go home
    arm.home()
# leaving the with block disconnects
```

`Arm().connect()` is the only entry point. Every session carries a read thread, so **you must
`close()` it** — `with` does that for you. Pass `port` to pick the serial port; leave it out and
the SDK auto-discovers it (VID:PID `1d50:606f`). The SDK **reads no environment variables**.

Every `arm` below refers to that connected session object; the snippets show only the steps
that section is about.

## API reference

### Connection

```python
arm = pa.Arm(port="/dev/ttyACM0").connect()     # omit port to auto-discover
print(arm.firmware)                             # version string, e.g. "Litearm1.8.0-7J"
print(arm.n)                                    # joint count
arm.close()                                     # disconnect (a with block does this for you)
```

### Joint motion

All three need `enable()` first: `movej` is a single shot (the firmware plans the S-curve and
completes it), `movej_sync` is a synchronised point to point, and `home` goes home.

```python
arm.enable()                                                        # required before motion
arm.movej([0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], speed=0.3)          # single shot
arm.home()                                                          # go home
```

### Cartesian motion

```python
P1 = (0.30, 0.0, 0.40, 3.1416, 0.0, 0.0)
P2 = (0.32, 0.0, 0.42, 3.1416, 0.0, 0.0)
P3 = (0.34, 0.0, 0.44, 3.1416, 0.0, 0.0)

arm.move_l(P2, speed=0.5)                       # straight line
arm.move_p(P2)                                  # joint-space point to point — not a line
arm.move_path([P1, P2, P3], speed=0.5)          # visit the waypoints in turn, sharp corners
arm.move_c(arm.get_tcp().value, P2, P3)         # arc — start must be the measured TCP

plan = arm.move_l(P2, wait=False)               # do not block; poll later
print(arm.poll_cart())
arm.set_speed(50)                               # global governor: integer percentage 0..100
```

### State / kinematics

```python
state = arm.get_state().value
print(state.q, state.dq, state.tau)             # joint angles / velocity / torque
print(state.enabled, state.faulted)             # enabled? faulted?
print(arm.get_tcp().value)                      # tool pose (firmware forward kinematics)
print(arm.ik((0.30, 0.0, 0.35, 3.1416, 0.0, 0.0)))   # pose → joint angles
```

### Life / safety

```python
arm.emergency_stop()                            # emergency stop: no preconditions at all
arm.reset()                                     # clear faults + re-anchor (not an MCU reboot)
arm.clear_faults()                              # clears RAM fault bits only
arm.disable()                                   # ⚠ the arm is no longer held once disabled
```

### Feed-forward / dynamics

```python
arm.set_payload(1.0)                            # always call this after changing payload
arm.set_gravity_scale([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
arm.ff_preset(1)                                # 0 all off / 1 factory / 2 all on
print(arm.get_ff_scalar(4).value)               # read payload_mass back
```

### Hand-guiding

```python
with arm.zero_g():
    input("Drag the arm, then press Enter")
```

### Passthrough / servo

```python
arm.send_mit(0, 0.0, 0.0, 30.0, 1.0, 0.0)       # ⚠ bypasses planning; keep alive at ≥10 Hz
```

### Parameters (`arm.params.*`)

```python
p = arm.params.get_joint_param(0).value         # J1's kp / kd / tau_max / soft limits
arm.params.set_joint_param(0, 30.0, 1.0, 20.0)  # idx, kp, kd, tau_max
for jp in arm.params.all_joint_params():
    print(jp.idx, jp.kp, jp.q_min, jp.q_max)
```

### Dynamics model (`arm.model.*`)

```python
print(arm.model.probe())                        # does the firmware support this?
print(arm.model.status().value)                 # override / staged_mask / dirty
print(arm.model.get_gravity([0.0] * 7).value)   # G(q)
```

### Data capture (`arm.log.*`)

```python
arm.log.start(300)                              # record 300 ticks, then stop
r = arm.log.reader()
print(r.total())                                # ticks written so far
for s in r.samples()[:3]:
    print(s.tick, s.q_ref, s.dq, s.tau)
```

### Firmware self-test (`arm.diag.*`)

```python
print(arm.diag.kin_bench().value)               # CAN link diagnostic counters
```

### Persistence

```python
arm.save_params()                               # ⚠ writes flash, irreversible
```

### Read-only properties

```python
print(arm.n, arm.firmware, arm.fw_version)      # joint count / version string / version tuple
print(arm.q_tol, arm.dq_tol, arm.move_timeout)  # arrival criteria and motion timeout
print(arm.zero_g_active, arm.last_reset_reason)
```

## Command line

```bash
litearm-python status                          # read-only
litearm-python fw                              # version string + axis count
litearm-python tcp                             # current pose + frame rate
litearm-python movej -0.1 0 0 0 0 0 0 --speed 0.3
litearm-python home
```

`status` / `fw` / `tcp` are read-only; the rest really drive the arm. `python -m litearm` is
equivalent.

## Things to watch out for

### Read the payload via `.value`

The 11 "read one frame" getters return the envelope `Msg(value, hz, timestamp)` — `.value` is
the payload itself, while `hz` / `timestamp` are how often frames of that kind arrive and when
the last one landed. If **both are 0**, no frame of that kind has ever arrived.

```python
print(arm.get_state())              # Msg(value=RobotState(...), hz=100.2, timestamp=...)
print(arm.get_state().value.q)      # [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

### Multiprocessing: a forked child must not use an inherited `Arm`

Commands **really go out on the wire**, but the parent's read thread consumes the replies — you
only see a "no response" timeout, and retrying means **sending the command twice**. Reading
state is subtler: it does not raise, it just **returns an eternally stale value**.

So this library is **fail-closed**: any command in a child process immediately raises
`ForkedSessionError`, and not a single byte goes out.

For a child to use the arm, the **parent must `close()` and release the port first**, then fork,
then create a new `Arm` inside the child:

```python
import multiprocessing

def worker():
    a = pa.Arm().connect()        # create it inside the child

a = pa.Arm().connect()
a.close()                          # without this the child cannot connect
p = multiprocessing.Process(target=worker)
p.start()
```

`close()` **is still callable** in the child (it only clears session state and never touches the
transport, so it does not hang) — but do not expect it to free the serial port.

### Safety rules

1. **After `disable()` the arm is no longer held by the position loop** — under load it sags.
2. **`movej` does not check joint limits** — an out-of-range target is clamped by the firmware
   and the arm **still travels the full stroke**.
3. **`move_js` / `send_mit` need keep-alive at ≥10 Hz** — otherwise the 0.1 s watchdog drops
   stiffness and the arm sags slowly.
4. **`enter_dfu()` is a terminal-state operation** — afterwards every entry point stops working
   and you need a new `Arm` after flashing.

### Irreversible commands: do not run these on a calibrated arm

These overwrite or erase that unit's per-arm identified dynamics model, with **no undo**:

| Entry point | Effect |
| --- | --- |
| `save_params()` | writes the current RAM to flash |
| `arm.model.commit()` | applies staged dynamics model changes |
| `arm.model.revert()` | rolls the dynamics model back (does not touch flash, so it returns on power-up) |
| `arm.params.reset_factory()` | restores factory settings |

Only do this on a board whose calibration has no value. **`arm.model.set_jm()` should never be
called** — a wrong joint mapping can make the arm flail, and there is no dependable way back.

When stress-testing the CAN link, run `candump` (read-only) only, never `cangen`: `can0` *is*
the motor bus.

## Examples

See [examples/README.md](examples/README.md):

- `01_hello.py` — handshake + firmware version + reading state
- `02_movej.py` — joint motion
- `03_move_p.py` — Cartesian point to point
- `04_ik_tcp.py` — inverse kinematics and the current pose
- `05_ff_tune.py` — dynamics / control-law tuning
- `06_cartesian.py` — Cartesian lines / arcs / waypoints
- `07_vel_jitter_trace.py` — 300 Hz per-tick capture

The examples are **read-only by default**; anything that moves needs `--go`:

```bash
source env.sh                       # exports PYTHONPATH / PYTHON_BIN / LITEARM_PORT
python3 examples/01_hello.py
./run_example.sh 02_movej.py --go
```

## Development

```bash
pip install -e ".[dev]"
pytest                         # full offline run, never touches hardware
LITEARM_LIVE=1 pytest          # plus hardware tests, moves a little
```

Running the tests does not require installing the package; `tests/conftest.py` sets `sys.path`
itself. Do not set `LITEARM_LIVE` with nobody present, and never call `enter_dfu()` /
`reset_factory()`.

On Windows use `env.ps1` / `env.cmd` and `run_example.ps1` / `run_example.cmd`.

## License

MIT
