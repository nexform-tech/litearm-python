# litearm-python developer guide

Python SDK for the LiteArm robotic arm, talking straight to the firmware over USB serial.

Planning, kinematics and dynamics live in the firmware; the PC side only encodes frames, sends
commands and decides arrival. A pose is 6 numbers (list or tuple), and `pyserial` is the only
dependency.

## Contents

1. [Requirements and install](#1-requirements-and-install)
2. [Quick start](#2-quick-start)
3. [Connection management](#3-connection-management)
4. [The read envelope `Msg`](#4-the-read-envelope-msg)
5. [API reference](#5-api-reference)
6. [Exceptions](#6-exceptions)
7. [Things to watch out for](#7-things-to-watch-out-for)
8. [Architecture](#8-architecture)
9. [Command line](#9-command-line)
10. [Testing](#10-testing)

---

## 1. Requirements and install

- Python **3.9 or later**
- `pyserial >= 3.4` (the only dependency)
- Firmware **`Litearm1.5.0` or later**, version string `Litearm<major.minor.patch>-{7J|1J}`
- Linux needs serial permissions: `sudo usermod -aG dialout $USER`, then log in again

```bash
pip install -e .            # install
pip install -e ".[dev]"     # development (includes pytest)
```

When `pip` and `python` point at different interpreters, use `python -m pip`.

---

## 2. Quick start

```python
import litearm as pa

arm = pa.Arm().connect()          # find the port, check the firmware version
arm.enable()                      # the arm must be enabled before it moves
arm.movej([0.1, 0, 0, 0, 0, 0, 0], speed=0.3)
print(arm.get_tcp().value)        # read the payload via .value, see §4
arm.close()
```

When `connect()` returns, the handshake is done and `arm.n` / `arm.firmware` are available.

Every session carries a read thread, so **you must `close()` it**; `with` does that for you:

```python
with pa.Arm().connect() as arm:
    print(arm.get_state().value.q)
```

Every `arm` in the sections below refers to this connected session object; the snippets show
only the steps that section is about.

A child process cannot inherit the session, see
[README](../README.md#multiprocessing-a-forked-child-must-not-use-an-inherited-arm).

---

## 3. Connection management

```python
Arm(port=None, *, transport_factory=None, min_firmware=MIN_FW,
    q_tol=0.03, dq_tol=0.10, arrive_frames=3, move_timeout=15.0)

connect(port=None) -> Arm
close() -> None
disconnect() -> None                 # alias for close()
reconnect(port=None) -> Arm          # close() then connect()
__enter__()                          # with support
__exit__(*exc)                       # returns False, never swallows
__del__()                            # last-resort cleanup
```

### Constructor arguments

| Argument | Default | Meaning |
| --- | --- | --- |
| `port` | `None` | Serial port path. `None` auto-discovers (VID:PID `1d50:606f`). The SDK reads **no** environment variables; `LITEARM_PORT` is used only by `examples/_common.py` |
| `transport_factory` | `None` | Inject a transport (tests), needs a placeholder `port` too |
| `min_firmware` | `(1, 5, 0)` | Lower bound of the version gate |
| `q_tol` | `0.03` | Arrival criterion: joint angle tolerance (rad) |
| `dq_tol` | `0.10` | Arrival criterion: joint velocity tolerance |
| `arrive_frames` | `3` | Arrival criterion: consecutive frames required |
| `move_timeout` | `15.0` | Motion timeout (s) |

### Session methods

| Method | Caveat |
| --- | --- |
| `connect()` | Idempotent. Any failing step **closes the link before raising**, so no half-open session is left behind |
| `close()` | Idempotent. Stops the read thread, then closes the transport. Every entry point afterwards raises `NotConnectedError` |
| `reconnect()` | Starts a new session: the read thread restarts and `Msg.hz` resets |

Module constants: `litearm.MIN_FW`, `litearm.FIRMWARE_PREFIX`.

### Firmware version convention

`firmware` returns `Litearm<major.minor.patch>-{7J|1J}` (for example `Litearm1.8.0-7J`).

| Firmware | Result |
| --- | --- |
| `Litearm1.5.x-*` or later | Accepted |
| `Litearm1.4.x-*` or earlier | `FirmwareMismatchError` |
| Anything else | `FirmwareMismatchError` |

---

## 4. The read envelope `Msg`

These 11 "read one frame" getters return `Msg[T]`:

| # | Entry point | Frame |
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
    value: T          # the payload (None if no frame could be obtained)
    hz: float         # average arrival rate of this frame kind in this session
    timestamp: float  # local time.monotonic() of the latest frame (0.0 if never seen)
```

`hz` is the average rate since the first frame of that kind arrived in this session, and it is
**`0.0` with fewer than 2 samples**.

- Passive streams (`RSP_STATUS`, 100 Hz): converge to about 100 after two or three frames; an
  idle link does not make it decay.
- The 4 request/response ones (`params.get_joint_param` / `model.get_body` / `model.get_jm` /
  `model.get_gravity`): one call yields one frame, so **the first call is always `hz == 0.0`**,
  and from the second call on it equals **your own polling rate**.
- ⚠ `diag.kin_bench()` is **not** in that group: its reply is **two consecutive frames** (a
  timing frame plus a LINK frame), so one call delivers 2 frames — `hz` is non-zero on the very
  first call, and that number is **meaningless** (the numerator is inflated by the split frame
  while the denominator is still the call interval). Use `timestamp` to tell whether a frame
  ever arrived.
- `reconnect()` resets it.

**`hz == 0.0` and `timestamp == 0.0` mean no frame of this kind has ever arrived**, not a slow
link.

### Entry points that do not return `Msg`

- `move_*` and `home()` — they return an action result (`RobotState` / `CartPlan`)
- `n` / `firmware` / `last_reset_reason` / `zero_g_active` — no frame at all
- `ik()` — a computation request
- `get_ff_mask()` returns a bare `int`; `params.all_joint_params()` returns `list[JointParam]`

### `RobotState`

```python
get_state(refresh=False, timeout=0.5) -> Msg[Optional[RobotState]]
get_status_now(timeout=0.5)           -> Msg[RobotState]
```

| Field | Meaning |
| --- | --- |
| `mode` / `mode_name` | Current mode |
| `flags` / `flag_names` | Flag bits and their names |
| `seq` | Arrival sequence number, for telling whether the stream is still moving |
| `joints` | `list[JointState]` |
| `joint_fault` | Per-axis drop-out bitmap |

Derived properties: `n`, `enabled`, `cart_busy`, `q`, `dq`, `tau`, `fault_axes`, `faulted`,
`fault_detail`, `drop_hold_inferred`.

`JointState`: `q`, `dq`, `tau`, `t_mos`, `t_coil`, `err`.

`get_status_now()` **actively sends a `GET_STATUS`**, unlike `get_state()` which only consumes
the passive stream — that makes it useful for confirming the link is alive.

⚠ `get_status_now(timeout=0.0)` is **not** a non-blocking poll; it means **return the current
cached value immediately**. It raises `MotionTimeoutError` if this session has not received a
single state frame yet.

---

## 5. API reference

A pose is 6 numbers: 3 of position (m) plus 3 of orientation (rad, RPY). Lists and tuples both
work.

```python
pose = [0.30, 0.0, 0.35, 3.1416, 0, 0]

arm.move_p(pose)
```

A "3 position values + 3×3 rotation matrix" pair is accepted too:

```python
arm.move_p(([0.30, 0.0, 0.35], [[1, 0, 0], [0, 1, 0], [0, 0, 1]]))
```

Return types are not uniform; each entry point's docstring states which one it uses:

| Return type | Entry points |
| --- | --- |
| `Msg[...]` | The 11 "read one frame" getters above |
| `RobotState` | `movej` `movej_sync` `move_p` `home` |
| `CartPlan` | `move_l` `move_c` `move_path` |
| `list[float]` | `ik` |

### 5.1 Life / safety

```python
enable(attempts=12)
disable()
emergency_stop()
reset()
clear_faults()
set_motion_mode(mode)
park()
```

| Method | Caveat |
| --- | --- |
| `enable(attempts=12)` | Enables all joints. Retries are whitelisted: **only `(0x10, 0x03)` is retried** |
| `disable()` | Cuts the position loop. The arm is no longer held |
| `emergency_stop()` | Single frame, one-way, does not read state — the only entry point with no preconditions |
| `reset()` | A software state reset, **not an MCU reboot**; the same object stays usable |
| `clear_faults()` | Clears RAM fault bits only, does not write flash |
| `set_motion_mode(mode)` | The firmware only accepts `0`; anything else raises `InvalidCommandError` locally |
| `park()` | Equivalent to `set_motion_mode(0)` |

### 5.2 Joint motion

```python
movej(q, speed=1.0) -> RobotState
movej_sync(q, speed=1.0) -> RobotState
move_js(q, dq=None, tau_ff=None) -> None
home(*, timeout=None) -> RobotState
```

| Method | Caveat |
| --- | --- |
| `movej(q, speed=1.0)` | Single shot: the firmware plans an S-curve, completes it and holds position. `speed` ∈ `0..1`. **Joint limits are not checked** — an out-of-range target is clamped and still travels the full stroke |
| `movej_sync(q, speed=1.0)` | Synchronised point to point, all axes arrive together |
| `move_js(q, dq=None, tau_ff=None)` | Low-level joint stream, bypasses planning. `dq` is a velocity reference, not a limit; the caller must resend at ≥10 Hz |
| `home(*, timeout=None)` | Go home. **Requires `enable()` first**, otherwise `ERR{0x2A,0x03}`. `timeout` is keyword-only; the firmware hard-codes the speed at 0.10 and rejects `speed`. The firmware **does** allow homing from a pose beyond the soft limits or against an end stop |

`home(speed=0.3)` raises `TypeError`; write `arm.home()` or `arm.home(timeout=30.0)`.

### 5.3 Cartesian motion

```python
move_p(pose, speed=1.0, pos_tol=0.006, rpy_tol=0.03) -> RobotState
move_l(pose, speed=1.0, wait=True) -> CartPlan
move_c(pose_start, pose_via, pose_goal, speed=1.0, wait=True) -> CartPlan
move_path(poses, speed=1.0, wait=True) -> CartPlan
poll_cart() -> Optional[CartPlan]
set_speed(percent)
```

All planning happens in the firmware; the PC sends points and receives a `0x4E` result frame.

| Method | What the tool travels along |
| --- | --- |
| `move_p(pose)` | Joint-space interpolation, point to point, **not a straight line** |
| `move_l(pose)` | A straight line (linear position + spherical orientation interpolation) |
| `move_c(start, via, goal)` | An arc (three points define the circle; the orientation of `via` is ignored) |
| `move_path(poses)` | Visits several waypoints in turn, **sharp corners** |

| Method | Caveat |
| --- | --- |
| `move_p` | Accepts a single pose only; a sequence raises `InvalidCommandError`. Arrival is judged by TCP tolerance |
| `move_l` / `move_c` / `move_path` | Return a `CartPlan`. With `wait=False` they do not block; poll with `poll_cart()` |
| `move_c` | `start` must match the measured TCP at call time (tolerance 6 mm / 0.03 rad); write `arm.get_tcp().value` |
| `poll_cart()` | Reads only the collector's unclaimed queue, does not touch the link |
| `set_speed(percent)` | A global, **persistent** governor. `percent` is an **integer percentage 0..100** — `set_speed(1)` means **1% speed**, not "full speed"; it is not the same thing as the per-trajectory factor in `movej(speed=0..1)` |

Capability boundaries: no corner blending, no pre-flight preview, and the speed pre-check exists
only inside the firmware.

Failures raise `CartesianPlanError` and reject the whole move — the arm does not budge:
`err=1` no IK solution / `err=2` three collinear points / `err=3` over capacity, unreachable.

### 5.4 Pose / kinematics

```python
get_tcp(timeout=0.6) -> Msg[Optional[tuple]]
ik(pose, q_seed=None, timeout=3.0) -> list[float]
```

| Method | Caveat |
| --- | --- |
| `get_tcp()` | The current tool pose (firmware forward kinematics), 6 numbers |
| `ik(pose, q_seed=None)` | Inverse kinematics. It may return another equally valid branch, so a result far from `q_seed` is not necessarily an error |

There is no `fk(q)`. The PC side carries no kinematic model, so forward kinematics is only
`get_tcp()`.

### 5.5 Feed-forward / dynamics tuning

```python
set_ff_mask(mask)
ff_preset(preset)                        # 0 / 1 / 2
set_ff_vec(item, values)                 # len(values) must equal n
set_ff_scalar(item, sub, value)
get_ff_vec(item, timeout=1.0) -> Msg[list[float]]
get_ff_scalar(item, sub=0, timeout=1.0) -> Msg[float]
get_ff_mask(timeout=1.0) -> int          # a bare int, not a Msg
set_gravity_scale(gs)                    # length = n
set_inertia_scale(isc)                   # length = n
set_payload(mass, com=(0.0, 0.0, 0.0))
set_gravity_vector(g)                    # length 3
```

`get_ff_mask()` returns a bare `int`; call `get_ff_scalar` if you want the envelope.

Items 12–15 of `set_ff_vec` have only the generic entry point: `12 zg_kp` / `13 zg_kd` /
`14 zg_damping` (used by hand-guiding) and `15 kd_extra` (a software derivative damping that
cures `movej` start-up ringing). **`kd_extra` must stay 0 for the wrist joints J5–J7** (factory
value `[6,6,6,6,0,0,0]`).

The item name tables are class attributes on `Arm` and double as the validation whitelist:
`FF_VEC_ITEMS` (1..15), `FF_SCALAR_ITEMS` (1..18, missing 9), `FF_SCALAR_RO_ITEMS` (only 9).

These write RAM; call `save_params()` to persist.

When writing through `set_ff_vec`, **any `NaN` component makes the firmware reject the whole
group** (`ERR{0x26,0x02}`); a magnitude outside the item's range is **silently clamped**
instead of raising.

### 5.6 Hand-guiding

```python
zero_g(period=0.04)              # context manager
zero_g_start(period=0.04)
zero_g_stop(raise_on_lost=False)
```

`zero_g()` is a client-side composition (`zero_g_start` + `zero_g_stop`), not a single command.

```python
with arm.zero_g():
    input("Drag the arm, then press Enter")
```

- Keep-alive is resent by a background SDK thread; `period` must be within `[0.005, 0.10)`.
- Other motion commands are rejected while keep-alive runs; queries are unaffected, and
  emergency stop / disable are exceptions.
- The exit is asynchronous; the firmware needs a moment after `zero_g_stop()` returns.
- If keep-alive breaks on a write failure, the exit raises rather than failing silently.
- The read-only `zero_g_active` / `zero_g_error` report the state.

### 5.7 Passthrough / servo

```python
send_mit(idx, q, dq, kp, kd, tau)
send_mit_all(q, dq, kp, kd, tau)
```

These bypass motion planning, and **the caller must keep them alive**: resend at ≥10 Hz, or the
0.1 s command watchdog drops into fail-soft (reduced stiffness + τ=0) and the arm sags slowly
under gravity.

The five arrays of `send_mit_all` must all have length `n` (checked locally), and they **must be
finite** — a `NaN` / `Inf` makes the **firmware** reject the whole frame with `ERR{cmd,0x02}`.
`send_mit` and `move_js` get the same finiteness check.

This group has not been fully verified on hardware, see
[troubleshooting §16](../TROUBLESHOOTING.md#16-not-yet-verified).

### 5.8 Sub-objects

#### `arm.params.*` — per-joint parameters

```python
set_joint_param(idx, kp, kd, tau_max)
set_joint_limits(idx, q_min, q_max)
get_joint_param(idx, timeout=1.0) -> Msg[JointParam]
all_joint_params() -> list[JointParam]
reset_factory()
```

`JointParam`: `idx`, `kp`, `kd`, `tau_max`, `q_min`, `q_max`.

`set_joint_limits()` **only allows narrowing**: writing the current values back is judged a
widening request and rejected (`ERR[23,2]`), so it is not idempotent. `reset_factory()` requires
the disabled state and is irreversible.

#### `arm.model.*` — online dynamics model import

```python
probe() -> bool
get_body(idx, timeout=1.0) -> Msg[list[float]]
set_body(idx, vals)              # needs 10 values
get_jm(timeout=1.0) -> Msg[list[float]]
set_jm(vals)                     # needs 7 values
status(timeout=1.0) -> Msg[ModelStatus]
get_gravity(q, timeout=1.0) -> Msg[list[float]]
commit(expected_mask)
revert()
```

`ModelStatus`: `override`, `staged_mask`, `dirty`.

Writes land in a staging layer and do not take effect immediately; judge by `staged_mask`.
`commit(expected_mask)` applies them, `revert()` discards them. **Both require the disabled
state**; while enabled they return `ERR{0x32,0x04}` / `ERR{0x37,0x04}` respectively.

⚠ `revert()` **rolls back RAM only — it does not touch flash.** Three consequences you need to
know:

1. the imported model in flash is still there, so it **comes back on the next power cycle**;
2. any subsequent `save_params()` **wipes it as well** (whole-sector erase, unrecoverable);
3. after the rollback `status().dirty == 1` (RAM ≠ flash), but do **not** read that as "needs
   committing".

**`set_jm()` should never be called**: it rewrites the joint mapping (including signs), a
mistake there can make the arm flail, and the only local recovery options are themselves
irreversible.

#### `arm.log.*` — 300 Hz control-tick capture

```python
start(n_ticks)
stop()
reader(timeout=1.0, retries=3) -> LogReader
capture(n_ticks, timeout=1.0, retries=3, record_timeout=None) -> list[LogSample]
dump(path, wait=True, timeout=1.0, retries=3, record_timeout=None) -> int
```

The `LogReader` you get from `r = arm.log.reader()` provides:

```python
r.total()                              # ticks written so far
r.wait_for(n_ticks, timeout=None, poll=0.05) -> int
r.read_all() -> bytes
r.samples() -> list[LogSample]
r.iter_chunks() -> Iterator[bytes]
r.total_bytes                          # property
```

`LogSample`: `tick`, `q_ref`, `dq`, `tau`.
Constants: `LOG_MAX_SAMPLES = 2400` (about 8 s @300 Hz, stops when full), `CTRL_HZ = 300`.

`reader()` reads back in chunks following the firmware cursor, retrying automatically on a
dropped chunk. `dump()` writes the raw byte stream to disk.

#### `arm.diag.*` — firmware self-test

```python
kin_bench(timeout=8.0) -> Msg[KinBenchResult]
```

`KinBenchResult`: `raw`, `timings`, `link`, plus `crc_errors`, `reply_dropped`, `can_tx_fail`,
`loop_max_kcycle`, `loop_overruns`, `rx_fifo_lost_motor`, `rx_fifo_lost_bridge`,
`gsusb_ring_drops`.

It is the only source of return-link diagnostic counters, but **all-zero counters may mean
nothing was actually read** — see
[troubleshooting §11](../TROUBLESHOOTING.md#11-every-kin_bench-counter-reads-0).

### 5.9 Firmware update (DFU)

```python
enter_dfu(timeout=0.3) -> None
```

The only terminal-state operation: enters the ROM bootloader without a probe.

- Two-stage: `ACK{0x15}` only means registered; you still wait for the device to disappear from
  CDC.
- Rejected locally while enabled (the jump stops TIM3, motors release within 100 ms).
- Afterwards this `Arm` is unusable (every entry point raises `ArmIsInDfuError`, `close()`
  excepted); the device re-enumerates as `0483:DF11`, and after flashing you create a new `Arm`.
- If the device does not disappear before the timeout it raises, and the object stays usable.

You cannot flash immediately after entering DFU; wait for USB re-enumeration.

### 5.10 Persistence

```python
save_params() -> None
```

Writes flash, irreversible.

### 5.11 Read-only properties

```python
params / model / log / diag      # sub-objects
last_reset_reason                # "normal" / "iwdg-rst" / None
zero_g_active / zero_g_error

n                                # joint count
firmware / fw_version            # version string / tuple
min_firmware / q_tol / dq_tol / arrive_frames / move_timeout
bench_model_axis                 # bench calibration axis
```

`last_reset_reason` is `None` in normal use, and that is correct: the boot signature is sent
once, only after a real MCU reset, and `reset()` does not make it repeat.

---

## 6. Exceptions

All inherit from `LiteArmError`.

```python
from litearm import (
    LiteArmError, NotConnectedError, ForkedSessionError, TransportError,
    FirmwareMismatchError, InvalidCommandError, MotorFaultError,
    MotionTimeoutError, IKError, CommandRejectedError, UnsupportedByFirmwareError,
    CartesianPlanError, MotionSupersededError, CartReplyLostError,
    ArmIsInDfuError,
)
```

| Exception | Raised when |
| --- | --- |
| `NotConnectedError` | Called while not connected, or after `close()` |
| `ForkedSessionError` | An inherited session is used in a child process (`NotConnectedError` subclass), sends nothing |
| `TransportError` | Serial read/write failure, or a frame fails its CRC check |
| `FirmwareMismatchError` | The firmware does not match the naming convention, or is below the lower bound |
| `InvalidCommandError` | Invalid argument (length, range, type), mostly caught locally |
| `MotorFaultError` | A FAULT flag or EMERGENCY appears in the state frames |
| `MotionTimeoutError` | The motion did not arrive within `move_timeout` |
| `IKError` | Inverse kinematics failed / the target is unreachable |
| `CommandRejectedError` | The firmware explicitly replied `ERR`, carries `.cmd` / `.code` |
| `UnsupportedByFirmwareError` | The firmware does not implement this command (`code == 0x00`), a subclass of `CommandRejectedError` |
| `CartesianPlanError` | The firmware rejected the plan. Hangs directly off `LiteArmError`, and does **not** inherit `InvalidCommandError` |
| `MotionSupersededError` | A Cartesian request was superseded — expected takeover, not a failure |
| `CartReplyLostError` | The `0x4E` reply was lost, outcome unknown |
| `ArmIsInDfuError` | This `Arm` has handed the device to the ROM bootloader, a terminal state |

`except InvalidCommandError` does not cover `CartesianPlanError`; what the firmware returned is
a planning result, not a rejected command.

In `ERR{cmd, code}`, `code == 0x00` always means the firmware does not have this command.

---

## 7. Things to watch out for

1. **The arm must never fly off.** This is the hard line, and the acceptance criterion for every
   safety-related change.
2. **A child process must not use an inherited session** — fail-closed, nothing sent. The parent
   must release the port first.
3. **`movej` returning does not mean it has settled** — a residual of about 0.012 rad, within
   `q_tol`.
4. **`movej` / `movej_sync` do not check joint limits** — compare against them yourself.
5. **After `disable()` the arm is no longer held.**
6. **`save_params()` cannot be undone.**

### Irreversible commands: do not run these on a calibrated arm

All four overwrite or erase that unit's per-arm identified dynamics model. The only way to be
safe is to do it on a board whose calibration has no value:

| Command | Entry point |
| --- | --- |
| `0x25` | `save_params()` |
| `0x32` | `arm.model.commit()` |
| `0x36` | `arm.params.reset_factory()` |
| `0x37` | `arm.model.revert()` |

`model.set_jm()` should never be called; a wrong joint mapping can make the arm flail, and there
is no dependable way back.

When stress-testing the CAN link, run `candump` (read-only) only, never `cangen`: `can0` is the
motor bus.

What has not been verified is listed in
[troubleshooting §16](../TROUBLESHOOTING.md#16-not-yet-verified).

---

## 8. Architecture

Each session starts one background read thread at `connect()` (`litearm-reader`, daemon). It is
the only place in the package that touches the transport's read side, and it does two things:
**deliver each frame to its queue, and be loud when it dies.**

```text
One read thread:  a frame arrives → file it into the matching queue by (frame id, echo code)
Everyone else:    send a command → wait on their own queues
```

A frame's ownership is decided by which queue it lands in, not by any thread. `ACK{0x10}` and
`ACK{0x11}` land in two different queues, so concurrent commands cannot eat each other's
replies. `RSP_STATUS` (a 100 Hz stream) does not go into a queue; it goes into a single slot
plus an arrival sequence number.

Two rules: **the read thread only delivers, it never judges**; and **the read thread must die
loudly** — the exception goes into `_reader_error`, waking every waiter, never exiting silently.

The cost is one extra thread per session, which is also why a child process cannot inherit the
session.

What it does not solve: there is no request id on the wire, so two identical concurrent commands
cannot be paired; and the firmware holds only one pending Cartesian plan, so Cartesian
concurrency depth is 1.

### Files

```text
src/litearm/
  _protocol.py     frame codec + command/reply constants + state parsing + coverage contract
  transport.py     pyserial CDC read/write / auto-discovery
  state.py         RobotState / JointState
  errors.py        exception hierarchy + error-code text
  arm.py           Arm core + CLI + read thread + fork guard
  cart.py          Cartesian: result-frame pairing + arrival criterion
  params.py        arm.params.*
  model.py         arm.model.*
  log.py           arm.log.*
  diagnostics.py   arm.diag.*
  _rot.py          pure rotation / pose math
  testing.py       offline stub (FakeTransport)
```

Every implemented downstream command in the firmware has an entry point. The contract lives in
`_protocol.COMMAND_COVERAGE` and is enforced in both directions by `tests/test_protocol_sync.py`,
which parses the firmware headers directly.

---

## 9. Command line

```bash
litearm-python [--port PORT] [ACTION] [TARGETS...] [--speed SPEED]
python -m litearm ...                 # equivalent
```

`ACTION` ∈ `status` (default) / `fw` / `enable` / `disable` / `reset` / `emergency` /
`movej` / `home` / `tcp`.

```bash
litearm-python status                          # read-only
litearm-python fw                              # version string + axis count
litearm-python tcp                             # current pose + frame rate
litearm-python movej -0.1 0 0 0 0 0 0 --speed 0.3
```

`movej` requires exactly `arm.n` values in `TARGETS`; `home` ignores `--speed` (the firmware
hard-codes 0.10).

---

## 10. Testing

```bash
pytest                         # full offline run, never touches hardware
LITEARM_LIVE=1 pytest          # plus hardware tests
python tests/test_offline.py   # runs without pytest too
```

Running the tests does not require installing the package; `tests/conftest.py` puts `src/` and
`tests/` on `sys.path`. Do not set `LITEARM_LIVE` with nobody present, and never call
`enter_dfu()` / `reset_factory()`.

| Test | What it guards |
| --- | --- |
| `test_protocol_sync.py` | Protocol drift protection: parses the firmware headers and compares command sets and IDs. The firmware repo location comes from the `LITEARM_FW_DIR` environment variable (default `~/litearm-stm32`); when it cannot be found the test **skips rather than passes** |
| `test_protocol_crc.py` | CRC16 against an external authoritative value plus an independent reference implementation |
| `test_frame_ownership.py` | A frame is not silently destroyed, and its owner receives it |
| `test_transport.py` | Real byte-stream parsing (back-to-back frames / noise / bad frames / partial frames) |
| `test_ack_echo.py` | Replies must echo the original command, so a late old ACK cannot make a new command succeed falsely |
| `test_capability.py` | "The firmware does not have this command" → `UnsupportedByFirmwareError` |
| `test_fork_guard.py` | The fork guard: a child sends nothing |
| `test_zero_g.py` | Hand-guiding keep-alive period / exit cleanup / thread reclamation / command gating |
| `test_live.py` | Hardware smoke test, only with `LITEARM_LIVE=1` |

The stub in `tests/fake_serial.py` must match the real firmware layout, or the offline run gives
a false green and fails on hardware.
