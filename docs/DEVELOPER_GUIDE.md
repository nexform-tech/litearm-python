# litearm-python Developer Guide & API Reference

Python SDK for the LiteArm robotic arm — **talks straight to the
`litearm-stm32` firmware** (USB CDC serial).

This SDK is a **thin protocol binding**: on the PC side it only encodes and
decodes frames, sends commands, and decides whether a move has arrived.
**Trajectory planning, kinematics and dynamics all live in the firmware**
(B2 S-curve + B3 kinematics + B4 dynamics + B1 control law); the PC side
**does none of it** — otherwise the same contract gets written twice, and
fixing one place while missing the other silently becomes two sets of
semantics.

**Zero dependencies** apart from `pyserial`. Poses are **plain Python lists**;
no numpy needed.

## Table of Contents

1. [Requirements & Installation](#1-requirements--installation)
2. [Quick Start](#2-quick-start)
3. [Connection Management](#3-connection-management)
4. [Reading State — The `Msg` Return Envelope](#4-reading-state--the-msg-return-envelope)
5. [API Reference](#5-api-reference)
6. [Exceptions](#6-exceptions)
7. [Architecture — One Reader Thread](#7-architecture--one-reader-thread)
8. [Command Line](#8-command-line)
9. [Testing](#9-testing)
10. [Safety Notes](#10-safety-notes)

---

## 1. Requirements & Installation

- Python **>= 3.9**
- `pyserial >= 3.4` (the only dependency)
- Firmware **`Litearm1.5.0+`**, convention `Litearm<major.minor.patch>-{7J|1J}`

```bash
pip install -e .

# development (includes pytest)
pip install -e ".[dev]"
```

> ⚠ When `pip` and `python` point at different interpreters, always use
> `python -m pip`, so the package lands in the interpreter you actually run.

---

## 2. Quick Start

```python
import litearm as pa

arm = pa.Arm().connect()          # auto-find CDC + check the firmware version convention
arm.enable()                      # must be enabled before motion
arm.movej([0.1, 0, 0, 0, 0, 0, 0], speed=0.3)
print(arm.get_tcp().value)        # read values through .value (return envelope since 2.0, see §4)
arm.close()
```

`Arm().connect()` is the **only entry point**. By the time `connect()` returns
the handshake is already done, so `arm.n` / `arm.firmware` are guaranteed
usable.

Every session carries **one background reader thread**, so every `Arm` needs
`close()` — `with` saves you the trouble:

```python
with pa.Arm().connect() as arm:
    print(arm.get_state().value.q)
# leaving the with block is close()
```

> ⚠ **After `fork` a child process must not inherit this session**, see
> [README](../README.md#1-multiprocessing--fork-a-child-must-not-use-an-inherited-arm).

---

## 3. Connection Management

```python
Arm(port=None, *, transport_factory=None, min_firmware=MIN_FW,
    q_tol=0.03, dq_tol=0.10, arrive_frames=3, move_timeout=15.0)

connect(port=None) -> Arm
close() -> None
disconnect() -> None                 # alias for close()
reconnect(port=None) -> Arm          # equals close() then connect()
__enter__() / __exit__(*exc)         # with usage; __exit__ returns False (does not swallow exceptions)
__del__()                            # GC fallback, equivalent to close()
```

| Parameter           | Default     | Meaning                                                                                                                              |
| ------------------- | ----------- | ------------------------------------------------------------------------------------------------------------------------------------ |
| `port`              | `None`      | serial path; `None` ⇒ auto-discover `1d50:606f`. Priority: the `port` argument > `LITEARM_PORT` > auto-discovery                     |
| `transport_factory` | `None`      | inject a transport (for tests). ⚠ you must still pass a placeholder `port`, because `connect()` goes through `find_cdc_port()` first |
| `min_firmware`      | `(1, 5, 0)` | lower bound of the version gate                                                                                                      |
| `q_tol`             | `0.03`      | arrival criterion: joint-angle tolerance (rad)                                                                                       |
| `dq_tol`            | `0.10`      | arrival criterion: joint-velocity tolerance                                                                                          |
| `arrive_frames`     | `3`         | arrival criterion: consecutive frames that must hold                                                                                 |
| `move_timeout`      | `15.0`      | motion timeout (s)                                                                                                                   |

| Method        | Notes                                                                                                                                                                                                                                                                      |
| ------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `connect()`   | **idempotent** (calling it again for the same target returns `self`). ⚠ when the handshake **write** fails it can leave a half-open session, and reconnecting then **silently reports success** |
| `close()`     | idempotent. Stops the reader thread → closes the transport. Afterwards every other entry point raises `NotConnectedError` (`close()` itself excepted)                                                                                                                      |
| `reconnect()` | **swaps the session**: the reader thread restarts and `Msg.hz` statistics reset                                                                                                                                                                                            |

Module constants: `litearm.MIN_FW`, `litearm.FIRMWARE_PREFIX`.

### Firmware Version Convention

`firmware` returns `Litearm<major.minor.patch>-{7J|1J}` (e.g. `Litearm1.8.0-7J`).

| Firmware                    | Result                           |
| --------------------------- | -------------------------------- |
| `Litearm1.5.x-*` and above  | ✅ accepted                      |
| `Litearm1.4.x-*` or earlier | ❌ `FirmwareMismatchError`       |
| `A1.x-*-USB` (old naming)   | ❌ does not match the convention |

> Status-frame parsing is **compatible with both** layouts, `4+21N` (≤1.4.x) and
> `6+21N` (≥1.5.0) — that compatibility branch is only used for offline /
> historical frame parsing (e.g. analysing a capture); `connect()` never
> reaches it.

---

## 4. Reading State — The `Msg` Return Envelope

### Which Entries Return `Msg`

**The 11 "read one frame" getters** return `Msg[T]` (a breaking change since
2.0):

| #  | Entry                         | Frame              |
| -- | ----------------------------- | ------------------ |
| 1  | `get_state()`                 | `RSP_STATUS`       |
| 2  | `get_status_now()`            | `RSP_STATUS`       |
| 3  | `get_tcp()`                   | `RSP_TCP`          |
| 4  | `get_ff_vec(item)`            | `RSP_FF_VEC`       |
| 5  | `get_ff_scalar(item, sub)`    | `RSP_FF_SCALAR`    |
| 6  | `params.get_joint_param(idx)` | `RSP_JOINT_PARAM`  |
| 7  | `model.get_body(idx)`         | `RSP_MODEL_PARAM`  |
| 8  | `model.get_jm()`              | `RSP_MODEL_JM`     |
| 9  | `model.status()`              | `RSP_MODEL_STATUS` |
| 10 | `model.get_gravity(q)`        | `RSP_GRAVITY`      |
| 11 | `diag.kin_bench()`            | `RSP_KIN_BENCH`    |

```python
@dataclass(frozen=True)
class Msg(Generic[T]):
    value: T          # raw return value (None on entries that cannot get a frame)
    hz: float         # average arrival rate of this frame class in this session
    timestamp: float  # local time.monotonic() of the latest frame (0.0 if never received)
```

**How `hz` is defined (fixed, not estimated)**: **the average rate of this
frame class since its first arrival in this session**,
`(frames arrived − 1) / (time of the latest frame − time of the first frame)`;
**when fewer than 2 samples exist it is `0.0`**.

- A passive continuous stream (`RSP_STATUS`, 100 Hz): converges to ~100 after
  two or three frames. An idle link does **not** make it decay.
- The 5 request/response ones (`get_joint_param` / `get_body` / `get_jm` /
  `get_gravity` / `kin_bench`): one call yields exactly one frame ⇒ **the first
  call necessarily has `hz == 0.0`**, and from the second call on it equals
  **your own polling rate**.
- After `reconnect()` the statistics reset.

⇒ **`hz == 0.0` together with `timestamp == 0.0` is the criterion for "this
frame class has never arrived"**, not for "the link is slow".

### Which Entries Do **Not** Return `Msg`

- `move_*` and `home()` — they return an "action result" (`RobotState` /
  `CartPlan`), not "one frame read"
- `n` / `firmware` / `last_reset_reason` / `zero_g_active` — there is no frame
  at all
- `ik()` — a **computation** request
- `license()` — a **request/response device-identity record** (no
  firmware-initiated traffic, so `hz` only measures your own polling rate)
- the two **derived** getters: `get_ff_mask()` is still a bare `int` (it is the
  scalar projection of `get_ff_scalar(9,0)`); `params.all_joint_params()` is
  still a `list[JointParam]` (it is an aggregate of N round trips, and a single
  `hz` cannot describe N frames)

### `RobotState`

```python
get_state(refresh=False, timeout=0.5) -> Msg[Optional[RobotState]]
get_status_now(timeout=0.5)           -> Msg[RobotState]
```

| Field                  | Meaning                                                                                 |
| ---------------------- | --------------------------------------------------------------------------------------- |
| `mode` / `mode_name`   | current mode                                                                            |
| `flags` / `flag_names` | raw flag bits and their names                                                           |
| `seq`                  | status-frame arrival sequence number — **tells you whether the stream is still moving** |
| `joints`               | `list[JointState]`                                                                      |
| `joint_fault`          | firmware G7 per-axis dropout bitmap (since 1.5.0; always 0 in the old layout)           |

Derived properties: `n`, `enabled` (flags bit9), `cart_busy` (flags bit10),
`q`, `dq`, `tau`, `fault_axes`, `faulted`, `fault_detail`,
`drop_hold_inferred`.

`JointState`: `q`, `dq`, `tau`, `t_mos`, `t_coil`, `err`.

> ⚠ `get_status_now(timeout=0.0)` is **not** "probe a frame without blocking";
> it means **"return the current cache immediately"**.

---

## 5. API Reference

A pose = a plain Python list of **6 numbers**: position 3 + RPY 3.

```python
pose = [px, py, pz, rx, ry, rz]

# all four spellings are accepted after normalisation by as_pose()
arm.move_p([0.30, 0.0, 0.35, 3.1416, 0, 0])
```

> ⚠ **Return shapes are not uniform**: `get_state()` / `get_status_now()` /
> `get_tcp()` give a `Msg` envelope; `movej` / `movej_sync` / `move_p` /
> `home` give `RobotState`; `move_l` / `move_c` / `move_path` give `CartPlan`;
> `ik()` gives `list[float]`. **Every entry's docstring states its own shape.**

### 5.1 Life / Safety

```python
enable(attempts=12)
disable()
emergency_stop()
reset()
clear_faults()
set_motion_mode(mode)
park()
```

| Method                  | Notes                                                                                                                                                       |
| ----------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `enable(attempts=12)`   | enables all joints. `attempts` is the retry count, and **retrying is whitelisted**: only `(0x10, 0x03)` is retried; resending other codes is useless        |
| `disable()`             | cuts the position loop. ⚠ once enable is cut, the arm is no longer held up                                                                                  |
| `emergency_stop()`      | single frame, one-way, reads no state — **the only entry point with no preconditions**                                                                      |
| `reset()`               | clears faults + re-anchors the control loop. ⚠ it is a **software state reset, not an MCU reboot** (no USB re-enumeration, the same object is still usable) |
| `clear_faults()`        | clears RAM fault bits only, **does not write flash**                                                                                                        |
| `set_motion_mode(mode)` | **the firmware only accepts `0`**; any other value raises `InvalidCommandError` locally (fail-closed)                                                       |
| `park()`                | equivalent to `set_motion_mode(0)`                                                                                                                          |

### 5.2 Joint-Space Motion

```python
movej(q, speed=1.0) -> RobotState
movej_sync(q, speed=1.0) -> RobotState
move_js(q, dq=None, tau_ff=None) -> None
home(*, timeout=None) -> RobotState
```

| Method                             | Notes                                                                                                                                                                                                                                                                                                                  |
| ---------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `movej(q, speed=1.0)`              | **single-shot**: the firmware plans the S-curve, completes it on its own, and holds still after arriving (no PC frame-by-frame keep-alive needed). `speed` ∈ `0..1`. ⚠ **joint limits are not checked** — an out-of-limit target is `clampf`-ed by the firmware and then **runs the full travel anyway**. Compare against the limits yourself before commanding motion |
| `movej_sync(q, speed=1.0)`         | synchronised PTP: all axes arrive together                                                                                                                                                                                                                                                                             |
| `move_js(q, dq=None, tau_ff=None)` | low-level joint stream, **bypasses the planner**. ⚠ `dq` is a **velocity reference, not a limit**; **the caller must resend at ≥10 Hz**, or the 0.1 s watchdog fail-softs                                                                                                                                              |
| `home(*, timeout=None)`            | firmware `CMD_HOME 0x2A`. ⚠ `timeout` is **keyword-only**; the firmware hard-codes the speed to **0.10 and does not accept speed**. Unlike `movej`, the firmware **explicitly allows** starting `home` from a pose past the soft limits or against an end stop                                                         |

> ⚠ `home(speed=0.3)` raises a `TypeError`. Write `arm.home()` or
> `arm.home(timeout=30.0)`.

### 5.3 Cartesian (Firmware-Planned)

```python
move_p(pose, speed=1.0, pos_tol=0.006, rpy_tol=0.03) -> RobotState
move_l(pose, speed=1.0, wait=True) -> CartPlan
move_c(pose_start, pose_via, pose_goal, speed=1.0, wait=True) -> CartPlan
move_path(poses, speed=1.0, wait=True) -> CartPlan
poll_cart() -> Optional[CartPlan]
set_speed(percent)
```

**Planning is entirely in the firmware**: the PC only sends points and receives
the `0x4E` result frame. How the three path entry points divide the work:

| Method                     | What the tool tip travels along                                                               |
| -------------------------- | --------------------------------------------------------------------------------------------- |
| `move_p(pose)`             | **joint-space** interpolation (point-to-point, **not** a straight line)                       |
| `move_l(pose)`             | a **straight line** (position lerp + orientation slerp)                                       |
| `move_c(start, via, goal)` | an **arc** (three points define the circle; `via`'s orientation is ignored)                   |
| `move_path(poses)`         | through the waypoints in order (**sharp corners**, the protocol has no corner-rounding field) |

| Method                            | Notes                                                                                                                              |
| --------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------- |
| `move_p`                          | ⚠ **accepts a single pose only**; passing a sequence raises `InvalidCommandError`. Arrival is judged by the TCP tolerance          |
| `move_l` / `move_c` / `move_path` | return a `CartPlan`. With `wait=False` they do not block; check progress with `poll_cart()`                                        |
| `move_c`                          | ⚠ `start` **must match the measured TCP at call time** (tolerance 6 mm / 0.03 rad) — it is a validated value, not a free parameter |
| `poll_cart()`                     | reads only the collector's unclaimed queue, **never touches the link**                                                             |
| `set_speed(percent)`              | global speed override. ⚠ **non-linear** (100→50 is only 1.48× slower), and the argument must be an **`int`** in `0..100`           |

**Known downgrades (relative to PC-side planning, deliberate since 2.0)**:
**no corner rounding**, **no preview before sending** (the firmware has no
dry-run; the `0x4E` only comes back once it has been sent), **PC-side speed
pre-check removed** (the criterion exists in exactly one place, the firmware).

⚠ The three failure shapes (all raise `CartesianPlanError`, and **the whole
plan is rejected, the arm does not move a single step**): `err=1` IK has no
solution / `err=2` three collinear points / `err=3` over capacity, unreachable.

⚠ **`movel` / `movec` / `movep` were renamed** to `move_l` / `move_c` /
`move_p`; the old names do not exist.

### 5.4 Pose / Kinematics

```python
get_tcp(timeout=0.6) -> Msg[Optional[tuple]]
ik(pose, q_seed=None, timeout=3.0) -> list[float]
```

| Method                  | Notes                                                                                                                           |
| ----------------------- | ------------------------------------------------------------------------------------------------------------------------------- |
| `get_tcp()`             | current TCP pose (firmware FK), **6 numbers** (not a rotation matrix)                                                           |
| `ik(pose, q_seed=None)` | inverse kinematics. ⚠ it may return **another, equally valid branch** ⇒ being far from the seed is **not necessarily** an error |

> ⚠ **There is no `fk(q)`** — the PC carries no kinematics model, so FK has
> only one route: "the current feedback" (`get_tcp()`).

### 5.5 Feed-Forward / Dynamics Tuning

```python
set_ff_mask(mask)
ff_preset(preset)                        # 0 / 1 / 2
set_ff_vec(item, values)                 # values length must = n
set_ff_scalar(item, sub, value)
get_ff_vec(item, timeout=1.0) -> Msg[list[float]]
get_ff_scalar(item, sub=0, timeout=1.0) -> Msg[float]
get_ff_mask(timeout=1.0) -> int          # ⚠ bare int, not Msg
set_gravity_scale(gs)                    # length = n
set_inertia_scale(isc)                   # length = n
set_payload(mass, com=(0.0, 0.0, 0.0))
set_gravity_vector(g)                    # length 3
```

⚠ **`get_ff_mask()` is a bare `int`** (it is the scalar projection of
`get_ff_scalar(9, 0)`; if you want that frame's envelope, call `get_ff_scalar`
directly).

⚠ `set_ff_vec` items **12~15** have only the generic entry point, no named
method: `12 zg_kp` / `13 zg_kd` / `14 zg_damping` (for zero-gravity
hand-guiding), `15 kd_extra` (τ-domain software derivative damping, cures the
ringing at the start of `movej`) — **`kd_extra` must stay 0 for wrist J5-J7**
(factory `[6,6,6,6,0,0,0]`).

⚠ The item-name tables are class attributes on `Arm` (and the whitelist used
for argument validation): `FF_VEC_ITEMS` (1..15), `FF_SCALAR_ITEMS` (1..18,
missing 9), `FF_SCALAR_RO_ITEMS` (only 9 = `ff_mask`).

⚠ Writes go to **RAM**; to persist them call `save_params()`.

### 5.6 Zero-Gravity Hand-Guiding

```python
zero_g(period=0.04)              # context manager
zero_g_start(period=0.04)
zero_g_stop(raise_on_lost=False)
```

`zero_g()` is a **client-side composition** (`zero_g_start` + `zero_g_stop`),
**not an RPC**.

```python
with arm.zero_g():
    input("drag the arm, then press Enter")
```

- **The SDK's background thread resends the keep-alive automatically**
  (default `period=0.04 s`) — firmware `0x06` carries its own `watchdog_kick`,
  so **failing to resend for 0.10 s drops out of fail-soft**. `period` must be
  ∈ `[0.005, 0.10)`.
- **Other motion commands are refused while the keep-alive is running**;
  **queries are unrestricted**, and **e-stop / disable are the exceptions**.
- **Exit is asynchronous**: after `zero_g_stop()` returns, the firmware side
  still needs a little time to wind down.
- If the keep-alive is interrupted by a **write failure**, exiting **raises**
  instead of staying silent.
- The read-only properties `zero_g_active` / `zero_g_error` report the state.

### 5.7 Passthrough / Servo (**The Second Channel**)

```python
move_js(q, dq=None, tau_ff=None)
send_mit(idx, q, dq, kp, kd, tau)
send_mit_all(q, dq, kp, kd, tau)
```

⚠ **These three bypass motion planning, and the caller must keep them alive
itself**: **resend at ≥10 Hz**, otherwise the 0.1 s command watchdog
fail-softs (drops stiffness + τ=0) and the arm slowly sags under gravity.

⚠ The five arrays of `send_mit_all` must each have length = `n`, and every
value must be **finite**.

⚠ **This group of entry points is not fully verified on real hardware** (see
[TROUBLESHOOTING](../TROUBLESHOOTING.md#explicitly-not-verified)).

### 5.8 Sub-Objects

#### `arm.params.*` — Joint-Level Parameters (4)

```python
set_joint_param(idx, kp, kd, tau_max)
set_joint_limits(idx, q_min, q_max)
get_joint_param(idx, timeout=1.0) -> Msg[JointParam]
all_joint_params() -> list[JointParam]
reset_factory()
```

`JointParam`: `idx`, `kp`, `kd`, `tau_max`, `q_min`, `q_max`.

- ⚠ `set_joint_limits()` **may only narrow**: writing back the **current
  values** is judged a "widening request" and rejected (`ERR[23,2]`) ⇒ **this
  entry point is not idempotent**, so don't use it for a write/read-back
  consistency check.
- ⚠ `reset_factory()` requires the **disabled state**; while enabled it
  returns `ERR{0x36,0x04}`. **Irreversible**.

#### `arm.model.*` — Online Dynamics-Model Import (9)

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

- Writes go into the **staging layer** and **do not take effect** — the
  criterion should be `staged_mask`, not "read back the live layer".
- Only `commit(expected_mask)` applies them; `revert()` discards them.
- ⚠ **`set_jm()` should never be called**: it changes the joint mapping
  (including signs), a wrong change risks the arm **flying off**, and on this
  machine the only recovery means (`revert` / `save_params`) are **themselves
  irreversible** ⇒ there is no fallback you can rely on.
- ⚠ `commit()` / `revert()` both **overwrite the per-unit identified dynamics
  model** stored on this machine, see §10.

#### `arm.log.*` — 300 Hz Control-Tick Capture (4 + `LogReader`)

```python
start(n_ticks)
stop()
reader(timeout=1.0, retries=3) -> LogReader
capture(n_ticks, timeout=1.0, retries=3, record_timeout=None) -> list[LogSample]
dump(path, wait=True, timeout=1.0, retries=3, record_timeout=None) -> int
```

```python
r = arm.log.reader()
r.total()                              # ticks already on disk in this session
r.wait_for(n_ticks, timeout=None, poll=0.05) -> int
r.read_all() -> bytes                  # read back the whole buffer
r.samples() -> list[LogSample]
r.iter_chunks() -> Iterator[bytes]
r.total_bytes                          # property
```

`LogSample`: `tick`, `q_ref`, `dq`, `tau`. Module constants:
`LOG_MAX_SAMPLES = 2400` (≈8 s@300 Hz, **stops by itself when full**),
`CTRL_HZ = 300`.

- `reader()` is a **factory**: it reads back in chunks following the firmware
  cursor `next_byte`, and **retries by cursor when a frame is dropped**.
- `dump(path)` writes the raw byte stream to disk (full scale ≈ 863 round
  trips; for a large buffer, dump to disk first and parse afterwards).
- ⚠ **In the disabled state `capture()` records 0 ticks, always** — that is
  **firmware behaviour**, not a defect.

#### `arm.diag.*` — Firmware Self-Test (1)

```python
kin_bench(timeout=8.0) -> Msg[KinBenchResult]
```

`KinBenchResult` methods/properties: `raw`, `timings`, `link`, plus
`crc_errors`, `reply_dropped`, `can_tx_fail`, `loop_max_kcycle`,
`loop_overruns`, `rx_fifo_lost_motor`, `rx_fifo_lost_bridge`,
`gsusb_ring_drops`.

⚠ **It is the only source of diagnostic counters for the back link** (`crc` /
`reply_dropped` / `can_tx_fail` / `loop_max_kcycle` / `loop_overruns`).
⚠ But **all five counters reading 0 may be a "silent 0"**, see
[TROUBLESHOOTING §11](../TROUBLESHOOTING.md#11-kin_benchs-five-counters-read-zero--silently).

### 5.9 License / Activation (Firmware 1.8.0+)

```python
license(timeout=1.0) -> LicenseInfo
activate(*, cust_id, issued, flags=0, mac, timeout=2.0) -> None
```

`LicenseInfo`: `state`, `ver`, `uid`, `cust_id`, `issued`, `flags` + the
derived `activated`, `factory_mode`, `state_name`, `uid_hex`.

- The firmware stores one record in a **dedicated flash sector** (sector 6),
  **written once and never erased**; while unactivated it **only locks
  `ENABLE`** (`ERR{0x10,0x08}`), and every other command behaves as usual.
- `license()` **does not raise while unactivated** (it is a **state**), and it
  **returns the UID even while unactivated** — that is the issuer's only
  source, so don't switch to the USB serial-number string.
- `activate()` takes **all arguments keyword-only**, and `mac` has no default.
  **The arm must be disabled first**, otherwise `ERR{0x3F,0x04}`.
- ⚠ **This package contains no key and no code that computes a MAC** — issuing
  happens in the vendor-side tool.
- ⚠⚠ `ERR{0x3F,0x02}` is an **aggregate code** (already activated / MAC
  mismatch / illegal key / write failure all share it). In this case the
  package **automatically reads back `0x2F`**: if the device really has
  `state != 0` it returns success, and only otherwise does it raise.
- Erasing the license record **is possible only over SWD**
  (`pyocd erase -s 0x080C0000`) — the firmware has **no** erase command.

### 5.10 Flashing DFU

```python
enter_dfu(timeout=0.3) -> None
```

**The only terminal-state operation.** Enters the ROM system bootloader
without a probe (`CMD_ENTER_DFU 0x15`).

- **Two-stage**: `ACK{0x15}` only means "registered"; you still have to wait
  for the device to really disappear from CDC.
- While enabled it is **rejected locally** (the jump stops TIM3 ⇒ the motors
  release after 100 ms and sag under load).
- After it returns successfully **this `Arm` can no longer be used** (every
  entry point raises `ArmIsInDfuError`, `close()` excepted); the device
  re-enumerates as `0483:DF11`, and after flashing the firmware you **create a
  new `Arm`**.
- If it has not disappeared before the timeout, it raises "registration
  withdrawn / not executed", and the object stays usable as before.

> ⚠ **Once in DFU you cannot flash immediately** — wait for USB
> re-enumeration. **First prove you can rescue it, then break it on purpose.**

### 5.11 Persistence

```python
save_params() -> None
```

Writes flash (`0x25`). ⚠ **Irreversible**, see §10.

### 5.12 Read-Only Properties

```python
params / model / log / diag      # sub-objects
last_reset_reason                # "normal" / "iwdg-rst" / None
zero_g_active                    # bool
zero_g_error                     # Optional[BaseException]

n                                # joint count (usable after the handshake)
firmware                         # version string, e.g. "Litearm1.8.0-7J"
fw_version                       # tuple, e.g. (1, 8, 0)
min_firmware                     # the lower bound this package requires
q_tol / dq_tol / arrive_frames   # arrival criterion
move_timeout                     # motion timeout
bench_model_axis                 # the axis used for bench calibration
```

> ⚠ **`last_reset_reason` is normally `None`, and that is correct
> behaviour** — the boot banner **is sent only once, after a real MCU reset**,
> and `reset()` does not make it repeat. See
> [TROUBLESHOOTING §10](../TROUBLESHOOTING.md#10-last_reset_reason-is-none-usually-correct).

---

## 6. Exceptions

All derive from `LiteArmError`.

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

| Exception                               | Raised when                                                                                                                                                                                               |
| --------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `NotConnectedError`                     | called while not connected; called after `close()`                                                                                                                                                        |
| `ForkedSessionError`                    | using an inherited session in a **child process that came out of `fork`** (a subclass of `NotConnectedError`). fail-closed: **not a single byte goes out**                                                |
| `TransportError`                        | serial read/write failure, or a frame fails its CRC check                                                                                                                                                 |
| `FirmwareMismatchError`                 | the firmware does not match the `Litearm<major.minor.patch>-{7J\|1J}` convention, or is below the lower bound                                                                                             |
| `InvalidCommandError`                   | invalid argument (length, range, type) — most are caught **locally**                                                                                                                                      |
| `MotorFaultError`                       | status-frame FAULT flag or EMERGENCY (including G7 single-axis fault degradation)                                                                                                                         |
| `MotionTimeoutError`                    | the motion did not arrive within `move_timeout`                                                                                                                                                           |
| `IKError`                               | inverse kinematics failed / target unreachable                                                                                                                                                            |
| `CommandRejectedError`                  | the firmware explicitly returned `ERR`. Carries `.cmd` / `.code` (**`.cmd` is echoed by the firmware, and is more trustworthy than what I just sent**)                                                    |
| `UnsupportedByFirmwareError`            | the firmware **does not implement** this command (`code == 0x00`). **Its superclass is `CommandRejectedError`**                                                                                           |
| `CartesianPlanError`                    | a firmware-planned move was rejected (no IK solution / three collinear points / over capacity / out of limits). ⚠ it **hangs directly off `LiteArmError`** and does **not** inherit `InvalidCommandError` |
| `MotionSupersededError`                 | a Cartesian request was superseded by a new one — **an expected takeover, not a failure**. **Deliberately not** a subclass of `CartesianPlanError`                                                        |
| `CartReplyLostError`                    | the `0x4E` reply was lost ⇒ **the outcome is unknown** (an SDK-invented semantic, not a firmware code)                                                                                                    |
| `ArmIsInDfuError`                       | this `Arm` has handed the device to the ROM bootloader — **a terminal state**, it cannot come back. **Deliberately not** a `NotConnectedError`                                                            |
| `NotRemoteable`                         | that entry point cannot be exposed remotely                                                                                                                                                               |
| `NotSupportedOnThisBackend`             | the current backend does not implement that capability                                                                                                                                                    |
| `TeleopLockedError` / `TeleopBusyError` | the teleop state refuses that command                                                                                                                                                                     |

> ⚠ **A deliberate change in what you can catch**: since 2.0
> `CartesianPlanError` hangs directly off `LiteArmError`, so
> `except InvalidCommandError` **no longer covers it** — what the firmware
> returns is a **planning result**, not "this command was rejected".

**Error codes**: in `ERR{cmd, code}`, `code == 0x00` **always means "the
firmware has no such command"**; it is a stable and unique capability-probe
sentinel.

---

## 7. Architecture — One Reader Thread

### Shape

Each session starts **one** background reader thread at `connect()`
(`litearm-reader`, daemon). It is the **only** place in the whole package that
touches `transport.read_frame`, and it does exactly two things: **deliver a
frame to the queue it belongs to, and die loudly.**

```text
one reader thread:  a frame arrives → put it on the right queue by (frame id, echoed code)
everyone else:      send a command → wait on their own queues
```

**A frame's ownership = which queue it lands on**, and is **not decided by the
thread**. `ACK{0x10}` and `ACK{0x11}` naturally land on **two** queues ⇒
concurrent commands cannot eat each other's replies.

- `RSP_STATUS` (a 100 Hz continuous stream) **does not go into a queue**: it
  goes into a **single slot** plus an arrival sequence number, and waiters wait
  for "the sequence number to advance".
- Stale replies are blocked by **clearing the queue before sending** (inside
  the **single write gate** `_raw_write`); `echo_cmd` is **mandatory** for
  `ACK`/`ERR` ⇒ same-id mutual eating is **structurally impossible**.

### Only Two Rules

1. **The reader thread only delivers, it never judges.**
2. **If the reader thread dies, it must die loudly.** A transport exception is
   stored in `_reader_error` → all waiters are woken → they raise it. **It
   never exits silently.**

### Three Implementation Constraints (None of Which May Be Wrong)

- **`close()` must stop the thread first, then close the transport.**
  Otherwise the reader thread raises `TransportError` from an already-closed
  transport, and a **normal shutdown** gets recorded as "link lost". Order:
  `zero_g_stop()` → stop the reader thread + `join(1.0)` → close the
  transport.
- **The reader thread is pinned to "the `_Ack` at the moment the thread
  started"** and does not re-read `self._a` inside the loop — otherwise the
  window during `reconnect()` would deliver frames to a mixed old/new object.
- **`_Ack` holds a weak reference to `Arm`.** The thread's target is a bound
  method ⇒ the thread strongly references `_Ack`; if `_Ack` then strongly
  referenced `Arm`, **an `Arm` whose `close()` you forgot could never be
  collected**, `__del__`'s fallback cleanup would never fire ⇒ the port would
  never be released.

### Cost

⚠ **One extra thread per session**. The reader thread's back-off uses
`Event.wait` rather than `time.sleep` (the latter would be caught tens of
thousands of times by the "how long did we wait" probes in the tests).

⚠ **A child process cannot be used after `fork`** — this is the **only**
system-level cost of this architecture, and corresponds to item 1 of the
README's "two must-reads".

### What This Mechanism Does **Not** Solve (Don't Expect It To)

| Not solved                                                                       | Root cause                                                                                   | Which layer   |
| -------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------- | ------------- |
| pairing two **completely identical** commands issued concurrently                | there is **no request id** on the wire                                                       | wire protocol |
| firmware `plan_pending` is a single slot ⇒ Cartesian concurrency depth 1         | device-side capacity                                                                         | firmware      |
| a stale reply still in a transport buffer / on the wire escaping the queue clear | same as above (no request id) — **a known residual window that can no longer be contracted** | wire protocol |

> 📌 Measured: under long-running concurrency the queue depth **peaks at 1**
> (the cap of 64 is never approached), and the firmware-side `crc` / `rxfifo`
> counter deltas are all 0 — **no frames are lost, structurally**.

### Two Performance Facts About the Transport Layer

- **Reads are "read as much as is available"** (request by `in_waiting`,
  capped at 4096 bytes), **not byte-by-byte**. The cost of byte-by-byte reading
  is proportional to the number of bytes, and this board's idle status stream
  alone is ~14 kB/s ⇒ **it would burn a whole core**.
- **CRC16 goes through `binascii.crc_hqx`** (poly `0x1021` / init `0xFFFF`),
  not a pure-Python double loop — **232×** faster.

Measured gains (idle CPU of a bare SDK session): byte-by-byte + pure-Python
CRC **98.8%** → read-as-much-as-available **21.4%** → CRC switched to the C
implementation **6.8%**.

### Files

```text
src/litearm/
  _protocol.py     frame codec (0xA5 CMD LEN PAYLOAD CRC16-CCITT-FALSE) + command/reply constants
                   + status-frame parsing + boot-banner parsing + the COMMAND_COVERAGE contract
                   ⚠ the two license entries are here too (0x2F/0x3F/0x4F/0x50); no MAC-computing code
  transport.py     pyserial CDC read/write + auto-discovery (VID:PID 1d50:606f);
                   one lock each for read and write (the zero_g keep-alive thread is the first concurrent writer)
  state.py         RobotState / JointState
  errors.py        error hierarchy + ERR_TEXT code table
  arm.py           Arm core + CLI + reader thread `_Ack` + fork guard
  cart.py          Cartesian: 0x4E pairing + arrival criterion (bit10) + capability probe
                   ⚠ no PC-side planning — planning is in the firmware
  params.py        arm.params.*  joint-level parameters
  model.py         arm.model.*   online dynamics-model import
  log.py           arm.log.*     300Hz capture + LogReader cursor read-back
  diagnostics.py   arm.diag.*    KIN_BENCH self-test
  _rot.py          pure rotation/pose math (rpy⇄matrix, as_pose normalisation)
  testing.py       the public offline stub (FakeTransport) — builds frames and replies from the real firmware layout
```

### Coverage Contract

Every **implemented** downstream command in the firmware's `hal/usb_cmd.h` has
a matching entry point. The contract lives in `_protocol.COMMAND_COVERAGE`
(command id → SDK entry point) and is enforced in both directions by
`tests/test_protocol_sync.py` **parsing the firmware header directly** — if
the firmware adds a command and the SDK doesn't follow, or a status-frame
layout is changed, the test fails immediately.

---

## 8. Command Line

```bash
litearm-python [--port PORT] [ACTION] [TARGETS...] [--speed SPEED]
python -m litearm ...                 # equivalent
```

`ACTION` ∈ `status` (default) / `fw` / `enable` / `disable` / `reset` /
`emergency` / `movej` / `home` / `tcp`.

```bash
litearm-python status                          # read-only
litearm-python fw                              # version string + axis count
litearm-python tcp                             # current pose + frame rate
litearm-python movej -0.1 0 0 0 0 0 0 --speed 0.3
```

⚠ `movej` requires the number of `TARGETS` to be **exactly `arm.n`**; `home`'s
`--speed` is **ignored** (the firmware hard-codes 0.10).

---

## 9. Testing

```bash
pytest                         # full offline flow (stub transport, no real hardware)
PYLITEARM_LIVE=1 pytest        # + live hardware (needs a Litearm1.5.0+ full arm/bench connected; it moves slightly)
python tests/test_offline.py   # works without pytest too (script-style assertions)
```

**Running the tests does not require installing the package** —
`tests/conftest.py` puts `src/` and `tests/` on `sys.path` itself.

⚠ **Do not set `PYLITEARM_LIVE` when nobody is present**, and never call
`enter_dfu()` / `reset_factory()`.

The critical groups:

| Test                      | What it guards                                                                                                                                                                                                                                                                                                       |
| ------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `test_protocol_sync.py`   | **protocol-drift protection** — parses the firmware repo's `usb_cmd.h` / `usb_cmd.c` / `joint_cfg.h` directly, compares command sets and IDs in both directions, and pins the status-frame layout. The firmware repo location is given by `LITEARM_FW_DIR`; **when it cannot be found it skips rather than passing** |
| `test_protocol_crc.py`    | CRC16 — uses an **external authoritative check value** (`"123456789"` → `0x29B1`) plus an independent in-file reference implementation, deliberately not depending on the implementation under test                                                                                                                  |
| `test_frame_ownership.py` | the frame-ownership contract: **the frame was not silently destroyed, its owner got it**                                                                                                                                                                                                                             |
| `test_transport.py`       | real byte-stream parsing (consecutive frames / noise / bad frames / half frames / both arrival methods decoding the same frame)                                                                                                                                                                                      |
| `test_ack_echo.py`        | an ACK must echo the original command (a late old ACK must not make a new command "succeed" falsely)                                                                                                                                                                                                                 |
| `test_capability.py`      | `ERR{cmd,0x00}` → `UnsupportedByFirmwareError` ("a real rejection" must not be misjudged)                                                                                                                                                                                                                            |
| `test_fork_guard.py`      | the fork guard: a child process sends nothing                                                                                                                                                                                                                                                                        |
| `test_zero_g.py`          | zero-gravity keep-alive period / exit wind-down / thread reclamation / command gating                                                                                                                                                                                                                                |
| `test_status_layout.py`   | both status-frame layouts, `6+21N` and `4+21N`                                                                                                                                                                                                                                                                       |
| `test_live.py`            | real-hardware smoke test (only runs with `PYLITEARM_LIVE=1`)                                                                                                                                                                                                                                                         |

⚠ **The stub in `tests/fake_serial.py` must match the real firmware layout** —
if the stub disagrees with the real firmware, offline tests go "falsely green"
while real hardware is guaranteed to fail. That lesson has a dedicated test
guarding it.

---

## 10. Safety Notes

1. **The arm must never fly away.** This is the product's hard line, and the
   acceptance criterion for every safety change.
2. **After `fork` a child process must not use an inherited session** —
   fail-closed, zero bytes sent. The parent **must release the port first**.
3. **`movej` returning ≠ settled** (residual ~0.012 rad, inside `q_tol`).
4. **`movej` / `movej_sync` do not currently check joint limits** — an
   out-of-limit target runs the full travel. Compare against the limits
   yourself before sending.
5. **After `disable()` the arm is no longer held up** — once the position loop
   is cut, it will move.
6. **`save_params()` has no undo.**

### ⚠ Irreversible Commands — Do Not Run on a Calibrated Arm

These four **overwrite/erase the per-unit identified dynamics model** of that
device (whole-sector erase + write of the current RAM):

| Command | Entry point                  |
| ------- | ---------------------------- |
| `0x25`  | `save_params()`              |
| `0x32`  | `arm.model.commit()`         |
| `0x36`  | `arm.params.reset_factory()` |
| `0x37`  | `arm.model.revert()`         |

**The only way to unlock this: do it on a board with no calibration value.**

⚠ **`model.set_jm()` should never be called** — a wrong joint mapping risks the
arm **flying off**, and there is no fallback you can rely on.

⚠ When stress-testing the CAN link, **run only `candump` (read-only), never
`cangen`** — `can0` is the motor bus.

**The explicitly unverified parts** are in
[TROUBLESHOOTING](../TROUBLESHOOTING.md#explicitly-not-verified).

## License

MIT
