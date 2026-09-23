# litearm-python — LiteArm STM32 Direct Backend

Python SDK for the LiteArm robotic arm: it **talks directly to the
`litearm-stm32` firmware** (USB CDC serial), mirroring the common usage of
`pylitearm` with a high-level subset.

This is a **thin protocol binding** — **the PC side does no trajectory
planning, no kinematics, no dynamics**. All of it is carried by the firmware
(B2 S-curve + B3 kinematics + B4 dynamics + B1 control law). The PC side does
exactly three things: encode and decode frames, issue commands, decide arrival.

**The `pylitearm` source is not modified.** **Zero dependencies** beyond
`pyserial` (no numpy / pinocchio).

## Install

```bash
pip install -e .            # the only dependency is pyserial
```

## Quick Start

```python
import litearm as pa

arm = pa.Arm().connect()          # auto-find the CDC port, check the firmware version convention
arm.enable()                      # motion requires enable first
arm.movej([0.1, 0, 0, 0, 0, 0, 0], speed=0.3)   # single shot: the firmware S-curve completes it
print(arm.get_tcp().value)        # current TCP pos[3] + rpy[3] (since 2.0, values are read via .value)
q = arm.ik((0.30, 0.0, 0.35, 3.1416, 0, 0))     # pose → joint angles (asynchronous firmware IK)
arm.close()
```

`Arm().connect()` is the **only entry point**. A session carries a background
read thread, so every `Arm` needs `close()` — a `with` block saves you that:

```python
with pa.Arm().connect() as arm:
    print(arm.get_state().value.q)
```

## CLI Inspection

```bash
litearm-python status                          # or python -m litearm ...
litearm-python movej -0.1 0 0 0 0 0 0 --speed 0.3
litearm-python home
```

`status` / `fw` / `tcp` are read-only; `enable` / `disable` / `reset` /
`emergency` / `movej` / `home` really move the arm.

## ⚠ Two Must-Reads (These Bite)

### 1. Multiprocessing / `fork()`: a child **must not** use an inherited `Arm`

Every session in this package carries **a background read thread** (see
[Architecture](docs/DEVELOPER_GUIDE.md#7-architecture--one-reader-thread)). Threads are **not copied
by `fork`**, but file descriptors are — so in a child process: commands **really
go out on the wire**, yet nothing ever reads the replies; the caller only sees a
"no response" timeout, and a retry means **a duplicate command**. Reading state
is subtler — it **does not error**, it just silently returns the inherited,
**stale** value.

So this package is **fail-closed**: any command in a child process immediately
raises `ForkedSessionError` (a subclass of `NotConnectedError`), and **not a
single byte goes out**.

**⚠ For a child process to use the arm, the parent must release the port
first.** The serial port is exclusive, and while the parent still holds it the
child **cannot connect** — both ends block it: the in-process port registry
(copied by `fork` and still pointing at the parent's live transport), and
pyserial's `flock` on the **inherited fd** from `exclusive=True`.
⇒ The right way is to **`close()` the parent's session first, then `fork`**, and
then **create** a new `Arm` in the child. ("Leave the parent alone and only
`connect()` in the child" **does not work** on real hardware.)

```python
# On Linux multiprocessing defaults to fork ⇒ this is not a rare path
def worker():
    a = pa.Arm().connect()        # ✅ create it inside the child
    ...

a = pa.Arm().connect()
a.close()                          # ✅ release the port first — miss this and the child cannot connect
p = multiprocessing.Process(target=worker)
p.start()                          # do not pass a into the child
```

⚠ A child process **should not call `close()`**: that takes a lock inherited from
the parent that will never be released — touch it and you **hang forever**. It is
also **not guaranteed safe** in a child: the light path itself takes
`_tx_repeat_lock`. Let that inherited fd be closed by the kernel when the
process exits — it neither reads nor writes, so it is harmless.

### 2. Unactivated board: `enable()` is rejected (`ERR{0x10,0x08}`)

Since firmware 1.8.0 the board is **locked from boot**: when unactivated, the
**first** criterion in `ctrl_enable()` is "is it authorized" — **resending does
nothing, there is no bypass**. **Every other command works as usual** (after-sales
and production lines must be able to diagnose), and `license()` returns normally
too — see [License / Activation](#license--activation-firmware-180).

## Firmware Version Convention

`firmware` returns `Litearm<major.minor.patch>-{7J|1J}` (e.g. `Litearm1.8.0-7J`).
Checked at connect time:

| Firmware                                        | Result                                                             |
| ----------------------------------------------- | ------------------------------------------------------------------ |
| `Litearm1.5.x-7J` / `Litearm1.5.x-1J` and above | ✅ accepted (minimum **1.5.0**)                                     |
| `Litearm1.4.x-*` or earlier                     | ❌ `FirmwareMismatchError`                                          |
| `A1.x-*-USB` (old naming)                       | ❌ does not match the convention (suggest flashing `Litearm1.5.0+`) |

> The version gate is pinned at 1.5.0, but state-frame parsing **also accepts**
> both layouts, `4+21N` (≤1.4.x) and `6+21N` (≥1.5.0) — that compatibility
> branch is only used for offline / historical frame parsing (e.g. analysing a
> capture), and `connect()` never reaches it.

## License / Activation (Firmware 1.8.0+)

The firmware stores one license record in a **separate flash sector** (sector 6),
**written once and never erased**; when unactivated it **locks only `ENABLE`**,
and every other command works as usual.

```python
lic = arm.license()                       # 0x2F → LicenseInfo
if not lic.activated:
    print(lic.state_name, lic.uid_hex)    # uid_hex is the 24-digit hex the issuer wants
    arm.disable()                         # activate requires the disabled state first
    arm.activate(cust_id=<customer-id>, issued=<YYYYMMDD>,
                 mac=<16 bytes issued by the vendor>)   # 0x3F
```

- `license()` → `LicenseInfo`, **does not raise when unactivated** (it is a
  **state**), and **returns the UID even when unactivated** — that is the
  issuer's only source; do not switch to the USB serial-number string.
- `activate(*, cust_id, issued, flags=0, mac)` — `mac` is issued by the vendor.
  **Must be disabled first**, otherwise `ERR{0x3F,0x04}`; a local precheck covers
  the `mac` length and the `flags` reserved bits, and a non-conforming frame is
  **never sent**.
- ⚠ **This package holds no key and no code that computes a MAC** — issuing
  happens in a vendor-side tool. This is a hard spec requirement: if the customer
  side has any code that can compute a MAC, this whole mechanism is worth
  nothing.
- ⚠⚠ `ERR{0x3F,0x02}` is an **aggregate code**: the firmware folds "already
  activated / MAC mismatch / invalid key / write failure" all into one code ⇒
  **the code alone reports a machine that is in fact unlocked as a failure**.
  This package **automatically reads back `0x2F`** on this code: if the device
  really has `state != 0` it returns success, and only otherwise raises.
- Erasing the license record **is only possible over SWD**
  (`pyocd erase -s 0x080C0000`) — the firmware has **no** erase command.

## API Overview

| Group                     | Entry                                                                                                                                                                        |
| ------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Session                   | `connect` `close` `disconnect` `reconnect` `__enter__`                                                                                                                       |
| Life / safety             | `enable` `disable` `emergency_stop` `reset` `clear_faults` `set_motion_mode` `park`                                                                                          |
| Joint motion              | `movej` `movej_sync` `move_js` `home`                                                                                                                                        |
| Cartesian                 | `move_p` `move_l` `move_c` `move_path` `poll_cart` `set_speed`                                                                                                               |
| State / kinematics        | `get_state` `get_status_now` `get_tcp` `ik`                                                                                                                                  |
| Feed-forward / dynamics   | `set_ff_mask` `ff_preset` `set_ff_vec` `set_ff_scalar` `get_ff_vec` `get_ff_scalar` `get_ff_mask` `set_gravity_scale` `set_inertia_scale` `set_payload` `set_gravity_vector` |
| Passthrough / servo       | `send_mit` `send_mit_all`                                                                                                                                                    |
| Zero-gravity hand-guiding | `zero_g` (context manager) `zero_g_start` `zero_g_stop`                                                                                                                      |
| License                   | `license` `activate`                                                                                                                                                         |
| Flashing                  | `enter_dfu`                                                                                                                                                                  |
| Persistence               | `save_params`                                                                                                                                                                |
| Sub-objects               | `arm.params.*` (4) · `arm.model.*` (9) · `arm.log.*` (4 + `LogReader`) · `arm.diag.kin_bench`                                                                                |
| Read-only properties      | `n` `firmware` `fw_version` `min_firmware` `q_tol` `dq_tol` `arrive_frames` `move_timeout` `bench_model_axis` `last_reset_reason` `zero_g_active` `zero_g_error`             |

Full signatures, return types and per-entry caveats: see
[docs/DEVELOPER_GUIDE.md](docs/DEVELOPER_GUIDE.md).

### ⚠ Dangerous Entry Points

Read the warning before the signature.

**`save_params()` — persist to flash (`0x25`).**
It writes the **current RAM**; there is no undo.

**`reset_factory()` — factory reset, invalidating flash (`0x36`).**
**The firmware requires the disabled state**; when enabled it returns
`ERR{0x36,0x04}`.

**`enter_dfu()` — the only terminal-state operation.**
Two-stage (`ACK{0x15}` only means "registered"; you still have to wait for the
device to really disappear from CDC); rejected locally while enabled (the jump
stops TIM3 ⇒ motors release in 100 ms and sag under load). After it returns
successfully **this `Arm` is unusable** (every entry point raises
`ArmIsInDfuError`, `close()` excepted), the device re-enumerates as `0483:DF11`,
and after flashing you **create a new `Arm`**.

**`send_mit` / `move_js` — bypass planning, and the caller must keep them alive.**
**Requires resending at ≥10 Hz**, otherwise the 0.1 s command watchdog fails
soft.

**`disable()` — once enable is cut, the arm is no longer held by the position
loop.**

## Docs

- [docs/DEVELOPER_GUIDE.md](docs/DEVELOPER_GUIDE.md) — full API reference, return
  envelopes, architecture
- [TROUBLESHOOTING.md](TROUBLESHOOTING.md) — field notes: failure modes that are
  easy to misdiagnose
- [examples/README.md](examples/README.md) — runnable examples (read-only by
  default, motion needs `--go`)

## Development

```bash
pip install -e ".[dev]"
pytest                         # full offline flow (stub transport, never touches real hardware)
PYLITEARM_LIVE=1 pytest        # + real-hardware live (needs a Litearm1.5.0+ arm/bench attached; moves a little)
```

⚠ **Do not set `PYLITEARM_LIVE` with nobody present**, and never call
`enter_dfu()` / `reset_factory()`.

Examples load the environment first:

```bash
source env.sh                       # exports PYTHONPATH/PYTHON_BIN/LITEARM_PORT
python3 examples/01_hello.py
./run_example.sh 02_movej.py --go   # or one-shot via the wrapper script
```

On Windows use `env.ps1` / `env.cmd` and `run_example.ps1` / `run_example.cmd`;
`LITEARM_PORT` can pin `COM5` and the like.

## License

MIT
