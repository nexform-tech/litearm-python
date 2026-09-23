# litearm-python — Troubleshooting

These are the failure modes that are **easy to misdiagnose**. Each entry gives three
things: the symptom, the real cause, and **how to tell the two causes apart**.

> There is **no request id** between this package and the firmware, so many problems
> that "look like network/timeout issues" are really protocol-semantics problems.
> Suggested order of investigation: look at the **error code** first (§4 has two code
> spaces, extremely easy to confuse), then the **link diagnostic counters** (§11), and
> only then suspect the wiring.

## Contents

1. [`connect()` fails](#1-connect-fails)
2. [`enable()` is refused — each of three codes means something
   different](#2-enable-is-refused--each-of-three-codes-means-something-different)
3. [Odd behaviour in a `fork`ed child](#3-odd-behaviour-in-a-forked-child)
4. [`ERR{0x02,0x03}` is ambiguous](#4-err0x020x03-is-ambiguous)
5. [Two different 1~6 code spaces](#5-two-different-16-code-spaces)
6. [A burst of 3+ Cartesian commands always loses a reply](#6-a-burst-of-3-cartesian-commands-always-loses-a-reply)
7. [`movej` returns before the arm has settled](#7-movej-returns-before-the-arm-has-settled)
8. [`move_c()` arc failures](#8-move_c-arc-failures)
9. [Cartesian accuracy is not a single number](#9-cartesian-accuracy-is-not-a-single-number)
10. [`last_reset_reason` is `None` (usually correct)](#10-last_reset_reason-is-none-usually-correct)
11. [`kin_bench`'s five counters read zero — silently](#11-kin_benchs-five-counters-read-zero--silently)
12. [A constant Cartesian offset — check `payload_mass` first](#12-a-constant-cartesian-offset--check-payload_mass-first)
13. [`zero_g` keep-alive and asynchronous teardown](#13-zero_g-keep-alive-and-asynchronous-teardown)
14. [Watchdog fail-soft on `move_js` / `send_mit`](#14-watchdog-fail-soft-on-move_js--send_mit)
15. [After `enter_dfu()`](#15-after-enter_dfu)
16. [Two counter-intuitive parameter writes](#16-two-counter-intuitive-parameter-writes)

---

## 1. `connect()` fails

```text
FirmwareMismatchError: firmware version does not match the convention ...
TransportError: cannot open the serial port / no CDC device found
```

| Cause | How to confirm |
| --- | --- |
| **Device not found** — not plugged in, driver not loaded, not `1d50:606f` | `lsusb` to see whether it is there; call `litearm.find_cdc_port()` on its own and see what it returns |
| **Port already taken** — another process/session still has it open | On Linux, `fuser /dev/ttyACM0`; on Windows endpoint exclusivity is **enforced by the OS**, so if you cannot grab it, it will not open |
| **Version gate** — firmware < 1.5.0, or the older `A1.x-*-USB` naming | Read the `FirmwareMismatchError` message directly; it echoes the version string it read verbatim |

⚠ Device re-enumeration (unplug/replug, after `enter_dfu()`, a real power-cycle restart)
makes `/dev/ttyACM*` **change number**. A script pinned to `LITEARM_PORT` then points at a
port that does not exist — this is the most common reason for "it was fine a moment ago".

---

## 2. `enable()` is refused — each of three codes means something different

`enable(attempts=12)`'s **retry is a whitelist**: **only `(0x10, 0x03)` is retried**.
Resending any other code is **useless** — it only makes you wait for nothing.

| Code | Meaning | What to do |
| --- | --- | --- |
| `ERR{0x10,0x03}` | A retryable transient failure (the only one in the firmware-side whitelist) | Leave it to `attempts`, or resend later |
| `ERR{0x10,0x08}` | **Not activated** — locked from boot as of firmware 1.8.0; the first check in `ctrl_enable()` is the licence | Go through `license()` / `activate()`; see the README section on licensing/activation |
| `ERR{0x10,0x06}` | A **latched** fault (the `joint_fault` family); **resending is useless** | `reset()` first, then find which axis it is (`state.joint_fault` / `state.fault_axes`) |
| `ERR{0x10,0x07}` | Resending is useless | Check the firmware code table |
| `ERR{0x10,0x00}` | **The firmware does not have this command** | The firmware is too old |

⚠ One item unrelated to `enable` that often gets mixed in: **when the arm is not enabled,
`movej` is refused with `ERR[01,3]`**, and its message names **both** possibilities at once
— "not enabled, **or** EMERGENCY latched". Do not read only the first half.

---

## 3. Odd behaviour in a `fork`ed child

See item 1 of the README's "two must-reads". Here we only list **how to recognise it**:

| Symptom | Explanation |
| --- | --- |
| The command "times out with no reply", but a retry "sometimes works again" | The command **really was sent** (the fd is inherited); the reply is eaten by the **parent's reader thread** ⇒ the retry is a **duplicate send** |
| `get_state()` raises nothing, but the readings **never move** | It silently returns the inherited **stale** values — the most insidious kind |
| `connect()` in the child raises | The parent still holds the port (see the README: **the parent must `close()` first**) |
| Any command immediately raises `ForkedSessionError` | ✅ The guard is **working properly**; this is not an error |

Real-hardware verification: in the child, `movej` / `get_state` / `get_tcp` / `connect`
**all** raise `ForkedSessionError`, **not one byte is sent**, and the parent session is
unaffected.

---

## 4. `ERR{0x02,0x03}` is ambiguous

**The same `(command, code)` has two completely different origins**:

- `usb_cmd.c` — **not enabled / EMERGENCY latched**;
- `kin_runner.c` — **IK unreachable / invalid solution**.

⇒ **Receiving it does not prove "the arm is not enabled"**; field attribution will point
the wrong way. Discriminator: look at the **current state** (`state.enabled` /
`state.mode`), not at the code. If the arm is in fact enabled, then it is the IK branch.

---

## 5. Two different 1~6 code spaces

| Source | Meaning |
| --- | --- |
| The `err` field in the `0x4E` reply (`CartPlan.err`) | `cart_err_t`: the planner's own verdict (no solution / collinear / over-capacity / out of limits …) |
| The **second byte** of `RSP_ERR` | The **gate reason code**: not enabled `0x03` / in zero-g `0x04` / `drop_hold` `0x06` |

**Both take values in 1~6, and their meanings have nothing to do with each other.** When
you get a "3", first ask which of the two paths it came out of.

⚠ One related known **comment bug**: `usb_cmd.h` says three-point collinearity is `0x03`,
but **the code is right** — collinear is `err = 2`.

---

## 6. A burst of 3+ Cartesian commands always loses a reply

**Reproduced on real hardware 3/3.** Symptom: send several `move_l` / `move_path` in a
row and one of them reports `CartReplyLostError` ("outcome unknown"), or the later
replies are **shifted wholesale** (this command's answer is picked up by the next one).

**The root cause is in the firmware, not the SDK**: `plan_pending` in `cart_exec.c` is a
**single bool plus a single payload, not a queue**. The CANCELED sent by superseded
commands overwrite each other ⇒ one missing reply shifts the entire subsequent FIFO
pairing.

**What to do about it**: send Cartesian commands **serially** — wait for one to finish
before sending the next. (The SDK already serialises calls **within one process** with
`_cart_serial`, so normal usage never hits this; you hit it when going concurrent across
processes/clients.)

---

## 7. `movej` returns before the arm has settled

`movej` is **"return as soon as the arrival criterion is met"**, where the criterion is
"every axis |q−target| < `q_tol` and dq quiescent for `arrive_frames` consecutive frames"
— **at the instant it returns, the arm has not settled yet**.

Measured (command J6 to 0.7): the value read the instant it returns is **0.6884**; within
5 s it converges on its own to **0.6991** and then holds steady for 20 s. The difference
is ~0.012 rad (0.7°), inside `q_tol = 0.03` — **a design convention, not drift**.

⇒ **"Read the state immediately after `movej` returns" shows you this residual.** If you
need the exact value, wait a few seconds before reading, or tighten `q_tol` yourself.

---

## 8. `move_c()` arc failures

| Symptom | Cause |
| --- | --- |
| `CartesianPlanError{err=2}` | **Three points collinear** (or nearly so) — the centre runs off to infinity |
| `CartesianPlanError{err=1}` | No IK solution. ⚠ Starting from the **fully extended `home` pose** (a singularity) **necessarily** gives `err=1`; that is **reasonable behaviour** of the firmware IK, not a defect |
| `CartesianPlanError{err=3}` | Over-capacity / unreachable |

⚠ `move_c(start, via, goal)`'s **`start` must match the measured TCP at call time**
(tolerance 6 mm / 0.03 rad). It is not a free "where to start from" parameter; it is
**validated on receipt** — using the TCP of an arm that is moving as the start point will
be refused.

⚠ `via`'s **orientation is ignored**; only its position takes part in defining the circle.

---

## 9. Cartesian accuracy is not a single number

**The residual endpoint error depends on all three of [distance × speed × payload
attitude]**; it is not a fixed specification of the device. A change of payload
significantly changes the residual error (the residual error **is payload-dependent**).

⇒ Any accuracy comparison must use **the same attitude, the same convention, the same
payload**; otherwise what you measure is an "attitude difference / payload difference",
not an "accuracy difference".

---

## 10. `last_reset_reason` is `None` (usually correct)

The boot banner **is sent once, and only after a real MCU reset** (`banner_sent` is static
on the firmware side), and `CMD_RESET` does **not** make it send again.

⇒ **Getting `None` in normal use is correct behaviour**; it is not "the signature failed
to parse". It only has a value in the one case where you **connect soon after a real
reset** (`"normal"` / `"iwdg-rst"`).

⚠ A closely related point, for clarity: **`reset()` is a software state reset, not an MCU
restart** (it ends up in `ctrl_reset()`, and there is no `NVIC_SystemReset` anywhere in
the tree) — so it does **not** trigger USB re-enumeration, **the same `Arm` object remains
usable afterwards**, and the banner is not sent again either.

---

## 11. `kin_bench`'s five counters read zero — silently

When `arm.diag.kin_bench()`'s link diagnostic counters (`crc_errors` / `reply_dropped` /
`can_tx_fail` …) read **all 0**, that is **not necessarily "a clean link"**: the
firmware's acknowledgement is **two consecutive frames**, and this package only takes one.
The symptom is **silent** — it raises nothing, it just gives you a zero that looks
perfectly healthy.

⇒ Before using `kin_bench` for a link health check, first confirm it **actually read
something** (look at `Msg.hz` / `Msg.timestamp`; if both are `0.0`, this class of frame
has never arrived).

⚠ Among the counters, `crc` is live and exact; `can_tx_fail` may be abnormally large (a
cumulative value self-reported by the firmware, semantics unverified).

---

## 12. A constant Cartesian offset — check `payload_mass` first

One case was seen in the field: a **~8 mm constant offset** in the Cartesian pose,
unrelated to both the SDK and the firmware planner.

**Cause**: the device had a stale `payload_mass = 1.0` left in it (the factory default
should be `0.0`). The firmware's dynamic compensation is computed for that mass, so the
end effector is systematically off a little.

⇒ **After changing the payload, always `set_payload()`**; when an "unexplainable constant
offset" appears, first read `payload_mass` back with `get_ff_scalar(4)` and check it, and
only then suspect anything else.

---

## 13. `zero_g` keep-alive and asynchronous teardown

- **Other motion commands are refused during the keep-alive** (firmware watchdog
  semantics); **queries are not restricted**, and **e-stop / disable are exceptions**
  (they must be able to get in at any time).
- **The keep-alive is re-sent automatically by an SDK background thread every `period`**
  (default `0.04 s`). Firmware `0x06` carries its own `watchdog_kick`; **stop re-sending
  for 0.10 s and it drops out of fail-soft** ⇒ `period` must be ∈ `[0.005, 0.10)`, and
  `period=0.5` is refused locally.
- **Teardown is asynchronous**: after `zero_g_stop()` returns, the firmware side still
  needs a little time to really finish. A motion command sent immediately afterwards may
  be refused — the conservative move is to wait a moment before moving.
- If the keep-alive is interrupted by a **write failure**, teardown **raises** instead of
  going silent.

---

## 14. Watchdog fail-soft on `move_js` / `send_mit`

These three are **continuous servo / pass-through** entry points; they **bypass motion
planning**, and **the caller must do its own keep-alive**:

- **Must be resent at ≥10 Hz.** After the 0.1 s command watchdog expires, the firmware
  enters fail-soft (reduced stiffness + τ=0) and the arm slowly sags under gravity — the
  measured sag matches the estimate for "stiffness × 0.6 + τ=0".
- The arrays passed to `send_mit` / `send_mit_all` must be **finite numbers**; `NaN` is
  refused locally / by the firmware.
- `move_js`'s `dq` is a **velocity reference**, not a limit.

⚠ These entry points are **not fully verified** on real hardware (the unverified list in
§17).

---

## 15. After `enter_dfu()`

- **You cannot flash immediately**: `ACK{0x15}` only means "registered"; the device has to
  **re-enumerate** as `0483:DF11`. Running pyocd straight away fails — re-running after
  a dozen-odd seconds succeeds.
- To decide that "the device really is gone", use **a read/write that raises**, **not**
  `is_open`, and **still less** "reading 0 bytes".
- After it returns successfully, **this `Arm` can no longer be used**: every entry point
  raises `ArmIsInDfuError` (`close()` excepted). Once the firmware is flashed, **create a
  new `Arm`**.
- Calling it while enabled is **refused locally** (the jump stops TIM3 ⇒ the motors
  release within 100 ms, and any payload sags).

---

## 16. Two counter-intuitive parameter writes

**`set_speed(percent)` is non-linear, and there is an integer trap.**
100 → 50 is only **1.48×** slower (there is fixed overhead), not 2×.
The argument must be an **`int` in 0..100**: `bool` and out-of-range values are refused.

**`set_joint_limits()` is not idempotent.**
Writing back the **current value** is judged by the firmware as a "loosening request" and
refused (`ERR[23,2]`) — the firmware **only permits narrowing**.
⇒ Do not use it for a "read it out and write it back" round-trip check.

## Explicitly Not Verified

Do not pretend these have been verified:

- `enter_dfu()` — the **only terminal-state operation**; recovery means re-flashing the
  firmware
- The persistence of `save_params()` (it writes flash)
- `reset_factory()` (it erases the tuned parameters and the flash)
- `activate()` — needs a vendor-issued `mac`; only the `license()` read path was verified
- The **success** path of `move_c()` — no reliably successful arc was ever constructed
- `move_js` / `send_mit` / `send_mit_all` — never run on real hardware
- An **effective narrowing** via `set_joint_limits()` (only "writing back the original
  value is refused" was verified)
- **The Windows platform has never been run, not once**

### ⚠ Irreversible commands: never run these on a calibrated arm

All four of these **overwrite/erase the per-unit identified dynamics model of that
device** (whole-sector erase + write of current RAM), with **no undo**:

| Command | Entry point |
| --- | --- |
| `0x25` | `save_params()` |
| `0x32` | `arm.model.commit()` |
| `0x36` | `arm.params.reset_factory()` |
| `0x37` | `arm.model.revert()` |

**The only way to unlock this: do it on a board that has no calibration value.**

⚠ Separately, **`0x33` `model.set_jm()` is best never called** — it changes the joint
mapping (signs included), a mistake there carries a risk of the arm **flying wildly**, and
the only recovery means on this machine (`revert` / `save_params`) happen to be exactly
the ones in the table above ⇒ **there is no fallback you can depend on**.

⚠ When stress-testing the CAN link, **only run `candump` (read-only), never `cangen`** —
`can0` is the motor bus.
