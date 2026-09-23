# litearm-python field troubleshooting

Every entry follows the same three beats: **symptom → cause → what to do**. Start with
the lookup table below.

Suggested order of investigation: check the **error code** first (§4 warns about two
code spaces that are easy to confuse), then the **link diagnostics counters** (§11), and
only then suspect the cabling.

## Symptom lookup

| What you see | Section |
| --- | --- |
| `connect()` cannot open the port / no device found | §1 |
| `connect()` reports a firmware version mismatch | §1 |
| `enable()` is rejected | §2 |
| Commands in a child process time out; a retry "sometimes works" | §3 |
| Readings in a child process never change, but nothing raises | §3 |
| You got `ERR{0x02,0x03}` and cannot tell which meaning applies | §4 |
| The number "3" means two different things in two places | §4 |
| Back-to-back Cartesian commands lose a reply | §5 |
| `move_c()` arc fails | §6 |
| Cartesian accuracy is far from what you expected | §7 |
| A constant Cartesian offset you cannot explain | §8 |
| Reading state right after `movej` returns shows a small residual | §9 |
| `last_reset_reason` is `None` | §10 |
| Every `kin_bench` counter reads 0 | §11 |
| A motion command right after `zero_g` is rejected | §12 |
| After `move_js` / `send_mit` the arm slowly sags | §13 |
| Flashing fails right after `enter_dfu()` | §14 |
| `set_speed` / `set_joint_limits` behave counter-intuitively | §15 |
| You want to know what has not been verified yet | §16 |

---

## 1. `connect()` cannot connect

**Symptom**: `TransportError` (cannot open the port / no device) or `FirmwareMismatchError`
(version mismatch).

| Cause | How to confirm |
| --- | --- |
| No device found — not plugged in, no driver, not `1d50:606f` | Check `lsusb`; call `litearm.find_cdc_port()` on its own and see what it returns |
| Port already held — another process or session is still open | On Linux, `fuser /dev/ttyACM0`; on Windows, port exclusivity is enforced by the OS, so if you cannot grab it you cannot open it |
| Firmware too old (below 1.5.0) or non-conforming name | Read the `FirmwareMismatchError` message — it quotes the version string it actually saw |

**What to do**: for `FirmwareMismatchError`, flash `Litearm1.5.0` or later. For a busy
port, shut down whatever is holding it.

⚠ Device **re-enumeration** (unplug/replug, after `enter_dfu()`, a real power cycle) makes
`/dev/ttyACM*` **change number**. A script pinned to `LITEARM_PORT` now points at a port
that does not exist — the most common reason for "it worked a minute ago".

---

## 2. `enable()` is rejected

**Symptom**: `enable(attempts=12)` raises `CommandRejectedError`.

**Cause and handling**: `attempts` retries on a **whitelist** — only the one code marked
"retryable" below is retried. For every other code, resending **does nothing** but waste time.

| Code | Meaning | What to do |
| --- | --- | --- |
| `ERR{0x10,0x03}` | Transient failure (the only retryable one in the firmware whitelist) | Let `attempts` handle it, or resend later |
| `ERR{0x10,0x06}` | A **latched** fault — resending is useless | Call `reset()` first, then find which axis (`state.joint_fault` / `state.fault_axes`) |
| `ERR{0x10,0x07}` | Resending is useless | Consult the firmware code table |
| `ERR{0x10,0x00}` | **The firmware does not have this command** | Firmware too old — update it |

⚠ A close cousin that often gets mixed in: **with the arm not enabled, `movej` is rejected
with `ERR[01,3]`**, and its message points at **two** possibilities at once — "not enabled,
**or** EMERGENCY latched". Do not read only the first half.

---

## 3. Strange behaviour in a forked child

**Cause**: threads are not copied by `fork`, but file descriptors are. So in the child,
commands really do go out on the wire, while the parent's read thread consumes the replies.
See [README](README.md#multiprocessing-a-forked-child-must-not-use-an-inherited-arm).

**How to recognise it**:

| Symptom | Explanation |
| --- | --- |
| A command "times out", but a retry "sometimes works" | The command did go out; the reply was read by the parent ⇒ the retry is a **duplicate command** |
| `get_state()` does not raise, but the numbers **never change** | It silently returns the inherited, **stale** value — the subtlest case |
| `connect()` raises in the child | The parent still holds the port ⇒ the parent must `close()` first |
| Any command immediately raises `ForkedSessionError` | ✅ The guard is **working**, not failing |

**What to do**: `close()` the parent session to release the port, then `fork`, then
**create** a new `Arm` inside the child.

---

## 4. Two confusing error codes

### `ERR{0x02,0x03}` is ambiguous

**The same `(command, code)` pair has two entirely different origins** in the firmware: it
means both "not enabled / EMERGENCY latched" and "inverse kinematics unreachable / invalid
solution".

**What to do**: receiving it does **not** mean the arm is disabled. Look at the **current
state** (`state.enabled` / `state.mode`) rather than the code — if the arm is enabled, it is
the IK branch.

### Two code spaces that both run 1–6

| Origin | Meaning |
| --- | --- |
| The `err` field in the `0x4E` reply (`CartPlan.err`) | The **planning result itself**: unreachable / collinear / over capacity / out of range |
| The second byte of `RSP_ERR` | The **gate reason code**: not enabled `0x03` / hand-guiding `0x04` / `drop_hold` `0x06` |

**Both take values 1–6 and the meanings are unrelated.** When you get a "3", first ask which
route it came from.

---

## 5. Back-to-back Cartesian commands lose a reply

**Symptom**: after firing several `move_l` / `move_path` in a row, one raises
`CartReplyLostError` ("outcome unknown"), or subsequent replies are **shifted** (the answer
to one command is collected by the next). Reproduced 3 out of 3 times on hardware.

**Cause**: the firmware holds only **one** pending Cartesian plan at a time — it is not a
queue. Cancellation replies from superseded plans overwrite each other, so one missing reply
misaligns the whole pairing sequence. **The root cause is in the firmware, not this library.**

**What to do**: send Cartesian commands **serially** — wait for each to finish before sending
the next. This library already serialises calls **within one process**, so normal usage never
hits this; concurrent use across processes or clients does.

---

## 6. `move_c()` arc fails

**Symptom**: `CartesianPlanError` is raised and the arm has not moved at all.

| `err` | Cause |
| --- | --- |
| `2` | **Three collinear points** (or nearly so) — the circle centre runs off to infinity |
| `1` | No IK solution. Starting from a **fully extended `home` pose** (a singularity) **always** does this; it is the firmware behaving sensibly, not a defect |
| `3` | Over capacity / unreachable |

**What to do**: pick three points that are not collinear; when starting from a singular pose,
leave the singularity first.

⚠ In `move_c(start, via, goal)` the **`start` must match the measured TCP at call time**
(tolerance 6 mm / 0.03 rad). It is not a free "start from here" parameter — it is
**validated against reality**, so a mid-motion TCP as the start point is rejected.
⚠ The **orientation of `via` is ignored**; only its position defines the circle.

---

## 7. Cartesian accuracy is not a single number

**The residual end-point error depends on [distance × speed × payload pose]**, not on a fixed
device specification, and it changes markedly with payload.

**What to do**: any accuracy comparison must use **the same pose, the same convention and the
same payload** — otherwise you are measuring a pose or payload difference, not an
accuracy difference.

---

## 8. Constant Cartesian offset — check `payload_mass` first

**Symptom**: the Cartesian pose carries a constant offset of about 8 mm, unrelated to the
protocol or the planner.

**Cause**: a stale `payload_mass = 1.0` was left in the device (the factory default should be
`0.0`). The firmware's dynamics compensation uses that mass, so the tool ends up
systematically offset.

**What to do**: always call `set_payload()` after changing the payload. When you see an
unexplained constant offset, read `payload_mass` back with `get_ff_scalar(4)` before
suspecting anything else.

---

## 9. `movej` has not settled when it returns

**Symptom**: reading state immediately after `movej` returns shows a small residual on
each axis.

**Cause**: `movej` returns as soon as the **arrival criterion** is met — every axis within
`q_tol` of the target, with velocity quiet for `arrive_frames` consecutive frames. The arm has
not come to rest at that instant.

Measured (moving J6 to 0.7): **0.6884** at the moment it returns, converging on its own to
**0.6991** within 5 s and holding there for 20 s. That is about 0.012 rad (0.7°), inside
`q_tol = 0.03` — **by design, not drift**.

**What to do**: wait a few seconds before reading if you need the exact value, or tighten
`q_tol` yourself.

---

## 10. `last_reset_reason` is `None`

**This is correct behaviour, not a parsing failure.** The boot signature is sent **once, only
after a real MCU reset**, and `reset()` does not make it repeat. Getting `None` in normal use
is expected.

It only has a value when you connect **soon after a real reset** (`"normal"` / `"iwdg-rst"`).

⚠ A related clarification: **`reset()` is a software state reset, not an MCU reboot.** It does
**not** trigger USB re-enumeration, **the same `Arm` object keeps working afterwards**, and the
signature is not resent.

---

## 11. Every `kin_bench` counter reads 0

**Symptom**: the link diagnostics counters from `arm.diag.kin_bench()` (`crc_errors` /
`reply_dropped` / `can_tx_fail` …) all read 0.

**Cause**: this is **not necessarily a clean link**. All-zero can mean "nothing was actually
read" — and that failure is **silent**: no error, just a healthy-looking 0.

**What to do**: before using it as a link health check, confirm frames are really arriving —
look at `Msg.hz` / `Msg.timestamp`. If both are `0.0`, no frame of that kind has ever arrived.

⚠ Among the counters, `crc` is live and exact; `can_tx_fail` can be surprisingly large (a
firmware-reported cumulative value whose exact definition has not been verified).

---

## 12. `zero_g` keep-alive period and asynchronous exit

| Symptom | Cause | What to do |
| --- | --- | --- |
| Motion commands rejected during hand-guiding | Firmware watchdog semantics: only queries pass through | Queries are unaffected; **emergency stop / disable are exceptions** and always get through |
| `period=0.5` rejected locally | The keep-alive must be **under 0.10 s**; the firmware drops out of fail-soft if not resent within 0.10 s | Use `period` in `[0.005, 0.10)`; the default `0.04` is fine |
| A motion command right after `zero_g_stop()` is rejected | **The exit is asynchronous** — the firmware needs a moment to wrap up after the call returns | Wait briefly before sending motion commands |

Keep-alive is resent automatically by a background thread. If it breaks because of a **write
failure**, exiting **raises** rather than failing silently. State is available via the
read-only `zero_g_active` / `zero_g_error`.

---

## 13. The arm slowly sags after `move_js` / `send_mit`

**Cause**: these are **continuous servo / passthrough** entry points that **bypass motion
planning**, and **the caller must keep them alive**: **resend at ≥10 Hz**. After the 0.1 s
watchdog expires the firmware enters fail-soft (reduced stiffness + τ=0) and the arm sags
slowly under gravity — the measured sag matches a "0.6× stiffness + τ=0" estimate.

**What to do**: keep resending at ≥10 Hz for as long as the motion is needed.

⚠ Arrays must be **length `n`** (checked locally) and **finite** — a `NaN` / `Inf` makes the
firmware reject the whole frame (`ERR{cmd,0x02}`). `send_mit`, `send_mit_all` and `move_js` all
get this check.
⚠ In `move_js`, `dq` is a **velocity reference**, not a limit.

---

## 14. After `enter_dfu()`

- **You cannot flash immediately**: `ACK{0x15}` only means "registered"; the device has to
  **re-enumerate** as `0483:DF11`. Running pyocd right away fails; retry after ten-odd seconds
  and it succeeds.
- To tell that the device has really gone, use a **read or write raising an error** — **not**
  `is_open`, and **not** "we read 0 bytes".
- After it returns successfully, **this `Arm` is unusable**: every entry point raises
  `ArmIsInDfuError` (`close()` excepted). After flashing, **create a new `Arm`**.
- Calling it while enabled is **rejected locally** (the jump stops TIM3 ⇒ the motors release
  within 100 ms and sag under load).

---

## 15. Two counter-intuitive parameter behaviours

**`set_speed(percent)` takes an integer percentage, not a multiplier.**
`set_speed(1)` means **1% speed** — calling it with 0..1 thinking gives you a crawling arm. It
is also **global and persistent** (it stays in effect until a reset-semantics call), not the
same thing as the per-trajectory factor in `movej(speed=0..1)`. The argument must be an **`int`
in 0..100**; `bool` and out-of-range values are rejected locally.

**`set_joint_limits()` is not idempotent.**
The firmware **only allows narrowing**, so writing the **current** values back is judged a
"widening request" and rejected (`ERR[23,2]`). ⇒ Do not use it for a read-modify-write
round-trip check.

---

## 16. Not yet verified

Do not assume any of the following has been verified:

- `enter_dfu()` — the only terminal-state operation; recovering means reflashing the firmware;
- `save_params()` persistence (it writes flash);
- `reset_factory()` (it wipes tuned parameters);
- The **success** path of `move_c()` — no reliably successful arc was constructed;
- `move_js` / `send_mit` / `send_mit_all` — never run on hardware;
- The **effective narrowing** of `set_joint_limits()` (only "writing the old value back is
  rejected" was verified);
- Whether **`capture()` always records 0 ticks with the arm disabled** — this appears only in
  this repo's field notes, with no matching check or test in the code; not re-verified;
- **Windows** — not yet verified.

### ⚠ Irreversible commands: do not run these on a calibrated arm

All four **overwrite or erase that unit's per-arm identified dynamics model**, and **there is
no undo**:

| Command | Entry point |
| --- | --- |
| `0x25` | `save_params()` |
| `0x32` | `arm.model.commit()` |
| `0x36` | `arm.params.reset_factory()` |
| `0x37` | `arm.model.revert()` |

**The only way to be safe: do it on a board whose calibration has no value.**

⚠ Separately, `0x33` `model.set_jm()` **should never be called** — it rewrites the joint
mapping (including signs), a mistake there can make the arm **flail**, and the only local
recovery options (`revert` / `save_params`) are both in the table above ⇒ **there is no
reliable way back**.

⚠ When stress-testing the CAN link, run **`candump` (read-only) only — never `cangen`**:
`can0` *is* the motor bus.
