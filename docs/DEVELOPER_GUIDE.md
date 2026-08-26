# litearm-python Developer Guide & API Reference

`litearm-python` is the Python client library for the LiteArm robotic arm
(`__version__ = "0.1.0"`). Connect to the arm control service over the network and
control the arm from any ordinary computer. **No hardware dependencies and no
numpy on the client** — run it on any machine, not necessarily the arm controller.

```text
Your program ──→ Arm control service ──→ Arm / CAN
```

---

## Table of Contents

- [1. Requirements & Installation](#1-requirements--installation)
- [2. Quick Start](#2-quick-start)
- [3. Connection Management](#3-connection-management)
- [4. API Reference](#4-api-reference)
  - [4.1 Computation (no motors driven)](#41-computation-no-motors-driven)
  - [4.2 Motion Control](#42-motion-control)
  - [4.3 State Reading](#43-state-reading)
  - [4.4 Emergency Stop / Enable](#44-emergency-stop--enable)
  - [4.5 DIRECT Mode — Per-frame MIT Direct Control](#45-direct-mode--per-frame-mit-direct-control)
  - [4.6 Parameters](#46-parameters)
  - [4.7 Peripheral Devices](#47-peripheral-devices)
  - [4.8 System / Settings](#48-system--settings)
  - [4.9 Trajectory Management](#49-trajectory-management-server-side-recording--management)
  - [4.10 End-Effector Device Management](#410-end-effector-device-management)
  - [4.11 Teleop (master / slave arms)](#411-teleop-master--slave-arms)
  - [4.12 CAN Tunnel (Advanced)](#412-can-tunnel-advanced)
- [5. Exceptions](#5-exceptions)
- [6. Safety Notes](#6-safety-notes)
- [7. FAQ](#7-faq)

## 1. Requirements & Installation

| Item | Requirement |
|---|---|
| Python | 3.10+ |
| Dependencies | Installed automatically |

```bash
pip install litearm-python          # release install
pip install -e .                    # development install (from the source directory)
```

## 2. Quick Start

```python
import litearm

with litearm.Arm(endpoint="tcp/192.168.1.100:7447") as arm:
    state = arm.get_state()          # read current state: q/dq/tau/fault/...
    arm.movej([0.0] * 7, speed=0.5)  # joint-space motion
    arm.home(speed=0.3)               # home all joints to zero (bypasses limit checks)

    hand = arm.device("hand_0")      # end-effector peripheral: dexterous hand
    hand.open()
    hand.set_gesture("pinch")
```

## 3. Connection Management

```python
arm = litearm.Arm(endpoint="tcp/127.0.0.1:7447", arm_id="armA")
# endpoint     : address of the arm control service, e.g. "tcp/192.168.1.100:7447"
# arm_id       : arm identifier, default "armA"; must match the server-side setting
# query_timeout: per-call timeout in seconds (default ~11.5 days; lower as needed)
arm.close()                          # close the connection
```

- A context manager is supported: `with litearm.Arm(...) as arm:` — `close()` is
  called automatically on exit.
- `get_state()` synchronously reads the latest state cache pushed by the service;
  returns `None` before the first update.

## 4. API Reference

> Conventions: poses are plain lists. `pose = [position, rotation]`,
> `position = [px, py, pz]`, `rotation = 3×3 row-major rotation matrix`.
> Motion methods (`movej`/`movel`/...) return `bool`; pure-computation methods return data.

### 4.1 Computation (no motors driven)

| Method | Description |
|---|---|
| `fk(q)` | Forward kinematics → `(position, rotation matrix)` |
| `ik(pos_d, R_d, q_seed=None)` | Inverse kinematics → `(q, success)` |
| `plan_movel(q_start, pose_goal)` | Cartesian line path planning → joint path |
| `plan_movec(q_start, pose_via, pose_goal)` | Circular-arc path planning (via a waypoint) |
| `plan_movep(q_start, poses_goal)` | Multi-waypoint path planning |

### 4.2 Motion Control

| Method | Description |
|---|---|
| `movej(q_target, speed=1.0, settle_s=1.0, max_cycles=None, allow_start_collision_recovery=False)` | Joint-space point-to-point |
| `home(speed=0.3, settle_s=0.5, max_cycles=None)` | Home all joints to zero — bypasses joint-limit and self-collision path checks |
| `recover_joint_limits(speed=0.05, settle_s=0.5, max_cycles=None, inset_rad=0.0)` | Slowly return out-of-limit joints to the safe boundary (requires server `allow_limit_recovery=True`) |
| `movel(pose_goal, speed=1.0, settle_s=0.8, max_cycles=None)` | Cartesian line move |
| `movec(pose_via, pose_goal, speed=1.0, settle_s=0.8, max_cycles=None)` | Circular arc move |
| `movep(poses_goal, speed=1.0, settle_s=0.8, max_cycles=None)` | Multi-waypoint move with corner blending |
| `replay_joint_path(q_path, speed=1.0, settle_s=0.5, goto_start=True, goto_speed=0.3, max_cycles=None)` | Replay a joint sequence |
| `replay_trajectory(traj_q, speed=1.0, goto_start=True, goto_speed=0.3, max_cycles=None, check_singularity=True)` | Replay a recorded trajectory (JointTrajectory or dict) |
| `replay_timed_trajectory(traj_q, traj_t, speed=1.0, goto_start=True, goto_speed=0.3, simplify_tolerance_rad=0.01, max_cycles=None)` | Replay on the original time axis (auto-stretch for safety) |
| `play_trajectory(trajectory, speed=1.0, goto_start=True, goto_speed=0.3, verify_robot=True, simplify_tolerance_rad=0.01, max_cycles=None)` | Replay a saved trajectory (object or server-side path string) |
| `record_trajectory(output="trajectories", duration_s=None, sample_rate_hz=100.0, filter_alpha=0.15, name=None)` | Record by drag → `JointTrajectory` |
| `hold(kp_scale=3.0, max_cycles=None)` | Hold with higher stiffness |
| `zero_gravity(max_cycles=None, duration_s=None, measured_overspeed_factor=None, vel_max=None)` | Zero-gravity (free-drag) mode |
| `joint_impedance(q_des, K, B, tau_max=None, engage_sec=0.3, max_cycles=None)` | Joint-space impedance control |
| `cartesian_impedance(q_des, K_cart, B_cart, v_des=None, tau_max=None, engage_sec=0.3, max_cycles=None, sigma_min_thresh=None, max_ori_err=None, measured_overspeed_factor=None, vel_max=None)` | Cartesian impedance control |
| `joint_follow(K=None, B=None, speed_limit=None, accel_limit=None, engage_sec=0.3, max_cycles=None, duration_s=None)` | Follow an external target |

### 4.3 State Reading

| Method | Description |
|---|---|
| `get_state(refresh=False)` | Latest cached state (sync); `refresh=True` forces a pull |
| `get_tcp_pose()` | Current TCP pose → `(position, rotation matrix)` |

### 4.4 Emergency Stop / Enable

| Method | Description |
|---|---|
| `request_stop()` | High-priority emergency stop (independent channel) |
| `clear_stop()` | Clear the stop condition and return to ready |
| `enable()` | Enable all motors and lock the current pose |
| `disable()` | ⚠️ Disables all motors (the arm drops under gravity!), CAN stays connected |
| `clear_faults()` | Clear motor faults → `[(motor_id, fault_code), ...]` |

### 4.5 DIRECT Mode — Per-frame MIT Direct Control

> DIRECT mode is LiteArm's per-frame MIT direct control channel. Send five-parameter
> (kp/kd/q_ref/dq_ref/tau_ff) commands at a typical 250Hz to control joint motors in real-time,
> with 4 never-disableable core safety guardrails built in.

**Comparison with ordinary motion control:**

| Feature | Ordinary motion control (`movej` etc.) | DIRECT mode (`send_mit`) |
| --- | --- | --- |
| Control method | Target position + velocity, auto-planned | Per-frame 5-parameter MIT command |
| Frame rate | One call, auto-execution | User loop control (typical 250Hz) |
| Blocking | Blocking, waits for completion | Non-blocking, returns immediately |
| Trajectory | Auto-planned + interpolated | User-generated |
| Guardrails | Built-in limits | 4 core + 3 optional guards |

**Entry and exit:**

- **Entry**: Automatically enters `ArmState.DIRECT` on the first `send_mit` call
- **Exit** (three ways):
  1. `request_stop()` — proactive exit, arm holds position with low-stiffness PD
  2. Watchdog timeout — command stream interrupted beyond `watchdog_timeout`, auto-hold
  3. Motor fault — auto-exit and hold

#### send_mit — Send MIT Control Frame

**Description:** Async publish a five-parameter MIT control frame to the arm command channel.
Non-blocking, returns immediately. First call auto-enters DIRECT mode. Frame rate is user-controlled.

**Function Definition:**

```python
def send_mit(
    self,
    kp: list[float],       # length 7, position stiffness
    kd: list[float],       # length 7, velocity damping
    q_ref: list[float],    # length 7, target joint angles (rad)
    dq_ref: list[float],   # length 7, target angular velocity (rad/s)
    tau_ff: list[float],   # length 7, feedforward torque (N·m)
) -> None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `kp` | `list[float]` | Position stiffness, length 7, range `[0, 500]`. Higher = tighter tracking. Typical: 15–200 |
| `kd` | `list[float]` | Velocity damping, length 7, range `[0, 5]`. Suppresses oscillation. Typical: 0.5–3.0 |
| `q_ref` | `list[float]` | Target joint angles (rad), length 7. Inter-frame jumps are slew-limited |
| `dq_ref` | `list[float]` | Target angular velocity (rad/s), length 7. Clamped to `±DQ_MAX` |
| `tau_ff` | `list[float]` | Feedforward torque (N·m), length 7. Clamped to `±min(guards_tau_max, TAU_MAX)` |

**Return Value:** `None` — async send, no acknowledgment.

**Usage Example:**

```python
import time
import litearm

arm = litearm.Arm(endpoint="tcp/192.168.1.100:7447")

# Single frame (auto-enters DIRECT mode)
arm.send_mit(
    kp=[50.0] * 7,
    kd=[1.5] * 7,
    q_ref=[0.0] * 7,
    dq_ref=[0.0] * 7,
    tau_ff=[0.0] * 7,
)
```

#### set_guards — Configure Global Guardrails

**Description:** One-time global guardrail configuration. All parameters are keyword-only,
`None` = no change. Numeric params are clamped to physical limits. **Globally persistent**:
not reset on DIRECT exit, re-applied on re-entry.

**Function Definition:**

```python
def set_guards(
    self,
    *,
    slew_limit: float | None = None,
    tau_max: float | None = None,
    watchdog_timeout: float | None = None,
    position_bounds: bool | None = None,
    velocity_bounds: bool | None = None,
    jerk_limit: bool | None = None,
) -> Any
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `slew_limit` | `float` or `None` | Global slew rate limit (rad/s). `None` = no change. Clamped to `(0, min(DQ_MAX)]` |
| `tau_max` | `float` or `None` | Global torque limit (N·m). `None` = no change. Clamped to `(0, min(TAU_MAX)]` |
| `watchdog_timeout` | `float` or `None` | Watchdog timeout (s), range `[0.05, 2.0]`. `None` = no change |
| `position_bounds` | `bool` or `None` | Enable position soft-limit clamping. `None` = no change. Default `False` |
| `velocity_bounds` | `bool` or `None` | Enable velocity soft-limit clamping. `None` = no change. Default `False` |
| `jerk_limit` | `bool` or `None` | Enable jerk limiting. `None` = no change. Default `False` |

**Return Value:** RPC reply object (usually ignored).

**Usage Example:**

```python
# Tighten slew limit (conservative mode, max single-frame jump ≈ 0.05 rad)
arm.set_guards(slew_limit=0.5)

# Tighten watchdog (required for high-frequency control, ≥20Hz frame rate)
arm.set_guards(watchdog_timeout=0.05)

# Enable position soft-limits
arm.set_guards(position_bounds=True)

# Combined configuration
arm.set_guards(
    slew_limit=1.0,
    tau_max=10.0,
    watchdog_timeout=0.10,
    position_bounds=True,
)
```

#### get_guards — Read Current Guardrail Configuration

**Description:** Read the currently active guardrail configuration (RPC, synchronous).

**Function Definition:**

```python
def get_guards(self) -> dict[str, Any]
```

**Return Value:**

```json
{
    "slew_limit": 1.0,
    "tau_max": 10.0,
    "watchdog_timeout": 0.10,
    "position_bounds": true,
    "velocity_bounds": false,
    "jerk_limit": false
}
```

**Usage Example:**

```python
guards = arm.get_guards()
print(f"slew_limit = {guards['slew_limit']} rad/s")
```

#### Full Control Loop Example

```python
"""DIRECT mode 250Hz control loop — sine wave on joint 1."""
import math
import time
import litearm

arm = litearm.Arm(endpoint="tcp/192.168.1.100:7447")

# Configure guardrails (one-time)
arm.set_guards(slew_limit=2.0, tau_max=20.0, watchdog_timeout=0.10, position_bounds=True)

DT = 0.004      # 4ms → 250Hz
FREQ = 0.5      # sine frequency (Hz)
AMP = 0.5       # amplitude (rad)

t = 0.0
try:
    while True:
        loop_start = time.perf_counter()

        # Joint 1 sine, others hold at zero
        q_ref = [AMP * math.sin(2 * math.pi * FREQ * t)] + [0.0] * 6

        arm.send_mit(
            kp=[50.0] * 7, kd=[1.5] * 7,
            q_ref=q_ref, dq_ref=[0.0] * 7, tau_ff=[0.0] * 7,
        )

        t += DT
        elapsed = time.perf_counter() - loop_start
        if elapsed < DT:
            time.sleep(DT - elapsed)

except KeyboardInterrupt:
    arm.request_stop()
    print("DIRECT mode exited")
```

#### Safety Guardrails

| Guardrail | Description | Disableable? |
| --- | --- | --- |
| **Guard 1: Protocol param clamp** | kp≤500, kd≤5, dq_ref≤DQ_MAX, tau_ff≤TAU_MAX | Never |
| **Guard 2: Command slew limit** | Inter-frame q_ref jump ≤ `min(slew_limit, rated) × dt`, dt capped at 0.10s | Never; `slew_limit` can be tightened |
| **Guard 3: Watchdog fail-soft** | Command interruption timeout → auto-hold (low-stiffness PD, kp=35, kd=1.2) | Never; `watchdog_timeout` adjustable |
| **Guard 4: Single ownership** | Rejects motion commands and teleop while DIRECT active; server-side single-client session | Never |
| **Position bounds (optional)** | q_ref clamped to joint soft-limits `[q_min, q_max]` per frame | Off by default; `position_bounds=True` |
| **Velocity bounds (optional)** | dq_ref clamped to `±DQ_MAX` per frame | Off by default; `velocity_bounds=True` |
| **Jerk limit (optional)** | dq_ref change rate limited per frame | Off by default; `jerk_limit=True` |

> **Safety bottom line: The arm must never fly away.**
> Command slew limit + single ownership + watchdog fail-soft + firmware fallback
> are the acceptance prerequisites for all safety changes.

### 4.6 Parameters

| Method | Description |
|---|---|
| `set_gains(kp=None, kd=None)` / `get_gains()` | Get/set PD gains |
| `set_payload(mass, com=(0,0,0))` / `get_payload()` | End-effector payload (mass + center of mass) |
| `set_installation(base_rpy=None, gravity=None)` / `get_installation()` | Mounting orientation (base RPY or gravity vector) |
| `get_joint_limits()` / `set_joint_limits(limits)` | Joint limits |
| `get_zero_offsets()` / `set_zero_offsets(offsets)` | Zero offsets |
| `get_end_effector()` / `set_end_effector(config)` | End-effector configuration |
| `get_cartesian_limits()` / `set_cartesian_limits(limits)` | Cartesian limits |
| `get_collision_config()` / `set_collision_config(config)` | Collision configuration |

### 4.7 Peripheral Devices

Unified entry `arm.device(device_id)`; methods route to the device's
`device.{device_id}.{method}` interface.

```python
hand = arm.device("hand_0")
hand.open(); hand.close()                    # open / close
hand.set_force(force)                        # grip force
hand.get_state(); hand.list_gestures()       # state / supported gestures
hand.set_gesture("pinch")                    # gesture
hand.finger_move(pose); hand.set_speed(speed); hand.set_torque(torque)  # per-finger

gripper = arm.device("gripper_0")
gripper.set_width(0.5); w = gripper.get_width()

teach = arm.device("teach_0")
teach.get_joints(); teach.get_buttons()

# Common: get_status / get_info / connect / disconnect / clear_faults
```

- Device manager: `arm.devices["hand_0"]` is equivalent to `arm.device("hand_0")`
  (lazily created).
- Backward-compatible convenience attribute: `arm.hand.open()` equals
  `arm.device("hand_0").open()`.

### 4.8 System / Settings

| Method | Description |
|---|---|
| `get_system_stats()` | CPU / memory / board temperature / uptime |
| `get_logs(page=1, size=50, search="")` | Paginated logs |
| `restart_service()` | Restart the arm service |
| `reconnect()` | Hardware reconnect — re-initialize motors from any state after arm hot-restart |

### 4.9 Trajectory Management (server-side recording & management)

```python
arm.start_recording();  arm.get_recording_state();  arm.stop_recording();  arm.discard_recording()
arm.list_trajectories()
arm.save_trajectory("t1", "demo", points, duration=None)
arm.delete_trajectory("t1")
arm.get_playback_state()
```

### 4.10 End-Effector Device Management

```python
arm.list_device_types()
arm.connect_device(category="hand", subtype="lite6_hand", device_id="end_0", can_iface="", config=None)
arm.get_active_device(device_id="end_0")
arm.disconnect_device(device_id="end_0")
```

### 4.11 Teleop (master / slave arms)

```python
arm.enter_teleop("master")                                    # this arm acts as master
arm.enter_teleop("slave", peer="tcp/10.0.0.2:7447")           # follow a master
arm.get_teleop_status()
arm.exit_teleop()
```

> In teleop mode the service rejects all manual-control commands; only read-only,
> emergency-stop, and `exit_teleop` calls are allowed.

### 4.12 CAN Tunnel (Advanced)

For when you need to use vendor CAN protocols directly on the local machine:

```python
from litearm.can_bridge import RemoteCAN

can = RemoteCAN("tcp/127.0.0.1:7447", vcan_iface="vcan0")
can.start()        # bridge the controller's can0 to the local vcan0
# ... send/receive with the vendor SDK on the local vcan0 ...
can.stop()
```

## 5. Exceptions

All exceptions inherit from `LiteArmError` (a `RuntimeError` subclass). Exceptions
raised on the server are re-thrown on the client with the same type.

```python
from litearm import LiteArmError, SafetyViolationError

try:
    arm.movej([0.0] * 7)
except SafetyViolationError as e:      # safety violations (timeout/follow/fault/watchdog)
    print(e.details)
except LiteArmError as e:              # fallback
    print(e)
```

Common types (selection): `NotConnectedError`, `ConfigurationError`,
`InvalidCommandError`, `CartesianPlanError`, `MotionTimeoutError`,
`MotorFaultError`, `ArmFault`, `WatchdogError`, `MotionCancelled`.

## 6. Safety Notes

- ⚠️ `disable()` drops the arm under gravity — make sure the area is clear.
- `request_stop()` is a high-priority emergency stop; bind it to an independent
  physical e-stop channel.
- Manual-control commands are rejected during teleop.
- `recover_joint_limits` is only available when the server runs with
  `allow_limit_recovery=True`.

## 7. FAQ

| Problem | Resolution |
|---|---|
| `get_state()` returns `None` | No state yet: confirm the service is up and endpoint/arm_id are correct |
| `NotConnectedError` | Operations that need hardware: confirm the service is online first |
| Call hangs | Lower `query_timeout` or check the network / service state |
| Configuration rejected | Check the arm control service configuration |

## License

Proprietary
