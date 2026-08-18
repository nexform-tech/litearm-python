# litearm-python Developer Guide & API Reference

`litearm-python` is the Python client library for the LiteArm robotic arm
(`__version__ = "0.1.0"`). Connect to the arm control service over the network and
control the arm from any ordinary computer. **No hardware dependencies and no
numpy on the client** — run it on any machine, not necessarily the arm controller.

```text
Your program ──→ Arm control service ──→ Arm / CAN
```

---

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

### 4.5 Parameters

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

### 4.6 Peripheral Devices

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

### 4.7 System / Settings

| Method | Description |
|---|---|
| `get_system_stats()` | CPU / memory / board temperature / uptime |
| `get_logs(page=1, size=50, search="")` | Paginated logs |
| `restart_service()` | Restart the arm service |

### 4.8 Trajectory Management (server-side recording & management)

```python
arm.start_recording();  arm.get_recording_state();  arm.stop_recording();  arm.discard_recording()
arm.list_trajectories()
arm.save_trajectory("t1", "demo", points, duration=None)
arm.delete_trajectory("t1")
arm.get_playback_state()
```

### 4.9 End-Effector Device Management

```python
arm.list_device_types()
arm.connect_device(category="hand", subtype="lite6_hand", device_id="end_0", can_iface="", config=None)
arm.get_active_device(device_id="end_0")
arm.disconnect_device(device_id="end_0")
```

### 4.10 Teleop (master / slave arms)

```python
arm.enter_teleop("master")                                    # this arm acts as master
arm.enter_teleop("slave", peer="tcp/10.0.0.2:7447")           # follow a master
arm.get_teleop_status()
arm.exit_teleop()
```

> In teleop mode the service rejects all manual-control commands; only read-only,
> emergency-stop, and `exit_teleop` calls are allowed.

### 4.11 CAN Tunnel (Advanced)

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
