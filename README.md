# litearm-python

Python client library for the LiteArm robotic arm. Install it, connect to the arm
control service, and control the arm from any ordinary computer: motion control,
state reading, peripheral devices (dexterous hand / gripper / teach pendant), and more.

## Features

- 🧩 **Pure Python**: no hardware dependencies, no numpy — runs on any ordinary computer
- 🔗 **Plug & play**: one line connects to the arm, calls execute immediately
- 🎮 **Unified device interface**: dexterous hand, gripper, and teach pendant share one access pattern
- 🚦 **High-priority emergency stop**: independent channel, safe to stop at any time
- 🌐 **Multi-language parity**: same API as [litearm-js](../litearm-js) / [litearm-cpp](../litearm-cpp) — code migrates across languages

> 📖 Full developer guide & API reference: [docs/DEVELOPER_GUIDE.md](docs/DEVELOPER_GUIDE.md).

## Installation

```bash
pip install litearm-python
```

## Quick Start

```python
import litearm

# Connect to the arm (address of the machine running the control service, default port 7447)
arm = litearm.Arm(endpoint="tcp/192.168.1.100:7447")

state = arm.get_state()               # read current state (joint angles, velocities, ...)
arm.movej([0.0] * 7, speed=0.5)       # joint-space motion
arm.home(speed=0.3)                    # home all joints to zero (bypasses limit checks)

hand = arm.device("hand_0")           # control the dexterous hand
hand.open()
hand.set_gesture("pinch")

arm.close()
```

A context manager is also supported — the connection closes automatically on exit:

```python
with litearm.Arm(endpoint="tcp/127.0.0.1:7447") as arm:
    arm.movej([0.0] * 7)
```

## Main Features

### Motion Control

Joint / line / arc / multi-waypoint motion, trajectory recording & replay, zero-gravity (free drag), impedance control, joint following:

```python
arm.movej([0.1, 0.2, 0.3, 0, 0, 0, 0])            # joint-space move
arm.movel(pose_goal)                              # line move
arm.movec(pose_via, pose_goal)                    # arc move
arm.movep([pose1, pose2, pose3])                  # multi-waypoint move

arm.replay_joint_path(q_path)                     # replay a joint path
arm.replay_trajectory(traj_q)                     # replay a recorded trajectory
arm.record_trajectory()                           # record by drag

arm.zero_gravity(duration_s=10)                   # zero-gravity (free drag)
arm.hold()                                        # hold in place
```

### State Reading

```python
state = arm.get_state()       # q / dq / tau / fault / state / ...
pos, rot = arm.get_tcp_pose() # current end-effector pose
```

### Emergency Stop / Enable

```python
arm.request_stop()            # high-priority e-stop (independent channel, safe anytime)
arm.clear_stop()              # clear the stop condition
arm.enable()                  # enable motors and hold current pose
arm.disable()                 # ⚠️ disables motors — the arm drops under gravity!
```

### Parameters

```python
arm.set_gains(kp=..., kd=...)                # PD gains
arm.set_payload(mass=1.5, com=(0, 0, 0.05))  # end-effector payload
arm.set_installation(base_rpy=[0, 0, 0])     # mounting orientation
arm.clear_faults()                           # clear motor faults
```

### Peripheral Devices

Dexterous hand, gripper, and teach pendant share one access pattern via `arm.device(...)`:

```python
hand = arm.device("hand_0")
hand.open(); hand.close()                  # open / close
hand.set_gesture("pinch")                  # gesture
hand.finger_move(pose)                     # per-finger motion

gripper = arm.device("gripper_0")
gripper.set_width(0.5)                     # gripper width
width = gripper.get_width()

teach = arm.device("teach_0")
teach.get_joints(); teach.get_buttons()    # read teach pendant values
```

### System / Settings

```python
arm.get_system_stats()                          # system info (CPU / memory / board temp)
arm.get_logs(page=1, size=50, search="movej")   # logs
arm.restart_service()                           # restart the control service

# Arm settings: joint limits / zero offsets / end effector / Cartesian limits / collision config
arm.get_joint_limits();  arm.set_joint_limits({...})
arm.get_zero_offsets();  arm.set_zero_offsets({...})
arm.get_end_effector();  arm.set_end_effector({...})
```

### Trajectory Management

```python
arm.start_recording(); arm.stop_recording()
arm.list_trajectories()
arm.save_trajectory("t1", "demo", points)
arm.delete_trajectory("t1")
arm.get_playback_state()
```

### Device Management / Teleop

```python
arm.list_device_types()
arm.connect_device(category="hand", subtype="lite6_hand", device_id="end_0")
arm.disconnect_device(device_id="end_0")

arm.enter_teleop("master")                               # this arm is the master
arm.enter_teleop("slave", peer="tcp/10.0.0.2:7447")      # follow a master
arm.get_teleop_status()
arm.exit_teleop()
```

### CAN Tunnel (Advanced)

For when you need to use vendor CAN protocols directly on the local machine:

```python
from litearm.can_bridge import RemoteCAN

can = RemoteCAN("tcp/127.0.0.1:7447", vcan_iface="vcan0")
can.start()
# exchange CAN frames locally with vendor protocol, bridged to the arm bus
can.stop()
```

## Pose Format

Poses are plain Python lists — no numpy required:

```python
pose = [position, rotation]
position = [px, py, pz]                     # 3 elements
rotation = [[r00, r01, r02],                # 3x3 row-major rotation matrix
            [r10, r11, r12],
            [r20, r21, r22]]
```

## Server Deployment

The arm control service runs on the controller (e.g., the arm's on-board computer):

```bash
python -m litearm_server --endpoint tcp/0.0.0.0:7447 --iface can0
```

The `endpoint` you pass to the client is that controller's address and port.

## Examples

See [examples/README.md](examples/README.md):

| Example | Demonstrates | Moves? |
|---|---|---|
| `01_read_state.py` | Connect + read state + TCP pose | ❌ read-only |
| `02_movej.py` | Joint-space motion movej | ✅ motion |
| `03_fk_ik.py` | Forward/inverse kinematics (pure computation) | ❌ no motion |
| `04_movel.py` | Line move movel + path planning | ✅ motion |
| `05_home.py` | Home all joints to zero — bypasses limit checks | ✅ motion |

```bash
python3 examples/01_read_state.py
python3 examples/02_movej.py --endpoint tcp/127.0.0.1:7447
```

## Development

```bash
pip install -e ".[dev]"
python -m pytest tests/ -q
```

## License

Proprietary
