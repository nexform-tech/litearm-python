# litearm-python

LiteArm 机械臂 Python 客户端 SDK，通过 zenoh 远程连接 litearm-server，把所有调用转发为 RPC。客户端无硬件依赖、无 numpy、无 Pinocchio。

与 [litearm-js](../litearm-js) / [litearm-cpp](../litearm-cpp) 按 server RPC 全集功能对齐：同样的方法名、同样的参数契约、同样的外设接口。

> 📖 完整开发指南与接口说明见 [docs/DEVELOPER_GUIDE.md](docs/DEVELOPER_GUIDE.md)。

## 与 pylitearm 的区别

| | pylitearm（本地 SDK） | litearm-python（远程 SDK） |
|---|---|---|
| 运行位置 | 机械臂控制器上 | 任意能连到 server 的机器 |
| 硬件依赖 | 有（CAN / 电机） | 无 |
| 状态获取 | 本地读硬件 | RPC / 状态广播订阅 |
| 配置 | 本地 yaml | server 端加载 |

## 安装

```bash
# 依赖：eclipse-zenoh>=1.0, protobuf>=4.0
pip install -e .        # 开发安装
# 或
pip install litearm-python
```

## 快速开始

```python
import litearm

arm = litearm.Arm(endpoint="tcp/192.168.1.100:7447")
state = arm.get_state()               # 从广播缓存读取（非 RPC）
arm.movej([0.0] * 7, speed=0.5)       # 关节运动

hand = arm.device("hand_0")           # 末端外设：灵巧手
hand.open()
hand.set_gesture("pinch")

arm.close()
```

支持上下文管理器：

```python
with litearm.Arm(endpoint="tcp/127.0.0.1:7447") as arm:
    arm.movej([0.0] * 7)
```

## API 参考

所有方法转发为 RPC 到 litearm-server，签名与 server 端 pylitearm.Arm 对齐。

### 连接

```python
arm = litearm.Arm(endpoint="tcp/127.0.0.1:7447", arm_id="armA")
arm.close()
```

### 运动控制

```python
arm.movej(q, speed=1.0, settle_s=1.0, max_cycles=None,
          allow_start_collision_recovery=False)      # 关节运动
arm.recover_joint_limits(speed=0.05, settle_s=0.5, inset_rad=0.0)  # 越限恢复

arm.movel(pose_goal, speed=0.8)
arm.movec(pose_via, pose_goal)
arm.movep([pose1, pose2, pose3])

arm.replay_joint_path(q_path)
arm.replay_trajectory(traj_q, check_singularity=True)   # 传 JointTrajectory 或 dict
arm.replay_timed_trajectory(traj_q, traj_t, simplify_tolerance_rad=0.01)
arm.play_trajectory(trajectory, verify_robot=True)      # 传 JointTrajectory 或 server 侧路径
traj = arm.record_trajectory(output="trajectories")     # 返回 JointTrajectory

arm.zero_gravity(duration_s=10)
arm.hold(kp_scale=3.0)
arm.joint_impedance(q_des, K, B, tau_max=None, engage_sec=0.3)
arm.cartesian_impedance(q_des, K_cart, B_cart, v_des=None, tau_max=None,
                        sigma_min_thresh=None, max_ori_err=None)
arm.joint_follow(K, B, speed_limit, accel_limit, engage_sec=0.3, duration_s=None)
```

### 纯计算（不驱动电机）

```python
pos, R = arm.fk(q)
q_sol, ok = arm.ik(pos_d, R_d, q_seed=None)
path = arm.plan_movel(q_start, pose_goal)
path = arm.plan_movec(q_start, pose_via, pose_goal)
path = arm.plan_movep(q_start, poses_goal)
```

### 状态读取

```python
state = arm.get_state()          # 广播缓存，非 RPC（q/dq/tau/fault/state/...）
pos, R = arm.get_tcp_pose()
```

### 急停 / 使能

```python
arm.request_stop()               # 高优先级急停（publish，独立通道）
arm.clear_stop()
arm.enable()                     # 使能全部电机并锁住当前姿态
arm.disable()                    # ⚠️ 失能后机械臂会在重力作用下坠落！
```

### 参数调节

```python
arm.set_gains(kp=..., kd=...)
gains = arm.get_gains()
arm.set_payload(mass=1.5, com=(0.0, 0.0, 0.05))
arm.set_installation(base_rpy=[0, 0, 0])
arm.clear_faults()
```

### 外设设备

```python
hand = arm.device("hand_0")
hand.open(); hand.close()
hand.set_gesture("pinch"); hand.list_gestures()
hand.finger_move(pose); hand.set_speed(speed); hand.set_torque(torque)
hand.get_state()

gripper = arm.device("gripper_0")
gripper.set_width(0.5)
width = gripper.get_width()

teach = arm.device("teach_0")
teach.get_joints(); teach.get_buttons()

# 通用方法：get_status / get_info / connect / disconnect / clear_faults / set_force
```

设备管理器：`arm.devices["hand_0"]` 语法，延迟创建 RemoteDevice。

向后兼容的灵巧手便捷属性：`arm.hand.open()`（等价 `arm.device("hand_0")`）。

### 系统 / 设置

```python
stats = arm.get_system_stats()
arm.get_logs(page=1, size=50, search="movej")
arm.restart_service()

# settings：关节限位 / 零位偏移 / 末端 / 笛卡尔限位 / 碰撞配置
arm.set_joint_limits({"q_max": [3.0] * 7}); arm.get_joint_limits()
arm.set_zero_offsets({"joint_0": 0.01});     arm.get_zero_offsets()
arm.set_end_effector({"type": "gripper"});   arm.get_end_effector()
arm.set_cartesian_limits({"linear_velocity": 0.5}); arm.get_cartesian_limits()
arm.set_collision_config({"enabled": True}); arm.get_collision_config()
```

### 轨迹管理（server 端录制 / CRUD）

```python
arm.start_recording(); arm.get_recording_state()
arm.stop_recording(); arm.discard_recording()
arm.list_trajectories()
arm.save_trajectory("t1", "demo", points, duration=None)
arm.delete_trajectory("t1")
arm.get_playback_state()
```

### 末端设备管理（server 按需 fork device_daemon）

```python
arm.list_device_types()
arm.connect_device(category="hand", subtype="lite6_hand", device_id="end_0",
                   can_iface="", config=None)
arm.get_active_device(device_id="end_0")
arm.disconnect_device(device_id="end_0")
```

### 遥操（与命令行 `--teleop-mode` 共享同一遥操状态）

```python
arm.enter_teleop("master")                                  # 本臂采样发布
arm.enter_teleop("slave", peer="tcp/10.0.0.2:7447")         # 跟随 master
arm.get_teleop_status()
arm.exit_teleop()
```

> 遥操态下 server 拒绝一切手动控制类 RPC，只放行只读 / 急停 / `exit_teleop`。

### CAN 隧道（直接使用厂商 SDK）

```python
from litearm.can_bridge import RemoteCAN

can = RemoteCAN("tcp/127.0.0.1:7447", iface="can0")
can.start()
# 在本地直接用厂商 CAN 协议收发帧，经 zenoh 隧道到 server 的 can0
can.stop()
```

## 位姿格式

客户端不依赖 numpy，位姿用纯 Python list：

```python
pose = [position, rotation]
position = [px, py, pz]                     # 3 元素
rotation = [[r00, r01, r02],                # 3x3 行主序旋转矩阵
            [r10, r11, r12],
            [r20, r21, r22]]
```

## 服务端配置

```bash
python -m litearm_server --endpoint tcp/0.0.0.0:7447 --iface can0
```

## 示例

见 [examples/README.md](examples/README.md)：

| 样例 | 演示 | 是否运动 |
|---|---|---|
| `01_read_state.py` | 连接 + 读状态 + TCP 位姿 | ❌ 只读 |
| `02_movej.py` | 关节空间运动 movej | ✅ 运动 |
| `03_fk_ik.py` | 正逆运动学（纯计算 RPC） | ❌ 不运动 |
| `04_movel.py` | 笛卡尔直线运动 movel + plan_movel | ✅ 运动 |

```bash
# 默认连接地瓜 (192.168.31.237:7447)
python3 examples/01_read_state.py
python3 examples/01_read_state.py --endpoint tcp/127.0.0.1:7447
```

## 开发

```bash
pip install -e ".[dev]"
python -m pytest tests/ -q
```

## License

Proprietary
