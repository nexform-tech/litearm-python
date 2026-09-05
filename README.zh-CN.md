# litearm-python

LiteArm 机械臂的 Python 客户端库。安装后连接机械臂控制服务，即可在任意普通电脑上控制机械臂：运动控制、状态读取、末端设备（灵巧手 / 夹爪 / 示教板）操作等。

## 特点

- 🧩 **纯 Python**：无需任何硬件相关依赖，无 numpy，普通电脑即可运行
- 🔗 **即插即用**：一行代码连接机械臂，调用即执行
- 🎮 **统一外设接口**：灵巧手、夹爪、示教板使用同一套访问方式
- 🚦 **高优先级急停**：独立通道，可随时安全停机
- 🌐 **多语言一致**：与 [litearm-js](../litearm-js) / [litearm-cpp](../litearm-cpp) 提供相同的 API，代码可跨语言迁移

> 📖 完整开发指南与接口说明见 [docs/DEVELOPER_GUIDE.zh-CN.md](docs/DEVELOPER_GUIDE.zh-CN.md)。
> 📊 三端方法对照（python / js / cpp）见 [docs/sdk-api-surface.zh-CN.md](docs/sdk-api-surface.zh-CN.md)。

## 安装

```bash
pip install litearm-python
```

## 快速开始

```python
import litearm

# 连接机械臂（地址为运行控制服务的机器，默认端口 7447）
arm = litearm.Arm(endpoint="tcp/192.168.1.100:7447")

state = arm.get_state()               # 读取当前状态（关节角、速度等）
arm.movej([0.0] * 7, speed=0.5)       # 关节空间运动
arm.home(speed=0.3)                    # 回零：所有关节归零，绕开限位检查

hand = arm.device("hand_0")           # 操作灵巧手
hand.open()
hand.set_gesture("pinch")

arm.close()
```

支持上下文管理器，退出时自动断开连接：

```python
with litearm.Arm(endpoint="tcp/127.0.0.1:7447") as arm:
    arm.movej([0.0] * 7)
```

## 主要功能

### 运动控制

关节 / 直线 / 圆弧 / 多航点运动，轨迹录制与回放，零重力（自由拖动），阻抗控制，关节跟随：

```python
arm.movej([0.1, 0.2, 0.3, 0, 0, 0, 0])            # 关节运动
arm.movel(pose_goal)                              # 直线运动
arm.movec(pose_via, pose_goal)                    # 圆弧运动
arm.movep([pose1, pose2, pose3])                  # 多航点运动

arm.replay_joint_path(q_path)                     # 回放关节轨迹
arm.replay_trajectory(traj_q)                     # 回放录制的轨迹
arm.record_trajectory()                           # 拖动录制轨迹

arm.zero_gravity(duration_s=10)                   # 零重力（自由拖动）
arm.hold()                                        # 原地保持
```

### 状态读取

```python
state = arm.get_state()       # q / dq / tau / fault / state / ...
pos, rot = arm.get_tcp_pose() # 当前末端位姿
```

### 急停 / 使能

```python
arm.request_stop()            # 高优先级急停（独立通道，可随时安全停机）
arm.clear_stop()              # 清除急停状态
arm.enable()                  # 使能电机并保持当前姿态
arm.disable()                 # ⚠️ 失能电机，机械臂会在重力作用下坠落！
```

### 参数调节

```python
arm.set_gains(kp=..., kd=...)            # PD 增益
arm.set_payload(mass=1.5, com=(0, 0, 0.05))  # 末端负载
arm.set_installation(base_rpy=[0, 0, 0])     # 安装姿态
arm.clear_faults()                         # 清除电机故障
```

### 末端设备

灵巧手、夹爪、示教板统一通过 `arm.device(...)` 访问：

```python
hand = arm.device("hand_0")
hand.open(); hand.close()                  # 开 / 合
hand.set_gesture("pinch")                  # 手势
hand.finger_move(pose)                     # 逐指运动

gripper = arm.device("gripper_0")
gripper.set_width(0.5)                     # 夹爪宽度
width = gripper.get_width()

teach = arm.device("teach_0")
teach.get_joints(); teach.get_buttons()    # 示教板读值
```

### 系统 / 设置

```python
arm.get_system_stats()                          # 系统信息（CPU / 内存 / 板温）
arm.get_logs(page=1, size=50, search="movej")   # 日志
arm.restart_service()                           # 重启控制服务

# 机械臂设置：关节限位 / 零位偏移 / 末端执行器 / 笛卡尔限位 / 碰撞配置
arm.get_joint_limits();  arm.set_joint_limits({...})
arm.get_zero_offsets();  arm.set_zero_offsets({...})
arm.get_end_effector();  arm.set_end_effector({...})
```

### 轨迹管理

```python
arm.start_recording(); arm.stop_recording()
arm.list_trajectories()
arm.save_trajectory("t1", "demo", points)
arm.delete_trajectory("t1")
arm.get_playback_state()
```

### 设备管理 / 遥操

```python
arm.list_device_types()
arm.connect_device(category="hand", subtype="lite6_hand", device_id="end_0")
arm.disconnect_device(device_id="end_0")

arm.enter_teleop("master")                               # 本机作为主臂
arm.enter_teleop("slave", peer="tcp/10.0.0.2:7447")      # 跟随主臂
arm.get_teleop_status()
arm.exit_teleop()
```

### CAN 隧道（高级）

需要在本地直接使用厂商 CAN 协议时可用：

```python
from litearm.can_bridge import RemoteCAN

can = RemoteCAN("tcp/127.0.0.1:7447", vcan_iface="vcan0")
can.start()
# 在本地用厂商协议收发 CAN 帧，与机械臂总线互通
can.stop()
```

## 位姿格式

位姿使用纯 Python list，无需 numpy：

```python
pose = [position, rotation]
position = [px, py, pz]                     # 3 元素
rotation = [[r00, r01, r02],                # 3x3 行主序旋转矩阵
            [r10, r11, r12],
            [r20, r21, r22]]
```

## 服务端部署

机械臂控制服务部署在控制器上（如机械臂自带主机）：

```bash
python -m litearm_server --endpoint tcp/0.0.0.0:7447 --iface can0
```

客户端填写的 `endpoint` 即为该控制器的地址与端口。

## 示例

见 [examples/README.zh-CN.md](examples/README.zh-CN.md)：

| 样例 | 演示 | 是否运动 |
|---|---|---|
| `01_read_state.py` | 连接 + 读状态 + TCP 位姿 | ❌ 只读 |
| `02_movej.py` | 关节空间运动 movej | ✅ 运动 |
| `03_fk_ik.py` | 正逆运动学（纯计算，不动臂） | ❌ 不运动 |
| `04_movel.py` | 直线运动 movel + 路径规划 | ✅ 运动 |
| `05_home.py` | 回零 home() — 所有关节归零，绕开限位检查 | ✅ 运动 |

```bash
python3 examples/01_read_state.py
python3 examples/02_movej.py --endpoint tcp/127.0.0.1:7447
```

## 开发

```bash
pip install -e ".[dev]"
python -m pytest tests/ -q
```

## License

Proprietary
