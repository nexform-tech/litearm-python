"""Remote Arm client — API-compatible with pylitearm.Arm.

Connects to litearm-server via zenoh and forwards all calls as RPC.
No pylitearm dependency, no Pinocchio, no numpy.

外设支持：
- arm.device("hand_0") → 灵巧手远程接口
- arm.device("gripper_0") → 夹爪远程接口
- arm.device("teach_0") → 示教板远程接口
- from litearm.can_bridge import RemoteCAN → CAN 隧道（直接使用厂商 SDK）
"""
from __future__ import annotations

import itertools
import json
import os
import time
import uuid
from typing import Any, Dict, List, Optional, Tuple, Union

from . import codec, protocol
from .device import DeviceManager, RemoteDevice
from .hand import RemoteHand
from .transport import Transport, ZenohTransport
from .types import JointTrajectory


class Arm:
    """LiteArm remote client. API mirrors pylitearm.Arm for seamless migration.

    Usage::

        arm = litearm.Arm(endpoint="tcp/192.168.1.100:7447")
        state = arm.get_state()
        arm.movej([0.0]*7, speed=0.5)
        arm.close()

    Or as context manager::

        with litearm.Arm(endpoint="tcp/127.0.0.1:7447") as arm:
            arm.movej([0.0]*7)
    """

    def __init__(
        self,
        endpoint: Optional[str] = None,
        arm_id: str = "armA",
        transport: Optional[Transport] = None,
        query_timeout: Optional[float] = None,
    ) -> None:
        """Connect to a litearm-server daemon.

        Args:
            endpoint: Zenoh endpoint to connect to (e.g. "tcp/127.0.0.1:7447").
                Defaults to ``LITEARM_ENDPOINT`` env var, then ``tcp/127.0.0.1:7447``.
            arm_id: Arm identifier (default "armA").
            transport: Optional pre-configured Transport (for testing with InProcTransport).
            query_timeout: RPC 超时（秒）。缺省用一个很大的有限值（~11.5 天，
                等于「永不超时」），使阻塞运动方法（movej/movel/...）能一直等到
                完成。快调用不受影响。别传 float("inf")（zenoh 会报 negative timeout）。
        """
        if endpoint is None:
            endpoint = os.environ.get("LITEARM_ENDPOINT", "tcp/127.0.0.1:7447")
        if transport is not None:
            self._tp = transport
        else:
            kw = {} if query_timeout is None else {"query_timeout": query_timeout}
            self._tp = ZenohTransport(connect_endpoints=[endpoint], **kw)
        self._arm_id = arm_id
        self._rpc_topic = protocol.rpc_topic(arm_id)
        self._state_sub = self._tp.sub(protocol.state_topic(arm_id))
        self._estop_topic = protocol.estop_topic(arm_id)
        self._command_topic = protocol.command_topic(arm_id)
        self._client_id = f"sdk-py-{uuid.uuid4().hex[:8]}"
        self._seq = itertools.count(1)
        self._last_state: Optional[dict] = None
        self._hand: Optional[RemoteHand] = None  # 延迟创建（向后兼容）
        self._devices: Optional[DeviceManager] = None  # 设备管理器（延迟创建）

    # ── Pure computation API (no hardware needed on server) ────────────────────

    def fk(self, q: List[float]) -> Tuple[List[float], List[List[float]]]:
        """Forward kinematics: joint angles → (position, rotation_matrix)."""
        return self._rpc("fk", q=q)

    def ik(
        self,
        pos_d: List[float],
        R_d: List[List[float]],
        q_seed: Optional[List[float]] = None,
    ) -> Tuple[List[float], bool]:
        """Inverse kinematics: (position, rotation) → (q, success)."""
        return self._rpc("ik", pos_d=pos_d, R_d=R_d, q_seed=q_seed)

    def plan_movel(
        self, q_start: List[float], pose_goal: Any
    ) -> List[List[float]]:
        """Plan a straight-line Cartesian path. Returns list of joint configs."""
        return self._rpc("plan_movel", q_start=q_start, pose_goal=pose_goal)

    def plan_movec(
        self,
        q_start: List[float],
        pose_via: Any,
        pose_goal: Any,
    ) -> List[List[float]]:
        """Plan a circular-arc Cartesian path through via-point."""
        return self._rpc("plan_movec", q_start=q_start, pose_via=pose_via, pose_goal=pose_goal)

    def plan_movep(
        self, q_start: List[float], poses_goal: List[Any]
    ) -> List[List[float]]:
        """Plan a multi-waypoint Cartesian path."""
        return self._rpc("plan_movep", q_start=q_start, poses_goal=poses_goal)

    # ── Motion execution ──────────────────────────────────────────────────────

    def movej(
        self,
        q_target: List[float],
        speed: float = 1.0,
        settle_s: float = 1.0,
        max_cycles: Optional[int] = None,
        allow_start_collision_recovery: bool = False,
        **kwargs: Any,
    ) -> bool:
        """Move to joint target. cancel_token/log_file are ignored (use request_stop)."""
        return self._rpc("movej", q_target=q_target, speed=speed,
                         settle_s=settle_s, max_cycles=max_cycles,
                         allow_start_collision_recovery=allow_start_collision_recovery)

    def recover_joint_limits(
        self,
        speed: float = 0.05,
        settle_s: float = 0.5,
        max_cycles: Optional[int] = None,
        inset_rad: float = 0.0,
        **kwargs: Any,
    ) -> bool:
        """Slowly return every out-of-limit joint to the nearest safe boundary.

        Requires server connected with ``allow_limit_recovery=True``.
        """
        return self._rpc("recover_joint_limits", speed=speed, settle_s=settle_s,
                         max_cycles=max_cycles, inset_rad=inset_rad)

    def movel(
        self,
        pose_goal: Any,
        speed: float = 1.0,
        settle_s: float = 0.8,
        max_cycles: Optional[int] = None,
        **kwargs: Any,
    ) -> bool:
        """Move in a straight Cartesian line."""
        return self._rpc("movel", pose_goal=pose_goal, speed=speed,
                         settle_s=settle_s, max_cycles=max_cycles)

    def movec(
        self,
        pose_via: Any,
        pose_goal: Any,
        speed: float = 1.0,
        settle_s: float = 0.8,
        max_cycles: Optional[int] = None,
        **kwargs: Any,
    ) -> bool:
        """Move in a circular arc through via-point."""
        return self._rpc("movec", pose_via=pose_via, pose_goal=pose_goal,
                         speed=speed, settle_s=settle_s, max_cycles=max_cycles)

    def movep(
        self,
        poses_goal: List[Any],
        speed: float = 1.0,
        settle_s: float = 0.8,
        max_cycles: Optional[int] = None,
        **kwargs: Any,
    ) -> bool:
        """Move through a sequence of Cartesian waypoints with corner blending."""
        return self._rpc("movep", poses_goal=poses_goal, speed=speed,
                         settle_s=settle_s, max_cycles=max_cycles)

    def replay_joint_path(
        self,
        q_path: List[List[float]],
        speed: float = 1.0,
        settle_s: float = 0.5,
        goto_start: bool = True,
        goto_speed: float = 0.3,
        max_cycles: Optional[int] = None,
        **kwargs: Any,
    ) -> bool:
        """Replay a sequence of joint configurations."""
        return self._rpc("replay_joint_path", q_path=q_path, speed=speed,
                         settle_s=settle_s, goto_start=goto_start,
                         goto_speed=goto_speed, max_cycles=max_cycles)

    def replay_trajectory(
        self,
        traj_q: Any,
        speed: float = 1.0,
        goto_start: bool = True,
        goto_speed: float = 0.3,
        max_cycles: Optional[int] = None,
        check_singularity: bool = True,
        **kwargs: Any,
    ) -> bool:
        """Replay a JointTrajectory or path. on_progress/on_sample/log/cancel_token ignored.

        check_singularity: 是否把雅可比奇异区当作硬错误（与 server 端 pylitearm 对齐）。
        """
        # If traj_q is a JointTrajectory, serialize to dict for transport
        if isinstance(traj_q, JointTrajectory):
            traj_q = traj_q.to_dict()
        return self._rpc("replay_trajectory", traj_q=traj_q, speed=speed,
                         goto_start=goto_start, goto_speed=goto_speed,
                         max_cycles=max_cycles,
                         check_singularity=check_singularity)

    def replay_timed_trajectory(
        self,
        traj_q: List[List[float]],
        traj_t: List[float],
        speed: float = 1.0,
        goto_start: bool = True,
        goto_speed: float = 0.3,
        simplify_tolerance_rad: float = 0.01,
        max_cycles: Optional[int] = None,
        **kwargs: Any,
    ) -> bool:
        """Replay a measured trajectory on its recorded time axis.

        Safety-enforced: automatically stretches time to respect vel/acc/jerk limits.
        """
        return self._rpc("replay_timed_trajectory", traj_q=traj_q, traj_t=traj_t,
                         speed=speed, goto_start=goto_start, goto_speed=goto_speed,
                         simplify_tolerance_rad=simplify_tolerance_rad,
                         max_cycles=max_cycles)

    def play_trajectory(
        self,
        trajectory: Union[JointTrajectory, str],
        speed: float = 1.0,
        goto_start: bool = True,
        goto_speed: float = 0.3,
        verify_robot: bool = True,
        simplify_tolerance_rad: float = 0.01,
        max_cycles: Optional[int] = None,
        **kwargs: Any,
    ) -> bool:
        """Load and replay a saved JointTrajectory.

        Args:
            trajectory: JointTrajectory 对象（转 dict 传输）或 server 侧轨迹
                文件路径字符串（如 "trajectories/traj_001.json"）。
        """
        if isinstance(trajectory, JointTrajectory):
            trajectory = trajectory.to_dict()
        return self._rpc("play_trajectory", trajectory=trajectory, speed=speed,
                         goto_start=goto_start, goto_speed=goto_speed,
                         verify_robot=verify_robot,
                         simplify_tolerance_rad=simplify_tolerance_rad,
                         max_cycles=max_cycles)

    def record_trajectory(
        self,
        output: str = "trajectories",
        duration_s: Optional[float] = None,
        sample_rate_hz: float = 100.0,
        filter_alpha: float = 0.15,
        name: Optional[str] = None,
        **kwargs: Any,
    ) -> JointTrajectory:
        """Record a trajectory by dragging the arm (zero_gravity mode)."""
        result = self._rpc("record_trajectory", output=output, duration_s=duration_s,
                           sample_rate_hz=sample_rate_hz, filter_alpha=filter_alpha,
                           name=name)
        return JointTrajectory.from_dict(result)

    def hold(
        self,
        kp_scale: float = 3.0,
        max_cycles: Optional[int] = None,
        **kwargs: Any,
    ) -> bool:
        """Hold current position with increased stiffness."""
        return self._rpc("hold", kp_scale=kp_scale, max_cycles=max_cycles)

    def zero_gravity(
        self,
        max_cycles: Optional[int] = None,
        duration_s: Optional[float] = None,
        measured_overspeed_factor: Optional[float] = None,
        vel_max: Optional[List[float]] = None,
        **kwargs: Any,
    ) -> bool:
        """Enable zero-gravity (free-drag) mode. on_sample/cancel_token ignored.

        measured_overspeed_factor / vel_max 与 server 端保持兼容（pylitearm
        中已废弃、无效果，仅向后兼容保留）。
        """
        return self._rpc("zero_gravity", max_cycles=max_cycles, duration_s=duration_s,
                         measured_overspeed_factor=measured_overspeed_factor,
                         vel_max=vel_max)

    def joint_impedance(
        self,
        q_des: List[float],
        K: Any,
        B: Any,
        tau_max: Optional[Any] = None,
        engage_sec: float = 0.3,
        max_cycles: Optional[int] = None,
        **kwargs: Any,
    ) -> bool:
        """Joint-space impedance control."""
        return self._rpc("joint_impedance", q_des=q_des, K=K, B=B,
                         tau_max=tau_max, engage_sec=engage_sec,
                         max_cycles=max_cycles)

    def cartesian_impedance(
        self,
        q_des: List[float],
        K_cart: Any,
        B_cart: Any,
        v_des: Optional[Any] = None,
        tau_max: Optional[Any] = None,
        engage_sec: float = 0.3,
        max_cycles: Optional[int] = None,
        sigma_min_thresh: Optional[float] = None,
        max_ori_err: Optional[float] = None,
        measured_overspeed_factor: Optional[float] = None,
        vel_max: Optional[List[float]] = None,
        **kwargs: Any,
    ) -> bool:
        """Cartesian-space impedance control."""
        return self._rpc("cartesian_impedance", q_des=q_des, K_cart=K_cart,
                         B_cart=B_cart, v_des=v_des, tau_max=tau_max,
                         engage_sec=engage_sec, max_cycles=max_cycles,
                         sigma_min_thresh=sigma_min_thresh,
                         max_ori_err=max_ori_err,
                         measured_overspeed_factor=measured_overspeed_factor,
                         vel_max=vel_max)

    def joint_follow(
        self,
        K: Optional[Any] = None,
        B: Optional[Any] = None,
        speed_limit: Optional[Any] = None,
        accel_limit: Optional[Any] = None,
        engage_sec: float = 0.3,
        max_cycles: Optional[int] = None,
        duration_s: Optional[float] = None,
        **kwargs: Any,
    ) -> bool:
        """Follow an external target provider. target_provider is server-side only."""
        return self._rpc("joint_follow", K=K, B=B, speed_limit=speed_limit,
                         accel_limit=accel_limit, engage_sec=engage_sec,
                         max_cycles=max_cycles, duration_s=duration_s)

    # ── State reading ─────────────────────────────────────────────────────────

    def get_state(self, refresh: bool = False) -> Optional[dict]:
        """Get latest robot state from broadcast cache (no RPC call).

        Args:
            refresh: If True, force a fresh read (currently same behavior).
        Returns:
            RobotState dict or None if no state received yet.
        """
        raw = self._state_sub.drain_latest()
        if raw is not None:
            self._last_state = codec.decode_state(raw)
        return self._last_state

    def get_tcp_pose(self) -> Tuple[List[float], List[List[float]]]:
        """Get current TCP pose as (position, rotation_matrix)."""
        return self._rpc("get_tcp_pose")

    # ── 外设设备 ──────────────────────────────────────────────────────────────

    def device(self, device_id: str) -> RemoteDevice:
        """获取外设远程接口。

        Args:
            device_id: 设备唯一标识（如 "hand_0", "gripper_0", "teach_0"）。

        Returns:
            RemoteDevice 实例。

        Usage::

            # 灵巧手
            hand = arm.device("hand_0")
            hand.open()
            hand.set_gesture("pinch")

            # 夹爪
            gripper = arm.device("gripper_0")
            gripper.set_width(0.5)

            # 示教板
            teach = arm.device("teach_0")
            joints = teach.get_joints()
        """
        if self._devices is None:
            self._devices = DeviceManager(self._rpc)
        return self._devices.get(device_id)

    @property
    def devices(self) -> DeviceManager:
        """设备管理器（支持 arm.devices["hand_0"] 语法）。"""
        if self._devices is None:
            self._devices = DeviceManager(self._rpc)
        return self._devices

    # ── 灵巧手 (向后兼容) ─────────────────────────────────────────────────────

    @property
    def hand(self) -> RemoteHand:
        """灵巧手远程接口（向后兼容）。

        推荐使用 arm.device("hand_0") 替代。

        Usage::

            arm.hand.open()
            arm.hand.set_gesture("pinch")
        """
        if self._hand is None:
            self._hand = RemoteHand(self._rpc)
        return self._hand

    # ── Emergency stop ────────────────────────────────────────────────────────

    def request_stop(self) -> None:
        """Send high-priority emergency stop signal."""
        self._tp.pub(self._estop_topic, codec.encode_estop())

    def clear_stop(self) -> None:
        """Clear the stop condition and return to ready state."""
        return self._rpc("clear_stop")

    # ── Direct MIT control ──────────────────────────────────────────────────

    def send_mit(self, kp, kd, q_ref, dq_ref, tau_ff) -> None:
        """直接控制关节电机（纯 MIT 五参数，逐帧）。

        异步 pub 到 command_topic，非阻塞，不等待回执；帧率由用户循环控制。
        首次调用后 server 自动进入 DIRECT 模式（护栏全在 pylitearm）。
        """
        frame = {
            "type": "mit",
            "client_id": self._client_id,
            "seq": next(self._seq),
            "kp": list(kp),
            "kd": list(kd),
            "q_ref": list(q_ref),
            "dq_ref": list(dq_ref),
            "tau_ff": list(tau_ff),
            "ts": time.time(),
        }
        self._tp.pub(self._command_topic, json.dumps(frame).encode("utf-8"))

    def set_guards(self, *, slew_limit=None, tau_max=None, watchdog_timeout=None,
                   position_bounds=None, velocity_bounds=None, jerk_limit=None) -> Any:
        """护栏全局一次性配置（RPC，带回执）。None = 不变。"""
        return self._rpc("set_guards", slew_limit=slew_limit, tau_max=tau_max,
                         watchdog_timeout=watchdog_timeout,
                         position_bounds=position_bounds,
                         velocity_bounds=velocity_bounds, jerk_limit=jerk_limit)

    def get_guards(self) -> Dict[str, Any]:
        """读取当前护栏配置（RPC）。"""
        return self._rpc("get_guards")

    # ── Parameter tuning ──────────────────────────────────────────────────────

    def set_gains(
        self,
        kp: Optional[Any] = None,
        kd: Optional[Any] = None,
    ) -> Dict:
        """Set PD controller gains."""
        return self._rpc("set_gains", kp=kp, kd=kd)

    def get_gains(self) -> Dict:
        """Get current PD controller gains."""
        return self._rpc("get_gains")

    def clear_faults(self) -> List[Tuple[int, int]]:
        """Clear motor faults. Returns list of cleared (motor_id, fault_code)."""
        return self._rpc("clear_faults")

    def enable(self) -> None:
        """Enable all motors and hold current pose (re-enable after disable())."""
        return self._rpc("enable")

    def disable(self) -> None:
        """Disable all motors (arm will drop under gravity!). CAN stays connected."""
        return self._rpc("disable")

    def set_payload(
        self,
        mass: float,
        com: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> Dict:
        """Set end-effector payload (mass + center of mass)."""
        return self._rpc("set_payload", mass=mass, com=list(com))

    def get_payload(self) -> Dict:
        """Get current payload configuration."""
        return self._rpc("get_payload")

    def set_installation(
        self,
        base_rpy: Optional[List[float]] = None,
        gravity: Optional[List[float]] = None,
    ) -> Dict:
        """Set installation orientation (base RPY or gravity vector)."""
        return self._rpc("set_installation", base_rpy=base_rpy, gravity=gravity)

    def get_installation(self) -> Dict:
        """Get current installation configuration."""
        return self._rpc("get_installation")

    # ── 系统 / 设置 / 轨迹管理 / 设备管理 / 遥操（server 扩展 RPC）──────────────

    def get_system_stats(self) -> Dict[str, Any]:
        """Get system stats (CPU, memory, board temperature, uptime)."""
        return self._rpc("get_system_stats")

    def get_logs(self, page: int = 1, size: int = 50, search: str = "") -> Dict[str, Any]:
        """Get server logs (paginated)."""
        return self._rpc("get_logs", page=page, size=size, search=search)

    def restart_service(self) -> Dict[str, Any]:
        """Request restart of the arm service."""
        return self._rpc("restart_service")

    def get_joint_limits(self) -> Dict[str, Any]:
        return self._rpc("get_joint_limits")

    def set_joint_limits(self, limits: Dict[str, Any]) -> Dict[str, Any]:
        return self._rpc("set_joint_limits", limits=limits)

    def get_zero_offsets(self) -> Dict[str, Any]:
        return self._rpc("get_zero_offsets")

    def set_zero_offsets(self, offsets: Dict[str, Any]) -> Dict[str, Any]:
        return self._rpc("set_zero_offsets", offsets=offsets)

    def get_end_effector(self) -> Dict[str, Any]:
        return self._rpc("get_end_effector")

    def set_end_effector(self, config: Dict[str, Any]) -> Dict[str, Any]:
        return self._rpc("set_end_effector", config=config)

    def get_cartesian_limits(self) -> Dict[str, Any]:
        return self._rpc("get_cartesian_limits")

    def set_cartesian_limits(self, limits: Dict[str, Any]) -> Dict[str, Any]:
        return self._rpc("set_cartesian_limits", limits=limits)

    def get_collision_config(self) -> Dict[str, Any]:
        return self._rpc("get_collision_config")

    def set_collision_config(self, config: Dict[str, Any]) -> Dict[str, Any]:
        return self._rpc("set_collision_config", config=config)

    # ── 轨迹管理（server 端录制/CRUD） ───────────────────────────────────────

    def start_recording(self) -> Dict[str, Any]:
        return self._rpc("start_recording")

    def stop_recording(self) -> Dict[str, Any]:
        return self._rpc("stop_recording")

    def discard_recording(self) -> Dict[str, Any]:
        return self._rpc("discard_recording")

    def get_recording_state(self) -> Dict[str, Any]:
        return self._rpc("get_recording_state")

    def get_playback_state(self) -> Dict[str, Any]:
        return self._rpc("get_playback_state")

    def list_trajectories(self) -> Dict[str, Any]:
        return self._rpc("list_trajectories")

    def save_trajectory(
        self, id: str, name: str, points: List[List[float]],
        duration: Optional[float] = None,
    ) -> Dict[str, Any]:
        return self._rpc("save_trajectory", id=id, name=name, points=points,
                         duration=duration)

    def delete_trajectory(self, id: str) -> Dict[str, Any]:
        return self._rpc("delete_trajectory", id=id)

    # ── 末端设备管理（server 按需 fork device_daemon） ──────────────────────

    def list_device_types(self) -> List[Dict[str, Any]]:
        """列出源码内置的可用末端类型（下拉框数据源）。"""
        return self._rpc("list_device_types")

    def connect_device(
        self,
        category: str,
        subtype: str,
        device_id: str = "end_0",
        can_iface: str = "",
        config: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        """连接末端：server fork device_daemon 并等其就绪 + 持久化。"""
        return self._rpc("connect_device", category=category, subtype=subtype,
                         device_id=device_id, can_iface=can_iface, config=config)

    def disconnect_device(self, device_id: str = "end_0") -> Dict[str, Any]:
        """断开末端：停止 daemon + 更新持久化。"""
        return self._rpc("disconnect_device", device_id=device_id)

    def get_active_device(self, device_id: str = "end_0") -> Dict[str, Any]:
        """查询当前末端状态（配置/在线/类型）。"""
        return self._rpc("get_active_device", device_id=device_id)

    # ── 遥操（主从遥操，与命令行 --teleop-mode 共享同一遥操状态） ────────────

    def enter_teleop(self, mode: str, **params: Any) -> Dict[str, Any]:
        """进入遥操。

        - master：本臂 zero_gravity 采样并 pub 关节流。
        - slave：需传 ``peer``（master 网络端点）+ 可选 ``master_arm_id``。

        进入后 server 拒绝一切手动控制类 RPC，只放行只读/急停/exit_teleop。
        """
        return self._rpc("enter_teleop", mode=mode, **params)

    def exit_teleop(self) -> Dict[str, Any]:
        """退出遥操：停跟随 + 解锁 + 机械臂就地持位。幂等。"""
        return self._rpc("exit_teleop")

    def get_teleop_status(self) -> Dict[str, Any]:
        """查询当前遥操状态（active / mode / stats）。"""
        return self._rpc("get_teleop_status")

    # ── Internal RPC ──────────────────────────────────────────────────────────

    def _rpc(self, method: str, **kwargs: Any) -> Any:
        """Send an RPC call and return the result (or raise on error)."""
        payload = codec.encode_request(method, kwargs)
        reply = self._tp.query(self._rpc_topic, payload)
        return codec.decode_reply(reply)

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def close(self) -> None:
        """Close the transport connection."""
        self._tp.close()

    def __enter__(self) -> "Arm":
        return self

    def __exit__(self, *args: Any) -> None:
        self.close()

    def __repr__(self) -> str:
        return f"Arm(arm_id={self._arm_id!r})"
