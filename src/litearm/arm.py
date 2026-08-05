"""Remote Arm client — API-compatible with pylitearm.Arm.

Connects to litearm-server via zenoh and forwards all calls as RPC.
No pylitearm dependency, no Pinocchio, no numpy.
"""
from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple, Union

from . import codec, protocol
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
        endpoint: str = "tcp/127.0.0.1:7447",
        arm_id: str = "armA",
        transport: Optional[Transport] = None,
    ) -> None:
        """Connect to a litearm-server daemon.

        Args:
            endpoint: Zenoh endpoint to connect to (e.g. "tcp/127.0.0.1:7447").
            arm_id: Arm identifier (default "armA").
            transport: Optional pre-configured Transport (for testing with InProcTransport).
        """
        if transport is not None:
            self._tp = transport
        else:
            self._tp = ZenohTransport(connect_endpoints=[endpoint])
        self._arm_id = arm_id
        self._rpc_topic = protocol.rpc_topic(arm_id)
        self._state_sub = self._tp.sub(protocol.state_topic(arm_id))
        self._estop_topic = protocol.estop_topic(arm_id)
        self._last_state: Optional[dict] = None

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
        **kwargs: Any,
    ) -> bool:
        """Move to joint target. cancel_token/log_file are ignored (use request_stop)."""
        return self._rpc("movej", q_target=q_target, speed=speed,
                         settle_s=settle_s, max_cycles=max_cycles)

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
        **kwargs: Any,
    ) -> bool:
        """Replay a JointTrajectory or path. on_progress/on_sample/log/cancel_token ignored."""
        # If traj_q is a JointTrajectory, serialize to dict for transport
        if isinstance(traj_q, JointTrajectory):
            traj_q = traj_q.to_dict()
        return self._rpc("replay_trajectory", traj_q=traj_q, speed=speed,
                         goto_start=goto_start, goto_speed=goto_speed,
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
        **kwargs: Any,
    ) -> bool:
        """Enable zero-gravity (free-drag) mode. on_sample/cancel_token ignored."""
        return self._rpc("zero_gravity", max_cycles=max_cycles, duration_s=duration_s)

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
        **kwargs: Any,
    ) -> bool:
        """Cartesian-space impedance control."""
        return self._rpc("cartesian_impedance", q_des=q_des, K_cart=K_cart,
                         B_cart=B_cart, v_des=v_des, tau_max=tau_max,
                         engage_sec=engage_sec, max_cycles=max_cycles)

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

    # ── Emergency stop ────────────────────────────────────────────────────────

    def request_stop(self) -> None:
        """Send high-priority emergency stop signal."""
        self._tp.pub(self._estop_topic, codec.encode_estop())

    def clear_stop(self) -> None:
        """Clear the stop condition and return to ready state."""
        return self._rpc("clear_stop")

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
