"""SDK-local LiteArm FK/IK/Jacobian and guarded joint-space PTP motion."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional, Sequence

from litearm import _rot
from litearm import _protocol as P
from litearm._robot_model import RobotModel
from litearm.errors import IKError, InvalidCommandError, MotorFaultError, TransportError

if TYPE_CHECKING:
    from litearm.arm import Arm
    from litearm.state import RobotState


MAX_CONTINUATION_POS_M = 0.010
MAX_CONTINUATION_ROT_RAD = math.radians(2.0)
MAX_IK_JOINT_STEP_RAD = 0.35
PATH_SAMPLE_RAD = 0.020
LIMIT_MARGIN_RAD = 0.010
MIN_CLEARANCE_M = 0.005
MIN_SIGMA = 0.05
START_DRIFT_RAD = 0.030
POS_TOL_M = 0.0005
ROT_TOL_RAD = 0.005


@dataclass(frozen=True)
class JacobianResult:
    matrix: tuple[tuple[float, ...], ...]
    sigma_min: float
    near_singular: bool


@dataclass(frozen=True)
class IKSolution:
    q: tuple[float, ...]
    position_error: float
    orientation_error: float
    sigma_min: float
    max_joint_delta: float
    iterations: int
    failure: str = "none"


class LocalKinematicsError(IKError):
    """Local IK or safety preflight failed before any motion was sent."""

    def __init__(self, message: str, *, reason: str,
                 sigma_min: Optional[float] = None,
                 max_joint_delta: Optional[float] = None):
        super().__init__(message)
        self.reason = reason
        self.sigma_min = sigma_min
        self.max_joint_delta = max_joint_delta


def _frame(frame: str) -> str:
    value = str(frame).lower()
    if value not in ("tool", "flange"):
        raise InvalidCommandError("frame 只能是 'tool' 或 'flange'")
    return value


class Kinematics:
    """``arm.kinematics`` local model; never calls firmware IK/FK commands."""

    def __init__(self, arm: "Arm"):
        self._arm = arm
        self._bundle: Optional[RobotModel] = None

    def _robot(self) -> RobotModel:
        if self._bundle is None:
            self._bundle = RobotModel(dependency_hint="litearm-python[robotics]")
        return self._bundle

    @property
    def model(self) -> RobotModel:
        """Shared local model bundle (advanced diagnostics/testing)."""
        return self._robot()

    def _pose_flange(self, q) -> tuple[list[float], list[list[float]]]:
        robot = self._robot()
        values = robot.q(q)
        with robot.lock:
            placement = robot.placement(values)
            pos = [float(v) for v in placement.translation]
            R = [[float(placement.rotation[i, j]) for j in range(3)] for i in range(3)]
        return pos, R

    def fk(self, q: Sequence[float], frame: str = "tool") -> tuple[float, ...]:
        """Compute local FK as ``(x,y,z,roll,pitch,yaw)`` without device I/O."""
        use_frame = _frame(frame)
        pos, R = self._pose_flange(q)
        pose = pos + _rot.mat_to_rpy(R)
        if use_frame == "tool":
            pose = self._arm._to_tool(pose, "kinematics.fk")
        return tuple(float(v) for v in pose)

    def _jacobian_array(self, q, frame: str):
        robot = self._robot()
        values = robot.q(q)
        with robot.lock:
            placement = robot.placement(values)
            J = robot.jacobian(values).copy()
            if frame == "tool" and not self._arm._tcp_offset.is_identity:
                offset = robot.np.asarray(self._arm._tcp_offset.xyz, dtype=float)
                r_world = robot.np.asarray(placement.rotation) @ offset
                for column in range(J.shape[1]):
                    J[:3, column] += robot.np.cross(J[3:, column], r_world)
        return J

    def jacobian(self, q: Sequence[float], frame: str = "tool") -> JacobianResult:
        """Return the world-aligned 6x7 geometric Jacobian and its smallest singular value."""
        use_frame = _frame(frame)
        robot = self._robot()
        J = self._jacobian_array(q, use_frame)
        singular = robot.np.linalg.svd(J, compute_uv=False)
        sigma = float(singular[-1]) if singular.size else 0.0
        if not math.isfinite(sigma):
            sigma = 0.0
        return JacobianResult(
            tuple(tuple(float(v) for v in row) for row in J),
            sigma, sigma < MIN_SIGMA)

    def _limits(self, *, live: bool):
        robot = self._robot()
        if live:
            params = self._arm.params.all_joint_params()
            if len(params) != robot.nq:
                raise InvalidCommandError(
                    f"本地模型是 {robot.nq} 轴，设备状态是 {len(params)} 轴")
            lo = robot.np.asarray([p.q_min for p in params], dtype=float)
            hi = robot.np.asarray([p.q_max for p in params], dtype=float)
        else:
            lo = robot.np.asarray(robot.model.lowerPositionLimit, dtype=float).copy()
            hi = robot.np.asarray(robot.model.upperPositionLimit, dtype=float).copy()
        if (lo.shape != (robot.nq,) or hi.shape != (robot.nq,)
                or not robot.np.all(robot.np.isfinite(lo))
                or not robot.np.all(robot.np.isfinite(hi))
                or robot.np.any(lo >= hi)):
            raise LocalKinematicsError("关节限位不可用", reason="invalid_limits")
        return lo, hi

    def _target(self, pose, frame: str):
        try:
            pos, R = _rot.as_pose(pose)
        except (TypeError, ValueError) as exc:
            raise InvalidCommandError(f"kinematics.ik pose 非法: {exc}") from None
        vec = list(pos) + _rot.mat_to_rpy(R)
        if frame == "tool":
            vec = self._arm._to_flange(vec, "kinematics.ik")
            pos, R = _rot.as_pose(vec)
        if not (all(math.isfinite(float(v)) for v in pos)
                and all(math.isfinite(float(v)) for row in R for v in row)):
            raise InvalidCommandError("kinematics.ik pose 含非有限值")
        return self._robot().np.asarray(pos, dtype=float), self._robot().np.asarray(R, dtype=float)

    def _one_target(self, q0, pos_d, R_d, lo, hi):
        robot = self._robot()
        np = robot.np
        q = np.clip(np.asarray(q0, dtype=float), lo, hi)
        seed = q.copy()
        middle = 0.5 * (lo + hi)
        span = np.maximum(hi - lo, 1e-6)
        damping = 1e-3
        best = None
        for iteration in range(1, 201):
            with robot.lock:
                placement = robot.placement(q)
                J = robot.jacobian(q).copy()
                pos = np.asarray(placement.translation, dtype=float)
                R = np.asarray(placement.rotation, dtype=float)
            ep_vec = pos_d - pos
            er_vec = np.asarray(robot.pin.log3(R_d @ R.T), dtype=float)
            ep = float(np.linalg.norm(ep_vec))
            er = float(np.linalg.norm(er_vec))
            cost = ep + er
            if best is None or cost < best[0]:
                best = (cost, q.copy(), ep, er)
            if ep < POS_TOL_M and er < ROT_TOL_RAD:
                return q, ep, er, iteration
            error = np.concatenate((ep_vec, er_vec))
            A = J @ J.T + damping * np.eye(6)
            try:
                primary = J.T @ np.linalg.solve(A, error)
                pinv = J.T @ np.linalg.solve(A, np.eye(6))
            except np.linalg.LinAlgError:
                damping = min(damping * 10.0, 1.0)
                continue
            null = np.eye(robot.nq) - pinv @ J
            # Stay on the seeded redundant branch and softly move away from limits.
            bias = 0.04 * (seed - q) + 0.01 * (middle - q) / span
            dq = primary + null @ bias
            norm = float(np.linalg.norm(dq))
            if norm > 0.20:
                dq *= 0.20 / norm
            accepted = False
            for scale in (1.0, 0.5, 0.25, 0.125):
                trial = np.clip(q + scale * dq, lo, hi)
                with robot.lock:
                    trial_place = robot.placement(trial)
                tep = float(np.linalg.norm(pos_d - np.asarray(trial_place.translation)))
                ter = float(np.linalg.norm(robot.pin.log3(
                    R_d @ np.asarray(trial_place.rotation).T)))
                if tep + ter < cost:
                    q = trial
                    damping = max(damping * 0.5, 1e-7)
                    accepted = True
                    break
            if not accepted:
                damping = min(damping * 5.0, 1.0)
        assert best is not None
        raise LocalKinematicsError(
            f"本地 IK 未收敛: 位置残差 {best[2]:.6f}m, 姿态残差 {best[3]:.6f}rad",
            reason="no_solution")

    def _solve(self, pose, seed, lo, hi, frame: str) -> IKSolution:
        robot = self._robot()
        np = robot.np
        q_seed = robot.q(seed)
        pos_goal, R_goal = self._target(pose, frame)
        with robot.lock:
            start = robot.placement(q_seed)
            pos_start = np.asarray(start.translation, dtype=float).copy()
            R_start = np.asarray(start.rotation, dtype=float).copy()
        dist = float(np.linalg.norm(pos_goal - pos_start))
        angle = float(np.linalg.norm(robot.pin.log3(R_start.T @ R_goal)))
        steps = max(1, int(math.ceil(dist / MAX_CONTINUATION_POS_M)),
                    int(math.ceil(angle / MAX_CONTINUATION_ROT_RAD)))
        q = q_seed.copy()
        total_iterations = 0
        pos_err = rot_err = 0.0
        rel_log = np.asarray(robot.pin.log3(R_start.T @ R_goal), dtype=float)
        for index in range(1, steps + 1):
            ratio = index / steps
            pos_d = pos_start + ratio * (pos_goal - pos_start)
            R_d = R_start @ robot.pin.exp3(ratio * rel_log)
            prev = q.copy()
            q, pos_err, rot_err, used = self._one_target(q, pos_d, R_d, lo, hi)
            total_iterations += used
            jump = float(np.max(np.abs(q - prev)))
            if not math.isfinite(jump) or jump > MAX_IK_JOINT_STEP_RAD:
                raise LocalKinematicsError(
                    f"IK 分支跳变 {jump:.4f}rad > {MAX_IK_JOINT_STEP_RAD:.2f}rad",
                    reason="branch_jump", max_joint_delta=jump)
        J = self._jacobian_array(q, frame)
        singular = np.linalg.svd(J, compute_uv=False)
        sigma = float(singular[-1]) if singular.size else 0.0
        if not math.isfinite(sigma) or sigma < MIN_SIGMA:
            raise LocalKinematicsError(
                f"目标接近奇异点: sigma_min={sigma:.6f} < {MIN_SIGMA:.3f}",
                reason="near_singular", sigma_min=sigma)
        delta = float(np.max(np.abs(q - q_seed)))
        return IKSolution(tuple(float(v) for v in q), pos_err, rot_err,
                          sigma, delta, total_iterations)

    def ik(self, pose, q_seed: Optional[Sequence[float]] = None,
           frame: str = "tool") -> IKSolution:
        """Solve pose IK locally; no command is sent to the controller."""
        use_frame = _frame(frame)
        live = q_seed is None
        if q_seed is None:
            state = self._arm.get_state(refresh=True).value
            if state is None:
                raise TransportError("本地 IK 需要当前关节状态，但取不到状态帧")
            q_seed = state.q
        lo, hi = self._limits(live=live)
        return self._solve(pose, q_seed, lo, hi, use_frame)

    def _check_path(self, start, goal, lo, hi, frame: str) -> None:
        robot = self._robot()
        np = robot.np
        start = robot.q(start)
        goal = robot.q(goal)
        count = max(1, int(math.ceil(float(np.max(np.abs(goal - start))) /
                                     PATH_SAMPLE_RAD)))
        for index in range(count + 1):
            q = start + (index / count) * (goal - start)
            if np.any(q < lo + LIMIT_MARGIN_RAD) or np.any(q > hi - LIMIT_MARGIN_RAD):
                raise LocalKinematicsError(
                    f"PTP 路径在采样点 {index}/{count} 进入软限位 {LIMIT_MARGIN_RAD:.3f}rad 余量",
                    reason="joint_limit")
            with robot.lock:
                if robot.in_collision(q):
                    pairs = robot.colliding_pairs(q)
                    raise LocalKinematicsError(
                        f"PTP 路径在采样点 {index}/{count} 自碰: {pairs}",
                        reason="collision")
                clearance = robot.minimum_distance(q)
            if not math.isfinite(clearance):
                raise LocalKinematicsError(
                    "PTP 路径碰撞距离不可用", reason="invalid_clearance")
            if clearance < MIN_CLEARANCE_M:
                raise LocalKinematicsError(
                    f"PTP 路径最小间隙 {clearance:.6f}m < {MIN_CLEARANCE_M:.3f}m",
                    reason="clearance")
            sigma = self.jacobian(q, frame=frame).sigma_min
            if sigma < MIN_SIGMA:
                raise LocalKinematicsError(
                    f"PTP 路径在采样点 {index}/{count} 接近奇异点: sigma_min={sigma:.6f}",
                    reason="near_singular", sigma_min=sigma)

    @staticmethod
    def _validate_motion_state(state: "RobotState") -> None:
        if not state.enabled:
            raise InvalidCommandError("move_pose 要求机械臂已使能")
        if state.faulted:
            raise MotorFaultError(f"move_pose 拒绝故障状态: {state.fault_detail}")
        if state.mode == P.MODE_ZERO_G:
            raise InvalidCommandError("move_pose 拒绝零重力模式；先 zero_g_stop()")
        if state.cart_busy:
            raise InvalidCommandError("move_pose 拒绝笛卡尔规划/播放在途")

    def _fresh_state(self) -> "RobotState":
        """Require state activity after entering the safety-critical read.

        ``Arm.get_state(refresh=True)`` intentionally preserves its historical
        cached-state fallback.  That is useful to ordinary callers but unsafe
        for preflight: a silent controller must not let an old q pass as the
        start we are about to execute from.  Compare the reader sequence here
        so the stricter rule remains local to ``move_pose``.
        """
        ack = self._arm._require()
        sequence = ack.status_seq
        message = self._arm.get_state(refresh=True)
        if message.value is None or ack.status_seq == sequence:
            raise TransportError("move_pose 取不到最新状态帧")
        return message.value

    def move_pose(self, pose, speed: float = 0.2,
                  frame: str = "tool") -> "RobotState":
        """Locally solve/check a pose, then send exactly one synchronized PTP target."""
        use_frame = _frame(frame)
        if not 0.0 < float(speed) <= 1.0:
            raise InvalidCommandError("move_pose speed 需在 (0,1]，0 会产生永不到位的轨迹")
        with self._arm._motion_serial:
            for attempt in range(2):
                state = self._fresh_state()
                self._validate_motion_state(state)
                lo, hi = self._limits(live=True)
                solution = self._solve(pose, state.q, lo, hi, use_frame)
                self._check_path(state.q, solution.q, lo, hi, use_frame)
                fresh = self._fresh_state()
                self._validate_motion_state(fresh)
                drift = max(abs(fresh.q[i] - state.q[i]) for i in range(len(state.q)))
                if drift <= START_DRIFT_RAD:
                    return self._arm.movej_sync(solution.q, speed=float(speed))
                if attempt:
                    raise LocalKinematicsError(
                        f"预检期间起点连续变化，最大漂移 {drift:.4f}rad",
                        reason="start_moved", max_joint_delta=drift)
        raise AssertionError("unreachable")


__all__ = [
    "Kinematics", "JacobianResult", "IKSolution", "LocalKinematicsError",
    "MAX_CONTINUATION_POS_M", "MAX_CONTINUATION_ROT_RAD", "MAX_IK_JOINT_STEP_RAD",
    "PATH_SAMPLE_RAD", "LIMIT_MARGIN_RAD", "MIN_CLEARANCE_M", "MIN_SIGMA",
]
