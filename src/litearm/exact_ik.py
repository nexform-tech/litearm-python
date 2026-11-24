"""Exact differential inverse kinematics with joint-velocity bounds.

This is the dependency-light SDK port of lite-vr's ``exact_ik`` backend.  The
solver accepts a Jacobian, not a robot model: callers remain responsible for
computing the Jacobian and for expressing the twist in the same frame.

NumPy is an optional dependency.  Install ``litearm-python[ik]`` (or the
``robotics`` extra) before importing this module.
"""
from __future__ import annotations

try:
    import numpy as np
except ImportError as exc:  # pragma: no cover - exercised in a clean environment
    raise ImportError(
        "litearm.exact_ik requires NumPy; install 'litearm-python[ik]'"
    ) from exc

FAIL_NONE = "none"
FAIL_SHAPE = "shape"
FAIL_NO_JACOBIAN = "no_jacobian"
FAIL_BOX = "box"
FAIL_RESIDUAL = "residual"
NEAR_SINGULAR_SIGMA = 0.05


class TaskVelocitySolution:
    """Result of :func:`exact_task_velocity`."""

    __slots__ = ("velocity", "residual", "achieved", "failure",
                 "worst_joint", "worst_excess", "worst_side")

    def __init__(self, velocity, residual=0.0, achieved=False,
                 failure=FAIL_NONE, worst_joint=-1, worst_excess=0.0,
                 worst_side=0):
        self.velocity = velocity
        self.residual = float(residual)
        self.achieved = bool(achieved)
        self.failure = failure
        self.worst_joint = int(worst_joint)
        self.worst_excess = float(worst_excess)
        self.worst_side = int(worst_side)


def null_space_interval(velocity, direction, lower, upper):
    """Return the alpha interval keeping ``velocity + alpha*direction`` in bounds."""
    velocity = np.asarray(velocity, float).ravel()
    direction = np.asarray(direction, float).ravel()
    lower = np.asarray(lower, float).ravel()
    upper = np.asarray(upper, float).ravel()
    low, high = -np.inf, np.inf
    n = velocity.size
    if n == 0 or direction.size != n or lower.size != n or upper.size != n:
        return low, high, True
    for i in range(n):
        d = direction[i]
        if d == 0.0:
            if velocity[i] < lower[i] or velocity[i] > upper[i]:
                return low, high, True
            continue
        a = (lower[i] - velocity[i]) / d
        b = (upper[i] - velocity[i]) / d
        low = max(low, min(a, b))
        high = min(high, max(a, b))
    return low, high, low > high


def exact_task_velocity(jacobian, twist, lower, upper,
                        rank_tolerance=1e-9, residual_tolerance=1e-3):
    """Solve ``J*dq = twist`` exactly inside a per-joint velocity box.

    The minimum-norm solution is used unless the one-dimensional redundant
    direction must be spent to enter the box.  If the task cannot be delivered
    without scaling, ``achieved`` is false and the returned velocity is zero.
    This matches lite-vr's deliberately binary gain (one or zero).
    """
    jacobian = np.asarray(jacobian, float)
    twist = np.asarray(twist, float).ravel()
    lower = np.asarray(lower, float).ravel()
    upper = np.asarray(upper, float).ravel()
    if jacobian.ndim != 2:
        return TaskVelocitySolution(np.zeros(0), residual=1.0,
                                    failure=FAIL_SHAPE)
    rows, cols = jacobian.shape
    if (rows <= 0 or cols <= 0 or twist.size != rows or
            lower.size != cols or upper.size != cols or
            np.any(lower > upper) or not np.all(np.isfinite(jacobian)) or
            not np.all(np.isfinite(twist)) or not np.all(np.isfinite(lower)) or
            not np.all(np.isfinite(upper))):
        return TaskVelocitySolution(np.zeros(max(cols, 0)), residual=1.0,
                                    failure=FAIL_SHAPE)

    u, singular, vh = np.linalg.svd(jacobian, full_matrices=True)
    if singular.size == 0 or singular[0] <= 0.0:
        return TaskVelocitySolution(np.zeros(cols), residual=1.0,
                                    failure=FAIL_NO_JACOBIAN)
    keep = singular > float(rank_tolerance) * singular[0]
    particular = (vh[:singular.size][keep].T @
                  ((u[:, :singular.size][:, keep].T @ twist) / singular[keep]))

    # LiteArm is 6x7, so one null direction is sufficient.  For a more general
    # Jacobian with multiple null directions we intentionally retain the same
    # last-vector policy as the source implementation.
    direction = vh[cols - 1].copy() if cols > rows else np.zeros(cols)
    low, high, empty = null_space_interval(particular, direction, lower, upper)
    if empty:
        breakpoints = [0.0]
        for i in range(cols):
            if direction[i] != 0.0:
                breakpoints.extend(((lower[i] - particular[i]) / direction[i],
                                    (upper[i] - particular[i]) / direction[i]))
        best, worst_joint, worst_side = np.inf, -1, 0
        for alpha in breakpoints:
            value = particular + alpha * direction
            miss = np.maximum(0.0, np.maximum(lower - value, value - upper))
            j = int(np.argmax(miss))
            if miss[j] < best:
                best = float(miss[j])
                worst_joint = j
                worst_side = 1 if value[j] > upper[j] else -1
        return TaskVelocitySolution(
            np.zeros(cols), residual=1.0, failure=FAIL_BOX,
            worst_joint=worst_joint,
            worst_excess=0.0 if not np.isfinite(best) else best,
            worst_side=worst_side)

    alpha = float(min(high, max(low, 0.0)))
    velocity = particular + alpha * direction
    reference = max(float(np.linalg.norm(twist)), 1e-9)
    ratio = float(np.linalg.norm(jacobian @ velocity - twist)) / reference
    if ratio > float(residual_tolerance):
        return TaskVelocitySolution(np.zeros(cols), residual=ratio,
                                    failure=FAIL_RESIDUAL)
    return TaskVelocitySolution(velocity, residual=ratio, achieved=True)


def box_binding(solution, box_lower, box_upper, speed_limit):
    """Classify a box failure as a speed or joint-range constraint."""
    j = solution.worst_joint
    if solution.failure != FAIL_BOX or j < 0 or solution.worst_side == 0:
        return None
    bound = box_upper[j] if solution.worst_side > 0 else box_lower[j]
    ceiling = speed_limit if solution.worst_side > 0 else -speed_limit
    return "speed" if abs(bound - ceiling) <= 1e-9 else "range"


def bounded_task_velocity(jacobian, twist, lower, upper, damping=0.02):
    """Return the best feasible damped task velocity inside the joint box.

    This is lite-vr's continuous fallback for poses where
    :func:`exact_task_velocity` cannot deliver the complete twist.  It requires
    SciPy in addition to NumPy (both are installed by the ``ik`` extra).
    """
    try:
        from scipy.optimize import lsq_linear
    except ImportError as exc:  # pragma: no cover - clean optional-dependency env
        raise ImportError(
            "bounded_task_velocity requires SciPy; install 'litearm-python[ik]'"
        ) from exc
    jacobian = np.asarray(jacobian, float)
    target = np.asarray(twist, float)
    lo, hi = np.asarray(lower, float), np.asarray(upper, float)
    if (jacobian.ndim != 2 or target.shape != (jacobian.shape[0],) or
            lo.shape != (jacobian.shape[1],) or hi.shape != lo.shape or
            not all(np.all(np.isfinite(x)) for x in (jacobian, target, lo, hi)) or
            np.any(lo > hi) or not np.isfinite(damping) or damping <= 0):
        raise ValueError("invalid bounded task velocity inputs")
    free = hi > lo
    velocity = lo.copy()
    if np.any(free):
        count = int(np.sum(free))
        matrix = np.vstack((jacobian[:, free], damping * np.eye(count)))
        rhs = np.concatenate((target - jacobian[:, ~free] @ velocity[~free],
                              np.zeros(count)))
        result = lsq_linear(matrix, rhs, bounds=(lo[free], hi[free]),
                            method="bvls", tol=1e-10, max_iter=100)
        velocity[free] = np.clip(result.x, lo[free], hi[free])
    return velocity


def describe_failure(solution, side="", binding=None, sigma_min=None,
                     near_singular_sigma=None):
    """Return the actionable operator message used by lite-vr."""
    threshold = (NEAR_SINGULAR_SIGMA if near_singular_sigma is None
                 else near_singular_sigma)
    tag = "[ok]" if solution.failure == FAIL_NONE else f"[{solution.failure}]"
    if side:
        tag = f"[{side}][{solution.failure}]"
    if solution.failure == FAIL_BOX:
        text = (f"{tag} 关节 {solution.worst_joint + 1} 需要的关节速度比该位姿允许的多 "
                f"{solution.worst_excess:.2f} rad/s，且冗余方向上没有任何位置能让它合法。")
        if binding == "speed":
            text += "起作用的是关节速度上限：放慢手速，或调低手柄到臂的映射增益。"
        elif binding == "range":
            text += "起作用的是关节行程：该关节已到行程末端。把手往工作空间内移。"
        if sigma_min is not None:
            text += ("该位姿接近奇异（σ_min 低），把手往奇异点外移。"
                     if sigma_min < threshold else
                     "该位姿不接近奇异，这一次不是奇异点的问题。")
        return text
    if solution.failure == FAIL_NO_JACOBIAN:
        return f"{tag} 此位姿的雅可比没有任何可用方向。把手往奇异点外移。"
    if solution.failure == FAIL_RESIDUAL:
        return f"{tag} 命令的 twist 有 {100.0 * solution.residual:.1f}% 在此位姿产生不了。"
    if solution.failure == FAIL_SHAPE:
        return f"{tag} 数组互相不匹配或含非法数值。"
    return f"{tag} twist 已被精确交付。"


__all__ = [
    "FAIL_NONE", "FAIL_SHAPE", "FAIL_NO_JACOBIAN", "FAIL_BOX",
    "FAIL_RESIDUAL", "NEAR_SINGULAR_SIGMA", "TaskVelocitySolution",
    "null_space_interval", "exact_task_velocity", "bounded_task_velocity", "box_binding",
    "describe_failure",
]
