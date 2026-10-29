# -*- coding: utf-8 -*-
"""逆动力学 —— 固件 `dyn.c::rnea_core_m` 的逐行 Python 移植。

世界系牛顿-欧拉，9 刚体串行链，重力通过 base 线加速度 −g 注入
(与 pinocchio `rnea` 同符号)。输出是**关节侧**力矩：

    RNEA(q, dq, ddq) = M(q)·ddq + C(q,dq)·dq + G(q)

固件的 `dyn_rnea` 额外加 `jm·ddq`；这里保持内核纯净，由 `theory_torques()`
按固件口径把 jm 单列，方便报告拆分误差来源。

⚠ M 不保证正定 (模型是辨识回归量)，本模块不提供求逆。
"""
from __future__ import annotations

import math
from typing import Dict, List, Sequence

from .model import (DYN_AXIS, DYN_FIX_RPY, DYN_FIX_XYZ, DYN_JTYPE, DYN_PARENT,
                    N_JOINT, Body, ModelSnapshot, rpy_to_matrix,
                    sym6_to_matrix)


def _matmul(a: Sequence[float], b: Sequence[float]) -> List[float]:
    return [
        a[0] * b[0] + a[1] * b[3] + a[2] * b[6],
        a[0] * b[1] + a[1] * b[4] + a[2] * b[7],
        a[0] * b[2] + a[1] * b[5] + a[2] * b[8],
        a[3] * b[0] + a[4] * b[3] + a[5] * b[6],
        a[3] * b[1] + a[4] * b[4] + a[5] * b[7],
        a[3] * b[2] + a[4] * b[5] + a[5] * b[8],
        a[6] * b[0] + a[7] * b[3] + a[8] * b[6],
        a[6] * b[1] + a[7] * b[4] + a[8] * b[7],
        a[6] * b[2] + a[7] * b[5] + a[8] * b[8],
    ]


def _matmul_t(a: Sequence[float], b: Sequence[float]) -> List[float]:
    """a · bᵀ (行主序 3x3)。"""
    return [
        a[0] * b[0] + a[1] * b[1] + a[2] * b[2],
        a[0] * b[3] + a[1] * b[4] + a[2] * b[5],
        a[0] * b[6] + a[1] * b[7] + a[2] * b[8],
        a[3] * b[0] + a[4] * b[1] + a[5] * b[2],
        a[3] * b[3] + a[4] * b[4] + a[5] * b[5],
        a[3] * b[6] + a[4] * b[7] + a[5] * b[8],
        a[6] * b[0] + a[7] * b[1] + a[8] * b[2],
        a[6] * b[3] + a[7] * b[4] + a[8] * b[5],
        a[6] * b[6] + a[7] * b[7] + a[8] * b[8],
    ]


def _matvec(m: Sequence[float], v: Sequence[float]) -> List[float]:
    return [m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
            m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
            m[6] * v[0] + m[7] * v[1] + m[8] * v[2]]


def _cross(a: Sequence[float], b: Sequence[float]) -> List[float]:
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]


def _zrot(th: float) -> List[float]:
    c, s = math.cos(th), math.sin(th)
    return [c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0]


def _flatten(m: Sequence[Sequence[float]]) -> List[float]:
    return [m[0][0], m[0][1], m[0][2],
            m[1][0], m[1][1], m[1][2],
            m[2][0], m[2][1], m[2][2]]


#: 固定旋转预算成行主序 9 元组 —— 热路径上每次 RNEA 都要用。
_FIX_IO = [tuple(_flatten(rpy_to_matrix(rpy))) for rpy in DYN_FIX_RPY]


def _sym6_flat(i6: Sequence[float]) -> List[float]:
    return _flatten(sym6_to_matrix(i6))


def rnea_core(q: Sequence[float], dq: Sequence[float], ddq: Sequence[float],
              bodies: Sequence[Body], gravity: Sequence[float] = (0.0, 0.0, -9.81),
              gravity_on: bool = True) -> List[float]:
    """完整 RNEA（不含 jm）。输入长度必须为 7。"""
    if len(q) != N_JOINT or len(dq) != N_JOINT or len(ddq) != N_JOINT:
        raise ValueError(f"q/dq/ddq 都需要 {N_JOINT} 个")
    if len(bodies) != 9:
        raise ValueError("bodies 需要 9 个刚体")

    Rw: List[List[float]] = [[0.0] * 9 for _ in range(9)]
    p: List[List[float]] = [[0.0, 0.0, 0.0] for _ in range(9)]
    u: List[List[float]] = [[0.0, 0.0, 0.0] for _ in range(9)]
    w: List[List[float]] = [[0.0, 0.0, 0.0] for _ in range(9)]
    al: List[List[float]] = [[0.0, 0.0, 0.0] for _ in range(9)]
    alin: List[List[float]] = [[0.0, 0.0, 0.0] for _ in range(9)]
    f: List[List[float]] = [[0.0, 0.0, 0.0] for _ in range(9)]
    npi: List[List[float]] = [[0.0, 0.0, 0.0] for _ in range(9)]

    Rw[0] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    if gravity_on:
        alin[0] = [-float(gravity[0]), -float(gravity[1]), -float(gravity[2])]

    for i in range(1, 9):
        pa = DYN_PARENT[i]
        rev = DYN_JTYPE[i] == 0
        sgn = 1.0 if DYN_AXIS[i][2] >= 0.0 else -1.0
        rwpf = _matmul(Rw[pa], _FIX_IO[i])
        if rev:
            Rw[i] = _matmul(rwpf, _zrot(sgn * float(q[i - 1])))
        else:
            Rw[i] = rwpf
        tmp = _matvec(Rw[pa], DYN_FIX_XYZ[i])
        p[i] = [p[pa][0] + tmp[0], p[pa][1] + tmp[1], p[pa][2] + tmp[2]]

        if rev:
            u[i] = _matvec(Rw[i], [0.0, 0.0, sgn])
            dqi = float(dq[i - 1])
            uq = [u[i][0] * dqi, u[i][1] * dqi, u[i][2] * dqi]
            w[i] = [w[pa][0] + uq[0], w[pa][1] + uq[1], w[pa][2] + uq[2]]
            cuq = _cross(w[pa], uq)
            ai = float(ddq[i - 1])
            al[i] = [al[pa][0] + u[i][0] * ai + cuq[0],
                     al[pa][1] + u[i][1] * ai + cuq[1],
                     al[pa][2] + u[i][2] * ai + cuq[2]]
        else:
            w[i] = list(w[pa])
            al[i] = list(al[pa])

        r = [p[i][0] - p[pa][0], p[i][1] - p[pa][1], p[i][2] - p[pa][2]]
        ar = _cross(al[pa], r)
        wr = _cross(w[pa], r)
        wwx = _cross(w[pa], wr)
        alin[i] = [alin[pa][0] + ar[0] + wwx[0],
                   alin[pa][1] + ar[1] + wwx[1],
                   alin[pa][2] + ar[2] + wwx[2]]

        body = bodies[i]
        comw_tmp = _matvec(Rw[i], body.com)
        comw = [p[i][0] + comw_tmp[0], p[i][1] + comw_tmp[1], p[i][2] + comw_tmp[2]]
        ro = [comw[0] - p[i][0], comw[1] - p[i][1], comw[2] - p[i][2]]
        a_cross = _cross(al[i], ro)
        wx = _cross(w[i], ro)
        wwx2 = _cross(w[i], wx)
        ac = [alin[i][0] + a_cross[0] + wwx2[0],
              alin[i][1] + a_cross[1] + wwx2[1],
              alin[i][2] + a_cross[2] + wwx2[2]]

        f[i] = [body.mass * ac[0], body.mass * ac[1], body.mass * ac[2]]

        It = _sym6_flat(body.inertia)
        Iw = _matmul_t(_matmul(Rw[i], It), Rw[i])
        Iw_al = _matvec(Iw, al[i])
        Iw_w = _matvec(Iw, w[i])
        c3 = _cross(w[i], Iw_w)
        n = [Iw_al[0] + c3[0], Iw_al[1] + c3[1], Iw_al[2] + c3[2]]
        rof = _cross(ro, f[i])
        npi[i] = [n[0] + rof[0], n[1] + rof[1], n[2] + rof[2]]

    tau = [0.0] * N_JOINT
    Ftot = [0.0, 0.0, 0.0]
    Macc = [0.0, 0.0, 0.0]
    for i in range(8, 0, -1):
        if i < 8:
            off = [p[i + 1][0] - p[i][0], p[i + 1][1] - p[i][1], p[i + 1][2] - p[i][2]]
            d1 = _cross(off, Ftot)
            Macc = [Macc[0] + d1[0], Macc[1] + d1[1], Macc[2] + d1[2]]
        mi = [npi[i][0] + Macc[0], npi[i][1] + Macc[1], npi[i][2] + Macc[2]]
        Ftot = [f[i][0] + Ftot[0], f[i][1] + Ftot[1], f[i][2] + Ftot[2]]
        if DYN_JTYPE[i] == 0:
            tau[i - 1] = u[i][0] * mi[0] + u[i][1] * mi[1] + u[i][2] * mi[2]
        Macc = mi
    return tau


def gravity_torque(q: Sequence[float], bodies: Sequence[Body],
                   gravity: Sequence[float] = (0.0, 0.0, -9.81)) -> List[float]:
    z = [0.0] * N_JOINT
    return rnea_core(q, z, z, bodies, gravity, gravity_on=True)


def coriolis_torque(q: Sequence[float], dq: Sequence[float],
                    bodies: Sequence[Body],
                    gravity: Sequence[float] = (0.0, 0.0, -9.81)) -> List[float]:
    z = [0.0] * N_JOINT
    full = rnea_core(q, dq, z, bodies, gravity, gravity_on=True)
    g = rnea_core(q, z, z, bodies, gravity, gravity_on=True)
    return [full[i] - g[i] for i in range(N_JOINT)]


def inertia_torque(q: Sequence[float], ddq: Sequence[float],
                   bodies: Sequence[Body]) -> List[float]:
    z = [0.0] * N_JOINT
    return rnea_core(q, z, ddq, bodies, (0.0, 0.0, 0.0), gravity_on=False)


def mass_matrix(q: Sequence[float], bodies: Sequence[Body]) -> List[List[float]]:
    """M(q)，含 jm？不 —— 这里只返回刚体项；jm 由调用方加到对角。"""
    z = [0.0] * N_JOINT
    M = [[0.0] * N_JOINT for _ in range(N_JOINT)]
    for j in range(N_JOINT):
        e = [0.0] * N_JOINT
        e[j] = 1.0
        col = rnea_core(q, z, e, bodies, (0.0, 0.0, 0.0), gravity_on=False)
        for i in range(N_JOINT):
            M[i][j] = col[i]
    for i in range(N_JOINT):
        for j in range(i + 1, N_JOINT):
            v = 0.5 * (M[i][j] + M[j][i])
            M[i][j] = v
            M[j][i] = v
    return M


def theory_torques(snapshot: ModelSnapshot, q: Sequence[float],
                   dq: Sequence[float], ddq: Sequence[float],
                   apply_scales: bool = False) -> Dict[str, List[float]]:
    """逐项理论力矩。

    返回 `gravity/coriolis/inertia/jm/friction/total/total_scaled`。
    `total` 是物理量 (M·ddq + C·dq + G + jm·ddq + friction)，不乘标定系数；
    `total_scaled` 按固件前馈口径乘 gravity_scale / inertia_scale。
    """
    bodies = snapshot.effective_bodies()
    G = gravity_torque(q, bodies, snapshot.gravity)
    C = coriolis_torque(q, dq, bodies, snapshot.gravity)
    I = inertia_torque(q, ddq, bodies)
    J = [snapshot.jm[i] * float(ddq[i]) for i in range(N_JOINT)]
    F = snapshot.friction_torque(dq)
    total = [G[i] + C[i] + I[i] + J[i] + F[i] for i in range(N_JOINT)]
    scaled = [snapshot.gravity_scale[i] * G[i]
              + snapshot.inertia_scale[i] * (C[i] + I[i] + J[i])
              + F[i] for i in range(N_JOINT)]
    return {"gravity": G, "coriolis": C, "inertia": I, "jm": J,
            "friction": F, "total": total, "total_scaled": scaled}
