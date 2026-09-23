"""旋转/位姿小工具 —— **纯 Python, 不依赖 numpy**。

约定与**固件 `kin.c` 完全一致**(ZYX 内旋), 也与 pylitearm 的 `control/cartesian.py`
一致。三条互不相同的表示在本包来回转, 别搞混:

===========  ==================================  ==============================
表示          形态                                 谁在用
===========  ==================================  ==============================
6 向量        ``(x, y, z, roll, pitch, yaw)``     固件 / 本包 ``get_tcp`` / ``move_p``
位姿对        ``(position[3], rotation[3x3])``     pylitearm 的 ``movel/movec/movep``
旋转矩阵      ``R[3][3]`` 行主序嵌套 list            内部统一用这个算
===========  ==================================  ==============================

``as_pose()`` 是唯一的入口, 上面前两种都吃, 免得调用方自己去想该传什么。
"""
from __future__ import annotations

import math
from typing import List, Sequence, Tuple

Mat3 = List[List[float]]
Vec3 = List[float]
#: 归一化后的位姿: (位置[3], 旋转矩阵[3x3])
Pose = Tuple[Vec3, Mat3]

_EPS = 1e-12


# --------------------------------------------------------------------------
# 基本 3x3 运算
# --------------------------------------------------------------------------
def eye3() -> Mat3:
    return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def mat_mul(A: Sequence[Sequence[float]], B: Sequence[Sequence[float]]) -> Mat3:
    return [[A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j]
             for j in range(3)] for i in range(3)]


def mat_T(A: Sequence[Sequence[float]]) -> Mat3:
    return [[A[j][i] for j in range(3)] for i in range(3)]


def mat_vec(A: Sequence[Sequence[float]], v: Sequence[float]) -> Vec3:
    return [A[i][0] * v[0] + A[i][1] * v[1] + A[i][2] * v[2] for i in range(3)]


def dot(a: Sequence[float], b: Sequence[float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]


def norm(v: Sequence[float]) -> float:
    return math.sqrt(dot(v, v))


def sub(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def unit(v: Sequence[float]) -> Vec3:
    n = norm(v)
    if n < _EPS:
        raise ValueError("零向量无法归一化")
    return [v[0] / n, v[1] / n, v[2] / n]


# --------------------------------------------------------------------------
# RPY <-> 旋转矩阵 (ZYX 内旋, 与固件 kin_rpy_to_rot / kin_rot_to_rpy 同约定 ——
# `kin.c:343-358` / `kin.c:360-385`; 固件里没有 `kin_rpy_to_rpy` 这个名字)
# --------------------------------------------------------------------------
def rpy_to_mat(rpy: Sequence[float]) -> Mat3:
    """``(roll, pitch, yaw)`` -> R, 即 ``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)``。

    与固件 `kin_rpy_to_rot` (`kin.c:343-358`) 逐元素同式 —— 别再自己推一遍,
    推错方向不会报错, 只会让所有姿态静默反着转。
    """
    r, p, y = (float(v) for v in rpy)
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp,     cp * sr,                cp * cr]]


def mat_to_rpy(R: Sequence[Sequence[float]]) -> Vec3:
    """R -> ``(roll, pitch, yaw)``。万向锁 (|pitch|≈π/2) 时**强制 yaw=0** —— 与固件
    `kin_rot_to_rpy` 的锁分支同约定 (`kin.c:380-384`: `rpy[2] = 0.0f`)。
    ⚠ 只保证"锁上强制 yaw=0"这条**约定**一致 —— 两侧的**锁带宽度不同**: 本函数用
    `|pitch|` 距 `π/2` 小于 `1e-6`, 固件用 `|cos(pitch)| <= 1e-4` (`kin.c:362`/`:376`),
    即固件的锁带约宽 100 倍。两者之间的窄带里, 两边走的分支可以不同。

    ⚠ 这条强制是 `move_p` 到位判定里那条「旋转等价」补丁的由来: 同一个旋转在万向锁
    附近可以给出差很远的 rpy 分量, 拿分量逐个比会**永远判不到位**。
    """
    p = math.atan2(-R[2][0], math.hypot(R[0][0], R[1][0]))
    if abs(abs(p) - math.pi / 2) < 1e-6:
        return [math.atan2(-R[1][2], R[1][1]), p, 0.0]
    return [math.atan2(R[2][1], R[2][2]), p, math.atan2(R[1][0], R[0][0])]


# --------------------------------------------------------------------------
# SO(3) 上的指数/对数 —— slerp 与姿态误差的基础
# --------------------------------------------------------------------------
def rot_log(R: Sequence[Sequence[float]]) -> Vec3:
    """R -> 旋转向量 (轴×角)。``acos`` 的入参必须夹住: 数值噪声会让它越界到
    ±(1+ε) 然后抛 `ValueError: math domain error`。"""
    tr = R[0][0] + R[1][1] + R[2][2]
    c = max(-1.0, min(1.0, (tr - 1.0) / 2.0))
    ang = math.acos(c)
    if ang < 1e-9:
        return [0.0, 0.0, 0.0]
    v = [R[2][1] - R[1][2], R[0][2] - R[2][0], R[1][0] - R[0][1]]
    k = ang / (2.0 * math.sin(ang))
    return [v[0] * k, v[1] * k, v[2] * k]


def rot_exp(w: Sequence[float]) -> Mat3:
    """旋转向量 -> R (Rodrigues)。"""
    ang = norm(w)
    if ang < 1e-12:
        return eye3()
    k = [w[0] / ang, w[1] / ang, w[2] / ang]
    K = [[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]]
    s, c = math.sin(ang), 1.0 - math.cos(ang)
    KK = mat_mul(K, K)
    return [[(1.0 if i == j else 0.0) + s * K[i][j] + c * KK[i][j]
             for j in range(3)] for i in range(3)]


def rot_slerp(R0: Sequence[Sequence[float]], R1: Sequence[Sequence[float]],
              t: float) -> Mat3:
    """姿态球面插值。``R0 @ exp(t·log(R0ᵀ R1))`` —— 走**最短弧**, 也是 pylitearm
    `plan_movel` 里那行 `_kin.rot_slerp` 的同式。"""
    w = rot_log(mat_mul(mat_T(R0), R1))
    if norm(w) < 1e-12:
        return [list(row) for row in R0]
    return mat_mul(R0, rot_exp([w[0] * t, w[1] * t, w[2] * t]))


def rot_angle(R: Sequence[Sequence[float]]) -> float:
    """旋转矩阵对应的转角 (rad)。"""
    return norm(rot_log(R))


# --------------------------------------------------------------------------
# 位姿
# --------------------------------------------------------------------------
def pose_err(pos: Sequence[float], R: Sequence[Sequence[float]],
             pos_d: Sequence[float], R_d: Sequence[Sequence[float]]
             ) -> Tuple[float, float]:
    """``(位置误差 m, 姿态误差 rad)``。语义同 pylitearm `pose_error`。

    ⚠ 姿态差必须是 ``rot_log(R_d @ Rᵀ)``, **不能**写成 ``rot_log(R @ R_dᵀ)`` —— 后者
    是复合不是差, 符号相反; 曾因此把 0.59mm 位置误差的位姿报成 153° 姿态误差
    (那个 153.359° 恰好是该位姿转角的 2 倍: 153.359/2 = 76.68°)。
    """
    ep = norm(sub(pos_d, pos))
    er = rot_angle(mat_mul(R_d, mat_T(R)))
    return ep, er


def is_rotation(R: Sequence[Sequence[float]], atol: float = 1e-5) -> bool:
    """是不是合法的 SO(3) 成员 (正交 + det≈+1)。照 pylitearm `_validate_pose` 的判据。"""
    Rt = mat_T(R)
    I3 = eye3()
    for i in range(3):
        for j in range(3):
            if abs(sum(Rt[i][k] * R[k][j] for k in range(3)) - I3[i][j]) > atol:
                return False
    det = (R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1])
           - R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0])
           + R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0]))
    return det > 0.999


def _is_sequence(x) -> bool:
    return isinstance(x, (list, tuple)) and not isinstance(x, (str, bytes))


def as_pose(pose) -> Pose:
    """把调用方给的东西归一化成 ``(position[3], R[3x3])``。

    接受(顺序即判定顺序):

    * ``(pos[3], R[3x3])`` —— pylitearm 原生形式, 与 `_validate_pose` 同判据
    * ``[x, y, z, roll, pitch, yaw]`` —— 固件/本包 `get_tcp` 的形式
    * ``R[4][4]`` 齐次矩阵 —— 取左上 3x3 与右上平移

    **不接受**扁平 9 元素 (按行主序的旋转矩阵) —— 它没有位置, 而位置不可省
    (见下面"形式 4"那个分支的注释); 也不接受任何缺分量的写法。

    不合法一律 `InvalidCommandError`, 文案里给出**实际收到的形状** —— 只说
    "pose 非法" 会让调用方在 6 向量和位姿对之间反复猜。

    ⚠ 但**数值转换**里的畸形 (`float("x")` 的 `ValueError`、容器类型不对的 `TypeError`)
    是从这里**原样冒出去**的 (本函数的契约只覆盖形状判定), 由调用方负责收进
    `LiteArmError` 体系 —— 见 `cart._as_pose6` (它把这两类一并转包成
    `InvalidCommandError`)。
    """
    from litearm.errors import InvalidCommandError

    if not _is_sequence(pose):
        raise InvalidCommandError(
            f"pose 需为 (pos[3], R[3x3]) 或 [x,y,z,r,p,y]; 收到 {type(pose).__name__}")

    items = list(pose)
    # 形式 1: (pos[3], R[3x3])
    if len(items) == 2 and _is_sequence(items[0]) and _is_sequence(items[1]):
        p, R = items
        if len(list(p)) != 3:
            raise InvalidCommandError(f"pose 位置需 3 个分量, 收到 {len(list(p))}")
        Rl = [list(row) for row in R]
        if len(Rl) != 3 or any(len(row) != 3 for row in Rl):
            raise InvalidCommandError(
                f"pose 旋转需 3x3, 收到 {len(Rl)}x{len(Rl[0]) if Rl else 0}")
        Rf = [[float(v) for v in row] for row in Rl]
        if not is_rotation(Rf):
            raise InvalidCommandError("pose 旋转矩阵不是有效 SO(3) (正交且 det≈+1)")
        return [float(v) for v in p], Rf

    # 形式 2: 6 向量 xyz+rpy
    if len(items) == 6 and not any(_is_sequence(v) for v in items):
        v = [float(x) for x in items]
        return v[:3], rpy_to_mat(v[3:])

    # 形式 3: 齐次 4x4
    if len(items) == 4 and all(_is_sequence(r) and len(list(r)) == 4 for r in items):
        M = [[float(v) for v in row] for row in items]
        Rf = [row[:3] for row in M[:3]]
        if not is_rotation(Rf):
            raise InvalidCommandError("pose 4x4 的旋转块不是有效 SO(3)")
        return [M[0][3], M[1][3], M[2][3]], Rf

    # 形式 4: 扁平 9 元素旋转矩阵 (无位置) —— 不接受, 位置不可省
    raise InvalidCommandError(
        f"pose 形状无法识别: len={len(items)}; 支持 (pos[3], R[3x3]) / "
        f"[x,y,z,r,p,y] / 4x4 齐次矩阵")
