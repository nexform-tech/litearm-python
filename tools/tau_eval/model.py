# -*- coding: utf-8 -*-
"""动力学模型快照 —— 固件模型参数 (0x30/0x33/0x34) 或 URDF 的纯 Python 表示。

本模块只做数据：刚体 (m/com/I)、jm、payload、重力、缩放系数与摩擦参数。
递推在 `rnea.py`；采集在 `collect.py`；分析在 `analyze.py`。

几何常量来自固件 `tools/export_model.py` 从
`assets/litearm/urdf/litearm_id.urdf` 生成的 `dyn_model_data.h`
(sha256[:16] = 640bf04bf532e395)。固件只在线暴露惯性参数与 jm，
**运动学链 (固定位姿/关节轴) 不提供在线读取**，所以这里内嵌同一份常量，
并用 `from_urdf()` 支持从 URDF 重算核对。

⚠ 固件模型是辨识得到的**回归量式**参数：link mass 多为 1、惯量含负值、
M 不是正定矩阵。τ = M·q̈ + C·q̇ + G 仍然正确，但**不要对 M 求逆**。
本工具不含任何求逆路径。
"""
from __future__ import annotations

import hashlib
import math
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

# ---------------------------------------------------------------------------
# 运动学几何 (dyn_model_data.h)
# ---------------------------------------------------------------------------

#: `tool/geometry.py` 内嵌常量的来源指纹；`from_urdf()` 会重算并记录实际文件的 sha。
GEOMETRY_SHA16 = "640bf04bf532e395"

DYN_PARENT = (-1, 0, 1, 2, 3, 4, 5, 6, 7)
#: -1 = base, 0 = revolute (有 DOF), 1 = fixed
DYN_JTYPE = (-1, 0, 0, 0, 0, 0, 0, 0, 1)
DYN_AXIS = (
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 1.0),
    (0.0, 0.0, 1.0),
    (0.0, 0.0, 1.0),
    (0.0, 0.0, -1.0),
    (0.0, 0.0, 1.0),
    (0.0, 0.0, -1.0),
    (0.0, 0.0, -1.0),
    (0.0, 0.0, 0.0),
)
DYN_FIX_XYZ = (
    (0.0, 0.0, 0.0),
    (-2.23694604e-07, 7.53497322e-07, 0.13324993),
    (0.0, 0.0, 0.0),
    (0.0, -0.2995, 0.0),
    (0.038, 0.0, 0.0),
    (-0.038, -0.2225, 0.0),
    (0.0, 0.0, 0.0),
    (-0.074, 0.0, 0.0),
    (-0.053749853, 6.97627774e-07, 1.77373939e-06),
)
DYN_FIX_RPY = (
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 1.57079265),
    (-1.57079633, 0.0, 0.0),
    (1.57079633, 0.0, 0.0),
    (-1.57079633, 0.0, 0.0),
    (1.57079633, 0.0, 0.0),
    (0.0, 1.57079633, 1.57079633),
    (1.57079633, 0.0, 0.0),
    (0.0, -1.5707881, -3.46291658e-06),
)
DYN_MASS = (0.364380717, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0)
DYN_COM = (
    (2.81752473e-05, -6.21980479e-07, 0.0355224095),
    (0.00387153474, 0.0, 0.0),
    (0.0527914515, 0.648081281, 0.0),
    (-0.0681886379, -0.00784773695, 0.0),
    (0.0320031263, 0.304352812, 0.0),
    (4.775927e-05, -3.12025358e-05, 0.0),
    (0.0242589492, 0.000115404839, 0.0),
    (-0.0033098347, 9.95219448e-05, 0.0),
    (0.0, 0.0, 0.0),
)
DYN_INERTIA = (
    (0.000273641151, 1.67098714e-08, -1.09565995e-08, 0.000251821003, -3.3686319e-09, 0.000377759326),
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    (-1.17575709, 0.0255899977, -0.0223129781, -0.00278693735, 0.0211291862, -1.22121557),
    (-0.0884669606, -0.00697576894, -0.0269419248, -0.00464969033, 0.0382667169, 0.0568640218),
    (-0.411442517, 0.0141869443, -1.53041911e-05, -0.00102420009, -0.0134242531, -0.439410942),
    (-0.0203443616, 0.00585590416, -0.00562145578, -2.28094787e-09, 6.44438452e-06, -0.0360883733),
    (0.0357078987, -0.000649135069, -0.000377149847, -0.000588496614, 5.6738306e-05, -0.0191189771),
    (-0.0028817802, -0.000974346793, -0.000418232835, -1.09550057e-05, -0.000162816495, 0.00673921812),
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
)
#: 出厂 jm (dyn_jm_default.h)。同样是辨识基系数，不是物理转子惯量。
DYN_JM_DEFAULT = (0.0, 0.0, -0.0188174839, -0.0236409545,
                  -0.00612815024, -0.0196070617, -0.0142279878)

N_BODY = 9
N_JOINT = 7


# ---------------------------------------------------------------------------
# 小型线性代数 / 旋转
# ---------------------------------------------------------------------------

def rpy_to_matrix(rpy: Sequence[float]) -> List[List[float]]:
    """URDF rpy -> R = Rz(y)·Ry(p)·Rx(r)。"""
    r, p, y = (float(v) for v in rpy)
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def sym6_to_matrix(i6: Sequence[float]) -> List[List[float]]:
    ixx, ixy, ixz, iyy, iyz, izz = (float(v) for v in i6)
    return [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]]


def matrix_to_sym6(m: Sequence[Sequence[float]]) -> Tuple[float, ...]:
    return (m[0][0], m[0][1], m[0][2], m[1][1], m[1][2], m[2][2])


def rotate_inertia(i6: Sequence[float], rpy: Sequence[float]) -> Tuple[float, ...]:
    """把惯量张量从 inertial 系旋到 link 系：I' = R·I·Rᵀ。"""
    if not any(abs(float(v)) > 1e-15 for v in rpy):
        return tuple(float(v) for v in i6)
    R = rpy_to_matrix(rpy)
    I = sym6_to_matrix(i6)
    IR = [[sum(I[i][k] * R[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
    out = [[sum(R[i][k] * IR[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    return matrix_to_sym6(out)


# ---------------------------------------------------------------------------
# 模型快照
# ---------------------------------------------------------------------------

@dataclass
class Body:
    """单个刚体的惯性参数 (link 系)。"""

    mass: float = 0.0
    com: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    inertia: Tuple[float, float, float, float, float, float] = (0.0,) * 6

    def as_list(self) -> List[float]:
        return [self.mass, *self.com, *self.inertia]

    @classmethod
    def from_vals(cls, vals: Sequence[float]) -> "Body":
        if len(vals) != 10:
            raise ValueError("body 参数必须是 10 个 (m, cx, cy, cz, ixx, ixy, ixz, iyy, iyz, izz)")
        v = [float(x) for x in vals]
        return cls(v[0], (v[1], v[2], v[3]), (v[4], v[5], v[6], v[7], v[8], v[9]))


@dataclass
class ModelSnapshot:
    """一份可落盘、可离线复用的动力学模型。"""

    bodies: List[Body]
    jm: List[float]
    payload_mass: float = 0.0
    payload_com: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    gravity: Tuple[float, float, float] = (0.0, 0.0, -9.81)
    gravity_scale: List[float] = field(default_factory=lambda: [1.0] * N_JOINT)
    inertia_scale: List[float] = field(default_factory=lambda: [1.0] * N_JOINT)
    friction_model: int = 1
    friction_v: List[float] = field(default_factory=lambda: [0.0] * N_JOINT)
    friction_fc0: List[float] = field(default_factory=lambda: [0.0] * N_JOINT)
    friction_fc1: List[float] = field(default_factory=lambda: [0.0] * N_JOINT)
    fric_v2_eps: float = 0.05
    ff_mask: Optional[int] = None
    firmware: str = ""
    joint_count: int = N_JOINT
    geometry_sha16: str = GEOMETRY_SHA16
    model_status: Dict[str, object] = field(default_factory=dict)
    read_warnings: List[str] = field(default_factory=list)

    def validate(self) -> None:
        if len(self.bodies) != N_BODY:
            raise ValueError(f"需要 {N_BODY} 个刚体, 实际 {len(self.bodies)}")
        if len(self.jm) != N_JOINT:
            raise ValueError(f"jm 需要 {N_JOINT} 个, 实际 {len(self.jm)}")
        for name in ("gravity_scale", "inertia_scale",
                     "friction_v", "friction_fc0", "friction_fc1"):
            if len(getattr(self, name)) != N_JOINT:
                raise ValueError(f"{name} 需要 {N_JOINT} 个")

    def effective_bodies(self) -> List[Body]:
        """bank + payload -> 生效层，与固件 `load_eff()` 同式。

        payload 只复合进 body8 (ee 固定体)，com 在 **ee_link 系**。
        body8 的 bank 质量必须为 0；和为 0/负时固件直接放弃复合，这里照做。
        """
        out = [Body(b.mass, b.com, b.inertia) for b in self.bodies]
        m1 = float(self.payload_mass)
        if not (m1 > 0.0):
            return out
        k = N_BODY - 1
        b = out[k]
        m0 = b.mass
        if not (m0 + m1 > 0.0):
            return out
        c0 = b.com
        c1 = self.payload_com
        c = tuple((m0 * c0[i] + m1 * c1[i]) / (m0 + m1) for i in range(3))
        I = list(b.inertia)
        for mass, r in ((m0, tuple(c0[i] - c[i] for i in range(3))),
                        (m1, tuple(c1[i] - c[i] for i in range(3)))):
            r2 = sum(v * v for v in r)
            I[0] += mass * (r2 - r[0] * r[0])
            I[1] += mass * (-r[0] * r[1])
            I[2] += mass * (-r[0] * r[2])
            I[3] += mass * (r2 - r[1] * r[1])
            I[4] += mass * (-r[1] * r[2])
            I[5] += mass * (r2 - r[2] * r[2])
        out[k] = Body(m0 + m1, c, tuple(I))
        return out

    def friction_torque(self, dq: Sequence[float]) -> List[float]:
        """固件 v2 摩擦前馈的稳态值 (以**实测** dq 为输入)。

        公式 (law.c `law_friction_v2`):
            th = tanh(dq/eps)
            tau = fv·dq + (fc0−fc1)/2·th + (fc0+fc1)/2·th²
        忽略 slew (稳态评估)；friction_model != 1 时返回 0 并由调用方在
        报告里标注「摩擦项未纳入」。固件对参数的钳制在这里同样执行。
        """
        if self.friction_model != 1:
            return [0.0] * N_JOINT
        eps = max(1e-3, float(self.fric_v2_eps))
        out = []
        for i in range(N_JOINT):
            fv = max(0.0, float(self.friction_v[i]))
            fc0 = max(0.0, float(self.friction_fc0[i]))
            fc1 = min(0.0, float(self.friction_fc1[i]))
            if fv == 0.0 and fc0 == 0.0 and fc1 == 0.0:
                out.append(0.0)
                continue
            th = math.tanh(float(dq[i]) / eps)
            out.append(fv * float(dq[i]) + 0.5 * (fc0 - fc1) * th
                       + 0.5 * (fc0 + fc1) * th * th)
        return out

    # ---- 序列化 ----
    def to_dict(self) -> Dict[str, object]:
        return {
            "geometry_sha16": self.geometry_sha16,
            "firmware": self.firmware,
            "joint_count": self.joint_count,
            "bodies": [b.as_list() for b in self.bodies],
            "jm": [float(v) for v in self.jm],
            "payload": {"mass": float(self.payload_mass),
                        "com": [float(v) for v in self.payload_com]},
            "gravity": [float(v) for v in self.gravity],
            "gravity_scale": [float(v) for v in self.gravity_scale],
            "inertia_scale": [float(v) for v in self.inertia_scale],
            "friction": {"model": int(self.friction_model),
                         "v": [float(v) for v in self.friction_v],
                         "fc0": [float(v) for v in self.friction_fc0],
                         "fc1": [float(v) for v in self.friction_fc1],
                         "v2_eps": float(self.fric_v2_eps)},
            "ff_mask": None if self.ff_mask is None else int(self.ff_mask),
            "model_status": self.model_status,
            "read_warnings": list(self.read_warnings),
        }

    @classmethod
    def from_dict(cls, d: Dict[str, object]) -> "ModelSnapshot":
        fr = d.get("friction") or {}
        pay = d.get("payload") or {}
        snap = cls(
            bodies=[Body.from_vals(v) for v in d["bodies"]],
            jm=[float(v) for v in d["jm"]],
            payload_mass=float(pay.get("mass", 0.0)),
            payload_com=tuple(float(v) for v in pay.get("com", (0.0, 0.0, 0.0))),
            gravity=tuple(float(v) for v in d.get("gravity", (0.0, 0.0, -9.81))),
            gravity_scale=[float(v) for v in d.get("gravity_scale", [1.0] * N_JOINT)],
            inertia_scale=[float(v) for v in d.get("inertia_scale", [1.0] * N_JOINT)],
            friction_model=int(fr.get("model", 1)),
            friction_v=[float(v) for v in fr.get("v", [0.0] * N_JOINT)],
            friction_fc0=[float(v) for v in fr.get("fc0", [0.0] * N_JOINT)],
            friction_fc1=[float(v) for v in fr.get("fc1", [0.0] * N_JOINT)],
            fric_v2_eps=float(fr.get("v2_eps", 0.05)),
            ff_mask=None if d.get("ff_mask") is None else int(d["ff_mask"]),
            firmware=str(d.get("firmware", "")),
            joint_count=int(d.get("joint_count", N_JOINT)),
            geometry_sha16=str(d.get("geometry_sha16", GEOMETRY_SHA16)),
            model_status=dict(d.get("model_status") or {}),
            read_warnings=list(d.get("read_warnings") or []),
        )
        snap.validate()
        return snap


def factory_snapshot() -> ModelSnapshot:
    """编译期默认模型 (与 `dyn_model_data.h` / `dyn_jm_default.h` 同值)。

    只在离线自测 / URDF 不可用且固件读不到时使用；正常采集一律以固件读回为准。
    """
    bodies = [Body(DYN_MASS[i], DYN_COM[i], DYN_INERTIA[i]) for i in range(N_BODY)]
    return ModelSnapshot(bodies=bodies, jm=list(DYN_JM_DEFAULT),
                         firmware="factory-defaults")


# ---------------------------------------------------------------------------
# 在线读取固件模型
# ---------------------------------------------------------------------------

def _read(arm, label: str, fn, default, warnings: List[str]):
    try:
        return fn()
    except Exception as e:                       # noqa: BLE001 - 读不到就降级并记账
        warnings.append(f"{label}: {type(e).__name__}: {e}")
        return default


def fetch_from_arm(arm, timeout: float = 1.0) -> ModelSnapshot:
    """把固件里的模型、payload、重力、缩放与摩擦参数读成快照。

    读取项全部是只读查询 (0x34/0x33/0x2B/0x2C/0x39 相关)；不写任何参数。
    固件版本过旧导致某项读不到时，用默认值并写进 `read_warnings`。
    """
    warnings: List[str] = []
    bodies = []
    for i in range(N_BODY):
        vals = _read(arm, f"model.get_body({i})",
                     lambda i=i: arm.model.get_body(i, timeout=timeout).value,
                     [0.0] * 10, warnings)
        if len(vals) != 10:
            vals = [0.0] * 10
        bodies.append(Body.from_vals(vals))
    jm = _read(arm, "model.get_jm",
               lambda: arm.model.get_jm(timeout=timeout).value,
               [0.0] * N_JOINT, warnings)

    payload_mass = _read(arm, "ff_scalar(4)",
                         lambda: arm.get_ff_scalar(4, 0, timeout=timeout).value,
                         0.0, warnings)
    payload_com = tuple(
        _read(arm, f"ff_scalar(5,{k})",
              lambda k=k: arm.get_ff_scalar(5, k, timeout=timeout).value,
              0.0, warnings) for k in range(3))
    gravity = tuple(
        _read(arm, f"ff_scalar(6,{k})",
              lambda k=k: arm.get_ff_scalar(6, k, timeout=timeout).value,
              0.0, warnings) for k in range(3))
    if all(abs(v) < 1e-9 for v in gravity):
        # 固件默认 s_g = (0,0,-9.81)；读回全 0 说明参数区没配置，按固件默认补上并记账。
        warnings.append("gravity 读回全 0，按固件默认 (0,0,-9.81) 处理")
        gravity = (0.0, 0.0, -9.81)

    gs = _read(arm, "ff_vec(7) gravity_scale",
               lambda: list(arm.get_ff_vec(7, timeout=timeout).value),
               [1.0] * N_JOINT, warnings)
    isc = _read(arm, "ff_vec(8) inertia_scale",
                lambda: list(arm.get_ff_vec(8, timeout=timeout).value),
                [1.0] * N_JOINT, warnings)
    fv = _read(arm, "ff_vec(9) friction_v",
               lambda: list(arm.get_ff_vec(9, timeout=timeout).value),
               [0.0] * N_JOINT, warnings)
    fc0 = _read(arm, "ff_vec(10) friction_fc0",
                lambda: list(arm.get_ff_vec(10, timeout=timeout).value),
                [0.0] * N_JOINT, warnings)
    fc1 = _read(arm, "ff_vec(11) friction_fc1",
                lambda: list(arm.get_ff_vec(11, timeout=timeout).value),
                [0.0] * N_JOINT, warnings)
    fmodel = _read(arm, "ff_scalar(7) friction_model",
                   lambda: int(round(arm.get_ff_scalar(7, 0, timeout=timeout).value)),
                   1, warnings)
    feps = _read(arm, "ff_scalar(8) fric_v2_eps",
                 lambda: float(arm.get_ff_scalar(8, 0, timeout=timeout).value),
                 0.05, warnings)
    ff_mask = _read(arm, "get_ff_mask", lambda: arm.get_ff_mask(timeout=timeout),
                    None, warnings)
    status = _read(arm, "model.status", lambda: arm.model.status(timeout=timeout).value,
                   None, warnings)

    snap = ModelSnapshot(
        bodies=bodies, jm=[float(v) for v in jm],
        payload_mass=float(payload_mass), payload_com=payload_com,
        gravity=gravity, gravity_scale=[float(v) for v in gs],
        inertia_scale=[float(v) for v in isc],
        friction_model=int(fmodel),
        friction_v=[float(v) for v in fv],
        friction_fc0=[float(v) for v in fc0],
        friction_fc1=[float(v) for v in fc1],
        fric_v2_eps=float(feps),
        ff_mask=None if ff_mask is None else int(ff_mask),
        firmware=str(getattr(arm, "firmware", "") or ""),
        joint_count=int(getattr(arm, "n", N_JOINT)),
        model_status=({"override": status.override,
                       "staged_mask": status.staged_mask,
                       "dirty": status.dirty} if status is not None else {}),
        read_warnings=warnings,
    )
    snap.validate()
    return snap


# ---------------------------------------------------------------------------
# URDF 解析 (离线模型 / 几何核对)
# ---------------------------------------------------------------------------

def _parse_floats(text: Optional[str], n: int, default: float = 0.0) -> List[float]:
    if not text:
        return [default] * n
    parts = [float(v) for v in text.replace(",", " ").split()]
    if len(parts) != n:
        raise ValueError(f"需要 {n} 个数, 得到 {text!r}")
    return parts


def _link_inertial(link) -> Body:
    ine = link.find("inertial")
    if ine is None:
        return Body()
    origin = ine.find("origin")
    xyz = _parse_floats(origin.get("xyz") if origin is not None else None, 3)
    rpy = _parse_floats(origin.get("rpy") if origin is not None else None, 3)
    mass_el = ine.find("mass")
    mass = float(mass_el.get("value", 0.0)) if mass_el is not None else 0.0
    inel = ine.find("inertia")
    if inel is None:
        i6 = (0.0,) * 6
    else:
        i6 = tuple(float(inel.get(k, 0.0)) for k in
                   ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"))
    return Body(mass, tuple(xyz), rotate_inertia(i6, rpy))


def from_urdf(path: str, jm: Optional[Sequence[float]] = None) -> ModelSnapshot:
    """从 URDF 串行链构造快照。

    要求结构与本臂一致：root + 7 个 revolute + 1 个 fixed 末端体。关节轴只允许
    沿局部 ±z (固件递推的约定)；轴位于 x/y 时直接报错，而不是静默算错。
    """
    with open(path, "rb") as f:
        raw = f.read()
    sha16 = hashlib.sha256(raw).hexdigest()[:16]
    root = ET.fromstring(raw)
    links = {l.get("name"): l for l in root.findall("link")}
    joints = root.findall("joint")
    child_of = {}
    joints_by_parent = {}
    for j in joints:
        p = j.find("parent").get("link")
        c = j.find("child").get("link")
        child_of[c] = p
        joints_by_parent.setdefault(p, []).append(j)
    roots = [name for name in links if name not in child_of]
    if len(roots) != 1:
        raise ValueError(f"URDF 需要单一根 link, 找到 {roots}")
    bodies = [_link_inertial(links[roots[0]])]
    cur = roots[0]
    while cur in joints_by_parent:
        children = joints_by_parent[cur]
        if len(children) != 1:
            raise ValueError(f"link {cur!r} 有 {len(children)} 个子关节，本工具只支持串行链")
        j = children[0]
        jtype = j.get("type")
        if jtype not in ("revolute", "continuous", "fixed"):
            raise ValueError(f"关节 {j.get('name')!r} 类型 {jtype!r} 不支持")
        origin = j.find("origin")
        xyz = _parse_floats(origin.get("xyz") if origin is not None else None, 3)
        rpy = _parse_floats(origin.get("rpy") if origin is not None else None, 3)
        axis_el = j.find("axis")
        axis = _parse_floats(axis_el.get("xyz") if axis_el is not None else None, 3,
                             default=0.0) if axis_el is not None else [0.0, 0.0, 1.0]
        if abs(axis[0]) > 1e-9 or abs(axis[1]) > 1e-9 or abs(abs(axis[2]) - 1.0) > 1e-6:
            raise ValueError(f"关节 {j.get('name')!r} 轴 {axis} 不是局部 ±z，固件递推不支持")
        child = j.find("child").get("link")
        bodies.append(_link_inertial(links[child]))
        cur = child
    if len(bodies) != N_BODY:
        raise ValueError(f"URDF 链得到 {len(bodies)} 个刚体，期望 {N_BODY}")
    if sum(1 for j in joints if j.get("type") != "fixed") != N_JOINT:
        raise ValueError(f"URDF 可动关节数不是 {N_JOINT}")
    snap = ModelSnapshot(bodies=bodies,
                         jm=list(jm) if jm is not None else [0.0] * N_JOINT,
                         firmware=f"urdf:{os.path.basename(path)}",
                         geometry_sha16=sha16)
    snap.validate()
    return snap


def compare_geometry(urdf_path: str) -> List[str]:
    """把 URDF 的固定位姿/轴与内嵌常量逐项比对，返回差异描述。

    用于回答「这份 URDF 是否和固件那套几何同源」。差异为空表示一致。
    """
    root = ET.fromstring(open(urdf_path, "rb").read())
    joints = root.findall("joint")
    child_of = {j.find("child").get("link"): j.find("parent").get("link") for j in joints}
    by_parent = {}
    for j in joints:
        by_parent.setdefault(j.find("parent").get("link"), []).append(j)
    roots = [l.get("name") for l in root.findall("link") if l.get("name") not in child_of]
    if len(roots) != 1:
        return [f"根 link 不唯一: {roots}"]
    diffs: List[str] = []
    cur = roots[0]
    idx = 1
    while cur in by_parent and idx < N_BODY:
        j = by_parent[cur][0]
        origin = j.find("origin")
        xyz = _parse_floats(origin.get("xyz") if origin is not None else None, 3)
        rpy = _parse_floats(origin.get("rpy") if origin is not None else None, 3)
        axis_el = j.find("axis")
        axis = _parse_floats(axis_el.get("xyz"), 3) if axis_el is not None else [0.0, 0.0, 1.0]
        for name, got, want in (
                ("xyz", xyz, DYN_FIX_XYZ[idx]), ("rpy", rpy, DYN_FIX_RPY[idx])):
            for k in range(3):
                if abs(got[k] - want[k]) > 5e-6:
                    diffs.append(f"body{idx} {name}[{k}]: urdf={got[k]:.9g} "
                                 f"firmware={want[k]:.9g}")
        sgn_urdf = 1.0 if axis[2] >= 0 else -1.0
        sgn_fw = 1.0 if DYN_AXIS[idx][2] >= 0 else -1.0
        if j.get("type") != "fixed" and sgn_urdf != sgn_fw:
            diffs.append(f"body{idx} axis 符号: urdf={sgn_urdf:+.0f} firmware={sgn_fw:+.0f}")
        cur = j.find("child").get("link")
        idx += 1
    if idx != N_BODY:
        diffs.append(f"URDF 链只走到 body{idx}，未覆盖全部 {N_BODY} 个刚体")
    return diffs
