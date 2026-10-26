"""末端负载辨识 —— 差分解 + 固件探针回归矩阵（纯 `math`，不引 numpy）。

夹上一个未知物体之后，臂自己怎么知道末端变重了？`set_payload()` 只是把**你告诉它**的
质量写进固件；质量从哪来，协议里没有答案。本模块就是那个答案：在几个静态构型上量出力矩，
与**空载基线**逐构型相减，把臂自身的模型误差消掉，剩下的就是负载的重力矩。

## 为什么必须差分

静止时 `τ_meas = G_arm(q) + G_load(q) + 常值偏置 + ε(q)`（`docs` 里那条静置方程）。
绝对法直接拿 `G_arm(q)` 去减，臂模型的误差（重力缩放、静摩擦偏置、辨识残差）会全部
冲进结果 —— 历史记录里 0.36 kg 被辨成 1.92 kg 走的就是这条路。同一台臂、同一组构型上
再测一遍**空载**并相减，那三项与构型无关的部分整体抵消。

## 回归矩阵从哪来（本模块的关键设计）

差分量与待求参数是**线性**关系：

    Δτ = Y_g(q)·[m, m·cx, m·cy, m·cz]

`Y_g` 是**几何**量，只依赖 `q` 与连杆几何。旧 SDK 用 pinocchio + URDF 算它 —— 本包刻意
不引 numpy、也不在 PC 侧持有动力学模型（见 `__init__` 的依赖边界），所以这里换成
**固件探针法**：

* 固件 `CMD_GET_GRAVITY(0x39)` 收**任意** `q[7]`、返回该构型下的重力项 `G(q)`，无门控；
* 固件算的 `G(q)` 读的是「bank 模型 + payload」的**生效层**，而 payload 只以**点质量**
  形式复合进 body8（`dyn.c` 里 body8 的 bank 质量恒 0）⇒
  `G(q | payload=(m,c)) − G(q | payload=0)` 正是这个点质量的重力矩；
* 于是读五组 payload（`0` / `(m, 0)` / `(m, 沿法兰系各轴 c)` 各一次），做差即得
  **四列 `Y_g`**，且与固件递推所用的模型同源 —— 没有第二份模型可以漂移。

⚠ **探针会扰动控制律**：`0x28` 写 payload 立即生效（`params.c` → `control_dyn_reload()`
→ `dyn_set_payload()`），而控制律每拍用 `gravity_scale·G(q)` 做重力前馈。所以探针期间
臂若在使能持位，它的前馈会凭空多出一份幻影负载力矩。为此探针质量取得很小
（:data:`PROBE_MASS` = 0.02 kg、:data:`PROBE_COM` = 0.2 m ⇒ 幻影重力矩 ≲ 0.14 N·m，
乘 `gravity_scale` 后仍 < 1 N·m，且每个探针只在一次请求/应答的窗口里存在）。
`finally` 里一定把原来的 payload 写回 —— 中途抛错也不会把台架留在幻影负载上。

## 用法（本函数只做**带载**测量）

空载基线来自更早一次空载流程，存成 JSON（:class:`Baseline`）后复用：

    from litearm.payload import Baseline, identify, apply

    base = Baseline.load("empty_payload_baseline.json")
    arm.enable()
    ... 夹上负载 ...
    result = identify(arm, base, speed=0.3)   # movej → 持位采样 → 探针 → 解算
    print(result)
    apply(arm, result)                        # set_payload(mass, com)

老脚本（`wkspace/scripts/负载实验脚本`）产出的 `noload_baseline.csv` 可用
`Baseline.from_csv()` 一次性转成 JSON，之后就不必再碰 CSV。

⚠ 固件里**留着旧的 payload 不会污染结果**：探针取几何量前总是把 payload 归零，而状态帧
里的力矩是真实物理量（不是固件算出来的前馈）。它只影响持位时的下垂量 —— 那点姿态差由模型
搬运补上，构型差越大残差越大。所以"忘了清零"不是错误来源，**没有基线**才是。
"""
from __future__ import annotations

import csv
import json
import math
import os
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, List, Optional, Sequence, Tuple

from litearm.errors import (InvalidCommandError, LiteArmError, MotionTimeoutError,
                            MotorFaultError, TransportError)

if TYPE_CHECKING:                       # 避免与 arm 循环导入
    from litearm.arm import Arm

__all__ = ["Baseline", "HoldSample", "PayloadResult", "apply", "identify",
           "regressor", "gravity", "sample_hold",
           "BASELINE_SCHEMA", "PROBE_MASS", "PROBE_COM", "MASS_TOL", "N_PARAMS"]

#: 本包只支持 7 轴整臂 —— 回归矩阵是 7×4，台架（1J）没有可辨识的负载几何。
N = 7

#: 待求参数个数：`[m, m·cx, m·cy, m·cz]`（质心乘进质量里，方程才是线性的）
N_PARAMS = 4

#: 基线 JSON 的 schema 标记（读到别的值就拒，不做"尽力而为"的猜测）
BASELINE_SCHEMA = "litearm.payload_baseline.v1"

#: 探针用的幻影负载质量 (kg)。取这么小是为了让幻影重力前馈 ≲ 0.14 N·m ——
#: 探针要在**使能持位**中做（见模块头），所以"信号够大"与"扰动够小"在这里是对立的。
#: f32 读回的绝对噪声 ~1e-6 N·m 量级，除以 0.02 后 `Y_g` 第 0 列的相对误差仍 ~1e-4。
PROBE_MASS = 0.02

#: 探针臂长 (m)：点质量沿法兰各轴移这么远。`Y_g` 第 1..3 列 = 两次读数差 /(m·c)，
#: 故 c 太小会把 f32 噪声放大；0.2 m 已是任何手持工具的质心上界。
PROBE_COM = 0.2

#: 静置判据：逐关节 |dq| 上限 (rad/s)。超限的帧不进均值（运动中惯量/科氏项会污染）。
DQ_TOL = 0.05

#: 带载/空载**实测构型**差超过它就给警告 (rad)：差分要靠模型把空载基线搬到带载构型，
#: 搬的距离越大，模型误差进得越多。它不是判据，只是"残差为什么大"的第一嫌疑人。
SHIFT_TOL = 0.05

#: `movej` 报未到位时允许的构型残差 (rad)。固件的到位判据是 `arm.q_tol`（默认 0.03 rad
#: ≈ 1.7°），而**带载且固件里还没有这份负载的重力前馈**时，下垂量常常正好落在那个门槛
#: 之外 ⇒ `movej` 走满了时间预算仍报 `MotionTimeoutError`。辨识用的是**实测构型**，
#: 所以残差在这个量级内继续做是有意义的；再大就说明真的没到位（顶住了/负载太重）。
MOVE_TOL = 0.05

#: 一次持位采样的帧数与帧间隔：25 × 40 ms = 1 s，与旧脚本同口径。
HOLD_FRAMES = 25
HOLD_PERIOD = 0.04

#: 饱和硬拒：|τ| ≥ tau_max × 该比例 的帧丢弃（饱和帧的力矩不是线性的，均值无意义）
SAT_MARGIN = 0.98

#: 质量不确定度门槛 (kg)：超过它就只打印、不建议 `apply()`
MASS_TOL = 0.05

#: `Δτ` 残差 RMS 的验收门槛 (N·m)：与空载基线的逐关节验收同口径 (0.5)
RESIDUAL_TOL = 0.5

#: 质心模长告警线 (m)：远超任何手持工具 ⇒ 多半是构型跨度不足把噪声当成了质心
COM_TOL = 0.3

#: 法方程（**列归一化之后**）的条件数告警线。归一化后它只反映姿态跨度：噪声在解上被
#: 放大 `cond` 倍，所以 100 意味着"千分之一的残差被放大到十分之一"。整臂用旧脚本那套
#: 10 构型实测在个位数；超限基本只有一种原因 —— 构型几乎同姿态。
COND_TOL = 100.0

#: 固件 `payload_mass` 的钳制区间 (params.c) —— 越界先在这里报，别指望固件回 ERR
MASS_RANGE = (0.0, 20.0)
#: 固件 `payload_com` 的逐轴钳制区间
COM_RANGE = (-1.0, 1.0)


def _row(value, label: str) -> Tuple[float, ...]:
    """7 个有限数 —— 基线文件里的畸形行在**读入时**就报出来。"""
    try:
        vals = [float(v) for v in value]
    except (TypeError, ValueError) as e:
        raise InvalidCommandError(f"{label}: 需要数字序列 ({e})") from None
    if len(vals) != N:
        raise InvalidCommandError(f"{label}: 需要 {N} 个数, 收到 {len(vals)} 个")
    if not all(math.isfinite(v) for v in vals):
        raise InvalidCommandError(f"{label}: 不能含 NaN/Inf")
    return tuple(vals)


@dataclass(frozen=True)
class HoldSample:
    """一次停驻采样的结果（逐关节均值）。

    `q` / `tau` 是**通过静置与饱和门的那几帧**的均值，`spread` 是同一批帧的峰峰值
    （τ 量化台阶 58.6/13.7/4.9 mN·m 之外还剩多少抖动，看它就是看数据可不可信），
    `dq` 是逐关节的最大 |dq|，`frames` 是被采纳的帧数。
    """

    q: Tuple[float, ...]
    tau: Tuple[float, ...]
    dq: Tuple[float, ...]
    spread: Tuple[float, ...]
    frames: int

    def __str__(self) -> str:
        q = " ".join(f"{v:+.4f}" for v in self.q)
        t = " ".join(f"{v:+7.3f}" for v in self.tau)
        s = " ".join(f"{v:5.3f}" for v in self.spread)
        return f"q[{q}] tau[{t}] spread[{s}] frames={self.frames}"


@dataclass(frozen=True)
class Baseline:
    """空载基线 —— 一组构型上的空载持位力矩，差分法的那半边。

    三个 7 元组列表一一对应：`poses` 是**下发目标**（复测时按它 movej），`q` 是当时的
    **实测**构型（重力项按实际构型算，带载下垂会让两者差一点点），`tau` 是实测力矩均值。
    只存 JSON（见 :meth:`save`）；老脚本的 CSV 用 :meth:`from_csv` 一次性导入。
    """

    poses: Tuple[Tuple[float, ...], ...]
    q: Tuple[Tuple[float, ...], ...]
    tau: Tuple[Tuple[float, ...], ...]
    created: str = ""
    firmware: str = ""
    source: str = ""

    def __post_init__(self) -> None:
        rows = (("poses", self.poses), ("q", self.q), ("tau", self.tau))
        n = len(self.poses)
        if n == 0:
            raise InvalidCommandError("基线为空: 至少要一个构型")
        for label, value in rows:
            if len(value) != n:
                raise InvalidCommandError(
                    f"基线的 {label} 有 {len(value)} 行, 与 poses 的 {n} 行不一致")
            for i, row in enumerate(value):
                _row(row, f"基线 {label}[{i}]")

    def __len__(self) -> int:
        return len(self.poses)

    def as_dict(self) -> dict:
        return {
            "schema": BASELINE_SCHEMA,
            "created": self.created,
            "firmware": self.firmware,
            "source": self.source,
            "poses": [list(r) for r in self.poses],
            "q": [list(r) for r in self.q],
            "tau": [list(r) for r in self.tau],
        }

    def save(self, path: str) -> str:
        """写 JSON（原子替换：半截基线文件比没有基线更危险）。返回实际路径。"""
        full = os.path.abspath(os.path.expanduser(path))
        parent = os.path.dirname(full)
        if parent:
            os.makedirs(parent, exist_ok=True)
        tmp = full + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(self.as_dict(), f, ensure_ascii=False, indent=2)
            f.write("\n")
        os.replace(tmp, full)
        return full

    @classmethod
    def load(cls, path: str) -> "Baseline":
        """读 JSON 基线。文件不存在抛 `FileNotFoundError`，内容畸形抛 `LiteArmError`。"""
        full = os.path.abspath(os.path.expanduser(path))
        with open(full, encoding="utf-8") as f:
            text = f.read()
        try:
            doc = json.loads(text)
        except ValueError as e:
            raise LiteArmError(f"{full} 不是合法 JSON: {e}") from None
        if not isinstance(doc, dict):
            raise LiteArmError(f"{full} 顶层应为对象, 收到 {type(doc).__name__}")
        got = doc.get("schema")
        if got != BASELINE_SCHEMA:
            raise LiteArmError(
                f"{full} 的 schema 是 {got!r}, 期望 {BASELINE_SCHEMA!r} —— "
                f"别把别的文件当基线读")
        return cls(
            poses=tuple(_row(r, f"{full} poses[{i}]") for i, r in enumerate(doc.get("poses") or [])),
            q=tuple(_row(r, f"{full} q[{i}]") for i, r in enumerate(doc.get("q") or [])),
            tau=tuple(_row(r, f"{full} tau[{i}]") for i, r in enumerate(doc.get("tau") or [])),
            created=str(doc.get("created") or ""),
            firmware=str(doc.get("firmware") or ""),
            source=str(doc.get("source") or ""),
        )

    @classmethod
    def from_csv(cls, path: str) -> "Baseline":
        """导入旧脚本的空载基准 CSV（`pose,q0..q6,tau_meas0..6,pred_*`）。

        只取 `q*` 与 `tau_meas*` 两组列：`pred_*` 是当时那份模型的预测，与差分无关
        （本包的回归矩阵来自固件探针，不复用任何历史模型）。
        CSV 里没有单独记下发目标，故 `poses` 就用实测 `q`（与实际只差下垂那一点点）。
        """
        full = os.path.abspath(os.path.expanduser(path))
        poses: List[Tuple[float, ...]] = []
        taus: List[Tuple[float, ...]] = []
        with open(full, newline="", encoding="utf-8") as f:
            for i, rec in enumerate(csv.DictReader(f)):
                try:
                    q = [float(rec[f"q{k}"]) for k in range(N)]
                    tau = [float(rec[f"tau_meas{k}"]) for k in range(N)]
                except (KeyError, TypeError, ValueError) as e:
                    raise LiteArmError(
                        f"{full} 第 {i + 2} 行缺列/值畸形 ({e}) —— 期望表头 "
                        f"pose,q0..q6,tau_meas0..6,...") from None
                poses.append(tuple(q))
                taus.append(tuple(tau))
        return cls(poses=tuple(poses), q=tuple(poses), tau=tuple(taus),
                   source=full)


@dataclass(frozen=True)
class PayloadResult:
    """辨识结果。`mass` 单位 kg，`com` 是法兰系（`ee_link`）下的质心 (m)。

    `sigma_*` 由残差与法方程逆矩阵的对角给出（不是"散布"）—— 它假设残差独立同分布，
    所以**只用来排序可疑程度**；要判断可信度看 `sigma_mass` 与 `residual_rms` 一起。
    `warnings` 非空不代表失败，只代表某个判据越界（见各条文案）。
    """

    mass: float
    com: Tuple[float, float, float]
    sigma_mass: float
    sigma_com: Tuple[float, float, float]
    rank: int
    cond: float
    residual_rms: float
    n_rows: int
    poses: int
    quality: Tuple[float, ...]          # 逐构型 τ 峰峰值的最大值 (体检)
    pose_residual: Tuple[float, ...] = ()   # 逐构型残差 RMS —— 哪个构型不对就看它
    warnings: Tuple[str, ...] = ()

    @property
    def ok(self) -> bool:
        """可以采纳吗：满秩、质量不确定度在门槛内、解出的值也落在固件可接受区间。"""
        return (self.rank == N_PARAMS
                and self.sigma_mass <= MASS_TOL
                and MASS_RANGE[0] < self.mass <= MASS_RANGE[1])

    def as_dict(self) -> dict:
        return {
            "schema": "litearm.payload_result.v1",
            "mass_kg": self.mass,
            "com_m": list(self.com),
            "sigma_mass_kg": self.sigma_mass,
            "sigma_com_m": list(self.sigma_com),
            "rank": self.rank,
            "cond": self.cond,
            "residual_rms_nm": self.residual_rms,
            "n_rows": self.n_rows,
            "poses": self.poses,
            "pose_residual_nm": list(self.pose_residual),
            "warnings": list(self.warnings),
        }

    def __str__(self) -> str:
        com = " ".join(f"{v:+.4f}" for v in self.com)
        scom = " ".join(f"{v:.4f}" for v in self.sigma_com)
        lines = [
            f"质量 m   = {self.mass:+.4f} kg  (σ {self.sigma_mass:.4f})",
            f"质心 c   = [{com}] m  (σ [{scom}])",
            f"回归     : rank {self.rank}/{N_PARAMS}, cond {self.cond:.2f}, "
            f"{self.poses} 构型 / {self.n_rows} 行",
            f"残差 RMS = {self.residual_rms:.4f} N·m  (门槛 {RESIDUAL_TOL})",
            f"持位抖动 = {max(self.quality) if self.quality else 0.0:.4f} N·m (τ 峰峰值)",
        ]
        if self.warnings:
            lines.append("警告:")
            lines.extend(f"  · {w}" for w in self.warnings)
        return "\n".join(lines)


# ---------------------------------------------------------------- 采样

def sample_hold(arm: "Arm", *, frames: int = HOLD_FRAMES, period: float = HOLD_PERIOD,
                dq_tol: float = DQ_TOL,
                tau_max: Optional[Sequence[float]] = None) -> HoldSample:
    """在**当前**构型上停驻采样，返回逐关节均值（不动机械臂）。

    `get_state(refresh=True)` 每次都要一条**新**状态帧（固件 100 Hz 被动流），所以
    `frames=25, period=0.04` 拿到的是 25 条互不相同的帧，不是同一帧读 25 次。

    三个门：报故障立即抛（数据已经没意义）、|dq| > `dq_tol` 的帧丢弃（运动中的惯量/
    科氏项会污染静置方程）、`|τ| ≥ 0.98·tau_max` 的帧丢弃（饱和帧的力矩不是线性的）。
    `tau_max` 不给就现查 `params.get_joint_param` —— 一轮辨识要查多次，调用方最好
    自己查一次传进来（:func:`identify` 就是这么做的）。
    """
    frames = int(frames)
    if frames < 1:
        raise InvalidCommandError(f"frames 需 ≥1, 收到 {frames}")
    if period < 0:
        raise InvalidCommandError(f"period 需 ≥0, 收到 {period}")
    if tau_max is None:
        tau_max = [arm.params.get_joint_param(i).value.tau_max for i in range(arm.n)]
    lim = [float(v) for v in tau_max]
    if len(lim) != arm.n:
        raise InvalidCommandError(f"tau_max 需 {arm.n} 个, 收到 {len(lim)} 个")

    acc_q: List[Tuple[float, ...]] = []
    acc_tau: List[Tuple[float, ...]] = []
    acc_dq: List[Tuple[float, ...]] = []
    for i in range(frames):
        if i:
            time.sleep(period)
        msg = arm.get_state(refresh=True, timeout=max(0.5, period * 4))
        st = msg.value
        if st is None:
            raise TransportError("停驻采样: 取不到状态帧")
        if st.faulted:
            raise MotorFaultError(f"停驻采样中报故障: {st.fault_detail}")
        q = tuple(float(j.q) for j in st.joints)
        dq = tuple(abs(float(j.dq)) for j in st.joints)
        tau = tuple(float(j.tau) for j in st.joints)
        if len(q) != arm.n or len(tau) != arm.n:
            raise TransportError(f"状态帧关节数 {len(q)} 与 arm.n={arm.n} 不符")
        if any(v > dq_tol for v in dq):
            continue                        # 还在动
        if any(abs(tau[k]) >= lim[k] * SAT_MARGIN for k in range(arm.n)):
            continue                        # 饱和
        acc_q.append(q)
        acc_tau.append(tau)
        acc_dq.append(dq)
    if not acc_q:
        raise LiteArmError(
            f"停驻采样: {frames} 帧全部被拒（|dq|>{dq_tol} rad/s 或力矩接近 tau_max）"
            f" —— 确认臂已到位静止、没有顶着限位")

    n = arm.n
    mean_q = tuple(sum(r[k] for r in acc_q) / len(acc_q) for k in range(n))
    mean_tau = tuple(sum(r[k] for r in acc_tau) / len(acc_tau) for k in range(n))
    max_dq = tuple(max(r[k] for r in acc_dq) for k in range(n))
    spread = tuple(max(r[k] for r in acc_tau) - min(r[k] for r in acc_tau)
                   for k in range(n))
    return HoldSample(mean_q, mean_tau, max_dq, spread, len(acc_q))


# ---------------------------------------------------------------- 固件探针

def _gravity(arm: "Arm", q: Sequence[float]) -> Tuple[float, ...]:
    """`model.get_gravity(q)` 的裸值封装（帧畸形一律报错，不猜）。"""
    msg = arm.model.get_gravity([float(v) for v in q])
    g = msg.value
    if g is None or len(g) != N:
        raise TransportError(f"G(q) 应答畸形: {g!r}")
    return tuple(float(v) for v in g)


def _payload_now(arm: "Arm") -> Tuple[float, Tuple[float, float, float]]:
    """读回固件当前的 payload (mass, com) —— 探针结束要原样写回。"""
    mass = float(arm.get_ff_scalar(4).value)
    com = tuple(float(arm.get_ff_scalar(5, k).value) for k in range(3))
    return mass, (com[0], com[1], com[2])


def _probe(arm: "Arm", qs: Sequence[Sequence[float]]
           ) -> Tuple[List[Tuple[float, ...]], List[List[List[float]]]]:
    """在一批构型上同时取 `G_0(q)` 与 `Y_g(q)`（payload 原值在 `finally` 里恢复）。

    返回 `(g0, ys)`：`g0[i]` 是 payload=0 时的 `G(q_i)`（差分基线的**姿态搬运**用它），
    `ys[i]` 是 7×4 的回归矩阵（列 = `[m, m·cx, m·cy, m·cz]`）。
    """
    mass0, com0 = _payload_now(arm)
    try:
        arm.set_payload(0.0, (0.0, 0.0, 0.0))
        g0 = [_gravity(arm, q) for q in qs]

        arm.set_payload(PROBE_MASS, (0.0, 0.0, 0.0))
        gm = [_gravity(arm, q) for q in qs]

        per_axis: List[List[Tuple[float, ...]]] = []
        for k in range(3):
            com = [0.0, 0.0, 0.0]
            com[k] = PROBE_COM
            arm.set_payload(PROBE_MASS, com)
            per_axis.append([_gravity(arm, q) for q in qs])
    finally:
        arm.set_payload(mass0, com0)

    ys: List[List[List[float]]] = []
    for i in range(len(qs)):
        Y = [[0.0] * N_PARAMS for _ in range(N)]
        for j in range(N):
            Y[j][0] = (gm[i][j] - g0[i][j]) / PROBE_MASS
            for k in range(3):
                Y[j][k + 1] = (per_axis[k][i][j] - gm[i][j]) / (PROBE_MASS * PROBE_COM)
        ys.append(Y)
    return g0, ys


def gravity(arm: "Arm", q: Sequence[float]) -> Tuple[float, ...]:
    """`q` 处**臂本体**（不含负载）的重力项 `G(q)`，7 个值，单位 N·m。

    走 `0x39`，但先把 payload 临时清零 —— 这与 :func:`regressor` 同一个理由：
    要的是几何量，不是"当前固件里配了什么"。原 payload 在返回前写回。
    """
    mass0, com0 = _payload_now(arm)
    try:
        arm.set_payload(0.0, (0.0, 0.0, 0.0))
        return _gravity(arm, q)
    finally:
        arm.set_payload(mass0, com0)


def regressor(arm: "Arm", q: Sequence[float]) -> List[List[float]]:
    """构型 `q` 处的重力回归矩阵 `Y_g(q)`，7×4，满足 `τ_load = Y_g(q)·[m, m·cx, m·cy, m·cz]`。

    用固件探针得到（见模块头）：临时把 payload 设成 `(PROBE_MASS, 0)` 与
    `(PROBE_MASS, 沿轴 PROBE_COM)`，各自 `get_gravity(q)` 后做差。原 payload 写回。

    ⚠ 探针期间控制律的重力前馈会带上幻影负载（≲ 0.14 N·m）。**这里不恢复使能状态、
    也不禁止使能** —— 想要零扰动就自己先 `disable()`（那时电机已松，幻影不进任何执行器）。
    """
    return _probe(arm, [q])[1][0]


# ---------------------------------------------------------------- 解算

def _inverse(A: List[List[float]]) -> Tuple[Optional[List[List[float]]], int, float]:
    """Gauss-Jordan 求逆（列主元）。返回 `(逆矩阵或 None, rank, 最大主元绝对值)`。"""
    n = len(A)
    M = [list(row) + [1.0 if i == k else 0.0 for k in range(n)]
         for i, row in enumerate(A)]
    scale = max((abs(v) for row in A for v in row), default=0.0)
    piv_max = 0.0
    rank = 0
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(M[r][col]))
        if abs(M[piv][col]) <= 1e-12 * max(scale, 1e-30):
            continue                        # 这一列在当前剩余空间中为 0 ⇒ 欠秩
        M[col], M[piv] = M[piv], M[col]
        pv = M[col][col]
        piv_max = max(piv_max, abs(pv))
        M[col] = [v / pv for v in M[col]]
        for r in range(n):
            if r != col and M[r][col] != 0.0:
                f = M[r][col]
                M[r] = [a - f * b for a, b in zip(M[r], M[col])]
        rank += 1
    if rank < n:
        return None, rank, piv_max
    return [row[n:] for row in M], rank, piv_max


def _solve(rows: Sequence[Tuple[List[List[float]], Sequence[float]]]) -> dict:
    """把 `(Y_i, Δτ_i)` 堆成法方程解 4 参数。返回原始数字（不做门槛判断）。

    **先做列归一化**再解法方程：4 个未知量的量纲不同（`kg` 与 `kg·m`），不归一化的话
    `cond` 只反映单位比（`Y[:,0]` ~ N·m/kg 与 `Y[:,1]` ~ N·m/(kg·m) 差了臂长量级），
    既不能跨台架比较，也让"构型可观测性"这条判据失去意义。归一化之后 `cond` 只由
    **姿态跨度**决定 —— 单一姿态恒欠秩、两个异姿态掉到个位数，与旧脚本报的量级一致。
    """
    # 列范数 —— 某一列为 0 说明该参数在此构型集下完全不可观测
    col = [math.sqrt(sum(Y[r][a] ** 2 for Y, _d in rows for r in range(len(Y))))
           for a in range(N_PARAMS)]
    if min(col) <= 0.0:
        return {"theta": None, "sigma": None, "rank": 0, "cond": float("inf"),
                "sigma_res": float("nan")}
    A = [[0.0] * N_PARAMS for _ in range(N_PARAMS)]
    b = [0.0] * N_PARAMS
    for Y, d in rows:
        for r in range(len(d)):
            ya = [Y[r][a] / col[a] for a in range(N_PARAMS)]
            for a in range(N_PARAMS):
                b[a] += ya[a] * d[r]
                for c in range(N_PARAMS):
                    A[a][c] += ya[a] * ya[c]
    inv, rank, _piv = _inverse(A)
    if inv is None:
        return {"theta": None, "sigma": None, "rank": rank, "cond": float("inf"),
                "sigma_res": float("nan")}
    z = [sum(inv[a][k] * b[k] for k in range(N_PARAMS)) for a in range(N_PARAMS)]
    theta = [z[a] / col[a] for a in range(N_PARAMS)]
    n_rows = sum(len(d) for _Y, d in rows)
    rss = 0.0
    for Y, d in rows:
        for r in range(len(d)):
            model = sum(Y[r][k] * theta[k] for k in range(N_PARAMS))
            rss += (d[r] - model) ** 2
    dof = max(n_rows - N_PARAMS, 1)
    sigma_res = math.sqrt(rss / dof)
    sigma = [sigma_res * math.sqrt(max(inv[a][a], 0.0)) / col[a]
             for a in range(N_PARAMS)]
    norm_a = max(sum(abs(v) for v in row) for row in A)
    norm_i = max(sum(abs(v) for v in row) for row in inv)
    return {"theta": theta, "sigma": sigma, "rank": rank, "cond": norm_a * norm_i,
            "sigma_res": sigma_res}


# ---------------------------------------------------------------- 辨识

def _move_to(arm: "Arm", pose: Sequence[float], speed: float,
             move_tol: float) -> None:
    """`movej` 到 `pose`; 未到位判据**带载**时放宽到 `move_tol`（见 :func:`identify`）。

    固件的 `q_tol` 是给空载臂定的。负载上车而固件里还没有这份前馈时，静态下垂全部由
    位置环刚度兜着，几十毫弧度的残差很常见 ⇒ 老实等只会等到超时。这里拿**实测构型**判:
    残差在 `move_tol` 内算到了（采样仍要过静止门，动着的帧进不了均值），否则照原样抛。
    """
    try:
        arm.movej(list(pose), speed)
        return
    except MotionTimeoutError:
        # 必须取**新**帧: 缓存里那帧是超时窗口之前的构型, 拿它判残差只会判出"已到位"
        st = arm.get_state(refresh=True).value
        if st is None or st.faulted:
            raise
        gap = max(abs(st.q[k] - pose[k]) for k in range(len(pose)))
        if gap > move_tol:
            raise MotionTimeoutError(
                f"movej 未到位: 实测构型残差 {gap:.3f} rad 超过 move_tol {move_tol} —— "
                f"带载下垂让固件 q_tol 过不去; 负载太重请减载, 或换更近的基线构型") from None


def identify(arm: "Arm", baseline: Baseline, *, speed: float = 0.3,
             settle: float = 0.4, frames: int = HOLD_FRAMES,
             period: float = HOLD_PERIOD, dq_tol: float = DQ_TOL,
             move_tol: float = MOVE_TOL) -> PayloadResult:
    """带载辨识：按基线构型逐个 `movej` → 停驻采样 → 探针 → 差分解算。

    **前置条件**（都会先检查，条件不满足直接抛，不会静默算出个数）：

    * 臂已使能并且没报故障（辨识靠固件的位置环持位，不是靠机械支撑）；
    * 负载已经夹好 —— 本函数只做带载这一半，空载基线必须是更早一次空载流程的 JSON。

    `settle` 是到位后再等多久才采样（让位置环把下垂量消化掉）。整个流程会移动机械臂，
    构型取自基线本身 —— 换句话说，基线里那 10 个构型就是这次辨识的行程。

    `move_tol` 只为一件事: 带载而固件里还没有这份负载的前馈时, 下垂会让固件的到位判据
    (`arm.q_tol`) 过不去, `movej` 于是一路报 `MotionTimeoutError`。构型残差在 `move_tol`
    以内就继续 (辨识认的是**实测构型**), 超过就照原样抛。
    """
    if not isinstance(baseline, Baseline):
        raise InvalidCommandError(
            f"baseline 需 Baseline (读 JSON 用 Baseline.load), 收到 {type(baseline).__name__}")
    # ⚠ 取状态**排在**轴数/速度校验之前: 链路没开时第一因是断开, 不该被入参形状抢先
    # (与 `cart.py` 三个入口的同一条顺序理由)。
    st = arm.get_state().value
    if st is None:
        raise TransportError("辨识开始前取不到状态帧")
    if arm.n != N:
        raise InvalidCommandError(
            f"负载辨识需要 {N} 轴整臂, 当前 arm.n={arm.n} —— 台架没有可辨识的负载几何")
    if not 0.0 <= float(speed) <= 1.0:
        raise InvalidCommandError(f"speed 需 0..1, 收到 {speed}")
    if st.faulted:
        raise MotorFaultError(f"辨识开始前已报故障: {st.fault_detail}")
    if not st.enabled:
        raise InvalidCommandError(
            "辨识需要臂处于使能持位状态 —— 先 arm.enable() 并确认 get_state().enabled 为真")

    tau_max = [arm.params.get_joint_param(i).value.tau_max for i in range(N)]

    loaded: List[HoldSample] = []
    for pose in baseline.poses:
        _move_to(arm, pose, speed, move_tol)
        if settle > 0:
            time.sleep(settle)
        loaded.append(sample_hold(arm, frames=frames, period=period,
                                  dq_tol=dq_tol, tau_max=tau_max))

    qs = [s.q for s in loaded] + list(baseline.q)
    g0, ys = _probe(arm, qs)
    n_pose = len(loaded)
    g0_loaded = g0[:n_pose]
    g0_empty = g0[n_pose:]
    y_loaded = ys[:n_pose]

    rows: List[Tuple[List[List[float]], List[float]]] = []
    warnings: List[str] = []
    for i, sample in enumerate(loaded):
        transport = [g0_loaded[i][k] - g0_empty[i][k] for k in range(N)]
        dtau = [sample.tau[k] - baseline.tau[i][k] - transport[k] for k in range(N)]
        rows.append((y_loaded[i], dtau))
        dq_shift = max(abs(sample.q[k] - baseline.q[i][k]) for k in range(N))
        if dq_shift > SHIFT_TOL:
            warnings.append(
                f"构型 {i}: 带载/空载实测构型差 {dq_shift:.3f} rad —— "
                f"差分要靠模型搬运补上, 差得越多残差越大")

    sol = _solve(rows)
    if sol["theta"] is None:
        raise LiteArmError(
            f"回归矩阵欠秩 (rank {sol['rank']}/{N_PARAMS}) —— 基线里的构型不足以分开 "
            f"质量与质心; 换一组姿态跨度更大的基线（至少两个异姿态, 且别只动 J2）")

    theta = sol["theta"]
    sigma = sol["sigma"]
    sigma_res = sol["sigma_res"]
    mass = theta[0]
    com = (theta[1], theta[2], theta[3])
    sigma_mass = sigma[0]
    sigma_com = tuple(
        math.sqrt((sigma[k] / mass) ** 2
                  + (theta[k] * sigma_mass / (mass ** 2)) ** 2)
        if mass else float("inf")
        for k in range(1, 4))
    com_val = tuple(v / mass for v in com) if mass else (float("inf"),) * 3

    if mass <= MASS_RANGE[0] or mass > MASS_RANGE[1]:
        warnings.append(
            f"质量 {mass:.3f} kg 落在固件可写范围 {MASS_RANGE} 之外 —— "
            f"set_payload() 会把它静默钳制, 别直接 apply")
    if math.sqrt(sum(v * v for v in com_val)) > COM_TOL:
        warnings.append(
            f"质心模长 {math.sqrt(sum(v * v for v in com_val)):.3f} m 远超任何手持工具 —— "
            f"多半是构型跨度不足, 把噪声解成了质心")
    if sigma_mass > MASS_TOL:
        warnings.append(
            f"质量不确定度 {sigma_mass:.3f} kg 超过门槛 {MASS_TOL} kg —— 残差太大")
    if sol["sigma_res"] > RESIDUAL_TOL:
        warnings.append(
            f"残差 RMS {sigma_res:.3f} N·m 超过验收门槛 {RESIDUAL_TOL} N·m —— "
            f"查空载基线是否同一台臂/同一份模型, 以及夹持是否牢固")
    if sol["cond"] > COND_TOL:
        warnings.append(
            f"法方程 cond {sol['cond']:.1f} 超过 {COND_TOL:.0f} —— 构型几乎同姿态, "
            f"残差会被放大同样的倍数")

    pose_residual = tuple(
        math.sqrt(sum((d[r] - sum(Y[r][k] * theta[k] for k in range(N_PARAMS))) ** 2
                      for r in range(len(d))) / max(len(d), 1))
        for Y, d in rows)

    return PayloadResult(
        mass=mass, com=com_val, sigma_mass=sigma_mass, sigma_com=sigma_com,
        rank=sol["rank"], cond=sol["cond"], residual_rms=sigma_res,
        n_rows=sum(len(d) for _y, d in rows), poses=n_pose,
        quality=tuple(max(s.spread) for s in loaded),
        pose_residual=pose_residual, warnings=tuple(warnings))


def apply(arm: "Arm", result: PayloadResult, *, com: bool = True) -> None:
    """把辨识结果写进固件（`set_payload`，RAM 生效）。

    ⚠ 要持久化再 `save_params()` —— 那是**不可逆的 flash 写**, 且须先 `disable()`;
    先读一遍 `Arm.save_params()` 的警告, 再决定要不要在这台臂上做。

    `com=False` 只写质量（质心按 0 处理）—— 这在质心解不可信、但有外部先验时有用。
    值越界在这里就拒：固件对 `payload_mass` / `payload_com` 是**静默钳制**的
    （`params.c`），写进去的值和你以为的不一样，比报错难查。
    """
    if not isinstance(result, PayloadResult):
        raise InvalidCommandError(
            f"result 需 PayloadResult, 收到 {type(result).__name__}")
    mass = float(result.mass)
    if not MASS_RANGE[0] < mass <= MASS_RANGE[1]:
        raise InvalidCommandError(
            f"质量 {mass:.3f} kg 超出固件可写范围 ({MASS_RANGE[0]}, {MASS_RANGE[1]}]")
    target = tuple(float(v) for v in result.com) if com else (0.0, 0.0, 0.0)
    for k, v in enumerate(target):
        if not COM_RANGE[0] <= v <= COM_RANGE[1]:
            raise InvalidCommandError(
                f"质心第 {k} 轴 {v:.3f} m 超出固件可写范围 {COM_RANGE}")
    arm.set_payload(mass, target)
