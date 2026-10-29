# -*- coding: utf-8 -*-
"""状态流采集 + 激励场景 —— 只读采集与安全性检查都在这里。

数据路径 (与 issue 的「采集」段一一对应):

    固件 100Hz RSP_STATUS ──> _Ack._on_status 探头 ──> Session (每帧一条)
                                                       │
                                                       ├─ status.csv  (可配置 fs 的均匀网格)
                                                       ├─ meta.json   (固件/模型/温度/丢帧账)
                                                       └─ model.json  (ModelSnapshot)

探头挂在 SDK 状态帧的**唯一**漏斗 `_Ack._on_status` 上: 不另开读口、不抢字节流。
采集期间先按到达时刻存**全部**帧, 落盘时再按 `--fs` 重采样到均匀网格
(`filters.linear_resample`): 配置 100Hz 时就是原始流; 配置 50Hz 时是 50Hz 的
均匀重采样, 而不是"碰巧每两帧取一帧" —— 后者在 100Hz 抖动的到达间隔下会变成
30/10ms 交替的假节拍, 差分求 ddq 时直接编出噪声。

激励场景 (issue 要求至少三类):
    hold   静态持位          —— 纯重力项, 校核 G(q)
    sine   低速正弦          —— 连续 q/dq 参考 (move_js ≥10Hz 重发), 激励惯性/摩擦
    poses  多姿态慢速 movej  —— 大范围姿态变化, 激励重力随构型的变化

安全:
    * 所有目标位形都按固件软限位检查, 余量不足 `MIN_MARGIN` 直接拒绝;
    * 默认只读 (打印计划); 只有 `go=True` 才 enable / 运动 / 采集;
    * 采集结束**不 disable** —— 该位形失能会因自重坠回 (见 examples/07 的同一条策略)。
"""
from __future__ import annotations

import bisect
import csv
import datetime as _dt
import json
import math
import os
import platform
import socket
import time
from dataclasses import asdict, dataclass
from typing import Dict, List, Optional, Sequence, Tuple

from . import filters
from .model import ModelSnapshot, fetch_from_arm

#: 目标位形到软限位的最小余量 (rad)。低于它直接拒绝 —— 位置越限锁存不受使能门控,
#: 也不是 reset 能清的那类故障 (与 examples/07 的 MIN_MARGIN 同值同理由)。
MIN_MARGIN = 0.15

#: 正弦幅值/频率的本地护栏 (不是固件限制, 是"这是精度评估不是耐久试验")。
MAX_SINE_AMP = 0.35
MAX_SINE_FREQ = 1.0

#: 连续伺服重发率下限。固件 0.1s 命令看门狗 ⇒ 低于 10Hz 会 fail-soft 丢前馈。
MIN_COMMAND_HZ = 10.0
MAX_COMMAND_HZ = 200.0

#: 静态重力核查的尾部窗口与"算静止"的速度门限。
GRAVITY_TAIL_S = 0.5
STATIC_DQ_MAX = 0.05

#: poses 场景段间参考速度 (rad/s, 再乘 `--speed`)。用 move_js 流式走平滑轨迹,
#: 不依赖 movej 到位判据 —— 真机上 movej 的静态误差会超过 SDK 的 q_tol。
POSE_SPEED_RAD_S = 0.5


# ---------------------------------------------------------------------------
# 会话数据
# ---------------------------------------------------------------------------

@dataclass
class Sample:
    """一帧状态 (时刻为相对会话起点的秒)。"""

    t: float
    seq: int
    mode: int
    mode_name: str
    flags: int
    joint_fault: int
    q: List[float]
    dq: List[float]
    tau: List[float]
    t_mos: List[float]
    t_coil: List[float]
    err: List[int]


class Session:
    """一段采集: 原始采样 + 元数据。

    `samples` 的 `t` 按到达顺序递增, 但间隔不保证均匀 —— `resample(fs)` 才把它变成
    均匀网格。分析层只吃 `resample()` 的结果。
    """

    def __init__(self, samples: List[Sample], meta: Optional[Dict] = None):
        self.samples = list(samples)
        self.meta = dict(meta or {})

    # ---- 基本属性 ----
    @property
    def n(self) -> int:
        return len(self.samples[0].q) if self.samples else 0

    @property
    def span(self) -> float:
        return self.samples[-1].t - self.samples[0].t if len(self.samples) > 1 else 0.0

    @property
    def raw_rate_hz(self) -> float:
        return (len(self.samples) - 1) / self.span if self.span > 0 else 0.0

    def column(self, name: str, j: int) -> List[float]:
        return [getattr(s, name)[j] for s in self.samples]

    def times(self) -> List[float]:
        return [s.t for s in self.samples]

    # ---- 均匀重采样 ----
    def resample(self, fs: float) -> "Session":
        """把不均匀到达序列重采样到 `[t0, t1]` 上的 1/fs 网格。"""
        if not self.samples:
            return Session([], dict(self.meta, fs=fs))
        t = self.times()
        grid = filters.uniform_grid(t[0], t[-1], fs)
        if len(grid) < 2:
            return Session(list(self.samples), dict(self.meta, fs=fs))

        cont: Dict[str, List[List[float]]] = {}
        for name in ("q", "dq", "tau", "t_mos", "t_coil"):
            cont[name] = [
                filters.linear_resample(t, self.column(name, j), grid)
                for j in range(self.n)
            ]
        flat_err = [
            filters.linear_resample(t, [float(s.err[j]) for s in self.samples], grid)
            for j in range(self.n)
        ]
        # 离散字段取"网格时刻之前最近一帧", 不做插值。
        disc = []
        for g in grid:
            k = max(0, bisect.bisect_right(t, g) - 1)
            disc.append(self.samples[k])

        out = []
        for k, g in enumerate(grid):
            src = disc[k]
            out.append(Sample(
                t=round(g, 9), seq=src.seq, mode=src.mode,
                mode_name=src.mode_name, flags=src.flags,
                joint_fault=src.joint_fault,
                q=[cont["q"][j][k] for j in range(self.n)],
                dq=[cont["dq"][j][k] for j in range(self.n)],
                tau=[cont["tau"][j][k] for j in range(self.n)],
                t_mos=[cont["t_mos"][j][k] for j in range(self.n)],
                t_coil=[cont["t_coil"][j][k] for j in range(self.n)],
                err=[int(round(flat_err[j][k])) for j in range(self.n)],
            ))
        return Session(out, dict(self.meta, fs=fs))

    # ---- 落盘 ----
    def write_csv(self, path: str) -> None:
        n = self.n
        cols = (["t", "seq", "mode", "mode_name", "flags", "joint_fault"]
                + [f"q{i}" for i in range(n)]
                + [f"dq{i}" for i in range(n)]
                + [f"tau{i}" for i in range(n)]
                + [f"tmos{i}" for i in range(n)]
                + [f"tcoil{i}" for i in range(n)]
                + [f"err{i}" for i in range(n)])
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(cols)
            for s in self.samples:
                w.writerow([f"{s.t:.9f}", s.seq, s.mode, s.mode_name,
                            s.flags, s.joint_fault]
                           + [f"{v:.12g}" for v in s.q]
                           + [f"{v:.12g}" for v in s.dq]
                           + [f"{v:.12g}" for v in s.tau]
                           + [f"{v:.12g}" for v in s.t_mos]
                           + [f"{v:.12g}" for v in s.t_coil]
                           + list(s.err))

    @classmethod
    def read_csv(cls, path: str, meta: Optional[Dict] = None) -> "Session":
        with open(path, newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        if not rows:
            return cls([], dict(meta or {}))
        n = 0
        while f"q{n}" in rows[0]:
            n += 1

        def _f(v: str) -> float:
            return float(v)

        samples = []
        for r in rows:
            samples.append(Sample(
                t=_f(r["t"]), seq=int(r["seq"]), mode=int(r["mode"]),
                mode_name=r.get("mode_name", ""), flags=int(r["flags"]),
                joint_fault=int(r["joint_fault"]),
                q=[_f(r[f"q{j}"]) for j in range(n)],
                dq=[_f(r[f"dq{j}"]) for j in range(n)],
                tau=[_f(r[f"tau{j}"]) for j in range(n)],
                t_mos=[_f(r[f"tmos{j}"]) for j in range(n)],
                t_coil=[_f(r[f"tcoil{j}"]) for j in range(n)],
                err=[int(float(r[f"err{j}"])) for j in range(n)],
            ))
        return cls(samples, dict(meta or {}))

    def dump(self, outdir: str, extra_meta: Optional[Dict] = None) -> Dict[str, str]:
        os.makedirs(outdir, exist_ok=True)
        csv_path = os.path.join(outdir, "status.csv")
        meta_path = os.path.join(outdir, "meta.json")
        self.write_csv(csv_path)
        meta = dict(self.meta)
        meta.update(extra_meta or {})
        meta.update({
            "sample_count": len(self.samples),
            "t_span_s": round(self.span, 6),
            "fs": meta.get("fs"),
        })
        with open(meta_path, "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)
        return {"status_csv": csv_path, "meta_json": meta_path}

    @classmethod
    def load(cls, outdir: str) -> "Session":
        meta_path = os.path.join(outdir, "meta.json")
        meta = {}
        if os.path.exists(meta_path):
            with open(meta_path, encoding="utf-8") as f:
                meta = json.load(f)
        return cls.read_csv(os.path.join(outdir, "status.csv"), meta)


# ---------------------------------------------------------------------------
# 状态流探头
# ---------------------------------------------------------------------------

class StatusRecorder:
    """挂在 `_Ack._on_status` 上的被动探头。

    只记录, 不判定、不发命令。`arm._a` 是 `_Ack` 实例; 把它替换掉的那个函数在
    退出时恢复 (`stop()`), 因此可以安全嵌套于 SDK 自己的到位判定。
    """

    def __init__(self, arm, clock=time.monotonic):
        self._arm = arm
        self._clock = clock
        self._records: List[Tuple[float, object]] = []
        self._orig = None
        self._ack = None
        self.started_at: Optional[float] = None
        self.finished_at: Optional[float] = None
        self.decode_errors = 0

    def start(self) -> "StatusRecorder":
        from litearm.state import decode_state

        ack = self._arm._a
        orig = ack._on_status
        self._ack = ack
        self._orig = orig
        self.started_at = self._clock()

        def spy(payload: bytes) -> None:
            now = self._clock()
            try:
                st = decode_state(payload)
            except ValueError:
                self.decode_errors += 1
                return
            self._records.append((now, st))
            orig(payload)          # 让 SDK 照常更新 state/status_seq

        ack._on_status = spy
        return self

    def stop(self) -> None:
        if self._ack is not None and self._orig is not None:
            self._ack._on_status = self._orig
        self._ack = None
        self._orig = None
        self.finished_at = self._clock()

    def __enter__(self) -> "StatusRecorder":
        return self.start()

    def __exit__(self, exc_type, exc, tb) -> bool:
        self.stop()
        return False

    # ---- 转成 Session ----
    def to_session(self, meta: Optional[Dict] = None) -> Session:
        if not self._records:
            return Session([], dict(meta or {}))
        t0 = self._records[0][0]
        samples = []
        for ts, st in self._records:
            samples.append(Sample(
                t=ts - t0, seq=st.seq, mode=st.mode,
                mode_name=st.mode_name, flags=st.flags,
                joint_fault=st.joint_fault,
                q=list(st.q), dq=list(st.dq), tau=list(st.tau),
                t_mos=[j.t_mos for j in st.joints],
                t_coil=[j.t_coil for j in st.joints],
                err=[j.err for j in st.joints],
            ))
        m = dict(meta or {})
        m.setdefault("raw_frames", len(samples))
        m.setdefault("raw_rate_hz", round(
            (len(samples) - 1) / max(samples[-1].t, 1e-9), 2))
        m.setdefault("decode_errors", self.decode_errors)
        return Session(samples, m)


def _seq_gaps(samples: Sequence[Sample]) -> Tuple[int, int]:
    """返回 (seq 缺失帧数, 最大连续缺失帧数)。"""
    missing = 0
    max_gap = 0
    for a, b in zip(samples, samples[1:]):
        d = b.seq - a.seq
        if 0 < d < 100000:          # 0/负 = 回绕或重复, 不计
            missing += d - 1
            max_gap = max(max_gap, d - 1)
    return missing, max_gap


# ---------------------------------------------------------------------------
# 场景
# ---------------------------------------------------------------------------

@dataclass
class ScenarioSpec:
    """一个激励场景的参数。`kind` 决定其余字段怎么用。"""

    kind: str                       # hold | sine | poses
    name: str = ""
    duration: float = 3.0           # hold/sine 的总时长 (s)
    q_center: Optional[List[float]] = None
    amp: float = 0.15               # sine 幅值 (rad)
    freq: float = 0.2               # sine 频率 (Hz)
    joints: Optional[List[int]] = None   # sine 激励哪些关节 (0 基); None = 全部
    poses: Optional[List[List[float]]] = None   # poses 场景的目标位形
    dwell: float = 1.2              # poses 每个位形停驻 (s)
    speed: float = 0.2              # poses 的 movej speed 倍率
    command_hz: float = 50.0        # sine 连续伺服重发率
    pre_roll: float = 0.5           # sine 前的静止段 (s)
    post_roll: float = 1.0          # sine 后的静止段 (s)

    def label(self) -> str:
        return self.name or self.kind


def read_joint_limits(arm) -> List[Tuple[float, float]]:
    """从固件读软限位 (0x24)。读不到就直接抛 —— 不会静默用宽松默认值。"""
    out = []
    for i in range(arm.n):
        jp = arm.params.get_joint_param(i).value
        out.append((float(jp.q_min), float(jp.q_max)))
    return out


def _check_pose(pose: Sequence[float], limits: Sequence[Tuple[float, float]],
                label: str, margin: float = MIN_MARGIN) -> None:
    if len(pose) != len(limits):
        raise ValueError(f"{label}: 位形需要 {len(limits)} 个关节, 给的是 {len(pose)}")
    for j, (q, (lo, hi)) in enumerate(zip(pose, limits)):
        if not (math.isfinite(q)):
            raise ValueError(f"{label}: J{j + 1} 角度非有限 ({q})")
        left = q - lo
        right = hi - q
        if left < margin or right < margin:
            raise ValueError(
                f"{label}: J{j + 1} q={q:+.4f} 离软限位 [{lo:+.3f}, {hi:+.3f}] "
                f"余量 {min(left, right):.3f} < {margin} rad，拒绝执行")


def validate_scenario(spec: ScenarioSpec, limits: Sequence[Tuple[float, float]],
                      margin: float = MIN_MARGIN) -> Dict[str, object]:
    """检查场景是否会越限/参数是否合法, 返回给用户看的计划摘要。"""
    n = len(limits)
    if spec.kind == "hold":
        if spec.q_center is None:
            raise ValueError("hold 场景需要 q_center")
        _check_pose(spec.q_center, limits, "hold", margin)
        if spec.duration <= 0:
            raise ValueError("hold duration 必须 > 0")
        return {"kind": "hold", "duration": spec.duration}

    if spec.kind == "sine":
        if spec.q_center is None:
            raise ValueError("sine 场景需要 q_center")
        if not (0.0 < spec.amp <= MAX_SINE_AMP):
            raise ValueError(f"sine amp 必须在 (0, {MAX_SINE_AMP}] rad")
        if not (0.0 < spec.freq <= MAX_SINE_FREQ):
            raise ValueError(f"sine freq 必须在 (0, {MAX_SINE_FREQ}] Hz")
        if spec.duration <= 0:
            raise ValueError("sine duration 必须 > 0")
        if not (MIN_COMMAND_HZ <= spec.command_hz <= MAX_COMMAND_HZ):
            raise ValueError(f"command_hz 必须在 [{MIN_COMMAND_HZ}, {MAX_COMMAND_HZ}]")
        joints = list(spec.joints) if spec.joints is not None else list(range(n))
        for j in joints:
            if not 0 <= j < n:
                raise ValueError(f"sine joints 里的 {j} 越界 (0..{n - 1})")
        # 所有选中关节同相时是最大偏移; 两个极端都查。
        hi = list(spec.q_center)
        lo = list(spec.q_center)
        for j in joints:
            hi[j] += spec.amp
            lo[j] -= spec.amp
        _check_pose(lo, limits, "sine 下极限", margin)
        _check_pose(hi, limits, "sine 上极限", margin)
        return {"kind": "sine", "duration": spec.duration, "amp_rad": spec.amp,
                "freq_hz": spec.freq, "joints": [j + 1 for j in joints],
                "command_hz": spec.command_hz}

    if spec.kind == "poses":
        if not spec.poses:
            raise ValueError("poses 场景需要 poses")
        if spec.dwell < 0:
            raise ValueError("poses dwell 不能为负")
        if not (0.0 < spec.speed <= 1.0):
            raise ValueError("poses speed 必须在 (0, 1]")
        for k, p in enumerate(spec.poses):
            _check_pose(p, limits, f"pose{k}")
        return {"kind": "poses", "poses": len(spec.poses),
                "dwell_s": spec.dwell, "speed": spec.speed}

    raise ValueError(f"未知场景 kind={spec.kind!r} (hold|sine|poses)")


# ---------------------------------------------------------------------------
# 使能与运动
# ---------------------------------------------------------------------------

def wait_enabled(arm, target: bool = True, timeout: float = 2.0) -> bool:
    """轮询状态帧直到 `enabled` 等于 `target`。

    `enable()` 在状态帧反映之前就返回 (真机实测要 17~48ms), 所以只能轮询。
    """
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        st = arm.get_state().value
        if st is not None and st.enabled == target:
            return True
        time.sleep(0.02)
    st = arm.get_state().value
    return st is not None and st.enabled == target


def ensure_enabled(arm, go: bool) -> None:
    st = arm.get_state().value
    if st is None:
        raise RuntimeError("拿不到状态帧, 无法确认使能状态")
    if st.faulted:
        raise RuntimeError(f"臂有故障位, 拒绝运动: {st.fault_detail}")
    if st.enabled:
        return
    if not go:
        raise RuntimeError("臂未使能; 采集需要使能持位 (加 --go 才会 enable)")
    arm.enable()
    if not wait_enabled(arm, True):
        raise RuntimeError("enable() 已返回但状态帧仍未使能")


def _hold_stream(arm, q: Sequence[float], duration: float, hz: float) -> int:
    """以 `hz` 重发 move_js 零速目标, 维持连续伺服看门狗。返回实际发帧数。"""
    if duration <= 0:
        return 0
    period = 1.0 / hz
    dq = [0.0] * len(q)
    n = 0
    t_end = time.monotonic() + duration
    t_next = time.monotonic()
    while True:
        now = time.monotonic()
        if now >= t_end:
            break
        arm.move_js(q, dq)
        n += 1
        t_next += period
        left = t_next - time.monotonic()
        if left > 0:
            time.sleep(left)
    return n


def _run_hold(arm, spec: ScenarioSpec) -> Dict[str, object]:
    """静止持位: 必须持续重发 move_js。

    固件的 0.1s 命令看门狗超时即 fail-soft (降刚度、断重力前馈)。只 sleep 的话
    整个 hold 段录到的是 WD_TRIPPED 下的假持位 —— 真机上实测 flags=0x202。
    """
    hz = float(spec.command_hz)
    sent = _hold_stream(arm, list(spec.q_center), spec.duration, hz)
    return {"duration_s": spec.duration, "command_frames": sent,
            "command_rate_hz": round(sent / max(spec.duration, 1e-9), 2)}


def _run_sine(arm, spec: ScenarioSpec) -> Dict[str, object]:
    q0 = list(spec.q_center)
    joints = list(spec.joints) if spec.joints is not None else list(range(len(q0)))
    hz = float(spec.command_hz)
    period = 1.0 / hz
    w = 2.0 * math.pi * float(spec.freq)
    amp = float(spec.amp)

    _hold_stream(arm, q0, spec.pre_roll, hz)
    t_start = time.monotonic()
    t_end = t_start + spec.duration
    sent = 0
    t_next = time.monotonic()
    while True:
        now = time.monotonic()
        if now >= t_end:
            break
        ph = w * (now - t_start)
        q_ref = list(q0)
        dq_ref = [0.0] * len(q0)
        s, c = math.sin(ph), math.cos(ph)
        for j in joints:
            q_ref[j] = q0[j] + amp * s
            dq_ref[j] = amp * w * c
        arm.move_js(q_ref, dq_ref)
        sent += 1
        t_next += period
        left = t_next - time.monotonic()
        if left > 0:
            time.sleep(left)
    _hold_stream(arm, q0, spec.post_roll, hz)
    return {"sine_s": spec.duration, "command_frames": sent,
            "command_rate_hz": round(sent / max(spec.duration, 1e-9), 2)}


def _run_poses(arm, spec: ScenarioSpec) -> Dict[str, object]:
    """多个位形的慢速往返: 用 move_js 流式走 smoothstep 参考, 每个位形停驻 `dwell`。

    为什么不用 `movej`: 固件 movej 的静止保持靠位置误差, 台架重力前馈偏差会留下
    0.04~0.05 rad 稳态误差, 超过 SDK 到位判据 `q_tol=0.03` ⇒ 必然 MotionTimeoutError,
    整段数据全丢。这里自己生成 jerk/加速度连续的参考, 顺带给 ddq 一条干净来源。
    """
    if not spec.poses:
        return {"poses": 0, "moves": []}
    hz = float(spec.command_hz)
    period = 1.0 / hz
    q = list(spec.q_center)
    v_ref = max(POSE_SPEED_RAD_S * float(spec.speed or 1.0), 1e-3)
    moves = []

    _hold_stream(arm, q, spec.pre_roll, hz)
    for k, target in enumerate(spec.poses):
        target = [float(v) for v in target]
        dq_seg = [target[j] - q[j] for j in range(len(q))]
        dur = max(0.6, max(abs(v) for v in dq_seg) / v_ref)
        t0 = time.monotonic()
        t_end = t0 + dur
        sent = 0
        t_next = t0
        while True:
            now = time.monotonic()
            if now >= t_end:
                break
            u = (now - t0) / dur
            s = filters.smoothstep(u)
            sd = filters.smoothstep_deriv(u) / dur
            arm.move_js([q[j] + dq_seg[j] * s for j in range(len(q))],
                        [dq_seg[j] * sd for j in range(len(q))])
            sent += 1
            t_next += period
            left = t_next - time.monotonic()
            if left > 0:
                time.sleep(left)
        arm.move_js(target, [0.0] * len(q))      # 精确末点 + 零速
        q = target
        _hold_stream(arm, q, spec.dwell, hz)
        moves.append({"pose": k, "move_s": round(time.monotonic() - t0 - spec.dwell, 3),
                      "command_frames": sent})
    _hold_stream(arm, q, spec.post_roll, hz)
    return {"poses": len(spec.poses), "moves": moves}


# ---------------------------------------------------------------------------
# 静态重力核查
# ---------------------------------------------------------------------------

def static_gravity_check(arm, samples: Sequence[Sample],
                         tail_s: float = GRAVITY_TAIL_S) -> Dict[str, object]:
    """用末尾静止段的实测 τ 对 `model.get_gravity(q)` 做一次静态核查。

    这是 issue 点名的 «用 model.get_gravity 做对照»。只在尾窗内所有关节
    |dq| < STATIC_DQ_MAX 时成立; 否则如实标 skipped —— 不拿运动段冒充。
    """
    if not samples:
        return {"skipped": "无采样"}
    t_end = samples[-1].t
    tail = [s for s in samples if s.t >= t_end - tail_s]
    if not tail:
        return {"skipped": "尾窗为空"}
    n = len(tail[0].q)
    dq_max = max(abs(v) for s in tail for v in s.dq)
    if dq_max > STATIC_DQ_MAX:
        return {"skipped": f"尾窗不静止 (max|dq|={dq_max:.4f} > {STATIC_DQ_MAX})",
                "max_abs_dq": round(dq_max, 6)}
    q = [filters.mean([s.q[j] for s in tail]) for j in range(n)]
    tau_meas = [filters.mean([s.tau[j] for s in tail]) for j in range(n)]
    try:
        tau_model = list(arm.model.get_gravity(q).value)
    except Exception as e:                      # noqa: BLE001 - 老固件可能没有 0x39
        return {"skipped": f"get_gravity 不可用: {type(e).__name__}: {e}"}
    resid = [tau_meas[j] - tau_model[j] for j in range(n)]
    rms = math.sqrt(sum(v * v for v in resid) / len(resid)) if resid else float("nan")
    return {
        "q": [round(v, 6) for v in q],
        "tau_meas": [round(v, 6) for v in tau_meas],
        "tau_model": [round(v, 6) for v in tau_model],
        "residual": [round(v, 6) for v in resid],
        "rms": round(rms, 6),
        "max_abs_dq": round(dq_max, 6),
    }


# ---------------------------------------------------------------------------
# 元数据
# ---------------------------------------------------------------------------

def _state_meta(st) -> Optional[Dict[str, object]]:
    if st is None:
        return None
    return {"t": 0.0, "seq": st.seq, "mode": st.mode, "mode_name": st.mode_name,
            "flags": st.flags, "enabled": st.enabled, "joint_fault": st.joint_fault,
            "q": [round(v, 6) for v in st.q],
            "dq": [round(v, 6) for v in st.dq],
            "tau": [round(v, 6) for v in st.tau],
            "flag_names": list(st.flag_names)}


def _build_meta(arm, spec: ScenarioSpec, snapshot: ModelSnapshot, st0, st_end,
                limits, plan, fs: float, scenario_result: Dict[str, object],
                gravity_check: Dict[str, object],
                ambient_c: Optional[float]) -> Dict[str, object]:
    now = _dt.datetime.now().astimezone()
    temps = []
    if st_end is not None:
        temps = [j.t_coil for j in st_end.joints]
    meta: Dict[str, object] = {
        "tool": "tau_eval",
        "schema": "tau_eval.session.v1",
        "created_at": now.isoformat(timespec="seconds"),
        "hostname": socket.gethostname(),
        "platform": platform.platform(),
        "python": platform.python_version(),
        "scenario": spec.label(),
        "scenario_spec": asdict(spec),
        "scenario_plan": plan,
        "scenario_result": scenario_result,
        "fs": fs,
        "firmware": getattr(arm, "firmware", "") or snapshot.firmware,
        "joint_count": int(getattr(arm, "n", snapshot.joint_count)),
        "model_geometry_sha16": snapshot.geometry_sha16,
        "model_file": "model.json",
        "model_read_warnings": list(snapshot.read_warnings),
        "payload": {"mass": snapshot.payload_mass,
                    "com": list(snapshot.payload_com)},
        "gravity": list(snapshot.gravity),
        "gravity_scale": list(snapshot.gravity_scale),
        "inertia_scale": list(snapshot.inertia_scale),
        "friction_model": snapshot.friction_model,
        "joint_limits": [[lo, hi] for lo, hi in limits],
        "initial_state": _state_meta(st0),
        "final_state": _state_meta(st_end),
        "motor_temperature_c": [round(v, 3) for v in temps],
        "motor_temperature_max_c": round(max(temps), 3) if temps else None,
        "ambient_temperature_c": ambient_c,
        "gravity_check": gravity_check,
        "notes": [
            "τ 来自固件电流环估计 (状态帧 joint.tau)，口径为关节侧。",
            "ddq 由实测 dq 中心差分得到，再做零相位低通；见 report 的 filter 段。",
            "仿真/台架可选 --gravity 覆盖, 但固件 get_gravity 的核查仍按固件参数。",
        ],
    }
    return meta


# ---------------------------------------------------------------------------
# 主入口
# ---------------------------------------------------------------------------

def run_scenario(arm, spec: ScenarioSpec, outdir: str, fs: float = 100.0,
                 go: bool = False, ambient_c: Optional[float] = None,
                 snapshot: Optional[ModelSnapshot] = None,
                 parquet: bool = False) -> Session:
    """执行一个场景并落盘 `status.csv` / `meta.json` / `model.json`。

    `go=False` 时只做只读校验并抛 `RuntimeError` 前先返回计划? —— 不, 本函数只负责
    真正执行; 规划/打印在 `cli.py`。`go` 只决定是否允许 enable。
    """
    if not (1.0 <= fs <= 100.0):
        raise ValueError("fs 必须在 [1, 100] Hz (固件状态流上限约 100Hz)")

    limits = read_joint_limits(arm)
    plan = validate_scenario(spec, limits)
    ensure_enabled(arm, go)

    if snapshot is None:
        snapshot = fetch_from_arm(arm)
    st0 = arm.get_state().value

    with StatusRecorder(arm) as rec:
        if spec.kind == "hold":
            result = _run_hold(arm, spec)
        elif spec.kind == "sine":
            result = _run_sine(arm, spec)
        elif spec.kind == "poses":
            result = _run_poses(arm, spec)
        else:                                    # validate_scenario 已挡住
            raise ValueError(spec.kind)
        st_end = arm.get_state().value

    raw = rec.to_session()
    missing, gap_max = _seq_gaps(raw.samples)
    raw.meta.update({"seq_missing": missing, "seq_gap_max": gap_max})

    gcheck = static_gravity_check(arm, raw.samples)
    meta = _build_meta(arm, spec, snapshot, st0, st_end, limits, plan,
                       fs, result, gcheck, ambient_c)
    meta.update({"seq_missing": missing, "seq_gap_max": gap_max,
                 "raw_frames": len(raw.samples),
                 "raw_rate_hz": round(raw.raw_rate_hz, 2),
                 "t_raw_span_s": round(raw.span, 6)})

    session = raw.resample(fs)
    session.meta = meta

    paths = session.dump(outdir)
    model_path = os.path.join(outdir, "model.json")
    with open(model_path, "w", encoding="utf-8") as f:
        json.dump(snapshot.to_dict(), f, ensure_ascii=False, indent=2)
    paths["model_json"] = model_path

    if parquet:
        pq = _write_parquet(session, os.path.join(outdir, "status.parquet"))
        if pq:
            paths["status_parquet"] = pq

    print(f"[tau_eval] {spec.label()}: {len(session.samples)} 帧 / "
          f"{session.span:.2f}s @ {fs:g}Hz (原始 {len(raw.samples)} 帧, "
          f"seq 缺 {missing}) -> {outdir}")
    if gcheck.get("skipped"):
        print(f"[tau_eval] 静态重力核查: 跳过 ({gcheck['skipped']})")
    else:
        print(f"[tau_eval] 静态重力核查: RMS {gcheck['rms']:.4f} N·m, "
              f"max|res| {max(abs(v) for v in gcheck['residual']):.4f} N·m")
    return session


def _write_parquet(session: Session, path: str) -> Optional[str]:
    """有 pyarrow 时额外写一份 Parquet; 没有就跳过 (核心路径坚持纯标准库)。"""
    try:
        import pyarrow as pa  # type: ignore
        import pyarrow.parquet as pq  # type: ignore
    except Exception:                       # noqa: BLE001 - 可选依赖
        print("[tau_eval] 未安装 pyarrow, 跳过 Parquet (CSV 已写)")
        return None
    n = session.n
    data = {"t": [s.t for s in session.samples],
            "seq": [s.seq for s in session.samples],
            "mode": [s.mode for s in session.samples],
            "flags": [s.flags for s in session.samples],
            "joint_fault": [s.joint_fault for s in session.samples]}
    for name in ("q", "dq", "tau", "t_mos", "t_coil"):
        for j in range(n):
            data[f"{name}{j}"] = [getattr(s, name)[j] for s in session.samples]
    for j in range(n):
        data[f"err{j}"] = [s.err[j] for s in session.samples]
    pq.write_table(pa.table(data), path)
    return path
