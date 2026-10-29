# -*- coding: utf-8 -*-
"""逆动力学误差分析 —— 实测 τ (电流环) vs 模型 τ = M·ddq + C·dq + G + friction + jm·ddq。

处理链 (每一步都可复现, 参数写进 metrics.json):

    1. 均匀重采样      会话本就按 fs 存, 这里再确认网格;
    2. 零相位低通      q/dq 用 `--cutoff-dq` (默认 15Hz), τ 用 `--cutoff-tau` (默认 10Hz);
    3. 中心差分 + 低通 ddq = filtfilt(diff(dq)) (`--cutoff-ddq`, 默认 5Hz);
    4. 逆动力学        逐采样调用 rnea.theory_torques (固件同款世界系 RNEA);
    5. 边沿裁剪        前后各裁 max(2/cutoff_ddq, 0.1)s, 避开 filtfilt 的反射瞬态;
    6. 指标/分层/延迟扫描。

为什么用零相位滤波: 普通因果低通会把 ddq 整体推迟几毫秒, 误差会被误归因成
'惯量模型错'。filtfilt 在通带内净相位 0, 只在两端留 ~2/fc 的瞬态, 所以第 5 步裁剪。

指标口径:
    residual = τ_meas − τ_model (N·m)
    bias     = mean(residual)
    RMSE/MAE/peak 都基于 residual
    rel_RMSE = RMSE / RMS(τ_meas)
    R²       = 1 − SS_res/SS_tot (对 τ_meas 的方差解释度)

`raw` 用物理量 (不加 gravity_scale/inertia_scale); `scaled` 用固件前馈口径,
两者都给出 —— 前者回答"模型对不对", 后者回答"固件前馈打得准不准"。
"""
from __future__ import annotations

import copy
import csv
import json
import math
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from . import filters, rnea
from .collect import Session
from .model import ModelSnapshot, N_JOINT


# ---------------------------------------------------------------------------
# 配置
# ---------------------------------------------------------------------------

@dataclass
class AnalyzeOptions:
    cutoff_dq: float = 15.0        # Hz
    cutoff_ddq: float = 5.0        # Hz
    cutoff_tau: float = 10.0       # Hz
    trim_s: Optional[float] = None  # None = max(2/cutoff_ddq, 0.1), 上限 span/5
    gravity_override: Optional[Tuple[float, float, float]] = None
    max_delay_ms: float = 100.0
    static_dq: float = 0.05        # |dq| 低于此值视为静止
    speed_edges: Tuple[float, ...] = (0.05, 0.3, 1.0)

    def to_dict(self) -> Dict[str, object]:
        return {"cutoff_dq_hz": self.cutoff_dq, "cutoff_ddq_hz": self.cutoff_ddq,
                "cutoff_tau_hz": self.cutoff_tau, "trim_s": self.trim_s,
                "gravity_override": list(self.gravity_override)
                if self.gravity_override is not None else None,
                "max_delay_ms": self.max_delay_ms, "static_dq": self.static_dq,
                "speed_edges": list(self.speed_edges)}


# ---------------------------------------------------------------------------
# 结果容器
# ---------------------------------------------------------------------------

@dataclass
class AnalysisResult:
    fs: float
    t: List[float]
    q: List[List[float]]                # [joint][sample]
    dq: List[List[float]]
    ddq: List[List[float]]
    tau_meas: List[List[float]]
    tau_model: List[List[float]]
    tau_scaled: List[List[float]]
    components: Dict[str, List[List[float]]]
    options: AnalyzeOptions
    snapshot: ModelSnapshot
    summary: Dict[str, object] = field(default_factory=dict)

    @property
    def n(self) -> int:
        return len(self.q)

    @property
    def n_samples(self) -> int:
        return len(self.t)


# ---------------------------------------------------------------------------
# 基础工具
# ---------------------------------------------------------------------------

def _median(vals: Sequence[float]) -> float:
    s = sorted(vals)
    n = len(s)
    if n == 0:
        return float("nan")
    m = n // 2
    return s[m] if n % 2 else 0.5 * (s[m - 1] + s[m])


def infer_fs(session: Session) -> float:
    """优先用 meta 的 fs, 否则用到达间隔中位数。"""
    fs = session.meta.get("fs")
    if isinstance(fs, (int, float)) and fs > 0:
        return float(fs)
    ts = session.times()
    if len(ts) < 2:
        return 100.0
    dt = _median([b - a for a, b in zip(ts, ts[1:]) if b > a])
    return 1.0 / dt if dt > 0 else 100.0


def _ensure_uniform(session: Session) -> Tuple[Session, float, bool]:
    fs = infer_fs(session)
    ts = session.times()
    if len(ts) < 3:
        return session, fs, False
    dt = [b - a for a, b in zip(ts, ts[1:])]
    nominal = 1.0 / fs
    jitter = max(abs(d - nominal) for d in dt) / nominal
    if jitter <= 0.02:
        return session, fs, False
    return session.resample(fs), fs, True


def _filtered(x: Sequence[float], fs: float, fc: float) -> List[float]:
    return filters.lowpass_zero_phase(x, fs, fc) if len(x) >= 4 else list(x)


def _metrics(meas: Sequence[float], pred: Sequence[float]) -> Dict[str, object]:
    n = len(meas)
    if n == 0:
        return {"n": 0}
    res = [meas[i] - pred[i] for i in range(n)]
    rmse = math.sqrt(sum(v * v for v in res) / n)
    mae = sum(abs(v) for v in res) / n
    peak = max(abs(v) for v in res)
    bias = filters.mean(res)
    std = filters.pstdev(res)
    meas_rms = math.sqrt(sum(v * v for v in meas) / n)
    rel = rmse / meas_rms if meas_rms > 1e-12 else float("nan")
    return {
        "n": n,
        "rmse": round(rmse, 6),
        "mae": round(mae, 6),
        "bias": round(bias, 6),
        "std": round(std, 6),
        "peak_abs": round(peak, 6),
        "rel_rmse": round(rel, 6) if rel == rel else None,
        "r2": round(filters.r2_score(meas, pred), 6),
        "corr": round(filters.pearson(meas, pred), 6),
        "meas_rms": round(meas_rms, 6),
        "model_mean": round(filters.mean(pred), 6),
        "meas_mean": round(filters.mean(meas), 6),
    }


def _agg_metrics(per_joint: Dict[str, Dict[str, object]]) -> Dict[str, object]:
    vals = [m for m in per_joint.values() if m.get("n")]
    if not vals:
        return {"n_joints": 0}
    rmse = [m["rmse"] for m in vals]
    return {
        "n_joints": len(vals),
        "mean_rmse": round(filters.mean(rmse), 6),
        "max_rmse": round(max(rmse), 6),
        "worst_joint": max(per_joint, key=lambda k: per_joint[k].get("rmse", -1)),
        "mean_abs_bias": round(filters.mean([abs(m["bias"]) for m in vals]), 6),
        "max_abs_bias": round(max(abs(m["bias"]) for m in vals), 6),
        "mean_rel_rmse": round(filters.mean(
            [m["rel_rmse"] for m in vals if m.get("rel_rmse") is not None]), 6),
    }


# ---------------------------------------------------------------------------
# 分层
# ---------------------------------------------------------------------------

def _speed_bins(edges: Sequence[float]) -> List[Tuple[float, float, str]]:
    """[0, e0) = static, [e0,e1) = slow, ... 最后一段到 ∞。"""
    out = [(0.0, edges[0], "static")]
    labels = ["slow", "mid", "fast", "very_fast"]
    for k in range(len(edges) - 1):
        out.append((edges[k], edges[k + 1], labels[min(k, len(labels) - 1)]))
    out.append((edges[-1], float("inf"), labels[min(len(edges) - 1, len(labels) - 1)]))
    return out


def speed_strata(result: AnalysisResult) -> Dict[str, object]:
    bins = _speed_bins(result.options.speed_edges)
    out: Dict[str, object] = {}
    for j in range(result.n):
        dq = result.dq[j]
        meas = result.tau_meas[j]
        model = result.tau_model[j]
        rows = {}
        for lo, hi, label in bins:
            idx = [i for i, v in enumerate(dq) if lo <= abs(v) < hi]
            if len(idx) < 10:
                continue
            rows[f"{label}_[{lo:g},{hi:g})"] = _metrics(
                [meas[i] for i in idx], [model[i] for i in idx])
        if rows:
            out[f"J{j + 1}"] = rows
    return out


def pose_strata(result: AnalysisResult) -> Dict[str, object]:
    out: Dict[str, object] = {}
    for j in range(result.n):
        q = result.q[j]
        meas = result.tau_meas[j]
        model = result.tau_model[j]
        qs = sorted(q)
        if len(qs) < 30:
            continue
        q1, q2 = qs[len(qs) // 3], qs[2 * len(qs) // 3]
        groups = [("q_low", -float("inf"), q1), ("q_mid", q1, q2),
                  ("q_high", q2, float("inf"))]
        rows = {}
        for label, lo, hi in groups:
            idx = [i for i, v in enumerate(q) if lo <= v < hi]
            if len(idx) < 10:
                continue
            m = _metrics([meas[i] for i in idx], [model[i] for i in idx])
            m["q_range"] = [round(lo if lo > -1e9 else min(q), 4),
                            round(hi if hi < 1e9 else max(q), 4)]
            rows[label] = m
        if rows:
            out[f"J{j + 1}"] = rows
    return out


# ---------------------------------------------------------------------------
# 延迟扫描
# ---------------------------------------------------------------------------

def delay_scan(result: AnalysisResult) -> Dict[str, object]:
    """在 ±max_delay_ms 内找使 RMSE 最小的时移。

    lag > 0 表示「把实测序列往后挪 lag 个采样才最贴合模型」⇒ 实测相对模型**滞后**。
    这能抓住因果滤波/电流环积分导致的群延迟, 也能量化"零相位滤波没有白做"。
    """
    fs = result.fs
    max_lag = max(1, int(round(result.options.max_delay_ms / 1000.0 * fs)))
    out: Dict[str, object] = {}
    for j in range(result.n):
        meas = result.tau_meas[j]
        model = result.tau_model[j]
        n = len(meas)
        if n < 2 * max_lag + 10:
            continue
        base = _rmse(meas, model)
        best = (0, base)
        curve = []
        for lag in range(-max_lag, max_lag + 1):
            if lag >= 0:
                m, y = model[:n - lag], meas[lag:]
            else:
                m, y = model[-lag:], meas[:n + lag]
            if len(m) < 10:
                continue
            r = _rmse(y, m)
            curve.append([lag, round(r, 6)])
            if r < best[1]:
                best = (lag, r)
        imp = (base - best[1]) / base if base > 1e-12 else 0.0
        out[f"J{j + 1}"] = {
            "lag_samples": best[0],
            "lag_ms": round(best[0] / fs * 1000.0, 3),
            "rmse_at_zero": round(base, 6),
            "rmse_at_best": round(best[1], 6),
            "improvement": round(imp, 6),
            "curve": curve,
        }
    return out


def _rmse(a: Sequence[float], b: Sequence[float]) -> float:
    if not a:
        return float("nan")
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)) / len(a))


# ---------------------------------------------------------------------------
# 残差归因
# ---------------------------------------------------------------------------

def residual_analysis(result: AnalysisResult,
                      static_bias: Sequence[float]) -> Dict[str, object]:
    """把 residual 与各理论项做相关, 给出可核查的归因证据 (不是判决)。"""
    out: Dict[str, object] = {}
    comp_names = ("gravity", "coriolis", "inertia", "jm", "friction")
    for j in range(result.n):
        res = [result.tau_meas[j][i] - result.tau_model[j][i]
               for i in range(result.n_samples)]
        dq = result.dq[j]
        corr = {}
        for name in comp_names:
            c = result.components[name][j]
            corr[name] = round(filters.pearson(res, c), 4)
        corr["dq"] = round(filters.pearson(res, dq), 4)
        corr["abs_dq"] = round(filters.pearson(res, [abs(v) for v in dq]), 4)
        corr["sign_dq"] = round(filters.pearson(res, [1.0 if v > 0 else -1.0 for v in dq]), 4)
        corr["ddq"] = round(filters.pearson(res, result.ddq[j]), 4)
        out[f"J{j + 1}"] = {
            "static_bias": round(static_bias[j], 6),
            "correlation": corr,
        }
    return out


def static_bias(result: AnalysisResult) -> List[float]:
    """静止采样的平均残差 —— 电流环标定偏置 + 重力模型误差的合并证据。"""
    out = []
    for j in range(result.n):
        idx = [i for i, v in enumerate(result.dq[j])
               if abs(v) < result.options.static_dq]
        if not idx:
            out.append(float("nan"))
            continue
        out.append(filters.mean(
            [result.tau_meas[j][i] - result.tau_model[j][i] for i in idx]))
    return out


# ---------------------------------------------------------------------------
# 诊断建议
# ---------------------------------------------------------------------------

def diagnose(result: AnalysisResult, summary: Dict[str, object]) -> List[Dict[str, str]]:
    diags: List[Dict[str, str]] = []
    raw = summary["metrics"]["raw"]
    agg = raw["aggregate"]
    if agg.get("n_joints", 0) == 0:
        return diags

    rel = agg.get("mean_rel_rmse")
    if rel is not None and rel < 0.02:
        return [{
            "source": "模型与实测高度一致",
            "evidence": f"平均相对 RMSE {rel * 100:.2f}% (<2%), 残差已接近滤波/数值噪声",
            "suggestion": "若为合成/回放数据, 说明采集-逆动力学链路正确; 若为真机数据, "
                          "精度已优于 2%, 结论限定在本组构型与速度范围内即可。",
        }]

    # 1) 滤波/时序延迟
    worst_delay = max(
        (v for v in summary["delay"].values() if "improvement" in v),
        key=lambda v: v["improvement"], default=None)
    if worst_delay is not None and worst_delay["improvement"] > 0.05 \
            and abs(worst_delay["lag_ms"]) >= 5.0:
        diags.append({
            "source": "时序/滤波延迟",
            "evidence": f"最优时移 {worst_delay['lag_ms']:+.1f} ms 时 RMSE 再降 "
                        f"{worst_delay['improvement'] * 100:.1f}% (J 最优轴)",
            "suggestion": "先看该轴 cutoff_ddq 是否过低 (ddq 相位滞后) 或电流环报数"
                          "有固定延迟; 可把 --cutoff-ddq 提到 8~10Hz 复测, 再把残差"
                          "与延迟解耦。",
        })

    # 2) 摩擦 (residual 与 sign(dq) / friction 相关)
    fr_corr = [(name, v["correlation"].get("friction", 0.0))
               for name, v in summary["residual"].items()]
    if fr_corr:
        name, c = max(fr_corr, key=lambda kv: abs(kv[1]))
        if abs(c) >= 0.3:
            diags.append({
                "source": "摩擦模型/低速换向",
                "evidence": f"最大残差-摩擦相关 |r|={abs(c):.2f} ({name})",
                "suggestion": "低速段与换向处最容易暴露 Stribeck 参数误差; 用 hold/"
                             "极低速 sine 段单独看 residual vs sign(dq), 必要时重新辨识 "
                             "fv/fc0/fc1 (get_ff_vec 9/10/11)。",
            })

    # 3) 重力 / 电流环零偏 (静止 bias)
    biases = [(k, v["static_bias"]) for k, v in summary["residual"].items()
              if isinstance(v.get("static_bias"), (int, float))
              and v["static_bias"] == v["static_bias"]]
    if biases:
        name, b = max(biases, key=lambda kv: abs(kv[1]))
        lim = max(0.1, 0.15 * max(m["rmse"] for m in raw["per_joint"].values()))
        if abs(b) > lim:
            diags.append({
                "source": "重力模型/电流环零偏",
                "evidence": f"最大静止残差 {b:+.3f} N·m ({name}); 静止时 ddq=dq=0, "
                            f"残差只能来自 G(q)、payload 或电流环标定",
                "suggestion": "先确认台架真实重力向量 (--gravity) 与固件 get_ff_scalar(6) "
                             "一致; 再对静止段做逐轴零偏校核。若偏置随姿态变, 指向重力/payload;"
                             "若各姿态几乎恒定, 指向电流环/减速器零偏。",
            })

    # 4) 惯量 (运动段残差-惯量相关 + 延迟已排除)
    in_corr = [(k, v["correlation"].get("inertia", 0.0))
               for k, v in summary["residual"].items()]
    if in_corr:
        name, c = max(in_corr, key=lambda kv: abs(kv[1]))
        if abs(c) >= 0.3:
            diags.append({
                "source": "惯量/加速度项",
                "evidence": f"最大残差-惯量项相关 |r|={abs(c):.2f} ({name})",
                "suggestion": "用 sine 段的 ddq 幅值与残差符号核对 jm/惯量缩放 "
                             "(--gravity 之外再看 inertia_scale); 先排除延迟 (上一条) "
                             "再怀疑惯量, 否则相位差会被误读成惯量误差。",
            })

    if not diags:
        diags.append({
            "source": "未见单一主导项",
            "evidence": f"平均 RMSE {agg['mean_rmse']:.4f} N·m, 平均 |bias| "
                        f"{agg['mean_abs_bias']:.4f} N·m, 无单项相关超过阈值",
            "suggestion": "看逐轴 metrics 与 residual 分布; 若精度已满足, 把结论限定在"
                          "本组构型/速度范围内。",
        })
    return diags


# ---------------------------------------------------------------------------
# 主分析
# ---------------------------------------------------------------------------

def analyze_session(session: Session, snapshot: ModelSnapshot,
                    options: Optional[AnalyzeOptions] = None) -> AnalysisResult:
    opts = options or AnalyzeOptions()
    if snapshot.joint_count != N_JOINT:
        raise ValueError(f"模型关节数 {snapshot.joint_count} != {N_JOINT}")

    session, fs, resampled = _ensure_uniform(session)
    n = session.n
    if n != N_JOINT:
        raise ValueError(f"会话有 {n} 个关节, 期望 {N_JOINT}")
    if len(session.samples) < 10:
        raise ValueError("样本太少, 无法分析 (至少 10 帧)")

    snap = copy.deepcopy(snapshot)
    if opts.gravity_override is not None:
        snap.gravity = tuple(float(v) for v in opts.gravity_override)

    t = session.times()
    t = [v - t[0] for v in t]
    s0 = session.samples[0]
    dt = 1.0 / fs
    q_raw = [[s.q[j] for s in session.samples] for j in range(n)]
    dq_raw = [[s.dq[j] for s in session.samples] for j in range(n)]
    tau_raw = [[s.tau[j] for s in session.samples] for j in range(n)]

    cutoff_dq = min(opts.cutoff_dq, 0.45 * fs)
    cutoff_ddq = min(opts.cutoff_ddq, 0.45 * fs)
    cutoff_tau = min(opts.cutoff_tau, 0.45 * fs)
    q = [_filtered(q_raw[j], fs, cutoff_dq) for j in range(n)]
    dq = [_filtered(dq_raw[j], fs, cutoff_dq) for j in range(n)]
    ddq = [_filtered(filters.central_diff(dq[j], dt), fs, cutoff_ddq)
           for j in range(n)]
    tau_meas = [_filtered(tau_raw[j], fs, cutoff_tau) for j in range(n)]

    n_s = len(t)
    tau_model = [[0.0] * n_s for _ in range(n)]
    tau_scaled = [[0.0] * n_s for _ in range(n)]
    comp: Dict[str, List[List[float]]] = {
        name: [[0.0] * n_s for _ in range(n)]
        for name in ("gravity", "coriolis", "inertia", "jm", "friction")}
    for i in range(n_s):
        d = rnea.theory_torques(snap, [q[j][i] for j in range(n)],
                                [dq[j][i] for j in range(n)],
                                [ddq[j][i] for j in range(n)])
        for name in comp:
            for j in range(n):
                comp[name][j][i] = d[name][j]
        for j in range(n):
            tau_model[j][i] = d["total"][j]
            tau_scaled[j][i] = d["total_scaled"][j]

    # 边沿裁剪: filtfilt 的反射瞬态约 2/fc 秒
    trim = opts.trim_s
    if trim is None:
        trim = max(2.0 / cutoff_ddq, 0.1)
    trim = min(trim, 0.2 * t[-1]) if t[-1] > 0 else 0.0
    i0 = int(math.ceil(trim * fs))
    i1 = n_s - i0
    if i1 - i0 < 10:
        i0, i1 = 0, n_s

    def cut(seq):
        return seq[i0:i1]

    res = AnalysisResult(
        fs=fs,
        t=cut(t),
        q=[cut(v) for v in q],
        dq=[cut(v) for v in dq],
        ddq=[cut(v) for v in ddq],
        tau_meas=[cut(v) for v in tau_meas],
        tau_model=[cut(v) for v in tau_model],
        tau_scaled=[cut(v) for v in tau_scaled],
        components={name: [cut(v) for v in vals] for name, vals in comp.items()},
        options=opts,
        snapshot=snap,
    )

    per_raw = {}
    per_scaled = {}
    for j in range(n):
        per_raw[f"J{j + 1}"] = _metrics(res.tau_meas[j], res.tau_model[j])
        per_scaled[f"J{j + 1}"] = _metrics(res.tau_meas[j], res.tau_scaled[j])

    sbias = static_bias(res)
    opts_dict = opts.to_dict()
    opts_dict["trim_s"] = round(trim, 6)
    summary: Dict[str, object] = {
        "schema": "tau_eval.analysis.v1",
        "scenario": session.meta.get("scenario"),
        "fs_hz": fs,
        "n_samples": res.n_samples,
        "t_span_s": round(res.t[-1] - res.t[0], 6) if res.t else 0.0,
        "resampled": resampled,
        "options": opts_dict,
        "model": {
            "firmware": snap.firmware,
            "geometry_sha16": snap.geometry_sha16,
            "gravity": list(snap.gravity),
            "payload": {"mass": snap.payload_mass, "com": list(snap.payload_com)},
            "friction_model": snap.friction_model,
            "read_warnings": snap.read_warnings,
        },
        "metrics": {
            "raw": {"per_joint": per_raw, "aggregate": _agg_metrics(per_raw)},
            "scaled": {"per_joint": per_scaled, "aggregate": _agg_metrics(per_scaled)},
        },
        "strata": {"speed": speed_strata(res), "pose": pose_strata(res)},
        "delay": delay_scan(res),
        "static_bias": [None if v != v else round(v, 6) for v in sbias],
        "residual": residual_analysis(res, sbias),
        "components_rms": {
            name: [round(math.sqrt(sum(v * v for v in vals[j]) / max(len(vals[j]), 1)), 6)
                   for j in range(n)]
            for name, vals in res.components.items()},
        "notes": [
            "residual = τ_meas − τ_model, 关节侧口径, 单位 N·m。",
            "raw 为物理量; scaled 按固件 gravity_scale/inertia_scale 口径。",
            "speed 分层基于滤波后的 |dq|; pose 分层基于 q 的 1/3、2/3 分位。",
            "delay>0 = 实测相对模型滞后; improvement 是时移后可再降的 RMSE 比例。",
        ],
    }
    summary["diagnosis"] = diagnose(res, summary)
    res.summary = summary
    return res


# ---------------------------------------------------------------------------
# 读写
# ---------------------------------------------------------------------------

def write_timeseries(result: AnalysisResult, path: str) -> None:
    n = result.n
    cols = ["t"]
    for j in range(n):
        cols += [f"q{j}", f"dq{j}", f"ddq{j}", f"tau_meas{j}",
                 f"tau_model{j}", f"tau_scaled{j}",
                 f"G{j}", f"C{j}", f"I{j}", f"F{j}"]
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for i in range(result.n_samples):
            row = [f"{result.t[i]:.9f}"]
            for j in range(n):
                row += [
                    f"{result.q[j][i]:.9g}", f"{result.dq[j][i]:.9g}",
                    f"{result.ddq[j][i]:.9g}", f"{result.tau_meas[j][i]:.9g}",
                    f"{result.tau_model[j][i]:.9g}", f"{result.tau_scaled[j][i]:.9g}",
                    f"{result.components['gravity'][j][i]:.9g}",
                    f"{result.components['coriolis'][j][i]:.9g}",
                    f"{result.components['inertia'][j][i]:.9g}",
                    f"{result.components['friction'][j][i]:.9g}",
                ]
            w.writerow(row)


def clean_json(obj):
    """递归把 NaN/±inf 换成 None —— `json.dump` 默认会写出非标准 NaN。"""
    if isinstance(obj, dict):
        return {k: clean_json(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [clean_json(v) for v in obj]
    if isinstance(obj, float) and not math.isfinite(obj):
        return None
    return obj


def save_analysis(result: AnalysisResult, outdir: str) -> Dict[str, str]:
    os.makedirs(outdir, exist_ok=True)
    ts_path = os.path.join(outdir, "timeseries.csv")
    metrics_path = os.path.join(outdir, "metrics.json")
    write_timeseries(result, ts_path)
    with open(metrics_path, "w", encoding="utf-8") as f:
        json.dump(clean_json(result.summary), f, ensure_ascii=False, indent=2)
    return {"timeseries_csv": ts_path, "metrics_json": metrics_path}


def _load_snapshot(indir: str) -> ModelSnapshot:
    path = os.path.join(indir, "model.json")
    with open(path, encoding="utf-8") as f:
        return ModelSnapshot.from_dict(json.load(f))


def analyze_dir(indir: str, outdir: Optional[str] = None,
                options: Optional[AnalyzeOptions] = None) -> AnalysisResult:
    """从一个采集目录 (`status.csv` + `model.json`) 分析, 结果写回同目录。"""
    session = Session.load(indir)
    snapshot = _load_snapshot(indir)
    result = analyze_session(session, snapshot, options)
    save_analysis(result, outdir or indir)
    return result
