# -*- coding: utf-8 -*-
"""把分析结果写成 Markdown 报告 —— 指标表、分层、延迟、归因与图索引。"""
from __future__ import annotations

import os
from typing import Dict, List, Optional

from .analyze import AnalysisResult


def _f(v, nd: int = 4) -> str:
    if v is None:
        return "—"
    if isinstance(v, float):
        if v != v:
            return "—"
        return f"{v:.{nd}f}"
    return str(v)


def _table(headers: List[str], rows: List[List[str]]) -> str:
    out = ["| " + " | ".join(headers) + " |",
           "|" + "|".join(["---"] * len(headers)) + "|"]
    for r in rows:
        out.append("| " + " | ".join(r) + " |")
    return "\n".join(out)


def _figs(paths: Dict[str, str], outdir: str, keys: List[str]) -> List[str]:
    lines = []
    for k in keys:
        p = paths.get(k)
        if not p:
            continue
        rel = os.path.relpath(p, outdir)
        title = os.path.splitext(os.path.basename(p))[0]
        lines.append(f"![{title}]({rel})")
        lines.append("")
    return lines


def write_report(result: AnalysisResult, figure_paths: Optional[Dict[str, str]] = None,
                 outdir: str = ".", filename: str = "report.md",
                 command: Optional[str] = None) -> str:
    s = result.summary
    figs = figure_paths or {}
    n = result.n
    raw = s["metrics"]["raw"]
    scaled = s["metrics"]["scaled"]
    agg = raw["aggregate"]
    meta = result.snapshot
    opts = s["options"]

    lines: List[str] = []
    lines.append("# 电流环力矩估计精度评估报告")
    lines.append("")
    lines.append(f"- 场景: **{s.get('scenario', '—')}**")
    lines.append(f"- 采样: {s['n_samples']} 帧 @ {s['fs_hz']:.2f} Hz, "
                 f"分析窗 {s['t_span_s']:.2f} s")
    lines.append(f"- 固件: `{s['model'].get('firmware') or '—'}`; "
                 f"几何指纹: `{s['model'].get('geometry_sha16')}`")
    lines.append(f"- 重力向量: {s['model'].get('gravity')} "
                 f"(台架若为 +Y 请用 `--gravity 0,9.81,0`)")
    lines.append(f"- payload: {s['model'].get('payload')}")
    lines.append("")

    lines.append("## 结论摘要")
    lines.append("")
    lines.append(f"- 逐轴平均 RMSE (raw): **{_f(agg.get('mean_rmse'))} N·m**; "
                 f"最大 {_f(agg.get('max_rmse'))} N·m "
                 f"({agg.get('worst_joint')})")
    lines.append(f"- 逐轴平均 |bias|: {_f(agg.get('mean_abs_bias'))} N·m; "
                 f"平均相对 RMSE: {_f(agg.get('mean_rel_rmse'), 3)}")
    lines.append(f"- 固件前馈口径 (scaled) 平均 RMSE: "
                 f"**{_f(scaled['aggregate'].get('mean_rmse'))} N·m**")
    lines.append("")
    diags = s.get("diagnosis") or []
    if diags:
        lines.append("主要误差来源 (按证据排序, 不是最终判决):")
        lines.append("")
        for d in diags:
            lines.append(f"- **{d['source']}** — {d['evidence']}")
            lines.append(f"  - 建议: {d['suggestion']}")
        lines.append("")

    lines.append("## 方法与口径")
    lines.append("")
    lines.append(f"- 模型: 固件在线读回的刚体参数 + jm + payload 复合 "
                 f"(`model.json`, 几何来自 `dyn_model_data.h`)。")
    lines.append("- 逆动力学: 世界系牛顿-欧拉 RNEA (固件 `rnea_core_m` 的逐行移植), "
                 "`τ = M·ddq + C·dq + G + jm·ddq + friction`。")
    lines.append(f"- 滤波: 零相位 Butterworth (filtfilt), "
                 f"q/dq {opts['cutoff_dq_hz']} Hz, ddq {opts['cutoff_ddq_hz']} Hz, "
                 f"τ {opts['cutoff_tau_hz']} Hz; ddq 由 dq 中心差分后再低通。")
    lines.append(f"- 边沿裁剪: {_f(opts.get('trim_s'), 3)} s "
                 f"(默认 `max(2/cutoff_ddq, 0.1)`, 上限 span/5), "
                 f"避开零相位滤波两端瞬态。")
    lines.append("- 残差定义: `τ_meas − τ_model`, 关节侧, N·m; "
                 "`raw` 为物理量, `scaled` 按固件 gravity_scale/inertia_scale 口径。")
    if s.get("resampled"):
        lines.append("- ⚠ 原始到达间隔不均匀, 分析前已重采样到均匀网格。")
    lines.append("")

    lines.append("## 逐轴指标 (raw)")
    lines.append("")
    rows = []
    for j in range(n):
        k = f"J{j + 1}"
        m = raw["per_joint"][k]
        ms = scaled["per_joint"][k]
        rows.append([k, _f(m["rmse"]), _f(m["mae"]), _f(m["bias"]),
                     _f(m["peak_abs"]), _f(m["rel_rmse"], 3),
                     _f(m["r2"], 3), _f(m["corr"], 3), _f(ms["rmse"])])
    lines.append(_table(
        ["关节", "RMSE", "MAE", "bias", "peak", "rel_RMSE", "R²", "corr",
         "RMSE (scaled)"], rows))
    lines.append("")

    lines.append("## 按速度分层 (raw RMSE, N·m)")
    lines.append("")
    sp = s["strata"]["speed"]
    cats = []
    for v in sp.values():
        for c in v:
            if c not in cats:
                cats.append(c)
    cats.sort()
    rows = []
    for j in range(n):
        k = f"J{j + 1}"
        row = [k]
        for c in cats:
            row.append(_f(sp.get(k, {}).get(c, {}).get("rmse")))
        rows.append(row)
    if cats:
        lines.append(_table(["关节"] + cats, rows))
        lines.append("")

    lines.append("## 延迟扫描")
    lines.append("")
    lines.append("lag > 0 = 实测相对模型滞后; improvement = 时移后可再降的 RMSE 比例。")
    lines.append("")
    rows = []
    for j in range(n):
        d = s["delay"].get(f"J{j + 1}")
        if not d:
            continue
        rows.append([f"J{j + 1}", _f(d["lag_ms"], 2), _f(d["rmse_at_zero"]),
                     _f(d["rmse_at_best"]), f"{d['improvement'] * 100:.1f}%"])
    if rows:
        lines.append(_table(["关节", "最优 lag (ms)", "零时移 RMSE",
                             "最优 RMSE", "可改善"], rows))
        lines.append("")

    lines.append("## 残差与其他项的相关")
    lines.append("")
    rows = []
    for j in range(n):
        r = s["residual"].get(f"J{j + 1}", {})
        c = r.get("correlation", {})
        rows.append([f"J{j + 1}", _f(r.get("static_bias")),
                     _f(c.get("gravity"), 2), _f(c.get("inertia"), 2),
                     _f(c.get("friction"), 2), _f(c.get("sign_dq"), 2),
                     _f(c.get("ddq"), 2)])
    lines.append(_table(["关节", "静止 bias", "r(G)", "r(I)", "r(F)",
                         "r(sign dq)", "r(ddq)"], rows))
    lines.append("")

    lines.append("## 各理论项 RMS (N·m)")
    lines.append("")
    cr = s.get("components_rms", {})
    rows = []
    for j in range(n):
        rows.append([f"J{j + 1}"] + [_f(cr.get(name, [None] * n)[j])
                                     for name in ("gravity", "coriolis", "inertia",
                                                  "jm", "friction")])
    lines.append(_table(["关节", "G", "C", "I", "jm·ddq", "friction"], rows))
    lines.append("")

    lines.append("## 图")
    lines.append("")
    keys = []
    for j in range(n):
        keys += [f"j{j + 1}_tau", f"j{j + 1}_res"]
    keys += ["rmse", "bias", "parity", "delay", "hist"]
    fig_lines = _figs(figs, outdir, keys)
    if fig_lines:
        lines += fig_lines
    else:
        lines.append("(未生成图)")
        lines.append("")

    lines.append("## 复现")
    lines.append("")
    if command:
        lines.append("```")
        lines.append(command)
        lines.append("```")
    else:
        lines.append("```")
        lines.append("python3 tools/tau_eval/run.py analyze <会话目录>")
        lines.append("```")
    lines.append("")
    lines.append("## 限制")
    lines.append("")
    lines.append("- τ 是**电流环估计值**, 不是关节力矩传感器; 电流环标定误差会直接进入结论。")
    lines.append("- 固件模型是辨识回归量: link mass 多为 1、惯量含负值、M 不正定; "
                 "只用于前向计算, **不可求 M⁻¹**。")
    lines.append("- `get_gravity` 的静态核查只在末段静止时有效; 运动段不冒充静态。")
    lines.append("- 环境温度读不到, 报告里的温度是电机绕组/MOS 温度; "
                 "ambient 只能用 `--ambient-c` 手工记录。")
    lines.append("- 结论的适用范围是本组构型与速度; 外推到限位附近/高速段需另采数据。")
    lines.append("")

    path = os.path.join(outdir, filename)
    os.makedirs(outdir, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    return path
