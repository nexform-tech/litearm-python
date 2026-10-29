# -*- coding: utf-8 -*-
"""纯 SVG 绘图 —— 不依赖 matplotlib/numpy。

为什么手写 SVG: 本仓库的 CI 只装 `.[dev]` (pytest), 而真机上 matplotlib 与
numpy 2.2.6 有 ABI 冲突。评测工具的核心路径坚持纯标准库, 图必须能在任何
有 Python 的机器上生成。输出是普通 `.svg`, 浏览器/VS Code/Markdown 都能直接看。

提供的最小图元:
    line      多序列折线 (时间序列 / 延迟扫描)
    bars      分组柱状 (逐轴 RMSE / bias / 分层)
    scatter   散点 + y=x 参考线 (实测 vs 模型)
    hist      直方图 (残差分布)
"""
from __future__ import annotations

import html
import math
import os
from typing import Dict, List, Optional, Sequence, Tuple

from .analyze import AnalysisResult

PALETTE = ["#1f77b4", "#d62728", "#2ca02c", "#ff7f0e",
           "#9467bd", "#8c564b", "#17becf"]


def _esc(s: object) -> str:
    return html.escape(str(s), quote=True)


def _finite(v: float) -> bool:
    return isinstance(v, (int, float)) and math.isfinite(v)


def _nice_ticks(lo: float, hi: float, count: int = 5) -> List[float]:
    if not (_finite(lo) and _finite(hi)) or hi <= lo:
        return [lo]
    span = hi - lo
    raw = span / max(count, 1)
    mag = 10 ** math.floor(math.log10(raw))
    for m in (1, 2, 2.5, 5, 10):
        step = m * mag
        if span / step <= count + 1:
            break
    start = math.floor(lo / step) * step
    ticks = []
    v = start
    while v <= hi + step * 0.5:
        if v >= lo - step * 0.5:
            ticks.append(v)
        v += step
    return ticks


def _fmt(v: float) -> str:
    if not _finite(v):
        return ""
    if v == 0:
        return "0"
    a = abs(v)
    if a >= 1000 or a < 0.001:
        return f"{v:.2e}"
    if a >= 10:
        return f"{v:.1f}"
    if a >= 1:
        return f"{v:.2f}"
    return f"{v:.3f}"


def _downsample(xs: Sequence[float], ys: Sequence[float], limit: int = 1500):
    n = len(xs)
    if n <= limit:
        return xs, ys
    step = max(1, n // limit)
    idx = list(range(0, n, step))
    if idx[-1] != n - 1:
        idx.append(n - 1)
    return [xs[i] for i in idx], [ys[i] for i in idx]


def _canvas(body: List[str], w: int, h: int, title: str) -> str:
    return "\n".join([
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" '
        f'viewBox="0 0 {w} {h}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        f'<text x="{w / 2:.1f}" y="24" text-anchor="middle" '
        f'font-family="DejaVu Sans, sans-serif" font-size="16" fill="#111">{_esc(title)}</text>',
        *body,
        "</svg>",
    ]) + "\n"


def _write(path: str, text: str) -> str:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
    return path


def _axes(body: List[str], x0: float, y0: float, x1: float, y1: float,
          xlo: float, xhi: float, ylo: float, yhi: float,
          xlabel: str, ylabel: str) -> None:
    body.append(f'<rect x="{x0:.1f}" y="{y0:.1f}" width="{x1 - x0:.1f}" '
                f'height="{y1 - y0:.1f}" fill="none" stroke="#333" stroke-width="1"/>')
    for v in _nice_ticks(xlo, xhi):
        px = x0 + (v - xlo) / (xhi - xlo) * (x1 - x0)
        body.append(f'<line x1="{px:.1f}" y1="{y0:.1f}" x2="{px:.1f}" y2="{y1:.1f}" '
                    f'stroke="#e6e6e6" stroke-width="1"/>')
        body.append(f'<text x="{px:.1f}" y="{y1 + 16:.1f}" text-anchor="middle" '
                    f'font-family="sans-serif" font-size="11" fill="#444">{_fmt(v)}</text>')
    for v in _nice_ticks(ylo, yhi):
        py = y1 - (v - ylo) / (yhi - ylo) * (y1 - y0)
        body.append(f'<line x1="{x0:.1f}" y1="{py:.1f}" x2="{x1:.1f}" y2="{py:.1f}" '
                    f'stroke="#e6e6e6" stroke-width="1"/>')
        body.append(f'<text x="{x0 - 8:.1f}" y="{py + 4:.1f}" text-anchor="end" '
                    f'font-family="sans-serif" font-size="11" fill="#444">{_fmt(v)}</text>')
    body.append(f'<text x="{(x0 + x1) / 2:.1f}" y="{y1 + 38:.1f}" text-anchor="middle" '
                f'font-family="sans-serif" font-size="12" fill="#222">{_esc(xlabel)}</text>')
    body.append(f'<text x="16" y="{(y0 + y1) / 2:.1f}" text-anchor="middle" '
                f'transform="rotate(-90 16 {(y0 + y1) / 2:.1f})" '
                f'font-family="sans-serif" font-size="12" fill="#222">{_esc(ylabel)}</text>')


def _legend(body: List[str], labels: Sequence[str], x0: float, y0: float,
            colors: Optional[Sequence[str]] = None,
            dashed: Optional[Sequence[bool]] = None) -> None:
    x = x0
    for k, lab in enumerate(labels):
        c = (colors or PALETTE)[k % len(colors or PALETTE)]
        dash = " stroke-dasharray=\"6 4\"" if dashed and dashed[k] else ""
        body.append(f'<line x1="{x:.1f}" y1="{y0:.1f}" x2="{x + 22:.1f}" y2="{y0:.1f}" '
                    f'stroke="{c}" stroke-width="2"{dash}/>')
        body.append(f'<text x="{x + 27:.1f}" y="{y0 + 4:.1f}" font-family="sans-serif" '
                    f'font-size="11" fill="#222">{_esc(lab)}</text>')
        x += 27 + 9 * len(str(lab)) + 18


# ---------------------------------------------------------------------------
# 图元
# ---------------------------------------------------------------------------

def svg_line(path: str, series: Sequence[Tuple[str, Sequence[float], Sequence[float]]],
             title: str, xlabel: str = "t (s)", ylabel: str = "",
             width: int = 960, height: int = 520,
             y_zero: bool = False,
             colors: Optional[Sequence[str]] = None,
             dashed: Optional[Sequence[bool]] = None) -> str:
    """series = [(label, xs, ys), ...]; 自动取所有序列的并集范围。"""
    clean = []
    for lab, xs, ys in series:
        pts = [(x, y) for x, y in zip(xs, ys) if _finite(x) and _finite(y)]
        if pts:
            clean.append((lab, pts))
    if not clean:
        return _write(path, _canvas([], width, height, title))
    xlo = min(x for _, p in clean for x, _ in p)
    xhi = max(x for _, p in clean for x, _ in p)
    if xhi - xlo < 1e-12:
        xhi = xlo + 1.0
    ylo = min(min(y for _, y in p) for _, p in clean)
    yhi = max(max(y for _, y in p) for _, p in clean)
    if y_zero:
        ylo, yhi = min(ylo, 0.0), max(yhi, 0.0)
    if yhi - ylo < 1e-9:
        ylo, yhi = ylo - 1.0, yhi + 1.0
    pad = 0.05 * (yhi - ylo)
    ylo, yhi = ylo - pad, yhi + pad

    x0, y0, x1, y1 = 78, 52, width - 24, height - 58
    body: List[str] = []
    _axes(body, x0, y0, x1, y1, xlo, xhi, ylo, yhi, xlabel, ylabel)
    if ylo <= 0.0 <= yhi:
        py = y1 - (0.0 - ylo) / (yhi - ylo) * (y1 - y0)
        body.append(f'<line x1="{x0:.1f}" y1="{py:.1f}" x2="{x1:.1f}" y2="{py:.1f}" '
                    f'stroke="#999" stroke-width="1"/>')
    for k, (lab, pts) in enumerate(clean):
        c = (colors or PALETTE)[k % len(colors or PALETTE)]
        d = " stroke-dasharray=\"6 4\"" if dashed and k < len(dashed) and dashed[k] else ""
        pts = _downsample([p[0] for p in pts], [p[1] for p in pts])
        dpath = " ".join(
            f'{"M" if i == 0 else "L"}{x0 + (x - xlo) / (xhi - xlo) * (x1 - x0):.1f},'
            f'{y1 - (y - ylo) / (yhi - ylo) * (y1 - y0):.1f}'
            for i, (x, y) in enumerate(zip(*pts)))
        body.append(f'<path d="{dpath}" fill="none" stroke="{c}" stroke-width="1.8"{d}/>')
    _legend(body, [lab for lab, _ in clean], x0, y0 - 12, colors, dashed)
    return _write(path, _canvas(body, width, height, title))


def svg_bars(path: str, categories: Sequence[str],
             series: Sequence[Tuple[str, Sequence[float]]], title: str,
             ylabel: str = "", width: int = 900, height: int = 480,
             colors: Optional[Sequence[str]] = None) -> str:
    vals = [v for _, vs in series for v in vs if _finite(v)]
    if not vals or not categories:
        return _write(path, _canvas([], width, height, title))
    ylo, yhi = min(vals + [0.0]), max(vals + [0.0])
    if yhi - ylo < 1e-9:
        yhi += 1.0
    pad = 0.08 * (yhi - ylo)
    ylo, yhi = ylo - pad, yhi + pad
    x0, y0, x1, y1 = 78, 52, width - 24, height - 58
    body: List[str] = []
    _axes(body, x0, y0, x1, y1, 0.0, float(len(categories)), ylo, yhi, "", ylabel)
    group_w = (x1 - x0) / len(categories)
    bar_w = group_w * 0.75 / max(len(series), 1)
    for ci, cat in enumerate(categories):
        gx = x0 + ci * group_w + group_w * 0.5
        body.append(f'<text x="{gx:.1f}" y="{y1 + 16:.1f}" text-anchor="middle" '
                    f'font-family="sans-serif" font-size="11" fill="#444">{_esc(cat)}</text>')
        for si, (lab, vs) in enumerate(series):
            v = vs[ci] if ci < len(vs) else float("nan")
            if not _finite(v):
                continue
            bx = gx - (len(series) * bar_w) / 2 + si * bar_w
            py = y1 - (v - ylo) / (yhi - ylo) * (y1 - y0)
            py0 = y1 - (0.0 - ylo) / (yhi - ylo) * (y1 - y0)
            top, hgt = min(py, py0), abs(py0 - py)
            c = (colors or PALETTE)[si % len(colors or PALETTE)]
            body.append(f'<rect x="{bx:.1f}" y="{top:.1f}" width="{bar_w * 0.9:.1f}" '
                        f'height="{max(hgt, 1.0):.1f}" fill="{c}" opacity="0.85"/>')
    _legend(body, [lab for lab, _ in series], x0, y0 - 12, colors)
    return _write(path, _canvas(body, width, height, title))


def svg_scatter(path: str,
                groups: Sequence[Tuple[str, Sequence[float], Sequence[float]]],
                title: str, xlabel: str = "", ylabel: str = "",
                width: int = 720, height: int = 680) -> str:
    pts_all = [(x, y) for _, xs, ys in groups for x, y in zip(xs, ys)
               if _finite(x) and _finite(y)]
    if not pts_all:
        return _write(path, _canvas([], width, height, title))
    lo = min(min(x, y) for x, y in pts_all)
    hi = max(max(x, y) for x, y in pts_all)
    if hi - lo < 1e-9:
        hi += 1.0
    pad = 0.04 * (hi - lo)
    lo, hi = lo - pad, hi + pad
    x0, y0, x1, y1 = 78, 52, width - 24, height - 58
    body: List[str] = []
    _axes(body, x0, y0, x1, y1, lo, hi, lo, hi, xlabel or "model (N·m)",
          ylabel or "measured (N·m)")
    body.append(f'<line x1="{x0:.1f}" y1="{y1:.1f}" x2="{x1:.1f}" y2="{y0:.1f}" '
                f'stroke="#111" stroke-width="1" stroke-dasharray="5 4"/>')
    for k, (lab, xs, ys) in enumerate(groups):
        c = PALETTE[k % len(PALETTE)]
        # 散点限流, 避免 5k+ 点时 SVG 体积膨胀 (视觉上看不出差别)
        step = max(1, len(xs) // 800)
        for x, y in list(zip(xs, ys))[::step]:
            if not (_finite(x) and _finite(y)):
                continue
            px = x0 + (x - lo) / (hi - lo) * (x1 - x0)
            py = y1 - (y - lo) / (hi - lo) * (y1 - y0)
            body.append(f'<circle cx="{px:.1f}" cy="{py:.1f}" r="1.6" '
                        f'fill="{c}" opacity="0.55"/>')
    _legend(body, [g[0] for g in groups], x0, y0 - 12)
    return _write(path, _canvas(body, width, height, title))


def svg_hist(path: str, values: Sequence[float], title: str,
             xlabel: str = "", bins: int = 30,
             width: int = 900, height: int = 460) -> str:
    vals = sorted(v for v in values if _finite(v))
    if len(vals) < 2:
        return _write(path, _canvas([], width, height, title))
    lo, hi = vals[0], vals[-1]
    if hi - lo < 1e-12:
        lo, hi = lo - 0.5, hi + 0.5
    step = (hi - lo) / bins
    counts = [0] * bins
    for v in vals:
        k = min(bins - 1, int((v - lo) / step))
        counts[k] += 1
    yhi = max(counts) * 1.08 if counts else 1
    x0, y0, x1, y1 = 78, 52, width - 24, height - 58
    body: List[str] = []
    _axes(body, x0, y0, x1, y1, lo, hi, 0.0, yhi, xlabel or "residual (N·m)", "count")
    bw = (x1 - x0) / bins
    for k, c in enumerate(counts):
        if c == 0:
            continue
        hgt = c / yhi * (y1 - y0)
        body.append(f'<rect x="{x0 + k * bw:.2f}" y="{y1 - hgt:.1f}" '
                    f'width="{max(bw - 1, 1):.2f}" height="{hgt:.1f}" '
                    f'fill="#1f77b4" opacity="0.8"/>')
    if lo <= 0.0 <= hi:
        px = x0 + (0.0 - lo) / (hi - lo) * (x1 - x0)
        body.append(f'<line x1="{px:.1f}" y1="{y0:.1f}" x2="{px:.1f}" y2="{y1:.1f}" '
                    f'stroke="#d62728" stroke-width="1"/>')
    return _write(path, _canvas(body, width, height, title))


# ---------------------------------------------------------------------------
# 分析结果 -> 一整套图
# ---------------------------------------------------------------------------

def generate_all(result: AnalysisResult, outdir: str,
                 prefix: str = "") -> Dict[str, str]:
    os.makedirs(outdir, exist_ok=True)
    n = result.n
    t = result.t
    paths: Dict[str, str] = {}

    def name(s: str) -> str:
        return os.path.join(outdir, f"{prefix}{s}.svg")

    for j in range(n):
        paths[f"j{j + 1}_tau"] = svg_line(
            name(f"joint{j + 1}_tau"),
            [("实测 τ", t, result.tau_meas[j]),
             ("模型 τ (raw)", t, result.tau_model[j]),
             ("模型 τ (scaled)", t, result.tau_scaled[j])],
            f"关节 J{j + 1} · 实测 vs 逆动力学", "t (s)", "τ (N·m)",
            colors=["#111", "#1f77b4", "#d62728"],
            dashed=[False, False, True])
        paths[f"j{j + 1}_res"] = svg_line(
            name(f"joint{j + 1}_residual"),
            [("residual (raw)", t,
              [result.tau_meas[j][i] - result.tau_model[j][i]
               for i in range(result.n_samples)]),
             ("residual (scaled)", t,
              [result.tau_meas[j][i] - result.tau_scaled[j][i]
               for i in range(result.n_samples)])],
            f"关节 J{j + 1} · 力矩残差", "t (s)", "τ_meas − τ_model (N·m)",
            y_zero=True, colors=["#1f77b4", "#d62728"], dashed=[False, True])
        paths[f"j{j + 1}_hist"] = svg_hist(
            name(f"joint{j + 1}_residual_hist"),
            [result.tau_meas[j][i] - result.tau_model[j][i]
             for i in range(result.n_samples)],
            f"关节 J{j + 1} · 残差分布",
            xlabel="τ_meas − τ_model (N·m)")

        # 速度分层柱状
        key = f"J{j + 1}"
        strata = result.summary["strata"]["speed"].get(key, {})
        if strata:
            cats = list(strata.keys())
            vals = [strata[c]["rmse"] for c in cats]
            paths[f"j{j + 1}_speed"] = svg_bars(
                name(f"joint{j + 1}_rmse_by_speed"), cats,
                [("RMSE", vals)], f"关节 J{j + 1} · 按速度分层 RMSE",
                ylabel="RMSE (N·m)")

    labels = [f"J{j + 1}" for j in range(n)]
    raw = result.summary["metrics"]["raw"]["per_joint"]
    scaled = result.summary["metrics"]["scaled"]["per_joint"]
    paths["rmse"] = svg_bars(
        name("rmse_by_joint"), labels,
        [("raw", [raw[l]["rmse"] for l in labels]),
         ("scaled", [scaled[l]["rmse"] for l in labels])],
        "逐轴 RMSE (raw = 物理量, scaled = 固件前馈口径)", ylabel="RMSE (N·m)")
    paths["bias"] = svg_bars(
        name("bias_by_joint"), labels,
        [("bias", [raw[l]["bias"] for l in labels])],
        "逐轴平均残差 (bias = mean(τ_meas − τ_model))",
        ylabel="bias (N·m)", colors=["#ff7f0e"])

    paths["parity"] = svg_scatter(
        name("parity_measured_vs_model"),
        [(f"J{j + 1}", result.tau_model[j], result.tau_meas[j]) for j in range(n)],
        "实测 vs 模型 (所有关节)")

    delay = result.summary["delay"]
    series = []
    for j in range(n):
        key = f"J{j + 1}"
        if key not in delay:
            continue
        curve = delay[key].get("curve") or []
        series.append((key, [c[0] / result.fs * 1000.0 for c in curve],
                       [c[1] for c in curve]))
    if series:
        paths["delay"] = svg_line(
            name("delay_scan"),
            series, "延迟扫描 · RMSE vs 实测时移", "lag (ms)", "RMSE (N·m)")

    # 归一化残差汇总 (各轴除以自身 RMSE, 便于同图比较)
    pooled = []
    for j in range(n):
        r = raw[f"J{j + 1}"]["rmse"]
        if not r or r <= 1e-12:
            continue
        pooled += [(result.tau_meas[j][i] - result.tau_model[j][i]) / r
                   for i in range(result.n_samples)]
    if pooled:
        paths["hist"] = svg_hist(name("residual_normalized_hist"), pooled,
                                 "所有关节 · 归一化残差 (residual / RMSE_j)",
                                 xlabel="residual / RMSE_j")
    return paths
