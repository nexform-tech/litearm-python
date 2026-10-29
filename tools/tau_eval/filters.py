# -*- coding: utf-8 -*-
"""重采样、零相位低通与差分 —— 纯标准库，无 numpy。

为什么坚持零相位：100 Hz 流里由 dq 差分得到的 ddq 噪声很大，必须低通；
但普通因果滤波会把 ddq 整体推迟若干毫秒，误差分析会把「相位延迟」误判成
「惯量模型错」。前后各滤一遍 (filtfilt) 在通带内净相位为 0，只在两端留下
约 `2/fc` 秒的边沿瞬态，分析时按边沿裁剪。
"""
from __future__ import annotations

import bisect
import math
from typing import Iterable, List, Sequence, Tuple


def uniform_grid(t0: float, t1: float, fs: float) -> List[float]:
    """[t0, t1] 上等间隔采样时刻 (含两端)。"""
    if fs <= 0:
        raise ValueError("fs 必须 > 0")
    n = int(math.floor((t1 - t0) * fs + 1e-9)) + 1
    return [t0 + k / fs for k in range(max(n, 1))]


def linear_resample(t: Sequence[float], x: Sequence[float],
                    grid: Sequence[float]) -> List[float]:
    """对一维序列做线性插值；grid 超出 [t0,t1] 时用端点值钳住。"""
    if len(t) != len(x):
        raise ValueError("t 与 x 长度不一致")
    if not t:
        return [float("nan")] * len(grid)
    out = []
    for g in grid:
        if g <= t[0]:
            out.append(float(x[0]))
            continue
        if g >= t[-1]:
            out.append(float(x[-1]))
            continue
        j = bisect.bisect_right(t, g)
        t0, t1 = t[j - 1], t[j]
        x0, x1 = float(x[j - 1]), float(x[j])
        if t1 <= t0:
            out.append(x1)
        else:
            a = (g - t0) / (t1 - t0)
            out.append(x0 + a * (x1 - x0))
    return out


def _butter2_coeffs(fs: float, fc: float) -> Tuple[List[float], List[float]]:
    """二阶 Butterworth 低通 (双线性变换 + 预畸变)。"""
    if not (0.0 < fc < fs / 2.0):
        raise ValueError(f"截止频率 fc={fc} 必须在 (0, fs/2={fs/2}) 内")
    K = math.tan(math.pi * fc / fs)
    norm = 1.0 + math.sqrt(2.0) * K + K * K
    b0 = K * K / norm
    b = [b0, 2.0 * b0, b0]
    a = [1.0, 2.0 * (K * K - 1.0) / norm, (1.0 - math.sqrt(2.0) * K + K * K) / norm]
    return b, a


def _lfilter(b: Sequence[float], a: Sequence[float], x: Sequence[float]) -> List[float]:
    # a[0] 已归一化为 1；输入输出都用零初始状态。
    b0, b1, b2 = b
    a1, a2 = a[1], a[2]
    x1 = x2 = y1 = y2 = 0.0
    out = []
    for v in x:
        y = b0 * v + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2
        x2, x1 = x1, v
        y2, y1 = y1, y
        out.append(y)
    return out


def _reflect_pad(x: Sequence[float], pad: int) -> List[float]:
    if pad <= 0 or len(x) < 2:
        return list(x)
    pad = min(pad, len(x) - 1)
    left = [x[i] for i in range(pad, 0, -1)]
    right = [x[i] for i in range(len(x) - 2, len(x) - pad - 2, -1)]
    return left + list(x) + right


def lowpass_zero_phase(x: Sequence[float], fs: float, fc: float) -> List[float]:
    """二阶 Butterworth 前后向滤波 (净相位 0)。过短序列原样返回。"""
    n = len(x)
    if n < 4 or not (fc > 0.0):
        return [float(v) for v in x]
    b, a = _butter2_coeffs(fs, fc)
    pad = max(9, int(math.ceil(2.0 * fs / fc)))
    y = _lfilter(b, a, _reflect_pad(x, pad))
    y.reverse()
    y = _lfilter(b, a, y)
    y.reverse()
    core = y[pad:pad + n] if len(y) >= pad + n else y[:n]
    return list(core)


def central_diff(x: Sequence[float], dt: float) -> List[float]:
    """二阶中心差分；端点用单侧一阶差分。"""
    n = len(x)
    if n == 0:
        return []
    if n == 1:
        return [0.0]
    out = [0.0] * n
    for i in range(1, n - 1):
        out[i] = (x[i + 1] - x[i - 1]) / (2.0 * dt)
    out[0] = (x[1] - x[0]) / dt
    out[-1] = (x[-1] - x[-2]) / dt
    return out


def smoothstep(u: float) -> float:
    """0..1 上的五次平滑窗 (端点一阶、二阶导均为 0)。"""
    if u <= 0.0:
        return 0.0
    if u >= 1.0:
        return 1.0
    return u * u * u * (u * (u * 6.0 - 15.0) + 10.0)


def smoothstep_deriv(u: float) -> float:
    if u <= 0.0 or u >= 1.0:
        return 0.0
    return 30.0 * u * u * (1.0 - u) * (1.0 - u)


def mean(vals: Sequence[float]) -> float:
    return sum(vals) / len(vals) if vals else float("nan")


def pstdev(vals: Sequence[float]) -> float:
    if len(vals) < 2:
        return 0.0
    m = mean(vals)
    return math.sqrt(sum((v - m) ** 2 for v in vals) / (len(vals) - 1))


def quantile(vals: Sequence[float], q: float) -> float:
    """线性插值分位数 (与 numpy 默认一致)。"""
    if not vals:
        return float("nan")
    s = sorted(float(v) for v in vals)
    if len(s) == 1:
        return s[0]
    pos = (len(s) - 1) * min(max(q, 0.0), 1.0)
    lo = int(math.floor(pos))
    hi = min(lo + 1, len(s) - 1)
    a = pos - lo
    return s[lo] * (1.0 - a) + s[hi] * a


def pearson(x: Sequence[float], y: Sequence[float]) -> float:
    if len(x) != len(y) or len(x) < 2:
        return float("nan")
    mx, my = mean(x), mean(y)
    sxx = sum((a - mx) ** 2 for a in x)
    syy = sum((b - my) ** 2 for b in y)
    if sxx <= 0.0 or syy <= 0.0:
        return float("nan")
    sxy = sum((a - mx) * (b - my) for a, b in zip(x, y))
    return sxy / math.sqrt(sxx * syy)


def linear_fit(x: Sequence[float], y: Sequence[float]) -> Tuple[float, float, float]:
    """最小二乘 y ≈ a + b·x，返回 (a, b, r)。"""
    if len(x) != len(y) or len(x) < 2:
        return (float("nan"), float("nan"), float("nan"))
    mx, my = mean(x), mean(y)
    sxx = sum((v - mx) ** 2 for v in x)
    if sxx <= 0.0:
        return (float("nan"), float("nan"), float("nan"))
    sxy = sum((a - mx) * (b - my) for a, b in zip(x, y))
    b = sxy / sxx
    a = my - b * mx
    return (a, b, pearson(x, y))


def r2_score(meas: Sequence[float], pred: Sequence[float]) -> float:
    if len(meas) != len(pred) or not meas:
        return float("nan")
    m = mean(meas)
    ss_tot = sum((v - m) ** 2 for v in meas)
    ss_res = sum((a - b) ** 2 for a, b in zip(meas, pred))
    if ss_tot <= 0.0:
        return float("nan")
    return 1.0 - ss_res / ss_tot
