"""Offline tests for tools/tau_eval — no hardware, no numpy/matplotlib."""
from __future__ import annotations

import json
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_TOOLS = os.path.join(os.path.dirname(_HERE), "tools")
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)

from tau_eval import filters, rnea  # noqa: E402
from tau_eval.analyze import AnalyzeOptions, analyze_session  # noqa: E402
from tau_eval.collect import (Sample, ScenarioSpec, Session,  # noqa: E402
                              validate_scenario)
from tau_eval.model import factory_snapshot  # noqa: E402
from tau_eval.plots import generate_all  # noqa: E402
from tau_eval.report import write_report  # noqa: E402


# ---------------------------------------------------------------------------
# RNEA: firmware golden probe (dyn.c MODEL_PROBE_Q / G_FACTORY_PROBE)
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("q, expected_max", [
    ([0.0, -0.22, 0.94, 0.0, 0.0, 0.0, 0.0], 2.1340647),
    ([0.0, 0.62, -1.60, 0.0, 0.0, 0.0, 0.0], 7.5729609),
])
def test_rnea_matches_firmware_gravity_probe(q, expected_max):
    snap = factory_snapshot()
    g = rnea.gravity_torque(q, snap.effective_bodies(), (0.0, 0.0, -9.81))
    assert max(abs(v) for v in g) == pytest.approx(expected_max, abs=1.5e-4)


def test_mass_matrix_is_consistent_with_rnea():
    snap = factory_snapshot()
    bodies = snap.effective_bodies()
    q = [0.1, -0.2, 0.3, -0.4, 0.2, 0.1, -0.1]
    ddq = [0.5, -0.3, 0.2, 0.1, -0.2, 0.3, -0.1]
    M = rnea.mass_matrix(q, bodies)
    tau = rnea.inertia_torque(q, ddq, bodies)
    pred = [sum(M[i][j] * ddq[j] for j in range(7)) for i in range(7)]
    for a, b in zip(tau, pred):
        assert a == pytest.approx(b, abs=1e-9)


def test_theory_torques_total_is_sum_of_parts():
    snap = factory_snapshot()
    q = [0.05, -0.1, 0.2, -0.15, 0.1, 0.05, -0.05]
    dq = [0.1, -0.2, 0.15, 0.05, -0.1, 0.2, -0.05]
    ddq = [0.3, -0.1, 0.2, 0.05, -0.15, 0.1, -0.2]
    d = rnea.theory_torques(snap, q, dq, ddq)
    for i in range(7):
        assert d["total"][i] == pytest.approx(
            d["gravity"][i] + d["coriolis"][i] + d["inertia"][i]
            + d["jm"][i] + d["friction"][i], abs=1e-12)


# ---------------------------------------------------------------------------
# Filters / resampling / metrics
# ---------------------------------------------------------------------------

def test_uniform_grid_and_linear_resample():
    grid = filters.uniform_grid(0.0, 1.0, 10.0)
    assert len(grid) == 11
    assert grid[-1] == pytest.approx(1.0)
    out = filters.linear_resample([0.0, 1.0], [0.0, 2.0], [0.0, 0.5, 1.0])
    assert out == [0.0, 1.0, 2.0]
    # 端点外钳住
    assert filters.linear_resample([0.0, 1.0], [3.0, 4.0], [-1.0, 2.0]) == [3.0, 4.0]


def test_zero_phase_lowpass_preserves_dc_and_phase():
    fs = 100.0
    n = 1000
    x = [1.0] * n
    y = filters.lowpass_zero_phase(x, fs, 5.0)
    # 稳态直流增益必须为 1 (边缘反射填充的过渡段会有 ~1e-6 残差, 跳过)
    assert min(y[100:-100]) == pytest.approx(1.0, abs=1e-12)

    t = [k / fs for k in range(n)]
    s = [math.sin(2 * math.pi * 1.0 * ti) for ti in t]
    y = filters.lowpass_zero_phase(s, fs, 10.0)
    assert filters.pearson(s[50:-50], y[50:-50]) > 0.9999
    # 零相位: 峰值时刻不移动 (普通因果滤波会右移)
    # 1 Hz 信号每 100 点一个峰, 取窗口中段的单个峰以避免周期重复取到别的波峰
    lo, hi = 400, 450
    i_s = max(range(lo, hi), key=lambda i: s[i])
    i_y = max(range(lo, hi), key=lambda i: y[i])
    assert abs(i_y - i_s) <= 1


def test_central_diff_matches_analytic_derivative():
    fs = 200.0
    f = 2.0
    t = [k / fs for k in range(400)]
    x = [math.sin(2 * math.pi * f * ti) for ti in t]
    dx = filters.central_diff(x, 1.0 / fs)
    want = [2 * math.pi * f * math.cos(2 * math.pi * f * ti) for ti in t]
    err = max(abs(dx[i] - want[i]) for i in range(2, len(t) - 2))
    # 中心差分截断误差上界 = |x'''| dt^2/6 = (2*pi*f)^3 dt^2/6
    bound = (2 * math.pi * f) ** 3 * (1.0 / fs) ** 2 / 6.0
    assert err < bound * 1.05


def test_metrics_helpers():
    x = [1.0, 2.0, 3.0, 4.0]
    assert filters.mean(x) == 2.5
    assert filters.pstdev(x) == pytest.approx(math.sqrt(5.0 / 3.0))
    assert filters.quantile([0.0, 10.0], 0.5) == 5.0
    assert filters.pearson(x, [2.0, 4.0, 6.0, 8.0]) == pytest.approx(1.0)
    a, b, r = filters.linear_fit(x, [3.0, 5.0, 7.0, 9.0])
    assert (a, b) == pytest.approx((1.0, 2.0))
    assert r == pytest.approx(1.0)
    assert filters.r2_score(x, x) == 1.0


# ---------------------------------------------------------------------------
# Session I/O
# ---------------------------------------------------------------------------

def _jittered_session(n_samples=300, n_joints=7):
    out = []
    t = 0.0
    for k in range(n_samples):
        t += 0.01 if k % 3 else 0.013
        out.append(Sample(
            t=t, seq=k, mode=3, mode_name="POS", flags=0x200, joint_fault=0,
            q=[0.1 * math.sin(t) + 0.01 * j for j in range(n_joints)],
            dq=[0.1 * math.cos(t) for _ in range(n_joints)],
            tau=[0.2 * j for j in range(n_joints)],
            t_mos=[30.0 + j for j in range(n_joints)],
            t_coil=[31.0 + j for j in range(n_joints)],
            err=[0] * n_joints))
    return Session(out, {"fs": 100.0, "scenario": "unit"})


def test_session_resample_is_uniform(tmp_path):
    sess = _jittered_session()
    r = sess.resample(50.0)
    dts = [b.t - a.t for a, b in zip(r.samples, r.samples[1:])]
    assert max(abs(d - 0.02) for d in dts) < 1e-9
    assert r.n == 7
    paths = r.dump(str(tmp_path))
    assert os.path.exists(paths["status_csv"])
    back = Session.load(str(tmp_path))
    assert len(back.samples) == len(r.samples)
    assert back.samples[10].q[3] == pytest.approx(r.samples[10].q[3], abs=1e-9)


# ---------------------------------------------------------------------------
# Scenario safety
# ---------------------------------------------------------------------------

def test_scenario_limit_margin_is_enforced():
    limits = [(-3.0, 3.0)] * 7
    with pytest.raises(ValueError, match="余量"):
        validate_scenario(ScenarioSpec(kind="hold", q_center=[2.95] + [0.0] * 6,
                                       duration=1.0), limits)
    with pytest.raises(ValueError, match="amp"):
        validate_scenario(ScenarioSpec(kind="sine", q_center=[0.0] * 7,
                                       amp=0.0, freq=0.2, duration=1.0), limits)


# ---------------------------------------------------------------------------
# Synthetic end-to-end: generator model == analyzer model
# ---------------------------------------------------------------------------

def _synthetic_session(fs=100.0, duration=2.0):
    snap = factory_snapshot()
    snap.firmware = "synthetic-test"
    samples = []
    amp, f = 0.15, 0.3
    for k in range(int(duration * fs) + 1):
        t = k / fs
        q, dq, ddq = [], [], []
        for j in range(7):
            ph = 2 * math.pi * f * t + 0.3 * j
            q.append(amp * math.sin(ph))
            dq.append(amp * 2 * math.pi * f * math.cos(ph))
            ddq.append(-amp * (2 * math.pi * f) ** 2 * math.sin(ph))
        d = rnea.theory_torques(snap, q, dq, ddq)
        samples.append(Sample(
            t=t, seq=k, mode=3, mode_name="POS", flags=0x200, joint_fault=0,
            q=q, dq=dq, tau=list(d["total"]),
            t_mos=[30.0] * 7, t_coil=[31.0] * 7, err=[0] * 7))
    return snap, Session(samples, {"fs": fs, "scenario": "sine"})


def test_analysis_and_outputs_offline(tmp_path):
    snap, sess = _synthetic_session()
    result = analyze_session(sess, snap, AnalyzeOptions())
    agg = result.summary["metrics"]["raw"]["aggregate"]
    assert agg["n_joints"] == 7
    assert agg["mean_rel_rmse"] < 1e-3      # 同一模型回放, 只应有数值噪声
    assert result.summary["diagnosis"][0]["source"] == "模型与实测高度一致"

    d = tmp_path / "session"
    paths = sess.dump(str(d))
    (d / "model.json").write_text(json.dumps(snap.to_dict()), encoding="utf-8")
    assert os.path.exists(paths["meta_json"])

    figs = generate_all(result, str(d / "figures"))
    assert len(figs) >= 20
    for p in figs.values():
        ET.parse(p)                          # 每个 SVG 都必须是合法 XML

    # x 轴必须覆盖整个绘图区 (曾因 xhi 取了首点的 y 值, 曲线被压到左边缘)
    svg = open(figs["j1_tau"], encoding="utf-8").read()
    xs = [float(m.group(1)) for m in re.finditer(r"[ML](-?[\d.]+),(-?[\d.]+)", svg)]
    assert min(xs) >= 77.5 and max(xs) <= 936.5
    assert max(xs) - min(xs) > 800

    report = write_report(result, figs, str(d), command="pytest")
    text = open(report, encoding="utf-8").read()
    assert "电流环力矩估计精度评估报告" in text
    assert "逐轴指标" in text
    assert "figures/joint1_tau.svg" in text
