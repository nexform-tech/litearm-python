#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generate a hardware-free example session from the factory model itself.

The synthetic `tau` is produced by the same RNEA used for analysis, so the
residual is pure numerical noise — a useful smoke test for the whole
collect → analyze → plot → report pipeline.

    python3 tools/tau_eval/examples/make_synthetic.py [--outdir DIR]

Writes status.csv, model.json, meta.json, timeseries.csv, metrics.json,
figures/*.svg, report.md. Generated files are meant to stay untracked
(AGENTS.md: never commit generated artifacts).
"""
import argparse
import json
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_TOOLS = os.path.dirname(os.path.dirname(_HERE))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)

from tau_eval.analyze import (AnalyzeOptions, analyze_session,  # noqa: E402
                              save_analysis)
from tau_eval.collect import Sample, Session  # noqa: E402
from tau_eval.model import factory_snapshot  # noqa: E402
from tau_eval.plots import generate_all  # noqa: E402
from tau_eval.report import write_report  # noqa: E402
from tau_eval import rnea  # noqa: E402


def build_session(fs=100.0, duration=2.0, amp=0.15, freq=0.3):
    snap = factory_snapshot()
    snap.firmware = "synthetic-example"
    samples = []
    for k in range(int(duration * fs) + 1):
        t = k / fs
        q, dq, ddq = [], [], []
        for j in range(7):
            ph = 2 * math.pi * freq * t + 0.3 * j
            q.append(amp * math.sin(ph))
            dq.append(amp * 2 * math.pi * freq * math.cos(ph))
            ddq.append(-amp * (2 * math.pi * freq) ** 2 * math.sin(ph))
        d = rnea.theory_torques(snap, q, dq, ddq)
        samples.append(Sample(
            t=t, seq=k, mode=3, mode_name="POS", flags=0x200, joint_fault=0,
            q=q, dq=dq, tau=list(d["total"]),
            t_mos=[30.0] * 7, t_coil=[31.0] * 7, err=[0] * 7))
    return snap, Session(samples, {"fs": fs, "scenario": "synthetic-sine"})


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--outdir", default=os.path.join(_HERE, "synthetic_sine"),
                    help="output directory (default: examples/synthetic_sine)")
    ap.add_argument("--fs", type=float, default=100.0)
    ap.add_argument("--duration", type=float, default=2.0)
    ap.add_argument("--amp", type=float, default=0.15)
    ap.add_argument("--freq", type=float, default=0.3)
    args = ap.parse_args(argv)

    snap, sess = build_session(args.fs, args.duration, args.amp, args.freq)
    outdir = os.path.abspath(args.outdir)
    os.makedirs(outdir, exist_ok=True)
    sess.dump(outdir)
    with open(os.path.join(outdir, "model.json"), "w", encoding="utf-8") as f:
        json.dump(snap.to_dict(), f, ensure_ascii=False, indent=2)

    result = analyze_session(sess, snap, AnalyzeOptions())
    save_analysis(result, outdir)
    figs = generate_all(result, os.path.join(outdir, "figures"))
    report = write_report(
        result, figs, outdir,
        command="python3 tools/tau_eval/examples/make_synthetic.py")

    agg = result.summary["metrics"]["raw"]["aggregate"]
    print(f"session   {os.path.join(outdir, 'status.csv')}")
    print(f"mean RMSE {agg['mean_rmse']:.3e} N·m   "
          f"max RMSE {agg['max_rmse']:.3e} N·m ({agg['worst_joint']})")
    print(f"figures   {len(figs)}")
    print(f"report    {report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
