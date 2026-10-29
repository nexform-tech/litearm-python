# -*- coding: utf-8 -*-
"""命令行入口: 一条命令完成 采集 → 逆动力学 → 图 → 报告。

子命令:
    run       默认。连接真机, 跑 hold/sine/poses 三个场景, 逐个分析出图出报告。
    collect   只采集一个场景 (仍会分析, 除非 --no-analyze)。
    analyze   对已有会话目录 (或包含多个会话的 run 目录) 做离线分析。

安全约定与 examples/ 一致: 默认只读打印计划; 任何 enable/运动/采集都要 `--go`。
采集结束不 disable (该位形失能会因自重坠回), 急停请放手边。
"""
from __future__ import annotations

import argparse
import datetime as _dt
import os
import shlex
import sys
from typing import List, Optional, Sequence, Tuple

from .analyze import AnalyzeOptions, analyze_dir
from .collect import (MIN_MARGIN, ScenarioSpec, fetch_from_arm,
                      read_joint_limits, run_scenario)
from .plots import generate_all
from .report import write_report

#: 默认三场景的构型偏移模板 (rad)。乘缩放系数后仍要过软限位余量检查。
_POSE_MAGS = (0.10, 0.10, 0.10, 0.08, 0.10, 0.10, 0.10)
_POSE_SIGNS = (
    (1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0),
    (-1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0),
)


def _gravity(s: str) -> Tuple[float, float, float]:
    parts = [p for p in s.replace(" ", "").split(",") if p]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("重力向量需要 3 个分量, 如 0,9.81,0")
    return tuple(float(v) for v in parts)  # type: ignore[return-value]


def _joints(s: str) -> List[int]:
    out = []
    for tok in s.replace(" ", "").split(","):
        if tok:
            j = int(tok) - 1
            if j < 0:
                raise argparse.ArgumentTypeError("关节号从 1 开始")
            out.append(j)
    return out


def _common_collect_args(ap: argparse.ArgumentParser) -> None:
    ap.add_argument("--port", default=None, help="串口 (默认自动发现 1d50:606f)")
    ap.add_argument("--go", action="store_true",
                    help="真正 enable/运动/采集 (默认只读, 只打印计划)")
    ap.add_argument("--outdir", default=None,
                    help="输出根目录 (默认 runs/<时间戳>_tau_eval)")
    ap.add_argument("--fs", type=float, default=100.0, help="落盘采样率, 1..100Hz")
    ap.add_argument("--scenario", default="all",
                    choices=["all", "hold", "sine", "poses"],
                    help="场景: all=hold+sine+poses")
    ap.add_argument("--hold-s", type=float, default=3.0, help="静态持位时长 (s)")
    ap.add_argument("--sine-s", type=float, default=10.0, help="正弦段时长 (s)")
    ap.add_argument("--amp", type=float, default=0.12, help="正弦幅值 (rad), 受软限位自动收窄")
    ap.add_argument("--freq", type=float, default=0.20, help="正弦频率 (Hz)")
    ap.add_argument("--command-hz", type=float, default=50.0,
                    help="连续伺服 (move_js) 重发率, ≥10Hz 才不断看门狗")
    ap.add_argument("--sine-joints", type=_joints, default=None,
                    help="正弦激励的关节, 逗号分隔 1 基 (默认全部)")
    ap.add_argument("--dwell-s", type=float, default=1.2, help="poses 每姿态停驻 (s)")
    ap.add_argument("--speed", type=float, default=0.2, help="poses 的 movej speed (0..1)")
    ap.add_argument("--ambient-c", type=float, default=None,
                    help="手工记录的室温 (°C); 固件只报绕组/MOS 温度")
    ap.add_argument("--parquet", action="store_true",
                    help="同时写 Parquet (需要 pyarrow, 否则自动跳过)")
    ap.add_argument("--gravity", type=_gravity, default=None,
                    help="分析用的重力向量, 如台架为 +Y 时写 0,9.81,0")
    ap.add_argument("--cutoff-dq", type=float, default=15.0, help="q/dq 零相位低通 (Hz)")
    ap.add_argument("--cutoff-ddq", type=float, default=5.0, help="ddq 零相位低通 (Hz)")
    ap.add_argument("--cutoff-tau", type=float, default=10.0, help="实测 τ 低通 (Hz)")
    ap.add_argument("--trim-s", type=float, default=None,
                    help="两端裁剪 (s), 默认 max(2/cutoff_ddq, 0.1)")
    ap.add_argument("--max-delay-ms", type=float, default=100.0,
                    help="延迟扫描范围 ±ms")
    ap.add_argument("--no-plot", action="store_true", help="只算指标, 不生成 SVG")
    ap.add_argument("--no-report", action="store_true", help="不写 report.md")


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="tau_eval",
        description="LiteArm 电流环力矩估计精度评估: 采集 → 逆动力学 → 误差图/报告")
    sub = ap.add_subparsers(dest="cmd")

    p_run = sub.add_parser("run", help="一条命令跑三场景并出图出报告 (默认)")
    _common_collect_args(p_run)

    p_col = sub.add_parser("collect", help="只采集 (默认仍分析)")
    _common_collect_args(p_col)
    p_col.add_argument("--no-analyze", action="store_true", help="只落盘, 不分析")

    p_an = sub.add_parser("analyze", help="分析已有会话/run 目录")
    p_an.add_argument("path", help="会话目录 (含 status.csv) 或 run 目录")
    p_an.add_argument("--outdir", default=None, help="分析输出目录 (默认写回原目录)")
    p_an.add_argument("--gravity", type=_gravity, default=None,
                      help="重力向量, 如台架为 +Y 时写 0,9.81,0")
    p_an.add_argument("--cutoff-dq", type=float, default=15.0)
    p_an.add_argument("--cutoff-ddq", type=float, default=5.0)
    p_an.add_argument("--cutoff-tau", type=float, default=10.0)
    p_an.add_argument("--trim-s", type=float, default=None)
    p_an.add_argument("--max-delay-ms", type=float, default=100.0)
    p_an.add_argument("--no-plot", action="store_true")
    p_an.add_argument("--no-report", action="store_true")
    return ap


def _analyze_options(args) -> AnalyzeOptions:
    return AnalyzeOptions(
        cutoff_dq=args.cutoff_dq, cutoff_ddq=args.cutoff_ddq,
        cutoff_tau=args.cutoff_tau, trim_s=args.trim_s,
        gravity_override=args.gravity, max_delay_ms=args.max_delay_ms)


# ---------------------------------------------------------------------------
# 场景规划
# ---------------------------------------------------------------------------

def _adaptive_amp(center: Sequence[float], limits: Sequence[Tuple[float, float]],
                  want: float, joints: Sequence[int],
                  margin: float = MIN_MARGIN) -> float:
    """所有选中关节都留出 margin 时, 同相正弦的最大公共幅值。"""
    caps = []
    for j in joints:
        lo, hi = limits[j]
        caps.append(min(center[j] - lo, hi - center[j]) - margin)
    return min([want] + caps)


def _build_poses(start: Sequence[float], limits: Sequence[Tuple[float, float]],
                 margin: float = MIN_MARGIN) -> Tuple[List[List[float]], float]:
    """生成 "起点 -> 正偏移 -> 负偏移 -> 起点" 的慢速序列, 自动收窄幅度。"""
    for scale in (1.0, 0.7, 0.5, 0.35, 0.25):
        poses = []
        for signs in _POSE_SIGNS:
            pose = [start[j] + scale * signs[j] * _POSE_MAGS[j]
                    for j in range(len(start))]
            poses.append(pose)
        poses.append(list(start))
        ok = True
        for p in poses:
            for j, q in enumerate(p):
                lo, hi = limits[j]
                if q - lo < margin or hi - q < margin:
                    ok = False
                    break
            if not ok:
                break
        if ok:
            return poses, scale
    raise RuntimeError(
        "当前位形离软限位太近, 生成不出安全的多姿态序列; 先把臂挪到更居中的位形")


def _make_specs(arm, args, limits) -> List[ScenarioSpec]:
    st = arm.get_state().value
    if st is None:
        raise RuntimeError("拿不到状态帧")
    q0 = list(st.q)
    n = len(q0)
    joints = args.sine_joints if args.sine_joints is not None else list(range(n))
    for j in joints:
        if not 0 <= j < n:
            raise SystemExit(f"⛔ --sine-joints 里的 {j + 1} 越界 (1..{n})")

    kinds = ["hold", "sine", "poses"] if args.scenario == "all" else [args.scenario]
    specs: List[ScenarioSpec] = []
    for kind in kinds:
        if kind == "hold":
            specs.append(ScenarioSpec(kind="hold", name="hold", q_center=list(q0),
                                      duration=args.hold_s))
        elif kind == "sine":
            if not (10.0 <= args.command_hz <= 200.0):
                raise SystemExit("⛔ --command-hz 必须在 [10, 200]")
            amp = _adaptive_amp(q0, limits, args.amp, joints)
            if amp < 0.03:
                raise SystemExit(
                    f"⛔ 当前位形只剩 {amp:.3f} rad 正弦空间 (<0.03), 先换一个更居中的位形")
            if amp < args.amp - 1e-9:
                print(f"[规划] 正弦幅值受软限位限制: {args.amp:.3f} -> {amp:.3f} rad")
            specs.append(ScenarioSpec(
                kind="sine", name="sine", q_center=list(q0), duration=args.sine_s,
                amp=amp, freq=args.freq, joints=list(joints),
                command_hz=args.command_hz))
        elif kind == "poses":
            poses, scale = _build_poses(q0, limits)
            if scale < 1.0:
                print(f"[规划] 多姿态幅度收窄到 {scale:.2f}× 以留出限位余量")
            specs.append(ScenarioSpec(
                kind="poses", name="poses", q_center=list(q0), poses=poses,
                dwell=args.dwell_s, speed=args.speed))
    return specs


def _describe(spec: ScenarioSpec) -> str:
    if spec.kind == "hold":
        return f"hold: 静止持位 {spec.duration:.1f}s"
    if spec.kind == "sine":
        js = spec.joints if spec.joints is not None else []
        return (f"sine: {spec.duration:.1f}s, {spec.amp:.3f} rad @ {spec.freq:.2f}Hz, "
                f"关节 {[j + 1 for j in js]}, 伺服 {spec.command_hz:.0f}Hz")
    return (f"poses: {len(spec.poses or [])} 个位形, dwell {spec.dwell:.1f}s, "
            f"speed {spec.speed:.2f}")


def _print_plan(arm, specs: Sequence[ScenarioSpec]) -> None:
    st = arm.get_state().value
    print(f"[连接] 固件 {getattr(arm, 'firmware', '?')} (n={arm.n}); "
          f"mode={st.mode_name} enabled={st.enabled} flags=0x{st.flags:X}")
    print(f"[位形] q = {[round(v, 4) for v in st.q]}")
    print("[计划]")
    for k, spec in enumerate(specs, 1):
        print(f"  {k}) {_describe(spec)}")
    print("[安全] 目标均过软限位余量检查; 结束后不 disable (失能会因自重坠回)")


# ---------------------------------------------------------------------------
# 输出与分析
# ---------------------------------------------------------------------------

def _timestamp_dir(root: Optional[str]) -> str:
    if root is None:
        root = os.path.join("runs", _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
                            + "_tau_eval")
    if os.path.exists(os.path.join(root, "status.csv")) or \
            (os.path.isdir(root) and any(
                os.path.exists(os.path.join(root, d, "status.csv"))
                for d in os.listdir(root) if os.path.isdir(os.path.join(root, d)))):
        root = root + "_" + _dt.datetime.now().strftime("%H%M%S")
    os.makedirs(root, exist_ok=True)
    return root


def _analyze_one(indir: str, outdir: str, opts: AnalyzeOptions, command: str,
                 no_plot: bool = False, no_report: bool = False):
    os.makedirs(outdir, exist_ok=True)
    result = analyze_dir(indir, outdir, opts)
    figs = {}
    if not no_plot:
        figs = generate_all(result, os.path.join(outdir, "figures"))
    if not no_report:
        write_report(result, figs, outdir, command=command)
    return result


def _print_summary(result) -> None:
    s = result.summary
    agg = s["metrics"]["raw"]["aggregate"]
    print(f"  [{s.get('scenario') or 'session'}] 平均 RMSE {agg['mean_rmse']:.4f} N·m, "
          f"最大 {agg['max_rmse']:.4f} N·m ({agg['worst_joint']}), "
          f"平均 |bias| {agg['mean_abs_bias']:.4f} N·m")
    for d in s.get("diagnosis", []):
        print(f"     → {d['source']}: {d['evidence']}")


def _write_overview(run_dir: str, results: Sequence) -> str:
    lines = [f"# tau_eval 运行总览 — {os.path.basename(run_dir)}", ""]
    lines.append("| 场景 | 样本 | 平均 RMSE (N·m) | 最大 RMSE | 最差轴 | 平均 \\|bias\\| | 报告 |")
    lines.append("|---|---|---|---|---|---|---|")
    for r in results:
        s = r.summary
        agg = s["metrics"]["raw"]["aggregate"]
        sub = s.get("scenario") or ""
        lines.append(
            f"| {sub} | {s['n_samples']} | {agg['mean_rmse']:.4f} | "
            f"{agg['max_rmse']:.4f} | {agg['worst_joint']} | "
            f"{agg['mean_abs_bias']:.4f} | [report]({sub}/report.md) |")
    lines.append("")
    lines.append("图与逐轴指标在各场景子目录: `metrics.json` / `timeseries.csv` / "
                 "`figures/*.svg`。")
    path = os.path.join(run_dir, "summary.md")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    return path


# ---------------------------------------------------------------------------
# 命令
# ---------------------------------------------------------------------------

def _connect(args):
    import litearm as pa
    return pa.Arm(port=args.port or os.environ.get("LITEARM_PORT") or None).connect()


def _command_line(argv: Sequence[str]) -> str:
    return "python3 tools/tau_eval/run.py " + " ".join(shlex.quote(a) for a in argv)


def cmd_run(args, argv: Sequence[str]) -> int:
    arm = _connect(args)
    try:
        st = arm.get_state().value
        if st is not None and st.faulted:
            print(f"⛔ 臂有故障位, 拒绝运行: {st.fault_detail}")
            return 2
        limits = read_joint_limits(arm)
        specs = _make_specs(arm, args, limits)
        _print_plan(arm, specs)
        if not args.go:
            print("\n只读模式: 加 --go 才会 enable/运动/采集。")
            return 0

        root = _timestamp_dir(args.outdir)
        print(f"[输出] {root}")
        snapshot = fetch_from_arm(arm)
        session_dirs = []
        for k, spec in enumerate(specs, 1):
            sub = f"{k:02d}_{spec.kind}"
            d = os.path.join(root, sub)
            print(f"\n[采集 {k}/{len(specs)}] {_describe(spec)}")
            run_scenario(arm, spec, d, fs=args.fs, go=True,
                         ambient_c=args.ambient_c, snapshot=snapshot,
                         parquet=args.parquet)
            session_dirs.append((spec, d))
    finally:
        arm.close()
        print("\n[断开] 已关闭串口 (刻意未 disable: 失能会因自重坠回)")

    opts = _analyze_options(args)
    cmd = _command_line(argv)
    results = []
    print("\n[分析]")
    for spec, d in session_dirs:
        results.append(_analyze_one(d, d, opts, cmd, args.no_plot, args.no_report))
        _print_summary(results[-1])
    if results:
        ov = _write_overview(root, results)
        print(f"\n[完成] 总览 {ov}")
    return 0


def cmd_collect(args, argv: Sequence[str]) -> int:
    arm = _connect(args)
    session_dir = None
    try:
        st = arm.get_state().value
        if st is not None and st.faulted:
            print(f"⛔ 臂有故障位, 拒绝运行: {st.fault_detail}")
            return 2
        limits = read_joint_limits(arm)
        specs = _make_specs(arm, args, limits)
        _print_plan(arm, specs)
        if not args.go:
            print("\n只读模式: 加 --go 才会 enable/运动/采集。")
            return 0
        session_dir = _timestamp_dir(args.outdir)
        print(f"[输出] {session_dir}")
        snapshot = fetch_from_arm(arm)
        for k, spec in enumerate(specs, 1):
            run_scenario(arm, spec, session_dir, fs=args.fs, go=True,
                         ambient_c=args.ambient_c, snapshot=snapshot,
                         parquet=args.parquet)
    finally:
        arm.close()
        print("[断开] 已关闭串口 (刻意未 disable: 失能会因自重坠回)")

    if session_dir and not args.no_analyze:
        result = _analyze_one(session_dir, session_dir, _analyze_options(args),
                              _command_line(argv), args.no_plot, args.no_report)
        _print_summary(result)
    return 0


def _session_dirs(path: str) -> List[str]:
    if os.path.isfile(os.path.join(path, "status.csv")):
        return [path]
    out = []
    if os.path.isdir(path):
        for d in sorted(os.listdir(path)):
            p = os.path.join(path, d)
            if os.path.isfile(os.path.join(p, "status.csv")):
                out.append(p)
    return out


def cmd_analyze(args, argv: Sequence[str]) -> int:
    dirs = _session_dirs(args.path)
    if not dirs:
        print(f"⛔ {args.path} 下没有找到 status.csv")
        return 2
    opts = _analyze_options(args)
    cmd = _command_line(argv)
    results = []
    for d in dirs:
        out = os.path.join(args.outdir, os.path.basename(d)) if args.outdir else d
        result = _analyze_one(d, out, opts, cmd, args.no_plot, args.no_report)
        results.append(result)
        _print_summary(result)
    if len(results) > 1 and args.outdir is None:
        _write_overview(os.path.dirname(dirs[0]), results)
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or (argv[0].startswith("-") and argv[0] not in ("-h", "--help")):
        argv = ["run"] + argv
    ap = build_parser()
    args = ap.parse_args(argv)
    if args.cmd is None:
        ap.print_help()
        return 0
    try:
        if args.cmd == "run":
            return cmd_run(args, argv)
        if args.cmd == "collect":
            return cmd_collect(args, argv)
        if args.cmd == "analyze":
            return cmd_analyze(args, argv)
    except KeyboardInterrupt:
        print("\n[中断] 用户 Ctrl-C; 串口已由 finally 关闭。")
        return 130
    except Exception as e:                          # noqa: BLE001 - 顶层收口
        print(f"⛔ {type(e).__name__}: {e}")
        return 2
    ap.print_help()
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
