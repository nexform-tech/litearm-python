#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 07 · 慢速 movej 的逐拍采集 (速度环抖动分析用, 真机, 需 --go)

做什么: 从当前位形出发, 让末端**水平**平移 `--dist`(默认 100mm), 并同时录**两路**
互补的数据 (速度环的输入/输出 vs 反馈):

  路① `*_status.csv` —— **100Hz 实测流**, 从状态帧取 (`usb_cmd.c:1218-1220` 的
       `joint[i].q/.dq/.tau`), 给速度环补上**反馈**那一路 (实测速度只在这里有)。
       列: t, seq, mode, flags, joint_fault, q0..6, dq0..6, tau0..6
       靠 `seq` 逐帧对账 (丢帧如实报数, 不掩盖)。走的是有间歇掉帧的 CDC 上行。
       探头挂在 SDK 状态帧的唯一漏斗 `_Ack._on_status` 上 —— 全程**只有一个读口**,
       不用第二个线程去抢字节流。

  路② `<tag>.csv` —— **300Hz 固件日志** (`CMD_LOG_CTRL 0x2D` / `CMD_LOG_READ 0x2E`),
       固件侧逐拍记录, 绕开 CDC 掉帧:
       列: tick, t, q_ref0..6, dq0..6, tau0..6
       `tick` = 控制拍号 (300Hz), `q_ref` = **参考**位置, `dq` = **指令**速度 dq_s,
       `tau` = **实测**力矩。⚠ 没有实测位置/速度 —— 那两路只在路①里。

为什么默认跑**四段**: `--legs` 逐段显式声明轨迹与 `ff_mask` (见下), 默认序列
  `out:on, back:on, out:off, back:on`
给的是"**同向 A/B** (段1 vs 段3, 唯一变量是 `FF_VELREF`) + **逐点重复性** (段2 vs 段4,
完全相同的运动跑两遍)"。⚠ 别用"去程 vs 回程"做 A/B —— 方向也是变量, 结论不成立
(踩过)。

⚠ **`FF_VELREF` 与日志 `dq` 列的关系**: `log_capture_tick` 记的是 `dq_s`, 而出厂
`ff_mask` 一般含 `FF_VELREF(0x100)`, 固件在 `control_loop.c` 里把 `kd·dq` 移进 tau
并令 `dq_s=0`, **且这发生在打点之前** ⇒ 出厂配置下日志的 `dq` 列恒为 0 (tau 列仍是
真话)。要看真实 `dq_s` 就把该位清掉 (`--legs ...,out:off`, 或 `--ff-mask 0x0BF`)。

**可复用的三个"拧旋钮"** (都只写 RAM, `finally` 里无条件恢复 + 回读校验):
  `--ff-mask 0x19F`      前馈逐位消融 (0x27; 该分支无 `ctrl_is_armed` 门控, 使能态可写)
  `--kd 6=1.25,7=1.25`   逐关节改 `mit_kd` (0x22, 同上可写态)
  `--payload 1.0:0.03,0,0`  末端负载 mass/com (0x28 item4/5; com 在 **ee_link 系**)
⚠ **一律不要先 `disable()`** —— 该位形失能会让臂因自重坠回。

安全: 默认只读, 加 `--go` 才动。跑完**刻意不 disable**: 该位形失能会因自重坠回,
保持使能持位才是安全终态。急停在手边。脚本自带**方向 IK 预检 + 终点软限位余量门槛**
(低于 `MIN_MARGIN` 直接弃用该方向)、**缓冲区截断体检**、以及每段的隐藏帧对账。

运行:
  python3 examples/07_vel_jitter_trace.py                      # 只读: 打印计划
  python3 examples/07_vel_jitter_trace.py --go                # 真跑 (默认四段)
  python3 examples/07_vel_jitter_trace.py --go --legs out:on,back:on   # 自定义序列
  python3 examples/07_vel_jitter_trace.py --go --dur 2.5      # 改速度 (≈40 mm/s)
"""
import argparse
import json
import math
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import make_arm

import litearm as pa
from litearm import _protocol as P
from litearm.state import decode_state

#: 采集拍数上限 (固件 LOG_MAX_SAMPLES=2400 ≈ 8s@300Hz) —— 整段运动必须落在里面,
#: 装不下时固件写满自停、运动照走, 结果是"数据被截断但不报错", 所以下面要显式体检。
LOG_TICKS = 2400
CTRL_HZ = 300.0

#: 各轴 v/a/j 上限 (litearm-stm32 `params/defaults.c` 整臂表)。`speed` 倍率是
#: **v/a/j 三者同比缩放** (control_loop.c 注释: 只缩 v 会让低速档用满加速度冲向低速度
#: -> 一顿一顿), 所以时间随 1/speed 变、而加速度相时长 v/a+a/j 与 speed 无关。
#: ⚠ 这三个量 SDK 读不回来 (0x24 只给 kp/kd/tau_max/q_min/q_max), 这里用编译期默认;
#: 实际时长以采集到的 tick 跨度为真, 脚本会打印实测值对账。
AXIS_LIMITS = [  # (speed_limit, acc_max, jerk)
    (2.00, 8.0, 160.0),   # J1 DM6248P
    (2.00, 8.0, 160.0),   # J2 DM6248P
    (1.75, 7.0, 140.0),   # J3 DM4340
    (1.75, 7.0, 140.0),   # J4 DM4340
    (2.00, 9.0, 180.0),   # J5 DM4310
    (2.00, 9.0, 180.0),   # J6 DM4310
    (2.00, 9.0, 180.0),   # J7 DM4310
]
#: 候选水平方向 (基座系单位向量) —— 只做水平平移, 姿态不动
DIRECTIONS = [("-X", (-1.0, 0.0)), ("+X", (1.0, 0.0)),
              ("-Y", (0.0, -1.0)), ("+Y", (0.0, 1.0))]
#: 终点各轴到软限位的最小余量门槛 (rad) —— 低于此值直接弃用该方向。
#: 起因: 一次只读预检里 "+X" 让 J2 落到 q_min **恰好相等**, 而位置越限锁存不受 enabled
#: 门控、reset 也清不掉 (J4 软限锁死那类故障), 不允许把余量赌成 0。
MIN_MARGIN = 0.15


def axis_time(d, v, a, j):
    """单轴受限 S 曲线从静止到静止走 `d` 所需时长 (v/a/j 为该档实际限值)。

    到达 v 的梯形剖面: T = d/v + (v/a + a/j); 无匀速段的三角剖面用 2√(d/a) 近似。
    本用例算出来的档位都落在梯形分支 (脚本会打印是否命中退化分支)。
    """
    if d <= 1e-9:
        return 0.0
    t_acc = v / a + a / j
    if d >= v * t_acc:
        return d / v + t_acc
    return 2.0 * math.sqrt(d / a) + a / j


def pick_target(arm, q_start, tcp, dist):
    """在四个水平方向里挑一个 IK 可解、且各轴离软限位余量最大的目标。

    返回 (name, q_target, min_margin, deltas, tcp_target); 都不合格则抛 RuntimeError。
    """
    lim = [(jp.q_min, jp.q_max) for jp in arm.params.all_joint_params()]
    best = None
    print("\n[方向预检] (只读 IK, 不动臂)")
    print("  方向     IK     max|Δq|   终点最小余量   余量所在的轴")
    for name, (ux, uy) in DIRECTIONS:
        tgt = [tcp[0] + ux * dist, tcp[1] + uy * dist, tcp[2],
               tcp[3], tcp[4], tcp[5]]
        try:
            q_t = arm.ik(tgt, q_seed=q_start)
        except pa.LiteArmError as e:
            print(f"  {name:<8} FAIL   {type(e).__name__}: {str(e)[:38]}")
            continue
        except pa.TransportError as e:
            print(f"  {name:<8} FAIL   {type(e).__name__}: {str(e)[:38]}")
            continue
        marg = [min(q_t[i] - lim[i][0], lim[i][1] - q_t[i]) for i in range(7)]
        deltas = [q_t[i] - q_start[i] for i in range(7)]
        j = marg.index(min(marg))
        print(f"  {name:<8} ok     {max(abs(v) for v in deltas):6.3f}   "
              f"{min(marg):6.3f} (J{j + 1})      J{j + 1}")
        if min(marg) < MIN_MARGIN:
            print(f"           ⚠ 余量 {min(marg):.3f} < {MIN_MARGIN} rad, 弃用")
            continue
        if best is None or min(marg) > best[2]:
            best = (name, q_t, min(marg), deltas, tgt)
    if best is None:
        raise RuntimeError(f"四个水平方向都没有余量 >= {MIN_MARGIN} rad 的解 —— "
                           f"换个位形再试, 别硬走")
    return best


def solve_speed(deltas, target_t):
    """求让**最慢轴**恰好花 `target_t` 秒的 speed 倍率。

    T_i(k) = d_i/(v_i·k) + c_i, c_i = v_i/a_i + a_i/j_i (与 k 无关) —— 直接解出:
    k = max_i d_i / (v_i·(target_t - c_i))。
    """
    ks = []
    for i, d in enumerate(deltas):
        v, a, j = AXIS_LIMITS[i]
        c = v / a + a / j
        if target_t <= c:
            raise ValueError(f"目标时长 {target_t}s 短于 J{i + 1} 的加速相 {c:.2f}s")
        if abs(d) > 1e-9:
            ks.append(abs(d) / (v * (target_t - c)))
    return max(ks), 1.0 / max(ks) if ks else 0.0


def predict_times(deltas, k):
    return [axis_time(abs(d), v * k, a * k, j * k)
            for d, (v, a, j) in zip(deltas, AXIS_LIMITS)]


def capture_status_stream(arm, t_end, rec):
    """把 RSP_STATUS 帧喂到采集窗结束 (给 `movej()` 返回后的尾巴补帧)。

    `rec` 元素 = (host 单调时刻, 原始载荷)。只走 `read_frame`, 不解析、不判到位。
    """
    while True:
        left = t_end - time.monotonic()
        if left <= 0:
            return
        fr = arm._tr.read_frame(min(0.05, max(0.001, left)))
        if fr is None:
            continue
        c, p = fr
        if c == P.RSP_STATUS:
            rec.append((time.monotonic(), p))


def write_status_csv(rec, path, n):
    """写 100Hz 实测流 CSV: t, seq, mode, flags, joint_fault, q*, dq*, tau*。

    ⚠ 列里是**实测**值 (`usb_cmd.c:1218-1220` 取的 `joint[i].q/.dq/.tau`), 与 300Hz
    日志的参考侧 (`q_ref`/`dq_s`) 正好互补 —— 速度环的**反馈**这一路只在这里。
    ⚠ 状态帧载荷不带控制拍号, 与 300Hz 日志只能按主机时刻粗对齐。
    """
    t0 = rec[0][0]
    seqs = []
    rows = []
    for ts, payload in rec:
        st = decode_state(payload)
        seqs.append(st.seq)
        rows.append((ts - t0, st))
    with open(path, "w") as f:
        cols = (["t", "seq", "mode", "flags", "joint_fault"]
                + [f"q{i}" for i in range(n)] + [f"dq{i}" for i in range(n)]
                + [f"tau{i}" for i in range(n)])
        f.write(",".join(cols) + "\n")
        for t, st in rows:
            f.write("%.6f,%d,%d,%d,%d," % (t, st.seq, st.mode, st.flags, st.joint_fault)
                    + ",".join("%.9g" % v for v in st.q) + ","
                    + ",".join("%.9g" % v for v in st.dq) + ","
                    + ",".join("%.9g" % v for v in st.tau) + "\n")
    gaps = [b - a for a, b in zip(seqs, seqs[1:]) if b - a > 1]
    return {
        "n_status": len(rows),
        "status_span_s": round(rows[-1][0], 3) if rows else 0.0,
        "seq_first": seqs[0] if seqs else None,
        "seq_last": seqs[-1] if seqs else None,
        "seq_expected": (seqs[-1] - seqs[0] + 1) if seqs else 0,
        "seq_missing": (seqs[-1] - seqs[0] + 1 - len(seqs)) if seqs else 0,
        "seq_gap_max": max(gaps) - 1 if gaps else 0,
        "rate_hz": round(len(rows) / rows[-1][0], 1) if rows and rows[-1][0] > 0 else 0.0,
    }


def apply_kd(arm, spec):
    """`--kd 6=1.25,7=1.25` (关节号 1 基) -> 逐个改写 `mit_kd` (RAM), 返回原值字典。

    为什么用 `kd`: 整臂 J5/J6/J7 的 `kd_extra=0`, 故 `mit_kd` 就是有效速度阻尼增益;
    改它 = 直接拧速度环增益, 是区分"机械谐振"与"驱动器量化自激"的关键旋钮。
    `0x22` 在固件里**无** `ctrl_is_armed` 门控 (usb_cmd.c `CMD_SET_JOINT_PARAM`),
    使能态可写 -> 不必先 disable (先 disable 会让臂因自重坠回)。
    改写后**逐关节回读校验**, 不符直接中止 (绝不带着未知增益跑)。
    """
    orig = {}
    for tok in (x.strip() for x in spec.split(",") if x.strip()):
        js, _, vs = tok.partition("=")
        idx = int(js.lstrip("Jj")) - 1
        jp = arm.params.get_joint_param(idx).value
        if idx not in orig:
            orig[idx] = (jp.kp, jp.kd, jp.tau_max)
        arm.params.set_joint_param(idx, jp.kp, float(vs), jp.tau_max)
        got = arm.params.get_joint_param(idx).value
        ok = abs(got.kd - float(vs)) < 1e-6
        print(f"      J{idx + 1} kd {jp.kd:.2f} -> {got.kd:.2f} "
              f"(kp={got.kp:.1f}, τmax={got.tau_max:.1f}) {'ok' if ok else '❌'}")
        if not ok:
            sys.exit(f"⛔ kd 没写进去 (要 {vs} 读到 {got.kd}), 中止")
    return orig


def restore_kd(arm, orig):
    for idx, (kp, kd, tm) in (orig or {}).items():
        arm.params.set_joint_param(idx, kp, kd, tm)
    for idx, (kp, kd, tm) in (orig or {}).items():
        got = arm.params.get_joint_param(idx).value
        print(f"      J{idx + 1} kd 恢复 -> {got.kd:.2f} "
              f"{'ok' if abs(got.kd - kd) < 1e-6 else '❌ 请手工写回 %.2f' % kd}")


def apply_payload(arm, spec, settle_s=3.0):
    """`--payload 1.0:0.03,0,0` -> 写 `payload_mass`(0x28 item4) / `payload_com`(item5 sub0..2)。

    com 在 **ee 法兰系** (`litearm_id.urdf` 里 ee_link 的 x 轴 = J7 轴 = 工具轴方向)。
    让固件的前馈知道负载, 姿态/力矩分布才与"模型正确"时一致 —— 测谐振时唯一的变量
    就只剩机械惯量。⚠ 写入瞬间重力前馈会跳变, 臂会动一点点(本次实测 ~1°) 并重新收敛,
    故写入后先等 `settle_s` 秒再开始跑。返回原值字典。
    """
    mass_s, _, com_s = spec.partition(":")
    mass = float(mass_s)
    com = [float(v) for v in com_s.split(",")] if com_s else [0.0, 0.0, 0.0]
    if len(com) != 3:
        sys.exit("⛔ payload com 需 3 个分量")
    orig = (arm.get_ff_scalar(4, 0).value, [arm.get_ff_scalar(5, s).value for s in range(3)])
    arm.set_ff_scalar(4, 0, mass)
    for s in range(3):
        arm.set_ff_scalar(5, s, com[s])
    got_m = arm.get_ff_scalar(4, 0).value
    got_c = [arm.get_ff_scalar(5, s).value for s in range(3)]
    ok = abs(got_m - mass) < 1e-6 and all(abs(got_c[s] - com[s]) < 1e-6 for s in range(3))
    print(f"      payload_mass {orig[0]:.3f} -> {got_m:.3f} kg, "
          f"com {[round(v, 4) for v in orig[1]]} -> {[round(v, 4) for v in got_c]} "
          f"{'ok' if ok else '❌'}")
    if not ok:
        sys.exit(f"⛔ payload 没写进去 (要 {mass}/{com} 读到 {got_m}/{got_c}), 中止")
    if settle_s:
        print(f"      等 {settle_s:.0f}s 让臂在有负载前馈下重新收敛…")
        time.sleep(settle_s)
    return orig


def restore_payload(arm, orig):
    if not orig:
        return
    arm.set_ff_scalar(4, 0, orig[0])
    for s in range(3):
        arm.set_ff_scalar(5, s, orig[1][s])
    got_m = arm.get_ff_scalar(4, 0).value
    got_c = [arm.get_ff_scalar(5, s).value for s in range(3)]
    print(f"      payload 恢复 -> mass={got_m:.3f}, com={[round(v, 4) for v in got_c]} "
          f"{'ok' if abs(got_m - orig[0]) < 1e-6 else '❌ 请手工写回'}")


def do_run(arm, q_goal, speed, tag, outdir, meta_common, capture_s=7.0):
    """跑一段 movej, 同时录两路: 300Hz 固件日志 + 100Hz 状态帧实测流。"""
    rec = []
    ack = arm._a                       # SDK 里状态帧的唯一漏斗: movej 的 expect/_arrive
    orig_on_status = ack._on_status    # 都从这里过, 在它上面挂探头 = 不丢帧、不开第二个读口

    def spy(payload):
        orig_on_status(payload)        # 先让 SDK 照常解析 (到位判定依赖它)
        rec.append((time.monotonic(), payload))

    ack._on_status = spy
    t_wall0 = time.monotonic()
    moved = err = None
    try:
        arm.log.start(LOG_TICKS)
        t_wall0 = time.monotonic()
        try:
            arm.movej(q_goal, speed=speed)
        except pa.LiteArmError as e:   # 数据是宝贵的: 出错也要把已录的读回来
            err = f"{type(e).__name__}: {e}"
        moved = time.monotonic() - t_wall0
        # movej 判到位就返回了, 而参考运动还在跑尾段 -> 自己把剩下的帧补满
        capture_status_stream(arm, t_wall0 + capture_s, rec)
    finally:
        ack._on_status = orig_on_status

    status_path = os.path.join(outdir, f"movej_vel_trace_{tag}_status.csv")
    sstat = write_status_csv(rec, status_path, arm.n) if rec else {}

    reader = arm.log.reader()
    reader.wait_for(LOG_TICKS, timeout=LOG_TICKS / CTRL_HZ * 1.5 + 5.0)
    samples = pa.parse_samples(reader.read_all(), arm.n)

    # ---- 写出 CSV ----
    n = arm.n
    path = os.path.join(outdir, f"movej_vel_trace_{tag}.csv")
    tick0 = samples[0].tick
    with open(path, "w") as f:
        cols = ["tick", "t"] + [f"q{i}" for i in range(n)] + \
               [f"dq{i}" for i in range(n)] + [f"tau{i}" for i in range(n)]
        f.write(",".join(cols) + "\n")
        for s in samples:
            f.write("%d,%.6f," % (s.tick, (s.tick - tick0) / CTRL_HZ)
                    + ",".join("%.9g" % v for v in s.q_ref) + ","
                    + ",".join("%.9g" % v for v in s.dq) + ","
                    + ",".join("%.9g" % v for v in s.tau) + "\n")

    # ---- 体检: 有没有把运动截断在缓冲区里 ----
    dq_ref = []
    for a_, b_ in zip(samples, samples[1:]):
        dq_ref.append(max(abs(b_.q_ref[i] - a_.q_ref[i]) * CTRL_HZ for i in range(n)))
    last_move = max((i for i, v in enumerate(dq_ref) if v > 0.01), default=0)
    span = (samples[-1].tick - tick0) / CTRL_HZ
    stats = {
        "tag": tag, "csv": path, "n_samples": len(samples),
        "t_span_log_s": round(span, 3),
        "t_wall_movej_s": round(moved, 3) if moved else None,
        "t_ref_motion_s": round((last_move + 1) / CTRL_HZ, 3),
        "truncated": bool(last_move >= len(samples) - 3),
        "movej_error": err,
        "status_csv": status_path if sstat else None,
        **sstat,
    }
    print(f"\n  [{tag}] 300Hz 日志 {len(samples)} 拍 / {span:.2f}s"
          f"; movej 墙钟 {stats['t_wall_movej_s']}s; 参考运动 {stats['t_ref_motion_s']}s")
    if sstat:
        print(f"          100Hz 实测流 {sstat['n_status']} 帧 / "
              f"{sstat['status_span_s']:.2f}s ({sstat['rate_hz']} Hz), "
              f"seq {sstat['seq_first']}..{sstat['seq_last']} "
              f"缺 {sstat['seq_missing']} 帧 (最大连丢 {sstat['seq_gap_max']})")
    if stats["truncated"]:
        print("  ⚠ 缓冲区在运动结束前就写满了 —— 这段是截断数据, 请调大 --dur 之外的 "
              "speed 或减小 --dist 后重跑")
    if err:
        print(f"  ⚠ movej 未正常到位: {err}")
    with open(os.path.join(outdir, f"movej_vel_trace_{tag}.meta.json"), "w") as f:
        json.dump({**meta_common, **stats,
                   "q_goal": [float(v) for v in q_goal]}, f,
                  ensure_ascii=False, indent=2)
    return path, stats


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None, help="串口 (默认自动找 1d50:606f)")
    ap.add_argument("--go", action="store_true", help="真正运动 (默认只读打印计划)")
    ap.add_argument("--dist", type=float, default=0.100, help="水平位移 (m), 默认 0.1")
    ap.add_argument("--dur", type=float, default=5.0, help="整段运动目标时长 (s)")
    ap.add_argument("--legs", default=None,
                    help="逐段序列, 每项 `out|back:on|off` (out=去目标位, back=回起点; "
                         "on/off = 是否保留 FF_VELREF)。默认 "
                         "`out:on,back:on,out:off,back:on` (同向 A/B + 重复性)")
    ap.add_argument("--capture-s", type=float, default=7.0,
                    help="状态帧(100Hz 实测流)采集窗长度 (s), 默认 7")
    ap.add_argument("--ff-mask", default=None,
                    help="本次运行用的 ff_mask (如 0x19F = 清 FF_FRICTION); "
                         "跑完 finally 恢复原值并回读校验。用于前馈逐位消融")
    ap.add_argument("--payload", default=None,
                    help="临时写末端负载, 例 `1.0:0.03,0,0` = 1kg, com 在 ee 法兰系 (m)。"
                         "跑完 finally 恢复并回读校验")
    ap.add_argument("--kd", default=None,
                    help="临时改写若干关节的 mit_kd, 例 `6=1.25,7=1.25` (关节号 1 基)。"
                         "跑完 finally 恢复并回读校验。用于拧速度环增益")
    ap.add_argument("--outdir", default=".", help="CSV 输出目录")
    args = ap.parse_args()

    arm = make_arm(args)
    orig_mask = arm.get_ff_mask()
    velref_on = bool(orig_mask & 0x100)
    orig_kd = {}
    orig_payload = None
    try:
        st = arm.get_state().value
        print(f"固件 {arm.firmware} (n={arm.n})  mode={st.mode_name} "
              f"flags=0x{st.flags:X} enabled={st.enabled}")
        if st.faulted:
            sys.exit(f"⛔ 臂有故障位, 拒绝运动: {st.fault_detail}")
        print(f"ff_mask = 0x{orig_mask:03X} (FF_VELREF {'ON -> dq 列会恒 0' if velref_on else 'off'})")
        if not st.enabled:
            print("臂当前未使能; 本脚本要求已使能持位, 先 enable()")
            if args.go:
                arm.enable()
            else:
                print("(只读模式: 不改状态)")

        q_start = list(arm.get_state().value.q)
        tcp = list(arm.get_tcp().value)
        name, q_target, margin, deltas, tcp_target = pick_target(
            arm, q_start, tcp, args.dist)
        speed, _ = solve_speed(deltas, args.dur)
        times = predict_times(deltas, speed)

        print(f"\n[计划] 方向 {name}  位移 {args.dist * 1000:.0f}mm")
        print(f"  tcp {[round(v, 4) for v in tcp[:3]]} -> "
              f"{[round(v, 4) for v in tcp_target[:3]]} (姿态不变)")
        print(f"  q_start  = {[round(v, 4) for v in q_start]}")
        print(f"  q_target = {[round(v, 4) for v in q_target]}")
        print(f"  Δq       = {[round(v, 4) for v in deltas]}  (max {max(abs(v) for v in deltas):.3f} rad)")
        print(f"  speed 倍率 k = {speed:.4f}  -> 预测各轴时长 "
              f"{[round(t, 2) for t in times]} (最慢 {max(times):.2f}s, 目标 {args.dur}s)")
        if max(times) > LOG_TICKS / CTRL_HZ * 0.9:
            print(f"  ⚠ 预测时长逼近缓冲区上限 {LOG_TICKS / CTRL_HZ:.1f}s, 会自动体检截断")
        print(f"  终点最小限位余量 {margin:.3f} rad (门槛 {MIN_MARGIN})")

        if not args.go:
            print("\n只读模式: 加 --go 才真正运动 (不改任何参数, 不动臂)")
            return

        os.makedirs(args.outdir, exist_ok=True)
        meta = {"firmware": arm.firmware, "n": arm.n,
                "direction": name, "dist_m": args.dist, "speed_mult": speed,
                "q_start": q_start, "tcp_start": tcp,
                "ff_mask_original": "0x%03X" % orig_mask,
                "limits_defaults_used": AXIS_LIMITS}

        back = []
        base_mask = orig_mask
        if args.ff_mask is not None:
            base_mask = int(str(args.ff_mask), 0)
            print(f"\n[消融] 本次 ff_mask 0x{orig_mask:03X} -> 0x{base_mask:03X} "
                  f"(跑完 finally 恢复)")
            meta["ff_mask_override"] = "0x%03X" % base_mask
        if args.payload:
            print(f"\n[写负载] {args.payload} (让固件前馈知道配重; 跑完 finally 恢复)")
            orig_payload = apply_payload(arm, args.payload)
            meta["payload_override"] = args.payload
        if args.kd:
            print(f"\n[拧 kd] {args.kd} (拧速度环增益; 跑完 finally 恢复)")
            orig_kd = apply_kd(arm, args.kd)
            meta["kd_override"] = args.kd
        # 每段**显式声明自己的 ff_mask** (0x27 在使能态可写, 该分支无 ctrl_is_armed
        # 门控 -> 不必 disable; 先 disable 会让臂因自重坠回)。默认四段 = 同向 A/B 对照
        # (out:on vs out:off, 消除"方向"混淆) + 两次完全相同的回程 (重复性证据)。
        spec = args.legs or "out:on,back:on,out:off,back:on"
        legs = []
        for k, tok in enumerate(x.strip() for x in spec.split(",") if x.strip()):
            goal_s, _, mask_s = tok.partition(":")
            if goal_s not in ("out", "back") or mask_s not in ("on", "off"):
                sys.exit(f"⛔ --legs 项 {tok!r} 非法 (应是 out|back : on|off)")
            goal = q_target if goal_s == "out" else q_start
            mask = base_mask if mask_s == "on" else (base_mask & ~0x100)
            legs.append((f"{k + 1}{goal_s}_velref_{mask_s}", goal, mask))

        cur_mask = orig_mask
        for tag, goal, mask in legs:
            if mask != cur_mask:
                arm.set_ff_mask(mask)
                got = arm.get_ff_mask()
                print(f"\n[切档] ff_mask 0x{cur_mask:03X} -> 0x{mask:03X}  回读 0x{got:03X} "
                      f"{'ok' if got == mask else '❌ 不符!'}")
                if got != mask:
                    sys.exit(f"⛔ ff_mask 没写进去 (要 0x{mask:03X} 读到 0x{got:03X}), 中止")
                cur_mask = mask
            print(f"\n[段 {tag}] ff_mask=0x{mask:03X}  -> q {[round(v, 3) for v in goal]}")
            _, s = do_run(arm, goal, speed, tag, args.outdir, meta, args.capture_s)
            back.append(s)

        print("\n[终态]")
        st = arm.get_state().value
        print(f"  mode={st.mode_name} flags=0x{st.flags:X} 断轴={st.fault_axes}")
        print(f"  q   = {[round(v, 4) for v in st.q]}")
        print(f"  tau = {[round(v, 3) for v in st.tau]}")
        for s in back:
            print(f"  {s['tag']}: {s['csv']}")
            if s.get("status_csv"):
                print(f"  {s['tag']} 实测流: {s['status_csv']}")
    finally:
        # 无论成败都恢复原 kd / ff_mask (两者都只写 RAM, 不 save 不会持久化)
        if orig_payload:
            print("\n[恢复负载]")
            restore_payload(arm, orig_payload)
        if orig_kd:
            print("\n[恢复 kd]")
            restore_kd(arm, orig_kd)
        try:
            arm.set_ff_mask(orig_mask)
            got = arm.get_ff_mask()
            print(f"\n[恢复] ff_mask -> 0x{got:03X} "
                  f"{'ok' if got == orig_mask else '❌ 恢复失败, 请手工写回 0x%03X' % orig_mask}")
        except Exception as e:            # noqa: BLE001 - 收尾尽力而为, 不掩盖主异常
            print(f"\n[恢复] ⚠ ff_mask 恢复失败: {type(e).__name__}: {e}")
        arm.close()
        print("已断开。⚠ 刻意未 disable: 该位形失能会因自重坠回, 保持使能持位才安全。")


if __name__ == "__main__":
    main()
