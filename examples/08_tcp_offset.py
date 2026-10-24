#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 08 · 工具 TCP 偏移标定 (多点法 + 最小二乘, 纯 math, 不引 numpy)。

先说清楚哪一层在做什么: 固件侧 TCP 是编译期定死的模型 (该机 URDF 的 `ee_link`, 原点
在法兰外端面), `get_tcp()` 就是它的 FK 输出, `ik(pose)` 按同一模型反解, 下行命令
0x01-0x49 里**没有**任何工具坐标系命令。`Arm.set_tcp_offset()` 是**宿主侧**补的那一层
(见 docs/DEVELOPER_GUIDE.md §5.4): 设上之后 `move_p`/`move_l`/`move_c`/`move_path`/`ik`
的位姿入参按工具位姿解释、内部换算成法兰再下发, `get_tcp()` 仍返回法兰真值,
`get_tool_tcp()` 给工具位姿。
本样例负责把 xyz **量出来**并调一次 `set_tcp_offset()`; rpy 由装配决定 (该机夹爪是绕
法兰 z 转 90° 装的), 用 `--rpy` 传进来, 默认 0 0 0。

顺带拆掉一个最常见的混淆: `set_payload(mass, com)` (样例 05) **不是** TCP 标定 ——
它是重力前馈用的点质量, 改它不会让 `get_tcp()` 少 6 厘米。判据: 末端差几厘米 = TCP
问题; 挂上工具后下沉 / 发软 / 超调 = payload 问题。

方法 (为什么做差分, 为什么要多点)
  工具点是刚性挂在法兰上的一个点:      p_fix = p_ee + R_ee · t     (t 在**法兰系**)
  让同一个物理点在不同姿态下都顶在同一个位置, 每换一个姿态得到一组 (p_ee, R_ee)。
  相邻样本相减, 把未知的参考点 p_fix 消掉:
        p_i − p_0 = (R_0 − R_i) · t
  ⇒ 3(n−1) 行、3 未知的线性最小二乘。3 个姿态就能定解, 但要 4-6 个才验得出残差。
  ⚠ 好处是不必事先知道参考点在哪 —— 差分把它消掉了。

怎么用 (臂**不必上力**, 能用手拖动即可)
  1. 把工具的工作点 (顶尖 / 球头中心) 顶在一个固定的参考点上 (尖角、锥孔、探针台);
  2. 保持尖端不动, 换一个明显不同的姿态;
  3. 回车采一帧。至少 3 个姿态, 建议 4-6 个, 然后输 `s` 解算。
  本样例**不发任何运动命令, 也不 enable** —— 只读 `get_tcp()`。臂已上力 (enable 过)
  拖不动的话, 先 `arm.disable()`。

四条硬要求 (每一条都能把精度吃掉)
  * **用 `get_tcp()` 的实测值**, 不要用你下发的目标位姿。到位容差 `q_tol=0.03 rad`
    折到法兰上是几毫米, 拿目标值当样本等于把标定精度直接扔掉;
  * **别在 `home` (全伸展) 位形取样**。那是奇异点: 实测关节动 13.4° 末端才走 1.12 mm
    (见 TROUBLESHOOTING "不要把奇异起点读成'总被拒'") ⇒ 该位形附近姿态之间的线性关系
    会坏掉;
  * **姿态要真岔开**, 相邻夹角 ≥15-20°。岔不开时正规方程病态, 本样例用主元比
    (min|pivot| / max|pivot|) 直接告警 —— 别忽略那行提示;
  * **残差 > 1 mm 就重采**, 不要调参。那说明尖端滑了, 或者采的时候臂还在动。

标完之后: `arm.set_tcp_offset(xyz, rpy)` 一行接管两个方向 —— 之后 `move_p` / `move_l` /
`move_c` / `move_path` / `ik()` 的位姿入参按**工具位姿**解释 (含 `move_c` 的 `start`
校验与 `move_p` 的到位判据), `get_tcp()` 仍返回法兰真值, 工具位姿用 `get_tool_tcp()`。
换算公式 (自己实现时才需要)::

    工具 → 法兰 (下发前):  R_ee = R_tool · R_offᵀ ,  p_ee = p_tool − R_ee · xyz
    法兰 → 工具 (读回后):  p_tool = p_ee + R_ee · xyz ,  R_tool = R_ee · R_off

⚠ 设了偏移之后**别再手工换算一遍** —— 两头都换会把偏移算两遍, 目标整体跑偏 2×|xyz|。

运行:
  python3 examples/08_tcp_offset.py                       # 离线自检 (合成数据, 不碰硬件)
  python3 examples/08_tcp_offset.py --go                  # 交互采点
  python3 examples/08_tcp_offset.py --go --rpy 0 0 1.5707963
  python3 examples/08_tcp_offset.py --go --save tcp_offset.json

`--go` 在本样例里只表示"开始交互采点" (不上力、不发运动命令)。
"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import make_arm, parser

import litearm as pa

#: 单次采样里连读三帧, 位置抖动超过它就判"臂还在动", 要求重采。
SAMPLE_POS_SPREAD = 0.0005                  # m (0.5 mm)
SAMPLE_ROT_SPREAD = math.radians(0.3)

#: 3 个姿态就能定解 (3 未知), 但 3 个样本的残差恒为 0 —— 至少要 4 个才验得出来。
MIN_SAMPLES = 3
RECOMMENDED_SAMPLES = 4
MAX_SAMPLES = 12

#: 相邻样本的姿态夹角下限。低于它正规方程病态, 解出来的 t 没有意义。
MIN_ROT_STEP_DEG = 10.0

#: 主元比低于它就判病态 (经验值: 姿态岔开 20° 以上时通常 >1e-2)。
ILL_CONDITIONED = 1e-3


def mat_mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mat_t(A):
    return [[A[j][i] for j in range(3)] for i in range(3)]


def mat_vec(A, v):
    return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]


def mat_sub(A, B):
    return [[A[i][j] - B[i][j] for j in range(3)] for i in range(3)]


def rot_angle(Ra, Rb):
    """两个旋转之间的夹角 (rad) —— trace 公式, 不需要 matrix log。"""
    D = mat_mul(mat_t(Ra), Rb)
    c = (D[0][0] + D[1][1] + D[2][2] - 1.0) / 2.0
    return math.acos(max(-1.0, min(1.0, c)))


def solve3(M, v):
    """3×3 线性方程组 —— 部分主元高斯消元。

    返回 `(解, 主元比)`。主元比 = min|主元| / max|主元|: 姿态没岔开时它塌向 0,
    那时解出来的 t 是噪声, 不是标定值 (主元比 < `ILL_CONDITIONED` 就告警)。
    """
    a = [list(M[i]) + [v[i]] for i in range(3)]
    pivots = []
    for col in range(3):
        p = max(range(col, 3), key=lambda r: abs(a[r][col]))
        if a[p][col] == 0.0:
            return None, 0.0
        a[col], a[p] = a[p], a[col]
        pivots.append(abs(a[col][col]))
        for r in range(col + 1, 3):
            f = a[r][col] / a[col][col]
            for c in range(col, 4):
                a[r][c] -= f * a[col][c]
    x = [0.0, 0.0, 0.0]
    for i in (2, 1, 0):
        x[i] = (a[i][3] - sum(a[i][j] * x[j] for j in range(i + 1, 3))) / a[i][i]
    return x, min(pivots) / max(pivots)


def solve_tool_offset(samples):
    """`[(p_ee, R_ee), ...]` → `(t, 残差列表_mm, 主元比, 姿态跨度_deg)`。

    解不出来 (姿态完全没岔开) 时 `t` 为 `None`。
    """
    p0, R0 = samples[0]
    M = [[0.0] * 3 for _ in range(3)]       # 正规方程 M·t = v
    v = [0.0] * 3
    for p_i, R_i in samples[1:]:
        dR = mat_sub(R0, R_i)               # p_i − p_0 = (R_0 − R_i)·t
        db = [p_i[k] - p0[k] for k in range(3)]
        dRt = mat_t(dR)
        dM = mat_mul(dRt, dR)               # 正规方程: (Σ dRᵀdR)·t = Σ dRᵀdb
        dv = mat_vec(dRt, db)
        for r in range(3):
            for c in range(3):
                M[r][c] += dM[r][c]
            v[r] += dv[r]

    t, ratio = solve3(M, v)
    if t is None:
        return None, [], 0.0, 0.0

    # 回代: 每个样本各自算一遍参考点, 它们之间的离散度就是标定残差。
    fixes = []
    for p, R in samples:
        Rt = mat_vec(R, t)
        fixes.append([p[i] + Rt[i] for i in range(3)])
    centre = [sum(f[i] for f in fixes) / len(fixes) for i in range(3)]
    residuals_mm = [1000.0 * math.sqrt(sum((f[i] - centre[i]) ** 2 for i in range(3)))
                    for f in fixes]
    span_deg = math.degrees(max(rot_angle(R0, R) for _, R in samples[1:]))
    return t, residuals_mm, ratio, span_deg


def self_check():
    """离线自检: 造一组带已知 t 的样本, 反解回来必须一致 —— 不碰硬件。"""
    t_true = [0.004, -0.012, 0.0641]        # 约等于该机夹爪的挂载偏移
    rpys = [(0.0, 0.0, 0.0), (0.35, -0.2, 0.1), (-0.4, 0.35, -0.25), (0.5, 0.15, 0.9)]
    p_fix = [0.35, 0.02, 0.42]
    samples = []
    for rpy in rpys:
        R = pa.rpy_to_mat(rpy)
        Rt = mat_vec(R, t_true)             # p_fix = p_ee + R·t ⇒ p_ee = p_fix − R·t
        samples.append(([p_fix[i] - Rt[i] for i in range(3)], R))

    t, residuals_mm, ratio, span_deg = solve_tool_offset(samples)
    print("离线自检 (合成数据, 已知真值):")
    print(f"  真值 t = {t_true}")
    print(f"  反解 t = {[round(x, 9) for x in t] if t else None}")
    print(f"  主元比 = {ratio:.2e}  姿态跨度 = {span_deg:.1f}°  "
          f"残差 max = {max(residuals_mm):.9f} mm")
    if t is None or max(abs(t[i] - t_true[i]) for i in range(3)) > 1e-9:
        raise AssertionError(f"自检失败: 反解 {t} 与真值 {t_true} 不符")
    print("  自检通过 (误差 < 1e-9 m)")


def read_stable_tcp(arm):
    """连读三帧确认臂已停稳, 返回 `(p_ee, R_ee)`; 还在动或取不到帧返回 `None`。"""
    reads = []
    for _ in range(3):
        val = arm.get_tcp().value
        if val is None:
            return None
        reads.append(list(val))
        time.sleep(0.03)

    spread = max(max(r[k] for r in reads) - min(r[k] for r in reads) for k in range(3))
    if spread > SAMPLE_POS_SPREAD:
        return None
    rots = [pa.rpy_to_mat(r[3:6]) for r in reads]
    if max(rot_angle(rots[-1], R) for R in rots) > SAMPLE_ROT_SPREAD:
        return None
    # 三帧已在阈值内, 取最后一帧的 (p, R) 保证是同一时刻的一组读数。
    return reads[-1][:3], rots[-1]


def collect(arm):
    """交互采点。返回样本列表; 用户放弃时返回 `None`。"""
    print("把工具工作点顶在固定参考点上, 保持尖端不动, 换姿态后回车采样。")
    print("  ⏎ = 采一帧    s = 采完, 开始解算    q = 放弃")
    samples = []
    while True:
        if len(samples) < MIN_SAMPLES:
            tip = f"[{len(samples)} 个样本, 还差 {MIN_SAMPLES - len(samples)} 个]"
        else:
            tip = f"[{len(samples)} 个样本, 再采可选 (建议 ≥{RECOMMENDED_SAMPLES})]"
        try:
            line = input(f"{tip} 摆好回车采样: ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            return None
        if line in ("q", "quit", "exit"):
            return None
        if line in ("s", "done"):
            if len(samples) < MIN_SAMPLES:
                print(f"  还差 {MIN_SAMPLES - len(samples)} 个, 现在解不出来")
                continue
            return samples

        got = read_stable_tcp(arm)
        if got is None:
            print("  臂还在动 (或取不到帧) —— 停稳再采")
            continue
        p, R = got
        if samples and math.degrees(rot_angle(samples[-1][1], R)) < MIN_ROT_STEP_DEG:
            print(f"  和上一个姿态太像 (<{MIN_ROT_STEP_DEG:.0f}°), 换个大点的姿态再来")
            continue
        samples.append(got)
        print(f"  采到 #{len(samples)}: xyz={[round(x, 4) for x in p]} "
              f"rpy={[round(x, 4) for x in pa.mat_to_rpy(R)]}")
        if len(samples) >= MAX_SAMPLES:
            print(f"  到 {MAX_SAMPLES} 个上限, 开始解算")
            return samples


def main():
    ap = parser(__doc__, go_help="开始交互采点 (只读 get_tcp(), 不上力、不发运动)")
    ap.add_argument("--rpy", nargs=3, type=float, default=[0.0, 0.0, 0.0],
                    metavar=("R", "P", "Y"),
                    help="工具相对法兰的姿态偏移 (rad), 由装配/CAD 决定")
    ap.add_argument("--save", default=None, metavar="PATH",
                    help="把结果写进本地配置 json (走 Arm.save_tool, 与 SDK 同一格式)")
    args = ap.parse_args()

    if not args.go:
        self_check()
        print("\n以上是离线自检 (合成数据)。真标定加 --go 交互采点。")
        return

    arm = make_arm(args)
    try:
        tcp = arm.get_tcp().value
        print("当前 TCP:", [round(v, 4) for v in tcp] if tcp else None)
        st = arm.get_state(refresh=True, timeout=0.5).value
        if st is not None and st.enabled:
            print("enabled=True —— 采样要能用手拖动, 请先 arm.disable()")

        samples = collect(arm)
        if samples is None:
            print("已放弃; 没有改动任何东西 (偏移只在宿主机侧, 本样例不写参数)")
            return

        t, residuals_mm, ratio, span_deg = solve_tool_offset(samples)
        if t is None:
            print("解不出来: 姿态完全没岔开 (正规方程奇异)。换几个差别更大的姿态重采。")
            return

        rmax, rmean = max(residuals_mm), sum(residuals_mm) / len(residuals_mm)
        print("\n=== 标定结果 ===")
        print(f"样本数 {len(samples)}   姿态跨度 (相对样本 0 的最大夹角) {span_deg:.1f}°")
        print(f"主元比 {ratio:.2e}"
              + ("   ⚠ 偏小: 姿态没岔开, 结果不可信, 重采" if ratio < ILL_CONDITIONED else ""))
        print(f"t (工具点在**法兰系**) = [{t[0]:+.6f}, {t[1]:+.6f}, {t[2]:+.6f}] m"
              f"  ({[round(v * 1000, 1) for v in t]} mm)")
        print(f"残差 (各样本回代的参考点离散度): max {rmax:.3f} mm, mean {rmean:.3f} mm")
        if rmax > 1.0:
            print("  ⚠ >1 mm: 尖端滑了 / 采时还在动 / 姿态读数有误 —— 重采, 不要调参")

        print(f"\n完整偏移 (法兰系): xyz={[round(x, 6) for x in t]}  "
              f"rpy={[round(x, 6) for x in args.rpy]}")
        print("  ⚠ rpy 来自装配/CAD, 本样例不标它 —— 换工具必须重新给")

        # 一行接管两个方向 (只改本进程内存, 不发任何固件命令)。
        arm.set_tcp_offset(t, args.rpy)
        print(f"已设 set_tcp_offset -> {arm.tcp_offset}")
        print(f"  法兰: {[round(v, 4) for v in arm.get_tcp().value]}")
        print(f"  工具: {[round(v, 4) for v in arm.get_tool_tcp().value]}")

        if args.save:
            print(f"\n已写 {arm.save_tool(args.save)}")
        else:
            print(f"\n要跨会话保留: arm.save_tool()  (默认 {arm.tool_config_path()})")
        print("⚠ 工具的**重量**是另一件事: set_payload(mass, com) —— 见样例 05")
    finally:
        arm.close()


if __name__ == "__main__":
    main()
