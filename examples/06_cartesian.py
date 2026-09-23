#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 06 · 笛卡尔路径 (固件规划): move_l / move_c / move_path。

与 03 的区别: 03 的 `move_p(单 pose)` 是固件做**关节空间**插值 (不是直线);
本样例这三条把**末端路径**交给固件规划 —— 末端真的沿直线/圆弧/折线走。

三件事, 对应三个 API:

  arm.move_l(goal, speed)                末端走**直线** (位置线性 + 姿态 slerp)
  arm.move_c(start, via, goal, speed)    末端走**圆弧** (三点定圆; via 的姿态被忽略)
  arm.move_path([p1, p2, ...], speed)    依次经过多个位姿 (**尖角**)

位姿两种写法都收: 固件的 `[x,y,z,roll,pitch,yaw]` 或 pylitearm 的 `(pos[3], R[3x3])`。
起点不是参数 —— `move_l`/`move_path` 由固件取**当前实测 TCP**, `move_c` 的 `start` 必须
与实测 TCP 一致 (容差 6mm / 0.03rad, 超差抛 `InvalidCommandError` 并给出实际值)。

返回值是 `CartPlan` (dataclass, **没有 `summary()`** —— 从前 PC 侧那条路径返回的
`MotionResult` 才有, 它已随 PC 侧规划一起退役)。字段:
`ok`/`err`/`n_wp`/`plan_us` 是固件给的规划结果; `started_busy`/`settled`/`q_final`/
`settle_err_rad` 只有 `wait=True` (默认) 才填真值, `wait=False` 时是"**没等**", 不是"没到位"。
⚠ `ok=True` 只说明**固件受理并规划出来了**, 不说明臂停在了目标上 (`settled=False` 就是
"轨迹被别的运动作废了") —— 现场要回读 `arm.get_tcp()` 看真实落点。

⚠ 三条已知降级 (相对被它取代的 PC 侧规划, 设计 §5.5):
  * **无拐角倒角**: 协议里没有 blend 字段, 所以多路点过拐角是**尖的**, 也不会有
    "不再精确经过中间点" 那种倒角行为 (固件 planner 支持 blend, 但协议没给它开口)。
  * **无下发前预览**: 固件没有 dry-run, `0x4E` 要发出去才回 —— 圆弧实际扫多大、弧长多少
    只能**事后**从 `n_wp` 判读 (PC 侧预览需要把已退役的规划器请回来, 不做)。
  * **速度预检改由固件做**: 从前 PC 侧按固件 `vel_max` 在发帧**之前**拦一次; 现在路径的
    可接受性由固件在规划阶段判, 入口按 `plan.err` 抛 `CartesianPlanError`
    (`CART_ERR_LIMIT`)。判据只在固件手里一份, PC 侧不再复制。

安全: 需 --go; 默认在当前 TCP 上沿 +X 平移 1cm 再回程 (小步可回退)。

运行:
  python3 examples/06_cartesian.py --go
  python3 examples/06_cartesian.py --go --speed 0.3     # 更慢
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import make_arm, parse_args


def show(label, plan):
    """`CartPlan` 没有 `summary()` —— 自己挑要打的字段。"""
    print(f"  {label}: ok={plan.ok} n_wp={plan.n_wp} plan={plan.plan_us / 1000.0:.1f}ms "
          f"busy={plan.started_busy} settled={plan.settled} "
          f"settle_err={plan.settle_err_rad:.4f}rad")


def main():
    args = parse_args(__doc__)
    arm = make_arm(args)
    try:
        tcp = arm.get_tcp().value
        print("当前 TCP:", [round(v, 4) for v in tcp])
        pos, R = tcp[:3], tcp[3:]

        if not args.go:
            print("只读连接: 加 --go 才会执行 (跳过)")
            return

        arm.enable()

        # ---- move_l: 直线 1cm ----
        goal = [pos[0] + 0.010, pos[1], pos[2]] + list(R)
        print("\n--- move_l 直线 +1cm ---")
        show("move_l", arm.move_l(goal, speed=args.speed))

        # ---- move_c: 小圆弧。⚠ start 必须与**此刻实测** TCP 一致 ----
        print("\n--- move_c 圆弧 ---")
        start = list(arm.get_tcp().value)
        via = [start[0] + 0.008, start[1] + 0.008, start[2]] + list(R)
        end = [start[0], start[1] + 0.016, start[2]] + list(R)
        plan = arm.move_c(start, via, end, speed=args.speed)
        show("move_c", plan)
        print(f"    (没有下发前预览: 扫了多大弧只能从 n_wp={plan.n_wp} 事后判读)")

        # ---- move_path: 两个路点, 尖角 (协议没有倒角字段) ----
        w1 = [start[0] + 0.010, start[1] + 0.005, start[2]] + list(R)
        w2 = [start[0] + 0.010, start[1] + 0.015, start[2]] + list(R)
        print("\n--- move_path 2 路点 (尖角, 无倒角) ---")
        show("move_path", arm.move_path([w1, w2], speed=args.speed))

        # ---- 回起点 ----
        print("\n回起点...")
        show("move_l", arm.move_l(list(tcp), speed=args.speed))
        time.sleep(0.3)
        print("收尾 TCP:", [round(v, 4) for v in arm.get_tcp().value])
    finally:
        arm.close()


if __name__ == "__main__":
    main()
