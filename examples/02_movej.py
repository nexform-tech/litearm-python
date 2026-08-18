#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 02 · 关节空间运动 movej（远程客户端）

演示：
  arm.movej(q_target, speed=..., settle_s=...)
      走一条 S 曲线关节轨迹到目标构型。server 端用计算力矩前馈执行。
      speed:    0~1，相对满速的比例
      settle_s: 到位后额外持位保持的秒数

⚠️ 连上的就是真机，会真实运动！首次请把 speed 调到 0.1~0.2，人站在急停旁。

运行：
  python3 examples/02_movej.py
  python3 examples/02_movej.py --endpoint tcp/127.0.0.1:7447
"""
from _common import N, make_arm, parse_args


def main():
    args = parse_args(__doc__)
    arm = make_arm(args)

    speed = 0.5  # 首跑保守；熟悉后再提

    try:
        # 目标 1：一个舒展的“预备”构型。
        home = [0.0, 0.5, 0.0, -1.0, 0.0, 0.6, 0.0]
        print(f"\n[movej] -> home {home}  speed={speed}")
        ok = arm.movej(home, speed=speed, settle_s=0.5)
        print("  完成 =", ok)

        # 目标 2：回零位（全 0）。观察是否平滑无过冲。
        zero = [0.0] * N
        print(f"\n[movej] -> zero {zero}  speed={speed}")
        ok = arm.movej(zero, speed=speed, settle_s=0.5)
        print("  完成 =", ok)
    finally:
        arm.close()
        print("\n[Arm] 已断开（server 端已 park 保持）")


if __name__ == "__main__":
    main()
