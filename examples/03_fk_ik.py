#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 03 · 正逆运动学 fk / ik（远程客户端，纯计算不运动）

演示：
  arm.fk(q)                 正运动学：关节角 -> (末端位置, 旋转矩阵)
  arm.ik(pos, R, q_seed)    逆运动学：(位置, 旋转) -> (关节角, 是否成功)

这两个是纯计算 RPC，server 端不驱动电机，安全无副作用。
常用来在下发 movej 前先验证目标位姿可达。

运行：
  python3 examples/03_fk_ik.py
"""
import time

from _common import N, make_arm, parse_args


def main():
    args = parse_args(__doc__)
    arm = make_arm(args)

    try:
        # 用当前关节角做 fk，拿到当前末端位姿
        time.sleep(0.8)
        state = arm.get_state()
        q_now = state["q"] if state else [0.0] * N
        print("\n[当前关节角]", [round(x, 3) for x in q_now])

        pos, R = arm.fk(q_now)
        print("[fk] 末端位置 =", [round(x, 4) for x in pos], "m")

        # 用 fk 得到的位姿反解 ik，应能解回接近原关节角
        q_sol, ok = arm.ik(pos, R, q_seed=q_now)
        print("\n[ik] 成功 =", ok)
        print("[ik] 解出关节角 =", [round(x, 3) for x in q_sol])

        # 对比 ik 解与原关节角的最大偏差
        max_err = max(abs(a - b) for a, b in zip(q_sol, q_now))
        print(f"[对比] 与原关节角最大偏差 = {max_err:.4f} rad")
    finally:
        arm.close()
        print("\n[Arm] 已断开")


if __name__ == "__main__":
    main()
