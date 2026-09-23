#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 04 · IK / TCP 查询 (只读, 不需 --go)。

演示:
  arm.get_tcp()         当前末端位姿 (固件当前反馈 FK)
  arm.ik(pose)          pose[6] → q[7] (固件后台 DLS IK, seed=当前 q)
  常见组合: 读当前 pose → 验证 ik(pose) 解回 q 应与当前关节角相近 (自洽)。

运行:
  python3 examples/04_ik_tcp.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import make_arm, parse_args


def main():
    args = parse_args(__doc__)
    arm = make_arm(args)
    try:
        q_cur = arm.get_state().value.q
        tcp = arm.get_tcp().value
        print("当前 q   =", [round(v, 3) for v in q_cur])
        print("当前 TCP =", [round(v, 4) for v in tcp])

        q_ik = arm.ik(tcp)               # 用当前 pose 反解
        print("\nik(当前pose) =", [round(v, 3) for v in q_ik])
        dev = max(abs(q_ik[i] - q_cur[i]) for i in range(7))
        print(f"与当前关节角最大偏差 = {dev:.3f} rad (自洽应 <~0.05)")
    finally:
        arm.close()
        print("\n已断开 (只读)")


if __name__ == "__main__":
    main()
