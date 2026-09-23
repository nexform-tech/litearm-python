#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 02 · movej 关节轨迹 (单发, 固件 S 曲线自完成 + 静止保持)。

演示:
  arm.enable()
  arm.movej(q_target, speed)   一次下发; 固件规划 S 曲线到位并静止保持,
                                无需 PC 逐帧保活 (B2/B 语义)。
  到位判定: 后端轮询状态, 各轴 |q−target|<容差 且 dq≈0 连续 N 帧。

安全: 需 --go 才 enable+运动。默认目标 = 当前位形小步试探。

运行:
  python3 examples/02_movej.py --go                      # 当前位置 + 小步
  python3 examples/02_movej.py --go 0.1 0 -0.1 ... (7 角) # 显式目标
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import make_arm, parser


def main():
    ap = parser(__doc__)
    ap.add_argument("targets", nargs="*", type=float,
                    help="7 个目标角(rad); 缺省=当前位置小步试探")
    args = ap.parse_args()

    arm = make_arm(args)
    try:
        if not args.go:
            print("只读连接: 加 --go 才会 enable 并运动 (跳过)")
            print("当前 q =", [round(v, 3) for v in arm.get_state().value.q])
            return

        arm.enable()
        cur = arm.get_state().value.q
        if args.targets:
            if len(args.targets) != arm.n:
                sys.exit(f"需 {arm.n} 个关节角")
            target = list(args.targets)
        else:
            target = list(cur)
            target[2] += 0.1          # 默认小步: J3 +0.1 (安全试探)
            print("默认目标 = 当前位置 + 小步:", [round(v, 3) for v in target])
        arm.movej(target, speed=args.speed)
        st = arm.get_state().value
        print("movej 到位  q =", [round(v, 3) for v in st.q])
        print("          tau =", [round(v, 2) for v in st.tau])
    finally:
        arm.disable()
        arm.close()
        print("已 disable 并断开")


if __name__ == "__main__":
    main()
