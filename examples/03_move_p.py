#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 03 · move_p 位姿运动 (单 pose, 固件内置 IK + S 曲线)。

演示:
  arm.get_tcp()               读当前末端 pos[3]+rpy[3]
  arm.move_p(pose, speed)     一次下发 pose; 固件后台 DLS-IK 求 q → S 曲线到位
                              后端轮询 get_tcp 至 TCP≈目标 (pos 容差/rpy 容差) 判定到位。

注意: move_p 是【点到点】**关节空间**插值 (非笛卡尔直线), 且**只收单个位姿** ——
要末端走直线/圆弧/多路点见 06_cartesian.py (`move_l`/`move_c`/`move_path`, 固件规划);
多路点传进 move_p 会抛 InvalidCommandError (旧 movep 的序列重载已移除)。

安全: 需 --go; 默认在当前 TCP 上沿 +X 平移 1cm (小步可回退)。

运行:
  python3 examples/03_move_p.py --go
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import make_arm, parse_args


def main():
    args = parse_args(__doc__)
    arm = make_arm(args)
    try:
        tcp = arm.get_tcp().value
        print("当前 TCP:", [round(v, 4) for v in tcp])
        if not args.go:
            print("只读连接: 加 --go 才会执行 move_p (跳过)")
            return
        arm.enable()
        target = list(tcp)
        target[0] += 0.01                      # 世界 +X 1cm 小步
        print("move_p 目标:", [round(v, 4) for v in target])
        arm.move_p(target, speed=args.speed)
        tcp2 = arm.get_tcp().value
        print("move_p 到位  TCP:", [round(v, 4) for v in tcp2])
    finally:
        arm.disable()
        arm.close()
        print("已 disable 并断开")


if __name__ == "__main__":
    main()
