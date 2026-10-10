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

⚠⚠ **move_p 没有"相邻解跳变"闸门, 别在近奇异位形上用它。** `move_l`/`move_c`/`move_path`
走 `cart_plan.c`, 那里有 `CART_MAX_IK_STEP = 0.35 rad` 一道闸; `move_p` 走
`kin_runner_request_move_p` -> 后台 DLS-IK, **一个判据都没有**。于是在近奇异位形(开机
零位就是)上, 一个 1cm 的目标位姿可以让 IK 选到远支 —— 真机实测 (Litearm1.10.0-7J):
**J1 转 74°、J3 转 52°, 而末端只动了 1cm, 脚本照样报"到位"**。
健康位形下同一个调用是 5° 量级 (实测 dq 0.095 rad / 20mm), 所以这不是"move_p 不能用",
是"**别从奇异位形发**"。本样例因此**先离开奇异位形**再发目标。

安全: 需 --go; 先把臂送到一个明显弯折的位形, 再在当前 TCP 上沿 +X 平移 1cm。

运行:
  python3 examples/03_move_p.py --go
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import make_arm, parse_args

#: 发笛卡尔命令前的安全起点 —— 明显弯折, 远离零位(伸直)那个奇异位形。
#: 与 README 的 Quick start 同一处置 (那里也是先 `movej` 弯一个关节再发笛卡尔)。
AWAY_FROM_SINGULAR = [0.0, -0.5, 0.0, -1.0, 0.0, 0.0, 0.0]


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
        arm.movej(AWAY_FROM_SINGULAR, speed=0.3)   # 先离开奇异位形
        tcp = arm.get_tcp().value
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
