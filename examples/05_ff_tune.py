#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 05 · 内置动力学/控制律调参 (固件 B1+B4; RAM 生效, 需 save 才持久)。

演示 (参考 pylitearm set_friction/set_integral/set_dynamics_scale/set_payload):
  arm.ff_preset(1)             出厂组合: master+G+惯量+科氏+摩擦+积分 (开箱)
  arm.set_gravity_scale(gs)    重力逐关节缩放 (承重 J2/J3 需 2~4.5x)
  arm.set_inertia_scale(is)    惯量逐关节缩放
  arm.set_gravity_vector(g)    换安装/倒装改重力方向
  arm.set_payload(mass, com)   末端负载 (点质量, com 在 ee 法兰系)
  arm.save_params()            持久化到 Flash (0x25)

安全: 需 --go 才真正下发; 调参后建议 movej 小步验证手感想硬还是软。

运行:
  python3 examples/05_ff_tune.py                 # 只打印将发的内容
  python3 examples/05_ff_tune.py --go            # 下发出厂 preset + 显示性 set
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import make_arm, parse_args

GS = [2.7, 2.1, 1.75, 1.85, 2.9, 3.1, 1.2]        # pylitearm litearm.yaml 出厂
IS = [2.84, 8.62, 9.5, 2.56, 5.27, 4.02, 3.56]


def main():
    args = parse_args(__doc__)
    arm = make_arm(args)
    try:
        print("将演示的固件 FF 命令 (0x26-0x28/0x31):")
        print("  ff_preset(1)                -> master+G+惯量+科氏+摩擦+积分")
        print(f"  set_gravity_scale(GS={GS}) -> 重力缩放")
        print(f"  set_inertia_scale(IS={IS}) -> 惯量缩放")
        print("  set_gravity_vector((0,0,-9.81)) / set_payload(...) / save_params()")
        if not args.go:
            print("只读: 加 --go 才下发 (不改固件参数)")
            return
        arm.ff_preset(1)
        arm.set_gravity_scale(GS)
        arm.set_inertia_scale(IS)
        arm.set_gravity_vector([0.0, 0.0, -9.81])
        # 负载演示: 先清空(默认出厂无负载), 注释掉即不清
        # arm.set_payload(0.0, (0, 0, 0))
        arm.save_params()
        print("已下发出厂 preset + gs/is/gravity 并保存到 Flash")
    finally:
        arm.close()
        print("已断开")


if __name__ == "__main__":
    main()
