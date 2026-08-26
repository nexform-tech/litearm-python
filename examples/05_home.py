#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 05 · 回零 home（远程客户端）

将所有关节运动到零位（[0, 0, 0, 0, 0, 0, 0]）。

与 movej 不同，home() 在回零时自动绕开限位和自碰路径安全检查，
适用于上电后机械臂处于任意构型需要回到零位的场景：
  - 关节当前位置超出软限位 → 仍然放行
  - 路径规划被判自碰 → 仍然放行

硬件层故障/温度/超速/跟随误差保护仍然生效。默认 speed=0.3（慢速），
回零是恢复性操作，低速更安全。

⚠️ 连上的就是真机，会真实运动！

运行：
  python3 examples/05_home.py
  python3 examples/05_home.py --endpoint tcp/127.0.0.1:7447
"""
from _common import make_arm, parse_args


def main():
    args = parse_args(__doc__)
    arm = make_arm(args)

    try:
        print("\n[home] 回零中... speed=0.3")
        ok = arm.home(speed=0.3, settle_s=0.5)
        print("  完成 =", ok)

        if ok:
            state = arm.get_state(refresh=True)
            if state:
                print("  当前关节角:", [round(v, 6) for v in state["q"]])
    finally:
        arm.close()
        print("\n[Arm] 已断开（server 端已 park 保持）")


if __name__ == "__main__":
    main()