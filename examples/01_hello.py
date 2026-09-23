#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 01 · 连接握手 + 只读状态/末端位姿 (安全, 不需要 --go)。

演示:
  Arm().connect()      自动找 CDC 并校验固件版本约定 (Litearm≥1.5.0)
  get_firmware/版本    固件版本串 + n(关节数)
  get_state()          状态帧: q/dq/tau/温度/err/flags/mode
  get_tcp()            当前末端 pos[3]+rpy[3] (固件 FK, 无运动学模型)

运行:
  python3 examples/01_hello.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import make_arm, parse_args


def main():
    args = parse_args(__doc__)
    arm = make_arm(args)
    try:
        print(f"固件: {arm.firmware}  (n={arm.n} 关节)")
        st = arm.get_state().value
        print("\n[状态] mode=", st.mode_name, "flags=", st.flag_names or "-")
        print("  q   =", [round(v, 3) for v in st.q])
        print("  dq  =", [round(v, 3) for v in st.dq])
        print("  tau =", [round(v, 3) for v in st.tau])
        if st.faulted:
            print("  ⚠️  检测到 FAULT/EMERGENCY —— 先排查再运动!")
        tcp = arm.get_tcp().value
        print("\n[末端] pos =", [round(v, 4) for v in tcp[:3]],
              " rpy =", [round(v, 3) for v in tcp[3:6]])
    finally:
        arm.close()
        print("\n已断开 (只读, 未改电机状态)")


if __name__ == "__main__":
    main()
