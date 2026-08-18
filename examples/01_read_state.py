#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 01 · 连接并读取实时状态（远程客户端）

演示：
  litearm.Arm(endpoint=...)   连接 litearm-server（无需 connect()，server 常连）
  arm.get_state()             从 ~50Hz 状态广播缓存读 {q, dq, tau, fault, state, ...}
  arm.get_tcp_pose()          当前末端位姿（server 端 fk(当前关节角)）
  arm.close()                 断开 zenoh（不影响 server 与机械臂）

状态是 server 广播过来的，get_state() 只读本地缓存、不发 RPC，非常快。

运行：
  python3 examples/01_read_state.py
  python3 examples/01_read_state.py --endpoint tcp/127.0.0.1:7447
"""
import time

from _common import make_arm, parse_args


def main():
    args = parse_args(__doc__)
    arm = make_arm(args)

    try:
        # 等待第一帧状态广播到达（客户端刚连上时缓存可能还空）
        time.sleep(1.0)

        state = arm.get_state()
        if state is None:
            print("\n[警告] 尚未收到状态广播，检查 server 是否在运行、endpoint 是否正确")
            return

        print("\n[状态]")
        print("  q    (关节角 rad) =", [round(x, 3) for x in state["q"]])
        print("  dq   (关节速度)   =", [round(x, 3) for x in state["dq"]])
        print("  tau  (关节力矩)   =", [round(x, 3) for x in state["tau"]])
        print("  state(状态机)     =", state["state"])
        print("  fault(故障电机)   =", state.get("fault", []))

        pos, R = arm.get_tcp_pose()
        print("\n[末端位姿] pos =", [round(x, 4) for x in pos], "m")
    finally:
        arm.close()
        print("\n[Arm] 已断开（server 与机械臂不受影响）")


if __name__ == "__main__":
    main()
