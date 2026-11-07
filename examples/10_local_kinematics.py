#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 10 · SDK 本地 FK/IK/Jacobian 与安全位姿 PTP。

默认只读：读取当前 q，在 SDK 本地计算 FK、Jacobian 和同位姿 IK，不发送固件 IK。
加 --go 后，把本地工具位姿沿世界 x 方向移动 --dx（默认 5 mm），完整预检通过后只发一条
CMD_MOVE_J_SYNC；STM32 负责同步 S 曲线、速度、加速度和 jerk 限制。

本地预检只覆盖软限位、奇异点和机械臂自碰，不检测工作台、人员或外部障碍物。

运行：
  python3 examples/10_local_kinematics.py
  python3 examples/10_local_kinematics.py --go --speed 0.2 --dx 0.005
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import litearm as pa

from _common import make_arm, parser


def _wait_enabled(arm, timeout=1.0):
    """Wait for a fresh status frame to confirm enable; do not use a fixed sleep."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = arm.get_state(refresh=True).value
        if state is not None:
            if state.faulted:
                raise pa.MotorFaultError(
                    f"使能后检测到关节故障: {state.fault_detail}")
            if state.enabled:
                return state
        time.sleep(0.01)
    raise pa.LiteArmError(f"{timeout}s 内状态帧没有确认使能，请检查供电、故障和固件状态")


def main():
    ap = parser(__doc__)
    ap.add_argument("--dx", type=float, default=0.005,
                    help="--go 时工具目标沿世界 x 的位移（m，默认 0.005）")
    args = ap.parse_args()

    arm = make_arm(args)
    try:
        state = arm.get_state(refresh=True).value
        q = state.q
        pose = arm.kinematics.fk(q, frame="tool")
        jac = arm.kinematics.jacobian(q, frame="tool")
        solution = arm.kinematics.ik(pose, q_seed=q, frame="tool")
        print("当前 q       =", [round(v, 4) for v in q])
        print("本地工具 FK  =", [round(v, 5) for v in pose])
        print(f"sigma_min    = {jac.sigma_min:.6f}")
        print(f"同位姿 IK Δq = {solution.max_joint_delta:.6g} rad")

        if not args.go:
            print("只读完成；加 --go 才会使能并执行安全 PTP")
            return

        target = list(pose)
        target[0] += args.dx
        arm.enable()
        _wait_enabled(arm)
        reached = arm.kinematics.move_pose(target, speed=args.speed, frame="tool")
        print("安全 PTP 到位 q =", [round(v, 4) for v in reached.q])
        actual = arm.kinematics.fk(reached.q, frame="tool")
        print("实测关节状态对应的工具 FK =", [round(v, 5) for v in actual])
    finally:
        if args.go:
            arm.disable()
        arm.close()


if __name__ == "__main__":
    main()
