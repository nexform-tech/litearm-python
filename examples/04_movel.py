#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 04 · 笛卡尔直线运动 movel（远程客户端）

末端沿一条【直线】走到目标位姿（位置线性插值 + 姿态 slerp）：
  arm.movel(pose_goal, speed=...)      pose_goal = [pos[3], R[3x3]]

也演示“先规划后执行”（纯计算 RPC、不动电机，可离线检查路径）：
  q_path = arm.plan_movel(q_start, pose_goal)

⚠️ 会真实运动！笛卡尔运动要求起点远离奇异，故本例先 movej 到 Q_HOME 展开，
再沿 -Y 走 20cm（-Y 工作域深、全程远离奇异）。

位姿格式（纯 list，客户端不依赖 numpy）：
  pose = [position, rotation]
  position = [px, py, pz]
  rotation = [[r00,r01,r02],[r10,r11,r12],[r20,r21,r22]]  (3x3 行主序)

运行：
  python3 examples/04_movel.py
"""
from _common import make_arm, parse_args, Q_HOME

DIST = 0.20   # 直线行程（m）
SPEED = 0.5


def main():
    args = parse_args(__doc__)
    arm = make_arm(args)

    try:
        # ── 先展开到舒展构型，规避奇异 ──
        print(f"\n[准备] movej 到 Q_HOME {Q_HOME}")
        arm.movej(Q_HOME, speed=SPEED, settle_s=0.5)

        # ── 纯规划演示（不动电机）──
        pos0, R0 = arm.fk(Q_HOME)
        goal_pos = [pos0[0], pos0[1] - DIST, pos0[2]]   # 沿 -Y 平移
        goal = [goal_pos, R0]
        q_path = arm.plan_movel(Q_HOME, goal)
        print(f"\n[plan_movel] 沿 -Y {DIST*100:.0f}cm，关节路点数 = {len(q_path)}")
        print("  首点 =", [round(float(v), 3) for v in q_path[0]])
        print("  末点 =", [round(float(v), 3) for v in q_path[-1]])

        # ── 真机执行：从当前末端位姿沿 -Y 走直线 ──
        pos_now, R_now = arm.get_tcp_pose()
        print("\n[起点] pos =", [round(x, 4) for x in pos_now])
        goal = [[pos_now[0], pos_now[1] - DIST, pos_now[2]], R_now]
        print(f"[movel] 沿 -Y 走 {DIST*100:.0f}cm，speed={SPEED}")
        ok = arm.movel(goal, speed=SPEED)
        print("  完成 =", ok)

        pos_end, _ = arm.get_tcp_pose()
        print("[终点] pos =", [round(x, 4) for x in pos_end])
    finally:
        arm.close()
        print("\n[Arm] 已断开")


if __name__ == "__main__":
    main()
