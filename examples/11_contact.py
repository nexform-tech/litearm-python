#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 11 · 接触保护快速上手 (固件 1.13.0+)。

流程:
  1. 连接, 只读打印当前接触状态 / 生效阈值;
  2. --go: enable, 移动到 litearm-studio 的「就绪姿态」按钮同一目标;
  3. 在就绪姿态上标定 (保持静止约 2 s), 等 phase 回到 ARMED;
  4. 等待 --wait 秒, 期间可用指尖轻推末端或 J6 附近 —— 触发后 phase 变 LATCHED_HOLD;
  5. 自动 contact_clear() 清锁存, 并在原地重新标定回 ARMED。

阈值从哪来 (二选一)。本样例**从不**用固件编译期的通用默认表武装 —— 那套表比真机标定
值灵敏 2.5~6 倍, 未经标定, 不能用于真机:
  A. 本轮上电已用固件 tools/contact_tune.py --execute --apply 标定过 → 直接复用固件
     RAM 里已生效的阈值 (本样例只发 0x17 重新标定, 不重发阈值);
  B. 手头有标定值 → 用 --threshold / --hard-threshold 各传 7 个数 (N·m), 样例发 0x16
     武装并自动标定。

安全:
  * 需要 --go 才会 enable + 运动; 首次运行把 --speed 保持在 0.1~0.3, 人站在急停旁;
  * 轻推即可, 不要用力掰; 超过 hard_threshold 会进入 HARD 锁存;
  * 接触配置只在 RAM, 断电即清。本样例刻意不调用 contact_off()。

运行:
  python3 examples/11_contact.py                    # 只读: 看当前状态/阈值
  python3 examples/11_contact.py --go               # 复用固件已生效阈值并标定
  python3 examples/11_contact.py --go --wait 30     # 留 30 s 时间手推
  python3 examples/11_contact.py --go \
      --threshold     <7 个标定值> \
      --hard-threshold <7 个标定值>                 # 显式武装
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import make_arm, parser

import litearm as pa

#: litearm-studio「就绪姿态」按钮的目标 (7J); 按 arm.n 裁剪/补齐
READY_POSE = [0.0, 0.5, 0.0, -1.0, 0.0, 0.6, 0.0]
ARMED = "ARMED"
WAIT_FOR_ARM = 8.0


def fit_joints(values, n):
    """和 studio 的 fitJoints 一样: 截断或补 0。"""
    return (list(values) + [0.0] * n)[:n]


def wait_armed(arm, timeout=WAIT_FOR_ARM):
    """轮询 0x19 直到 ARMED / 锁存 / 超时, 打印相位变化。"""
    deadline = time.time() + timeout
    last = None
    while True:
        st = arm.get_contact_state().value
        if st["phase_name"] != last:
            print(f"  contact phase → {st['phase_name']}")
            last = st["phase_name"]
        if (st["phase_name"] in (ARMED, "DISABLED") or st["latched"]
                or time.time() >= deadline):
            return st
        time.sleep(0.1)


def ensure_armed(arm):
    """等 ARMED; 若标定释放瞬态触发了一次锁存, clear 后重试一次。"""
    st = wait_armed(arm)
    if st["latched"]:
        print(f"  标定过程触发了锁存 ({st['phase_name']}), 清除后重试一次 ...")
        arm.contact_clear()
        st = wait_armed(arm)
    return st


def peak_text(st):
    peak = st["peak"]
    j = max(range(len(peak)), key=lambda i: peak[i])
    return f"J{j + 1} 峰值 {peak[j]:.2f} N·m"


def main():
    ap = parser(__doc__)
    ap.add_argument("--threshold", nargs=7, type=float, metavar="J",
                    help="标定后的普通阈值 (7 个, N·m)")
    ap.add_argument("--hard-threshold", nargs=7, type=float, metavar="J",
                    help="硬阈值 (7 个, N·m; 每个都必须大于 threshold)")
    ap.add_argument("--wait", type=float, default=15.0,
                    help="标定完成后等待手推的秒数 (默认 15; 0 = 跳过)")
    args = ap.parse_args()
    if (args.threshold is None) != (args.hard_threshold is None):
        sys.exit("--threshold 与 --hard-threshold 必须成对给出")

    arm = make_arm(args)
    ready = fit_joints(READY_POSE, arm.n)
    try:
        try:
            st = arm.get_contact_state().value
            mode = arm.get_contact_mode().value
        except pa.UnsupportedByFirmwareError:
            sys.exit(f"固件 {arm.firmware} 不支持接触保护 (需要 1.13.0+)")

        print(f"固件 {arm.firmware}  (n={arm.n} 关节)")
        print("当前接触状态:", st["phase_name"],
              "| response =", mode["response_name"],
              "| level =", mode["level"])
        print("就绪姿态目标:", [round(v, 2) for v in ready])

        if not args.go:
            print("\n只读连接: 加 --go 才会 enable、移动到就绪姿态并标定 (跳过)")
            print("当前生效阈值:", [round(v, 3) for v in mode["threshold"]])
            return

        if args.threshold is None and st["phase_name"] == "DISABLED":
            sys.exit(
                "固件里没有可用的接触配置 (phase=DISABLED), 本次不运动。\n"
                "可能从未发过 0x16, 或 level=0 关闭了检测。\n"
                "  先用固件 tools/contact_tune.py --execute --apply 标定, 或\n"
                "  用 --threshold/--hard-threshold 显式传 7+7 个标定值。\n"
                "本样例不会用编译期通用默认表武装 —— 它未经标定, 不可用于真机。")

        arm.enable()

        # enable 后若保护已配置, 固件会自动标定并暂时接管持位 —— 等它结束再动,
        # 否则 movej 会被拒绝 (固件 ERR[0x2A,0x06])。
        st = arm.get_contact_state().value
        if st["latched"] or st["phase_name"] in ("CALIBRATING",
                                                 "NEEDS_CALIBRATION"):
            if st["latched"]:
                print("检测处于锁存, 先 contact_clear() ...")
                arm.contact_clear()
            st = ensure_armed(arm)
            if st["phase_name"] == "DISABLED":
                sys.exit("接触保护处于 DISABLED —— 先标定/武装后再运行本样例")
            if st["phase_name"] != ARMED:
                sys.exit(f"未能回到 ARMED (phase={st['phase_name']}) —— 先排查再运行")

        try:
            arm.movej(ready, speed=args.speed)
        except pa.CommandRejectedError as e:
            sys.exit(f"移动到就绪姿态被拒绝: {e}\n"
                     "通常是接触标定仍接管持位或已锁存: 等 2~3 s 重试, "
                     "或先 arm.contact_clear()")
        time.sleep(0.3)                      # 让 S 曲线收尾、反馈归零

        print("已到就绪姿态, 开始标定 (保持静止约 2 s) ...")
        if args.threshold is not None:
            arm.contact_on(threshold=args.threshold,
                           hard_threshold=args.hard_threshold)
        else:
            arm.contact_calibrate()

        st = ensure_armed(arm)
        if st["phase_name"] == "DISABLED":
            sys.exit("检测仍为 DISABLED —— 多半是 level=0; "
                     "先 arm.set_contact_mode(level=6) 再重跑")
        if st["phase_name"] != ARMED:
            sys.exit(f"没有回到 ARMED (phase={st['phase_name']}) —— "
                     "保持静止两三秒后重跑")

        mode = arm.get_contact_mode().value
        print("ARMED。生效阈值:", [round(v, 2) for v in mode["threshold"]])
        print("        硬阈值:", [round(v, 2) for v in mode["hard_threshold"]])

        if args.wait > 0:
            print(f"\n请在 {args.wait:.0f} s 内用指尖轻推末端或 J6 附近 "
                  "(别用力掰) ...")
            deadline = time.time() + args.wait
            last = st["phase_name"]
            while time.time() < deadline:
                st = arm.get_contact_state().value
                if st["phase_name"] != last:
                    print(f"  phase → {st['phase_name']}"
                          f"  mask=0x{st['joint_mask']:04X}  {peak_text(st)}")
                    last = st["phase_name"]
                if st["latched"]:
                    break
                time.sleep(0.1)

        if st["latched"]:
            print("触发:", st["phase_name"],
                  f"| 关节掩码 0x{st['joint_mask']:04X}",
                  "|", peak_text(st))
            print("contact_clear(): 清锁存并在原地重新标定 ...")
            arm.contact_clear()
            st = ensure_armed(arm)
            if st["phase_name"] == ARMED:
                print("已回到 ARMED。")
            else:
                print(f"仍未回到 ARMED (phase={st['phase_name']}) —— "
                      "保持静止后重跑")
        else:
            print("这段时间没有触发锁存 "
                  "(推得不够重, 或 response=warn 只告警不锁存)。")
    finally:
        arm.disable()
        arm.close()
        print("已 disable 并断开 (接触配置只在 RAM, 断电即清)")


if __name__ == "__main__":
    main()
