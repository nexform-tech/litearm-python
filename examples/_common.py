#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""_common.py — litearm-python 客户端样例的共用启动样板。

与 pylitearm 的本地样例不同，这里是【远程客户端】：
  - 不接硬件、不 dry-run、不加载 config —— 这些都在 server 端（地瓜）处理。
  - 客户端只通过 zenoh 连到 litearm-server，把方法调用变成 RPC。
  - 因此没有 --real 开关：连上的就是真机，运动会真实发生！

⚠️ 运动样例会驱动真机！首次跑请把 speed 调到 0.1~0.2，人站在急停旁。

前提：
  1) 地瓜上 litearm-server 已启动（./start_server.sh）
  2) 客户端与地瓜网络互通（默认 endpoint 指向地瓜 IP）
  3) 客户端已装 litearm-python（或用 PYTHONPATH=src）
"""
import argparse

import litearm

N = 7  # 关节数

# 默认连接地瓜控制器（按实际部署改）。本机 loopback 用 tcp/127.0.0.1:7447。
DEFAULT_ENDPOINT = "tcp/192.168.31.237:7447"

# 一个舒展、远离奇异的构型：笛卡尔样例前先 movej 到这里，规避上电竖直伸直位奇异。
Q_HOME = [0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0]


def parse_args(desc=""):
    """标准命令行开关：--endpoint / --arm-id。"""
    ap = argparse.ArgumentParser(description=desc)
    ap.add_argument("--endpoint", default=DEFAULT_ENDPOINT,
                    help=f"litearm-server 的 zenoh 端点（默认 {DEFAULT_ENDPOINT}）")
    ap.add_argument("--arm-id", default="armA",
                    help="Arm 标识（默认 armA）")
    return ap.parse_args()


def make_arm(args):
    """按命令行开关连接远程 Arm。返回的 Arm 记得 close()。"""
    arm = litearm.Arm(endpoint=args.endpoint, arm_id=args.arm_id)
    print(f"[Arm] 已连接 · endpoint={args.endpoint} · arm_id={args.arm_id}")
    return arm
