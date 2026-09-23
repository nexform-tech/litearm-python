#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""_common.py —— 样例共用样板: 命令行开关 + 建 Arm。

约定(安全): 默认【只读】—— 连接后仅能 status/tcp/ik 等查询。
任何会 enable / 运动 / 改参 的样例都要显式加 ``--go`` 才执行, 防误动真机。
"""
import argparse
import os

import litearm as pa


def parser(desc=""):
    ap = argparse.ArgumentParser(description=desc)
    ap.add_argument("--port", default=None, help="串口 (默认自动找 1d50:606f)")
    ap.add_argument("--go", action="store_true",
                    help="真正 enable/运动/改参 (默认只读连接, 不上力)")
    ap.add_argument("--speed", type=float, default=0.3, help="move 速度倍率 0~1")
    return ap


def parse_args(desc=""):
    return parser(desc).parse_args()


def make_arm(args):
    """连接并校验固件版本约定 (Litearm<主.次.修>-{7J|1J} ≥1.5.0)。

    端口优先级: --port > 环境变量 LITEARM_PORT > 自动发现 (1d50:606f)。
    """
    port = args.port or os.environ.get("LITEARM_PORT") or None
    return pa.Arm(port=port).connect()
