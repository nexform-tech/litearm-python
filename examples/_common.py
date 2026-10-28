#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""_common.py —— 样例共用样板: 命令行开关 + 建 Arm。

约定(安全): 默认【只读】—— 连接后仅能 status/tcp/ik 等查询。
任何会 enable / 运动 / 改参 的样例都要显式加 ``--go`` 才执行, 防误动真机。
"""
import argparse
import os

import litearm as pa


#: `--go` 的默认说明。个别样例的 `--go` 不 enable/运动 (例如 08 只做只读采样),
#: 那种样例传自己的文案进来, 避免 `--help` 里写着"会上力"而实际不上力。
GO_HELP = "真正 enable/运动/改参 (默认只读连接, 不上力)"


def parser(desc="", go_help=GO_HELP):
    ap = argparse.ArgumentParser(description=desc)
    ap.add_argument("--port", default=None, help="串口 (默认自动找 1d50:606f)")
    ap.add_argument("--go", action="store_true", help=go_help)
    ap.add_argument("--speed", type=float, default=0.3, help="move 速度倍率 0~1")
    return ap


def parse_args(desc=""):
    return parser(desc).parse_args()


def final_enabled(arm):
    """收尾前尽力读一次真实的使能态。

    返回 ``True`` / ``False``; 链路已断或读不到状态时返回 ``None`` —— 调用方只在能
    确定时报告"持位 / 未使能", 否则直说"未知", 不要拿 ``enable()`` 是否正常返回去推断
    (``Arm.enable()`` 收到 ACK 就返回, 状态帧要过几拍才反映; 运行中的故障也可能让臂
    在收尾前已经失能)。
    """
    try:
        st = arm.get_state(refresh=True, timeout=0.3).value
    except Exception:                       # noqa: BLE001 - 收尾尽力而为, 不掩盖主异常
        return None
    return None if st is None else bool(st.enabled)


def make_arm(args):
    """连接并校验固件版本约定 (Litearm<主.次.修>-{7J|1J} ≥1.5.0)。

    端口优先级: --port > 环境变量 LITEARM_PORT > 自动发现 (1d50:606f)。
    """
    port = args.port or os.environ.get("LITEARM_PORT") or None
    return pa.Arm(port=port).connect()
