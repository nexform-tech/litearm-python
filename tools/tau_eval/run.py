#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tau_eval 启动脚本 —— 从仓库任意目录可直接运行。

    python3 tools/tau_eval/run.py              # 默认子命令 run: 只读打印计划
    python3 tools/tau_eval/run.py --go         # 真跑三场景 + 分析 + 出图 + 报告
    python3 tools/tau_eval/run.py analyze runs/20261029_153000_tau_eval

把 `tools/` 放到 sys.path 后以 `tau_eval` 包导入, 避免依赖当前工作目录。
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_TOOLS = os.path.dirname(_HERE)
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)

from tau_eval.cli import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
