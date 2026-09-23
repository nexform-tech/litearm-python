#! /usr/bin/env bash
# ============================================================================
# litearm-python 运行环境 (source 用) —— 让 examples / tests 免安装即可跑
#
#   source env.sh
#   python3 examples/01_hello.py          # 或 python3 -m examples? 见 run_example.sh
#   ./run_example.sh 01_hello.py          # 更省事
#
# 作用:
#   1) PYTHONPATH 指向 src —— 即使未 `pip install -e .` 也能 import litearm;
#   2) 选一个装了 pyserial 的解释器 (见下, 无需手改);
#   3) LITEARM_PORT 可覆盖 CDC 端口; 不设则自动发现 (VID:PID 1d50:606f)。
# ============================================================================
set -euo pipefail

export LITEARM_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="${LITEARM_REPO}/src${PYTHONPATH:+:${PYTHONPATH}}"

# 解释器: 默认 python3; 已设 PYTHON_BIN 则尊重之。否则若默认解释器 import 不到
# pyserial, 就在常见的 conda 位置里找一个能 import 的 —— 这样你不必手改本文件。
if [ -z "${PYTHON_BIN:-}" ]; then
    PYTHON_BIN=python3
    if ! "$PYTHON_BIN" -c 'import serial' >/dev/null 2>&1; then
        for cand in "${HOME}/miniconda3/bin/python3" "${HOME}/anaconda3/bin/python3" python; do
            if command -v "$cand" >/dev/null 2>&1 && "$cand" -c 'import serial' >/dev/null 2>&1; then
                PYTHON_BIN="$cand"
                break
            fi
        done
    fi
    export PYTHON_BIN
fi

# CDC 端口: 留空=自动发现; 想锁定就 `LITEARM_PORT=/dev/ttyACM1 ./run_example.sh 01_hello.py`
export LITEARM_PORT="${LITEARM_PORT:-}"

echo "[litearm-python env] repo=${LITEARM_REPO}"
echo "  PYTHON_BIN   = ${PYTHON_BIN}"
echo "  LITEARM_PORT = '${LITEARM_PORT}'  (空=自动发现 1d50:606f)"
echo "  PYTHONPATH   = ${PYTHONPATH}"
