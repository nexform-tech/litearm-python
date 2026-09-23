#! /usr/bin/env bash
# ============================================================================
# 运行单个 example (设置好环境再跑), 用法:
#
#   ./run_example.sh 01_hello.py
#   ./run_example.sh 02_movej.py --go --speed 0.2
#   LITEARM_PORT=/dev/ttyACM0 ./run_example.sh 01_hello.py
#
# 等价于: source env.sh && $PYTHON_BIN examples/<name> [args...]
# (每个样例自带 sys.path.insert(example目录) 以 import _common)
# ============================================================================
set -euo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ $# -lt 1 ]; then
    echo "用法: $0 <example.py> [args...]"
    echo "样例:"
    ls "$DIR/examples/"*.py 2>/dev/null | sed 's#.*/##' || true
    exit 1
fi

name="$1"; shift || true
source "$DIR/env.sh"

example="$DIR/examples/$name"
if [ ! -f "$example" ]; then
    echo "找不到样例: $example"; exit 1
fi

exec "$PYTHON_BIN" "$example" "$@"
