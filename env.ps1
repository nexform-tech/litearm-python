# litearm-python 运行环境 (PowerShell) —— 在 Windows 下跑 examples / tests
#
# 用法 (PowerShell):
#     . .\env.ps1                # dot-source, 在当前会话设好环境变量
#     python examples\01_hello.py
#     .\run_example.ps1 02_movej.py --go
#
# 作用 (对应 Linux 版 env.sh):
#   1) PYTHONPATH 指向 src\  —— 免 `pip install -e .` 也能 import litearm;
#   2) PYTHON_BIN 默认 'python' (须在 PATH; 装了 pyserial 即可);
#   3) LITEARM_PORT 留空=自动发现 CDC (VID:PID 1d50:606f, 自动映射到 COMx);
#      也可锁成 COM5 之类。
# ============================================================================
$ErrorActionPreference = "Stop"

$env:PYLITEARM_REPO = Split-Path -Parent $MyInvocation.MyCommand.Path

# Windows 分隔符 ';' 合并已有的 PYTHONPATH
$src = Join-Path $env:PYLITEARM_REPO "src"
if (-not $env:PYTHONPATH) {
    $env:PYTHONPATH = $src
} else {
    $env:PYTHONPATH = "$src;$env:PYTHONPATH"
}

# 解释器: 默认 'python' (在 PATH), 可用 $env:PYTHON_BIN 覆盖
if (-not $env:PYTHON_BIN) { $env:PYTHON_BIN = "python" }

# CDC 端口: 留空 = 自动发现
if (-not $env:LITEARM_PORT) { $env:LITEARM_PORT = "" }

Write-Host "[litearm-python env] repo=$($env:PYLITEARM_REPO)"
Write-Host "  PYTHON_BIN   = $($env:PYTHON_BIN)"
Write-Host "  LITEARM_PORT = '$($env:LITEARM_PORT)'  (空=自动发现 1d50:606f)"
Write-Host "  PYTHONPATH   = $($env:PYTHONPATH)"
