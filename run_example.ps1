# litearm-python 一键跑样例 (Windows PowerShell)
#
# 用法:
#     .\run_example.ps1 01_hello.py
#     .\run_example.ps1 02_movej.py --go --speed 0.2
#     $env:LITEARM_PORT="COM5"; .\run_example.ps1 03_move_p.py --go
#     .\run_example.ps1                 # 无参: 列出样例
# ============================================================================
param(
    [Parameter(Mandatory = $false)][string]$Name = "",
    [Parameter(ValueFromRemainingArguments = $true)][string[]]$Rest
)
$ErrorActionPreference = "Stop"

. (Join-Path $PSScriptRoot "env.ps1")

if (-not $Name) {
    Write-Host "用法: $PSCommandPath <example.py> [args...]"
    Get-ChildItem (Join-Path $env:PYLITEARM_REPO "examples") -Filter *.py |
        ForEach-Object { Write-Host "  $($_.Name)" }
    exit 1
}

$example = Join-Path $env:PYLITEARM_REPO ("examples\" + $Name)
if (-not (Test-Path $example)) {
    Write-Error "找不到样例: $example"
    exit 1
}

& $env:PYTHON_BIN $example @Rest
exit $LASTEXITCODE
