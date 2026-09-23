@echo off
rem ===========================================================================
rem litearm-python 运行环境 (cmd.exe) —— 同 env.sh/env.ps1
rem
rem 用法 (cmd):
rem     call env.cmd
rem     python examples\01_hello.py
rem     set LITEARM_PORT=COM5 & call env.cmd & python examples\01_hello.py
rem ===========================================================================
set "LITEARM_REPO=%~dp0"
set "PYTHONPATH=%LITEARM_REPO%src;%PYTHONPATH%"
if not defined PYTHON_BIN set "PYTHON_BIN=python"
if not defined LITEARM_PORT set "LITEARM_PORT="
echo [litearm-python env] repo=%LITEARM_REPO%
echo   PYTHON_BIN   = %PYTHON_BIN%
echo   LITEARM_PORT = '%LITEARM_PORT%'  ^(空=自动发现 1d50:606f^)
echo   PYTHONPATH   = %PYTHONPATH%
