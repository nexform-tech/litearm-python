@echo off
rem ===========================================================================
rem 一句话跑样例 (Windows cmd) —— 在仓库根目录里:
rem     run_example 01_hello.py
rem     run_example 02_movej.py --go --speed 0.2
rem     set LITEARM_PORT=COM5 && run_example 03_move_p.py --go
rem 自动设 PYTHONPATH(=src) 后调 python; 样例名外参数原样透传。
rem ===========================================================================
setlocal enabledelayedexpansion
if "%~1"=="" goto :usage

set "NAME=%~1"
shift
set "REST="
:collect
if "%~1"=="" goto :run
set "REST=!REST! "%~1""
shift
goto :collect

:run
call "%~dp0env.cmd" >nul
python "%~dp0examples\%NAME%" !REST!
exit /b %errorlevel%

:usage
echo 用法: run_example ^<example.py^> [args...]
dir /b "%~dp0examples\*.py" 2>nul
exit /b 1
