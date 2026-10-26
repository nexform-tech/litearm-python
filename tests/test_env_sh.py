"""env.sh 的严格模式只能作用于自己 —— `source env.sh` 不能改变调用者的 shell。

`source` 在调用者的 shell 里执行，所以文件里一句 `set -e` 会留在调用者的交互
shell 里：之后任何一条命令返回非零，终端就直接退出。这几条用例在子 shell 里
source 一次，确认调用者的选项没被改动、失败命令不再中断 shell。
"""
from __future__ import annotations

import os
import subprocess
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
ENV_SH = REPO / "env.sh"


def _bash(script: str, **kwargs) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["bash", "--norc", "--noprofile", "-c", script],
        capture_output=True,
        text=True,
        timeout=30,
        **kwargs,
    )


def test_sourcing_env_sh_leaves_errexit_off():
    """source 之后 `false` 不能把 shell 干掉。"""
    proc = _bash(f'source "{ENV_SH}" >/dev/null 2>&1; false; echo survived')
    assert proc.returncode == 0, proc.stderr
    assert "survived" in proc.stdout


def test_sourcing_env_sh_keeps_caller_shell_options():
    """source 前后 `set +o` 的输出必须逐字相同。"""
    proc = _bash(
        f'before=$(set +o); source "{ENV_SH}" >/dev/null 2>&1; after=$(set +o); '
        '[ "$before" = "$after" ] && echo unchanged'
    )
    assert proc.returncode == 0, proc.stderr
    assert "unchanged" in proc.stdout


def test_executing_env_sh_enables_strict_mode_cleanly():
    """直接执行时仍然开严格模式，并且在最小环境里也不因未定义变量而失败。"""
    env = {"PATH": os.environ["PATH"], "HOME": os.environ.get("HOME", "")}
    proc = subprocess.run(
        ["bash", str(ENV_SH)],
        capture_output=True,
        text=True,
        timeout=30,
        env=env,
    )
    assert proc.returncode == 0, proc.stderr
    assert f"repo={REPO}" in proc.stdout
