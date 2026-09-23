"""CLI (`litearm-python` / `python -m litearm`) —— 破坏性变更后必须仍可用。

`home()` 去掉 speed 参数后, CLI 的 home 分支若还传 `args.speed` 会直接 TypeError;
这类"改了一处忘了另一处"的死角只有真的把 CLI 跑一遍才会暴露。
"""
from __future__ import annotations

import pytest

from litearm import arm as arm_mod


@pytest.fixture
def cli_arm(fake_transport_factory):
    fake_transport_factory()
    return None


def _run(argv):
    return arm_mod.main(argv)


def test_cli_home_runs_without_speed(cli_arm, capsys):
    """`home` 动作不得再依赖 --speed 参数。"""
    assert _run(["--port", "fake", "home"]) == 0
    assert "home done" in capsys.readouterr().out


def test_cli_home_still_accepts_speed_flag_but_ignores_it(cli_arm):
    """--speed 是 CLI 全局参数 (movej 也用); home 上误传不应崩。"""
    assert _run(["--port", "fake", "home", "--speed", "0.5"]) == 0


def test_cli_enable_disable_cycle(cli_arm, capsys):
    assert _run(["--port", "fake", "enable"]) == 0
    assert "enabled" in capsys.readouterr().out
    assert _run(["--port", "fake", "disable"]) == 0


def test_cli_status_and_fw(cli_arm, capsys):
    assert _run(["--port", "fake", "status"]) == 0
    assert _run(["--port", "fake", "fw"]) == 0
    assert "Litearm" in capsys.readouterr().out


# ---------------------------------------------------------------------------
# Task 8 (返回信封 `Msg`): CLI 的两条读分支
#
# `status` 分支直接读 `st.mode_name/st.seq/st.flag_names/st.joints` —— `get_state` 一旦回
# `Msg`, 那一行立刻 `AttributeError` (本文件在此前就会红); 而 `tcp` 分支**更静默**:
# 它从前直接 `print("tcp:", arm.get_tcp())`, 包了信封之后会**把 `Msg` 的 repr 打给用户**
# 而没有任何断言看得见 —— 所以下面那条是**新加**的 (此前零覆盖)。
# ---------------------------------------------------------------------------

def test_cli_status_prints_the_state_and_the_envelope(cli_arm, capsys):
    assert _run(["--port", "fake", "status"]) == 0
    out = capsys.readouterr().out
    assert "mode=" in out and "seq=" in out, "状态正文没打出来 (读 .value 之前就用了?)"
    assert "hz=" in out, "信封里的 hz 没打出来 —— CLI 是唯一的人眼出口"
    assert "J1:" in out, "逐关节那几行没打出来"


def test_cli_tcp_prints_the_value_not_the_msg_repr(cli_arm, capsys):
    assert _run(["--port", "fake", "tcp"]) == 0
    out = capsys.readouterr().out
    assert "Msg(" not in out, "把信封的 repr 打给用户了 —— 要的是那一帧的值"
    assert out.startswith("tcp: (0.3"), f"没打出 TCP 值: {out!r}"
    assert "hz=" in out


def test_cli_movej_requires_right_arity(cli_arm, capsys):
    assert _run(["--port", "fake", "movej", "0.1", "0", "0", "0", "0", "0", "0"]) == 0
    assert "movej done" in capsys.readouterr().out
