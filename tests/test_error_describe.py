"""错误码与文案 —— `describe()` 的最小判据。

设计口径（见 litearm-stm32 的错误可观测性方案 v4）：
  * 码 = 固件二元组，不编号、不建反向表
  * 文案 = 命令名 + 通用档模板，只有少数二义码用 _OVERRIDES
  * 未知码也要能渲染，不许空白
"""
from __future__ import annotations

try:
    import pytest
except ImportError:
    import unittest
    raise unittest.SkipTest("pytest is not installed")

import litearm as pa
from litearm import errors as E


def test_describe_command_rejection_zh():
    t = E.describe(0x2B, 0x02)
    assert t.code == "2B-02"
    assert "GET_FF_VEC" in t.title
    assert "字段非法" in t.title
    assert t.action


def test_describe_command_rejection_en():
    t = E.describe(0x2B, 0x02, "en-US")
    assert t.code == "2B-02"
    assert "GET_FF_VEC" in t.title
    assert "invalid field" in t.title
    assert t.action


def test_describe_unknown_code_is_not_blank():
    t = E.describe(0x99, 0x09, "en-US")
    assert t.code == "99-09"
    assert "CMD 0x99" in t.title
    assert "unregistered" in t.title


def test_describe_override_ambiguous_code():
    t = E.describe(0x02, 0x03, "zh-CN")
    assert "IK" in t.title and "未使能" in t.title


def test_describe_fault_reason():
    t = E.describe_fault(1)
    assert t.code == "FF-01"
    assert "CAN" in t.title
    assert "80 ms" in t.cause
    assert "复位" in t.action

    en = E.describe_fault(1, "en-US")
    assert en.code == "FF-01"
    assert "CAN feedback timeout" in en.title


def test_describe_fault_unknown_reason_is_not_blank():
    t = E.describe_fault(99, "en-US")
    assert t.code == "FF-63"
    assert "unregistered" in t.title


def test_every_local_error_class_has_unique_id():
    ids = {}
    for name in dir(E):
        obj = getattr(E, name)
        if isinstance(obj, type) and issubclass(obj, E.LiteArmError) and obj is not E.LiteArmError:
            eid = getattr(obj, "err_id", None)
            if isinstance(eid, str) and eid.startswith("E-") and eid != "E-??":
                assert eid not in ids, f"{name} 与 {ids[eid]} 共用 {eid}"
                ids[eid] = name
    assert len(ids) >= 16


def test_every_local_id_has_text_in_both_locales():
    for loc in ("zh-CN", "en-US"):
        for eid in E.LOCAL_TEXT[loc]:
            t = E.local_text(eid, loc)
            assert t.title and t.title != eid


def test_describe_exc_reaches_wrapped_firmware_error():
    cause = E.CommandRejectedError("x", cmd=0x2B, code=0x02)
    try:
        try:
            raise cause
        except E.CommandRejectedError:
            raise OSError("参数读回被拒") from cause
    except OSError as outer:
        t = E.describe_exc(outer)
        assert t.code == "2B-02"
        assert "GET_FF_VEC" in t.title


def test_describe_exc_uses_local_id():
    t = E.describe_exc(E.TransportError("boom"))
    assert t.code == "E-03"
    assert "链路" in t.title


def test_locale_normalization():
    assert E.normalize_locale("zh") == "zh-CN"
    assert E.normalize_locale("zh_CN") == "zh-CN"
    assert E.normalize_locale("fr-FR") == "en-US"
    assert E.normalize_locale(None) == "zh-CN"


def test_all_firmware_err_text_pairs_render():
    """ERR_TEXT 里的每个 (cmd, code) 都能经 describe() 得到非空标题。"""
    bad = []
    for (cmd, code) in E.ERR_TEXT:
        t = E.describe(cmd, code, "en-US")
        if not t.title or not t.code:
            bad.append((cmd, code))
    assert not bad, f"{len(bad)} 条渲染为空"


def test_state_guards_have_their_own_codes():
    """零重力/笛卡尔在途不再折成通用的 E-05。"""
    from litearm.errors import (CartInFlightError, ZeroGActiveError,
                                describe_exc)
    zg = ZeroGActiveError("零重力保活正在进行")
    assert zg.err_id == "E-17"
    assert isinstance(zg, E.InvalidCommandError)
    t = describe_exc(zg)
    assert t.code == "E-17" and "零重力" in t.title
    assert t.action  # 处置建议在 action，不是 cause（曾错位）

    ci = CartInFlightError("笛卡尔在途")
    assert ci.err_id == "E-18"
    assert "笛卡尔" in describe_exc(ci).title


def test_command_rejected_error_keeps_fields():
    exc = pa.CommandRejectedError("x", cmd=0x10, code=0x06)
    assert exc.cmd == 0x10 and exc.code == 0x06
