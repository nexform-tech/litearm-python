"""笛卡尔固件规划的三种结局 —— 必须与「固件拒绝了这条命令」可区分。

固件原生笛卡尔 (`0x3A/0x3B/0x3C-0x3E` + `RSP_CART_PLAN 0x4E`) 会回三种
**语义完全不同**的结果, 上位机若把它们混进一个通用异常, 调用方就分不清:

- `CartesianPlanError`  —— 真失败 (IK 无解 / 三点共线 / 超容量 / 越限位);
- `MotionSupersededError` —— **预期内的接管**, 我这条请求被新请求取代了, 不是故障;
  把它当失败处理会让正常的抢占流程报错, 甚至触发不该有的故障恢复;
- `CartReplyLostError` —— **未知结局**: 固件单槽 pending 在突发下会吞掉中段应答,
  臂可能动了也可能没动, 只能靠回读状态判定。

三者都**不是** `CommandRejectedError`: 后者是「固件显式 ERR{cmd, code}」,
带 `cmd`/`code` 字段。这三条没有固件错误码, 硬套进去会污染 `code` 的语义
(例如 `code=0x00` 在既有约定里意为「固件无此命令」—— 见 `UnsupportedByFirmwareError`)。
"""
from __future__ import annotations

import importlib.util

import litearm as pa
from litearm.errors import (CartReplyLostError, CartesianPlanError,
                                 CommandRejectedError, LiteArmError,
                                 MotionSupersededError)

NEW_ERRORS = (CartesianPlanError, MotionSupersededError, CartReplyLostError)


def test_three_errors_exist_and_subclass_lite_arm_error():
    for exc in NEW_ERRORS:
        assert issubclass(exc, LiteArmError), f"{exc.__name__} 未继承 LiteArmError"
        assert issubclass(exc, Exception)


def test_none_of_them_is_a_command_rejected_error():
    """它们不是「固件拒绝了这条命令」—— 不得继承 `CommandRejectedError`。

    继承会带来两个实伤: (1) `except CommandRejectedError` 的既有调用方会被
    接管/应答丢失误触发; (2) 凭空多出 `cmd`/`code` 字段而语义是错的。
    """
    for exc in NEW_ERRORS:
        assert not issubclass(exc, CommandRejectedError), (
            f"{exc.__name__} 继承了 CommandRejectedError —— 它不是固件显式拒绝")


def test_superseded_is_distinguishable_from_failure():
    """**接管不是失败**: 捕获失败的代码不得误吞接管, 反之亦然。"""
    assert not issubclass(MotionSupersededError, CartesianPlanError)
    assert not issubclass(CartesianPlanError, MotionSupersededError)
    # 应答丢失是「未知结局」, 既不是失败也不是接管
    assert not issubclass(CartReplyLostError, CartesianPlanError)
    assert not issubclass(CartReplyLostError, MotionSupersededError)


def test_each_error_has_a_docstring_and_carries_a_message():
    for exc in NEW_ERRORS:
        assert exc.__doc__, f"{exc.__name__} 缺 docstring (本仓库风格: 中文 docstring)"
        assert str(exc("具体原因")) == "具体原因"


def test_errors_are_reachable_from_the_catch_all_raise():
    """`raise` / `except` 端到端可用, 且带上原始信息。"""
    try:
        raise MotionSupersededError("被 0x4E 新请求取代")
    except CartesianPlanError:                        # pragma: no cover
        raise AssertionError("接管被误判为规划失败")
    except MotionSupersededError as e:
        assert "取代" in str(e)


def test_new_errors_are_reachable_from_the_package_top_level():
    """SDK 对外只有 `import litearm as pa` 一个入口, 没进 `__all__` 等于没交付。

    这里必须比**对象身份**, 不能只比名字: 重名冲突下 `hasattr(pa, name)` 会
    指着**另一个类**照样绿 —— 那正是本仓最忌讳的假绿。
    """
    for name, exc in (("MotionSupersededError", MotionSupersededError),
                      ("CartReplyLostError", CartReplyLostError)):
        assert name in pa.__all__, f"{name} 未进 __all__"
        assert getattr(pa, name) is exc, \
            f"pa.{name} 不是 litearm.errors 里那个类"


def test_cartesian_plan_error_now_comes_from_errors_and_is_no_longer_the_legacy_class():
    """✅ **绊线已兑现** —— 场景二 Task 8 删掉 `cartesian/` 时公开名已翻转到新类。

    这条的前身是 Task 1 立下的现场绊线: 那时 `pa.CartesianPlanError` 指向**旧**规划器类
    (`cartesian/planner.py`, 基类 `InvalidCommandError`), 新类 `errors.CartesianPlanError`
    从包顶层取不到。绊线的存在意义就是提醒「删包这一刻必须把导入改到 `errors`」——
    那一刻它确实变红了, 本函数就是翻转后的形态。

    ⚠ **为什么不留着"旧类已删"这条事实的空断言**: 旧类随包一起没了, 于是
    "`pa.CartesianPlanError` 不是旧类"这条**换个判据**来钉 —— 旧类**唯一**可观察的
    区别是它继承 `InvalidCommandError` (新类直挂 `LiteArmError`)。用基类关系而非
    模块路径来判, 使本条在旧包已不存在时仍能证伪 (写 `assert not hasattr(...)` 之类
    是永真断言, 证不了任何东西)。

    ⚠ 这里比的是**对象身份**而非名字: 重名冲突下 `hasattr(pa, name)` 指着另一个类
    照样绿 —— 本仓最忌讳的假绿 (见上面 `test_new_errors_are_reachable_from_the_package_top_level`)。
    """
    from litearm import errors as err_mod
    assert pa.CartesianPlanError is err_mod.CartesianPlanError, \
        "pa.CartesianPlanError 不指向 litearm.errors 里那个类"
    #: 旧类的判别式: 删掉的那个类继承 `InvalidCommandError`, 新类**刻意不继承**
    assert not issubclass(pa.CartesianPlanError, pa.InvalidCommandError), \
        "pa.CartesianPlanError 仍继承 InvalidCommandError —— 公开名还指着旧规划器类"
    #: `__all__` 里只许出现一次 (重名冲突的另一个残留形态)
    assert pa.__all__.count("CartesianPlanError") == 1
    #: 旧类真的随包走了 —— 不留"其实还能 import 到"的夹层
    assert importlib.util.find_spec("litearm.cartesian") is None, \
        "litearm.cartesian 仍可导入 —— 子包没删干净"
