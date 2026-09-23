"""公开 API 面 —— 新能力与错误类型必须能从包顶层直接取到。

SDK 对外只有 `import litearm as pa` 一个入口, 新加的东西若没进 `__all__`,
使用者就得去摸私有模块 —— 那等于没交付。
"""
from __future__ import annotations

import pytest

import litearm as pa


def test_error_types_are_exported():
    assert issubclass(pa.UnsupportedByFirmwareError, pa.CommandRejectedError)
    assert pa.CommandRejectedError("x", cmd=0x2A, code=0x00).cmd == 0x2A


def test_result_types_are_exported():
    assert pa.JointParam is not None
    assert pa.LogSample is not None
    assert pa.KinBenchResult is not None


def test_all_listed_names_are_actually_present():
    missing = [n for n in pa.__all__ if not hasattr(pa, n)]
    assert not missing, f"__all__ 里列了不存在的名字: {missing}"


def test_new_names_are_in_all():
    for name in ("UnsupportedByFirmwareError", "JointParam", "LogSample",
                 "KinBenchResult", "LogReader", "parse_samples"):
        assert name in pa.__all__, f"{name} 未进 __all__"
        assert hasattr(pa, name)


def test_every_api_raises_not_connected_when_offline():
    """未连接时**所有**对外 API 都必须抛 NotConnectedError。

    漏掉 _require() 的先写后检路径会静默变成 `AttributeError: 'NoneType' object has
    no attribute 'write_frame'` —— 调用方按错误类型做分支 (重连/重试) 的代码全失效。
    """
    arm = pa.Arm()                                    # 故意不 connect()
    cases = {
        "get_status_now": lambda: arm.get_status_now(),
        "get_tcp": lambda: arm.get_tcp(),
        "ik": lambda: arm.ik((0.3, 0, 0.35, 0, 0, 0)),
        "get_ff_vec": lambda: arm.get_ff_vec(1),
        "get_ff_scalar": lambda: arm.get_ff_scalar(1),
        "save_params": lambda: arm.save_params(),
        "zero_g": lambda: arm.zero_g(),
        "enable": lambda: arm.enable(),
        # DFU 也是"先判连接": 未连接时第一因是断开, 不是"本对象已进 DFU 终态"
        # (`enter_dfu` 的 `_require()` 排在预检读取之前)。
        "enter_dfu": lambda: arm.enter_dfu(),
        "movej": lambda: arm.movej([0.0] * 7),
        "params.get_joint_param": lambda: arm.params.get_joint_param(0),
        "params.set_joint_param": lambda: arm.params.set_joint_param(0, 1.0, 1.0, 1.0),
        "params.set_joint_limits": lambda: arm.params.set_joint_limits(0, -1.0, 1.0),
        "params.reset_factory": lambda: arm.params.reset_factory(),
        "log.start": lambda: arm.log.start(10),
        "log.stop": lambda: arm.log.stop(),
        "log.reader().read_all": lambda: arm.log.reader().read_all(),
        "diag.kin_bench": lambda: arm.diag.kin_bench(),
    }
    wrong = []
    for name, fn in cases.items():
        try:
            fn()
            wrong.append(f"{name}: 未报错")
        except pa.NotConnectedError:
            pass
        except Exception as e:                        # noqa: BLE001
            wrong.append(f"{name}: {type(e).__name__} (应为 NotConnectedError)")
    assert not wrong, "未连接时的错误类型不一致: " + "; ".join(wrong)


@pytest.mark.parametrize("call", [
    lambda a: a.move_l([0.32, 0.0, 0.35, 0.0, 0.0, 0.0]),
    lambda a: a.move_c([0.32, 0.0, 0.35, 0.0, 0.0, 0.0],
                       [0.30, 0.02, 0.35, 0.0, 0.0, 0.0],
                       [0.31, 0.01, 0.35, 0.0, 0.0, 0.0]),
    lambda a: a.move_path([[0.31, 0.0, 0.35, 0.0, 0.0, 0.0]]),
    lambda a: a.move_p([[0.31, 0.0, 0.35, 0.0, 0.0, 0.0]]),
    lambda a: a.poll_cart(),
])
def test_cartesian_api_offline_raises_not_connected(call):
    """笛卡尔入口在未连接时也抛 `NotConnectedError` —— 不因入参形状抢先报错。

    ⚠ **改靶说明**: 本条从已删的 `tests/test_cartesian_motion.py` 摘出保留。原靶是
    `movel` / `movec` / **`arm.cartesian` 属性** —— 三者都随 PC 侧规划子包一起没了,
    照原样搬就是一条调已删方法的死用例。现靶是**仍在**的公开入口
    (`move_l` / `move_c` / `move_path` / `move_p` / `poll_cart`)。

    ⚠ 其中 `move_p` 那条刻意传**位姿序列** (它本身要报"请用 `move_path`", 见
    `tests/test_arm_offline.py`): 这条钉的是**守卫顺序** —— 链路没开时第一因是断开,
    不该被入参形状的报错抢先。也就是说它测的不是"序列被拒", 而是"**先判连接**"。
    `cart.py` 三个入口都是 `arm._require()` 排在能力探测/零重力守卫/参数校验之前
    (①~④ 的顺序), 这条用例就是那个顺序的公开面哨兵。
    """
    with pytest.raises(pa.NotConnectedError):
        call(pa.Arm())


def test_version_bumped_for_breaking_change():
    """home()/zero_g() 是破坏性变更 —— 版本必须已越过 1.5.x。"""
    major, minor, *_ = (int(x) for x in pa.__version__.split("."))
    assert (major, minor) >= (1, 6), f"破坏性变更后版本仍为 {pa.__version__}"
