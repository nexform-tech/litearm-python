"""跨线异常必须落在 `LiteArmError` 家族内, 且从包根可导出。

为什么单独钉这两条:
* 「是 LiteArmError 子类」—— 这 4 个原先各自定义在 server 侧且直接继承
  `Exception`, 客户端写 `except LiteArmError` **抓不到**"已被遥操占用"这条
  最常撞的拒绝;
* 「在包根 `__all__` 里」—— SDK 既有的 `test_public_api.py::test_all_listed_names_are_actually_present`
  是**单向**的 (只查"列了的得存在"), 漏加 `__all__` 它**不会红**, 故必须在此显式断言。
"""
from litearm import errors as E

CROSSABLE = ("NotRemoteable", "NotSupportedOnThisBackend",
             "TeleopLockedError", "TeleopBusyError")


def test_crossable_errors_are_litearm_errors():
    for name in CROSSABLE:
        assert issubclass(getattr(E, name), E.LiteArmError), f"{name} 不是 LiteArmError 子类"


def test_exported_from_package_root():
    import litearm as pa
    for name in CROSSABLE:
        assert name in pa.__all__ and hasattr(pa, name)
