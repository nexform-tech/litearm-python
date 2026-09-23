"""可注入传输口 —— 让 Arm 能接一个假传输（离线运行 / litearm-server --dry-run 用）。"""
from litearm import Arm
from litearm.testing import FakeTransport


def test_transport_factory_is_used():
    """给了 transport_factory ⇒ connect() 用它，而不是 SerialTransport。"""
    made = []

    def factory(port):
        made.append(port)
        return FakeTransport()

    arm = Arm(port="fake", transport_factory=factory).connect()
    try:
        assert made == ["fake"], "工厂没被调用 / 拿到的 port 不对"
        assert arm.n == 7, "假件的状态帧应定出 7 轴"
        assert arm.firmware.startswith("Litearm")
    finally:
        arm.close()


def test_no_factory_is_none_by_default():
    """不给工厂 ⇒ 保持 None（connect() 里回落到 SerialTransport）。"""
    assert Arm(port="fake")._transport_factory is None
