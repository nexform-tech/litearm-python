"""Tests for litearm.hand — RemoteHand (backward compat via device.* routing)."""
import pytest

from litearm.hand import RemoteHand


class MockRPC:
    """Mock RPC function for testing."""

    def __init__(self):
        self.calls = []

    def __call__(self, method, **kwargs):
        self.calls.append((method, kwargs))
        # 返回模拟结果
        if method == "device.hand_0.get_state":
            return {"connected": True, "type": "lingxin", "open": False}
        if method == "device.hand_0.list_gestures":
            return ["open", "close", "pinch", "point", "ok", "rock", "peace"]
        return True


class TestRemoteHand:
    def test_open(self):
        rpc = MockRPC()
        hand = RemoteHand(rpc)
        assert hand.open() is True
        assert rpc.calls == [("device.hand_0.open", {})]

    def test_close(self):
        rpc = MockRPC()
        hand = RemoteHand(rpc)
        assert hand.close() is True
        assert rpc.calls == [("device.hand_0.close", {})]

    def test_set_gesture(self):
        rpc = MockRPC()
        hand = RemoteHand(rpc)
        assert hand.set_gesture("pinch") is True
        assert rpc.calls == [("device.hand_0.set_gesture", {"gesture": "pinch"})]

    def test_set_force(self):
        rpc = MockRPC()
        hand = RemoteHand(rpc)
        assert hand.set_force(0.5) is True
        assert rpc.calls == [("device.hand_0.set_force", {"force": 0.5})]

    def test_get_state(self):
        rpc = MockRPC()
        hand = RemoteHand(rpc)
        state = hand.get_state()
        assert state == {"connected": True, "type": "lingxin", "open": False}
        assert rpc.calls == [("device.hand_0.get_state", {})]

    def test_list_gestures(self):
        rpc = MockRPC()
        hand = RemoteHand(rpc)
        gestures = hand.list_gestures()
        assert gestures == ["open", "close", "pinch", "point", "ok", "rock", "peace"]
        assert rpc.calls == [("device.hand_0.list_gestures", {})]

    def test_finger_move(self):
        rpc = MockRPC()
        hand = RemoteHand(rpc)
        assert hand.finger_move([0.0, 0.5, 1.0]) is True
        assert rpc.calls == [("device.hand_0.finger_move", {"pose": [0.0, 0.5, 1.0]})]

    def test_set_speed(self):
        rpc = MockRPC()
        hand = RemoteHand(rpc)
        assert hand.set_speed([1.0, 1.0, 0.5, 0.5, 0.5, 0.5]) is True
        assert rpc.calls == [("device.hand_0.set_speed",
                              {"speed": [1.0, 1.0, 0.5, 0.5, 0.5, 0.5]})]

    def test_set_torque(self):
        rpc = MockRPC()
        hand = RemoteHand(rpc)
        assert hand.set_torque([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]) is True
        assert rpc.calls == [("device.hand_0.set_torque",
                              {"torque": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]})]

    def test_clear_faults(self):
        rpc = MockRPC()
        hand = RemoteHand(rpc)
        assert hand.clear_faults() is True
        assert rpc.calls == [("device.hand_0.clear_faults", {})]

    def test_repr(self):
        rpc = MockRPC()
        hand = RemoteHand(rpc)
        assert repr(hand) == "RemoteHand('hand_0')"

    def test_custom_device_id(self):
        rpc = MockRPC()
        hand = RemoteHand(rpc, device_id="hand_1")
        hand.open()
        assert rpc.calls == [("device.hand_1.open", {})]


class TestArmHandProperty:
    """Test arm.hand property integration."""

    def test_hand_property_lazy_creation(self):
        """arm.hand 应该延迟创建 RemoteHand 实例。"""
        from litearm.transport import InProcTransport
        from litearm import Arm

        tp = InProcTransport()
        # 注册一个 mock queryable 用于 RPC
        def mock_handler(payload):
            from litearm import codec
            return codec.encode_reply(ok=True, result=True)
        tp.declare_queryable("litearm/v4/armA/rpc", mock_handler)

        arm = Arm(transport=tp, arm_id="armA")

        # 首次访问应该创建实例
        hand1 = arm.hand
        assert isinstance(hand1, RemoteHand)

        # 再次访问应该返回同一实例
        hand2 = arm.hand
        assert hand1 is hand2

        arm.close()

    def test_hand_rpc_routing(self):
        """arm.hand.open() 应该路由到 device.hand_0.open RPC。"""
        from litearm.transport import InProcTransport
        from litearm import Arm, codec

        tp = InProcTransport()
        received_methods = []

        def mock_handler(payload):
            method, kwargs = codec.decode_request(payload)
            received_methods.append(method)
            return codec.encode_reply(ok=True, result=True)

        tp.declare_queryable("litearm/v4/armA/rpc", mock_handler)

        arm = Arm(transport=tp, arm_id="armA")
        arm.hand.open()
        arm.hand.close()
        arm.hand.set_gesture("pinch")

        assert received_methods == [
            "device.hand_0.open",
            "device.hand_0.close",
            "device.hand_0.set_gesture",
        ]

        arm.close()
