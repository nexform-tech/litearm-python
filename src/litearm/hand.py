"""RemoteHand — 向后兼容的灵巧手接口。

通过 arm.hand 访问灵巧手基础操作。
内部路由到 arm.device("hand_0") 实现。

推荐使用 arm.device("hand_0") 代替。
"""
from __future__ import annotations

from typing import Any, Callable, Dict


class RemoteHand:
    """灵巧手远程客户端（向后兼容）。

    内部使用 device.hand_0.* 路由。
    推荐使用 arm.device("hand_0") 代替。

    Usage::

        arm = litearm.Arm(endpoint="tcp/192.168.1.100:7447")

        # 向后兼容（内部路由到 device.hand_0.*）
        arm.hand.open()
        arm.hand.close()
        arm.hand.set_gesture("pinch")

        # 推荐方式
        arm.device("hand_0").open()
    """

    def __init__(
        self,
        rpc_fn: Callable[[str, Any], Any],
        device_id: str = "hand_0",
    ) -> None:
        """创建 RemoteHand 实例。

        Args:
            rpc_fn: RPC 调用函数（由 Arm 注入）。
            device_id: 灵巧手设备 ID（默认 "hand_0"）。
        """
        self._rpc = rpc_fn
        self._device_id = device_id

    def _call(self, method: str, **kwargs) -> Any:
        """调用设备方法（路由到 device.{device_id}.{method}）。"""
        full_method = f"device.{self._device_id}.{method}"
        return self._rpc(full_method, **kwargs)

    def open(self) -> bool:
        """打开手掌。"""
        return self._call("open")

    def close(self) -> bool:
        """关闭手掌。"""
        return self._call("close")

    def set_gesture(self, gesture: str) -> bool:
        """设置手势。"""
        return self._call("set_gesture", gesture=gesture)

    def set_force(self, force: float) -> bool:
        """设置抓取力。"""
        return self._call("set_force", force=force)

    def get_state(self) -> Dict[str, Any]:
        """获取灵巧手状态（device.state_method，与 js handGetState 对齐）。"""
        return self._call("get_state")

    def list_gestures(self) -> list:
        """列出支持的手势。"""
        return self._call("list_gestures")

    def finger_move(self, pose: list) -> bool:
        """逐指运动（pose 为各指角度；仅支持 finger_move 的手有效）。"""
        return self._call("finger_move", pose=pose)

    def set_speed(self, speed: list) -> bool:
        """设置各指速度（仅支持 set_speed 的手有效）。"""
        return self._call("set_speed", speed=speed)

    def set_torque(self, torque: list) -> bool:
        """设置各指力矩（仅支持 set_torque 的手有效）。"""
        return self._call("set_torque", torque=torque)

    def clear_faults(self) -> bool:
        """清除故障。"""
        return self._call("clear_faults")

    def __repr__(self) -> str:
        return f"RemoteHand({self._device_id!r})"
