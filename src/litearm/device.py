"""RemoteDevice — 通用外设远程接口。

通过 litearm-server 代理访问外设设备（夹爪、灵巧手、示教板等）。
每个设备有唯一的 device_id，方法通过 "device.{device_id}.{method}" 调用。

Usage::

    arm = litearm.Arm(endpoint="tcp/192.168.1.100:7447")

    # 获取设备接口
    hand = arm.device("hand_0")
    gripper = arm.device("gripper_0")
    teach = arm.device("teach_0")

    # 调用设备方法
    hand.open()
    hand.set_gesture("pinch")

    gripper.set_width(0.5)
    width = gripper.get_width()

    joints = teach.get_joints()
"""
from __future__ import annotations

from typing import Any, Callable, Dict, List, Optional


class RemoteDevice:
    """通用外设远程接口。

    通过 litearm-server 代理调用设备方法。
    方法名自动添加 "device.{device_id}." 前缀。
    """

    def __init__(self, device_id: str, rpc_fn: Callable[[str, Any], Any]):
        """创建 RemoteDevice 实例。

        Args:
            device_id: 设备唯一标识（如 "hand_0", "gripper_0"）。
            rpc_fn: RPC 调用函数（由 Arm 注入）。
        """
        self._device_id = device_id
        self._rpc = rpc_fn

    @property
    def device_id(self) -> str:
        return self._device_id

    def _call(self, method: str, **kwargs) -> Any:
        """调用设备方法。"""
        full_method = f"device.{self._device_id}.{method}"
        return self._rpc(full_method, **kwargs)

    # ── 通用方法 ──────────────────────────────────────────────────────────

    def get_status(self) -> Dict[str, Any]:
        """获取设备状态。"""
        return self._call("get_status")

    def get_info(self) -> Dict[str, Any]:
        """获取设备信息。"""
        return self._call("get_info")

    def connect(self) -> bool:
        """连接设备。"""
        return self._call("connect")

    def disconnect(self) -> None:
        """断开设备。"""
        return self._call("disconnect")

    def clear_faults(self) -> bool:
        """清除故障。"""
        return self._call("clear_faults")

    # ── 末端执行器方法（适用于夹爪/灵巧手） ──────────────────────────────

    def open(self) -> bool:
        """打开末端执行器。"""
        return self._call("open")

    def close(self) -> bool:
        """关闭末端执行器。"""
        return self._call("close")

    def set_force(self, force: float) -> bool:
        """设置抓取力。"""
        return self._call("set_force", force=force)

    # ── 灵巧手方法 ────────────────────────────────────────────────────────

    def get_state(self) -> Dict[str, Any]:
        """获取设备状态（device.state_method，与 js handGetState 对齐）。"""
        return self._call("get_state")

    def set_gesture(self, gesture: str) -> bool:
        """设置手势（灵巧手专用）。"""
        return self._call("set_gesture", gesture=gesture)

    def list_gestures(self) -> List[str]:
        """列出支持的手势（灵巧手专用）。"""
        return self._call("list_gestures")

    def finger_move(self, pose: List[float]) -> bool:
        """逐指运动（pose 为各指角度；仅支持 finger_move 的手有效）。"""
        return self._call("finger_move", pose=pose)

    def set_speed(self, speed: List[float]) -> bool:
        """设置各指速度（仅支持 set_speed 的手有效）。"""
        return self._call("set_speed", speed=speed)

    def set_torque(self, torque: List[float]) -> bool:
        """设置各指力矩（仅支持 set_torque 的手有效）。"""
        return self._call("set_torque", torque=torque)

    # ── 夹爪方法 ──────────────────────────────────────────────────────────

    def set_width(self, width: float) -> bool:
        """设置夹爪宽度（夹爪专用）。"""
        return self._call("set_width", width=width)

    def get_width(self) -> float:
        """获取夹爪宽度（夹爪专用）。"""
        return self._call("get_width")

    # ── 示教板方法 ────────────────────────────────────────────────────────

    def get_joints(self) -> List[float]:
        """读取关节角（示教板专用）。"""
        return self._call("get_joints")

    def get_buttons(self) -> Dict[str, bool]:
        """读取按钮状态（示教板专用）。"""
        return self._call("get_buttons")

    def __repr__(self) -> str:
        return f"RemoteDevice({self._device_id!r})"


class DeviceManager:
    """设备管理器 — 管理所有外设接口。

    Usage::

        arm = litearm.Arm(...)
        devices = DeviceManager(arm._rpc)

        hand = devices.get("hand_0")
        gripper = devices.get("gripper_0")

        # 或者直接使用 arm.device()
        hand = arm.device("hand_0")
    """

    def __init__(self, rpc_fn: Callable[[str, Any], Any]):
        self._rpc = rpc_fn
        self._devices: Dict[str, RemoteDevice] = {}

    def get(self, device_id: str) -> RemoteDevice:
        """获取设备接口（延迟创建）。"""
        if device_id not in self._devices:
            self._devices[device_id] = RemoteDevice(device_id, self._rpc)
        return self._devices[device_id]

    def __getitem__(self, device_id: str) -> RemoteDevice:
        return self.get(device_id)

    def __contains__(self, device_id: str) -> bool:
        return device_id in self._devices

    def __repr__(self) -> str:
        return f"DeviceManager({list(self._devices.keys())})"
