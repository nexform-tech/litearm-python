"""关节级运行时参数 —— 固件 `CMD_SET_JOINT_PARAM(0x22)` / `SET_JOINT_LIMITS(0x23)` /
`GET_JOINT_PARAM(0x24)` / `PARAM_RESET(0x36)` 的面向对象封装。

与 FF/动力学调参族 (留在 `Arm` 上) 的分工: 那一族是**前馈/控制律**参数
(0x26/0x27/0x28/0x31 + 读回 0x2B/0x2C); 本模块是**关节级** MIT 刚度/阻尼/力矩钳幅与
软限位。两者都是 RAM 生效, 持久化统一走 `Arm.save_params()` (0x25)。

固件侧载荷与错误码见 `litearm-stm32/User/litearm/hal/usb_cmd.c` 的
`CMD_SET_JOINT_PARAM`/`CMD_SET_JOINT_LIMITS`/`CMD_GET_JOINT_PARAM`/`CMD_PARAM_RESET` 分支。
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import TYPE_CHECKING

from litearm import _protocol as P
from litearm.errors import InvalidCommandError, TransportError

if TYPE_CHECKING:                       # 避免与 arm 循环导入
    from litearm.arm import Arm, Msg

__all__ = ["JointParam", "JointParams"]


@dataclass(frozen=True)
class JointParam:
    """固件 `RSP_JOINT_PARAM(0x49)` 的 21B 应答: idx + kp,kd,tau_max,q_min,q_max。"""

    idx: int
    kp: float
    kd: float
    tau_max: float
    q_min: float
    q_max: float

    @classmethod
    def decode(cls, payload: bytes) -> "JointParam":
        if len(payload) < 21:
            raise TransportError(f"RSP_JOINT_PARAM 帧短 ({len(payload)}B, 期望 21B)")
        idx = payload[0]
        kp, kd, tm, lo, hi = struct.unpack_from("<fffff", payload, 1)
        return cls(idx, kp, kd, tm, lo, hi)


class JointParams:
    """`arm.params` —— 关节级参数的读写入口 (RAM 生效)。"""

    def __init__(self, arm: "Arm"):
        self._arm = arm

    # ---- 内部 ----
    def _check_idx(self, idx: int) -> int:
        idx = int(idx)
        n = self._arm.n
        if not 0 <= idx < max(n, 1):
            raise InvalidCommandError(f"关节索引越界: {idx} (有效 0..{n - 1})")
        return idx

    # ---- 0x22 ----
    def set_joint_param(self, idx: int, kp: float, kd: float, tau_max: float) -> None:
        """改写单关节 MIT 刚度/阻尼/力矩钳幅 (RAM; 须 `save_params()` 才持久化)。

        固件侧还有一道值合法性校验 (`params_set_joint_ctrl`), 非法时回 `ERR{0x22,0x02}`。
        """
        idx = self._check_idx(idx)
        payload = bytes([idx]) + struct.pack("<fff", float(kp), float(kd), float(tau_max))
        self._arm._cmd(P.CMD_SET_JOINT_PARAM, payload, f"set_joint_param(J{idx + 1})")

    # ---- 0x23 ----
    def set_joint_limits(self, idx: int, q_min: float, q_max: float) -> None:
        """改写单关节软限位 (RAM)。固件要求 `q_min < q_max`。"""
        idx = self._check_idx(idx)
        if not float(q_min) < float(q_max):
            raise InvalidCommandError(f"软限位需 q_min < q_max (给的是 {q_min}, {q_max})")
        payload = bytes([idx]) + struct.pack("<ff", float(q_min), float(q_max))
        self._arm._cmd(P.CMD_SET_JOINT_LIMITS, payload, f"set_joint_limits(J{idx + 1})")

    # ---- 0x24 ----
    def get_joint_param(self, idx: int, timeout: float = 1.0) -> "Msg[JointParam]":
        """读回单关节参数 (与 `set_joint_param`/`set_joint_limits` 构成写→读回闭环)。

        返回 :class:`Msg` 信封 (帧 id `RSP_JOINT_PARAM`)。单发请求/应答式 ⇒ 第一次调用
        `hz == 0.0`, 第二次起等于本调用方自己的轮询频率 (见 `Msg`)。
        """
        idx = self._check_idx(idx)
        arm = self._arm
        arm._write_query(P.CMD_GET_JOINT_PARAM, bytes([idx]))      # 查询类: 不受守卫限制
        _, p = arm._require().expect(P.RSP_JOINT_PARAM, timeout,
                                     f"get_joint_param(J{idx + 1})",
                                     echo_cmd=P.CMD_GET_JOINT_PARAM)
        return arm._msg(JointParam.decode(p), P.RSP_JOINT_PARAM)

    def all_joint_params(self) -> list:
        """逐关节读回全部参数 (N 次往返)。

        ⚠ 返回 `list[JointParam]` —— **不是** `list[Msg]`: 它是 N 次往返的**聚合**
        (N 帧、N 个到达时刻), 一个 `hz`/`timestamp` 描述不了它; 要单帧的信封请调
        `get_joint_param(i)`。
        """
        return [self.get_joint_param(i).value for i in range(self._arm.n)]

    # ---- 0x36 ----
    def reset_factory(self) -> None:
        """恢复出厂默认 + 失效 flash (固件 0x36)。

        ⚠ 固件**要求失能态** (擦写窗口 CPU 停顿, 电机不能在无监督下保持使能),
        已武装时回 `ERR{0x36,0x04}`; 本方法不代劳 `disable()`, 以免替调用方做安全决策。
        ⚠ 失败时固件保持 RAM 与 flash 一致 (先失效 flash 再重载 RAM), 回出的 ERR 是真话。
        """
        self._arm._cmd(P.CMD_PARAM_RESET, b"", "reset_factory", timeout=2.5)
