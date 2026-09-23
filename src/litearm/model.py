"""动力学模型在线导入 —— 固件 `0x30/0x32/0x33/0x34/0x35/0x37/0x38/0x39` 的面向对象封装。

## 为什么需要它

stm32 固件里, 连杆动力学 (m / com / I) 与电机转子惯量 jm 原先都是**编译期常量**
(`dyn_model_data.h` / `dyn_jm_default.h`)。产线要求「每台臂出厂前上 HYY 控制器辨识
一次动力学 → 把参数写进固件」, 逐台不同 ⇒ 必须能在线写入, 否则每台都得重编译烧录。

固件的模型分三层 (见 `litearm-stm32/User/litearm/dynamics/dyn.h`):

    bank     模型唯一权威, **永不含 payload**; 来源 = 编译期常量 或 导入/flash
    staging  0x30 / 0x33 写这里, 不参与递推
    生效层   bank + payload 复合, RNEA 读这一层

`0x32` commit 把 staging **整组原子**拷进 bank ⇒ 不存在"半截模型生效"。

## 正确用法

```python
arm.model.set_body(1, [m, cmx, cmy, cmz, ixx, ixy, ixz, iyy, iyz, izz])   # J1
...                                                                        # J2..J7
arm.model.set_jm([...])                                                    # 7 个
arm.model.commit(0x2FE)     # body1..7 + jm 的位图; 掩码必须与实际写过的项逐位相等
```

`expected_mask` 的位: `bit i (0..8) = body i`, `bit 9 = jm`。固件用它关掉两类误用:
「写一半就 commit」与「跨会话拼接」。本设计的工具侧固定写 `body1..7 + jm` ⇒ `0x2FE`
(见 `MODEL_MASK_WRITTEN`)。

## 门控

`commit` 与 `revert` **要求失能** (固件判据 `enabled || enable_pending`), 否则回
`ERR{cmd,0x04}`。写 staging (0x30/0x33) 不要求失能 —— 不生效即无风险。

固件侧载荷与错误码见 `litearm-stm32/User/litearm/hal/usb_cmd.c` 的对应分支。
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import TYPE_CHECKING, List, Sequence

from litearm import _protocol as P
from litearm.errors import (InvalidCommandError, TransportError,
                                    UnsupportedByFirmwareError)

if TYPE_CHECKING:                       # 避免与 arm 循环导入
    from litearm.arm import Arm, Msg

__all__ = ["ModelStatus", "ModelParams", "MODEL_NBODY", "MODEL_BODY_PARAMS",
           "MODEL_BIT_BODY", "MODEL_BIT_JM", "MODEL_MASK_WRITTEN"]

#: 模型刚体数 —— **与固件 `dyn_model_data.h` 的 `DYN_MODEL_NBODY` 同值**。
#: ⚠ 与 `arm.n` (关节数, 台架 1J 时为 1) **无关**: 模型恒 9 刚体 (0=base, 1..7=关节,
#: 8=ee 固定体)。照抄 `JointParams._check_idx` 用 `arm.n` 做上界会让台架完全无法验证模型。
MODEL_NBODY = 9

#: 单 body 的字段数: m, cmx, cmy, cmz, ixx, ixy, ixz, iyy, iyz, izz
MODEL_BODY_PARAMS = 10

#: jm 长度 (关节数)
MODEL_JM_N = 7


def MODEL_BIT_BODY(i: int) -> int:
    """body i 的 `expected_mask` 位 (i = 0..8)。"""
    return 1 << int(i)


MODEL_BIT_JM = 1 << 9                       #: jm 的 expected_mask 位

#: 工具/产线实际写入的集合: body1..7 + jm (body0=base 与 body8=ee 不写) = 0x2FE。
#: 固件侧的 `T_DIRTY_ALL & ~(1 | 1 << 8)` 必须与之逐位相等。
MODEL_MASK_WRITTEN = sum(MODEL_BIT_BODY(i) for i in range(1, 8)) | MODEL_BIT_JM
assert MODEL_MASK_WRITTEN == 0x2FE, "MODEL_MASK_WRITTEN 常量漂移"


@dataclass(frozen=True)
class ModelStatus:
    """固件 `RSP_MODEL_STATUS(0x56)` 的 5B 应答。"""

    override: int        #: 0 = 编译期常量; 1 = 导入/flash 模型 (**不区分二者**)
    staged_mask: int     #: bit0..8 = body0..8, bit9 = jm (0..0x3FF)
    dirty: int           #: 1 = RAM 有未固化改动 (定义 = RAM != flash)

    @classmethod
    def decode(cls, payload: bytes) -> "ModelStatus":
        # payload 含首字节 RSP id (与 RSP_FF_VEC/RSP_FF_SCALAR 同口径)
        if len(payload) != 5:
            raise TransportError(f"RSP_MODEL_STATUS 帧长 {len(payload)}B, 期望 5B")
        if payload[0] != P.RSP_MODEL_STATUS:
            raise TransportError(f"RSP_MODEL_STATUS 帧 id 0x{payload[0]:02X}, 期望 0x{P.RSP_MODEL_STATUS:02X}")
        mask = struct.unpack_from("<H", payload, 2)[0]      # u16 **LE**
        return cls(int(payload[1]), int(mask), int(payload[4]))


def _check_body_vals(vals: Sequence[float]) -> List[float]:
    if len(vals) != MODEL_BODY_PARAMS:
        raise InvalidCommandError(
            f"body 参数须为 {MODEL_BODY_PARAMS} 个 (m, cmx, cmy, cmz, ixx, ixy, ixz, iyy, iyz, izz), "
            f"给的是 {len(vals)} 个")
    return [float(v) for v in vals]


class ModelParams:
    """`arm.model` —— 动力学模型的读/写/提交/回退入口。

    无状态: 每次调用直发一帧 (与 `arm.params` 同风格), 不做任何本地缓存
    —— 固件才是唯一权威, 缓存会与"另一端可能改了"打架。
    """

    def __init__(self, arm: "Arm"):
        self._arm = arm

    # ---- 内部 ----
    def _check_body_idx(self, idx: int) -> int:
        idx = int(idx)
        if not 0 <= idx < MODEL_NBODY:
            raise InvalidCommandError(f"刚体索引越界: {idx} (有效 0..{MODEL_NBODY - 1})")
        return idx

    # ---- 0x34 (探测 + 读回共用) ----
    def probe(self) -> bool:
        """固件是否支持模型在线导入。

        用 `0x34` 探测: 旧固件**没有**这条命令的 case ⇒ 落 `default` ⇒ `ERR{0x34,0x00}`
        ⇒ 这里捕获 `UnsupportedByFirmwareError` 并回 `False`。

        ⚠ **不能用 `0x30`/`0x33` 探测** —— 旧固件对它们有**显式 case** 回 `ERR{cmd,0x02}`
        (与"数值非法"同码), 无法区分「无此命令」与「参数非法」。
        """
        try:
            self.get_body(0)
            return True
        except UnsupportedByFirmwareError:
            return False

    def get_body(self, idx: int, timeout: float = 1.0) -> "Msg[List[float]]":
        """读**生效 bank** 的第 `idx` 个刚体 (0..8) 的 10 个参数。

        读的是 bank (无 payload 复合), 故与 `set_body` 写入值可直接逐项比对。
        返回 :class:`Msg` 信封 (帧 id `RSP_MODEL_PARAM`); 单发请求/应答式, 见 `Msg`。
        """
        idx = self._check_body_idx(idx)
        arm = self._arm
        arm._write_query(P.CMD_GET_MODEL_PARAM, bytes([idx]))
        _, p = arm._require().expect(P.RSP_MODEL_PARAM, timeout, f"model.get_body({idx})",
                                     echo_cmd=P.CMD_GET_MODEL_PARAM)
        if len(p) != 2 + 4 * MODEL_BODY_PARAMS:
            raise TransportError(f"RSP_MODEL_PARAM 帧长 {len(p)}B, 期望 {2 + 4 * MODEL_BODY_PARAMS}B")
        if p[0] != P.RSP_MODEL_PARAM:
            raise TransportError(f"RSP_MODEL_PARAM 帧 id 0x{p[0]:02X}, 期望 0x{P.RSP_MODEL_PARAM:02X}")
        if p[1] != idx:
            raise TransportError(f"RSP_MODEL_PARAM 回显 body_idx={p[1]}, 期望 {idx}")
        return arm._msg(list(struct.unpack_from("<" + "f" * MODEL_BODY_PARAMS, p, 2)),
                        P.RSP_MODEL_PARAM)

    # ---- 0x30 ----
    def set_body(self, idx: int, vals: Sequence[float]) -> None:
        """写 staging 的第 `idx` 个刚体。**不生效**, 须 `commit()`。

        固件拒绝 (回 `ERR{0x30,0x02}`) 的情形: 索引越界 / 任一分量非有限或 |v|>1e6 /
        `idx == 8` (ee 固定体, payload 复合位) 而质量非 0。
        """
        idx = self._check_body_idx(idx)
        payload = bytes([idx]) + struct.pack("<" + "f" * MODEL_BODY_PARAMS,
                                             *_check_body_vals(vals))
        self._arm._cmd(P.CMD_SET_MODEL_PARAM, payload, f"model.set_body({idx})")

    # ---- 0x33 ----
    def get_jm(self, timeout: float = 1.0) -> "Msg[List[float]]":
        """读**生效 bank** 的 jm (7 个关节转子惯量)。

        返回 :class:`Msg` 信封 (帧 id `RSP_MODEL_JM`); 单发请求/应答式, 见 `Msg`。
        """
        arm = self._arm
        arm._write_query(P.CMD_GET_MODEL_JM, b"")
        _, p = arm._require().expect(P.RSP_MODEL_JM, timeout, "model.get_jm",
                                     echo_cmd=P.CMD_GET_MODEL_JM)
        if len(p) != 1 + 4 * MODEL_JM_N:
            raise TransportError(f"RSP_MODEL_JM 帧长 {len(p)}B, 期望 {1 + 4 * MODEL_JM_N}B")
        if p[0] != P.RSP_MODEL_JM:
            raise TransportError(f"RSP_MODEL_JM 帧 id 0x{p[0]:02X}, 期望 0x{P.RSP_MODEL_JM:02X}")
        return arm._msg(list(struct.unpack_from("<" + "f" * MODEL_JM_N, p, 1)),
                        P.RSP_MODEL_JM)

    def set_jm(self, vals: Sequence[float]) -> None:
        """写 staging 的 jm[7]。**不生效**, 须 `commit()`。"""
        if len(vals) != MODEL_JM_N:
            raise InvalidCommandError(f"jm 须为 {MODEL_JM_N} 个, 给的是 {len(vals)} 个")
        payload = struct.pack("<" + "f" * MODEL_JM_N, *[float(v) for v in vals])
        self._arm._cmd(P.CMD_SET_MODEL_JM, payload, "model.set_jm")

    # ---- 0x32 ----
    def commit(self, expected_mask: int) -> None:
        """staging 整组原子生效。

        `expected_mask` 必须与固件侧"本次会话实际写过的项"逐位相等, 否则回
        `ERR{0x32,0x07}` (掩码不符 —— 与"数值非法"分开的码, 便于产线定位)。
        零模型哨兵不过回 `ERR{0x32,0x02}`; 已武装回 `ERR{0x32,0x04}`。

        ⚠ u16 **LE** —— `_protocol` 没有 u16 pack helper, 必须显式 `struct.pack("<H", ...)`;
        写成 `bytes([mask])` 会截断成 1 字节 (`0x2FE & 0xFF = 0xFE`) 而固件 `len < 2` 直接拒。
        """
        self._arm._cmd(P.CMD_MODEL_COMMIT, struct.pack("<H", int(expected_mask)),
                       "model.commit", timeout=2.5)

    # ---- 0x37 ----
    def revert(self) -> None:
        """**只回退模型**: bank 回编译期常量 (含 jm) + 清 staging。须失能。

        ⚠ 三条后果 (工具/产线必须知道):
        1. **不动 flash** ⇒ flash 里的导入模型仍在, **重新上电会复活**。
        2. 此后任何一次 `save_params()` 会把 flash 里的导入模型**一并抹掉**
           (整扇区擦除, 旧记录不可恢复)。
        3. 回退后 `status().dirty == 1` (RAM != flash), 但**不要**据此提示"补固化"。
        """
        self._arm._cmd(P.CMD_REVERT_MODEL, b"", "model.revert", timeout=2.5)

    # ---- 0x38 ----
    def status(self, timeout: float = 1.0) -> "Msg[ModelStatus]":
        """读模型状态 (override / staged_mask / dirty)。

        返回 :class:`Msg` 信封 (帧 id `RSP_MODEL_STATUS`); 单发请求/应答式, 见 `Msg`。
        """
        arm = self._arm
        arm._write_query(P.CMD_GET_MODEL_STATUS, b"")
        _, p = arm._require().expect(P.RSP_MODEL_STATUS, timeout, "model.status",
                                     echo_cmd=P.CMD_GET_MODEL_STATUS)
        return arm._msg(ModelStatus.decode(p), P.RSP_MODEL_STATUS)

    # ---- 0x39 ----
    def get_gravity(self, q: Sequence[float], timeout: float = 1.0) -> "Msg[List[float]]":
        """给定关节角算重力项 `G(q)` (纯读, 无门控, 不改任何状态)。

        产线的「静态重力核查」用它: 到位静止后比对实测 τ 与 `G(q)` —— 这是唯一能抓住
        「错台 yaml / 重力符号错 / 模型没生效」的判据 (读回比对只能证明字节落位)。

        返回 :class:`Msg` 信封 (帧 id `RSP_GRAVITY`); 单发请求/应答式, 见 `Msg`。
        """
        q = [float(v) for v in q]
        if len(q) != MODEL_JM_N:
            raise InvalidCommandError(f"q 须为 {MODEL_JM_N} 个, 给的是 {len(q)} 个")
        arm = self._arm
        arm._write_query(P.CMD_GET_GRAVITY, struct.pack("<" + "f" * MODEL_JM_N, *q))
        _, p = arm._require().expect(P.RSP_GRAVITY, timeout, "model.get_gravity",
                                     echo_cmd=P.CMD_GET_GRAVITY)
        if len(p) != 1 + 4 * MODEL_JM_N:
            raise TransportError(f"RSP_GRAVITY 帧长 {len(p)}B, 期望 {1 + 4 * MODEL_JM_N}B")
        if p[0] != P.RSP_GRAVITY:
            raise TransportError(f"RSP_GRAVITY 帧 id 0x{p[0]:02X}, 期望 0x{P.RSP_GRAVITY:02X}")
        return arm._msg(list(struct.unpack_from("<" + "f" * MODEL_JM_N, p, 1)),
                        P.RSP_GRAVITY)
