"""工具点 (TCP 偏移) —— 法兰 ↔ 工具点的刚体换算、入参校验与本地持久化。

**为什么这一层在宿主侧**: 固件的 TCP 恒为 `ee_link` 的 FK (`get_tcp()`), `move_*` /
`ik()` 的入参与到位判据也全按法兰算 —— 协议里没有工具坐标系命令 (0x01–0x49 无此项)。
要让"我控的是工具尖"成立, 只能在 SDK 里加这一层; 且**收发两侧都得加**: 只加下发侧时,
纯旋转会让工具尖跟着法兰平移, 而不是绕尖原地转 (litearm-vr 的 `tcp_offset` 注释里
记的是同一件事)。

语义 (与 litearm-vr 一致, 另补姿态)::

    p_tool = p_ee + R_ee · xyz        R_tool = R_ee · R_off(rpy)
    p_ee   = p_tool − R_ee · xyz      R_ee   = R_tool · R_offᵀ

`xyz` / `rpy` 定义在**法兰系** (`ee_link`), 与固件的 `payload_com` 同一参考系 ——
换工具时两者要一起改。`rpy = (0, 0, 0)` 时退化成 litearm-vr 那套纯平移。
"""
from __future__ import annotations

import json
import math
import os
from typing import Dict, List, NamedTuple, Optional, Sequence, Tuple

from litearm import _rot
from litearm.errors import InvalidCommandError, LiteArmError

#: 偏移量的模长上限 (米)。超过它几乎只可能是单位/小数点填错 (`0.18` 写成 `180`) ——
#: 直接拒, 而不是让臂按一个离谱的工具点去够。1 m 已远超任何手持工具。
MAX_OFFSET_M = 1.0

#: `rpy` 的合法范围没有物理限制 (旋转本来就在球面上), 但 `|rpy| > 10π` 说明填的是**度数**
#: 或别的量纲 —— 与 `MAX_OFFSET_M` 一样按"量纲错误"拒掉。
MAX_RPY_RAD = 10.0 * math.pi


class TcpOffset(NamedTuple):
    """工具点相对法兰的偏移 (xyz 米 / rpy 弧度, 均在**法兰系**)。"""

    xyz: Tuple[float, float, float]
    rpy: Tuple[float, float, float]

    @property
    def is_identity(self) -> bool:
        """全零 = 直接控法兰 —— 各入口走"原样返回"的快路径, 行为与没有这层时一致。"""
        return not any(self.xyz) and not any(self.rpy)


#: 默认偏移: 全零。litearm-vr 出厂 `(0, 0, 0.18)` 是**那台台架**的量测值, 不该进 SDK 默认值。
ZERO = TcpOffset((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))


def _triple(value, name: str) -> Tuple[float, float, float]:
    """3 个有限数的归一化 —— 形状/量纲问题一律在发帧之前报出来。"""
    if value is None:
        return (0.0, 0.0, 0.0)
    try:
        vals = [float(v) for v in value]
    except (TypeError, ValueError) as e:
        raise InvalidCommandError(f"{name} 需 3 个数字: {e}") from None
    if len(vals) != 3:
        raise InvalidCommandError(f"{name} 需 3 个数 (收到 {len(vals)} 个)")
    if not all(math.isfinite(v) for v in vals):
        raise InvalidCommandError(f"{name} 不能含 NaN/Inf: {vals}")
    return (vals[0], vals[1], vals[2])


def as_offset(xyz=None, rpy=None) -> Tuple[TcpOffset, List[List[float]]]:
    """归一化并校验偏移量, 返回 `(TcpOffset, R_off[3][3])`。

    `xyz` / `rpy` 任一为 `None` 时按 0 处理 (只改平移时不必重述姿态)。
    """
    x = _triple(xyz, "tcp_xyz")
    r = _triple(rpy, "tcp_rpy")
    norm = math.sqrt(sum(v * v for v in x))
    if norm > MAX_OFFSET_M:
        raise InvalidCommandError(
            f"tcp_xyz 模长 {norm:.3f} m 超过上限 {MAX_OFFSET_M} m —— 单位是**米**"
            f" (0.18, 不是 180) 且定义在法兰系")
    if any(abs(v) > MAX_RPY_RAD for v in r):
        raise InvalidCommandError(
            f"tcp_rpy 超过 {MAX_RPY_RAD:.1f} rad —— 单位是**弧度**, 度数请先换算")
    return TcpOffset(x, r), _rot.rpy_to_mat(r)


def _pose(pose, label: str) -> Tuple[List[float], List[List[float]]]:
    """任意位姿写法 -> `([x,y,z], R[3x3])`; 形状/数值畸形一律 `InvalidCommandError`。

    形态判定整个委托给 :func:`litearm._rot.as_pose` (收 `(pos[3], R[3x3])` /
    `[x,y,z,r,p,y]` / 4x4 齐次矩阵); 这里只补两件事: 加上 `label` 前缀, 以及把
    `as_pose` 契约之外的数值畸形 (`float("x")` 的 `ValueError`、容器类型不对的
    `TypeError`) 一并收进 `LiteArmError` 体系 —— 与 `cart._as_pose6` 同一条理由。
    """
    try:
        pos, R = _rot.as_pose(pose)
    except InvalidCommandError as e:
        raise InvalidCommandError(f"{label}: {e}") from None
    except (TypeError, ValueError) as e:
        raise InvalidCommandError(f"{label}: {e}") from None
    return pos, R


def to_tool(pose, offset: TcpOffset, R_off: Sequence[Sequence[float]],
            label: str = "pose") -> List[float]:
    """法兰位姿 → 工具位姿 (`[x,y,z,r,p,y]`)。读回侧用。"""
    pos, R_ee = _pose(pose, label)
    R_tool = _rot.mat_mul(R_ee, R_off)
    t = _rot.mat_vec(R_ee, offset.xyz)
    return [pos[i] + t[i] for i in range(3)] + _rot.mat_to_rpy(R_tool)


def to_flange(pose, offset: TcpOffset, R_off: Sequence[Sequence[float]],
              label: str = "pose") -> List[float]:
    """工具位姿 → 法兰位姿 (`[x,y,z,r,p,y]`)。**下发 `move_*` / `ik` 前必须过这一步。**"""
    pos, R_tool = _pose(pose, label)
    R_ee = _rot.mat_mul(R_tool, _rot.mat_T(R_off))
    t = _rot.mat_vec(R_ee, offset.xyz)
    return [pos[i] - t[i] for i in range(3)] + _rot.mat_to_rpy(R_ee)


# ---------------------------------------------------------------- 本地持久化

def default_path() -> str:
    """默认配置文件: `$LITEARM_TOOL_FILE` 优先, 否则放平台用户配置目录。

    只存"这把工具"的属性 (偏移 + 负载), 不改固件参数 —— 固件里没有这两者的写口组合。
    """
    env = os.environ.get("LITEARM_TOOL_FILE")
    if env:
        return os.path.expanduser(env)
    if os.name == "nt":
        base = os.environ.get("APPDATA") or os.path.expanduser("~")
    else:
        base = os.environ.get("XDG_CONFIG_HOME") or os.path.expanduser("~/.config")
    return os.path.join(base, "litearm", "tool.json")


def save(path: str, offset: TcpOffset, payload: Optional[Dict[str, object]] = None) -> str:
    """写配置 (原子替换: 半截文件比没有文件更糟)。返回实际写入的路径。"""
    doc: Dict[str, object] = {
        "tcp_offset": {"xyz": list(offset.xyz), "rpy": list(offset.rpy),
                       "units": "m / rad, flange (ee_link) frame"},
    }
    if payload:
        doc["payload"] = payload
    full = os.path.abspath(os.path.expanduser(path))
    os.makedirs(os.path.dirname(full), exist_ok=True)
    tmp = full + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(doc, f, ensure_ascii=False, indent=2)
        f.write("\n")
    os.replace(tmp, full)
    return full


def load(path: str) -> Tuple[TcpOffset, Dict[str, object]]:
    """读配置 → `(offset, payload)`。文件不存在抛 `FileNotFoundError` (由调用方决定口径)。"""
    full = os.path.abspath(os.path.expanduser(path))
    with open(full, encoding="utf-8") as f:
        text = f.read()
    try:
        doc = json.loads(text)
    except ValueError as e:
        raise LiteArmError(f"{full} 不是合法 JSON: {e}") from None
    if not isinstance(doc, dict):
        raise LiteArmError(f"{full} 顶层应为对象, 收到 {type(doc).__name__}")
    block = doc.get("tcp_offset") or {}
    if not isinstance(block, dict):
        raise LiteArmError(f"{full} 的 tcp_offset 应为对象")
    offset, _R = as_offset(block.get("xyz"), block.get("rpy"))
    payload = doc.get("payload")
    return offset, payload if isinstance(payload, dict) else {}
