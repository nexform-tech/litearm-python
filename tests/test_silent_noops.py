"""静默失效 —— 命令回了 ACK，但**实际效果不是调用方以为的那个**。

这类缺陷比崩溃危险: 调用方拿到成功返回，臂却处于另一种状态。三条来自固件对照审阅，
每一条都已在固件源码里核实过:

* `set_ff_mask(0x1000)`  —— 固件 `params/params.c:237` 是 `ff_mask = mask & FF_ALL_MASK`，
  SDK 也这么掩。于是误用高位 → 掩成 **0 = 前馈全关** → **重力补偿被静默关掉, 臂会垂下来**。
* `set_motion_mode(4)`   —— 固件 `control_loop.c:1338-1342` 的 `ctrl_set_motion_mode()` 只有
  一行 `park_requested = (mode == 0u);`，**mode 从不写进 `g_arm.mode`**。除 0 以外任何值
  都只是"清除 park 声明"，回 ACK 但模式不变。**从前这里只 `warnings.warn` 一下就照发
  命令** —— 告警被日志淹没时调用方拿到的仍是"成功"；现在直接抛 `InvalidCommandError`。
* `set_speed(0.3)`       —— `bytes([0.3])` 抛裸 `TypeError`（不是本包的错误类型）。
"""
from __future__ import annotations

import math

import pytest

from litearm import _protocol as P
from litearm.errors import InvalidCommandError


# --------------------------------------------------------------- ff_mask
def test_set_ff_mask_rejects_bits_outside_the_mask(offline_arm):
    """误用高位不得被静默折成 0 (那等于把前馈全关, 臂会垂)。"""
    arm = offline_arm
    with pytest.raises(InvalidCommandError) as ei:
        arm.set_ff_mask(0x1000)
    assert "0x1FF" in str(ei.value) or "FF_ALL" in str(ei.value) or "范围" in str(ei.value)


def test_set_ff_mask_rejects_negative(offline_arm):
    """`-1 & 0x1FF == 0x1FF` —— 会被静默变成"全开", 同样必须拒绝。"""
    with pytest.raises(InvalidCommandError):
        offline_arm.set_ff_mask(-1)


def test_set_ff_mask_accepts_all_defined_bits(offline_arm):
    arm = offline_arm
    arm.set_ff_mask(P.FF_ALL)
    assert arm.get_ff_mask() == P.FF_ALL
    arm.set_ff_mask(0)
    assert arm.get_ff_mask() == 0


# --------------------------------------------------------- motion mode
def test_set_motion_mode_rejects_out_of_range(offline_arm):
    with pytest.raises(InvalidCommandError):
        offline_arm.set_motion_mode(256)
    with pytest.raises(InvalidCommandError):
        offline_arm.set_motion_mode(-1)


def test_set_motion_mode_rejects_values_other_than_zero(offline_arm):
    """非 0 模式在本固件无效果 (阶段 A 未内建) —— 必须**响亮失败**, 不能回 ACK 当成功。

    ⚠ 这里从前只发一条 `RuntimeWarning` 就照发命令: 告警会被日志淹没, 而调用方
    拿到的是一个"成功"的返回, 臂其实还在旧模式。非 0 值固件**只**改
    `park_requested`(即"清除 park 声明"), 不写 `g_arm.mode`。
    """
    arm = offline_arm
    before = list(arm._tr.tx_log)
    with pytest.raises(InvalidCommandError) as ei:
        arm.set_motion_mode(4)
    msg = str(ei.value)
    assert "只识别 0" in msg, f"报错没有写清本固件只认 0: {msg}"
    assert "park" in msg, f"报错没有给出替代动作 (arm.park()): {msg}"
    assert arm._tr.tx_log == before, "拒绝之后仍然把 0x20 发了出去"


def test_set_motion_mode_zero_is_quiet(offline_arm):
    """mode=0 是 park 声明, 有真实语义, 不该有告警。"""
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        offline_arm.set_motion_mode(0)


def test_park_is_recommended_path(offline_arm):
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        offline_arm.park()


# --------------------------------------------------------------- speed
def test_set_speed_rejects_non_integer(offline_arm):
    """`set_speed(0.3)` 不得抛裸 TypeError —— 按 0..1 思维调用是高发误用。"""
    with pytest.raises(InvalidCommandError):
        offline_arm.set_speed(0.3)


def test_set_speed_rejects_out_of_range(offline_arm):
    with pytest.raises(InvalidCommandError):
        offline_arm.set_speed(101)
    with pytest.raises(InvalidCommandError):
        offline_arm.set_speed(-1)


def test_set_speed_boundaries_are_accepted(offline_arm):
    offline_arm.set_speed(0)
    offline_arm.set_speed(100)


# ------------------------------------------------- move_p 万向锁到位判定
def test_move_p_arrives_when_firmware_canonicalises_rpy_at_gimbal_lock(offline_arm):
    """固件 `kin_rot_to_rpy` 在 |pitch|≈π/2 时**强制 yaw=0** (锁分支 `kin.c:380-384`)。

    目标 (roll=0.5, pitch=π/2, yaw=0.2) 与固件回读的 (roll=0.3, π/2, 0.0) 其实是
    **同一个旋转** (只依赖 roll-yaw), 但逐分量比较会差 0.2rad > rpy_tol ->
    旧实现每次都耗满 move_timeout 才报"未到位"。
    """
    arm = offline_arm
    arm.move_timeout = 0.5
    arm._tr.pos_override = [0.30, 0.0, 0.35]
    arm._tr.rpy_override = [0.3, math.pi / 2, 0.0]     # 固件规范化后的等价表示
    st = arm.move_p((0.30, 0.0, 0.35, 0.5, math.pi / 2, 0.2))
    assert st is not None


def test_move_p_still_rejects_a_genuinely_different_orientation(offline_arm):
    """别把判定放松成"永远到位": 真的差 90° 必须仍判未到位。"""
    arm = offline_arm
    arm.move_timeout = 0.4
    arm._tr.pos_override = [0.30, 0.0, 0.35]
    arm._tr.rpy_override = [0.0, 0.0, math.pi / 2]     # yaw 差 90°
    with pytest.raises(Exception) as ei:
        arm.move_p((0.30, 0.0, 0.35, 0.0, 0.0, 0.0))
    assert "未到" in str(ei.value) or "到位" in str(ei.value)


def test_pose_near_keeps_componentwise_behaviour(offline_arm):
    """新增的旋转等价判定必须是**超集**: 逐分量已判到的, 不能反而判丢。"""
    from litearm.arm import Arm
    tcp = (0.3002, 0.001, 0.35, 3.1416, 0.0, 0.001)
    goal = (0.30, 0.0, 0.35, 3.1416, 0.0, 0.0)
    assert Arm._pose_near(tcp, goal, 0.006, 0.03)
    far = (0.60, 0.0, 0.35, 3.1416, 0.0, 0.0)
    assert not Arm._pose_near(far, goal, 0.006, 0.03)
