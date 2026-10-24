"""工具点 (TCP 偏移) —— 换算、入参校验、各入口的落地位置、以及持久化。

判据分三层, 缺一层就会出现"注了偏移但没生效"或"注了偏移把别的入口搞坏":

1. **换算本身** (`litearm.tool` 的纯数学, 不连臂);
2. **落到线上** —— 从 `tx_log` 里把那条帧的载荷解出来, 断言它是**法兰**位姿
   (固件只认法兰), 而不是"函数返回了个东西";
3. **零偏移时逐字节不变** —— 这条是回归闸: 默认全零 ⇒ 各入口必须与没有这层时完全相同,
   连"多算一遍再四舍五入回来"都不许。
"""
from __future__ import annotations

import math

import pytest

from litearm import _protocol as P
from litearm import tool
from litearm.errors import InvalidCommandError, MotionTimeoutError


def _payload(arm, cmd: int) -> bytes:
    """最后一条 `cmd` 的载荷 (没有就是测试写错了, 直接失败)。"""
    for c, p in reversed(arm._tr.tx_log):
        if c == cmd:
            return p
    raise AssertionError(f"0x{cmd:02X} 从未被发出")


def _pose_of(payload: bytes, off: int = 0) -> list:
    """载荷里的位姿 (6 个 f32) —— `off` 用来跳过 `0x3D` 前面的 idx 字节。"""
    return [round(v, 6) for v in P.unpack_f32s(payload, off, 6)]


# ---------------------------------------------------------------- 换算本身

def test_offset_round_trip_keeps_pose():
    """法兰 → 工具 → 法兰必须回到原值 (含 rpy 偏移)。"""
    off, R = tool.as_offset((0.05, -0.02, 0.1), (0.0, 0.0, math.pi / 2))
    pose = [0.30, 0.10, 0.40, 0.20, -0.30, 0.50]
    back = tool.to_flange(tool.to_tool(pose, off, R), off, R)
    assert max(abs(pose[i] - back[i]) for i in range(3)) < 1e-12
    assert max(abs(math.remainder(pose[i] - back[i], 2 * math.pi))
               for i in range(3, 6)) < 1e-12


def test_offset_uses_flange_frame():
    """偏移方向随法兰姿态走 —— pitch=90° 时法兰的 +z 在世界系是 +x。"""
    off, R = tool.as_offset((0.0, 0.0, 0.18))
    tool_pose = tool.to_tool([0.30, 0.0, 0.35, 0.0, math.pi / 2, 0.0], off, R)
    assert [round(v, 6) for v in tool_pose[:3]] == [0.48, 0.0, 0.35]


def test_zero_offset_is_identity():
    assert tool.ZERO.is_identity
    assert not tool.as_offset((0.0, 0.0, 1e-9))[0].is_identity


@pytest.mark.parametrize("xyz,rpy", [
    ((0.0, 0.0, float("nan")), None),
    ((0.0, float("inf"), 0.0), None),
    ((0.0, 0.0), None),                       # 两个数
    ((0.0, 0.0, 0.0, 0.0), None),             # 四个数
    (None, (90.0, 0.0, 0.0)),                 # 度数当弧度填 (超过 10π)
    ((0.0, 0.0, 180.0), None),                # 毫米当米填
])
def test_offset_rejects_bad_input(xyz, rpy):
    with pytest.raises(InvalidCommandError):
        tool.as_offset(xyz, rpy)


# ---------------------------------------------------------------- 落到线上

def test_move_p_sends_flange_pose(offline_arm):
    """入参是工具位姿, 线上必须是法兰位姿。"""
    arm = offline_arm
    arm.set_tcp_offset((0.0, 0.0, 0.18))
    arm.move_p((0.48, 0.0, 0.35, 0.0, math.pi / 2, 0.0), speed=0.3)

    sent = _pose_of(_payload(arm, P.CMD_MOVE_P))
    assert sent[:3] == [0.30, 0.0, 0.35]        # 法兰退回 0.18
    assert sent[4] == pytest.approx(math.pi / 2, abs=1e-6)


def test_move_p_arrival_is_judged_on_the_tool(offline_arm, monkeypatch):
    """臂没走到位时仍然要抛超时 —— 偏移不改变"到位"这件事的判据强度。"""
    arm = offline_arm
    arm.set_tcp_offset((0.0, 0.0, 0.18))
    arm.move_timeout = 0.1
    arm._tr.pos_override = [0.90, 0.90, 0.90]   # 桩永远回这个位姿
    with pytest.raises(MotionTimeoutError):
        arm.move_p((0.48, 0.0, 0.35, 0.0, math.pi / 2, 0.0), speed=0.3)


def test_move_l_move_path_convert_every_pose(offline_arm):
    arm = offline_arm
    arm.set_tcp_offset((0.0, 0.0, 0.1))
    arm.move_l((0.30, 0.0, 0.45, 0.0, 0.0, 0.0), speed=0.3)
    assert _pose_of(_payload(arm, P.CMD_MOVE_L))[:3] == [0.30, 0.0, 0.35]

    arm.move_path([(0.30, 0.0, 0.45, 0.0, 0.0, 0.0),
                   (0.32, 0.0, 0.45, 0.0, 0.0, 0.0)], speed=0.3)
    added = [p for c, p in arm._tr.tx_log if c == P.CMD_CART_ADD]
    # `0x3D` 的载荷 = idx u8 + pose[6] ⇒ 位姿从第 1 个字节起
    assert [_pose_of(p, 1)[2] for p in added] == [0.35, 0.35]
    assert [_pose_of(p, 1)[0] for p in added] == [0.30, 0.32]


def test_move_c_accepts_a_tool_space_start(offline_arm):
    """`start` 与实测 TCP 的本地校验两端都换算 ⇒ 传**工具**起点必须能过。

    不换算的写法会在这里直接 `InvalidCommandError` (实测 TCP 是法兰, 差 0.18 m)。
    """
    arm = offline_arm
    arm.set_tcp_offset((0.0, 0.0, 0.18))
    arm._tr.pose = [0.30, 0.0, 0.35, 0.0, 0.0, 0.0]          # 固件实测 (法兰)
    tool_start = [0.30, 0.0, 0.53, 0.0, 0.0, 0.0]            # 同一构型的工具点

    arm.move_c(tool_start,
               (0.30, 0.0, 0.55, 0.0, 0.0, 0.0),
               (0.32, 0.0, 0.55, 0.0, 0.0, 0.0), speed=0.3)
    # `0x3B` 的载荷 = via[6] + goal[6] + sp —— 起点不在帧里 (固件取实测 TCP),
    # 它只参与上面那道**本地**校验, 而那道校验两端都做了换算才会放行。
    raw = _payload(arm, P.CMD_MOVE_C)
    assert _pose_of(raw, 0)[:3] == [0.30, 0.0, 0.37]     # via 换算法兰
    assert _pose_of(raw, 24)[:3] == [0.32, 0.0, 0.37]    # goal 换算法兰


def test_ik_converts_pose(offline_arm):
    arm = offline_arm
    arm.set_tcp_offset((0.0, 0.0, 0.18))
    arm.ik((0.48, 0.0, 0.35, 0.0, math.pi / 2, 0.0), q_seed=[0.0] * 7)
    sent = _pose_of(_payload(arm, P.CMD_GET_IK))
    assert sent[:3] == [0.30, 0.0, 0.35]
    assert sent[4] == pytest.approx(math.pi / 2, abs=1e-6)


def test_get_tcp_stays_flange_and_tool_readback_is_offset(offline_arm):
    arm = offline_arm
    assert arm.get_tcp().value == pytest.approx((0.30, 0.0, 0.35, 0.0, 0.0, 0.0))
    assert arm.get_tool_tcp().value == pytest.approx((0.30, 0.0, 0.35, 0.0, 0.0, 0.0))

    arm.set_tcp_offset((0.0, 0.0, 0.10))
    assert arm.get_tcp().value == pytest.approx((0.30, 0.0, 0.35, 0.0, 0.0, 0.0))
    assert arm.get_tool_tcp().value == pytest.approx((0.30, 0.0, 0.45, 0.0, 0.0, 0.0))


# ---------------------------------------------------------------- 零偏移 = 行为不变

def test_zero_offset_sends_the_pose_untouched(offline_arm):
    """默认零偏移时载荷必须与入参逐字节一致 (回归闸)。"""
    arm = offline_arm
    pose = (0.31, -0.02, 0.36, 0.1, -0.2, 0.3)
    arm.move_p(pose, speed=0.3)
    assert _pose_of(_payload(arm, P.CMD_MOVE_P)) == [round(v, 6) for v in pose]


def test_offset_survives_reconnect(offline_arm):
    """偏移是"这把工具"的属性, 不是会话状态 —— `connect()` 不许清它。"""
    arm = offline_arm
    arm.set_tcp_offset((0.0, 0.0, 0.18), (0.0, 0.0, 0.5))
    arm.reconnect()
    assert arm.tcp_offset.xyz == (0.0, 0.0, 0.18)
    assert arm.tcp_offset.rpy == (0.0, 0.0, 0.5)


def test_clear_offset_restores_flange_control(offline_arm):
    arm = offline_arm
    arm.set_tcp_offset((0.0, 0.0, 0.18))
    arm.clear_tcp_offset()
    assert arm.tcp_offset.is_identity
    pose = (0.31, 0.0, 0.36, 0.0, 0.0, 0.0)
    arm.move_p(pose, speed=0.3)
    assert _pose_of(_payload(arm, P.CMD_MOVE_P)) == [round(v, 6) for v in pose]


# ---------------------------------------------------------------- 负载耦合 + 持久化

def test_payload_is_set_together_with_the_offset(offline_arm):
    """"换工具"两半: 同时给 `payload_mass` 时顺带下发 0x28 item4。"""
    arm = offline_arm
    arm.set_tcp_offset((0.0, 0.0, 0.18), payload_mass=0.58,
                       payload_com=(0.0, 0.0, 0.05))
    item4 = [(c, p) for c, p in arm._tr.tx_log
             if c == P.CMD_SET_FF_SCALAR and p[0] == 4]
    assert item4, "0x28 item4 (payload_mass) 没有下发"
    assert arm.get_ff_scalar(4).value == pytest.approx(0.58)


def test_save_and_load_round_trip(offline_arm, tmp_path):
    arm = offline_arm
    path = str(tmp_path / "sub" / "tool.json")
    assert arm.load_tool(path) is False                   # 文件不存在 ⇒ False, 不抛

    arm.set_tcp_offset((0.01, -0.02, 0.18), (0.0, 0.0, math.pi / 2))
    written = arm.save_tool(path)
    assert written.endswith("tool.json")

    arm.clear_tcp_offset()
    assert arm.tcp_offset.is_identity
    assert arm.load_tool(path) is True
    assert arm.tcp_offset.xyz == pytest.approx((0.01, -0.02, 0.18))
    assert arm.tcp_offset.rpy[2] == pytest.approx(math.pi / 2)


def test_malformed_config_is_reported_not_swallowed(offline_arm, tmp_path):
    arm = offline_arm
    path = tmp_path / "tool.json"
    path.write_text("不是 json", encoding="utf-8")
    with pytest.raises(Exception) as exc:
        arm.load_tool(str(path))
    assert "JSON" in str(exc.value)
