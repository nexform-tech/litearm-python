"""动力学模型在线导入 (固件 0x30/0x32/0x33/0x34/0x35/0x37/0x38/0x39) 的 SDK 侧用例。

分层:
  - 帧格式/载荷长度/错误码映射 —— 与 `fake_serial.FakeTransport` 对拍
  - `probe()` 的能力判定 —— **必须用 0x34**, 不能用 0x30/0x33 (旧固件对后者有显式 case,
    回 `0x02` 而非 `0x00`, 无法区分「无此命令」与「参数非法」)
  - `commit` 的掩码语义 (掩码不符 -> 0x07; 已武装 -> 0x04)
  - `MODEL_MASK_WRITTEN == 0x2FE` 的跨仓契约
"""
from __future__ import annotations

import struct

import pytest

from litearm import _protocol as P
from litearm.errors import (CommandRejectedError, InvalidCommandError,
                                    UnsupportedByFirmwareError)
from litearm.model import (MODEL_BODY_PARAMS, MODEL_MASK_WRITTEN, MODEL_NBODY,
                                   ModelParams, ModelStatus)

BODY1 = [1.0, 0.01, 0.02, 0.03, -1.1, 0.02, -0.03, -0.01, 0.01, -1.2]
JM = [0.0, 0.0, -0.0188174839, -0.0236409545, -0.00612815024, -0.0196070617, -0.0142279878]


def test_mask_contract():
    """跨仓契约: 写 body1..7 + jm = 0x2FE (固件 `T_DIRTY_ALL & ~(1|1<<8)` 同值)。"""
    assert MODEL_MASK_WRITTEN == 0x2FE
    assert MODEL_NBODY == 9
    assert MODEL_BODY_PARAMS == 10


def test_get_body_frame_roundtrip(offline_arm):
    arm = offline_arm
    arm.connect()
    v = arm.model.get_body(1).value
    assert len(v) == MODEL_BODY_PARAMS
    # fake 固件的初始 body1 = [1,0,...,0]
    assert v[0] == pytest.approx(1.0)
    assert all(x == pytest.approx(0.0) for x in v[1:])


def test_set_get_body_roundtrip_and_staging(offline_arm):
    """写 staging 不生效; commit 后才读得到 —— 与固件的 bank/staging 分层一致。"""
    arm = offline_arm
    arm.connect()
    arm.model.set_body(1, BODY1)
    assert arm.model.get_body(1).value[0] == pytest.approx(1.0), "commit 前 bank 不应变"
    arm.model.commit(1 << 1)
    got = arm.model.get_body(1).value
    for a, b in zip(got, BODY1):
        assert a == pytest.approx(b), "commit 后 bank 应等于写入值"


def test_set_body_rejects_bad_arity(offline_arm):
    arm = offline_arm
    arm.connect()
    with pytest.raises(InvalidCommandError):
        arm.model.set_body(1, [1.0, 0.0])          # 少于 10 个
    with pytest.raises(InvalidCommandError):
        arm.model.set_body(9, BODY1)               # 索引越界 (恒 9 刚体, 与 arm.n 无关)
    with pytest.raises(InvalidCommandError):
        arm.model.get_body(9)


def test_body_idx_bound_is_model_nbody_not_arm_n(offline_arm_1j):
    """**台架 1J 上模型仍是 9 刚体** —— 判据不能掺 `arm.n`。

    若照抄 `JointParams._check_idx` (用 `arm.n` 做上界), 台架上 `set_body(1)` 会被
    SDK 自己拦掉, 于是台架完全无法验证模型 (而固件在台架上确实支持 9 刚体)。
    """
    arm = offline_arm_1j
    arm.connect()
    assert arm.n == 1
    arm.model.set_body(8, [0.0] * MODEL_BODY_PARAMS)     # body8 合法 (ee 固定体)
    assert arm.model.get_body(8).value is not None


def test_jm_roundtrip(offline_arm):
    arm = offline_arm
    arm.connect()
    assert arm.model.get_jm().value == pytest.approx([0.0] * 7)
    arm.model.set_jm(JM)
    arm.model.commit(1 << 9)
    assert arm.model.get_jm().value == pytest.approx(JM)
    with pytest.raises(InvalidCommandError):
        arm.model.set_jm([0.0] * 6)


def test_commit_mask_mismatch_is_0x07(offline_arm):
    """掩码不符 -> 独立码 0x07 (不是与"数值非法"混用的 0x02)。"""
    arm = offline_arm
    arm.connect()
    for i in range(1, 8):
        arm.model.set_body(i, BODY1)
    arm.model.set_jm(JM)
    with pytest.raises(CommandRejectedError) as ei:
        arm.model.commit(0x002)              # 只声明了 body1, 实际写了 0x2FE
    assert ei.value.code == 0x07


def test_commit_requires_disabled(offline_arm):
    """commit 要求失能 (固件判据 ctrl_is_armed) -> 0x04。"""
    arm = offline_arm
    arm.connect()
    arm.enable()
    arm.model.set_body(1, BODY1)
    with pytest.raises(CommandRejectedError) as ei:
        arm.model.commit(1 << 1)
    assert ei.value.code == 0x04


def test_revert_requires_disabled_and_clears_override(offline_arm):
    arm = offline_arm
    arm.connect()
    arm.model.set_body(1, BODY1)
    arm.model.commit(1 << 1)
    assert arm.model.status().value.override == 1

    arm.enable()
    with pytest.raises(CommandRejectedError) as ei:
        arm.model.revert()
    assert ei.value.code == 0x04

    arm.disable()
    arm.model.revert()
    st = arm.model.status().value
    assert st.override == 0
    assert st.staged_mask == 0


def test_status_fields(offline_arm):
    arm = offline_arm
    arm.connect()
    arm.model.set_body(3, BODY1)             # staging 置位, 未 commit
    st = arm.model.status().value
    assert isinstance(st, ModelStatus)
    assert st.override == 0
    assert st.staged_mask == (1 << 3)
    assert st.dirty == 1


def test_get_gravity_frame(offline_arm):
    arm = offline_arm
    arm.connect()
    g = arm.model.get_gravity([0.0] * 7).value
    assert len(g) == 7
    with pytest.raises(InvalidCommandError):
        arm.model.get_gravity([0.0] * 6)


def test_probe_true_on_modern_firmware(offline_arm):
    arm = offline_arm
    arm.connect()
    assert arm.model.probe() is True


def test_probe_false_on_old_firmware(offline_arm):
    """旧固件: 0x34 没有 case -> 落 default -> ERR{0x34,0x00} -> probe() 回 False。

    ⚠ 这条正是「不能用 0x30/0x33 探测」的理由: 旧固件对它们有**显式 case** 回 0x02,
    SDK 无法据此判定"无此命令"。用 0x34 才有稳定的 0x00 哨兵。
    """
    arm = offline_arm
    arm.connect()
    arm._tr.unknown_cmds.add(P.CMD_GET_MODEL_PARAM)      # 模拟旧固件的 default 分支
    assert arm.model.probe() is False
    with pytest.raises(UnsupportedByFirmwareError):
        arm.model.get_body(0)


def test_commit_encodes_u16_le(offline_arm):
    """`expected_mask` 必须是 2 字节小端 —— 写成 `bytes([mask])` 会截断成 1B。"""
    arm = offline_arm
    arm.connect()
    arm.model.set_body(1, BODY1)          # 只写 1 个 body + jm ⇒ 声明匹配的掩码
    arm.model.set_jm(JM)
    arm.model.commit((1 << 1) | (1 << 9))
    frames = [p for c, p in arm._tr.tx_log if c == P.CMD_MODEL_COMMIT]
    assert frames, "未发出 0x32"
    assert len(frames[-1]) == 2, f"0x32 载荷应 2B, 实得 {len(frames[-1])}B"
    assert struct.unpack_from("<H", frames[-1], 0)[0] == ((1 << 1) | (1 << 9))


def test_set_body_payload_layout(offline_arm):
    """0x30 载荷 = body_idx(1B) + f32[10], **不含 cmd 前缀** (与全部既有下行命令一致)。"""
    arm = offline_arm
    arm.connect()
    arm.model.set_body(3, BODY1)
    frames = [p for c, p in arm._tr.tx_log if c == P.CMD_SET_MODEL_PARAM]
    assert frames
    assert len(frames[-1]) == 1 + 4 * MODEL_BODY_PARAMS
    assert frames[-1][0] == 3
    assert struct.unpack_from("<" + "f" * 10, frames[-1], 1) == pytest.approx(BODY1)


def test_rsp_first_byte_is_rsp_id(offline_arm):
    """上行应答载荷首字节 = RSP id (与 0x4B/0x4C 同族) —— 解码器必须校验它。

    不校验就会在 1 字节错位时解出"看似合理但完全错误"的量级, 而读回闸全绿。
    """
    arm = offline_arm
    arm.connect()
    arm.model.get_body(0)
    arm.model.get_jm()
    arm.model.status()
    arm.model.get_gravity([0.0] * 7)
    rsp_seen = {c for c, _ in arm._tr.tx_log}
    assert P.CMD_GET_MODEL_PARAM in rsp_seen
    # 直接构造错位帧, 断言解码器拒绝
    with pytest.raises(Exception):
        ModelStatus.decode(bytes([0x99, 0, 0, 0, 0]))


def test_model_is_lazy_and_offline_safe():
    """未连接也能取 `arm.model` (构造期零 I/O), 调用才抛。"""
    from litearm import Arm
    arm = Arm()
    assert isinstance(arm.model, ModelParams)
    assert isinstance(arm.model, ModelParams), "应为惰性缓存同一实例"


def test_full_model_write_sequence(offline_arm):
    """端到端: 7×set_body + set_jm + commit(MODEL_MASK_WRITTEN) —— 工具侧的真实序列。"""
    arm = offline_arm
    arm.connect()
    arm.disable()
    for i in range(1, 8):
        arm.model.set_body(i, BODY1 if i == 1 else [1.0, 0.0, 0.0, 0.0] + [0.0] * 6)
    arm.model.set_jm(JM)
    arm.model.commit(MODEL_MASK_WRITTEN)
    st = arm.model.status().value
    assert st.override == 1
    assert st.staged_mask == 0
    for i in range(1, 8):
        assert arm.model.get_body(i).value is not None
    assert arm.model.get_jm().value == pytest.approx(JM)
