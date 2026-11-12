"""参数读写口与固件对齐 —— item 全集 + 读回闭环 (0x2B/0x2C -> 0x4B/0x4C)。

固件权威 (litearm-stm32/User/litearm/params/params.c + hal/usb_cmd.c):
  0x26 SET_FF_VEC   item 1..11 + 15/16 (1-11 关节/摩擦 v2 / 15 kd_extra / 16 drag_gain)
                     12..14 (zg_kp/zg_kd/zg_damping) 固件 1.11.0 已删除, 编号不复用
  0x28 SET_FF_SCALAR item 1..8 + 11 + 17,18 (item 9 保留: 写口拒绝, 只给 0x2C 读 ff_mask;
                     item 10 全局 drag_gain 固件 1.12.0 已删除, 改为 0x26 item16)
                     12 (drag_kd_margin) 与 13..16 (zg_vel_thr/zg_engage_*) 1.11.0 已删除
  0x2B -> RSP_FF_VEC    0x4B: payload = [0x4B, item, 7×f32 LE] (30B)
  0x2C -> RSP_FF_SCALAR 0x4C: payload = [0x4C, item, sub, f32 LE] (7B)

⚠ item 合法性**只由固件判定** (不支持 -> ERR{cmd,0x02}); SDK 只校验 u8 范围。
  客户端白名单会与固件版本耦合 —— 1.10.x 上 12..14/13..16 仍可读写, 1.11.0 起
  才删除; 维护白名单必然把某个版本的真参数挡在门外。
"""
from __future__ import annotations

import struct

import pytest

from litearm import _protocol as P
from litearm.errors import InvalidCommandError

VEC_ITEMS = list(range(1, 12)) + [15, 16]             # 1..11 + 15 kd_extra + 16 drag_gain
SCALAR_ITEMS = list(range(1, 9)) + [11, 17, 18]       # 9 = 只读 ff_mask; 10 已于 1.12.0 删除


def test_protocol_ids_match_firmware():
    assert (P.CMD_GET_FF_VEC, P.CMD_GET_FF_SCALAR) == (0x2B, 0x2C)
    assert (P.RSP_FF_VEC, P.RSP_FF_SCALAR) == (0x4B, 0x4C)


#: 按固件 params.c 的**符号/值域契约**给每个 item 取合法样例值。
#: ⚠ 桩 (tests/fake_serial.py) 只存值、不建模 params.c 的钳幅与符号守卫 ——
#:   样例值若违反契约, 离线仍会"绿"而真机必被拒 (item 11 fc1 必须 ≤0 就是这么踩的)。
_VEC_SAMPLE = {
    11: [-0.1] * 7,          # friction_fc1 ≤ 0
}


@pytest.mark.parametrize("item", VEC_ITEMS)
def test_set_ff_vec_accepts_all_firmware_items(offline_arm, item):
    """固件 1.12.0 支持的 1..11 + 15/16 全部可写。"""
    offline_arm.set_ff_vec(item, _VEC_SAMPLE.get(item, [0.1] * 7))


def test_vec_sample_values_respect_firmware_sign_contract():
    """守住上面的样例值本身: fc1 必须 ≤0, 其余非负, 否则测试会掩盖真机拒绝。"""
    assert all(v <= 0 for v in _VEC_SAMPLE[11])
    for item, vals in _VEC_SAMPLE.items():
        if item == 11:
            continue
        assert all(v >= 0 for v in vals)


@pytest.mark.parametrize("item", [0, -1, 256])   # 只有 u8 范围外才本地拒
def test_set_ff_vec_rejects_out_of_range(offline_arm, item):
    with pytest.raises(InvalidCommandError):
        offline_arm.set_ff_vec(item, [0.0] * 7)


@pytest.mark.parametrize("item", [12, 13, 14, 17, 200])
def test_set_ff_vec_passes_unknown_items_to_the_firmware(offline_arm, item):
    """客户端不再维护白名单 —— 帧必须真的发出去, 由固件回 ERR 判定支持与否。

    判据看 `tx_log` 而不是"没抛异常": 后者对"本地静默丢弃"没有观测面。
    """
    offline_arm.set_ff_vec(item, [0.0] * 7)
    assert item in [p[0] for c, p in offline_arm._tr.tx_log
                    if c == P.CMD_SET_FF_VEC]


@pytest.mark.parametrize("item", SCALAR_ITEMS)
def test_set_ff_scalar_accepts_all_firmware_items(offline_arm, item):
    offline_arm.set_ff_scalar(item, 0, 1.0)


@pytest.mark.parametrize("item", [9, 12, 13, 14, 15, 16, 19])
def test_set_ff_scalar_passes_every_item_to_the_firmware(offline_arm, item):
    """9 是只读扩展、12..16 是 1.11.0 已删项 —— 客户端一律透传, 固件自会拒绝。"""
    offline_arm.set_ff_scalar(item, 0, 1.0)
    assert item in [p[0] for c, p in offline_arm._tr.tx_log
                    if c == P.CMD_SET_FF_SCALAR]


def test_set_ff_scalar_sub_out_of_range(offline_arm):
    with pytest.raises(InvalidCommandError):
        offline_arm.set_ff_scalar(5, 3, 0.0)


# ------------------------------------------------------- 读回 (写→读闭环)
def test_get_ff_vec_roundtrip(offline_arm):
    vals = [1.0, 2.0, 3.0, 4.0, 0.5, 0.25, 0.125]
    offline_arm.set_ff_vec(7, vals)                 # gravity_scale
    got = offline_arm.get_ff_vec(7).value
    assert len(got) == 7
    assert all(abs(a - b) < 1e-6 for a, b in zip(got, vals))


def test_get_ff_vec_item_echoed(offline_arm):
    """应答载荷首字节是 RSP id, 第二字节是 item —— 解析必须跳过这两字节。"""
    offline_arm.set_ff_vec(9, [0.5] * 7)            # 摩擦 v2 fv
    assert offline_arm.get_ff_vec(9).value == [0.5] * 7


def test_drag_gain_is_per_joint_item16(offline_arm):
    """[1.12.0] 逐关节拖动摩擦补偿比例走 0x26 item16 (7 值)。"""
    vals = [0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]
    offline_arm.set_drag_gain(vals)
    ps = [p for c, p in offline_arm._tr.tx_log if c == P.CMD_SET_FF_VEC]
    assert ps[-1][0] == 16
    got = offline_arm.get_drag_gain().value
    assert all(abs(a - b) < 1e-6 for a, b in zip(got, vals))
    with pytest.raises(InvalidCommandError):
        offline_arm.set_drag_gain([0.6] * 6)


def test_get_ff_scalar_roundtrip(offline_arm):
    offline_arm.set_payload(0.75, (0.01, 0.02, 0.03))
    assert abs(offline_arm.get_ff_scalar(4, 0).value - 0.75) < 1e-6      # payload_mass
    assert abs(offline_arm.get_ff_scalar(5, 1).value - 0.02) < 1e-6      # payload_com[1]


def test_set_payload_sends_the_raw_value_and_leaves_clamping_to_the_firmware(offline_arm):
    """⚠ **SDK 不替固件钳制**: `set_payload(-5)` 那一帧的 f32 载荷**就是 `-5.0`**。

    钳制发生在固件 (`params.c:184` 的 `ff_clamp(v, 0.0f, 20.0f)`, `:189` 对 com 逐轴
    `[-1,1]`) —— 主机侧的**原值**是固件唯一的输入。判据必须**解载荷**:
    只断言"帧发出去了"对"SDK 先把值钳了再发"**完全没有观测面** (实测: 把 `set_payload`
    改成先 `min(max(mass,0),20)` / `min(max(v,-1),1)` 再发, 全量 531 passed 全绿), 而那
    与整条 `(0x28,0x02)` 口径的"**固件权威**"主张**直接冲突** —— 读回值
    (`get_ff_scalar`) 与发出值会分叉, 现场按发出值推断重力前馈就会错。

    ⚠ **别写成"读回 == 0"**: 桩 (`fake_serial.py:297-299`) **不建模 `ff_clamp`**, 只存
    原值 —— "钳到 0" 是真机的行为, 在离线用例里那样断言是**假判据**。
    """
    arm = offline_arm
    arm.set_payload(-5.0, (2.0, -3.0, 0.0))

    ps = [p for c, p in arm._tr.tx_log if c == P.CMD_SET_FF_SCALAR]
    assert [(p[0], p[1]) for p in ps] == [(4, 0), (5, 0), (5, 1), (5, 2)], (
        f"`set_payload` 发出的 (item, sub) 序列变了: {[(p[0], p[1]) for p in ps]}")
    sent = [struct.unpack_from("<f", p, 2)[0] for p in ps]
    assert sent == [-5.0, 2.0, -3.0, 0.0], (
        f"SDK 在本地把载荷钳过再发 (发出去的是 {sent}) —— 钳制的权威在固件, "
        f"主机侧必须原样发")


def test_get_ff_mask_roundtrip(offline_arm):
    """item 9 只读扩展: 用 0x2C 读回 ff_mask。"""
    offline_arm.set_ff_mask(P.FF_MASTER | P.FF_G | P.FF_FRICTION)
    assert offline_arm.get_ff_mask() == (P.FF_MASTER | P.FF_G | P.FF_FRICTION)


@pytest.mark.parametrize("item", [0, -1, 256])
def test_get_ff_vec_rejects_out_of_u8_range(offline_arm, item):
    """范围外的 item 仍本地拒 (帧格式装不下); 范围内的未知项交给固件。"""
    with pytest.raises(InvalidCommandError):
        offline_arm.get_ff_vec(item)


def test_get_ff_vec_sends_unknown_item_to_the_firmware(offline_arm):
    offline_arm.get_ff_vec(99)      # 桩对未知 item 也回帧; 真机则回 ERR{0x2B,0x02}
    assert 99 in [p[0] for c, p in offline_arm._tr.tx_log
                  if c == P.CMD_GET_FF_VEC]
