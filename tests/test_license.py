"""授权/激活 (`0x2F` 查询 / `0x3F` 提交) —— 固件 1.8.0 起的开机即锁。

固件侧事实 (逐条对着 `litearm-stm32` 源码抄的, 不是转述):
  · `ctrl_enable()` 的**第一条**判据是 `!license_is_activated()` -> `0x08`
    (`control/control_loop.c:938-946`) ⇒ 未激活时 `ENABLE` 恒被拒, **重发无用、无旁路**;
  · `0x2F` 的应答 `RSP_LICENSE(0x4F)` 是 **26B**, 首字节是 `state` 而**不是**冗余帧 id
    (`hal/usb_cmd.c:1016-1030`);
  · `0x3F` 的失败码除长度 `0x01` 外**一律折成 `0x02`**, 于是"已经激活过"与"真失败"
    **同码** (`hal/usb_cmd.c:1067-1079`);
  · 激活成功回的是 `ACK{0x3F}`, 不是 `RSP_ACTIVATE(0x50)` —— 后者**本版无生产者**。

桩 (`litearm.testing.FakeTransport`) 按同一套顺序建模, 但有一处**结构性**做不到:
它**不验 MAC** —— 桩没有也不该有密钥 (规格: 客户侧不许有能算 MAC 的代码)。
故"签名不对"只能靠 `license_fail_next` 注入, 而本文件要测的也只是上游那一段判据。
"""
from __future__ import annotations

import struct

import pytest

import litearm as pa
from litearm import _protocol as P
from litearm.errors import CommandRejectedError, InvalidCommandError, err_reason

MAC = bytes(range(16))          # 16B; 桩不验内容, 只验长度


def _frames(arm):
    return arm._tr.tx_log


# ---------------------------------------------------------------------------
# 1. 查询 (`0x2F`)
# ---------------------------------------------------------------------------

def test_license_reads_the_full_record(offline_arm):
    """已激活: 五个字段 + UID 都要读出来。"""
    tr = offline_arm._tr
    tr.license_cust_id, tr.license_issued, tr.license_flags = 1042, 20260917, 0
    lic = offline_arm.license()
    assert isinstance(lic, pa.LicenseInfo)
    assert lic.state == 1 and lic.activated is True
    assert lic.ver == 1
    assert lic.uid == tr.license_uid
    # 签发器要的就是这个形态 —— 逐字节等于设备给的那 12 字节
    assert lic.uid_hex == tr.license_uid.hex() and len(lic.uid_hex) == 24
    assert (lic.cust_id, lic.issued, lic.flags) == (1042, 20260917, 0)
    assert lic.factory_mode is False and lic.state_name == "activated"


def test_unactivated_is_a_state_not_an_error(offline_arm):
    """**未激活不抛异常** —— 它是状态。且**UID 照回**, 其余字段恒 0。

    这两条都直接对应固件行为: 未激活时"除 ENABLE 外一切照常"(售后/产线要能诊断),
    且签发器**必须**能从本应答拿到 UID (否则没法给这台机器签)。
    """
    tr = offline_arm._tr
    tr.activated = False
    tr.license_cust_id, tr.license_issued, tr.license_flags = 999, 20260101, 1
    lic = offline_arm.license()
    assert lic.state == 0 and lic.activated is False
    assert lic.state_name == "not_activated"
    assert lic.uid == tr.license_uid, "未激活也必须回 UID (签发器的唯一来源)"
    assert (lic.cust_id, lic.issued, lic.flags) == (0, 0, 0), (
        "未激活时这三格必须是 0 —— 固件靠'调用方预置零'保证, 桩照抄")


def test_license_is_deliberately_not_wrapped_in_msg(offline_arm):
    """刻意**不**返回 `Msg`（`Msg` 的"刻意不包"清单里有它）—— 别顺手全包。

    理由: `RSP_LICENSE` 没有固件发起的流量, 它的 `hz`/`timestamp` 只会度量
    "调用方自己轮询的频率"。
    """
    assert not isinstance(offline_arm.license(), pa.Msg)


def test_factory_mode_is_the_flags_bit_not_the_state(offline_arm):
    """产线码是 `flags` bit0 ⇒ `state` 报 2; 但它**不**是"激活与否"的判据。"""
    offline_arm._tr.license_flags = 0x1
    lic = offline_arm.license()
    assert lic.state == 2 and lic.state_name == "activated_factory"
    assert lic.activated is True and lic.factory_mode is True
    # 反向: 有产线码但未激活 -> state 仍是 0 (桩与固件同口径)
    offline_arm._tr.activated = False
    lic = offline_arm.license()
    assert lic.state == 0 and lic.activated is False


def test_unknown_state_carries_the_raw_value(offline_arm):
    """认不出的 `state` 码要**带上原值**回, 不许静默折叠。"""
    offline_arm._tr.license_ver = 9
    assert offline_arm.license().state_name == "activated"      # ver 不参与 state
    assert pa.LicenseInfo(state=7, ver=1, uid=b"\x00" * 12, cust_id=0,
                          issued=0, flags=0).state_name == "unknown_state_7"


# ---------------------------------------------------------------------------
# 2. 开机即锁 (`ERR{0x10,0x08}`)
# ---------------------------------------------------------------------------

def test_unactivated_enable_is_rejected_with_the_license_code(offline_arm):
    """未激活时 `enable()` 抛 `ERR{0x10,0x08}`, 且**只发一条**(重发无用)。"""
    offline_arm._tr.activated = False
    with pytest.raises(CommandRejectedError) as ei:
        offline_arm.enable()
    e = ei.value
    assert (e.cmd, e.code) == (0x10, 0x08)
    # 0x08 **不在**重试白名单 (`ENABLE_RETRYABLE_CODES`) ⇒ 不该有第二次尝试
    assert len([f for f in _frames(offline_arm) if f[0] == P.CMD_ENABLE]) == 1, (
        "0x08 被重试了 —— 它是'这台臂没被授权', 重发无用")
    assert "未激活" in str(e) and "activate" in str(e), (
        "报错没把补救路径说清楚 (用户看到的应当是'未激活', 不是'未登记错误码')")


def test_activation_unlocks_enable(offline_arm):
    """对照组: 激活之后同一条 `enable()` 就通了 —— 锁确实挂在 license 上。"""
    tr = offline_arm._tr
    tr.activated = False
    with pytest.raises(CommandRejectedError):
        offline_arm.enable()
    offline_arm.activate(cust_id=7, issued=20260922, mac=MAC)
    offline_arm.enable()
    assert tr.enabled is True


# ---------------------------------------------------------------------------
# 3. 提交 (`0x3F`)
# ---------------------------------------------------------------------------

def test_activate_sends_the_28_byte_payload_and_lands(offline_arm):
    """载荷 = `cust_id LE + issued LE + flags LE + mac[16]`, 且**设备侧真的落位**。"""
    tr = offline_arm._tr
    tr.activated = False
    offline_arm.activate(cust_id=1042, issued=20260917, mac=MAC)
    at = [f for f in _frames(offline_arm) if f[0] == P.CMD_ACTIVATE]
    assert len(at) == 1
    payload = at[0][1]
    assert len(payload) == 28
    assert struct.unpack_from("<III", payload, 0) == (1042, 20260917, 0)
    assert payload[12:] == MAC
    # 落位验证走**读回**, 不是靠 ACK
    lic = offline_arm.license()
    assert lic.activated is True
    assert (lic.cust_id, lic.issued) == (1042, 20260917)


def test_activate_requires_disabled(offline_arm):
    """已武装 -> `ERR{0x3F,0x04}`（与 `save_params` 同语义）。

    ⚠ 这个场景**只能是"已激活"的机器**: 未激活的机器根本使能不了（`ERR{0x10,0x08}`）
    ⇒ 现实里撞上本码的正是"拿同一份凭据再提交一次、而臂正被使能着"。
    门禁顺序也照固件: **武装判据排在"已经激活过"之前**（`usb_cmd.c:1035` 早于
    `license_activate()` 里的 EXISTS 判据）—— 故这台机器回的是 0x04, 不是 0x02。
    """
    tr = offline_arm._tr
    assert tr.activated is True
    offline_arm.enable()
    assert tr.enabled is True
    with pytest.raises(CommandRejectedError) as ei:
        offline_arm.activate(cust_id=1, issued=20260922, mac=MAC)
    assert (ei.value.cmd, ei.value.code) == (0x3F, 0x04)


def test_local_prechecks_reject_before_any_frame_goes_out(offline_arm):
    """两条本地预检 —— 关键是**帧没发**（否则会撞上聚合档 0x02, 归因变模糊）。"""
    for kwargs, needle in (
        (dict(cust_id=1, issued=20260922, flags=0x2, mac=MAC), "flags"),
        (dict(cust_id=1, issued=20260922, mac=MAC[:-1]), "16 字节"),
    ):
        before = len(_frames(offline_arm))
        with pytest.raises(InvalidCommandError) as ei:
            offline_arm.activate(**kwargs)
        assert needle in str(ei.value)
        assert len(_frames(offline_arm)) == before, "本地预检失败却把帧发出去了"


def test_activate_0x02_on_an_already_activated_device_is_not_a_failure(offline_arm):
    """⚠⚠ **本文件最重要的一条**: `0x02` 是聚合档, 不能光看码判失败。

    形态: 上一条 ACK 被丢掉后重发 —— 设备**已经激活**, 故回 `0x02`, 而它其实成功了。
    若照着码号抛异常, 产线会把一台**已经解锁**的机器当成激活失败去返工。
    正确动作 (厂商侧工具同款): 回读 `0x2F`, 看 `state != 0`。
    """
    tr = offline_arm._tr
    assert tr.activated is True                     # 已经是激活态
    offline_arm.activate(cust_id=1042, issued=20260917, mac=MAC)
    # 没抛 => 认出了"其实已经激活"
    at = [f for f in _frames(offline_arm) if f[0] == P.CMD_ACTIVATE]
    lic_queries = [f for f in _frames(offline_arm) if f[0] == P.CMD_GET_LICENSE]
    assert len(at) == 1 and len(lic_queries) >= 1, (
        "0x02 之后没有回读 —— 那就不可能区分'已激活'与'真失败'")


def test_activate_0x02_on_a_genuine_failure_still_raises(offline_arm):
    """**同一档码的对照**: 设备确实没激活时, `0x02` 必须抛 —— 否则就是静默失败。"""
    tr = offline_arm._tr
    tr.activated = False
    tr.license_fail_next = True                     # 模拟 MAC 不符 / 密钥非法 / 写失败
    with pytest.raises(CommandRejectedError) as ei:
        offline_arm.activate(cust_id=1, issued=20260922, mac=MAC)
    assert (ei.value.cmd, ei.value.code) == (0x3F, 0x02)
    assert tr.activated is False
    assert [f for f in _frames(offline_arm) if f[0] == P.CMD_GET_LICENSE], (
        "判'真失败'之前也必须先回读过 —— 两条路径的回读是同一个动作")


# ---------------------------------------------------------------------------
# 4. 错误码文本 (用户真正读到的那句话)
# ---------------------------------------------------------------------------

def test_err_text_names_the_remedy_for_the_two_license_codes():
    """`0x10/0x08` 与 `0x3F/0x02` 的文本必须**可执行**, 而不是"未登记"。

    这两条是用户可见性最强的一对: 前者是"每台未激活的机器一开机就撞"的码,
    后者是最容易被误判成失败的那个聚合档。
    """
    blocked = err_reason(0x10, 0x08)
    assert "未激活" in blocked and "activate" in blocked
    assert "旁路" in blocked, "没写清'不得加旁路'这条规格硬要求"
    agg = err_reason(0x3F, 0x02)
    assert "回查" in agg and "0x2F" in agg
    assert "已经激活" in agg, "没提醒'可能其实已经解锁'这个陷阱"
