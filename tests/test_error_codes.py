"""`(cmd, code)` 语义表 + `enable()` 重试策略 + 解析固件的防漂移哨兵 (spec §6.3)。

三块内容分开看:

1. **表本身** (`errors.ERR_TEXT` / `ERR_CODE_TEXT` / `err_reason`) —— 语义文本、
   未登记的码不静默、异常层级不变;
2. **`enable()` 的重试白名单** (`errors.ENABLE_RETRYABLE_CODES`) —— 锁存故障
   **只发一次**, 不白耗 3.3 秒 (12 次尝试只有 11 次 `sleep(0.3)`); 可重试码照旧重试;
3. **哨兵**: 直接解析固件源码, 断言 SDK 的表**没有漏掉固件任何一档错误码**。

## ⚠⚠ 第 3 块的口径必须先定 (否则这条哨兵写不出来/写成假绿)

固件 `usb_cmd.c` 的 94 处 `usb_cmd_reply(RSP_ERR, …)` 里, **27 处载荷不是纯十六进制
字面量** (一条只抓 `{0x.., 0x..}` 的正则会得到 67 处, 漏 27 处):

* **14 处** 的**命令码是符号名** (`{CMD_MOVE_L, 0x01}` … `{CMD_CART_ADD, 0x04}`) ——
  这 14 处**会被本文件自动解析**: 符号名从固件 `usb_cmd.h` 的 `#define CMD_*` 取值
  (与 `test_protocol_sync.py` 同一套解析)。⚠ 这正是"笛卡尔那 5 条对纯字面量正则
  得到空集"那个坑的解法 —— 不靠白名单, 靠**真的去解析符号**。
* **13 处** 的载荷**无法字面解析** (`{cmd, rc}` / `{0x10, e}` …) —— 其中 **9 处**
  的**错码**是变量, 另 **4 处**的**命令码**是变量 (这 4 处的错码是字面量; 有 1 处
  两者都是变量, 归入前 9 处)。它们的取值集合由固件的收口函数决定, 字面上无从解析。

⚠ `SYMBOLIC_ERR_SITES` 里那 9 处变量错码的**码集合是手工转录**的 —— 逐条对着固件
`control_loop.c` / `cart_exec.c` 的收口函数**函数体**抄下来的, 不是解析出来的。
⇒ 收口函数**新增一个返回码**而**没有新增发射点**时 (例: `ctrl_accept_move_js`
将来多回一个 `0x05`), 本哨兵**不会红** —— 得靠人跟着改清单。这是本口径已知的
精度上限, 不是漏写。

⇒ 本文件采用的**口径 = 计划里的 (B) 「单向 + 显式清单」**, 并做了一处**加强**:
`CMD_*` 符号名自动解析 (于是清单只需覆盖那 13 处**无法字面解析**的位点, 而不是 27 处)。
写死在此处, 免得下一个人以为可以随手改成别的口径:

    · **正向 (自动, 无清单)**: 固件里所有能解析成 `(cmd, code)` 的发射点
      (76 个不同的 `(cmd, code)`, 覆盖 **83 处** = 96 − 13) **必须**在
      `errors.ERR_TEXT` 里
      (唯一例外: `code == 0x00`, 那是 `default` 分支, 见 `test_protocol_sync.py`
      的 `test_err_code_zero_only_comes_from_default_branch`)。
    · **变量码 (清单, 13 条)**: `SYMBOLIC_ERR_SITES` 逐条列举, 每条声明它的
      `(命令集合 × 码集合)`; **清单条数被断言钉死** (见
      `test_symbolic_site_manifest_matches_firmware`), 清单自己漂了也红。
    · **反向闭合 (这是"双向差集"唯一写得出来的形态)**: `ERR_TEXT` 里**不在**上两类
      覆盖范围内的条目 = 必须是**显式列举且计数**的 `ERR_TEXT_OUT_OF_TREE`
      (当前 2 条, 出处是另一条固件分支)。于是"表里凭空多一条固件发不出的码"
      也会红 —— 单靠正向是抓不到的。

⚠ 本哨兵的价值是"**固件新增一档错误码时 SDK 不会静默**"。**别为了让它绿而降级**
(拆掉反向闭合、把清单改成白名单、或对不上就 `pytest.skip`) —— 那是本仓最忌讳的假绿,
`_read()` 里那句"找不到时 skip, 不是 pass"就是同一条纪律。
"""
from __future__ import annotations

import glob
import os
import re
from collections import Counter

import pytest

from litearm import _protocol as P
from litearm import arm as arm_mod
from litearm import errors as E
from litearm import state as ST

FW_DIR = os.environ.get("LITEARM_FW_DIR") or os.path.expanduser("~/litearm-stm32")
_FW_SUBDIR = os.path.join("User", "litearm")
USB_CMD_H = os.path.join(FW_DIR, _FW_SUBDIR, "hal", "usb_cmd.h")


def _read(path: str) -> str:
    if not os.path.isfile(path):
        pytest.skip(f"固件源码不在 {path} —— 设 LITEARM_FW_DIR 指向 litearm-stm32 仓库")
    with open(path, encoding="utf-8", errors="replace") as f:
        return f.read()


# ===========================================================================
# 1. 表本身
# ===========================================================================

def test_table_is_not_empty_and_covers_the_cartesian_range():
    """笛卡尔 `0x3A~0x3E` 的码必须在表里 —— 旧固件工具的那张表**没有**它们,
    正是本次重新穷举的理由 (spec §6.3 的起点)。

    ⚠ `0x3E` (CART_RUN) **没有** `0x01` 档: 它的载荷是**空的**, dispatch 里根本
    没有长度校验 (`usb_cmd.c` 的 `case CMD_CART_RUN` 直接进门禁)。四条有 `0x01`
    的是 `0x3A/0x3B/0x3C/0x3D`。别按"笛卡尔五条同形"去推 —— 它们的载荷形状不同。
    """
    assert len(E.ERR_TEXT) > 90
    for cmd in (0x3A, 0x3B, 0x3C, 0x3D):
        assert (cmd, 0x01) in E.ERR_TEXT, f"笛卡尔 0x{cmd:02X} 的载荷长度档未登记"
    assert (0x3E, 0x01) not in E.ERR_TEXT, "CART_RUN 载荷为空, 不存在长度不足档"


#: 会因 drop_hold 而回 `0x06` 的命令。⚠ **`0x02` MOVE_P 不在其中** —— 它的 dispatch
#: 直接问 `kin_runner_request_move_p` (后台 IK), 三档只有 `!enabled`(0x03) /
#: 零重力(0x04) / 忙(0x05), **没有** drop_hold 门禁 (固件 `usb_cmd.c` 的
#: `case CMD_MOVE_P`)。它是运动类命令里唯一的例外 —— 别按"运动类都判"去推。
DROP_HOLD_CMD_CODES = (0x01, 0x03, 0x04, 0x05, 0x07, 0x2A,
                       0x3A, 0x3B, 0x3C, 0x3D, 0x3E)


def test_the_two_previously_untabulated_entries_are_present():
    """S4 fix 的两条"今天不在任何表里"的码 (计划 Step 5 点名)。"""
    assert (0x25, 0x03) in E.ERR_TEXT, "异步保存失败码 {0x25,0x03} 未登记"
    for cmd in DROP_HOLD_CMD_CODES:
        assert (cmd, 0x06) in E.ERR_TEXT, f"{{0x{cmd:02X},0x06}} (drop_hold 门禁) 未登记"
    assert (0x02, 0x06) not in E.ERR_TEXT, "MOVE_P 无 drop_hold 门禁, 不该有 0x06"


@pytest.mark.parametrize("cmd", DROP_HOLD_CMD_CODES)
def test_code_0x06_on_motion_commands_means_drop_hold(cmd):
    """`0x06` 在**运动类**命令上恒等于 drop_hold 锁存 (`g_arm.drop_hold`) ——
    这是 SDK 唯一能**确证**该锁存的信号 (见 `state.drop_hold_inferred` 的文档)。

    ⚠ 但 `0x10` ENABLE 的 `0x06` **不含** drop_hold 判据 (它判 EMERGENCY /
    `joint_fault`, 出处 `control_loop.c` 的 `ctrl_enable`), 所以下面单独断言它
    **不得**在文本里提 drop_hold —— 混了会把"该去 reset"引向错的排查方向。
    """
    assert "drop_hold" in E.ERR_TEXT[(cmd, 0x06)]


def test_enable_code_0x06_does_not_claim_drop_hold():
    assert "drop_hold" not in E.ERR_TEXT[(0x10, 0x06)]


def test_no_code_zero_entry_anywhere():
    """`code == 0x00` 恒为"固件没有这条命令" (唯一真源是 `default` 分支) ——
    登记进具体档会让它变成"某条命令的一个普通错误码", 破坏能力探测的判定。"""
    assert not [k for k in E.ERR_TEXT if k[1] == 0x00]


def test_unregistered_code_is_not_silent():
    """未登记的码必须带上**原始 cmd/code** —— 这是固件新增一档时唯一会让上位机
    看见"我没见过这个码"的地方。"""
    txt = E.err_reason(0x3A, 0x55)
    assert "0x3A" in txt and "0x55" in txt
    assert "未登记" in txt


def test_generic_fallback_still_carries_the_raw_code():
    """通用档命中时同样要带原始码 (计划: "未登记的码不许静默")。"""
    txt = E.err_reason(0x3C, 0x05)          # 0x05 有通用档, 但 (0x3C,0x05) 没登记
    assert "忙" in txt and "0x3C" in txt and "0x05" in txt


def test_every_entry_has_nonempty_text_and_int_keys():
    for (cmd, code), txt in E.ERR_TEXT.items():
        assert isinstance(cmd, int) and isinstance(code, int)
        assert isinstance(txt, str) and txt.strip(), f"({cmd:#04x},{code:#04x}) 文本为空"


def test_enable_retryable_codes_are_registered():
    for code in E.ENABLE_RETRYABLE_CODES:
        assert (0x10, code) in E.ERR_TEXT, f"重试白名单里的 0x{code:02X} 未登记"


def test_exception_hierarchy_is_unchanged():
    """计划: **不新增异常类型**, `CommandRejectedError` 仍是基类 (spec §6.3)。"""
    assert issubclass(E.UnsupportedByFirmwareError, E.CommandRejectedError)
    assert issubclass(E.CommandRejectedError, E.LiteArmError)
    # 两条"刻意不继承"的既有约定, 一并钉住 (它们上面都写了很长的理由)
    assert not issubclass(E.CartesianPlanError, E.CommandRejectedError)
    assert not issubclass(E.MotionSupersededError, E.CartesianPlanError)


def test_err_from_carries_code_and_readable_text():
    """`_err_from` 是 `CommandRejectedError` 的唯一生产者 —— 它必须同时给出
    可编程判定的 `.code` 与可读文本。"""
    e = arm_mod._err_from(bytes([0x10, 0x06]), "enable(1) 被固件拒绝: ")
    assert isinstance(e, E.CommandRejectedError)
    assert (e.cmd, e.code) == (0x10, 0x06)
    assert "[10,6]" in str(e)                    # 原始码 (固件 ERR 回显的两位十六进制)
    assert "锁存" in str(e) and "RESET" in str(e)


def test_err_from_keeps_the_unsupported_by_firmware_subclass():
    """`code == 0x00` 走的仍是子类 (能力探测依赖它), 且**不进**具体档。"""
    e = arm_mod._err_from(bytes([0x15, 0x00]), "")
    assert isinstance(e, E.UnsupportedByFirmwareError)
    assert (e.cmd, e.code) == (0x15, 0x00)


# ===========================================================================
# 2. enable() 区分"可重试"与"锁存"
# ===========================================================================

def _enable_sends(arm) -> int:
    return len(arm._tr.stamps_of(P.CMD_ENABLE))


def test_enable_latched_code_sends_once_and_does_not_burn_the_retry_budget(
        offline_arm, monkeypatch):
    """`(0x10, 0x06)` = 锁存, 须先 RESET ⇒ **立刻抛, 别重试**。

    判据两条 (第二条是"不耗满预算"的直接观测): 只发**一次**;
    且**一次 `sleep(0.3)` 都没有发生** —— 12 次预算 = 11 × 0.3 = 3.3 秒, 全是白耗。
    """
    arm = offline_arm
    arm._tr.err_override[P.CMD_ENABLE] = 0x06
    sleeps = []
    monkeypatch.setattr("time.sleep", lambda s: sleeps.append(s))

    before = _enable_sends(arm)
    with pytest.raises(E.CommandRejectedError) as ei:
        arm.enable()                        # 默认 attempts=12, 一次都不该用满
    assert (ei.value.cmd, ei.value.code) == (P.CMD_ENABLE, 0x06)
    assert _enable_sends(arm) - before == 1
    assert sleeps == [], f"锁存码之后还发生了 {len(sleeps)} 次重试等待"
    assert isinstance(ei.value, E.CommandRejectedError)   # 层级不变


@pytest.mark.parametrize("code", [0x07])
def test_enable_other_non_retryable_codes_also_send_once(offline_arm, monkeypatch, code):
    """`0x07` (CMODE 补写预算耗尽, 固件原注释: "重发无用, 须现场排查") 与 `0x06`
    是同一类 —— 白名单口径 (`ENABLE_RETRYABLE_CODES`) 让它们都 fail-fast。"""
    arm = offline_arm
    arm._tr.err_override[P.CMD_ENABLE] = code
    monkeypatch.setattr("time.sleep", lambda s: None)
    before = _enable_sends(arm)
    with pytest.raises(E.CommandRejectedError) as ei:
        arm.enable()
    assert ei.value.code == code
    assert _enable_sends(arm) - before == 1


def test_enable_retryable_code_still_retries(offline_arm, monkeypatch):
    """`(0x10, 0x03)` = 可重试 (反馈未齐 / CMODE 首写) ⇒ **照旧重试**。"""
    arm = offline_arm
    arm._tr.err_override[P.CMD_ENABLE] = 0x03
    sleeps = []
    monkeypatch.setattr("time.sleep", lambda s: sleeps.append(s))

    before = _enable_sends(arm)
    with pytest.raises(E.CommandRejectedError) as ei:
        arm.enable(attempts=3)
    assert ei.value.code == 0x03
    assert _enable_sends(arm) - before == 3
    assert sleeps == [0.3, 0.3]


def test_enable_retries_then_succeeds(offline_arm, monkeypatch):
    """0x03 之后放行 -> 重试确实能走到 ACK (证明白名单没把正常首使能路径堵死)。

    固件真机时序就是 "ENABLE#1 -> 0x03(首写 CMODE), ENABLE#2 -> ACK"。
    """
    arm = offline_arm
    tr = arm._tr
    tr.err_override[P.CMD_ENABLE] = 0x03
    orig = tr.write_frame

    def wf(cmd, payload=b""):
        orig(cmd, payload)
        if cmd == P.CMD_ENABLE:
            tr.err_override.pop(P.CMD_ENABLE, None)      # 只拒第一次

    monkeypatch.setattr(tr, "write_frame", wf)
    monkeypatch.setattr("time.sleep", lambda s: None)

    before = _enable_sends(arm)
    arm.enable(attempts=3)                                # 不抛
    assert _enable_sends(arm) - before == 2


def test_enable_missing_command_fails_fast_as_unsupported(offline_arm, monkeypatch):
    """固件没有 `0x10` 时 (`ERR{0x10,0x00}`) 必须**立刻**抛
    `UnsupportedByFirmwareError` —— 重试 12 次只会把"固件太旧"这个结论推迟 3.3 秒。"""
    arm = offline_arm
    arm._tr.unknown_cmds.add(P.CMD_ENABLE)
    monkeypatch.setattr("time.sleep", lambda s: None)
    before = _enable_sends(arm)
    with pytest.raises(E.UnsupportedByFirmwareError):
        arm.enable()
    assert _enable_sends(arm) - before == 1


# ===========================================================================
# 3. drop_hold 的具名暴露 (⚠ 固件不上报它, 只能是推断)
# ===========================================================================

def test_drop_hold_inferred_tracks_joint_fault():
    assert ST.RobotState(joint_fault=0).drop_hold_inferred is False
    assert ST.RobotState(joint_fault=1 << 3).drop_hold_inferred is True
    assert ST.RobotState(joint_fault=0x0008).drop_hold_inferred is True


def test_drop_hold_is_not_exposed_as_a_directly_read_bit():
    """⚠ "不许假装能直接读到" (spec §6.3) —— 状态帧里**没有** drop_hold 的位。

    具名的那个属性必须带 `_inferred` 后缀, 免得调用方以为它来自状态帧。
    """
    assert not hasattr(ST.RobotState, "drop_hold")
    assert hasattr(ST.RobotState, "drop_hold_inferred")


def test_doc_warns_that_inference_is_one_sided():
    """**文档存在性守卫, 不计入判据强度** —— 它只查 `drop_hold_inferred` 的 docstring
    里还在不在那三个词 (`单向` / `joint_fault` / `不上报`), 不查行为。

    ⚠ 它**两侧都能骗人**, 这是刻意的取舍, 别把它当成"方向正确"的证明:

    * **能误报**: 把同一句话**等价改写** (例如"该推断不可逆"), 词面一去掉就红 ——
      文档没变坏, 但用例红了;
    * **能漏报**: 把这句话写成**反向的错话** (例如"`joint_fault != 0` ⇒
      `drop_hold` 必为真"), 只要还没删掉那三个词, 这里照样绿。

    ⇒ **方向本身由行为用例覆盖**, 不靠这一条: 真值表在同文件的
    `test_drop_hold_inferred_tracks_joint_fault` (`joint_fault == 0 -> False`、
    `0x0008 -> True`, 属性实现就是 `bool(self.joint_fault)`, `state.py:125`)。
    本用例的唯一职责是"那段警告没被顺手删掉", 所以它
    用**最松**的判据 (三个子串) 避免自己变成维护负担 —— 也正因如此, 它的绿
    不构成任何方向性结论。
    """
    txt = ST.RobotState.drop_hold_inferred.__doc__
    assert "单向" in txt and "joint_fault" in txt and "不上报" in txt


# ===========================================================================
# 4. 哨兵 —— 解析固件, 防 (cmd, code) 表漂移
# ===========================================================================

#: 固件 `ERR` 发射点: `usb_cmd_reply(RSP_ERR, (const uint8_t[]){<cmd>, <code>}, 2)`。
#: 用 `[^}]*` 取花括号里的载荷 (载荷里不含 `}`)。
#: ⚠ 函数名到 `,` 之间用 `\s*` 容错 —— 好让下面那条"裸调用头 vs 本正则"的核对
#:   只对**载荷形态**敏感, 不对**换行缩进**敏感 (否则有人把参数折行就会假红)。
_ERR_SITE = re.compile(
    r"usb_cmd_reply\s*\(\s*RSP_ERR\s*,\s*\(const uint8_t\[\]\)\{([^}]*)\}")

#: 同一个发射**函数名**的"裸调用头"—— 只看调用, 不看载荷形态。用途见
#: `_firmware_err_sites` 里的逐文件核对: 把"`usb_cmd_reply(RSP_ERR` 出现过几次"与
#: "`_ERR_SITE` 命中几次"钉成相等, 于是**换一种载荷发射形态**时哨兵不会静默。
#: 实测今天这棵树上两者是 94 + 2, 一个不多一个不少。
_ERR_CALL_HEAD = re.compile(r"usb_cmd_reply\s*\(\s*RSP_ERR")
_HEX = re.compile(r"^0x[0-9A-Fa-f]+$")

#: 13 条**变量码**符号位点 (清单, 见模块 docstring 的口径说明)。
#: key = `(相对固件仓的路径, cmd 表达式, code 表达式)`;
#: value = `(可达的 (cmd, code) 对集合, 说明)` —— 命令位写 `None` 表示"任意命令"
#: (只有 `default` 分支的 `{cmd, 0x00}` 是这样)。
#:
#: ⚠ 声明的是**对集合**而不是"命令集合 × 码集合"的笛卡尔积: `cart_reply` 的五个
#:   调用点可达的码**不同** (`cart_req_run` 只会回 `0x04`, 从不回 `0x02`), 笛卡尔积
#:   会凭空多声明 `(0x3E, 0x02)` —— 那会让
#:   `test_manifest_declared_codes_are_registered` 报一条假缺陷。这正是
#:   "口径不定就写不出来"的实例: 精度只能做到**发射点**这一层, 再细化就得解析
#:   `cart_exec.c` 的函数体了。
#: ⚠ 上面那句"再细化就得解析函数体"的另一面, 就是本清单的**精度上限**:
#:   这几个 `pairs` 是**手工转录**函数体里的返回码得到的 (不是算出来的) ⇒
#:   收口函数**新增一个码而不新增发射点**时 (例: `ctrl_accept_move_js` 将来多回一个
#:   `0x05`), 固件树里的发射点一行没变 ⇒ `symbolic` 多重集不变 ⇒ **哨兵不红**。
#:   只有**多一个调用点/多一处载荷**才会红。加码请人工跟这张清单。
#: ⚠ 这份清单**必须与固件逐条对齐**: 条数与内容都被
#: `test_symbolic_site_manifest_matches_firmware` 钉住。
_USB_CMD_C = os.path.join(_FW_SUBDIR, "hal", "usb_cmd.c")
SYMBOLIC_ERR_SITES = {
    (_USB_CMD_C, "cmd", "0x03"):
        (frozenset({(0x3A, 0x03), (0x3B, 0x03), (0x3E, 0x03)}),
         "cart_gate_ok: 未使能 —— 三个门禁调用点 (MOVE_L/MOVE_C/CART_RUN)"),
    (_USB_CMD_C, "cmd", "0x06"):
        (frozenset({(0x3A, 0x06), (0x3B, 0x06), (0x3E, 0x06)}),
         "cart_gate_ok: 掉线刚性持位锁存 (drop_hold) —— [S4 fix]"),
    (_USB_CMD_C, "cmd", "0x04"):
        (frozenset({(0x3A, 0x04), (0x3B, 0x04), (0x3E, 0x04)}),
         "cart_gate_ok: 零重力(拖动示教)中"),
    (_USB_CMD_C, "cmd", "rc"):
        (frozenset({(0x3A, 0x04), (0x3B, 0x04),          # movel/movec: 只有 STATE
                    (0x3C, 0x02), (0x3C, 0x04),          # begin: BADARG(n 越界) / STATE
                    (0x3D, 0x02), (0x3D, 0x04),          # add:   BADARG(idx 越界) / STATE
                    (0x3E, 0x04)}),                      # run:   只有 STATE (点名不满)
         "cart_reply: 透传 cart_req_* 的 rc (cart_exec.h: CART_REQ_BADARG=0x02 / "
         "CART_REQ_STATE=0x04) —— 五个调用点, 可达码各不相同"),
    (_USB_CMD_C, "0x01", "mj"):
        (frozenset({(0x01, 0x03), (0x01, 0x04), (0x01, 0x06)}),
         "ctrl_accept_move_j 的返回码"),
    (_USB_CMD_C, "0x07", "ms"):
        (frozenset({(0x07, 0x03), (0x07, 0x04), (0x07, 0x06)}),
         "ctrl_accept_move_j_sync 的返回码 (与 0x01 同表)"),
    (_USB_CMD_C, "0x2A", "hr"):
        (frozenset({(0x2A, 0x03), (0x2A, 0x04), (0x2A, 0x06)}),
         "home 复用 ctrl_accept_move_j 的返回码 ([Z0 fix]: 原恒回 ACK)"),
    (_USB_CMD_C, "0x03", "rjs"):
        (frozenset({(0x03, 0x03), (0x03, 0x04), (0x03, 0x06)}),
         "ctrl_accept_move_js 的返回码"),
    (_USB_CMD_C, "0x04", "rmit"):
        (frozenset({(0x04, 0x03), (0x04, 0x04), (0x04, 0x06)}),
         "ctrl_accept_move_mit 的返回码"),
    (_USB_CMD_C, "0x05", "rma"):
        (frozenset({(0x05, 0x03), (0x05, 0x04), (0x05, 0x06)}),
         "ctrl_accept_move_mit_all 的返回码"),
    (_USB_CMD_C, "0x10", "e"):
        (frozenset({(0x10, 0x03), (0x10, 0x06), (0x10, 0x07), (0x10, 0x08)}),
         "ctrl_enable 的返回码 (control_api.h: 0x03 可重试 / 0x06 锁存须 reset / "
         "0x07 CMODE 补写预算耗尽 / 0x08 未激活 —— 固件 1.8.0 起 `ctrl_enable()` 的"
         "**第一条**判据 (`control_loop.c:944`) 就是 `!license_is_activated()`)"),
    (_USB_CMD_C, "0x15", "r"):
        (frozenset({(0x15, 0x02), (0x15, 0x03)}),
         "ctrl_request_enter_dfu 的返回码 (0x02 ROM 向量表无效 / 0x03 使能中)"),
    (_USB_CMD_C, "cmd", "0x00"):
        (frozenset({(None, 0x00)}),
         "default 分支: 固件没有这条命令 (唯一一处 0x00, 能力探测的哨兵)"),
}

#: 清单自身的**条数** —— 断言它, 免得清单被"顺手"改小而不自知。
SYMBOLIC_ERR_SITES_COUNT = 13

#: `ERR_TEXT` 里**不在本固件树产出范围内**、但刻意保留的条目 (spec R5):
#: 出处是 `origin/feat/hyy-model-import` (1.5.3 `cdb744a` 不是 master 祖先),
#: 那里 `0x26 item7` / `0x28 item6` 在武装态回 `0x04`。留着是为了产线若烧那条
#: 分支时 SDK 的解读不落回通用档。**显式列举 + 计数**, 不是一张可以顺手塞东西的
#: 白名单 —— 加条目就要一起改下面的计数。
ERR_TEXT_OUT_OF_TREE = frozenset({(0x26, 0x04), (0x28, 0x04)})
ERR_TEXT_OUT_OF_TREE_COUNT = 2


def _cmd_defines(hdr_src: str):
    """`#define CMD_NAME 0xNN` -> {名字: 值} (与 test_protocol_sync.py 同一套)。"""
    pat = r"^#define[ \t]+(CMD_\w+)[ \t]+(0x[0-9A-Fa-f]+)[ \t]*(.*)$"
    return {m.group(1): int(m.group(2), 16) for m in re.finditer(pat, hdr_src, re.M)}


def _firmware_err_sites():
    """扫描固件 `User/litearm` 树 (不只 `usb_cmd.c`) 的**全部** `RSP_ERR` 发射点。

    ⚠ 必须整树扫: `kin_runner.c` 里也有 2 处 (`0x02, 0x03` 的后台 IK 失败) ——
    只扫 `usb_cmd.c` 会漏掉它们。返回 `[(相对路径, cmd 表达式, code 表达式), …]`。

    ⚠ **逐文件核对: 裸调用头数 == `_ERR_SITE` 命中数**。
    `_ERR_SITE` 只认**花括号字面量**这一种发射形态; 换成"先声明数组再传指针"
    (`uint8_t eb[2]={…}; usb_cmd_reply(RSP_ERR, eb, 2);`) 就整棵树扫不到。
    这棵固件树今天 96 处**恰好全是**花括号形态 ⇒ 是"未来形态变化"的洞, 不是现存缺陷。
    ⚠ 而且这种漏**不一定**会被既有的正/反向判据接住 —— 实测: 在 `kin_runner.c`
    把一处 `(const uint8_t[])` 的 `const` 去掉 (语义等价的形态变化), 因为
    `(0x02, 0x03)` 另有两处在发, 正向与反向闭合**都照样绿**,
    **旧代码 36 passed 全静默**。下面这行是唯一的兜底: 让"换形态"当场变红。

    ⚠ 它的口径上限是"只认 `usb_cmd_reply` 这**一个函数名**": 固件若新增另一个发射
    函数 (或把发射收进 helper), 裸调用头与 `_ERR_SITE` **两侧都看不见**。
    ⚠ 这个洞**已封**, 但**不是**靠放宽本函数的正则 (放宽成"任意函数名 + `RSP_ERR`"
    会大量误伤: 注释、`==` 比较、`#define` 行都会命中) —— 而是由下面**另一条窄判据**
    `test_only_usb_cmd_reply_receives_rsp_err` 回答"还有没有别人在收这个常量"。
    两条的分工: **本函数管载荷形态漂移, 那条管发射面漂移**。
    """
    sites = []
    root = os.path.join(FW_DIR, _FW_SUBDIR)
    if not os.path.isdir(root):
        pytest.skip(f"固件源码树不在 {root} —— 设 LITEARM_FW_DIR")
    for path in sorted(glob.glob(os.path.join(root, "**", "*.c"), recursive=True)):
        src = open(path, encoding="utf-8", errors="replace").read()
        rel = os.path.relpath(path, FW_DIR)
        hits = _ERR_SITE.findall(src)
        n_head = len(_ERR_CALL_HEAD.findall(src))
        assert n_head == len(hits), (
            f"{rel}: 看到 {n_head} 处 `usb_cmd_reply(RSP_ERR` 调用, 而 `_ERR_SITE` "
            f"只解析出 {len(hits)} 处 —— 有 {n_head - len(hits)} 处换了发射形态 "
            f"(不是 `(const uint8_t[]{{…}})` 花括号载荷), 哨兵看不见它们。"
            f" 要么把 `_ERR_SITE` 改成能认这种形态, 要么把该处写回花括号形态;"
            f" **别把本断言删掉** —— 少了它, 换形态的位点会静默少算"
            f" (实测: 旧代码对 `kin_runner.c` 的同型变异 36 passed 全静默)。")
        for m in _ERR_SITE.finditer(src):
            parts = [x.strip() for x in m.group(1).split(",")]
            assert len(parts) == 2, f"{rel}: 载荷不是 [cmd, code] 两段: {m.group(1)!r}"
            sites.append((rel, parts[0], parts[1]))
    return sites


#: C 里会引导一个**括号**、但**不是**"接收 `RSP_ERR` 的函数"的关键字。
#: 例: `if (rsp == RSP_ERR)` —— 那个 `(` 同样是"最内层未闭合的括号", 不排掉就会把
#: 接收者读成 `if` (一条假红, 而且名字看着莫名其妙)。
#: ⚠ 今天的固件树里 **0 命中** —— 这条是给**未来写法**留的 (今天全部 96 处都是直接
#: 调用 `usb_cmd_reply(...)`), 不是现存缺陷的补丁。
_NOT_A_CALLER = frozenset({
    "if", "while", "for", "switch", "return", "sizeof", "do", "else", "case",
})


def _strip_noncode(src: str) -> str:
    """把 C 注释与字符串/字符字面量换成**等长空白** —— 只留代码, 且下标与原文对齐。

    为什么必须去: 本判据问的是"谁在**代码里**接收 `RSP_ERR`", 而固件树里 `RSP_ERR`
    还出现在**注释**中 (`usb_cmd.c` 里那句 0x00 约定说明、`kin_runner.c` 的 include
    注释) —— 不去注释就得逐处判"这段是不是注释", 那比去注释本身更容易写错。
    等长替换 (**不是删除**) 的两个理由: 报错能指回原文行号; 不会把相邻 token 粘成
    一个 (`a/*x*/b` 不能变成 `ab`)。
    """
    out, i, n = [], 0, len(src)
    while i < n:
        c = src[i]
        if c == "/" and i + 1 < n and src[i + 1] == "*":
            j = src.find("*/", i + 2)
            j = n if j < 0 else j + 2
        elif c == "/" and i + 1 < n and src[i + 1] == "/":
            j = src.find("\n", i)
            j = n if j < 0 else j
        elif c in "\"'":
            j = i + 1
            while j < n and src[j] != c and src[j] != "\n":
                j += 2 if src[j] == "\\" else 1
            j = min(j + 1, n)
        else:
            out.append(c)
            i += 1
            continue
        out.append(re.sub(r"[^\n]", " ", src[i:j]))
        i = j
    return "".join(out)


def _err_receiver_owners(code_src: str):
    """`RSP_ERR` 的**接收者** → 计数; 外加"没有接收者"的下标表 (裸引用)。

    接收者 = **最内层未闭合的那个 `(`** 前面的标识符, 即"谁在调它、把这个常量传进去"。
    向后扫 (遇到 `)` 记一层, `(` 且深度为 0 即本层) 是为了正确处理
    `usb_cmd_reply(RSP_ERR, (const uint8_t[]){cmd, code})` 这种**载荷里还有括号**的形态。
    解析不出接收者的 (`rsp == RSP_ERR` 这类裸引用、关键字引导的括号) 进第二个返回值
    —— 它们**不构成发射面**, 但要在失败信息里列出来, 免得读的人以为"扫到了却没人收"。
    """
    owners, bare = Counter(), []
    for m in re.finditer(r"\bRSP_ERR\b", code_src):
        depth, i = 0, m.start() - 1
        while i >= 0:
            if code_src[i] == ")":
                depth += 1
            elif code_src[i] == "(":
                if depth == 0:
                    break
                depth -= 1
            i -= 1
        if i < 0:
            bare.append(m.start())
            continue
        j = i - 1
        while j >= 0 and code_src[j] in " \t\r\n":
            j -= 1
        k = j
        while k >= 0 and (code_src[k].isalnum() or code_src[k] == "_"):
            k -= 1
        name = code_src[k + 1:j + 1]
        if not name or name in _NOT_A_CALLER:
            bare.append(m.start())
        else:
            owners[name] += 1
    return owners, bare


def test_only_usb_cmd_reply_receives_rsp_err(fw_err_sites):
    """⚠ **发射面**哨兵: 固件 `.c` 里接收 `RSP_ERR` 的函数名集合**恰好** `{usb_cmd_reply}`。

    为什么另设一条, 而不是把 `_ERR_SITE` 的正则放宽: 放宽成"任意函数名 + `RSP_ERR`"
    会大量误伤 (注释 / `==` 比较 / `#define` 行都会命中), 而**窄判据**只回答一个问题:
    **还有没有别人在收这个常量** —— 这正是 `_ERR_CALL_HEAD` 那条兜底**封不住**的洞
    (固件新增第二个发射函数、或把发射收进 helper 时, 裸调用头与 `_ERR_SITE` 两侧都
    看不见, 见 `_firmware_err_sites` 的 docstring)。

    **口径** (实测于 2026-09-19, 与 `_firmware_err_sites` **同一个根** `User/litearm`):

    * 扫 `User/litearm` 整棵树的 `.c` (**不扫全仓**, 两个理由: 全仓会连带扫到 `tools/`
      —— 那是**宿主编译**的检查工具, 把 `RSP_ERR` 当**值**比较是它的正常用法
      (`tools/s4_drop_hold_gate_check_host.c:218`); 以及 `.claude/worktrees/`
      —— 那是**另一条分支的检出**, 不是本树);
    * 只算**调用点**: 先 `_strip_noncode` 去注释/字面量, 再取"最内层括号前面的标识符";
      裸引用与注释里的提及**都不算**"接收" (故 `usb_cmd.c` 里那句
      `RSP_ERR{*,0x00}` 的约定说明不会命中);
    * 今天: **96 处**接收点 (94 处在 `usb_cmd.c` + 2 处在 `kin_runner.c`), **全部**属主是
      `usb_cmd_reply`, 裸引用 **0** 处 —— 集合恰好单元素。

    ⚠ **已知边界** (留底, 别读成"全仓都封了"): 只在 `User/litearm` **之外**的 `.c` 里
    新增发射函数 (例: `Core/Src/main.c`) 时本判据看不见。截至今天全仓
    `grep -rln RSP_ERR --include=*.c` 只有 3 个文件, 树外那个是**比较**不是发射。

    ⚠ 判据能失败: 在 `/tmp` 的固件副本里给任一 `.c` 加一句
    `usb_cmd_reply2(RSP_ERR, …)` 就红 (实测过); 集合变了就红, 不看条数。
    """
    root = os.path.join(FW_DIR, _FW_SUBDIR)
    if not os.path.isdir(root):
        pytest.skip(f"固件源码树不在 {root} —— 设 LITEARM_FW_DIR")
    owners, bare, n_files = Counter(), 0, 0
    for path in sorted(glob.glob(os.path.join(root, "**", "*.c"), recursive=True)):
        n_files += 1
        o, b = _err_receiver_owners(_strip_noncode(_read(path)))
        owners.update(o)
        bare += len(b)
    assert n_files, f"{root} 下一个 .c 都没有 —— 路径坏了"
    # ⚠ 空转防护与上一条**同源但会分流归因**: 解析器坏掉时读数是 0, 那时"集合 != {…}"
    #   也能红, 但红的原因会被读成"固件里多了个发射函数"; 这条把两者分开。
    # ⚠⚠ 判据是"**与本判据自己的 `_ERR_SITE` 命中数相等**", **不是**"≥ 一个硬编码下界":
    #   实测把扫描面退回 `usb_cmd.c` (94 处) 时 `94 >= 90` **照样通过** ⇒ 口径漂了而守卫
    #   不响。两个解析器是**相互独立**的 (`_err_receiver_owners` 去注释/字面量后取最内层
    #   括号前的标识符; `_ERR_SITE` 只认花括号字面量那一形态), 各自覆盖**同一棵树**
    #   ⇒ 数不相等就说明至少有一侧的口径漂了, 该当场红。
    assert sum(owners.values()) == len(fw_err_sites[1]), (
        f"本判据解析出 {sum(owners.values())} 个 `RSP_ERR` 接收点, 而 `_ERR_SITE` 在同一棵"
        f"树上命中 {len(fw_err_sites[1])} 处 —— 两侧口径已经不一致 (扫描面/解析器漂了), "
        f"别让本判据空转成假绿")
    offenders = sorted(n for n in owners if n != "usb_cmd_reply")
    assert not offenders, (
        f"固件里有 `usb_cmd_reply` 之外的函数在接收 `RSP_ERR`: {offenders} "
        f"(逐处: {dict(owners)}; 另有 {bare} 处裸引用/非调用形态, 不计入接收面) —— "
        f"若它是**新的发射面**, 那么 `_ERR_SITE`/`_ERR_CALL_HEAD` 那套**看不见它** "
        f"(它们只认 `usb_cmd_reply` 一个名字), 本判据正是为它设的; 别直接删本用例。")


def _split_sites(sites, defines):
    """发射点 -> (字面量 `(cmd,code)` 集合, 变量位点 Counter)。

    命令码优先按固件 `usb_cmd.h` 的 `CMD_*` 符号名解析 (那 14 处笛卡尔/收口点),
    故"字面量"的定义是**可解析**, 不是"纯十六进制"。解析不出的进第二桶。
    """
    literal, symbolic = set(), Counter()
    for rel, cmd_expr, code_expr in sites:
        if cmd_expr in defines:
            cmd = defines[cmd_expr]
        elif _HEX.match(cmd_expr):
            cmd = int(cmd_expr, 16)
        else:
            symbolic[(rel, cmd_expr, code_expr)] += 1
            continue
        if not _HEX.match(code_expr):
            symbolic[(rel, cmd_expr, code_expr)] += 1
            continue
        literal.add((cmd, int(code_expr, 16)))
    return literal, symbolic


@pytest.fixture(scope="module")
def fw_err_sites():
    defines = _cmd_defines(_read(USB_CMD_H))
    return defines, _firmware_err_sites()


def test_every_firmware_literal_err_code_is_registered(fw_err_sites):
    """**正向**: 固件每一个可解析的 `(cmd, code)` 都要在 `errors.ERR_TEXT` 里。

    固件**新增一个字面量错误码**而 SDK 没跟 -> 这里红 (这正是"不会静默"的下半句)。
    `code == 0x00` 例外: 它是 `default` 分支的"固件无此命令", 不进具体档。
    """
    defines, sites = fw_err_sites
    literal, _ = _split_sites(sites, defines)
    assert len(literal) > 60, "解析到的字面量码太少 —— 解析器大概坏了 (别让哨兵空转)"
    missing = sorted((c, k) for (c, k) in literal if k != 0x00 and (c, k) not in E.ERR_TEXT)
    assert not missing, (
        "固件里有、SDK 的 (cmd,code) 表里没有的码: "
        + ", ".join(f"0x{c:02X}/0x{k:02X}" for c, k in missing))


def test_symbolic_site_manifest_matches_firmware(fw_err_sites):
    """**清单**: 13 条变量码位点必须与固件**逐条**一致 (多重集比对, 不是子集)。

    ⚠ 用 `Counter` 而不是 `set`: 同一段载荷文本若在**新的**位置再出现一份
    (例如又加一个 `{cmd, 0x03}`), 集合比对会看不见, 多重集比对会红。
    """
    defines, sites = fw_err_sites
    _, symbolic = _split_sites(sites, defines)
    assert len(SYMBOLIC_ERR_SITES) == SYMBOLIC_ERR_SITES_COUNT, (
        f"清单条数被改成 {len(SYMBOLIC_ERR_SITES)}, 而钉死的是 {SYMBOLIC_ERR_SITES_COUNT}"
        f" —— 改条数要连这个常量一起改 (它是故意的摩擦, 不是笔误)")
    assert set(symbolic) == set(SYMBOLIC_ERR_SITES), (
        "变量码位点清单与固件对不上:\n  固件有而清单没有: "
        f"{sorted(set(symbolic) - set(SYMBOLIC_ERR_SITES))}\n  清单有而固件没有: "
        f"{sorted(set(SYMBOLIC_ERR_SITES) - set(symbolic))}")
    assert symbolic == Counter({k: 1 for k in SYMBOLIC_ERR_SITES}), (
        f"某个变量码位点在固件里出现次数变了: {sorted(symbolic.items())}")


def test_manifest_declared_codes_are_registered(fw_err_sites):
    """清单里声明的每个 `(cmd, code)` 对都必须在 `ERR_TEXT` 里 ——
    清单说"某收口函数会回这个码", 表就必须认识它。"""
    problems = []
    for (rel, cmd_expr, code_expr), (pairs, why) in SYMBOLIC_ERR_SITES.items():
        assert pairs, f"{rel}:{cmd_expr}/{code_expr} 声明了空码集"
        for cmd, code in pairs:
            if cmd is not None and code != 0x00 and (cmd, code) not in E.ERR_TEXT:
                problems.append(f"0x{cmd:02X}/0x{code:02X} ({why})")
    assert not problems, f"清单声明了表里没有的码: {problems}"


def test_err_text_has_no_entry_the_firmware_cannot_produce(fw_err_sites):
    """**反向闭合** —— 这条是"双向差集"里 SDK→固件 的那一半, 且它是写得出来的。

    `ERR_TEXT` 的每一条都必须能被下面三者之一解释:
      ① 固件里作为一个**字面量**出现 (`{0x23, 0x03}`);
      ② 由**清单**里某条变量位点产出 (`cmds × codes`);
      ③ 显式列举的树外条目 `ERR_TEXT_OUT_OF_TREE` (另一条固件分支)。

    否则 = **表里凭空多了一条固件发不出的码** (典型成因: 照着旧分支的表抄、
    或固件删了某个码而 SDK 没跟) —— 纯正向判据**抓不到**这一类。
    """
    defines, sites = fw_err_sites
    literal, _ = _split_sites(sites, defines)
    covered = set()
    for (rel, cmd_expr, code_expr), (pairs, _why) in SYMBOLIC_ERR_SITES.items():
        for cmd, code in pairs:
            if cmd is None:                      # default 分支: 任意命令
                covered.update((c, code) for c in range(0x100))
            else:
                covered.add((cmd, code))
    unexplained = sorted(set(E.ERR_TEXT) - literal - covered - ERR_TEXT_OUT_OF_TREE)
    assert not unexplained, (
        "ERR_TEXT 里有固件产不出的条目 (表里凭空多出来的码): "
        + ", ".join(f"0x{c:02X}/0x{k:02X}" for c, k in unexplained)
        + " —— 若它属于另一条固件分支, 请显式加进 ERR_TEXT_OUT_OF_TREE 并改计数")


def test_out_of_tree_exemption_is_explicit_and_counted():
    """③ 那张树外清单本身也要守 —— 它不是白名单, 是**计数过的例外**。"""
    assert len(ERR_TEXT_OUT_OF_TREE) == ERR_TEXT_OUT_OF_TREE_COUNT, (
        f"树外条目被改成 {len(ERR_TEXT_OUT_OF_TREE)}, 钉死的是 "
        f"{ERR_TEXT_OUT_OF_TREE_COUNT}")
    for pair in ERR_TEXT_OUT_OF_TREE:
        assert pair in E.ERR_TEXT, f"树外条目 {pair} 不在 ERR_TEXT 里 (过期登记)"


def test_err_site_scan_is_not_silently_empty(fw_err_sites):
    """哨兵自身的空转防护: 固件树必须真的扫到东西。

    ⚠ 没有这条, "路径写错/正则失效"会让上面四条**全部恒真** —— 那是最隐蔽的假绿。
    """
    defines, sites = fw_err_sites
    assert len(sites) >= 90, f"只扫到 {len(sites)} 处 ERR 发射点 (期望 90+, 实测 96)"
    assert len(defines) > 30, f"只解析到 {len(defines)} 个 CMD_* 定义"


# ===========================================================================
# 6. `{0x28,0x02}` 的**拒绝条件** —— 文本曾错报一条固件没有的判据 (回归闸)
# ===========================================================================

def test_0x28_02_text_does_not_claim_a_mass_rejection_that_firmware_lacks():
    """**文档存在性守卫, 不计入判据强度** —— 它查的是 `ERR_TEXT[(0x28,0x02)]` 那段散文里
    几个字面量在不在 (`NaN` / `sub>2` / `item 9`+`保留` / `default` / `静默钳制` / `0,20`),
    不查任何行为; 唯一的**取反**断言是 ① 那条被撤回的子串不许回来。⚠ 两侧都能骗人:

    * **能误报**: 同一句话**等价改写** (例如把 "只有三类" 写成 "仅有三种情形") 就可能红;
    * **能漏报**: 反过来说成**错话**只要还带着那几个字面量就照样绿 —— 例如把 "只有三类"
      改成 "只有四类" (`NaN`/`sub>2`/`item 9`/`default` 全都还在) ⇒ 本用例绿。

    ⇒ **内容对不对**由行为与固件地面真值那几条覆盖 (`test_symbolic_site_manifest_matches_firmware`
    / 下面 `test_0x28_02_text_scopes_the_clamp_claim_to_items_that_actually_clamp` 的
    `_ff_scalar_cases()` 现解), 不靠本条。

    ⚠ **回归闸**: `ERR_TEXT[(0x28,0x02)]` 曾写 "值非法 (… / payload_mass<=0)"。

    固件 `params_ff_scalar()` 里**没有**这条路径 —— item 4 走
    `ff_clamp(v, 0.0f, 20.0f)` **静默钳制**后 `return true` 回 ACK
    (`params.c:184` 钳、`:232` 返回真)。留着那条鬼判据的代价是双向的:
      · 现场看到 `ERR{0x28,0x02}` 会去查一个**不存在**的质量校验;
      · 反过来运维会以为 `set_payload(-5)` **失败了** —— 实际它 ACK, 且把质量
        静默钳成 0 (重力前馈已经变了)。
    会回本码的**只有四类** (逐条对着 `params.c:170-232` 抄):
      NaN (`:172`) / item 5·6 的 `sub>2` (`:188`/`:193`) /
      item 7 的 `v∉{0,1,2}` (`:199`) / 其余 item 落 `default` (`:219-220`, 含保留的 item 9)。
    """
    text = E.ERR_TEXT[(0x28, 0x02)]
    # ① 那条已被撤回的判据不许回来 (直接钉那个子串)
    assert "payload_mass<=0" not in text, "那条固件不存在的质量判据又回来了"
    # ② 四类真拒绝理由逐条在册
    assert "NaN" in text, "少了 NaN 这一类"
    assert "sub>2" in text.replace(" ", ""), "少了 item 5/6 的 sub>2 这一类"
    assert "item 9" in text and "保留" in text, (
        "少了「item 9 保留、写它走 default 也回本码」这句 —— 合法区间是 1..8 与 10..18, "
        "写成 1..18 会让人以为 item 9 可写")
    assert "default" in text, "少了「其余 item 落 default」这一类"
    # ③ "幅值静默钳制" 必须在 —— 它才是 `set_payload(-5)` 的真实命运
    assert "静默钳制" in text, "少了「幅值一律静默钳制」这句 (0x28 最容易被误读的地方)"
    assert "0,20" in text, "少了 payload_mass 的钳幅 [0,20]"


# ---------------------------------------------------------------------------
# 6b. 「幅值一律静默钳制」的**射程** —— 上一句曾被读成"每个 item 都钳"(回归闸)
# ---------------------------------------------------------------------------

_FF_SCALAR_BODY = re.compile(r"bool\s+params_ff_scalar\s*\([^)]*\)\s*\{(.*?)\n\}", re.DOTALL)


def _ff_scalar_cases():
    """从固件 `params_ff_scalar()` 抽 `item -> 该 case 的正文` —— 本判据的**地面真值**。

    只切 `case N:` 段 (到下一个 `case`/`default` 为止), 不解析语义 —— 判据只用
    "`return false` 出现在任何 `ff_clamp(` **之前**"这一个形态特征来判"值域拒绝"。

    ⚠ **射程上限 (已声明, 只记账)**: 正因为只判**形态特征**, 它**挡得住"删掉"、挡不住
    "语义关掉"** —— 把那一行写成 `if (0) return false;` 形态仍在, 本判据照样绿。⇒ 它覆盖
    的只是"拒绝位点漂到 `ff_clamp(` 之后"这一类改动, **语义正确性不由它保证**。
    """
    src = _read(os.path.join(FW_DIR, _FW_SUBDIR, "params", "params.c"))
    m = _FF_SCALAR_BODY.search(src)
    assert m, "params.c 里找不到 params_ff_scalar() 的函数体 —— 形态变了, 本判据要跟着改"
    body = m.group(1)
    cases = {}
    for cm in re.finditer(
            r"case\s+(\d+)\s*:(.*?)(?=\n\s*case\s+\d+\s*:|\n\s*default\s*:|\Z)", body,
            re.DOTALL):
        cases[int(cm.group(1))] = cm.group(2)
    return cases


def _rejects_before_any_clamp(case_body: str) -> bool:
    """该 case 是不是"**在任何 `ff_clamp()` 之前**就 `return false`" —— 值域拒绝。"""
    i_false = case_body.find("return false")
    i_clamp = case_body.find("ff_clamp(")
    return i_false != -1 and (i_clamp == -1 or i_false < i_clamp)


def test_0x28_02_text_scopes_the_clamp_claim_to_items_that_actually_clamp():
    """**文档存在性守卫, 不计入判据强度** —— 加上面 `_ff_scalar_cases()` 的**地面真值**那
    几行: 前者查的是那段散文里的字面量, 后者查的才是固件 `params.c` **今天真的是什么形状**
    (它只判"`return false` 出现在任何 `ff_clamp(` 之前"这一个**形态特征**, **不解析语义**
    —— 把 `return false` 写成 `if (0) return false;` 之类的**语义关掉**, 它照样绿; 那是
    已声明的射程上限)。两侧都能骗人:

    * **能误报**: 同一句话**等价改写**就红 (下面那段"已知误红"有实测);
    * **能漏报**: 反过来说成**错话**只要还带着那几个字面量就照样绿 —— 这正是下面那条
      **取反**断言存在的理由 (实测: 把历史错话**原文追加回去**, 原来那三条"子串在场"
      的断言**全部仍在场** ⇒ 旧判据绿)。

    ⚠ **回归闸**: `ERR_TEXT[(0x28,0x02)]` 曾写"⚠ **幅值一律静默钳制**: 每个 item 过
    `ff_clamp()` 后照样 `return true` 回 ACK, 固件**没有**「幅值越界 ⇒ ERR」这条路径"。

    那句话**自己打自己**: 同一条文本前一句刚把 item 7 的 `v∉{0,1,2}` 列为回本码的三类
    之一, 而 item 7 **根本不过 `ff_clamp()`** —— 它的 case 正文是
    `if (v != 0.0f && v != 1.0f && v != 2.0f) return false;` 后直接赋值
    (`params.c:197-200`), 固件自查头也写着"其余拒绝" (`usb_cmd.h:83`)。
    同理 item 5/6 的 `sub > 2` (`:188` / `:193`) 也在**任何 clamp 之前** `return false`。

    危害是**双向**的 (与上面那条"质量判据"同型): 现场面对
    `set_ff_scalar(7, 0, 3.0)` 的 `ERR{0x28,0x02}` 会去查一个**不存在**的钳幅边界,
    而真正的判据是"值域只有 {0,1,2}"。

    ⇒ 判据 = ① 钳制那句必须**被限定**在"有 clamp 的 item"上; ② item 7 与 item 5/6 的
    `sub` 必须被点名成**值域拒绝**。地面真值从固件源码现解, 不写死在这里。

    ⚠ **已知误红 —— 刻意留着, 口径同本文件的
    `test_doc_warns_that_inference_is_one_sided`**: 那三条断言查的是**字面量**, 所以把
    同一句话**等价改写**就会红。实测 (2026-09-19, `/tmp` 副本): 把
    "只对**有 `clamp` 的那些 item**" 改成 "只对**会走 `ff_clamp()` 的 item**"
    (语义等价) ⇒ `assert "有clamp" in flat` 立刻失败, 文档本身**没变坏**。
    **为什么不放宽**: 这句钳制文案是**给现场读的错误文本**(它的危害是双向的, 见上),
    改写它**就等于重写判据的表述** ⇒ 本来就该连带把地面真值重核一遍 (本用例的地面真值
    正是从 `params.c` **现解**的), 让它红是**提醒**, 不是维护负担。
    ⚠ **尤其别**放宽成"含 `clamp` 或 `ff_clamp`" —— 实测那句**历史错话**
    ("幅值一律静默钳制: 每个 item 过 `ff_clamp()` 后照样 `return true` 回 ACK")
    **同样含 `ff_clamp`** ⇒ 这种放宽**放过了它自己的回归对象**, 等于把闸拆了。
    (能同时接住"现文本 / 等价改写"又挡住历史错话的写法是"`只对`/`仅对` **且** `clamp`";
    但那是**另一条**任意线 —— 它会放过"只对 item 7 钳制"这类**真错话**, 故不取。)

    ⚠ **补的那条取反断言 (③) 才是真正的回归闸**: 上面 ①② 与 `item7` 只查"正确的话在场",
    接不住"错话**并存**" —— 实测 (2026-09-19) 把历史错话原文**追加回**
    `ERR_TEXT[(0x28,0x02)]`, 那三条子串断言**全部仍在场** ⇒ 旧判据 **GREEN**。③ 因此
    **取反**钉住那句的**普遍量化**形状 ("每个 item 都过 `ff_clamp()`")。⚠ 它自己的射程:
    只钉 `每个item` / `所有item` 两个扁平化词面, **换个说法的同义错话** (例: "任何 item
    均过 `ff_clamp()`") **不在此列** —— 那种改写要连带重核地面真值, 由上面那段"已知误红"
    的口径兜底。
    """
    cases = _ff_scalar_cases()
    for item in (5, 6, 7):
        assert item in cases, f"params.c 里没有 case {item} —— 地面真值变了"
        assert _rejects_before_any_clamp(cases[item]), (
            f"params.c 的 case {item} 不再是「clamp 之前就 return false」了 —— "
            f"本判据的地面真值变了, 先核对固件再改这里 (别为了让用例变绿而放宽断言)")
    assert "ff_clamp(" in cases[4], (
        "case 4 不再是走 ff_clamp 的钳幅那一路 —— 地面真值变了 ('有 clamp 的 item' "
        "这句话的正面样本就是它)")

    text = E.ERR_TEXT[(0x28, 0x02)]
    flat = text.replace(" ", "").replace("*", "").replace("`", "")
    # ① 钳制那句必须被限定 —— 少了限定就会被读成"每个 item 都钳", 而同一条文本自己
    #    列的 item 7 就是**不钳**的直接反例
    assert "有clamp" in flat, (
        "缺少「只对**有 clamp 的那些 item** 成立」这个限定 —— 强断言会把 item 7 / "
        "item 5·6 的 sub 一起说成'被钳', 与同一条文本自己列的拒绝条件矛盾")
    # ② 那两处必须被点名成**值域拒绝**, 而不是幅值越界
    assert "值域拒绝" in flat, (
        "缺少「item 7 与 item 5/6 的 sub 是**值域拒绝**」这句 —— 它们是 rejection, "
        "不是幅值越界; 少了它, 现场会去查一个不存在的钳幅边界")
    assert "item7" in flat, "「值域拒绝」那句没有点名 item 7 (最容易被误读的就是它)"
    # ③ (**取反题**) 历史错话的**普遍量化**形状不许并存 —— ①② 只查"正确的话在场",
    #    追加回历史错话时它们全部仍在场 ⇒ 只靠 ①② 的旧判据接不住那个回归对象 (实测)
    for universal in ("每个item", "所有item"):
        assert universal not in flat, (
            f"「幅值一律静默钳制」又被写成了**普遍量化** ({universal}) —— 历史错话 "
            f"(docstring 里那一段) 正是这个形状: \"每个 item 过 `ff_clamp()` 后照样 "
            f"`return true` 回 ACK\"; 而同一条文本自己列的 item 7 / item 5·6 的 sub "
            f"就是**不钳**的直接反例")


def test_set_payload_accepts_negative_mass_without_a_local_rejection(offline_arm):
    """`set_payload(-5)` **不抛** —— SDK 侧刻意**不**加 "质量须 >0" 的本地预检。

    ⚠ 本用例**压着两件事, 判据强度不同**, 别把它们读成同一档:

    * **最后那条查 docstring 的断言**是 **文档存在性守卫, 不计入判据强度** —— 它只查
      `set_payload` 的 docstring 里还有没有 `钳` 与 `负` 两个字, 不查行为 (把整段说明
      换成任何**别的话**、只要这两个字还在, 它照样绿; 而等价改写缺了字就红);
    * 前面两条查的是**行为**: "不抛" + "`0x28` 帧照发出去"。⚠ 那两条也**只管到
      '发出去了'为止** —— **不校验载荷值**, 所以"SDK 先钳了再发"对它**没有观测面**
      (实测: 给 `set_payload` 加本地钳制, 全量 531 passed 全绿)。载荷值由
      `tests/test_ff_io.py::test_set_payload_sends_the_raw_value_and_leaves_clamping_to_the_firmware`
      钉住。

    固件对负质量是**钳成 0 后回 ACK** (`params.c:184`), **不是**拒绝。SDK 若自作主张
    抛 `InvalidCommandError`, 调用方拿到的"被拒"就是**假的** (参数其实写进去了, 只是
    被钳过), 而且与 `set_ff_scalar` 的"固件权威 + 幅值钳制"口径分叉。

    ⚠ 桩 (`fake_serial.py:297-299`) **不建模钳幅**, 只存原值 —— 所以本用例钉的是
    "**SDK 不拦、帧照发**", **不是** "钳到 0" (后者只能真机验)。别把断言写成
    `读回 == 0.0`, 那会去断言一个桩压根没实现的语义。
    """
    arm = offline_arm
    arm.set_payload(-5.0)                     # 不抛即通过 (无本地预检)
    assert P.CMD_SET_FF_SCALAR in {c for c, _p in arm._tr.tx_log}, (
        "负质量被本地吞掉了, 一个字节都没发 —— 固件那侧根本没机会钳制它")
    doc = arm_mod.Arm.set_payload.__doc__ or ""
    assert "钳" in doc and "负" in doc, (
        "`set_payload` 的 docstring 必须写明负值会被钳成 0 —— 否则调用方无从知道"
        "传负值不是「被拒」而是「质量变成 0、重力前馈跟着变」")
