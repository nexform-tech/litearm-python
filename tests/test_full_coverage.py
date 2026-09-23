"""「覆盖所有功能」的可执行断言 —— 固件每条命令都要被某个 SDK 入口**真的发出去过**。

`test_protocol_sync.py` 只保证「入口能解析到」(属性存在且可调用), 那是**静态**的。
这里做**动态**验证: 在桩固件上把每个入口真跑一遍, 断言固件会收到对应的命令 id。
两者缺一不可 —— 属性存在但一调用就炸, 静态检查是看不出来的。

[2026-09-14] **无豁免** —— `0x30`/`0x33` 已由「动力学模型在线导入」实现,
`P.UNIMPLEMENTED_CMDS` 现为空表。
"""
from __future__ import annotations

import struct

import pytest

from litearm import _protocol as P
from litearm.errors import ArmIsInDfuError, CommandRejectedError


def _sent_ids_of(tr):
    return {c for c, _ in list(tr.tx_log)}


def _sent_ids(arm):
    return _sent_ids_of(arm._tr)


def test_every_implemented_command_is_exercised_through_its_entry(offline_arm):
    """逐个入口跑一遍, 断言固件确实收到了那条命令。"""
    arm = offline_arm
    arm.connect()                       # 0x41 固件版本
    arm.get_status_now()                # 0x40 主动取状态

    arm.enable()                        # 0x10
    arm.movej([0.05] * 7, speed=0.3)    # 0x01
    arm.movej_sync([0.05] * 7, speed=0.3)                    # 0x07 [SYNC] 同步 PTP
    arm.move_p((0.30, 0.0, 0.35, 0.0, 0.0, 0.0), speed=0.3)  # 0x02
    arm.move_js([0.0] * 7, dq=[0.0] * 7)                     # 0x03
    arm.move_js([0.0] * 7, dq=[0.0] * 7, tau_ff=[0.0] * 7)
    arm.send_mit(0, 0.0, 0.0, 50.0, 2.0, 0.0)                # 0x04
    arm.send_mit_all([0.0] * 7, [0.0] * 7, [50.0] * 7, [2.0] * 7, [0.0] * 7)  # 0x05
    arm.set_motion_mode(0)              # 0x20
    arm.set_speed(50)                   # 0x21
    arm.ik((0.30, 0.0, 0.35, 0.0, 0.0, 0.0))                 # 0x42
    arm.get_tcp()                       # 0x43
    arm.set_ff_vec(1, [0.0] * 7)        # 0x26
    arm.set_ff_scalar(1, 0, 0.0)        # 0x28
    arm.set_ff_mask(P.FF_ALL)           # 0x27
    arm.ff_preset(1)                    # 0x31
    arm.get_ff_vec(1)                   # 0x2B
    arm.get_ff_scalar(1, 0)             # 0x2C
    arm.home()                          # 0x2A
    arm.params.set_joint_param(0, 50.0, 2.0, 10.0)            # 0x22
    arm.params.set_joint_limits(0, -3.0, 3.0)                 # 0x23
    arm.params.get_joint_param(0)                             # 0x24
    arm.log.start(4)                                          # 0x2D
    arm.log.reader().read_all()                               # 0x2E
    arm.log.stop()                                            # 0x2D (0)
    arm.diag.kin_bench()                                      # 0x49
    with arm.zero_g():                                        # 0x06
        pass
    # ---- [笛卡尔] 固件原生规划 (0x3A/0x3B/0x3C/0x3D/0x3E) ----
    # ⚠ **必须留在这段使能窗口内**: 下面 `arm.disable()` 之后固件侧三道门禁会回
    # `ERR{cmd,0x03}` (未使能) —— 那是**固件的**行为, 与桩无关, 但真机上放在下面
    # 就是一次 `CommandRejectedError`。
    # `move_c` 的 pose_start 必须与**实测 TCP** 一致 (上面 move_p 已把桩的 pose 设成它)。
    arm.move_l((0.30, 0.0, 0.35, 0.0, 0.0, 0.0), speed=0.3)          # 0x3A
    arm.move_c((0.30, 0.0, 0.35, 0.0, 0.0, 0.0),
               (0.30, 0.0, 0.40, 0.0, 0.0, 0.0),
               (0.32, 0.0, 0.40, 0.0, 0.0, 0.0), speed=0.3)          # 0x3B
    arm.move_path([(0.30, 0.0, 0.35, 0.0, 0.0, 0.0),
                   (0.30, 0.0, 0.40, 0.0, 0.0, 0.0),
                   (0.32, 0.0, 0.40, 0.0, 0.0, 0.0)], speed=0.3)     # 0x3C/0x3D/0x3E

    arm.clear_faults()                  # 0x13
    arm.reset()                         # 0x14
    arm.enable()
    arm.disable()                       # 0x11
    # ⚠ `0x25` **必须留在这段失能窗口内** (固件 `ctrl_is_armed()` ⇒ `ERR{0x25,0x04}`)。
    # 它从前排在 0x24 之后、即**上面那段使能窗口里** —— 桩那时不建模该门禁, 离线全绿,
    # 真机上却是一次货真价实的 `CommandRejectedError`。
    # ⚠ **别改成"在使能窗口里插一句 `disable()`"**: 那会把 0x3A~0x3E 的笛卡尔块顶出
    # 使能窗口 (见上面那段注释), 而离线仍绿 (桩不建模笛卡尔的使能门禁) ⇒ 那句注释
    # 变成假话、真机上 5 条笛卡尔命令全被拒。这就是本文件要消灭的那类"会说谎的测试"。
    arm.save_params()                   # 0x25 (须失能 — 上面已 disable)
    arm.emergency_stop()                # 0x12
    arm.params.reset_factory()          # 0x36 (须失能 — 上面已 disable)

    # 其余公开入口也真跑一遍 (只登记不调用 = 死接口)
    arm.get_state(refresh=True)
    arm.get_ff_mask()                                        # 0x2C item 9 只读
    arm.set_gravity_scale([1.0] * 7)                         # 0x26 item 7
    arm.set_inertia_scale([1.0] * 7)                         # 0x26 item 8
    arm.set_payload(0.5, (0.0, 0.0, 0.0))                    # 0x28 item 4/5
    arm.set_gravity_vector((0.0, 0.0, -9.81))                # 0x28 item 6
    arm.park()                                               # 0x20 mode 0
    arm.zero_g_start()                                       # 0x06 显式配对
    arm.zero_g_stop()

    # ---- [2026-09-14] 动力学模型在线导入 (0x30/0x32/0x33/0x34/0x35/0x37/0x38/0x39) ----
    # commit / revert 要求失能 (固件判据 ctrl_is_armed = enabled || enable_pending)
    arm.disable()
    arm.model.probe()                                        # 0x34 (能力探测)
    arm.model.get_body(0)                                    # 0x34
    for i in range(1, 8):                                    # 0x30 ×7 (body1..7)
        arm.model.set_body(i, [1.0, 0.0, 0.0, 0.0] + [0.0] * 6)
    arm.model.set_jm([0.0] * 7)                              # 0x33
    arm.model.get_jm()                                       # 0x35
    from litearm.model import MODEL_MASK_WRITTEN
    arm.model.commit(MODEL_MASK_WRITTEN)                     # 0x32 (掩码必须逐位相符)
    arm.model.status()                                       # 0x38
    arm.model.get_gravity([0.0] * 7)                         # 0x39
    arm.model.revert()                                       # 0x37

    # ---- [授权] 0x2F 查询 / 0x3F 提交 (见 tests/test_license.py) ----
    # ⚠ `0x3F` 同样**必须留在这段失能窗口内** —— 固件对已武装的臂回 `ERR{0x3F,0x04}`
    #   (与 `0x25` 同语义), 而桩照同一判据建模。
    # ⚠ 桩默认 `activated=True` ⇒ 这一发会得到聚合档 `0x02`(已存在), 再由 SDK 回读
    #   `0x2F` 定性后**正常返回** —— 那正是本包要跑的路径 (0x02 的处理在 test_license.py
    #   另有正反两条判据)。本用例只负责"入口可达、命令真发出去了"。
    arm.license()                                            # 0x2F
    arm.activate(cust_id=1, issued=20260922, mac=bytes(16))  # 0x3F

    # ---- [DFU] 0x15 `Arm.enter_dfu()` —— **必须留在这段失能窗口内** ----
    # 固件侧它是**两段式**: ACK 只表示"已登记", 还要等设备真的消失才置终态 (见
    # tests/test_dfu.py)。桩默认模拟"真跳转" (ACK 之后读路径抛 TransportError),
    # 于是本行正常返回、而 `arm` 从此进**终端态** ⇒ 它只能是本测试的**最后**一条命令,
    # 上面的 `sent` 快照与下面的 close 都还在终态之前/之中完成。
    # ⚠ 放在**使能中**会怎样: 固件回 `ERR{0x15,0x03}` (`ctrl_is_armed`), 而桩照同一
    # 判据守 —— 故这里必须是失能态 (上面那几处 `disable()` 之后一路失能; ⚠ 别按行号找,
    # 行号会漂)。
    tr = arm._tr                   # 终态会把 `arm._tr` 置 None ⇒ 先抓住 transport 引用
    arm.enter_dfu()                                          # 0x15 (终端态: 此后本对象不可用)

    sent = _sent_ids_of(tr)        # 关链前取快照 (reconnect 会换成新的 transport)
    arm.close()                    # 终态下仍必须可用 (幂等空操作, 不抛)
    with pytest.raises(ArmIsInDfuError):
        # 终态**不因 reconnect 复活** —— 要接着用臂请新建一个 `Arm`。
        # (这条钉在这里而不是只写在 test_dfu.py: 它是"终端态"最容易被顺手放宽的一处)
        arm.reconnect()

    want = set(P.COMMAND_COVERAGE) - set(P.UNIMPLEMENTED_CMDS)
    missing = sorted(want - sent)
    assert not missing, (
        "以下固件命令从未被任何 SDK 入口真正发出: "
        + ", ".join(f"0x{m:02X}({P.COMMAND_COVERAGE[m]})" for m in missing))


def test_no_extra_command_ids_are_sent(offline_arm):
    """反向: 发出去的 id 必须都在覆盖表里 (防止发了未登记/越界的命令)。"""
    arm = offline_arm
    arm.connect()
    arm.enable()
    arm.movej([0.0] * 7)
    arm.get_tcp()
    arm.disable()
    sent = _sent_ids(arm)
    unknown = sorted(sent - set(P.COMMAND_COVERAGE) - set(P.UNIMPLEMENTED_CMDS))
    assert not unknown, f"发出了未登记的命令 id: {[hex(x) for x in unknown]}"


def test_public_api_surface_is_all_exercised(offline_arm):
    """`Arm` 的公开方法/属性 —— **名单守卫**: 不许出现没登记进 `exercised` 的公开成员。

    ⚠ **措辞要说准** (本条从前写成"不许有**从没被调用过**的死接口" —— 与实现不符):
    实现只做**集合比较** `public - exercised - allowed_extra`, 它**不真的验证**这些成员
    被调用过。哪个成员由哪条用例、在什么形状下被练习到, 是**那些用例自己**的事,
    不在这条判据的射程内 —— 别把"名单里有"读成"已经验过了"。
    """
    arm = offline_arm
    exercised = {
        "connect", "close", "get_state", "get_status_now", "get_tcp", "ik", "home",
        "enable", "disable", "emergency_stop", "reset", "clear_faults",
        "set_motion_mode", "park", "set_speed", "zero_g", "zero_g_start", "zero_g_stop",
        "movej", "movej_sync", "move_p", "move_js", "send_mit", "send_mit_all", "reconnect",
        # 收尾的两个名字 (同一个操作): 判据在 tests/test_teardown.py
        "disconnect",
        "set_ff_mask", "ff_preset", "set_ff_vec", "set_ff_scalar", "get_ff_vec",
        "get_ff_scalar", "get_ff_mask", "set_gravity_scale", "set_inertia_scale",
        "set_payload", "set_gravity_vector", "save_params",
        "params", "log", "diag", "zero_g_active", "zero_g_error", "last_reset_reason",
        # DFU (唯一会置**终端态**的入口; 判据在 tests/test_dfu.py, 本文件只跑一次真下发)
        "enter_dfu",
        # 笛卡尔 —— 只有固件原生路径一条 (PC 侧规划的 cartesian/movel/movec 已退役)
        "move_l", "move_c", "move_path", "poll_cart",
        # [2026-09-14] 动力学模型在线导入 (见 tests/test_model.py)
        "model",
        # [授权] 开机即锁 + 激活/查询 (见 tests/test_license.py)
        "license", "activate",
    }
    public = {n for n in dir(arm) if not n.startswith("_")}
    # 允许的少量内部/兼容名
    allowed_extra = {
        # 只读状态
        "min_firmware", "firmware", "fw_version", "n", "q_tol", "dq_tol",
        "arrive_frames", "move_timeout", "bench_model_axis",
        # 类级数据表 (不是方法)
        "FF_VEC_ITEMS", "FF_SCALAR_ITEMS", "FF_SCALAR_RO_ITEMS",
    }
    unlisted = public - exercised - allowed_extra
    assert not unlisted, (
        f"Arm 上有未纳入清单的公开成员: {sorted(unlisted)} —— "
        f"要么补进 exercised(并加测试), 要么确认它是内部状态")


# =========================== 门禁 (0x23 / 0x25) ===========================
#
# 上面那条覆盖用例只证明"命令真的发出去了" —— 它读的是**下行** `tx_log`。若桩对
# `0x23`/`0x25` 一律回 ACK, 那么"SDK 在固件拒绝时会给调用方一个可判定的失败"这件事
# 在离线用例里**完全没有观测面**: 桩的绿灯就变成"桩不建模门禁"的同义语。
#
# 固件出处 (两条都是 `usb_cmd.c` 的 case 内第一道判据, 排在一切副作用之前):
#   `CMD_PARAM_SAVE(0x25)`        -> `ctrl_is_armed()`             ⇒ `ERR{0x25,0x04}`
#   `CMD_SET_JOINT_LIMITS(0x23)`  -> `ctrl_is_armed() && ctrl_axis_outside(idx,lo,hi)`
#                                                                   ⇒ `ERR{0x23,0x03}`
#   ⚠ 0x23 的**顺序**是关键: 门禁在 idx/区间校验**之前** ⇒ "武装 + idx 越界" 固件回
#   `0x03`, 而不是值校验那条 `0x02`。桩若把门禁排在后面就会在这格上与固件相反。
#   ⚠ 门禁判据是 `ctrl_is_armed() = enabled || enable_pending`。`enable_pending` 在状态
#   帧里没有位 (主机预检看不见), 桩用 `dfu_armed_pending` 造那个形状, 并由 `FakeTransport
#   .armed` **一个判据供五处共用** (0x25/0x36/0x32/0x37 + 0x23 的第一层, 逐条对得上固件
#   那五处 case) —— 所以下面既有"武装 (enable) 中"的用例, 也有"仅使能在途"的那条。
#
# 桩的 `self.q[idx]` 是固件 `g_arm.cmd[idx].q_ref` 的**代理** (桩不建模运动过程) ——
# ⚠ 只在 `0x01`/`0x2A` 上精确, 其余命令后 `self.q` 停在旧值 (潜伏, 现无用例能踩到);
# 精确的例外清单见 `fake_serial.py` 的 `_axis_outside` 文档串。


def test_set_joint_limits_refused_while_armed_when_new_range_cannot_hold_the_ref(offline_arm):
    """armed **且**新限位关不住当前参考 ⇒ `ERR{0x23,0x03}` (固件 B1 fix)。

    载荷本身完全合法 (`0 < 0.10 < 0.20`), 被拒**只**因为 q_ref=0.05 落在新限位之外
    —— 固件那条注释的原话: ACK 了会出现在途轨迹仍奔向收窄前的旧目标、轴被开出新限位
    ⇒ 位置包络锁存 (该轴失力)。
    """
    arm = offline_arm
    arm.enable()
    arm.movej([0.05] * 7)                     # 桩: 此后 q_ref = 0.05
    with pytest.raises(CommandRejectedError) as ei:
        arm.params.set_joint_limits(0, q_min=0.10, q_max=0.20)
    assert (ei.value.cmd, ei.value.code) == (P.CMD_SET_JOINT_LIMITS, 0x03)


def test_set_joint_limits_allowed_while_armed_when_new_range_holds_the_ref(offline_arm):
    """⚠ "武装中"本身**不是**拒绝理由 —— 判据是"武装 **且** 关不住"。

    专防把门禁写成"armed 一律拒": 那样上面那条覆盖用例里的一次
    `set_joint_limits(0, -3.0, 3.0)` (`q[0]=0.05 ∈ [-3,3]`) 会立刻变红, 真机上
    任何"武装中收窄一个无关轴"的合法用法也都会变成 ERR。
    """
    arm = offline_arm
    arm.enable()
    arm.movej([0.05] * 7)
    arm.params.set_joint_limits(0, q_min=-3.0, q_max=3.0)     # 0.05 ∈ [-3,3] ⇒ 放行
    # 不只是"没抛": 读回确认这一笔真的落到了参数表 (ACK 与生效是两件事)
    assert arm.params.get_joint_param(0).value.q_min == -3.0


def test_set_joint_limits_gate_precedes_the_value_checks(offline_arm):
    """⚠ 桩级 (裸帧): 门禁在 idx / 区间校验**之前** —— 判据只有"门禁"命中的那两格。

    固件 `ctrl_axis_outside` 的六条里有**两条只看入参** (① `idx` 越界、② 空区间),
    它们**只在武装态**才成为拒绝理由 (门禁是 `ctrl_is_armed() && ...`):
      (a) armed + idx 越界   ⇒ `0x03` (不是值校验那条 `0x02`)
      (b) armed + 空区间     ⇒ `0x03` (同上)

    `Arm.params.set_joint_limits` 在本地 (`_check_idx` / `q_min < q_max`) 就把这两格拦掉
    了, 所以只有直接发裸帧才到得了桩 —— 它钉的是**分支顺序**: 桩原有分支是
    len → idx(0x02) → lo<hi(0x02), 固件是 len → 门禁(0x03) → 值校验(0x02)。顺序错开时
    这两格的回码与真机**相反**, 而任何走 SDK 入口的用例都看不见这个分叉。
    """
    arm = offline_arm
    arm.enable()
    for label, payload, want in [
            ("idx 越界", bytes([99]) + struct.pack("<ff", -1.0, 1.0), 0x03),
            ("空区间", bytes([0]) + struct.pack("<ff", 1.0, -1.0), 0x03)]:
        arm._tr.write_frame(P.CMD_SET_JOINT_LIMITS, payload)
        fr = arm._tr.read_frame(0.2)
        assert fr is not None, f"{label}: 桩没回应答"
        cmd, body = fr
        assert cmd == P.RSP_ERR, f"{label}: 期望 ERR, 拿到 {hex(cmd)}"
        assert body[:2] == bytes([P.CMD_SET_JOINT_LIMITS, want]), (
            f"{label}: 期望 0x{want:02X}, 拿到 0x{body[1]:02X}")


def test_save_params_refused_while_armed(offline_arm):
    """武装中整扇区擦写 = 电机无监督运行窗口 ⇒ 固件回 `ERR{0x25,0x04}` (须先失能)。"""
    arm = offline_arm
    arm.enable()
    with pytest.raises(CommandRejectedError) as ei:
        arm.save_params()
    assert (ei.value.cmd, ei.value.code) == (P.CMD_PARAM_SAVE, 0x04)


def test_save_params_ok_when_disarmed(offline_arm):
    """失能后才 ACK —— `_cmd` 已经把"必须真收到 ACK"钉住了, 这里只需要它不抛。"""
    arm = offline_arm
    arm.enable()
    arm.disable()
    arm.save_params()


def test_save_params_refused_when_enable_is_only_pending(offline_arm):
    """⚠ `ctrl_is_armed() = enabled || enable_pending` 的**另一半** (`usb_cmd.c:670`)。

    `enable_pending` 在状态帧里**没有位** ⇒ 主机侧预检看不见它, 但固件那条门禁判的
    就是它。桩若只判 `self.enabled`, 这一格会 ACK 掉一次真机上必然被拒的整扇区擦写
    (擦写窗口电机无监督) —— 假绿。桩用 `dfu_armed_pending` 造这个形状 (与 0x15 门禁
    同一个旗标、同一套语义)。
    """
    arm = offline_arm
    arm._tr.dfu_armed_pending = True     # 固件侧: 使能已登记、`enabled` 还没落地
    assert arm.get_state(refresh=True).value.enabled is False, (
        "前提: 状态帧里这一位仍是 0 —— 预检放行, 只有固件门禁能拦")

    with pytest.raises(CommandRejectedError) as ei:
        arm.save_params()
    assert (ei.value.cmd, ei.value.code) == (P.CMD_PARAM_SAVE, 0x04)


@pytest.mark.parametrize("cmd, payload, q0, want_code", [
    # 桩侧 `FakeTransport.armed` **一个判据供五处共用** —— 这里逐条钉住"使能在途也
    # 等于武装", 否则任何一处退回只判 `self.enabled` 都会重新变成半格假绿。
    (P.CMD_PARAM_SAVE,       b"",                                  0.0, 0x04),
    (P.CMD_PARAM_RESET,      b"",                                  0.0, 0x04),
    (P.CMD_MODEL_COMMIT,     struct.pack("<H", 0),                 0.0, 0x04),
    (P.CMD_REVERT_MODEL,     b"",                                  0.0, 0x04),
    # 0x23 是 `ctrl_is_armed() && ctrl_axis_outside(...)`: 载荷本身合法, 只有把 q_ref
    # 代理 (`self.q`) 放在新区间外才命中门禁 ⇒ `0x03` (不是值校验的 `0x02`)。
    (P.CMD_SET_JOINT_LIMITS, bytes([0]) + struct.pack("<ff", 0.10, 0.20), 0.5, 0x03),
], ids=["0x25_PARAM_SAVE", "0x36_PARAM_RESET", "0x32_MODEL_COMMIT",
        "0x37_REVERT_MODEL", "0x23_SET_JOINT_LIMITS"])
def test_all_gates_share_the_one_armed_predicate(offline_arm, cmd, payload, q0, want_code):
    """⚠ **裸帧级**: 五处门禁 (0x25/0x36/0x32/0x37/0x23) 判的都是固件 `ctrl_is_armed()`
    (`control_loop.c:1334`), 含 `enable_pending` 那一半 —— 桩用 `dfu_armed_pending` 造。

    走 SDK 入口到不了这里: `enable_pending` 在状态帧里没有位 ⇒ 主机预检恒放行, 且 0x23
    那格还要绕开 SDK 本地的 `_check_idx`/`q_min < q_max`。所以这条钉的是桩的门禁判据本身。
    """
    arm = offline_arm
    arm._tr.dfu_armed_pending = True
    arm._tr.q = [q0] * arm._tr.n

    arm._tr.write_frame(cmd, payload)
    fr = arm._tr.read_frame(0.2)
    assert fr is not None, f"0x{cmd:02X}: 桩没回应答"
    got_cmd, body = fr
    assert got_cmd == P.RSP_ERR, f"0x{cmd:02X}: 期望 ERR, 拿到 {hex(got_cmd)} (使能在途未被当武装)"
    assert body[:2] == bytes([cmd, want_code]), (
        f"0x{cmd:02X}: 期望 0x{want_code:02X}, 拿到 0x{body[1]:02X}")
