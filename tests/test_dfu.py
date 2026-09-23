"""DFU `0x15` —— SDK 里**唯一**的终端态操作 (spec §6.1)。

固件契约 (逐条对着固件源码核过; 本文件每条判据的依据都写在注释里, 固件只读):

* **受理**: `usb_cmd.c:543-555` 的 `case CMD_ENTER_DFU` —— 载荷非空 ⇒
  `ERR{0x15,0x01}`; 否则 `ctrl_request_enter_dfu()` 返回 0 ⇒ `ACK{0x15}`, 非 0 ⇒
  `ERR{0x15, r}`。
* **门禁与登记**: `control_loop.c:1087-1099` —— `g_arm.enabled || enable_pending`
  ⇒ `0x03`; ROM 向量表无效 ⇒ `0x02`; 否则 `dfu_pending = true` (**只登记**)。
* ⚠ **`ACK{0x15}` 只表示"已登记"**: 不表示"会跳"、更不表示"已经跳"。登记可被**三个**
  点**静默撤销** (都 `dfu_pending = false; return;` 且**不回报** —— 而 ACK 是成功的):
  `control_loop.c:1106` (并发 ENABLE: 同一个 USB 批里 `0x15` 紧接 `0x10` 即命中)、
  `:1142` (交棒前向量表**复读**失败) 与 `:1112` (1s 总超时兜底)。
  ⚠ 第三条 (`:1112`) **实际不可达**: 10ms/100ms/100ms 三道门控 (`:1118`/`:1131`/`:1137`)
  都在 `age ≤ 100ms` 处放行, 到点必然先在 `:1142`/`:1156` 落定, 走不到 1s 判据
  (`:1111` 要 `age > 300` 拍)。理由与算术同 `arm.py` 模块顶部 `DFU_REVOKED_MESSAGE` 那段。
* **真跳转在 main 线程** (`Core/Src/main.c:189` 的 `ctrl_dfu_poll_main()`), 上界
  **100ms** —— 三道门控共用同一个 `age` (`control_loop.c:1110/1118/1131/1137`),
  **不是** 10+100+100=210ms。

⇒ 所以 `enter_dfu()` 必须**两段式**: ACK 之后**等设备真的消失**才置终端态; 超时未消失
就抛"登记被撤销/未执行"并**让 `Arm` 保持可用** (静默撤销路径的**唯一**出口)。

"设备已消失"的判据只有一条: **读路径抛 `TransportError`** —— pyserial 在设备被摘掉时
`select` 报就绪而 `read()` 返回空 ⇒ `serialposix.py:591-597` 抛 `SerialException`
(注释原话: Linux 上断开的设备就是这样), 被 `transport._read_chunk` 包成
`TransportError` (`transport.py:318-336` —— 函数从 `:318` 起, `raise` 在 `:336`)。
⚠ 两条**不可用**的判据 (spec §8 V1): `is_open` 拔线后仍为真 (它是 pyserial **自己维护的
实例属性**, 开置 True / 关置 False, 而拔线不经过 `close()`); "读到 0 字节"是**读超时**的正常
返回 (`serialposix.py:573-574` → `b''`) —— 拿它当判据会把**真跳转**读成"什么都没发生",
结论正好反了。
"""
from __future__ import annotations

import pytest

import litearm as pa
from litearm import _protocol as P


def _sent(tr) -> set:
    return {c for c, _p in tr.tx_log}


def _dfu_payloads(tr) -> list:
    """发出去的 `0x15` 的载荷 (逐条) —— "我们恒发空载荷"这条由它钉住。

    ⚠ 取的是**桩 transport 对象**, 不是 `arm._tr`: 真跳转那条路径上 `enter_dfu()`
    会关链路 (`arm._tr` 置 `None`), 之后再想读 `tx_log` 就没得读了 —— 测试必须在调用
    之前抓住 transport 引用。
    """
    return [p for c, p in tr.tx_log if c == P.CMD_ENTER_DFU]


# ---------------------------------------------------------------- 预检
def test_enabled_precheck_refuses_readably_and_sends_nothing(offline_arm):
    """使能中 ⇒ **不发帧**、给可读错 (spec §6.1 的门禁 1)。

    ⚠ 这**不是**洁癖: 跳转停 TIM3 ⇒ 不再发 MIT 帧 ⇒ 电机侧 `RID_TIMEOUT`(100ms)
    松开 ⇒ 使能中跳会让臂**下垂** (`control_loop.c:1088-1090` 的原注释)。
    ⚠ 本地预检的口径**只有** `st.enabled` (状态帧 bit9) —— 固件那道门禁是
    `ctrl_is_armed() = enabled || enable_pending` (定义在 `control_loop.c:1334-1336`;
    `:1091` 是 `ctrl_request_enter_dfu` 里把同一谓词**内联**写的那一行), 所以
    **使能在途**时预检会放行、由固件回 `ERR{0x15,0x03}` (那一条的判据见下面
    `test_err_0x03_*`)。
    """
    arm = offline_arm
    arm.enable()
    tr = arm._tr
    before = len(tr.tx_log)

    with pytest.raises(pa.InvalidCommandError) as ei:
        arm.enter_dfu()

    assert "disable()" in str(ei.value), f"错误消息没给出路: {ei.value}"
    assert len(tr.tx_log) == before, "预检拒了却还是把帧发出去了"
    assert P.CMD_ENTER_DFU not in _sent(tr)


def test_state_unreadable_does_not_block_the_command(offline_arm):
    """读不到状态帧时**不拦** —— 本地预检只为可读性, 门禁的权威在固件。

    理由: 固件对 `enabled || enable_pending` 一律回 `ERR{0x15,0x03}`, 所以"没确认
    失能就发"**不会**让使能中的臂被跳掉; 反过来, 拿"读不到状态"当拒绝理由会把一次
    合法升级卡死在一个与它无关的故障上。
    """
    arm = offline_arm
    arm._tr.auto_status = False          # 桩不再补空闲状态帧 ⇒ 读不到状态
    arm._tr._resp.clear()
    tr = arm._tr

    arm.enter_dfu()                      # 桩默认"真跳转" ⇒ 正常返回

    assert _dfu_payloads(tr) == [b""], "预检取不到状态时 0x15 没发出去"


# ---------------------------------------------------------------- 静默撤销
def test_silent_revocation_times_out_and_leaves_the_arm_usable(offline_arm):
    """桩只回 ACK、设备**不消失** ⇒ 抛"登记被撤销/未执行", 且 `Arm` 仍可用。

    这就是那三个静默撤销点 (`control_loop.c:1106` / `:1142` / `:1112`, 最后一条不可达)
    的**唯一出口**:
    ACK 是成功的、而设备还在 CDC 上, 缺了这条出口就是"ACK 成功了却什么都没发生"。
    "仍可用"必须**行为化**验证 (读一帧 + 写一条), 因为终态的对象**所有走到取帧/写帧
    收口的**入口都抛 `ArmIsInDfuError` (⚠ 射程的准话见 `_reject_if_in_dfu`) ——
    只看"没抛异常"证明不了它没进终态。
    """
    arm = offline_arm
    arm._tr.dfu_vanishes = False         # 登记成功但设备不跳 (静默撤销那一支)
    tr = arm._tr

    with pytest.raises(pa.LiteArmError) as ei:
        arm.enter_dfu()

    assert "撤销" in str(ei.value) or "未执行" in str(ei.value), (
        f"超时未消失的归因不对 (要指明'登记被撤销/未执行'): {ei.value}")
    assert _dfu_payloads(tr) == [b""], "0x15 没发出去 —— 那测的不是这条路径"

    st = arm.get_state(refresh=True).value     # 读路径仍可用 (且没被置成终态)
    assert st is not None
    arm.enable()                         # 写路径仍可用
    assert P.CMD_ENABLE in _sent(tr)


# ---------------------------------------------------------------- 真消失 ⇒ 终态
def test_a_real_jump_sets_the_terminal_state(offline_arm):
    """桩在 ACK 之后让读路径抛 `TransportError` (= 真跳转) ⇒ 置**终端态**。

    终态的两条性质都钉在这里:
      · 关 transport (`Arm._tr` 置 `None`, 与 `close()` 同一套收尾);
      · 此后**所有走到取帧/写帧收口的**入口都抛 `ArmIsInDfuError` —— 而不是
        `NotConnectedError`
        (后者会被读成"一次可以重连恢复的掉线")、也不是 `MotionTimeoutError`
        ("固件不理我")。
    ⚠ `close()` **必须仍然可用**: teardown 在任何状态下都不能抛 (否则 `with Arm(...)`
    的退出路径会把一次成功升级报成异常)。
    """
    arm = offline_arm

    assert arm.enter_dfu() is None       # 桩默认: ACK 后设备消失 ⇒ 正常返回

    assert arm._tr is None, "终端态必须把 transport 关掉 (与 close() 同一套收尾)"
    assert isinstance(pa.ArmIsInDfuError("x"), pa.LiteArmError)
    assert not issubclass(pa.ArmIsInDfuError, pa.NotConnectedError), (
        "终态刻意**不**继承 NotConnectedError —— 它不是一次可重连恢复的掉线")

    entries = {
        "get_state": lambda: arm.get_state(),
        "get_state(refresh)": lambda: arm.get_state(refresh=True),
        "enable": lambda: arm.enable(),
        "disable": lambda: arm.disable(),
        "movej": lambda: arm.movej([0.0] * 7),
        # ⚠ 这三条是**带本地 arity/idx 预检**的入口: 它们把 `self.n` 检查排在 `_cmd()`
        #   之前, 而终态下 `self.n` 已被 `close()` 清成 0 ⇒ 少了各自最前那句守卫就会
        #   抛 `InvalidCommandError("q 需 N 个")` (N=0)、把人引向一个**不存在**的 arity
        #   bug, 并让文档给的"捕获 `ArmIsInDfuError` → 新建 `Arm`"迁移姿势漏掉它们。
        #   放进本表 = 与其余入口**同判据** (见 `_reject_if_in_dfu` 的 docstring)。
        "move_js": lambda: arm.move_js([0.0] * 7),
        "send_mit": lambda: arm.send_mit(0, 0.0, 0.0, 50.0, 2.0, 0.0),
        "send_mit_all": lambda: arm.send_mit_all([0.0] * 7, [0.0] * 7, [50.0] * 7,
                                                 [2.0] * 7, [0.0] * 7),
        "get_tcp": lambda: arm.get_tcp(),
        "save_params": lambda: arm.save_params(),
        "params.get_joint_param": lambda: arm.params.get_joint_param(0),
        "log.start": lambda: arm.log.start(4),
        "diag.kin_bench": lambda: arm.diag.kin_bench(),
        "poll_cart": lambda: arm.poll_cart(),
        "zero_g_start": lambda: arm.zero_g_start(),
        "enter_dfu (again)": lambda: arm.enter_dfu(),
        "connect": lambda: arm.connect(),
        "reconnect": lambda: arm.reconnect(),
        # 私有写口也要拦 —— 终态下"不会再有任何帧发出去"必须是**结构性**的事实
        # (`_raw_write` 是唯一写口, 保活线程当年就是绕过 `_write_cmd` 走它的),
        # 而不是靠逐入口枚举 (`_require()` 之外还有它这条旁路)。
        "_raw_write (私有写口)": lambda: arm._raw_write(P.CMD_GET_FIRMWARE),
    }
    wrong = []
    for name, fn in entries.items():
        try:
            fn()
            wrong.append(f"{name}: 没报错")
        except pa.ArmIsInDfuError:
            pass
        except Exception as e:           # noqa: BLE001
            wrong.append(f"{name}: {type(e).__name__} (应为 ArmIsInDfuError)")
    assert not wrong, "终端态下这些入口的行为不对: " + "; ".join(wrong)

    arm.close()                          # teardown 永远可用 (且不抛)


def test_a_held_ack_reference_also_reports_the_terminal_state(offline_arm):
    """直接持 `_Ack` 引用那条路 (不经 `_require()`) 也必须报终态。

    终态下 `_tr`/`_a` 都已置 `None`, 所以等待口 (`_Ack._wait`) 顶部那两句守卫
    **必须把终态排在"链路已关"之前** —— 否则报出来的是 `NotConnectedError`
    ("一次可重连恢复的掉线"), 把"我们主动把设备交出去了"说成"没连上"。
    判据与 `tests/test_frame_ownership.py` 里
    `test_a_held_ack_reference_after_close_reports_not_connected` **同形**
    (那条钉的是 close 之后的归因方向, 这条钉的是终态优先于它)。
    """
    arm = offline_arm
    a = arm._a                      # 先抓住引用 —— enter_dfu() 会把 arm._a 一并置 None
    arm.enter_dfu()

    with pytest.raises(pa.ArmIsInDfuError):
        a.expect(P.RSP_FIRMWARE, 0.2, "test", echo_cmd=P.CMD_GET_FIRMWARE)
    with pytest.raises(pa.ArmIsInDfuError):
        a.expect(P.RSP_ACK, 0.2, "test", echo_cmd=P.CMD_ENABLE)


def test_the_vanish_window_does_not_consume_frames(offline_arm):
    """⚠ 消失观察窗口**一帧都不碰链路** —— 窗口里到的帧原样留在自己队列里。

    ⚠ 本用例改过三次名，每次都在追同一件事：**别人的帧不许被这个窗口吃掉**。
      · `..._counts_the_frames_it_drops`（断言 `foreign_frames == 1`）—— 描述的是缺陷本身；
      · `..._preserves_the_frames_it_does_not_use`（断言帧进了暂存盒）—— 暂存盒已随读路径
        重构删除；
      · 现在 —— `Arm._wait_until_link_lost` **根本不读链路**了：它只等
        `_Ack._reader_error` 被置（读线程发现设备消失时写的那一格）。于是"窗口吃掉别人的帧"
        这条**路径不存在**：帧由读线程投进各自的队列，谁也拿不走。

    ⚠ 判据钉的是**更强**的那件事：那条 ACK **原样躺在它自己的队列里**（不是"被存起来了"，
    而是"根本没被碰过"），且 `dropped` 没涨。
    """
    arm = offline_arm
    arm._tr.dfu_vanishes = False
    # 桩: 登记 ACK 之后、设备离开之前还会投递的帧 (这里塞一条别人的 ACK)
    arm._tr.dfu_post_ack_frames = [(P.RSP_ACK, bytes([0x11]))]

    with pytest.raises(pa.LiteArmError):
        arm.enter_dfu()

    # 等读线程把它投递掉（消失窗口本身不驱动读，读线程一直在读）
    import time as _t
    for _ in range(200):
        if arm._a._queues.get((P.RSP_ACK, 0x11)):
            break
        _t.sleep(0.002)

    assert arm._a._queues.get((P.RSP_ACK, 0x11)) == [(P.RSP_ACK, bytes([0x11]), arm._a._queues[(P.RSP_ACK, 0x11)][0][2])], (
        f"窗口里那条 ACK 没原样留在自己的队列里: {arm._a._queues}")
    assert arm._a.dropped == 0, "它不该被计成丢弃（没人丢它）"


# ---------------------------------------------------------------- 固件三种回码
def test_enter_dfu_always_sends_an_empty_payload(offline_arm):
    """我们恒发**空载荷** ⇒ 固件的 `{0x15,0x01}` 那一档够不到 (`usb_cmd.c:547`)。

    ⚠ 这句话的**依据**是紧邻的 `test_the_stub_models_the_length_gate_verbatim` ——
    它拿**裸帧**证明桩真的会对非空载荷回 `ERR{0x15,0x01}`; 少了那一条, 这里的
    "够不到"就只是一句没有观测面的断言 (桩把那一档删掉, 本用例照样绿)。
    """
    arm = offline_arm
    tr = arm._tr
    arm.enter_dfu()
    assert _dfu_payloads(tr) == [b""]


def test_the_stub_models_the_length_gate_verbatim(offline_arm):
    """⚠ **桩保真度检查** (钉的是 `tests/fake_serial.py` 的 `0x15` 分支, 不是 SDK 行为)。

    固件 `usb_cmd.c:543-555` 的 `case CMD_ENTER_DFU` 是**长度优先**: 载荷非空 ⇒
    `ERR{0x15,0x01}`, 连门禁都不看。桩必须照抄这一档 —— 它是
    `test_enter_dfu_always_sends_an_empty_payload` 那句"我们恒发空载荷 ⇒ 那一档够不到"
    的**唯一**依据。

    判据只能走**裸帧**: SDK 入口 `enter_dfu()` **恒发空载荷** (上一条用例), 所以经 SDK
    永远够不到这一档 —— 实测把桩里 `if payload:` 改成 `if False:` (等价于删掉这一档),
    **全量 531 passed 全绿**, 没有任何用例报错。
    """
    tr = offline_arm._tr
    tr.write_frame(P.CMD_ENTER_DFU, b"\x00")        # 非空载荷 ⇒ 固件那一档
    got = tr.read_frame(0.2)
    assert got is not None, "桩对非空载荷一个字节都没回"
    assert (got[0], bytes(got[1])) == (P.RSP_ERR, bytes([P.CMD_ENTER_DFU, 0x01])), (
        f"桩没有照固件 `usb_cmd.c:547` 回 ERR{{0x15,0x01}} (实测回的是 {got!r}) —— "
        f"少这一档, 「SDK 恒发空载荷」后面的那句『那一档够不到』就没有依据了"
    )


def test_err_0x02_is_surfaced_verbatim(offline_arm):
    """`ERR{0x15,0x02}` (ROM 向量表无效) ⇒ **照实透出** —— 它是**硬件故障**, 不是用法
    错误, 更不是"登记被撤销" (`control_loop.c:1092-1095`: 当场拒, 不等安全点)。"""
    arm = offline_arm
    arm._tr.dfu_rom_table_invalid = True

    with pytest.raises(pa.CommandRejectedError) as ei:
        arm.enter_dfu()

    assert (ei.value.cmd, ei.value.code) == (P.CMD_ENTER_DFU, 0x02)
    assert "向量表" in str(ei.value), f"没照实透出固件的归因: {ei.value}"
    assert "撤销" not in str(ei.value), (
        "把'向量表坏了'吞成了'登记被撤销' —— 两者一个是硬件故障、一个是时序竞态")
    assert arm.get_state(refresh=True).value is not None, (
        "请求根本没被受理 ⇒ 不该置终态")


def test_err_0x03_from_enable_pending_is_surfaced_and_is_not_a_revocation(offline_arm):
    """`ERR{0x15,0x03}` —— **预检够不到的那一半**: `enable_pending` 在途。

    桩只置 `dfu_armed_pending` (固件侧那个"已武装但 `enabled` 还没落地"的窗口,
    `control_loop.c:1091` **内联**的那条 `|| enable_pending` —— 与 `:1334-1336` 的
    `ctrl_is_armed()` 同义), 于是 `st.enabled` 仍为 False ⇒
    预检**放行**、由固件回 `0x03`。

    **处置 (写死)**: 它是**用法错误**的照实透出 —— `CommandRejectedError` 带
    `.code == 0x03`, 文本取 `errors.ERR_TEXT[(0x15,0x03)]` (已写明"使能中, 或使能在途;
    跳转会停 TIM3...下垂"); **不算**"登记被撤销" —— 那一刻固件**还没登记**
    (`ctrl_request_enter_dfu` 在 `dfu_pending = true` 之前就返回了), 所以既没有
    "ACK 成功了却什么都没发生"这回事, 也**不该**置终态。
    """
    arm = offline_arm
    arm._tr.dfu_armed_pending = True     # 固件侧 enable_pending 在途 (enabled 仍 False)

    with pytest.raises(pa.CommandRejectedError) as ei:
        arm.enter_dfu()

    assert (ei.value.cmd, ei.value.code) == (P.CMD_ENTER_DFU, 0x03)
    assert "撤销" not in str(ei.value), (
        "把'使能在途, 请求未被受理'读成了'登记被撤销' —— 后者意味着 ACK 过了")
    assert arm.get_state(refresh=True).value is not None, "这条拒绝不该置终态"
    arm.enable()                         # 照旧可用 (用户该做的就是等使能落地/失能后重来)
