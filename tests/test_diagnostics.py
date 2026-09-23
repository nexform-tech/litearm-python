"""KIN_BENCH 运动学开销自测 (固件 0x49 -> RSP_KIN_BENCH 0x4A) —— 此前 SDK 无入口。

固件侧 (`kin_runner.c` 的 `kin_bench_run`, `:335-564`; LINK 行在 `:514`): 后台 DWT-cycle
测 FK/JAC/IK/S 曲线/重力/RNEA/M/控制律
单拍开销, 最终文本经 `RSP_KIN_BENCH(0x4A)` 可靠通道回。**最后一行 LINK 是拿到
`usb_cmd_crc_err_count` (CRC 坏帧) / `usb_cmd_reply_dropped` (应答 FIFO 丢) /
`fdcanx_tx_fail_count` (CAN TX 失败) 等诊断计数的唯一通道** —— 这些计数在固件里
原本全部静默 (L3/M1 fix)。

文本格式 (逐行):
  FK<n> <avg_cyc> <max_cyc>   / JAC / IK / LOOP1 / SC / G / RNEA / M / LAW
  LINK crc=.. ovf=.. rfd=.. txf=.. tfe=.. tfd=.. tfc=.. tfs=.. tfm=.. loop_k=.. ovr=.. flt=<tick>/<cause>st=<0|1>
"""
from __future__ import annotations

import pytest

from litearm import _protocol as P
from litearm.errors import TransportError, UnsupportedByFirmwareError


def test_kin_bench_returns_raw_text(offline_arm):
    r = offline_arm.diag.kin_bench().value
    assert "FK200" in r.raw
    assert r.raw.splitlines()[0] == "FK200 108 240 "
    assert str(r) == r.raw


def test_kin_bench_parses_per_block_timings(offline_arm):
    """⚠ 固件把块名与计数**相连** (`FK200 108 240 `), `LOOP1` 行是 `LOOP14000 ` ——
    名字里本身带数字, 故按固件已知块名集合切分, 不能用通用正则。"""
    r = offline_arm.diag.kin_bench().value
    assert r.timings["FK"] == (200, 108, 240)
    assert r.timings["RNEA"] == (200, 800, 1500)
    assert r.timings["IK"] == (100, 2600, 5200)
    assert r.timings["LOOP1"] == (4000,)


def test_kin_bench_parses_link_diagnostics(offline_arm):
    """链路诊断是 KIN_BENCH 最实用的产出 —— 必须逐个解析出来。"""
    link = offline_arm.diag.kin_bench().value.link
    assert link["crc"] == 3            # CRC 坏帧累计
    assert link["rfd"] == 1            # 应答 FIFO 队满丢弃
    assert link["txf"] == 7            # CAN TX 失败
    assert link["tfc"] == 3            # TX 失败分类桶 c
    assert link["loop_k"] == 2         # 控制拍峰值 (千 cycle)
    assert link["ovr"] == 4            # 超预算拍数
    assert link["fault_tick"] == 12345
    assert link["fault_cause"] == 6
    assert link["store_from_flash"] is True


def test_kin_bench_parses_link_keys_that_contain_digits(offline_arm):
    """⚠ `rxl0` / `rxl1` **名字里带数字** —— 解析器必须吃得下。

    它们不是可有可无的边角: 固件 `kin_runner.c:520-522` 明写
    "`[N-12 fix]` RX FIFO 溢出 (原为零遥测): **溢出 = 静默丢反馈 = 可能整臂 EMERGENCY**"。
    即**最该看见的那两个计数器**。

    而 `_KV_RE` 从前是 `[a-z_]+` —— 匹配不到带数字的键, 于是那两个**静默丢失**,
    三个出口恒返回 0, 与"没丢过"长得一模一样。

    ⚠ 这是**同一个文件里第二次**栽在"名字里带数字"上: `_TIMING_RE` 段落早就写过
    `LOOP1` 行是 `LOOP14000`, "名字里本身带数字…不能用通用正则"。
    """
    link = offline_arm.diag.kin_bench().value.link
    assert "rxl0" in link, "`rxl0` 没解析出来 —— 字符类又写窄了"
    assert "rxl1" in link, "`rxl1` 没解析出来 —— 字符类又写窄了"
    assert link["rxl0"] == 13
    assert link["rxl1"] == 17

    r = offline_arm.diag.kin_bench().value
    assert r.rx_fifo_lost_motor == 13
    assert r.rx_fifo_lost_bridge == 17
    assert r.gsusb_ring_drops == 19


def test_kin_bench_reads_the_second_link_frame(offline_arm):
    """⚠ 固件的回执是**连续两帧** (第 1 帧各项耗时 / 第 2 帧 LINK 诊断行)。

    只收第 1 帧会**静默**丢掉全部链路计数 —— 那五个便捷出口是 `link.get(k, 0)`,
    拿不到就是 **0**, 与"没有出错"分不开 (真机上实测踩过: 回执里没有 LINK 行,
    `link == {}`, 五个出口全报 0)。

    桩已按真机拆帧。**本用例的判别力正来自这个桩**: 若桩仍把全文塞一帧,
    它会在"只收一帧"的实现上照样通过。
    """
    r = offline_arm.diag.kin_bench().value
    assert r.link, "第 2 帧 (LINK 行) 没被收下 —— 五个链路计数会静默全 0"
    assert r.crc_errors == 3
    assert r.reply_dropped == 1
    assert r.can_tx_fail == 7
    assert r.loop_max_kcycle == 2
    assert r.loop_overruns == 4


def test_kin_bench_tolerates_old_firmware_with_only_one_frame(offline_arm):
    """旧固件只发一帧 (无 LINK 行): 第 2 帧等超时**不是错误**。

    兼容分支要**静默降级** —— 前一半 (各项耗时) 仍必须可用, 不能因此抛异常。
    """
    arm = offline_arm
    arm._tr.kin_bench_one_frame = True
    r = arm.diag.kin_bench().value
    assert r.timings["FK"] == (200, 108, 240)
    assert r.link == {}            # 拿不到就是拿不到 —— 但降级不许抛
    assert r.crc_errors == 0       # 静默 0 是这一刻的诚实结果, 不是"没出错"


def test_kin_bench_empty_reply_raises(offline_arm):
    """空文本 = 非正常回执, 不能当成"全 0"静默返回。"""
    arm = offline_arm
    arm._tr.kin_bench_text = ""
    with pytest.raises(TransportError):
        arm.diag.kin_bench()


def test_kin_bench_unsupported_on_old_firmware(offline_arm):
    arm = offline_arm
    arm._tr.unknown_cmds.add(P.CMD_KIN_BENCH)
    with pytest.raises(UnsupportedByFirmwareError):
        arm.diag.kin_bench()
