"""协议漂移防护 —— SDK 与固件头文件的**双向强制比对**。

为什么必须有这个文件: 上一次栽的跟头就是「命令/帧布局改了, SDK 没跟上」——
1.5.x 把状态帧从 `4+21N` 改成 `6+21N` 时漏同步本 SDK, 而桩测试整替换 transport、
自造旧布局, 结果**离线 293 绿、真机必挂**。人工核对挡不住这种事, 只有把固件头文件
当输入喂给测试才挡得住。

做法: 直接解析固件仓库的头文件源码 (不依赖固件编译), 断言:
  1. 固件每条 `CMD_*` 在 SDK `_protocol` 里都有同名同值常量 (反向亦然);
     **两类例外, 语义不同**: ① `_protocol.FIRMWARE_ONLY_CMDS` /
     `FIRMWARE_ONLY_RSPS` 里的条目豁免 —— 这两张表**历史上**装的是"用户裁决 SDK
     **有意不暴露**"的命令/应答 (笛卡尔 0x3A~0x3E / 0x4E 曾是这里的例子), ⚠ **现在两张
     都是空的**: 段二已给笛卡尔补上常量与入口, 于是"豁免"不再成立
     (空表本身就是"当前没有任何固件单边命令"的断言载体);
     ② **既有的**固件/SDK 不同步 (目前 1 条, 见 `_protocol.PREEXISTING_GAPS`) ——
     只豁免不修复。
     两类豁免的有效性都由 `test_firmware_only_exemption_is_still_valid` 单独守
     (不是永久白名单);
  2. 固件每条**已实现**命令在 `COMMAND_COVERAGE` 里都有 SDK 入口, 且该入口
     `getattr` 得到 (覆盖"无死角"这件事本身);
  3. 固件标注「未实现」的命令仍然标注着 (固件一旦实现, 这里立刻红, 提醒补入口);
  4. 状态帧布局表达式未被改动;
  5. `joint_cfg.h` 的 `LITEARM_BENCH_MODEL_AXIS` 与 SDK 常量一致;
  6. `ERR{cmd,0x00}` 在固件里仍**只**由 `default` 分支产生 —— 这条守的是
     「用 ERR 码判定固件能力」这个核心假设 (见 test_capability.py);
  7. 固件自报版本不低于 SDK 要求的 `MIN_FW`。

固件仓库位置: 环境变量 `LITEARM_FW_DIR`, 默认 `~/litearm-stm32`。
**找不到时 skip, 不是 pass** —— 假绿比没测更糟。
"""
from __future__ import annotations

import os
import re

import pytest

from litearm import _protocol as P
from litearm import arm as arm_mod

FW_DIR = os.environ.get("LITEARM_FW_DIR") or os.path.expanduser("~/litearm-stm32")
USB_CMD_H = os.path.join(FW_DIR, "User/litearm/hal/usb_cmd.h")
USB_CMD_C = os.path.join(FW_DIR, "User/litearm/hal/usb_cmd.c")
LITEARM_H = os.path.join(FW_DIR, "User/litearm/litearm.h")
JOINT_CFG_H = os.path.join(FW_DIR, "User/litearm/params/joint_cfg.h")


def _read(path: str) -> str:
    if not os.path.isfile(path):
        pytest.skip(f"固件源码不在 {path} —— 设 LITEARM_FW_DIR 指向 litearm-stm32 仓库")
    with open(path, encoding="utf-8", errors="replace") as f:
        return f.read()


@pytest.fixture(scope="module")
def hdr():
    return _read(USB_CMD_H)


def _fw_defines(hdr_src: str, prefix: str):
    """`#define <prefix>NAME 0xNN` -> {名字: (值, 该行其余内容)}。

    ⚠ 空白一律用 `[ \\t]` 而非 `\\s` —— `\\s` 会跨越换行, 于是 `(.*)$` 会把**下一行**
    整行吞掉, 使相邻的 #define 交替漏匹配 (本文件 34 条只命中 26 条)。
    """
    out = {}
    pat = (r"^#define[ \t]+(" + prefix + r"\w+)[ \t]+(0x[0-9A-Fa-f]+)[ \t]*(.*)$")
    for m in re.finditer(pat, hdr_src, re.M):
        out[m.group(1)] = (int(m.group(2), 16), m.group(3))
    return out


def _fn_body(src: str, name: str):
    """取出 `static void <name>(void) { ... }` 的函数体 (用于把解析限定在单个函数内)。"""
    m = re.search(r"static void[ \t]+" + re.escape(name) + r"\(void\)[ \t]*\{(.*?)\n\}",
                  src, re.S)
    return m.group(1) if m else None


# ---------------------------------------------------------------- 1. 常量双向
def test_every_firmware_cmd_has_sdk_constant(hdr):
    """固件每条已实现命令都要在 SDK 里有同名同值常量 (未实现的那两条按豁免跳过,
    其豁免有效性由 test_unimplemented_exemption_is_still_true 单独守)。

    ⚠ 这张表按**名字**豁免 (下面的 `if name in exempt`) —— 与
    `test_every_implemented_cmd_has_reachable_api` 的**码值**口径不同, 改一处必须改两处;
    这里并进去的必须是 FIRMWARE_ONLY_CMDS / PREEXISTING_GAPS 的 **values()（名字）**,
    键是码值。
    """
    fw = _fw_defines(hdr, "CMD_")
    exempt = (set(P.UNIMPLEMENTED_CMDS.values())
              | set(P.FIRMWARE_ONLY_CMDS.values())
              | set(P.PREEXISTING_GAPS.values()))
    missing = []
    for name, (val, _) in fw.items():
        if name in exempt:
            continue
        got = getattr(P, name, None)
        if got != val:
            missing.append(f"{name}=0x{val:02X} (SDK: {got!r})")
    assert not missing, "SDK _protocol 缺少/值不符的固件命令: " + ", ".join(sorted(missing))


def test_sdk_has_no_command_the_firmware_lacks(hdr):
    fw_names = set(_fw_defines(hdr, "CMD_"))
    sdk_names = {n for n in dir(P) if n.startswith("CMD_")}
    extra = sdk_names - fw_names
    assert not extra, f"SDK 声明了固件没有的命令 (漂移): {sorted(extra)}"


def test_every_firmware_rsp_has_sdk_constant(hdr):
    """⚠ 本条原本**没有豁免路径**, 笛卡尔那次新增了一条 —— 与上面第一条同口径
    （按**名字**判, 故并进去的是 FIRMWARE_ONLY_RSPS.values()）。"""
    fw = _fw_defines(hdr, "RSP_")
    exempt = set(P.FIRMWARE_ONLY_RSPS.values())
    missing = [f"{n}=0x{v:02X}" for n, (v, _) in fw.items()
               if n not in exempt and getattr(P, n, None) != v]
    assert not missing, "SDK _protocol 缺少/值不符的固件应答: " + ", ".join(sorted(missing))


# ---------------------------------------------------------------- 2. 入口可达
def _resolve(instance, dotted: str):
    """解析 `Arm.params.set_joint_param` 形式的入口路径 (首段 `Arm` 从实例起算)。"""
    parts = dotted.split(".")
    if parts and parts[0] == "Arm":
        parts = parts[1:]
    obj = instance
    for part in parts:
        obj = getattr(obj, part)
    return obj


def test_every_implemented_cmd_has_reachable_api(hdr):
    """「无死角」: 固件每条已实现命令都要能在 Arm 上摸到入口。

    ⚠ 本条按**码值**判 (`val in ...`, 因为它拿不到名字那条路径的豁免集合) ——
    与 `test_every_firmware_cmd_has_sdk_constant` 的**名字**口径不同, 两者都要改。
    """
    fw = _fw_defines(hdr, "CMD_")
    arm = arm_mod.Arm()                      # 不连接: 只解析属性/方法是否存在
    problems = []
    for name, (val, _) in sorted(fw.items(), key=lambda kv: kv[1][0]):
        if (val in P.UNIMPLEMENTED_CMDS or val in P.FIRMWARE_ONLY_CMDS
                or val in P.PREEXISTING_GAPS):
            continue
        entry = P.COMMAND_COVERAGE.get(val)
        if entry is None:
            problems.append(f"0x{val:02X} {name}: 未登记 SDK 入口")
            continue
        try:
            target = _resolve(arm, entry)
        except AttributeError as e:
            problems.append(f"0x{val:02X} {name}: 入口 {entry} 不存在 ({e})")
            continue
        if not callable(target):
            problems.append(f"0x{val:02X} {name}: 入口 {entry} 不可调用")
    assert not problems, "覆盖缺口: " + "; ".join(problems)


def test_coverage_map_has_no_stale_entries(hdr):
    """反向: 覆盖表里不许有固件已不存在的命令 id (否则是过期登记)。"""
    fw_ids = {v for v, _ in _fw_defines(hdr, "CMD_").values()}
    stale = [f"0x{k:02X}" for k in P.COMMAND_COVERAGE if k not in fw_ids]
    assert not stale, f"COMMAND_COVERAGE 有过期登记: {sorted(stale)}"


def test_unimplemented_exemption_is_still_true(hdr):
    """豁免不是永久白名单: 固件一旦实现, 头文件里的「未实现」标注会消失 -> 这里红。"""
    fw = _fw_defines(hdr, "CMD_")
    still_exempt = []
    for val, name in P.UNIMPLEMENTED_CMDS.items():
        assert name in fw, f"{name} 已从固件头文件消失, 应从豁免表移除"
        assert fw[name][0] == val, f"{name} 的 id 变了: 0x{fw[name][0]:02X} != 0x{val:02X}"
        if "未实现" in fw[name][1]:
            still_exempt.append(name)
    assert len(still_exempt) == len(P.UNIMPLEMENTED_CMDS), (
        "固件已经实现了原来豁免的命令: " +
        ", ".join(sorted(set(P.UNIMPLEMENTED_CMDS.values()) - set(still_exempt))) +
        " —— 请补 SDK 入口并从 UNIMPLEMENTED_CMDS 移除")


def test_firmware_only_exemption_is_still_valid(hdr):
    """三张豁免表都不是永久白名单: 固件侧若删掉/改名/换号, 这条必须红。

    守 `_protocol` 的 `FIRMWARE_ONLY_CMDS` / `FIRMWARE_ONLY_RSPS` / `PREEXISTING_GAPS`。
    ⚠ 三张表的**语义不同**, 别以为一条判据就是一个意思:
      · 前两张 (`FIRMWARE_ONLY_CMDS`/`FIRMWARE_ONLY_RSPS`) **现在都是空表** —— 它们的
        **历史**语义是"用户裁决 SDK **有意不暴露**"（"不做", 不是"还没做"; 笛卡尔
        0x3A~0x3E/0x4E 曾是例子, 段二已暴露并补上入口）。表空 ⇒ 下面两个循环一条都不跑,
        本判据对它们只是"**表里若有东西**, 固件侧必须仍存在且 SDK 还没实现";
      · `PREEXISTING_GAPS` = **既有的固件/SDK 不同步**（DFU）, 本应同步而未同步。
      判据相同（固件里必须仍有这几条）不等于理由相同。

    ⚠ 与 `test_unimplemented_exemption_is_still_true` 同一意图, 但判据**相反**:
    那条要求固件注释里**仍**写着"未实现", 这条要求固件里**确实有**这些命令。

    ⚠ 它同时守反向: 一旦 SDK 补上了同名常量（= 真的实现了入口）, 豁免就该撤掉,
    否则豁免表会变成"谁都看不见的过期登记"。

    ⚠ 本守卫读的是**磁盘上的固件树**（`LITEARM_FW_DIR`）, 所以对着**没有这些命令的
    固件分支**（如 master）跑会红 —— **这是预期**: 它抓的正是"固件有、SDK 无"。
    刻意不做"区分换分支与真删了"（那是推测性复杂度）。
    """
    fw = _fw_defines(hdr, "CMD_")
    for code, name in P.FIRMWARE_ONLY_CMDS.items():
        assert name in fw, (
            f"豁免表里的 {name}(0x{code:02X}) 在固件头文件里已不存在 —— "
            f"请从 _protocol.FIRMWARE_ONLY_CMDS 移除")
        assert fw[name][0] == code, (
            f"{name} 在固件里是 0x{fw[name][0]:02X}, 豁免表写的是 0x{code:02X} —— 请同步")
        assert not hasattr(P, name), f"{name} 已被 SDK 实现, 请从豁免表移除"

    fwr = _fw_defines(hdr, "RSP_")
    for code, name in P.FIRMWARE_ONLY_RSPS.items():
        assert name in fwr, (
            f"豁免表里的 {name}(0x{code:02X}) 在固件头文件里已不存在 —— "
            f"请从 _protocol.FIRMWARE_ONLY_RSPS 移除")
        assert fwr[name][0] == code, (
            f"{name} 在固件里是 0x{fwr[name][0]:02X}, 豁免表写的是 0x{code:02X} —— 请同步")
        assert not hasattr(P, name), f"{name} 已被 SDK 实现, 请从豁免表移除"

    # PREEXISTING_GAPS 同样要守: 它记的是"**本应同步而未同步**"的既有缺口,
    # 一旦固件侧消失或 SDK 补上了入口, 豁免就该撤 —— 否则它会变成没人看得见的过期登记。
    for code, name in P.PREEXISTING_GAPS.items():
        assert name in fw, (
            f"PREEXISTING_GAPS 里的 {name}(0x{code:02X}) 在固件头文件里已不存在 —— "
            f"请从 _protocol.PREEXISTING_GAPS 移除")
        assert fw[name][0] == code, (
            f"{name} 在固件里是 0x{fw[name][0]:02X}, PREEXISTING_GAPS 写的是 0x{code:02X} —— 请同步")
        assert not hasattr(P, name), (
            f"{name} 已被 SDK 实现, 请从 PREEXISTING_GAPS 移除（缺口已补上）")


# ---------------------------------------------------------------- 3. 状态帧布局
def test_status_frame_layout_unchanged():
    """状态帧载荷长度表达式必须仍是 `6 + N*21` (SDK 解析器按此硬编码)。"""
    body = _fn_body(_read(USB_CMD_C), "usb_cmd_report_status")
    assert body, "在 usb_cmd.c 里找不到 usb_cmd_report_status 函数体"
    m = re.search(r"uint8_t[ \t]+payload[ \t]*\[[ \t]*([^\]]+)\]", body)
    assert m, "找不到状态帧 payload 声明"
    expr = re.sub(r"\s+", " ", m.group(1)).strip()
    assert expr == "6 + LITEARM_NUM_JOINTS * 21", (
        f"固件状态帧布局变了: `{expr}` —— SDK decode_status 的 6+21N 假设需同步; "
        f"历史上正是这类改动漏同步导致离线全绿真机必挂")


def test_status_frame_joint_stride_matches_sdk_constant():
    """每关节字节数 (5×f32 + 1×err = 21) 必须与 SDK 的步长一致。"""
    body = _fn_body(_read(USB_CMD_C), "usb_cmd_report_status")
    assert body, "找不到 usb_cmd_report_status 函数体"
    m = re.search(r"for[ \t]*\(int i = 0; i < LITEARM_NUM_JOINTS; i\+\+\)[ \t]*\{(.*?)\n    \}",
                  body, re.S)
    assert m, "找不到状态帧的逐关节打包循环"
    seg = m.group(1)
    n_floats = len(re.findall(r"f32_to_le\(", seg))
    n_u8 = len(re.findall(r"\*pp\+\+[ \t]*=", seg))
    assert n_floats * 4 + n_u8 == 21, (
        f"每关节 {n_floats} 个 f32 + {n_u8} 个 u8 = {n_floats * 4 + n_u8}B, "
        f"与 SDK 的 21B 步长不符")


# ---------------------------------------------------------------- 4. 台架常量
def test_bench_model_axis_matches_sdk_constant():
    src = _read(JOINT_CFG_H)
    m = re.search(r"#if\s+LITEARM_BENCH_1J(.*?)#else", src, re.S)
    assert m, "joint_cfg.h 结构变了 (找不到 #if LITEARM_BENCH_1J ... #else)"
    ax = re.search(r"#define\s+LITEARM_BENCH_MODEL_AXIS\s+(\d+)", m.group(1))
    assert ax, "找不到台架分支的 LITEARM_BENCH_MODEL_AXIS"
    assert int(ax.group(1)) == P.BENCH_MODEL_AXIS, (
        f"固件台架模型轴 = {ax.group(1)}, SDK BENCH_MODEL_AXIS = {P.BENCH_MODEL_AXIS} "
        f"—— IK 种子会填错轴")


# ---------------------------------------------------------------- 5. 能力判定假设
def test_err_code_zero_only_comes_from_default_branch():
    """`ERR{cmd,0x00}` 必须**只**由未实现命令的 default 分支产生 ——
    SDK 的 UnsupportedByFirmwareError 判定完全建立在这个假设上。"""
    src = _read(USB_CMD_C)
    codes = re.findall(r"RSP_ERR,\s*\(const uint8_t\[\]\)\{([^}]*)\}", src)
    zeros = [c for c in codes if c.strip().endswith("0x00")]
    assert len(zeros) == 1, (
        f"固件里有 {len(zeros)} 处回 ERR 码 0x00 (期望恰好 1 处, 即 default 分支): "
        f"{zeros} —— SDK 会把它误判成「固件不支持该命令」")
    assert re.search(r"default:\s*\n\s*usb_cmd_reply\(RSP_ERR,\s*\(const uint8_t\[\]\)\{cmd, 0x00\}",
                     src), "那唯一一处 0x00 不在 default 分支里"


# ---------------------------------------------------------------- 6. 版本门
def test_firmware_version_not_below_sdk_minimum():
    src = _read(LITEARM_H)
    vers = re.findall(r'#define\s+LITEARM_FW_VERSION\s+"([^"]+)"', src)
    assert vers, "找不到 LITEARM_FW_VERSION"
    parsed = [P.parse_firmware_version(v) for v in vers]
    assert all(p is not None for p in parsed), f"版本串不符合约定: {vers}"
    for v, p in zip(vers, parsed):
        assert p[:3] >= arm_mod.MIN_FW, (
            f"固件自报 {v} 低于 SDK 要求的 "
            f"{'.'.join(map(str, arm_mod.MIN_FW))}")
