"""文档里的代码必须是对的 —— 逐块校验 `*.md` 的 python 代码块。

文档示例写错是**静默**故障: 不跑就发现不了。本用例把 8 份文档里所有 ```python 块
抽出来, 按**内容**分三类处理 (不按块序号 —— 序号会随文档编辑错位):

* **可执行示例** —— 接到离线桩 `Arm` 上真跑一遍, 所以不会碰真机。
  块里出现任何没交代的名字 (例如留了 `p1` / `tcp` / `via_pose` 这种占位符) 都会
  `NameError` 而失败。
* **签名清单** —— `foo(a, b=1) -> T` 这种不执行, 改为**逐条比对 `inspect.signature`**:
  参数名、位置顺序、关键字专用性 (`*`)、默认值全都要对得上。
  只读属性名单 (`n` / `q_tol` / …) 也走这条, 逐个确认成员真的存在。
* **说明性片段** —— 会 fork、会阻塞、或本身就是类型定义示意, 见 `ILLUSTRATIVE`,
  只做语法检查。

**新增示例若跑不起来, 本用例直接失败。** 别把只是写错的块塞进 `ILLUSTRATIVE` 蒙混过去 ——
那条正则只匹配"本来就不是可执行代码"的块。
"""
from __future__ import annotations

import ast
import inspect
import os
import re

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)

from litearm.arm import Arm  # noqa: E402 - conftest 已在更早处设好 sys.path

#: ⚠ `conftest.py` 那条 autouse 的读线程哨兵会把 `Arm.__init__` 换成
#: `(self, *args, **kwargs)` 的包装, 于是**测试期里 `inspect.signature(Arm.__init__)`
#: 再也读不到真实签名**。这里在补丁生效前先把原始函数抓住。
_ARM_INIT_PRISTINE = Arm.__init__

DOCS = [
    "README.md",
    "README.zh-CN.md",
    "TROUBLESHOOTING.md",
    "TROUBLESHOOTING.zh-CN.md",
    "docs/DEVELOPER_GUIDE.md",
    "docs/DEVELOPER_GUIDE.zh-CN.md",
    "examples/README.md",
    "examples/README.zh-CN.md",
]

#: 只做语法检查的块 —— 按内容判, 不按序号。
#: ⚠ 别加 `re.X` / 行尾 `#` 注释: `re.X` 下第一个 `#` 会把**后面所有分支**注释掉。
ILLUSTRATIVE = re.compile(
    r"\bmultiprocessing\b"     # fork 示例, 会真的开子进程
    r"|\binput\s*\("           # 交互示例, 会阻塞
    r"|^\s*@\w+",              # 类型定义示意 (Msg 的 dataclass)
    re.M,
)

#: `foo(...)` / `group.foo(...)`, 允许尾随的 `-> 返回类型`
_SIG = re.compile(r"^(?:([A-Za-z_]\w*)\.)?([A-Za-z_]\w*)\((.*)\)\s*(?:->\s*.+)?$", re.S)

#: 裸属性引用 —— `r.total_bytes` / `n`
_ATTR = re.compile(r"^(?:([A-Za-z_]\w*)\.)?([A-Za-z_]\w*)$")

#: 一个"像参数"的形参: 裸标识符 / `*` 或 `/` 分隔符 / `**name` / `name=任何字面量`
_PARAM_OK = re.compile(r"^(\*$|/$|\*{0,2}[A-Za-z_]\w*(=.*)?)$", re.S)

_EVAL_NS: dict = {}


def _blocks():
    for rel in DOCS:
        path = os.path.join(_ROOT, rel)
        if not os.path.exists(path):
            continue
        text = open(path, encoding="utf-8").read()
        for i, body in enumerate(re.findall(r"```python\n(.*?)```", text, re.S), 1):
            yield rel, i, body


ALL_BLOCKS = list(_blocks())
IDS = [f"{r}#{i}" for r, i, _ in ALL_BLOCKS]


# ---------------------------------------------------------------- 文本切分

def _strip_comment(line: str) -> str:
    """去掉行尾注释 —— 文档里的 `# 需 10 个值` 之类不算代码。"""
    out, quote = [], None
    for ch in line:
        if quote:
            out.append(ch)
            if ch == quote:
                quote = None
            continue
        if ch in "\"'":
            quote = ch
            out.append(ch)
            continue
        if ch == "#":
            break
        out.append(ch)
    return "".join(out).rstrip()


def _entries(body: str):
    """把块切成逻辑条目: 括号未闭合的续行并进上一条。"""
    buf, depth = "", 0
    for raw in body.splitlines():
        line = _strip_comment(raw)
        if not line and not buf:
            continue
        buf = f"{buf} {line}".strip() if buf else line
        depth += line.count("(") - line.count(")")
        if depth <= 0:
            yield buf
            buf, depth = "", 0
    if buf:
        yield buf


def _split_params(text: str):
    """按顶层逗号切参数, 保住嵌套结构。"""
    parts, buf, depth = [], "", 0
    for ch in text:
        if ch in "([{":
            depth += 1
        elif ch in ")]}":
            depth -= 1
        if ch == "," and depth == 0:
            parts.append(buf.strip())
            buf = ""
        else:
            buf += ch
    if buf.strip():
        parts.append(buf.strip())
    return [p for p in parts if p]


# ---------------------------------------------------------------- 名字解析

def _sub_objects():
    from litearm.diagnostics import Diagnostics
    from litearm.log import ArmLog, LogReader
    from litearm.model import ModelParams
    from litearm.params import JointParams

    return (JointParams, ModelParams, ArmLog, LogReader, Diagnostics)


def _group_of(selector: str | None):
    """`params.` / `model.` / `log.` / `diag.` / `r.` 前缀 → 对应的子对象类。"""
    from litearm.diagnostics import Diagnostics
    from litearm.log import ArmLog, LogReader
    from litearm.model import ModelParams
    from litearm.params import JointParams

    return {
        "params": JointParams, "model": ModelParams,
        "log": ArmLog, "diag": Diagnostics,
        "r": LogReader, "reader": LogReader,
    }.get(selector or "")


def _has_member(cls, name: str) -> bool:
    """类上有这个成员吗 —— **含 `__init__` 里赋的实例属性** (`total_bytes` 就是这种)。

    `hasattr(cls, ...)` 看不见实例属性, 所以要再扫一遍类源码里的 `self.<name>`。
    """
    if hasattr(cls, name):
        return True
    try:
        src = inspect.getsource(cls)
    except (OSError, TypeError):  # pragma: no cover
        return False
    return re.search(rf"self\.{re.escape(name)}\s*[:=]", src) is not None


def _owner_of(name: str, selector: str | None):
    """这个成员属于哪个类。带前缀就认前缀; 裸名字先找 `Arm`, 再在各子对象里唯一命中。"""
    if selector:
        return _group_of(selector) or Arm
    if _has_member(Arm, name):
        return Arm
    hits = [c for c in _sub_objects() if _has_member(c, name)]
    return hits[0] if len(hits) == 1 else None


def _verify_member(name: str, selector: str | None, rel: str, idx: int):
    owner = _owner_of(name, selector)
    assert owner is not None and _has_member(owner, name), (
        f"{rel}#{idx}: 文档写了 {name}，但真实 API 里没有")


def _resolve_call(name: str, selector: str | None, rel: str, idx: int):
    if name == "Arm":
        return _ARM_INIT_PRISTINE
    _verify_member(name, selector, rel, idx)
    owner = _owner_of(name, selector)
    return getattr(owner, name)


# ---------------------------------------------------------------- 签名核对

def _check_signature_entry(entry: str, rel: str, idx: int) -> bool:
    """核对一条签名/属性条目。返回 True 表示这一条确实是签名条目。"""
    # 只读属性名单可以一行写几个: `n` / `q_tol` / `dq_tol` … 用 ` / ` 分隔
    if "/" in entry and not _SIG.match(entry):
        for piece in entry.split("/"):
            piece = piece.strip()
            m = _ATTR.match(piece)
            assert m, f"{rel}#{idx}: 认不出的条目 {entry!r}"
            _verify_member(m.group(2), m.group(1), rel, idx)
        return True

    attr = _ATTR.match(entry)
    if attr:
        _verify_member(attr.group(2), attr.group(1), rel, idx)
        return True

    m = _SIG.match(entry)
    if not m:
        return False
    selector, name, raw_params = m.group(1), m.group(2), m.group(3)
    real = inspect.signature(_resolve_call(name, selector, rel, idx))
    real_params = [p for p in real.parameters.values() if p.name != "self"]
    real_by_name = {p.name: p for p in real_params}

    kw_only_from, doc_names = None, []
    for pos, item in enumerate(_split_params(raw_params)):
        if item == "*":
            kw_only_from = len(doc_names)
            continue
        pname = item.split("=", 1)[0].strip().lstrip("*")
        doc_names.append(pname)

        assert pname in real_by_name, (
            f"{rel}#{idx}: 文档写了 {name}({pname}=…)，但真实签名没有这个参数"
            f"（真实参数：{list(real_by_name)}）")

        real_p = real_by_name[pname]
        if kw_only_from is not None and pos >= kw_only_from:
            assert real_p.kind is inspect.Parameter.KEYWORD_ONLY, (
                f"{rel}#{idx}: 文档把 {pname} 写成关键字专用（`*` 之后），"
                f"但真实签名里它是 {real_p.kind.name}")
        elif real_p.kind is inspect.Parameter.KEYWORD_ONLY:
            # 反方向: 真实是关键字专用, 文档却没写 `*` ⇒ 读者会以为能按位置传
            assert item.startswith("*"), (
                f"{rel}#{idx}: {name}() 的 {pname!r} 是**关键字专用**，"
                f"文档的签名清单漏了 `*`，读者会以为能按位置传")

        if "=" in item:
            doc_default = item.split("=", 1)[1].strip()
            try:
                expected = eval(doc_default, dict(_EVAL_NS))  # noqa: S307 - 文档字面量
            except Exception:  # noqa: BLE001 - 求不出来就只查名字
                continue
            assert real_p.default == expected, (
                f"{rel}#{idx}: {name}(… {pname}={doc_default}) 文档写的默认值是 "
                f"{expected!r}，真实是 {real_p.default!r}")

    for p in real_params:
        if p.name in doc_names:
            continue
        assert p.default is not inspect.Parameter.empty or p.kind in (
            inspect.Parameter.VAR_POSITIONAL, inspect.Parameter.VAR_KEYWORD), (
            f"{rel}#{idx}: {name}() 有个必填参数 {p.name!r}，文档的签名清单里没写出来")
    return True


def _looks_like_signature(entry: str) -> bool:
    """一条条目是不是"签名"而非"代码"。

    ⚠ 判据的关键在**形参必须是标识符**: `movej(q, speed=1.0)` 是签名, 而
    `movej([0.1, 0, ...], speed=0.3)` 是**真调用** —— 后者要拿去执行。
    """
    if _ATTR.match(entry):
        return True
    if "/" in entry and not _SIG.match(entry):
        return all(_ATTR.match(p.strip()) for p in entry.split("/"))
    m = _SIG.match(entry)
    return bool(m) and all(_PARAM_OK.match(p) for p in _split_params(m.group(3)))


def _is_signature_block(body: str) -> bool:
    """块里**每条**逻辑条目都是签名/属性 → 当成签名清单。"""
    entries = list(_entries(body))
    return bool(entries) and all(_looks_like_signature(e) for e in entries)


# ---------------------------------------------------------------- 可执行示例

def _make_arm():
    """连上离线桩固件 —— 与 conftest 的 offline_arm 同构。"""
    from fake_serial import FakeTransport

    return Arm(port="fake", transport_factory=lambda port="fake", timeout=0.2, **k:
               FakeTransport(port=port, timeout=timeout, fw="Litearm1.7.0-7J", **k)).connect()


@pytest.fixture
def _stub_port_discovery(monkeypatch):
    """让 `pa.Arm().connect()`（不带 port）在**没接真机**的机器上也能跑。

    ⚠ 必须打这个补丁: `connect()` 里 `find_cdc_port()` 排在传输工厂**之前**
    (`arm.py:919`), 它只枚举 USB 不开设备 —— 但机器上没接臂时它回 `None`,
    紧跟的 `raise TransportError` 会让"最小完整程序"那类示例在 CI 上假失败。
    """
    import litearm.arm as arm_mod

    monkeypatch.setattr(arm_mod, "find_cdc_port", lambda: "fake")


@pytest.fixture(scope="module", autouse=True)
def _eval_ns():
    import litearm
    from litearm.arm import FIRMWARE_PREFIX, MIN_FW

    _EVAL_NS.update({"MIN_FW": MIN_FW, "FIRMWARE_PREFIX": FIRMWARE_PREFIX,
                     "litearm": litearm})


@pytest.mark.parametrize("rel,idx,body", ALL_BLOCKS, ids=IDS)
def test_doc_block(rel, idx, body, fake_transport_factory, _stub_port_discovery):
    fake_transport_factory()          # 把 SerialTransport 换成桩（monkeypatch，自动还原）

    # ⚠ 签名清单要**先**判, 不能放在语法检查后面: 签名清单本来就不是合法 Python
    # (`f(a, *, b=1)` 当语句写是语法错误), 放在后面会被 SyntaxError 直接 return 掉,
    # 于是核对整个被跳过 —— 实测踩过, 变异测试立刻显形。
    if _is_signature_block(body):
        for e in _entries(body):
            _check_signature_entry(e, rel, idx)
        return

    try:
        ast.parse(body)
    except SyntaxError as e:
        pytest.fail(f"{rel}#{idx} 语法错误: {e}")

    if ILLUSTRATIVE.search(body):
        return

    # 真跑。只注入 `arm` 与 `pa` —— 别的名字未定义就会 NameError，
    # 这正是要抓的"示例里留了没交代的占位符"。
    import litearm as pa

    arm = _make_arm()
    try:
        exec(compile(body, f"{rel}#{idx}", "exec"), {"pa": pa, "arm": arm})
    except Exception as e:  # noqa: BLE001 - 示例跑不起来就是失败
        pytest.fail(f"{rel}#{idx} 执行失败:\n{body}\n→ {type(e).__name__}: {e}")
    finally:
        arm.close()
