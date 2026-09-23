"""帧 CRC 的契约 —— **判据必须是独立的**。

## 为什么单开一个文件

`crc16_ccitt_false` 在 2026-09-22 从**纯 Python 位循环**换成了 `binascii.crc_hqx`
（C 实现，快 **232×**；真机实测它占读线程每帧成本的 **34%**）。换实现**必须**有一条
能钉住"算法没变"的用例，而这个文件就是它。

## ⚠⚠ 判据的独立性是这里的**全部要点**

**不许**把这里的用例写成 `crc16_ccitt_false(x) == binascii.crc_hqx(x, 0xFFFF)` ——
那在改动之后**恒真**（实现就是它），等于**自己验自己**：谁把 poly 改错，用例照样绿。
（本仓的规矩：判据要有判别力，见 `discriminating-power-of-a-criterion`。）

所以判据有三条，**都不依赖被测实现**：

1. **CRC 目录的标准检查值**（外部权威）：`CRC-16/CCITT-FALSE("123456789") == 0x29B1`；
2. 本文件内**自带一份独立的纯 Python 参考实现**（逐位算，与被测实现无共享代码）
   —— 长度 0..259 全覆盖 + 大批随机载荷逐一比对；
3. 线级契约：`pack_frame`/`unpack_frame` 的**低字节在前**、坏 CRC 必须被拒。

⚠ 第 2 条**不随被测实现一起改**：它是**参照物**，不是重复实现。
"""
from __future__ import annotations

import random

import pytest

from litearm import _protocol as P


def _crc_ref_bitwise(data: bytes) -> int:
    """**独立的参考实现** —— 逐位算，与被测实现无共享代码。

    ⚠ 这是**参照物**：改动 `_protocol` 时**不要动它**。把参照物也改成新算法，
    这条判据就废了（两边的错会互相抵消）。
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def test_the_standard_check_value_from_the_crc_catalogue():
    """CRC-16/CCITT-FALSE 的**标准检查值**：`"123456789"` → `0x29B1`。

    这条**不来自本仓任何代码** —— 它来自 CRC 参数目录（`poly=0x1021, init=0xFFFF,
    refin=false, refout=false, xorout=0x0000`）。抽查值一错，说明算法参数被改过。
    """
    assert P.crc16_ccitt_false(b"123456789") == 0x29B1


def test_matches_the_independent_reference_on_every_length_0_to_259():
    """长度 **0..259 全覆盖**（帧载荷上限 255 + 头 3 + CRC 2 = 260 ⇒ 259 是上限-1）。

    每个长度都用**确定性的**内容（不靠随机，失败可复现）。
    """
    for n in range(0, 260):
        data = bytes((i * 7 + n) & 0xFF for i in range(n))
        assert P.crc16_ccitt_false(data) == _crc_ref_bitwise(data), (
            f"长度 {n} 上与被测实现不一致")


def test_matches_the_independent_reference_on_random_payloads():
    """随机载荷铺一层（覆盖全覆盖用例碰不到的字节组合）。"""
    rng = random.Random(20260922)
    for _ in range(2000):
        data = bytes(rng.randrange(256) for _ in range(rng.randrange(0, 256)))
        assert P.crc16_ccitt_false(data) == _crc_ref_bitwise(data)


def test_empty_input_is_the_init_value():
    """空输入必须回 init（`0xFFFF`）—— 钉住"没有末异或"这一格。"""
    assert P.crc16_ccitt_false(b"") == 0xFFFF
    assert P.crc16_ccitt_false(b"") == _crc_ref_bitwise(b"")


def test_a_single_flipped_bit_changes_the_crc():
    """**负控**：改一个 bit 必须换一个 CRC。

    ⚠ 没有这条，一个"恒返回常数"的实现也能过上面所有长度/随机用例 ——
    因为参考实现会跟着一起算，只有恒等比较才能发现。所以这里直接问
    "它对输入敏感吗"。
    """
    a = _crc_ref_bitwise(b"\x01\x02\x03")
    b = _crc_ref_bitwise(b"\x01\x02\x02")          # 翻最低位
    assert a != b, "参考实现自己对输入不敏感 —— 上面那些用例的判据不成立"
    assert P.crc16_ccitt_false(b"\x01\x02\x03") != P.crc16_ccitt_false(b"\x01\x02\x02")


# ---------------------------------------------------------------------------
# 线级契约 —— CRC 在帧里的**位置与字节序**
# ---------------------------------------------------------------------------

def test_the_frame_carries_the_crc_low_byte_first():
    """`SOF CMD LEN PAYLOAD crc_LO crc_HI` —— **低字节在前**。

    ⚠ 这是与固件的线契约里最容易改错的一格（换个字节序，本机全绿、真机全哑）。
    """
    payload = bytes(range(20))
    frame = P.pack_frame(0x34, payload)
    body = frame[:-2]
    want = P.crc16_ccitt_false(body)
    assert frame[-2] == (want & 0xFF), "CRC 低字节不在倒数第二位"
    assert frame[-1] == (want >> 8), "CRC 高字节不在最后一位"
    assert P.unpack_frame(frame) == (0x34, payload)


def test_a_bad_crc_is_rejected():
    """坏 CRC 必须被拒（返回 `None`）—— 否则坏帧会被当成好帧收下。"""
    frame = bytearray(P.pack_frame(P.RSP_STATUS, b"\x01\x02\x03"))
    frame[-1] ^= 0x01                                # 翻 CRC 高字节
    assert P.unpack_frame(bytes(frame)) is None


@pytest.mark.parametrize("bad", [
    b"",                                             # 空
    bytes([P.SOF, 0x34, 0x05, 1, 2]),                # 长度说 5 但只有 2 个载荷字节
])
def test_truncated_or_short_input_is_rejected(bad):
    """长度不足 / 声明长度超过实际 ⇒ 拒绝，不许越界读或猜。"""
    assert P.unpack_frame(bad) is None
