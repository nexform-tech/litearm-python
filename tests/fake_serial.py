"""向后兼容薄壳 —— 实物已提升为公开的 `litearm.testing`。

本文件**只做 re-export，不要在这里加逻辑**：两份假件必然漂移，而漂移的假件会让
离线全绿、真机全挂。见本仓 `kin-bench-two-frame-contract` 那条教训 ——
"桩把多帧合成一帧 = 把契约测掉"。

⚠ 保留本文件是**必须**的：`tests/` 下 5 个文件在 `from fake_serial import ...`
（`conftest.py` 把 `tests/` 插进了 `sys.path`）。
"""
from litearm.testing import *          # noqa: F401,F403
from litearm.testing import (          # noqa: F401  下划线名显式转出
    _N,
    _CART_MIN_LEN,
    _CART_PLANNING_CMDS,
    _ack,
    _mk_log,
    _split_kin_bench,
    _status,
)
