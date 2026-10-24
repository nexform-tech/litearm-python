"""Packaging metadata —— PyPI 把 `[project].description` 当作项目副标题展示。

副标题会直接出现在 PyPI 项目页和搜索结果里, 所以它必须是英文; 这条用例把
"别再把中文写回 description" 变成可检查的门禁。
"""
from __future__ import annotations

import re
from pathlib import Path

PYPROJECT = Path(__file__).resolve().parents[1] / "pyproject.toml"


def _project_description() -> str:
    text = PYPROJECT.read_text(encoding="utf-8")
    match = re.search(r'^description\s*=\s*"([^"]*)"\s*$', text, re.MULTILINE)
    assert match is not None, "pyproject.toml has no [project].description"
    return match.group(1)


def test_pypi_subtitle_is_english():
    """PyPI 副标题 (Summary) 必须是英文, 不能是中文。"""
    description = _project_description()
    assert description, "[project].description must not be empty"
    assert description.isascii(), (
        "PyPI shows [project].description as the project subtitle; "
        f"write it in English: {description!r}"
    )
