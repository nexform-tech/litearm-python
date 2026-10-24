"""Packaging metadata —— PyPI 的副标题和许可证都取自 `pyproject.toml`。

副标题 (`[project].description`) 会直接出现在 PyPI 项目页和搜索结果里, 所以它
必须是英文。许可证 (`[project].license`) 则必须与仓库根目录的 `LICENSE` 一致:
GitHub 只扫 `LICENSE` 文件, PyPI 只读包元数据, 两处不一致时两个平台会显示不同
的许可证。这两条用例把上述约束变成可检查的门禁。
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


def _project_license() -> str:
    text = PYPROJECT.read_text(encoding="utf-8")
    match = re.search(r'^license\s*=\s*"([^"]+)"\s*$', text, re.MULTILINE)
    assert match is not None, "pyproject.toml has no [project].license"
    return match.group(1)


def test_pypi_subtitle_is_english():
    """PyPI 副标题 (Summary) 必须是英文, 不能是中文。"""
    description = _project_description()
    assert description, "[project].description must not be empty"
    assert description.isascii(), (
        "PyPI shows [project].description as the project subtitle; "
        f"write it in English: {description!r}"
    )


def test_license_metadata_matches_the_license_file():
    """PyPI 读 pyproject 的 license 字段, GitHub 扫 LICENSE 文件, 两边必须一致。"""
    declared = _project_license()
    license_file = (PYPROJECT.parent / "LICENSE").read_text(encoding="utf-8")
    assert declared == "Apache-2.0"
    assert "Apache License" in license_file
    assert "Version 2.0" in license_file
