"""Optional host-side collision checking (no hardware required)."""
from __future__ import annotations

import importlib
from pathlib import Path
from types import SimpleNamespace

try:
    import pytest
except ImportError:
    import unittest
    raise unittest.SkipTest("pytest is not installed")

import litearm as pa
import litearm.collision as collision_mod


class _Pair:
    def __init__(self, first, second):
        self.first = first
        self.second = second


class _Result:
    def __init__(self, hit=False):
        self.hit = hit

    def isCollision(self):
        return self.hit


class _Model:
    nq = 7
    # universe, joint1, joint2, joint3
    parents = [0, 0, 1, 2]

    def createData(self):
        return object()


class _Geometry:
    def __init__(self):
        self.geometryObjects = [
            SimpleNamespace(parentJoint=0, name="base"),
            SimpleNamespace(parentJoint=1, name="link1"),
            SimpleNamespace(parentJoint=2, name="link2"),
            SimpleNamespace(parentJoint=3, name="link3"),
        ]
        self.collisionPairs = []

    def addAllCollisionPairs(self):
        self.collisionPairs = [
            _Pair(i, j) for i in range(4) for j in range(i + 1, 4)
        ]

    def removeCollisionPair(self, target):
        self.collisionPairs = [
            pair for pair in self.collisionPairs
            if (pair.first, pair.second) != (target.first, target.second)
        ]

    def createData(self):
        return SimpleNamespace(
            collisionResults=[_Result() for _ in self.collisionPairs])


class _Pin:
    COLLISION = object()
    CollisionPair = _Pair

    def __init__(self):
        self.geometries = []

    @staticmethod
    def buildModelFromUrdf(_path):
        return _Model()

    def buildGeomFromUrdf(self, *_args, **_kwargs):
        geometry = _Geometry()
        self.geometries.append(geometry)
        return geometry

    @staticmethod
    def computeCollisions(_model, _data, geometry, geometry_data, q, stop):
        # The fake's deterministic collision: positive J1 makes the first
        # surviving pair collide.  Keep result-vector order aligned with pairs.
        hits = [bool(q[0] > 0.0 and index == 0)
                for index in range(len(geometry.collisionPairs))]
        geometry_data.collisionResults = [_Result(hit) for hit in hits]
        return any(hits) if stop else False


@pytest.fixture
def fake_dependencies(monkeypatch):
    real_import = importlib.import_module
    pin = _Pin()

    def load(name):
        if name == "pinocchio":
            return pin
        return real_import(name)

    monkeypatch.setattr(collision_mod.importlib, "import_module", load)
    return pin


def test_assets_are_packaged_and_public_api_is_exported():
    urdf = Path(pa.collision_urdf_path())
    assert urdf.is_file()
    assert len(list(urdf.parents[1].joinpath("meshes").glob("*.STL"))) == 9
    assert pa.CollisionModel is collision_mod.CollisionModel
    assert "CollisionModel" in pa.__all__
    assert "collision_urdf_path" in pa.__all__


def test_checker_excludes_same_or_adjacent_links_and_reports_pairs(fake_dependencies):
    checker = pa.CollisionModel()
    assert checker.n_all_pairs == 6
    assert checker.n_active_pairs == 3
    assert not checker.in_collision([0.0] * 7)
    assert checker.in_collision([0.1] + [0.0] * 6)
    assert checker.colliding_pairs([0.1] + [0.0] * 6) == [("base", "link2")]


@pytest.mark.parametrize("bad", [
    [], [0.0] * 6, [0.0] * 8,
    [0.0] * 6 + [float("nan")],
    [0.0] * 6 + [float("inf")],
    [0.0] * 6 + ["not-a-number"],
    None,
])
def test_checker_strictly_validates_joint_vector(fake_dependencies, bad):
    checker = pa.CollisionModel()
    with pytest.raises(ValueError, match="7 个有限数值"):
        checker.in_collision(bad)


def test_each_checker_owns_its_geometry_model(fake_dependencies):
    first = pa.CollisionModel()
    second = pa.CollisionModel()
    assert first.geom is not second.geom
    first.geom.collisionPairs.clear()
    assert len(second.geom.collisionPairs) == second.n_active_pairs == 3


def test_missing_optional_dependency_has_install_hint(monkeypatch):
    real_import = importlib.import_module

    def load(name):
        if name == "pinocchio":
            raise ModuleNotFoundError(name)
        return real_import(name)

    monkeypatch.setattr(collision_mod.importlib, "import_module", load)
    with pytest.raises(ImportError, match=r"litearm-python\[collision\]"):
        pa.CollisionModel()


def test_real_packaged_model_when_pinocchio_is_available():
    try:
        importlib.import_module("pinocchio")
    except (ImportError, OSError) as exc:
        pytest.skip(f"本机 Pinocchio 不可用: {exc}")

    checker = pa.CollisionModel()
    assert checker.nq == 7
    assert checker.n_active_pairs < checker.n_all_pairs
    assert not checker.in_collision([0.0] * 7)
    folded = [-0.753, 1.4236, 0.1053, -2.6041, -0.0948, -0.7629, 0.4072]
    assert checker.in_collision(folded)
    assert checker.colliding_pairs(folded)
