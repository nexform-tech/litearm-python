"""Guard the packaged SDK URDF against the STM32 generated FK/Jacobian model."""
from __future__ import annotations

import ctypes
import os
from pathlib import Path
import random
import shutil
import subprocess

try:
    import pytest
except ImportError:
    import unittest
    raise unittest.SkipTest("pytest is not installed")

import litearm as pa
from litearm import _rot


def _stm32_root() -> Path:
    configured = os.environ.get("LITEARM_STM32_ROOT")
    if configured:
        return Path(configured).expanduser().resolve()
    return Path(__file__).resolve().parents[2] / "litearm-stm32"


def test_sdk_model_matches_stm32_generated_fk_and_jacobian(tmp_path):
    np = pytest.importorskip("numpy")
    pytest.importorskip("pinocchio")
    root = _stm32_root()
    kin_c = root / "User/litearm/kinematics/kin.c"
    if not kin_c.is_file() or shutil.which("gcc") is None:
        pytest.skip("litearm-stm32 sibling checkout and gcc are needed for cross-repo guard")

    shared = tmp_path / "liblitearm_kin.so"
    subprocess.run(
        ["gcc", "-shared", "-fPIC", "-O2", "-o", str(shared), str(kin_c), "-lm"],
        check=True, capture_output=True, text=True)
    firmware = ctypes.CDLL(str(shared))
    firmware.kin_fk.argtypes = [ctypes.POINTER(ctypes.c_float),
                                ctypes.POINTER(ctypes.c_float),
                                ctypes.POINTER(ctypes.c_float)]
    firmware.kin_jacobian.argtypes = [ctypes.POINTER(ctypes.c_float),
                                      ctypes.POINTER(ctypes.c_float)]

    arm = pa.Arm()
    points = [
        [0.0] * 7,
        [0.0, -0.5, 0.0, -1.0, 0.0, 0.5, 0.0],
        [0.15, -0.55, 0.10, -1.10, 0.05, 0.55, -0.05],
        [-0.31, 0.72, -0.44, -1.38, 0.27, 0.81, -0.19],
    ]
    rng = random.Random(7)
    points.extend([[rng.uniform(-2.0, 2.0) for _ in range(7)]
                   for _ in range(20)])
    for q in points:
        cq = (ctypes.c_float * 7)(*q)
        cpos = (ctypes.c_float * 3)()
        crot = (ctypes.c_float * 9)()
        cjac = (ctypes.c_float * 42)()
        firmware.kin_fk(cq, cpos, crot)
        firmware.kin_jacobian(cq, cjac)

        pose = arm.kinematics.fk(q, frame="flange")
        sdk_rot = np.asarray(_rot.rpy_to_mat(pose[3:]), dtype=float)
        sdk_jac = np.asarray(
            arm.kinematics.jacobian(q, frame="flange").matrix, dtype=float)
        np.testing.assert_allclose(pose[:3], np.asarray(cpos[:]), atol=2e-5)
        np.testing.assert_allclose(sdk_rot, np.asarray(crot[:]).reshape(3, 3), atol=2e-5)
        np.testing.assert_allclose(sdk_jac, np.asarray(cjac[:]).reshape(6, 7), atol=2e-4)
