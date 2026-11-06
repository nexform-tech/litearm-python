import importlib.util

import pytest

pytestmark = pytest.mark.skipif(importlib.util.find_spec("numpy") is None,
                                reason="optional NumPy dependency is not installed")


def test_exact_solution_and_zero_space_policy():
    import numpy as np
    from litearm.exact_ik import FAIL_NONE, exact_task_velocity

    solution = exact_task_velocity([[1.0, 0.0]], [0.5], [-1.0, -1.0], [1.0, 1.0])
    assert solution.achieved
    assert solution.failure == FAIL_NONE
    np.testing.assert_allclose(solution.velocity, [0.5, 0.0], atol=1e-12)


def test_box_failure_reports_joint_side_and_excess():
    from litearm.exact_ik import FAIL_BOX, box_binding, exact_task_velocity

    solution = exact_task_velocity([[1.0, 0.0]], [5.0], [-1.0, -1.0], [1.0, 1.0])
    assert not solution.achieved
    assert solution.failure == FAIL_BOX
    assert solution.worst_joint == 0
    assert solution.worst_side == 1
    assert solution.worst_excess == pytest.approx(4.0)
    assert box_binding(solution, [-1.0, -1.0], [1.0, 1.0], 1.0) == "speed"


def test_rank_deficiency_is_measured_by_residual():
    from litearm.exact_ik import FAIL_RESIDUAL, exact_task_velocity

    solution = exact_task_velocity([[1.0, 0.0], [0.0, 0.0]], [0.0, 1.0],
                                   [-2.0, -2.0], [2.0, 2.0])
    assert not solution.achieved
    assert solution.failure == FAIL_RESIDUAL
    assert solution.residual == pytest.approx(1.0)


def test_invalid_values_are_shape_failure():
    from litearm.exact_ik import FAIL_SHAPE, exact_task_velocity

    solution = exact_task_velocity([[float("nan")]], [0.0], [-1.0], [1.0])
    assert solution.failure == FAIL_SHAPE


def test_bounded_fallback_respects_saturation_and_fixed_joints():
    pytest.importorskip("scipy")
    import numpy as np
    from litearm.exact_ik import bounded_task_velocity

    velocity = bounded_task_velocity(np.eye(2), [10.0, 0.2],
                                     [-0.3, 0.0], [0.3, 0.0])
    np.testing.assert_allclose(velocity, [0.3, 0.0], atol=1e-9)
