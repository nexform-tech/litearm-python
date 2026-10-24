"""末端负载辨识 —— 解算数学 / 基线文件 / 固件探针 / 端到端回收真值。

端到端那几条用例的"物理"是桩里那个固定轴链点质量模型（`litearm.testing` 的
`fake_arm_gravity` / `fake_payload_gravity`）：`0x39` 的应答与状态帧里的力矩由**同一对
函数**生成，所以「辨识能否回收注入的质量与质心」是一条可以写成等式的断言，而不是
"看着差不多"。
"""
from __future__ import annotations

import json
import os

import pytest

import litearm as pa
from litearm import _protocol as P
from litearm import payload as pl
from litearm.errors import CommandRejectedError, InvalidCommandError, LiteArmError
from litearm.testing import fake_model_gravity, fake_payload_gravity

#: 注入的"真值"负载：质量 + 法兰系质心 (m)
TRUE_MASS = 0.36
TRUE_COM = (0.02, -0.01, 0.05)

#: 与旧脚本同族的姿态集（软限位内、大臂多构型多样化）
POSES = [
    (0.0, 0.6, 0.0, -1.0, 0.0, 0.5, 0.0),
    (0.0, 1.0, 0.0, -1.2, 0.0, 0.6, 0.0),
    (0.0, 1.3, 0.0, -0.8, 0.0, 0.4, 0.0),
    (0.0, 0.8, 0.5, -1.4, 0.3, 0.5, 0.2),
]


#: 没有 arm 可用时的臂本体项（默认真固件的模型: body1..7 各 1 kg ⇒ msum 8.0）
_DEFAULT_ARM = lambda q: fake_model_gravity(q, 8.0)      # noqa: E731


def _plant(arm, mass, com):
    """把桩变成一个准静态植物：状态帧里的 τ = 臂本体 + 负载(真值)。

    ⚠ 臂本体项用 `arm._tr.arm_gravity`（与 `0x39` 逐值同源），**不用**
    `fake_arm_gravity` —— 后者是另一份"力学上说得通"的模型。差分基线的姿态搬运靠
    `0x39` 做，两份模型混用会让"构型有偏差时"的结果系统性偏掉（用例
    `test_identify_tolerates_a_sagging_arrival` 就是为这一条钉的）。
    """
    body = arm._tr.arm_gravity

    def tau_of(q):
        return [a + b for a, b in
                zip(body(q), fake_payload_gravity(q, mass, com))]
    arm._tr.tau_provider = tau_of
    arm.enable()
    return arm


def _baseline(arm=None, mass=0.0, com=(0.0, 0.0, 0.0), poses=POSES):
    """一份"空载"基线：力矩就是臂本体（+ 指定负载，用来造脏基线）。"""
    body = arm._tr.arm_gravity if arm is not None else _DEFAULT_ARM
    taus = [tuple(a + b for a, b in
                  zip(body(q), fake_payload_gravity(q, mass, com)))
            for q in poses]
    return pl.Baseline(poses=tuple(poses), q=tuple(poses), tau=tuple(taus))


def _identify(arm, baseline):
    """短采样跑完整流程（桩的 auto_status 是 1 ms 一节，2 帧足够）。"""
    return pl.identify(arm, baseline, speed=0.3, settle=0.0, frames=2, period=0.0)


# ---------------------------------------------------------------- 基线文件

def test_baseline_json_roundtrip(tmp_path):
    base = _baseline()
    path = base.save(str(tmp_path / "sub" / "baseline.json"))
    assert os.path.isfile(path)
    back = pl.Baseline.load(path)
    assert back.poses == base.poses
    assert back.q == base.q
    assert back.tau == base.tau
    assert len(back) == len(base)
    with open(path, encoding="utf-8") as f:
        doc = json.load(f)
    assert doc["schema"] == pl.BASELINE_SCHEMA


def test_baseline_rejects_foreign_schema(tmp_path):
    path = tmp_path / "x.json"
    path.write_text(json.dumps({"schema": "something.else", "poses": [], "q": [], "tau": []}),
                    encoding="utf-8")
    with pytest.raises(LiteArmError, match="schema"):
        pl.Baseline.load(str(path))


def test_baseline_rejects_row_count_mismatch():
    with pytest.raises(InvalidCommandError, match="不一致"):
        pl.Baseline(poses=tuple(POSES), q=tuple(POSES[:2]), tau=tuple(POSES))


def test_baseline_rejects_wrong_arity():
    with pytest.raises(InvalidCommandError, match="需要 7 个数"):
        pl.Baseline(poses=((0.0,) * 6,), q=((0.0,) * 6,), tau=((0.0,) * 6,))


def test_baseline_from_legacy_csv(tmp_path):
    """老脚本的 `noload_baseline.csv` 表头必须能直接导入（pred_* 列忽略）。"""
    path = tmp_path / "noload_baseline.csv"
    head = ["pose"] + [f"q{k}" for k in range(7)] + [f"tau_meas{k}" for k in range(7)]
    head += [f"pred_id{k}" for k in range(7)] + [f"pred_cad{k}" for k in range(7)]
    rows = []
    for i, q in enumerate(POSES):
        tau = fake_model_gravity(q, 8.0)
        rows.append([str(i)] + [repr(v) for v in q] + [repr(v) for v in tau]
                    + ["0.0"] * 14)
    path.write_text("\n".join(",".join(r) for r in [head] + rows) + "\n",
                    encoding="utf-8")

    base = pl.Baseline.from_csv(str(path))
    assert len(base) == len(POSES)
    assert base.poses[1] == pytest.approx(POSES[1])
    assert base.q == base.poses                      # CSV 没有单独记下发目标
    assert base.tau[0] == pytest.approx(tuple(fake_model_gravity(POSES[0], 8.0)))


def test_baseline_from_csv_reports_missing_column(tmp_path):
    path = tmp_path / "bad.csv"
    path.write_text("pose,q0,q1\n0,0.0,0.0\n", encoding="utf-8")
    with pytest.raises(LiteArmError, match="缺列"):
        pl.Baseline.from_csv(str(path))


# ---------------------------------------------------------------- 回归矩阵与解算

def _y_of(q, mass=1.0):
    """数值回归矩阵：第 k 列 = 该方向单位参数引起的重力矩（桩模型的精确解）。"""
    cols = []
    base = fake_payload_gravity(q, mass, (0.0, 0.0, 0.0))
    cols.append([v / mass for v in base])
    for k in range(3):
        com = [0.0, 0.0, 0.0]
        com[k] = 1e-3
        bumped = fake_payload_gravity(q, mass, com)
        cols.append([(b - a) / (mass * 1e-3) for a, b in zip(base, bumped)])
    return [[cols[c][r] for c in range(4)] for r in range(7)]


def test_a_single_pose_cannot_separate_mass_from_com():
    """单构型回归矩阵 rank = 3 —— 这是"基线至少要两个异姿态"的几何原因。

    `m` 一列 + `m·c` 的像最多 2 维（重力矢量的叉乘矩阵秩为 2），故 4 参数欠定；
    真机与桩在这一点上结构相同，所以这条断言不是桩的自娱自乐。
    """
    Y = _y_of(POSES[0])
    _inv, rank, _p = pl._inverse([[sum(Y[r][a] * Y[r][b] for r in range(7))
                                   for b in range(4)] for a in range(4)])
    assert rank < 4, f"单构型竟然满秩 (rank {rank}) —— 桩的几何已经不忠实了"


def test_two_poses_make_the_regressor_full_rank():
    Ys = [_y_of(q) for q in POSES[:2]]
    A = [[sum(Y[r][a] * Y[r][b] for Y in Ys for r in range(7)) for b in range(4)]
         for a in range(4)]
    _inv, rank, _p = pl._inverse(A)
    assert rank == 4


def test_solver_recovers_theta_from_exact_rows():
    theta = [TRUE_MASS, TRUE_MASS * TRUE_COM[0], TRUE_MASS * TRUE_COM[1],
             TRUE_MASS * TRUE_COM[2]]
    rows = []
    for q in POSES:
        Y = _y_of(q)
        rows.append((Y, [sum(Y[r][k] * theta[k] for k in range(4)) for r in range(7)]))
    sol = pl._solve(rows)
    assert sol["rank"] == 4
    assert sol["theta"] == pytest.approx(theta, rel=1e-9)
    assert sol["sigma_res"] == pytest.approx(0.0, abs=1e-12)


def test_solver_reports_rank_deficiency():
    Y = _y_of(POSES[0])
    sol = pl._solve([(Y, [0.4] * 7)])
    assert sol["theta"] is None
    assert sol["rank"] < 4
    assert sol["sigma_res"] != sol["sigma_res"]        # NaN: 没有结果就没有残差


# ---------------------------------------------------------------- 固件探针

def test_probe_regressor_matches_the_fake_model(offline_arm):
    """探针解出的 `Y_g(q)` 必须与桩的解析回归矩阵逐元素一致。"""
    Y = pl.regressor(offline_arm, POSES[2])
    want = _y_of(POSES[2])
    for r in range(7):
        for c in range(4):
            assert Y[r][c] == pytest.approx(want[r][c], rel=5e-4, abs=5e-4)


def test_probe_restores_the_payload_it_found(offline_arm):
    arm = offline_arm
    arm.set_payload(0.7, (0.1, -0.2, 0.3))
    pl.regressor(arm, POSES[1])
    assert arm.get_ff_scalar(4).value == pytest.approx(0.7)
    assert [arm.get_ff_scalar(5, k).value for k in range(3)] == pytest.approx(
        [0.1, -0.2, 0.3], abs=1e-6)


def test_probe_writes_a_small_phantom_load(offline_arm):
    """幻影负载必须真的存在（否则探针读不出东西）而且足够小（不推臂）。"""
    arm = offline_arm
    pl.regressor(arm, POSES[0])
    masses = [struct_value for (item, sub), struct_value in arm._tr.ff_scalar.items()
              if item == 4]
    assert masses, "探针没写过 payload —— 回归矩阵不可能是从固件取的"
    assert max(masses) <= 0.05, f"幻影质量 {max(masses)} kg 太大: 使能持位时会被推"
    assert arm.get_ff_scalar(4).value == 0.0, "探针结束没把 payload 恢复成 0"


def test_probe_restores_payload_even_when_the_query_fails(offline_arm):
    arm = offline_arm
    arm.set_payload(1.2, (0.0, 0.0, 0.0))
    arm._tr.err_override[P.CMD_GET_GRAVITY] = 0x01
    with pytest.raises(CommandRejectedError):
        pl.regressor(arm, POSES[0])
    arm._tr.err_override.pop(P.CMD_GET_GRAVITY)
    assert arm.get_ff_scalar(4).value == pytest.approx(1.2), "探针抛错后 payload 没恢复"


def test_gravity_ignores_the_firmware_payload(offline_arm):
    """`gravity()` 要的是几何量：固件里配着负载也不该被算进来。"""
    arm = offline_arm
    clean = pl.gravity(arm, POSES[3])
    arm.set_payload(2.0, (0.0, 0.0, 0.0))
    dirty = pl.gravity(arm, POSES[3])
    assert dirty == pytest.approx(clean, abs=1e-6), "固件里的负载被算进了 G(q)"
    assert arm.get_ff_scalar(4).value == pytest.approx(2.0), "原 payload 没写回"


# ---------------------------------------------------------------- 停驻采样

def test_sample_hold_averages_and_reports_spread(offline_arm):
    arm = offline_arm
    arm._tr.tau = [1.0, -2.0, 3.0, -4.0, 5.0, -6.0, 7.0]
    s = pl.sample_hold(arm, frames=3, period=0.0)
    assert s.frames == 3
    assert s.tau == pytest.approx((1.0, -2.0, 3.0, -4.0, 5.0, -6.0, 7.0))
    assert max(s.spread) == 0.0            # 桩的力矩恒定
    assert max(s.dq) == 0.0


def test_sample_hold_drops_moving_frames(offline_arm):
    arm = offline_arm
    arm._tr.dq = [0.2] * 7                 # 一直"在动"
    with pytest.raises(LiteArmError, match="全部被拒"):
        pl.sample_hold(arm, frames=3, period=0.0)


def test_sample_hold_drops_saturated_frames(offline_arm):
    arm = offline_arm
    arm._tr.tau = [9.9] * 7                # 桩的 tau_max = 10.0 ⇒ 饱和
    with pytest.raises(LiteArmError, match="全部被拒"):
        pl.sample_hold(arm, frames=3, period=0.0)


def test_sample_hold_rejects_bad_frames_argument(offline_arm):
    with pytest.raises(InvalidCommandError, match="frames"):
        pl.sample_hold(offline_arm, frames=0)


# ---------------------------------------------------------------- 端到端

def test_identify_recovers_the_injected_payload(offline_arm):
    arm = _plant(offline_arm, TRUE_MASS, TRUE_COM)
    r = _identify(arm, _baseline(arm))
    assert r.rank == 4
    assert r.mass == pytest.approx(TRUE_MASS, rel=1e-5)
    assert r.com == pytest.approx(TRUE_COM, abs=1e-5)
    assert r.sigma_mass < 1e-5
    assert r.residual_rms < 1e-5
    assert r.ok is True
    assert r.warnings == ()
    assert "质量" in str(r) and "质心" in str(r)


def test_identify_survives_a_payload_already_written_to_the_firmware(offline_arm):
    """固件里预置了同样的负载 ⇒ 实测与基线都含它，差分仍应给出真值（不多计一次）。"""
    arm = _plant(offline_arm, TRUE_MASS, TRUE_COM)
    arm.set_payload(TRUE_MASS, TRUE_COM)          # 固件 G(q) 也带上了它
    r = _identify(arm, _baseline(arm))
    assert r.mass == pytest.approx(TRUE_MASS, rel=1e-5)


def test_identify_reports_a_dirty_baseline(offline_arm):
    """基线里已经含着一份负载 ⇒ 结果多出一份重量，且这件事瞒不住。"""
    arm = _plant(offline_arm, TRUE_MASS, TRUE_COM)
    dirty = _baseline(arm, mass=0.2, com=(0.0, 0.0, 0.0))
    r = _identify(arm, dirty)
    assert r.mass == pytest.approx(TRUE_MASS - 0.2, rel=1e-3)


def test_identify_needs_two_poses(offline_arm):
    arm = _plant(offline_arm, TRUE_MASS, TRUE_COM)
    with pytest.raises(LiteArmError, match="欠秩"):
        _identify(arm, _baseline(arm, poses=POSES[:1]))


def test_identify_refuses_a_disabled_arm(offline_arm):
    with pytest.raises(InvalidCommandError, match="使能"):
        _identify(offline_arm, _baseline())


def test_identify_refuses_a_bench_arm(offline_arm_1j):
    with pytest.raises(InvalidCommandError, match="7 轴"):
        pl.identify(offline_arm_1j, _baseline(), settle=0.0, frames=2, period=0.0)


def test_identify_refuses_a_csv_path_instead_of_a_baseline(offline_arm):
    arm = _plant(offline_arm, TRUE_MASS, TRUE_COM)
    with pytest.raises(InvalidCommandError, match="Baseline"):
        pl.identify(arm, "runs/20260918_151253_空载基准/noload_baseline.csv")


def test_identify_refuses_bad_speed(offline_arm):
    arm = _plant(offline_arm, TRUE_MASS, TRUE_COM)
    with pytest.raises(InvalidCommandError, match="speed"):
        pl.identify(arm, _baseline(), speed=2.0)


def test_apply_writes_mass_and_com(offline_arm):
    arm = _plant(offline_arm, TRUE_MASS, TRUE_COM)
    r = _identify(arm, _baseline(arm))
    pl.apply(arm, r)
    assert arm.get_ff_scalar(4).value == pytest.approx(r.mass)
    assert [arm.get_ff_scalar(5, k).value for k in range(3)] == pytest.approx(r.com)


def test_apply_can_write_the_mass_only(offline_arm):
    arm = _plant(offline_arm, TRUE_MASS, TRUE_COM)
    r = _identify(arm, _baseline(arm))
    pl.apply(arm, r, com=False)
    assert arm.get_ff_scalar(4).value == pytest.approx(r.mass)
    assert [arm.get_ff_scalar(5, k).value for k in range(3)] == pytest.approx(
        [0.0, 0.0, 0.0])


def test_apply_refuses_values_the_firmware_would_silently_clamp(offline_arm):
    arm = offline_arm
    bad = pa.PayloadResult(mass=-0.1, com=(0.0, 0.0, 0.0), sigma_mass=0.0,
                           sigma_com=(0.0, 0.0, 0.0), rank=4, cond=1.0,
                           residual_rms=0.0, n_rows=28, poses=4, quality=(0.0,))
    with pytest.raises(InvalidCommandError, match="质量"):
        pl.apply(arm, bad)
    far = pa.PayloadResult(mass=0.5, com=(0.0, 0.0, 3.0), sigma_mass=0.0,
                           sigma_com=(0.0, 0.0, 0.0), rank=4, cond=1.0,
                           residual_rms=0.0, n_rows=28, poses=4, quality=(0.0,))
    with pytest.raises(InvalidCommandError, match="质心"):
        pl.apply(arm, far)


def test_result_is_json_safe(offline_arm):
    arm = _plant(offline_arm, TRUE_MASS, TRUE_COM)
    r = _identify(arm, _baseline(arm))
    json.dumps(r.as_dict())                          # 不能有 tuple/NaN 之外的坑
    assert r.as_dict()["mass_kg"] == pytest.approx(r.mass)


# ---------------------------------------------------------------- 未到位容忍

def test_identify_tolerates_a_sagging_arrival(monkeypatch, offline_arm):
    """带载下垂让固件判据过不去时, 构型残差在 `move_tol` 内仍然继续。

    这条走的是真机上最常见的一种失败: 负载上车、固件里还没有它的前馈, 静态下垂几十毫弧度
    ⇒ `arm.q_tol`（0.03 rad）永远凑不齐, `movej` 走满预算报 `MotionTimeoutError`。
    """
    arm = _plant(offline_arm, TRUE_MASS, TRUE_COM)
    real_movej = arm.movej
    sag = 0.01                                  # 每个构型都差 10 mrad

    def sagging_movej(q, speed=1.0):
        real_movej(q, speed)
        arm._tr.q = [v - sag for v in q]         # 桩报回来的构型偏一点
        raise pa.MotionTimeoutError("未到位 (注入)")

    monkeypatch.setattr(arm, "movej", sagging_movej)
    r = _identify(arm, _baseline(arm))
    assert r.rank == 4
    assert r.mass == pytest.approx(TRUE_MASS, rel=1e-5)


def test_identify_gives_up_when_the_arm_really_did_not_arrive(monkeypatch, offline_arm):
    arm = _plant(offline_arm, TRUE_MASS, TRUE_COM)
    real_movej = arm.movej

    def stuck_movej(q, speed=1.0):
        real_movej(q, speed)
        arm._tr.q = [v - 0.4 for v in q]         # 差 0.4 rad = 23°, 远不是下垂
        raise pa.MotionTimeoutError("未到位 (注入)")

    monkeypatch.setattr(arm, "movej", stuck_movej)
    with pytest.raises(pa.MotionTimeoutError, match="move_tol"):
        _identify(arm, _baseline(arm))
