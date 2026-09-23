"""离线全流程 (桩 transport): 连接/版本门禁/enable/movej 到位/move_p/ik/get_tcp/
状态解析/FF 命令/伺服编码 —— 镜像 pylitearm tests (test_arm_embedded_dryrun 风格)。"""
from __future__ import annotations

import pytest

from litearm import _protocol as P
from litearm import Arm
from litearm.errors import (
    FirmwareMismatchError,
    InvalidCommandError,
    MotionTimeoutError,
)


def test_connect_fw_ok(offline_arm):
    assert offline_arm.firmware == "Litearm1.7.0-7J"
    assert offline_arm.fw_version == (1, 7, 0)
    assert offline_arm.n == 7
    assert not offline_arm.get_state().value.faulted


def test_version_gate_old_naming(fake_transport_factory):
    fake_transport_factory(fw="A1.3.0-7J-USB")
    with pytest.raises(FirmwareMismatchError):
        Arm(port="fake").connect()


def test_version_gate_too_old(fake_transport_factory):
    fake_transport_factory(fw="Litearm1.3.0-7J")
    with pytest.raises(FirmwareMismatchError):
        Arm(port="fake").connect()


def test_enable_disable(offline_arm):
    offline_arm.enable()
    offline_arm.disable()


def test_movej_arrives(offline_arm):
    target = [0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0]
    offline_arm.movej(target, speed=0.5)   # 桩会先给 3 拍"运动中"再到位
    st = offline_arm.get_state().value
    assert all(abs(st.q[i] - target[i]) < 1e-6 for i in range(7))


def test_movej_bad_count(offline_arm):
    with pytest.raises(InvalidCommandError):
        offline_arm.movej([0.0] * 6)   # N=7 缺一


def test_move_p_tcp_arrives(offline_arm):
    pose = [0.30, 0.02, 0.35, 0.0, 0.0, 0.0]
    offline_arm.move_p(pose, speed=0.2, pos_tol=0.006, rpy_tol=0.03)
    tcp = offline_arm.get_tcp().value
    assert abs(tcp[0] - pose[0]) < 1e-4 and abs(tcp[1] - pose[1]) < 1e-4


def test_move_p_rejects_a_pose_sequence_and_points_at_move_path(offline_arm):
    """位姿序列必须给出一条**指名 `move_path`** 的错。

    ⚠ 为什么非要指名: 从前 `move_p` (旧名 `movep`) 收序列就走 PC 侧规划。序列重载移除
    后, 若只让它落到 `_rot.as_pose` 的形状判据, 用户拿到的是那句
    "pose 形状无法识别: len=3" —— **完全看不出"这个 API 变了"**, 只会以为自己的位姿
    写错了。

    ⚠ 文案里还得带语义差异, 不能只说"改名了": 旧序列形态是**关节空间**多路点 (PC 侧
    规划, 已退役), `move_path` 是**笛卡尔**多路点 (固件规划), 两者不是同一个东西。
    """
    with pytest.raises(InvalidCommandError) as ei:
        offline_arm.move_p([[0.31, 0.0, 0.35, 0.0, 0.0, 0.0],
                            [0.31, 0.01, 0.35, 0.0, 0.0, 0.0],
                            [0.31, 0.02, 0.35, 0.0, 0.0, 0.0]])
    msg = str(ei.value)
    assert "move_path" in msg
    assert "关节空间" in msg and "笛卡尔" in msg      # 语义差异必须在文案里
    assert P.CMD_MOVE_P not in [c for c, _ in offline_arm._tr.tx_log]   # 一帧没发


#: 单位旋转 —— `move_p` 的 `(pos[3], R[3x3])` 输入形态要用。
ID_R = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def test_move_p_single_pose_uses_firmware_move_p(offline_arm):
    """单点 `move_p` 走固件 `CMD_MOVE_P` —— **不是**走 PC 侧规划 (`move_js` 流式)。

    ⚠ **改靶说明**: 本条从已删的 `tests/test_cartesian_motion.py` 摘出保留 (原
    `test_move_p_single_pose_uses_firmware_move_p`)。它测的从来不是被删的子包, 而是
    `Arm.move_p` 的**管道契约** —— 固件后台 IK + S 曲线 (一条 `0x02` 就够), 与 PC 侧
    规划那条流 (`move_js` 100Hz 逐拍 + `movej` 收尾) 是两回事。子包退役后这条契约
    **更**要守住: 现在它是"`move_p` 没有偷偷退回 PC 规划"的唯一哨兵。

    返回类型也钉住: `RobotState` (有 `mode_name`), 不是那条流返回的 `CartPlan`。
    (原 docstring 写的是"不是 CartPlan/MotionResult" —— `MotionResult` 随子包没了,
     改写成现在真实存在的两个类型。)
    """
    st = offline_arm.move_p([0.32, 0.0, 0.35, 0.0, 0.0, 0.0])
    sent = [c for c, _ in offline_arm._tr.tx_log]
    assert P.CMD_MOVE_P in sent
    assert P.CMD_MOVE_JS not in sent                     # 没走 PC 规划
    assert hasattr(st, "mode_name")                      # RobotState


def test_move_p_accepts_a_pose_pair(offline_arm):
    """`(pos[3], R[3x3])` 是合法**单点**输入 —— 它形状像"两个位姿的序列", 不能判成序列。

    ⚠ **改靶说明**: 从已删的 `test_cartesian_motion.py` 摘出保留, 是**唯一**一条
    测 `move_p` 收位姿对形态的用例 (其余 `move_p` 用例传的都是 6 标量或序列)。
    `Arm.move_p` 里那条 `else` 分支 (`_rot.as_pose` + `mat_to_rpy`) 今天仍是活代码,
    删掉本条就等于让"位姿对静默失效"回归无人看守 —— 同形态在三条笛卡尔入口上**各有一条**
    用例, 但**都不覆盖 `move_p`**: `tests/test_cart_protocol.py` 的
    `test_a_pose_pair_produces_the_same_frame_as_the_six_scalars`(…`:1289`, `move_l`)、
    `test_a_homogeneous_4x4_pose_is_accepted_too`(`:1313`, `move_l` 的 4×4)、
    `test_move_c_pose_start_also_accepts_a_pose_pair`(`:1324`, `move_c`)、
    `test_move_path_waypoints_also_accept_a_pose_pair`(`:1341`, `move_path`)。
    (此处原写"`…:902` 守的是 `move_l`/`move_c`/`move_path`" —— **引错位置**: `:902` 落在
    `test_a_pose_pair_*` 中间, 而且当时**全仓没有**任何 `move_path` 位姿对用例; 行号与
    覆盖缺口两处一并订正。)
    """
    offline_arm.move_p(([0.32, 0.0, 0.35], ID_R))
    assert P.CMD_MOVE_P in [c for c, _ in offline_arm._tr.tx_log]


def test_ik_returns(offline_arm):
    qs = offline_arm.ik([0.30, 0.0, 0.35, 0.0, 0.0, 0.0])
    assert len(qs) == 7


def test_get_tcp(offline_arm):
    tcp = offline_arm.get_tcp().value
    assert tcp is not None and len(tcp) == 6


def test_ff_commands(offline_arm):
    offline_arm.ff_preset(1)
    offline_arm.set_ff_mask(P.FF_MASTER | P.FF_G)
    offline_arm.set_gravity_scale([2.7, 2.1, 1.75, 1.85, 2.9, 3.1, 1.2])
    offline_arm.set_inertia_scale([2.84, 8.62, 9.5, 2.56, 5.27, 4.02, 3.56])
    offline_arm.set_payload(0.4, (0.0, 0.0, 0.02))
    offline_arm.set_gravity_vector([0.0, 0.0, -9.81])
    offline_arm.save_params()


def test_move_js_send_mit_encoding(offline_arm):
    q = [0.0] * 7
    offline_arm.move_js(q, dq=[0.0] * 7)
    offline_arm.send_mit(0, q[0], 0.0, 50.0, 2.0, 0.0)
    offline_arm.send_mit_all(q, [0.0] * 7, [50.0] * 7, [2.0] * 7, [0.0] * 7)
    with pytest.raises(InvalidCommandError):
        offline_arm.send_mit(9, 0.0, 0.0, 0.0, 0.0, 0.0)


def test_state_decode_shape(offline_arm):
    st = offline_arm.get_state(refresh=True).value
    assert len(st.joints) == 7
    assert len(st.q) == 7 and len(st.dq) == 7


def test_ik_without_state_raises_litearm_error(offline_arm):
    """拿不到状态帧时 ik() 应抛本包错误, 不能漏 AttributeError (用户 except LiteArmError 抓不住)。"""
    from litearm.errors import LiteArmError

    offline_arm._a.state = None
    offline_arm._read_status = lambda timeout=0.5: None      # 模拟状态帧缺失
    with pytest.raises(LiteArmError):
        offline_arm.ik([0.30, 0.0, 0.35, 0.0, 0.0, 0.0])


def test_state_faulted_true_when_axis_latched(offline_arm):
    """固件 G7 单轴断轴不一定置全局 FAULT 位 —— 状态视图必须把 joint_fault 算作故障,
    否则 movej 不会早失败, 只会耗满 15s 超时才报'未到位'(M7 要消除的"断轴装活")。"""
    from litearm import state as ST
    st = ST.RobotState(mode=1, joint_fault=0b100)     # 只有 J3 断轴, 无全局 FAULT
    assert st.faulted is True
    assert st.fault_axes == [2]


def test_move_p_raises_litearm_error_without_state(offline_arm):
    """与 ik() 同款: 取不到状态帧时不能漏 AttributeError (要耗超时也不能崩类型)。

    注意目标位姿要**不同于桩位姿**, 否则会走"tcp 到位但无状态帧"的等待循环而不触发异常。
    """
    from litearm.errors import LiteArmError, MotionTimeoutError

    offline_arm.move_timeout = 0.4
    offline_arm._a.state = None
    offline_arm._read_status = lambda timeout=0.5: None
    with pytest.raises(MotionTimeoutError) as ei:
        offline_arm.move_p([0.9, 0.9, 0.9, 0.0, 0.0, 0.0])
    assert isinstance(ei.value, LiteArmError)
