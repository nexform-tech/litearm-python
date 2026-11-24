#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 09 · 带载辨识: 量出末端负载的质量与质心, 写进固件 (真机, 需 --go)

做什么: 按一份**空载基线**里的构型逐个 `movej` 过去持位采样, 与基线逐构型相减,
解出负载的 `[m, m·cx, m·cy, m·cz]`, 再把 `set_payload(mass, com)` 写进固件 ——
之后的重力前馈、零力拖动、阻抗控制才知道末端变重了。

为什么必须差分 (这条最容易被跳过): 静置时 `τ_meas = G_arm(q) + G_load(q) + 偏置 + ε`。
拿模型 `G_arm` 直接减, 臂模型自身的误差会全部冲进结果 —— 历史上 0.36 kg 被辨成 1.92 kg
走的就是绝对法。同一台臂、同一组构型上再测一遍空载并相减, 误差里与构型无关的部分整体
抵消。**所以本样例只做带载那一半**: 空载基线必须是更早一次空载流程的产物。

回归矩阵从哪来: 固件探针 (`0x39` 收任意 `q`, 配合临时 `set_payload`) —— PC 侧不持有
任何动力学模型, 也不引 numpy。细节见 `litearm/payload.py` 的模块头。

## 基线文件

基线是 JSON (schema `litearm.payload_baseline.v1`)。旧脚本产出的
`runs/<ts>_空载基准/noload_baseline.csv` 只需转一次:

    python3 -c "
    from litearm.payload import Baseline
    Baseline.from_csv('runs/20260918_151253_空载基准/noload_baseline.csv').save('empty.json')"

`Baseline.load()` 会核对 schema —— 别把别的 JSON 当基线读, 那只会静默算出个数。

## 安全

* 全程**保持使能持位**: 该位形一旦 `disable()` 臂会因自重坠回, 所以本样例收尾同样
  不 disable (与 `07_vel_jitter_trace.py` 同一条策略)。
* 流量由基线决定 —— 10 个构型的基线就是 10 段 movej。第一次跑请把 `--speed` 压到 0.2。
* 探针会短暂把 payload 写成 0.02 kg 的幻影负载 (幻影重力矩 ≲ 0.14 N·m, 每次一个往返),
  结束时**无条件写回**原来的 payload —— 中途抛错也写回 (`finally`)。
* 结果的 `σ` 与残差只是**可疑程度**, 不是精度承诺。夹持不牢、线缆牵拉、限位顶都在残差里;
  拿称重值对一次是最便宜的仲裁。

运行 (从仓库根目录):

    python3 examples/09_payload_identify.py --empty-baseline empty.json          # 只打印计划
    python3 examples/09_payload_identify.py --empty-baseline empty.json --go     # 真跑
    python3 examples/09_payload_identify.py --empty-baseline empty.json --go --speed 0.2
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import make_arm, parser

import litearm as pa

#: 本样例的 `--go` 说明 ("会上力"是默认文案, 这里要额外点明会写固件参数)
GO_HELP = "真正 enable/movej/写 payload (默认只读连接, 不动臂)"


def main():
    ap = parser(__doc__, go_help=GO_HELP)
    ap.add_argument("--empty-baseline", required=True, metavar="JSON",
                    help="空载基线 JSON (见文件头: 老 CSV 用 Baseline.from_csv 转一次)")
    args = ap.parse_args()

    baseline = pa.Baseline.load(args.empty_baseline)
    print(f"基线: {args.empty_baseline}  ({len(baseline)} 个构型)")
    if baseline.created:
        print(f"  采样时间: {baseline.created}")
    if baseline.source:
        print(f"  来源: {baseline.source}")
    print(f"  首个构型: {' '.join(f'{v:+.4f}' for v in baseline.poses[0])}")
    print(f"  首个构型空载力矩: {' '.join(f'{v:+7.3f}' for v in baseline.tau[0])} N·m")

    print()
    print("将执行:")
    print(f"  1) 按上面 {len(baseline)} 个构型逐个 movej (speed {args.speed}), 到位后停驻采样")
    print("  2) 固件探针求每个构型的重力回归矩阵 Y_g(q) (临时 payload 0.02 kg, 用完写回)")
    print("  3) 差分最小二乘解 [m, m·cx, m·cy, m·cz]")
    print("  4) 检出量在门槛内就 set_payload(mass, com) 并回读校验")
    if not args.go:
        print()
        print("只读: 加 --go 才动臂 (会移动机械臂, 急停放手边)")
        return

    arm = make_arm(args)
    try:
        st = arm.get_state().value
        if st is not None and not st.enabled:
            print("使能持位……")
            arm.enable()
            _wait_enabled(arm)

        print()
        print("开始辨识 (夹好负载; 中途不要碰臂)……")
        result = pa.payload.identify(arm, baseline, speed=args.speed)
        print()
        print(result)
        if len(result.pose_residual) == result.poses:
            worst = max(range(result.poses), key=lambda i: result.pose_residual[i])
            print(f"逐构型残差 RMS: "
                  f"{' '.join(f'{v:.3f}' for v in result.pose_residual)} N·m "
                  f"(最大在构型 {worst})")
        print()

        if not result.ok:
            print("不采纳: 结果没过门槛 (见上面的警告), 固件参数**未改动**")
            print("  排查顺序: 夹持是否牢固 → 基线是否同一台臂/同一份模型 → 构型跨度是否够")
            return

        pa.payload.apply(arm, result)
        mass = arm.get_ff_scalar(4).value
        com = [arm.get_ff_scalar(5, k).value for k in range(3)]
        print(f"已写入固件: payload_mass={mass:.4f} kg, payload_com="
              f"[{' '.join(f'{v:+.4f}' for v in com)}] m (法兰系)")
        print("  RAM 生效; 本样例不写 Flash (持久化请自行评估后手动操作)")
        print("  验收建议: 小步 movej 看手感和跟随误差, 或用称重值对一次质量")
    finally:
        arm.close()
        print("已断开 (刻意不 disable: 该位形失能会因自重坠回)")


def _wait_enabled(arm, timeout: float = 1.0) -> None:
    """`enable()` 返回时状态帧还没反映 (实测 2~48 ms), 必须轮询到它变真。"""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        st = arm.get_state().value
        if st is not None and st.enabled:
            return
        time.sleep(0.01)
    raise pa.LiteArmError(f"{timeout}s 内状态帧没有报使能 —— 查固件/供电/故障位")


if __name__ == "__main__":
    main()
