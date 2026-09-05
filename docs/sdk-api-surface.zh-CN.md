# LiteArm SDK 接口对照（python / js / cpp）

三端客户端与 server（`pylitearm.Arm`，经 `litearm-server` RPC 暴露）之间已对齐的
**运行时配置类接口**方法矩阵。**RPC 名是唯一契约**：下表所有客户端方法最终都转发到
同一个 server RPC。

范围说明（2026-09）：**自动负载辨识接口 `identify_payload` 已从客户端 SDK 移除**。
它此前只在 python 客户端暴露；该特性整体迁到独立分支。`pylitearm/identify` 下保留的
离线辨识工具仅供离线标定/该分支使用，**不属于** server RPC 面。

## 负载（末端质量 + 质心）

| Server RPC | python (远程) | js (node) | js (browser/ws) | cpp |
|---|---|---|---|---|
| `set_payload` | `set_payload(mass, com=[0,0,0])` | `setPayload(mass, com)` | `setPayload(mass, com)` | `set_payload(mass, com)` |
| `get_payload` | `get_payload()` | `getPayload()` | `getPayload()` | `get_payload()` |
| `save_payload` | `save_payload()` | `savePayload()` | `savePayload()` | `save_payload()` |

- `set_payload` 运行期生效（下一控制周期读到新负载），`mass<=0` 视为空载。
- `save_payload` 把当前运行期 `mass`/`com` 持久化到 yaml（自动重算
  `metadata.checksum_sha256`），重启后仍生效。

## 安装方向（base 姿态 / 重力方向）

| Server RPC | python (远程) | js (node) | js (browser/ws) | cpp |
|---|---|---|---|---|
| `set_installation` | `set_installation(base_rpy=None, gravity=None)` | `setInstallation({base_rpy?, gravity?})` | `setInstallation(base_rpy?, gravity?)` | `set_installation(base_rpy, gravity)` |
| `get_installation` | `get_installation()` | `getInstallation()` | `getInstallation()` | `get_installation()` |
| `save_installation` | `save_installation()` | `saveInstallation()` | `saveInstallation()` | `save_installation()` |

- `set_installation` 运行期改写 base 系重力方向（M/C 不依赖 base 朝向）；
  `base_rpy` 与 `gravity` 二选一。
- `save_installation` 持久化 `base_rpy` 到 yaml（恒写 base 配置，`model` 段不被
  serial profile 覆盖）。仅用裸 `gravity` 向量设置、无记录的 `base_rpy` 时保存会被拒绝
  ——请改用 `base_rpy` 设置以便持久化。

## 重力标定系数（逐关节 `scale[7]`）

| Server RPC | python (远程) | js (node) | js (browser/ws) | cpp |
|---|---|---|---|---|
| `set_gravity_scale` | `set_gravity_scale(scale, transition_s=2.0)` | `setGravityScale(scale, transition_s=2)` | `setGravityScale(scale, transition_s=2)` | `set_gravity_scale(scale, transition_s=2.0)` |
| `get_gravity_scale` | `get_gravity_scale()` | `getGravityScale()` | `getGravityScale()` | `get_gravity_scale()` |
| `save_gravity_scale` | `save_gravity_scale()` | `saveGravityScale()` | `saveGravityScale()` | `save_gravity_scale()` |

- `set_gravity_scale` 作用到重力前馈 `G(q)=scale·G_CAD(q)`，默认 **2s 线性渐变**
  保证补偿力矩不跳变（即"禁止补偿突变"的安全前提）。`transition_s<=0` 立即生效
  （仅离线/静止用）。渐变中再次 set 会从当前中间值平滑续走，无台阶。
- `get_gravity_scale` 返回 `{ scale, target }`：渐变中 scale=当前中间值、稳定后
  `target=null`；空闲无控制律运行时也会按墙钟收敛。
- `save_gravity_scale` 持久化渐变的 **target**（标定意图）到 yaml。上限
  `yaml safety.max_gravity_scale`（默认 10.0），在 set 时校验。

## 已移除接口

| 已移除 | 原因 |
|---|---|
| `identify_payload`（python 远程 + `examples/identify_payload.py`） | 自动负载辨识已移出客户端 SDK 面（特性在独立分支）；js/cpp 从未暴露过它。 |

## 部署注意

- 新 server RPC（`set/get/save gravity_scale`、`save_payload`、
  `save_installation`）需**重启 litearm-server** 才能被客户端调到（pylitearm 模块在
  进程内缓存）。
- 各客户端库需从本仓库源码重新安装/发包（js/cpp 本次为源码级改动）。
