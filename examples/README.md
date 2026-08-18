# litearm-python 客户端样例

通过 zenoh 远程连接 litearm-server（运行在控制器/地瓜上）控制机械臂。

与 pylitearm 本地样例的区别：

- **无硬件依赖**：客户端只发 RPC，硬件在 server 端
- **无 dry-run**：连上的就是真机，运动会真实发生
- **无 config**：配置在 server 端加载

## 前提

1. 控制器（地瓜）上 litearm-server 已启动：

   ```bash
   # 在地瓜上
   cd /home/sunrise/luo && ./start_server.sh
   ```

2. 客户端已装 litearm-python（或用 `PYTHONPATH=src`）
3. 客户端与地瓜网络互通

## 运行

```bash
# 默认连接地瓜 (192.168.31.237:7447)，见 _common.py DEFAULT_ENDPOINT
python3 examples/01_read_state.py

# 指定其他端点
python3 examples/01_read_state.py --endpoint tcp/127.0.0.1:7447

# 指定 arm-id
python3 examples/01_read_state.py --arm-id armA
```

## 样例列表

| 样例 | 演示 | 是否运动 |
|---|---|---|
| `01_read_state.py` | 连接 + 读状态 + TCP 位姿 | ❌ 只读 |
| `02_movej.py` | 关节空间运动 movej | ✅ 运动 |
| `03_fk_ik.py` | 正逆运动学（纯计算 RPC） | ❌ 不运动 |
| `04_movel.py` | 笛卡尔直线运动 movel + plan_movel | ✅ 运动 |

## ⚠️ 安全提示

运动样例（02/04）会**真实驱动机械臂**：

- 首次运行 speed 保持 0.1~0.2
- 人站在急停旁
- 确保机械臂周围无人无障碍

## 位姿格式

客户端不依赖 numpy，位姿用纯 Python list：

```python
pose = [position, rotation]
position = [px, py, pz]                              # 3 元素
rotation = [[r00,r01,r02],                           # 3x3 行主序旋转矩阵
            [r10,r11,r12],
            [r20,r21,r22]]
```
