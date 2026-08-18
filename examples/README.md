# litearm-python Client Examples

Control the arm by connecting to the arm control service (running on the arm
controller) from any machine.

Key points:

- **No hardware dependencies**: the arm driver lives server-side; the client only
  needs network access
- **It drives the real arm**: motion happens for real — there is no dry-run
- **No local configuration**: configuration is loaded server-side

## Prerequisites

1. The arm control service is running on the controller:

   ```bash
   # on the controller
   cd /home/sunrise/luo && ./start_server.sh
   ```

2. `litearm-python` is installed on the client (or use `PYTHONPATH=src`)
3. Client and controller are on the same network

## Run

```bash
# default endpoint: tcp/192.168.31.237:7447 (see _common.py DEFAULT_ENDPOINT)
python3 examples/01_read_state.py

# specify another endpoint
python3 examples/01_read_state.py --endpoint tcp/127.0.0.1:7447

# specify an arm-id
python3 examples/01_read_state.py --arm-id armA
```

## Examples

| Example | Demonstrates | Moves? |
|---|---|---|
| `01_read_state.py` | Connect + read state + TCP pose | ❌ read-only |
| `02_movej.py` | Joint-space move `movej` | ✅ motion |
| `03_fk_ik.py` | Forward/inverse kinematics (pure computation) | ❌ no motion |
| `04_movel.py` | Cartesian line move `movel` + `plan_movel` | ✅ motion |

## ⚠️ Safety

The motion examples (02/04) **drive the real arm**:

- Keep speed at 0.1–0.2 on the first runs
- Stand by the emergency stop
- Make sure nobody and no obstacles are near the arm

## Pose Format

The client does not depend on numpy — poses are plain Python lists:

```python
pose = [position, rotation]
position = [px, py, pz]                              # 3 elements
rotation = [[r00,r01,r02],                           # 3x3 row-major rotation matrix
            [r10,r11,r12],
            [r20,r21,r22]]
```
