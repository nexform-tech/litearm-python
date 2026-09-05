# LiteArm SDK API Surface (python / js / cpp)

Cross-SDK method matrix for the runtime-configuration interfaces that are
currently aligned across the three clients and the server (`pylitearm.Arm` as
exposed through `litearm-server` RPC). RPC names are the canonical contract:
every client method below forwards to the same server RPC.

Scope note (2026-09): the **auto payload identification** interface
(`identify_payload`) has been removed from the client SDK surface. It was only
ever exposed in the python client; the underlying feature moved to a separate
branch. Offline identification helpers remain in `pylitearm/identify` for
offline calibration / that branch and are **not** part of the server RPC set.

## Payload (end-effector mass + center of mass)

| Server RPC | python (remote) | js (node) | js (browser/ws) | cpp |
|---|---|---|---|---|
| `set_payload` | `set_payload(mass, com=[0,0,0])` | `setPayload(mass, com)` | `setPayload(mass, com)` | `set_payload(mass, com)` |
| `get_payload` | `get_payload()` | `getPayload()` | `getPayload()` | `get_payload()` |
| `save_payload` | `save_payload()` | `savePayload()` | `savePayload()` | `save_payload()` |

- `set_payload` applies the load at runtime (compensation takes effect next
  control cycle). `mass <= 0` clears the load.
- `save_payload` persists the current runtime `mass`/`com` into the yaml
  (recomputing `metadata.checksum_sha256`); effective after server restart.

## Installation (base orientation / gravity direction)

| Server RPC | python (remote) | js (node) | js (browser/ws) | cpp |
|---|---|---|---|---|
| `set_installation` | `set_installation(base_rpy=None, gravity=None)` | `setInstallation({base_rpy?, gravity?})` | `setInstallation(base_rpy?, gravity?)` | `set_installation(base_rpy, gravity)` |
| `get_installation` | `get_installation()` | `getInstallation()` | `getInstallation()` | `get_installation()` |
| `save_installation` | `save_installation()` | `saveInstallation()` | `saveInstallation()` | `save_installation()` |

- `set_installation` writes the base-frame gravity vector at runtime (M/C do
  not depend on base orientation). Requires `base_rpy` **or** `gravity`.
- `save_installation` persists `base_rpy` to the yaml (only the base config;
  `model` is not profile-overridable). Setting orientation with a bare
  `gravity` vector that has no recorded `base_rpy` is refused by save — pass
  `base_rpy` to enable persistence.

## Gravity calibration scale (per-joint, `scale[7]`)

| Server RPC | python (remote) | js (node) | js (browser/ws) | cpp |
|---|---|---|---|---|
| `set_gravity_scale` | `set_gravity_scale(scale, transition_s=2.0)` | `setGravityScale(scale, transition_s=2)` | `setGravityScale(scale, transition_s=2)` | `set_gravity_scale(scale, transition_s=2.0)` |
| `get_gravity_scale` | `get_gravity_scale()` | `getGravityScale()` | `getGravityScale()` | `get_gravity_scale()` |
| `save_gravity_scale` | `save_gravity_scale()` | `saveGravityScale()` | `saveGravityScale()` | `save_gravity_scale()` |

- `set_gravity_scale` feeds the gravity feed-forward `G(q)=scale·G_CAD(q)` and
  defaults to a 2 s linear transition so the compensation torque never jumps
  (this is the "no abrupt compensation change" safety requirement). Pass
  `transition_s <= 0` for instant apply (offline/static only). A re-set during a
  transition continues smoothly from the current mid-transition value.
- `get_gravity_scale` returns `{ scale, target }`: mid-transition value while
  easing, `target: null` when stable. The value also converges by wall-clock
  even when no control loop is running.
- `save_gravity_scale` persists the transition **target** (the intended
  calibration) to the yaml. Upper bound is `yaml safety.max_gravity_scale`
  (default 10.0), enforced at `set`.

## Removed interfaces

| Removed | Reason |
|---|---|
| `identify_payload` (python remote + `examples/identify_payload.py`) | Auto payload identification moved out of the client SDK surface (feature on a separate branch). js/cpp never exposed it. |

## Deployment notes

- New server RPCs (`set/get/save gravity_scale`, `save_payload`,
  `save_installation`) require a **litearm-server restart** to pick up the new
  `pylitearm.Arm` methods (module cache is per-process).
- Client libraries must be re-installed/repackaged from source after these
  edits (the js/cpp changes here are source-level).
