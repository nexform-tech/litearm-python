# tau_eval — current-loop torque estimation accuracy evaluation

Python implementation of issue
[#44](https://github.com/nexform-tech/litearm-python/issues/44):
collect → inverse dynamics → error visualization.

The tool records every `q / dq / tau` status frame from the arm, replays a
rigid-body inverse dynamics model (the same world-frame RNEA the firmware
runs), and reports per-joint error metrics, SVG figures and a Markdown
report. Everything is reproducible from the persisted session files and the
options recorded in `metrics.json`.

```
collect (hardware) ──► status.csv + meta.json + model.json
                              │
                              ▼
analyze (offline)   ──► metrics.json + timeseries.csv + figures/*.svg + report.md
```

## Requirements

- Python 3.9+ (the SDK's minimum), standard library only for the analysis path.
- Hardware collection uses the `litearm` SDK (pyserial). Optional PyArrow
  enables Parquet output (`--parquet`).
- No numpy / matplotlib; figures are emitted as plain SVG.

## Quick start

```bash
# 1. Dry run: validate the plan against the firmware soft limits, no motion.
python3 tools/tau_eval/run.py

# 2. Real run: enable, execute three scenarios, analyze, plot, report.
python3 tools/tau_eval/run.py --go

# 3. Re-analyze a recorded run with different filter settings.
python3 tools/tau_eval/run.py analyze runs/20261029_153000_tau_eval \
        --cutoff-ddq 3 --gravity 0,9.81,0
```

`run.py` can be launched from any working directory. `--help` on every
sub-command lists the options.

## Scenarios

`--scenario all` (default) runs these in order:

| kind | excitation | purpose |
|---|---|---|
| `hold` | static hold at the centre pose, `move_js` streamed at `--command-hz` | gravity model + static bias |
| `sine` | low-speed sine `q0 + amp·sin(2πft)` with analytic `dq`, amplitudes automatically narrowed until inside the soft-limit margin | inertia / Coriolis / friction, delay |
| `poses` | several slow point-to-point moves, each streamed as a jerk-continuous `move_js` reference with a dwell at the target | gravity across the workspace |

Defaults: 3 s hold, 10 s sine (0.12 rad, 0.20 Hz), then up to five poses at
1.0/0.7/0.5/0.35/0.25 of the usable range with a 1.2 s dwell each (targets that
fail the margin check are dropped).

Every scenario is driven exclusively through `move_js` re-sent at
`--command-hz` (default 50 Hz). Two reasons:

- The firmware has a 0.1 s command watchdog; a silent hold fails soft (lower
  stiffness, gravity feed-forward cut) and the recorded "hold" would be a
  watchdog-tripped artifact. The hold and the dwells therefore stream a
  constant `move_js` setpoint for their whole duration.
- `poses` does **not** use `movej`. Its static hold relies on position error,
  and the gravity feed-forward bias on a bench with a non-default gravity
  vector leaves 0.04–0.05 rad of steady-state error — above the SDK arrival
  tolerance (`q_tol = 0.03 rad`), so `movej` would raise
  `MotionTimeoutError` and lose the whole segment. Instead each move streams a
  smoothstep reference (`POSE_SPEED_RAD_S = 0.5 rad/s` × `--speed`, min 0.6 s
  per move) with its analytic velocity, finishes with an exact target frame at
  zero velocity, then dwells.

## Collected data

Per scenario directory:

| file | content |
|---|---|
| `status.csv` | uniform-grid samples: `t, seq, mode, flags, joint_fault, q0..q6, dq.., tau.., t_mos.., t_coil.., err..` |
| `meta.json` | timestamps, host, scenario spec/plan/result, fs, firmware version, joint count, geometry hash, payload, gravity, scales, friction model, soft limits, initial/final state, motor and ambient temperatures, static gravity check, warnings |
| `model.json` | full body / joint model snapshot used for the inverse dynamics (`ModelSnapshot.to_dict()`) |
| `status.parquet` | optional, identical columns (`--parquet`) |

The recorder hooks `arm._a._on_status` — the single status funnel of the SDK —
so it never opens a second reader and does not disturb the SDK's own arrival
logic. Frames are resampled to a uniform grid at save time; discrete fields
(`seq`, `mode`, `flags`, `err`) take the most recent value, continuous fields
are linearly interpolated.

## Inverse dynamics

For each sample the tool evaluates

```
tau = M(q)·ddq + C(q,dq)·dq + G(q) + tau_jm + tau_friction
```

with the firmware model snapshot (9 bodies, 7 joints), in newtons · metres and
radians, joint order as reported by the SDK. Gravity is injected as a base
linear acceleration `−g`, matching the firmware; the default vector is
`(0, 0, -9.81)` and can be overridden with `--gravity x,y,z` (the test bench
uses `(0, +9.81, 0)`).

`analyze.rnea.gravity_torque` is verified against the firmware golden probe
(`MODEL_PROBE_Q` / `G_FACTORY_PROBE`) in `tests/test_tau_eval.py`.

### Processing chain (all options recorded in `metrics.json`)

1. Confirm the uniform sampling grid (re-sample if the jitter exceeds 2 %).
2. Zero-phase 2nd-order Butterworth low-pass (`filtfilt`): `q/dq` at
   `--cutoff-dq` 15 Hz, measured `tau` at `--cutoff-tau` 10 Hz.
3. `ddq` by central difference, then low-pass at `--cutoff-ddq` 5 Hz.
4. Evaluate the inverse dynamics per sample.
5. Trim `max(2/cutoff_ddq, 0.1)` s at both ends to drop the reflection
   transient of the zero-phase filter.

Zero-phase filtering is used deliberately: a causal filter would add a
frequency-dependent delay between the measured and the model torque. The
residual phase lag that remains (sensor latency, current-loop bandwidth) is
measured explicitly by scanning lags in ±`--max-delay-ms`.

## Outputs

- `metrics.json` — aggregate, per-joint (`rmse, mae, bias, std, peak_abs,
  rel_rmse, r2, corr, meas_rms, model_mean, meas_mean`), speed and pose
  strata, delay scan, residual correlations, component RMS, static bias,
  diagnosis.
- `timeseries.csv` — per joint: `q, dq, ddq, tau_meas, tau_model, tau_scaled,
  G, C, I, F`.
- `figures/*.svg` — per-joint torque comparison, residual, histogram and
  speed-strata bars; RMSE/bias bars; measured-vs-model parity scatter; delay
  scan; normalized residual histogram (≈ 33 files).
- `report.md` — summary, diagnosis, method, per-joint tables, limitations,
  and the exact reproduce command.
- `summary.md` at the run root — one row per scenario.

## Reading the diagnosis

`analyze.diagnose` inspects the residual structure and attributes the error:

| evidence | likely source |
|---|---|
| static hold bias, signed, uniform across poses | gravity model or current-loop gain/offset calibration |
| residual correlates with `sign(dq)` / `|dq|` | Coulomb / viscous friction model mismatch |
| residual correlates with `ddq` / inertia term | body inertia (`I`) or COM parameters |
| negative lag after delay scan shifts the minimum | filter or sensor delay (check `--cutoff-*`) |
| mean relative RMSE < 2 % | model and measurement agree; residual is numerical noise |

## Safety

- Without `--go` the tool is read-only and only prints the plan.
- Every target pose and sine envelope is validated against the firmware soft
  limits with a `MIN_MARGIN = 0.15 rad` margin; invalid amplitudes are narrowed
  automatically.
- The arm is enabled before recording and **left enabled afterwards** — at a
  held pose, disabling lets gravity drop the arm (same policy as
  `examples/07`). Keep an e-stop within reach; disable deliberately when
  supported.
- A static gravity check compares the held `tau` with `model.get_gravity(q)`
  and is skipped when any joint moves faster than 0.05 rad/s.
- No torque/stiffness feed-forward is commanded by the sine scenario.

## Limitations

- `tau` is the current-loop estimate, not a joint torque sensor; current-loop
  calibration errors are indistinguishable from model errors.
- The factory model is regression-identified: masses are mostly 1, inertias
  can be negative and the mass matrix is not positive definite. The tool never
  inverts `M`; it only evaluates the forward product.
- Gravity is validated only in static holds; inertia/friction identification
  is out of scope (see `examples/09_payload_identify.py`).
- Motor winding / MOS temperatures are reported, but ambient temperature must
  be supplied manually (`--ambient-c`).
- The default gravity is `(0, 0, -9.81)`; a bench mounted differently must pass
  `--gravity`.
- The tool covers a single arm configuration; tool payload has to be described
  in the model snapshot (or identified separately) before comparing absolute
  torques.

## Hardware-free example

```bash
python3 tools/tau_eval/examples/make_synthetic.py
```

The script synthesises a sine session from the factory model with the same
RNEA the analyzer uses, then runs analyze → figures → report. The residual is
pure numerical noise (mean RMSE ≈ 3e-6 N·m), so it is a smoke test for the
whole pipeline. Output goes to `tools/tau_eval/examples/synthetic_sine/`
(ignored by git — generated artifacts are never committed).

## Tests

```bash
python3 -m pytest tests/test_tau_eval.py -q
```

Offline: firmware gravity golden probe, mass-matrix/RNEA consistency, filter
and resampling properties, CSV round-trip, soft-limit validation, and a
synthetic end-to-end analyze/plot/report run.
