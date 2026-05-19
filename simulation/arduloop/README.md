# arduloop

A self-contained Python port of ArduPilot's traditional-helicopter rate /
attitude loop, intended for **early controller design and parameter tuning**
before moving to ArduPilot SITL or hardware. The block diagram, signal flow,
and parameter names mirror the C++ implementations so that every gain or
filter tuned here transfers directly:

| arduloop field                       | ArduPilot parameter        |
|--------------------------------------|----------------------------|
| `RateAxisParams.P / I / D / FF`      | `ATC_RAT_xxx_P / I / D / FF` |
| `RateAxisParams.IMAX / PDMX / D_FF`  | `ATC_RAT_xxx_IMAX / PDMX / D_FF` |
| `RateAxisParams.FLTT / FLTE / FLTD`  | `ATC_RAT_xxx_FLTT / FLTE / FLTD` |
| `RateAxisParams.NTF_*`               | `ATC_RAT_xxx_NTF` → `FILT*`  |
| `RateAxisParams.NEF_*`               | `ATC_RAT_xxx_NEF` → `FILT*`  |
| `HeliParams.HOVR_ROL_TRM_cd`         | `ATC_HOVR_ROL_TRM`          |
| `HeliParams.PIRO_COMP_enabled`       | `ATC_PIRO_COMP`             |
| `HeliParams.H_SW_H3_PHANG`           | `H_SW_H3_PHANG`             |
| `HeliParams.loop_rate_hz`            | `SCHED_LOOP_RATE`           |

C++ references:
- `libraries/AC_PID/AC_PID.cpp` — `update_all` mirrored in
  [`pid.py`](pid.py).
- `libraries/AC_AttitudeControl/AC_AttitudeControl_Heli.cpp` — rate-loop
  wrapper, PIRO_COMP, hover roll trim mirrored in
  [`attitude_heli.py`](attitude_heli.py).
- `libraries/AP_Motors/AP_MotorsHeli_Swash.cpp` — phase rotation in
  [`swash.py`](swash.py).
- `libraries/Filter/NotchFilter.cpp` and `LowPassFilter.cpp` — in
  [`filters.py`](filters.py).

## Layout

```
arduloop/
├── __init__.py
├── filters.py         LowPassFilter1p, NotchFilter  (AP-compatible)
├── pid.py             AC_PID port with FLTT/FLTE/FLTD + NTF/NEF
├── swash.py           SwashH3 — phase rotation only
├── attitude_heli.py   HeliRateController, PIRO_COMP, hover trim
├── plant.py           Coupled rotational + pendulum + tether-spring plant
├── signals.py         step, chirp, multisine, doublet
├── analysis.py        empirical FRF, stability margins, step score
├── params.py          RateAxisParams, HeliParams dataclasses
└── run_demo.py        end-to-end tuning example
```

## Quick start

```python
from arduloop import HeliParams, RateAxisParams, HeliRateController, HeliPlant

p = HeliParams(loop_rate_hz=400.0, H_SW_H3_PHANG=0.0)
p.roll = RateAxisParams(P=0.12, I=0.10, D=0.004, FF=0.05,
                        FLTT=1.5, FLTD=20.0,
                        NEF_center_hz=3.77, NEF_bandwidth_hz=0.5)
ctrl = HeliRateController(p)
plant = HeliPlant()

dt = 1.0 / p.loop_rate_hz
gp = gq = gr = 0.0
for _ in range(1600):
    out = ctrl.update(rate_target_rads=(1.0, 0.0, 0.0),
                      gyro_rate_rads=(gp, gq, gr), dt=dt,
                      collective_norm=0.5)
    gp, gq, gr = plant.step(out.roll_cyclic, out.pitch_cyclic,
                            out.yaw_cmd, 0.5, dt)
```

Run the bundled demo (no plotting needed):

```
python -m arduloop.run_demo
```

It prints step-response metrics and the closed-loop magnitude at the
tether-spring frequency, before and after enabling a 3.77 Hz error notch.

## Recommendations for controller design before ArduPilot

The package was built around a specific tuning workflow. The recommendations
below apply to *any* heli with the dynamics characterised in the user
analysis (inner rate loop crossover well above 1 Hz, pendulum mode near
0.05 Hz, tether spring near 3.77 Hz that coincides with the rotor nutation
crossover).

### 1. Match ArduPilot's loop topology, not just "a PID"

The rate axis is *not* a textbook PID. Per axis it is:

```
target ──► [target notch] ──► [FLTT lowpass] ──┐
                                                ├──► error ──► [error notch] ──► [FLTE LP] ──► _error
gyro ────► [INS gyro filt] ─────────────────────┘
                                                         │
                                  P = kp·_error          │
                                  D = kd·LP_FLTD(d_error)│   ── sum ──► [swash phase rot] ──► cyclic
                                  I = ∫ki·_error  (anti-windup, IMAX)
                                  FF = kff·_target
                                  DFF = kdff·d_target
```

This is what `pid.py` implements. Use it instead of `scipy.signal.PID` or a
hand-rolled `kp*e + ki*∫e + kd*de`.

### 2. Tune in this order — each step is one ArduPilot parameter

1. **Inner rate loop, no notches, no outer loop**. Tune `P`, `D`, `FF` with
   `FLTD ≈ 0.3 × loop_rate`. Validate with `signals.step` and
   `analysis.step_response_score`. → `ATC_RAT_RLL_P / D / FF / FLTD`.
2. **Add the tether-spring error notch**: `NEF_center_hz = 3.77`,
   `NEF_bandwidth_hz = 0.4–0.6` (Q ≈ 7). Confirm via `empirical_frf` that
   loop gain drops ≥ 25 dB at 3.77 Hz. → `ATC_RAT_RLL_NEF` pointing at a
   `FILT*` slot.
3. **Lowpass below pendulum coupling region**: set `FLTT = FLTE = 1.0–1.5
   Hz`. Confirms the closed loop never excites the nutation crossover from
   the outside. → `ATC_RAT_RLL_FLTT / FLTE`.
4. **Swash phase**: sweep `H_SW_H3_PHANG ∈ [-30°, 30°]` and pick the value
   that minimises off-axis (cross-coupled) response at high frequency — the
   only frequency band where the phase is well-defined. → `H_SW_H3_PHANG`.
5. **I and IMAX last**, on slow-drift signals only. → `ATC_RAT_RLL_I / IMAX`.

Repeat 1–5 for pitch. Do yaw independently — it does not couple through the
swash.

### 3. Things to bake in from day one

- **Run at ArduPilot's real loop rate** (typically 400 Hz for heli). Notch
  Q-factors and discretisation effects change a lot between 50 Hz and 400 Hz.
- **Use the same sign conventions**: body-frame rates in rad/s, swash output
  in `[-1, 1]`, phase angle in degrees with the sign convention from
  `AP_MotorsHeli_Swash.cpp:125-127` (`servoN_pos − phase_angle`). The
  `SwashH3` implementation here matches that convention so a value found in
  Python ports directly.
- **Probe with the same signals**: `signals.logarithmic_chirp`,
  `signals.multisine` and `signals.step` are the same shapes that
  `probe_open_loop_plant` / `probe_step_response` produce on the rig and
  that `pymavlink` can decode from logs.
- **Log everything** — `analysis.empirical_frf` and `step_response_score`
  expect plain numpy arrays; dump them to `.npz` so you can overlay
  Python-sim and decoded `.bin` traces on the same axes.

### 4. What this package deliberately does **not** do

- **No exact bit-for-bit replica of `AC_PID`**. Structural equivalence is
  enough for tuning to transfer; small numerical differences are absorbed by
  the final retune on hardware. If you need bit-exact behaviour, run SITL.
- **No 6-DOF heli model**. `HeliPlant` is a coupled rotational + pendulum +
  tether-spring model calibrated to the modes the user identified
  (inner-loop crossover, pendulum at ~0.05 Hz, tether spring at ~3.77 Hz at
  the nutation crossover). It is sufficient to differentiate good tunings
  from bad ones in the regimes that matter; it is not a fidelity model for
  certification.
- **No frequency-dependent decoupling matrix**. ArduPilot itself does not
  expose one (only the static `H_SW_H3_PHANG` and `PIRO_COMP` for yaw rate).
  If you decide a frequency-dependent decoupler is necessary, this package
  is the right place to prototype it — see "Extending" below — but you will
  also need to add the hook on the C++ side before flying.

### 5. Path to ArduPilot

1. **Python-only tuning** — what this package supports today.
2. **SITL with the JSON backend** (the user's current branch
   `sitl-json-rpm-feedback-4.6.3` is suitable) — drive ArduPilot from
   `HeliPlant` over the JSON socket. The tuned parameter set goes directly
   into `params.parm`.
3. **Hardware** — a final autotune-style refinement (`AUTOTUNE` mode or
   manual frequency-sweep tuning) on the rig, starting from the SITL set.

### 6. Extending

To experiment with the things ArduPilot does *not* yet provide:

- **Frequency-dependent decoupling**: add a method to `HeliRateController`
  that runs a 2×2 dynamic mixer between `(roll_out, pitch_out)` before
  `swash.mix`. Implement it as e.g. two complementary biquads producing the
  high-frequency and low-frequency portions, each rotated by a different
  phase angle. This is the algorithmic form of "swash phase is frequency-
  dependent: ~0° at high freq, ~−90° at low freq, sliding through 180° at
  the nutation crossover."
- **MIMO loop shaping**: build the open-loop transfer matrix from
  `analysis.empirical_frf` runs at multiple operating points, then design
  the decoupler with `python-control` (`pip install control`).

The plant and controller modules are intentionally small and dataclass-driven,
so live retuning is just mutation of `params.roll`, etc., followed by
`ctrl.reload_params()`.
