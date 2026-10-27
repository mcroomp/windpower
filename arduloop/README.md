# arduloop

A self-contained Python port of ArduPilot's traditional-helicopter attitude
and rate-control stack. It has two layers:

| Layer | Class | ArduPilot equivalent |
|-------|-------|----------------------|
| **Outer (attitude)** | `GuidedAttitudeController` | `AC_AttitudeControl::input_quaternion` + `attitude_controller_run_quat` (GUIDED mode) |
| **Inner (rate)** | `HeliRateController` | `AC_AttitudeControl_Heli` rate PID wrapper |

Both layers use ArduPilot-style parameter names. This models the control math
only, not mode entry, arming/spool state, or failsafes — validate those in
SITL. Python tuning results are not hardware acceptance.

Current RAWES Lua steady/passive/takeoff behavior uses the direct-thrust Guided
angle path (`set_target_angle_and_rate_and_throttle(...)`). This package also
keeps the rate-only and climbrate-compatible Guided paths for parity/testing.

---

## Module map

```
arduloop/
├── __init__.py         public exports
├── params.py           RateAxisParams, HeliParams  (all AP param names)
├── guided.py           GuidedAttitudeController + GuidedAttitudeParams
├── attitude_heli.py    HeliRateController, HeliRateOutput
├── pid.py              AC_PID — target/error notch + 3 LPFs + PIDFFD
├── swash.py            SwashH3 — H_SW_H3_PHANG phase rotation
├── filters.py          LowPassFilter1p, NotchFilter  (AP biquad)
├── plant.py            HeliPlant — coupled rotational + pendulum + spring
├── signals.py          step, chirp, multisine, doublet generators
├── analysis.py         empirical FRF, stability margins, step-response score
└── run_demo.py         end-to-end example — rate loop + notch filter
```

---

## Quick-start: rate loop only

```python
from arduloop import HeliParams, HeliRateController, HeliPlant
from simulation.param_defaults import load_ap_params

p = HeliParams.from_ap_dict(load_ap_params())
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

Run the bundled demo:

```
python -m arduloop.run_demo
```

---

## Quick-start: GUIDED attitude loop

`GuidedAttitudeController` is the class to use for the Guided attitude path in
simulation. Current `rawes.lua` steady/passive/takeoff behavior mirrors
`vehicle:set_target_angle_and_rate_and_throttle(...)`; the port also retains
`set_target_angle_and_climbrate(...)` for API parity with ArduPilot.

```python
from scipy.spatial.transform import Rotation
from arduloop import HeliParams, GuidedAttitudeController, GuidedAttitudeParams
from simulation.param_defaults import load_ap_params

hp = HeliParams.from_ap_dict(load_ap_params())
gp = GuidedAttitudeParams.from_heli_params(hp)
ctrl = GuidedAttitudeController(hp, gp)
rot = Rotation.identity()

# -- One outer tick (current Lua steady uses this API at 50 Hz) --
ctrl.set_target_angle_and_rate_and_throttle(
    roll_deg=0.0, pitch_deg=-65.0, yaw_deg=0.0,
    roll_rate_degs=0.0, pitch_rate_degs=0.0, yaw_rate_degs=0.0,
    throttle=0.26, sim_time=0.0)

# Alternatively, from a 3×3 rotation matrix (body columns in NED):
# ctrl.set_target_rotation(R_body_ned, sim_time=t_outer)

# -- 400 Hz inner tick (mirrors physics runner) --
out = ctrl.update(
    q_body_ned=rot.as_quat(),        # [x,y,z,w], body-to-NED
    gyro_body_rads=(0.0, 0.0, 0.0),
    dt=0.0025,
    collective_norm=0.5,
    sim_time=0.0025)

# out.roll_cyclic  -> swashplate roll  [-1, 1]
# out.pitch_cyclic -> swashplate pitch [-1, 1]
# out.yaw_cmd      -> tail / yaw axis  [-1, 1]
```

### How `GuidedAttitudeController` maps to ArduPilot

The master walkthrough for target integration, third-order angle shaping,
thrust/heading decomposition, the heading-error clamp, and 30/60-degree
feedforward blending is
[GUIDED_CONTROL_LOOPS.md](../design/GUIDED_CONTROL_LOOPS.md).
This README owns package APIs/examples, not a second set of loop equations.

### Timeout behaviour

If no target setter is called for `timeout_s` (default 3 s),
the next `update()` snap-resets `_q_commanded` and `_attitude_target` to the
current body attitude, zeroes `_ang_vel_target`, and resets the rate PIDs.
It also clears direct thrust and returns to the port's simplified vertical
path. This is Python-adapter behavior, not a model of firmware `GUID_TIMEOUT`
or a supported hardware recovery procedure.

### Quaternion convention

`q_body_ned` throughout this module is `[x, y, z, w]` (scipy convention),
body-to-NED rotation. The scalar ordering differs from Lua/MAVLink `[w,x,y,z]`.
For a matrix whose columns are body axes in NED, use
`Rotation.from_matrix(R).as_quat()`; do not transpose it.

### `set_target_rotation` vs `set_target_angle_and_climbrate`

`set_target_rotation(R_body_ned)` takes a 3×3 matrix where the columns are
body axes expressed in NED — the same `R_hub` used in the physics runner.
It skips the Euler round-trip, which matters at extreme tilts (>80°) where
gimbal lock degrades the ZYX decomposition. Prefer this when you have
the full rotation matrix.

The climb-rate path is a simplified vertical controller, not a full port of
ArduPilot's `AC_PosControl`. Setters do not all clear each other's state, so
use a new controller instance when switching between API paths in an
experiment.

---

## Parameter reference

### `GuidedAttitudeParams` (outer loop)

| Field | AP parameter | Default | Notes |
|-------|-------------|---------|-------|
| `ATC_ANG_RLL_P` | `ATC_ANG_RLL_P` | loaded from .parm | rad/s per rad |
| `ATC_ANG_PIT_P` | `ATC_ANG_PIT_P` | loaded from .parm | |
| `ATC_ANG_YAW_P` | `ATC_ANG_YAW_P` | loaded from .parm | |
| `ATC_ACCEL_R_MAX` | `ATC_ACC_R_MAX` (fallback `ATC_ACCEL_R_MAX`) | loaded from .parm | deg/s² in AP 4.7 |
| `ATC_ACCEL_P_MAX` | `ATC_ACC_P_MAX` (fallback `ATC_ACCEL_P_MAX`) | loaded from .parm | |
| `ATC_ACCEL_Y_MAX` | `ATC_ACC_Y_MAX` (fallback `ATC_ACCEL_Y_MAX`) | loaded from .parm | |
| `ATC_RATE_R_MAX` | `ATC_RATE_R_MAX` | 0 | deg/s, 0=unlimited |
| `ATC_RATE_P_MAX` | `ATC_RATE_P_MAX` | 0 | |
| `ATC_RATE_Y_MAX` | `ATC_RATE_Y_MAX` | 0 | |
| `ATC_INPUT_TC` | `ATC_INPUT_TC` | loaded from .parm | s; controls slew speed |

Use `GuidedAttitudeParams.from_heli_params(hp)` to pull values from an
existing `HeliParams` (avoids duplicating the same gains).

### `HeliParams` (rate loop + shared attitude gains)

| Field | AP parameter | Default |
|-------|-------------|---------|
| `roll / pitch / yaw` | per-axis `RateAxisParams` | see below |
| `ATC_ANG_RLL_P / PIT_P / YAW_P` | `ATC_ANG_*_P` | loaded from .parm |
| `ATC_ACCEL_R/P/Y_MAX` | `ATC_ACC_*_MAX` (fallback `ATC_ACCEL_*_MAX`) | loaded from .parm |
| `ATC_RATE_R/P/Y_MAX` | `ATC_RATE_*_MAX` | loaded from .parm |
| `HOVR_ROL_TRM_cd` | `ATC_HOVR_ROL_TRM` | 0 centi-deg |
| `PIRO_COMP_enabled` | `ATC_PIRO_COMP` | False |
| `H_SW_H3_PHANG` | `H_SW_H3_PHANG` | 0 deg |
| `loop_rate_hz` | `SCHED_LOOP_RATE` | 400 Hz |
| `output_limit` | — | 1.0 |

### `RateAxisParams` (one axis)

| Field | AP parameter | Default |
|-------|-------------|---------|
| `P / I / D / FF` | `ATC_RAT_xxx_P/I/D/FF` | loaded from .parm |
| `IMAX` | `ATC_RAT_xxx_IMAX` | loaded from .parm |
| `D_FF` | `ATC_RAT_xxx_D_FF` | 0 |
| `PDMX` | `ATC_RAT_xxx_PDMX` | 0 (disabled) |
| `FLTT / FLTE / FLTD` | `ATC_RAT_xxx_FLTT/FLTE/FLTD` | loaded from .parm |
| `NTF_center_hz / bandwidth_hz / attn_db` | `ATC_RAT_xxx_NTF → FILT*` | 0 (disabled) |
| `NEF_center_hz / bandwidth_hz / attn_db` | `ATC_RAT_xxx_NEF → FILT*` | 0 (disabled) |

Yaw values are loaded from the ArduPilot .parm source by default.

---

## `HeliRateController` — inner rate loop

```python
out = ctrl.update(
    rate_target_rads=(roll_rate, pitch_rate, yaw_rate),
    gyro_rate_rads=(gr, gp, gy),
    dt=dt,
    collective_norm=0.5)
# out.roll_cyclic, out.pitch_cyclic, out.yaw_cmd
```

Anti-windup saturation flags (AP `_motors.limit.roll/pitch/yaw`) are computed
internally each tick from the combined roll/pitch cyclic magnitude vs
`H_CYC_MAX` (and yaw vs the output limit), then fed into the NEXT tick's
`update_all(limit=...)` — mirroring AP's one-cycle-delayed feedback from
`AP_MotorsHeli_Single::move_actuators`.


PID filtering, leakage, P+D derating, and pirouette-compensation equations
are owned by [the control-loop reference](../design/GUIDED_CONTROL_LOOPS.md#6-rate-control-loop-helicopter-specific).

`reset()` clears all integrators and filter states. Call after a mode
transition to prevent stale I-term from spiking the output.

---

## `HeliPlant` — design-time only

A simple coupled plant for rate-loop tuning offline (not used in simtests).
It is useful for comparative controller experiments, not as a physics-accurate
runtime model.

---

## AP-faithful design decisions

### Why `_attitude_target` slews instead of jumping

In ArduPilot, `set_target_angle_and_climbrate` stores the commanded quaternion
but does NOT immediately set `_attitude_target` to it. Instead, every 400 Hz
tick, the port integrates its previous target rate and uses
`attitude_command_model` to update jerk-limited rate/acceleration toward
`_q_commanded`. Skipping this step would change the transient response the
model is meant to reproduce.

### Why `_attitude_target` is initialised to body attitude

If `_attitude_target` started at identity while the body was already tilted,
the first tick would see a large step error and command spike. `update()` sets `_attitude_target = q_body` on the very first call to
avoid this. (Firmware mode-entry behavior is described in
[arming.md](../design/arming.md).)

### Why notches are applied before FLTE/FLTD

`pid.py` applies target/error notch filters before their low-passes. The D
derivative is taken from filtered error, then passed through FLTD; FF uses
filtered target. SMAX derates P+D rather than rate-limiting the complete
output. Detailed equations and control-layer distinctions are owned by
[GUIDED_CONTROL_LOOPS.md](../design/GUIDED_CONTROL_LOOPS.md).

---

## C++ source cross-references

Use the implementations in `guided.py`, `attitude_heli.py`, and `pid.py`,
together with the [control-loop owner](../design/GUIDED_CONTROL_LOOPS.md).
To compare against firmware, use the ArduPilot release pinned in
`simulation/Dockerfile`.
