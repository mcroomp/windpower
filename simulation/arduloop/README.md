# arduloop

A self-contained Python port of ArduPilot's traditional-helicopter attitude
and rate-control stack. It has two layers:

| Layer | Class | ArduPilot equivalent |
|-------|-------|----------------------|
| **Outer (attitude)** | `GuidedAttitudeController` | `AC_AttitudeControl::input_quaternion` + `attitude_controller_run_quat` (GUIDED mode) |
| **Inner (rate)** | `HeliRateController` | `AC_AttitudeControl_Heli` rate PID wrapper |

Both layers use parameter names that are 1:1 with ArduPilot. Gains tuned here
transfer directly to a `.parm` file.

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
from arduloop import HeliParams, RateAxisParams, HeliRateController, HeliPlant

p = HeliParams(loop_rate_hz=400.0)
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

Run the bundled demo:

```
python -m arduloop.run_demo
```

---

## Quick-start: GUIDED attitude loop (RAWES / tethered hover)

`GuidedAttitudeController` is the class to use whenever `rawes.lua` calls
`vehicle:set_target_angle_and_climbrate(...)` — i.e. any simtest or stack
test that uses GUIDED mode.

```python
from scipy.spatial.transform import Rotation
from arduloop import HeliParams, GuidedAttitudeController, GuidedAttitudeParams

hp = HeliParams(loop_rate_hz=400.0)
gp = GuidedAttitudeParams()              # AP defaults: ATC_INPUT_TC=0.15, etc.
ctrl = GuidedAttitudeController(hp, gp)

# -- 50 Hz outer tick (mirrors rawes.lua) --
ctrl.set_target_angle_and_climbrate(
    roll_deg=0.0, pitch_deg=-65.0, yaw_deg=0.0,
    climbrate_ms=0.0, sim_time=t_outer)

# Alternatively, from a 3×3 rotation matrix (body columns in NED):
# ctrl.set_target_rotation(R_body_ned, sim_time=t_outer)

# -- 400 Hz inner tick (mirrors physics runner) --
out = ctrl.update(
    q_body_ned=rot.as_quat(),        # [x,y,z,w] scipy passive body-to-NED
    gyro_body_rads=(roll_rate, pitch_rate, yaw_rate),
    dt=0.0025,
    collective_norm=0.5,
    sim_time=t_inner)

# out.roll_cyclic  -> swashplate roll  [-1, 1]
# out.pitch_cyclic -> swashplate pitch [-1, 1]
# out.yaw_cmd      -> tail / yaw axis  [-1, 1]
```

### How `GuidedAttitudeController` maps to ArduPilot

Every 400 Hz tick runs the same four steps as ArduPilot:

```
set_target_angle_and_climbrate(roll, pitch, yaw)
    stores _q_commanded

400 Hz update():
  Step 1  update_attitude_target()
             _attitude_target *= from_rotvec(_ang_vel_target * dt)
             [slews the internal target, NOT a jump to commanded]

  Step 2  attitude error:
             err_rot = _attitude_target.inv * _q_commanded
             err_rotvec = axis_angle(err_rot)

  Step 3  input_shaping_angle (per axis) -> update _ang_vel_target
             shaped by ATC_INPUT_TC + ATC_ACCEL_R/P/Y_MAX

  Step 4  attitude_controller_run_quat():
    a. thrust_vector_rotation_angles(_attitude_target, q_body)
          -> att_error (body-frame roll/pitch/yaw)
    b. update_ang_vel_target_from_att_error(att_error)
          -> P-correction via sqrt_controller
    c. ang_vel_ff = rot_t2b * _ang_vel_target   (feedforward in body frame)
    d. blend feedforward based on thrust_error_angle:
          < 30 deg  : add full ff (roll+pitch+yaw)
          30–60 deg : partial ff, yaw blends toward gyro
          > 60 deg  : no roll/pitch ff; yaw rate target = gyro (locked)
    e. (roll_rate_target, pitch_rate_target, yaw_rate_target) -> HeliRateController
```

### The 30/60 degree feedforward blending threshold

This is not intuitive and causes real SITL bugs if missed:

- The threshold is on the angle between **`_attitude_target`** and **`q_body`**,
  NOT between `_q_commanded` and `q_body`.
- Because `_attitude_target` slews slowly (via `ATC_INPUT_TC`), it usually
  stays close to the body. The `>60 deg` locked branch only fires when the
  body has been physically pushed far from where AP last computed the target.
- When locked: `yaw_rate_target = gyro[2]` — yaw PID error = 0, `yaw_cmd ≈ 0`.
  This prevents yaw I-term windup while the thrust vector is being recovered.
- For RAWES at 65° tether elevation: the slewed target should track the body
  closely. If `_attitude_target` ever diverges (timeout reset, disturbance),
  the locked branch will suppress yaw output until error < 60°.

### Timeout behaviour

If `set_target_angle_and_climbrate` is not called for `timeout_s` (default 3 s),
the next `update()` snap-resets `_q_commanded` and `_attitude_target` to the
current body attitude, zeroes `_ang_vel_target`, and resets the rate PIDs.
This matches ArduPilot re-init behaviour on GUIDED mode entry.

### Quaternion convention

`q_body_ned` throughout this module is `[x, y, z, w]` (scipy convention),
passive body-to-NED rotation. This matches `ahrs:get_quaternion()` in Lua.
To convert from a rotation matrix: `Rotation.from_matrix(R.T).as_quat()`.

### `set_target_rotation` vs `set_target_angle_and_climbrate`

`set_target_rotation(R_body_ned)` takes a 3×3 matrix where the columns are
body axes expressed in NED — the same `R_hub` used in the physics runner.
It skips the Euler round-trip, which matters at extreme tilts (>80°) where
gimbal lock degrades the ZYX decomposition. Always prefer this when you have
the full rotation matrix.

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
    collective_norm=0.5,
    saturated=(False, False, False))
# out.roll_cyclic, out.pitch_cyclic, out.yaw_cmd
```

Signal path per axis (`pid.py` / `AC_PID::update_all`):

```
target  -> [NTF notch] -> [FLTT lowpass] -> _target
                                              |
gyro  ------------------------------------------+
                           error = _target - gyro
                                              |
         +------ [NEF notch] -> [FLTE lowpass] -> _error
         |
         P  = p * _error
         D  = d * FLTD(d(_error)/dt)
         I  = integral(i * _error) clamped ±IMAX  [gated by saturated flag]
         FF = ff * _target
         DFF= d_ff * d(_target)/dt
         sum -> [swash phase rotation H_SW_H3_PHANG] -> cyclic output
```

`reset()` clears all integrators and filter states. Call after a mode
transition to prevent stale I-term from spiking the output.

---

## `HeliPlant` — design-time only

A simple coupled plant for rate-loop tuning offline (not used in simtests).
Modes: inner-loop flap response, pendulum (~0.05 Hz), tether spring (~3.77 Hz).
Sufficient to distinguish good tunings from bad, not a physics-accurate model.

---

## AP-faithful design decisions

### Why `_attitude_target` slews instead of jumping

In ArduPilot, `set_target_angle_and_climbrate` stores the commanded quaternion
but does NOT immediately set `_attitude_target` to it. Instead, every 400 Hz
tick, `input_shaping_angle` advances `_attitude_target` a small step toward
`_q_commanded`, capped by `ATC_ACCEL_*_MAX` and `ATC_INPUT_TC`. With AP
defaults loaded from `.parm` (for example `ATC_INPUT_TC=0.2 s` and
`ATC_ACC_P_MAX=600 deg/s² ≈ 10.5 rad/s²` in `copter-heli.parm`),
a 65° step takes roughly 1.5 s to ramp up. This is the correct closed-loop
transient. A simplified controller that skips this will converge faster in
simulation but slower in SITL — defeating the purpose of pre-SITL tuning.

### Why `_attitude_target` is initialised to body attitude

If `_attitude_target` started at identity and the body is at 65° tether
equilibrium, the first tick would see a 65° step error → huge rate command →
spike. `update()` sets `_attitude_target = q_body` on the very first call to
avoid this. This matches ArduPilot's `input_quaternion` initialisation.

### Why notches are applied before FLTE/FLTD

ArduPilot applies notch filters on the raw signal before the smoothing
low-passes. Reversing the order shifts the effective notch centre frequency
and reduces its depth. `pid.py` preserves the AP ordering.

---

## Tuning workflow (rate loop)

1. **No notches, no outer loop.** Tune `P`, `D`, `FF` with `FLTD ≈ 0.3 × Hz`.
   Use `signals.step` + `analysis.step_response_score`.
2. **Add tether-spring error notch.** `NEF_center_hz = 3.77`, `bandwidth = 0.4–0.6`.
   Confirm ≥ 25 dB attenuation via `empirical_frf`.
3. **Lowpass.** `FLTT = FLTE = 1.0–1.5 Hz`.
4. **Swash phase.** Sweep `H_SW_H3_PHANG ∈ [-30°, 30°]`; minimise cross-axis
   coupling at high frequency.
5. **I + IMAX last**, on slow-drift signals only.

Repeat for pitch. Yaw is independent.

---

## C++ source cross-references

| Python | C++ source |
|--------|-----------|
| `guided.py` | `AC_AttitudeControl/AC_AttitudeControl.cpp` — `input_quaternion`, `update_attitude_target`, `attitude_controller_run_quat`, `thrust_vector_rotation_angles`, `input_shaping_angle`, `input_shaping_ang_vel` |
| `guided.py` | `AP_Math/control.cpp` — `sqrt_controller` |
| `guided.py` | `ArduCopter/mode_guided.cpp` — `set_angle`, `run_angle_control` |
| `attitude_heli.py` | `AC_AttitudeControl/AC_AttitudeControl_Heli.cpp` — rate wrapper, PIRO_COMP, hover trim |
| `pid.py` | `AC_PID/AC_PID.cpp` — `update_all` |
| `swash.py` | `AP_Motors/AP_MotorsHeli_Swash.cpp` — phase rotation |
| `filters.py` | `Filter/NotchFilter.cpp`, `Filter/LowPassFilter.cpp` |
