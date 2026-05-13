# ArduPilot Helicopter Attitude Controllers

This document describes the attitude-control PID stack used by ArduPilot's
**traditional helicopter** frame (`AC_AttitudeControl_Heli`) and provides
a faithful Python re-implementation of each loop. All references point at
the ArduPilot source tree (branch state at the time of writing).

## Overall structure

The heli controller is a **two-loop cascade**, identical in topology to the
multicopter one but with heli-specific extensions:

```
                  ┌────────────────┐      target rate     ┌────────────────┐
  attitude error  │   Angle P      │  ─────────────────▶  │  Rate PID      │  ──▶ swash / tail
  (rad)           │   ATC_ANG_*_P  │   (rad/s)            │  ATC_RAT_*_*   │     servos
                  └────────────────┘                      └────────────────┘
```

- **Outer loop** — a pure P controller on attitude error, gain
  `ATC_ANG_RLL_P`, `ATC_ANG_PIT_P`, `ATC_ANG_YAW_P`. Input: quaternion
  attitude error. Output: a body-frame angular-rate target.
- **Inner loop** — three `AC_HELI_PID` instances (a subclass of the generic
  `AC_PID`) running at the main loop rate (≥400 Hz). Input: gyro vs. rate
  target. Output: normalized [-1, 1] command sent to the swashplate /
  tail mixer.

What makes the heli inner loops different from the multi-rotor ones:

- **Leaky integrator** (`update_leaky_i`). On heli, the I-term holds the
  *static* control surface trim needed to fight rotor torque, tail thrust,
  CG offset etc. That trim must not run away during ground idle, so when
  the rotor is not spun up (or when the `leaky_i` flag is set) the
  integrator decays toward `±ILMI` at `AC_ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE`
  (= 0.02 per call) instead of being frozen.
- **Pirouette compensation** (`ATC_PIRO_COMP`). Rotates the roll & pitch
  rate I-terms by `-yaw_rate*dt` so the hover trim follows heading
  during fast yaws. This prevents the disk-tilt trim built up in one axis
  from corrupting the other axis as the airframe spins.
- **Feed-forward gain** (`FF`, `D_FF`) is enabled on heli; the FF term is
  applied **outside** the slew-rate-limited P+D path, then summed into the
  final actuator command.

Source files:
- [libraries/AC_AttitudeControl/AC_AttitudeControl_Heli.cpp](../../ardupilot/libraries/AC_AttitudeControl/AC_AttitudeControl_Heli.cpp)
- [libraries/AC_AttitudeControl/AC_AttitudeControl_Heli.h](../../ardupilot/libraries/AC_AttitudeControl/AC_AttitudeControl_Heli.h)
- [libraries/AC_PID/AC_HELI_PID.cpp](../../ardupilot/libraries/AC_PID/AC_HELI_PID.cpp)
- [libraries/AC_PID/AC_HELI_PID.h](../../ardupilot/libraries/AC_PID/AC_HELI_PID.h)
- [libraries/AC_PID/AC_PID.cpp](../../ardupilot/libraries/AC_PID/AC_PID.cpp) (`update_all`, `update_i`)

---

## Common heli PID building block (`AC_HELI_PID`)

Every rate axis uses an instance of this class. It is identical to the
generic `AC_PID` except for `update_leaky_i()` which decays the integrator
toward `±ILMI` instead of holding it.

```python
import math


class HeliPID:
    """Faithful re-implementation of ArduPilot's AC_HELI_PID.

    The control law per call to `update_all` is:

        target_f      = LPF(target,    FLTT)            # target filter
        error_f       = LPF(target_f - measured, FLTE)  # error filter
        d_error_f     = LPF(d/dt error_f, FLTD)         # derivative filter
        d_target      = d/dt target_f                   # used by D_FF

        integrator   += error_f * I * dt                # anti-wind-up gated
        integrator    = clamp(integrator, ±IMAX)

        P_out         = error_f * P
        D_out         = d_error_f * D
        # SMAX slew limiter scales (P_out + D_out) by Dmod ∈ [0.1, 1] when
        # the high-frequency component would exceed the configured slew rate.
        # PDMX clamps |P_out + D_out|.
        return P_out + integrator + D_out               # FF added by caller

    `get_ff()` returns the feed-forward term:

        FF_out = FF * target_f + D_FF * d_target
    """

    # ---- Tunable parameters (mirror ATC_RAT_*_* params) ----
    def __init__(self,
                 P, I, D,
                 FF=0.0, D_FF=0.0,
                 IMAX=0.0, ILMI=0.1,
                 FLTT=20.0, FLTE=20.0, FLTD=10.0,
                 SMAX=0.0, PDMX=0.0):
        # Gains
        self.kp, self.ki, self.kd = P, I, D
        self.kff, self.kdff       = FF, D_FF
        # Integrator clamp + leak floor
        self.imax     = IMAX
        self.leak_min = ILMI
        # Filter cutoffs (Hz)
        self.fltt, self.flte, self.fltd = FLTT, FLTE, FLTD
        # Slew rate limit on (P+D) and PDMX clamp
        self.smax = SMAX
        self.pdmx = PDMX

        # ---- State ----
        self.target       = 0.0   # target after FLTT
        self.error        = 0.0   # error after FLTE
        self.derivative   = 0.0   # derivative after FLTD
        self.t_derivative = 0.0   # d(target)/dt for D_FF
        self.integrator   = 0.0
        self._reset       = True  # forces first sample to seed filters

    # ---- Helpers ----
    @staticmethod
    def _alpha(dt, fc_hz):
        """First-order low-pass alpha used everywhere in AC_PID.
        alpha = dt / (dt + 1/(2*pi*fc)).  fc_hz<=0 disables filter."""
        if fc_hz <= 0.0:
            return 1.0
        rc = 1.0 / (2.0 * math.pi * fc_hz)
        return dt / (dt + rc)

    # ---- Heli-specific I-term leak ----
    def update_leaky_i(self, leak_rate):
        """Bleeds I-term toward ±ILMI. Called when rotor is not spun up
        (rate yaw) or when AC_AttitudeControl_Heli's leaky_i flag is set
        (rate roll/pitch). Replaces the normal anti-windup hold."""
        if self.ki == 0.0:
            return
        if   self.integrator >  self.leak_min:
            self.integrator -= (self.integrator - self.leak_min) * leak_rate
        elif self.integrator < -self.leak_min:
            self.integrator -= (self.integrator + self.leak_min) * leak_rate

    # ---- Main PID step ----
    def update_all(self, target, measured, dt, motor_limit):
        """One PID iteration. Returns P + I + D (no FF).

        motor_limit: True if the actuator on this axis is currently saturated.
                     Used to gate I-term integration (anti-windup)."""
        if self._reset:
            # First sample after reset: seed filters, no derivative yet.
            self._reset       = False
            self.target       = target
            self.error        = target - measured
            self.derivative   = 0.0
            self.t_derivative = 0.0
        else:
            # Target low-pass filter (FLTT).
            tgt_last = self.target
            self.target += self._alpha(dt, self.fltt) * (target - self.target)

            # Error low-pass filter (FLTE).
            err_last = self.error
            raw_err  = self.target - measured
            self.error += self._alpha(dt, self.flte) * (raw_err - self.error)

            # Derivative of filtered error, then filtered (FLTD).
            # d(target)/dt is also captured for D_FF.
            if dt > 0.0:
                d = (self.error - err_last) / dt
                self.derivative += self._alpha(dt, self.fltd) * (d - self.derivative)
                self.t_derivative = (self.target - tgt_last) / dt

        # ---- I-term update with anti-wind-up ----
        # ArduPilot freezes integration only if the motor is saturated AND
        # the proposed delta would push it further into saturation.
        if self.ki != 0.0:
            i_delta = self.error * self.ki * dt
            push_into_saturation = motor_limit and (
                (i_delta > 0 and self.integrator > 0) or
                (i_delta < 0 and self.integrator < 0)
            )
            if not push_into_saturation:
                self.integrator += i_delta
                self.integrator = max(-self.imax, min(self.imax, self.integrator))

        P_out = self.error      * self.kp
        D_out = self.derivative * self.kd

        # SMAX / PDMX (simplified — see AC_PID::update_all for the real
        # slew-rate limiter `_slew_limiter.modifier`).
        if self.pdmx > 0.0:
            pd = max(-self.pdmx, min(self.pdmx, P_out + D_out))
            # split clamp proportionally
            scale = pd / (P_out + D_out) if (P_out + D_out) != 0.0 else 1.0
            P_out *= scale
            D_out *= scale

        return P_out + self.integrator + D_out

    def get_ff(self):
        """FF term: kff*target_filtered + kdff*d(target)/dt."""
        return self.kff * self.target + self.kdff * self.t_derivative

    def reset_filter(self):
        self._reset = True

    def reset_I(self):
        self.integrator = 0.0
```

---

## 1. Roll rate PID (`ATC_RAT_RLL_*`)

**Source:** `AC_AttitudeControl_Heli::rate_bf_to_motor_roll_pitch`
in [AC_AttitudeControl_Heli.cpp:460](../../ardupilot/libraries/AC_AttitudeControl/AC_AttitudeControl_Heli.cpp#L460).

```cpp
if (_flags_heli.leaky_i) {
    _pid_rate_roll.update_leaky_i(AC_ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE);
}
float roll_pid = _pid_rate_roll.update_all(rate_roll_target_rads,
                                           rate_rads.x, _dt,
                                           _motors.limit.roll) + _actuator_sysid.x;
float roll_ff  = _pid_rate_roll.get_ff();
float roll_out = constrain_float(roll_pid + roll_ff,
                                 -AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX,
                                  AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX);
_motors.set_roll(roll_out);
```

**Parameters:** `ATC_RAT_RLL_P/I/D/FF/D_FF/IMAX/ILMI/FLTT/FLTE/FLTD/SMAX/PDMX/NTF/NEF`.
Typical heli defaults are around `P=0.10, I=0.20, D=0.005, FF=0.05, IMAX=0.5`.

The output drives the **lateral cyclic** input on the swashplate, range ±1.

```python
RP_OUT_MAX     = 1.0     # AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX
HELI_LEAK_RATE = 0.02    # AC_ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE


def rate_to_motor_roll(pid_roll, gyro_x, target_rate, dt,
                       motor_limit_roll, leaky_i):
    """Mirror of the roll branch of rate_bf_to_motor_roll_pitch.

    leaky_i is set by the heli attitude controller during e.g. ground idle
    so the hover-trim integrator drifts back toward ILMI rather than holding
    a stale value.
    """
    if leaky_i:
        pid_roll.update_leaky_i(HELI_LEAK_RATE)

    pid_out = pid_roll.update_all(target_rate, gyro_x, dt, motor_limit_roll)
    ff_out  = pid_roll.get_ff()                # FF outside the slew limiter
    return max(-RP_OUT_MAX, min(RP_OUT_MAX, pid_out + ff_out))
```

---

## 2. Pitch rate PID (`ATC_RAT_PIT_*`)

Identical structure to roll, just on the longitudinal cyclic axis. Same
`AC_HELI_PID` class, same wrapper, same ±1 output clamp.

**Parameters:** `ATC_RAT_PIT_P/I/D/FF/D_FF/IMAX/ILMI/FLTT/FLTE/FLTD/SMAX/PDMX/NTF/NEF`.

```python
def rate_to_motor_pitch(pid_pitch, gyro_y, target_rate, dt,
                        motor_limit_pitch, leaky_i):
    """Pitch branch of rate_bf_to_motor_roll_pitch."""
    if leaky_i:
        pid_pitch.update_leaky_i(HELI_LEAK_RATE)
    pid_out = pid_pitch.update_all(target_rate, gyro_y, dt, motor_limit_pitch)
    ff_out  = pid_pitch.get_ff()
    return max(-RP_OUT_MAX, min(RP_OUT_MAX, pid_out + ff_out))
```

### Pirouette compensation (couples roll & pitch I-terms)

Run *after* both axes have been updated. When the airframe yaws, the
disk-tilt I-terms get rotated by `-yaw_rate*dt` so the hover trim stays
attached to the wind, not the airframe.

```python
import math


def piro_comp(pid_roll, pid_pitch, yaw_rate, dt, enabled):
    """Re-implementation of AC_AttitudeControl_Heli::rate_bf_to_motor_roll_pitch
    pirouette-compensation tail end (lines 494–507)."""
    if not enabled:
        return
    i_r = pid_roll.integrator
    i_p = pid_pitch.integrator
    # Unit vector rotated by -yaw_rate*dt
    c = math.cos(-yaw_rate * dt)
    s = math.sin(-yaw_rate * dt)
    pid_roll.integrator  = i_r * c - i_p * s
    pid_pitch.integrator = i_p * c + i_r * s
```

---

## 3. Yaw rate PID (`ATC_RAT_YAW_*`)

**Source:** `AC_AttitudeControl_Heli::rate_target_to_motor_yaw`
in [AC_AttitudeControl_Heli.cpp:512](../../ardupilot/libraries/AC_AttitudeControl/AC_AttitudeControl_Heli.cpp#L512).

```cpp
if (!((AP_MotorsHeli&)_motors).rotor_runup_complete()) {
    _pid_rate_yaw.update_leaky_i(AC_ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE);
}
float pid = _pid_rate_yaw.update_all(rate_target_rads, rate_yaw_actual_rads,
                                     _dt, _motors.limit.yaw) + _actuator_sysid.z;
float vff = _pid_rate_yaw.get_ff() * _feedforward_scalar;
float yaw_out = constrain_float(pid + vff,
                                -AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX,
                                 AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX);
return yaw_out;   // sent to _motors.set_yaw()
```

**Parameters:** `ATC_RAT_YAW_P/I/D/FF/D_FF/IMAX/ILMI/FLTT/FLTE/FLTD/SMAX/PDMX/NTF/NEF`.
Typical defaults `P=0.18, I=0.12, D=0.003, FF=0.024, IMAX=0.4`.

Differences from roll/pitch:
- Leak gating uses `rotor_runup_complete()` (not `leaky_i` flag) — so the
  I-term decays only while the main rotor is below operating speed.
- The FF term is multiplied by `_feedforward_scalar` (rotor-spool ramp).
- Output drives the tail rotor / tail servo, range ±1.

```python
YAW_OUT_MAX = 1.0   # AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX


def rate_to_motor_yaw(pid_yaw, gyro_z, target_rate, dt,
                      motor_limit_yaw, rotor_runup_complete,
                      feedforward_scalar=1.0):
    """Mirror of AC_AttitudeControl_Heli::rate_target_to_motor_yaw."""
    if not rotor_runup_complete:
        pid_yaw.update_leaky_i(HELI_LEAK_RATE)

    pid_out = pid_yaw.update_all(target_rate, gyro_z, dt, motor_limit_yaw)
    ff_out  = pid_yaw.get_ff() * feedforward_scalar
    return max(-YAW_OUT_MAX, min(YAW_OUT_MAX, pid_out + ff_out))
```

---

## 4. Outer attitude P controllers (`ATC_ANG_RLL_P`, `ATC_ANG_PIT_P`, `ATC_ANG_YAW_P`)

These are not full PIDs — they are pure P with a square-root shaping
function (`sqrt_controller`) so the rate target tapers nicely as the error
shrinks, capped by the per-axis acceleration limit. They live in the
generic `AC_AttitudeControl` base, so heli inherits them unchanged.

Conceptually for each axis:

```
target_rate = sqrt_controller(angle_error,
                              kp = ATC_ANG_*_P,
                              accel_max = ATC_ACCEL_*_MAX,
                              dt)
```

`sqrt_controller` (from `AC_AttitudeControl`) behaves like:
- For small errors: linear, `target_rate ≈ kp * error`.
- For large errors: `target_rate ≈ sign(error) * sqrt(2 * accel_max * |error|)`,
  i.e. the rate that, decelerated at `accel_max`, will hit zero exactly at
  zero error. This avoids overshoot when `kp` would otherwise demand a rate
  the rate loop cannot achieve.

```python
def sqrt_controller(error, kp, accel_max, dt):
    """Re-implementation of AC_AttitudeControl::sqrt_controller used by
    the outer angle loop. Returns a rate command (rad/s).

    error     : attitude error (rad). Positive => target ahead of measured.
    kp        : ATC_ANG_*_P
    accel_max : ATC_ACCEL_*_MAX in rad/s² (0 disables the sqrt branch).
    dt        : controller dt (only needed when kp == 0 to limit jerk).
    """
    if kp <= 0.0:
        # Pure sqrt fallback (rare — kp is normally >0 on a tuned heli).
        if accel_max <= 0.0 or dt <= 0.0:
            return 0.0
        return math.copysign(
            math.sqrt(2.0 * accel_max * abs(error)),
            error,
        )

    if accel_max <= 0.0:
        # No accel cap: pure proportional.
        return kp * error

    # Linear region threshold: switch to sqrt when |error| exceeds the
    # value at which kp*error == sqrt(2*accel_max*|error|).
    linear_dist = accel_max / (kp * kp)
    if abs(error) > linear_dist:
        return math.copysign(
            math.sqrt(2.0 * accel_max * (abs(error) - 0.5 * linear_dist)),
            error,
        )
    return kp * error


def angle_to_rate_target(angle_error_rad, kp, accel_max, dt):
    """Outer attitude loop on one axis (roll/pitch/yaw). Output is the rate
    target that gets handed to the inner rate PID."""
    return sqrt_controller(angle_error_rad, kp, accel_max, dt)
```

**Parameters:**

| Param | Meaning |
|-------|---------|
| `ATC_ANG_RLL_P` | Roll angle-error → roll-rate gain |
| `ATC_ANG_PIT_P` | Pitch angle-error → pitch-rate gain |
| `ATC_ANG_YAW_P` | Yaw angle-error → yaw-rate gain |
| `ATC_ACCEL_R_MAX` / `ATC_ACCEL_P_MAX` / `ATC_ACCEL_Y_MAX` | per-axis maximum angular acceleration; sets the sqrt-controller break point |

---

## 5. End-to-end stitched example

Putting one full axis (roll) together: outer angle P → inner rate PID →
swash command, simulated against a trivial 1st-order plant.

```python
def heli_axis_step(angle_setpoint, angle_meas, gyro,
                   pid_rate, kp_angle, accel_max,
                   dt, motor_limit, leaky_i):
    """Run one full attitude+rate iteration on a single axis.

    Returns: (servo_command in [-1,1], rate_target rad/s)
    """
    # --- Outer loop: angle error -> rate target ---
    angle_error = angle_setpoint - angle_meas
    rate_target = angle_to_rate_target(angle_error, kp_angle, accel_max, dt)

    # --- Inner loop: rate PID -> servo command (for roll/pitch axis) ---
    if leaky_i:
        pid_rate.update_leaky_i(HELI_LEAK_RATE)
    pid_out = pid_rate.update_all(rate_target, gyro, dt, motor_limit)
    ff_out  = pid_rate.get_ff()
    servo   = max(-RP_OUT_MAX, min(RP_OUT_MAX, pid_out + ff_out))
    return servo, rate_target


if __name__ == "__main__":
    # Tune values are illustrative defaults, not flight-quality.
    pid_roll = HeliPID(P=0.10, I=0.20, D=0.005,
                       FF=0.05, D_FF=0.0,
                       IMAX=0.5, ILMI=0.1,
                       FLTT=20.0, FLTE=20.0, FLTD=10.0)

    dt        = 1.0 / 400.0          # 400 Hz inner loop
    kp_angle  = 6.0                  # ATC_ANG_RLL_P
    accel_max = math.radians(110000) / 1000.0  # ~110000 cdss² in rad/s²

    angle    = 0.0                   # measured roll (rad)
    gyro     = 0.0                   # measured roll rate (rad/s)
    setpoint = math.radians(20.0)    # commanded 20° bank

    for k in range(800):  # 2 seconds
        servo, rt = heli_axis_step(
            angle_setpoint=setpoint,
            angle_meas=angle,
            gyro=gyro,
            pid_rate=pid_roll,
            kp_angle=kp_angle,
            accel_max=accel_max,
            dt=dt,
            motor_limit=False,
            leaky_i=False,
        )
        # Trivial plant: servo -> angular accel, with damping
        ang_acc = servo * 30.0 - gyro * 0.5
        gyro   += ang_acc * dt
        angle  += gyro    * dt

        if k % 50 == 0:
            print(f"t={k*dt:5.3f}s  servo={servo:+.3f}  "
                  f"rate_tgt={math.degrees(rt):+6.2f}°/s  "
                  f"angle={math.degrees(angle):+6.2f}°")
```

---

## Parameter cheat-sheet

| Parameter group | Owner | Purpose |
|-----------------|-------|---------|
| `ATC_ANG_RLL_P` / `PIT_P` / `YAW_P` | Outer angle loop | Attitude-error → rate-target proportional gain. |
| `ATC_ACCEL_R_MAX` / `P_MAX` / `Y_MAX` | Outer angle loop | Caps the rate slope used by `sqrt_controller`. |
| `ATC_RAT_RLL_*`  / `PIT_*` / `YAW_*` | Rate PID per axis | `P I D FF D_FF IMAX ILMI FLTT FLTE FLTD SMAX PDMX NTF NEF`. |
| `ATC_PIRO_COMP`  | Heli only | Enable pirouette compensation on roll/pitch I-terms. |
| `AC_ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE` (=0.02) | Compile-time | Per-call decay of leaky I-term toward `±ILMI`. |
| `AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX` / `_YAW_*` (=1.0) | Compile-time | Final output clamp on roll/pitch & yaw rate loops. |

## 6. ACRO mode (heli) — what bypasses what

ACRO on a traditional heli (`ModeAcro_Heli` in
[ArduCopter/mode_acro_heli.cpp](../../ardupilot/ArduCopter/mode_acro_heli.cpp))
is the mode that exposes the **inner rate loop** most directly. The pilot
sticks command body-frame angular **rates**, not angles, so the outer
angle-P controller from §4 is **not used**. There are three sub-flavours
inside ACRO depending on the airframe:

### a) "Normal" ACRO (no flybar, no external tail gyro)

Path: pilot sticks → `get_pilot_desired_angle_rates()` → optional
"virtual flybar" leak → `input_rate_bf_roll_pitch_yaw()` →
the **same `AC_HELI_PID` rate loops described in §1–3**.

Key points:
- The outer angle-P loop (§4) is skipped entirely. The rate target is set
  from the stick.
- A *virtual flybar* (`virtual_flybar`, lines 130–154) leaks the attitude
  target back toward the current attitude when `ACRO_TRAINER == OFF`.
  Leak rates come from `ACRO_BAL_PITCH` / `ACRO_BAL_ROLL`, or `3.0` while
  landed. This emulates the natural self-centering of a real flybar so a
  pilot can fly stick-released.
- If `ACRO_OPTIONS bit 0 (RATE_LOOP_ONLY)` is set, the call goes to
  `input_rate_bf_roll_pitch_yaw_2()` which skips even the attitude-target
  bookkeeping and feeds the rate PID directly — the cleanest "rate only"
  path in ArduPilot.
- Yaw stick scaling uses `ACRO_YAW_P` (a separate gain from
  `ATC_ANG_YAW_P`) — see `get_pilot_desired_yaw_rate()`.
- I-term reset rules in `ModeAcro_Heli::run`:
  - `SHUT_DOWN`: `reset_rate_controller_I_terms()` immediately.
  - `GROUND_IDLE` / `THROTTLE_UNLIMITED` while landed:
    `reset_rate_controller_I_terms_smoothly()` *only if the airframe
    isn't using the leaky integrator* (because the leaky-I machinery
    already handles it).
  - In flight: I-terms are kept (heli needs the trim).

### b) Flybar-equipped ACRO

If `motors->has_flybar()` is true, the rate PIDs are bypassed entirely.
`attitude_control->use_flybar_passthrough(true, ...)` flips the
`_flags_heli.flybar_passthrough` bit, and inside the rate-controller
runner ([AC_AttitudeControl_Heli.cpp:427](../../ardupilot/libraries/AC_AttitudeControl/AC_AttitudeControl_Heli.cpp#L427))
the cyclic command is written directly to the swashplate:

```cpp
if (_flags_heli.flybar_passthrough) {
    _motors.set_roll(_passthrough_roll  / 4500.0f);
    _motors.set_pitch(_passthrough_pitch / 4500.0f);
} else {
    rate_bf_to_motor_roll_pitch(...);    // the §1/§2 PID path
}
```

The mechanical flybar provides damping. ArduPilot's rate PIDs would fight
it, so they're disabled.

### c) External-gyro tail (`H_TAIL_TYPE = 1`, SERVO_EXTGYRO)

`motors->supports_yaw_passthrough()` returns true; `tail_passthrough` is
set in the heli flags and the yaw rate PID is bypassed:

```cpp
if (_flags_heli.tail_passthrough) {
    _motors.set_yaw(_passthrough_yaw / 4500.0f);
} else {
    _motors.set_yaw(rate_target_to_motor_yaw(_rate_gyro.z, _ang_vel_body.z));
}
```

The external gyro hardware (e.g. a 3-axis tail gyro) closes the yaw rate
loop in the servo itself, so the FC just hands it the pilot's yaw stick.

### Python: ACRO entry point

```python
def acro_heli_step(stick_roll, stick_pitch, stick_yaw,
                   pid_roll, pid_pitch, pid_yaw,
                   gyro, dt,
                   has_flybar, has_extgyro_tail,
                   landed, motor_runup_complete,
                   acro_balance_roll=1.0, acro_balance_pitch=1.0,
                   trainer_off=True,
                   passthrough_roll=0.0, passthrough_pitch=0.0,
                   passthrough_yaw=0.0):
    """High-level ACRO-mode dispatcher. Returns (roll_out, pitch_out, yaw_out).

    `gyro` is a tuple (gx, gy, gz) in rad/s. `stick_*` are commanded body
    rates in rad/s (already converted from raw stick).
    `passthrough_*` are normalized -1..1 used in flybar / extgyro paths.
    """
    # --- Flybar path: bypass rate PIDs entirely ---
    if has_flybar:
        roll_out  = passthrough_roll
        pitch_out = passthrough_pitch
        if has_extgyro_tail:
            yaw_out = passthrough_yaw
        else:
            # ACRO_YAW_P stick-scaled rate still runs through the yaw PID.
            yaw_out = rate_to_motor_yaw(pid_yaw, gyro[2], stick_yaw,
                                        dt, motor_limit_yaw=False,
                                        rotor_runup_complete=motor_runup_complete)
        return roll_out, pitch_out, yaw_out

    # --- Normal heli ACRO: rate-only on roll/pitch/yaw ---
    target_roll, target_pitch, target_yaw = stick_roll, stick_pitch, stick_yaw

    # Virtual flybar leak (only when trainer is off).
    if trainer_off:
        leak_p = 3.0 if landed else acro_balance_pitch
        leak_r = 3.0 if landed else acro_balance_roll
        # NB: the real implementation leaks the *attitude target* back
        # toward current attitude and adds an EF→BF rate correction;
        # captured here as a simple proportional drag toward zero rate.
        target_roll  *= max(0.0, 1.0 - leak_r * dt)
        target_pitch *= max(0.0, 1.0 - leak_p * dt)

    if has_extgyro_tail:
        yaw_out = passthrough_yaw
    else:
        yaw_out = rate_to_motor_yaw(pid_yaw, gyro[2], target_yaw, dt,
                                    motor_limit_yaw=False,
                                    rotor_runup_complete=motor_runup_complete)

    roll_out  = rate_to_motor_roll (pid_roll,  gyro[0], target_roll,  dt,
                                    motor_limit_roll=False,  leaky_i=False)
    pitch_out = rate_to_motor_pitch(pid_pitch, gyro[1], target_pitch, dt,
                                    motor_limit_pitch=False, leaky_i=False)
    return roll_out, pitch_out, yaw_out
```

**ACRO-relevant parameters:**

| Param | Meaning |
|-------|---------|
| `ACRO_RP_RATE` / `ACRO_Y_RATE` | Max stick→rate scaling (deg/s at full stick) |
| `ACRO_RP_EXPO` / `ACRO_Y_EXPO` | Stick expo |
| `ACRO_BAL_ROLL` / `ACRO_BAL_PITCH` | Virtual-flybar leak rates |
| `ACRO_TRAINER` | 0=Off (raw), 1=Leveling, 2=Limited |
| `ACRO_OPTIONS` bit 0 | `RATE_LOOP_ONLY` — bypass attitude-target bookkeeping |
| `ACRO_YAW_P` | Yaw stick scaling for flybar path (separate from `ATC_ANG_YAW_P`) |

---

## 7. Direct Drive Fixed Pitch (DDFP) tails — where they fit

`H_TAIL_TYPE` (`AP_MotorsHeli_Single::TAIL_TYPE`) selects how the yaw
rate PID's output is delivered to the airframe. Defined in
[AP_MotorsHeli_Single.cpp:32](../../ardupilot/libraries/AP_Motors/AP_MotorsHeli_Single.cpp#L32):

| Value | Name | What it is |
|-------|------|------------|
| 0 | `SERVO`             | Variable-pitch tail blades driven by a servo (classic). |
| 1 | `SERVO_EXTGYRO`     | Same servo but yaw rate is closed by an external gyro; FC bypasses its yaw PID. |
| 2 | `DIRECTDRIVE_VARPITCH` | Tail rotor has its own ESC speed governor + var-pitch blades. |
| 3 | `DIRECTDRIVE_FIXEDPITCH_CW`  | DDFP, blades are fixed pitch, **motor RPM** controls thrust, CW rotation. |
| 4 | `DIRECTDRIVE_FIXEDPITCH_CCW` | Same as 3 but CCW (output is sign-flipped). |

**Where DDFP plugs in.** The yaw rate PID (§3) is unchanged — it still
produces a normalized command in [-1, 1]. What changes is the *output
stage* in `AP_MotorsHeli_Single::output_to_motors()`
([line 469](../../ardupilot/libraries/AP_Motors/AP_MotorsHeli_Single.cpp#L469)):

```cpp
case TAIL_TYPE::DIRECTDRIVE_FIXEDPITCH_CCW:
    _servo4_out *= -1.0;          // invert sign for CCW airframe
    FALLTHROUGH;
case TAIL_TYPE::DIRECTDRIVE_FIXEDPITCH_CW: {
    thr_lin.update_lift_max_from_batt_voltage();
    switch (_spool_state) {
      case SHUT_DOWN: case GROUND_IDLE: case SPOOLING_DOWN:
          output_to_ddfp_tail(0.0);  // motor stopped
          break;
      case SPOOLING_UP: case THROTTLE_UNLIMITED:
          // PID's signed yaw command -> linearized throttle 0..1
          output_to_ddfp_tail(thr_lin.thrust_to_actuator(_servo4_out));
          break;
    }
    break;
}
```

Key implications for the rate loop and tuning:

1. **Unidirectional thrust.** A fixed-pitch fan can only push one way.
   Negative yaw command translates into "less thrust", not reverse
   thrust. If the airframe needs the opposite torque, you can't get it
   — that's why a DC (clockwise) main rotor can use either CW or CCW
   tail and the firmware sign-flips one of them.
2. **Static `H_YAW_TRIM`.** Adds a *fixed offset* to the PID output so
   the I-term doesn't have to build up to fight steady main-rotor drag.
   See [`get_yaw_offset()`](../../ardupilot/libraries/AP_Motors/AP_MotorsHeli_Single.cpp#L442).
   `H_YAW_TRIM` is only honoured for DDFP tails (`have_DDFP_tail()`).
3. **Collective→Yaw feed-forward (`H_COL2YAW`).** Injects
   `H_COL2YAW * |collective − coll_zero_thrust|^1.5` into the yaw command,
   approximating hover-power vs. weight (3/2-power rule). Lets the rate
   PID see a smaller residual error during collective changes.
4. **Thrust linearization (`H_DDFP_THST_EXPO`, `H_DDFP_SPIN_MIN`,
   `H_DDFP_SPIN_MAX`, `H_DDFP_BAT_*`).** A `Thrust_Linearization` object
   maps the linear "thrust demand" coming out of the PID into the
   non-linear motor PWM curve, then compensates for battery sag.
5. **Saturation feeds back into anti-windup.**
   `output_to_ddfp_tail()` sets `limit.yaw = true` when the throttle
   clips at 0 or 1, and that `limit.yaw` flag is exactly the
   `motor_limit` argument the yaw rate PID sees in `update_all` — so the
   integrator stops winding up against a saturated tail motor.
6. **Spool gating.** Below `THROTTLE_UNLIMITED` the tail motor is forced
   to 0. The `update_leaky_i` path in §3 (gated on
   `rotor_runup_complete()`) is what keeps the yaw I-term from drifting
   while the tail is producing zero thrust.

### Python: DDFP output stage

```python
class ThrustLinearization:
    """Stand-in for AP_Motors's Thrust_Linearization. The real one applies
    expo + battery voltage compensation. Here we just do the expo curve
    and clamp to [SPIN_MIN, SPIN_MAX]."""
    def __init__(self, spin_min=0.15, spin_max=0.95, expo=0.65):
        self.spin_min, self.spin_max, self.expo = spin_min, spin_max, expo

    def thrust_to_actuator(self, thrust_in):
        # thrust_in is signed PID output in [-1, 1]; only positive is usable.
        t = max(0.0, min(1.0, thrust_in))
        # Inverse of the DDFP_THST_EXPO model (matches AP_Motors approximately):
        if self.expo <= 0.0:
            actuator = t
        else:
            actuator = (math.sqrt(self.expo*self.expo + 4.0*self.expo*(1.0 - self.expo)*t)
                        - self.expo) / (2.0 * (1.0 - self.expo))
        return self.spin_min + actuator * (self.spin_max - self.spin_min)


def get_yaw_offset(collective, col_zero_thrust_pct,
                   col2yaw_scale, yaw_trim,
                   rotor_active, ddfp_tail):
    """Mirror of AP_MotorsHeli_Single::get_yaw_offset.

    rotor_active : False during autorotation or when at/below idle output.
    ddfp_tail    : True for TAIL_TYPE 3 or 4. Adds H_YAW_TRIM offset.
    """
    if not rotor_active:
        return 0.0
    offset = col2yaw_scale * (abs(collective - col_zero_thrust_pct) ** 1.5)
    if ddfp_tail:
        offset += yaw_trim
    return offset


def output_to_ddfp_tail(pid_yaw_signed_out, collective, col_zero_thrust_pct,
                        col2yaw_scale, yaw_trim,
                        thr_lin, tail_type_ccw,
                        spool_state, rotor_active):
    """Full output path for a DDFP tail. Returns (motor_throttle_0_1, limit_yaw).

    spool_state: one of "SHUT_DOWN","GROUND_IDLE","SPOOLING_UP",
                 "SPOOLING_DOWN","THROTTLE_UNLIMITED".
    """
    # Add steady-state torque feed-forward (computed by motors lib in C++).
    yaw_cmd = pid_yaw_signed_out + get_yaw_offset(
        collective, col_zero_thrust_pct, col2yaw_scale, yaw_trim,
        rotor_active, ddfp_tail=True,
    )
    # CCW airframe: invert the signed command before linearization.
    if tail_type_ccw:
        yaw_cmd = -yaw_cmd

    if spool_state in ("SHUT_DOWN", "GROUND_IDLE", "SPOOLING_DOWN"):
        return 0.0, False

    # Linearize and clamp; flag saturation so the rate PID can stop
    # winding up next iteration (this becomes motor_limit_yaw).
    throttle = thr_lin.thrust_to_actuator(yaw_cmd)
    limit_yaw = throttle <= thr_lin.spin_min or throttle >= thr_lin.spin_max
    throttle = max(thr_lin.spin_min, min(thr_lin.spin_max, throttle))
    return throttle, limit_yaw
```

### How DDFP changes day-to-day tuning

- `ATC_RAT_YAW_FF` is usually **lower** than on a servo tail because
  `H_COL2YAW` already supplies most of the steady-state feed-forward.
- `ATC_RAT_YAW_I` is typically **lower** as well; `H_YAW_TRIM` removes
  the static load that I would otherwise carry. Per the parameter
  description: *"this trim is used to compensate for the main rotor
  profile drag. If H_COL2YAW is not used, this value can be set to
  reduce the yaw I contribution to zero when in a steady hover."*
- The leaky-I behavior (§3) is more important on DDFP because the tail
  motor is forced to zero during ground idle, so any non-zero yaw
  I-term would kick the airframe the moment the tail spins up.

**Relevant parameters:**

| Param | Owner | Purpose |
|-------|-------|---------|
| `H_TAIL_TYPE` | `AP_MotorsHeli_Single` | Selects servo / extgyro / DDVP / DDFP CW/CCW. |
| `H_COL2YAW` | DDFP/DDVP/servo | Collective→yaw feed-forward, scaled by `|coll−coll0|^1.5`. |
| `H_YAW_TRIM` | DDFP only | Static yaw command offset to null hover I-term. |
| `H_DDFP_THST_EXPO` | DDFP | Thrust-curve exponent. |
| `H_DDFP_SPIN_MIN` / `H_DDFP_SPIN_MAX` | DDFP | Motor PWM limits. |
| `H_DDFP_BAT_IDX` / `H_DDFP_BAT_V_MIN` / `H_DDFP_BAT_V_MAX` | DDFP | Battery sag compensation. |
| `H_TAIL_SPEED` | DDVP only | Governor setpoint for variable-pitch DD tail. |

---

## 8. "Purest" yaw rate PID — just damp angular velocity

A useful thought experiment: strip the yaw loop down to *only* the term
that opposes body-frame angular velocity around the Z axis. No
feed-forward, no integrator carrying static trim, no collective coupling,
no slew/notch shaping, no piro coupling — just `output = -Kp * gyro_z`
(or with a stick target, `Kp * (target − gyro_z)`).

This isn't a flight-quality tune; it's the **isolation configuration**
you'd use to characterize the airframe's yaw damping, validate gyro/PID
plumbing, or build a textbook reference to compare more featureful tunes
against.

### What to zero, what to keep

| Parameter | Set to | Why |
|-----------|--------|-----|
| `ATC_RAT_YAW_P` | small positive (e.g. **0.10–0.18**) | The **only** active term. This is what you're isolating. |
| `ATC_RAT_YAW_I` | **0** | Removes the integrator. Without I, no hover trim builds up, no leaky-I dynamics, no anti-windup interaction. The loop becomes memoryless. |
| `ATC_RAT_YAW_D` | **0** | D in `AC_PID` operates on filtered error (`d/dt(error_f)`), so it adds dynamics that aren't "just rate proportional". Drop it for the purest form. (If you want a fast-acting rate damper add a tiny D, but understand it's no longer pure.) |
| `ATC_RAT_YAW_FF` | **0** | FF is `kff * filtered_target` — adds an open-loop term that does *not* depend on measured rate. Kills the "compensating for angular velocity" property. |
| `ATC_RAT_YAW_D_FF` | **0** | Same reason — derivative-of-target FF is open-loop. |
| `ATC_RAT_YAW_IMAX` | 0 | I is already 0; this just makes intent explicit. |
| `ATC_RAT_YAW_ILMI` | 0 | Same. |
| `ATC_RAT_YAW_FLTT` | high (e.g. **80–100 Hz**) or `0` | Don't filter the stick target — it should arrive unsmoothed so the loop's response to it is just `Kp * Δ`. |
| `ATC_RAT_YAW_FLTE` | moderate (**20–30 Hz**) | Keeps gyro noise out of the P term. Setting it too low introduces phase lag that masquerades as plant dynamics. |
| `ATC_RAT_YAW_FLTD` | irrelevant | D=0, so this filter never runs. |
| `ATC_RAT_YAW_SMAX` | **0** | Disables the dynamic `Dmod` scaler. With `Dmod ≠ 1` your effective P changes during oscillation, contaminating the experiment. |
| `ATC_RAT_YAW_PDMX` | **0** | Disables the P+D clamp. |
| `ATC_RAT_YAW_NTF` / `NEF` | **0** | No notch filters — they add poles you don't want in the model. |
| `ATC_PIRO_COMP` | **0** | This affects roll/pitch I-terms only, but turn it off so cross-axis I rotation can't masquerade as yaw behavior in tests. |
| `H_COL2YAW` | **0** | Removes the collective-driven yaw feed-forward. Otherwise yaw output depends on collective, not just gyro. |
| `H_YAW_TRIM` (DDFP only) | **0** | Removes the static offset. Pure P should sit at zero output when measured = target. |
| `ATC_ANG_YAW_P` | n/a (use ACRO) | Bypass the outer attitude loop entirely by flying ACRO. The rate target then comes from the stick, not from an angle error. |
| `ACRO_OPTIONS` bit 0 (`RATE_LOOP_ONLY`) | **set** | Routes through `input_rate_bf_roll_pitch_yaw_2()`, which skips attitude-target bookkeeping. Closest path the firmware provides to "raw rate PID". |
| `ACRO_BAL_*` / `ACRO_TRAINER` | **off / 0** | Disable the virtual-flybar leak so the rate target equals the stick exactly. |
| `H_TAIL_TYPE` | preferably **0 (servo)** | Variable-pitch tail is bidirectional and roughly linear, so PID output → torque is closest to "linear plant". DDFP introduces unidirectional thrust + thrust-curve expo + sat clamping, which all distort the experiment. If you must use DDFP, also set `H_DDFP_THST_EXPO=0` and avoid running near `H_DDFP_SPIN_MIN/MAX`. |
| Spool state | **THROTTLE_UNLIMITED**, rotor at speed | Otherwise `update_leaky_i` (which is dormant since I=0) and the DDFP zero-throttle gate change behavior. |

### What this leaves you with

The yaw rate loop collapses (after `update_all`) to:

```
error_f = LPF(target − measured, FLTE)
output  = clamp( Kp * error_f, ±1 )
```

If the stick is centered (`target = 0`) the output is purely
**−Kp · LPF(measured)** — a textbook angular-velocity damper. The motor
torque the controller demands is proportional (with sign reversal) to
the airframe's instantaneous yaw rate, exactly the behavior of a viscous
damper on the yaw axis.

Behavioral consequences worth knowing:

- **Static yaw error is permanent.** With I=0 there is nothing to null
  out steady main-rotor torque. The airframe will sit at a non-zero yaw
  *rate* equal to `(static_disturbance_torque) / (Kp · plant_gain + B)`.
  That's expected — that residual rate is the price of removing I.
- **No droop compensation across collective changes.** Lifting collective
  injects torque; without `H_COL2YAW` and without I, the rate PID will
  see a transient yaw rate and oppose it only proportionally. Yaw will
  visibly kick on collective inputs.
- **Step response is first-order-ish.** `Kp` directly sets the closed-loop
  damping. Doubling `Kp` halves the yaw-rate decay time constant against
  the airframe's inertia (subject to actuator bandwidth and `FLTE`).
- **Saturation is benign.** No I-term to wind up, so motor saturation
  causes only transient cropping; recovery is immediate when the
  disturbance ends.

### Python: pure yaw rate damper

```python
def make_pure_yaw_rate_damper(Kp=0.15, FLTE=25.0):
    """Yaw rate PID configured as a pure angular-velocity damper.

    Output = clamp(Kp * LPF(target - gyro_z, FLTE), ±1).
    """
    return HeliPID(
        P=Kp,
        I=0.0, D=0.0,             # no I, no D
        FF=0.0, D_FF=0.0,         # no feed-forward
        IMAX=0.0, ILMI=0.0,       # explicit
        FLTT=0.0,                 # don't filter the target
        FLTE=FLTE,                # only filter we keep
        FLTD=0.0,
        SMAX=0.0, PDMX=0.0,       # no slew limiter, no PD clamp
    )


def pure_yaw_step(pid, gyro_z, target_rate, dt):
    """Single iteration of the pure rate damper.
    Returns the yaw command in [-1, 1]. No FF, no leaky-I, no offsets."""
    out = pid.update_all(target_rate, gyro_z, dt, motor_limit=False)
    return max(-YAW_OUT_MAX, min(YAW_OUT_MAX, out))


if __name__ == "__main__":
    pid = make_pure_yaw_rate_damper(Kp=0.15, FLTE=25.0)
    dt = 1.0 / 400.0

    # Plant: scalar yaw inertia with viscous damping. Disturbance torque
    # simulates main-rotor reaction torque (which I would normally null).
    yaw_rate = math.radians(0.0)
    inertia  = 0.05         # kg·m²
    damping  = 0.02         # N·m / (rad/s)
    K_act    = 1.5          # N·m per unit command
    disturbance_torque = 0.05  # constant main-rotor torque (N·m)

    target = 0.0  # stick centered → pure damper

    for k in range(400):       # 1 s
        cmd   = pure_yaw_step(pid, yaw_rate, target, dt)
        torque = cmd * K_act + disturbance_torque - damping * yaw_rate
        yaw_rate += (torque / inertia) * dt
        if k % 50 == 0:
            print(f"t={k*dt:5.3f}s  cmd={cmd:+.3f}  "
                  f"yaw_rate={math.degrees(yaw_rate):+6.2f} deg/s")
    # Steady-state: yaw_rate settles at non-zero value because no I-term
    # nulls the disturbance — that is the expected pure-P behavior.
```

### When this is useful

- **Plant identification.** Drive a chirp or step on `target_rate` and
  log gyro vs. command. A pure P loop gives you an open-loop transfer
  estimate without I- or FF-distortion.
- **Sanity-checking a tune.** If a heli flies poorly, temporarily zeroing
  I/D/FF/COL2YAW/YAW_TRIM tells you whether the problem is in the
  fundamental P path or in the supporting machinery.
- **Hover-rig / ground tests** on a yaw-only gimbal where there is no
  aerodynamic torque to integrate.

### When *not* to use it for actual flight

Real heli yaw needs the I-term and `H_COL2YAW`/`H_YAW_TRIM` to handle
main-rotor torque without a permanent heading drift. A pure-P configuration
will fly, but the airframe will sit at a small constant yaw rate and the
yaw will visibly kick on every collective change — both unacceptable for
position-mode flight and most ACRO use.

---

## 8a. Pure yaw rate damper on a DDFP tail (no roll/pitch contribution)

Same goal as §8 — `output = Kp · (target − gyro_z)` and nothing else —
but now constrained to `H_TAIL_TYPE = 3 (CW)` or `4 (CCW)` and with the
explicit requirement that **no signal from roll, pitch, collective, or
any other axis** can reach the tail motor.

### What does and doesn't already cross-couple

By the structure of `AP_MotorsHeli_Single`, the DDFP tail motor is wired
to its own ESC channel. The swashplate (driven by the roll/pitch PIDs
and collective) is mechanically and electrically separate. Concretely:

- `_motors.set_roll(...)` and `_motors.set_pitch(...)` write into
  `_swashplate.output()`, which never touches the tail motor.
- `_motors.set_yaw(_servo4_out)` is the **only** channel that feeds
  `output_to_motors() → output_to_ddfp_tail()`.
- `ATC_PIRO_COMP` rotates the **roll/pitch** I-terms using yaw rate; it
  does *not* feed roll/pitch back into yaw. Off either way for clarity.
- The two firmware-level injections that *do* mix into yaw at the motor
  layer are:
  1. `get_yaw_offset()` — adds `H_COL2YAW · |coll−coll0|^1.5` (collective
     coupling) and `H_YAW_TRIM` (static offset).
  2. `_actuator_sysid.z` — only non-zero during system-ID injection.

So "purely yaw rate, nothing else" on DDFP requires zeroing both items
in (1), plus the §8 PID-side parameters, and not running SYSID.

### DDFP-specific issues that the §8 settings don't address

A DDFP fan can only push one way. The **sign** convention inside ArduPilot
is that `_servo4_out` is bipolar (the rate PID outputs ±1), but the
hardware can only realize positive thrust. Two consequences:

1. **The CCW variant sign-flips before the linearizer** (line 485 in
   `output_to_motors`). This is not a tuning choice — it's airframe
   geometry. Pick whichever of `3` or `4` makes the motor spin to oppose
   main-rotor reaction torque, then **don't change it during the test**.
2. **Negative PID output collapses to motor-off.**
   `thr_lin.thrust_to_actuator(yaw_cmd)` clamps at zero internally, then
   `output_to_ddfp_tail()` further clamps `[0, 1]` and sets
   `limit.yaw = true` at both ends. So the plant sees:

   ```
   actual_torque ≈  K(t) ·  yaw_cmd      for yaw_cmd > 0
                  =  0                    for yaw_cmd ≤ 0   (until rotor spins down)
   ```

   For a *pure* P loop centered around `yaw_cmd = 0`, this means the
   damper is **one-sided**: it can only resist yaw rates of one sign.
   Yaw rates in the opposite direction are unopposed (or only opposed by
   passive aerodynamic damping).

### How to make the test meaningful anyway

Three options, in order of fidelity:

**(i) Bias the operating point with `H_YAW_TRIM` only (no `H_COL2YAW`).**
This violates "no other inputs" in spirit but keeps it static and
predictable. With `H_YAW_TRIM = T0`:

```
yaw_motor_thrust ≈ thr_lin(  Kp · (target − gyro_z) + T0  )
```

The PID still operates purely on rate; `T0` just sets the linearization
midpoint. Pick `T0` to put `_servo4_out` near the middle of
`[H_DDFP_SPIN_MIN, H_DDFP_SPIN_MAX]` at hover so both signs of yaw error
produce real, opposite-sign thrust deltas. This is the closest you can
get to a symmetric bipolar damper with DDFP hardware.

**(ii) Run the rotor at idle / disarmed.** Main rotor torque is then
near zero, so the damper has no static disturbance to fight and the
one-sided saturation rarely matters. Useful for ground identification on
a yaw-axis test rig but not for actual flight.

**(iii) Pre-spin the tail at a fixed throttle.** Outside the PID,
externally hold the tail at, say, 50% via a custom mixer. ArduPilot
doesn't expose this directly without `H_YAW_TRIM`, so option (i) is the
practical realization.

### Recommended parameters (DDFP, pure-P yaw rate, isolated)

| Parameter | Value | Reason |
|-----------|-------|--------|
| `H_TAIL_TYPE` | `3` (CW) or `4` (CCW) | Match your airframe. |
| `H_COL2YAW` | **0** | No collective coupling. |
| `H_YAW_TRIM` | `T0` (≈ midpoint) | Sets the linearization bias so the damper is symmetric. Strictly speaking it's an "input", but it's a constant — see option (i). |
| `H_DDFP_THST_EXPO` | **0** | Linear thrust curve so PID output ≈ proportional motor torque. |
| `H_DDFP_SPIN_MIN` | as low as the ESC tolerates (e.g. **0.05**) | Wider linear range = more symmetric damping. |
| `H_DDFP_SPIN_MAX` | **1.0** | Same. |
| `H_DDFP_BAT_*` | leave default | Battery comp doesn't break the rate-only property. |
| `ATC_RAT_YAW_P` | **0.10–0.18** | Sole active gain. |
| `ATC_RAT_YAW_I/D/FF/D_FF` | **0** | As §8. |
| `ATC_RAT_YAW_IMAX/ILMI/SMAX/PDMX/NTF/NEF` | **0** | As §8. |
| `ATC_RAT_YAW_FLTT` | high or 0 | Don't filter target. |
| `ATC_RAT_YAW_FLTE` | **20–30 Hz** | Gyro noise rejection only. |
| `ATC_PIRO_COMP` | **0** | Defensive — affects roll/pitch I, not yaw. |
| `ATC_RAT_RLL_*` / `ATC_RAT_PIT_*` | unchanged | Irrelevant — they drive the swashplate, never the tail. |
| `ATC_ANG_*_P` | n/a | Use ACRO. |
| `ACRO_OPTIONS` bit 0 | **set** | `RATE_LOOP_ONLY` path. |
| Spool state during test | `THROTTLE_UNLIMITED` with rotor at speed | Otherwise tail is forced to 0. |

### Python: DDFP pure-P yaw damper, fully isolated

```python
def ddfp_pure_yaw_step(pid_yaw, gyro_z, target_rate, dt,
                       thr_lin, tail_type_ccw, yaw_trim_T0=0.5):
    """Pure-P yaw rate damper sent through the DDFP output stage.

    Roll/pitch/collective contribute nothing: H_COL2YAW=0 means the only
    non-PID term is the static H_YAW_TRIM bias `T0`, which centers the
    bipolar PID command inside the unipolar motor's linear range.

    Returns (motor_throttle_0_1, limit_yaw, raw_pid_out).
    """
    # 1) PID: only term that depends on a sensed signal is gyro_z.
    pid_out = pid_yaw.update_all(target_rate, gyro_z, dt,
                                 motor_limit=False)
    pid_out = max(-YAW_OUT_MAX, min(YAW_OUT_MAX, pid_out))

    # 2) Add ONLY the static trim bias. H_COL2YAW is 0, so no collective
    #    feedforward; no other axis can contribute by construction.
    yaw_cmd = pid_out + yaw_trim_T0

    # 3) CCW airframe sign flip (geometry, not a tuning choice).
    if tail_type_ccw:
        yaw_cmd = -yaw_cmd

    # 4) Thrust linearization (linear when H_DDFP_THST_EXPO=0) and clamp.
    throttle = thr_lin.thrust_to_actuator(yaw_cmd)
    limit_yaw = throttle <= thr_lin.spin_min or throttle >= thr_lin.spin_max
    throttle = max(thr_lin.spin_min, min(thr_lin.spin_max, throttle))
    return throttle, limit_yaw, pid_out


if __name__ == "__main__":
    # Pure-P PID (same as §8).
    pid = make_pure_yaw_rate_damper(Kp=0.15, FLTE=25.0)

    # Linear thrust curve (expo=0), wide spin range, for the cleanest test.
    thr_lin = ThrustLinearization(spin_min=0.05, spin_max=1.0, expo=0.0)

    dt          = 1.0 / 400.0
    yaw_rate    = 0.0
    inertia     = 0.05
    aero_damp   = 0.02
    K_thrust    = 1.5      # motor throttle [0..1] -> tail torque (N·m)
    disturb     = 0.05     # main-rotor reaction torque (N·m)

    yaw_trim_T0 = 0.5      # midpoint bias -> bipolar damping range

    for k in range(800):
        thr, sat, pid_raw = ddfp_pure_yaw_step(
            pid, yaw_rate, target_rate=0.0, dt=dt,
            thr_lin=thr_lin, tail_type_ccw=False,
            yaw_trim_T0=yaw_trim_T0,
        )
        # Plant: tail thrust opposes disturbance + own damping.
        torque = thr * K_thrust + disturb - aero_damp * yaw_rate
        yaw_rate += (torque / inertia) * dt
        if k % 100 == 0:
            print(f"t={k*dt:5.3f}s  pid={pid_raw:+.3f}  "
                  f"thr={thr:.3f}  sat={sat}  "
                  f"yaw_rate={math.degrees(yaw_rate):+6.2f} deg/s")
```

### Behavioral notes specific to DDFP pure-P

- **Asymmetric authority remains** even with `T0` bias. If
  `T0 = 0.5`, the damper has +0.5 of headroom in one yaw direction and
  −0.5 in the other (before clipping at `SPIN_MIN/MAX`). Disturbances
  that push the airframe *past* the SPIN_MIN-equivalent yaw rate become
  unopposed.
- **`limit.yaw` is set on every clip.** Even with I=0, this is the right
  behavior to log; it lets you spot when the test exited the linear
  regime.
- **`H_DDFP_BAT_*` battery compensation is still active.** It scales the
  thrust curve with measured voltage. If you need *strict* time-invariant
  plant gain for system ID, also constrain the test to a small
  voltage window or set `H_DDFP_BAT_IDX = 0` to disable comp.
- **Cross-axis isolation is structural.** With `H_COL2YAW = 0` and
  `_actuator_sysid.z = 0`, there is no firmware-internal path from
  roll, pitch, collective, throttle, or any other axis into
  `output_to_ddfp_tail()`. The tail motor's command is exactly
  `thr_lin( ±(Kp·(target−gyro_z) + T0) )`, clipped.

---

## Notable simplifications in this Python port

The Python translations above are accurate enough to reason about behavior
and to use as a hardware-in-the-loop reference, but they intentionally
simplify a few details from the C++:

- **`SMAX` slew limiter.** ArduPilot's `AP_PIDInfo::_slew_limiter` tracks
  recent oscillation amplitude and scales `(P+D)` by `Dmod ∈ [0.1, 1]`.
  The Python version implements `PDMX` clamping but skips the dynamic
  `Dmod` adjustment.
- **Notch filters (`NTF`, `NEF`).** Optional target/error notch filters
  are omitted; in flight they suppress airframe resonances.
- **`AP_FILTER_ENABLED` paths** (configurable filter manager) are
  omitted — we always use the inline `FLTT/FLTE/FLTD` low-passes.
- **`_actuator_sysid` / `_sysid_ang_vel_body`** system-ID injection paths
  are omitted; they only matter when running `SYSID_*` tests.
- **`set_throttle_out` / collective angle-boost** is a separate concern
  from the rate PIDs and is not covered here.
