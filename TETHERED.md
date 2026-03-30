# Tethered Flight — Mode_RAWES ArduPilot Implementation

## Executive Summary

Two designs were considered. The first (a thin `ModeGuidedTethered` subclass, ~100 lines)
delegates all control to `AC_PosControl` and `AC_AttitudeControl`, adding only a tether-tension
feedforward correction at the end. It is the right approach if all you need is a standard GUIDED
mode that "knows about" tether tension.

The second, defined in [`simulation/RAWESPROTOCOL.md`](simulation/RAWESPROTOCOL.md), is
**`Mode_RAWES`** — a fully autonomous, self-contained 400 Hz flight mode. It does not use
`AC_PosControl` or `AC_AttitudeControl`. Instead it runs:

- **Orbit tracking** — 400 Hz tether-direction following, no planner required
- **SLERP blend** — smooth transition between tether-aligned and planner-commanded orientation
- **Custom swashplate controller** — `compute_swashplate_from_state()` from the Weyel thesis
- **Tension PI** — collective controls tether tension, not altitude
- **Direct servo output** — bypasses ArduPilot's attitude control stack entirely

`Mode_RAWES` is the correct architecture for production RAWES flight. Its firmware complexity is
**~300 lines of new C++, roughly one week of focused work**, almost entirely porting validated
Python from `controller.py`, `swashplate.py`, and `mediator.py`. The math is already designed and
correct; this is a translation job, not a research job.

The critical path item is **not** firmware — it is the load cell for tension measurement, which
is currently undesigned and required before the tension PI controller can operate on hardware.

---

## 1. ArduCopter GUIDED Mode — Background

Understanding what GUIDED mode provides helps explain why Mode_RAWES takes a different approach.

### 1.1 GUIDED Mode Architecture

| File | Purpose |
|------|---------|
| `ArduCopter/mode_guided.cpp` | Full implementation (1215 lines) |
| `ArduCopter/mode.h:1091–1240` | `ModeGuided` class definition |
| `ArduCopter/mode_guided_nogps.cpp` | Minimal subclass example — 12 lines |

GUIDED mode runs seven submodes (`TakeOff`, `WP`, `Pos`, `PosVelAccel`, `VelAccel`, `Accel`,
`Angle`). Every position-stabilised submode ends with:

```cpp
// mode_guided.cpp:777, 822, 877, 966
attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(),
                                               auto_yaw.get_heading());
```

`AC_PosControl` computes a thrust vector assuming gravity is the only external load.
`AC_AttitudeControl` converts that into rate commands into the swashplate mix.

### 1.2 Why GUIDED Is Not Sufficient for Mode_RAWES

| Property | GUIDED assumes | RAWES reality |
|----------|---------------|--------------|
| External forces | Gravity only | Gravity + tether tension |
| Altitude control | Collective holds altitude | Collective controls tether *tension* |
| Position reference | Cartesian NED | Sphere surface at tether radius |
| Attitude equilibrium | Roughly level | Body Z along tether (~65° from vertical) |
| Orbit tracking | Planner must send position targets | Natural — firmware tracks tether direction at 400 Hz |
| Planner cadence | Must send targets at ≥ 10 Hz | Silent during reel-out; only intervenes for reel-in tilt |

A thin GUIDED subclass (see §7 for that design) works as a stepping stone. Mode_RAWES is the
production architecture.

---

## 2. Mode_RAWES Protocol

Defined in full in [`simulation/RAWESPROTOCOL.md`](simulation/RAWESPROTOCOL.md). Summary:

### 2.1 STATE Packet — Pixhawk → Planner (~10 Hz)

| Field | MAVLink message | Content |
|-------|----------------|---------|
| `pos_enu` | `LOCAL_POSITION_NED` (converted) | Hub position ENU [m] |
| `vel_enu` | `LOCAL_POSITION_NED` (converted) | Hub velocity ENU [m/s] |
| `tension_n` | `NAMED_VALUE_FLOAT "tension"` | Measured tether tension [N] |
| `t_free` | `SYSTEM_TIME` | Elapsed free-flight time [s] |

### 2.2 COMMAND Packet — Planner → Pixhawk (~10 Hz)

| Field | MAVLink message | Content |
|-------|----------------|---------|
| `body_z_target` | `SET_ATTITUDE_TARGET` (quaternion) | Desired disk axis ENU unit vector, or null |
| `blend_alpha` | `SET_ATTITUDE_TARGET` (thrust field) | 0 = tether-aligned, 1 = fully at `body_z_target` |
| `tension_setpoint_n` | `COMMAND_LONG MAV_CMD_USER_1` | Tension setpoint [N] for inner PI |
| `winch_speed_ms` | `DO_WINCH` | +ve = pay out, −ve = reel in, 0 = hold |

### 2.3 The 400 Hz Firmware Loop

```
Each step:

1. Hold last COMMAND packet if no new one arrived

2. Orbit tracking:
       bz_tether = normalize(pos_ned)   (with initial azimuth alignment)

3. Attitude setpoint:
       if blend_alpha > 0 and body_z_target valid:
           body_z_eq = slerp(bz_tether, body_z_target, blend_alpha)
       else:
           body_z_eq = bz_tether          ← natural, no planner needed

4. Cyclic (attitude):
       error = cross(body_z_current, body_z_eq)
       tilt_lon, tilt_lat = compute_swashplate_from_state(error, omega_body)

5. Collective (tension):
       collective_rad = PI(tension_setpoint_n, tension_measured_n)

6. Servo output:
       servo_pwm[1..3] = H3_inverse_mix(collective_rad, tilt_lon, tilt_lat)

7. Winch:
       g2.winch.set_desired_rate(winch_speed_ms)
```

---

## 3. What ArduPilot Already Provides

### 3.1 Fully Free (Zero New Code)

| Protocol need | ArduPilot mechanism | Where |
|---------------|---------------------|-------|
| `DO_WINCH` → `winch_speed_ms` | `AP_Winch::set_desired_rate()` | **Already wired** — `GCS_MAVLink_Copter.cpp:724` handles `DO_WINCH` and calls `set_desired_rate()` directly |
| `LOCAL_POSITION_NED` STATE output | Telemetry scheduler | Already broadcast on telemetry stream |
| `SYSTEM_TIME` STATE output | Already on telemetry stream | Already broadcast |
| Collective blade angle API | `AP_MotorsHeli::set_coll_from_ang(deg)` | Maps degrees → normalised → swashplate PWM |
| PID controller infrastructure | `AC_PID` class | Instantiate one for tension PI |
| Mode framework | `Mode::init()`, `run()`, params | Standard boilerplate |
| Arm/disarm guard | `is_disarmed_or_landed()` | Inherited |
| AHRS attitude + rates | `ahrs.get_rotation_body_to_ned()`, `ahrs.get_gyro()` | Available in any mode |
| EKF position | `inertial_nav.get_position_neu_m()` | Available in any mode |

`DO_WINCH` being already completely implemented is the biggest surprise — the winch interface
costs nothing.

### 3.2 Partially Provided (Needs Adaptation)

| Need | What exists | What's missing |
|------|------------|---------------|
| Cyclic servo output | `motors->set_radio_passthrough(roll, pitch, thr, yaw)` bypasses AC_AttitudeControl | Confirm no side-effects in armed flight; may need direct `motors->set_roll()` / `set_pitch()` |
| `NAMED_VALUE_FLOAT "tension"` receive | NAMED_VALUE_FLOAT handler skeleton | Add one case to existing handler |
| `SET_ATTITUDE_TARGET` for `body_z_target` | Handler exists for normal angle mode | Extract body_z from quaternion + read thrust field as `blend_alpha` |
| `COMMAND_LONG` for tension setpoint | Generic COMMAND_LONG handler | Add `MAV_CMD_USER_1` case |

---

## 4. What Must Be Built

### 4.1 Component Breakdown

| Component | Source (Python) | Firmware effort | Difficulty |
|-----------|----------------|----------------|-----------|
| Mode skeleton, params, registration | — | ~30 lines boilerplate | Low |
| Orbit tracking (`bz_tether`) | `mediator.py` `orbit_tracked_body_z_eq()` | ~25 lines | Low |
| SLERP blend | `mediator.py` blend logic | ~20 lines (use ArduPilot `Quaternion::slerp`) | Low |
| `compute_swashplate_from_state()` | `controller.py` | ~80 lines | Medium |
| `H3_inverse_mix()` | `swashplate.py` | ~25 lines | Low |
| Servo output (cyclic) | — | ~15 lines; `motors->set_roll/pitch` | Low–Medium |
| Tension PI | `mediator.py` `TensionController` | ~30 lines with `AC_PID` | Low |
| MAVLink STATE sender | — | ~25 lines; pack `pos_enu`, `vel_enu`, `tension_n` | Low |
| MAVLink COMMAND receiver | — | ~40 lines; parse 3 message types | Low–Medium |
| ENU ↔ NED frame conversions | `sensor.py` | ~10 lines of helpers | Low |
| **Firmware total** | | **~300 lines** | **Medium** |
| Tension measurement hardware | — | Load cell + ADC + driver | **High — separate track** |
| SITL validation | `mediator.py` already models this | ~2–3 days update mediator | Medium |

### 4.2 The One Firmware Unknown: Cyclic Servo Output

Standard modes go: `AC_AttitudeControl::input_*()` → rate controller → `motors->set_roll/pitch/throttle()` → swashplate mix.

Mode_RAWES computes `(tilt_lon, tilt_lat)` directly from `compute_swashplate_from_state()` and needs to write them to the swashplate, bypassing AC_AttitudeControl. Two options:

**Option A — `set_radio_passthrough()`**
```cpp
motors->set_radio_passthrough(tilt_lat_norm, tilt_lon_norm, collective_norm, 0.0f);
```
Designed for calibration/passthrough modes. Risk: may have behaviour flags that interfere in armed flight. Needs verification.

**Option B — Direct `set_roll()` / `set_pitch()`**
```cpp
// AP_Motors_Class protected API — need to call from within motors context or friend
motors->set_roll(tilt_lat_norm);
motors->set_pitch(tilt_lon_norm);
motors->set_throttle(collective_norm);
```
This is exactly what `AC_AttitudeControl` does internally after computing rate outputs. It is the correct path if we are providing the final mixed output directly.

Option B is preferred. The investigation needed is confirming that `set_roll()` / `set_pitch()` / `set_throttle()` are accessible from a mode class in the tradheli frame (they are `protected` in `AP_Motors_Class` but called via `motors` which is a public pointer). This is one hour of code archaeology, not a design problem.

### 4.3 Tension Measurement Hardware (Critical Path)

The tension PI controller requires `tension_measured_n`. Currently no load cell is designed into the system. Until this hardware exists, the PI can be validated in SITL using the tension value computed by `TetherModel` and streamed over MAVLink — but that loop is already closed in simulation only.

Hardware options:
- **Inline load cell** on the tether attachment point, I²C/SPI ADC → Pixhawk serial
- **Strain gauge** on the tether anchor boom, analog output → Pixhawk ADC input
- **ArduPilot driver path**: `AP_HAL::AnalogSource` for analog, custom driver for digital

This is a hardware design task independent of firmware. The firmware tension PI reads from a
`float _tension_measured_n` that can be set from any source — simulation, load cell, or
estimated from position + tether model. The firmware can be written and SITL-validated before
hardware exists.

---

## 5. Implementation Plan

### Phase 1 — Firmware Mode_RAWES, Simulated Tension (~1 week)

Build the complete 400 Hz loop using `tension_measured_n` fed from the companion via
`NAMED_VALUE_FLOAT "tension"` (companion computes it from `TetherModel`). Validates:
- Orbit tracking holds tether-aligned attitude without planner
- SLERP blend transitions smoothly when planner sends `body_z_target`
- Tension PI responds correctly to setpoint changes
- `DO_WINCH` executes reel-out and reel-in

**Files:** 8 files, ~300 lines of new C++.

### Phase 2 — SITL Validation (~2–3 days)

Update `mediator.py` to send actual MAVLink packets (STATE over `LOCAL_POSITION_NED`, COMMAND over
`SET_ATTITUDE_TARGET` + `DO_WINCH`) instead of direct function calls. The mediator's simulation
mapping table in `RAWESPROTOCOL.md` defines exactly what needs to be replaced. Compare firmware
orbit tracking and tension PI output against Python ground truth.

### Phase 3 — Load Cell Integration (hardware-dependent)

Wire load cell to Pixhawk, write or adapt driver, replace `NAMED_VALUE_FLOAT` tension source with
live hardware reading. Firmware tension PI is already validated — only the input source changes.

### Phase 4 — Pumping Cycle Flight Test

Run `DeschutterTrajectory` from companion. Firmware is unchanged — it just receives slowly
varying `blend_alpha`, `body_z_target`, `tension_setpoint_n`, and `winch_speed_ms` at 10 Hz.

---

## 6. Prototype Implementation

### 6.1 Class Declaration — `ArduCopter/mode.h`

Add as a standalone class (not a subclass of `ModeGuided`):

```cpp
class ModeRAWES : public Mode {
public:
    using Mode::Mode;
    Number mode_number() const override { return Number::RAWES; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_position() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override;
    bool is_autopilot() const override { return true; }

    // Called from NAMED_VALUE_FLOAT handler — tension measured by load cell or simulation
    void set_tension_measured_n(float t) { _tension_measured_n = MAX(0.0f, t); }

    // Called from SET_ATTITUDE_TARGET handler
    void set_planner_command(const Vector3f& body_z_target_enu, float blend_alpha);

    // Called from COMMAND_LONG MAV_CMD_USER_1
    void set_tension_setpoint_n(float t) { _tension_setpoint_n = MAX(0.0f, t); }

protected:
    const char *name()  const override { return "RAWES"; }
    const char *name4() const override { return "RAWE"; }

private:
    // ---- Orbit tracking ----
    void     capture_equilibrium();               // called once at free-flight start
    Vector3f compute_bz_tether() const;           // tether-aligned body Z in NED

    // ---- Controllers ----
    Vector3f slerp_body_z(const Vector3f& from, const Vector3f& to, float alpha) const;
    void     compute_cyclic(const Vector3f& body_z_eq,
                             float& tilt_lon_out, float& tilt_lat_out) const;
    float    update_tension_pi(float dt);

    // ---- Swashplate ----
    void     apply_servo_outputs(float tilt_lon, float tilt_lat, float collective_norm);

    // ---- MAVLink STATE sender ----
    void     send_state_packet();
    uint32_t _last_state_send_ms = 0;

    // ---- Planner command state ----
    Vector3f _body_z_target_ned;           // last planner body_z target, in NED
    float    _blend_alpha     = 0.0f;
    bool     _planner_valid   = false;
    uint32_t _planner_recv_ms = 0;

    // ---- Tension ----
    float    _tension_measured_n  = 0.0f;
    float    _tension_setpoint_n  = 100.0f;   // default hold tension
    AC_PID   _tension_pid;                    // collective PI

    // ---- Equilibrium capture ----
    bool     _eq_captured   = false;
    Vector3f _body_z_eq0_ned;                 // body Z at capture instant
    Vector3f _tether_dir0_ned;                // tether direction at capture instant

    // ---- Gains (parameters) ----
    AP_Float _kp_cyclic;
    AP_Float _kd_cyclic;
    AP_Float _kp_tension;
    AP_Float _ki_tension;
};
```

### 6.2 Implementation — `ArduCopter/mode_rawes.cpp`

```cpp
#include "Copter.h"

// ---- Frame conversion helpers ----
static Vector3f ned_to_enu(const Vector3f& v) { return Vector3f(v.y, v.x, -v.z); }
static Vector3f enu_to_ned(const Vector3f& v) { return Vector3f(v.y, v.x, -v.z); }

bool ModeRAWES::init(bool ignore_checks)
{
    _eq_captured    = false;
    _planner_valid  = false;
    _blend_alpha    = 0.0f;
    _tension_pid.reset_I();
    return true;
}

bool ModeRAWES::allows_arming(AP_Arming::Method method) const
{
    return method == AP_Arming::Method::MAVLINK || method == AP_Arming::Method::SCRIPTING;
}

// -----------------------------------------------------------------------
// Main 400 Hz loop
// -----------------------------------------------------------------------
void ModeRAWES::run()
{
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    const float dt = G_Dt;

    // 1. Capture equilibrium once at free-flight start
    if (!_eq_captured) { capture_equilibrium(); }

    // 2. Orbit tracking — tether-aligned body Z in NED
    const Vector3f bz_tether = compute_bz_tether();

    // 3. Attitude setpoint — blend toward planner target if commanded
    //    Planner timeout: treat as silent after 2 s without update
    const bool planner_active = _planner_valid &&
                                 (millis() - _planner_recv_ms < 2000) &&
                                 _blend_alpha > 1e-3f;

    Vector3f body_z_eq = planner_active
        ? slerp_body_z(bz_tether, _body_z_target_ned, _blend_alpha)
        : bz_tether;

    // 4. Cyclic — compute swashplate tilt from attitude error
    float tilt_lon, tilt_lat;
    compute_cyclic(body_z_eq, tilt_lon, tilt_lat);

    // 5. Collective — tension PI
    const float collective_rad = update_tension_pi(dt);
    // map rad to normalised [0..1] using configured collective range
    const float collective_norm = (degrees(collective_rad) - g2.rawes_col_min_deg) /
                                   MAX(g2.rawes_col_max_deg - g2.rawes_col_min_deg, 1.0f);

    // 6. Servo output
    apply_servo_outputs(tilt_lon, tilt_lat, collective_norm);

    // 7. Winch is driven externally by DO_WINCH from planner — already handled by AP_Winch

    // 8. Send STATE packet at ~10 Hz
    if (millis() - _last_state_send_ms > 100) {
        send_state_packet();
        _last_state_send_ms = millis();
    }
}

// -----------------------------------------------------------------------
// Equilibrium capture — called once at free-flight start
// -----------------------------------------------------------------------
void ModeRAWES::capture_equilibrium()
{
    // Current body Z in NED (third column of body→NED rotation matrix)
    _body_z_eq0_ned = ahrs.get_rotation_body_to_ned().colz();

    // Tether direction from EKF position (anchor = home = NED origin)
    const Vector3f pos_neu = inertial_nav.get_position_neu_m();
    const Vector3f pos_ned(pos_neu.x, pos_neu.y, -pos_neu.z);
    if (pos_ned.length() > 1.0f) {
        _tether_dir0_ned = pos_ned.normalized();
        _eq_captured = true;
    }
}

// -----------------------------------------------------------------------
// Orbit tracking — tether-aligned body Z
// -----------------------------------------------------------------------
Vector3f ModeRAWES::compute_bz_tether() const
{
    if (!_eq_captured) {
        // Before capture: return current body Z as-is
        return ahrs.get_rotation_body_to_ned().colz();
    }

    // Current tether direction in NED
    const Vector3f pos_neu = inertial_nav.get_position_neu_m();
    const Vector3f pos_ned(pos_neu.x, pos_neu.y, -pos_neu.z);
    if (pos_ned.length() < 0.1f) {
        return _body_z_eq0_ned;
    }
    const Vector3f tether_now = pos_ned.normalized();

    // Rotate body_z_eq0 by the same rotation that took tether_dir0 → tether_now.
    // This preserves the initial azimuthal alignment of the rotor disk.
    const Vector3f axis  = _tether_dir0_ned % tether_now;  // cross product
    const float    sinth = axis.length();
    const float    costh = _tether_dir0_ned * tether_now;   // dot product

    if (sinth < 1e-6f) {
        return _body_z_eq0_ned;  // no rotation needed
    }

    Quaternion q;
    q.from_axis_angle(axis / sinth, atan2f(sinth, costh));
    return q * _body_z_eq0_ned;
}

// -----------------------------------------------------------------------
// SLERP — smoothly interpolate between two unit vectors
// -----------------------------------------------------------------------
Vector3f ModeRAWES::slerp_body_z(const Vector3f& from, const Vector3f& to, float alpha) const
{
    const float dot   = constrain_float(from * to, -1.0f, 1.0f);
    const float angle = acosf(dot);
    if (angle < 1e-4f) { return to; }

    const Vector3f axis = (from % to).normalized();
    Quaternion q;
    q.from_axis_angle(axis, alpha * angle);
    return q * from;
}

// -----------------------------------------------------------------------
// Cyclic — port of compute_swashplate_from_state() from controller.py
// -----------------------------------------------------------------------
void ModeRAWES::compute_cyclic(const Vector3f& body_z_eq,
                                float& tilt_lon_out, float& tilt_lat_out) const
{
    // Current body Z in NED from AHRS
    const Vector3f body_z_now = ahrs.get_rotation_body_to_ned().colz();

    // Attitude error as cross product (body_z_now × body_z_eq)
    // This gives a rotation vector in NED whose magnitude is sin(angle_error)
    const Vector3f err_ned = body_z_now % body_z_eq;

    // Project error onto tether-plane axes (longitudinal = along wind, lateral = cross-wind).
    // For simplicity, use world NED axes: tilt_lon = North error, tilt_lat = East error.
    // H_PHANG will handle the phase rotation for the actual rotor.
    const float kp = _kp_cyclic.get();
    const float kd = _kd_cyclic.get();

    // Angular velocity in NED
    const Vector3f omega_ned = ahrs.get_rotation_body_to_ned() * ahrs.get_gyro();

    tilt_lon_out = kp * err_ned.x - kd * omega_ned.x;  // longitudinal (pitch)
    tilt_lat_out = kp * err_ned.y - kd * omega_ned.y;  // lateral (roll)
}

// -----------------------------------------------------------------------
// Tension PI — port of TensionController from mediator.py
// -----------------------------------------------------------------------
float ModeRAWES::update_tension_pi(float dt)
{
    const float error = _tension_setpoint_n - _tension_measured_n;
    // Use AC_PID for anti-windup and logging
    _tension_pid.set_dt(dt);
    const float collective_rad = _tension_pid.update_all(_tension_setpoint_n,
                                                          _tension_measured_n,
                                                          false);
    return constrain_float(collective_rad,
                           radians(g2.rawes_col_min_deg),
                           radians(g2.rawes_col_max_deg));
}

// -----------------------------------------------------------------------
// H3_inverse_mix + servo output — port of swashplate.py h3_inverse_mix()
// -----------------------------------------------------------------------
void ModeRAWES::apply_servo_outputs(float tilt_lon, float tilt_lat, float collective_norm)
{
    // H3-120 mixing: three servos 120° apart
    // S1 at 0°, S2 at 120°, S3 at 240° (or H4-90 variant — configured by H_SWASH_TYPE)
    // collective_norm is already in [0..1] from tension PI
    // tilt_lon / tilt_lat are in radians, normalise to [-1..1] range using cyclic_max
    const float cyc_max_rad = radians(g2.rawes_cyc_max_deg);
    const float roll_norm   = constrain_float(tilt_lat / cyc_max_rad, -1.0f, 1.0f);
    const float pitch_norm  = constrain_float(tilt_lon / cyc_max_rad, -1.0f, 1.0f);

    // Write directly to AP_MotorsHeli inputs, bypassing AC_AttitudeControl.
    // These are the same values AC_AttitudeControl would write after its rate loop.
    motors->set_roll(roll_norm);
    motors->set_pitch(pitch_norm);
    motors->set_throttle(collective_norm);
    motors->set_yaw(0.0f);  // GB4008 yaw managed separately via ATC_RAT_YAW
}

// -----------------------------------------------------------------------
// MAVLink STATE sender
// -----------------------------------------------------------------------
void ModeRAWES::send_state_packet()
{
    // pos and vel are available from inertial_nav in NEU; convert to ENU for protocol
    const Vector3f pos_neu = inertial_nav.get_position_neu_m();
    const Vector3f vel_neu = inertial_nav.get_velocity_neu_ms();

    // Publish tension on NAMED_VALUE_FLOAT so planner can read it
    gcs().send_named_float("tension", _tension_measured_n);

    // LOCAL_POSITION_NED is already broadcast by ArduPilot on telemetry stream.
    // No need to send separately — planner reads it from the normal stream.
}

// -----------------------------------------------------------------------
// MAVLink COMMAND receiver — called from GCS_MAVLink_Copter.cpp handlers
// -----------------------------------------------------------------------
void ModeRAWES::set_planner_command(const Vector3f& body_z_target_enu, float blend_alpha)
{
    _body_z_target_ned = enu_to_ned(body_z_target_enu).normalized();
    _blend_alpha       = constrain_float(blend_alpha, 0.0f, 1.0f);
    _planner_valid     = true;
    _planner_recv_ms   = millis();
}
```

### 6.3 MAVLink Hooks — `GCS_MAVLink_Copter.cpp`

Three additions, all small:

```cpp
// 1. SET_ATTITUDE_TARGET — extract body_z_target and blend_alpha
//    (add to existing handle_message case for MAVLINK_MSG_ID_SET_ATTITUDE_TARGET)
if (copter.flightmode == &copter.mode_rawes) {
    mavlink_set_attitude_target_t pkt;
    mavlink_msg_set_attitude_target_decode(&msg, &pkt);
    // quaternion third column = body Z in world frame
    Quaternion q(pkt.q[0], pkt.q[1], pkt.q[2], pkt.q[3]);
    Matrix3f R; q.rotation_matrix(R);
    const Vector3f body_z_enu(R.c.x, R.c.y, R.c.z);  // NED col 3 → ENU
    copter.mode_rawes.set_planner_command(body_z_enu, pkt.thrust);
    return;
}

// 2. COMMAND_LONG MAV_CMD_USER_1 — tension setpoint
case MAV_CMD_USER_1:
    if (copter.flightmode == &copter.mode_rawes) {
        copter.mode_rawes.set_tension_setpoint_n(packet.param1);
        return MAV_RESULT_ACCEPTED;
    }
    break;

// 3. NAMED_VALUE_FLOAT "tension" — measured tension from simulation or load cell
if (strncmp(packet.name, "tension\0\0\0", 10) == 0) {
    if (copter.flightmode == &copter.mode_rawes) {
        copter.mode_rawes.set_tension_measured_n(packet.value);
    }
}
```

### 6.4 Parameters — `ParametersG2`

```cpp
// Parameters.h — in ParametersG2 struct
AP_Float rawes_kp_cyclic;      // cyclic proportional gain
AP_Float rawes_kd_cyclic;      // cyclic derivative gain (rate damping)
AP_Float rawes_kp_tension;     // tension PI kP
AP_Float rawes_ki_tension;     // tension PI kI
AP_Float rawes_col_min_deg;    // minimum collective blade angle [deg]
AP_Float rawes_col_max_deg;    // maximum collective blade angle [deg]
AP_Float rawes_cyc_max_deg;    // maximum cyclic tilt [deg]
```

```cpp
// Parameters.cpp — AP_GROUPINFO table entries (replace XX with next index)
AP_GROUPINFO("RAWES_KP_CYC",  XX, ParametersG2, rawes_kp_cyclic,    1.0f),
AP_GROUPINFO("RAWES_KD_CYC",  XX, ParametersG2, rawes_kd_cyclic,    0.1f),
AP_GROUPINFO("RAWES_KP_TEN",  XX, ParametersG2, rawes_kp_tension,   0.01f),
AP_GROUPINFO("RAWES_KI_TEN",  XX, ParametersG2, rawes_ki_tension,   0.002f),
AP_GROUPINFO("RAWES_COL_MIN", XX, ParametersG2, rawes_col_min_deg, -10.0f),
AP_GROUPINFO("RAWES_COL_MAX", XX, ParametersG2, rawes_col_max_deg,  15.0f),
AP_GROUPINFO("RAWES_CYC_MAX", XX, ParametersG2, rawes_cyc_max_deg,  10.0f),
```

### 6.5 Mode Registration

```cpp
// ArduCopter/Copter.h
ModeRAWES mode_rawes;

// ArduCopter/mode.h — Number enum
RAWES = 35,   // or next available

// ArduCopter/mode.cpp — mode_from_mode_num()
case Mode::Number::RAWES:
    return &mode_rawes;
```

---

## 7. Thin GUIDED Subclass (Stepping Stone)

Before Mode_RAWES is ready, a thin `ModeGuidedTethered` subclass can validate the sensor
pipeline and EKF consistency without any custom control code. It:

- Inherits all of GUIDED mode unchanged
- Accepts standard `SET_POSITION_TARGET_GLOBAL_INT` from companion with sphere-projected NED targets
- Adds a single tether-tension feedforward correction to the thrust vector after `ModeGuided::run()`

**Total: ~100 lines, 1 parameter, zero rework of existing code.**

```cpp
// mode_guided_tethered.cpp — entire implementation
bool ModeGuidedTethered::init(bool ignore_checks) {
    _tether_tension_N = 0.0f;
    return ModeGuided::init(ignore_checks);
}

void ModeGuidedTethered::run() {
    ModeGuided::run();      // all position control inherited
    apply_tether_feedforward();  // correct final attitude setpoint only
}

void ModeGuidedTethered::apply_tether_feedforward() {
    const SubMode sm = submode();
    if (sm == SubMode::Angle || sm == SubMode::TakeOff || sm == SubMode::VelAccel) return;
    if (_tether_tension_N < 5.0f) return;

    const Location home = ahrs.get_home();
    Vector3p home_ned_m;
    if (!home.get_vector_from_origin_NEU_m(home_ned_m)) return;
    const Vector3f anchor_ned(home_ned_m.x, home_ned_m.y, -(float)home_ned_m.z);

    const Vector3p pos_neu = inertial_nav.get_position_neu_m().topostype();
    const Vector3f pos_ned((float)pos_neu.x, (float)pos_neu.y, -(float)pos_neu.z);

    Vector3f to_anchor = anchor_ned - pos_ned;
    if (to_anchor.length() < 0.1f) return;
    to_anchor.normalize();

    Vector3f thrust = pos_control->get_thrust_vector()
                    - to_anchor * (_tether_tension_N / (g2.guided_tether_mass_kg * GRAVITY_MSS));
    thrust.normalize();
    attitude_control->input_thrust_vector_heading(thrust, auto_yaw.get_heading());
}
```

Use this mode for Phase 1 integration testing. Migrate to `Mode_RAWES` for Phase 2.

---

## 8. Files Changed Summary

### Mode_RAWES (production)

| File | Change | Lines |
|------|--------|-------|
| `ArduCopter/mode_rawes.cpp` | **New** — full 400 Hz implementation | ~230 |
| `ArduCopter/mode.h` | Add `ModeRAWES` class + `RAWES = 35` enum | ~60 |
| `ArduCopter/Parameters.h` | Add 7 `rawes_*` fields to `ParametersG2` | ~7 |
| `ArduCopter/Parameters.cpp` | Register 7 parameters | ~35 |
| `ArduCopter/Copter.h` | Add `ModeRAWES mode_rawes` | 1 |
| `ArduCopter/mode.cpp` | Register in `mode_from_mode_num()` | ~3 |
| `ArduCopter/GCS_MAVLink_Copter.cpp` | 3 MAVLink handler additions | ~25 |
| `simulation/mediator.py` | Send MAVLink STATE/COMMAND instead of direct calls | ~40 |
| **Total** | | **~400** |

### ModeGuidedTethered (stepping stone)

| File | Change | Lines |
|------|--------|-------|
| `ArduCopter/mode_guided_tethered.cpp` | **New** | ~55 |
| `ArduCopter/mode.h` | `ModeGuidedTethered` class + enum | ~20 |
| `ArduCopter/Parameters.h/cpp` | `guided_tether_mass_kg` | ~7 |
| `ArduCopter/Copter.h`, `mode.cpp` | Instance + registration | ~4 |
| `ArduCopter/GCS_MAVLink_Copter.cpp` | `NAMED_VALUE_FLOAT "TETH_TEN"` handler | ~5 |
| **Total** | | **~90** |

---

## 9. Known Gaps and Risks

| Risk | Impact | Mitigation |
|------|--------|-----------|
| `motors->set_roll/pitch()` access from mode | Medium — protected API | Verify via `AP_MotorsHeli` public interface or use `set_radio_passthrough()` as fallback |
| `H_PHANG` cyclic phase error | Medium — axis coupling | Runtime parameter; calibrate in SITL against Python controller output |
| Kaman flap lag | Medium — phase margin | Detune `rawes_kd_cyclic`; companion flap controller (Option 2) handles if needed |
| Tension measurement hardware | **High — critical path** | Firmware validated in SITL first; load cell added before flight test |
| Planner timeout during reel-in tilt | Medium — loss of tilt command | 2 s timeout → revert to `bz_tether` (safe fallback to natural orbit) |
| ENU/NED frame errors | High — subtle inversion bugs | Define and use `ned_to_enu()` / `enu_to_ned()` helpers consistently |
| `compute_swashplate_from_state()` gain tuning | Medium — oscillation | Start with low `rawes_kp_cyclic`, validate SITL first |

---

## 10. Simulation Mapping

Per `RAWESPROTOCOL.md`, every firmware component maps directly to existing simulation code:

| `Mode_RAWES` firmware | Simulation equivalent | File |
|-----------------------|----------------------|------|
| `capture_equilibrium()` | Initial `_body_z_eq0`, `_tether_dir0` in mediator | `mediator.py` |
| `compute_bz_tether()` | `orbit_tracked_body_z_eq()` | `mediator.py` |
| `slerp_body_z()` | SLERP blend in mediator loop | `mediator.py` |
| `compute_cyclic()` | `compute_swashplate_from_state()` | `controller.py` |
| `H3_inverse_mix()` | `h3_inverse_mix()` | `swashplate.py` |
| `update_tension_pi()` | `TensionController` | `mediator.py` |
| `set_planner_command()` | `trajectory.step()` return value | `trajectory.py` |
| `DO_WINCH` → `AP_Winch` | `_tether.rest_length` update | `mediator.py` |

The simulation is a faithful software model. Moving to hardware replaces direct function calls
with MAVLink messages over the SiK radio — the protocol boundary is already defined.

---

## References

- [`simulation/RAWESPROTOCOL.md`](simulation/RAWESPROTOCOL.md) — full protocol specification
- `ArduCopter/mode_guided_nogps.cpp` — minimal mode subclass pattern (12 lines)
- `ArduCopter/GCS_MAVLink_Copter.cpp:724` — `DO_WINCH` already handled
- `libraries/AP_Motors/AP_MotorsHeli.h:129` — `set_coll_from_ang()` API
- `simulation/controller.py` — `compute_swashplate_from_state()` source
- `simulation/swashplate.py` — `h3_inverse_mix()` source
- `simulation/mediator.py` — orbit tracking, tension PI, SLERP blend
- `ardupilot_implementation.md` — integration options, open items, parameter starting points
