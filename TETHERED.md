# Tethered Flight — Mode_RAWES ArduPilot Implementation

## Executive Summary

`Mode_RAWES` is a custom 400 Hz flight mode for the Pixhawk 6C. Its structure is almost identical
to `ModeAcro_Heli` (the traditional helicopter ACRO mode), with three substitutions:

| ACRO_Heli | Mode_RAWES |
|-----------|-----------|
| Pilot RC sticks → body-frame rate commands | Body_z error (orbit-tracked vs. current) → body-frame rate commands |
| Pilot throttle stick → collective | Tension PI output → collective |
| — | Orbit tracking + rate-limited slerp (the "autopilot pilot" layer) |

Everything else — `AC_AttitudeControl` rate PIDs, swashplate mixing, spool state, counter-torque
yaw control, arming checks — is identical to ACRO_Heli and already exists.

**Firmware complexity: ~150 lines of new C++, mostly a port of validated Python.**
The `H3_inverse_mix()` and direct motor write questions from earlier analysis are eliminated —
`AC_AttitudeControl` already owns the swashplate path, just as it does in ACRO_Heli.

The winch is a **ground-station node** commanded directly by the trajectory planner over a local
interface. The Pixhawk is never involved in winch control.

---

## 1. ArduCopter ACRO_Heli — The Template

`ModeAcro_Heli::run()` is the reference implementation:

```cpp
// ArduCopter/mode_acro_heli.cpp — entire run() loop
void ModeAcro_Heli::run()
{
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // spool state guards (SHUT_DOWN, GROUND_IDLE) ...

    // 1. Rate targets from pilot sticks
    get_pilot_desired_rates_rads(target_roll, target_pitch, target_yaw);
    virtual_flybar(target_roll, target_pitch, target_yaw, ...);  // optional leak

    // 2. Feed to AC_AttitudeControl rate loop → runs rate PIDs → swashplate mix
    attitude_control->input_rate_bf_roll_pitch_yaw_rads(target_roll, target_pitch, target_yaw);

    // 3. Collective from pilot throttle
    attitude_control->set_throttle_out(pilot_throttle_scaled, false, g.throttle_filt);
}
```

`Mode_RAWES` makes two substitutions to this pattern:

```cpp
// Step 1 replacement: compute body-frame rate target from orbit-tracking error
Vector3f rate_bf = compute_body_rate_from_body_z_error(body_z_eq, body_z_current);
attitude_control->input_rate_bf_roll_pitch_yaw_rads(rate_bf.x, rate_bf.y, 0.0f);

// Step 3 replacement: collective from tension PI, not pilot stick
float collective_norm = update_tension_pi(dt);
attitude_control->set_throttle_out(collective_norm, false, g.throttle_filt);
```

`AC_AttitudeControl` then runs its rate PIDs and writes to `AP_MotorsHeli` exactly as in
ACRO_Heli — no changes to the motor layer needed.

---

## 2. Protocol Summary

Defined in full in [`simulation/RAWESPROTOCOL.md`](simulation/RAWESPROTOCOL.md).

### 2.1 STATE Packet — Pixhawk → Planner (~10 Hz)

| Field | MAVLink message | Content |
|-------|----------------|---------|
| `pos_enu` | `LOCAL_POSITION_NED` (converted) | Hub position ENU [m] |
| `vel_enu` | `LOCAL_POSITION_NED` (converted) | Hub velocity ENU [m/s] |
| `tension_n` | `NAMED_VALUE_FLOAT "tension"` | Measured tether tension [N] |
| `omega_spin` | `NAMED_VALUE_FLOAT "omega_spin"` | Rotor spin rate [rad/s] |
| `t_free` | `SYSTEM_TIME` | Elapsed free-flight time [s] |

`LOCAL_POSITION_NED` is already broadcast on the telemetry stream — no new code needed.

### 2.2 COMMAND Packet — Planner → Pixhawk (~10 Hz)

Exactly one MAVLink message: `SET_ATTITUDE_TARGET`.

| `SET_ATTITUDE_TARGET` field | Content |
|----------------------------|---------|
| `quaternion` | Desired disk orientation ENU. Identity `[1,0,0,0]` = natural tether-aligned orbit (planner silent). Non-identity: `body_z_target = quat_apply(q, [0,0,1])`. |
| `thrust` | Normalised tension setpoint [0..1]. Firmware converts: `tension_setpoint_n = thrust × tension_max_n`. |

The planner never sends blend factors, ramp timing, or winch commands to the Pixhawk.
Smooth body_z transitions are owned entirely by firmware's rate-limited slerp.

### 2.3 The 400 Hz Firmware Loop

```
Each step:

1. Hold last COMMAND packet if no new one arrived

2. Orbit tracking:
       Rotate initial equilibrium body_z by azimuthal change since free-flight start
       → body_z_tether (current tether-aligned target)

3. Attitude setpoint:
       if attitude_q == identity:
           body_z_eq = body_z_tether              ← natural orbit, planner silent
       else:
           body_z_target = quat_apply(attitude_q, [0,0,1])
           body_z_eq = slerp_rate_limited(body_z_eq_prev, body_z_target,
                                          body_z_slew_rate_rad_s, dt)

4. Rate command (cyclic):
       error = cross(body_z_current, body_z_eq)
       rate_lon, rate_lat = kp_cyclic × error components
       attitude_control->input_rate_bf_roll_pitch_yaw_rads(rate_lat, rate_lon, 0)
       // AC_AttitudeControl rate PIDs + swashplate mix handle the rest

5. Collective (tension):
       tension_setpoint_n = thrust × tension_max_n
       collective_norm = PI(tension_setpoint_n, tension_measured_n)
       attitude_control->set_throttle_out(collective_norm, false, throttle_filt)

6. Counter-torque yaw:
       Managed by ATC_RAT_YAW — no new code needed
```

---

## 3. What ArduPilot Already Provides

### 3.1 Free (Zero New Code)

| Mode_RAWES need | ArduPilot mechanism |
|----------------|---------------------|
| Rate PIDs + swashplate H3/H4 mix | `AC_AttitudeControl::input_rate_bf_roll_pitch_yaw_rads()` — used by ACRO_Heli |
| Collective → PWM | `AC_AttitudeControl::set_throttle_out()` — used by ACRO_Heli |
| Counter-torque yaw (GB4008 motor) | `ATC_RAT_YAW` PID — same as any tradheli yaw axis |
| Spool state guards | ACRO_Heli pattern: `THROTTLE_UNLIMITED`, `SHUT_DOWN`, `GROUND_IDLE` |
| `LOCAL_POSITION_NED` STATE output | Already on telemetry stream |
| `SYSTEM_TIME` STATE output | Already on telemetry stream |
| PID infrastructure for tension | `AC_PID` class |
| AHRS attitude + rates | `ahrs.get_rotation_body_to_ned()`, `ahrs.get_gyro()` |
| EKF position | `inertial_nav.get_position_neu_m()` |
| Mode framework, arming | `Mode::init()`, `is_disarmed_or_landed()`, boilerplate |

### 3.2 Small Additions Needed

| Need | What to add |
|------|-------------|
| `SET_ATTITUDE_TARGET` receive (RAWES branch) | Add guard: if `flightmode == mode_rawes`, parse quaternion + thrust, call `mode_rawes.set_command()` |
| `NAMED_VALUE_FLOAT "tension"` receive | One case in existing handler |
| `NAMED_VALUE_FLOAT "tension"` + `"omega_spin"` send | ~5 lines each in STATE sender |

---

## 4. What Must Be Built

| Component | Python source | Firmware lines | Notes |
|-----------|--------------|---------------|-------|
| Mode skeleton + registration | — | ~30 | Boilerplate |
| Orbit tracking (`compute_bz_tether`) | `controller.py` `orbit_tracked_body_z_eq()` | ~25 | Pure geometry |
| Rate-limited slerp | `mediator.py` blend loop | ~20 | Replaces `blend_alpha`; firmware owns timing |
| Cyclic rate command | `controller.py` `compute_swashplate_from_state()` | ~15 | P gain × body_z error → rate target; AC_AttitudeControl does the rest |
| Tension PI | `controller.py` `TensionController` | ~20 | `AC_PID`, one instance |
| MAVLink COMMAND receiver | — | ~20 | Parse `SET_ATTITUDE_TARGET` quaternion + thrust |
| STATE sender | — | ~20 | `tension` + `omega_spin` NAMED_VALUE_FLOAT |
| ENU ↔ NED helpers | `sensor.py` | ~5 | Two inline functions |
| **Total** | | **~155 lines** | |

### 4.1 Orbit Tracking

Port of `orbit_tracked_body_z_eq()` from `controller.py`. Captures initial body_z and tether
direction once at free-flight start, then rotates body_z by the azimuthal change of the tether
direction as the hub orbits:

```cpp
// capture once
_body_z_eq0_ned = ahrs.get_rotation_body_to_ned().colz();
_tether_dir0_ned = pos_ned.normalized();

// each 400 Hz step
Vector3f tether_now = inertial_nav.get_position_neu_m();
tether_now.z = -tether_now.z;         // NEU → NED
tether_now.normalize();
Vector3f axis = _tether_dir0_ned % tether_now;
float angle   = atan2f(axis.length(), _tether_dir0_ned * tether_now);
Quaternion q; q.from_axis_angle(axis.normalized(), angle);
Vector3f bz_tether = q * _body_z_eq0_ned;
```

### 4.2 Rate-Limited Slerp

When the planner sends a non-identity quaternion, body_z slews toward the target at
`body_z_slew_rate_rad_s` (configured in `config.py`, default 0.12 rad/s). The planner
never manages ramp timing — it just sends the target; firmware handles the transition:

```cpp
float angle_remaining = acosf(constrain_float(_body_z_eq * body_z_target, -1.0f, 1.0f));
float step = MIN(body_z_slew_rate_rad_s * dt, angle_remaining);
if (angle_remaining > 1e-4f) {
    Vector3f axis = (_body_z_eq % body_z_target).normalized();
    Quaternion q; q.from_axis_angle(axis, step);
    _body_z_eq = q * _body_z_eq;
}
```

### 4.3 Cyclic Rate Command

Body_z error → body-frame rate command. `AC_AttitudeControl` runs its rate PIDs and the D-term
provides angular velocity damping — no separate derivative gain needed in Mode_RAWES:

```cpp
// body_z_current = third column of body→NED rotation matrix
Vector3f body_z_now = ahrs.get_rotation_body_to_ned().colz();

// error in NED world frame; project to body frame for rate command
Vector3f err_ned = body_z_now % _body_z_eq;                // cross product
Vector3f err_body = ahrs.get_rotation_body_to_ned().transposed() * err_ned;

float rate_lon = _kp_cyclic * err_body.y;  // pitch axis
float rate_lat = _kp_cyclic * err_body.x;  // roll axis

attitude_control->input_rate_bf_roll_pitch_yaw_rads(rate_lat, rate_lon, 0.0f);
```

### 4.4 Tension PI

```cpp
// init
_tension_pid.set_kP(_kp_tension);
_tension_pid.set_kI(_ki_tension);
_tension_pid.set_imax(radians(RAWES_COL_MAX_DEG));

// each step — AC_PID handles anti-windup and logging
float setpoint = _thrust_cmd * _tension_max_n;
float collective_rad = _tension_pid.update_all(setpoint, _tension_measured_n, false);
float collective_norm = (degrees(collective_rad) - RAWES_COL_MIN_DEG)
                       / MAX(RAWES_COL_MAX_DEG - RAWES_COL_MIN_DEG, 1.0f);

attitude_control->set_throttle_out(constrain_float(collective_norm, 0.0f, 1.0f),
                                    false, g.throttle_filt);
```

---

## 5. Implementation

### 5.1 Class Declaration — `ArduCopter/mode.h`

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

    // Called from SET_ATTITUDE_TARGET MAVLink handler
    void set_command(const Quaternion& attitude_q_enu, float thrust_norm);

    // Called from NAMED_VALUE_FLOAT "tension" handler
    void set_tension_measured_n(float t) { _tension_measured_n = MAX(0.0f, t); }

protected:
    const char *name()  const override { return "RAWES"; }
    const char *name4() const override { return "RAWE"; }

private:
    void capture_equilibrium();
    Vector3f compute_bz_tether() const;
    void update_body_z_eq(float dt);
    void send_state();

    // Equilibrium capture
    bool     _eq_captured     = false;
    Vector3f _body_z_eq0_ned;
    Vector3f _tether_dir0_ned;

    // Current attitude setpoint (rate-limited slerp target)
    Vector3f _body_z_eq_ned;

    // Planner command state
    Vector3f _body_z_target_ned;
    float    _thrust_cmd       = 0.5f;   // normalised [0..1]
    bool     _planner_active   = false;
    uint32_t _planner_recv_ms  = 0;

    // Tension
    float    _tension_measured_n = 0.0f;
    AC_PID   _tension_pid;

    // STATE telemetry
    uint32_t _last_state_ms = 0;

    // Parameters
    AP_Float _kp_cyclic;
    AP_Float _kp_tension;
    AP_Float _ki_tension;
    AP_Float _tension_max_n;
    AP_Float _body_z_slew_rate_rads;
    AP_Float _col_min_deg;
    AP_Float _col_max_deg;
};
```

### 5.2 Implementation — `ArduCopter/mode_rawes.cpp`

```cpp
#include "Copter.h"

// ENU ↔ NED: swap x/y, negate z
static Vector3f enu_to_ned(const Vector3f& v) { return {v.y, v.x, -v.z}; }

bool ModeRAWES::init(bool ignore_checks)
{
    _eq_captured   = false;
    _planner_active = false;
    _thrust_cmd    = 0.5f;
    _tension_pid.reset_I();
    return true;
}

bool ModeRAWES::allows_arming(AP_Arming::Method method) const
{
    return method == AP_Arming::Method::MAVLINK || method == AP_Arming::Method::SCRIPTING;
}

void ModeRAWES::run()
{
    // ---- Spool state — identical to ModeAcro_Heli ----
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        attitude_control->reset_target_and_rate(false);
        attitude_control->reset_rate_controller_I_terms();
        return;
    case AP_Motors::SpoolState::GROUND_IDLE:
        attitude_control->reset_target_and_rate(false);
        attitude_control->reset_rate_controller_I_terms_smoothly();
        return;
    default:
        break;
    }

    const float dt = G_Dt;

    // ---- 1. Capture equilibrium once ----
    if (!_eq_captured) { capture_equilibrium(); }

    // ---- 2. Orbit tracking ----
    const Vector3f bz_tether = compute_bz_tether();

    // ---- 3. Rate-limited slerp toward planner target (or hold tether-aligned) ----
    // Planner timeout: 2 s without update → revert to identity (tether-aligned)
    const bool planner_live = _planner_active &&
                               (millis() - _planner_recv_ms < 2000);
    if (!planner_live) {
        _body_z_eq_ned = bz_tether;   // snap back to natural orbit
    } else {
        // Slerp _body_z_eq toward _body_z_target at slew rate limit
        const float rate   = _body_z_slew_rate_rads.get();
        const float dot    = constrain_float(_body_z_eq_ned * _body_z_target_ned, -1.0f, 1.0f);
        const float remain = acosf(dot);
        const float step   = MIN(rate * dt, remain);
        if (remain > 1e-4f) {
            const Vector3f axis = (_body_z_eq_ned % _body_z_target_ned).normalized();
            Quaternion q; q.from_axis_angle(axis, step);
            _body_z_eq_ned = q * _body_z_eq_ned;
        }
    }

    // ---- 4. Cyclic — body_z error → body-frame rate commands ----
    const Vector3f body_z_now = ahrs.get_rotation_body_to_ned().colz();
    const Vector3f err_ned    = body_z_now % _body_z_eq_ned;
    const Vector3f err_body   = ahrs.get_rotation_body_to_ned().transposed() * err_ned;

    const float kp = _kp_cyclic.get();
    // AC_AttitudeControl rate PIDs run D-term → angular velocity damping free
    attitude_control->input_rate_bf_roll_pitch_yaw_rads(kp * err_body.x,
                                                         kp * err_body.y,
                                                         0.0f);

    // ---- 5. Collective — tension PI ----
    _tension_pid.set_dt(dt);
    const float setpoint_n   = _thrust_cmd * _tension_max_n.get();
    const float collective_rad = _tension_pid.update_all(setpoint_n,
                                                          _tension_measured_n, false);
    const float collective_norm = constrain_float(
        (degrees(collective_rad) - _col_min_deg.get()) /
         MAX(_col_max_deg.get() - _col_min_deg.get(), 1.0f),
        0.0f, 1.0f);

    // set_throttle_out path is identical to ModeAcro_Heli
    attitude_control->set_throttle_out(collective_norm, false, g.throttle_filt);

    // ---- 6. STATE telemetry at ~10 Hz ----
    if (millis() - _last_state_ms >= 100) {
        send_state();
        _last_state_ms = millis();
    }
}

// -----------------------------------------------------------------------

void ModeRAWES::capture_equilibrium()
{
    const Vector3f pos_neu = inertial_nav.get_position_neu_m();
    const Vector3f pos_ned(pos_neu.x, pos_neu.y, -pos_neu.z);
    if (pos_ned.length() < 1.0f) return;   // wait for valid position

    _body_z_eq0_ned  = ahrs.get_rotation_body_to_ned().colz();
    _tether_dir0_ned = pos_ned.normalized();
    _body_z_eq_ned   = _body_z_eq0_ned;
    _eq_captured     = true;
}

Vector3f ModeRAWES::compute_bz_tether() const
{
    if (!_eq_captured) return ahrs.get_rotation_body_to_ned().colz();

    const Vector3f pos_neu = inertial_nav.get_position_neu_m();
    const Vector3f pos_ned(pos_neu.x, pos_neu.y, -pos_neu.z);
    if (pos_ned.length() < 0.1f) return _body_z_eq0_ned;

    const Vector3f tether_now = pos_ned.normalized();
    const Vector3f axis       = _tether_dir0_ned % tether_now;
    const float    sinth      = axis.length();
    const float    costh      = _tether_dir0_ned * tether_now;
    if (sinth < 1e-6f) return _body_z_eq0_ned;

    Quaternion q; q.from_axis_angle(axis / sinth, atan2f(sinth, costh));
    return q * _body_z_eq0_ned;
}

void ModeRAWES::set_command(const Quaternion& attitude_q_enu, float thrust_norm)
{
    // Extract body_z from quaternion (apply to ENU [0,0,1])
    Vector3f body_z_enu;
    attitude_q_enu.earth_to_body(Vector3f(0,0,1), body_z_enu);
    // body_z_enu is in ENU body frame; we want world-frame body_z in NED
    // attitude_q encodes world→body; we want body Z column in world frame
    Matrix3f R; const_cast<Quaternion&>(attitude_q_enu).rotation_matrix(R);
    const Vector3f body_z_enu_world(R.c.x, R.c.y, R.c.z);  // third col = body Z in world

    const bool is_identity = attitude_q_enu.is_unit() &&
                              fabsf(attitude_q_enu.q1 - 1.0f) < 1e-3f;
    if (is_identity) {
        _planner_active = false;   // planner silent → natural orbit
    } else {
        _body_z_target_ned = enu_to_ned(body_z_enu_world).normalized();
        _planner_active    = true;
    }
    _thrust_cmd       = constrain_float(thrust_norm, 0.0f, 1.0f);
    _planner_recv_ms  = millis();
}

void ModeRAWES::send_state()
{
    // tension and omega_spin sent as NAMED_VALUE_FLOAT
    gcs().send_named_float("tension",   _tension_measured_n);
    // omega_spin measured by Hall-effect sensor on rotor hub — source TBD
    // gcs().send_named_float("omega_spin", _omega_spin_measured);

    // pos_enu and vel_enu are already in LOCAL_POSITION_NED on the telemetry stream
    // The planner reads them from the standard stream — nothing extra needed here
}
```

### 5.3 MAVLink Hook — `GCS_MAVLink_Copter.cpp`

In the `SET_ATTITUDE_TARGET` handler, add a RAWES branch before the existing GUIDED angle handler:

```cpp
// In handle_message / MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
if (copter.flightmode == &copter.mode_rawes) {
    mavlink_set_attitude_target_t pkt;
    mavlink_msg_set_attitude_target_decode(&msg, &pkt);
    Quaternion q(pkt.q[0], pkt.q[1], pkt.q[2], pkt.q[3]);
    copter.mode_rawes.set_command(q, pkt.thrust);
    return;
}

// In NAMED_VALUE_FLOAT handler:
if (strncmp(packet.name, "tension\0\0\0", 10) == 0) {
    if (copter.flightmode == &copter.mode_rawes) {
        copter.mode_rawes.set_tension_measured_n(packet.value);
    }
}
```

### 5.4 Parameters — `ParametersG2`

```cpp
// Parameters.h — in ParametersG2 struct
AP_Float rawes_kp_cyclic;          // body_z error proportional gain → rate [1/s]
AP_Float rawes_kp_tension;         // tension PI kP [rad/N]
AP_Float rawes_ki_tension;         // tension PI kI [rad/(N·s)]
AP_Float rawes_tension_max_n;      // thrust=1.0 maps to this tension [N]
AP_Float rawes_bz_slew_rads;       // body_z slew rate limit [rad/s]
AP_Float rawes_col_min_deg;        // minimum collective blade angle [°]
AP_Float rawes_col_max_deg;        // maximum collective blade angle [°]
```

```cpp
// Parameters.cpp — AP_GROUPINFO table (replace XX with next available index)
AP_GROUPINFO("RAWES_KP_CYC",   XX, ParametersG2, rawes_kp_cyclic,       1.0f),
AP_GROUPINFO("RAWES_KP_TEN",   XX, ParametersG2, rawes_kp_tension,      0.01f),
AP_GROUPINFO("RAWES_KI_TEN",   XX, ParametersG2, rawes_ki_tension,      0.002f),
AP_GROUPINFO("RAWES_TEN_MAX",  XX, ParametersG2, rawes_tension_max_n,   200.0f),
AP_GROUPINFO("RAWES_BZ_SLEW",  XX, ParametersG2, rawes_bz_slew_rads,    0.12f),
AP_GROUPINFO("RAWES_COL_MIN",  XX, ParametersG2, rawes_col_min_deg,    -10.0f),
AP_GROUPINFO("RAWES_COL_MAX",  XX, ParametersG2, rawes_col_max_deg,     15.0f),
```

### 5.5 Mode Registration

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

## 6. Files Changed Summary

| File | Change | Lines |
|------|--------|-------|
| `ArduCopter/mode_rawes.cpp` | **New** — full implementation | ~140 |
| `ArduCopter/mode.h` | `ModeRAWES` class declaration + `RAWES` enum value | ~55 |
| `ArduCopter/Parameters.h` | 7 `rawes_*` fields in `ParametersG2` | ~7 |
| `ArduCopter/Parameters.cpp` | Register 7 parameters | ~35 |
| `ArduCopter/Copter.h` | Add `ModeRAWES mode_rawes` | 1 |
| `ArduCopter/mode.cpp` | Register in `mode_from_mode_num()` | ~3 |
| `ArduCopter/GCS_MAVLink_Copter.cpp` | `SET_ATTITUDE_TARGET` RAWES branch + `NAMED_VALUE_FLOAT "tension"` case | ~15 |
| **Total** | | **~256** |

---

## 7. Simulation Mapping

Every firmware component maps directly to existing simulation code:

| `Mode_RAWES` firmware | Python equivalent | File |
|-----------------------|------------------|------|
| `capture_equilibrium()` | Initial `_body_z_eq0`, `_tether_dir0` capture | `mediator.py` |
| `compute_bz_tether()` | `orbit_tracked_body_z_eq()` | `controller.py` |
| Rate-limited slerp in `run()` | Rate-limited slerp in mediator inner loop | `mediator.py` |
| `attitude_control->input_rate_bf_*()` | `compute_swashplate_from_state()` P-gain error term | `controller.py` |
| `attitude_control` rate PIDs + swashplate mix | `compute_swashplate_from_state()` D-term + `h3_inverse_mix()` | `controller.py`, `swashplate.py` |
| Tension PI → `set_throttle_out()` | `TensionController` | `controller.py` |
| `set_command()` | `trajectory.step()` return value (`attitude_q`, `thrust`) | `trajectory.py` |
| `send_state()` | `_state_pkt` dict in mediator loop | `mediator.py` |

The simulation is a faithful software model. Moving to hardware replaces direct Python function
calls with actual MAVLink messages over the SiK radio link.

---

## 8. Known Gaps and Risks

| Risk | Impact | Mitigation |
|------|--------|-----------|
| `H_PHANG` cyclic phase error | Medium — axis coupling | Runtime parameter; calibrate in SITL by comparing firmware orbit tracking to Python `orbit_tracked_body_z_eq()` output |
| Kaman flap lag | Medium — phase margin loss | Detune `RAWES_KP_CYC`; `AC_AttitudeControl` D-term damps oscillation |
| Tension measurement hardware (load cell) | **High — critical path** | Firmware validated in SITL using `NAMED_VALUE_FLOAT "tension"` from companion; load cell added before flight test |
| Orbit tracking before first tension (takeoff) | Medium — no tether direction during free climb | Planner sends explicit `attitude_q` target during slack-tether phase; `_eq_captured` guard prevents orbit tracking until position is valid |
| Planner timeout during reel-in tilt | Medium — loss of tilt command | 2 s timeout → `_planner_active = false` → snap back to `bz_tether` (safe fallback to natural orbit) |
| ENU/NED frame errors | High — subtle sign inversions | Single `enu_to_ned()` helper used at every boundary |
| `kp_cyclic` tuning | Medium — oscillation on first flight | Start at 0.3, increase slowly; SITL comparison to Python ground truth first |

---

## 9. References

- [`simulation/RAWESPROTOCOL.md`](simulation/RAWESPROTOCOL.md) — full protocol specification
- `ArduCopter/mode_acro_heli.cpp` — the structural template for Mode_RAWES
- `simulation/controller.py` — `orbit_tracked_body_z_eq()`, `compute_swashplate_from_state()`, `TensionController`
- `simulation/mediator.py` — rate-limited slerp, STATE/COMMAND packet assembly
- `ardupilot_implementation.md` — hardware parameters, swashplate config, open items
