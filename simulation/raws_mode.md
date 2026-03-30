# RAWES — Mode_RAWES Protocol and Firmware Implementation

## 1. System Overview

### 1.1 What is RAWES?

A RAWES (Rotary Airborne Wind Energy System) is a tethered autogyrating rotor
kite.  Wind drives the rotor to spin (autorotation); the spinning rotor
generates lift; the tether is angled so that lift has a component along the
tether, pulling against it and generating tension.  A ground-based winch
converts that tension into electrical energy by paying out the tether under
load (reel-out), then reeling it back in under low load (reel-in).  The
difference in tension between the two phases is the net energy per cycle.

The rotor has no motor to drive rotation — only trailing-edge flaps on each
blade, driven by a swashplate, which adjust blade pitch and therefore thrust
direction.  This is the only control input.  The flight controller's job is to
point the rotor disk in the right direction at the right time.

### 1.2 Glossary

| Term | Meaning |
|---|---|
| **body_z** | The unit vector along the rotor axle (spin axis). Naming convention used throughout this document and the codebase. |
| **Orbit tracking** | The Pixhawk-side control function that continuously rotates the attitude setpoint to match the hub's orbital position, keeping body_z aligned with the tether direction as the hub moves around the anchor. Runs at 400 Hz; the trajectory planner is not involved. |
| **ENU** | East-North-Up coordinate frame — used throughout this document. X = East, Y = North, Z = Up. Preferred over ArduPilot's native NED because positive-Z altitude makes tether and orbit geometry more intuitive. The conversion to NED is contained at the sensor boundary. |

### 1.3 Physical and control variables

| Symbol | Name | Description |
|---|---|---|
| **pos** | Hub position | 3D position of the rotor hub in ENU world frame [m] |
| **vel** | Hub velocity | 3D velocity of the rotor hub ENU [m/s] |
| **body_z** | Disk axis | Unit vector along the rotor axle (also tether direction at equilibrium) |
| **ξ** | Disk tilt from wind | Angle between body_z and the horizontal wind direction [°]. Controls how much thrust acts along vs. perpendicular to the tether. |
| **β** | Tether elevation | Angle of the tether above horizontal [°]. Determines the natural body_z direction at equilibrium. |
| **T** | Tether tension | Force along the tether [N]. Directly determines power = T × v_reel. |
| **ω_spin** | Rotor spin | Angular velocity of the rotor about its axle [rad/s]. Sustained by autorotation. Derived from GB4008 counter-torque motor eRPM: `omega_spin = eRPM × 2π/60 / 11 × 44/80`. |
| **ω** | Orbital angular velocity | Angular velocity of the hub about axes perpendicular to the axle [rad/s]. Controlled by cyclic swashplate inputs. |
| **L₀** | Tether rest length | Unstretched tether length [m]. Changed by the winch to reel out/in. |
| **θ_col** | Collective pitch | Average blade pitch across all blades [rad]. Controls thrust magnitude. |
| **θ_lon** | Longitudinal cyclic | Tilt of the swashplate fore/aft [rad]. Tilts disk north/south. |
| **θ_lat** | Lateral cyclic | Tilt of the swashplate left/right [rad]. Tilts disk east/west. |
| **v_winch** | Winch speed | Rate of tether length change [m/s]. +ve = pay out, −ve = reel in. |

### 1.4 The natural orbit

At equilibrium, the hub does not hover in one place — it orbits the anchor
point at constant tether length and elevation.  The tether direction defines
the equilibrium disk axis (body_z), and as the hub moves the disk tilts with
it, creating a lateral force that drives the orbit.  This is the system's
natural resting state and requires zero control effort to maintain.

The orbit is not a problem to be corrected — it is the expected flight
condition.  The baseline attitude setpoint rotates with the orbit automatically
(orbit tracking), and the trajectory planner only needs to command deviations.

### 1.5 The pumping cycle (De Schutter 2018)

**Reel-out (power phase):** The disk is tether-aligned (ξ ≈ 30–55°).  High
collective produces high thrust mostly along the tether.  High tether tension.
The winch pays out against this tension, driving a generator.

**Reel-in (recovery phase):** The disk tilts so that ξ increases toward 90°.
Thrust acts mostly upward rather than along the tether.  Tether tension drops
to near the gravity component alone (~15–30 N).  The winch reels in cheaply.

Net energy per cycle = (T_out − T_in) × v_reel × t_phase > 0 as long as T_out > T_in.

### 1.6 Counter-torque motor

The rotor hub spins freely in the wind, but the electronics, swashplate, and
servos must stay stationary.  The GB4008 (66KV, 24N22P = 11 pole pairs) drives
the axle via an 80:44 spur gear, applying continuous counter-torque to prevent
the electronics assembly from co-rotating with the hub — directly analogous to
the tail rotor of a helicopter.

The gear ratio sets the motor operating RPM band; all authority comes from the
yaw rate PI loop on the Pixhawk.  The AM32 ESC reports eRPM via DShot telemetry,
which the Pixhawk converts to rotor spin rate:

```
omega_spin = eRPM × 2π/60 / 11 × 44/80
```

This is an inner-loop function owned entirely by `Mode_RAWES`.  The trajectory
planner never commands it.

---

## 2. Control Architecture

Three nodes, two communication boundaries:

```
┌─────────────────────────────────────────────────┐
│  Trajectory Planner  (ground station, ~10 Hz)   │
│                                                 │
│  Inputs:  pos_ned, vel_ned, body_z_ned,           │
│           omega_spin — all standard streams      │
│           tension (local from winch controller) │
│           tether_length (local from winch)      │
│  Outputs: attitude_q + thrust → Pixhawk         │
│           winch_speed → Winch Controller        │
│                                                 │
│  Owns: phase logic, energy accounting,          │
│        tension PI, wind estimation, MPC (future)│
└──────────────┬──────────────────┬───────────────┘
               │                  │
               │ MAVLink          │ local interface
               │ (~10 Hz,         │ (serial / CAN /
               │ SiK 433 MHz)     │ analog — TBD)
               │ std streams ↑    │
               │ (pos,vel,att,    │ winch_speed_ms ↓
               │  ESC_STATUS)     │ tension (local) ↑
               │ attitude_q,      │
               │ thrust ↓         │
┌──────────────┴──────────────┐  ┌┴──────────────────────┐
│  Mode_RAWES                 │  │  Winch Controller      │
│  (Pixhawk 6C, in air)       │  │  (ground station)      │
│  400 Hz                     │  │                        │
│  Inputs: attitude_q, thrust │  │  Drives motor to pay   │
│          EKF, IMU           │  │  out / reel in tether  │
│  Outputs: swashplate PWM,   │  │  at commanded rate.    │
│           counter-torque PWM│  │                        │
│                             │  │  Owns: reel speed PID, │
│  Owns: orbit tracking,      │  │  tension safety limits │
│  body_z slewing,            │  │                        │
│  swashplate mixing,         │  │  Load cell + encoder   │
│  counter-torque control     │  │  — source of tension   │
│                             │  │  and tether length     │
└─────────────────────────────┘  └────────────────────────┘
```

**Tether tension is measured at the base station**, not on the hub.  A load
cell on the winch drum measures exactly the right quantity for energy accounting
(work = T_base × v_reel).  The base-to-hub tension difference is the tether
weight component along the tether (0.3–3.1 N at operating lengths — under 5 %
of the operating range; absorbed by the PI integrator).

The tension PI runs on the ground (trajectory planner), where the load cell
measurement is local and fresh.  The PI output — a normalized collective value
[0..1] — is sent to the Pixhawk as the `thrust` field of `SET_ATTITUDE_TARGET`.
The Pixhawk passes it directly to `set_throttle_out()` with no further
computation.  The Pixhawk has no knowledge of tension.

### Design principles

**Natural orbit is free.** `Mode_RAWES` tracks the tether direction at 400 Hz
without planner involvement.  The planner only intervenes to request a specific
disk orientation.

**Inner loops stay on the Pixhawk.** Attitude tracking (cyclic), body_z
slewing, and counter-torque control run at 400 Hz inside `Mode_RAWES`.
The planner sends the collective directly; the Pixhawk executes it.

**Winch is on the ground.** The Pixhawk is never involved in winch control.

**Thrust field = normalized collective.** The `thrust` field of
`SET_ATTITUDE_TARGET` carries a normalized collective [0..1] computed by the
ground PI.  `Mode_RAWES` passes it directly to `set_throttle_out()` — no
tension awareness, no conversion, no PI on the Pixhawk.  This is structurally
identical to a pilot's throttle stick in ACRO mode.

---

## 3. Protocol

### 3.1 STATE — Pixhawk → Planner (~10 Hz, all standard streams)

The planner reads everything it needs from standard ArduPilot telemetry — no custom messages
and no `send_state()` function in `Mode_RAWES`:

| Standard stream | MAVLink message | What the planner uses |
|---|---|---|
| Position + velocity | `LOCAL_POSITION_NED` (msg #32) | Hub position and velocity in NED |
| Attitude | `ATTITUDE_QUATERNION` (msg #31) | Full orientation; `body_z_ned = quat_apply(q, [0,0,1])` |
| Rotor spin | `ESC_STATUS` (msg #291) | `rpm[RAWES_CTR_ESC]`; planner converts: `omega_spin = rpm × 2π/60 / 11 × 44/80` |

`ESC_STATUS` is streamed automatically by ArduPilot from `AP_ESC_Telem` — no Pixhawk-side code
needed beyond setting the stream rate.

### 3.1.1 Anchor Position

`Mode_RAWES` needs the anchor position to compute `bz_tether = normalize(pos_hub − anchor)`
at every 400 Hz step.  The anchor is set via three parameters:

| Parameter | Description |
|---|---|
| `RAWES_ANCHOR_LAT` | Anchor latitude [degrees, AP_Float] |
| `RAWES_ANCHOR_LON` | Anchor longitude [degrees, AP_Float] |
| `RAWES_ANCHOR_ALT` | Anchor altitude AMSL [m, AP_Float] |

At `init()` these are converted once to local NED relative to the EKF origin and
stored as `_anchor_ned`.  The hub can be anywhere when `Mode_RAWES` is entered —
ground-launched, hand-launched, or already in the air.

The wind direction does **not** affect the anchor calculation.  `bz_tether` is
derived from actual hub position, so it naturally tracks wherever the hub flies —
downwind, crosswind, or during wind shifts — without any wind knowledge on the
Pixhawk.

---

`tension_n` is **not** in the STATE packet.  The Pixhawk has no load cell and
never measures tension.  The planner reads it locally from the winch controller.

`tether_length_m` is also **not** in the STATE packet.  The Pixhawk does not
know how much tether has been paid out — only the winch controller knows this
(from its encoder).  The planner reads tether length from the winch controller
via the same local link as tension, requiring no protocol change.

### 3.2 Planner → Pixhawk Uplink (~10 Hz)

Exactly one MAVLink message: `SET_ATTITUDE_TARGET`.

| Field | Description |
|---|---|
| `quaternion` | Desired disk orientation ENU. Identity `[1,0,0,0]` = natural tether-aligned orbit (planner silent). Non-identity: `body_z_target = quat_apply(q, [0,0,1])`. `Mode_RAWES` slews at `RAWES_BZ_SLEW`. |
| `thrust` | Normalized collective [0..1], computed by the ground PI from the load cell measurement. `Mode_RAWES` passes this directly to `set_throttle_out()` — no conversion on the Pixhawk. |

The ground PI runs locally with fresh load cell data:
```
error          = tension_setpoint_n − tension_measured_n   (both in N, local)
collective_rad = kP × error + kI × ∫error dt
thrust         = clamp((collective_rad − col_min_rad) / (col_max_rad − col_min_rad), 0, 1)
```
`col_min_rad` / `col_max_rad` and the PI gains are ground-station configuration.

### 3.3 Winch Command — Planner → Winch Controller (local link)

| Field | Description |
|---|---|
| `winch_speed_ms` | Winch rate [m/s]. +ve = pay out, −ve = reel in, 0 = hold. Maps to `MAV_CMD_DO_WINCH` if the winch controller speaks MAVLink. |

The Pixhawk is not involved.

### 3.4 Trajectory Planner Logic (~10 Hz)

```
Each step:

1. Read standard streams:
        pos_ned + vel_ned  ← LOCAL_POSITION_NED
        body_z_ned         ← ATTITUDE_QUATERNION  (quat_apply(q, [0,0,1]))
        omega_spin         ← ESC_STATUS[RAWES_CTR_ESC].rpm × 2π/60 / 11 × 44/80
2. Read tension_n from Winch Controller (local link — no MAVLink hop)

3. Determine phase (reel-out / reel-in) from local elapsed time since release

4. Run tension PI (local, fresh data):
       error          = tension_setpoint_n − tension_n
       collective_rad = kP × error + kI × ∫error dt
       thrust         = clamp((collective_rad − col_min_rad) / (col_max_rad − col_min_rad), 0, 1)

5. Compute:
       attitude_q     ← identity during reel-out (planner silent)
                         quat_from_vectors([0,0,1], body_z_reel_in) during reel-in
                         body_z_reel_in = cos(xi)*wind_dir + sin(xi)*[0,0,1]
       winch_speed_ms ← +v_reel_out or −v_reel_in

6. Send SET_ATTITUDE_TARGET (attitude_q + thrust) → Pixhawk
7. Send winch_speed_ms                             → Winch Controller
```

---

## 4. Wind Estimation

Wind direction and speed are needed to compute `body_z_reel_in`.  Four methods,
ordered by implementation complexity:

**Method 1 — Rotor spin rate (in-plane speed):**
Using the autorotation torque balance: `v_inplane = omega_spin² × K_drag / K_drive`.
`v_wind ≈ v_inplane / sin(xi)` where xi comes from Method 2.

**Method 2 — Orbital mean position (direction):**
Over one orbit, `wind_dir ≈ normalize(mean(pos_ned_horizontal))`.
No extra hardware. Direction converges within one orbit (~60 s).

**Method 3 — Ground anemometer (future):**
3D ultrasonic anemometer at ground station, extrapolated to hub altitude via
log profile: `v(h) = v_ref × ln(h/z₀) / ln(h_ref/z₀)`.  No protocol impact —
the planner reads it locally.

**Method 4 — EKF wind state augmentation (future):**
Augment the Pixhawk EKF with a 3D wind vector estimated from tension, velocity,
and omega_spin.  If implemented, `wind_ned` would be added as a `NAMED_VALUE_FLOAT` custom field.

---

## 5. Takeoff and Landing

### 5.1 Takeoff

**Physical sequence:**
1. Ground station spins rotor to ω_spin ≥ ω_min (~10–15 rad/s). Planner monitors STATE; Pixhawk does nothing.
2. Release mechanism drops rotor. Lift > weight → rapid climb.
3. Tether pays out. Once taut, tension develops and lateral stability begins.
4. Natural orbit establishes. Planner begins pumping cycle.

**Phase ownership:**

| Phase | Planner | Mode_RAWES |
|---|---|---|
| Spin-up | Monitor `omega_spin`; trigger release at ω ≥ ω_min | None |
| Free climb (slack tether) | Pay out winch; send explicit `attitude_q` target | Attitude hold via swashplate; collective from `thrust` |
| Tether catch | Detect tension > threshold (local winch read); reduce thrust | Orbit tracking begins |
| Transition to pumping | Ramp to operating tension setpoint | Natural orbit; follow COMMAND packets |

During the slack-tether phase, `_eq_captured` is false so orbit tracking is
suppressed until EKF position is valid and tether direction is meaningful.

### 5.2 Landing — spiral descent

```
Step 1 — Normal reel-in:
    Planner: slow winch reel-in, reduce tension setpoint.
    Mode_RAWES: orbit track (identity attitude_q); collective from thrust.
    Hub spirals inward and downward.

Step 2 — Final approach (tether_length < ~10 m):
    Planner: ramp thrust → 0 over ~5 s; hold or very slow reel-in.
    Mode_RAWES: collective approaches zero; body_z alignment maintained.
    Autorotation continues — rotor stores kinetic energy during descent.

Step 3 — Flare (optional):
    Planner: brief thrust spike (~0.3) for 1–2 s.
    Mode_RAWES: momentary collective increase slows final descent.

Step 4 — Ground contact:
    Planner: engage ground motor (local command).
    Mode_RAWES: hold last attitude command.
```

**The division of responsibility is identical to pumping flight.**  Landing is
a subset of the normal protocol — no new COMMAND fields are required.

---

## 6. Example: De Schutter Pumping Cycle

```
t=0s:   → Pixhawk:          attitude_q=[1,0,0,0] (identity),
                             thrust = ground PI output (targeting ~200 N, ≈0.2 normalized)
        → Winch controller: winch_speed=+0.4 m/s (pay out)
        Mode_RAWES holds tether-aligned orbit naturally.  Winch pays out.
        Ground PI regulates tension to 200 N by adjusting thrust each 10 Hz step.

t=30s:  → Pixhawk:          attitude_q=quat_from_vectors([0,0,1],[cos55°,0,sin55°]),
                             thrust = ground PI output (targeting ~80 N, ≈0.5 normalized)
        → Winch controller: winch_speed=-0.4 m/s (reel in)
        Mode_RAWES slews body_z toward 55° from wind (~5 s at RAWES_BZ_SLEW).

t=35s+: Mode_RAWES holds 55° tilt.  Ground PI holds low tension.  Winch reels in.

t=60s:  → Pixhawk:          attitude_q=[1,0,0,0] (identity),
                             thrust = ground PI output (targeting ~200 N)
        → Winch controller: winch_speed=+0.4 m/s (pay out)
        Mode_RAWES slews body_z back to tether-aligned.  Next reel-out begins.
```

---

## 7. Mode_RAWES Firmware

### 7.1 Relationship to ACRO_Heli

`ModeRAWES` **subclasses** `ModeAcro_Heli` directly.  The inheritance chain is:

```
Mode → ModeAcro → ModeAcro_Heli → ModeRAWES
```

`run()` is overridden with two substitutions; everything else is inherited:

| ACRO_Heli `run()` | ModeRAWES `run()` |
|---|---|
| Pilot RC sticks → body-frame rate commands | Orbit-tracking body_z error → body-frame rate commands |
| Pilot throttle → collective | Ground PI `thrust` field → collective (direct passthrough) |
| — | Rate-limited slerp for body_z transitions |

Spool state guards, `AC_AttitudeControl` rate PIDs, swashplate mixing,
counter-torque yaw control, and arming infrastructure are all **inherited**
from `ModeAcro_Heli` — no code needed.

**Firmware complexity: ~120 lines of new C++.**

### 7.2 The 400 Hz Loop

```
Each step:

1. Spool state guards (inherited behaviour):
       if SHUT_DOWN or GROUND_IDLE: ModeAcro_Heli::run(); return;

2. Hold last COMMAND packet if no new one arrived.
   Planner timeout (2 s): snap body_z_eq back to bz_tether (natural orbit).

3. Orbit tracking:
       Rotate initial equilibrium body_z by azimuthal change since free-flight start
       → bz_tether (current tether-aligned target).

4. Attitude setpoint:
       if attitude_q == identity:
           body_z_eq = bz_tether              ← natural orbit, planner silent
       else:
           body_z_target = quat_apply(attitude_q, [0,0,1])
           body_z_eq = slerp_rate_limited(body_z_eq_prev, body_z_target,
                                          RAWES_BZ_SLEW, dt)

5. Cyclic — body_z error → body-frame rate commands:
       error = cross(body_z_now, body_z_eq)   (world frame)
       err_body = R_body_to_ned.T × error
       attitude_control->input_rate_bf_roll_pitch_yaw_rads(
           kp × err_body.x, kp × err_body.y, 0.0)
       // AC_AttitudeControl rate PIDs + D-term damping handle the rest

6. Collective — direct passthrough:
       // thrust [0..1] = normalized collective from ground PI; no conversion needed
       attitude_control->set_throttle_out(_thrust_cmd, false, throttle_filt)

7. Counter-torque — null assembly yaw rate:
       // yaw rate command = 0.0 → ATC_RAT_YAW drives GB4008 motor via H_TAIL_TYPE=4
       // (zero yaw rate already sent in step 4 above)
       // AP_ESC_Telem streams ESC_STATUS automatically — no send_state() needed
```

### 7.3 What ArduPilot Provides Free

**Inherited from `ModeAcro_Heli` (zero new code):**

| Need | Mechanism |
|---|---|
| Spool state guards (`SHUT_DOWN`, `GROUND_IDLE`) | `ModeAcro_Heli::run()` called for guard cases |
| Rate PIDs + swashplate H3/H4 mix | `AC_AttitudeControl::input_rate_bf_roll_pitch_yaw_rads()` |
| Collective → PWM | `AC_AttitudeControl::set_throttle_out()` |
| Counter-torque yaw (GB4008 via `H_TAIL_TYPE=4`) | `ATC_RAT_YAW` PID |
| Arming infrastructure | `ModeAcro_Heli::allows_arming()` base (overridden to restrict to MAVLink) |
| Mode registration boilerplate | Inherited `Mode` framework |
| AHRS attitude + rates | `ahrs.get_rotation_body_to_ned()`, `ahrs.get_gyro()` |
| EKF position | `inertial_nav.get_position_neu_m()` |
| ESC telemetry (AM32 eRPM) | `AP::esc_telem().get_rpm()` |
| `LOCAL_POSITION_NED` telemetry | Already on stream — planner reads directly |

### 7.4 What Must Be Built

| Component | Python source | Lines | Notes |
|-----------|--------------|-------|-------|
| Class declaration + overrides | — | ~20 | Inherits from `ModeAcro_Heli`; only override `run()`, `is_autopilot()`, etc. |
| Orbit tracking (`compute_bz_tether`) | `controller.py::orbit_tracked_body_z_eq()` | ~25 | Pure geometry |
| Rate-limited slerp | `mediator.py` blend loop | ~20 | Firmware owns timing |
| Cyclic rate command | `controller.py::compute_swashplate_from_state()` | ~15 | P gain × body_z error → rate |
| Tension PI | `controller.py::TensionController` | — | **Ground-side** only; not on Pixhawk |
| Counter-torque | `sensor.py::SpinSensor` | ~15 | `ATC_RAT_YAW` → GB4008 via `H_TAIL_TYPE=4` |
| MAVLink COMMAND receiver | — | ~15 | Parse `SET_ATTITUDE_TARGET` |
| ENU ↔ NED helpers | `sensor.py` | ~5 | Two inline functions |
| **Total** | | **~115 lines** | |

---

## 8. Implementation (C++)

### 8.0 `init()` — Anchor Conversion

```cpp
bool ModeRAWES::init(bool ignore_checks) {
    if (!ModeAcro_Heli::init(ignore_checks)) return false;

    // Convert RAWES_ANCHOR_LAT/LON/ALT parameters to local NED frame.
    // Hub can be anywhere when mode is entered (ground, hand-launch, mid-air).
    Location anchor_loc;
    anchor_loc.lat         = (int32_t)(_anchor_lat.get() * 1.0e7f);
    anchor_loc.lng         = (int32_t)(_anchor_lon.get() * 1.0e7f);
    anchor_loc.alt         = (int32_t)(_anchor_alt.get() * 100.0f);  // m → cm
    anchor_loc.relative_alt = false;

    Vector3f offset_ned;
    if (!AP::ahrs().get_relative_position_NED_origin(anchor_loc, offset_ned)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "RAWES: EKF origin not set — set RAWES_ANCHOR params");
        return false;
    }
    _anchor_ned = offset_ned;

    _eq_captured    = false;
    _planner_active = false;
    _thrust_cmd     = 0.5f;
    return true;
}
```

### 8.1 Orbit Tracking

Port of `orbit_tracked_body_z_eq()` from `controller.py`:

```cpp
// capture once at free-flight start
_body_z_eq0_ned  = ahrs.get_rotation_body_to_ned().colz();
Vector3f pos_ned = inertial_nav.get_position_neu_m();
pos_ned.z        = -pos_ned.z;                          // NEU → NED
_tether_dir0_ned = (pos_ned - _anchor_ned).normalized();

// each 400 Hz step
Vector3f cur_pos_ned = inertial_nav.get_position_neu_m();
cur_pos_ned.z = -cur_pos_ned.z;                         // NEU → NED
Vector3f tether_now = (cur_pos_ned - _anchor_ned).normalized();
const Vector3f axis  = _tether_dir0_ned % tether_now;
const float    sinth = axis.length();
const float    costh = _tether_dir0_ned * tether_now;
if (sinth > 1e-6f) {
    Quaternion q; q.from_axis_angle(axis / sinth, atan2f(sinth, costh));
    bz_tether = q * _body_z_eq0_ned;
}
```

### 8.2 Rate-Limited Slerp

```cpp
const float dot    = constrain_float(_body_z_eq_ned * _body_z_target_ned, -1.0f, 1.0f);
const float remain = acosf(dot);
const float step   = MIN(_body_z_slew_rate_rads.get() * dt, remain);
if (remain > 1e-4f) {
    const Vector3f axis = (_body_z_eq_ned % _body_z_target_ned).normalized();
    Quaternion q; q.from_axis_angle(axis, step);
    _body_z_eq_ned = q * _body_z_eq_ned;
}
```

### 8.3 Cyclic Rate Command

```cpp
const Vector3f body_z_now = ahrs.get_rotation_body_to_ned().colz();
const Vector3f err_ned    = body_z_now % _body_z_eq_ned;
const Vector3f err_body   = ahrs.get_rotation_body_to_ned().transposed() * err_ned;
const float    kp         = _kp_cyclic.get();
attitude_control->input_rate_bf_roll_pitch_yaw_rads(kp * err_body.x,
                                                     kp * err_body.y,
                                                     0.0f);
```

### 8.4 Collective Passthrough

The tension PI runs on the ground (trajectory planner).  The Pixhawk receives
the PI output as a normalized collective in the `thrust` field and passes it
directly to `set_throttle_out()`:

```cpp
// _thrust_cmd is updated by set_command() whenever SET_ATTITUDE_TARGET arrives
attitude_control->set_throttle_out(_thrust_cmd, false, g.throttle_filt);
```

### 8.5 Counter-Torque Motor and omega_spin

The Pixhawk IMU is mounted on the stationary assembly.  Its gyro Z-axis
measures co-rotation rate directly — the quantity the counter-torque loop must
null.  Passing `yaw_rate = 0.0f` in the rate command (step 4 of the loop) causes
`AC_AttitudeControl` to drive `ATC_RAT_YAW`, which commands the GB4008 motor
via `H_TAIL_TYPE = 4` (Direct Drive Fixed Pitch).

**Direction:** GB4008 must oppose hub spin direction.  Set `H_TAIL_DIR = 1`
(reversed) if the default direction is wrong — verify on bench before first
powered test.

**omega_spin measurement** — ArduPilot streams `ESC_STATUS` (msg #291) automatically
from `AP_ESC_Telem` at the configured telemetry rate.  The planner reads
`ESC_STATUS[RAWES_CTR_ESC].rpm` (mechanical RPM, already divided by pole pairs
internally by ArduPilot) and applies the gear reduction:

```python
# Planner-side, no Pixhawk code needed
omega_spin = esc_status.rpm[RAWES_CTR_ESC] * (2*pi/60) * (44/80)
```

The 80:44 gear sets the motor operating RPM band (hub 18–35 rad/s → motor
~33–64 rad/s mechanical) so the AM32 commutates at a useful rate across the
full spin range.  No `send_state()` function is needed in `Mode_RAWES`.

### 8.6 Class Declaration — `ArduCopter/mode.h`

Inherits from `ModeAcro_Heli`.  Only override what differs:

```cpp
class ModeRAWES : public ModeAcro_Heli {
public:
    using ModeAcro_Heli::ModeAcro_Heli;
    Number mode_number() const override { return Number::RAWES; }

    void run() override;
    bool init(bool ignore_checks) override;

    // Autopilot mode — disallow manual arming, require MAVLink/scripting
    bool is_autopilot()        const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method m) const override {
        return m == AP_Arming::Method::MAVLINK ||
               m == AP_Arming::Method::SCRIPTING;
    }

    void set_command(const Quaternion& attitude_q_enu, float thrust_norm);

protected:
    const char *name()  const override { return "RAWES"; }
    const char *name4() const override { return "RAWE"; }

private:
    void capture_equilibrium();
    Vector3f compute_bz_tether() const;

    bool     _eq_captured     = false;
    Vector3f _anchor_ned;                // tether anchor in local NED — set from params at init()
    Vector3f _body_z_eq0_ned;
    Vector3f _tether_dir0_ned;
    Vector3f _body_z_eq_ned;
    Vector3f _body_z_target_ned;
    float    _thrust_cmd      = 0.5f;
    bool     _planner_active  = false;
    uint32_t _planner_recv_ms = 0;

    AP_Float _kp_cyclic;
    AP_Float _body_z_slew_rate_rads;
    AP_Int8  _ctr_esc_idx;
    AP_Float _anchor_lat;               // deg
    AP_Float _anchor_lon;               // deg
    AP_Float _anchor_alt;               // m AMSL
};
```

### 8.7 MAVLink Hook — `GCS_MAVLink_Copter.cpp`

```cpp
// SET_ATTITUDE_TARGET handler:
if (copter.flightmode == &copter.mode_rawes) {
    mavlink_set_attitude_target_t pkt;
    mavlink_msg_set_attitude_target_decode(&msg, &pkt);
    Quaternion q(pkt.q[0], pkt.q[1], pkt.q[2], pkt.q[3]);
    copter.mode_rawes.set_command(q, pkt.thrust);
    return;
}

// No NAMED_VALUE_FLOAT "tension" handler needed — tension PI runs on the ground.
```

### 8.8 Parameters

```cpp
// Parameters.cpp — AP_GROUPINFO (replace XX with next available index)
AP_GROUPINFO("RAWES_KP_CYC",      XX, ParametersG2, rawes_kp_cyclic,        1.0f),
AP_GROUPINFO("RAWES_BZ_SLEW",     XX, ParametersG2, rawes_bz_slew_rads,     0.12f),
AP_GROUPINFO("RAWES_CTR_ESC",     XX, ParametersG2, rawes_ctr_esc_idx,           3),
AP_GROUPINFO("RAWES_ANCHOR_LAT",  XX, ParametersG2, rawes_anchor_lat,        0.0f),
AP_GROUPINFO("RAWES_ANCHOR_LON",  XX, ParametersG2, rawes_anchor_lon,        0.0f),
AP_GROUPINFO("RAWES_ANCHOR_ALT",  XX, ParametersG2, rawes_anchor_alt,        0.0f),
```

The tension PI gains (`kP`, `kI`), collective range (`col_min_rad`, `col_max_rad`),
and tension setpoints are **ground-station configuration**, not Pixhawk parameters.
See `controller.py::TensionController` for defaults (beaupoil_2026 BEM scale).

`RAWES_CTR_ESC = 3` is the DShot output index for the GB4008 motor (0-based).
Adjust to match hardware wiring.

### 8.9 Mode Registration

```cpp
// Copter.h:      ModeRAWES mode_rawes;
// mode.h enum:   RAWES = 35,   // or next available
// mode.cpp:      case Mode::Number::RAWES: return &mode_rawes;
```

---

## 9. Files Changed

| File | Change | Lines |
|------|--------|-------|
| `ArduCopter/mode_rawes.cpp` | **New** — subclass of `ModeAcro_Heli` | ~95 |
| `ArduCopter/mode.h` | Class declaration (inherits most from `ModeAcro_Heli`) + RAWES enum | ~35 |
| `ArduCopter/Parameters.h` | 3 `rawes_*` fields | ~3 |
| `ArduCopter/Parameters.cpp` | Register 3 parameters | ~15 |
| `ArduCopter/Copter.h` | `ModeRAWES mode_rawes` | 1 |
| `ArduCopter/mode.cpp` | `mode_from_mode_num()` entry | ~3 |
| `ArduCopter/GCS_MAVLink_Copter.cpp` | `SET_ATTITUDE_TARGET` RAWES branch | ~10 |
| **Total** | | **~162 lines** |

---

## 10. Simulation Mapping

Every firmware component maps to existing Python simulation code:

| Mode_RAWES firmware | Python equivalent | File |
|---------------------|------------------|------|
| `capture_equilibrium()` | `_body_z_eq0`, `_tether_dir0` capture at free-flight start | `mediator.py` |
| `compute_bz_tether()` | `orbit_tracked_body_z_eq()` | `controller.py` |
| Rate-limited slerp | Rate-limited slerp in mediator inner loop | `mediator.py` |
| Cyclic rate command + AC_AttitudeControl | `compute_swashplate_from_state()` + `h3_inverse_mix()` | `controller.py`, `swashplate.py` |
| `set_throttle_out(_thrust_cmd)` | `TensionController` PI output → normalized collective → `trajectory.step()` thrust | `controller.py`, `trajectory.py` |
| `set_command()` | `trajectory.step()` return value (`attitude_q`, `thrust`) | `trajectory.py` |
| omega_spin (planner reads ESC_STATUS) | `SpinSensor.measure()` — models AM32 eRPM jitter via Gaussian σ | `sensor.py` |
| Counter-torque | Not modelled (single-body; internal force cancels) | — |
| STATE (pos, vel, att, ESC_STATUS) | Standard ArduPilot streams; planner reads directly | `mediator.py` |

---

## 11. Known Gaps and Risks

| Risk | Impact | Mitigation |
|------|--------|-----------|
| `H_PHANG` cyclic phase error | Medium — axis coupling | Calibrate in SITL: compare firmware orbit tracking to Python `orbit_tracked_body_z_eq()` output |
| Kaman flap lag | Medium — phase margin loss | Detune `RAWES_KP_CYC`; AC_AttitudeControl D-term damps oscillation |
| Load cell hardware (tension feedback) | **High — critical path** | Validate ground PI in simulation (`TensionController`) before hardware; add load cell to winch before flight |
| Orbit tracking before first tether tension | Medium — no tether direction during free climb | Planner sends explicit `attitude_q`; `_eq_captured` guard prevents orbit tracking until EKF position valid |
| Planner timeout during reel-in tilt | Medium — loss of tilt command | 2 s timeout → snap back to `bz_tether` (natural orbit fallback) |
| ENU/NED frame errors | High — subtle sign inversions | Single `enu_to_ned()` helper; test against Python `T_ENU_NED` from `frames.py` |
| `RAWES_KP_CYC` tuning | Medium — oscillation on first flight | Start at 0.3, increase slowly; compare to Python closed-loop in SITL first |
| GB4008 direction (H_TAIL_DIR) | Medium — yaw runaway if wrong | Verify on bench: with hub spinning CCW (viewed from above), motor must apply CW torque |

---

## 12. References

- `ArduCopter/mode_acro_heli.cpp` — structural template
- `simulation/controller.py` — `orbit_tracked_body_z_eq()`, `compute_swashplate_from_state()`, `TensionController`
- `simulation/mediator.py` — rate-limited slerp, STATE/COMMAND packet assembly
- `simulation/sensor.py` — `SpinSensor` (omega_spin noise model)
- `ardupilot_implementation.md` — hardware parameters, swashplate config
