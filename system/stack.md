# RAWES -- Flight Control Stack Reference

Complete reference for the deployed RAWES flight control system: ground planner, winch
controller, Pixhawk Lua scripts, ArduPilot configuration, and startup/arming procedures.

---

## 1. System Architecture

Three nodes, two communication boundaries. The Pixhawk runs two distinct loops at different
rates -- the 400 Hz ArduPilot main loop (ACRO_Heli) and the Lua scripting scheduler (50/100 Hz).
Lua writes RC overrides that the main loop consumes at full rate.

```mermaid
flowchart TB
    subgraph WC["Winch Controller"]
        WCI["<b>Winch Controller</b><BR/>Reel speed PID<BR/>Load cell · Encoder"]
        WM["<b>Wind Meter</b><BR/>Anemometer<BR/>speed · direction"]
    end

    subgraph GS["Ground Station"]
        TP["<b>Trajectory Planner</b> (~10 Hz)<BR/>Phase logic · Tension PI<BR/>Wind estimation"]
    end

    subgraph PX["Pixhawk 6C (in air)"]
        direction TB
        LUA["<b>rawes.lua</b>  100 Hz base<BR/>SCR_USER6: 0=none, 1=steady 50 Hz, 4=landing 50 Hz, 5=pumping 50 Hz<BR/>RAWES_ARM (named float): timed arm/disarm<BR/>Orbit tracking · Rate-limited slerp · Cyclic P loop<BR/>VZ collective (modes 4,5) · Ch1/Ch2/Ch3 overrides · Ch4 neutral (no yaw wind-up)"]
        ACRO["<b>ACRO_Heli</b>  400 Hz  (ArduPilot main loop)<BR/>Rate PIDs · H3-120 mix · Spool guards · Servo PWM"]
        LUA --> ACRO
    end

    TP -->|"MAV_CMD_DO_WINCH<BR/>(winch_speed_ms, MAVLink)"| WCI
    WCI -->|"tension_n · encoder_ms<BR/>wind_speed · wind_dir<BR/>(MAVLink)"| TP
    TP -->|"SET_ATTITUDE_TARGET<BR/>(attitude_q + thrust, MAVLink)"| PX
    PX -->|"LOCAL_POSITION_NED<BR/>ATTITUDE_QUATERNION · ESC_STATUS"| TP
```

**Key design principles:**

- **Natural orbit is free.** Lua orbit-tracking tracks the tether direction at 50 Hz without
  planner involvement. The planner only intervenes to request a specific disk orientation.
- **Inner loops stay on the Pixhawk.** Attitude tracking (cyclic), body_z slewing, and
  counter-torque control run inside Lua scripts at 50-100 Hz. No custom firmware required --
  both scripts run on top of stock ACRO_Heli mode.
- **Winch is a separate MAVLink node.** The Pixhawk is never involved in winch control. The ground station commands winch speed via MAV_CMD_DO_WINCH and receives tension, encoder, and anemometer data back over MAVLink. The wind meter is co-located on the winch node.
- **Thrust field = normalized collective.** The thrust field of SET_ATTITUDE_TARGET carries
  normalized collective [0..1] from the ground PI. The Lua script forwards it directly to
  Ch3 RC override -- no tension awareness on the Pixhawk.

**Tether tension** is measured at the base station, not on the hub. A load cell on the winch
drum measures exactly the right quantity for energy accounting (work = T_base x v_reel).
The base-to-hub tension difference is the tether weight component along the tether
(0.3-3.1 N at operating lengths -- under 5% of operating range; absorbed by PI integrator).

---

## 2. Concepts & Glossary

### 2.1 Glossary

| Term | Meaning |
|---|---|
| body_z | Unit vector along the rotor axle (spin axis) |
| Orbit tracking | Pixhawk-side control that rotates attitude setpoint to match hub orbital position |
| NED | North-East-Down coordinate frame (X=North, Y=East, Z=Down). Up is [0,0,-1] |
| Rodrigues rotation | Rotates a unit vector **v** around a unit axis **k** by angle **theta**: `v_rot = v*cos(theta) + (k x v)*sin(theta) + k*(k.v)*(1-cos(theta))`. Used throughout rawes.lua for orbit tracking and rate-limited slerp because it operates directly on Vector3f components without requiring a quaternion library. |
| Slerp | **Spherical Linear intERPolation** — moves a unit vector toward a target at a constant angular rate (rad/s), independent of frame rate. Given current vector **a**, target **b**, and slew rate **r**: compute the angle between them, cap the step at `r * dt`, then rotate **a** toward **b** by exactly that angle using Rodrigues rotation. The result stays on the unit sphere (no renormalization needed) and moves at a predictable maximum angular speed regardless of how large the error is. Used by `OrbitTracker` / `slerp_body_z` (Python) and `rawes.lua`'s `slerp_step` to rate-limit body_z attitude setpoint changes to 0.40 rad/s. |

### 2.2 Physical and Control Variables

| Symbol | Name | Description |
|---|---|---|
| pos | Hub position | 3D position of rotor hub in NED [m] |
| vel | Hub velocity | 3D velocity of rotor hub in NED [m/s] |
| body_z | Disk axis | Unit vector along rotor axle (also tether direction at equilibrium) |
| xi | Disk tilt from wind | Angle between body_z and horizontal wind direction [deg] |
| beta | Tether elevation | Angle of tether above horizontal [deg] |
| T | Tether tension | Force along tether [N]. Power = T x v_reel |
| omega_spin | Rotor spin | Angular velocity about axle [rad/s]. From GB4008 eRPM |
| omega | Orbital angular velocity | Angular velocity about axes perpendicular to axle [rad/s] |
| L0 | Tether rest length | Unstretched tether length [m]. Changed by winch |
| theta_col | Collective pitch | Average blade pitch across all blades [rad] |
| theta_lon | Longitudinal cyclic | Swashplate fore/aft tilt [rad] |
| theta_lat | Lateral cyclic | Swashplate left/right tilt [rad] |
| v_winch | Winch speed | Tether length change rate [m/s]. +ve = pay out, -ve = reel in |

---

## 3. Ground Station

### 3.1 Overview

The rotor is a wind-driven autogyrating hub on a tether. Gyroscopic precession causes it
to orbit the anchor point continuously -- this is the natural equilibrium, not a disturbance
to be corrected. Blade pitch is controlled via a swashplate (H3-120 servo layout) using
cyclic and collective, actuated indirectly through Kaman trailing-edge flaps.

Energy extraction follows a pumping cycle (De Schutter 2018):

- **Reel-out:** disk axis tether-aligned (xi ~ 30-55 deg), high collective, high tether
  tension (~200 N). Winch pays out against this tension and drives a generator.
- **Reel-in:** disk tilted to xi = 80 deg, thrust acts mostly upward, tether tension drops
  to ~58 N. Winch reels in at low cost. Net energy per cycle ~ +1735 J.

Control is split across three nodes, all communicating via MAVLink. The ground station runs
the pumping cycle logic and tension PI, receives load cell, encoder, and anemometer data from
the winch controller via MAVLink, and commands winch speed via MAV_CMD_DO_WINCH. The wind meter is hosted on the winch node (co-located physically) and its readings serve as the
seed/fallback for the ground station's orbital wind estimator. The ground station sends one MAVLink message (SET_ATTITUDE_TARGET) to
the Pixhawk at ~10 Hz. The Pixhawk runs two Lua scripts on top of stock ACRO_Heli:
rawes.lua tracks the tether direction and closes the cyclic attitude loop at 50 Hz;
rawes.lua feeds forward motor torque to the GB4008 counter-torque motor at 100 Hz.
No custom firmware is required.

### 3.2 The Natural Orbit

At equilibrium, the hub orbits the anchor point at constant tether length and elevation. The
tether direction defines the equilibrium disk axis (body_z), and as the hub moves the disk
tilts with it, creating a lateral force that drives the orbit. This is the system's natural
resting state and requires zero control effort to maintain.

The orbit is not a problem to be corrected -- it is the expected flight condition. The baseline
attitude setpoint rotates with the orbit automatically (orbit tracking), and the trajectory
planner only needs to command deviations.

> **Sim:** `test_steady_flight.py` (unit) runs the open-loop physics to equilibrium and writes `steady_state_starting.json`. `test_closed_loop_90s.py` (simtest) runs a 90 s orbit using the two-loop attitude controller (RatePID) with no ArduPilot.

### 3.3 Pumping Cycle

**Reel-out (power phase):** The disk is tether-aligned (xi ~= 30-55 deg). High collective
produces high thrust mostly along the tether. High tether tension. The winch pays out against
this tension, driving a generator.

**Reel-in (recovery phase):** The disk tilts to xi=80 deg. Thrust acts almost entirely upward
rather than along the tether. Tether tension drops to ~58 N (mostly gravity component).
The winch reels in cheaply.

Net energy per cycle = (T_out - T_in) x v_reel x t_phase > 0 as long as T_out > T_in.

**DeschutterPlanner** (`simulation/planner.py`) implements this strategy:

```mermaid
flowchart LR
    RO["<b>Reel-Out</b> (Power Phase)<BR/><BR/>body_z tether-aligned<BR/>xi ~ 30-55 deg<BR/>TensionPI -> 200 N<BR/>col_max = 0.10 rad<BR/>col_min = -0.28 rad<BR/>Winch pays out<BR/>-> generator power"]
    TR["<b>Transition</b> (15 s)<BR/><BR/>body_z slews to<BR/>xi = 80 deg from wind<BR/>at 0.40 rad/s"]
    RI["<b>Reel-In</b> (Recovery Phase)<BR/><BR/>body_z at xi = 80 deg<BR/>Thrust acts upward<BR/>col_min = 0.079 rad<BR/>Tether tension ~58 N<BR/>-> cheap reel-in"]

    RO -->|"tether length<BR/>reached"| TR
    TR -->|"slerp<BR/>complete"| RI
    RI -->|"fully<BR/>reeled in"| RO
```

**Why col_min_reel_in = 0.079 rad:** At xi=80 deg, the disk is nearly perpendicular to the
wind. Almost all thrust acts upward (sin(80 deg) ~= 0.985), so even modest collective
maintains altitude. col_min is set just above the collective at which net vertical thrust
equals gravity (zero-altitude-hold point), giving a hard floor below which the TensionPI
cannot push. This is derived from the rotor definition via `col_min_for_altitude_rad()`.

**Stack test results (beaupoil_2026, SkewedWakeBEM, wind=10 m/s East):**

| Config | Reel-out tension | Reel-in tension | Net energy | Peak tension |
|---|---|---|---|---|
| xi=80 deg (production) | 199 N | ~58 N | +1735 J | 455 N |
| xi=55 deg (baseline) | 199 N | 86 N | +1396 J | 455 N |

Peak tension 455 N < 496 N (80% break load limit). Min physics altitude 5.7 m throughout.

> **Sim:** `test_deschutter_cycle.py` (unit) validates a full reel-out/reel-in cycle with `DeschutterPlanner` against the physics model. `test_pumping_cycle.py` (stack) runs end-to-end with ArduPilot SITL -- reel-out 199 N, reel-in ~58 N, net energy +1735 J.

### 3.4 Tension PI Controller

The ground PI runs locally with fresh load cell data:

```
error          = tension_setpoint_n - tension_measured_n   (both in N, local)
collective_rad = kP x error + kI x integral(error) dt
thrust         = clamp((collective_rad - col_min_rad) / (col_max_rad - col_min_rad), 0, 1)
```

col_min_rad / col_max_rad and PI gains are ground-station configuration.

Anti-windup: conditional integration (stop integrating when saturated and error pushes further).
Prevents integral wind-up during kinematic startup.

> **Sim:** `TensionController` in `controller.py`. In unit/simtests it runs against truth-state tension from `tether.py` at 400 Hz. In stack tests `mediator.py` calls it each physics step and forwards the normalized collective as a Ch3 RC override.

### 3.5 Wind Estimation

Wind direction and speed are needed to compute body_z_reel_in.
WindEstimator in planner.py implements all estimation in-loop; DeschutterPlanner
passes state to it on every step and reads back wind_dir_ned /
wind_speed_at(altitude_m) for reel-in body_z computation.

**Direction -- orbital mean position (implemented):**
Over one full orbit the hub's mean horizontal position points downwind.
```
wind_dir_ned = normalize(mean(pos_ned_horizontal))   # over rolling 60 s window
```
No extra hardware. Converges within one orbit (~60 s). Before ready
(< min_samples accumulated) the estimator returns a seed direction supplied
at construction (ground anemometer reading or operator input).

**In-plane speed -- autorotation torque balance (implemented):**
At spin equilibrium, aerodynamic drive torque equals drag torque:
```
v_inplane = omega_spin^2 * K_drag / K_drive
```
K_drive and K_drag are rotor-specific constants (currently tuned for beaupoil_2026).
Full wind speed is recovered from:
```
v_wind = v_inplane / sin(xi)
```
where xi is the angle between body_z (mean over window) and wind_dir_ned.
Exposed as wind_speed_ms (bulk mean) and wind_speed_at(altitude_m) (shear-corrected).

**Altitude-stratified shear and veer (implemented):**
The buffer is altitude-binned (5 m bins, minimum 3 samples/bin, minimum 3 bins
before fitting). Two fits are performed each call:

| Property            | Model                                    | API              |
| ------------------- | ---------------------------------------- | ---------------- |
| shear_alpha         | Power-law: log(v) = alpha*log(z) + const | wind_speed_at(z) |
| veer_rate_deg_per_m | Linear: azimuth = veer_rate*z + const    | wind_dir_at(z)   |

Typically available after 3-5 pumping cycles of altitude excursion. Both models
fall back gracefully to the bulk mean when insufficient data.

Buffer management: rolling 60 s window; hard cap of 600 samples (prevents O(n^2)
at 400 Hz). Per-step cache invalidated on each update() call.

**Ground anemometer (not yet implemented):**
Already supported as the seed/fallback direction -- wiring to actual hardware
just requires passing the live anemometer reading as seed_wind_ned at construction.

**EKF wind state augmentation (future):**
Augment the Pixhawk EKF with a 3D wind vector estimated from tension, velocity,
and omega_spin. If implemented, wind_ned would be added as a NAMED_VALUE_FLOAT
custom field.

> **Sim:** `WindEstimator` in `planner.py`, seeded with the true simulation wind vector (stand-in for the hardware anemometer). `test_wind_estimator.py` validates rolling-window direction and speed. `test_deschutter_wind.py` validates shear-corrected speed across altitude bins.

### 3.6 Winch Controller & MAVLink Protocol

**Pixhawk -> Planner (~10 Hz, standard streams):**

| Standard stream | MAVLink message | What the planner uses |
|---|---|---|
| Position + velocity | LOCAL_POSITION_NED (msg #32) | Hub position and velocity in NED |
| Attitude | ATTITUDE_QUATERNION (msg #31) | Full orientation; body_z_ned = quat_apply(q, [0,0,1]) |
| Rotor spin | ESC_STATUS (msg #291) | rpm[RAWES_CTR_ESC]; planner converts: omega_spin = rpm x 2pi/60 / 11 x 44/80 |

ESC_STATUS is streamed automatically by ArduPilot from AP_ESC_Telem -- no Pixhawk-side code
needed beyond setting the stream rate.

**Planner -> Pixhawk Uplink (~10 Hz):**

Exactly one MAVLink message: SET_ATTITUDE_TARGET.

| Field | Description |
|---|---|
| quaternion | Desired disk orientation in NED frame. Identity [1,0,0,0] = natural tether-aligned orbit (planner silent). Non-identity: body_z_target_ned = quat_apply(q, [0,0,1]). rawes.lua slews toward it at SCR_USER2 (RAWES_BZ_SLEW) rad/s. |
| thrust | Normalized collective [0..1], computed by ground PI from load cell. rawes.lua passes this directly to Ch3 RC override -- no conversion on the Pixhawk. |

**Planner -> Winch Controller (local link):**

| Field | Description |
|---|---|
| winch_speed_ms | Winch rate [m/s]. +ve = pay out, -ve = reel in, 0 = hold. Maps to MAV_CMD_DO_WINCH if the winch controller speaks MAVLink. |

The Pixhawk is not involved in winch control.

**Anchor Position:**

rawes.lua needs the anchor position to compute bz_tether = normalize(pos_hub - anchor)
at every 50 Hz Lua step. The anchor is set via three SCR_USER parameters:

| Parameter | Description |
|---|---|
| SCR_USER3 | Anchor North offset from EKF origin (m) |
| SCR_USER4 | Anchor East offset from EKF origin (m) |
| SCR_USER5 | Anchor Down offset from EKF origin (m) |

These are read at each Lua step. The hub can be anywhere when the script starts -- ground-launched,
hand-launched, or already in the air.

The wind direction does NOT affect the anchor calculation. bz_tether is derived from actual hub
position, so it naturally tracks wherever the hub flies without wind knowledge on the Pixhawk.

---

> **Sim:** `winch_node.py` -- `WinchNode` enforces the protocol boundary. The mediator calls `update_sensors(tension, wind_world)` after `tether.compute()` each step; the planner reads `get_telemetry()` which returns `{tension_n, tether_length_m, wind_ned}` only -- no direct access to `tether._last_info`, `wind_world`, or `WinchController`. Wind seed for `WindEstimator` comes from `Anemometer.measure()` (3 m height), not the raw wind vector. `winch_node.receive_command(speed, dt)` delegates to `WinchController.step()` with tension safety limiting.

## 4. Pixhawk Lua Scripts

### 4.1 rawes.lua -- Flight Subsystem (SCR_USER6=1 or 3)

**SITL Validation Status:**

- test_lua_flight_steady PASSES -- rawes.lua stable orbit 86-110 s, internal_controller=False, no EKF yaw reset.
- test_h_phang PASSES -- confirms H_SW_PHANG=0 and H_SWASH_TYPE=3 correct for RAWES servo
  layout. See section 6 ArduPilot Configuration.

**Parameters (SCR_USER slots):**

| Parameter | SCR_USER | Default | Description |
|---|---|---|---|
| RAWES_KP_CYC | SCR_USER1 | 1.0 | Cyclic P gain -- rad/s per rad of body_z error |
| RAWES_BZ_SLEW | SCR_USER2 | 0.40 | body_z slew rate limit (rad/s) |
| RAWES_ANCHOR_N | SCR_USER3 | 0.0 | Anchor North offset from EKF origin (m) |
| RAWES_ANCHOR_E | SCR_USER4 | 0.0 | Anchor East offset from EKF origin (m) |
| RAWES_ANCHOR_D | SCR_USER5 | 0.0 | Anchor Down offset from EKF origin (m) |
| RAWES_MODE | SCR_USER6 | 0 | Mode selector: plain integer, valid values 0,1,4,5. 0=none (passive), 1=steady (cyclic orbit-tracking, 50 Hz), 2=reserved, 3=reserved, 4=landing (cyclic + VZ descent, 50 Hz), 5=pumping (De Schutter pumping cycle, 50 Hz). Substates for modes 4+5 are sent separately via `NAMED_VALUE_FLOAT("RAWES_SUB", N)` — never encoded in SCR_USER6. |

Cyclic output rate limiter is hardcoded at 30 PWM/step (~0.67 s full-stick traverse).
No further SCR_USER params are available on hardware (firmware exposes only SCR_USER1..6).

Set before flight via MAVLink parameter set or .parm file. No firmware recompilation needed.

**Algorithm Notes:**

**Orbit tracking** applies the same rotation to _bz_eq0 that the tether has made since
equilibrium capture. This keeps body_z tracking the natural tether direction as the hub orbits,
with zero control effort during steady orbit. Port of controller.py::orbit_tracked_body_z_eq().

**Rate-limited slerp** rate-limits body_z transitions (identity->tilted during reel-in) at
SCR_USER2 rad/s (default 0.40 rad/s). During steady orbit the slerp target moves slowly
(orbit angular rate ~0.2 rad/s < 0.40 rad/s slew limit), so the slerp stays locked to orbit
with no perceptible lag.

**Cyclic P loop** converts body_z error to body-frame roll and pitch rates. ACRO's ATC_RAT_RLL/PIT
inner PIDs supply the rate damping. Start SCR_USER1 = 0.3 and increase slowly -- Kaman flap lag
adds phase delay that reduces the stability margin vs. direct blade pitch.

**ACRO_RP_RATE** (ArduPilot parameter, default 360 deg/s) sets the full-stick rate. The scale
factor maps the computed rate command to PWM so the ACRO PID sees the correct physical rate.
If ACRO_RP_RATE is changed, update the constant in rawes.lua.

> **Sim:** The script runs unchanged inside the ArduPilot SITL Docker container. `mediator.py` provides physics via the SITL UDP JSON protocol. `test_lua_flight_steady` (stack) validates rawes.lua stable orbit under full ArduPilot control (internal_controller=False). Unit tests in `simulation/tests/unit/` (`test_math_lua.py`, `test_steady_flight_lua.py`) cover Rodrigues rotation, orbit tracking, slerp, and cyclic projection independently of SITL.

### 4.1.1 rawes.lua -- Landing Mode (SCR_USER6=4)

**Division of labour:**
- Lua (50 Hz): cyclic orbit tracking + VZ collective controller + final_drop detection
- Mediator: LandingPlanner runs the winch (reel-in). No RC override on Ch3 from ground.

**Algorithm:**
1. Gates on `ahrs:healthy()` (GPS position fusion required). Until healthy, returns immediately.
2. On first healthy call: captures `_bz_eq0` (equilibrium body_z) and `_tdir0` (tether direction). Sends "RAWES land: captured" STATUSTEXT.
3. Altitude estimate: `alt_est = anch.z - hub_ned.z` (EKF-frame). Independent of horizontal EKF origin offset because `vel0[2]=0` during kinematic keeps EKF origin Z aligned with launch altitude.
4. Collective: `col_cmd = COL_CRUISE_RAD + KP_VZ * (vz_actual - VZ_LAND_SP)` where `VZ_LAND_SP=0.5 m/s` (positive = descending in NED).
5. Final drop: when `alt_est <= LAND_MIN_TETHER_M=2.0 m`, sets collective=0, sends "RAWES land: final_drop" STATUSTEXT.

**GPS / Lua capture timing:** GPS fuses at t~80 s from mediator start (15 s after kinematic exit at t=65 s). Lua captures 2 s later (~t=82 s). With `internal_controller=False`, the hub is under ArduPilot ACRO with no Ch3 override during the ~17 s gap (kinematic exit t=65 s to Lua capture t=82 s). Hub survival during this gap depends on ArduPilot ACRO trim; this is an open design problem.

**SCR_USER5 (RAWES_ANCHOR_D):** Set to `-pos0[2]` (altitude of hub above EKF origin at kinematic exit) so the Lua altitude estimate matches the physics coordinate. With `vel0[2]=0`, `EKF_ORIGIN.z = pos0[2]`, so `SCR_USER5 = -pos0[2] = physics altitude`.

> **Sim fixture:** `acro_armed_landing_lua` in `conftest.py`. `internal_controller=False`. Hub at xi=80 deg (pos0=[0, 3.47, -19.70], tether_rest_length=20 m). Validates: "RAWES land: captured", "RAWES land: final_drop", floor reached (alt<=2.5 m), tension < 496 N. Stack test: `test_landing_stack.py::test_landing_lua`.

### 4.1.2 rawes.lua -- Pumping Mode (SCR_USER6=5)

**Division of labour:**
- Lua (50 Hz): cyclic orbit tracking + per-phase body_z slerp + per-phase collective
- Ground planner (mediator): DeschutterPlanner owns phase timing and winch commands
- Synchronisation: ground planner sends `NAMED_VALUE_FLOAT("RAWES_SUB", N)` to signal each phase transition. Lua executes the correct collective and body_z slerp for each substate.

**Algorithm:**
1. Gates on `ahrs:healthy()`. Until healthy, returns immediately.
2. On first healthy call: captures `_bz_eq0` and `_tdir0`. Sets `_pump_bz_ri` (reel-in body_z at xi=80 deg from wind). Sends "RAWES pump: bz_ri=..." STATUSTEXT. Starts in "hold" phase.
3. Phase state machine (RAWES_SUB-driven):
   - **hold** (RAWES_SUB=0): reel-out collective, orbit tracking. Sends "RAWES pump: hold" STATUSTEXT.
   - **reel_out** (RAWES_SUB=1): reel-out collective, orbit tracking. Sends "RAWES pump: reel_out" STATUSTEXT.
   - **transition** (RAWES_SUB=2): collective ramps up, body_z slews to xi=80 deg. Sends "RAWES pump: transition".
   - **reel_in** (RAWES_SUB=3): reel-in collective, body_z at xi=80 deg.
   - **transition_back** (RAWES_SUB=4): collective ramps down, body_z slews back to orbit.

> **Sim fixture:** `acro_armed_pumping_lua` in `conftest.py`. `internal_controller=True`. DeschutterPlanner trajectory (t_hold_s=10 s, then reel_out starts). Validates: "RAWES pump: reel_out" STATUSTEXT fires. Stack test: `test_pumping_cycle.py::test_pumping_cycle_lua`.

### 4.2 rawes.lua -- RAWES_ARM: Timed Arm/Disarm (cross-mode)

`NAMED_VALUE_FLOAT("RAWES_ARM", ms)` arms the vehicle and starts a disarm countdown. Re-sending refreshes the timer. Works in any mode.

**Sequence (SITL):**
1. GCS force-arms the vehicle (bypasses SITL-specific prearm failures: motor interlock timing race, ATC_RAT_YAW_P=0, accels inconsistent).
2. GCS sends `NAMED_VALUE_FLOAT("RAWES_ARM", ms)`.
3. Lua receives it, sets Ch3=1000 (throttle low) and Ch8=2000 (motor interlock ON), starts countdown.
4. Once `arming:is_armed()` is true, Lua sends `"RAWES arm-on: armed, expires in Xs"` STATUSTEXT.
5. On expiry: Lua calls `arming:disarm()` and sends `"RAWES arm-on: expired, disarmed"`.

**On hardware:** `arming:arm()` can be called directly from Lua (no force arm needed; hardware prearm checks pass normally). Ch3/Ch8 RC overrides are held for the full duration of the countdown.

**Re-send:** Sending a new `RAWES_ARM` value refreshes the deadline. Sending with `ms > 0` while already armed extends the timer. The "armed" confirmation is only sent once per receive (tracked by `_armon_armed_sent`).

> **SITL fixture:** `torque_armed_lua` — GCS force-arms, sends `RAWES_ARM(3_600_000 ms)`, waits for "RAWES arm-on: armed" confirmation. `torque_unarmed_lua` — yields unarmed; test controls arm timing. Stack test: `test_armon.py` validates regulation (psi_dot < 5 deg/s for 10 s) then expiry (motor to 800 µs, psi_dot > 10 deg/s).

### 4.3 Yaw Regulation — ArduPilot ATC_RAT_YAW (DDFP)

Yaw regulation is handled entirely by ArduPilot's built-in yaw rate PID.  rawes.lua writes
**no** commands to the GB4008 motor.  Ch4 (ACRO yaw rate override) is held at 1500 µs every
tick to prevent integrator wind-up from neutral sticks.

```
Sensing:    gyro.z (from EKF attitude, ACRO_Heli)
Control:    ATC_RAT_YAW P/I/D -> tail-channel PWM (SERVO4, H_TAIL_TYPE=4 DDFP CCW)
Actuator:   GB4008 motor via standard PWM on MAIN OUT 4 (output 4, IOMCU)
```

**H_TAIL_TYPE enum** (AP_MotorsHeli_Single — relevant values only):

| Value | Name | Output mapping | Notes |
|-------|------|----------------|-------|
| 0 | Servo | Bidirectional — SERVO4 centred at SERVO4_TRIM (1500 µs); PID ±1 maps to servo range | No sign flip; used for conventional tail rotor servos |
| 1 | Servo+ExtGyro | Same as 0 but with external heading-hold gyro on Ch7 | Not used |
| 2 | DDFP | Unidirectional motor; bidirectional PWM mapping | Not used |
| 3 | DDFP CW | Unidirectional motor, CW rotation; positive PID → more throttle | **Wrong for GB4008** — see below |
| 4 | DDFP CCW | Unidirectional motor, CCW rotation; applies sign flip (`×−1`) before thrust mapping | **Correct for GB4008** |

**Why CCW (4) and not CW (3):**
CW hub drift → positive psi_dot → yaw error = 0 − positive = **negative** PID output.
CCW sign flip maps −PID → +throttle → GB4008 spins up → CCW counter-torque opposes drift. ✓
Type 3 (no sign flip): −PID → clamps to 0 → motor stays off → drift uncorrected. ✗

Pass criterion (stack): `test_yaw_regulation.py` — max |psi_dot| < 5 deg/s over the
last 20 s after a 75 s SITL settle period.

> **Future work:** A Lua feedforward trim (`throttle ≈ omega_rotor × GEAR_RATIO / RPM_SCALE`)
> could reduce the load on ATC_RAT_YAW by pre-commanding the back-EMF equilibrium.
> Not currently implemented; mode 2 is reserved for this purpose.

### 4.4 Channel Ownership

| Channel | Owner | Rate | Path |
|---|---|---|---|
| Ch1 -- roll rate | rawes.lua | 50 Hz | body_z error (roll) -> ATC_RAT_RLL PID -> swashplate |
| Ch2 -- pitch rate | rawes.lua | 50 Hz | body_z error (pitch) -> ATC_RAT_PIT PID -> swashplate |
| Ch3 -- collective | ground PI via MAVLink (modes 1-3) OR rawes.lua (modes 4-5) OR RAWES_ARM | ~10 Hz / 50 Hz / 100 Hz | Modes 1-3: normalized thrust [0..1] from load cell -> Ch3 RC override. Modes 4 (landing) and 5 (pumping): Lua owns Ch3 entirely. RAWES_ARM active: Lua holds Ch3=1000 (throttle low) for the full countdown duration. |
| Ch8 -- motor interlock | rawes.lua (RAWES_ARM active) | 100 Hz | Lua holds Ch8=2000 (interlock ON, motor enabled) for the full countdown duration. Released on disarm. |
| SERVO4 -- GB4008 throttle | ArduPilot (H_TAIL_TYPE=4 DDFP) | 400 Hz | ATC_RAT_YAW PID output -> SERVO4 (MAIN OUT 4, DShot300). rawes.lua does NOT write to SERVO4. |

### 4.5 Simulation Mapping

| Lua component                            | Python equivalent                                    | File               |
| ---------------------------------------- | ---------------------------------------------------- | ------------------ |
| rawes.lua equilibrium capture     | _body_z_eq0, _tether_dir0 at free-flight start       | mediator.py        |
| rawes.lua orbit tracking          | orbit_tracked_body_z_eq()                            | controller.py      |
| rawes.lua rate-limited slerp      | Rate-limited slerp in mediator inner loop            | mediator.py        |
| rawes.lua cyclic P loop           | compute_swashplate_from_state()                      | controller.py      |
| ACRO ATC_RAT_RLL/PIT (rate damping)      | RatePID(kp=2/3) inner loop                           | controller.py      |
| Ch3 collective (from ground RC override) | TensionController PI output -> normalized collective | controller.py      |
| ArduPilot ATC_RAT_YAW (yaw regulation)   | torque_model.py hub yaw ODE + mediator_torque.py     | mediator_torque.py |
| ESC_STATUS rpm (planner reads)           | SpinSensor.measure() -- models AM32 eRPM jitter      | sensor.py          |

---

## 5. Yaw / Torque Compensation

### 5.1 The Problem

The RAWES rotor (blades + outer hub shell) spins freely in autorotation driven by wind.
The stationary inner assembly -- flight controller, battery, servo linkages -- is mounted
on bearings inside the spinning shell. The inner assembly must maintain a fixed heading
while the outer shell spins; otherwise the tether wraps and heading control is lost.

### 5.2 Actuator: GB4008 + 80:44 Gear

**Motor:** EMAX GB4008, 66 KV, 90T, 24N22P brushless gimbal motor (hollow shaft)
**ESC:**   REVVitRC 50A AM32, 3-6S, servo PWM / DSHOT, firmware AM32

The motor stator is fixed to the stationary inner assembly. The motor rotor is geared to the
spinning outer rotor hub via an **80:44 spur gear** (hub side has 80 teeth; motor pinion has
44 teeth). The motor therefore spins at `80/44 ~= 1.82x` the rotor hub speed.

The ESC maintains the commanded motor RPM proportional to commanded PWM, drawing whatever
current is needed to hold that speed. Bearing drag and swashplate friction only affect
**power consumption** — the ESC compensates for them and they have no direct effect on yaw
dynamics. The inner assembly remains at a fixed heading when motor RPM equals the hub rotation rate
times the gear ratio. Yaw drift occurs when RPM deviates: too low and the inner assembly
rotates with the hub (CW); too high and it counter-rotates against it (CCW).

The ESC targets the motor shaft at a speed proportional to commanded PWM, but the motor
shaft speed responds with a first-order lag (MOTOR_TAU = 20 ms, representing ESC + motor
electrical/mechanical inertia):

```
d(omega_motor)/dt = (throttle x RPM_SCALE - omega_motor) / MOTOR_TAU
```

The gear coupling is instantaneous — once omega_motor is known, the inner assembly yaw rate
follows directly:

```
psi_dot = omega_rotor - omega_motor / GEAR_RATIO
```

There is no torque equation and no hub inertia term. Bearing drag and swashplate friction only
affect how much current the ESC draws to hold the commanded RPM — they have no effect on
yaw dynamics.

**Equilibrium throttle** (psi_dot = 0 at steady state, omega_motor = throttle x RPM_SCALE):

```
throttle_eq = omega_rotor x GEAR_RATIO / RPM_SCALE
            = 28 x 1.818 / 105 ~= 0.485  (48.5%)
```

Model parameters:

| Symbol     | Value           | Source                                    |
| ---------- | --------------- | ----------------------------------------- |
| RPM_SCALE  | 105 rad/s       | GB4008 66KV x 15.2V (4S LiPo)            |
| GEAR_RATIO | 80/44 ~= 1.818  | Motor pinion faster than rotor hub        |
| MOTOR_TAU  | 0.02 s          | Typical small BLDC + ESC step response    |

**Gear Efficiency Analysis** (omega_rotor = 28 rad/s):

| Gear ratio     | Motor speed % | eq_throttle | Headroom |
| -------------- | ------------- | ----------- | -------- |
| 1.0  (no gear) | 26.7%         | 26.7%       | 73%      |
| 1.818 (80:44)  | 48.5%         | 48.5%       | 52%      |
| 3.191 (max)    | 85.0%         | 85.0%       | 15%      |

The current 80:44 gear leaves 52% throttle headroom for disturbances. A higher ratio
approaches full throttle at equilibrium, leaving no headroom.

### 5.3 Control Architecture

Yaw regulation uses a single loop:

1. **ArduPilot ATC_RAT_YAW PID (ACRO_Heli):** ACRO mode commands a desired yaw rate (zero
   with neutral stick). The yaw rate PID outputs a tail-channel command (SERVO4, H_TAIL_TYPE=4
   DDFP) that drives the GB4008 anti-rotation motor.  rawes.lua holds Ch4 at 1500 µs (neutral)
   every tick; the DDFP mapping sends the PID output directly to SERVO9 (output 9, DShot).

**Biased throttle mapping in SITL:**

`mediator_torque.py` maps the tail-channel PWM to motor throttle with a biased curve so that
1500 µs corresponds exactly to the equilibrium trim throttle:

```
pwm <= 1500 us:  throttle = trim x (pwm - 1000) / 500
pwm >  1500 us:  throttle = trim + (1 - trim) x (pwm - 1500) / 500
trim = equilibrium_throttle(omega_rotor, params) ~= 0.485
```

This pre-biases the operating point so the PID corrects deviations rather than driving the
motor from zero to equilibrium.  On hardware the same bias should be encoded in the ESC
throttle curve calibration.

### 5.4 Key Parameters

| Parameter       | Value  | Purpose                                |
| --------------- | ------ | -------------------------------------- |
| ARMING_SKIPCHK  | 0xFFFF | Skip all pre-arm checks                |
| H_RSC_MODE      | 1      | CH8 passthrough -- instant runup       |
| H_TAIL_TYPE     | 4      | DDFP CCW — sign-flip maps negative yaw error to positive GB4008 throttle |
| H_COL2YAW       | 0      | No collective->yaw feedforward         |
| ATC_RAT_YAW_P   | 0.20   | Starting P gain (SITL validated)       |
| ATC_RAT_YAW_I   | 0.05   | Corrects residual yaw not cancelled by PID |
| ATC_RAT_YAW_D   | 0.0    | Start at zero                          |
| ATC_RAT_YAW_IMAX| 0      | No integrator windup                   |

### 5.5 Hardware Tuning

Four-step procedure for commissioning the GB4008 on the physical rotor:

1. **Verify equilibrium throttle:** At nominal autorotation RPM, command motor until
   psi_dot = 0 with no PID active. Record the throttle percentage. It should be close to
   `omega_rotor x GEAR_RATIO / RPM_SCALE` (~48.5% at 28 rad/s). Any deviation indicates
   the RPM_SCALE constant needs updating (e.g. different battery voltage or motor KV).

3. **Set trim:** Update `--trim-throttle` (mediator) or H_TRIM_THROTTLE (hardware) to
   the measured equilibrium throttle so neutral stick = exact equilibrium.

4. **Tune ATC_RAT_YAW_P:** Increase from 0.001 until yaw disturbance rejection is fast
   enough without oscillation. The trim handles steady state; P only corrects transients.
   With the GB4008 at R=7.5 ohm, the stability limit at 400 Hz is P ~= 2.7.

### 5.6 Power Budget

The GB4008 at equilibrium draws approximately:

```
I_eq ~= 0.53 A
P_eq ~= 6 W
```

The 4S 450 mAh LiPo provides roughly 450 mAh / 0.53 A ~= 50 minutes of continuous
anti-rotation at nominal autorotation RPM -- well within flight duration limits.

---

> **Sim:** Full physics model in `simulation/torque_model.py` (hub yaw ODE, GB4008 motor, RK4 integrator). `simulation/mediator_torque.py` runs a standalone SITL co-simulation with configurable RPM profiles (constant, slow/fast sinusoidal, gust, pitch-roll). 9 unit tests + 6 stack tests all pass.

## 6. ArduPilot Configuration

### 6.1 Scripting Parameters

| Parameter    | Value | Reason                                                                    |
| ------------ | ----- | ------------------------------------------------------------------------- |
| SCR_ENABLE   | 1     | Enable Lua scripting subsystem                                            |
| SCR_USER1    | 1.0   | RAWES_KP_CYC -- cyclic P gain; start at 0.3                               |
| SCR_USER2    | 0.40  | RAWES_BZ_SLEW -- body_z slew rate (rad/s)                                 |
| SCR_USER3..5 | 0.0   | Anchor N/E/D offsets (m) -- set to anchor NED from EKF origin             |
| SCR_USER6    | 0/1/4/5 | RAWES_MODE -- mode selector: 0=none, 1=steady, 4=landing, 5=pumping    |

### 6.2 Swashplate and RSC

| Parameter | Value | Reason |
|-----------|-------|--------|
| FRAME_CLASS | 6 (Heli) | Traditional helicopter frame |
| H_SWASH_TYPE | 3 (H3_120, default) | 3-servo lower ring at 120 deg driving 4 push-rods |
| H_RSC_MODE | 1 (CH8 passthrough) | Wind-driven rotor -- instant runup_complete |
| H_SW_PHANG | 0 (confirmed) | Empirically verified: cross-coupling 1.5% (roll) and 19.7% (pitch). ArduPilot built-in +90 deg roll advance angle in H3_120 formula already aligns with RAWES servo layout (S1=0/East, S2=120, S3=240 deg). |
| H_COL_MAX | TBD (~0.10 rad) | Limit collective to keep flap loads in linear regime |
| H_CYC_MAX | TBD | Limit cyclic amplitude to <=15 deg rotor tilt |
| SERVO1_FUNCTION | 33 (Motor1 / S1) | Swashplate servo S1 |
| SERVO2_FUNCTION | 34 (Motor2 / S2) | Swashplate servo S2 |
| SERVO3_FUNCTION | 35 (Motor3 / S3) | Swashplate servo S3 |
| ATC_RAT_RLL_IMAX | 0 | Prevent orbital angular rate integrator windup |
| ATC_RAT_PIT_IMAX | 0 | Same |
| ATC_RAT_YAW_IMAX | 0 | Same |
| ACRO_TRAINER | 0 | Disable leveling trainer (equilibrium is 65 deg from vertical) |
| ACRO_RP_RATE | 360 | Must match constant in rawes.lua |

**Why ACRO mode, not STABILIZE or GUIDED:** The hub has no passive stability -- the tether attaches 0.3 m below the centre of mass (inverted pendulum configuration), and for a spinning rotor the tether restoring moment causes gyroscopic precession at 90 deg rather than attitude restoration. Active cyclic is the only stabilisation mechanism. ACRO mode converts RC rate commands into cyclic without any attitude-restoration toward level. rawes.lua supplies the continuous rate commands (orbit-tracking angular rates) needed to keep body_z at the correct fixed angular offset from the tether as the hub orbits; ACRO's inner PIDs convert those into the cyclic that holds the hub at its natural 65 deg tilt. No cyclic is applied to drive the hub back toward roll=0/pitch=0. STABILIZE and GUIDED command cyclic to drive roll=0/pitch=0 (level) and crash within seconds. Do not switch to STABILIZE. See `theory/orbit_mechanics.md §5.3` for the full stability physics analysis.

### 6.3 GB4008 Anti-Rotation Motor

| Parameter       | Value       | Reason                                                               |
| --------------- | ----------- | -------------------------------------------------------------------- |
| H_TAIL_TYPE     | 4 (DDFP CCW) | Routes yaw rate PID to SERVO4 with CCW sign flip (see §4.3)       |
| SERVO4_FUNCTION | 36 (Motor4) | GB4008 ESC                                                           |
| SERVO4_MIN      | 1000        | ESC disarm                                                           |
| SERVO4_MAX      | 2000        | ESC maximum                                                          |
| SERVO4_TRIM     | ~1150       | Idle torque at rest                                                  |
| ATC_RAT_YAW_P   | 0.20        | Starting value (feedforward sets counter-rotation)                   |
| ATC_RAT_YAW_I   | 0.05        | Corrects residual yaw rate not cancelled by feedforward trim         |
| ATC_RAT_YAW_D   | 0.0         | Start at zero                                                        |
| H_COL2YAW       | TBD         | Feedforward: collective changes alter drag -> GB4008 must compensate |

### 6.4 Kaman Flap Lag

RAWES blade pitch changes via aerodynamic moment on the flap, not direct mechanical linkage.
This adds a second-order lag in the cyclic response. SCR_USER1 (KP_CYC) is the primary tuning
lever -- start at 0.3 and increase slowly. The ATC_RAT_RLL/PIT_D term provides damping.

### 6.5 Known Gaps and Risks

| Risk                                       | Impact                                          | Mitigation                                                                                 |      |                                                 |
| ------------------------------------------ | ----------------------------------------------- | ------------------------------------------------------------------------------------------ | ---- | ----------------------------------------------- |
| H_SW_PHANG cyclic phase                    | Resolved -- H_SW_PHANG=0 confirmed              | test_h_phang measured cross_ch1=1.5%, cross_ch2=19.7% (both <20%).                         |      |                                                 |
| Kaman flap lag                             | Medium -- phase margin loss                     | Start KP_CYC=0.3; increase slowly; D-term in ATC_RAT_RLL/PIT damps oscillation             |      |                                                 |
| Load cell hardware (tension feedback)      | High -- critical path                           | Validate ground PI in simulation before hardware; load cell must be on winch before flight |      |                                                 |
| Orbit tracking before first tether tension | Medium -- no tether direction during free climb | Equilibrium capture guard (                                                                | diff | < 0.5 m) prevents tracking until tether is taut |
| Planner timeout during reel-in tilt        | Low -- automatic fallback                       | 2 s timeout snaps slerp goal back to _bz_orbit (natural orbit)                             |      |                                                 |
| ACRO_RP_RATE mismatch                      | Medium -- wrong rate scaling                    | Constant in rawes.lua must match ArduPilot ACRO_RP_RATE parameter                   |      |                                                 |
| GB4008 direction (H_TAIL_DIR)              | Medium -- yaw runaway                           | Verify on bench: hub spinning CCW -> motor must apply CW torque                            |      |                                                 |

---

> **Sim:** Parameters are applied via `gcs.py` (`set_param`) over MAVLink before arming. The Docker image bakes defaults into `copter-heli.parm` (SCR_ENABLE, H_RSC_MODE, etc.). Stack test fixtures in `conftest.py` apply the full required parameter set automatically at fixture startup.

## 7. Takeoff & Landing

### 7.1 Takeoff

Physical sequence:
1. Ground station spins rotor to omega_spin >= omega_min (~10-15 rad/s). Planner monitors STATE.
2. Release mechanism drops rotor. Lift > weight -> rapid climb.
3. Tether pays out. Once taut, tension develops and lateral stability begins.
4. Natural orbit establishes. Planner begins pumping cycle.

Phase ownership:

| Phase | Planner | Pixhawk (Lua + ACRO) |
|---|---|---|
| Spin-up | Monitor omega_spin; trigger release at omega >= omega_min | None |
| Free climb (slack tether) | Pay out winch; send explicit attitude_q target | rawes.lua holds attitude; Ch3 from thrust |
| Tether catch | Detect tension > threshold (local winch read); reduce thrust | Orbit tracking begins once _eq_captured |
| Transition to pumping | Ramp to operating tension setpoint | Natural orbit; follow SET_ATTITUDE_TARGET |

During the slack-tether phase, _eq_captured is false so orbit tracking is suppressed until
EKF position is valid and tether direction is meaningful.

### 7.2 Landing -- Vertical Descent

**Key constraint:** the rotor disk must be horizontal (body_z pointing up, NED [0,0,-1]) at
touchdown or the blade tips strike the ground.  A tilted disk during the pumping orbit is fine;
during final approach it is not.

**Geometry insight:** at the end of the reel-in phase, body_z is already at xi=80 deg from the
wind direction.  Because the East wind is horizontal, that means the disk is only ~10 deg from
horizontal -- leveling is nearly instant.  The tether, now nearly vertical, supports >95% of the
hub weight throughout the descent, making the tether a passive altitude-hold mechanism rather than
a source of tension spikes.

**Why not a spiral descent?**  As the tether shortens during a tethered orbit the hub speeds up
(figure-skater effect).  At short tether lengths the orbital speed exceeds the reel-in rate and
the tether goes slack, causing tension spikes and oscillation.  A vertical drop directly above the
anchor avoids this entirely.

**Landing sequence (validated in simulation, test_pump_and_land.py):**

```
Step 1 -- End of reel-in (normal pumping):
    Planner: complete reel-in to REST_LENGTH (~50 m). Body_z at xi~80 deg.
    Lua: standard reel-in orbit tracking.  Hub is ~49 m above anchor.

Step 2 -- Leveling (<1 s):
    Planner: send attitude_q targeting BZ_LEVEL = [0,0,-1] NED.
    Lua: slerp body_z toward [0,0,-1] at body_z_slew_rate_rad_s (0.40 rad/s).
    Because xi=80 deg -> only 10 deg from horizontal, leveling completes in ~1 s.
    Switch to descent rate controller (see below).

Step 3 -- Vertical descent (~32 s from 50 m):
    Planner: winch reel in at V_LAND = 1.5 m/s.
    Planner: descent rate controller for collective:
        vz_error = hub_vel_z - V_LAND    (NED; +ve = too fast)
        collective = clamp(COL_CRUISE + KP_VZ * vz_error, col_min, col_max)
        COL_CRUISE = 0.079 rad  (col_min_reel_in at xi=80 deg)
        KP_VZ      = 0.05 rad/(m/s)
    Lua: body_z_eq fixed at [0,0,-1]; hover gains (kp=1.5, kd=0.5).
    Tether pendulum effect centres hub above anchor during descent.

Step 4 -- Final drop (tether <= MIN_TETHER_M = 2 m):
    Planner: collective = 0; winch hold.
    Hub drops last ~2 m onto catch device.
    No flare needed -- hub is already directly above anchor.
```

**Responsibility split (same protocol -- no new COMMAND fields):**

| Step | Planner (ground) | Lua / ACRO (Pixhawk) |
|---|---|---|
| Leveling | attitude_q = bz_level quaternion; descent rate ctrl | slerp body_z; hover kp/kd; Ch3 thrust |
| Descent | V_LAND winch command; descent rate ctrl | body_z_eq = [0,0,-1]; hover kp/kd |
| Final drop | collective = 0; winch hold | hold last attitude |

**Simulation results (test_landing.py -- isolated landing from xi=80 deg, 20 m tether):**

| Metric | Value |
|---|---|
| Max tension (leveling + descent) | 403 N  (<620 N break load) |
| Min altitude during descent | 1.92 m  (floor = 1.0 m) |
| Anchor distance at touchdown | 0.51 m |
| Disk tilt at touchdown | 2.2 deg |
| Touchdown speed (vz) | 0.00 m/s |
| Total time | 12.3 s |

**Simulation results (test_pump_and_land.py -- full pumping cycle then land from ~50 m):**

| Metric | Value |
|---|---|
| Net pumping energy | +1857 J |
| Max tension during landing | 359 N |
| Anchor distance at touchdown | 0.69 m |
| Disk tilt at touchdown | 2.0 deg |
| Touchdown speed (vz) | 0.00 m/s |
| Total sequence time | 92 s |

**Why the descent rate controller instead of TensionPI:**
TensionPI reacts to tension error.  When the hub descends faster than the reel-in rate the tether
goes slack, TensionPI sees near-zero tension, commands max collective, tether snaps taut again --
classic oscillation.  The descent rate controller reacts to hub vz directly (from
LOCAL_POSITION_NED) and keeps descent at V_LAND regardless of tether state.

---

> **Sim:** Landing is validated in `tests/unit/test_landing.py` (isolated from xi=80 deg, 20 m)
> and `tests/unit/test_pump_and_land.py` (full pumping cycle + landing from ~50 m).
> Both run without ArduPilot (simtest mark).  Stack tests still start with the hub already
> airborne at the equilibrium position, held kinematically for 45 s while the EKF converges.

## Appendix A. Algorithm Detail

### A.1 Planner Logic (~10 Hz)

```
Each step:

1. Read standard streams:
        pos_ned + vel_ned  <- LOCAL_POSITION_NED
        body_z_ned         <- ATTITUDE_QUATERNION  (quat_apply(q, [0,0,1]))
        omega_spin         <- ESC_STATUS[RAWES_CTR_ESC].rpm x 2pi/60 / 11 x 44/80
2. Read tension_n from Winch Controller (local link -- no MAVLink hop)

3. Determine phase (reel-out / reel-in) from local elapsed time since release

4. Run tension PI (local, fresh data):
       error          = tension_setpoint_n - tension_n
       collective_rad = kP x error + kI x integral(error) dt
       thrust         = clamp((collective_rad - col_min_rad) / (col_max_rad - col_min_rad), 0, 1)

5. Compute:
       attitude_q     <- identity during reel-out (planner silent)
                         quat_from_vectors([0,0,1], body_z_reel_in_ned) during reel-in
                         body_z_reel_in_ned = cos(xi)*wind_dir_ned + sin(xi)*[0,0,-1]
                         ([0,0,-1] = up in NED)
       winch_speed_ms <- +v_reel_out or -v_reel_in

6. Send SET_ATTITUDE_TARGET (attitude_q + thrust) -> Pixhawk
7. Send winch_speed_ms                             -> Winch Controller
```

### A.2 50 Hz Control Loop

```mermaid
flowchart TD
    START(["Every 20 ms"]) --> GUARD{"ACRO mode?<BR/>vehicle:get_mode() == 1"}
    GUARD -- No --> SKIP(["return early"])
    GUARD -- Yes --> READ["<b>2. Read state</b><BR/>bz_now = ahrs:body_to_earth([0,0,1])<BR/>pos_ned = ahrs:get_relative_position_NED_origin()<BR/>diff = pos_ned - anchor(SCR_USER3/4/5)"]

    READ --> CAPT{"Equilibrium<BR/>captured?"}
    CAPT -- "No, |diff| >= 0.5 m" --> CAPTURE["<b>3. Capture equilibrium</b> (once)<BR/>_bz_eq0   = bz_now<BR/>_tdir0    = diff / |diff|<BR/>_bz_slerp = _bz_eq0"]
    CAPTURE --> ORBIT
    CAPT -- Yes --> ORBIT["<b>4. Orbit tracking</b><BR/>bz_tether = diff / |diff|<BR/>axis  = _tdir0 x bz_tether<BR/>angle = atan2(|axis|, _tdir0 . bz_tether)<BR/>_bz_orbit = Rodrigues(_bz_eq0, axis/|axis|, angle)"]

    ORBIT --> TIMEOUT{"Planner silent<BR/>> 2 s?"}
    TIMEOUT -- Yes --> CLEAR["<b>5. Clear _bz_target</b><BR/>revert to natural orbit"]
    TIMEOUT -- No --> SLERP
    CLEAR --> SLERP["<b>6. Rate-limited slerp</b><BR/>goal = _bz_target or _bz_orbit<BR/>step = min(SCR_USER2 * dt, remain)<BR/>_bz_slerp = Rodrigues(_bz_slerp, axis, step)"]

    SLERP --> CYCLIC["<b>7. Cyclic P loop</b><BR/>err_ned  = bz_now x _bz_slerp<BR/>err_body = ahrs:earth_to_body(err_ned)<BR/>roll_rate  = SCR_USER1 * err_body.x<BR/>pitch_rate = SCR_USER1 * err_body.y"]

    CYCLIC --> RC["<b>8. RC override</b><BR/>scale = 500 / (ACRO_RP_RATE * pi/180)<BR/>Ch1 = clamp(1500 + scale * roll_rate,  1000, 2000)<BR/>Ch2 = clamp(1500 + scale * pitch_rate, 1000, 2000)"]

    RC --> RLIMIT["<b>9. Output rate limiter</b> (hardcoded 100 PWM/step)<BR/>Ch1 = prev_ch1 + clamp(Ch1-prev_ch1, -delta, +delta)<BR/>Ch2 = prev_ch2 + clamp(Ch2-prev_ch2, -delta, +delta)<BR/>rc:get_channel(1):set_override(Ch1)<BR/>rc:get_channel(2):set_override(Ch2)"]
    RLIMIT --> START
```

Note on Rodrigues: Lua's Vector3f operator overloading (* , +) is not available in this ArduPilot
build. rodrigues() in the actual script uses explicit component arithmetic. See
simulation/scripts/rawes.lua for the exact implementation.

### A.3 Yaw Regulation

Yaw regulation is handled by ArduPilot's ATC_RAT_YAW PID (not by rawes.lua).
rawes.lua holds Ch4 at 1500 µs (neutral) every tick to prevent ACRO integrator wind-up.
The PID output goes to the tail channel (SERVO4, H_TAIL_TYPE=4 DDFP) which drives
the GB4008 anti-rotation motor via DShot300 on output 9.

```
Sensing:   gyro.z -> ATC_RAT_YAW PID
Output:    SERVO4 (tail channel, H_TAIL_TYPE=4)
Actuator:  GB4008 motor via DShot300 (SERVO9 output 9, SERVO_BLH_MASK=256)
```

Equilibrium throttle for reference (not used by Lua):
  throttle_eq = omega_rotor x GEAR_RATIO / RPM_SCALE ~= 0.485 at 28 rad/s

---

## Appendix B. Startup & Arming

### B.1 Working SITL Arm Sequence

**Direct arm (non-Lua tests):** Confirmed working (tested test_arm_minimal.py, ArduPilot 4.7+):

```python
# Parameters that must be set before arming:
params = {
    "ARMING_SKIPCHK": 0xFFFF,  # skip ALL pre-arm checks (4.7+ renamed from ARMING_CHECK)
    "H_RSC_MODE":     1,        # CH8 passthrough -- instant runup_complete
    "FS_THR_ENABLE":  0,        # no RC throttle failsafe
    "FS_GCS_ENABLE":  0,        # no GCS heartbeat failsafe
    "COMPASS_USE":    0,        # velocity-derived yaw only
}

# Sequence:
# 1. Set params above
# 2. Wait for ATTITUDE messages (EKF attitude aligned)
# 3. Send CH8=2000 (motor interlock ON, RSC at setpoint for mode 1)
# 4. Send force arm (param2=21196)
# 5. HEARTBEAT shows armed=True immediately (mode 1 has instant runup_complete)
```

Note: In ArduPilot 4.7+ the parameter was renamed from ARMING_CHECK to ARMING_SKIPCHK.
Setting ARMING_CHECK silently fails -- no ACK, no error. Always use ARMING_SKIPCHK.

**RAWES_ARM Lua timer (Lua torque tests):** Used when rawes.lua must own Ch3/Ch8 and manage the disarm countdown.

```python
# Additional params required in _LUA_TORQUE_EXTRA_PARAMS:
#   "ARMING_CHECK": 0   -- disable prearm checks so the fixture's force-arm is not
#                          fighting the motor-interlock mandatory check in some AP builds
#   "H_RSC_RUNUP_TIME": 2  -- must be > H_RSC_RAMP_TIME (default 1) to pass prearm

# Sequence:
# 1. GCS force-arms the vehicle (bypasses SITL prearm failures)
# 2. GCS sends NAMED_VALUE_FLOAT("RAWES_ARM", ms)
# 3. Lua receives it, holds Ch3=1000 + Ch8=2000, starts countdown timer
# 4. Lua sends "RAWES arm-on: armed, expires in Xs" STATUSTEXT (once arming:is_armed())
# 5. On timer expiry: Lua calls arming:disarm(), sends "RAWES arm-on: expired, disarmed"
# Re-sending RAWES_ARM refreshes the deadline without re-arming.
```

### B.2 Common Failure Modes

| Symptom | Cause | Fix |
|---------|-------|-----|
| "PreArm: Motors: H_RSC_MODE invalid" | H_RSC_MODE=0 (SITL default) | Set H_RSC_MODE=1 |
| COMMAND_ACK ACCEPTED but HEARTBEAT never armed | RSC not at runup_complete | Use H_RSC_MODE=1 + CH8=2000 |
| "PreArm: Motor Interlock Enabled" | CH8=2000 at arm time with checks enabled | Set ARMING_CHECK=0 |
| EKF_STATUS stuck at 0x0400 | EKF in constant-position mode, GPS not locked | Normal during freeze; arm with ARMING_CHECK=0 |
| EKF_STATUS flashes 0x0000 | EKF reinitialising after compass switch | Wait for 0x0000 to pass before arming |
| Vehicle arms but no servo outputs on CH1-3 | Not in a flying mode | Set ACRO mode after arm |

### B.3 Parameter Reference

| Parameter      | Recommended value | Reason                                                                             |
| -------------- | ----------------- | ---------------------------------------------------------------------------------- |
| ARMING_SKIPCHK | 0xFFFF            | Skip all pre-arm checks for SITL (4.7+ name; old name ARMING_CHECK silently fails) |
| H_RSC_MODE     | 1                 | CH8 passthrough -- instant runup_complete                                          |
| COMPASS_USE    | 0                 | Velocity-derived yaw only (no hardware compass in SITL)                            |
| FS_THR_ENABLE  | 0                 | No RC throttle failsafe (SITL has no real RC)                                      |
| FS_GCS_ENABLE  | 0                 | No GCS heartbeat failsafe                                                          |
| INITIAL_MODE   | 1                 | Boot into ACRO (takes effect on next boot, so also set mode explicitly)            |

---

## Appendix C. ArduPilot Internals

### C.1 How ArduCopter Helicopter Arming Works

ArduCopter Helicopter has a two-stage arm that confuses people who are used to multirotor:

1. **Pre-arm checks** -- AP_Arming_Copter::arm() runs checks controlled by ARMING_CHECK.
   - force=True (param2=21196 in MAV_CMD_COMPONENT_ARM_DISARM) bypasses these.
   - ARMING_CHECK=0 disables all checks (including in the non-force path).

2. **Motor armed state** -- AP_MotorsHeli::output() runs on every loop and resets
   _flags.armed=false if is_armed_and_runup_complete() returns false.
   **This is NOT bypassed by force arm.** COMMAND_ACK will say ACCEPTED but HEARTBEAT
   armed bit will be false until the RSC has completed its runup.

The HEARTBEAT armed flag = AP_Notify::flags.armed = motors->armed() = _flags.armed.
So you cannot be "armed" in the HEARTBEAT sense until the RSC is happy.

### C.2 RSC Modes

| Mode | Name | Runup behaviour | Notes |
|------|------|-----------------|-------|
| 0 | Disabled | N/A | **INVALID** -- causes "PreArm: Motors: H_RSC_MODE invalid" and immediate auto-disarm |
| 1 | CH8 Passthrough | Immediate -- RSC = CH8 | No ramp; runup_complete fires instantly when CH8 is high. **Use this for SITL.** |
| 2 | Setpoint | Ramp to H_RSC_SETPOINT | Requires H_RUNUP_TIME (minimum > 0); motor interlock must be ON after arm |
| 3 | Throttle Curve | Ramp via throttle curve | Similar runup wait as mode 2 |
| 4 | External Governor | External feedback | Requires RPM telemetry from ESC |

**SITL default is 0 (invalid).** Always set H_RSC_MODE explicitly.

### C.3 Motor Interlock

CH8 PWM controls the "motor interlock" -- a safety gate that allows/prevents motor operation:

| CH8 value | Interlock state | RSC behaviour |
|-----------|----------------|---------------|
| 1000 | LOW (disabled) | RSC output = 0 / no runup |
| 2000 | HIGH (enabled) | RSC can run |

STATUSTEXT messages:
- "RC8: MotorInterlock HIGH" -- CH8 crossed HIGH threshold
- "RC8: MotorInterlock LOW" -- CH8 crossed LOW threshold
- "PreArm: Motor Interlock Enabled" -- CH8 is HIGH at arm time (pre-arm check)

### C.4 EKF Initialization Sequence

The EKF goes through these phases during SITL startup:

| Time | EKF_STATUS flags | Event |
|------|-----------------|-------|
| t=0 | 0x0400 (CONST_POS_MODE) | EKF initialising in constant-position mode |
| t+2s | 0x0400 | "EKF3 IMU0 switching to compass 1" -- EKF resets for compass init |
| t+3s | 0x0000 (all zero) | Brief re-init flash -- DO NOT ARM HERE |
| t+3s | 0x0000 | "EKF3 IMU0 tilt alignment complete" |
| t+3s | 0x0000 | "EKF3 IMU0 MAG1 initial yaw alignment complete" |
| t+3s | 0x00a7 | EKF has: attitude + velocity (horiz+vert) + vert position |
| t+17s* | 0x00a7 | "EKF3 IMU0 origin set" -- GPS origin fixed |

* "origin set" fires ~14 s after first arm command was sent (GPS detection trigger).
**Do not wait for "origin set" before arming** -- it may not fire until after arm.

0x00a7 decodes as:
- 0x0001 EKF_ATTITUDE [PASS]
- 0x0002 EKF_VELOCITY_HORIZ [PASS]
- 0x0004 EKF_VELOCITY_VERT [PASS]
- 0x0020 EKF_POS_VERT_ABS [PASS]
- 0x0080 EKF_PRED_POS_HORIZ_REL [PASS]
- (no EKF_POS_HORIZ_REL/ABS -- normal with COMPASS_USE=0 + no GPS lock)

**Safe to arm** when EKF_STATUS >= 0x00a7 (or any non-zero, non-0x0400).

---

## Appendix D. EKF3 GPS Position Fusion

Analysed from /ardupilot/build/sitl/bin/arducopter-heli (ArduCopter V4.8.0-dev).

### D.1 The GPS Position Fusion Gate

EKF3 transitions from AID_NONE (= CONST_POS_MODE) to AID_ABSOLUTE (= GPS position fusing)
only when readyToUseGPS() returns true (Control.cpp:594):

```cpp
bool NavEKF3_core::readyToUseGPS(void) const {
    if (frontend->sources.getPosXYSource(core_index) != AP_NavEKF_Source::SourceXY::GPS)
        return false;
    return validOrigin
        && tiltAlignComplete
        && yawAlignComplete
        && (delAngBiasLearned || assume_zero_sideslip())
        && gpsGoodToAlign
        && gpsDataToFuse;
}
```

All six conditions must be simultaneously true.

### D.2 Condition: gpsGoodToAlign (10-second mandatory delay)

calcGpsGoodToAlign() (VehicleStatus.cpp) runs each EKF loop. On its very first call, it sets
lastGpsVelFail_ms=now regardless of check results. After 10 s, gpsGoodToAlign becomes true.

**EK3_GPS_CHECK=0 masks all individual quality checks** (HDOP, sats, speed accuracy, drift, yaw
error, etc.) so they all pass immediately. But **the 10-second clock still runs** from the first
GPS check. There is no parameter to shorten this delay.

**Practical consequence:** GPS position fusion cannot happen until at least **11-12 s** after
SITL launch, even with EK3_GPS_CHECK=0. Stack test timeouts must account for this.

### D.3 Condition: yawAlignComplete (THE critical gate for RAWES setup)

yawAlignComplete starts false and is set only when a yaw measurement successfully fuses.
The source is determined by EK3_SRC1_YAW:

| EK3_SRC1_YAW | SourceYaw value | What it uses |
|---|---|---|
| 0 | NONE | No yaw -- position fusion never starts |
| 1 | COMPASS | SITL simulated magnetometer |
| 2 | GPS | GPS velocity heading -- **default for heli** |
| 3 | GPS_COMPASS_FALLBACK | GPS velocity with compass fallback |
| 6 | EXTNAV | External navigation |
| 8 | GSF | Gaussian Sum Filter (from GPS velocity) |

**EK3_SRC1_YAW=2 (default) FAILS with near-zero velocity.** Both GPS velocity heading and GSF
require a meaningful velocity vector. With velocity near zero, the computed heading is
noise-dominated and yawAlignComplete stays false, so GPS position never fuses.

**Fix: EK3_SRC1_YAW=1 (COMPASS).** SITL simulates the magnetometer based on the synthetic earth
field at the launch location. Yaw aligns within ~1-2 s after tiltAlignComplete.

### D.4 Condition: delAngBiasLearned

ArduCopter/ArduHeli **never calls set_fly_forward(true)** (only ArduPlane does). Therefore
assume_zero_sideslip()=false for helicopter, and delAngBiasLearned is **required**.

**Convergence timing (measured in SITL):**
- With zero motion: P does not converge -- bias fully unobservable.
- With yaw-only rotation (gyro=[0,0,5 deg/s], attitude flat): delAngBiasLearned becomes true
  ~37 s from SITL start.
- With multi-axis oscillation: biases converge faster but cause dead-reckoning drift -> EKF
  unhealthy. Do not add real roll/pitch tilt to the stub.

**Practical consequence:** Stack tests must allow >= 40 s before expecting GPS position fusion.
The binding gate is usually delAngBiasLearned, not gpsGoodToAlign (which clears at t~=19 s).

### D.5 Required Parameters for GPS Position Fusion

| Parameter | Required value | Why |
|---|---|---|
| EK3_SRC1_POSXY | 3 (GPS) | GPS as horizontal position source (default OK) |
| EK3_SRC1_VELXY | 3 (GPS) | GPS as horizontal velocity source (default OK) |
| EK3_SRC1_YAW | **1 (COMPASS)** | Compass yaw -- default=2 (GPS velocity) never works at low speed |
| EK3_GPS_CHECK | **0** | Mask all GPS quality checks (SITL GPS has no real quality fields) |
| COMPASS_USE | **1** | Enable compass for EKF yaw fusion |
| COMPASS_ENABLE | **1** | Enable compass sensor |
| EK3_MAG_CAL | 0 (Never) or 3 (Always) | 0=use raw compass immediately |

Parameters that **do NOT exist** in this build (4.8.0-dev heli):
- EK3_GPS_CTRL -- not compiled; GPS position+velocity fusion is always on when EK3_SRC1_POSXY=3

### D.6 Timing Requirements (stack test design)

**Stationary GPS test (yaw-only stub, EK3_SRC1_YAW=1):**

| Event | Time from SITL start |
|---|---|
| EKF3 tilt alignment | ~3-4 s |
| Compass yaw alignment (EK3_SRC1_YAW=1) | ~4-5 s |
| GPS detected (SITL JSON backend) | ~8-9 s |
| gpsGoodToAlign=true (10 s delay from GPS detect) | ~18-19 s |
| validOrigin=true + GPS data in buffer | ~19 s |
| delAngBiasLearned=true (yaw-only stub motion) | **~37-42 s** |
| GPS position fusion starts | **~41 s** |

**Real RAWES mediator (kinematic_vel_ramp_s=15, startup_damp_seconds=65, EK3_SRC1_YAW=1):**

Empirically measured from passing stack tests (acro_armed_lua, test_lua_flight_steady):

| Event | Time from mediator start |
|---|---|
| EKF3 tilt alignment | ~4-5 s |
| Compass yaw alignment | ~5-6 s |
| GPS detected | ~9 s |
| GPS origin set ("EKF3 IMU0 origin set") | **~14-25 s** |
| Kinematic exit (hub transitions to free flight) | **65 s** |
| GPS position fusion ("EKF3 IMU0 is using GPS") | **~80 s** (~15 s after kinematic exit) |
| Lua capture ("RAWES flight: captured" / "RAWES land: captured") | **~82 s** (~2 s after GPS fusion) |

GPS fusion fires ~15 s after kinematic exit, NOT during kinematic. The velocity taper
(kinematic_vel_ramp_s=15: vel ramps to 0 in the last 15 s of kinematic) causes the EKF
predicted position to converge toward GPS position near kinematic exit, enabling fusion.

**CRITICAL:** Do NOT override kinematic_vel_ramp_s=0 in Lua fixtures. With constant
velocity (vel_ramp=0), the EKF predicted position drifts monotonically away from GPS
position -- innovations exceed the gate and GPS is rejected for the entire test duration.

**Stack tests:** startup_damp_seconds=65 s (current default) is correct. GPS fuses ~15 s
after kinematic exit, not during kinematic. For Lua modes (SCR_USER6=4/5), this means Lua
captures at t~82 s. With internal_controller=True (pumping fixture), the hub orbits stably
during the ~17 s gap and Lua can capture cleanly. With internal_controller=False (landing
fixture), the hub is uncontrolled for ~17 s -- this is an open design problem.

Required param set for GPS position fusion in real stack tests:
```
EK3_SRC1_YAW  = 1   (compass, NOT GPS velocity)
EK3_MAG_CAL   = 0   (Never -- use raw compass immediately)
COMPASS_USE   = 1
COMPASS_ENABLE= 1
EK3_GPS_CHECK = 0
```

### D.7 CONST_POS_MODE Bit Meaning

EKF_STATUS_REPORT.flags bit 7 (0x0080) = const_pos_mode:

```cpp
status.flags.const_pos_mode = (PV_AidingMode == AID_NONE) && filterHealthy;
```

This is set when the EKF is healthy (tilt + yaw aligned, filterHealthy=true) but has no
position/velocity aiding source (AID_NONE). It clears when readyToUseGPS() returns true.

Seeing 0x00a7 = bits {0,1,2,5,7} means:
- 0x0001 att [PASS], 0x0002 horiz_vel [PASS], 0x0004 vert_vel [PASS], 0x0020 vert_pos_abs [PASS]
- 0x0080 CONST_POS_MODE -- stuck in AID_NONE, GPS position not yet fusing

### D.8 Heli-Specific Behaviour Differences from Copter

1. **fly_forward=false** -- copter/heli never sets fly_forward, so assume_zero_sideslip()=false.
   This means delAngBiasLearned is always required (cannot be bypassed).

2. **Default EK3_SRC1_YAW=2** (GPS velocity heading) -- works for fixed-wing (always moving
   forward) but fails for hover-capable vehicles at low/zero speed.

3. **EK3_MAG_CAL=3 (ALWAYS)** -- more aggressive than fixed-wing default. The EKF tries to
   learn magnetometer biases in-flight always. Does not block yaw alignment itself, but
   wasLearningCompass_ms check can temporarily suppress compass yaw if COMPASS_LEARN=2 is active.

---

## Appendix E. Lua API Constraints

These constraints apply to rawes.lua (and any future Lua scripts) running on the
ArduPilot SITL Docker image and on the Pixhawk 6C with the same firmware.

| What you'd expect | What actually works |
|---|---|
| ahrs:get_rotation_body_to_ned() | Doesn't exist. Use ahrs:body_to_earth(v) and ahrs:earth_to_body(v) |
| Vector3f(x, y, z) | Constructor ignores args (warning + wrong value). Use Vector3f() then :x()/:y()/:z() setters |
| v:normalized() | Doesn't exist. Copy then :normalize() in-place: local r = v3_copy(v); r:normalize(); return r |
| vec * scalar or vec + vec | * not overloaded; + may silently fail. Use component arithmetic directly |
| rc:set_override(chan, pwm) | Doesn't exist. Correct API: rc:get_channel(n):set_override(pwm) (cache channel at module load) |
| ArduCopter ACRO mode = 6 | ACRO = **1**. Mode 6 is RTL. Use vehicle:get_mode() == 1 |

**SCR_ENABLE bootstrap:** In this build `SCRIPTING_ENABLE_DEFAULT=0` (compiled-in). The startup
sequence is: (1) AP_Param reads EEPROM -- if EEPROM has SCR_ENABLE=0 (compiled-in default, stored
on first boot), scripting.init() sees 0 and Lua does NOT start. (2) load_defaults_file() applies
copter-heli.parm (SCR_ENABLE=1) -- this write lands in EEPROM via _timer_tick(). (3) On the NEXT
boot, EEPROM has SCR_ENABLE=1 so scripting.init() starts Lua.

Consequence: Lua is unavailable on the FIRST boot from a fresh EEPROM. All subsequent boots start
Lua automatically. The EEPROM is NOT wiped between test runs; copter-heli.parm + rawes_sitl_defaults
both carry SCR_ENABLE=1 and self-heal EEPROM after one failing boot.

**"RAWES flight: loaded" STATUSTEXT timing:** Lua sends this message at module load (~1 s after
SITL starts). The GCS connects ~4 s after SITL starts. The message is dropped before the GCS has
any active link -- it will NEVER appear in STATUSTEXT polling. Do NOT use "loaded" as a readiness
signal. Instead poll for the periodic "RAWES: ch1=..." diagnostic messages (sent every ~5 s,
_diag % 250 == 1 at 50 Hz) or wait for "RAWES flight: captured" in the observation window.

**Anchor position in LOCAL_POSITION_NED:** After the NED migration, initial_state["pos"][2] is
NED Z (negative ~= -7.12 for altitude 7.12 m above ground). The anchor in LOCAL_POSITION_NED
frame is at [0, 0, -initial_state["pos"][2]] = +7.12 m Down from EKF origin. So
SCR_USER5 = -home_z_enu (negate).

---

## Appendix F. Files & References

### Files

| File                                         | Description                                                                   |
| -------------------------------------------- | ----------------------------------------------------------------------------- |
| simulation/scripts/rawes.lua                 | Unified Lua controller (SCR_USER6: 0=none, 1=steady, 4=landing, 5=pumping; RAWES_ARM named float for timed arm/disarm; yaw via ArduPilot ATC_RAT_YAW) |
| simulation/scripts/torque/lua_defaults.parm  | SITL param overrides for Lua torque tests                                     |
| simulation/torque_model.py                   | Hub yaw kinematics — HubParams, HubState, step(), equilibrium_throttle() |
| simulation/mediator_torque.py                | Standalone torque SITL mediator (RPM profiles, hub yaw physics)               |
| simulation/controller.py                     | orbit_tracked_body_z_eq(), compute_swashplate_from_state(), TensionController |
| simulation/mediator.py                       | Rate-limited slerp, STATE/COMMAND packet assembly                             |
| simulation/sensor.py                         | SpinSensor (omega_spin noise model)                                           |
| hardware/design.md                           | Swashplate geometry, servo specs, power architecture                          |

To deploy to hardware: copy both .lua files to APM/scripts/ on the SD card.

### References

- ArduPilot Traditional Helicopter docs: https://ardupilot.org/copter/docs/traditional-helicopter-connecting-apm.html
- ArduPilot Lua scripting API: https://ardupilot.org/dev/docs/lua-scripts.html
- EKF3 source: libraries/AP_NavEKF3/ in ArduPilot repository
