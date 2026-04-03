# RAWES -- Flight Control Stack Reference

Complete reference for the deployed RAWES flight control system: ground planner, winch
controller, Pixhawk Lua scripts, ArduPilot configuration, and startup/arming procedures.

---

## 1. System Architecture

Three nodes, two communication boundaries. The Pixhawk runs two distinct loops at different
rates -- the 400 Hz ArduPilot main loop (ACRO_Heli) and the Lua scripting scheduler (50/100 Hz).
Lua writes RC overrides that the main loop consumes at full rate.

```
+-------------------------------------------------------------+
|  Ground Station                                             |
|                                                             |
|  Trajectory Planner (~10 Hz)     Winch Controller          |
|  Phase logic, Tension PI,        Reel speed PID            |
|  Wind estimation                 Load cell, Encoder        |
+--+----------------------------------------------------------+
   |                                     ^
   | SET_ATTITUDE_TARGET                 | tension_n (local)
   | (attitude_q + thrust, MAVLink)      | winch_speed_ms
   | LOCAL_POSITION_NED, ATTITUDE_QUATERNION, ESC_STATUS
   v
+-------------------------------------------------------------+
|  Pixhawk 6C (in air)                                        |
|                                                             |
|  rawes_flight.lua   50 Hz                                   |
|    Orbit tracking, rate-limited slerp                       |
|    Cyclic P loop, Ch1/Ch2/Ch3 RC overrides                  |
|                                                             |
|  rawes_yaw_trim.lua  100 Hz                                 |
|    Yaw trim and correction, Ch4 RC override                 |
|                                                             |
|  ACRO_Heli  400 Hz (ArduPilot main loop)                    |
|    Rate PIDs, H3-120 mix, spool guards, servo PWM           |
+-------------------------------------------------------------+
```

**Key design principles:**

- **Natural orbit is free.** Lua orbit-tracking tracks the tether direction at 50 Hz without
  planner involvement. The planner only intervenes to request a specific disk orientation.
- **Inner loops stay on the Pixhawk.** Attitude tracking (cyclic), body_z slewing, and
  counter-torque control run inside Lua scripts at 50-100 Hz. No custom firmware required --
  both scripts run on top of stock ACRO_Heli mode.
- **Winch is on the ground.** The Pixhawk is never involved in winch control.
- **Thrust field = normalized collective.** The thrust field of SET_ATTITUDE_TARGET carries
  normalized collective [0..1] from the ground PI. The Lua script forwards it directly to
  Ch3 RC override -- no tension awareness on the Pixhawk.

**Tether tension** is measured at the base station, not on the hub. A load cell on the winch
drum measures exactly the right quantity for energy accounting (work = T_base x v_reel).
The base-to-hub tension difference is the tether weight component along the tether
(0.3-3.1 N at operating lengths -- under 5% of operating range; absorbed by PI integrator).

---

## 2. Ground Planner

### 2.1 Glossary

| Term | Meaning |
|---|---|
| body_z | Unit vector along the rotor axle (spin axis) |
| Orbit tracking | Pixhawk-side control that rotates attitude setpoint to match hub orbital position |
| NED | North-East-Down coordinate frame (X=North, Y=East, Z=Down). Up is [0,0,-1] |

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

### 2.3 The Natural Orbit

At equilibrium, the hub orbits the anchor point at constant tether length and elevation. The
tether direction defines the equilibrium disk axis (body_z), and as the hub moves the disk
tilts with it, creating a lateral force that drives the orbit. This is the system's natural
resting state and requires zero control effort to maintain.

The orbit is not a problem to be corrected -- it is the expected flight condition. The baseline
attitude setpoint rotates with the orbit automatically (orbit tracking), and the trajectory
planner only needs to command deviations.

### 2.4 Pumping Cycle State Machine (De Schutter 2018)

**Reel-out (power phase):** The disk is tether-aligned (xi ~= 30-55 deg). High collective
produces high thrust mostly along the tether. High tether tension. The winch pays out against
this tension, driving a generator.

**Reel-in (recovery phase):** The disk tilts so that xi increases toward 90 deg. Thrust acts
mostly upward rather than along the tether. Tether tension drops to near the gravity component
alone (~15-30 N). The winch reels in cheaply.

Net energy per cycle = (T_out - T_in) x v_reel x t_phase > 0 as long as T_out > T_in.

**DeschutterPlanner** (`simulation/planner.py`) implements this strategy:

```
Reel-out: body_z tether-aligned, TensionPI targets tension_out (200 N)
          -> high tether tension -> winch generates power
          col_min = -0.28 rad (safe above zero-thrust at -0.34 rad)

Transition (t_transition=15 s): body_z slews to xi=55 deg from wind at 0.40 rad/s

Reel-in: body_z at xi=55 deg, TensionPI targets tension_in (>=80 N for altitude maintenance)
         -> thrust acts upward, not along tether -> low aerodynamic resistance
         col_min = -0.20 rad (safe above zero-thrust at -0.228 rad for xi=55 deg)
```

**Why tension_in >= 80 N:** At xi=55 deg, vertical thrust component is collective-dependent.
With col_min=-0.20, thrust ~= 70-100 N vertically. Hub mass is 5 kg -> gravity 49 N. At low
collective the hub would fall -- tension_in must be high enough to keep TensionPI outputting
adequate collective.

**Stack test results (beaupoil_2026, SkewedWakeBEM, wind=10 m/s East):**
- Reel-out mean tension: 199 N
- Reel-in steady mean tension: 86 N
- Net energy: +1396 J per cycle
- Peak tension: 455 N (< 496 N = 80% break load limit)
- Min physics altitude: 5.7 m throughout cycle

**High-tilt De Schutter (xi=80 deg reel-in):** Net energy improves to approximately +1735 J/cycle
(+24%), with reel-in tension dropping to ~58 N. Requires col_max=0.10 rad and
col_min_reel_in=0.079 rad.

### 2.5 Tension PI Controller

The ground PI runs locally with fresh load cell data:

```
error          = tension_setpoint_n - tension_measured_n   (both in N, local)
collective_rad = kP x error + kI x integral(error) dt
thrust         = clamp((collective_rad - col_min_rad) / (col_max_rad - col_min_rad), 0, 1)
```

col_min_rad / col_max_rad and PI gains are ground-station configuration.

Anti-windup: conditional integration (stop integrating when saturated and error pushes further).
Prevents integral wind-up during kinematic startup.

### 2.6 Wind Estimation

Wind direction and speed are needed to compute body_z_reel_in. Four methods:

**Method 1 -- Rotor spin rate (in-plane speed):**
Using the autorotation torque balance: v_inplane = omega_spin^2 x K_drag / K_drive.
v_wind ~= v_inplane / sin(xi) where xi comes from Method 2.

**Method 2 -- Orbital mean position (direction):**
Over one orbit, wind_dir ~= normalize(mean(pos_ned_horizontal)).
No extra hardware. Direction converges within one orbit (~60 s).

**Method 3 -- Ground anemometer (future):**
3D ultrasonic anemometer at ground station, extrapolated to hub altitude via log profile.
No protocol impact -- the planner reads it locally.

**Method 4 -- EKF wind state augmentation (future):**
Augment the Pixhawk EKF with a 3D wind vector estimated from tension, velocity, and omega_spin.

### 2.7 Planner Logic (~10 Hz)

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

---

## 3. Winch Controller

### 3.1 MAVLink Protocol

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
| quaternion | Desired disk orientation in NED frame. Identity [1,0,0,0] = natural tether-aligned orbit (planner silent). Non-identity: body_z_target_ned = quat_apply(q, [0,0,1]). rawes_flight.lua slews toward it at SCR_USER2 (RAWES_BZ_SLEW) rad/s. |
| thrust | Normalized collective [0..1], computed by ground PI from load cell. rawes_flight.lua passes this directly to Ch3 RC override -- no conversion on the Pixhawk. |

**Planner -> Winch Controller (local link):**

| Field | Description |
|---|---|
| winch_speed_ms | Winch rate [m/s]. +ve = pay out, -ve = reel in, 0 = hold. Maps to MAV_CMD_DO_WINCH if the winch controller speaks MAVLink. |

The Pixhawk is not involved in winch control.

### 3.2 Anchor Position

rawes_flight.lua needs the anchor position to compute bz_tether = normalize(pos_hub - anchor)
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

## 4. Pixhawk -- Orbit Tracker (rawes_flight.lua)

### 4.1 SITL Validation Status

- test_lua_flight_rc_overrides PASSES -- script loads in SITL, captures equilibrium at t~=0.5 s
  after ACRO arm, generates cyclic RC overrides (max cyclic activity 227 PWM).
- test_h_swash_phang PASSES -- confirms H_SW_PHANG=0 and H_SWASH_TYPE=3 correct for RAWES servo
  layout. See section 6 ArduPilot Configuration.

### 4.2 Parameters (SCR_USER slots)

| Parameter | SCR_USER | Default | Description |
|---|---|---|---|
| RAWES_KP_CYC | SCR_USER1 | 1.0 | Cyclic P gain -- rad/s per rad of body_z error |
| RAWES_BZ_SLEW | SCR_USER2 | 0.40 | body_z slew rate limit (rad/s) |
| RAWES_ANCHOR_N | SCR_USER3 | 0.0 | Anchor North offset from EKF origin (m) |
| RAWES_ANCHOR_E | SCR_USER4 | 0.0 | Anchor East offset from EKF origin (m) |
| RAWES_ANCHOR_D | SCR_USER5 | 0.0 | Anchor Down offset from EKF origin (m) |
| RAWES_MAX_CYC_DELTA | SCR_USER6 | 30 | Max cyclic PWM change per 20 ms step. 30 PWM/step = 1500 PWM/s ~= 0.67 s to traverse full stick. 0 = disabled. |

SCR_USER7..8 are reserved (future: reel-in tilt angle, gain scheduling).

Set before flight via MAVLink parameter set or .parm file. No firmware recompilation needed.

### 4.3 50 Hz Control Loop

```
Each step (every 20 ms):

1. Guard: only run in ACRO mode (mode 1 in ArduCopter). Return early otherwise.

2. Read state:
       bz_now  <- ahrs:body_to_earth(Vector3f([0,0,1]))   -- body_z in NED
       pos_ned <- ahrs:get_relative_position_NED_origin()
       anchor  <- Vector3f(SCR_USER3, SCR_USER4, SCR_USER5)
       diff    <- pos_ned - anchor

3. Capture equilibrium (once, when |diff| >= 0.5 m):
       _bz_eq0   <- bz_now                    (body_z at capture)
       _tdir0    <- diff / |diff|              (tether direction at capture)
       _bz_slerp <- _bz_eq0                   (initialise rate-limited setpoint)

4. Orbit tracking (each step):
       bz_tether <- diff / |diff|
       axis  <- _tdir0 x bz_tether
       angle <- atan2(|axis|, _tdir0 . bz_tether)
       _bz_orbit <- Rodrigues(_bz_eq0, axis/|axis|, angle)

5. Planner timeout (2 s since last SET_ATTITUDE_TARGET):
       clear _bz_target -> revert to natural orbit

6. Rate-limited slerp:
       goal      <- _bz_target or _bz_orbit
       remain    <- acos(_bz_slerp . goal)
       step      <- min(SCR_USER2 x dt, remain)
       _bz_slerp <- Rodrigues(_bz_slerp, (_bz_slerp x goal)/|...|, step)

7. Cyclic:
       err_ned  <- bz_now x _bz_slerp          (world-frame error)
       err_body <- ahrs:earth_to_body(err_ned)  (body-frame error)
       err_bx   <- err_body.x                   (roll)
       err_by   <- err_body.y                   (pitch)
       roll_rate  <- SCR_USER1 x err_bx         (rad/s)
       pitch_rate <- SCR_USER1 x err_by         (rad/s)

8. RC override (ACRO_RP_RATE deg/s = +-500 us):
       scale <- 500 / (ACRO_RP_RATE x pi/180)
       Ch1 PWM <- clamp(1500 + scale x roll_rate,  1000, 2000)
       Ch2 PWM <- clamp(1500 + scale x pitch_rate, 1000, 2000)

9. Output rate limiter (SCR_USER6 = RAWES_MAX_CYC_DELTA, default 30 PWM/step):
       Ch1 PWM <- prev_ch1 + clamp(Ch1 - prev_ch1, -max_delta, +max_delta)
       Ch2 PWM <- prev_ch2 + clamp(Ch2 - prev_ch2, -max_delta, +max_delta)
       rc:get_channel(1):set_override(Ch1)
       rc:get_channel(2):set_override(Ch2)
```

Note on Rodrigues: Lua's Vector3f operator overloading (* , +) is not available in this ArduPilot
build. rodrigues() in the actual script uses explicit component arithmetic. See
simulation/scripts/rawes_flight.lua for the exact implementation.

### 4.4 Algorithm Notes

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
If ACRO_RP_RATE is changed, update the constant in rawes_flight.lua.

### 4.5 Channel Ownership

| Channel | Owner | Rate | Path |
|---|---|---|---|
| Ch1 -- roll rate | rawes_flight.lua | 50 Hz | body_z error (roll) -> ATC_RAT_RLL PID -> swashplate |
| Ch2 -- pitch rate | rawes_flight.lua | 50 Hz | body_z error (pitch) -> ATC_RAT_PIT PID -> swashplate |
| Ch3 -- collective | ground PI via MAVLink | ~10 Hz | Normalized thrust [0..1] from load cell -> Ch3 RC override |
| Ch4 -- yaw rate | rawes_yaw_trim.lua | 100 Hz | Torque trim + correction -> ATC_RAT_YAW -> GB4008 |

### 4.6 Simulation Mapping

| Lua component | Python equivalent | File |
|---|---|---|
| rawes_flight.lua equilibrium capture | _body_z_eq0, _tether_dir0 at free-flight start | mediator.py |
| rawes_flight.lua orbit tracking | orbit_tracked_body_z_eq() | controller.py |
| rawes_flight.lua rate-limited slerp | Rate-limited slerp in mediator inner loop | mediator.py |
| rawes_flight.lua cyclic P loop | compute_swashplate_from_state() | controller.py |
| ACRO ATC_RAT_RLL/PIT (rate damping) | RatePID(kp=2/3) inner loop | controller.py |
| Ch3 collective (from ground RC override) | TensionController PI output -> normalized collective | controller.py |
| rawes_yaw_trim.lua trim + correction | mediator_torque.py compute_trim + Kp | mediator_torque.py |
| ESC_STATUS rpm (planner reads) | SpinSensor.measure() -- models AM32 eRPM jitter | sensor.py |

---

## 5. Pixhawk -- Yaw Trim (rawes_yaw_trim.lua)

The counter-torque script is already validated (15/15 tests pass). Full documentation in
simulation/torque/README.md. Summary:

```
motor_rpm  <- battery:voltage(0)   [SITL: mediator encodes RPM as voltage]
           or RPM:get_rpm(0)       [hardware: DSHOT telemetry from AM32]

trim       = tau_bearing / (tau_stall x (1 - omega_motor/omega_0))
           ~= 0.747 at nominal 28 rad/s axle speed

yaw_corr   = -Kp_yaw x gyro:z()   [Kp_yaw = 0.001]

throttle   = clamp(trim + yaw_corr, 0, 1)
Ch4 PWM    <- 1000 + throttle x 1000
```

The trim feedforward handles steady-state bearing drag. The Kp_yaw correction handles transient
disturbances. ArduPilot's ATC_RAT_YAW is still active and provides additional correction on top
of the Lua trim.

---

## 6. ArduPilot Configuration

### 6.1 Scripting Parameters

| Parameter | Value | Reason |
|---|---|---|
| SCR_ENABLE | 1 | Enable Lua scripting subsystem |
| SCR_USER1 | 1.0 | RAWES_KP_CYC -- cyclic P gain; start at 0.3 |
| SCR_USER2 | 0.40 | RAWES_BZ_SLEW -- body_z slew rate (rad/s) |
| SCR_USER3..5 | 0.0 | Anchor N/E/D offsets (m) -- set to anchor NED from EKF origin |
| SCR_USER6 | 30 | RAWES_MAX_CYC_DELTA -- max cyclic PWM change per 20 ms step; 0 = disabled |

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
| ACRO_RP_RATE | 360 | Must match constant in rawes_flight.lua |

### 6.3 GB4008 Anti-Rotation Motor

| Parameter | Value | Reason |
|-----------|-------|--------|
| H_TAIL_TYPE | 4 (DDFP) | Assigns yaw rate PID output to SERVO4 |
| SERVO4_FUNCTION | 36 (Motor4) | GB4008 ESC |
| SERVO4_MIN | 1000 | ESC disarm |
| SERVO4_MAX | 2000 | ESC maximum |
| SERVO4_TRIM | ~1150 | Idle torque at rest |
| ATC_RAT_YAW_P | 0.20 | Starting value (trim handles steady state) |
| ATC_RAT_YAW_I | 0.05 | Absorbs steady-state bearing friction |
| ATC_RAT_YAW_D | 0.0 | Start at zero |
| H_COL2YAW | TBD | Feedforward: collective changes alter drag -> GB4008 must compensate |

### 6.4 Kaman Flap Lag

RAWES blade pitch changes via aerodynamic moment on the flap, not direct mechanical linkage.
This adds a second-order lag in the cyclic response. SCR_USER1 (KP_CYC) is the primary tuning
lever -- start at 0.3 and increase slowly. The ATC_RAT_RLL/PIT_D term provides damping.

### 6.5 Known Gaps and Risks

| Risk | Impact | Mitigation |
|------|--------|-----------|
| H_SW_PHANG cyclic phase | Resolved -- H_SW_PHANG=0 confirmed | test_h_swash_phang measured cross_ch1=1.5%, cross_ch2=19.7% (both <20%). |
| Kaman flap lag | Medium -- phase margin loss | Start KP_CYC=0.3; increase slowly; D-term in ATC_RAT_RLL/PIT damps oscillation |
| Load cell hardware (tension feedback) | High -- critical path | Validate ground PI in simulation before hardware; load cell must be on winch before flight |
| Orbit tracking before first tether tension | Medium -- no tether direction during free climb | Equilibrium capture guard (|diff| < 0.5 m) prevents tracking until tether is taut |
| Planner timeout during reel-in tilt | Low -- automatic fallback | 2 s timeout snaps slerp goal back to _bz_orbit (natural orbit) |
| ACRO_RP_RATE mismatch | Medium -- wrong rate scaling | Constant in rawes_flight.lua must match ArduPilot ACRO_RP_RATE parameter |
| GB4008 direction (H_TAIL_DIR) | Medium -- yaw runaway | Verify on bench: hub spinning CCW -> motor must apply CW torque |

---

## 7. Startup and Arming Sequence

### 7.1 How ArduCopter Helicopter Arming Actually Works

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

### 7.2 RSC Modes (H_RSC_MODE parameter)

| Mode | Name | Runup behaviour | Notes |
|------|------|-----------------|-------|
| 0 | Disabled | N/A | **INVALID** -- causes "PreArm: Motors: H_RSC_MODE invalid" and immediate auto-disarm |
| 1 | CH8 Passthrough | Immediate -- RSC = CH8 | No ramp; runup_complete fires instantly when CH8 is high. **Use this for SITL.** |
| 2 | Setpoint | Ramp to H_RSC_SETPOINT | Requires H_RUNUP_TIME (minimum > 0); motor interlock must be ON after arm |
| 3 | Throttle Curve | Ramp via throttle curve | Similar runup wait as mode 2 |
| 4 | External Governor | External feedback | Requires RPM telemetry from ESC |

**SITL default is 0 (invalid).** Always set H_RSC_MODE explicitly.

### 7.3 Motor Interlock (CH8)

CH8 PWM controls the "motor interlock" -- a safety gate that allows/prevents motor operation:

| CH8 value | Interlock state | RSC behaviour |
|-----------|----------------|---------------|
| 1000 | LOW (disabled) | RSC output = 0 / no runup |
| 2000 | HIGH (enabled) | RSC can run |

STATUSTEXT messages:
- "RC8: MotorInterlock HIGH" -- CH8 crossed HIGH threshold
- "RC8: MotorInterlock LOW" -- CH8 crossed LOW threshold
- "PreArm: Motor Interlock Enabled" -- CH8 is HIGH at arm time (pre-arm check)

### 7.4 Working SITL Arm Sequence

Confirmed working (tested test_arm_minimal.py, ArduPilot 4.7+):

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
# 2. Wait for ATTITUDE messages (EKF attitude aligned -- gcs.wait_ekf_attitude())
# 3. Send CH8=2000 (motor interlock ON, RSC at setpoint for mode 1)
# 4. Send force arm (param2=21196)
# 5. HEARTBEAT shows armed=True immediately (mode 1 has instant runup_complete)
```

Note: In ArduPilot 4.7+ the parameter was renamed from ARMING_CHECK to ARMING_SKIPCHK.
Setting ARMING_CHECK silently fails -- no ACK, no error. Always use ARMING_SKIPCHK.

### 7.5 EKF Initialization Sequence (SITL JSON backend)

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

### 7.6 Common Failure Modes

| Symptom | Cause | Fix |
|---------|-------|-----|
| "PreArm: Motors: H_RSC_MODE invalid" | H_RSC_MODE=0 (SITL default) | Set H_RSC_MODE=1 |
| COMMAND_ACK ACCEPTED but HEARTBEAT never armed | RSC not at runup_complete | Use H_RSC_MODE=1 + CH8=2000 |
| "PreArm: Motor Interlock Enabled" | CH8=2000 at arm time with checks enabled | Set ARMING_CHECK=0 |
| EKF_STATUS stuck at 0x0400 | EKF in constant-position mode, GPS not locked | Normal during freeze; arm with ARMING_CHECK=0 |
| EKF_STATUS flashes 0x0000 | EKF reinitialising after compass switch | Wait for 0x0000 to pass before arming |
| Vehicle arms but no servo outputs on CH1-3 | Not in a flying mode | Set ACRO mode after arm |

### 7.7 Parameter Reference

| Parameter | Recommended value | Reason |
|-----------|------------------|--------|
| ARMING_SKIPCHK | 0xFFFF | Skip all pre-arm checks for SITL (4.7+ name; old name ARMING_CHECK silently fails) |
| H_RSC_MODE | 1 | CH8 passthrough -- instant runup_complete |
| COMPASS_USE | 0 | Velocity-derived yaw only (no hardware compass in SITL) |
| FS_THR_ENABLE | 0 | No RC throttle failsafe (SITL has no real RC) |
| FS_GCS_ENABLE | 0 | No GCS heartbeat failsafe |
| INITIAL_MODE | 1 | Boot into ACRO (takes effect on next boot, so also set mode explicitly) |

---

## 8. EKF3 GPS Position Fusion

Analysed from /ardupilot/build/sitl/bin/arducopter-heli (ArduCopter V4.8.0-dev).

### 8.1 The GPS Position Fusion Gate

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

### 8.2 Condition: gpsGoodToAlign (10-second mandatory delay)

calcGpsGoodToAlign() (VehicleStatus.cpp) runs each EKF loop. On its very first call, it sets
lastGpsVelFail_ms=now regardless of check results. After 10 s, gpsGoodToAlign becomes true.

**EK3_GPS_CHECK=0 masks all individual quality checks** (HDOP, sats, speed accuracy, drift, yaw
error, etc.) so they all pass immediately. But **the 10-second clock still runs** from the first
GPS check. There is no parameter to shorten this delay.

**Practical consequence:** GPS position fusion cannot happen until at least **11-12 s** after
SITL launch, even with EK3_GPS_CHECK=0. Stack test timeouts must account for this.

### 8.3 Condition: yawAlignComplete (THE critical gate for RAWES setup)

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

### 8.4 Condition: delAngBiasLearned

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

### 8.5 Required Parameters for GPS Position Fusion

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

### 8.6 Timing Requirements (stack test design)

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

**Real RAWES mediator (constant-velocity kinematic, EK3_SRC1_YAW=1):**

| Event | Time from SITL start |
|---|---|
| EKF3 tilt alignment | ~4-5 s |
| Compass yaw alignment (immediate after tilt) | ~5-6 s |
| GPS detected | ~9 s |
| gpsGoodToAlign=true | ~19-20 s |
| delAngBiasLearned=true | **~29-33 s** (faster than stub) |
| GPS position fusion ("EKF3 is using GPS") | **~31-33 s** |

With EK3_SRC1_YAW=1 (compass) and constant body orientation (R=_R0 locked):
- Compass heading is constant -> P[12] converges in ~1 s after tilt align
- P[10,11] converge passively in ~28 s -> delAngBiasLearned at ~33 s
- GPS fuses at ~33 s (hub still in kinematic = stable) -> SUCCESS

**Stack tests must set STARTUP_DAMP_S = 45 s** so GPS fusion (~33 s) occurs during the stable
kinematic phase. The binding gate is delAngBiasLearned.

Required param set for GPS position fusion in real stack tests:
```
EK3_SRC1_YAW  = 1   (compass, NOT GPS velocity)
EK3_MAG_CAL   = 0   (Never -- use raw compass immediately)
COMPASS_USE   = 1
COMPASS_ENABLE= 1
EK3_GPS_CHECK = 0
```

### 8.7 CONST_POS_MODE Bit Meaning

EKF_STATUS_REPORT.flags bit 7 (0x0080) = const_pos_mode:

```cpp
status.flags.const_pos_mode = (PV_AidingMode == AID_NONE) && filterHealthy;
```

This is set when the EKF is healthy (tilt + yaw aligned, filterHealthy=true) but has no
position/velocity aiding source (AID_NONE). It clears when readyToUseGPS() returns true.

Seeing 0x00a7 = bits {0,1,2,5,7} means:
- 0x0001 att [PASS], 0x0002 horiz_vel [PASS], 0x0004 vert_vel [PASS], 0x0020 vert_pos_abs [PASS]
- 0x0080 CONST_POS_MODE -- stuck in AID_NONE, GPS position not yet fusing

### 8.8 Heli-Specific Behaviour Differences from Copter

1. **fly_forward=false** -- copter/heli never sets fly_forward, so assume_zero_sideslip()=false.
   This means delAngBiasLearned is always required (cannot be bypassed).

2. **Default EK3_SRC1_YAW=2** (GPS velocity heading) -- works for fixed-wing (always moving
   forward) but fails for hover-capable vehicles at low/zero speed.

3. **EK3_MAG_CAL=3 (ALWAYS)** -- more aggressive than fixed-wing default. The EKF tries to
   learn magnetometer biases in-flight always. Does not block yaw alignment itself, but
   wasLearningCompass_ms check can temporarily suppress compass yaw if COMPASS_LEARN=2 is active.

---

## 9. Lua API Constraints (This ArduPilot Build)

These constraints apply to rawes_flight.lua (and any future Lua scripts) running on the
ArduPilot SITL Docker image and on the Pixhawk 6C with the same firmware.

| What you'd expect | What actually works |
|---|---|
| ahrs:get_rotation_body_to_ned() | Doesn't exist. Use ahrs:body_to_earth(v) and ahrs:earth_to_body(v) |
| Vector3f(x, y, z) | Constructor ignores args (warning + wrong value). Use Vector3f() then :x()/:y()/:z() setters |
| v:normalized() | Doesn't exist. Copy then :normalize() in-place: local r = v3_copy(v); r:normalize(); return r |
| vec * scalar or vec + vec | * not overloaded; + may silently fail. Use component arithmetic directly |
| rc:set_override(chan, pwm) | Doesn't exist. Correct API: rc:get_channel(n):set_override(pwm) (cache channel at module load) |
| ArduCopter ACRO mode = 6 | ACRO = **1**. Mode 6 is RTL. Use vehicle:get_mode() == 1 |

**SCR_ENABLE bootstrap:** After wiping EEPROM, scripting does NOT start from copter-heli.parm
defaults on the first cold boot in this build. Scripting starts only when SCR_ENABLE=1 is already
in EEPROM from a previous session. The acro_armed_lua fixture handles this by NOT wiping EEPROM
and setting SCR_ENABLE=1 via MAVLink post-arm (persists to EEPROM for future boots).
copter-heli.parm already contains SCR_ENABLE 1 but this only takes effect on the second boot.

**Anchor position in LOCAL_POSITION_NED:** After the NED migration, initial_state["pos"][2] is
NED Z (negative ~= -7.12 for altitude 7.12 m above ground). The anchor in LOCAL_POSITION_NED
frame is at [0, 0, -initial_state["pos"][2]] = +7.12 m Down from EKF origin. So
SCR_USER5 = -home_z_enu (negate).

---

## 10. Takeoff and Landing

### 10.1 Takeoff

Physical sequence:
1. Ground station spins rotor to omega_spin >= omega_min (~10-15 rad/s). Planner monitors STATE.
2. Release mechanism drops rotor. Lift > weight -> rapid climb.
3. Tether pays out. Once taut, tension develops and lateral stability begins.
4. Natural orbit establishes. Planner begins pumping cycle.

Phase ownership:

| Phase | Planner | Pixhawk (Lua + ACRO) |
|---|---|---|
| Spin-up | Monitor omega_spin; trigger release at omega >= omega_min | None |
| Free climb (slack tether) | Pay out winch; send explicit attitude_q target | rawes_flight.lua holds attitude; Ch3 from thrust |
| Tether catch | Detect tension > threshold (local winch read); reduce thrust | Orbit tracking begins once _eq_captured |
| Transition to pumping | Ramp to operating tension setpoint | Natural orbit; follow SET_ATTITUDE_TARGET |

During the slack-tether phase, _eq_captured is false so orbit tracking is suppressed until
EKF position is valid and tether direction is meaningful.

### 10.2 Landing -- Spiral Descent

```
Step 1 -- Normal reel-in:
    Planner: slow winch reel-in, reduce tension setpoint.
    Lua: orbit track (identity attitude_q); Ch3 from thrust.
    Hub spirals inward and downward.

Step 2 -- Final approach (tether_length < ~10 m):
    Planner: ramp thrust -> 0 over ~5 s; hold or very slow reel-in.
    Lua: collective approaches zero; body_z alignment maintained.
    Autorotation continues -- rotor stores kinetic energy during descent.

Step 3 -- Flare (optional):
    Planner: brief thrust spike (~0.3) for 1-2 s.
    Lua: momentary collective increase slows final descent.

Step 4 -- Ground contact:
    Planner: engage ground motor (local command).
    Lua: hold last attitude command (slerp goal unchanged).
```

The division of responsibility is identical to pumping flight. Landing is a subset of the normal
protocol -- no new COMMAND fields are required.

---

## 11. Files

| File | Description |
|------|-------------|
| simulation/scripts/rawes_flight.lua | Orbit tracking + cyclic controller |
| simulation/torque/scripts/rawes_yaw_trim.lua | Counter-torque feedforward (validated) |
| simulation/torque/scripts/lua_defaults.parm | SITL param overrides for Lua tests |
| simulation/torque/README.md | Full counter-torque documentation and test results |
| simulation/controller.py | orbit_tracked_body_z_eq(), compute_swashplate_from_state(), TensionController |
| simulation/mediator.py | Rate-limited slerp, STATE/COMMAND packet assembly |
| simulation/sensor.py | SpinSensor (omega_spin noise model) |
| hardware/design.md | Swashplate geometry, servo specs, power architecture |

To deploy to hardware: copy both .lua files to APM/scripts/ on the SD card.

---

## 12. References

- ArduPilot Traditional Helicopter docs: https://ardupilot.org/copter/docs/traditional-helicopter-connecting-apm.html
- ArduPilot Lua scripting API: https://ardupilot.org/dev/docs/lua-scripts.html
- EKF3 source: libraries/AP_NavEKF3/ in ArduPilot repository
