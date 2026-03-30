# RAWES MAVLink Protocol

## Introduction

### What is RAWES?

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

There is a small motor on the axle, but it does not drive rotation.  Its sole
purpose is to apply a counter-torque that prevents the electronics, servos, and
swashplate assembly from spinning with the hub — directly analogous to the tail
rotor of a helicopter, but acting on a stationary assembly rather than countering
fuselage yaw.

### Glossary

| Term | Meaning |
|---|---|
| **body_z** | The unit vector along the rotor axle (spin axis). Naming convention used throughout this document and the codebase. |
| **Orbit tracking** | The Pixhawk-side control function that continuously rotates the attitude setpoint to match the hub's orbital position, keeping body_z aligned with the tether direction as the hub moves around the anchor. Runs at 400 Hz; the trajectory planner is not involved. |
| **ENU** | East-North-Up coordinate frame — used throughout this document. X = East, Y = North, Z = Up. Preferred over ArduPilot's native NED because positive-Z altitude makes tether and orbit geometry more intuitive. The conversion to NED is contained at the sensor boundary. |

### Physical variables

| Symbol | English name | Name | Description |
|---|---|---|---|
| **pos** | pos | Hub position | 3D position of the rotor hub in ENU world frame [m] |
| **vel** | vel | Hub velocity | 3D velocity of the rotor hub ENU [m/s] |
| **body_z** | body_z | Disk axis | Unit vector along the rotor axle (also tether direction at equilibrium) |
| **ξ** | xi | Disk tilt from wind | Angle between body_z and the horizontal wind direction [°]. Controls how much thrust acts along vs. perpendicular to the tether. |
| **β** | beta | Tether elevation | Angle of the tether above horizontal [°]. Determines the natural body_z direction at equilibrium. |
| **T** | T | Tether tension | Force along the tether [N]. Directly determines power = T × v_reel. |
| **ω_spin** | omega_spin | Rotor spin | Angular velocity of the rotor about its axle [rad/s]. Sustained by autorotation. |
| **ω** | omega | Orbital angular velocity | Angular velocity of the rotor hub about axes perpendicular to the axle [rad/s]. This is the "wobbling" or tilting rate of the disk, not the spin. Controlled by cyclic swashplate inputs. |
| **L₀** | L0 | Tether rest length | Unstretched tether length [m]. Changed by the winch to reel out/in. |

### Control variables

| Symbol | English name | Name | Description |
|---|---|---|---|
| **θ_col** | theta_col | Collective pitch | Average blade pitch across all blades [rad]. Controls thrust magnitude. |
| **θ_lon** | theta_lon | Longitudinal cyclic | Tilt of the swashplate fore/aft [rad]. Tilts disk north/south. |
| **θ_lat** | theta_lat | Lateral cyclic | Tilt of the swashplate left/right [rad]. Tilts disk east/west. |
| **v_winch** | v_winch | Winch speed | Rate of tether length change [m/s]. +ve = pay out, −ve = reel in. |
| **Γ_motor** | gamma_motor | Counter-torque | Torque applied by the GB4008 anti-rotation motor [N·m]. Keeps the electronics assembly stationary. See [Counter-torque motor](#counter-torque-motor). |

### The natural orbit

At equilibrium, the hub does not hover in one place — it orbits the anchor
point at constant tether length and elevation.  This is because the tether
direction defines the equilibrium disk axis (body_z), and as the hub moves
the disk tilts with it, creating a lateral force that drives the orbit.  This
is the system's natural resting state and requires zero control effort to
maintain.

The orbit is not a problem to be corrected — it is the expected flight
condition.  The control system is designed around it: the baseline attitude
setpoint rotates with the orbit automatically (orbit tracking), and the
trajectory planner only needs to command deviations from it.

### The pumping cycle (De Schutter 2018)

The fundamental AWE power generation strategy is a two-phase cycle:

**Reel-out (power phase):**
The disk is tether-aligned (ξ ≈ 30–55°).  High collective produces high
thrust mostly along the tether.  High tether tension.  The winch pays out
against this tension, driving a generator.  Power = T_out × v_reel_out.

**Reel-in (recovery phase):**
The disk tilts so that ξ increases toward 90°.  Thrust now acts mostly
upward rather than along the tether.  Tether tension drops to near the
gravity component alone (~15–30 N).  The winch reels in against this low
tension.  Cost = T_in × v_reel_in.

Net energy per cycle = (T_out − T_in) × v_reel × t_phase.  This is positive
as long as T_out > T_in, which requires ξ to be meaningfully different between
the two phases.

### Counter-torque motor

The rotor hub spins freely in the wind, but the electronics, swashplate, and
servos must stay stationary.  A motor on the axle applies a continuous
counter-torque to prevent the electronics assembly from co-rotating with the
hub — directly analogous to the tail rotor of a helicopter.

This is an inner-loop function owned entirely by `Mode_RAWES` on the Pixhawk.
The trajectory planner never commands it.

### Control architecture

Three nodes, two communication boundaries:

```
┌─────────────────────────────────────────────────┐
│  Trajectory Planner  (ground station, ~10 Hz)   │
│                                                 │
│  Inputs:  pos, vel, tension, time               │
│  Outputs: disk orientation + thrust → Pixhawk   │
│           winch speed → Winch Controller        │
│                                                 │
│  Owns: phase logic, energy accounting,          │
│        wind estimation, MPC (future)            │
└──────────────┬──────────────────┬───────────────┘
               │                  │
               │ MAVLink           │ local interface
               │ (~10 Hz,          │ (serial / CAN /
               │ SiK 433 MHz)      │ analog — TBD)
               │ STATE ↑           │
               │ attitude_q,       │ winch_speed_ms ↓
               │ thrust ↓          │
┌──────────────┴──────────────┐  ┌┴──────────────────────┐
│  Mode_RAWES                 │  │  Winch Controller      │
│  (Pixhawk 6C, in air)       │  │  (ground station)      │
│  400 Hz                     │  │                        │
│                             │  │  Drives motor to pay   │
│  Inputs: attitude_q, thrust │  │  out / reel in tether  │
│          load cell, EKF     │  │  at commanded rate.    │
│  Outputs: swashplate PWM,   │  │                        │
│           counter-torque PWM│  │  Owns: reel speed PID, │
│                             │  │  tension safety limits │
│  Owns: orbit tracking,      │  └────────────────────────┘
│  body_z slewing, tension PI,│
│  swashplate mixing,         │
│  counter-torque control     │
└─────────────────────────────┘
```

**Key separation:** The Pixhawk is physically on the flying kite — it cannot
wire to a ground-based winch.  The trajectory planner sends attitude and thrust
setpoints to the Pixhawk via the SiK radio link, and sends winch speed
commands directly to the winch controller via a separate local interface
(both are on the ground).

Orbit tracking runs at 400 Hz on the Pixhawk.  The trajectory planner at 10 Hz
only needs to say "point in this direction" — it does not manage orbit tracking
or smooth transitions.  At identity attitude_q the planner is completely silent
and Mode_RAWES sustains the natural orbit indefinitely.

---

## Design Principles

**Natural orbit is free.**
The RAWES hub naturally orbits the anchor at constant tether elevation.
`Mode_RAWES` tracks the tether direction at 400 Hz without any planner
involvement.  The planner only intervenes when it wants a specific disk
orientation (e.g. during reel-in tilt).

**Inner loops stay on the Pixhawk.**
Attitude tracking (cyclic), body_z slewing (rate-limited), tension control
(collective), and counter-torque motor control run at 400 Hz inside `Mode_RAWES`.
The planner sends *setpoints*, not actuator commands.

**Winch is on the ground.**
The winch controller is a separate ground-station node.  The trajectory planner
commands it directly via a local interface — the Pixhawk is never involved in
winch control.

**Planner sends targets, not transitions.**
`attitude_q` is the desired disk orientation.  Identity = natural tether-aligned
orbit.  Non-identity = desired ENU orientation.  `Mode_RAWES` slews body_z at
`body_z_slew_rate_rad_s` — the planner never manages blend factors or ramp timing.

---

## STATE Packet — Pixhawk → Planner

Sent by `Mode_RAWES` at ~10 Hz over MAVLink.  Maps to standard messages:

| Field | Type | MAVLink source | Description |
|---|---|---|---|
| `pos_enu` | float[3] | `LOCAL_POSITION_NED` (converted) | Hub position ENU [m] |
| `vel_enu` | float[3] | `LOCAL_POSITION_NED` (converted) | Hub velocity ENU [m/s] |
| `tension_n` | float | `NAMED_VALUE_FLOAT "tension"` | Tether tension [N] |
| `omega_spin` | float | `NAMED_VALUE_FLOAT "omega_spin"` | Rotor spin rate [rad/s]. Measured by a Hall-effect sensor on the rotor hub. Used by the trajectory planner's wind estimator and for physical model validation. See [Wind Estimation](#wind-estimation). |
| `t_free` | float | `SYSTEM_TIME` or onboard timer | Free-flight elapsed time [s] |

---

## COMMAND Packet — Planner → Pixhawk

Sent by the trajectory planner at ~10 Hz over the SiK MAVLink radio link.
Maps to standard MAVLink messages:

| Field | Type | MAVLink message / field | Description |
|---|---|---|---|
| `attitude_q` | float[4] (w,x,y,z) | `SET_ATTITUDE_TARGET` quaternion | Desired disk orientation ENU. `[1,0,0,0]` (identity) = tether-aligned natural orbit. Non-identity: `body_z_target = quat_apply(q, [0,0,1])`. `Mode_RAWES` slews at `body_z_slew_rate_rad_s`. |
| `thrust` | float [0..1] | `SET_ATTITUDE_TARGET` thrust field | Normalised tension setpoint. `Mode_RAWES` converts: `tension_setpoint_n = thrust × tension_max_n`. |
| `phase` | str | — | Telemetry label: `"hold"` \| `"reel-out"` \| `"reel-in"`. Not sent on wire. |

## Winch Command — Planner → Winch Controller

Sent by the trajectory planner directly to the ground-based winch controller
via a local interface (serial / CAN / analog — hardware TBD).  The Pixhawk
is not involved.

| Field | Type | Description |
|---|---|---|
| `winch_speed_ms` | float [m/s] | Winch rate command. +ve = pay out, −ve = reel in, 0 = hold. Maps to `MAV_CMD_DO_WINCH` (`WINCH_RATE_CONTROL`) if the winch controller speaks MAVLink. |

---

## Mode_RAWES Responsibilities (400 Hz, on Pixhawk)

```
Each 400 Hz step:

1. Receive latest COMMAND packet (hold last if no new one arrived)

2. Orbit tracking:
       Compute current tether direction from EKF position.
       Rotate initial equilibrium body_z_eq by the azimuthal change
       since free-flight start → body_z_tether (orbit-tracked equilibrium).

3. Attitude setpoint:
       if attitude_q == identity:
           body_z_eq = body_z_tether      ← natural equilibrium, no planner needed
       else:
           body_z_target = quat_apply(attitude_q, [0,0,1])
           body_z_eq = slerp_rate_limited(body_z_eq_prev, body_z_target,
                                          body_z_slew_rate_rad_s, dt)

4. Attitude controller:
       error = cross(body_z_current, body_z_eq)
       tilt_lon, tilt_lat = cyclic correction proportional to error

5. Tension PI controller:
       tension_setpoint_n = thrust × tension_max_n
       collective_rad = PI(tension_setpoint_n, tension_measured)

6. Swashplate output:
       servo_pwm = H3_inverse_mix(collective_rad, tilt_lon, tilt_lat)

7. Counter-torque motor:
       motor_pwm = anti_rotation_controller(imu_yaw_rate_body)
       (nulls assembly co-rotation; open-loop speed command)
```

---

## Trajectory Planner Responsibilities (~10 Hz, offboard)

```
Each planner step:

1. Receive STATE packet from Pixhawk

2. Determine phase from t_free and cycle parameters
   (reel-out / reel-in)

3. Compute:
       attitude_q         ← identity during reel-out (planner silent)
                            quat_from_vectors([0,0,1], body_z_reel_in) during reel-in
                            where body_z_reel_in = cos(xi)*wind_dir + sin(xi)*[0,0,1]
       thrust             ← tension_out/tension_max_n (reel-out)
                            tension_in/tension_max_n  (reel-in)
       winch_speed_ms     ← +v_reel_out or -v_reel_in

4. Send attitude_q + thrust to Pixhawk  (MAVLink SET_ATTITUDE_TARGET)
5. Send winch_speed_ms to Winch Controller  (local interface)
```

The planner does **not** manage orbit tracking, body_z slewing, attitude errors,
collective, swashplate mixing, or counter-torque.

---

## Wind Estimation

Wind direction and speed must be known to compute the reel-in attitude
quaternion (`body_z_reel_in = cos(xi) × wind_dir + sin(xi) × [0,0,1]`).
Four methods are available, ordered by implementation complexity.

### Method 1 — Rotor spin rate (in-plane speed)

`omega_spin` is in the STATE packet, measured by a Hall-effect sensor on the
rotor hub.

The trajectory planner uses the autorotation torque balance:

```
v_inplane = omega_spin² × K_drag / K_drive
```

`v_inplane` is the wind component perpendicular to the rotor axle.  To recover
full wind speed: `v_wind ≈ v_inplane / sin(xi)`, where xi is known from body_z
and wind_dir (Method 2).

**Physical model validation:** The planner can cross-check the measured spin
rate against the model prediction.  A deviation > 15 % indicates aerodynamic
model drift or a sensor fault.  Useful for HIL testing and regression validation.

### Method 2 — Orbital mean position (direction)

Over one full orbit the hub's mean horizontal position points downwind from the
anchor.  The orbit is symmetric about the wind direction, so:

```
wind_dir_enu ≈ normalize(mean(pos_enu_horizontal))   over one orbit period (~60 s)
```

Requires no extra sensors or hardware.  Direction converges within one orbit.
The trajectory planner recomputes the reel-in quaternion as the estimate
converges.

### Method 3 — Ground anemometer (future)

A 3D ultrasonic anemometer at the ground station provides an independent wind
measurement.  Useful for startup (before one orbit has completed) and
cross-validation.  Wind at hub altitude is extrapolated using a standard
logarithmic profile: `v(h) = v_ref × ln(h/z₀) / ln(h_ref/z₀)`.

**Protocol impact:** None — the anemometer is local to the ground station.
The trajectory planner reads it directly; it never crosses the MAVLink link.

### Method 4 — EKF wind state augmentation (future)

Augment the Pixhawk EKF state with a 3D wind vector.  The EKF observes tether
tension, hub velocity, and omega_spin and estimates wind as a slowly-varying
state.  More accurate than Methods 1–2 at transient conditions.

**Protocol impact:** If implemented, `wind_enu` would be added to the STATE
packet so the Pixhawk's wind estimate is available to the trajectory planner.

| Field | Type | MAVLink source | Description |
|---|---|---|---|
| `wind_enu` *(future)* | float[3] | `WIND` or `NAMED_VALUE_FLOAT "wind_enu"` | EKF-estimated wind vector ENU [m/s]. Enables planner to use Pixhawk's onboard estimate rather than computing its own. |

---

## Takeoff and Landing

### Overview

Takeoff and landing are the hardest phases to control because the tether — the
primary source of lateral stability in flight — is either slack (early takeoff)
or nearly vertical (late landing).  Understanding what stability it provides,
and when it stops providing it, determines how control must be divided between
the Pixhawk and the trajectory planner.

---

### Takeoff

#### Physical sequence

1. The ground station motor spins the rotor up to autorotation threshold
   (ω_spin ≥ ω_min, ~10–15 rad/s).  The Pixhawk monitors `omega_spin` from
   the STATE packet but does nothing — this phase is entirely ground-controlled.

2. The release mechanism drops the rotor.  Lift immediately exceeds weight
   because the rotor is already spinning fast.  The rotor climbs rapidly.

3. The tether pays out as the rotor climbs.  Once it goes taut, tether tension
   develops and lateral stability begins.

4. As tether length and β stabilise, the natural orbit establishes itself and
   the planner can begin commanding the pumping cycle.

#### What controls are needed and who owns them

The key challenge is **phase 3**: between release and first tether tension,
the rotor is a free-flying autogyro with no lateral constraint.  Body_z must
be held stable by the swashplate alone.

| Phase | Planner responsibility | Mode_RAWES responsibility |
|---|---|---|
| Spin-up | Monitor `omega_spin`; issue release command when ω ≥ ω_min | None — ground motor runs independently |
| Release | Trigger release mechanism (ground-local command) | None |
| Free climb (slack tether) | Pay out winch at climb rate; command `attitude_q = quat([0,0,1], body_z_initial)` to hold disk roughly vertical | Attitude hold — keep body_z stable using swashplate; collective proportional to `thrust` setpoint |
| Tether catch (first tension) | Detect tension > threshold in STATE packet; reduce thrust setpoint to normal operating level | Orbit tracking begins as tension is established |
| Transition to pumping | Wind up to operating tension setpoint; begin pumping cycle | Natural orbit establishes; follow normal COMMAND packets |

**Why the planner commands attitude during free climb:**
During the slack-tether phase, Mode_RAWES has no tether direction to orbit-track.
The planner must provide an explicit `attitude_q` target (e.g. body_z vertical or
along the expected initial tether direction) so the Pixhawk has a stable reference.
Mode_RAWES should not use orbit-tracking until tension is first detected.

**Protocol additions needed for takeoff:**

The STATE packet needs `tether_length` so the planner knows when the winch has
paid out enough tether to allow the rotor to reach operating altitude.

| Field | Type | MAVLink source | Description |
|---|---|---|---|
| `tether_length_m` *(future)* | float | `NAMED_VALUE_FLOAT "tether_length"` | Current deployed tether length [m]. Used by planner to detect free-climb vs. taut-tether phase transition. |

---

### Landing

#### The core stability problem

In flight, the tether provides lateral stability: any horizontal displacement of
the hub from the tether equilibrium direction is resisted by the tether's
restoring force component.  As the winch reels in and the tether shortens:

- **Long tether (β ≈ 30°):** Tether provides strong lateral restoring force.
  Hub is stable. Normal pumping control.
- **Short tether, high β (β → 60–70°):** Restoring force weakens as the tether
  approaches vertical.  Orbit radius shrinks.  Autorotation is still sustained
  but orbit becomes very tight.
- **Near-vertical tether (β → 90°):** Tether provides almost no lateral
  restoring force — it only pulls the hub straight down.  Any perturbation in
  body_z causes the hub to swing laterally with no correction.  **This is the
  landing instability zone.**

#### Landing approach — spiral descent

The safest strategy is to reel in slowly enough that the rotor never enters the
instability zone.  Once tether length is short enough that β is high, collective
is reduced gradually.  Less thrust means less tension, means less lift, means
the hub descends toward the anchor as the winch reels in.

```
Step 1 — Normal reel-in:
    Planner commands slow winch reel-in, reduced tension setpoint.
    Mode_RAWES holds tether-aligned orbit (identity attitude_q).
    Hub spirals inward and downward as orbit radius shrinks.

Step 2 — Final approach (tether_length < threshold, ~10 m):
    Planner commands thrust → 0 over several seconds.
    Winch holds or reels in very slowly.
    Hub descends as lift decreases; tether remains taut and guides descent.
    Autorotation still provides drag that slows the descent.

Step 3 — Ground catch:
    Hub is within reach of ground crew or mechanical catch.
    Ground motor re-engages to absorb remaining spin kinetic energy.
    Tether is short and nearly vertical — it constrains hub to descend
    straight down.
```

**Why tether tension is still useful in Step 2:**
Even with a nearly-vertical tether, downward tension is stabilising in the
vertical sense — it prevents the hub from rising.  The hub descends in a
controlled way because thrust (collective) is the only upward force and it is
being commanded to zero.

**The autorotation advantage:**
Unlike a helicopter (which needs engine power to arrest descent), the RAWES
rotor continues autorotating during descent.  Descending air flowing upward
through the disk actually drives autorotation faster — the rotor stores kinetic
energy as it descends.  This stored energy can be used in a flare manoeuvre
(momentary collective increase) to slow the final few metres of descent before
ground contact, just as an autogyro lands.

#### What controls are needed and who owns them

| Phase | Planner responsibility | Mode_RAWES responsibility |
|---|---|---|
| Descent reel-in | Reel-in winch at descent rate; reduce tension setpoint gradually | Orbit track (identity attitude_q); collective from tension PI |
| Final approach | Ramp `thrust` → 0 over ~5 s; hold winch or very slow reel-in | Apply collective from PI (approaches zero as tension setpoint → 0); maintain body_z alignment |
| Flare (optional) | Briefly command `thrust` spike (~0.3) for 1–2 s | Momentary collective increase slows descent |
| Ground contact | Issue ground motor engage command (ground-local) | Hold last attitude command; zero winch |

**The division of responsibility is the same as in flight:**
The planner decides *when* to transition phases and *what* tension setpoint to
target.  Mode_RAWES executes it at 400 Hz — it does not need to know it is
landing vs. flying; it just follows `thrust` and `attitude_q` commands exactly
as in the pumping cycle.

Landing is therefore a subset of the normal protocol — no new COMMAND fields
are required.  The planner uses `phase` field labels (`"land-reel-in"`,
`"land-final"`, `"flare"`) for telemetry only.

---

## Example: De Schutter Pumping Cycle

```
t=0s:   → Pixhawk:          attitude_q=[1,0,0,0] (identity), thrust=1.0 (→200 N)
        → Winch controller: winch_speed=+0.4 m/s (pay out)
        Mode_RAWES holds tether-aligned orbit naturally.  Winch pays out.

t=30s:  → Pixhawk:          attitude_q=quat_from_vectors([0,0,1],[cos55°,0,sin55°]),
                             thrust=0.1 (→20 N)
        → Winch controller: winch_speed=-0.4 m/s (reel in)
        Mode_RAWES slews body_z from tether-aligned toward 55° from wind
        at body_z_slew_rate_rad_s (~5 s for ~33° transition).

t=35s+: Mode_RAWES holds 55° tilt at low tension.  Winch reels in.

t=60s:  → Pixhawk:          attitude_q=[1,0,0,0] (identity), thrust=1.0 (→200 N)
        → Winch controller: winch_speed=+0.4 m/s (pay out)
        Mode_RAWES slews body_z back to tether-aligned.  Next reel-out begins.
```

---

---

## Appendix A: Simulation Implementation

This appendix describes how the protocol maps to the Python simulation in
`simulation/`.  Nothing here affects the protocol itself — it is a reference
for developers working on the simulator.

### Module map

| Protocol concept | Python module / class |
|---|---|
| Mode_RAWES 400 Hz loop | `mediator.py` — inner controller block |
| Trajectory planner | `trajectory.py` — `HoldTrajectory`, `DeschutterTrajectory` |
| STATE packet | `_state_pkt` dict assembled in mediator loop |
| COMMAND packet (attitude_q + thrust) | return value of `trajectory.step(state, dt)` |
| Winch command (separate ground link) | `winch_speed_ms` key in same dict; applied directly to `tether.rest_length` — no separate interface in simulation |
| Orbit tracking | `orbit_tracked_body_z_eq()` in `controller.py`, called from mediator loop |
| Body_z slewing | rate-limited slerp in mediator inner loop |
| Tension PI | `TensionController` in `controller.py`, called from mediator loop |
| Swashplate | `compute_swashplate_from_state()` in `controller.py` |
| Counter-torque motor | not modelled (single-body model; internal force cancels) |
| MAVLink radio link | direct Python function call (same process) |

### Sensor simulation

| Protocol concept | Simulation implementation |
|---|---|
| `omega_spin` in STATE packet | `SpinSensor.measure(omega_true)` in `sensor.py` — Hall-effect noise model: 8 magnets, σ ≈ 0.65 rad/s at 400 Hz |
| Physical model validation | `SpinSensor.validate(omega_measured, v_inplane)` — cross-checks measured spin against model prediction; warns if deviation > 15 % |

### Wind estimation

| Protocol concept | Simulation implementation |
|---|---|
| Method 1 (spin rate → v_inplane) | `WindEstimator.v_inplane_ms` property in `trajectory.py` |
| Method 2 (orbit mean position → wind direction) | `WindEstimator.wind_dir_enu` property in `trajectory.py` |
| Live wind tracking in planner | `DeschutterTrajectory(wind_estimator=WindEstimator(...))` — recomputes reel-in quaternion as estimate converges |

### Configuration

Simulation-specific parameters in `config.py` (not hardware parameters):

| Parameter | Default | Description |
|---|---|---|
| `tension_max_n` | 200.0 N | Converts normalised `thrust` [0..1] to tension setpoint [N] |
| `body_z_slew_rate_rad_s` | 0.12 rad/s | Max angular slew rate for body_z transitions |
| `spin_sensor_n_magnets` | 8 | Hall-effect sensor magnets; set 0 for ideal (noiseless) sensor |

### Moving to hardware

The simulation is a faithful software model of the protocol.  To move to
hardware:

- Direct Python function calls → actual MAVLink messages over the SiK radio
- `SpinSensor.measure()` → real Hall-effect interrupt on Pixhawk GPIO
- `tether.rest_length +=` → actual winch motor command over serial/CAN
- `WindEstimator` → runs on ground-station companion computer reading STATE packets
