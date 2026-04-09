# RAWES Simulation Internals

Detailed technical reference for the simulation stack. See [CLAUDE.md](../CLAUDE.md) for the overview and running tests.

---

## Natural / Equilibrium Hub Orientation

**The rotor axle (body Z) always aligns with the tether direction.** At tether elevation angle β:
```
body Z = [0, cos(β), -sin(β)]   (NED, hub East of anchor; East=Y, Up=-Z)
```

Always initialise the hub with body Z along the tether, not upright. The `build_orb_frame(body_z)` utility constructs a valid R0 from any body_z unit vector.

---

## Sensor Design — Physical Attitude

`PhysicalSensor` (the only sensor class) reports the **true physical orbital-frame orientation** — approximately roll=124°, pitch=−46° at tether equilibrium (ZYX Euler, NED). ACRO mode is compatible with this because it only damps angular rates toward commanded rates; the large physical tilt causes no automatic corrective cyclic.

**`PhysicalSensor.compute()`** (`sensor.py`):
- `rpy` = actual ZYX Euler angles of the orbital frame (spin stripped), with yaw replaced by velocity heading
- yaw derived from `atan2(vE, vN)`, rate-limited to ~0.05 rad/s to prevent gyro body-axis remapping when the tether activates
- gyro has spin stripped; rotated into the physical orbital body frame via `R_body.T`
- accel = specific force `R_body.T @ (accel_world_ned − [0,0,9.81])`

**`PhysicalHoldController`** (`controller.py`): captures equilibrium roll/pitch at kinematic startup end. Calls `compute_rc_from_attitude(roll − roll_eq, pitch − pitch_eq, ...)` so the function receives the deviation from equilibrium, not the raw physical angle.

**Sensor consistency rules (must all agree or EKF triggers emergency yaw reset):**
1. `velocity_ned` heading must match `rpy[2]` (yaw) — both from `atan2(vE, vN)`
2. `gyro_body` in physical orbital body frame (spin stripped, then `R_body.T @ omega_orbital`)
3. `accel_body` = `R_body.T @ (accel_world_ned − [0,0,9.81])`

---

## Controller Design

`simulation/controller.py` provides several functions and classes with different use cases:

### Orbit tracking — two algorithms, two purposes

**`orbit_tracked_body_z_eq(cur_pos, tether_dir0, body_z_eq0)`** — azimuthal (Z-preserving)

Rotates `body_z_eq0` azimuthally to track the hub's horizontal orbit. The Z-component of the setpoint is preserved from the initial equilibrium — it does not change when the hub gains or loses altitude. Safe to call at 400 Hz with no rate limiting downstream.

Used by: `test_steady_flight.py`, `test_closed_loop_60s.py` — inner-loop physics tests where collective is not separately controlled. The Z-preserving property prevents positive altitude feedback (hub sinks → setpoint stays at same elevation → no extra tilt → stable).

**`orbit_tracked_body_z_eq_3d(cur_pos, tether_dir0, body_z_eq0)`** — 3D Rodrigues (Lua-equivalent)

Rotates `body_z_eq0` by the full 3D rotation that maps `tether_dir0` onto the current tether direction `cur_pos/|cur_pos|`. Matches `rawes.lua`'s `orbit_track_3d()` exactly. The setpoint tracks the altitude component of the tether direction — requires a rate-limited slerp downstream to be stable.

Used by: `OrbitTracker` (below), `mediator.py`, pumping cycle tests.

**Why two functions exist:** In the Lua/SITL, altitude is controlled by the ground PI (winch speed → tether tension), so the 3D setpoint changing with altitude is safe — collective compensates. In Python inner-loop tests, there is no separate altitude controller, so the Z-preserving azimuthal algorithm is required.

### `OrbitTracker` — stateful orbit tracker (3D + slerp)

```python
OrbitTracker(body_z_eq0, tether_dir0, slew_rate_rad_s)
orbit_tracker.update(pos, dt, bz_target=None) -> body_z_slerped
```

Encapsulates `orbit_tracked_body_z_eq_3d` + `slerp_body_z` in one object, mirroring `rawes.lua`'s `_bz_orbit`/`_bz_slerp` state machine. When `bz_target` is provided (e.g., planner attitude override during landing), slerps toward that target instead of the natural orbit setpoint.

Used by: `mediator.py`, `test_gyroscopic_orbit.py`, `test_pumping_cycle.py`, `test_deschutter_cycle.py`, `test_deschutter_wind.py`, `test_kinematic_transition.py`, `test_pump_and_land.py`, `compare_rotors.py`.

### `compute_swashplate_from_state(hub_state, anchor_pos, ...)` — truth-state controller

Takes hub NED state (pos, R, omega) directly. Computes attitude error as `cross(body_z_cur, body_z_eq)` and damps orbital rates. Returns `{collective_rad, tilt_lon, tilt_lat}` to feed directly into `aero.compute_forces()`. **No ArduPilot involved.**

Used by: `test_closed_loop.py` (closed-loop physics validation), mediator internal controller.

**Sign convention (SkewedWakeBEM):** SkewedWakeBEM's per-blade physics gives the opposite cyclic sign to RotorAero's empirical K_cyc formula. With SkewedWakeBEM, `tilt_lat > 0` produces `-My` (opposite to the old model). Controller projections negate accordingly:
```python
tilt_lon = float(np.clip(+np.dot(corr, disk_x) / tilt_max_rad, -1.0, 1.0))
tilt_lat = float(np.clip(-np.dot(corr, disk_y) / tilt_max_rad, -1.0, 1.0))
```

### `compute_rc_rates(hub_state, anchor_pos, vel_ned, ...)` — NED truth-state RC controller

Same logic as above but outputs RC PWM dict `{1: pwm, 2: pwm, 4: pwm, 8: 2000}` for ArduPilot's ACRO rate loop. Converts correction to yaw-aligned NED body frame.

### `compute_rc_from_attitude(roll, pitch, rollspeed, pitchspeed, yawspeed, ...)` — MAVLink controller

Works directly with ArduPilot ATTITUDE message fields. Since `sensor.build_sitl_packet` reports tether-relative attitude, `roll` and `pitch` ARE the attitude error angles. Returns RC PWM dict.

Used by: `test_guided_flight.py` (ACRO hold loop via RC override, ~10 Hz from MAVLink)

### `TensionPI` — tension-to-collective PI controller

Owned by the trajectory planner (runs on ground station). Adjusts `collective_rad` to maintain requested tether tension.

**Critical collective limits (SkewedWakeBEM, beaupoil_2026):**
- Zero-thrust collective ≈ **−0.34 rad** when body_z is tether-aligned
- Zero-thrust collective ≈ **−0.228 rad** when body_z is at ξ=55° (reel-in orientation)
- `col_min_rad = −0.28` during reel-out (safe margin above −0.34)
- `col_min_reel_in_rad = −0.20` during reel-in (safe margin above −0.228)
- Below zero-thrust, BEM gives downforce → hub falls regardless of tether

Anti-windup: conditional integration (stop integrating when saturated and error pushes further). Prevents integral wind-up during kinematic startup.

---

## Dynamics Model

`simulation/dynamics.py` — RK4 6-DOF rigid-body integrator.

| Parameter | Value | Location |
|-----------|-------|----------|
| Timestep | 2.5e-3 s (400 Hz) | `DT_TARGET` in mediator.py |
| Mass | 5.0 kg | `beaupoil_2026.yaml` |
| Ixx = Iyy | 5.0 kg·m² | `beaupoil_2026.yaml` |
| Izz | 10.0 kg·m² | `beaupoil_2026.yaml` |
| I_spin | ~3.94 kg·m² | `beaupoil_2026.yaml` (gyroscopic coupling enabled in simtests) |
| Initial pos | `[13.9, 47.5, -7.1]` NED m | `steady_state_starting.json` (SkewedWakeBEM IC) |
| Initial vel | `[-0.257, 0.916, -0.093]` m/s | `config.py` DEFAULTS (startup ramp velocity) |
| Initial body_z | `[0.878, 0.276, 0.392]` | `steady_state_starting.json` (SkewedWakeBEM IC) |
| Initial spin | ~19.3 rad/s | `steady_state_starting.json` |

Gravity is applied internally — do **not** add it to forces.

**Rotor spin** is maintained as a separate scalar `omega_spin`, updated each step via the BEM-derived spin torque:
```
omega_spin += result.Q_spin / I_SPIN_KGMS2 × dt
```
`Q_spin` comes from `SkewedWakeBEM.compute_forces()` which balances drive torque (from inflow) against profile drag torque. Gives a stable equilibrium without manual K_drive/K_drag constants.

---

## Aerodynamic Model

**Production model:** `simulation/aero/aero_skewed_wake.py` — `SkewedWakeBEM`

All simulation code, tests, and the mediator use `SkewedWakeBEM` (imported via `simulation/aero.py`'s `create_aero()` factory). The old `RotorAero` empirical model is archived in `simulation/aero/` for comparison only.

### Why SkewedWakeBEM (not RotorAero)

`RotorAero` had three fundamental physics errors:
1. **Wrong cyclic**: empirical `K_cyc × tilt × T` scaling with sign tuned for the old model; per-blade physics gives the opposite sign.
2. **Wrong H-force**: `0.5 × μ × T` formula wildly overestimates (~105 N vs ~13 N actual).
3. **Wrong spin torque**: `K_drive × v_inplane − K_drag × omega²` are empirical constants with no physical basis.

### SkewedWakeBEM architecture

Per-blade strip BEM with:
- **Prandtl tip and root loss** factors on each radial strip
- **Coleman skewed-wake induction** factor: non-uniform induction across the disk at high inflow angle (pumping cycle reel-out has ξ≈31°, strongly non-uniform)
- **5-second aero startup ramp** to avoid impulse loads at t=0

Returns `AeroResult(F_world, M_orbital, Q_spin, M_spin)`:
- `F_world` — net aerodynamic force in NED world frame [N]
- `M_orbital` — cyclic/drag moments in NED (drives hub attitude) [N·m]
- `Q_spin` — net rotor torque (drive − drag) for the omega_spin ODE [N·m]
- `M_spin` — gyroscopic couple for rigid-body dynamics [N·m]

### Alternative models (for comparison / testing)

All located in `simulation/aero/` and importable via `simulation/aero.py`:

| Class | File | Description |
|-------|------|-------------|
| `SkewedWakeBEM` | `aero_skewed_wake.py` | **Production reference** — Prandtl + Coleman; explicit loops, full docstrings |
| `SkewedWakeBEMJit` | `aero_skewed_wake_jit.py` | **Fast path** — Numba `@njit` drop-in; select via `create_aero(model="jit")` |
| `PrandtlBEM` | `aero_prandtl_bem.py` | BEM + Prandtl tip/root loss, no skewed wake |
| `GlauertStateBEM` | `aero_glauert_states.py` | BEM + Glauert inflow-state detection |
| `RotorAero` | `aero_rotor.py` | Archived empirical model (do not use in simulation) |
| `DeSchutterAero` | `aero_deschutter.py` | De Schutter 2018 lumped-blade BEM (equation validation only) |

`SkewedWakeBEM` is the human-readable reference; `SkewedWakeBEMJit` inherits from it and
overrides `compute_forces` with two Numba kernels. Equivalence verified to `atol=1e-10`.

`DeSchutterAero` implements the paper's thin-airfoil model (Eq. 25–31) including:
- β side-slip diagnostic (`last_sideslip_mean_deg`) — validity check, not a force modifier
- C_{D,T} structural parasitic drag (cable drag, Eq. 29, 31) — 0.021 for de_schutter_2018
- See `simulation/aero/deschutter.md` for the full equation-level validation

Test framework in `simulation/aero/tests/` validates all models against each other.

---

## Tether Model

`simulation/tether.py` — `TetherModel` class.

| Property | Value |
|----------|-------|
| Material | Dyneema SK75 1.9 mm braided UHMWPE |
| EA (axial stiffness) | ~281 kN |
| k(L) | EA / L — nonlinear, stiffer when shorter |
| Linear mass | 2.1 g/m |
| Break load | ~620 N |
| Structural damping | 16.8 N·s/m |
| Rest length | ~49.97 m (SkewedWakeBEM IC) |
| Anchor | World origin (0, 0, 0) NED |

When tether is slack (hub closer than rest length) → zero force. Logs warning when tension exceeds 80% of break load.

Restoring moment (`r_attach × F_tether`) is supported but currently disabled (`axle_attachment_length=0.0`) — body_z stability comes from aerodynamics, not tether moment.

---

## Initial State and `steady_state_starting.json`

The mediator's default initial state is the warmup-settled equilibrium from `test_steady_flight.py`, which runs the TensionPI warmup sequence with SkewedWakeBEM:

| Parameter | Value (SkewedWakeBEM) | Notes |
|-----------|----------------------|-------|
| `pos` | `[13.9, 47.5, -7.1]` NED m | ~50 m tether at ~8° elevation |
| `vel` | ≈ 0 m/s | near-zero at settled equilibrium |
| `body_z` | `[0.878, 0.276, 0.392]` | tether-aligned |
| `omega_spin` | ~19.3 rad/s | equilibrium autorotation spin |
| `rest_length` | ~49.97 m | tether taut from t=0 |
| `coll_eq_rad` | −0.28 rad | TensionPI equilibrium collective |
| `home_z_ned` | 0.0 m | GPS home at ground level |

**IMPORTANT:** The `vel` in this file is the near-zero physics velocity at settled state. The stack test does **NOT** use it as `vel0` for the kinematic startup ramp. `vel0 = [-0.257, 0.916, -0.093]` from `config.py` DEFAULTS is always used for the ramp — it provides a non-zero heading so the EKF gets a velocity-derived yaw from frame 0.

**Workflow for regenerating:**
1. `pytest simulation/tests/unit/test_steady_flight.py` — writes `simulation/steady_state_starting.json`
2. Stack test reads this file and passes `pos0`, `body_z`, `omega_spin`, `rest_length` to mediator
3. If absent, mediator uses the built-in config defaults

---

## Kinematic Startup Ramp

The mediator runs a 45 s kinematic override so the ArduPilot EKF can initialise before free flight.

During the ramp:
- Hub position follows a constant-velocity trajectory from `launch_pos` to `pos0`
- `vel = vel0 = [-0.257, 0.916, -0.093]` m/s constant throughout (zero acceleration → clean IMU signal)
- Orientation locked to R0 throughout (prevents ACRO servo commands from misaligning the disk before physics starts)
- Non-zero velocity from frame 0 gives the EKF a velocity-derived yaw heading immediately

**EKF timeline (physical sensor mode, EK3_SRC1_YAW=1/compass):**
- ~2 s: tiltAlignComplete, yawAlignComplete
- ~10 s: GPS detected
- ~20 s: EKF origin set (gpsGoodToAlign, 10 s hardcoded delay)
- ~41 s: delAngBiasLearned → GPS position fuses → LOCAL_POSITION_NED appears
- t=45 s: kinematic end, physics starts

**EKF altitude during pumping cycle:** EKF altitude (from `LOCAL_POSITION_NED`) can drift significantly during GPS glitch events triggered by rapid hub position changes at reel-in body_z transition. Physics altitude from mediator telemetry (`hub_pos_z`) is authoritative for crash detection — EKF altitude is diagnostic only.

---

## Pumping Cycle Architecture

**DeschutterPlanner** (`simulation/planner.py`) implements the De Schutter (2018) reel-out/reel-in strategy:

```
Reel-out: body_z tether-aligned, TensionPI targets tension_out (200 N)
          -> high tether tension -> winch generates power
          col_max = 0.10 rad, col_min = -0.28 rad

Transition (15 s): body_z slews to xi=80 deg from wind at 0.40 rad/s

Reel-in:  body_z at xi=80 deg, TensionPI targets tension_in (~55 N)
          -> thrust acts upward (sin(80 deg) ~= 0.985), low tether tension
          col_min = 0.079 rad (above altitude-hold floor at xi=80 deg)
```

**Collective passthrough:** `DeschutterPlanner.step()` returns `collective_rad` directly (the raw value from `TensionPI.update()`). The mediator uses this directly without any normalization/denormalization. This avoids the col_min mismatch that would occur if the planner normalized using `col_min_reel_in` but the mediator denormalized using `col_min_reel_out`.

**Stack test results (beaupoil_2026, SkewedWakeBEM, wind=10 m/s East):**

| Config | Reel-out tension | Reel-in tension | Net energy | Peak tension |
|--------|-----------------|-----------------|------------|--------------|
| xi=80 deg (production) | 199 N | ~58 N | +1735 J | 455 N |
| xi=55 deg (baseline)   | 199 N |  86 N | +1396 J | 455 N |

Peak tension 455 N < 496 N (80% break load limit). Min physics altitude 5.7 m.

**WinchNode protocol boundary:** The planner communicates with the winch node exclusively
through `WinchNode.get_telemetry()` and `WinchNode.receive_command()`. It has no direct
access to `tether._last_info`, `wind_world`, or `WinchController`. Wind direction for
`WindEstimator` is seeded from `Anemometer.measure()` (3 m height) -- not the raw wind
vector. This mirrors the hardware MAVLink boundary and prevents simulation cheating.

---

## Landing Architecture

Validated in `tests/unit/test_landing.py` (isolated, from xi=80 deg, 20 m tether) and
`tests/unit/test_pump_and_land.py` (full pumping cycle then land from ~50 m).

### Why vertical descent, not spiral

During a tethered orbit the hub orbits faster as the tether shortens (angular momentum
conservation -- figure-skater effect).  At short tether lengths the orbital speed exceeds the
reel-in rate, the tether goes slack, and tension spikes 400+ N as it snaps taut.  A vertical
drop directly above the anchor avoids this entirely.

At the end of the De Schutter reel-in, body_z is at xi=80 deg from the (horizontal) wind vector.
That puts the disk only ~10 deg from horizontal -- leveling is essentially instant.  The nearly-
vertical tether supports >95% of hub weight throughout descent.

### Phase sequence

```
Pumping ends: tether ~50 m, body_z at xi=80 deg, altitude ~49 m.

Leveling (<1 s):
  Planner sends attitude_q targeting BZ_LEVEL = [0,0,-1] NED.
  Lua: slerp body_z at BODY_Z_SLEW_RATE (0.40 rad/s) -- 10 deg covered in ~0.4 s.
  Descent rate controller active from start of this phase.

Vertical descent (~32 s from 50 m at V_LAND=1.5 m/s):
  Planner: winch -V_LAND; descent rate controller for collective:
    vz_error = hub_vel_z - V_LAND   (NED +ve = downward)
    collective = clamp(COL_CRUISE + KP_VZ * vz_error, col_min, col_max)
    COL_CRUISE = 0.079 rad  KP_VZ = 0.05 rad/(m/s)
  Lua: body_z_eq = [0,0,-1]; hover gains (kp=1.5, kd=0.5).
  Tether pendulum naturally centres hub above anchor.

Final drop (tether <= 2 m):
  Planner: collective = 0; winch hold.
  Hub drops ~2 m onto catch device.
```

### Controller gains

| Phase | kp | kd | Rationale |
|---|---|---|---|
| Pumping (orbit) | 0.5 | 0.2 | Tether provides passive attitude restoring force |
| Leveling + descent | 1.5 | 0.5 | Cyclic must carry all attitude authority alone |

### Why descent rate controller (not TensionPI)

TensionPI reacts to tension error.  If the hub descends faster than the reel-in rate the tether
goes slack, TensionPI sees near-zero tension and commands max collective.  The tether snaps taut
and the cycle repeats.  The descent rate controller reacts to hub vz from LOCAL_POSITION_NED and
holds descent at V_LAND regardless of tether state.

### Collective floor during descent

`COL_CRUISE = 0.079 rad` = `col_min_reel_in` at xi=80 deg = the minimum collective to sustain
Fz >= hub weight when the disk is horizontal.  This is the no-load hover point; the descent rate
controller adds or subtracts a small correction (KP_VZ * vz_error) around this base.

### Validated results

| Test | Max tension | Anchor dist | Tilt | vz at touchdown |
|---|---|---|---|---|
| test_landing.py (20 m start) | 403 N | 0.51 m | 2.2 deg | 0.00 m/s |
| test_pump_and_land.py (~50 m start) | 359 N | 0.69 m | 2.0 deg | 0.00 m/s |

Both below 620 N break load.  Hub lands within 5 m (ANCHOR_LAND_RADIUS_M) of the catch device.

---

## Known Gaps Between Source Thesis Model And Actual Hardware

| Item | Thesis model | Actual design |
|------|-------------|--------------|
| Blade count | 3 | 4 |
| Blade length | 1.5 m | 2.0 m |
| Cyclic phase offset | 2π/3 (120°) | π/2 (90°) |
| Rotor mass | 40 kg | 5 kg |
| Flap actuation | Individual servo per blade | Swashplate push-rods (mechanical) |
| Anti-rotation | Not modeled | GB4008 + 80:44 gear |

Additional physics limitations in the current simulation:
- No dynamic inflow, no wake memory, no tip vortices
- Tether: tension-only elastic spring, no sag, no distributed mass
- Spin ODE is separate scalar, not fully coupled with orbital dynamics
- Anti-rotation motor not modeled (internal force in single-body model)

---

## Background Academic References

1. **Felix Weyel (2025)** — "Modeling and Closed Loop Control of a Cyclic Pitch Actuated Rotary Airborne Wind Energy System", Bachelor's Thesis, Uni Freiburg.
2. **De Schutter, Leuthold, Diehl (2018)** — "Optimal Control of a Rigid-Wing Rotary Kite System for Airborne Wind Energy".
3. **US Patent US3217809** (Kaman/Bossler, 1965) — Canonical servo-flap rotor control system.
