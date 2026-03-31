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

## Sensor Design — Tether-Relative Attitude

**Critical design decision:** attitude is reported as the **deviation from the tether equilibrium orientation**, not absolute orientation relative to world Up.

**Why:** The RAWES equilibrium has body Z at ~30° elevation — ~65° from vertical. Reporting absolute attitude causes ArduPilot to see 58° roll / 36° pitch at rest and command maximum cyclic, crashing the hub within 1 second.

**How** (`sensor.build_sitl_packet`):
- `rpy = [0, 0, yaw_from_velocity]` — roll and pitch are always zero (tether-relative)
- yaw is derived from velocity heading so EKF attitude and velocity are consistent
- gyro has spin stripped; rotated into yaw-aligned body frame
- accel is kinematic accel in NED body frame plus −g correction

**EKF consequence:** ArduPilot interprets deviations from tether equilibrium as small roll/pitch errors and corrects with modest cyclic, not maximum deflection.

**Sensor consistency rules (must all agree or EKF triggers emergency yaw reset):**
1. `velocity_ned` heading must match `rpy[2]` (yaw) — both derived from `atan2(vE, vN)`
2. `gyro_body` must be in yaw-aligned NED body frame (spin stripped, then `Rz(-yaw) @ omega_nospin`)
3. `accel_body` must be `Rz(-yaw) @ (accel_world_ned - [0,0,9.81])`

**Physical sensor mode** (`PhysicalSensor`) reports actual orbital-frame orientation (~65° from NED vertical). Used by the pumping cycle stack test. Applies the same velocity-derived yaw override + rate-limiting to avoid a yaw jump when the tether activates (which would remap the gyro body axes and destabilise ACRO).

---

## Controller Design

`simulation/controller.py` provides three functions with different use cases:

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
| `SkewedWakeBEM` | `aero_skewed_wake.py` | **Production** — Prandtl + Coleman |
| `PrandtlBEM` | `aero_prandtl_bem.py` | BEM + Prandtl tip/root loss, no skewed wake |
| `GlauertStateBEM` | `aero_glauert_states.py` | BEM + Glauert inflow-state detection |
| `RotorAero` | `aero_rotor.py` | Archived empirical model (do not use in simulation) |
| `DeSchutterAero` | `aero_deschutter.py` | De Schutter 2018 lumped-blade BEM (for validation) |

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
          → high tether tension → winch generates power
          col_min = −0.28 rad (safe above zero-thrust at −0.34 rad)

Transition (t_transition=15 s): body_z slews to xi=55° from wind at body_z_slew_rate=0.40 rad/s

Reel-in:  body_z at xi=55°, TensionPI targets tension_in (≥80 N for altitude maintenance)
          → thrust acts upward, not along tether → low aerodynamic resistance
          col_min = −0.20 rad (safe above zero-thrust at −0.228 rad for xi=55°)
```

**Why tension_in ≥ 80 N:** At ξ=55°, vertical thrust component ≈ collective-dependent. With col_min=−0.20, thrust ≈ 70–100 N vertically. Hub mass is 5 kg → gravity 49 N. At low collective the hub would fall — tension_in must be high enough to keep TensionPI outputting adequate collective.

**Collective passthrough:** `DeschutterPlanner.step()` returns `collective_rad` directly (the raw value from `TensionPI.update()`). The mediator uses this directly without any normalization/denormalization. This avoids the col_min mismatch that would occur if the planner normalized using `col_min_reel_in` but the mediator denormalized using `col_min_reel_out`.

**Stack test results (beaupoil_2026, SkewedWakeBEM, wind=10 m/s East):**
- Reel-out mean tension: 199 N
- Reel-in steady mean tension: 86 N
- Net energy: +1396 J per cycle
- Peak tension: 455 N (< 496 N = 80% break load limit)
- Min physics altitude: 5.7 m throughout cycle

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
