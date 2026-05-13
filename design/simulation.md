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

`PhysicalSensor` (the only sensor class) reports the **true physical hub orientation** — approximately roll=124°, pitch=−46° at tether equilibrium (ZYX Euler, NED). ACRO mode is compatible with this because it only damps angular rates toward commanded rates; the large physical tilt causes no automatic corrective cyclic.

**`PhysicalSensor.compute()`** (`sensor.py`):
- `rpy` = actual ZYX Euler angles from `R_hub` directly (no overrides)
- `gyro_body` = `R_hub.T @ omega_body` — full body angular velocity in electronics body frame. No stripping: GB4008 keeps electronics non-rotating via K_YAW damping in dynamics.
- `accel_body` = `R_hub.T @ (accel_world_ned − [0,0,9.81])` — specific force in electronics body frame

**`PhysicalHoldController`** (`controller.py`): captures equilibrium roll/pitch at kinematic startup end. Calls `compute_rc_from_attitude(roll − roll_eq, pitch − pitch_eq, ...)` so the function receives the deviation from equilibrium, not the raw physical angle.

**Sensor consistency rules (must all agree or EKF triggers emergency yaw reset):**
1. `rpy[2]` = actual hub orientation yaw from `R_hub` (never overridden with velocity heading)
2. `gyro_body` = `R_hub.T @ omega_body` — no artificial stripping
3. `accel_body` = `R_hub.T @ (accel_world_ned − [0,0,9.81])`

---

## Controller Design

`simulation/controller.py` is organised as a **portable core** (functions that map 1:1 to Lua) plus higher-level Python wrappers used by simtests. The production design has no orbit-tracking step — `rawes.lua` uses `bz_altitude_hold` (rate-limited elevation hold + gravity-compensated tilt), and Python simtests use the same primitive via `AltitudeHoldController` / `ElevationHoldController`.

### Portable core (mirrored in rawes.lua)

| Function | Purpose |
|----------|---------|
| `compute_bz_tether(pos)` | Unit vector along tether from anchor to hub |
| `compute_bz_altitude_hold(pos, target_el_rad, tension_n, mass_kg)` | Stateless: target tether direction at `target_el_rad` + gravity-compensation tilt → returns `body_z_eq` |
| `slerp_body_z(bz, goal, slew_rate, dt)` | Rate-limited slerp toward goal (the only smoothing layer on body_z) |
| `compute_rate_cmd(bz_now, bz_goal, kp, omega_body, kd=0)` | Outer-loop P (+ optional D): body_z error → angular-rate setpoint |
| `col_min_for_altitude_rad(pos, ...)` | Collective floor at the current tether elevation |

`test_math_lua.py` cross-checks these against `rawes.lua` — a failure there means `controller.py` diverged from Lua; fix `controller.py`.

### Higher-level Python wrappers (simtests + planners)

- **`AltitudeHoldController`** — wraps `compute_bz_altitude_hold` with an internal elevation rate-limiter. `from_pos(pos, slew_rate_rad_s)` initialises `_el_rad` from the IC elevation; `update(pos, target_alt_m, tension_n, mass_kg, dt)` rate-limits `_el_rad` toward `asin(target_alt_m/tlen)` and returns the corresponding `body_z_eq`. Used by `TensionApController` and `LandingApController`.
- **`ElevationHoldController`** — adds `compute_rate_cmd` on top of `AltitudeHoldController`, returning `(rate_roll_sp, rate_pitch_sp)` directly. Used inside `TensionApController`.
- **`RatePID(kp=2/3)`** — inner rate loop. Maps to ArduPilot `ATC_RAT_RLL` / `ATC_RAT_PIT` PID on the stack side.
- **`compute_rc_from_attitude(roll, pitch, rollspeed, pitchspeed, yawspeed, ...)`** — MAVLink-side helper: takes ArduPilot ATTITUDE-message fields (which already encode tether-relative attitude error via `PhysicalSensor`) and returns RC PWM dict. Used by Python-side hold loops where the only feedback is a 10 Hz MAVLink stream.

### Legacy / dead code (do not use)

`controller.py` still contains `orbit_tracked_body_z_eq`, `orbit_tracked_body_z_eq_3d`, `compute_swashplate_from_state`, `compute_rc_rates`, and a `BodyZSlewedController` class from the Phase 2 orbit-tracking era. They are no longer referenced by `rawes.lua`, any simtest, or any production code. Treat them as candidates for deletion; do not add new callers. The replacement primitive is `compute_bz_altitude_hold` + `slerp_body_z`. See [history.md § Why orbit-tracking was replaced](history.md).

### `TensionPI` — tension-to-collective PID controller

Runs on the AP side. In Lua mode 5 it lives inside `rawes.lua` (TensionPID); in the Python unified pumping test it lives inside `TensionApController` at 400 Hz. The ground sends the setpoint + ground-measured tension via `TensionCommand` (10 Hz); the AP closes the loop using ground-transmitted feedback. Output: `collective_rad` clamped to `[coll_min, coll_max]`.

Output: `col = kp*err + ki*∫err + kd*(err − prev_err)/dt`, clamped to `[coll_min, coll_max]`.

**Gains (controller.py defaults, 400 Hz):**

| Gain | Default | Notes |
|------|---------|-------|
| `kp` | 5e-4 | rad/N |
| `ki` | 1e-4 | rad/(N·s) |
| `kd` | 0.0 | rad·s/N — derivative on error; try ~2e-5 to damp tension oscillations (Peters-He) |

**Gains (rawes.lua / TensionApController, 10 Hz):**

| Constant | Default | Notes |
|----------|---------|-------|
| `KP_TEN` | 2e-4 | rad/N |
| `KI_TEN` | 1e-3 | rad/(N·s) at DT_CMD=0.1 s |
| `KD_TEN` | 0.0 | rad·s/N; derivative step = DT_CMD |

**Critical collective limits (SkewedWakeBEM, beaupoil_2026):**
- Zero-thrust collective ≈ **−0.34 rad** when body_z is tether-aligned
- Zero-thrust collective ≈ **−0.228 rad** when body_z is at ξ=55° (reel-in orientation)
- `col_min_rad = −0.28` during reel-out (safe margin above −0.34)
- `col_min_reel_in_rad = −0.20` during reel-in (safe margin above −0.228)
- Below zero-thrust, BEM gives downforce → hub falls regardless of tether

Anti-windup: conditional integration (stop integrating when saturated and error pushes further). Derivative term bypasses anti-windup — it is added after the clamp check and does not affect the integrator.

---

## Dynamics Model

`simulation/dynamics.py` — RK4 6-DOF rigid-body integrator.

| Parameter | Value | Location |
|-----------|-------|----------|
| Timestep | 2.5e-3 s (400 Hz) | `DT_TARGET` in `mediator.py`; integration in `physics_core.py` |
| Mass | 5.0 kg | `beaupoil_2026.yaml` |
| Ixx = Iyy | 5.0 kg·m² | `beaupoil_2026.yaml` |
| Izz | 10.0 kg·m² | `beaupoil_2026.yaml` |
| I_spin | ~3.94 kg·m² | `beaupoil_2026.yaml` (gyroscopic coupling enabled in simtests) |
| Initial pos | `[13.9, 47.5, -7.1]` NED m | `steady_state_starting.json` (SkewedWakeBEM IC) |
| Initial vel | `[-0.257, 0.916, -0.093]` m/s | `config.py` DEFAULTS (startup ramp velocity) |
| Initial body_z | `[0.878, 0.276, 0.392]` | `steady_state_starting.json` (SkewedWakeBEM IC) |
| Initial spin | ~19.3 rad/s | `steady_state_starting.json` |

Gravity is applied internally — do **not** add it to forces.

### Electronics hub as fuselage

The RAWES hub maps directly onto a helicopter model:

| Helicopter | RAWES |
|---|---|
| Fuselage | Electronics hub (Pixhawk, GB4008 stator, frame) |
| Main rotor | Spinning blade assembly (4 blades + outer shell) |
| Tail rotor | GB4008 counter-torque motor |
| Fuselage orientation (R) | `R_hub` — full 3-DOF rotation matrix |
| Rotor speed (Ω) | `omega_spin` — separate scalar ODE |

**`R_hub` is the fuselage orientation.** It is a full 3×3 rotation matrix with all three orientation DOF:
- `R_hub[:, 2]` = `disk_normal` — where the rotor axle points (2 DOF: tilt direction)
- `R_hub[:, 0]` = electronics x-axis — **the yaw DOF** (1 DOF: rotation around axle)

The yaw DOF is not a convention or a derived quantity — it is a physical state integrated
by the dynamics ODE like any other orientation component. The sensor reads `R_hub` directly
to build the attitude packet sent to ArduPilot; no separate yaw state is needed.

**GB4008 as tail rotor.** The GB4008 provides a torque around `disk_normal` on the electronics
hub to counteract the aerodynamic spin-up torque from the rotor drag. In `PhysicsCore._integrate()` this is
modelled as:

```python
omega_yaw = np.dot(hub_state["omega"], disk_normal)   # electronics yaw rate [rad/s]
T_GB4008  = -K_yaw * omega_yaw                         # counter-torque [N·m]
M_orbital += disk_normal * T_GB4008
```

`K_yaw → large`: perfect damper (yaw rate ≈ 0, hardware nominal).
`K_yaw` finite: realistic GB4008 with residual yaw drift, visible in `rpy_yaw` telemetry.

**Rotor spin** is maintained as a separate scalar `omega_spin`, updated each step via the BEM-derived spin torque:
```
omega_spin += result.Q_spin / I_SPIN_KGMS2 × dt
```
`Q_spin` comes from `PetersHeBEMJit.compute_forces()` which balances drive torque (from inflow) against profile drag torque. Gives a stable equilibrium — no empirical K_drive/K_drag constants needed.

**Gyroscopic coupling** is included in Euler's equations (`dynamics.py`):
```
H_spin = I_spin * omega_spin * body_z   (spin angular momentum vector)
domega_b = I_body_inv @ (tau_b - omega_b × (I_body @ omega_b + H_spin))
```
The `omega_b × H_spin` cross-product term is the precession coupling: any torque applied to the spinning rotor causes the hub to precess at 90° to that torque rather than rotating directly toward the torque direction.

---

## Hub Stability Physics

The RAWES hub has **no passive stability** — only active cyclic prevents divergence. The same physics produces a *natural self-sustaining orbit* that the controller must work with rather than fight. This section explains both.

### Inverted-pendulum configuration

The tether attaches at the **bottom** of the axle, 0.3 m below the centre of mass. For a non-spinning body this is an inverted pendulum — the CM sits above the pivot point, so any tilt causes the tether torque to increase the tilt further rather than restore it. Unconditionally unstable without active control.

### Gyroscopic conversion of torques to precession

The spinning rotor changes the instability character. Any torque applied to a spinning gyroscope does not produce rotation in the torque direction — it produces **precession at 90°** to the torque. This applies to every torque acting on the hub:

| Torque source | Non-spinning response | Spinning rotor response |
|---|---|---|
| Tether offset (inverted pendulum) | Increases tilt (unstable) | Precesses body_z sideways |
| Gravity on CM above attachment | Pulls CM down (unstable) | Precesses body_z (like a spinning top) |
| Aerodynamic cyclic moment | Tilts disk | Precesses body_z in controlled direction |

The tether and gravity torques therefore drive **uncontrolled orbital precession** rather than a simple fall. Without active cyclic control the hub drifts into an uncontrolled orbit and crashes within seconds. Confirmed empirically: neutral sticks in ACRO mode always crashes the hub.

Both effects are fully implemented:
- **Tether offset moment** (`M = r_attach × F_tether`, `r_attach = 0.3 m along body_z`) — computed in `tether.py` and applied to `dynamics.step()` every timestep.
- **Gyroscopic precession** (`omega × (I_body @ omega + H_spin)`) — included in Euler's equations in `dynamics.py`.

### Natural orbit at equilibrium

Starting from rest with a 200 N tension setpoint at 10 m/s wind, the hub settles into a quasi-circular orbit on the tether-length sphere. Settled state (beaupoil_2026, 50 m tether):

| Quantity | Value |
|----------|-------|
| Position (NED) | [13.9 N, 47.5 E, −7.1 D] m |
| Hub altitude AGL | 7.1 m |
| Tether elevation angle β | 8.2° |
| Disk tilt from wind ξ | 29° |
| Disk tilt from vertical | 67° (true Euler ≈ roll 124°, pitch −46° ZYX) |
| Rotor spin ω_eq | 19.3 rad/s (184 RPM) |
| Spin angular momentum `H = I_spin · ω` | ≈ 76 kg·m²/s |
| Orbit half-angle θ (cone around wind axis) | ≈ 18° |
| Orbital radius `L · sin θ` | ≈ 15.6 m |
| Orbital period | ≈ 60 s |
| World-frame orbital rate | ≈ 0.105 rad/s |
| **Body-frame angular rate (roll/pitch)** | **≈ 0.2–0.3 rad/s** ← persistent rate-loop bias |

### Why the orbit is self-sustaining

The orbit needs no active steering. The disk is tilted ξ ≈ 29° from the wind, producing a thrust component perpendicular to both tether and wind. That lateral component pushes the hub azimuthally; as the tether direction rotates, the disk precesses gyroscopically to track it; the hub's orbital velocity adds in-plane wind that sustains autorotation spin. Period and radius are set by the balance of centripetal force, tether stiffness, and gyroscopic precession rate.

**Stability requires damping.** With `base_k_ang = 50 N·m·s/rad` (modelling the AcroController rate PIDs, τ = I_body/k_ang ≈ 0.1 s), gyroscopic cross-coupling `ω_orbital × H` is small and the orbit is stable for `H_SW_PHANG` values across 0°–180°. Without damping (k_ang = 0) the orbit diverges within seconds.

### Five control implications

These follow directly from the orbit physics and pin down several CLAUDE.md invariants:

1. **Use ACRO, never STABILIZE.** STABILIZE commands absolute NED attitude (roll=0=level), which fights the 67° tether equilibrium and crashes within 1–2 s. ACRO only damps angular rates toward commanded rates, so the large physical tilt causes no automatic corrective cyclic. PhysicalSensor reports the true hub orientation directly.
2. **Disable rate-loop I-term:** `ATC_RAT_RLL_IMAX = ATC_RAT_PIT_IMAX = ATC_RAT_YAW_IMAX = 0`. The 0.2–0.3 rad/s orbital body rate is a *desired* steady-state rate, not a disturbance to reject. Without IMAX=0 the I-term integrates this as a tracking error and saturates the swashplate in ≈ 50 s.
3. **Reference body_z must follow the rotating tether direction.** A fixed reference accumulates ≈ 90° of phase lag per quarter orbit (~15 s). `rawes.lua` uses `compute_bz_altitude_hold(pos, target_el_rad, tension)` — target tether direction at the rate-limited elevation, plus a gravity-compensation tilt. Position is the only sensor needed.
4. **Disk-tilt slew rate limit = 0.40 rad/s** (`SCR_USER2`). Gyroscopic precession could theoretically tilt the disk at ≈ 20 rad/s; closed-loop bandwidth caps the useful rate at ≈ 2 % of that. Faster slews cause oscillation; slower wastes reel-in time. Minimum reel-out → reel-in transition (Δξ = 45°) ≈ 2 s, with a 3–4 s budget per cycle boundary including settling.
5. **Orbital mean position is a passive wind direction estimate.** Mean horizontal position over one orbit lies downwind from the anchor. Convergence: one orbital period (~60 s); accuracy: ~20° absolute (limited by the azimuthal offset from rotor angular momentum). In-plane wind speed is simultaneously recoverable from `omega_spin` via the autorotation torque balance.

### Implementation mapping

| Implication | Where it lives |
|---|---|
| Physical attitude + ACRO | `PhysicalSensor` reports true `R_hub`; `COMPASS_USE=0`; `EK3_SRC1_YAW=2` |
| Rate-loop bias | `ATC_RAT_*_IMAX = 0` (boot params, `rawes_sitl_defaults.parm`) |
| Reference body_z tracking | `rawes.lua`: `bz_altitude_hold(pos, _el_rad, tension)` at 50 Hz |
| Slew rate limiting | `rawes.lua`: elevation slew `SCR_USER2 = 0.40 rad/s` |
| Gyro phase compensation | `H_SW_PHANG = 0` (empirical; BASE_K_ANG damping τ ≈ 0.08 s « orbital period) |

---

## Aerodynamic Model

**Production model:** `PetersHeBEMJit` (`simulation/aero/aero_peters_he_jit.py`)

All simulation code, tests, and the mediator use `PetersHeBEMJit` (imported via the `simulation/aero` package's `create_aero()` factory, or instantiated directly as `PetersHeBEMJit(rotor)`).

### Why PetersHeBEM (not the old RotorAero)

`RotorAero` had three fundamental physics errors:
1. **Wrong cyclic**: empirical `K_cyc × tilt × T` scaling with sign tuned for the old model; per-blade physics gives the opposite sign.
2. **Wrong H-force**: `0.5 × μ × T` formula wildly overestimates (~105 N vs ~13 N actual).
3. **Wrong spin torque**: `K_drive × v_inplane − K_drag × omega²` empirical constants with no physical basis; replaced by `Q_spin` from BEM strip integration.

### PetersHeBEMJit architecture

3-state dynamic inflow ODE (v0, v1c, v1s) with per-blade BEM strip integration:
- **Peters-He induction**: `v_i(r, ψ) = v0 + v1c·(r/R)·cos(ψ) + v1s·(r/R)·sin(ψ)` — non-uniform induction evolved dynamically via implicit-Euler ODE
- **Numba `@njit` hot loop**: strip accumulation compiled to native SIMD
- **No skew-angle validity limit** — momentum ODE valid from hover through axial descent (xi up to 90°)
- **5-second aero startup ramp** to avoid impulse loads at t=0
- **Stateful**: serialize/restore inflow states via `aero.to_dict()` / `create_aero(model="jit", state_dict=d)`

Returns `AeroResult(F_world, M_orbital, Q_spin, M_spin)`:
- `F_world` — net aerodynamic force in NED world frame [N]
- `M_orbital` — cyclic/drag moments in NED (drives hub attitude) [N·m]
- `Q_spin` — net rotor torque (drive − drag) for the omega_spin ODE [N·m]
- `M_spin` — gyroscopic couple for rigid-body dynamics [N·m]

### AeroBase interface

All models in `simulation/aero/` inherit from `AeroBase` (ABC defined in `aero/__init__.py`).
Instantiate any model directly — no factory required:

```python
from aero import PetersHeBEMJit, PetersHeBEM, create_aero
aero = PetersHeBEMJit(rotor)                         # direct (preferred)
aero = create_aero(rotor, model="jit")               # factory shim
aero = PetersHeBEMJit(rotor, state_dict=prev.to_dict())  # warm-start
```

Every subclass inherits `AeroBase.from_definition(rotor, state_dict=None)` — identical to calling the constructor directly.

### Available models

All in `simulation/aero/`, importable via the package:

| Class | File | Description |
|-------|------|-------------|
| `PetersHeBEMJit` | `aero_peters_he_jit.py` | **Production** — Numba JIT, 3-state dynamic inflow; `model="jit"` |
| `PetersHeBEM` | `aero_peters_he.py` | Pure-numpy reference; `model="numpy"` |
| `OpenFASTBEM` | `aero_openfast_bem.py` | Quasi-static BEM with Brent phi-solve, Prandtl + Buhl correction |
| `CCBladeBEM` | `aero_ccblade_bem.py` | Tabulated XFOIL/AeroDyn polar (Re-dependent CL/CD); requires ccblade |
| `SimpleBEM` | `aero_simple_bem.py` | Minimal textbook BEM — sanity cross-check only |

Test framework in `simulation/aero/tests/` validates models against each other.

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

Tether offset moment (`M = r_attach × F_tether`) is **enabled**: `axle_attachment_length = 0.3 m` (from `beaupoil_2026.yaml`). This moment is computed at every timestep and passed to `dynamics.step()` as part of `M_orbital`. See **Hub Stability Physics** below for the physical implications.

---

## Initial State and `steady_state_starting.json`

The default initial state is the warmup-settled equilibrium produced by `test_generate_ic.py::test_create_ic`, which runs a 60 s TensionPI warmup sequence with SkewedWakeBEM:

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
1. `pytest simulation/tests/unit/test_generate_ic.py::test_create_ic -s` — writes `simulation/steady_state_starting.json`
2. Stack test reads this file and passes `pos0`, `body_z`, `omega_spin`, `rest_length` to mediator via config
3. `test_steady_flight.py` reads the file but never writes it

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

The current design uses a **unified ground/AP split** with a `TensionCommand` protocol; see [flight_stack.md §4.4](flight_stack.md) for the Lua-side `RAWES_SUB` / `RAWES_ALT` / `RAWES_TEN` contract.

### Components

```
PumpingGroundController (10 Hz)  ──TensionCommand──▶  TensionApController (400 Hz)
                ▲                                                   │
        load cell, hub alt                                    TensionPI collective
                │                                                   ▼
        WinchController (400 Hz, tension-controlled)         elevation rate-limited toward target
```

- **`pumping_planner.PumpingGroundController`** (10 Hz) emits `TensionCommand(tension_setpoint_n, tension_measured_n, alt_m, phase)`. Ground reads the load cell and packs both the setpoint and the measurement so the AP's `TensionPI` can close the loop using ground-transmitted feedback. `alt_m` is smoothly ramped at phase boundaries using `hub_alt_m` telemetry received from the kite at 10 Hz: up over `t_transition` seconds entering "transition", down at the start of each next "reel_out". Sudden `alt_m` jumps are ground-controller bugs — detected by `ap_unreachable_alt`.
- **`ap_controller.TensionApController`** (400 Hz, AP side) runs `TensionPI` collective + rate-limited elevation hold. Caches its own `pos_ned` from `step()` and validates each received command via `BadEventLog`: `ap_impossible_alt` (alt_m > tether_length), `ap_unreachable_alt` (elevation gap > `slew_rate × FEASIBILITY_WINDOW_S = 1 s`). **Blame rule:** `ap_*` events → ground planner sent unreachable commands; slack/tension_spike without `ap_*` → AP tracking failure.
- **`winch.WinchController`** (400 Hz) is tension-controlled: cruise speed proportional to tension error, trapezoidal accel/decel profile, virtual battery accumulates energy_out_j / energy_in_j / net_energy_j.
- **`unified_ground.UnifiedGroundController`** wraps the two with pluggable comms: `DirectComms` (Python AP), `LuaComms` (Lua simtest), `GcsComms` (SITL stack).

### Critical design invariants

- **`winch_target_tension = tension_ic` during reel-out (NOT `tension_out`).** Generator load point is `tension_ic` (300 N). The AP drives tension up to `tension_out` (435 N), giving cruise speed `kp × (435 − 300) = 0.675 m/s` → capped at `v_max_out = 0.40 m/s`. If you mistakenly set `winch_target_tension = tension_out` the AP tension never significantly exceeds the target so the winch barely moves. During reel-in, `winch_target_tension = tension_in` (226 N).
- **Ground owns altitude smoothing; AP must not add a second layer.** The AP already rate-limits elevation at `slew_rate_rad_s = 0.40 rad/s` — that is its smoothing. Adding a second smoothing layer would create two competing integrators. Smoothing happens on the ground via `alt_m` ramps at phase boundaries.

### Winch parameters

`kp = 0.005 (m/s)/N`, `v_max_out = 0.40 m/s`, `v_max_in = 0.80 m/s`, `accel_limit_ms2 = 0.5 m/s²`. Tested in `test_winch_tension_control.py` (23 tests).

### Telemetry — altitude command chain

Three columns, each owned by a different component (see `telemetry_csv.py`):

| Column | Owner | Meaning |
|---|---|---|
| `gnd_alt_cmd_m` | `PumpingGroundController` (10 Hz) | TensionCommand.alt_m sent to AP |
| `elevation_rad` | `TensionApController._el` (400 Hz) | AP's internally rate-limited target |
| `pos_z` | physics | actual hub altitude |

Comparing these three reveals who is at fault when altitude tracking fails.

---

## Landing Architecture

Unified architecture parallel to pumping. See [flight_stack.md §4.5](flight_stack.md) for the Lua-side contract.

```
LandingGroundController (10 Hz) → LandingCommand → LandingApController (400 Hz) + WinchController (400 Hz)
```

### Why vertical descent, not spiral

During a tethered orbit the hub orbits faster as the tether shortens (angular-momentum conservation — figure-skater effect). At short tether lengths the orbital speed exceeds the reel-in rate, the tether goes slack, and tension spikes 400+ N as it snaps taut. A vertical drop directly above the anchor avoids this entirely.

At the end of the De Schutter reel-in, body_z is at xi=80° from the (horizontal) wind vector — only ~10° from horizontal — so leveling is essentially instant.

### Phase sequence

| Phase | body_z | Collective | Winch |
|---|---|---|---|
| `reel_in` | slerps xi~30°→80° | VZ PI (vz_sp=0) | holds at IC length |
| `descent` | fixed (xi~80°) | VZ PI (vz_sp=0.5 m/s) | tension target = 180 N so `kp × (180 − natural_T) ≈ v_land` |
| `final_drop` | hold last | collective = 0 | hold |

Lua landing (mode=4) receives `RAWES_SUB = LAND_FINAL_DROP` (value 1) when `cmd.phase == "final_drop"`.

`LandingPlanner` was deleted — all landing logic now lives in `LandingGroundController` + `LandingApController`. The old "leveling + descent rate controller" approach is gone.

### Fixture

`acro_armed_landing_lua` (`kinematic_vel_ramp_s = 20` so the hub exits kinematic at vel=0 — eliminates linear tether jolt).

### Diagnosis

`analyse_landing.py` — per-bucket table of alt/vz/winch/tension/collective; phase timeline; slack event list with before/after tension and winch speed; tension spike list; descent summary.

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
- Tether: tension-only elastic spring, no sag, no distributed mass, no reel dynamics
- Spin ODE is a separate scalar — not fully coupled with orbital dynamics (spin-axis torque Q_spin updates omega_spin independently; gyroscopic coupling from omega_spin is fed back into Euler's equations each step)
- Anti-rotation motor not modeled (internal force in single-body model)
- Kaman flap aerodynamic response dynamics not modeled (Weyel level B); servo slew rate limits the mechanical input but blade pitch responds instantaneously

---

## Module Map

```
simulation/
├── dynamics.py          RK4 6-DOF rigid-body integrator
├── aero/                Package: AeroBase (ABC) + AeroResult dataclass + create_aero() factory.
│                        PetersHeBEMJit (default, model="jit"), PetersHeBEM (numpy ref),
│                        CCBladeBEM (tabulated polar), OpenFASTBEM (Brent phi-solve), SimpleBEM (sanity ref).
├── tether.py            Tension-only elastic tether (Dyneema SK75)
├── swashplate.py        H3-120 inverse mixing, cyclic blade pitch
├── frames.py            build_orb_frame(), T_ENU_NED (legacy external-data utility only)
├── sensor.py            PhysicalSensor — honest R_hub orientation, NED throughout
├── sitl_interface.py    ArduPilot SITL UDP binary protocol
├── physics_core.py      PhysicsCore — shared 400 Hz physics (dynamics, aero, tether, spin ODE,
│                        angular damping, KinematicStartup, lock_orientation). Used by both
│                        mediator.py and PhysicsRunner (simtests).
├── controller.py        compute_swashplate_from_state, compute_rc_from_attitude, RatePID,
│                        portable core (compute_bz_tether/slerp_body_z/compute_rate_cmd/
│                        col_min_for_altitude_rad), compute_bz_altitude_hold, AltitudeHoldController,
│                        ElevationHoldController, TensionPI (collective PID at 400 Hz; kd=0 default).
├── mediator.py          SITL co-simulation loop — thin wrapper around PhysicsCore
├── mediator_torque.py   Standalone torque SITL mediator (RPM profiles, hub yaw kinematics)
├── torque_model.py      Hub yaw model (kinematic + motor lag) — HubParams (rpm_scale, gear_ratio,
│                        motor_tau), HubState (psi, psi_dot, omega_motor), step(), equilibrium_throttle()
├── kinematic.py         KinematicStartup — hub trajectory during EKF init phase
├── winch_node.py        WinchNode + Anemometer (physics/planner protocol boundary)
├── gcs.py               MAVLink GCS client (arm, mode, RC override, params).
│                        recv_local_position_latest() — non-blocking poll of LOCAL_POSITION_NED.
├── comms.py             MAVLink comms boundary between ground and AP.
│                        VirtualComms — simtest: latency queue + optional Gaussian noise on hub_alt_m;
│                        inject(t, alt) at 400 Hz, receive_telemetry(t)/send_command(t,cmd)/
│                        poll_ap_command(t) at 10 Hz.
│                        MavlinkComms — SITL/hardware: receive_telemetry() polls LOCAL_POSITION_NED;
│                        send_command() sends RAWES_TEN + RAWES_ALT + RAWES_SUB via gcs.py.
├── unified_ground.py    UnifiedGroundController — wraps PumpingGroundController + WinchController
│                        with pluggable comms: DirectComms (Python AP), LuaComms (Lua simtest),
│                        GcsComms (SITL stack). step(t, tension, altitude, dt) at 400 Hz.
│                        net_energy_j property passes through WinchController.net_energy_j.
├── pumping_planner.py   TensionCommand dataclass + PumpingGroundController (10 Hz phase schedule).
│                        step(t_sim, tension_measured_n, rest_length, hub_alt_m) → TensionCommand.
│                        alt_m is smoothly ramped at every phase boundary using hub_alt_m telemetry.
│                        winch_target_length / winch_target_tension properties for WinchController.
├── ap_controller.py     TensionApController (400 Hz AP side): TensionPI collective + rate-limited
│                        elevation hold. receive_command(cmd, dt) validates alt_m against cached
│                        pos_ned (updated each step): ap_impossible_alt (alt > tether_length),
│                        ap_unreachable_alt (elevation gap > slew_rate × 1 s).
│                        log_fields() → {tension_setpoint, elevation_rad, el_correction_rad,
│                        coll_saturated, comms_ok, collective_from_tension_ctrl}.
│                        LandingApController.log_fields() → {elevation_rad, body_z_eq}.
├── winch.py             WinchController — tension-controlled motion profile. set_target(length_m,
│                        tension_n) at 10 Hz; step(tension_measured, dt) at 400 Hz. Cruise speed
│                        proportional to tension error; trapezoidal accel/decel profile. Virtual
│                        battery: energy_out_j / energy_in_j / net_energy_j running totals.
├── telemetry_csv.py     Canonical CSV schema (TelRow, COLUMNS, heartbeat). Altitude command chain:
│                        gnd_alt_cmd_m (ground cmd) / elevation_rad (AP internal) / pos_z (actual).
│                        TelRow.from_physics(runner, step_result, col, wind, **kwargs) is the single
│                        telemetry factory for all simtests.
├── simtest_log.py       SimtestLog (per-test log dir + human-readable summary), BadEventLog
│                        (slack/tension_spike/floor_hit event tracking with phase tagging).
├── rawes_lua_harness.py RawesLua class — runs rawes.lua in-process via lupa; shared by unit
│                        tests and simtests.
├── rawes_modes.py       Python constants mirroring rawes.lua mode/substate numbers.
├── scripts/rawes.lua    Unified Lua controller (SCR_USER6 modes 0/1/4/5).
├── scripts/rawes_test_surface.lua  Test-surface table (_rawes_fns) splicing internal locals
│                        for Python unit tests via lupa.
├── analysis/
│   ├── flight_log.py    Unified data loader — FlightLog.load(log_dir), FlightLog.buckets(bucket_s),
│   │                    FlightEvent, Bucket; reads all log sources into one structure.
│   ├── analyse_run.py   Post-run report: print_flight_report, compute_steady_metrics,
│   │                    validate_ekf_window; CLI: --bucket S.
│   ├── analyse_landing.py    Landing diagnosis (alt/vz/winch/tension/collective per bucket).
│   ├── pump_envelope.py      Pumping cycle envelope sweep (tension setpoints, wind, tilt).
│   └── pump_diagnosis.py     Per-bucket compact summary + CSV (osc, corr) to test log dir.
├── viz3d/
│   ├── visualize_3d.py       Interactive 3D playback of any telemetry.csv (default viz tool).
│   ├── scrub.py              Interactive frame scrubber.
│   ├── render_cycle.py       Off-screen render to MP4 / GIF.
│   ├── telemetry.py          TelemetryFrame dataclass + CSVSource / LiveQueueSource protocol.
│   ├── visualize_torque.py   Torque telemetry 3-panel replay.
│   └── torque_telemetry.py   TorqueTelemetryFrame dataclass.
└── tests/
    ├── unit/            Windows native, no Docker (~685 fast unit tests; no simtests).
    │   └── README.md    Unit & simtest reference guide.
    ├── simtests/        Windows native, no Docker (~13 full physics simulation tests; marker: simtest).
    │   ├── conftest.py            simtest marker registration; 600 s auto-timeout.
    │   ├── simtest_runner.py      PhysicsRunner — thin wrapper around PhysicsCore. AcroControllerSitl
    │   │                          baked in. step(dt, col, rate_roll, rate_pitch, omega_body).
    │   │                          LuaAP(sim, wind, dt, initial_col_rad) / PythonAP(ap, wind, dt)
    │   │                          tick helpers.
    │   └── simtest_ic.py          load_ic() — loads steady_state_starting.json.
    └── sitl/            Docker; all SITL/stack tests live here.
        ├── conftest.py                 thin re-exporter — pytest_addoption + pytest_configure only.
        ├── stack_infra.py              StackConfig, SitlContext, _sitl_stack, _acro_stack, _torque_stack.
        ├── stack_utils.py              port checks, log copy, mediator launcher.
        ├── rawes_sitl_defaults.parm    boot-time ArduPilot params (EEPROM defaults).
        ├── flight/                     flight stack tests (mediator + physics + ArduPilot).
        └── torque/                     torque/anti-rotation tests.
```

**Data flow (400 Hz):** SITL servo PWM → swashplate mix → **PhysicsCore** (aero + tether + RK4 + spin ODE) → sensor packet → SITL. `mediator.py` is a thin wrapper; `PhysicsCore` (`physics_core.py`) owns the integration loop.

---

## Background Academic References

1. **Felix Weyel (2025)** — "Modeling and Closed Loop Control of a Cyclic Pitch Actuated Rotary Airborne Wind Energy System", Bachelor's Thesis, Uni Freiburg.
2. **De Schutter, Leuthold, Diehl (2018)** — "Optimal Control of a Rigid-Wing Rotary Kite System for Airborne Wind Energy".
3. **US Patent US3217809** (Kaman/Bossler, 1965) — Canonical servo-flap rotor control system.

---

## SITL Lockstep Protocol

### How lockstep works

ArduPilot SITL uses a **lockstep** physics protocol:

1. ArduPilot sends a binary servo packet (UDP port 9002) and then **blocks** waiting for a state reply.
2. The physics backend (mediator) receives the packet, integrates one timestep (400 Hz = 2.5 ms), and sends back a JSON state packet.
3. ArduPilot unblocks, processes the state, and emits MAVLink messages with the new `time_boot_ms`.

Because ArduPilot cannot advance until it receives a reply, the physics worker must **reply to every servo packet without exception**. Any attempt to rate-limit the physics loop using sim time will cause ArduPilot to stall permanently.

### sim_now() — ArduPilot's internal clock, not wall-clock

`gcs.sim_now()` returns `time_boot_ms / 1000.0` from the most recently **processed** MAVLink message. This is ArduPilot's internal simulation clock:

- It advances only when MAVLink messages are received (which requires the physics loop to be running).
- At SITL speedup=1 (default), sim time ≈ wall time (roughly 1:1), but they are **not** guaranteed equal.
- Returns 0.0 before the first MAVLink message is received.
- All test timeouts and deadlines (arm, set_mode, wait_ekf_attitude, etc.) are expressed in sim seconds.
- **`sim_now()` is always consistent with the message that caused `_recv` to return.**

### `_recv` internal buffer — clock consistency guarantee

`_recv` drains all available network bytes into an internal deque, then pops and processes messages one at a time. It returns on the first match, leaving the rest for the next call. **Result: `sim_now()` always equals the `time_boot_ms` of the message that caused `_recv` to return** — never a later timestamp from the same network burst. `SimClock.update()` asserts non-decreasing timestamps (zero skipped); out-of-order delivery surfaces immediately as `AssertionError`.

### sim_sleep(N) — waits N sim-seconds, not N wall-seconds

`gcs.sim_sleep(N)` loops calling `_recv(blocking=True, timeout=0.1)` until sim time has advanced by N seconds. It does **not** call `time.sleep()`. Key properties:

- **The physics worker must keep running** while `sim_sleep` is active — otherwise ArduPilot stalls, no MAVLink messages arrive, and `sim_sleep` never returns.
- Every received message — whether it matches a type filter or not — advances the sim-clock and is written to the MAVLink log. The type filter in `_recv` only controls what is *returned* to the caller; discarded messages still tick the clock.
- At speedup=1, `sim_sleep(70)` takes ~70 seconds of real time.

### Anti-pattern: rate-limiting inside a physics worker

```python
# WRONG — deadlock if worker skips replies
def worker():
    while True:
        servos = sitl.recv_servos()
        if sitl.sim_now() - last_send > 0.1:   # rate limiting = skipping replies
            sitl.send_state(...)
            last_send = sitl.sim_now()

# CORRECT — reply to every servo packet
def worker():
    while True:
        servos = sitl.recv_servos()
        if servos is None:
            break
        sitl.send_state(...)   # always reply
```

If the worker skips replying in order to "wait for sim time to advance", ArduPilot blocks, `time_boot_ms` stops advancing, `sim_now()` never crosses the threshold, and the test deadlocks permanently.
