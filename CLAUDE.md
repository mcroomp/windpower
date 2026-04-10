# RAWES — Project Context for Claude

## Project Goal

Build an **ArduPilot flight controller model** for a Rotary Airborne Wind Energy System (RAWES) that can fly in all standard modes: takeoff, stabilized flight, autonomous flight, landing. This is a long-term, step-by-step effort.

**Current phase:** Phase 3, Milestone 3 — Full stack working. 487 unit + 38 simtests + 7 stack tests passing (1 xfailed: test_pumping_cycle kinematic-transition instability). SkewedWakeBEM is production aero model. `rawes.lua` orbit-tracking controller validated in SITL. H_SW_PHANG=0 and H_SW_TYPE=3 confirmed. Test suite rationalized: 8 stack tests (was 10), Lua math tests merged into `test_lua_math.py`. Next: configure GB4008 (H_TAIL_TYPE=4, ATC_RAT_YAW_*, H_COL2YAW), write `rawes_params.parm`, fix test_pumping_cycle xfail.

**Current stack test status:** 7 PASS, 1 XFAIL (test_pumping_cycle — known kinematic-transition instability), 0 failures. Tests: test_arm_minimal, test_acro_armed, test_acro_hold, test_h_swash_phang, test_lua_flight_rc_overrides, test_stack_integration_smoke, test_stationary_gps_fusion all PASS.

See **[Phase 3 Plan](#phase-3-plan)** below for the full milestone breakdown.

---

## What Is This System?

A **RAWES** (Rotary Airborne Wind Energy System) is a tethered, rotating 4-blade rotor kite. It operates like an autogyro rotor — wind drives autorotation, and cyclic pitch control tilts the rotor disk to steer. A tether connects to the bottom of the rotor axis; tension during reel-out drives a ground generator (pumping cycle power generation).

Key distinction from a drone: **no motor drives rotation** — wind does. Control is entirely through blade pitch, actuated indirectly via trailing-edge flaps on each blade.

---

## Control Architecture

Three nested control problems from the source literature:

```
Planned Trajectory → [MPC] → Reference Blade Pitch β̄
                                      ↓
                             [Flap Controller] → Flap Actuation γ_CL
                                      ↑                      ↓
                              Actual Blade Pitch β ← [RAWES Model] → Output
```

- **A — Trajectory → reference pitch:** MPC (Model Predictive Control) — not yet implemented
- **B — Reference pitch → flap actuation:** Feed-forward + PID — implemented in source thesis, not yet in runtime stack
- **C — Physics simulation:** Full nonlinear ODE state space model — source thesis; current stack uses a compact BEM + RK4 model

The ArduPilot integration sits at level A (trajectory planning) and will delegate to level B for actuation.

---

## Physical System

> Full details in [hardware/design.md](hardware/design.md) and [hardware/components.md](hardware/components.md)

**Key parameters:**

| Parameter          | Value         |
|--------------------|---------------|
| Blade count        | 4 (90° apart) |
| Blade length       | 2000 mm       |
| Total rotor radius | ~2500 mm      |
| Rotor mass         | 5 kg          |
| Blade airfoil      | SG6042        |
| Tether diameter    | 1.9 mm (Dyneema SK75) |
| Max tether length  | 300 m         |
| Min altitude       | 10 m          |
| Tether attachment  | Bottom of axle|
| Anti-rotation motor| EMAX GB4008 66KV, 80:44 spur gear |
| Servos S1/S2/S3    | DS113MG V6.0 |
| Flight controller  | Holybro Pixhawk 6C |
| Battery            | 4S LiPo 15.2V, 450 mAh |

**Critical simulation note:** Only the spinning outer hub shell and blades contribute to `I_spin`. The stationary assembly (~1 kg) does not spin. Both bearing friction and push-rod reaction forces are **internal** forces in the single-body model — motor torque must **not** be added as an external couple.

---

## Reference Documents

### Hardware & Physical Design
| File | When to read |
|------|-------------|
| [hardware/design.md](hardware/design.md) | Full assembly layout, rotor geometry, blade design (SG6042), swashplate, Kaman servo flap mechanism (US3217809), anti-rotation motor, electronics and power architecture |
| [hardware/components.md](hardware/components.md) | Detailed component specs: GB4008 motor, REVVitRC ESC, AM32 firmware, DS113MG servos, SiK radio, RP3-H receiver, Boxer M2 transmitter |
| [hardware/flap_sensor_bench.md](hardware/flap_sensor_bench.md) | Bench measurement system for swashplate-to-flap deflection characterisation |
| [hardware/calibrate.md](hardware/calibrate.md) | **calibrate.py reference** — all CLI commands for servo control, motor test, ESC diagnostics (diag/monitor), and Lua script upload over MAVLink. Read when doing bench calibration or diagnosing motor/ESC issues. |

### Control Theory
| File | When to read |
|------|-------------|
| [theory/pumping_cycle.md](theory/pumping_cycle.md) | De Schutter et al. 2018 -- pumping cycle, state variables, aerodynamics, structural constraints, parameter table |
| [theory/orbit_mechanics.md](theory/orbit_mechanics.md) | Beaupoil 2026 -- orbit characteristics, gyroscopic analysis, five control design requirements |
| [theory/flap_dynamics.md](theory/flap_dynamics.md) | Weyel 2025 -- flap state-space model, feed-forward + PID controller, N4SID identification |

### ArduPilot / Firmware
| File | When to read |
|------|-------------|
| [system/stack.md](system/stack.md) | **Complete flight control reference** -- system architecture, ground planner, winch controller, Pixhawk orbit tracker (rawes.lua), yaw trim, ArduPilot configuration, startup/arming sequence, EKF3 GPS fusion analysis, Lua API constraints. **Update whenever new arming or EKF behavior is discovered.** |

### Simulation Internals
| File | When to read |
|------|-------------|
| [simulation/internals.md](simulation/internals.md) | Sensor design, controller functions, aero model (SkewedWakeBEM), tether, pumping cycle COL_MIN rules, initial state, known gaps |
| [simulation/history.md](simulation/history.md) | Phase 2 and Phase 3 M3 decisions -- why SkewedWakeBEM, collective passthrough fix, EKF altitude unreliability, test results |
| [simulation/aero/deschutter.md](simulation/aero/deschutter.md) | De Schutter 2018 equation-level validation -- maps each paper equation (Eq. 25-31, Table I) to its implementation, documents C_{D,T} derivation, beta diagnostic, known gaps vs SkewedWakeBEM |

### Key Design Decisions (this session)
- **RotorAero removed** — `aero/aero_rotor.py` deleted; `create_aero()` factory now has 4 models (SkewedWakeBEM default).
- **Two-loop attitude controller**: `compute_rate_cmd(kp, kd=0)` → rate setpoint; `RatePID(kp=2/3)` → swashplate tilt. Matches hardware architecture where ArduPilot rate PIDs supply damping.
- **Portable core** in `controller.py`: `compute_bz_tether`, `slerp_body_z`, `compute_rate_cmd`, `col_min_for_altitude_rad` — frame-agnostic, map 1:1 to Lua (`rawes.lua`). No C++ custom firmware planned.
- **High-tilt De Schutter**: ξ=80° viable. AoA stays below stall (14.4°) because low v_axial at high tilt reduces inflow angle. Requires `col_max=0.10 rad`, `col_min_reel_in=0.079 rad`. BEM invalid above ξ≈85°.
- **body_z_slew_rate** = `rotor.body_z_slew_rate_rad_s` = 2% of gyroscopic limit = **0.40 rad/s** for beaupoil_2026. Optimal from sweep; faster than 0.40 causes oscillation, slower wastes reel-in time.
- **`swashplate_pitch_gain_rad`** added to YAML/RotorDefinition — physically measurable via flap deflection angle × tau at full stick deflection.
- **Visualizer** (`viz3d/visualize_3d.py`): create-once actor pattern (`user_matrix` for rotor/hub/arrows, `.points` in-place for tether/trail), wall-clock while loop, linear interpolation between telemetry frames, net energy HUD. Swashplate inset (bottom-right) shows the schematic oriented by the actual hub rotation matrix (`R_hub → ENU`), so the diagram always matches the orientation seen in the main view. Inset camera is non-interactive and tracks the main camera direction every loop iteration. `--no-inset` disables the inset renderer.
- **Canonical telemetry schema** (`simulation/tests/unit/tel.py`): all simtests use `make_tel()` to produce a uniform JSON dict. Keys: `t_sim, phase, pos_ned, vel_ned, R, altitude_m, omega_rotor, collective_rad, tilt_lon, tilt_lat, tether_tension, tether_rest_length, tether_slack, body_z_eq, wind_ned, tension_setpoint`. Extra test-specific keys (e.g. `xi_level_deg`, `wind_est_ned`) passed as `**extra`. `viz3d/telemetry.py` JSONSource reads this schema directly (`collective_rad` → `swash_collective`, `tilt_lon/lat` → `swash_tilt_*`).
- **`rawes.lua` unified Lua controller** — single script replaces `rawes.lua` + `rawes.lua`; SCR_USER7 selects mode (0=none, 1=flight, 2=yaw, 3=both). Flight mode SITL-validated via `test_lua_flight_rc_overrides`. See [Lua API Constraints](#lua-api-constraints-this-ardupilot-build) below.
- **`SkewedWakeBEMJit`** (`aero/aero_skewed_wake_jit.py`) — Numba `@njit` drop-in replacement for `SkewedWakeBEM`; two kernels (`_jit_vi0`, `_jit_strip_loop`); 18-test equivalence suite (`test_skewed_wake_jit.py`, atol=1e-10). Select via `create_aero(model="jit")`.
- **`aero_skewed_wake.py` rewritten for clarity** — non-JIT reference version; explicit double for-loop, `np.cross()`, `_prandtl_F()` helper; all intermediate numpy broadcasting and dead code removed. Full class + method docstrings added. Only purpose is human-readable reference and JIT equivalence validation.
- **De Schutter 2018 aero audit** — `DeSchutterAero` compared to paper equations 25–31. Two additions: (1) **β side-slip** (`last_sideslip_mean_deg`) as validity diagnostic (Eq. 27-28); β does NOT enter force formulas. (2) **C_{D,T}=0.021** structural parasitic drag (Eq. 29, 31) added to blade CD; derived from cable geometry in Table I. Bug fixed: induction bootstrap floor `max(T,0.01)` replaced with `abs(T)` — floor caused phantom 200 N thrust at zero collective. Validation doc at `simulation/aero/deschutter.md`.
- **`CD_structural`** field added to `RotorDefinition` and YAML files; `beaupoil_2026.yaml` = 0.0 (direct spar mount), `de_schutter_2018.yaml` = 0.021 (thin cable arms).
- **Two orbit-tracking algorithms** — `orbit_tracked_body_z_eq` (azimuthal, Z-preserving) and `orbit_tracked_body_z_eq_3d` (3D Rodrigues, Lua-equivalent). The 3D version matches `rawes.lua` exactly but requires a downstream rate-limited slerp; without it the altitude component of the setpoint creates positive feedback (hub sinks → setpoint more horizontal → less lift → more sinking). Inner-loop physics tests (`test_steady_flight`, `test_closed_loop_60s`) use the azimuthal version because they have no separate altitude controller. `mediator.py` and all pumping-cycle tests use `OrbitTracker` (3D + slerp), which mirrors the Lua. Lua/Python algorithm equivalence is verified in `test_lua_math.py`.
- **`OrbitTracker` class** — encapsulates `orbit_tracked_body_z_eq_3d` + `slerp_body_z`; mirrors Lua's `_bz_orbit`/`_bz_slerp` state machine. `update(pos, dt, bz_target=None)` returns the slerped body_z; pass `bz_target` for planner overrides (e.g. landing leveling). Replaces the 20-line slerp block that was duplicated across 6 files.
- **`WinchNode` protocol boundary** (`winch_node.py`) — enforces hardware MAVLink boundary in simulation. Mediator calls `update_sensors(tension, wind_world)` (physics side only); planner calls `get_telemetry()` returning `{tension_n, tether_length_m, wind_ned}` and `receive_command(speed, dt)` (planner side only). Wind seed for `WindEstimator` comes from `Anemometer.measure()` at 3 m height, not from `wind_world` directly. Prevents simulation from cheating by accessing physics state unavailable on hardware.
- **Vertical landing validated** (`test_landing.py`, `test_pump_and_land.py`) — landing is a vertical drop directly above the anchor, not a spiral descent. Key facts: (1) At the end of De Schutter reel-in the disk is already at xi=80° from the horizontal wind = only ~10° from horizontal; leveling is instant. (2) The nearly-vertical tether supports >95% of hub weight throughout descent. (3) Descent rate controller (COL_CRUISE + KP_VZ * vz_error) replaces TensionPI during landing — TensionPI oscillates when the tether briefly goes slack; the vz controller does not. (4) Tether pendulum effect naturally centres hub over the anchor: touchdown 0.5–0.7 m from anchor, 2° disk tilt, 0 m/s. (5) Floor hit may occur during DESCENT phase (tether briefly slack), not only during final_drop — floor_hit detection covers both phases; the crash-guard `floor_hits > 200` break is restricted to pumping phase only (landing floor contact is expected and correct). Full sequence (pumping + landing) validated end-to-end in test_pump_and_land.py: +1857 J net energy, 92 s total. See `system/stack.md §7.2` and `simulation/internals.md §Landing Architecture`.
- **Gyroscopic swashplate phase compensation NOT needed** — with `BASE_K_ANG=50 N·m·s/rad`, gyroscopic coupling time constant τ = I_spin/k ≈ 0.08 s — fully damped before completing one orbit. Any `swashplate_phase_deg ≠ 0` adds wrong cyclic and degrades orbit stability. Phase=0° is stable; phase=90° destabilises. Confirmed in `test_gyroscopic_orbit.py` (`test_gyro_baseline_stable`, `test_gyro_90deg_not_helpful`, `test_gyro_phase_sweep`). This is consistent with H_SW_PHANG=0 being correct for the hardware swashplate.
- **`AcroController` `use_servo=True` required for De Schutter simtests** — `use_servo=False` (default) allows instantaneous tilt changes that can produce brief diving transients at the start of reel-in. All De Schutter cycle simtests (`test_deschutter_cycle.py`, `test_deschutter_wind.py`, `test_kinematic_transition.py`) use `AcroController.from_rotor(_ROTOR, use_servo=True)` to match SITL behavior (25 ms servo lag on tilt output). Inner-loop tests (`test_steady_flight`, `test_closed_loop_60s`) do not need it.
- **De Schutter power-law wind tension calibration** — with power-law wind (alpha=1/7, 10 m/s at 10 m), hub orbit at ~15 m altitude sees ~10.6 m/s wind and naturally generates ~250 N tether tension. TensionPI setpoint must be `tension_out=250 N` (not 200 N used for uniform wind); using 200 N causes TensionPI to reduce collective → descent spiral → floor hits. Also: `col_min_reel_in_rad` for the reel-in floor must use the actual power-law wind speed at hub altitude (computed via `col_min_for_altitude_rad(aero, xi, mass, wind_m_s=actual_wind)`).
- **`tether_relative` sensor mode removed** — `SensorSim` class deleted from `sensor.py`; `make_sensor()` always returns `PhysicalSensor`. `TetherRelativeHoldController` deleted from `controller.py`; `make_hold_controller(anchor_ned)` always returns `PhysicalHoldController`. All stack tests now run in physical sensor mode (actual orbital-frame attitude, roll=124°/pitch=-46° at tether equilibrium). The tether_relative mode was a workaround that masked the real EKF arming challenge. `test_full_stack_integration` was removed — its MAVLink bidirectionality assertion was folded into `test_acro_armed`.

### Counter-Torque Motor Simulation
| File | When to read |
|------|-------------|
| [simulation/torque/README.md](simulation/torque/README.md) | **Complete reference** — physics model, motor specs, gear efficiency, ArduPilot integration, Lua feedforward controller, hardware deployment. Stack tests are in `simulation/tests/stack/` (unified with flight stack). |

### Lua Flight Controller
| File | When to read |
|------|-------------|
| [simulation/scripts/rawes.lua](simulation/scripts/rawes.lua) | Unified Lua controller (SCR_USER7: 0=none, 1=flight cyclic, 2=yaw trim, 3=both) |
| [simulation/scripts/calibrate.py](simulation/scripts/calibrate.py) | Hardware calibration tool — servo/motor/ESC/script management over MAVLink. See [hardware/calibrate.md](hardware/calibrate.md) for full CLI reference. |

---

## Lua API Constraints (this ArduPilot build)

These constraints apply to `rawes.lua` (and any future Lua scripts) running on the ArduPilot SITL Docker image and on the Pixhawk 6C with the same firmware.

| What you'd expect | What actually works |
|---|---|
| `ahrs:get_rotation_body_to_ned()` | Doesn't exist. Use `ahrs:body_to_earth(v)` and `ahrs:earth_to_body(v)` |
| `Vector3f(x, y, z)` | Constructor ignores args (warning + wrong value). Use `Vector3f()` then `:x()/:y()/:z()` setters |
| `v:normalized()` | Doesn't exist. Copy then `:normalize()` in-place: `local r = v3_copy(v); r:normalize(); return r` |
| `vec * scalar` or `vec + vec` | `*` not overloaded; `+` may silently fail. Use component arithmetic directly |
| `rc:set_override(chan, pwm)` | Doesn't exist. Correct API: `rc:get_channel(n):set_override(pwm)` (cache channel at module load) |
| ArduCopter ACRO mode = 6 | ACRO = **1**. Mode 6 is RTL. Use `vehicle:get_mode() == 1` |

**SCR_ENABLE bootstrap:** After wiping EEPROM, scripting does NOT start from `copter-heli.parm` defaults on the first cold boot in this build. Scripting starts only when `SCR_ENABLE=1` is already in EEPROM from a previous session. The `acro_armed_lua` fixture handles this by NOT wiping EEPROM and setting `SCR_ENABLE=1` via MAVLink post-arm (persists to EEPROM for future boots). `copter-heli.parm` already contains `SCR_ENABLE 1` but this only takes effect on the second boot.

**Anchor position in `LOCAL_POSITION_NED`:** After the NED migration, `initial_state["pos"][2]` is NED Z (negative ≈ −7.12 for altitude 7.12 m above ground). The anchor in `LOCAL_POSITION_NED` frame is at `[0, 0, -initial_state["pos"][2]]` = +7.12 m Down from EKF origin. So `SCR_USER5 = -home_z_enu` (negate).

---

## Simulation Architecture

### Design Philosophy — Three-Phase Validation

1. **Physics validation** — does tether + aero + dynamics produce stable autorotation? (`test_steady_flight.py`)
2. **Closed-loop controller validation** — does `compute_swashplate_from_state` stabilize the physics model without ArduPilot? (`test_closed_loop_60s.py`)
3. **ArduPilot integration** — can ArduPilot's ACRO/GUIDED modes fly the hub once physics + control are known good? (`test_guided_flight.py`)

Phases 1 and 2 run on Windows with no Docker. Phase 3 requires Docker + ArduPilot SITL.

### Module Map

```
simulation/
├── Core physics
│   ├── dynamics.py          RK4 6-DOF rigid-body integrator
│   ├── aero.py              Facade: re-exports all aero models, create_aero() factory → SkewedWakeBEM
│   ├── aero/                Per-blade BEM models (SkewedWakeBEM production; SkewedWakeBEMJit fast JIT; DeSchutterAero reference)
│   ├── tether.py            Tension-only elastic tether (Dyneema SK75)
│   └── swashplate.py        H3-120 inverse mixing, cyclic blade pitch
│
├── Coordinate frames
│   └── frames.py            build_orb_frame(), T_ENU_NED (legacy utility) — single source
│
├── Sensor & interface
│   ├── sensor.py            build_sitl_packet(), PhysicalSensor — all NED, no frame conversion needed
│   └── sitl_interface.py    ArduPilot SITL UDP binary protocol
│
├── Control
│   └── controller.py        compute_swashplate_from_state()  — truth-state controller
│                            compute_rc_rates()               — ArduPilot RC override controller
│                            compute_rc_from_attitude()       — ATTITUDE message controller
│                            RatePID                          — simulated ACRO inner rate loop
│                            compute_bz_tether/slerp_body_z/compute_rate_cmd  — portable core (Lua/C++ portable)
│                            col_min_for_altitude_rad()       — aero-derived altitude floor collective
│                            orbit_tracked_body_z_eq()        — azimuthal orbit tracker (Z-preserving, inner-loop tests)
│                            orbit_tracked_body_z_eq_3d()     — 3D Rodrigues orbit tracker (Lua-equivalent)
│                            OrbitTracker                     — stateful 3D orbit tracker + slerp (mirrors Lua state machine)
│
├── Orchestration
│   ├── mediator.py          400 Hz co-simulation loop (SITL ↔ physics)
│   ├── winch_node.py        WinchNode + Anemometer — protocol boundary between physics and planner
│   └── gcs.py               MAVLink GCS client (arm, mode, RC override, params)
│
├── Reporting
│   └── flight_report.py     Multi-panel flight report plotter
│
├── Analysis tools
│   └── analysis/
│       ├── analyse_run.py             Post-run structured report (always run after stack test)
│       ├── generate_flight_report.py  Offline report from mediator telemetry CSV
│       ├── redraw_flight_report.py    Regenerate PNG from saved flight_data.json
│       └── merge_logs.py              Unified log timeline merger
│
└── tests/
    ├── unit/                Windows native, no Docker
    │   ├── test_closed_loop_60s.py           ★ 60 s closed-loop physics (dynamics+aero+tether+controller)
    │   ├── test_closed_loop_60s.py           60 s orbit — uses RatePID two-loop architecture
    │   ├── test_steady_flight.py             Open-loop equilibrium → writes steady_state_starting.json
    │   ├── test_controller.py                Unit tests (incl. RatePID, portable core functions)
    │   ├── test_aero_trajectory_points.py    SkewedWakeBEM at De Schutter trajectory operating points
    │   ├── test_deschutter_cycle.py          De Schutter pumping cycle (ξ=80°, col_max=0.10 rad)
    │   ├── test_deschutter_equations.py      ★ Eq-level validation of DeSchutterAero vs paper (32 tests)
    │   ├── test_deschutter_wind.py           Wind estimator integration with De Schutter planner
    │   ├── test_skewed_wake_jit.py           SkewedWakeBEMJit equivalence to reference (18 tests, atol=1e-10)
    │   ├── test_wind_estimator.py            Rolling-window wind estimator unit tests
    │   ├── test_lua_flight_logic.py          ★ Rodrigues rotation, orbit tracking, slerp, cyclic error (rawes.lua math)
    │   ├── test_lua_math.py                 ★ Lua/Python algorithm equivalence (Rodrigues, orbit tracking, slerp, PWM)
    │   ├── test_landing.py                   ★ Vertical landing from xi=80° high-elevation hover (20 m tether)
    │   ├── test_pump_and_land.py             ★ Full De Schutter pumping cycle + vertical landing (~50 m)
    │   └── ...
    └── stack/               Docker required
        ├── stack_utils.py           Shared constants + helpers (env vars, logging, process launch/teardown, port kill, log copy)
        ├── conftest.py              acro_armed + acro_armed_lua fixtures (full stack lifecycle)
        ├── test_guided_flight.py    60 s ACRO hold with tether-alignment RC controller
        ├── test_pumping_cycle.py    Pumping cycle stack test
        ├── test_lua_flight.py       ★ rawes.lua flight-mode (SCR_USER7=1) orbit-tracking validation
        └── test_setup.py            Verifies setup reaches armed ACRO
```

### Data Flow (400 Hz mediator loop)

```
ArduPilot SITL (UDP 9002)
  │ servo PWM [16 channels]
  ▼
mediator.py
  ├─ swashplate.h3_inverse_mix()        → collective, tilt_lon, tilt_lat
  ├─ aero.compute_forces()              → F_world[6] (NED wrench)
  ├─ tether.compute()                   → F_tether, M_tether (added to wrench)
  ├─ dynamics.step()                    → {pos, vel, R, omega} (NED)
  └─ sensor.build_sitl_packet()         → {pos_ned, vel_ned, rpy, accel_body, gyro_body}
  │ JSON state packet
  ▼
ArduPilot SITL (UDP 9003)
```

### Coordinate Conventions

All defined in `frames.py`. Import from there — do not duplicate.

| Frame | Axes | Used by |
|-------|------|---------|
| NED (world) | X=North, Y=East, Z=Down | dynamics, aero, tether, controller, sensor, SITL |
| Body | columns of R_hub | gyro, accel, swashplate commands |

**Gravity:** `[0, 0, +g·m]` in NED (Z=Down). Altitude = `-pos[2]`.

**T_ENU_NED:** `frames.py` keeps this as a legacy utility for converting ENU data (e.g. old JSON files). Not used in the simulation loop.

**Orbital frame:** `build_orb_frame(body_z)` from `frames.py`. Removes rotor spin; body X = East (NED Y) projected onto disk plane.

> ⚠️ **NED-only policy:** The entire codebase uses NED exclusively. We migrated from a mixed ENU/NED codebase — there may be vestigial ENU references (variable names, comments, formulas) that survived the migration and must be removed when found. If you see `pos_enu`, `home_z_enu`, `pos_ENU`, ENU-style altitude arithmetic (`altitude = pos[2]` without negation), or similar, treat it as a bug. Altitude above ground = `-pos_ned[2]`. The `T_ENU_NED` utility in `frames.py` exists only for converting legacy external data, never for internal simulation state.

---

## Workflow Rules

**No silent defaults for physics parameters.** Do not use `dict.get("key", fallback)` or `x = x or default` for any value that is a physical constant, control gain, mechanical limit, or rotor/airfoil property. If a required config key is absent, raise `KeyError` or `ValueError` explicitly so the gap is visible at startup, not hidden at runtime. The only acceptable defaults in function signatures are structural/optional flags (e.g. `wind_world=None`, `spin_angle=0.0`). Convenience defaults like `_PITCH_GAIN_RAD_DEFAULT = 0.3` are the exact anti-pattern to avoid — they make it impossible to audit the effective configuration and silently produce wrong physics when a YAML field is missing.

**Do NOT consult git history (`git log`, `git diff`, `git show`, `git blame`) when diagnosing problems unless you first ask the user whether that would make sense.** Diagnose from the current code and runtime logs instead. Git history is rarely the right tool for debugging and adds noise to the investigation.

**⚠️ NEVER use non-ASCII characters in Python `print()` output** (no `─`, `✓`, `✗`, `σ`, `→`, `−`, `∫`, `•`, etc.). Python scripts run on Windows with cp1252 encoding by default; non-ASCII chars cause `UnicodeEncodeError` and crash analysis scripts. Use only 7-bit ASCII: `-` for lines, `[PASS]`/`[FAIL]` for status, `sd=` for standard deviation, `-` for minus, etc.

---

## Running Tests

Tests run in three sequential stages. Always run them in order — later stages depend on earlier ones passing.

**CRITICAL: Unit tests and simtests run on Windows natively (no Docker). Stack tests (flight + torque) require Docker. Never mix these.**

**CRITICAL: Use the Bash tool directly for unit/simtests and for Docker commands — do NOT use `wsl.exe`. The Bash tool runs Git Bash on Windows, and Docker Desktop exposes the `docker` CLI directly in Git Bash. WSL `/mnt/...` paths do NOT exist in Git Bash.**

**CRITICAL: The Bash tool's working directory is NOT the repo root. Always use absolute paths.**

**CRITICAL: NEVER call `docker exec` directly to run stack tests.** Always use `bash sim.sh test-stack [...]`. The `dev.sh` scripts manage process teardown and port cleanup between tests — calling `docker exec ... pytest` directly bypasses this cleanup and leaves SITL processes holding ports, causing subsequent tests to fail with "Address already in use". If a port is stuck after an aborted run, restart the container with `bash sim.sh stop && bash sim.sh start`.

One-time venv setup (Windows, run from repo root in Git Bash):
```bash
bash sim.sh setup
```
This creates `simulation/.venv` and installs `simulation/requirements.txt` (Windows-safe packages).

---

### Stage 1 — Unit tests (fast, Windows, no Docker)

Pure Python: physics, aero, tether, controller, sensor, planner. No ArduPilot, no network, no Docker.

```bash
bash sim.sh test-unit -q
```

Expected: ~460 tests, ~65 s. Fix all failures here before proceeding.

---

### Stage 2 — Simtests (slow, Windows, no Docker)

Full closed-loop physics (dynamics + aero + tether + attitude controller) for 10–60 s. No ArduPilot.

```bash
bash sim.sh test-simtest -q
```

Expected: ~29 tests, ~5 min. Fix all failures here before proceeding.

To run both stages together:
```bash
bash sim.sh test-unit -q && bash sim.sh test-simtest -q
```

---

### Stage 3 — Stack tests (Docker, ArduPilot SITL)

Full SITL co-simulation: mediator + ArduPilot + MAVLink GCS. Runs sequentially — never launch two stack tests at the same time. Includes both flight orbit tests and counter-torque motor tests.

```bash
bash sim.sh test-stack -v
```

Filtered runs:
```bash
bash sim.sh test-stack -v -k test_pumping_cycle
bash sim.sh test-stack -v -k test_lua_yaw_trim
bash sim.sh test-stack -v -k torque_armed    # all counter-torque tests
```

**Output is filtered by default** (summary mode: PASSED/FAILED lines + failure details only).
Full raw output always saved to `simulation/logs/pytest_last_run.log`. After the run:
```
[LOGS] summary: C:\repos\windpower\simulation\logs\pytest_last_run_summary.json
[LOGS] full:    C:\repos\windpower\simulation\logs\pytest_last_run.log
```
Read `pytest_last_run_summary.json` for pass/fail counts and failed test list. Do NOT re-run.

**CRITICAL: Per-test log directories.** Each stack test writes its own logs to `simulation/logs/{test_name}/` (e.g., `simulation/logs/test_h_swash_phang/`). These contain: `mediator.log`, `sitl.log`, `gcs.log`, `telemetry.csv`, **`arducopter.log`**. Always read these per-test logs when diagnosing a failure — **never look at `/tmp/ArduCopter.log` inside the container**, which accumulates across all test sessions and is stale. The per-test `arducopter.log` is truncated before each SITL launch so it contains only that test's SITL output.

Other filter modes:
```bash
bash sim.sh test-stack --filterstatus   # failures section only
bash sim.sh test-stack --raw            # full unfiltered output
```

**Always run `analyse_run.py` after a stack test:**
```bash
bash sim.sh exec 'python3 /rawes/simulation/analysis/analyse_run.py'
bash sim.sh exec 'python3 /rawes/simulation/analysis/analyse_run.py --plot'
```

---

### Other commands

| Task | Command |
|------|---------|
| Regenerate steady state | `bash sim.sh test-unit -k test_steady_flight` |
| Torque visualizer | `bash sim.sh exec 'python3 /rawes/simulation/torque/visualize_torque.py'` |
| 3D visualizer (live) | `cd simulation && .venv/Scripts/python.exe viz3d/visualize_3d.py logs/telemetry_pump_and_land.json` |
| Export MP4 | `cd simulation && .venv/Scripts/python.exe viz3d/visualize_3d.py <telemetry.json> --export <out.mp4> --fps 30` |
| Build Docker image | `bash sim.sh build` |
| Start/stop container | `bash sim.sh start` / `bash sim.sh stop` |
| Add Windows package | Add to `simulation/requirements.txt`, re-run `bash sim.sh setup` |
| Add Docker package | Add to `simulation/requirements-docker.txt`, rebuild image |

**MP4 export** requires `imageio` and `imageio-ffmpeg` (both in `requirements.txt`).
`imageio-ffmpeg` bundles its own ffmpeg binary — no system ffmpeg needed.
The visualizer auto-computes spin substeps to avoid stroboscopic blade aliasing.

**⚠️ Rebuilding the Docker image — CRITICAL rules:**

1. **Always pass `--build-arg INSTALL_ARDUPILOT=true`** (or use `build.cmd ardupilot`). Running `docker build` without this arg replaces the ArduPilot image with a base-only 1.9 GB image — ArduPilot binary vanishes and all stack tests fail with `RAWES_SIM_VEHICLE` errors.

2. **The full build takes ~30–60 min** (ArduPilot clone + waf compile). Use `run_in_background=true` on the Bash tool call **without** a trailing `&` inside the command. Adding `&` inside causes the shell to exit early and SIGHUP kills the Docker build mid-compile, producing a corrupt image with exit code 0.

3. After rebuilding: `dev.sh stop && dev.sh start` to swap the container to the new image.

Build Docker image (drive-independent):
```bash
bash sim.sh build
```
| Python analysis script | `bash sim.sh exec 'python3 /rawes/simulation/...'` |
| One-off inside container | `bash sim.sh exec 'python3 /rawes/simulation/...'` |

Last run logs: `simulation/logs/` — `pytest_last_run_summary.json` (machine-readable, read this first), `pytest_last_run.log` (full raw), `mediator_last_run.log`, `sitl_last_run.log`, `gcs_last_run.log`, `telemetry.csv`

**Path note:** `sim.sh` (repo root) converts paths automatically for any drive. Do not hardcode `/mnt/X/...` WSL paths — use `sim.sh` instead.

---

## ArduPilot ACRO Mode — Design Notes

ACRO mode was chosen because:
- GUIDED's attitude-hold loop fights the tether equilibrium (~65° from vertical) with maximum cyclic
- ACRO only damps angular rates toward commanded RC rates

**⚠️ DO NOT switch to STABILIZE mode.** STABILIZE has been tried and always crashes:
- Holds absolute NED attitude (roll=0=level), which fights the 65° tether equilibrium
- Yaw discontinuity at unfreeze triggers EKF emergency yaw reset → crash within 4 s

**Key problem with neutral sticks in ACRO:** Zero rate command nulls all angular rates including the hub's natural tether-orbit precession → non-zero cyclic → crash. Solution: `compute_rc_from_attitude` sends corrective rates matching the tether-relative attitude error.

**ACRO integrator windup:** ACRO's I-term accumulates the hub's orbital angular velocity (0.2–0.3 rad/s) as a persistent rate error, building ever-growing cyclic tilt. Fix: `ATC_RAT_RLL_IMAX=ATC_RAT_PIT_IMAX=ATC_RAT_YAW_IMAX=0`.

**Sensor consistency (must all agree or EKF triggers emergency yaw reset):**
1. `velocity_ned` heading must match `rpy[2]` — both from `atan2(vE, vN)`
2. `gyro_body` in yaw-aligned NED body frame: spin stripped, then `Rz(-yaw) @ omega_nospin`
3. `accel_body`: `Rz(-yaw) @ (accel_world_ned - [0,0,9.81])`

See [simulation/internals.md](simulation/internals.md) for full sensor/controller design details.

---

## Current Implementation Limits

1. **Single-body hub model** — no blade multibody dynamics, no flapping DOF
2. **Rotor spin is a scalar ODE** — driven by `Q_spin` from SkewedWakeBEM; gyroscopic coupling (`M_spin`) computed but effect is small at current I_spin
3. **Tether is tension-only elastic** — no sag, no distributed mass, no reel dynamics
4. **Aerodynamics are steady-state BEM** — no dynamic inflow, no stall hysteresis; Coleman skewed wake handles non-uniform induction
5. **Anti-rotation motor not modeled** — internal force in single-body model; cancels out
6. **Controller runs at 10 Hz in stack test** — ACRO RC override via MAVLink; truth-state controller runs at 400 Hz (unit tests and internal controller mode)

---

## Phase 3 Plan

Four milestones. M1+M2 complete. M3 in progress (stack test passing with SkewedWakeBEM). M4 not started.

### M1 — Wire Pumping Cycle into Mediator ✅
- TensionController, orbit_tracked_body_z_eq, blend_body_z → `controller.py`
- Pumping cycle state machine in `mediator.py` (REEL_OUT/REEL_IN phases, WinchNode, TensionController)
- **Gate:** All unit tests pass, test_acro_hold passes ✓

### M2 — Force Balance Audit & Rotor Abstraction ✅
- `rotor_definition.py` — full RotorDefinition API + YAML files (`beaupoil_2026.yaml`, `de_schutter_2018.yaml`)
- Mediator/aero/dynamics fully wired to rotor definition (no hardcoded rotor constants)
- **Gate:** 265 unit tests pass ✓

### M3 — ArduPilot Configuration & Pumping Cycle Stack Test (in progress)
- [x] Run test_pumping_cycle — PASSED with SkewedWakeBEM (reel-out 199 N, reel-in 86 N, net energy +1396 J)
- [x] Switch production aero to SkewedWakeBEM (per-blade BEM + Prandtl + Coleman); 487 fast unit + 38 simtests passing
- [x] Design `ModeRAWES` firmware architecture — documented in `system/stack.md`
- [x] Write and validate orbit-tracking controller in SITL — `test_lua_flight_rc_overrides` PASSES; equilibrium captured at t≈0.5 s; 31 unit tests for Lua math (Rodrigues, orbit tracking, slerp, cyclic projection); unified into `rawes.lua` (SCR_USER7 mode selection)
- [x] Confirm H_SW_TYPE=3 (H3-120) — `test_h_swash_phang` PASSES; default is already H3_120 (value 3, not 1)
- [x] Determine H_PHANG — `test_h_swash_phang` empirical step-cyclic test: H_SW_PHANG=0 confirmed; cross_ch1=1.5%, cross_ch2=19.7%; ArduPilot H3_120 +90° roll advance angle already aligns with RAWES servo layout (S1=0°/East, S2=120°, S3=240°)
- [ ] Configure GB4008: H_TAIL_TYPE=4 (DDFP), tune ATC_RAT_YAW_* and H_COL2YAW feedforward
- [ ] Write `rawes_params.parm` (full parameter file for Pixhawk 6C)
- **Gate:** rawes_params.parm exists + H_PHANG determined

### M4 — Hardware-in-the-Loop (Pixhawk 6C)
- [ ] Write `hil_interface.py` — MAVLink HIL_SENSOR / HIL_GPS / HIL_ACTUATOR_CONTROLS
- [ ] Add --hil-mode --hil-port to mediator
- [ ] Confirm IMU mounting orientation (AHRS_ORIENTATION)
- [ ] Write `test_hil_interface.py`
- [ ] Document HIL bench procedure in system/stack.md
- **Gate:** test_hil_interface.py passes + successful 60 s HIL telemetry log
