# RAWES — Project Context for Claude

## Project Goal

Build an **ArduPilot flight controller model** for a Rotary Airborne Wind Energy System (RAWES) that can fly in all standard modes: takeoff, stabilized flight, autonomous flight, landing. This is a long-term, step-by-step effort.

**Current phase:** Phase 3, Milestone 3 — Pumping cycle stack test PASSED. SkewedWakeBEM is production aero model; RotorAero fully removed. Counter-torque motor simulation complete (Lua feedforward in SITL). **High-tilt De Schutter cycle validated** — ξ=80° reel-in achieves +24% net energy vs ξ=55° baseline; requires `col_max=0.10`, `col_min_reel_in=0.079`, `body_z_slew_rate=0.40 rad/s` (all now derived from rotor definition). **Two-loop attitude controller** implemented (`RatePID` class + `compute_rate_cmd`); `test_closed_loop_60s` uses it with `kd=0`. **Portable core** (`compute_bz_tether`, `slerp_body_z`, `compute_rate_cmd`, `col_min_for_altitude_rad`) in `controller.py` — maps 1:1 to planned Lua/C++ Mode_RAWES. **`rawes_flight.lua` orbit-tracking controller validated in SITL** — `test_lua_flight_rc_overrides` PASSES; captures equilibrium at t≈0.5 s, produces cyclic RC overrides. **H_SW_PHANG=0 confirmed** — `test_h_swash_phang` PASSES; ArduPilot H3_120 +90° roll advance angle already aligns with RAWES servo layout; H_SWASH_TYPE=3 (H3_120) is the correct default enum value (not 1). 398 fast unit tests + 23 simtests passing. Next: configure GB4008 (H_TAIL_TYPE=4, ATC_RAT_YAW_*, H_COL2YAW), then write `rawes_params.parm`.

**Known stack test status:** `test_pumping_cycle`, `test_gps_fuses_during_startup`, `test_acro_armed`, `test_stationary_gps_fusion`, `test_arm_minimal`, `test_stack_integration`, **`test_lua_flight_rc_overrides`**, **`test_h_swash_phang`** all PASS. `test_acro_hold` FAILS — hub descends below 2 m during neutral-stick hold due to insufficient thrust margin after the 45 s kinematic damping phase; unit-level equivalent (`test_closed_loop_60s`) passes.

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

> Full details in [physical_design.md](physical_design.md) and [hardware.md](hardware.md)

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
| [physical_design.md](physical_design.md) | Full assembly layout, rotor geometry, swashplate, servo flaps, all component specs |
| [hardware.md](hardware.md) | Detailed component specs, power architecture, wiring |
| [flaprotordesign.md](flaprotordesign.md) | Blade design, airfoil selection (SG6042), Reynolds numbers, flap prototype |
| [servoflaps.md](servoflaps.md) | Kaman flap mechanism (US patent US3217809), swashplate linkage path |

### Control Theory
| File | When to read |
|------|-------------|
| [deshutter.md](deshutter.md) | De Schutter et al. 2018 — pumping cycle, state variables, aerodynamics, structural constraints, parameter table |

### ArduPilot / Firmware
| File | When to read |
|------|-------------|
| [simulation/ARMING.md](simulation/ARMING.md) | **Arming reference** — RSC modes, motor interlock, EKF init sequence, failure modes. **Update whenever new arming behavior is discovered.** |
| [simulation/heliparams.md](simulation/heliparams.md) | **EKF3 GPS fusion** — exact fusion conditions, required params, 10 s GPS-check delay. Read before debugging EKF3 CONST_POS_MODE. |
| [simulation/raws_mode.md](simulation/raws_mode.md) | **ModeRAWES complete spec** — MAVLink protocol, C++ firmware, orbit tracking, tension PI, omega_spin, ArduPilot parameter table, GB4008 config, Kaman flap lag notes. |

### Simulation Internals
| File | When to read |
|------|-------------|
| [simulation/sim_internals.md](simulation/sim_internals.md) | Sensor design, controller functions, aero model (SkewedWakeBEM), tether, pumping cycle COL_MIN rules, initial state, known gaps |
| [simulation/history.md](simulation/history.md) | Phase 2 and Phase 3 M3 decisions — why SkewedWakeBEM, collective passthrough fix, EKF altitude unreliability, test results |

### Key Design Decisions (this session)
- **RotorAero removed** — `aero/aero_rotor.py` deleted; `create_aero()` factory now has 4 models (SkewedWakeBEM default).
- **Two-loop attitude controller**: `compute_rate_cmd(kp, kd=0)` → rate setpoint; `RatePID(kp=2/3)` → swashplate tilt. Matches hardware architecture where ArduPilot rate PIDs supply damping.
- **Portable core** in `controller.py`: `compute_bz_tether`, `slerp_body_z`, `compute_rate_cmd`, `col_min_for_altitude_rad` — frame-agnostic, map 1:1 to Lua/C++ Mode_RAWES.
- **High-tilt De Schutter**: ξ=80° viable. AoA stays below stall (14.4°) because low v_axial at high tilt reduces inflow angle. Requires `col_max=0.10 rad`, `col_min_reel_in=0.079 rad`. BEM invalid above ξ≈85°.
- **body_z_slew_rate** = `rotor.body_z_slew_rate_rad_s` = 2% of gyroscopic limit = **0.40 rad/s** for beaupoil_2026. Optimal from sweep; faster than 0.40 causes oscillation, slower wastes reel-in time.
- **`swashplate_pitch_gain_rad`** added to YAML/RotorDefinition — physically measurable via flap deflection angle × tau at full stick deflection.
- **Visualizer** (`viz3d/visualize_3d.py`): create-once actor pattern (`user_matrix` for rotor/hub/arrows, `.points` in-place for tether/trail), wall-clock while loop, linear interpolation between telemetry frames, net energy HUD.
- **`rawes_flight.lua` SITL-validated** — orbit-tracking cyclic controller confirmed working in SITL via `test_lua_flight_rc_overrides`. See [Lua API Constraints](#lua-api-constraints-this-ardupilot-build) below.

### Counter-Torque Motor Simulation
| File | When to read |
|------|-------------|
| [simulation/torque/README.md](simulation/torque/README.md) | **Complete reference** — physics model, motor specs, gear efficiency, ArduPilot integration, all 6 tests, Lua feedforward controller, hardware deployment |

### Lua Flight Controller
| File | When to read |
|------|-------------|
| [simulation/scripts/rawes_flight.lua](simulation/scripts/rawes_flight.lua) | Orbit-tracking cyclic controller — captures equilibrium, tracks tether, rate-limited slerp, cyclic P loop |

---

## Lua API Constraints (this ArduPilot build)

These constraints apply to `rawes_flight.lua` (and any future Lua scripts) running on the ArduPilot SITL Docker image and on the Pixhawk 6C with the same firmware.

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
2. **Closed-loop controller validation** — does `compute_swashplate_from_state` stabilize the physics model without ArduPilot? (`test_closed_loop.py`)
3. **ArduPilot integration** — can ArduPilot's ACRO/GUIDED modes fly the hub once physics + control are known good? (`test_guided_flight.py`)

Phases 1 and 2 run on Windows with no Docker. Phase 3 requires Docker + ArduPilot SITL.

### Module Map

```
simulation/
├── Core physics
│   ├── dynamics.py          RK4 6-DOF rigid-body integrator
│   ├── aero.py              Facade: re-exports all aero models, create_aero() factory → SkewedWakeBEM
│   ├── aero/                Per-blade BEM models (SkewedWakeBEM production; RotorAero archived)
│   ├── tether.py            Tension-only elastic tether (Dyneema SK75)
│   └── swashplate.py        H3-120 inverse mixing, cyclic blade pitch
│
├── Coordinate frames
│   └── frames.py            build_orb_frame(), T_ENU_NED (legacy utility) — single source
│
├── Sensor & interface
│   ├── sensor.py            build_sitl_packet(), SensorSim — all NED, no frame conversion needed
│   └── sitl_interface.py    ArduPilot SITL UDP binary protocol
│
├── Control
│   └── controller.py        compute_swashplate_from_state()  — truth-state controller
│                            compute_rc_rates()               — ArduPilot RC override controller
│                            compute_rc_from_attitude()       — ATTITUDE message controller
│                            RatePID                          — simulated ACRO inner rate loop
│                            compute_bz_tether/slerp_body_z/compute_rate_cmd  — portable core (Lua/C++ portable)
│                            col_min_for_altitude_rad()       — aero-derived altitude floor collective
│
├── Orchestration
│   ├── mediator.py          400 Hz co-simulation loop (SITL ↔ physics)
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
    │   ├── test_closed_loop.py               ★ Closed-loop physics (dynamics+aero+tether+controller)
    │   ├── test_closed_loop_60s.py           60 s orbit — uses RatePID two-loop architecture
    │   ├── test_steady_flight.py             Open-loop equilibrium → writes steady_state_starting.json
    │   ├── test_controller.py                Unit tests (incl. RatePID, portable core functions)
    │   ├── test_aero_trajectory_points.py    SkewedWakeBEM at De Schutter trajectory operating points
    │   ├── test_deschutter_cycle.py          De Schutter pumping cycle (ξ=80°, col_max=0.10 rad)
    │   ├── test_lua_flight_logic.py          ★ Rodrigues rotation, orbit tracking, slerp, cyclic error (rawes_flight.lua math)
    │   └── ...
    └── stack/               Docker required
        ├── stack_utils.py           Shared constants + helpers (env vars, logging, process launch/teardown, port kill, log copy)
        ├── conftest.py              acro_armed + acro_armed_lua fixtures (full stack lifecycle)
        ├── test_guided_flight.py    60 s ACRO hold with tether-alignment RC controller
        ├── test_pumping_cycle.py    Pumping cycle stack test
        ├── test_lua_flight.py       ★ rawes_flight.lua orbit-tracking validation (Lua captures + RC overrides)
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

**CRITICAL: Unit tests and simtests run on Windows natively (no Docker). Stack/torque tests require Docker via WSL. Never mix these.**

**CRITICAL: Use the Bash tool directly for unit/simtests — do NOT use `wsl.exe`. The Bash tool runs Git Bash on Windows. `/mnt/e/...` WSL paths do NOT exist in Git Bash.**

**CRITICAL: The Bash tool's working directory is NOT the repo root. Always use absolute paths for the venv python and for file arguments. Relative paths like `simulation/tests/unit/.venv/Scripts/python.exe` will fail with "No such file or directory". Use `e:/repos/windpower/simulation/tests/unit/.venv/Scripts/python.exe` instead.**

One-time venv setup (Windows):
```cmd
py -3 -m venv simulation\tests\unit\.venv
simulation\tests\unit\.venv\Scripts\python.exe -m pip install numpy pytest matplotlib
```

---

### Stage 1 — Unit tests (fast, Windows, no Docker)

Pure Python: physics, aero, tether, controller, sensor, planner. No ArduPilot, no network, no Docker.

```
e:/repos/windpower/simulation/tests/unit/.venv/Scripts/python.exe -m pytest e:/repos/windpower/simulation/tests/unit -m "not simtest" -q
```

Expected: ~398 tests, ~65 s. Fix all failures here before proceeding.

---

### Stage 2 — Simtests (slow, Windows, no Docker)

Full closed-loop physics (dynamics + aero + tether + attitude controller) for 10–60 s. No ArduPilot.

```
e:/repos/windpower/simulation/tests/unit/.venv/Scripts/python.exe -m pytest e:/repos/windpower/simulation/tests/unit -m simtest -q
```

Expected: ~23 tests, ~5 min. Fix all failures here before proceeding.

To run both stages together:
```
e:/repos/windpower/simulation/tests/unit/.venv/Scripts/python.exe -m pytest e:/repos/windpower/simulation/tests/unit -q
```

---

### Stage 3 — Stack tests (Docker, ArduPilot SITL)

Full SITL co-simulation: mediator + ArduPilot + MAVLink GCS. Runs sequentially — never launch two stack/torque tests at the same time.

```
wsl.exe bash -c 'bash /mnt/e/repos/windpower/simulation/dev.sh test-stack -v'
wsl.exe bash -c 'bash /mnt/e/repos/windpower/simulation/dev.sh test-torque -v'
```

Filtered runs:
```
wsl.exe bash -c 'bash /mnt/e/repos/windpower/simulation/dev.sh test-stack -v -k test_pumping_cycle'
wsl.exe bash -c 'bash /mnt/e/repos/windpower/simulation/dev.sh test-torque -v -k test_lua_yaw_trim'
```

Status-only (pass/fail summary, no logs):
```
wsl.exe bash -c 'bash /mnt/e/repos/windpower/simulation/dev.sh test-stack --filterstatus'
```

**Always run `analyse_run.py` after a stack test:**
```
wsl.exe bash -c 'python3 /mnt/e/repos/windpower/simulation/analysis/analyse_run.py'
wsl.exe bash -c 'python3 /mnt/e/repos/windpower/simulation/analysis/analyse_run.py --plot'
```

---

### Other commands

| Task | Command |
|------|---------|
| Regenerate steady state | `simulation/tests/unit/.venv/Scripts/python.exe -m pytest simulation/tests/unit -k test_steady_flight` |
| Torque visualizer | `simulation/tests/unit/.venv/Scripts/python.exe simulation/torque/visualize_torque.py` |
| Build Docker image | `simulation\build.cmd ardupilot` |
| Start/stop container | `wsl.exe bash -c 'bash /mnt/e/repos/windpower/simulation/dev.sh start/stop'` |
| Add Python package to stack | Edit `simulation/Dockerfile` pip install line → rebuild image (see below) |
| Add Python package to unit tests | `simulation/tests/unit/.venv/Scripts/python.exe -m pip install <pkg>` |

**⚠️ Rebuilding the Docker image — CRITICAL rules:**

1. **Always pass `--build-arg INSTALL_ARDUPILOT=true`** (or use `build.cmd ardupilot`). Running `docker build` without this arg replaces the ArduPilot image with a base-only 1.9 GB image — ArduPilot binary vanishes and all stack tests fail with `RAWES_SIM_VEHICLE` errors.

2. **The full build takes ~30–60 min** (ArduPilot clone + waf compile). Use `run_in_background=true` on the Bash tool call **without** a trailing `&` inside the command. Adding `&` inside causes the shell to exit early and SIGHUP kills the Docker build mid-compile, producing a corrupt image with exit code 0.

3. After rebuilding: `dev.sh stop && dev.sh start` to swap the container to the new image.

WSL equivalent of `build.cmd ardupilot`:
```
wsl.exe bash -c 'docker build /mnt/e/repos/windpower/simulation -t rawes-sim --build-arg INSTALL_ARDUPILOT=true'
```
| Python analysis script | `wsl.exe bash -c 'python3 /mnt/e/repos/windpower/simulation/...'` |
| One-off inside container | `wsl.exe bash -c 'docker exec rawes-dev python3 /rawes/simulation/...'` |

Last run logs: `simulation/logs/` — `pytest_last_run.log`, `mediator_last_run.log`, `sitl_last_run.log`, `gcs_last_run.log`, `telemetry.csv`

Windows repo root on WSL path: `/mnt/e/repos/windpower`

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

See [simulation/sim_internals.md](simulation/sim_internals.md) for full sensor/controller design details.

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
- Pumping cycle state machine in `mediator.py` (REEL_OUT/REEL_IN phases, winch step, TensionController)
- **Gate:** All unit tests pass, test_acro_hold passes ✓

### M2 — Force Balance Audit & Rotor Abstraction ✅
- `rotor_definition.py` — full RotorDefinition API + YAML files (`beaupoil_2026.yaml`, `de_schutter_2018.yaml`)
- Mediator/aero/dynamics fully wired to rotor definition (no hardcoded rotor constants)
- **Gate:** 265 unit tests pass ✓

### M3 — ArduPilot Configuration & Pumping Cycle Stack Test (in progress)
- [x] Run test_pumping_cycle — PASSED with SkewedWakeBEM (reel-out 199 N, reel-in 86 N, net energy +1396 J)
- [x] Switch production aero to SkewedWakeBEM (per-blade BEM + Prandtl + Coleman); 534 unit + 23 simtests passing
- [x] Design `ModeRAWES` firmware architecture — documented in `simulation/raws_mode.md`
- [x] Write and validate `rawes_flight.lua` orbit-tracking controller in SITL — `test_lua_flight_rc_overrides` PASSES; equilibrium captured at t≈0.5 s; 31 unit tests for Lua math (Rodrigues, orbit tracking, slerp, cyclic projection)
- [x] Confirm H_SWASH_TYPE=3 (H3-120) — `test_h_swash_phang` PASSES; default is already H3_120 (value 3, not 1)
- [x] Determine H_PHANG — `test_h_swash_phang` empirical step-cyclic test: H_SW_PHANG=0 confirmed; cross_ch1=1.5%, cross_ch2=19.7%; ArduPilot H3_120 +90° roll advance angle already aligns with RAWES servo layout (S1=0°/East, S2=120°, S3=240°)
- [ ] Configure GB4008: H_TAIL_TYPE=4 (DDFP), tune ATC_RAT_YAW_* and H_COL2YAW feedforward
- [ ] Write `rawes_params.parm` (full parameter file for Pixhawk 6C)
- **Gate:** rawes_params.parm exists + H_PHANG determined

### M4 — Hardware-in-the-Loop (Pixhawk 6C)
- [ ] Write `hil_interface.py` — MAVLink HIL_SENSOR / HIL_GPS / HIL_ACTUATOR_CONTROLS
- [ ] Add --hil-mode --hil-port to mediator
- [ ] Confirm IMU mounting orientation (AHRS_ORIENTATION)
- [ ] Write `test_hil_interface.py`
- [ ] Document HIL bench procedure in ARMING.md
- **Gate:** test_hil_interface.py passes + successful 60 s HIL telemetry log
