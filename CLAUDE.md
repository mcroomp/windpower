# RAWES — Project Context for Claude

## Project Goal

Build an **ArduPilot flight controller** for a Rotary Airborne Wind Energy System (RAWES) — a tethered, 4-blade autogyro kite. Wind drives autorotation; cyclic pitch control steers; tether tension during reel-out drives a ground generator. No motor drives rotation.

**Current phase:** Phase 3, Milestone 3. `rawes.lua` uses `AltitudeHoldController` (elevation rate-limited toward `RAWES_ALT`, gravity-compensated disk tilt). Pumping uses `TensionPI` inside rawes.lua (mirrors Python `controller.TensionPI`). Pre-GPS: gyro feedthrough only. GPS fusion uses dual GPS (`EK3_SRC1_YAW=2`, RELPOSNED heading). **Next: validate `test_pumping_cycle` (stack), then `test_landing_stack` (stack).**

**Status:**
- Simtests (13): 11 PASS. `test_landing` / `test_landing_lua` failing (winch control during descent). `test_kinematic_transition` pre-existing failure.
- Stack tests (parallel -n 8): 9 PASS including `test_lua_flight_steady` (stable 86–110 s, 3/3). `test_pumping_cycle` + `test_landing_stack` in dev.

See [design/history.md](design/history.md) for milestone history and the milestone table at the end of this file for current gates.

---

## Reference Documents

All design docs live under `design/`. CLAUDE.md is intentionally lean — most architecture lives in these.

**System & flight stack**
| File | Purpose |
|------|---------|
| [design/physical.md](design/physical.md) | Physical system parameters — rotor, tether, motor, servos, winch motor/generator spec |
| [design/flight_stack.md](design/flight_stack.md) | **Complete flight control reference** — architecture, GCS, rawes.lua modes 0/1/4/5, EKF3, arming, channel ownership, ArduPilot configuration |
| [design/ardupilot_pids.md](design/ardupilot_pids.md) | ArduPilot heli rate-PID stack — `AC_AttitudeControl_Heli`, Python re-implementation |
| [design/ardupilot_swashplate.md](design/ardupilot_swashplate.md) | Swashplate signal flow: RC3 collective → servo PWM (H3-120) |
| [design/ekf_const_pos_mode.md](design/ekf_const_pos_mode.md) | EKF3 `const_pos_mode` — causes and log diagnostics |

**Simulation**
| File | Purpose |
|------|---------|
| [design/simulation.md](design/simulation.md) | **Simulation internals** — sensor design, controller stack, dynamics, aero, tether, kinematic startup, pumping/landing architecture, **module map**, SITL lockstep |
| [design/aero.md](design/aero.md) | De Schutter Eq. 25–31 validation vs. implementation |
| [design/history.md](design/history.md) | Phase 2 + Phase 3 decisions, root causes, results |
| [design/testing.md](design/testing.md) | **Unit & simtest guide** — test catalogue, Lua harness API, SCR_USER6 encoding, IC loading, telemetry |

**Hardware**
| File | Purpose |
|------|---------|
| [design/hardware.md](design/hardware.md) | Assembly layout, rotor geometry, swashplate, Kaman flap mechanism |
| [design/components.md](design/components.md) | Component specs: GB4008, REVVitRC ESC, DS113MG servos |
| [design/calibration.md](design/calibration.md) | calibrate.py CLI reference — servo/motor/ESC/Lua upload over MAVLink |
| [design/dshot.md](design/dshot.md) | DShot reference — bench and flight-mode parameter tables, AM32 EDT, wiring, troubleshooting |
| [design/flap_sensor_bench.md](design/flap_sensor_bench.md) | Bench measurement system for swashplate→push-rod→flap deflection |

**Theory**
| File | Purpose |
|------|---------|
| [design/theory_pumping.md](design/theory_pumping.md) | De Schutter 2018 — pumping cycle, aero, structural constraints |
| [design/theory_flap.md](design/theory_flap.md) | Weyel 2025 — flap state-space, feed-forward + PID, N4SID ID |

---

## Critical Invariants

These are the rules that, if violated, silently destroy correctness. Read every one.

### Coordinates & signs

- **NED everywhere.** X=North, Y=East, Z=Down. Gravity = `[0, 0, +9.81·m]`. Altitude = `-pos[2]`. `T_ENU_NED` in `frames.py` is for legacy external data only — never used in the simulation loop. Any `pos_enu`, `altitude = pos[2]` (without negation), or ENU-style arithmetic is a bug.
- **Thrust sign:** for body_z = disk_normal = `[0,0,-1]` (horizontal disk in NED), upward thrust = `dot(F_world, body_z) = -F_world[2]`. **Never negate `body_z` when projecting thrust** — `dot(F_world, -body_z) = +F_world[2]` gives the *downward* component. Rule: **upward thrust = `dot(F_world, disk_normal)`** regardless of disk orientation. Verified by `test_hover_sign.py`.

### Sensors & EKF (physically faithful, no overrides)

`sensor.py` must report exactly what the real Pixhawk hardware would see. The electronics hub is the fuselage; `R_hub` is its full 3-DOF orientation, integrated by the dynamics ODE.

- `rpy` = ZYX Euler angles from `R_hub` directly. **Never override `rpy[2]` with velocity heading.**
- `gyro_body = R_hub.T @ omega_body` — full body angular velocity. No stripping. The anti-rotation motor keeps electronics non-rotating via `K_YAW` damping in dynamics.
- `accel_body = R_hub.T @ (accel_world − gravity)` — in electronics body frame.
- Yaw is a real physical DOF, not a convention. `EK3_SRC1_YAW=2` (dual-antenna GPS yaw, RELPOSNED) + `COMPASS_USE=0` — compass disabled (GB4008 motor interference + cycling SITL compasses).
- **GPS/EKF glitches mean physics inputs are wrong — fix the physics, not the EKF.** Do NOT disable EKF fusion sources or reduce thresholds. Emergency yaw resets indicate a compass/attitude mismatch in the sensor model.

### Kinematic-phase sensor consistency

The kinematic trajectory is purely artificial — only used to bring the EKF and GPS to a fused, healthy state before real physics begins. **All sensors sent to SITL during kinematic must be physically consistent with the prescribed trajectory as if a magical external force holds the hub in place.**

- `accel_body = R.T @ (d_vel/dt − gravity)` — for a stationary hold, d_vel/dt=0 so `accel_body = R.T @ [0,0,−g]`.
- `gyro_body = R.T @ omega_body` — full body angular velocity, no stripping.
- `vel` sent directly (zero for stationary hold).

Use `validate_sitl_sensors.py` to verify consistency after any kinematic change.

### Stack tests must validate the actual stack

- **`internal_controller` MUST be `False` for all full stack flight tests.** The entire purpose of SITL stack tests is to validate that ArduPilot + Lua actually fly the vehicle. `internal_controller=True` is only valid in unit tests and simtests where Lua/ArduPilot are not involved.
- **SITL must run as close to hardware as possible.** Find and fix root causes. Do NOT paper over failures with simulation-only hacks. Confirm with user before adding any override.
- **Stack tests must not violate physics.** Never add artificial mechanisms just to stabilise a test. Acceptable: `lock_orientation=False`, `base_k_ang=50`.

### Other

- **Use ACRO. Never STABILIZE.** STABILIZE holds NED roll=0, fighting the 65° tether equilibrium; yaw discontinuity at unfreeze → EKF emergency reset → crash within 4 s. ACRO + `compute_rc_from_attitude` is the only working mode. ArduCopter ACRO = mode **1** (mode 6 is RTL). `ATC_RAT_RLL_IMAX = ATC_RAT_PIT_IMAX = ATC_RAT_YAW_IMAX = 0` to prevent orbital-rate I-windup. See [design/flight_stack.md §6](design/flight_stack.md).
- **No silent defaults for physics parameters.** Raise `KeyError`/`ValueError` if a required config key is absent. Never use `dict.get("key", fallback)` or `x = x or default` for physical constants, control gains, or rotor/airfoil properties.
- **NEVER use non-ASCII characters in Python `print()`.** Windows cp1252 → `UnicodeEncodeError`. Use 7-bit ASCII only: `-`, `[PASS]`/`[FAIL]`, `sd=` for sigma.

---

## Workflow Rules

- **Do NOT consult git history** (`git log`/`diff`/`show`/`blame`) when diagnosing problems unless you first ask the user.
- **Fix telemetry/logging before diagnosing test failures.** When a simtest or stack test fails, inspect the telemetry CSV and logs first. If columns are zero/missing/wrong (e.g. `tether_m=0`, phase never changes), fix the logging bug before attempting to diagnose physics. Diagnosing from bad telemetry produces wrong conclusions.
- **Run `analyse_run.py` first after any stack test failure.** It loads all log sources (telemetry CSV, mavlink.jsonl, mediator.log, arducopter.log) into a unified `FlightLog` and prints a single bucketed report. `--bucket 10` for overview, `--bucket 1` for frame-level detail.
- **Keep `rawes_test_surface.lua` in sync with `rawes.lua`.** Lua unit tests access constants and functions through `_rawes_fns`, which is spliced inside `rawes.lua`'s anonymous function wrapper and so can see module-level locals only. Whenever you add a local constant or function to `rawes.lua` that tests need, add it to `_rawes_fns` in `rawes_test_surface.lua` in the same commit. Function-local variables are not accessible — hoist them to module level first.
- **`controller.py` follows `rawes.lua`.** `test_math_lua.py` cross-checks `rawes.lua` against `controller.py`; a failure there means `controller.py` diverged — fix `controller.py`.

---

## Lua API Gotchas

| What you'd expect | What actually works |
|---|---|
| `ahrs:get_rotation_body_to_ned()` | Doesn't exist. Use `ahrs:body_to_earth(v)` / `ahrs:earth_to_body(v)` |
| `Vector3f(x, y, z)` | Constructor ignores args. Use `Vector3f()` then `:x()/:y()/:z()` setters |
| `v:normalized()` | Doesn't exist. Copy then `:normalize()` in-place |
| `vec * scalar` or `vec + vec` | `*` not overloaded; `+` may silently fail. Use component arithmetic |
| `rc:set_override(chan, pwm)` | Use `rc:get_channel(n):set_override(pwm)` (cache channel at module load) |
| ArduCopter ACRO = 6 | ACRO = **1**. Mode 6 is RTL |

---

## Running Tests

**Unit tests and simtests: Windows native, no Docker. Stack tests: Docker required. Never mix.**

- **`.venv`** — Windows venv for unit tests and simtests. `run_tests.py` auto-installs when `requirements.txt` changes (SHA-256 hash stamp).
- **Docker container** — has its own Python env (never use the Windows venv inside Docker). Managed by `dev.sh build`.

### Rules

- **Use Bash tool directly — never `wsl.exe`. Always absolute paths.**
- **Always pass an explicit test path to `run_tests.py`** (e.g. `simulation/tests/unit` or `simulation/tests/simtests`). Running without a path lets pytest wander into `simulation/analysis/` and other non-test scripts using `argparse`, causing collection errors.
- **Scope `Grep` to `e:/repos/windpower/simulation/`** — the repo root contains `.venv/` with hundreds of thousands of third-party files.
- **NEVER call `docker exec` directly to run stack tests. Use `bash simulation/dev.sh test-stack`.** Each stack test always runs in its own fresh Docker container.
- **Unit/simtests run via the Windows venv directly — NOT via `dev.sh test-unit`** (which routes to Docker and fails because `tests/unit` is excluded from the container sync).

### Commands

| Task | Command |
|------|---------|
| Unit tests (~685) | `.venv/Scripts/python.exe -m pytest simulation/tests/unit -m "not simtest" -q` |
| Simtests (~13) | `.venv/Scripts/python.exe simulation/run_tests.py simulation/tests/simtests -m simtest -q` |
| Simtest (single) | `.venv/Scripts/python.exe simulation/run_tests.py simulation/tests/simtests -k test_foo -s` |
| Stack test (single) | `bash simulation/dev.sh test-stack -n 1 -k test_foo` |
| Stack test (full) | `bash simulation/dev.sh test-stack -n 8` |
| **Post-failure analysis** | `.venv/Scripts/python.exe simulation/analysis/analyse_run.py <test_name>` (add `--bucket 10` for coarser view) |
| **Visualize result** | `.venv/Scripts/python.exe simulation/viz3d/visualize_3d.py simulation/logs/<test_name>/telemetry.csv` |
| Scrub frames | `.venv/Scripts/python.exe simulation/viz3d/scrub.py simulation/logs/<test_name>/telemetry.csv` |
| Render to MP4/GIF | `.venv/Scripts/python.exe simulation/viz3d/render_cycle.py <csv> [--out cycle.mp4] [--speed 2]` |
| **Pumping envelope** | `.venv/Scripts/python.exe simulation/analysis/pump_envelope.py` (add `--wind 8 10 12`, `--telemetry <csv>`) |
| **Pump cycle diagnosis** | `.venv/Scripts/python.exe simulation/analysis/pump_diagnosis.py --test test_pump_cycle_unified --bucket 1` |
| **Landing diagnosis** | `.venv/Scripts/python.exe simulation/analysis/analyse_landing.py [--test test_landing_lua] [--bucket 2]` |
| **High-freq telemetry** | `RAWES_TEL_HZ=400 .venv/Scripts/python.exe simulation/run_tests.py simulation/tests/simtests -k <name> -s` (default 20 Hz) |
| **Regenerate `steady_state_starting.json`** | `.venv/Scripts/python.exe -m pytest simulation/tests/simtests/test_generate_ic.py::test_create_ic -s` — **the ONLY test that writes the file.** Run after any aero model change. |
| Container start/stop | `bash simulation/dev.sh start` / `bash simulation/dev.sh stop` |
| Docker build | `bash simulation/dev.sh build` (~30–60 min; use `run_in_background=true`, no trailing `&`) |
| Run inside container | `bash simulation/dev.sh exec 'python3 /rawes/simulation/...'` |

Stack test logs: `simulation/logs/{test_name}/` — `mediator.log`, `sitl.log`, `gcs.log`, `telemetry.csv`, `arducopter.log`. Suite summary: `simulation/logs/suite_summary.json`.

---

## Key Design Decisions (one-liners — see references for detail)

- **Production aero:** `PetersHeBEMJit` (3-state dynamic inflow, Numba). No skew-angle validity limit; momentum ODE valid hover→axial descent. Pure-numpy reference: `PetersHeBEM`. See [design/simulation.md](design/simulation.md) Aerodynamic Model + [design/aero.md](design/aero.md).
- **Two-loop attitude:** `compute_rate_cmd(kp)` → rate setpoint; `RatePID(kp=2/3)` → swashplate tilt. **Portable core** in `controller.py` maps 1:1 to Lua: `compute_bz_tether`, `slerp_body_z`, `compute_rate_cmd`, `col_min_for_altitude_rad`, `compute_bz_altitude_hold`.
- **High-tilt De Schutter:** xi=80° viable. `col_max=0.10`, `col_min_reel_in=0.079`. BEM invalid above xi≈85°. `body_z_slew_rate = 0.40 rad/s`.
- **rawes.lua modes (valid: 0, 1, 4, 5):** 0=none, 1=steady, 4=landing, 5=pumping. Modes 1/4/5 own Ch3. Substates via `NAMED_VALUE_FLOAT("RAWES_SUB", N)`. See [design/flight_stack.md §4](design/flight_stack.md).
- **Yaw regulation** lives entirely in ArduPilot's `ATC_RAT_YAW` PID (`H_TAIL_TYPE=4` DDFP CCW → SERVO4 → anti-rotation motor). **rawes.lua writes no commands to SERVO9/Ch9.** No DShot active (`RPM1_TYPE=0`); anti-rotation motor on standard PWM, MAIN OUT 4. Current hardware: GB4008 + 80:44 gear. See [design/dshot.md](design/dshot.md), [design/flight_stack.md §4.7–§5](design/flight_stack.md).
- **GPS fusion timing:** `EK3_GPS_CHECK=0` + widened gates (`EK3_POS_I_GATE=50`, `EK3_VEL_I_GATE=50`) — required boot params in `rawes_sitl_defaults.parm`. `GPS_AUTO_CONFIG=0` is critical (prevents ArduPilot from corrupting RELPOSNED in SITL). See [design/flight_stack.md Appendix D](design/flight_stack.md).
- **Anchor in `LOCAL_POSITION_NED`:** `SCR_USER5 = −initial_state["pos"][2]`. Anchor at `[0, 0, −pos0[2]]` in EKF frame.
- **Pumping (Python simtest):** ground/AP split with `TensionCommand` protocol. Ground owns altitude smoothing; AP must not add a second layer. **`winch_target_tension = tension_ic` during reel-out (NOT `tension_out`).** See [design/simulation.md](design/simulation.md) Pumping Cycle Architecture.
- **Landing:** unified — `LandingGroundController` (10 Hz) → `LandingCommand` → `LandingApController` (400 Hz) + `WinchController`. Three phases: reel_in / descent / final_drop. Old `LandingPlanner` deleted. See [design/simulation.md](design/simulation.md) Landing Architecture.
- **IC generation targets 300 N tension.** `test_generate_ic.py::test_create_ic` runs 60 s warmup with TensionPI targeting 300 N. `coll_eq_rad` is the TensionPI-settled collective, not a hardcoded constant. TensionPI warm-starts at this collective in all simtests.
- **AcroControllerSitl (25 ms servo lag) is baked into `PhysicsRunner` and always active for simtests.**
- **Gyroscopic phase NOT needed:** `H_SW_PHANG=0`. `BASE_K_ANG=50 N·m·s/rad` → τ≈0.08 s. `swashplate_phase_deg≠0` degrades orbit stability.
- **Torque model:** kinematic + first-order motor lag. ESC holds RPM proportional to PWM at steady state; `MOTOR_TAU=0.02 s`. `equilibrium_throttle ≈ 0.485` at 28 rad/s. `H_YAW_TRIM = −0.419`. See [design/flight_stack.md §5](design/flight_stack.md).

---

## SITL Lockstep — Key Rule

The physics worker must reply to **every** SITL servo packet without exception — skipping a reply causes ArduPilot to stall permanently. `gcs.sim_now()` returns `time_boot_ms/1000` from the most recently processed MAVLink message, not wall-clock time. `sim_sleep(N)` waits N sim-seconds; the physics loop must keep running during the wait. Full reference: [design/simulation.md § SITL Lockstep Protocol](design/simulation.md).

---

## Current Limits

1. Single-body hub model — no blade multibody, no flapping DOF
2. Rotor spin is a scalar ODE — gyroscopic coupling computed but `I_spin` effect is small
3. Tether: tension-only elastic — no sag, no distributed mass, no reel dynamics
4. Aero: steady-state BEM — no dynamic inflow; Coleman skewed wake handles non-uniform induction
5. Controller: 10 Hz in stack test (MAVLink RC override); 400 Hz in unit tests / internal controller

---

## Phase 3 Plan

See [design/history.md](design/history.md) for full decision history.

**Test progression: steady → pumping → landing.** Fix `test_lua_flight_steady` (stack) before debugging pumping or landing Lua stack tests. Do not debug `test_pumping_cycle` or `test_landing_stack` (stack) until `test_lua_flight_steady` passes cleanly (orbit_r < 5 m, altitude stable ±2 m, yaw gap < 15 deg for ≥ 60 s).

| Milestone | Status | Gate |
|-----------|--------|------|
| M1 Wire Pumping Cycle | done | — |
| M2 Force Balance & Rotor Abstraction | done | — |
| M3 Step 1 — test_lua_flight_steady (stack) | done: stable=86–110 s, 3/3 runs | orbit_r < 5 m, no EKF yaw reset, ≥ 60 s stable |
| M3 Step 1b — test_landing.py + test_landing_lua (simtest) | failing: winch control during descent | descent slack=0, floor hit, anchor_dist < 20 m |
| M3 Step 2 — test_pumping_cycle (stack, SCR_USER6=5) | in dev | "RAWES pump: reel_out" + net_energy > 0 + peak_tension < 496 N |
| M3 Step 3 — test_landing_stack (stack, SCR_USER6=4) | in dev | "RAWES land: captured" + "final_drop" + hub alt ≤ 2.5 m |
| M3 Step 4 — rawes_params.parm (Pixhawk 6C) | not started | file exists + H_PHANG determined |
| M4 — Hardware-in-the-Loop (Pixhawk 6C) | not started | test_hil_interface.py passes + 60 s HIL log |
