# RAWES — Project Context for Claude

## Project Goal

Build an **ArduPilot flight controller model** for a Rotary Airborne Wind Energy System (RAWES) — a tethered, 4-blade autogyro kite. Wind drives autorotation; cyclic pitch control steers; tether tension during reel-out drives a ground generator. No motor drives rotation.

**Current phase:** Phase 3, Milestone 3. 482 unit + 45 simtests + 14 stack tests passing. SkewedWakeBEMJit is production aero model. `rawes.lua` orbit-tracking validated in SITL. H_SW_PHANG=0, H_SW_TYPE=3 confirmed. **Next priority: steady orbit under full ArduPilot control (internal_controller=False) as the foundation for pumping and landing.**

**Stack test status (14 PASS):** test_arm_minimal, test_acro_armed, test_acro_hold, test_h_swash_phang, test_lua_flight_rc_overrides, test_stack_integration_smoke, test_wobble, test_gust, test_pitch_roll, test_slow_vary, test_startup, test_lua_yaw_trim, test_yaw_regulation, test_pumping_cycle (net_energy=+2098 J, peak_tension=315 N). test_pumping_cycle_lua and test_landing_lua: in development.

**Test infrastructure:** Stack tests moved from `tests/stack/` to `tests/sitl/flight/` + `tests/sitl/torque/`. Shared code extracted into `stack_infra.py` (context managers `_sitl_stack`, `_acro_stack`, `_torque_stack`). `conftest.py` is now a thin re-exporter only.

---

## Physical System

| Parameter | Value |
|-----------|-------|
| Blade count | 4 (90° apart) |
| Blade length | 2000 mm, rotor radius ~2500 mm |
| Rotor mass | 5 kg, airfoil SG6042 |
| Tether | Dyneema SK75, 1.9 mm, max 300 m |
| Anti-rotation motor | EMAX GB4008 66KV, 80:44 spur gear |
| Servos S1/S2/S3 | DS113MG V6.0 |
| Flight controller | Holybro Pixhawk 6C |
| Battery | 4S LiPo 15.2V, 450 mAh |

**Critical:** Only the spinning outer hub shell + blades contribute to `I_spin`. Motor torque is an **internal** force — never add as external couple.

---

## Reference Documents

| File | Purpose |
|------|---------|
| [hardware/design.md](hardware/design.md) | Assembly layout, rotor geometry, swashplate, Kaman flap mechanism |
| [hardware/components.md](hardware/components.md) | Component specs: GB4008, REVVitRC ESC, DS113MG servos |
| [hardware/calibrate.md](hardware/calibrate.md) | **calibrate.py CLI reference** — servo/motor/ESC/Lua upload over MAVLink |
| [theory/pumping_cycle.md](theory/pumping_cycle.md) | De Schutter 2018 — pumping cycle, aero, structural constraints |
| [theory/orbit_mechanics.md](theory/orbit_mechanics.md) | Beaupoil 2026 — orbit characteristics, gyroscopic analysis |
| [theory/flap_dynamics.md](theory/flap_dynamics.md) | Weyel 2025 — flap state-space, feed-forward + PID, N4SID ID |
| [system/stack.md](system/stack.md) | **Complete flight control reference** — architecture, GCS, rawes.lua, EKF3, arming. Update when new EKF/arming behaviour is found. |
| [simulation/internals.md](simulation/internals.md) | Sensor design, aero model, tether, COL_MIN rules, known gaps |
| [simulation/history.md](simulation/history.md) | Phase 2 + Phase 3 decisions |
| [simulation/aero/deschutter.md](simulation/aero/deschutter.md) | De Schutter Eq. 25–31 validation vs. implementation |
| [simulation/torque/README.md](simulation/torque/README.md) | Counter-torque motor physics, GB4008, Lua feedforward |
| [simulation/scripts/rawes.lua](simulation/scripts/rawes.lua) | Unified Lua controller (SCR_USER6 modes 0–5) |

---

## Key Design Decisions

- **Production aero:** `SkewedWakeBEMJit` (`aero/aero_skewed_wake_jit.py`) — Numba `@njit` drop-in. 18-test equivalence suite vs. reference (atol=1e-10). `create_aero(model="jit")`. Non-JIT version is human-readable reference only.
- **Two-loop attitude:** `compute_rate_cmd(kp, kd=0)` → rate setpoint; `RatePID(kp=2/3)` → swashplate tilt.
- **Portable core** in `controller.py`: `compute_bz_tether`, `slerp_body_z`, `compute_rate_cmd`, `col_min_for_altitude_rad` — map 1:1 to Lua.
- **High-tilt De Schutter:** xi=80° viable. `col_max=0.10 rad`, `col_min_reel_in=0.079 rad`. BEM invalid above xi≈85°.
- **body_z_slew_rate** = 0.40 rad/s (`rotor.body_z_slew_rate_rad_s`). Faster → oscillation; slower → wastes reel-in time.
- **RotorDefinition YAML fields:** `swashplate_pitch_gain_rad` (measurable via flap deflection); `CD_structural` (beaupoil_2026=0.0, de_schutter_2018=0.021).
- **Canonical telemetry** (`simulation/telemetry_csv.py`): 67 columns, `TelRow` dataclass. `damp_alpha`=0.0 in free flight; used by `analyse_run.py` to split kinematic vs. free-flight phases. `TelRow.heartbeat()` = mediator 1 Hz status line.
- **rawes.lua SCR_USER6 modes:** 0=none, 1=flight cyclic (50 Hz), 2=yaw trim (100 Hz), 3=both, 4=landing (cyclic + VZ descent + auto final_drop, 50 Hz), 5=pumping (De Schutter cyclic + per-phase collective, 50 Hz). Modes 4+5 own Ch3 entirely.
- **GPS fusion timing:** `EK3_GPS_CHECK=0` + widened gates (`EK3_POS_I_GATE=50`, `EK3_VEL_I_GATE=50`) are required boot params (in `rawes_sitl_defaults.parm`). GPS origin sets during arm; "EKF3 IMU0 is using GPS" appears once EKF innovations settle. Without widened gates, GPS fusion fails. Active diagnosis: "EKF3 IMU0 switching to compass X" cycling (every ~10 s) observed in SITL — compass instability may be blocking GPS fusion.
- **EK3_SRC1_YAW=1** (compass) for SITL — `rawes_sitl_defaults.parm` boot default. SITL compass is synthetic and tracks `rpy[2]` exactly, so compass and attitude always agree → no EKF emergency yaw reset. On hardware, consider switching to `=8` (GPS velocity yaw) for orbital flight where attitude yaw is ambiguous.
- **rawes.lua landing (SCR_USER6=4):** captures body_z on `ahrs:healthy()`; VZ descent controller (`col_cmd = COL_CRUISE + KP_VZ*(vz_actual - VZ_LAND_SP)`, VZ_LAND_SP=0.5 m/s); `final_drop` (collective→0) when alt ≤ 2.0 m. Fixture: `acro_armed_landing_lua` (`internal_controller=False`, `kinematic_vel_ramp_s=20` so hub exits kinematic at vel=0 — eliminates linear tether jolt without internal controller).
- **rawes.lua pumping (SCR_USER6=5):** phase detection uses **cumulative** tether length change vs. running reference (`PUMP_LEN_THRESH=0.05 m`). Hold tracks min tlen; reel-out tracks peak; reel-in tracks trough. Per-iteration delta is too small (≈0.0024 m per 20 ms).
- **Anchor in `LOCAL_POSITION_NED`:** `SCR_USER5 = -initial_state["pos"][2]` (NED Z negated). Anchor at `[0, 0, -pos0[2]]` in EKF frame.
- **SCR_ENABLE bootstrap:** After EEPROM wipe, scripting only starts if `SCR_ENABLE=1` is already in EEPROM. `acro_armed_lua` fixture sets it via MAVLink post-arm (persists for future boots).
- **Two orbit-tracking algorithms:** `orbit_tracked_body_z_eq` (azimuthal, inner-loop tests); `orbit_tracked_body_z_eq_3d` (3D Rodrigues, Lua-equivalent). `OrbitTracker` = stateful 3D + slerp, mirrors Lua state machine; `update(pos, dt, bz_target=None)`.
- **WinchNode** (`winch_node.py`): mediator calls `update_sensors(tension, wind_world)` (physics only); planner calls `get_telemetry()` + `receive_command()`. Wind seed from `Anemometer.measure()` at 3 m height.
- **Vertical landing:** direct drop above anchor (not spiral). Descent rate controller replaces TensionPI. Floor hit during DESCENT phase is expected; crash-guard `floor_hits>200` restricted to pumping phase only. Validated: +1857 J, 92 s (`test_pump_and_land.py`).
- **Gyroscopic phase NOT needed:** H_SW_PHANG=0. BASE_K_ANG=50 N·m·s/rad → τ≈0.08 s (damped before one orbit). `swashplate_phase_deg≠0` degrades orbit stability.
- **`AcroController use_servo=True`** required for De Schutter simtests (25 ms servo lag). Inner-loop tests (`test_steady_flight`, `test_closed_loop_60s`) do not need it.
- **Power-law wind tension:** tension_out=250 N (not 200 N) at hub altitude ~15 m (10.6 m/s wind). `col_min_reel_in` must use actual wind speed at hub altitude.
- **De Schutter aero:** `C_{D,T}=0.021` structural drag; induction bootstrap uses `abs(T)` not `max(T, 0.01)`. See `simulation/aero/deschutter.md`.

---

## Lua API Constraints

| What you'd expect | What actually works |
|---|---|
| `ahrs:get_rotation_body_to_ned()` | Doesn't exist. Use `ahrs:body_to_earth(v)` / `ahrs:earth_to_body(v)` |
| `Vector3f(x, y, z)` | Constructor ignores args. Use `Vector3f()` then `:x()/:y()/:z()` setters |
| `v:normalized()` | Doesn't exist. Copy then `:normalize()` in-place |
| `vec * scalar` or `vec + vec` | `*` not overloaded; `+` may silently fail. Use component arithmetic |
| `rc:set_override(chan, pwm)` | Use `rc:get_channel(n):set_override(pwm)` (cache channel at module load) |
| ArduCopter ACRO = 6 | ACRO = **1**. Mode 6 is RTL |

---

## Module Map

```
simulation/
├── dynamics.py          RK4 6-DOF rigid-body integrator
├── aero.py              Facade → create_aero() factory (default: SkewedWakeBEMJit)
├── aero/                SkewedWakeBEMJit (production), SkewedWakeBEM (reference), DeSchutterAero
├── tether.py            Tension-only elastic tether (Dyneema SK75)
├── swashplate.py        H3-120 inverse mixing, cyclic blade pitch
├── frames.py            build_orb_frame(), T_ENU_NED (legacy external-data utility only)
├── sensor.py            PhysicalSensor — honest R_orb sensors, NED throughout
├── sitl_interface.py    ArduPilot SITL UDP binary protocol
├── controller.py        compute_swashplate_from_state, compute_rc_from_attitude, RatePID,
│                        portable core (compute_bz_tether/slerp_body_z/compute_rate_cmd/
│                        col_min_for_altitude_rad), orbit_tracked_body_z_eq,
│                        orbit_tracked_body_z_eq_3d, OrbitTracker
├── mediator.py          400 Hz co-simulation loop (SITL <-> physics)
├── kinematic.py         KinematicStartup — hub trajectory during EKF init phase
├── winch_node.py        WinchNode + Anemometer (physics/planner protocol boundary)
├── gcs.py               MAVLink GCS client (arm, mode, RC override, params)
├── telemetry_csv.py     Canonical 67-column CSV schema (TelRow, COLUMNS, heartbeat)
└── tests/
    ├── unit/            Windows native, no Docker (~460 tests)
    └── sitl/            Docker; all SITL/stack tests live here
        ├── conftest.py          thin re-exporter — pytest_addoption + pytest_configure only
        ├── stack_infra.py       ALL shared infrastructure: StackConfig, SitlContext,
        │                        _sitl_stack, _acro_stack, _torque_stack, _run_acro_setup,
        │                        _install_lua_scripts, diagnostics helpers
        ├── stack_utils.py       low-level helpers: port checks, log copy, mediator launcher
        ├── rawes_sitl_defaults.parm  boot-time ArduPilot params (EEPROM defaults)
        ├── flight/              flight stack tests (mediator + physics + ArduPilot)
        │   ├── conftest.py      flight fixtures: acro_armed, acro_armed_*_lua, etc.
        │   ├── test_arm_minimal.py          no mediator; static sensor; arm only
        │   ├── test_gps_fusion_layers.py    GPS fusion diagnostic (6 layers + armed variant)
        │   ├── test_acro_armed.py           ...
        │   └── ...
        └── torque/              torque/anti-rotation tests
            ├── conftest.py      torque fixtures: torque_armed, torque_armed_profile, etc.
            └── ...
```

**Data flow (400 Hz):** SITL servo PWM → swashplate mix → aero → tether → RK4 dynamics → sensor packet → SITL

---

## Coordinate Conventions

**NED everywhere.** X=North, Y=East, Z=Down. Gravity = `[0, 0, +9.81*m]`. Altitude = `-pos[2]`.

> ⚠️ **NED-only policy.** `T_ENU_NED` in `frames.py` is for legacy external data conversion only — never used in the simulation loop. Any `pos_enu`, `altitude = pos[2]` (without negation), or ENU-style arithmetic is a bug. `build_orb_frame(body_z)` from `frames.py` is the single source for the orbital frame.

---

## Workflow Rules

**No silent defaults for physics parameters.** Raise `KeyError`/`ValueError` if a required config key is absent. Never use `dict.get("key", fallback)` or `x = x or default` for physical constants, control gains, or rotor/airfoil properties.

**Do NOT consult git history** (`git log`, `git diff`, `git show`, `git blame`) when diagnosing problems unless you first ask the user.

**Stack tests must not violate physics.** Never add artificial mechanisms just to stabilise a test. Acceptable: `lock_orientation=False`, `internal_controller=False`, `base_k_ang=50`.

**CRITICAL — `internal_controller` MUST be `False` for all full stack flight tests.** The entire purpose of SITL stack tests is to validate that ArduPilot + Lua actually fly the vehicle. Using `internal_controller=True` means the Python controller drives physics and ArduPilot/Lua outputs are ignored — the SITL test becomes meaningless. There are NO exceptions to this rule for stack tests. `internal_controller=True` is only valid in unit tests and simtests where Lua/ArduPilot are not involved.

**CRITICAL — SITL must run as close to hardware as possible.** Find and fix root causes. Do NOT paper over failures with simulation-only hacks. Confirm with user before adding any override.

**CRITICAL — GPS/EKF glitches mean physics inputs are wrong; fix the physics.** Do NOT disable EKF fusion sources or reduce thresholds. Telemetry column `orb_yaw_rad` (= `rpy[2]` with honest sensors) and `v_horiz_ms` are logged for diagnostics. With honest sensors, EKF emergency yaw resets indicate a compass/attitude mismatch in the physical model — fix the sensor output, not the EKF config.

**⚠️ NEVER use non-ASCII characters in Python `print()` output.** Windows cp1252 encoding causes `UnicodeEncodeError`. Use only 7-bit ASCII: `-` for lines, `[PASS]`/`[FAIL]` for status, `sd=` for sigma, etc.

---

## Running Tests

**Unit tests and simtests: Windows native, no Docker. Stack tests: Docker required. Never mix.**

**CRITICAL: Use the Bash tool directly — do NOT use `wsl.exe`. Always use absolute paths.**

**CRITICAL: NEVER call `docker exec` directly to run stack tests.** Use `bash simulation/dev.sh test-stack [...]`. Direct `docker exec pytest` bypasses port cleanup → "Address already in use" on next test. If stuck: `bash simulation/dev.sh stop && bash simulation/dev.sh start`.

**CRITICAL: Unit tests and simtests run on Windows using the venv directly — NOT via `dev.sh test-unit` (which routes to Docker and fails because `tests/unit` is excluded from the container sync). Always use the Windows venv paths below.**

```bash
bash simulation/dev.sh setup                              # one-time venv setup

# Stage 1 — Unit tests (~460, ~65 s, Windows native — use venv directly)
simulation/.venv/Scripts/python.exe -m pytest simulation/tests/unit -m "not simtest" -q

# Stage 2 — Simtests (~29, ~5 min, Windows native — use venv directly)
simulation/.venv/Scripts/python.exe -m pytest simulation/tests/unit -m simtest -q

# Stage 3 — Stack tests (Docker; test files live in tests/sitl/flight/ and tests/sitl/torque/)
# Preferred: parallel fresh mode — one isolated container per test, fully isolated EEPROM.
# -n 8 is safe (each test binds its own ports inside its own container; no port conflicts).
bash simulation/dev.sh test-stack-parallel --fresh -n 8
# Fewer workers if the host is resource-constrained:
bash simulation/dev.sh test-stack-parallel --fresh -n 4
# Single test (non-parallel is fine when -k isolates one test)
bash simulation/dev.sh test-stack -v -k test_pumping_cycle
# GPS fusion diagnostic tests:
bash simulation/dev.sh test-stack -v -k test_gps_fusion_armed
bash simulation/dev.sh test-stack -v -k "test_gps_fusion_layers[L0"
# All tests sequentially (slow, use only when parallel is not available)
bash simulation/dev.sh test-stack -v

# Post-run analysis (always do this after a stack test)
bash simulation/dev.sh exec 'python3 /rawes/simulation/analysis/analyse_run.py test_acro_armed'
bash simulation/dev.sh exec 'python3 /rawes/simulation/analysis/analyse_run.py test_pumping_cycle --plot'
```

**Per-test logs:** `simulation/logs/{log_prefix}_{test_name}/` — `mediator.log`, `sitl.log`, `gcs.log`, `telemetry.csv`, `arducopter.log`. The directory name is `"{log_prefix}_{test_name}"` when both are set, or just `"{test_name}"` when no prefix. Always read these; never `/tmp/ArduCopter.log` (stale, accumulates across tests).

Example log paths:
- `simulation/logs/test_acro_armed/` — acro_armed flight test (no prefix)
- `simulation/logs/gps_fusion_test_gps_fusion_armed/` — GPS diagnostic (prefix=`gps_fusion`)
- `simulation/logs/gps_layers_test_gps_fusion_layers[L0_baseline]/` — parametrized layer test

**Suite summary:** `simulation/logs/suite_summary.json` — pass/fail counts + failed list.

| Task | Command |
|------|---------|
| Regenerate steady state | `simulation/.venv/Scripts/python.exe -m pytest simulation/tests/unit -k test_steady_flight` |
| 3D visualizer | `cd simulation && .venv/Scripts/python.exe viz3d/visualize_3d.py logs/telemetry_pump_and_land.json` |
| Container | `bash simulation/dev.sh start` / `bash simulation/dev.sh stop` |
| Docker build | `bash simulation/dev.sh build` (~30–60 min; use `run_in_background=true`, no trailing `&`) |
| Run inside container | `bash simulation/dev.sh exec 'python3 /rawes/simulation/...'` |

**Docker build:** always uses `--build-arg INSTALL_ARDUPILOT=true` (omitting it produces a base-only image — ArduPilot binary vanishes). After rebuild: `bash simulation/dev.sh stop && bash simulation/dev.sh start`.

---

## ArduPilot ACRO Mode

**Use ACRO. Never STABILIZE** — STABILIZE holds NED roll=0, fighting the 65° tether equilibrium; yaw discontinuity at unfreeze → EKF emergency reset → crash within 4 s.

**Neutral sticks in ACRO crash:** zero rate command kills orbital precession → non-zero cyclic. Fix: `compute_rc_from_attitude` sends rates matching tether-relative attitude error.

**ACRO I-term windup:** 0.2–0.3 rad/s orbital angular velocity accumulates as rate error. Fix: `ATC_RAT_RLL_IMAX=ATC_RAT_PIT_IMAX=ATC_RAT_YAW_IMAX=0`.

**CRITICAL — Physically faithful sensors: no transformations, no overrides.** `sensor.py` must report exactly what the real Pixhawk hardware would see:
- `rpy` = actual Euler angles from `build_orb_frame(disk_normal)` — the real hub orientation. **Never override `rpy[2]` with velocity heading.** The SITL compass reads the same `R_orb` orientation, so compass and attitude always agree → no EKF yaw reset.
- `gyro_body` = `R_orb.T @ (omega_body − omega_spin_component)` — spin stripped (GB4008 keeps electronics non-rotating), projected into actual body frame.
- `accel_body` = `R_orb.T @ (accel_world − gravity)` — in actual body frame.
- The exact yaw direction of the electronics platform does not matter; it just needs to be steady. `build_orb_frame` (East-projection convention) provides a stable, deterministic yaw consistent with the GB4008 counter-torque assumption.
- `EK3_SRC1_YAW=1` (compass) + `COMPASS_USE=1`: compass is consistent with honest `rpy[2]` → correct EKF yaw source. On hardware also use `EK3_SRC1_YAW=1` during orbital flight.

---

## Current Limits

1. Single-body hub model — no blade multibody, no flapping DOF
2. Rotor spin is a scalar ODE — gyroscopic coupling computed but I_spin effect is small
3. Tether: tension-only elastic — no sag, no distributed mass, no reel dynamics
4. Aero: steady-state BEM — no dynamic inflow; Coleman skewed wake handles non-uniform induction
5. Controller: 10 Hz in stack test (MAVLink RC override); 400 Hz in unit tests / internal controller

---

## Phase 3 Plan

### M1 — Wire Pumping Cycle ✅  
### M2 — Force Balance & Rotor Abstraction ✅  
### M3 — ArduPilot Config & Full-Stack Flight (in progress)
- [x] test_pumping_cycle PASSES (SkewedWakeBEM, +2098 J, peak_tension=315 N)
- [x] rawes.lua orbit tracking validated (test_lua_flight_rc_overrides, internal_controller=True)
- [x] H_SW_PHANG=0 confirmed; H_SW_TYPE=3 (H3-120) confirmed
- [x] SCR_USER6=4 (landing) + SCR_USER6=5 (pumping) added; PUMP_LEN_THRESH cumulative tracking fixed
- [x] internal_controller=False mandated for all stack flight tests (CLAUDE.md critical rule)

**CRITICAL — Test progression: steady → pumping → landing.**
Pumping and landing build directly on the stable orbit established by steady flight.  **Whenever
any of the three tests fails, fix test_lua_flight_steady first.**  A broken steady orbit makes
pumping and landing failures uninformative — they will fail for reasons unrelated to their own
logic.  Do not debug test_pumping_cycle_lua or test_landing_stack until test_lua_flight_steady
passes cleanly (orbit_r < 5 m, altitude stable ±2 m, yaw gap < 15 deg for ≥ 60 s).

**Step 1 — Steady orbit under full ArduPilot control (current focus)**

The goal is a stack test where ArduPilot + rawes.lua (no internal Python controller) sustains a
stable orbit at tether equilibrium for ≥ 60 s with no crash and no EKF yaw reset.  This is the
foundation; pumping and landing are built on top.

Three sub-problems to solve in order:

1. **Kinematic exit conditions** — hub must exit kinematic with state that ArduPilot can hold
   without a jolt.  Key levers:
   - `vel0` / `pos0`: hub at tether equilibrium (pos0 = body_z * tether_m), vel0 = orbital velocity
     ([0, 0.96, 0] for East wind) so velocity heading = body_z heading → no GPS glitch.
   - `kinematic_vel_ramp_s`: ramp vel to 0 vs keep orbital vel at exit.  vel=0 eliminates linear
     tether jolt but requires thrust ≥ gravity at the moment of release (collective must be correct).
     Orbital vel (vel_ramp_s=0) exits in established motion but has a radial jolt component.
   - EKF settle time: GPS fuses at t≈23 s; body_z capture after KINEMATIC_SETTLE_MS (62 s).

2. **Collective management at kinematic exit** — with internal_controller=False, Lua's VZ
   controller runs during the last 3 s of kinematic.  EKF baro noise during the vel_ramp
   deceleration phase drives vz_EKF < 0, pulling collective below hover → free-fall at exit.
   Fix options: clamp collective ≥ COL_CRUISE in Lua landing mode; or start Lua only AFTER
   kinematic exit (KINEMATIC_SETTLE_MS > kinematic duration).

3. **Orbit stability** — after a clean kinematic exit, Lua (50 Hz) must maintain body_z at the
   tether equilibrium direction.  Key checks: KP_CYC gain, BZ_SLEW rate, yaw regulation (Ch8
   keepalive for motor interlock; ArduPilot yaw PID active).  EKF must not emergency-yaw-reset.

- [ ] test_lua_flight_steady: hub in stable orbit ≥ 60 s, internal_controller=False, no EKF yaw reset
- **Gate:** telemetry shows orbit_r < 5 m, altitude stable ±2 m, yaw gap < 15 deg for ≥ 60 s

**Step 2 — Pumping cycle under Lua (test_pumping_cycle_lua)**

Start from the steady-orbit state established in Step 1.  Switch SCR_USER6=5; LandingPlanner drives
winch for De Schutter reel-out/reel-in cycle; Lua detects phases from tether length change.

- [ ] test_pumping_cycle_lua passing (SCR_USER6=5, internal_controller=False)
- **Gate:** "RAWES pump: reel_out" STATUSTEXT + net_energy > 0 + peak_tension < 496 N

**Step 3 — Landing under Lua (test_landing_lua)**

Start from the steady-orbit state.  Switch SCR_USER6=4; LandingPlanner reels in tether;
Lua VZ controller descends hub; final_drop at alt ≤ 2 m.

- [ ] test_landing_lua passing (SCR_USER6=4, internal_controller=False)
- **Gate:** "RAWES land: captured" + "RAWES land: final_drop" STATUSTEXTs + hub alt ≤ 2.5 m

**Step 4 — ArduPilot parameter file**
- [ ] Configure GB4008: H_TAIL_TYPE=4 (DDFP), tune ATC_RAT_YAW_* and H_COL2YAW
- [ ] Write `rawes_params.parm` (full parameter file for Pixhawk 6C)
- **Gate:** rawes_params.parm exists + H_PHANG determined

### M4 — Hardware-in-the-Loop (Pixhawk 6C) — not started
- [ ] `hil_interface.py` — MAVLink HIL_SENSOR / HIL_GPS / HIL_ACTUATOR_CONTROLS
- [ ] `--hil-mode` in mediator; `test_hil_interface.py`; HIL bench procedure in stack.md
- **Gate:** test_hil_interface.py passes + 60 s HIL telemetry log
