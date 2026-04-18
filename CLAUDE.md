# RAWES — Project Context for Claude

## Project Goal

Build an **ArduPilot flight controller model** for a Rotary Airborne Wind Energy System (RAWES) — a tethered, 4-blade autogyro kite. Wind drives autorotation; cyclic pitch control steers; tether tension during reel-out drives a ground generator. No motor drives rotation.

**Current phase:** Phase 3, Milestone 3. `rawes.lua` orbit-tracking under full ArduPilot control (`internal_controller=False`) achieved: `test_lua_flight_steady` passes reliably (stable=86–110 s, 3/3 runs). GPS fusion uses dual GPS (EK3_SRC1_YAW=2, RELPOSNED heading): stationary hold, yaw known from first fix, GPS fuses at ~34 s. **Next: test_pumping_cycle, then test_landing_stack (stack).**

**Simtest status (14 simtests, 12 PASS):** All Python-only simtests pass: `test_steady_flight`, `test_pump_cycle`, `test_landing` (reel-in → xi~80° → descent → floor), gyroscopic, kinematic, sensor, deschutter. Lua simtests `test_pump_cycle_lua` and `test_landing_lua`: in development (never passed).

**Stack test status (parallel -n 8, 9 PASS):** test_arm_minimal, test_gps_fusion_layers, test_gust_recovery, test_lua_yaw_trim, test_pitch_roll, test_slow_rpm, test_startup, test_wobble, test_yaw_regulation. `test_lua_flight_steady` passes reliably (stable=86–110 s, 3/3). `test_pumping_cycle` and `test_landing_stack` (stack): in development.

**Test infrastructure:** Stack tests in `tests/sitl/flight/` + `tests/sitl/torque/`. Shared code in `stack_infra.py` (`_sitl_stack`, `_acro_stack`, `_torque_stack`). `conftest.py` is a thin re-exporter only.

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
| [hardware/dshot.md](hardware/dshot.md) | **DShot reference** — bench and flight-mode parameter tables, AM32 EDT, wiring, troubleshooting |
| [theory/pumping_cycle.md](theory/pumping_cycle.md) | De Schutter 2018 — pumping cycle, aero, structural constraints |
| [theory/orbit_mechanics.md](theory/orbit_mechanics.md) | Beaupoil 2026 — orbit characteristics, gyroscopic analysis |
| [theory/flap_dynamics.md](theory/flap_dynamics.md) | Weyel 2025 — flap state-space, feed-forward + PID, N4SID ID |
| [system/stack.md](system/stack.md) | **Complete flight control reference** — architecture, GCS, rawes.lua, EKF3, arming. Update when new EKF/arming behaviour is found. |
| [simulation/internals.md](simulation/internals.md) | Sensor design, aero model, tether, COL_MIN rules, SITL lockstep timing, known gaps |
| [simulation/history.md](simulation/history.md) | Phase 2 + Phase 3 decisions, root causes, and test results |
| [simulation/aero/deschutter.md](simulation/aero/deschutter.md) | De Schutter Eq. 25–31 validation vs. implementation |
| [simulation/torque_model.py](simulation/torque_model.py) | Counter-torque hub yaw ODE — `HubParams`, `HubState`, `motor_torque()`, `step()`, `equilibrium_throttle()` |
| [simulation/scripts/rawes.lua](simulation/scripts/rawes.lua) | Unified Lua controller (SCR_USER6 modes 0–8) |
| [simulation/tests/unit/README.md](simulation/tests/unit/README.md) | **Unit test guide** — test catalogue, Lua harness API, SCR_USER6 encoding, IC loading, telemetry |

---

## Key Design Decisions

- **Production aero:** `SkewedWakeBEMJit` (`aero/aero_skewed_wake_jit.py`) — Numba `@njit` drop-in. 18-test equivalence suite vs. reference (atol=1e-10). `create_aero(model="jit")`. Non-JIT version is human-readable reference only.
- **Two-loop attitude:** `compute_rate_cmd(kp, kd=0)` → rate setpoint; `RatePID(kp=2/3)` → swashplate tilt.
- **Portable core** in `controller.py`: `compute_bz_tether`, `slerp_body_z`, `compute_rate_cmd`, `col_min_for_altitude_rad` — map 1:1 to Lua.
- **High-tilt De Schutter:** xi=80° viable. `col_max=0.10 rad`, `col_min_reel_in=0.079 rad`. BEM invalid above xi≈85°.
- **body_z_slew_rate** = 0.40 rad/s (`rotor.body_z_slew_rate_rad_s`). Faster → oscillation; slower → wastes reel-in time.
- **RotorDefinition YAML fields:** `swashplate_pitch_gain_rad` (measurable via flap deflection); `CD_structural` (beaupoil_2026=0.0, de_schutter_2018=0.021).
- **Canonical telemetry** (`simulation/telemetry_csv.py`): 67 columns, `TelRow` dataclass. `damp_alpha`=0.0 in free flight; used by `analyse_run.py` to split kinematic vs. free-flight phases. `TelRow.heartbeat()` = mediator 1 Hz status line. **CSV files are always regenerated — never worry about backward compatibility for the CSV schema.**
- **rawes.lua SCR_USER6 modes (plain integer 0–8):** 0=none (passive: no RC overrides, logs every 5 s + any NV message), 1=steady (cyclic orbit-tracking, 50 Hz), 2=yaw (counter-torque yaw trim only, Ch9/SERVO9, 100 Hz), 3=reserved, 4=landing (cyclic + VZ descent + auto final_drop, 50 Hz), 5=pumping (De Schutter pumping cycle, 50 Hz), 6=arm_hold (Ch3=1000 Ch8=2000 keepalive only), 7=yaw_test (motor ON at 25% for 20 s then off, resets on re-entry, 100 Hz), 8=yaw_limited (yaw PI but motor hard-stops 30 s after first throttle, resets on re-entry). Modes 4+5 own Ch3 entirely. **Yaw trim alongside any flight mode:** send `NAMED_VALUE_FLOAT("RAWES_YAW", 1)` to enable, 0 to disable; cleared on every mode change. **Substates** (landing: 0=descend/1=final_drop; pumping: 0=hold/1=reel_out/2=transition/3=reel_in/4=transition_back) are delivered via `NAMED_VALUE_FLOAT("RAWES_SUB", N)` — never encoded in SCR_USER6. `_nv_floats` resets to `{}` on every mode change. Constants in `simulation/rawes_modes.py`; send via `gcs.send_named_float("RAWES_SUB", N)` or `sim.send_named_float("RAWES_SUB", N)` (unit harness).
- **GPS fusion timing:** `EK3_GPS_CHECK=0` + widened gates (`EK3_POS_I_GATE=50`, `EK3_VEL_I_GATE=50`) are required boot params (in `rawes_sitl_defaults.parm`). GPS origin sets during arm; "EKF3 IMU0 is using GPS" appears once EKF innovations settle. Without widened gates, GPS fusion fails.
- **EK3_SRC1_YAW=2** (dual-antenna GPS yaw, RELPOSNED) — `rawes_sitl_defaults.parm` boot default. Two F9P antennas 50 cm apart (±25 cm along body X) give yaw from the NED baseline vector. Yaw is known from the first GPS fix — no motion, no EKFGSF rotation needed. `delAngBiasLearned` converges with constant-zero gyro (~21 s after arm). `COMPASS_USE=0`, `COMPASS_ENABLE=0` — compass is physically useless near the GB4008 motor on hardware and SITL synthetic compasses cycle every 10 s blocking GPS fusion. `GPS_AUTO_CONFIG=0` is **critical**: prevents ArduPilot from reconfiguring the UBLOX chips over serial, which corrupts the RELPOSNED stream in SITL. See `rawes_sitl_defaults.parm` for the full parameter list.
- **rawes.lua landing (SCR_USER6=4):** captures body_z on `ahrs:healthy()`; VZ descent controller (`col_cmd = COL_CRUISE + KP_VZ*(vz_actual - VZ_LAND_SP)`, VZ_LAND_SP=0.5 m/s); `final_drop` (collective→0) triggered by ground planner sending `NAMED_VALUE_FLOAT("RAWES_SUB", 1)` (LAND_FINAL_DROP). Fixture: `acro_armed_landing_lua` (`internal_controller=False`, `kinematic_vel_ramp_s=20` so hub exits kinematic at vel=0 — eliminates linear tether jolt without internal controller).
- **rawes.lua pre-GPS collective bypass:** Before GPS fuses (`_tdir0 == nil`), the VZ altitude controller is bypassed and collective is held at `col_cruise` (-0.18 rad). Without this, GPS altitude noise biases EKF vz ≈ 0.24 m/s → VZ controller commands col=-0.168 → T_aero=249N > T_tether=199N → 50N net outward force → tether tension runaway → crash at kinematic exit. Implemented as `elseif _tdir0 == nil then col_cmd = col_cruise` before the VZ controller branch.
- **rawes.lua pre-GPS orbit tracking:** Before GPS fuses (`_tdir0 == nil`), `_bz_orbit` tracks `bz_now` every step (err=0, pure neutral stick). When GPS fuses (`yawAlignComplete` set via RELPOSNED, then `delAngBiasLearned`), GPS tether recapture fires: `_bz_eq0` and `_bz_orbit` are reset to `disk_normal_ned()` at that moment (zero step change) and `_tdir0 = normalize(diff)` anchors the orbit reference. Orbit tracking runs before kinematic exit — hub has active cyclic control at the transition.
- **rawes.lua no gyro feedthrough:** Cyclic is pure P control: `roll_rads = kp * err_bx`, `pitch_rads = kp * err_by`. Gyro feedthrough was removed because it is sensitive to EKF yaw error. The orbital body_z precession rate is ~0.01 rad/s; pure P with azimuthal orbit tracking handles it with negligible steady-state error.
- **EKF yaw cancellation in body-frame cyclic error:** `err_body = R_ekf.T @ (bz_now × bz_orbit)`. Both `bz_now` and `bz_orbit` live in the same EKF frame, so a constant yaw offset `Δψ` between EKF and physics cancels exactly. Stable (even if wrong) EKF yaw is sufficient for the P-gain error term.
- **`acro_armed_lua_full` kinematic design:** Stationary hold (vel0=[0,0,0], linear trajectory, 80 s). Hub sits at tether equilibrium; no motion required. With dual GPS (EK3_SRC1_YAW=2), RELPOSNED gives yaw from the first fix; `delAngBiasLearned` converges at ~34 s; GPS fuses → `_tdir0` fires → Lua orbit tracking active ~46 s before kinematic exit.
- **CRITICAL — Kinematic phase sensor consistency:** The kinematic trajectory is a purely artificial path used only to bring the EKF and GPS to a fused, healthy state before real physics begins. Tether and aero forces have no effect on the trajectory (hub follows the prescribed path regardless). **All sensors sent to SITL during kinematic must be physically consistent with the prescribed trajectory as if a magical external force holds the hub in place.** Specifically: (1) `accel_body = R.T @ (d_vel/dt - gravity)` — for a stationary hold, d_vel/dt=0 so accel_body = R.T @ [0,0,-g] (gravity in body frame). (2) `gyro_body = R.T @ omega_body` — sensor.py reports the full body angular velocity; no stripping. (3) `vel` sent directly (zero for stationary hold). Use `validate_sitl_sensors.py` to verify consistency after any kinematic change.
- **rawes.lua pumping (SCR_USER6=5):** phase is driven entirely by `NAMED_VALUE_FLOAT("RAWES_SUB", N)` from the ground planner (0=hold, 1=reel_out, 2=transition, 3=reel_in, 4=transition_back). Lua executes the correct collective and body_z slerp goal for each substate; the planner owns the phase state machine and winch timing.
- **Anchor in `LOCAL_POSITION_NED`:** `SCR_USER5 = -initial_state["pos"][2]` (NED Z negated). Anchor at `[0, 0, -pos0[2]]` in EKF frame.
- **SCR_ENABLE bootstrap:** After EEPROM wipe, scripting only starts if `SCR_ENABLE=1` is already in EEPROM. `acro_armed_lua` fixture sets it via MAVLink post-arm (persists for future boots).
- **Two orbit-tracking algorithms:** `orbit_tracked_body_z_eq` (azimuthal, inner-loop tests); `orbit_tracked_body_z_eq_3d` (3D Rodrigues, Lua-equivalent). `OrbitTracker` = stateful 3D + slerp, mirrors Lua state machine; `update(pos, dt, bz_target=None)`.
- **WinchNode** (`winch_node.py`): mediator calls `update_sensors(tension, wind_world)` (physics only); planner calls `get_telemetry()` + `receive_command()`. Wind seed from `Anemometer.measure()` at 3 m height.
- **Vertical landing:** direct drop above anchor (not spiral). Descent rate controller replaces TensionPI. Sequence: reel-in only (no reel-out, `T_REEL_OUT=0`) via `DeschutterPlanner` swings hub from xi~8° to xi~48° in 30 s with no slack/spikes; hub reaches ~67 m altitude. `LandingPlanner` then descends from ~89 m tether to 2 m, final_drop. Validated in `test_landing.py` (22 s run, floor hit, descent slack=0).
- **Gyroscopic phase NOT needed:** H_SW_PHANG=0. BASE_K_ANG=50 N·m·s/rad → τ≈0.08 s (damped before one orbit). `swashplate_phase_deg≠0` degrades orbit stability.
- **`AcroController use_servo=True`** required for De Schutter simtests (25 ms servo lag). `test_steady_flight` and pure-orbit tests do not need it.
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
├── mediator_torque.py   Standalone torque SITL mediator (RPM profiles, hub yaw ODE)
├── torque_model.py      Hub yaw physics — HubParams, HubState, motor_torque(), step(), equilibrium_throttle()
├── kinematic.py         KinematicStartup — hub trajectory during EKF init phase
├── winch_node.py        WinchNode + Anemometer (physics/planner protocol boundary)
├── gcs.py               MAVLink GCS client (arm, mode, RC override, params)
├── telemetry_csv.py     Canonical 67-column CSV schema (TelRow, COLUMNS, heartbeat)
├── viz3d/
│   ├── visualize_torque.py  Torque telemetry 3-panel replay (psi_dot, motor throttle, PWM)
│   └── torque_telemetry.py  TorqueTelemetryFrame dataclass for torque replay
└── tests/
    ├── unit/            Windows native, no Docker (~497 tests: 483 fast + 14 simtests)
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
        │   ├── test_gps_fusion_layers.py    GPS fusion diagnostic (layers + armed + trajectory)
        │   ├── test_h_phang.py              H_SW_PHANG calibration
        │   ├── test_kinematic_gps.py        GPS fusion during kinematic (parametrized)
        │   ├── test_lua_flight_steady.py    steady orbit under full ArduPilot/Lua control
        │   ├── test_nvf_receive.py          NAMED_VALUE_FLOAT receive round-trip gate test
        │   ├── test_pumping_cycle.py        pumping cycle under Lua (in dev)
        │   └── test_landing_stack.py        landing under Lua (test_landing_lua, in dev)
        └── torque/              torque/anti-rotation tests
            ├── conftest.py      torque fixtures: torque_armed, torque_armed_profile, etc.
            ├── torque_test_utils.py  shared loop/assert helpers for torque stack tests
            └── ...
```

**Data flow (400 Hz):** SITL servo PWM → swashplate mix → aero → tether → RK4 dynamics → sensor packet → SITL

---

## Coordinate Conventions

**NED everywhere.** X=North, Y=East, Z=Down. Gravity = `[0, 0, +9.81*m]`. Altitude = `-pos[2]`.

> ⚠️ **NED-only policy.** `T_ENU_NED` in `frames.py` is for legacy external data conversion only — never used in the simulation loop. Any `pos_enu`, `altitude = pos[2]` (without negation), or ENU-style arithmetic is a bug. `build_orb_frame(body_z)` from `frames.py` is the single source for the orbital frame.

---

## Workflow Rules

**CRITICAL — Keep `rawes_test_surface.lua` in sync with rawes.lua.** Lua unit tests run rawes.lua in-process via lupa. Constants and functions are exposed to Python tests through `rawes_test_surface.lua`'s `_rawes_fns` table, which is spliced inside rawes.lua's anonymous function wrapper and therefore has access to all **module-level** locals. **Whenever you add a local constant or function to rawes.lua that tests need to access, add it to `_rawes_fns` in `rawes_test_surface.lua` in the same commit.** Function-local variables are not accessible — hoist them to module level first.

`test_yaw_lua.py` and `test_math_lua.py` read all constants from `sim.fns.*` at module load — no Python copies to maintain. `test_math_lua.py` also cross-checks rawes.lua functions against `controller.py` equivalents; a failing test there means `controller.py` diverged from rawes.lua — fix `controller.py`.

**No silent defaults for physics parameters.** Raise `KeyError`/`ValueError` if a required config key is absent. Never use `dict.get("key", fallback)` or `x = x or default` for physical constants, control gains, or rotor/airfoil properties.

**Do NOT consult git history** (`git log`, `git diff`, `git show`, `git blame`) when diagnosing problems unless you first ask the user.

**CRITICAL — Fix telemetry/logging before diagnosing test failures.** When a simtest or stack test fails, inspect the telemetry CSV and logs first. If any columns are zero, missing, or clearly wrong (e.g. tether_m=0 throughout, tension_n=0, phase never changes), fix the logging bug before attempting to diagnose the physics failure. Diagnosing from bad telemetry produces wrong conclusions.

**Stack tests must not violate physics.** Never add artificial mechanisms just to stabilise a test. Acceptable: `lock_orientation=False`, `internal_controller=False`, `base_k_ang=50`.

**CRITICAL — `internal_controller` MUST be `False` for all full stack flight tests.** The entire purpose of SITL stack tests is to validate that ArduPilot + Lua actually fly the vehicle. Using `internal_controller=True` means the Python controller drives physics and ArduPilot/Lua outputs are ignored — the SITL test becomes meaningless. There are NO exceptions to this rule for stack tests. `internal_controller=True` is only valid in unit tests and simtests where Lua/ArduPilot are not involved.

**CRITICAL — SITL must run as close to hardware as possible.** Find and fix root causes. Do NOT paper over failures with simulation-only hacks. Confirm with user before adding any override.

**CRITICAL — GPS/EKF glitches mean physics inputs are wrong; fix the physics.** Do NOT disable EKF fusion sources or reduce thresholds. Telemetry column `orb_yaw_rad` (= `rpy[2]` with honest sensors) and `v_horiz_ms` are logged for diagnostics. With honest sensors, EKF emergency yaw resets indicate a compass/attitude mismatch in the physical model — fix the sensor output, not the EKF config.

**⚠️ NEVER use non-ASCII characters in Python `print()` output.** Windows cp1252 encoding causes `UnicodeEncodeError`. Use only 7-bit ASCII: `-` for lines, `[PASS]`/`[FAIL]` for status, `sd=` for sigma, etc.

---

## SITL Lockstep

See [simulation/internals.md](simulation/internals.md) (`## SITL Lockstep Protocol`) for the full reference.

**Key rule:** The physics worker must reply to every SITL servo packet without exception — skipping a reply causes ArduPilot to stall permanently. `gcs.sim_now()` returns `time_boot_ms/1000` from the most recently processed MAVLink message, not wall-clock time. `sim_sleep(N)` waits N sim-seconds; the physics loop must keep running during the wait.

---

## Running Tests

**Unit tests and simtests: Windows native, no Docker. Stack tests: Docker required. Never mix.**

**Two venvs — one per environment:**
- **`simulation/.venv`** — Windows venv for all unit tests and simtests. Packages: numpy, scipy, numba, lupa, pyvista, pymavlink, pytest, etc. `run_tests.py` auto-installs when `requirements.txt` changes (SHA-256 hash stamp). Manual update: `simulation/.venv/Scripts/pip install -r simulation/requirements.txt`.
- **Docker container** — the container has its own Python env (never use the Windows venv inside Docker). Managed by `dev.sh build`.

**CRITICAL:** Use Bash tool directly — do NOT use `wsl.exe`. Always use absolute paths.

**CRITICAL:** NEVER call `docker exec` directly to run stack tests. Use `bash simulation/dev.sh test-stack [...]`. If stuck: `bash simulation/dev.sh stop && bash simulation/dev.sh start`.

**Stack test isolation:** Each stack test always runs in its own fresh Docker container — clean EEPROM, no stale processes, no port conflicts between tests.

**CRITICAL:** Unit/simtests run via the Windows venv directly — NOT via `dev.sh test-unit` (which routes to Docker and fails because `tests/unit` is excluded from the container sync).

| Task | Command |
|------|---------|
| Unit tests (~483) | `simulation/.venv/Scripts/python.exe -m pytest simulation/tests/unit -m "not simtest" -q` |
| Simtests (14) | `simulation/.venv/Scripts/python.exe -m pytest simulation/tests/unit -m simtest -q` |
| Stack test (single) | `bash simulation/dev.sh test-stack -n 1 -k test_foo` |
| Stack test (full suite) | `bash simulation/dev.sh test-stack -n 8` |
| **Post-failure: physics** | `simulation/.venv/Scripts/python.exe simulation/analysis/analyse_run.py <test_name>` |
| **Post-failure: EKF/GPS** | `simulation/.venv/Scripts/python.exe simulation/analysis/analyse_mavlink.py <test_name>` |
| Regenerate `steady_state_starting.json` | `simulation/.venv/Scripts/python.exe -m pytest simulation/tests/unit/test_steady_flight.py::test_steady_state_hub_does_not_drift -s` — written by that test only; used by all stack tests as initial conditions |
| Container start/stop | `bash simulation/dev.sh start` / `bash simulation/dev.sh stop` |
| Docker build | `bash simulation/dev.sh build` (~30–60 min; use `run_in_background=true`, no trailing `&`) |
| Run inside container | `bash simulation/dev.sh exec 'python3 /rawes/simulation/...'` |

**Always run BOTH** `analyse_run.py` (physics telemetry) and `analyse_mavlink.py` (ArduPilot/EKF/GPS) after a stack test failure before reading raw logs.

**Stack test logs:** `simulation/logs/{test_name}/` — `mediator.log`, `sitl.log`, `gcs.log`, `telemetry.csv`, `arducopter.log`. Always read with the Windows path (`e:/repos/windpower/simulation/logs/...`). Suite summary: `simulation/logs/suite_summary.json`.

---

## ArduPilot ACRO Mode

**Use ACRO. Never STABILIZE** — STABILIZE holds NED roll=0, fighting the 65° tether equilibrium; yaw discontinuity at unfreeze → EKF emergency reset → crash within 4 s.

**Neutral sticks in ACRO crash:** zero rate command kills orbital precession → non-zero cyclic. Fix: `compute_rc_from_attitude` sends rates matching tether-relative attitude error.

**ACRO I-term windup:** 0.2–0.3 rad/s orbital angular velocity accumulates as rate error. Fix: `ATC_RAT_RLL_IMAX=ATC_RAT_PIT_IMAX=ATC_RAT_YAW_IMAX=0`.

**CRITICAL — Physically faithful sensors: no transformations, no overrides.** `sensor.py` must report exactly what the real Pixhawk hardware would see. The electronics hub is the fuselage — `R_hub` is its full 3-DOF orientation, integrated by the dynamics ODE:
- `rpy` = actual ZYX Euler angles from `R_hub` directly. `R_hub[:, 2]` = disk_normal (tilt, 2 DOF); `R_hub[:, 0]` = electronics x-axis (yaw, 1 DOF). **Never override `rpy[2]` with velocity heading.** The SITL compass reads the same `R_hub` orientation, so compass and attitude always agree → no EKF yaw reset.
- `gyro_body` = `R_hub.T @ omega_body` — full body angular velocity projected into electronics body frame. The GB4008 keeps the electronics non-rotating via the K_YAW damping torque in dynamics; no stripping is applied in the sensor. The sensor faithfully reports whatever the body does, just like a helicopter IMU reads full body rate regardless of the tail rotor.
- `accel_body` = `R_hub.T @ (accel_world − gravity)` — in electronics body frame.
- Yaw is a real physical DOF — not a convention. The GB4008 counter-torque damps yaw around `disk_normal` and is modelled explicitly in the mediator as `M_orbital += -K_YAW * dot(omega, disk_normal) * disk_normal`. `K_YAW` (default 100 N·m·s/rad) = near-perfect damper; reduce to model drift.
- `EK3_SRC1_YAW=2` (dual-antenna GPS yaw) + `COMPASS_USE=0`: compass disabled (GB4008 motor interference on hardware; cycling in SITL). EKF derives yaw from the RELPOSNED baseline vector between two F9P antennas. Yaw is known from the first GPS fix — no motion required.

---

## Current Limits

1. Single-body hub model — no blade multibody, no flapping DOF
2. Rotor spin is a scalar ODE — gyroscopic coupling computed but I_spin effect is small
3. Tether: tension-only elastic — no sag, no distributed mass, no reel dynamics
4. Aero: steady-state BEM — no dynamic inflow; Coleman skewed wake handles non-uniform induction
5. Controller: 10 Hz in stack test (MAVLink RC override); 400 Hz in unit tests / internal controller

---

## Hardware DShot Configuration

See [hardware/dshot.md](hardware/dshot.md) for bench and flight-mode parameter tables, AM32 EDT setup, wiring, and troubleshooting.

GB4008 wired to **AUX OUT 1 (output 9, FMU)**. In flight mode (`SERVO9_FUNCTION=94`): Lua writes GB4008 PWM via `SRV_Channels` (Heli frame). In bench mode (`SERVO9_FUNCTION=36`): direct PWM via Rover frame (`MOT_PWM_TYPE=6`). Use `calibrate.py bench-mode` / `flight-mode` to toggle. In DShot mode the ESC auto-arms from the first valid DShot packet — no PWM arming sequence needed.

---

## Phase 3 Plan

See [simulation/history.md](simulation/history.md) for full decision history (M1/M2 results, Step 1 root causes and fixes).

**CRITICAL — Test progression: steady → pumping → landing.** Fix `test_lua_flight_steady` (stack) before debugging pumping or landing Lua stack tests. A broken steady orbit makes downstream failures uninformative. Do not debug `test_pumping_cycle` or `test_landing_stack` (stack) until `test_lua_flight_steady` passes cleanly (orbit_r < 5 m, altitude stable ±2 m, yaw gap < 15 deg for ≥ 60 s).

| Milestone | Status | Gate |
|-----------|--------|------|
| M1 Wire Pumping Cycle | done | — |
| M2 Force Balance & Rotor Abstraction | done | — |
| M3 Step 1 — test_lua_flight_steady (stack) | done: stable=86–110 s, 3/3 runs | orbit_r < 5 m, no EKF yaw reset, ≥ 60 s stable |
| M3 Step 1b — test_landing.py (simtest) | done: reel-in → xi~80° → descent → floor | descent slack=0, floor hit, anchor_dist < 20 m |
| M3 Step 2 — test_pumping_cycle (stack, SCR_USER6=5) | in dev | "RAWES pump: reel_out" + net_energy > 0 + peak_tension < 496 N |
| M3 Step 3 — test_landing_stack (stack, SCR_USER6=4) | in dev | "RAWES land: captured" + "final_drop" + hub alt ≤ 2.5 m |
| M3 Step 4 — rawes_params.parm (Pixhawk 6C) | not started | file exists + H_PHANG determined |
| M4 — Hardware-in-the-Loop (Pixhawk 6C) | not started | test_hil_interface.py passes + 60 s HIL log |
