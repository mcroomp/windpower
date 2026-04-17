# RAWES — Project Context for Claude

## Project Goal

Build an **ArduPilot flight controller model** for a Rotary Airborne Wind Energy System (RAWES) — a tethered, 4-blade autogyro kite. Wind drives autorotation; cyclic pitch control steers; tether tension during reel-out drives a ground generator. No motor drives rotation.

**Current phase:** Phase 3, Milestone 3. `rawes.lua` orbit-tracking under full ArduPilot control (`internal_controller=False`) achieved: `test_lua_flight_steady` passes reliably (stable=86–110 s, 3/3 runs). Three root causes fixed: (1) `vel0=[0,0.96,0]` (East) so GPS fuses at t≈23 s during kinematic → orbit tracking active before exit; (2) gyro feedthrough removed from Lua cyclic (EKF-yaw-sensitive, caused ~33% flakiness); (3) pre-GPS collective bypass in rawes.lua (col=cruise while `_tdir0==nil`). **Next: test_pumping_cycle_lua, then test_landing_lua.**

**Stack test status (parallel -n 8, 8 PASS):** test_arm_minimal, test_gust_recovery, test_lua_yaw_trim, test_pitch_roll, test_slow_rpm, test_startup, test_wobble, test_yaw_regulation. `test_lua_flight_steady` passes reliably (stable=86–110 s, 3/3). `test_pumping_cycle_lua` and `test_landing_lua`: in development.

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
- **Canonical telemetry** (`simulation/telemetry_csv.py`): 67 columns, `TelRow` dataclass. `damp_alpha`=0.0 in free flight; used by `analyse_run.py` to split kinematic vs. free-flight phases. `TelRow.heartbeat()` = mediator 1 Hz status line. **CSV files are always regenerated — never worry about backward compatibility for the CSV schema.**
- **rawes.lua SCR_USER6 modes:** 0=none, 1=flight cyclic (50 Hz), 2=yaw trim (100 Hz), 3=both, 4=landing (cyclic + VZ descent + auto final_drop, 50 Hz), 5=pumping (De Schutter cyclic + per-phase collective, 50 Hz). Modes 4+5 own Ch3 entirely.
- **GPS fusion timing:** `EK3_GPS_CHECK=0` + widened gates (`EK3_POS_I_GATE=50`, `EK3_VEL_I_GATE=50`) are required boot params (in `rawes_sitl_defaults.parm`). GPS origin sets during arm; "EKF3 IMU0 is using GPS" appears once EKF innovations settle. Without widened gates, GPS fusion fails.
- **EK3_SRC1_YAW=8** (GPS velocity yaw) — `rawes_sitl_defaults.parm` boot default. `COMPASS_USE=0`, `COMPASS_ENABLE=0`. Compass is physically useless near the GB4008 motor on hardware, and SITL's 3 synthetic compasses cycle every 10 s (EKF3 `magFailTimeLimit_ms`) when innovations fail, blocking GPS fusion. GPS velocity yaw uses EKFGSF (Gaussian Sum Filter) with N=5 heading hypotheses. **EKFGSF requires turning motion to converge**: a constant straight-line velocity makes all hypotheses predict identical NED velocities → zero innovation divergence → YCS stays at maximum → `yawAlignComplete` never set → EKF stays in const_pos_mode forever.
- **rawes.lua landing (SCR_USER6=4):** captures body_z on `ahrs:healthy()`; VZ descent controller (`col_cmd = COL_CRUISE + KP_VZ*(vz_actual - VZ_LAND_SP)`, VZ_LAND_SP=0.5 m/s); `final_drop` (collective→0) when alt ≤ 2.0 m. Fixture: `acro_armed_landing_lua` (`internal_controller=False`, `kinematic_vel_ramp_s=20` so hub exits kinematic at vel=0 — eliminates linear tether jolt without internal controller).
- **rawes.lua pre-GPS collective bypass:** Before GPS fuses (`_tdir0 == nil`), the VZ altitude controller is bypassed and collective is held at `col_cruise` (-0.18 rad). Without this, GPS altitude noise biases EKF vz ≈ 0.24 m/s → VZ controller commands col=-0.168 → T_aero=249N > T_tether=199N → 50N net outward force → tether tension runaway → crash at kinematic exit. Implemented as `elseif _tdir0 == nil then col_cmd = col_cruise` before the VZ controller branch.
- **rawes.lua pre-GPS orbit tracking:** Before GPS fuses (`_tdir0 == nil`), `_bz_orbit` tracks `bz_now` every step (err=0, pure neutral stick). When GPS fuses (after EKFGSF converges and `yawAlignComplete` sets), GPS tether recapture fires: `_bz_eq0` and `_bz_orbit` are reset to `disk_normal_ned()` at that moment (zero step change) and `_tdir0 = normalize(diff)` anchors the orbit reference. Orbit tracking runs before kinematic exit — hub has active cyclic control at the transition.
- **rawes.lua no gyro feedthrough:** Cyclic is pure P control: `roll_rads = kp * err_bx`, `pitch_rads = kp * err_by`. Gyro feedthrough was removed because it is sensitive to EKF yaw error. The orbital body_z precession rate is ~0.01 rad/s; pure P with azimuthal orbit tracking handles it with negligible steady-state error.
- **EKF yaw cancellation in body-frame cyclic error:** `err_body = R_ekf.T @ (bz_now × bz_orbit)`. Both `bz_now` and `bz_orbit` live in the same EKF frame, so a constant yaw offset `Δψ` between EKF and physics cancels exactly. Stable (even if wrong) EKF yaw is sufficient for the P-gain error term.
- **`acro_armed_lua_full` kinematic design:** Orbital kinematic (`kinematic_traj_type="orbital"`). Hub circles anchor at r_h≈99 m, alt=14 m, v_orb=0.96 m/s. GPS velocity heading rotates at ω≈0.0097 rad/s (0.56 deg/s), giving EKFGSF the discriminating signal to converge on true heading → `yawAlignComplete` set → EKF exits const_pos_mode → GPS fuses → `_tdir0` fires → Lua orbit tracking active before kinematic exit. Hub exits kinematic mid-orbit at orbital velocity → smooth free-flight entry with active tether tension.
- **CRITICAL — Kinematic phase sensor consistency:** The kinematic trajectory is a purely artificial path used only to bring the EKF and GPS to a fused, healthy state before real physics begins. Tether and aero forces have no effect on the trajectory (hub follows the prescribed path regardless). **All sensors sent to SITL during kinematic must be physically consistent with the prescribed trajectory as if a magical external force is moving the hub along it.** Specifically: (1) `accel_body = R.T @ (d_vel/dt - gravity)` — the body-frame specific force must equal the kinematic acceleration minus gravity, rotated into body frame. (2) `gyro_body = R.T @ omega_body` — sensor.py reports the FULL body angular velocity from `hub_state["omega"]`; no stripping. The gyro must be consistent with R_fn so the EKF sees matching attitude and rate. (3) `vel` sent directly. (4) Body orientation `R_fn(t)` must use `build_vel_aligned_frame(body_z, vel)` so that body yaw tracks the GPS velocity heading — otherwise EKFGSF sees a heading gap and fails to converge. (5) `dynamics._omega` (free-flight initial condition) has the GPS-heading spin component stripped to avoid injecting artificial ~1 rad/s at kinematic exit — this is separate from `hub_state["omega"]` used by sensors. Use `validate_sitl_sensors.py` to verify consistency after any kinematic trajectory change.
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
        │   ├── test_gps_fusion_layers.py    GPS fusion diagnostic (layers + armed + trajectory)
        │   ├── test_h_phang.py              H_SW_PHANG calibration (test_h_swash_phang)
        │   ├── test_kinematic_gps.py        GPS fusion during kinematic (parametrized)
        │   ├── test_lua_flight_steady.py    steady orbit under full ArduPilot/Lua control
        │   ├── test_pumping_cycle.py        pumping cycle under Lua (test_pumping_cycle_lua)
        │   └── test_landing_stack.py        landing under Lua (test_landing_lua)
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

## Timing: sim time, wall time, lockstep, and sim_sleep

### SITL lockstep protocol

ArduPilot SITL uses a **lockstep** physics protocol:

1. ArduPilot sends a binary servo packet (UDP port 9002) and then **blocks** waiting for a state reply.
2. The physics backend (mediator) receives the packet, integrates one timestep (400 Hz = 2.5 ms), and sends back a JSON state packet.
3. ArduPilot unblocks, processes the state, and emits MAVLink messages with the new `time_boot_ms`.

Because ArduPilot cannot advance until it receives a reply, the physics worker must **reply to every servo packet without exception**. Any attempt to rate-limit the physics loop using sim time will cause ArduPilot to stall.

### sim_now() — ArduPilot's internal clock, not wall-clock

`gcs.sim_now()` returns `time_boot_ms / 1000.0` from the most recently **processed** MAVLink message. This is ArduPilot's internal simulation clock:

- It advances only when MAVLink messages are received (which requires the physics loop to be running).
- At SITL speedup=1 (default), sim time ≈ wall time (roughly 1:1), but they are **not** guaranteed equal.
- Returns 0.0 before the first MAVLink message is received.
- All test timeouts and deadlines (arm, set_mode, wait_ekf_attitude, etc.) are expressed in sim seconds.
- **`sim_now()` is always consistent with the message that caused `_recv` to return.** See below.

### `_recv` internal buffer — clock consistency guarantee

`_recv` drains all available network bytes into an internal deque, then pops and processes messages one at a time. It returns on the first match, leaving the rest for the next call. **Result: `sim_now()` always equals the `time_boot_ms` of the message that caused `_recv` to return** — never a later timestamp from the same network burst. `SimClock.update()` asserts non-decreasing timestamps (zero skipped); out-of-order delivery surfaces immediately as `AssertionError`.

### sim_sleep(N) — waits N sim-seconds, not N wall-seconds

`gcs.sim_sleep(N)` loops calling `_recv(blocking=True, timeout=0.1)` until sim time has advanced by N seconds. It does **not** call `time.sleep()`. Key properties:

- **The physics worker must keep running** while `sim_sleep` is active — otherwise ArduPilot stalls, no MAVLink messages arrive, and `sim_sleep` never returns.
- Every received message — whether it matches a type filter or not — advances the sim-clock and is written to the MAVLink log. The type filter in `_recv` only controls what is *returned* to the caller; discarded messages still tick the clock. Tests can inspect `gcs_log` after `sim_sleep` to check what happened during that window.
- At speedup=1, `sim_sleep(70)` takes ~70 seconds of real time.

### Lockstep anti-pattern: rate-limiting inside a physics worker

```python
# WRONG — deadlock if worker skips replies
def worker():
    while True:
        servos = sitl.recv_servos()
        if sitl.sim_now() - last_send > 0.1:   # <-- rate limiting = skipping replies
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
# CRITICAL: Stack tests MUST use dev.sh — NEVER run them via run_tests.py or the unit venv.
#   Wrong: simulation/.venv/Scripts/python.exe simulation/run_tests.py --fresh -n 1 -k test_foo
#   Wrong: simulation/tests/unit/.venv/Scripts/python.exe simulation/run_tests.py ...
#   Right: bash simulation/dev.sh test-stack-parallel --fresh -n 1 -k test_foo
# The pytest skill defaults to run_tests.py (unit venv) — override it for any test under tests/sitl/.
# DEFAULT: always use --fresh. Each test gets its own isolated container + clean EEPROM.
# Full suite (parallel, -n 8 is safe — each test owns its own ports inside its own container):
bash simulation/dev.sh test-stack-parallel --fresh -n 8
# Single test:
bash simulation/dev.sh test-stack-parallel --fresh -n 1 -k test_lua_flight_steady
# GPS fusion diagnostic tests:
bash simulation/dev.sh test-stack-parallel --fresh -n 1 -k test_gps_fusion_armed
bash simulation/dev.sh test-stack-parallel --fresh -n 1 -k "test_gps_fusion_layers[L0"

# Post-run analysis (always do this after a stack test failure -- run BOTH tools)
simulation/.venv/Scripts/python.exe simulation/analysis/analyse_run.py test_lua_flight_steady
simulation/.venv/Scripts/python.exe simulation/analysis/analyse_mavlink.py test_lua_flight_steady
# analyse_run.py  -- physics telemetry: altitude, tension, orbit quality, EKF sensor consistency
# analyse_mavlink.py -- ArduPilot view: EKF flags (correct names), GPS acquisition, STATUSTEXT timeline
# Note: analyse_run.py can also run inside the container via dev.sh exec; analyse_mavlink.py runs on Windows.
```

**Per-test logs:** `simulation/logs/{test_name}/` — `mediator.log`, `sitl.log`, `gcs.log`, `telemetry.csv`, `arducopter.log`. The directory name is always `request.node.name` (the pytest test node name, which includes parametrize brackets). Always read these with the Read tool using the local Windows path (`e:/repos/windpower/simulation/logs/.../arducopter.log`); never use `docker exec cat /tmp/ArduCopter.log` (stale across tests, requires container access). **`Loaded defaults from ...` prints once per parameter group — multiple repetitions are normal ArduPilot startup behavior, not a crash indicator.**

Example log paths:
- `simulation/logs/test_lua_flight_steady/` — steady flight test
- `simulation/logs/test_gps_fusion_armed/` — GPS diagnostic
- `simulation/logs/test_gps_fusion_layers[L0_baseline]/` — parametrized layer test
- `simulation/logs/test_slow_rpm[slow_vary]/` — parametrized torque test

**Suite summary:** `simulation/logs/suite_summary.json` — pass/fail counts + failed list.

| Task | Command |
|------|---------|
| **Post-failure: physics report** | `simulation/.venv/Scripts/python.exe simulation/analysis/analyse_run.py <test_name>` |
| **Post-failure: ArduPilot/EKF/GPS** | `simulation/.venv/Scripts/python.exe simulation/analysis/analyse_mavlink.py <test_name>` |
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

**CRITICAL — Physically faithful sensors: no transformations, no overrides.** `sensor.py` must report exactly what the real Pixhawk hardware would see. The electronics hub is the fuselage — `R_hub` is its full 3-DOF orientation, integrated by the dynamics ODE:
- `rpy` = actual ZYX Euler angles from `R_hub` directly. `R_hub[:, 2]` = disk_normal (tilt, 2 DOF); `R_hub[:, 0]` = electronics x-axis (yaw, 1 DOF). **Never override `rpy[2]` with velocity heading.** The SITL compass reads the same `R_hub` orientation, so compass and attitude always agree → no EKF yaw reset.
- `gyro_body` = `R_hub.T @ omega_body` — full body angular velocity projected into electronics body frame. The GB4008 keeps the electronics non-rotating via the K_YAW damping torque in dynamics; no stripping is applied in the sensor. The sensor faithfully reports whatever the body does, just like a helicopter IMU reads full body rate regardless of the tail rotor.
- `accel_body` = `R_hub.T @ (accel_world − gravity)` — in electronics body frame.
- Yaw is a real physical DOF — not a convention. The GB4008 counter-torque damps yaw around `disk_normal` and is modelled explicitly in the mediator as `M_orbital += -K_YAW * dot(omega, disk_normal) * disk_normal`. `K_YAW` (default 100 N·m·s/rad) = near-perfect damper; reduce to model drift.
- `EK3_SRC1_YAW=8` (GPS velocity yaw) + `COMPASS_USE=0`: compass disabled (GB4008 motor interference on hardware; cycling in SITL). EKF derives yaw from GPS velocity heading. Requires vehicle moving ≥ ~0.5 m/s — met during orbital flight at 0.96 m/s.

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
- [x] test_pumping_cycle_lua PASSES (SkewedWakeBEM, +2098 J, peak_tension=315 N)
- [x] rawes.lua orbit tracking validated (internal_controller=True, predecessor to test_lua_flight_steady)
- [x] H_SW_PHANG=0 confirmed; H_SW_TYPE=3 (H3-120) confirmed
- [x] SCR_USER6=4 (landing) + SCR_USER6=5 (pumping) added; PUMP_LEN_THRESH cumulative tracking fixed
- [x] internal_controller=False mandated for all stack flight tests (CLAUDE.md critical rule)
- [x] test_lua_flight_steady PASSES reliably: stable=86–110 s, 3/3 runs, max_activity≤1000 PWM, no EKF yaw reset (three fixes: vel0=East so GPS fuses during kinematic; no gyro feedthrough; pre-GPS col=cruise bypass)

**CRITICAL — Test progression: steady → pumping → landing.**
Pumping and landing build directly on the stable orbit established by steady flight.  **Whenever
any of the three tests fails, fix test_lua_flight_steady first.**  A broken steady orbit makes
pumping and landing failures uninformative — they will fail for reasons unrelated to their own
logic.  Do not debug test_pumping_cycle_lua or test_landing_lua until test_lua_flight_steady
passes cleanly (orbit_r < 5 m, altitude stable ±2 m, yaw gap < 15 deg for ≥ 60 s).

**Step 1 — Steady orbit under full ArduPilot control** ✅

Three root causes fixed:

1. **GPS fusion too late** — `vel0=tangential` gave GPS heading=163.6°; GPS did not fuse until
   after kinematic exit (t>65 s) → `_tdir0` was nil at exit → zero cyclic correction → tether
   restoring torque spun hub uncontrolled → flip and crash within 1 s.
   Fix: `vel0=[0,0.96,0]` (East) → GPS fuses at t≈23 s during kinematic → `_tdir0` fires →
   orbit tracking runs for 42 s before exit → hub has active cyclic control at the transition.

2. **Gyro feedthrough EKF-yaw sensitive** — `roll_rads = kp*err_bx + gyro_x` added a rate bias
   in EKF body frame (~46° off from physical frame with vel0=East).  At kinematic exit, if EKF
   yaw was unstable, feedthrough applied torque in the wrong axes → hub tumbled (~33% flakiness).
   Fix: removed gyro feedthrough.  Pure P control `roll_rads = kp * err_bx`.  Orbital precession
   rate is ~0.01 rad/s; the P gain + azimuthal orbit tracking handle it with negligible lag.

3. **VZ controller used biased EKF vz before GPS** — GPS altitude noise biased EKF vz≈0.24 m/s
   before GPS fused → col=-0.168 instead of -0.18 → T_aero=249N > T_tether=199N → 50N net
   outward force → tension runaway at kinematic exit.
   Fix: `elseif _tdir0 == nil then col_cmd = col_cruise` bypasses VZ controller before GPS.

- [x] test_lua_flight_steady: hub in stable orbit ≥ 60 s, internal_controller=False, no EKF yaw reset
- **Gate:** met — stable=86–110 s, orbit_r < 5 m, altitude stable, no yaw reset

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
