# RAWES — Project Context for Agents

## Project Goal

Build an **ArduPilot flight controller** for a Rotary Airborne Wind Energy System (RAWES) — a tethered, 4-blade autogyro kite. Wind drives autorotation; cyclic pitch control steers; tether tension during reel-out drives a ground generator. No motor drives rotation.

**Current phase:** Phase 3, Milestone 3. The ground-to-air link carries only two slow setpoints — **commanded tension** (`RAWES_TEN`) and **target altitude** (`RAWES_ALT`) — never measured/actual tension and never fast feedback. `rawes.lua` decouples flight control into: **orientation** (feedforward force balance from commanded tension + actual position + gravity → disk-axis direction), **altitude** (50 Hz PID on altitude error → collective), and **attitude** (ArduPilot native 400 Hz rate PID → cyclic). There is **no TensionPI on the AP**; the only tension feedback loop lives on the ground winch (its own load cell → reel speed). Pre-GPS: gyro feedthrough only. GPS fusion uses dual GPS (`EK3_SRC1_YAW=2`, RELPOSNED heading). See [design/tension_collective_control_loop.md](design/tension_collective_control_loop.md). **Next: fix `test_lua_flight_steady_sitl` regression (stack), then validate `test_pumping_cycle_lua_sitl` and `test_landing_lua_sitl` (stack).**

**Status:**
- Simtests (13): 11 PASS. `test_landing` / `test_landing_lua` failing (winch control during descent).
- Stack tests: `test_lua_flight_steady_sitl` **regressed** by aero migration (body rate runs away from ATC PID ~250 ms after kinematic exit). `test_pumping_cycle_lua_sitl` + `test_landing_lua_sitl` (stack) in dev.

See [design/history.md](design/history.md) for milestone history and the milestone table at the end of this file for current gates.

---

## Reference Documents

All design docs live under `design/`. `AGENTS.md` is the standard agent context file; most architecture details live in the design docs.

**System & flight stack**
| File | Purpose |
|------|---------|
| [design/physical.md](design/physical.md) | Physical system parameters — rotor, tether, motor, servos, winch motor/generator spec |
| [design/flight_stack.md](design/flight_stack.md) | **Complete flight control reference** — architecture, GCS, rawes.lua modes 0/1/3/4, EKF3, arming, channel ownership, ArduPilot configuration |
| [design/ardupilot_pids.md](design/ardupilot_pids.md) | ArduPilot heli rate-PID stack — `AC_AttitudeControl_Heli`, Python re-implementation |
| [design/ardupilot_swashplate.md](design/ardupilot_swashplate.md) | Swashplate signal flow: RC3 collective → servo PWM (H3-120) |
| [design/ekf_const_pos_mode.md](design/ekf_const_pos_mode.md) | EKF3 `const_pos_mode` — causes and log diagnostics |
| [design/EKF_GATING.md](design/EKF_GATING.md) | **EKF3 GPS-aiding & yaw-alignment gating** — `readyToUseGPS()` 7 gates, moving-baseline GPS-yaw flow (base/rover, `gps_yaw_deg` XOR), `alignYawAngle` vs GSF `realignYawGPS`, `storedYawAng.recall()` timestamp failure, ArduPilot source index. Verified root cause of `test_lua_flight_steady_sitl` const_pos hang |

**Simulation**
| File | Purpose |
|------|---------|
| [design/simulation.md](design/simulation.md) | **Simulation internals** — sensor design, controller stack, dynamics, aero, tether, kinematic startup, pumping/landing architecture, **module map**, SITL lockstep |
| [design/sitl_testing.md](design/sitl_testing.md) | **SITL stack testing** — read when editing or diagnosing SITL stack tests: how to run them (`test.sh stack`), the `diagnose_sitl.py` CHECK 1/CHECK 2 post-run workflow, `analyse_run.py`, the SITL lockstep contract, and the one central kinematic-hold implementation + canonical IC-start timeline. Not loaded by default — pull in only for SITL work. |
| [design/aero_conventions.md](design/aero_conventions.md) | **Aero model interface** — RotorInputs/RotorOutput frames, sign conventions, cyclic tilts, moment transformations |
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

#### Frames used in the project

The whole simulation loop runs in **one frame: NED world + FRD body**. ENU only appears at the edges (visualization, legacy ingest, paper-reproduction scripts) and never inside the physics.

| Frame | Where it lives | Axes |
|-------|----------------|------|
| **NED world** | `dynamics`, `tether`, `controller`, `physics_core`, `mediator`, `sensor`, `aero`, `rawes.lua` | X=North, Y=East, Z=Down. Gravity = `[0, 0, +9.81·m]`. Altitude = `−pos[2]`. |
| **FRD body** | Columns of any `R_hub` / `R_body` matrix in the project | `R[:,0]` = forward (nose), `R[:,1]` = right, **`R[:,2]` = hub axis pointing DOWN through the disk** (toward the ground for level hover; toward the anchor in tethered hover). Matches ArduPilot / EKF / `aero` package end-to-end. |
| **ENU** (legacy / external only) | `frames.T_ENU_NED` (ingest only), `viz3d/visualize_3d.py` (PyVista is Z-up), a handful of `analysis/` and `references/` scripts that follow paper conventions | X=East, Y=North, Z=Up. **Never** used inside the simulation loop. |

There is exactly one body-frame rotation convention. The body axes are described as vectors in NED via the columns of `R`; ENU never enters the physics path.

#### Origin conventions

The world frame is NED, but the **origin** can differ between subsystems. Same axes, only the translation changes — there is no rotation between origins.

| Origin | Used by | Anchor location | Notes |
|--------|---------|-----------------|-------|
| **Anchor-origin NED** | Physics, simtests, unit tests | `anchor_ned = [0, 0, 0]` | `pos` is hub-relative-to-anchor; `tdir = -pos / |pos|` gives FRD body_z directly. |
| **GPS / EKF-home NED** | Mediator, ArduPilot, stack tests | Anchor sent post-arm via NVFs `RAWES_ANN`/`RAWES_ANE`/`RAWES_AND` | `LOCAL_POSITION_NED` origin is the GPS first-fix home, not the anchor. Code that derives tether direction must use the actual anchor offset, not zero. |

Because only the origin translates (no rotation), any vector that comes from a difference (velocity, gyro, body_z) is identical in both origins. Only `pos` and `anchor_pos` care which origin the caller is using. Anywhere you see `pos / np.linalg.norm(pos)` is silently assuming anchor-at-origin and is fragile in the mediator path.

#### Signs

- **`body_z` is "down through the disk", not "up".** For a level hover, `R[:,2] = [0, 0, +1]` in NED. For tethered hover, `body_z = (anchor − pos) / |anchor − pos|`.
- **Thrust sign:** upward thrust = `−F_world[2]` (NED Z is down). The aero returns `F_world = −T·R[:,2]`, so for `T > 0` and `R[:,2] = [0,0,+1]`, `F_world = [0,0,−T]` ⇒ `F_world[2] < 0` is upward. Verified by `test_hover_sign.py`.
- **Cyclic (helicopter signs, `HeliCyclicController`):** `tilt_lon > 0` ⇒ nose-down disk (forward stick); `tilt_lat > 0` ⇒ roll right. `HeliCyclicController` maps body-rate-roll → `tilt_lat` and body-rate-pitch → `−tilt_lon`.
- **Pitch / roll body rates (FRD standard):** `+roll_rate` = right wing drops, `+pitch_rate` = nose up, `+yaw_rate` = nose right. So "nose-down" command is `−pitch_rate`, matched in `HeliCyclicController.step()` (`pitch_cyclic → −tilt_lon`) and `simtest_runner.step_guided()`.

#### ArduPilot pitch vs. our `tilt_lon` — the mandatory sign flip

There are **two different "pitch" conventions** in the stack and they are **opposite in sign**. Conflating them inverts the cyclic and causes positive feedback (the body rate runs away from the ATC rate PID, or — in the kinematic `nul` aero — rotates *away* from the commanded attitude until the crash check disarms).

| Quantity | Domain | Sign convention | Positive means |
|----------|--------|-----------------|----------------|
| ArduPilot attitude/cyclic pitch (`ATTITUDE.pitch`, EKF, `pitch_norm` from `ardupilot_h3_120_inverse`, `heli_out.pitch_cyclic`) | aircraft attitude (FRD, RH about +y) | aerospace standard | **nose-up** |
| Project `tilt_lon` (`RotorInputs.tilt_lon`, aero, `nul` aero, telemetry) | helicopter cyclic / forward-stick | rotor-disk tilt | **nose-down** (forward stick) |

Because they are opposite:

$$\boxed{\text{tilt\_lon} = -\,\text{pitch\_cyclic}_{\text{ArduPilot}}} \qquad \text{tilt\_lat} = +\,\text{roll\_norm}_{\text{ArduPilot}}\ \text{(roll has NO flip)}$$

- **Verification:** the current `quasi_static` aero gives `d(M_body_pitch)/d(tilt_lon) ≈ −1900` (nose-down moment for `tilt_lon > 0`) — consistent with the table. Probe: `simulation/tests/oneoff/probe_tilt_lon_sign.py`.
- **Where the flip lives:** the simtest path `simtest_runner.step_guided()` does `tilt_lon_cmd = -heli_out.pitch_cyclic` (correct). The stack path **must** do the same after `ardupilot_h3_120_inverse`: `_tilt_lon = -pitch_norm` in [simulation/mediator.py](simulation/mediator.py). Roll is `_tilt_lat = roll_norm` (no flip) in both.
- **Unit guard:** `test_cyclic_direction_mapping.py::test_pitch_cyclic_positive_to_tilt_lon_negative` encodes the flip. Do **not** "simplify" `_tilt_lon = -pitch_norm` to `= pitch_norm`.
- **Only pitch flips, never roll/yaw.** ArduPilot roll (`+roll` = right wing down) already matches `tilt_lat > 0` = roll right.

#### Downwind-plane criteria

- **The kite should remain on the wind-aligned downwind plane during pumping and steady flight.** Horizontal motion should be aligned with the wind direction and tether geometry, not an explicit sideways circular target around the anchor.
- **Controllers do not magically know the true wind direction.** The physics may use the configured ambient wind vector, but AP/Lua/Python controllers must not read that truth wind to choose a wind-plane azimuth. Use measured/estimated quantities (position, tether geometry, anemometer/winch-node telemetry, or an explicit estimator) only. Test helpers must not silently derive controller setpoints from the true `WIND` constant.
- **Do not command an explicit horizontal radial/azimuthal velocity target as part of normal flight control.** Ground/AP controllers may command tension, tether length, altitude, and body-z orientation, but they must not add a separate off-plane horizontal motion setpoint.
- **If a lateral correction is required, it must be for plane-keeping or damping only.** Any such correction must be minimal, physically justified, and must not introduce a new circling or crosswind motion objective.

#### Rotor spin direction (US helicopter convention — baked into the whole stack)

**Main rotor spins CCW viewed from above.** This is the US convention (Sikorsky / Bell / Robinson); European / Russian (Eurocopter / Kamov / Mil) typically spin CW. The whole stack — `rawes.lua` `MODE_YAW`, `mediator_torque.torque_model`, `H_TAIL_TYPE`, the GB4008 anti-rotation motor wiring — assumes US convention.

What it implies in NED with body-z DOWN:

| Quantity | Sign | Why |
|----------|------|-----|
| Rotor angular velocity vector | along **−body_z** (UP) | Right-hand rule: thumb up ⇒ CCW from above |
| `omega_spin` (scalar, in code) | **positive** | Magnitude only; the −body_z direction is implicit (see `H_spin_b = [0, 0, −I_spin·omega_spin]` in `dynamics.py`) |
| Body drift under rotor drag (no motor) | **CCW from above** | Newton 3rd: drag on rotor is CW, reaction on body is CCW |
| `gyro:z()` when body drifting unhindered | **negative** | CCW from above = −Z rotation in NED RH-rule |
| GB4008 motor reaction needed | **CW on body** | To counter the CCW drift |
| `ATC_RAT_YAW` setpoint = 0 | error = `−gyro:z()` > 0 | When body drifting CCW |
| `H_TAIL_TYPE` | **3** (DDFP CW, no sign flip) | Positive PID → positive throttle → motor on. `H_TAIL_TYPE=4` (CCW with `_servo4_out *= −1`) would clamp positive PID to 0 and let the body drift unopposed. |
| `MODE_MANUAL` Lua `err = −gyro:z()` | matches | err > 0 when body drifting CCW → PID winds up → SERVO4 PWM goes high → motor on. No sign flip needed in `MODE_MANUAL` because it writes SERVO4 directly via `SRV_Channels:set_output_pwm_chan_timeout`. |
| `torque_model.step` formula | `psi_dot = −omega_rotor + omega_motor / GEAR_RATIO` | Body drifts negative (CCW) under positive `omega_rotor` magnitude; positive motor throttle pushes `psi_dot` back toward 0 |

**Don't flip the sign of `omega_rotor` to "fix" a symptom in a test.** The whole stack assumes a positive `omega_rotor` is the CCW-from-above spin magnitude. If a test breaks, the bug is upstream (sign convention violation), not in the rotor input.

#### Swashplate geometry (H3-120 — physically canonical layout)

The bench rig's swashplate is a non-standard H3-120: **two front servos and one rear servo**. The Pixhawk sits between them; its nose points the same direction as the airframe nose (`AHRS_ORIENTATION = 0`). Servo azimuths are measured CCW from the FC's forward axis looking down:

| Servo | Azimuth | Physical position |
|---|---|---|
| **S1** | -60° | front-right |
| **S2** | +60° | front-left |
| **S3** | 180° | back (on the longitudinal axis) |

`H_SW_TYPE = 3` (H3 generic), `H_SW_H3_PHANG = 0`. The single source of truth in code is [simulation/swashplate.py:43](simulation/swashplate.py#L43) (`_AZIMUTHS_DEG = (-60.0, 60.0, 180.0)`).

ArduPilot's swash mixer derives roll/pitch factors from azimuth via [AP_MotorsHeli_Swash::add_servo_angle](C:/repos/ardupilot/libraries/AP_Motors/AP_MotorsHeli_Swash.cpp#L180):

| Servo | roll_factor = -sin(az) × 0.45 | pitch_factor = cos(az) × 0.45 |
|---|---|---|
| S1 | +0.390 | +0.225 |
| S2 | -0.390 | +0.225 |
| S3 | 0.000 | -0.450 |

Verified empirically by `run passive --trim oscillate=1` ([logs/calibrate/run_passive_*.csv](simulation/logs/calibrate/)):

| Command | S1 (front-R) | S2 (front-L) | S3 (back) |
|---|---|---|---|
| `tlon +` (nose-down disk) | ↓ | ↓ | ↑ |
| `tlat +` (roll-right disk) | ↑ | ↓ | — |
| `col +` (collective up) | ↑ | ↑ | ↑ |

**Consistency rules:**
- The four params `H_SW_H3_SV1_POS`, `H_SW_H3_SV2_POS`, `H_SW_H3_SV3_POS`, `AHRS_ORIENTATION` form a coupled set. **Never change one without checking the other three** — a 180° mismatch silently inverts all cyclic commands.
- If the FC is ever physically rotated (e.g. mounted facing backward), update `AHRS_ORIENTATION` (e.g. `4` = YAW_180) rather than swapping individual servo positions.
- If servos are physically swapped on the swashplate, update `H_SW_H3_SV{1,2,3}_POS` to match — do NOT compensate via SERVO reversal flags (works for direction but not for the geometric mixer math).
- [simulation/scripts/calibrate.py](simulation/scripts/calibrate.py)'s `_h3_forward_mix` (used by the `swash <coll%> <lon%> <lat%>` calibration command) must use the SAME azimuths (-60° / +60° / 180°). The function bypasses the heli mixer (direct DO_SET_SERVO), so it has its own copy of the geometry — keep it in sync.

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
- **Stack tests must not violate physics.** Never add artificial mechanisms just to stabilise a test. `base_k_ang` is diagnostic-only and defaults to 0.

### Other

- **Use GUIDED. Never STABILIZE.** STABILIZE holds NED roll=0, fighting the 65° tether equilibrium; yaw discontinuity at unfreeze → EKF emergency reset → crash within 4 s. GUIDED + `vehicle:set_target_angle_and_climbrate` is the flight mode. Lua computes `bz_goal` (tether-equilibrium body\_z in NED) via `bz_altitude_hold`, converts to ZYX Euler via `bz_ned_to_roll_pitch(bz_goal, ahrs:get_yaw())`, and calls `set_target_angle_and_climbrate` at 50 Hz. ArduCopter GUIDED = mode **4** (RTL = **6**). `ATC_RAT_RLL_IMAX = ATC_RAT_PIT_IMAX = ATC_RAT_YAW_IMAX = 0` to prevent angular-rate I-windup. See [design/flight_stack.md §6](design/flight_stack.md).
- **No silent defaults for physics parameters.** Raise `KeyError`/`ValueError` if a required config key is absent. Never use `dict.get("key", fallback)` or `x = x or default` for physical constants, control gains, or rotor/airfoil properties.
- **NEVER use non-ASCII characters in Python `print()`.** Windows cp1252 → `UnicodeEncodeError`. Use 7-bit ASCII only: `-`, `[PASS]`/`[FAIL]`, `sd=` for sigma.

---

## Workflow Rules

- **Do NOT consult git history** (`git log`/`diff`/`show`/`blame`) when diagnosing problems unless you first ask the user.
- **Fix telemetry/logging before diagnosing test failures.** When a simtest or stack test fails, inspect the telemetry CSV and logs first. If columns are zero/missing/wrong (e.g. `tether_m=0`, phase never changes), fix the logging bug before attempting to diagnose physics. Diagnosing from bad telemetry produces wrong conclusions.
- **SITL stack tests: see [design/sitl_testing.md](design/sitl_testing.md).** That doc owns the SITL workflow — running (`test.sh stack`), the mandatory `diagnose_sitl.py` CHECK 1/CHECK 2 post-run gate, `analyse_run.py`, the lockstep contract, and the one central kinematic-hold implementation (`_ic_trapezoid_stack` → `simulation/kinematic.py`) that every IC-start test must reuse. Read it before editing or diagnosing any SITL stack test, and keep it in sync when the SITL workflow or kinematic-hold timeline changes.
- **Keep `rawes_test_surface.lua` in sync with `rawes.lua`.** Lua unit tests access constants and functions through `_rawes_fns`, which is spliced inside `rawes.lua`'s anonymous function wrapper and so can see module-level locals only. Whenever you add a local constant or function to `rawes.lua` that tests need, add it to `_rawes_fns` in `rawes_test_surface.lua` in the same commit. Function-local variables are not accessible — hoist them to module level first.
- **`controller.py` follows `rawes.lua`.** `test_math_lua.py` cross-checks `rawes.lua` against `controller.py`; a failure there means `controller.py` diverged — fix `controller.py`.
- **One-off / diagnostic scripts go in `simulation/tests/oneoff/`, never in `tests/unit/`.** Any script run with `python -c "..."` for a gain sweep, Bode probe, plant identification, debug trace, etc. that isn't a pytest-discoverable unit test must be saved as a standalone script in `simulation/tests/oneoff/`. Reasons: (1) keeps the unit-test discovery clean — these scripts are not regression guards; (2) makes the diagnostic reproducible without scrolling chat history; (3) tools-required for the next person who hits the same problem. Prefix file names with the date or topic (e.g. `phase_sweep.py`, `bode_attitude.py`). Add a one-line header `"""<topic> — one-off diagnostic, not a unit test."""`.
- **Reusable log merge/align utilities belong in `simulation/analysis/` (or `simulation/scripts/` if they are operational), not `tests/oneoff/`.** Example shape: a CSV-producing tool that aligns `telemetry.csv` against `dataflash.BIN` with a solved time offset should live in `simulation/analysis/` and be documented like any other reusable analysis entry point.
- **Telemetry CSV columns are centralized in `simulation/telemetry_columns.py`.** `COLUMNS` is the single canonical ordered schema. When adding telemetry, add the field there first, update `TelRow` plus the relevant constructor mapping (`from_physics` / `from_tel`), and write via `write_csv()` or a `csv.DictWriter` that imports `COLUMNS`. Do not invent per-test or per-module telemetry headers.
- **When asked any question about telemetry CSV fields, inspect `simulation/telemetry_columns.py` first.** Treat it as the authoritative field list and frame annotation reference.

---

## Lua API Gotchas

| What you'd expect | What actually works |
|---|---|
| `ahrs:get_rotation_body_to_ned()` | Doesn't exist. Use `ahrs:body_to_earth(v)` / `ahrs:earth_to_body(v)` |
| `Vector3f(x, y, z)` | Constructor ignores args. Use `Vector3f()` then `:x()/:y()/:z()` setters |
| `v:normalized()` | Doesn't exist. Copy then `:normalize()` in-place |
| `vec * scalar` or `vec + vec` | `*` not overloaded; `+` may silently fail. Use component arithmetic |
| `rc:set_override(chan, pwm)` | Use `rc:get_channel(n):set_override(pwm)` (cache channel at module load) |
| ArduCopter mode numbering | GUIDED = **4**. Mode 6 is RTL |

---

## Running Tests

**Unit tests and simtests: Windows native, no Docker. Stack tests: Docker required. Never mix.**

- **`.venv`** — the one and only Windows venv for unit tests, simtests, and `calibrate.py`. Located at the **repo root** (`e:\repos\windpower\.venv`). Created/refreshed by `setup.cmd` (pure batch, no bash required); hash-gated: requirements are reinstalled only when `requirements.txt` changes. There is **no** `simulation/.venv`. (`am32config/.venv` belongs to a separate tool — the AM32 ESC configurator — and is unrelated.)
- **Docker container** — has its own Python env (never use the Windows venv inside Docker). Image built by `bash setup.sh build` (with ArduPilot) or `bash setup.sh build-lite` (without ArduPilot); container lifecycle via `bash test.sh start|stop|sync|shell|exec`.

### Rules

- **Use Bash tool directly — never `wsl.exe`. Always absolute paths.**
- **Pin Python commands to `.venv/Scripts/python.exe` for Windows-native tests and scripts.** Do not use system Python or an unactivated venv.
- **Always pass an explicit test path to `run_tests.py`** (e.g. `simulation/tests/unit` or `simulation/tests/simtests`). Running without a path lets pytest wander into `simulation/analysis/` and other non-test scripts using `argparse`, causing collection errors.
- **Scope `Grep` to source dirs (e.g. `simulation/scripts/`, `simulation/tests/`)** — `.venv/` contains hundreds of thousands of third-party files.
- **NEVER call `docker exec` directly to run stack tests. Use `bash test.sh stack`.** Each stack test always runs in its own fresh Docker container.
- **Unit/simtests run on the Windows venv** — either directly (`.venv/Scripts/python.exe ...`) or via `bash test.sh unit` / `bash test.sh simtest`, which both invoke that same venv (NOT Docker).

### Commands

**For agents: use direct pytest/run_tests entry points**

| Task | Command |
|------|---------|
| **Unit tests (agent mode)** | `.venv/Scripts/python.exe -m pytest simulation/tests/unit -m "not simtest" -q` |
| **Simtests (agent mode)** | `.venv/Scripts/python.exe simulation/run_tests.py simulation/tests/simtests -m simtest -q` |
| **Simtest (agent mode, specific)** | `.venv/Scripts/python.exe simulation/run_tests.py simulation/tests/simtests -k test_foo -s` |
| **Unit test (agent mode, specific)** | `.venv/Scripts/python.exe -m pytest simulation/tests/unit -m "not simtest" -k test_math -q` |

**Stack tests (SITL) run in Docker and have their own workflow — see [design/sitl_testing.md](design/sitl_testing.md)** (running via `test.sh stack`, the mandatory `diagnose_sitl.py` CHECK 1/CHECK 2 gate, `analyse_run.py`, and the kinematic-hold timeline). Quick entry point: `bash test.sh stack -n 1 -k test_foo`.

| Task | Command |
|------|---------|
| Unit tests (~685) | `.venv/Scripts/python.exe -m pytest simulation/tests/unit -m "not simtest" -q` |
| Simtests (~13) | `.venv/Scripts/python.exe simulation/run_tests.py simulation/tests/simtests -m simtest -q` |
| Simtest (single) | `.venv/Scripts/python.exe simulation/run_tests.py simulation/tests/simtests -k test_foo -s` |
| **Visualize result** | `visualize.cmd simulation/logs/<test_name>/telemetry.csv` |
| Scrub frames | `.venv/Scripts/python.exe simulation/viz3d/scrub.py simulation/logs/<test_name>/telemetry.csv` |
| Render to MP4/GIF | `.venv/Scripts/python.exe simulation/viz3d/render_cycle.py <csv> [--out cycle.mp4] [--speed 2]` |

**Viz note:** Launch visualization with `visualize.cmd <telemetry.csv>` rather than running `visualize_3d.py` inline. The batch file uses `start` so PyVista/VTK output stays in a separate console and does not block or flood the agent terminal. Ignore VTK/OpenGL shader errors from `visualize_3d.py` such as `vtkShaderProgram: Could not create shader object` / `vtkOpenGLPolyDataMapper: Could not set shader program`. They are local rendering/OpenGL backend failures, not simulation or telemetry failures; inspect the CSV or use non-OpenGL analysis when they occur.

**Diagnosis**

| Task | Command |
|------|---------|
| **Pumping envelope** | `.venv/Scripts/python.exe simulation/analysis/pump_envelope.py` (add `--wind 8 10 12`, `--telemetry <csv>`) |
| **Pump cycle diagnosis** | `.venv/Scripts/python.exe simulation/analysis/pump_diagnosis.py --test test_pump_cycle_unified --bucket 1` |
| **Landing diagnosis** | `.venv/Scripts/python.exe simulation/analysis/analyse_landing.py [--test test_landing_lua_sitl] [--bucket 2]` |
| **High-freq telemetry** | `RAWES_TEL_HZ=400 .venv/Scripts/python.exe simulation/run_tests.py simulation/tests/simtests -k <name> -s` (default 20 Hz) |
| **Regenerate `steady_state_starting.json`** | `.venv/Scripts/python.exe -m pytest simulation/tests/simtests/test_generate_ic.py::test_create_ic -s` — **the ONLY test that writes the file.** Run after any aero model change. |

**Docker & Infrastructure**

Stack tests run in isolated Docker containers (one per test file) and log to `simulation/logs/{test_name}/`. Full SITL workflow, log layout, and diagnosis: [design/sitl_testing.md](design/sitl_testing.md).

---

## Running calibrate.py (real hardware)

**Venv:** `.venv` at the repo root (the same venv as the tests). Use `--port` / `--baud` flags — positional args are parsed as commands.

```powershell
# Interactive REPL (SiK radio on COM7 at 57600)
& "e:\repos\windpower\.venv\Scripts\python.exe" simulation/scripts/calibrate.py --port COM7 --baud 57600

# USB direct (default 115200)
& "e:\repos\windpower\.venv\Scripts\python.exe" simulation/scripts/calibrate.py --port COM4
```

**Key REPL commands:**
| Command | Effect |
|---------|--------|
| `manual --col -8.6 --tlon 1.15 --duration 120` | Interactive manual mode (SCR_USER6=2, H_FLYBAR_MODE=1) |
| `run passive --trim tlon=1.15,col=-8.6` | Armed-but-quiet, holds trim at IC |
| `set H_FLYBAR_MODE 1` | Write param with ACK + readback verification |
| `script upload simulation/scripts/rawes.lua` | Upload Lua to /APM/scripts, restart scripting engine |
| `status` | Vehicle / battery / EKF / servos / key params snapshot |

---

## Key Design Decisions (one-liners — see references for detail)

- **Production/default flight aero:** `quasi_static` BEM (from `dynbem` external package at `C:/repos/aero`). Use it for simtests, stack-facing physics, IC replay, pumping, landing, and diagnostics unless a test/script is explicitly comparing aero models or investigating dynamic-inflow behavior. Dynamic models (`oye`, `pitt_peters`, `jit`) are opt-in only.
- **Two-loop attitude:** `compute_rate_cmd(kp)` → rate setpoint; `HeliCyclicController` (rate PIDs + 25 ms servo lag) → swashplate tilt. **Portable core** in `controller.py` maps 1:1 to Lua: `compute_bz_tether`, `slerp_body_z`, `compute_rate_cmd`, `col_min_for_altitude_rad`, `compute_bz_altitude_hold`.
- **High-tilt De Schutter:** xi=80° viable. `col_max=0.10`, `col_min_reel_in=0.079`. BEM invalid above xi≈85°. `body_z_slew_rate = 0.40 rad/s`.
- **rawes.lua modes (valid: 0, 1, 2, 3, 4):** 0=none, 1=steady, 2=manual (bench yaw PID + NVF cyclic/collective — `RAWES_TLN`/`RAWES_TLT`/`RAWES_COL`; `H_FLYBAR_MODE=1`), 3=passive (armed-but-quiet, commands the IC attitude as a GUIDED angle target via `set_target_angle_and_rate_and_throttle` — `RAWES_RIC`/`RAWES_PIC` roll/pitch + AHRS-captured yaw, zero rate FF — plus IC collective `RAWES_COL` via throttle, during kinematic release), 4=landing. Pumping has **no dedicated mode** — it runs in steady (mode 1) with the ground varying `RAWES_TEN`/`RAWES_SUB`. Modes 1/2/3/4 own Ch3. Substates via `NAMED_VALUE_FLOAT("RAWES_SUB", N)`. See [design/flight_stack.md §4](design/flight_stack.md).
- **Lua MAVLink rx queue:** `mavlink:init(20, 10)` — first arg is the per-tick rx buffer depth. `mavlink:init(1, 10)` (a common copy-paste default) drops back-to-back NAMED_VALUE_FLOAT messages — only the first survives until the next update() drains it.
- **Yaw regulation** lives in ArduPilot's `ATC_RAT_YAW` PID (`H_TAIL_TYPE=3` DDFP CW, no sign flip — matches US-convention rotor: positive yaw error from CCW body drift → positive SERVO4 throttle).  The Lua's `MODE_MANUAL` (SCR_USER6=2) bypasses this entirely and writes SERVO4 directly via `SRV_Channels:set_output_pwm_chan_timeout`; additionally it commands cyclic via `RAWES_TLN`/`RAWES_TLT` NVFs and collective via `RAWES_COL` with `H_FLYBAR_MODE=1` (RC passthrough, no rate PID). Used for bench yaw-tuning and manual swash validation. `calibrate manual` is the interactive interface; `test_lua_manual_mode_sitl` is the SITL stack test. No DShot active (`RPM1_TYPE=0`); anti-rotation motor on standard PWM, MAIN OUT 4. Current hardware: GB4008 + 80:44 gear. See [design/dshot.md](design/dshot.md), [design/flight_stack.md §4.7–§5](design/flight_stack.md).
- **GPS fusion timing:** `EK3_GPS_CHECK=0` + widened gates (`EK3_POS_I_GATE=50`, `EK3_VEL_I_GATE=50`) — required SITL-only boot params in `rawes_sitl_defaults.parm` (loaded after `rawes_common_defaults.parm`). `GPS_AUTO_CONFIG=0` is critical (prevents ArduPilot from corrupting RELPOSNED in SITL). See [design/flight_stack.md Appendix D](design/flight_stack.md).
- **Anchor in `LOCAL_POSITION_NED`:** anchor position is sent post-arm as NVFs `RAWES_ANN`/`RAWES_ANE`/`RAWES_AND`. `RAWES_AND = −initial_state["pos"][2]` (NED Z negated). MODE_STEADY will not initialise altitude hold until all three have been received.
- **Ground-to-air interface (all flight modes):** the AP receives ONLY commanded tension (`RAWES_TEN`) + target altitude (`RAWES_ALT`), plus the phase/substate (`RAWES_SUB`) for sequencing. Never actual/measured tension, never fast feedback. Commanded tension is feedforward into the orientation force balance (sets disk-axis direction + a lift-magnitude feedforward); it is NOT a tension feedback setpoint on the AP. The winch closes the only tension loop on its own load cell.
- **Pumping (Python simtest):** ground/AP split with `TensionCommand` protocol carrying the commanded tension + altitude per phase. Ground owns altitude smoothing; AP must not add a second layer. The winch drives reel speed from its own load-cell error; **`winch_target_tension = tension_ic` during reel-out (NOT `tension_out`).** See [design/simulation.md](design/simulation.md) Pumping Cycle Architecture.
- **Landing:** unified — `LandingGroundController` (10 Hz) → `LandingCommand` → `LandingApController` (400 Hz) + `WinchController`. Three phases: reel_in / descent / final_drop. Old `LandingPlanner` deleted. See [design/simulation.md](design/simulation.md) Landing Architecture.
- **IC generation targets 300 N tension.** `test_generate_ic.py::test_create_ic` runs 60 s warmup with `TensionPI` targeting 300 N. This `TensionPI` is an **offline IC-generation tool only** (mirrors the ground winch's tension loop), NOT the AP flight loop. `coll_eq_rad` is the settled collective, not a hardcoded constant; the AP warm-starts its altitude-PID collective at this value (`RAWES_COL`) in all simtests.
- **`HeliCyclicController` (25 ms servo lag) is baked into `PhysicsRunner` and always active for simtests.** `runner.step()` for Python-AP tests; `runner.step_guided()` for Lua/GUIDED tests (takes `HeliRateOutput` from `arduloop.GuidedAttitudeController`).
- **Gyroscopic phase NOT needed:** `H_SW_PHANG=0` with dynbem v0.4.0 rotor response. `base_k_ang` defaults to 0; `swashplate_phase_deg≠0` degrades steady-flight stability.
- **Torque model:** ESC speed governor with **finite peak torque** driving the gear-reflected hub inertia (`J_total = I_hub/GEAR² + I_motor`); motor speed cannot jump, so yaw-rate slew is bounded (`|d psi_dot/dt| ≤ ESC_Q_MAX/(J_total·GEAR)`). Replaces the old zero-inertia algebraic model. `equilibrium_throttle ≈ 0.485` at 28 rad/s (unchanged). `H_YAW_TRIM = −0.419`. See [design/flight_stack.md §5](design/flight_stack.md).

---

## SITL Lockstep — Key Rule

The physics worker must reply to **every** SITL servo packet without exception (skipping a reply stalls ArduPilot permanently). Full reference, plus the rest of the SITL workflow: [design/sitl_testing.md](design/sitl_testing.md) and [design/simulation.md § SITL Lockstep Protocol](design/simulation.md).

---

## Current Limits

1. Single-body hub model — no blade multibody, no flapping DOF
2. Rotor spin is a scalar ODE — gyroscopic coupling computed but `I_spin` effect is small
3. Tether: tension-only elastic — no sag, no distributed mass, no reel dynamics
4. Aero: steady-state BEM — no dynamic inflow; Coleman skewed wake handles non-uniform induction
5. Controller: rawes.lua runs at 50 Hz (GUIDED `vehicle:set_target_angle_and_climbrate`); `arduloop.GuidedAttitudeController` closes the attitude loop at 400 Hz in simtests; stack tests use real ArduPilot SITL

---

## Phase 3 Plan

See [design/history.md](design/history.md) for full decision history.

**Test progression: steady → pumping → landing.** Fix `test_lua_flight_steady_sitl` (stack) before debugging pumping or landing Lua stack tests. Do not debug `test_pumping_cycle_lua_sitl` or `test_landing_lua_sitl` (stack) until `test_lua_flight_steady_sitl` passes cleanly (horizontal deviation < 5 m, altitude stable ±2 m, yaw gap < 15 deg for ≥ 60 s).

| Milestone | Status | Gate |
|-----------|--------|------|
| M1 Wire Pumping Cycle | done | — |
| M2 Force Balance & Rotor Abstraction | done | — |
| M3 Step 1 — test_lua_flight_steady_sitl (stack) | **regressed** by aero migration; kinematic_exit clean (MODE_PASSIVE + trim cyclic + IC collective); body rate runs away from ATC PID + saturated cyclic ~250 ms after release | horizontal deviation < 5 m, no EKF yaw reset, ≥ 60 s stable |
| M3 Step 1b — test_landing.py + test_landing_lua (simtest) | failing: winch control during descent | descent slack=0, floor hit, anchor_dist < 20 m |
| M3 Step 2 — test_pumping_cycle_lua_sitl (stack, SCR_USER6=1 steady) | in dev | "RAWES steady: captured" + net_energy > 0 + peak_tension < 496 N |
| M3 Step 3 — test_landing_lua_sitl (stack, SCR_USER6=4) | in dev | "RAWES land: captured" + "final_drop" + hub alt ≤ 2.5 m |
| M3 Step 4 — rawes_params.parm (Pixhawk 6C) | not started | file exists + H_PHANG determined |
| M4 — Hardware-in-the-Loop (Pixhawk 6C) | not started | test_hil_interface.py passes + 60 s HIL log |
