# RAWES — Project Context for Claude

## Project Goal

Build an **ArduPilot flight controller model** for a Rotary Airborne Wind Energy System (RAWES) — a tethered, 4-blade autogyro kite. Wind drives autorotation; cyclic pitch control steers; tether tension during reel-out drives a ground generator. No motor drives rotation.

**Current phase:** Phase 3, Milestone 3. `rawes.lua` rewritten to use **AltitudeHoldController** (`bz_altitude_hold`): elevation rate-limited toward `RAWES_ALT` target, gravity-compensated disk tilt, replaces orbit tracking. Pumping uses **TensionPI** inside rawes.lua (mirrors Python `controller.TensionPI`). Pre-GPS: gyro feedthrough only. GPS fusion uses dual GPS (EK3_SRC1_YAW=2, RELPOSNED heading): `_el_initialized` fires on first valid position fix. **Next: validate `test_pumping_cycle` (stack), then `test_landing_stack` (stack).**

**Simtest status (12 simtests, 10 PASS):** 9 Python-only simtests pass. `test_steady_flight_lua` and `test_pump_cycle_lua` pass. `test_landing` and `test_landing_lua` failing (winch control during descent — under investigation). `test_kinematic_transition` pre-existing failure. `test_pump_cycle_unified.py` uses `TensionCommand`/`TensionApController` architecture. Landing uses unified architecture: `LandingGroundController` (10 Hz) → `LandingCommand` → `LandingApController` (400 Hz) + `WinchController` (400 Hz). Old `LandingPlanner` deleted.

**Stack test status (parallel -n 8, 9 PASS):** test_arm_minimal, test_armon, test_gps_fusion_layers, test_gust_recovery, test_pitch_roll, test_slow_rpm, test_startup, test_wobble, test_yaw_regulation. `test_lua_flight_steady` passes reliably (stable=86–110 s, 3/3). `test_pumping_cycle` and `test_landing_stack` (stack): in development.

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

### Ground Winch Motor/Generator

Separate from the GB4008. Sized to deliver 435 N at 0.40 m/s reel-out and 226 N at 0.80 m/s reel-in with ~200 W continuous.

| Parameter | Value | Derivation |
|-----------|-------|------------|
| Motor type | 400 W BLDC servo, 48 V | P = 435 N × 0.40 m/s = 174 W, add 15% margin |
| Rated speed | 3000 RPM | — |
| Rated torque | 1.27 N·m | P / ω = 400 / (3000 × 2π/60) |
| Peak torque | 3.8 N·m | 3× rated, typical BLDC |
| Gearbox | 20:1 planetary, η = 0.85 | — |
| Drum radius | 50 mm | — |
| Reel-out speed | 0.40 m/s → 1528 RPM motor | v / r / (2π/60) / gear_ratio |
| Reel-in speed | 0.80 m/s → 3056 RPM motor | near rated speed |
| T_reel_out (435 N) | drum 21.75 N·m → motor 1.28 N·m | ≈ rated torque ✓ |
| T_hard_max (496 N) | drum 24.8 N·m → motor 1.46 N·m | 115% rated → current-limit here |
| T_reel_in (226 N) | drum 11.3 N·m → motor 0.66 N·m | 52% rated, ample margin |

**Winch simulation parameters derived from this spec:**

- `T_soft_max = 470 N`, `T_hard_max = 496 N` — reel-out generator taper (matches motor 115% current-limit ceiling)
- `T_reel_in_start = 250 N` — gate: AP must reduce tension before reel-in motor engages
- `T_soft_reel_in = 350 N`, `T_hard_reel_in = 496 N` — reel-in upper taper: motor current-limits and tapers to stop as tension rises toward 496 N (same ceiling as reel-out; shared hardware limit)
- `T_soft_min = 30 N`, `T_hard_min = 10 N` — slack boost: below 30 N reel-in speed increases toward 2× nominal to take up slack; models the drum coasting freely when tether goes slack

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
| [simulation/torque_model.py](simulation/torque_model.py) | Counter-torque hub yaw model — `HubParams` (rpm_scale, gear_ratio, motor_tau), `HubState` (psi, psi_dot, omega_motor), `step()`, `equilibrium_throttle()` |
| [simulation/scripts/rawes.lua](simulation/scripts/rawes.lua) | Unified Lua controller (SCR_USER6 modes 0–8) |
| [simulation/rawes_lua_harness.py](simulation/rawes_lua_harness.py) | `RawesLua` class — runs rawes.lua in-process via lupa; shared by unit and simtests |
| [simulation/rawes_modes.py](simulation/rawes_modes.py) | Python constants mirroring rawes.lua mode/substate numbers |
| [simulation/tests/unit/README.md](simulation/tests/unit/README.md) | **Unit & simtest guide** — test catalogue, Lua harness API, SCR_USER6 encoding, IC loading, telemetry |

---

## Key Design Decisions

- **Production aero:** `SkewedWakeBEMJit` (`aero/aero_skewed_wake_jit.py`) — Numba `@njit` drop-in. 18-test equivalence suite vs. reference (atol=1e-10). `create_aero(model="jit")`. Non-JIT version is human-readable reference only. **Validity limit: xi ≲ 85° from horizontal (Coleman skewed-wake correction degenerates at chi→90°; disk horizontal = invalid).**
- **Peters-He aero:** `PetersHeBEMJit` (`aero/aero_peters_he_jit.py`) — 3-state dynamic inflow (v0, v1c, v1s). `create_aero(model="peters_he")`. Pure-numpy reference: `model="peters_he_numpy"`. **No skew-angle validity limit** — uses momentum ODE (`V_T = sqrt(v_inplane² + v_axial²)`) valid from hover through axial descent and forward flight. Required for near-vertical descent (hub above anchor, disk horizontal, xi ≳ 85°). Stateful model: serialize/restore inflow states via `aero.to_dict()` / `create_aero(model="peters_he", state_dict=d)`. Refs: Pitt & Peters 1981, Peters & He 1991.
- **Two-loop attitude:** `compute_rate_cmd(kp, kd=0)` → rate setpoint; `RatePID(kp=2/3)` → swashplate tilt.
- **Portable core** in `controller.py`: `compute_bz_tether`, `slerp_body_z`, `compute_rate_cmd`, `col_min_for_altitude_rad` — map 1:1 to Lua.
- **High-tilt De Schutter:** xi=80° viable. `col_max=0.10 rad`, `col_min_reel_in=0.079 rad`. BEM invalid above xi≈85°.
- **body_z_slew_rate** = 0.40 rad/s (`rotor.body_z_slew_rate_rad_s`). Faster → oscillation; slower → wastes reel-in time.
- **RotorDefinition YAML fields:** `swashplate_pitch_gain_rad` (measurable via flap deflection); `CD_structural` (beaupoil_2026=0.0, de_schutter_2018=0.021).
- **Canonical telemetry** (`simulation/telemetry_csv.py`): `TelRow` dataclass. `damp_alpha`=0.0 in free flight; used by `flight_log.py` / `analyse_run.py` to split kinematic vs. free-flight phases. `TelRow.heartbeat()` = mediator 1 Hz status line. **CSV files are always regenerated — never worry about backward compatibility for the CSV schema.** Altitude command chain (three columns, each owned by a different component): `gnd_alt_cmd_m` = `TensionCommand.alt_m` set by `PumpingGroundController`; `elevation_rad` = AP's internally rate-limited elevation (`TensionApController._el`); `pos_z` = actual hub altitude from physics. Comparing these three reveals who is at fault when altitude tracking fails. `flight_log.FlightLog.load(log_dir)` reads all log sources (telemetry CSV + mavlink.jsonl + mediator.log + arducopter.log) into one structure; `FlightLog.buckets(bucket_s)` returns parameterized time-window aggregates with physics, MAVLink attitude/EKF, and `FlightEvent` list per window. `validate_ekf_window(mavlink_path, t_start_s, t_end_s)` is in `analyse_run` (imported by `test_kinematic_gps.py`).
- **rawes.lua SCR_USER6 modes (valid values 0,1,5):** 0=none (passive: no RC overrides, logs every 5 s + any NV message), 1=steady (cyclic altitude hold + VZ PI collective, 50 Hz), 4=landing (reserved, not yet implemented), 5=pumping (De Schutter pumping cycle, 50 Hz). Modes 1+5 own Ch3 entirely. **Yaw regulation** is handled entirely by ArduPilot's ATC_RAT_YAW PID (ACRO_Heli, H_TAIL_TYPE=4 DDFP); rawes.lua writes no commands to SERVO9/Ch9. **RAWES_ARM:** send `NAMED_VALUE_FLOAT("RAWES_ARM", ms)` to arm the vehicle and start a disarm countdown of `ms` milliseconds; re-sending refreshes the timer; Lua disarms and sends a STATUSTEXT when the countdown expires. Works in any mode. Default state is unarmed. **Substates** (pumping: 0=hold/1=reel_out/2=transition/3=reel_in/4=transition_back) are delivered via `NAMED_VALUE_FLOAT("RAWES_SUB", N)` — never encoded in SCR_USER6. `_nv_floats` resets to `{}` on every mode change. **`RAWES_ALT`**: target altitude [m] above anchor; Lua rate-limits elevation toward `asin(RAWES_ALT/tlen)` at `SCR_USER2` rad/s. **`RAWES_TEN`**: tether tension estimate [N] for gravity compensation. Constants in `simulation/rawes_modes.py`; send via `gcs.send_named_float("RAWES_SUB", N)` or `sim.send_named_float("RAWES_SUB", N)` (unit harness).
- **GPS fusion timing:** `EK3_GPS_CHECK=0` + widened gates (`EK3_POS_I_GATE=50`, `EK3_VEL_I_GATE=50`) are required boot params (in `rawes_sitl_defaults.parm`). GPS origin sets during arm; "EKF3 IMU0 is using GPS" appears once EKF innovations settle. Without widened gates, GPS fusion fails.
- **EK3_SRC1_YAW=2** (dual-antenna GPS yaw, RELPOSNED) — `rawes_sitl_defaults.parm` boot default. Two F9P antennas 50 cm apart (±25 cm along body X) give yaw from the NED baseline vector. Yaw is known from the first GPS fix — no motion, no EKFGSF rotation needed. `delAngBiasLearned` converges with constant-zero gyro (~21 s after arm). `COMPASS_USE=0`, `COMPASS_ENABLE=0` — compass is physically useless near the GB4008 motor on hardware and SITL synthetic compasses cycle every 10 s blocking GPS fusion. `GPS_AUTO_CONFIG=0` is **critical**: prevents ArduPilot from reconfiguring the UBLOX chips over serial, which corrupts the RELPOSNED stream in SITL. See `rawes_sitl_defaults.parm` for the full parameter list.
- **rawes.lua landing (SCR_USER6=4):** captures body_z on `ahrs:healthy()`; VZ descent controller (`col_cmd = COL_CRUISE + KP_VZ*(vz_actual - VZ_LAND_SP)`, VZ_LAND_SP=0.5 m/s); `final_drop` (collective→0) triggered by ground planner sending `NAMED_VALUE_FLOAT("RAWES_SUB", 1)` (LAND_FINAL_DROP). Fixture: `acro_armed_landing_lua` (`internal_controller=False`, `kinematic_vel_ramp_s=20` so hub exits kinematic at vel=0 — eliminates linear tether jolt without internal controller).
- **rawes.lua pre-GPS stabilization:** Before `ahrs:get_relative_position_NED_origin()` returns a valid position with `tlen >= MIN_TETHER_M` (`_el_initialized == false`), run_flight() returns early after: (1) holding collective at `COL_CRUISE_FLIGHT_RAD` (-0.18 rad) to prevent tension runaway from the VZ PI controller during GPS noise, (2) feeding gyro through to Ch1/Ch2 so ACRO `desired_rate = measured_rate → rate_error = 0` → zero corrective torque → natural orbital rate preserved. On first valid GPS position fix: `_el_rad` and `_target_alt` are initialized from position, `_el_initialized = true`, and STATUSTEXT sent.
- **rawes.lua altitude hold (post-GPS):** After `_el_initialized`, each 50 Hz step: (1) rate-limit `_el_rad` toward `asin(target_alt/tlen)` at `SCR_USER2` rad/s (default 0.40), (2) compute `bz_goal = bz_altitude_hold(rel, _el_rad, _tension_n)` — mirrors Python `compute_bz_altitude_hold` exactly (tether direction + gravity-compensation tilt), (3) cyclic P loop: `err = bz_now × bz_goal` projected to body frame, `ch1/ch2 = rate_to_pwm(kp * err_bx/by)`. Collective: pumping = open-loop per substate; steady = VZ PI (vz_sp=0).
- **`acro_armed_lua_full` kinematic design:** Stationary hold (vel0=[0,0,0], linear trajectory, 80 s). Hub sits at tether equilibrium; no motion required. With dual GPS (EK3_SRC1_YAW=2), RELPOSNED gives yaw from the first fix; `delAngBiasLearned` converges at ~34 s; GPS fuses → `_el_initialized` fires → altitude hold active ~46 s before kinematic exit.
- **CRITICAL — Kinematic phase sensor consistency:** The kinematic trajectory is a purely artificial path used only to bring the EKF and GPS to a fused, healthy state before real physics begins. Tether and aero forces have no effect on the trajectory (hub follows the prescribed path regardless). **All sensors sent to SITL during kinematic must be physically consistent with the prescribed trajectory as if a magical external force holds the hub in place.** Specifically: (1) `accel_body = R.T @ (d_vel/dt - gravity)` — for a stationary hold, d_vel/dt=0 so accel_body = R.T @ [0,0,-g] (gravity in body frame). (2) `gyro_body = R.T @ omega_body` — sensor.py reports the full body angular velocity; no stripping. (3) `vel` sent directly (zero for stationary hold). Use `validate_sitl_sensors.py` to verify consistency after any kinematic change.
- **rawes.lua pumping (SCR_USER6=5):** phase driven entirely by `NAMED_VALUE_FLOAT("RAWES_SUB", N)` (0=hold, 1=reel_out, 2=transition, 3=reel_in, 4=transition_back). **Collective: TensionPI** (mirrors Python `TensionPI`; `KP_TEN=5e-4, KI_TEN=1e-4, COL_MAX_TEN=0.0`); setpoint `TEN_REEL_OUT=435 N` during hold/reel-out/transition-back, `TEN_REEL_IN=226 N` during transition/reel-in; integrator warm-starts at `COL_REEL_OUT / KI_TEN`; feedback via `RAWES_TEN`. **Cyclic: altitude hold;** planner sends `RAWES_ALT = IC_altitude` (constant — the Python simtest holds constant altitude, body_z tracks tether direction at that elevation). Planner sends `RAWES_TEN` each tick for gravity compensation. Phase state machine and winch timing owned by ground planner; `xi_reel_in_deg=None` (no disk tilt change, simtest only).
- **Anchor in `LOCAL_POSITION_NED`:** `SCR_USER5 = -initial_state["pos"][2]` (NED Z negated). Anchor at `[0, 0, -pos0[2]]` in EKF frame.
- **SCR_ENABLE bootstrap:** After EEPROM wipe, scripting only starts if `SCR_ENABLE=1` is already in EEPROM. `acro_armed_lua` fixture sets it via MAVLink post-arm (persists for future boots).
- **Python orbit-tracking (legacy, Python simtests only):** `orbit_tracked_body_z_eq` (azimuthal), `orbit_tracked_body_z_eq_3d` (3D Rodrigues). Used only by Python-only simtests. **`OrbitTracker` class is removed** — replaced by `AltitudeHoldController` in all callers including `mediator.py`. **Not used in rawes.lua** — Lua uses `bz_altitude_hold` (AltitudeHoldController logic).
- **AltitudeHoldController** (`controller.py`): primary cyclic controller for altitude-holding flight (pumping cycle, landing approach, mediator internal_controller path). Converts a target altitude setpoint to `body_z_eq` with gravity compensation. `from_pos(ic.pos, slew_rate_rad_s)` → initialized at IC elevation. `update(pos, target_alt_m, tension_n, mass_kg, dt)` → rate-limited `body_z_eq`. Uses `compute_bz_altitude_hold` internally (stateless primitive: pos + target_el_rad + tension + mass → body_z_eq).
- **TensionPI runs at 400 Hz (physics loop); DeschutterPlanner runs at 10 Hz.** Planner provides `tension_setpoint` and `col_min_rad` to TensionPI at 10 Hz; TensionPI adjusts collective every physics step. `planner.step()` returns `col_min_rad` key for this purpose. Correct pattern: `if i % planner_every == 0: pump_cmd = planner.step(state, DT_PLANNER); tension_pi.setpoint = ...; tension_pi.coll_min = pump_cmd["col_min_rad"]`. Then every step: `col = tension_pi.update(tension_now, DT)`. **Note:** TensionPI is used by Lua stack tests (`test_pump_cycle_lua`) and the Python `test_pump_cycle.py`. The newer `test_pump_cycle_unified.py` uses ThrustCommand instead — see below.
- **TensionCommand protocol (Python simtest pumping):** `pumping_planner.PumpingGroundController` emits `TensionCommand(tension_setpoint_n, tension_measured_n, alt_m, phase)` at 10 Hz. Ground reads the load cell and packs both the setpoint and measurement so the AP's `TensionPI` can close the loop using ground-transmitted feedback. `alt_m` is smoothly ramped at phase boundaries (using `hub_alt_m` telemetry received from the kite at 10 Hz): ramps UP over `t_transition` seconds when entering "transition", ramps DOWN over `t_transition` seconds at the start of the next "reel-out". `TensionApController` (AP side, 400 Hz) caches its own `pos_ned` from `step()` and validates each received command via `BadEventLog`: `ap_impossible_alt` (alt_m > tether_length), `ap_unreachable_alt` (elevation gap > `slew_rate × FEASIBILITY_WINDOW_S = 1 s`). Blame rule: `ap_*` events → ground planner sent unreachable commands; slack/tension_spike without `ap_*` → AP tracking failure. `WinchController` is tension-controlled (no `T_reel_in_start` gate in unified test). Telemetry: `gnd_alt_cmd_m` (ground-commanded alt), `elevation_rad` (AP's rate-limited internal target), `pos_z` (actual hub).
- **IC generation targets 300 N tension.** `test_generate_ic.py::test_create_ic` runs 60 s warmup with TensionPI targeting 300 N (midway between TENSION_IN=226 N and TENSION_OUT=435 N). `coll_eq_rad` in the IC is the TensionPI-settled collective at that tension — not a hardcoded constant. TensionPI warm-starts at this collective in all simtests.
- **PhysicsCore** (`simulation/physics_core.py`): shared 400 Hz physics integration used by both `mediator.py` and simtests. Owns `RigidBodyDynamics`, `create_aero`, `TetherModel`, `AcroController(use_servo=True)`, spin ODE, angular damping (`base_k_ang` + `k_yaw`), `KinematicStartup`, `lock_orientation`. `step(dt, col, tilt_lon, tilt_lat)` for raw-tilt callers (mediator/Lua path); `step_acro(dt, col, body_z_eq)` for AcroController path (Python simtests). **PhysicsRunner** (`tests/unit/simtest_runner.py`): thin testing wrapper around `PhysicsCore`. `step(dt, col, body_z_eq)` → `core.step_acro()`; `step_raw(dt, col, tilt_lon, tilt_lat)` → `core.step()`. `for_warmup(rotor, pos, R0, rest_length, coll_eq_rad, omega_spin, wind)` classmethod for IC generation.
- **WinchController design — `winch_target_tension = tension_ic` during reel-out (critical).** The generator's load point is `tension_ic` (300 N). The AP drives tension up to `tension_out` (435 N), giving cruise speed `kp * (435 − 300) = 0.675 m/s → capped at v_max_out = 0.40 m/s`. If you mistakenly set `winch_target_tension = tension_out` (435 N) during reel-out, the AP tension never significantly exceeds the target so `v_cruise ≈ 0` and the winch barely moves. During reel-in, `winch_target_tension = tension_in` (226 N): motor boosts when slack (T < 226 N) and backs off when kite resists (T > 226 N). Parameters: `kp = 0.005 (m/s)/N`, `v_max_out = 0.40 m/s`, `v_max_in = 0.80 m/s`, `accel_limit_ms2 = 0.5 m/s²`. Tested in `test_winch_tension_control.py` (23 tests: reel-out, reel-in, motion profile, bounds, landing).
- **Ground owns altitude smoothing; AP must not add a second layer.** The AP already rate-limits elevation at `slew_rate_rad_s = 0.40 rad/s` — that is its smoothing. Adding a second smoothing layer in the AP would create two competing integrators. Instead, `PumpingGroundController` smoothly ramps `alt_m` across every phase boundary using `hub_alt_m` telemetry it receives at 10 Hz: ramp up over `t_transition` seconds entering "transition", ramp down over `t_transition` seconds at the start of each subsequent "reel-out". Sudden `alt_m` jumps are ground-controller bugs, not AP limitations — detected by `ap_unreachable_alt` events (`FEASIBILITY_WINDOW_S = 1.0 s`: if the elevation gap > `slew_rate × 1 s`, the command is flagged as unreachable).
- **WinchNode** (`winch_node.py`): mediator calls `update_sensors(tension, wind_world)` (physics only); planner calls `get_telemetry()` + `receive_command()`. Wind seed from `Anemometer.measure()` at 3 m height.
- **Vertical landing — unified architecture:** `LandingGroundController` (10 Hz) emits `LandingCommand`; `LandingApController` (400 Hz) tracks it; `WinchController` (400 Hz) tension-controlled. Three phases: `reel_in` (body_z slerps xi~30°→80°, winch holds at IC length, VZ PI vz_sp=0), `descent` (body_z fixed, winch tension target=180 N so kp*(180−natural_T)≈v_land, VZ PI vz_sp=0.5 m/s), `final_drop` (collective=0). `LandingPlanner` deleted — all landing logic now in `LandingGroundController` + `LandingApController`. Lua landing (mode=4): `RAWES_SUB=LAND_FINAL_DROP` (value 1) sent by ground when `cmd.phase=="final_drop"`. Use `analyse_landing.py` to diagnose descent winch/tension/VZ behaviour.
- **Gyroscopic phase NOT needed:** H_SW_PHANG=0. BASE_K_ANG=50 N·m·s/rad → τ≈0.08 s (damped before one orbit). `swashplate_phase_deg≠0` degrades orbit stability.
- **`AcroController use_servo=True`** required for De Schutter simtests (25 ms servo lag). `test_steady_flight` and pure-orbit tests do not need it.
- **Power-law wind tension:** tension_out=250 N (not 200 N) at hub altitude ~15 m (10.6 m/s wind). `col_min_reel_in` must use actual wind speed at hub altitude.
- **De Schutter aero:** `C_{D,T}=0.021` structural drag; induction bootstrap uses `abs(T)` not `max(T, 0.01)`. See `simulation/aero/deschutter.md`.
- **Torque model — kinematic with first-order motor lag:** Motor shaft speed tracks the PWM command with a first-order lag (`MOTOR_TAU=0.02 s`): `d(omega_motor)/dt = (throttle × RPM_SCALE − omega_motor) / MOTOR_TAU`. Gear coupling is instantaneous: `psi_dot = omega_rotor − omega_motor / GEAR_RATIO`. The ESC holds RPM proportional to PWM at steady state; bearing/swashplate drag only affects power draw. `RPM_SCALE=105 rad/s` (GB4008 66KV × 15.2V), `GEAR_RATIO=80/44≈1.818`, `MOTOR_TAU=0.02 s`. Equilibrium: `throttle_eq = omega_rotor × GEAR_RATIO / RPM_SCALE ≈ 0.485` at 28 rad/s. H_YAW_TRIM = −(throttle_eq − SPIN_MIN)/(SPIN_MAX − SPIN_MIN) = −0.419. `HubState` has `psi`, `psi_dot`, `omega_motor`. `HubParams` has `rpm_scale`, `gear_ratio`, `motor_tau` — no `I_hub`, no `tau_stall`, no `k_yaw`.

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
├── aero/                SkewedWakeBEMJit (production, xi<85°), SkewedWakeBEM (reference), DeSchutterAero,
│                        PetersHeBEMJit (model="peters_he", all skew angles), PetersHeBEM (numpy ref)
├── tether.py            Tension-only elastic tether (Dyneema SK75)
├── swashplate.py        H3-120 inverse mixing, cyclic blade pitch
├── frames.py            build_orb_frame(), T_ENU_NED (legacy external-data utility only)
├── sensor.py            PhysicalSensor — honest R_orb sensors, NED throughout
├── sitl_interface.py    ArduPilot SITL UDP binary protocol
├── physics_core.py      PhysicsCore — shared 400 Hz physics (dynamics, aero, tether, spin ODE,
│                        angular damping, KinematicStartup, lock_orientation). Used by both
│                        mediator.py and PhysicsRunner (simtests).
├── controller.py        compute_swashplate_from_state, compute_rc_from_attitude, RatePID,
│                        portable core (compute_bz_tether/slerp_body_z/compute_rate_cmd/
│                        col_min_for_altitude_rad), orbit_tracked_body_z_eq,
│                        orbit_tracked_body_z_eq_3d,
│                        compute_bz_altitude_hold, AltitudeHoldController,
│                        TensionPI (collective PI at 400 Hz)
├── mediator.py          SITL co-simulation loop — thin wrapper around PhysicsCore
├── mediator_torque.py   Standalone torque SITL mediator (RPM profiles, hub yaw kinematics)
├── torque_model.py      Hub yaw model (kinematic + motor lag) — HubParams (rpm_scale, gear_ratio, motor_tau), HubState (psi, psi_dot, omega_motor), step(), equilibrium_throttle()
├── kinematic.py         KinematicStartup — hub trajectory during EKF init phase
├── winch_node.py        WinchNode + Anemometer (physics/planner protocol boundary)
├── gcs.py               MAVLink GCS client (arm, mode, RC override, params)
├── pumping_planner.py   TensionCommand dataclass + PumpingGroundController (10 Hz phase schedule).
│                        step(t_sim, tension_measured_n, rest_length, hub_alt_m) → TensionCommand.
│                        alt_m is smoothly ramped at every phase boundary using hub_alt_m telemetry:
│                        up over t_transition on reel-out→transition, down at next reel-out start.
│                        winch_target_length / winch_target_tension properties for WinchController.
├── ap_controller.py     TensionApController (400 Hz AP side): TensionPI collective + rate-limited
│                        elevation hold. receive_command(cmd, dt) validates alt_m against cached
│                        pos_ned (updated each step): ap_impossible_alt (alt > tether_length),
│                        ap_unreachable_alt (elevation gap > slew_rate × 1 s). Pass BadEventLog
│                        to constructor to capture these; use them to distinguish ground vs. AP fault.
├── winch.py             WinchController — tension-controlled motion profile. set_target(length_m,
│                        tension_n) at 10 Hz; step(tension_measured, dt) at 400 Hz. Cruise speed
│                        proportional to tension error; trapezoidal accel/decel profile.
├── telemetry_csv.py     Canonical CSV schema (TelRow, COLUMNS, heartbeat). Altitude command chain:
│                        gnd_alt_cmd_m (ground cmd) / elevation_rad (AP internal) / pos_z (actual).
│                        TelRow.from_physics(runner, step_result, col, wind, **kwargs) is the single
│                        telemetry factory for all simtests — extracts state from PhysicsRunner +
│                        step result dict; optional kwargs add ground-side columns.
├── simtest_log.py       SimtestLog (per-test log dir + human-readable summary), BadEventLog (slack/
│                        tension_spike/floor_hit event tracking with phase tagging).
├── analysis/
│   ├── flight_log.py    Unified data loader — FlightLog.load(log_dir), FlightLog.buckets(bucket_s),
│   │                    FlightEvent, Bucket; reads all log sources into one structure
│   └── analyse_run.py   Post-run report: print_flight_report(log_dir, bucket_s=5.0),
│                        compute_steady_metrics, validate_ekf_window; CLI: --bucket S
├── viz3d/
│   ├── visualize_3d.py      **Default viz tool** — interactive 3D playback of any telemetry.csv.
│   │                        Space=play/pause, +/-=speed, left/right=step, mouse=orbit/pan/zoom.
│   │                        CLI: `python simulation/viz3d/visualize_3d.py simulation/logs/<test>/telemetry.csv`
│   ├── scrub.py             Interactive frame scrubber (large slider, no auto-play required).
│   │                        CLI: `python simulation/viz3d/scrub.py simulation/logs/<test>/telemetry.csv`
│   ├── render_cycle.py      Off-screen render to MP4 or GIF (no display needed).
│   │                        CLI: `python simulation/viz3d/render_cycle.py telemetry.csv [--out cycle.mp4] [--speed 2]`
│   ├── telemetry.py         TelemetryFrame dataclass + CSVSource / LiveQueueSource protocol
│   ├── visualize_torque.py  Torque telemetry 3-panel replay (psi_dot, motor throttle, PWM)
│   └── torque_telemetry.py  TorqueTelemetryFrame dataclass for torque replay
└── tests/
    ├── unit/            Windows native, no Docker (~480 fast unit tests; no simtests)
    │   └── README.md    Unit & simtest reference guide (Lua harness API, IC loading, telemetry)
    ├── simtests/        Windows native, no Docker (~11 full physics simulation tests; marker: simtest)
    │   ├── conftest.py      simtest marker registration; 600 s auto-timeout (from simulation/pytest.ini)
    │   ├── simtest_runner.py    PhysicsRunner — thin wrapper around PhysicsCore for simtests.
    │   │                        step(dt, col, body_z_eq) → core.step_acro() [Python-controlled];
    │   │                        step_raw(dt, col, tilt_lon, tilt_lat) → core.step() [Lua-controlled].
    │   │                        Caller owns: PumpingGroundController/TensionApController/WinchController.
    │   └── simtest_ic.py        load_ic() — loads steady_state_starting.json
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

**Data flow (400 Hz):** SITL servo PWM → swashplate mix → **PhysicsCore** (aero + tether + RK4 + spin ODE) → sensor packet → SITL. `mediator.py` is a thin wrapper; `PhysicsCore` (`physics_core.py`) owns the integration loop.

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
- **`.venv`** — Windows venv for all unit tests and simtests. Packages: numpy, scipy, numba, lupa, pyvista, pymavlink, pytest, etc. `run_tests.py` auto-installs when `requirements.txt` changes (SHA-256 hash stamp). Manual update: `.venv/Scripts/pip install -r simulation/requirements.txt`.
- **Docker container** — the container has its own Python env (never use the Windows venv inside Docker). Managed by `dev.sh build`.

**CRITICAL:** Use Bash tool directly — do NOT use `wsl.exe`. Always use absolute paths.

**CRITICAL:** NEVER call `docker exec` directly to run stack tests. Use `bash simulation/dev.sh test-stack [...]`. If stuck: `bash simulation/dev.sh stop && bash simulation/dev.sh start`.

**Stack test isolation:** Each stack test always runs in its own fresh Docker container — clean EEPROM, no stale processes, no port conflicts between tests.

**CRITICAL:** Unit/simtests run via the Windows venv directly — NOT via `dev.sh test-unit` (which routes to Docker and fails because `tests/unit` is excluded from the container sync).

| Task | Command |
|------|---------|
| Unit tests (~480) | `.venv/Scripts/python.exe -m pytest simulation/tests/unit -m "not simtest" -q` |
| Simtests (~11) | `.venv/Scripts/python.exe -m pytest simulation/tests/simtests -m simtest -q` |
| Stack test (single) | `bash simulation/dev.sh test-stack -n 1 -k test_foo` |
| Stack test (full suite) | `bash simulation/dev.sh test-stack -n 8` |
| **Post-failure analysis** | `.venv/Scripts/python.exe simulation/analysis/analyse_run.py <test_name>` |
| Post-failure (coarser view) | `.venv/Scripts/python.exe simulation/analysis/analyse_run.py <test_name> --bucket 10` |
| **Visualize any test result** | `.venv/Scripts/python.exe simulation/viz3d/visualize_3d.py simulation/logs/<test_name>/telemetry.csv` — **default tool for all simtest and stack test telemetry** |
| **Pumping cycle envelope** | `.venv/Scripts/python.exe simulation/analysis/pump_envelope.py` — **go-to tool for any pumping cycle parameter analysis**: tension setpoints, net energy vs TENSION_IN sweep, reel-in tilt cost/benefit, wind speed sensitivity. Use `--wind 8 10 12` to sweep wind speeds. Add `--telemetry simulation/logs/test_pump_cycle/telemetry.csv` to compare an actual run against the optimal envelope: per-cycle tension tracking, TensionPI saturation %, altitude hold bias, energy gap to optimal. |
| **Landing diagnosis** | `.venv/Scripts/python.exe simulation/analysis/analyse_landing.py` — per-bucket table of alt/vz/winch/tension/collective; phase timeline; slack event list with before/after tension and winch speed; tension spike list; descent summary. Use `--test test_landing_lua` or `--bucket 2` for finer resolution. |
| **Pump cycle diagnosis** | `.venv/Scripts/python.exe simulation/analysis/pump_diagnosis.py` — controller decision audit: altitude tracking, TensionPI feasibility, slack/spike flags per bucket. |
| Scrub to specific frame | `.venv/Scripts/python.exe simulation/viz3d/scrub.py simulation/logs/<test_name>/telemetry.csv` |
| Render to MP4/GIF | `.venv/Scripts/python.exe simulation/viz3d/render_cycle.py simulation/logs/<test_name>/telemetry.csv [--out cycle.mp4] [--speed 2]` |
| **Regenerate `steady_state_starting.json`** | **`.venv/Scripts/python.exe -m pytest simulation/tests/simtests/test_generate_ic.py::test_create_ic -s`** — **CRITICAL: this is the ONLY test that writes the file.** `test_steady_flight.py` reads it but never writes it. Run after any aero model change. Used by all simtests and stack tests as initial conditions. |
| Container start/stop | `bash simulation/dev.sh start` / `bash simulation/dev.sh stop` |
| Docker build | `bash simulation/dev.sh build` (~30–60 min; use `run_in_background=true`, no trailing `&`) |
| Run inside container | `bash simulation/dev.sh exec 'python3 /rawes/simulation/...'` |

**Run `analyse_run.py` first after any stack test failure.** It loads all log sources (telemetry CSV, mavlink.jsonl, mediator.log, arducopter.log) into a unified `FlightLog` and prints a single bucketed report covering physics, EKF/GPS events, attitude tracking, and sensor consistency. Use `--bucket 10` for a high-level overview, `--bucket 1` for frame-level detail. `analyse_mavlink.py` no longer exists — everything is in `analyse_run.py`.

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

GB4008 wired to **MAIN OUT 4 (output 4, IOMCU)**. Motor is controlled via **standard PWM** (800–2000 µs). ArduPilot controls yaw via `H_TAIL_TYPE=4` (DDFP CCW): `ATC_RAT_YAW` PID output → `SERVO4`. Lua writes no commands to the motor. No DShot or BLHeli subsystem is active (`RPM1_TYPE=0`). DShot (REVVitRC AM32 supports it) is a future option — see [hardware/dshot.md](hardware/dshot.md) for the enable procedure.

**H_TAIL_TYPE enum** (values used in this project): `0`=servo (bidirectional, SERVO4_TRIM=1500 µs, no sign flip), `3`=DDFP CW (positive PID → more throttle — wrong for GB4008), `4`=DDFP CCW (sign flip: negative PID → more throttle — correct; CW hub drift produces negative yaw-error → negative PID → CCW sign flip → positive throttle → GB4008 counters drift). Full table in [system/stack.md §4.3](system/stack.md) and [hardware/dshot.md](hardware/dshot.md).

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
