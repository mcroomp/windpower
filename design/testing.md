# Unit & Simtest Reference Guide

Tests are split across three directories:

| Directory | Host | Marker |
|-----------|------|--------|
| `tests/unit/` | Windows-native | *(none)* |
| `tests/simtests/` | Windows-native | `simtest` |
| `tests/sitl/` | Docker (Linux) | *(none)* |

---

## Running Tests

```bash
# Fast unit tests only (~685)
.venv/Scripts/python.exe -m pytest simulation/tests/unit -m "not simtest" -q

# Simtests only (~13)
.venv/Scripts/python.exe -m pytest simulation/tests/simtests -m simtest -q

# Both together
.venv/Scripts/python.exe -m pytest simulation/tests/unit simulation/tests/simtests -q

# Single test
.venv/Scripts/python.exe -m pytest simulation/tests/simtests/test_steady_flight.py -q

# Regenerate steady-state IC (required after any aero model change)
.venv/Scripts/python.exe -m pytest simulation/tests/simtests/test_generate_ic.py::test_create_ic -s
```

**CRITICAL:** Unit tests run via the Windows venv (`.venv/Scripts/python.exe -m pytest ...`) — never inside Docker, which excludes `tests/unit`.

---

## Test Markers

| Marker | Meaning |
|--------|---------|
| *(none)* | Fast unit test, no physics loop |
| `simtest` | Full time-domain physics loop — seconds of compute (auto-timeout 600 s) |

The `simtest` timeout is set globally in `simulation/pytest.ini`.

---

## Unit Tests (`tests/unit/`)

### Physics & Aero

| File | What it tests |
|------|--------------|
| `test_aero_trajectory_points.py` | BEM aero at specific trajectory sample points |
| `test_deschutter_equations.py` | De Schutter Eq. 25–31 formula correctness |
| `test_deschutter_validation.py` | Numerical validation vs. reference values |
| `test_skewed_wake_jit.py` | SkewedWakeBEMJit vs. reference (18 equivalence cases, atol=1e-10) |
| `test_force_balance.py` | Static force balance at equilibrium |
| `test_physical_validation.py` | Physical consistency checks (mass, inertia, geometry) |
| `test_swashplate.py` | H3-120 inverse mixing, cyclic blade pitch |
| `test_swashplate_servo_model.py` | SwashplateServoModel: slew-limiting, saturation, reset, properties |
| `test_swashplate_servo_directions.py` | **Servo output directions for cyclic commands** — catches parameter order bugs |
| `test_swashplate_aero.py` | Swashplate + aero integrated checks |
| `test_swashplate_aero_glauert.py` | Glauert correction in swashplate+aero path |
| `test_tether_stability.py` | Tether elastic model — tension, slack, snap |
| `test_rotor_definition.py` | RotorDefinition YAML loading and validation |

### Controller & Sensor

| File | What it tests |
|------|--------------|
| `test_controller.py` | `compute_bz_tether`, `slerp_body_z`, `compute_rate_cmd`, `RatePID`, `AltitudeHoldController`, `compute_bz_altitude_hold` |
| `test_sensor.py` | `PhysicalSensor` output consistency (gyro, accel, body_to_earth) |
| `test_startup_trajectory.py` | KinematicStartup orbit trajectory validation |
| `test_ap_controller.py` | `TensionApController` — bad-event detection, elevation clamping |
| `test_winch_tension_control.py` | `WinchController` — reel-out, reel-in, motion profile, bounds, landing (23 tests) |

### Mediator & Protocol

| File | What it tests |
|------|--------------|
| `test_interfaces.py` | `sitl_interface.py` binary protocol encoding |
| `test_planner_protocol.py` | `WinchNode` planner/physics protocol boundary |
| `test_torque_model.py` | `HubParams` / `HubState` motor lag ODE |

### Lua Controller Tests

| File | Non-Lua reference | What it tests |
|------|------------------|--------------|
| `test_math_lua.py` | — | Lua math: `bz_altitude_hold`, `cyclic_error_body`, `output_rate_limit`, `rate_to_pwm`, constants |
| `test_yaw_lua.py` | — | Yaw-trim: PI, dead zone, watchdog, hard-stop, closed-loop equilibrium |
| `test_armon_lua.py` | — | `RAWES_ARM` countdown and disarm logic |

---

## Simtests (`tests/simtests/`)

All simtests start from the same `steady_state_starting.json` IC (written by `test_generate_ic.py::test_create_ic`).

### Physics Scenarios

| Python simtest | Lua simtest | Scenario |
|----------------|-------------|----------|
| `test_steady_flight.py` | `test_steady_flight_lua.py` | 10 s orbit from IC — drift, tether, axle angle |
| `test_pump_cycle_unified.py` | `test_pump_cycle_lua.py` | N De Schutter cycles — `PumpingGroundController` / `TensionApController` |
| `test_landing.py` | `test_landing_lua.py` | reel-in only (xi~30° → ~80°) then VZ descent + final drop |

IC generation:

| File | What it tests |
|------|--------------|
| `test_generate_ic.py` | **Writes** `steady_state_starting.json`; verifies the IC is stable in 30 s flight |

Supporting simtests:

| File | What it tests |
|------|--------------|
| `test_kinematic_transition.py` | Kinematic → free-flight hand-off (pos, vel, R continuity) |
| `test_sensor_closed_loop.py` | Sensor feeding EKF-equivalent closed loop |

### Simtest Support Machinery

| File | Purpose |
|------|---------|
| `simtest_ic.py` | `load_ic()` — loads `steady_state_starting.json` |
| `simtest_runner.py` | `PhysicsRunner` — thin wrapper around `PhysicsCore` for simtests |
| `conftest.py` | `simtest` marker registration; 600 s auto-timeout |

---

## SITL Stack Tests (`tests/sitl/`)

Full ArduPilot SITL tests running in Docker. Require `bash test.sh stack`.
Never mix with Windows-native unit/simtests.

### Torque Stack Tests (`tests/sitl/torque/`)

Exercises the counter-torque motor stack (stationary hub, rotor spinning at constant RPM).
No GPS — arming uses `RAWES_ARM` with GPS-independent stack setup.

| File | Fixture | What it tests |
|------|---------|---------------|
| `test_yaw_regulation_sitl.py` | `torque_armed` | Lua yaw trim observer (rawes.lua) holds psi_dot < 5 deg/s via H_YAW_TRIM |
| `test_wobble_sitl.py` | `torque_armed_profile` | Canonical fixed-RPM wobble disturbance stays bounded |
| `test_slow_rpm_sitl.py` | `torque_armed_profile` | Canonical slow varying-RPM disturbance stays bounded |

**IC torque rig notes (`torque_armed` / `profile="ic"`):**
- `FS_CRASH_CHECK=0` is set in `_IC_TORQUE_YAW_PARAMS`. Without it, ArduPilot's crash
  detector (AngErr > 30 deg + low accel) disarms the vehicle during the observer's
  initial convergence ramp, before H_YAW_TRIM reaches u\_eq.
- The observer resets `_yaw_ff_trim=0` on PASSIVE entry; the first 15 s kinematic hold
  keeps psi\_dot=0 so the trim stays near 0; after hub release the trim ramps to u\_eq
  (~0.485) in ~1 s.  SETTLE\_S=75 s gives ample convergence time.

**Torque arming note:** torque stack setup avoids GPS-dependent guided checks and arms through the
dedicated torque sequence. This keeps no-GPS torque tests deterministic.

### Flight Stack Tests (`tests/sitl/flight/`)

| File | Fixture | What it tests |
|------|---------|--------------|
| `test_kinematic_gps_sitl.py` | guided/GPS fixture | Canonical kinematic hold + dual-GPS fusion + clean EKF window |
| `test_lua_flight_ic_passive_sitl.py` | `guided_nogps_armed_lua_full` | MODE_PASSIVE holds the seeded IC operating point after kinematic exit |
| `test_lua_flight_steady_sitl.py` | `guided_nogps_armed_lua_full` | Canonical passive->steady handoff + sustained steady flight |
| `test_pumping_cycle_sitl.py` | `guided_nogps_armed_pumping_lua` | Canonical pumping stack: planner + winch + Lua steady control |

---

## Lua Test Infrastructure

The Lua tests run `rawes.lua` **in-process** using [lupa](https://github.com/scoder/lupa) (Lua 5.4
embedded in Python). No SITL, no Docker, no real sleeping.

### Key Files

| File | Location | Purpose |
|------|----------|---------|
| `rawes_lua_harness.py` | `simulation/` root | `RawesLua` class — Python interface to rawes.lua |
| `mock_ardupilot.lua` | `simulation/` root | ArduPilot API stub (AHRS, RC, SRV_Channels, param, gcs, arming, vehicle) |
| `rawes_test_surface.lua` | `simulation/scripts/` | Test surface spliced into rawes.lua — exposes internals via `_rawes_fns` |
| `rawes_modes.py` | `simulation/` root | Central Python constants mirroring rawes.lua mode/substate numbers |

`rawes_lua_harness.py` lives in `simulation/` (not in a test subdirectory) because it is imported
by both `tests/unit/` Lua tests and `tests/simtests/` Lua simtests.

### How the Harness Works

```
rawes.lua
  ← splice rawes_test_surface.lua at -- @@UNIT_TEST_HOOK
  ← wrap in anonymous function (locals stay encapsulated)
  ← execute after mock_ardupilot.lua (global stubs in scope)

_rawes_update  = the update() return value (callable each tick)
_rawes_fns     = table of exposed internal functions (constants + helpers)
_mock          = global state table (inputs/outputs bridged to Python)
```

1. **`mock_ardupilot.lua`** defines all ArduPilot globals rawes.lua expects:
   `Vector3f`, `millis()`, `ahrs`, `rc`, `SRV_Channels`, `param`, `gcs`, `arming`, `vehicle`.
2. **`rawes_test_surface.lua`** is spliced in at `-- @@UNIT_TEST_HOOK` so it has access to all
   rawes.lua `local` variables and functions. It builds `_rawes_fns` with wrapped exports.
3. **`RawesLua`** wraps the Lua runtime. Python writes sensor state into `_mock`, calls
  `_rawes_update()`, and reads channel outputs from `_mock.ch_out[n]`.

### RawesLua API

```python
from rawes_lua_harness import RawesLua
from rawes_modes import MODE_STEADY, PUMP_HOLD

sim = RawesLua(mode=MODE_STEADY)  # RAWES_MODE = 1 (pumping schedule runs in steady)
sim.armed        = True
sim.healthy      = True
sim.vehicle_mode = 1           # RATE-MODE = 1
sim.pos_ned      = [50, 0, -14]  # GPS fused (None = not yet fused)
sim.vel_ned      = [0.0, 0.96, 0.0]
sim.R            = build_orb_frame(body_z)
sim.gyro         = [0.0, 0.0, 0.0]

# Advance time (no real sleep)
sim.run(2.0)       # 2 sim-seconds at 100 Hz
sim.tick()         # single 10 ms tick

# Read outputs
ch1 = sim.ch_out[1]   # roll PWM (int or None)
ch3 = sim.ch_out[3]   # collective PWM
sim.srv_pwm(94)        # GB4008 motor PWM (func=94)

# GCS messages
sim.messages           # [(level, text), ...]
sim.has_message("GPS") # bool

# Ground planner: keep the vehicle in steady mode and send the substate via NVF
sim.send_named_float("RAWES_SUB", PUMP_REEL_OUT)  # telemetry/diagnostics only

# Access Lua internals (via rawes_test_surface)
sim.fns.T_TRANSITION           # constant
bz = sim.fns.disk_normal_ned() # Lua Vector3f
sim.vec_to_list(bz)            # -> [x, y, z]
```

### `_mock` Layout

| Field | Direction | Type | Notes |
|-------|-----------|------|-------|
| `armed` | in | bool | arming state |
| `mode` | in | int | vehicle flight mode (1 = RATE-MODE) |
| `healthy` | in | bool | AHRS health |
| `millis_val` | in | int | fake milliseconds |
| `gyro` | in | {x,y,z} | body-frame gyro rad/s |
| `pos_ned` | in | {x,y,z} or nil | GPS position; nil = not fused |
| `vel_ned` | in | {x,y,z} | NED velocity m/s |
| `R` | in | flat 1..9 | body-to-NED rotation, row-major |
| `params` | in/out | table | ArduPilot params (ATC_*, RAWES_*, etc.) |
| `ch_out[n]` | out | int or nil | RC channel override PWM |
| `srv_out[func]` | out | int or nil | SRV_Channels PWM by function |
| `gcs_msgs` | out | array | {level, msg} log |

---

## RAWES_MODE / Substate Encoding

`RAWES_MODE` is a **script-generated parameter** (registered by rawes.lua at load time; visible as `RAWES_MODE` in GCS). Substate is delivered separately via
`NAMED_VALUE_FLOAT("RAWES_SUB", N)` — never encoded in the mode param.

Constants are in `simulation/rawes_modes.py` (Python) and as locals in `rawes.lua` (Lua).

```python
from rawes_modes import (
    MODE_NONE, MODE_STEADY,
    MODE_LANDING,
    LAND_DESCEND, LAND_FINAL_DROP,
    PUMP_HOLD, PUMP_REEL_OUT, PUMP_TRANSITION, PUMP_REEL_IN, PUMP_TRANSITION_BACK,
)
```

| Mode | RAWES_MODE | Substates (RAWES_SUB) | Notes |
|------|-----------|----------------------|-------|
| `MODE_NONE` | 0 | — | Logging only; RAWES_ARM still handled |
| `MODE_STEADY` | 1 | — | Altitude hold + tether tracking |
| `MODE_PASSIVE` | 3 | — | Armed-but-quiet: commands IC attitude angle (RAWES_RIC/PIC) + IC thrust (RAWES_THR via throttle) as a GUIDED angle target during kinematic |
| `MODE_LANDING` | 4 | 0=DESCEND, 1=FINAL_DROP | |

Pumping has **no dedicated mode**: it runs in `MODE_STEADY` (RAWES_MODE=1) while the ground
varies `RAWES_TEN` and sends the `RAWES_SUB` pumping substate (0=HOLD, 1=REEL_OUT,
2=TRANSITION, 3=REEL_IN, 4=TRANSITION_BACK) for telemetry/diagnostics.

**Sending substates in simtests** (via `RawesLua.send_named_float`):

```python
sim = RawesLua(mode=MODE_STEADY)          # RAWES_MODE = 1 (pumping schedule)

# Only send on transitions — Lua drains the inbox each update tick
prev_sub = None
for i in range(max_steps):
    sub_now = _compute_substate(t_sim)
    if sub_now != prev_sub:
        sim.send_named_float("RAWES_SUB", sub_now)
        prev_sub = sub_now
    sim._update_fn()
```

**Sending substates in stack tests** (via `gcs.send_named_float`):

```python
gcs.send_named_float("RAWES_SUB", LAND_FINAL_DROP)
```

**`_nv_floats` reset:** Lua resets its internal `_nv_floats` table to `{}` on every mode
entry (`_on_mode_enter`). This prevents stale substates from a previous mode bleeding into
a newly entered mode.

**Timers in Lua:** `_mode_ms` resets on mode change; `_submode_ms` resets on substate change.
Ground planner owns the pumping + landing phase state machine; Lua executes per-substate
collective and body_z slerp logic.

---

## Steady-State Initial Conditions

`simtest_ic.py` loads `steady_state_starting.json`.
**Only `test_generate_ic.py::test_create_ic` writes this file** — all other tests read it.

```python
from simtest_ic import load_ic
ic = load_ic()
# ic.pos, ic.vel, ic.R0, ic.omega_spin, ic.rest_length
# ic.eq_thrust       -- thrust [0..1] at which TensionPI settled (~300 N tension)
# ic.home_z_ned      -- GPS home NED Z [m]
```

`ic.eq_thrust` is determined by running 60 s warmup with `TensionPI(setpoint_n=300 N)`.
300 N is midway between pumping reel-in (226 N) and reel-out (435 N) targets.

Regenerate after any aero model change:
```bash
.venv/Scripts/python.exe -m pytest simulation/tests/simtests/test_generate_ic.py::test_create_ic -s
```

---

## PhysicsRunner — 400 Hz Physics Interface for Simtests

`simtest_runner.py` provides `PhysicsRunner`, a thin wrapper around `PhysicsCore`
(`simulation/physics_core.py`). `PhysicsCore` owns `RigidBodyDynamics`, `create_aero`,
`TetherModel`, the spin ODE, and yaw damping. `RateControllerSitl` (RatePID + servo model)
is baked into `PhysicsRunner` — callers produce `(col, rate_roll, rate_pitch)` and
`runner.step()` handles the rest. Callers own `PumpingGroundController`,
`TensionApController`, `WinchController` at their own rates.

### Python-AP simtest loop

```python
from simtest_runner import PhysicsRunner, tel_every_from_env

runner    = PhysicsRunner(rotor, ic, wind)
tel_every = tel_every_from_env(DT)

for i in range(max_steps):
    t_sim      = i * DT
    tension    = runner.tension_now
    altitude   = runner.altitude          # -pos_z [m]

    # Ground 10 Hz
    if i % planner_every == 0:
        cmd = ground.step(t_sim, tension, rest_length=winch.rest_length, hub_alt_m=altitude)
        ap.receive_command(cmd, DT_PLANNER)
        winch.set_target(ground.winch_target_length, ground.winch_target_tension)

    # Winch 400 Hz
    winch.step(tension, DT)

    # AP 400 Hz
    hub_state  = runner.hub_state
    omega_body = runner.omega_body        # mutable R.T @ omega copy
    col, rate_roll, rate_pitch = ap.step(hub_state["pos"], hub_state["R"], DT)
    sr = runner.step(DT, col, rate_roll, rate_pitch, omega_body,
                     rest_length=winch.rest_length)

    if i % tel_every == 0:
        telemetry.append(TelRow.from_physics(runner, sr, col, WIND, ...))

# IC generation warmup (zero initial velocity):
runner = PhysicsRunner.for_warmup(rotor, pos0, R0, rest_length, thrust_to_coll_rad(ic.eq_thrust), omega_spin, wind)
```

### Lua simtest loop

```python
from simtest_runner import PhysicsRunner, LuaAP

runner = PhysicsRunner(rotor, ic, wind)
lua    = LuaAP(sim, initial_thrust=ic.eq_thrust, wind=WIND, dt=DT)  # wind+dt mandatory
lua.tel_fn = lambda r, sr: dict(body_z_eq=None, phase=phase_label)     # set before loop

for i in range(max_steps):
    t_sim = i * DT

    # Ground 10 Hz
    if i % planner_every == 0:
        ground_ctrl.step(t_sim, runner.tension_now, runner.altitude, DT)

    # Lua tick (50–100 Hz depending on LUA_PERIOD)
    if i % lua_every == 0:
        lua.tick(t_sim, runner)
        # or: lua.tick(t_sim, runner, inject=lambda s, r: s.send_named_float("RAWES_TEN", r.tension_now))

    # Physics 400 Hz
    omega_body    = runner.omega_body
    omega_body[2] = 0.0          # yaw not controlled by Lua
    sr = runner.step(DT, lua.col_rad, lua.roll_sp, lua.pitch_sp, omega_body,
                     rest_length=winch.rest_length)
    lua.log(runner, sr)          # rate-gated telemetry append (uses tel_fn)

lua.write_telemetry(_log.log_dir / "telemetry.csv")
```

### Properties and methods

After each `step()`:
- `runner.hub_state` — dict: pos, vel, R, omega (state after integration)
- `runner.tension_now` — tether tension [N]
- `runner.altitude` — hub altitude [m] = `-pos_z`
- `runner.omega_body` — `R.T @ omega`, mutable copy (caller may zero `[2]` for yaw)
- `runner.omega_spin` — rotor spin [rad/s]
- `runner.tether` — TetherModel (inspect `_last_info["slack"]`)
- `runner.aero` — aero object
- `sr["tilt_lon"], sr["tilt_lat"]` — tilts used in that step
- `sr["aero_result"], sr["tether_force"], sr["tether_moment"]` — forces for telemetry

### LuaAP

`LuaAP(sim, *, wind, dt, initial_thrust=0.263)` wraps `RawesLua` for the standard Lua simtest tick.
`wind` and `dt` are mandatory keyword arguments.

```python
lua = LuaAP(sim, initial_thrust=ic.eq_thrust, wind=WIND, dt=DT)

lua.tick(t_sim, runner)
# lua.col_rad, lua.roll_sp, lua.pitch_sp now hold latest decoded PWM

# inject= pushes NV floats between feed_obs and update_fn:
lua.tick(t_sim, runner,
         inject=lambda s, r: s.send_named_float("RAWES_TEN", r.tension_now))
```

Lua collective is decoded from guided throttle (`guided_throttle`) when present,
with channel-3 PWM as fallback, then converted through `thrust_to_coll_rad()`.

### PythonAP

`PythonAP(ap, *, wind, dt)` mirrors `LuaAP` for Python AP controllers (`TensionApController`,
`LandingApController`). `wind` and `dt` are mandatory.

```python
ap = PythonAP(_ap, wind=WIND, dt=DT)
ap.tel_fn = lambda r, sr: {**ap.log_fields(), "phase": phase, "winch_speed_ms": speed}

for i in range(steps):
    if i % ap_every == 0:
        ap.tick(t_sim, runner)
    sr = runner.step(...)
    ap.log(runner, sr)   # rate-gated; uses tel_fn kwargs

ap.write_telemetry(_log.log_dir / "telemetry.csv")
```

`PythonAP.log_fields()` delegates to the wrapped controller's `log_fields()`:
- `TensionApController.log_fields()` → `{tension_setpoint, elevation_rad, el_correction_rad, coll_saturated, comms_ok, collective_from_tension_ctrl}`
- `LandingApController.log_fields()` → `{elevation_rad, body_z_eq}`

`WinchController.log_fields()` → `{winch_speed_ms}`. Use `**winch.log_fields()` alongside
`**ap.log_fields()` in the lambda; only test-specific context (phase, gnd_alt_cmd_m) goes directly.

---

## Telemetry in Simtests

Telemetry is fully managed by `LuaAP`/`PythonAP` — no manual `TelRow` or `write_csv` needed.

```python
# Set tel_fn once before the loop:
ap.tel_fn = lambda r, sr: {
    **ap.log_fields(),
    **winch.log_fields(),
    "phase": phase_label,
    "gnd_alt_cmd_m": cmd.alt_m,
}

# In the loop, after runner.step():
ap.log(runner, sr)   # rate-gated by RAWES_TEL_HZ env var (default 20 Hz)

# After the loop:
ap.write_telemetry(_log.log_dir / "telemetry.csv")
```

`TelRow.from_physics(runner, step_result, collective_rad, wind_ned, **kwargs)` is called internally.

`TelRow.from_physics(runner, step_result, collective_rad, wind_ned, **kwargs)` extracts all
physics state from the runner (`hub_state`, `tension_now`, `omega_spin`, `t_sim`, `aero`)
and from the step result dict (`tilt_lon`, `tilt_lat`, `aero_result`, `tether_force`,
`tether_moment`, `omega_body`, `accel_world`). Optional keyword args add ground-side
telemetry columns (`phase`, `tension_setpoint`, `gnd_alt_cmd_m`, `elevation_rad`, etc.).

`SimtestLog` from `simulation/simtest_log.py` writes human-readable summaries alongside
the CSV. Log output goes to `simulation/logs/test_<name>/`.

---

## Key Lua Sync Rules

**CRITICAL — `rawes_test_surface.lua` must be updated when rawes.lua gains new locals.**
The harness splices `rawes_test_surface.lua` inside rawes.lua's anonymous function wrapper,
giving it access to all module-level locals. Constants and functions are exposed through
`_rawes_fns`; Python tests read them via `sim.fns.*` — no Python copies to maintain.

- When adding a **module-level** `local` constant or function that tests need, add it to
  `_rawes_fns` in `rawes_test_surface.lua` in the same commit.
- When adding a **function-local** constant that tests need, hoist it to module level
  first, then add it to `_rawes_fns`. Function-locals are out of scope for the splice.

**CRITICAL — `TelRow` fields in `telemetry_csv.py` must match `COLUMNS` in `telemetry_columns.py`.**
The two files are not auto-generated from each other.  When the telemetry schema changes:

1. Update `COLUMNS` / `ASYNC_MAV_COLUMNS` in `telemetry_columns.py` (the schema source of truth).
2. Update the `TelRow` dataclass fields in `telemetry_csv.py` to match exactly.
3. Update any code that constructs `TelRow` explicitly — especially `torque_test_utils.py`
   which hard-codes `nvf_latest` dict keys and `TelRow(...)` keyword arguments.
4. Update `mediator.py` row-write dict to use the new column names.

A mismatch causes `TelRow.to_dict()` to raise `AttributeError` at test runtime, not at
collection time, so it only surfaces when a test actually writes telemetry.
- `test_yaw_lua.py` reads all yaw constants from `sim.fns.*` — no manual copies.
- `test_math_lua.py` runs actual rawes.lua functions via `_rawes_fns` and cross-checks
  against `controller.py` equivalents. A failing test means `controller.py` diverged; fix that.
- A failing Lua unit test after a rawes.lua edit means the Python assertion constants
  are stale. Fix the constants, then fix the test.

---

## Testing Pitfall: Round-Trip Tests Mask Symmetric Bugs

**Classic Bug Pattern:** Forward/inverse mixing or bi-directional transforms where **both directions are swapped identically**. Example: swashplate servo model mixing functions.

**Why it's dangerous:**  
If you swap parameters in both `forward_mix(col, roll, pitch)` → `(s1, s2, s3)` AND `inverse_mix(s1, s2, s3)` → `(col, roll, pitch)` symmetrically, then:
- Round-trip tests pass: `forward(x)` → `inverse(result)` = `x` ✓ (both swaps cancel)
- Direct function tests pass: each function individually tested with correct arguments ✓
- **Integration tests fail silently:** SwashplateServoModel calls forward() with swapped args, getting wrong servo positions, but inverse() swaps them back to match the command

**Mitigation:**  
For any matrix or transform with an inverse, add **directional validation tests** that verify intermediate results are physically meaningful:
- Servo positions move in expected directions for cyclic commands
- Moment vectors align with expected axes
- Cross-coupling is minimal when not commanded

Example: `test_swashplate_servo_directions.py` validates that positive `tilt_lat` (roll right) produces:
- S1 (front-right servo) moves **UP** (expected for roll)
- S2 (front-left servo) moves **DOWN** (expected for roll)
- S3 (back servo) unchanged (no roll effect)

This catches parameter swaps that round-trip tests cannot.
