# Unit Tests — Reference Guide

Unit tests live in `simulation/tests/unit/` and run **Windows-native** (no Docker).
They cover the full simulation stack: physics, aero, controller maths, and the actual
`rawes.lua` Lua controller running in-process via lupa.

---

## Running Tests

```bash
# Fast unit tests (~483) — exclude slow simtests
simulation/.venv/Scripts/python.exe simulation/run_tests.py simulation/tests/unit -m "not simtest" -q

# Slow simtests (14, 12 pass) — full physics loops
simulation/.venv/Scripts/python.exe simulation/run_tests.py simulation/tests/unit -m simtest -q

# Single test
simulation/.venv/Scripts/python.exe simulation/run_tests.py simulation/tests/unit/test_steady_flight_lua.py -q

# Regenerate steady-state IC (required after any aero model change)
simulation/.venv/Scripts/python.exe simulation/run_tests.py simulation/tests/unit/test_steady_flight.py -q
```

**CRITICAL:** Never use `dev.sh test-unit` — it routes to Docker which excludes `tests/unit`.
**CRITICAL:** Always use `simulation/run_tests.py` (not `python -m pytest` directly) — it checks
the venv is up to date and saves structured logs.

---

## Test Markers

| Marker | Meaning | Default run |
|--------|---------|-------------|
| *(none)* | Fast unit test, no physics loop | Included |
| `simtest` | Full time-domain physics loop — seconds of compute | Excluded |

Defined in `conftest.py`. Mark slow tests with `@pytest.mark.simtest`.

---

## Test Files

### Physics & Aero

| File | What it tests |
|------|--------------|
| `test_aero_trajectory_points.py` | BEM aero at specific trajectory sample points |
| `test_deschutter_equations.py` | De Schutter Eq. 25–31 formula correctness |
| `test_deschutter_validation.py` | Numerical validation vs. reference values |
| `test_skewed_wake_jit.py` | SkewedWakeBEMJit vs. reference (18 equivalence cases, atol=1e-10) |
| `test_force_balance.py` | Static force balance at equilibrium |
| `test_physical_validation.py` | Physical consistency checks (mass, inertia, geometry) |
| `test_gyroscopic_orbit.py` | Gyroscopic coupling during orbital flight |
| `test_swashplate.py` | H3-120 inverse mixing, cyclic blade pitch |
| `test_tether_stability.py` | Tether elastic model — tension, slack, snap |
| `test_rotor_definition.py` | RotorDefinition YAML loading and validation |

### Simulation Loops (simtests)

Three scenario pairs — each has a Python-only version and a Python+Lua version.
All three start from the same `steady_state_starting.json` IC (written by `test_steady_flight.py`).

| Python | Lua | Scenario |
|--------|-----|----------|
| `test_steady_flight.py` | `test_steady_flight_lua.py` | 90 s orbit from IC; Python also writes IC |
| `test_pump_cycle.py` | `test_pump_cycle_lua.py` | N De Schutter reel-out/reel-in cycles from IC |
| `test_landing.py` | `test_landing_lua.py` | reel-in only (no reel-out) swings hub to xi~80°, then VZ descent + final drop |

Supporting simtests (no Lua pair):

| File | What it tests |
|------|--------------|
| `test_gyroscopic_orbit.py` | Gyroscopic coupling during orbital flight |
| `test_kinematic_transition.py` | Kinematic → free-flight hand-off (pos, vel, R continuity) |
| `test_sensor_closed_loop.py` | Sensor feeding EKF-equivalent closed loop |

### Controller & Sensor

| File | What it tests |
|------|--------------|
| `test_controller.py` | `compute_bz_tether`, `slerp_body_z`, `compute_rate_cmd`, `RatePID`, `OrbitTracker` |
| `test_sensor.py` | `PhysicalSensor` output consistency (gyro, accel, body_to_earth) |
| `test_sensor_closed_loop.py` | Sensor feeding EKF-equivalent closed loop |
| `test_kinematic_transition.py` | Kinematic → free-flight hand-off (pos, vel, R continuity) |
| `test_startup_trajectory.py` | KinematicStartup orbit trajectory validation |
| `test_wind_estimator.py` | Wind estimator accuracy vs. ground truth |

### Mediator & Protocol

| File | What it tests |
|------|--------------|
| `test_mediator.py` | 400 Hz co-simulation loop, sensor packet format |
| `test_mediator_transport.py` | UDP send/receive, lockstep compliance |
| `test_interfaces.py` | `sitl_interface.py` binary protocol encoding |
| `test_planner_protocol.py` | `WinchNode` planner/physics protocol boundary |

### Lua Controller Tests

`test_<name>_lua.py` mirrors `test_<name>.py` — same physics and IC, Lua controller instead of Python.

| File | Non-Lua reference | What it tests |
|------|------------------|--------------|
| `test_math_lua.py` | — | Lua math: `rodrigues`, `orbit_track_azimuthal`, `slerp_step`, cyclic error |
| `test_yaw_lua.py` | — | Yaw-trim: runs rawes.lua via `RawesLua` harness (PI, dead zone, watchdog, hard-stop, closed-loop equilibrium) |
| `test_steady_flight_lua.py` | `test_steady_flight.py` | rawes.lua mode=1: 90 s orbit from IC via harness |
| `test_pump_cycle_lua.py` | `test_pump_cycle.py` | rawes.lua mode=5: N De Schutter cycles from IC via harness |
| `test_landing_lua.py` | `test_landing.py` | rawes.lua mode=4: landing from IC via harness |

---

## Lua Test Infrastructure

The Lua tests run `rawes.lua` **in-process** using [lupa](https://github.com/scoder/lupa) (Lua 5.4
embedded in Python). No SITL, no Docker, no real sleeping.

### Key Files

| File | Purpose |
|------|---------|
| `rawes_lua_harness.py` | `RawesLua` class — Python interface to rawes.lua |
| `mock_ardupilot.lua` | ArduPilot API stub (AHRS, RC, SRV_Channels, param, gcs, arming, vehicle) |
| `simulation/scripts/rawes_test_surface.lua` | Test surface spliced into rawes.lua — exposes internal functions via `_rawes_fns` |
| `simulation/rawes_modes.py` | Central Python constants mirroring rawes.lua mode/substate numbers |

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
   `_rawes_update()`, and reads RC overrides from `_mock.ch_out[n]`.

### RawesLua API

```python
from rawes_lua_harness import RawesLua
from rawes_modes import MODE_PUMPING, PUMP_HOLD

sim = RawesLua(mode=MODE_PUMPING + PUMP_HOLD)  # SCR_USER6 = 5000
sim.armed        = True
sim.healthy      = True
sim.vehicle_mode = 1           # ACRO = 1
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

# Ground planner: write mode + substate
sim.set_param("mode", MODE_PUMPING + PUMP_REEL_OUT)  # SCR_USER6 = 5001

# Access Lua internals (via rawes_test_surface)
sim.fns.T_TRANSITION           # constant
bz = sim.fns.disk_normal_ned() # Lua Vector3f
sim.vec_to_list(bz)            # -> [x, y, z]
```

### `_mock` Layout

| Field | Direction | Type | Notes |
|-------|-----------|------|-------|
| `armed` | in | bool | arming state |
| `mode` | in | int | vehicle flight mode (1 = ACRO) |
| `healthy` | in | bool | AHRS health |
| `millis_val` | in | int | fake milliseconds |
| `gyro` | in | {x,y,z} | body-frame gyro rad/s |
| `pos_ned` | in | {x,y,z} or nil | GPS position; nil = not fused |
| `vel_ned` | in | {x,y,z} | NED velocity m/s |
| `R` | in | flat 1..9 | body-to-NED rotation, row-major |
| `params` | in/out | table | SCR_USER1..6 |
| `ch_out[n]` | out | int or nil | RC channel override PWM |
| `srv_out[func]` | out | int or nil | SRV_Channels PWM by function |
| `gcs_msgs` | out | array | {level, msg} log |

---

## SCR_USER6 Mode/Substate Encoding

`SCR_USER6 = mode_num * 1000 + substate`

Constants are in `simulation/rawes_modes.py` (Python) and `rawes.lua` (Lua).
**Keep both in sync.**

```python
from rawes_modes import (
    MODE_NONE, MODE_STEADY, MODE_YAW, MODE_STEADY_YAW,
    MODE_LANDING, MODE_PUMPING, MODE_ARM_HOLD, MODE_YAW_TEST, MODE_YAW_LTD,
    LAND_DESCEND, LAND_FINAL_DROP,
    PUMP_HOLD, PUMP_REEL_OUT, PUMP_TRANSITION, PUMP_REEL_IN, PUMP_TRANSITION_BACK,
)
# e.g. SCR_USER6 = 5001  ->  MODE_PUMPING + PUMP_REEL_OUT
```

| Mode | Value | Substates |
|------|-------|-----------|
| `MODE_NONE` | 0 | — |
| `MODE_STEADY` | 1000 | — |
| `MODE_YAW` | 2000 | — |
| `MODE_STEADY_YAW` | 3000 | — |
| `MODE_LANDING` | 4000 | 0=DESCEND, 1=FINAL_DROP |
| `MODE_PUMPING` | 5000 | 0=HOLD, 1=REEL_OUT, 2=TRANSITION (after winch reverses), 3=REEL_IN, 4=TRANSITION_BACK (cycle>0 start) |
| `MODE_ARM_HOLD` | 6000 | — |
| `MODE_YAW_TEST` | 7000 | — |
| `MODE_YAW_LTD` | 8000 | — |

**Timers in Lua:** `_mode_ms` resets on mode change; `_submode_ms` resets on substate change.
Ground planner owns the pumping + landing phase state machine; Lua executes per-substate
collective and body_z slerp logic.

---

## Steady-State Initial Conditions

`simtest_ic.py` loads `steady_state_starting.json` (written by `test_steady_flight.py`).

```python
from simtest_ic import load_ic
ic = load_ic()
# ic.pos, ic.vel, ic.body_z, ic.omega_spin, ic.rest_length
# ic.coll_eq_rad     -- equilibrium collective [rad]
# ic.stack_coll_eq   -- orbit collective (coll_eq + 0.10 rad)
# ic.home_z_ned      -- GPS home NED Z [m]
```

Regenerate after aero model changes:
```bash
simulation/.venv/Scripts/python.exe -m pytest simulation/tests/unit -k test_steady_flight
```

---

## Telemetry in Simtests

All simtests that emit telemetry use `make_tel()` from `tel.py`. Rows are written to CSV
via `TelRow.from_tel()` + `write_csv()` from `telemetry_csv.py`.

```python
from tel import make_tel
from telemetry_csv import TelRow, write_csv

row = make_tel(t, hub_state, omega_spin, tether, tension, col, tilt_lon, tilt_lat, wind)
write_csv([TelRow.from_tel(r) for r in rows], path / "telemetry.csv")
```

Log output goes to `simulation/logs/test_<name>/`.  Use `SimtestLog` from `simtest_log.py`
to write human-readable summaries alongside the CSV.

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
- `test_yaw_lua.py` reads all yaw constants (`KP_YAW`, `YAW_DEAD_ZONE_RAD_S`, etc.)
  from `sim.fns.*` — no manual copies.
- `test_math_lua.py` runs actual rawes.lua functions via `_rawes_fns` and cross-checks
  against `controller.py`. A failing test means `controller.py` diverged; fix that.
- A failing Lua unit test after a rawes.lua edit means the Python assertion constants
  are stale. Fix the constants, then fix the test.
