# RAWES Simulation

End-to-end flight simulation for the **Rotary Airborne Wind Energy System (RAWES)** — a 4-blade tethered autorotating rotor kite controlled via trailing-edge flaps through an H3-120 swashplate.

Combines ArduPilot SITL (flight controller logic), a Python RK4 dynamics engine, and a Python mediator (aerodynamics + sensor simulation).

## Scope

This file is implementation-oriented (module layout, how to run, where logs live).
Canonical design details are maintained in:

- [../design/simulation.md](../design/simulation.md) for simulation internals and physics/controller concepts
- [../design/flight_stack.md](../design/flight_stack.md) for AP/Lua flight-control ownership and behavior
- [../design/sitl_testing.md](../design/sitl_testing.md) for stack workflow and diagnosis

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│ ArduPilot SITL (heli JSON backend)                              │
│   Sends servo PWM commands, receives JSON sensor state          │
└────────────────┬──────────────────────────────────────────────┘
                 │ UDP 9002 (servo commands)
                 │ UDP 9003 (sensor state JSON)
                 ▼
┌─────────────────────────────────────────────────────────────────┐
│ mediator.py  (400 Hz loop)                                      │
│   servo PWM → swashplate.py → collective, tilt_lon, tilt_lat   │
│   aero/          → aerodynamic wrench (NED world frame)        │
│   tether.py      → tether tension force + moment               │
│   dynamics.py    → RK4 6-DOF step → {pos, vel, R, omega}       │
│   sensor.py      → build_sitl_packet() → NED sensor outputs    │
└─────────────────────────────────────────────────────────────────┘
                 │ MAVLink TCP 5760
                 ▼
┌─────────────────────────────────────────────────────────────────┐
│ gcs.py / test GCS                                               │
│   Arm, set mode, send NV floats, read MAVLink messages          │
└─────────────────────────────────────────────────────────────────┘
```

---

## Module Summary

| File | Role |
|------|------|
| `mediator.py` | 400 Hz co-simulation loop — orchestrates all subsystems |
| `dynamics.py` | Python RK4 6-DOF rigid-body integrator (gravity internal) |
| `aero/` | Aerodynamic model package |
| `tether.py` | Tension-only elastic tether (Dyneema SK75) |
| `swashplate.py` | H3-120 inverse mixing and cyclic blade pitch math |
| `frames.py` | Coordinate-frame utilities (`build_orb_frame()`, transforms) |
| `sensor.py` | `build_sitl_packet()` — NED truth state → ArduPilot JSON sensor packet |
| `sitl_interface.py` | ArduPilot SITL UDP binary protocol (servo recv, state send) |
| `controller.py` | `compute_bz_altitude_hold` + `slerp_body_z` — active elevation-hold path |
| `gcs.py` | MAVLink GCS client (arm, mode, params, named-float commands) |
| `flight_report.py` | Multi-panel PNG flight report from position/attitude/servo history |

Analysis scripts (not part of simulation runtime): `analysis/`

---

## Coordinate Frames

All defined in and imported from `frames.py`.

| Frame | Axes | Where used |
|-------|------|-----------|
| NED (world) | X=North, Y=East, Z=Down | dynamics, aero, tether, controller, sensor, SITL |
| Body | columns of R_hub (body→world) | gyro, accel, swashplate |

All physics uses NED. `T_ENU_NED` in `frames.py` is a utility for external ENU conversions.

---

## Sensor Design

For the full physical sensor model, conventions, and invariants, see
[../design/simulation.md](../design/simulation.md).

---

## Initial State

The generated startup state is stored in `steady_state_starting.json`.
Generation and acceptance criteria are documented in [../design/testing.md](../design/testing.md).

---

## Running Tests

### Unit tests (Windows, no Docker)

```bash
.venv/Scripts/python.exe -m pytest tests/unit -m "not simtest" -q
```

Key unit tests:
- `test_closed_loop.py` -- closed-loop physics (dynamics + aero + tether + controller), no ArduPilot
- `test_steady_flight.py` -- open-loop equilibrium, writes `steady_state_starting.json`
- `test_controller.py` -- controller function unit tests

### Stack integration tests (Docker)

```bash
bash test.sh stack -v
```

See [../design/sitl_testing.md](../design/sitl_testing.md) for full Docker setup and test commands.

---

## Analysis Tools

Standalone scripts in `analysis/`. Not part of the simulation runtime; not imported by
`mediator.py` or any test fixture. Run after stack tests to produce structured reports and plots.

**Always run analyse_run.py after a stack test:**

```bash
# List available test runs (newest first)
.venv/Scripts/python.exe analysis/analyse_run.py
# Analyse a specific test
.venv/Scripts/python.exe analysis/analyse_run.py test_acro_armed
.venv/Scripts/python.exe analysis/analyse_run.py test_pumping_cycle --plot
```

| Script | Purpose | Status |
|--------|---------|--------|
| `analyse_run.py` | Post-run structured report; reads `logs/{test_name}/telemetry.csv` + `mediator.log` | Active |
| `analyse_gps_fusion.py` | EKF3 GPS fusion event analysis from `logs/{test_name}/gcs.log` | Active |
| `analyse_pumping_cycle.py` | Pumping cycle energy/tension analysis from telemetry CSV | Active |
| `flight_report.py` | Offline multi-panel PNG plot from mediator telemetry CSV | Active |
| `merge_logs.py` | Merge mediator/SITL/GCS logs in timestamp order for unified timeline | Active |
| `build.py` | Docker image build progress monitor | Active |
| `compare_rotors.py` | Side-by-side rotor definition comparison | Active |
| `sg6042_polar.py` | SG6042 airfoil polar plotter | Active |

Per-test logs: `simulation/logs/{test_name}/` -- `telemetry.csv`, `mediator.log`,
`sitl.log`, `gcs.log`, `arducopter.log`. Suite log: `simulation/logs/pytest_last_run.log`.

