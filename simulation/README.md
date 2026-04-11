# RAWES Simulation

End-to-end flight simulation for the **Rotary Airborne Wind Energy System (RAWES)** — a 4-blade tethered autorotating rotor kite controlled via trailing-edge flaps through an H3-120 swashplate.

Combines ArduPilot SITL (flight controller logic), a Python RK4 dynamics engine, and a Python mediator (aerodynamics + sensor simulation).

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
│   aero.py        → aerodynamic wrench (NED world frame)        │
│   tether.py      → tether tension force + moment               │
│   dynamics.py    → RK4 6-DOF step → {pos, vel, R, omega}       │
│   sensor.py      → build_sitl_packet() → NED sensor outputs    │
└─────────────────────────────────────────────────────────────────┘
                 │ MAVLink TCP 5760
                 ▼
┌─────────────────────────────────────────────────────────────────┐
│ gcs.py / test GCS                                               │
│   Arm, set mode, send RC overrides, read MAVLink messages       │
└─────────────────────────────────────────────────────────────────┘
```

---

## Module Summary

| File | Role |
|------|------|
| `mediator.py` | 400 Hz co-simulation loop — orchestrates all subsystems |
| `dynamics.py` | Python RK4 6-DOF rigid-body integrator (gravity internal) |
| `aero.py` | De Schutter (2018) BEM aerodynamic model |
| `tether.py` | Tension-only elastic tether (Dyneema SK75) |
| `swashplate.py` | H3-120 inverse mixing and cyclic blade pitch math |
| `frames.py` | `T_ENU_NED` legacy constant and `build_orb_frame()` — shared coordinate-frame utilities |
| `sensor.py` | `build_sitl_packet()` — NED truth state → ArduPilot JSON sensor packet |
| `sitl_interface.py` | ArduPilot SITL UDP binary protocol (servo recv, state send) |
| `controller.py` | `compute_swashplate_from_state()` — truth-state tether-alignment controller; `compute_rc_from_attitude()` — ACRO RC override controller |
| `gcs.py` | MAVLink GCS client (arm, mode, RC override, params) |
| `flight_report.py` | Multi-panel PNG flight report from position/attitude/servo history |

Analysis scripts (not part of simulation runtime): `analysis/`

---

## Coordinate Frames

All defined in and imported from `frames.py`.

| Frame | Axes | Where used |
|-------|------|-----------|
| NED (world) | X=North, Y=East, Z=Down | dynamics, aero, tether, controller, sensor, SITL |
| Body | columns of R_hub (body→world) | gyro, accel, swashplate |

All physics uses NED. `T_ENU_NED` in `frames.py` is kept as a utility for converting legacy ENU data.

---

## Sensor Design — Physical Attitude

`PhysicalSensor` in `sensor.py` reports the **true physical orbital-frame orientation** (~124° roll / −46° pitch at tether equilibrium, ZYX Euler NED). ACRO mode is used because it only damps angular rates, so the large physical tilt causes no automatic corrective cyclic.

- `rpy` = actual ZYX Euler angles (spin stripped), yaw replaced by velocity heading
- `yaw` derived from `atan2(vE, vN)`, rate-limited to prevent gyro body-axis remapping at tether activation
- Gyro: spin stripped, rotated into physical orbital body frame
- Accel: specific force in physical orbital body frame

---

## Initial State

Default starting state (warmup-settled equilibrium, 50 m tether at ~30° elevation):

| Parameter | Value |
|-----------|-------|
| pos (NED) | `[14.241, 46.258, -12.530]` m |
| vel (NED) | `[0.916, -0.257, 0.093]` m/s |
| body_z    | `[0.851, 0.305, 0.427]` (axle aligned with tether) |
| omega_spin | 20.148 rad/s |

Regenerate: `pytest tests/unit/test_steady_flight.py` → writes `steady_state_starting.json`.

---

## Running Tests

### Unit tests (Windows, no Docker)

```bash
bash sim.sh test-unit -q
```

Key unit tests:
- `test_closed_loop.py` -- closed-loop physics (dynamics + aero + tether + controller), no ArduPilot
- `test_steady_flight.py` -- open-loop equilibrium, writes `steady_state_starting.json`
- `test_controller.py` -- controller function unit tests

### Stack integration tests (Docker via WSL)

```bash
bash sim.sh test-stack -v
```

See `CLAUDE.md` in the repo root for full Docker setup and test commands.

---

## Analysis Tools

Standalone scripts in `analysis/`. Not part of the simulation runtime; not imported by
`mediator.py` or any test fixture. Run after stack tests to produce structured reports and plots.

**Always run analyse_run.py after a stack test:**

```bash
# List available test runs (newest first)
bash sim.sh exec 'python3 /rawes/simulation/analysis/analyse_run.py'
# Analyse a specific test
bash sim.sh exec 'python3 /rawes/simulation/analysis/analyse_run.py test_acro_armed'
bash sim.sh exec 'python3 /rawes/simulation/analysis/analyse_run.py test_pumping_cycle --plot'
```

| Script | Purpose | Status |
|--------|---------|--------|
| `analyse_run.py` | Post-run structured report; reads `logs/{test_name}/telemetry.csv` + `mediator.log` | Active |
| `analyse_gps_fusion.py` | EKF3 GPS fusion event analysis from `logs/{test_name}/gcs.log` | Active |
| `analyse_pumping_cycle.py` | Pumping cycle energy/tension analysis from telemetry CSV | Active |
| `generate_flight_report.py` | Offline multi-panel PNG plot from mediator telemetry CSV | Active |
| `merge_logs.py` | Merge mediator/SITL/GCS logs in timestamp order for unified timeline | Active |
| `build.py` | Docker image build progress monitor | Active |
| `compare_rotors.py` | Side-by-side rotor definition comparison | Active |
| `sg6042_polar.py` | SG6042 airfoil polar plotter | Active |

Per-test logs: `simulation/logs/{test_name}/` -- `telemetry.csv`, `mediator.log`,
`sitl.log`, `gcs.log`, `arducopter.log`. Suite log: `simulation/logs/pytest_last_run.log`.

