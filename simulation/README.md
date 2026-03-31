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

## Sensor Design — Tether-Relative Attitude

The RAWES equilibrium has body Z ~65° from vertical (aligned with the tether). Reporting absolute attitude causes ArduPilot to see ~65° tilt at rest and command maximum cyclic, crashing the hub.

`build_sitl_packet()` in `sensor.py` instead reports **tether-relative attitude**:
- `roll = 0, pitch = 0` always (zero at tether equilibrium)
- `yaw` derived from velocity heading (keeps EKF attitude/velocity consistent)
- Gyro: spin stripped, rotated into yaw-aligned body frame
- Accel: kinematic accel in NED body frame + −g correction

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

```cmd
simulation\tests\unit\.venv\Scripts\python.exe -m pytest simulation\tests\unit
```

Key unit tests:
- `test_closed_loop.py` — closed-loop physics (dynamics + aero + tether + controller), no ArduPilot
- `test_steady_flight.py` — open-loop equilibrium, writes `steady_state_starting.json`
- `test_controller.py` — controller function unit tests

### Stack integration tests (Docker via WSL)

```cmd
wsl.exe bash -c 'bash /mnt/e/repos/windpower/simulation/dev.sh test-stack'
```

See `CLAUDE.md` in the repo root for full Docker setup and test commands.

