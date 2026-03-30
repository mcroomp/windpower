# RAWES — Project Context for Claude

## Project Goal

Build an **ArduPilot flight controller model** for a Rotary Airborne Wind Energy System (RAWES) that can fly in all standard modes: takeoff, stabilized flight, autonomous flight, landing. This is a long-term, step-by-step effort.

**Current phase:** Phase 3, Milestone 3 — Pumping cycle stack test PASSED (`test_pumping_cycle.py`: net energy +1901 J, reel-in 55 N vs reel-out 211 N). Rotor definition abstracted into `rotor_definition.py` + YAML files; mediator/aero/dynamics wired to `beaupoil_2026.yaml`. 268 fast unit tests + 23 simtests passing. `ModeRAWES` ArduPilot firmware architecture fully designed (documented in `simulation/raws_mode.md`). Next: write `rawes_params.parm` and ArduPilot hardware frame configuration.

See **[Phase 3 Plan](#phase-3-plan)** below for the full milestone breakdown.

---

## What Is This System?

A **RAWES** (Rotary Airborne Wind Energy System) is a tethered, rotating 4-blade rotor kite. It operates like an autogyro rotor — wind drives autorotation, and cyclic pitch control tilts the rotor disk to steer. A tether connects to the bottom of the rotor axis; tension during reel-out drives a ground generator (pumping cycle power generation).

Key distinction from a drone: **no motor drives rotation** — wind does. Control is entirely through blade pitch, actuated indirectly via trailing-edge flaps on each blade.

---

## Control Architecture

Three nested control problems from the source literature:

```
Planned Trajectory → [MPC] → Reference Blade Pitch β̄
                                      ↓
                             [Flap Controller] → Flap Actuation γ_CL
                                      ↑                      ↓
                              Actual Blade Pitch β ← [RAWES Model] → Output
```

- **A — Trajectory → reference pitch:** MPC (Model Predictive Control) — not yet implemented
- **B — Reference pitch → flap actuation:** Feed-forward + PID — implemented in source thesis, not yet in runtime stack
- **C — Physics simulation:** Full nonlinear ODE state space model — source thesis; current stack uses a compact BEM + RK4 model

The ArduPilot integration sits at level A (trajectory planning) and will delegate to level B for actuation.

---

## Physical System

> Full details in [physical_design.md](physical_design.md) and [hardware.md](hardware.md)

**Key parameters:**

| Parameter          | Value         |
|--------------------|---------------|
| Blade count        | 4 (90° apart) |
| Blade length       | 2000 mm       |
| Total rotor radius | ~2500 mm      |
| Rotor mass         | 5 kg          |
| Blade airfoil      | SG6042        |
| Tether diameter    | 1.9 mm (Dyneema SK75) |
| Max tether length  | 300 m         |
| Min altitude       | 10 m          |
| Tether attachment  | Bottom of axle|
| Anti-rotation motor| EMAX GB4008 66KV, 80:44 spur gear |
| Servos S1/S2/S3    | DS113MG V6.0 |
| Flight controller  | Holybro Pixhawk 6C |
| Battery            | 4S LiPo 15.2V, 450 mAh |

**Critical simulation note:** Only the spinning outer hub shell and blades contribute to `I_spin`. The stationary assembly (~1 kg) does not spin. Both bearing friction and push-rod reaction forces are **internal** forces in the single-body model — motor torque must **not** be added as an external couple.

---

## Reference Documents

### Hardware & Physical Design
| File | When to read |
|------|-------------|
| [physical_design.md](physical_design.md) | Full assembly layout, rotor geometry, swashplate, servo flaps, all component specs |
| [hardware.md](hardware.md) | Detailed component specs, power architecture, wiring |
| [flaprotordesign.md](flaprotordesign.md) | Blade design, airfoil selection (SG6042), Reynolds numbers, flap prototype |
| [servoflaps.md](servoflaps.md) | Kaman flap mechanism (US patent US3217809), swashplate linkage path |

### Control Theory
| File | When to read |
|------|-------------|
| [flapmodel.md](flapmodel.md) | Full Weyel 2025 math model — aerodynamics, state space, flap dynamics, controller, parameter tables |
| [summary.md](summary.md) | De Schutter et al. 2018 optimal control — pumping cycle, structural constraints, atmosphere model |

### ArduPilot / Firmware
| File | When to read |
|------|-------------|
| [ardupilot_implementation.md](ardupilot_implementation.md) | Swashplate config, Kaman flap lag problem, RSC, companion computer architecture |
| [simulation/ARMING.md](simulation/ARMING.md) | **Arming reference** — RSC modes, motor interlock, EKF init sequence, failure modes. **Update whenever new arming behavior is discovered.** |
| [simulation/heliparams.md](simulation/heliparams.md) | **EKF3 GPS fusion** — exact fusion conditions, required params, 10 s GPS-check delay. Read before debugging EKF3 CONST_POS_MODE. |
| [simulation/raws_mode.md](simulation/raws_mode.md) | **ModeRAWES complete spec** — MAVLink protocol, C++ firmware (~160 lines), orbit tracking, tension PI, omega_spin, parameter table. |

### Simulation Internals
| File | When to read |
|------|-------------|
| [simulation/sim_internals.md](simulation/sim_internals.md) | Sensor design, controller functions, dynamics/aero/tether model details, initial state, startup freeze, known gaps |
| [simulation/history.md](simulation/history.md) | Phase 2 accomplishments, Phase 3 M3 architecture decisions |
| [simulation/mbdyn_reference.md](simulation/mbdyn_reference.md) | MBDyn restoration instructions (archived; MBDyn removed from runtime) |

---

## Simulation Architecture

### Design Philosophy — Three-Phase Validation

1. **Physics validation** — does tether + aero + dynamics produce stable autorotation? (`test_steady_flight.py`)
2. **Closed-loop controller validation** — does `compute_swashplate_from_state` stabilize the physics model without ArduPilot? (`test_closed_loop.py`)
3. **ArduPilot integration** — can ArduPilot's ACRO/GUIDED modes fly the hub once physics + control are known good? (`test_guided_flight.py`)

Phases 1 and 2 run on Windows with no Docker. Phase 3 requires Docker + ArduPilot SITL.

### Module Map

```
simulation/
├── Core physics
│   ├── dynamics.py          RK4 6-DOF rigid-body integrator
│   ├── aero.py              BEM aerodynamic model (thrust, drag, cyclic moments)
│   ├── tether.py            Tension-only elastic tether (Dyneema SK75)
│   └── swashplate.py        H3-120 inverse mixing, cyclic blade pitch
│
├── Coordinate frames
│   └── frames.py            T_ENU_NED matrix, build_orb_frame() — single source
│
├── Sensor & interface
│   ├── sensor.py            ENU→NED conversion, build_sitl_packet(), SensorSim
│   └── sitl_interface.py    ArduPilot SITL UDP binary protocol
│
├── Control
│   └── controller.py        compute_swashplate_from_state()  — truth-state controller
│                            compute_rc_rates()               — ArduPilot RC override controller
│                            compute_rc_from_attitude()       — ATTITUDE message controller
│
├── Orchestration
│   ├── mediator.py          400 Hz co-simulation loop (SITL ↔ physics)
│   └── gcs.py               MAVLink GCS client (arm, mode, RC override, params)
│
├── Reporting
│   └── flight_report.py     Multi-panel flight report plotter
│
├── Analysis tools
│   └── analysis/
│       ├── analyse_run.py             Post-run structured report (always run after stack test)
│       ├── generate_flight_report.py  Offline report from mediator telemetry CSV
│       ├── redraw_flight_report.py    Regenerate PNG from saved flight_data.json
│       └── merge_logs.py              Unified log timeline merger
│
└── tests/
    ├── unit/                Windows native, no Docker
    │   ├── test_closed_loop.py      ★ Closed-loop physics (dynamics+aero+tether+controller)
    │   ├── test_steady_flight.py    Open-loop equilibrium → writes steady_state_starting.json
    │   ├── test_controller.py       Unit tests for controller.py functions
    │   ├── test_mediator.py         Mediator loop with fake SITL/dynamics
    │   ├── test_aero.py             Aerodynamic model
    │   └── ...
    └── stack/               Docker required
        ├── conftest.py              acro_armed fixture (full stack lifecycle)
        ├── test_guided_flight.py    60 s ACRO hold with tether-alignment RC controller
        ├── test_pumping_cycle.py    Pumping cycle stack test
        └── test_setup.py            Verifies setup reaches armed ACRO
```

### Data Flow (400 Hz mediator loop)

```
ArduPilot SITL (UDP 9002)
  │ servo PWM [16 channels]
  ▼
mediator.py
  ├─ swashplate.h3_inverse_mix()        → collective, tilt_lon, tilt_lat
  ├─ aero.compute_forces()              → F_world[6] (ENU wrench)
  ├─ tether.compute()                   → F_tether, M_tether (added to wrench)
  ├─ dynamics.step()                    → {pos, vel, R, omega} (ENU)
  └─ sensor.build_sitl_packet()         → {pos_ned, vel_ned, rpy, accel_body, gyro_body}
  │ JSON state packet
  ▼
ArduPilot SITL (UDP 9003)
```

### Coordinate Conventions

All defined in `frames.py`. Import from there — do not duplicate.

| Frame | Axes | Used by |
|-------|------|---------|
| ENU (world) | X=East, Y=North, Z=Up | dynamics, aero, tether, controller |
| NED (ArduPilot) | X=North, Y=East, Z=Down | sensor output, SITL JSON |
| Body | columns of R_hub | gyro, accel, swashplate commands |

**ENU ↔ NED:** `T_ENU_NED` from `frames.py`. This matrix is symmetric (T @ T = I).

**Orbital frame:** `build_orb_frame(body_z)` from `frames.py`. Removes rotor spin; body X = East projected onto disk plane.

---

## Running Tests

### Unit Tests and Simtests — Windows native, no Docker

**CRITICAL: Only ArduPilot/stack tests require Docker. Unit tests and simtests run directly on Windows using the local venv.**

One-time venv setup:
```cmd
py -3 -m venv simulation\tests\unit\.venv
simulation\tests\unit\.venv\Scripts\python.exe -m pip install numpy pytest matplotlib
```

### Quick Reference

| Task | Command |
|------|---------|
| **Unit tests (fast, local)** | `simulation/tests/unit/.venv/Scripts/python.exe -m pytest simulation/tests/unit -m "not simtest" -q` |
| **Simtests (slow, local)** | `simulation/tests/unit/.venv/Scripts/python.exe -m pytest simulation/tests/unit -m simtest -q` |
| All unit+simtests (local) | `simulation/tests/unit/.venv/Scripts/python.exe -m pytest simulation/tests/unit -q` |
| Stack tests (Docker) | `wsl.exe bash -c 'bash /mnt/e/repos/windpower/simulation/dev.sh test-stack'` |
| Stack tests (filtered) | `wsl.exe bash -c 'bash /mnt/e/repos/windpower/simulation/dev.sh test-stack -v -k test_name'` |
| Stack tests (status only) | `wsl.exe bash -c 'bash /mnt/e/repos/windpower/simulation/dev.sh test-stack --filterstatus'` |
| Build Docker image | `simulation\build.cmd ardupilot` |
| Start/stop container | `wsl.exe bash -c 'bash /mnt/e/repos/windpower/simulation/dev.sh start/stop'` |
| Regenerate steady state | `simulation/tests/unit/.venv/Scripts/python.exe -m pytest simulation/tests/unit -k test_steady_flight` |
| **Post-run analysis** | `wsl.exe bash -c 'python3 /mnt/e/repos/windpower/simulation/analysis/analyse_run.py'` |
| Post-run analysis (plot) | `wsl.exe bash -c 'python3 /mnt/e/repos/windpower/simulation/analysis/analyse_run.py --plot'` |
| Last run logs | `simulation/pytest_last_run.log`, `mediator_last_run.log`, `sitl_last_run.log`, `telemetry.csv` |

### Post-run analysis

**Always run `analyse_run.py` after a stack test to understand what happened.** Do not grep through log files manually — the script parses `mediator_last_run.log` and `pytest_last_run.log` and prints a structured report. If the script lacks a feature, modify it directly rather than falling back to ad-hoc grep.

### Running Python scripts

**CRITICAL: Unit tests and simtests run directly on Windows — no WSL, no Docker needed.**
Only stack tests (ArduPilot SITL) require Docker and must go through WSL.

**CRITICAL: Use the Bash tool directly — do NOT use `wsl.exe`, `powershell`, or `cmd /c` for unit tests.**
The Bash tool runs Git Bash on Windows. `/mnt/e/...` WSL paths do NOT exist in Git Bash — never `cd /mnt/e/...`.

| Context | How to run |
|---------|-----------|
| Unit tests (fast) | `simulation/tests/unit/.venv/Scripts/python.exe -m pytest simulation/tests/unit -m "not simtest" -q` |
| Simtests (slow) | `simulation/tests/unit/.venv/Scripts/python.exe -m pytest simulation/tests/unit -m simtest -q` |
| Stack tests (Docker) | `wsl.exe bash -c 'bash /mnt/e/repos/windpower/simulation/dev.sh test-stack [-v] [-k name]'` |
| Any Python analysis script | `wsl.exe bash -c 'python3 /mnt/e/repos/windpower/simulation/...'` |
| One-off inside container | `wsl.exe bash -c 'docker exec rawes-dev python3 /rawes/simulation/...'` |

Windows repo root on WSL path: `/mnt/e/repos/windpower`

---

## ArduPilot ACRO Mode — Design Notes

ACRO mode was chosen because:
- GUIDED's attitude-hold loop fights the tether equilibrium (~65° from vertical) with maximum cyclic
- ACRO only damps angular rates toward commanded RC rates

**⚠️ DO NOT switch to STABILIZE mode.** STABILIZE has been tried and always crashes:
- Holds absolute NED attitude (roll=0=level), which fights the 65° tether equilibrium
- Yaw discontinuity at unfreeze triggers EKF emergency yaw reset → crash within 4 s

**Key problem with neutral sticks in ACRO:** Zero rate command nulls all angular rates including the hub's natural tether-orbit precession → non-zero cyclic → crash. Solution: `compute_rc_from_attitude` sends corrective rates matching the tether-relative attitude error.

**ACRO integrator windup:** ACRO's I-term accumulates the hub's orbital angular velocity (0.2–0.3 rad/s) as a persistent rate error, building ever-growing cyclic tilt. Fix: `ATC_RAT_RLL_IMAX=ATC_RAT_PIT_IMAX=ATC_RAT_YAW_IMAX=0`.

**Sensor consistency (must all agree or EKF triggers emergency yaw reset):**
1. `velocity_ned` heading must match `rpy[2]` — both from `atan2(vE, vN)`
2. `gyro_body` in yaw-aligned NED body frame: spin stripped, then `Rz(-yaw) @ (T @ omega_nospin)`
3. `accel_body`: `Rz(-yaw) @ (T @ accel_world + [0,0,-9.81])`

See [simulation/sim_internals.md](simulation/sim_internals.md) for full sensor/controller design details.

---

## Current Implementation Limits

1. **Single-body hub model** — no blade multibody dynamics, no flapping DOF
2. **Rotor spin is a scalar ODE** — not fully integrated with orbital dynamics; gyroscopic coupling disabled
3. **Tether is tension-only elastic** — no sag, no distributed mass, no reel dynamics
4. **Aerodynamics are first-order BEM** — no dynamic inflow, no wake model, no stall hysteresis
5. **Anti-rotation motor not modeled** — internal force in single-body model; cancels out
6. **Controller runs at 10 Hz in stack test** — ACRO RC override via MAVLink; truth-state controller runs at 400 Hz but only used in unit tests

---

## Phase 3 Plan

Four milestones. M1+M2 complete. M3 in progress. M4 not started.

### M1 — Wire Pumping Cycle into Mediator ✅
- TensionController, orbit_tracked_body_z_eq, blend_body_z → `controller.py`
- Pumping cycle state machine in `mediator.py` (REEL_OUT/REEL_IN phases, winch step, TensionController)
- **Gate:** All unit tests pass, test_acro_hold passes ✓

### M2 — Force Balance Audit & Rotor Abstraction ✅
- `rotor_definition.py` — full RotorDefinition API + YAML files (`beaupoil_2026.yaml`, `de_schutter_2018.yaml`)
- Mediator/aero/dynamics fully wired to rotor definition (no hardcoded rotor constants)
- **Gate:** 265 unit tests pass ✓

### M3 — ArduPilot Configuration & Pumping Cycle Stack Test (in progress)
- [x] Run test_pumping_cycle — PASSED (reel-out 211 N, reel-in 55 N, net energy +1901 J)
- [x] Design `ModeRAWES` firmware architecture — documented in `simulation/raws_mode.md`
- [ ] Confirm H_SWASH_TYPE=1 (H3-120) in SITL
- [ ] Determine H_PHANG via step cyclic → measure tilt response
- [ ] Configure GB4008: H_TAIL_TYPE=4 (DDFP), tune ATC_RAT_YAW_* and H_COL2YAW feedforward
- [ ] Write `rawes_params.parm` (full parameter file for Pixhawk 6C)
- **Gate:** rawes_params.parm exists + H_PHANG determined

### M4 — Hardware-in-the-Loop (Pixhawk 6C)
- [ ] Write `hil_interface.py` — MAVLink HIL_SENSOR / HIL_GPS / HIL_ACTUATOR_CONTROLS
- [ ] Add --hil-mode --hil-port to mediator
- [ ] Confirm IMU mounting orientation (AHRS_ORIENTATION)
- [ ] Write `test_hil_interface.py`
- [ ] Document HIL bench procedure in ARMING.md
- **Gate:** test_hil_interface.py passes + successful 60 s HIL telemetry log
