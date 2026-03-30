# RAWES — Project Context for Claude

## Project Goal

Build an **ArduPilot flight controller model** for a Rotary Airborne Wind Energy System (RAWES) that can fly in all standard modes: takeoff, stabilized flight, autonomous flight, landing. This is a long-term, step-by-step effort.

**Current phase:** Phase 3, Milestone 3 — Pumping cycle stack test PASSED (`test_pumping_cycle.py`: net energy +1901 J, reel-in 55 N vs reel-out 211 N). Rotor definition abstracted into `rotor_definition.py` + YAML files; mediator/aero/dynamics wired to `beaupoil_2026.yaml`. 265 unit tests passing. Next: write `rawes_params.parm` and ArduPilot hardware frame configuration.

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

> Full physical design detail in [physical_design.md](physical_design.md)

### Assembly Layout (top to bottom)

```
[ rotor hub top bearing ]

  4× blades + trailing-edge flaps   ← no electronics; blades free-pitch within damper
       push-rods (×4, rotate with hub)

  upper swashplate ring              ← rotates with hub; push-rod anchor points
  swashplate bearing
  lower swashplate ring              ← NON-rotating; tilted by S1/S2/S3
  servos S1, S2, S3
  Pixhawk 6C, ESC, GB4008 motor     ← motor counter-torque keeps this stationary
  battery, UBEC, radios

[ rotor hub bottom bearing ]

  axle → tether → ground/winch
```

| Rotates (wind-driven)     | Stationary (GB4008 counter-torque)       |
|---------------------------|------------------------------------------|
| Rotor hub (outer shell)   | Lower swashplate ring                    |
| 4× blades + flaps         | Servos, Pixhawk, ESC, battery, radios    |
| Push-rods                 | Axle                                     |
| Upper swashplate ring     |                                          |

### Key Parameters

| Parameter          | Value         |
|--------------------|---------------|
| Blade count        | 4 (90° apart) |
| Blade length       | 2000 mm       |
| Total rotor radius | ~2500 mm      |
| Rotor mass         | 5 kg          |
| Blade airfoil      | SG6042        |
| Root airfoil       | SG6040        |
| Blade material     | EPP RG30 (foam)|
| Twist              | None          |
| Tether diameter    | 1.9 mm        |
| Max tether length  | 300 m         |
| Min altitude       | 10 m          |
| Tether attachment  | Bottom of axle|

### Pitch Control
Blade pitch is controlled **indirectly** — no pitch bearings at blade root:
```
S1/S2/S3 servos → tilt lower swashplate ring → upper ring follows →
push-rods → trailing-edge flaps → blade aerodynamic twist → pitch change
```
Sign convention: γ < 0 (upward flap) → blade pitches UP

### Anti-Rotation Motor
**EMAX GB4008** (66KV, hollow shaft) via **80:44 spur gear** applies counter-torque to keep the electronics/swashplate assembly non-rotating.

Both bearing friction and push-rod reaction forces are **internal** forces in the current single-body simulation model — they cancel in the equations of motion. Motor torque must **not** be added as an external couple. Motor modeling becomes necessary only when the simulation is split into separate spinning-hub and non-rotating-electronics bodies.

---

## Airfoil Selection — SG6042

Selected over NACA 8H12 and Clark-Y.
- Thickness: 10%, zero lift AoA: −4.92°
- ~2× lift of Clark-Y at operating Reynolds numbers (Re ≈ 127,301)
- No blade twist — AoA variation from rotational position dominates at operating speeds

---

## Known Gaps Between Source Thesis Model And Actual Hardware

| Item | Thesis model | Actual design |
|------|-------------|--------------|
| Blade count | 3 | 4 |
| Blade length | 1.5 m | 2.0 m |
| Cyclic phase offset | 2π/3 (120°) | π/2 (90°) |
| Rotor mass | 40 kg | 5 kg |
| Flap actuation | Individual servo per blade | Swashplate push-rods (mechanical) |
| Anti-rotation | Not modeled | GB4008 + 80:44 gear |

Additional physics limitations in the source thesis model:
- Controller uses fixed system matrices — does not adapt to changing wind
- Tether force not modeled (only stabilizing moment approximated)
- Tip vortices, wake effects, induced drag neglected
- Rotor tilt >15° causes significant pitch tracking error

---

## Electronics & Power

### Power Architecture
```
Battery 4S 15.2V
  ├── XT30 → BM/PM → XT60 → ESC → GB4008 Motor
  └── XT30 → UBEC → 8.0V → Pixhawk servo rail → S1, S2, S3 servos
                         └── Pixhawk FMU (via PM)
```
Note: PM powers FMU only — UBEC must supply the servo rail separately (pin 1 of any servo output).

### Communication Architecture
```
MissionPlanner (PC) ──SiK Radio V3 (433/915MHz)────────► Pixhawk 6C
Boxer M2 RC         ──ExpressLRS 2.4GHz──► RP3-H Rx ───► Pixhawk 6C
Pixhawk 6C          ──PWM (S1, S2, S3)──────────────────► Swashplate servos
Pixhawk 6C          ──DSHOT/PWM──────────► REVVitRC ESC ► GB4008 Motor
```

### Component Summary

| Component        | Model                    | Role                                      |
|------------------|--------------------------|-------------------------------------------|
| Flight controller| Holybro Pixhawk 6C       | Swashplate mixing, servo + ESC outputs    |
| Motor            | EMAX GB4008 (66KV)       | Counter-torque, keeps electronics stationary |
| ESC              | REVVitRC 50A AM32        | Motor speed control                       |
| Servos S1/S2/S3  | DS113MG V6.0             | Swashplate tilt (collective + cyclic)     |
| UBEC             | —                        | 8.0V servo rail power                     |
| Battery          | 4S LiPo 15.2V            | 450 mAh (confirmed)                       |
| Telemetry        | Holybro SiK Radio V3     | MAVLink ground link, 300m+                |
| RC receiver      | RadioMaster RP3-H        | ExpressLRS 2.4GHz, CRSF/S.Bus             |
| RC transmitter   | RadioMaster Boxer M2     | EdgeTX, Hall effect gimbals               |

---

## Reference Documents in This Repo

| File | Contents |
|------|---------|
| [physical_design.md](physical_design.md) | Full physical design — assembly layout, rotor geometry, swashplate, servo flaps, all component specs |
| [flapmodel.md](flapmodel.md) | Full mathematical model from Weyel 2025 — aerodynamics, state space, flap dynamics, controller, all parameter tables |
| [flaprotordesign.md](flaprotordesign.md) | Blade design decisions — airfoil selection (SG6042), dimensions, Reynolds number, flap prototype |
| [servoflaps.md](servoflaps.md) | Kaman flap design reference — US patent US3217809, mechanical principle, swashplate linkage path |
| [hardware.md](hardware.md) | Hardware component specs (detailed) |
| [ardupilot_implementation.md](ardupilot_implementation.md) | ArduPilot integration — swashplate config, Kaman flap lag problem, RSC, companion computer architecture, implementation phases |
| [simulation/ARMING.md](simulation/ARMING.md) | **ArduPilot helicopter arming reference** — RSC modes, motor interlock, EKF init sequence, common failure modes. **Update this file whenever new arming behavior is discovered.** |
| [summary.md](summary.md) | RAWES optimal control model from De Schutter et al. 2018 — pumping cycle, structural constraints, atmosphere model |
| [simulation/mbdyn_reference.md](simulation/mbdyn_reference.md) | MBDyn integration documentation and restoration instructions (MBDyn removed from runtime but archived) |
| [simulation/heliparams.md](simulation/heliparams.md) | **ArduPilot heli EKF3 GPS fusion source analysis** — exact conditions for GPS position fusion, required params (`EK3_SRC1_YAW=1`, `EK3_GPS_CHECK=0`), 10 s mandatory GPS-check delay, `assume_zero_sideslip()=false` for heli. Read this before debugging EKF3 CONST_POS_MODE. |

---

## Background Academic References

1. **Felix Weyel (2025)** — "Modeling and Closed Loop Control of a Cyclic Pitch Actuated Rotary Airborne Wind Energy System", Bachelor's Thesis, Uni Freiburg.
2. **De Schutter, Leuthold, Diehl (2018)** — "Optimal Control of a Rigid-Wing Rotary Kite System for Airborne Wind Energy".
3. **US Patent US3217809** (Kaman/Bossler, 1965) — Canonical servo-flap rotor control system.

---

## Simulation Architecture

### Design Philosophy — Three-Phase Validation

The simulation is structured in three phases. Do not jump to a later phase before the earlier one is proven:

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
├── Analysis tools (one-off scripts, not part of simulation runtime)
│   └── analysis/
│       ├── generate_flight_report.py  Offline report from mediator telemetry CSV
│       ├── redraw_flight_report.py    Regenerate PNG from saved flight_data.json
│       ├── merge_logs.py              Unified log timeline merger
│       ├── analyse_30s_dip.py         Diagnostic for 30 s guided flight dip
│       ├── plot_30s_mechanism.py      Annotated dip mechanism diagram
│       ├── airfoil_gen.py             SG6042 C81 table generator (MBDyn legacy)
│       ├── validate.py                MBDyn syntax validator (legacy)
│       └── build.py                  Docker image build script
│
└── tests/
    ├── unit/                Windows native, no Docker
    │   ├── test_closed_loop.py      ★ Closed-loop physics (dynamics+aero+tether+controller)
    │   ├── test_steady_flight.py    Open-loop equilibrium → writes steady_state_starting.json
    │   ├── test_controller.py       Unit tests for controller.py functions
    │   ├── test_mediator.py         Mediator loop with fake SITL/dynamics
    │   ├── test_mediator_transport.py  Real UDP sockets, fake dynamics
    │   ├── test_aero.py             Aerodynamic model
    │   ├── test_sensor.py           Sensor conversion
    │   ├── test_swashplate.py       Swashplate mixing
    │   ├── test_interfaces.py       SITL interface
    │   └── ...
    └── stack/               Docker required
        ├── conftest.py              acro_armed fixture (full stack lifecycle)
        ├── test_guided_flight.py    60 s ACRO hold with tether-alignment RC controller
        ├── test_setup.py            Verifies setup reaches armed ACRO
        ├── test_arm_minimal.py      Arm sequence test (no mediator)
        └── test_stack_integration.py  Process launch helpers
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

**Orbital frame:** `build_orb_frame(body_z)` from `frames.py`. Removes rotor spin; body X = East projected onto disk plane. Used to build R0 from body_z and to compute tether-relative attitude in `sensor.py`.

---

## Natural / Equilibrium Hub Orientation

**The rotor axle (body Z) always aligns with the tether direction.** At tether elevation angle β:
```
body Z = [cos(β), 0, sin(β)]   (ENU, hub East of anchor)
```

Always initialise the hub with body Z along the tether, not upright. The `build_orb_frame(body_z)` utility constructs a valid R0 from any body_z unit vector.

---

## Sensor Design — Tether-Relative Attitude

**Critical design decision:** attitude is reported as the **deviation from the tether equilibrium orientation**, not absolute orientation relative to world Up.

**Why:** The RAWES equilibrium has body Z at ~30° elevation — ~65° from vertical. Reporting absolute attitude causes ArduPilot to see 58° roll / 36° pitch at rest and command maximum cyclic, crashing the hub within 1 second.

**How** (`sensor.build_sitl_packet`):
- `rpy = [0, 0, yaw_from_velocity]` — roll and pitch are always zero (tether-relative)
- yaw is derived from velocity heading so EKF attitude and velocity are consistent
- gyro has spin stripped; rotated into yaw-aligned body frame
- accel is kinematic accel in NED body frame plus −g correction

**EKF consequence:** ArduPilot interprets deviations from tether equilibrium as small roll/pitch errors and corrects with modest cyclic, not maximum deflection.

---

## Controller Design

`simulation/controller.py` provides three functions with different use cases:

### `compute_swashplate_from_state(hub_state, anchor_pos, ...)` — truth-state controller

Takes hub ENU state (pos, R, omega) directly. Computes attitude error as `cross(body_z_cur, body_z_eq)` and damps orbital rates. Returns `{collective_rad, tilt_lon, tilt_lat}` to feed directly into `aero.compute_forces()`. **No ArduPilot involved.**

Used by: `test_closed_loop.py` (closed-loop physics validation)

### `compute_rc_rates(hub_state, anchor_pos, vel_ned, ...)` — ENU truth-state RC controller

Same logic as above but outputs RC PWM dict `{1: pwm, 2: pwm, 4: pwm, 8: 2000}` for ArduPilot's ACRO rate loop. Converts correction to yaw-aligned NED body frame.

### `compute_rc_from_attitude(roll, pitch, rollspeed, pitchspeed, yawspeed, ...)` — MAVLink controller

Works directly with ArduPilot ATTITUDE message fields. Since `sensor.build_sitl_packet` reports tether-relative attitude, `roll` and `pitch` ARE the attitude error angles. Returns RC PWM dict.

Used by: `test_guided_flight.py` (ACRO hold loop via RC override, ~10 Hz from MAVLink)

**Important limitation:** The ACRO hold controller runs at ~10 Hz via MAVLink messages. The physics diverges at 400 Hz. The truth-state controller (`compute_swashplate_from_state`) operates at full 400 Hz and is the correct long-term architecture.

---

## Dynamics Model

`simulation/dynamics.py` — RK4 6-DOF rigid-body integrator.

| Parameter | Value | Location |
|-----------|-------|----------|
| Timestep | 2.5e-3 s (400 Hz) | `DT_TARGET` in mediator.py |
| Mass | 5.0 kg | mediator.py |
| Ixx = Iyy | 5.0 kg·m² | mediator.py |
| Izz | 10.0 kg·m² | mediator.py |
| I_spin | 0.0 | mediator.py (gyroscopic coupling disabled — see comment) |
| Initial pos | `[46.258, 14.241, 12.530]` ENU m | `DEFAULT_POS0` |
| Initial vel | `[-0.257, 0.916, -0.093]` m/s | `DEFAULT_VEL0` |
| Initial body_z | `[0.851, 0.305, 0.427]` | `DEFAULT_BODY_Z` |
| Initial spin | 20.148 rad/s | `DEFAULT_OMEGA_SPIN` |

Gravity is applied internally — do **not** add it to forces.

**Why I_spin = 0:** The lumped single-body model uses ArduPilot's cyclic commands as tilting moments. The 90° gyroscopic precession a real spinning rotor produces is already compensated by swashplate phase angle in real hardware. Adding gyroscopic coupling here without the corresponding phase compensation would cause ArduPilot's attitude controller to tilt the disk in the wrong direction.

**Rotor spin** is maintained as a separate scalar `omega_spin`, updated each step via:
```
Q_net = K_DRIVE_SPIN × v_inplane − K_DRAG_SPIN × omega_spin²
omega_spin += Q_net / I_SPIN_KGMS2 × dt
```
This gives a stable equilibrium at `omega_eq = sqrt(K_DRIVE × v_inplane / K_DRAG)`.

---

## Aerodynamic Model

`simulation/aero.py` — De Schutter (2018) lumped-blade BEM model.

**What it does:**
- Computes relative wind = ambient wind − hub velocity
- Estimates induced velocity via actuator-disk momentum theory
- Integrates lift and drag over 20 radial strips, root to tip
- CL = CL0 + CL_alpha × AoA; CD = CD0 + CL²/(π·AR·Oe)
- AoA clamped to ±15° for linear model validity
- Produces: thrust (along disk normal), drag torque (about disk axis), cyclic moments (proportional to swashplate tilt)
- 5 s startup ramp to avoid impulsive loads

**What it does not do:** dynamic inflow, tip-loss, wake swirl, flap-lag coupling, tether aero, true flap-to-pitch dynamics.

`compute_anti_rotation_moment()` exists but is **not called** in the current single-body model — motor torque is an internal force and cancels. Reserved for future two-body model.

---

## Tether Model

`simulation/tether.py` — `TetherModel` class.

| Property | Value |
|----------|-------|
| Material | Dyneema SK75 1.9 mm braided UHMWPE |
| EA (axial stiffness) | ~281 kN |
| k(L) | EA / L — nonlinear, stiffer when shorter |
| Linear mass | 2.1 g/m |
| Break load | ~620 N |
| Structural damping | 5 N·s/m (extension only) |
| Rest length | 49.949 m default (taut at steady-state equilibrium) |
| Anchor | World origin (0, 0, 0) ENU |

When tether is slack (hub closer than rest length) → zero force. Logs warning when tension exceeds 80% of break load.

Restoring moment (`r_attach × F_tether`) is supported but currently disabled (`axle_attachment_length=0.0`) — body_z stability comes from aerodynamics, not tether moment.

---

## Initial State and `steady_state_starting.json`

The mediator's default initial state is the warmup-settled equilibrium from `test_steady_flight.py`:

| Parameter | Value | Source |
|-----------|-------|--------|
| `pos0` | `[46.258, 14.241, 12.530]` ENU m | 50 m tether at ~30° elevation |
| `vel0` | `[-0.257, 0.916, -0.093]` m/s | settled hub velocity |
| `body_z` | `[0.851, 0.305, 0.427]` | axle aligned with tether |
| `omega_spin` | `20.148` rad/s | equilibrium autorotation spin |
| `rest_length` | `49.949` m | tether taut from t=0 |

**Workflow for regenerating:**
1. `pytest simulation/tests/unit/test_steady_flight.py` — writes `simulation/steady_state_starting.json`
2. Stack test reads this file and passes values to mediator via `--pos0`, `--vel0`, `--body-z`, `--omega-spin`
3. If absent, mediator uses the built-in defaults above

---

## Startup Freeze

The mediator holds the hub stationary for `--startup-freeze-seconds` (default: 30 s) so the ArduPilot EKF achieves GPS position lock before physics starts.

During freeze:
- Hub state is not updated
- A 0.15 m/s hint velocity is sent in the equilibrium heading direction so the EKF converges yaw before physics starts (prevents "Yaw Imbalance" / emergency yaw reset when freeze ends)
- Real position is sent (no drift)

---

## Running Tests

### Unit Tests and Simtests — Windows native, no Docker

**CRITICAL: Only ArduPilot/stack tests require Docker. Unit tests and simtests run directly on Windows using the local venv.**

One-time venv setup:
```cmd
py -3 -m venv simulation\tests\unit\.venv
simulation\tests\unit\.venv\Scripts\python.exe -m pip install numpy pytest matplotlib
```

Fast unit tests (excludes long-running simulation loops):
```cmd
simulation\tests\unit\.venv\Scripts\python.exe -m pytest simulation\tests\unit -m "not simtest"
```

Simtests only (full 60 s physics simulation loops — slow, ~minutes):
```cmd
simulation\tests\unit\.venv\Scripts\python.exe -m pytest simulation\tests\unit -m simtest
```

All tests (unit + simtest):
```cmd
simulation\tests\unit\.venv\Scripts\python.exe -m pytest simulation\tests\unit
```

With flags:
```cmd
simulation\tests\unit\.venv\Scripts\python.exe -m pytest simulation\tests\unit -m "not simtest" -v -k test_closed_loop
```

### Stack Integration Tests — Docker via WSL

Stack tests launch SITL + mediator inside the `rawes-sim` Docker container and drive a full flight sequence via MAVLink. All Docker commands must go through `wsl.exe bash -c '...'`.

**Build image (one-time, ~30 min):**
```cmd
simulation\build.cmd ardupilot
```

**Manage container and run tests:**
```cmd
wsl.exe bash -c 'bash /mnt/e/repos/windpower/simulation/dev.sh start'
wsl.exe bash -c 'bash /mnt/e/repos/windpower/simulation/dev.sh stop'
wsl.exe bash -c 'bash /mnt/e/repos/windpower/simulation/dev.sh test-unit'
wsl.exe bash -c 'bash /mnt/e/repos/windpower/simulation/dev.sh test-stack'
wsl.exe bash -c 'bash /mnt/e/repos/windpower/simulation/dev.sh shell'
```

Extra pytest args are forwarded:
```cmd
wsl.exe bash -c 'bash /mnt/e/repos/windpower/simulation/dev.sh test-stack -v -k test_guided_flight'
```

### Quick Reference

| Task | Command |
|------|---------|
| Build Docker image | `simulation\build.cmd ardupilot` |
| **Unit tests (fast, local)** | `simulation/tests/unit/.venv/Scripts/python.exe -m pytest simulation/tests/unit -m "not simtest" -q` |
| **Simtests (slow, local)** | `simulation/tests/unit/.venv/Scripts/python.exe -m pytest simulation/tests/unit -m simtest -q` |
| All unit+simtests (local) | `simulation/tests/unit/.venv/Scripts/python.exe -m pytest simulation/tests/unit -q` |
| Stack tests (Docker) | `wsl.exe bash -c '... dev.sh test-stack'` |
| Stack tests (filtered) | `wsl.exe bash -c '... dev.sh test-stack -v -k test_name'` |
| Stack tests (status only) | `wsl.exe bash -c '... dev.sh test-stack --filterstatus'` |
| Simtests (container) | `wsl.exe bash -c '... dev.sh test-simtest'` |
| Interactive shell | `wsl.exe bash -c '... dev.sh shell'` |
| Regenerate steady state | `simulation\tests\unit\.venv\Scripts\python.exe -m pytest simulation\tests\unit -k test_steady_flight` |
| Redraw flight report | `wsl.exe bash -c 'python3 /mnt/e/repos/windpower/simulation/analysis/redraw_flight_report.py'` |
| EKF consistency analysis | `wsl.exe bash -c 'python3 /mnt/e/repos/windpower/simulation/analysis/analyse_ekf_consistency.py'` |
| **Post-run analysis** | `wsl.exe bash -c 'python3 /mnt/e/repos/windpower/simulation/analysis/analyse_run.py'` |
| Post-run analysis (plot) | `wsl.exe bash -c 'python3 /mnt/e/repos/windpower/simulation/analysis/analyse_run.py --plot'` |
| Last run logs | `simulation/pytest_last_run.log`, `mediator_last_run.log`, `sitl_last_run.log`, `gcs_last_run.log`, `telemetry.csv` |

### Post-run analysis (Claude — read this)

**Always run `analyse_run.py` after a stack test to understand what happened.**  Do not grep through log files manually — the script parses `mediator_last_run.log` and `pytest_last_run.log` and prints a structured report covering damping phase stability, EKF timeline, setup step progress, hold stats, and test outcomes.

If `analyse_run.py` lacks a feature you need, or a parser is broken, **modify `simulation/analysis/analyse_run.py` directly** — do not fall back to ad-hoc grep commands.

```
wsl.exe bash -c 'python3 /mnt/e/repos/windpower/simulation/analysis/analyse_run.py'
wsl.exe bash -c 'python3 /mnt/e/repos/windpower/simulation/analysis/analyse_run.py --plot'
wsl.exe bash -c 'python3 /mnt/e/repos/windpower/simulation/analysis/analyse_run.py --all-statustext'
```

### Running Python scripts (Claude — read this)

**CRITICAL: Unit tests and simtests run directly on Windows — no WSL, no Docker needed.**
Only stack tests (ArduPilot SITL) require Docker and must go through WSL.

**CRITICAL: Use the Bash tool directly — do NOT use `wsl.exe`, `powershell`, or `cmd /c` for unit tests.**
The Bash tool runs Git Bash on Windows with the working directory already set to `E:\repos\windpower`.
Python can be called directly with forward-slash paths. `/mnt/e/...` WSL paths do NOT exist in Git Bash —
never `cd /mnt/e/...`.

**Correct pattern for unit tests:**
```
simulation/tests/unit/.venv/Scripts/python.exe -m pytest simulation/tests/unit -m "not simtest" -q
```

| Context | How to run |
|---------|-----------|
| Unit tests (fast) | `simulation/tests/unit/.venv/Scripts/python.exe -m pytest simulation/tests/unit -m "not simtest" -q` |
| Simtests (slow, ~minutes) | `simulation/tests/unit/.venv/Scripts/python.exe -m pytest simulation/tests/unit -m simtest -q` |
| Stack tests (Docker required) | `wsl.exe bash -c 'bash /mnt/e/repos/windpower/simulation/dev.sh test-stack [-v] [-k name] [--filterstatus]'` |
| Any Python analysis script | `wsl.exe bash -c 'python3 /mnt/e/repos/windpower/simulation/...'` |
| One-off command inside container | `wsl.exe bash -c 'docker exec rawes-dev python3 /rawes/simulation/...'` |

Windows repo root on WSL path: `/mnt/e/repos/windpower`

---

## ArduPilot ACRO Mode — Design Notes

ACRO mode was chosen because:
- GUIDED's attitude-hold loop fights the tether equilibrium (~65° from vertical) with maximum cyclic
- ACRO only damps angular rates toward commanded RC rates

**⚠️ DO NOT switch to STABILIZE mode.** STABILIZE has been tried and always crashes:
- STABILIZE holds absolute NED attitude (roll=0=level), which fights the 65° tether equilibrium just like GUIDED
- Even with tether-relative attitude reporting (roll=pitch=0 at equilibrium), STABILIZE's yaw hold requires a consistent EKF yaw
- Yaw is derived from velocity heading; at zero or near-zero initial velocity, yaw defaults to 0° then jumps discontinuously when velocity builds → EKF emergency yaw reset → crash within 4 s
- The ACRO + `compute_rc_from_attitude` approach is the correct path

**Key problem with neutral sticks in ACRO:** Zero rate command nulls *all* angular rates including the hub's natural tether-orbit precession → non-zero cyclic → hub crashes. The solution is `compute_rc_from_attitude` which sends corrective rates matching the tether-relative attitude error.

**ACRO integrator windup:** ACRO's rate-controller I-term accumulates the hub's orbital angular velocity (0.2–0.3 rad/s) as a persistent rate error, building ever-growing cyclic tilt until the hub crashes. Fix: set `ATC_RAT_RLL_IMAX=ATC_RAT_PIT_IMAX=ATC_RAT_YAW_IMAX=0` to zero the I-term limits.

**EKF yaw in ACRO:** EKF yaw is irrelevant to ACRO's inner-loop rate control. ACRO uses raw gyro directly. Do not invest in EKF yaw debugging for ACRO stability — it is a different control path.

**Sensor consistency rules (must all agree or EKF triggers emergency yaw reset):**
1. `velocity_ned` heading must match `rpy[2]` (yaw) — both derived from `atan2(vE, vN)`
2. `gyro_body` must be in yaw-aligned NED body frame (spin stripped, then `Rz(-yaw) @ (T @ omega_nospin)`)
3. `accel_body` must be `Rz(-yaw) @ (T @ accel_world + [0,0,-9.81])`

---

## Current Implementation Limits

1. **Single-body hub model** — no blade multibody dynamics, no flapping DOF
2. **Rotor spin is a scalar ODE** — not fully integrated with orbital dynamics; gyroscopic coupling disabled
3. **Tether is tension-only elastic** — no sag, no distributed mass, no reel dynamics
4. **Aerodynamics are first-order BEM** — no dynamic inflow, no wake model, no stall hysteresis
5. **Anti-rotation motor not modeled** — internal force in single-body model; cancels out
6. **Controller runs at 10 Hz in stack test** — ACRO RC override via MAVLink; truth-state controller (`compute_swashplate_from_state`) runs at 400 Hz but only used in unit tests currently

---

## Phase 2 — Internal Controller: What Was Accomplished

### Bug fixed: `compute_swashplate_from_state` tilt_lon sign error

The longitudinal cyclic correction was applied with the wrong sign.  The aero
model applies `Mx_body = -K × tilt_lon × T` (negative sign), so a positive
`tilt_lon` produces a westward torque that tilts body_z **north**.  The
controller was projecting the error onto `disk_x` (eastward), giving
`tilt_lon < 0` when the hub needed to tilt north — the exact opposite.  Fixed
by negating the projection: `tilt_lon = -dot(corr_enu, disk_x) / tilt_max_rad`.

The tilt_lat sign was already correct (aero uses `My_body = +K × tilt_lat × T`).

### 60 s closed-loop stability proven (`test_closed_loop_60s.py`)

Four tests, all passing:
- `test_60s_altitude_maintained` — 0 floor hits, z rises from 12.5 m → 13.3 m over 60 s
- `test_60s_no_runaway` — hub drift stays below 200 m
- `test_60s_spin_maintained` — autorotation spin 20→21 rad/s (sustained)
- `test_zero_tilt_at_equilibrium` — controller outputs exactly zero tilt at POS0/BODY_Z0

Key design decisions validated:
- `axle_attachment_length=0.0` matches mediator (no tether restoring torque; attitude via aero only)
- `T_AERO_OFFSET=45.0` — aero ramp already complete at kinematic-phase end; passed as `t=45+t`
- **Orbit-tracking body_z_eq**: body_z_eq rotates azimuthally as hub orbits, keeping error=0 at
  equilibrium throughout the orbit.  Without this, body_z_eq drifts from the hub's actual
  aerodynamic equilibrium as it orbits and the controller generates ever-growing spurious tilt.

### Orbit-tracking formula (used in mediator and all unit tests)

```python
# Capture at free-flight t=0:
tether_dir0 = pos0 / |pos0|
body_z_eq0  = R0[:, 2]

# Each step:
th0h = [tether_dir0.x, tether_dir0.y, 0]  (horizontal projection of initial tether dir)
thh  = [cur_tdir.x,    cur_tdir.y,    0]  (horizontal projection of current tether dir)
# normalise both, compute azimuthal rotation angle phi
cos_phi = dot(th0h, thh)
sin_phi = th0h.x * thh.y - th0h.y * thh.x
body_z_eq = rotate_z(body_z_eq0, phi)    # rotate initial eq body_z by orbital azimuth
```

### Key physics insight: zero tilt = natural stability

With `axle_attachment_length=0.0` and the aero ramp complete:
- Zero tilt → `M_orbital = M_total - M_spin = 0` → no orbital moment → hub is perfectly stable
- The hub needs **no controller at all** to maintain equilibrium — the aerodynamic equilibrium
  is self-stable.  The controller only corrects perturbations.

The open-loop diagnostic (`_diag2.py`) confirmed: 60 s, 0 floor hits, body_z_z=0.427
constant, z=12.82 m constant, spin=20.3 rad/s constant — identical to the closed-loop result.

### Pumping cycle — naive collective-only approach does not work

`test_pumping_cycle.py` tests a collective-only strategy (no tilt change between phases):
- Reel-out mean tension: 261 N, reel-in mean tension: 288 N → reel-in > reel-out → net energy negative
- Root cause: tether tension during reel-in equals the force to pull hub inward against its
  aerodynamic equilibrium, which is dominated by the full aerodynamic force regardless of collective.
- Collective modulation alone cannot achieve the asymmetry needed for net positive energy.

### Pumping cycle — De Schutter tilt strategy works (`test_deschutter_cycle.py`)

Implements De Schutter (2018) Fig. 4: tilt ξ from wind direction changes between phases.

**Reel-out:** body_z aligned with tether, ξ ≈ 31° from wind, high collective via PI controller
- Mean tension: 233 N, max: 270 N, energy generated: 2791 J

**Reel-in:** body_z transitions to vertical [0,0,1] over 5 s (ξ = 90° from wind, >70° criterion met)
- Thrust acts upward (fighting gravity), not along tether → aerodynamic resistance to winch ≈ 0
- Tension drops to mean: 55 N (winch only overcomes gravity component along tether ~17 N + inertia)
- Energy consumed: 854 J

**Net energy: +1937 J** — fundamental AWE pumping condition satisfied.

Five tests all passing:
- `test_deschutter_no_crash` — 0 floor hits throughout
- `test_deschutter_reel_in_lower_tension` — 55 N < 233 N ✓
- `test_deschutter_net_energy_positive` — +1937 J ✓
- `test_deschutter_reel_in_tilt_achieved` — steady ξ = 90° ≥ 60° ✓
- `test_deschutter_reel_out_tilt_in_range` — ξ = 31° within 25–60° ✓
- `test_deschutter_tether_not_broken` — peak 270 N < 496 N (80% break load) ✓

### Architecture: how the pumping cycle is controlled

Three independent control loops, all at 400 Hz:
```
Attitude controller  →  tilt_lon, tilt_lat   (compute_swashplate_from_state)
                         body_z_eq = tether-aligned (reel-out) or vertical (reel-in)
Tension controller   →  collective_rad        (TensionController PI on tether tension)
Winch controller     →  tether.rest_length    (±= v_reel × DT each step)
```

The **attitude controller's body_z_eq setpoint is the primary lever** for tension control,
not collective.  Collective provides fine-grained tension tuning within a phase.

### Mediator wiring (completed in this session)

`mediator.py` was updated with orbit-tracking state captured at free-flight start:
- `_ic_tether_dir0` and `_ic_body_z_eq0` captured once when `_damp_alpha == 0.0` first triggers
- Body_z_eq tracked azimuthally each step (same formula as unit tests)
- Internal controller branch calls `compute_swashplate_from_state` with orbit-tracked body_z_eq
- collective_rad = 0.0 (pumping cycle collective control not yet wired into mediator)

---

## Next Steps (Planned)

- [x] Confirm battery capacity: 450 mAh (confirmed 2026-03-20)
- [x] Confirm servo model for S1/S2/S3: DS113MG V6.0 Digital Metal Gear Micro Servo
- [x] Define takeoff/landing modes for a tethered autorotating system
- [x] Fix aero.py spin torque model: K_DRIVE_SPIN / K_DRAG_SPIN two-term model
- [x] Replace MBDyn with Python RK4 dynamics (dynamics.py)
- [x] Restructure simulation into clean modules (frames, tether, sensor, controller)
- [x] Closed-loop controller unit tests (test_closed_loop.py)
- [x] Fix tilt_lon sign bug in compute_swashplate_from_state (was destabilising; negated projection)
- [x] Prove 60 s closed-loop stability with orbit-tracking controller (test_closed_loop_60s.py)
- [x] Wire orbit-tracking internal controller into mediator.py
- [x] Validate De Schutter pumping cycle in unit tests (test_deschutter_cycle.py)
- [x] Prove closed-loop stack test passes with internal controller (test_acro_hold PASSED: min alt 10 m, max drift 30 m in 60 s)
- [x] Wire pumping cycle (TensionController + body_z_eq tilt strategy) into mediator
- [x] Abstract rotor definition into YAML + API (rotor_definition.py, beaupoil_2026.yaml, de_schutter_2018.yaml)
- [x] Wire rotor definition into mediator/aero/dynamics (replaces all hardcoded rotor constants)
- [x] Physical model validation tests (test_physical_validation.py — 27 tests)
- [x] Write pumping cycle stack test (test_pumping_cycle.py)
- [x] **Run test_pumping_cycle — PASSED** (reel-out 211 N, reel-in 55 N, net energy +1901 J, peak 341 N, min alt 8.98 m)
- [ ] Write rawes_params.parm (full ArduPilot parameter file for Pixhawk 6C)
- [ ] ArduPilot helicopter frame configuration for RAWES (H_SWASH_TYPE, H_PHANG, H_TAIL_TYPE)
- [ ] Hardware-in-the-loop testing with Pixhawk SITL (hil_interface.py)

---

## Phase 3 Plan

Four milestones. M1+M2 are complete. M3 is in progress. M4 not started.

### M1 — Wire Pumping Cycle into Mediator ✅
- TensionController, orbit_tracked_body_z_eq, blend_body_z → `controller.py`
- Pumping cycle state machine in `mediator.py` (REEL_OUT/REEL_IN phases, winch step, TensionController)
- Telemetry columns: pumping_phase, tether_rest_length, tension_setpoint, collective_from_tension_ctrl
- **Gate:** All unit tests pass, test_acro_hold passes ✓

### M2 — Force Balance Audit & Rotor Abstraction ✅
- Equilibrium collective confirmed ~−5.7° (rotor over-generates lift; tether balances)
- `test_force_balance.py` passes (10 tests)
- `rotor_definition.py` — full RotorDefinition API with validation, derived geometry, aero/dynamics factory helpers
- `rotor_definitions/beaupoil_2026.yaml` — actual hardware spec (4 blades, 2 m, 5 kg, SG6042, Kaman flaps estimated)
- `rotor_definitions/de_schutter_2018.yaml` — thesis reference (3 blades, 1.5 m, 40 kg)
- `test_rotor_definition.py` — 67 tests; `test_physical_validation.py` — 27 tests
- Mediator/aero/dynamics fully wired to rotor definition (no hardcoded rotor constants)
- **Gate:** 265 unit tests pass ✓

### M3 — ArduPilot Configuration & Pumping Cycle Stack Test (in progress)
- [x] `test_pumping_cycle.py` written (stack test — mirrors test_deschutter_cycle.py)
- [ ] Run test_pumping_cycle — confirm reel-in tension < reel-out, net energy > 0
- [ ] Confirm H_SWASH_TYPE=1 (H3-120) in SITL
- [ ] Determine H_PHANG via step cyclic → measure tilt response
- [ ] Configure GB4008 tail: H_TAIL_TYPE, SERVO4_TRIM, yaw rate PID
- [ ] Write `rawes_params.parm` (full parameter file for Pixhawk 6C)
- **Gate:** test_pumping_cycle passes + rawes_params.parm exists

### M4 — Hardware-in-the-Loop (Pixhawk 6C)
- [ ] Write `hil_interface.py` — MAVLink HIL_SENSOR / HIL_GPS / HIL_ACTUATOR_CONTROLS
- [ ] Add --hil-mode --hil-port to mediator
- [ ] Confirm IMU mounting orientation (AHRS_ORIENTATION)
- [ ] Write `test_hil_interface.py` (unit test with fake MAVLink)
- [ ] Document HIL bench procedure in ARMING.md
- **Gate:** test_hil_interface.py passes + successful 60 s HIL telemetry log
