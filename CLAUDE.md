# RAWES — Project Context for Claude

## Project Goal

Build an **ArduPilot flight controller model** for a Rotary Airborne Wind Energy System (RAWES) that can fly in all standard modes: takeoff, stabilized flight, autonomous flight, landing. This is a long-term, step-by-step effort.

**Current phase:** Closed-loop physics validation. The simulation stack is structurally complete and all physics tests pass. The next step is proving closed-loop stability (controller + dynamics) before investing further in ArduPilot ACRO mode integration.

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

### Unit Tests — Windows native, no Docker

```cmd
py -3 -m venv simulation\tests\unit\.venv
simulation\tests\unit\.venv\Scripts\python.exe -m pip install numpy pytest matplotlib
```

Then run:
```cmd
simulation\tests\unit\.venv\Scripts\python.exe -m pytest simulation\tests\unit
```

With flags:
```cmd
simulation\tests\unit\.venv\Scripts\python.exe -m pytest simulation\tests\unit -v -k test_closed_loop
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
| Unit tests (Windows) | `simulation\tests\unit\.venv\Scripts\python.exe -m pytest simulation\tests\unit` |
| Unit tests (container) | `wsl.exe bash -c '... dev.sh test-unit'` |
| Stack tests | `wsl.exe bash -c '... dev.sh test-stack'` |
| Interactive shell | `wsl.exe bash -c '... dev.sh shell'` |
| Regenerate steady state | `pytest simulation/tests/unit/test_steady_flight.py` |
| Redraw flight report | `wsl.exe bash -c 'python3 /mnt/e/repos/windpower/simulation/analysis/redraw_flight_report.py'` |
| EKF consistency analysis | `wsl.exe bash -c 'python3 /mnt/e/repos/windpower/simulation/analysis/analyse_ekf_consistency.py'` |
| Last run logs | `simulation/pytest_last_run.log`, `mediator_last_run.log`, `sitl_last_run.log`, `gcs_last_run.log`, `telemetry.csv` |

### Running Python scripts (Claude — read this)

**Never** use bare `python3 script.py` or Windows-style paths in Bash tool calls — the shell is WSL bash, which cannot see Windows paths or the Windows Python.

| Context | How to run |
|---------|-----------|
| Any Python script (analysis, one-offs) | `wsl.exe bash -c 'python3 /mnt/e/repos/windpower/simulation/...'` |
| Unit tests (faster, no Docker) | `wsl.exe bash -c 'bash /mnt/e/repos/windpower/simulation/dev.sh test-unit'` |
| Stack tests (Docker required) | `wsl.exe bash -c 'bash /mnt/e/repos/windpower/simulation/dev.sh test-stack'` |
| One-off command inside container | `wsl.exe bash -c 'docker exec rawes-sim python3 /rawes/simulation/...'` |

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

## Next Steps (Planned)

- [x] Confirm battery capacity: 450 mAh (confirmed 2026-03-20)
- [x] Confirm servo model for S1/S2/S3: DS113MG V6.0 Digital Metal Gear Micro Servo
- [x] Define takeoff/landing modes for a tethered autorotating system
- [x] Fix aero.py spin torque model: K_DRIVE_SPIN / K_DRAG_SPIN two-term model
- [x] Replace MBDyn with Python RK4 dynamics (dynamics.py)
- [x] Restructure simulation into clean modules (frames, tether, sensor, controller)
- [x] Closed-loop controller unit tests (test_closed_loop.py)
- [ ] Wire `compute_swashplate_from_state` into mediator loop (bypass ArduPilot servo inputs at 400 Hz)
- [ ] Prove closed-loop stack test passes with truth-state controller
- [ ] Update simulation parameters for 4-blade, 2m geometry
- [ ] ArduPilot helicopter frame configuration for RAWES (swashplate type, RSC mode)
- [ ] Mapping: ArduPilot collective/cyclic outputs → swashplate → flap deflection → blade pitch
- [ ] Hardware-in-the-loop testing with Pixhawk SITL
