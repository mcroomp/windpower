# RAWES — Project Context for Claude

## Project Goal

Build an **ArduPilot flight controller model** for a Rotary Airborne Wind Energy System (RAWES) that can fly in all standard modes: takeoff, stabilized flight, autonomous flight, landing. This is a long-term, step-by-step effort. **Current phase: knowledge collection and organization.**

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
- **B — Reference pitch → flap actuation:** Feed-forward + PID — implemented in the source thesis code, not yet in the current MBDyn/SITL runtime stack
- **C — Physics simulation:** Full nonlinear ODE state space model — implemented in the source thesis code, separate from the current MBDyn rigid-body simulation

The ArduPilot integration sits at level A (trajectory planning) and will delegate to level B for actuation.

---

## Physical System

> Full physical design detail in [physical_design.md](physical_design.md)

### Assembly Layout (top to bottom)

The rotor hub is the outer rotating shell. The swashplate and all electronics are sandwiched inside it, with the rotor hub running on bearings above and below the swashplate assembly. The axle runs through the center top to bottom; the tether attaches at the bottom of the axle.

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
**EMAX GB4008** (66KV, hollow shaft) via **80:44 spur gear** applies counter-torque to keep the electronics/swashplate assembly non-rotating. It does not drive rotation.

The motor must overcome two load sources:
1. **Bearing friction** — the hub bearings transmit a drag torque from the spinning rotor to the stationary inner assembly (lower swashplate ring, servos, electronics).
2. **Push-rod reaction forces** — during cyclic pitch operation, the push-rods alternately compress/extend as each blade passes through the tilt plane. The aerodynamic and inertial forces on the flaps react back through the push-rods onto the lower swashplate ring, creating a net torque the motor must hold against.

**Single-body simulation note:** In the current MBDyn model the hub and electronics are represented as one lumped rigid body. Both of the above loads are therefore *internal* forces between parts of the same body — they cancel in the equations of motion. The motor torque must **not** be added as an external couple in the single-body MBDyn model. When the model is eventually split into two bodies (spinning hub + non-rotating electronics), these internal loads become external coupling forces that the motor torque must balance explicitly.

---

## Airfoil Selection — SG6042

Selected over NACA 8H12 (reflexed TE, flutter risk) and Clark-Y.

- Thickness: 10%, zero lift AoA: −4.92°
- ~2× lift of Clark-Y at operating Reynolds numbers
- Designed for low-Re: Re ≈ 127,301 at operating conditions
- No blade twist — AoA variation from rotational position dominates at operating speeds

---

## Mathematical Model

The simulation model (from Weyel 2025 thesis) was developed on a 3-blade, 1.5m blade version. Parameters below are the thesis values — **will need updating for 4-blade, 2m blade geometry.**

### Simulation Parameters (Weyel 2025 — 3-blade reference)

**RAWES dynamics:**
| Symbol | Value | Unit | Note |
|--------|-------|------|------|
| ρ | 1.22 | kg/m³ | |
| g | 9.81 | m/s² | |
| \|\|v_w\|\| | 10 | m/s | |
| Blade span | 1.5 | m | **→ actual: 2.0 m** |
| Blade area S | 0.2 | m² | needs update |
| CL,0 | 0.11 | — | |
| CL,α | 0.87 | — | |
| CD,0 | 0.007 | — | |
| CD,α | 0.00013 | — | |
| Rotor mass m | 40 | kg | **→ actual: 5 kg** |
| Rotor inertia Jr | [20,20,40]·I | kg·m² | needs update |
| Damping D | 50 | N·m·s | |
| CoM offset d | -1 | m | |
| ω_θ | 15π | rad/s | |
| κ_R (Baumgarte) | 0.2 | — | |

> ω_θ = 15π → tip speed ≈ 70 m/s → tip speed ratio λ = 7 at 10 m/s wind

**Flap dynamics:**
| Symbol | Value | Unit |
|--------|-------|------|
| Chord l | 0.15 | m |
| CM,0 | -0.08 | — |
| CM,α | 0.002 | — |
| CM,γ | -0.5 | — |
| Blade inertia Jb | 1.20 | kg·m² |
| Damping Dω | -13.5 | N·m·s |

**PID controller:**
| Kp | KI | KD |
|----|----|----|
| 2  | 3  | 0.2 |

### State Space

Full RAWES state vector:
```
x_r(t) = [p(t), v(t), R(t), ω_R(t), θ(t), ω_θ]ᵀ
```

Input: `u_r(t) = [β₁, β₂, β₃, β₄]` (pitch angles of 4 blades)

Flap dynamics (per blade):
```
ẋ_f,i = A · x_f,i + B · γ_i

A = | 0                             1  |
    | q∞·S·l·(CM,β+CM,γ)·Jb⁻¹     Dω |

B = | 0                   |
    | q∞·S·l·CM,γ·Jb⁻¹   |
```

### Cyclic Pitch Signal

For **4 blades** (90° apart), tilt rotor in direction Ψ with amplitude U0:
```
βi(t) = U0 · sin(ω_θ·t + (i-1)·π/2 + Ψ)    i = 1,2,3,4
```

Phase offset is **π/2 (90°)** for 4 blades — previously 2π/3 (120°) for 3 blades.

### Numerical Integration
- Method: RK4
- RAWES model: Δt = 0.001 s
- Flap model: Δt = 0.013 s
- Implementation: Python, NumPy + Matplotlib

### Key Design Constraints
| Constraint | Value |
|-----------|-------|
| Max rotor tilt for good tracking | ≤ 15° |
| Min tip speed ratio | λ ≥ 7 |
| AoA for linear model validity | < 15–20° |
| Controller settling time | ~1 s |
| PID RMSE (open → closed loop) | 9.693° → 2.610° |

---

## Electronics & Power

> Full component specs in [physical_design.md](physical_design.md)

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
| [summary.md](summary.md) | RAWES optimal control model from De Schutter et al. 2018 — pumping cycle, structural constraints, atmosphere model |
| [simulation_model.md](simulation_model.md) | Duplicate of flapmodel.md |

---

## Background Academic References

1. **Felix Weyel (2025)** — "Modeling and Closed Loop Control of a Cyclic Pitch Actuated Rotary Airborne Wind Energy System", Bachelor's Thesis, Uni Freiburg. Source reference for flap-control and modeling assumptions; not the current runtime simulation implementation in this repo.

2. **De Schutter, Leuthold, Diehl (2018)** — "Optimal Control of a Rigid-Wing Rotary Kite System for Airborne Wind Energy". Full system optimization including pumping cycle, structural constraints, atmosphere model.

3. **US Patent US3217809** (Kaman/Bossler, 1965) — Canonical servo-flap rotor control system.

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

## Flight Modes

### Takeoff — Jump Takeoff (Gyrocopter-style)

RAWES uses a jump takeoff, exploiting the rotor's stored kinetic energy for vertical launch:

1. **Pre-rotation:** The rotor rests on a launch arm mounted on a gimbal. The bottom of the rotor hub is cone-shaped; this cone engages a motorized pre-rotator shaft that spins the rotor to an RPM significantly above what is needed for level autorotation. Blades are held at **zero collective pitch** during this phase to minimize aerodynamic drag on the motor.
2. **Jump:** Collective pitch is rapidly increased. This generates a spike in lift at the cost of decaying rotor RPM — the stored rotational kinetic energy is converted into vertical thrust, pushing the rotor upward.
3. **Transition to autorotation:** As the rotor climbs to altitude, it transitions from consuming stored kinetic energy to sustained autorotation driven by the relative wind. Tether pays out as the rotor ascends.

Key constraint: the pre-rotator shaft must disengage cleanly before autorotation establishes, and the gimbal launch arm must release without disturbing rotor attitude.

### Landing — Autorotation Landing (Tether-assisted)

Landing resembles a helicopter autorotation flare, with the tether providing additional control authority:

1. **Descent:** The rotor descends in autorotation. Tether reel-in by the ground winch maintains relative wind across the rotor disk, sustaining autorotation throughout the descent — this is a key difference from an untethered gyrocopter.
2. **Flare:** Just before the rotor cone touches the pre-rotator shaft, collective pitch is increased. This reduces sink rate and bleeds off rotor RPM in a controlled flare.
3. **Touch-down / RPM match:** The pre-rotator shaft motor spins up to match the incoming rotor RPM before cone contact, allowing a smooth re-engagement without shock loading.

---

## Current Simulation Implementation In Repo

The repository now contains a working simulation stack under `simulation/` built around ArduPilot SITL, a Python mediator, and an MBDyn rigid-body model.

### Simulation Stack Overview

The implemented runtime architecture is:

```text
ArduPilot SITL (heli JSON backend)
   ⇅ UDP 9002 / 9003
Python mediator.py
   ⇅ UNIX sockets
MBDyn rigid-body model
```

- **ArduPilot SITL** provides flight-controller outputs (servo commands) and consumes synthetic IMU/pose/velocity state.
- **mediator.py** is the co-simulation bridge. It converts SITL servo outputs into swashplate commands, computes aerodynamic loads, exchanges data with MBDyn, and synthesizes the sensor/state packet sent back to SITL.
- **MBDyn** integrates the hub rigid-body motion, gravity, and tether constraint.

### Key Simulation Files

| File | Role |
|------|------|
| `simulation/mediator.py` | Main 400 Hz co-simulation loop |
| `simulation/mbdyn_interface.py` | UNIX socket transport to/from MBDyn |
| `simulation/sitl_interface.py` | UDP JSON transport to/from ArduPilot SITL |
| `simulation/swashplate.py` | H3-120 inverse mixing and cyclic pitch math |
| `simulation/aero.py` | Simplified rotor aerodynamic model |
| `simulation/sensor.py` | ENU → NED/state → sensor conversion |
| `simulation/mbdyn/rotor.mbd` | MBDyn input deck |
| `simulation/mbdyn/rawes.set` | MBDyn parameter definitions |

### Coordinate Conventions

- **MBDyn world frame:** ENU (`X=East, Y=North, Z=Up`)
- **ArduPilot frame:** NED (`X=North, Y=East, Z=Down`)
- **Mediator responsibility:** convert MBDyn ENU state into the NED/body-frame quantities expected by ArduPilot's JSON backend.

---

## Current MBDyn Model

The MBDyn model is intentionally compact. It is not yet a blade-resolved multibody rotor.

### What Is Modeled In MBDyn

The current MBDyn deck represents the RAWES as:

1. **One dynamic hub node** starting at 50 m altitude
2. **One static ground node** at the world origin
3. **One rigid body** representing the full rotor/hub assembly as a lumped mass/inertia
4. **One clamp joint** fixing the ground node to the inertial frame
5. **One rod joint** representing the tether as a linear elastic spring constraint
6. **Uniform gravity**
7. **An externally applied force** from Python
8. **An externally applied couple** from Python
9. **A state output stream** sending hub position, velocity, orientation matrix, and angular velocity back to Python

### What Is Not Modeled In MBDyn

The current MBDyn deck does **not** yet contain:

- Separate blade bodies
- Blade flapping or lag DOFs
- Explicit swashplate bodies or kinematic linkages
- Explicit trailing-edge flap mechanics
- Distributed tether dynamics or catenary shape
- A full rotor spin state equation with gyroscopic blade dynamics

So the MBDyn part is best understood as a **hub-centric 6-DOF rigid-body surrogate** with a spring tether and externally injected aerodynamic wrench.

### Current MBDyn Parameters

The current parameter set in `simulation/mbdyn/rawes.set` uses:

- `DT = 2.5e-3 s` → 400 Hz
- `T_FINAL = 300 s`
- Initial altitude `Z0 = 50 m`
- Initial spin rate `OMEGA0 = 28 rad/s`
- `N_BLADES = 4`
- `BLADE_LEN = 2.0 m`
- `R_TIP = 2.5 m`
- `M_ROTOR = 5.0 kg`
- `Ixx = Iyy = 5.0 kg·m²`
- `Izz = 10.0 kg·m²`
- `TETHER_K = 300 N/m`

These values are closer to the intended 4-blade hardware than the thesis reference model, but the dynamics are still simplified.

---

## Current Mediator Behavior

`simulation/mediator.py` is the executable bridge between controller, aerodynamics, sensors, and rigid-body physics.

### Main Loop Responsibilities

The mediator runs at a target rate of **400 Hz**. Each iteration performs:

1. **Receive servo outputs from SITL**
  - UDP port 9002
  - 16 servo channels
  - reused last values if no new packet arrives in time

2. **Decode swashplate commands**
  - Channels 1–3 are interpreted as `S1/S2/S3`
  - H3-120 inverse mixing computes:
    - collective
    - longitudinal tilt
    - lateral tilt
  - Channel 4 is interpreted as the anti-rotation motor ESC command

3. **Estimate rotor spin from current MBDyn state**
  - Rotor angular rate is estimated by projecting hub angular velocity onto the rotor disk normal
  - If the estimate collapses near zero during startup, it falls back to a nominal `28 rad/s`

4. **Compute aerodynamic wrench**
  - Calls `RotorAero.compute_forces(...)`
  - Produces `[Fx, Fy, Fz, Mx, My, Mz]` in the world ENU frame

5. **Send forces to MBDyn**
  - Python writes 6 float64 values to the MBDyn force socket

6. **Receive new state from MBDyn**
  - Python reads 18 float64 values in MBDyn's fixed output order: position, orientation matrix, velocity, angular velocity
  - Parsed into pos (3), R (3×3 row-major), vel (3), omega (3)

7. **Estimate acceleration**
  - Uses finite-difference of velocity

8. **Synthesize sensor outputs**
  - Converts ENU position/velocity into NED
  - Computes roll/pitch/yaw from the rotation matrix
  - Converts world acceleration and angular rate into body-frame IMU data
  - Adds simple accelerometer and gyroscope noise

9. **Send JSON physics state back to SITL**
   - UDP port 9003

### Startup Behavior

Before the main loop begins, the mediator:

- binds the SITL UDP sockets
- waits for MBDyn to create its UNIX sockets
- sends an initial gravity-compensation wrench `[0, 0, 49.05, 0, 0, 0]`
- reads an initial hub state from MBDyn if available

This avoids immediate free-fall before SITL servo commands begin arriving.

---

## Current Aerodynamic Model

The aerodynamic model in `simulation/aero.py` is a **simplified BEM-style model**, not the thesis flap-dynamics controller and not a CFD-based rotor model.

### What The Aero Model Currently Does

- uses a rotor disk normal derived from the hub rotation matrix
- computes relative wind from ambient wind minus hub velocity
- estimates induced velocity using actuator-disk momentum theory
- integrates thrust over 20 radial strips from root to tip
- uses a linear aerodynamic model:
  - `CL = CL0 + CL_alpha * AoA`
  - `CD = CD0 + CD_alpha * AoA²`
- clamps angle of attack to roughly ±15° for linear-model validity
- generates:
  - thrust force along the disk normal
  - drag torque about the disk axis
  - cyclic moments proportional to swashplate tilt and thrust
- ramps loads in over the first 5 seconds to avoid an impulsive startup

### What The Aero Model Does Not Yet Do

- dynamic inflow
- tip-loss correction
- wake swirl
- flap-lag coupling
- tether aerodynamics
- true flap deflection to blade-pitch dynamics from the Weyel controller

This means the present mediator/aero path is a practical first-pass load generator, not yet the final physical model.

---

## Current Sensor Simulation

The sensor model in `simulation/sensor.py` converts rigid-body truth state into what ArduPilot expects.

### Outputs Synthesized For SITL

- position in NED
- velocity in NED
- attitude as roll/pitch/yaw
- body-frame accelerometer
- body-frame gyroscope

### Important Detail

MBDyn returns angular velocity in the **world frame**, so the sensor model rotates it into the body frame before packaging the gyro data.

Simple noise is added to both accelerometer and gyro outputs so SITL is not fed perfectly noise-free sensors.

---

## Current Protocol Interfaces

### ArduPilot SITL ↔ mediator

- **SITL → mediator:** UDP JSON servo packets on port `9002`
- **mediator → SITL:** UDP JSON state packets on port `9003`

The JSON state packet contains:

- timestamp
- IMU gyro
- IMU accel
- position `[north, east, down]`
- attitude `[roll, pitch, yaw]`
- velocity `[vn, ve, vd]`

### mediator ↔ MBDyn

- **mediator → MBDyn:** UNIX socket `/tmp/rawes_forces.sock`
- **MBDyn → mediator:** UNIX socket `/tmp/rawes_state.sock`

Binary layout:

- outgoing force packet: `6 x float64`
- incoming state packet: `18 x float64`

The communication pattern is synchronous per step:

1. Python sends forces
2. MBDyn integrates one step
3. Python reads the updated state

---

## Current Implementation Limits

Important limitations of the current simulation stack:

1. **Hub-centric MBDyn model only**
  - No explicit blade multibody dynamics yet

2. **Rotor spin is estimated, not fully integrated**
  - The mediator infers spin from the hub angular velocity projected onto the disk normal

3. **Tether is a spring-rod only**
  - No sag, drag, reel dynamics, or distributed mass

4. **Aerodynamics are first-order**
  - No wake model, no dynamic inflow, no stall hysteresis, no explicit flap aerodynamics

5. **Anti-rotation motor not modeled in single-body simulation**
  - The motor keeps the lower swashplate stationary by counteracting bearing friction and push-rod reaction forces from cyclic pitch
  - Both loads are internal forces in the current single-body MBDyn model and cancel out — the motor torque is therefore correctly absent from the external wrench sent to MBDyn
  - Motor modeling becomes necessary when the simulation is split into separate spinning-hub and non-rotating-electronics bodies

6. **Sensor path is synthetic only**
  - Good enough for SITL integration, not yet a hardware-grade sensor model

7. **Sensor path is NED body-frame convention only**
  - Attitude, gyro, and accel are all expressed in NED body convention (body Z = Down) consistent with ArduPilot's EKF expectations

---

## Running Tests

There are two test suites under `simulation/tests/`. They have different environments and are launched differently.

### Test Suites

| Suite | Location | What it tests | Environment |
|-------|----------|---------------|-------------|
| Unit tests | `tests/unit/` | Individual Python modules: sensor, swashplate, aero | Windows native (no Docker) |
| Stack integration | `tests/stack/` | Full MBDyn + ArduPilot SITL end-to-end loop | Docker (requires ArduPilot image) |

### Unit Tests — run on Windows, no Docker needed

Unit tests only require numpy and pytest. A local venv is created automatically on first run.

```cmd
simulation\run_unit_tests.cmd
```

To pass extra pytest flags (e.g. `-v`, `-k test_sensor`):
```cmd
simulation\run_unit_tests.cmd -v
```

### Stack Integration Tests — run in Docker

Stack tests launch three processes inside the container (ArduPilot SITL, MBDyn, mediator.py) and drive a full guided-flight sequence via MAVLink. They require a Docker image built with ArduPilot SITL.

**Step 1 — build the image (one-time, ~30 min):**
```cmd
simulation\build.cmd ardupilot
```

**Step 2 — run the tests:**
```cmd
simulation\run_stack_integration.cmd
```

This mounts `simulation\` into a one-shot container and runs `tests/stack/` via `test_stack.sh`.

### Dev Container — persistent container for faster iteration

`dev.cmd` manages a persistent `rawes-dev` container so each test run skips Docker startup overhead. The `simulation\` directory is still mounted live, so code changes are reflected immediately.

```cmd
simulation\dev.cmd start            # start container (idempotent)
simulation\dev.cmd test-unit        # run unit tests inside container
simulation\dev.cmd test-stack       # run stack integration tests inside container
simulation\dev.cmd shell            # interactive bash shell in container
simulation\dev.cmd stop             # stop and remove container
```

Extra pytest args are forwarded:
```cmd
simulation\dev.cmd test-stack -v -k test_guided_flight
simulation\dev.cmd test-unit  -v -k test_sensor
```

### When each script must be used

| Task | Command |
|------|---------|
| First-time setup (build image) | `simulation\build.cmd ardupilot` |
| Quick unit-test pass on Windows | `simulation\run_unit_tests.cmd` |
| One-shot stack integration run | `simulation\run_stack_integration.cmd` |
| Iterating on stack tests (faster) | `simulation\dev.cmd test-stack` |
| Interactive debugging in container | `simulation\dev.cmd shell` |

---

## Next Steps (Planned)

- [x] Confirm battery capacity: 450 mAh (confirmed 2026-03-20)
- [x] Confirm servo model for S1/S2/S3: DS113MG V6.0 Digital Metal Gear Micro Servo
- [x] Define takeoff/landing modes for a tethered autorotating system
- [ ] Update simulation parameters for 4-blade, 2m geometry
- [ ] Python simulation running with flapmodel.md equations
- [ ] ArduPilot helicopter frame configuration for RAWES (swashplate type, RSC mode)
- [ ] Mapping: ArduPilot collective/cyclic outputs → swashplate → flap deflection → blade pitch
- [ ] Hardware-in-the-loop testing with Pixhawk SITL
