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

The repository now contains a working simulation stack under `simulation/` built around ArduPilot SITL and a Python mediator with an integrated RK4 rigid-body dynamics engine.

### Simulation Stack Overview

The implemented runtime architecture is:

```text
ArduPilot SITL (heli JSON backend)
   ⇅ UDP 9002 / 9003
Python mediator.py
   ⇅ function call
dynamics.py (RK4 6-DOF integrator)
```

- **ArduPilot SITL** provides flight-controller outputs (servo commands) and consumes synthetic IMU/pose/velocity state.
- **mediator.py** is the co-simulation bridge. It converts SITL servo outputs into swashplate commands, computes aerodynamic and tether loads, steps the dynamics integrator, and synthesizes the sensor/state packet sent back to SITL.
- **dynamics.py** integrates the hub rigid-body motion using RK4, with gravity applied internally.

MBDyn has been removed from the runtime loop. See `simulation/mbdyn_reference.md` for the full MBDyn integration documentation and restoration instructions.

### Key Simulation Files

| File | Role |
|------|------|
| `simulation/mediator.py` | Main 400 Hz co-simulation loop |
| `simulation/dynamics.py` | Python RK4 6-DOF rigid-body integrator |
| `simulation/sitl_interface.py` | UDP JSON transport to/from ArduPilot SITL |
| `simulation/swashplate.py` | H3-120 inverse mixing and cyclic pitch math |
| `simulation/aero.py` | BEM rotor aerodynamic model (per-strip torque integral) |
| `simulation/sensor.py` | ENU → NED/state → sensor conversion |
| `simulation/mbdyn_reference.md` | MBDyn integration documentation for future restoration |
| `simulation/archived/mbdyn_interface.py` | UNIX socket transport to/from MBDyn |
| `simulation/archived/rotor.mbd` | MBDyn input deck |
| `simulation/archived/rawes.set` | MBDyn parameter definitions |

### Coordinate Conventions

- **World frame:** ENU (`X=East, Y=North, Z=Up`)
- **ArduPilot frame:** NED (`X=North, Y=East, Z=Down`)
- **Mediator responsibility:** convert ENU state into the NED/body-frame quantities expected by ArduPilot's JSON backend.

---

## Natural / Equilibrium Hub Orientation

**The rotor axle (body Z) always aligns with the tether direction.** The rotor disk rotates around the same axis as the tether. At any tether elevation angle β the equilibrium orientation is:

```
body Z = [cos(β), 0, sin(β)]   (ENU, hub East of anchor)
```

The tether restoring torque (`M = r_attach × F_tether`, implemented in `TetherModel`) enforces this alignment whenever the tether is taut. The hub should be initialised with this orientation, not upright. For simulation and tests, always set the initial rotation matrix so body Z points along the tether direction rather than vertically.

---

## Current Dynamics Model

The rigid-body dynamics are integrated by `simulation/dynamics.py` using RK4 at 400 Hz. The model is intentionally compact — a single hub body, not a blade-resolved multibody rotor.

### What Is Modeled

1. **Hub position and velocity** in ENU world frame
2. **Hub orientation** as a rotation matrix (body → world), re-orthogonalised via SVD every 200 steps
3. **Hub angular velocity** in world frame
4. **Gravity** applied internally as `[0, 0, −m·g]`
5. **External force** from Python (aerodynamic + tether combined)
6. **External moment** from Python (aerodynamic moments)

### What Is Not Modeled

- Separate spinning hub body vs. non-rotating electronics body
- Blade bodies, flapping, or lag DOFs
- Explicit swashplate kinematic linkages
- Distributed tether dynamics or catenary shape
- Gyroscopic coupling between blades and hub

### Current Dynamics Parameters (hardcoded in `mediator.py`)

| Parameter | Value |
|-----------|-------|
| Timestep | 2.5e-3 s → 400 Hz |
| Initial altitude | 50 m |
| Initial spin rate | 28 rad/s |
| Mass | 5.0 kg |
| Ixx = Iyy | 5.0 kg·m² |
| Izz | 10.0 kg·m² |

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

4b. **Compute tether force and add to wrench**
  - Calls `TetherModel.compute(hub_pos, hub_vel)`
  - Tension-only: zero if hub is closer to anchor than rest length (slack)
  - Result added to `Fx/Fy/Fz`; moments unchanged

5. **Step the rigid-body dynamics integrator**
  - Calls `RigidBodyDynamics.step(F_world, M_world, dt)`
  - `F_world = forces[0:3]` (aero + tether); `M_world = forces[3:6]` (aero moments)
  - Gravity applied internally — do **not** add it to forces
  - Returns state dict: `{pos, vel, R, omega}` in ENU world frame

7. **Estimate acceleration**
  - Uses finite-difference of velocity

8. **Synthesize sensor outputs**
  - Converts ENU position/velocity into NED
  - Computes roll/pitch/yaw from the rotation matrix
  - Converts world acceleration and angular rate into body-frame IMU data
  - Adds simple accelerometer and gyroscope noise

9. **Send JSON physics state back to SITL**
   - UDP port 9003

### Tether Model (`TetherModel` in `mediator.py`)

| Property | Value |
|----------|-------|
| Material | Dyneema SK75 1.9 mm braided UHMWPE |
| EA (axial stiffness) | ~281 kN (109 GPa × 2.835×10⁻⁶ m² × 0.91 braid eff.) |
| k(L) | EA / L — nonlinear, stiffer at shorter lengths |
| Linear mass | 2.1 g/m |
| Break load | ~620 N (conservative for 1.9 mm) |
| Structural damping | 5 N·s/m (extension only — no compression damping) |
| Rest length | 49.940 m default (configurable via `--tether-rest-length`) |
| Anchor | World origin (0, 0, 0) ENU |
| Slack zone | Hub closer than rest length → zero force |

Default rest length matches the steady-state equilibrium so the tether is taut from the first step. Logs warn if tension exceeds 80% of break load.

### Startup Behavior

Before the main loop begins, the mediator:

- binds the SITL UDP sockets
- initialises `RigidBodyDynamics` with the initial state (see below)

No warm-up force send is needed — gravity is handled internally by the integrator from step 1.

### Initial State and `steady_state_starting.json`

The mediator's default initial state comes from the **warmup-settled equilibrium** of `test_steady_flight.py`:

| Parameter | Value | Source |
|-----------|-------|--------|
| `pos0` | `[46.258, 14.241, 12.530]` ENU m | 50 m tether, ~30° elev, ~14 m N drift from warmup |
| `vel0` | `[-0.257, 0.916, -0.093]` m/s | settled hub velocity |
| `body_z` | `[0.851, 0.305, 0.427]` | axle aligned with tether |
| `omega_spin` | `20.148` rad/s | equilibrium autorotation spin |
| `rest_length` | `49.940` m | tether taut from t=0 |

These constants are exposed as `DEFAULT_POS0`, `DEFAULT_VEL0`, `DEFAULT_BODY_Z`, `DEFAULT_OMEGA_SPIN` in `mediator.py`.

**Workflow for regenerating initial state:**
1. Run `pytest simulation/tests/unit/test_steady_flight.py` — this writes `simulation/steady_state_starting.json`
2. The stack flight test (`test_guided_flight.py`) reads this file and passes the values to the mediator via `--pos0`, `--vel0`, `--body-z`, `--omega-spin` CLI args
3. If `steady_state_starting.json` is absent, mediator uses the built-in defaults above

**Why the initial state matters:** Starting with body Z aligned to the tether and `omega_spin` at autorotation equilibrium means ArduPilot sees roll=0, pitch=0 from the first packet (tether-relative attitude, see Sensor section below) and the hub is already in a stable force balance with the tether taut.

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

### Attitude Representation — Tether-Relative, Not Absolute

**Critical design decision:** attitude is reported as the **deviation from the tether equilibrium orientation**, not as absolute orientation relative to world Up.

**Why this is necessary:** The RAWES equilibrium has the rotor axle (body Z) aligned with the tether at ~30° elevation — roughly 65° from vertical. If absolute attitude were reported, ArduPilot would see 58° roll / 36° pitch at rest and command maximum cyclic continuously to "correct" the perceived tilt. The hub would crash within 1 second of the startup freeze lifting.

**How it works (`sensor.py`):**
1. `R_orb_eq`: equilibrium orbital frame, built from `body_z_eq = (hub_pos − anchor) / |hub_pos − anchor|` (tether direction)
2. `R_orb`: actual orbital frame, built from `disk_normal = R_hub[:, 2]` (spin removed)
3. `R_dev = R_orb_eq.T @ R_orb`: deviation rotation (identity at tether equilibrium)
4. Attitude sent to ArduPilot: ZYX Euler of `T @ R_dev @ T` (NED convention) → **roll=0, pitch=0 when axle is tether-aligned**

Accel and gyro remain expressed in the physical `R_orb` frame (actual electronics body frame). At equilibrium `R_orb = R_orb_eq` so all three outputs are consistent.

**Consequence for ArduPilot:** its attitude controller interprets small deviations from tether equilibrium as small roll/pitch errors, which it corrects with modest cyclic commands rather than saturating at maximum tilt.

### Other Important Details

Angular velocity is provided in the **world frame** from the dynamics integrator; sensor.py strips the spin component and rotates into the body frame before packaging the gyro.

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

1. **Hub-centric Python RK4 model**
  - Single 6-DOF rigid body (translational + rotational), gravity internal
  - No explicit blade multibody dynamics yet

2. **Rotor spin is estimated, not fully integrated**
  - The mediator infers spin from the hub angular velocity projected onto the disk normal

3. **Tether is tension-only elastic (Python `TetherModel`)**
  - Dyneema SK75 1.9 mm: EA ≈ 281 kN, break load ≈ 620 N, 2.1 g/m
  - Stiffness k = EA/L (nonlinear — stiffer when shorter)
  - Structural damping 5 N·s/m (extension only; rope can't push)
  - Rest length 200 m; hub starts at 50 m → slack at launch
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

Unit tests only require numpy and pytest. Create a venv once, then run pytest directly:

```cmd
py -3 -m venv simulation\tests\unit\.venv
simulation\tests\unit\.venv\Scripts\python.exe -m pip install numpy pytest
```

Then to run:
```cmd
simulation\tests\unit\.venv\Scripts\python.exe -m pytest simulation\tests\unit
```

With extra pytest flags (e.g. `-v`, `-k test_sensor`):
```cmd
simulation\tests\unit\.venv\Scripts\python.exe -m pytest simulation\tests\unit -v -k test_sensor
```

### Stack Integration Tests — run in Docker via WSL

Stack tests launch SITL + mediator inside the container and drive a full guided-flight sequence via MAVLink. They require a Docker image built with ArduPilot SITL.

The image name is `rawes-sim`. The container mounts `simulation/` at `/rawes/simulation`.

All Docker commands must run via `wsl.exe bash -c '...'` — this avoids MSYS/Git Bash path conversion. Do **not** use Git Bash or cmd.exe directly for docker volume mounts.

**Build the image (one-time, ~30 min):**
```cmd
simulation\build.cmd ardupilot
```

**`dev.sh` — manage the persistent container:**

Run from Windows terminal using `wsl.exe bash -c '...'`:
```cmd
wsl.exe bash -c 'bash /mnt/e/repos/windpower/simulation/dev.sh start'
wsl.exe bash -c 'bash /mnt/e/repos/windpower/simulation/dev.sh stop'
wsl.exe bash -c 'bash /mnt/e/repos/windpower/simulation/dev.sh test-unit'
wsl.exe bash -c 'bash /mnt/e/repos/windpower/simulation/dev.sh test-stack'
wsl.exe bash -c 'bash /mnt/e/repos/windpower/simulation/dev.sh shell'
```

Extra pytest args are forwarded after the subcommand:
```cmd
wsl.exe bash -c 'bash /mnt/e/repos/windpower/simulation/dev.sh test-stack -v -k test_guided_flight'
wsl.exe bash -c 'bash /mnt/e/repos/windpower/simulation/dev.sh test-unit -v -k test_sensor'
```

### When each command must be used

| Task | Command |
|------|---------|
| First-time setup (build image) | `simulation\build.cmd ardupilot` |
| Quick unit-test pass on Windows | `simulation\tests\unit\.venv\Scripts\python.exe -m pytest simulation\tests\unit` |
| Unit tests in container | `wsl.exe bash -c 'bash /mnt/e/repos/windpower/simulation/dev.sh test-unit'` |
| Stack integration tests | `wsl.exe bash -c 'bash /mnt/e/repos/windpower/simulation/dev.sh test-stack'` |
| Interactive debugging in container | `wsl.exe bash -c 'bash /mnt/e/repos/windpower/simulation/dev.sh shell'` |
| Regenerate flight_report.png from last run (no re-test) | `wsl.exe bash -c 'python3 /mnt/e/repos/windpower/simulation/redraw_flight_report.py'` |

---

## Next Steps (Planned)

- [x] Confirm battery capacity: 450 mAh (confirmed 2026-03-20)
- [x] Confirm servo model for S1/S2/S3: DS113MG V6.0 Digital Metal Gear Micro Servo
- [x] Define takeoff/landing modes for a tethered autorotating system
- [x] Fix aero.py spin torque model: replaced lumped K_drag/K_auto constants with per-strip BEM torque integral `dQ = (CL·sin φ − CD·cos φ)·q·chord·r·dr·N`
- [ ] Update simulation parameters for 4-blade, 2m geometry
- [ ] Python simulation running with flapmodel.md equations
- [ ] ArduPilot helicopter frame configuration for RAWES (swashplate type, RSC mode)
- [ ] Mapping: ArduPilot collective/cyclic outputs → swashplate → flap deflection → blade pitch
- [ ] Hardware-in-the-loop testing with Pixhawk SITL
