# RAWES — Project Context for Claude

## Project Goal

Build an **ArduPilot flight controller model** for a Rotary Airborne Wind Energy System (RAWES) that can fly in all standard modes: takeoff, stabilized flight, autonomous flight, landing. This is a long-term, step-by-step effort. **Current phase: knowledge collection and organization.**

---

## What Is This System?

A **RAWES** (Rotary Airborne Wind Energy System) is a tethered, rotating 4-blade rotor kite. It operates like an autogyro rotor — wind drives autorotation, and cyclic pitch control tilts the rotor disk to steer. A tether connects to the bottom of the rotor axis; tension during reel-out drives a ground generator (pumping cycle power generation).

Key distinction from a drone: **no motor drives rotation** — wind does. Control is entirely through blade pitch, actuated indirectly via trailing-edge flaps on each blade.

---

## Control Architecture

Three nested control problems (from Felix Weyel's thesis):

```
Planned Trajectory → [MPC] → Reference Blade Pitch β̄
                                      ↓
                             [Flap Controller] → Flap Actuation γ_CL
                                      ↑                      ↓
                              Actual Blade Pitch β ← [RAWES Model] → Output
```

- **A — Trajectory → reference pitch:** MPC (Model Predictive Control) — not yet implemented
- **B — Reference pitch → flap actuation:** Feed-forward + PID — implemented in thesis (Python)
- **C — Physics simulation:** Full nonlinear ODE state space model — implemented in thesis (Python)

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
**EMAX GB4008** (66KV, hollow shaft) via **80:44 spur gear** applies counter-torque to keep the electronics/swashplate assembly non-rotating against rotor drag. It does not drive rotation.

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
| [summary.md](summary.md) | RAWES optimal control model from De Schutter et al. 2018 — pumping cycle, structural constraints, atmosphere model |
| [simulation_model.md](simulation_model.md) | Duplicate of flapmodel.md |

---

## Key Academic References

1. **Felix Weyel (2025)** — "Modeling and Closed Loop Control of a Cyclic Pitch Actuated Rotary Airborne Wind Energy System", Bachelor's Thesis, Uni Freiburg. Primary simulation/control reference. Measurement data by Christof Beaupoil / someAWE Labs.

2. **De Schutter, Leuthold, Diehl (2018)** — "Optimal Control of a Rigid-Wing Rotary Kite System for Airborne Wind Energy". Full system optimization including pumping cycle, structural constraints, atmosphere model.

3. **US Patent US3217809** (Kaman/Bossler, 1965) — Canonical servo-flap rotor control system.

---

## Known Model Gaps (thesis model vs actual hardware)

| Item | Thesis model | Actual design |
|------|-------------|--------------|
| Blade count | 3 | 4 |
| Blade length | 1.5 m | 2.0 m |
| Cyclic phase offset | 2π/3 (120°) | π/2 (90°) |
| Rotor mass | 40 kg | 5 kg |
| Flap actuation | Individual servo per blade | Swashplate push-rods (mechanical) |
| Anti-rotation | Not modeled | GB4008 + 80:44 gear |

Additional physics limitations in current model:
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

## Next Steps (Planned)

- [x] Confirm battery capacity: 450 mAh (confirmed 2026-03-20)
- [x] Confirm servo model for S1/S2/S3: DS113MG V6.0 Digital Metal Gear Micro Servo
- [x] Define takeoff/landing modes for a tethered autorotating system
- [ ] Update simulation parameters for 4-blade, 2m geometry
- [ ] Python simulation running with flapmodel.md equations
- [ ] ArduPilot helicopter frame configuration for RAWES (swashplate type, RSC mode)
- [ ] Mapping: ArduPilot collective/cyclic outputs → swashplate → flap deflection → blade pitch
- [ ] Hardware-in-the-loop testing with Pixhawk SITL
