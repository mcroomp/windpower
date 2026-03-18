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

### Rotor Geometry
| Parameter | Value |
|-----------|-------|
| Number of blades | 4 (90° apart) |
| Blade length | 2000 mm |
| Hub radius | ~500 mm (hub diameter ~1000 mm) |
| Total rotor radius | ~2500 mm |
| Total rotor diameter | 5000 mm |
| Rotor mass | 5 kg |
| Blade airfoil | SG6042 |
| Root airfoil | SG6040 |
| Blade material | EPP RG30 (foam) |
| Twist | None |
| Tether attachment | Bottom of rotor axis (center) |

### Pitch Control — Trailing-Edge Flaps via Swashplate

Pitch is controlled **indirectly** via trailing-edge flaps on each blade, driven mechanically by a conventional 3-servo swashplate via push-rods. This is Kaman-style aerodynamic pitch control:

```
3 Servos (S1, S2, S3) → Swashplate plane → Push-rods → Trailing-edge flaps → Blade twist → Pitch change
```

- The flaps are **not** individually motorized — they are mechanically linked to the swashplate
- 3 servos control the swashplate plane (collective + cyclic), regardless of 4 blades
- Low servo torque required: servos move only the small flaps, not the blades directly
- Flap aerodynamic moment twists the flexible blade, changing angle of attack

Sign convention: γ < 0 (upward flap) → positive moment → blade pitches UP

### Anti-Rotation Motor (Swashplate Hold)
The **EMAX GB4008 gimbal motor** prevents the swashplate lower ring from rotating with the rotor:
- Motor is **offset** from the rotor axis
- Connected via **spur gear, ratio 80:44** (motor gear 44T → swashplate ring 80T)
- Gear reduction gives ~1.82× torque multiplication at swashplate
- Motor actively holds the non-rotating swashplate ring stationary against rotor drag

### Tether
| Parameter | Value (reference) |
|-----------|-------|
| Tether attachment | Bottom center of rotor axis |
| Tether diameter | 1.9 mm |
| Max tether length | 300 m |
| Min altitude | 10 m |

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

### Power Architecture (from whiteboard)

```
Battery 4S 15.2V
  ├── XT30 → BM/PM (Battery Monitor + Power Module) → XT60 → PDB → XT60 → ESC → [JST X7 2.34] → GB4008 Motor
  └── XT30 → UBEC → 8.0V → Pixhawk servo rail → S1, S2, S3 (servos)
                         └── Pixhawk FMU (via PM power module)
```

**Note on "?" in whiteboard:** The Pixhawk 6C PM powers the FMU electronics but does **not** power the servo output rail. The UBEC must connect to the servo rail (pin 1 of any servo output) to supply servo power. This connection is required.

### Communication Architecture
```
MissionPlanner (PC) ──SiK Radio V3────────────────> Pixhawk 6C
Boxer Controller    ──ExpressLRS 2.4GHz──> RP3-H ──> Pixhawk 6C
Pixhawk 6C          ──PWM (S1,S2,S3)────────────> Swashplate Servos
Pixhawk 6C          ──DSHOT/PWM─────────────────> REVVitRC ESC ──> GB4008 Motor
```

### Components

**Flight Controller — Holybro Pixhawk 6C** (minimum spec)
- ArduPilot / MissionPlanner
- Manages collective + cyclic swashplate mixing in software (H_RSC_MODE, H_SV_MAN etc.)
- S1, S2, S3 → swashplate servo outputs

**Motor — EMAX GB4008 Brushless Gimbal Motor**
- Hollow shaft, 66KV, 90T, 24N22P, 7.5Ω, 101g
- Role: holds swashplate non-rotating ring stationary via 80:44 spur gear
- Hollow shaft allows push-rod routing through blade axis

**ESC — REVVitRC 50A AM32**
- AM32 firmware, Servo PWM / DSHOT 300 / DSHOT 600
- Built-in BEC 7.4V/8.4V (not used here — UBEC used instead)
- Sinusoidal startup mode for smooth motor control

**UBEC**
- Output: 8.0V
- Powers: Pixhawk servo rail + S1, S2, S3 servos
- Input: battery via XT30

**Battery**
- 4S LiPo, 15.2V nominal
- Capacity: 450mAh *(flagged as small — verify, may be 4500mAh)*

**Telemetry — Holybro SiK Telemetry Radio V3**
- MAVLink, 433/915MHz, up to 500mW, 300m+ range

**RC Receiver — RadioMaster RP3-H ExpressLRS 2.4GHz**
- CRSF / S.Bus output, 25–500Hz

**RC Transmitter — RadioMaster Boxer (M2)**
- EdgeTX, ExpressLRS 2.4GHz, Hall effect gimbals

---

## Reference Documents in This Repo

| File | Contents |
|------|---------|
| [flapmodel.md](flapmodel.md) | Full mathematical model from Weyel 2025 — aerodynamics, state space, flap dynamics, controller, all parameter tables |
| [flaprotordesign.md](flaprotordesign.md) | Physical blade design decisions — airfoil selection (SG6042), dimensions, Reynolds number, flap prototype |
| [servoflaps.md](servoflaps.md) | Kaman flap design reference — US patent US3217809, mechanical principle, swashplate linkage path |
| [hardware.md](hardware.md) | Hardware stack — all components with full specs |
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

## Next Steps (Planned)

- [ ] Confirm battery capacity (450mAh vs 4500mAh)
- [ ] Confirm servo model for S1/S2/S3
- [ ] Update simulation parameters for 4-blade, 2m geometry
- [ ] Python simulation running with flapmodel.md equations
- [ ] ArduPilot helicopter frame configuration for RAWES (swashplate type, RSC mode)
- [ ] Mapping: ArduPilot collective/cyclic outputs → swashplate → flap deflection → blade pitch
- [ ] Define takeoff/landing modes for a tethered autorotating system
- [ ] Hardware-in-the-loop testing with Pixhawk SITL
