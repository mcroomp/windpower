> **OBSOLETE — do not use.** This is a duplicate of `flapmodel.md`. Retained for historical reference only. Refer to `flapmodel.md` instead.

# RAWES Simulation Model Reference
*Extracted from: "Modeling and Closed Loop Control of a Cyclic Pitch Actuated Rotary Airborne Wind Energy System" — Felix Weyel, Uni Freiburg, Sept 2025*

---

## System Architecture

```
Planned Trajectory → [MPC] → Reference Blade Pitch β̄
                                    ↓
                           [Flap Controller] → Flap Actuation γ_CL
                                    ↑                    ↓
                            Actual Blade Pitch β ← [RAWES Model] → Output
```

Three control problems:
- **A:** Trajectory → reference blade pitch (MPC — not implemented in thesis)
- **B:** Reference pitch → flap actuation (feed-forward + PID — implemented)
- **C:** System simulation given inputs (state space model — implemented)

---

## 1. Aerodynamics

### Forces and Moments
```
L = q∞ · CL(α) · S
D = q∞ · CD(α) · S
M = q∞ · CM(α) · S · l

q∞ = ½ · ρ · V²∞    (dynamic pressure)
```

### Aerodynamic Coefficients (linearized, valid for -20° < α < 20°)
```
CL(α) = CL,0 + CL,α · α       (linear)
CD(α) = CD,0 + CD,α · α²      (quadratic)
CM(α) = CM,0 + CM,α · α       (linear)
```

### Angle of Attack
```
α = β + φ

φ = atan(vz / vy)              (inflow angle, positive if wind has +z component)

v_ap = v_b - v_w               (apparent wind = blade velocity - external wind)
```

---

## 2. Kaman Flap (Servo Flap) Mechanics

```
αf = α + γ                     (flap angle of attack; γ = flap deflection angle)

γ < 0 (upward)   → positive moment M → blade pitches UP
γ > 0 (downward) → negative moment M → blade pitches DOWN
```

### Moment on blade from flap:
```
Mi = q∞ · S · l · (CM,β · αi + CM,γ · αf,i)
```
Where:
- `CM,β` = moment coefficient of blade body
- `CM,γ` = linear offset coefficient from flap deflection
- `αf,i = αi + γi`

---

## 3. RAWES Full State Space Model

### Reference Frames
- **Global frame:** [ex, ey, ez] — world coordinates
- **Rotor frame:** [e'x, e'y, e'z] — rotor disc lies flat in e'x·e'y plane; e'z = rotational axis
- **Blade frame:** [e''x, e''y, e''z] — blade lies flat in e''x·e''y plane

### Blade rotation matrix (blade i, i = 1,2,3):
```
R_θ,i = | cos(θ + (i-1)·2π/3)   -sin(θ + (i-1)·2π/3)   0 |
         | sin(θ + (i-1)·2π/3)    cos(θ + (i-1)·2π/3)   0 |
         | 0                       0                       1 |
```

### Blade velocity (at r = 2/3 · span, counter-clockwise rotation):
```
v_b = e''_y · r · ω_θ
```

### Apparent wind for blade i:
```
v_ap,i = e''_y · r · ω_θ - R_θ,i · Rᵀ · v_w
```
Where R = rotor inclination rotation matrix (global ↔ rotor frame)

### Aerodynamic force on blade i (in blade frame):
```
F_i = R_α,i · e''_z · ||L_i|| + R_α,i · e''_y · ||D_i||

R_α,i = | 1    0          0       |
         | 0    cos(αi)   -sin(αi) |    (rotation by αi around e''_x)
         | 0    sin(αi)    cos(αi) |
```

### Total rotor force (in rotor frame):
```
F_rotor = F_G + Σ(i=1..3) R_θ,iᵀ · F_i

F_G = -Rᵀ · ez · m · g             (gravity in rotor frame)
```

### Tether stabilizing moment:
```
M_tether = -e'_z · d × F_G         (d = distance, CoM below rotor)
```

### Total moment on rotor:
```
M_rotor = Σ(i=1..3) [R_θ,iᵀ · r × R_θ,iᵀ · F_i] + M_tether + D_r · ω_R
```

### State vector:
```
x_r(t) = [p(t), v(t), R(t), ω_R(t), θ(t), ω_θ]ᵀ

p     ∈ R³    position (global frame)
v     ∈ R³    velocity (global frame)
R     ∈ R³ˣ³  rotor inclination matrix
ω_R   ∈ R³    angular velocity of inclination (roll, pitch, yaw)
θ     ∈ R     rotational advancement of blade 1
ω_θ   ∈ R     angular velocity of rotation (constant for given wind)
```

### Input vector:
```
u_r(t) = [β(t)]    β ∈ R³ (pitch angles of 3 blades)
```

### System dynamics ODE:
```
ẋ_r(t) = f_r(x_r, u_r) = [
    v(t),
    R(t) · F_rotor(t) · m⁻¹,
    R(t) · [ω_R(t)]_x,                              (with Baumgarte correction)
    J_r⁻¹ · (M_rotor(t) - ω_R(t) × (J_r · ω_R(t))),
    ω_θ,
    0
]
```

### Baumgarte correction (prevents R drift from orthonormality):
```
Ṙ(t) = R(t) · { (κ_R/2) · ((Rᵀ(t)R(t))⁻¹ - I) + [ω_R(t)]_x }
```

### Rotational speed (autorotation):
```
ω_θ = λ · V∞ / R_radius

λ = tip_speed / V∞    (tip speed ratio, typically λ=7 for wind turbines)
```

---

## 4. Flap Dynamics State Space Model

### States and input:
```
x_f,i(t) = [βi(t), ωi(t)]ᵀ     (blade pitch, pitch rate)
u_f,i(t) = [γi(t)]              (flap deflection)
```

### Dynamics:
```
ẋ_f,i = [
    ωi(t),
    Mi · Jb⁻¹ + Dω · ωi
]
```

### Linear form (A, B matrices):
```
ẋ_f,i = A · x_f,i + B · u_f,i

A = | 0                                    1  |
    | q∞·S·l·(CM,β + CM,γ)·Jb⁻¹           Dω |

B = | 0                          |
    | q∞·S·l·CM,γ·Jb⁻¹           |
```

---

## 5. Flap Controller Design

### Feed-forward with PID closed-loop correction:

Reference signal:
```
β̄(t) = U0 · sin(ω·t + Ψ)
```

Shifted reference (feed-forward, compensates phase lag and gain):
```
β̃(t) = (U0 / M) · sin(ω·t - Φ + Ψ)

M = |G(jω)|    (gain from transfer function)
Φ = arg(G(jω)) (phase shift from transfer function)
```

PID correction:
```
u_PID(t) = Kp·e(t) + KI·∫e(τ)dτ + KD·de/dt
e(t) = β̄(t) - β(t)
```

Controller output:
```
γ_CL(β̄, β) = β̃(t) + γ_PID(β̄(t), β(t))
```

Transfer function from system matrices:
```
G(s) = C·(sI - A)⁻¹·B + D
```

### System Identification Method (N4SID / subspace identification)
- Input: flap angle γ(t)
- Output: blade pitch β(t)
- Uses block Hankel matrices + SVD to identify A, B, C, D matrices from measurement data
- Python package: `nfoursid`
- Model RMSE on validation set: **1.4201°** over 30s

---

## 6. Extended State Space (Flap + RAWES integrated)

```
x(t) = [x_r(t), x_f,1(t), x_f,2(t), x_f,3(t)]ᵀ

u(t) = [β̄_1(t), β̄_2(t), β̄_3(t)]ᵀ    (reference pitch signals)

ẋ(t) = [
    f_r(x_r(t), β(t)),
    f_f,1(x_f,1(t), γ_CL(β̄_1, β_1)),
    f_f,2(x_f,2(t), γ_CL(β̄_2, β_2)),
    f_f,3(x_f,3(t), γ_CL(β̄_3, β_3))
]
```

---

## 7. Simulation Parameters

### Table 1 — RAWES Parameters
| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Air density | ρ | 1.22 | kg/m³ |
| Gravitational acceleration | g | 9.81 | m/s² |
| Wind velocity | \|\|v_w\|\| | 10 | m/s |
| Blade span | s | 1.5 | m |
| Blade area | S | 0.2 | m² |
| Zero lift coefficient | CL,0 | 0.11 | — |
| Linear lift coefficient | CL,α | 0.87 | — |
| Zero-lift drag coefficient | CD,0 | 0.007 | — |
| Quadratic drag coefficient | CD,α | 0.00013 | — |
| Rotor mass | m | 40 | kg |
| Rotor inertia tensor | Jr | [20,20,40]·I | kg·m² |
| Damping coefficient | D | 50 | N·m·s |
| Center of mass offset | d | -1 | m |
| Rotor angular velocity | ω_θ | 15π | rad/s |
| Baumgarte tuning factor | κ_R | 0.2 | — |

> ω_θ = 15π rad/s → tip speed ≈ 70 m/s at 1.5m span → tip speed ratio λ = 7 at 10 m/s wind

### Table 2 — Flap Dynamics Parameters
| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Chord length | l | 0.15 | m |
| Zero-lift moment coefficient | CM,0 | -0.08 | — |
| Linear moment coefficient | CM,α | 0.002 | — |
| Linear flap moment offset | CM,γ | -0.5 | — |
| Blade inertia | Jb | 1.20 | kg·m² |
| Damping coefficient | Dω | -13.5 | N·m·s |

### PID Controller Parameters
| Parameter | Value |
|-----------|-------|
| Kp | 2 |
| KI | 3 |
| KD | 0.2 |

---

## 8. Numerical Integration

- Method: **RK4** (Runge-Kutta 4th order)
- Step size: **Δt = 0.001 s** (RAWES model), **Δt = 0.013 s** (flap model, = sampling interval)
- Implementation: Python, NumPy + Matplotlib

---

## 9. Key Design Constraints & Findings

| Constraint | Value | Reason |
|-----------|-------|--------|
| Max rotor tilt for good tracking | **≤ 15°** | Controller error grows sharply beyond this |
| Tip speed ratio for good tracking | **λ ≥ 7** | Higher λ → more stable apparent wind → better pitch control |
| AoA for linear model validity | **< 15–20°** | Beyond this, CL becomes nonlinear (stall) |
| Controller settling time | **~1 s** | = 1 pitching cycle |
| PID RMSE improvement | 9.693° → 2.610° | With closed-loop correction over 6s |

### Known Limitations
1. Controller uses fixed system matrices identified without wind → does not adapt to changing wind conditions
2. Rotor tilt >15° causes significant pitch tracking error
3. Tip vortices, wake effects, induced drag neglected in model
4. Tether force not modeled (only tether stabilizing moment approximated)

### Future Improvements Suggested
- Continuously update system matrices (adaptive identification)
- Pre-compute matrices for multiple operating scenarios, select nearest match
- Implement trajectory-to-pitch MPC (section A)
- Controller code already rewritten in C for Arduino (servo motor actuation) — not yet tested on hardware at time of writing

---

## 10. Cyclic Pitch Signal Formulas

For blade i (i=1,2,3), a cyclic pitch input tilting the rotor in direction Ψ:
```
βi(t) = U0 · sin(ω_θ·t + (i-1)·2π/3 + Ψ)
```

Example signals used in simulation:
```
β¹_i(t) = 0                                            (no pitching)
β²_i(t) = 20·sin(ω_θ·t + (i-1)·2π/3)                 (Ψ=0)
β³_i(t) = 20·sin(ω_θ·t + (i-1)·2π/3 + π/2)           (Ψ=π/2)
```
Varying Ψ changes the tilt direction; varying U0 changes tilt magnitude.

---

## Source
Felix Weyel, "Modeling and Closed Loop Control of a Cyclic Pitch Actuated Rotary Airborne Wind Energy System", Bachelor's Thesis, Albert-Ludwigs-University Freiburg, Sept 2025.
Measurement data provided by Christof Beaupoil / someAWE Labs.
