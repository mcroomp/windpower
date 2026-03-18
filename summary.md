# RAWES Simulation Design Summary
**Based on:** "Optimal Control of a Rigid-Wing Rotary Kite System for Airborne Wind Energy"
— De Schutter, Leuthold, Diehl (2018)

---

## 1. Concept Overview

A **Rotary Airborne Wind Energy System (RAWES)** is a tethered, rotating multi-wing kite that operates in **pumping mode**:
- The kite flies crosswind at altitude, rotating like a horizontal rotor
- A tether connects it to a ground winch/generator
- **Reel-out phase**: tether extends, tether tension drives the generator (power generation)
- **Reel-in phase**: tether retracts at low force (power consumed, net negative)
- Net energy = energy out − energy in over one cycle

The design trades efficiency for simplicity: only **pitch control** is needed on-board (no lateral steering, no roll control).

---

## 2. Physical Configuration

```
3 wings (120° apart)
    └─ connected by carbon fiber beams (length Lb, diameter db) to a central rod
    └─ reinforcement cables (diameter dck) from wing center-of-pressure to central rod
        └─ central rod attached at bottom to tether (diameter dc)
            └─ tether runs to ground winch → generator
```

### Key Geometry
| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Wing span | Lw | 1.5 | m |
| Aspect ratio | A | 12 | — |
| Beam length | Lb | 1.60 | m |
| Tether diameter | dc | 1.9 | mm |
| Reinf. cable diameter | dck | 1.5 | mm |
| Beam diameter | db | 36 | mm |
| Max tether length | lmax | 300 | m |
| Min altitude | zmin | 10 | m |
| Beam-rod angle | δ | 45° | — |

### Mass Model
```
mK = 3 * m0_S * Sw  +  ρb * (3*Lb*(π*db²/4) + Lr*(π*dr²/4))
```
where `Sw = Lw² / A` is wing area, `m0_S` is mass per wing area, `ρb` is carbon fiber density (1750 kg/m³).

Central rod length: `Lr = (Lb + cp*Lw) / cos(δ)`, with `cp` = relative chordwise center-of-pressure position.

---

## 3. State Variables

The full system state vector is:
```
x = (r, ṙ, R, ω, ψ, ψ̇, l, l̇, l̈, E)
```

| Variable | Dimension | Description |
|----------|-----------|-------------|
| r | ℝ³ | Position of kite center of mass (inertial frame) |
| ṙ | ℝ³ | Velocity of kite center of mass |
| R | ℝ³ˣ³ | Rotation matrix: body frame axes in inertial frame |
| ω | ℝ³ | Angular velocity (body frame) |
| ψ | ℝ^(2nψ+1) | Cyclic pitch control Fourier coefficients |
| ψ̇ | ℝ^(2nψ+1) | Pitch coefficient derivatives |
| l | ℝ | Tether length |
| l̇ | ℝ | Tether speed (positive = reel-out) |
| l̈ | ℝ | Tether acceleration |
| E | ℝ | Cumulative mechanical energy transferred to ground |

### Control Inputs
```
u = (l⃛,  ψ_ref)
```
- `l⃛` — tether jerk (third derivative of tether length)
- `ψ_ref` — reference pitch Fourier coefficients (input to pitch actuator model)

---

## 4. Equations of Motion

### Tether Constraint
The kite's tether attachment point is constrained to a sphere of radius `l`:
```
c(r, R, l) = ½ * ((r + R*rT)ᵀ(r + R*rT) − l²) = 0
```
where `rT` is the tether attachment point in the body frame.

### Lagrangian (with tether inertia approximation)
```
T = ½(mK + mT/3) * ṙᵀṙ  +  ½ωᵀJω
V = (mK + mT/2) * rᵀez
L = T − V + λᵀc
```
Tether mass: `mT = ρT * l * π*dc²/4`

### Translational Dynamics
```
d/dt(∂L/∂ṙ) − ∂L/∂r = FA
```

### Rotational Dynamics
```
J * dω/dt + ω × (J*ω) = MA − λ(rT × Rᵀr)
```

### Tether Constraint (Baumgarte stabilized)
```
c̈ + 2κċ + κ²c = 0
```

### Rotation Matrix Kinematics (Baumgarte stabilized)
```
dR/dt = R * (κR/2 * (I − RᵀR) + [ω]×)
```

### Power Output
```
Ė = P = λ * l̇ * l
```
Average power over cycle: `P̄ = E(Tp) / Tp`

### Pitch Actuator (closed-loop second-order)
```
d/dt(ψ̇) = −(ψ − ψ_ref)/Tψ²  −  2*cψ*ψ̇/Tψ
```
| Parameter | Symbol | Value |
|-----------|--------|-------|
| Time constant | Tψ | 0.5 s |
| Damping | cψ | 0.7 |

---

## 5. Cyclic Pitch Control Parametrization

Each wing's pitch angle is a Fourier series in the rotation angle φ:
```
pk(φ) = ψ0  +  Σ[m=1..nψ]  ψ1m/2 * sin(mφ + 2mπ/3*(k−1))
                           +  ψ2m/2 * cos(mφ + 2mπ/3*(k−1))
```
- `φ` = rotation advancement angle (tracked via IMU from gravity projection)
- Wings are offset by 120° from each other (k=1,2,3)
- **nψ = 1** (first harmonic only) — sufficient, higher harmonics add <1% power

Rotation angle φ:
```
φ = atan2( (−Rᵀez)ᵀ e'x,  (−Rᵀez)ᵀ e'z )
```

---

## 6. Aerodynamics

### Apparent Wind at Wing k
```
ua,k = uw*ey  −  ṙ  −  R*(ω × r'cp,k)
```
where `r'cp,k` = center of pressure of wing k in body frame (at 2/3 of wing span from root).

### Wing Orientation Vectors
```
e''1,k = R * Ry(2π/3*(k−1)) * e'x        (spanwise, root to tip)
e''2,k = R * Ry(2π/3*(k−1)) * Rx(pk) * (−e'z)   (chordwise, trailing to leading)
```

### Angle of Attack & Side Slip
```
αk ≈ (ua,k)ᵀ e''3,k / (ua,k)ᵀ e''2,k       (small angle approx)
βk = arcsin( (ua,k)ᵀ e''1,k / ‖ua,k‖ )
```
Constraints to prevent stall: `|αk| ≤ 15°`, `|βk| ≤ 15°`

### Lift and Drag Coefficients (thin airfoil, elliptical wing)
```
CL,k = (2π / (1 + 2/A)) * αk
CD,k = CD,0 + CL,k² / (π * A * Oe)
```
| Parameter | Symbol | Value |
|-----------|--------|-------|
| Parasitic drag | CD,0 | 0.01 |
| Oswald efficiency | Oe | 0.8 |

### Parasitic Drag from Structure
```
CD,T = ¼ * (CD,b*db*Lb + CD,ck*dck*Lck) / Sw
```
- Beam: `CD,b = 0.08` (streamlined fairing, cb = 3db)
- Cable: `CD,ck = 1.0` (cylindrical)

### Forces per Wing
```
Lk = ½ * ρ(z) * Sw * CL,k * ‖ua,k‖² * (ua,k × e''1,k)
Dk = ½ * ρ(z) * Sw * (CD,k + CD,T) * ‖ua,k‖² * ua,k
Fk = Lk + Dk
```

### Total Aerodynamic Force and Moment
```
FA = Σ Fk
MA = Σ r'cp,k × Rᵀ Fk
```

---

## 7. Atmosphere Model

### Wind Speed (logarithmic shear)
```
u∞(z) = uref * log(z/zr) / log(z0/zr)
```
| Parameter | Symbol | Value |
|-----------|--------|-------|
| Reference altitude | z0 | 100 m |
| Roughness length | zr | 0.1 m |
| Rated wind speed | uref | 10 m/s |

### Axial Induction (Actuator Disk)
Available wind at wings: `uw = (1 − a) * u∞(z)`

Induction factor `a` from:
```
FAᵀ e'y = 2 * ρ(z) * u∞(z)² * (a − a²) * AK
AK = π * ((Lb + Lw)² − Lb²)       (annular actuator disk area)
0 ≤ a ≤ 0.5
```

### Atmospheric Density
Use International Standard Atmosphere (ISA) model: density decreases with altitude.

---

## 8. Pumping Cycle

The system operates in two phases per cycle (period Tp):

| Phase | Duration | Tether | Power |
|-------|----------|--------|-------|
| Reel-out | T1 ≈ 1.45 s | l̇ ≥ 0, extends to lmax | Generated (high) |
| Reel-in | T2 | l̇ ≤ 0, retracts | Consumed (low) |

**Key insight:** High pitch during reel-out (maximize lift/tether tension), low pitch during reel-in (minimize drag/tether tension).

**Reel-out factor:** `f = l̇ / u∞(z)`  — optimal near f ≈ 1/3 (Loyd's prediction), actual ≈ 0.38.

---

## 9. Structural Constraints

### Tether tension
```
λ ≥ 0
τmax * π*dc²/4  −  fs * λ * l  ≥  0
```

### Reinforcement cable (prevent bending along rotation axis)
```
τmax * π*dck²/4 * cos(δ)  −  fs * Fkᵀ e'y  ≥  0
```

### Beam bending (in plane of rotation, max 1% of beam length)
```
0.01*Lb  −  ‖Fb,k‖ * Lb³/(3*Eb*Ib)  ≥  0
Ib = π*db⁴/64       (second moment of area)
Eb = 250 GPa         (carbon fiber Young's modulus)
```

**Safety factor:** `fs = 10`
**Tether tensile strength:** `τmax = 3.6 × 10⁹ Pa`

---

## 10. Operating Regimes

| Region | Wind Speed | Behavior |
|--------|-----------|----------|
| I (below cut-in) | < 5.4 m/s | Consumes power to stay airborne ("reversed pumping") |
| II (normal) | 5.4–~15 m/s | Increasing power output, maximize extraction |
| III (above rated) | > ~15 m/s | Structural limits active; fly lower, tilt rotor sideways |

**Rated output:** ~3 kW at uref = 10 m/s
**Harvesting factor:** ζ̄ ≈ 4.6 (vs ~5.5 for conventional turbines)
**Cycle period:** Tp ≈ 2.44 seconds

---

## 11. Key Simplifications for Initial Simulation

For a first simulation, the following simplifications are reasonable:

1. **Inelastic, non-sagging tether** — model as rigid length constraint
2. **No tether dynamics** — lump tether mass into kite (mT/3 for KE, mT/2 for PE)
3. **No aerodynamic moments** — rotational dynamics driven entirely by force moments
4. **Thin airfoil theory** — simple CL/CD from angle of attack only
5. **Uniform wind field** — ignore shear initially, add logarithmic shear later
6. **nψ = 1** — only first Fourier harmonic for pitch control (5 parameters: ψ0, ψ11, ψ21)
7. **Ignore induction** — set a = 0 initially, add actuator disk model later
8. **Fixed design parameters** — use Table I values, no design optimization needed

---

## 12. Coordinate Frames

- **Inertial frame** `{ex, ey, ez}`: ground station origin, `ez` points up, `ey` points downwind
- **Body frame** `{e'x, e'y, e'z}`: attached to kite, `e'y` is rotation axis (nominally aligned with wind)
- Wing k=1 is along `e'x` in body frame; wings 2 and 3 are at +120° and +240°

---

## 13. Simulation Implementation Checklist

- [ ] Coordinate system and rotation matrix representation
- [ ] Kite mass and inertia tensor computation
- [ ] Tether constraint (holonomic, Baumgarte stabilized)
- [ ] Translational dynamics (Lagrangian)
- [ ] Rotational dynamics (Lagrangian on SO(3))
- [ ] Cyclic pitch parametrization (Fourier, nψ=1)
- [ ] Pitch actuator second-order dynamics
- [ ] Apparent wind velocity per wing
- [ ] Wing orientation vectors from rotation angle φ
- [ ] Angle of attack and side slip computation
- [ ] Lift/drag coefficients (thin airfoil + induced drag)
- [ ] Structural parasitic drag
- [ ] Aerodynamic forces and moments (sum over 3 wings)
- [ ] Logarithmic wind shear model
- [ ] Atmospheric density (ISA)
- [ ] Actuator disk induction factor
- [ ] Power/energy state equation
- [ ] Pumping cycle phases (reel-out / reel-in)
- [ ] Structural inequality constraints (tether, cable, beam bending)
