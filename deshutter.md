# De Schutter, Leuthold, Diehl (2018) — Detailed Summary

**"Optimal Control of a Rigid-Wing Rotary Kite System for Airborne Wind Energy"**
Jochem De Schutter, Rachel Leuthold, Moritz Diehl — University of Freiburg

---

## 1. Purpose and Contribution

The paper demonstrates that a small-scale rigid-wing pumping RAWES can be controlled efficiently
at different wind speeds using **only pitch control** as on-board actuation. Optimal control
(Periodic Optimal Control Problem, POCP) is applied to compute pumping trajectories that
maximise average mechanical power output across different operating regimes. The design geometry
is optimised for a rated wind speed under structural constraints.

Key claim: pitch-only actuation is sufficient; no other on-board control surface is needed.

---

## 2. System Configuration

The rotary kite is a rigid body with **three wings** (120° apart), each of span L_w = 1.5 m,
connected by carbon fibre beams of length L_b to a central carbon fibre rod. The tether attaches
at the bottom of the central rod. Thin Dyneema cables (d_ck) connect wing centre-of-pressure
points to the central rod to compensate bending moments from lift.

The angle δ = 45° is chosen between the beam axis and the rotation axis. This sets how much
the lift vector tilts into the rotational plane vs. the axial direction.

The tether connects via a winch to a ground generator. Reel-out drives the generator;
reel-in resets the system. This is the **pumping cycle**.

---

## 3. System Dynamics

The model is a fully-implicit index-1 DAE (Eq. 1):

```
F(x(t), ẋ(t), u(t), z(t), θ) = 0
```

### State vector

```
x = (r, ṙ, R, ω, ψ, l, l̇, l̈, E)
```

| Symbol | Meaning |
|--------|---------|
| r ∈ ℝ³ | Position of kite centre of mass in inertial frame |
| R ∈ ℝ³×³ | Rotation matrix — body frame axes in inertial frame |
| ω ∈ ℝ³ | Angular velocity in **body frame** |
| ψ ∈ ℝ^(2n_ψ+1) | Cyclic pitch control parameters (Fourier coefficients) |
| l, l̇, l̈ | Tether length, speed, acceleration |
| E | Mechanical energy transferred to ground station |

### Controls

```
u = (l̈, ψ_ref)
```

- l̈ ∈ ℝ: tether jerk
- ψ_ref ∈ ℝ^(2n_ψ+1): reference values for the cyclic pitch controller

### Algebraic variables

```
z = (λ, a)
```

- λ: Lagrange multiplier for the tether length constraint (= tether tension force / l)
- a: induction factor (actuator-disk)

---

## 4. Tether Constraint

The tether is modelled as **inextensible, non-sagging**. The holonomic constraint is:

```
c(r, R, l) = ½ · ((r + R·r_T)ᵀ(r + R·r_T) − l²) = 0
```

where r_T is the tether attachment point in the body frame (bottom of central rod).

A Baumgarte stabilisation is applied:

```
c̈ + 2κċ + κ²c = 0
```

with tuning factor κ. This prevents constraint drift in the numerical solver.

The tether force is **λl** (tension positive when pulling). Instantaneous power to the ground:

```
Ė = P = λ·l̇
```

---

## 5. Pitch Control Architecture

### Parametrization (Eq. 23)

The pitch of wing k is a Fourier series in the rotation angle φ:

```
p_k(φ) = ψ₀
        + Σₘ (ψ₁ₘ/2) · sin(mφ + 2mπ(k−1)/3)
        + Σₘ (ψ₂ₘ/2) · cos(mφ + 2mπ(k−1)/3)
```

- **ψ₀ = collective pitch** (DC component, same for all blades)
- Sine/cosine terms = **cyclic pitch** (tilts the rotor disk)
- Phase offset = **2π/3 = 120°** for 3 wings
- Harmonic order **n_ψ = 1** chosen — order 3 only improves rated power by 1%

### Rotation angle φ (Eq. 24)

φ tracks the angular advancement of the kite around e'_y (the rotation axis):

```
φ = tan⁻¹( (−Rᵀe_x)ᵀe'_x / (−Rᵀe_z)ᵀe'_x )
```

This can be computed from an IMU by projecting the gravity vector into the plane of rotation.

### Closed-loop pitch controller (Eq. 2)

The actual pitch tracks the reference via a second-order closed-loop:

```
d/dt ψ = −(ψ − ψ_ref)/T²_ψ − 2c_a·ψ̇/T_ψ
```

| Parameter | Value |
|-----------|-------|
| Time constant T_a | 0.5 s |
| Damping coefficient c_a | 0.7 |

---

## 6. Atmosphere and Induction Model

### Wind shear (Eq. 15)

Logarithmic wind profile:

```
u_∞(z) = u_ref · log(z/z_r) / log(z₀/z_r)
```

| Parameter | Value |
|-----------|-------|
| Reference altitude z₀ | 100 m |
| Surface roughness z_r | 0.1 m |

### Axial induction (Eq. 16–17)

Available wind at the rotor is reduced by the induction factor a:

```
u_w = (1 − a) · u_∞(rᵀe_z)
```

The induction factor is determined by the actuator-disk (AD) momentum equation:

```
F_A^y · e_y = 2ρ(rᵀe_z) · u²_∞(rᵀe_z) · (a − a²) · A_K
```

where A_K = π((L_b + L_w)² − L²_b) is the actuator annulus area.

**AD validity constraints:** small spanwise velocity variation (L_b >> L_w) and small tilt
ξ = cos⁻¹(e'_y · R·e_y). The model breaks down at large tilt angles (>~45°), which occurs
during the reel-in phase. The paper accepts this because a → 0 during reel-in anyway.

---

## 7. Aerodynamics (Eq. 20–31)

For each wing k, the apparent wind velocity is:

```
u_{a,k} = u_w·e_y − ṙ − R(ω × r'_{cp,k})
```

where r'_{cp,k} is the centre of pressure of wing k in the body frame (at 2/3 of span from root).

### Lift and drag coefficients (Eq. 25)

From thin airfoil theory with elliptical span loading:

```
C_{L,k} = 2π / (1 + 2/R) · α_k
C_{D,k} = C_{D,0} + C²_{L,k} / (π·R·O_e)
```

| Parameter | Value |
|-----------|-------|
| Aspect ratio R | 12 |
| Oswald efficiency O_e | 0.8 |
| Parasitic drag C_{D,0} | 0.01 |

### Angle of attack (Eq. 26)

```
α_k ≈ (u_{a,k})ᵀ · e''_{3,k} / (u_{a,k})ᵀ · e''_{2,k}
```

### Side-slip angle (Eq. 27)

```
β_k = sin⁻¹( (u_{a,k})ᵀ · e''_{1,k} / ‖u_{a,k}‖ )
```

### AoA and side-slip constraints (Eq. 28)

```
−15° ≤ α_k ≤ 15°
−15° ≤ β_k ≤ 15°
```

These are hard OCP inequality constraints. They enforce linear aerodynamic model validity and
prevent flow separation.

### Lift and drag forces (Eq. 30–31)

```
L_k = ½ρ(rᵀe_z) · S_w · C_{L,k} · ‖u_{a,k}‖² · (u_{a,k} × e''_{1,k})
D_k = ½ρ(rᵀe_z) · S_w · (C_{D,k} + C_{D,T}) · ‖u_{a,k}‖₂ · u_{a,k}
```

A parasitic drag coefficient C_{D,T} accounts for the connecting beams and cables (Eq. 29).

The total generalised aerodynamic force and moment:

```
F_A = Σ F_k
M_A = Σ r'_{cp,k} × Rᵀ F_k
```

---

## 8. Structural Constraints

### Tether tension (Eq. 33)

```
λ ≥ 0                                    (tether must stay taut)
τ_max · πd²_c/4 − f_s·λ·l ≥ 0           (tension < break load / safety factor)
```

Safety factor **f_s = 10**. This means operating tension ≤ break load ÷ 10.

For a 1.9 mm Dyneema tether with ~620 N break load, maximum operating tension ≈ 62 N.

### Reinforcement cable (Eq. 34)

The cable from wing centre-of-pressure to central rod must compensate lift-induced bending:

```
τ_max · πd²_{ck}/4 · cos(δ) − f_s · F_A^y · e'_y ≥ 0
```

### Beam bending (Eq. 35)

Bending displacement at the beam-wing joint is bounded to 1% of beam length:

```
0.01·L_b − ‖F_{b,k}‖₂ · L³_b / (3·E_b·I_b) ≥ 0
```

where E_b = 250 GPa (carbon fibre Young's modulus) and I_b = πd⁴_b/64.

---

## 9. Periodic Optimal Control Problem (POCP)

### Objective (Eq. 41)

Minimise negative average power:

```
min  −P̄ = −E(T_p) / T_p
```

### Periodicity conditions (Eq. 37)

```
ẋ(0) = ẋ(T_p)    (adjusted state periodic)
E(0) = 0          (energy state starts at zero)
```

### Pumping cycle structure (Eq. 38–39)

The cycle period T_p is split into reel-out T₁ and reel-in T₂:

```
T_p = T₁ + T₂
l̇ ≥ 0  for t ∈ [0, T₁]       (reel-out)
l̇ ≤ 0  for t ∈ ]T₁, T_p]     (reel-in)
```

### Additional constraints (Eq. 40)

```
l(T₁) = l_max = 300 m         (reel to maximum tether length)
rᵀe_z ≥ z_min = 10 m          (remain above minimum altitude)
```

### Solver

Problem discretised by **direct collocation** (Radau scheme, polynomial order 4). Posed as NLP
in CasADi, solved with interior-point solver IPOPT using linear solver MA57.

---

## 10. System Parameters (Table I)

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Wing span | L_w | 1.5 | m |
| Aspect ratio | R | 12 | — |
| Oswald efficiency | O_e | 0.8 | — |
| Wing parasitic drag | C_{D,0} | 0.01 | — |
| Pitch harmonic order | n_ψ | 1 | — |
| Pitch time constant | T_a | 0.5 | s |
| Pitch damping coefficient | c_a | 0.7 | — |
| Cable material density | ρ_c | 1450 | kg/m³ |
| Cable tensile strength | τ_max | 3.6×10⁹ | Pa |
| Safety factor | f_s | 10 | — |
| Carbon fibre density | ρ_b | 1750 | kg/m³ |
| Carbon fibre Young modulus | E_b | 250×10⁹ | Pa |
| Reference wind altitude | z₀ | 100 | m |
| Surface roughness length | z_r | 0.1 | m |
| Max tether length | l_max | 300 | m |
| Min altitude | z_min | 10 | m |

---

## 11. Optimal Design Results (u_ref = 10 m/s)

Solving the POCP gives optimal design parameters θ*:

| Parameter | Value |
|-----------|-------|
| Central rod diameter d_c | 1.9 mm |
| Reinforcement cable diameter d_ck | 1.5 mm |
| Beam diameter d_b | 36 mm |
| Connecting beam length L_b | 1.60 m (≈ L_w) |
| Cycle period T_p | 2.44 s |
| Reel-out duration T₁ | 1.45 s (59% of cycle) |
| Mean reel-out factor f̄ | 0.38 |
| Power harvesting factor ζ̄ | 4.6 |

The harvesting factor of 4.6 is lower than conventional wind turbines (~5.5) but significantly
lower than the theoretical maximum for AWE (~30). RAWES trades peak efficiency for simplicity.

---

## 12. Optimal Pitch and Tilt Profiles (Fig. 4)

### At u_ref = 10 m/s (rated, solid line)

**Pitch profile p₁(t/T_p):**
- Reel-out: varies ~0° to +15°, mean collective ≈ +5–8°
- Reel-in: drops to ~−20°, mean collective ≈ −10 to −15°
- Profile is smooth — first harmonic (n_ψ = 1) is sufficient

**Rotor tilt ξ = cos⁻¹(e'_y · R·e_y):**
- Reel-out: **30–50°**, rotor axis pointed toward the wind — small tilt maintains autorotation
- Reel-in: **>70°**, rotor axis tilted sideways to reduce tether tension

### At u_ref = 20 m/s (above rated, dash line)

- Similar smooth pitch profile; reel-in pitch is lower (less drag needed)
- Rotor tilt does not vary greatly — structural constraints limit power increase

### At u_ref = 4 m/s (cut-in, dash-dot line)

- Very aggressive pitch oscillations
- Tilt oscillates rapidly between **30° and 130°**
- System must consume ~30% of rated power to stay airborne — reversed pumping

---

## 13. Three Operating Regimes (Fig. 3)

| Region | Wind speed | Behaviour |
|--------|-----------|-----------|
| **I — Reversed pumping** | < ~5.4 m/s | System consumes power to stay aloft. Aggressive pitch/tilt control |
| **II — Normal pumping** | ~5.4–10 m/s | Increasing power output. Qualitatively similar trajectories to rated case |
| **III — Structural limit** | > 10 m/s | Structural constraints limit power. Decreasing altitude, rotor axis points sideways |

Cut-in speed ≈ 5.4 m/s. Power consumed at 4 m/s ≈ 30% of rated output.

---

## 14. Relevance to This Repo (4-Blade, 2 m System)

### Direct applicability

| Paper result | Applicable to repo |
|-------------|-------------------|
| ±15° AoA constraint | Already implemented in `aero.py` |
| Collective pitch +5–8° during autorotation | Target value for BEM model tuning |
| n_ψ = 1 sufficient for control | Justifies simple cyclic pitch in `swashplate.py` |
| Tether safety factor 10× | Operating tension limit: ≤ 62 N for 1.9 mm Dyneema |
| Reel-out = high collective, reel-in = low collective | Informs future ArduPilot mode transitions |

### Adaptations needed for 4-blade system

| Item | Paper (3 blades) | This repo (4 blades) |
|------|-----------------|---------------------|
| Phase offset | 2π/3 = 120° | **π/2 = 90°** |
| Fourier series offset term | 2mπ(k−1)/3 | **mπ(k−1)/2** |
| Actuator annulus area | 3-blade geometry | Needs update for 4-blade, 2 m span |
| Beam/cable sizing | L_b ≈ 1.6 m | Different geometry — recalculate |

### Limitations of paper model vs. current `aero.py`

- Paper uses thin airfoil theory (no viscous effects) — `aero.py` uses empirical CL/CD
- Paper models induction factor a explicitly — `aero.py` uses actuator-disk momentum theory separately
- Paper neglects aerodynamic moments — `aero.py` computes cyclic moments from swashplate tilt
- Paper assumes rigid inextensible tether — `aero.py`/`mediator.py` use elastic Dyneema model

### Parameters NOT in paper (gaps for this repo)

- Flap dynamics (trailing-edge flap → blade pitch lag) — covered in Weyel 2025 instead
- Anti-rotation motor — not applicable (paper has no rotating/non-rotating body split)
- Swashplate mixing — not applicable (paper abstracts away actuation)
- Jump takeoff / tether-assisted landing — not addressed

---

## 15. Equations Quick Reference

| Equation | Content |
|----------|---------|
| Eq. 2 | Pitch closed-loop dynamics |
| Eq. 15 | Logarithmic wind shear |
| Eq. 16–17 | Actuator-disk induction |
| Eq. 23 | Cyclic pitch Fourier parametrization |
| Eq. 24 | Rotation angle φ from IMU gravity projection |
| Eq. 25 | 3D lift and drag coefficients |
| Eq. 26–27 | Angle of attack and side-slip |
| Eq. 28 | ±15° AoA and side-slip hard constraints |
| Eq. 33–35 | Structural constraints (tether, cable, beam bending) |
| Eq. 41 | POCP objective (maximise average power) |
