# De Schutter, Leuthold, Diehl (2018) -- Detailed Summary

**"Optimal Control of a Rigid-Wing Rotary Kite System for Airborne Wind Energy"**
Jochem De Schutter, Rachel Leuthold, Moritz Diehl -- University of Freiburg

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

The rotary kite is a rigid body with **three wings** (120 deg apart), each of span L_w = 1.5 m,
connected by carbon fibre beams of length L_b to a central carbon fibre rod. The tether attaches
at the bottom of the central rod. Thin Dyneema cables (d_ck) connect wing centre-of-pressure
points to the central rod to compensate bending moments from lift.

The angle delta = 45 deg is chosen between the beam axis and the rotation axis. This sets how much
the lift vector tilts into the rotational plane vs. the axial direction.

The tether connects via a winch to a ground generator. Reel-out drives the generator;
reel-in resets the system. This is the **pumping cycle**.

---

## 3. System Dynamics

The model is a fully-implicit index-1 DAE (Eq. 1):

```
F(x(t), x_dot(t), u(t), z(t), theta) = 0
```

### State vector

```
x = (r, r_dot, R, omega, psi, l, l_dot, l_ddot, E)
```

| Symbol | Meaning |
|--------|---------|
| r in R^3 | Position of kite centre of mass in inertial frame |
| R in R^3x^3 | Rotation matrix -- body frame axes in inertial frame |
| omega in R^3 | Angular velocity in **body frame** |
| psi in R^(2n_psi+1) | Cyclic pitch control parameters (Fourier coefficients) |
| l, l_dot, l_ddot | Tether length, speed, acceleration |
| E | Mechanical energy transferred to ground station |

### Controls

```
u = (l_ddot, psi_ref)
```

- l_ddot in R: tether jerk
- psi_ref in R^(2n_psi+1): reference values for the cyclic pitch controller

### Algebraic variables

```
z = (lambda, a)
```

- lambda: Lagrange multiplier for the tether length constraint (= tether tension force / l)
- a: induction factor (actuator-disk)

---

## 4. Tether Constraint

The tether is modelled as **inextensible, non-sagging**. The holonomic constraint is:

```
c(r, R, l) = 0.5 . ((r + R.r_T)T(r + R.r_T) - l^2) = 0
```

where r_T is the tether attachment point in the body frame (bottom of central rod).

A Baumgarte stabilisation is applied:

```
c_ddot + 2kappac_dot + kappa^2c = 0
```

with tuning factor kappa. This prevents constraint drift in the numerical solver.

The tether force is **lambdal** (tension positive when pulling). Instantaneous power to the ground:

```
E_dot = P = lambda.l_dot
```

---

## 5. Pitch Control Architecture

### Parametrization (Eq. 23)

The pitch of wing k is a Fourier series in the rotation angle phi:

```
p_k(phi) = psi_0
        + Sum_m (psi_1_m/2) . sin(mphi + 2mpi(k-1)/3)
        + Sum_m (psi_2_m/2) . cos(mphi + 2mpi(k-1)/3)
```

- **psi_0 = collective pitch** (DC component, same for all blades)
- Sine/cosine terms = **cyclic pitch** (tilts the rotor disk)
- Phase offset = **2pi/3 = 120 deg** for 3 wings
- Harmonic order **n_psi = 1** chosen -- order 3 only improves rated power by 1%

### Rotation angle phi (Eq. 24)

phi tracks the angular advancement of the kite around e'_y (the rotation axis):

```
phi = tan^-^1( (-RTe_x)Te'_x / (-RTe_z)Te'_x )
```

This can be computed from an IMU by projecting the gravity vector into the plane of rotation.

### Closed-loop pitch controller (Eq. 2)

The actual pitch tracks the reference via a second-order closed-loop:

```
d/dt psi = -(psi - psi_ref)/T^2_psi - 2c_a.psi_dot/T_psi
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
u_inf(z) = u_ref . log(z/z_r) / log(z_0/z_r)
```

| Parameter | Value |
|-----------|-------|
| Reference altitude z_0 | 100 m |
| Surface roughness z_r | 0.1 m |

### Axial induction (Eq. 16-17)

Available wind at the rotor is reduced by the induction factor a:

```
u_w = (1 - a) . u_inf(rTe_z)
```

The induction factor is determined by the actuator-disk (AD) momentum equation:

```
F_A^y . e_y = 2rho(rTe_z) . u^2_inf(rTe_z) . (a - a^2) . A_K
```

where A_K = pi((L_b + L_w)^2 - L^2_b) is the actuator annulus area.

**AD validity constraints:** small spanwise velocity variation (L_b >> L_w) and small tilt
xi = cos^-^1(e'_y . R.e_y). The model breaks down at large tilt angles (>~45 deg), which occurs
during the reel-in phase. The paper accepts this because a -> 0 during reel-in anyway.

---

## 7. Aerodynamics (Eq. 20-31)

For each wing k, the apparent wind velocity is:

```
u_{a,k} = u_w.e_y - r_dot - R(omega x r'_{cp,k})
```

where r'_{cp,k} is the centre of pressure of wing k in the body frame (at 2/3 of span from root).

### Lift and drag coefficients (Eq. 25)

From thin airfoil theory with elliptical span loading:

```
C_{L,k} = 2pi / (1 + 2/R) . alpha_k
C_{D,k} = C_{D,0} + C^2_{L,k} / (pi.R.O_e)
```

| Parameter | Value |
|-----------|-------|
| Aspect ratio R | 12 |
| Oswald efficiency O_e | 0.8 |
| Parasitic drag C_{D,0} | 0.01 |

### Angle of attack (Eq. 26)

```
alpha_k ~= (u_{a,k})T . e''_{3,k} / (u_{a,k})T . e''_{2,k}
```

### Side-slip angle (Eq. 27)

```
beta_k = sin^-^1( (u_{a,k})T . e''_{1,k} / ||u_{a,k}|| )
```

### AoA and side-slip constraints (Eq. 28)

```
-15 deg <= alpha_k <= 15 deg
-15 deg <= beta_k <= 15 deg
```

These are hard OCP inequality constraints. They enforce linear aerodynamic model validity and
prevent flow separation.

### Lift and drag forces (Eq. 30-31)

```
L_k = 0.5rho(rTe_z) . S_w . C_{L,k} . ||u_{a,k}||^2 . (u_{a,k} x e''_{1,k})
D_k = 0.5rho(rTe_z) . S_w . (C_{D,k} + C_{D,T}) . ||u_{a,k}||_2 . u_{a,k}
```

A parasitic drag coefficient C_{D,T} accounts for the connecting beams and cables (Eq. 29).

The total generalised aerodynamic force and moment:

```
F_A = Sum F_k
M_A = Sum r'_{cp,k} x RT F_k
```

---

## 8. Structural Constraints

### Tether tension (Eq. 33)

```
lambda >= 0                                    (tether must stay taut)
tau_max . pid^2_c/4 - f_s.lambda.l >= 0           (tension < break load / safety factor)
```

Safety factor **f_s = 10**. This means operating tension <= break load / 10.

For a 1.9 mm Dyneema tether with ~620 N break load, maximum operating tension ~= 62 N.

### Reinforcement cable (Eq. 34)

The cable from wing centre-of-pressure to central rod must compensate lift-induced bending:

```
tau_max . pid^2_{ck}/4 . cos(delta) - f_s . F_A^y . e'_y >= 0
```

### Beam bending (Eq. 35)

Bending displacement at the beam-wing joint is bounded to 1% of beam length:

```
0.01.L_b - ||F_{b,k}||_2 . L^3_b / (3.E_b.I_b) >= 0
```

where E_b = 250 GPa (carbon fibre Young's modulus) and I_b = pid^4_b/64.

---

## 9. Periodic Optimal Control Problem (POCP)

### Objective (Eq. 41)

Minimise negative average power:

```
min  -P_bar = -E(T_p) / T_p
```

### Periodicity conditions (Eq. 37)

```
x_dot(0) = x_dot(T_p)    (adjusted state periodic)
E(0) = 0          (energy state starts at zero)
```

### Pumping cycle structure (Eq. 38-39)

The cycle period T_p is split into reel-out T_1 and reel-in T_2:

```
T_p = T_1 + T_2
l_dot >= 0  for t in [0, T_1]       (reel-out)
l_dot <= 0  for t in ]T_1, T_p]     (reel-in)
```

### Additional constraints (Eq. 40)

```
l(T_1) = l_max = 300 m         (reel to maximum tether length)
rTe_z >= z_min = 10 m          (remain above minimum altitude)
```

### Solver

Problem discretised by **direct collocation** (Radau scheme, polynomial order 4). Posed as NLP
in CasADi, solved with interior-point solver IPOPT using linear solver MA57.

---

## 10. System Parameters (Table I)

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Wing span | L_w | 1.5 | m |
| Aspect ratio | R | 12 | -- |
| Oswald efficiency | O_e | 0.8 | -- |
| Wing parasitic drag | C_{D,0} | 0.01 | -- |
| Pitch harmonic order | n_psi | 1 | -- |
| Pitch time constant | T_a | 0.5 | s |
| Pitch damping coefficient | c_a | 0.7 | -- |
| Cable material density | rho_c | 1450 | kg/m^3 |
| Cable tensile strength | tau_max | 3.6x10^9 | Pa |
| Safety factor | f_s | 10 | -- |
| Carbon fibre density | rho_b | 1750 | kg/m^3 |
| Carbon fibre Young modulus | E_b | 250x10^9 | Pa |
| Reference wind altitude | z_0 | 100 | m |
| Surface roughness length | z_r | 0.1 | m |
| Max tether length | l_max | 300 | m |
| Min altitude | z_min | 10 | m |

---

## 11. Optimal Design Results (u_ref = 10 m/s)

Solving the POCP gives optimal design parameters theta*:

| Parameter | Value |
|-----------|-------|
| Central rod diameter d_c | 1.9 mm |
| Reinforcement cable diameter d_ck | 1.5 mm |
| Beam diameter d_b | 36 mm |
| Connecting beam length L_b | 1.60 m (~= L_w) |
| Cycle period T_p | 2.44 s |
| Reel-out duration T_1 | 1.45 s (59% of cycle) |
| Mean reel-out factor f_bar | 0.38 |
| Power harvesting factor zeta_bar | 4.6 |

The harvesting factor of 4.6 is lower than conventional wind turbines (~5.5) but significantly
lower than the theoretical maximum for AWE (~30). RAWES trades peak efficiency for simplicity.

---

## 12. Optimal Pitch and Tilt Profiles (Fig. 4)

### At u_ref = 10 m/s (rated, solid line)

**Pitch profile p_1(t/T_p):**
- Reel-out: varies ~0 deg to +15 deg, mean collective ~= +5-8 deg
- Reel-in: drops to ~-20 deg, mean collective ~= -10 to -15 deg
- Profile is smooth -- first harmonic (n_psi = 1) is sufficient

**Rotor tilt xi = cos^-^1(e'_y . R.e_y):**
- Reel-out: **30-50 deg**, rotor axis pointed toward the wind -- small tilt maintains autorotation
- Reel-in: **>70 deg**, rotor axis tilted sideways to reduce tether tension

### At u_ref = 20 m/s (above rated, dash line)

- Similar smooth pitch profile; reel-in pitch is lower (less drag needed)
- Rotor tilt does not vary greatly -- structural constraints limit power increase

### At u_ref = 4 m/s (cut-in, dash-dot line)

- Very aggressive pitch oscillations
- Tilt oscillates rapidly between **30 deg and 130 deg**
- System must consume ~30% of rated power to stay airborne -- reversed pumping

---

## 13. Three Operating Regimes (Fig. 3)

| Region | Wind speed | Behaviour |
|--------|-----------|-----------|
| **I -- Reversed pumping** | < ~5.4 m/s | System consumes power to stay aloft. Aggressive pitch/tilt control |
| **II -- Normal pumping** | ~5.4-10 m/s | Increasing power output. Qualitatively similar trajectories to rated case |
| **III -- Structural limit** | > 10 m/s | Structural constraints limit power. Decreasing altitude, rotor axis points sideways |

Cut-in speed ~= 5.4 m/s. Power consumed at 4 m/s ~= 30% of rated output.

---

## 14. Relevance to This Repo (4-Blade, 2 m System)

### Direct applicability

| Paper result | Applicable to repo |
|-------------|-------------------|
| +-15 deg AoA constraint | Already implemented in `aero.py` |
| Collective pitch +5-8 deg during autorotation | Target value for BEM model tuning |
| n_psi = 1 sufficient for control | Justifies simple cyclic pitch in `swashplate.py` |
| Tether safety factor 10x | Operating tension limit: <= 62 N for 1.9 mm Dyneema |
| Reel-out = high collective, reel-in = low collective | Informs future ArduPilot mode transitions |

### Adaptations needed for 4-blade system

| Item | Paper (3 blades) | This repo (4 blades) |
|------|-----------------|---------------------|
| Phase offset | 2pi/3 = 120 deg | **pi/2 = 90 deg** |
| Fourier series offset term | 2mpi(k-1)/3 | **mpi(k-1)/2** |
| Actuator annulus area | 3-blade geometry | Needs update for 4-blade, 2 m span |
| Beam/cable sizing | L_b ~= 1.6 m | Different geometry -- recalculate |

### Limitations of paper model vs. current `aero.py`

- Paper uses thin airfoil theory (no viscous effects) -- `aero.py` uses empirical CL/CD
- Paper models induction factor a explicitly -- `aero.py` uses actuator-disk momentum theory separately
- Paper neglects aerodynamic moments -- `aero.py` computes cyclic moments from swashplate tilt
- Paper assumes rigid inextensible tether -- `aero.py`/`mediator.py` use elastic Dyneema model

### Parameters NOT in paper (gaps for this repo)

- Flap dynamics (trailing-edge flap -> blade pitch lag) -- covered in Weyel 2025 instead
- Anti-rotation motor -- not applicable (paper has no rotating/non-rotating body split)
- Swashplate mixing -- not applicable (paper abstracts away actuation)
- Jump takeoff / tether-assisted landing -- not addressed

---

## 15. Equations Quick Reference

| Equation | Content |
|----------|---------|
| Eq. 2 | Pitch closed-loop dynamics |
| Eq. 15 | Logarithmic wind shear |
| Eq. 16-17 | Actuator-disk induction |
| Eq. 23 | Cyclic pitch Fourier parametrization |
| Eq. 24 | Rotation angle phi from IMU gravity projection |
| Eq. 25 | 3D lift and drag coefficients |
| Eq. 26-27 | Angle of attack and side-slip |
| Eq. 28 | +-15 deg AoA and side-slip hard constraints |
| Eq. 33-35 | Structural constraints (tether, cable, beam bending) |
| Eq. 41 | POCP objective (maximise average power) |
