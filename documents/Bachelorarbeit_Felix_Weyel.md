# Bachelorarbeit Felix Weyel (2025) — Data Summary

**Title:** Modeling and Closed Loop Control of a Cyclic Pitch Actuated Rotary Airborne Wind Energy System
**Author:** Felix Weyel
**Institution:** Albert-Ludwins-University Freiburg, System Control and Optimization Laboratory
**Date:** September 4, 2025
**Hardware partner:** someAWE Labs (Christof Beaupoil provided measurement data)

---

## Simulation Reference Rotor (Table 1, p. 24)

This is the **thesis simulation model** — a 3-blade reference rotor based on De Schutter et al. [8].
These are preliminary estimates, **not** the Beaupoil hardware.

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Air density | ρ | 1.22 | kg/m³ |
| Wind velocity | ‖v_w‖ | 10 | m/s |
| Blade span | s | 1.5 | m (per blade) |
| Blade area | S | 0.2 | m² (per blade) |
| Zero-lift lift coefficient | C_L,0 | 0.11 | — |
| Lift slope | C_L,α | 0.87 | /rad |
| Zero-lift drag coefficient | C_D,0 | 0.007 | — |
| Quadratic drag coefficient | C_D,α | 0.00013 | /rad² |
| Rotor mass | m | 40 | kg |
| Rotor inertia | J_r | [20, 20, 40] | kg·m² |
| Angular damping | D | 50 | N·m·s |
| CM offset (tether arm) | d | −1.0 | m |
| Rotor spin rate | ω_θ | 15π ≈ 47.1 | rad/s |

**Drag polar form used:** C_D = C_D,0 + C_D,α × α²  (quadratic in α, not Oswald form)

---

## Airfoil (p. 7–8, Figure 4)

**Airfoil used in thesis:** SG6040 (root airfoil of the SG6040–SG6043 series)
- Coefficients read **visually from airfoiltools polar plots** at Re ≈ 2×10⁵–5×10⁵
- Linear approximation valid for −20° < α < 20° for C_L and C_M
- C_D approximated as quadratic in α

**Note:** C_L,0 = 0.11 and C_L,α = 0.87 are approximate visual reads from SG6040 polars,
not a precise fit or wind tunnel measurement.

---

## Flap Dynamics Parameters (Table 2, p. 36)

Govern blade-pitch response to flap deflection γ.

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Chord length | l | 0.15 | m |
| Zero-lift moment coefficient | C_M,0 | −0.08 | — |
| Moment slope vs AoA | C_M,α | 0.002 | /rad |
| **Flap moment effectiveness** | **C_M,γ** | **−0.5** | **/rad** |
| Blade pitch inertia | J_b | 1.20 | kg·m² |
| Aerodynamic pitch damping | D_ω | −13.5 | N·m·s |

**Sign convention (p. 11–12, Fig. 7):**
- γ < 0 (upward flap) → positive moment → blade pitches UP
- C_M,γ = −0.5: positive flap deflection produces negative pitching moment

**Pitching moment equation (p. 29, eq. 62):**
```
M_i = q_∞ · S · l · (C_M,β · α_i + C_M,γ · α_f,i)
```
where α_f = α + γ (blade AoA + flap deflection).

---

## Cyclic Pitch Control

**Reference signal (eq. 59, p. 26):**
```
β_i(t) = 20° · sin(ω_θ·t + (i−1)·2π/3)
```
- 3 blades, 120° phase separation
- Amplitude ±20° in demonstration simulations

**PID controller gains (p. 32, Section 5.5):**

| Gain | Value |
|------|-------|
| K_P | 2 |
| K_I | 3 |
| K_D | 0.2 |

**Controller performance:**
- RMSE open-loop: 9.693°
- RMSE closed-loop PID: 2.610° (t_N = 6 s)
- RMSE with system ID: 1.401° (t_N = 30 s)

**Tilt limit:** Controller degrades significantly above 15° tilt; not valid above 30–45°.

---

## Experimental Setup (Section 5.1, p. 28)

- Single blade + flap rotated by brushless motor; flap by servo motor
- Rotation speed: 40–66 RPM
- No external wind (enclosed environment)
- Pitch measured by electromagnetic induction sensor
- Sampling interval: Δt = 0.013 s (≈ 77 Hz)
- System identification: N4SID subspace method (Python `nfoursid` package)

---

## Key Equations

**Angle of attack (eqs. 15–16, p. 10):**
```
φ = atan(v_z / v_y)       (inflow angle)
α = β + φ                  (AoA = blade pitch + inflow)
```

**Apparent wind in blade frame (eq. 39, p. 20):**
```
v_ap,i = e''_y · r · ω_θ − R_θ,i · Rᵀ · v_w
```

**Tether stabilising moment (eq. 44, p. 21):**
```
M_tether = −e'_z · d × F_G     (d = −1 m centre-of-mass offset)
```

**State space for flap dynamics (eqs. 63–65, p. 28–29):**
```
x_f,i = [β_i, ω_i]ᵀ     (blade pitch, pitch rate)
u_f,i = [γ_i]             (flap deflection)
A = [[0, 1], [q·S·l·(C_M,β + C_M,γ)/J, D_ω/J]]
B = [[0], [q·S·l·C_M,γ/J]]
```

**Integration timestep:** Δt = 0.001 s

---

## What Is NOT in This Document

- No measured value for C_L,α of SG6042 (thesis uses SG6040)
- No explicit Oswald efficiency e (uses C_D,0 + C_D,α·α² drag form)
- No measured C_M,γ for Beaupoil flap geometry (value is for thesis prototype geometry)
- No blade mass breakdown
- No Reynolds number calculation for thesis rotor
