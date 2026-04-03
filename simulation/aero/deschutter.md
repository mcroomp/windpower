# De Schutter 2018 — Aerodynamic Model: Implementation & Validation

Reference: De Schutter J., Leuthold R., Diehl M. (2018).
"Optimal Control of a Rigid-Wing Rotary Kite System for Airborne Wind Energy."
IFAC Proceedings, Vol. 51, No. 13, pp. 523-528.
https://publications.syscop.de/DeSchutter2018.pdf

---

## Overview

`aero_deschutter.py` implements the aerodynamic model of De Schutter et al. (2018)
as a NumPy-vectorized per-blade BEM (Blade Element Momentum) strip-theory model.
The rotor geometry and airfoil coefficients are defined in
`rotor_definitions/de_schutter_2018.yaml`.

The validation suite in `tests/unit/test_deschutter_equations.py` checks each
paper equation against the implementation and the YAML parameter values.

---

## Paper Equations: What Is Implemented and How

### Table I — Rotor Geometry

| Paper symbol | Value   | Where set              | Test                   |
|-------------|---------|------------------------|------------------------|
| N           | 3       | YAML `n_blades`        | `TestTableI`           |
| L_w (span)  | 1.50 m  | YAML `root_cutout_m`   | `TestTableI`           |
| L_b (arm)   | 1.60 m  | derived R - L_w        | `TestTableI`           |
| R (tip)     | 3.10 m  | YAML `radius_m`        | `TestTableI`           |
| c (chord)   | 0.125 m | YAML `chord_m`         | `TestTableI`           |
| A (AR)      | 12.0    | derived span/chord     | `TestTableI`           |
| r_cp        | 2.60 m  | R_ROOT + 2/3 * span    | `TestTableI`           |
| C_{D,0}     | 0.01    | YAML `CD0`             | `TestTableI`           |
| O_e         | 0.8     | YAML `oswald_eff`      | `TestTableI`           |
| alpha_stall | 15 deg  | YAML `aoa_limit`       | `TestTableI`           |

The centre of pressure r_cp = L_b + (2/3)*L_w = 2.60 m is stated in
Section II-D: "two-thirds of wing span from root."

---

### Eq. 25 — Lift and Drag Coefficients (Thin Airfoil + Prandtl Correction)

Paper:

    C_{L,k} = (2*pi / (1 + 2/A)) * alpha_k

    C_{D,k} = C_{D,0} + C_{L,k}^2 / (pi * A * O_e)

Implementation (aero_deschutter.py lines 255-256):

    CL = self.CL0 + self.CL_ALPHA * alpha
    CD = self.CD0 + CL**2 / (pi * self.AR * self.OSWALD)

Key values:

    CL_alpha = 2*pi / (1 + 2/12) = 12*pi/7 = 5.385 /rad
    CL0 = 0.0   (symmetric wing, no camber)
    CD0 = 0.01  (Table I)
    O_e = 0.8   (Table I)

At alpha = 5 deg:

    CL = 5.385 * 0.08727 = 0.470
    CD = 0.01 + 0.470^2 / (pi * 12 * 0.8) = 0.01 + 0.00583 = 0.01583
    L/D = 29.7

Tests: `TestEq25LiftDragCoefficients` (7 tests)

---

### Eq. 26 — Angle of Attack

Paper:

    alpha_k = arctan(u_{a,k} · e_{3,k} / u_{a,k} · e_{2,k})
            = -u_{a,normal} / u_{a,chord}  (small angle, same sign convention)

where:
- e_{2,k} = chord direction (in-plane, tangential)
- e_{3,k} = normal direction (out-of-plane)
- u_{a,k} = apparent wind at blade strip k

Implementation (lines 249-252):

    safe_chord = where(|ua_chord| >= 1e-6, ua_chord, 1.0)
    alpha = where(valid,
                  clip(-ua_normal / safe_chord, -AOA_LIMIT, AOA_LIMIT),
                  0.0)

Stall clamp |alpha| <= 15 deg (Eq. 28).  Strips with |ua| < 0.5 m/s or
|ua_chord| < 1e-6 are marked invalid and contribute zero force.

Design operating point (V_wind=10 m/s, omega=47.12 rad/s, r_cp=2.60 m):

    v_tangential = 47.12 * 2.60 = 122.5 m/s
    inflow_angle = atan(10 / 122.5) = 4.7 deg < 15 deg  (within Eq. 28 validity)

Tests: `TestEq26AngleOfAttack` (2 tests)

---

### Eq. 27, 28 — Side-Slip Angle beta (Validity Indicator Only)

Paper:

    beta_k = arcsin( (u_{a,k} · e_{1,k}) / |u_{a,k}| )

where e_{1,k} = span direction.  Eq. 28 requires |beta_k| <= 15 deg for the
thin-airfoil linearisation to remain valid.

**beta does NOT appear in the force or moment formulas.**  The lift direction
is given by the Kutta-Joukowski cross product (u_a x e_span), which already
handles spanwise flow geometry correctly without an explicit beta correction.
beta is a post-hoc validity check, not a force modifier.

Implementation (lines 232-238):

    ua_span  = einsum('ijk,ik->ij', ua, e_span_world)
    beta_rad = arcsin(clip(ua_span / max(ua_norm, 1e-9), -1, 1))

Diagnostic stored in `model.last_sideslip_mean_deg` after each call.

At design conditions (V_wind=10 m/s, omega=47.12 rad/s):

    beta_mean < 5 deg  (spanwise wind component is small vs tangential speed)

Tests: `TestEq27SideSlip` (4 tests)

---

### Eq. 29, 31 — Structural Parasitic Drag C_{D,T}

Paper: The connecting cable (tether arm) from hub to blade generates
cylindrical drag, added to the blade's drag coefficient:

    C_{D,T} = C_{D,cyl} * d_{cable} * L_{cable} / S_w

Table I gives:
- d_cable = 1.5 mm (cable diameter)
- L_cable ~= r_cp = 2.60 m  (arm length from hub to blade centre of pressure)
- S_w = chord * span = 0.125 * 1.50 = 0.1875 m^2

Calculation:

    C_{D,T} = 1.0 * 0.0015 * 2.60 / 0.1875 = 0.0208 =~ 0.021

Note: The paper does not state C_{D,T} numerically — it must be derived
from the cable geometry parameters in Table I.

Implementation (lines 269-276):

    CD_total = CD + self.CD_T      # self.CD_T = 0.021
    F_strip = q_dyn * (CL * e_lift + CD_total * ua_unit)

YAML (`rotor_definitions/de_schutter_2018.yaml`):

    CD_structural: 0.021   # under airfoil:

Effect at design conditions:

    delta_CD = 0.021  (vs CD0 = 0.01, so C_{D,T} roughly doubles parasitic drag)
    Increases H-force (in-plane drag) by ~40-80 N
    Reduces net spin torque Q_spin (more drag opposing rotation)

For the Beaupoil 2026 rotor (`beaupoil_2026.yaml`), `CD_structural = 0.0` since
blade roots connect directly to the hub via spar with no thin cables.

Tests: `TestEq29StructuralDrag` (6 tests)

---

### Eq. 30, 31 — Thrust T and H-Force H

Paper:

    T = sum_k [ q_k * (C_L * cos(inflow) - C_D * sin(inflow)) ]
    H = sum_k [ q_k * (C_L * sin(inflow) + C_D * cos(inflow)) ]

These are the axial (disk-normal) and in-plane force components
after revolution-averaging.

Implementation: forces are accumulated per strip in world NED coordinates.
Thrust is decomposed as the component along `disk_normal`; H-force is the
in-plane remainder.

Tests: `TestEq30Eq31ForcDirections` (3 tests)

---

## What the Paper Does NOT Model (Known Gaps)

| Missing element                        | Reason / note                                      |
|----------------------------------------|----------------------------------------------------|
| Dynamic stall / stall hysteresis       | Steady-state BEM only                              |
| Blade flapping DOF                     | Rigid blade assumption (paper also rigid)          |
| Spanwise induced velocity variation    | Paper uses a single-point induction at r_cp        |
| Wake skew / Coleman correction         | DeSchutterAero omits this (SkewedWakeBEM includes) |
| Dynamic inflow transient               | BEM bootstrap converges instantly (3 iterations)   |
| Tether / ground-station dynamics       | Separate tether.py model                           |
| Kaman flap lag                         | Not in De Schutter 2018; see Beaupoil 2026         |

The production aerodynamic model (`SkewedWakeBEM` in `aero_skewed_wake.py`)
adds Coleman skewed-wake correction and Prandtl tip/root loss, and is the
default model used in simulation.  `DeSchutterAero` is the reference
implementation closest to the paper and is used for equation-level validation.

---

## Numerical Fix: Induced Velocity Bootstrap Floor

The original `_induced_velocity()` used `T_abs = max(abs(T_guess), 0.01)` to
prevent near-zero issues.  This caused a phantom 0.01 N seed that — through
the nonlinear BEM iteration — amplified to ~4 m/s induced velocity and ~200 N
thrust at zero collective + purely in-plane wind.

The correct formula requires no floor:

    disc = v_axial^2 + 2*T_abs / (rho * A_disk)
    v_i  = max(0, (-v_axial + sqrt(disc)) / 2)

When T=0: disc = v_axial^2, v_i = (-|v_axial| + |v_axial|)/2 = 0. Correct.
When T>0: formula gives the standard actuator-disk induced velocity. Correct.

The floor `max(..., 0.01)` was mathematically unnecessary and physically
incorrect; it has been removed.

---

## File Map

| File                                           | Purpose                              |
|------------------------------------------------|--------------------------------------|
| `aero/aero_deschutter.py`                      | DeSchutterAero implementation        |
| `rotor_definitions/de_schutter_2018.yaml`      | Geometry + aerodynamics from paper   |
| `tests/unit/test_deschutter_equations.py`      | Equation-level validation (32 tests) |
| `aero/tests/test_deschutter_validation.py`     | Geometry + behavior validation       |
| `theory/pumping_cycle.md` (repo root docs)     | Control theory overview              |
