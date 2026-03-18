# Felix Weyel (2025) -- Thesis Summary

**"Modeling and Closed Loop Control of a Cyclic Pitch Actuated Rotary Airborne Wind Energy System"**
Bachelor's Thesis, University of Freiburg, 2025.
Measurement data: Christof Beaupoil / someAWE Labs.

---

## What the Thesis Does

Derives and simulates a nonlinear state-space model of a 3-blade RAWES, identifies a flap
dynamics model from measurement data (N4SID), and designs a feed-forward + PID controller
that tracks a cyclic blade pitch reference. The controller is validated in simulation with
modelling errors and sensor noise.

---

## RAWES Model (Chapter 4)

### State Vector (20 elements)

```
x = [p(3), v(3), R(9), omega_R(3), theta, omega_theta]
```

- p        : rotor CoM position in global frame      [m]
- v        : CoM velocity in global frame             [m/s]
- R        : rotation matrix (rotor -> global)        [3x3 flat]
- omega_R  : angular velocity in rotor frame          [rad/s]
- theta    : rotor azimuth angle                      [rad]
- omega_theta : rotor spin speed (constant input)     [rad/s]

### Input

```
u = [beta_1, beta_2, beta_3]    blade pitch angles  [rad]
```

### Reference Frames

- Global  [ex, ey, ez]     -- world coords, ez points up
- Rotor   [e'x, e'y, e'z]  -- rotor disc plane, e'z = spin axis
- Blade i [e''x, e''y, e''z] -- e''y along blade span

### Key Equations

Apparent wind in blade frame (eq 39):
```
v_ap = v_blade - R_theta_i * R^T * v_wind
```
where v_blade = [0, r * omega_theta, 0] (blade tip velocity).

Inflow angle and angle of attack (eq 15-16):
```
phi   = atan2(v_ap_z, v_ap_y)
alpha = beta + phi
```

Aerodynamic coefficients (linearised):
```
CL = CL0 + CL_alpha * alpha
CD = CD0 + CD_alpha * alpha^2
```

Dynamics (eq 53):
```
p_dot       = v
v_dot       = R * F_rotor / m
R_dot       = R * (kappa/2*(I - R^T*R) + [omega_R]_x)   -- Baumgarte stabilised
omega_R_dot = Jr^{-1} * (M_total - omega_R x Jr*omega_R)
theta_dot   = omega_theta
```

Baumgarte correction keeps R orthonormal during numerical integration (eq 57).

### Parameters (Table 1)

| Symbol       | Value    | Unit    |
|--------------|----------|---------|
| rho          | 1.22     | kg/m^3  |
| g            | 9.81     | m/s^2   |
| Blade span   | 1.5      | m       |
| Blade area S | 0.2      | m^2     |
| CL0          | 0.11     | --      |
| CL_alpha     | 0.87     | 1/rad   |
| CD0          | 0.007    | --      |
| CD_alpha     | 0.00013  | 1/rad^2 |
| Rotor mass m | 40       | kg      |
| Jr           | diag(20,20,40) | kg*m^2 |
| Damp D       | 50       | N*m*s   |
| CoM offset d | -1       | m       |
| omega_theta  | 15*pi    | rad/s   |
| kappa_R      | 0.2      | --      |

Integration: RK4, dt = 0.001 s.

---

## Flap Dynamics (Chapter 5)

Each blade has a trailing-edge flap. Flap deflection gamma generates an aerodynamic moment
that twists the flexible blade, changing pitch angle beta.

### State Space (per blade)

```
x_f = [beta, omega_beta]^T
u_f = gamma                (flap deflection angle)

x_f_dot = A * x_f + B * u_f

A = | 0                              1     |
    | q_inf*S*l*(CM_beta+CM_gamma)/Jb   D_omega |

B = | 0                         |
    | q_inf*S*l*CM_gamma / Jb   |

C = [1, 0]    (output is beta)
```

### Parameters (Table 2)

| Symbol    | Value  | Unit    |
|-----------|--------|---------|
| Chord l   | 0.15   | m       |
| CM_beta   | 0.002  | --      |
| CM_gamma  | -0.5   | --      |
| Jb        | 1.20   | kg*m^2  |
| D_omega   | -13.5  | N*m*s   |

In practice the thesis identifies A, B from measurement data using N4SID (subspace
system identification). Physical A, B above are used here as a stand-in.

Integration: RK4, dt = 0.013 s.

---

## Controller (Chapter 5.4)

### Reference Signal

For a rotor tilting in direction Psi with amplitude U0:
```
beta_ref(t) = U0 * sin(omega_theta * t + Psi)
```

### Feed-Forward

Compute the transfer function G(jw) = C(jwI - A)^{-1} B + D at w = omega_theta.
Extract gain M = |G(jw)| and phase Phi = arg(G(jw)).

Feed-forward command (eq 68):
```
gamma_ff(t) = (U0 / M) * sin(omega_theta * t - Phi + Psi)
```

This pre-inverts the flap dynamics so the output tracks the reference without lag.

### Closed-Loop (Feed-Forward + PID)

```
gamma_CL = gamma_ff + gamma_PID
gamma_PID from: error = beta_ref - beta_actual
```

PID gains: Kp = 2, Ki = 3, Kd = 0.2

### Controller Performance

| Condition                  | RMSE   |
|----------------------------|--------|
| Open-loop (FF only)        | 9.693 deg |
| Closed-loop (FF + PID)     | 2.610 deg |
| +10% model error, open     | ~12 deg   |
| +10% model error, closed   | ~3 deg    |

Closed-loop reduces tracking error by ~73% and compensates model uncertainty.

---

## Python Simulation (this directory)

Three modules reproduce the key thesis figures:

- **rawes_model.py** -- RAWES ODE (Chapter 4), RK4 integrator, blade data collector
- **flap_controller.py** -- Flap state space, transfer function, PID, simulate_flap()
- **simulate.py** -- Main script: generates fig10-fig15

Figures produced:

| File                     | Content                                         |
|--------------------------|-------------------------------------------------|
| fig10_blade_forces.png   | Per-blade forces, AoA, apparent wind over one cycle |
| fig11_trajectories.png   | 3D rotor trajectory: beta=0 and beta=cyclic     |
| fig14_controller.png     | Controller tracking: Psi switch, amp switch, noise |
| fig15_model_error.png    | FF-only vs closed-loop under 10% model error    |

Run with:
```
venv/Scripts/python simulate.py
```

---

## Key Findings

1. The cyclic pitch input beta_i(t) = U0*sin(omega_theta*t + Psi + (i-1)*2pi/3) tilts
   the rotor disc in direction Psi, enabling steering.

2. The feed-forward controller inverts the flap transfer function to remove phase lag.
   Without it, PID alone cannot keep up at omega_theta = 15*pi rad/s.

3. The Baumgarte correction (kappa=0.2) is essential for long simulations -- without it
   the rotation matrix R drifts from SO(3) and the simulation diverges.

4. The identified (N4SID) A, B matrices from real measurement data would differ from
   the physical matrices used here, but the controller structure and performance are the same.

---

## Gaps vs Actual Hardware

The thesis uses a 3-blade, 1.5m prototype. Actual RAWES hardware has:

| Item              | Thesis  | Hardware    |
|-------------------|---------|-------------|
| Blade count       | 3       | 4           |
| Blade span        | 1.5 m   | 2.0 m       |
| Phase offset      | 120 deg | 90 deg      |
| Rotor mass        | 40 kg   | 5 kg        |
| Flap actuation    | Direct  | Swashplate  |
| Anti-rotation     | None    | GB4008 motor|
