"""
RAWES State Space Model -- Chapter 4, Weyel (2025)

Implements the full nonlinear ODE for a 3-blade RAWES.
State vector: [p(3), v(3), R(9), omega_R(3), theta(1), omega_theta(1)]  = 20 elements
Input:        beta(3)  -- blade pitch angles [rad]

Reference frames:
  Global:  [ex, ey, ez]   -- world coords, ez points up
  Rotor:   [e'x, e'y, e'z]  -- rotor disc in e'x*e'y plane, e'z = spin axis
  Blade i: [e''x, e''y, e''z] -- blade lies in e''x*e''y plane
"""

import numpy as np

# -- Parameters (Table 1) ----------------------------------------------------
RHO        = 1.22                      # Air density            [kg/m^3]
G_GRAV     = 9.81                      # Gravity                [m/s^2]
BLADE_SPAN = 1.5                       # Blade span s           [m]
BLADE_AREA = 0.2                       # Blade area S           [m^2]
CL0        = 0.11                      # Zero-lift coeff
CL_ALPHA   = 0.87                      # Lift slope             [1/rad]
CD0        = 0.007                     # Zero-drag coeff
CD_ALPHA   = 0.00013                   # Drag curvature         [1/rad^2]
ROTOR_MASS = 40.0                      # Rotor mass             [kg]
Jr         = np.diag([20.0, 20.0, 40.0])  # Inertia tensor     [kg*m^2]
Jr_inv     = np.linalg.inv(Jr)
DAMP_R     = 50.0                      # Rotational damping     [N*m*s]
COM_OFFSET = -1.0                      # CoM below disc         [m]
OMEGA_THETA = 15.0 * np.pi             # Rotor spin speed       [rad/s]
KAPPA_R    = 0.2                       # Baumgarte factor

R_CALC     = (2.0 / 3.0) * BLADE_SPAN  # Force calc point on blade [m]
N_BLADES   = 3
PHASE      = 2.0 * np.pi / 3.0         # 120 deg between blades


def skew(v):
    """Skew-symmetric matrix [v]_x such that [v]_x @ u = v x u  (eq 50)."""
    return np.array([
        [ 0.0,  -v[2],  v[1]],
        [ v[2],  0.0,  -v[0]],
        [-v[1],  v[0],  0.0 ]
    ])


def R_theta(theta, i):
    """
    Blade rotation matrix for blade i (1-indexed), rotating around e'_z (eq 35).
    Transforms rotor-frame coords into blade-frame coords for blade i.
    """
    angle = theta + (i - 1) * PHASE
    c, s = np.cos(angle), np.sin(angle)
    return np.array([
        [ c, -s, 0.0],
        [ s,  c, 0.0],
        [0.0, 0.0, 1.0]
    ])


def R_alpha_mat(alpha):
    """Rotation by alpha around e''_x axis (eq 40), aligning lift/drag with frame."""
    c, s = np.cos(alpha), np.sin(alpha)
    return np.array([
        [1.0,  0.0,  0.0],
        [0.0,   c,  -s  ],
        [0.0,   s,   c  ]
    ])


def aero_force(v_ap, beta_i):
    """
    Compute aerodynamic force on blade i in blade frame (eq 38-41).

    v_ap   : apparent wind vector in blade frame  [m/s]
    beta_i : blade pitch angle                    [rad]
    returns F_i in blade frame                    [N]
    """
    # Inflow angle from z and y components of apparent wind (eq 15)
    phi   = np.arctan2(v_ap[2], v_ap[1])
    alpha = beta_i + phi                        # angle of attack (eq 16)

    # Dynamic pressure using apparent wind magnitude
    q_inf = 0.5 * RHO * np.dot(v_ap, v_ap)

    # Linearised aero coefficients (Section 2.3)
    CL = CL0 + CL_ALPHA * alpha
    CD = CD0 + CD_ALPHA * alpha**2

    L = q_inf * CL * BLADE_AREA
    D = q_inf * CD * BLADE_AREA

    # Force in blade frame (eq 41): lift perp to apparent wind, drag parallel
    Ra  = R_alpha_mat(alpha)
    e_y = np.array([0.0, 1.0, 0.0])
    e_z = np.array([0.0, 0.0, 1.0])
    F_i = Ra @ e_z * L + Ra @ e_y * D
    return F_i, alpha, phi


def rawes_rhs(t, x, beta, v_wind):
    """
    Right-hand side of RAWES ODE (eq 53).

    x      : state vector [p(3), v(3), R_flat(9), omega_R(3), theta, omega_theta]
    beta   : blade pitch angles array shape (3,)  [rad]
    v_wind : global wind velocity                  [m/s]
    """
    p        = x[0:3]
    v        = x[3:6]
    R        = x[6:15].reshape(3, 3)
    omega_R  = x[15:18]
    theta    = x[18]
    om_th    = x[19]

    e_z_global = np.array([0.0, 0.0, 1.0])
    e_z_rotor  = np.array([0.0, 0.0, 1.0])

    # Gravity in rotor frame (eq 42)
    F_G = -R.T @ e_z_global * ROTOR_MASS * G_GRAV

    # Aerodynamic forces and moments from each blade
    F_rotor = F_G.copy()
    M_aero  = np.zeros(3)

    for i in range(1, N_BLADES + 1):
        Rth = R_theta(theta, i)

        # Blade velocity in blade frame (eq 38): v_b = e''_y * r * omega_theta
        v_b = np.array([0.0, R_CALC * om_th, 0.0])

        # Wind in blade frame (eq 37): v''_w = R_theta_i * R^T * v_w
        v_w_blade = Rth @ (R.T @ v_wind)

        # Apparent wind in blade frame (eq 39)
        v_ap = v_b - v_w_blade

        # Aerodynamic force in blade frame
        F_i, _, _ = aero_force(v_ap, beta[i - 1])

        # Transform force to rotor frame and accumulate (eq 43)
        F_rotor += Rth.T @ F_i

        # Moment arm in rotor frame
        r_arm = Rth.T @ np.array([0.0, R_CALC, 0.0])
        M_aero += np.cross(r_arm, Rth.T @ F_i)

    # Tether stabilising moment (eq 44)
    # M_tether = (-e'_z * d) x F_G   with d = -1 -> [0,0,1] x F_G
    r_tether = -e_z_rotor * COM_OFFSET     # [0, 0, 1]
    M_tether = np.cross(r_tether, F_G)

    # Total moment (eq 45)
    M_rotor = M_aero + M_tether - DAMP_R * omega_R

    # State derivatives (eq 53) -------------------------------------------------

    # Position derivative
    p_dot = v

    # Velocity: acceleration = R * F_rotor / m  (eq 48, global frame)
    a = R @ F_rotor / ROTOR_MASS

    # Rotation matrix with Baumgarte correction (eq 57).
    # Original: kappa/2 * (inv(R^T R) - I). For small deviations this equals
    # kappa/2 * (I - R^T R) to first order, but the latter is numerically stable
    # when R drifts far from orthogonality (avoids singular-matrix error).
    RtR       = R.T @ R
    baumgarte = (KAPPA_R / 2.0) * (np.eye(3) - RtR)
    R_dot     = R @ (baumgarte + skew(omega_R))

    # Angular acceleration (eq 52): Euler's rotation equations
    omega_R_dot = Jr_inv @ (M_rotor - np.cross(omega_R, Jr @ omega_R))

    x_dot = np.zeros(20)
    x_dot[0:3]   = p_dot
    x_dot[3:6]   = a
    x_dot[6:15]  = R_dot.flatten()
    x_dot[15:18] = omega_R_dot
    x_dot[18]    = om_th   # d(theta)/dt = omega_theta
    x_dot[19]    = 0.0     # omega_theta constant
    return x_dot


def rk4_step(rhs, t, x, dt, beta, v_wind):
    """Single RK4 step (eq 55-56, Butcher tableau Fig 9)."""
    k1 = rhs(t,        x,              beta, v_wind)
    k2 = rhs(t + dt/2, x + dt/2 * k1, beta, v_wind)
    k3 = rhs(t + dt/2, x + dt/2 * k2, beta, v_wind)
    k4 = rhs(t + dt,   x + dt   * k3, beta, v_wind)
    return x + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)


def simulate(beta_func, t_end=5.0, dt=0.001, v_wind=None):
    """
    Integrate RAWES model from x0 = [0,..., R=I, omega_theta=15pi].

    beta_func(t, theta, omega_theta) -> np.array shape (3,)  [rad]
    Returns (t_arr, x_arr).
    """
    if v_wind is None:
        v_wind = np.array([0.0, 10.0, 0.0])

    x0 = np.zeros(20)
    x0[6:15] = np.eye(3).flatten()   # R = identity
    x0[19]   = OMEGA_THETA            # omega_theta = 15*pi

    N     = int(t_end / dt)
    t_arr = np.arange(N + 1) * dt
    x_arr = np.zeros((N + 1, 20))
    x_arr[0] = x0

    for k in range(N):
        t     = t_arr[k]
        xk    = x_arr[k]
        theta = xk[18]
        om_th = xk[19]
        beta  = beta_func(t, theta, om_th)
        x_arr[k + 1] = rk4_step(rawes_rhs, t, xk, dt, beta, v_wind)

    return t_arr, x_arr


def collect_blade_data(t_arr, x_arr, beta_func, v_wind=None):
    """
    Re-extract per-blade quantities (v_ap, alpha, phi, forces) for plotting Fig 10.
    """
    if v_wind is None:
        v_wind = np.array([0.0, 10.0, 0.0])

    N = len(t_arr)
    v_ap_mag  = np.zeros((N, N_BLADES))
    alpha_arr = np.zeros((N, N_BLADES))
    beta_arr  = np.zeros((N, N_BLADES))
    F_ex      = np.zeros((N, N_BLADES))
    F_ey      = np.zeros((N, N_BLADES))
    F_ez      = np.zeros((N, N_BLADES))

    for k in range(N):
        xk    = x_arr[k]
        R     = xk[6:15].reshape(3, 3)
        theta = xk[18]
        om_th = xk[19]
        beta  = beta_func(t_arr[k], theta, om_th)

        for i in range(1, N_BLADES + 1):
            Rth  = R_theta(theta, i)
            v_b  = np.array([0.0, R_CALC * om_th, 0.0])
            v_ap = v_b - Rth @ (R.T @ v_wind)
            F_i, alpha, _ = aero_force(v_ap, beta[i - 1])
            F_rot = Rth.T @ F_i

            v_ap_mag[k, i-1]  = np.linalg.norm(v_ap)
            alpha_arr[k, i-1] = np.degrees(alpha)
            beta_arr[k, i-1]  = np.degrees(beta[i - 1])
            F_ex[k, i-1]      = F_rot[0]
            F_ey[k, i-1]      = F_rot[1]
            F_ez[k, i-1]      = F_rot[2]

    return v_ap_mag, alpha_arr, beta_arr, F_ex, F_ey, F_ez
