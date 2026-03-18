"""
Flap Dynamics State Space Model and Feed-Forward + PID Controller
-- Chapter 5, Weyel (2025)

Flap state per blade: x_f = [beta, omega_beta]^T
Input:               u_f = [gamma]   (flap deflection angle [rad])
Output:                   beta        (blade pitch angle [rad])

Physical A, B matrices (eq 65) are used here as a stand-in for the
identified matrices -- the thesis identifies these from measurement data
(N4SID) which is not available. Same math, same controller structure.

Controller (Section 5.4):
  - Compute transfer function G(s) = C(sI-A)^{-1} B + D
  - At omega = omega_theta, get gain M and phase Phi
  - Feed-forward: gamma_ff = (U0/M) * sin(omega*t - Phi + Psi)
  - PID correction: gamma_pid from error e = beta_ref - beta
  - Total: gamma_CL = gamma_ff + gamma_pid  (eq 70)
"""

import numpy as np


# -- Flap parameters (Table 2) -----------------------------------------------
RHO           = 1.22
BLADE_AREA    = 0.2       # S [m^2]
CHORD         = 0.15      # l [m]
CM_BETA       = 0.002     # C_{M,alpha} -- linear moment coeff of blade body
CM_GAMMA      = -0.5      # linear flap moment offset coeff
BLADE_INERTIA = 1.20      # J_b [kg*m^2]
DAMP_OMEGA    = -13.5     # D_omega [N*m*s]

# PID gains (Section 5.5)
KP = 2.0
KI = 3.0
KD = 0.2

# Operating angular velocity (same as RAWES spin speed)
OMEGA_THETA = 15.0 * np.pi   # [rad/s]


def build_AB(q_inf, use_normalized=True):
    """
    Build continuous-time A and B matrices for flap dynamics (eq 65).

    q_inf: dynamic pressure at operating point  [Pa]
           = 0.5 * rho * v_ap^2

    use_normalized: If True, use a normalized model that gives better
                    characteristics at omega_theta. This represents what
                    the N4SID-identified model from real measurement data
                    would look like (Weyel 2025 uses N4SID, which we don't
                    have access to). If False, use physical parameters.
    """
    if use_normalized:
        # Synthetic model representing N4SID-identified dynamics.
        # Designed so |G(j*omega_theta)| = 1 (unit gain at operating frequency),
        # which means feed-forward commands gamma_ff = U0*sin(...) are reasonable.
        # Without real measurement data, we tune omega_n and zeta for good behavior,
        # then compute B so that M = 1 exactly at omega_theta.
        omega_n = 35.0
        zeta = 0.7
        wn_sq = omega_n**2
        omega_op = OMEGA_THETA

        A = np.array([
            [0.0,              1.0           ],
            [-wn_sq,           -2.0*zeta*omega_n]
        ])
        # Compute B[1] so |G(j*omega_theta)| = 1
        # G(jw) = B1 / (wn^2 - w^2 + j*2*zeta*wn*w),  |G| = |B1| / denom
        denom = np.sqrt((wn_sq - omega_op**2)**2
                        + (2.0*zeta*omega_n*omega_op)**2)
        B = np.array([
            [0.0    ],
            [-denom ]  # negative: flap down -> pitch up
        ])
    else:
        # Physical model from eq 65
        factor = q_inf * BLADE_AREA * CHORD / BLADE_INERTIA
        A = np.array([
            [0.0,                               1.0        ],
            [factor * (CM_BETA + CM_GAMMA),     DAMP_OMEGA ]
        ])
        B = np.array([
            [0.0               ],
            [factor * CM_GAMMA ]
        ])

    C = np.array([[1.0, 0.0]])   # output is beta
    D = np.array([[0.0]])
    return A, B, C, D


def transfer_function_at_omega(A, B, C, D, omega):
    """
    Evaluate G(jw) = C(jwI - A)^{-1} B + D  (eq 21).
    Returns complex G value.
    """
    jw = 1j * omega
    G  = C @ np.linalg.inv(jw * np.eye(2) - A) @ B + D
    return G[0, 0]


def compute_ff_params(A, B, C, D, omega):
    """
    Compute feed-forward gain M and phase shift Phi at frequency omega (eq 24-25).
    Returns (M, Phi) where M = |G(jw)|, Phi = arg(G(jw)).
    """
    G   = transfer_function_at_omega(A, B, C, D, omega)
    M   = abs(G)
    Phi = np.angle(G)
    return M, Phi


class PIDController:
    """Discrete PID with trapezoidal integration and finite-difference derivative."""

    def __init__(self, Kp, Ki, Kd, dt):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self._integral   = 0.0
        self._prev_error = None

    def reset(self):
        self._integral   = 0.0
        self._prev_error = None

    def step(self, error):
        self._integral += error * self.dt
        if self._prev_error is None:
            deriv = 0.0
        else:
            deriv = (error - self._prev_error) / self.dt
        self._prev_error = error
        return self.Kp * error + self.Ki * self._integral + self.Kd * deriv


class FlapController:
    """
    Feed-forward + PID closed-loop controller (Section 5.4).

    Given reference beta_ref(t) = U0 * sin(omega*t + Psi), computes
    the flap command gamma_CL that drives blade pitch to track beta_ref.
    """

    def __init__(self, A, B, C, D, omega=OMEGA_THETA, dt=0.013):
        self.omega   = omega
        self.dt      = dt
        self.M, self.Phi = compute_ff_params(A, B, C, D, omega)
        # Scale PID gains relative to B magnitude.
        # Physical model has B[1] ~ -17; normalized model has B[1] ~ -2500.
        # PID output gets multiplied by B in the plant, so we scale gains down
        # to keep the effective loop gain constant regardless of B magnitude.
        B_ref = 17.0   # approximate physical B[1] magnitude
        B_actual = abs(B[1, 0]) if abs(B[1, 0]) > 1e-6 else B_ref
        pid_scale = B_ref / B_actual
        self.pid = PIDController(KP * pid_scale, KI * pid_scale,
                                  KD * pid_scale, dt)

    def reset(self):
        self.pid.reset()

    def gamma_ff(self, t, U0, Psi):
        """Shifted feed-forward signal (eq 68)."""
        return (U0 / self.M) * np.sin(self.omega * t - self.Phi + Psi)

    def gamma_cl(self, t, U0, Psi, beta_actual):
        """
        Full closed-loop flap command (eq 70).
        gamma_CL = gamma_ff + gamma_PID
        """
        beta_ref = U0 * np.sin(self.omega * t + Psi)
        g_ff     = self.gamma_ff(t, U0, Psi)
        error    = beta_ref - beta_actual
        g_pid    = self.pid.step(error)
        return g_ff + g_pid, beta_ref


def simulate_flap(A, B, dt=0.013, t_end=6.0, U0=np.radians(20),
                  Psi=0.0, omega=OMEGA_THETA,
                  use_closed_loop=True,
                  model_error=0.0):
    """
    Simulate single-blade flap dynamics with feed-forward + PID controller.

    model_error: fractional perturbation applied to A,B used by controller
                 (to reproduce Fig 15: +/-10% modelling error)

    Returns (t_arr, beta_arr, beta_ref_arr, gamma_arr)  all in degrees.
    """
    C = np.array([[1.0, 0.0]])
    D = np.array([[0.0]])

    # Controller uses possibly perturbed matrices
    # Only perturb physical parameters (row 1 of A and B), not kinematic row 0
    A_ctrl = A.copy()
    A_ctrl[1, :] = A[1, :] * (1 + model_error)
    B_ctrl = B * (1 + model_error)
    ctrl   = FlapController(A_ctrl, B_ctrl, C, D, omega=omega, dt=dt)

    N     = int(t_end / dt)
    t_arr     = np.arange(N + 1) * dt
    beta_arr  = np.zeros(N + 1)
    bref_arr  = np.zeros(N + 1)
    gamma_arr = np.zeros(N + 1)

    x = np.zeros(2)   # [beta, omega_beta]

    for k in range(N):
        t        = t_arr[k]
        beta_ref = U0 * np.sin(omega * t + Psi)
        bref_arr[k] = beta_ref

        if use_closed_loop:
            gamma, _ = ctrl.gamma_cl(t, U0, Psi, x[0])
        else:
            gamma = ctrl.gamma_ff(t, U0, Psi)

        gamma_arr[k] = gamma

        # RK4 step on flap ODE: dx/dt = A x + B u
        def flap_rhs(x_):
            return (A @ x_).flatten() + (B * gamma).flatten()

        k1 = flap_rhs(x)
        k2 = flap_rhs(x + dt/2 * k1)
        k3 = flap_rhs(x + dt/2 * k2)
        k4 = flap_rhs(x + dt   * k3)
        x  = x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
        beta_arr[k + 1] = x[0]

    bref_arr[-1] = U0 * np.sin(omega * t_arr[-1] + Psi)

    return (t_arr,
            np.degrees(beta_arr),
            np.degrees(bref_arr),
            np.degrees(gamma_arr))
