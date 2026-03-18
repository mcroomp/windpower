"""
Main simulation script -- reproduces key figures from Weyel (2025).

Run:
    python simulate.py

Produces:
    fig10_blade_forces.png    -- per-blade forces during one pitching cycle (Fig 10)
    fig11_trajectories.png    -- 3D RAWES trajectories for 3 pitch inputs (Fig 11)
    fig14_controller.png      -- flap controller tracking performance (Fig 14)
    fig15_model_error.png     -- controller with/without CL under model error (Fig 15)
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import os

import rawes_model as rm
import flap_controller as fc

CSV_DIR = "csv_simulated"


# ============================================================================
# Chapter 4 -- RAWES trajectory simulation
# ============================================================================

def beta_zero(t, theta, om_th):
    """beta=0: no pitching (eq 58)."""
    return np.zeros(3)


def beta_cyclic_psi0(t, theta, om_th):
    """beta cyclic, Psi=0, amplitude 20 deg (eq 59)."""
    U0 = np.radians(20.0)
    return np.array([
        U0 * np.sin(om_th * t + (i - 1) * rm.PHASE)
        for i in range(1, 4)
    ])


def beta_cyclic_psi_half_pi(t, theta, om_th):
    """beta cyclic, Psi=pi/2, amplitude 20 deg (eq 60)."""
    U0  = np.radians(20.0)
    Psi = np.pi / 2.0
    return np.array([
        U0 * np.sin(om_th * t + (i - 1) * rm.PHASE + Psi)
        for i in range(1, 4)
    ])


def run_fig10():
    """
    Figure 10: forces, apparent wind, AoA, blade pitch during one pitching cycle.
    Uses beta_cyclic_psi0 with 10 m/s wind over one full revolution.
    """
    print("Simulating Fig 10 (one pitching cycle)...")
    T_cycle = 2.0 * np.pi / rm.OMEGA_THETA          # one revolution ~0.133 s
    t_arr, x_arr = rm.simulate(beta_cyclic_psi0, t_end=T_cycle, dt=0.0001,
                                v_wind=np.array([0.0, 10.0, 0.0]))

    v_wind = np.array([0.0, 10.0, 0.0])
    v_ap, alpha, beta, Fex, Fey, Fez = rm.collect_blade_data(
        t_arr, x_arr, beta_cyclic_psi0, v_wind)

    colours = ['blue', 'green', 'red']
    labels  = ['Blade 1', 'Blade 2', 'Blade 3']
    lstyles = ['-', '--', '-.']

    fig, axes = plt.subplots(3, 2, figsize=(12, 10))
    fig.suptitle("Fig 10 -- Forces on rotor blades during one pitching cycle\n"
                 "(cyclic beta input, Psi=0, 10 m/s wind)", fontsize=12)

    for i in range(3):
        axes[0, 0].plot(t_arr, v_ap[:, i],  color=colours[i], ls=lstyles[i], label=labels[i])
        axes[1, 0].plot(t_arr, alpha[:, i], color=colours[i], ls=lstyles[i])
        axes[2, 0].plot(t_arr, beta[:, i],  color=colours[i], ls=lstyles[i])
        axes[0, 1].plot(t_arr, Fex[:, i],   color=colours[i], ls=lstyles[i])
        axes[1, 1].plot(t_arr, Fey[:, i],   color=colours[i], ls=lstyles[i])
        axes[2, 1].plot(t_arr, Fez[:, i],   color=colours[i], ls=lstyles[i])

    axes[0, 0].set_title("Apparent wind v_ap");   axes[0, 0].set_ylabel("v [m/s]")
    axes[1, 0].set_title("Angle of attack");      axes[1, 0].set_ylabel("Angle [deg]")
    axes[2, 0].set_title("Blade pitch");          axes[2, 0].set_ylabel("Angle [deg]")
    axes[0, 1].set_title("Force e'_x direction"); axes[0, 1].set_ylabel("Force [N]")
    axes[1, 1].set_title("Force e'_y direction"); axes[1, 1].set_ylabel("Force [N]")
    axes[2, 1].set_title("Force e'_z direction"); axes[2, 1].set_ylabel("Force [N]")

    for ax in axes.flat:
        ax.set_xlabel("Time [s]")
        ax.grid(True, alpha=0.3)
    axes[0, 0].legend()

    fig.tight_layout()
    fig.savefig("fig10_blade_forces.png", dpi=150)
    print("  saved fig10_blade_forces.png")
    plt.close(fig)

    # Save CSV data
    os.makedirs(CSV_DIR, exist_ok=True)
    for name, data in [("v_ap", v_ap), ("alpha", alpha), ("beta", beta),
                        ("F_ex", Fex), ("F_ey", Fey), ("F_ez", Fez)]:
        hdr = "time,blade_1,blade_2,blade_3"
        rows = np.column_stack([t_arr, data])
        np.savetxt(os.path.join(CSV_DIR, "sim_fig10_%s.csv" % name),
                   rows, delimiter=",", header=hdr, comments="", fmt="%.6f")
    print("  saved fig10 CSVs to %s/" % CSV_DIR)


def run_fig11():
    """
    Figure 11: 3D RAWES trajectories for beta=0, Psi=0, Psi=pi/2
    without wind (left) and with 10 m/s wind (right). 5 second simulation.
    """
    print("Simulating Fig 11 (3D trajectories, t=5s) -- this takes ~2 min...")
    cases = [
        (beta_zero,               "beta=0",   "blue",  ":"),
        (beta_cyclic_psi_half_pi, "Psi=pi/2", "red",   "--"),
        (beta_cyclic_psi0,        "Psi=0",    "green", "-."),
    ]

    fig = plt.figure(figsize=(16, 7))
    fig.suptitle("Fig 11 -- RAWES trajectories for different cyclic pitch inputs", fontsize=12)

    for col, (wind_vec, wind_label) in enumerate([
        (np.zeros(3),            "v_wind = 0 m/s"),
        (np.array([0, 10.0, 0]), "v_wind = 10 m/s"),
    ]):
        ax = fig.add_subplot(1, 2, col + 1, projection='3d')
        ax.set_title(wind_label)

        for beta_fn, lbl, col_, ls in cases:
            t_arr, x_arr = rm.simulate(beta_fn, t_end=5.0, dt=0.001,
                                        v_wind=wind_vec)
            ax.plot(x_arr[:, 0], x_arr[:, 1], x_arr[:, 2],
                    color=col_, ls=ls, label=lbl, linewidth=1.5)

            # Draw rotor disc outline at t = 0,1,2,3,4,5 s
            disc_r  = 0.8
            angles  = np.linspace(0, 2*np.pi, 20)
            circle  = np.array([disc_r*np.cos(angles),
                                 disc_r*np.sin(angles),
                                 np.zeros_like(angles)])
            for t_mark in [0, 1, 2, 3, 4, 5]:
                idx = int(t_mark / 0.001)
                if idx < len(x_arr):
                    R_mat  = x_arr[idx, 6:15].reshape(3, 3)
                    p_c    = x_arr[idx, 0:3]
                    disc   = p_c[:, None] + R_mat @ circle
                    ax.plot(disc[0], disc[1], disc[2],
                            color=col_, alpha=0.35, linewidth=0.8)

        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_zlabel("Z [m]")
        ax.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig("fig11_trajectories.png", dpi=150)
    print("  saved fig11_trajectories.png")
    plt.close(fig)


# ============================================================================
# Chapter 5 -- Flap controller simulation
# ============================================================================

def _build_flap_system():
    """Build A, B matrices at no-wind operating point (v_ap = r * omega_theta).

    Uses the normalized model (representing N4SID-identified matrices from real data).
    The physical model has too-low bandwidth at operating frequency.
    """
    r_blade  = (2.0 / 3.0) * 1.5          # 1.0 m
    v_ap_ref = r_blade * fc.OMEGA_THETA    # approx 47.1 m/s
    q_inf    = 0.5 * fc.RHO * v_ap_ref**2
    A, B, C, D = fc.build_AB(q_inf, use_normalized=True)
    return A, B, C, D


def _rk4_flap(A, x, gamma, dt):
    """One RK4 step for flap ODE dx/dt = A*x + B*u (B*gamma is constant over step)."""
    def f(x_):
        return A @ x_

    # gamma is applied as a constant external input via B
    # Since B*gamma is constant, add it outside the A*x term
    def rhs(x_):
        return f(x_) + np.array([0.0, 0.0])   # placeholder -- see below

    # Inline to avoid closure issues
    Bg = np.array([0.0, 0.0])   # filled below per caller

    k1 = A @ x
    k2 = A @ (x + dt/2 * k1)
    k3 = A @ (x + dt/2 * k2)
    k4 = A @ (x + dt   * k3)
    return x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)


def _step_flap(A, B, x, gamma, dt):
    """One RK4 step: dx/dt = A*x + B*gamma  (gamma constant over step)."""
    Bg = (B * gamma).flatten()
    k1 = A @ x + Bg
    k2 = A @ (x + dt/2 * k1) + Bg
    k3 = A @ (x + dt/2 * k2) + Bg
    k4 = A @ (x + dt   * k3) + Bg
    return x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)


def run_fig14():
    """
    Figure 14: controller output for varying Psi, varying amplitude,
    constant input error (+5 deg), and measurement noise.
    """
    print("Simulating Fig 14 (controller performance)...")
    A, B, C, D = _build_flap_system()
    U0_base    = np.radians(20.0)
    omega      = fc.OMEGA_THETA
    dt         = 0.013
    t_end      = 6.0
    N          = int(t_end / dt)
    t_arr      = np.arange(N + 1) * dt

    def make_ctrl():
        return fc.FlapController(A, B, C, D, omega=omega, dt=dt)

    # (a) Psi switches from 0 to pi/2 at t=3s
    def sim_psi_switch():
        ctrl = make_ctrl()
        beta_arr = np.zeros(N + 1)
        bref_arr = np.zeros(N + 1)
        x = np.zeros(2)
        for k in range(N):
            t   = t_arr[k]
            Psi = 0.0 if t < 3.0 else np.pi / 2.0
            bref = U0_base * np.sin(omega * t + Psi)
            bref_arr[k] = bref
            gamma, _ = ctrl.gamma_cl(t, U0_base, Psi, x[0])
            x = _step_flap(A, B, x, gamma, dt)
            beta_arr[k + 1] = x[0]
        bref_arr[-1] = U0_base * np.sin(omega * t_arr[-1])
        return np.degrees(beta_arr), np.degrees(bref_arr)

    # (b) Amplitude switches from 20 to 10 deg at t=3s
    def sim_amp_switch():
        ctrl = make_ctrl()
        beta_arr = np.zeros(N + 1)
        bref_arr = np.zeros(N + 1)
        x = np.zeros(2)
        for k in range(N):
            t  = t_arr[k]
            U0 = U0_base if t < 3.0 else U0_base * 0.5
            bref = U0 * np.sin(omega * t)
            bref_arr[k] = bref
            gamma, _ = ctrl.gamma_cl(t, U0, 0.0, x[0])
            x = _step_flap(A, B, x, gamma, dt)
            beta_arr[k + 1] = x[0]
        bref_arr[-1] = U0_base * np.sin(omega * t_arr[-1])
        return np.degrees(beta_arr), np.degrees(bref_arr)

    # (c) Constant +5 deg input error
    def sim_input_error():
        ctrl   = make_ctrl()
        offset = np.radians(5.0)
        beta_arr = np.zeros(N + 1)
        bref_arr = np.zeros(N + 1)
        x = np.zeros(2)
        for k in range(N):
            t    = t_arr[k]
            bref = U0_base * np.sin(omega * t)
            bref_arr[k] = bref
            gamma, _ = ctrl.gamma_cl(t, U0_base, 0.0, x[0])
            x = _step_flap(A, B, x, gamma + offset, dt)
            beta_arr[k + 1] = x[0]
        bref_arr[-1] = U0_base * np.sin(omega * t_arr[-1])
        return np.degrees(beta_arr), np.degrees(bref_arr)

    # (d) Measurement noise
    def sim_noise():
        ctrl      = make_ctrl()
        rng       = np.random.default_rng(42)
        noise_std = np.radians(2.0)
        beta_arr  = np.zeros(N + 1)
        bref_arr  = np.zeros(N + 1)
        x = np.zeros(2)
        for k in range(N):
            t    = t_arr[k]
            bref = U0_base * np.sin(omega * t)
            bref_arr[k] = bref
            beta_noisy  = x[0] + rng.normal(0, noise_std)
            gamma, _    = ctrl.gamma_cl(t, U0_base, 0.0, beta_noisy)
            x = _step_flap(A, B, x, gamma, dt)
            beta_arr[k + 1] = x[0]
        bref_arr[-1] = U0_base * np.sin(omega * t_arr[-1])
        return np.degrees(beta_arr), np.degrees(bref_arr)

    datasets = [
        (sim_psi_switch(),   "(a) Changing Psi"),
        (sim_amp_switch(),   "(b) Changing amplitude"),
        (sim_input_error(),  "(c) +5 deg input error"),
        (sim_noise(),        "(d) Measurement noise"),
    ]

    fig, axes = plt.subplots(4, 2, figsize=(12, 14))
    fig.suptitle("Fig 14 -- Closed-loop flap controller performance", fontsize=12)

    for row, ((b_, br_), title) in enumerate(datasets):
        err = br_ - b_
        axes[row, 0].plot(t_arr, br_, 'k--', lw=1.2, label='beta_ref')
        axes[row, 0].plot(t_arr, b_,  'b-',  lw=1.2, label='beta')
        axes[row, 0].set_title(title)
        axes[row, 0].set_ylabel("beta [deg]")
        axes[row, 0].legend(fontsize=8)
        axes[row, 0].grid(True, alpha=0.3)

        axes[row, 1].plot(t_arr, err, 'g-', lw=1.2, label='error')
        axes[row, 1].set_title(title + " -- error")
        axes[row, 1].set_ylabel("e [deg]")
        axes[row, 1].axhline(0, color='k', lw=0.5)
        axes[row, 1].grid(True, alpha=0.3)

    for ax in axes[-1]:
        ax.set_xlabel("Time [s]")

    fig.tight_layout()
    fig.savefig("fig14_controller.png", dpi=150)
    print("  saved fig14_controller.png")
    plt.close(fig)

    # Save CSV data
    os.makedirs(CSV_DIR, exist_ok=True)
    for (b_, br_), title in datasets:
        err = br_ - b_
        label = title[1]  # 'a', 'b', 'c', 'd'
        hdr = "time,beta,beta_ref,error"
        rows = np.column_stack([t_arr, b_, br_, err])
        np.savetxt(os.path.join(CSV_DIR, "sim_fig14_%s.csv" % label),
                   rows, delimiter=",", header=hdr, comments="", fmt="%.6f")
    print("  saved fig14 CSVs to %s/" % CSV_DIR)


def run_fig15():
    """
    Figure 15: +/-10% modelling error, with and without closed-loop correction.
    """
    print("Simulating Fig 15 (modelling error)...")
    A, B, _, _ = _build_flap_system()

    U0 = np.radians(20.0)

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle("Fig 15 -- Modelling error: open-loop vs closed-loop", fontsize=12)

    for col, use_cl in enumerate([False, True]):
        t_, b_, br_, _ = fc.simulate_flap(A, B, U0=U0,
                                           use_closed_loop=use_cl,
                                           model_error=0.1, t_end=6.0)
        err  = br_ - b_
        rmse = np.sqrt(np.mean(err**2))
        title = "Without closed-loop" if not use_cl else "With closed-loop"

        axes[0, col].plot(t_, br_, 'k--', lw=1.2, label='beta_ref')
        axes[0, col].plot(t_, b_,  'b-',  lw=1.2, label='beta')
        axes[0, col].set_title(title)
        axes[0, col].set_ylabel("beta [deg]")
        axes[0, col].legend(fontsize=8)
        axes[0, col].grid(True, alpha=0.3)

        axes[1, col].plot(t_, err, 'g-', lw=1.2,
                          label='error  RMSE=%.3f deg' % rmse)
        axes[1, col].set_ylabel("Error e [deg]")
        axes[1, col].set_xlabel("Time [s]")
        axes[1, col].axhline(0, color='k', lw=0.5)
        axes[1, col].legend(fontsize=8)
        axes[1, col].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig("fig15_model_error.png", dpi=150)
    print("  saved fig15_model_error.png")
    plt.close(fig)

    # Save CSV data
    os.makedirs(CSV_DIR, exist_ok=True)
    for col, use_cl in enumerate([False, True]):
        t_, b_, br_, _ = fc.simulate_flap(A, B, U0=U0,
                                           use_closed_loop=use_cl,
                                           model_error=0.1, t_end=6.0)
        err = br_ - b_
        label = "open_loop" if not use_cl else "closed_loop"
        hdr = "time,beta,beta_ref,error"
        rows = np.column_stack([t_, b_, br_, err])
        np.savetxt(os.path.join(CSV_DIR, "sim_fig15_%s.csv" % label),
                   rows, delimiter=",", header=hdr, comments="", fmt="%.6f")
    print("  saved fig15 CSVs to %s/" % CSV_DIR)


# ============================================================================
# Main
# ============================================================================

if __name__ == "__main__":
    print("=== RAWES Simulation -- Weyel (2025) ===\n")

    run_fig10()
    run_fig11()
    run_fig14()
    run_fig15()

    print("\nAll figures saved.")
