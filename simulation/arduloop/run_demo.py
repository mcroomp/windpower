"""
End-to-end demo: tune the roll-axis rate loop against the tether-spring
crossover.

Run with:
    python -m arduloop.run_demo

Produces:
    * Step-response trace before / after adding the error notch at 3.77 Hz.
    * Empirical Bode of the closed-loop response.
    * Stability margins of the open loop, before / after.

Requires only numpy. Plots are optional (matplotlib if available).
"""

from __future__ import annotations

import numpy as np

from .params import HeliParams, RateAxisParams
from .attitude_heli import HeliRateController
from .plant import HeliPlant, PlantParams
from . import signals, analysis


def simulate(params: HeliParams,
             plant: HeliPlant,
             u_roll_target: np.ndarray,
             dt: float) -> dict:
    ctrl = HeliRateController(params)
    ctrl.reset()
    n = len(u_roll_target)
    p_meas = np.zeros(n)
    q_meas = np.zeros(n)
    out_r  = np.zeros(n)
    out_p  = np.zeros(n)
    p_cur = q_cur = r_cur = 0.0
    for i in range(n):
        o = ctrl.update(
            rate_target_rads=(u_roll_target[i], 0.0, 0.0),
            gyro_rate_rads=(p_cur, q_cur, r_cur),
            dt=dt,
            collective_norm=0.5,
        )
        p_cur, q_cur, r_cur = plant.step(o.roll_cyclic, o.pitch_cyclic,
                                         o.yaw_cmd, 0.5, dt)
        p_meas[i] = p_cur
        q_meas[i] = q_cur
        out_r[i]  = o.roll_cyclic
        out_p[i]  = o.pitch_cyclic
    return {"p": p_meas, "q": q_meas, "ur": out_r, "up": out_p}


def main() -> None:
    fs = 400.0
    dt = 1.0 / fs
    T = 4.0
    t = np.arange(0.0, T, dt)

    # Baseline parameter set — modest gains, no notches.
    base = HeliParams(loop_rate_hz=fs)
    base.roll  = RateAxisParams(P=0.12, I=0.10, D=0.004, FF=0.05, IMAX=0.3,
                                FLTT=20.0, FLTE=0.0, FLTD=20.0)
    base.pitch = RateAxisParams(P=0.12, I=0.10, D=0.004, FF=0.05, IMAX=0.3,
                                FLTT=20.0, FLTE=0.0, FLTD=20.0)

    # Tuned: error notch on the tether-spring frequency, lowered FLTT for the
    # outer loop's expected pendulum coupling band.
    tuned = HeliParams(loop_rate_hz=fs, H_SW_H3_PHANG=0.0)
    tuned.roll  = RateAxisParams(P=0.12, I=0.10, D=0.004, FF=0.05, IMAX=0.3,
                                 FLTT=1.5, FLTE=0.0, FLTD=20.0,
                                 NEF_center_hz=3.77, NEF_bandwidth_hz=0.5,
                                 NEF_attn_db=40.0)
    tuned.pitch = RateAxisParams(P=0.12, I=0.10, D=0.004, FF=0.05, IMAX=0.3,
                                 FLTT=1.5, FLTE=0.0, FLTD=20.0,
                                 NEF_center_hz=3.77, NEF_bandwidth_hz=0.5,
                                 NEF_attn_db=40.0)

    # ---------------- Step response ----------------
    u_step = signals.step(t, amplitude=1.0, t0=0.2)
    res_base  = simulate(base,  HeliPlant(PlantParams()), u_step, dt)
    res_tuned = simulate(tuned, HeliPlant(PlantParams()), u_step, dt)

    score_base  = analysis.step_response_score(t, res_base["p"],  target=1.0)
    score_tuned = analysis.step_response_score(t, res_tuned["p"], target=1.0)
    print("Step response — baseline:", score_base)
    print("Step response — tuned   :", score_tuned)

    # ---------------- Bode (closed-loop p / target) ----------------
    u_chirp = signals.logarithmic_chirp(t, f0=0.1, f1=50.0, amplitude=0.3)
    cl_base  = simulate(base,  HeliPlant(PlantParams()), u_chirp, dt)
    cl_tuned = simulate(tuned, HeliPlant(PlantParams()), u_chirp, dt)
    f, H_base  = analysis.empirical_frf(u_chirp, cl_base["p"],  fs=fs)
    _, H_tuned = analysis.empirical_frf(u_chirp, cl_tuned["p"], fs=fs)
    print(f"Closed-loop |H| at 3.77 Hz — baseline:"
          f" {np.interp(3.77, f, np.abs(H_base)):.3f}")
    print(f"Closed-loop |H| at 3.77 Hz — tuned   :"
          f" {np.interp(3.77, f, np.abs(H_tuned)):.3f}")

    # ---------------- Plot if matplotlib is around ----------------
    try:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 1, figsize=(8, 6))
        ax[0].plot(t, res_base["p"],  label="baseline p")
        ax[0].plot(t, res_tuned["p"], label="tuned p")
        ax[0].plot(t, u_step, "k--", label="target")
        ax[0].set_xlabel("time [s]"); ax[0].set_ylabel("roll rate [rad/s]")
        ax[0].legend()
        ax[1].semilogx(f, 20*np.log10(np.abs(H_base)+1e-9),  label="baseline")
        ax[1].semilogx(f, 20*np.log10(np.abs(H_tuned)+1e-9), label="tuned")
        ax[1].axvline(3.77, color="r", ls=":", label="tether spring")
        ax[1].set_xlabel("Hz"); ax[1].set_ylabel("|H| [dB]")
        ax[1].set_xlim(0.1, 100); ax[1].legend()
        plt.tight_layout(); plt.show()
    except ImportError:
        pass


if __name__ == "__main__":
    main()
