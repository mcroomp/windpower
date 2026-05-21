"""warmup_gain_sweep — one-off diagnostic, not a unit test.

Sweeps HeliCyclicController gains on the test_create_ic warmup loop *with* a
tension-regulating winch (P-controller on rest_length).  Picks the best gain
set for the elastic-tether + wind + TensionPI environment.

Run:
    .venv/Scripts/python.exe simulation/tests/oneoff/warmup_gain_sweep.py
"""
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests" / "simtests"))

from dynbem            import create_aero, RotorInputs, relax_inflow, solve_trim_cyclic
from frames          import build_orb_frame
from simtest_runner  import PhysicsRunner, PythonAP
from ap_controller   import TensionApController
from pumping_planner import TensionCommand
from controller      import HeliCyclicController
from mediator        import TetherModel
from tests.simtests._rotor_helpers import load_default_rotor, BODY_Z_SLEW_RATE_RAD_S


_ROTOR  = load_default_rotor()
MASS    = _ROTOR.inertia.mass_kg
WIND    = np.array([0.0, 10.0, 0.0])
STACK_COLL = -0.18
IC_TENSION_N = 300.0
L_TETHER = 100.0
_BODY_Z_DESIGN = np.array([0.305391, 0.851018, -0.427206])
_DT = 2.5e-3
_DT_CMD = 0.1
_PLANNER_EVERY = max(1, round(_DT_CMD / _DT))
_AP_EVERY      = max(1, round(1.0 / (PythonAP.AP_HZ * _DT)))


def _build_initial_state():
    tether_hat = _BODY_Z_DESIGN / np.linalg.norm(_BODY_Z_DESIGN)
    pos0  = L_TETHER * tether_hat
    R0    = build_orb_frame(-tether_hat)

    aero  = create_aero(_ROTOR, model="oye")
    state = aero.initial_rotor_state()
    state.omega_rad_s = 20.0
    dt_eq = 1.0 / 400.0
    relax_kw  = dict(
        collective_rad=STACK_COLL, tilt_lon=0.0, tilt_lat=0.0,
        R_hub=R0, v_hub_world=np.zeros(3), wind_world=WIND,
        dt=dt_eq, t=PhysicsRunner.T_AERO_OFFSET,
    )
    inputs_eq = RotorInputs(
        collective_rad=STACK_COLL, tilt_lon=0.0, tilt_lat=0.0,
        R_hub=R0, v_hub_world=np.zeros(3), wind_world=WIND,
        t=PhysicsRunner.T_AERO_OFFSET,
    )
    for _ in range(8):
        state = relax_inflow(aero, state, n_steps=400, fix_omega=True, **relax_kw)
        for _ in range(200):
            _, deriv = aero.compute_forces(inputs_eq, state)
            state = state.from_array(state.to_array() + dt_eq * deriv.to_array())
    omega_spin = float(state.omega_rad_s)
    trim = solve_trim_cyclic(
        aero, state,
        collective_rad=STACK_COLL,
        R_hub=R0, v_hub_world=np.zeros(3), wind_world=WIND,
        n_inflow_relax=200, dt_relax=dt_eq, fix_omega=True,
        tolerance_Nm=0.1, t=PhysicsRunner.T_AERO_OFFSET,
    )
    state = trim.final_state
    f_est, _ = aero.compute_forces(inputs_eq, state)
    T_est = max(-float(np.dot(f_est.F_world, -tether_hat)), 10.0)
    k_eff = TetherModel.EA_N / L_TETHER
    rest_length = L_TETHER - max(T_est / k_eff, 0.001)
    return R0, pos0, omega_spin, rest_length, trim


def run_combo(P, I, D, FLTT, kp_winch, t_total,
              R0, pos0, omega_spin, rest_length, trim):
    runner = PhysicsRunner.for_warmup(
        _ROTOR, pos0, R0, rest_length, STACK_COLL, omega_spin, WIND)
    runner._acro = HeliCyclicController(
        _ROTOR, col_min_rad=-0.28, col_max_rad=0.10,
        P=P, I=I, D=D, FLTT=FLTT, FLTE=0.0, FLTD=FLTT, IMAX=0.30,
    )
    runner._acro._servo.reset(STACK_COLL)
    runner._acro.set_trim(trim.tilt_lon, trim.tilt_lat)

    _ap = TensionApController(
        ic_pos=pos0, mass_kg=MASS,
        slew_rate_rad_s=BODY_Z_SLEW_RATE_RAD_S,
        warm_coll_rad=STACK_COLL, tension_ic=IC_TENSION_N,
        kp_pos=0.0, kd_pos=0.0,
    )
    ap = PythonAP(_ap, wind=WIND, dt=_DT)
    target_alt = float(-pos0[2])
    _ap.receive_command(TensionCommand(
        tension_setpoint_n=IC_TENSION_N, tension_measured_n=runner.tension_now,
        alt_m=target_alt, phase="reel-out",
    ), _DT_CMD)

    rest_now  = float(rest_length)
    v_max     = 1.0

    n = int(round(t_total / _DT))
    alt0 = -pos0[2]
    max_angle = 0.0
    min_T     = 9999.0
    max_T     = 0.0
    n_T_zero  = 0
    min_alt   = alt0
    max_alt   = alt0
    rms_T_err = 0.0
    n_samp    = 0
    diverged_at = None
    last_half_alt = []
    last_half_ang = []

    for step in range(n):
        if step % _PLANNER_EVERY == 0:
            _ap.receive_command(TensionCommand(
                tension_setpoint_n=IC_TENSION_N, tension_measured_n=runner.tension_now,
                alt_m=target_alt, phase="reel-out",
            ), _DT_CMD)
        if step % _AP_EVERY == 0:
            ap.tick(step * _DT, runner)
        dT_err = runner.tension_now - IC_TENSION_N
        v_winch = max(-v_max, min(v_max, kp_winch * dT_err))
        rest_now += v_winch * _DT
        runner.step(_DT, ap.col_rad, ap.roll_sp, ap.pitch_sp, runner.omega_body,
                    rest_length=rest_now)

        s = runner.hub_state
        pos = s["pos"]
        if not np.all(np.isfinite(pos)) or np.linalg.norm(pos) > 1e4:
            diverged_at = step * _DT
            break
        if step % 40 == 0:
            bz_eq = -pos / np.linalg.norm(pos)
            ang = math.acos(max(-1.0, min(1.0, float(np.dot(s["R"][:, 2], bz_eq)))))
            max_angle = max(max_angle, ang)
            alt = float(-pos[2])
            min_alt = min(min_alt, alt)
            max_alt = max(max_alt, alt)
            T = runner.tension_now
            min_T = min(min_T, T)
            max_T = max(max_T, T)
            if T < 1.0:
                n_T_zero += 1
            rms_T_err += (T - IC_TENSION_N) ** 2
            n_samp += 1
            if step * _DT > 0.5 * t_total:
                last_half_alt.append(alt)
                last_half_ang.append(math.degrees(ang))

    rms_T_err = math.sqrt(rms_T_err / max(n_samp, 1))
    alt_swing_late = (max(last_half_alt) - min(last_half_alt)) if last_half_alt else float('inf')
    ang_max_late   = max(last_half_ang) if last_half_ang else float('inf')
    return dict(
        diverged_at=diverged_at,
        max_angle_deg=math.degrees(max_angle),
        alt_swing=max_alt - min_alt,
        alt_swing_late=alt_swing_late,
        ang_max_late=ang_max_late,
        n_T_zero=n_T_zero,
        min_T=min_T,
        max_T=max_T,
        rms_T_err=rms_T_err,
    )


def main():
    import os
    print("Building initial state...")
    R0, pos0, omega_spin, rest_length, trim = _build_initial_state()
    print(f"omega_spin={omega_spin:.2f}, rest0={rest_length:.3f}, "
          f"trim=({trim.tilt_lon:+.4f},{trim.tilt_lat:+.4f})\n")

    if os.environ.get("LONG"):
        # Long-run test: best combos at 60 s
        print("Long run - 60 s on selected combos")
        print(f"{'P':>5} {'I':>5} {'D':>6} {'FLTT':>5} {'kp_w':>6} | {'dvg':>5} {'ang_lat':>7} {'alt_sw_lat':>10} {'minT':>5} {'rmsT':>6}")
        for combo in [
            (0.67, 0.15, 0.02, 40.0, 0.01),
            (0.67, 0.15, 0.02, 40.0, 0.03),
            (0.45, 0.15, 0.02, 40.0, 0.03),
            (0.67, 0.30, 0.02, 40.0, 0.03),
            (0.67, 0.15, 0.05, 40.0, 0.03),
            (1.0,  0.30, 0.02, 40.0, 0.03),
        ]:
            P, I, D, FLTT, kp_w = combo
            r = run_combo(P, I, D, FLTT, kp_w, 60.0,
                          R0, pos0, omega_spin, rest_length, trim)
            div = f"{r['diverged_at']:.1f}" if r['diverged_at'] else "  ok"
            print(f"{P:5.2f} {I:5.2f} {D:6.3f} {FLTT:5.1f} {kp_w:6.3f} | {div:>5} {r['ang_max_late']:7.2f} {r['alt_swing_late']:10.2f} {r['min_T']:5.0f} {r['rms_T_err']:6.1f}")
        return

    print("Stage 1 - 10 s, kp_winch=0.01")
    print(f"{'P':>5} {'I':>5} {'D':>6} {'FLTT':>5} | {'dvg':>5} {'ang_lat':>7} {'alt_sw_lat':>10} {'minT':>5} {'rmsT':>6} {'nT0':>4}")
    # broader winch + gain grid
    survivors = []
    for kp_w in [0.01]:
        for P in [0.20, 0.30, 0.45, 0.67]:
            for I in [0.05, 0.15, 0.30]:
                for D in [0.005, 0.02]:
                    for FLTT in [20.0, 40.0]:
                        r = run_combo(P, I, D, FLTT, kp_w, 10.0,
                                       R0, pos0, omega_spin, rest_length, trim)
                        div = f"{r['diverged_at']:.1f}" if r['diverged_at'] else "  ok"
                        print(f"{P:5.2f} {I:5.2f} {D:6.3f} {FLTT:5.1f} | {div:>5} {r['ang_max_late']:7.2f} {r['alt_swing_late']:10.2f} {r['min_T']:5.0f} {r['rms_T_err']:6.1f} {r['n_T_zero']:4d}")
                        if (r['diverged_at'] is None and r['ang_max_late'] < 15.0
                                and r['alt_swing_late'] < 10.0):
                            survivors.append((P, I, D, FLTT, kp_w, r))

    survivors.sort(key=lambda x: (x[5]['alt_swing_late'] + 0.5*x[5]['ang_max_late'] + 0.01*x[5]['rms_T_err']))
    survivors = survivors[:8]

    print("\nStage 2 - 30 s on top survivors")
    print(f"{'P':>5} {'I':>5} {'D':>6} {'FLTT':>5} {'kp_w':>6} | {'dvg':>5} {'ang_lat':>7} {'alt_sw_lat':>10} {'minT':>5} {'rmsT':>6} {'nT0':>4}")
    for P, I, D, FLTT, kp_w, _ in survivors:
        r = run_combo(P, I, D, FLTT, kp_w, 30.0,
                      R0, pos0, omega_spin, rest_length, trim)
        div = f"{r['diverged_at']:.1f}" if r['diverged_at'] else "  ok"
        print(f"{P:5.2f} {I:5.2f} {D:6.3f} {FLTT:5.1f} {kp_w:6.3f} | {div:>5} {r['ang_max_late']:7.2f} {r['alt_swing_late']:10.2f} {r['min_T']:5.0f} {r['rms_T_err']:6.1f} {r['n_T_zero']:4d}")


if __name__ == "__main__":
    main()
