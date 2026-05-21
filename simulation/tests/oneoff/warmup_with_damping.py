"""warmup_with_damping — one-off diagnostic, not a unit test.

Adds lateral velocity damping (``controller.damp_bz_eq_lateral``) on top of the
TensionApController's bz_goal before the rate-cmd is built.  Checks whether the
pendulum mode that wouldn't settle in warmup_gain_sweep is damped by it.

Run:
    .venv/Scripts/python.exe simulation/tests/oneoff/warmup_with_damping.py
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
from controller      import (HeliCyclicController, compute_bz_altitude_hold,
                              compute_rate_cmd, damp_bz_eq_lateral)
from mediator        import TetherModel
from tests.simtests._rotor_helpers import load_default_rotor, BODY_Z_SLEW_RATE_RAD_S


_ROTOR  = load_default_rotor()
MASS    = _ROTOR.inertia.mass_kg
WIND    = np.array([0.0, 10.0, 0.0])
STACK_COLL   = -0.18
IC_TENSION_N = 300.0
L_TETHER     = 100.0
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


def run(kd_lat, t_total, R0, pos0, omega_spin, rest_length, trim,
        kp_outer=2.5, P=0.67, I=0.15, D=0.02, kp_winch=0.01):
    runner = PhysicsRunner.for_warmup(
        _ROTOR, pos0, R0, rest_length, STACK_COLL, omega_spin, WIND)
    runner._acro = HeliCyclicController(
        _ROTOR, col_min_rad=-0.28, col_max_rad=0.10,
        P=P, I=I, D=D, FLTT=40.0, FLTE=0.0, FLTD=40.0, IMAX=0.30,
    )
    runner._acro._servo.reset(STACK_COLL)
    runner._acro.set_trim(trim.tilt_lon, trim.tilt_lat)

    _ap = TensionApController(
        ic_pos=pos0, mass_kg=MASS,
        slew_rate_rad_s=BODY_Z_SLEW_RATE_RAD_S,
        warm_coll_rad=STACK_COLL, tension_ic=IC_TENSION_N,
        kp_outer=kp_outer, kp_pos=0.0, kd_pos=0.0,
    )

    target_alt = float(-pos0[2])
    _ap.receive_command(TensionCommand(
        tension_setpoint_n=IC_TENSION_N, tension_measured_n=runner.tension_now,
        alt_m=target_alt, phase="reel-out",
    ), _DT_CMD)

    rest_now = float(rest_length)
    v_max    = 1.0
    anchor   = np.zeros(3)

    # Cache last AP outputs between AP ticks.
    col_held = float(STACK_COLL)
    roll_sp_held = 0.0
    pitch_sp_held = 0.0

    n = int(round(t_total / _DT))
    log = []
    for step in range(n):
        if step % _PLANNER_EVERY == 0:
            _ap.receive_command(TensionCommand(
                tension_setpoint_n=IC_TENSION_N, tension_measured_n=runner.tension_now,
                alt_m=target_alt, phase="reel-out",
            ), _DT_CMD)
        if step % _AP_EVERY == 0:
            # Build the rate setpoint ourselves with lateral damping added.
            obs = runner.observe()
            # Advance the AP to update TensionPI / elevation state, but use the
            # collective only.
            col_held, _, _ = _ap.step(obs, dt=1.0 / PythonAP.AP_HZ)
            # Now recompute the rate command with lateral velocity damping.
            tlen      = float(np.linalg.norm(obs.pos))
            target_el = float(np.arcsin(max(-1.0, min(1.0, target_alt / max(tlen, 0.1)))))
            # Use the AP's internal rate-limited elevation.
            el_now = _ap.elevation_rad
            bz_goal = compute_bz_altitude_hold(obs.pos, el_now, runner.tension_now, MASS)
            if kd_lat > 0.0:
                bz_goal = damp_bz_eq_lateral(
                    bz_goal, obs.pos, obs.vel, anchor,
                    runner.tension_now, kd_lat,
                )
            R = obs.R
            rate_sp = compute_rate_cmd(R[:, 2], bz_goal, R, kp=kp_outer, kd=0.0)
            roll_sp_held  = float(rate_sp[0])
            pitch_sp_held = float(rate_sp[1])

        # Winch tension regulator
        dT = runner.tension_now - IC_TENSION_N
        v_winch  = max(-v_max, min(v_max, kp_winch * dT))
        rest_now += v_winch * _DT

        runner.step(_DT, col_held, roll_sp_held, pitch_sp_held,
                    runner.omega_body, rest_length=rest_now)

        s = runner.hub_state
        pos = s["pos"]
        if not np.all(np.isfinite(pos)) or np.linalg.norm(pos) > 1e4:
            print(f"DIVERGED at t={step*_DT:.3f}s")
            return None
        if step % 200 == 0:
            bz_eq = -pos / np.linalg.norm(pos)
            ang = math.acos(max(-1.0, min(1.0, float(np.dot(s["R"][:, 2], bz_eq)))))
            log.append((step*_DT, float(-pos[2]), float(np.linalg.norm(s["vel"])),
                        math.degrees(ang), runner.tension_now))

    return log


def main():
    print("Building initial state...")
    R0, pos0, omega_spin, rest_length, trim = _build_initial_state()
    print(f"omega_spin={omega_spin:.2f}, rest0={rest_length:.3f}\n")

    for kd_lat in [0.0, 5.0, 20.0, 50.0, 100.0]:
        print(f"\n=== kd_lat = {kd_lat} ===")
        log = run(kd_lat, t_total=60.0,
                  R0=R0, pos0=pos0, omega_spin=omega_spin,
                  rest_length=rest_length, trim=trim)
        if log is None:
            continue
        print(f"  {'t':>5} {'alt':>6} {'speed':>6} {'angle':>5} {'T':>5}")
        for row in log[::5]:
            t, alt, sp, an, T = row
            print(f"  {t:5.2f} {alt:6.2f} {sp:6.2f} {an:5.2f} {T:5.1f}")

        # Compute settling metrics over last 20 s
        late = [r for r in log if r[0] > 40.0]
        if late:
            alts = [r[1] for r in late]
            angs = [r[3] for r in late]
            spds = [r[2] for r in late]
            print(f"  Late (40-60s): alt_swing={max(alts)-min(alts):.2f}m, "
                  f"max_ang={max(angs):.2f}deg, max_speed={max(spds):.2f}m/s")


if __name__ == "__main__":
    main()
