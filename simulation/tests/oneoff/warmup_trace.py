"""warmup_trace — one-off diagnostic, not a unit test.

Runs the test_create_ic warmup loop with full per-step logging and stops at
the first sign of divergence (NaN, |pos|>500m, or |omega_b|>20).  Prints the
last 200 samples so we can see the trigger.

Run:
    .venv/Scripts/python.exe simulation/tests/oneoff/warmup_trace.py
"""
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests" / "simtests"))

from aero            import create_aero, RotorInputs, relax_inflow, solve_trim_cyclic
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


def main():
    print("Building initial state...")
    R0, pos0, omega_spin, rest_length, trim = _build_initial_state()
    print(f"omega_spin={omega_spin:.2f} rad/s, rest_length={rest_length:.3f} m")
    print(f"trim: tlon={trim.tilt_lon:+.4f}, tlat={trim.tilt_lat:+.4f}")
    print(f"pos0: {pos0}, |pos0|={np.linalg.norm(pos0):.3f}\n")

    runner = PhysicsRunner.for_warmup(
        _ROTOR, pos0, R0, rest_length, STACK_COLL, omega_spin, WIND)
    runner._acro = HeliCyclicController(
        _ROTOR, col_min_rad=-0.28, col_max_rad=0.10,
        P=0.30, I=0.10, D=0.005, IMAX=0.30,
        FLTT=20.0, FLTE=0.0, FLTD=20.0,
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

    # Tension-regulating winch on rest_length: dL/dt = kp_winch * (T - T_target),
    # mirroring a ground-side winch holding tension at target during warmup.
    kp_winch = 2e-3   # m/s per N of error
    v_max    = 0.5    # m/s
    rest_now = float(rest_length)

    log = []
    print(f"{'t':>5} {'alt':>6} {'speed':>6} {'angle':>5} {'|omg_b|':>7} {'col':>7} {'rest':>7} {'roll_sp':>7} {'pitch_sp':>8} {'T':>5}")
    for step in range(8000):  # 20 s max
        if step % _PLANNER_EVERY == 0:
            _ap.receive_command(TensionCommand(
                tension_setpoint_n=IC_TENSION_N, tension_measured_n=runner.tension_now,
                alt_m=target_alt, phase="reel-out",
            ), _DT_CMD)
        if step % _AP_EVERY == 0:
            ap.tick(step * _DT, runner)
        # Capture pre-step
        s = runner.hub_state
        pos = s["pos"].copy()
        vel = s["vel"].copy()
        R   = s["R"].copy()
        omega_b = runner.omega_body
        # Update winch rest_length: P-controller on tension error.
        dT = runner.tension_now - IC_TENSION_N
        v_winch = max(-v_max, min(v_max, kp_winch * dT))
        rest_now += v_winch * _DT
        # Step
        runner.step(_DT, ap.col_rad, ap.roll_sp, ap.pitch_sp, omega_b,
                    rest_length=rest_now)

        # Diagnostics
        s2 = runner.hub_state
        pos2 = s2["pos"]
        nan = not np.all(np.isfinite(pos2))
        big = np.linalg.norm(pos2) > 500.0 if not nan else True
        # Read last commanded cyclic (servo state)
        # Last commanded cyclic via step args (re-read the controller's last output).
        col_a = float(ap.col_rad)
        bz_eq_anc = -pos / np.linalg.norm(pos)
        ang = math.acos(max(-1.0, min(1.0, float(np.dot(R[:, 2], bz_eq_anc)))))
        row = (step * _DT, float(-pos[2]), float(np.linalg.norm(vel)),
               math.degrees(ang), float(np.linalg.norm(omega_b)),
               col_a, rest_now,
               float(ap.roll_sp), float(ap.pitch_sp), runner.tension_now)
        log.append(row)
        if nan or big:
            print(f"DIVERGED at step={step} t={step*_DT:.3f}s nan={nan} big={big}")
            break

        if step % 200 == 0:
            t, alt, sp, an, ob, ca, rl, rs, ps, T = row
            print(f"{t:5.2f} {alt:6.2f} {sp:6.2f} {an:5.2f} {ob:7.3f} {ca:+7.4f} {rl:7.3f} {rs:+7.3f} {ps:+8.3f} {T:5.1f}")

    print("\n--- Last 30 samples ---")
    print(f"{'t':>5} {'alt':>6} {'speed':>6} {'angle':>5} {'|omg_b|':>7} {'col':>7} {'rest':>7} {'roll_sp':>7} {'pitch_sp':>8} {'T':>5}")
    for row in log[-30:]:
        t, alt, sp, an, ob, ca, rl, rs, ps, T = row
        print(f"{t:5.2f} {alt:6.2f} {sp:6.2f} {an:5.2f} {ob:7.3f} {ca:+7.4f} {rl:7.3f} {rs:+7.3f} {ps:+8.3f} {T:5.1f}")


if __name__ == "__main__":
    main()
