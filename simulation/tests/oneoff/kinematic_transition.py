"""kinematic_transition — one-off diagnostic, not a unit test.

Replicates the stack test's kinematic -> free-flight transition in Python so
we can iterate on the SITL crash without paying SITL startup cost.

Setup:
  * Load IC from steady_state_starting.json.
  * Build PhysicsCore with the kinematic StartupDamper (proper kinematic
    pos+orientation hold + damp_alpha ramp — the same plumbing the mediator
    uses).
  * Run a HeliCyclicController whose gains match ArduPilot's *actual* boot
    params on the heli SITL (P=0.06, I=0.105, FF=0.21, FLTT=20, FLTE=20).
    These are what we observed in the params.json from the SITL run.
  * Drive the controller with the same rate setpoints rawes.lua would emit
    (compute_bz_altitude_hold + compute_rate_cmd).
  * Run through kinematic phase + free flight.

If the body-rate runaway reproduces, we know it's a Python-side issue with
the (ArduPilot-default rate-PID, new aero) combination — independent of SITL.

Run:
    .venv/Scripts/python.exe simulation/tests/oneoff/kinematic_transition.py
"""
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests" / "simtests"))

from aero            import solve_trim_cyclic
from physics_core    import PhysicsCore
from controller      import (
    HeliCyclicController, compute_bz_altitude_hold, compute_rate_cmd,
    damp_bz_eq_lateral,
)
from kinematic       import KinematicStartup, make_linear_traj
from simtest_ic      import load_ic
from tests.simtests._rotor_helpers import load_default_rotor

_ROTOR  = load_default_rotor()
MASS    = float(_ROTOR.inertia.mass_kg)
WIND    = np.array([0.0, 10.0, 0.0])
DT      = 1.0 / 400.0
HOLD_S  = 10.0       # mimics the 80-s kinematic, compressed
RUN_S   = 12.0       # total simulation time
ANCHOR  = np.zeros(3)


# Rate PID gains:
#   stack    = ArduPilot heli defaults (the SITL params.json values that
#              produced the crash).
#   tuned    = arduloop gains from tests/oneoff/warmup_gain_sweep.py.
CONFIGS = {
    "stack-default":  dict(P=0.06, I=0.105, D=1e-4, FF=0.21,
                           FLTT=20.0, FLTE=20.0, FLTD=0.0, IMAX=0.0),
    "tuned":          dict(P=0.67, I=0.15,  D=0.02, FF=0.0,
                           FLTT=40.0, FLTE=0.0,  FLTD=40.0, IMAX=0.30),
}


def _make_runner(P, I, D, FF, FLTT, FLTE, FLTD, IMAX, ic, kin):
    core = PhysicsCore(
        _ROTOR, ic, WIND, kinematic=kin, z_floor=-1.0,
        base_k_ang=50.0, startup_damp_k_ang=500.0,
    )
    acro = HeliCyclicController(
        _ROTOR, col_min_rad=-0.28, col_max_rad=0.10,
        P=P, I=I, D=D, FF=FF, IMAX=IMAX,
        FLTT=FLTT, FLTE=FLTE, FLTD=FLTD,
    )
    acro._servo.reset(ic.coll_eq_rad)
    return core, acro


def run(label: str, params: dict, *, use_trim: bool = True,
        kd_lat: float = 0.0):
    """Run a 12-second sim: HOLD_S held + remainder free flight."""
    ic = load_ic()
    # Wrap in SimpleNamespace because PhysicsCore expects attribute access
    ic_ns = SimpleNamespace(
        pos        = np.asarray(ic.pos, dtype=float),
        vel        = np.asarray(ic.vel, dtype=float),
        R0         = np.asarray(ic.R0,  dtype=float).reshape(3, 3),
        rest_length= float(ic.rest_length),
        coll_eq_rad= float(ic.stack_coll_eq),
        omega_spin = float(ic.omega_spin),
    )

    kin = KinematicStartup(
        target_pos = ic_ns.pos,
        target_vel = np.zeros(3),
        duration   = HOLD_S,
        ramp_s     = 0.0,
        R0         = ic_ns.R0,
    )
    core, acro = _make_runner(**params, ic=ic_ns, kin=kin)

    if use_trim:
        trim = solve_trim_cyclic(
            core._aero, core._rotor_state,
            collective_rad=ic_ns.coll_eq_rad,
            R_hub=ic_ns.R0, v_hub_world=np.zeros(3), wind_world=WIND,
            tolerance_Nm=0.2, n_inflow_relax=200, dt_relax=DT,
            fix_omega=True,
        )
        acro.set_trim(trim.tilt_lon, trim.tilt_lat)
        core._rotor_state = trim.final_state
    else:
        trim = None

    target_alt = float(-ic_ns.pos[2])
    rest_now   = float(ic_ns.rest_length)
    tension_target_n = 300.0
    kp_winch = 0.01
    v_max    = 1.0

    n_steps  = int(round(RUN_S / DT))
    n_hold   = int(round(HOLD_S / DT))
    released = False
    log = []
    exit_announced = False

    for i in range(n_steps):
        t = i * DT
        if i == n_hold and not exit_announced:
            exit_announced = True
            released = True
            print(f"\n=== {label}: kinematic exit at t={t:.3f}s ===")

        # Lua-equivalent: bz_altitude_hold + optional lateral damping
        hub = core.hub_state
        pos = hub["pos"]
        R   = hub["R"]
        bz_now = R[:, 2]
        # Lua passes _tension_n (defaults 200, updated by RAWES_TEN).  The
        # ground test does not stream RAWES_TEN, so the Lua sees ~200 N.
        # Reproduce by clamping the tension fed to bz_altitude_hold.
        tension_for_bz = max(core.tension_now, 50.0)
        bz_goal = compute_bz_altitude_hold(
            pos, math.asin(max(-1.0, min(1.0, target_alt/max(np.linalg.norm(pos), 0.1)))),
            tension_for_bz, MASS,
        )
        if kd_lat > 0:
            bz_goal = damp_bz_eq_lateral(
                bz_goal, pos, hub["vel"], ANCHOR, core.tension_now, kd_lat,
            )
        rate_sp = compute_rate_cmd(bz_now, bz_goal, R, kp=1.0, kd=0.0)
        omega_b = R.T @ hub["omega"]

        # HeliCyclicController inner loop (arduloop AC_PID + servo)
        tlon, tlat, col_act = acro.step(
            collective_cmd=ic_ns.coll_eq_rad,
            rate_roll_sp =float(rate_sp[0]),
            rate_pitch_sp=float(rate_sp[1]),
            omega_body   =omega_b,
            dt           =DT,
        )

        # Winch tension regulator — only during free flight (kinematic
        # reports tension=0 and would otherwise drive rest_length away
        # from the IC).
        if released:
            dT = core.tension_now - tension_target_n
            v_winch = max(-v_max, min(v_max, kp_winch * dT))
            rest_now += v_winch * DT

        core.step(DT, col_act, tlon, tlat, rest_length=rest_now)

        # Sample every 5 ms for 200 ms after release, then 100 ms intervals.
        post = i - n_hold
        sample = False
        if released:
            if 0 <= post <= 80:
                sample = (post % 2 == 0)
            else:
                sample = (post % 40 == 0)
        else:
            sample = (i % 400 == 0)
        if sample:
            hub2 = core.hub_state
            wb_b = hub2["R"].T @ hub2["omega"]
            log.append((
                t, *hub2["pos"], *hub2["vel"], *wb_b,
                tlon, tlat, col_act, core.tension_now,
                core.omega_spin,
            ))
        # Stop early on divergence.
        if not np.all(np.isfinite(core.hub_state["pos"])):
            print(f"  ! NaN at t={t:.3f}s")
            break

    print(f"{label}: {'%-7s' % 't':>5} {'alt':>6} {'tens':>6} {'wb_x':>7} {'wb_y':>7} {'wb_z':>7} {'tlon':>6} {'tlat':>6} {'omS':>6}")
    for row in log:
        t, px, py, pz, vx, vy, vz, wbx, wby, wbz, tl, ta, ca, tn, om = row
        marker = "*" if t >= HOLD_S and t < HOLD_S + 0.3 else " "
        print(f"  {marker} {t:6.3f} {-pz:6.2f} {tn:6.1f} {wbx:+7.3f} {wby:+7.3f} {wbz:+7.3f} {tl:+6.3f} {ta:+6.3f} {om:6.2f}")

    if trim is not None:
        print(f"{label}: trim tlon={trim.tilt_lon:+.4f} tlat={trim.tilt_lat:+.4f}\n")


def main():
    print("=" * 80)
    print("kinematic_transition: replicate SITL kinematic-exit crash in Python")
    print("=" * 80)
    for label, p in CONFIGS.items():
        run(label, p)
    print("\n--- Tuned + lateral damping (apples-to-apples with simtest) ---")
    run("tuned+kd_lat50", CONFIGS["tuned"], kd_lat=50.0)


if __name__ == "__main__":
    main()
