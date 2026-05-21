"""
test_generate_ic.py — Create and verify steady-state initial conditions.

Two tests:
  test_create_ic        — warmup physics to find settled state; saves steady_state_starting.json
  test_ic_steady_flight — loads the saved JSON; runs 10 s more physics; asserts hub stays steady
                          (serialize + deserialize round-trip)

Regenerate:
    .venv/Scripts/python.exe -m pytest simulation/tests/unit/test_generate_ic.py -s

WHY WARMUP-BASED (not purely analytical)
-----------------------------------------
An analytical IC (pos, vel=0, rest_length from thrust estimate) encodes zero velocity and an
orientation derived from build_orb_frame, whose yaw may not match the hub's natural free-flight
heading.  The GPS Glitch detection in ArduPilot compares compass heading (= R0 yaw) against
GPS velocity heading; a mismatch at kinematic exit triggers a GPS Glitch that collapses EKF
fusion.

Running a warmup with TensionPI + orbit-tracking lets the hub settle to a real physics state:
real velocity, real R0 yaw, and a tether tension consistent with the collective.  That state
is a valid starting point for both simtests (internal controller) and stack tests (ArduPilot +
Lua), and the R0 yaw will be consistent with the hub's initial free-flight velocity direction.

NED coordinates throughout.  X=North, Y=East, Z=Down.
"""
import math
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

pytestmark = [pytest.mark.simtest, pytest.mark.timeout(300)]

import mediator as _mediator_module
from dynbem            import create_aero, RotorInputs, relax_inflow, solve_trim_cyclic
from frames          import build_orb_frame
from simtest_runner  import PhysicsRunner, PythonAP
from ap_controller   import TensionApController
from pumping_planner import TensionCommand
from tests.simtests._rotor_helpers import (
    load_default_rotor, dynamics_kwargs, BODY_Z_SLEW_RATE_RAD_S,
)

TetherModel = _mediator_module.TetherModel

# ── Rotor / physics constants ──────────────────────────────────────────────────
_ROTOR  = load_default_rotor()
_DYN_KW = dynamics_kwargs(_ROTOR)
MASS    = _DYN_KW["mass"]
G       = 9.81

WIND = np.array([0.0, 10.0, 0.0])   # NED: 10 m/s East
WIND.flags.writeable = False

# ── Collectives ────────────────────────────────────────────────────────────────
STACK_COLL = -0.18  # rad — col_cruise used by Lua at kinematic exit (COL_MIN=-0.28 + 0.10)

# ── IC target tension ──────────────────────────────────────────────────────────
# Midway between pumping reel-in (226 N) and reel-out (435 N) targets.
# TensionPI adjusts collective during warmup until settled tension ≈ this value.
IC_TARGET_TENSION_N = 300.0   # N

# ── IC quality bounds (asserted in test_create_ic) ────────────────────────────
IC_TENSION_MIN_N  =  50.0   # N  — must not be slack or nearly slack
IC_TENSION_MAX_N  = 500.0   # N  — below ~80% of break load (620 N)
IC_SPEED_MAX_MS   =   2.0   # m/s — IC should be nearly stationary
IC_ELEV_MIN_DEG   =   5.0   # deg — must be above horizon
IC_ELEV_MAX_DEG   =  55.0   # deg — must not be nearly vertical

# ── Tether / geometry ──────────────────────────────────────────────────────────
L_TETHER = 100.0   # m

# Design orientation (from beaupoil_2026.yaml, NED): ~25 deg elevation, ~70 deg E-of-N
_BODY_Z_DESIGN = np.array([0.305391, 0.851018, -0.427206])
_BODY_Z_DESIGN.flags.writeable = False

# ── Simulation parameters ──────────────────────────────────────────────────────
_DT            = 2.5e-3  # s (400 Hz)
WARMUP_STEPS   = 24000   # 60 s warmup — hub settles to natural equilibrium
_STEADY_STEPS  = 12000   # 30 s steady-flight check
_DRIFT_BOUND   = 15.0    # m — max 3-D drift in steady-flight check
_DT_CMD        = 0.1     # 10 Hz ground commands
_PLANNER_EVERY = max(1, round(_DT_CMD / _DT))                    # 40
_AP_EVERY      = max(1, round(1.0 / (PythonAP.AP_HZ * _DT)))    # 8

_JSON_PATH = Path(__file__).resolve().parents[2] / "steady_state_starting.json"


# ── IC computation ─────────────────────────────────────────────────────────────

def _compute_ic() -> dict:
    """
    Run warmup physics (60 s, TensionPI at IC_TARGET_TENSION_N, orbit-tracking)
    from the design orientation and return the settled state as the IC.

    The warmup lets the hub find its natural equilibrium: real velocity, real R0
    yaw consistent with the free-flight trajectory, and tether tension balanced
    at IC_TARGET_TENSION_N.  This avoids GPS Glitch at kinematic exit in stack
    tests (compass heading = R0 yaw matches GPS velocity heading).
    """
    # _BODY_Z_DESIGN encodes the geometric anchor→hub unit vector for the
    # design point.  FRD body_z points hub→anchor (down through the disk).
    tether_hat = _BODY_Z_DESIGN / np.linalg.norm(_BODY_Z_DESIGN)
    pos0  = L_TETHER * tether_hat
    t_dir = -tether_hat
    R0    = build_orb_frame(t_dir)

    # ── Initial omega_spin via aero.relax_inflow + a few explicit ω steps ──
    # ``relax_inflow`` settles the dynamic-inflow states semi-implicitly with
    # ω held fixed; we then take a handful of explicit Euler steps with ω
    # free so the spin ODE finds its autorotation equilibrium.
    _aero_est = create_aero(_ROTOR, model="oye")
    state     = _aero_est.initial_rotor_state()
    state.omega_rad_s = 20.0
    omega_min = _ROTOR.autorotation.omega_min_rad_s or 0.5
    dt_eq     = 1.0 / 400.0
    relax_kw  = dict(
        collective_rad=STACK_COLL, tilt_lon=0.0, tilt_lat=0.0,
        R_hub=R0, v_hub_world=np.zeros(3), wind_world=WIND,
        dt=dt_eq, t=PhysicsRunner.T_AERO_OFFSET,
    )
    # Alternate: settle inflow at current ω, then take 200 explicit-Euler
    # steps with ω free to allow the spin ODE to find equilibrium.  Two
    # cycles is enough — ω drifts < 0.5 rad/s between them at convergence.
    inputs_eq = RotorInputs(
        collective_rad=STACK_COLL, tilt_lon=0.0, tilt_lat=0.0,
        R_hub=R0, v_hub_world=np.zeros(3), wind_world=WIND,
        t=PhysicsRunner.T_AERO_OFFSET,
    )
    for _outer in range(8):
        state = relax_inflow(_aero_est, state, n_steps=400, fix_omega=True, **relax_kw)
        for _ in range(200):
            _, deriv = _aero_est.compute_forces(inputs_eq, state)
            state = state.from_array(state.to_array() + dt_eq * deriv.to_array())
            if state.omega_rad_s < omega_min:
                state.omega_rad_s = float(omega_min)
    omega_spin = float(state.omega_rad_s)

    # ── Trim cyclic at the IC ──────────────────────────────────────────────
    # Find the (tilt_lon, tilt_lat) that null hub-frame Mx, My at the IC
    # operating point.  Used as a feedforward in HeliCyclicController so the
    # P-only rate loop doesn't have to fight the wind-driven baseline moment.
    trim = solve_trim_cyclic(
        _aero_est, state,
        collective_rad=STACK_COLL,
        R_hub=R0, v_hub_world=np.zeros(3), wind_world=WIND,
        n_inflow_relax=200, dt_relax=dt_eq, fix_omega=True,
        tolerance_Nm=0.1, t=PhysicsRunner.T_AERO_OFFSET,
    )
    state = trim.final_state

    # Initial rest_length from aero thrust estimate at the trimmed state.
    f_est, _ = _aero_est.compute_forces(inputs_eq, state)
    T_est = max(-float(np.dot(f_est.F_world, t_dir)), 10.0)
    k_eff = TetherModel.EA_N / L_TETHER
    rest_length = L_TETHER - max(T_est / k_eff, 0.001)

    # Warmup: PhysicsRunner + TensionApController targeting IC_TARGET_TENSION_N.
    # The AcroController is pre-loaded with the trim cyclic so the inner PID
    # only handles small perturbations around equilibrium.  Re-bind the
    # AcroController with ki/kd gains so the rate loop has damping +
    # integral action — required for stability under wind disturbance
    # (see tests/unit/test_full_loop_stability for the unit-level demo).
    from controller import HeliCyclicController as _AcroForWarmup
    runner = PhysicsRunner.for_warmup(_ROTOR, pos0, R0, rest_length,
                                       STACK_COLL, omega_spin, WIND)
    # Inner-PID gains: pure-P at default kp.  Adding ki/kd was destabilising
    # the warmup; integral wind-up under the wind-driven baseline moment +
    # the new derivative-kick fix combine to cause NaN explosion at 50 Hz
    # AP rate.  Trim FF alone handles steady-state moments.
    # Tuned for elastic tether + wind + 50 Hz AP rate.  Found in
    # tests/oneoff/warmup_gain_sweep.py — D=0.02 + FLTT=40 hold tension
    # near setpoint by damping the tether-spring excitation through the
    # rate-PID's filter; without them tension swings 0 - 800 N and the
    # loop blows up in ~9 s.
    runner._acro = _AcroForWarmup(
        _ROTOR, col_min_rad=-0.28, col_max_rad=0.10,
        P=0.67, I=0.15, D=0.02, IMAX=0.30,
        FLTT=40.0, FLTE=0.0, FLTD=40.0,
    )
    runner._acro._servo.reset(STACK_COLL)
    runner._acro.set_trim(trim.tilt_lon, trim.tilt_lat)
    _ap_wu = TensionApController(
        ic_pos=pos0, mass_kg=MASS,
        slew_rate_rad_s=BODY_Z_SLEW_RATE_RAD_S,
        warm_coll_rad=STACK_COLL, tension_ic=IC_TARGET_TENSION_N,
        # AP-side position feedback disabled here.  The PD helper
        # ``controller.position_feedback_bz_eq`` works in the constant-
        # tether-force unit tests (see test_full_loop_stability) but
        # destabilises the elastic-tether simtest at 50 Hz AP rate.
        # Lateral-velocity damping (kd_lat) is sufficient to settle the
        # pendulum mode for IC generation.
        kp_pos=0.0, kd_pos=0.0, kd_lat=50.0,
    )
    ap_wu = PythonAP(_ap_wu, wind=WIND, dt=_DT)

    target_alt = float(-pos0[2])
    _ap_wu.receive_command(TensionCommand(
        tension_setpoint_n=IC_TARGET_TENSION_N, tension_measured_n=runner.tension_now,
        alt_m=target_alt, phase="reel-out",
    ), _DT_CMD)
    # Ground-side tension-regulating winch: adjusts rest_length toward the
    # value that holds tension at IC_TARGET_TENSION_N.  Mirrors the real
    # winch controller that prevents tether slack during steady flight.
    # Without this the tether's spring mode is undamped and tension swings
    # 0 - 800 N during warmup, exciting the cyclic loop into divergence.
    rest_now    = float(rest_length)
    _WINCH_KP   = 0.01    # m/s per N of tension error
    _WINCH_VMAX = 1.0     # m/s rest-length change limit
    for step in range(WARMUP_STEPS):
        if step % _PLANNER_EVERY == 0:
            _ap_wu.receive_command(TensionCommand(
                tension_setpoint_n=IC_TARGET_TENSION_N, tension_measured_n=runner.tension_now,
                alt_m=target_alt, phase="reel-out",
            ), _DT_CMD)
        if step % _AP_EVERY == 0:
            ap_wu.tick(step * _DT, runner)
        # Winch tension regulator: pay out when tension is high, reel in
        # when low.  P-controller on rest_length saturated at ±_WINCH_VMAX.
        dT       = runner.tension_now - IC_TARGET_TENSION_N
        v_winch  = max(-_WINCH_VMAX, min(_WINCH_VMAX, _WINCH_KP * dT))
        rest_now += v_winch * _DT
        runner.step(_DT, ap_wu.col_rad, ap_wu.roll_sp, ap_wu.pitch_sp,
                    runner.omega_body, rest_length=rest_now)
    coll_settled = ap_wu.col_rad
    rest_length  = rest_now

    # Settled state
    s     = runner.hub_state
    pos_s = s["pos"].copy()
    vel_s = s["vel"].copy()
    R_s   = s["R"].copy()
    omega_spin_settled = runner.omega_spin

    # Force balance diagnostics at STACK_COLL from settled position.
    # NB: anchor at origin, so t_dir_s = pos_s/|pos_s| = anchor→hub direction.
    t_dir_s    = pos_s / np.linalg.norm(pos_s)
    t_aero_end = PhysicsRunner.T_AERO_OFFSET + WARMUP_STEPS * _DT
    diag_inputs = RotorInputs(
        collective_rad=STACK_COLL, tilt_lon=0.0, tilt_lat=0.0,
        R_hub=R_s, v_hub_world=vel_s, wind_world=WIND, t=t_aero_end,
    )
    # The runner's aero state already encodes the equilibrium inflow + omega;
    # using runner.aero.compute_forces with the live state preserves it.
    diag_state = runner._core._rotor_state  # noqa: SLF001 — diag-only, intentional
    f_stack, _ = runner.aero.compute_forces(diag_inputs, diag_state)
    F_aero     = f_stack.F_world
    f_teth, m_teth = runner.tether.compute(pos_s, vel_s, R_s)
    T_tether   = float(runner.tether._last_info.get("tension", 0.0))
    gravity    = np.array([0.0, 0.0, MASS * G])
    F_net      = F_aero + f_teth
    F_residual  = F_net + gravity
    F_res_along = float(np.dot(F_residual, t_dir_s))
    grav_along  = float(np.dot(gravity, t_dir_s))
    grav_perp   = gravity - grav_along * t_dir_s

    # orbit_bz: tilt settled tether direction to pre-cancel gravity_perp
    f_res_perp   = F_residual - F_res_along * t_dir_s
    orbit_bz_raw = t_dir_s - f_res_perp / max(T_tether, 1.0)
    orbit_bz     = orbit_bz_raw / np.linalg.norm(orbit_bz_raw)
    R0_orbit     = build_orb_frame(orbit_bz)

    # R0_kinematic: same disk normal as R0, body_x North-aligned.
    # Used by the kinematic phase so the GPS/RELPOSNED heading is consistent
    # with a stationary hub (pure spin around body_z, no tilt change).
    R0_kinematic = build_orb_frame(R_s[:, 2])

    elevation_deg = math.degrees(math.asin(max(-1.0, min(1.0, -t_dir_s[2]))))

    return {
        "pos0":          pos_s,
        "vel0":          vel_s,
        "R0":            R_s,
        "R0_orbit":      R0_orbit,
        "R0_kinematic":  R0_kinematic,
        "orbit_bz":      orbit_bz,
        "omega_spin":    omega_spin_settled,
        "rest_length":   rest_length,
        "coll_settled":  coll_settled,
        "T_tether":      T_tether,
        "F_aero":        F_aero,
        "f_teth":        f_teth,
        "M_aero":        f_stack.M_orbital,
        "m_teth":        m_teth,
        "F_residual":    F_residual,
        "F_res_along":   F_res_along,
        "grav_perp":     grav_perp,
        "elevation_deg": elevation_deg,
    }


# ── IC serialiser ──────────────────────────────────────────────────────────────

def _save_ic(path: Path, ic: dict) -> None:
    """Write steady_state_starting.json consumed by all stack/simtests via config.py."""
    F_aero  = ic["F_aero"]
    f_teth  = ic["f_teth"]
    M_aero  = ic["M_aero"]
    m_teth  = ic["m_teth"]
    F_res   = ic["F_residual"]
    F_net   = F_aero + f_teth

    out = {
        "pos":           ic["pos0"].tolist(),
        "vel":           ic["vel0"].tolist(),
        "R0":            ic["R0"].tolist(),
        "R0_kinematic":  ic["R0_kinematic"].tolist(),
        "R0_orbit":      ic["R0_orbit"].tolist(),
        "orbit_bz":      ic["orbit_bz"].tolist(),
        "omega_spin":    float(ic["omega_spin"]),
        "rest_length":   float(ic["rest_length"]),
        # coll_eq_rad: collective at which TensionPI settled (≈ IC_TARGET_TENSION_N).
        # IC is settled at this collective so TensionPI warm-starts at equilibrium.
        "coll_eq_rad":   float(ic["coll_settled"]),
        "tension_eq_n":  float(ic["T_tether"]),
        "stack_coll_eq": float(ic["coll_settled"]),
        "home_z_ned":    0.0,
        "eq_physics": {
            "mass_kg":        float(MASS),
            "wind_ned":       WIND.tolist(),
            "collective_rad": float(STACK_COLL),
            "tilt_lon":       0.0,
            "tilt_lat":       0.0,
            "omega_spin":     float(ic["omega_spin"]),
            "note": (
                "Forces evaluated at STACK_COLL from warmup-settled position. "
                "Perpendicular residual (gravity_perp ~48 N) balanced by Lua cyclic in flight."
            ),
            "aero_fx":        float(F_aero[0]),
            "aero_fy":        float(F_aero[1]),
            "aero_fz":        float(F_aero[2]),
            "tether_fx":      float(f_teth[0]),
            "tether_fy":      float(f_teth[1]),
            "tether_fz":      float(f_teth[2]),
            "F_net_x":        float(F_net[0]),
            "F_net_y":        float(F_net[1]),
            "F_net_z":        float(F_net[2]),
            "F_residual_x":   float(F_res[0]),
            "F_residual_y":   float(F_res[1]),
            "F_residual_z":   float(F_res[2]),
            "F_residual_mag": float(np.linalg.norm(F_res)),
            "F_res_along":    float(ic["F_res_along"]),
            "grav_perp_x":    float(ic["grav_perp"][0]),
            "grav_perp_y":    float(ic["grav_perp"][1]),
            "grav_perp_z":    float(ic["grav_perp"][2]),
            "grav_perp_mag":  float(np.linalg.norm(ic["grav_perp"])),
            "aero_mx":        float(M_aero[0]),
            "aero_my":        float(M_aero[1]),
            "aero_mz":        float(M_aero[2]),
            "tether_mx":      float(m_teth[0]),
            "tether_my":      float(m_teth[1]),
            "tether_mz":      float(m_teth[2]),
        },
    }
    path.write_text(json.dumps(out, indent=2))


# ── Tests ──────────────────────────────────────────────────────────────────────

def test_create_ic(simtest_log):
    """
    Run 60 s warmup physics (TensionPI at 200 N, orbit-tracking) from the design
    orientation.  Save the settled state to steady_state_starting.json.

    The settled state encodes real velocity, real R0 (yaw consistent with free-
    flight heading), and tether tension at equilibrium — a valid starting point
    for both simtests and stack tests.
    """
    ic = _compute_ic()

    pos_s         = ic["pos0"]
    vel_s         = ic["vel0"]
    elevation_deg = ic["elevation_deg"]
    T_tether      = ic["T_tether"]
    F_res_along   = ic["F_res_along"]
    grav_perp_mag = float(np.linalg.norm(ic["grav_perp"]))
    tlen          = float(np.linalg.norm(pos_s))
    speed         = float(np.linalg.norm(vel_s))
    coll_settled  = ic["coll_settled"]

    simtest_log.write(
        [
            f"IC_TARGET_TENSION = {IC_TARGET_TENSION_N:.1f} N",
            f"coll_settled   = {coll_settled:.4f} rad  ({math.degrees(coll_settled):.2f} deg)",
            f"pos (settled)  = {pos_s.tolist()}",
            f"vel (settled)  = {vel_s.tolist()}",
            f"|vel|          = {speed:.4f} m/s",
            f"elevation      = {elevation_deg:.2f} deg",
            f"tether length  = {tlen:.4f} m",
            f"omega_spin     = {ic['omega_spin']:.4f} rad/s",
            f"rest_length    = {ic['rest_length']:.6f} m",
            f"T_tether       = {T_tether:.2f} N",
            f"F_aero         = {ic['F_aero'].tolist()}",
            f"F_tether       = {ic['f_teth'].tolist()}",
            f"F_residual     = {ic['F_residual'].tolist()}",
            f"|F_residual|   = {np.linalg.norm(ic['F_residual']):.4f} N",
            f"F_res_along    = {F_res_along:.4f} N  (informational)",
            f"grav_perp_mag  = {grav_perp_mag:.2f} N  (expected ~48 N; balanced by Lua cyclic)",
            f"orbit_bz       = {ic['orbit_bz'].tolist()}",
        ],
        f"elev={elevation_deg:.1f} deg  |vel|={speed:.3f} m/s  T={T_tether:.0f} N  "
        f"coll={math.degrees(coll_settled):.1f} deg",
    )

    # ── IC quality assertions ─────────────────────────────────────────────────
    assert IC_TENSION_MIN_N <= T_tether <= IC_TENSION_MAX_N, (
        f"IC tension {T_tether:.1f} N outside [{IC_TENSION_MIN_N}, {IC_TENSION_MAX_N}] N — "
        f"warmup did not converge to a reasonable tension point"
    )
    assert speed <= IC_SPEED_MAX_MS, (
        f"IC hub speed {speed:.3f} m/s > {IC_SPEED_MAX_MS} m/s — hub not settled"
    )
    assert IC_ELEV_MIN_DEG <= elevation_deg <= IC_ELEV_MAX_DEG, (
        f"IC elevation {elevation_deg:.1f} deg outside [{IC_ELEV_MIN_DEG}, {IC_ELEV_MAX_DEG}] deg"
    )

    _save_ic(_JSON_PATH, ic)


def _run_steady(pos0: np.ndarray, vel0: np.ndarray, R0: np.ndarray,
                omega_spin: float, rest: float, tension_sp: float,
                stack_coll: float, label: str, csv_path: Path) -> dict:
    """
    Run _STEADY_STEPS of altitude-holding physics from the given IC.

    Collective: TensionPI at tension_sp.
    Cyclic:     body_z_eq at (target_elevation, current_azimuth) — stateless,
                no orbit reference needed.

    Returns orbit characteristic metrics dict.
    """
    target_alt = float(-pos0[2])

    ic = SimpleNamespace(
        pos=np.asarray(pos0, dtype=float),
        vel=np.asarray(vel0, dtype=float),
        R0=R0,
        rest_length=float(rest),
        coll_eq_rad=float(stack_coll),
        omega_spin=float(omega_spin),
    )
    runner = PhysicsRunner(_ROTOR, ic, WIND, col_min_rad=-0.28, col_max_rad=0.10)
    # Same tuned cyclic gains as test_create_ic warmup.
    from controller import HeliCyclicController as _Heli
    runner._acro = _Heli(
        _ROTOR, col_min_rad=-0.28, col_max_rad=0.10,
        P=0.67, I=0.15, D=0.02, IMAX=0.30,
        FLTT=40.0, FLTE=0.0, FLTD=40.0,
    )
    runner._acro._servo.reset(stack_coll)
    _ap    = TensionApController(
        ic_pos=pos0, mass_kg=MASS,
        slew_rate_rad_s=BODY_Z_SLEW_RATE_RAD_S,
        warm_coll_rad=stack_coll, tension_ic=tension_sp,
        kd_lat=50.0,
    )
    ap     = PythonAP(_ap, wind=WIND, dt=_DT)
    ap.tel_fn = lambda r, sr: {
        **ap.log_fields(),
        "body_z_eq": r.hub_state["R"][:, 2],
        "phase":     label,
    }
    _ap.receive_command(TensionCommand(
        tension_setpoint_n=tension_sp, tension_measured_n=runner.tension_now,
        alt_m=target_alt, phase="reel-out",
    ), _DT_CMD)
    tlen_arr    = np.zeros(_STEADY_STEPS)
    alt_arr     = np.zeros(_STEADY_STEPS)
    speed_arr   = np.zeros(_STEADY_STEPS)
    elev_arr    = np.zeros(_STEADY_STEPS)
    az_arr      = np.zeros(_STEADY_STEPS)
    tension_arr = np.zeros(_STEADY_STEPS)

    # Ground-side tension-regulating winch — mirrors the real winch
    # tension controller.  Keeps the tether under tension so the spring
    # mode stays bounded.  Same gains as in the IC warmup.
    rest_now    = float(rest)
    _WINCH_KP   = 0.01
    _WINCH_VMAX = 1.0

    for step in range(_STEADY_STEPS):
        hub   = runner.hub_state
        pos   = hub["pos"]
        tlen  = float(np.linalg.norm(pos))
        tlen_arr[step]    = tlen
        alt_arr[step]     = float(-pos[2])
        speed_arr[step]   = float(np.linalg.norm(hub["vel"]))
        tension_arr[step] = runner.tension_now
        elev_arr[step]    = float(math.degrees(math.asin(
            max(-1.0, min(1.0, -pos[2] / max(tlen, 0.1))))))
        az_arr[step]      = float(math.degrees(math.atan2(pos[1], pos[0])))

        if step % _PLANNER_EVERY == 0:
            _ap.receive_command(TensionCommand(
                tension_setpoint_n=tension_sp, tension_measured_n=runner.tension_now,
                alt_m=target_alt, phase="reel-out",
            ), _DT_CMD)
        if step % _AP_EVERY == 0:
            ap.tick(step * _DT, runner)
        dT       = runner.tension_now - tension_sp
        v_winch  = max(-_WINCH_VMAX, min(_WINCH_VMAX, _WINCH_KP * dT))
        rest_now += v_winch * _DT
        sr = runner.step(_DT, ap.col_rad, ap.roll_sp, ap.pitch_sp,
                         runner.omega_body, rest_length=rest_now)
        ap.log(runner, sr)

    ap.write_telemetry(csv_path)

    mean_tlen    = float(np.mean(tlen_arr))
    max_tlen_dev = float(np.max(np.abs(tlen_arr - mean_tlen)))
    return dict(
        mean_tlen    = mean_tlen,
        max_tlen_dev = max_tlen_dev,
        mean_alt     = float(np.mean(alt_arr)),
        alt_range    = float(np.max(alt_arr) - np.min(alt_arr)),
        min_alt      = float(np.min(alt_arr)),
        max_alt      = float(np.max(alt_arr)),
        mean_elev    = float(np.mean(elev_arr)),
        elev_range   = float(np.max(elev_arr) - np.min(elev_arr)),
        mean_speed   = float(np.mean(speed_arr)),
        max_speed    = float(np.max(speed_arr)),
        az_range     = float(np.max(az_arr) - np.min(az_arr)),
        tlen_arr     = tlen_arr,
        tension_arr  = tension_arr,
        min_tension  = float(np.min(tension_arr)),
        mean_tension = float(np.mean(tension_arr)),
        csv_path     = csv_path,
    )


def _print_steady(log, m: dict, label: str) -> None:
    lines = [
        f"tether length : mean={m['mean_tlen']:.3f} m   max_dev={m['max_tlen_dev']:.3f} m",
        f"altitude      : mean={m['mean_alt']:.2f} m   range={m['alt_range']:.2f} m  (min={m['min_alt']:.2f}  max={m['max_alt']:.2f})",
        f"elevation     : mean={m['mean_elev']:.2f} deg  range={m['elev_range']:.2f} deg",
        f"azimuth range : {m['az_range']:.2f} deg",
        f"speed         : mean={m['mean_speed']:.3f} m/s  max={m['max_speed']:.3f} m/s",
        f"tension       : mean={m['mean_tension']:.1f} N   min={m['min_tension']:.1f} N",
    ]
    log.write(lines, f"[{label}] tlen_dev={m['max_tlen_dev']:.3f} m  alt_range={m['alt_range']:.2f} m  tension_min={m['min_tension']:.0f} N")
    print()
    for l in lines:
        print(f"  [{label}] {l}")
    print(f"  [{label}] telemetry -> {m['csv_path']}")


def test_ic_steady_flight(simtest_log):
    """
    Load steady_state_starting.json and run 30 s altitude-holding physics from
    the warmup-settled R0.  Verifies the serialised IC is physically stable.
    """
    if not _JSON_PATH.exists():
        pytest.skip("steady_state_starting.json not found — run test_create_ic first")

    d   = json.loads(_JSON_PATH.read_text())
    pos0       = np.array(d["pos"],  dtype=float)
    vel0       = np.array(d["vel"],  dtype=float)
    R0         = np.array(d["R0"],   dtype=float).reshape(3, 3)
    omega_spin = float(d["omega_spin"])
    rest       = float(d["rest_length"])
    tension_sp = float(d.get("tension_eq_n", 435.0))
    stack_coll = float(d["stack_coll_eq"])

    m = _run_steady(pos0, vel0, R0, omega_spin, rest, tension_sp, stack_coll,
                    label="ic_steady_flight",
                    csv_path=simtest_log.log_dir / "telemetry.csv")
    _print_steady(simtest_log, m, "ic_steady_flight")

    slack_steps = int(np.sum(m["tension_arr"] < 1.0))
    assert np.all(np.isfinite(m["tlen_arr"])), "NaN/inf in tether length"
    assert m["max_tlen_dev"] < _DRIFT_BOUND, (
        f"Tether length deviated {m['max_tlen_dev']:.2f} m > {_DRIFT_BOUND} m"
    )
    assert slack_steps == 0, (
        f"Tether slack at {slack_steps} steps (tension < 1 N) -- "
        f"IC has insufficient tether tension or wrong rest_length"
    )
    assert m["min_tension"] > 10.0, (
        f"Min tension {m['min_tension']:.1f} N -- IC too close to slack"
    )


def test_ic_r0_kinematic(simtest_log):
    """
    Verify that starting from R0_kinematic (same disk normal as the IC, but
    body_x North-aligned for GPS/RELPOSNED lock) and zero initial velocity,
    steady altitude-holding flight is maintained.

    The kinematic phase holds the hub stationary so ArduPilot can acquire GPS.
    At kinematic exit the hub has zero velocity and R0_kinematic orientation.
    This test checks that the altitude-holding controller recovers from that
    state without the hub sinking or diverging.
    """
    if not _JSON_PATH.exists():
        pytest.skip("steady_state_starting.json not found — run test_create_ic first")

    from simtest_ic import load_ic
    ic = load_ic()

    pos0       = ic.pos
    R0         = ic.R0
    R0_kinematic = ic.R0_kinematic   # shared: same body_z, body_x North-aligned
    omega_spin = ic.omega_spin
    rest       = ic.rest_length
    tension_sp = float(json.loads(_JSON_PATH.read_text()).get("tension_eq_n", 435.0))
    stack_coll = ic.stack_coll_eq

    spin_deg = float(math.degrees(math.acos(max(-1.0, min(1.0,
        float(np.dot(R0[:, 0], R0_kinematic[:, 0])))))))

    print(f"\n  [ic_r0_kinematic] spin adjustment = {spin_deg:.1f} deg around body_z")

    # Zero velocity — hub exits kinematic phase stationary
    vel0 = np.zeros(3)

    m = _run_steady(pos0, vel0, R0_kinematic, omega_spin, rest, tension_sp, stack_coll,
                    label="ic_r0_kinematic",
                    csv_path=simtest_log.log_dir / "telemetry_kinematic.csv")
    _print_steady(simtest_log, m, "ic_r0_kinematic")

    assert np.all(np.isfinite(m["tlen_arr"])), "NaN/inf in tether length"
    assert m["max_tlen_dev"] < _DRIFT_BOUND, (
        f"Tether length deviated {m['max_tlen_dev']:.2f} m > {_DRIFT_BOUND} m"
    )
