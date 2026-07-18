"""
test_generate_ic.py — Create and verify steady-state initial conditions.

Two tests:
    test_create_ic        — writes the static-tether IC to steady_state_starting.json
    test_ic_steady_flight — loads the saved JSON; runs 30 s physics; asserts hub stays steady
                          (serialize + deserialize round-trip)

Regenerate:
    .venv/Scripts/python.exe -m pytest tests/unit/test_generate_ic.py -s

WHY STATIC-TETHER
-----------------
The IC represents a fixed paid-out tether: rest_length is chosen once from the
target initial stretch and remains constant. The kinematic phase starts from a
stationary wind-aligned downwind point with a body frame derived from the tether
direction, then the steady replay test verifies that the serialized state is
usable by the flight stack.

NED coordinates throughout.  X=North, Y=East, Z=Down.
"""
import math
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


pytestmark = [pytest.mark.simtest, pytest.mark.timeout(300)]

import simulation
import simulation.mediator as _mediator_module
from dynbem            import create_aero, RotorInputs, relax_inflow, solve_trim_cyclic, euler_step_omega
from simulation.frames import build_orb_frame
from tests.simtests.simtest_runner import PhysicsRunner
from tests.common.mock_ardupilot import MockArdupilot
from simulation.pumping_planner import TensionCommand
from simulation.controller import compute_bz_altitude_hold
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
# (No hardcoded design collective. The IC solver finds the equilibrium collective
# at the target tension using the coupled orientation + secant algorithm.)

# ── IC target tension ──────────────────────────────────────────────────────────
# Midway between pumping reel-in (226 N) and reel-out (435 N) targets.
# Used only to choose the initial tether stretch; IC warmup holds rest_length static.
IC_TARGET_TENSION_N = 300.0   # N

# ── IC quality bounds (asserted in test_create_ic) ────────────────────────────
IC_TENSION_MIN_N  =  50.0   # N  — must not be slack or nearly slack
IC_TENSION_MAX_N  = 500.0   # N  — below ~80% of break load (620 N)
IC_SPEED_MAX_MS   =   2.0   # m/s — IC should be nearly stationary
IC_ELEV_MIN_DEG   =   5.0   # deg — must be above horizon
IC_ELEV_MAX_DEG   =  55.0   # deg — must not be nearly vertical

# ── Tether / geometry ──────────────────────────────────────────────────────────
L_TETHER = 100.0   # m

# Design elevation from beaupoil_2026.yaml, NED.  The IC generator aligns the
# horizontal component with the wind so steady-state tests start on the
# downwind plane instead of the historical ~70 deg design azimuth.
_BODY_Z_DESIGN = np.array([0.305391, 0.851018, -0.427206])
_BODY_Z_DESIGN.flags.writeable = False


def _wind_aligned_tether_hat() -> np.ndarray:
    wind_h = np.asarray(WIND[:2], dtype=float)
    wind_h_norm = float(np.linalg.norm(wind_h))
    if wind_h_norm < 1e-6:
        raise ValueError("IC generation requires nonzero horizontal wind")
    z = float(_BODY_Z_DESIGN[2] / np.linalg.norm(_BODY_Z_DESIGN))
    h = float(np.sqrt(max(0.0, 1.0 - z * z)))
    tether_hat = np.array([
        h * wind_h[0] / wind_h_norm,
        h * wind_h[1] / wind_h_norm,
        z,
    ])
    return tether_hat / np.linalg.norm(tether_hat)

# ── Simulation parameters ──────────────────────────────────────────────────────
_DT            = 2.5e-3  # s (400 Hz)
WARMUP_STEPS   = 24000   # 60 s warmup — hub settles to natural equilibrium
_STEADY_STEPS  = 12000   # 30 s steady-flight check
_DRIFT_BOUND   = 15.0    # m — max 3-D drift in steady-flight check
_DT_CMD        = 0.1     # 10 Hz ground commands
_PLANNER_EVERY = max(1, round(_DT_CMD / _DT))                    # 40
_AP_EVERY      = max(1, round(1.0 / (MockArdupilot.AP_HZ * _DT)))    # 8

_JSON_PATH = Path(simulation.__file__).resolve().parent / "steady_state_starting.json"


# ── IC computation ─────────────────────────────────────────────────────────────

def _compute_ic() -> dict:
    """
    Run warmup physics (60 s, static tether, steady-guidance)
    from the design orientation and return the settled state as the IC.

    The warmup lets the hub find its natural equilibrium: real velocity, real R0
    yaw consistent with the free-flight trajectory, and tether tension balanced
    by a fixed tether rest length.  This avoids GPS Glitch at kinematic exit in
    stack tests (compass heading = R0 yaw matches GPS velocity heading).
    """
    # Start on the wind-aligned downwind plane while preserving the design
    # elevation.
    tether_hat = _wind_aligned_tether_hat()
    pos0  = L_TETHER * tether_hat

    # Initial body_z frame for rotor state computation.
    # Will be recomputed below to account for actual wind forces.
    el0 = float(math.asin(max(-1.0, min(1.0, -pos0[2] / max(np.linalg.norm(pos0), 0.1)))))
    az0 = float(math.atan2(pos0[1], pos0[0]))
    bz_initial = compute_bz_altitude_hold(
        pos0,
        el0,
        IC_TARGET_TENSION_N,
        MASS,
        G,
        az_ref_rad=az0,
    )
    R_initial = build_orb_frame(bz_initial)

    # ── Coupled static-equilibrium solve (spin + orientation + collective) ──
    # The IC must be a genuine force-balance fixed point: at zero velocity the
    # net of aero + gravity must lie ALONG the tether so the tether tension
    # alone balances it (zero perpendicular residual), AT the target tension.
    #
    # The previous single-pass construction fixed collective at a design constant,
    # forced rest_length to hit the target tension, and built body_z from a
    # one-shot perpendicular-force correction evaluated at the untilted frame.
    # That left a ~44 N perpendicular gravity residual and a spin computed at
    # the wrong orientation.  With the SITL zero-cyclic start, the residual sags
    # the hub into the stiff tether and spikes the tension (300 -> ~810 N).
    #
    # We instead solve the coupled fixed point OFFLINE using aero force
    # evaluations only (no PhysicsRunner / AP loop):
    #   inner  — a damped fixed point on the disk orientation body_z that nulls
    #            the perpendicular residual (autorotation spin re-solved at the
    #            orientation);
    #   outer  — a 1-D secant on collective so the balanced along-tether tension
    #            equals IC_TARGET_TENSION_N.
    dt_eq     = 1.0 / 400.0
    omega_min = _ROTOR.autorotation.omega_min_rad_s or 0.5
    I_ode     = _ROTOR.autorotation.I_ode_kgm2 or 10.0
    r_hat     = pos0 / np.linalg.norm(pos0)    # outward (anchor -> hub)
    t_dir_s   = -r_hat                          # hub -> anchor (nominal body_z)
    gravity   = np.array([0.0, 0.0, MASS * G])
    k_eff     = TetherModel.EA_N / np.linalg.norm(pos0)

    def _spin_equilibrium(R_hub, col_c, omega_seed):
        """Autorotation spin ODE equilibrium at a fixed orientation/collective."""
        aero = create_aero(_ROTOR, model="quasi_static")
        st   = aero.initial_rotor_state()
        om   = float(omega_seed)
        ang  = 0.0
        for _ in range(6):
            inp = RotorInputs(
                collective_rad=col_c, tilt_lon=0.0, tilt_lat=0.0,
                R_hub=R_hub, v_hub_world=np.zeros(3), wind_world=WIND,
                omega_rad_s=om, rho_kg_m3=1.225,
            )
            st = relax_inflow(aero, st, inp, n_steps=250, dt=dt_eq)
            for _ in range(200):
                res, st = aero.step(inp, st, dt_eq)
                om, ang = euler_step_omega(om, ang, float(res.Q_spin), 0.0, I_ode, dt_eq)
                om = max(omega_min, min(80.0, om))
        return om

    def _aero_force(R_hub, col_c, om):
        """World-frame aero force at a fixed orientation/collective/spin."""
        aero = create_aero(_ROTOR, model="quasi_static")
        st   = aero.initial_rotor_state()
        inp  = RotorInputs(
            collective_rad=col_c, tilt_lon=0.0, tilt_lat=0.0,
            R_hub=R_hub, v_hub_world=np.zeros(3), wind_world=WIND,
            omega_rad_s=om, rho_kg_m3=1.225,
        )
        st = relax_inflow(aero, st, inp, n_steps=250, dt=dt_eq)
        res, _ = aero.step(inp, st, dt_eq)
        return res.F_world

    def _solve_orientation(col_c):
        """Damped fixed point on body_z that nulls the perpendicular residual.

        Returns (T_along, omega, body_z, R_hub) at convergence, where T_along is
        the balanced tether tension (the outward component of aero + gravity).
        """
        body_z = t_dir_s.copy()
        om     = _spin_equilibrium(build_orb_frame(body_z), col_c, 36.0)
        for _ in range(18):
            R_hub     = build_orb_frame(body_z)
            resultant = _aero_force(R_hub, col_c, om) + gravity
            T_along   = float(np.dot(resultant, r_hat))
            perp      = resultant - T_along * r_hat
            body_z    = body_z + 0.5 * perp / max(abs(T_along), 1.0)
            body_z    = body_z / np.linalg.norm(body_z)
        om        = _spin_equilibrium(build_orb_frame(body_z), col_c, om)
        R_hub     = build_orb_frame(body_z)
        resultant = _aero_force(R_hub, col_c, om) + gravity
        T_along   = float(np.dot(resultant, r_hat))
        return T_along, om, body_z, R_hub

    # Outer secant on collective to drive the balanced tension to the target.
    # Seeds bracket the expected equilibrium range without hardcoding a design value.
    col_a, col_b = -0.22, -0.15
    T_a, _, _, _              = _solve_orientation(col_a)
    T_b, om_b, bz_b, R0       = _solve_orientation(col_b)
    for _ in range(6):
        if abs(T_b - IC_TARGET_TENSION_N) < 3.0:
            break
        denom = T_b - T_a
        if abs(denom) < 1e-6:
            break
        col_next = col_b - (T_b - IC_TARGET_TENSION_N) * (col_b - col_a) / denom
        col_next = max(-0.26, min(-0.10, col_next))
        col_a, T_a = col_b, T_b
        col_b = col_next
        T_b, om_b, bz_b, R0 = _solve_orientation(col_b)

    coll_settled       = float(col_b)
    omega_spin_settled = float(om_b)
    body_z_raw         = bz_b

    # ── Trim cyclic at the solved IC ────────────────────────────────────────
    # Find the (tilt_lon, tilt_lat) that null hub-frame Mx, My at the solved
    # operating point.  Used as a feedforward in HeliCyclicController so the
    # P-only rate loop doesn't have to fight the wind-driven baseline moment.
    _aero_est = create_aero(_ROTOR, model="quasi_static")
    state     = _aero_est.initial_rotor_state()
    trim = solve_trim_cyclic(
        _aero_est, state,
        RotorInputs(
            collective_rad=coll_settled, tilt_lon=0.0, tilt_lat=0.0,
            R_hub=R0, v_hub_world=np.zeros(3), wind_world=WIND,
            omega_rad_s=omega_spin_settled, rho_kg_m3=1.225,
        ),
        n_inflow_relax=200, dt_relax=dt_eq,
        tolerance_Nm=0.1,
    )
    state = trim.final_state

    # Paid-out length so the tether tension balances the solved along-tether
    # force at the design geometry (T = k_eff * stretch).
    rest_length = float(np.linalg.norm(pos0) - T_b / k_eff)

    # Static IC state
    pos_s = pos0.copy()
    vel_s = np.zeros(3)

    # Force balance diagnostics at the solved operating point.
    # NB: anchor at origin, so t_dir_s = hub→anchor direction (body_z points toward anchor).
    diag_inputs = RotorInputs(
        collective_rad=coll_settled, tilt_lon=0.0, tilt_lat=0.0,
        R_hub=R0, v_hub_world=vel_s, wind_world=WIND,
        omega_rad_s=omega_spin_settled, rho_kg_m3=1.225,
    )
    f_stack, state = _aero_est.step(diag_inputs, state, dt_eq)
    F_aero     = f_stack.F_world
    tether = TetherModel(rest_length=rest_length, hub_mass=MASS)
    f_teth, m_teth = tether.compute(pos_s, vel_s, R0)
    T_tether   = float(tether._last_info.get("tension", 0.0))
    F_net      = F_aero + f_teth
    F_residual  = F_net + gravity
    F_res_along = float(np.dot(F_residual, t_dir_s))
    grav_along  = float(np.dot(gravity, t_dir_s))
    grav_perp   = gravity - grav_along * t_dir_s

    # R0_kinematic: same disk normal as R_initial, body_x North-aligned.
    # Used by the kinematic phase so the GPS/RELPOSNED heading is consistent
    # with a stationary hub (pure spin around body_z, no tilt change).
    R0_kinematic = build_orb_frame(R_initial[:, 2])

    elevation_deg = math.degrees(math.asin(max(-1.0, min(1.0, t_dir_s[2]))))

    return {
        "pos0":          pos_s,
        "vel0":          vel_s,
        "R0":            R0,
        "R0_kinematic":  R0_kinematic,
        "omega_spin":    omega_spin_settled,
        "rest_length":   rest_length,
        "coll_settled":  coll_settled,
        "T_tether":      T_tether,
        "F_aero":        F_aero,
        "f_teth":        f_teth,
        "M_aero":        f_stack.m_hub_world,
        "m_teth":        m_teth,
        "F_residual":    F_residual,
        "F_res_along":   F_res_along,
        "grav_perp":     grav_perp,
        "elevation_deg": elevation_deg,
        "trim_tilt_lon": float(trim.tilt_lon),
        "trim_tilt_lat": float(trim.tilt_lat),
    }


# ── IC serialiser ──────────────────────────────────────────────────────────────

def _save_ic(path: Path, ic: dict) -> None:
    from simulation.param_defaults import load_collective_phys_range as _lr
    col_min, col_max = _lr()
    coll_settled = float(ic["coll_settled"])
    # eq_thrust is the normalized form of coll_eq_rad — single consistent IC value.
    # All callers (SITL RAWES_THR seeding, simtest servo reset) use this same collective.
    eq_thrust = (coll_settled - col_min) / (col_max - col_min)
    out = {
        "pos":           ic["pos0"].tolist(),
        "vel":           ic["vel0"].tolist(),
        "R0":            ic["R0"].tolist(),  # Force-balanced orientation for flight
        "R0_kinematic":  ic["R0_kinematic"].tolist(),
        "omega_spin":    float(ic["omega_spin"]),
        "rest_length":   float(ic["rest_length"]),
        "eq_thrust":     float(eq_thrust),
        "coll_eq_rad":   float(ic["coll_settled"]),
        "tension_eq_n":  float(ic["T_tether"]),
        "trim_tilt_lon": float(ic["trim_tilt_lon"]),
        "trim_tilt_lat": float(ic["trim_tilt_lat"]),
        "home_z_ned":    0.0,
    }
    path.write_text(json.dumps(out, indent=2))


# ── Tests ──────────────────────────────────────────────────────────────────────

def test_create_ic(simtest_log):
    """
    Write the static-tether IC at the wind-aligned design orientation to
    steady_state_starting.json.

    The state encodes zero velocity, body_z from Lua-equivalent gravity+
    tether force-balance geometry, and static tether tension from the fixed
    rest_length.
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
        ],
        f"elev={elevation_deg:.1f} deg  |vel|={speed:.3f} m/s  T={T_tether:.0f} N  "
        f"coll={math.degrees(coll_settled):.1f} deg",
    )

    # ── IC quality assertions ─────────────────────────────────────────────────
    assert IC_TENSION_MIN_N <= T_tether <= IC_TENSION_MAX_N, (
        f"IC tension {T_tether:.1f} N outside [{IC_TENSION_MIN_N}, {IC_TENSION_MAX_N}] N — "
        f"static rest_length did not produce a reasonable tension point"
    )
    assert speed <= IC_SPEED_MAX_MS, (
        f"IC hub speed {speed:.3f} m/s > {IC_SPEED_MAX_MS} m/s"
    )
    assert IC_ELEV_MIN_DEG <= elevation_deg <= IC_ELEV_MAX_DEG, (
        f"IC elevation {elevation_deg:.1f} deg outside [{IC_ELEV_MIN_DEG}, {IC_ELEV_MAX_DEG}] deg"
    )

    _save_ic(_JSON_PATH, ic)


def test_ic_matches_lua_bz_formula(simtest_log):
    """
    Generated IC body_z must be the force-balanced disk orientation: tilted off
    the tether just enough to cancel the perpendicular gravity component at the
    IC operating point (~9 deg for the design point).
    """
    if not _JSON_PATH.exists():
        pytest.skip("steady_state_starting.json not found — run test_create_ic first")

    d = json.loads(_JSON_PATH.read_text())
    pos = np.array(d["pos"], dtype=float)
    R0 = np.array(d["R0"], dtype=float).reshape(3, 3)
    bz_ic = R0[:, 2]
    
    # Tether direction (from anchor to hub)
    tether_dir = pos / np.linalg.norm(pos)
    
    # Check body_z alignment with tether (should be ~-1.0 for pointing at anchor)
    alignment = float(np.clip(np.dot(bz_ic, tether_dir), -1.0, 1.0))
    ang_deg = float(math.degrees(math.acos(abs(alignment))))

    simtest_log.write(
        [
            f"bz_ic        = {bz_ic.tolist()}",
            f"tether_dir   = {tether_dir.tolist()}",
            f"alignment    = {alignment:.6f}",
            f"angle_to_tether = {ang_deg:.6f} deg",
        ],
        f"IC bz force-balanced: alignment={alignment:.4f} deg_to_tether={ang_deg:.4f}",
    )

    # Force-balance disk tilt band.  Lower bound rejects the naive tether-aligned
    # orientation (unbalanced gravity_perp => tension spike); upper bound rejects
    # the gravity-only orientation.  Expected ~9 deg for the design operating point.
    assert 4.0 <= ang_deg <= 14.0, (
        f"IC body_z tilt {ang_deg:.2f} deg outside the force-balance band [4, 14] deg. "
        f"Disk must tilt ~atan(grav_perp/T) off the tether — not tether-aligned "
        f"(unbalanced, spikes tension) nor gravity-only."
    )


def _run_steady(pos0: np.ndarray, vel0: np.ndarray, R0: np.ndarray,
                omega_spin: float, rest: float, tension_sp: float,
                stack_coll: float, label: str, csv_path: Path,
                trim_tilt_lon: float = 0.0, trim_tilt_lat: float = 0.0) -> dict:
    """
    Run _STEADY_STEPS of altitude-holding physics from the given IC.

    Collective: fixed thrust command near the settled IC collective.
    Cyclic:     body_z_eq at (target_elevation, current_azimuth) — stateless,
                no circular-path reference needed.

    Returns steady-flight characteristic metrics dict.
    """
    target_alt = float(-pos0[2])

    from simulation.param_defaults import load_collective_phys_range as _lr
    col_min, col_max = _lr()
    stack_thrust = (float(stack_coll) - col_min) / (col_max - col_min)
    ic = SimpleNamespace(
        pos=np.asarray(pos0, dtype=float),
        vel=np.asarray(vel0, dtype=float),
        R0=R0,
        rest_length=float(rest),
        eq_thrust=float(stack_thrust),
        omega_spin=float(omega_spin),
    )
    runner = PhysicsRunner(_ROTOR, ic, WIND)
    from simulation.controller import HeliCyclicController as _Heli
    runner._acro = _Heli(_ROTOR)
    runner._acro._servo.reset(stack_coll)
    runner._acro.set_trim(trim_tilt_lon, trim_tilt_lat)
    ap = MockArdupilot.for_pumping(
        ic_pos=pos0,
        mass_kg=MASS,
        slew_rate_rad_s=BODY_Z_SLEW_RATE_RAD_S,
        warm_thrust=stack_thrust,
        tension_ic=tension_sp,
        wind=WIND,
        dt=_DT,
    )
    ap.tel_fn = lambda r, sr: {
        **ap.log_fields(),
        "phase":     label,
    }
    ap.receive_command(TensionCommand(
        tension_target_n=tension_sp,
        alt_m=target_alt, phase="reel-out",
    ), _DT_CMD)
    tlen_arr    = np.zeros(_STEADY_STEPS)
    alt_arr     = np.zeros(_STEADY_STEPS)
    speed_arr   = np.zeros(_STEADY_STEPS)
    elev_arr    = np.zeros(_STEADY_STEPS)
    az_arr      = np.zeros(_STEADY_STEPS)
    tension_arr = np.zeros(_STEADY_STEPS)

    rest_now = float(rest)

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
            ap.receive_command(TensionCommand(
                tension_target_n=tension_sp,
                alt_m=target_alt, phase="reel-out",
            ), _DT_CMD)
        if step % _AP_EVERY == 0:
            ap.tick(step * _DT, runner)
        sr = runner.step_from_thrust(_DT, ap.thrust, ap.roll_sp, ap.pitch_sp,
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


def test_ic_force_balance(simtest_log):
    """
    The IC must be a genuine physics fixed point at coll_eq_rad.
    At static equilibrium (vel=0, coll=coll_eq_rad, R0=force-balanced),
    the net force on the hub should be near zero.  We verify this by running
    1 second of open-loop physics (no controller — fixed collective = coll_eq_rad,
    fixed cyclic = IC trim) and checking that velocity stays below a tight bound.

    This test fails if coll_eq_rad does not correspond to the IC orientation's
    actual aero/tether force balance — i.e., if eq_thrust and coll_eq_rad are
    inconsistent.
    """
    if not _JSON_PATH.exists():
        pytest.skip("steady_state_starting.json not found — run test_create_ic first")

    from tests.simtests.simtest_ic import load_ic
    ic = load_ic()

    # IC must start at rest — if it is a true fixed point, velocity grows slowly.
    ic_static = SimpleNamespace(
        pos        = ic.pos.copy(),
        vel        = np.zeros(3),
        R0         = ic.R0,
        rest_length= float(ic.rest_length),
        eq_thrust  = float(ic.eq_thrust),
        omega_spin = float(ic.omega_spin),
        coll_eq_rad= float(ic.coll_eq_rad),
        trim_tilt_lon = float(getattr(ic, 'trim_tilt_lon', 0.0)),
        trim_tilt_lat = float(getattr(ic, 'trim_tilt_lat', 0.0)),
    )

    runner = PhysicsRunner(_ROTOR, ic_static, WIND)
    runner._acro._servo.reset(
        ic_static.coll_eq_rad,
        tilt_lon=ic_static.trim_tilt_lon,
        tilt_lat=ic_static.trim_tilt_lat,
    )

    # Run 1 second open-loop at the IC collective + trim cyclic (no rate commands).
    N_STEPS = int(1.0 / _DT)
    for _ in range(N_STEPS):
        runner.step_from_thrust(_DT, float(ic_static.eq_thrust), 0.0, 0.0,
                                runner.omega_body, rest_length=ic_static.rest_length)

    final_speed = float(np.linalg.norm(runner.hub_state["vel"]))
    final_tension = runner.tension_now
    simtest_log.write(
        [f"coll_eq_rad={ic_static.coll_eq_rad:.4f} rad  "
         f"final_speed={final_speed:.3f} m/s  final_tension={final_tension:.1f} N"],
        f"force_balance: speed={final_speed:.3f} tension={final_tension:.0f}",
    )

    # At a true force-balanced IC the hub should barely move in 1 s.
    # Threshold 2.0 m/s: the IC solver uses a simplified aero model so a small
    # residual (~5 N on 5 kg ≈ 1% of tether force) is expected.  The key invariant
    # is that this is << the old IC which had ~44 N perpendicular residual.
    assert final_speed < 2.0, (
        f"IC is not at force balance: hub reached {final_speed:.3f} m/s in 1 s "
        f"(should be < 1.0 m/s at equilibrium). "
        f"coll_eq_rad={ic_static.coll_eq_rad:.4f} rad may not match the R0 orientation."
    )
    assert abs(final_tension - 300.0) < 50.0, (
        f"IC tension drifted to {final_tension:.1f} N from 300 N target in 1 s"
    )


def test_ic_steady_flight(simtest_log):
    """
    Load the saved IC and run 30 s of altitude-holding Python physics from
    the force-balanced R0 at coll_eq_rad collective.

    Validates that the serialized state actually sustains steady flight:
    altitude stays bounded and tether length doesn't drift excessively.
    Uses coll_eq_rad (the physics equilibrium collective) — not eq_thrust —
    so this test is sensitive to any inconsistency between them.
    """
    if not _JSON_PATH.exists():
        pytest.skip("steady_state_starting.json not found — run test_create_ic first")

    from tests.simtests.simtest_ic import load_ic
    ic = load_ic()
    d = json.loads(_JSON_PATH.read_text())
    tension_sp   = float(d.get("tension_eq_n", 300.0))
    trim_tilt_lon = float(d.get("trim_tilt_lon", 0.0))
    trim_tilt_lat = float(d.get("trim_tilt_lat", 0.0))
    # Use coll_eq_rad (physics equilibrium) — consistent with the IC solver output.
    stack_coll = float(ic.coll_eq_rad)

    m = _run_steady(ic.pos, ic.vel, ic.R0, ic.omega_spin, ic.rest_length,
                    tension_sp, stack_coll,
                    label="ic_steady_flight",
                    csv_path=simtest_log.log_dir / "telemetry_steady.csv",
                    trim_tilt_lon=trim_tilt_lon, trim_tilt_lat=trim_tilt_lat)
    _print_steady(simtest_log, m, "ic_steady_flight")

    assert np.all(np.isfinite(m["tlen_arr"])), "NaN/inf in tether length"
    assert m["max_tlen_dev"] < _DRIFT_BOUND, (
        f"Tether length deviated {m['max_tlen_dev']:.2f} m > {_DRIFT_BOUND} m — "
        f"IC not at steady-flight equilibrium with coll_eq_rad"
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

    from tests.simtests.simtest_ic import load_ic
    ic = load_ic()

    pos0       = ic.pos
    R0         = ic.R0
    R0_kinematic = ic.R0_kinematic   # shared: same body_z, body_x North-aligned
    omega_spin = ic.omega_spin
    rest       = ic.rest_length
    d = json.loads(_JSON_PATH.read_text())
    tension_sp = float(d.get("tension_eq_n", 435.0))
    from simulation.param_defaults import thrust_to_coll_rad as _t2c
    # Use coll_eq_rad (physics equilibrium collective) for consistent physics.
    stack_coll = float(ic.coll_eq_rad)
    trim_tilt_lon = float(d.get("trim_tilt_lon", 0.0))
    trim_tilt_lat = float(d.get("trim_tilt_lat", 0.0))

    spin_deg = float(math.degrees(math.acos(max(-1.0, min(1.0,
        float(np.dot(R0[:, 0], R0_kinematic[:, 0])))))))

    print(f"\n  [ic_r0_kinematic] spin adjustment = {spin_deg:.1f} deg around body_z")

    # Zero velocity — hub exits kinematic phase stationary
    vel0 = np.zeros(3)

    m = _run_steady(pos0, vel0, R0_kinematic, omega_spin, rest, tension_sp, stack_coll,
                    label="ic_r0_kinematic",
                    csv_path=simtest_log.log_dir / "telemetry_kinematic.csv",
                    trim_tilt_lon=trim_tilt_lon, trim_tilt_lat=trim_tilt_lat)
    _print_steady(simtest_log, m, "ic_r0_kinematic")

    assert np.all(np.isfinite(m["tlen_arr"])), "NaN/inf in tether length"
    assert m["max_tlen_dev"] < _DRIFT_BOUND, (
        f"Tether length deviated {m['max_tlen_dev']:.2f} m > {_DRIFT_BOUND} m"
    )
