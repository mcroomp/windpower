"""
test_generate_ic.py — Create and verify steady-state initial conditions.

Two tests:
  test_create_ic        — warmup physics to find settled state; saves steady_state_starting.json
  test_ic_steady_flight — loads the saved JSON; runs 10 s more physics; asserts hub stays steady
                          (serialize + deserialize round-trip)

Regenerate:
    simulation/.venv/Scripts/python.exe -m pytest simulation/tests/unit/test_generate_ic.py -s

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
from aero            import create_aero
import rotor_definition as _rd
from controller      import (ElevationHoldController, TensionPI,
                              AcroControllerSitl)
from frames          import build_orb_frame
from simtest_log     import SimtestLog
from simtest_runner  import PhysicsRunner
from telemetry_csv   import TelRow, write_csv

TetherModel = _mediator_module.TetherModel
_log = SimtestLog(__file__)

# ── Rotor / physics constants ──────────────────────────────────────────────────
_ROTOR         = _rd.default()
_DYN_KW        = _ROTOR.dynamics_kwargs()
MASS           = _DYN_KW["mass"]
G              = 9.81

WIND = np.array([0.0, 10.0, 0.0])   # NED: 10 m/s East
WIND.flags.writeable = False

# ── Collectives ────────────────────────────────────────────────────────────────
STACK_COLL = TensionPI.COLL_MIN_RAD + 0.10  # -0.18 rad — col_cruise used by Lua at kinematic exit

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
    t_dir = _BODY_Z_DESIGN / np.linalg.norm(_BODY_Z_DESIGN)
    R0    = build_orb_frame(t_dir)
    pos0  = L_TETHER * t_dir

    # Initial omega_spin estimate from spin ODE equilibrium at v_inplane ~10 m/s
    omega_spin = math.sqrt(_ROTOR.K_drive_Nms_m * 10.0 / _ROTOR.K_drag_Nms2_rad2)

    # Initial rest_length from aero thrust estimate at STACK_COLL
    _aero_est = create_aero(_ROTOR)
    f_est = _aero_est.compute_forces(STACK_COLL, 0.0, 0.0, R0, np.zeros(3),
                                      omega_spin, WIND, t=PhysicsRunner.T_AERO_OFFSET)
    T_est = max(float(np.dot(f_est.F_world, t_dir)), 10.0)
    k_eff = TetherModel.EA_N / L_TETHER
    rest_length = L_TETHER - max(T_est / k_eff, 0.001)

    # Warmup: PhysicsRunner + TensionPI targeting IC_TARGET_TENSION_N.
    # TensionPI adjusts collective so the hub settles where tension ≈ target.
    runner     = PhysicsRunner.for_warmup(_ROTOR, pos0, R0, rest_length,
                                           STACK_COLL, omega_spin, WIND)
    tension_pi = TensionPI(setpoint_n=IC_TARGET_TENSION_N, warm_coll_rad=STACK_COLL)
    _el_ctrl   = ElevationHoldController.from_pos(pos0, _ROTOR.body_z_slew_rate_rad_s, MASS)
    _acro      = AcroControllerSitl()
    coll_settled = STACK_COLL

    target_alt = float(-pos0[2])
    for _ in range(WARMUP_STEPS):
        hub          = runner.hub_state
        R            = np.asarray(hub["R"], dtype=float)
        omega_b      = R.T @ np.asarray(hub["omega"], dtype=float)
        coll_settled = tension_pi.update(runner.tension_now, _DT)
        rate_roll, rate_pitch = _el_ctrl.update(hub["pos"], R, target_alt, runner.tension_now, _DT)
        tilt_lon, tilt_lat    = _acro.update(rate_roll, rate_pitch, omega_b, _DT)
        runner.step_raw(_DT, coll_settled, tilt_lon, tilt_lat)

    # Settled state
    s     = runner.hub_state
    pos_s = s["pos"].copy()
    vel_s = s["vel"].copy()
    R_s   = s["R"].copy()
    omega_spin_settled = runner.omega_spin

    # Force balance diagnostics at STACK_COLL from settled position
    t_dir_s    = pos_s / np.linalg.norm(pos_s)
    t_aero_end = PhysicsRunner.T_AERO_OFFSET + WARMUP_STEPS * _DT
    f_stack    = runner.aero.compute_forces(STACK_COLL, 0.0, 0.0, R_s, vel_s,
                                             omega_spin_settled, WIND, t=t_aero_end)
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

def test_create_ic():
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

    _log.write(
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
    runner     = PhysicsRunner(_ROTOR, ic, WIND)
    tension_pi = TensionPI(setpoint_n=tension_sp, warm_coll_rad=stack_coll)
    el_ctrl    = ElevationHoldController.from_pos(pos0, _ROTOR.body_z_slew_rate_rad_s, MASS)
    acro       = AcroControllerSitl()

    tlen_arr  = np.zeros(_STEADY_STEPS)
    alt_arr   = np.zeros(_STEADY_STEPS)
    speed_arr = np.zeros(_STEADY_STEPS)
    elev_arr  = np.zeros(_STEADY_STEPS)
    az_arr    = np.zeros(_STEADY_STEPS)

    tel_every = max(1, int(0.05 / _DT))
    telemetry = []

    for step in range(_STEADY_STEPS):
        hub   = runner.hub_state
        pos   = hub["pos"]
        tlen  = float(np.linalg.norm(pos))
        tlen_arr[step]  = tlen
        alt_arr[step]   = float(-pos[2])
        speed_arr[step] = float(np.linalg.norm(hub["vel"]))
        elev_arr[step]  = float(math.degrees(math.asin(
            max(-1.0, min(1.0, -pos[2] / max(tlen, 0.1))))))
        az_arr[step]    = float(math.degrees(math.atan2(pos[1], pos[0])))

        R           = np.asarray(hub["R"], dtype=float)
        omega_b     = R.T @ np.asarray(hub["omega"], dtype=float)
        tension_now = runner.tension_now
        collective  = tension_pi.update(tension_now, _DT)
        rate_roll, rate_pitch = el_ctrl.update(pos, R, target_alt, tension_now, _DT)
        tilt_lon, tilt_lat    = acro.update(rate_roll, rate_pitch, omega_b, _DT)
        sr          = runner.step_raw(_DT, collective, tilt_lon, tilt_lat)

        if step % tel_every == 0:
            telemetry.append(TelRow.from_physics(
                runner, sr, collective, WIND,
                body_z_eq=R[:, 2], phase=label,
                tension_setpoint=tension_sp,
            ))

    write_csv(telemetry, csv_path)

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
        csv_path     = csv_path,
    )


def _print_steady(m: dict, label: str) -> None:
    lines = [
        f"tether length : mean={m['mean_tlen']:.3f} m   max_dev={m['max_tlen_dev']:.3f} m",
        f"altitude      : mean={m['mean_alt']:.2f} m   range={m['alt_range']:.2f} m  (min={m['min_alt']:.2f}  max={m['max_alt']:.2f})",
        f"elevation     : mean={m['mean_elev']:.2f} deg  range={m['elev_range']:.2f} deg",
        f"azimuth range : {m['az_range']:.2f} deg",
        f"speed         : mean={m['mean_speed']:.3f} m/s  max={m['max_speed']:.3f} m/s",
    ]
    _log.write(lines, f"[{label}] tlen_dev={m['max_tlen_dev']:.3f} m  alt_range={m['alt_range']:.2f} m  mean_alt={m['mean_alt']:.1f} m")
    print()
    for l in lines:
        print(f"  [{label}] {l}")
    print(f"  [{label}] telemetry -> {m['csv_path']}")


def test_ic_steady_flight():
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
                    csv_path=_log.log_dir / "telemetry.csv")
    _print_steady(m, "ic_steady_flight")

    assert np.all(np.isfinite(m["tlen_arr"])), "NaN/inf in tether length"
    assert m["max_tlen_dev"] < _DRIFT_BOUND, (
        f"Tether length deviated {m['max_tlen_dev']:.2f} m > {_DRIFT_BOUND} m"
    )


def test_ic_r0_kinematic():
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
                    csv_path=_log.log_dir / "telemetry_kinematic.csv")
    _print_steady(m, "ic_r0_kinematic")

    assert np.all(np.isfinite(m["tlen_arr"])), "NaN/inf in tether length"
    assert m["max_tlen_dev"] < _DRIFT_BOUND, (
        f"Tether length deviated {m['max_tlen_dev']:.2f} m > {_DRIFT_BOUND} m"
    )
