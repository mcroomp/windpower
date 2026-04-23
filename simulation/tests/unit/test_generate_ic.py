"""
test_generate_ic.py — Create and verify steady-state initial conditions.

Two tests:
  test_create_ic        — analytical IC at STACK_COLL; saves steady_state_starting.json
  test_ic_steady_flight — loads the saved JSON; runs 10 s physics; asserts hub stays steady
                          (serialize + deserialize round-trip)

Regenerate:
    simulation/.venv/Scripts/python.exe -m pytest simulation/tests/unit/test_generate_ic.py -s

WHY A DEDICATED FILE
--------------------
test_steady_flight.py generated the IC using a warmup at coll = -0.28 (TensionPI minimum),
then evaluated forces at coll = -0.18.  The IC position was the equilibrium for coll = -0.28,
but the stack test uses coll = -0.18.  At kinematic exit the 170 N thrust mismatch caused a
~188 N net force — hub crashed.

ANALYTICAL APPROACH
-------------------
  1. omega_spin: converged autorotation equilibrium at STACK_COLL.
  2. rest_length: chosen so the elastic tether tension exactly equals the along-tether
     force balance requirement at the design position:
         T_tether = F_aero_along_t + dot(gravity, tether_dir)
  3. pos0 = L_TETHER * tether_dir, vel = 0, R = build_orb_frame(tether_dir).
  4. orbit_bz: tether_dir tilted by -F_residual_perp/T to pre-cancel the ~48 N
     gravity_perp component so the orbit tracker starts with near-zero error.

FORCE BALANCE
-------------
At design position with the computed rest_length:
  - Along-tether: balanced by construction  (|F_along_t| < 1 N)
  - Perpendicular: gravity_perp ~48 N (expected; balanced by Lua cyclic in flight)

NED coordinates throughout.  X=North, Y=East, Z=Down.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

pytestmark = [pytest.mark.simtest, pytest.mark.timeout(120)]

import mediator as _mediator_module
from aero            import create_aero
import rotor_definition as _rd
from dynamics        import RigidBodyDynamics
from controller      import (compute_swashplate_from_state,
                              orbit_tracked_body_z_eq, TensionPI)
from swashplate      import SwashplateServoModel
from frames          import build_orb_frame
from simtest_log     import SimtestLog

TetherModel = _mediator_module.TetherModel
_log = SimtestLog(__file__)

# ── Rotor / physics constants ──────────────────────────────────────────────────
_ROTOR         = _rd.default()
_DYN_KW        = _ROTOR.dynamics_kwargs()
MASS           = _DYN_KW["mass"]
G              = 9.81
K_DRIVE_SPIN   = _ROTOR.K_drive_Nms_m
K_DRAG_SPIN    = _ROTOR.K_drag_Nms2_rad2
OMEGA_SPIN_MIN = _ROTOR.omega_min_rad_s
I_SPIN         = _ROTOR.I_ode_kgm2

WIND = np.array([0.0, 10.0, 0.0])   # NED: 10 m/s East
WIND.flags.writeable = False

# ── Stack test collective ──────────────────────────────────────────────────────
# col_cruise sent by rawes.lua during kinematic phase and at kinematic exit.
# Along-tether forces must be balanced at exactly this collective.
STACK_COLL = TensionPI.COLL_MIN_RAD + 0.10   # -0.18 rad

# ── Tether geometry ────────────────────────────────────────────────────────────
L_TETHER = 100.0   # m

# Design orientation (from beaupoil_2026.yaml, NED): ~25 deg elevation, ~70 deg E-of-N
_BODY_Z_DESIGN = np.array([0.305391, 0.851018, -0.427206])
_BODY_Z_DESIGN.flags.writeable = False

T_AERO_OFFSET = 45.0   # s — aero ramp already complete at this sim time

# Along-tether force tolerance
F_ALONG_TETHER_TOL = 2.0   # N

# Steady-flight check constants
_BASE_K_ANG = 50.0   # N·m·s/rad — angular damping (matches mediator default)
_DRIFT_BOUND = 15.0  # m — maximum 3-D drift from start over 10 s
_DT          = 2.5e-3  # s (400 Hz)
_STEPS       = 4000    # 10 s

_JSON_PATH = Path(__file__).resolve().parents[2] / "steady_state_starting.json"


# ── IC computation ─────────────────────────────────────────────────────────────

def _compute_ic() -> dict:
    """Analytical equilibrium at _BODY_Z_DESIGN with STACK_COLL collective."""
    aero   = create_aero(_ROTOR)
    t_dir  = _BODY_Z_DESIGN / np.linalg.norm(_BODY_Z_DESIGN)
    R0     = build_orb_frame(t_dir)
    pos0   = L_TETHER * t_dir

    # Step 1: converge omega_spin to autorotation equilibrium
    omega_spin = math.sqrt(K_DRIVE_SPIN * 10.0 / K_DRAG_SPIN)
    for _ in range(40):
        aero.compute_forces(STACK_COLL, 0.0, 0.0, R0, np.zeros(3),
                            omega_spin, WIND, t=T_AERO_OFFSET)
        omega_new = math.sqrt(max(1e-6, K_DRIVE_SPIN * float(aero.last_v_inplane) / K_DRAG_SPIN))
        if abs(omega_new - omega_spin) < 1e-5:
            omega_spin = omega_new
            break
        omega_spin = omega_new

    # Step 2: aero forces at design position
    f_result = aero.compute_forces(STACK_COLL, 0.0, 0.0, R0, np.zeros(3),
                                   omega_spin, WIND, t=T_AERO_OFFSET)
    F_aero = f_result.F_world

    # Step 3: rest_length so along-tether forces are balanced
    #   T_tether = F_aero_along_t + gravity_along_t
    gravity        = np.array([0.0, 0.0, MASS * G])
    F_aero_along   = float(np.dot(F_aero, t_dir))
    grav_along     = float(np.dot(gravity, t_dir))
    T_tether_eq    = F_aero_along + grav_along

    k_eff      = TetherModel.EA_N / L_TETHER
    extension  = max(T_tether_eq, 0.0) / k_eff
    rest_length = L_TETHER - max(extension, 0.001)

    # Step 4: verify force balance
    tether   = TetherModel(anchor_ned=np.zeros(3), rest_length=rest_length)
    f_teth, m_teth = tether.compute(pos0, np.zeros(3), R0)
    T_actual = float(tether._last_info.get("tension", 0.0))

    F_net      = F_aero + f_teth
    F_residual = F_net + gravity
    F_res_along = float(np.dot(F_residual, t_dir))
    grav_perp   = gravity - grav_along * t_dir

    elevation_deg = math.degrees(math.asin(-t_dir[2]))

    # Step 5: orbit_bz — tilt tether_dir to pre-cancel gravity_perp
    f_res_perp = F_residual - F_res_along * t_dir
    orbit_bz_raw = t_dir - f_res_perp / T_tether_eq
    orbit_bz     = orbit_bz_raw / np.linalg.norm(orbit_bz_raw)
    R0_orbit     = build_orb_frame(orbit_bz)

    return {
        "pos0":          pos0,
        "vel0":          np.zeros(3),
        "R0":            R0,
        "R0_orbit":      R0_orbit,
        "orbit_bz":      orbit_bz,
        "omega_spin":    omega_spin,
        "rest_length":   rest_length,
        "T_tether_eq":   T_tether_eq,
        "F_aero":        F_aero,
        "f_teth":        f_teth,
        "M_aero":        f_result.M_orbital,
        "m_teth":        m_teth,
        "T_actual":      T_actual,
        "F_residual":    F_residual,
        "F_res_along":   F_res_along,
        "grav_perp":     grav_perp,
        "elevation_deg": elevation_deg,
        "aero_obj":      aero,
    }


# ── IC serialiser ──────────────────────────────────────────────────────────────

def _save_ic(path: Path, ic: dict) -> None:
    """Write steady_state_starting.json consumed by all stack/simtests via config.py."""
    F_aero  = ic["F_aero"]
    f_teth  = ic["f_teth"]
    M_aero  = ic["M_aero"]
    m_teth  = ic["m_teth"]
    F_res   = ic["F_residual"]
    gravity = np.array([0.0, 0.0, MASS * G])
    F_net   = F_aero + f_teth

    out = {
        "pos":           ic["pos0"].tolist(),
        "vel":           ic["vel0"].tolist(),
        "R0":            ic["R0"].tolist(),
        "R0_orbit":      ic["R0_orbit"].tolist(),
        "orbit_bz":      ic["orbit_bz"].tolist(),
        "omega_spin":    float(ic["omega_spin"]),
        "rest_length":   float(ic["rest_length"]),
        # coll_eq_rad: equilibrium collective; equals stack_coll_eq since IC is
        # balanced analytically at this collective (used as TensionPI warm-start).
        "coll_eq_rad":   float(STACK_COLL),
        "tension_eq_n":  float(ic["T_tether_eq"]),
        "stack_coll_eq": float(STACK_COLL),
        "home_z_ned":    0.0,
        "eq_physics": {
            "mass_kg":        float(MASS),
            "wind_ned":       WIND.tolist(),
            "collective_rad": float(STACK_COLL),
            "tilt_lon":       0.0,
            "tilt_lat":       0.0,
            "omega_spin":     float(ic["omega_spin"]),
            "note": (
                "Along-tether forces balanced by construction (F_res_along < 1 N). "
                "Perpendicular residual is gravity_perp (~48 N) — balanced by Lua "
                "cyclic in flight, NOT by the tether."
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


# ── Physics loop for steady-flight check ───────────────────────────────────────

def _run_steady(steps: int = _STEPS) -> dict:
    """
    Load IC from JSON and run `steps` steps of free physics at STACK_COLL.
    Returns pos_arr [steps x 3] and the final drift scalar.
    """
    d          = json.loads(_JSON_PATH.read_text())
    pos0       = np.array(d["pos"],        dtype=float)
    vel0       = np.array(d["vel"],        dtype=float)
    R0         = np.array(d["R0"],         dtype=float).reshape(3, 3)
    omega_spin = float(d["omega_spin"])
    rest       = float(d["rest_length"])
    collective = float(d["stack_coll_eq"])
    # orbit_bz as the initial orbit-tracking target
    orbit_bz0  = np.array(d.get("orbit_bz", d["R0"][2]), dtype=float)
    # tether direction for orbit tracking
    tlen = np.linalg.norm(pos0)
    tdir0 = pos0 / tlen if tlen > 0.1 else R0[:, 2]

    aero   = create_aero(_ROTOR)
    tether = TetherModel(anchor_ned=np.zeros(3), rest_length=rest)
    servo  = SwashplateServoModel.from_rotor(_rd.default())
    dyn    = RigidBodyDynamics(
        **_DYN_KW,
        pos0=pos0.tolist(), vel0=vel0.tolist(), R0=R0, omega0=[0., 0., 0.],
    )

    pos_arr = np.zeros((steps, 3))

    for step in range(steps):
        state  = dyn.state
        pos    = state["pos"]
        pos_arr[step] = pos

        f_teth, m_teth = tether.compute(pos, state["vel"], state["R"])
        body_z_eq = orbit_tracked_body_z_eq(pos, tdir0, orbit_bz0)
        sw = compute_swashplate_from_state(state, np.zeros(3), body_z_eq=body_z_eq)
        tilt_lon, tilt_lat = servo.step(sw["tilt_lon"], sw["tilt_lat"], _DT)

        result = aero.compute_forces(
            collective_rad=collective,
            tilt_lon=tilt_lon, tilt_lat=tilt_lat,
            R_hub=state["R"], v_hub_world=state["vel"],
            omega_rotor=omega_spin, wind_world=WIND, t=T_AERO_OFFSET + step * _DT,
        )

        Q_spin     = K_DRIVE_SPIN * aero.last_v_inplane - K_DRAG_SPIN * omega_spin ** 2
        omega_spin = max(OMEGA_SPIN_MIN, omega_spin + Q_spin / I_SPIN * _DT)

        F_net = result.F_world + f_teth
        M_net = result.M_orbital + m_teth - _BASE_K_ANG * state["omega"]
        dyn.step(F_net, M_net, _DT)

    drift = float(np.linalg.norm(pos_arr[-1] - pos_arr[0]))
    return {"pos": pos_arr, "drift": drift}


# ── Tests ──────────────────────────────────────────────────────────────────────

def test_create_ic():
    """
    Compute the steady-state IC analytically at the design orientation and verify
    along-tether force balance at STACK_COLL.

    Saves steady_state_starting.json on pass and fail (useful for diagnostics).
    """
    ic = _compute_ic()

    F_res_along   = ic["F_res_along"]
    grav_perp_mag = float(np.linalg.norm(ic["grav_perp"]))
    elevation_deg = ic["elevation_deg"]
    T_actual      = ic["T_actual"]

    _log.write(
        [
            f"STACK_COLL     = {STACK_COLL:.4f} rad  ({math.degrees(STACK_COLL):.2f} deg)",
            f"pos0           = {ic['pos0'].tolist()}",
            f"elevation      = {elevation_deg:.2f} deg",
            f"|pos0|         = {np.linalg.norm(ic['pos0']):.4f} m",
            f"omega_spin     = {ic['omega_spin']:.4f} rad/s",
            f"rest_length    = {ic['rest_length']:.6f} m",
            f"T_tether_eq    = {ic['T_tether_eq']:.2f} N",
            f"T_actual       = {T_actual:.2f} N",
            f"F_aero         = {ic['F_aero'].tolist()}",
            f"F_tether       = {ic['f_teth'].tolist()}",
            f"F_residual     = {ic['F_residual'].tolist()}",
            f"|F_residual|   = {np.linalg.norm(ic['F_residual']):.4f} N",
            f"F_res_along    = {F_res_along:.4f} N  (target < {F_ALONG_TETHER_TOL})",
            f"grav_perp_mag  = {grav_perp_mag:.2f} N  (expected ~48 N; balanced by Lua cyclic)",
            f"orbit_bz       = {ic['orbit_bz'].tolist()}",
        ],
        f"F_res_along={F_res_along:.3f} N  grav_perp={grav_perp_mag:.1f} N  "
        f"elev={elevation_deg:.1f} deg  T={T_actual:.0f} N",
    )

    _save_ic(_JSON_PATH, ic)

    assert abs(F_res_along) < F_ALONG_TETHER_TOL, (
        f"|F_along_tether| = {abs(F_res_along):.4f} N > {F_ALONG_TETHER_TOL} N. "
        f"Along-tether force balance failed.  "
        f"rest={ic['rest_length']:.6f} m  T_actual={T_actual:.2f} N  "
        f"T_eq={ic['T_tether_eq']:.2f} N  F_residual={ic['F_residual'].tolist()}"
    )


def test_ic_steady_flight():
    """
    Load steady_state_starting.json and run 10 s of free physics with the orbit-
    tracking controller at STACK_COLL.  The hub must not drift more than _DRIFT_BOUND m.

    This is the serialize-then-deserialize round-trip: if test_create_ic produced a
    valid physical state, the hub should stay near its starting point without a warmup
    transient.  A large drift indicates a force imbalance in the saved IC.
    """
    if not _JSON_PATH.exists():
        pytest.skip("steady_state_starting.json not found — run test_create_ic first")

    result = _run_steady()
    drift  = result["drift"]

    _log.write(
        [f"drift_3d={drift:.3f} m  steps={_STEPS}  dt={_DT}  bound={_DRIFT_BOUND} m"],
        f"drift_3d={drift:.3f} m",
    )

    pos_arr = result["pos"]
    assert np.all(np.isfinite(pos_arr)), "NaN/inf in position during steady-flight check"
    assert drift < _DRIFT_BOUND, (
        f"Hub drifted {drift:.2f} m > {_DRIFT_BOUND} m over {_STEPS * _DT:.0f} s. "
        f"IC force balance likely wrong — re-run test_create_ic and inspect logs."
    )
