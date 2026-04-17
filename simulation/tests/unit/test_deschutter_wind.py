"""
test_deschutter_wind.py — De Schutter pumping cycle with power-law wind shear.

De Schutter et al. (2018) use a power-law wind profile (alpha=1/7) rather than
a uniform wind field.  This test re-runs the standard De Schutter cycle with
that model and verifies the same pass criteria.

Wind model
----------
    v(z) = v_ref * (z / z_ref) ** (1/7)

    v_ref = 10 m/s at z_ref = 10 m  (standard AWE reference conditions)

    At rotor operating altitude (~50-80 m) this gives ~13-14 m/s, meaningfully
    higher than the 10 m/s ground reference.

Ground station anemometer
-------------------------
    The WindEstimator seed (what the planner knows at startup) is computed from
    the wind model at ANEMOMETER_HEIGHT_M = 3 m above the anchor.  The planner
    never has direct access to the simulated wind at rotor altitude.
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

pytestmark = [pytest.mark.simtest, pytest.mark.timeout(300)]

import rotor_definition as rd
from dynamics    import RigidBodyDynamics
from aero        import create_aero
from tether      import TetherModel
from controller  import OrbitTracker, col_min_for_altitude_rad, AcroController
from planner     import DeschutterPlanner, WindEstimator, quat_apply, quat_is_identity
from winch       import WinchController
from frames      import build_orb_frame
from simtest_log import SimtestLog, BadEventLog
from simtest_ic  import load_ic
from tel         import make_tel

_log   = SimtestLog(__file__)
_IC    = load_ic()
_ROTOR = rd.default()

DT            = 1.0 / 400.0
ANCHOR        = np.zeros(3)
POS0          = _IC.pos
VEL0          = _IC.vel
BODY_Z0       = _IC.body_z
OMEGA_SPIN0   = _IC.omega_spin
REST_LENGTH0  = _IC.rest_length
T_AERO_OFFSET = 45.0
I_SPIN_KGMS2  = 10.0
OMEGA_SPIN_MIN = 0.5
BREAK_LOAD_N  = 620.0

# De Schutter (2018) power-law wind: 10 m/s at 10 m, alpha=1/7, East direction (NED Y)
_WIND_SPEED_REF  = 10.0
_WIND_Z_REF      = 10.0
_WIND_ALPHA      = 1.0 / 7.0
_WIND_DIR_NED    = np.array([0.0, 1.0, 0.0])   # East = NED Y

# Ground station anemometer height.
# WindEstimator seed = wind_model evaluated here — what the planner knows at startup.
ANEMOMETER_HEIGHT_M = 3.0
ANEMOMETER_POS_NED  = np.array([0.0, 0.0, -ANEMOMETER_HEIGHT_M])

T_REEL_OUT   = 30.0
T_REEL_IN    = 30.0
V_REEL_OUT   =  0.4
V_REEL_IN    =  0.4
XI_REEL_IN_DEG = 80.0
COL_MIN_RAD    = -0.28
COL_MAX_RAD    =  0.10
TENSION_SAFETY_N = 496.0

T_TRANSITION = (
    math.radians(XI_REEL_IN_DEG - 30.0) / _ROTOR.body_z_slew_rate_rad_s + 1.5
)


def _wind(pos_ned: np.ndarray) -> np.ndarray:
    """Power-law wind at hub NED position.  v(z) = v_ref * (z / z_ref)^alpha."""
    z = max(-float(pos_ned[2]), 0.1)
    return _WIND_DIR_NED * _WIND_SPEED_REF * (z / _WIND_Z_REF) ** _WIND_ALPHA


def _run() -> dict:
    aero = create_aero(rd.default())
    # Compute reel-in altitude floor using the actual power-law wind speed at the
    # starting altitude.  The default (wind_m_s=10.0) is the ground-reference speed,
    # which overestimates the lift available at the hub's 7 m operating altitude
    # (power-law gives ~9.5 m/s there).  Using the correct speed gives a more
    # restrictive floor and prevents altitude loss during reel-in.
    _reel_in_wind_ms = float(np.linalg.norm(_wind(POS0)))
    col_min_reel_in_rad  = col_min_for_altitude_rad(
        aero, XI_REEL_IN_DEG, _ROTOR.mass_kg, wind_m_s=_reel_in_wind_ms)

    dyn    = RigidBodyDynamics(
        mass=5.0, I_body=[5.0, 5.0, 10.0], I_spin=0.0,
        pos0=POS0.tolist(), vel0=VEL0.tolist(),
        R0=build_orb_frame(BODY_Z0), omega0=[0.0, 0.0, 0.0], z_floor=-1.0,
    )
    tether = TetherModel(anchor_ned=ANCHOR, rest_length=REST_LENGTH0,
                         axle_attachment_length=_ROTOR.axle_attachment_length_m)

    estimator  = WindEstimator(seed_wind_ned=_wind(ANEMOMETER_POS_NED))
    trajectory = DeschutterPlanner(
        t_reel_out          = T_REEL_OUT,
        t_reel_in           = T_REEL_IN,
        t_transition        = T_TRANSITION,
        v_reel_out          = V_REEL_OUT,
        v_reel_in           = V_REEL_IN,
        tension_out         = 250.0,   # higher than uniform-wind 200 N: power-law wind
                                         # is stronger at 15 m altitude (10.6 m/s vs 10 m/s),
                                         # so the orbit naturally generates ~250 N tension at
                                         # the equilibrium collective that maintains altitude.
        tension_in          =  55.0,
        wind_estimator      = estimator,
        col_min_rad              = COL_MIN_RAD,
        col_max_rad              = COL_MAX_RAD,
        xi_reel_in_deg           = XI_REEL_IN_DEG,
        col_min_reel_in_rad      = col_min_reel_in_rad,
    )
    winch = WinchController(rest_length=REST_LENGTH0, tension_safety_n=TENSION_SAFETY_N)
    orbit_tracker = OrbitTracker(BODY_Z0, POS0 / np.linalg.norm(POS0), _ROTOR.body_z_slew_rate_rad_s)
    acro = AcroController.from_rotor(_ROTOR, use_servo=True)

    from planner import Q_IDENTITY

    hub_state  = dyn.state
    omega_spin = OMEGA_SPIN0

    n_steps   = int((T_REEL_OUT + T_REEL_IN) / DT)
    rec_every = int(1.0 / DT)
    tel_every = max(1, int(0.05 / DT))   # 20 Hz telemetry for visualizer

    ts               = []
    tensions         = []
    tensions_out_acc = []
    tensions_in_acc  = []
    altitudes        = []
    collectives      = []
    tilts_from_wind  = []
    wind_actual_speeds   = []
    wind_actual_azimuths = []
    wind_est_speeds      = []
    wind_est_azimuths    = []
    wind_est_ready_log   = []
    telemetry        = []
    events      = BadEventLog()
    energy_out  = 0.0
    energy_in   = 0.0

    sw             = {"tilt_lon": 0.0, "tilt_lat": 0.0}
    cmd            = {"attitude_q": Q_IDENTITY.copy(), "thrust": 1.0,
                      "winch_speed_ms": 0.0, "phase": "reel-out"}
    tether.compute(hub_state["pos"], hub_state["vel"], hub_state["R"])
    tension_now    = tether._last_info.get("tension", 0.0)
    collective_rad = 0.0
    body_z_eq      = BODY_Z0.copy()

    for i in range(n_steps):
        t = i * DT

        wind_now = _wind(hub_state["pos"])

        cmd = trajectory.step({
            "pos_ned":         hub_state["pos"],
            "vel_ned":         hub_state["vel"],
            "omega_spin":      omega_spin,
            "body_z":          hub_state["R"][:, 2],
            "tension_n":       tension_now,
            "tether_length_m": winch.tether_length_m,
        }, DT)

        winch.step(cmd["winch_speed_ms"], tension_now, DT)
        tether.rest_length = winch.rest_length

        collective_rad = acro.slew_collective(
            COL_MIN_RAD + cmd["thrust"] * (COL_MAX_RAD - COL_MIN_RAD), DT
        )

        _aq = cmd["attitude_q"]
        _bz_target = (None if quat_is_identity(_aq)
                      else quat_apply(_aq, np.array([0.0, 0.0, -1.0])))
        body_z_eq = orbit_tracker.update(hub_state["pos"], DT, _bz_target)

        tilt_lon, tilt_lat = acro.update(hub_state, body_z_eq, DT)

        result = aero.compute_forces(
            collective_rad = collective_rad,
            tilt_lon       = tilt_lon,
            tilt_lat       = tilt_lat,
            R_hub          = hub_state["R"],
            v_hub_world    = hub_state["vel"],
            omega_rotor    = omega_spin,
            wind_world     = wind_now,
            t              = T_AERO_OFFSET + t,
        )

        tf, tm = tether.compute(hub_state["pos"], hub_state["vel"], hub_state["R"])
        tension_now = tether._last_info.get("tension", 0.0)

        if cmd["phase"] == "reel-out":
            energy_out += tension_now * V_REEL_OUT * DT
            tensions_out_acc.append(tension_now)
        else:
            energy_in  += tension_now * V_REEL_IN  * DT
            tensions_in_acc.append(tension_now)

        omega_spin = max(OMEGA_SPIN_MIN,
                         omega_spin + aero.last_Q_spin / I_SPIN_KGMS2 * DT)
        M_net = result.M_orbital + tm + (-50.0 * hub_state["omega"])
        hub_state = dyn.step(result.F_world + tf, M_net, DT, omega_spin=omega_spin)

        if events.check_floor(hub_state["pos"][2], t, str(cmd.get("phase", "reel-out")), 1.05):
            if events.count("floor_hit") > 200:
                break

        if i % rec_every == 0:
            hub_alt    = -hub_state["pos"][2]
            body_z_cur = hub_state["R"][:, 2]
            xi_deg = float(np.degrees(np.arccos(
                np.clip(np.dot(body_z_cur, _WIND_DIR_NED), -1.0, 1.0)
            )))
            ts.append(t)
            tensions.append(tension_now)
            altitudes.append(hub_alt)
            collectives.append(collective_rad)
            tilts_from_wind.append(xi_deg)

            actual_speed = float(np.linalg.norm(wind_now))
            wind_actual_speeds.append(actual_speed)
            wind_actual_azimuths.append(
                float(math.degrees(math.atan2(wind_now[1], wind_now[0]))))

            est_dir   = estimator.wind_dir_at(hub_alt)
            est_speed = estimator.wind_speed_at(hub_alt) or estimator.v_inplane_ms
            wind_est_speeds.append(float(est_speed) if est_speed is not None else 0.0)
            wind_est_azimuths.append(
                float(math.degrees(math.atan2(est_dir[1], est_dir[0]))))
            wind_est_ready_log.append(estimator.ready)

        if i % tel_every == 0:
            telemetry.append(make_tel(
                t, hub_state, omega_spin, tether, tension_now,
                collective_rad, sw["tilt_lon"], sw["tilt_lat"], wind_now,
                body_z_eq=body_z_eq,
                phase=str(cmd.get("phase", "")),
                tension_setpoint=float(cmd.get("tension_setpoint_n", 0.0)),
                wind_est_ned=(estimator.wind_dir_ned * float(est_speed or 0.0)).tolist(),
            ))

    t_split = int(T_REEL_OUT)
    return dict(
        ts               = ts,
        tensions         = tensions,
        tensions_out     = tensions[:t_split],
        tensions_in      = tensions[t_split:],
        tensions_out_all = tensions_out_acc,
        tensions_in_all  = tensions_in_acc,
        altitudes        = altitudes,
        collectives      = collectives,
        tilts_from_wind  = tilts_from_wind,
        wind_actual_speeds   = wind_actual_speeds,
        wind_actual_azimuths = wind_actual_azimuths,
        wind_est_speeds      = wind_est_speeds,
        wind_est_azimuths    = wind_est_azimuths,
        wind_est_ready       = wind_est_ready_log,
        shear_alpha      = estimator.shear_alpha,
        veer_rate        = estimator.veer_rate_deg_per_m,
        telemetry        = telemetry,
        events           = events,
        energy_out       = energy_out,
        energy_in        = energy_in,
        net_energy       = energy_out - energy_in,
    )


# ── Diagnostic log ─────────────────────────────────────────────────────────────

def _print_result(r: dict) -> None:
    skip = int(T_TRANSITION)
    out  = r["tensions_out"]
    inn  = r["tensions_in"][skip:] if r["tensions_in"][skip:] else r["tensions_in"]
    lines = [
        "De Schutter cycle -- power-law wind (alpha=1/7, 10 m/s at 10 m)",
        f"  {'t':>5}  {'phase':>8}  {'tension':>9}  {'xi':>6}  {'alt':>6}"
        f"  {'v_actual':>8}  {'v_est':>6}  {'ready':>5}",
    ]
    for t, ten, xi, alt, va, ve, rdy in zip(
            r["ts"], r["tensions"], r["tilts_from_wind"], r["altitudes"],
            r["wind_actual_speeds"], r["wind_est_speeds"], r["wind_est_ready"]):
        phase = "reel-out" if t <= T_REEL_OUT else "reel-in "
        lines.append(
            f"  {t:5.1f}  {phase:>8}  {ten:8.1f}N  {xi:5.1f}d  {alt:5.1f}m"
            f"  {va:7.2f}ms  {ve:5.2f}ms  {'Y' if rdy else 'N':>5}"
        )
    lines += [
        "",
        f"  Reel-out: mean={np.mean(out):.1f}N  max={max(out):.1f}N  energy={r['energy_out']:.1f}J",
        f"  Reel-in (steady): mean={np.mean(inn):.1f}N  max={max(inn):.1f}N  energy={r['energy_in']:.1f}J",
        f"  Net energy: {r['net_energy']:.1f}J  {r['events'].summary()}",
        f"  Wind actual: mean={np.mean(r['wind_actual_speeds']):.2f}m/s  "
        f"anemometer@3m={float(np.linalg.norm(_wind(ANEMOMETER_POS_NED))):.2f}m/s",
        f"  Wind est:    mean={np.mean(r['wind_est_speeds']):.2f}m/s  "
        f"converged={r['wind_est_ready'][-1] if r['wind_est_ready'] else False}",
        f"  Shear alpha fitted: {r['shear_alpha']}  (truth=0.143)",
        f"  Veer rate fitted:   {r['veer_rate']} deg/m  (truth=0.0)",
    ]
    _log.write(lines, f"net={r['net_energy']:.0f}J  "
               f"reel-out={np.mean(out):.0f}N  reel-in={np.mean(inn):.0f}N  "
               f"{r['events'].summary()}")


# ── Tests ──────────────────────────────────────────────────────────────────────

def test_deschutter_wind():
    """Power-law wind De Schutter cycle: no bad events, mechanism works, net energy positive."""
    r = _run()
    _print_result(r)
    failures = []
    if r["events"]:
        failures.append(r["events"].summary())
    skip = int(T_TRANSITION)
    mean_out = float(np.mean(r["tensions_out"])) if r["tensions_out"] else 0.0
    mean_in  = float(np.mean(r["tensions_in"][skip:])) if r["tensions_in"][skip:] else 0.0
    if mean_in >= mean_out:
        failures.append(f"De Schutter mechanism failed: reel-in {mean_in:.1f}N >= reel-out {mean_out:.1f}N")
    if r["net_energy"] <= 0:
        failures.append(f"net_energy={r['net_energy']:.1f}J <= 0")
    peak = max(r["tensions"])
    if peak >= 0.8 * BREAK_LOAD_N:
        failures.append(f"peak_tension={peak:.1f}N >= 80% break load ({0.8 * BREAK_LOAD_N:.1f}N)")
    assert not failures, "\n  ".join(failures)


# ── CLI: generate telemetry JSON for 3D visualizer ────────────────────────────

if __name__ == "__main__":
    import argparse as _ap
    _p = _ap.ArgumentParser(
        description="Run De Schutter cycle with power-law wind and save telemetry")
    _p.add_argument("--save-telemetry", metavar="PATH",
                    default="telemetry_deschutter_wind.csv",
                    help="Output CSV path (default: telemetry_deschutter_wind.csv)")
    _args = _p.parse_args()

    from telemetry_csv import TelRow as _TelRow, write_csv as _write_csv  # type: ignore

    r = _run()
    print(f"Power-law wind (alpha=1/7, {_WIND_SPEED_REF}m/s at {_WIND_Z_REF}m)")
    print(f"  Anemometer seed at {ANEMOMETER_HEIGHT_M}m: "
          f"{float(np.linalg.norm(_wind(ANEMOMETER_POS_NED))):.2f} m/s")
    print(f"  Reel-out energy: {r['energy_out']:.1f} J")
    print(f"  Reel-in energy:  {r['energy_in']:.1f} J")
    print(f"  Net:             {r['net_energy']:.1f} J")
    print(f"  Events:          {r['events'].summary()}")
    _write_csv([_TelRow.from_tel(d) for d in r["telemetry"]], _args.save_telemetry)
    print(f"\nRun visualizer:")
    print(f"  python simulation/viz3d/visualize_3d.py {_args.save_telemetry}")
