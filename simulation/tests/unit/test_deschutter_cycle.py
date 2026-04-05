"""
test_deschutter_cycle.py — De Schutter (2018) pumping cycle, no ArduPilot.

Implements the two-phase strategy from De Schutter et al. (2018) Fig. 4:

  Reel-out (power generation):
    Rotor tilt ξ = 30–50° from wind direction (body_z aligned with tether).
    High collective → high thrust along tether → high tension → high power.

  Reel-in (recovery):
    Rotor tilt ξ = 55° from wind direction — constrained from the original 90° to
    stay within BEM model validity (v_axial > 0) and cyclic linearity range.
    Reduced thrust along tether → lower tension → lower energy cost.
    Tension ratio ~0.6× reel-out (vs ~0.25× at ξ=90°) but model remains valid.

The key difference from a naive collective-only approach: tilt is the primary
lever for reducing reel-in tension, not collective.  By tilting body_z to
vertical, thrust no longer contributes to tether pull.  The tether tension
during reel-in is then only the force needed to overcome the gravity component
along the tether direction plus inertia — much lower than during reel-out.

Controller split:
  - Attitude   → compute_swashplate_from_state  (tilt_lon / tilt_lat)
                 body_z_eq transitions from tether-aligned to ξ=55° at reel-in
  - Collective → TensionController (same as test_pumping_cycle.py)
  - Winch      → tether.rest_length ±= v_reel × DT  each step

Pass criteria:
  1. Hub stays above z_floor throughout.
  2. Reel-in mean tension < reel-out mean tension  (De Schutter mechanism works).
  3. Net energy positive  (reel-out energy > reel-in energy).
  4. Peak tension stays below 80% of tether break load.
"""
import csv
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
from simtest_log import SimtestLog
from simtest_ic  import load_ic

_log   = SimtestLog(__file__)
_IC    = load_ic()
_ROTOR = rd.default()
_AERO  = create_aero(_ROTOR)

# ── Simulation constants ───────────────────────────────────────────────────────
DT            = 1.0 / 400.0
ANCHOR        = np.zeros(3)
POS0          = _IC.pos
VEL0          = _IC.vel
BODY_Z0       = _IC.body_z
OMEGA_SPIN0   = _IC.omega_spin
REST_LENGTH0  = _IC.rest_length

T_AERO_OFFSET = 45.0

I_SPIN_KGMS2   = 10.0
OMEGA_SPIN_MIN = 0.5

WIND          = np.array([0.0, 10.0, 0.0])  # NED: East wind = Y axis
BREAK_LOAD_N  = 620.0

# ── De Schutter cycle parameters ──────────────────────────────────────────────
T_REEL_OUT    = 30.0    # s
T_REEL_IN     = 30.0    # s
V_REEL_OUT    =  0.4    # m/s
V_REEL_IN     =  0.4    # m/s

# Reel-in tilt target and transition time
XI_REEL_IN_DEG  = 80.0   # ° from wind — aerodynamic equilibrium ~80° at this target
# Transition time: angle to cover / slew_rate + margin
import math as _math
_XI_START_DEG   = 30.0   # approximate tether elevation at reel-in start
BODY_Z_SLEW_RATE = _ROTOR.body_z_slew_rate_rad_s   # derived from gyroscopic limit
T_TRANSITION    = _math.radians(XI_REEL_IN_DEG - _XI_START_DEG) / BODY_Z_SLEW_RATE + 1.5

# Tension setpoints
DEFAULT_TENSION_OUT = 200.0   # N
DEFAULT_TENSION_IN  =  55.0   # N — above min achievable at xi=80° with col_min_reel_in

# Collective range
# col_min_reel_in: minimum collective to sustain Fz ≥ weight at xi=80°, derived from aero model
COL_MIN_RAD          = -0.28   # reel-out floor [rad]
COL_MAX_RAD          =  0.10   # extended to support altitude at high tilt [rad]
COL_MIN_REEL_IN_RAD  = col_min_for_altitude_rad(_AERO, XI_REEL_IN_DEG, _ROTOR.mass_kg)

# WinchController safety limit
TENSION_SAFETY_N = 496.0   # ≈ 80% break load


# ── Main simulation ────────────────────────────────────────────────────────────

def _run_deschutter_cycle(
    tension_out:          float = DEFAULT_TENSION_OUT,
    tension_in:           float = DEFAULT_TENSION_IN,
    v_reel_out:           float = V_REEL_OUT,
    v_reel_in:            float = V_REEL_IN,
    t_reel_out:           float = T_REEL_OUT,
    t_reel_in:            float = T_REEL_IN,
    t_transition:         float = T_TRANSITION,
    n_cycles:             int   = 1,
    xi_reel_in_deg:       float = XI_REEL_IN_DEG,
    col_max_rad:          float = COL_MAX_RAD,
    col_min_reel_in_rad:  float = COL_MIN_REEL_IN_RAD,
    body_z_slew_rate:     float = BODY_Z_SLEW_RATE,
) -> dict:
    """
    Run n_cycles De Schutter-style reel-out / reel-in pumping cycles.

    Phase per cycle:
      Reel-out: body_z tether-aligned, high collective → high tension → power.
      Reel-in:  body_z transitions to vertical over t_transition seconds →
                thrust acts upward, low tether tension → low winch energy cost.

    Between reel-in and the next reel-out, body_z blends back from vertical
    to tether-aligned over t_transition seconds.
    """
    dyn    = RigidBodyDynamics(
        mass=5.0, I_body=[5.0, 5.0, 10.0], I_spin=0.0,
        pos0=POS0.tolist(), vel0=VEL0.tolist(),
        R0=build_orb_frame(BODY_Z0), omega0=[0.0, 0.0, 0.0], z_floor=-1.0,
    )
    aero   = create_aero(rd.default())
    tether = TetherModel(anchor_ned=ANCHOR, rest_length=REST_LENGTH0,
                         axle_attachment_length=0.0)

    # ── Trajectory planner (ground station, ~10 Hz equivalent) ──────────────
    # Owns the tension PI internally; outputs normalised thrust [0..1].
    trajectory = DeschutterPlanner(
        t_reel_out      = t_reel_out,
        t_reel_in       = t_reel_in,
        t_transition    = t_transition,
        v_reel_out      = v_reel_out,
        v_reel_in       = v_reel_in,
        tension_out     = tension_out,
        tension_in      = tension_in,
        wind_estimator  = WindEstimator(seed_wind_ned=WIND),
        col_min_rad              = COL_MIN_RAD,
        col_max_rad              = col_max_rad,
        xi_reel_in_deg           = xi_reel_in_deg,
        col_min_reel_in_rad      = col_min_reel_in_rad,
    )

    # ── WinchController (ground station) ─────────────────────────────────────
    winch = WinchController(rest_length=REST_LENGTH0, tension_safety_n=TENSION_SAFETY_N)
    orbit_tracker = OrbitTracker(BODY_Z0, POS0 / np.linalg.norm(POS0), body_z_slew_rate)
    acro          = AcroController.from_rotor(_ROTOR, use_servo=True)

    # ── Mode_RAWES inner loop (400 Hz, on Pixhawk) ────────────────────────────
    # Receives COMMAND packets from trajectory planner.
    # Owns orbit tracking and rate-limited attitude slew.
    # Collective: set_throttle_out(thrust) — direct passthrough, no PI on Pixhawk.

    hub_state  = dyn.state
    omega_spin = OMEGA_SPIN0

    t_total   = n_cycles * (t_reel_out + t_reel_in)
    n_steps   = int(t_total / DT)
    rec_every = int(1.0 / DT)
    tel_every = max(1, int(0.05 / DT))

    ts               = []
    tensions         = []
    tensions_out_acc = []
    tensions_in_acc  = []
    altitudes        = []
    rest_lengths     = []
    collectives      = []
    tilts_from_wind  = []
    floor_hits       = 0
    telemetry        = []
    energy_out       = 0.0
    energy_in        = 0.0

    from planner import Q_IDENTITY
    sw             = {"tilt_lon": 0.0, "tilt_lat": 0.0}
    cmd            = {"attitude_q": Q_IDENTITY.copy(), "thrust": 1.0,
                      "winch_speed_ms": 0.0, "phase": "reel-out"}
    tether.compute(hub_state["pos"], hub_state["vel"], hub_state["R"])
    tension_now    = tether._last_info.get("tension", 0.0)
    collective_rad = 0.0
    body_z_eq      = BODY_Z0.copy()

    for i in range(n_steps):
        t = i * DT

        # ── STATE packet (ground station assembles from MAVLink + local sensors) ──
        state_pkt = {
            "pos_ned":         hub_state["pos"],
            "vel_ned":         hub_state["vel"],
            "omega_spin":      omega_spin,
            "body_z":          hub_state["R"][:, 2],
            "tension_n":       tension_now,
            "tether_length_m": winch.tether_length_m,
        }
        cmd = trajectory.step(state_pkt, DT)
        # cmd: attitude_q, thrust [0..1], winch_speed_ms, phase

        # ── WinchController (ground station) ─────────────────────────────────
        winch.step(cmd["winch_speed_ms"], tension_now, DT)
        tether.rest_length = winch.rest_length

        # ── Mode_RAWES: collective passthrough (SET_ATTITUDE_TARGET thrust) ───
        # Pixhawk denormalises thrust → collective_rad for aero model
        collective_rad = COL_MIN_RAD + cmd["thrust"] * (col_max_rad - COL_MIN_RAD)

        # ── Mode_RAWES: orbit tracking + rate-limited slew → body_z_eq ───────
        _aq = cmd["attitude_q"]
        _bz_target = (None if quat_is_identity(_aq)
                      else quat_apply(_aq, np.array([0.0, 0.0, -1.0])))
        body_z_eq = orbit_tracker.update(hub_state["pos"], DT, _bz_target)

        # ── Mode_RAWES: attitude controller + servo (emulates ArduPilot ACRO) ──
        tilt_lon, tilt_lat = acro.update(hub_state, body_z_eq, DT)
        result = aero.compute_forces(
            collective_rad = collective_rad,
            tilt_lon       = tilt_lon,
            tilt_lat       = tilt_lat,
            R_hub          = hub_state["R"],
            v_hub_world    = hub_state["vel"],
            omega_rotor    = omega_spin,
            wind_world     = WIND,
            t              = T_AERO_OFFSET + t,
        )

        # ── Tether ────────────────────────────────────────────────────────
        tf, tm = tether.compute(hub_state["pos"], hub_state["vel"], hub_state["R"])
        F_net     = result.F_world + tf
        M_orbital = result.M_orbital + tm
        tension_now = tether._last_info.get("tension", 0.0)

        # ── Energy accounting ─────────────────────────────────────────────
        if cmd["phase"] == "reel-out":
            energy_out += tension_now * v_reel_out * DT
            tensions_out_acc.append(tension_now)
        else:
            energy_in  += tension_now * v_reel_in  * DT
            tensions_in_acc.append(tension_now)

        # ── Spin & dynamics ───────────────────────────────────────────────
        omega_spin = max(OMEGA_SPIN_MIN,
                         omega_spin + aero.last_Q_spin / I_SPIN_KGMS2 * DT)
        M_orbital += -50.0 * hub_state["omega"]
        hub_state  = dyn.step(F_net, M_orbital, DT, omega_spin=omega_spin)

        if hub_state["pos"][2] >= -1.05:  # NED: Z >= -1.05 m = altitude <= 1.05 m
            floor_hits += 1
            if floor_hits > 200:
                break   # persistent crash — abort early, test will fail

        # ── Record (1 Hz) ─────────────────────────────────────────────────
        if i % rec_every == 0:
            body_z_cur = hub_state["R"][:, 2]
            xi_deg = float(np.degrees(np.arccos(
                np.clip(np.dot(body_z_cur, WIND / np.linalg.norm(WIND)), -1.0, 1.0)
            )))
            ts.append(t); tensions.append(tension_now)
            altitudes.append(hub_state["pos"][2])
            rest_lengths.append(tether.rest_length)
            collectives.append(collective_rad)
            tilts_from_wind.append(xi_deg)

        # ── Telemetry (20 Hz) — columns match mediator CSV format ────────
        if i % tel_every == 0:
            _ti = tether._last_info
            telemetry.append({
                # ── mediator-compatible columns (same names as mediator CSV) ──
                "t_sim":              t,
                "hub_pos_x":          float(hub_state["pos"][0]),
                "hub_pos_y":          float(hub_state["pos"][1]),
                "hub_pos_z":          float(hub_state["pos"][2]),
                "hub_vel_x":          float(hub_state["vel"][0]),
                "hub_vel_y":          float(hub_state["vel"][1]),
                "hub_vel_z":          float(hub_state["vel"][2]),
                "tether_length":      float(_ti.get("length",   0.0)),
                "tether_extension":   float(_ti.get("extension", 0.0)),
                "tether_tension":     float(tension_now),
                "tether_rest_length": float(tether.rest_length),
                "tether_slack":       int(_ti.get("slack", True)),
                "collective_rad":     float(collective_rad),
                "collective_norm":    float(cmd.get("thrust", 0.0)),
                "pumping_phase":      str(cmd.get("phase", "")),
                "tension_setpoint":   float(cmd.get("tension_setpoint_n", 0.0)),
                "collective_from_tension_ctrl": float(cmd.get("thrust", 0.0)),
                "omega_rotor":        float(omega_spin),
                # ── visualiser-only extras kept for compatibility ─────────────
                "pos_ned":            hub_state["pos"].tolist(),
                "R":                  hub_state["R"].tolist(),
                "swash_collective":   collective_rad,
                "swash_tilt_lon":     sw["tilt_lon"],
                "swash_tilt_lat":     sw["tilt_lat"],
                "body_z_eq":          body_z_eq.tolist(),
                "wind_ned":           WIND.tolist(),
            })

    # Final snapshot
    body_z_cur = hub_state["R"][:, 2]
    xi_deg = float(np.degrees(np.arccos(
        np.clip(np.dot(body_z_cur, WIND / np.linalg.norm(WIND)), -1.0, 1.0)
    )))
    ts.append(t_total); tensions.append(tension_now)
    altitudes.append(hub_state["pos"][2]); rest_lengths.append(tether.rest_length)
    collectives.append(collective_rad); tilts_from_wind.append(xi_deg)

    split = int(t_reel_out)
    return dict(
        ts               = ts,
        tensions         = tensions,
        tensions_out     = tensions[:split],
        tensions_in      = tensions[split:split + int(t_reel_in)],
        tensions_out_all = tensions_out_acc,
        tensions_in_all  = tensions_in_acc,
        altitudes        = altitudes,
        rest_lengths     = rest_lengths,
        collectives      = collectives,
        tilts_from_wind  = tilts_from_wind,
        floor_hits       = floor_hits,
        energy_out       = energy_out,
        energy_in        = energy_in,
        net_energy       = energy_out - energy_in,
        tension_out_sp   = tension_out,
        tension_in_sp    = tension_in,
        n_cycles         = n_cycles,
        telemetry        = telemetry,
    )


# ── Telemetry CSV export (mediator-compatible format) ─────────────────────────

_MEDIATOR_CSV_COLS = [
    "t_sim", "hub_pos_x", "hub_pos_y", "hub_pos_z",
    "hub_vel_x", "hub_vel_y", "hub_vel_z",
    "tether_length", "tether_extension", "tether_tension", "tether_rest_length",
    "tether_slack", "collective_rad", "collective_norm",
    "pumping_phase", "tension_setpoint", "collective_from_tension_ctrl",
    "omega_rotor",
]


def write_mediator_csv(telemetry: list[dict], path: Path) -> None:
    """
    Write telemetry rows to a CSV in the same column format as the mediator.

    Only the mediator-compatible columns are written; visualiser extras
    (pos_ned, R, body_z_eq, wind_ned) are skipped.  This allows
    analyse_pumping_cycle.py to process both unit-test and stack-test runs.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=_MEDIATOR_CSV_COLS,
                                extrasaction="ignore")
        writer.writeheader()
        for row in telemetry:
            writer.writerow({k: row.get(k, "") for k in _MEDIATOR_CSV_COLS})


# ── Tests ──────────────────────────────────────────────────────────────────────

def test_deschutter_no_crash():
    """Hub must stay above z_floor throughout both reel phases."""
    r = _run_deschutter_cycle()
    _print_cycle(r)
    assert r["floor_hits"] == 0, \
        f"Hub hit z_floor {r['floor_hits']} times during De Schutter cycle"


def test_deschutter_reel_in_lower_tension():
    """
    Reel-in mean tension must be less than reel-out mean tension.

    This is the De Schutter mechanism: tilting body_z to vertical during
    reel-in redirects thrust upward (fighting gravity), not along the tether,
    so the winch encounters near-zero aerodynamic resistance.
    """
    r = _run_deschutter_cycle()
    # Exclude the first T_TRANSITION seconds of reel-in (hub is still tilting)
    skip = int(T_TRANSITION)
    mean_out = float(np.mean(r["tensions_out"])) if r["tensions_out"] else 0.0
    mean_in  = float(np.mean(r["tensions_in"][skip:])) if r["tensions_in"][skip:] else 0.0
    assert mean_in < mean_out, (
        f"Reel-in steady tension ({mean_in:.1f} N) not less than reel-out ({mean_out:.1f} N) — "
        "De Schutter tilt mechanism not working"
    )


def test_deschutter_net_energy_positive():
    """
    Reel-out energy must exceed reel-in energy.

    This is the fundamental AWE pumping-cycle condition.  With the De Schutter
    tilt strategy, reel-in tension drops to near the gravity component along
    the tether (~15–30 N), well below the reel-out tension (~150–250 N).
    """
    r = _run_deschutter_cycle()
    assert r["net_energy"] > 0, (
        f"Net energy {r['net_energy']:.1f} J ≤ 0 — "
        "De Schutter cycle does not produce net power"
    )


def test_deschutter_reel_in_tilt_achieved():
    """
    During steady reel-in (after transition), body_z must be within ±10° of
    XI_REEL_IN_DEG (the module default reel-in tilt target).

    The aerodynamic equilibrium settles ~1–3° below the commanded target due
    to controller lag; ±10° tolerance accommodates this.
    """
    r = _run_deschutter_cycle()
    skip = int(T_TRANSITION) + 2   # allow a couple of extra seconds for settling
    tilts_steady = r["tilts_from_wind"][int(T_REEL_OUT) + skip:]
    if not tilts_steady:
        pytest.skip("No steady reel-in data to check tilt")
    mean_tilt = float(np.mean(tilts_steady))
    assert XI_REEL_IN_DEG - 10.0 <= mean_tilt <= XI_REEL_IN_DEG + 10.0, (
        f"Steady reel-in mean tilt = {mean_tilt:.1f}° not within ±10° of "
        f"target {XI_REEL_IN_DEG:.0f}° — body_z not tracking reel-in orientation"
    )


def test_deschutter_reel_out_tilt_in_range():
    """
    During reel-out, body_z must stay within De Schutter's 30–55° tilt range
    (tether-aligned orbit flight).  A large deviation means the tilt controller
    is fighting the tether alignment.
    """
    r = _run_deschutter_cycle()
    tilts_out = r["tilts_from_wind"][:int(T_REEL_OUT)]
    if not tilts_out:
        pytest.skip("No reel-out tilt data")
    mean_tilt_out = float(np.mean(tilts_out))
    assert 25.0 <= mean_tilt_out <= 60.0, (
        f"Reel-out mean tilt ξ = {mean_tilt_out:.1f}° outside 25–60° — "
        "hub not in expected reel-out orientation"
    )


def test_deschutter_tether_not_broken():
    """Peak tension must stay below 80% of Dyneema SK75 1.9 mm break load (620 N)."""
    r    = _run_deschutter_cycle()
    peak = max(r["tensions"])
    limit = 0.8 * BREAK_LOAD_N
    assert peak < limit, \
        f"Peak tension {peak:.1f} N ≥ 80% break load ({limit:.1f} N)"


# ── Analysis CSV export test ─────────────────────────────────────────────────

_LOGS_DIR = Path(__file__).resolve().parents[3] / "simulation" / "logs"
_UNIT_CSV = _LOGS_DIR / "telemetry_unit_deschutter.csv"


@pytest.mark.simtest
def test_deschutter_write_analysis_csv():
    """
    Run one De Schutter cycle and write telemetry to the standard mediator CSV
    format so analyse_pumping_cycle.py can compare unit-test vs stack-test runs.

    Output: simulation/logs/telemetry_unit_deschutter.csv
    Run:    pytest simulation/tests/unit -k test_deschutter_write_analysis_csv
    """
    r = _run_deschutter_cycle()
    write_mediator_csv(r["telemetry"], _UNIT_CSV)
    assert _UNIT_CSV.exists(), f"CSV not written to {_UNIT_CSV}"
    n_rows = sum(1 for _ in _UNIT_CSV.open(encoding="utf-8")) - 1  # minus header
    assert n_rows > 100, f"Only {n_rows} rows written — telemetry appears empty"


# ── Diagnostic log ────────────────────────────────────────────────────────────

def _print_cycle(r):
    sp_out = r["tension_out_sp"]
    sp_in  = r["tension_in_sp"]
    lines = []
    lines.append(f"De Schutter cycle  (setpoints: reel-out={sp_out:.0f}N  reel-in={sp_in:.0f}N)")
    lines.append(f"  {'t':>5}  {'phase':>8}  {'tension':>9}  {'coll':>7}  "
                 f"{'xi_wind':>7}  {'altitude':>9}  {'rest_L':>8}")
    for t, ten, coll, xi, alt, rl in zip(
            r["ts"], r["tensions"], r["collectives"], r["tilts_from_wind"],
            r["altitudes"], r["rest_lengths"]):
        phase = "reel-out" if t <= T_REEL_OUT else "reel-in "
        lines.append(f"  {t:5.1f}  {phase:>8}  {ten:8.1f}N  {np.degrees(coll):6.2f}deg  "
                     f"{xi:6.1f}deg  {alt:8.2f}m  {rl:7.2f}m")
    out = r["tensions_out"]
    inn = r["tensions_in"][int(T_TRANSITION):]
    lines.append(f"")
    lines.append(f"  Reel-out: mean={np.mean(out):.1f}N  max={max(out):.1f}N  "
                 f"energy={r['energy_out']:.1f}J")
    lines.append(f"  Reel-in (steady): mean={np.mean(inn):.1f}N  max={max(inn):.1f}N  "
                 f"energy={r['energy_in']:.1f}J")
    lines.append(f"  Net energy: {r['net_energy']:.1f}J  floor_hits={r['floor_hits']}")
    _log.write(lines, f"net={r['net_energy']:.0f}J  "
               f"reel-out={np.mean(out):.0f}N  reel-in={np.mean(inn):.0f}N  "
               f"floor_hits={r['floor_hits']}")


# ── CLI: generate telemetry JSON for 3D visualizer ────────────────────────────
if __name__ == "__main__":
    import argparse as _ap
    p = _ap.ArgumentParser(description="Run De Schutter cycle and save telemetry")
    p.add_argument("--save-telemetry", metavar="PATH",
                   default="telemetry_deschutter.json",
                   help="Output JSON path (default: telemetry_deschutter.json)")
    args = p.parse_args()

    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
    from simulation.viz3d.telemetry import save_telemetry  # type: ignore

    result = _run_deschutter_cycle(n_cycles=2)
    print(f"Cycles: {result['n_cycles']}  "
          f"Reel-out energy: {result['energy_out']:.1f} J  "
          f"Reel-in energy: {result['energy_in']:.1f} J  "
          f"Net: {result['net_energy']:.1f} J  "
          f"Floor hits: {result['floor_hits']}")
    save_telemetry(args.save_telemetry, result["telemetry"])
    print(f"\nRun visualizer:")
    print(f"  python simulation/viz3d/visualize_3d.py {args.save_telemetry}")
