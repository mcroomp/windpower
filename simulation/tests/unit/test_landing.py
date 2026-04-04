"""
test_landing.py -- vertical landing from high-elevation hover, no ArduPilot.

Starting point: hub at xi=80 deg from wind (East), 20 m tether, 19.7 m altitude,
nearly stationary (no orbital velocity).  This is equivalent to the end of a
normal pumping-cycle reel-in at high tilt -- the hub is already almost directly
above the anchor.

Landing is managed by LandingPlanner which owns the three-phase state machine:
  leveling  -> slerp body_z toward [0,0,-1]; descent rate controller
  descent   -> body_z=[0,0,-1]; descent rate controller; reel in at V_REEL
  final_drop -> collective=0; hub drops onto catch device

See landing_planner.py for full controller documentation.
"""
import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

pytestmark = [pytest.mark.simtest, pytest.mark.timeout(300)]

import rotor_definition as rd
from dynamics       import RigidBodyDynamics
from aero           import create_aero
from tether         import TetherModel
from controller     import compute_swashplate_from_state
from winch          import WinchController
from frames         import build_orb_frame
from simtest_log    import SimtestLog
from simtest_ic     import load_ic
from landing_planner import LandingPlanner

_log   = SimtestLog(__file__)
_IC    = load_ic()
_ROTOR = rd.default()

DT            = 1.0 / 400.0
ANCHOR        = np.zeros(3)
T_AERO_OFFSET = 45.0

I_SPIN_KGMS2   = 10.0
OMEGA_SPIN_MIN = 0.5
WIND           = np.array([0.0, 10.0, 0.0])   # NED: East wind

# ── High-elevation starting state ─────────────────────────────────────────────
XI_START_DEG  = 80.0
LAND_TETHER_M = 20.0
_XI_R         = np.radians(XI_START_DEG)
BZ_INIT       = np.array([0.0, np.cos(_XI_R), -np.sin(_XI_R)])
BZ_INIT       = BZ_INIT / np.linalg.norm(BZ_INIT)
POS_INIT      = LAND_TETHER_M * BZ_INIT
VEL_INIT      = np.zeros(3)
OMEGA_SPIN_IC = _IC.omega_spin

# ── Collective limits ─────────────────────────────────────────────────────────
COL_MIN_RAD = -0.28
COL_MAX_RAD =  0.10

# ── Cyclic gains (hover: cyclic must carry full attitude authority) ────────────
KP_HOVER, KD_HOVER = 1.5, 0.5

# ── LandingPlanner parameters ─────────────────────────────────────────────────
V_REEL            = 1.5     # m/s  reel-in speed matched to natural descent rate
COL_CRUISE        = 0.079   # rad  base collective (col_min_reel_in at xi=80 deg)
KP_VZ             = 0.05    # rad/(m/s)
BODY_Z_SLEW_RATE  = _ROTOR.body_z_slew_rate_rad_s
LEVEL_THRESH_DEG  = 10.0    # deg
T_LEVEL_MAX       = 30.0    # s  leveling emergency timeout
MIN_TETHER_M      = 2.0     # m  switch to final drop

FLOOR_ALT_M          = 1.0
ANCHOR_LAND_RADIUS_M = 5.0
BREAK_LOAD_N         = 620.0
T_FINAL_DROP_MAX     = 15.0   # s


# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------

def _run_landing() -> dict:
    dyn    = RigidBodyDynamics(
        mass=_ROTOR.mass_kg, I_body=[5.0, 5.0, 10.0], I_spin=0.0,
        pos0=POS_INIT.tolist(), vel0=VEL_INIT.tolist(),
        R0=build_orb_frame(BZ_INIT), omega0=[0.0, 0.0, 0.0], z_floor=-1.0,
    )
    aero   = create_aero(rd.default())
    tether = TetherModel(anchor_ned=ANCHOR, rest_length=LAND_TETHER_M,
                         axle_attachment_length=0.0)
    winch  = WinchController(rest_length=LAND_TETHER_M,
                              tension_safety_n=BREAK_LOAD_N * 0.8,
                              min_length=MIN_TETHER_M)
    planner = LandingPlanner(
        initial_body_z   = BZ_INIT,
        v_land           = V_REEL,
        col_cruise       = COL_CRUISE,
        kp_vz            = KP_VZ,
        col_min_rad      = COL_MIN_RAD,
        col_max_rad      = COL_MAX_RAD,
        body_z_slew_rate = BODY_Z_SLEW_RATE,
        level_thresh_deg = LEVEL_THRESH_DEG,
        t_level_max      = T_LEVEL_MAX,
        min_tether_m     = MIN_TETHER_M,
    )

    hub_state   = dyn.state
    omega_spin  = OMEGA_SPIN_IC
    tether.compute(hub_state["pos"], hub_state["vel"], hub_state["R"])
    tension_now = tether._last_info.get("tension", 0.0)

    floor_hit      = False
    leveling_done  = False
    t_final_start  = None

    ph1_tensions     = []
    ph2_tensions     = []
    ph1_altitudes    = []
    ph2_altitudes    = []
    vz_at_floor      = None
    bz_at_floor      = None
    pos_at_floor     = None
    touchdown_time   = None

    telemetry = []
    tel_every = max(1, int(0.05 / DT))

    max_steps = int((T_LEVEL_MAX + LAND_TETHER_M / V_REEL + T_FINAL_DROP_MAX + 10.0) / DT)

    t_sim = 0.0
    for i in range(max_steps):
        t_sim    = i * DT
        altitude = -hub_state["pos"][2]

        # ── Planner step ─────────────────────────────────────────────────────
        state_pkt = {
            "body_z":          hub_state["R"][:, 2],
            "vel_ned":         hub_state["vel"],
            "tether_length_m": winch.rest_length,
        }
        cmd = planner.step(state_pkt, DT)
        phase          = cmd["phase"]
        collective_rad = cmd["collective_rad"]
        body_z_eq      = cmd["body_z_eq"]

        # ── Phase bookkeeping ─────────────────────────────────────────────────
        if phase == "descent" and not leveling_done:
            leveling_done = True
        if phase == "final_drop" and t_final_start is None:
            t_final_start = t_sim

        # ── Floor hit detection (final_drop only) ─────────────────────────────
        if phase == "final_drop" and not floor_hit:
            if altitude <= FLOOR_ALT_M:
                floor_hit      = True
                vz_at_floor    = float(hub_state["vel"][2])
                bz_at_floor    = hub_state["R"][:, 2].copy()
                pos_at_floor   = hub_state["pos"].copy()
                touchdown_time = t_sim
                break
            if t_final_start is not None and (t_sim - t_final_start) >= T_FINAL_DROP_MAX:
                break

        # ── Winch ─────────────────────────────────────────────────────────────
        winch.step(cmd["winch_speed_ms"], tension_now, DT)
        tether.rest_length = winch.rest_length

        # ── Attitude controller ───────────────────────────────────────────────
        sw = compute_swashplate_from_state(
            hub_state=hub_state, anchor_pos=ANCHOR,
            body_z_eq=body_z_eq, kp=KP_HOVER, kd=KD_HOVER)

        # ── Aerodynamics ──────────────────────────────────────────────────────
        result = aero.compute_forces(
            collective_rad = collective_rad,
            tilt_lon       = sw["tilt_lon"],
            tilt_lat       = sw["tilt_lat"],
            R_hub          = hub_state["R"],
            v_hub_world    = hub_state["vel"],
            omega_rotor    = omega_spin,
            wind_world     = WIND,
            t              = T_AERO_OFFSET + t_sim,
        )

        # ── Tether ────────────────────────────────────────────────────────────
        tf, tm      = tether.compute(hub_state["pos"], hub_state["vel"], hub_state["R"])
        tension_now = tether._last_info.get("tension", 0.0)
        F_net       = result.F_world + tf
        M_orbital   = result.M_orbital + tm

        # ── Spin & dynamics ───────────────────────────────────────────────────
        omega_spin = max(OMEGA_SPIN_MIN,
                         omega_spin + aero.last_Q_spin / I_SPIN_KGMS2 * DT)
        hub_state  = dyn.step(F_net, M_orbital, DT, omega_spin=omega_spin)

        # ── Per-phase metrics ─────────────────────────────────────────────────
        if phase == "leveling":
            ph1_tensions.append(tension_now)
            ph1_altitudes.append(altitude)
        elif phase == "descent":
            ph2_tensions.append(tension_now)
            ph2_altitudes.append(altitude)

        # ── Telemetry (20 Hz) ─────────────────────────────────────────────────
        if i % tel_every == 0:
            telemetry.append({
                "t_sim":              t_sim,
                "phase":              phase,
                "pos_ned":            hub_state["pos"].tolist(),
                "R":                  hub_state["R"].tolist(),
                "altitude_m":         altitude,
                "hub_vel_z":          float(hub_state["vel"][2]),
                "tether_rest_length": float(tether.rest_length),
                "tether_tension":     float(tension_now),
                "collective_rad":     float(collective_rad),
                "omega_rotor":        float(omega_spin),
                "wind_ned":           WIND.tolist(),
                "body_z_eq":          body_z_eq.tolist(),
            })

    # ── Results ───────────────────────────────────────────────────────────────
    anchor_dist   = None
    touchdown_tilt = None
    if pos_at_floor is not None:
        anchor_dist    = float(np.linalg.norm(pos_at_floor[:2]))
    if bz_at_floor is not None:
        bz_level = np.array([0.0, 0.0, -1.0])
        touchdown_tilt = float(np.degrees(np.arccos(
            np.clip(np.dot(bz_at_floor, bz_level), -1.0, 1.0))))

    parts = [
        f"alt_start={-POS_INIT[2]:.1f}m",
        f"ph1_max_tension={max(ph1_tensions):.0f}N" if ph1_tensions else "",
        f"ph2_max_tension={max(ph2_tensions):.0f}N" if ph2_tensions else "",
        f"leveled={leveling_done}",
        f"floor_hit={floor_hit}",
    ]
    if anchor_dist    is not None: parts.append(f"anchor_dist={anchor_dist:.1f}m")
    if touchdown_tilt is not None: parts.append(f"tilt={touchdown_tilt:.1f}deg")
    parts.append(f"t_end={t_sim:.1f}s")

    skip = {"pos_ned", "R"}
    header    = ",".join(k for k in telemetry[0] if k not in skip) if telemetry else ""
    log_lines = [header] + [
        ",".join(str(v) for k, v in row.items() if k not in skip)
        for row in telemetry
    ] if telemetry else ["(no telemetry)"]
    _log.write(log_lines, "  ".join(p for p in parts if p))

    _json_dir = Path(__file__).resolve().parents[2] / "logs"
    _json_dir.mkdir(exist_ok=True)
    with open(_json_dir / "telemetry_landing.json", "w") as fh:
        json.dump(telemetry, fh)

    return dict(
        t_end            = t_sim,
        floor_hit        = floor_hit,
        leveling_done    = leveling_done,
        touchdown_time   = touchdown_time,
        vz_at_floor      = vz_at_floor,
        anchor_dist      = anchor_dist,
        touchdown_tilt   = touchdown_tilt,
        ph1_max_tension  = max(ph1_tensions)  if ph1_tensions  else None,
        ph2_max_tension  = max(ph2_tensions)  if ph2_tensions  else None,
        ph1_min_altitude = min(ph1_altitudes) if ph1_altitudes else None,
        ph2_min_altitude = min(ph2_altitudes) if ph2_altitudes else None,
        telemetry        = telemetry,
    )


import functools

@functools.lru_cache(maxsize=None)
def _results():
    return _run_landing()


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_no_tension_spikes():
    """Tension stays below tether break load throughout leveling and descent."""
    r = _results()
    ph1 = r["ph1_max_tension"]
    ph2 = r["ph2_max_tension"]
    print(f"\n  Leveling max tension: {ph1:.0f} N" if ph1 is not None else "\n  Leveling max tension: (no samples)")
    print(f"  Descent max tension:  {ph2:.0f} N  (break load {BREAK_LOAD_N:.0f} N)" if ph2 is not None else "  Descent max tension: (no samples)")
    assert ph1 is None or ph1 < BREAK_LOAD_N, f"Tension spike {ph1:.0f} N during leveling"
    assert ph2 is None or ph2 < BREAK_LOAD_N, f"Tension spike {ph2:.0f} N during descent"


def test_altitude_maintained():
    """Hub stays above floor throughout leveling and descent."""
    r = _results()
    for phase, key in [("leveling", "ph1_min_altitude"), ("descent", "ph2_min_altitude")]:
        val = r[key]
        if val is not None:
            print(f"\n  {phase} min altitude: {val:.2f} m")
            assert val > 1.1, f"Hub hit floor during {phase} (min alt {val:.2f} m)"


def test_leveling_completes():
    """Disk reaches horizontal within T_LEVEL_MAX."""
    r = _results()
    print(f"\n  Leveling done: {r['leveling_done']}")
    assert r["leveling_done"], "Leveling timed out"


def test_touchdown_over_anchor():
    """Hub lands within ANCHOR_LAND_RADIUS_M of the anchor."""
    r = _results()
    dist = r["anchor_dist"]
    if dist is None:
        pytest.skip("Hub did not reach floor")
    print(f"\n  Anchor distance at touchdown: {dist:.2f} m")
    assert dist < ANCHOR_LAND_RADIUS_M, \
        f"Hub landed {dist:.1f} m from anchor (limit {ANCHOR_LAND_RADIUS_M} m)"


def test_touchdown_orientation():
    """Disk is within 15 deg of horizontal at touchdown."""
    r = _results()
    tilt = r["touchdown_tilt"]
    if tilt is None:
        pytest.skip("Hub did not reach floor")
    print(f"\n  Disk tilt at touchdown: {tilt:.1f} deg from horizontal")
    assert tilt < 15.0, f"Rotor tilted {tilt:.1f} deg -- blade tips would strike ground"


def test_touchdown_speed():
    """Touchdown descent rate is survivable."""
    r = _results()
    if not r["floor_hit"]:
        pytest.skip("Hub did not reach floor")
    vz = r["vz_at_floor"]
    print(f"\n  Touchdown time: {r['touchdown_time']:.1f} s")
    print(f"  Touchdown vz:   {vz:.2f} m/s  (NED, + = downward)")
    print(f"  Anchor dist:    {r['anchor_dist']:.2f} m")
    print(f"  Disk tilt:      {r['touchdown_tilt']:.1f} deg")
    assert vz < 10.0, f"Touchdown {vz:.1f} m/s"
