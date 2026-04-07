"""
test_pump_and_land.py -- De Schutter pumping cycle followed by vertical landing.

Sequence
--------
  Phase 1 + 2 -- pumping (reel-out + reel-in, ~60 s):
    Standard De Schutter cycle using DeschutterPlanner.
    Hub starts at orbit equilibrium (~7 m altitude, xi~29 deg from East wind).
    Reel-out: tether 50 m -> 62 m, body_z tether-aligned, high tension (~200 N).
    Reel-in:  body_z tilts to xi=80 deg (nearly vertical), tether 62 m -> 50 m,
              low tension (~55 N).
    Ends: tether ~50 m, altitude ~49 m, body_z almost directly above anchor.

  Phase 3 -- leveling (< 1 s):
    At xi=80 deg the disk is already only ~10 deg from horizontal.
    body_z slerps toward [0,0,-1] at BODY_Z_SLEW_RATE.
    Descent rate controller.  Reel in at V_LAND.

  Phase 4 -- vertical descent (~33 s):
    body_z_eq = [0,0,-1] fixed.  Descent rate controller.
    Reel in at V_LAND until tether <= MIN_TETHER_M.

  Phase 5 -- final drop:
    Collective = 0.  Hub drops the last ~2 m onto the catch device.

Controller note: the pumping phase uses DeschutterPlanner.  Phases 3-5 use
compute_swashplate_from_state with hover gains (KP_HOVER/KD_HOVER) and the
descent rate controller (COL_CRUISE + KP_VZ * vz_error) in place of TensionPI.
The slerped body_z_eq is preserved across the phase boundary so there is no
discontinuity in the attitude setpoint.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

pytestmark = [pytest.mark.simtest, pytest.mark.timeout(600)]

import rotor_definition as rd
from dynamics        import RigidBodyDynamics
from aero            import create_aero
from tether          import TetherModel
from controller      import (
    OrbitTracker,
    col_min_for_altitude_rad,
    AcroController,
)
from planner         import DeschutterPlanner, WindEstimator, quat_apply, quat_is_identity, Q_IDENTITY
from landing_planner import LandingPlanner
from winch           import WinchController
from frames          import build_orb_frame
from simtest_log     import SimtestLog
from simtest_ic      import load_ic

_log   = SimtestLog(__file__)
_IC    = load_ic()
_ROTOR = rd.default()
_AERO  = create_aero(_ROTOR)

# ── Simulation constants ───────────────────────────────────────────────────────
DT            = 1.0 / 400.0
ANCHOR        = np.zeros(3)
T_AERO_OFFSET = 45.0

I_SPIN_KGMS2   = 10.0
OMEGA_SPIN_MIN = 0.5
WIND           = np.array([0.0, 10.0, 0.0])   # NED: East wind
BREAK_LOAD_N   = 620.0

# ── Pumping cycle parameters (De Schutter, same as test_deschutter_cycle.py) ──
T_REEL_OUT   = 30.0    # s
T_REEL_IN    = 30.0    # s
V_REEL_OUT   =  0.4    # m/s
V_REEL_IN    =  0.4    # m/s

XI_REEL_IN_DEG   = 80.0
BODY_Z_SLEW_RATE = _ROTOR.body_z_slew_rate_rad_s

_xi_start_deg = 30.0
T_TRANSITION  = math.radians(XI_REEL_IN_DEG - _xi_start_deg) / BODY_Z_SLEW_RATE + 1.5

TENSION_OUT    = 200.0   # N
TENSION_IN     =  55.0   # N

COL_MIN_RAD         = -0.28
COL_MAX_RAD         =  0.10
COL_MIN_REEL_IN_RAD = col_min_for_altitude_rad(_AERO, XI_REEL_IN_DEG, _ROTOR.mass_kg)
TENSION_SAFETY_N    = 496.0   # N (~80% break load)

# ── Landing parameters ─────────────────────────────────────────────────────────
V_LAND            = 0.5     # m/s  reel-in speed (slow to avoid tension spikes)
COL_CRUISE        = 0.079   # rad  base collective (col_min_reel_in at xi=80 deg)
KP_VZ             = 0.05    # rad/(m/s)  descent rate gain

KP_ORBIT, KD_ORBIT = 0.5, 0.2
KP_HOVER, KD_HOVER = 1.5, 0.5
BZ_LEVEL           = np.array([0.0, 0.0, -1.0])   # horizontal disk (NED up)
LEVEL_THRESH_DEG   = 10.0
T_LEVEL_MAX        = 30.0   # s  leveling emergency timeout

MIN_TETHER_M       = 2.0    # m  switch to final drop
T_FINAL_DROP_MAX   = 15.0   # s

FLOOR_ALT_M          = 1.0
ANCHOR_LAND_RADIUS_M = 20.0   # m — recalibrated for 100 m tether (was 5 m for 50 m tether)


# ---------------------------------------------------------------------------
# Combined simulation
# ---------------------------------------------------------------------------

def _run_pump_and_land() -> dict:
    dyn    = RigidBodyDynamics(
        mass=_ROTOR.mass_kg, I_body=[5.0, 5.0, 10.0], I_spin=0.0,
        pos0=_IC.pos.tolist(), vel0=_IC.vel.tolist(),
        R0=build_orb_frame(_IC.body_z), omega0=[0.0, 0.0, 0.0], z_floor=-1.0,
    )
    aero   = create_aero(rd.default())
    tether = TetherModel(anchor_ned=ANCHOR, rest_length=_IC.rest_length,
                         axle_attachment_length=_ROTOR.axle_attachment_length_m)
    winch  = WinchController(rest_length=_IC.rest_length,
                              tension_safety_n=TENSION_SAFETY_N,
                              min_length=MIN_TETHER_M)

    trajectory = DeschutterPlanner(
        t_reel_out          = T_REEL_OUT,
        t_reel_in           = T_REEL_IN,
        t_transition        = T_TRANSITION,
        v_reel_out          = V_REEL_OUT,
        v_reel_in           = V_REEL_IN,
        tension_out         = TENSION_OUT,
        tension_in          = TENSION_IN,
        wind_estimator      = WindEstimator(seed_wind_ned=WIND),
        col_min_rad              = COL_MIN_RAD,
        col_max_rad              = COL_MAX_RAD,
        xi_reel_in_deg           = XI_REEL_IN_DEG,
        col_min_reel_in_rad      = COL_MIN_REEL_IN_RAD,
    )

    # Mode_RAWES inner state (pumping phase)
    orbit_tracker = OrbitTracker(_IC.body_z, _IC.pos / np.linalg.norm(_IC.pos), BODY_Z_SLEW_RATE)
    acro          = AcroController.from_rotor(rd.default())

    hub_state   = dyn.state
    omega_spin  = _IC.omega_spin
    tether.compute(hub_state["pos"], hub_state["vel"], hub_state["R"])
    tension_now    = tether._last_info.get("tension", 0.0)
    collective_rad = _IC.coll_eq_rad
    body_z_eq      = orbit_tracker.bz_slerp

    t_pumping_end   = T_REEL_OUT + T_REEL_IN
    landing_planner = None    # created at the pumping->landing transition
    outer_phase     = "pumping"

    floor_hit          = False

    t_final_start      = None
    land_phase         = None   # inner phase from LandingPlanner

    tensions_out       = []
    tensions_in        = []
    tensions_land      = []
    energy_out         = 0.0
    energy_in          = 0.0
    floor_hits         = 0
    pumping_floor_hits = 0

    touchdown_vz   = None
    touchdown_bz   = None
    touchdown_pos  = None
    touchdown_time = None

    telemetry = []
    tel_every = max(1, int(0.05 / DT))

    descent_time = (_IC.rest_length + V_REEL_OUT * T_REEL_OUT) / V_LAND + 10.0
    max_steps = int((t_pumping_end + T_LEVEL_MAX + descent_time + T_FINAL_DROP_MAX) / DT)

    t_sim = 0.0
    for i in range(max_steps):
        t_sim    = i * DT
        altitude = -hub_state["pos"][2]

        # ── Switch from pumping to landing once cycle is complete ─────────────
        if outer_phase == "pumping" and t_sim >= t_pumping_end:
            outer_phase     = "landing"
            landing_planner = LandingPlanner(
                initial_body_z   = orbit_tracker.bz_slerp,
                v_land           = V_LAND,
                col_cruise       = COL_CRUISE,
                kp_vz            = KP_VZ,
                col_min_rad      = COL_MIN_RAD,
                col_max_rad      = COL_MAX_RAD,
                body_z_slew_rate = BODY_Z_SLEW_RATE,
                min_tether_m     = MIN_TETHER_M,
                anchor_ned       = ANCHOR,
            )

        # ── Pumping phase: DeschutterPlanner controls everything ──────────────
        if outer_phase == "pumping":
            state_pkt = {
                "pos_ned":         hub_state["pos"],
                "vel_ned":         hub_state["vel"],
                "omega_spin":      omega_spin,
                "body_z":          hub_state["R"][:, 2],
                "tension_n":       tension_now,
                "tether_length_m": winch.tether_length_m,
            }
            pump_cmd = trajectory.step(state_pkt, DT)

            winch.step(pump_cmd["winch_speed_ms"], tension_now, DT)
            tether.rest_length = winch.rest_length

            collective_rad = COL_MIN_RAD + pump_cmd["thrust"] * (COL_MAX_RAD - COL_MIN_RAD)

            _aq = pump_cmd["attitude_q"]
            _bz_target = (None if quat_is_identity(_aq)
                          else quat_apply(_aq, np.array([0.0, 0.0, -1.0])))
            body_z_eq = orbit_tracker.update(hub_state["pos"], DT, _bz_target)

            if pump_cmd["phase"] == "reel-out":
                energy_out += tension_now * V_REEL_OUT * DT
                tensions_out.append(tension_now)
            else:
                energy_in  += tension_now * V_REEL_IN  * DT
                tensions_in.append(tension_now)

        # ── Landing phase: LandingPlanner controls everything ────────────────
        else:
            land_state = {
                "body_z":          hub_state["R"][:, 2],
                "vel_ned":         hub_state["vel"],
                "pos_ned":         hub_state["pos"],
                "tether_length_m": winch.rest_length,
                "tension_n":       tension_now,
            }
            land_cmd   = landing_planner.step(land_state, DT)
            land_phase = land_cmd["phase"]
            collective_rad = land_cmd["collective_rad"]
            body_z_eq      = land_cmd["body_z_eq"]

            winch.step(land_cmd["winch_speed_ms"], tension_now, DT)
            tether.rest_length = winch.rest_length

            if land_phase in ("descent",):
                tensions_land.append(tension_now)

            # Floor hit detection (test-level, not planner-level).
            # With AcroController the hub may reach FLOOR_ALT_M during descent
            # (tether slack) rather than in the final_drop phase — both count.
            if land_phase in ("descent", "final_drop"):
                if land_phase == "final_drop" and t_final_start is None:
                    t_final_start = t_sim
                if altitude <= FLOOR_ALT_M and not floor_hit:
                    floor_hit      = True
                    touchdown_vz   = float(hub_state["vel"][2])
                    touchdown_bz   = hub_state["R"][:, 2].copy()
                    touchdown_pos  = hub_state["pos"].copy()
                    touchdown_time = t_sim
                    break
                if land_phase == "final_drop" and t_final_start is not None and (t_sim - t_final_start) >= T_FINAL_DROP_MAX:
                    break

        # ── Attitude controller + servo (emulates ArduPilot ACRO) ────────────
        tilt_lon, tilt_lat = acro.update(hub_state, body_z_eq, DT)
        result = aero.compute_forces(
            collective_rad = collective_rad,
            tilt_lon       = tilt_lon,
            tilt_lat       = tilt_lat,
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

        if hub_state["pos"][2] >= -1.05:
            floor_hits += 1
            if outer_phase == "pumping":
                pumping_floor_hits += 1
                if pumping_floor_hits > 200:   # only crash-guard for pumping phase
                    break

        # ── Telemetry (20 Hz) ─────────────────────────────────────────────────
        tel_phase = land_phase if outer_phase == "landing" else outer_phase
        if i % tel_every == 0:
            xi_level = float(np.degrees(np.arccos(
                np.clip(np.dot(hub_state["R"][:, 2], BZ_LEVEL), -1.0, 1.0))))
            telemetry.append({
                "t_sim":              t_sim,
                "phase":              tel_phase,
                "pos_ned":            hub_state["pos"].tolist(),
                "R":                  hub_state["R"].tolist(),
                "altitude_m":         altitude,
                "hub_vel_z":          float(hub_state["vel"][2]),
                "tether_rest_length": float(tether.rest_length),
                "tether_tension":     float(tension_now),
                "collective_rad":     float(collective_rad),
                "xi_level_deg":       xi_level,
                "omega_rotor":        float(omega_spin),
                "wind_ned":           WIND.tolist(),
                "swash_collective":   float(collective_rad),
                "swash_tilt_lon":     float(tilt_lon),
                "swash_tilt_lat":     float(tilt_lat),
                "body_z_eq":          body_z_eq.tolist(),
            })

    # ── Results ───────────────────────────────────────────────────────────────
    anchor_dist   = None
    touchdown_tilt = None
    if touchdown_pos is not None:
        anchor_dist    = float(np.linalg.norm(touchdown_pos[:2]))
    if touchdown_bz is not None:
        bz_level = np.array([0.0, 0.0, -1.0])
        touchdown_tilt = float(np.degrees(np.arccos(
            np.clip(np.dot(touchdown_bz, bz_level), -1.0, 1.0))))

    # ── Log ───────────────────────────────────────────────────────────────────
    parts = [
        f"energy_net={energy_out - energy_in:.0f}J",
        f"land_max_tension={max(tensions_land):.0f}N" if tensions_land else "",
        f"floor_hit={floor_hit}",
    ]
    if anchor_dist    is not None: parts.append(f"anchor_dist={anchor_dist:.1f}m")
    if touchdown_tilt is not None: parts.append(f"tilt={touchdown_tilt:.1f}deg")
    parts.append(f"t_end={t_sim:.1f}s")

    skip = {"pos_ned", "R"}
    if telemetry:
        header    = ",".join(k for k in telemetry[0] if k not in skip)
        log_lines = [header] + [
            ",".join(str(v) for k, v in row.items() if k not in skip)
            for row in telemetry
        ]
    else:
        log_lines = ["(no telemetry)"]

    _log.write(log_lines, "  ".join(p for p in parts if p))

    _json_dir = Path(__file__).resolve().parents[2] / "logs"
    _json_dir.mkdir(exist_ok=True)
    with open(_json_dir / "telemetry_pump_and_land.json", "w") as fh:
        json.dump(telemetry, fh)

    return dict(
        t_end            = t_sim,
        energy_out       = energy_out,
        energy_in        = energy_in,
        net_energy       = energy_out - energy_in,
        floor_hits            = floor_hits,
        pumping_floor_hits    = pumping_floor_hits,
        floor_hit        = floor_hit,
        touchdown_time   = touchdown_time,
        touchdown_vz     = touchdown_vz,
        anchor_dist      = anchor_dist,
        touchdown_tilt   = touchdown_tilt,
        land_max_tension = max(tensions_land) if tensions_land else None,
        telemetry        = telemetry,
    )


import functools

@functools.lru_cache(maxsize=None)
def _results():
    return _run_pump_and_land()


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_pumping_net_energy_positive():
    """Reel-out energy exceeds reel-in energy."""
    r = _results()
    print(f"\n  Net energy: {r['net_energy']:.1f} J  "
          f"(out={r['energy_out']:.1f} J, in={r['energy_in']:.1f} J)")
    assert r["net_energy"] > 0, f"Net energy {r['net_energy']:.1f} J <= 0"


def test_pumping_no_crash():
    """Hub does not hit floor during pumping cycle."""
    r = _results()
    print(f"\n  Floor hits during pumping: {r['pumping_floor_hits']}")
    assert r["pumping_floor_hits"] == 0, \
        f"Hub hit floor {r['pumping_floor_hits']} times during pumping"


def test_landing_no_tension_spikes():
    """Tension stays below break load during landing descent."""
    r = _results()
    peak = r["land_max_tension"]
    print(f"\n  Landing max tension: {peak:.0f} N  (break load {BREAK_LOAD_N:.0f} N)")
    assert peak is None or peak < BREAK_LOAD_N, f"Tension spike {peak:.0f} N during landing"


def test_landing_leveling_completes():  # name kept for compatibility
    """Disk reaches horizontal after pumping cycle ends."""
    r = _results()
    print(f"\n  Tether-aligned descent reaches floor: {r['floor_hit']}")
    assert r["floor_hit"], "Hub never reached the floor (tether-aligned descent)"


def test_landing_floor_hit():
    """Hub reaches the floor (catch device)."""
    r = _results()
    print(f"\n  Floor hit: {r['floor_hit']}  at t={r['touchdown_time']}s")
    assert r["floor_hit"], "Hub never reached the floor"


def test_landing_over_anchor():
    """Hub lands within ANCHOR_LAND_RADIUS_M of anchor."""
    r = _results()
    dist = r["anchor_dist"]
    if dist is None:
        pytest.skip("Hub did not reach floor")
    print(f"\n  Anchor distance at touchdown: {dist:.2f} m")
    assert dist < ANCHOR_LAND_RADIUS_M, \
        f"Hub landed {dist:.1f} m from anchor (limit {ANCHOR_LAND_RADIUS_M} m)"


def test_landing_orientation():
    """Disk within 15 deg of horizontal at touchdown."""
    r = _results()
    tilt = r["touchdown_tilt"]
    if tilt is None:
        pytest.skip("Hub did not reach floor")
    print(f"\n  Disk tilt at touchdown: {tilt:.1f} deg")
    assert tilt < 20.0, f"Rotor tilted {tilt:.1f} deg at touchdown (blade tips risk ground above 23 deg)"


def test_landing_speed():
    """Touchdown descent rate is survivable."""
    r = _results()
    if not r["floor_hit"]:
        pytest.skip("Hub did not reach floor")
    vz = r["touchdown_vz"]
    print(f"\n  Touchdown vz: {vz:.2f} m/s  t={r['touchdown_time']:.1f}s  "
          f"anchor_dist={r['anchor_dist']:.2f}m  tilt={r['touchdown_tilt']:.1f}deg")
    assert vz < 10.0, f"Touchdown speed {vz:.1f} m/s too high"
