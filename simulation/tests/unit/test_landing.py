"""
test_landing.py -- De Schutter reel-in followed by vertical landing.

Sequence starting from steady_state_starting.json IC
(same IC as test_steady_flight.py and test_pump_cycle.py):

  reel_in    -- DeschutterPlanner reel-in only (T_REEL_OUT=0): tether
                ~100 m -> ~89 m, body_z slerps to xi=80 deg (nearly
                vertical).  Hub swings from xi~8 deg up to ~67 m altitude.
                No reel-out needed: the reel-in alone swings the hub
                upward cleanly with no slack or tension spikes.

  descent    -- LandingPlanner: tension-PI winch, VZ descent controller,
                body_z fixed at xi~80 deg captured at reel_in exit.
                Reel in from ~89 m to MIN_TETHER_M.

  final_drop -- LandingPlanner: collective=0, hub drops onto catch pad.

Non-Lua reference: test_landing_lua.py
See landing_planner.py for LandingPlanner controller documentation.
"""
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
from planner         import DeschutterPlanner, WindEstimator, quat_apply, quat_is_identity
from landing_planner import LandingPlanner
from winch           import WinchController
from frames          import build_orb_frame
from simtest_log     import SimtestLog, BadEventLog
from simtest_ic      import load_ic
from tel             import make_tel
from telemetry_csv   import TelRow, write_csv

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

# ── Reel-in phase parameters (no reel-out: reel-in alone swings hub to xi~80 deg) ─
T_REEL_OUT   =  0.0    # s  skip reel-out
T_REEL_IN    = 30.0    # s
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
V_LAND        = 0.5    # m/s  descent reel-in speed
COL_CRUISE    = 0.079  # rad  base collective at xi=80 deg (col_min_reel_in)
KP_VZ         = 0.05   # rad/(m/s)  descent rate gain
MIN_TETHER_M  = 2.0    # m    descent -> final_drop transition
T_FINAL_DROP_MAX = 15.0  # s

# ── Pass/fail thresholds ──────────────────────────────────────────────────────
FLOOR_ALT_M          = 1.0
ANCHOR_LAND_RADIUS_M = 20.0
T_PUMPING_END        = T_REEL_OUT + T_REEL_IN


# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------

def _run_landing() -> dict:
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
        t_reel_out              = T_REEL_OUT,
        t_reel_in               = T_REEL_IN,
        t_transition            = T_TRANSITION,
        v_reel_out              = 0.4,
        v_reel_in               = V_REEL_IN,
        tension_out             = TENSION_OUT,
        tension_in              = TENSION_IN,
        wind_estimator          = WindEstimator(seed_wind_ned=WIND),
        col_min_rad             = COL_MIN_RAD,
        col_max_rad             = COL_MAX_RAD,
        xi_reel_in_deg          = XI_REEL_IN_DEG,
        col_min_reel_in_rad     = COL_MIN_REEL_IN_RAD,
    )

    orbit_tracker = OrbitTracker(_IC.body_z, _IC.pos / np.linalg.norm(_IC.pos),
                                 BODY_Z_SLEW_RATE)
    acro          = AcroController.from_rotor(rd.default(), use_servo=True)

    hub_state   = dyn.state
    omega_spin  = _IC.omega_spin
    tether.compute(hub_state["pos"], hub_state["vel"], hub_state["R"])
    tension_now    = tether._last_info.get("tension", 0.0)
    collective_rad = _IC.coll_eq_rad
    body_z_eq      = orbit_tracker.bz_slerp

    outer_phase     = "pumping"
    landing_planner = None
    land_phase      = None

    floor_hit      = False
    t_final_start  = None

    tensions_land  = []
    altitudes      = []
    vz_at_floor    = None
    bz_at_floor    = None
    pos_at_floor   = None
    touchdown_time = None

    events    = BadEventLog()
    telemetry = []
    tel_every = max(1, int(0.05 / DT))

    descent_time = _IC.rest_length / V_LAND + 10.0
    max_steps = int((T_PUMPING_END + descent_time + T_FINAL_DROP_MAX) / DT)

    t_sim = 0.0
    for i in range(max_steps):
        t_sim    = i * DT
        altitude = -hub_state["pos"][2]

        # ── Switch from pumping to landing once one cycle is complete ─────────
        if outer_phase == "pumping" and t_sim >= T_PUMPING_END:
            outer_phase     = "landing"
            landing_planner = LandingPlanner(
                initial_body_z   = orbit_tracker.bz_slerp,
                v_land           = V_LAND,
                col_cruise       = COL_CRUISE,
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

            collective_rad = acro.slew_collective(
                COL_MIN_RAD + pump_cmd["thrust"] * (COL_MAX_RAD - COL_MIN_RAD), DT
            )

            _aq = pump_cmd["attitude_q"]
            _bz_target = (None if quat_is_identity(_aq)
                          else quat_apply(_aq, np.array([0.0, 0.0, -1.0])))
            body_z_eq = orbit_tracker.update(hub_state["pos"], DT, _bz_target)

        # ── Landing phase: LandingPlanner controls everything ─────────────────
        else:
            assert landing_planner is not None
            land_state = {
                "body_z":          hub_state["R"][:, 2],
                "vel_ned":         hub_state["vel"],
                "pos_ned":         hub_state["pos"],
                "tether_length_m": winch.rest_length,
                "tension_n":       tension_now,
            }
            land_cmd   = landing_planner.step(land_state, DT)
            land_phase = land_cmd["phase"]

            if land_phase == "final_drop":
                collective_rad = acro.slew_collective(0.0, DT)
                if t_final_start is None:
                    t_final_start = t_sim
            else:
                collective_rad = acro.step_vz(
                    land_cmd["vz_setpoint_ms"], float(hub_state["vel"][2]),
                    land_cmd["col_cruise_rad"], COL_MIN_RAD, COL_MAX_RAD, KP_VZ, DT)

            body_z_eq = land_cmd["body_z_eq"]

            winch.step(land_cmd["winch_speed_ms"], tension_now, DT)
            tether.rest_length = winch.rest_length

            if land_phase in ("descent",):
                tensions_land.append(tension_now)
                altitudes.append(altitude)

            # Floor hit detection
            if altitude <= FLOOR_ALT_M and not floor_hit:
                floor_hit      = True
                vz_at_floor    = float(hub_state["vel"][2])
                bz_at_floor    = hub_state["R"][:, 2].copy()
                pos_at_floor   = hub_state["pos"].copy()
                touchdown_time = t_sim
                break
            if t_final_start is not None and (t_sim - t_final_start) >= T_FINAL_DROP_MAX:
                break

        # ── Attitude controller ────────────────────────────────────────────────
        tilt_lon, tilt_lat = acro.update(hub_state, body_z_eq, DT)

        # ── Aerodynamics ──────────────────────────────────────────────────────
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

        # ── Bad-event tracking ────────────────────────────────────────────────
        current_phase = (land_phase or "pumping") if outer_phase == "landing" else "pumping"
        if tether._last_info.get("slack", False):
            events.record("slack", t_sim, current_phase, altitude)
        if tension_now > BREAK_LOAD_N:
            events.record("tension_spike", t_sim, current_phase, altitude,
                          tension=tension_now)
        if outer_phase == "landing" and land_phase == "descent":
            events.check_floor(hub_state["pos"][2], t_sim, current_phase, FLOOR_ALT_M)

        # ── Spin & dynamics ───────────────────────────────────────────────────
        omega_spin = max(OMEGA_SPIN_MIN,
                         omega_spin + aero.last_Q_spin / I_SPIN_KGMS2 * DT)
        hub_state  = dyn.step(F_net, M_orbital, DT, omega_spin=omega_spin)

        # ── Telemetry (20 Hz) ─────────────────────────────────────────────────
        if i % tel_every == 0:
            tel_phase = (land_phase or "pumping") if outer_phase == "landing" else "pumping"
            telemetry.append(make_tel(
                t_sim, hub_state, omega_spin, tether, tension_now,
                collective_rad, tilt_lon, tilt_lat, WIND,
                body_z_eq=body_z_eq, phase=tel_phase,
            ))

    # ── Results ───────────────────────────────────────────────────────────────
    anchor_dist    = None
    touchdown_tilt = None
    if pos_at_floor is not None:
        anchor_dist = float(np.linalg.norm(pos_at_floor[:2]))
    if bz_at_floor is not None:
        bz_level       = np.array([0.0, 0.0, -1.0])
        touchdown_tilt = float(np.degrees(np.arccos(
            np.clip(np.dot(bz_at_floor, bz_level), -1.0, 1.0))))

    parts = [
        f"alt_start={-_IC.pos[2]:.1f}m",
        f"tether_start={_IC.rest_length:.0f}m",
        f"max_land_tension={max(tensions_land):.0f}N" if tensions_land else "",
        f"floor_hit={floor_hit}",
    ]
    if anchor_dist    is not None: parts.append(f"anchor_dist={anchor_dist:.1f}m")
    if touchdown_tilt is not None: parts.append(f"tilt={touchdown_tilt:.1f}deg")
    parts.append(f"t_end={t_sim:.1f}s")

    if telemetry:
        write_csv([TelRow.from_tel(d) for d in telemetry],
                  _log.log_dir / "telemetry.csv")
    _log.write(["(telemetry: telemetry.csv)"],
               "  ".join(p for p in parts if p))

    return dict(
        t_end          = t_sim,
        events         = events,
        floor_hit      = floor_hit,
        touchdown_time = touchdown_time,
        vz_at_floor    = vz_at_floor,
        anchor_dist    = anchor_dist,
        touchdown_tilt = touchdown_tilt,
        max_land_tension = max(tensions_land) if tensions_land else None,
        min_altitude   = min(altitudes) if altitudes else None,
        telemetry      = telemetry,
    )


# ---------------------------------------------------------------------------
# Test
# ---------------------------------------------------------------------------

def test_landing():
    """reel_out + reel_in (to xi~80 deg) + descent + final_drop: no bad events, floor reached."""
    r = _run_landing()
    failures = []
    # descent slack/spikes are fatal; pumping and final_drop events are not checked here
    for kind in ("slack", "tension_spike"):
        n = r["events"].count(kind, "descent")
        if n:
            failures.append(f"descent {kind}={n}")
    n_floor = r["events"].count("floor_hit", "descent")
    if n_floor:
        failures.append(f"descent floor_hit={n_floor}")
    if not r["floor_hit"]:
        failures.append("hub did not reach floor during final_drop or descent")
    if r["anchor_dist"] is not None and r["anchor_dist"] >= ANCHOR_LAND_RADIUS_M:
        failures.append(f"anchor_dist={r['anchor_dist']:.1f}m >= {ANCHOR_LAND_RADIUS_M}m")
    if r["touchdown_tilt"] is not None and r["touchdown_tilt"] >= 20.0:
        failures.append(f"touchdown_tilt={r['touchdown_tilt']:.1f}deg >= 20 deg")
    if r["vz_at_floor"] is not None and r["vz_at_floor"] >= 10.0:
        failures.append(f"touchdown_vz={r['vz_at_floor']:.2f} m/s >= 10 m/s")
    assert not failures, "\n  ".join(failures)
