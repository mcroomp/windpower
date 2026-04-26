"""
test_landing.py -- Vertical landing via LandingGroundController / LandingApController.

Mirrors test_pump_cycle_unified.py architecture:
  Ground (10 Hz): LandingGroundController -> LandingCommand
  Winch  (400 Hz): WinchController (set_target at 10 Hz)
  AP     (400 Hz): LandingApController -> (collective, rate_roll, rate_pitch)
                   AcroControllerSitl  -> (tilt_lon, tilt_lat)

Phase sequence from steady_state_starting.json IC:
  reel_in    -- body_z slerps from xi~30 deg to xi~80 deg (~4 s).
                Winch holds at IC rest_length.  VZ PI holds altitude.
  descent    -- body_z fixed at xi~80 deg.  Tension-PI winch reels in
                from IC rest_length to min_tether_m.  VZ PI targets v_land.
  final_drop -- collective=0; hub drops onto catch pad.
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

pytestmark = [pytest.mark.simtest, pytest.mark.timeout(600)]

import rotor_definition as rd
from controller      import AcroControllerSitl
from winch           import WinchController
from simtest_log     import SimtestLog, BadEventLog
from simtest_ic      import load_ic
from simtest_runner  import PhysicsRunner
from telemetry_csv   import TelRow, write_csv
from landing_planner import LandingGroundController
from ap_controller   import LandingApController

_log   = SimtestLog(__file__)
_IC    = load_ic()
_ROTOR = rd.default()

# ── Simulation constants ───────────────────────────────────────────────────────
DT         = 1.0 / 400.0
DT_PLANNER = 1.0 / 10.0
WIND       = np.array([0.0, 10.0, 0.0])
WIND.flags.writeable = False
BREAK_LOAD_N         = 620.0

# ── Landing parameters ─────────────────────────────────────────────────────────
XI_REEL_IN_DEG  = 80.0
COL_REEL_IN_RAD = 0.079    # col_min at xi=80 deg: feedforward seed for VZ PI
COL_CRUISE      = 0.079    # same; VZ PI seed during descent

# Winch: kp*(tension_descent - natural_tension) ≈ v_land
# natural tension during descent ≈ 80 N; kp=0.005 → tension_descent = 80 + 0.5/0.005 = 180 N
TENSION_DESCENT = 180.0    # N

V_LAND       = 0.5    # m/s NED descent rate
MIN_TETHER_M = 2.0    # m   descent → final_drop transition

# ── Pass/fail thresholds ───────────────────────────────────────────────────────
FLOOR_ALT_M          = 1.0
ANCHOR_LAND_RADIUS_M = 20.0
T_FINAL_DROP_MAX     = 15.0   # s


# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------

def _run_landing() -> dict:
    runner = PhysicsRunner(_ROTOR, _IC, WIND)

    ground = LandingGroundController(
        initial_body_z   = _IC.R0[:, 2],
        xi_reel_in_deg   = XI_REEL_IN_DEG,
        slew_rate_rad_s  = _ROTOR.body_z_slew_rate_rad_s,
        col_reel_in_rad  = COL_REEL_IN_RAD,
        col_cruise_rad   = COL_CRUISE,
        tension_descent  = TENSION_DESCENT,
        v_land           = V_LAND,
        min_tether_m     = MIN_TETHER_M,
    )

    ap = LandingApController(
        ic_body_z       = _IC.R0[:, 2],
        slew_rate_rad_s = _ROTOR.body_z_slew_rate_rad_s,
        warm_coll_rad   = _IC.coll_eq_rad,
    )

    winch = WinchController(
        rest_length     = _IC.rest_length,
        kp_tension      = 0.005,
        v_max_out       = 0.40,
        v_max_in        = 0.80,
        accel_limit_ms2 = 0.05,
        min_length      = MIN_TETHER_M,
    )

    events    = BadEventLog()
    acro_sitl = AcroControllerSitl()

    planner_every = max(1, round(DT_PLANNER / DT))
    tel_every     = max(1, int(0.05 / DT))

    floor_hit      = False
    t_final_start  = None
    tensions_desc  = []
    altitudes      = []
    vz_at_floor    = None
    bz_at_floor    = None
    pos_at_floor   = None
    touchdown_time = None
    telemetry      = []

    # Prime first command
    cmd = ground.step(0.0, 0.0, rest_length=_IC.rest_length,
                      hub_alt_m=float(-_IC.pos[2]))
    ap.receive_command(cmd, DT_PLANNER)
    winch.set_target(ground.winch_target_length, ground.winch_target_tension)

    descent_time = _IC.rest_length / V_LAND * 2.0 + 30.0
    max_steps    = int((120.0 + descent_time + T_FINAL_DROP_MAX) / DT) + 1

    t_sim = 0.0
    for i in range(max_steps):
        t_sim = i * DT

        hub_state   = runner.hub_state
        altitude    = -hub_state["pos"][2]
        tension_now = runner.tension_now

        # ── Ground 10 Hz ─────────────────────────────────────────────────
        if i % planner_every == 0:
            cmd = ground.step(t_sim, tension_now,
                              rest_length=winch.rest_length, hub_alt_m=altitude)
            ap.receive_command(cmd, DT_PLANNER)
            winch.set_target(ground.winch_target_length, ground.winch_target_tension)

        # ── Winch 400 Hz ──────────────────────────────────────────────────
        len_before = winch.rest_length
        winch.step(tension_now, DT)
        speed_now = (winch.rest_length - len_before) / DT

        # ── AP 400 Hz ─────────────────────────────────────────────────────
        R          = np.asarray(hub_state["R"], dtype=float)
        omega_body = R.T @ np.asarray(hub_state["omega"], dtype=float)
        vel_ned    = np.asarray(hub_state["vel"], dtype=float)
        collective_rad, rate_roll, rate_pitch = ap.step(hub_state["pos"], R, vel_ned, DT)
        tilt_lon, tilt_lat = acro_sitl.update(rate_roll, rate_pitch, omega_body, DT)

        # ── Physics 400 Hz ────────────────────────────────────────────────
        sr = runner.step_raw(DT, collective_rad, tilt_lon, tilt_lat,
                             rest_length=winch.rest_length)

        # ── Floor detection (final_drop only) ────────────────────────────
        phase = cmd.phase
        if phase == "final_drop":
            if t_final_start is None:
                t_final_start = t_sim
            if not floor_hit and altitude <= FLOOR_ALT_M:
                floor_hit      = True
                vz_at_floor    = float(hub_state["vel"][2])
                bz_at_floor    = hub_state["R"][:, 2].copy()
                pos_at_floor   = hub_state["pos"].copy()
                touchdown_time = t_sim
                break
            if (t_sim - t_final_start) >= T_FINAL_DROP_MAX:
                break
        elif phase == "descent":
            tensions_desc.append(runner.tension_now)
            altitudes.append(altitude)

        # ── Bad-event tracking ────────────────────────────────────────────
        if runner.tether._last_info.get("slack", False):
            events.record("slack", t_sim, phase, altitude)
        if runner.tension_now > BREAK_LOAD_N:
            events.record("tension_spike", t_sim, phase, altitude,
                          tension=runner.tension_now)

        # ── Telemetry 20 Hz ───────────────────────────────────────────────
        if i % tel_every == 0:
            telemetry.append(TelRow.from_physics(
                runner, sr, collective_rad, WIND,
                body_z_eq      = ap.bz_current,
                phase          = phase,
                winch_speed_ms = speed_now,
                elevation_rad  = ap.elevation_rad,
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
        f"floor_hit={floor_hit}",
        f"max_desc_tension={max(tensions_desc):.0f}N" if tensions_desc else "",
    ]
    if anchor_dist    is not None: parts.append(f"anchor_dist={anchor_dist:.1f}m")
    if touchdown_tilt is not None: parts.append(f"tilt={touchdown_tilt:.1f}deg")
    parts.append(f"t_end={t_sim:.1f}s")
    parts.append(events.summary() or "no_bad_events")

    if telemetry:
        write_csv(telemetry, _log.log_dir / "telemetry.csv")
    _log.write(["(telemetry: telemetry.csv)"],
               "  ".join(p for p in parts if p))

    return dict(
        t_end            = t_sim,
        events           = events,
        floor_hit        = floor_hit,
        touchdown_time   = touchdown_time,
        vz_at_floor      = vz_at_floor,
        anchor_dist      = anchor_dist,
        touchdown_tilt   = touchdown_tilt,
        max_desc_tension = max(tensions_desc) if tensions_desc else None,
        min_altitude     = min(altitudes)     if altitudes     else None,
        telemetry        = telemetry,
    )


# ---------------------------------------------------------------------------
# Test
# ---------------------------------------------------------------------------

def test_landing():
    """reel_in (xi~30→80 deg) + VZ descent + final_drop: no descent bad events, floor reached."""
    r = _run_landing()
    failures = []

    for kind in ("slack", "tension_spike"):
        n = r["events"].count(kind, "descent")
        if n:
            failures.append(f"descent {kind}={n}")

    if not r["floor_hit"]:
        failures.append("hub did not reach floor during final_drop")
    if r["anchor_dist"] is not None and r["anchor_dist"] >= ANCHOR_LAND_RADIUS_M:
        failures.append(f"anchor_dist={r['anchor_dist']:.1f}m >= {ANCHOR_LAND_RADIUS_M}m")
    if r["touchdown_tilt"] is not None and r["touchdown_tilt"] >= 20.0:
        failures.append(f"touchdown_tilt={r['touchdown_tilt']:.1f}deg >= 20 deg")
    if r["vz_at_floor"] is not None and r["vz_at_floor"] >= 10.0:
        failures.append(f"touchdown_vz={r['vz_at_floor']:.2f} m/s >= 10 m/s")

    assert not failures, "\n  ".join(failures)
