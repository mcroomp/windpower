"""
test_landing.py -- Vertical landing via LandingGroundController / LandingApController.

Mirrors test_pump_cycle_unified.py architecture:
  Ground (10 Hz): LandingGroundController -> LandingCommand
  Winch  (400 Hz): WinchController (set_target at 10 Hz)
  AP     (400 Hz): LandingApController -> (collective, rate_roll, rate_pitch)
                   AcroControllerSitl  -> (tilt_lon, tilt_lat)

Phase sequence from steady_state_starting.json IC:
  reel_in      -- winch reels in from IC rest_length (~97m) to target_length_m (50m).
                  body_z stays at IC orientation.  VZ PI holds altitude.
  get_vertical -- winch holds at 50m.  body_z slerps to [0,0,-1] (xi=90 deg, disk horizontal).
                  VZ PI holds altitude.  Uses Peters-He aero (SkewedWakeBEM invalid at xi>85 deg).
  descent      -- disk fixed horizontal.  Winch tension target pulls hub down.
                  VZ PI descends at v_land.  Exits when rest_length <= min_tether_m.
  flare        -- winch holds.  VZ PI at vz_sp=0, seeded at col_descent + col_flare_delta.

Each phase is checked at exit before the test continues to the next assertion.
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

pytestmark = [pytest.mark.simtest, pytest.mark.timeout(600)]

from aero import rotor_definition as rd
from winch           import WinchController
from simtest_log     import BadEventLog
from simtest_ic      import load_ic
from simtest_runner  import PhysicsRunner, PythonAP
from landing_planner import LandingGroundController
from ap_controller   import LandingApController

_IC    = load_ic()
_ROTOR = rd.default()

# ── Simulation constants ───────────────────────────────────────────────────────
DT         = 1.0 / 400.0
DT_PLANNER = 1.0 / 10.0
WIND       = np.array([0.0, 10.0, 0.0])
WIND.flags.writeable = False
BREAK_LOAD_N = 620.0

# ── Landing parameters ─────────────────────────────────────────────────────────
TARGET_LENGTH_M  = 50.0    # m  reel_in target tether length
COL_REEL_IN_RAD  = -0.23   # VZ PI seed during reel_in and get_vertical
COL_DESCENT_RAD  =  0.02   # VZ PI seed during descent (~1.3 deg, autorotation equilibrium at 1 m/s axial inflow)
COL_FLARE_DELTA  = 0.05    # collective bump above col_descent at flare entry [rad]

# Winch reel-in: set target well above expected tension so kp*(target-T) > 0 → full speed
TENSION_REEL_IN = 500.0    # N   forces full-speed reel-in (kp*(500-325)=0.875 → cap v_max_in)
# Descent: kp*(tension_descent - natural_T) ≈ v_land
# Horizontal disk natural tension ≈ 0 N (only gravity); kp=0.005 → 200 N → ~1 m/s
TENSION_DESCENT = 200.0    # N

V_LAND       = 1.0    # m/s NED descent rate during descent
MIN_TETHER_M = 5.0    # m   descent → flare transition

# ── Pass/fail thresholds ───────────────────────────────────────────────────────
REEL_IN_TOL_M        = 2.0    # m   acceptable overshoot past TARGET_LENGTH_M
GET_VERTICAL_TOL_DEG = 5.0    # deg  xi must reach 90 - tol
DESCENT_ALT_MAX_M    = 10.0   # m   hub must be below this at descent exit
FLOOR_ALT_M          = 1.0    # m   flare touchdown threshold
ANCHOR_LAND_RADIUS_M = 20.0   # m
T_FLARE_MAX          = 30.0   # s   max time in flare before timeout


# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------

def _run_landing(log) -> "tuple[dict, PythonAP]":
    runner = PhysicsRunner(_ROTOR, _IC, WIND, aero_model="peters_he", col_min_rad=-0.28, col_max_rad=0.10)

    ground = LandingGroundController(
        initial_body_z   = _IC.R0[:, 2],
        target_length_m  = TARGET_LENGTH_M,
        slew_rate_rad_s  = _ROTOR.body_z_slew_rate_rad_s,
        col_reel_in_rad  = COL_REEL_IN_RAD,
        col_vertical_rad = COL_REEL_IN_RAD,
        col_descent_rad  = COL_DESCENT_RAD,
        col_flare_delta  = COL_FLARE_DELTA,
        tension_reel_in  = TENSION_REEL_IN,
        tension_descent  = TENSION_DESCENT,
        v_land           = V_LAND,
        min_tether_m     = MIN_TETHER_M,
    )

    _ap = LandingApController(
        ic_body_z       = _IC.R0[:, 2],
        slew_rate_rad_s = _ROTOR.body_z_slew_rate_rad_s,
        warm_coll_rad   = _IC.coll_eq_rad,
        kp_vz           = LandingApController.KP_VZ,
        ki_vz           = LandingApController.KI_VZ,
        col_min_rad     = LandingApController.COL_MIN_RAD,
        col_max_rad     = LandingApController.COL_MAX_RAD,
        kp_outer        = 2.5,
    )
    ap = PythonAP(_ap, wind=WIND, dt=DT)
    ap.tel_fn = lambda r, sr: {
        **ap.log_fields(),
        "phase":          phase,
        "winch_speed_ms": speed_now,
    }

    winch = WinchController(
        rest_length     = _IC.rest_length,
        kp_tension      = 0.005,
        v_max_out       = 0.40,
        v_max_in        = 0.80,
        accel_limit_ms2 = 0.5,
        min_length      = MIN_TETHER_M,
    )

    events        = BadEventLog()
    planner_every = max(1, round(DT_PLANNER / DT))
    ap_every      = max(1, round(1.0 / (PythonAP.AP_HZ * DT)))

    # Per-phase exit snapshots: filled when the phase first transitions away
    phase_exit: "dict[str, dict]" = {}

    floor_hit      = False
    t_flare_start  = None
    tensions_desc  = []
    altitudes_desc = []
    vz_at_floor    = None
    bz_at_floor    = None
    pos_at_floor   = None
    phase          = "reel_in"
    prev_phase     = "reel_in"
    speed_now      = 0.0

    # Prime first command
    cmd = ground.step(0.0, 0.0, rest_length=_IC.rest_length,
                      hub_alt_m=float(-_IC.pos[2]))
    ap.ap.receive_command(cmd, DT_PLANNER)
    winch.set_target(ground.winch_target_length, ground.winch_target_tension)

    descent_time = _IC.rest_length / V_LAND * 3.0 + 60.0
    max_steps    = int((120.0 + descent_time + T_FLARE_MAX) / DT) + 1

    t_sim = 0.0
    for i in range(max_steps):
        t_sim = i * DT

        tension_now = runner.tension_now
        altitude    = runner.altitude

        # ── Ground 10 Hz ─────────────────────────────────────────────────
        if i % planner_every == 0:
            cmd = ground.step(t_sim, tension_now,
                              rest_length=winch.rest_length, hub_alt_m=altitude)
            ap.ap.receive_command(cmd, DT_PLANNER)
            winch.set_target(ground.winch_target_length, ground.winch_target_tension)

        # ── Winch 400 Hz ──────────────────────────────────────────────────
        len_before = winch.rest_length
        winch.step(tension_now, DT)
        speed_now = (winch.rest_length - len_before) / DT

        # ── AP 50 Hz ──────────────────────────────────────────────────────
        if i % ap_every == 0:
            ap.tick(t_sim, runner)

        # ── Physics 400 Hz ────────────────────────────────────────────────
        omega_body    = runner.omega_body
        omega_body[2] = 0.0
        sr = runner.step(DT, ap.col_rad, ap.roll_sp, ap.pitch_sp, omega_body,
                         rest_length=winch.rest_length)

        phase     = cmd.phase
        hub_state = runner.hub_state

        # ── Record phase exit snapshot ─────────────────────────────────────
        if phase != prev_phase and prev_phase not in phase_exit:
            bz   = hub_state["R"][:, 2]
            xi   = float(np.degrees(np.arcsin(np.clip(-bz[2], -1.0, 1.0))))
            phase_exit[prev_phase] = dict(
                t            = t_sim,
                rest_length  = winch.rest_length,
                altitude     = altitude,
                xi_deg       = xi,
                timeout      = (prev_phase == "reel_in"      and winch.rest_length > TARGET_LENGTH_M + REEL_IN_TOL_M) or
                               (prev_phase == "get_vertical" and xi < 90.0 - GET_VERTICAL_TOL_DEG),
            )
        prev_phase = phase

        # ── Phase-specific tracking ────────────────────────────────────────
        if phase == "flare":
            if t_flare_start is None:
                t_flare_start = t_sim
            if not floor_hit and altitude <= FLOOR_ALT_M:
                floor_hit      = True
                vz_at_floor    = float(hub_state["vel"][2])
                bz_at_floor    = hub_state["R"][:, 2].copy()
                pos_at_floor   = hub_state["pos"].copy()
                break
            if (t_sim - t_flare_start) >= T_FLARE_MAX:
                break
        elif phase == "descent":
            tensions_desc.append(tension_now)
            altitudes_desc.append(altitude)

        # ── Bad-event tracking ────────────────────────────────────────────
        if runner.tether._last_info.get("slack", False):
            events.record("slack", t_sim, phase, altitude)
        if runner.tension_now > BREAK_LOAD_N:
            events.record("tension_spike", t_sim, phase, altitude,
                          tension=runner.tension_now)

        # ── Telemetry 20 Hz ───────────────────────────────────────────────
        ap.log(runner, sr)

    # Capture final phase if it never transitioned away
    if phase not in phase_exit:
        bz = runner.hub_state["R"][:, 2]
        xi = float(np.degrees(np.arcsin(np.clip(-bz[2], -1.0, 1.0))))
        phase_exit[phase] = dict(
            t           = t_sim,
            rest_length = winch.rest_length,
            altitude    = runner.altitude,
            xi_deg      = xi,
            timeout     = True,
        )

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
    log.write(["(telemetry: telemetry.csv)"],
              "  ".join(p for p in parts if p))

    return (
        dict(
            t_end            = t_sim,
            events           = events,
            phase_exit       = phase_exit,
            floor_hit        = floor_hit,
            vz_at_floor      = vz_at_floor,
            anchor_dist      = anchor_dist,
            touchdown_tilt   = touchdown_tilt,
            max_desc_tension = max(tensions_desc) if tensions_desc else None,
            min_altitude     = min(altitudes_desc) if altitudes_desc else None,
        ),
        ap,
    )


# ---------------------------------------------------------------------------
# Test
# ---------------------------------------------------------------------------

def test_landing(simtest_log):
    """4-phase landing: each phase verified at exit before checking the next."""
    r, ap = _run_landing(simtest_log)
    try:
        px = r["phase_exit"]

        # ── Phase 1: reel_in ──────────────────────────────────────────────────
        ri = px.get("reel_in")
        assert ri is not None, "reel_in phase never started"
        assert ri["rest_length"] <= TARGET_LENGTH_M + REEL_IN_TOL_M, (
            f"reel_in did not reach target: rest_length={ri['rest_length']:.1f}m "
            f"(target={TARGET_LENGTH_M}m, tol={REEL_IN_TOL_M}m) -- "
            f"winch tension_reel_in may be too low relative to flight tension"
        )

        # ── Phase 2: get_vertical ─────────────────────────────────────────────
        gv = px.get("get_vertical")
        assert gv is not None, "get_vertical phase never started"
        assert gv["xi_deg"] >= 90.0 - GET_VERTICAL_TOL_DEG, (
            f"get_vertical did not reach horizontal disk: xi={gv['xi_deg']:.1f}deg "
            f"(need >= {90.0 - GET_VERTICAL_TOL_DEG:.1f} deg)"
        )

        # ── Phase 3: descent ──────────────────────────────────────────────────
        de = px.get("descent")
        assert de is not None, "descent phase never started"
        for kind in ("slack", "tension_spike"):
            n = r["events"].count(kind, "descent")
            assert n == 0, f"descent {kind}={n}"
        assert de["rest_length"] <= MIN_TETHER_M + 1.0, (
            f"descent did not reel in to min_tether: rest_length={de['rest_length']:.1f}m "
            f"(target<={MIN_TETHER_M}m)"
        )
        assert de["altitude"] <= DESCENT_ALT_MAX_M, (
            f"descent exit altitude too high: {de['altitude']:.1f}m (max={DESCENT_ALT_MAX_M}m)"
        )

        # ── Phase 4: flare ────────────────────────────────────────────────────
        assert r["floor_hit"], "hub did not reach floor during flare"
        if r["anchor_dist"] is not None:
            assert r["anchor_dist"] < ANCHOR_LAND_RADIUS_M, (
                f"anchor_dist={r['anchor_dist']:.1f}m >= {ANCHOR_LAND_RADIUS_M}m"
            )
        if r["touchdown_tilt"] is not None:
            assert r["touchdown_tilt"] < 20.0, (
                f"touchdown_tilt={r['touchdown_tilt']:.1f}deg >= 20 deg"
            )
        if r["vz_at_floor"] is not None:
            assert r["vz_at_floor"] < 10.0, (
                f"touchdown_vz={r['vz_at_floor']:.2f} m/s >= 10 m/s"
            )
    finally:
        ap.write_telemetry(simtest_log.log_dir / "telemetry.csv")
