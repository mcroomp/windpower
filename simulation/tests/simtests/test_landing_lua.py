"""
test_landing_lua.py -- rawes.lua mode=4 landing from steady-flight IC.

Mirrors test_landing.py: replaces LandingApController with rawes.lua (mode=4)
running in-process via lupa (Lua 5.4).

Ground side (Python, 10 Hz): LandingGroundController + WinchController.
AP side (Lua, 100 Hz):       rawes.lua VZ PI collective (ch3) + cyclic (ch1/ch2).
                              final_drop triggered by RAWES_SUB=LAND_FINAL_DROP.

Phase sequence from steady_state_starting.json IC:
  reel_in    -- body_z slerps xi~30→80 deg; winch holds; Lua VZ PI holds altitude.
  descent    -- body_z fixed; winch tension-PI reels in; Lua VZ PI descends at v_land.
  final_drop -- collective=0; winch holds; hub drops onto catch pad.
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

pytestmark = [pytest.mark.simtest, pytest.mark.timeout(600)]

import rotor_definition as rd
from winch           import WinchController
from simtest_log     import SimtestLog, BadEventLog
from simtest_ic      import load_ic
from simtest_runner  import PhysicsRunner, feed_obs, tel_every_from_env
from telemetry_csv   import TelRow, write_csv
from landing_planner import LandingGroundController
from rawes_lua_harness import RawesLua
from rawes_modes       import MODE_LANDING, LAND_FINAL_DROP

_log   = SimtestLog(__file__)
_IC    = load_ic()
_ROTOR = rd.default()

# ── Timing ─────────────────────────────────────────────────────────────────────
DT         = 1.0 / 400.0
DT_PLANNER = 1.0 / 10.0
LUA_PERIOD = 0.010          # 100 Hz Lua ticks
LUA_EVERY  = round(LUA_PERIOD / DT)

# ── Environment ────────────────────────────────────────────────────────────────
WIND = np.array([0.0, 10.0, 0.0])
WIND.flags.writeable = False
BREAK_LOAD_N         = 620.0

# ── Landing parameters (identical to test_landing.py) ─────────────────────────
XI_REEL_IN_DEG  = 80.0
COL_REEL_IN_RAD = 0.079
COL_CRUISE      = 0.079
TENSION_DESCENT = 140.0    # N  (natural~40N + V_LAND/kp = 40+100 = 140N)
V_LAND          = 0.5
MIN_TETHER_M    = 2.0

FLOOR_ALT_M          = 1.0
ANCHOR_LAND_RADIUS_M = 20.0
T_FINAL_DROP_MAX     = 15.0

# ── PWM decoding (must match rawes.lua constants) ─────────────────────────────
_COL_MIN    = -0.28
_COL_MAX    =  0.10
_ACRO_SCALE = 500.0 / (360.0 * math.pi / 180.0)


def _pwm_to_rate(pwm: int) -> float:
    return (pwm - 1500) / _ACRO_SCALE


def _pwm_to_col(pwm: int) -> float:
    return _COL_MIN + (pwm - 1000) / 1000.0 * (_COL_MAX - _COL_MIN)


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

    winch = WinchController(
        rest_length     = _IC.rest_length,
        kp_tension      = 0.005,
        v_max_out       = 0.40,
        v_max_in        = 0.80,
        accel_limit_ms2 = 0.5,
        min_length      = MIN_TETHER_M,
    )

    # ── Lua harness (AP side) ──────────────────────────────────────────────
    sim = RawesLua(mode=MODE_LANDING)
    sim.armed        = True
    sim.healthy      = True
    sim.vehicle_mode = 1   # ACRO
    feed_obs(sim, runner.observe())

    events    = BadEventLog()
    planner_every = max(1, round(DT_PLANNER / DT))
    tel_every     = tel_every_from_env(DT)

    col_rad         = _IC.coll_eq_rad
    roll_sp         = 0.0
    pitch_sp        = 0.0
    floor_hit       = False
    t_final_start   = None
    final_drop_sent = False

    tensions_desc = []
    altitudes     = []
    vz_at_floor   = None
    bz_at_floor   = None
    pos_at_floor  = None
    touchdown_time= None
    telemetry     = []

    # Prime first ground command
    cmd = ground.step(0.0, 0.0, rest_length=_IC.rest_length,
                      hub_alt_m=float(-_IC.pos[2]))
    winch.set_target(ground.winch_target_length, ground.winch_target_tension)

    descent_time = _IC.rest_length / V_LAND * 2.0 + 30.0
    max_steps    = int((120.0 + descent_time + T_FINAL_DROP_MAX) / DT) + 1

    t_sim = 0.0
    for i in range(max_steps):
        t_sim     = i * DT
        hub_state = runner.hub_state
        altitude  = -hub_state["pos"][2]

        # ── Ground 10 Hz ─────────────────────────────────────────────────
        if i % planner_every == 0:
            cmd = ground.step(t_sim, runner.tension_now,
                              rest_length=winch.rest_length, hub_alt_m=altitude)
            winch.set_target(ground.winch_target_length, ground.winch_target_tension)

        # ── Winch 400 Hz ──────────────────────────────────────────────────
        winch.step(runner.tension_now, DT)

        # ── Lua 100 Hz ────────────────────────────────────────────────────
        if i % LUA_EVERY == 0:
            sim._mock.millis_val = int(t_sim * 1000)
            feed_obs(sim, runner.observe())
            sim._update_fn()

            ch1 = sim.ch_out[1]
            ch2 = sim.ch_out[2]
            ch3 = sim.ch_out[3]
            if ch1 is not None: roll_sp  = _pwm_to_rate(ch1)
            if ch2 is not None: pitch_sp = _pwm_to_rate(ch2)
            if ch3 is not None: col_rad  = _pwm_to_col(ch3)

        # ── Send final_drop substate to Lua once ──────────────────────────
        if cmd.phase == "final_drop" and not final_drop_sent:
            sim.send_named_float("RAWES_SUB", LAND_FINAL_DROP)
            final_drop_sent = True
            t_final_start   = t_sim

        # ── Inner ACRO rate PID + physics step ───────────────────────────
        omega_body    = hub_state["R"].T @ hub_state["omega"]
        omega_body[2] = 0.0
        sr = runner.step(DT, col_rad, roll_sp, pitch_sp, omega_body,
                         rest_length=winch.rest_length)

        # ── Floor detection + phase metrics ───────────────────────────────
        phase = cmd.phase
        if phase == "final_drop":
            if not floor_hit and altitude <= FLOOR_ALT_M:
                floor_hit      = True
                vz_at_floor    = float(hub_state["vel"][2])
                bz_at_floor    = hub_state["R"][:, 2].copy()
                pos_at_floor   = hub_state["pos"].copy()
                touchdown_time = t_sim
                break
            if t_final_start is not None and (t_sim - t_final_start) >= T_FINAL_DROP_MAX:
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
                runner, sr, col_rad, WIND,
                body_z_eq=None, phase=phase,
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
    for level, msg in sim.messages:
        parts.append(f"[GCS {level}] {msg}")

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
        messages         = sim.messages,
        telemetry        = telemetry,
    )


# ---------------------------------------------------------------------------
# Test
# ---------------------------------------------------------------------------

def test_landing_lua():
    """rawes.lua mode=4: floor reached within anchor radius, no descent bad events."""
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
    if r["vz_at_floor"] is not None and r["vz_at_floor"] >= 10.0:
        failures.append(f"touchdown_vz={r['vz_at_floor']:.2f} m/s >= 10 m/s")

    assert not failures, "\n  ".join(failures)
