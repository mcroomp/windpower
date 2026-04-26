"""
test_steady_flight_lua.py -- Lua-controlled 90 s altitude-hold steady flight from IC.

Mirrors test_steady_flight.py, replacing the Python controller with rawes.lua
(mode=1) running in-process via lupa (Lua 5.4).

GPS is available from t=0 (pos_ned set at IC position) so rawes.lua fires
_el_initialized on the first tick and starts bz_altitude_hold cyclic +
VZ-PI collective immediately.  RAWES_TEN is fed from the live physics tension
each Lua tick for gravity compensation.

The test uses PhysicsRunner.step_raw(): Lua ch1/ch2 → RatePID + SwashplateServoModel
→ tilt_lon/tilt_lat into physics (no internal Python controller).

Non-Lua reference: test_steady_flight.py
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

pytestmark = [pytest.mark.simtest, pytest.mark.timeout(300)]

import rotor_definition as rd
from controller    import RatePID
from swashplate    import SwashplateServoModel
from simtest_ic    import load_ic
from simtest_log   import SimtestLog, BadEventLog
from simtest_runner import PhysicsRunner, feed_obs
from telemetry_csv import TelRow, write_csv
from rawes_lua_harness import RawesLua
from rawes_modes   import MODE_STEADY

_IC    = load_ic()
_ROTOR = rd.default()
_log   = SimtestLog(__file__)

DT           = 1.0 / 400.0
LUA_PERIOD   = 0.010
LUA_EVERY    = round(LUA_PERIOD / DT)
T_SIM        = 90.0
WIND         = np.array([0.0, 10.0, 0.0])

_COL_MIN    = -0.28
_COL_MAX    =  0.10
_ACRO_SCALE = 500.0 / (360.0 * math.pi / 180.0)

KP_INNER = RatePID.DEFAULT_KP

FLOOR_ALT_M  = 1.0


def _pwm_to_rate(pwm: int) -> float:
    return (pwm - 1500) / _ACRO_SCALE


def _pwm_to_col(pwm: int) -> float:
    return _COL_MIN + (pwm - 1000) / 1000.0 * (_COL_MAX - _COL_MIN)


def _run_steady() -> dict:
    """rawes.lua mode=1: 90 s altitude-hold steady flight from IC."""
    sim = RawesLua(mode=MODE_STEADY)
    sim.armed        = True
    sim.healthy      = True
    sim.vehicle_mode = 1
    sim.pos_ned      = _IC.pos.tolist()   # GPS available from t=0
    sim.vel_ned      = _IC.vel.tolist()
    sim.R            = _IC.R0
    sim.gyro         = [0.0, 0.0, 0.0]

    runner  = PhysicsRunner(_ROTOR, _IC, WIND)
    pid_lon = RatePID(kp=KP_INNER)
    pid_lat = RatePID(kp=KP_INNER)
    servo   = SwashplateServoModel.from_rotor(_ROTOR)

    col_rad  = _IC.coll_eq_rad
    roll_sp  = 0.0
    pitch_sp = 0.0

    events           = BadEventLog()
    max_axle_err_deg = 0.0
    tel_every        = max(1, int(0.05 / DT))
    telemetry        = []
    pos_hist         = []

    for i in range(int(T_SIM / DT)):
        t         = i * DT
        hub_state = runner.hub_state

        if i % LUA_EVERY == 0:
            sim._mock.millis_val = int(t * 1000)
            feed_obs(sim, runner.observe())
            sim.send_named_float("RAWES_TEN", runner.tension_now)
            sim._update_fn()

            ch1 = sim.ch_out[1]
            ch2 = sim.ch_out[2]
            ch3 = sim.ch_out[3]
            if ch1 is not None: roll_sp  = _pwm_to_rate(ch1)
            if ch2 is not None: pitch_sp = _pwm_to_rate(ch2)
            if ch3 is not None: col_rad  = _pwm_to_col(ch3)

        omega_body    = hub_state["R"].T @ hub_state["omega"]
        omega_body[2] = 0.0
        tilt_lon_cmd  =  pid_lon.update(roll_sp,  omega_body[0], DT)
        tilt_lat_cmd  = -pid_lat.update(pitch_sp, omega_body[1], DT)
        tilt_lon, tilt_lat = servo.step(tilt_lon_cmd, tilt_lat_cmd, DT)

        sr = runner.step_raw(DT, col_rad, tilt_lon, tilt_lat)

        altitude    = -runner.hub_state["pos"][2]
        tension_now = runner.tension_now

        if runner.tether._last_info.get("slack", False):
            events.record("slack", t, "flight", altitude, tension=tension_now)
        if runner.hub_state["pos"][2] >= -FLOOR_ALT_M:
            events.record("floor_hit", t, "flight", altitude)

        if t >= 60.0:
            bz   = runner.hub_state["R"][:, 2]
            pos  = runner.hub_state["pos"]
            tdir = pos / max(np.linalg.norm(pos), 0.1)
            dot  = float(np.dot(bz, tdir))
            err  = math.degrees(math.acos(max(-1.0, min(1.0, dot))))
            if err > max_axle_err_deg:
                max_axle_err_deg = err

        pos_hist.append(runner.hub_state["pos"].copy())

        if i % tel_every == 0:
            telemetry.append(TelRow.from_physics(
                runner, sr, col_rad, WIND,
                body_z_eq=None,
            ))

    if max_axle_err_deg > 45.0:
        events.record("axle_misaligned", 60.0, "steady_state", 0.0,
                      max_deg=round(max_axle_err_deg, 1))

    pos_arr    = np.array(pos_hist)
    alts       = -pos_arr[:, 2]
    tensions   = [tel["tether_tension"] for tel in telemetry]

    # Orbit radius over last 30 s (steady state)
    steady_pos = pos_arr[int(60.0 / DT):]
    centroid   = steady_pos.mean(axis=0)
    radii      = np.linalg.norm(steady_pos[:, :2] - centroid[:2], axis=1)

    lines = [
        f"min_alt={alts.min():.2f}m  max_alt={alts.max():.2f}m",
        f"min_tension={min(tensions):.0f}N  max_tension={max(tensions):.0f}N",
        f"orbit_r_mean={radii.mean():.1f}m  orbit_r_max={radii.max():.1f}m",
        f"max_axle_err={max_axle_err_deg:.1f}deg",
        events.summary() or "no bad events",
    ]
    for level, msg in sim.messages[:10]:
        lines.append(f"  [GCS {level}] {msg}")
    if telemetry:
        write_csv(telemetry, _log.log_dir / "telemetry.csv")
    _log.write(lines, "lua_steady")

    return dict(
        events      = events,
        min_alt     = float(alts.min()),
        orbit_r_max = float(radii.max()),
    )


# ── Test ──────────────────────────────────────────────────────────────────────

def test_steady_flight_lua():
    """rawes.lua mode=1: hub aloft, orbit bounded, tether taut, axle aligned for 90 s."""
    r = _run_steady()
    failures = []
    if r["events"]:
        failures.append(r["events"].summary())
    if r["min_alt"] < 3.0:
        failures.append(f"min_alt={r['min_alt']:.2f} m < 3 m")
    if r["orbit_r_max"] > 20.0:
        failures.append(f"orbit_r_max={r['orbit_r_max']:.1f} m > 20 m")
    assert not failures, "\n  ".join(failures)
