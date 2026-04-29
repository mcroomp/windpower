"""
test_steady_flight_lua.py -- Lua-controlled 90 s altitude-hold steady flight from IC.

Mirrors test_steady_flight.py, replacing the Python controller with rawes.lua
(mode=1) running in-process via lupa (Lua 5.4).

GPS is available from t=0 (pos_ned set at IC position) so rawes.lua fires
_el_initialized on the first tick and starts bz_altitude_hold cyclic +
VZ-PI collective immediately.  RAWES_TEN is fed from the live physics tension
each Lua tick for gravity compensation.

Lua ch1/ch2/ch3 → decoded to rate setpoints + collective → PhysicsRunner.step().

Non-Lua reference: test_steady_flight.py
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

pytestmark = [pytest.mark.simtest, pytest.mark.timeout(300)]

from aero import rotor_definition as rd
from simtest_ic    import load_ic
from simtest_log   import BadEventLog
from simtest_runner import PhysicsRunner, LuaAP
from rawes_lua_harness import RawesLua
from rawes_modes   import MODE_STEADY

_IC    = load_ic()
_ROTOR = rd.default()

DT           = 1.0 / 400.0
LUA_PERIOD   = 0.010
LUA_EVERY    = round(LUA_PERIOD / DT)
T_SIM        = 90.0
WIND         = np.array([0.0, 10.0, 0.0])

FLOOR_ALT_M  = 1.0


def _run_steady(log) -> dict:
    """rawes.lua mode=1: 90 s altitude-hold steady flight from IC."""
    sim = RawesLua(mode=MODE_STEADY)
    sim.armed        = True
    sim.healthy      = True
    sim.vehicle_mode = 1
    sim.pos_ned      = _IC.pos.tolist()   # GPS available from t=0
    sim.vel_ned      = _IC.vel.tolist()
    sim.R            = _IC.R0
    sim.gyro         = [0.0, 0.0, 0.0]

    runner  = PhysicsRunner(_ROTOR, _IC, WIND, col_min_rad=-0.28, col_max_rad=0.10)
    lua     = LuaAP(sim, initial_col_rad=_IC.coll_eq_rad, wind=WIND, dt=DT)
    lua.tel_fn = lambda r, sr: dict(body_z_eq=None)

    events           = BadEventLog()
    max_axle_err_deg = 0.0
    pos_hist         = []

    for i in range(int(T_SIM / DT)):
        t = i * DT

        if i % LUA_EVERY == 0:
            lua.tick(t, runner,
                     inject=lambda s, r: s.send_named_float("RAWES_TEN", r.tension_now))

        omega_body    = runner.omega_body
        omega_body[2] = 0.0
        sr = runner.step(DT, lua.col_rad, lua.roll_sp, lua.pitch_sp, omega_body)
        lua.log(runner, sr)

        altitude    = runner.altitude
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

    if max_axle_err_deg > 45.0:
        events.record("axle_misaligned", 60.0, "steady_state", 0.0,
                      max_deg=round(max_axle_err_deg, 1))

    pos_arr    = np.array(pos_hist)
    alts       = -pos_arr[:, 2]
    tensions   = [tel.tether_tension for tel in lua.telemetry]

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
    lua.write_telemetry(log.log_dir / "telemetry.csv")
    log.write(lines, "lua_steady")

    return dict(
        events      = events,
        min_alt     = float(alts.min()),
        orbit_r_max = float(radii.max()),
    )


# ── Test ──────────────────────────────────────────────────────────────────────

def test_steady_flight_lua(simtest_log):
    """rawes.lua mode=1: hub aloft, orbit bounded, tether taut, axle aligned for 90 s."""
    r = _run_steady(simtest_log)
    failures = []
    if r["events"]:
        failures.append(r["events"].summary())
    if r["min_alt"] < 3.0:
        failures.append(f"min_alt={r['min_alt']:.2f} m < 3 m")
    if r["orbit_r_max"] > 20.0:
        failures.append(f"orbit_r_max={r['orbit_r_max']:.1f} m > 20 m")
    assert not failures, "\n  ".join(failures)
