"""
test_steady_flight_lua.py -- Lua-controlled 90 s orbit from aerodynamic equilibrium.

Mirrors test_steady_flight.py, replacing the Python controller with rawes.lua
running in-process via lupa (Lua 5.4).

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
from dynamics      import RigidBodyDynamics
from aero          import create_aero
from tether        import TetherModel
from controller    import RatePID
from swashplate    import SwashplateServoModel
from simtest_ic    import load_ic
from simtest_log   import SimtestLog, BadEventLog
from tel           import make_tel
from telemetry_csv import TelRow, write_csv
from rawes_lua_harness import RawesLua
from rawes_modes   import MODE_STEADY

_IC    = load_ic()
_ROTOR = rd.default()
_log   = SimtestLog(__file__)

DT            = 1.0 / 400.0
LUA_PERIOD    = 0.010
LUA_EVERY     = round(LUA_PERIOD / DT)
T_AERO_OFFSET = 45.0
T_SIM_STEADY  = 90.0

ANCHOR = np.zeros(3)
WIND   = np.array([0.0, 10.0, 0.0])

I_SPIN_KGMS2   = 10.0
OMEGA_SPIN_MIN = 0.5

_COL_MIN    = -0.28
_COL_MAX    =  0.10
_ACRO_SCALE = 500.0 / (360.0 * math.pi / 180.0)

KP_INNER = RatePID.DEFAULT_KP


def _pwm_to_rate(pwm: int) -> float:
    return (pwm - 1500) / _ACRO_SCALE


def _pwm_to_col(pwm: int) -> float:
    return _COL_MIN + (pwm - 1000) / 1000.0 * (_COL_MAX - _COL_MIN)


def _run_steady() -> dict:
    """rawes.lua mode=1 (steady_noyaw): 90 s orbit from aerodynamic equilibrium."""
    sim = RawesLua(mode=MODE_STEADY)
    sim.armed        = True
    sim.healthy      = True
    sim.vehicle_mode = 1
    sim.pos_ned      = _IC.pos.tolist()
    sim.vel_ned      = _IC.vel.tolist()
    sim.R            = _IC.R0
    sim.gyro         = [0.0, 0.0, 0.0]

    dyn    = RigidBodyDynamics(
        mass=_ROTOR.mass_kg, I_body=[5.0, 5.0, 10.0], I_spin=0.0,
        pos0=_IC.pos.tolist(), vel0=_IC.vel.tolist(),
        R0=_IC.R0, omega0=[0.0, 0.0, 0.0], z_floor=-1.0,
    )
    aero   = create_aero(_ROTOR)
    tether = TetherModel(anchor_ned=ANCHOR, rest_length=_IC.rest_length,
                         axle_attachment_length=_ROTOR.axle_attachment_length_m)
    pid_lon = RatePID(kp=KP_INNER)
    pid_lat = RatePID(kp=KP_INNER)
    servo   = SwashplateServoModel.from_rotor(_ROTOR)

    omega_spin = _IC.omega_spin
    col_rad    = _IC.stack_coll_eq
    roll_sp    = 0.0
    pitch_sp   = 0.0

    events  = BadEventLog()
    hist    = []
    crash_t = None

    tel_every = max(1, int(0.05 / DT))
    telemetry = []

    for i in range(int(T_SIM_STEADY / DT)):
        t         = i * DT
        hub_state = dyn.state

        if i % LUA_EVERY == 0:
            omega_body = hub_state["R"].T @ hub_state["omega"]
            sim._mock.millis_val = int(t * 1000)
            sim.R       = hub_state["R"]
            sim.pos_ned = hub_state["pos"].tolist()
            sim.vel_ned = hub_state["vel"].tolist()
            sim.gyro    = omega_body.tolist()
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

        result = aero.compute_forces(
            collective_rad=col_rad,
            tilt_lon=tilt_lon, tilt_lat=tilt_lat,
            R_hub=hub_state["R"], v_hub_world=hub_state["vel"],
            omega_rotor=omega_spin, wind_world=WIND,
            t=T_AERO_OFFSET + t,
        )
        tf, tm      = tether.compute(hub_state["pos"], hub_state["vel"], hub_state["R"])
        tension_now = tether._last_info.get("tension", 0.0)
        F_net       = result.F_world + tf
        M_orbital   = result.M_orbital + tm
        omega_spin  = max(OMEGA_SPIN_MIN,
                          omega_spin + aero.last_Q_spin / I_SPIN_KGMS2 * DT)
        hub_state   = dyn.step(F_net, M_orbital, DT, omega_spin=omega_spin)

        alt = -hub_state["pos"][2]
        if events.check_floor(hub_state["pos"][2], t, "flight"):
            if crash_t is None:
                crash_t = t

        if i % int(10.0 / DT) == 0:
            hist.append({"t": t, "alt": alt, "tension": tension_now, "col": col_rad})

        if i % tel_every == 0:
            telemetry.append(make_tel(
                t, hub_state, omega_spin, tether, tension_now,
                col_rad, tilt_lon, tilt_lat, WIND,
                body_z_eq=None,
            ))

    alts     = [h["alt"]     for h in hist]
    tensions = [h["tension"] for h in hist]
    lines = [
        f"crash_t={crash_t:.1f}s" if crash_t else "no crash",
        f"min_alt={min(alts):.2f}m  max_tension={max(tensions):.0f}N",
    ]
    for h in hist:
        lines.append(f"  t={h['t']:5.1f}  alt={h['alt']:6.2f}m  "
                     f"tension={h['tension']:6.0f}N  col={h['col']:.3f}rad")
    for level, msg in sim.messages[:10]:
        lines.append(f"  [GCS {level}] {msg}")
    if telemetry:
        write_csv([TelRow.from_tel(d) for d in telemetry],
                  _log.log_dir / "telemetry.csv")
    _log.write(lines, "lua_steady")

    return dict(crash_t=crash_t, events=events, min_alt=min(alts), max_tension=max(tensions),
                hist=hist, messages=sim.messages)


def test_lua_steady_orbit():
    """rawes.lua mode=1: no bad events, hub orbits above 3 m for 90 s."""
    r = _run_steady()
    failures = []
    if r["events"]:
        failures.append(r["events"].summary())
    if r["min_alt"] < 3.0:
        failures.append(f"min_alt={r['min_alt']:.2f}m < 3 m")
    assert not failures, "\n  ".join(failures)
