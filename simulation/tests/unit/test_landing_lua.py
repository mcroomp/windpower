"""
test_landing_lua.py -- Lua-controlled landing from steady-flight equilibrium.

Mirrors test_landing.py, replacing the Python LandingPlanner + AcroController
with rawes.lua (mode=4) running in-process via lupa (Lua 5.4).  The physics loop,
IC, aero, tether, and starting state are identical to the non-Lua reference.

Starting point: hub at the aerodynamic equilibrium from steady_state_starting.json
(same IC as test_steady_flight_lua.py and test_pump_cycle_lua.py).

Division of labour (mirrors real stack):
  Python: WinchController reels in at V_REEL; writes final_drop substate to Lua
  Lua:    VZ descent-rate controller (ch3); cyclic orbit tracking (ch1/ch2)

Non-Lua reference: test_landing.py
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
from winch         import WinchController
from simtest_ic    import load_ic
from simtest_log   import SimtestLog, BadEventLog
from tel           import make_tel
from telemetry_csv import TelRow, write_csv
from rawes_lua_harness import RawesLua
from rawes_modes   import MODE_LANDING, LAND_FINAL_DROP

_IC    = load_ic()
_ROTOR = rd.default()
_log   = SimtestLog(__file__)

# ── Timing (matches test_landing.py) ─────────────────────────────────────────
DT            = 1.0 / 400.0
LUA_PERIOD    = 0.010
LUA_EVERY     = round(LUA_PERIOD / DT)
T_AERO_OFFSET = 45.0

# ── Environment (matches test_landing.py) ────────────────────────────────────
ANCHOR = np.zeros(3)
WIND   = np.array([0.0, 10.0, 0.0])   # 10 m/s East (NED Y)

# ── Spin (matches test_landing.py) ───────────────────────────────────────────
I_SPIN_KGMS2   = 10.0
OMEGA_SPIN_MIN = 0.5

# ── Starting state — steady-flight IC (identical to test_landing.py) ─────────
R0_INIT       = _IC.R0
POS_INIT      = _IC.pos
VEL_INIT      = _IC.vel
LAND_TETHER_M = _IC.rest_length
OMEGA_SPIN_IC = _IC.omega_spin

# ── Landing parameters (identical to test_landing.py) ────────────────────────
V_REEL             = 0.5    # m/s reel-in speed (== rawes.lua VZ_LAND_SP)
MIN_TETHER_M       = 2.0    # m   switch to final_drop
BREAK_LOAD_N       = 620.0
FLOOR_ALT_M        = 1.0
ANCHOR_LAND_RADIUS_M = 5.0
T_FINAL_DROP_MAX   = 15.0   # s

# ── PWM decoding (must match rawes.lua constants) ────────────────────────────
_COL_MIN    = -0.28
_COL_MAX    =  0.10
_ACRO_SCALE = 500.0 / (360.0 * math.pi / 180.0)

KP_INNER = RatePID.DEFAULT_KP


def _pwm_to_rate(pwm: int) -> float:
    return (pwm - 1500) / _ACRO_SCALE


def _pwm_to_col(pwm: int) -> float:
    return _COL_MIN + (pwm - 1000) / 1000.0 * (_COL_MAX - _COL_MIN)


def _run_landing() -> dict:
    """
    rawes.lua mode=4 (landing_noyaw) from high-elevation state.
    Mirrors _run_landing() in test_landing.py.
    Python WinchController reels in at V_REEL; Lua drives collective + cyclic.
    """
    sim = RawesLua(mode=MODE_LANDING)
    sim.armed        = True
    sim.healthy      = True
    sim.vehicle_mode = 1
    sim.pos_ned      = POS_INIT.tolist()
    sim.vel_ned      = VEL_INIT.tolist()
    sim.R            = R0_INIT
    sim.gyro         = [0.0, 0.0, 0.0]

    dyn    = RigidBodyDynamics(
        mass=_ROTOR.mass_kg, I_body=[5.0, 5.0, 10.0], I_spin=0.0,
        pos0=POS_INIT.tolist(), vel0=VEL_INIT.tolist(),
        R0=R0_INIT, omega0=[0.0, 0.0, 0.0], z_floor=-1.0,
    )
    aero   = create_aero(_ROTOR)
    tether = TetherModel(anchor_ned=ANCHOR, rest_length=LAND_TETHER_M,
                         axle_attachment_length=_ROTOR.axle_attachment_length_m)
    winch  = WinchController(rest_length=LAND_TETHER_M,
                              tension_safety_n=BREAK_LOAD_N * 0.8,
                              min_length=MIN_TETHER_M)
    pid_lon = RatePID(kp=KP_INNER)
    pid_lat = RatePID(kp=KP_INNER)
    servo   = SwashplateServoModel.from_rotor(_ROTOR)

    omega_spin = OMEGA_SPIN_IC
    hub_state   = dyn.state
    tether.compute(hub_state["pos"], hub_state["vel"], hub_state["R"])
    tension_now = tether._last_info.get("tension", 0.0)

    col_rad  = 0.079   # COL_CRUISE_LAND_RAD (rawes.lua initial)
    roll_sp  = 0.0
    pitch_sp = 0.0

    events         = BadEventLog()
    floor_hit      = False
    t_final_start  = None

    tensions  = []
    altitudes = []
    vz_at_floor    = None
    bz_at_floor    = None
    pos_at_floor   = None
    touchdown_time = None

    tel_every = max(1, int(0.05 / DT))
    telemetry = []

    max_steps = int((LAND_TETHER_M / V_REEL + T_FINAL_DROP_MAX + 10.0) / DT)

    t_sim = 0.0
    for i in range(max_steps):
        t_sim     = i * DT
        hub_state = dyn.state
        altitude  = -hub_state["pos"][2]

        # ── Lua update at 100 Hz ──────────────────────────────────────────
        if i % LUA_EVERY == 0:
            omega_body = hub_state["R"].T @ hub_state["omega"]
            sim._mock.millis_val = int(t_sim * 1000)
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

        # ── Ground planner: trigger final_drop when tether short enough ───
        if not floor_hit and t_final_start is None:
            if tether.rest_length <= MIN_TETHER_M:
                sim.send_named_float("RAWES_SUB", LAND_FINAL_DROP)
                t_final_start = t_sim

        final_drop = (t_final_start is not None)

        # ── Floor hit detection (final_drop only) ─────────────────────────
        if final_drop and not floor_hit:
            if altitude <= FLOOR_ALT_M:
                floor_hit      = True
                vz_at_floor    = float(hub_state["vel"][2])
                bz_at_floor    = hub_state["R"][:, 2].copy()
                pos_at_floor   = hub_state["pos"].copy()
                touchdown_time = t_sim
                break
            if (t_sim - t_final_start) >= T_FINAL_DROP_MAX:
                break

        # ── Winch ─────────────────────────────────────────────────────────
        winch.step(-V_REEL, tension_now, DT)
        tether.rest_length = winch.rest_length

        # ── Inner ACRO rate PID at 400 Hz ────────────────────────────────
        omega_body    = hub_state["R"].T @ hub_state["omega"]
        omega_body[2] = 0.0
        tilt_lon_cmd  =  pid_lon.update(roll_sp,  omega_body[0], DT)
        tilt_lat_cmd  = -pid_lat.update(pitch_sp, omega_body[1], DT)
        tilt_lon, tilt_lat = servo.step(tilt_lon_cmd, tilt_lat_cmd, DT)

        # ── Aerodynamics ──────────────────────────────────────────────────
        result = aero.compute_forces(
            collective_rad=col_rad,
            tilt_lon=tilt_lon, tilt_lat=tilt_lat,
            R_hub=hub_state["R"], v_hub_world=hub_state["vel"],
            omega_rotor=omega_spin, wind_world=WIND,
            t=T_AERO_OFFSET + t_sim,
        )

        # ── Tether ────────────────────────────────────────────────────────
        tf, tm      = tether.compute(hub_state["pos"], hub_state["vel"], hub_state["R"])
        tension_now = tether._last_info.get("tension", 0.0)
        F_net       = result.F_world + tf
        M_orbital   = result.M_orbital + tm

        # ── Bad-event tracking ────────────────────────────────────────────
        phase_str = "final_drop" if final_drop else "descent"
        if tether._last_info.get("slack", False):
            events.record("slack", t_sim, phase_str, altitude)
        if tension_now > BREAK_LOAD_N:
            events.record("tension_spike", t_sim, phase_str, altitude, tension=tension_now)
        if not final_drop:
            events.check_floor(hub_state["pos"][2], t_sim, phase_str, FLOOR_ALT_M)

        # ── Spin & dynamics ───────────────────────────────────────────────
        omega_spin = max(OMEGA_SPIN_MIN,
                         omega_spin + aero.last_Q_spin / I_SPIN_KGMS2 * DT)
        hub_state  = dyn.step(F_net, M_orbital, DT, omega_spin=omega_spin)

        # ── Per-phase metrics (descent only) ─────────────────────────────
        if not final_drop:
            tensions.append(tension_now)
            altitudes.append(altitude)

        if i % tel_every == 0:
            phase_str = "final_drop" if final_drop else "descent"
            telemetry.append(make_tel(
                t_sim, hub_state, omega_spin, tether, tension_now,
                col_rad, tilt_lon, tilt_lat, WIND,
                body_z_eq=None, phase=phase_str,
            ))

    anchor_dist    = None
    touchdown_tilt = None
    if pos_at_floor is not None:
        anchor_dist = float(np.linalg.norm(pos_at_floor[:2]))
    if bz_at_floor is not None:
        bz_level       = np.array([0.0, 0.0, -1.0])
        touchdown_tilt = float(np.degrees(np.arccos(
            np.clip(np.dot(bz_at_floor, bz_level), -1.0, 1.0))))

    parts = [
        f"alt_start={-POS_INIT[2]:.1f}m",
        f"max_tension={max(tensions):.0f}N" if tensions else "",
        f"floor_hit={floor_hit}",
    ]
    if anchor_dist    is not None: parts.append(f"anchor_dist={anchor_dist:.1f}m")
    if touchdown_tilt is not None: parts.append(f"tilt={touchdown_tilt:.1f}deg")
    parts.append(f"t_end={t_sim:.1f}s")
    for level, msg in sim.messages:
        parts.append(f"[GCS {level}] {msg}")

    if telemetry:
        write_csv([TelRow.from_tel(d) for d in telemetry],
                  _log.log_dir / "telemetry.csv")
    _log.write(["(telemetry: telemetry.csv)"], "  ".join(p for p in parts if p))

    return dict(
        t_end          = t_sim,
        events         = events,
        floor_hit      = floor_hit,
        touchdown_time = touchdown_time,
        vz_at_floor    = vz_at_floor,
        anchor_dist    = anchor_dist,
        touchdown_tilt = touchdown_tilt,
        max_tension    = max(tensions)  if tensions else None,
        min_altitude   = min(altitudes) if altitudes else None,
        messages       = sim.messages,
        telemetry      = telemetry,
    )


# ── Test ──────────────────────────────────────────────────────────────────────

def test_landing_lua():
    """rawes.lua mode=4: no bad events, floor reached, touchdown within anchor radius."""
    r = _run_landing()
    failures = []
    if r["events"]:
        failures.append(r["events"].summary())
    if not r["floor_hit"]:
        failures.append("hub did not reach floor during final_drop")
    if r["anchor_dist"] is None:
        failures.append("no anchor_dist (floor not reached)")
    elif r["anchor_dist"] > ANCHOR_LAND_RADIUS_M:
        failures.append(f"anchor_dist={r['anchor_dist']:.1f}m > {ANCHOR_LAND_RADIUS_M}m")
    assert not failures, "\n  ".join(failures)
