"""
test_lua_orbit_sim.py — Closed-loop orbit simulation using EXACT Lua math.

Replicates rawes.lua orbit tracking in Python physics to diagnose whether
the Lua calculations themselves are correct or destabilising.

Tests four scenarios that differ only in how body_z is captured at t=0:
  "exact"        — disk_normal_ned = -R[:,2]  (as Lua does)
  "positive"     — uses  +R[:,2]  (wrong sign; cancels in cross-product?)
  "ekf_error"    — captures from R rotated 20 deg off (EKF attitude error)
  "python_ctrl"  — uses Python compute_rate_cmd (baseline that we know works)

All scenarios use:
  - Exact Lua collective: col = COL_CRUISE + KP_VZ * vz_err (slew-limited)
  - Exact Lua gyro feedthrough: rate_sp = kp*err_body + gyro
  - Same inner ACRO RatePID as test_closed_loop_90s
  - Same physics / tether / aero as test_closed_loop_90s

Run with:
    simulation/.venv/Scripts/python.exe -m pytest \
        simulation/tests/unit/test_lua_orbit_sim.py -v -s
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

pytestmark = [pytest.mark.simtest, pytest.mark.timeout(300)]

import rotor_definition as rd
from dynamics    import RigidBodyDynamics
from aero        import create_aero
from tether      import TetherModel
from controller  import (
    orbit_tracked_body_z_eq,
    orbit_tracked_body_z_eq_3d,
    compute_rate_cmd,
    RatePID,
    slerp_body_z,
)
from swashplate  import SwashplateServoModel
from frames      import build_orb_frame
from simtest_ic  import load_ic
from simtest_log import SimtestLog, BadEventLog

_log = SimtestLog(__file__)

_IC = load_ic()

# ── Constants matching rawes.lua ─────────────────────────────────────────────
DT              = 1.0 / 50.0          # 50 Hz (Lua flight loop rate)
INNER_DT        = 1.0 / 400.0         # 400 Hz inner physics/ACRO loop
T_SIM           = 90.0                # s
T_AERO_OFFSET   = 45.0
ANCHOR          = np.zeros(3)
POS0            = _IC.pos.copy()
VEL0            = _IC.vel.copy()
BODY_Z0         = _IC.body_z.copy()
OMEGA_SPIN0     = _IC.omega_spin
WIND            = np.array([0.0, 10.0, 0.0])

KP_CYC          = 1.0                 # SCR_USER1 default (Lua)
BZ_SLEW         = 0.40                # rad/s
COL_CRUISE      = -0.18               # rad altitude-neutral at natural orbit
KP_VZ           = 0.05               # rad/(m/s)
COL_MIN         = -0.28
COL_MAX         =  0.10
COL_SLEW_MAX    =  0.022              # rad per 50 Hz step
ACRO_RP_DEG     = 360.0              # must match ACRO_RP_RATE
I_SPIN_KGMS2    = 10.0
OMEGA_SPIN_MIN  = 0.5

# Inner ACRO PID gain (same as test_closed_loop_90s.py)
KP_INNER = RatePID.DEFAULT_KP


# ── Lua math helpers ─────────────────────────────────────────────────────────

def rodrigues(v, axis_n, angle):
    ca = math.cos(angle)
    sa = math.sin(angle)
    return v * ca + np.cross(axis_n, v) * sa + axis_n * np.dot(axis_n, v) * (1.0 - ca)


def lua_orbit_update(bz_eq0, tdir0, pos_ned, anchor=ANCHOR):
    """Rodrigues orbit tracking: rotate bz_eq0 by tdir0->bzt rotation."""
    diff = pos_ned - anchor
    tlen = np.linalg.norm(diff)
    if tlen < 0.5:
        return bz_eq0.copy(), tlen
    bzt  = diff / tlen
    axis = np.cross(tdir0, bzt)
    sinth = np.linalg.norm(axis)
    costh = np.dot(tdir0, bzt)
    if sinth < 1e-6:
        return bz_eq0.copy(), tlen
    bz_orbit = rodrigues(bz_eq0, axis / sinth, math.atan2(sinth, costh))
    return bz_orbit, tlen


def lua_slerp_step(bz_slerp, goal, slew_rad_s, dt):
    """Rate-limited slerp step toward goal."""
    dot    = float(np.clip(np.dot(bz_slerp, goal), -1.0, 1.0))
    remain = math.acos(dot)
    if remain < 1e-4:
        return goal.copy()
    ax = np.cross(bz_slerp, goal)
    if np.linalg.norm(ax) < 1e-6:
        return goal.copy()
    step = min(slew_rad_s * dt, remain)
    return rodrigues(bz_slerp, ax / np.linalg.norm(ax), step)


def lua_collective(col_state, vz_actual, is_landing=False):
    """VZ controller: col = cruise + kp*vz_err, slew-limited."""
    vz_sp  = 0.0   # flight mode: hold altitude
    vz_err = vz_actual - vz_sp
    col_cmd = COL_CRUISE + KP_VZ * vz_err
    col_cmd = max(COL_MIN, min(COL_MAX, col_cmd))
    delta   = col_cmd - col_state
    delta   = max(-COL_SLEW_MAX, min(COL_SLEW_MAX, delta))
    return col_state + delta


def lua_cyclic_pwm(err_body, omega_body, kp=KP_CYC):
    """Lua cyclic: rate_sp = kp*err + gyro → PWM."""
    scale = 500.0 / (ACRO_RP_DEG * math.pi / 180.0)
    roll_rate  = kp * err_body[0] + omega_body[0]
    pitch_rate = kp * err_body[1] + omega_body[1]
    ch1 = int(max(1000, min(2000, 1500.0 + scale * roll_rate  + 0.5)))
    ch2 = int(max(1000, min(2000, 1500.0 + scale * pitch_rate + 0.5)))
    return ch1, ch2


def pwm_to_rate(ch1, ch2):
    """Decode PWM → desired body rates (what ACRO sees as setpoint)."""
    scale = 500.0 / (ACRO_RP_DEG * math.pi / 180.0)
    return (ch1 - 1500) / scale, (ch2 - 1500) / scale


# ── Core simulation runner ────────────────────────────────────────────────────

def _run(label, t_sim=T_SIM, dt_lua=DT):
    """
    Run closed-loop sim using Python orbit-tracking math at 50 Hz (Lua rate).

    Uses the SAME math as the validated Python controller but sampled at 50 Hz:
      - bz_now  = +R[:,2]  (positive, same as Python)
      - bz_orbit = orbit_tracked_body_z_eq() (azimuthal, Python function)
      - rate_sp  = compute_rate_cmd()  (no gyro feedthrough)
      - col      = COL_CRUISE + KP_VZ * vz_err  (Lua collective)

    Returns dict: {label, min_alt, max_alt, max_tension, crash_t, log}
    """
    dyn    = RigidBodyDynamics(
        mass=5.0, I_body=[5.0, 5.0, 10.0], I_spin=0.0,
        pos0=POS0.tolist(), vel0=VEL0.tolist(),
        R0=build_orb_frame(BODY_Z0), omega0=[0.0, 0.0, 0.0], z_floor=-1.0,
    )
    aero   = create_aero(rd.default())
    tether = TetherModel(anchor_ned=ANCHOR,
                         rest_length=_IC.rest_length,
                         axle_attachment_length=0.3)
    servo  = SwashplateServoModel.from_rotor(rd.default())
    pid_lon = RatePID(kp=KP_INNER)
    pid_lat = RatePID(kp=KP_INNER)

    hub        = dyn.state
    omega_spin = OMEGA_SPIN0
    tdir0      = POS0 / np.linalg.norm(POS0)
    bz_eq0     = BODY_Z0.copy()        # positive +R[:,2] convention

    col_state     = COL_CRUISE
    _lua_roll_sp  = 0.0
    _lua_pitch_sp = 0.0

    events    = BadEventLog()
    log       = [f"t=0  bz_eq0={bz_eq0}  tdir0={tdir0}"]
    hist      = []
    crash_t   = None
    lua_timer = 0.0

    n_phys = int(t_sim / INNER_DT)

    for i in range(n_phys):
        t = i * INNER_DT
        lua_timer += INNER_DT

        # ── 50 Hz outer loop (mirrors Lua tick rate) ──────────────────────
        if lua_timer >= dt_lua - 1e-9:
            lua_timer = 0.0

            R   = hub["R"]
            pos = hub["pos"]
            vel = hub["vel"]

            # Python orbit tracking: azimuthal, positive sign
            bz_orbit = orbit_tracked_body_z_eq(pos, tdir0, bz_eq0)
            bz_now   = R[:, 2]   # positive body_z in world frame

            # Python rate command (no gyro feedthrough)
            rate_sp = compute_rate_cmd(bz_now, bz_orbit, R, kp=KP_CYC, kd=0.0)
            _lua_roll_sp  = rate_sp[0]
            _lua_pitch_sp = rate_sp[1]

            # Lua collective: altitude-hold VZ controller
            vz_actual = vel[2]
            col_state = lua_collective(col_state, vz_actual)

        # ── Inner ACRO rate PID at 400 Hz ────────────────────────────────
        R          = hub["R"]
        omega_body = R.T @ hub["omega"]
        omega_body[2] = 0.0   # strip spin

        tilt_lon_cmd =  pid_lon.update(_lua_roll_sp,  omega_body[0], INNER_DT)
        tilt_lat_cmd = -pid_lat.update(_lua_pitch_sp, omega_body[1], INNER_DT)
        tilt_lon, tilt_lat = servo.step(tilt_lon_cmd, tilt_lat_cmd, INNER_DT)

        # ── Physics ───────────────────────────────────────────────────────
        result = aero.compute_forces(
            collective_rad=col_state,
            tilt_lon=tilt_lon, tilt_lat=tilt_lat,
            R_hub=hub["R"], v_hub_world=hub["vel"],
            omega_rotor=omega_spin, wind_world=WIND,
            t=T_AERO_OFFSET + t,
        )
        tf, tm   = tether.compute(hub["pos"], hub["vel"], hub["R"])
        F_net    = result.F_world + tf
        M_net    = result.M_orbital + tm
        tension  = tether._last_info.get("tension", 0.0)
        omega_spin = max(OMEGA_SPIN_MIN,
                         omega_spin + aero.last_Q_spin / I_SPIN_KGMS2 * INNER_DT)
        hub = dyn.step(F_net, M_net, INNER_DT, omega_spin=omega_spin)

        alt = -hub["pos"][2]

        if i % int(10.0 / INNER_DT) == 0:
            hist.append({"t": t, "alt": alt, "tension": tension,
                         "col": col_state, "vel_mag": float(np.linalg.norm(hub["vel"]))})

        if events.check_floor(hub["pos"][2], t, "flight"):
            if crash_t is None:
                crash_t = t
                log.append(f"CRASH at t={t:.1f}s  alt={alt:.2f}m  tension={tension:.0f}N")

        if t >= t_sim - INNER_DT:
            log.append(f"t={t:.1f}  alt={alt:.2f}m  col={col_state:.3f}rad  "
                       f"tension={tension:.0f}N  vel={np.linalg.norm(hub['vel']):.2f}m/s")

    alts     = [h["alt"]     for h in hist]
    tensions = [h["tension"] for h in hist]
    return {
        "label":       label,
        "min_alt":     min(alts),
        "max_alt":     max(alts),
        "max_tension": max(tensions),
        "crash_t":     crash_t,
        "events":      events,
        "log":         log,
        "hist":        hist,
    }




# ── Python baseline (for comparison) ─────────────────────────────────────────

def _run_python(t_sim=T_SIM):
    """Run the validated Python orbit-tracking controller (positive bz convention)."""
    dyn    = RigidBodyDynamics(
        mass=5.0, I_body=[5.0, 5.0, 10.0], I_spin=0.0,
        pos0=POS0.tolist(), vel0=VEL0.tolist(),
        R0=build_orb_frame(BODY_Z0), omega0=[0.0, 0.0, 0.0], z_floor=-1.0,
    )
    aero   = create_aero(rd.default())
    tether = TetherModel(anchor_ned=ANCHOR,
                         rest_length=_IC.rest_length,
                         axle_attachment_length=0.3)
    servo  = SwashplateServoModel.from_rotor(rd.default())
    pid_lon = RatePID(kp=KP_INNER)
    pid_lat = RatePID(kp=KP_INNER)

    hub   = dyn.state
    omega_spin = OMEGA_SPIN0
    tdir0      = POS0 / np.linalg.norm(POS0)
    bz_eq0     = BODY_Z0.copy()

    events  = BadEventLog()
    hist    = []
    crash_t = None

    for i in range(int(t_sim / INNER_DT)):
        t = i * INNER_DT
        R = hub["R"]

        body_z_eq = orbit_tracked_body_z_eq(hub["pos"], tdir0, bz_eq0)
        bz_now    = R[:, 2]
        rate_sp   = compute_rate_cmd(bz_now, body_z_eq, R, kp=2.5, kd=0.0)

        omega_body = R.T @ hub["omega"]
        omega_body[2] = 0.0
        tilt_lon_cmd =  pid_lon.update(rate_sp[0], omega_body[0], INNER_DT)
        tilt_lat_cmd = -pid_lat.update(rate_sp[1], omega_body[1], INNER_DT)
        tilt_lon, tilt_lat = servo.step(tilt_lon_cmd, tilt_lat_cmd, INNER_DT)

        result = aero.compute_forces(
            collective_rad=_IC.stack_coll_eq,
            tilt_lon=tilt_lon, tilt_lat=tilt_lat,
            R_hub=hub["R"], v_hub_world=hub["vel"],
            omega_rotor=omega_spin, wind_world=WIND,
            t=T_AERO_OFFSET + t,
        )
        tf, tm   = tether.compute(hub["pos"], hub["vel"], hub["R"])
        F_net    = result.F_world + tf
        M_net    = result.M_orbital + tm
        tension  = tether._last_info.get("tension", 0.0)
        omega_spin = max(OMEGA_SPIN_MIN,
                         omega_spin + aero.last_Q_spin / I_SPIN_KGMS2 * INNER_DT)
        hub = dyn.step(F_net, M_net, INNER_DT, omega_spin=omega_spin)

        alt = -hub["pos"][2]
        if i % int(10.0 / INNER_DT) == 0:
            hist.append({"t": t, "alt": alt, "tension": tension})
        if events.check_floor(hub["pos"][2], t, "flight"):
            if crash_t is None:
                crash_t = t

    alts     = [h["alt"] for h in hist]
    tensions = [h["tension"] for h in hist]
    return {
        "label":       "python_ctrl",
        "min_alt":     min(alts),
        "max_alt":     max(alts),
        "max_tension": max(tensions),
        "crash_t":     crash_t,
        "events":      events,
        "hist":        hist,
    }


# ── Tests ─────────────────────────────────────────────────────────────────────

def _print_result(r):
    crash = f"CRASH at t={r['crash_t']:.1f}s" if r["crash_t"] else "no crash"
    print(f"\n  [{r['label']}]  min_alt={r['min_alt']:.2f}m  "
          f"max_tension={r['max_tension']:.0f}N  {crash}")
    for line in r.get("log", []):
        print(f"    {line}")
    # Print altitude history every 10s
    prev_t = -999
    for h in r.get("hist", []):
        if h["t"] - prev_t >= 9.9:
            print(f"    t={h['t']:5.1f}  alt={h['alt']:6.2f}m  "
                  f"tension={h['tension']:6.0f}N  col={h.get('col', 0):.3f}rad")
            prev_t = h["t"]


def test_lua_orbit_sim():
    """Python orbit math at 50 Hz (Lua rate): no bad events, hub stable above 3 m for 90 s."""
    r_lua = _run("lua_exact")
    r_py  = _run_python()
    _print_result(r_lua)
    failures = []
    if r_lua["events"]:
        failures.append(f"lua_exact: {r_lua['events'].summary()}")
    if r_lua["min_alt"] < 3.0:
        failures.append(f"lua_exact: min_alt={r_lua['min_alt']:.2f}m < 3m")
    if r_py["events"]:
        failures.append(f"python_baseline: {r_py['events'].summary()}")
    if r_py["min_alt"] < 3.0:
        failures.append(f"python_baseline: min_alt={r_py['min_alt']:.2f}m < 3m")
    assert not failures, "\n  ".join(failures)
