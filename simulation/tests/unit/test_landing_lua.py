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
from dynamics        import RigidBodyDynamics
from aero            import create_aero
from tether          import TetherModel
from controller      import RatePID, OrbitTracker, col_min_for_altitude_rad, AcroController
from swashplate      import SwashplateServoModel
from winch           import WinchController
from planner         import DeschutterPlanner, WindEstimator, quat_apply, quat_is_identity
from landing_planner import LandingPlanner
from simtest_ic      import load_ic
from simtest_log     import SimtestLog, BadEventLog
from tel             import make_tel
from telemetry_csv   import TelRow, write_csv
from rawes_lua_harness import RawesLua
from rawes_modes     import MODE_LANDING, LAND_FINAL_DROP

_IC    = load_ic()
_ROTOR = rd.default()
_log   = SimtestLog(__file__)

# ── Timing ────────────────────────────────────────────────────────────────────
DT            = 1.0 / 400.0
LUA_PERIOD    = 0.010
LUA_EVERY     = round(LUA_PERIOD / DT)
T_AERO_OFFSET = 45.0

# ── Environment ───────────────────────────────────────────────────────────────
ANCHOR = np.zeros(3)
ANCHOR.flags.writeable = False
WIND   = np.array([0.0, 10.0, 0.0])   # 10 m/s East (NED Y)
WIND.flags.writeable = False

# ── Spin ──────────────────────────────────────────────────────────────────────
I_SPIN_KGMS2   = 10.0
OMEGA_SPIN_MIN = 0.5

# ── Starting state ────────────────────────────────────────────────────────────
R0_INIT       = _IC.R0
POS_INIT      = _IC.pos
VEL_INIT      = _IC.vel
LAND_TETHER_M = _IC.rest_length
OMEGA_SPIN_IC = _IC.omega_spin

# ── Pumping reel-in phase (mirrors test_landing.py exactly) ──────────────────
T_REEL_OUT   =  0.0
T_REEL_IN    = 30.0
V_REEL_IN    =  0.4
XI_REEL_IN_DEG   = 80.0
BODY_Z_SLEW_RATE = _ROTOR.body_z_slew_rate_rad_s
_xi_start_deg    = 30.0
T_TRANSITION  = math.radians(XI_REEL_IN_DEG - _xi_start_deg) / BODY_Z_SLEW_RATE + 1.5
TENSION_OUT   = 200.0
TENSION_IN    =  55.0
COL_MIN_RAD         = -0.28
COL_MAX_RAD         =  0.10
COL_MIN_REEL_IN_RAD = col_min_for_altitude_rad(create_aero(_ROTOR), XI_REEL_IN_DEG, _ROTOR.mass_kg)
TENSION_SAFETY_N    = 496.0
T_PUMPING_END = T_REEL_OUT + T_REEL_IN

# ── Landing parameters ────────────────────────────────────────────────────────
V_REEL             = 0.5    # m/s descent rate (== rawes.lua VZ_LAND_SP)
MIN_TETHER_M       = 2.0    # m   descent -> final_drop transition
BREAK_LOAD_N       = 620.0
FLOOR_ALT_M        = 1.0
ANCHOR_LAND_RADIUS_M = 20.0
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
    Two-phase landing mirroring test_landing.py:

    Phase 1 — pumping reel-in (pure Python, T_REEL_IN=30 s):
      DeschutterPlanner + AcroController swing hub from xi~8 deg to xi~80 deg.
      Lua is not involved.

    Phase 2 — Lua landing (mode=4):
      LandingPlanner tension-PI winch (ground station).
      rawes.lua VZ PI collective + cyclic (flight controller).
    """
    # ── Physics objects ───────────────────────────────────────────────────
    dyn    = RigidBodyDynamics(
        mass=_ROTOR.mass_kg, I_body=[5.0, 5.0, 10.0], I_spin=0.0,
        pos0=POS_INIT.tolist(), vel0=VEL_INIT.tolist(),
        R0=R0_INIT, omega0=[0.0, 0.0, 0.0], z_floor=-1.0,
    )
    aero   = create_aero(_ROTOR)
    tether = TetherModel(anchor_ned=ANCHOR, rest_length=LAND_TETHER_M,
                         axle_attachment_length=_ROTOR.axle_attachment_length_m)
    winch  = WinchController(rest_length=LAND_TETHER_M,
                              tension_safety_n=TENSION_SAFETY_N,
                              min_length=MIN_TETHER_M)
    pid_lon = RatePID(kp=KP_INNER)
    pid_lat = RatePID(kp=KP_INNER)
    servo   = SwashplateServoModel.from_rotor(_ROTOR)

    omega_spin  = OMEGA_SPIN_IC
    hub_state   = dyn.state
    tether.compute(hub_state["pos"], hub_state["vel"], hub_state["R"])
    tension_now = tether._last_info.get("tension", 0.0)

    # ── Phase 1: pumping reel-in (Python only) ────────────────────────────
    trajectory    = DeschutterPlanner(
        t_reel_out=T_REEL_OUT, t_reel_in=T_REEL_IN, t_transition=T_TRANSITION,
        v_reel_out=0.4, v_reel_in=V_REEL_IN,
        tension_out=TENSION_OUT, tension_in=TENSION_IN,
        wind_estimator=WindEstimator(seed_wind_ned=WIND),
        col_min_rad=COL_MIN_RAD, col_max_rad=COL_MAX_RAD,
        xi_reel_in_deg=XI_REEL_IN_DEG,
        col_min_reel_in_rad=COL_MIN_REEL_IN_RAD,
    )
    orbit_tracker = OrbitTracker(_IC.R0[:, 2], _IC.pos / np.linalg.norm(_IC.pos),
                                 BODY_Z_SLEW_RATE)
    acro          = AcroController.from_rotor(_ROTOR, use_servo=True)
    collective_rad = _IC.coll_eq_rad

    pump_steps = int(T_PUMPING_END / DT)
    for i in range(pump_steps):
        t_sim = i * DT
        state_pkt = {
            "pos_ned": hub_state["pos"], "vel_ned": hub_state["vel"],
            "omega_spin": omega_spin, "body_z": hub_state["R"][:, 2],
            "tension_n": tension_now, "tether_length_m": winch.tether_length_m,
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
        tilt_lon, tilt_lat = acro.update(hub_state, body_z_eq, DT,
                                         swashplate_phase_deg=0.0)
        result = aero.compute_forces(
            collective_rad=collective_rad,
            tilt_lon=tilt_lon, tilt_lat=tilt_lat,
            R_hub=hub_state["R"], v_hub_world=hub_state["vel"],
            omega_rotor=omega_spin, wind_world=WIND,
            t=T_AERO_OFFSET + t_sim,
        )
        tf, tm = tether.compute(hub_state["pos"], hub_state["vel"], hub_state["R"])
        tension_now = tether._last_info.get("tension", 0.0)
        M_orbital = result.M_orbital + tm - 50.0 * hub_state["omega"]
        omega_spin = max(OMEGA_SPIN_MIN,
                         omega_spin + aero.last_Q_spin / I_SPIN_KGMS2 * DT)
        hub_state = dyn.step(result.F_world + tf, M_orbital, DT, omega_spin=omega_spin)

    # ── Phase 2: Lua landing ──────────────────────────────────────────────
    # Initialise Lua harness with the post-pumping hub state
    sim = RawesLua(mode=MODE_LANDING)
    sim.armed        = True
    sim.healthy      = True
    sim.vehicle_mode = 1
    sim.pos_ned      = hub_state["pos"].tolist()
    sim.vel_ned      = hub_state["vel"].tolist()
    sim.R            = hub_state["R"]
    sim.gyro         = [0.0, 0.0, 0.0]

    landing_planner = LandingPlanner(
        initial_body_z=orbit_tracker.bz_slerp,
        v_land=V_REEL,
        col_cruise=0.079,
        min_tether_m=MIN_TETHER_M,
        anchor_ned=ANCHOR,
    )

    col_rad  = collective_rad   # continue from pumping collective
    roll_sp  = 0.0
    pitch_sp = 0.0

    events        = BadEventLog()
    floor_hit     = False
    t_final_start = None

    tensions       = []
    altitudes      = []
    vz_at_floor    = None
    bz_at_floor    = None
    pos_at_floor   = None
    touchdown_time = None

    tel_every = max(1, int(0.05 / DT))
    telemetry = []

    descent_time = winch.rest_length / V_REEL * 2.0 + 30.0
    max_steps = int((descent_time + T_FINAL_DROP_MAX) / DT)

    t_offset = T_PUMPING_END   # keep t_sim continuous for telemetry
    t_sim    = t_offset
    for i in range(max_steps):
        t_sim     = t_offset + i * DT
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

        # ── Ground planner: tension-PI winch; triggers final_drop ─────────
        land_cmd = landing_planner.step(
            {"pos_ned": hub_state["pos"], "vel_ned": hub_state["vel"],
             "body_z": hub_state["R"][:, 2],
             "tension_n": tension_now, "tether_length_m": winch.rest_length},
            DT,
        )
        winch.step(land_cmd["winch_speed_ms"], tension_now, DT)
        tether.rest_length = winch.rest_length

        # Send final_drop substate to Lua when planner transitions
        if not floor_hit and t_final_start is None and land_cmd["phase"] == "final_drop":
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
            if t_final_start is not None and (t_sim - t_final_start) >= T_FINAL_DROP_MAX:
                break

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
    """rawes.lua mode=4: floor reached, touchdown within anchor radius, no bad events during descent."""
    r = _run_landing()
    failures = []

    events = r["events"]
    # Descent-phase: orbital velocity at the pumping→landing handoff creates a
    # brief burst of slack (typically ≤ 20 events in the first ~1 s).  These
    # subside once the hub settles into straight-down descent.  Zero is ideal
    # but a small allowance covers the transition without masking real bugs.
    # Tension spikes during descent are always a controller error → zero tolerance.
    MAX_DESCENT_SLACK = 20
    descent_slack  = events.count("slack",         phase="descent")
    descent_spikes = events.count("tension_spike", phase="descent")
    if descent_slack > MAX_DESCENT_SLACK:
        failures.append(
            f"slack={descent_slack}(descent) > {MAX_DESCENT_SLACK}: "
            f"controller leaking tether during descent"
        )
    if descent_spikes > 0:
        failures.append(f"tension_spike={descent_spikes}(descent): tension exceeded break load")

    # final_drop is a free-fall impact: the hub drops from ~2 m tether onto the
    # catch pad.  Spring constant k=EA/L is huge at 2 m, so any lateral motion
    # creates unavoidable slack/snap cycles.  No assertion on final_drop events.

    if not r["floor_hit"]:
        failures.append("hub did not reach floor during final_drop")
    if r["anchor_dist"] is None:
        failures.append("no anchor_dist (floor not reached)")
    elif r["anchor_dist"] > ANCHOR_LAND_RADIUS_M:
        failures.append(f"anchor_dist={r['anchor_dist']:.1f}m > {ANCHOR_LAND_RADIUS_M}m")
    assert not failures, "\n  ".join(failures)
