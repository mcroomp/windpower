"""
test_pump_cycle_lua.py -- Lua pumping cycle using UnifiedGroundController + LuaComms.

Mirrors test_pump_cycle_unified.py exactly, replacing TensionApController with
rawes.lua (mode=5) running in-process via lupa.

Division of labour (mirrors real stack):
  Ground (Python, 10 Hz): UnifiedGroundController → PumpingGroundController +
                          WinchController; delivers TensionCommand to Lua via
                          LuaComms (send_named_float: RAWES_TSP, RAWES_TEN,
                          RAWES_ALT, RAWES_SUB).
  AP     (Lua, 100 Hz):   rawes.lua mode=5 — TensionPI collective (steps on
                          RAWES_TSP) + bz_altitude_hold cyclic.
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

pytestmark = [pytest.mark.simtest, pytest.mark.timeout(600)]

import rotor_definition as rd
from controller     import RatePID
from swashplate     import SwashplateServoModel
from winch          import WinchController
from simtest_ic     import load_ic
from simtest_log    import SimtestLog, BadEventLog
from simtest_runner import PhysicsRunner, feed_obs
from telemetry_csv  import TelRow, write_csv
from pumping_planner import PumpingGroundController
from rawes_lua_harness import RawesLua
from rawes_modes    import MODE_PUMPING
from unified_ground import UnifiedGroundController, LuaComms

_IC    = load_ic()
_ROTOR = rd.default()
_log   = SimtestLog(__file__)

# ── Simulation constants ──────────────────────────────────────────────────────
DT         = 1.0 / 400.0
DT_PLANNER = 1.0 / 10.0
LUA_PERIOD = 0.020           # 50 Hz Lua tick (FLIGHT_PERIOD_MS=20)
LUA_EVERY  = round(LUA_PERIOD / DT)
WIND       = np.array([0.0, 10.0, 0.0])
WIND.flags.writeable = False
BREAK_LOAD_N = 620.0

_COL_MIN    = -0.28
_COL_MAX    =  0.10
_ACRO_SCALE = 500.0 / (360.0 * math.pi / 180.0)

# ── Pumping parameters (mirror test_pump_cycle_unified.py exactly) ────────────
N_CYCLES         = 3
DELTA_L          = 12.0

_XI_START_DEG    = 30.0
_XI_REEL_IN_DEG  = 50.0
T_TRANSITION = (
    math.radians(_XI_REEL_IN_DEG - _XI_START_DEG) / _ROTOR.body_z_slew_rate_rad_s + 3.0
)

TENSION_OUT      = 435.0
TENSION_IN       = 240.0
TENSION_IN_AP    = 213.0
TENSION_IC       = 300.0

EL_REEL_IN_RAD   = math.radians(_XI_REEL_IN_DEG)
TENSION_SAFETY_N = 496.0
FLOOR_ALT_M      = 1.0

T_REEL_OUT_MAX = 120.0
T_REEL_IN_MAX  = 120.0
T_END_SIM      = N_CYCLES * (T_REEL_OUT_MAX + T_TRANSITION + T_REEL_IN_MAX) * 1.2


def _pwm_to_rate(pwm: int) -> float:
    return (pwm - 1500) / _ACRO_SCALE


def _pwm_to_col(pwm: int) -> float:
    return _COL_MIN + (pwm - 1000) / 1000.0 * (_COL_MAX - _COL_MIN)


def _run_pumping() -> dict:
    # ── Lua AP ───────────────────────────────────────────────────────────────
    sim = RawesLua(mode=MODE_PUMPING)
    sim.armed        = True
    sim.healthy      = True
    sim.vehicle_mode = 1
    sim.pos_ned      = _IC.pos.tolist()
    sim.vel_ned      = _IC.vel.tolist()
    sim.R            = _IC.R0
    sim.gyro         = [0.0, 0.0, 0.0]

    # ── Physics ───────────────────────────────────────────────────────────────
    runner = PhysicsRunner(_ROTOR, _IC, WIND)

    # ── Ground: UnifiedGroundController with LuaComms ─────────────────────────
    ground = PumpingGroundController(
        t_transition   = T_TRANSITION,
        target_alt_m   = float(-_IC.pos[2]),
        delta_l        = DELTA_L,
        el_reel_in_rad = EL_REEL_IN_RAD,
        n_cycles       = N_CYCLES,
        tension_out    = TENSION_OUT,
        tension_in     = TENSION_IN,
        tension_in_ap  = TENSION_IN_AP,
        tension_ic     = TENSION_IC,
        tension_ramp_s = 8.0,
        t_reel_out_max = T_REEL_OUT_MAX,
        t_reel_in_max  = T_REEL_IN_MAX,
    )
    winch = WinchController(
        rest_length     = _IC.rest_length,
        kp_tension      = 0.005,
        v_max_out       = 0.40,
        v_max_in        = 0.80,
        accel_limit_ms2 = 0.05,
        min_length      = 2.0,
    )
    ground_ctrl = UnifiedGroundController(
        ground  = ground,
        winch   = winch,
        comms   = LuaComms(sim.send_named_float),
        dt_plan = DT_PLANNER,
    )

    # ── Inner loop (mirrors AcroControllerSitl) ───────────────────────────────
    pid_lon = RatePID(kp=RatePID.DEFAULT_KP)
    pid_lat = RatePID(kp=RatePID.DEFAULT_KP)
    servo   = SwashplateServoModel.from_rotor(_ROTOR)

    col_rad  = _IC.coll_eq_rad
    roll_sp  = 0.0
    pitch_sp = 0.0

    events   = BadEventLog()
    telemetry = []
    tel_every = max(1, int(0.05 / DT))

    cycle_energy_out = [0.0] * N_CYCLES
    cycle_energy_in  = [0.0] * N_CYCLES

    max_steps = int(T_END_SIM / DT) + 1
    t_sim = 0.0

    for i in range(max_steps):
        t_sim = i * DT
        if ground_ctrl.phase == "hold":
            break

        hub_state   = runner.hub_state
        tension_now = runner.tension_now
        altitude    = -hub_state["pos"][2]
        cycle_idx   = min(ground_ctrl.cycle_count, N_CYCLES - 1)

        # ── Ground 10 Hz (internal to UnifiedGroundController) ────────────────
        ground_ctrl.step(t_sim, tension_now, altitude, DT)

        # ── Lua 50 Hz ─────────────────────────────────────────────────────────
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

        # ── Inner rate loop 400 Hz ────────────────────────────────────────────
        omega_body    = hub_state["R"].T @ hub_state["omega"]
        omega_body[2] = 0.0
        tilt_lon_cmd  =  pid_lon.update(roll_sp,  omega_body[0], DT)
        tilt_lat_cmd  = -pid_lat.update(pitch_sp, omega_body[1], DT)
        tilt_lon, tilt_lat = servo.step(tilt_lon_cmd, tilt_lat_cmd, DT)

        sr = runner.step_raw(DT, col_rad, tilt_lon, tilt_lat,
                             rest_length=ground_ctrl.rest_length)

        # ── Energy accounting ─────────────────────────────────────────────────
        speed_now = ground_ctrl.winch_speed_ms
        if speed_now > 0.0:
            cycle_energy_out[cycle_idx] += runner.tension_now * speed_now * DT
        elif speed_now < 0.0:
            cycle_energy_in[cycle_idx]  += runner.tension_now * abs(speed_now) * DT

        # ── Bad events ────────────────────────────────────────────────────────
        phase_label = f"cycle{cycle_idx+1}_{ground_ctrl.phase}"
        if runner.tether._last_info.get("slack", False):
            events.record("slack", t_sim, phase_label, altitude,
                          tension=runner.tension_now)
        if runner.tension_now > BREAK_LOAD_N:
            events.record("tension_spike", t_sim, phase_label, altitude,
                          tension=runner.tension_now)
        if runner.hub_state["pos"][2] >= -FLOOR_ALT_M:
            events.record("floor_hit", t_sim, phase_label, altitude)

        # ── Telemetry 20 Hz ───────────────────────────────────────────────────
        if i % tel_every == 0:
            telemetry.append(TelRow.from_physics(
                runner, sr, col_rad, WIND,
                body_z_eq = None,
                phase     = phase_label,
            ))

    net_per_cycle = [cycle_energy_out[k] - cycle_energy_in[k] for k in range(N_CYCLES)]
    total_net     = sum(net_per_cycle)

    cycle_summary = "  ".join(
        f"c{k+1}={net_per_cycle[k]:.0f}J" for k in range(N_CYCLES)
    )
    lua_msgs = sim.messages
    parts = [
        f"total_net={total_net:.0f}J",
        cycle_summary,
        f"t_end={t_sim:.1f}s",
        events.summary(),
    ]
    for level, msg in lua_msgs[:10]:
        parts.append(f"  [GCS {level}] {msg}")

    if telemetry:
        write_csv(telemetry, _log.log_dir / "telemetry.csv")
    _log.write(["  ".join(p for p in parts if p)], "lua_pumping_unified")

    return dict(
        t_end            = t_sim,
        cycle_energy_out = cycle_energy_out,
        cycle_energy_in  = cycle_energy_in,
        net_per_cycle    = net_per_cycle,
        total_net        = total_net,
        events           = events,
        floor_hits       = events.count("floor_hit"),
        slack_events     = events.count("slack"),
        tension_spikes   = events.count("tension_spike"),
        messages         = lua_msgs,
        telemetry        = telemetry,
    )


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_lua_pumping_constants():
    """rawes.lua mode=5 constants match Python-side expectations (fast, no simulation)."""
    sim = RawesLua(mode=MODE_PUMPING)
    f   = sim.fns
    # NV names are ≤ 10 chars (MAVLink hard limit)
    from unified_ground import _cmd_to_nv
    from pumping_planner import TensionCommand
    dummy = TensionCommand(
        tension_setpoint_n=435.0,
        tension_measured_n=300.0,
        alt_m=38.0,
        phase="reel-out",
    )
    for name, _ in _cmd_to_nv(dummy):
        assert len(name) <= 10, f"NV name '{name}' exceeds 10-char MAVLink limit"

    # TensionPI gains must match Python TensionApController defaults
    assert float(f.KP_TEN)      == pytest.approx(2e-4,  rel=1e-3)
    assert float(f.KI_TEN)      == pytest.approx(1e-3,  rel=1e-3)
    assert float(f.COL_MAX_TEN) == pytest.approx(0.0,   abs=1e-9)
    assert float(f.COL_REEL_OUT) == pytest.approx(-0.20, rel=1e-3)

    # Integrator warm-starts so initial output = COL_REEL_OUT at zero error
    kp, ki = float(f.KP_TEN), float(f.KI_TEN)
    col_reel_out = float(f.COL_REEL_OUT)
    i_init = col_reel_out / max(ki, 1e-12)
    col_init = kp * 0.0 + ki * i_init   # zero error at warm-start
    assert col_init == pytest.approx(col_reel_out, rel=1e-3)

    # Collective limits encompass TensionPI range
    assert float(f.COL_MIN_RAD) <  float(f.COL_MAX_TEN)
    assert float(f.COL_MAX_TEN) <= float(f.COL_MAX_RAD)

    # Initial state: no fresh command, held at COL_REEL_OUT, setpoint warm at 300 N
    assert not sim.fns.ten_sp_fresh()
    assert float(sim.fns.col_held())    == pytest.approx(col_reel_out, rel=1e-3)
    assert float(sim.fns.ten_setpoint()) == pytest.approx(300.0, rel=1e-3)


def test_lua_pumping_unified():
    """rawes.lua mode=5 via UnifiedGroundController/LuaComms: no bad events, net > 0."""
    r = _run_pumping()
    failures = []
    if r["events"]:
        failures.append(r["events"].summary())
    for k, net in enumerate(r["net_per_cycle"]):
        if net <= 0:
            failures.append(f"cycle {k+1} net={net:.1f}J <= 0")
    if r["total_net"] <= 0:
        failures.append(f"total_net={r['total_net']:.1f}J <= 0")
    assert not failures, "\n  ".join(failures)
