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
from winch          import WinchController
from simtest_ic     import load_ic
from simtest_log    import BadEventLog
from simtest_runner import PhysicsRunner, LuaAP
from pumping_planner import PumpingGroundController
from rawes_lua_harness import RawesLua
from rawes_modes    import MODE_PUMPING
from unified_ground import UnifiedGroundController, LuaComms

_IC    = load_ic()
_ROTOR = rd.default()

# ── Simulation constants ──────────────────────────────────────────────────────
DT         = 1.0 / 400.0
DT_PLANNER = 1.0 / 10.0
LUA_PERIOD = 0.020           # 50 Hz Lua tick (FLIGHT_PERIOD_MS=20)
LUA_EVERY  = round(LUA_PERIOD / DT)
WIND       = np.array([0.0, 10.0, 0.0])
WIND.flags.writeable = False
BREAK_LOAD_N = 620.0

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


def _run_pumping(log, aero_model: str = "skewed_wake") -> dict:
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
    runner = PhysicsRunner(_ROTOR, _IC, WIND, aero_model=aero_model, col_min_rad=-0.28, col_max_rad=0.10)

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

    lua = LuaAP(sim, initial_col_rad=_IC.coll_eq_rad, wind=WIND, dt=DT)
    lua.tel_fn = lambda r, sr: dict(
        body_z_eq                    = None,
        phase                        = phase_label,
        winch_speed_ms               = ground_ctrl.winch_speed_ms,
        tension_setpoint             = sim.fns.ten_setpoint(),
        collective_from_tension_ctrl = sim.fns.col_held(),
        vib_corr                     = sim.fns.vib_corr_last(),
        ten_pi_integral              = sim.fns.col_i_ten(),
        elevation_rad                = 0.0,
        el_correction_rad            = 0.0,
        coll_saturated               = 0,
        comms_ok                     = 1,
    )

    events         = BadEventLog()
    cycle_net_start = [0.0] * N_CYCLES
    prev_cycle_idx  = -1

    max_steps      = int(T_END_SIM / DT) + 1
    t_sim          = 0.0
    prev_accel_ned = None   # one-step lagged IMU specific force for vibration damper

    for i in range(max_steps):
        t_sim = i * DT
        if ground_ctrl.phase == "hold":
            break

        tension_now = runner.tension_now
        altitude    = runner.altitude
        cycle_idx   = min(ground_ctrl.cycle_count, N_CYCLES - 1)

        # ── Ground 10 Hz (internal to UnifiedGroundController) ────────────────
        ground_ctrl.step(t_sim, tension_now, altitude, DT)

        # ── Lua 50 Hz ─────────────────────────────────────────────────────────
        if i % LUA_EVERY == 0:
            lua.tick(t_sim, runner, accel_ned=prev_accel_ned)

        # ── Inner rate loop 400 Hz ────────────────────────────────────────────
        omega_body    = runner.omega_body
        omega_body[2] = 0.0
        sr = runner.step(DT, lua.col_rad, lua.roll_sp, lua.pitch_sp, omega_body,
                         rest_length=ground_ctrl.rest_length)
        prev_accel_ned = sr.get("accel_specific_world")

        # ── Bad events ────────────────────────────────────────────────────────
        phase_label = f"cycle{cycle_idx+1}_{ground_ctrl.phase}"
        if cycle_idx != prev_cycle_idx:
            cycle_net_start[cycle_idx] = ground_ctrl.net_energy_j
            prev_cycle_idx = cycle_idx
        if runner.tether._last_info.get("slack", False):
            events.record("slack", t_sim, phase_label, altitude,
                          tension=runner.tension_now)
        if runner.tension_now > BREAK_LOAD_N:
            events.record("tension_spike", t_sim, phase_label, altitude,
                          tension=runner.tension_now)
        if runner.hub_state["pos"][2] >= -FLOOR_ALT_M:
            events.record("floor_hit", t_sim, phase_label, altitude)

        # ── Telemetry 20 Hz ───────────────────────────────────────────────────
        lua.log(runner, sr)

    net_per_cycle = [ground_ctrl.net_energy_j - cycle_net_start[k] for k in range(N_CYCLES)]
    total_net     = ground_ctrl.net_energy_j

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

    lua.write_telemetry(log.log_dir / "telemetry.csv")
    log.write(["  ".join(p for p in parts if p)], "lua_pumping_unified")

    return dict(
        t_end         = t_sim,
        net_per_cycle = net_per_cycle,
        total_net     = total_net,
        events        = events,
        floor_hits    = events.count("floor_hit"),
        slack_events  = events.count("slack"),
        tension_spikes= events.count("tension_spike"),
        messages      = lua_msgs,
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


def test_lua_pumping_unified(simtest_log):
    """rawes.lua mode=5 via UnifiedGroundController/LuaComms: no bad events, net > 0."""
    r = _run_pumping(simtest_log)
    failures = []
    if r["events"]:
        failures.append(r["events"].summary())
    for k, net in enumerate(r["net_per_cycle"]):
        if net <= 0:
            failures.append(f"cycle {k+1} net={net:.1f}J <= 0")
    if r["total_net"] <= 0:
        failures.append(f"total_net={r['total_net']:.1f}J <= 0")
    assert not failures, "\n  ".join(failures)


def test_lua_pumping_unified_peters_he(simtest_log):
    """rawes.lua mode=5 with Peters-He dynamic inflow: vibration damper suppresses
    tether spring resonance (~5 Hz) that the skewed-wake model does not excite."""
    r = _run_pumping(simtest_log, aero_model="peters_he")
    failures = []
    if r["events"]:
        failures.append(r["events"].summary())
    for k, net in enumerate(r["net_per_cycle"]):
        if net <= 0:
            failures.append(f"cycle {k+1} net={net:.1f}J <= 0")
    if r["total_net"] <= 0:
        failures.append(f"total_net={r['total_net']:.1f}J <= 0")
    assert not failures, "\n  ".join(failures)
