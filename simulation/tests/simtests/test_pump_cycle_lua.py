"""
test_pump_cycle_lua.py -- Lua pumping cycle, mirror of test_pump_cycle_unified.py.

This drives rawes.lua (steady mode, in-process via lupa) with the SAME ground/winch
harness as test_pump_cycle_unified.py: an inline 2-phase (reel_out/reel_in) state
machine, a GovernedWinchController (tension-governed, jerk-limited), and a 300/100 N
per-phase tension target. The only difference from the unified test is that the
kite-side controller is the real Lua flight code instead of the Python equivalent.

Division of labour (mirrors real stack):
  Ground (Python, 10 Hz): inline reel_out/reel_in state machine; commands the
                          GovernedWinchController and sends RAWES_ALT/RAWES_TEN/
                          RAWES_SUB to Lua via LuaComms (send_named_float).
  Winch  (400 Hz):        GovernedWinchController -- +/-V_CRUISE cruise with a
                          proportional tension governor, acceleration- and
                          jerk-limited for smooth motion.
  AP     (Lua, 50 Hz):    rawes.lua steady mode -- altitude PID collective plus
                          rate-only bz_altitude_hold cyclic. The kite derives the
                          downwind-plane azimuth from its own position (no truth-
                          wind oracle), per the AGENTS.md no-truth-wind invariant.
"""
import os
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

pytestmark = [pytest.mark.simtest,
              pytest.mark.timeout(int(os.environ.get("RAWES_PUMP_TIMEOUT", "600")))]

from winch          import GovernedWinchController
from winch_node     import GovernedWinchNode, WinchCommand, Anemometer
from simtest_ic     import load_ic
from simtest_log    import BadEventLog
from simtest_runner import PhysicsRunner
from tests.common.mock_ardupilot import MockArdupilot
from pumping_planner import PumpingGroundController
from rawes_lua_harness import RawesLua
from rawes_modes    import MODE_STEADY
from unified_ground import LuaComms
from tests.simtests._rotor_helpers import load_default_rotor

_IC    = load_ic()
_ROTOR = load_default_rotor()

# ── Simulation constants ──────────────────────────────────────────────────────
DT         = 1.0 / 400.0
DT_PLANNER = 1.0 / 10.0
LUA_PERIOD = 0.020           # 50 Hz Lua tick (FLIGHT_PERIOD_MS=20)
LUA_EVERY  = round(LUA_PERIOD / DT)
WIND       = np.array([0.0, 10.0, 0.0])
WIND.flags.writeable = False
BREAK_LOAD_N = 620.0

# ── Pumping cycle parameters (mirror test_pump_cycle_unified.py) ──────────────
N_CYCLES     = 3
DELTA_L      = 12.0    # tether payout per cycle [m]
V_CRUISE_OUT = 0.5     # nominal reel-out cruise velocity [m/s]
V_CRUISE_IN  = 0.5     # nominal reel-in cruise speed [m/s] (applied as -V_CRUISE_IN)

# Governor headroom: cruise is +/-0.5 m/s; the tension governor may push speed up
# to these bounds to shed an over-tension transient (reel-out) or pull in slack
# (reel-in). Keep above the cruise so the governor has authority.
V_MAX_OUT    = 1.5     # max pay-out speed [m/s]
V_MAX_IN     = 1.5     # max reel-in speed [m/s]
KP_TENSION   = 4.0e-4  # governor gain [(m/s)/N] -- ~0.5 s tension settling
ACCEL_MS2    = 2.0     # acceleration limit [m/s^2]
JERK_MS3     = 10.0    # jerk limit [m/s^3] (S-curve smoothing)
TENSION_TAU  = 0.08    # load-cell low-pass time constant [s]

TENSION_IC       = 300.0   # IC equilibrium tension
TENSION_REEL_OUT = TENSION_IC
TENSION_REEL_IN  = 100.0
FLOOR_ALT_M      = 1.0
CAPTURE_SETTLE_S = 2.0

# Safety timeouts
T_REEL_OUT_MAX = 120.0
T_REEL_IN_MAX  = 300.0
T_END_SIM      = N_CYCLES * (T_REEL_OUT_MAX + T_REEL_IN_MAX) * 1.2


def _run_pumping(log, aero_model: "str | None" = None) -> dict:
    # Aero model: explicit arg wins; otherwise the RAWES_AERO env var
    # (quasi_static [default] / pitt_peters / oye / vpm) selects it.
    if aero_model is None:
        aero_model = os.environ.get("RAWES_AERO", "quasi_static")
    # ── Lua AP ───────────────────────────────────────────────────────────────
    sim = RawesLua(mode=MODE_STEADY)
    sim.armed        = True
    sim.healthy      = True
    sim.vehicle_mode = 4   # GUIDED
    sim.pos_ned      = _IC.pos.tolist()
    sim.vel_ned      = _IC.vel.tolist()
    sim.R            = _IC.R0
    sim.gyro         = [0.0, 0.0, 0.0]
    # rawes.lua now gates steady capture on anchor NVFs. In this simtest path
    # the anchor is at NED origin, so publish zeros once at startup.
    sim.send_named_float("RAWES_ANN", 0.0)
    sim.send_named_float("RAWES_ANE", 0.0)
    sim.send_named_float("RAWES_AND", 0.0)

    # ── Physics ───────────────────────────────────────────────────────────────
    runner = PhysicsRunner(_ROTOR, _IC, WIND, aero_model=aero_model, col_min_rad=-0.28, col_max_rad=0.10)
    ic_alt = float(-_IC.pos[2])

    # ── Winch node: hosts the fast GovernedWinchController loop + load cell + ──
    # anemometer.  The planner reaches it ONLY through the cable boundary
    # node.exchange(WinchCommand) -> WinchTelemetry (targets down, sensed up).
    gov = GovernedWinchController(
        rest_length     = _IC.rest_length,
        v_max_out       = V_MAX_OUT,
        v_max_in        = V_MAX_IN,
        kp_tension      = KP_TENSION,
        accel_limit_ms2 = ACCEL_MS2,
        jerk_limit_ms3  = JERK_MS3,
        tension_tau_s   = TENSION_TAU,
        min_length      = 2.0,
    )
    node = GovernedWinchNode(gov, Anemometer())

    # ── Ground -> Lua command channel (NAMED_VALUE_FLOAT) ─────────────────────
    comms = LuaComms(sim.send_named_float)

    lua = MockArdupilot.for_lua(sim, initial_col_rad=_IC.coll_eq_rad, wind=WIND, dt=DT)

    events = BadEventLog()

    # ── Ground controller (shared PumpingGroundController -- same as SITL) ────
    # Two-phase, length-driven, capture-gated, constant IC altitude.  Starts in
    # 'hold' and begins pumping only after Lua reports steady capture + settle.
    planner = PumpingGroundController(
        target_alt_m     = ic_alt,
        delta_l          = DELTA_L,
        n_cycles         = N_CYCLES,
        tension_out      = TENSION_REEL_OUT,
        tension_in       = TENSION_REEL_IN,
        tension_ic       = TENSION_IC,
        v_cruise_out     = V_CRUISE_OUT,
        v_cruise_in      = V_CRUISE_IN,
        t_reel_out_max   = T_REEL_OUT_MAX,
        t_reel_in_max    = T_REEL_IN_MAX,
        capture_settle_s = CAPTURE_SETTLE_S,
    )

    cycle_net_start = [0.0] * N_CYCLES
    phase_label     = "pre_capture_hold"
    captured        = False

    lua.tel_fn = lambda r, sr: dict(
        body_z_eq                = None,
        phase                    = phase_label,
        winch_speed_ms           = gov.speed_ms,
        tension_feedforward_n    = planner.winch_target_tension,
        collective_from_alt_ctrl = lua.col_rad,
        alt_pid_integral         = sim.fns.alt_i(),
        gnd_alt_cmd_m            = ic_alt,
        elevation_rad            = 0.0,
        el_correction_rad        = 0.0,
        coll_saturated           = 0,
        comms_ok                 = 1,
    )

    max_steps      = int(T_END_SIM / DT) + 1
    planner_every  = max(1, round(DT_PLANNER / DT))
    prev_accel_ned = None   # one-step lagged IMU specific force for vibration damper
    prev_phase     = planner.phase

    # Seed the cable: an initial hold exchange gives the planner its first
    # telemetry frame (the only winch data it is ever allowed to see).
    node.update_sensors(runner.tension_now, WIND)
    tel = node.exchange(WinchCommand(cruise_v=0.0, tension_target=TENSION_IC))

    # Initial command so Lua's _target_alt / _tension_n are set before flight.
    comms.send(planner.step(0.0, tel.tension_n, tel.rest_length), DT_PLANNER)

    # ── Inflow warm-up ────────────────────────────────────────────────────
    runner._core.warm_inflow(collective_rad=float(_IC.coll_eq_rad), n_steps=500)

    t_sim = 0.0
    for i in range(max_steps):
        t_sim       = i * DT
        tension_now = runner.tension_now
        altitude    = runner.altitude

        # Winch node feels its load cell + anemometer (physics side).
        node.update_sensors(tension_now, WIND)

        # ── Detect Lua steady capture and gate the planner ───────────────
        if not captured:
            if any("RAWES steady: captured" in msg for _, msg in sim.messages):
                captured = True
                planner.notify_captured(t_sim)

        # ── Ground 10 Hz: step planner, exchange over the cable, cmd Lua ───
        if i % planner_every == 0:
            # Planner sees ONLY the last telemetry frame from the winch node.
            cmd = planner.step(t_sim, tel.tension_n, tel.rest_length)
            tel = node.exchange(WinchCommand(
                cruise_v       = planner.winch_target_velocity,
                tension_target = planner.winch_target_tension,
            ))
            comms.send(cmd, DT_PLANNER)

        # Telemetry label + per-cycle net-energy bookkeeping
        if planner.phase != prev_phase and planner.phase == "reel-out":
            idx = min(planner.cycle_count, N_CYCLES - 1)
            cycle_net_start[idx] = gov.net_energy_j
        prev_phase = planner.phase

        if captured:
            phase_label = f"cycle{min(planner.cycle_count + 1, N_CYCLES)}_{planner.phase}"
        else:
            phase_label = "pre_capture_hold"

        # All cycles complete -> planner returns to hold.
        if planner.cycle_count >= N_CYCLES:
            break

        # ── Lua 50 Hz ─────────────────────────────────────────────────────
        if i % LUA_EVERY == 0:
            lua.tick(t_sim, runner, accel_ned=prev_accel_ned)

        # ── Winch node fast loop 400 Hz ────────────────────────────
        node.step(DT)

        # ── Inner physics step driven by GuidedAttitudeController ────────────
        sr = lua.step(runner, DT, rest_length=node.rest_length)
        prev_accel_ned = sr.get("accel_specific_world")

        # ── Safety events ─────────────────────────────────────────────────
        if runner.tether._last_info.get("slack", False):
            events.record("slack", t_sim, phase_label, altitude,
                          tension=runner.tension_now)
        if captured and planner.phase != "hold" and runner.tension_now > BREAK_LOAD_N:
            events.record("tension_spike", t_sim, phase_label, altitude,
                          tension=runner.tension_now)
        if runner.hub_state["pos"][2] >= -FLOOR_ALT_M:
            events.record("floor_hit", t_sim, phase_label, altitude)
            break  # kite on ground — simulation over

        # ── Telemetry 20 Hz ───────────────────────────────────────────────
        lua.log(runner, sr)

    # ── Results ───────────────────────────────────────────────────────────────
    net_per_cycle = [gov.net_energy_j - cycle_net_start[k] for k in range(N_CYCLES)]
    total_net     = gov.net_energy_j

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
    """rawes.lua pumping constants match Python-side expectations (fast, no simulation)."""
    sim = RawesLua(mode=MODE_STEADY)
    f   = sim.fns
    # NV names are ≤ 10 chars (MAVLink hard limit)
    from unified_ground import _cmd_to_nv
    from pumping_planner import TensionCommand
    dummy = TensionCommand(
        tension_target_n=300.0,
        alt_m=38.0,
        phase="reel-out",
    )
    for name, _ in _cmd_to_nv(dummy):
        assert len(name) <= 10, f"NV name '{name}' exceeds 10-char MAVLink limit"

    # Lua pumping now matches Python-mode altitude PID + rate-only body_z path.
    assert float(f.KP_ALT)                == pytest.approx(0.010, rel=1e-3)
    assert float(f.KI_ALT)                == pytest.approx(0.001, rel=1e-3)
    assert float(f.KD_VZ)                 == pytest.approx(0.040, rel=1e-3)
    assert float(f.KP_EL)                 == pytest.approx(2.5, rel=1e-3)
    assert float(f.RATE_ACCEL_MAX_RADSS)  == pytest.approx(4.0, rel=1e-3)


def test_lua_pumping_unified(simtest_log):
    """3 pumping cycles with rawes.lua (steady mode) + GovernedWinchController.
    Pass if no bad events (no tension spike, slack, or floor hit) -- mirrors
    test_pumping_unified."""
    r = _run_pumping(simtest_log)
    failures = []
    if r["events"]:
        failures.append(r["events"].summary())
    assert not failures, "\n  ".join(failures)


@pytest.mark.skip(reason="Dynamic-inflow comparison only; quasi_static is the default flight model")
def test_lua_pumping_unified_peters_he(simtest_log):
    """Same as test_lua_pumping_unified but with Pitt-Peters aero (no xi limit)."""
    r = _run_pumping(simtest_log, aero_model="pitt_peters")
    failures = []
    if r["events"]:
        failures.append(r["events"].summary())
    assert not failures, "\n  ".join(failures)
