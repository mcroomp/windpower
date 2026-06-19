"""
test_pump_cycle_unified.py -- Pumping cycle with VelocityWinchController.

Architecture:
    Ground (10 Hz): simple reel_out/reel_in state machine.
                                    Sends altitude target and target tension feed-forward.
  Winch  (400 Hz): VelocityWinchController -- velocity-commanded with soft tension limits.
                   +V_REEL_OUT during reel_out, -V_REEL_IN during reel_in.
                   Soft zone brakes as tension approaches T_MIN or T_MAX.
    AP     (400 Hz): MockArdupilot Python equivalent -- altitude PID collective, rate-only body_z control.

This test validates stable winch cycles without tension spikes, slack, or floor hits.
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

pytestmark = [pytest.mark.simtest, pytest.mark.timeout(600)]

from winch            import VelocityWinchController
from simtest_log      import BadEventLog
from simtest_ic       import load_ic
from simtest_runner   import PhysicsRunner
from tests.common.mock_ardupilot import MockArdupilot
from pumping_planner  import TensionCommand
from comms            import VirtualComms
from tests.simtests._rotor_helpers import load_default_rotor, BODY_Z_SLEW_RATE_RAD_S

_IC    = load_ic()
_ROTOR = load_default_rotor()

# ── Simulation constants ──────────────────────────────────────────────────────
DT         = 1.0 / 400.0
DT_PLANNER = 1.0 / 10.0
WIND       = np.array([0.0, 10.0, 0.0])
WIND.flags.writeable = False
BREAK_LOAD_N = 620.0

# ── Pumping cycle parameters ──────────────────────────────────────────────────
N_CYCLES     = 3
DELTA_L      = 12.0    # tether payout per cycle [m]
V_REEL_OUT   = 0.5     # reel-out velocity [m/s]
V_REEL_IN    = 0.5     # reel-in speed [m/s] (applied as -V_REEL_IN)

# Winch tension limits
T_MIN        = 200.0   # lower hard limit [N]
T_MAX        = 400.0   # upper hard limit [N]
SOFT_ZONE_N  = 25.0    # braking zone width [N]

TENSION_IC   = 300.0   # IC equilibrium tension
FLOOR_ALT_M  = 1.0

# Safety timeouts
T_REEL_OUT_MAX = 120.0
T_REEL_IN_MAX  = 300.0
T_END_SIM      = N_CYCLES * (T_REEL_OUT_MAX + T_REEL_IN_MAX) * 1.2


# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------

def _run_pumping(log, aero_model: str = "quasi_static") -> dict:
    runner = PhysicsRunner(_ROTOR, _IC, WIND, aero_model=aero_model, col_min_rad=-0.28, col_max_rad=0.10)

    COL_MIN, COL_MAX = -0.28, 0.10
    thrust_ic = float((_IC.coll_eq_rad - COL_MIN) / (COL_MAX - COL_MIN))
    ic_alt    = float(-_IC.pos[2])

    # ── Winch: velocity-commanded with soft tension limits ────────────────
    winch = VelocityWinchController(
        rest_length     = _IC.rest_length,
        v_max_out       = V_REEL_OUT,
        v_max_in        = V_REEL_IN,
        accel_limit_ms2 = 1.0,
        T_min           = T_MIN,
        T_max           = T_MAX,
        soft_zone_n     = SOFT_ZONE_N,
        min_length      = 2.0,
    )

    events = BadEventLog()
    mass_kg = _ROTOR.inertia.mass_kg
    if mass_kg is None:
        raise ValueError("Rotor inertia.mass_kg is required")

    # ── AP: MockArdupilot Python equivalent — altitude PID + rate-only body_z ─────────
    ap = MockArdupilot.for_pumping(
        ic_pos=_IC.pos,
        mass_kg=float(mass_kg),
        slew_rate_rad_s=BODY_Z_SLEW_RATE_RAD_S,
        warm_coll_rad=_IC.coll_eq_rad,
        tension_ic=TENSION_IC,
        events=events,
        wind=WIND,
        dt=DT,
    )

    # State machine
    phase         = "reel_out"
    phase_start_t = 0.0
    start_length  = _IC.rest_length
    cycle_idx     = 0
    cycle_net_start = [0.0] * N_CYCLES
    cycle_net_start[0] = 0.0   # winch starts at 0 energy

    phase_label = f"cycle1_{phase}"

    def _ap_tension_target() -> float:
        return TENSION_IC

    def _tel_fn(r, sr):
        pos_h  = r.hub_state["pos"][:2]
        r_norm = max(float(np.linalg.norm(pos_h)), 0.1)
        v_rad  = float(np.dot(r.hub_state["vel"][:2], pos_h / r_norm))
        return {
            **ap.log_fields(),
            **winch.log_fields(),
            "phase":          phase_label,
            "gnd_alt_cmd_m":  ic_alt,
            "roll_sp_rads":   ap.roll_sp,
            "pitch_sp_rads":  ap.pitch_sp,
            "vel_radial_mps": v_rad,
        }

    ap.tel_fn = _tel_fn
    comms     = VirtualComms()
    max_steps = int(T_END_SIM / DT) + 1

    ap_every      = max(1, round(1.0 / (MockArdupilot.AP_HZ * DT)))
    planner_every = max(1, round(DT_PLANNER / DT))

    prev_alt       = ic_alt
    prev_accel_ned = None

    # Initial command: AP owns collective locally via altitude PID.
    cmd = TensionCommand(
        tension_target_n   = _ap_tension_target(),
        alt_m              = ic_alt,
        phase              = phase,
    )
    comms.send_command(0.0, cmd)

    # ── Inflow warm-up ────────────────────────────────────────────────────
    runner._core.warm_inflow(collective_rad=float(_IC.coll_eq_rad), n_steps=500)

    t_sim = 0.0
    for i in range(max_steps):
        t_sim       = i * DT
        tension_now = runner.tension_now
        altitude    = runner.altitude
        t_in_phase  = t_sim - phase_start_t
        phase_label = f"cycle{cycle_idx+1}_{phase}"

        # ── Phase transitions ─────────────────────────────────────────────
        prev_phase = phase
        if phase == "reel_out":
            if (winch.rest_length >= start_length + DELTA_L - 0.05
                    or t_in_phase >= T_REEL_OUT_MAX):
                phase = "reel_in"
                phase_start_t = t_sim
        elif phase == "reel_in":
            if (winch.rest_length <= start_length + 0.05
                    or t_in_phase >= T_REEL_IN_MAX):
                cycle_idx += 1
                if cycle_idx >= N_CYCLES:
                    break
                phase = "reel_out"
                phase_start_t = t_sim
                cycle_net_start[cycle_idx] = winch.net_energy_j

        # ── Downlink ──────────────────────────────────────────────────────
        comms.inject(t_sim, altitude)

        # ── Ground 10 Hz: update winch velocity + refresh AP command ─────
        if i % planner_every == 0:
            tel = comms.receive_telemetry(t_sim)
            if tel is not None:
                prev_alt = tel.hub_alt_m
            winch.set_velocity(V_REEL_OUT if phase == "reel_out" else -V_REEL_IN)
            cmd = TensionCommand(
                tension_target_n   = _ap_tension_target(),
                alt_m              = ic_alt,
                phase              = phase,
            )
            comms.send_command(t_sim, cmd)

        # ── Uplink ────────────────────────────────────────────────────────
        ap_cmd = comms.poll_ap_command(t_sim)

        # ── AP 50 Hz ──────────────────────────────────────────────────────
        if i % ap_every == 0:
            ap.tick(t_sim, runner,
                    accel_ned=prev_accel_ned,
                    inject=(lambda _ap, __: _ap.receive_command(ap_cmd, DT_PLANNER))
                           if ap_cmd is not None else None)

        # ── Winch 400 Hz ──────────────────────────────────────────────────
        winch.step(tension_now, DT)

        # ── Physics 400 Hz ────────────────────────────────────────────────
        omega_body    = runner.omega_body
        omega_body[2] = 0.0
        sr = runner.step(DT, ap.col_rad, ap.roll_sp, ap.pitch_sp, omega_body,
                         rest_length=winch.rest_length)
        prev_accel_ned = sr.get("accel_specific_world")

        # ── Safety events ─────────────────────────────────────────────────
        if runner.tether._last_info.get("slack", False):
            events.record("slack", t_sim, phase_label, altitude,
                          tension=runner.tension_now)
        if runner.tension_now > BREAK_LOAD_N:
            events.record("tension_spike", t_sim, phase_label, altitude,
                          tension=runner.tension_now)
        if runner.hub_state["pos"][2] >= -FLOOR_ALT_M:
            events.record("floor_hit", t_sim, phase_label, altitude)
            break  # kite on ground — simulation over

        # ── Telemetry 20 Hz ───────────────────────────────────────────────
        ap.log(runner, sr)

    # ── Results ───────────────────────────────────────────────────────────────
    net_per_cycle = [winch.net_energy_j - cycle_net_start[k] for k in range(N_CYCLES)]
    total_net     = winch.net_energy_j

    cycle_summary = "  ".join(f"c{k+1}={net_per_cycle[k]:.0f}J" for k in range(N_CYCLES))
    parts = [
        f"total_net={total_net:.0f}J",
        cycle_summary,
        f"t_end={t_sim:.1f}s",
        events.summary(),
    ]

    ap.write_telemetry(log.log_dir / "telemetry.csv")
    log.write(["(telemetry: telemetry.csv)"],
               "  ".join(p for p in parts if p))

    return dict(
        t_end          = t_sim,
        net_per_cycle  = net_per_cycle,
        total_net      = total_net,
        events         = events,
        floor_hits     = events.count("floor_hit"),
        slack_events   = events.count("slack"),
        tension_spikes = events.count("tension_spike"),
    )


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_pumping_unified(simtest_log):
    """3 pumping cycles with VelocityWinchController at constant IC thrust.
    Net energy not expected (constant collective) -- pass if no bad events."""
    r = _run_pumping(simtest_log)
    failures = []
    if r["events"]:
        failures.append(r["events"].summary())
    assert not failures, "\n  ".join(failures)


@pytest.mark.skip(reason="Dynamic-inflow comparison only; quasi_static is the default flight model")
def test_pumping_unified_pitt_peters(simtest_log):
    """Same as test_pumping_unified but with Pitt-Peters aero (no xi limit)."""
    r = _run_pumping(simtest_log, aero_model="pitt_peters")
    failures = []
    if r["events"]:
        failures.append(r["events"].summary())
    assert not failures, "\n  ".join(failures)
