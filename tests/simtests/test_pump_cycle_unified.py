"""
test_pump_cycle_unified.py -- Pumping cycle with a coupled TorqueWinchController.

Architecture:
    Ground (10 Hz): simple reel_out/reel_in state machine.
                                    Sends altitude target and target tension feed-forward.
  Winch  (physics): finite-inertia drum driven by cable load against the
                   commanded tension and speed damping, stepped with the tether.
    AP     (400 Hz): MockArdupilot Python equivalent -- altitude PID collective, rate-only body_z control.

This test validates smooth, tension-controlled winch cycles without tension
spikes, slack, or floor hits.
"""
import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest


pytestmark = [pytest.mark.simtest,
              pytest.mark.timeout(int(os.environ.get("RAWES_PUMP_TIMEOUT", "600")))]

from simulation.winch import TorqueWinchController
from simulation.simtest_log import BadEventLog
from tests.simtests.simtest_ic import load_ic
from tests.simtests.simtest_runner import PhysicsRunner
from tests.common.mock_ardupilot import MockArdupilot
from groundstation.pumping_planner import TensionCommand
from simulation.comms import VirtualComms
from tests.simtests._rotor_helpers import load_default_rotor, BODY_Z_SLEW_RATE_RAD_S
from simulation.param_defaults import thrust_to_coll_rad

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
V_CRUISE_OUT = 0.5     # nominal reel-out cruise velocity [m/s]
V_CRUISE_IN  = 0.5     # nominal reel-in cruise speed [m/s] (applied as -V_CRUISE_IN)

# Simulation assumptions shared with the takeoff drum, not measured hardware.
DRUM_REFLECTED_MASS_KG = 0.2
DRUM_SPEED_DAMPING_NS_M = 20.0
DRUM_MAX_LENGTH_M = 200.0

TENSION_IC   = 300.0   # IC equilibrium tension
TENSION_REEL_OUT = TENSION_IC
TENSION_REEL_IN  = 100.0
FLOOR_ALT_M  = 1.0

# Safety timeouts
T_REEL_OUT_MAX = 120.0
T_REEL_IN_MAX  = 300.0
T_END_SIM      = N_CYCLES * (T_REEL_OUT_MAX + T_REEL_IN_MAX) * 1.2


# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------

def _run_pumping(log, aero_model: "str | None" = None) -> dict:
    # Aero model: explicit arg wins; otherwise the RAWES_AERO env var
    # (quasi_static [default] / pitt_peters / oye / vpm) selects it.
    if aero_model is None:
        aero_model = os.environ.get("RAWES_AERO", "quasi_static")
    thrust_ic = _IC.eq_thrust
    ic_alt    = float(-_IC.pos[2])

    winch = TorqueWinchController(
        rest_length=_IC.rest_length,
        reflected_mass_kg=DRUM_REFLECTED_MASS_KG,
        speed_damping_ns_m=DRUM_SPEED_DAMPING_NS_M,
        min_length=2.0,
        max_length=DRUM_MAX_LENGTH_M,
    )
    runner = PhysicsRunner(_ROTOR, _IC, WIND, aero_model=aero_model, winch=winch)

    events = BadEventLog()
    mass_kg = _ROTOR.inertia.mass_kg
    if mass_kg is None:
        raise ValueError("Rotor inertia.mass_kg is required")

    # ── AP: MockArdupilot Python equivalent — altitude PID + rate-only body_z ─────────
    ap = MockArdupilot.for_pumping(
        ic_pos=_IC.pos,
        mass_kg=float(mass_kg),
        slew_rate_rad_s=BODY_Z_SLEW_RATE_RAD_S,
        warm_thrust=_IC.eq_thrust,
        tension_ic=TENSION_IC,
        events=events,
        wind=WIND,
        dt=DT,
        # Body-z azimuth is a plane-keeping estimate (low-pass of the kite's own
        # position azimuth) inside the AP — no truth-wind oracle.
    )

    # State machine
    phase         = "reel_out"
    phase_start_t = 0.0
    start_length  = _IC.rest_length
    cycle_idx     = 0
    cycle_net_start = 0.0
    net_per_cycle = []

    phase_label = f"cycle1_{phase}"

    def _ap_tension_target() -> float:
        if phase == "reel_in":
            return TENSION_REEL_IN
        return TENSION_REEL_OUT

    def _tel_fn(r, sr):
        pos_h  = r.hub_state["pos"][:2]
        r_norm = max(float(np.linalg.norm(pos_h)), 0.1)
        v_rad  = float(np.dot(r.hub_state["vel"][:2], pos_h / r_norm))
        return {
            **ap.log_fields(),
            "phase":          phase_label,
            "gnd_alt_cmd_m":  ic_alt,
            "roll_sp_rads":   ap.roll_sp,
            "pitch_sp_rads":  ap.pitch_sp,
            "vel_radial_mps": v_rad,
        }

    ap.tel_fn = _tel_fn
    ap.winch  = winch   # centralized winch telemetry (winch_speed_ms) -- see MockArdupilot.log()
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
    runner._core.warm_inflow_from_thrust(float(_IC.eq_thrust), n_steps=500)

    t_sim = 0.0
    for i in range(max_steps):
        t_sim       = i * DT
        tension_now = runner.tension_now
        altitude    = runner.altitude
        t_in_phase  = t_sim - phase_start_t
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
                net_per_cycle.append(winch.net_energy_j - cycle_net_start)
                cycle_idx += 1
                if cycle_idx >= N_CYCLES:
                    break
                phase = "reel_out"
                phase_start_t = t_sim
                cycle_net_start = winch.net_energy_j

        if phase != prev_phase:
            print(
                f"t={t_sim:.2f}s {prev_phase}->{phase} "
                f"length={winch.rest_length:.2f}m speed={winch.speed_ms:.2f}m/s",
                flush=True,
            )

        phase_label = f"cycle{cycle_idx+1}_{phase}"

        # ── Downlink ──────────────────────────────────────────────────────
        comms.inject(t_sim, altitude)

        # ── Ground 10 Hz: update winch velocity + refresh AP command ─────
        if i % planner_every == 0 or phase != prev_phase:
            tel = comms.receive_telemetry(t_sim)
            if tel is not None:
                prev_alt = tel.hub_alt_m
            cruise_v = V_CRUISE_OUT if phase == "reel_out" else -V_CRUISE_IN
            winch.set_command(cruise_v, _ap_tension_target())
            cmd = TensionCommand(
                tension_target_n   = _ap_tension_target(),
                alt_m              = ic_alt,
                phase              = phase,
            )
            comms.send_command(t_sim, cmd)

        # ── Uplink ────────────────────────────────────────────────────────
        ap_cmd = comms.poll_ap_command(t_sim)

        # ── AP 50 Hz ──────────────────────────────────────────────────────
        if i % ap_every == 0 or phase != prev_phase:
            ap.tick(t_sim, runner,
                    accel_ned=prev_accel_ned,
                    inject=(lambda _ap, __: _ap.receive_command(ap_cmd, DT_PLANNER))
                           if ap_cmd is not None else None)

        # ── Physics 400 Hz ────────────────────────────────────────────────
        omega_body    = runner.omega_body
        omega_body[2] = 0.0
        sr = runner.step_from_thrust(DT, ap.thrust, ap.roll_sp, ap.pitch_sp, omega_body)
        prev_accel_ned = sr.get("accel_specific_world")

        # ── Safety events ─────────────────────────────────────────────────
        if runner.tether_result.slack:
            events.record("slack", t_sim, phase_label, altitude,
                          tension=runner.tension_now)
        if runner.tension_now > BREAK_LOAD_N:
            events.record("tension_spike", t_sim, phase_label, altitude,
                          tension=runner.tension_now)
        if runner.hub_state["pos"][2] >= -FLOOR_ALT_M:
            events.record("floor_hit", t_sim, phase_label, altitude)
            ap.log(runner, sr, force=True)
            break  # kite on ground — simulation over

        # ── Telemetry 20 Hz ───────────────────────────────────────────────
        ap.log(runner, sr)
        if i % round(10.0 / DT) == 0:
            print(
                f"t={runner.t_sim:.1f}s phase={phase_label} "
                f"alt={runner.altitude:.2f}m length={winch.rest_length:.2f}m "
                f"tension={runner.anchor_tension:.1f}N speed={winch.speed_ms:.2f}m/s",
                flush=True,
            )
            ap.write_telemetry(log.log_dir / "telemetry.csv")

    # ── Results ───────────────────────────────────────────────────────────────
    total_net     = winch.net_energy_j

    cycle_summary = "  ".join(f"c{k+1}={energy:.0f}J" for k, energy in enumerate(net_per_cycle))
    parts = [
        f"total_net={total_net:.0f}J",
        f"completed_cycles={cycle_idx}/{N_CYCLES}",
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
        completed_cycles = cycle_idx,
        events         = events,
        floor_hits     = events.count("floor_hit"),
        slack_events   = events.count("slack"),
        tension_spikes = events.count("tension_spike"),
    )


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_pumping_unified(simtest_log):
    """3 pumping cycles with the finite-inertia TorqueWinchController.
    Pass if no bad events (no tension spike, slack, or floor hit)."""
    r = _run_pumping(simtest_log)
    failures = []
    if r["events"]:
        failures.append(r["events"].summary())
    assert not failures, "\n  ".join(failures)
    assert r["completed_cycles"] == N_CYCLES


@pytest.mark.skip(reason="Dynamic-inflow comparison only; quasi_static is the default flight model")
def test_pumping_unified_pitt_peters(simtest_log):
    """Same as test_pumping_unified but with Pitt-Peters aero (no xi limit)."""
    r = _run_pumping(simtest_log, aero_model="pitt_peters")
    failures = []
    if r["events"]:
        failures.append(r["events"].summary())
    assert not failures, "\n  ".join(failures)
    assert r["completed_cycles"] == N_CYCLES
