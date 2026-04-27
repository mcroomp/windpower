"""
test_pump_cycle_unified.py -- De Schutter pumping cycle using PumpingGroundController
and TensionApController (clean ground/AP boundary).

Ground (10 Hz):  PumpingGroundController  -> TensionCommand(setpoint + measurement + alt + phase)
Winch  (400 Hz): WinchController          -> tether length
AP     (400 Hz): TensionApController      -> collective + body_z_eq
                 TensionPI tracks tension_setpoint using tension_measured from ground.
                 AltitudeHoldController tracks alt_m for body_z.

Communication boundary:
  - Ground reads load cell, computes setpoint, packs TensionCommand at 10 Hz
  - AP has no tension sensor; uses ground-transmitted measurement as PI feedback
  - AP holds last received measurement between 10 Hz updates

Reel-out: ground ramps setpoint from tension_ic (~300N) to tension_out (435N) over ramp_s
Transition/reel-in: ground sends tension_in (226N); winch waits for T < T_reel_in_start
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

pytestmark = [pytest.mark.simtest, pytest.mark.timeout(600)]

import rotor_definition as rd
from winch            import WinchController
from simtest_log      import SimtestLog, BadEventLog
from simtest_ic       import load_ic
from simtest_runner   import PhysicsRunner, tel_every_from_env
from telemetry_csv    import TelRow, write_csv
from pumping_planner  import PumpingGroundController
from ap_controller    import TensionApController
from comms            import VirtualComms

_log   = SimtestLog(__file__)
_IC    = load_ic()
_ROTOR = rd.default()

# ── Simulation constants ──────────────────────────────────────────────────────
DT         = 1.0 / 400.0
DT_PLANNER = 1.0 / 10.0
WIND       = np.array([0.0, 10.0, 0.0])
WIND.flags.writeable = False
BREAK_LOAD_N = 620.0

# ── Pumping cycle parameters ──────────────────────────────────────────────────
N_CYCLES         = 3
DELTA_L          = 12.0    # tether length paid out per cycle [m]

_XI_START_DEG    = 30.0
_XI_REEL_IN_DEG  = 50.0
T_TRANSITION = (
    math.radians(_XI_REEL_IN_DEG - _XI_START_DEG) / _ROTOR.body_z_slew_rate_rad_s + 3.0
)

TENSION_OUT      = 435.0
TENSION_IN       = 240.0   # winch threshold during reel-in
TENSION_IN_AP    = 213.0   # AP setpoint: natural tension at 50° coll=0 is ~226N; oscillation dip 213-16=197N>0
TENSION_IC       = 300.0   # IC targets 300 N (midway between min and max)

EL_REEL_IN_RAD   = math.radians(_XI_REEL_IN_DEG)
TENSION_SAFETY_N = 496.0
FLOOR_ALT_M      = 1.0

# Safety timeouts — phases exit on tether length, but these cap runaway phases.
T_REEL_OUT_MAX = 120.0
T_REEL_IN_MAX  = 120.0
# Generous simulation budget: 3 cycles × (60 s reel-out + transition + 60 s reel-in)
T_END_SIM      = N_CYCLES * (T_REEL_OUT_MAX + T_TRANSITION + T_REEL_IN_MAX) * 1.2


# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------

def _run_pumping(aero_model: str = "skewed_wake") -> dict:
    runner = PhysicsRunner(_ROTOR, _IC, WIND, aero_model=aero_model)

    # ── Ground: PumpingGroundController (10 Hz outer loop) ────────────────
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
        k_ff_winch     = 75.0,
        k_ff_vel       = 40.0,
    )

    # ── Ground: WinchController (tension-controlled motion profile) ──────
    winch = WinchController(
        rest_length     = _IC.rest_length,
        kp_tension      = 0.005,
        v_max_out       = 0.40,
        v_max_in        = 0.80,
        accel_limit_ms2 = 0.05,
        min_length      = 2.0,
    )

    events    = BadEventLog()

    # ── AP: TensionApController + AcroControllerSitl (mirrors rawes.lua + ArduPilot ACRO) ──
    ap        = TensionApController(
        ic_pos          = _IC.pos,
        mass_kg         = _ROTOR.mass_kg,
        slew_rate_rad_s = _ROTOR.body_z_slew_rate_rad_s,
        warm_coll_rad   = _IC.coll_eq_rad,
        tension_ic      = TENSION_IC,
        el_corr_ki      = 0.0,   # daisy-chain disabled until gain is tuned
        events          = events,
    )
    comms     = VirtualComms()   # zero latency/noise; set latency_s/alt_noise_m to model radio
    max_steps = int(T_END_SIM / DT) + 1

    cycle_energy_out = [0.0] * N_CYCLES
    cycle_energy_in  = [0.0] * N_CYCLES
    telemetry = []
    tel_every     = tel_every_from_env(DT)
    planner_every = max(1, round(DT_PLANNER / DT))

    collective_rad = _IC.coll_eq_rad
    prev_alt       = float(-_IC.pos[2])
    cmd            = ground.step(0.0, 0.0, rest_length=_IC.rest_length,
                                 hub_alt_m=prev_alt)  # sentinel for t=0

    t_sim = 0.0
    for i in range(max_steps):
        t_sim = i * DT
        if ground.phase == "hold":
            break

        tension_now = runner.tension_now
        altitude    = runner.altitude
        cycle_idx   = min(ground.cycle_count, N_CYCLES - 1)

        # ── Downlink: inject physics truth into comms (400 Hz) ────────────
        comms.inject(t_sim, altitude)

        # ── Ground 10 Hz: receive telemetry, compute setpoint, send to AP ─
        if i % planner_every == 0:
            tel = comms.receive_telemetry(t_sim)
            if tel is not None:
                prev_alt = tel.hub_alt_m
            cmd = ground.step(t_sim, tension_now,
                              rest_length=winch.rest_length, hub_alt_m=prev_alt)
            if ground.phase == "hold":
                break   # all cycles done; don't apply hold command to AP
            comms.send_command(t_sim, cmd)

        # ── Uplink: deliver any arrived command to AP ─────────────────────
        ap_cmd = comms.poll_ap_command(t_sim)
        if ap_cmd is not None:
            ap.receive_command(ap_cmd, DT_PLANNER)

        # ── Winch 400 Hz (wired local sensor — no radio limit) ────────────
        winch.set_target(ground.winch_target_length, ground.winch_target_tension)
        len_before = winch.rest_length
        winch.step(tension_now, DT)
        speed_now = (winch.rest_length - len_before) / DT

        # ── AP 400 Hz: TensionApController → rate commands → physics ─────
        hub_state  = runner.hub_state
        omega_body = runner.omega_body
        collective_rad, rate_roll, rate_pitch = ap.step(
            hub_state["pos"], hub_state["R"], DT)
        sr = runner.step(DT, collective_rad, rate_roll, rate_pitch, omega_body,
                         rest_length=winch.rest_length)

        # ── Per-cycle energy accounting ───────────────────────────────────
        if speed_now > 0.0:
            cycle_energy_out[cycle_idx] += runner.tension_now * speed_now * DT
        elif speed_now < 0.0:
            cycle_energy_in[cycle_idx]  += runner.tension_now * abs(speed_now) * DT

        # ── Bad-event tracking ────────────────────────────────────────────
        phase_label = f"cycle{cycle_idx+1}_{ground.phase}"
        if runner.tether._last_info.get("slack", False):
            events.record("slack", t_sim, phase_label, altitude,
                          tension=runner.tension_now)
        if runner.tension_now > BREAK_LOAD_N:
            events.record("tension_spike", t_sim, phase_label, altitude,
                          tension=runner.tension_now)
        if runner.hub_state["pos"][2] >= -FLOOR_ALT_M:
            events.record("floor_hit", t_sim, phase_label, altitude)

        # ── Telemetry 20 Hz ───────────────────────────────────────────────
        if i % tel_every == 0:
            telemetry.append(TelRow.from_physics(
                runner, sr, collective_rad, WIND,
                body_z_eq                    = hub_state["R"][:, 2],
                phase                        = phase_label,
                tension_setpoint             = ap.tension_setpoint,
                collective_from_tension_ctrl = collective_rad,
                gnd_alt_cmd_m                = cmd.alt_m,
                winch_speed_ms               = speed_now,
                elevation_rad                = ap.elevation_rad,
                el_correction_rad            = ap.el_correction_rad,
                coll_saturated               = ap.coll_saturated,
                comms_ok                     = ap.comms_ok,
            ))

    # ── Results ───────────────────────────────────────────────────────────────
    net_per_cycle = [cycle_energy_out[k] - cycle_energy_in[k] for k in range(N_CYCLES)]
    total_net     = sum(net_per_cycle)

    cycle_summary = "  ".join(
        f"c{k+1}={net_per_cycle[k]:.0f}J" for k in range(N_CYCLES)
    )
    parts = [
        f"total_net={total_net:.0f}J",
        cycle_summary,
        f"t_end={t_sim:.1f}s",
        events.summary(),
    ]

    if telemetry:
        write_csv(telemetry, _log.log_dir / "telemetry.csv")
    _log.write(["(telemetry: telemetry.csv)"],
               "  ".join(p for p in parts if p))

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
        telemetry        = telemetry,
    )


# ---------------------------------------------------------------------------
# Test
# ---------------------------------------------------------------------------

def test_pumping_unified():
    """3 pumping cycles via PumpingGroundController/TensionApController: no bad events, net > 0."""
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


def test_pumping_unified_peters_he():
    """Same pumping cycle as test_pumping_unified but with Peters-He aero (no xi limit)."""
    r = _run_pumping(aero_model="peters_he")
    failures = []
    if r["events"]:
        failures.append(r["events"].summary())
    for k, net in enumerate(r["net_per_cycle"]):
        if net <= 0:
            failures.append(f"cycle {k+1} net={net:.1f}J <= 0")
    if r["total_net"] <= 0:
        failures.append(f"total_net={r['total_net']:.1f}J <= 0")
    assert not failures, "\n  ".join(failures)
