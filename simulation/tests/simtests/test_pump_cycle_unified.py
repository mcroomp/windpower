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
from simtest_log      import BadEventLog
from simtest_ic       import load_ic
from simtest_runner   import PhysicsRunner, PythonAP
from pumping_planner  import PumpingGroundController
from ap_controller    import TensionApController
from comms            import VirtualComms

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

def _run_pumping(log, aero_model: str = "skewed_wake") -> dict:
    runner = PhysicsRunner(_ROTOR, _IC, WIND, aero_model=aero_model, col_min_rad=-0.28, col_max_rad=0.10)

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

    # ── AP: TensionApController wrapped in PythonAP (mirrors LuaAP interface) ──
    _ap       = TensionApController(
        ic_pos          = _IC.pos,
        mass_kg         = _ROTOR.mass_kg,
        slew_rate_rad_s = _ROTOR.body_z_slew_rate_rad_s,
        warm_coll_rad   = _IC.coll_eq_rad,
        tension_ic      = TENSION_IC,
        events          = events,
    )
    ap        = PythonAP(_ap, wind=WIND, dt=DT)
    ap.tel_fn = lambda r, sr: {
        **ap.log_fields(),
        **winch.log_fields(),
        "body_z_eq":    r.hub_state["R"][:, 2],
        "phase":        phase_label,
        "gnd_alt_cmd_m": cmd.alt_m,
    }
    comms     = VirtualComms()   # zero latency/noise; set latency_s/alt_noise_m to model radio
    max_steps = int(T_END_SIM / DT) + 1

    cycle_net_start  = [0.0] * N_CYCLES   # winch.net_energy_j snapshot at cycle start
    prev_cycle_idx   = -1
    ap_every     = max(1, round(1.0 / (PythonAP.AP_HZ * DT)))
    planner_every = max(1, round(DT_PLANNER / DT))

    prev_alt       = float(-_IC.pos[2])
    prev_accel_ned = None   # one-step lagged IMU specific force for vibration damper
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

        # ── Uplink: deliver arrived command to AP (mirrors NV float injection) ─
        ap_cmd = comms.poll_ap_command(t_sim)

        # ── AP 50 Hz: attitude hold + vibration damper → rate/collective cmds ─
        if i % ap_every == 0:
            ap.tick(t_sim, runner,
                    accel_ned=prev_accel_ned,
                    inject=(lambda _ap, __: _ap.receive_command(ap_cmd, DT_PLANNER))
                           if ap_cmd is not None else None)

        # ── Winch 400 Hz (wired local sensor — no radio limit) ────────────
        winch.set_target(ground.winch_target_length, ground.winch_target_tension)
        winch.step(tension_now, DT)

        # ── Physics 400 Hz ────────────────────────────────────────────────
        omega_body    = runner.omega_body
        omega_body[2] = 0.0
        sr = runner.step(DT, ap.col_rad, ap.roll_sp, ap.pitch_sp, omega_body,
                         rest_length=winch.rest_length)
        prev_accel_ned = sr.get("accel_specific_world")

        # ── Per-cycle energy snapshot ─────────────────────────────────────
        phase_label = f"cycle{cycle_idx+1}_{ground.phase}"
        if cycle_idx != prev_cycle_idx:
            cycle_net_start[cycle_idx] = winch.net_energy_j
            prev_cycle_idx = cycle_idx
        if runner.tether._last_info.get("slack", False):
            events.record("slack", t_sim, phase_label, altitude,
                          tension=runner.tension_now)
        if runner.tension_now > BREAK_LOAD_N:
            events.record("tension_spike", t_sim, phase_label, altitude,
                          tension=runner.tension_now)
        if runner.hub_state["pos"][2] >= -FLOOR_ALT_M:
            events.record("floor_hit", t_sim, phase_label, altitude)

        # ── Telemetry 20 Hz ───────────────────────────────────────────────
        ap.log(runner, sr)

    # ── Results ───────────────────────────────────────────────────────────────
    net_per_cycle = [winch.net_energy_j - cycle_net_start[k] for k in range(N_CYCLES)]
    total_net     = winch.net_energy_j

    cycle_summary = "  ".join(
        f"c{k+1}={net_per_cycle[k]:.0f}J" for k in range(N_CYCLES)
    )
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
        t_end         = t_sim,
        net_per_cycle = net_per_cycle,
        total_net     = total_net,
        events        = events,
        floor_hits    = events.count("floor_hit"),
        slack_events  = events.count("slack"),
        tension_spikes= events.count("tension_spike"),
    )


# ---------------------------------------------------------------------------
# Test
# ---------------------------------------------------------------------------

def test_pumping_unified(simtest_log):
    """3 pumping cycles via PumpingGroundController/TensionApController: no bad events, net > 0."""
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


def test_pumping_unified_peters_he(simtest_log):
    """Same pumping cycle as test_pumping_unified but with Peters-He aero (no xi limit)."""
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
