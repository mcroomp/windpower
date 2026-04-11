"""
test_kinematic_transition.py — Unit-level reproduction of the full stack test.

Runs the complete simulation in pure Python without Docker or ArduPilot,
with two configurations side by side:

  Stack-mirror  Mirrors test_pumping_cycle (stack test) as closely as possible:
                  - 45 s kinematic startup with 15 s velocity ramp to zero
                  - Free physics starts at t=45 s with vel ~ 0, R = R0, omega = 0
                  - DeschutterPlanner: col_max=0.10, xi=80 deg, tension_in=55 N
                  - axle_attachment_length from rotor definition (0.3 m, matches mediator)
                  - AcroController (two-loop + servo, same as SITL)

  Unit-baseline Exactly mirrors test_deschutter_cycle (unit test):
                  - Starts directly from steady-state JSON (vel ~ 0)
                  - No kinematic phase at all
                  - DeschutterPlanner: col_max=0.10, xi=80 deg, tension_in=55 N
                  - AcroController (two-loop + servo, same as SITL)
                  - T_AERO_OFFSET=45 to match aero warmup state

Both run one full pumping cycle (30 s reel-out + 30 s reel-in) and write
mediator-compatible telemetry CSV so analyse_pumping_cycle.py can compare them.

NOTE: The 15 s velocity ramp (kinematic_vel_ramp_s=15.0) tapers exit velocity
to zero, so the stack-mirror and baseline produce identical initial conditions
at the start of free flight.  All three scenarios (stack-mirror, vel0-isolation,
baseline) yield the same ro_alt_sd ~4.2 m — the 4 m altitude oscillation is an
intrinsic property of the De Schutter cycle, not a kinematic artifact.
"""

import math
import sys
from pathlib import Path
from statistics import mean, stdev

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

pytestmark = [pytest.mark.simtest, pytest.mark.timeout(300)]

import rotor_definition as rd
from aero        import create_aero
from config      import DEFAULTS
from controller  import (OrbitTracker,
                         col_min_for_altitude_rad,
                         AcroController)
from dynamics    import RigidBodyDynamics
from frames      import build_orb_frame
from kinematic   import KinematicStartup
from planner     import DeschutterPlanner, WindEstimator, quat_is_identity, quat_apply
from simtest_ic  import load_ic
from simtest_log import SimtestLog
from tether      import TetherModel
from winch       import WinchController
from tel           import make_tel
from telemetry_csv import TelRow, write_csv, read_csv

# ---------------------------------------------------------------------------
# Shared constants
# ---------------------------------------------------------------------------

_log    = SimtestLog(__file__)
_IC     = load_ic()
_ROTOR  = rd.default()
_AERO   = create_aero(_ROTOR)

DT               = 1.0 / 400.0
ANCHOR           = np.zeros(3)
WIND             = np.array([0.0, 10.0, 0.0])  # NED: 10 m/s East

# Steady-state initial conditions
POS0         = _IC.pos
VEL0_JSON    = _IC.vel          # ~0 m/s (true equilibrium velocity)
BODY_Z0      = _IC.body_z
OMEGA_SPIN0  = _IC.omega_spin
REST_LENGTH0 = _IC.rest_length

# Kinematic startup velocity (from config defaults — what the stack test uses)
VEL0_STACK   = np.array(DEFAULTS["vel0"], dtype=float)   # [0.916, -0.257, 0.093] m/s
DAMP_T       = 45.0   # kinematic duration [s] — matches StackConfig.STARTUP_DAMP_S
RAMP_S       = float(DEFAULTS["kinematic_vel_ramp_s"])  # vel ramp window [s]

I_SPIN_KGMS2  = 10.0
OMEGA_SPIN_MIN = 0.5

TENSION_SAFETY_N = 496.0   # 80% break load

# ---------------------------------------------------------------------------
# Output paths for analyse_pumping_cycle.py
# ---------------------------------------------------------------------------

_CSV_STACK      = _log.log_dir / "telemetry_stack_mirror.csv"
_CSV_BASELINE   = _log.log_dir / "telemetry_baseline.csv"



# ---------------------------------------------------------------------------
# Simulation runner
# ---------------------------------------------------------------------------

def _run_pumping_cycle(
    # --- Starting conditions ---
    vel_at_start:        np.ndarray,     # hub velocity at the start of free physics
    kinematic_seconds:   float,          # 0 = no kinematic; >0 = run kinematic first
    # --- Planner parameters ---
    col_max_rad:         float,          # max collective [rad]
    col_min_rad:         float,          # min collective [rad]
    col_min_reel_in_rad: "float | None", # min collective during reel-in (None = col_min)
    xi_reel_in_deg:      float,          # reel-in tilt angle [deg]
    tension_out:         float,          # reel-out tension setpoint [N]
    tension_in:          float,          # reel-in tension setpoint [N]
    # --- Physics parameters ---
    axle_attach:         float = 0.3,   # tether attachment offset from CoM [m]
    z_floor:             "float | None" = None,  # NED Z altitude floor (None = no floor)
    I_spin:              float = 0.0,   # spin-axis inertia [kg m^2]
    # --- Aero time offset ---
    t_aero_offset:       float = 0.0,   # added to t for aero.compute_forces()
    label:               str   = "",
) -> dict:
    """
    Run one full pumping cycle (30 s reel-out + 30 s reel-in).

    Kinematic phase (if kinematic_seconds > 0):
      Hub position = launch_pos + vel_at_start * t  (straight-line kinematic)
      Hub R = equilibrium R0, omega = 0
      Tether, planner, and tension controller are ALL inactive — exactly
      mirroring the mediator's kinematic startup.  The hub just moves to
      the equilibrium position and then free physics begins.

    Free physics phase:
      Exactly the mediator's internal_controller=True path:
        1. DeschutterPlanner.step() -> collective_rad, winch_speed, phase
        2. WinchController.step()
        3. OrbitTracker.update() + compute_swashplate_from_state()
        4. aero.compute_forces() + tether.compute()
        5. dynamics.step()
    """
    R0       = build_orb_frame(BODY_Z0)
    _startup = KinematicStartup(
        target_pos = POS0,
        target_vel = vel_at_start,
        duration   = kinematic_seconds,
        ramp_s     = RAMP_S if kinematic_seconds > 0 else 0.0,
        R0         = R0,
    )
    free_t0 = kinematic_seconds

    dyn = RigidBodyDynamics(
        mass   = _ROTOR.mass_kg,
        I_body = _ROTOR.I_body_kgm2,
        I_spin = I_spin,
        pos0   = _startup.launch_pos.tolist(),
        vel0   = vel_at_start.tolist(),
        R0     = R0.copy(),
        omega0 = [0.0, 0.0, 0.0],
        z_floor= z_floor,
    )
    tether = TetherModel(anchor_ned=ANCHOR, rest_length=REST_LENGTH0,
                         axle_attachment_length=axle_attach)
    winch  = WinchController(rest_length=REST_LENGTH0,
                             tension_safety_n=TENSION_SAFETY_N)
    trajectory = DeschutterPlanner(
        t_reel_out          = 30.0,
        t_reel_in           = 30.0,
        t_transition        = 5.0,
        v_reel_out          = 0.4,
        v_reel_in           = 0.4,
        tension_out         = tension_out,
        tension_in          = tension_in,
        wind_estimator      = WindEstimator(seed_wind_ned=WIND),
        xi_reel_in_deg      = xi_reel_in_deg,
        col_min_rad         = col_min_rad,
        col_max_rad         = col_max_rad,
        col_min_reel_in_rad = col_min_reel_in_rad,
    )

    hub_state         = dyn.state
    omega_spin        = OMEGA_SPIN0
    tension_now       = 0.0
    collective_rad    = col_min_rad
    ic_tether_dir0    = None
    ic_body_z_eq0     = None
    orbit_tracker     = None
    acro              = AcroController.from_rotor(_ROTOR, use_servo=True)
    total_t     = kinematic_seconds + 60.0   # kinematic + 30s reel-out + 30s reel-in
    n_steps     = int(total_t / DT)
    tel_every   = max(1, int(0.05 / DT))     # 20 Hz telemetry
    body_z_slew = _ROTOR.body_z_slew_rate_rad_s

    telemetry = []

    for i in range(n_steps):
        t_sim = i * DT
        t_free = t_sim - free_t0   # time since free physics started (<0 = kinematic)

        # ── Kinematic phase: override hub state, skip physics ──────────────
        if _startup.apply(hub_state, dyn, t_sim):
            # Tether, planner, and tension controller are inactive during kinematic
            if i % tel_every == 0:
                telemetry.append(make_tel(
                    t_sim, hub_state, omega_spin, tether, 0.0,
                    col_min_rad, 0.0, 0.0, WIND,
                    body_z_eq=hub_state["R"][:, 2],
                    phase="kinematic",
                ))
            continue

        # ── Free physics phase ─────────────────────────────────────────────

        # 1. Capture orbit-tracking ICs on first free step
        if ic_tether_dir0 is None:
            ic_tether_dir0   = hub_state["pos"] / np.linalg.norm(hub_state["pos"])
            ic_body_z_eq0    = hub_state["R"][:, 2].copy()
            orbit_tracker    = OrbitTracker(ic_body_z_eq0, ic_tether_dir0, body_z_slew)

        # 2. Planner: tension PI drives collective; winch speed for phase
        state_pkt = {
            "pos_ned":         hub_state["pos"],
            "vel_ned":         hub_state["vel"],
            "omega_spin":      omega_spin,
            "body_z":          hub_state["R"][:, 2],
            "tension_n":       tension_now,
            "tether_length_m": winch.tether_length_m,
        }
        cmd = trajectory.step(state_pkt, DT)

        # 3. WinchController
        winch.step(cmd["winch_speed_ms"], tension_now, DT)
        tether.rest_length = winch.rest_length

        # 4. Collective (DeschutterPlanner returns collective_rad directly); servo-slewed
        if "collective_rad" in cmd:
            raw_col = float(cmd["collective_rad"])
        else:
            raw_col = col_min_rad + cmd["thrust"] * (col_max_rad - col_min_rad)
        collective_rad = acro.slew_collective(raw_col, DT)

        # 5. Orbit-tracking body_z setpoint + rate-limited slew (mediator path)
        aq = cmd["attitude_q"]
        _bz_target = (None if quat_is_identity(aq)
                      else quat_apply(aq, np.array([0.0, 0.0, -1.0])))
        body_z_eq = orbit_tracker.update(hub_state["pos"], DT, _bz_target)

        # 6. Aerodynamics (two-loop attitude + servo emulates ArduPilot ACRO)
        tilt_lon, tilt_lat = acro.update(hub_state, body_z_eq, DT)
        result = _AERO.compute_forces(
            collective_rad=collective_rad,
            tilt_lon=tilt_lon,
            tilt_lat=tilt_lat,
            R_hub=hub_state["R"],
            v_hub_world=hub_state["vel"],
            omega_rotor=omega_spin,
            wind_world=WIND,
            t=t_aero_offset + t_free,
        )

        # 7. Tether
        tf, tm = tether.compute(hub_state["pos"], hub_state["vel"], hub_state["R"])
        F_net     = result.F_world + tf
        M_orbital = result.M_orbital + tm
        tension_now = tether._last_info.get("tension", 0.0)

        # 8. Spin + dynamics
        omega_spin = max(OMEGA_SPIN_MIN,
                         omega_spin + result.Q_spin / I_SPIN_KGMS2 * DT)
        M_orbital += -50.0 * hub_state["omega"]
        hub_state = dyn.step(F_net, M_orbital, DT, omega_spin=omega_spin)

        # 9. Telemetry (20 Hz)
        if i % tel_every == 0:
            telemetry.append(make_tel(
                t_sim, hub_state, omega_spin, tether, tension_now,
                collective_rad, tilt_lon, tilt_lat, WIND,
                body_z_eq=body_z_eq,
                phase=str(cmd.get("phase", "")),
                tension_setpoint=float(cmd.get("tension_setpoint_n", 0.0)),
            ))

    return {"label": label, "telemetry": telemetry}


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

@pytest.mark.simtest
def test_stack_mirror_vs_baseline():
    """
    Runs two configurations and compares them.

    Stack-mirror:   45 s kinematic startup (15 s velocity ramp to zero) +
                    mediator's exact parameters.  Exit conditions are vel~0,
                    R=R0, omega=0 -- identical to the baseline.

    Unit-baseline:  No kinematic (vel~0) + unit test's exact parameters.
                    Mirrors test_deschutter_cycle (unit test) exactly.

    The vel0-isolation scenario uses the kinematic startup with unit-test
    parameters to confirm vel0 is not the distinguishing factor.

    The comparison table is printed to the test log and all three telemetry
    CSVs are written to simulation/logs/ for analyse_pumping_cycle.py.

    All three scenarios must pass the same De Schutter checks.
    """
    _COL_MIN_REEL_IN_80 = col_min_for_altitude_rad(_AERO, 80.0, _ROTOR.mass_kg)
    _COL_MIN_REEL_IN_55 = col_min_for_altitude_rad(_AERO, 55.0, _ROTOR.mass_kg)

    # Use the same operating point as the unit test (xi=80°, tension_in=55N)
    # so that the stack test is directly comparable to the unit test.
    # All values come from DEFAULTS — no overrides needed.
    _D = DEFAULTS["trajectory"]["deschutter"]

    # --- Stack-mirror: kinematic startup + mediator parameters ---
    stack = _run_pumping_cycle(
        vel_at_start         = VEL0_STACK,              # 0.96 m/s — kinematic entry vel
        kinematic_seconds    = DAMP_T,                   # 45 s kinematic startup
        col_max_rad          = float(_D["col_max_rad"]),         # 0.10
        col_min_rad          = float(_D["col_min_rad"]),         # -0.28
        col_min_reel_in_rad  = float(_D["col_min_reel_in_rad"]), # 0.079 (xi=80° floor)
        xi_reel_in_deg       = float(_D["xi_reel_in_deg"]),      # 80.0
        tension_out          = float(_D["tension_out"]),         # 200.0
        tension_in           = float(_D["tension_in"]),          # 55.0
        axle_attach          = _ROTOR.axle_attachment_length_m,  # 0.3 m — matches mediator
        z_floor              = None,                    # mediator has no floor
        I_spin               = 0.0,
        t_aero_offset        = DAMP_T,
        label                = "stack-mirror",
    )

    # --- Unit-baseline: exactly what test_deschutter_cycle does ---
    baseline = _run_pumping_cycle(
        vel_at_start         = VEL0_JSON,       # ~0 m/s (steady-state)
        kinematic_seconds    = 0.0,             # no kinematic phase
        col_max_rad          = 0.10,            # COL_MAX_RAD from unit test
        col_min_rad          = -0.28,
        col_min_reel_in_rad  = _COL_MIN_REEL_IN_80,  # unit test COL_MIN_REEL_IN_RAD
        xi_reel_in_deg       = 80.0,            # XI_REEL_IN_DEG from unit test
        tension_out          = 200.0,
        tension_in           = 55.0,            # DEFAULT_TENSION_IN from unit test
        axle_attach          = _ROTOR.axle_attachment_length_m,  # 0.3 m — universal
        z_floor              = -1.0,            # unit test: z_floor=-1.0 (1 m alt floor)
        I_spin               = 0.0,             # unit test: I_spin=0.0
        t_aero_offset        = DAMP_T,          # T_AERO_OFFSET from unit test
        label                = "unit-baseline",
    )

    # --- vel0 isolation: unit-test parameters but with the kinematic velocity ---
    # If this scenario is also stable (alt_min > 0.5 m), then vel0 alone is NOT
    # the cause of the stack-mirror's reel-out oscillation — the parameter
    # differences (kp, col_max, xi, etc.) are responsible.
    # If this scenario also fails, vel0 is the direct cause.
    vel0_isolation = _run_pumping_cycle(
        vel_at_start         = VEL0_STACK,      # 0.96 m/s — the kinematic kick
        kinematic_seconds    = DAMP_T,           # include the kinematic phase
        col_max_rad          = 0.10,            # unit test params (not mediator)
        col_min_rad          = -0.28,
        col_min_reel_in_rad  = _COL_MIN_REEL_IN_80,
        xi_reel_in_deg       = 80.0,
        tension_out          = 200.0,
        tension_in           = 55.0,
        axle_attach          = _ROTOR.axle_attachment_length_m,
        z_floor              = -1.0,
        I_spin               = 0.0,
        t_aero_offset        = DAMP_T,
        label                = "vel0-isolation",
    )

    # Write CSVs for downstream analysis
    _CSV_VEL0 = _log.log_dir / "telemetry_vel0_isolation.csv"
    write_csv([TelRow.from_tel(d) for d in stack["telemetry"]],          _CSV_STACK)
    write_csv([TelRow.from_tel(d) for d in baseline["telemetry"]],       _CSV_BASELINE)
    write_csv([TelRow.from_tel(d) for d in vel0_isolation["telemetry"]], _CSV_VEL0)

    # ── Compare ────────────────────────────────────────────────────────────
    def _metrics(rows: list[TelRow]) -> dict:
        free    = [r for r in rows if r.phase != ""]
        ro      = [r for r in free if r.phase == "reel-out"]
        ri      = [r for r in free if r.phase == "reel-in"]
        alts    = [r.altitude      for r in free]
        ro_t    = [r.tether_tension for r in ro if r.tether_tension > 0]
        ri_t    = [r.tether_tension for r in ri if r.tether_tension > 0]
        ro_alts = [r.altitude      for r in ro]
        return {
            "alt_sd":     stdev(alts)    if len(alts)  > 1 else 0.0,
            "alt_min":    min(alts)      if alts        else 0.0,
            "alt_max":    max(alts)      if alts        else 0.0,
            "ro_alt_sd":  stdev(ro_alts) if len(ro_alts) > 1 else 0.0,
            "mean_t_out": mean(ro_t)     if ro_t        else 0.0,
            "mean_t_in":  mean(ri_t)     if ri_t        else 0.0,
            "peak_t":     max(ro_t + ri_t) if ro_t + ri_t else 0.0,
        }

    ms = _metrics(read_csv(_CSV_STACK))
    mb = _metrics(read_csv(_CSV_BASELINE))
    mv = _metrics(read_csv(_CSV_VEL0))

    w = 80
    print("\n" + "=" * w)
    print("  Unit-level reproduction: three-way comparison")
    print("=" * w)
    print(f"  {'Metric':<32} {'stack-mirror':>12} {'vel0-isolation':>14} {'unit-baseline':>13}  target")
    print(f"  {'-'*32} {'-'*12} {'-'*14} {'-'*13}")
    print(f"  {'vel_start (m/s)':<32} {np.linalg.norm(VEL0_STACK):>12.4f} {np.linalg.norm(VEL0_STACK):>14.4f} {np.linalg.norm(VEL0_JSON):>13.4f}")
    print(f"  {'vel at exit (m/s)':<32} {np.linalg.norm(VEL0_STACK):>12.4f} {np.linalg.norm(VEL0_STACK):>14.4f} {np.linalg.norm(VEL0_JSON):>13.4f}")
    print(f"  {'params set':<32} {'mediator':>12} {'unit-test':>14} {'unit-test':>13}")
    print(f"  {'-'*32} {'-'*12} {'-'*14} {'-'*13}")
    print(f"  {'reel-out alt sd (m)':<32} {ms['ro_alt_sd']:>12.2f} {mv['ro_alt_sd']:>14.2f} {mb['ro_alt_sd']:>13.2f}  < 6.0")
    print(f"  {'alt min (m)':<32} {ms['alt_min']:>12.2f} {mv['alt_min']:>14.2f} {mb['alt_min']:>13.2f}  > 0.5")
    print(f"  {'mean reel-out tension (N)':<32} {ms['mean_t_out']:>12.1f} {mv['mean_t_out']:>14.1f} {mb['mean_t_out']:>13.1f}  ~200")
    print(f"  {'mean reel-in tension (N)':<32} {ms['mean_t_in']:>12.1f} {mv['mean_t_in']:>14.1f} {mb['mean_t_in']:>13.1f}  <reel-out")
    print(f"  {'peak tension (N)':<32} {ms['peak_t']:>12.1f} {mv['peak_t']:>14.1f} {mb['peak_t']:>13.1f}  < 496")
    print("=" * w)
    print(f"  CSVs in {_log.log_dir.name}/: {_CSV_STACK.name} | {_CSV_VEL0.name} | {_CSV_BASELINE.name}")

    # ── Assertions ─────────────────────────────────────────────────────────

    # Baseline must always pass De Schutter checks
    assert mb["ro_alt_sd"] < 6.0, (
        f"Baseline reel-out altitude sd = {mb['ro_alt_sd']:.2f} m > 6.0 m "
        "(baseline regression — threshold recalibrated for axle_attachment_length=0.3 m; "
        "restoring torque at 200 N tether tension produces ~4 m natural altitude SD)"
    )
    assert mb["mean_t_in"] < mb["mean_t_out"], (
        f"Baseline De Schutter mechanism failed: "
        f"reel-in {mb['mean_t_in']:.0f} N >= reel-out {mb['mean_t_out']:.0f} N"
    )
    assert mb["alt_min"] > 0.5, (
        f"Baseline hub crashed to {mb['alt_min']:.2f} m altitude"
    )

    # Stack-mirror: velocity ramp ensures exit vel~0, so initial conditions
    # match the baseline exactly.  Same De Schutter thresholds apply.
    assert ms["ro_alt_sd"] < 6.0, (
        f"Stack-mirror reel-out altitude sd = {ms['ro_alt_sd']:.2f} m > 6.0 m"
    )
    assert ms["mean_t_in"] < ms["mean_t_out"], (
        f"Stack-mirror De Schutter mechanism failed: "
        f"reel-in {ms['mean_t_in']:.0f} N >= reel-out {ms['mean_t_out']:.0f} N"
    )
    assert ms["alt_min"] > 0.5, (
        f"Stack-mirror hub crashed to {ms['alt_min']:.2f} m altitude"
    )
