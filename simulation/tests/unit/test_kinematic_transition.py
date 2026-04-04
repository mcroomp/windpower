"""
test_kinematic_transition.py — Unit-level reproduction of the full stack test.

Runs the complete simulation in pure Python without Docker or ArduPilot,
with two configurations side by side:

  Stack-mirror  Exactly mirrors test_pumping_cycle (stack test):
                  - 45 s kinematic startup (hub at constant vel0, R locked,
                    tether/planner completely inactive — just like the mediator)
                  - Free physics starts at t=45 s with vel = vel0 = 0.96 m/s
                  - DeschutterPlanner: col_max=0.0, xi=55 deg, tension_in=80 N
                  - compute_swashplate_from_state: kp=0.30, kd=0.12

  Unit-baseline Exactly mirrors test_deschutter_cycle (unit test):
                  - Starts directly from steady-state JSON (vel ~ 0)
                  - No kinematic phase at all
                  - DeschutterPlanner: col_max=0.10, xi=80 deg, tension_in=55 N
                  - compute_swashplate_from_state: kp=0.5, kd=0.2 (defaults)
                  - T_AERO_OFFSET=45 to match aero warmup state

Both run one full pumping cycle (30 s reel-out + 30 s reel-in) and write
mediator-compatible telemetry CSV so analyse_pumping_cycle.py can compare them.

Key insight: during kinematic the planner, tether and tension controller are
intentionally inactive.  The hub just moves along a straight trajectory to
reach the equilibrium position.  Only at t=45 s does the full physics stack
activate — but at that point the hub still carries the 0.96 m/s startup
velocity.  This test isolates whether that velocity, or the different planner
parameters, or both, cause the reel-in instability.
"""

import csv
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
from controller  import (compute_swashplate_from_state,
                         OrbitTracker,
                         col_min_for_altitude_rad)
from dynamics    import RigidBodyDynamics
from frames      import build_orb_frame
from planner     import DeschutterPlanner, WindEstimator, quat_is_identity, quat_apply
from simtest_ic  import load_ic
from tether      import TetherModel
from winch       import WinchController

# ---------------------------------------------------------------------------
# Shared constants
# ---------------------------------------------------------------------------

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

I_SPIN_KGMS2  = 10.0
OMEGA_SPIN_MIN = 0.5

TENSION_SAFETY_N = 496.0   # 80% break load

# ---------------------------------------------------------------------------
# Output paths for analyse_pumping_cycle.py
# ---------------------------------------------------------------------------

_LOGS_DIR       = Path(__file__).resolve().parents[3] / "simulation" / "logs"
_CSV_STACK      = _LOGS_DIR / "telemetry_unit_stack_mirror.csv"
_CSV_BASELINE   = _LOGS_DIR / "telemetry_unit_baseline.csv"

# Mediator-compatible CSV columns (must match analyse_pumping_cycle.py TelRow)
_CSV_COLS = [
    "t_sim", "hub_pos_x", "hub_pos_y", "hub_pos_z",
    "hub_vel_x", "hub_vel_y", "hub_vel_z",
    "tether_length", "tether_extension", "tether_tension", "tether_rest_length",
    "tether_slack", "collective_rad", "collective_norm",
    "pumping_phase", "tension_setpoint", "collective_from_tension_ctrl",
    "omega_rotor",
]


def _write_csv(rows: list[dict], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=_CSV_COLS, extrasaction="ignore")
        w.writeheader()
        for row in rows:
            w.writerow({k: row.get(k, "") for k in _CSV_COLS})


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
    # --- Attitude controller gains ---
    kp:                  float,          # proportional gain for tilt error
    kd:                  float,          # derivative gain for angular rate
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
    R0          = build_orb_frame(BODY_Z0)
    launch_pos  = POS0 - vel_at_start * kinematic_seconds if kinematic_seconds > 0 else POS0
    free_t0     = kinematic_seconds    # simulation time when free physics begins

    dyn = RigidBodyDynamics(
        mass   = _ROTOR.mass_kg,
        I_body = _ROTOR.I_body_kgm2,
        I_spin = I_spin,
        pos0   = (launch_pos.copy() if kinematic_seconds > 0 else POS0.copy()),
        vel0   = vel_at_start.copy(),
        R0     = R0.copy(),
        omega0 = np.zeros(3),
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

    total_t     = kinematic_seconds + 60.0   # kinematic + 30s reel-out + 30s reel-in
    n_steps     = int(total_t / DT)
    tel_every   = max(1, int(0.05 / DT))     # 20 Hz telemetry
    body_z_slew = _ROTOR.body_z_slew_rate_rad_s

    telemetry = []

    for i in range(n_steps):
        t_sim = i * DT
        t_free = t_sim - free_t0   # time since free physics started (<0 = kinematic)

        # ── Kinematic phase: override hub state, skip physics ──────────────
        if t_free < 0.0:
            kin_pos = launch_pos + vel_at_start * t_sim
            hub_state["pos"]   = kin_pos
            hub_state["vel"]   = vel_at_start.copy()
            hub_state["R"]     = R0.copy()
            hub_state["omega"] = np.zeros(3)
            # Keep dynamics in sync so the first free step integrates correctly
            dyn._pos[:]   = kin_pos
            dyn._vel[:]   = vel_at_start.copy()
            dyn._R[:]     = R0.copy()
            dyn._omega[:] = np.zeros(3)
            # Tether, planner, and tension controller are inactive during kinematic
            if i % tel_every == 0:
                pos = hub_state["pos"]
                telemetry.append({
                    "t_sim":       t_sim,
                    "hub_pos_x":   float(pos[0]),
                    "hub_pos_y":   float(pos[1]),
                    "hub_pos_z":   float(pos[2]),
                    "hub_vel_x":   float(vel_at_start[0]),
                    "hub_vel_y":   float(vel_at_start[1]),
                    "hub_vel_z":   float(vel_at_start[2]),
                    "tether_length":      0.0,
                    "tether_extension":   0.0,
                    "tether_tension":     0.0,
                    "tether_rest_length": float(REST_LENGTH0),
                    "tether_slack":       1,
                    "collective_rad":     float(col_min_rad),
                    "collective_norm":    0.0,
                    "pumping_phase":      "",
                    "tension_setpoint":   0.0,
                    "collective_from_tension_ctrl": 0.0,
                    "omega_rotor":        float(omega_spin),
                })
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

        # 4. Collective (DeschutterPlanner returns collective_rad directly)
        if "collective_rad" in cmd:
            collective_rad = float(cmd["collective_rad"])
        else:
            collective_rad = col_min_rad + cmd["thrust"] * (col_max_rad - col_min_rad)

        # 5. Orbit-tracking body_z setpoint + rate-limited slew (mediator path)
        aq = cmd["attitude_q"]
        _bz_target = (None if quat_is_identity(aq)
                      else quat_apply(aq, np.array([0.0, 0.0, -1.0])))
        body_z_eq = orbit_tracker.update(hub_state["pos"], DT, _bz_target)

        swash = compute_swashplate_from_state(
            hub_state=hub_state, anchor_pos=ANCHOR,
            body_z_eq=body_z_eq, kp=kp, kd=kd)

        # 6. Aerodynamics
        result = _AERO.compute_forces(
            collective_rad=collective_rad,
            tilt_lon=swash["tilt_lon"],
            tilt_lat=swash["tilt_lat"],
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
            ti = tether._last_info
            pos = hub_state["pos"]
            telemetry.append({
                "t_sim":          t_sim,
                "hub_pos_x":      float(pos[0]),
                "hub_pos_y":      float(pos[1]),
                "hub_pos_z":      float(pos[2]),
                "hub_vel_x":      float(hub_state["vel"][0]),
                "hub_vel_y":      float(hub_state["vel"][1]),
                "hub_vel_z":      float(hub_state["vel"][2]),
                "tether_length":       float(ti.get("length",    0.0)),
                "tether_extension":    float(ti.get("extension",  0.0)),
                "tether_tension":      float(tension_now),
                "tether_rest_length":  float(tether.rest_length),
                "tether_slack":        int(ti.get("slack", True)),
                "collective_rad":      float(collective_rad),
                "collective_norm":     float(cmd.get("thrust", 0.0)),
                "pumping_phase":       str(cmd.get("phase", "")),
                "tension_setpoint":    float(cmd.get("tension_setpoint_n", 0.0)),
                "collective_from_tension_ctrl": float(cmd.get("thrust", 0.0)),
                "omega_rotor":         float(omega_spin),
            })

    return {"label": label, "telemetry": telemetry}


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

@pytest.mark.simtest
def test_stack_mirror_vs_baseline():
    """
    Runs two configurations and compares them.

    Stack-mirror:   45 s kinematic startup + mediator's exact parameters.
                    Reproduces test_pumping_cycle (stack test) in pure Python.

    Unit-baseline:  No kinematic (vel~0) + unit test's exact parameters.
                    Mirrors test_deschutter_cycle (unit test) exactly.

    The comparison table is printed to the test log and both telemetry CSVs
    are written to simulation/logs/ for analyse_pumping_cycle.py.

    The baseline must pass all De Schutter checks.
    The stack-mirror is expected to fail (xfail) — this test captures when
    and by how much they diverge, making the regression visible without Docker.
    """
    _COL_MIN_REEL_IN_80 = col_min_for_altitude_rad(_AERO, 80.0, _ROTOR.mass_kg)
    _COL_MIN_REEL_IN_55 = col_min_for_altitude_rad(_AERO, 55.0, _ROTOR.mass_kg)

    # Use the same operating point as the unit test (xi=80°, tension_in=55N)
    # so that the stack test is directly comparable to the unit test.
    # All values come from DEFAULTS — no overrides needed.
    _D = DEFAULTS["trajectory"]["deschutter"]

    # --- Stack-mirror: kinematic startup + mediator parameters, same xi as unit test ---
    stack = _run_pumping_cycle(
        vel_at_start         = VEL0_STACK,              # 0.96 m/s at kinematic end
        kinematic_seconds    = DAMP_T,                   # 45 s kinematic startup
        col_max_rad          = float(_D["col_max_rad"]),         # 0.10
        col_min_rad          = float(_D["col_min_rad"]),         # -0.28
        col_min_reel_in_rad  = float(_D["col_min_reel_in_rad"]), # 0.079 (xi=80° floor)
        xi_reel_in_deg       = float(_D["xi_reel_in_deg"]),      # 80.0
        tension_out          = float(_D["tension_out"]),         # 200.0
        tension_in           = float(_D["tension_in"]),          # 55.0
        kp                   = 0.30,                    # mediator kp
        kd                   = 0.12,                    # mediator kd
        axle_attach          = 0.3,                     # mediator default
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
        kp                   = 0.5,             # default kp
        kd                   = 0.2,             # default kd
        axle_attach          = 0.0,             # unit test: axle_attachment_length=0
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
        kp                   = 0.5,             # unit test gains
        kd                   = 0.2,
        axle_attach          = 0.0,
        z_floor              = -1.0,
        I_spin               = 0.0,
        t_aero_offset        = DAMP_T,
        label                = "vel0-isolation",
    )

    # Write CSVs for downstream analysis
    _CSV_VEL0 = _LOGS_DIR / "telemetry_unit_vel0_isolation.csv"
    _write_csv(stack["telemetry"],          _CSV_STACK)
    _write_csv(baseline["telemetry"],       _CSV_BASELINE)
    _write_csv(vel0_isolation["telemetry"], _CSV_VEL0)

    # ── Compare ────────────────────────────────────────────────────────────
    def _metrics(tel: list[dict]) -> dict:
        free    = [r for r in tel if r.get("pumping_phase", "") != ""]
        ro      = [r for r in free if r.get("pumping_phase") == "reel-out"]
        ri      = [r for r in free if r.get("pumping_phase") == "reel-in"]
        alts    = [-float(r["hub_pos_z"]) for r in free
                   if r.get("hub_pos_z") not in ("", None)]
        ro_t    = [float(r["tether_tension"]) for r in ro
                   if r.get("tether_tension") not in ("", None, "0.0")]
        ri_t    = [float(r["tether_tension"]) for r in ri
                   if r.get("tether_tension") not in ("", None, "0.0")]
        ro_alts = [-float(r["hub_pos_z"]) for r in ro
                   if r.get("hub_pos_z") not in ("", None)]
        return {
            "alt_sd":     stdev(alts)    if len(alts)  > 1 else 0.0,
            "alt_min":    min(alts)      if alts        else 0.0,
            "alt_max":    max(alts)      if alts        else 0.0,
            "ro_alt_sd":  stdev(ro_alts) if len(ro_alts) > 1 else 0.0,
            "mean_t_out": mean(ro_t)     if ro_t        else 0.0,
            "mean_t_in":  mean(ri_t)     if ri_t        else 0.0,
            "peak_t":     max(ro_t + ri_t) if ro_t + ri_t else 0.0,
        }

    ms = _metrics(stack["telemetry"])
    mb = _metrics(baseline["telemetry"])
    mv = _metrics(vel0_isolation["telemetry"])

    print("\n" + "=" * 80)
    print("  Unit-level reproduction: three-way comparison")
    print("=" * 80)
    print(f"  {'Metric':<32} {'stack-mirror':>12} {'vel0-isolation':>14} {'unit-baseline':>13}  target")
    print(f"  {'-'*32} {'-'*12} {'-'*12}  {'-'*8}")
    print(f"  {'vel_start (m/s)':<32} {np.linalg.norm(VEL0_STACK):>12.4f} {np.linalg.norm(VEL0_JSON):>12.4f}")
    w = 80
    print(f"  {'-'*32} {'-'*12} {'-'*14} {'-'*13}")
    print(f"  {'vel_start (m/s)':<32} {np.linalg.norm(VEL0_STACK):>12.4f} {np.linalg.norm(VEL0_STACK):>14.4f} {np.linalg.norm(VEL0_JSON):>13.4f}")
    print(f"  {'params set':<32} {'mediator':>12} {'unit-test':>14} {'unit-test':>13}")
    print(f"  {'-'*32} {'-'*12} {'-'*14} {'-'*13}")
    print(f"  {'reel-out alt sd (m)':<32} {ms['ro_alt_sd']:>12.2f} {mv['ro_alt_sd']:>14.2f} {mb['ro_alt_sd']:>13.2f}  < 1.5")
    print(f"  {'alt min (m)':<32} {ms['alt_min']:>12.2f} {mv['alt_min']:>14.2f} {mb['alt_min']:>13.2f}  > 0.5")
    print(f"  {'mean reel-out tension (N)':<32} {ms['mean_t_out']:>12.1f} {mv['mean_t_out']:>14.1f} {mb['mean_t_out']:>13.1f}  ~200")
    print(f"  {'mean reel-in tension (N)':<32} {ms['mean_t_in']:>12.1f} {mv['mean_t_in']:>14.1f} {mb['mean_t_in']:>13.1f}  <reel-out")
    print(f"  {'peak tension (N)':<32} {ms['peak_t']:>12.1f} {mv['peak_t']:>14.1f} {mb['peak_t']:>13.1f}  < 496")
    print("=" * w)
    print("  If vel0-isolation matches unit-baseline: vel0 alone is NOT the cause.")
    print("  If vel0-isolation matches stack-mirror:  vel0 IS the remaining cause.")
    print(f"  CSVs: {_CSV_STACK.name} | {_CSV_VEL0.name} | {_CSV_BASELINE.name}")

    # ── Assertions ─────────────────────────────────────────────────────────

    # Baseline must always pass De Schutter checks
    assert mb["ro_alt_sd"] < 1.5, (
        f"Baseline reel-out altitude sd = {mb['ro_alt_sd']:.2f} m > 1.5 m "
        "(baseline regression)"
    )
    assert mb["mean_t_in"] < mb["mean_t_out"], (
        f"Baseline De Schutter mechanism failed: "
        f"reel-in {mb['mean_t_in']:.0f} N >= reel-out {mb['mean_t_out']:.0f} N"
    )
    assert mb["alt_min"] > 0.5, (
        f"Baseline hub crashed to {mb['alt_min']:.2f} m altitude"
    )

    # Stack-mirror is expected to show worse behaviour.
    # If it ALSO passes, the regression has been fixed — promote to regular pass.
    stack_stable    = ms["ro_alt_sd"] < 1.5
    stack_deschutter = ms["mean_t_in"] < ms["mean_t_out"]
    stack_no_crash  = ms["alt_min"] > 0.5

    if stack_stable and stack_deschutter and stack_no_crash:
        # Regression is fixed — both scenarios healthy
        pass
    else:
        issues = []
        if not stack_stable:
            issues.append(
                f"reel-out alt sd = {ms['ro_alt_sd']:.2f} m > 1.5 m "
                "(orbit unstable — kinematic transition instability reproduced)"
            )
        if not stack_deschutter:
            issues.append(
                f"De Schutter inverted: reel-in {ms['mean_t_in']:.0f} N "
                f">= reel-out {ms['mean_t_out']:.0f} N"
            )
        if not stack_no_crash:
            issues.append(f"hub crashed to {ms['alt_min']:.2f} m")
        pytest.xfail(
            "Stack-mirror shows known instability:\n  " + "\n  ".join(issues)
            + "\nFix: reduce vel0, ramp vel0 to zero, or extend settle period."
        )
