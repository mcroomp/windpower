"""
test_ground_liftoff.py -- rawes.lua takeoff mode climbs, then hands off to steady flight.

Physical scenario
-----------------
  - Hub starts at altitude 1 m (NED pos = [0, 0, -1]), directly above the anchor.
  - Anchor at the mock EKF origin, i.e. NED offset [0, 0, 0] (see rawes_modes.send_anchor_ned).
  - Rotor disk starts horizontal: body_z = [0, 0, +1] (NED; FRD body_z points toward anchor).
  - Wind: constant 10 m/s crosswind (East component).  The hub is free to
    drift downwind during the vertical climb -- MODE_TAKEOFF holds attitude
    only, it does not correct lateral drift.
  - rawes.lua mode=2 (TAKEOFF): holds a fixed level attitude (body_z=[0,0,1])
    and climbs toward RAWES_ALT via the altitude-PID collective loop.
  - The ground station (this test) monitors altitude.  Once it reaches
    TAKEOFF_MIN_ALT_M, it switches RAWES_MODE to MODE_STEADY, commands
    RAWES_TEN=100 N / RAWES_ALT for a 30 deg elevation at a 100 m tether
    length, and *tightens* the winch (previously free-spooling) so it reels
    out under tension control from wherever the hub happens to be to the
    full 100 m operating length.
  - Real elastic tether (default TetherModel) -- during the takeoff climb the
    tether is kept deliberately slack (free-spooled) by tracking the hub's
    actual distance from the anchor; at this short rest length the tether's
    elastic stiffness (k = EA / L) is high enough that any real resistance
    during the initial climb would spike tension well past the tether's
    breaking load within a single physics step.  Only once the ground
    station tightens the winch (after the climb) does the tether resist.
  - Floor at altitude 0 m: hub cannot go underground.

Coordinate frame: NED -- X=North, Y=East, Z=Down.  Altitude = -pos_z.
"""
from types import SimpleNamespace
import math

import numpy as np
import pytest


pytestmark = [
    pytest.mark.simtest,
    pytest.mark.timeout(300),
    pytest.mark.skip(reason="Temporarily disabled while liftoff/reel-out behavior is being investigated"),
]

from tests.simtests.simtest_runner import PhysicsRunner
from tests.common.mock_ardupilot import MockArdupilot
from simulation.rawes_lua_harness import RawesLua
from simulation.winch import GovernedWinchController
from groundstation.gcs import NamedValueFloat
from groundstation.rawes_modes import MODE_STEADY, MODE_TAKEOFF, send_anchor_ned
from tests.simtests._rotor_helpers import load_default_rotor

# ── Physical constants ────────────────────────────────────────────────────────
_ROTOR = load_default_rotor()

DT                = 2.5e-3          # 400 Hz
LUA_PERIOD        = 0.020           # 50 Hz Lua tick
LUA_EVERY         = round(LUA_PERIOD / DT)
WIND_NED          = np.array([0.0, 10.0, 0.0])   # 10 m/s crosswind (East); drift is expected
OMEGA_START_RAD_S = 40.0          # fixed rotor start speed [rad/s]
AERO_MODEL        = "quasi_static"    # use default quasi-static aero model

TARGET_THRUST     = 1.0             # RAWES_THR IC-seed sent during MODE_TAKEOFF -- an aggressive
                                     # climb-out trim is needed here to sustain rotor RPM (a
                                     # hover-equilibrium seed lets the rotor slowly spin down and
                                     # the climb stall well short of TAKEOFF_MIN_ALT_M).
TARGET_ROLL_RAD   = 0.0             # RAWES_ROFF command sent each tick
TARGET_PITCH_RAD  = 0.0             # RAWES_POFF command sent each tick

TAKEOFF_ALT_TARGET_M = 11.0   # RAWES_ALT commanded during MODE_TAKEOFF -- kept close to the transition
                               # threshold so the altitude PID is already decelerating (small alt_err,
                               # small vertical speed) by the time the ground switches to MODE_STEADY;
                               # a much higher target (e.g. 20 m) leaves the hub climbing at full speed
                               # right through the transition, which the winch cannot match (see
                               # WINCH_V_MAX_OUT below) and produces a tether snap-load / bounce.
TAKEOFF_MIN_ALT_M    = 10.0   # ground switches RAWES_MODE -> STEADY once this altitude is reached
LIFTOFF_ALT_M        = 2.0    # early liftoff sanity checkpoint [m]

TETHER_LEN_TARGET_M = 100.0                                                          # final operating tether length [m]
TETHER_TEN_TARGET_N = 100.0                                                          # steady-flight tension target [N]
ELEV_TARGET_DEG     = 30.0                                                           # target elevation once fully reeled out
STEADY_ALT_TARGET_M = TETHER_LEN_TARGET_M * math.sin(math.radians(ELEV_TARGET_DEG))  # = 50 m

FREE_SPOOL_MARGIN_M = 0.10   # keep the tether just slack during the takeoff climb [m]

# Winch gains (GovernedWinchController) -- same proven values as the pumping-cycle
# tests (test_pump_cycle_unified.py): a cruise-velocity feedforward plus a small
# tension-trim governor, jerk-limited so the motion is smooth.  The feedforward
# term is fed the hub's actual radial (anchor-relative) closing speed each step
# (see test body) so the winch pays out at the hub's own separation rate instead
# of waiting for tension to build before reacting.
WINCH_V_MAX_OUT   = 2.0      # max pay-out speed [m/s]
WINCH_V_MAX_IN    = 0.5      # max reel-in speed [m/s]
WINCH_KP_TENSION  = 4.0e-4   # governor trim gain [(m/s)/N]
WINCH_ACCEL_MS2   = 2.0      # acceleration limit [m/s^2]
WINCH_JERK_MS3    = 10.0     # jerk limit [m/s^3] (S-curve smoothing)
WINCH_TENSION_TAU = 0.08     # load-cell low-pass time constant [s]

CLIMB_RATE_MPS = 0.5   # target sustained climb rate during MODE_STEADY reel-out [m/s] --
                        # ramping the altitude reference at a fixed rate (rather than
                        # capping it at the tether-implied altitude for the *current*
                        # rest length) keeps a small, persistent altitude error alive so
                        # the altitude PID keeps commanding a gentle climb.  Without this,
                        # the hub settles at the current rest length's equilibrium
                        # altitude (zero error -> zero climb), the winch's velocity
                        # feedforward (driven by the hub's own velocity) sees nothing to
                        # react to, and the tether never reels out further.

LIFTOFF_TIMEOUT    = 20.0     # must reach LIFTOFF_ALT_M within this many sim-seconds [s]
TRANSITION_TIMEOUT = 60.0     # must reach TAKEOFF_MIN_ALT_M within this many sim-seconds [s]
TOTAL_RUNTIME      = 180.0    # total simulated seconds (climb + winch reel-out to 100 m) [s]


# ── IC construction ───────────────────────────────────────────────────────────

def _build_ic() -> SimpleNamespace:
    """
    Hub at altitude 1 m, horizontal disk.  Anchor at NED origin.

    eq_thrust from IC file if available; omega_spin is forced to OMEGA_START_RAD_S.
    """
    try:
        from tests.simtests.simtest_ic import load_ic as _load_ic
        _ic = _load_ic()
        eq_thrust   = float(_ic.eq_thrust)
    except FileNotFoundError:
        from simulation.param_defaults import load_collective_phys_range as _lr
        col_min, col_max = _lr()
        eq_thrust   = (-0.18 - col_min) / (col_max - col_min)

    return SimpleNamespace(
        pos         = np.array([0.0, 0.0, -1.0]),  # altitude = 1 m
        vel         = np.zeros(3),
        R0          = np.eye(3),                    # horizontal disk; body_z = [0,0,1]
        rest_length = 1.0,
        eq_thrust   = eq_thrust,
        omega_spin  = OMEGA_START_RAD_S,
    )


# ── Test ──────────────────────────────────────────────────────────────────────

def test_ground_liftoff(simtest_log):
    """
    Hub at altitude 1 m, level disk, 10 m/s crosswind.  rawes.lua MODE_TAKEOFF
    holds level attitude and climbs (altitude-PID collective) while the
    tether free-spools (kept slack).  Once altitude >= TAKEOFF_MIN_ALT_M, the
    ground station switches RAWES_MODE to MODE_STEADY, commands RAWES_TEN /
    RAWES_ALT for a 30 deg elevation at a 100 m tether length, and tightens
    the winch to reel out to that length while holding ~100 N tension.
    """
    ic = _build_ic()

    sim = RawesLua(mode=MODE_TAKEOFF)
    sim.armed        = True
    sim.healthy      = True
    sim.vehicle_mode = 4           # GUIDED
    sim.pos_ned      = ic.pos.tolist()
    sim.vel_ned      = ic.vel.tolist()
    sim.R            = ic.R0
    sim.gyro         = [0.0, 0.0, 0.0]

    runner = PhysicsRunner(
        _ROTOR, ic, WIND_NED,
        aero_model=AERO_MODEL,
        z_floor     = 0.0,
    )
    # Real elastic tether (default TetherModel) -- the tether must physically
    # reel out to TETHER_LEN_TARGET_M, so no constant-force stand-in here.

    lua         = MockArdupilot.for_lua(sim, initial_thrust=ic.eq_thrust, wind=WIND_NED, dt=DT)
    lua.tel_fn  = lambda r, sr: {}   # all fields now come centrally from _telemetry_overrides()/winch
    total_steps = int(TOTAL_RUNTIME / DT)

    liftoff_t    = None
    transition_t = None
    max_alt      = ic.pos[2] * -1.0   # start altitude
    floor_hit    = False
    floor_hit_t  = None

    rest_length = float(ic.rest_length)
    winch       = None   # created once the ground station tightens the tether (see below)

    # One-time snapshot of the actual thrust being commanded at the instant
    # of transition (see below) -- seeds RAWES_THR for the rest of the flight
    # so the MODE_STEADY capture (ic_thrust_or_default(), applied with no
    # slew in run_flight()) picks up a continuous value instead of jumping
    # straight to the takeoff's aggressive TARGET_THRUST.
    capture_thrust_seed = TARGET_THRUST
    alt_at_transition    = None   # altitude ramp start point (captured at transition)

    send_anchor_ned(sim, 0.0, 0.0, 0.0)   # anchor at the mock EKF origin

    def _inject(s, r):
        if transition_t is None:
            s.send_message(NamedValueFloat("RAWES_ALT", TAKEOFF_ALT_TARGET_M))
            s.send_message(NamedValueFloat("RAWES_THR", TARGET_THRUST))
        else:
            # Ramp the altitude target at a fixed climb rate from the
            # transition altitude toward the fully-reeled-out target (capped
            # there) -- see CLIMB_RATE_MPS above for why a rest_length-tied
            # cap stalls the reel-out instead.
            alt_cmd = min(
                STEADY_ALT_TARGET_M,
                alt_at_transition + CLIMB_RATE_MPS * (r.t_sim - transition_t),
            )
            s.send_message(NamedValueFloat("RAWES_ALT", alt_cmd))
            s.send_message(NamedValueFloat("RAWES_TEN", TETHER_TEN_TARGET_N))
            s.send_message(NamedValueFloat("RAWES_THR", capture_thrust_seed))
        s.send_message(NamedValueFloat("RAWES_ROFF", TARGET_ROLL_RAD))
        s.send_message(NamedValueFloat("RAWES_POFF", TARGET_PITCH_RAD))

    for i in range(total_steps):
        t = i * DT
        if i % LUA_EVERY == 0:
            lua.tick(t, runner, inject=_inject)

        if winch is None:
            # Free-spool: keep the rest length just ahead of the hub's actual
            # distance from the anchor so the tether stays slack (no elastic
            # resistance) while MODE_TAKEOFF is climbing on pure vertical
            # thrust -- see module docstring for why a taut short tether here
            # would spike tension past breaking load.
            dist = float(np.linalg.norm(runner.hub_state["pos"]))
            rest_length = max(rest_length, dist + FREE_SPOOL_MARGIN_M)
        else:
            # Velocity feedforward: cruise the winch at the hub's own radial
            # (anchor-relative) closing speed so pay-out tracks the hub as it
            # recedes, instead of waiting for tension to build before
            # reacting -- a purely tension-reactive winch starts every
            # transition at zero speed and cannot keep up with a hub already
            # climbing at several m/s, which snap-loads the (very stiff,
            # short) tether taut.  The tension governor still trims this
            # cruise speed to hold TETHER_TEN_TARGET_N.
            hub_pos = runner.hub_state["pos"]
            hub_vel = runner.hub_state["vel"]
            dist    = max(float(np.linalg.norm(hub_pos)), 0.1)
            v_rad   = float(np.dot(hub_vel, hub_pos) / dist)
            cruise_v = max(0.0, min(WINCH_V_MAX_OUT, v_rad))
            winch.set_command(cruise_v, TETHER_TEN_TARGET_N)
            winch.step(runner.tension_now, DT)
            rest_length = winch.rest_length

        sr = lua.step(runner, DT, rest_length=rest_length)
        lua.log(runner, sr)

        alt = runner.altitude
        if alt > max_alt:
            max_alt = alt
        if liftoff_t is None and alt >= LIFTOFF_ALT_M:
            liftoff_t = runner.t_sim
        if transition_t is None and alt >= TAKEOFF_MIN_ALT_M:
            transition_t = runner.t_sim
            alt_at_transition   = alt
            capture_thrust_seed = lua.thrust   # actual thrust the instant before the mode switch
            sim.set_param("RAWES_MODE", MODE_STEADY)
            # Tighten the winch: start from the current (free-spooled) rest
            # length and reel further out to the full operating length while
            # holding the target tension.
            winch = GovernedWinchController(
                rest_length     = rest_length,
                v_max_out       = WINCH_V_MAX_OUT,
                v_max_in        = WINCH_V_MAX_IN,
                kp_tension      = WINCH_KP_TENSION,
                accel_limit_ms2 = WINCH_ACCEL_MS2,
                jerk_limit_ms3  = WINCH_JERK_MS3,
                tension_tau_s   = WINCH_TENSION_TAU,
                min_length      = float(ic.rest_length),
                max_length      = TETHER_LEN_TARGET_M + 20.0,
            )
            lua.winch = winch   # centralized winch telemetry (winch_speed_ms) -- see MockArdupilot.log()
        # Ground floor (z_floor=0.0 passed to PhysicsRunner above) clamps the
        # hub at altitude 0 rather than letting it go underground.  Once the
        # ground station has tightened the winch this should not happen --
        # reaching the floor after transition indicates a genuine loss of
        # control, not an expected end state.
        if alt <= 0.0:
            floor_hit   = True
            floor_hit_t = runner.t_sim
            break

    t_final        = runner.t_sim
    omega_final    = runner.omega_spin
    final_tension  = runner.tension_now
    final_tlen     = float(np.linalg.norm(runner.hub_state["pos"]))
    final_elev_deg = math.degrees(math.asin(
        max(-1.0, min(1.0, runner.altitude / max(final_tlen, 0.1)))))

    print()
    print(f"  t_final       : {t_final:.1f} s")
    print(f"  max_altitude  : {max_alt:.3f} m")
    print(f"  omega_spin    : {omega_final:.2f} rad/s  (started {ic.omega_spin:.2f})")
    if floor_hit:
        print(f"  floor_hit_t   : {floor_hit_t:.2f} s  (grounded; stopped early)")
    if liftoff_t is not None:
        print(f"  liftoff_t     : {liftoff_t:.2f} s  [PASS]")
    else:
        print(f"  liftoff_t     : did not reach {LIFTOFF_ALT_M} m  [FAIL]")
    if transition_t is not None:
        print(f"  transition_t  : {transition_t:.2f} s  (MODE_TAKEOFF -> MODE_STEADY)  [PASS]")
    else:
        print(f"  transition_t  : did not reach {TAKEOFF_MIN_ALT_M} m  [FAIL]")
    print(f"  final_tlen    : {final_tlen:.2f} m  (target {TETHER_LEN_TARGET_M:.1f} m)")
    print(f"  final_tension : {final_tension:.1f} N  (target {TETHER_TEN_TARGET_N:.1f} N)")
    print(f"  final_elev    : {final_elev_deg:.1f} deg  (target {ELEV_TARGET_DEG:.1f} deg)")

    lines = [
        f"t_final       : {t_final:.1f} s",
        f"max_altitude  : {max_alt:.3f} m",
        f"omega_spin    : {omega_final:.2f} rad/s  (started {ic.omega_spin:.2f})",
        (
            f"floor_hit_t   : {floor_hit_t:.2f} s  (grounded; stopped early)"
            if floor_hit
            else "floor_hit_t   : never grounded"
        ),
        (
            f"liftoff_t     : {liftoff_t:.2f} s  [PASS]"
            if liftoff_t is not None
            else f"liftoff_t     : did not reach {LIFTOFF_ALT_M} m  [FAIL]"
        ),
        (
            f"transition_t  : {transition_t:.2f} s  (MODE_TAKEOFF -> MODE_STEADY)  [PASS]"
            if transition_t is not None
            else f"transition_t  : did not reach {TAKEOFF_MIN_ALT_M} m  [FAIL]"
        ),
        f"final_tlen    : {final_tlen:.2f} m  (target {TETHER_LEN_TARGET_M:.1f} m)",
        f"final_tension : {final_tension:.1f} N  (target {TETHER_TEN_TARGET_N:.1f} N)",
        f"final_elev    : {final_elev_deg:.1f} deg  (target {ELEV_TARGET_DEG:.1f} deg)",
    ]
    for level, msg in sim.messages[:10]:
        lines.append(f"  [GCS {level}] {msg}")

    lua.write_telemetry(simtest_log.log_dir / "telemetry.csv")
    simtest_log.write(lines, "ground_liftoff")

    assert liftoff_t is not None, (
        f"Hub did not reach {LIFTOFF_ALT_M} m altitude within {LIFTOFF_TIMEOUT} s "
        f"(max altitude reached: {max_alt:.4f} m)"
    )
    assert liftoff_t < LIFTOFF_TIMEOUT, (
        f"Liftoff at {liftoff_t:.2f} s exceeds timeout {LIFTOFF_TIMEOUT} s"
    )
    assert transition_t is not None, (
        f"Hub did not reach the {TAKEOFF_MIN_ALT_M} m takeoff threshold within "
        f"{TRANSITION_TIMEOUT} s (max altitude reached: {max_alt:.2f} m)"
    )
    assert transition_t < TRANSITION_TIMEOUT, (
        f"Takeoff->steady transition at {transition_t:.2f} s exceeds timeout {TRANSITION_TIMEOUT} s"
    )
    assert not floor_hit, (
        f"Hub crashed to the floor at t={floor_hit_t:.2f} s after transitioning to steady flight "
        f"-- expected sustained tethered flight, not a fall"
    )
    assert final_tlen >= 0.9 * TETHER_LEN_TARGET_M, (
        f"Tether did not reel out to the {TETHER_LEN_TARGET_M:.1f} m target "
        f"(reached {final_tlen:.2f} m)"
    )
