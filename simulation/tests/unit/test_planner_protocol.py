"""
test_planner_protocol.py -- Protocol conformance tests for planner step() methods.

These tests verify that each planner correctly reads sensor values from the right
source (kwargs vs state_pkt) and responds to them in the correct direction.

WHY THIS FILE EXISTS
--------------------
The mediator calls planner.step(state_pkt, dt, tension_n=T, tether_length_m=L).
``state_pkt`` contains only MAVLink-style fields (pos_ned, vel_ned, omega_spin,
body_z); tension_n and tether_length_m are ground-station local measurements
passed via **kwargs.  If a planner reads ``state.get("tension_n", 0.0)`` instead
of ``kwargs.get("tension_n", state.get("tension_n", 0.0))`` it silently sees 0 N
tension on every step, making TensionPI drive collective the wrong way.

The root invariant tested here is purely directional and survives any legitimate
change to gains, limits, or warm-start values:

  DeschutterPlanner (reel-out, body_z tether-aligned):
    higher actual tension  --> TensionPI lowers output --> more negative collective
    lower  actual tension  --> TensionPI raises output --> less negative collective
    ==>  col_rad(T_high) < col_rad(T_low)   [more negative == smaller number]

  LandingPlanner (descent, winch-tension PI):
    higher actual tension  --> winch pays out faster (less reel-in)
    lower  actual tension  --> winch reels in faster
    ==>  winch_speed_ms(T_high) > winch_speed_ms(T_low)
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import config as _cfg
from planner       import DeschutterPlanner, WindEstimator
from landing_planner import LandingPlanner


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------

_DCFG = _cfg.DEFAULTS["trajectory"]["deschutter"]
_DT   = 1.0 / 400.0   # 400 Hz mediator rate


def _make_deschutter(t_hold_s: float = 0.0) -> DeschutterPlanner:
    """Create a DeschutterPlanner from production config defaults.

    ``t_hold_s=0`` skips the hold phase so the first step() enters reel-out
    immediately, letting us probe the transition code without waiting 10+ s.
    """
    we = WindEstimator(seed_wind_ned=np.array([0.0, 10.0, 0.0]))   # East wind
    return DeschutterPlanner(
        t_reel_out           = float(_DCFG["t_reel_out"]),
        t_reel_in            = float(_DCFG["t_reel_in"]),
        t_transition         = float(_DCFG["t_transition"]),
        v_reel_out           = float(_DCFG["v_reel_out"]),
        v_reel_in            = float(_DCFG["v_reel_in"]),
        tension_out          = float(_DCFG["tension_out"]),
        tension_in           = float(_DCFG["tension_in"]),
        wind_estimator       = we,
        xi_reel_in_deg       = float(_DCFG["xi_reel_in_deg"]),
        tension_kp           = float(_DCFG["tension_kp"]),
        tension_ki           = float(_DCFG["tension_ki"]),
        col_min_rad          = float(_DCFG["col_min_rad"]),
        col_min_reel_in_rad  = float(_DCFG["col_min_reel_in_rad"]),
        col_max_rad          = float(_DCFG["col_max_rad"]),
        warm_coll_rad        = float(_DCFG["warm_coll_rad"]),
        t_hold_s             = t_hold_s,
    )


# State dict as the mediator builds it: MAVLink fields only, NO tension_n.
_MEDIATOR_STATE_PKT = {
    "pos_ned":    np.array([27.9, 95.0, -14.1]),   # 100 m from anchor, 14 m alt
    "vel_ned":    np.zeros(3),
    "omega_spin": 30.0,
    "body_z":     np.array([0.28, 0.95, -0.14]),   # approx tether direction
}


# ---------------------------------------------------------------------------
# DeschutterPlanner
# ---------------------------------------------------------------------------

def test_deschutter_step_reads_tension_from_kwargs():
    """
    Mediator passes tension_n via **kwargs; it must not be read from state.

    Two identical planners are advanced through one identical warmup step, then
    given very different tensions via kwargs only.  If kwargs are read correctly,
    their collectives diverge in the expected direction (high tension gives a
    less negative collective than low tension).  If kwargs are ignored and state
    is used instead, both planners see the same missing-tension default and
    produce identical output -- the assertion fails.

    The direction invariant holds for any positive kp, any warm-start, and any
    col_min/col_max range:
      tension > setpoint  --> error < 0 --> TensionPI lowers output --> more negative
      tension < setpoint  --> error > 0 --> TensionPI raises output --> less negative
    """
    tension_setpoint = float(_DCFG["tension_out"])
    # Tensions well away from the setpoint on both sides so the PI response is
    # clearly directional even if gains or t_transition change in the future.
    tension_high = tension_setpoint * 2.0    # well above setpoint
    tension_low  = tension_setpoint * 0.25   # well below setpoint

    planner_high = _make_deschutter()
    planner_low  = _make_deschutter()

    # Warmup step: both planners enter reel-out together.
    # At transition alpha = 0 the output equals the warm-start collective
    # regardless of tension, so both produce identical output and identical
    # internal state after this step.
    warmup_tension = tension_setpoint   # any value -- doesn't affect alpha=0 output
    planner_high.step(_MEDIATOR_STATE_PKT, _DT, tension_n=warmup_tension)
    planner_low.step( _MEDIATOR_STATE_PKT, _DT, tension_n=warmup_tension)

    # Divergence step: alpha > 0 now, so the transition target formula uses
    # tension_n.  The two planners receive their distinct tensions here.
    cmd_high = planner_high.step(_MEDIATOR_STATE_PKT, _DT, tension_n=tension_high)
    cmd_low  = planner_low.step( _MEDIATOR_STATE_PKT, _DT, tension_n=tension_low)

    col_high = cmd_high["collective_rad"]
    col_low  = cmd_low["collective_rad"]

    # High tension above setpoint: TensionPI lowers output (more negative collective).
    # Low tension below setpoint: TensionPI raises output (less negative collective).
    assert col_high < col_low, (
        f"DeschutterPlanner.step() is not reading tension_n from **kwargs. "
        f"With tension_high={tension_high:.0f} N, collective={col_high:.4f} rad should be "
        f"LESS (more negative) than with tension_low={tension_low:.0f} N "
        f"collective={col_low:.4f} rad. "
        f"If both are equal the planner is using state.get('tension_n', 0) instead of "
        f"kwargs.get('tension_n', ...) -- both calls see 0 N so TensionPI gives "
        f"the same output regardless of the kwargs value."
    )


def test_deschutter_step_kwargs_tension_differs_from_missing_tension():
    """
    A planner that receives tension via kwargs must produce a different collective
    from one that receives no tension at all (default 0 N).

    This directly mirrors the production failure mode: if step() ignores kwargs
    and falls back to state.get("tension_n", 0.0), every call looks like tension=0.
    The test will PASS only when kwargs are actually read.
    """
    tension_setpoint = float(_DCFG["tension_out"])
    # Use a tension that produces a clearly different PI output from the 0 N default.
    # Any value significantly above setpoint works; 3x gives plenty of margin.
    tension_explicit = tension_setpoint * 3.0

    planner_explicit = _make_deschutter()
    planner_missing  = _make_deschutter()

    # Warmup: identical for both (alpha=0 => output independent of tension)
    planner_explicit.step(_MEDIATOR_STATE_PKT, _DT, tension_n=tension_setpoint)
    planner_missing.step( _MEDIATOR_STATE_PKT, _DT, tension_n=tension_setpoint)

    # Divergence step.
    # planner_explicit: receives real tension via kwargs
    # planner_missing:  state_pkt has no tension_n, no kwargs tension either
    #                   => if step() reads from kwargs.get(..., state.get(..., 0))
    #                      it sees 0 N; if it reads from state.get(..., 0) it also
    #                      sees 0 N -- so missing and bug-present are the same.
    #                      The explicit planner MUST differ when kwargs are read.
    cmd_explicit = planner_explicit.step(_MEDIATOR_STATE_PKT, _DT, tension_n=tension_explicit)
    cmd_missing  = planner_missing.step( _MEDIATOR_STATE_PKT, _DT)

    col_explicit = cmd_explicit["collective_rad"]
    col_missing  = cmd_missing["collective_rad"]

    assert col_explicit != col_missing, (
        f"DeschutterPlanner.step() ignores tension_n in kwargs: "
        f"explicit tension ({tension_explicit:.0f} N via kwargs) gives collective={col_explicit:.4f} rad "
        f"but no-tension call gives collective={col_missing:.4f} rad -- they are equal. "
        f"The planner must read tension_n from kwargs, not from state (which lacks it)."
    )


# ---------------------------------------------------------------------------
# LandingPlanner
# ---------------------------------------------------------------------------

def _make_landing_planner(k_winch: float = 0.005) -> LandingPlanner:
    """Create a LandingPlanner with a non-trivial winch PI gain."""
    return LandingPlanner(
        initial_body_z   = np.array([0.0, 1.0, 0.0]),   # any unit vector
        v_land           = 0.5,
        col_cruise       = 0.079,
        min_tether_m     = 2.0,
        k_winch          = k_winch,
    )


def _make_landing_state(tension_n: float, tether_length_m: float = 10.0) -> dict:
    """Build a state_pkt for LandingPlanner.step() with tension_n in the dict.

    Unit tests use this path (tension in state_pkt).
    The mediator uses the **kwargs path (tension as keyword argument).
    Both are supported: kwargs take priority, state_pkt is the fallback.
    """
    return {
        "body_z":          np.array([0.0, 1.0, 0.0]),
        "vel_ned":         np.zeros(3),
        "pos_ned":         np.array([0.0, 0.0, -10.0]),
        "tether_length_m": tether_length_m,
        "tension_n":       tension_n,
    }


# state_pkt as the mediator builds it: MAVLink fields only, NO tension_n.
_MEDIATOR_LANDING_STATE_PKT = {
    "body_z":   np.array([0.0, 0.2, -0.98]),   # nearly vertical (xi~80 deg)
    "vel_ned":  np.zeros(3),
    "pos_ned":  np.array([0.0, 5.0, -20.0]),    # 20 m altitude, 5 m East
    "tether_length_m": 20.0,                     # present here so unit tests work;
                                                  # mediator overrides with kwarg
}


def test_landing_planner_reads_tension_from_state_pkt():
    """
    LandingPlanner.step() accepts tension_n in the state_pkt dict.

    The winch PI formula is: speed = -v_land + k_winch * (tension - T_target)
    Higher tension --> less negative / more positive winch speed (pay out to shed tension).
    Lower tension  --> more negative winch speed (reel in to build tension).

    This invariant holds for any positive k_winch value regardless of v_land,
    T_target, or the clipping bounds.
    """
    planner = _make_landing_planner()
    tension_target = planner._T_target

    # Tensions clearly on opposite sides of the target, within the non-clipped range.
    # Using +/-50% margin so the test survives if T_target or k_winch changes.
    tension_high = tension_target * 2.5
    tension_low  = tension_target * 0.4

    cmd_high = planner.step(_make_landing_state(tension_high), _DT)
    cmd_low  = planner.step(_make_landing_state(tension_low),  _DT)

    ws_high = cmd_high["winch_speed_ms"]
    ws_low  = cmd_low["winch_speed_ms"]

    assert ws_high > ws_low, (
        f"LandingPlanner winch PI is not responding to tension_n in state_pkt. "
        f"tension_high={tension_high:.0f} N should give a greater (less negative) "
        f"winch_speed_ms ({ws_high:.4f}) than tension_low={tension_low:.0f} N "
        f"({ws_low:.4f}). If they are equal the planner is ignoring tension_n "
        f"and using the hardcoded default ({tension_target:.0f} N) instead."
    )


def test_landing_planner_reads_tension_differs_from_default():
    """
    Providing a non-default tension in state_pkt must produce a different winch
    speed from providing no tension at all (default = tension_target_n).

    The landing planner defaults tension to ``self._T_target`` when the key is
    absent from both state_pkt and kwargs, giving zero winch PI error.
    Any real tension that differs from the target must produce a different speed.
    """
    planner = _make_landing_planner()
    tension_target = planner._T_target

    state_with_tension = _make_landing_state(tension_target * 2.0)
    state_no_tension   = {k: v for k, v in state_with_tension.items()
                          if k != "tension_n"}

    cmd_with    = planner.step(state_with_tension, _DT)
    cmd_without = planner.step(state_no_tension,   _DT)

    assert cmd_with["winch_speed_ms"] != cmd_without["winch_speed_ms"], (
        "LandingPlanner must use tension_n from state_pkt when provided; "
        "the result should differ from the no-tension default."
    )


def test_landing_planner_reads_tension_from_kwargs():
    """
    LandingPlanner.step() also accepts tension_n via **kwargs (mediator path).

    The mediator calls step(state_pkt, dt, tension_n=..., tether_length_m=...)
    where state_pkt contains only MAVLink fields.  kwargs take priority over
    state_pkt values.

    Direction invariant (identical to state_pkt path):
      higher tension --> TensionPI pays out faster (greater winch_speed_ms)
      lower  tension --> TensionPI reels in faster (lesser winch_speed_ms)
    ==> ws(T_high) > ws(T_low)
    """
    planner = _make_landing_planner()
    tension_target = planner._T_target

    tension_high = tension_target * 2.5
    tension_low  = tension_target * 0.4

    # State pkt has NO tension_n (mediator format); tension comes only via kwargs.
    cmd_high = planner.step(_MEDIATOR_LANDING_STATE_PKT, _DT, tension_n=tension_high)
    cmd_low  = planner.step(_MEDIATOR_LANDING_STATE_PKT, _DT, tension_n=tension_low)

    ws_high = cmd_high["winch_speed_ms"]
    ws_low  = cmd_low["winch_speed_ms"]

    assert ws_high > ws_low, (
        f"LandingPlanner.step() is not reading tension_n from **kwargs. "
        f"tension_high={tension_high:.0f} N should give greater winch_speed_ms "
        f"({ws_high:.4f}) than tension_low={tension_low:.0f} N ({ws_low:.4f}). "
        f"If equal, kwargs are ignored and both see the state_pkt default."
    )


def test_landing_planner_kwargs_override_state_pkt():
    """
    When tension_n appears in BOTH state_pkt and kwargs, kwargs wins.

    This enforces the mediator protocol: ground-station measurements arrive as
    keyword arguments from the WinchNode; the state_pkt value (if any) is stale
    or absent.  A planner that reads state_pkt first sees wrong tension when the
    mediator passes the real value via kwargs.
    """
    planner = _make_landing_planner()
    tension_target = planner._T_target

    # state_pkt carries an old/wrong tension; kwargs carries the real value.
    wrong_tension_in_state = tension_target           # at setpoint -> zero PI error
    real_tension_in_kwargs = tension_target * 3.0     # well above -> pay-out speed

    state_with_stale = _make_landing_state(wrong_tension_in_state)

    cmd = planner.step(state_with_stale, _DT, tension_n=real_tension_in_kwargs)
    cmd_stale = planner.step(state_with_stale, _DT)   # no kwargs: uses state_pkt

    # If kwargs override correctly, cmd uses 3x tension -> positive pay-out.
    # If state_pkt is used instead, cmd uses 1x tension -> near-zero (at setpoint).
    assert cmd["winch_speed_ms"] != cmd_stale["winch_speed_ms"], (
        "LandingPlanner.step() must let kwargs override state_pkt tension. "
        "When kwargs tension differs from state_pkt tension, winch_speed_ms must differ."
    )


def test_landing_planner_returns_attitude_q_and_collective():
    """
    LandingPlanner.step() must return 'attitude_q' and 'collective_rad' for the
    mediator's internal-controller path.

    attitude_q encodes the fixed body_z so OrbitTracker slerps toward it.
    collective_rad is the descent-rate-controlled collective (no external step_vz needed).
    """
    planner = _make_landing_planner()

    cmd = planner.step(_MEDIATOR_LANDING_STATE_PKT, _DT,
                       tension_n=planner._T_target)

    assert "attitude_q" in cmd, "LandingPlanner must return 'attitude_q' for mediator"
    assert "collective_rad" in cmd, "LandingPlanner must return 'collective_rad' for mediator"

    q = cmd["attitude_q"]
    assert q.shape == (4,), f"attitude_q must be shape (4,); got {q.shape}"

    # Quaternion must encode a body_z close to the planner's fixed orientation.
    from planner import quat_apply
    bz_decoded = quat_apply(q, np.array([0.0, 0.0, -1.0]))
    bz_expected = planner._body_z_eq
    dot = float(np.dot(bz_decoded, bz_expected))
    assert dot > 0.999, (
        f"attitude_q does not encode the correct body_z. "
        f"Decoded bz={bz_decoded}, expected bz={bz_expected}, dot={dot:.4f}"
    )


def test_landing_planner_captures_body_z_from_first_step():
    """
    When initial_body_z=None, LandingPlanner captures body_z from the first
    state_pkt['body_z'] and holds it fixed for all subsequent steps.
    """
    bz_first = np.array([0.0, 0.3, -0.954])   # some unit vector
    bz_first /= np.linalg.norm(bz_first)

    planner = LandingPlanner(
        initial_body_z   = None,
        v_land           = 0.5,
        col_cruise       = 0.079,
        min_tether_m     = 2.0,
        k_winch          = 0.005,
    )

    state = {
        "body_z":          bz_first,
        "vel_ned":         np.zeros(3),
        "pos_ned":         np.array([0.0, 0.0, -10.0]),
        "tether_length_m": 10.0,
    }

    cmd = planner.step(state, _DT)

    assert planner._body_z_captured, "body_z should be captured after first step"
    assert np.allclose(planner._body_z_eq, bz_first, atol=1e-10), (
        f"Captured body_z {planner._body_z_eq} does not match first step body_z {bz_first}"
    )

    # body_z_eq in command should match captured value
    assert np.allclose(cmd["body_z_eq"], bz_first, atol=1e-10)
