"""
test_planner_protocol.py -- Protocol conformance tests for DeschutterPlanner.step().

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
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import config as _cfg
from planner import DeschutterPlanner, WindEstimator


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
