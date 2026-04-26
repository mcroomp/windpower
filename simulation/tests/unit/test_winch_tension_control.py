"""
test_winch_tension_control.py — Unit tests for WinchController.

The winch is tension-controlled: cruise speed is proportional to tension
error, and a trapezoidal motion profile ensures smooth arrival at the
target length.

Covers:
  Reel-out (generator mode):
    - speed proportional to tension above target
    - zero speed when tension at or below target
    - reaches target length and stops
    - speed capped at v_max_out

  Reel-in (motor mode):
    - speed proportional to tension below target (slack boost)
    - zero speed when tension at or above target (kite resisting)
    - returns to target length and stops
    - speed capped at v_max_in

  Motion profile:
    - accelerates from rest rather than instant jump
    - decelerates smoothly near target (no overshoot)
    - speed is zero at target length
    - phase reversal decelerates before accelerating in new direction

  Tension changes mid-move:
    - tension dropping below target during reel-out causes smooth stop
    - tension spiking above target during reel-in causes smooth stop
    - tension recovering resumes motion

  Bounds:
    - min_length prevents over-reel-in
    - max_length prevents over-reel-out

  Landing scenario:
    - long reel-in at low target tension arrives correctly
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from winch import WinchController

DT          = 1.0 / 400.0
REST_INIT   = 100.0
DELTA_L     = 12.0
KP          = 0.005    # (m/s)/N
V_MAX_OUT   = 0.40     # m/s
V_MAX_IN    = 0.80     # m/s
ACCEL       = 0.5      # m/s²
T_TARGET    = 300.0    # nominal target tension [N]

# Generous step counts so the profile has time to ramp and decelerate
STEPS_RAMP  = int(max(V_MAX_OUT, V_MAX_IN) / ACCEL / DT) + 20
STEPS_OUT   = int(DELTA_L / V_MAX_OUT / DT) + STEPS_RAMP * 2 + 200
STEPS_IN    = int(DELTA_L / V_MAX_IN  / DT) + STEPS_RAMP * 2 + 200


def _make(rest=REST_INIT, min_length=2.0, max_length=300.0):
    return WinchController(
        rest_length     = rest,
        kp_tension      = KP,
        v_max_out       = V_MAX_OUT,
        v_max_in        = V_MAX_IN,
        accel_limit_ms2 = ACCEL,
        min_length      = min_length,
        max_length      = max_length,
    )


def _run(w, n, tension):
    for _ in range(n):
        w.step(tension, DT)


def _reel_out(w, delta=DELTA_L, tension=T_TARGET + 100):
    """Drive winch out by delta metres at a tension above target."""
    w.set_target(w.rest_length + delta, T_TARGET)
    _run(w, STEPS_OUT, tension)


# ---------------------------------------------------------------------------
# Reel-out — generator mode
# ---------------------------------------------------------------------------

class TestReelOut:

    def test_tension_above_target_produces_outward_speed(self):
        """Tension above target → positive (pay-out) speed."""
        w = _make()
        w.set_target(REST_INIT + DELTA_L, T_TARGET)
        _run(w, STEPS_RAMP, T_TARGET + 100)
        assert w.speed_ms > 0

    def test_tension_at_target_gives_zero_cruise(self):
        """Tension exactly at target → cruise speed = 0 → winch decelerates to stop."""
        w = _make()
        w.set_target(REST_INIT + DELTA_L, T_TARGET)
        _run(w, STEPS_RAMP * 3, T_TARGET)
        assert abs(w.speed_ms) < 0.01

    def test_speed_capped_at_v_max_out(self):
        """Even with very high tension error, speed is bounded by v_max_out."""
        w = _make()
        w.set_target(REST_INIT + DELTA_L, T_TARGET)
        _run(w, STEPS_RAMP * 3, T_TARGET + 10_000)
        assert w.speed_ms <= V_MAX_OUT + 1e-6

    def test_reaches_target_length(self):
        """Winch pays out delta_l and stops at target length."""
        w = _make()
        w.set_target(REST_INIT + DELTA_L, T_TARGET)
        _run(w, STEPS_OUT, T_TARGET + 100)
        assert w.rest_length == pytest.approx(REST_INIT + DELTA_L, abs=0.05)

    def test_zero_speed_at_target(self):
        """Motor is at rest once target length is reached."""
        w = _make()
        w.set_target(REST_INIT + DELTA_L, T_TARGET)
        _run(w, STEPS_OUT, T_TARGET + 100)
        assert abs(w.speed_ms) < 0.01

    def test_no_further_movement_after_target(self):
        """At target, continued steps produce no movement."""
        w = _make()
        w.set_target(REST_INIT + DELTA_L, T_TARGET)
        _run(w, STEPS_OUT, T_TARGET + 100)
        length_stopped = w.rest_length
        _run(w, 200, T_TARGET + 100)
        assert w.rest_length == pytest.approx(length_stopped, abs=1e-6)

    def test_low_tension_during_reel_out_stops_winch(self):
        """Tension dropping below target mid reel-out: winch decelerates to stop."""
        w = _make()
        w.set_target(REST_INIT + DELTA_L, T_TARGET)
        _run(w, STEPS_RAMP + 5, T_TARGET + 100)   # ramp up
        _run(w, STEPS_RAMP * 3, T_TARGET - 50)    # tension falls below target
        assert abs(w.speed_ms) < 0.01


# ---------------------------------------------------------------------------
# Reel-in — motor mode
# ---------------------------------------------------------------------------

class TestReelIn:

    def test_tension_below_target_produces_reel_in_speed(self):
        """Tension below target → negative (reel-in) speed."""
        w = _make()
        _reel_out(w)
        w.set_target(REST_INIT, T_TARGET)
        _run(w, STEPS_RAMP, T_TARGET - 100)
        assert w.speed_ms < 0

    def test_tension_at_or_above_target_gives_zero_cruise(self):
        """Tension at or above target during reel-in → cruise = 0 → winch stops."""
        w = _make()
        _reel_out(w)
        w.set_target(REST_INIT, T_TARGET)
        _run(w, STEPS_RAMP * 3, T_TARGET)
        assert abs(w.speed_ms) < 0.01

    def test_slack_tension_boosts_speed(self):
        """Very low tension (slack) drives reel-in toward v_max_in."""
        w = _make()
        _reel_out(w)
        w.set_target(REST_INIT, T_TARGET)
        _run(w, STEPS_RAMP * 3, 0.0)   # fully slack → max cruise
        assert abs(w.speed_ms) > V_MAX_OUT   # faster than reel-out speed

    def test_speed_capped_at_v_max_in(self):
        """Even fully slack, speed is bounded by v_max_in."""
        w = _make()
        _reel_out(w)
        w.set_target(REST_INIT, T_TARGET)
        _run(w, STEPS_RAMP * 3, 0.0)
        assert abs(w.speed_ms) <= V_MAX_IN + 1e-6

    def test_returns_to_start_length(self):
        """Full reel-in returns rest_length to the value at reel-out start."""
        w = _make()
        start = w.rest_length
        _reel_out(w)
        w.set_target(start, T_TARGET)
        _run(w, STEPS_IN, 0.0)   # zero tension → max slack boost → v_max_in cruise
        assert w.rest_length == pytest.approx(start, abs=0.05)

    def test_zero_speed_at_target(self):
        """Motor is at rest once target length is reached."""
        w = _make()
        start = w.rest_length
        _reel_out(w)
        w.set_target(start, T_TARGET)
        _run(w, STEPS_IN, 0.0)   # zero tension → max slack boost → v_max_in cruise
        assert abs(w.speed_ms) < 0.01

    def test_high_tension_during_reel_in_stops_winch(self):
        """Tension spiking above target mid reel-in: winch decelerates (kite resisting)."""
        w = _make()
        _reel_out(w)
        w.set_target(REST_INIT, T_TARGET)
        _run(w, STEPS_RAMP + 5, T_TARGET - 100)   # ramp up
        _run(w, STEPS_RAMP * 3, T_TARGET + 50)    # tension above target
        assert abs(w.speed_ms) < 0.01


# ---------------------------------------------------------------------------
# Motion profile
# ---------------------------------------------------------------------------

class TestMotionProfile:

    def test_reel_out_ramps_up(self):
        """Speed after first step is accel*dt, not instant full speed."""
        w = _make()
        w.set_target(REST_INIT + DELTA_L, T_TARGET)
        w.step(T_TARGET + 1000, DT)   # large tension error → would be v_max without ramp
        assert abs(w.speed_ms) == pytest.approx(ACCEL * DT, rel=1e-3)

    def test_reel_in_ramps_up(self):
        """Reel-in speed ramps from zero on first step."""
        w = _make()
        _reel_out(w)
        w.set_target(REST_INIT, T_TARGET)
        w.step(0.0, DT)   # fully slack → would be v_max_in without ramp
        assert abs(w.speed_ms) == pytest.approx(ACCEL * DT, rel=1e-3)

    def test_no_overshoot_on_arrival(self):
        """Winch decelerates and does not overshoot the target length."""
        w = _make()
        target = REST_INIT + DELTA_L
        w.set_target(target, T_TARGET)
        _run(w, STEPS_OUT, T_TARGET + 100)
        assert w.rest_length <= target + 0.05

    def test_phase_reversal_decelerates_first(self):
        """Reversing target mid-move causes deceleration before direction change."""
        w = _make()
        w.set_target(REST_INIT + DELTA_L, T_TARGET)
        _run(w, STEPS_RAMP + 5, T_TARGET + 100)   # moving outward at cruise
        speed_before = w.speed_ms
        assert speed_before > 0.1
        # Now reverse: reel in
        w.set_target(REST_INIT, T_TARGET)
        w.step(0.0, DT)   # one step: still decelerating outward or starting to reverse
        # Speed magnitude should have decreased (not instantly at full reel-in speed)
        assert abs(w.speed_ms) < abs(speed_before) + ACCEL * DT + 1e-6

    def test_tension_recovery_resumes_motion(self):
        """After tension drops and stops the winch, tension recovery resumes."""
        w = _make()
        w.set_target(REST_INIT + DELTA_L, T_TARGET)
        _run(w, STEPS_RAMP + 5, T_TARGET + 100)
        _run(w, STEPS_RAMP * 3, T_TARGET - 50)    # winch stops
        assert abs(w.speed_ms) < 0.01
        length_stopped = w.rest_length
        _run(w, STEPS_RAMP * 3, T_TARGET + 100)   # tension recovered
        assert w.rest_length > length_stopped      # moving again


# ---------------------------------------------------------------------------
# Bounds
# ---------------------------------------------------------------------------

class TestBounds:

    def test_min_length_respected(self):
        """Winch never reels in past min_length."""
        w = _make(rest=3.0, min_length=2.0)
        w.set_target(0.0, T_TARGET)   # impossible target below min
        _run(w, STEPS_IN, 0.0)
        assert w.rest_length >= 2.0

    def test_max_length_respected(self):
        """Winch never pays out past max_length."""
        w = _make(rest=298.0, max_length=300.0)
        w.set_target(400.0, T_TARGET)   # impossible target above max
        _run(w, STEPS_OUT, T_TARGET + 200)
        assert w.rest_length <= 300.0


# ---------------------------------------------------------------------------
# Landing scenario
# ---------------------------------------------------------------------------

class TestLanding:

    def test_landing_reel_in_to_min_length(self):
        """
        Landing: reel in from 20 m to min_length (2 m) at low target tension.
        With 0 N measured tension (fully slack), cruise = kp*T_land = 0.5 m/s.
        The winch should arrive cleanly without overshoot.
        """
        w = _make(rest=20.0, min_length=2.0)
        min_len = 2.0
        T_land  = 100.0
        v_cruise = KP * T_land   # 0.005 * 100 = 0.5 m/s (below v_max_in)
        w.set_target(min_len, T_land)
        steps = int(18.0 / v_cruise / DT) + STEPS_RAMP * 2 + 500
        _run(w, steps, 0.0)   # fully slack → cruise at kp*T_land
        assert w.rest_length == pytest.approx(min_len, abs=0.1)
        assert abs(w.speed_ms) < 0.01

    def test_landing_never_pays_out(self):
        """During landing reel-in, tension above target stops the motor — never pays out."""
        w = _make(rest=100.0, min_length=2.0)
        T_land = 100.0
        w.set_target(2.0, T_land)
        # Apply very high tension (kite resisting strongly)
        _run(w, STEPS_RAMP * 3, T_land + 300)
        assert w.speed_ms <= 0.0   # never positive (never pays out)
