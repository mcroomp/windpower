"""
test_winch_tension_control.py — Unit tests for WinchController.

Covers:
  Program mode (set_phase):
    - reel-out pays out delta_l at v_reel_out then holds
    - reel-in returns to start_length at v_reel_in then holds
    - full cycle closes (rest_length same before and after)
    - safety layer slows reel-out when tension approaches T_hard_max
    - safety layer slows reel-in when tension approaches T_hard_min (slack)
    - transition/hold phases produce no movement
    - successive cycles remain closed

  Legacy 3-zone mode (no set_phase):
    - reels out when T > T_max_n
    - reels in when T < T_min_n
    - holds in deadband
    - length bounds respected
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from winch import WinchController

DT          = 1.0 / 400.0
REST_INIT   = 100.0
DELTA_L     = 12.0
V_OUT       = 0.40
V_IN        = 0.80
T_SOFT_MAX  = 470.0
T_HARD_MAX  = 496.0
T_SOFT_MIN  =  30.0
T_HARD_MIN  =  10.0
T_NOMINAL   = 300.0   # safe mid-range tension


def _make_winch(rest_length=REST_INIT):
    return WinchController(
        rest_length = rest_length,
        delta_l     = DELTA_L,
        v_reel_out  = V_OUT,
        v_reel_in   = V_IN,
        T_soft_max  = T_SOFT_MAX,
        T_hard_max  = T_HARD_MAX,
        T_soft_min  = T_SOFT_MIN,
        T_hard_min  = T_HARD_MIN,
        min_length  = 2.0,
    )


# ---------------------------------------------------------------------------
# Program mode — reel-out
# ---------------------------------------------------------------------------

class TestReelOut:

    def test_reels_out_at_nominal_speed(self):
        """Reel-out at safe tension advances at v_reel_out."""
        w = _make_winch()
        w.set_phase("reel-out")
        w.step(T_NOMINAL, DT)
        assert w.rest_length == pytest.approx(REST_INIT + V_OUT * DT, rel=1e-6)

    def test_stops_at_delta_l(self):
        """Reel-out stops once delta_l has been paid out."""
        w = _make_winch()
        w.set_phase("reel-out")
        steps = int(DELTA_L / V_OUT / DT) + 200
        for _ in range(steps):
            w.step(T_NOMINAL, DT)
        assert w.rest_length == pytest.approx(REST_INIT + DELTA_L, abs=1e-3)

    def test_safety_slows_at_soft_max(self):
        """Tension above T_soft_max reduces reel-out speed proportionally."""
        w = _make_winch()
        w.set_phase("reel-out")
        T_mid = (T_SOFT_MAX + T_HARD_MAX) / 2.0   # halfway → factor = 0.5
        w.step(T_mid, DT)
        expected_full = V_OUT * DT
        assert w.rest_length - REST_INIT < expected_full * 0.6

    def test_safety_stops_at_hard_max(self):
        """Tension at or above T_hard_max: winch stops completely."""
        w = _make_winch()
        w.set_phase("reel-out")
        w.step(T_HARD_MAX, DT)
        assert w.rest_length == pytest.approx(REST_INIT, abs=1e-9)

    def test_no_movement_after_target_reached(self):
        """Once target length reached, further steps produce no movement."""
        w = _make_winch()
        w.set_phase("reel-out")
        steps = int(DELTA_L / V_OUT / DT) + 400
        for _ in range(steps):
            w.step(T_NOMINAL, DT)
        length_at_stop = w.rest_length
        for _ in range(400):
            w.step(T_NOMINAL, DT)
        assert w.rest_length == pytest.approx(length_at_stop, abs=1e-9)


# ---------------------------------------------------------------------------
# Program mode — reel-in
# ---------------------------------------------------------------------------

class TestReelIn:

    def test_reels_in_at_nominal_speed(self):
        """Reel-in at safe tension moves at v_reel_in."""
        w = _make_winch()
        w.set_phase("reel-out")
        for _ in range(int(DELTA_L / V_OUT / DT) + 200):
            w.step(T_NOMINAL, DT)
        length_after_out = w.rest_length
        w.set_phase("reel-in")
        w.step(T_NOMINAL, DT)
        assert length_after_out - w.rest_length == pytest.approx(V_IN * DT, rel=1e-4)

    def test_returns_to_start_length(self):
        """Full reel-in returns rest_length to the value recorded at reel-out start."""
        w = _make_winch()
        start = w.rest_length
        w.set_phase("reel-out")
        for _ in range(int(DELTA_L / V_OUT / DT) + 200):
            w.step(T_NOMINAL, DT)
        w.set_phase("reel-in")
        for _ in range(int(DELTA_L / V_IN / DT) + 400):
            w.step(T_NOMINAL, DT)
        assert w.rest_length == pytest.approx(start, abs=1e-3)

    def test_safety_slows_near_slack(self):
        """Tension below T_soft_min reduces reel-in speed."""
        w = _make_winch()
        w.set_phase("reel-out")
        for _ in range(int(DELTA_L / V_OUT / DT) + 200):
            w.step(T_NOMINAL, DT)
        length_pre = w.rest_length
        w.set_phase("reel-in")
        T_mid = (T_SOFT_MIN + T_HARD_MIN) / 2.0   # halfway → factor ≈ 0.5
        w.step(T_mid, DT)
        actual_delta = length_pre - w.rest_length
        assert actual_delta < V_IN * DT * 0.6

    def test_safety_stops_at_hard_min(self):
        """Tension at or below T_hard_min: winch stops (slack prevention)."""
        w = _make_winch()
        w.set_phase("reel-out")
        for _ in range(int(DELTA_L / V_OUT / DT) + 200):
            w.step(T_NOMINAL, DT)
        length_pre = w.rest_length
        w.set_phase("reel-in")
        w.step(T_HARD_MIN, DT)
        assert w.rest_length == pytest.approx(length_pre, abs=1e-9)

    def test_no_movement_after_target_reached(self):
        """Once back at start length, further reel-in steps do nothing."""
        w = _make_winch()
        w.set_phase("reel-out")
        for _ in range(int(DELTA_L / V_OUT / DT) + 200):
            w.step(T_NOMINAL, DT)
        w.set_phase("reel-in")
        for _ in range(int(DELTA_L / V_IN / DT) + 400):
            w.step(T_NOMINAL, DT)
        length_at_home = w.rest_length
        for _ in range(400):
            w.step(T_NOMINAL, DT)
        assert w.rest_length == pytest.approx(length_at_home, abs=1e-9)


# ---------------------------------------------------------------------------
# Program mode — transition / hold / cycle closure
# ---------------------------------------------------------------------------

class TestCycleClosure:

    def test_transition_holds_length(self):
        """Transition phase: no movement at any tension."""
        w = _make_winch()
        w.set_phase("reel-out")
        for _ in range(int(DELTA_L / V_OUT / DT) + 200):
            w.step(T_NOMINAL, DT)
        w.set_phase("transition")
        length = w.rest_length
        for _ in range(400):
            w.step(T_NOMINAL, DT)
        assert w.rest_length == pytest.approx(length, abs=1e-9)

    def test_hold_holds_length(self):
        """Hold phase: no movement."""
        w = _make_winch()
        w.set_phase("hold")
        for _ in range(400):
            w.step(T_NOMINAL, DT)
        assert w.rest_length == pytest.approx(REST_INIT, abs=1e-9)

    def test_full_cycle_closes(self):
        """After one complete reel-out + reel-in cycle, length is back to start."""
        w = _make_winch()
        start = w.rest_length
        w.set_phase("reel-out")
        for _ in range(int(DELTA_L / V_OUT / DT) + 400):
            w.step(T_NOMINAL, DT)
        w.set_phase("transition")
        for _ in range(200):
            w.step(T_NOMINAL, DT)
        w.set_phase("reel-in")
        for _ in range(int(DELTA_L / V_IN / DT) + 400):
            w.step(T_NOMINAL, DT)
        assert w.rest_length == pytest.approx(start, abs=1e-3)

    def test_successive_cycles_stay_closed(self):
        """Three consecutive cycles each return to the same start length."""
        w = _make_winch()
        start = w.rest_length
        for _ in range(3):
            w.set_phase("reel-out")
            for _ in range(int(DELTA_L / V_OUT / DT) + 400):
                w.step(T_NOMINAL, DT)
            w.set_phase("reel-in")
            for _ in range(int(DELTA_L / V_IN / DT) + 400):
                w.step(T_NOMINAL, DT)
        assert w.rest_length == pytest.approx(start, abs=1e-3)

    def test_reel_in_faster_than_reel_out(self):
        """v_reel_in > v_reel_out: reel-in completes in fewer steps."""
        w = _make_winch()
        w.set_phase("reel-out")
        steps_out = 0
        while w.rest_length < REST_INIT + DELTA_L - 0.01:
            w.step(T_NOMINAL, DT)
            steps_out += 1
        w.set_phase("reel-in")
        steps_in = 0
        while w.rest_length > REST_INIT + 0.01:
            w.step(T_NOMINAL, DT)
            steps_in += 1
        assert steps_in < steps_out


# ---------------------------------------------------------------------------
# Legacy 3-zone mode (no set_phase)
# ---------------------------------------------------------------------------

class TestLegacyMode:

    def _make_legacy(self, rest=REST_INIT):
        return WinchController(
            rest_length   = rest,
            T_min_n       = 150.0,
            T_max_n       = 450.0,
            v_reel_in_ms  = 0.4,
            v_reel_out_ms = 0.4,
            kp            = 0.01,
            min_length    = 2.0,
        )

    def test_reels_out_above_T_max(self):
        w = self._make_legacy()
        w.step(460.0, DT)
        assert w.rest_length > REST_INIT

    def test_reels_in_below_T_min(self):
        w = self._make_legacy()
        w.step(100.0, DT)
        assert w.rest_length < REST_INIT

    def test_holds_in_deadband(self):
        w = self._make_legacy()
        w.step(300.0, DT)
        assert w.rest_length == pytest.approx(REST_INIT, abs=1e-9)

    def test_length_respects_min_bound(self):
        w = self._make_legacy(rest=2.1)
        for _ in range(400):
            w.step(10.0, DT)
        assert w.rest_length >= 2.0
