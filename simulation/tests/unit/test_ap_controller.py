"""
test_ap_controller.py -- Unit tests for TensionApController.

Uses a fake linear plant: T = T_IC + K_TRUE * (coll - COLL_IC)
Elevation is not part of the fake plant — the daisy-chain correction
tests verify the controller mechanics (el_correction accumulates / decays)
independently of whether the real aero responds to elevation.

Key behaviours verified:

  Collective (TensionPI — unchanged from before):
  - Collective is held constant between receive_command() calls
  - A new command with a higher setpoint increases collective
  - PI converges to setpoint within ~15 cycles at 10 Hz with tuned gains
  - No large overshoot (monotonic convergence for static plant)
  - Aggressive 400 Hz gains produce more overshoot than 10 Hz-tuned gains
  - Comms dropout falls back to tension_ic setpoint
  - Comms restored after new command

  Daisy-chain elevation correction (new):
  - No correction when collective is not saturated
  - Floor saturation + tension above setpoint → negative el_correction (tilt down)
  - Ceiling saturation + tension below setpoint → positive el_correction (tilt up)
  - Correction is bounded by el_corr_max
  - Correction decays toward zero when collective is no longer saturated
  - Elevation angle reflects the active correction
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ap_controller import TensionApController
from pumping_planner import TensionCommand

# ── Fake physics ──────────────────────────────────────────────────────────────
K_TRUE   = 2000.0   # N/rad  (true plant sensitivity)
T_IC     = 300.0    # N
COLL_IC  = -0.240   # rad

# Tension at collective limits (from fake linear plant)
# TensionPI.COLL_MIN_RAD = -0.28 rad → T_MIN = 300 + 2000*(-0.28+0.24) = 220 N
# TensionPI.COLL_MAX_RAD =  0.00 rad → T_MAX = 300 + 2000*(0.00+0.24) = 780 N
T_AT_FLOOR   = 220.0
T_AT_CEILING = 780.0

def fake_tension(coll: float) -> float:
    return T_IC + K_TRUE * (coll - COLL_IC)

# ── Shared helpers ────────────────────────────────────────────────────────────
IC_POS = np.array([0.0, 0.0, -50.0])
# Simple rotation: body_z = [0, 0, -1] (hub above anchor, tether points up in NED)
IC_R   = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=float)
DT     = 1.0 / 400.0
DT_CMD = 0.1   # 10 Hz


def _make_ap(kp=2e-4, ki=1e-3, el_corr_ki=5e-4, el_corr_max=0.35, el_corr_tau=5.0):
    return TensionApController(
        ic_pos          = IC_POS,
        mass_kg         = 5.0,
        slew_rate_rad_s = 0.4,
        warm_coll_rad   = COLL_IC,
        tension_ic      = T_IC,
        kp              = kp,
        ki              = ki,
        el_corr_ki      = el_corr_ki,
        el_corr_max     = el_corr_max,
        el_corr_tau     = el_corr_tau,
    )


def _cmd(setpoint, measured, alt=50.0, phase="reel-out"):
    return TensionCommand(
        tension_setpoint_n = setpoint,
        tension_measured_n = measured,
        alt_m              = alt,
        phase              = phase,
    )


def _run_cycles(ap, n_cycles, setpoint):
    """Simulate n_cycles of 10 Hz commands with fake linear physics."""
    T    = T_IC
    peak = T
    for _ in range(n_cycles):
        ap.receive_command(_cmd(setpoint, T), DT_CMD)
        for _ in range(40):
            coll, _, _ = ap.step(IC_POS, IC_R, DT)
        T    = fake_tension(coll)
        peak = max(peak, T)
    return T, peak


# ---------------------------------------------------------------------------
# Collective held between commands
# ---------------------------------------------------------------------------

class TestCollectiveHeld:

    def test_collective_unchanged_between_commands(self):
        """Between receive_command() calls, step() returns the same collective."""
        ap = _make_ap()
        ap.receive_command(_cmd(T_IC, T_IC), DT_CMD)
        coll0, _, _ = ap.step(IC_POS, IC_R, DT)
        for _ in range(38):
            coll, _, _ = ap.step(IC_POS, IC_R, DT)
            assert coll == coll0

    def test_higher_setpoint_increases_collective(self):
        """A command with a higher setpoint causes a higher (less negative) collective."""
        ap = _make_ap()
        ap.receive_command(_cmd(T_IC, T_IC), DT_CMD)
        coll_eq, _, _ = ap.step(IC_POS, IC_R, DT)

        ap.receive_command(_cmd(T_IC + 100.0, T_IC), DT_CMD)
        coll_high, _, _ = ap.step(IC_POS, IC_R, DT)
        assert coll_high > coll_eq


# ---------------------------------------------------------------------------
# Convergence with 10 Hz-tuned gains
# ---------------------------------------------------------------------------

class TestConvergence:

    def test_converges_to_setpoint(self):
        """PI reaches within 5 N of setpoint within 20 cycles at 10 Hz."""
        ap = _make_ap()
        setpoint = T_IC + 135.0   # 435 N
        T_final, _ = _run_cycles(ap, n_cycles=20, setpoint=setpoint)
        assert abs(T_final - setpoint) < 5.0, f"did not converge: T_final={T_final:.1f} N"

    def test_no_overshoot_for_static_plant(self):
        """Monotonic convergence: peak tension stays within 10 N above setpoint."""
        ap = _make_ap()
        setpoint = T_IC + 135.0
        _, peak  = _run_cycles(ap, n_cycles=20, setpoint=setpoint)
        assert peak < setpoint + 10.0, f"overshoot too large: peak={peak:.1f} N"

    def test_aggressive_gains_overshoot_more(self):
        """Original 400 Hz gains produce larger overshoot than 10 Hz-tuned gains."""
        setpoint = T_IC + 135.0
        _, peak_tuned = _run_cycles(_make_ap(kp=2e-4, ki=1e-3), 15, setpoint)
        _, peak_agg   = _run_cycles(_make_ap(kp=5e-4, ki=1e-4), 15, setpoint)
        assert peak_agg > peak_tuned, (
            f"aggressive gains should overshoot more: "
            f"tuned={peak_tuned:.1f} N  agg={peak_agg:.1f} N"
        )


# ---------------------------------------------------------------------------
# Comms dropout
# ---------------------------------------------------------------------------

class TestCommsDropout:

    def test_dropout_resets_setpoint_to_tension_ic(self):
        """After comms timeout, tension setpoint falls back to tension_ic."""
        ap = _make_ap()
        ap.receive_command(_cmd(T_IC + 100.0, T_IC + 50.0), DT_CMD)
        assert ap.tension_setpoint > T_IC

        for _ in range(int(TensionApController.CMD_TIMEOUT_S / DT) + 10):
            ap.step(IC_POS, IC_R, DT)

        assert not ap.comms_ok
        assert ap.tension_setpoint == pytest.approx(T_IC)

    def test_comms_restored_after_new_command(self):
        """After dropout, a new receive_command() restores comms_ok."""
        ap = _make_ap()
        for _ in range(int(TensionApController.CMD_TIMEOUT_S / DT) + 10):
            ap.step(IC_POS, IC_R, DT)
        assert not ap.comms_ok

        ap.receive_command(_cmd(T_IC, T_IC), DT_CMD)
        assert ap.comms_ok


# ---------------------------------------------------------------------------
# Daisy-chain elevation correction
# ---------------------------------------------------------------------------

class TestDaisyChain:

    def test_no_correction_when_not_saturated(self):
        """When collective has range, el_correction stays at zero."""
        ap = _make_ap()
        setpoint = T_IC + 135.0   # 435 N — well within collective range
        T = T_IC
        for _ in range(15):
            ap.receive_command(_cmd(setpoint, T), DT_CMD)
            for _ in range(40):
                coll, _, _ = ap.step(IC_POS, IC_R, DT)
            T = fake_tension(coll)
        assert not ap.coll_saturated
        assert abs(ap.el_correction_rad) < 1e-6

    def test_floor_saturation_builds_negative_correction(self):
        """Below T_AT_FLOOR: collective pins at floor, el_correction goes negative."""
        ap = _make_ap()
        # Setpoint below what floor can achieve → collective saturates
        setpoint = T_AT_FLOOR - 30.0   # 190 N; T_AT_FLOOR = 220 N
        for _ in range(20):
            ap.receive_command(_cmd(setpoint, T_AT_FLOOR), DT_CMD)
            for _ in range(40):
                ap.step(IC_POS, IC_R, DT)
        assert ap.coll_saturated
        assert ap.el_correction_rad < -1e-4

    def test_ceiling_saturation_builds_positive_correction(self):
        """Above T_AT_CEILING: collective pins at ceiling, el_correction goes positive."""
        ap = _make_ap()
        # Warm-start integrator to ceiling: run with very high setpoint until saturated
        setpoint = T_AT_CEILING + 50.0   # 830 N; integral climbs ~5/step → ~46 steps to ceiling
        for _ in range(60):
            ap.receive_command(_cmd(setpoint, T_AT_CEILING), DT_CMD)
            for _ in range(40):
                ap.step(IC_POS, IC_R, DT)
        assert ap.coll_saturated
        assert ap.el_correction_rad > 1e-4

    def test_correction_bounded_by_el_corr_max(self):
        """el_correction never exceeds el_corr_max regardless of cycles."""
        max_corr = 0.10   # deliberately small for this test
        ap = _make_ap(el_corr_max=max_corr)
        setpoint = T_AT_FLOOR - 30.0
        for _ in range(200):
            ap.receive_command(_cmd(setpoint, T_AT_FLOOR), DT_CMD)
            for _ in range(40):
                ap.step(IC_POS, IC_R, DT)
        assert abs(ap.el_correction_rad) <= max_corr + 1e-9

    def test_correction_decays_when_not_saturated(self):
        """After saturation clears, el_correction decays toward zero."""
        tau = 2.0
        ap  = _make_ap(el_corr_tau=tau)

        # Build up a negative correction
        for _ in range(30):
            ap.receive_command(_cmd(T_AT_FLOOR - 30.0, T_AT_FLOOR), DT_CMD)
            for _ in range(40):
                ap.step(IC_POS, IC_R, DT)
        corr_peak = ap.el_correction_rad
        assert corr_peak < -1e-3

        # Switch to a setpoint that doesn't saturate collective
        T = T_IC
        for _ in range(15):
            ap.receive_command(_cmd(T_IC + 135.0, T), DT_CMD)
            for _ in range(40):
                coll, _, _ = ap.step(IC_POS, IC_R, DT)
            T = fake_tension(coll)

        # Correction must have decayed substantially
        assert ap.el_correction_rad > corr_peak   # moved toward zero
        assert abs(ap.el_correction_rad) < abs(corr_peak) * 0.5

    def test_elevation_lower_with_negative_correction(self):
        """After floor saturation, elevation_rad is below the altitude-based target."""
        ap = _make_ap()
        tlen = float(np.linalg.norm(IC_POS))
        alt  = float(-IC_POS[2])
        altitude_el = float(np.arcsin(alt / tlen))   # what altitude alone gives

        # Build negative correction
        for _ in range(30):
            ap.receive_command(_cmd(T_AT_FLOOR - 30.0, T_AT_FLOOR), DT_CMD)
            # Run enough 400 Hz steps for elevation to slew toward corrected target
            for _ in range(400):
                ap.step(IC_POS, IC_R, DT)

        assert ap.el_correction_rad < 0
        assert ap.elevation_rad < altitude_el
