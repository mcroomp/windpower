"""
test_ap_controller.py -- Unit tests for TensionApController.

Uses a fake linear plant: T = T_IC + K_TRUE * (coll - COLL_IC)

Key behaviours verified:

  Collective (TensionPI):
  - Collective is held constant between receive_command() calls
  - A new command with a higher setpoint increases collective
  - PI converges to setpoint within ~15 cycles at 10 Hz with tuned gains
  - No large overshoot (monotonic convergence for static plant)
  - Aggressive 400 Hz gains produce more overshoot than 10 Hz-tuned gains
  - Comms dropout falls back to tension_ic setpoint
  - Comms restored after new command
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ap_controller  import TensionApController
from physics_core   import HubObservation
from pumping_planner import TensionCommand

# ── Fake physics ──────────────────────────────────────────────────────────────
K_TRUE   = 2000.0   # N/rad  (true plant sensitivity)
T_IC     = 300.0    # N
COLL_IC  = -0.240   # rad

def fake_tension(coll: float) -> float:
    return T_IC + K_TRUE * (coll - COLL_IC)

# ── Shared helpers ────────────────────────────────────────────────────────────
IC_POS = np.array([0.0, 0.0, -50.0])
# Simple rotation: body_z = [0, 0, -1] (hub above anchor, tether points up in NED)
IC_R   = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=float)
DT     = 1.0 / 400.0
DT_CMD = 0.1   # 10 Hz

IC_OBS = HubObservation(
    R          = IC_R,
    pos        = IC_POS,
    vel        = np.zeros(3),
    body_z     = IC_R[:, 2],
    gyro       = np.zeros(3),
    omega_spin = 0.0,
)


def _make_ap(kp=2e-4, ki=1e-3, kd=0.0, kp_outer=2.5):
    return TensionApController(
        ic_pos          = IC_POS,
        mass_kg         = 5.0,
        slew_rate_rad_s = 0.4,
        warm_coll_rad   = COLL_IC,
        tension_ic      = T_IC,
        cmd_timeout_s   = 0.5,
        kp              = kp,
        ki              = ki,
        kd              = kd,
        coll_min_rad    = -0.28,
        coll_max_rad    =  0.10,
        kp_outer        = kp_outer,
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
            coll, _, _ = ap.step(IC_OBS, DT)
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
        coll0, _, _ = ap.step(IC_OBS, DT)
        for _ in range(38):
            coll, _, _ = ap.step(IC_OBS, DT)
            assert coll == coll0

    def test_higher_setpoint_increases_collective(self):
        """A command with a higher setpoint causes a higher (less negative) collective."""
        ap = _make_ap()
        ap.receive_command(_cmd(T_IC, T_IC), DT_CMD)
        coll_eq, _, _ = ap.step(IC_OBS, DT)

        ap.receive_command(_cmd(T_IC + 100.0, T_IC), DT_CMD)
        coll_high, _, _ = ap.step(IC_OBS, DT)
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
            ap.step(IC_OBS, DT)

        assert not ap.comms_ok
        assert ap.tension_setpoint == pytest.approx(T_IC)

    def test_comms_restored_after_new_command(self):
        """After dropout, a new receive_command() restores comms_ok."""
        ap = _make_ap()
        for _ in range(int(TensionApController.CMD_TIMEOUT_S / DT) + 10):
            ap.step(IC_OBS, DT)
        assert not ap.comms_ok

        ap.receive_command(_cmd(T_IC, T_IC), DT_CMD)
        assert ap.comms_ok


