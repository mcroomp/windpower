"""Unit tests for MockArdupilot's pumping Python altitude-PID collective mode."""
import sys
from pathlib import Path

import numpy as np
import pytest


from simulation.physics_core import HubObservation
from simulation.pumping_planner import TensionCommand
from tests.common.mock_ardupilot import MockArdupilot

# ── Shared helpers ────────────────────────────────────────────────────────────
from simulation.param_defaults import load_collective_phys_range
_COL_MIN, _COL_MAX = load_collective_phys_range()
T_IC     = 300.0   # N
COLL_IC  = -0.240  # rad IC collective (test-specific value)
THRUST_IC = (COLL_IC - _COL_MIN) / (_COL_MAX - _COL_MIN)

IC_POS = np.array([0.0, 0.0, -50.0])
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


def _make_ap():
    return MockArdupilot.for_python(
        mode="pumping",
        ic_pos          = IC_POS,
        mass_kg         = 5.0,
        slew_rate_rad_s = 0.4,
        warm_thrust     = THRUST_IC,
        tension_ic      = T_IC,
        cmd_timeout_s   = 0.5,
        wind            = np.zeros(3),
        dt              = DT,
    )


def _cmd(target=T_IC, alt=50.0, phase="reel-out"):
    return TensionCommand(
        tension_target_n   = target,
        alt_m              = alt,
        phase              = phase,
    )

# ---------------------------------------------------------------------------
# Altitude PID collective
# ---------------------------------------------------------------------------

class TestAltitudePidCollective:

    def test_collective_unchanged_at_target_altitude(self):
        """At target altitude with zero vz, altitude PID returns warm collective."""
        ap = _make_ap()
        ap.receive_command(_cmd(), DT_CMD)
        coll0, _, _ = ap.controller_step(IC_OBS, DT)
        for _ in range(38):
            coll, _, _ = ap.controller_step(IC_OBS, DT)
            assert coll == coll0
        assert coll0 == pytest.approx(THRUST_IC)

    def test_low_altitude_increases_collective(self):
        """If hub is below target altitude, altitude PID increases collective."""
        ap = _make_ap()
        ap.receive_command(_cmd(alt=50.0), DT_CMD)
        obs = HubObservation(
            R=IC_R,
            pos=np.array([0.0, 0.0, -48.0]),
            vel=np.zeros(3),
            body_z=IC_R[:, 2],
            gyro=np.zeros(3),
            omega_spin=0.0,
        )
        coll, _, _ = ap.controller_step(obs, DT)
        assert coll > THRUST_IC

    def test_high_altitude_decreases_collective(self):
        """If hub is above target altitude, altitude PID decreases collective."""
        ap = _make_ap()
        ap.receive_command(_cmd(alt=50.0), DT_CMD)
        obs = HubObservation(
            R=IC_R,
            pos=np.array([0.0, 0.0, -52.0]),
            vel=np.zeros(3),
            body_z=IC_R[:, 2],
            gyro=np.zeros(3),
            omega_spin=0.0,
        )
        coll, _, _ = ap.controller_step(obs, DT)
        assert coll < THRUST_IC


# ---------------------------------------------------------------------------
# Comms dropout
# ---------------------------------------------------------------------------

class TestCommsDropout:

    def test_dropout_freezes_collective(self):
        """After comms timeout, comms_ok is False; collective stays at last value."""
        ap = _make_ap()
        ap.receive_command(_cmd(), DT_CMD)
        coll_before, _, _ = ap.controller_step(IC_OBS, DT)

        timeout_s = MockArdupilot.python_constants("pumping")["CMD_TIMEOUT_S"]
        for _ in range(int(timeout_s / DT) + 10):
            ap.controller_step(IC_OBS, DT)

        assert not ap.comms_ok
        # collective should stay at whatever was last commanded
        coll_after, _, _ = ap.controller_step(IC_OBS, DT)
        assert coll_after == pytest.approx(coll_before)

    def test_comms_restored_after_new_command(self):
        """After dropout, a new receive_command() restores comms_ok."""
        ap = _make_ap()
        timeout_s = MockArdupilot.python_constants("pumping")["CMD_TIMEOUT_S"]
        for _ in range(int(timeout_s / DT) + 10):
            ap.controller_step(IC_OBS, DT)
        assert not ap.comms_ok

        ap.receive_command(_cmd(), DT_CMD)
        assert ap.comms_ok


