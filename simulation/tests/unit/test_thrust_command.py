"""
test_thrust_command.py -- Unit tests for ThrustCommand protocol.

Architecture under test:
    Ground (10 Hz): phase schedule -> ThrustCommand(thrust 0..1, alt_m, phase)
                    thrust=0: AP goes to minimum tension (tether-tilt, coll_min)
                    thrust=1: AP goes to maximum tension (alt-hold, coll_max)
                    Reel-out: ramps thrust 0->target over thrust_ramp_s
                    Transition/reel-in/hold: thrust=0

    AP (400 Hz): direct thrust -> body_z + collective mapping
                 body_z  = slerp(tether_dir, alt_hold, thrust)
                 coll    = coll_min + thrust * (coll_max - coll_min), slewed
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from pumping_planner import ThrustCommand, PumpingGroundController
from ap_controller   import ThrustApController
from simtest_ic      import load_ic
import rotor_definition as rd

_IC    = load_ic()
_ROTOR = rd.default()

DT          = 1.0 / 400.0
WARM_COLL   = _IC.coll_eq_rad
COLL_MIN    = -0.28
COLL_MAX    =  0.10
ALT_M       = float(-_IC.pos[2])


def _make_ap(timeout=3600.0, slew=0.15):
    return ThrustApController(
        ic_pos          = _IC.pos,
        mass_kg         = _ROTOR.mass_kg,
        slew_rate_rad_s = _ROTOR.body_z_slew_rate_rad_s,
        warm_coll_rad   = WARM_COLL,
        coll_min        = COLL_MIN,
        coll_max        = COLL_MAX,
        coll_slew_rate  = slew,
        cmd_timeout_s   = timeout,
    )


def _make_ground(n_cycles=1, target_thrust=0.9, ramp_s=5.0):
    return PumpingGroundController(
        t_reel_out    = 30.0,
        t_reel_in     = 30.0,
        t_transition  = 5.0,
        target_alt_m  = ALT_M,
        n_cycles      = n_cycles,
        target_thrust = target_thrust,
        thrust_ramp_s = ramp_s,
    )


# ---------------------------------------------------------------------------
# ThrustCommand dataclass
# ---------------------------------------------------------------------------

class TestThrustCommand:

    def test_carries_thrust(self):
        cmd = ThrustCommand(thrust=0.7, alt_m=15.0, phase="reel-out")
        assert cmd.thrust == pytest.approx(0.7)

    def test_carries_alt_m(self):
        cmd = ThrustCommand(thrust=0.0, alt_m=22.5, phase="hold")
        assert cmd.alt_m == pytest.approx(22.5)

    def test_carries_phase(self):
        cmd = ThrustCommand(thrust=0.0, alt_m=15.0, phase="reel-in")
        assert cmd.phase == "reel-in"

    def test_is_frozen(self):
        cmd = ThrustCommand(thrust=0.5, alt_m=15.0, phase="reel-out")
        with pytest.raises((AttributeError, TypeError)):
            cmd.thrust = 0.9  # type: ignore[misc]


# ---------------------------------------------------------------------------
# PumpingGroundController phase schedule
# ---------------------------------------------------------------------------

class TestGroundController:

    def test_reel_out_phase_at_start(self):
        g = _make_ground()
        cmd = g.step(0.0)
        assert cmd.phase == "reel-out"

    def test_transition_phase(self):
        g = _make_ground()
        # t=31 is within transition window (30..35)
        cmd = g.step(31.0)
        assert cmd.phase == "transition"

    def test_reel_in_phase(self):
        g = _make_ground()
        # t=36 is in reel-in (after 30+5 transition)
        cmd = g.step(36.0)
        assert cmd.phase == "reel-in"

    def test_hold_after_n_cycles(self):
        g = _make_ground(n_cycles=1)
        # t=65 > 1 * 60 s cycle
        cmd = g.step(65.0)
        assert cmd.phase == "hold"

    def test_thrust_zero_during_hold(self):
        g = _make_ground(n_cycles=1)
        cmd = g.step(65.0)
        assert cmd.thrust == pytest.approx(0.0)

    def test_thrust_zero_during_transition(self):
        g = _make_ground()
        cmd = g.step(31.0)
        assert cmd.thrust == pytest.approx(0.0)

    def test_thrust_zero_during_reel_in(self):
        g = _make_ground()
        cmd = g.step(40.0)
        assert cmd.thrust == pytest.approx(0.0)

    def test_thrust_ramps_from_zero_at_reel_out_start(self):
        """Thrust starts at 0 when reel-out begins."""
        g = _make_ground(ramp_s=8.0)
        cmd = g.step(0.0)
        assert cmd.thrust == pytest.approx(0.0, abs=1e-6)

    def test_thrust_increases_during_ramp(self):
        """Thrust grows monotonically during ramp."""
        g = _make_ground(ramp_s=8.0)
        thrusts = [g.step(t).thrust for t in [0.0, 2.0, 4.0, 6.0, 8.0]]
        for a, b in zip(thrusts, thrusts[1:]):
            assert b >= a

    def test_thrust_reaches_target_after_ramp(self):
        """After ramp_s, thrust equals target_thrust."""
        g = _make_ground(target_thrust=0.9, ramp_s=8.0)
        g.step(0.0)          # initialize phase_start at t=0
        cmd = g.step(10.0)   # well past 8s ramp
        assert cmd.thrust == pytest.approx(0.9, rel=1e-4)

    def test_thrust_capped_at_target(self):
        """Thrust never exceeds target_thrust during reel-out."""
        g = _make_ground(target_thrust=0.7, ramp_s=4.0)
        for t in np.linspace(0, 29, 100):
            cmd = g.step(float(t))
            if cmd.phase == "reel-out":
                assert cmd.thrust <= 0.7 + 1e-9

    def test_phase_label_from_step(self):
        """step() exposes phase via property."""
        g = _make_ground()
        g.step(5.0)
        assert g.phase == "reel-out"

    def test_no_tension_argument_needed(self):
        """step() takes only t_sim -- no tension feedback."""
        g = _make_ground()
        cmd = g.step(5.0)
        assert isinstance(cmd, ThrustCommand)


# ---------------------------------------------------------------------------
# ThrustApController collective mapping
# ---------------------------------------------------------------------------

class TestApCollective:

    # full-range slew: 0.38 rad / 0.15 rad/s = 2.53 s = ~1012 steps; use 1400 for margin
    _SETTLE = 1400

    def _run(self, ap, thrust, steps=None):
        if steps is None:
            steps = self._SETTLE
        pos = _IC.pos.copy()
        bz  = _IC.R0[:, 2].copy()
        ap.receive_command(ThrustCommand(thrust=thrust, alt_m=ALT_M, phase="reel-out"))
        for _ in range(steps):
            col, bz = ap.step(pos, bz, DT)
        return col, bz

    def test_thrust_zero_drives_to_coll_min(self):
        """thrust=0 -> collective slews to coll_min."""
        ap = _make_ap()
        col, _ = self._run(ap, thrust=0.0)
        assert col == pytest.approx(COLL_MIN, abs=1e-4)

    def test_thrust_one_drives_to_coll_max(self):
        """thrust=1 -> collective slews to coll_max."""
        ap = _make_ap()
        col, _ = self._run(ap, thrust=1.0)
        assert col == pytest.approx(COLL_MAX, abs=1e-4)

    def test_thrust_half_drives_to_midpoint(self):
        """thrust=0.5 -> collective at midpoint between coll_min and coll_max."""
        ap = _make_ap()
        col, _ = self._run(ap, thrust=0.5)
        expected = COLL_MIN + 0.5 * (COLL_MAX - COLL_MIN)
        assert col == pytest.approx(expected, abs=1e-3)

    def test_collective_clamps_at_coll_max(self):
        """thrust=1 -> collective never exceeds coll_max."""
        ap = _make_ap()
        col, _ = self._run(ap, thrust=1.0)
        assert col <= COLL_MAX + 1e-9

    def test_collective_clamps_at_coll_min(self):
        """thrust=0 -> collective never goes below coll_min."""
        ap = _make_ap()
        col, _ = self._run(ap, thrust=0.0)
        assert col >= COLL_MIN - 1e-9

    def test_higher_thrust_higher_collective(self):
        """Higher thrust -> higher collective (monotone)."""
        ap1 = _make_ap()
        ap2 = _make_ap()
        col1, _ = self._run(ap1, thrust=0.3)
        col2, _ = self._run(ap2, thrust=0.7)
        assert col2 > col1

    def test_collective_slew_limits_rate(self):
        """Collective changes no faster than coll_slew_rate per step."""
        ap = _make_ap(slew=0.10)
        pos = _IC.pos.copy()
        bz  = _IC.R0[:, 2].copy()
        ap.receive_command(ThrustCommand(thrust=1.0, alt_m=ALT_M, phase="reel-out"))
        prev_col = WARM_COLL
        for _ in range(200):
            col, bz = ap.step(pos, bz, DT)
            assert abs(col - prev_col) <= 0.10 * DT + 1e-9
            prev_col = col


# ---------------------------------------------------------------------------
# ThrustApController body_z behavior
# ---------------------------------------------------------------------------

class TestApBodyZ:

    # body_z slerp: up to 90 deg at 0.4 rad/s = ~3.9s = ~1570 steps; use 2500 for margin
    _BZ_SETTLE = 2500

    def test_thrust_zero_tilts_toward_tether(self):
        """thrust=0 -> body_z slews toward tether direction (-pos/|pos|)."""
        ap = _make_ap()
        pos = _IC.pos.copy()
        bz  = _IC.R0[:, 2].copy()
        ap.receive_command(ThrustCommand(thrust=0.0, alt_m=ALT_M, phase="reel-in"))
        for _ in range(self._BZ_SETTLE):
            col, bz = ap.step(pos, bz, DT)
        tether_dir = -np.asarray(_IC.pos) / np.linalg.norm(_IC.pos)
        dot = float(np.dot(bz, tether_dir))
        assert dot > 0.8   # well-aligned with tether direction

    def test_thrust_one_holds_altitude_direction(self):
        """thrust=1 -> body_z slews toward altitude-hold direction (not tether)."""
        ap = _make_ap()
        pos = _IC.pos.copy()
        bz  = _IC.R0[:, 2].copy()
        tether_dir = -np.asarray(_IC.pos) / np.linalg.norm(_IC.pos)

        ap.receive_command(ThrustCommand(thrust=1.0, alt_m=ALT_M, phase="reel-out"))
        for _ in range(self._BZ_SETTLE):
            col, bz = ap.step(pos, bz, DT)

        dot_tether = float(np.dot(bz, tether_dir))
        assert dot_tether < 0.99   # not pointing straight toward tether

    def test_body_z_is_unit_vector(self):
        """body_z output is always a unit vector."""
        ap = _make_ap()
        pos = _IC.pos.copy()
        bz  = _IC.R0[:, 2].copy()
        ap.receive_command(ThrustCommand(thrust=0.5, alt_m=ALT_M, phase="reel-out"))
        for _ in range(400):
            col, bz = ap.step(pos, bz, DT)
        assert float(np.linalg.norm(bz)) == pytest.approx(1.0, abs=1e-6)


# ---------------------------------------------------------------------------
# Diagnostics properties
# ---------------------------------------------------------------------------

class TestDiagnostics:

    def test_thrust_property_returns_received_thrust(self):
        ap = _make_ap()
        ap.receive_command(ThrustCommand(thrust=0.7, alt_m=ALT_M, phase="reel-out"))
        assert ap.thrust == pytest.approx(0.7)

    def test_tilt_frac_is_one_minus_thrust(self):
        ap = _make_ap()
        ap.receive_command(ThrustCommand(thrust=0.6, alt_m=ALT_M, phase="reel-out"))
        assert ap.tilt_frac == pytest.approx(0.4, abs=1e-6)

    def test_raw_coll_equals_current_collective(self):
        ap = _make_ap()
        pos = _IC.pos.copy()
        bz  = _IC.R0[:, 2].copy()
        ap.receive_command(ThrustCommand(thrust=1.0, alt_m=ALT_M, phase="reel-out"))
        for _ in range(200):
            col, bz = ap.step(pos, bz, DT)
        assert ap.raw_coll == pytest.approx(col, abs=1e-9)

    def test_drive_in_range(self):
        """drive property is in [0, 1]."""
        ap = _make_ap()
        pos = _IC.pos.copy()
        bz  = _IC.R0[:, 2].copy()
        ap.receive_command(ThrustCommand(thrust=0.5, alt_m=ALT_M, phase="reel-out"))
        for _ in range(400):
            col, bz = ap.step(pos, bz, DT)
        assert 0.0 <= ap.drive <= 1.0


# ---------------------------------------------------------------------------
# Comms dropout
# ---------------------------------------------------------------------------

class TestCommsDropout:

    def test_dropout_sets_thrust_to_zero(self):
        """After dropout, thrust property reports 0."""
        ap = _make_ap(timeout=0.05)
        pos = _IC.pos.copy()
        bz  = _IC.R0[:, 2].copy()
        ap.receive_command(ThrustCommand(thrust=0.9, alt_m=ALT_M, phase="reel-out"))
        for _ in range(200):   # 0.5 s >> 0.05 s timeout
            col, bz = ap.step(pos, bz, DT)
        assert ap.thrust == pytest.approx(0.0)

    def test_dropout_drives_to_coll_min(self):
        """After dropout (thrust->0), collective slews toward coll_min."""
        ap = _make_ap(timeout=1.0)   # long enough to build, short enough to expire
        pos = _IC.pos.copy()
        bz  = _IC.R0[:, 2].copy()
        # Build collective to coll_max, refreshing comms every 10 Hz
        refresh_every = max(1, int(0.1 / DT))
        for i in range(1500):
            if i % refresh_every == 0:
                ap.receive_command(ThrustCommand(thrust=1.0, alt_m=ALT_M, phase="reel-out"))
            col, bz = ap.step(pos, bz, DT)
        assert col == pytest.approx(COLL_MAX, abs=0.01)
        col_before = col

        # Stop refreshing — timeout fires at 1s, then run 3 more s
        for _ in range(1600):
            col, bz = ap.step(pos, bz, DT)

        assert col < col_before   # has moved toward coll_min

    def test_comms_restored_resumes(self):
        """After comms restored with new thrust, collective moves toward new target."""
        ap = _make_ap(timeout=3.0)   # long timeout to survive 100-step window
        pos = _IC.pos.copy()
        bz  = _IC.R0[:, 2].copy()
        # Drive to coll_min
        ap.receive_command(ThrustCommand(thrust=0.0, alt_m=ALT_M, phase="hold"))
        for _ in range(1500):
            col, bz = ap.step(pos, bz, DT)
        col_at_min = col
        assert col_at_min == pytest.approx(COLL_MIN, abs=0.01)

        # Restore with high thrust; collective should start rising
        ap.receive_command(ThrustCommand(thrust=1.0, alt_m=ALT_M, phase="reel-out"))
        for _ in range(100):
            col, bz = ap.step(pos, bz, DT)
        assert col > col_at_min
