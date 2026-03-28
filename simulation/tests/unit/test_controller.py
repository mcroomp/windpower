"""
test_controller.py — Unit tests for simulation/controller.py.

Covers both:
  compute_rc_rates()          — ENU truth-state based controller
  compute_rc_from_attitude()  — ArduPilot ATTITUDE message based controller


All tests use a hub at a realistic RAWES tether position (50 m tether,
~30° elevation, hub East of anchor at origin).
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from controller import compute_rc_rates, compute_rc_from_attitude


# ── Helpers ──────────────────────────────────────────────────────────────────

def _Ry(angle_rad):
    """Rotation matrix about Y axis (ENU: tilts from Z toward X)."""
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    return np.array([[c, 0., s], [0., 1., 0.], [-s, 0., c]])


def _Rx(angle_rad):
    """Rotation matrix about X axis."""
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    return np.array([[1., 0., 0.], [0., c, -s], [0., s, c]])


def _Rz(angle_rad):
    """Rotation matrix about Z axis."""
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


def _state(pos, R, omega):
    return {
        "pos":   np.array(pos,   dtype=float),
        "R":     np.array(R,     dtype=float),
        "omega": np.array(omega, dtype=float),
    }


# Reference configuration: hub 50 m from origin at ~30° elevation, hub East
# tether direction: [cos(30°), 0, sin(30°)] = [0.866, 0, 0.5]
_ANCHOR      = np.zeros(3)
_TETHER_ELEV = np.radians(30.0)
_POS         = 50.0 * np.array([np.cos(_TETHER_ELEV), 0., np.sin(_TETHER_ELEV)])
_BODY_Z_EQ   = _POS / np.linalg.norm(_POS)   # tether direction = desired body_z

# Rotation matrix whose body_z = tether direction.
# body_z = [cos30, 0, sin30]; choose body_x = [0,1,0], body_y = cross(body_z, body_x)
_body_x = np.array([0., 1., 0.])
_body_z = _BODY_Z_EQ
_body_y = np.cross(_body_z, _body_x)
_R_EQ   = np.column_stack([_body_x, _body_y, _body_z])  # columns = body axes in world

# A representative NED velocity (hub drifting slowly North-East)
_VEL_NED = np.array([1.0, 0.5, 0.0])


# ── Tests ─────────────────────────────────────────────────────────────────────

def test_body_z_on_tether_no_rate():
    """Hub aligned with tether, zero omega → neutral sticks."""
    state = _state(_POS, _R_EQ, [0., 0., 0.])
    rc = compute_rc_rates(state, _ANCHOR, _VEL_NED)
    assert rc[1] == 1500
    assert rc[2] == 1500
    assert rc[4] == 1500
    assert rc[8] == 2000


def test_body_z_on_tether_with_spin_only():
    """Hub aligned with tether, only spin omega (along body_z) → neutral sticks.

    Spin is along the tether direction; stripping it should leave zero orbital
    rate and therefore no correction command.
    """
    spin_world = 25.0 * _BODY_Z_EQ   # 25 rad/s along tether = pure rotor spin
    state = _state(_POS, _R_EQ, spin_world)
    rc = compute_rc_rates(state, _ANCHOR, _VEL_NED)
    assert rc[1] == 1500
    assert rc[2] == 1500
    assert rc[4] == 1500
    assert rc[8] == 2000


def test_tilt_error_gives_non_neutral_command():
    """A small tilt away from tether direction produces non-neutral RC."""
    # Rotate hub 5° away from equilibrium around world Y
    R_tilted = _Ry(np.radians(5.0)) @ _R_EQ
    state = _state(_POS, R_tilted, [0., 0., 0.])
    rc = compute_rc_rates(state, _ANCHOR, _VEL_NED)
    # At least one channel must differ from 1500
    assert rc[1] != 1500 or rc[2] != 1500 or rc[4] != 1500


def test_tilt_direction_corrects_toward_tether():
    """Opposite tilts produce opposite RC commands (sign symmetry)."""
    R_pos = _Ry(+np.radians(10.0)) @ _R_EQ
    R_neg = _Ry(-np.radians(10.0)) @ _R_EQ
    state_pos = _state(_POS, R_pos, [0., 0., 0.])
    state_neg = _state(_POS, R_neg, [0., 0., 0.])
    rc_pos = compute_rc_rates(state_pos, _ANCHOR, _VEL_NED, kd=0.0)
    rc_neg = compute_rc_rates(state_neg, _ANCHOR, _VEL_NED, kd=0.0)
    # Commands should be mirrored around 1500 for at least one axis
    # (the tilted axis will have |cmd - 1500| equal and opposite sign)
    dev_pos = [rc_pos[ch] - 1500 for ch in (1, 2, 4)]
    dev_neg = [rc_neg[ch] - 1500 for ch in (1, 2, 4)]
    # Sum of each pair should be near zero (opposite deviations)
    for dp, dn in zip(dev_pos, dev_neg):
        assert abs(dp + dn) <= 2, f"Not mirrored: {dp} vs {dn}"


def test_larger_error_gives_larger_command():
    """Larger tilt angle → larger magnitude RC deviation."""
    R_small = _Ry(np.radians(5.0)) @ _R_EQ
    R_large = _Ry(np.radians(20.0)) @ _R_EQ
    rc_small = compute_rc_rates(_state(_POS, R_small, [0., 0., 0.]), _ANCHOR, _VEL_NED, kd=0.0)
    rc_large = compute_rc_rates(_state(_POS, R_large, [0., 0., 0.]), _ANCHOR, _VEL_NED, kd=0.0)
    # Max deviation over all channels
    dev_small = max(abs(rc_small[ch] - 1500) for ch in (1, 2, 4))
    dev_large = max(abs(rc_large[ch] - 1500) for ch in (1, 2, 4))
    assert dev_large > dev_small


def test_orbital_rate_gives_damping_command():
    """A non-zero orbital rate at equilibrium attitude produces a damping RC command."""
    # Hub is aligned but has a small orbital angular velocity
    omega_orbital = np.array([0., 0.3, 0.])  # world ENU rad/s
    state = _state(_POS, _R_EQ, omega_orbital)
    rc = compute_rc_rates(state, _ANCHOR, _VEL_NED, kp=0.0)   # P off; only D
    assert rc[1] != 1500 or rc[2] != 1500 or rc[4] != 1500


def test_opposing_orbital_rates_give_opposing_commands():
    """Opposite orbital rates produce opposite damping commands."""
    om_pos = np.array([0., +0.5, 0.])
    om_neg = np.array([0., -0.5, 0.])
    rc_pos = compute_rc_rates(_state(_POS, _R_EQ, om_pos), _ANCHOR, _VEL_NED, kp=0.0)
    rc_neg = compute_rc_rates(_state(_POS, _R_EQ, om_neg), _ANCHOR, _VEL_NED, kp=0.0)
    dev_pos = [rc_pos[ch] - 1500 for ch in (1, 2, 4)]
    dev_neg = [rc_neg[ch] - 1500 for ch in (1, 2, 4)]
    for dp, dn in zip(dev_pos, dev_neg):
        assert abs(dp + dn) <= 2, f"Damping not mirrored: {dp} vs {dn}"


def test_spin_does_not_affect_command():
    """Pure rotor spin (omega along body_z) must be stripped: neutral output."""
    # Large spin along actual body_z (column 2 of R_EQ)
    body_z_actual = _R_EQ[:, 2]
    omega_spin    = 28.0 * body_z_actual
    state = _state(_POS, _R_EQ, omega_spin)
    rc = compute_rc_rates(state, _ANCHOR, _VEL_NED)
    assert rc[1] == 1500
    assert rc[2] == 1500
    assert rc[4] == 1500


def test_rc_values_always_in_range():
    """All RC values must stay in [1000, 2000] regardless of inputs."""
    # Very large tilt error
    R_huge = _Ry(np.radians(80.0)) @ _R_EQ
    omega_huge = np.array([5., 5., 5.])
    state = _state(_POS, R_huge, omega_huge)
    rc = compute_rc_rates(state, _ANCHOR, _VEL_NED)
    for ch in (1, 2, 4, 8):
        assert 1000 <= rc[ch] <= 2000, f"Channel {ch} out of range: {rc[ch]}"


def test_large_error_saturates():
    """A large tilt error must saturate RC to 1000 or 2000."""
    R_sat = _Ry(np.radians(89.0)) @ _R_EQ
    state = _state(_POS, R_sat, [0., 0., 0.])
    rc = compute_rc_rates(state, _ANCHOR, _VEL_NED, kp=10.0, kd=0.0)
    devs = [abs(rc[ch] - 1500) for ch in (1, 2, 4)]
    assert max(devs) == 500, f"Expected saturation but got max dev {max(devs)}"


def test_hub_too_close_to_anchor_gives_neutral():
    """Hub at/near anchor (tether length < 0.1 m) → safe neutral output."""
    state = _state([0.0, 0.0, 0.05], _R_EQ, [1., 2., 3.])
    rc = compute_rc_rates(state, _ANCHOR, _VEL_NED)
    assert rc[1] == 1500
    assert rc[2] == 1500
    assert rc[4] == 1500
    assert rc[8] == 2000


def test_motor_interlock_always_2000():
    """Channel 8 (motor interlock) must always be 2000."""
    # Several different states
    for omega in ([0., 0., 0.], [5., 5., 5.], [0., 0., 25.]):
        for tilt_deg in (0, 10, 45):
            R = _Ry(np.radians(tilt_deg)) @ _R_EQ
            state = _state(_POS, R, omega)
            rc = compute_rc_rates(state, _ANCHOR, _VEL_NED)
            assert rc[8] == 2000


def test_kp_zero_no_proportional_correction():
    """With kp=0, a pure attitude error (no rate) → neutral sticks."""
    R_tilted = _Ry(np.radians(15.0)) @ _R_EQ
    state = _state(_POS, R_tilted, [0., 0., 0.])
    rc = compute_rc_rates(state, _ANCHOR, _VEL_NED, kp=0.0, kd=0.0)
    assert rc[1] == 1500
    assert rc[2] == 1500
    assert rc[4] == 1500


def test_kd_zero_no_damping():
    """With kd=0, a pure rate at equilibrium attitude → neutral sticks."""
    omega = np.array([0.3, 0.3, 0.0])
    state = _state(_POS, _R_EQ, omega)
    rc = compute_rc_rates(state, _ANCHOR, _VEL_NED, kp=0.0, kd=0.0)
    assert rc[1] == 1500
    assert rc[2] == 1500
    assert rc[4] == 1500


# ── compute_rc_from_attitude tests ────────────────────────────────────────────

def test_att_zero_attitude_zero_rates_neutral():
    """Zero attitude error and zero rates → neutral sticks."""
    rc = compute_rc_from_attitude(0.0, 0.0, 0.0, 0.0, 0.0)
    assert rc[1] == 1500
    assert rc[2] == 1500
    assert rc[4] == 1500
    assert rc[8] == 2000


def test_att_positive_roll_gives_negative_roll_command():
    """Positive roll error → negative roll rate command (corrects toward zero)."""
    rc = compute_rc_from_attitude(roll=0.1, pitch=0.0, rollspeed=0.0,
                                   pitchspeed=0.0, yawspeed=0.0, kd=0.0)
    assert rc[1] < 1500   # negative roll rate → below neutral


def test_att_positive_pitch_gives_negative_pitch_command():
    """Positive pitch error → negative pitch rate command."""
    rc = compute_rc_from_attitude(roll=0.0, pitch=0.1, rollspeed=0.0,
                                   pitchspeed=0.0, yawspeed=0.0, kd=0.0)
    assert rc[2] < 1500


def test_att_positive_rollspeed_gives_damping_command():
    """Positive rollspeed → negative roll rate command (damps the rate)."""
    rc = compute_rc_from_attitude(roll=0.0, pitch=0.0, rollspeed=0.2,
                                   pitchspeed=0.0, yawspeed=0.0, kp=0.0)
    assert rc[1] < 1500


def test_att_positive_yawspeed_gives_damping_command():
    """Positive yawspeed → negative yaw rate command."""
    rc = compute_rc_from_attitude(roll=0.0, pitch=0.0, rollspeed=0.0,
                                   pitchspeed=0.0, yawspeed=0.3, kp=0.0)
    assert rc[4] < 1500


def test_att_opposite_errors_give_opposite_commands():
    """Opposite roll errors → opposite RC deviations."""
    rc_pos = compute_rc_from_attitude(+0.2, 0.0, 0.0, 0.0, 0.0, kd=0.0)
    rc_neg = compute_rc_from_attitude(-0.2, 0.0, 0.0, 0.0, 0.0, kd=0.0)
    assert rc_pos[1] + rc_neg[1] == 3000   # symmetric around 1500


def test_att_larger_error_larger_command():
    """Larger attitude error → larger RC deviation."""
    rc_small = compute_rc_from_attitude(0.05, 0.0, 0.0, 0.0, 0.0, kd=0.0)
    rc_large = compute_rc_from_attitude(0.30, 0.0, 0.0, 0.0, 0.0, kd=0.0)
    assert abs(rc_large[1] - 1500) > abs(rc_small[1] - 1500)


def test_att_output_always_in_range():
    """PWM output always stays in [1000, 2000]."""
    for roll in (-2.0, -0.5, 0.0, 0.5, 2.0):
        for pitch in (-2.0, 0.0, 2.0):
            for speed in (-5.0, 0.0, 5.0):
                rc = compute_rc_from_attitude(roll, pitch, speed, speed, speed)
                for ch in (1, 2, 4, 8):
                    assert 1000 <= rc[ch] <= 2000


def test_att_large_error_saturates():
    """Very large error saturates to 1000 or 2000."""
    rc = compute_rc_from_attitude(100.0, 0.0, 0.0, 0.0, 0.0, kp=10.0, kd=0.0)
    assert rc[1] == 1000   # max negative roll rate


def test_att_motor_interlock_always_2000():
    """Channel 8 is always 2000 regardless of inputs."""
    for roll in (-1.0, 0.0, 1.0):
        rc = compute_rc_from_attitude(roll, 0.0, 0.0, 0.0, 0.0)
        assert rc[8] == 2000
