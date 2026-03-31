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

from controller import (
    compute_rc_rates,
    compute_rc_from_attitude,
    compute_rc_from_physical_attitude,
    TetherRelativeHoldController,
    PhysicalHoldController,
    make_hold_controller,
    compute_bz_tether,
    slerp_body_z,
    compute_rate_cmd,
    RatePID,
)


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


# ---------------------------------------------------------------------------
# compute_rc_from_physical_attitude tests
# ---------------------------------------------------------------------------

def _att_aligned():
    """Attitude dict for hub exactly aligned with tether (equilibrium)."""
    import math
    elev = math.radians(30.0)
    L    = 50.0
    pos  = np.array([L * math.cos(elev), 0.0, L * math.sin(elev)])  # ENU
    # body_z along tether, build orbital frame
    tether_dir = pos / np.linalg.norm(pos)
    east = np.array([1., 0., 0.])
    bx   = east - np.dot(east, tether_dir) * tether_dir
    bx  /= np.linalg.norm(bx)
    R_hub = np.column_stack([bx, np.cross(tether_dir, bx), tether_dir])
    # NED attitude from R_hub
    T = np.array([[0,1,0],[1,0,0],[0,0,-1]], dtype=float)
    R_ned = T @ R_hub @ T
    sy = R_ned[1, 0]; cy = R_ned[0, 0]
    sp = -R_ned[2, 0]
    cp = math.sqrt(R_ned[2,1]**2 + R_ned[2,2]**2)
    sr = R_ned[2, 1] / cp; cr = R_ned[2, 2] / cp
    roll  = math.atan2(sr, cr)
    pitch = math.atan2(sp, cp)
    yaw   = math.atan2(sy, cy)
    # pos_ned for tether equilibrium hub
    pos_ned = np.array([pos[1], pos[0], -pos[2]])
    anchor_ned = np.zeros(3)
    return roll, pitch, yaw, pos_ned, anchor_ned


def test_physical_att_aligned_gives_neutral():
    """Hub exactly on tether → correction is near zero → neutral RC."""
    roll, pitch, yaw, pos_ned, anchor_ned = _att_aligned()
    rc = compute_rc_from_physical_attitude(
        roll, pitch, yaw, 0.0, 0.0, 0.0,
        pos_ned=pos_ned, anchor_ned=anchor_ned,
        kp=1.0, kd=0.3,
    )
    # At perfect alignment cross(body_z, body_z_eq) = 0 → all neutral
    assert rc[1] == 1500
    assert rc[2] == 1500
    assert rc[8] == 2000


def test_physical_att_output_in_range():
    """RC values from physical attitude controller always in [1000, 2000]."""
    roll, pitch, yaw, pos_ned, anchor_ned = _att_aligned()
    for delta in (-0.5, 0.0, 0.5):
        rc = compute_rc_from_physical_attitude(
            roll + delta, pitch + delta, yaw, 0.3, 0.3, 0.3,
            pos_ned=pos_ned, anchor_ned=anchor_ned,
        )
        for ch in (1, 2, 4):
            assert 1000 <= rc[ch] <= 2000, f"ch{ch}={rc[ch]} out of range"


# ---------------------------------------------------------------------------
# TetherRelativeHoldController tests
# ---------------------------------------------------------------------------

class _FakeGCS:
    """Minimal GCS stub: records send_rc_override calls."""
    def __init__(self):
        self.sent = []
    def send_rc_override(self, rc):
        self.sent.append(dict(rc))


def test_tether_relative_neutral_at_zero_attitude():
    ctrl = TetherRelativeHoldController()
    gcs  = _FakeGCS()
    att  = {"roll": 0.0, "pitch": 0.0, "rollspeed": 0.0, "pitchspeed": 0.0, "yawspeed": 0.0}
    rc   = ctrl.send_correction(att, None, gcs)
    assert rc[1] == 1500
    assert rc[2] == 1500
    assert rc[3] == 1500    # neutral collective
    assert rc[8] == 2000    # motor interlock


def test_tether_relative_sends_neutral_on_large_roll():
    ctrl = TetherRelativeHoldController()
    gcs  = _FakeGCS()
    import math
    att  = {"roll": math.radians(70.0), "pitch": 0.0,
            "rollspeed": 0.0, "pitchspeed": 0.0, "yawspeed": 0.0}
    rc   = ctrl.send_correction(att, None, gcs)
    # Roll > 60° → guard fires → neutral roll/pitch
    assert rc[1] == 1500
    assert rc[2] == 1500
    assert rc[8] == 2000


def test_tether_relative_extra_params_include_compass_disable():
    ctrl = TetherRelativeHoldController()
    assert ctrl.extra_params.get("COMPASS_USE") == 0
    assert ctrl.extra_params.get("COMPASS_ENABLE") == 0
    assert ctrl.extra_params.get("ATC_RAT_RLL_IMAX") == 0.0


def test_physical_hold_extra_params_compass_enabled():
    """PhysicalHoldController keeps compass enabled (COMPASS_USE at default).
    Startup damping holds the hub stationary during EKF init so compass heading
    is stable; GPS+compass fusion works without disabling either source.
    EK3_SRC1_YAW is NOT set by the controller — conftest overrides it to 1
    (compass) via _gps_params which apply after controller extra_params."""
    ctrl = PhysicalHoldController(anchor_ned=np.zeros(3))
    assert "COMPASS_USE" not in ctrl.extra_params
    assert "COMPASS_ENABLE" not in ctrl.extra_params
    assert "EK3_SRC1_YAW" not in ctrl.extra_params
    assert ctrl.extra_params.get("ATC_RAT_RLL_IMAX") == 0.0


# ---------------------------------------------------------------------------
# PhysicalHoldController tests
# ---------------------------------------------------------------------------

def test_physical_hold_neutral_when_no_pos():
    ctrl = PhysicalHoldController(anchor_ned=np.zeros(3))
    gcs  = _FakeGCS()
    att  = {"roll": 0.1, "pitch": 0.2, "yaw": 0.0,
            "rollspeed": 0.0, "pitchspeed": 0.0, "yawspeed": 0.0}
    rc   = ctrl.send_correction(att, pos_ned=None, gcs=gcs)
    assert rc[1] == 1500
    assert rc[2] == 1500
    assert rc[8] == 2000


def test_physical_hold_neutral_when_no_equilibrium():
    """send_correction returns neutral sticks until set_equilibrium is called."""
    import math
    ctrl = PhysicalHoldController(anchor_ned=np.zeros(3))
    gcs  = _FakeGCS()
    pos_ned = (0.0, 0.0, -10.0)
    att = {"roll": 0.5, "pitch": 0.3, "yaw": 1.0,
           "rollspeed": 0.0, "pitchspeed": 0.0, "yawspeed": 0.0}
    rc  = ctrl.send_correction(att, pos_ned=pos_ned, gcs=gcs)
    assert rc[1] == 1500
    assert rc[2] == 1500
    assert rc[8] == 2000


def test_physical_hold_sends_correction_near_equilibrium():
    """Near-equilibrium hub: controller sends a non-trivial correction.

    set_equilibrium must be called first (normally done in conftest step 4
    from the first ATTITUDE message during the 45 s kinematic startup).
    """
    import math
    roll, pitch, yaw, pos_ned, anchor_ned = _att_aligned()
    # Tilt slightly to create a small error
    ctrl = PhysicalHoldController(anchor_ned=anchor_ned)
    ctrl.set_equilibrium(roll, pitch)   # capture equilibrium before use
    gcs  = _FakeGCS()
    small_tilt = math.radians(5.0)
    att  = {"roll": roll + small_tilt, "pitch": pitch + small_tilt, "yaw": yaw,
            "rollspeed": 0.0, "pitchspeed": 0.0, "yawspeed": 0.0}
    rc   = ctrl.send_correction(att, pos_ned=tuple(pos_ned), gcs=gcs)
    # RC must be in valid range; motor interlock must be 2000
    for ch in (1, 2, 3, 4):
        assert 1000 <= rc[ch] <= 2000
    assert rc[8] == 2000
    assert len(gcs.sent) == 1


# ---------------------------------------------------------------------------
# compute_bz_tether tests
# ---------------------------------------------------------------------------

def test_bz_tether_unit_vector():
    """Result is always a unit vector."""
    bz = compute_bz_tether([3., 4., 0.], [0., 0., 0.])
    assert bz is not None
    assert abs(np.linalg.norm(bz) - 1.0) < 1e-12


def test_bz_tether_direction():
    """Hub along +X from anchor → returns [1,0,0]."""
    bz = compute_bz_tether([10., 0., 0.], [0., 0., 0.])
    np.testing.assert_allclose(bz, [1., 0., 0.], atol=1e-12)


def test_bz_tether_with_nonzero_anchor():
    """Hub 5 m above anchor at [10,10,0] → result points straight up."""
    bz = compute_bz_tether([10., 10., 5.], [10., 10., 0.])
    np.testing.assert_allclose(bz, [0., 0., 1.], atol=1e-12)


def test_bz_tether_degenerate_returns_none():
    """Hub at anchor (within 0.1 m) → None."""
    assert compute_bz_tether([0., 0., 0.], [0., 0., 0.]) is None
    assert compute_bz_tether([0.05, 0., 0.], [0., 0., 0.]) is None


def test_bz_tether_ned_frame():
    """Works in NED equally: hub 30 m north of anchor → [1,0,0] in NED."""
    bz = compute_bz_tether([30., 0., 0.], [0., 0., 0.])
    np.testing.assert_allclose(bz, [1., 0., 0.], atol=1e-12)


# ---------------------------------------------------------------------------
# slerp_body_z tests
# ---------------------------------------------------------------------------

def test_slerp_identical_vectors():
    """Identical vectors → returns the target unchanged."""
    bz = np.array([0., 0., 1.])
    out = slerp_body_z(bz, bz, slew_rate_rad_s=1.0, dt=0.01)
    np.testing.assert_allclose(out, bz, atol=1e-12)


def test_slerp_result_is_unit_vector():
    """Output is always a unit vector regardless of angle."""
    bz_from = np.array([1., 0., 0.])
    bz_to   = np.array([0., 1., 0.])
    for dt in (0.001, 0.01, 0.1, 1.0, 100.0):
        out = slerp_body_z(bz_from, bz_to, slew_rate_rad_s=0.5, dt=dt)
        assert abs(np.linalg.norm(out) - 1.0) < 1e-12


def test_slerp_advances_at_most_one_step():
    """With a 10° target and 1 deg/s slew rate, 1 s step → exactly 1° advance."""
    bz_from = np.array([0., 0., 1.])
    # 10° off in the XZ plane
    angle_target = np.radians(10.0)
    bz_to = np.array([np.sin(angle_target), 0., np.cos(angle_target)])
    slew  = np.radians(1.0)   # 1 deg/s
    out   = slerp_body_z(bz_from, bz_to, slew_rate_rad_s=slew, dt=1.0)
    advanced = float(np.arccos(np.clip(np.dot(bz_from, out), -1, 1)))
    np.testing.assert_allclose(advanced, slew * 1.0, atol=1e-10)


def test_slerp_large_dt_reaches_target():
    """Large dt → result equals bz_target."""
    bz_from = np.array([1., 0., 0.])
    bz_to   = np.array([0., 0., 1.])
    out = slerp_body_z(bz_from, bz_to, slew_rate_rad_s=10.0, dt=1000.0)
    np.testing.assert_allclose(out, bz_to, atol=1e-12)


def test_slerp_monotonically_approaches_target():
    """Successive steps reduce the angle to bz_target."""
    bz      = np.array([1., 0., 0.])
    bz_to   = np.array([0., 0., 1.])
    slew    = np.radians(5.0)
    prev_angle = float(np.arccos(np.dot(bz, bz_to)))
    for _ in range(20):
        bz    = slerp_body_z(bz, bz_to, slew, dt=0.02)
        angle = float(np.arccos(np.clip(np.dot(bz, bz_to), -1, 1)))
        assert angle <= prev_angle + 1e-10
        prev_angle = angle


# ---------------------------------------------------------------------------
# compute_rate_cmd tests
# ---------------------------------------------------------------------------

# Identity rotation: body frame = world frame
_I3 = np.eye(3)


def test_rate_cmd_aligned_gives_zero():
    """bz_now == bz_eq → zero rate command."""
    bz = np.array([0., 0., 1.])
    rates = compute_rate_cmd(bz, bz, _I3, kp=1.0)
    np.testing.assert_allclose(rates, [0., 0., 0.], atol=1e-12)


def test_rate_cmd_opposite_tilts_give_opposite_rates():
    """Tilting +5° vs −5° off bz_eq produces mirrored rate commands."""
    bz_eq  = np.array([0., 0., 1.])
    bz_pos = np.array([np.sin(np.radians( 5.)), 0., np.cos(np.radians( 5.))])
    bz_neg = np.array([np.sin(np.radians(-5.)), 0., np.cos(np.radians(-5.))])
    r_pos  = compute_rate_cmd(bz_pos, bz_eq, _I3, kp=1.0, kd=0.0)
    r_neg  = compute_rate_cmd(bz_neg, bz_eq, _I3, kp=1.0, kd=0.0)
    np.testing.assert_allclose(r_pos + r_neg, [0., 0., 0.], atol=1e-12)


def test_rate_cmd_larger_error_larger_rate():
    """A larger tilt angle produces a proportionally larger rate command."""
    bz_eq    = np.array([0., 0., 1.])
    bz_small = np.array([np.sin(np.radians( 5.)), 0., np.cos(np.radians( 5.))])
    bz_large = np.array([np.sin(np.radians(20.)), 0., np.cos(np.radians(20.))])
    r_small  = np.linalg.norm(compute_rate_cmd(bz_small, bz_eq, _I3, kp=1.0))
    r_large  = np.linalg.norm(compute_rate_cmd(bz_large, bz_eq, _I3, kp=1.0))
    assert r_large > r_small


def test_rate_cmd_spin_not_damped():
    """Pure spin along bz_now is stripped; kd has no effect on it."""
    bz = np.array([0., 0., 1.])
    omega_spin = np.array([0., 0., 25.])   # 25 rad/s along bz_now
    r_no_spin  = compute_rate_cmd(bz, bz, _I3, kp=1.0, kd=1.0, omega_world=omega_spin)
    np.testing.assert_allclose(r_no_spin, [0., 0., 0.], atol=1e-12)


def test_rate_cmd_orbital_rate_damped():
    """Orbital omega (perpendicular to bz_now) contributes damping when kd > 0."""
    bz          = np.array([0., 0., 1.])
    omega_orb   = np.array([0.3, 0., 0.])  # orbital, not spin
    rates_no_kd = compute_rate_cmd(bz, bz, _I3, kp=0.0, kd=0.0, omega_world=omega_orb)
    rates_kd    = compute_rate_cmd(bz, bz, _I3, kp=0.0, kd=1.0, omega_world=omega_orb)
    np.testing.assert_allclose(rates_no_kd, [0., 0., 0.], atol=1e-12)
    assert np.linalg.norm(rates_kd) > 0


def test_rate_cmd_transforms_into_body_frame():
    """Result is in body frame: rotating R by 90° about Z flips roll/pitch."""
    bz_now = np.array([0., 0., 1.])
    bz_eq  = np.array([np.sin(np.radians(5.)), 0., np.cos(np.radians(5.))])
    R_world = _I3
    R_rot90 = _Rz(np.radians(90.0))   # body rotated 90° about Z in world
    r_world = compute_rate_cmd(bz_now, bz_eq, R_world, kp=1.0)
    r_rot90 = compute_rate_cmd(bz_now, bz_eq, R_rot90, kp=1.0)
    # Norms must be equal (same physical correction magnitude)
    np.testing.assert_allclose(np.linalg.norm(r_world), np.linalg.norm(r_rot90), atol=1e-10)
    # But directions differ (frame has rotated)
    assert not np.allclose(r_world, r_rot90, atol=1e-6)


# ---------------------------------------------------------------------------
# RatePID tests
# ---------------------------------------------------------------------------

def test_rate_pid_zero_error_zero_output():
    """setpoint == actual → output is 0."""
    pid = RatePID(kp=1.0)
    assert pid.update(0.0, 0.0, 0.01) == 0.0
    assert pid.update(0.5, 0.5, 0.01) == 0.0


def test_rate_pid_positive_error_positive_output():
    """setpoint > actual → positive correction."""
    pid = RatePID(kp=1.0)
    assert pid.update(1.0, 0.0, 0.01) > 0.0


def test_rate_pid_negative_error_negative_output():
    """setpoint < actual → negative correction."""
    pid = RatePID(kp=1.0)
    assert pid.update(-1.0, 0.0, 0.01) < 0.0


def test_rate_pid_proportional_to_error():
    """Output scales linearly with error magnitude (P-only)."""
    pid = RatePID(kp=0.5)
    out_small = pid.update(0.1, 0.0, 0.01)
    pid.reset()
    out_large = pid.update(0.4, 0.0, 0.01)
    np.testing.assert_allclose(out_large / out_small, 4.0, rtol=1e-6)


def test_rate_pid_output_clipped_to_output_max():
    """Very large error saturates at output_max."""
    pid = RatePID(kp=10.0, output_max=1.0)
    assert pid.update(100.0, 0.0, 0.01) == pytest.approx(1.0)
    assert pid.update(-100.0, 0.0, 0.01) == pytest.approx(-1.0)


def test_rate_pid_output_max_respected():
    """Custom output_max is respected."""
    pid = RatePID(kp=10.0, output_max=0.5)
    assert abs(pid.update(100.0, 0.0, 0.01)) <= 0.5


def test_rate_pid_integrator_accumulates():
    """With ki > 0 and persistent error, output grows across steps."""
    pid = RatePID(kp=0.0, ki=1.0, imax=10.0)
    out1 = pid.update(1.0, 0.0, 0.1)
    out2 = pid.update(1.0, 0.0, 0.1)
    assert out2 > out1


def test_rate_pid_integrator_anti_windup():
    """Integrator is clamped so output does not exceed imax."""
    pid = RatePID(kp=0.0, ki=1.0, imax=0.5)
    for _ in range(1000):
        pid.update(1.0, 0.0, 0.1)
    out = pid.update(1.0, 0.0, 0.1)
    assert abs(out) <= 0.5 + 1e-9


def test_rate_pid_reset_clears_state():
    """reset() zeros integrator and derivative history."""
    pid = RatePID(kp=0.5, ki=1.0, imax=1.0)
    for _ in range(50):
        pid.update(1.0, 0.0, 0.01)
    pid.reset()
    out = pid.update(0.0, 0.0, 0.01)
    assert out == 0.0


def test_rate_pid_default_kp_matches_legacy():
    """Default kp matches the calibrated legacy value (kd_old / tilt_max_rad)."""
    np.testing.assert_allclose(RatePID.DEFAULT_KP,
                               0.2 / 0.3,    # kd_old=0.2, tilt_max_rad=0.3
                               rtol=1e-3)


# ---------------------------------------------------------------------------
# make_hold_controller factory tests
# ---------------------------------------------------------------------------

def test_make_hold_controller_tether_relative():
    ctrl = make_hold_controller("tether_relative")
    assert isinstance(ctrl, TetherRelativeHoldController)


def test_make_hold_controller_physical():
    ctrl = make_hold_controller("physical", anchor_ned=np.zeros(3))
    assert isinstance(ctrl, PhysicalHoldController)


def test_make_hold_controller_invalid_raises():
    with pytest.raises(ValueError, match="Unknown sensor mode"):
        make_hold_controller("bogus")
