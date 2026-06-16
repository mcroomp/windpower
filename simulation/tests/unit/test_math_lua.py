"""
test_lua_math.py  --  Unit tests for rawes.lua math via the actual Lua runtime.

Runs rawes.lua under lupa (Lua 5.4) using mock ArduPilot bindings.
Tests call into the real Lua functions via the _rawes_fns test surface,
so any change to rawes.lua is immediately reflected here -- no Python
transcription to drift out of sync.

Cross-checks compare Lua output against controller.py equivalents to catch
divergence when either side is updated.

No SITL, no Docker.  Runs with the existing unit-test venv.
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from controller import (
    compute_bz_altitude_hold,
    damp_bz_eq_lateral,
)
from rawes_lua_harness import RawesLua

# ── Module-level harness ──────────────────────────────────────────────────────

@pytest.fixture(scope="module")
def sim():
    s = RawesLua()
    s.R = np.eye(3)
    return s


# ── Helpers ───────────────────────────────────────────────────────────────────

def _unit(v):
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


def _rot_z(angle):
    """3x3 rotation matrix around NED Z (yaw)."""
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


def _bz_ned_to_roll_pitch(bz_ned, yaw_rad):
    """Python reference for bz_ned_to_roll_pitch (mirrors the Lua implementation)."""
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    bz_fwd   =  cy * bz_ned[0] + sy * bz_ned[1]
    bz_right = -sy * bz_ned[0] + cy * bz_ned[1]
    bz_down  = bz_ned[2]
    pitch_deg = math.degrees(math.atan2(bz_fwd, bz_down))
    roll_deg  = math.degrees(math.asin(max(-1.0, min(1.0, -bz_right))))
    return roll_deg, pitch_deg


# ── Constants sanity ─────────────────────────────────────────────────────────

def test_constants_have_expected_values(sim):
    """Key rawes.lua constants match their documented values."""
    f = sim.fns
    assert float(f.COL_CRUISE_FLIGHT_RAD) == pytest.approx(-0.18)
    assert float(f.COL_MIN_RAD)           == pytest.approx(-0.28)
    assert float(f.COL_MAX_RAD)           == pytest.approx(0.10)
    assert float(f.COL_REEL_OUT)          == pytest.approx(-0.20)
    assert float(f.T_PUMP_TRANSITION)     == pytest.approx(3.7)
    assert float(f.KP_VZ)                 == pytest.approx(0.05)
    assert float(f.MASS_KG)               == pytest.approx(5.0)
    assert float(f.G_ACCEL)               == pytest.approx(9.81)
    assert float(f.KP_TEN)               == pytest.approx(2e-4)
    assert float(f.KI_TEN)               == pytest.approx(1e-3)
    assert float(f.COL_MAX_TEN)          == pytest.approx(0.10)


# ── bz_altitude_hold ─────────────────────────────────────────────────────────

class TestBzAltitudeHold:
    """
    Tests for bz_altitude_hold(pos, el_rad, tension_n).

    Lua equivalent of Python compute_bz_altitude_hold:
      - points the disk at (el_rad, azimuth-from-pos) in NED
      - adds a gravity-compensation tilt: raw = tdir + (mass*g*cos(el)/tension) * e_up
      - normalises to a unit vector
    """

    def test_output_is_unit_vector(self, sim):
        """Result is always a unit vector."""
        pos = sim.lua_vec(10.0, 5.0, -8.0)
        r   = sim.fns.bz_altitude_hold(pos, 0.3, 200.0)
        rv  = sim.vec_to_list(r)
        assert math.sqrt(sum(x**2 for x in rv)) == pytest.approx(1.0, abs=1e-12)

    def test_vertical_tether_points_down(self, sim):
        """FRD: at el=pi/2 (vertical) the hub is straight up, so body_z (= hub→
        anchor = down through disk) points straight down: NED z = +1."""
        pos = sim.lua_vec(0.0, 0.0, -50.0)   # directly above anchor
        r   = sim.fns.bz_altitude_hold(pos, math.pi / 2, 300.0)
        rv  = sim.vec_to_list(r)
        assert rv[0] == pytest.approx(0.0, abs=1e-6)
        assert rv[1] == pytest.approx(0.0, abs=1e-6)
        assert rv[2] == pytest.approx(1.0, abs=1e-6)

    def test_azimuth_from_position(self, sim):
        """Changing azimuth of pos rotates the output body_z horizontally."""
        el = 0.3
        r  = 30.0
        alt = r * math.sin(el)
        # North position → body_z should have X-dominant horizontal component
        pos_n = sim.lua_vec(r * math.cos(el), 0.0, -alt)
        rv_n  = sim.vec_to_list(sim.fns.bz_altitude_hold(pos_n, el, 200.0))
        # East position → body_z should have Y-dominant horizontal component
        pos_e = sim.lua_vec(0.0, r * math.cos(el), -alt)
        rv_e  = sim.vec_to_list(sim.fns.bz_altitude_hold(pos_e, el, 200.0))
        # North case: X component dominant, Y small
        assert abs(rv_n[0]) > abs(rv_n[1])
        # East case: Y component dominant, X small
        assert abs(rv_e[1]) > abs(rv_e[0])

    def test_gravity_comp_increases_with_lower_elevation(self, sim):
        """
        The angular deviation of body_z from the tether direction is larger
        at low elevation: k = mass*g*cos(el)/tension is larger when el is small.
        """
        r = 30.0
        tension = 200.0

        def deviation_angle(el_deg):
            el  = math.radians(el_deg)
            pos = sim.lua_vec(r * math.cos(el), 0.0, -r * math.sin(el))
            bz  = sim.vec_to_list(sim.fns.bz_altitude_hold(pos, el, tension))
            # FRD body_z = hub→anchor direction (= -anchor→hub)
            tdir = [-math.cos(el), 0.0, math.sin(el)]
            dot  = sum(bz[i] * tdir[i] for i in range(3))
            return math.acos(max(-1.0, min(1.0, dot)))

        assert deviation_angle(10.0) > deviation_angle(70.0)

    @pytest.mark.parametrize("el_deg,az_deg,tension", [
        (10.0, 0.0,   150.0),
        (25.0, 45.0,  200.0),
        (40.0, 90.0,  300.0),
        (70.0, 180.0, 250.0),
        (80.0, 270.0, 400.0),
        (15.0, 30.0,  180.0),
    ])
    def test_matches_python_compute_bz_altitude_hold(self, sim, el_deg, az_deg, tension):
        """
        Cross-check: Lua bz_altitude_hold matches controller.py
        compute_bz_altitude_hold across a sweep of elevations and azimuths.
        """
        MASS_KG = float(sim.fns.MASS_KG)
        el_rad  = math.radians(el_deg)
        az_rad  = math.radians(az_deg)
        r       = 40.0
        pos_np  = np.array([
            r * math.cos(el_rad) * math.cos(az_rad),
            r * math.cos(el_rad) * math.sin(az_rad),
            -r * math.sin(el_rad),
        ])
        expected = compute_bz_altitude_hold(pos_np, el_rad, tension, MASS_KG).tolist()

        pos_lua = sim.lua_vec(*pos_np)
        result  = sim.vec_to_list(sim.fns.bz_altitude_hold(pos_lua, el_rad, tension))

        assert result == pytest.approx(expected, abs=1e-10), \
            f"Mismatch at el={el_deg} az={az_deg} T={tension}"


# ── damp_bz_eq_lateral ────────────────────────────────────────────────────────


class TestDampBzEqLateral:
    """Cross-check: Lua damp_bz_eq_lateral matches Python damp_bz_eq_lateral."""

    @pytest.mark.parametrize("vel_xyz, tension", [
        ((0.0, 0.0,  0.0), 300.0),   # no velocity → no correction
        ((1.0, 0.0,  0.0), 300.0),
        ((0.0, 2.0, -0.5), 200.0),
        ((1.5, 0.5,  0.0), 100.0),
        ((0.0, 0.0,  3.0), 400.0),   # purely along-tether (full cancel)
    ])
    def test_matches_python(self, sim, vel_xyz, tension):
        """Lua matches controller.damp_bz_eq_lateral across velocity / tension grid."""
        kd_lat  = float(sim.fns.KD_LAT)
        # Hub position at 30 deg elevation, east of anchor.
        pos_np  = np.array([0.0, 86.6025, -50.0])
        # Use the gravity-comped altitude-hold body_z as the base target.
        MASS_KG = float(sim.fns.MASS_KG)
        bz_np   = compute_bz_altitude_hold(pos_np, math.radians(30.0), tension, MASS_KG)
        vel_np  = np.array(vel_xyz, dtype=float)

        expected = damp_bz_eq_lateral(
            bz_np, pos_np, vel_np, np.zeros(3), tension, kd_lat,
        ).tolist()

        bz_lua  = sim.lua_vec(*bz_np)
        pos_lua = sim.lua_vec(*pos_np)
        vel_lua = sim.lua_vec(*vel_np)
        result  = sim.vec_to_list(sim.fns.damp_bz_eq_lateral(
            bz_lua, pos_lua, vel_lua, tension, kd_lat,
        ))

        assert result == pytest.approx(expected, abs=1e-10), \
            f"Mismatch at vel={vel_xyz} T={tension}"


# ── bz_ned_to_roll_pitch ──────────────────────────────────────────────────────

class TestBzNedToRollPitch:
    """
    Tests for bz_ned_to_roll_pitch(bz_ned, yaw_rad).

    This is the GUIDED-mode conversion: given a desired body_z vector in NED
    and the current heading, produce the ZYX Euler roll/pitch (degrees) to
    pass to vehicle:set_target_angle_and_climbrate.
    """

    def _call(self, sim, bz_ned, yaw_rad):
        """Call Lua bz_ned_to_roll_pitch and return (roll_deg, pitch_deg)."""
        bz = sim.lua_vec(*bz_ned)
        r, p = sim.fns.bz_ned_to_roll_pitch(bz, yaw_rad)
        return float(r), float(p)

    def test_straight_down_is_zero_angles(self, sim):
        """body_z straight down (level hover) -> roll=0, pitch=0 regardless of yaw."""
        for yaw in [0.0, 0.5, math.pi, -1.2]:
            roll, pitch = self._call(sim, [0.0, 0.0, 1.0], yaw)
            assert roll  == pytest.approx(0.0, abs=1e-10), f"roll nonzero at yaw={yaw}"
            assert pitch == pytest.approx(0.0, abs=1e-10), f"pitch nonzero at yaw={yaw}"

    def test_north_tilt_at_zero_yaw_is_pure_pitch(self, sim):
        """
        bz tilted North at yaw=0: should produce nonzero pitch and zero roll.
        (North tilt = nose-down = positive pitch in ZYX convention.)
        """
        bz = _unit([0.2, 0.0, 1.0])
        roll, pitch = self._call(sim, bz, 0.0)
        assert abs(roll) < 1e-10
        assert pitch > 1e-6

    def test_east_tilt_at_zero_yaw_is_pure_roll(self, sim):
        """bz tilted East at yaw=0: should produce nonzero roll and zero pitch."""
        bz = _unit([0.0, 0.2, 1.0])
        roll, pitch = self._call(sim, bz, 0.0)
        assert abs(pitch) < 1e-10
        assert roll < -1e-6

    def test_tilt_angle_invariant_under_yaw(self, sim):
        """
        The underlying tilt angle acos(cos(roll)*cos(pitch)) must not change as
        yaw rotates. This is the actual geometric invariant: the same body_z
        vector has the same angular separation from vertical regardless of yaw.
        """
        bz = _unit([0.15, 0.10, 1.0])
        expected_tilt = math.degrees(math.acos(bz[2]))   # acos(bz_down) for unit bz
        for yaw in [0.0, 0.4, 1.2, -0.7, math.pi / 3, math.pi]:
            r, p = self._call(sim, bz, yaw)
            actual_tilt = math.degrees(math.acos(
                max(-1.0, min(1.0, math.cos(math.radians(r)) * math.cos(math.radians(p))))))
            assert actual_tilt == pytest.approx(expected_tilt, abs=1e-4), \
                f"Tilt angle changed at yaw={math.degrees(yaw):.1f} deg"

    def test_round_trip_via_rotation_matrix(self, sim):
        """
        R = Rz(yaw)*Ry(pitch)*Rx(roll) should reconstruct body_z = R[:,2]
        matching the original bz_ned (up to floating-point rounding).
        """
        for bz_raw, yaw in [
            ([0.0,  0.0, 1.0], 0.0),
            ([0.2,  0.0, 1.0], 0.3),
            ([0.0,  0.2, 1.0], -0.8),
            ([0.15, 0.1, 0.8], 1.2),
        ]:
            bz = _unit(bz_raw)
            roll_d, pitch_d = self._call(sim, bz, yaw)
            r, p = math.radians(roll_d), math.radians(pitch_d)
            Rz = _rot_z(yaw)
            cr, sr = math.cos(r), math.sin(r)
            cp, sp = math.cos(p), math.sin(p)
            Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
            R  = Rz @ Ry @ Rx
            bz_rec = R[:, 2]
            assert bz_rec == pytest.approx(bz, abs=1e-6), \
                f"Round-trip failed for bz={bz_raw} yaw={math.degrees(yaw):.1f}"

    @pytest.mark.parametrize("bz_raw,yaw", [
        ([0.0,  0.0, 1.0], 0.0),
        ([0.2,  0.0, 1.0], 0.0),
        ([0.0,  0.2, 1.0], 0.0),
        ([0.15, 0.1, 0.8], 0.5),
        ([0.15, 0.1, 0.8], -0.7),
        ([0.3, -0.2, 0.9], math.pi / 4),
        ([0.0,  0.3, 0.7], math.pi),
    ])
    def test_matches_python_reference(self, sim, bz_raw, yaw):
        """Cross-check: Lua result matches the Python reference formula."""
        bz = _unit(bz_raw)
        lua_r, lua_p = self._call(sim, bz, yaw)
        py_r, py_p   = _bz_ned_to_roll_pitch(bz, yaw)
        assert lua_r == pytest.approx(py_r, abs=1e-10), \
            f"roll mismatch bz={bz_raw} yaw={math.degrees(yaw):.1f}"
        assert lua_p == pytest.approx(py_p, abs=1e-10), \
            f"pitch mismatch bz={bz_raw} yaw={math.degrees(yaw):.1f}"
