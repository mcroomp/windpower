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
import json
import sys
from pathlib import Path

import numpy as np
import pytest


from controller import (
    compute_bz_altitude_hold,
    compute_rate_cmd_sqrt,
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


def _body_z_from_attitude_target(roll_deg, pitch_deg, yaw_deg):
    """Reconstruct body_z (NED) from ZYX roll/pitch/yaw attitude target."""
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)
    cy, sy = math.cos(y), math.sin(y)
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)

    bz_fwd = sp * cr
    bz_right = -sr
    bz_down = cp * cr

    bx = cy * bz_fwd - sy * bz_right
    by = sy * bz_fwd + cy * bz_right
    bz = bz_down
    return np.array([bx, by, bz], dtype=float)


# ── Constants sanity ─────────────────────────────────────────────────────────

def test_constants_have_expected_values(sim):
    """Key rawes.lua constants match their documented values."""
    f = sim.fns
    assert float(f.COL_CRUISE_FLIGHT_RAD) == pytest.approx(-0.18)
    assert float(f.COL_MIN_RAD)           == pytest.approx(-0.28)
    assert float(f.COL_MAX_RAD)           == pytest.approx(0.10)
    assert float(f.MASS_KG)               == pytest.approx(5.0)
    assert float(f.G_ACCEL)               == pytest.approx(9.81)
    assert float(f.KP_ALT)                == pytest.approx(0.010)
    assert float(f.KI_ALT)                == pytest.approx(0.001)
    assert float(f.KD_VZ)                 == pytest.approx(0.040)
    assert float(f.RATE_KP_OUTER)         == pytest.approx(2.5)
    assert float(f.RATE_ACCEL_MAX_RADSS)  == pytest.approx(4.0)


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


class TestRateCmdSqrt:
    def test_matches_python_reference(self, sim):
        """Lua compute_rate_cmd_sqrt mirrors Python for identity body frame."""
        bz_now = sim.lua_vec(0.0, 0.0, 1.0)
        angle = math.radians(15.0)
        bz_goal = sim.lua_vec(math.sin(angle), 0.0, math.cos(angle))
        r_lua = sim.vec_to_list(sim.fns.compute_rate_cmd_sqrt(bz_now, bz_goal, 2.5, 4.0, 0.02))
        r_py = compute_rate_cmd_sqrt(
            np.array([0.0, 0.0, 1.0]),
            np.array([math.sin(angle), 0.0, math.cos(angle)]),
            np.eye(3),
            kp=2.5,
            accel_max=4.0,
            dt=0.02,
        )
        np.testing.assert_allclose(r_lua, r_py, atol=1e-9)

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


class TestLuaCaptureAgainstStartingIc:
    def test_first_angle_command_is_close_to_ic_body_z(self, sim):
        """
        Using steady_state_starting.json, once GPS capture occurs in Lua steady
        mode, the first angle target should be close to the IC body_z.
        """
        ic_path = Path(__file__).resolve().parents[2] / "steady_state_starting.json"
        ic = json.loads(ic_path.read_text(encoding="utf-8"))

        pos0 = np.array(ic["pos"], dtype=float)
        R0 = np.array(ic["R0"], dtype=float)
        ic_bz = R0[:, 2]

        sim.armed = True
        sim.healthy = True
        sim.vehicle_mode = 20
        sim.R = R0
        sim.gyro = [0.0, 0.0, 0.0]
        sim.vel_ned = [0.0, 0.0, 0.0]
        sim.clear_messages()

        # Match the stack path: anchor encoded in EKF frame, GPS position at home.
        sim.set_param("mode", 1)
        sim.send_named_float("RAWES_ANN", -float(pos0[0]))
        sim.send_named_float("RAWES_ANE", -float(pos0[1]))
        sim.send_named_float("RAWES_AND", -float(pos0[2]))
        sim.pos_ned = [0.0, 0.0, 0.0]

        sim.send_named_float("RAWES_COL", float(ic["coll_eq_rad"]))
        sim.send_named_float("RAWES_TEN", float(ic["tension_eq_n"]))

        # Capture needs one run_flight call; angle command appears on the next.
        sim.run(0.20)

        assert bool(sim.fns.el_initialized()), "Lua did not capture GPS geometry"
        tgt = sim.guided_target
        assert tgt is not None, "Expected GUIDED angle target after capture"
        assert sim.guided_rate_target is None, "Expected angle-path target, not rate-path"

        cmd_bz = _body_z_from_attitude_target(
            tgt["roll_deg"], tgt["pitch_deg"], tgt["yaw_deg"]
        )
        cmd_bz = _unit(cmd_bz)

        dot = float(np.clip(np.dot(ic_bz, cmd_bz), -1.0, 1.0))
        angle_deg = math.degrees(math.acos(dot))

        assert angle_deg <= 10.0, (
            "Post-capture Lua angle target is not close to IC body_z: "
            f"angle={angle_deg:.2f} deg"
        )
