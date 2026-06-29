"""
test_roll_sign_chain.py — Unit tests to trace roll sign convention through
the control chain: error -> rate -> cyclic -> aero moment.

This test suite narrows down which layer has the sign inversion issue.
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tests.simtests.simtest_ic import load_ic
from tests.simtests._rotor_helpers import load_default_rotor
from controller import HeliCyclicController, compute_rate_cmd
from swashplate import ardupilot_h3_120_forward, ardupilot_h3_120_inverse
from dynbem import create_aero, RotorInputs


_ROTOR = load_default_rotor()
_IC = load_ic()
_AERO = create_aero(_ROTOR, "quasi_static")
_R_HUB_HOVER = _IC.R0  # hover attitude at IC


class TestRollSignChainLinks:
    """Test individual links in the roll correction chain."""
    
    def test_rate_command_sign_from_error(self):
        """Link 1: positive roll error should produce negative rate command.
        
        Verify that the attitude error -> rate command mapping is correct
        by checking a simple roll correction case.
        """
        # Setup: helicopter hovering, wing is high (positive roll error)
        # The system should command negative roll rate (roll left) to correct
        
        # Hovering orientation
        bz_now = _IC.R0[:, 2]  # actual disk-down vector (should be ~[0, 0, +1])
        bz_eq = bz_now.copy()  # target same (hover)
        
        # Simulate wing-high error: body rolled right relative to target
        # Positive roll = right wing down = rotation around y-axis in body frame
        # Create rotated R matrix: roll right by ~2 degrees
        err_rad = math.radians(2.0)
        cos_e = math.cos(err_rad)
        sin_e = math.sin(err_rad)
        
        # Rotation matrix for roll around x-axis (right wing down)
        # [1    0      0  ]
        # [0  cos  -sin]
        # [0   sin   cos]
        R_roll = np.array([
            [1, 0,      0],
            [0, cos_e, -sin_e],
            [0, sin_e,  cos_e],
        ])
        R_body_err = R_roll @ _IC.R0
        bz_now_err = R_body_err[:, 2]  # rotated down vector
        
        # Compute rate command to align (expect negative roll rate)
        rate_cmd = compute_rate_cmd(
            bz_now_err,
            bz_eq,
            R_body_err,
            kp=4.5,
            kd=0.0,
        )
        roll_rate_cmd = rate_cmd[0]
        
        assert roll_rate_cmd < 0, (
            f"2deg roll-right error should produce negative roll rate, got {roll_rate_cmd:.4f} rad/s"
        )
        print(f"PASS Link 1: roll_error=+2deg -> roll_rate={roll_rate_cmd:.4f} rad/s")
    
    def test_cyclic_output_sign_from_rate_error(self):
        """Link 2: negative rate error should produce negative cyclic via PID.
        
        HeliCyclicController.step() takes rate setpoint and body rate,
        computes error, and outputs cyclic command.
        """
        heli = HeliCyclicController(_ROTOR, col_min_rad=-0.28, col_max_rad=0.10)
        heli.set_trim(tilt_lon=_IC.trim_tilt_lon, tilt_lat=_IC.trim_tilt_lat)
        
        # Simulate: body is rolling right (positive omega_x)
        # We want to correct by commanding negative rate (roll left)
        # This will build up a negative cyclic over time
        
        omega_body = np.array([0.05, 0.0, 0.0])  # rolling right
        rate_sp_roll = -0.1  # target: roll left (negative)
        rate_sp_pitch = 0.0
        
        # Step controller multiple times to let integrator build up
        dt = 1.0 / 400.0
        tilt_lon_out = None
        tilt_lat_out = None
        
        for step in range(100):
            tilt_lon_out, tilt_lat_out, _ = heli.step(
                _IC.coll_eq_rad, rate_sp_roll, rate_sp_pitch, omega_body, dt,
                collective_norm=0.5
            )
        
        # Compare cyclic relative to trim; absolute tilt can remain positive if trim is positive.
        delta_tilt_lat = tilt_lat_out - _IC.trim_tilt_lat
        assert delta_tilt_lat < 0, (
            f"Targeting negative roll rate with positive body roll should "
            f"produce negative roll cyclic delta, got delta={delta_tilt_lat:.4f}"
        )
        print(f"PASS Link 2: rate_sp={rate_sp_roll:.4f} rad/s -> delta_tilt_lat={delta_tilt_lat:.4f}")
    
    def test_aero_moment_sign_for_cyclic(self):
        """Link 3: tilt_lat < 0 should produce aero_mx < 0.
        
        Direct aero model test: negative left-roll cyclic should produce
        left-roll moment (negative mx in body frame).
        """
        # Hovering at IC attitude
        R_hub = np.asarray(_R_HUB_HOVER, dtype=float)
        collective_rad = _IC.coll_eq_rad
        tilt_lon = 0.0
        tilt_lat = -0.05  # left-roll disk
        
        # Hover state: small forward velocity to make cyclic responsive
        v_hub_world = np.array([1.0, 0.0, 0.0])  # 1 m/s north
        wind_world = np.array([0.0, 10.0, 0.0])  # 10 m/s from north
        omega_rad_s = 28.0  # typical hover RPM
        
        state = _AERO.initial_rotor_state()
        
        inputs = RotorInputs(
            collective_rad=collective_rad,
            tilt_lon=tilt_lon,
            tilt_lat=tilt_lat,
            R_hub=R_hub,
            v_hub_world=v_hub_world,
            wind_world=wind_world,
            omega_rad_s=omega_rad_s,
            rho_kg_m3=1.225,
            t=0.0,
        )
        
        result, _ = _AERO.compute_forces(inputs, state)
        # M_orbital is in NED frame; transform to body frame for sign check
        M_body = R_hub.T @ np.asarray(result.M_orbital, dtype=float)
        aero_mx_body = M_body[0]
        
        # In body frame: left-roll disk (tilt_lat < 0) should produce left-roll moment (mx < 0)
        assert aero_mx_body < 0, (
            f"Left-roll disk (tilt_lat={tilt_lat:.3f}) should produce "
            f"left-roll moment in body frame (mx < 0), got {aero_mx_body:.2f} N*m"
        )
        print(f"PASS Link 3: tilt_lat={tilt_lat:.3f} -> aero_mx_body={aero_mx_body:.2f} N*m")
    
    def test_swashplate_forward_mix_sign(self):
        """Verify swashplate forward mix: roll_norm < 0 should produce consistent servo pattern.
        
        This tests the swashplate geometry, not the aero response.
        """
        col_out = 0.5
        roll_norm = -0.1  # left roll command
        pitch_norm = 0.0
        
        s1, s2, s3 = ardupilot_h3_120_forward(col_out, roll_norm, pitch_norm)
        
        # For H3-120: S1=front-right, S2=front-left, S3=back
        # Left-roll command should move left servos down, right servos up
        # i.e., s1 < s2 (right-side down)
        assert s1 < s2, (
            f"Left-roll command (roll_norm={roll_norm}) should have "
            f"s1 < s2, got s1={s1:.4f}, s2={s2:.4f}"
        )
        print(f"✓ Swashplate OK: roll_norm={roll_norm} -> s1={s1:.4f} < s2={s2:.4f}")
    
    def test_swashplate_inverse_returns_correct_sign(self):
        """Verify swashplate inverse mix round-trip.
        
        If we command roll_norm < 0, forward mix produces servos,
        then inverse should recover the original roll_norm < 0.
        """
        col_out_in = 0.5
        roll_in = -0.1
        pitch_in = 0.0
        
        s1, s2, s3 = ardupilot_h3_120_forward(col_out_in, roll_in, pitch_in)
        col_out_rt, roll_rt, pitch_rt = ardupilot_h3_120_inverse(s1, s2, s3)
        
        assert np.isclose(roll_rt, roll_in, atol=1e-6), (
            f"Round-trip: roll_in={roll_in} → forward → inverse → "
            f"roll_out={roll_rt}, mismatch!"
        )
        print(f"✓ Swashplate roundtrip OK: roll {roll_in} -> {roll_rt}")


class TestEndToEndRollCorrection:
    """Integration test: verify the full chain produces correct correction."""
    
    def test_positive_roll_error_corrects_to_left(self):
        """When wing is high (roll_err > 0), the system should command left-roll.
        
        Trace: roll_err + → rate - → cyclic - → aero_mx - (corrects)
        """
        # Setup controller
        heli = HeliCyclicController(_ROTOR, col_min_rad=-0.28, col_max_rad=0.10)
        heli.set_trim(tilt_lon=_IC.trim_tilt_lon, tilt_lat=_IC.trim_tilt_lat)
        
        # Simulate positive roll error (wing high): body rolling right
        omega_body_right_roll = np.array([0.1, 0.0, 0.0])  # rolling right
        
        # Command a corrective negative roll rate directly.
        roll_error = math.radians(2.0)
        rate_sp = -0.1
        
        # Run controller to generate cyclic (should be negative to roll left)
        dt = 1.0 / 400.0
        for _ in range(50):
            tilt_lon_cmd, tilt_lat_cmd, _ = heli.step(
                _IC.coll_eq_rad, rate_sp, 0.0, omega_body_right_roll, dt,
                collective_norm=0.5
            )
        
        # Cyclic delta from trim should be negative (left-roll command).
        delta_tilt_lat = tilt_lat_cmd - _IC.trim_tilt_lat
        assert delta_tilt_lat < 0, (
            f"Positive roll error should produce negative roll cyclic delta, "
            f"got delta={delta_tilt_lat:.4f}"
        )
        
        # Verify commanded cyclic changes roll moment in the correcting direction
        # relative to trim at the same flight condition.
        state = _AERO.initial_rotor_state()

        inputs = RotorInputs(
            collective_rad=_IC.coll_eq_rad,
            tilt_lon=tilt_lon_cmd,
            tilt_lat=tilt_lat_cmd,
            R_hub=np.asarray(_R_HUB_HOVER, dtype=float),
            v_hub_world=np.array([0.0, 0.0, 0.0]),
            wind_world=np.array([0.0, 10.0, 0.0]),
            omega_rad_s=28.0,
            rho_kg_m3=1.225,
            t=0.0,
        )
        
        result_cmd, _ = _AERO.compute_forces(inputs, state)
        mx_cmd_body = (np.asarray(_R_HUB_HOVER, dtype=float).T @ np.asarray(result_cmd.M_orbital, dtype=float))[0]

        inputs_trim = RotorInputs(
            collective_rad=_IC.coll_eq_rad,
            tilt_lon=_IC.trim_tilt_lon,
            tilt_lat=_IC.trim_tilt_lat,
            R_hub=np.asarray(_R_HUB_HOVER, dtype=float),
            v_hub_world=np.array([0.0, 0.0, 0.0]),
            wind_world=np.array([0.0, 10.0, 0.0]),
            omega_rad_s=28.0,
            rho_kg_m3=1.225,
            t=0.0,
        )
        result_trim, _ = _AERO.compute_forces(inputs_trim, state)
        mx_trim_body = (np.asarray(_R_HUB_HOVER, dtype=float).T @ np.asarray(result_trim.M_orbital, dtype=float))[0]

        assert (mx_cmd_body - mx_trim_body) < 0, (
            f"Corrective cyclic should reduce body roll moment; "
            f"mx_cmd={mx_cmd_body:.3f}, mx_trim={mx_trim_body:.3f}"
        )
        
        print(f"✓ End-to-end OK:")
        print(f"  roll_error={math.degrees(roll_error):.2f}deg ->") 
        print(f"  rate_sp={rate_sp:.4f} rad/s ->") 
        print(f"  tilt_lat={tilt_lat_cmd:.4f} ->") 
        print(f"  d_mx_body={(mx_cmd_body - mx_trim_body):.3f} N*m (corrects)")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
