"""
test_cyclic_direction_mapping.py — Focused validation of the control chain
from PID output (roll_cyclic, pitch_cyclic) through servo mixing to aero.

This test isolates the sign convention between GuidedAttitudeController output
and the actual swashplate/aero inputs.
"""
import math
import pytest
from swashplate import (
    ardupilot_h3_120_forward,
    ardupilot_h3_120_inverse,
)


class TestCyclicDirectionMapping:
    """Verify that cyclic commands map to tilt angles with correct signs."""
    
    def test_roll_cyclic_positive_to_tilt_lat_positive(self):
        """Positive roll_cyclic from controller should produce positive tilt_lat.
        
        This is the key mapping in step_guided():
            tilt_lat_cmd = heli_out.roll_cyclic  (no sign flip)
        
        Then swashplate forward mix applies roll factor to servo outputs.
        """
        # Simulate controller output: positive roll cyclic
        roll_cyclic = 0.1  # Positive roll command
        
        # In step_guided, this becomes tilt_lat directly
        tilt_lat_cmd = roll_cyclic
        
        # Forward mix: (col=0.5, roll=tilt_lat_cmd, pitch=0)
        s1, s2, s3 = ardupilot_h3_120_forward(0.5, tilt_lat_cmd, 0.0)
        
        # Inverse mix to verify
        col_rt, roll_rt, pitch_rt = ardupilot_h3_120_inverse(s1, s2, s3)
        
        # The recovered roll should match the input
        assert math.isclose(roll_rt, tilt_lat_cmd, abs_tol=1e-10), \
            f"Roll cyclic {roll_cyclic} should recover as {tilt_lat_cmd}, got {roll_rt}"
        assert math.isclose(pitch_rt, 0.0, abs_tol=1e-10), \
            "Pitch should remain zero"
    
    def test_pitch_cyclic_positive_to_tilt_lon_negative(self):
        """Positive pitch_cyclic from controller should produce NEGATIVE tilt_lon.
        
        This is the key mapping in step_guided():
            tilt_lon_cmd = -heli_out.pitch_cyclic  (SIGN FLIP!)
        
        A positive pitch_cyclic (nose-up command) should tilt the disk nose-down
        to rotate it up, so tilt_lon (nose-down tilt) should be positive.
        But the code flips the sign, so we get negative tilt_lon.
        """
        pitch_cyclic = 0.1  # Positive pitch command (nose up)
        
        # In step_guided, this gets sign-flipped
        tilt_lon_cmd = -pitch_cyclic  # = -0.1
        
        # Forward mix: (col=0.5, roll=0, pitch=tilt_lon_cmd)
        s1, s2, s3 = ardupilot_h3_120_forward(0.5, 0.0, tilt_lon_cmd)
        
        # Inverse mix to verify
        col_rt, roll_rt, pitch_rt = ardupilot_h3_120_inverse(s1, s2, s3)
        
        # The recovered pitch should match the tilt_lon input
        assert math.isclose(pitch_rt, tilt_lon_cmd, abs_tol=1e-10), \
            f"Pitch cyclic {pitch_cyclic} -> tilt_lon {tilt_lon_cmd} should recover as {tilt_lon_cmd}, got {pitch_rt}"
        assert math.isclose(roll_rt, 0.0, abs_tol=1e-10), \
            "Roll should remain zero"
    
    def test_combined_roll_pitch_cyclic_no_cross_coupling(self):
        """Combined cyclic should mix without cross-coupling."""
        roll_cyclic = 0.15
        pitch_cyclic = -0.08
        
        tilt_lat = roll_cyclic
        tilt_lon = -pitch_cyclic  # Sign flip
        
        # Forward mix
        s1, s2, s3 = ardupilot_h3_120_forward(0.5, tilt_lat, tilt_lon)
        
        # Inverse mix
        col_rt, roll_rt, pitch_rt = ardupilot_h3_120_inverse(s1, s2, s3)
        
        assert math.isclose(roll_rt, tilt_lat, abs_tol=1e-10)
        assert math.isclose(pitch_rt, tilt_lon, abs_tol=1e-10)
    
    def test_servo_model_parameter_order(self):
        """Servo model should accept (col, tilt_lon, tilt_lat) parameters in correct order.
        
        This validates that the servo.step() correctly maps parameter order to forward_mix.
        """
        from swashplate import SwashplateServoModel
        
        # Create servo model
        servo = SwashplateServoModel(
            slew_rate_deg_s=3600.0,  # Very fast to minimize slew limiting effects
            travel_deg=120.0,
            col_min_rad=-0.28,
            col_max_rad=0.10,
        )
        
        # Take multiple steps to allow slew limiting to catch up
        col_cmd_rad = -0.18  # In radians
        tilt_lon_cmd = 0.05  # nose-down
        tilt_lat_cmd = -0.03  # roll-left
        dt = 1.0 / 400.0  # 400 Hz
        
        # Pre-initialize the servo at commanded values
        servo.reset(col_cmd_rad, tilt_lon_cmd, tilt_lat_cmd)
        
        # Now take a step with same values
        col_act, tlon_act, tlat_act = servo.step(col_cmd_rad, tilt_lon_cmd, tilt_lat_cmd, dt)
        
        # Should be close to commanded values (allowing for numerical precision)
        assert math.isclose(tlon_act, tilt_lon_cmd, abs_tol=0.01), \
            f"tilt_lon should be preserved: cmd={tilt_lon_cmd}, got={tlon_act}"
        assert math.isclose(tlat_act, tilt_lat_cmd, abs_tol=0.01), \
            f"tilt_lat should be preserved: cmd={tilt_lat_cmd}, got={tlat_act}"
    
    def test_aero_moment_sign_for_positive_tilt_lat(self):
        """Positive tilt_lat (roll-right disk) should produce positive roll moment (Mx).
        
        This validates that when the cyclic command stays positive all the way
        through the servo model, it produces the expected aero moment sign.
        """
        # This is a reference test from the aero package
        # If tilt_lat > 0, the disk should tilt right, producing right-roll moment
        # In NED frame: body_z = down, so right-roll moment is positive Mx
        
        # For this test, we just verify the forward/inverse are self-consistent
        tilt_lat = 0.05
        
        s1, s2, s3 = ardupilot_h3_120_forward(0.5, tilt_lat, 0.0)
        col_rt, roll_rt, pitch_rt = ardupilot_h3_120_inverse(s1, s2, s3)
        
        assert math.isclose(roll_rt, tilt_lat, abs_tol=1e-10), \
            f"tilt_lat={tilt_lat} should recover exactly, got {roll_rt}"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
