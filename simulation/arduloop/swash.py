"""
Swashplate phase rotation, matching `AP_MotorsHeli_Swash`.

Only the *cyclic* rotation matters for the control loop: a positive
`phase_angle_deg` rotates the (roll_cmd, pitch_cmd) vector clockwise when
viewed from above, so that a pure pitch-axis cyclic command produces a roll
moment offset by `phase_angle_deg`. This compensates the gyroscopic precession
of the rotor at the *bandwidth at which the gain is tuned*.

This module does **not** mix individual servo positions — the simulator works
with normalised (roll_cyclic, pitch_cyclic) outputs in [-1, 1].
"""

from __future__ import annotations

import math


class SwashH3:
    def __init__(self, phase_angle_deg: float = 0.0):
        self.phase_angle_deg = float(phase_angle_deg)

    def set_phase(self, phase_angle_deg: float) -> None:
        self.phase_angle_deg = float(phase_angle_deg)

    def mix(self, roll_cmd: float, pitch_cmd: float) -> tuple[float, float]:
        """Rotate the (roll, pitch) cyclic command by the phase angle.

        Sign convention matches ArduPilot's `add_servo_angle(.., pos - phase, ..)`:
        a positive phase angle subtracts from the servo azimuth, which when
        re-projected onto the body frame is equivalent to rotating the cyclic
        command by `+phase` about the rotor axis.
        """
        a = math.radians(self.phase_angle_deg)
        ca = math.cos(a)
        sa = math.sin(a)
        roll_out  =  ca * roll_cmd + sa * pitch_cmd
        pitch_out = -sa * roll_cmd + ca * pitch_cmd
        return roll_out, pitch_out
