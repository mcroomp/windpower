"""
viz3d/torque_telemetry.py — TorqueTelemetryFrame: internal display type.

Used only by visualize_torque.py for rendering.
Telemetry is written/read as canonical TelRow CSV via telemetry_csv.py.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class TorqueTelemetryFrame:
    """One snapshot of the hub/motor system state (display convention: ENU)."""

    # Time
    t: float = 0.0                   # simulation time [s]

    # Hub attitude (ENU display convention — yaw negated vs NED storage)
    psi_deg: float = 0.0             # hub yaw angle [deg]   (display, ENU)
    psi_dot_degs: float = 0.0        # hub yaw rate [deg/s]  (display, ENU)
    roll_deg: float = 0.0            # hub roll [deg]
    pitch_deg: float = 0.0           # hub pitch [deg]

    # Motor command
    throttle: float = 0.0            # equilibrium throttle (computed by visualiser)
    servo_pwm_us: int = 0            # Ch4/Ch9 PWM [us]

    # Physics model state
    omega_rotor_rads: float = 0.0    # rotor hub spin rate [rad/s]
    q_bearing_nm: float = 0.0        # bearing drag torque on hub [N.m]
    q_motor_nm: float = 0.0          # motor reaction torque on hub [N.m]

    # Phase flag
    phase: str = "DYNAMIC"           # "STARTUP" or "DYNAMIC"

    @property
    def psi_rad(self) -> float:
        return math.radians(self.psi_deg)

    @property
    def psi_dot_rads(self) -> float:
        return math.radians(self.psi_dot_degs)

    @property
    def net_torque_nm(self) -> float:
        """Net torque on hub = bearing - motor [N.m]."""
        return self.q_bearing_nm - self.q_motor_nm
