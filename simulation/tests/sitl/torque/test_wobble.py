"""
torque/test_wobble.py — High-tilt swashplate wobble test.

Simulates a RAWES hub under aggressive cyclic swashplate input: roll +/-20 deg,
pitch +/-15 deg, orbital frequency 0.10 Hz.

This represents the worst-case tilt the stationary assembly would experience
during a high-bank orbit, rapid heading correction, or flight in strong
crosswinds.  The test verifies the yaw PID remains stable even when the body
frame gyro Z component is significantly contaminated by the roll/pitch motion.

Why this stresses the yaw loop
--------------------------------
At 20 deg roll with droll/dt approx +/-12.6 deg/s (0.10 Hz), the gyro Z projection error
in the body frame is roughly sin(20 deg) x 12.6 deg/s approx 4.3 deg/s.  The yaw PID must
reject this apparent yaw rate without over-correcting.

GPS fusion is disabled (same as test_pitch_roll) because the large horizontal
accel component (g*sin(20 deg) approx 3.4 m/s^2) would trigger GPS Glitch.

EKF compass-tilt artifact -- why the threshold is high
------------------------------------------------------
The wobble profile uses roll = 20 deg*sin(wt) and pitch = 15 deg*cos(wt) (same
frequency, 90 deg phase offset).  This is circular tilting: the body-z direction
traces an ellipse at 0.10 Hz.  From a compass perspective this looks like
continuous yaw rotation: the horizontal magnetic field projection rotates at
approximately w*sin(theta_avg) approx 2*pi*0.10*sin(17.5 deg) approx 0.19 rad/s approx 10.8 deg/s.

ATTITUDE.yawspeed (what the test measures) therefore shows ~10-15 deg/s even
though physics psi_dot = 0.000 deg/s throughout (confirmed from mediator log).
This is a compass/EKF measurement limitation for circular tilt, not actual
hub rotation.  The ACRO PID uses raw-gyro feedback and keeps the hub still.
ATC_RAT_YAW_P = 0.001 is so small that even a 15 deg/s EKF error only produces
a ~0.015 correction — negligible.

Pass criterion
--------------
  After 40 s settle: max |ATTITUDE.yawspeed| < 16 deg/s over 20 s.
  (Physics psi_dot is confirmed 0 deg/s; the limit accommodates the ~11 deg/s
   compass-tilt artifact from circular tilting at +/-20 deg/0.10 Hz.)

Telemetry -> simulation/logs/torque_telemetry_wobble.csv
"""
from __future__ import annotations

import math
import pytest

from torque_test_utils  import run_observation_loop, save_telemetry, assert_yaw_rate

_SETTLE_S   = 40.0
_OBSERVE_S  = 20.0
_THRESHOLD  = math.radians(16.0)  # [rad/s] -- circular-tilt EKF compass artifact (see module docstring)
                                  # Physics psi_dot confirmed 0.000 deg/s; 16 deg/s accommodates the
                                  # ~10-13 deg/s ATTITUDE.yawspeed bias from compass-tilt coupling at +/-20 deg.


@pytest.mark.parametrize("torque_armed_profile", ["wobble"], indirect=True)
def test_wobble(torque_armed_profile):
    """
    Hub tilts aggressively: roll +/-20 deg, pitch +/-15 deg, at orbital frequency 0.10 Hz.
    Circular tilting (sin/cos at same frequency) creates ~11 deg/s apparent yaw in
    ATTITUDE.yawspeed via compass-tilt coupling.  Physics psi_dot remains 0 deg/s.

    Pass: max |ATTITUDE.yawspeed| < 16 deg/s after 40 s settle.  The high limit
    accommodates the EKF compass artifact; the ACRO PID (P=0.001) is too weak
    to respond meaningfully to this spurious signal.
    """
    ctx = torque_armed_profile
    rows: list = []

    # EK3_SRC1_POSXY=0 and EK3_SRC1_VELXY=0 are in _BASE_TORQUE_BOOT_PARAMS (boot file).
    # Writing EK3_SRC* via MAVLink post-boot triggers EKF forced reset; boot file avoids it.

    obs = run_observation_loop(
        ctx=ctx, rows=rows,
        settle_s=_SETTLE_S, observe_s=_OBSERVE_S,
        timeout_s=_SETTLE_S + _OBSERVE_S + 20.0,
    )

    save_telemetry(rows, "wobble", ctx.log)
    assert_yaw_rate(obs, _THRESHOLD, _SETTLE_S, ctx.log)
