"""
torque/test_wobble.py — High-tilt swashplate wobble test.

Simulates a RAWES hub under aggressive cyclic swashplate input: roll ±20° +
±5° harmonic, pitch ±15° + ±4° harmonic, orbital frequency ~0.10 Hz.

This represents the worst-case tilt the stationary assembly would experience
during a high-bank orbit, rapid heading correction, or flight in strong
crosswinds.  The test verifies the yaw PID remains stable even when the body
frame gyro Z component is significantly contaminated by the roll/pitch motion.

Why this stresses the yaw loop
--------------------------------
At 20° roll with droll/dt ≈ ±12.6°/s (0.10 Hz), the gyro Z projection error
in the body frame is roughly sin(20°) × 12.6°/s ≈ 4.3°/s.  The yaw PID must
reject this apparent yaw rate without over-correcting.

GPS fusion is disabled (same as test_pitch_roll) because the large horizontal
accel component (g·sin 20° ≈ 3.4 m/s²) would trigger GPS Glitch.

Pass criterion
--------------
  After 40 s settle: max |ψ_dot| < 5°/s over 20 s.
  The threshold is looser than constant-RPM (1°/s) to account for the
  ~4°/s gyro projection contamination from the large tilt motion.

Telemetry → simulation/logs/torque_telemetry_wobble.json
"""
from __future__ import annotations

import pytest

from torque_telemetry import TorqueTelemetryRecorder
from torque_test_utils  import run_observation_loop, save_telemetry, assert_yaw_rate

_SETTLE_S   = 40.0
_OBSERVE_S  = 20.0
_THRESHOLD  = 5.0     # °/s — ~4°/s gyro Z contamination from 20° tilt at 0.1 Hz


@pytest.mark.parametrize("torque_armed_profile", ["wobble"], indirect=True)
def test_wobble(torque_armed_profile):
    """
    Hub tilts aggressively: roll ±20° + harmonic, pitch ±15° + harmonic,
    at orbital frequency 0.10 Hz.  Simulates high cyclic swashplate tilt.

    The yaw rate PID must not diverge despite large gyro Z contamination
    from the tilt motion.  Pass: max |ψ_dot| < 5°/s after 40 s settle.
    """
    ctx = torque_armed_profile
    rec = TorqueTelemetryRecorder(meta={
        "test":                "wobble",
        "profile":             "wobble",
        "omega_axle_rads":     ctx.omega_axle,
        "roll_amplitude_deg":  20.0,
        "pitch_amplitude_deg": 15.0,
        "orbital_freq_hz":     0.10,
        "settle_s":            _SETTLE_S,
        "observe_s":           _OBSERVE_S,
        "threshold_degs":      _THRESHOLD,
    })

    # Disable GPS position/velocity fusion — 20° tilt → 3.4 m/s² horizontal
    # accel → GPS Glitch false positive without this.
    for pname, pval in [
        ("EK3_SRC1_POSXY", 0),
        ("EK3_SRC1_VELXY", 0),
    ]:
        ok = ctx.gcs.set_param(pname, pval, timeout=3.0)
        ctx.log.info("  %-25s = %g  ACK=%s", pname, pval, ok)

    obs = run_observation_loop(
        ctx=ctx, rec=rec,
        settle_s=_SETTLE_S, observe_s=_OBSERVE_S,
        timeout_s=_SETTLE_S + _OBSERVE_S + 20.0,
    )

    save_telemetry(rec, "wobble", ctx.log)
    assert_yaw_rate(obs, _THRESHOLD, _SETTLE_S, rec, ctx.log)
