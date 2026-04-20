"""
test_gps_fusion_layers.py -- GPS fusion diagnostic with dual-GPS moving-baseline.

Two GPS antennas 50 cm apart (+-25 cm along body X) give heading from RELPOSNED.
Yaw is known from first GPS fix -- no motion needed.

Hub is held at tether equilibrium with constant sensor values (zero vel, zero gyro,
fixed attitude). GPS fuses within ~34 s.

Architecture: mediator_static.py subprocess runs the SITL lockstep loop with
static sensor values. The test drives GCS in the foreground with blocking calls
(sim_sleep, arm, wait for STATUSTEXT). Same pattern as all other stack tests.

Run:
  bash simulation/dev.sh test-stack -n 1 -k test_gps_fusion_dual_gps
"""
from __future__ import annotations

import json
import logging
import sys
from pathlib import Path

import numpy as np
import pytest

_SIM_DIR  = Path(__file__).resolve().parents[3]
_SITL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_SIM_DIR))
sys.path.insert(0, str(_SITL_DIR))

from stack_infra import StackConfig, _static_stack
from gcs import RawesGCS

_STEADY_STATE_JSON = _SIM_DIR / "steady_state_starting.json"

log = logging.getLogger("test_gps_layers")

_GRAVITY        = 9.81
_T_WARMUP       = 12.0   # sim-seconds before arming (EKF tilt align + first GPS fix)
_FUSION_TIMEOUT = 40.0   # sim-seconds to wait for "is using gps" after arm


# ---------------------------------------------------------------------------
# test_gps_fusion_dual_gps
# ---------------------------------------------------------------------------

def test_gps_fusion_dual_gps(tmp_path, request):
    """
    GPS fusion using dual-GPS moving-baseline heading (EK3_SRC1_YAW=2).

    --- Why this works ---

    ArduPilot EKF3 requires two things before fusing GPS position/velocity:
      1. Yaw alignment complete (yawAlignComplete flag set).
      2. delAngBiasLearned (gyro delta-angle bias converged).

    With EK3_SRC1_YAW=2 (dual-antenna RELPOSNED), yaw is derived directly from
    the NED baseline vector between the two antennas. Yaw is known from the
    very first GPS fix -- no motion required.

    delAngBiasLearned also converges without motion. With constant-zero gyro
    input, the EKF bias estimator converges to zero bias. GPS fuses at ~34 s
    from test start.

    --- Architecture ---

    mediator_static.py subprocess runs the SITL lockstep loop with static
    sensor values. The foreground connects GCS, waits for EKF warmup, arms,
    then waits for GPS fusion. Same pattern as all other stack tests -- no
    threading in the test file.

    --- Sensor input ---

    Absolutely static: same packet every lockstep step.
      pos   = p_eq  (tether equilibrium from steady_state_starting.json)
      vel   = [0, 0, 0]
      gyro  = [0, 0, 0]
      accel = [0, 0, -g]  (flat orientation: gravity straight down in body frame)
      rpy   = [0, 0, 0]   (flat orientation)

    --- Why flat orientation (not tether-equilibrium tilt) ---

    At 82 deg tilt, the large horizontal gravity component (g*sin(82) = 9.7 m/s^2)
    amplifies any EKF attitude error into large velocity drift. When ArduPilot
    first transitions to GPS position fusion (~t=21 s), it drives the gyro bias
    state to EK3_GBIAS_LIM = 0.01 rad/s in an attempt to explain the velocity
    error. The clamped bias then causes attitude drift, growing position
    innovations, and GPS_GTA = 1 (GPS innovation gate test permanently failing).
    With a flat body, gravity cancels perfectly, bias stays at zero, and GPS fuses.

    The GPS fusion mechanism (dual-GPS yaw, delAngBiasLearned, EKF3 state machine)
    is unaffected by the choice of tilt.

    --- Timeline ---
      t=0-12 s   Static hold. EKF tilt aligns, GPS acquires first fix.
      t~12 s     Arm.
      t~22 s     delAngBiasLearned converges (zero gyro = zero bias).
      t~22 s     "EKF3 IMU0 is using GPS" -- GPS fused.
    """
    from pymavlink import mavutil

    ss  = json.loads(_STEADY_STATE_JSON.read_text())
    pos = np.array(ss["pos"], dtype=float)

    # Flat orientation: rpy=0, accel=gravity down, GPS2 heading=0 (North).
    # This avoids the high-tilt gyro-bias divergence that prevents GPS fusion.
    rpy        = np.zeros(3)
    accel_body = np.array([0.0, 0.0, -_GRAVITY])
    vel        = np.zeros(3)
    gyro       = np.zeros(3)

    with _static_stack(
        tmp_path, test_name=request.node.name,
        pos=pos, vel=vel, rpy=rpy, accel_body=accel_body, gyro=gyro,
    ) as ctx:

        def _assert_alive() -> None:
            if ctx.mediator_proc.poll() is not None:
                tail = (
                    ctx.mediator_log.read_text(errors="replace")[-1000:]
                    if ctx.mediator_log is not None and ctx.mediator_log.exists()
                    else ""
                )
                pytest.fail(
                    f"static mediator exited (rc={ctx.mediator_proc.returncode})"
                    + (f"\n{tail}" if tail else "")
                )

        gcs = RawesGCS(address=StackConfig.GCS_ADDRESS, mavlog_path=ctx.mavlink_log)
        try:
            # connect() blocks until the first heartbeat from ArduPilot.
            # The mediator subprocess keeps SITL alive so ArduPilot can boot.
            gcs.connect(timeout=30.0)
            gcs.start_heartbeat(rate_hz=1.0)
            gcs.request_stream(0, 10)

            # Wait for EKF tilt alignment and first GPS fix before arming.
            ctx.log.info(
                "Waiting %.1f sim-s before arm (EKF warmup + first GPS fix)", _T_WARMUP,
            )
            gcs.sim_sleep(_T_WARMUP, check=_assert_alive)

            # Switch to ACRO (mode 1) before arming.
            gcs._mav.mav.command_long_send(
                gcs._target_system, gcs._target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 1.0,
                0, 0, 0, 0, 0,
            )

            # Force-arm: bypasses pre-arm checks.
            # rc_override keeps the motor interlock released every 0.5 s sim-time.
            gcs.arm(force=True, rc_override={3: 1700, 8: 2000})
            ctx.log.info("Armed at sim t=%.1f s -- waiting for GPS fusion", gcs.sim_now())

            # Refresh RC override at ~2.5 Hz while waiting.
            t_last_rc = gcs.sim_now()
            t_start   = gcs.sim_now()
            fused     = False

            while gcs.sim_now() - t_start < _FUSION_TIMEOUT:
                _assert_alive()

                if gcs.sim_now() - t_last_rc >= 0.4:
                    gcs.send_rc_override({3: 1700, 8: 2000})
                    t_last_rc = gcs.sim_now()

                msg = gcs._recv(blocking=True, timeout=0.2)
                if msg is None:
                    continue

                if msg.get_type() == "STATUSTEXT":
                    text = msg.text.strip()
                    ctx.log.info("[t=%5.1f] %s", gcs.sim_now(), text)
                    if "is using gps" in text.lower():
                        fused = True
                        break

            if not fused:
                pytest.fail(
                    f"GPS fusion timed out after {_FUSION_TIMEOUT:.0f} s post-arm"
                )

            ctx.log.info("GPS fused at sim t=%.1f s -- PASS", gcs.sim_now())

        finally:
            try:
                gcs.close()
            except Exception:
                pass
