"""
test_tail_tilt.py -- Diagnostic: does H_TAIL_TYPE=4 cause swashplate tilt?

Holds the hub stationary at the steady-state equilibrium orientation
(from steady_state_starting.json) with H_TAIL_TYPE=4 + SERVO4 configured
for the GB4008, then checks whether ArduPilot commands any significant
lateral or longitudinal cyclic tilt in ACRO mode with neutral sticks.

This isolates the regression found during telemetry refactoring: adding
H_TAIL_TYPE=4 to rawes_sitl_defaults.parm caused tilt_lat=0.432 during
the kinematic phase, destabilising the hub at kinematic exit.

Root cause: vehicle was in STABILIZE (mode 0). STABILIZE targets roll=0°/
pitch=0°, generating constant cyclic at 65° tilt. Fix: set ACRO before arm
(wait for EKF3 active first so DO_SET_MODE is accepted).

Pass criterion: |tilt_lon| < 0.10 and |tilt_lat| < 0.10 throughout 15 s
of observation (neutral sticks, stationary at equilibrium orientation).
"""
import json
import logging
import math
import sys
from pathlib import Path

import numpy as np
import pytest

_SIM_DIR  = Path(__file__).resolve().parents[3]
_SITL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_SIM_DIR))
sys.path.insert(0, str(_SITL_DIR))

from stack_infra import StackConfig, _arm_sequence, _static_stack
from gcs import RawesGCS
from swashplate import h3_inverse_mix, pwm_to_normalized

log = logging.getLogger("test_tail_tilt")

# ---------------------------------------------------------------------------
# Load equilibrium IC from steady_state_starting.json
# ---------------------------------------------------------------------------
_IC_PATH = _SIM_DIR / "steady_state_starting.json"

def _load_ic():
    with open(_IC_PATH) as f:
        ic = json.load(f)
    pos = np.array(ic["pos"])
    R0  = np.array(ic["R0"])
    vel = np.zeros(3)   # stationary hold
    gyro = np.zeros(3)

    # ZYX Euler angles from rotation matrix
    roll  = math.atan2(float(R0[2, 1]), float(R0[2, 2]))
    pitch = -math.asin(float(np.clip(R0[2, 0], -1.0, 1.0)))
    yaw   = math.atan2(float(R0[1, 0]), float(R0[0, 0]))
    rpy   = np.array([roll, pitch, yaw])

    # Specific force in body frame: R.T @ (accel_world - gravity)
    # For stationary hold: accel_world = 0, gravity_ned = [0, 0, +9.81]
    # => accel_body = R.T @ (0 - [0,0,9.81]) = R.T @ [0,0,-9.81]
    accel_body = R0.T @ np.array([0.0, 0.0, -9.81])

    return pos, vel, rpy, accel_body, gyro

# ---------------------------------------------------------------------------
# H_TAIL_TYPE=4 extra params (the change under investigation)
# ---------------------------------------------------------------------------
_TAIL4_EXTRA_PARAMS: dict[str, float] = {
    "H_TAIL_TYPE":      4.0,   # DDFP CCW: ATC_RAT_YAW PID drives SERVO4
    "SERVO4_MIN":       800.0,
    "SERVO4_MAX":       2000.0,
    "SERVO4_TRIM":      800.0,
}

_OBS_SECONDS   = 15.0
_TILT_LIMIT    = 0.10   # normalised [-1,1]; > this is a failure


@pytest.mark.timeout(120)
def test_tail_tilt_h_tail_type_4(tmp_path, request):
    """
    Static hold at IC equilibrium with H_TAIL_TYPE=4: tilt must stay near zero.

    If tilt_lat or tilt_lon exceeds 0.10 in ACRO mode with neutral sticks,
    H_TAIL_TYPE=4 is corrupting the swashplate output — do NOT add it to
    rawes_sitl_defaults.parm for flight tests.
    """
    pos, vel, rpy, accel_body, gyro = _load_ic()
    log.info(
        "IC: pos=(%.1f, %.1f, %.1f) m  rpy=(%.1f, %.1f, %.1f) deg",
        *pos, *np.degrees(rpy),
    )

    with _static_stack(
        tmp_path, test_name=request.node.name,
        pos=pos, vel=vel, rpy=rpy, accel_body=accel_body, gyro=gyro,
        extra_boot_params=_TAIL4_EXTRA_PARAMS,
    ) as ctx:

        def _assert_alive() -> None:
            proc = ctx.mediator_proc
            if proc is not None and proc.poll() is not None:
                pytest.fail(f"static mediator exited (rc={proc.returncode})")

        gcs = RawesGCS(address=StackConfig.GCS_ADDRESS, watchdog=_assert_alive)
        try:
            gcs.connect(timeout=20.0)
            gcs.start_heartbeat()
            gcs.request_stream(0, 10)

            if not gcs.wait_ekf_attitude(timeout=20.0):
                pytest.fail("EKF never aligned")

            # ArduPilot ignores DO_SET_MODE while AHRS is on DCM (EKF3 not yet active).
            # Wait for "EKF3 active" STATUSTEXT before handing off to _arm_sequence.
            log.info("Waiting for EKF3 to become active before switching to ACRO...")
            t_ekf3_deadline = gcs.sim_now() + 30.0
            ekf3_active = False
            while gcs.sim_now() < t_ekf3_deadline:
                msg = gcs._recv(
                    type=["STATUSTEXT", "HEARTBEAT"], blocking=True, timeout=0.5
                )
                if msg is None:
                    continue
                if msg.get_type() == "HEARTBEAT":
                    gcs.send_rc_override({8: 2000})
                elif msg.get_type() == "STATUSTEXT":
                    txt = msg.text.rstrip("\x00").strip()
                    if "EKF3" in txt and "active" in txt.lower():
                        log.info("EKF3 active: %s", txt)
                        ekf3_active = True
                        break
            if not ekf3_active:
                pytest.fail("EKF3 never became active within 30 s")

            # Standard arm sequence: ACRO before arm + hard assert.
            _arm_sequence(
                gcs, log,
                rc_override={8: 2000},
                fail=pytest.fail,
                mode_timeout=10.0,
                arm_timeout=20.0,
            )
            log.info("Armed -- observing servo outputs for %.0f s", _OBS_SECONDS)

            max_tilt_lon = 0.0
            max_tilt_lat = 0.0
            samples = []

            def _handle(msg, t_rel):
                nonlocal max_tilt_lon, max_tilt_lat
                if msg is None:
                    return None
                if msg.get_type() == "SERVO_OUTPUT_RAW":
                    s1 = pwm_to_normalized(msg.servo1_raw)
                    s2 = pwm_to_normalized(msg.servo2_raw)
                    s3 = pwm_to_normalized(msg.servo3_raw)
                    _, tilt_lon, tilt_lat = h3_inverse_mix(s1, s2, s3)
                    if abs(tilt_lon) > max_tilt_lon:
                        max_tilt_lon = abs(tilt_lon)
                    if abs(tilt_lat) > max_tilt_lat:
                        max_tilt_lat = abs(tilt_lat)
                    if len(samples) % 20 == 0:
                        log.info(
                            "t=%.1fs  servo=[%d,%d,%d]  tilt_lon=%.3f  tilt_lat=%.3f",
                            t_rel,
                            msg.servo1_raw, msg.servo2_raw, msg.servo3_raw,
                            tilt_lon, tilt_lat,
                        )
                    samples.append((t_rel, tilt_lon, tilt_lat))
                return None

            t_start = gcs.sim_now()
            t_keepalive = t_start
            while True:
                proc = ctx.mediator_proc
                if proc is not None and getattr(proc, "poll", lambda: None)() is not None:
                    pytest.fail("static mediator exited unexpectedly")
                now = gcs.sim_now()
                if now - t_start >= _OBS_SECONDS:
                    break
                if now - t_keepalive >= 0.5:
                    gcs.send_rc_override({1: 1500, 2: 1500, 4: 1500, 8: 2000})
                    t_keepalive = now
                msg = gcs._recv(type="SERVO_OUTPUT_RAW", blocking=True, timeout=0.2)
                _handle(msg, now - t_start)

            log.info(
                "Result: max_tilt_lon=%.3f  max_tilt_lat=%.3f  (limit=%.2f)",
                max_tilt_lon, max_tilt_lat, _TILT_LIMIT,
            )

            failures = []
            if max_tilt_lon > _TILT_LIMIT:
                failures.append(
                    f"max_tilt_lon={max_tilt_lon:.3f} > {_TILT_LIMIT} "
                    f"-- H_TAIL_TYPE=4 is driving swashplate lon tilt"
                )
            if max_tilt_lat > _TILT_LIMIT:
                failures.append(
                    f"max_tilt_lat={max_tilt_lat:.3f} > {_TILT_LIMIT} "
                    f"-- H_TAIL_TYPE=4 is driving swashplate lat tilt"
                )
            assert not failures, "\n".join(failures)

        finally:
            try:
                gcs.close()
            except Exception:
                pass
