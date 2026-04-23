"""
test_tail_tilt.py -- ACRO static-hold gate: no swashplate tilt, no PID firing.

Holds the hub stationary at the steady-state equilibrium orientation
(from steady_state_starting.json) with the production config
(H_TAIL_TYPE=4 DDFP CCW from rawes_sitl_defaults.parm), then checks:

  1. Swashplate tilt: |tilt_lon| < 0.10 and |tilt_lat| < 0.10
  2. Attitude PID: ATTITUDE_TARGET body_roll/pitch_rate < 0.05 rad/s

Both checks must pass for 15 s with neutral sticks in ACRO mode.

History: adding H_TAIL_TYPE=4 to rawes_sitl_defaults.parm initially caused
tilt_lat=0.432 during the kinematic phase because the vehicle was in
STABILIZE (mode 0). STABILIZE targets roll=0/pitch=0, generating constant
cyclic at 65 deg tilt. Fix: set ACRO before arm (wait for EKF3 active first
so DO_SET_MODE is accepted). This test is the regression gate for that fix.
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
from swashplate import ardupilot_h3_120_inverse, pwm_to_normalized

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
# No extra params needed — H_TAIL_TYPE=4 + SERVO4 range are now in rawes_sitl_defaults.parm.
_TAIL4_EXTRA_PARAMS: dict[str, float] = {}

_OBS_SECONDS    = 15.0
_TILT_LIMIT     = 0.10   # normalised [-1,1]; > this is a failure
_RATE_CMD_LIMIT = 0.05   # rad/s; ATTITUDE_TARGET rate command; > this means PID is firing


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

            max_tilt_lon   = 0.0
            max_tilt_lat   = 0.0
            max_roll_rate  = 0.0   # ATTITUDE_TARGET body_roll_rate  [rad/s]
            max_pitch_rate = 0.0   # ATTITUDE_TARGET body_pitch_rate [rad/s]
            max_yaw_rate   = 0.0   # ATTITUDE_TARGET body_yaw_rate   [rad/s]
            servo_samples  = []
            att_samples    = []

            def _handle(msg, t_rel):
                nonlocal max_tilt_lon, max_tilt_lat
                nonlocal max_roll_rate, max_pitch_rate, max_yaw_rate
                if msg is None:
                    return
                mt = msg.get_type()

                if mt == "SERVO_OUTPUT_RAW":
                    _, tilt_lat, tilt_lon = ardupilot_h3_120_inverse(
                        pwm_to_normalized(msg.servo1_raw),
                        pwm_to_normalized(msg.servo2_raw),
                        pwm_to_normalized(msg.servo3_raw),
                    )  # returns (collective_out, roll_norm, pitch_norm)
                    if abs(tilt_lon) > max_tilt_lon:
                        max_tilt_lon = abs(tilt_lon)
                    if abs(tilt_lat) > max_tilt_lat:
                        max_tilt_lat = abs(tilt_lat)
                    if len(servo_samples) % 20 == 0:
                        log.info(
                            "t=%.1fs  servo=[%d,%d,%d]  tilt_lon=%.3f  tilt_lat=%.3f",
                            t_rel,
                            msg.servo1_raw, msg.servo2_raw, msg.servo3_raw,
                            tilt_lon, tilt_lat,
                        )
                    servo_samples.append((t_rel, tilt_lon, tilt_lat))

                elif mt == "ATTITUDE_TARGET":
                    # body_roll/pitch/yaw_rate: rate commands from ArduPilot attitude
                    # controller. With neutral sticks in ACRO these must stay near zero --
                    # any nonzero value means the PID is reacting to a perceived attitude
                    # error (e.g. STABILIZE-style levelling or I-term windup).
                    rr = float(msg.body_roll_rate)
                    pr = float(msg.body_pitch_rate)
                    yr = float(msg.body_yaw_rate)
                    if abs(rr) > max_roll_rate:
                        max_roll_rate = abs(rr)
                    if abs(pr) > max_pitch_rate:
                        max_pitch_rate = abs(pr)
                    if abs(yr) > max_yaw_rate:
                        max_yaw_rate = abs(yr)
                    if len(att_samples) % 20 == 0:
                        log.info(
                            "t=%.1fs  att_target rate cmd: roll=%.3f  pitch=%.3f  yaw=%.3f rad/s",
                            t_rel, rr, pr, yr,
                        )
                    att_samples.append((t_rel, rr, pr, yr))

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
                msg = gcs._recv(
                    type=["SERVO_OUTPUT_RAW", "ATTITUDE_TARGET"],
                    blocking=True, timeout=0.2,
                )
                _handle(msg, now - t_start)

            log.info(
                "Result: max_tilt_lon=%.3f  max_tilt_lat=%.3f  (limit=%.2f)",
                max_tilt_lon, max_tilt_lat, _TILT_LIMIT,
            )
            log.info(
                "Result: max_rate_cmd  roll=%.3f  pitch=%.3f  yaw=%.3f  (limit=%.2f rad/s)",
                max_roll_rate, max_pitch_rate, max_yaw_rate, _RATE_CMD_LIMIT,
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
            if max_roll_rate > _RATE_CMD_LIMIT:
                failures.append(
                    f"max_roll_rate_cmd={max_roll_rate:.3f} rad/s > {_RATE_CMD_LIMIT} "
                    f"-- attitude PID commanding roll rate (STABILIZE contamination or I-term windup?)"
                )
            if max_pitch_rate > _RATE_CMD_LIMIT:
                failures.append(
                    f"max_pitch_rate_cmd={max_pitch_rate:.3f} rad/s > {_RATE_CMD_LIMIT} "
                    f"-- attitude PID commanding pitch rate (STABILIZE contamination or I-term windup?)"
                )
            assert not failures, "\n".join(failures)

        finally:
            try:
                gcs.close()
            except Exception:
                pass
