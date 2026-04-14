"""
gcs.py — MAVLink GCS client for RAWES simulation control.

Replaces a human Mission Planner operator for automated simulation scenarios.
Handles: connection, background heartbeat, parameter set, arm, mode change,
EKF health check, and GUIDED position targets.

Usage
-----
    gcs = RawesGCS()
    gcs.connect()
    gcs.start_heartbeat()
    gcs.set_param("ARMING_CHECK", 0)
    gcs.wait_ekf_ok()
    gcs.arm()
    gcs.set_mode(GUIDED)
    gcs.send_position_target_ned(north=0.0, east=35.4, down=14.6)
    pos = gcs.recv_local_position()
    gcs.close()
"""

import logging
import os
import sys
import threading
import time
from pathlib import Path

from pymavlink import mavutil

# sim_time lives in simulation/ — add it to sys.path if needed (e.g. when
# gcs.py is imported from a test subdirectory before the caller has done so).
_SIM_DIR = os.path.dirname(os.path.abspath(__file__))
if _SIM_DIR not in sys.path:
    sys.path.insert(0, _SIM_DIR)
from sim_time import wall_s, sim_sleep, record_sim_step

log = logging.getLogger(__name__)

# ArduCopter custom mode numbers
STABILIZE  = 0
ACRO       = 1
ALT_HOLD   = 2
AUTO       = 3
GUIDED     = 4
LOITER     = 5

# SET_POSITION_TARGET_LOCAL_NED type_mask: ignore everything except x, y, z
# (bits set = ignore that field)
_POS_ONLY_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)


class RawesGCS:
    """
    Minimal MAVLink GCS client for RAWES SITL control.

    Parameters
    ----------
    address : str
        pymavlink connection string.  Default connects to the SITL MAVLink
        output port (udpin = we listen, SITL sends).
    source_system : int
        MAVLink system ID to use for outgoing messages (GCS = 255).
    """

    def __init__(
        self,
        address: str = "udpin:localhost:14550",
        source_system: int = 255,
    ):
        self._address = address
        self._source_system = source_system
        self._mav = None
        self._target_system = 1
        self._target_component = 1
        self._hb_thread: threading.Thread | None = None
        self._hb_stop = threading.Event()
        # Speed estimator tracking state — updated by _track_msg_timing()
        self._last_boot_ms:   float | None = None
        self._last_boot_wall: float | None = None
        # Message hooks — callables invoked for every received MAVLink message.
        # Use add_message_hook() to register; hooks are called in recv order.
        self._msg_hooks: list = []

    # ------------------------------------------------------------------
    # Speed estimation from MAVLink message timing
    # ------------------------------------------------------------------

    def _track_msg_timing(self, msg) -> None:
        """Update the adaptive speed estimator from a timestamped MAVLink message.

        Any message that carries time_boot_ms (e.g. ATTITUDE, LOCAL_POSITION_NED)
        lets us compute: sim_dt = delta_time_boot_ms / 1000, wall_dt = elapsed
        wall-clock.  This is the GCS-side equivalent of the mediator's
        recv_servos() click timing — both feed record_sim_step() so wall_s()
        and sim_sleep() automatically reflect the actual simulation speed.

        HEARTBEAT does not carry time_boot_ms and is silently skipped.
        """
        boot_ms = getattr(msg, "time_boot_ms", None)
        if boot_ms is None or boot_ms == 0:
            return
        now = time.monotonic()
        if self._last_boot_ms is not None and self._last_boot_wall is not None:
            sim_dt  = (boot_ms - self._last_boot_ms) / 1000.0
            wall_dt = now - self._last_boot_wall
            if sim_dt > 0 and wall_dt > 0:
                record_sim_step(wall_dt, sim_dt)
        self._last_boot_ms   = boot_ms
        self._last_boot_wall = now

    def add_message_hook(self, callback) -> None:
        """Register a callable invoked for every MAVLink message received.

        The callback is called as ``callback(msg)`` from the thread that calls
        _recv().  Use a thread-safe queue inside the callback if the hook needs
        to hand messages to another thread.
        """
        self._msg_hooks.append(callback)

    def _recv(self, **kwargs):
        """Thin wrapper around ``self._mav.recv_match`` that fires message hooks."""
        msg = self._mav.recv_match(**kwargs)
        if msg is not None:
            self._track_msg_timing(msg)
            for hook in self._msg_hooks:
                try:
                    hook(msg)
                except Exception:
                    pass
        return msg

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def connect(self, timeout: float = 30.0) -> None:
        """Connect and wait for the first heartbeat from the vehicle."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                self._mav = mavutil.mavlink_connection(
                    self._address,
                    source_system=self._source_system,
                )
                hb = self._mav.wait_heartbeat(timeout=5.0)
                if hb:
                    self._target_system    = self._mav.target_system
                    self._target_component = self._mav.target_component
                    log.info(
                        "GCS connected — vehicle sys=%d comp=%d",
                        self._target_system, self._target_component,
                    )
                    return
            except Exception as exc:
                log.debug("Connect attempt failed: %s", exc)
                sim_sleep(0.5)
        raise TimeoutError(
            f"Could not connect to vehicle at {self._address!r} within {timeout:.0f}s"
        )

    def close(self) -> None:
        """Stop heartbeat thread and close socket."""
        self.stop_heartbeat()
        if self._mav is not None:
            try:
                self._mav.close()
            except Exception:
                pass
            self._mav = None

    # ------------------------------------------------------------------
    # Heartbeat
    # ------------------------------------------------------------------

    def start_heartbeat(self, rate_hz: float = 1.0) -> None:
        """
        Start a background thread that sends GCS heartbeats.

        SITL requires periodic GCS heartbeats to stay in non-failsafe states.
        Must be called after connect().
        """
        self._hb_stop.clear()
        self._hb_thread = threading.Thread(
            target=self._heartbeat_worker,
            args=(rate_hz,),
            daemon=True,
            name="gcs-heartbeat",
        )
        self._hb_thread.start()
        log.debug("GCS heartbeat thread started at %.1f Hz", rate_hz)

    def stop_heartbeat(self) -> None:
        self._hb_stop.set()
        if self._hb_thread is not None:
            self._hb_thread.join(timeout=2.0)
            self._hb_thread = None

    def _heartbeat_worker(self, rate_hz: float) -> None:
        interval = 1.0 / rate_hz
        while not self._hb_stop.wait(interval):
            try:
                self._mav.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0,
                )
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------

    def set_param(
        self,
        name: str,
        value: float,
        timeout: float = 5.0,
        retries: int = 3,
    ) -> bool:
        """
        Set a parameter and confirm via PARAM_VALUE acknowledgement.

        If no ACK arrives within *timeout* seconds, verifies the current value
        with a PARAM_REQUEST_READ and retries the set up to *retries* times.
        Returns True if the parameter is confirmed at the requested value.
        """
        name_bytes = name.encode("utf-8")

        for attempt in range(retries):
            if attempt > 0:
                log.debug("set_param %s retry %d/%d", name, attempt, retries - 1)

            self._mav.mav.param_set_send(
                self._target_system,
                self._target_component,
                name_bytes,
                float(value),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            )

            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                msg = self._recv(
                    type="PARAM_VALUE", blocking=True, timeout=1.0
                )
                if msg is None:
                    continue
                pid = msg.param_id.rstrip("\x00")
                log.debug(
                    "PARAM_VALUE received: %s = %g (looking for %s)",
                    pid, msg.param_value, name,
                )
                if pid == name:
                    log.info("Param %-20s = %g", name, msg.param_value)
                    return True

            # ACK not received — read back to check if the value was applied silently
            log.debug("No ACK for %s — verifying via PARAM_REQUEST_READ", name)
            self._mav.mav.param_request_read_send(
                self._target_system,
                self._target_component,
                name_bytes,
                -1,
            )
            verify_deadline = time.monotonic() + 2.0
            while time.monotonic() < verify_deadline:
                msg = self._recv(
                    type="PARAM_VALUE", blocking=True, timeout=0.5
                )
                if msg is None:
                    continue
                pid = msg.param_id.rstrip("\x00")
                if pid == name:
                    if msg.param_value == float(value):
                        log.info(
                            "Param %-20s = %g (set confirmed via readback)",
                            name, msg.param_value,
                        )
                        return True
                    log.debug(
                        "Param %s readback = %g (wanted %g) — will retry set",
                        name, msg.param_value, value,
                    )
                    break

        log.warning("Failed to set %s = %g after %d attempts", name, value, retries)
        return False

    def get_param(self, name: str, timeout: float = 5.0) -> float | None:
        """
        Read a parameter value via PARAM_REQUEST_READ.

        Returns the float value, or None if no response within *timeout*.
        """
        name_bytes = name.encode("utf-8")
        self._mav.mav.param_request_read_send(
            self._target_system,
            self._target_component,
            name_bytes,
            -1,
        )
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self._recv(type="PARAM_VALUE", blocking=True, timeout=1.0)
            if msg is None:
                continue
            pid = msg.param_id.rstrip("\x00")
            if pid == name:
                log.debug("get_param %s = %g", name, msg.param_value)
                return float(msg.param_value)
        log.warning("get_param %s: no response within %.1f s", name, timeout)
        return None

    # ------------------------------------------------------------------
    # EKF health
    # ------------------------------------------------------------------

    def wait_ekf_attitude(self, timeout: float = 45.0) -> bool:
        """
        Block until a finite ATTITUDE message is received (EKF tilt aligned).

        This is a weaker condition than wait_ekf_ok() — it only requires
        attitude, not position.  Sufficient for force-arm in simulation.

        Returns True on success, False on timeout.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self._recv(
                type=["ATTITUDE", "STATUSTEXT", "EKF_STATUS_REPORT"],
                blocking=True, timeout=1.0,
            )
            if msg is None:
                continue
            mt = msg.get_type()
            if mt == "STATUSTEXT":
                log.info("EKF wait STATUSTEXT: %s", msg.text.rstrip("\x00").strip())
            elif mt == "EKF_STATUS_REPORT":
                log.debug("EKF_STATUS flags=0x%04x", msg.flags)
            elif mt == "ATTITUDE":
                import math as _math
                r = _math.degrees(msg.roll)
                p = _math.degrees(msg.pitch)
                y = _math.degrees(msg.yaw)
                if all(_math.isfinite(v) for v in (r, p, y)):
                    log.info("EKF attitude ready rpy=(%.1f, %.1f, %.1f)°", r, p, y)
                    return True
        return False

    def wait_ekf_ok(self, timeout: float = 60.0) -> None:
        """
        Block until the EKF reports a healthy position and velocity estimate.

        EKF_STATUS_REPORT flags checked:
          attitude, velocity_horiz, velocity_vert,
          pos_horiz_rel, pos_horiz_abs, pos_vert_abs
        """
        NEEDED = (
            mavutil.mavlink.EKF_ATTITUDE
            | mavutil.mavlink.EKF_VELOCITY_HORIZ
            | mavutil.mavlink.EKF_VELOCITY_VERT
            | mavutil.mavlink.EKF_POS_HORIZ_REL
            | mavutil.mavlink.EKF_POS_HORIZ_ABS
            | mavutil.mavlink.EKF_POS_VERT_ABS
        )
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self._recv(
                type="EKF_STATUS_REPORT", blocking=True, timeout=2.0
            )
            if msg is None:
                continue
            if (msg.flags & NEEDED) == NEEDED:
                log.info("EKF healthy (flags=0x%04x)", msg.flags)
                return
            log.debug("EKF not ready yet (flags=0x%04x)", msg.flags)
        raise TimeoutError(f"EKF not healthy after {timeout:.0f}s")

    # ------------------------------------------------------------------
    # Arm
    # ------------------------------------------------------------------

    def arm(
        self,
        timeout: float = 15.0,
        force: bool = False,
        rc_override: dict[int, int] | None = None,
    ) -> None:
        """Send arm command and confirm via HEARTBEAT armed flag.

        Parameters
        ----------
        force : bool
            Pass the ArduPilot force-arm magic number (21196) as param2 to
            bypass all remaining pre-arm safety checks.  Use in simulation
            when hardware-specific interlocks (motor interlock, RC failsafe)
            are irrelevant.
        rc_override : dict[int, int] | None
            If provided, RC_CHANNELS_OVERRIDE is re-sent every 0.5 s while
            waiting for arm confirmation, preventing ArduPilot from expiring
            the override (default expiry ~1 s).  Format: {channel: pwm, ...}
        """
        param2 = 21196.0 if force else 0.0
        log.info("Sending arm command (force=%s) …", force)
        self._mav.mav.command_long_send(
            self._target_system,
            self._target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,       # confirmation
            1,       # param1: 1 = arm
            param2, 0, 0, 0, 0, 0,
        )
        deadline = time.monotonic() + timeout
        t_last_override = time.monotonic()
        t_last_arm_send = time.monotonic()
        _poll = wall_s(0.5)   # RC refresh interval and recv timeout — scales with speedup
        while time.monotonic() < deadline:
            # Refresh RC override every wall_s(0.5) seconds.
            # ArduPilot's RC-override expiry is ~1 sim-second; refreshing at half
            # that nominal rate keeps us safe at any speedup.
            if rc_override and (time.monotonic() - t_last_override) >= _poll:
                self.send_rc_override(rc_override)
                t_last_override = time.monotonic()
                _poll = wall_s(0.5)   # re-read in case estimator updated

            msg = self._recv(
                type=["HEARTBEAT", "COMMAND_ACK", "STATUSTEXT", "ATTITUDE"],
                blocking=True, timeout=max(_poll, 0.005),
            )
            if msg is None:
                continue

            if msg.get_type() == "STATUSTEXT":
                log.warning("STATUSTEXT during arm: %s",
                            msg.text.rstrip("\x00").strip())
                continue

            if msg.get_type() == "COMMAND_ACK":
                if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        log.info("Arm command ACCEPTED — waiting for armed heartbeat ...")
                    elif msg.result in (
                        mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED,
                        mavutil.mavlink.MAV_RESULT_FAILED,
                    ):
                        # Transient pre-arm failure (e.g. interlock not yet registered,
                        # EKF still converging).  ArduPilot returns FAILED (=4) for
                        # pre-arm check failures, TEMPORARILY_REJECTED (=1) for busy.
                        # Sleep 1 s then resend; do NOT raise.
                        log.info(
                            "Arm rejected (result=%d) — retrying", msg.result
                        )
                        sim_sleep(1.0)
                        self._mav.mav.command_long_send(
                            self._target_system,
                            self._target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                            0, 1, param2, 0, 0, 0, 0, 0,
                        )
                        t_last_arm_send = time.monotonic()
                    else:
                        raise RuntimeError(
                            f"Arm rejected by vehicle (result={msg.result})"
                        )
                elif msg.command != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                    log.debug("COMMAND_ACK for cmd=%d result=%d (not arm)",
                              msg.command, msg.result)

            if msg.get_type() == "HEARTBEAT":
                armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                log.info("HEARTBEAT: sysid=%d base_mode=0x%02x armed=%s custom_mode=%d",
                         msg.get_srcSystem(), msg.base_mode, armed, msg.custom_mode)
                if armed:
                    log.info("Vehicle is armed.")
                    return

        raise TimeoutError(f"Vehicle did not confirm armed within {timeout:.0f}s")

    # ------------------------------------------------------------------
    # Mode
    # ------------------------------------------------------------------

    def set_mode(
        self,
        mode_id: int,
        timeout: float = 10.0,
        rc_override: dict[int, int] | None = None,
    ) -> None:
        """Set ArduCopter flight mode by custom mode number.

        Parameters
        ----------
        rc_override : dict[int, int] | None
            If provided, RC_CHANNELS_OVERRIDE is re-sent every 0.5 s while
            waiting for mode confirmation (prevents ArduPilot from expiring
            the override, which would disengage the motor interlock).
        """
        log.info("Setting mode %d …", mode_id)
        t_last_override = time.monotonic()
        t_last_send     = time.monotonic()
        self._mav.mav.command_long_send(
            self._target_system,
            self._target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            float(mode_id),
            0, 0, 0, 0, 0,
        )
        deadline = time.monotonic() + timeout
        _poll = wall_s(0.5)
        while time.monotonic() < deadline:
            # Refresh RC override at wall_s(0.5) intervals (see arm() for rationale)
            if rc_override and (time.monotonic() - t_last_override) >= _poll:
                self.send_rc_override(rc_override)
                t_last_override = time.monotonic()
                _poll = wall_s(0.5)

            msg = self._recv(
                type=["HEARTBEAT", "COMMAND_ACK", "STATUSTEXT", "ATTITUDE"],
                blocking=True,
                timeout=max(_poll, 0.005),
            )
            if msg is None:
                continue
            t = msg.get_type()
            if t == "HEARTBEAT":
                if msg.custom_mode == mode_id:
                    log.info("Mode confirmed: %d", mode_id)
                    return
                log.debug("Heartbeat custom_mode=%d (waiting for %d)", msg.custom_mode, mode_id)
            elif t == "COMMAND_ACK":
                if msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                    if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        log.debug("MAV_CMD_DO_SET_MODE accepted, waiting for heartbeat confirmation")
                    elif msg.result == mavutil.mavlink.MAV_RESULT_FAILED:
                        # Transient rejection (e.g. EKF not yet providing position).
                        # Retry after 1 s.
                        log.debug(
                            "Mode %d rejected (result=%d) — retrying",
                            mode_id, msg.result,
                        )
                        sim_sleep(1.0)
                        self._mav.mav.command_long_send(
                            self._target_system,
                            self._target_component,
                            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                            0,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            float(mode_id),
                            0, 0, 0, 0, 0,
                        )
                        t_last_send = time.monotonic()
                    else:
                        raise RuntimeError(
                            f"Mode change rejected by vehicle "
                            f"(mode={mode_id}, result={msg.result})"
                        )
            elif t == "STATUSTEXT":
                log.warning("STATUSTEXT during mode set: %s", msg.text.rstrip("\x00").strip())
        raise TimeoutError(f"Mode {mode_id} not confirmed within {timeout:.0f}s")

    # ------------------------------------------------------------------
    # GUIDED position target
    # ------------------------------------------------------------------

    def send_position_target_ned(
        self,
        north: float,
        east:  float,
        down:  float,
        yaw:   float = 0.0,
    ) -> None:
        """
        Send SET_POSITION_TARGET_LOCAL_NED in MAV_FRAME_LOCAL_NED.

        Coordinate origin: EKF home (where SITL was launched).
        Positive down = below home altitude.

        Parameters
        ----------
        north, east, down : float   Target position [m]
        yaw               : float   Target yaw [rad], default 0 (ignored in mask)
        """
        self._mav.mav.set_position_target_local_ned_send(
            0,                                          # time_boot_ms
            self._target_system,
            self._target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            _POS_ONLY_MASK,
            north, east, down,                          # position
            0.0, 0.0, 0.0,                              # velocity (ignored)
            0.0, 0.0, 0.0,                              # acceleration (ignored)
            yaw, 0.0,                                   # yaw, yaw_rate (ignored)
        )
        log.info(
            "Position target sent: N=%.1f E=%.1f D=%.1f m", north, east, down
        )

    # ------------------------------------------------------------------
    # Telemetry
    # ------------------------------------------------------------------

    def request_stream(self, stream_id: int, rate_hz: int) -> None:
        """
        Request a MAVLink data stream from the vehicle.

        Parameters
        ----------
        stream_id : int
            MAV_DATA_STREAM_* constant (e.g. mavutil.mavlink.MAV_DATA_STREAM_POSITION)
        rate_hz : int
            Requested message rate in Hz.  Call once; ArduPilot sustains the rate.
        """
        self._mav.mav.request_data_stream_send(
            self._target_system,
            self._target_component,
            stream_id,
            rate_hz,
            1,   # 1 = start streaming
        )
        log.debug("Requested stream id=%d at %d Hz", stream_id, rate_hz)

    def send_rc_override(self, channels: dict[int, int]) -> None:
        """
        Send RC_CHANNELS_OVERRIDE to the vehicle.

        Parameters
        ----------
        channels : dict[int, int]
            Mapping of channel number (1-indexed) to PWM value.
            Channels not listed are set to 0 (= no override).

        Example
        -------
        gcs.send_rc_override({8: 2000})   # CH8 = full high (motor interlock release)
        """
        pwm = [0] * 18
        for ch, val in channels.items():
            if 1 <= ch <= 18:
                pwm[ch - 1] = val
        self._mav.mav.rc_channels_override_send(
            self._target_system,
            self._target_component,
            *pwm[:18],
        )
        log.debug("RC override sent: %s", channels)

    # ------------------------------------------------------------------
    # Telemetry
    # ------------------------------------------------------------------

    def recv_local_position(
        self, timeout: float = 2.0
    ) -> tuple[float, float, float] | None:
        """
        Return current LOCAL_POSITION_NED (x=N, y=E, z=D) or None on timeout.
        """
        msg = self._recv(
            type="LOCAL_POSITION_NED", blocking=True, timeout=timeout
        )
        if msg:
            return (msg.x, msg.y, msg.z)
        return None

    def recv_attitude(
        self, timeout: float = 2.0
    ) -> tuple[float, float, float] | None:
        """Return (roll, pitch, yaw) radians or None on timeout."""
        msg = self._recv(
            type="ATTITUDE", blocking=True, timeout=timeout
        )
        if msg:
            return (msg.roll, msg.pitch, msg.yaw)
        return None


# ---------------------------------------------------------------------------
# EkfLogger — continuous EKF state capture on a second MAVLink connection
# ---------------------------------------------------------------------------

#: CSV columns written by EkfLogger.  One row per ATTITUDE message.
EKF_LOG_COLUMNS: list[str] = [
    "time_boot_ms",
    # ArduPilot EKF attitude estimate (ATTITUDE msg)
    "att_roll", "att_pitch", "att_yaw",
    "att_rollspeed", "att_pitchspeed", "att_yawspeed",
    # EKF position + velocity (LOCAL_POSITION_NED msg)
    "pos_n", "pos_e", "pos_d",
    "vel_n", "vel_e", "vel_d",
    # EKF health (EKF_STATUS_REPORT msg — may lag behind by one ATTITUDE interval)
    "ekf_flags",
    "ekf_vel_variance",
    "ekf_pos_horiz_variance",
    "ekf_pos_vert_variance",
    "ekf_compass_variance",
    "ekf_terrain_alt_variance",
]


class EkfLogger:
    """
    Logs EKF state to a CSV file by hooking into an existing RawesGCS connection.

    Registers itself as a message hook on the GCS so it receives every MAVLink
    message the GCS processes — no second TCP connection needed.  One row is
    written per ATTITUDE message using the most recently seen LOCAL_POSITION_NED
    and EKF_STATUS_REPORT values.

    Usage
    -----
        logger = EkfLogger()
        logger.start("logs/test_foo/ekf_telemetry.csv", gcs)
        ...test...
        logger.stop()
    """

    def __init__(self):
        import queue as _queue
        self._log_path: Path | None = None
        self._thread:   threading.Thread | None = None
        self._stop      = threading.Event()
        self._queue     = _queue.SimpleQueue()

    def start(self, log_path: "str | Path", gcs: "RawesGCS") -> None:
        """Register hook on *gcs* and start the background writer thread."""
        self._log_path = Path(log_path)
        self._stop.clear()
        gcs.add_message_hook(self._on_message)
        self._thread = threading.Thread(
            target=self._run, daemon=True, name="ekf-logger"
        )
        self._thread.start()
        log.debug("EkfLogger started -> %s", self._log_path)

    def stop(self) -> None:
        """Signal the writer thread to stop and wait for it to finish."""
        self._stop.set()
        self._queue.put(None)  # unblock the writer
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None

    def _on_message(self, msg) -> None:
        """Called from the GCS thread for every received MAVLink message."""
        mt = msg.get_type()
        if mt in ("ATTITUDE", "LOCAL_POSITION_NED", "EKF_STATUS_REPORT"):
            self._queue.put(msg)

    def _run(self) -> None:
        import csv as _csv

        assert self._log_path is not None
        self._log_path.parent.mkdir(parents=True, exist_ok=True)

        last_pos: dict = {}
        last_ekf: dict = {}

        with self._log_path.open("w", newline="", encoding="utf-8") as fh:
            writer = _csv.DictWriter(fh, fieldnames=EKF_LOG_COLUMNS)
            writer.writeheader()

            while not self._stop.is_set():
                try:
                    msg = self._queue.get(timeout=0.5)
                except Exception:
                    continue
                if msg is None:
                    break

                mt  = msg.get_type()
                tbs = int(getattr(msg, "time_boot_ms", 0))

                if mt == "LOCAL_POSITION_NED":
                    last_pos = {
                        "pos_n": msg.x,  "pos_e": msg.y,  "pos_d": msg.z,
                        "vel_n": msg.vx, "vel_e": msg.vy, "vel_d": msg.vz,
                    }

                elif mt == "EKF_STATUS_REPORT":
                    last_ekf = {
                        "ekf_flags":                msg.flags,
                        "ekf_vel_variance":         msg.velocity_variance,
                        "ekf_pos_horiz_variance":   msg.pos_horiz_variance,
                        "ekf_pos_vert_variance":    msg.pos_vert_variance,
                        "ekf_compass_variance":     msg.compass_variance,
                        "ekf_terrain_alt_variance": msg.terrain_alt_variance,
                    }

                elif mt == "ATTITUDE" and last_pos:
                    row: dict = {
                        "time_boot_ms":   tbs,
                        "att_roll":       msg.roll,
                        "att_pitch":      msg.pitch,
                        "att_yaw":        msg.yaw,
                        "att_rollspeed":  msg.rollspeed,
                        "att_pitchspeed": msg.pitchspeed,
                        "att_yawspeed":   msg.yawspeed,
                    }
                    row.update(last_pos)
                    row.update(last_ekf)
                    writer.writerow(row)
                    fh.flush()
