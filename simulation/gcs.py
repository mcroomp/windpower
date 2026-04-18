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

import collections
import logging
import threading
import time
from pathlib import Path

from pymavlink import mavutil

from mavlink_log import MavlinkLogWriter

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


class SimClock:
    """
    Simulation-time clock driven by ArduPilot MAVLink messages.

    **Sim time vs wall-clock time**

    ArduPilot SITL uses a lockstep physics protocol: the flight-controller
    binary sends a servo packet and then blocks until the physics backend
    replies with a state packet.  Because of this lockstep, ArduPilot's
    internal clock (``time_boot_ms``) advances only when the physics loop is
    running — it is *not* a real-time clock.

    - At SITL speedup=1 (default), sim time ≈ wall-clock time (roughly 1:1),
      but the two are never guaranteed to be equal.
    - Every received MAVLink message carries ``time_boot_ms``; this class
      advances monotonically from those timestamps so callers using
      ``sim_now()`` stay synchronised with ArduPilot's internal time rather
      than with wall-clock.
    - **All messages advance the clock — including discarded ones.**  When
      ``_recv(type="ATTITUDE")`` is waiting and a HEARTBEAT arrives first,
      the clock is updated from the HEARTBEAT's ``time_boot_ms`` before the
      HEARTBEAT is thrown away.  The type filter controls what is *returned*
      to the caller; it does not affect clock updates.

    No lock is needed: only the main test thread reads and writes this clock
    (the heartbeat thread only *sends* MAVLink messages, never receives them).
    """

    def __init__(self) -> None:
        self._t_ms: int = 0

    def update(self, time_boot_ms: int) -> None:
        """Advance the clock to *time_boot_ms*.

        Messages with time_boot_ms=0 are silently skipped (not all message
        types carry a meaningful timestamp).  Any non-zero timestamp that is
        less than the current clock value is a bug — messages must arrive in
        non-decreasing order.
        """
        if time_boot_ms == 0:
            return
        assert time_boot_ms >= self._t_ms, (
            f"sim clock went backward: {time_boot_ms} ms < {self._t_ms} ms"
        )
        self._t_ms = time_boot_ms

    def now(self) -> float:
        """
        Current simulation time in seconds (ArduPilot ``time_boot_ms / 1000``).

        Returns 0.0 before the first MAVLink message is received.
        """
        return self._t_ms / 1000.0

    def now_ms(self) -> int:
        """Current simulation time in milliseconds (raw ``time_boot_ms``)."""
        return self._t_ms


class WallClock:
    """
    Wall-clock drop-in for SimClock — for use outside SITL (e.g. hardware calibration).

    Uses ``time.monotonic()`` so all deadline arithmetic in RawesGCS works correctly
    on a real serial connection where ``time_boot_ms`` starts at an arbitrary uptime
    value rather than near zero.
    """

    def __init__(self) -> None:
        self._t0 = time.monotonic()

    def update(self, time_boot_ms: int) -> None:  # noqa: ARG002
        pass  # wall clock is self-advancing; ignore MAVLink timestamps

    def now(self) -> float:
        return time.monotonic() - self._t0

    def now_ms(self) -> int:
        return int((time.monotonic() - self._t0) * 1000)


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
    baud : int
        Baud rate for serial connections (ignored for UDP).  Default 115200.
    clock : SimClock | WallClock | None
        Clock to use for all deadline arithmetic.  Pass a ``WallClock()``
        instance when connecting to real hardware over USB/serial; omit (or
        pass None) for SITL where the sim clock is driven by ``time_boot_ms``.
    """

    def __init__(
        self,
        address: str = "udpin:localhost:14550",
        source_system: int = 255,
        mavlog_path: "str | Path | None" = None,
        baud: int = 115200,
        clock: "SimClock | WallClock | None" = None,
    ):
        self._address = address
        self._source_system = source_system
        self._baud = baud
        self._mav = None
        self._target_system = 1
        self._target_component = 1
        self._hb_thread: threading.Thread | None = None
        self._hb_stop = threading.Event()
        self._sim_clock = clock if clock is not None else SimClock()
        # Internal receive buffer — messages drained from the network socket but
        # not yet returned to a caller.  Populated by _recv; popped in FIFO order.
        # Clock is advanced only when a message is popped, so sim_now() stays
        # consistent with the message that caused _recv to return.
        self._recv_buf: collections.deque = collections.deque()
        # JSON message log — every received MAVLink message written as one line
        self._mavlog: MavlinkLogWriter | None = None
        if mavlog_path is not None:
            self._mavlog = MavlinkLogWriter.open(mavlog_path)

    # ------------------------------------------------------------------
    # Simulation time
    # ------------------------------------------------------------------

    def sim_now(self) -> float:
        """
        Current ArduPilot simulation time in seconds.

        Derived from the ``time_boot_ms`` field of the most recently received
        MAVLink message.  This is ArduPilot's internal clock — it advances in
        lockstep with the physics backend, not with wall-clock time.

        At SITL speedup=1 (default) sim time ≈ wall time, but they are never
        guaranteed equal.  Returns 0.0 before the first message is received.
        """
        return self._sim_clock.now()

    def sim_sleep(self, duration_s: float, check=None) -> None:
        """
        Block until *duration_s* of **simulation** time has elapsed.

        **How it works (lockstep-aware)**

        ``sim_sleep`` does *not* call ``time.sleep``.  Instead it pumps
        ``_recv(blocking=True, timeout=0.1)`` in a tight loop.  Each call
        waits up to 0.1 wall-clock seconds for the next MAVLink message;
        when one arrives its ``time_boot_ms`` advances the sim-clock.  The
        loop exits once the sim-clock has advanced by *duration_s*.

        Because ArduPilot SITL uses a lockstep physics protocol (the
        flight-controller binary blocks until the physics backend replies),
        the physics worker must keep running and replying while ``sim_sleep``
        is active — otherwise ArduPilot stalls and the sim-clock never
        advances.  ``sim_sleep`` itself is purely a *consumer* of MAVLink
        messages; it never sends physics state.

        All STATUSTEXT, EKF_STATUS_REPORT, GPS_RAW_INT, and other messages
        that arrive during the wait are consumed by ``_recv()`` and written
        to the MAVLink log.  This means tests can inspect ``gcs_log``
        (the JSON MAVLink log) after ``sim_sleep`` to check what happened.

        **Sim time vs wall time**

        At SITL speedup=1 (default), 1 sim-second takes approximately 1
        wall-clock second, so ``sim_sleep(70)`` takes ~70 s in real time.
        The two clocks are not guaranteed equal — use sim time for all
        test deadlines and assertions.

        **Lockstep anti-pattern**

        Do NOT use ``sim_now()`` as a rate-limiting clock inside a physics
        worker.  If the worker skips replying to servo packets in order to
        "wait for sim time to advance", ArduPilot will block waiting for a
        reply, ``time_boot_ms`` will stop advancing, ``sim_now()`` will
        never cross the threshold, and the test will deadlock.

        Parameters
        ----------
        duration_s : float
            Number of simulation seconds to wait.
        check : callable | None
            Zero-argument callable invoked after each ~0.1 s recv poll.
            Use it to detect process crashes or other failure conditions.
            Exceptions raised by *check* propagate immediately.
        """
        deadline = self._sim_clock.now() + duration_s
        while self._sim_clock.now() < deadline:
            self._recv(blocking=True, timeout=0.1)
            if check is not None:
                check()

    def _recv(
        self,
        type=None,
        blocking: bool = True,
        timeout: float = 1.0,
    ):
        """Receive the next matching MAVLink message, advancing the sim-clock.

        Drains all available bytes from the network socket into an internal
        deque, then pops messages from that deque one at a time.  For each
        popped message the clock is advanced and the message is logged; if it
        matches *type* it is returned immediately.  Any messages still in the
        deque are left for the next call, so ``sim_now()`` is always
        consistent with the message that caused this call to return.

        The *timeout* parameter is a **wall-clock** deadline (``time.monotonic``),
        not a sim-time deadline.  All test logic that should be tied to sim
        time should use ``sim_now()`` + ``sim_sleep()`` instead.

        Parameters
        ----------
        type : str | list[str] | None
            Message type(s) to accept.  None accepts any type.
        blocking : bool
            If True, keep reading until a match is found or *timeout* expires.
        timeout : float
            Maximum wall-clock seconds to wait (only used when blocking=True).
        """
        type_set = None
        if type is not None:
            type_set = set(type) if isinstance(type, list) else {type}
        deadline = time.monotonic() + (timeout or 0.0)
        while True:
            # Drain all currently available messages from the network into the
            # internal buffer without blocking.
            while True:
                msg = self._mav.recv_match(blocking=False)
                if msg is None:
                    break
                self._recv_buf.append(msg)

            # Process the buffer in arrival order.  Clock and log are updated
            # as each message is popped; unprocessed messages stay for next call.
            while self._recv_buf:
                msg = self._recv_buf.popleft()
                self._sim_clock.update(getattr(msg, 'time_boot_ms', 0))
                if self._mavlog is not None:
                    self._mavlog.write(msg, self._sim_clock.now_ms())
                if type_set is None or msg.get_type() in type_set:
                    return msg
                # Non-matching: clock updated + logged; discard and continue.

            # Buffer exhausted.
            if not blocking or time.monotonic() >= deadline:
                return None
            time.sleep(0.002)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def connect(self, timeout: float = 30.0) -> None:
        """Connect and wait for the first heartbeat from the vehicle.

        Uses wall-clock time: no MAVLink messages have been received yet,
        so the sim-clock is unavailable.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                self._mav = mavutil.mavlink_connection(
                    self._address,
                    baud=self._baud,
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
                time.sleep(0.5)
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
        if self._mavlog is not None:
            self._mavlog.close()
            self._mavlog = None

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

            deadline = self.sim_now() + timeout
            while self.sim_now() < deadline:
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
            verify_deadline = self.sim_now() + 2.0
            while self.sim_now() < verify_deadline:
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
        deadline = self.sim_now() + timeout
        while self.sim_now() < deadline:
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
        deadline = self.sim_now() + timeout
        while self.sim_now() < deadline:
            msg = self._recv(
                type=["ATTITUDE", "STATUSTEXT", "EKF_STATUS_REPORT"],
                blocking=True, timeout=1.0,
            )
            if msg is None:
                continue
            mt = msg.get_type()
            if mt == "STATUSTEXT":
                log.info("[t=%.1f] EKF wait STATUSTEXT: %s",
                         self.sim_now(), msg.text.rstrip("\x00").strip())
            elif mt == "EKF_STATUS_REPORT":
                log.debug("EKF_STATUS flags=0x%04x", msg.flags)
            elif mt == "ATTITUDE":
                import math as _math
                r = _math.degrees(msg.roll)
                p = _math.degrees(msg.pitch)
                y = _math.degrees(msg.yaw)
                if all(_math.isfinite(v) for v in (r, p, y)):
                    log.info("[t=%.1f] EKF attitude ready rpy=(%.1f, %.1f, %.1f)deg",
                             self.sim_now(), r, p, y)
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
        deadline = self.sim_now() + timeout
        while self.sim_now() < deadline:
            msg = self._recv(
                type="EKF_STATUS_REPORT", blocking=True, timeout=2.0
            )
            if msg is None:
                continue
            if (msg.flags & NEEDED) == NEEDED:
                log.info("[t=%.1f] EKF healthy (flags=0x%04x)", self.sim_now(), msg.flags)
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
        deadline        = self.sim_now() + timeout
        t_last_override = self.sim_now()
        t_last_arm_send = self.sim_now()
        _poll = 0.5
        while self.sim_now() < deadline:
            # Refresh RC override every 0.5 s sim-time (ArduPilot expiry is ~1 s).
            if rc_override and (self.sim_now() - t_last_override) >= _poll:
                self.send_rc_override(rc_override)
                t_last_override = self.sim_now()

            msg = self._recv(
                type=["HEARTBEAT", "COMMAND_ACK", "STATUSTEXT", "ATTITUDE"],
                blocking=True, timeout=max(_poll, 0.005),
            )
            if msg is None:
                continue

            if msg.get_type() == "STATUSTEXT":
                log.warning("[t=%.1f] STATUSTEXT during arm: %s",
                            self.sim_now(), msg.text.rstrip("\x00").strip())
                continue

            if msg.get_type() == "COMMAND_ACK":
                if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        log.info("[t=%.1f] Arm command ACCEPTED — waiting for armed heartbeat ...",
                                 self.sim_now())
                    elif msg.result in (
                        mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED,
                        mavutil.mavlink.MAV_RESULT_FAILED,
                    ):
                        # Transient pre-arm failure (e.g. interlock not yet registered,
                        # EKF still converging).  ArduPilot returns FAILED (=4) for
                        # pre-arm check failures, TEMPORARILY_REJECTED (=1) for busy.
                        # Sleep 1 s sim-time then resend; do NOT raise.
                        log.info(
                            "[t=%.1f] Arm rejected (result=%d) — retrying",
                            self.sim_now(), msg.result,
                        )
                        self.sim_sleep(1.0)
                        self._mav.mav.command_long_send(
                            self._target_system,
                            self._target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                            0, 1, param2, 0, 0, 0, 0, 0,
                        )
                        t_last_arm_send = self.sim_now()
                    else:
                        raise RuntimeError(
                            f"Arm rejected by vehicle (result={msg.result})"
                        )
                elif msg.command != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                    log.debug("COMMAND_ACK for cmd=%d result=%d (not arm)",
                              msg.command, msg.result)

            if msg.get_type() == "HEARTBEAT":
                armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                log.info("[t=%.1f] HEARTBEAT: sysid=%d base_mode=0x%02x armed=%s custom_mode=%d",
                         self.sim_now(), msg.get_srcSystem(), msg.base_mode, armed, msg.custom_mode)
                if armed:
                    log.info("[t=%.1f] Vehicle is armed.", self.sim_now())
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
        t_last_override = self.sim_now()
        t_last_send     = self.sim_now()
        self._mav.mav.command_long_send(
            self._target_system,
            self._target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            float(mode_id),
            0, 0, 0, 0, 0,
        )
        deadline = self.sim_now() + timeout
        _poll = 0.5
        while self.sim_now() < deadline:
            # Refresh RC override every 0.5 s sim-time (ArduPilot expiry is ~1 s).
            if rc_override and (self.sim_now() - t_last_override) >= _poll:
                self.send_rc_override(rc_override)
                t_last_override = self.sim_now()

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
                    log.info("[t=%.1f] Mode confirmed: %d", self.sim_now(), mode_id)
                    return
                log.debug("Heartbeat custom_mode=%d (waiting for %d)", msg.custom_mode, mode_id)
            elif t == "COMMAND_ACK":
                if msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                    if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        log.debug("MAV_CMD_DO_SET_MODE accepted, waiting for heartbeat confirmation")
                    elif msg.result == mavutil.mavlink.MAV_RESULT_FAILED:
                        # Transient rejection (e.g. EKF not yet providing position).
                        # Retry after 1 s sim-time.
                        log.debug(
                            "Mode %d rejected (result=%d) — retrying",
                            mode_id, msg.result,
                        )
                        self.sim_sleep(1.0)
                        self._mav.mav.command_long_send(
                            self._target_system,
                            self._target_component,
                            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                            0,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            float(mode_id),
                            0, 0, 0, 0, 0,
                        )
                        t_last_send = self.sim_now()
                    else:
                        raise RuntimeError(
                            f"Mode change rejected by vehicle "
                            f"(mode={mode_id}, result={msg.result})"
                        )
            elif t == "STATUSTEXT":
                log.warning("[t=%.1f] STATUSTEXT during mode set: %s",
                            self.sim_now(), msg.text.rstrip("\x00").strip())
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

    def send_named_float(self, name: str, value: float) -> None:
        """Send a NAMED_VALUE_FLOAT MAVLink message to the vehicle.

        ArduPilot Lua scripts that have called mavlink.register_rx_msgid(251)
        will receive this message via mavlink.receive_chan() on their next tick.

        Parameters
        ----------
        name : str
            Message name, up to 10 ASCII characters (truncated + null-padded).
        value : float
            Floating-point value carried by the message.
        """
        name_b = name.encode("ascii")[:10].ljust(10, b"\x00")
        self._mav.mav.named_value_float_send(
            0,       # time_boot_ms — not meaningful for GCS-to-vehicle messages
            name_b,
            float(value),
        )
        log.debug("NAMED_VALUE_FLOAT sent: %s=%.4g", name, value)

    # ------------------------------------------------------------------
    # Telemetry receive
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
