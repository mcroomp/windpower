"""
sitl_interface.py — Python ↔ ArduPilot SITL UDP interface

Implements the ArduPilot JSON physics backend protocol as documented in
libraries/SITL/examples/JSON/readme.md:

  SITL → Python (binary, port 9002):
    uint16  magic       (18458 for 16-ch, 29569 for 32-ch)
    uint16  frame_rate
    uint32  frame_count
    uint16  pwm[16]     (PWM microseconds, typically 1000–2000)

  Python → SITL (reply to source address, JSON + newline):
    {"timestamp":<s>,"imu":{"gyro":[gx,gy,gz],"accel_body":[ax,ay,az]},
     "position":[N,E,D],"attitude":[roll,pitch,yaw],"velocity":[vN,vE,vD]}\\n

Protocol notes:
- Physics backend binds to port 9002.  SITL sends servo packets TO that port.
- Physics backend replies to the SOURCE address of each servo packet — no
  fixed target port is needed.
- State JSON must be \\n-terminated.
- SITL resends its last servo packet every 10 s if no state is received,
  allowing the physics backend to restart and reconnect.
"""

import json
import logging
import socket
import struct
from typing import Optional

import numpy as np

log = logging.getLogger(__name__)

# ArduPilot SITL JSON physics protocol — listen port only (reply is to source)
_SITL_RECV_PORT = 9002   # we listen here for binary servo frames FROM SITL

# Binary servo packet layout (little-endian)
_SERVO_MAGIC_16  = 18458
_SERVO_MAGIC_32  = 29569
_SERVO_FMT_16    = "<HHI16H"   # magic, frame_rate, frame_count, pwm×16
_SERVO_FMT_32    = "<HHI32H"   # magic, frame_rate, frame_count, pwm×32
_SERVO_SIZE_16   = struct.calcsize(_SERVO_FMT_16)   # 40 bytes
_SERVO_SIZE_32   = struct.calcsize(_SERVO_FMT_32)   # 72 bytes


class SITLInterface:
    """
    UDP bridge between the mediator and ArduPilot SITL.

    ArduPilot SITL sends binary servo packets to recv_port (default 9002).
    The physics backend replies to the source address of each packet with a
    JSON state string.

    Parameters
    ----------
    recv_port       : int    UDP port to bind for incoming servo data (default 9002)
    recv_timeout_ms : float  Non-blocking receive timeout in milliseconds
    """

    def __init__(
        self,
        recv_port: int = _SITL_RECV_PORT,
        recv_timeout_ms: float = 5.0,
        # Legacy parameter — kept so callers that pass send_port= don't break.
        # The send address is now determined dynamically from each incoming packet.
        send_port: int = 9003,
        host: str = "127.0.0.1",
    ):
        self._recv_port = recv_port
        self._timeout_s = recv_timeout_ms / 1000.0

        self._sock: socket.socket | None = None
        self._sitl_addr: tuple[str, int] | None = None   # filled on first recv

        self._last_servos: np.ndarray = np.zeros(16, dtype=np.float64)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def bind(self) -> None:
        """Create and bind the single UDP socket."""
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("", self._recv_port))
        self._sock.settimeout(self._timeout_s)
        log.info("SITLInterface bound to port %d", self._recv_port)

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        log.info("SITLInterface socket closed.")

    def __enter__(self):
        self.bind()
        return self

    def __exit__(self, *_):
        self.close()

    # ------------------------------------------------------------------
    # State → SITL
    # ------------------------------------------------------------------

    def send_state(
        self,
        timestamp:   float,
        pos_ned:     np.ndarray,   # [N, E, D] metres
        vel_ned:     np.ndarray,   # [vN, vE, vD] m/s
        rpy_rad:     np.ndarray,   # [roll, pitch, yaw] radians
        accel_body:  np.ndarray,   # [ax, ay, az] m/s² in body frame
        gyro_body:   np.ndarray,   # [gx, gy, gz] rad/s in body frame
        rpm_rad_s:   float = 0.0,  # rotor spin rate [rad/s] → sent as RPM for RPM1_TYPE=10
    ) -> None:
        """
        Send a JSON physics state packet back to ArduPilot SITL.

        Sends to the source address of the last received servo packet.
        Does nothing if no servo packet has been received yet.

        The ``rpm`` field is read by ArduPilot when RPM1_TYPE=10 (SITL RPM).
        ArduPilot uses the RPM sensor to determine whether the rotor has
        completed its runup sequence before allowing GUIDED/AUTO modes.
        """
        if self._sock is None:
            raise RuntimeError("Not bound — call bind() first.")
        if self._sitl_addr is None:
            return   # SITL hasn't sent anything yet; nothing to reply to

        rpm1 = rpm_rad_s * (60.0 / (2.0 * 3.141592653589793))

        msg = {
            "timestamp": float(timestamp),
            "imu": {
                "gyro":       [float(gyro_body[0]),
                               float(gyro_body[1]),
                               float(gyro_body[2])],
                "accel_body": [float(accel_body[0]),
                               float(accel_body[1]),
                               float(accel_body[2])],
            },
            "position": [float(pos_ned[0]),
                         float(pos_ned[1]),
                         float(pos_ned[2])],
            "attitude": [float(rpy_rad[0]),
                         float(rpy_rad[1]),
                         float(rpy_rad[2])],
            "velocity": [float(vel_ned[0]),
                         float(vel_ned[1]),
                         float(vel_ned[2])],
            "rpm":       {"rpm_1": rpm1, "rpm_2": 0.0, "rpm_3": 0.0, "rpm_4": 0.0},
        }
        payload = (json.dumps(msg) + "\n").encode("utf-8")
        try:
            self._sock.sendto(payload, self._sitl_addr)
        except OSError as exc:
            log.warning("SITL send failed: %s", exc)

    # ------------------------------------------------------------------
    # Servos ← SITL
    # ------------------------------------------------------------------

    def recv_servos(self) -> Optional[np.ndarray]:
        """
        Non-blocking receive of binary servo data from SITL.

        Returns
        -------
        np.ndarray, shape (16,), dtype float64
            Servo values normalised to [-1, 1].
            Returns None if no packet arrived within recv_timeout_ms.

        PWM → normalised mapping: (pwm_us - 1500) / 500
            1000 µs → -1.0
            1500 µs →  0.0
            2000 µs → +1.0
        """
        if self._sock is None:
            raise RuntimeError("Not bound — call bind() first.")

        try:
            data, addr = self._sock.recvfrom(4096)
        except socket.timeout:
            return None
        except OSError as exc:
            log.warning("SITL recv error: %s", exc)
            return None

        if len(data) == _SERVO_SIZE_16:
            fields = struct.unpack(_SERVO_FMT_16, data)
            magic, frame_rate, frame_count = fields[0], fields[1], fields[2]
            pwm_raw = fields[3:]
            if magic != _SERVO_MAGIC_16:
                log.warning("Unexpected servo packet magic 0x%04x (expected 0x%04x)", magic, _SERVO_MAGIC_16)
                return None
        elif len(data) == _SERVO_SIZE_32:
            fields = struct.unpack(_SERVO_FMT_32, data)
            magic, frame_rate, frame_count = fields[0], fields[1], fields[2]
            pwm_raw = fields[3:19]   # first 16 channels only
            if magic != _SERVO_MAGIC_32:
                log.warning("Unexpected servo packet magic 0x%04x (expected 0x%04x)", magic, _SERVO_MAGIC_32)
                return None
        else:
            log.warning("Unexpected servo packet size %d bytes (expected %d or %d)",
                        len(data), _SERVO_SIZE_16, _SERVO_SIZE_32)
            return None

        # Record SITL's address so send_state() knows where to reply
        self._sitl_addr = addr

        pwm = np.array(pwm_raw[:16], dtype=np.float64)
        normalised = (pwm - 1500.0) / 500.0
        normalised = np.clip(normalised, -1.0, 1.0)
        self._last_servos = normalised
        return normalised

    def last_servos(self) -> np.ndarray:
        """Return the most recently received servo array (never None)."""
        return self._last_servos.copy()


# ---------------------------------------------------------------------------
# Standalone smoke test
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import sys

    logging.basicConfig(level=logging.DEBUG, format="%(levelname)s %(message)s")

    print("SITLInterface smoke test")

    # PWM normalisation
    pwm_vals = np.array([1000, 1500, 2000, 1250, 1750], dtype=np.float64)
    norm = (pwm_vals - 1500.0) / 500.0
    expected = np.array([-1.0, 0.0, 1.0, -0.5, 0.5])
    assert np.allclose(norm, expected), f"Normalisation failed: {norm}"
    print(f"  PWM normalisation: OK  {pwm_vals} → {norm}")

    # Binary servo packet round-trip (16-channel)
    frame_rate, frame_count = 400, 42
    pwm_in = [1500] * 16
    packed = struct.pack(_SERVO_FMT_16, _SERVO_MAGIC_16, frame_rate, frame_count, *pwm_in)
    assert len(packed) == _SERVO_SIZE_16
    fields = struct.unpack(_SERVO_FMT_16, packed)
    assert fields[0] == _SERVO_MAGIC_16
    assert fields[1] == frame_rate
    assert fields[2] == frame_count
    assert list(fields[3:]) == pwm_in
    print(f"  Binary servo packet round-trip: OK  ({len(packed)} bytes)")

    # JSON state packet format (with newline terminator)
    ts      = 1.234
    pos_ned = np.array([10.0, 5.0, -50.0])
    vel_ned = np.array([1.0, 0.5, 0.0])
    rpy     = np.array([0.01, 0.02, 1.57])
    acc     = np.array([0.0, 0.0, 9.81])
    gyro    = np.array([0.0, 0.0, 28.0])

    msg = {
        "timestamp": ts,
        "imu":       {"gyro": gyro.tolist(), "accel_body": acc.tolist()},
        "position":  pos_ned.tolist(),
        "attitude":  rpy.tolist(),
        "velocity":  vel_ned.tolist(),
    }
    payload = json.dumps(msg) + "\n"
    assert payload.endswith("\n")
    parsed = json.loads(payload.strip())
    assert parsed["timestamp"] == ts
    assert parsed["position"] == pos_ned.tolist()
    assert "accel_body" in parsed["imu"], "Must use 'accel_body' key, not 'accel'"
    print(f"  JSON state packet: OK  ({len(payload)} bytes, newline-terminated)")

    print("All smoke tests passed.")
    sys.exit(0)
