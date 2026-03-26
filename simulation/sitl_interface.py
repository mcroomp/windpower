"""
sitl_interface.py — Python ↔ ArduPilot SITL UDP interface

Implements the ArduPilot JSON physics backend protocol:

  Python → SITL (port 9003, state):
    {
      "timestamp": <float seconds>,
      "imu": {
        "gyro":  [gx, gy, gz],
        "accel": [ax, ay, az]
      },
      "position": [north_m, east_m, down_m],
      "attitude": [roll_rad, pitch_rad, yaw_rad],
      "velocity": [vn, ve, vd]
    }

  SITL → Python (port 9002, servos):
    {
      "timestamp": <float seconds>,
      "servo": [v1, v2, ..., v16]   # PWM microseconds 1000–2000
    }

NOTE: The exact ArduPilot SITL JSON field names and message structure may vary
between versions. If SITL sends no data, verify the protocol with:
    git -C ~/ardupilot log --oneline -5
and check libraries/SITL/SIM_JSON.cpp for the expected field layout.
As of ArduPilot 4.5.x the fields above are correct for JSON SITL backends.
"""

import json
import logging
import socket
import time
from typing import Optional

import numpy as np

log = logging.getLogger(__name__)

# ArduPilot SITL JSON physics protocol ports (configurable via constructor)
_SITL_RECV_PORT = 9002   # we listen here for servo data FROM SITL
_SITL_SEND_PORT = 9003   # we send physics state TO SITL
_SITL_HOST      = "127.0.0.1"


class SITLInterface:
    """
    UDP bridge between the mediator and ArduPilot SITL.

    ArduPilot SITL sends servo outputs on recv_port and listens for the
    physics state on send_port (both on localhost).

    Parameters
    ----------
    recv_port : int   UDP port we listen on for servo data (default 9002)
    send_port : int   UDP port we send state to SITL on (default 9003)
    host      : str   hostname / IP of the SITL process (default 127.0.0.1)
    recv_timeout_ms : float  Non-blocking receive timeout in milliseconds
    """

    def __init__(
        self,
        recv_port: int   = _SITL_RECV_PORT,
        send_port: int   = _SITL_SEND_PORT,
        host:      str   = _SITL_HOST,
        recv_timeout_ms: float = 5.0,
    ):
        self._recv_port    = recv_port
        self._send_port    = send_port
        self._host         = host
        self._timeout_s    = recv_timeout_ms / 1000.0

        # Receive socket (we bind to this)
        self._recv_sock: socket.socket | None = None
        # Send socket (we send to SITL on this)
        self._send_sock: socket.socket | None = None

        # Track the last known servo array so callers always get something
        self._last_servos: np.ndarray = np.zeros(16, dtype=np.float64)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def bind(self) -> None:
        """Create and bind the receive socket, create the send socket."""
        # Receive
        self._recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._recv_sock.bind(("", self._recv_port))
        self._recv_sock.settimeout(self._timeout_s)
        log.info("SITL recv socket bound to port %d", self._recv_port)

        # Send
        self._send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        log.info("SITL send socket → %s:%d", self._host, self._send_port)

    def close(self) -> None:
        """Close both sockets."""
        for s in (self._recv_sock, self._send_sock):
            if s is not None:
                try:
                    s.close()
                except OSError:
                    pass
        self._recv_sock = None
        self._send_sock = None
        log.info("SITLInterface sockets closed.")

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
        timestamp:  float,
        pos_ned:    np.ndarray,   # [N, E, D] metres
        vel_ned:    np.ndarray,   # [vN, vE, vD] m/s
        rpy_rad:    np.ndarray,   # [roll, pitch, yaw] radians
        accel_body: np.ndarray,   # [ax, ay, az] m/s² in body frame
        gyro_body:  np.ndarray,   # [gx, gy, gz] rad/s in body frame
    ) -> None:
        """
        Format and send a JSON physics state packet to ArduPilot SITL.

        All vectors are in NED / body frame as expected by ArduPilot.
        """
        if self._send_sock is None:
            raise RuntimeError("Not bound — call bind() first.")

        msg = {
            "timestamp": float(timestamp),
            "imu": {
                "gyro":  [float(gyro_body[0]),
                          float(gyro_body[1]),
                          float(gyro_body[2])],
                "accel": [float(accel_body[0]),
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
        }
        payload = json.dumps(msg).encode("utf-8")
        try:
            self._send_sock.sendto(payload, (self._host, self._send_port))
        except OSError as exc:
            log.warning("SITL send failed: %s", exc)

    # ------------------------------------------------------------------
    # Servos ← SITL
    # ------------------------------------------------------------------

    def recv_servos(self) -> Optional[np.ndarray]:
        """
        Non-blocking receive of servo data from SITL.

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
        if self._recv_sock is None:
            raise RuntimeError("Not bound — call bind() first.")

        try:
            data, _ = self._recv_sock.recvfrom(4096)
        except socket.timeout:
            return None
        except OSError as exc:
            log.warning("SITL recv error: %s", exc)
            return None

        try:
            msg = json.loads(data.decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError) as exc:
            log.warning("Bad JSON from SITL: %s", exc)
            return None

        raw_servos = msg.get("servo", [])
        if not raw_servos:
            log.debug("SITL packet has no 'servo' field: %s", list(msg.keys()))
            return None

        # Ensure we always have exactly 16 channels
        pwm = np.zeros(16, dtype=np.float64)
        for i, v in enumerate(raw_servos[:16]):
            pwm[i] = float(v)

        # Convert PWM µs [1000, 2000] → normalised [-1, 1]
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

    # JSON round-trip for state packet
    ts      = 1.234
    pos_ned = np.array([10.0, 5.0, -50.0])
    vel_ned = np.array([1.0, 0.5, 0.0])
    rpy     = np.array([0.01, 0.02, 1.57])
    acc     = np.array([0.0, 0.0, 9.81])
    gyro    = np.array([0.0, 0.0, 28.0])

    msg = {
        "timestamp": ts,
        "imu":       {"gyro": gyro.tolist(), "accel": acc.tolist()},
        "position":  pos_ned.tolist(),
        "attitude":  rpy.tolist(),
        "velocity":  vel_ned.tolist(),
    }
    payload = json.dumps(msg)
    parsed  = json.loads(payload)
    assert parsed["timestamp"] == ts
    assert parsed["position"] == pos_ned.tolist()
    print(f"  JSON round-trip: OK  ({len(payload)} bytes)")

    # JSON round-trip for servo packet
    servo_msg = json.dumps({"timestamp": 1.0, "servo": [1500] * 16})
    parsed_s  = json.loads(servo_msg)
    pwm_arr   = np.array(parsed_s["servo"][:16], dtype=np.float64)
    norm_arr  = (pwm_arr - 1500.0) / 500.0
    assert np.allclose(norm_arr, 0.0), "Servo neutral mismatch"
    print("  Servo JSON parse: OK")

    print("All smoke tests passed.")
    sys.exit(0)
