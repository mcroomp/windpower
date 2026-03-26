"""
mbdyn_interface.py — Python ↔ MBDyn UNIX socket interface

Handles bidirectional communication with a running MBDyn process via two
UNIX domain sockets:
  /tmp/rawes_forces.sock  — Python sends  6 float64: [Fx Fy Fz Mx My Mz]
  /tmp/rawes_state.sock   — Python reads 18 float64: [pos vel R omega]

MBDyn creates both socket files (create, yes) and acts as server.
This class connects as a client with a retry loop (up to 30 s).

Synchronisation protocol (each simulation step):
  1. Python calls send_forces()   → MBDyn reads, integrates one step
  2. MBDyn writes state to socket
  3. Python calls recv_state()    → reads the 18 values
  Repeat.
"""

import socket
import struct
import time
import logging
import numpy as np
from pathlib import Path

log = logging.getLogger(__name__)

# Binary layout: all values are native-endian float64
_FORCE_FMT  = "6d"   # 6 × float64 = 48 bytes
_STATE_FMT  = "18d"  # 18 × float64 = 144 bytes
_FORCE_SIZE = struct.calcsize(_FORCE_FMT)
_STATE_SIZE = struct.calcsize(_STATE_FMT)


def _wait_for_socket(path: str, timeout: float = 30.0) -> None:
    """Poll until the socket file appears or timeout is reached."""
    deadline = time.monotonic() + timeout
    p = Path(path)
    log.info("Waiting for socket %s (timeout %.0fs) ...", path, timeout)
    while time.monotonic() < deadline:
        if p.exists():
            log.info("Socket %s found.", path)
            return
        time.sleep(0.2)
    raise TimeoutError(
        f"MBDyn socket {path!r} did not appear within {timeout:.0f} s. "
        "Is MBDyn running and using 'create, yes' for this socket?"
    )


def _connect_unix(path: str) -> socket.socket:
    """Create a connected UNIX domain socket (stream)."""
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.connect(path)
    sock.setblocking(True)
    log.info("Connected to %s", path)
    return sock


def _recv_exactly(sock: socket.socket, n_bytes: int) -> bytes:
    """Read exactly n_bytes from sock, handling partial reads."""
    buf = bytearray()
    while len(buf) < n_bytes:
        chunk = sock.recv(n_bytes - len(buf))
        if not chunk:
            raise ConnectionResetError(
                f"MBDyn socket closed unexpectedly "
                f"(received {len(buf)}/{n_bytes} bytes)"
            )
        buf.extend(chunk)
    return bytes(buf)


class MBDynInterface:
    """
    Manages the Python side of the MBDyn co-simulation sockets.

    Parameters
    ----------
    force_sock_path : str
        Path to the UNIX socket used to *send* forces to MBDyn.
        Default: /tmp/rawes_forces.sock
    state_sock_path : str
        Path to the UNIX socket used to *receive* state from MBDyn.
        Default: /tmp/rawes_state.sock
    connect_timeout : float
        Seconds to wait for MBDyn to create the socket files.
    """

    def __init__(
        self,
        force_sock_path: str = "/tmp/rawes_forces.sock",
        state_sock_path: str = "/tmp/rawes_state.sock",
        connect_timeout: float = 30.0,
    ):
        self._force_path = force_sock_path
        self._state_path = state_sock_path
        self._timeout    = connect_timeout
        self._force_sock: socket.socket | None = None
        self._state_sock: socket.socket | None = None

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------

    def connect(self) -> None:
        """
        Wait for MBDyn to create both socket files, then connect.
        Raises TimeoutError if sockets do not appear within connect_timeout.
        """
        _wait_for_socket(self._force_path, self._timeout)
        _wait_for_socket(self._state_path, self._timeout)
        self._force_sock = _connect_unix(self._force_path)
        self._state_sock = _connect_unix(self._state_path)
        log.info(
            "MBDynInterface connected. Force socket: %s  State socket: %s",
            self._force_path,
            self._state_path,
        )

    def close(self) -> None:
        """Close both sockets."""
        for sock in (self._force_sock, self._state_sock):
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
        self._force_sock = None
        self._state_sock = None
        log.info("MBDynInterface sockets closed.")

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *_):
        self.close()

    # ------------------------------------------------------------------
    # Data exchange
    # ------------------------------------------------------------------

    def send_forces(self, forces: np.ndarray) -> None:
        """
        Send 6 float64 values to MBDyn representing the aerodynamic wrench
        in the world (ENU) frame:

            forces[0] = Fx  [N]   — East
            forces[1] = Fy  [N]   — North
            forces[2] = Fz  [N]   — Up
            forces[3] = Mx  [N·m] — moment about East axis
            forces[4] = My  [N·m] — moment about North axis
            forces[5] = Mz  [N·m] — moment about Up axis

        Parameters
        ----------
        forces : array_like, shape (6,)
        """
        if self._force_sock is None:
            raise RuntimeError("Not connected — call connect() first.")
        arr = np.asarray(forces, dtype=np.float64).ravel()
        if arr.size != 6:
            raise ValueError(f"forces must have 6 elements, got {arr.size}")
        data = struct.pack(_FORCE_FMT, *arr)
        self._force_sock.sendall(data)

    def recv_state(self) -> dict:
        """
        Receive 18 float64 values from MBDyn and unpack them into a state dict.

        Returns
        -------
        dict with keys:
            pos   : np.ndarray (3,)   — hub position in ENU [m]
            vel   : np.ndarray (3,)   — hub velocity in ENU [m/s]
            R     : np.ndarray (3,3)  — hub rotation matrix (body→world)
            omega : np.ndarray (3,)   — hub angular velocity in world frame [rad/s]
        """
        if self._state_sock is None:
            raise RuntimeError("Not connected — call connect() first.")

        raw = _recv_exactly(self._state_sock, _STATE_SIZE)
        vals = struct.unpack(_STATE_FMT, raw)

        pos   = np.array(vals[0:3],   dtype=np.float64)
        vel   = np.array(vals[3:6],   dtype=np.float64)
        # MBDyn sends rotation matrix in row-major order
        R     = np.array(vals[6:15],  dtype=np.float64).reshape(3, 3)
        omega = np.array(vals[15:18], dtype=np.float64)

        return {
            "pos":   pos,
            "vel":   vel,
            "R":     R,
            "omega": omega,
        }


# ---------------------------------------------------------------------------
# Standalone smoke test
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import sys

    logging.basicConfig(level=logging.DEBUG, format="%(levelname)s %(message)s")

    print("MBDyn Interface smoke test")
    print(f"  Force packet size  : {_FORCE_SIZE} bytes")
    print(f"  State packet size  : {_STATE_SIZE} bytes")

    # Pack/unpack round-trip check
    test_forces = np.array([0.0, 0.0, 49.05, 0.0, 0.0, 0.0])
    packed = struct.pack(_FORCE_FMT, *test_forces)
    unpacked = np.array(struct.unpack(_FORCE_FMT, packed))
    assert np.allclose(test_forces, unpacked), "Round-trip failed!"
    print("  Pack/unpack round-trip: OK")

    test_state = np.concatenate([
        [0.0, 0.0, 50.0],          # pos
        [0.0, 0.0,  0.0],          # vel
        np.eye(3).ravel(),          # R (identity)
        [0.0, 0.0, 28.0],          # omega
    ])
    packed_s = struct.pack(_STATE_FMT, *test_state)
    vals = struct.unpack(_STATE_FMT, packed_s)
    state = {
        "pos":   np.array(vals[0:3]),
        "vel":   np.array(vals[3:6]),
        "R":     np.array(vals[6:15]).reshape(3, 3),
        "omega": np.array(vals[15:18]),
    }
    assert np.allclose(state["pos"], [0, 0, 50]), "State pos mismatch!"
    assert np.allclose(state["R"], np.eye(3)),    "State R mismatch!"
    assert np.allclose(state["omega"], [0, 0, 28]), "State omega mismatch!"
    print("  State unpack test    : OK")
    print("All smoke tests passed.")
    sys.exit(0)
