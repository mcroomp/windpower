"""
test_mbdyn_interface.py — Archived MBDyn interface unit tests (reference only).

These tests were removed from tests/unit/test_interfaces.py when MBDyn was
replaced by the Python RigidBodyDynamics integrator.  They are kept here for
reference if MBDyn is ever restored.  See mbdyn_reference.md for the full
restoration guide.

NOTE: These tests will NOT run as-is on Windows (AF_UNIX) and require
mbdyn_interface.py on the sys.path.  They are reference documentation only.
"""
import socket
import struct
import sys
import threading
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # simulation/archived/

from mbdyn_interface import MBDynInterface, _FORCE_FMT, _FORCE_SIZE, _STATE_FMT, _recv_exactly


def test_recv_exactly_handles_partial_stream_reads():
    left, right = socket.socketpair()
    try:
        right.sendall(b"abc")
        right.sendall(b"def")
        data = _recv_exactly(left, 6)
        assert data == b"abcdef"
    finally:
        left.close()
        right.close()


def test_recv_exactly_raises_if_stream_closes_early():
    left, right = socket.socketpair()
    try:
        right.sendall(b"abc")
        right.close()
        with pytest.raises(ConnectionResetError):
            _recv_exactly(left, 6)
    finally:
        left.close()


def test_mbdyn_interface_send_forces_writes_expected_binary_packet():
    interface = MBDynInterface()
    left, right = socket.socketpair()
    try:
        interface._force_sock = left
        forces = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0], dtype=np.float64)

        interface.send_forces(forces)

        raw = _recv_exactly(right, _FORCE_SIZE)
        unpacked = struct.unpack(_FORCE_FMT, raw)
        np.testing.assert_allclose(np.array(unpacked), forces)
    finally:
        left.close()
        right.close()


def test_mbdyn_interface_recv_state_unpacks_expected_fields():
    interface = MBDynInterface()
    left, right = socket.socketpair()
    try:
        interface._state_sock = left
        # MBDyn output order: pos(3), R(9), vel(3), omega(3)
        state_values = np.concatenate([
            np.array([10.0, 20.0, 30.0], dtype=np.float64),
            np.eye(3, dtype=np.float64).ravel(),
            np.array([1.0, 2.0, 3.0], dtype=np.float64),
            np.array([4.0, 5.0, 6.0], dtype=np.float64),
        ])

        right.sendall(struct.pack(_STATE_FMT, *state_values))
        state = interface.recv_state()

        np.testing.assert_allclose(state["pos"], np.array([10.0, 20.0, 30.0]))
        np.testing.assert_allclose(state["vel"], np.array([1.0, 2.0, 3.0]))
        np.testing.assert_allclose(state["R"], np.eye(3))
        np.testing.assert_allclose(state["omega"], np.array([4.0, 5.0, 6.0]))
    finally:
        left.close()
        right.close()


def test_mbdyn_interface_connects_to_real_unix_socket_paths():
    if not hasattr(socket, "AF_UNIX"):
        pytest.skip("AF_UNIX sockets are not available on this platform")

    with TemporaryDirectory() as tmpdir:
        force_path = str(Path(tmpdir) / "forces.sock")
        state_path = str(Path(tmpdir) / "state.sock")

        force_server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        state_server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        force_server.bind(force_path)
        state_server.bind(state_path)
        force_server.listen(1)
        state_server.listen(1)

        interface = MBDynInterface(
            force_sock_path=force_path,
            state_sock_path=state_path,
            connect_timeout=1.0,
        )

        connected = threading.Event()

        def accept_both():
            force_server.accept()
            state_server.accept()
            connected.set()

        t = threading.Thread(target=accept_both)
        t.start()

        interface.connect()
        connected.wait(timeout=2.0)
        assert connected.is_set()

        interface.close()
        force_server.close()
        state_server.close()
        t.join(timeout=1.0)
