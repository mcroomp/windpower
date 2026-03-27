import json
import socket
import struct
import sys
import threading
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from mbdyn_interface import MBDynInterface, _FORCE_FMT, _FORCE_SIZE, _STATE_FMT, _recv_exactly
from sitl_interface import SITLInterface


def _get_free_udp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


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

        accepted = {}

        def accept_force():
            conn, _ = force_server.accept()
            accepted["force"] = conn

        def accept_state():
            conn, _ = state_server.accept()
            accepted["state"] = conn

        force_thread = threading.Thread(target=accept_force)
        state_thread = threading.Thread(target=accept_state)
        force_thread.start()
        state_thread.start()

        interface = MBDynInterface(
            force_sock_path=force_path,
            state_sock_path=state_path,
            connect_timeout=1.0,
        )

        try:
            interface.connect()

            force_thread.join(timeout=1.0)
            state_thread.join(timeout=1.0)

            interface.send_forces(np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0]))
            raw_force = _recv_exactly(accepted["force"], _FORCE_SIZE)
            np.testing.assert_allclose(struct.unpack(_FORCE_FMT, raw_force), np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0]))

            # MBDyn output order: pos(3), R(9), vel(3), omega(3)
            state_values = np.concatenate([
                np.array([7.0, 8.0, 9.0]),
                np.eye(3).ravel(),
                np.array([10.0, 11.0, 12.0]),
                np.array([13.0, 14.0, 15.0]),
            ])
            accepted["state"].sendall(struct.pack(_STATE_FMT, *state_values))

            state = interface.recv_state()
            np.testing.assert_allclose(state["pos"], np.array([7.0, 8.0, 9.0]))
            np.testing.assert_allclose(state["vel"], np.array([10.0, 11.0, 12.0]))
            np.testing.assert_allclose(state["omega"], np.array([13.0, 14.0, 15.0]))
        finally:
            interface.close()
            for conn in accepted.values():
                conn.close()
            force_server.close()
            state_server.close()


def _make_servo_pkt_16(pwm: list[int], frame_rate: int = 400, frame_count: int = 1) -> bytes:
    from sitl_interface import _SERVO_FMT_16, _SERVO_MAGIC_16
    pwm_full = (pwm + [1500] * 16)[:16]
    return struct.pack(_SERVO_FMT_16, _SERVO_MAGIC_16, frame_rate, frame_count, *pwm_full)


def test_sitl_interface_recv_servos_normalizes_binary_payload():
    from sitl_interface import _SERVO_MAGIC_16
    recv_port = _get_free_udp_port()
    interface = SITLInterface(recv_port=recv_port, recv_timeout_ms=50.0)
    interface.bind()
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        pkt = _make_servo_pkt_16([1000, 1500, 2000, 1250])
        sender.sendto(pkt, ("127.0.0.1", recv_port))

        servos = interface.recv_servos()

        expected = np.zeros(16, dtype=np.float64)
        expected[:4] = np.array([-1.0, 0.0, 1.0, -0.5])
        np.testing.assert_allclose(servos, expected)
        np.testing.assert_allclose(interface.last_servos(), expected)
    finally:
        sender.close()
        interface.close()


def test_sitl_interface_send_state_replies_to_servo_source():
    """send_state must reply to the source address of the last servo packet."""
    recv_port = _get_free_udp_port()
    interface = SITLInterface(recv_port=recv_port, recv_timeout_ms=200.0)
    interface.bind()

    # sender acts as the SITL process
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sender.bind(("127.0.0.1", 0))
    sender.settimeout(1.0)
    try:
        # Send a servo packet so the interface learns SITL's address
        sender.sendto(_make_servo_pkt_16([1500] * 16), ("127.0.0.1", recv_port))
        interface.recv_servos()

        interface.send_state(
            timestamp=1.5,
            pos_ned=np.array([1.0, 2.0, -3.0]),
            vel_ned=np.array([4.0, 5.0, -6.0]),
            rpy_rad=np.array([0.1, 0.2, 0.3]),
            accel_body=np.array([7.0, 8.0, 9.0]),
            gyro_body=np.array([10.0, 11.0, 12.0]),
        )

        raw, _addr = sender.recvfrom(4096)
        message = json.loads(raw.decode("utf-8").strip())

        assert message["timestamp"] == 1.5
        assert message["position"] == [1.0, 2.0, -3.0]
        assert message["velocity"] == [4.0, 5.0, -6.0]
        assert message["attitude"] == [0.1, 0.2, 0.3]
        assert message["imu"]["accel_body"] == [7.0, 8.0, 9.0]
        assert message["imu"]["gyro"] == [10.0, 11.0, 12.0]
    finally:
        sender.close()
        interface.close()


def test_sitl_interface_send_state_no_op_before_first_servo():
    """send_state should silently do nothing before any servo packet arrives."""
    recv_port = _get_free_udp_port()
    interface = SITLInterface(recv_port=recv_port, recv_timeout_ms=50.0)
    interface.bind()
    try:
        # Should not raise
        interface.send_state(
            timestamp=0.0,
            pos_ned=np.zeros(3),
            vel_ned=np.zeros(3),
            rpy_rad=np.zeros(3),
            accel_body=np.zeros(3),
            gyro_body=np.zeros(3),
        )
    finally:
        interface.close()


def test_sitl_interface_recv_servos_returns_none_for_wrong_size():
    recv_port = _get_free_udp_port()
    interface = SITLInterface(recv_port=recv_port, recv_timeout_ms=50.0)
    interface.bind()
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sender.sendto(b"not-binary-servo", ("127.0.0.1", recv_port))
        assert interface.recv_servos() is None
    finally:
        sender.close()
        interface.close()


def test_sitl_interface_recv_servos_keeps_last_good_packet_on_bad_packet():
    recv_port = _get_free_udp_port()
    interface = SITLInterface(recv_port=recv_port, recv_timeout_ms=50.0)
    interface.bind()
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sender.sendto(_make_servo_pkt_16([1500, 1750, 1250]), ("127.0.0.1", recv_port))
        expected = interface.recv_servos()
        assert expected is not None

        sender.sendto(b"bad-packet", ("127.0.0.1", recv_port))
        assert interface.recv_servos() is None
        np.testing.assert_allclose(interface.last_servos(), expected)
    finally:
        sender.close()
        interface.close()


def test_sitl_interface_recv_servos_32ch_reads_first_16():
    from sitl_interface import _SERVO_FMT_32, _SERVO_MAGIC_32
    recv_port = _get_free_udp_port()
    interface = SITLInterface(recv_port=recv_port, recv_timeout_ms=50.0)
    interface.bind()
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        pwm32 = list(range(1000, 1000 + 32 * 50, 50))   # 32 distinct values
        pkt = struct.pack(_SERVO_FMT_32, _SERVO_MAGIC_32, 400, 1, *pwm32)
        sender.sendto(pkt, ("127.0.0.1", recv_port))

        servos = interface.recv_servos()
        assert servos is not None
        assert servos.shape == (16,)
        expected_pwm = np.array(pwm32[:16], dtype=np.float64)
        expected = np.clip((expected_pwm - 1500.0) / 500.0, -1.0, 1.0)
        np.testing.assert_allclose(servos, expected)
    finally:
        sender.close()
        interface.close()