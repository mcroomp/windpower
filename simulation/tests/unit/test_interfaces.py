import json
import socket
import struct
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from sitl_interface import SITLInterface


def _get_free_udp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def _make_servo_pkt_16(pwm: list[int], frame_rate: int = 400, frame_count: int = 1) -> bytes:
    from sitl_interface import _SERVO_FMT_16, _SERVO_MAGIC_16
    pwm_full = (pwm + [1500] * 16)[:16]
    return struct.pack(_SERVO_FMT_16, _SERVO_MAGIC_16, frame_rate, frame_count, *pwm_full)


def test_sitl_interface_recv_servos_normalizes_binary_payload():
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

    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sender.bind(("127.0.0.1", 0))
    sender.settimeout(1.0)
    try:
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
        pwm32 = list(range(1000, 1000 + 32 * 50, 50))
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
