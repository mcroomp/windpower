"""
test_mediator_transport_mbdyn.py — Archived MBDyn mediator transport test (reference only).

This test was removed from tests/unit/test_mediator_transport.py when MBDyn was
replaced by the Python RigidBodyDynamics integrator.  It verifies the full
mediator pipeline using real UNIX socket peers for MBDyn and real UDP sockets
for SITL.  Kept here for reference if MBDyn is ever restored.

See mbdyn_reference.md for the full restoration guide.

NOTE: Requires Linux (AF_UNIX), mbdyn_interface.py on sys.path, and the
original MBDyn-based mediator.py.  This is reference documentation only.
"""
import argparse
import json
import socket
import struct
import sys
import threading
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # simulation/archived/
sys.path.insert(0, str(Path(__file__).resolve().parents[1].parent))  # simulation/

import mediator
from mbdyn_interface import _FORCE_FMT, _FORCE_SIZE, _STATE_FMT, _recv_exactly
from mbdyn_interface import MBDynInterface
from sitl_interface import SITLInterface


def _get_free_udp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def _pack_state(pos, vel, omega):
    # MBDyn output order: pos(3), R(9), vel(3), omega(3)
    values = np.concatenate([
        np.array(pos, dtype=np.float64),
        np.eye(3, dtype=np.float64).ravel(),
        np.array(vel, dtype=np.float64),
        np.array(omega, dtype=np.float64),
    ])
    return struct.pack(_STATE_FMT, *values)


class FakeAero:
    def __init__(self):
        self.compute_calls = []
        self.motor_calls = []

    def compute_forces(self, **kwargs):
        self.compute_calls.append(kwargs)
        return np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0], dtype=np.float64)

    def compute_anti_rotation_moment(self, **kwargs):
        self.motor_calls.append(kwargs)
        return 7.0


class FakeSensor:
    def __init__(self):
        self.calls = []

    def compute(self, **kwargs):
        self.calls.append(kwargs)
        return {
            "pos_ned": np.array([20.0, 10.0, -30.0], dtype=np.float64),
            "vel_ned": np.array([2.0, 1.0, -3.0], dtype=np.float64),
            "rpy": np.array([0.1, 0.2, 0.3], dtype=np.float64),
            "accel_body": np.array([0.4, 0.5, 0.6], dtype=np.float64),
            "gyro_body": np.array([0.7, 0.8, 0.9], dtype=np.float64),
        }


def _args(force_path, state_path, recv_port, send_port):
    return argparse.Namespace(
        mbdyn_force_sock=force_path,
        mbdyn_state_sock=state_path,
        sitl_recv_port=recv_port,
        sitl_send_port=send_port,
        wind_x=10.0,
        wind_y=0.0,
        wind_z=0.0,
        log_level="INFO",
    )


def _make_servo_pkt_16(pwm: list[int], frame_rate: int = 400, frame_count: int = 1) -> bytes:
    from sitl_interface import _SERVO_FMT_16, _SERVO_MAGIC_16
    pwm_full = (pwm + [1500] * 16)[:16]
    return struct.pack(_SERVO_FMT_16, _SERVO_MAGIC_16, frame_rate, frame_count, *pwm_full)


def test_run_mediator_uses_real_interfaces_with_fake_socket_peers(monkeypatch):
    """
    Full pipeline test: real UNIX sockets for MBDyn, real UDP sockets for SITL,
    mocked aero and sensor.  Verifies:
    - GRAVITY_COMP warm-up force is sent first
    - Aero forces (minus motor torque) are sent on second call
    - JSON state reply is correctly formatted and sent back to SITL
    """
    recv_port = _get_free_udp_port()

    # This socket acts as SITL: sends binary servo packets and receives state replies.
    sitl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sitl_sock.bind(("127.0.0.1", 0))
    sitl_sock.settimeout(1.0)

    force_client, force_peer = socket.socketpair()
    state_client, state_peer = socket.socketpair()

    captured_forces = []
    state_packets = [
        _pack_state([0.0, 0.0, 50.0], [0.0, 0.0, 0.0], [0.0, 0.0, 28.0]),
        _pack_state([10.0, 20.0, 30.0], [4.0, 5.0, 6.0], [0.0, 0.0, 28.0]),
    ]
    sitl_bound = threading.Event()

    def force_peer_worker():
        try:
            for _ in range(2):
                raw = _recv_exactly(force_peer, _FORCE_SIZE)
                captured_forces.append(np.array(struct.unpack(_FORCE_FMT, raw)))
        finally:
            force_peer.close()

    def state_peer_worker():
        try:
            for packet in state_packets:
                state_peer.sendall(packet)
        finally:
            state_peer.close()

    force_thread = threading.Thread(target=force_peer_worker)
    state_thread = threading.Thread(target=state_peer_worker)
    force_thread.start()
    state_thread.start()

    real_mbdyn = MBDynInterface(
        force_sock_path="unused-force.sock",
        state_sock_path="unused-state.sock",
        connect_timeout=1.0,
    )

    def connect_with_preconnected_sockets():
        real_mbdyn._force_sock = force_client
        real_mbdyn._state_sock = state_client

    real_mbdyn.connect = connect_with_preconnected_sockets

    real_sitl = SITLInterface(recv_port=recv_port, recv_timeout_ms=50.0)

    original_bind = real_sitl.bind

    def bind_and_signal():
        original_bind()
        sitl_bound.set()

    real_sitl.bind = bind_and_signal

    def sitl_sender():
        sitl_bound.wait(timeout=1.0)
        pkt = _make_servo_pkt_16([1500, 1500, 1500, 1750])
        sitl_sock.sendto(pkt, ("127.0.0.1", recv_port))

    sitl_thread = threading.Thread(target=sitl_sender)
    sitl_thread.start()

    fake_aero = FakeAero()
    fake_sensor = FakeSensor()

    monkeypatch.setattr(mediator, "MBDynInterface", lambda **kwargs: real_mbdyn)
    monkeypatch.setattr(mediator, "SITLInterface", lambda **kwargs: real_sitl)
    monkeypatch.setattr(mediator, "RotorAero", lambda: fake_aero)
    monkeypatch.setattr(mediator, "SensorSim", lambda: fake_sensor)

    times = iter([10.0, 10.1, 10.1005])
    monkeypatch.setattr(mediator.time, "monotonic", lambda: next(times))
    monkeypatch.setattr(
        mediator.time,
        "sleep",
        lambda _seconds: (_ for _ in ()).throw(KeyboardInterrupt()),
    )

    mediator.run_mediator(_args("unused-force.sock", "unused-state.sock", recv_port, 9003))

    force_thread.join(timeout=1.0)
    state_thread.join(timeout=1.0)
    sitl_thread.join(timeout=1.0)

    assert len(captured_forces) == 2
    np.testing.assert_allclose(captured_forces[0], mediator.GRAVITY_COMP)
    # Motor torque is NOT added — it is internal in the single-body MBDyn model
    np.testing.assert_allclose(captured_forces[1], np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0]))

    # State reply is sent back to sitl_sock's source address (not a fixed port)
    raw_state, _ = sitl_sock.recvfrom(4096)
    message = json.loads(raw_state.decode("utf-8").strip())
    assert message["position"] == [20.0, 10.0, -30.0]
    assert message["velocity"] == [2.0, 1.0, -3.0]
    assert message["attitude"] == [0.1, 0.2, 0.3]
    assert message["imu"]["accel_body"] == [0.4, 0.5, 0.6]
    assert message["imu"]["gyro"] == [0.7, 0.8, 0.9]

    sitl_sock.close()
