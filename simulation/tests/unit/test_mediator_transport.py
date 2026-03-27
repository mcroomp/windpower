"""
test_mediator_transport.py — End-to-end mediator test with a real SITL socket peer.

Tests that the mediator produces a correctly formatted JSON state reply when driven
by a real UDP servo packet from a fake SITL client.  The dynamics integrator is
mocked so the test does not depend on physics accuracy.

Note: MBDyn socket transport tests were removed when MBDyn was replaced by the
Python RigidBodyDynamics integrator.  See simulation/mbdyn_reference.md for the
former MBDyn socket protocol documentation.
"""
import argparse
import json
import socket
import struct
import sys
import threading
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import mediator
from sitl_interface import SITLInterface


def _get_free_udp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def _make_servo_pkt_16(pwm: list[int], frame_rate: int = 400, frame_count: int = 1) -> bytes:
    from sitl_interface import _SERVO_FMT_16, _SERVO_MAGIC_16
    pwm_full = (pwm + [1500] * 16)[:16]
    return struct.pack(_SERVO_FMT_16, _SERVO_MAGIC_16, frame_rate, frame_count, *pwm_full)


def _state(pos, vel, omega):
    return {
        "pos":   np.array(pos,   dtype=np.float64),
        "vel":   np.array(vel,   dtype=np.float64),
        "R":     np.eye(3,       dtype=np.float64),
        "omega": np.array(omega, dtype=np.float64),
    }


class FakeDynamics:
    def __init__(self, states):
        self.states = list(states)
        self.step_calls = []
        self._current = _state([0.0, 0.0, 50.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])

    def step(self, F_world, M_world, dt, omega_spin=0.0):
        self.step_calls.append((
            np.asarray(F_world).copy(),
            np.asarray(M_world).copy(),
            float(dt),
            float(omega_spin),
        ))
        self._current = self.states.pop(0)
        return self._current

    @property
    def state(self):
        return {k: v.copy() for k, v in self._current.items()}


class FakeAero:
    last_M_spin    = np.zeros(3, dtype=np.float64)
    last_Q_drive   = 0.0
    last_Q_drag    = 0.0
    last_v_inplane = 0.0

    def compute_forces(self, **kwargs):
        return np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0], dtype=np.float64)

    def compute_anti_rotation_moment(self, **kwargs):
        return 0.0


class FakeSensor:
    def compute(self, **kwargs):
        return {
            "pos_ned":    np.array([20.0, 10.0, -30.0], dtype=np.float64),
            "vel_ned":    np.array([2.0,  1.0,   -3.0], dtype=np.float64),
            "rpy":        np.array([0.1,  0.2,    0.3], dtype=np.float64),
            "accel_body": np.array([0.4,  0.5,    0.6], dtype=np.float64),
            "gyro_body":  np.array([0.7,  0.8,    0.9], dtype=np.float64),
        }


def _args(recv_port, send_port):
    return argparse.Namespace(
        sitl_recv_port=recv_port,
        sitl_send_port=send_port,
        wind_x=10.0,
        wind_y=0.0,
        wind_z=0.0,
        log_level="WARNING",
        telemetry_log=None,
        tether_rest_length=200.0,
        startup_freeze_seconds=0.0,  # disable freeze so transport test hits physics path
        pos0=None,
        vel0=None,
        body_z=None,
        omega_spin=None,
    )


def test_run_mediator_uses_real_sitl_interface_with_fake_dynamics(monkeypatch):
    """
    Full pipeline test: real UDP sockets for SITL, mocked dynamics and aero.
    Verifies the mediator produces a correctly structured JSON state reply.
    """
    recv_port = _get_free_udp_port()
    send_port = _get_free_udp_port()

    # Fake SITL client: sends a servo packet, waits for a JSON state reply
    sitl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sitl_sock.bind(("127.0.0.1", send_port))
    sitl_sock.settimeout(2.0)

    fake_dynamics = FakeDynamics([
        _state([10.0, 20.0, 30.0], [4.0, 5.0, 6.0], [0.0, 0.0, 28.0]),
    ])
    fake_aero   = FakeAero()
    fake_sensor = FakeSensor()

    real_sitl = SITLInterface(recv_port=recv_port, send_port=send_port, recv_timeout_ms=100.0)

    sitl_bound = threading.Event()
    original_bind = real_sitl.bind

    def bind_and_signal():
        original_bind()
        sitl_bound.set()

    real_sitl.bind = bind_and_signal

    def sitl_sender():
        sitl_bound.wait(timeout=2.0)
        pkt = _make_servo_pkt_16([1500, 1500, 1500, 1750])
        sitl_sock.sendto(pkt, ("127.0.0.1", recv_port))

    sitl_thread = threading.Thread(target=sitl_sender)
    sitl_thread.start()

    monkeypatch.setattr(mediator, "RigidBodyDynamics", lambda **kwargs: fake_dynamics)
    monkeypatch.setattr(mediator, "SITLInterface",     lambda **kwargs: real_sitl)
    monkeypatch.setattr(mediator, "RotorAero",         lambda: fake_aero)
    monkeypatch.setattr(mediator, "SensorSim",         lambda **kw: fake_sensor)

    times = iter([10.0, 10.1, 10.1005])
    monkeypatch.setattr(mediator.time, "monotonic", lambda: next(times))
    monkeypatch.setattr(
        mediator.time, "sleep",
        lambda _: (_ for _ in ()).throw(KeyboardInterrupt()),
    )

    mediator.run_mediator(_args(recv_port, send_port))

    sitl_thread.join(timeout=2.0)

    # Verify the JSON state reply sent back to the fake SITL
    raw_state, _ = sitl_sock.recvfrom(4096)
    message = json.loads(raw_state.decode("utf-8").strip())

    assert message["position"] == [20.0, 10.0, -30.0]
    assert message["velocity"] == [2.0, 1.0, -3.0]
    assert message["attitude"] == [0.1, 0.2, 0.3]
    assert message["imu"]["accel_body"] == [0.4, 0.5, 0.6]
    assert message["imu"]["gyro"] == [0.7, 0.8, 0.9]

    # dynamics.step must have been called with the aero forces
    assert len(fake_dynamics.step_calls) == 1
    np.testing.assert_allclose(fake_dynamics.step_calls[0][0], np.array([1.0, 2.0, 3.0]))
    np.testing.assert_allclose(fake_dynamics.step_calls[0][1], np.array([4.0, 5.0, 6.0]))

    sitl_sock.close()
