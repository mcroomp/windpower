"""
test_mediator_transport.py — End-to-end mediator test with a real SITL socket peer.

Tests that the mediator produces a correctly formatted JSON state reply when driven
by a real UDP servo packet from a fake SITL client.  The dynamics integrator is
mocked so the test does not depend on physics accuracy.

Note: physics socket transport tests are no longer applicable; the dynamics
integrator runs in-process as RigidBodyDynamics.
"""
import argparse
import json
import socket
import struct
import sys
import threading
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import mediator
import config as _mcfg
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
        from aero import AeroResult
        return AeroResult(
            F_world   = np.array([1.0, 2.0, 3.0], dtype=np.float64),
            M_orbital = np.array([4.0, 5.0, 6.0], dtype=np.float64),
            Q_spin    = 0.0,
            M_spin    = np.zeros(3, dtype=np.float64),
        )

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
    import tempfile, os as _os
    cfg = _mcfg.defaults()
    cfg["startup_damp_seconds"] = 0.0   # disable damping → immediate free flight
    cfg["tether_rest_length"]   = 200.0
    tmp = tempfile.mkdtemp()
    cfg_path = _os.path.join(tmp, "transport_test_config.json")
    _mcfg.save(cfg, cfg_path)
    return argparse.Namespace(
        config=cfg_path,
        run_id=None,
        sitl_recv_port=recv_port,
        sitl_send_port=send_port,
        log_level="WARNING",
        telemetry_log=None,
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
    class _FakeSkewedWakeBEM:
        @classmethod
        def from_definition(cls, defn):
            return fake_aero
    monkeypatch.setattr(mediator, "SkewedWakeBEM", _FakeSkewedWakeBEM)
    monkeypatch.setattr(mediator, "make_sensor",       lambda *a, **kw: fake_sensor)

    times = iter([10.0, 10.1, 10.1005])
    monkeypatch.setattr(mediator.time, "monotonic", lambda: next(times))
    monkeypatch.setattr(
        mediator.time, "sleep",
        lambda _: (_ for _ in ()).throw(KeyboardInterrupt()),
    )

    mediator.run_mediator(_args(recv_port, send_port))

    sitl_thread.join(timeout=2.0)

    # Verify the JSON state reply sent back to the fake SITL.
    # FakeSensor.compute() returns fixed canned values regardless of hub state:
    #   pos_ned=[20,10,-30], vel_ned=[2,1,-3], rpy=[0.1,0.2,0.3],
    #   accel_body=[0.4,0.5,0.6], gyro_body=[0.7,0.8,0.9]
    raw_state, _ = sitl_sock.recvfrom(4096)
    message = json.loads(raw_state.decode("utf-8").strip())

    assert message["position"]       == pytest.approx([20.0, 10.0, -30.0], abs=1e-6)
    assert message["velocity"]       == pytest.approx([2.0,  1.0,  -3.0],  abs=1e-6)
    assert message["attitude"]       == pytest.approx([0.1,  0.2,   0.3],  abs=1e-6)
    assert message["imu"]["accel_body"] == pytest.approx([0.4, 0.5, 0.6],  abs=1e-6)
    assert message["imu"]["gyro"]    == pytest.approx([0.7,  0.8,   0.9],  abs=1e-6)

    # dynamics.step must have been called with the aero forces
    assert len(fake_dynamics.step_calls) == 1
    np.testing.assert_allclose(fake_dynamics.step_calls[0][0], np.array([1.0, 2.0, 3.0]))
    np.testing.assert_allclose(fake_dynamics.step_calls[0][1], np.array([4.0, 5.0, 6.0]))

    sitl_sock.close()
