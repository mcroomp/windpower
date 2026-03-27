import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import mediator


def _state(pos, vel, omega):
    return {
        "pos": np.array(pos, dtype=np.float64),
        "vel": np.array(vel, dtype=np.float64),
        "R": np.eye(3, dtype=np.float64),
        "omega": np.array(omega, dtype=np.float64),
    }


class FakeMBDyn:
    def __init__(self, states):
        self.states = list(states)
        self.sent_forces = []
        self.connected = False
        self.closed = False

    def connect(self):
        self.connected = True

    def send_forces(self, forces):
        self.sent_forces.append(np.asarray(forces, dtype=np.float64))

    def recv_state(self):
        return self.states.pop(0)

    def close(self):
        self.closed = True


class FakeSITL:
    def __init__(self, servo_packets):
        self.servo_packets = list(servo_packets)
        self.sent_states = []
        self.bound = False
        self.closed = False

    def bind(self):
        self.bound = True

    def recv_servos(self):
        if self.servo_packets:
            return self.servo_packets.pop(0)
        return None

    def send_state(self, **kwargs):
        self.sent_states.append(kwargs)

    def close(self):
        self.closed = True


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


def _install_fakes(monkeypatch, fake_mbdyn, fake_sitl, fake_aero, fake_sensor):
    monkeypatch.setattr(mediator, "MBDynInterface", lambda **kwargs: fake_mbdyn)
    monkeypatch.setattr(mediator, "SITLInterface", lambda **kwargs: fake_sitl)
    monkeypatch.setattr(mediator, "RotorAero", lambda: fake_aero)
    monkeypatch.setattr(mediator, "SensorSim", lambda: fake_sensor)


def _install_time(monkeypatch, values, sleep_behavior):
    values_iter = iter(values)
    monkeypatch.setattr(mediator.time, "monotonic", lambda: next(values_iter))
    monkeypatch.setattr(mediator.time, "sleep", sleep_behavior)


def _args():
    return argparse.Namespace(
        mbdyn_force_sock="/tmp/rawes_forces.sock",
        mbdyn_state_sock="/tmp/rawes_state.sock",
        sitl_recv_port=9002,
        sitl_send_port=9003,
        wind_x=10.0,
        wind_y=0.0,
        wind_z=0.0,
        log_level="INFO",
    )


def test_run_mediator_single_iteration_sends_forces_and_state(monkeypatch):
    initial_state = _state([0.0, 0.0, 50.0], [0.0, 0.0, 0.0], [0.0, 0.0, 28.0])
    stepped_state = _state([10.0, 20.0, 30.0], [4.0, 5.0, 6.0], [0.0, 0.0, 28.0])
    fake_mbdyn = FakeMBDyn([
        initial_state,
        stepped_state,
    ])
    fake_sitl = FakeSITL([np.zeros(16, dtype=np.float64)])
    fake_aero = FakeAero()
    fake_sensor = FakeSensor()

    _install_fakes(monkeypatch, fake_mbdyn, fake_sitl, fake_aero, fake_sensor)
    _install_time(monkeypatch, [10.0, 10.1, 10.1005], lambda _seconds: (_ for _ in ()).throw(KeyboardInterrupt()))

    mediator.run_mediator(_args())

    assert fake_mbdyn.connected is True
    assert fake_sitl.bound is True
    assert len(fake_mbdyn.sent_forces) == 2
    np.testing.assert_allclose(fake_mbdyn.sent_forces[0], mediator.GRAVITY_COMP)
    # Motor torque is NOT added — it is internal in the single-body MBDyn model
    np.testing.assert_allclose(fake_mbdyn.sent_forces[1], np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0]))

    assert len(fake_aero.compute_calls) == 1
    assert fake_aero.compute_calls[0]["collective_rad"] == 0.0
    assert fake_aero.compute_calls[0]["omega_rotor"] == 28.0
    np.testing.assert_allclose(fake_aero.compute_calls[0]["wind_world"], np.array([10.0, 0.0, 0.0]))
    assert len(fake_aero.motor_calls) == 0

    assert len(fake_sensor.calls) == 1
    expected_accel = (stepped_state["vel"] - initial_state["vel"]) / mediator.DT_TARGET
    np.testing.assert_allclose(fake_sensor.calls[0]["accel_world_enu"], expected_accel)

    assert len(fake_sitl.sent_states) == 1
    np.testing.assert_allclose(fake_sitl.sent_states[0]["pos_ned"], np.array([20.0, 10.0, -30.0]))
    np.testing.assert_allclose(fake_sitl.sent_states[0]["gyro_body"], np.array([0.7, 0.8, 0.9]))
    assert fake_mbdyn.closed is True
    assert fake_sitl.closed is True


def test_run_mediator_falls_back_to_nominal_rotor_speed(monkeypatch):
    fake_mbdyn = FakeMBDyn([
        _state([0.0, 0.0, 50.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.2]),
        _state([0.0, 0.0, 50.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.2]),
    ])
    fake_sitl = FakeSITL([np.zeros(16, dtype=np.float64)])
    fake_aero = FakeAero()
    fake_sensor = FakeSensor()

    _install_fakes(monkeypatch, fake_mbdyn, fake_sitl, fake_aero, fake_sensor)
    _install_time(monkeypatch, [5.0, 5.1, 5.1005], lambda _seconds: (_ for _ in ()).throw(KeyboardInterrupt()))

    mediator.run_mediator(_args())

    assert len(fake_aero.compute_calls) == 1
    assert fake_aero.compute_calls[0]["omega_rotor"] == 28.0
    assert fake_mbdyn.closed is True
    assert fake_sitl.closed is True


def test_run_mediator_reuses_last_servo_packet(monkeypatch):
    servo_packet = np.zeros(16, dtype=np.float64)
    servo_packet[0:3] = 0.5

    fake_mbdyn = FakeMBDyn([
        _state([0.0, 0.0, 50.0], [0.0, 0.0, 0.0], [0.0, 0.0, 28.0]),
        _state([1.0, 1.0, 50.0], [0.0, 0.0, 0.0], [0.0, 0.0, 28.0]),
        _state([2.0, 2.0, 50.0], [0.0, 0.0, 0.0], [0.0, 0.0, 28.0]),
    ])
    fake_sitl = FakeSITL([servo_packet, None])
    fake_aero = FakeAero()
    fake_sensor = FakeSensor()

    sleep_calls = {"count": 0}

    def fake_sleep(_seconds):
        sleep_calls["count"] += 1
        if sleep_calls["count"] >= 2:
            raise KeyboardInterrupt()

    _install_fakes(monkeypatch, fake_mbdyn, fake_sitl, fake_aero, fake_sensor)
    _install_time(monkeypatch, [0.0, 0.01, 0.0105, 0.02, 0.0205], fake_sleep)

    mediator.run_mediator(_args())

    assert len(fake_aero.compute_calls) == 2
    assert fake_aero.compute_calls[0]["collective_rad"] == fake_aero.compute_calls[1]["collective_rad"]
    assert fake_aero.compute_calls[0]["tilt_lon"] == fake_aero.compute_calls[1]["tilt_lon"]
    assert fake_aero.compute_calls[0]["tilt_lat"] == fake_aero.compute_calls[1]["tilt_lat"]
    assert fake_mbdyn.closed is True
    assert fake_sitl.closed is True