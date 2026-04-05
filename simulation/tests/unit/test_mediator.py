import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from aero import AeroResult

import mediator
import config as _mcfg
DEFAULT_OMEGA_SPIN = _mcfg.DEFAULTS["omega_spin"]
DEFAULT_VEL0       = _mcfg.DEFAULTS["vel0"]


def _state(pos, vel, omega):
    return {
        "pos": np.array(pos, dtype=np.float64),
        "vel": np.array(vel, dtype=np.float64),
        "R": np.eye(3, dtype=np.float64),
        "omega": np.array(omega, dtype=np.float64),
    }


class FakeDynamics:
    """Replaces RigidBodyDynamics — returns pre-canned states from step()."""

    def __init__(self, states):
        self.states = list(states)
        self.step_calls = []   # list of (F_world, M_world, dt, omega_spin)
        self._current = _state([0.0, 0.0, 50.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])

    def step(self, F_world, M_world, dt, omega_spin=0.0):
        self.step_calls.append((
            np.asarray(F_world, dtype=np.float64).copy(),
            np.asarray(M_world, dtype=np.float64).copy(),
            float(dt),
            float(omega_spin),
        ))
        self._current = self.states.pop(0)
        return self._current

    @property
    def state(self):
        return {k: v.copy() for k, v in self._current.items()}


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
        self.last_M_spin    = np.zeros(3, dtype=np.float64)
        self.last_Q_drive   = 0.0
        self.last_Q_drag    = 0.0
        self.last_v_inplane = 0.0

    def compute_forces(self, **kwargs):
        self.compute_calls.append(kwargs)
        return AeroResult(
            F_world   = np.array([1.0, 2.0, 3.0], dtype=np.float64),
            M_orbital = np.array([4.0, 5.0, 6.0], dtype=np.float64),
            Q_spin    = 0.0,
            M_spin    = np.zeros(3, dtype=np.float64),
        )

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


def _install_fakes(monkeypatch, fake_dynamics, fake_sitl, fake_aero, fake_sensor=None):
    monkeypatch.setattr(mediator, "RigidBodyDynamics", lambda **kwargs: fake_dynamics)
    monkeypatch.setattr(mediator, "SITLInterface",     lambda **kwargs: fake_sitl)

    # Mediator now calls SkewedWakeBEM.from_definition(rotor) — patch the classmethod.
    class _FakeSkewedWakeBEM:
        @classmethod
        def from_definition(cls, defn):
            return fake_aero

    monkeypatch.setattr(mediator, "SkewedWakeBEM", _FakeSkewedWakeBEM)
    # PhysicalSensor is NOT mocked here — mediator uses the real sensor with the test hub state.
    # Gyro noise is small (sigma=0.003 rad/s); assertions use atol=0.05 to accommodate it.


def _install_time(monkeypatch, values, sleep_behavior):
    values_iter = iter(values)
    monkeypatch.setattr(mediator.time, "monotonic", lambda: next(values_iter))
    monkeypatch.setattr(mediator.time, "sleep", sleep_behavior)


def _args(tmp_path=None):
    """Build a minimal args Namespace for run_mediator unit tests.

    Writes a JSON config with startup_damp_seconds=0 (so tests hit the
    free-flight physics path immediately, without waiting 30 s of kinematic
    startup) and tether_rest_length=200 (hub starts well inside tether envelope).
    """
    import tempfile, os as _os
    cfg = _mcfg.defaults()
    cfg["startup_damp_seconds"] = 0.0   # disable damping → immediate free flight
    cfg["tether_rest_length"]   = 200.0  # hub starts inside tether envelope
    if tmp_path is None:
        tmp_path = tempfile.mkdtemp()
    cfg_path = _os.path.join(str(tmp_path), "test_mediator_config.json")
    _mcfg.save(cfg, cfg_path)
    return argparse.Namespace(
        config=cfg_path,
        run_id=None,
        sitl_recv_port=9002,
        sitl_send_port=9003,
        log_level="INFO",
        telemetry_log=None,
    )


def test_run_mediator_single_iteration_sends_forces_and_state(monkeypatch):
    stepped_state = _state([10.0, 20.0, 30.0], [4.0, 5.0, 6.0], [0.0, 0.0, 28.0])
    fake_dynamics = FakeDynamics([stepped_state])
    fake_sitl     = FakeSITL([np.zeros(16, dtype=np.float64)])
    fake_aero     = FakeAero()
    fake_sensor   = FakeSensor()

    _install_fakes(monkeypatch, fake_dynamics, fake_sitl, fake_aero, fake_sensor)
    _install_time(monkeypatch, [10.0, 10.1, 10.1005],
                  lambda _seconds: (_ for _ in ()).throw(KeyboardInterrupt()))

    mediator.run_mediator(_args())

    assert fake_sitl.bound is True

    # dynamics.step must be called exactly once per loop iteration
    assert len(fake_dynamics.step_calls) == 1

    # F_world = aero forces [1,2,3] + tether (slack at 50m < 200m rest length = 0)
    np.testing.assert_allclose(fake_dynamics.step_calls[0][0], np.array([1.0, 2.0, 3.0]))
    # M_world = aero moments [4,5,6]
    np.testing.assert_allclose(fake_dynamics.step_calls[0][1], np.array([4.0, 5.0, 6.0]))

    assert fake_aero.compute_calls[0]["collective_rad"] == 0.0
    assert fake_aero.compute_calls[0]["omega_rotor"] == DEFAULT_OMEGA_SPIN
    np.testing.assert_allclose(fake_aero.compute_calls[0]["wind_world"], np.array([0.0, 10.0, 0.0]))  # NED East
    assert len(fake_aero.motor_calls) == 0

    # Mediator uses PhysicalSensor.compute() with NED pos and home_ned_z.
    # pos_ned_rel = [pos[0], pos[1], pos[2] - home_ned_z]
    # home_ned_z = cfg["pos0"][2] (NED Z of start position)
    home_ned_z = _mcfg.DEFAULTS["pos0"][2]   # = -12.530 in NED
    expected_pos_ned = np.array([
        stepped_state["pos"][0],
        stepped_state["pos"][1],
        stepped_state["pos"][2] - home_ned_z,
    ])
    # gyro_body should be near-zero: omega=[0,0,28], body_z=[0,0,1] → omega_nospin=0.
    # PhysicalSensor adds MEMS noise (sigma=0.003 rad/s); allow atol=0.05 for noise.
    assert len(fake_sitl.sent_states) == 1
    np.testing.assert_allclose(fake_sitl.sent_states[0]["pos_ned"],   expected_pos_ned)
    np.testing.assert_allclose(fake_sitl.sent_states[0]["gyro_body"], np.zeros(3), atol=0.05)

    assert fake_sitl.closed is True


def test_run_mediator_uses_separate_omega_spin_for_aero(monkeypatch):
    # omega_spin is kept separate from hub body omega.
    # Even if hub omega is near-zero the aero receives the correct spin.
    fake_dynamics = FakeDynamics([
        _state([0.0, 0.0, 50.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
    ])
    fake_sitl   = FakeSITL([np.zeros(16, dtype=np.float64)])
    fake_aero   = FakeAero()
    fake_sensor = FakeSensor()

    _install_fakes(monkeypatch, fake_dynamics, fake_sitl, fake_aero, fake_sensor)
    _install_time(monkeypatch, [5.0, 5.1, 5.1005],
                  lambda _seconds: (_ for _ in ()).throw(KeyboardInterrupt()))

    mediator.run_mediator(_args())

    assert len(fake_aero.compute_calls) == 1
    assert fake_aero.compute_calls[0]["omega_rotor"] == DEFAULT_OMEGA_SPIN
    assert fake_sitl.closed is True


def test_run_mediator_reuses_last_servo_packet(monkeypatch):
    servo_packet = np.zeros(16, dtype=np.float64)
    servo_packet[0:3] = 0.5

    fake_dynamics = FakeDynamics([
        _state([0.0, 0.0, 50.0], [0.0, 0.0, 0.0], [0.0, 0.0, 28.0]),
        _state([1.0, 1.0, 50.0], [0.0, 0.0, 0.0], [0.0, 0.0, 28.0]),
    ])
    fake_sitl   = FakeSITL([servo_packet, None])
    fake_aero   = FakeAero()
    fake_sensor = FakeSensor()

    sleep_calls = {"count": 0}

    def fake_sleep(_seconds):
        sleep_calls["count"] += 1
        if sleep_calls["count"] >= 2:
            raise KeyboardInterrupt()

    _install_fakes(monkeypatch, fake_dynamics, fake_sitl, fake_aero, fake_sensor)
    _install_time(monkeypatch, [0.0, 0.01, 0.0105, 0.02, 0.0205], fake_sleep)

    mediator.run_mediator(_args())

    assert len(fake_aero.compute_calls) == 2
    assert fake_aero.compute_calls[0]["collective_rad"] == fake_aero.compute_calls[1]["collective_rad"]
    assert fake_aero.compute_calls[0]["tilt_lon"]      == fake_aero.compute_calls[1]["tilt_lon"]
    assert fake_aero.compute_calls[0]["tilt_lat"]      == fake_aero.compute_calls[1]["tilt_lat"]
    assert fake_sitl.closed is True
