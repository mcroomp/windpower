"""Torque mediator physics must use the timestep declared by SITL."""
from __future__ import annotations

import math

import numpy as np
import pytest

from simulation import mediator_torque


def _run_frames(
    monkeypatch, *, frame_rate: int, profile: str, motor_delay_ms: float = 0.0,
    start_time: float = 25.0, pwm_us: float = 1500.0,
):
    dt = 1.0 / frame_rate
    states = []
    steps = []
    real_step = mediator_torque._m.step

    class Interface:
        last_pwm_raw = np.full(16, pwm_us)

        def __init__(self, *, recv_port: int) -> None:
            self.time = 0.0

        def bind(self) -> None:
            pass

        def close(self) -> None:
            pass

        def dt(self) -> float:
            return dt

        def sim_now(self) -> float:
            return self.time

    def lockstep(iface, step_fn, is_stopped, **kwargs):
        for index in range(frame_rate // 2):
            iface.time = start_time + index * dt
            states.append(step_fn(None, iface.time))
        return len(states)

    def step(state, omega_rotor, throttle, params, step_dt):
        steps.append((throttle, step_dt))
        return real_step(state, omega_rotor, throttle, params, step_dt)

    monkeypatch.setattr(mediator_torque, "SITLInterface", Interface)
    monkeypatch.setattr(mediator_torque, "run_lockstep", lockstep)
    monkeypatch.setattr(mediator_torque, "install_sigterm_handler", lambda: lambda: False)
    monkeypatch.setattr(mediator_torque, "setup_logging", lambda level: None)
    monkeypatch.setattr(mediator_torque._m, "step", step)
    mediator_torque.run(28.0, startup_hold_s=0.0, profile=profile, motor_delay_ms=motor_delay_ms)
    return states, steps


@pytest.mark.parametrize("frame_rate", [400, 1200])
def test_motor_ode_uses_sitl_timestep(monkeypatch, frame_rate: int) -> None:
    _, steps = _run_frames(monkeypatch, frame_rate=frame_rate, profile="constant")
    assert all(step_dt == pytest.approx(1.0 / frame_rate) for _, step_dt in steps)


@pytest.mark.parametrize("frame_rate", [400, 1200])
def test_prescribed_angle_change_matches_gyro(monkeypatch, frame_rate: int) -> None:
    states, _ = _run_frames(monkeypatch, frame_rate=frame_rate, profile="yaw_slow_ramp")
    for previous, current in zip(states, states[1:]):
        angle_change = current["rpy_rad"][2] - previous["rpy_rad"][2]
        assert angle_change == pytest.approx(current["gyro_body"][2] / frame_rate)


@pytest.mark.parametrize("frame_rate", [400, 1200])
def test_ic_spin_angle_matches_body_gyro(monkeypatch, frame_rate: int) -> None:
    states, _ = _run_frames(
        monkeypatch, frame_rate=frame_rate, profile="ic", start_time=0.0, pwm_us=1000.0,
    )
    for previous, current in zip(states, states[1:]):
        previous_rotation = mediator_torque._euler_to_R(*previous["rpy_rad"])
        current_rotation = mediator_torque._euler_to_R(*current["rpy_rad"])
        delta = previous_rotation.T @ current_rotation
        spin_angle = math.atan2(delta[1, 0], delta[0, 0])
        assert spin_angle == pytest.approx(current["gyro_body"][2] / frame_rate)


@pytest.mark.parametrize("frame_rate", [400, 1200])
def test_motor_delay_is_in_simulation_seconds(monkeypatch, frame_rate: int) -> None:
    _, steps = _run_frames(
        monkeypatch, frame_rate=frame_rate, profile="constant", motor_delay_ms=300.0,
    )
    first_on = next(index for index, (throttle, _) in enumerate(steps) if throttle > 0.0)
    assert first_on / frame_rate == pytest.approx(0.3, abs=1.0 / frame_rate)


@pytest.mark.parametrize("frame_rate", [400, 1200])
def test_wobble_tilt_rate_matches_attitude_change(monkeypatch, frame_rate: int) -> None:
    states, _ = _run_frames(monkeypatch, frame_rate=frame_rate, profile="wobble")
    for previous, current in zip(states, states[1:]):
        roll, pitch, _ = current["rpy_rad"]
        roll_dot, pitch_dot = (current["rpy_rad"][:2] - previous["rpy_rad"][:2]) * frame_rate
        psi_dot = (current["gyro_body"][2] + pitch_dot * math.sin(roll)) / (
            math.cos(roll) * math.cos(pitch)
        )
        expected, _ = mediator_torque._body_vectors(roll, pitch, psi_dot, roll_dot, pitch_dot)
        np.testing.assert_allclose(current["gyro_body"], expected, atol=1e-10)
