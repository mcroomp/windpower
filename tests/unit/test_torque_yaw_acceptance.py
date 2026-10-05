"""Regression tests for complete, finite torque physics acceptance windows."""
from __future__ import annotations

import logging
import math
from pathlib import Path

import pytest

from simulation.mediator_events import MediatorEventLog
from tests.sitl.torque.torque_test_utils import (
    assert_physics_yaw_rate,
    read_physics_psi_dot,
)


def _events(tmp_path: Path, times: list[float], rate: float = 15.0) -> MediatorEventLog:
    events = MediatorEventLog(tmp_path / "events.jsonl")
    with events:
        events.write("dynamics_start", t_sim=15.0)
        for t in times:
            events.write("heartbeat", t_sim=t, phase="DYNAMIC", psi_dot_deg_s=rate)
    return events


def _assert_rate(events: MediatorEventLog) -> None:
    assert_physics_yaw_rate(
        events, math.radians(16.0), 115.0, 20.0, logging.getLogger(__name__),
    )


@pytest.mark.parametrize("rate", [15.0, 16.0])
def test_complete_window_accepts_small_residual_rate(tmp_path: Path, rate: float) -> None:
    events = _events(tmp_path, [115.1 + index for index in range(20)], rate=rate)
    _assert_rate(events)


def test_rate_above_limit_still_fails(tmp_path: Path) -> None:
    events = _events(tmp_path, [115.1 + index for index in range(20)], rate=16.1)
    with pytest.raises(AssertionError, match="exceeded 16.0 deg/s"):
        _assert_rate(events)


@pytest.mark.parametrize("times", [
    [120.1 + index for index in range(15)],
    [115.1 + index for index in range(15)],
    [115.1, 116.1, 134.1],
    [115.1, 116.1, 116.1, 134.1],
    [115.1 + index for index in range(20) if index != 10],
])
def test_incomplete_window_cannot_pass(tmp_path: Path, times: list[float]) -> None:
    with pytest.raises(pytest.fail.Exception, match="Incomplete physics"):
        _assert_rate(_events(tmp_path, times, rate=0.0))


@pytest.mark.parametrize("rate", [math.nan, math.inf, -math.inf])
def test_non_finite_rate_cannot_pass(tmp_path: Path, rate: float) -> None:
    with pytest.raises(pytest.fail.Exception, match="non-finite psi_dot_deg_s"):
        _assert_rate(_events(tmp_path, [115.1, 116.1, 117.1], rate=rate))


def test_missing_rate_cannot_pass_as_zero(tmp_path: Path) -> None:
    events = MediatorEventLog(tmp_path / "events.jsonl")
    with events:
        events.write("heartbeat", t_sim=115.1, phase="DYNAMIC")
    with pytest.raises(pytest.fail.Exception, match="missing psi_dot_deg_s"):
        _assert_rate(events)


@pytest.mark.parametrize("t_sim", [math.nan, math.inf, -math.inf])
def test_non_finite_time_cannot_pass(tmp_path: Path, t_sim: float) -> None:
    with pytest.raises(pytest.fail.Exception, match="non-finite t_sim"):
        _assert_rate(_events(tmp_path, [t_sim]))


def test_physics_window_uses_absolute_simulation_time(tmp_path: Path) -> None:
    events = _events(tmp_path, [130.0, 145.0, 165.0, 180.0])
    samples = read_physics_psi_dot(events, 145.0, 20.0)
    assert [sample["t"] for sample in samples] == [145.0, 165.0]
