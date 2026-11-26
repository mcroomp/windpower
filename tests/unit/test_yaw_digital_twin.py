import json
import math
from pathlib import Path

import numpy as np
import pytest

from analysis.yaw_digital_twin import (
    DEFAULT_YAW_SLOPE_RPM_PER_US,
    LUA_DT,
    Configuration,
    Strategy,
    captured_parameters,
    fit_amplitude,
    harmonic_amplitude,
    mark_windows,
    observations,
    observed_phase,
    replay_observer,
    simulate,
)
from simulation.rawes_lua_harness import RawesLua


def test_two_fitted_disturbances_can_match_gyro_but_disagree_about_body_motion():
    config = Configuration(trim_equilibrium=0.135, motor_rpm=582)
    strategy = Strategy(rate_p=config.yaw_rate_p, trim_tau_s=config.trim_tau_s)
    measured = 19.0
    sensor = simulate(config, "sensor", fit_amplitude(config, "sensor", measured), strategy)
    torque = simulate(config, "torque", fit_amplitude(config, "torque", measured), strategy)
    assert sensor.measured_yaw_std_deg_s == pytest.approx(measured, abs=0.05)
    assert torque.measured_yaw_std_deg_s == pytest.approx(measured, abs=0.05)
    assert sensor.actual_yaw_std_deg_s > torque.actual_yaw_std_deg_s * 2


def test_fixed_trim_does_not_remove_external_oscillation():
    config = Configuration(trim_equilibrium=0.135, motor_rpm=582)
    adaptive = simulate(config, "torque", 1.0, Strategy(0.12, 0.3))
    fixed = simulate(config, "torque", 1.0, Strategy(0.12, None))
    assert adaptive.measured_yaw_std_deg_s > 10
    assert fixed.measured_yaw_std_deg_s > 10
    assert fixed.trim_std < 1e-10
    assert adaptive.trim_std > fixed.trim_std


def test_fractional_control_delay_is_rejected():
    config = Configuration(trim_equilibrium=0.135, motor_rpm=582, delay_s=0.006)
    with pytest.raises(ValueError, match="whole number"):
        simulate(config, "torque", 1.0, Strategy(0.12, 0.3))


def test_observed_phase_does_not_bridge_long_gaps():
    time = np.arange(0, 3, 0.01)
    phase = math.radians(120)
    attitude = [(float(t), math.sin(math.tau * 8.3333333333 * t)) for t in time]
    servo = [(float(t), math.sin(math.tau * 8.3333333333 * t + phase)) for t in time]
    blocks, measured = observed_phase(attitude, servo)
    assert blocks >= 3
    assert measured == pytest.approx(120, abs=1)
    sparse = [(t, v) for t, v in servo if t < 0.2 or t > 2.8]
    blocks, measured = observed_phase(attitude, sparse)
    assert blocks == 0
    assert measured is None


def test_observations_require_armed_fresh_heartbeat(tmp_path: Path):
    origin = 1_000_000_000
    records = []

    def add(t: float, name: str, fields: dict):
        records.append({
            "kind": "mavlink.rx", "ingest_time_ns": origin + int(t * 1e9),
            "sim_clock": {"epoch": 1},
            "data": {"system_id": 1, "component_id": 1, "message_name": name, "fields": fields},
        })

    add(0, "HEARTBEAT", {"base_mode": "MAV_MODE_FLAG_SAFETY_ARMED", "custom_mode": 20})
    for i in range(12):
        t = 0.1 + i * 0.05
        add(t, "PID_TUNING", {"axis": {"type": "PID_TUNING_YAW"}, "P": i / 1000, "achieved": i / 100})
        add(t, "DEBUG_FLOAT_ARRAY", {"array_id": 1, "data": [3, 0.14, 0.15]})
        add(t, "RPM", {"rpm1": 580.0})
    path = tmp_path / "mavlog.json"
    payload = {"experiment": {"started_at_ns": origin}, "record_count": len(records), "records": records}
    path.write_text(json.dumps(payload), encoding="utf-8")
    result = observations(path, [(0, 1)])[0]
    assert result.pid_samples == 12
    assert result.diagnostic_samples == 12
    assert result.vehicle_mode == 20
    assert result.motor_rpm_mean == 580
    records[0]["data"]["fields"]["base_mode"] = ""
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="Insufficient samples"):
        observations(path, [(0, 1)])
    records[0]["data"]["fields"]["base_mode"] = "MAV_MODE_FLAG_SAFETY_ARMED"
    records[0]["ingest_time_ns"] = origin - 4_000_000_000
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="Insufficient samples"):
        observations(path, [(0, 1)])


def test_mark_windows_skip_the_transition_after_each_mark(tmp_path: Path):
    origin = 1_000_000_000
    records = [
        {"kind": "diagnostic.event", "ingest_time_ns": origin + int(t * 1e9),
         "data": {"event": "experiment.mark", "fields": {"label": label}}}
        for t, label in ((10, "high"), (30, "low"), (30.5, "blip"))
    ]
    path = tmp_path / "mavlog.json"
    path.write_text(json.dumps({
        "experiment": {"started_at_ns": origin, "stopped_at_ns": origin + 50_000_000_000},
        "records": records,
    }), encoding="utf-8")
    assert mark_windows(path) == {"high": (11.0, 30.0), "blip": (31.5, 50.0)}


def test_harmonic_amplitude_handles_irregular_gapped_samples():
    rng = np.random.default_rng(1)
    t = np.sort(rng.uniform(0, 4, 300))
    t = t[(t < 1.2) | (t > 1.9)]
    x = 0.04 * np.sin(math.tau * 8.58 * t + 0.7) + 0.13 + 0.002 * t
    assert harmonic_amplitude(t, x, 8.58) == pytest.approx(0.04, rel=1e-6)
    assert harmonic_amplitude(t, x, 5.0) < 0.01


def _lua_recording(tau: float, slope: float, smin: float, span: float):
    """Servo, gyro and diagnostic streams as rawes.lua would log them, with a dropout."""
    servo_t = np.arange(0, 4, 0.0025)
    keep = (servo_t < 1.5) | (servo_t > 1.9)
    u = 0.15 + 0.04 * np.sin(math.tau * 8.6 * servo_t) + 0.01 * np.sin(math.tau * 0.7 * servo_t)
    gyro = 0.35 * np.sin(math.tau * 8.6 * servo_t + 2.4)
    lua = RawesLua(RAWES_YFF_TAU=tau, RAWES_YAW_SLP=slope)
    lua.fns.yaw_trim_reset()
    lua.fns.yaw_trim_step(1e6, 0.15, 0.0)
    diagnostic = []
    for tick in np.arange(LUA_DT, 4, LUA_DT):
        a, g = np.interp(tick, servo_t, u), np.interp(tick, servo_t, gyro)
        trim = float(lua.fns.yaw_trim_step(LUA_DT, a, g))
        if round(tick / LUA_DT) % 5 == 0 and not 1.5 <= tick <= 1.9:
            diagnostic.append((tick, trim, a, g))
    servo = np.column_stack([servo_t, smin + span * u])[keep]
    attitude = np.column_stack([servo_t, gyro])[keep]
    return attitude, servo, np.asarray(diagnostic)


def test_observer_replay_recovers_the_logged_observer_and_servo_map():
    attitude, servo, diagnostic = _lua_recording(tau=0.6, slope=0.25, smin=1100, span=800)
    replay = replay_observer(attitude, servo, diagnostic, 8.6)
    assert replay.segments == 2
    assert replay.servo_min_us == pytest.approx(1100, abs=0.5)
    assert replay.servo_span_us == pytest.approx(800, abs=0.5)
    assert replay.gyro_alignment_rms_deg_s < 0.01
    assert replay.fitted_tau_s == pytest.approx(0.6, rel=0.05)
    assert replay.fitted_slope_rpm_per_us == pytest.approx(0.25, rel=0.05)
    assert replay.fitted_trim_rms < 0.1 * replay.default_trim_rms


def test_captured_parameters_override_the_model_and_respect_lua_slope_default(tmp_path: Path):
    (tmp_path / "parameters.json").write_text(json.dumps({"parameters": [
        {"name": "ATC_RAT_YAW_P", "type": {"type": "MAV_PARAM_TYPE_REAL32"}, "value": 0.18},
        {"name": "ATC_RAT_YAW_FLTE", "type": {"type": "MAV_PARAM_TYPE_REAL32"}, "value": 0.0},
        {"name": "RAWES_YAW_SLP", "type": {"type": "MAV_PARAM_TYPE_REAL32"}, "value": 0.0},
        {"name": "SERVO9_FUNCTION", "type": {"type": "MAV_PARAM_TYPE_INT8"}, "value": 36},
    ]}), encoding="utf-8")
    overrides = captured_parameters(tmp_path / "mavlog.json")
    assert overrides == {
        "yaw_rate_p": 0.18,
        "error_filter_hz": 0.0,
        "yaw_slope_rpm_per_us": DEFAULT_YAW_SLOPE_RPM_PER_US,
    }
    assert captured_parameters(tmp_path / "missing" / "mavlog.json") is None
