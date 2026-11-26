"""Offline yaw hypotheses using the real Lua observer and Python AP PID port.

The plant and its excitation are assumptions, not an identified hardware model.
This tool must not be used to select or deploy flight-controller parameters.
"""

from __future__ import annotations

import argparse
import json
import math
from collections import deque
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Literal

import numpy as np
from scipy import optimize, signal

from arduloop.params import RateAxisParams
from arduloop.pid import AC_PID
from groundstation.rawes_diag import diag_values
from simulation.rawes_lua_harness import RawesLua
from simulation.torque_model import HubParams, HubState, step

DT = 0.0025
LUA_INTERVAL = 4
# rawes.lua constants: 100 Hz tick with a fixed dt, fixed slope-conversion span
# and bench-default slope used when RAWES_YAW_SLP is 0.
LUA_DT = 0.01
SERVO9_SPAN_US = 1000.0
DEFAULT_YAW_SLOPE_RPM_PER_US = 0.504
# Vehicle-clock spacing beyond which a 200 Hz stream is treated as a dropout.
MAX_STREAM_GAP_S = 0.03
PEAK_SEARCH_HZ = (3.0, 15.0)
# Transition skipped after an experiment.mark before its segment is analysed.
MARK_SETTLE_S = 1.0
Hypothesis = Literal["sensor", "torque"]


@dataclass(frozen=True)
class ObserverReplay:
    """The deployed Lua observer re-run on vehicle-timed servo and gyro data."""

    segments: int
    replay_seconds: float
    compared_samples: int
    servo_min_us: float
    servo_span_us: float
    servo_map_rms: float
    gyro_alignment_rms_deg_s: float
    gyro_alignment_lag_ms: float
    default_trim_rms: float
    fitted_tau_s: float
    fitted_slope_rpm_per_us: float
    fitted_trim_rms: float
    recorded_trim_amplitude: float
    replayed_trim_amplitude: float
    output_amplitude: float


@dataclass(frozen=True)
class Observation:
    start_s: float
    end_s: float
    yaw_std_deg_s: float
    p_std: float
    trim_mean: float
    trim_std: float
    output_std: float
    motor_rpm_mean: float
    motor_rpm_std: float
    pid_samples: int
    diagnostic_samples: int
    excluded_samples: int
    vehicle_mode: int
    phase_blocks: int
    output_relative_gyro_phase_deg: float | None
    stream_coverage_fraction: float
    diagnostic_interval_ms: float
    diagnostic_lost_fraction: float
    peak_hz: float | None
    gyro_peak_amplitude_deg_s: float | None
    replay: ObserverReplay | None
    replay_note: str | None


def harmonic_amplitude(t: np.ndarray, x: np.ndarray, frequency_hz: float) -> float:
    """Sinusoid amplitude at a known frequency for irregular, gapped samples."""
    t = np.asarray(t, dtype=float)
    x = np.asarray(x, dtype=float)
    omega = math.tau * frequency_hz
    basis = np.column_stack(
        [np.sin(omega * t), np.cos(omega * t), np.ones_like(t), t - t.mean()]
    )
    coefficients, *_ = np.linalg.lstsq(basis, x, rcond=None)
    return float(math.hypot(coefficients[0], coefficients[1]))


def peak_frequency(t: np.ndarray, x: np.ndarray) -> tuple[float, float]:
    """Frequency and amplitude of the strongest sinusoid in PEAK_SEARCH_HZ."""
    candidates = np.arange(PEAK_SEARCH_HZ[0], PEAK_SEARCH_HZ[1] + 1e-9, 0.02)
    amplitudes = [harmonic_amplitude(t, x, f) for f in candidates]
    best = int(np.argmax(amplitudes))
    return float(candidates[best]), float(amplitudes[best])


def _vehicle_series(samples: list[tuple[float, ...]], columns: int) -> np.ndarray:
    a = np.asarray(samples, dtype=float).reshape(-1, columns)
    if np.any(np.diff(a[:, 0]) < 0):
        raise ValueError("Vehicle clock moved backward or wrapped in an observation window")
    _, index = np.unique(a[:, 0], return_index=True)
    return a[index]


def _covered_spans(times: np.ndarray, max_gap: float) -> list[tuple[float, float]]:
    if len(times) < 2:
        return []
    breaks = np.flatnonzero(np.diff(times) > max_gap)
    starts = np.r_[0, breaks + 1]
    ends = np.r_[breaks, len(times) - 1]
    return [(float(times[s]), float(times[e])) for s, e in zip(starts, ends, strict=True) if e > s]


def _intersect(a: list[tuple[float, float]], b: list[tuple[float, float]]) -> list[tuple[float, float]]:
    spans = []
    for a0, a1 in a:
        for b0, b1 in b:
            start, end = max(a0, b0), min(a1, b1)
            if end > start:
                spans.append((start, end))
    return sorted(spans)


def _inside(t: np.ndarray, spans: list[tuple[float, float]]) -> np.ndarray:
    mask = np.zeros(len(t), dtype=bool)
    for start, end in spans:
        mask |= (t >= start) & (t <= end)
    return mask


def _replay_segments(
    tau_s: float,
    slope_rpm_per_us: float,
    segments: list[tuple[np.ndarray, np.ndarray, np.ndarray, float]],
) -> list[np.ndarray]:
    """Trim from the actual Lua observer for each (ticks, u, gyro, seed) segment."""
    lua = RawesLua(RAWES_YFF_TAU=tau_s, RAWES_YAW_SLP=slope_rpm_per_us)
    results = []
    for _, u, gyro, seed in segments:
        lua.fns.yaw_trim_reset()
        lua.fns.yaw_trim_step(1e6, seed, 0.0)
        results.append(np.array([
            float(lua.fns.yaw_trim_step(LUA_DT, float(a), float(b)))
            for a, b in zip(u, gyro, strict=True)
        ]))
    return results


def replay_observer(
    attitude: np.ndarray, servo: np.ndarray, diagnostic: np.ndarray, frequency_hz: float
) -> ObserverReplay:
    """Re-run rawes.lua's yaw_trim_step on vehicle-timed servo 9 PWM and gyro z.

    The servo PWM-to-throttle map is fitted from the recorded YFF_U readback,
    so it is the map Lua actually used. Each dropout-free span is seeded from
    the recorded YFF_T and compared with later YFF_T samples in that span.
    """
    attitude_spans = _covered_spans(attitude[:, 0], MAX_STREAM_GAP_S)
    servo_spans = _covered_spans(servo[:, 0], MAX_STREAM_GAP_S)
    spans = [s for s in _intersect(attitude_spans, servo_spans) if s[1] - s[0] >= 0.2]
    t_diag, trim, readback, gyro_used = diagnostic.T
    covered = _inside(t_diag, spans)
    pwm_at_diag = np.interp(t_diag, servo[:, 0], servo[:, 1])
    mapped = covered & (readback > 0.02) & (readback < 0.98)
    if mapped.sum() < 10:
        raise ValueError("Too few unclamped YFF_U samples inside covered spans")
    slope, offset = np.polyfit(pwm_at_diag[mapped], readback[mapped], 1)
    if slope <= 0:
        raise ValueError("YFF_U does not increase with servo 9 PWM")
    servo_map_rms = float(np.sqrt(np.mean(
        (slope * pwm_at_diag[mapped] + offset - readback[mapped]) ** 2
    )))
    lags = np.arange(-0.02, 0.02001, 0.0025)
    alignment = [
        float(np.sqrt(np.mean((
            np.interp(t_diag[covered] + lag, attitude[:, 0], attitude[:, 1]) - gyro_used[covered]
        ) ** 2)))
        for lag in lags
    ]
    best_lag = int(np.argmin(alignment))

    segments = []
    compare: list[tuple[np.ndarray, np.ndarray, np.ndarray]] = []
    for start, end in spans:
        inside = np.flatnonzero((t_diag >= start) & (t_diag <= end))
        if len(inside) < 3:
            continue
        seed_time = t_diag[inside[0]]
        ticks = np.arange(seed_time + LUA_DT, end, LUA_DT)
        if len(ticks) < 10:
            continue
        u = np.clip(slope * np.interp(ticks, servo[:, 0], servo[:, 1]) + offset, 0.0, 1.0)
        g = np.interp(ticks, attitude[:, 0], attitude[:, 1])
        segments.append((ticks, u, g, float(trim[inside[0]])))
        later = inside[1:][t_diag[inside[1:]] <= ticks[-1]]
        compare.append((ticks, t_diag[later], trim[later]))
    compared = sum(len(c[1]) for c in compare)
    if not segments or compared < 10:
        raise ValueError("Too few dropout-free spans with diagnostic samples to replay")

    def rms(tau_s: float, slope_rpm: float) -> float:
        replayed = _replay_segments(tau_s, slope_rpm, segments)
        errors = np.concatenate([
            np.interp(times, ticks, values) - recorded
            for (ticks, times, recorded), values in zip(compare, replayed, strict=True)
        ])
        return float(np.sqrt(np.mean(errors ** 2)))

    default_rms = rms(0.3, DEFAULT_YAW_SLOPE_RPM_PER_US)
    fit = optimize.minimize(
        lambda x: rms(math.exp(x[0]), math.exp(x[1])),
        x0=[math.log(0.3), math.log(DEFAULT_YAW_SLOPE_RPM_PER_US)],
        method="Nelder-Mead",
        options={"xatol": 1e-3, "fatol": 1e-7, "maxiter": 120},
    )
    tau_fit, slope_fit = (math.exp(v) for v in fit.x)
    replayed = _replay_segments(tau_fit, slope_fit, segments)
    all_ticks = np.concatenate([s[0] for s in segments])
    all_u = np.concatenate([s[1] for s in segments])
    return ObserverReplay(
        segments=len(segments),
        replay_seconds=float(len(all_ticks) * LUA_DT),
        compared_samples=compared,
        servo_min_us=float(-offset / slope),
        servo_span_us=float(1 / slope),
        servo_map_rms=servo_map_rms,
        gyro_alignment_rms_deg_s=float(np.degrees(alignment[best_lag])),
        gyro_alignment_lag_ms=float(lags[best_lag] * 1000),
        default_trim_rms=default_rms,
        fitted_tau_s=tau_fit,
        fitted_slope_rpm_per_us=slope_fit,
        fitted_trim_rms=float(fit.fun),
        recorded_trim_amplitude=harmonic_amplitude(
            t_diag[covered], trim[covered], frequency_hz
        ),
        replayed_trim_amplitude=harmonic_amplitude(
            all_ticks, np.concatenate(replayed), frequency_hz
        ),
        output_amplitude=harmonic_amplitude(all_ticks, all_u, frequency_hz),
    )


def observed_phase(
    attitude: list[tuple[float, float]], servo: list[tuple[float, float]]
) -> tuple[int, float | None]:
    """Estimate 8.33 Hz output/gyro phase using unbridged 0.6 s boot-clock blocks."""
    if len(attitude) < 2 or len(servo) < 2:
        return 0, None
    a, s = np.asarray(attitude), np.asarray(servo)
    if np.any(np.diff(a[:, 0]) < 0) or np.any(np.diff(s[:, 0]) < 0):
        raise ValueError("Source clock moved backward or wrapped in an observation window")
    _, ai = np.unique(a[:, 0], return_index=True)
    _, si = np.unique(s[:, 0], return_index=True)
    a, s = a[ai], s[si]
    if len(a) < 2 or len(s) < 2:
        return 0, None
    grid = np.arange(max(a[0, 0], s[0, 0]), min(a[-1, 0], s[-1, 0]), 0.02)

    def valid(t: np.ndarray) -> np.ndarray:
        index = np.searchsorted(t, grid).clip(1, len(t) - 1)
        return (t[index] - t[index - 1]) <= 0.1

    selected = np.flatnonzero(valid(a[:, 0]) & valid(s[:, 0]))
    if not len(selected):
        return 0, None
    spans = np.split(selected, np.flatnonzero(np.diff(selected) > 1) + 1)
    gyro = np.interp(grid, a[:, 0], a[:, 1])
    output = np.interp(grid, s[:, 0], s[:, 1])
    cross = []
    for span in spans:
        for offset in range(0, len(span) - 29, 30):
            part = span[offset:offset + 30]
            frequencies, spectrum = signal.csd(
                gyro[part], output[part], fs=50, window="hann", nperseg=30, detrend="linear"
            )
            cross.append(spectrum[int(np.argmin(abs(frequencies - 8.3333333333)))])
    if not cross or abs(np.mean(cross)) < 1e-15:
        return len(cross), None
    return len(cross), float(np.degrees(np.angle(np.mean(cross))))


def observations(path: Path, windows: list[tuple[float, float]]) -> list[Observation]:
    """Read selected receive-time windows; PID and diagnostics are not paired."""
    data = json.loads(path.read_bytes())
    if data["record_count"] != len(data["records"]):
        raise ValueError(f"{path}: record count mismatch")
    origin = data["experiment"]["started_at_ns"]
    collected: list[dict[str, list[float]]] = [
        {"yaw": [], "p": [], "trim": [], "output": [], "rpm": []} for _ in windows
    ]
    clocked: list[dict[str, list[tuple[float, ...]]]] = [
        {"attitude": [], "servo": [], "diagnostic": []} for _ in windows
    ]
    epochs: list[set[int]] = [set() for _ in windows]
    modes: list[set[int]] = [set() for _ in windows]
    excluded = [0 for _ in windows]
    heartbeat_time: float | None = None
    heartbeat_epoch: int | None = None
    heartbeat_mode: int | None = None
    armed = False
    for record in data["records"]:
        if record["kind"] != "mavlink.rx":
            continue
        message = record["data"]
        if (message["system_id"], message["component_id"]) != (1, 1):
            continue
        t = (record["ingest_time_ns"] - origin) / 1e9
        fields = message["fields"]
        name = message["message_name"]
        if name == "HEARTBEAT":
            heartbeat_time = t
            heartbeat_epoch = record["sim_clock"]["epoch"]
            heartbeat_mode = int(fields["custom_mode"])
            armed = "MAV_MODE_FLAG_SAFETY_ARMED" in fields["base_mode"].split(" | ")
        if name not in ("PID_TUNING", "DEBUG_FLOAT_ARRAY", "RPM",
                        "ATTITUDE_QUATERNION", "SERVO_OUTPUT_RAW"):
            continue
        for index, ((start, end), out, clocks, seen_epochs, seen_modes) in enumerate(zip(
            windows, collected, clocked, epochs, modes, strict=True
        )):
            if not start <= t < end:
                continue
            if (heartbeat_time is None or t - heartbeat_time > 3 or not armed
                    or record["sim_clock"]["epoch"] != heartbeat_epoch or heartbeat_mode is None):
                excluded[index] += 1
                continue
            seen_modes.add(heartbeat_mode)
            if name == "PID_TUNING" and fields["axis"]["type"] == "PID_TUNING_YAW":
                out["yaw"].append(float(fields["achieved"]))
                out["p"].append(float(fields["P"]))
            elif name == "DEBUG_FLOAT_ARRAY":
                values = diag_values(fields["array_id"], fields["data"])
                if "YFF_T" in values and "YFF_U" in values:
                    out["trim"].append(values["YFF_T"])
                    out["output"].append(values["YFF_U"])
                    if "YFF_GZ" in values:
                        # time_usec is rawes.lua's vehicle boot time (ms resolution).
                        clocks["diagnostic"].append((
                            fields["time_usec"] / 1e6,
                            values["YFF_T"], values["YFF_U"], values["YFF_GZ"],
                        ))
            elif name == "RPM" and fields["rpm1"] >= 0:
                out["rpm"].append(float(fields["rpm1"]))
            elif name == "ATTITUDE_QUATERNION":
                clocks["attitude"].append((fields["time_boot_ms"] / 1000, fields["yawspeed"]))
                seen_epochs.add(record["sim_clock"]["epoch"])
            elif name == "SERVO_OUTPUT_RAW" and fields["port"] == 0:
                clocks["servo"].append((fields["time_usec"] / 1e6, fields["servo9_raw"]))
                seen_epochs.add(record["sim_clock"]["epoch"])

    result = []
    for (start, end), out, clocks, seen_epochs, seen_modes, dropped in zip(
        windows, collected, clocked, epochs, modes, excluded, strict=True
    ):
        if end <= start or any(len(values) < 10 for values in out.values()):
            raise ValueError(f"Insufficient samples in {start}-{end} s")
        if not all(np.isfinite(values).all() for values in out.values()):
            raise ValueError(f"Non-finite telemetry in {start}-{end} s")
        if len(seen_epochs) > 1:
            raise ValueError(f"Multiple boot epochs in {start}-{end} s")
        if len(seen_modes) != 1:
            raise ValueError(f"Expected one armed vehicle mode in {start}-{end} s")
        blocks, phase = observed_phase(clocks["attitude"], clocks["servo"])
        attitude = _vehicle_series(clocks["attitude"], 2)
        servo = _vehicle_series(clocks["servo"], 2)
        diagnostic = _vehicle_series(clocks["diagnostic"], 4)
        coverage = sum(e - s for s, e in _intersect(
            _covered_spans(attitude[:, 0], MAX_STREAM_GAP_S),
            _covered_spans(servo[:, 0], MAX_STREAM_GAP_S),
        ))
        interval = float(np.median(np.diff(diagnostic[:, 0]))) if len(diagnostic) > 2 else math.nan
        if len(diagnostic) > 2:
            expected = (diagnostic[-1, 0] - diagnostic[0, 0]) / interval + 1
            lost = max(0.0, float(1 - len(diagnostic) / expected))
        else:
            lost = math.nan
        peak = peak_amplitude = None
        replay = None
        note = None
        if len(attitude) >= 200:
            peak, peak_amplitude = peak_frequency(attitude[:, 0], attitude[:, 1])
            try:
                replay = replay_observer(attitude, servo, diagnostic, peak)
            except ValueError as error:
                note = str(error)
        else:
            note = "Too few vehicle-timed attitude samples for spectrum and replay"
        result.append(Observation(
            start_s=start, end_s=end,
            yaw_std_deg_s=float(np.degrees(out["yaw"]).std()),
            p_std=float(np.std(out["p"])),
            trim_mean=float(np.mean(out["trim"])),
            trim_std=float(np.std(out["trim"])),
            output_std=float(np.std(out["output"])),
            motor_rpm_mean=float(np.mean(out["rpm"])),
            motor_rpm_std=float(np.std(out["rpm"])),
            pid_samples=len(out["yaw"]), diagnostic_samples=len(out["trim"]),
            excluded_samples=dropped, vehicle_mode=next(iter(seen_modes)),
            phase_blocks=blocks, output_relative_gyro_phase_deg=phase,
            stream_coverage_fraction=float(coverage / (end - start)),
            diagnostic_interval_ms=interval * 1000,
            diagnostic_lost_fraction=lost,
            peak_hz=peak,
            gyro_peak_amplitude_deg_s=(
                float(np.degrees(peak_amplitude)) if peak_amplitude is not None else None
            ),
            replay=replay, replay_note=note,
        ))
    return result


@dataclass(frozen=True)
class Configuration:
    trim_equilibrium: float
    motor_rpm: float
    gear_ratio: float = 10.88
    frequency_hz: float = 8.3
    delay_s: float = 0.005
    yaw_angle_p: float = 5.8
    yaw_rate_p: float = 0.12
    yaw_rate_i: float = 0.018
    yaw_rate_d: float = 0.0
    yaw_rate_ff: float = 0.0
    yaw_rate_imax: float = 1.0
    target_filter_hz: float = 20.0
    error_filter_hz: float = 15.0
    derivative_filter_hz: float = 10.0
    trim_tau_s: float = 0.3
    yaw_slope_rpm_per_us: float = DEFAULT_YAW_SLOPE_RPM_PER_US
    duration_s: float = 10.0
    settle_s: float = 5.0
    rotor_step_fraction: float = 0.0
    rotor_step_time_s: float = 3.0


# Captured vehicle parameters and the Configuration fields they set.
PARAMETER_FIELDS = {
    "ATC_ANG_YAW_P": "yaw_angle_p",
    "ATC_RAT_YAW_P": "yaw_rate_p",
    "ATC_RAT_YAW_I": "yaw_rate_i",
    "ATC_RAT_YAW_D": "yaw_rate_d",
    "ATC_RAT_YAW_FF": "yaw_rate_ff",
    "ATC_RAT_YAW_IMAX": "yaw_rate_imax",
    "ATC_RAT_YAW_FLTT": "target_filter_hz",
    "ATC_RAT_YAW_FLTE": "error_filter_hz",
    "ATC_RAT_YAW_FLTD": "derivative_filter_hz",
    "RAWES_YFF_TAU": "trim_tau_s",
    "RAWES_YAW_SLP": "yaw_slope_rpm_per_us",
}


def captured_parameters(experiment: Path) -> dict[str, float] | None:
    """Configuration overrides from the experiment's start snapshot, if any."""
    path = experiment.parent / "parameters.json"
    if not path.exists():
        return None
    values = {
        entry["name"]: float(entry["value"])
        for entry in json.loads(path.read_text(encoding="utf-8"))["parameters"]
    }
    overrides = {
        field: values[name] for name, field in PARAMETER_FIELDS.items() if name in values
    }
    # rawes.lua uses its bench default when RAWES_YAW_SLP is 0.
    if overrides.get("yaw_slope_rpm_per_us", 1.0) <= 0:
        overrides["yaw_slope_rpm_per_us"] = DEFAULT_YAW_SLOPE_RPM_PER_US
    return overrides


@dataclass(frozen=True)
class Strategy:
    rate_p: float
    trim_tau_s: float | None
    error_notch_hz: float = 0.0


@dataclass(frozen=True)
class Metrics:
    measured_yaw_std_deg_s: float
    actual_yaw_std_deg_s: float
    output_std: float
    trim_std: float
    p_std: float
    heading_std_deg: float
    heading_mean_deg: float
    trim_mean: float
    motor_rpm_std: float
    output_floor_fraction: float
    peak_hz: float | None
    output_relative_gyro_phase_deg: float | None


def simulate(config: Configuration, hypothesis: Hypothesis, amplitude: float, strategy: Strategy) -> Metrics:
    """Run one illustrative closed loop, assuming completed runup and linear DDFP."""
    if hypothesis not in ("sensor", "torque"):
        raise ValueError(f"Unknown hypothesis: {hypothesis}")
    values = (
        config.trim_equilibrium, config.motor_rpm, config.gear_ratio,
        config.frequency_hz, config.delay_s, config.yaw_angle_p,
        config.yaw_rate_p, config.yaw_rate_i, config.yaw_rate_d, config.yaw_rate_ff,
        config.yaw_rate_imax, config.target_filter_hz, config.error_filter_hz,
        config.derivative_filter_hz, config.trim_tau_s, config.yaw_slope_rpm_per_us,
        config.duration_s, config.settle_s, config.rotor_step_fraction,
        config.rotor_step_time_s, amplitude, strategy.rate_p,
        strategy.error_notch_hz,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Non-finite model configuration")
    if not (
        0 < config.trim_equilibrium < 1 and config.motor_rpm > 0
        and config.gear_ratio > 0 and 0 < config.frequency_hz < 1 / (2 * DT)
        and config.delay_s >= 0 and config.yaw_angle_p >= 0
        and config.yaw_rate_p >= 0 and config.yaw_rate_i >= 0 and config.error_filter_hz >= 0
        and config.yaw_rate_d >= 0 and config.yaw_rate_ff >= 0 and config.yaw_rate_imax > 0
        and config.target_filter_hz >= 0 and config.derivative_filter_hz >= 0
        and config.yaw_slope_rpm_per_us > 0
        and config.trim_tau_s > 0 and config.duration_s > config.settle_s >= 0
        and config.rotor_step_fraction > -1 and 0 <= config.rotor_step_time_s < config.duration_s
        and amplitude >= 0 and strategy.rate_p >= 0 and strategy.error_notch_hz >= 0
        and (strategy.trim_tau_s is None or (
            math.isfinite(strategy.trim_tau_s) and strategy.trim_tau_s > 0
        ))
    ):
        raise ValueError("Invalid model configuration")
    delay_steps = round(config.delay_s / DT)
    if not math.isclose(delay_steps * DT, config.delay_s, abs_tol=1e-9):
        raise ValueError("Delay must be a whole number of 2.5 ms steps")

    omega_motor = config.motor_rpm * math.tau / 60
    omega_rotor = omega_motor / config.gear_ratio
    plant = HubParams(
        gear_ratio=config.gear_ratio,
        rpm_scale=omega_motor / config.trim_equilibrium,
    )
    lua = RawesLua(
        RAWES_YFF_TAU=strategy.trim_tau_s or config.trim_tau_s,
        RAWES_YAW_SLP=config.yaw_slope_rpm_per_us,
    )
    lua.fns.yaw_trim_reset()
    lua.fns.yaw_trim_step(1e6, config.trim_equilibrium, 0)
    trim = config.trim_equilibrium
    params = RateAxisParams(
        P=strategy.rate_p, I=config.yaw_rate_i, D=config.yaw_rate_d, FF=config.yaw_rate_ff,
        IMAX=config.yaw_rate_imax, FLTT=config.target_filter_hz,
        FLTE=config.error_filter_hz, FLTD=config.derivative_filter_hz,
        NEF_center_hz=strategy.error_notch_hz,
        NEF_bandwidth_hz=4.0 if strategy.error_notch_hz else 0.0,
        NEF_attn_db=30.0,
    )
    pid = AC_PID(params, 1 / DT)
    state = HubState(psi=0, psi_dot=0.01, omega_motor=(omega_rotor + 0.01) * config.gear_ratio)
    history = deque([trim] * (delay_steps + 1), maxlen=delay_steps + 1)
    saturated = False
    samples: list[tuple[float, float, float, float, float, float, float]] = []
    for index in range(round(config.duration_s / DT)):
        t = index * DT
        disturbance = amplitude * math.sin(math.tau * config.frequency_hz * t)
        gyro = state.psi_dot + (disturbance if hypothesis == "sensor" else 0.0)
        target = -config.yaw_angle_p * state.psi
        pid_output = pid.update_all(target, gyro, DT, limit=saturated, sim_time_s=t)
        yaw_command = max(-1.0, min(1.0, pid_output))
        demand = trim + yaw_command
        saturated = demand <= 0.0 or demand >= 1.0
        output = max(0.0, min(1.0, demand))
        if index % LUA_INTERVAL == 0 and strategy.trim_tau_s is not None:
            trim = float(lua.fns.yaw_trim_step(LUA_INTERVAL * DT, output, gyro))
        history.append(output)
        rotor = omega_rotor * (1 + config.rotor_step_fraction if t >= config.rotor_step_time_s else 1)
        state = step(state, rotor, history[0], plant, DT)
        if hypothesis == "torque":
            # External hub torque, reflected through the assumed rigid gear.
            delta_motor = disturbance / config.gear_ratio * DT / plant.j_total()
            delta_rate = delta_motor / config.gear_ratio
            state = HubState(
                psi=state.psi + delta_rate * DT,
                psi_dot=state.psi_dot + delta_rate,
                omega_motor=state.omega_motor + delta_motor,
            )
        if t >= config.settle_s:
            samples.append((gyro, state.psi_dot, output, trim, pid.debug.P, state.psi, state.omega_motor))
    a = np.asarray(samples)
    if len(a) < 800 or not np.isfinite(a).all():
        raise ValueError("Simulation requires at least two seconds of finite post-settle samples")
    frequencies, power = signal.welch(a[:, 0], fs=1 / DT, nperseg=min(len(a), 1600), detrend="linear")
    mask = (frequencies >= 1) & (frequencies <= 15)
    peak_index = np.flatnonzero(mask)[np.argmax(power[mask])]
    peak = float(frequencies[peak_index]) if power[peak_index] > 1e-15 else None
    _, cross = signal.csd(a[:, 0], a[:, 2], fs=1 / DT, nperseg=min(len(a), 1600), detrend="linear")
    return Metrics(
        measured_yaw_std_deg_s=float(np.degrees(a[:, 0]).std()),
        actual_yaw_std_deg_s=float(np.degrees(a[:, 1]).std()),
        output_std=float(a[:, 2].std()),
        trim_std=float(a[:, 3].std()),
        p_std=float(a[:, 4].std()),
        heading_std_deg=float(np.degrees(a[:, 5]).std()),
        heading_mean_deg=float(np.degrees(a[:, 5]).mean()),
        trim_mean=float(a[:, 3].mean()),
        motor_rpm_std=float((a[:, 6] * 60 / math.tau).std()),
        output_floor_fraction=float(np.mean(a[:, 2] <= 0)),
        peak_hz=peak,
        output_relative_gyro_phase_deg=(
            float(np.degrees(np.angle(cross[peak_index])))
            if peak is not None and abs(cross[peak_index]) > 1e-15 else None
        ),
    )


def fit_amplitude(config: Configuration, hypothesis: Hypothesis, target_deg_s: float) -> float:
    """Fit one excitation amplitude to the calibration window's yaw std only."""
    if not math.isfinite(target_deg_s) or target_deg_s <= 0:
        raise ValueError("Calibration yaw std must be positive and finite")
    baseline = Strategy(config.yaw_rate_p, config.trim_tau_s)

    def residual(amplitude: float) -> float:
        return simulate(config, hypothesis, amplitude, baseline).measured_yaw_std_deg_s - target_deg_s

    if residual(0.0) >= 0:
        raise ValueError("Unforced model exceeds the observed calibration yaw variation")
    upper = 0.25
    while residual(upper) < 0 and upper < 16:
        upper *= 2
    if residual(upper) < 0:
        raise ValueError("Could not bracket an excitation amplitude")
    solution = optimize.root_scalar(residual, bracket=(0.0, upper), method="brentq", xtol=1e-5)
    if not solution.converged:
        raise ValueError(f"Excitation fit did not converge: {solution.flag}")
    return float(solution.root)


def configure(path: Path, calibration: Observation) -> tuple[Configuration, dict[str, str]]:
    """Model configuration and where each value came from."""
    defaults = Configuration(trim_equilibrium=calibration.trim_mean, motor_rpm=calibration.motor_rpm_mean)
    provenance = {name: "assumed" for name in asdict(defaults)}
    provenance |= {
        "trim_equilibrium": "calibration window mean YFF_T",
        "motor_rpm": "calibration window mean RPM",
    }
    overrides: dict[str, float] = {}
    if calibration.peak_hz is not None:
        overrides["frequency_hz"] = calibration.peak_hz
        provenance["frequency_hz"] = "calibration window vehicle-timed gyro peak"
    replay = calibration.replay
    if replay is not None:
        # Fitted observer values are used only when they clearly beat the
        # rawes.lua defaults; a marginal gain means they are not identified.
        if replay.fitted_trim_rms < 0.5 * replay.default_trim_rms:
            overrides["trim_tau_s"] = replay.fitted_tau_s
            overrides["yaw_slope_rpm_per_us"] = replay.fitted_slope_rpm_per_us
            source = "identified by replaying rawes.lua on the calibration window"
        else:
            source = "rawes.lua default, consistent with observer replay"
        provenance["trim_tau_s"] = provenance["yaw_slope_rpm_per_us"] = source
    captured = captured_parameters(path)
    if captured:
        overrides |= captured
        provenance |= {field: "captured parameters.json" for field in captured}
    return replace(defaults, **overrides), provenance


def study(path: Path, windows: list[tuple[float, float]]) -> dict:
    calibration, smooth, holdout = observations(path, windows)
    config, provenance = configure(path, calibration)
    strategies = {
        "baseline": Strategy(config.yaw_rate_p, config.trim_tau_s),
        "fixed_trim": Strategy(config.yaw_rate_p, None),
        "slow_trim": Strategy(config.yaw_rate_p, 2.0),
        "half_p": Strategy(config.yaw_rate_p / 2, config.trim_tau_s),
        "higher_p": Strategy(config.yaw_rate_p * 1.5, config.trim_tau_s),
        "error_notch_at_peak": Strategy(config.yaw_rate_p, config.trim_tau_s, config.frequency_hz),
    }
    hypotheses = {}
    for hypothesis in ("sensor", "torque"):
        amplitude = fit_amplitude(config, hypothesis, calibration.yaw_std_deg_s)
        smooth_config = replace(
            config, trim_equilibrium=smooth.trim_mean, motor_rpm=smooth.motor_rpm_mean
        )
        smooth_amplitude = fit_amplitude(smooth_config, hypothesis, smooth.yaw_std_deg_s)
        results = {
            name: asdict(simulate(config, hypothesis, amplitude, strategy))
            for name, strategy in strategies.items()
        }
        delay_sensitivity = {
            str(delay): {
                name: asdict(simulate(
                    replace(config, delay_s=delay), hypothesis, amplitude, strategies[name]
                ))
                for name in ("baseline", "half_p", "higher_p", "error_notch_at_peak")
            }
            for delay in (0.0, 0.005, 0.01, 0.015)
        }
        hypotheses[hypothesis] = {
            "fitted_excitation_amplitude": amplitude,
            "excitation_unit": "rad/s" if hypothesis == "sensor" else "N m at hub",
            "smooth_separately_fitted_amplitude": smooth_amplitude,
            "smooth_separately_fitted_baseline": asdict(simulate(
                smooth_config, hypothesis, smooth_amplitude, strategies["baseline"]
            )),
            "results": results,
            "delay_sensitivity_s": delay_sensitivity,
        }
    rotor_step = {
        name: asdict(simulate(
            replace(config, rotor_step_fraction=0.05), "torque", 0.0, strategies[name]
        ))
        for name in ("baseline", "fixed_trim", "slow_trim")
    }
    return {
        "warning": (
            "Exploratory hypotheses, not a validated hardware twin or a tuning recommendation. "
            "The excitation is fitted to the calibration yaw standard deviation; see "
            "configuration_provenance for which values are captured, identified or assumed. "
            "Smooth and rough excitation amplitudes are fitted independently; the model "
            "does not predict the dependence on collective."
        ),
        "experiment": str(path),
        "time_bases": {
            "window_selection_and_heartbeat_gating": "LinkHub receive time",
            "PID_TUNING": "LinkHub receive time (the message has no vehicle timestamp)",
            "DEBUG_FLOAT_ARRAY": "vehicle boot time_usec (ms resolution, set by rawes.lua)",
            "ATTITUDE_QUATERNION": "vehicle time_boot_ms",
            "SERVO_OUTPUT_RAW": "vehicle time_usec",
        },
        "observations": {
            "calibration_rough": asdict(calibration),
            "smooth_separately_fitted": asdict(smooth),
            "rough_holdout": asdict(holdout),
        },
        "configuration": asdict(config),
        "configuration_provenance": provenance,
        "implementation_assumptions": {
            "controller": (
                "Python AC_PID port with the configured P/I/D/FF/IMAX and FLTT/FLTE/FLTD "
                "(0 disables a filter, as in AP); no D_FF, slew or dynamic scaling; "
                "completed rotor runup"
            ),
            "mixer": "Active DDFP, linear normalized output, no collective-yaw offset or voltage compensation",
            "observer": (
                "Current Lua yaw_trim_step at 100 Hz with the configured RAWES_YFF_TAU and "
                "RAWES_YAW_SLP; adaptive or pinned"
            ),
            "plant": (
                "Existing torque_model rigid gear and governed motor with default inertia, "
                "ESC gain and torque limit; fixed rotor speed, inferred RPM/throttle ratio"
            ),
            "sample_time_s": DT,
            "sensor_hypothesis": "Additive sinusoid on measured gyro; heading uses true body angle",
            "torque_hypothesis": "Sinusoidal external torque on the hub; gyro measures true body rate",
        },
        "strategies": {name: asdict(value) for name, value in strategies.items()},
        "hypotheses": hypotheses,
        "rotor_speed_step_5_percent_no_periodic_forcing": rotor_step,
    }


def mark_windows(path: Path) -> dict[str, tuple[float, float]]:
    """Receive-time window after each `experiment.mark`, skipping its transition.

    A window runs from MARK_SETTLE_S after the mark to the next mark or the
    experiment's end. A repeated label keeps its last segment.
    """
    data = json.loads(path.read_bytes())
    origin = data["experiment"]["started_at_ns"]
    end = (data["experiment"]["stopped_at_ns"] - origin) / 1e9
    marks = [
        ((record["ingest_time_ns"] - origin) / 1e9, record["data"]["fields"]["label"])
        for record in data["records"]
        if record["kind"] == "diagnostic.event" and record["data"].get("event") == "experiment.mark"
    ]
    if not marks:
        return {}
    bounds = [time for time, _ in marks[1:]] + [end]
    return {
        label: (time + MARK_SETTLE_S, stop)
        for (time, label), stop in zip(marks, bounds, strict=True)
        if stop > time + MARK_SETTLE_S
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--experiment", required=True, type=Path, help="LinkHub experiment mavlog.json")
    parser.add_argument("--rough", nargs=2, type=float, default=(140, 146), metavar=("START", "END"))
    parser.add_argument("--smooth", nargs=2, type=float, default=(152, 156), metavar=("START", "END"))
    parser.add_argument("--holdout", nargs=2, type=float, default=(164, 166), metavar=("START", "END"))
    for name in ("rough", "smooth", "holdout"):
        parser.add_argument(
            f"--{name}-mark", metavar="LABEL",
            help=f"Use the segment after this experiment mark instead of --{name}",
        )
    parser.add_argument("--output", type=Path, help="Save JSON report; otherwise print it")
    args = parser.parse_args()
    segments = mark_windows(args.experiment)
    windows = []
    for name in ("rough", "smooth", "holdout"):
        label = getattr(args, f"{name}_mark")
        if label is None:
            start, end = getattr(args, name)
        elif label not in segments:
            parser.error(f"No experiment mark {label!r}; marks: {sorted(segments)}")
        else:
            start, end = segments[label]
        windows.append((start, end))
    report = study(args.experiment, windows)
    text = json.dumps(report, indent=2, allow_nan=False) + "\n"
    if args.output:
        args.output.write_text(text, encoding="utf-8")
    else:
        print(text, end="")


if __name__ == "__main__":
    main()
