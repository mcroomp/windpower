"""Benchmark cable evaluations independently of aero, controllers, and logging."""

import argparse
import cProfile
from dataclasses import replace
import io
import json
from pathlib import Path
import platform
import pstats
import sys
from time import perf_counter

import numpy as np
import scipy

from simulation.tether import (
    SpringDamperTether,
    TallakQuasiStaticTether,
    Tether,
    TetherInputs,
    TetherProperties,
)


def make_inputs(steps: int) -> tuple[TetherInputs, list[TetherInputs]]:
    properties = TetherProperties()
    direction = np.array([np.sqrt(3) / 2, 0.0, -0.5])
    baseline = TetherInputs(
        anchor_pos_ned=np.zeros(3),
        anchor_vel_ned=np.zeros(3),
        attachment_pos_ned=100.0 * direction,
        attachment_vel_ned=np.zeros(3),
        rest_length_m=100.0 * (1 - 300.0 / properties.axial_stiffness_n),
        wind_ned=np.array([0.0, 10.0, 0.0]),
    )
    trajectory = []
    for index in range(steps):
        time_s = index / 1200.0
        phase = 2 * np.pi * 0.2 * time_s
        radial_speed = 0.4 * np.cos(phase)
        displacement = 0.4 / (2 * np.pi * 0.2) * np.sin(phase)
        trajectory.append(replace(
            baseline,
            attachment_pos_ned=(100.0 + displacement) * direction,
            attachment_vel_ned=radial_speed * direction,
            rest_length_m=baseline.rest_length_m + displacement,
            payout_rate_ms=float(radial_speed),
            wind_ned=np.array([0.0, 10.0 + 2.0 * np.sin(phase), 0.0]),
        ))
    return baseline, trajectory


def benchmark(
    name: str, inputs: list[TetherInputs], *, segments: int = 16,
    cold: bool = False, spring: bool = False, repeats: int = 3,
) -> dict:
    rates = []
    durations = []
    iterations = []
    max_residual = 0.0
    max_balance_error = 0.0
    last_tension = 0.0
    for _ in range(repeats):
        model: Tether = (
            SpringDamperTether() if spring
            else TallakQuasiStaticTether(segments=segments)
        )
        if not cold:
            for _ in range(5):
                model.evaluate(inputs[0])
        started = perf_counter()
        for item in inputs:
            if cold:
                model = TallakQuasiStaticTether(segments=segments)
            before = perf_counter()
            result = model.evaluate(item)
            durations.append(perf_counter() - before)
            iterations.append(result.iterations)
            max_residual = max(max_residual, result.residual_m)
            max_balance_error = max(max_balance_error, float(np.linalg.norm(
                result.force_on_hub_ned + result.force_on_anchor_ned
                - result.distributed_force_ned,
            )))
            if not np.all(np.isfinite(result.positions_ned)):
                raise RuntimeError(f"{name}: nonfinite cable positions")
            if result.residual_m > 1e-6 or max_balance_error > 1e-8:
                raise RuntimeError(f"{name}: endpoint or force-balance failure")
            last_tension = result.hub_tension_n
        elapsed = perf_counter() - started
        rates.append(len(inputs) / elapsed)
    latency = np.asarray(durations) * 1000
    return {
        "name": name,
        "segments": None if spring else segments,
        "evaluations_per_repeat": len(inputs),
        "repeats": repeats,
        "evaluations_per_second_median": float(np.median(rates)),
        "evaluations_per_second_range": [min(rates), max(rates)],
        "evaluate_ms_median": float(np.median(latency)),
        "evaluate_ms_p95": float(np.percentile(latency, 95)),
        "evaluate_ms_max": float(np.max(latency)),
        "fraction_over_1200hz_budget": float(np.mean(latency > 1000 / 1200)),
        "outer_iterations_mean": float(np.mean(iterations)),
        "outer_iterations_max": max(iterations),
        "max_endpoint_residual_m": max_residual,
        "max_force_balance_error_n": max_balance_error,
        "last_hub_tension_n": last_tension,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int, default=1200)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument(
        "--output", type=Path, default=Path("simulation") / "logs" / "tether_profile",
    )
    args = parser.parse_args()
    if args.steps < 1 or args.repeats < 1:
        parser.error("--steps and --repeats must be positive")
    args.output.mkdir(parents=True, exist_ok=True)
    baseline, trajectory = make_inputs(args.steps)
    sagging = replace(
        baseline, attachment_pos_ned=np.array([80.0, 0.0, -40.0]),
        rest_length_m=100.0,
    )
    cases = [
        ("tallak_steady_16", [baseline] * args.steps, 16, False, False),
        ("tallak_moving_16", trajectory, 16, False, False),
        ("tallak_cold_16", trajectory[:min(args.steps, 120)], 16, True, False),
        ("tallak_sagging_16", [sagging] * args.steps, 16, False, False),
        ("tallak_moving_8", trajectory, 8, False, False),
        ("tallak_moving_32", trajectory, 32, False, False),
        ("spring_moving", trajectory, 16, False, True),
    ]
    results = []
    for name, items, segments, cold, spring in cases:
        result = benchmark(
            name, items, repeats=args.repeats, segments=segments,
            cold=cold, spring=spring,
        )
        results.append(result)
        print(
            f"{name:24s} {result['evaluations_per_second_median']:9.1f} eval/s"
            f"  median/p95 {result['evaluate_ms_median']:.3f}/"
            f"{result['evaluate_ms_p95']:.3f} ms"
            f"  outer {result['outer_iterations_mean']:.1f}/"
            f"{result['outer_iterations_max']}",
            flush=True,
        )
    model = TallakQuasiStaticTether()
    for _ in range(5):
        model.evaluate(trajectory[0])
    profiler = cProfile.Profile()
    profiler.enable()
    for item in trajectory:
        model.evaluate(item)
    profiler.disable()
    profiler.dump_stats(str(args.output / "moving_16.pstats"))
    stream = io.StringIO()
    stats = pstats.Stats(profiler, stream=stream).strip_dirs()
    stats.sort_stats("cumulative").print_stats(30)
    stats.sort_stats("tottime").print_stats(30)
    (args.output / "profile.txt").write_text(stream.getvalue(), encoding="utf-8")
    report = {
        "python": sys.version,
        "executable": sys.executable,
        "platform": platform.platform(),
        "numpy": np.__version__,
        "scipy": scipy.__version__,
        "method": (
            "Single-thread serial evaluations; prebuilt validated inputs; five warmups; "
            "GC enabled. Throughput includes timing and result-quality checks, "
            "excludes input construction, imports, warmup, and cProfile. "
            "Cold includes model construction. No full physics-loop timing."
        ),
        "scenario": (
            "100 m chord, 30 degree elevation, nominal 300 N preload, "
            "10 m/s transverse wind. Moving: 1200 Hz prescribed trajectory, "
            "0.2 Hz radial motion (peak 0.4 m/s), matched payout, "
            "wind 10 + 2*sin(phase) m/s. Sagging: [80,0,-40] m, 100 m rest length."
        ),
        "results": results,
    }
    (args.output / "benchmark.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8",
    )
    print(f"Results: {args.output.resolve()}", flush=True)


if __name__ == "__main__":
    main()
