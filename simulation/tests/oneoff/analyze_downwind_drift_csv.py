"""Downwind drift CSV analysis - one-off diagnostic, not a unit test."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


def _float(row: dict[str, str], name: str, default: float = 0.0) -> float:
    value = row.get(name, "")
    if value == "":
        return default
    return float(value)


def _phase_edges(rows: list[dict[str, str]]) -> list[int]:
    if not rows:
        return []
    edges = [0]
    prev = rows[0].get("phase", "")
    for idx, row in enumerate(rows[1:], start=1):
        phase = row.get("phase", "")
        if phase != prev:
            edges.append(idx)
            prev = phase
    if edges[-1] != len(rows) - 1:
        edges.append(len(rows) - 1)
    return edges


def _summary(path: Path) -> None:
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        print(f"{path}: empty")
        return

    # All current IC/pumping tests use WIND=[0, 10, 0], so downwind is +E and
    # crosswind is -N. If wind columns are present, compute the basis from row 0.
    wind_n = _float(rows[0], "wind_x", 0.0)
    wind_e = _float(rows[0], "wind_y", 10.0)
    wind_norm = math.hypot(wind_n, wind_e)
    if wind_norm <= 1e-9:
        down_n, down_e = 0.0, 1.0
    else:
        down_n, down_e = wind_n / wind_norm, wind_e / wind_norm
    cross_n, cross_e = -down_e, down_n

    samples = []
    for idx, row in enumerate(rows):
        north = _float(row, "pos_x")
        east = _float(row, "pos_y")
        downwind = north * down_n + east * down_e
        crosswind = north * cross_n + east * cross_e
        samples.append((idx, _float(row, "t_sim"), row.get("phase", ""), north, east, downwind, crosswind, -_float(row, "pos_z")))

    first = samples[0]
    last = samples[-1]
    max_abs = max(samples, key=lambda item: abs(item[6]))
    min_cross = min(samples, key=lambda item: item[6])
    max_cross = max(samples, key=lambda item: item[6])

    print(f"\n{path}")
    print(f"  rows={len(rows)}  wind_h=({wind_n:.3f}, {wind_e:.3f})  downwind basis=({down_n:.3f}, {down_e:.3f})")
    print(
        "  first: "
        f"t={first[1]:.2f}s phase={first[2]} N={first[3]:.2f} E={first[4]:.2f} "
        f"downwind={first[5]:.2f} crosswind={first[6]:.2f} alt={first[7]:.2f}"
    )
    print(
        "  last:  "
        f"t={last[1]:.2f}s phase={last[2]} N={last[3]:.2f} E={last[4]:.2f} "
        f"downwind={last[5]:.2f} crosswind={last[6]:.2f} alt={last[7]:.2f}"
    )
    print(
        "  crosswind: "
        f"min={min_cross[6]:.2f}m at t={min_cross[1]:.2f}s/{min_cross[2]}  "
        f"max={max_cross[6]:.2f}m at t={max_cross[1]:.2f}s/{max_cross[2]}  "
        f"max_abs={max_abs[6]:.2f}m at t={max_abs[1]:.2f}s/{max_abs[2]}"
    )

    print("  phase edges:")
    for idx in _phase_edges(rows):
        item = samples[idx]
        print(
            f"    i={idx:6d} t={item[1]:8.2f}s phase={item[2]:20s} "
            f"N={item[3]:8.2f} E={item[4]:8.2f} cross={item[6]:8.2f} alt={item[7]:7.2f}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="+", type=Path)
    args = parser.parse_args()
    for path in args.csv:
        _summary(path)


if __name__ == "__main__":
    main()