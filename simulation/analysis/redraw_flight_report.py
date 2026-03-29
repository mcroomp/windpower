#!/usr/bin/env python3
"""
redraw_flight_report.py — Redraw flight_report.png from saved run data.

Reads flight_data.json and (optionally) telemetry.csv written by
test_guided_flight after each run, then calls _plot_flight_report to
regenerate flight_report.png without re-running the full stack test.

Usage:
    python3 simulation/redraw_flight_report.py
    python3 simulation/redraw_flight_report.py --dir simulation
"""
import argparse
import json
import math
import sys
from pathlib import Path

_SIM_DIR = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description="Redraw RAWES flight report from saved data")
    parser.add_argument("--dir", default=str(_SIM_DIR),
                        help="Directory containing flight_data.json and telemetry.csv")
    parser.add_argument("--out", default=None,
                        help="Output PNG path (default: <dir>/flight_report.png)")
    args = parser.parse_args()

    data_dir = Path(args.dir)
    json_path = data_dir / "flight_data.json"
    telem_path = data_dir / "telemetry.csv"
    out_path = Path(args.out) if args.out else data_dir / "flight_report.png"

    if not json_path.exists():
        print(f"ERROR: {json_path} not found — run the stack test first to generate it.")
        sys.exit(1)

    with json_path.open(encoding="utf-8") as f:
        d = json.load(f)

    pos_history      = [tuple(r) for r in d["pos_history"]]
    attitude_history = [tuple(r) for r in d["attitude_history"]]
    servo_history    = [tuple(r) for r in d["servo_history"]]
    events           = d["events"]
    target           = tuple(d["target"])
    anchor_ned       = tuple(d["anchor_ned"]) if "anchor_ned" in d else None

    sys.path.insert(0, str(_SIM_DIR.parent))
    from flight_report import plot_flight_report

    plot_flight_report(
        pos_history      = pos_history,
        attitude_history = attitude_history,
        servo_history    = servo_history,
        events           = events,
        target           = target,
        out_path         = out_path,
        telemetry_path   = telem_path if telem_path.exists() else None,
        anchor_ned       = anchor_ned,
    )
    print(f"Flight report saved → {out_path}")


if __name__ == "__main__":
    main()
