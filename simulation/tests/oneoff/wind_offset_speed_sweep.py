"""wind offset speed sweep - one-off diagnostic, not a unit test."""
from __future__ import annotations

import csv
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SIM = ROOT / "simulation"
sys.path.insert(0, str(SIM / "tests" / "oneoff"))

from wind_offset_from_trim_probe import estimate_from_log


def main() -> None:
    speeds = [4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0]
    paths = [
        SIM / "logs" / "test_steady_flight" / "telemetry.csv",
        SIM / "logs" / "test_steady_flight_converges_from_30deg_wind_offset" / "telemetry.csv",
    ]
    out_path = SIM / "logs" / "wind_offset_from_trim_probe" / "speed_sweep_summary.csv"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    for speed in speeds:
        for path in paths:
            result = estimate_from_log(path, tail_rows=100, wind_speed_mps=speed, step_deg=2.0)
            best = result["best"]
            rows.append({
                "log": path.parent.name,
                "assumed_wind_speed_mps": speed,
                "truth_offset_deg": result["truth_offset_deg"],
                "best_offset_deg": best["candidate_offset_deg"],
                "cost_deg": best["cost_deg"],
                "pred_lon_deg": best["pred_tilt_lon_deg"],
                "pred_lat_deg": best["pred_tilt_lat_deg"],
                "obs_lon_deg": result["observed_tilt_lon_deg"],
                "obs_lat_deg": result["observed_tilt_lat_deg"],
            })
    with out_path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(out_path)
    print("log,speed,truth,best,cost,pred_lon,pred_lat,obs_lon,obs_lat")
    for row in rows:
        print(
            f"{row['log']},{row['assumed_wind_speed_mps']:.0f},"
            f"{row['truth_offset_deg']:+.2f},{row['best_offset_deg']:+.2f},"
            f"{row['cost_deg']:.3f},{row['pred_lon_deg']:+.3f},"
            f"{row['pred_lat_deg']:+.3f},{row['obs_lon_deg']:+.3f},"
            f"{row['obs_lat_deg']:+.3f}"
        )


if __name__ == "__main__":
    main()
