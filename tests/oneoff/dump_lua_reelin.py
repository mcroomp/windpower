"""dump_lua_reelin -- one-off diagnostic, not a unit test.

Print key telemetry columns around the cycle1 reel-in transition for the Lua
pumping run to locate which control term blows up.
"""
import csv
import sys
from pathlib import Path

csv_path = Path(r"c:\repos\windpower\simulation\logs\test_lua_pumping_unified\telemetry.csv")
rows = list(csv.DictReader(open(csv_path)))
cols = rows[0].keys()
# pick columns that exist
want = ["t_sim", "phase", "pos_z", "tether_tension", "collective_rad",
        "collective_from_alt_ctrl", "alt_pid_integral", "vib_corr",
        "winch_speed_ms", "tension_feedforward_n", "vel_z",
        "omega_x", "omega_y", "omega_z"]
present = [c for c in want if c in cols]
print("available extra cols:", [c for c in cols if c not in present][:30])
print("  ".join(present))
for r in rows:
    t = float(r["t_sim"])
    if 22.0 <= t <= 31.0:
        vals = []
        for c in present:
            v = r.get(c, "")
            try:
                vals.append(f"{float(v):9.3f}")
            except (ValueError, TypeError):
                vals.append(f"{str(v):>9.9}")
        print("  ".join(vals))
