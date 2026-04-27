"""
aero_xi80_test.py -- Forces at xi=80 deg reel-in orientation across collective
and cyclic corners.

xi=80 deg: body_z rotated 80 deg from wind direction [0,1,0] toward zenith,
so body_z = [0, sin(10), -cos(10)] ~ [0, 0.174, -0.985] (nearly straight up).

"Cyclic on max in tether direction" = max tilt_lon (longitudinal swashplate tilt).
In this orientation the tether runs nearly straight down, so the tether direction
in body frame is approximately -body_z, and longitudinal tilt is in the tether
plane.

Outputs
-------
Section A: collective sweep at neutral cyclic (tilt_lon=0, tilt_lat=0)
Section B: 4 cyclic corners at a fixed collective

Usage
-----
    .venv/Scripts/python.exe simulation/analysis/aero_xi80_test.py
"""
import sys, math
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import rotor_definition as rd
from aero import create_aero

# ── Setup ──────────────────────────────────────────────────────────────────────

rotor  = rd.default()
aero   = create_aero(rotor)
MASS   = rotor.mass_kg
G      = 9.81
MG     = MASS * G
WIND   = np.array([0.0, 10.0, 0.0])   # NED: 10 m/s East

# Rotor spin from steady-state JSON
import json
_d     = json.loads((Path(__file__).resolve().parents[1] / "steady_state_starting.json").read_text())
OMEGA  = float(_d["omega_spin"])

COL_MIN = -0.28
COL_MAX =  0.10
TILT_MAX = 0.30   # rad -- typical max swashplate tilt

# ── Build R_hub at xi=80 deg from wind ────────────────────────────────────────
# Wind direction (NED): [0, 1, 0] (East)
# body_z rotated 80 deg from wind toward zenith:
#   zenith in NED = [0, 0, -1]
#   Rodrigues: rotate wind_dir by 80 deg around (wind_dir x zenith)
wind_dir = np.array([0.0, 1.0, 0.0])
zenith   = np.array([0.0, 0.0, -1.0])
ax       = np.cross(wind_dir, zenith)   # rotation axis
ax_len   = float(np.linalg.norm(ax))
if ax_len > 1e-9:
    ax = ax / ax_len

xi_rad   = math.radians(80.0)
c, s     = math.cos(xi_rad), math.sin(xi_rad)
# Rodrigues: v*cos + (ax x v)*sin + ax*(ax.v)*(1-cos)
bz  = wind_dir * c + np.cross(ax, wind_dir) * s + ax * (np.dot(ax, wind_dir)) * (1 - c)
bz  = bz / np.linalg.norm(bz)

# Body_x: pick a direction orthogonal to bz; use North [-1,0,0] projected
north = np.array([-1.0, 0.0, 0.0])
bx    = north - np.dot(north, bz) * bz
bx    = bx / np.linalg.norm(bx)
by    = np.cross(bz, bx)
by    = by / np.linalg.norm(by)

R_hub = np.column_stack([bx, by, bz])   # body-to-NED (cols = body axes in NED)

el_deg = math.degrees(math.asin(float(-bz[2])))   # elevation of body_z = xi from horizontal
print()
print("aero_xi80_test -- forces at xi=80 deg reel-in orientation")
print(f"  wind = {WIND} m/s NED")
print(f"  OMEGA = {OMEGA:.2f} rad/s")
print(f"  mass = {MASS:.2f} kg  mg = {MG:.2f} N")
print(f"  body_z (NED) = [{bz[0]:.3f}, {bz[1]:.3f}, {bz[2]:.3f}]")
print(f"  xi from horizontal = {el_deg:.1f} deg  (should be ~80 deg)")
print(f"  COL range: [{COL_MIN}, {COL_MAX}]  TILT_MAX: {TILT_MAX} rad")
print()


def run(col, tilt_lon, tilt_lat, label=""):
    res = aero.compute_forces(
        collective_rad=col,
        tilt_lon=tilt_lon,
        tilt_lat=tilt_lat,
        R_hub=R_hub,
        v_hub_world=np.zeros(3),
        omega_rotor=OMEGA,
        wind_world=WIND,
        t=45.0,
    )
    F  = res.F_world
    Fm = float(np.linalg.norm(F))
    # Project onto body_z (thrust along disk normal)
    F_along_bz  = float(np.dot(F, bz))
    # Component in wind direction (East)
    F_east  = float(F[1])
    # Component downward (NED Z positive = down; altitude loss)
    F_down  = float(F[2])
    # Upward force = -F_down
    F_up    = -F_down
    return res, Fm, F_along_bz, F_east, F_up


# ── Section A: collective sweep, neutral cyclic ────────────────────────────────
print("=" * 72)
print("A: Collective sweep -- neutral cyclic (tilt_lon=0, tilt_lat=0)")
print()
print(f"  {'col_rad':>8}  {'|F|_N':>8}  {'F_bz_N':>8}  "
      f"{'F_up_N':>8}  {'F_east_N':>9}  {'note':}")
print(f"  {'-'*8}  {'-'*8}  {'-'*8}  {'-'*8}  {'-'*9}  {'-'*20}")

cols_A = [COL_MIN, -0.24, -0.20, -0.18, -0.15, -0.10, -0.05, 0.0, 0.05, COL_MAX]
for col in cols_A:
    res, Fm, F_bz, F_e, F_up = run(col, 0.0, 0.0)
    note = ""
    if F_up < MG:
        note = "CANNOT HOLD ALTITUDE"
    elif col == COL_MIN:
        note = "<-- col_min"
    elif col == COL_MAX:
        note = "<-- col_max"
    print(f"  {col:>8.3f}  {Fm:>8.1f}  {F_bz:>8.1f}  "
          f"  {F_up:>8.1f}  {F_e:>9.1f}  {note}")

print()

# ── Section B: 4 cyclic corners at two collectives ────────────────────────────
print("=" * 72)
print("B: 4 cyclic corners -- tether-direction tilt at col_min and col_max")
print("   Corner = max tilt in one axis; tilt_lon = along tether/wind plane")
print("   tilt_lat = perpendicular to wind")
print()
print(f"  {'col':>6}  {'tlt_lon':>8}  {'tlt_lat':>8}  {'|F|_N':>8}  "
      f"{'F_bz_N':>8}  {'F_up_N':>8}  {'F_east_N':>9}  label")
print(f"  {'-'*6}  {'-'*8}  {'-'*8}  {'-'*8}  {'-'*8}  {'-'*8}  {'-'*9}  {'-'*20}")

corners = [
    ( 0.0,    0.0,    "neutral"),
    (+TILT_MAX, 0.0,  "+lon (into tether)"),
    (-TILT_MAX, 0.0,  "-lon (away)"),
    ( 0.0, +TILT_MAX, "+lat"),
    ( 0.0, -TILT_MAX, "-lat"),
]

for col_B in [COL_MIN, COL_MAX]:
    for tilt_lon, tilt_lat, lbl in corners:
        res, Fm, F_bz, F_e, F_up = run(col_B, tilt_lon, tilt_lat)
        alt_ok = "OK" if F_up >= MG else "CRASH"
        print(f"  {col_B:>6.3f}  {tilt_lon:>8.3f}  {tilt_lat:>8.3f}  {Fm:>8.1f}  "
              f"{F_bz:>8.1f}  {F_up:>8.1f}  {F_e:>9.1f}  {lbl}  [{alt_ok}]")
    print()

# ── Section C: tilt_lon sweep at col_min ──────────────────────────────────────
print("=" * 72)
print("C: tilt_lon sweep at col_min (-0.28) -- how does tether-dir tilt affect F_up?")
print()
print(f"  {'tlt_lon':>8}  {'|F|_N':>8}  {'F_up_N':>8}  {'F_east_N':>9}  alt_hold?")
print(f"  {'-'*8}  {'-'*8}  {'-'*8}  {'-'*9}  {'-'*10}")
for tilt_lon in [-0.30, -0.20, -0.10, 0.0, 0.10, 0.20, 0.30]:
    res, Fm, F_bz, F_e, F_up = run(COL_MIN, tilt_lon, 0.0)
    print(f"  {tilt_lon:>8.3f}  {Fm:>8.1f}  {F_up:>8.1f}  {F_e:>9.1f}  "
          f"{'YES' if F_up >= MG else 'NO'}")

print()
