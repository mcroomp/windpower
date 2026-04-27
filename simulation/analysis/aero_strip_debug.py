"""
aero_strip_debug.py -- Step-by-step trace of one blade strip for hover case.

Runs the BEM strip calculation manually for one azimuth position and prints
every intermediate quantity so we can see where the sign diverges between
SkewedWakeBEM (gives downforce) and PetersHeBEM (gives upward force).

Usage:
    .venv/Scripts/python.exe simulation/analysis/aero_strip_debug.py
"""
import sys, math, json
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import rotor_definition as rd
from aero import create_aero

rotor = rd.default()
aero_skew = create_aero(rotor, model="skewed_wake_numpy")
aero_ph   = create_aero(rotor, model="peters_he_numpy")

MASS = rotor.mass_kg
G    = 9.81
d    = json.loads((Path(__file__).resolve().parents[1] / "steady_state_starting.json").read_text())
OMEGA = float(d["omega_spin"])

# Disk facing up: body_z = [0,0,-1] in NED
R_HOVER = np.array([[1,0,0],[0,-1,0],[0,0,-1]], dtype=float)
disk_normal = R_HOVER[:, 2]   # [0,0,-1]

print()
print("=" * 60)
print("Setup")
print(f"  R_HOVER body_z (disk_normal) = {disk_normal}")
print(f"  OMEGA = {OMEGA:.2f} rad/s")
print(f"  mass  = {MASS:.2f} kg,  mg = {MASS*G:.2f} N")

# ── Print rotor parameters (shared) ──────────────────────────────────────────
print()
print("Rotor blade parameters:")
for attr in ["CL0","CL_ALPHA","CD0","AR","OSWALD","pitch_gain_rad",
             "N_BLADES","R_TIP","R_ROOT","CHORD","RHO"]:
    if hasattr(aero_skew, attr):
        print(f"  {attr:20s} = {getattr(aero_skew, attr)}")

# ── Single-strip trace ────────────────────────────────────────────────────────
# phi = 0 (blade pointing North), r = R_CP (representative radius)
phi_az  = 0.0
ca, sa  = math.cos(phi_az), math.sin(phi_az)
r       = aero_skew.R_CP

COLLECTIVES = [-0.28, -0.18, 0.0, 0.10]

for col in COLLECTIVES:
    print()
    print("=" * 60)
    print(f"Collective = {col:.3f} rad,  phi_az = {phi_az:.1f} rad,  r = {r:.3f} m")
    print()

    # Blade vectors in body frame
    p_k  = col
    cp_k = math.cos(p_k)
    sp_k = math.sin(p_k)

    e_span_body   = np.array([ca,           sa,       0.0])
    e_chord_body  = np.array([-sa * cp_k,   ca * cp_k, sp_k])
    e_normal_body = np.array([ sa * sp_k,  -ca * sp_k, cp_k])

    e_span_world   = R_HOVER @ e_span_body
    e_chord_world  = R_HOVER @ e_chord_body
    e_normal_world = R_HOVER @ e_normal_body
    e_tang         = np.cross(disk_normal, e_span_world)

    print(f"  e_span_world  = {e_span_world.round(3)}")
    print(f"  e_chord_world = {e_chord_world.round(3)}")
    print(f"  e_normal_world= {e_normal_world.round(3)}")
    print(f"  e_tang        = {e_tang.round(3)}")

    v_rot = OMEGA * r * e_tang
    print(f"  v_rot         = {v_rot.round(3)}")

    # ── No induction first (like PetersHeBEM bootstrap v0=0) ─────────────────
    v_induced_0 = np.zeros(3)
    ua_0 = np.zeros(3) - v_rot - v_induced_0   # wind=0, v_hub=0
    ua_chord_0  = float(np.dot(ua_0, e_chord_world))
    ua_normal_0 = float(np.dot(ua_0, e_normal_world))
    alpha_0     = -ua_normal_0 / ua_chord_0 if abs(ua_chord_0) > 1e-6 else 0.0
    CL_0 = aero_skew.CL0 + aero_skew.CL_ALPHA * alpha_0
    CD_0 = aero_skew.CD0 + CL_0**2 / (math.pi * aero_skew.AR * aero_skew.OSWALD)
    ua_norm_0 = float(np.linalg.norm(ua_0))
    q_0  = 0.5 * aero_skew.RHO * ua_norm_0**2
    e_lift_raw_0 = np.cross(ua_0, e_span_world)
    e_lift_0 = e_lift_raw_0 / max(float(np.linalg.norm(e_lift_raw_0)), 1e-9)
    F_strip_0 = q_0 * (CL_0 * e_lift_0 + CD_0 * (ua_0 / ua_norm_0))

    print()
    print("  [v_i=0, no induction]")
    print(f"  ua_0          = {ua_0.round(3)}")
    print(f"  ua_chord      = {ua_chord_0:.4f}")
    print(f"  ua_normal     = {ua_normal_0:.4f}")
    print(f"  alpha_0       = {math.degrees(alpha_0):.2f} deg")
    print(f"  CL_0          = {CL_0:.4f}")
    print(f"  e_lift_0      = {e_lift_0.round(3)}")
    print(f"  F_strip_0     = {F_strip_0.round(2)}  (F_up = {-F_strip_0[2]:.2f} N per strip)")
    T0_sign = float(np.dot(F_strip_0, disk_normal))
    print(f"  F . disk_norm = {T0_sign:.4f}  (T0 > 0 means upward thrust)")

    # ── With induction from SkewedWakeBEM bootstrap ───────────────────────────
    # Run the actual bootstrap
    v_i0 = 0.0
    v_tan = OMEGA * aero_skew.R_CP
    for _ in range(3):
        v_loc = math.sqrt(v_tan**2 + (0 + v_i0)**2)
        if v_loc > 0.5:
            inflow = math.atan2(0 + v_i0, v_tan)
            aoa0   = max(-aero_skew.AOA_LIMIT, min(aero_skew.AOA_LIMIT, inflow + col))
            CL_b   = aero_skew.CL0 + aero_skew.CL_ALPHA * aoa0
            CD_b   = aero_skew.CD0 + CL_b**2 / (math.pi * aero_skew.AR * aero_skew.OSWALD)
            q0     = 0.5 * aero_skew.RHO * v_loc**2
            T_est  = max(0.0, aero_skew.N_BLADES * q0 * aero_skew.S_blade
                         * (CL_b * math.cos(inflow) - CD_b * math.sin(inflow)))
            disc   = 0.0**2 + 2.0 * max(abs(T_est), 0.01) / (aero_skew.RHO * aero_skew.disk_area)
            v_i0   = max(0.0, (-0.0 + math.sqrt(disc)) / 2.0)

    print()
    print(f"  [SkewedWakeBEM bootstrap: v_i0 = {v_i0:.4f} m/s]")

    v_induced_sw = v_i0 * disk_normal
    ua_sw = np.zeros(3) - v_rot - v_induced_sw
    ua_chord_sw  = float(np.dot(ua_sw, e_chord_world))
    ua_normal_sw = float(np.dot(ua_sw, e_normal_world))
    alpha_sw = -ua_normal_sw / ua_chord_sw if abs(ua_chord_sw) > 1e-6 else 0.0
    CL_sw = aero_skew.CL0 + aero_skew.CL_ALPHA * alpha_sw
    ua_norm_sw = float(np.linalg.norm(ua_sw))
    q_sw = 0.5 * aero_skew.RHO * ua_norm_sw**2
    e_lift_raw_sw = np.cross(ua_sw, e_span_world)
    e_lift_sw = e_lift_raw_sw / max(float(np.linalg.norm(e_lift_raw_sw)), 1e-9)
    F_strip_sw = q_sw * (CL_sw * e_lift_sw +
                  (aero_skew.CD0 + CL_sw**2/(math.pi*aero_skew.AR*aero_skew.OSWALD))
                  * (ua_sw / ua_norm_sw))

    print(f"  v_induced_sw  = {v_induced_sw.round(4)}")
    print(f"  ua_sw         = {ua_sw.round(3)}")
    print(f"  ua_chord_sw   = {ua_chord_sw:.4f}")
    print(f"  ua_normal_sw  = {ua_normal_sw:.4f}")
    print(f"  alpha_sw      = {math.degrees(alpha_sw):.2f} deg")
    print(f"  CL_sw         = {CL_sw:.4f}")
    print(f"  e_lift_sw     = {e_lift_sw.round(3)}")
    print(f"  F_strip_sw    = {F_strip_sw.round(2)}  (F_up = {-F_strip_sw[2]:.2f} N per strip)")

    # ── With PetersHeBEM induction: v0 from abs(T0) formula ──────────────────
    rho_A = aero_skew.RHO * aero_skew.disk_area
    T0_ph = T0_sign  # F_strip_0 . disk_normal at v0=0
    v0_ph = math.sqrt(max(abs(T0_ph), 0.1) / (2.0 * rho_A))

    print()
    print(f"  [PetersHeBEM: T0 = {T0_ph:.4f}, v0 = {v0_ph:.4f} m/s]")

    v_induced_ph = v0_ph * disk_normal
    ua_ph = np.zeros(3) - v_rot - v_induced_ph
    ua_chord_ph  = float(np.dot(ua_ph, e_chord_world))
    ua_normal_ph = float(np.dot(ua_ph, e_normal_world))
    alpha_ph = -ua_normal_ph / ua_chord_ph if abs(ua_chord_ph) > 1e-6 else 0.0
    CL_ph = aero_skew.CL0 + aero_skew.CL_ALPHA * alpha_ph
    ua_norm_ph = float(np.linalg.norm(ua_ph))
    q_ph = 0.5 * aero_skew.RHO * ua_norm_ph**2
    e_lift_raw_ph = np.cross(ua_ph, e_span_world)
    e_lift_ph = e_lift_raw_ph / max(float(np.linalg.norm(e_lift_raw_ph)), 1e-9)
    F_strip_ph = q_ph * (CL_ph * e_lift_ph +
                  (aero_skew.CD0 + CL_ph**2/(math.pi*aero_skew.AR*aero_skew.OSWALD))
                  * (ua_ph / ua_norm_ph))

    print(f"  v_induced_ph  = {v_induced_ph.round(4)}")
    print(f"  ua_ph         = {ua_ph.round(3)}")
    print(f"  ua_chord_ph   = {ua_chord_ph:.4f}")
    print(f"  ua_normal_ph  = {ua_normal_ph:.4f}")
    print(f"  alpha_ph      = {math.degrees(alpha_ph):.2f} deg")
    print(f"  CL_ph         = {CL_ph:.4f}")
    print(f"  e_lift_ph     = {e_lift_ph.round(3)}")
    print(f"  F_strip_ph    = {F_strip_ph.round(2)}  (F_up = {-F_strip_ph[2]:.2f} N per strip)")

print()
