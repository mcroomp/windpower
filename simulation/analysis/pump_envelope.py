"""
pump_envelope.py — Performance envelope for the RAWES pumping cycle.

Physics summary
---------------
Two actuators: collective (thrust magnitude) and body_z tilt (cyclic direction).
Two outputs:   tether tension T  and  hub altitude h.

At static equilibrium, the altitude constraint fixes a relationship between
body_z tilt angle delta (measured from tether direction toward zenith) and thrust:

    sin(delta) = mg*cos(el) / T_thrust        [altitude maintenance]
    T           = T_thrust*cos(delta) - mg*sin(el) [tether tension]

Eliminating T_thrust:

    T = mg * cos(el + delta) / sin(delta)     [combined formula]

So tension is controlled by delta alone (for altitude-maintaining flight).
Given a target tension T*:

    delta* = arctan( mg*cos(el) / (T* + mg*sin(el)) )
    T_thrust* = mg*cos(el) / sin(delta*)

Net energy per pumping cycle:
    E_net = T_out * v_out * t_out - T_in * v_in * t_in
    (positive means net generation; maximised by large T_out - T_in differential)

Usage
-----
    # Static envelope analysis only:
    .venv/Scripts/python.exe simulation/analysis/pump_envelope.py
    .venv/Scripts/python.exe simulation/analysis/pump_envelope.py --wind 8 12 15

    # Static envelope + actual vs optimal from a telemetry CSV:
    .venv/Scripts/python.exe simulation/analysis/pump_envelope.py \\
        --telemetry simulation/logs/test_pump_cycle/telemetry.csv
"""

import argparse
import sys
import math
import json
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from aero import create_aero
import rotor_definition as rd

# ── Current simtest parameters (shown as baseline in all tables) ──────────────
TENSION_OUT_NOW  = 435.0   # N  reel-out TensionPI setpoint
TENSION_IN_NOW   = 226.0   # N  reel-in TensionPI setpoint
V_REEL_OUT       = 0.4     # m/s
V_REEL_IN        = 0.4     # m/s
T_REEL_OUT       = 30.0    # s
T_REEL_IN        = 30.0    # s
XI_REEL_IN_NOW   = 80.0    # deg  (None = no tilt in Lua test)

COL_MIN          = -0.28   # rad
COL_MAX          =  0.10   # rad  (simtest override; TensionPI default is 0.00)
BREAK_LOAD_N     = 620.0   # N
SAFETY_LOAD_N    = 496.0   # N  (80% break load)

# ── Physics constants ─────────────────────────────────────────────────────────
rotor = rd.default()
MASS  = rotor.mass_kg
G     = 9.81
MG    = MASS * G

_IC = json.loads(
    (Path(__file__).resolve().parents[1] / "steady_state_starting.json").read_text()
)
IC_POS     = np.array(_IC["pos"])
IC_R0      = np.array(_IC["R0"]).reshape(3, 3)
OMEGA_SPIN = float(_IC["omega_spin"])

IC_TLEN = float(np.linalg.norm(IC_POS))
IC_EL   = math.degrees(math.asin(max(-1.0, min(1.0, -IC_POS[2] / IC_TLEN))))


# ── Core formulas ─────────────────────────────────────────────────────────────

def delta_for_tension(el_rad: float, target_t: float) -> float:
    return math.atan2(MG * math.cos(el_rad), target_t + MG * math.sin(el_rad))

def thrust_for_delta(el_rad: float, delta_rad: float) -> float:
    s = math.sin(delta_rad)
    return MG * math.cos(el_rad) / s if s > 1e-9 else float("inf")

def tension_from_delta(el_rad: float, delta_rad: float, t_thrust: float) -> float:
    return t_thrust * math.cos(delta_rad) - MG * math.sin(el_rad)

def t_min_at_elevation(el_rad: float, t_min_aero: float) -> "float | None":
    """Minimum achievable tether tension at given elevation and aero thrust floor."""
    if t_min_aero < MG * math.cos(el_rad):
        return None   # can't maintain altitude
    d_opt = math.asin(MG * math.cos(el_rad) / t_min_aero)
    return tension_from_delta(el_rad, d_opt, t_min_aero)


# ── Aero model queries ────────────────────────────────────────────────────────

def _make_aero(wind_ned: np.ndarray):
    return create_aero(rotor), wind_ned

def aero_thrust(col: float, wind_ned: np.ndarray, R_hub: np.ndarray = None) -> float:
    aero, w = _make_aero(wind_ned)
    R = IC_R0 if R_hub is None else R_hub
    res = aero.compute_forces(col, 0.0, 0.0, R, np.zeros(3), OMEGA_SPIN, w, t=45.0)
    return float(np.linalg.norm(res.F_world))

def col_for_thrust(target: float, wind_ned: np.ndarray, n: int = 40) -> "float | None":
    t_lo = aero_thrust(COL_MIN, wind_ned)
    t_hi = aero_thrust(COL_MAX, wind_ned)
    if target < t_lo or target > t_hi:
        return None
    lo, hi = COL_MIN, COL_MAX
    for _ in range(n):
        mid = (lo + hi) / 2
        if aero_thrust(mid, wind_ned) < target:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


# ── Section 1: formula validation ────────────────────────────────────────────

def validate_formula(wind_ned: np.ndarray) -> None:
    t_min_aero = aero_thrust(COL_MIN, wind_ned)
    t_max_aero = aero_thrust(COL_MAX, wind_ned)
    el = math.radians(IC_EL)

    print("=" * 72)
    print("FORMULA VALIDATION at IC conditions")
    print(f"  mass={MASS:.1f} kg  mg={MG:.1f} N  wind={np.linalg.norm(wind_ned):.1f} m/s")
    print(f"  IC: elevation={IC_EL:.1f} deg  altitude={-IC_POS[2]:.1f} m  tether={IC_TLEN:.1f} m")
    print(f"  Aero: T_thrust at COL_MIN={t_min_aero:.1f} N  COL_MAX={t_max_aero:.1f} N")
    print()
    print(f"  {'T_target_N':>12}  {'delta_deg':>10}  {'T_thrust_N':>12}  {'T_check_N':>11}")
    print(f"  {'-'*12}  {'-'*10}  {'-'*12}  {'-'*11}")
    for t_star in [55, 100, 150, 200, 226, 250, 300, 350, 400, 435, 496]:
        d   = delta_for_tension(el, float(t_star))
        thr = thrust_for_delta(el, d)
        chk = tension_from_delta(el, d, thr)
        cur = " <- TENSION_OUT" if t_star == 435 else (" <- TENSION_IN (now)" if t_star == 226 else "")
        print(f"  {t_star:>12.0f}  {math.degrees(d):>10.2f}  {thr:>12.1f}  {chk:>11.1f}  {cur}")
    print()


# ── Section 2: T_min across elevation range ───────────────────────────────────

def elevation_envelope(wind_ned: np.ndarray) -> None:
    t_min_aero = aero_thrust(COL_MIN, wind_ned)
    t_max_aero = aero_thrust(COL_MAX, wind_ned)

    print("=" * 72)
    print("PERFORMANCE ENVELOPE: achievable tension range per elevation")
    print(f"  COL range [{COL_MIN:+.2f}, {COL_MAX:+.2f}] rad  "
          f"omega={OMEGA_SPIN:.1f} rad/s  wind={np.linalg.norm(wind_ned):.1f} m/s")
    print()
    print(f"  {'el_deg':>7}  {'T_min_N':>8}  {'T_max_N':>8}  "
          f"{'col@435N':>9}  {'col@226N':>9}  {'col@100N':>9}  status")
    print(f"  {'-'*7}  {'-'*8}  {'-'*8}  {'-'*9}  {'-'*9}  {'-'*9}  {'-'*18}")

    for el_deg in [5, 10, 15, IC_EL, 20, 25, 30, 40, 50, 60, 70, 80]:
        el    = math.radians(el_deg)
        t_min = t_min_at_elevation(el, t_min_aero)
        if t_min is None:
            print(f"  {el_deg:>7.1f}  {'---':>8}  {'---':>8}  altitude fail")
            continue

        d_max  = math.asin(min(1.0, MG * math.cos(el) / t_max_aero)) if t_max_aero >= MG * math.cos(el) else 0.0
        t_max  = tension_from_delta(el, d_max, t_max_aero)

        def _col(t_target):
            d   = delta_for_tension(el, t_target)
            thr = thrust_for_delta(el, d)
            c   = col_for_thrust(thr, wind_ned)
            return f"{c:+.3f}" if c is not None else " infeas"

        status = ("BREAKS" if t_min > BREAK_LOAD_N
                  else "above safety" if t_min > SAFETY_LOAD_N
                  else "T>226 always" if t_min > 226.0
                  else "T>100 always" if t_min > 100.0
                  else "OK")
        marker = " <-- IC" if abs(el_deg - IC_EL) < 0.2 else ""
        print(f"  {el_deg:>7.1f}  {t_min:>8.1f}  {t_max:>8.1f}  "
              f"{_col(435.0):>9}  {_col(226.0):>9}  {_col(100.0):>9}  {status}{marker}")
    print()


# ── Section 3: net energy sweep over TENSION_IN ───────────────────────────────

def net_energy_sweep(wind_ned: np.ndarray) -> None:
    t_min_aero = aero_thrust(COL_MIN, wind_ned)
    el         = math.radians(IC_EL)
    t_min_abs  = t_min_at_elevation(el, t_min_aero)

    print("=" * 72)
    print("NET ENERGY SWEEP: effect of lowering TENSION_IN")
    print(f"  TENSION_OUT={TENSION_OUT_NOW:.0f} N  v_out={V_REEL_OUT} m/s  t_out={T_REEL_OUT:.0f} s")
    print(f"  v_in={V_REEL_IN} m/s  t_in={T_REEL_IN:.0f} s  elevation={IC_EL:.1f} deg")
    print(f"  Minimum achievable tension at COL_MIN: {t_min_abs:.1f} N" if t_min_abs else "  altitude fail")
    print()

    e_out = TENSION_OUT_NOW * V_REEL_OUT * T_REEL_OUT

    print(f"  {'T_in_N':>8}  {'col_needed':>11}  {'E_out_J':>8}  {'E_in_J':>8}  "
          f"{'E_net_J':>8}  {'vs_now':>8}  note")
    print(f"  {'-'*8}  {'-'*11}  {'-'*8}  {'-'*8}  {'-'*8}  {'-'*8}  {'-'*14}")

    e_net_now = None
    for t_in in [50, 75, 80, 90, 100, 110, 120, 150, 175, 200, 226, 250, 300]:
        # Is this tension achievable?
        if t_min_abs is not None and t_in < t_min_abs:
            note = "below T_min"
            d    = delta_for_tension(el, t_in)
            thr  = thrust_for_delta(el, d)
            c    = col_for_thrust(thr, wind_ned)
            col_s = f"{c:+.3f}" if c is not None else " infeas"
            print(f"  {t_in:>8.0f}  {col_s:>11}  {e_out:>8.0f}  {'---':>8}  {'---':>8}  {'---':>8}  {note}")
            continue

        d   = delta_for_tension(el, float(t_in))
        thr = thrust_for_delta(el, d)
        c   = col_for_thrust(thr, wind_ned)
        col_s = f"{c:+.3f}" if c is not None else " infeas"

        if c is None:
            print(f"  {t_in:>8.0f}  {col_s:>11}  {e_out:>8.0f}  {'---':>8}  {'---':>8}  {'---':>8}  infeasible")
            continue

        e_in  = float(t_in) * V_REEL_IN * T_REEL_IN
        e_net = e_out - e_in
        if t_in == 226:
            e_net_now = e_net

        vs = (f"{e_net - e_net_now:+.0f} J" if e_net_now is not None and t_in != 226
              else "  baseline" if t_in == 226 else "")
        note = "<-- current" if t_in == 226 else ("recommended" if t_in == 100 else "")
        print(f"  {t_in:>8.0f}  {col_s:>11}  {e_out:>8.0f}  {e_in:>8.0f}  "
              f"{e_net:>8.0f}  {vs:>8}  {note}")
    print()


# ── Section 4: wind speed sensitivity ────────────────────────────────────────

def wind_sensitivity(wind_speeds: "list[float]") -> None:
    print("=" * 72)
    print("WIND SENSITIVITY: T_min and optimal TENSION_IN vs wind speed")
    print(f"  (at IC elevation={IC_EL:.1f} deg, COL_MIN={COL_MIN:+.2f} rad)")
    print()
    print(f"  {'wind_ms':>8}  {'T_thrust_min':>13}  {'T_tether_min':>13}  "
          f"{'rec_T_in':>10}  {'E_net_J':>9}  note")
    print(f"  {'-'*8}  {'-'*13}  {'-'*13}  {'-'*10}  {'-'*9}  {'-'*16}")

    el = math.radians(IC_EL)
    for ws in wind_speeds:
        wind = np.array([0.0, ws, 0.0])
        t_thrust_min = aero_thrust(COL_MIN, wind)
        t_min = t_min_at_elevation(el, t_thrust_min)
        if t_min is None:
            print(f"  {ws:>8.1f}  {t_thrust_min:>13.1f}  {'altitude fail':>13}")
            continue

        # Recommended TENSION_IN: 20 N above T_min (margin for TensionPI overshoot)
        rec_t_in = math.ceil((t_min + 20.0) / 10) * 10.0

        d   = delta_for_tension(el, rec_t_in)
        thr = thrust_for_delta(el, d)
        c   = col_for_thrust(thr, wind)

        e_out = TENSION_OUT_NOW * V_REEL_OUT * T_REEL_OUT
        e_in  = rec_t_in * V_REEL_IN * T_REEL_IN
        e_net = e_out - e_in

        note = "<-- current sim" if abs(ws - 10.0) < 0.1 else ""
        col_s = f"{c:+.3f}" if c is not None else "infeas"
        print(f"  {ws:>8.1f}  {t_thrust_min:>13.1f}  {t_min:>13.1f}  "
              f"{rec_t_in:>10.0f}  {e_net:>9.0f}  {note}  col={col_s}")
    print()


# ── Section 5: reel-in tilt cost/benefit ──────────────────────────────────────

def tilt_analysis(wind_ned: np.ndarray) -> None:
    t_min_aero = aero_thrust(COL_MIN, wind_ned)
    slew_rate  = rotor.body_z_slew_rate_rad_s   # rad/s

    print("=" * 72)
    print("REEL-IN TILT COST/BENEFIT: is tilting to high xi worth the transition time?")
    print(f"  Slew rate={slew_rate:.2f} rad/s  v_reel_in={V_REEL_IN} m/s")
    print(f"  Baseline xi_start=30 deg (IC equilibrium disk angle)")
    print()
    print(f"  {'xi_deg':>7}  {'t_trans_s':>10}  {'T_min_N':>8}  {'DeltaT_vs_notilt':>18}  "
          f"{'tether_saved_m':>15}  {'E_saved_J':>10}  verdict")
    print(f"  {'-'*7}  {'-'*10}  {'-'*8}  {'-'*18}  {'-'*15}  {'-'*10}  {'-'*14}")

    xi_start_rad = math.radians(30.0)
    el = math.radians(IC_EL)
    t_min_notilt = t_min_at_elevation(el, t_min_aero)

    for xi_deg in [30, 40, 50, 60, 70, 80]:
        xi_rad    = math.radians(xi_deg)
        t_trans   = max(0.0, (xi_rad - xi_start_rad) / slew_rate) + 1.5
        t_min_xi  = t_min_at_elevation(xi_rad, t_min_aero)

        if t_min_notilt is None or t_min_xi is None:
            print(f"  {xi_deg:>7.0f}  altitude fail")
            continue

        delta_t   = t_min_notilt - t_min_xi             # tension reduction from tilting
        tether_saved = V_REEL_IN * t_trans               # tether NOT reeled in during transition
        # Energy saved by lower tension during T_REEL_IN after transition
        e_saved   = delta_t * V_REEL_IN * (T_REEL_IN - t_trans)

        verdict = ("current" if abs(xi_deg - XI_REEL_IN_NOW) < 1
                   else "recommended" if xi_deg == 30
                   else "")
        marker  = " <--" if abs(xi_deg - XI_REEL_IN_NOW) < 1 or xi_deg == 30 else ""
        print(f"  {xi_deg:>7.0f}  {t_trans:>10.1f}  {t_min_xi:>8.1f}  "
              f"{delta_t:>18.1f}  {tether_saved:>15.1f}  {e_saved:>10.0f}  {verdict}{marker}")
    print()
    print(f"  Conclusion: tilting from 30->80 deg saves only "
          f"{(t_min_notilt or 0) - (t_min_at_elevation(math.radians(80), t_min_aero) or 0):.1f} N "
          f"but costs {(math.radians(80)-xi_start_rad)/slew_rate+1.5:.0f} s of transition time.")
    print(f"  Removing the tilt (xi_reel_in_deg=None) is recommended for the new aero model.")
    print()


# ── Section 6: pump cycle telemetry analysis ──────────────────────────────────

def _parse_cycle_phase(phase_str: str) -> "tuple[int, str] | None":
    """Parse 'cycle1_reel-out' -> (1, 'reel-out'). None for non-pump phases."""
    if not phase_str.startswith("cycle"):
        return None
    parts = phase_str.split("_", 1)
    if len(parts) != 2:
        return None
    try:
        cycle = int(parts[0][5:])
    except ValueError:
        return None
    return cycle, parts[1]


def pump_cycle_report(csv_path: str) -> None:
    """
    Load a pump-cycle telemetry CSV and compare actual performance against the
    optimal envelope.  Outputs:
      - Per-cycle phase summary (tension tracking, altitude hold, energy)
      - Net energy per cycle vs theoretical maximum
      - Gap to optimal TENSION_IN setpoint
      - TensionPI saturation events
      - Altitude hold quality
    """
    from collections import defaultdict
    from telemetry_csv import read_csv

    rows = read_csv(csv_path)
    if not rows:
        print(f"  No rows in {csv_path}")
        return

    groups: "dict[tuple[int,str], list]" = defaultdict(list)
    for r in rows:
        cp = _parse_cycle_phase(r.phase)
        if cp is not None:
            groups[cp].append(r)

    if not groups:
        print("  No pump cycle phases found (expected 'cycleN_phase' labels).")
        return

    n_cycles = max(cp[0] for cp in groups)
    wind_ms  = float(np.linalg.norm([rows[0].wind_x, rows[0].wind_y, rows[0].wind_z]))
    pwind    = np.array([0.0, wind_ms, 0.0])

    t_min_aero   = aero_thrust(COL_MIN, pwind)
    el           = math.radians(IC_EL)
    t_min_tether = t_min_at_elevation(el, t_min_aero)

    print("=" * 72)
    print(f"PUMP CYCLE TELEMETRY ANALYSIS: {Path(csv_path).name}")
    print(f"  {n_cycles} cycle(s)  wind={wind_ms:.1f} m/s  elevation={IC_EL:.1f} deg")
    if t_min_tether is not None:
        print(f"  T_min at COL_MIN ({COL_MIN:+.2f} rad): {t_min_tether:.1f} N  "
              f"(aero floor {t_min_aero:.1f} N)")
    print()

    # ── Per-cycle phase summary ────────────────────────────────────────────────
    print(f"  {'cy':>2}  {'phase':>10}  {'t_start':>7}  {'t_end':>6}  "
          f"{'T_mean':>7}  {'T_sp':>6}  {'T_err':>6}  "
          f"{'col':>7}  {'alt_m':>6}  {'tgt_m':>6}  {'E_J':>7}")
    print(f"  {'--':>2}  {'-'*10}  {'-'*7}  {'-'*6}  "
          f"{'-'*7}  {'-'*6}  {'-'*6}  "
          f"{'-'*7}  {'-'*6}  {'-'*6}  {'-'*7}")

    cycle_e_out:  "dict[int, float]" = {}
    cycle_e_in:   "dict[int, float]" = {}
    pruned_groups: "dict[tuple[int,str], list]" = {}

    for cy in range(1, n_cycles + 1):
        for ph in ("reel-out", "hold", "reel-in"):
            grp = groups.get((cy, ph), [])
            if not grp:
                continue

            # Drop isolated rows (planner wrap-around artifacts at cycle boundaries).
            # Keep only the largest contiguous block where consecutive time gaps
            # are <= 2x the median step interval.
            tms = np.array([r.t_sim for r in grp])
            if len(tms) > 2:
                dt_steps = np.diff(tms)
                med_dt   = float(np.median(dt_steps))
                gaps     = np.where(dt_steps > 2.0 * med_dt)[0]
                if len(gaps):
                    blocks  = np.split(np.arange(len(grp)), gaps + 1)
                    biggest = max(blocks, key=len)
                    grp     = [grp[i] for i in biggest]

            pruned_groups[(cy, ph)] = grp

            tensions = np.array([r.tether_tension  for r in grp])
            colls    = np.array([r.collective_rad   for r in grp])
            alts     = np.array([-r.pos_z           for r in grp])
            tsps     = np.array([r.tension_setpoint for r in grp])
            tgts     = np.array([r.gnd_alt_cmd_m     for r in grp])

            # Energy via rest_length trapezoidal integration
            rls   = np.array([r.tether_rest_length for r in grp])
            drls  = np.abs(np.diff(rls))
            t_avg = 0.5 * (tensions[:-1] + tensions[1:])
            energy = float(np.sum(t_avg * drls))

            if ph == "reel-out":
                cycle_e_out[cy] = energy
            elif ph == "reel-in":
                cycle_e_in[cy] = energy

            t_mean   = float(tensions.mean())
            valid_sp = tsps[tsps > 0]
            sp_mean  = float(valid_sp.mean()) if len(valid_sp) else 0.0
            t_err    = t_mean - sp_mean if sp_mean > 0 else math.nan
            col_m    = float(colls.mean())
            alt_m    = float(alts.mean())
            valid_tgt = tgts[tgts > 0]
            tgt_m    = float(valid_tgt.mean()) if len(valid_tgt) else math.nan

            t_err_s = f"{t_err:+.1f}" if not math.isnan(t_err) else "  ---"
            tgt_s   = f"{tgt_m:.1f}"  if not math.isnan(tgt_m) else "  ---"

            print(f"  {cy:>2}  {ph:>10}  {grp[0].t_sim:>7.1f}  {grp[-1].t_sim:>6.1f}  "
                  f"{t_mean:>7.1f}  {sp_mean:>6.0f}  {t_err_s:>6}  "
                  f"{col_m:>+7.3f}  {alt_m:>6.1f}  {tgt_s:>6}  {energy:>7.0f}")
    print()

    # ── Net energy per cycle ───────────────────────────────────────────────────
    print(f"  NET ENERGY PER CYCLE")
    print(f"  {'cycle':>6}  {'E_out_J':>9}  {'E_in_J':>8}  {'E_net_J':>9}")
    print(f"  {'-'*6}  {'-'*9}  {'-'*8}  {'-'*9}")
    nets = []
    for cy in range(1, n_cycles + 1):
        eo = cycle_e_out.get(cy, 0.0)
        ei = cycle_e_in.get(cy, 0.0)
        en = eo - ei
        nets.append(en)
        print(f"  {cy:>6}  {eo:>9.0f}  {ei:>8.0f}  {en:>9.0f}")
    if nets:
        avg_net = float(np.mean(nets))
        print(f"  {'avg':>6}  {'---':>9}  {'---':>8}  {avg_net:>9.0f}")
    print()

    # ── vs optimal envelope ────────────────────────────────────────────────────
    if t_min_tether is not None and nets:
        rec_t_in = math.ceil((t_min_tether + 20.0) / 10.0) * 10.0

        all_tin_rows = []
        all_tout_rows = []
        for cy in range(1, n_cycles + 1):
            all_tin_rows.extend([r.tether_tension for r in groups.get((cy, "reel-in"),  [])])
            all_tout_rows.extend([r.tether_tension for r in groups.get((cy, "reel-out"), [])])
        actual_t_in  = float(np.mean(all_tin_rows))  if all_tin_rows  else math.nan
        actual_t_out = float(np.mean(all_tout_rows)) if all_tout_rows else math.nan

        # Actual reel-in energy vs what it would be at rec_t_in
        avg_e_in  = float(np.mean([cycle_e_in.get(cy, 0.0) for cy in range(1, n_cycles + 1)]))
        avg_e_out = float(np.mean([cycle_e_out.get(cy, 0.0) for cy in range(1, n_cycles + 1)]))
        # Theoretical E_in at rec_t_in uses same reel-in duration as actual run
        actual_rl_delta = sum(
            abs(groups[(cy, "reel-in")][-1].tether_rest_length
                - groups[(cy, "reel-in")][0].tether_rest_length)
            for cy in range(1, n_cycles + 1)
            if groups.get((cy, "reel-in"))
        ) / max(1, sum(1 for cy in range(1, n_cycles + 1) if groups.get((cy, "reel-in"))))
        e_in_opt  = rec_t_in * actual_rl_delta
        e_net_opt = avg_e_out - e_in_opt

        print(f"  VS OPTIMAL ENVELOPE")
        print(f"  T_min at COL_MIN:              {t_min_tether:.1f} N")
        if not math.isnan(actual_t_out):
            print(f"  Actual mean TENSION_OUT:       {actual_t_out:.1f} N")
        if not math.isnan(actual_t_in):
            print(f"  Actual mean TENSION_IN:        {actual_t_in:.1f} N  "
                  f"(+{actual_t_in - t_min_tether:.1f} N above T_min)")
        print(f"  Recommended TENSION_IN:        {rec_t_in:.0f} N  (T_min + 20 N margin)")
        print(f"  Theoretical E_net @ T_in={rec_t_in:.0f} N: {e_net_opt:.0f} J  "
              f"(using actual reel-in delta={actual_rl_delta:.1f} m)")
        gap = e_net_opt - avg_net
        pct = 100.0 * gap / avg_net if avg_net > 0 else math.nan
        pct_s = f"  (+{pct:.0f}% improvement)" if not math.isnan(pct) else ""
        print(f"  Actual avg E_net:              {avg_net:.0f} J")
        print(f"  Gap to optimal:                +{gap:.0f} J{pct_s}")
        print()

    # ── AP controller diagnostics (TensionApController) ───────────────────────
    print(f"  AP CONTROLLER DIAGNOSTICS (TensionApController + AcroControllerSitl)")
    print(f"  {'phase':>10}  {'n_rows':>7}  {'sat_%':>7}  note")
    print(f"  {'-'*10}  {'-'*7}  {'-'*7}  {'-'*36}")
    for ph in ("reel-out", "transition", "reel-in"):
        sat_vals = []
        for cy in range(1, n_cycles + 1):
            sat_vals.extend([r.coll_saturated for r in groups.get((cy, ph), [])])
        if not sat_vals:
            continue
        n       = len(sat_vals)
        pct_sat = 100.0 * sum(sat_vals) / n
        note = ""
        if ph == "reel-in"  and pct_sat > 50:
            note = "collective pinned: T_in limited by aero floor"
        elif ph == "reel-out" and pct_sat > 50:
            note = "collective pinned: T_out limited by aero ceiling"
        print(f"  {ph:>10}  {n:>7}  {pct_sat:>7.1f}  {note}")
    print()

    # ── Altitude hold quality ─────────────────────────────────────────────────
    print(f"  ALTITUDE HOLD QUALITY")
    print(f"  {'phase':>10}  {'mean_m':>7}  {'target_m':>10}  {'std_m':>7}  {'bias_m':>7}")
    print(f"  {'-'*10}  {'-'*7}  {'-'*10}  {'-'*7}  {'-'*7}")
    for ph in ("reel-out", "reel-in"):
        alts = []
        tgts = []
        for cy in range(1, n_cycles + 1):
            grp = pruned_groups.get((cy, ph), [])
            alts.extend([-r.pos_z      for r in grp])
            tgts.extend([r.gnd_alt_cmd_m for r in grp])
        if not alts:
            continue
        alts_arr = np.array(alts)
        tgts_arr = np.array(tgts)
        valid    = tgts_arr[tgts_arr > 0]
        tgt_m    = float(valid.mean()) if len(valid) else math.nan
        bias     = alts_arr.mean() - tgt_m if not math.isnan(tgt_m) else math.nan
        tgt_s    = f"{tgt_m:.1f}"  if not math.isnan(tgt_m) else "---"
        bias_s   = f"{bias:+.1f}" if not math.isnan(bias)   else "---"
        print(f"  {ph:>10}  {alts_arr.mean():>7.1f}  {tgt_s:>10}  "
              f"{alts_arr.std():>7.2f}  {bias_s:>7}")
    print()

    # ── Orbital trajectory + elevation tracking ───────────────────────────────

    def _r_horiz(r) -> float:
        return math.sqrt(r.pos_x**2 + r.pos_y**2)

    def _el_actual_deg(r) -> float:
        tlen = math.sqrt(r.pos_x**2 + r.pos_y**2 + r.pos_z**2)
        return math.degrees(math.asin(max(-1.0, min(1.0, -r.pos_z / tlen)))) if tlen > 0.1 else 0.0

    print(f"  ORBITAL TRAJECTORY + ELEVATION TRACKING")
    print(f"  {'cy':>2}  {'phase':>10}  {'r_s':>6}  {'r_e':>6}  {'dr_m':>6}  "
          f"{'dr/dt':>6}  {'el_s':>5}  {'el_e':>5}  {'el_tgt':>7}  {'el_err':>7}  note")
    print(f"  {'--':>2}  {'-'*10}  {'-'*6}  {'-'*6}  {'-'*6}  "
          f"{'-'*6}  {'-'*5}  {'-'*5}  {'-'*7}  {'-'*7}  {'-'*22}")

    for cy in range(1, n_cycles + 1):
        for ph in ("reel-out", "transition", "reel-in"):
            grp = pruned_groups.get((cy, ph), [])
            if len(grp) < 2:
                continue

            r0, r1   = grp[0], grp[-1]
            r_start  = _r_horiz(r0)
            r_end    = _r_horiz(r1)
            dr       = r_end - r_start
            duration = r1.t_sim - r0.t_sim
            dr_dt    = dr / duration if duration > 0.01 else math.nan

            el_s = _el_actual_deg(r0)
            el_e = _el_actual_deg(r1)

            # AP elevation target (from TensionApController.elevation_rad)
            el_tgt_vals = [math.degrees(r.elevation_rad) for r in grp if r.elevation_rad != 0.0]
            el_tgt      = float(np.mean(el_tgt_vals)) if el_tgt_vals else math.nan
            el_err      = float(np.mean([_el_actual_deg(r) - math.degrees(r.elevation_rad)
                                          for r in grp if r.elevation_rad != 0.0])) if el_tgt_vals else math.nan

            note = ""
            if ph == "reel-out" and dr > 0:
                note = "OK: hub expands"
            elif ph == "reel-out" and dr <= 0:
                note = "WARN: hub not expanding"
            elif ph == "reel-in" and dr < 0:
                note = "OK: hub retracts"
            elif ph == "reel-in" and dr >= 0:
                note = "WARN: hub not retracting"

            dr_s    = f"{dr:+.1f}"
            drdt_s  = f"{dr_dt:+.2f}" if not math.isnan(dr_dt) else "  ---"
            tgt_s   = f"{el_tgt:.1f}" if not math.isnan(el_tgt) else "  ---"
            err_s   = f"{el_err:+.2f}" if not math.isnan(el_err) else "  ---"

            print(f"  {cy:>2}  {ph:>10}  {r_start:>6.1f}  {r_end:>6.1f}  {dr_s:>6}  "
                  f"{drdt_s:>6}  {el_s:>5.1f}  {el_e:>5.1f}  {tgt_s:>7}  {err_s:>7}  {note}")
    print()

    # ── Daisy-chain elevation correction ──────────────────────────────────────
    has_correction = any(
        r.el_correction_rad != 0.0
        for grp in pruned_groups.values() for r in grp
    )
    if has_correction:
        print(f"  DAISY-CHAIN ELEVATION CORRECTION (el_corr_ki > 0)")
        print(f"  {'phase':>10}  {'mean_deg':>9}  {'max_deg':>8}  note")
        print(f"  {'-'*10}  {'-'*9}  {'-'*8}  {'-'*30}")
        for ph in ("reel-out", "reel-in"):
            corrs = []
            for cy in range(1, n_cycles + 1):
                corrs.extend([math.degrees(r.el_correction_rad)
                               for r in pruned_groups.get((cy, ph), [])])
            if not corrs:
                continue
            corr_arr = np.array(corrs)
            note = "tilting down to reduce tension" if corr_arr.mean() < -0.5 else ""
            print(f"  {ph:>10}  {corr_arr.mean():>+9.2f}  {corr_arr.min():>+8.2f}  {note}")
        print()
        print()


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(description="RAWES pumping cycle performance envelope")
    p.add_argument("--wind", type=float, nargs="+", default=[10.0],
                   metavar="W", help="Wind speed(s) in m/s for sensitivity analysis (default: 10)")
    p.add_argument("--telemetry", type=str, default=None,
                   metavar="CSV",
                   help="Path to a pump-cycle telemetry.csv — adds actual vs optimal analysis")
    args = p.parse_args()

    primary_wind = np.array([0.0, args.wind[0], 0.0])

    print()
    print("pump_envelope.py -- RAWES pumping cycle performance envelope")
    print(f"IC: elevation={IC_EL:.1f} deg  altitude={-IC_POS[2]:.1f} m  "
          f"tether={IC_TLEN:.1f} m  omega={OMEGA_SPIN:.1f} rad/s")
    print(f"Current simtest: TENSION_OUT={TENSION_OUT_NOW:.0f} N  "
          f"TENSION_IN={TENSION_IN_NOW:.0f} N  xi_reel_in={XI_REEL_IN_NOW:.0f} deg")
    print()

    validate_formula(primary_wind)
    elevation_envelope(primary_wind)
    net_energy_sweep(primary_wind)
    tilt_analysis(primary_wind)

    all_speeds = sorted(set(args.wind) | {6.0, 8.0, 10.0, 12.0, 15.0})
    wind_sensitivity(all_speeds)

    if args.telemetry:
        pump_cycle_report(args.telemetry)


if __name__ == "__main__":
    main()
