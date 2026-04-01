#!/usr/bin/env python3
"""
analyse_pumping_cycle.py — Structured analysis of a pumping cycle stack test run.

Reads the mediator telemetry CSV produced by test_pumping_cycle and prints a
structured report covering:

  • Phase overview     — timing, duration, row counts per phase
  • Altitude          — min/max altitude (NED physics, not drifted EKF)
  • Tether            — length, extension, rest_length progression
  • Tension           — mean/max per phase, spike detection, energy
  • Net energy        — ∫ T·v_reel dt per phase, net = out − in
  • Anomalies         — sustained high tension, altitude floor, tether slack

Usage (from repo root):
  wsl.exe bash -c 'python3 /mnt/e/repos/windpower/simulation/analysis/analyse_pumping_cycle.py'
  wsl.exe bash -c 'python3 /mnt/e/repos/windpower/simulation/analysis/analyse_pumping_cycle.py --csv /path/to/telemetry.csv'
  python3 simulation/analysis/analyse_pumping_cycle.py --csv simulation/logs/telemetry_pumping.csv
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from statistics import mean, stdev
from typing import Optional

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------

_SIM_DIR     = Path(__file__).resolve().parents[1]
_DEFAULT_CSV = _SIM_DIR / "logs" / "telemetry_pumping.csv"

# Thresholds (match test_pumping_cycle.py)
_MIN_ALT_M       = 0.5     # m — crash floor
_BREAK_LOAD_N    = 620.0   # N — Dyneema SK75 1.9 mm
_TENSION_LIMIT_N = 0.8 * _BREAK_LOAD_N  # 496 N — 80% break load


# ---------------------------------------------------------------------------
# Telemetry parsing
# ---------------------------------------------------------------------------

@dataclass
class TelRow:
    t:                   float
    hub_pos_x:           float = 0.0
    hub_pos_y:           float = 0.0
    hub_pos_z:           float = 0.0    # NED Z; altitude = -hub_pos_z
    hub_vel_x:           float = 0.0
    hub_vel_y:           float = 0.0
    hub_vel_z:           float = 0.0
    tether_length:       float = 0.0
    tether_extension:    float = 0.0
    tether_tension:      float = 0.0
    tether_rest_length:  float = 0.0
    tether_slack:        int   = 1
    collective_rad:      float = 0.0
    collective_norm:     float = 0.0
    pumping_phase:       str   = ""     # "reel-out" | "reel-in" | ""
    tension_setpoint:    float = 0.0
    collective_from_tension_ctrl: float = 0.0
    omega_rotor:         float = 0.0
    aero_T:              float = 0.0

    @property
    def altitude(self) -> float:
        """Hub altitude above anchor [m] = -NED_Z."""
        return -self.hub_pos_z

    @property
    def orbit_radius(self) -> float:
        """Horizontal distance from anchor [m]."""
        return math.sqrt(self.hub_pos_x**2 + self.hub_pos_y**2)


def _try_float(s: str) -> Optional[float]:
    if s in ("", "None", "nan", "inf", "-inf"):
        return None
    try:
        return float(s)
    except ValueError:
        return None


def load_telemetry(path: Path) -> list[TelRow]:
    """Load telemetry CSV into a list of TelRow objects."""
    rows: list[TelRow] = []
    if not path.exists():
        return rows

    with path.open(encoding="utf-8", errors="replace") as f:
        reader = csv.DictReader(f)
        for raw in reader:
            t = _try_float(raw.get("t_sim", ""))
            if t is None:
                continue
            r = TelRow(t=t)
            for fld in (
                "hub_pos_x", "hub_pos_y", "hub_pos_z",
                "hub_vel_x", "hub_vel_y", "hub_vel_z",
                "tether_length", "tether_extension", "tether_tension",
                "tether_rest_length", "collective_rad", "collective_norm",
                "tension_setpoint", "collective_from_tension_ctrl",
                "omega_rotor", "aero_T",
            ):
                v = _try_float(raw.get(fld, ""))
                if v is not None:
                    setattr(r, fld, v)
            # tether_slack is an integer flag
            v_slack = _try_float(raw.get("tether_slack", ""))
            if v_slack is not None:
                r.tether_slack = int(v_slack)
            # pumping_phase is a string label
            r.pumping_phase = raw.get("pumping_phase", "").strip()
            rows.append(r)
    return rows


# ---------------------------------------------------------------------------
# Phase splitting
# ---------------------------------------------------------------------------

@dataclass
class PumpingCycle:
    """One complete reel-out + reel-in pair."""
    index:    int
    out_rows: list[TelRow]
    in_rows:  list[TelRow]

    @property
    def t_start(self) -> float:
        return self.out_rows[0].t if self.out_rows else 0.0

    @property
    def t_reel_in_start(self) -> float:
        return self.in_rows[0].t if self.in_rows else 0.0

    @property
    def t_end(self) -> float:
        return self.in_rows[-1].t if self.in_rows else (
            self.out_rows[-1].t if self.out_rows else 0.0)

    @property
    def complete(self) -> bool:
        return bool(self.out_rows) and bool(self.in_rows)


def split_cycles(rows: list[TelRow]) -> list[PumpingCycle]:
    """
    Detect all pumping cycles in telemetry, returning one PumpingCycle per
    reel-out + reel-in pair.  A cycle is started by the first "reel-out" row
    after a non-reel-out row (or the start of data).  Partial cycles at the
    end (e.g. a reel-out with no following reel-in) are included but marked
    incomplete via PumpingCycle.complete = False.
    """
    cycles: list[PumpingCycle] = []
    current_out: list[TelRow] = []
    current_in:  list[TelRow] = []
    last_phase = ""
    cycle_idx  = 0

    for r in rows:
        phase = r.pumping_phase  # "reel-out" | "reel-in" | ""

        if phase == "reel-out":
            if last_phase == "reel-in":
                # Transition from reel-in → reel-out: previous cycle is complete
                cycles.append(PumpingCycle(cycle_idx, current_out, current_in))
                cycle_idx  += 1
                current_out = []
                current_in  = []
            current_out.append(r)

        elif phase == "reel-in":
            current_in.append(r)

        elif last_phase in ("reel-out", "reel-in"):
            # Phase ended (transition to idle): save current cycle
            if current_out or current_in:
                cycles.append(PumpingCycle(cycle_idx, current_out, current_in))
                cycle_idx  += 1
                current_out = []
                current_in  = []

        last_phase = phase if phase else last_phase

    # Flush anything remaining at end of data
    if current_out or current_in:
        cycles.append(PumpingCycle(cycle_idx, current_out, current_in))

    return cycles


def split_phases(rows: list[TelRow]) -> dict[str, list[TelRow]]:
    """
    Aggregate all reel-out and reel-in rows across ALL cycles.
    Use split_cycles() for per-cycle analysis.
    """
    phases: dict[str, list[TelRow]] = {"": [], "reel-out": [], "reel-in": []}
    for r in rows:
        label = r.pumping_phase if r.pumping_phase in ("reel-out", "reel-in") else ""
        phases[label].append(r)
    return phases


# ---------------------------------------------------------------------------
# Statistics helpers
# ---------------------------------------------------------------------------

def _stats(vals: list[float]) -> str:
    if not vals:
        return "  (no data)"
    mn, mx, mu = min(vals), max(vals), mean(vals)
    sd = stdev(vals) if len(vals) > 1 else 0.0
    return f"  min={mn:.2f}  max={mx:.2f}  mean={mu:.2f}  sd={sd:.2f}"


def _integrate(vals: list[float], times: list[float]) -> float:
    """Trapezoidal integration."""
    result = 0.0
    for i in range(1, len(vals)):
        result += 0.5 * (vals[i - 1] + vals[i]) * (times[i] - times[i - 1])
    return result


# ---------------------------------------------------------------------------
# Spike detection
# ---------------------------------------------------------------------------

@dataclass
class TensionSpike:
    t_start:  float
    t_end:    float
    peak:     float
    duration: float

    def __str__(self) -> str:
        return (f"t={self.t_start:.2f}–{self.t_end:.2f}s  "
                f"peak={self.peak:.0f}N  dur={self.duration:.3f}s")


def find_tension_spikes(rows: list[TelRow], threshold: float = 500.0,
                        min_gap_s: float = 0.5) -> list[TensionSpike]:
    """Find continuous intervals where tension > threshold."""
    spikes: list[TensionSpike] = []
    in_spike = False
    t0 = peak = 0.0

    for r in rows:
        if r.tether_tension > threshold:
            if not in_spike:
                t0 = r.t
                peak = r.tether_tension
                in_spike = True
            else:
                peak = max(peak, r.tether_tension)
        else:
            if in_spike:
                dur = r.t - t0
                if dur >= 0.005:   # ignore sub-timestep artifacts
                    spikes.append(TensionSpike(t0, r.t, peak, dur))
                in_spike = False

    if in_spike and rows:
        dur = rows[-1].t - t0
        spikes.append(TensionSpike(t0, rows[-1].t, peak, dur))

    # Merge spikes separated by < min_gap_s
    merged: list[TensionSpike] = []
    for sp in spikes:
        if merged and sp.t_start - merged[-1].t_end < min_gap_s:
            merged[-1].t_end  = sp.t_end
            merged[-1].peak   = max(merged[-1].peak, sp.peak)
            merged[-1].duration = merged[-1].t_end - merged[-1].t_start
        else:
            merged.append(sp)
    return merged


# ---------------------------------------------------------------------------
# Main report
# ---------------------------------------------------------------------------

def _section(title: str) -> None:
    print(f"\n{'-' * 60}")
    print(f"  {title}")
    print('-' * 60)


def _pass_fail(ok: bool, msg: str) -> str:
    return ("  [PASS]  " if ok else "  [FAIL]  ") + msg


def _phase_detail(rows: list[TelRow]) -> dict:
    """Extract per-row data for a phase as lists suitable for JSON output."""
    return {
        "t":                   [r.t                  for r in rows],
        "altitude_m":          [-r.hub_pos_z          for r in rows],
        "tether_length_m":     [r.tether_length       for r in rows],
        "tether_extension_m":  [r.tether_extension    for r in rows],
        "tether_rest_length_m":[r.tether_rest_length  for r in rows],
        "tether_tension_n":    [r.tether_tension       for r in rows],
        "tension_setpoint_n":  [r.tension_setpoint    for r in rows],
        "collective_rad":      [r.collective_rad       for r in rows],
        "collective_ctrl":     [r.collective_from_tension_ctrl for r in rows],
        "orbit_radius_m":      [r.orbit_radius        for r in rows],
    }


def analyse(rows: list[TelRow]) -> dict:
    """
    Compute all metrics. Returns a dict of results for programmatic use.
    Also prints a human-readable report to stdout.
    """
    results: dict = {}

    # ── Overall ──────────────────────────────────────────────────────────
    _section("OVERVIEW")
    if not rows:
        print("  No telemetry rows loaded.")
        return results

    t_total = rows[-1].t - rows[0].t
    print(f"  rows: {len(rows):,}   t: {rows[0].t:.1f}–{rows[-1].t:.1f}s  "
          f"(duration={t_total:.1f}s)")

    phases = split_phases(rows)
    for label in ("reel-out", "reel-in"):
        ph = phases[label]
        if ph:
            dt = ph[-1].t - ph[0].t
            print(f"  {label:<10}: {len(ph):>6} rows  "
                  f"t={ph[0].t:.1f}–{ph[-1].t:.1f}s  ({dt:.1f}s)")
        else:
            print(f"  {label:<10}: (no rows)")

    results["t_start"] = rows[0].t
    results["t_end"]   = rows[-1].t

    # ── Altitude ─────────────────────────────────────────────────────────
    _section("ALTITUDE  (physics NED, altitude = -hub_pos_z)")
    alts = [-r.hub_pos_z for r in rows]
    min_alt = min(alts)
    max_alt = max(alts)
    print(f"  All phases:    {_stats(alts)}")
    for label in ("reel-out", "reel-in"):
        ph = phases[label]
        if ph:
            a = [-r.hub_pos_z for r in ph]
            print(f"  {label:<10}: {_stats(a)}")

    alt_ok = min_alt >= _MIN_ALT_M
    print(_pass_fail(alt_ok, f"min altitude {min_alt:.2f} m  (limit={_MIN_ALT_M:.1f} m)"))
    results["min_alt_m"]  = min_alt
    results["max_alt_m"]  = max_alt
    results["alt_pass"]   = alt_ok

    # ── Tether ───────────────────────────────────────────────────────────
    _section("TETHER LENGTH / EXTENSION")
    taut = [r for r in rows if not r.tether_slack and r.tether_length > 0]
    if taut:
        lens  = [r.tether_length      for r in taut]
        exts  = [r.tether_extension   for r in taut]
        rests = [r.tether_rest_length for r in taut]
        print(f"  length:       {_stats(lens)}")
        print(f"  extension:    {_stats(exts)}")
        print(f"  rest_length:  {_stats(rests)}")
    else:
        print("  (no taut-tether rows)")

    for label in ("reel-out", "reel-in"):
        ph = phases[label]
        if ph:
            taut_ph = [r for r in ph if not r.tether_slack and r.tether_length > 0]
            if taut_ph:
                maxl = max(r.tether_length for r in taut_ph)
                maxe = max(r.tether_extension for r in taut_ph)
                rest_range = (min(r.tether_rest_length for r in taut_ph),
                              max(r.tether_rest_length for r in taut_ph))
                print(f"  {label:<10}: max_len={maxl:.2f}m  max_ext={maxe:.3f}m  "
                      f"rest={rest_range[0]:.1f}–{rest_range[1]:.1f}m")

    # ── Tension ──────────────────────────────────────────────────────────
    _section("TENSION")
    taut_tens = [r for r in rows if r.tether_tension > 0]
    if not taut_tens:
        print("  (no non-zero tension rows)")
    else:
        tens_all = [r.tether_tension for r in taut_tens]
        print(f"  All phases:    {_stats(tens_all)}")

    for label in ("reel-out", "reel-in"):
        ph = phases[label]
        if not ph:
            continue
        tvals = [r.tether_tension for r in ph if r.tether_tension > 0]
        tsets = [r.tension_setpoint for r in ph if r.tension_setpoint > 0]
        if tvals:
            print(f"\n  {label}:")
            print(f"    tension:    {_stats(tvals)}")
            if tsets:
                print(f"    setpoint:   {_stats(tsets)}")
            matching = [(r.tether_tension, r.tension_setpoint)
                        for r in ph if r.tether_tension > 0 and r.tension_setpoint > 0]
            if matching:
                errs = [abs(t - s) for t, s in matching]
                print(f"    |error|:    {_stats(errs)}")
        results[f"tension_{label.replace('-','_')}_mean"] = mean(tvals) if tvals else 0.0
        results[f"tension_{label.replace('-','_')}_max"]  = max(tvals) if tvals else 0.0

    # Tension pass/fail
    peak_tension = max((r.tether_tension for r in rows), default=0.0)
    tension_ok   = peak_tension < _TENSION_LIMIT_N
    print()
    print(_pass_fail(tension_ok,
                     f"peak tension {peak_tension:.1f} N  (limit={_TENSION_LIMIT_N:.0f} N)"))
    results["peak_tension_n"] = peak_tension
    results["tension_pass"]   = tension_ok

    # ── Tension spikes ───────────────────────────────────────────────────
    _section("TENSION SPIKES  (threshold=500 N)")
    spikes_500 = find_tension_spikes(rows, threshold=500.0, min_gap_s=0.5)
    if spikes_500:
        print(f"  {len(spikes_500)} spike(s) above 500 N:")
        for sp in spikes_500[:10]:
            print(f"    {sp}")
        if len(spikes_500) > 10:
            print(f"    ... {len(spikes_500) - 10} more")
    else:
        print("  None above 500 N")

    dur_above_500 = sum(sp.duration for sp in spikes_500)
    print(f"  Total time above 500 N: {dur_above_500:.2f}s")
    results["spike_dur_above_500s"] = dur_above_500
    results["n_spikes_above_500"]   = len(spikes_500)

    # ── Reel-in tension controller analysis ──────────────────────────────
    _section("REEL-IN TENSION CONTROLLER")
    ri = phases["reel-in"]
    if ri:
        # Check if controller is saturating (collective at floor)
        col_vals  = [r.collective_from_tension_ctrl for r in ri
                     if r.collective_from_tension_ctrl != 0.0]
        tens_ri   = [r.tether_tension for r in ri]
        t_ri      = [r.t for r in ri]

        if col_vals:
            print(f"  collective_ctrl: {_stats(col_vals)}")
            n_near_zero = sum(1 for c in col_vals if c < 0.02)
            print(f"  rows at floor (col<0.02): {n_near_zero}/{len(col_vals)} "
                  f"({100*n_near_zero/len(col_vals):.0f}%)")

        # Energy absorbed by winch during reel-in (should be small → net positive)
        # v_reel = d(rest_length)/dt; energy_in = ∫ T × |v_reel| dt (negative work)
        rest_ri = [r.tether_rest_length for r in ri]
        v_reel_ri = []
        for i in range(1, len(ri)):
            dt = ri[i].t - ri[i-1].t
            if dt > 0:
                v_reel_ri.append(abs(rest_ri[i] - rest_ri[i-1]) / dt)
            else:
                v_reel_ri.append(0.0)
        v_reel_ri.insert(0, v_reel_ri[0] if v_reel_ri else 0.0)

        energy_in = _integrate([t * v for t, v in zip(tens_ri, v_reel_ri)], t_ri)
        print(f"  winch energy absorbed (reel-in): {energy_in:.0f} J")
        results["energy_reel_in_j"] = energy_in

    # ── Reel-out energy production ────────────────────────────────────────
    _section("REEL-OUT ENERGY")
    ro = phases["reel-out"]
    if ro:
        tens_ro  = [r.tether_tension for r in ro]
        t_ro     = [r.t for r in ro]
        rest_ro  = [r.tether_rest_length for r in ro]
        v_reel_ro = []
        for i in range(1, len(ro)):
            dt = ro[i].t - ro[i-1].t
            v_reel_ro.append(abs(rest_ro[i] - rest_ro[i-1]) / dt if dt > 0 else 0.0)
        v_reel_ro.insert(0, v_reel_ro[0] if v_reel_ro else 0.0)

        energy_out = _integrate([t * v for t, v in zip(tens_ro, v_reel_ro)], t_ro)
        print(f"  winch energy generated (reel-out): {energy_out:.0f} J")
        results["energy_reel_out_j"] = energy_out

        net_energy = energy_out - results.get("energy_reel_in_j", 0.0)
        net_ok     = net_energy > 0
        print(_pass_fail(net_ok, f"net energy = {net_energy:.0f} J  "
                         f"(out={energy_out:.0f} - in={results.get('energy_reel_in_j',0):.0f})"))
        results["net_energy_j"] = net_energy
        results["energy_pass"]  = net_ok

    # ── Phase tension comparison ──────────────────────────────────────────
    _section("DE SCHUTTER MECHANISM CHECK")
    mean_out = results.get("tension_reel_out_mean", 0.0)
    mean_in  = results.get("tension_reel_in_mean",  0.0)
    mechanism_ok = mean_out > mean_in and mean_in >= 0
    print(f"  reel-out mean tension: {mean_out:.1f} N")
    print(f"  reel-in  mean tension: {mean_in:.1f} N")
    print(_pass_fail(mechanism_ok,
                     f"reel-out ({mean_out:.0f} N) > reel-in ({mean_in:.0f} N)"))
    results["mechanism_pass"] = mechanism_ok

    # ── Overall summary ───────────────────────────────────────────────────
    _section("SUMMARY")
    checks = [
        ("Altitude",          results.get("alt_pass",       False)),
        ("Peak tension",      results.get("tension_pass",   False)),
        ("De Schutter mech",  results.get("mechanism_pass", False)),
        ("Net energy > 0",    results.get("energy_pass",    False)),
    ]
    all_pass = all(v for _, v in checks)
    for name, ok in checks:
        print(_pass_fail(ok, name))
    print()
    print(f"  {'PASSED' if all_pass else 'FAILED'}  ({sum(v for _,v in checks)}/{len(checks)} checks)")
    results["all_pass"] = all_pass

    # ── Per-cycle breakdown ───────────────────────────────────────────────
    cycles = split_cycles(rows)
    _section(f"PER-CYCLE BREAKDOWN  ({len(cycles)} cycle(s) detected)")
    cycle_summaries = []
    for cyc in cycles:
        label = f"Cycle {cyc.index + 1}"
        status = "complete" if cyc.complete else "partial"
        t_out_dur = (cyc.out_rows[-1].t - cyc.out_rows[0].t) if len(cyc.out_rows) > 1 else 0.0
        t_in_dur  = (cyc.in_rows[-1].t  - cyc.in_rows[0].t)  if len(cyc.in_rows)  > 1 else 0.0
        print(f"  {label} [{status}]  t={cyc.t_start:.1f}–{cyc.t_end:.1f}s  "
              f"reel-out={t_out_dur:.1f}s  reel-in={t_in_dur:.1f}s")

        cyc_summary: dict = {
            "index":       cyc.index,
            "complete":    cyc.complete,
            "t_start":     cyc.t_start,
            "t_end":       cyc.t_end,
        }

        if cyc.out_rows:
            out_t = [r.tether_tension for r in cyc.out_rows if r.tether_tension > 0]
            out_rests = [r.tether_rest_length for r in cyc.out_rows]
            e_out = sum(r.tether_tension * 0.4 * 0.0025 for r in cyc.out_rows)  # ≈ integral
            print(f"    reel-out: T mean={mean(out_t):.1f} N  max={max(out_t):.1f} N  "
                  f"rest={min(out_rests):.1f}–{max(out_rests):.1f} m  E={e_out:.0f} J")
            cyc_summary["mean_tension_out_n"]  = mean(out_t) if out_t else 0.0
            cyc_summary["max_tension_out_n"]   = max(out_t)  if out_t else 0.0
            cyc_summary["energy_reel_out_j"]   = e_out

        if cyc.in_rows:
            in_t = [r.tether_tension for r in cyc.in_rows if r.tether_tension > 0]
            in_rests = [r.tether_rest_length for r in cyc.in_rows]
            e_in = sum(r.tether_tension * 0.4 * 0.0025 for r in cyc.in_rows)
            print(f"    reel-in:  T mean={mean(in_t):.1f} N  max={max(in_t):.1f} N  "
                  f"rest={min(in_rests):.1f}–{max(in_rests):.1f} m  E={e_in:.0f} J")
            e_out_cyc = cyc_summary.get("energy_reel_out_j", 0.0)
            net = e_out_cyc - e_in
            de_schutter = (cyc_summary.get("mean_tension_out_n", 0) > mean(in_t)) if in_t else False
            print(f"    net energy: {net:.0f} J  De Schutter: {'OK' if de_schutter else 'FAIL'}")
            cyc_summary["mean_tension_in_n"]   = mean(in_t) if in_t else 0.0
            cyc_summary["max_tension_in_n"]    = max(in_t)  if in_t else 0.0
            cyc_summary["energy_reel_in_j"]    = e_in
            cyc_summary["net_energy_j"]        = net
            cyc_summary["de_schutter_ok"]      = de_schutter

        cycle_summaries.append(cyc_summary)

    results["cycles"] = cycle_summaries

    # ── Structured per-phase data for downstream analysis ─────────────────
    results["phases"] = {
        label: _phase_detail(phases[label])
        for label in ("reel-out", "reel-in")
        if phases[label]
    }
    results["spikes"] = [
        {"t_start": s.t_start, "t_end": s.t_end,
         "peak_n": s.peak, "duration_s": s.duration}
        for s in spikes_500
    ]
    return results


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

_DEFAULT_JSON = _SIM_DIR / "logs" / "pumping_cycle_analysis.json"


def compare(rows_a: list[TelRow], label_a: str,
            rows_b: list[TelRow], label_b: str) -> None:
    """Print a side-by-side comparison of two telemetry runs."""
    print(f"\n{'=' * 70}")
    print(f"  COMPARISON: {label_a}  vs  {label_b}")
    print('=' * 70)

    def _key_metrics(rows: list[TelRow]) -> dict:
        cycles = split_cycles(rows)
        phases = split_phases(rows)
        ro = phases["reel-out"]
        ri = phases["reel-in"]
        alts = [-r.hub_pos_z for r in rows]
        out_t = [r.tether_tension for r in ro if r.tether_tension > 0]
        in_t  = [r.tether_tension for r in ri if r.tether_tension > 0]
        return {
            "n_rows":          len(rows),
            "n_cycles":        len(cycles),
            "t_span":          f"{rows[0].t:.1f}-{rows[-1].t:.1f}s" if rows else "?",
            "alt_min":         min(alts) if alts else 0.0,
            "alt_max":         max(alts) if alts else 0.0,
            "alt_out_sd":      stdev([-r.hub_pos_z for r in ro]) if len(ro) > 1 else 0.0,
            "peak_tension":    max((r.tether_tension for r in rows), default=0.0),
            "t_above_500":     sum(sp.duration for sp in find_tension_spikes(rows, 500.0)),
            "mean_t_out":      mean(out_t) if out_t else 0.0,
            "mean_t_in":       mean(in_t)  if in_t  else 0.0,
            "col_saturated_%": (100.0 * sum(1 for r in ri if r.collective_from_tension_ctrl < 0.02)
                                / max(len(ri), 1)),
        }

    ma = _key_metrics(rows_a)
    mb = _key_metrics(rows_b)

    rows_cmp = [
        ("rows",             f"{ma['n_rows']:,}",              f"{mb['n_rows']:,}",          ""),
        ("cycles detected",  str(ma['n_cycles']),              str(mb['n_cycles']),           ""),
        ("time span",        ma['t_span'],                     mb['t_span'],                  ""),
        ("--- ALTITUDE ---", "", "", ""),
        ("min altitude (m)", f"{ma['alt_min']:.2f}",           f"{mb['alt_min']:.2f}",        ">0.5"),
        ("max altitude (m)", f"{ma['alt_max']:.2f}",           f"{mb['alt_max']:.2f}",        ""),
        ("reel-out alt sd",  f"{ma['alt_out_sd']:.2f}",        f"{mb['alt_out_sd']:.2f}",     "small = stable"),
        ("--- TENSION ---",  "", "", ""),
        ("peak tension (N)", f"{ma['peak_tension']:.0f}",      f"{mb['peak_tension']:.0f}",   "<496"),
        ("time >500N (s)",   f"{ma['t_above_500']:.2f}",       f"{mb['t_above_500']:.2f}",    "0 ideal"),
        ("reel-out mean (N)",f"{ma['mean_t_out']:.1f}",        f"{mb['mean_t_out']:.1f}",     "~200"),
        ("reel-in mean (N)", f"{ma['mean_t_in']:.1f}",         f"{mb['mean_t_in']:.1f}",      "<reel-out"),
        ("col saturated %",  f"{ma['col_saturated_%']:.0f}",   f"{mb['col_saturated_%']:.0f}", "0 ideal"),
    ]

    w = 22
    print(f"  {'Metric':<{w}} {'':>18} {'':>18}  {'Target'}")
    print(f"  {'-'*w} {'-'*18} {'-'*18}  {'-'*12}")
    pad = " " * w
    print(f"  {pad} {label_a:>18} {label_b:>18}")
    print(f"  {'-'*w} {'-'*18} {'-'*18}")
    for name, va, vb, target in rows_cmp:
        if va == "" and vb == "":
            print(f"\n  {name}")
            continue
        print(f"  {name:<{w}} {va:>18} {vb:>18}  {target}")
    print()


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Analyse pumping cycle telemetry CSV and write structured JSON.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  python3 analyse_pumping_cycle.py\n"
            "  python3 analyse_pumping_cycle.py --csv /path/to/telemetry_pumping.csv\n"
            "  python3 analyse_pumping_cycle.py --json /path/to/output.json\n"
            "  python3 analyse_pumping_cycle.py --no-json   # suppress JSON output\n"
        ),
    )
    ap.add_argument("--csv",         type=Path, default=_DEFAULT_CSV,
                    help=f"Input telemetry CSV  (default: {_DEFAULT_CSV})")
    ap.add_argument("--json",        type=Path, default=_DEFAULT_JSON,
                    help=f"Output JSON file  (default: {_DEFAULT_JSON})")
    ap.add_argument("--no-json",     action="store_true",
                    help="Suppress JSON output (print report only)")
    ap.add_argument("--compare-csv", type=Path, default=None,
                    help="Second CSV to compare against --csv (side-by-side table)")
    args = ap.parse_args()

    print(f"Loading: {args.csv}")
    rows = load_telemetry(args.csv)
    if not rows:
        print(f"ERROR: no rows loaded from {args.csv}", file=sys.stderr)
        sys.exit(1)

    results = analyse(rows)

    # Optional side-by-side comparison
    if args.compare_csv is not None:
        print(f"\nLoading comparison: {args.compare_csv}")
        rows_b = load_telemetry(args.compare_csv)
        if rows_b:
            compare(rows, args.csv.name, rows_b, args.compare_csv.name)
        else:
            print(f"WARNING: no rows loaded from {args.compare_csv}")

    # Write structured JSON (omit large phase-series data by default to keep
    # the file compact; pass --json-full for the raw series — future flag)
    if not args.no_json:
        json_out = args.json
        # Compact summary: exclude raw series (kept in results["phases"])
        summary = {k: v for k, v in results.items() if k not in ("phases",)}
        json_out.parent.mkdir(parents=True, exist_ok=True)
        with json_out.open("w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2)
        print(f"\n  JSON summary: {json_out}")

    sys.exit(0 if results.get("all_pass") else 1)


if __name__ == "__main__":
    main()
