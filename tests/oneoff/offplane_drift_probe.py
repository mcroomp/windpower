"""offplane_drift_probe.py — one-off diagnostic, not a unit test.

Investigates why the kite crabs sideways when the tether is not in the wind
plane, or when cyclic is zeroed from the steady-state IC.

APPROACH
--------
Rather than integrating forward (which diverges due to translational
instability without feedback), we compute the *instantaneous* crosswind
force balance at t=0 for a grid of lateral (crosswind) displacements.

This gives a force-vs-displacement map that directly answers:
  - Is there a tether restoring force when displaced crosswind?
  - Does the aero produce a crosswind force at the off-plane position?
  - What is the net crosswind acceleration as a function of displacement?

Two cyclic cases:
  TRIM    — tilt_lat = ic.trim_tilt_lat, tilt_lon = ic.trim_tilt_lon
  ZERO    — tilt_lat = 0, tilt_lon = 0

For each, we sweep N ∈ [-40 .. +40] m in 5 m steps and log:
  F_aero_cw, F_tether_cw, F_total_cw, acc_cw, body_z, tether angles.

Additionally, two free-rotation short runs (2 s at 400 Hz) are included:
  D  On-plane IC,  zero cyclic — shows how fast disk tumbles when released
  C  Off-plane IC, zero cyclic — same but off-plane start

Run:
  c:\\repos\\windpower\\simulation\\.venv\\Scripts\\python.exe ^
      tests/oneoff/offplane_drift_probe.py
"""
from __future__ import annotations

import csv
import copy
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

# ── Path setup ────────────────────────────────────────────────────────────────
ROOT = Path(__file__).resolve().parents[3]
SIM  = ROOT / "simulation"

from simulation.physics_core import PhysicsCore
from tests.simtests.simtest_ic import load_ic
from tests.simtests._rotor_helpers import load_default_rotor
from simulation.frames import build_orb_frame
from simulation.rotor_physics import resolve_i_spin_kgm2
from simulation.tether import TetherModel

# ── Constants ─────────────────────────────────────────────────────────────────
WIND     = np.array([0.0, 10.0, 0.0])   # NED: 10 m/s East
DT       = 2.5e-3                        # 400 Hz
DURATION = 20.0                          # seconds (for free-rotation reference)
STEPS    = int(DURATION / DT)

# Wind-plane basis (NED horizontal)
# downwind = unit East; crosswind = unit North (perpendicular, left of downwind)
_wind_h   = np.array([WIND[0], WIND[1]], dtype=float)
_wh_norm  = float(np.linalg.norm(_wind_h))
_DW       = _wind_h / _wh_norm                           # (down_N, down_E)
_CW       = np.array([-_DW[1], _DW[0]], dtype=float)    # (cross_N, cross_E)

OFFPLANE_NORTH_DISP = 20.0   # m — crosswind displacement for off-plane cases
LOG_DIR = SIM / "logs" / "offplane_drift_probe"


class ForceOnlyTether:
    """Wrap TetherModel but suppress its moment while preserving force/tension."""

    def __init__(self, base: TetherModel):
        self._base = base
        self._last_info: dict = {}

    @property
    def rest_length(self) -> float:
        return self._base.rest_length

    @rest_length.setter
    def rest_length(self, value: float) -> None:
        self._base.rest_length = float(value)

    @property
    def EA(self) -> float:
        return self._base.EA

    @property
    def BREAK_LOAD_N(self) -> float:
        return self._base.BREAK_LOAD_N

    @property
    def damping(self) -> float:
        return self._base.damping

    def compute(self, hub_pos: np.ndarray, hub_vel: np.ndarray, R_hub: np.ndarray = None) -> tuple:
        force, _moment = self._base.compute(hub_pos, hub_vel, R_hub)
        self._last_info = dict(self._base._last_info)
        return force, np.zeros(3)


def _force_only_tether(rotor, rest_length: float) -> ForceOnlyTether:
    if rotor.control is None or rotor.control.axle_attachment_length_m is None:
        raise ValueError("rotor.control.axle_attachment_length_m must be set")
    base = TetherModel(
        anchor_ned=np.zeros(3),
        rest_length=float(rest_length),
        axle_attachment_length=float(rotor.control.axle_attachment_length_m),
    )
    return ForceOnlyTether(base)


def _rotor_with_i_spin(rotor, i_spin_kgm2: float):
    clone = copy.deepcopy(rotor)
    setattr(clone.inertia, "I_spin_kgm2", float(i_spin_kgm2))
    return clone


def _decompose_h(v3: np.ndarray) -> tuple[float, float]:
    """Return (downwind, crosswind) components of a NED 3-vector."""
    h = np.asarray(v3[:2], dtype=float)
    dw = float(np.dot(h, _DW))
    cw = float(np.dot(h, _CW))
    return dw, cw


def _compute_instant_forces(
    pos: np.ndarray,
    ic,
    rotor,
    tilt_lon: float,
    tilt_lat: float,
) -> dict:
    """
    Compute forces at ONE instant (t=0, vel=0) at the given hub position.

    The orientation is set by build_orb_frame(-pos/|pos|) so body_z tracks
    the tether direction toward the anchor.  No integration.

    Returns a flat dict of scalars.
    """
    tether_dir = -pos / float(np.linalg.norm(pos))
    R0 = build_orb_frame(tether_dir)

    ic_adj = SimpleNamespace(
        pos          = np.asarray(pos, dtype=float),
        vel          = np.zeros(3),
        R0           = R0,
        rest_length  = ic.rest_length,
        coll_eq_rad  = ic.coll_eq_rad,
        omega_spin   = ic.omega_spin,
    )

    core = PhysicsCore(rotor, ic_adj, WIND, z_floor=-1.0)
    mass = float(rotor.inertia.mass_kg)
    g    = 9.81

    # One integration step so PhysicsCore runs the aero and tether
    result = core.step(DT, ic.coll_eq_rad, tilt_lon, tilt_lat)

    aero_r  = result["aero_result"]
    F_aero  = np.asarray(aero_r.F_world, dtype=float)
    F_teth  = np.asarray(result["tether_force"], dtype=float)
    F_grav  = np.array([0.0, 0.0, mass * g])
    F_total = F_aero + F_teth + F_grav

    def _cw(v3): return float(np.dot(np.asarray(v3[:2], dtype=float), _CW))
    def _dw(v3): return float(np.dot(np.asarray(v3[:2], dtype=float), _DW))

    body_z  = R0[:, 2]
    body_x  = R0[:, 0]
    tlen    = float(np.linalg.norm(pos))
    _, cw_pos = _decompose_h(pos)

    return {
        "cw_disp_m":      cw_pos,
        "F_aero_cw":      _cw(F_aero),
        "F_aero_dw":      _dw(F_aero),
        "F_aero_Z":       float(F_aero[2]),
        "F_teth_cw":      _cw(F_teth),
        "F_teth_dw":      _dw(F_teth),
        "F_teth_Z":       float(F_teth[2]),
        "F_total_cw":     _cw(F_total),
        "F_total_dw":     _dw(F_total),
        "F_total_Z":      float(F_total[2]),
        "acc_cw":         _cw(F_total) / mass,
        "tension":        float(result["tension_now"]),
        "tether_length":  tlen,
        "body_z_N":       float(body_z[0]),
        "body_z_E":       float(body_z[1]),
        "body_z_cw_deg":  math.degrees(math.asin(max(-1.0, min(1.0, _cw(body_z))))),
        "body_x_cw_deg":  math.degrees(math.asin(max(-1.0, min(1.0, _cw(body_x))))),
    }


def _run_free_short(
    label: str,
    ic,
    rotor,
    pos_override: np.ndarray | None,
    tilt_lon: float,
    tilt_lat: float,
    duration: float = 2.0,
    suppress_tether_moment: bool = False,
) -> list[dict]:
    """
    Short free-rotation run (no attitude control), logging per-step forces.
    Used to confirm disk-tumble rate and resulting crosswind force pattern.
    """
    pos0 = np.asarray(pos_override if pos_override is not None else ic.pos, dtype=float)
    tether_dir = -pos0 / float(np.linalg.norm(pos0))
    R0 = build_orb_frame(tether_dir)

    ic_adj = SimpleNamespace(
        pos=pos0, vel=np.zeros(3), R0=R0,
        rest_length=ic.rest_length, coll_eq_rad=ic.coll_eq_rad, omega_spin=ic.omega_spin,
    )
    tether_override = (_force_only_tether(rotor, ic.rest_length)
                       if suppress_tether_moment else None)
    core  = PhysicsCore(rotor, ic_adj, WIND, z_floor=-1.0,
                        tether_override=tether_override)
    mass  = float(rotor.inertia.mass_kg)
    g     = 9.81
    steps = int(duration / DT)
    rows  = []

    for step in range(steps):
        t   = step * DT
        hub = core.hub_state
        pos = np.asarray(hub["pos"], dtype=float)
        vel = np.asarray(hub["vel"], dtype=float)
        R   = np.asarray(hub["R"],   dtype=float)

        body_z = R[:, 2]
        tlen   = float(np.linalg.norm(pos))
        tdir   = -pos / max(tlen, 1e-6)
        bz_ang = math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(body_z, tdir))))))
        _, cw_pos = _decompose_h(pos)
        _, cw_vel = _decompose_h(vel)

        result  = core.step(DT, ic.coll_eq_rad, tilt_lon, tilt_lat)
        F_aero  = np.asarray(result["aero_result"].F_world, dtype=float)
        F_teth  = np.asarray(result["tether_force"], dtype=float)
        M_teth  = np.asarray(result["tether_moment"], dtype=float)
        F_total = F_aero + F_teth + np.array([0.0, 0.0, mass * g])
        omega   = np.asarray(result["hub_state"]["omega"], dtype=float)

        def _cw(v3): return float(np.dot(np.asarray(v3[:2], dtype=float), _CW))

        rows.append({
            "t":        f"{t:.4f}",
            "cw_pos":   f"{cw_pos:.4f}",
            "cw_vel":   f"{cw_vel:.5f}",
            "F_aero_cw":f"{_cw(F_aero):.4f}",
            "F_teth_cw":f"{_cw(F_teth):.4f}",
            "M_teth_norm": f"{np.linalg.norm(M_teth):.6f}",
            "omega_norm": f"{np.linalg.norm(omega):.6f}",
            "acc_cw":   f"{_cw(F_total)/mass:.5f}",
            "bz_teth":  f"{bz_ang:.2f}",
        })

    return rows


def _write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


def main() -> None:
    ic    = load_ic()
    rotor = load_default_rotor()
    rotor_no_gyro = _rotor_with_i_spin(rotor, 0.0)
    mass  = float(rotor.inertia.mass_kg)
    i_spin = resolve_i_spin_kgm2(rotor)

    print("=" * 72)
    print("RAWES off-plane drift probe")
    print(f"IC  pos=[{ic.pos[0]:.2f}, {ic.pos[1]:.2f}, {ic.pos[2]:.2f}] m")
    print(f"    alt={-ic.pos[2]:.2f} m  tether_len={np.linalg.norm(ic.pos):.2f} m")
    print(f"    coll={math.degrees(ic.coll_eq_rad):.2f} deg  omega_spin={ic.omega_spin:.2f} rad/s")
    print(f"    trim_tilt_lon={ic.trim_tilt_lon:.5f}  trim_tilt_lat={ic.trim_tilt_lat:.5f}")
    print(f"    I_spin={i_spin:.3f} kg*m^2  H_spin={i_spin * ic.omega_spin:.2f} kg*m^2/s")
    print(f"Wind={WIND.tolist()}")
    print("=" * 72)

    # ── Part 1: Quasi-static force sweep over crosswind displacement ──────────
    print("\nPart 1: Quasi-static crosswind force sweep (body_z locked to tether, vel=0)")
    print("  (Forces computed at t=0 for each displacement — no integration divergence)")
    print("  N_disp = North offset from wind plane [m]  (wind is East, so N = crosswind)")
    print("  F_*_cw = force projected onto CW axis = South direction  (>0 = toward N=0 when N>0)")
    print("  acc_cw = net CW acceleration [m/s^2]  (pos = toward N=0 when N>0 = RESTORING)")
    print()

    north_disps = [-40, -30, -20, -10, -5, 0, 5, 10, 20, 30, 40]  # metres

    for label, tlon, tlat in [
        ("TRIM cyclic", ic.trim_tilt_lon, ic.trim_tilt_lat),
        ("ZERO cyclic", 0.0, 0.0),
    ]:
        print(f"  --- {label} (tilt_lon={tlon:.4f}, tilt_lat={tlat:.4f}) ---")
        print(
            f"  {'N_disp':>8}  {'F_aero_cw':>10}  {'F_teth_cw':>10}  "
            f"{'F_total_cw':>11}  {'acc_cw':>9}  "
            f"{'tension':>9}  {'bz_cw_deg':>10}"
        )
        sweep_rows = []
        for dn in north_disps:
            pos = ic.pos + np.array([float(dn), 0.0, 0.0])
            r = _compute_instant_forces(pos, ic, rotor, tlon, tlat)
            sweep_rows.append(r)
            cw_disp = float(r["cw_disp_m"])   # = -dn (CW points South)
            ftc     = float(r["F_total_cw"])
            # Restoring: force must oppose crosswind displacement
            #   cw_disp < 0 (N>0) and F_total_cw > 0 (South, back to N=0)
            #   cw_disp > 0 (N<0) and F_total_cw < 0 (North, back to N=0)
            if cw_disp != 0:
                is_restoring = (cw_disp < 0 and ftc > 0) or (cw_disp > 0 and ftc < 0)
            else:
                is_restoring = None
            tag = ("  <-- RESTORING" if is_restoring
                   else ("  <-- DESTABILIZING" if is_restoring is not None else ""))
            print(
                f"  {dn:8d}  {r['F_aero_cw']:10.3f}  {r['F_teth_cw']:10.3f}  "
                f"{r['F_total_cw']:11.3f}  {r['acc_cw']:9.4f}  "
                f"{r['tension']:9.1f}  {r['body_z_cw_deg']:10.4f}"
                + tag
            )
        csv_path = LOG_DIR / f"sweep_{label.lower().replace(' ','_')}.csv"
        _write_csv(csv_path, sweep_rows)
        print(f"  Sweep CSV -> {csv_path.relative_to(ROOT)}")
        print()

    # ── Part 2: Short free-rotation runs (2 s) ───────────────────────────────
    print("\nPart 2: Short free-rotation runs (2 s, no attitude control)")
    print("  Compares normal dynamics against no-gyro and force-only tether variants.")
    print()

    short_cases = [
        ("D_onplane_zero_cyclic",  None,  rotor, 0.0, 0.0, False),
        ("C_offplane_zero_cyclic", ic.pos + np.array([20.0, 0.0, 0.0]), rotor, 0.0, 0.0, False),
        ("C_offplane_zero_cyclic_no_tether_moment", ic.pos + np.array([20.0, 0.0, 0.0]), rotor, 0.0, 0.0, True),
        ("C_offplane_zero_cyclic_no_gyro", ic.pos + np.array([20.0, 0.0, 0.0]), rotor_no_gyro, 0.0, 0.0, False),
        ("C_offplane_zero_cyclic_no_gyro_no_tether_moment", ic.pos + np.array([20.0, 0.0, 0.0]), rotor_no_gyro, 0.0, 0.0, True),
    ]
    stride = max(1, int(0.1 / DT))  # every 0.1 s

    for label, pos_ov, case_rotor, tlon, tlat, suppress_moment in short_cases:
        disp_str = f"+{OFFPLANE_NORTH_DISP}m N" if pos_ov is not None else "on-plane"
        gyro_str = f"I_spin={resolve_i_spin_kgm2(case_rotor):.3f}"
        moment_str = "force-only tether" if suppress_moment else "normal tether moment"
        print(f"  --- Case {label}  ({disp_str}, {gyro_str}, {moment_str}) ---")
        rows = _run_free_short(
            label, ic, case_rotor, pos_ov, tlon, tlat, duration=2.0,
            suppress_tether_moment=suppress_moment,
        )
        csv_path = LOG_DIR / f"{label}.csv"
        _write_csv(csv_path, rows)

        print(f"  {'t':>6}  {'cw_pos':>9}  {'cw_vel':>9}  {'F_aero_cw':>10}  "
              f"{'F_teth_cw':>10}  {'M_teth':>9}  {'omega':>9}  {'acc_cw':>9}  {'bz_teth':>8}")
        for i, r in enumerate(rows):
            if i % stride == 0:
                print(
                    f"  {float(r['t']):6.2f}  {float(r['cw_pos']):9.4f}  "
                    f"{float(r['cw_vel']):9.5f}  {float(r['F_aero_cw']):10.4f}  "
                    f"{float(r['F_teth_cw']):10.4f}  {float(r['M_teth_norm']):9.4f}  "
                    f"{float(r['omega_norm']):9.4f}  {float(r['acc_cw']):9.5f}  "
                    f"{float(r['bz_teth']):8.2f}"
                )
        print(f"  CSV -> {csv_path.relative_to(ROOT)}")
        print()

    # ── Conclusion ────────────────────────────────────────────────────────────
    print("=" * 72)
    print("CONCLUSIONS")
    print()
    print("Part 1 'RESTORING' / 'DESTABILIZING' markers show sign of net F_total_cw:")
    print("  F_total_cw > 0 points South; N_disp > 0 means displaced North")
    print("  If F_total_cw and N_disp have same sign     => restoring (stable crosswind)")
    print("  If F_total_cw and N_disp have opposite sign => destabilizing")
    print()
    print("Dominant force at off-plane positions:")
    print("  Compare |F_aero_cw| vs |F_teth_cw| to identify which drives crabbing")
    print()
    print("Part 2 comparison columns isolate dynamic mechanisms:")
    print("  normal vs no_tether_moment => effect of tether torque on attitude/precession")
    print("  normal vs no_gyro           => effect of rotor gyroscopic coupling")
    print("  bz_teth growth means attitude controller is essential (no self-stability)")


if __name__ == "__main__":
    main()

