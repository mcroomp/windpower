"""
test_tension_continuation.py -- Can the system track a slowly-increasing tension?

Starts settled at T_start (known good), then ramps tension at ramp_rate N/s.
If the system can stay on-target throughout, the failure at higher tensions is
purely a transient initialization problem (warm-start fixes it).
If it diverges mid-ramp, there is a genuine physics stability limit.
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import rotor_definition as rd
from aero import create_aero
from frames import build_orb_frame
from envelope.point_mass import tether_hat, balance_bz


EL         = 30.0
T_START    = 600.0
T_MAX      = 1000.0
RAMP_RATE  = 10.0    # N/s  — slow enough for the PI to track
V_TARGET   = -0.8
WIND       = 10.0
OMEGA_INIT = 28.0
DT         = 0.02
T_SETTLE   = 30.0    # seconds at T_START before ramp begins
TOL        = 0.15    # v_along must stay within this of v_target


def _run_continuation(el=EL, t_start=T_START, t_max=T_MAX,
                      ramp_rate=RAMP_RATE, v_target=V_TARGET,
                      wind=WIND, omega_init=OMEGA_INIT,
                      dt=DT, t_settle=T_SETTLE,
                      kp_col=0.02, ki_col=0.005,
                      kp_cyc=0.5,  ki_cyc=0.1,
                      cyc_clamp_deg=8.6,
                      col_min=-0.25, col_max=0.20):
    """
    Run settle phase then ramp phase.
    Returns list of dicts with per-second snapshots.
    """
    rotor  = rd.default()
    aero   = create_aero(rotor, model="peters_he")
    dk     = rotor.dynamics_kwargs()
    mass   = dk["mass"]
    I_spin = dk["I_spin"]
    gravity = 9.81
    weight  = mass * gravity

    t_hat = tether_hat(el)
    wind_v = np.array([0.0, -float(wind), 0.0])
    clamp  = math.radians(cyc_clamp_deg)

    omega    = float(omega_init)
    vel      = np.zeros(3)
    col_now  = 0.0
    c_lon    = c_lat = 0.0
    int_vx   = int_vy = int_vcol = 0.0

    log_every = max(1, int(round(1.0 / dt)))
    history   = []

    tension   = t_start
    t_ramp_start = None   # set when ramp begins
    diverged_at  = None

    total_steps = int((t_settle + (t_max - t_start) / ramp_rate + 10.0) / dt)

    for i in range(total_steps):
        t_sim = i * dt

        # tension schedule
        if t_sim < t_settle:
            tension = t_start
        else:
            if t_ramp_start is None:
                t_ramp_start = t_sim
            tension = min(t_start + ramp_rate * (t_sim - t_ramp_start), t_max)

        # recompute bz for current tension (disk orientation tracks load)
        bz     = balance_bz(el, tension, mass)
        R      = build_orb_frame(bz)
        F_teth = tension * t_hat

        try:
            f = aero.compute_forces(col_now, c_lon, c_lat, R, vel.copy(),
                                    omega, wind_v, t=45.0)
        except (OverflowError, ValueError, FloatingPointError):
            if diverged_at is None:
                diverged_at = (t_sim, tension)
            break

        thrust = float(np.dot(f.F_world, bz))
        Q_net  = float(np.dot(aero.last_M_spin, bz))
        F_net  = f.F_world + F_teth + np.array([0.0, 0.0, weight])

        omega = max(0.0, omega + (Q_net / I_spin) * dt)
        vel   = vel + (F_net / mass) * dt

        if not np.all(np.isfinite(vel)):
            if diverged_at is None:
                diverged_at = (t_sim, tension)
            break

        # cyclic PI — cross-tether only, gain-normalized, anti-windup
        v_perp  = vel - float(np.dot(vel, t_hat)) * t_hat
        t_norm  = max(abs(thrust), 1.0)
        kp_eff  = kp_cyc / t_norm
        ki_eff  = ki_cyc / t_norm
        c_lon_r = -(kp_eff * v_perp[0] + ki_eff * int_vx)
        c_lat_r = -(kp_eff * v_perp[1] + ki_eff * int_vy)
        c_lon   = float(np.clip(c_lon_r, -clamp, clamp))
        c_lat   = float(np.clip(c_lat_r, -clamp, clamp))
        if abs(c_lon) < clamp:
            int_vx += v_perp[0] * dt
        if abs(c_lat) < clamp:
            int_vy += v_perp[1] * dt

        # collective PI
        v_along_now = float(np.dot(vel, t_hat))
        err = v_along_now - v_target
        int_vcol += err * dt
        col_now = float(np.clip(
            col_now + (kp_col * err + ki_col * int_vcol),
            col_min, col_max,
        ))

        if i % log_every == 0:
            v_along = float(np.dot(vel, t_hat))
            v_horiz = float(np.linalg.norm(vel[:2]))
            history.append(dict(
                t        = round(t_sim, 2),
                tension  = round(tension, 1),
                v_along  = round(v_along, 4),
                v_horiz  = round(v_horiz, 4),
                col_deg  = round(math.degrees(col_now), 3),
                c_lon_deg= round(math.degrees(c_lon), 3),
                c_lat_deg= round(math.degrees(c_lat), 3),
                omega    = round(omega, 3),
                phase    = "settle" if t_sim < t_settle else "ramp",
            ))

    return history, diverged_at


def test_settles_at_t_start():
    """System must be settled at T_START before the ramp begins."""
    history, diverged_at = _run_continuation()
    assert diverged_at is None or diverged_at[1] > T_START + 50, (
        f"diverged during settle phase at T={diverged_at}"
    )
    settle_window = [h for h in history
                     if h["phase"] == "settle" and h["t"] >= T_SETTLE - 10.0]
    assert settle_window, "no settle-phase data"
    v_range = max(h["v_along"] for h in settle_window) - min(h["v_along"] for h in settle_window)
    assert v_range < 0.1, f"did not settle at T_START: v_along range={v_range:.3f}"


def test_ramp_tracking():
    """Print tension vs v_along during ramp to see where tracking breaks."""
    history, diverged_at = _run_continuation()

    ramp = [h for h in history if h["phase"] == "ramp"]
    if not ramp:
        pytest.skip("ramp phase not reached")

    # find where v_along first leaves the tolerance band
    lost_tracking_at = None
    for h in ramp:
        if abs(h["v_along"] - V_TARGET) > TOL:
            lost_tracking_at = h
            break

    print(f"\n  Tension ramp: {T_START:.0f} -> {T_MAX:.0f} N  at {RAMP_RATE:.0f} N/s")
    if diverged_at:
        print(f"  Diverged (overflow) at T={diverged_at[1]:.0f} N  t={diverged_at[0]:.1f} s")
    if lost_tracking_at:
        print(f"  Lost tracking at T={lost_tracking_at['tension']:.0f} N  "
              f"v_along={lost_tracking_at['v_along']:+.3f}")
    else:
        print(f"  Tracked all the way to T={ramp[-1]['tension']:.0f} N")

    print(f"\n  {'t':>6}  {'T':>6}  {'v_along':>8}  {'v_horiz':>8}  {'col':>7}")
    for h in ramp[::2]:
        marker = " <--" if abs(h["v_along"] - V_TARGET) > TOL else ""
        print(f"  {h['t']:6.1f}  {h['tension']:6.0f}  {h['v_along']:+8.3f}  "
              f"{h['v_horiz']:+8.3f}  {h['col_deg']:+7.2f}{marker}")

    # soft assertion — we just want to see how far it gets
    assert True
