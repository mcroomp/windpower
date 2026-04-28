"""
point_mass.py -- Generalised point-mass equilibrium ODE for arbitrary flight conditions.

Extends the analyse_descent approach to arbitrary tether elevation angles.

Physics
-------
- Disk orientation fixed at force-balance direction: bz = -norm(F_teth + F_grav)
- Tether force: constant vector T * [0, cos(el), sin(el)] in NED (East + Down)
- Wind: [0, +wind_speed, 0] in NED (blowing East, i.e. from West)
- Cyclic PI drives horizontal velocity (North, East) toward zero
- Along-tether and vertical velocities evolve freely from force balance
- Spin ODE: d(omega)/dt = Q_spin / I_spin

At el=90 (vertical) this is identical to simulate_descent() in analyse_descent.py.

NED convention: X=North, Y=East, Z=Down. Gravity = [0, 0, +g*m].
Altitude = -pos[2]. Upward thrust = dot(F_world, body_z).
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import rotor_definition as rd
from aero   import create_aero
from frames import build_orb_frame

_G = 9.81


def tether_hat(elevation_deg: float) -> np.ndarray:
    """Unit vector from hub toward anchor (East + Down in NED)."""
    el = math.radians(elevation_deg)
    return np.array([0.0, math.cos(el), math.sin(el)])


def balance_bz(elevation_deg: float, tension_n: float, mass: float,
               gravity: float = _G) -> np.ndarray:
    """Disk normal that opposes combined tether + gravity load."""
    t_hat  = tether_hat(elevation_deg)
    F_teth = tension_n * t_hat
    F_grav = np.array([0.0, 0.0, mass * gravity])
    F_load = F_teth + F_grav
    return -F_load / np.linalg.norm(F_load)


def simulate_point(
    col:           float,
    elevation_deg: float = 90.0,
    tension_n:     float = 176.0,
    wind_speed:    float = 0.0,
    v_along_init:  float = 0.0,
    omega_init:    float = 5.0,
    dt:            float = 0.01,
    t_end:         float = 40.0,
    kp_cyc:        float = 0.5,
    ki_cyc:        float = 0.1,
    cyc_clamp_deg: float = 8.6,
    v_target:      float | None = None,
    kp_col:        float = 0.02,
    ki_col:        float = 0.005,
    col_min:       float = -0.25,
    col_max:       float = 0.20,
    gravity:       float = _G,
    ic:            dict | None = None,
    omega_conv_tol: float = 0.1,
    conv_window_s: float = 5.0,
) -> dict:
    """
    Point-mass equilibrium simulation.

    When v_target is None: runs at fixed collective `col`.
    When v_target is set:  col is the initial collective; a PI on
        (v_along - v_target) adjusts collective every step to drive
        v_along toward v_target.  The settled collective is reported
        in eq["collective_rad"] and eq["collective_deg"].

    Parameters
    ----------
    col           : collective [rad] (initial value when v_target is set)
    elevation_deg : tether elevation above horizontal [deg]
    tension_n     : tether tension magnitude [N]
    wind_speed    : wind speed [m/s], blowing West (away from anchor)
    v_along_init  : initial velocity along tether toward anchor [m/s]
    omega_init    : initial rotor speed [rad/s]
    dt            : timestep [s]
    t_end         : simulation duration [s] (hard limit)
    kp_cyc/ki_cyc : horizontal-velocity PI cyclic gains
    cyc_clamp_deg : cyclic clamp limit [deg]
    v_target      : target along-tether velocity [m/s]; enables collective PI
    kp_col/ki_col : collective PI gains (rad per m/s error)
    col_min/max   : collective clamp [rad]
    omega_conv_tol : omega convergence tolerance [rad/s]; exit early if omega
                     range over conv_window_s is below this
    conv_window_s : window duration [s] to check for convergence
    """
    rotor  = rd.default()
    dk     = rotor.dynamics_kwargs()
    mass   = dk["mass"]
    I_spin = dk["I_spin"]
    weight = mass * gravity

    t_hat   = tether_hat(elevation_deg)
    F_teth  = tension_n * t_hat
    bz      = balance_bz(elevation_deg, tension_n, mass, gravity=gravity)
    R       = build_orb_frame(bz)
    wind    = np.array([0.0, -float(wind_speed), 0.0])  # blowing West, away from anchor
    clamp   = math.radians(cyc_clamp_deg)

    if ic is not None:
        aero     = create_aero(rotor, model="peters_he",
                               state_dict=ic.get("aero_state"))
        omega    = float(ic.get("omega",    omega_init))
        vel      = np.array(ic.get("vel",   float(v_along_init) * t_hat), dtype=float)
        col_now  = float(ic.get("col",      col))
        c_lon    = float(ic.get("c_lon",    0.0))
        c_lat    = float(ic.get("c_lat",    0.0))
        int_vx   = float(ic.get("int_vx",   0.0))
        int_vy   = float(ic.get("int_vy",   0.0))
        int_vcol = float(ic.get("int_vcol", 0.0))
    else:
        aero     = create_aero(rotor, model="peters_he")
        omega    = float(omega_init)
        vel      = float(v_along_init) * t_hat
        col_now  = float(col)
        c_lon    = 0.0
        c_lat    = 0.0
        int_vx   = 0.0
        int_vy   = 0.0
        int_vcol = 0.0

    history   = []
    log_every = max(1, int(round(1.0 / dt)))
    n_steps   = int(t_end / dt)
    conv_steps = max(1, int(round(conv_window_s / dt)))

    for i in range(n_steps):
        try:
            f = aero.compute_forces(col_now, c_lon, c_lat, R, vel.copy(), omega, wind, t=45.0)
        except (OverflowError, ValueError, FloatingPointError):
            break
        if not aero.is_valid():
            break

        thrust     = float(np.dot(f.F_world, bz))
        Q_net      = float(np.dot(aero.last_M_spin, bz))
        F_net      = f.F_world + F_teth + np.array([0.0, 0.0, weight])  # weight = mass*gravity

        omega = max(0.0, omega + (Q_net / I_spin) * dt)
        vel   = vel + (F_net / mass) * dt

        # Cyclic PI: null cross-tether velocity only (project out along-tether)
        # Gains normalized by thrust so bandwidth stays constant across tension levels.
        # Anti-windup: freeze integrator when output is saturated.
        v_perp   = vel - float(np.dot(vel, t_hat)) * t_hat
        t_norm   = max(abs(thrust), 1.0)
        kp_eff   = kp_cyc / t_norm
        ki_eff   = ki_cyc / t_norm
        c_lon_raw = -(kp_eff * v_perp[0] + ki_eff * int_vx)
        c_lat_raw = -(kp_eff * v_perp[1] + ki_eff * int_vy)
        c_lon = float(np.clip(c_lon_raw, -clamp, clamp))
        c_lat = float(np.clip(c_lat_raw, -clamp, clamp))
        if abs(c_lon) < clamp:
            int_vx += v_perp[0] * dt
        if abs(c_lat) < clamp:
            int_vy += v_perp[1] * dt

        # Collective PI: drive v_along toward v_target
        # Sign: higher collective → more thrust opposing tether → lower v_along
        # So to raise v_along we decrease collective: col += kp * err
        if v_target is not None:
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
                t              = round(i * dt, 2),
                omega          = round(omega, 3),
                v_along        = round(v_along, 4),
                v_horiz        = round(v_horiz, 4),
                vel_z          = round(float(vel[2]), 4),
                thrust         = round(thrust, 2),
                F_net_along    = round(float(np.dot(F_net, t_hat)), 3),
                c_lon_deg      = round(math.degrees(c_lon), 3),
                c_lat_deg      = round(math.degrees(c_lat), 3),
                collective_rad = round(col_now, 5),
                collective_deg = round(math.degrees(col_now), 3),
            ))

        # Early exit if v_along, cyclic, and omega have all converged
        if len(history) >= conv_steps and i > int(round(10.0 / dt)):
            recent = history[-conv_steps:]
            v_along_vals = np.array([h["v_along"] for h in recent])
            c_lon_vals   = np.array([h["c_lon_deg"] for h in recent])
            c_lat_vals   = np.array([h["c_lat_deg"] for h in recent])
            omega_vals   = np.array([h["omega"] for h in recent])

            v_along_range = np.max(v_along_vals) - np.min(v_along_vals)
            c_lon_range   = np.max(c_lon_vals) - np.min(c_lon_vals)
            c_lat_range   = np.max(c_lat_vals) - np.min(c_lat_vals)
            omega_range   = np.max(omega_vals) - np.min(omega_vals)

            # Convergence thresholds
            v_conv       = v_along_range < 0.05  # m/s
            cyc_conv     = max(c_lon_range, c_lat_range) < 0.5  # deg
            omega_conv   = omega_range < omega_conv_tol  # rad/s

            if v_conv and cyc_conv and omega_conv:
                break

    final = [h for h in history if h["t"] >= (history[-1]["t"] - 10.0)] if history else []
    eq = ({k: round(float(np.mean([h[k] for h in final])), 5)
           for k in ("omega", "v_along", "v_horiz", "vel_z", "thrust",
                     "F_net_along", "c_lon_deg", "c_lat_deg",
                     "collective_rad", "collective_deg")}
          if final else (history[-1] if history else {}))

    cyc_sat = (abs(eq.get("c_lon_deg", 0)) >= cyc_clamp_deg * 0.95 or
               abs(eq.get("c_lat_deg", 0)) >= cyc_clamp_deg * 0.95)
    col_sat = (v_target is not None and
               (col_now <= col_min * 0.99 or col_now >= col_max * 0.99))

    ic_out = dict(
        col      = col_now,
        vel      = vel.copy(),
        omega    = omega,
        c_lon    = c_lon,
        c_lat    = c_lat,
        int_vx   = int_vx,
        int_vy   = int_vy,
        int_vcol = int_vcol,
        aero_state = aero.to_dict(),
    )

    return dict(history=history, eq=eq, cyc_saturated=cyc_sat,
                col_saturated=col_sat, ic=ic_out)
