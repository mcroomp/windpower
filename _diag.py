import sys, math
sys.path.insert(0, "e:/repos/windpower/simulation")

import numpy as np
from envelope.point_mass import tether_hat, balance_bz, simulate_point
import rotor_definition as rd
from aero import create_aero
from frames import build_orb_frame

rotor  = rd.default()
aero   = create_aero(rotor, model="peters_he")
dk     = rotor.dynamics_kwargs()
mass   = dk["mass"]
I_spin = dk["I_spin"]

elevation_deg = 30.0
tension_n     = 700.0
wind_speed    = 10.0
gravity       = 9.81
dt            = 0.02
col_now       = 0.0
kp_col, ki_col = 0.02, 0.005
kp_cyc, ki_cyc = 0.5, 0.1
cyc_clamp_deg  = 8.6
v_target       = -0.8

t_hat  = tether_hat(elevation_deg)
F_teth = tension_n * t_hat
bz     = balance_bz(elevation_deg, tension_n, mass)
R      = build_orb_frame(bz)
wind   = np.array([0.0, -float(wind_speed), 0.0])
clamp  = math.radians(cyc_clamp_deg)
weight = mass * gravity

omega    = 28.0
vel      = np.zeros(3)
c_lon = c_lat = 0.0
int_vx = int_vy = int_vcol = 0.0

print(f"{'t':>5}  {'v_along':>8}  {'v_horiz':>8}  {'vel_z':>7}  {'omega':>6}  {'col':>7}  {'c_lon':>6}  {'c_lat':>6}")
for i in range(2000):
    t = i * dt
    try:
        f = aero.compute_forces(col_now, c_lon, c_lat, R, vel.copy(), omega, wind, t=45.0)
    except Exception as e:
        print(f"t={t:.2f}  CRASH in aero: {e}")
        print(f"  vel={vel}  omega={omega:.2f}  col={math.degrees(col_now):.2f}")
        break

    thrust = float(np.dot(f.F_world, bz))
    Q_net  = float(np.dot(aero.last_M_spin, bz))
    F_net  = f.F_world + F_teth + np.array([0.0, 0.0, weight])

    omega = max(0.0, omega + (Q_net / I_spin) * dt)
    vel   = vel + (F_net / mass) * dt

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

    v_along_now = float(np.dot(vel, t_hat))
    err = v_along_now - v_target
    int_vcol += err * dt
    col_now = float(np.clip(col_now + (0.02 * err + 0.005 * int_vcol), -0.25, 0.20))

    if i % 5 == 0:
        v_along = float(np.dot(vel, t_hat))
        v_horiz = float(np.linalg.norm(vel[:2]))
        print(f"{t:5.1f}  {v_along:+8.3f}  {v_horiz:+8.3f}  {float(vel[2]):+7.3f}  "
              f"{omega:6.2f}  {math.degrees(col_now):+7.2f}  "
              f"{math.degrees(c_lon):+6.2f}  {math.degrees(c_lat):+6.2f}")
