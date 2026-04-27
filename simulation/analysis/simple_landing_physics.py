"""
simple_landing_physics.py -- From-scratch force balance for descent at xi=80 deg.

Uses only momentum theory (actuator disk) + simple blade element theory.
No BEM iteration, no Coleman correction.  Goal: validate direction of forces.

Usage:
    .venv/Scripts/python.exe simulation/analysis/simple_landing_physics.py
"""
import math

# ── Rotor parameters (from rotor_definition.py beaupoil_2026) ──────────────
R       = 2.5       # rotor radius m
R_ROOT  = 0.5       # root cutout m
CHORD   = 0.2       # blade chord m
N_BLADES = 4
CL0     = 0.524     # zero-AoA lift coefficient
CL_A    = 5.47      # lift slope  per rad
CD0     = 0.018     # profile drag
MASS    = 5.0       # kg
G       = 9.81      # m/s^2
RHO     = 1.22      # kg/m^3

# ── Derived ────────────────────────────────────────────────────────────────
A       = math.pi * R**2                       # disk area m^2
SIGMA   = N_BLADES * CHORD / (math.pi * R)     # blade solidity
W       = MASS * G                             # weight N

# ── Scenario ───────────────────────────────────────────────────────────────
XI_DEG  = 80.0      # elevation angle from horizontal (= 90 - tilt_from_vert)
V_WIND  = 10.0      # horizontal wind m/s
OMEGA_RPM = 220.0   # rotor speed (typical from full sim diagnosis)

XI_FROM_VERT = math.radians(90.0 - XI_DEG)   # tilt from vertical = 10 deg
COS_XI = math.cos(XI_FROM_VERT)               # ~0.985 -- most of descent is axial
SIN_XI = math.sin(XI_FROM_VERT)               # ~0.174 -- small wind-axial fraction

OMEGA   = OMEGA_RPM * 2 * math.pi / 60.0
V_TIP   = OMEGA * R

print("=" * 65)
print("Simple landing physics: actuator disk + BET, xi=80 deg")
print("=" * 65)
print(f"  Rotor: R={R}m  A={A:.1f}m^2  sigma={SIGMA:.3f}")
print(f"  Mass:  {MASS}kg  weight={W:.1f}N")
print(f"  xi={XI_DEG}deg elevation  (tilt from vert = {math.degrees(XI_FROM_VERT):.1f}deg)")
print(f"  disk nearly horizontal: cos_xi={COS_XI:.3f}  sin_xi={SIN_XI:.3f}")
print(f"  Wind axial  (through disk) = {V_WIND*SIN_XI:.2f} m/s")
print(f"  Wind in-plane (across disk) = {V_WIND*COS_XI:.2f} m/s")
print(f"  Rotor: {OMEGA_RPM:.0f} RPM  V_tip={V_TIP:.1f} m/s")
print()

# ── Section 1: Momentum theory hover ──────────────────────────────────────
# For pure hover (no descent, no wind): T = 2*rho*A*v_i^2
# v_i_hover = sqrt(T / (2*rho*A))
v_i_hover = math.sqrt(W / (2 * RHO * A))
print("-- 1. Momentum theory hover --")
print(f"  Induced velocity needed for T=W: v_i = {v_i_hover:.3f} m/s")
print(f"  Inflow ratio: lambda_i = v_i/V_tip = {v_i_hover/V_TIP:.4f}")
print()

# ── Section 2: Thrust vs descent rate ─────────────────────────────────────
# For a given collective theta and descent rate v_d:
#   total axial inflow  = v_d*cos_xi + V_wind*sin_xi + v_i  (all positive = up through disk)
#   momentum theory:    T = 2*rho*A*(v_inflow)*v_i
#   blade element:      T = 0.5*rho*(omega*R)^2 * A * sigma * (CL_A*(theta + lambda) + CL0)
#     where lambda = v_axial_total / V_tip  (inflow ratio, positive for upward flow)
#
# We solve for v_i iteratively: given theta and v_d, find v_i so both agree.

def blade_element_thrust(theta_rad, v_axial, omega=OMEGA, rho=RHO):
    """
    Simple uniform-inflow BET (no twist, no taper).
    v_axial: total axial velocity through disk (positive = up through disk, m/s).
    Returns thrust T (positive = up, opposing gravity).
    """
    # Inflow ratio (positive = upward through disk)
    lam = v_axial / (omega * R)
    # Mean AoA at 3/4 radius (Glauert): alpha = theta + lam/0.75 (rough)
    # More carefully: alpha = theta - v_axial/(omega * 0.75*R) = theta - lam/0.75
    # Sign: if disk nearly horizontal and air flows UP through disk, that REDUCES AoA
    alpha_eff = theta_rad - lam / 0.75
    alpha_eff = max(min(alpha_eff, math.radians(14.0)), math.radians(-10.0))  # clamp to stall
    CL = CL0 + CL_A * alpha_eff
    CD = CD0
    # Thrust from lift: T = 0.5 * rho * V_tip^2 * A * sigma * CL
    T = 0.5 * rho * (omega * R)**2 * A * SIGMA * CL
    return T, CL, alpha_eff


def solve_thrust(theta_rad, v_d, n_iter=30):
    """
    Iteratively solve for induced velocity v_i given collective and descent rate.
    v_d: descent rate (positive = downward = increasing NED z = decreasing altitude).
    Returns (T, v_i, alpha_eff, CL).
    """
    v_wind_axial = V_WIND * SIN_XI        # wind pushing up through disk
    v_desc_axial = v_d * COS_XI           # descent pushing up through disk
    v_i = v_i_hover                       # initial guess

    for _ in range(n_iter):
        v_axial = v_desc_axial + v_wind_axial + v_i   # total upward airspeed at disk
        T, CL, alpha = blade_element_thrust(theta_rad, v_axial)
        # Momentum theory: T = 2*rho*A*v_axial_net * v_i
        # v_axial_net = v_desc_axial + v_wind_axial + v_i
        if v_axial > 0.01:
            v_i_new = T / (2 * RHO * A * v_axial)
        else:
            v_i_new = v_i_hover
        v_i = 0.5 * v_i + 0.5 * v_i_new  # damped update

    T, CL, alpha = blade_element_thrust(theta_rad, v_axial)
    return T, v_i, alpha, CL


print("-- 2. Thrust vs descent rate at various collectives --")
print(f"  (T_needed for weight balance = {W:.1f} N)")
print()

THETAS = [-0.28, -0.10, 0.0, 0.05, 0.079, 0.10]

for theta in THETAS:
    print(f"  coll = {theta:+.3f} rad ({math.degrees(theta):+.1f} deg)")
    print(f"  {'v_d_ms':>7}  {'v_axial':>8}  {'T_N':>7}  {'net_F':>7}  {'alpha_deg':>10}  {'CL':>6}  note")
    print(f"  " + "-" * 65)
    for v_d in [0.0, 0.2, 0.5, 1.0, 2.0, 4.0]:
        T, v_i, alpha, CL = solve_thrust(theta, v_d)
        net = T - W
        alpha_deg = math.degrees(alpha)
        note = ""
        if abs(net) < 5:
            note = "<-- equilibrium"
        elif net > 0:
            note = "climbing"
        else:
            note = "falling"
        v_ax = v_d * COS_XI + V_WIND * SIN_XI + v_i
        print(f"  {v_d:7.2f}  {v_ax:8.3f}  {T:7.1f}  {net:+7.1f}  {alpha_deg:10.2f}  {CL:6.3f}  {note}")
    print()

# ── Section 3: Autorotation check ────────────────────────────────────────
# Power balance: rotor torque = 0 in steady autorotation
# Profile drag power consumed = P_profile = rho * A * (omega*R)^3 * sigma * CD0 / 8
# This must be supplied by the descent (air flowing up through disk does work on blades)
# Available power from descent: P_descent = W * v_d * cos_xi  (gravitational PE release rate)
# Autorotation condition: P_descent >= P_profile

P_profile = RHO * A * V_TIP**3 * SIGMA * CD0 / 8.0
print("-- 3. Autorotation power check --")
print(f"  Profile drag power at {OMEGA_RPM:.0f} RPM: {P_profile:.1f} W")
print(f"  (must be supplied by descent to sustain autorotation)")
print()
print(f"  {'v_d_ms':>7}  {'P_grav_W':>9}  {'margin_W':>9}  note")
print(f"  " + "-" * 40)
for v_d in [0.1, 0.2, 0.3, 0.5, 1.0, 2.0]:
    P_grav = W * v_d * COS_XI
    margin = P_grav - P_profile
    note = "autorotation OK" if margin >= 0 else "rotor will slow down"
    print(f"  {v_d:7.2f}  {P_grav:9.1f}  {margin:+9.1f}  {note}")
print()

# ── Section 4: Equilibrium descent rate for each collective ───────────────
print("-- 4. Equilibrium descent rate (T = W) for each collective --")
print(f"  {'coll_rad':>9}  {'v_d_eq_ms':>10}  {'alpha_deg':>10}  note")
print(f"  " + "-" * 45)
for theta in [-0.28, -0.20, -0.10, 0.0, 0.05, 0.079, 0.10]:
    # Binary search for v_d where T = W
    lo, hi = 0.0, 20.0
    for _ in range(50):
        mid = (lo + hi) / 2.0
        T, v_i, alpha, CL = solve_thrust(theta, mid)
        if T > W:
            lo = mid
        else:
            hi = mid
    v_d_eq = (lo + hi) / 2.0
    T, v_i, alpha, CL = solve_thrust(theta, v_d_eq)
    alpha_deg = math.degrees(alpha)
    note = ""
    if alpha_deg > 13:
        note = "WARNING: near stall"
    elif v_d_eq > 5:
        note = "WARNING: very fast"
    elif v_d_eq < 0.1:
        note = "near hover"
    print(f"  {theta:9.3f}  {v_d_eq:10.3f}  {alpha_deg:10.2f}  {note}")
print()
print(f"  COL_CRUISE = 0.079 rad -> equilibrium descent rate shown above")
print(f"  V_LAND target = 0.5 m/s")
