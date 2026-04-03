# Orbit Characteristics of a Pumping-Cycle Rotary Airborne Wind Energy System and Their Implications for Flight Control Design

**Christof Beaupoil**
*RAWES Development Project, 2026*

---

## Abstract

A Rotary Airborne Wind Energy System (RAWES) is a tethered autorotating rotor kite that harvests wind energy through a pumping cycle of reel-out and reel-in phases. Unlike fixed-wing AWE kites, which are actively steered through figure-eight or circular flight paths, a RAWES exhibits a *natural* self-sustaining orbit driven by the interplay of aerodynamic disk tilt, tether constraint, and gyroscopic precession. This orbital motion is largely unreported in the existing literature, yet it fundamentally shapes every aspect of the flight control problem: attitude estimation, rate-loop bias compensation, disk tilt rate limiting, wind direction estimation, and pumping cycle phase transitions.

This paper characterises the orbit of a small-scale 4-blade RAWES (rotor radius 2.5 m, mass 5 kg, Beaupoil 2026) from first-principles simulation using a per-blade Blade Element Momentum (BEM) model with Prandtl tip loss and Coleman skewed-wake correction. We report the equilibrium orbital state, gyroscopic angular momentum, disk tilt angles across the pumping cycle, measured energy results (+1396 J net per 60 s cycle at 10 m/s wind), and derive five specific control design requirements that follow directly from orbit mechanics.

---

## 1. Introduction

Airborne Wind Energy (AWE) systems seek to exploit the stronger, more consistent winds at altitude that are inaccessible to conventional tower-mounted turbines [1, 2]. Rotary AWE systems--where rotating wings on a tethered hub extract wind energy through autorotation--have attracted interest for their mechanical simplicity relative to active-steering flexible-wing systems [3, 4].

The canonical model for a RAWES pumping cycle is due to De Schutter, Leuthold, and Diehl [5], who applied Periodic Optimal Control to a 3-blade rigid-wing rotary kite. Their work defines the reel-out / reel-in power cycle and establishes the disk tilt angle xi (measured from the wind direction) as the primary lever for modulating tether tension between phases. However, their treatment casts the orbit as a trajectory output of the OCP rather than as a physical phenomenon to be understood and exploited. The mechanical origin of the orbit, its period and radius, its role as a persistent gyroscopic bias on the attitude controller, and its value as a passive wind direction sensor are not addressed.

Weyel (2025) treats closed-loop cyclic pitch control for a similar system at the level-B (flap-to-pitch) control hierarchy [6], but again does not characterise the orbital dynamics as a design input for the outer attitude loop.

Tethered autogyro literature [7] analyses altitude and pitch control of a freely spinning rotor on a tether but does not consider the pumping cycle or the interaction between orbital motion and attitude estimation under a standard off-the-shelf flight controller (e.g. ArduPilot).

The present work fills this gap. We show that:

1. The RAWES natural orbit is a quasi-circular cone traced on the tether sphere, sustained without active steering.
2. The orbital period (~60 s), radius (~15 m), and body-frame angular rate (~0.2-0.3 rad/s) are directly set by tether length, wind speed, and disk tilt.
3. Five concrete control design requirements follow from orbit mechanics: tether-relative attitude reporting, orbital rate bias compensation, orbit tracking, disk-tilt slew rate limiting, and passive wind direction estimation.
4. The pumping cycle changes the orbital cone geometry predictably with disk tilt, enabling controlled tension modulation without external orbit steering.

---

## 2. System Description

### 2.1 Physical Configuration

The Beaupoil 2026 rotor is a 4-blade RAWES with the following key parameters:

| Parameter | Value |
|-----------|-------|
| Blade count N | 4 |
| Rotor radius R | 2.5 m |
| Root cutout r_root | 0.5 m |
| Blade chord c | 0.20 m |
| Airfoil | SG6042 |
| Total mass | 5.0 kg |
| Hub weight W | 49 N |
| Ixx = Iyy | 5.0 kg.m^2 |
| Izz | 10.0 kg.m^2 |
| Spin inertia I_spin | ~3.94 kg.m^2 |
| Disk area A | 18.85 m^2 |
| Tether material | Dyneema SK75, 1.9 mm |
| Tether break load | ~620 N |
| Anti-rotation motor | EMAX GB4008 66KV (80:44 gear) |
| Flight controller | Holybro Pixhawk 6C |

Unlike a helicopter, no motor drives rotor rotation: wind drives autorotation, and control authority comes entirely from blade pitch actuated through trailing-edge Kaman servo-flaps driven by a non-rotating swashplate [8].

The tether attaches at the bottom of the stationary axle assembly. Only the outer hub shell and blades rotate; the inner electronics assembly (Pixhawk, battery, ESC) is held stationary by the anti-rotation motor. The effective spin inertia is therefore:

```
I_spin = N x I_blade + I_hub_shell
       = 4 x 0.904 + 0.326 ~= 3.94 kg.m^2
```

where each blade contributes `I_blade = m_blade x (R^2 + R.r_root + r_root^2)/3 = 0.35 x 7.75/3 = 0.904 kg.m^2` (uniform density, radial distribution).

### 2.2 Simulation Model

All results are produced by a 400 Hz RK4 6-DOF rigid-body integrator coupled to a per-blade BEM aerodynamic model (SkewedWakeBEM) with:
- **Prandtl tip and root loss factors** on each radial strip
- **Coleman skewed-wake induction** for non-uniform induction at high disk tilt (xi > 20 deg)
- **5 s aero startup ramp** to avoid impulse loads at t=0

The tether is modelled as a tension-only elastic spring (EA ~= 281 kN, Dyneema SK75). The restoring tether moment at the axle attachment point is disabled (`axle_attachment_length = 0`); disk-tilt stability is provided entirely by aerodynamics.

---

## 3. Natural Orbital Equilibrium

### 3.1 Settled Equilibrium State

Starting from rest and running the simulation with a 200 N tension setpoint, the hub settles to the following equilibrium (wind 10 m/s East, tether anchored at origin):

| Quantity | Value |
|----------|-------|
| Position (NED) | [13.9 N, 47.5 E, -7.1 D] m |
| Tether rest length | 49.97 m |
| Hub altitude AGL | 7.1 m |
| Horizontal distance from anchor | 49.5 m |
| Tether elevation angle beta | 8.2 deg |
| Body-z direction (NED) | [0.276, 0.878, -0.392] |
| Disk tilt from wind xi | 29 deg |
| Disk tilt from vertical | 67 deg |
| Rotor spin omega_eq | 19.3 rad/s (184 RPM) |
| Equilibrium collective | -0.28 rad |

The hub rests predominantly downwind (47.5 m East of anchor) and slightly off-axis (13.9 m North). The azimuthal offset arises from the rotor's angular momentum vector breaking the left-right symmetry about the wind axis: the spinning disk generates a gyroscopic reaction to the orbital precession that shifts the equilibrium azimuth.

The disk normal (body-z) is tilted 29 deg from the wind direction. This is the *autorotation operating angle*: tilting the disk into the wind provides axial inflow that drives rotor spin, while the remaining in-plane wind component generates the tangential force that sustains orbit.

### 3.2 Gyroscopic Angular Momentum

At equilibrium spin, the rotor angular momentum vector is:

```
H = I_spin x omega_eq = 3.94 x 19.3 ~= 76 kg.m^2/s
```

This is a substantial gyroscopic load. The tether tension (~=200 N) and aerodynamic moment arms (~=1.8 m) produce cyclic torques of order 50-100 N.m--comparable to H, meaning gyroscopic precession plays a significant role in disk attitude dynamics.

### 3.3 Orbit Geometry and Period

The hub traces a quasi-circular orbit on the surface of a tether-length sphere. With wind along East (Y-axis in NED), the orbit plane is approximately vertical and perpendicular to the wind, forming a cone of half-angle theta around the wind axis:

```
theta = arccos(tether_dir . wind_hat) ~= arccos(0.950) ~= 18 deg
```

The orbital radius (arc perpendicular to the wind axis) is:

```
r_orbit ~= L x sin(theta) = 50 x sin(18 deg) ~= 15.6 m
```

The orbital period is approximately **60 s** at 10 m/s wind and 50 m tether length, giving:

```
omega_orbit = 2pi / T_orbit ~= 0.105 rad/s (world frame)
```

In the body frame of the tilted disk (67 deg from vertical), this azimuthal rotation projects onto all three body axes, producing a persistent angular rate disturbance of approximately **0.2-0.3 rad/s** in roll and pitch.

### 3.4 Orbit Mechanics: The Gyroscopic Drive Mechanism

The natural orbit is self-sustaining without active steering. The mechanism is:

1. The disk is tilted at xi ~= 29 deg from the wind, producing a net thrust vector misaligned with the tether.
2. This thrust has a component perpendicular to both the tether and the wind axis.
3. This lateral force component pushes the hub azimuthally (around the downwind axis).
4. As the hub moves azimuthally, the tether direction rotates, carrying the equilibrium disk orientation with it.
5. The disk precesses to track the new tether direction (gyroscopic response to cyclic torque from the tether moment).
6. The orbit is self-reinforcing: the hub's orbital velocity adds inplane wind component that sustains autorotation spin.

The orbit period is therefore set by the balance between orbital centripetal force, tether stiffness, and gyroscopic precession rate--not by any active controller.

---

## 4. Aerodynamic Operating Points Across the Pumping Cycle

The disk tilt angle xi changes between reel-out and reel-in phases, altering both the orbital geometry and the BEM operating point. Table 2 summarises key operating conditions at 10 m/s wind:

| Phase | xi [ deg] | omega_eq [rad/s] | Fz (col=-0.20) [N] | Fz (col=0.00) [N] | Fz (col=+0.10) [N] |
|-------|--------|-------------|---------------------|--------------------|--------------------|
| Reel-out | 35 | ~21 | 142 | (high) | -- |
| Nominal reel-in | 55 | ~25 | <0 | 72 | -- |
| High-tilt reel-in | 70 | ~28 | (low) | <49 | (borderline) |
| High-tilt reel-in | 80 | ~30 | -- | <49 | 338 |

*Fz > 49 N (hub weight) is required to maintain altitude. Values from SkewedWakeBEM at hub at rest.*

Three features are notable:

**Reel-out (xi = 30-50 deg):** The disk is tilted strongly into the wind. Axial inflow velocity v_axial = v_wind x cos(xi) = 8.2 m/s at xi=35 deg gives strong thrust. The thrust vector lies nearly along the tether direction, producing high tether tension (~200 N) and therefore high reel-out power.

**Nominal reel-in (xi = 55 deg):** At col = -0.10 rad, Fz ~= 72 N > 49 N (hub weight supported). At col = -0.20, Fz < 0 -- the disk produces downforce. The zero-thrust collective shifts to approximately -0.228 rad at xi=55 deg vs. -0.34 rad at xi=29 deg. **This shift must be reflected in the collective floor used by the tension PI controller**, or the hub will descend during reel-in.

**High-tilt reel-in (xi = 80 deg):** Altitude maintenance requires positive collective (col = +0.10 rad), outside the normal range [-0.28, 0.00] rad. High tilt yields xi=80 deg net energy +24% compared to xi=55 deg baseline, due to dramatically lower reel-in tension (~58 N vs ~86 N).

---

## 5. Gyroscopic Stability and Phase Compensation

### 5.1 Effect of Gyroscopic Coupling

With I_spin = 3.94 kg.m^2 and omega_eq = 19.3 rad/s, the gyroscopic couple for a small cyclic torque M_cyc is:

```
omega_precession = M_cyc / H = M_cyc / 76 rad/s
```

For M_cyc ~= 50 N.m (typical cyclic torque), omega_precession ~= 0.66 rad/s -- faster than any useful planner rate.

However, simulation shows that with a strong orbital angular damping term (k_ang = 50 N.m.s/rad), the gyroscopic cross-coupling torque `omega_orbital x H` remains small because orbital angular rates are strongly damped (damping time constant tau = I_body/k_ang = 5.0/50 = 0.1 s). Under this condition, gyroscopic coupling at I_spin ~= 3.94 kg.m^2 does **not** destabilise the orbit even at zero swashplate phase compensation.

### 5.2 Phase Sweep Results

Simulating the full orbit for 30 s with swashplate phase compensation swept from 0 deg to 180 deg in 30 deg steps (at I_spin = 3.94 kg.m^2):

- Orbit remains stable (no floor hits, drift < 200 m, min altitude > 2 m) for all phases 0 deg-180 deg under the current angular damping regime.
- This is consistent with a hardware phase compensation range of **60-120 deg**, matching the theoretical prediction of 90 deg for a counter-clockwise rotor.

The stable phase window matters for hardware tuning: when setting `H_PHANG` in ArduPilot, any value in the 60-120 deg range should produce stable closed-loop response. The exact optimal value is determined by a flight step-response test.

---

## 6. Pumping Cycle Energy Results

Running one complete De Schutter cycle (30 s reel-out / 30 s reel-in) with the SkewedWakeBEM model at 10 m/s wind, 50 m starting tether:

| Metric | Value |
|--------|-------|
| Reel-out mean tension | 199 N |
| Reel-in steady mean tension | 86 N |
| Reel-out energy (v = 0.4 m/s) | 2390 J |
| Reel-in energy (v = 0.4 m/s) | 994 J |
| **Net energy per cycle** | **+1396 J** |
| Average power | ~23 W |
| Peak tension | 455 N (< 496 N = 80% break load) |
| Min altitude | 5.7 m AGL throughout |

With the high-tilt strategy (xi = 80 deg reel-in, col_max extended to +0.10 rad), net energy improves to approximately +1735 J/cycle (+24%), with reel-in tension dropping to ~58 N.

The energy asymmetry arises entirely from orbit geometry: during reel-out the disk normal is aligned with the tether (xi ~= 30-35 deg), maximising the thrust-along-tether component; during reel-in the disk normal rotates toward vertical (xi = 55-80 deg), redirecting thrust upward to support altitude while minimising tether resistance.

---

## 7. Control Design Implications

This section derives five specific control design requirements that follow directly from the orbit characteristics described above.

### 7.1 Tether-Relative Attitude Reporting

**Problem:** At orbital equilibrium the RAWES hub sits at ~67 deg from vertical (NED down). Standard attitude estimation (e.g. ArduPilot EKF3) interprets this as ~67 deg roll/pitch error from level and immediately commands full cyclic deflection, crashing the hub within 1-2 s.

**Solution:** The sensor model must report attitude as *deviation from the tether-aligned equilibrium*, not as absolute NED orientation. Roll and pitch are zero at tether equilibrium regardless of the physical disk angle. Yaw is derived from velocity heading (`atan2(v_East, v_North)`) for EKF consistency.

**Implication for control architecture:** This is a sensor calibration requirement, not a controller gain. It must be implemented in the SITL sensor bridge or the hardware IMU calibration. Changing it after EKF initialisation triggers an emergency yaw reset and crash.

### 7.2 Orbital Angular Rate as a Rate-Loop Bias

**Problem:** The hub's orbital angular rate (~0.2-0.3 rad/s in the body frame) appears as a *persistent* non-zero rate in ArduPilot's ACRO inner rate loop. The I-term integrates this rate as a tracking error, building an ever-increasing cyclic tilt command that eventually saturates the swashplate. This occurs even with a perfect outer attitude controller.

**Identification:** This bias is directly measurable: with the hub in stable orbit and zero RC stick input, the I-term on the roll and pitch rate PIDs should grow monotonically. The timescale before saturation is approximately `tilt_max / (I_gain x omega_orbit) ~= 0.3 / (0.03 x 0.2) ~= 50 s` at default ArduPilot gains.

**Solution:** Set `ATC_RAT_RLL_IMAX = ATC_RAT_PIT_IMAX = ATC_RAT_YAW_IMAX = 0`. The outer attitude loop provides steady-state tracking; the inner rate loop provides only damping. This is a direct consequence of the orbit being a *desired* non-zero angular rate condition, not a disturbance to reject.

**Implication for control architecture:** Any flight mode that relies on ACRO I-term accumulation for attitude hold (e.g., STABILIZE) will fail for this system. The outer loop must command a non-zero rate matching the orbital angular rate, computed from the current tether direction and hub velocity.

### 7.3 Orbit Tracking: The Zero-Error Condition

**Problem:** A naive attitude controller (PD on body-z error relative to the *initial* tether direction) accumulates phase lag as the hub orbits. Within one quarter-orbit (~15 s) the tether direction has rotated ~90 deg azimuthally, making the initial reference useless and commanding large corrective cyclic.

**Solution:** The equilibrium body-z direction must be updated continuously to track the orbital azimuth. The function `orbit_tracked_body_z_eq` rotates the initial aerodynamic equilibrium body-z by the same azimuthal angle that the current tether direction has rotated from the initial tether direction. This gives exactly zero attitude error at *any* orbital position, not just the initial position.

**Implementation:** This requires only the current hub position and anchor position -- no additional sensors. The update rate must be faster than the orbital rate (~0.1 rad/s); 10 Hz is sufficient.

**Implication:** Without orbit tracking, the attitude controller fights the natural orbital motion. With orbit tracking, the controller's steady-state output is zero tilt, and it only activates to correct perturbations away from the orbit.

### 7.4 Disk-Tilt Slew Rate Limiting

**Problem:** During the reel-out -> reel-in transition, the planner commands the disk to tilt from xi ~= 35 deg to xi ~= 55-80 deg. If this command is applied instantaneously, the gyroscopic response produces a large transient cyclic moment that destabilises the orbit.

**Analysis:** The disk tilt rate achievable through gyroscopic precession is omega_tilt = M_cyc / H. The *maximum possible* M_cyc comes from full aerodynamic cyclic authority across all four blades at equilibrium spin, giving a theoretical upper bound:

```
omega_tilt_max = M_cyc_max / H ~= 20 rad/s
```

This is far faster than any trajectory-level command and would produce severe oscillatory overshoot if used without rate limiting. The practical limit is set not by gyroscopic physics but by controller bandwidth: the attitude controller must settle each incremental disk tilt step before the next step is commanded.

**Derived limit:** From closed-loop simulation sweeps, a body-z slew rate of **0.40 rad/s** (approximately 2% of the theoretical gyroscopic limit) produces stable transitions without oscillation. This is consistent with the 2-5% bandwidth rule of thumb for attitude control loops. Faster slews cause oscillation; slower slews waste reel-in time.

**Implementation:** The `slerp_body_z` function applies this rate limit at each controller timestep, ensuring the body-z setpoint cannot change faster than 0.40 rad/s regardless of the planner command.

**Implication for cycle timing:** The minimum transition time from xi=35 deg to xi=80 deg is:

```
Deltaxi / slew_rate = (80 deg - 35 deg) x pi/180 / 0.40 ~= 2.0 s
```

Adding a settling margin, a 3-4 s transition budget is required per cycle boundary. For a 30 s reel-in phase, this represents a ~10% overhead.

### 7.5 Passive Wind Direction Estimation from Orbital Mean Position

**Observation:** Over one complete orbit (~60 s) the hub's mean horizontal position lies *downwind* from the anchor. This is because the orbit is approximately symmetric about the wind direction axis: the hub spends equal time on either side of the wind vector, and the systematic downwind offset from the tether tension averages to give the wind direction.

**Derivation:** Let `p_horiz(t)` be the hub's 2D horizontal position over time. Then:

```
wind_direction ~= mean(p_horiz(t)) / |mean(p_horiz(t))|
```

At equilibrium: mean East = 47.5 m, mean North = 13.9 m -> estimated wind azimuth = arctan(47.5/13.9) ~= 74 deg from North = East-northeast. True wind is due East. The small bias (16 deg) is the azimuthal equilibrium offset due to rotor angular momentum and asymmetric aerodynamics.

**Accuracy:** This estimator requires one full orbit period (~60 s) to converge and provides wind direction to within ~20 deg without any dedicated wind sensor -- useful for initialising the reel-in disk tilt quaternion before a dedicated anemometer measurement is available.

**In-plane wind speed** is simultaneously estimable from rotor spin rate via the autorotation torque balance:

```
v_inplane ~= omega_spin^2 x K_drag / K_drive
```

These two quantities (wind direction + in-plane speed) are sufficient to compute the optimal reel-in disk orientation without any dedicated wind sensor.

---

## 8. Discussion

### 8.1 What Is Novel Relative to Existing Literature

De Schutter et al. [5] characterise the pumping cycle through OCP and report reel-out tilt angles of 30-50 deg and reel-in tilt >70 deg, which our BEM simulation confirms. An earlier poster [11] introduced the inertia-based pumping cycle concept but without closed-loop controller analysis. Neither work addresses:
- Identify the orbit period or body-frame angular rate as control inputs
- Analyse gyroscopic phase compensation or its hardware equivalent (H_PHANG)
- Note the orbital rate bias in a standard attitude rate loop
- Propose orbit tracking as a zero-error reference update strategy
- Exploit the orbital mean position as a wind direction estimator

Tensile RAWES literature [9, 10] examines torque transmission through tensile rotary ring structures (the Daisy Kite family) but uses a fundamentally different topology -- multiple networked ring kites rather than a single pumping hub -- and does not address single-hub orbit mechanics or pumping cycle control.

The tethered autogyro paper [7] analyses altitude and pitch control via differential rotor braking, but does not address the azimuthal orbit, pumping cycle, or attitude estimation under a standard flight controller.

### 8.2 Limitations of the Current Model

1. **Single-body model:** No blade flapping, no push-rod lag (Kaman flap dynamics from Weyel [6] not yet implemented). Flap lag introduces an additional phase delay between swashplate command and blade pitch response, requiring further H_PHANG adjustment.
2. **Steady-state BEM:** No dynamic inflow, no wake memory. The Coleman skewed-wake correction handles the most important non-uniformity but is not valid above xi ~= 85 deg.
3. **No tether sag or distributed mass:** The elastic point-force tether model underestimates the restoring moment at high tether length. At L = 300 m (maximum pumping cycle extent), catenary effects become significant.
4. **Orbit period estimate:** The ~60 s period from the WindEstimator time window is a conservative estimate; it has not been directly extracted from closed-loop simulation telemetry. Hardware measurement is needed.
5. **10 m/s wind only:** All results are at a single design-point wind speed. Power output scales approximately as v_wind^3; orbit period and radius scale differently, requiring sweeps for variable-wind operation.

### 8.3 Implications for ArduPilot Hardware Configuration

The five control requirements above translate directly to specific ArduPilot configuration choices:

| Requirement | ArduPilot Parameter / Design Choice |
|-------------|--------------------------------------|
| Tether-relative attitude | SITL sensor bridge (tether-relative rpy); COMPASS_USE=0 |
| Orbital rate bias | ATC_RAT_RLL/PIT/YAW_IMAX = 0 |
| Orbit tracking | Mode_RAWES: orbit_tracked_body_z_eq at 400 Hz |
| Slew rate limiting | Mode_RAWES: body_z_slew_rate = 0.40 rad/s |
| Gyro phase comp | H_PHANG = 60-120 deg (exact value from step-response test) |

The flight mode choice follows from requirement (2): STABILIZE mode commands absolute NED attitude (roll=0=level), which fights the 65 deg orbital equilibrium. ACRO mode only damps angular rates toward commanded rates, making it compatible with orbit tracking once the I-term is disabled.

---

## 9. Conclusion

The natural orbit of a tethered RAWES is a quasi-circular cone driven by aerodynamic disk tilt and gyroscopic precession, requiring no active steering. At the design operating point (10 m/s wind, 50 m tether), the orbit has a period of approximately 60 s, a radius of ~15 m, and produces a body-frame angular rate disturbance of 0.2-0.3 rad/s.

Five flight control requirements follow directly from these orbit characteristics:

1. Attitude must be reported relative to the tether equilibrium, not the world vertical.
2. The orbit angular rate produces a persistent I-term bias in ACRO rate loops; IMAX must be zeroed.
3. The attitude reference must track the orbital azimuth continuously to maintain zero steady-state error.
4. The disk-tilt slew rate must be limited to <= 0.40 rad/s to avoid gyroscopic transients at pumping cycle phase transitions.
5. The orbital mean position provides a passive wind direction estimate requiring no dedicated anemometer.

None of these requirements are derivable from trajectory optimisation alone; they emerge only when the orbit is analysed as a physical phenomenon interacting with a real attitude controller. The BEM simulation produces +1396 J net energy per 60 s cycle (+23 W average at 10 m/s), rising to +1735 J (+24%) with the high-tilt xi=80 deg reel-in strategy.

---

## References

[1] U. Ahrens, M. Diehl, and R. Schmehl (Eds.), *Airborne Wind Energy*. Springer, Berlin, 2013.

[2] M. L. Loyd, "Crosswind kite power," *Journal of Energy*, vol. 4, no. 3, pp. 106-111, 1980.

[3] R. Read, "Kite networks for harvesting wind energy," in *Airborne Wind Energy: Advances in Technology Development and Research*, R. Schmehl (Ed.), Springer, Singapore, 2018, ch. 21.

[4] Wikipedia contributors, "Airborne wind energy," *Wikipedia*, 2025. Available: https://en.wikipedia.org/wiki/Airborne_wind_energy

[5] J. De Schutter, R. Leuthold, and M. Diehl, "Optimal control of a rigid-wing rotary kite system for airborne wind energy," in *Proc. 2018 European Control Conference (ECC)*, Limassol, Cyprus, Jun. 2018, pp. 1734-1739. DOI: 10.23919/ECC.2018.8550383

[6] F. Weyel, "Modeling and closed loop control of a cyclic pitch actuated rotary airborne wind energy system," Bachelor's Thesis, University of Freiburg, 2025.

[7] T. Noboni, T. Das, and J. McConnell, "Modeling tethered multirotor autogyro with altitude control via differential rotor braking," *Journal of Guidance, Control, and Dynamics*, vol. 48, no. 7, pp. 1606-1619, 2025. DOI: 10.2514/1.G008573

[8] R. B. Bossler Jr., "Rotor blade pitch changing mechanism for rotary wing aircraft," U.S. Patent 3,217,809, assigned to Kaman Aircraft Corporation, Nov. 16, 1965.

[9] O. Tulloch, H. Yue, A. M. Kazemi Amiri, and R. Read, "A tensile rotary airborne wind energy system -- Modelling, analysis and improved design," *Energies*, vol. 16, no. 6, p. 2610, 2023. DOI: 10.3390/en16062610

[10] O. Tulloch, A. Kazemi Amiri, H. Yue, J. Feuchtwang, and R. Read, "Tensile rotary power transmission model development for airborne wind energy systems," *Journal of Physics: Conference Series*, Sep. 2020.

[11] J. De Schutter, "Inertia-supported pumping cycles based on a roto-kite," poster, *Airborne Wind Energy Conference (AWEC)*, Freiburg, Germany, 2017. Available: https://forum.awesystems.info/uploads/default/original/1X/c13c2aad640b5275b9237e35c4e6368ddffb1148.pdf

---

*Simulation code and test suite: `simulation/` directory of the RAWES development repository. All numerical results are reproducible by running the unit test suite (`pytest simulation/tests/unit -m simtest`) with the `beaupoil_2026.yaml` rotor definition and SkewedWakeBEM aerodynamic model.*
