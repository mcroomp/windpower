# RAWES Simulation Internals

Detailed technical reference for the simulation stack. See [CLAUDE.md](../CLAUDE.md) for the overview and running tests.

---

## Natural / Equilibrium Hub Orientation

**The rotor axle (body Z) always aligns with the tether direction.** At tether elevation angle β:
```
body Z = [cos(β), 0, sin(β)]   (ENU, hub East of anchor)
```

Always initialise the hub with body Z along the tether, not upright. The `build_orb_frame(body_z)` utility constructs a valid R0 from any body_z unit vector.

---

## Sensor Design — Tether-Relative Attitude

**Critical design decision:** attitude is reported as the **deviation from the tether equilibrium orientation**, not absolute orientation relative to world Up.

**Why:** The RAWES equilibrium has body Z at ~30° elevation — ~65° from vertical. Reporting absolute attitude causes ArduPilot to see 58° roll / 36° pitch at rest and command maximum cyclic, crashing the hub within 1 second.

**How** (`sensor.build_sitl_packet`):
- `rpy = [0, 0, yaw_from_velocity]` — roll and pitch are always zero (tether-relative)
- yaw is derived from velocity heading so EKF attitude and velocity are consistent
- gyro has spin stripped; rotated into yaw-aligned body frame
- accel is kinematic accel in NED body frame plus −g correction

**EKF consequence:** ArduPilot interprets deviations from tether equilibrium as small roll/pitch errors and corrects with modest cyclic, not maximum deflection.

**Sensor consistency rules (must all agree or EKF triggers emergency yaw reset):**
1. `velocity_ned` heading must match `rpy[2]` (yaw) — both derived from `atan2(vE, vN)`
2. `gyro_body` must be in yaw-aligned NED body frame (spin stripped, then `Rz(-yaw) @ (T @ omega_nospin)`)
3. `accel_body` must be `Rz(-yaw) @ (T @ accel_world + [0,0,-9.81])`

---

## Controller Design

`simulation/controller.py` provides three functions with different use cases:

### `compute_swashplate_from_state(hub_state, anchor_pos, ...)` — truth-state controller

Takes hub ENU state (pos, R, omega) directly. Computes attitude error as `cross(body_z_cur, body_z_eq)` and damps orbital rates. Returns `{collective_rad, tilt_lon, tilt_lat}` to feed directly into `aero.compute_forces()`. **No ArduPilot involved.**

Used by: `test_closed_loop.py` (closed-loop physics validation)

### `compute_rc_rates(hub_state, anchor_pos, vel_ned, ...)` — ENU truth-state RC controller

Same logic as above but outputs RC PWM dict `{1: pwm, 2: pwm, 4: pwm, 8: 2000}` for ArduPilot's ACRO rate loop. Converts correction to yaw-aligned NED body frame.

### `compute_rc_from_attitude(roll, pitch, rollspeed, pitchspeed, yawspeed, ...)` — MAVLink controller

Works directly with ArduPilot ATTITUDE message fields. Since `sensor.build_sitl_packet` reports tether-relative attitude, `roll` and `pitch` ARE the attitude error angles. Returns RC PWM dict.

Used by: `test_guided_flight.py` (ACRO hold loop via RC override, ~10 Hz from MAVLink)

**Important limitation:** The ACRO hold controller runs at ~10 Hz via MAVLink messages. The physics diverges at 400 Hz. The truth-state controller (`compute_swashplate_from_state`) operates at full 400 Hz and is the correct long-term architecture.

---

## Dynamics Model

`simulation/dynamics.py` — RK4 6-DOF rigid-body integrator.

| Parameter | Value | Location |
|-----------|-------|----------|
| Timestep | 2.5e-3 s (400 Hz) | `DT_TARGET` in mediator.py |
| Mass | 5.0 kg | mediator.py |
| Ixx = Iyy | 5.0 kg·m² | mediator.py |
| Izz | 10.0 kg·m² | mediator.py |
| I_spin | 0.0 | mediator.py (gyroscopic coupling disabled — see comment) |
| Initial pos | `[46.258, 14.241, 12.530]` ENU m | `DEFAULT_POS0` |
| Initial vel | `[-0.257, 0.916, -0.093]` m/s | `DEFAULT_VEL0` |
| Initial body_z | `[0.851, 0.305, 0.427]` | `DEFAULT_BODY_Z` |
| Initial spin | 20.148 rad/s | `DEFAULT_OMEGA_SPIN` |

Gravity is applied internally — do **not** add it to forces.

**Why I_spin = 0:** The lumped single-body model uses ArduPilot's cyclic commands as tilting moments. The 90° gyroscopic precession a real spinning rotor produces is already compensated by swashplate phase angle in real hardware. Adding gyroscopic coupling here without the corresponding phase compensation would cause ArduPilot's attitude controller to tilt the disk in the wrong direction.

**Rotor spin** is maintained as a separate scalar `omega_spin`, updated each step via:
```
Q_net = K_DRIVE_SPIN × v_inplane − K_DRAG_SPIN × omega_spin²
omega_spin += Q_net / I_SPIN_KGMS2 × dt
```
This gives a stable equilibrium at `omega_eq = sqrt(K_DRIVE × v_inplane / K_DRAG)`.

---

## Aerodynamic Model

`simulation/aero.py` — De Schutter (2018) lumped-blade BEM model.

**What it does:**
- Computes relative wind = ambient wind − hub velocity
- Estimates induced velocity via actuator-disk momentum theory
- Integrates lift and drag over 20 radial strips, root to tip
- CL = CL0 + CL_alpha × AoA; CD = CD0 + CL²/(π·AR·Oe)
- AoA clamped to ±15° for linear model validity
- Produces: thrust (along disk normal), drag torque (about disk axis), cyclic moments (proportional to swashplate tilt)
- 5 s startup ramp to avoid impulsive loads

**What it does not do:** dynamic inflow, tip-loss, wake swirl, flap-lag coupling, tether aero, true flap-to-pitch dynamics.

`compute_anti_rotation_moment()` exists but is **not called** in the current single-body model — motor torque is an internal force and cancels. Reserved for future two-body model.

---

## Tether Model

`simulation/tether.py` — `TetherModel` class.

| Property | Value |
|----------|-------|
| Material | Dyneema SK75 1.9 mm braided UHMWPE |
| EA (axial stiffness) | ~281 kN |
| k(L) | EA / L — nonlinear, stiffer when shorter |
| Linear mass | 2.1 g/m |
| Break load | ~620 N |
| Structural damping | 5 N·s/m (extension only) |
| Rest length | 49.949 m default (taut at steady-state equilibrium) |
| Anchor | World origin (0, 0, 0) ENU |

When tether is slack (hub closer than rest length) → zero force. Logs warning when tension exceeds 80% of break load.

Restoring moment (`r_attach × F_tether`) is supported but currently disabled (`axle_attachment_length=0.0`) — body_z stability comes from aerodynamics, not tether moment.

---

## Initial State and `steady_state_starting.json`

The mediator's default initial state is the warmup-settled equilibrium from `test_steady_flight.py`:

| Parameter | Value | Source |
|-----------|-------|--------|
| `pos0` | `[46.258, 14.241, 12.530]` ENU m | 50 m tether at ~30° elevation |
| `vel0` | `[-0.257, 0.916, -0.093]` m/s | settled hub velocity |
| `body_z` | `[0.851, 0.305, 0.427]` | axle aligned with tether |
| `omega_spin` | `20.148` rad/s | equilibrium autorotation spin |
| `rest_length` | `49.949` m | tether taut from t=0 |

**Workflow for regenerating:**
1. `pytest simulation/tests/unit/test_steady_flight.py` — writes `simulation/steady_state_starting.json`
2. Stack test reads this file and passes values to mediator via `--pos0`, `--vel0`, `--body-z`, `--omega-spin`
3. If absent, mediator uses the built-in defaults above

---

## Startup Freeze

The mediator holds the hub stationary for `--startup-freeze-seconds` (default: 30 s) so the ArduPilot EKF achieves GPS position lock before physics starts.

During freeze:
- Hub state is not updated
- A 0.15 m/s hint velocity is sent in the equilibrium heading direction so the EKF converges yaw before physics starts (prevents "Yaw Imbalance" / emergency yaw reset when freeze ends)
- Real position is sent (no drift)

---

## Known Gaps Between Source Thesis Model And Actual Hardware

| Item | Thesis model | Actual design |
|------|-------------|--------------|
| Blade count | 3 | 4 |
| Blade length | 1.5 m | 2.0 m |
| Cyclic phase offset | 2π/3 (120°) | π/2 (90°) |
| Rotor mass | 40 kg | 5 kg |
| Flap actuation | Individual servo per blade | Swashplate push-rods (mechanical) |
| Anti-rotation | Not modeled | GB4008 + 80:44 gear |

Additional physics limitations in the source thesis model:
- Controller uses fixed system matrices — does not adapt to changing wind
- Tether force not modeled (only stabilizing moment approximated)
- Tip vortices, wake effects, induced drag neglected
- Rotor tilt >15° causes significant pitch tracking error

---

## Background Academic References

1. **Felix Weyel (2025)** — "Modeling and Closed Loop Control of a Cyclic Pitch Actuated Rotary Airborne Wind Energy System", Bachelor's Thesis, Uni Freiburg.
2. **De Schutter, Leuthold, Diehl (2018)** — "Optimal Control of a Rigid-Wing Rotary Kite System for Airborne Wind Energy".
3. **US Patent US3217809** (Kaman/Bossler, 1965) — Canonical servo-flap rotor control system.
