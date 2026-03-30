# RAWES Simulation — Development History

Phase 2 and Phase 3 M3 decisions. See [CLAUDE.md](../CLAUDE.md) for current status and [raws_mode.md](raws_mode.md) for the full ModeRAWES spec.

---

## Phase 2 — Internal Controller: What Was Accomplished

### Bug fixed: `compute_swashplate_from_state` tilt_lon sign error

The longitudinal cyclic correction was applied with the wrong sign.  The aero
model applies `Mx_body = -K × tilt_lon × T` (negative sign), so a positive
`tilt_lon` produces a westward torque that tilts body_z **north**.  The
controller was projecting the error onto `disk_x` (eastward), giving
`tilt_lon < 0` when the hub needed to tilt north — the exact opposite.  Fixed
by negating the projection: `tilt_lon = -dot(corr_enu, disk_x) / tilt_max_rad`.

The tilt_lat sign was already correct (aero uses `My_body = +K × tilt_lat × T`).

### 60 s closed-loop stability proven (`test_closed_loop_60s.py`)

Four tests, all passing:
- `test_60s_altitude_maintained` — 0 floor hits, z rises from 12.5 m → 13.3 m over 60 s
- `test_60s_no_runaway` — hub drift stays below 200 m
- `test_60s_spin_maintained` — autorotation spin 20→21 rad/s (sustained)
- `test_zero_tilt_at_equilibrium` — controller outputs exactly zero tilt at POS0/BODY_Z0

Key design decisions validated:
- `axle_attachment_length=0.0` matches mediator (no tether restoring torque; attitude via aero only)
- `T_AERO_OFFSET=45.0` — aero ramp already complete at kinematic-phase end; passed as `t=45+t`
- **Orbit-tracking body_z_eq**: body_z_eq rotates azimuthally as hub orbits, keeping error=0 at
  equilibrium throughout the orbit.  Without this, body_z_eq drifts from the hub's actual
  aerodynamic equilibrium as it orbits and the controller generates ever-growing spurious tilt.

### Orbit-tracking formula (used in mediator and all unit tests)

```python
# Capture at free-flight t=0:
tether_dir0 = pos0 / |pos0|
body_z_eq0  = R0[:, 2]

# Each step:
th0h = [tether_dir0.x, tether_dir0.y, 0]  (horizontal projection of initial tether dir)
thh  = [cur_tdir.x,    cur_tdir.y,    0]  (horizontal projection of current tether dir)
# normalise both, compute azimuthal rotation angle phi
cos_phi = dot(th0h, thh)
sin_phi = th0h.x * thh.y - th0h.y * thh.x
body_z_eq = rotate_z(body_z_eq0, phi)    # rotate initial eq body_z by orbital azimuth
```

### Key physics insight: zero tilt = natural stability

With `axle_attachment_length=0.0` and the aero ramp complete:
- Zero tilt → `M_orbital = M_total - M_spin = 0` → no orbital moment → hub is perfectly stable
- The hub needs **no controller at all** to maintain equilibrium — the aerodynamic equilibrium
  is self-stable.  The controller only corrects perturbations.

The open-loop diagnostic (`_diag2.py`) confirmed: 60 s, 0 floor hits, body_z_z=0.427
constant, z=12.82 m constant, spin=20.3 rad/s constant — identical to the closed-loop result.

### Pumping cycle — naive collective-only approach does not work

`test_pumping_cycle.py` tests a collective-only strategy (no tilt change between phases):
- Reel-out mean tension: 261 N, reel-in mean tension: 288 N → reel-in > reel-out → net energy negative
- Root cause: tether tension during reel-in equals the force to pull hub inward against its
  aerodynamic equilibrium, which is dominated by the full aerodynamic force regardless of collective.
- Collective modulation alone cannot achieve the asymmetry needed for net positive energy.

### Pumping cycle — De Schutter tilt strategy works (`test_deschutter_cycle.py`)

Implements De Schutter (2018) Fig. 4: tilt ξ from wind direction changes between phases.

**Reel-out:** body_z aligned with tether, ξ ≈ 31° from wind, high collective via PI controller
- Mean tension: 233 N, max: 270 N, energy generated: 2791 J

**Reel-in:** body_z transitions to vertical [0,0,1] over 5 s (ξ = 90° from wind, >70° criterion met)
- Thrust acts upward (fighting gravity), not along tether → aerodynamic resistance to winch ≈ 0
- Tension drops to mean: 55 N (winch only overcomes gravity component along tether ~17 N + inertia)
- Energy consumed: 854 J

**Net energy: +1937 J** — fundamental AWE pumping condition satisfied.

Five tests all passing:
- `test_deschutter_no_crash` — 0 floor hits throughout
- `test_deschutter_reel_in_lower_tension` — 55 N < 233 N ✓
- `test_deschutter_net_energy_positive` — +1937 J ✓
- `test_deschutter_reel_in_tilt_achieved` — steady ξ = 90° ≥ 60° ✓
- `test_deschutter_reel_out_tilt_in_range` — ξ = 31° within 25–60° ✓
- `test_deschutter_tether_not_broken` — peak 270 N < 496 N (80% break load) ✓

### Pumping cycle control architecture (three independent loops at 400 Hz)

```
Attitude controller  →  tilt_lon, tilt_lat   (compute_swashplate_from_state)
                         body_z_eq = tether-aligned (reel-out) or vertical (reel-in)
Tension controller   →  collective_rad        (TensionController PI on tether tension)
Winch controller     →  tether.rest_length    (±= v_reel × DT each step)
```

The **attitude controller's body_z_eq setpoint is the primary lever** for tension control,
not collective.  Collective provides fine-grained tension tuning within a phase.

### Mediator wiring (completed in Phase 2)

`mediator.py` was updated with orbit-tracking state captured at free-flight start:
- `_ic_tether_dir0` and `_ic_body_z_eq0` captured once when `_damp_alpha == 0.0` first triggers
- Body_z_eq tracked azimuthally each step (same formula as unit tests)
- Internal controller branch calls `compute_swashplate_from_state` with orbit-tracked body_z_eq
- collective_rad = 0.0 (pumping cycle collective control not yet wired into mediator)

---

## Phase 3 M3 — ModeRAWES Architecture: What Was Decided

Full spec in `simulation/raws_mode.md`. Key settled decisions:

### ModeRAWES inherits ModeAcro_Heli (not Mode directly)
```
Mode → ModeAcro → ModeAcro_Heli → ModeRAWES
```
Only `run()`, `init()`, and 5 metadata overrides needed. Spool-state guards delegate to `ModeAcro_Heli::run()`. ~162 lines new C++.

### 400 Hz loop (run()) pseudocode
```
1. Spool guards: SHUT_DOWN/GROUND_IDLE → ModeAcro_Heli::run(); return
2. Planner timeout (2 s) → snap bz_target back to bz_tether
3. Orbit tracking → update bz_tether from current tether direction
4. Attitude setpoint: identity attitude_q → use bz_tether; else slerp toward bz_target
5. Cyclic: error = cross(body_z_now, body_z_eq); rate_bf = kp × err_body → ATC_RAT_RLL/PIT
6. Collective: set_throttle_out(_thrust_cmd, false, filt)  ← direct passthrough from ground PI
7. Counter-torque: yaw rate = 0.0f → ATC_RAT_YAW → GB4008 (H_TAIL_TYPE=4)
8. omega_spin: AP_ESC_Telem.get_rpm() × 2π/60 × 44/80 (11 pole pairs, 80:44 gear)
9. send_state() at 10 Hz → Pixhawk→Planner STATE packet
```

### Protocol (MAVLink, 10 Hz)
- **Planner → Pixhawk:** `SET_ATTITUDE_TARGET` — `attitude_q` (ENU quaternion attitude setpoint), `thrust` (normalized collective 0..1 from ground PI)
- **Pixhawk → Planner (all standard streams, zero custom code):** `LOCAL_POSITION_NED` (pos + vel), `ATTITUDE_QUATERNION` (body_z_ned = quat_apply(q,[0,0,1])), `ESC_STATUS[RAWES_CTR_ESC]` (omega_spin — planner applies `× 2π/60 / 11 × 44/80`)
- No `send_state()` function needed in `Mode_RAWES`
- Tension and tether_length are **NOT** in the STATE packet — Pixhawk cannot measure them. Winch reads both locally.

### Tension PI on ground (not Pixhawk)
```python
error          = tension_setpoint_n - tension_measured_n   # fresh local measurement
collective_rad = kP * error + kI * integral                # ground config: kP=5e-4, kI=1e-4
thrust         = clamp((collective_rad - col_min_rad) / (col_max_rad - col_min_rad), 0, 1)
# col_min = -25°, col_max = 0° for beaupoil_2026
# thrust sent in SET_ATTITUDE_TARGET.thrust field
```
**Why ground:** load cell is at winch; 10 Hz radio latency is fine for slow collective changes; winch is the fast tension regulator.

### Pixhawk parameters (6)
```
RAWES_KP_CYC       = 1.0      # cyclic rate gain [rad/s per rad error]
RAWES_BZ_SLEW      = 0.12     # body_z setpoint slew rate [rad/s]
RAWES_CTR_ESC      = 3        # AP_ESC_Telem index for GB4008 counter-torque motor
RAWES_ANCHOR_LAT   = 0.0      # tether anchor latitude [deg]
RAWES_ANCHOR_LON   = 0.0      # tether anchor longitude [deg]
RAWES_ANCHOR_ALT   = 0.0      # tether anchor altitude AMSL [m]
```
Anchor params allow any launch style (ground, hand-launch, mid-air entry).
All tension PI params (KP, KI, col_min, col_max, tension setpoint) live in ground config only.

### omega_spin measurement
AM32 ESC telemetry via `AP_ESC_Telem`:
```
omega_spin = get_rpm(RAWES_CTR_ESC) × 2π/60 / 11 × 44/80
```
(11 pole pairs, 80:44 gear reduction)
