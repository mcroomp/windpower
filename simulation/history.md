# RAWES Simulation — Development History

Phase 2, Phase 3 M3 architecture decisions, and Phase 3 M3 SkewedWakeBEM production switch. See [CLAUDE.md](../CLAUDE.md) for current status and [system/stack.md](../system/stack.md) for the full flight control stack spec.

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

### 60 s closed-loop stability proven (`test_closed_loop_90s.py`)

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

### Mediator wiring (completed in Phase 2, extended in Phase 3 M3)

`mediator.py` was updated with orbit-tracking state captured at free-flight start:
- `_ic_tether_dir0` and `_ic_body_z_eq0` captured once when `_damp_alpha == 0.0` first triggers
- Body_z_eq tracked azimuthally each step (same formula as unit tests)
- Internal controller branch calls `compute_swashplate_from_state` with orbit-tracked body_z_eq
- Collective from `DeschutterPlanner` via `_traj_cmd["collective_rad"]` (wired in Phase 3 M3)

---

## Phase 3 M3 — ModeRAWES Architecture: What Was Decided

Full spec in `system/stack.md`. Key settled decisions:

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
- **Planner → Pixhawk:** `SET_ATTITUDE_TARGET` — `attitude_q` (NED quaternion attitude setpoint), `thrust` (normalized collective 0..1 from ground PI)
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

---

## Phase 3 M3 — SkewedWakeBEM Production Switch: What Was Decided

### RotorAero replaced by SkewedWakeBEM

`RotorAero` had three fundamental physics errors that made it incompatible with the pumping cycle:

1. **Wrong cyclic sign**: empirical `K_cyc × tilt × T` had the sign tuned to work with the old controller. Per-blade BEM gives the opposite cyclic sign — `tilt_lat > 0` produces `−My`, not `+My`. The controller sign convention was updated to match (`tilt_lat = −dot(corr_enu, disk_y)`).

2. **Wrong H-force**: `0.5 × μ × T` ≈ 105 N vs SkewedWakeBEM ≈ 13 N. A factor of 8× over-estimate.

3. **No physical spin model**: `K_drive × v_inplane − K_drag × omega²` empirical constants with no BEM basis.

**SkewedWakeBEM** uses per-blade strip BEM with Prandtl tip/root loss and Coleman skewed-wake induction. Returns `AeroResult` dataclass with `F_world`, `M_orbital`, `Q_spin`, `M_spin`. The `Q_spin` field drives the omega_spin ODE directly — no empirical constants.

**Speed:** SkewedWakeBEM is ~3.3× slower than RotorAero per call (~0.5 ms vs ~0.15 ms), but comfortably runs in real-time at 400 Hz on the simulation host.

### Aero model parameter attribution corrections

Both `RotorAero` and earlier code had wrong airfoil parameters attributed to academic sources:

- **De Schutter 2018**: `CL0=0.0, CL_alpha=5.385/rad` (thin airfoil formula `2π`-ish), `R=3.1 m`, `r_root=1.6 m` (L_b beam arm). Earlier code had `CL_alpha=0.87/rad` (a Weyel plot approximation) and wrong geometry.
- **Weyel 2025** bench-test data is for SG6040 at Re=127,000 — not directly applicable to beaupoil_2026 (SG6042, Re≈490,000). The `weyel_2025.yaml` rotor definition was removed.

### Phase-dependent COL_MIN (critical finding)

The zero-thrust collective is geometry-dependent:
- **Tether-aligned body_z** (reel-out, ξ≈31°): zero-thrust at **−0.34 rad** → `col_min=−0.28` safe
- **ξ=55° body_z** (reel-in): zero-thrust at **−0.228 rad** → `col_min=−0.28` gives **downforce** → hub falls

Solution: `DeschutterPlanner` switches `col_min` at the phase boundary:
- Reel-out: `col_min=−0.28`
- Reel-in: `col_min=−0.20` (20 mrad above zero-thrust at ξ=55°)

The PI integral is re-warmed to `col_min_reel_in / ki` at the phase boundary to avoid integral wind-up from the reel-out phase.

### Collective passthrough: normalized thrust vs raw collective_rad

**Bug found and fixed:** `DeschutterPlanner` normalised collective to `thrust ∈ [0,1]` using `col_min_reel_in=−0.20`, but the mediator always denormalised using `col_min_reel_out=−0.28`. During reel-in this mapped the planner's floor (−0.20) to −0.28 in the aero model — below zero-thrust → downforce → crash.

**Fix:** `DeschutterPlanner.step()` now returns `collective_rad` directly alongside `thrust`. The mediator uses `collective_rad` when present, bypassing the normalization roundtrip entirely.

### vel0 must not come from steady_state_starting.json

`steady_state_starting.json` stores the physics-settled velocity (≈ 0 m/s at equilibrium). If the mediator uses this as `vel0`, the kinematic startup ramp has zero velocity — the EKF gets no velocity-derived yaw heading from frame 0 → compass flipping → no GPS lock.

**Rule:** `vel0` always comes from `config.py DEFAULTS = [-0.257, 0.916, -0.093]` m/s. This is the designed startup velocity tangent to the tether orbit (~0.96 m/s). The stack test helper (`_launch_mediator`) explicitly does **not** override `vel0` from `initial_state`.

### EKF altitude unreliable during pumping cycle

GPS glitch events occur during the rapid body_z transition at reel-in start (hub accelerates ≈5 m upward in 2 s, compass switches compasses repeatedly as the SITL magnetometer rotates with the hub). During these events `LOCAL_POSITION_NED` altitude (EKF output) can drop by 5–10 m below physics truth.

**Rule:** Use mediator telemetry `hub_pos_z` (physics truth) for crash detection. `LOCAL_POSITION_NED` altitude is logged as diagnostic only.

### Stack test results (SkewedWakeBEM, beaupoil_2026, wind=10 m/s East)

```
Reel-out mean tension   : 199 N   (setpoint 200 N)
Reel-in steady mean     :  86 N   (setpoint  80 N)
Net energy per cycle    : +1396 J
Peak tension            :  455 N  (< 496 N = 80% break load)
Min physics altitude    :   5.7 m (> 2.0 m limit)
```

---

## Post-M3 Infrastructure Changes

### test_acro_hold known failure (not a regression)

`test_acro_hold` fails with "Hub crashed: altitude < 2.0 m". The hub descends from ~7 m to ~1.3 m over 60 s of neutral-stick ACRO hold after the 45 s kinematic damping phase. Unit-level equivalent (`test_closed_loop_90s`) passes.

**Root cause:** Kinematic phase ends with ~0.9 m/s orbital velocity. `HoldPlanner` → `thrust=0.0 → collective=col_min=−0.28 rad` provides barely-positive lift at shallow tether elevation (~8°). Hub spirals down. `test_closed_loop_90s` starts from near-zero IC velocity so lift is sufficient.

### Output paths moved to simulation/logs/

All runtime artifacts (logs, telemetry CSVs, analysis PNGs, unit test artefacts) now write to `simulation/logs/` instead of the simulation root or source directories.

### Stack test SITL cleanup: _kill_by_port()

`sim_vehicle.py` spawns `arducopter-heli` in a new process group so `os.killpg()` left it running and holding port 5760. `_kill_by_port()` in `stack_utils.py` finds and SIGKILLs the process via `/proc/net/tcp` inode lookup. Called in all teardowns after `_terminate_process()`. Port check now retries for 15 s instead of failing immediately.

### stack_utils.py: central stack test helpers

Shared infrastructure moved from `conftest.py` and `test_stack_integration.py` to `simulation/tests/stack/stack_utils.py`: env-var constants, `_configure_logging`, `copy_logs_to_dir`, `check_ports_free`, `_resolve_sim_vehicle`, `_launch_sitl`, `_terminate_process`, `_kill_by_port`. `test_stack_integration.py` re-exports for backward compatibility.

---

## Post-M3 — Aero Model Clarity & De Schutter Validation

### aero_skewed_wake.py rewritten as clear reference

The non-JIT version had accumulated intermediate optimisations (numpy broadcasting over
`(N_AB, N_RADIAL, 3)`, manual cross products, `valid` mask without `continue`) that obscured
the physics. Since its only purpose is to be the human-readable reference for the JIT version,
it was rewritten with:
- Explicit double for-loop: `for i in range(N_AB): for j in range(N_RADIAL):`
- `np.cross()` for all cross products
- `_prandtl_F()` helper function
- `continue` statements for invalid strips (no `valid` mask accumulation)
- Removed dead `Q_strip` and `spin_sign` code
- Full class docstring with 8-step algorithm summary
- Per-method and per-step inline physics comments

All 18 JIT equivalence tests (`test_skewed_wake_jit.py`, `atol=1e-10`) pass after the rewrite.

### SkewedWakeBEMJit — Numba JIT fast path

`aero/aero_skewed_wake_jit.py` implements `SkewedWakeBEMJit`, a drop-in `SkewedWakeBEM`
subclass that overrides `compute_forces` with two Numba `@njit` kernels:
- `_jit_vi0`: induced velocity bootstrap (3 iterations at R_CP)
- `_jit_strip_loop`: full (N_AB × N_RADIAL) strip loop → F, M, Q_spin accumulation

Select with `create_aero(model="jit")`. First call triggers JIT compilation (~2 s);
subsequent calls are C-speed. Equivalence to reference verified to `atol=1e-10`.

### DeSchutterAero — paper equation audit

`DeSchutterAero` (in `aero_deschutter.py`) was audited against all aerodynamic
equations in De Schutter et al. (2018).

**Two new features implemented:**

1. **β side-slip (Eq. 27, 28) as diagnostic:**
   `beta_rad = arcsin(ua_span / |ua|)` computed per strip and stored as
   `last_sideslip_mean_deg`. The paper constrains |β| ≤ 15° for BEM validity.
   β does NOT appear in the lift or drag formulas — the Kutta-Joukowski cross
   product `ua × e_span` already handles the geometry. β is a post-hoc validity
   check only.

2. **C_{D,T} structural parasitic drag (Eq. 29, 31):**
   Added to blade CD: `CD_total = CD + self.CD_T` where `CD_T = CD_structural`
   from the rotor YAML.  For de_schutter_2018: `C_{D,T} = 0.021`, derived from
   `C_D_cyl(1.0) × d_cable(1.5 mm) × L_cable(2.60 m) / S_w(0.1875 m²)`.
   The paper gives cable dimensions in Table I but not the final C_{D,T} value.
   For beaupoil_2026: `CD_structural = 0.0` (blades attach to hub via spar).

3. **RotorDefinition.CD_structural field** added to `rotor_definition.py` and
   both YAML files.

**Bug fixed — induction bootstrap floor:**
`_induced_velocity()` had `T_abs = max(abs(T_guess), 0.01)`. At zero collective +
symmetric airfoil, the 0.01 N floor seeded a nonlinear iteration that converged to
~4 m/s induced velocity and ~200 N phantom thrust. Correct formula uses
`T_abs = abs(T_guess)` — when T=0, discriminant reduces to v_axial^2 and v_i=0
correctly. Normal flight cases (T >> 0.01) are unaffected.

### test_deschutter_equations.py — 32 equation-level validation tests

New test file mapping paper equations to implementation and YAML values:

| Test class | Paper equations | Tests |
|------------|----------------|-------|
| `TestEq25LiftDragCoefficients` | Eq. 25 | 7 |
| `TestEq26AngleOfAttack` | Eq. 26, 28 | 2 |
| `TestEq27SideSlip` | Eq. 27, 28 | 4 |
| `TestEq29StructuralDrag` | Eq. 29, 31 | 6 |
| `TestEq30Eq31ForcDirections` | Eq. 30, 31 | 3 |
| `TestTableI` | Table I | 10 |

Module-level constants mirror paper symbols exactly (`N`, `L_w`, `L_b`, `R`,
`A_aspect`, `chord`, `S_w`, `r_cp`, `CL_alpha`, `omega_paper`, `v_tan_cp`, `q_cp`).
All 32 pass. Validation doc at `simulation/aero/deschutter.md`.

### Minimum tension_in for altitude maintenance at ξ=55°

At ξ=55°, the vertical thrust component from the rotor must exceed hub weight (5 kg × 9.81 = 49 N) plus tether downward component. Testing showed tension_in=20 N (matching unit test default) was insufficient — the TensionPI reduced collective to near col_min=−0.20 where vertical thrust ≈ 38 N < gravity → hub fell.

**Rule:** `tension_in ≥ 80 N` for altitude maintenance at ξ=55°. This matches `test_deschutter_cycle.py`'s `DEFAULT_TENSION_IN=80 N` which documents the same constraint in its source comment.

---

## Phase 3 M3 Step 1 — Dual GPS Steady Flight

### Why dual GPS (EK3_SRC1_YAW=2) replaced EKFGSF

Previous designs relied on EKFGSF yaw estimation (`EK3_SRC1_YAW=8`), which requires the hub to move at ≥0.5 m/s for the EKF Ground Speed Fusion to accumulate enough heading discriminating signal (~3–5 full circles). This forced an orbital kinematic trajectory (vel0=East, circular motion) purely for EKFGSF convergence.

Switching to dual-antenna GPS yaw (`EK3_SRC1_YAW=2`, RELPOSNED) eliminates all of this:
- Two F9P antennas 50 cm apart (±25 cm along body X) give yaw from the NED baseline vector.
- Yaw is known from the **first GPS fix** — no motion, no EKFGSF rotation needed.
- `delAngBiasLearned` converges with constant-zero gyro at ~21 s after arm.
- GPS fuses at ~34 s total from startup.
- A stationary hold (vel0=[0,0,0]) is the standard kinematic pattern.

Key parameters required (in `rawes_sitl_defaults.parm`):
- `GPS_AUTO_CONFIG=0` — **critical**: prevents ArduPilot reconfiguring UBLOX chips over serial, which corrupts the RELPOSNED stream in SITL.
- `COMPASS_USE=0`, `COMPASS_ENABLE=0` — SITL synthetic compasses cycle every 10 s, blocking GPS fusion.
- `EK3_GPS_CHECK=0` + `EK3_POS_I_GATE=50` + `EK3_VEL_I_GATE=50` — widened gates required for fusion.

### Three root causes fixed for test_lua_flight_steady (orbit-tracking era)

> **Note:** These fixes applied to the orbit-tracking implementation of rawes.lua (since replaced
> by AltitudeHoldController). They explain why the original orbit-tracking design had those
> constraints; the new design resolves them differently.

**1. Pre-GPS collective bypass** (`elseif _tdir0 == nil then col_cmd = col_cruise`):

Before GPS fuses, GPS altitude noise biases EKF vz ≈ 0.24 m/s. Without the bypass, the VZ controller commands col=-0.168 → T_aero=249 N > T_tether=199 N → 50 N net outward force → tether tension runaway → crash at kinematic exit. Bypass holds collective at `col_cruise` (-0.18 rad) until `_tdir0` fires.

**2. No gyro feedthrough in Lua cyclic** (`roll_rads = kp*err_bx`, pure P):

Gyro feedthrough (`kd * gyro_body`) is sensitive to EKF yaw error. Any constant EKF yaw offset `Δψ` maps through `R_ekf` and produces a spurious rate correction. With ~33% flakiness before removal. Orbital body_z precession ~0.01 rad/s; pure P + azimuthal orbit tracking handles it with negligible steady-state error.

EKF yaw cancellation in the error term: `err_body = R_ekf.T @ (bz_now × bz_orbit)`. Both `bz_now` and `bz_orbit` live in the same EKF frame, so a constant yaw offset `Δψ` cancels exactly. Stable (even if wrong) EKF yaw is sufficient for the P-gain error term.

**3. Pre-GPS orbit tracking** (`_bz_orbit` tracks `bz_now` every step before `_tdir0` fires):

Before GPS fuses, `_bz_orbit = bz_now` (err=0, pure neutral stick). When GPS fuses (yawAlignComplete via RELPOSNED + delAngBiasLearned), GPS tether recapture fires: `_bz_eq0` and `_bz_orbit` reset to `disk_normal_ned()` (zero step change) and `_tdir0 = normalize(diff)` anchors the orbit reference. Orbit tracking runs ~46 s before kinematic exit — hub has active cyclic control at the kinematic-to-physics transition.

### Results (orbit-tracking era)

test_lua_flight_steady: stable=86–110 s, 3/3 runs, max_activity≤1000 PWM, no EKF yaw reset.
Orbit quality: orbit_r < 5 m, altitude stable ±2 m, yaw gap < 15 deg for ≥ 60 s.

---

## Phase 3 M3 — AltitudeHoldController Rewrite of rawes.lua

### Why orbit-tracking was replaced

The orbit-tracking implementation (`rodrigues`, `slerp_step`, `orbit_track_azimuthal`, `_bz_orbit`/`_bz_slerp` state machine) was replaced with `bz_altitude_hold` (AltitudeHoldController). Key reasons:

1. **Stateless geometry**: `bz_altitude_hold(pos, el_rad, tension_n)` needs no captured reference state (`_bz_eq0`, `_tdir0`). GPS init simply sets `_el_rad` and `_target_alt` once; subsequent updates are rate-limited toward `asin(target_alt / tlen)`.
2. **Natural gravity compensation**: The function tilts the disk slightly inward (elevation-upward tangent direction) by `k = mass*g*cos(el)/tension` so the thrust vector counteracts the elevation-lowering component of gravity — no empirical tuning.
3. **Simpler pre-GPS path**: Gyro feedthrough (desired_rate = measured_rate → ACRO rate_error = 0 → zero corrective torque) preserves the natural orbital rate. No need to track `_bz_orbit = bz_now` to synthesize neutral-stick behavior.
4. **Pumping collective via TensionPI**: The open-loop collective schedule (`COL_REEL_OUT`, `COL_REEL_IN`) was replaced with a TensionPI running inside rawes.lua. TensionPI feedback on `RAWES_TEN` produces correct tension for each phase without requiring phase-specific collective tuning.

### New algorithm (rawes.lua post-rewrite)

```
Pre-GPS (_el_initialized == false):
  - Collective: hold at COL_CRUISE_FLIGHT_RAD = -0.18 rad
  - Cyclic: gyro feedthrough (ch1/ch2 = rate_to_pwm(gyro_body_x/y))
  - On first valid GPS fix with tlen >= 0.5 m:
      _el_rad = asin(-rz / tlen)
      _target_alt = -rz
      _el_initialized = true

Post-GPS:
  - Rate-limit _el_rad toward asin(_target_alt / tlen) at SCR_USER2 rad/s
  - bz_goal = bz_altitude_hold(rel, _el_rad, _tension_n)
  - err = cyclic_error_body(bz_now, bz_goal)
  - ch1/ch2 = rate_to_pwm(kp * err_bx/by)
  - Collective: VZ PI (steady) or TensionPI (pumping)
```

`RAWES_ALT` (NV float) updates `_target_alt` each tick; `RAWES_TEN` updates `_tension_n`.
Simtest sends constant `RAWES_ALT = IC_altitude` for all pumping phases (no disk tilt change;
energy differential comes entirely from TensionPI tension setpoint switching).

### Deleted / removed from rawes.lua

`rodrigues`, `orbit_track_azimuthal`, `slerp_step`, `v3_normalize`, `_bz_eq0`, `_tdir0`,
`_bz_orbit`, `_bz_slerp`, `_bz_target`, `_plan_ms`, `_pump_bz_ri`, `_captured`,
`XI_REEL_IN_DEG`, `T_TRANSITION`, `PLANNER_TIMEOUT`, `COL_REEL_IN`, `COL_CRUISE_LAND_RAD`,
`VZ_LAND_SP`, `test_yaw_rotation_invariance.py`.

### Simtest results (AltitudeHoldController era)

- `test_steady_flight_lua`: 3/3 tests pass (min_alt=15.6 m, orbit_r_max=3.2 m, no bad events)
- `test_pump_cycle_lua`: passes (total_net=560 J, 3 cycles each positive, no bad events)
